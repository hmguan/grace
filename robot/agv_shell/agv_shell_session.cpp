#include "agv_shell_session.h"
#include "agv_shell_common.h"
#include "agv_shell_define.h"
#include "agv_shell_server.h"
#include "agv_shell_proto.hpp"
#include "cmd_parse.h"
#include "const.h"
#include "file_manager.h"
#include "icom/hash.h"
#include "log.h"
#include "os_util.hpp"
#include "public.h"    //get_process_name
#include "session_task_manager.h"
#include "singleton.hpp"
#include "swnet.h"
#include "sys_info.h"
#include <algorithm>
#include <set>

#ifdef _WIN32
#include <io.h>
#else
#define _access access
#endif

enum AgvShellLogsCancel {
    LOGS_DEFAULT = 0,
    LOGS_CANCEL
};

enum ModifyFileMutex{
	UNLOCK_MUTEX,//解锁
	LOCK_MUTEX	//加锁
};

agv_shell_session::agv_shell_session() {
}

agv_shell_session::agv_shell_session(HTCPLINK lnk) : tcp_application_client(lnk) {
}

agv_shell_session::~agv_shell_session() {

}

void agv_shell_session::recv_dispatch(const unsigned char *buffer, int cb) {
	nsp::proto::proto_head agv_sh_head;
    int len = cb;
    if (!agv_sh_head.build(buffer, len)) {
		loerror("agv_shell") << "agv shell build package failed,then close " << remote_.to_string() << " session.";
        this->close();
        return;
    }
	
	uint32_t type = agv_sh_head.type_;
	loinfo("agv_shell")<<"recv_dispatch type:" << type;

	const unsigned char *pos = (const unsigned char  *)buffer; 
	switch (type) {
	case PKTTYPE_AGV_SHELL_KEEPALIVE:
		on_keepalive_ack(pos, cb);
		break;
	case PKTTYPE_AGV_SHELL_GET_FIXED_SYSINFO:
		on_get_fix_sysinfo(pos, cb);
		break;
	case PKTTYPE_AGV_SHELL_PROCESS_COMMAND:
		on_deal_process_cmd(nsp::proto::shared_for<nsp::proto::proto_command_process>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_FRIMWARE_INFO:
		on_frimware_info(nsp::proto::shared_for<nsp::proto::proto_frimware_info_request>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_FRIMWARE_UPDATE:
		on_frimware_update(nsp::proto::shared_for<nsp::proto::proto_frimware_update>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_FRIMWARE_DOWNLOAD:
		on_frimware_download(nsp::proto::shared_for<nsp::proto::proto_frimware_download_request>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_FRIMWARE_RESTART:
		on_frimware_restart(nsp::proto::shared_for<nsp::proto::proto_frimware_info_request>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_QUERY_VCU_ALIVE_STATUS:
		nsp::toolkit::singleton<agv_shell_server>::instance()->on_query_vcu_keep_alive_status(this->lnk_, pos, cb);
		break;
	case PKTTYPE_AGV_SHELL_SET_VCU_ALIVE_STATUS:
		on_set_vcu_keep_alive_status(nsp::proto::shared_for<nsp::proto::proto_set_keepalive_status>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_SET_PROCESS_LIST:
		on_update_cmd_list(nsp::proto::shared_for<nsp::proto::proto_process_list_reponse>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_GET_LOG_TYPE:
		on_post_log_type_list(pos, cb);
		break;
	case PKTTYPE_AGV_SHELL_GET_LOG_FILE_NAME:
		on_post_log_info(nsp::proto::shared_for<nsp::proto::proto_log_condition>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_FILE_MUTEX_STATUS:
		nsp::toolkit::singleton<agv_shell_server>::instance()->post_file_mutex(this->lnk_, pos, cb);
		break;
	case PKTTYPE_AGV_SHELL_READ_FILE_HEAD:
		recv_read_file_head(nsp::proto::shared_for<file::proto::proto_request_file_head_t>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_PUSH_FILE_HEAD:
		recv_push_file_head(nsp::proto::shared_for<file::proto::proto_file_head_info_t>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_READ_FILE_BLOCK:
		recv_read_file_block(nsp::proto::shared_for<file::proto::proto_request_file_data_t>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_PUSH_FILE_BLOCK:
		recv_push_file_block(nsp::proto::shared_for<file::proto::proto_file_data_t>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_RW_FILE_STATUS:
		recv_file_status(nsp::proto::shared_for<file::proto::proto_file_status_t>(pos, cb));
		break;
	case PKTTYPE_AGV_SHELL_BACKUP_FILES:
		on_backup_files(nsp::proto::shared_for<nsp::proto::proto_logs_file_path>(pos, cb));
		break;
	default:
		loinfo("agv_shell") << "recv_dispatch get package type：" << type << ", it's invaild type.";
		break;
	}
	
}

void agv_shell_session::on_recvdata(const std::string &pkt) {
    nsp::proto::proto_head agv_sh_head;
    int cb = pkt.size();
    if (!agv_sh_head.build((const unsigned char *) pkt.data(), cb)) {
		loerror("agv_shell") << "agv shell build package failed,then close " << remote_.to_string() << " session.";
        this->close();
        return;
    }

    int type = agv_sh_head.type_;
	if( !is_login() && type != PKTTYPE_AGV_SHELL_LOGIN) {
		loerror("agv_shell") << "pkt type:" << type << ", client " << remote_.to_string() << " havenot login, close connect.";
		this->close();
		return;
	}
	this->alive_count_ = 0;
	
    switch (type) {
		case PKTTYPE_AGV_SHELL_LOGIN:
			client_login(nsp::proto::shared_for<nsp::proto::proto_login_agv_t>(pkt.c_str(), pkt.length())); 
			break;
		case PKTTYPE_AGV_SHELL_UPGRADE:
			begin_upgrade(nsp::proto::shared_for<nsp::proto::proto_msg_str>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_GET_PROCESS_LIST:
			post_process_list_info(nsp::proto::shared_for<nsp::proto::proto_msg_int>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_UPDATE_NTP:
			on_update_ntp(nsp::proto::shared_for<nsp::proto::proto_msg>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_RUN_SCRIPT:
			on_run_script(nsp::proto::shared_for<nsp::proto::proto_msg>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_CANCEL_GET_LOG:
			on_cancel_get_file_task(nsp::proto::shared_for<nsp::proto::proto_msg_int>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_MODIFY_FILE_MUTEX:
			on_modify_file_mutex(nsp::proto::shared_for<nsp::proto::proto_msg_int_sync>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_CAN_CUSTOM_UPDATE:
			on_can_custom_update(nsp::proto::shared_for<nsp::proto::proto_frimware_custom_update>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_CAN_CUSTOM_INFO:
			on_can_custom_info(nsp::proto::shared_for<nsp::proto::proto_frimware_custom_update>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_BACKUP_DEMARCATE:
			on_backupdemarcate(nsp::proto::shared_for<nsp::proto::proto_common_stream>(pkt.c_str(), pkt.length()));
			break;
		case PKTTYPE_AGV_SHELL_SOFT_VERSION_INFO:
			post_shell_version(agv_sh_head.id_);
			break;
		//case PKTTYPE_AGV_SHELL_CONFIGFILE_VERSION:
		//	post_config_version(agv_sh_head.id_, nsp::toolkit::singleton<global_parameter>::instance()->get_config_version());
		//	break;
		//file transform
		case PKTTYPE_AGV_SHELL_KEEPALIVE:
			nsp::toolkit::singleton<session_keepalive_manager>::instance()->schedule_async_receive(
				pkt, std::bind(&agv_shell_session::recv_dispatch,
					std::dynamic_pointer_cast<agv_shell_session>(shared_from_this()),
					std::placeholders::_1, std::placeholders::_2)
			);
			break;
		case PKTTYPE_AGV_SHELL_READ_FILE_HEAD:
		case PKTTYPE_AGV_SHELL_PUSH_FILE_HEAD:
		case PKTTYPE_AGV_SHELL_READ_FILE_BLOCK:
		case PKTTYPE_AGV_SHELL_PUSH_FILE_BLOCK:
		case PKTTYPE_AGV_SHELL_RW_FILE_STATUS:
		//end file transform
		case PKTTYPE_AGV_SHELL_PROCESS_COMMAND:
		case PKTTYPE_AGV_SHELL_FRIMWARE_INFO:
		case PKTTYPE_AGV_SHELL_FRIMWARE_UPDATE:
		case PKTTYPE_AGV_SHELL_FRIMWARE_DOWNLOAD:
		case PKTTYPE_AGV_SHELL_FRIMWARE_RESTART:
		case PKTTYPE_AGV_SHELL_QUERY_VCU_ALIVE_STATUS:
		case PKTTYPE_AGV_SHELL_SET_VCU_ALIVE_STATUS:
		case PKTTYPE_AGV_SHELL_SET_PROCESS_LIST:
		case PKTTYPE_AGV_SHELL_GET_LOG_TYPE:
		case PKTTYPE_AGV_SHELL_GET_LOG_FILE_NAME:
		case PKTTYPE_AGV_SHELL_GET_FIXED_SYSINFO:
		case PKTTYPE_AGV_SHELL_FILE_MUTEX_STATUS:
		case PKTTYPE_AGV_SHELL_BACKUP_FILES:
			nsp::toolkit::singleton<session_task_manager>::instance()->schedule_async_receive(
				pkt, std::bind(&agv_shell_session::recv_dispatch,
					std::dynamic_pointer_cast<agv_shell_session>(shared_from_this()),
					std::placeholders::_1, std::placeholders::_2)
			);
			break;
        default:
			loinfo("agv_shell") << "agv shell get package type: " << type << ", this type is invaild type,so close the " << remote_.to_string() << " session.";
            //this->close();
            break;
    }
	
}

void agv_shell_session::on_connected(){
	loinfo("agv_shell") << "the target endpoint: "<< remote_.to_string() <<" is connected，the link is " << lnk_;
}

int agv_shell_session::on_established() {
    loinfo("agv_shell") << "agv_shell_session on_established,target endpoint is "<< remote_.to_string() <<" the link is " << lnk_ ;
	memset(key_, 0, PROTO_SESSION_KEY_LENGTH);
	// 生成属于这个链接的随机密钥 
	for (int i = 0; i < PROTO_SESSION_KEY_LENGTH; ++i) {
        key_[i] = rand() % 0xFE;
    }
	
	nsp::proto::proto_pre_login_agv_t pkt_pre_login(PKTTYPE_AGV_SHELL_PRE_LOGIN_ACK);
	pkt_pre_login.key_.assign((const char*)key_, PROTO_SESSION_KEY_LENGTH);
	pkt_pre_login.head_.size_ = pkt_pre_login.length();
	return psend(&pkt_pre_login);
}

void agv_shell_session::on_disconnected(const HTCPLINK previous) {
	nsp::toolkit::singleton<agv_shell_server>::instance()->reduce_client_lnk(previous);
	
	//关闭读写文件句柄
	nsp::toolkit::singleton<file_manager>::instance()->close_rw_file_handler(previous);
	loinfo("agv_shell") << "disconnect from " << remote_.ipv4() << ":" << remote_.port() << " the lnk is " << previous;
}

void agv_shell_session::client_login(const std::shared_ptr<nsp::proto::proto_login_agv_t>& p_login){
	nsp::proto::proto_head ack_login(PKTTYPE_AGV_SHELL_LOGIN_ACK);
    ack_login.size_ = ack_login.length();
	
	if (!p_login){
		loinfo("agv_shell") << "build login in package failure";
		ack_login.err_ = -EINVAL;
		psend(&ack_login);
		return;
	}
	ack_login.id_ = p_login->head_.id_;
	
	int retval = -1;
	unsigned char *out;
    int outcb;
	MD5_CTX md5ctx;
	unsigned char ori_digest[16], digest[16];
	
	do {
		if (p_login->original_buffer_.size() != PROTO_SESSION_KEY_LENGTH || 16 != p_login->encrypted_md5_.size()  ) {
			retval = -EINVAL;
			loinfo("agv_shell") << "buffer size:" << p_login->original_buffer_.size() <<", encrypt md5 size:" << p_login->encrypted_md5_.size();
			break;
		}
		
		//TODO: 角色校验 
		
		if (nsp::toolkit::encrypt((const unsigned char *)p_login->original_buffer_.data(), p_login->original_buffer_.size(), \
					(const unsigned char *)this->key_, sizeof ( this->key_), &out, &outcb) >= 0) {
			MD5__Init(&md5ctx);
			MD5__Update(&md5ctx, out, outcb);
			MD5__Final(&md5ctx, digest);
			memcpy(ori_digest, p_login->encrypted_md5_.data(), sizeof(ori_digest));
			loinfo("agv_shell") << "ori_digest:" << ori_digest <<" . digest:" << digest;
			if (0 != (retval = memcmp(digest, ori_digest, sizeof ( digest)))) {
				loinfo("agv_shell") << "type:" << p_login->acct_ <<" from " << remote_.ipv4() <<":" << remote_.port()<<" access decline.";
				retval = -EFAULT;
				break;
			}
			free(out);
			retval = 0;
			login = 1;
		}
	} while (0);
		
	// 应答
	ack_login.err_ = retval;
    psend(&ack_login);
}

int agv_shell_session::on_keepalive_ack(const unsigned char *buffer, int cb) {
	loinfo("agv_shell")<< "alive_count_:" << alive_count_;
	int ret = nsp::toolkit::singleton<agv_shell_server>::instance()->on_get_sysinfo_changed(this->lnk_, buffer, cb);
	return ret;
}

int agv_shell_session::on_get_fix_sysinfo(const unsigned char *buffer, int cb) {
	int ret = nsp::toolkit::singleton<agv_shell_server>::instance()->on_get_sysinfo_fixed(this->lnk_, buffer, cb);
	return ret;
}

void agv_shell_session::on_deal_process_cmd(const std::shared_ptr<nsp::proto::proto_command_process>& p_info){
	if (!p_info){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_deal_process_cmd(this->lnk_, p_info);
}

void agv_shell_session::on_frimware_info(const std::shared_ptr<nsp::proto::proto_frimware_info_request>& p_info){
	if (!p_info){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_get_frimware_info(this->lnk_, p_info->frimware_type_, p_info->target_endpoint_);
}

void agv_shell_session::on_frimware_update(const std::shared_ptr<nsp::proto::proto_frimware_update>& p_f){
	if (!p_f){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_frimware_update(this->lnk_, p_f->frimware_type_, p_f->frimware_path_, p_f->target_endpoint_);
}

void agv_shell_session::on_frimware_download(const std::shared_ptr<nsp::proto::proto_frimware_download_request>& p_f)
{
	if (!p_f){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_frimware_download(this->lnk_, p_f->frimware_type_, p_f->frimware_length_, p_f->target_endpoint_);
}

void agv_shell_session::on_frimware_restart(const std::shared_ptr<nsp::proto::proto_frimware_info_request>& p_f)
{
	if (!p_f){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_frimware_restart(this->lnk_, p_f->frimware_type_, p_f->target_endpoint_);
}

void agv_shell_session::on_set_vcu_keep_alive_status(const std::shared_ptr<nsp::proto::proto_set_keepalive_status>& p_f) 
{
	if (!p_f){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_set_vcu_keep_alive_status(this->lnk_, p_f->head_.id_, p_f->keepalive_status_);
}

int agv_shell_session::post_vcu_info(const FIRMWARE_SOFTWARE_TYPE f_type, const FRIMWARE_STATUS status, const VCU_MESSAGE msg_ty, const std::string& msg){
	nsp::proto::proto_frimware_info_response pkt(PKTTYPE_AGV_SHELL_FRIMWARE_INFO_ACK);
	pkt.vcu_type_ = msg_ty;
	pkt.frimware_msg_ = msg;
	pkt.frimware_status = status;
	pkt.frimware_type_ = f_type;
	return psend(&pkt);
}

int agv_shell_session::post_frimware_update(const FIRMWARE_SOFTWARE_TYPE f_type, const FRIMWARE_STATUS status, const int step){
	nsp::proto::proto_frimware_update_response pkt(PKTTYPE_AGV_SHELL_FRIMWARE_UPDATE_ACK);
	pkt.frimware_status = status;
	pkt.frimware_step = step;
	pkt.frimware_type_ = f_type;
	return psend(&pkt);
}

int agv_shell_session::post_frimware_download_ack(const std::string& file_path){
	nsp::proto::proto_frimware_download_reponse pkt(PKTTYPE_AGV_SHELL_FRIMWARE_DOWNLOAD_ACK);
	pkt.frimware_path_ = file_path;
	return psend(&pkt);
}

//upgrade from path files
void agv_shell_session::begin_upgrade(const std::shared_ptr<nsp::proto::proto_msg_str>& p_f) {
	nsp::proto::proto_head pkt(PKTTYPE_AGV_SHELL_UPGRADE_ACK);
	pkt.err_ = 0;
	pkt.size_ = pkt.length();
	if (!p_f){
		pkt.err_ = -EINVAL;
		psend(&pkt);
		return;
	}

	//解压缩动作可能花费时间长，所以解压缩后才放开升级(避免前台再次传升级文件过来)
	nsp::toolkit::singleton<file_manager>::instance()->set_upgrade();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
	pkt.id_ = p_f->head_.id_;
	int check_seccuss_flag = -1;
    int programe_size = p_f->msg_int_;
    std::string programe_name = get_process_name(p_f->msg_str_);
    std::string file_path = nsp::os::get_module_directory<char>() + "/standard/";
	std::string programe_path = file_path + programe_name;
    if(programe_path.rfind(".tar") == std::string::npos) {
		pkt.err_ = -EINVAL;
        loinfo("agv_shell") << programe_path << " cannot find .tar";
		psend(&pkt);
		nsp::toolkit::singleton<file_manager>::instance()->release_upgrade();
		return;
    }
    std::string untar_dest_patch = file_path + "patch/";
    std::string update_file_path = untar_dest_patch + UPDATE_FILE_NAME;
    loinfo("agv_shell") << "new file name:" << programe_path << ", size:" << programe_size << ", script path:"<< update_file_path;
#ifdef _WIN32
	// do nothing.
#else
    for (int i=0; i<3; ++i) {
        // 校验文件大小
        struct stat st;
        if (stat(programe_path.c_str(), &st) < 0) {
			pkt.err_ = -EACCES;
            loerror("agv_shell") << "failed to get file stat for size";
            perror(programe_path.c_str());
			psend(&pkt);
			nsp::toolkit::singleton<file_manager>::instance()->release_upgrade();
			return;
        }
        
        if (programe_size != st.st_size) {
            loinfo("agv_shell") << "programe_size:" << programe_size << ", file size:" << (uint64_t)st.st_size;
			std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
            check_seccuss_flag=1;
            break;
        }
    }
    if (check_seccuss_flag < 0) {
		pkt.err_ = -EINVAL;
        loerror("agv_shell") << "programe file size not equal server file size, upgrade stoped.";
		psend(&pkt);
		nsp::toolkit::singleton<file_manager>::instance()->release_upgrade();
		return;
    }
	
	std::string path_tar = "/bin/tar";
    nsp::os::rmdir_s<char>(untar_dest_patch.c_str()); // remove old directories and files
	nsp::os::mkdir_s<char>(untar_dest_patch.c_str()); // create new directory for unpack
	std::string process_param = "-C " + untar_dest_patch + " -xf " + programe_path;
	//unpack: /bin/tar -C /agvshell/standard/patch/ -xf patchX.Y.Z.tar 
	std::thread th_un_tar(run_process_by_popen, (void*)path_tar.c_str(), (void*)process_param.c_str());
	th_un_tar.join();
    
	int ret = -1;
	loinfo("agv_shell") << "graceup thread start ...";
	//try {
		// execute upgrade script(graceup.sh)
		//std::thread th_run_script(run_process_by_popen, (void*)update_file_path.c_str(), (void*)NULL);
		//th_run_script.detach();
		//ret = 0;
	//} catch (...) {
		//loerror("agv_shell") << "start graceup thread failure!!!";
		//ret = -1;
	//}

	nsp::toolkit::singleton<file_manager>::instance()->release_upgrade();
	pkt.err_ = ret;
	psend(&pkt);
	//close and restart
	loinfo("agv_shell") << "graceup thread start successful, agv_shell will exit...";
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	reboot_os();

#endif
}

int agv_shell_session::post_shell_version(uint32_t id)
{
	nsp::proto::proto_msg pkt(PKTTYPE_AGV_SHELL_SOFT_VERSION_INFO_ACK);
	pkt.head_.id_ = id;
	pkt.head_.err_ = 0;
	pkt.msg_ = nsp::toolkit::singleton<global_parameter>::instance()->get_shell_version();
	pkt.head_.size_ = pkt.length();
	return psend(&pkt);
}

int agv_shell_session::post_config_version(uint32_t id, const std::string& msg)
{
	nsp::proto::proto_msg pkt(PKTTYPE_AGV_SHELL_CONFIGFILE_VERSION_ACK);
	pkt.head_.id_ = id;
	pkt.head_.err_ = 0;
	pkt.msg_ = msg;
	pkt.head_.size_ = pkt.length();
	return psend(&pkt);
}

int agv_shell_session::post_process_list_info(const std::shared_ptr<nsp::proto::proto_msg_int>& data) {
	nsp::proto::proto_process_list_reponse pkt(PKTTYPE_AGV_SHELL_GET_PROCESS_LIST_ACK);
	pkt.head_.size_ = pkt.length();
	pkt.head_.err_ = 0;
	if (!data){
		pkt.head_.err_ = -EINVAL;
		return psend(&pkt);
	}
	
	pkt.head_.id_ = data->head_.id_;
	pkt.pkt_id_ = data->msg_int_;
	std::vector<agv_process_info > process_list;
	int ret = nsp::toolkit::singleton<global_parameter>::instance()->load_processes_fxml(process_list);
	if ( ret == 0 ) {
		auto ap = process_list.begin();
		int index = 0;
		while (ap != process_list.end()) {
			nsp::proto::proto_process pr;
			pr.process_id_ = index++;
			pr.process_name_ = ap->process_name_;
			pr.process_path_ = ap->name_;
			pr.process_cmd_ = ap->ori_cmd_;
			pr.process_delay_ = ap->delay_start_;
			pkt.process_list_.push_back(pr);
			++ap;
		}
	}
	
	pkt.head_.size_ = pkt.length();
	return psend(&pkt);
}

void agv_shell_session::on_update_ntp(const std::shared_ptr<nsp::proto::proto_msg>& data)
{
	nsp::proto::proto_msg pkt(PKTTYPE_AGV_SHELL_UPDATE_NTP_ACK);
	pkt.head_.err_ = 0;
	pkt.msg_ = data->msg_;
	pkt.head_.size_ = pkt.length();
	if (!data){
		pkt.head_.err_ = -EINVAL;
		psend(&pkt);
		return;
	}
	
	pkt.head_.id_ = data->head_.id_;
	if (data->msg_.empty()) {
		pkt.head_.err_ = -EINVAL;
		psend(&pkt);
		return;
	}

	pkt.head_.err_ = nsp::toolkit::singleton<global_parameter>::instance()->set_ntp_server(data->msg_);
	pkt.head_.size_ = pkt.length();
	psend(&pkt);
}

void agv_shell_session::on_run_script(const std::shared_ptr<nsp::proto::proto_msg>& data)
{
	nsp::proto::proto_head pkt(PKTTYPE_AGV_SHELL_RUN_SCRIPT_ACK);
	pkt.err_ = 0;
	pkt.size_ = pkt.length();
	if (!data){
		pkt.err_ = -EINVAL;
		psend(&pkt);
		return;
	}
	
	pkt.id_ = data->head_.id_;
	if( data->msg_.empty() ) {
		pkt.err_ = -EINVAL;
		psend(&pkt);
		return;
	}
	
	std::string process_name = data->msg_;
	std::string process_param("");
	std::size_t index = 0;
	if (std::basic_string<char>::npos != (index = process_name.find_first_of(" ", index))) {
		process_param.assign(&data->msg_[index]);
		process_name.assign(data->msg_.c_str(), index);
	}
	
	std::thread th_run_process(run_process_by_popen, (void*)process_name.c_str(), (void*)process_param.c_str());
	th_run_process.detach();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	psend(&pkt);
}

int agv_shell_session::on_update_cmd_list(const std::shared_ptr<nsp::proto::proto_process_list_reponse>& data)
{
	nsp::proto::proto_process_list_reponse_t pkt(PKTTYPE_AGV_SHELL_SET_PROCESS_LIST_ACK);
	pkt.head_.id_ = data->head_.id_;
	pkt.head_.err_ = 0;
	pkt.head_.size_ = pkt.length();
	if (!data){
		pkt.head_.err_ = -EINVAL;
		psend(&pkt);
		return -1;
	}
	std::vector<agv_process_info> vct_agv;
	for (const auto& iter : data->process_list_)
	{
		agv_process_info agv_info;
		agv_info.name_ = iter.process_path_;
		agv_info.ori_cmd_ = iter.process_cmd_;
		agv_info.delay_start_ = iter.process_delay_;
		vct_agv.push_back(agv_info);
	}
	int res = update_config_file(vct_agv);
	if (res == 0) pkt.process_list_ = data->process_list_;
	
	pkt.pkt_id_ = data->pkt_id_;
	pkt.head_.size_ = pkt.length();
	return psend(&pkt);
}

int agv_shell_session::on_post_log_type_list(const unsigned char *buffer, int cb) {
	if(!buffer) return -1;
	
	std::set<std::string> log_types;
	std::set<std::string> set_path_;
	std::string strmodule_path = nsp::os::get_module_directory<char>() + POSIX__DIR_SYMBOL_STR;

	//agvshell module log
	set_path_.emplace(strmodule_path + "log" POSIX__DIR_SYMBOL_STR + nsp::os::get_module_filename<char>() + POSIX__DIR_SYMBOL_STR);

	std::string strpath = "";
	for (auto ap = global_parameter::agv_process_.begin(); ap != global_parameter::agv_process_.end(); ap++){
		if ('.' == ap->name_.at(0)){
			strpath = strmodule_path;
		}
		else{
			strpath = "";
		}
#ifdef _WIN32
		set_path_.emplace(strpath + get_process_path(ap->name_) + "log" POSIX__DIR_SYMBOL_STR + ap->process_name_ + ".exe" POSIX__DIR_SYMBOL_STR);
#else
		set_path_.emplace(strpath + get_process_path(ap->name_) + "log" POSIX__DIR_SYMBOL_STR + ap->process_name_ + POSIX__DIR_SYMBOL_STR);
#endif
	}

	for (auto iter : set_path_ ){
		nsp::toolkit::singleton<global_parameter>::instance()->run_progess_by_get_logtype(iter, log_types);
	}
   
	nsp::proto::proto_log_type_vct pkt(PKTTYPE_AGV_SHELL_GET_LOG_TYPE_ACK);
	pkt.head_.id_ = *(uint32_t *)buffer;
	pkt.head_.err_ = 0;
	nsp::proto::proto_log_type pr;
	for (auto&iter : log_types){
		pr.log_type_ = iter;
		pkt.vct_log_type_.push_back(pr);
	}
	pkt.head_.size_ = pkt.length();
	return psend(&pkt);
}

int agv_shell_session::on_post_log_info(const std::shared_ptr<nsp::proto::proto_log_condition>& data)
{
	int ret_value = 0;
	nsp::proto::proto_logs_file_path_t pkt(PKTTYPE_AGV_SHELL_GET_LOG_FILE_NAME_ACK);
	pkt.head_.size_ = pkt.length();
	pkt.head_.err_ = 0;
	if (!data){
		pkt.head_.err_ = -EINVAL;
		return psend(&pkt);
	}
	pkt.head_.id_ = data->head_.id_;
	pkt.task_id = data->task_id;
	std::string start_time = data->start_time.empty() ? "16000000_000000" : data->start_time;//empty means ignore start_time;
	std::string end_time = data->end_time.empty() ? "99999999_999999" : data->end_time;//empty means ignore end_time
	std::string module_path = nsp::os::get_module_directory<char>();
	std::set<std::string> set_path;
	set_path.emplace(module_path + "/log/");
	std::vector<std::string> log_files;
	std::string compressed_file = module_path + "/standard/";
	char tar_file[64] = { 0 };
	posix__sprintf(tar_file, sizeof(tar_file), "log_" UINT64_STRFMT ".tar", nsp::os::clock_gettime());
	compressed_file += tar_file;

	set_get_file_task(data->task_id, compressed_file);
	// add syslog & messages
	auto iter = std::find_if(data->vct_log_type_.begin(), data->vct_log_type_.end(), [&](const nsp::proto::proto_log_type& type)->int {
		return type.log_type_ == "system_log" ? 1 : 0;
	});
	if (iter != data->vct_log_type_.end()) {
		if (_access("/var/log/syslog", R_OK) == 0){
			log_files.push_back("/var/log/syslog");
		} else {
			loinfo("agv_shell") << "can not access file:/var/log/syslog errno:" << errno;
		}
		if (_access("/var/log/messages", R_OK) == 0){
			log_files.push_back("/var/log/messages");
		} else {
			loinfo("agv_shell") << "can not access file:/var/log/messages errno:" << errno;
		}
	}
	std::string strpath = "";
	for (auto & iter : global_parameter::agv_process_) {
		if ('.' == iter.name_.at(0)){
			strpath = module_path + "/";
		}
		else{
			strpath = "";
		}

		set_path.emplace(strpath + get_process_path(iter.name_) + "log/");
	}
	// add other type log files
	for (const auto &iter : set_path) {
		ret_value = search_file(iter, start_time, end_time, data->task_id, [&](const std::string &path, const std::string &type)->void {
			for (const auto & vec_iter : data->vct_log_type_){
				if (vec_iter.log_type_ == type){
					log_files.push_back(path);
					loinfo("agv_shell") << "push file name:" << path;
					break;
				}
			}
		});
		if (ret_value) {
			pkt.head_.err_ = ret_value;
			break;
		}
	}

	if (exist_get_file_task(data->task_id)) {
		//compressd file, wait for client pull 
	#ifdef _WIN32
		for(auto& item : log_files) {
			pkt.vct_log_file_name_.push_back(item);
		}
	#else
		if(log_files.size() > 0) {
			ret_value = compress_files(log_files, compressed_file);
			if (ret_value) {
				pkt.head_.err_ = ret_value;
			} else {
				pkt.vct_log_file_name_.push_back(compressed_file);
			}
		}
	#endif
	} else {
		pkt.head_.err_ = -ECANCELED;;
	}
	release_get_file_task(data->task_id, compressed_file);
	pkt.head_.size_ = pkt.length();
	return psend(&pkt);
}

int agv_shell_session::search_file(
	std::string strPath, std::string start_time, std::string end_time, int task_id, \
	std::function<void(const std::string &, const std::string &)> call_back_push_path)
{
	int ret_value = 0;
	std::string type;
	std::string file_full_path;
	std::string strfullpath;
	struct tm tm_time;
	char file_modify_time[32];
	
	if (strPath.find_last_of('/') != strPath.length()-1 ){
		strPath += "/";
	}

#ifdef _WIN32
	_finddata_t filedata;
	long Handle = -1;

	strfullpath = strPath + "*.*";
	if (-1 == (Handle = _findfirst(strfullpath.c_str(), &filedata))){
		return 0;
	}

	do{
		if (!exist_get_file_task(task_id)) {
			ret_value = -ECANCELED;
			break;
		}
		std::string file_name = filedata.name;
		if (!file_name.compare(".") || !file_name.compare("..")){
			continue;
		}
		std::string file_full_path = strPath + file_name;
		if (filedata.attrib & _A_SUBDIR){
			ret_value = search_file(file_full_path + "/", start_time, end_time, task_id, call_back_push_path);
			if (ret_value) {
				break;
			}
			continue;
		}
		localtime_s(&tm_time, &filedata.time_write);//时间转换
		sprintf(file_modify_time, "%04d%02d%02d_%02d%02d%02d", tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
		if ( nsp::toolkit::singleton<global_parameter>::instance()->check_file(file_name, file_modify_time, start_time, end_time) != 0){
			continue;
		}

		type = nsp::toolkit::singleton<global_parameter>::instance()->get_type(file_name);
		call_back_push_path(file_full_path,type);
	} while (0 == _findnext(Handle, &filedata));
	_findclose(Handle);
#else
	struct dirent *ent = nullptr;
	DIR *dirptr = nullptr;
	struct stat st;

	if (!(dirptr = opendir(strPath.c_str()))){
		loinfo("agv_shell") << "can not open directory:" << strPath << " errno:" << errno;
		return 0;
	}
	while ( (ent = readdir(dirptr)) ) {
		if (! exist_get_file_task(task_id)) {
			ret_value = -ECANCELED;
			break;
		}

		std::string file_name = ent->d_name;
		if (!file_name.compare(".") || !file_name.compare("..")){
			continue;
		}
		std::string file_full_path = strPath + file_name;
		stat(file_full_path.c_str(), &st);
		if (S_ISDIR(st.st_mode)){
			ret_value = search_file(file_full_path + "/", start_time, end_time, task_id, call_back_push_path);
			if (ret_value) {
				break;
			}
			continue;
		}

		localtime_r(&st.st_mtime, &tm_time);
		sprintf(file_modify_time, "%04d%02d%02d_%02d%02d%02d", tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
		if ( nsp::toolkit::singleton<global_parameter>::instance()->check_file(file_name, file_modify_time, start_time, end_time) != 0){
			continue;
		}

		type = nsp::toolkit::singleton<global_parameter>::instance()->get_type(file_name);
		call_back_push_path(file_full_path,type);
	}
	closedir(dirptr);
#endif
	return ret_value;
}

int agv_shell_session::on_modify_file_mutex(const std::shared_ptr<nsp::proto::proto_msg_int_sync>& data)
{
	nsp::proto::proto_msg_int_sync pkt(PKTTYPE_AGV_SHELL_MODIFY_FILE_MUTEX_ACK);
	pkt.head_.err_ = 0;
	pkt.head_.size_ = pkt.length();
	if (!data){
		pkt.head_.err_ = -EINVAL;
		return psend(&pkt);
	}
	int pkt_id = data->pkt_id;
	int res = nsp::toolkit::singleton<global_parameter>::instance()->modify_file_lock(data->msg_int);

	pkt.head_.err_ = res;
	pkt.head_.id_ = data->head_.id_;
	pkt.pkt_id = pkt_id;
	pkt.msg_int = data->msg_int;
	pkt.head_.size_ = pkt.length();
	return psend(&pkt);
}

void agv_shell_session::on_can_custom_update(const std::shared_ptr<nsp::proto::proto_frimware_custom_update>& p_f)
{
	if (!p_f){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_frimware_update(this->lnk_, CAN_CUSTOM_SOFTWARE, 
		p_f->frimware_path_, p_f->target_endpoint_, p_f->node_id, p_f->serial_type);
}

void agv_shell_session::on_can_custom_info(const std::shared_ptr<nsp::proto::proto_frimware_custom_update>& data)
{
	if (!data){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_get_frimware_info(this->lnk_,
		FIRMWARE_SOFTWARE_TYPE::CAN_CUSTOM_SOFTWARE, data->target_endpoint_, data->node_id, data->serial_type);
}

int agv_shell_session::on_backup_files(const std::shared_ptr<nsp::proto::proto_logs_file_path>& data)
{
	nsp::proto::proto_logs_file_path_t pkt(PKTTYPE_AGV_SHELL_BACKUP_FILES_ACK);
	if (!data){
		return -1;
	}
#ifndef _WIN32
	pkt.head_.id_ = data->head_.id_;
	pkt.task_id = data->task_id;
	char tar_file[64] = { 0 };
	posix__sprintf(tar_file, sizeof(tar_file), "backup_" UINT64_STRFMT ".tar", nsp::os::clock_gettime());
	std::string compressed_file = nsp::os::get_module_directory<char>() + "/standard/";
	compressed_file += tar_file;

	set_get_file_task(data->task_id, compressed_file);
	//打压缩包 
	std::vector<std::string> backup_files;
	for (const auto iter : data->vct_log_file_name_)
	{
		if( std::string::npos != iter.find("/gzrbot/") || 
			std::string::npos != iter.find("/agvshell/") ||
			std::string::npos != iter.find("/etc/agv/")) {
			backup_files.push_back(iter);
		}
	}
	//压缩成一个文件  client 主动拉取 
	if(backup_files.size() > 0) {
		compress_files(backup_files, compressed_file);
		pkt.vct_log_file_name_.push_back(compressed_file);
	}
	
	return psend(&pkt);
	
#else

#endif
	return 0;
}

void agv_shell_session::on_backupdemarcate(const std::shared_ptr<nsp::proto::proto_common_stream>& data)
{
	nsp::proto::proto_msg_int_sync pkt(PKTTYPE_AGV_SHELL_BACKUP_DEMARCATE_ACK);
	pkt.head_.err_ = 0;
	pkt.head_.size_ = pkt.length();
	if (!data) {
		pkt.head_.err_ = -EINVAL;
		pkt.msg_int = -EINVAL;
		psend(&pkt);
		loerror("agv_shell") << "failed to build backup demarcate file.";//未能解析备份标定文件数据包
		return;
	}
	pkt.head_.id_ = data->head_.id_;
	pkt.pkt_id = data->pkt_id;
	std::string tmp = convert_positive(data->common_stream, '\\', '/');
	std::string des_path = nsp::os::get_module_directory<char>() + "/backup/";
	nsp::os::mkdir_s<char>(des_path.c_str());
	std::string src_path = "";
	if (tmp.substr(0, 2) == "./")
	{
		//替换相对路径
		src_path = nsp::os::get_module_directory<char>() + "/standard/" + tmp.substr(2);
	}
	int res = run_copye_file_by_popen(src_path, des_path);

	//回包
	pkt.msg_int = res;
	pkt.head_.size_ = pkt.length();
	psend(&pkt);
}


//file transform interface
void agv_shell_session::recv_read_file_head(const std::shared_ptr<file::proto::proto_request_file_head_t>& data)
{
	file::proto::proto_file_head_info_t pkt(PKTTYPE_AGV_SHELL_READ_FILE_HEAD_ACK);
	if (!data) {
		pkt.head_.size_ = pkt.length();
		pkt.head_.err_ = -EINVAL;
		psend(&pkt);
		loerror("agv_shell") << "failed to build recv_read_file_head proto.";
		return;
	}
	
	int error_code = 0;
	file::st_file_head f_head;
	f_head.file_path = data->file_path_;
	f_head.file_id = data->file_id_;
	nsp::toolkit::singleton<file_manager>::instance()->read_file_info(this->lnk_, f_head, error_code);

	pkt.file_id_ = f_head.file_id;
	pkt.file_name_ = f_head.file_name;
	pkt.total_size_ = f_head.total_size;
	pkt.file_create_time_ = f_head.file_create_time;
	pkt.file_modify_time_ = f_head.file_modify_time;
	pkt.file_access_time_ = f_head.file_access_time;
	pkt.head_.id_ = data->head_.id_;
	pkt.head_.err_ = error_code;
	pkt.head_.size_ = pkt.length();
	psend(&pkt);
}

void agv_shell_session::recv_push_file_head(const std::shared_ptr<file::proto::proto_file_head_info_t>& data)
{
	file::proto::proto_file_status_t pkt(PKTTYPE_AGV_SHELL_PUSH_FILE_HEAD_ACK);
	if (!data) {
		pkt.head_.size_ = pkt.length();
		pkt.head_.err_ = -EINVAL;
		psend(&pkt);
		loerror("agv_shell") << "failed to build recv_push_file_head proto.";
		return;
	}
	
	int error_code = 0;
	file::st_file_head f_head;
	f_head.file_path = data->file_name_;
	f_head.file_id = data->file_id_;
	f_head.file_type = data->file_type_;
	f_head.total_size = data->total_size_;
	f_head.file_create_time = data->file_create_time_;
	f_head.file_modify_time = data->file_modify_time_;
	f_head.file_access_time = data->file_access_time_;
	
	nsp::toolkit::singleton<file_manager>::instance()->create_files(this->lnk_, f_head, error_code);

	pkt.file_id_ = f_head.file_id;
	pkt.error_code_ = error_code;
	pkt.head_.id_ = data->head_.id_;
	pkt.head_.err_ = error_code;
	pkt.head_.size_ = pkt.length();
	psend(&pkt);
}

void agv_shell_session::recv_read_file_block(const std::shared_ptr<file::proto::proto_request_file_data_t>& data)
{
	if (!data) {
		file::proto::proto_file_data_t pkt(PKTTYPE_AGV_SHELL_READ_FILE_BLOCK_ACK);
		pkt.head_.size_ = pkt.length();
		pkt.head_.err_ = -EINVAL;
		psend(&pkt);
		loerror("agv_shell") << "failed to build recv_read_file_block proto.";
		return;
	}
	
	int ret = nsp::toolkit::singleton<file_manager>::instance()->read_file_block_stream(this->lnk_, data->head_.id_, 
			data->file_id_, data->block_num_, data->file_offset_, data->file_length_);
	if(ret < 0) {
		//TODO: read_file_block_stream return failure
		loinfo("agv_shell")<< "task doesnot exist to read file data";
	}
	
}

void agv_shell_session::recv_push_file_block(const std::shared_ptr<file::proto::proto_file_data_t>& data)
{
	if (!data) {
		nsp::proto::proto_head pkt(PKTTYPE_AGV_SHELL_PUSH_FILE_BLOCK_ACK);
		pkt.size_ = pkt.length();
		pkt.err_ = -EINVAL;
		psend(&pkt);
		loerror("agv_shell") << "failed to build recv_push_file_block proto.";
		return;
	}
	
	int ret = nsp::toolkit::singleton<file_manager>::instance()->write_file_block_stream(this->lnk_, data->head_.id_, 
		data->file_id_, data->block_num_, data->file_offset_, data->file_data_);
	if(ret < 0) {
		//TODO: write_file_block_stream return failure
		loinfo("agv_shell")<< "task doesnot exist to write file data";
	}
}

void agv_shell_session::recv_file_status(const std::shared_ptr<file::proto::proto_file_status_t>& data) {
	nsp::proto::proto_head pkt(PKTTYPE_AGV_SHELL_RW_FILE_STATUS_ACK);
	pkt.size_ = pkt.length();
	if (!data) {
		pkt.err_ = -EINVAL;
		psend(&pkt);
		loerror("agv_shell") << "failed to build recv_file_status proto.";
		return;
	}
	
	pkt.id_ = data->head_.id_;
	if(data->head_.err_ != 0 || data->error_code_ != 0) {
		loerror("agv_shell") << "file transform error, file id:" << data->file_id_ << ", current block num:" << data->block_num_ << ", err code:" << data->error_code_;
		nsp::toolkit::singleton<file_manager>::instance()->file_complete(this->lnk_, data->file_id_);
		pkt.err_ = 0;
		psend(&pkt);
		return;
	}
	
	loinfo("agv_shell")<< "file transform finished, file id:" << data->file_id_;
	int ret = nsp::toolkit::singleton<file_manager>::instance()->file_complete(this->lnk_, data->file_id_);
	loinfo("agv_shell")<< "file_complete return:" << ret;
	pkt.err_ = ret;
	psend(&pkt);
}

void agv_shell_session::set_get_file_task(unsigned int task_id, std::string &file_name) {
	std::lock_guard<decltype(get_file_task_locker_)> guard(get_file_task_locker_);

	auto iter = get_file_task_map_.find(task_id);
	if (iter == get_file_task_map_.end()) {
		get_file_task_map_[task_id] = file_name;
		loinfo("agv_shell") << "get file task_id:" << task_id << " file:" << file_name;
	}
}

int agv_shell_session::exist_get_file_task(unsigned int task_id) {
	std::lock_guard<decltype(get_file_task_locker_)> guard(get_file_task_locker_);

	auto iter = get_file_task_map_.find(task_id);
	if (iter != get_file_task_map_.end()) {
		return 1;
	}
	return 0;
}

int agv_shell_session::release_get_file_task(unsigned int task_id, std::string &file_name) {
	std::lock_guard<decltype(get_file_task_locker_)> guard(get_file_task_locker_);

	auto iter = get_file_task_map_.find(task_id);
	if (iter != get_file_task_map_.end()) {
		file_name = iter->second;
		get_file_task_map_.erase(iter);
	} else {
		return -ENOENT;
	}
	return 0;
}

int agv_shell_session::on_cancel_get_file_task(const std::shared_ptr<nsp::proto::proto_msg_int>& data) {
	nsp::proto::proto_msg_int pkt(data->head_.type_ | PKTTYPE_ACK);
	if (!data){
		pkt.head_.err_ = -EINVAL;
		return psend(&pkt);
	}

	unsigned int task_id = 0;
	std::string file_name;
	pkt.head_.id_ = data->head_.id_;
	pkt.msg_int_ = data->msg_int_;
	pkt.head_.size_ = pkt.length();
	// get tar filename & delete task_id from task_map
	task_id = (unsigned int)data->msg_int_;
	pkt.head_.err_ = release_get_file_task(task_id, file_name);

	loinfo("agv_shell") << "cancel file task_id:" << task_id << " file:" << file_name;
	if(!file_name.empty()){
		// kill processes tar/xz
		std::string cmd_str = "ps -ef | grep ";
		cmd_str += file_name;
		cmd_str += " | grep -v grep | awk '{print $2}' | xargs kill -9 ";
		system(cmd_str.c_str());
		// delete xxx.tar file
		nsp::os::rmfile(file_name);
		file_name += ".xz";
		//delete xxx.tar.xz file
		nsp::os::rmfile(file_name);
	}

	return psend(&pkt);
}

