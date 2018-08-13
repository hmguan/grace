#include "agv_shell_session.h"
#include "agv_shell.h"
#include "agv_shell_common.h"
#include "agv_shell_define.h"
#include "agv_shell_server.h"
#include "agv_shell_proto.hpp"
#include "cmd_parse.h"
#include "keepalive.h"
#include "log.h"
#include "singleton.hpp"
#include "swnet.h"
#include "os_util.hpp"
#include "set"
#include "sys_info.h"
#include <algorithm>

#ifdef _WIN32
#include <io.h>
#endif

#define UPDATE_FILE_NAME "graceup.sh"

enum AgvShellLogsCancel {
    LOGS_DEFAULT = 0,
    LOGS_CANCEL
};

enum ModifyFileMutex{
	UNLOCK_MUTEX,//解锁
	LOCK_MUTEX	//加锁
};

agv_shell_session::agv_shell_session() : logs_cancel_flag_(LOGS_DEFAULT) {
}

agv_shell_session::agv_shell_session(HTCPLINK lnk) : tcp_application_client(lnk), logs_cancel_flag_(LOGS_DEFAULT) {
}

agv_shell_session::~agv_shell_session() {

}

void agv_shell_session::on_recvdata(const std::string &pkt) {
    nsp::proto::agv_shell_proto_head agv_sh_head;
    int cb = pkt.size();
    if (!agv_sh_head.build((const unsigned char *) pkt.data(), cb)) {
		loerror("agv_shell") << "agv shell build package failed,then close " << remote_.to_string() << " session.";
        this->close();
        return;
    }

    if (agv_sh_head.op != 0xC || agv_sh_head.fn != 'C' || agv_sh_head.ln != 'K') {
        loerror("agv_shell") << "invalid protocol head specify.";
        this->close();
        return;
    }

    int type = agv_sh_head.type;
    switch (type) {
		case kAgvShellProto_Login:
			on_get_login(nsp::proto::shared_for<nsp::proto::proto_login_agv>(pkt.c_str(), pkt.length()));
			break;
        case kAgvShellProto_HeartBeat:
            this->send(pkt);
            break;
        case kAgvShellProto_Cmd:
            break;
        case kAgvShellProto_StartProcess:
        case kAgvShellProto_StopProcess:
        case kAgvShellProto_RestartProcess:
        case kAgvShellProto_RestartOS:
		case kAgvShellProto_ProcessCmd:
			on_deal_process_cmd(nsp::proto::shared_for<nsp::proto::proto_command_process>(pkt.c_str(), pkt.length()));
            break;
        case kAgvShellProto_Status:
            break;
		case kAgvShellProto_Hostname:
			on_get_hostname();
			break;
		case kAgvShellProto_Frimware_Info:
			on_frimware_info(nsp::proto::shared_for<nsp::proto::proto_frimware_info_request>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_Frimware_Update:
			on_frimware_update(nsp::proto::shared_for<nsp::proto::proto_frimware_update>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_Frimware_Download:
			on_frimware_download(nsp::proto::shared_for<nsp::proto::proto_frimware_download_request>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_Frimware_Restart:
			on_frimware_restart(nsp::proto::shared_for<nsp::proto::proto_frimware_info_request>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_Upgrade:
			begin_upgrade(nsp::proto::shared_for<nsp::proto::proto_msg_str>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_Query_Keepalive_Status:
			nsp::toolkit::singleton<agv_shell_server>::instance()->on_query_vcu_keep_alive_status(this->lnk_);
			break;
		case kAgvShellProto_Set_Keepalive_Status:
			on_set_vcu_keep_alive_status(nsp::proto::shared_for<nsp::proto::proto_set_keepalive_status>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_Process_List:
			post_process_list_info(nsp::proto::shared_for<nsp::proto::proto_msg_int>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_UpdateNTP:
			on_update_ntp(nsp::proto::shared_for<nsp::proto::proto_msg>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_RunScript:
			on_run_script(nsp::proto::shared_for<nsp::proto::proto_msg>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_CmdList:
			on_cmd_list(nsp::proto::shared_for<nsp::proto::proto_msg_int>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_UpdateCmdList:
			on_update_cmd_list(nsp::proto::shared_for<nsp::proto::proto_process_list_reponse>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_GetLogType:
			on_post_log_type_list();
			break;
		case kAgvShellProto_TimeTypeConditon:
			on_post_log_info(nsp::proto::shared_for<nsp::proto::proto_log_condition>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_Logs_Cancel:
			this->logs_cancel_flag_ = LOGS_CANCEL;
			break;
		case kAgvShellProto_ModifyFileMutex:
			on_modify_file_mutex(nsp::proto::shared_for<nsp::proto::proto_msg_int_sync>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_Custom_Update:
			on_can_custom_update(nsp::proto::shared_for<nsp::proto::proto_frimware_custom_update>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_KeepAlive_ACk:
			alive_count_ = 0;
			break;
		case kAgvShellProto_Frimware_Custom_Info:
			on_can_custom_info(nsp::proto::shared_for<nsp::proto::proto_frimware_custom_update>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_BackupFiles:
			on_backup_files(nsp::proto::shared_for<nsp::proto::proto_logs_file_path>(pkt.c_str(), pkt.length()));
			break;
		case kAgvShellProto_BackupDemarcate:
			on_backupdemarcate(nsp::proto::shared_for<nsp::proto::proto_common_stream>(pkt.c_str(), pkt.length()));
			break;
        default:
			loinfo("agv_shell") << "agv shell get package type：" << type << ", this type is invaild type,so close the " << remote_.to_string() << " session.";
            this->close();
            break;
    }
	
}

void agv_shell_session::on_connected(){
	loinfo("agv_shell") << "the target endpoint: "<< remote_.to_string() <<" is connected，the link is " << lnk_;
}

int agv_shell_session::on_established() {
    loinfo("agv_shell") << "agv_shell_session on_established,target endpoint is "<< remote_.to_string() <<" the link is " << lnk_ ;
    return 0;
}

void agv_shell_session::on_disconnected(const HTCPLINK previous) {
	nsp::toolkit::singleton<agv_shell_server>::instance()->reduce_client_lnk(previous);
	loinfo("agv_shell") << "disconnect from " << remote_.ipv4() << ":" << remote_.port() << " the lnk is " << previous;
}

uint32_t agv_shell_session::get_link(){
	return this->lnk_;
}

void agv_shell_session::on_get_login(const std::shared_ptr<nsp::proto::proto_login_agv>& p_f){
	if (!p_f){
		return;
	}
	if (p_f->login_identify == AgvClientType_CMP)
	{
		//判断当前针对配置平台的链接是否已经存在，如果已经存在，则本次链接无效，断开链接
		//查看当前连接上来的客户端数目 
		int count = nsp::toolkit::singleton<agv_shell_server>::instance()->get_client_number();
		loinfo("agv_shell") << "agv_shell has " << count << " client session currentlly.";
		if (count >= 1){
			post_login_ack();
			lowarn("agv_shell") << "agv_shell has already client session exists.";
			return;
		}
		//获取本机主机名
		if (on_get_hostname() >= 0)
		{
			nsp::toolkit::singleton<agv_shell_server>::instance()->add_client_lnk(this->lnk_);
		}
	}
}

void agv_shell_session::post_login_ack()
{
	nsp::proto::proto_msg_int pkt(kAgvShellProto_Login_Ack);
	pkt.msg_int_ = nsp::proto::errorno_t::kItemExist;
	psend(&pkt);
}

int agv_shell_session::on_get_hostname(){
	//推送版本号
	post_shell_version(nsp::toolkit::singleton<global_parameter>::instance()->get_shell_version());
	post_config_version(nsp::toolkit::singleton<global_parameter>::instance()->get_config_version());
	//推送主机名
	//从版本2.0.10之后推送MAC地址替代主机名
	char host_name[64] = { 0 };
	nsp::toolkit::singleton<nsp::tcpip::swnet>::instance()->nis_lgethost(host_name, 64);
	std::string host_str = host_name;
	if (host_str.length() == 0){
		host_str = "N/A";
	}
	sys_info t_sys_info;
	nsp::proto::proto_msg pkt(kAgvShellProto_Hostname_Ack);
	pkt.msg_ = t_sys_info.get_mac_addr();
	return psend(&pkt);
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
void agv_shell_session::on_query_vcu_keep_alive_status()
{
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_query_vcu_keep_alive_status(this->lnk_);
}
void agv_shell_session::on_set_vcu_keep_alive_status(const std::shared_ptr<nsp::proto::proto_set_keepalive_status>& p_f) 
{
	if (!p_f){
		return;
	}
	nsp::toolkit::singleton<agv_shell_server>::instance()->on_set_vcu_keep_alive_status(this->lnk_, p_f->keepalive_status_);
}

int agv_shell_session::post_vcu_info(const FIRMWARE_SOFTWARE_TYPE f_type, const FRIMWARE_STATUS status, const VCU_MESSAGE msg_ty, const std::string& msg){
	nsp::proto::proto_frimware_info_response pkt(kAgvShellProto_Frimware_Info_Ack);
	pkt.vcu_type_ = msg_ty;
	pkt.frimware_msg_ = msg;
	pkt.frimware_status = status;
	pkt.frimware_type_ = f_type;
	return psend(&pkt);
}

int agv_shell_session::post_frimware_update(const FIRMWARE_SOFTWARE_TYPE f_type, const FRIMWARE_STATUS status, const int step){
	nsp::proto::proto_frimware_update_response pkt(kAgvShellProto_Frimware_Update_Ack);
	pkt.frimware_status = status;
	pkt.frimware_step = step;
	pkt.frimware_type_ = f_type;
	return psend(&pkt);
}

int agv_shell_session::post_frimware_download_ack(const std::string& file_path){
	nsp::proto::proto_frimware_download_reponse pkt(kAgvShellProto_Frimware_Download_Ack);
	pkt.frimware_path_ = file_path;
	return psend(&pkt);
}
//全量更新
void agv_shell_session::begin_upgrade(const std::shared_ptr<nsp::proto::proto_msg_str>& p_f) {
	if (!p_f){
		return;
	}

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
	int check_seccuss_flag = -1;
    int programe_size = p_f->msg_int_;
    std::string programe_name = CK_GetProcessName(p_f->msg_str_);
    std::string file_path = nsp::os::get_module_directory<char>() + "/standard/";
	std::string programe_path = file_path + programe_name;
    if(programe_path.rfind(".tar") == std::string::npos) {
        loinfo("agv_shell") << programe_path << " cannot find .tar";
        return;
    }
    std::string untar_dest_patch = file_path + "patch/";
    std::string update_file_path = untar_dest_patch + UPDATE_FILE_NAME;
    loinfo("agv_shell") << "new file name:" << programe_path << ", size:" << programe_size << ", script path:"<< update_file_path;
#ifdef _WIN32
	/*HANDLE hFile = CreateFileA(programe_path.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_NORMAL, NULL);
	if (INVALID_HANDLE_VALUE == hFile) {
		loerror("agv_shell") << "can not open file : "<< file_path<<" in local,then can not read file total size or file time.The error code is "<< (int)GetLastError();
		return;
	}
	LARGE_INTEGER size;
	if (!GetFileSizeEx(hFile, &size)) {
		loerror("agv_shell") << "can not get file " << file_path << " size.The error code is "<< (int)GetLastError();
		CloseHandle(hFile);
		return;
	}
	uint64_t total_size_ = size.QuadPart;

	if(total_size_ != programe_size)
	{
		loerror("agv_shell") << "programe file size not equal server file size, upgrade stoped.";
		return;
	}

	fts_destory_receiver("0.0.0.0", nsp::toolkit::singleton<global_parameter>::instance()->get_fts_port());
	nsp::toolkit::singleton<agv_shell_server>::instance()->close_all();

	kill_agv_processes();
	//启动新的进程
	std::string cmd_line = " -u " + programe_path + " -P " + file_path;
	if (start_process(programe_path, cmd_line) < 0)
	{
		loerror("agv_shell") << "failed to create new agv_shell process,the errno:" << errno;
		return;
	}
	else
	{
		loinfo("agv_shell") << "start new agv_shell process successfully.";
		exit(0);
	}*/

#else
    for (int i=0; i<3; ++i) {
        // 校验文件大小
        struct stat st;
        if (stat(programe_path.c_str(), &st) < 0) {
            loerror("agv_shell") << "failed to get file stat for size";
            perror(programe_path.c_str());
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
        loerror("agv_shell") << "programe file size not equal server file size, upgrade stoped.";
        return;
    }
	
    //unpack /usr/bin/tar /bin/tar -xvf ***.tar 
	std::string path_tar = "/bin/tar";
	std::string path_bash = "/bin/bash";
	nsp::toolkit::singleton<global_parameter>::instance()->bash_command("/usr/bin/which tar",path_tar,16);
	//nsp::toolkit::singleton<global_parameter>::instance()->bash_command("/usr/bin/which bash",path_bash,16);
	loinfo("agv_shell") << "tar file path:" << path_tar;
    nsp::os::mkdir_s<char>(untar_dest_patch.c_str());
    std::string process_param = "-C " + untar_dest_patch + " -xvf " + programe_path;
	std::thread th_un_tar(run_process_by_popen, (void*)path_tar.c_str(), (void*)process_param.c_str());
	th_un_tar.join();
    
    //kill processes
	kill_agv_processes();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //wait all processes stoped
    
    //execute script
	int ret = -1;
	try {
		//start_process_normal(path_bash, update_file_path.c_str(), 0);
		std::thread th_run_script(run_process_by_popen, (void*)update_file_path.c_str(), (void*)untar_dest_patch.c_str());
		th_run_script.detach();
		ret = 0;
	} catch (...) {
		loerror("agv_shell") << "start graceup thread failure!!!";
		ret = -1;
	}
    
	if(ret >= 0) {
		loinfo("agv_shell") << "graceup thread start successful, agv_shell will exit...";
		fts_destory_receiver("0.0.0.0", nsp::toolkit::singleton<global_parameter>::instance()->get_fts_port());
		nsp::toolkit::singleton<agv_shell_server>::instance()->close_all();
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		exit(0);
	}
    
#endif
}

int agv_shell_session::post_keepalive_status_ack(const int proto_type, const int status){
	nsp::proto::proto_keepalive_status_reponse pkt(proto_type);
	pkt.status_ = status;
	return psend(&pkt);
}

int agv_shell_session::post_process_list_info(const std::shared_ptr<nsp::proto::proto_msg_int>& data) {
	if (!data){
		return -1;
	}
	nsp::proto::proto_process_list_reponse pkt(kAgvShellProto_Process_List_Ack);
	pkt.pkt_id_ = data->msg_int_;
	auto ap = global_parameter::agv_process_.begin();
	int index = 0;
	while( ap != global_parameter::agv_process_.end() ) {
		nsp::proto::proto_process pr;
		pr.process_id_ = index++;
		pr.process_name_ = ap->process_name_;
		pkt.process_list_.push_back(pr);
		ap++;
	}
	return psend(&pkt);
}

int agv_shell_session::post_file_status(const int status)
{
	nsp::proto::proto_keepalive_status_reponse pkt(kAgvShellProto_FileMutexStatus);
	pkt.status_ = status;
	return psend(&pkt);
}

int agv_shell_session::post_shell_version(const std::string &msg)
{
	nsp::proto::proto_msg pkt(kAgvShellProto_ShellVersionInfo);
	pkt.msg_ = msg;
	return psend(&pkt);
}

int agv_shell_session::on_cmd_list(const std::shared_ptr<nsp::proto::proto_msg_int>& data)
{
	if (!data){
		return -1;
	}
	nsp::proto::proto_process_list_reponse pkt(kAgvShellProto_CmdList_ACK);
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
			ap++;
		}
	}
	
	return psend(&pkt);
}

void agv_shell_session::on_update_ntp(const std::shared_ptr<nsp::proto::proto_msg>& data)
{
	if (!data){
		return;
	}
	if( data->msg_.empty() ) return;
	
	auto ap = global_parameter::vct_auto_run_list_.begin();
	while( ap != global_parameter::vct_auto_run_list_.end() ) {
		std::thread th_run_process(run_process_by_popen, (void*)(*ap).c_str(), (void*)data->msg_.c_str());
		th_run_process.detach();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		++ap;
	}
	
}

void agv_shell_session::on_run_script(const std::shared_ptr<nsp::proto::proto_msg>& data)
{
	if (!data){
		return;
	}
	if( data->msg_.empty() ) return;
	
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
}

int agv_shell_session::on_update_cmd_list(const std::shared_ptr<nsp::proto::proto_process_list_reponse>& data)
{
	if (!data){
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
	nsp::proto::proto_common_stream pkt;
	pkt.head_.type = kAgvShellProto_UpdateCmdList_ACK;
	pkt.pkt_id = data->pkt_id_;
	if (res == 0)pkt.common_stream = "0";
	else pkt.common_stream = "-1";
	return psend(&pkt);
}

int agv_shell_session::on_post_log_type_list()
{
	std::set<std::string> log_types;
	std::set<std::string> set_path_;
	std::string strmodule_path = nsp::os::get_module_directory<char>() + POSIX__DIR_SYMBOL_STR;

#ifdef _WIN32
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

		set_path_.emplace(strpath + CK_GetProcessRelatePath(ap->name_) + "log" POSIX__DIR_SYMBOL_STR + ap->process_name_ + ".exe" POSIX__DIR_SYMBOL_STR);
	}
#else
	set_path_.emplace(strmodule_path + "log" POSIX__DIR_SYMBOL_STR);
	set_path_.emplace("/gzrobot/log/");
#endif
	for (auto iter : set_path_ ){
		nsp::toolkit::singleton<global_parameter>::instance()->run_progess_by_get_logtype(iter, log_types);
	}
   
	nsp::proto::proto_log_type_vct pkt(kAgvShellProto_GetLogType_ACK);
	nsp::proto::proto_log_type pr;
	for (auto&iter : log_types){
		pr.log_type_ = iter;
		pkt.vct_log_type_.push_back(pr);
	}
	return psend(&pkt);
}

int agv_shell_session::on_post_log_info(const std::shared_ptr<nsp::proto::proto_log_condition>& data)
{
	if (!data){
		return -1;
	}
	std::string start_time = data->start_time.empty() ? "16000000_000000" : data->start_time;//empty means ignore start_time;
	std::string end_time = data->end_time.empty() ? "99999999_999999" : data->end_time;//empty means ignore end_time
	nsp::proto::proto_logs_file_path_t pkt(kAgvShellProto_TimeTypeConditon_ACK);
	std::string module_path = nsp::os::get_module_directory<char>() + POSIX__DIR_SYMBOL_STR;
	std::set<std::string> set_path;
	set_path.emplace(module_path + "/log/");

#ifdef _WIN32
	std::string strpath = "";
	for (auto & iter : global_parameter::agv_process_){
		if ('.' == iter.name_.at(0)){
			strpath = module_path;
		}
		else{
			strpath = "";
		}

		set_path.emplace(strpath + CK_GetProcessRelatePath(iter.name_) + "log/");
	}
#else
	set_path.emplace("/gzrobot/log/");
#endif
	for (const auto &iter : set_path){
		search_file(iter, start_time, end_time, [&](const std::string &path,const std::string &type)->void{
			for (const auto & vec_iter : data->vct_log_type_){
				if (vec_iter.log_type_ == type){
					pkt.vct_log_file_name_.push_back(path);
					loinfo("agv_shell") << "push file name:" << path;
					break;
				}
			}
		});
	}
	//添加系统日志
	auto iter = std::find_if(data->vct_log_type_.begin(), data->vct_log_type_.end(), [&](const nsp::proto::proto_log_type& type)->bool{
		return type.log_type_ == "system_log" ? true : false;
	});
	if (iter != data->vct_log_type_.end()){
		pkt.vct_log_file_name_.push_back("/var/log/syslog");
		pkt.vct_log_file_name_.push_back("/var/log/messages");
	}

	return psend(&pkt);
}

void agv_shell_session::search_file(std::string strPath, std::string start_time, std::string end_time, std::function<void(const std::string &, const std::string &)> call_back_push_path){
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
		return;
	}

	do{
		if (LOGS_CANCEL == this->logs_cancel_flag_) {
			break;
		}

		std::string file_name = filedata.name;
		if (!file_name.compare(".") || !file_name.compare("..")){
			continue;
		}
		std::string file_full_path = strPath + file_name;
		if (filedata.attrib & _A_SUBDIR){
			search_file(file_full_path + "/", start_time, end_time, call_back_push_path);
			continue;
		}
		localtime_s(&tm_time, &filedata.time_write);//时间转换
		sprintf(file_modify_time, "%04d%02d%02d_%02d%02d%02d", tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
		if ( !nsp::toolkit::singleton<global_parameter>::instance()->check_file(file_name, file_modify_time, start_time, end_time)){
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
		loinfo("agv_shell") << " open dir error";
		return;
	}
	while ((ent = readdir(dirptr))) {
		if (LOGS_CANCEL == this->logs_cancel_flag_) {
			break;
		}

		std::string file_name = ent->d_name;
		if (!file_name.compare(".") || !file_name.compare("..")){
			continue;
		}
		std::string file_full_path = strPath + file_name;
		stat(file_full_path.c_str(), &st);
		if (S_ISDIR(st.st_mode)){
			search_file(file_full_path + "/", start_time, end_time, call_back_push_path);
			continue;
		}

		localtime_r(&st.st_mtime, &tm_time);
		sprintf(file_modify_time, "%04d%02d%02d_%02d%02d%02d", tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
		if (!nsp::toolkit::singleton<global_parameter>::instance()->check_file(file_name, file_modify_time, start_time, end_time)){
			continue;
		}

		type = nsp::toolkit::singleton<global_parameter>::instance()->get_type(file_name);
		call_back_push_path(file_full_path,type);
	}
	closedir(dirptr);
#endif
}

int agv_shell_session::on_modify_file_mutex(const std::shared_ptr<nsp::proto::proto_msg_int_sync>& data)
{
	if (!data){
		return -1;
	}
	int pkt_id = data->pkt_id;
	bool resul_t = data->msg_int == LOCK_MUTEX ? true : false;
	bool res = nsp::toolkit::singleton<global_parameter>::instance()->modify_file_lock(resul_t);

	nsp::proto::proto_msg_int_sync pkt(kAgvShellProto_ModifyFileMutex_ACK);
	pkt.pkt_id = pkt_id;
	pkt.msg_int = res ? 0 : -1;
	
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
	if (!data){
		return -1;
	}
#ifndef _WIN32
	//打压缩包 tar -cvzf path file file
	std::string path_tar = "/bin/tar";
	nsp::toolkit::singleton<global_parameter>::instance()->bash_command("/usr/bin/which tar",path_tar,16);
	
	loinfo("agv_shell") << "tar file path:" << path_tar;
	std::string file_path = nsp::os::get_module_directory<char>() + "/backup/";
	nsp::os::mkdir_s<char>(file_path.c_str());
	std::string cmd = path_tar;
	std::string des_path = file_path + "backups.tar.gz";
	std::string src_path;
	for (const auto iter : data->vct_log_file_name_)
	{
		src_path += iter + " ";
	}
	std::thread th_backup_tar(run_tar_by_popen, cmd, "-czf", des_path, src_path, (uint32_t)this->lnk_);
	th_backup_tar.join();
#else

#endif
	return 0;
}

int agv_shell_session::post_tar_backups(const std::string& msg)
{
	nsp::proto::proto_msg pkt(kAgvShellProto_BackupFiles_ACK);
	pkt.msg_ = msg;
	return psend(&pkt);
}

void agv_shell_session::on_backupdemarcate(const std::shared_ptr<nsp::proto::proto_common_stream>& data)
{
	if (!data)
	{
		loerror("agv_shell") << "failed to build backup demarcate file.";//未能解析备份标定文件数据包
		return;
	}
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
	nsp::proto::proto_msg_int_sync pkt(kAgvShellProto_BackupDemarcate_ACK);
	pkt.pkt_id = data->pkt_id;
	pkt.msg_int = res;

	psend(&pkt);
}

int agv_shell_session::post_config_version(const std::string& msg)
{
	nsp::proto::proto_msg pkt(kAgvShellProto_ConfigVersion);
	pkt.msg_ = msg;
	return psend(&pkt);
}