#include "agv_shell_common.h"
#include "agv_shell_server.h"
#include "getopt.h"
#include "log.h"
#include "os_util.hpp"
#include "public.h"  //split_symbol_string
#include "proto_agv_sysinfo.h"  //proto_process_run_info_t
#include "rapidxml.hpp"
#include "rapidxml_iterators.hpp"
#include "rapidxml_print.hpp"
#include "rapidxml_utils.hpp"
#include "toolkit.h"
#include <stdio.h>
#include <iostream>
#include <memory>

#ifdef _WIN32
#include <io.h>
#endif


std::vector<agv_process_info > global_parameter::agv_process_;
std::vector<std::string> global_parameter::vct_auto_run_list_;

//update <agv_shell > node configure
int update_config_file( std::vector<agv_process_info >& replace_cfg ) {
	std::string file_path = nsp::os::get_module_directory<char>();
    std::string dock_path = file_path + CONFIGE_FILE_NAME;
	std::string temp_file = dock_path + "_tmp";
	
	FILE *fp, *tmp_fp;
	char* str_line = (char*)malloc(MAX_LINE);
	if(( fp = fopen(dock_path.c_str(), "r") ) == NULL) {
		loerror("agv_shell") << "fopen file " << dock_path << " failure, errno:" << errno;
		return -1;
	}
	if(( tmp_fp = fopen(temp_file.c_str(),"a+") ) == NULL) {
		loerror("agv_shell") << "fopen file " << temp_file << " failure, errno:" << errno;
		fclose(fp);
		return -1;
	}
	
	int replace_flag = 0;
	while ( !feof(fp) ) {
		memset(str_line, 0, MAX_LINE);
		str_line = fgets(str_line, MAX_LINE, fp);
		if(!str_line) break;
		if (replace_flag != 0) ++replace_flag;
		if (strstr(str_line, "agv_shell")  && strstr(str_line, "port")) {
			++replace_flag;
		} else if (strstr(str_line, "/agv_shell")) {
			if (replace_flag > 0) {
				std::string new_line;
				size_t v_size = replace_cfg.size();
				for(size_t i = 0; i < v_size; ++i) {
					std::string delay = std::to_string(replace_cfg[i].delay_start_);
					new_line = "\t<process name=\"" + replace_cfg[i].name_ + "\" cmd=\"" + replace_cfg[i].ori_cmd_ + "\"" + "delay_start=\"" + delay + "\"/>\n";
					fputs(new_line.c_str(), tmp_fp);
				}
			}
			replace_flag = 0;
		}
		if (replace_flag < 2) {
			fputs(str_line, tmp_fp);
		}
	}
	fclose(fp);
	fclose(tmp_fp);
	remove(dock_path.c_str());
	rename(temp_file.c_str(), dock_path.c_str());

	return 0;
}

//param1: full name; 
//param2: 0/NULL non-block, otherwise block; 
//param3: execute param to param1; 
//param4: int data for return pid 
static
void start_process_normal_fun(void* param1, void* param2, void* param3, void* param4) {
    if( !param1 || !param4 ) {
		return;
	}
	char* p_process_name = (char*)param1;
	
	int block_flag = non_block_thread;
	if( param2 ) { block_flag = *(int*)param2;}
	
	std::string run_param = "";
	if( param3 ) { run_param = (char*)param3;}
	
	loinfo("agv_shell") << "start:" << p_process_name << ", param:" << run_param << ", block:" << block_flag;
	
#ifdef _WIN32
	
#else
    //add file execute access
	if ( chmod(p_process_name, S_IRWXU | S_IRWXG ) < 0 ) {
		loerror("agv_shell") << "file chmod failed, errno:" << errno;
		*(int*)param4 = -1;
		return;
	}
	//Only support 8 params, 0: process name, last one must be NULL 
	char *argv[10] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
	argv[0] = p_process_name;
	size_t agrv_pos = 1;
	
	std::vector<std::string> v_param;
	if( 0 == nsp::toolkit::slicing_symbol_string(run_param, ' ', v_param)  && !run_param.empty()) {
        argv[agrv_pos] = (char*)run_param.c_str();
    } else {
        for (size_t i = 0; i < v_param.size() && i < 8; ++i) {
            if (v_param[i].empty()) {
                continue;
            }
            
            argv[agrv_pos] = (char*)v_param[i].c_str();
            ++agrv_pos;
        }
    }
	
    pid_t pid = fork();
    switch (pid) {
	case -1:
		loerror("agv_shell") << "fork failed";
		*(int*)param4 = -1;
		return ;
		break;
	case 0:
		//child start new process by execv 
		loinfo("agv_shell") << "child " << p_process_name << " " << run_param << "  begin...";
		printf("child %s %s begin...\n", p_process_name, run_param.c_str());
		if (-1 == execv(p_process_name, argv)) {
			loerror("agv_shell") << "execv failure, errno:" << errno;
			printf("execv failure, errno:%d\n", errno);
			exit(errno);
		}
		break;
	default:
		//这是在父进程中，等待子进程结束 
		loinfo("agv_shell") << "wait " << p_process_name << " stop, pid=" << pid;
		int status=0;
		if(non_block_thread == block_flag) { waitpid(pid, &status, WNOHANG); }
		else { waitpid(pid, &status, 0); }
		
		if( WIFEXITED(status) ) {
			WEXITSTATUS(status);
			loinfo("agv_shell") << "process " << p_process_name << " stopped, status=" << status;
		}
		if( WIFSIGNALED(status) ) {
			WTERMSIG(status);
			loinfo("agv_shell") << "process " << p_process_name << " stopped, WTERMSIG status=" << status;
		}
		*(int*)param4 = pid;
		loinfo("agv_shell") << "process " << p_process_name << " stopped, status=" << status;
		break;
    }
#endif
}

int start_process_normal(const char* name, const char* param, int flag) {
	int ret = -1;
	std::thread th_hd(start_process_normal_fun, (void*)name, (void*)&flag, (void*)param, (void*)&ret);
	th_hd.detach();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	return ret;
}

static
void start_process_functional(void* p) {
    if( !p ) return;
	agv_process_info* p_pinfo = (agv_process_info*)p;
	
	loinfo("agv_shell") << "start :" << p_pinfo->name_;
#ifdef _WIN32
	p_pinfo->name_ = convert_positive(p_pinfo->name_, '\\', '/');
	size_t pos = p_pinfo->name_.find_last_of('/');

	std::string work_path = p_pinfo->name_.substr(0, pos);
    if (p_pinfo->hdl_ != NULL) {
        CloseHandle(p_pinfo->hdl_);
        p_pinfo->hdl_ = NULL;
    }
    PROCESS_INFORMATION pi;
    STARTUPINFO si; //用于指定新进程的主窗口特性的一个结构  
    memset(&si, 0, sizeof (si));
    si.cb = sizeof (STARTUPINFO);
    si.dwFlags = STARTF_USESHOWWINDOW;
    si.wShowWindow = SW_SHOW; //SW_HIDE隐藏窗口  

	BOOL ret = CreateProcessA((char*)p_pinfo->process_name_.c_str(), (char*)p_pinfo->cmd_.c_str(), NULL, NULL, FALSE, /*CREATE_NEW_CONSOLE*/0, NULL, work_path.size() == 0 ? NULL : work_path.c_str() , &si, &pi);
    if (ret) {
        //关闭子进程的主线程句柄  
        CloseHandle(pi.hThread);
        p_pinfo->pid_ = pi.dwProcessId;
        p_pinfo->hdl_ = pi.hProcess;

        loinfo("agv_shell") << "start_process_functional " << p_pinfo->name_ << " pid=" << p_pinfo->pid_ << " hdl=" << p_pinfo->hdl_;
        return ;
    }
	else
	{
		loerror("agv_shell") << "failed to start " << p_pinfo->name_<< " ,the error code is "<< (int)GetLastError();
	}
	
#else
    //add file execute access
	if ( chmod(p_pinfo->name_.c_str(), S_IRWXU | S_IRWXG ) < 0 ) {
		loerror("agv_shell") << "file chmod failed, errno:" << errno;
		return;
	}
	//Only support 8 params, 0: process name, last one must be NULL 
	char *argv[10] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
	argv[0] = (char*)p_pinfo->process_name_.c_str();
	std::size_t agrv_pos = 1;
	
	std::vector<std::string> v_param;
	if(!p_pinfo->cmd_.empty()){
		if( 0 == nsp::toolkit::slicing_symbol_string(p_pinfo->cmd_, ' ', v_param)){
			std::string cmd = p_pinfo->cmd_;
			nsp::toolkit::trim<char>(cmd);
			if( cmd.length() > 0 ){
				argv[agrv_pos] = (char *)cmd.c_str();
			}
		}
		else{
			for (int i = 0; i < (int) v_param.size() && i < 8; ++i) {
				if (v_param[i].empty()) {
					continue;
				}

				argv[agrv_pos++] = (char*)v_param[i].c_str();
			}
		}
	}

    pid_t pid = fork();
    switch (pid) {
	case -1:
		loerror("agv_shell") << "fork failed";
		return ;
		break;
	case 0:
		//这是在子进程中，调用execlp切换为ps进程   
		loinfo("agv_shell") << "child " << p_pinfo->name_ << " " << p_pinfo->cmd_ << "  begin...";
		if (-1 == execv(p_pinfo->name_.c_str(), argv)) {
			loerror("agv_shell") << "execv failure, errno:" << errno;
			exit(errno);
		}
		break;
	default:
		//这是在父进程中，等待子进程结束 
		p_pinfo->pid_ = pid;
		loinfo("agv_shell") << "wait " << p_pinfo->process_name_ << " stop, pid=" << p_pinfo->pid_;
		int status;
		waitpid(pid, &status, 0);
		if( WIFEXITED(status) ) {
			WEXITSTATUS(status);
			loinfo("agv_shell") << "process " << p_pinfo->process_name_ << " stopped, status=" << status;
		}
		if( WIFSIGNALED(status) ) {
			WTERMSIG(status);
			loinfo("agv_shell") << "process " << p_pinfo->process_name_ << " stopped, WTERMSIG status=" << status;
		}
		p_pinfo->pid_ = 0;
		loinfo("agv_shell") << "process " << p_pinfo->process_name_ << " stopped, status=" << status;
		break;
    }

#endif
	
}

int start_process(agv_process_info& pf) {
	nsp::os::waitable_handle time_wait;
    pf.pid_ = 0;
    pf.hdl_ = 0;
    loinfo("agv_shell") << "delay:" << pf.delay_start_ << " start " << pf.name_;
    time_wait.wait(pf.delay_start_);
	if (pf.th_hd_ && pf.th_hd_->joinable()) {
		pf.th_hd_->join();
		delete pf.th_hd_;
		pf.th_hd_ = nullptr;
	}
	
	std::thread *th_hd = new std::thread(start_process_functional, (void *) &pf );
	if ( th_hd ) {
		pf.th_hd_ = th_hd;
	}
	
	return 0;
}

int kill_process(agv_process_info& pf) {
#ifdef _WIN32 
    HANDLE handLe = OpenProcess(PROCESS_TERMINATE, FALSE, pf.pid_);
    if (pf.hdl_ == NULL) {
        return 0;
    }
    BOOL bResult = TerminateProcess(pf.hdl_, 0);

    //等待子进程的退出  
    WaitForSingleObject(pf.hdl_, INFINITE);

    //关闭子进程句柄  
    CloseHandle(pf.hdl_);
	pf.hdl_ = NULL;
    loinfo("agv_shell") << "kill_process " << pf.name_ << " pid=" << pf.pid_;
	pf.hdl_ = 0;
#else
    if (pf.pid_ > 0) {
        loinfo("agv_shell") << "begin kill_process " << pf.name_ << " pid=" << pf.pid_;
        kill(pf.pid_, SIGKILL);
        int status = 0;
        waitpid(pf.pid_, &status, 0);
        loinfo("agv_shell") << "killed " << pf.name_ << " pid=" << pf.pid_;
        pf.pid_ = 0;
		if (pf.th_hd_ && pf.th_hd_->joinable()) {
            pf.th_hd_->join();
            delete pf.th_hd_;
            pf.th_hd_ = nullptr;
        }
    }
#endif
    return 0;
}

int start_agv_processes() {
    auto iter = global_parameter::agv_process_.begin();
	for (; iter != global_parameter::agv_process_.end(); ++iter) {
		iter->cmd_ = iter->ori_cmd_;
		
#ifndef _WIN32
		// 已经是启动状态的进程不做处理 
		if ( iter->pid_ > 0 && 0 == kill(iter->pid_, 0) ) {
			lotrace("agv_shell")<< "the process " << iter->process_name_ << " is already started.";
			continue;
		}
#else
		if (iter->hdl_ != NULL)
		{
			loinfo("agv_shell") << "the process " << iter->process_name_ << " is already started.";
			continue;
		}
#endif
		start_process( *iter );
    }

    return 0;
}

int kill_agv_processes() {
    for (auto ap = global_parameter::agv_process_.begin();
            ap != global_parameter::agv_process_.end(); ++ap) {
        kill_process( *ap );
    }
    return 0;
}

int restart_agv_processes() {
    kill_agv_processes();
    start_agv_processes();
    return 0;
}

int reboot_agv_syn() {
    kill_agv_processes();
    reboot_os();
    return 0;
}

int shutdown_agv_syn() {
    kill_agv_processes();
    shutdown_os();
    return 0;
}

int get_process_config_info(nsp::proto::proto_vector_t<agv::proto::proto_process_info_t>& vec) {
	agv::proto::proto_process_info_t pr;
	int index = 0;
	for (auto &ap : global_parameter::agv_process_) {
		pr.process_id_ = index++;
		pr.process_name_ = ap.process_name_;
		pr.process_path_ = ap.name_;
		pr.process_cmd_ = ap.ori_cmd_;
		pr.process_delay_ = ap.delay_start_;
		vec.push_back(pr);
	}
	
	return vec.size();
}

int get_process_run_info(nsp::proto::proto_vector_t<agv::proto::proto_process_run_info_t>& process_vec) {
	agv::proto::proto_process_run_info_t t_prot_proc;
	std::string command("ps -A -opid -oetime -osize -orss -o%cpu -o%mem -oargs | grep -v ps | grep -v grep | grep -eagv_shell ");
	std::string process_info_str;
	std::vector<std::string> v_str;
	char * pchar = nullptr;
	char shell_name[32];
	
	for (auto &iter : global_parameter::agv_process_) {
#ifndef _WIN32
		if (iter.pid_ <= 0 || 0 != kill(iter.pid_, 0)) {
			iter.pid_ = 0;
		}
#else
		if (iter.hdl_){
			loinfo("agv_shell") << "the process:" << iter.process_name_ << " and handler:" << iter.hdl_;
			int result = WaitForSingleObjectEx(iter.hdl_, 0, TRUE);
			if (result != WAIT_TIMEOUT){
				CloseHandle(iter.hdl_);						//进程不存在
				iter.hdl_ = NULL;
				iter.pid_ = 0;
			}
		}
#endif
		t_prot_proc.name = iter.process_name_;
		t_prot_proc.pid = iter.pid_;
		process_vec.push_back(t_prot_proc);
		
		command += (" -e" + iter.process_name_);
	}
	//add shell self
	posix__getpename2(shell_name, 32);
	t_prot_proc.name = shell_name;
	t_prot_proc.pid = posix__getpid();
	process_vec.push_back(t_prot_proc);
	//loinfo("agv_shell") << "get running process info cmd:" << command;
#ifndef _WIN32
	char * buffer = nullptr;
	try{
		buffer= new char[1024];
		memset(buffer,0,sizeof(char)*(1024));
	}
	catch(...){
		return -1;
	}

	FILE *p = nullptr;
	p = popen(command.c_str(), "r");
	if (!p) {
		loerror("agv_shell") << "popen failure, cmd:" << command;
		delete [] buffer;
		return -1;
	} else {
		while( (buffer = fgets(buffer, 1024, p)) ) {
			if((pchar = strchr(buffer,'\n'))!= nullptr){
				*pchar = '\0';
			}
			process_info_str = buffer;
			v_str.clear();
			split_symbol_string(process_info_str, ' ', v_str);
			if(v_str.size() <= 0 || v_str[0].empty()) continue;
			for(auto &it : process_vec) {
				if(it.pid == atol(v_str[0].c_str())) {
					it.run_time = v_str[1];
					it.vir_mm = atol(v_str[2].c_str());
					it.rss = atol(v_str[3].c_str());
					it.average_cpu = atol(v_str[4].c_str());
					it.average_mem = atol(v_str[5].c_str());
					break;
				}
			}
		}
	}
	if(buffer) delete [] buffer;
	pclose(p);
	
#endif
	return 0;
}

global_parameter::global_parameter() {
	;
}
global_parameter::~global_parameter() {
	auto iter = global_parameter::agv_process_.begin();
	for (; iter != global_parameter::agv_process_.end(); ++iter) {
		if (iter->th_hd_ && iter->th_hd_->joinable()) {
            iter->th_hd_->join();
            delete iter->th_hd_;
            iter->th_hd_ = nullptr;
        }
	}
}

int global_parameter::load_processes_fxml(std::vector<agv_process_info >& process_list) {
    std::string file_path = nsp::os::get_module_directory<char>();
    std::string dock_path = file_path + CONFIGE_FILE_NAME;
    rapidxml::xml_document<> doc;
    rapidxml::file<char> *file = nullptr;
	process_list.clear();
    loinfo("agv_shell") << "loadXml:" << dock_path;
    try {
        file = new rapidxml::file<char>(dock_path.c_str());
        doc.parse<0>(file->data());
        rapidxml::xml_node<>* root = doc.first_node("agv_shell");
        if (root) {
            for (rapidxml::xml_node<>* pr = root->first_node(); pr; pr = pr->next_sibling()) {
                agv_process_info ap;
                ap.pid_ = 0;
                ap.hdl_ = NULL;
				ap.th_hd_ = NULL;
                rapidxml::xml_attribute<>* lp = pr->first_attribute("name");
                if (lp) {
                    ap.name_ = lp->value();
                    ap.process_name_ = get_process_name(ap.name_);
                }

                rapidxml::xml_attribute<>* dp = pr->first_attribute("cmd");
                if (dp) {
                    ap.cmd_ = dp->value();
                    ap.ori_cmd_ = ap.cmd_;
                }

                rapidxml::xml_attribute<>* ds = pr->first_attribute("delay_start");
                if (ds) {
                    ap.delay_start_ = atoi(ds->value());
                } else {
                    ap.delay_start_ = 0;
                }
				process_list.push_back( ap );
            }
        }
        delete file;
    }
    catch (...) 
	{
		if (file){
			delete file;
		}
        loinfo("agv_shell") << "load_processes_fxml error";
		return -1;
    }
    loinfo("agv_shell") << "load_processes_fxml fin";
    return 0;
}

int global_parameter::loadXml() {
    std::string file_path = nsp::os::get_module_directory<char>();
    std::string dock_path = file_path + CONFIGE_FILE_NAME;
    rapidxml::xml_document<> doc;
    rapidxml::file<char> *file = nullptr;
    loinfo("agv_shell") << "loadXml:" << dock_path;
	agv_process_.clear();
	vct_white_list_.clear();
	vct_auto_run_list_.clear();
    try {
        file = new rapidxml::file<char>(dock_path.c_str());
        doc.parse<0>(file->data());
        rapidxml::xml_node<>* root = doc.first_node("agv_shell");
        if (root) {
            rapidxml::xml_attribute<>* st = root->first_attribute("auto_startup");
            if (st) {
                set_auto_startup( atoi(st->value()) );
                loinfo("agv_shell") << "loadXml auto_startup:" << get_auto_startup();
            }
            rapidxml::xml_attribute<>* pt = root->first_attribute("port");
            if (pt) {
                set_server_port( atoi(pt->value()) );
                loinfo("agv_shell") << "loadXml port:" << get_server_port();
            }
			rapidxml::xml_attribute<>* fts_pt = root->first_attribute("fts_port");
			if (fts_pt) {
				set_fts_port(atoi(fts_pt->value()));
				loinfo("agv_shell") << "loadXml fts port:" << get_fts_port();
			}
			rapidxml::xml_attribute<>* fts_long_pt = root->first_attribute("fts_long_port");
			if (fts_long_pt) {
				set_fts_long_port(atoi(fts_long_pt->value()));
				loinfo("agv_shell") << "loadXml fts fts_long_port port:" << get_fts_long_port();
			}

            for (rapidxml::xml_node<>* pr = root->first_node(); pr; pr = pr->next_sibling()) {
                agv_process_info ap;
                ap.pid_ = 0;
                ap.hdl_ = NULL;
				ap.th_hd_ = NULL;
                rapidxml::xml_attribute<>* lp = pr->first_attribute("name");
                if (lp) {
                    ap.name_ = lp->value();
                    ap.process_name_ = get_process_name(ap.name_);
                }

                rapidxml::xml_attribute<>* dp = pr->first_attribute("cmd");
                if (dp) {
                    ap.cmd_ = dp->value();
                    ap.ori_cmd_ = ap.cmd_;
                }

                rapidxml::xml_attribute<>* ds = pr->first_attribute("delay_start");
                if (ds) {
                    ap.delay_start_ = atoi(ds->value());
                } else {
                    ap.delay_start_ = 0;
                }
				agv_process_.push_back( ap );
				

            }

			//读取frimware节点信息
			if ((root = root->next_sibling())){
				for (rapidxml::xml_node<>* anode = root->first_node(); anode != NULL; anode = anode->next_sibling()){
					if (strcmp(anode->name(), "udp_m_core") == 0){
						rapidxml::xml_attribute<>* ip = anode->first_attribute("ipv4");
						if (ip) set_m_core_ipv4( ip->value() );
						rapidxml::xml_attribute<>* port = anode->first_attribute("port");
						if (port) set_m_core_port( atoi(port->value()) );
					}
					else if (strcmp(anode->name(), "reset_wait_time") == 0){
						set_reset_wait_time( atoi(anode->value()) );
					}
					else if (strcmp(anode->name(), "firmware_version_control") == 0){
						set_frimware_version_control( atoi(anode->value()) );
					}
					else if (strcmp(anode->name(), "udp_local") == 0){
						rapidxml::xml_attribute<>* ip = anode->first_attribute("ipv4");
						if (ip) set_local_ipv4( ip->value() );
						rapidxml::xml_attribute<>* port = anode->first_attribute("port");
						if (port) set_local_udp_port( atoi(port->value()) );
					}
				}
			}

			//读取白名单节点信息
			if (root && (root = root->next_sibling())){
				for (rapidxml::xml_node<>* pr = root->first_node(); pr; pr = pr->next_sibling())
				{
					white_item_info_t  tmp;
					tmp.file_name = pr->value();
					vct_white_list_.push_back(tmp);
				}
			}
            
            //读自动启动脚本
			if (root && (root = root->next_sibling())){
				for (rapidxml::xml_node<>* pr = root->first_node(); pr; pr = pr->next_sibling())
				{
					vct_auto_run_list_.push_back(pr->value());
				}
			}
			
			//get fix ip and port for DHCP
			if (root && (root = root->next_sibling())){
				if (strcmp(root->name(), "fix_info") == 0){
					rapidxml::xml_attribute<>* ip = root->first_attribute("ipv4");
					if (ip) set_dhcp_fix_ipv4( ip->value() );
					rapidxml::xml_attribute<>* port = root->first_attribute("port");
					if (port) set_dhcp_fix_port( atoi(port->value()) );
				}
			}
			
			//get mclog & syslog ip and port
			if (root && (root = root->next_sibling())){
				for (rapidxml::xml_node<>* anode = root->first_node(); anode != NULL; anode = anode->next_sibling()){
					if (strcmp(anode->name(), "mclog") == 0){
						rapidxml::xml_attribute<>* ip = anode->first_attribute("ipv4");
						if (ip) set_local_mclog_ipv4( ip->value() );
						rapidxml::xml_attribute<>* port = anode->first_attribute("port");
						if (port) set_local_mclog_port( atoi(port->value()) );
					}
					else if (strcmp(anode->name(), "syslog") == 0){
						rapidxml::xml_attribute<>* ip = anode->first_attribute("ipv4");
						if (ip) set_local_syslog_ipv4( ip->value() );
						rapidxml::xml_attribute<>* port = anode->first_attribute("port");
						if (port) set_local_syslog_port( atoi(port->value()) );
					}
				}
			}
        }
        delete file;
    }
    catch (...) 
	{
        delete file;
        loinfo("agv_shell") << "loadXml e";
    }
    loinfo("agv_shell") << "loadXml fin";
    return 0;
}

int global_parameter::init_white_list()
{
	int result = -1;
#ifdef _WIN32
	for ( auto &iter : vct_white_list_)
	{
		if (iter.file_handler != INVALID_HANDLE_VALUE)continue;
		HANDLE hFile = CreateFileA(iter.file_name.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 
			FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_NORMAL, NULL);
		if (INVALID_HANDLE_VALUE == hFile) {
			loerror("agv_shell") << "failed to monopolize the file " << iter.file_name;
			iter.file_handler = INVALID_HANDLE_VALUE;
			continue;
		}
		loinfo("agv_shell") << "success to monolize the file " << iter.file_name;
		iter.file_handler = hFile;
		result = 0;
	}
	return result;
#else
	std::string command;
	auto iter = vct_white_list_.begin();
	for(; iter != vct_white_list_.end(); ++iter) {
		command = "chattr +i " + iter->file_name;
		int sys_ret = system(command.c_str());
		if(sys_ret) {
			loerror("agv_shell") << "failed to lock file:" << iter->file_name << " ret:" << sys_ret;
			continue;
		}
		result = 0;
        loinfo("agv_shell") << "success to lock file:" << iter->file_name;
	}
	return result;
#endif
}

int global_parameter::modify_file_lock(const int is_lock)
{
	int result = -1;
#ifdef _WIN32
	if (1 == is_lock)
	{
		//加锁
		result = init_white_list();
	}
	else
	{
		//解锁
		for (auto & iter : vct_white_list_)
		{
			if (iter.file_handler != INVALID_HANDLE_VALUE)
			{
				if (!CloseHandle(iter.file_handler))
				{
					result = -1;
					loerror("agv_shell") << "failed to close white list file:" << iter.file_name << " handler.";
				}
				else
				{
					loinfo("agv_shell") << "success to close white list file:" << iter.file_name << " handler.";
					iter.file_handler = INVALID_HANDLE_VALUE;
				}
			}
		}
	}
#else
	if(1 == is_lock)
	{
		//加锁
		result = init_white_list();
	}
	else
	{
		//解锁
		std::string command;
		auto iter = vct_white_list_.begin();
		for (; iter != vct_white_list_.end(); ++iter) {
			command = "chattr -i " + iter->file_name;
			int sys_ret = system(command.c_str());
			if(sys_ret) {
				loinfo("agv_shell") << "failed to unlock file:" << iter->file_name << " ret:" << sys_ret;
				continue;
			}
			result = 0;
			loinfo("agv_shell") << "success to unlock file:" << iter->file_name;
		}
	}
#endif
	return result;
}

int global_parameter::query_file_lock()
{
	int status = 0;
#ifdef _WIN32
	for (auto& iter : vct_white_list_)
	{
		if (iter.file_handler != INVALID_HANDLE_VALUE)
		{
			status = 1;
			break;
		}
	}
#else
	std::string command;
	std::string char_ret;
	auto iter = vct_white_list_.begin();
	for (; iter != vct_white_list_.end(); ++iter) {
		command = "lsattr " + iter->file_name;
		if (0 != nsp::toolkit::singleton<global_parameter>::instance()->bash_command(command,char_ret,16, "r")){
			continue;
		}

		if(char_ret.length() > 4 && char_ret.at(4) == 'i'){
			status = 1;
			break;
		}
	}
#endif
	return status;
}

void run_process_by_popen(void* p1, void* p2) {
	loinfo("agv_shell") << "run_process_by_popen start running..";
	if ( !p1 ) return ;
	char* name = (char*)p1;
	
#ifndef _WIN32
    //add file execute access
	if ( chmod(name, S_IRWXU | S_IRWXG ) < 0 ) {
		loerror("agv_shell") << "file:" << name << " chmod failed, errno:" << errno;
		return;
	}
	
	char command[256];
	memset(command,0,256);
	if( p2 ) {
		snprintf(command, 255, "%s %s ", name, (char*)p2);
	} else {
		snprintf(command, 255, "%s ", name);
	}
	command[255]=0;
	
	loinfo("agv_shell") << "start process:" << command;
	
	if (0 != nsp::toolkit::singleton<global_parameter>::instance()->bash_command(command,"w")){
		return;
	}
	loinfo("agv_shell") << "process:" << command << " finished.";
#else

#endif
	return;
}

void run_tar_by_popen(std::string cmd, std::string cmd_parament, std::string des_file, std::string src_file, uint32_t lnk, int id) {
#ifndef _WIN32
	int err = 0;
	//add file execute access
	if (chmod(cmd.c_str(), S_IRWXU | S_IRWXG) < 0) {
		err = errno;
		nsp::toolkit::singleton<agv_shell_server>::instance()->post_tar_backups(lnk, id, des_file, err);
		loerror("agv_shell") << "file:" << cmd << " chmod failed, errno:" << err;
		return;
	}

	std::string command = cmd + " " + cmd_parament +" " +des_file + " " + src_file;
	loinfo("agv_shell") << "start tar backups file:" << command;
	if (0 != nsp::toolkit::singleton<global_parameter>::instance()->bash_command(command,"r")){
		err = -1;
		nsp::toolkit::singleton<agv_shell_server>::instance()->post_tar_backups(lnk, id, des_file, err);
		return;
	}
	loinfo("agv_shell") << "backups files:" << command << " finished.";
	//压缩完通知客户端已完成tar压缩
	nsp::toolkit::singleton<agv_shell_server>::instance()->post_tar_backups(lnk, id, des_file, err);
#else

#endif
	
	return;
}

int run_copye_file_by_popen(const std::string& src_file, const std::string& des_file)
{
#ifndef _WIN32
	std::string command = "cp " + src_file + " " + des_file;
	loinfo("agv_shell") << "start copy file: " << src_file << " to " << des_file;

	if (0 != nsp::toolkit::singleton<global_parameter>::instance()->bash_command(command,"r")){
		return -1;
	}
	loinfo("agv_shell") << "copy files:" << command << " finished.";
#else

#endif
	return 0;
}

//start auto start script
int global_parameter::start_auto_script() {
    auto it = vct_auto_run_list_.begin();
	for(; it != vct_auto_run_list_.end(); ++it) {
		run_process_by_popen( (void*)(*it).c_str(), nullptr);
	}
	
	return 0;
}

void global_parameter::run_progess_by_get_logtype(std::string&log_path, std::set<std::string>&log_type)
{
#ifdef _WIN32
	std::string motion_path = log_path + "/*.*";
	_finddata_t FileInfo;
	long Handle;
	if ((Handle = _findfirst(motion_path.c_str(), &FileInfo)) == -1){
		return;
	}
	do{
		if (0 == strcmp(FileInfo.name, ".") || 0 == strcmp(FileInfo.name, "..")){
			continue;
		}
		std::size_t pos = ((std::string)FileInfo.name).rfind(".log");
		if (pos == std::string::npos ) {
			loinfo("agv_shell") << FileInfo.name << " pos:" << pos;
			continue;
		}
		if ((pos = ((std::string)FileInfo.name).find("20")) == std::string::npos) {
			loinfo("agv_shell") << FileInfo.name << " not log file";
			continue;
		}
		std::string name = ((std::string)FileInfo.name).substr(0, pos - 1);
		if (log_type.find(name) == log_type.end()){
			log_type.emplace(name);
		}
	} while (!_findnext(Handle, &FileInfo));
#else
	struct dirent *ent;
	DIR *dirptr;
	loinfo("agv_shell") << "process path:" << log_path;

	if ( !(dirptr = opendir(log_path.c_str())) ) {
		return;
	}

	while (NULL != (ent = readdir(dirptr))) {
		if (0 == strcmp(ent->d_name, ".") || 0 == strcmp(ent->d_name, "..")) {
			continue;
		}
		
		std::size_t pos = ((std::string)ent->d_name).rfind(".log");
		if (pos == std::string::npos ) {
			loinfo("agv_shell") << ent->d_name << " pos:" << pos;
			continue;
		}
		if( (pos = ((std::string)ent->d_name).find("20")) == std::string::npos) {
			loinfo("agv_shell") << ent->d_name << " not log file";
			continue;
		}
		std::string name = ((std::string)ent->d_name).substr(0, pos - 1);
		if (log_type.find(name) == log_type.end()){
			log_type.emplace(name);
		}
	}

	closedir(dirptr);
#endif
}

void global_parameter::load_shell_version(){
	shell_version_ = load_version_file(SOFT_VERSION_FILE_NAME);
}

void global_parameter::load_config_version(){
	config_version = load_version_file(CONFIG_VERSION_FILE_NAME);
}

std::string global_parameter::load_version_file(const std::string& file_name)
{
	std::string res = "";
	FILE *handle = NULL;
	std::string full_name = nsp::os::get_module_directory<char>() + POSIX__DIR_SYMBOL_STR;
	full_name += file_name;

	uint64_t file_size = nsp::os::get_filesize(full_name);
	if (file_size <= 0) {
		return res;
	}
	if ((handle = fopen(full_name.c_str(), "r")) == NULL) {
		loerror("agv_shell") << "failed to open version file:" << full_name << ".";
		return res;
	}
	//fseek函数将数据指针指向初始位置
	fseek(handle, 0, SEEK_SET);
	char * pchar = nullptr;
	char* strLine= (char*)malloc(MAX_LINE);
	//要求只读第一行数据
	strLine = fgets(strLine, MAX_LINE, handle);
	if((pchar = strchr(strLine,'\n')) != nullptr){
		*pchar = '\0';
	}
	res = strLine;
	fclose(handle);
	handle = NULL;
	return res;
}

int global_parameter::check_file(std::string strfilename, const std::string &strmodify_time, const std::string &start_time, const std::string& end_time){
	std::string::size_type pos;
	std::string filetime;

	if (strfilename.find(".log") == std::string::npos &&
		strfilename.find(".jpg") == std::string::npos){
		return -1;
	}

	if ((pos = strfilename.find_first_of("20")) == std::string::npos){
		loinfo("agv_shell") << strfilename.c_str() << " not log file";
		return -1;
	}
	filetime = strfilename.substr(pos, 15);
	if (filetime.length() < 15){
		return -1;
	}

	std::string::size_type pos_replace;
	if ((pos_replace = filetime.find("T")) != std::string::npos){
		filetime.replace(pos_replace, 1, "_");
	}

	if (start_time <= strmodify_time && end_time >= filetime){
		return 0;
	}
	return -1;
}

std::string global_parameter::get_type(std::string strname){
	std::string type = "";
	std::string::size_type pos;
	pos = strname.find_first_of("20");
	if (strname.find(".log") != std::string::npos && pos > 0){
		type = strname.substr(0, pos - 1);
	}
	else if (strname.find("-C-") != std::string::npos || strname.find("-search-invalid-") != std::string::npos){
		type = "localization";
	}
	else if (strname.find("Deviation_") == 0){
		type = "deviation";
	}

	return type;
}

void global_parameter::load_ntp_server() {
	std::string file_name;
	std::size_t pos;

	auto ap = global_parameter::vct_auto_run_list_.begin();
	while (ap != global_parameter::vct_auto_run_list_.end()) {
		pos = ap->find_last_of(AUTO_START_FILE_NAME);
		if (pos != std::string::npos) {
			file_name = *ap;
			break;
		}
		++ap;
	}
	if (0 == file_name.size()) {
		loinfo("agv_shell") << "can not find file " << AUTO_START_FILE_NAME << " from auto_run_list";
		return;
	}
	try {
		std::ifstream read_file(file_name);
		if (!read_file) {
			return;
		}
		std::string line;
		while (getline(read_file, line)) {
			if (line.substr(0, 14) == "ip_ntp_server=") {
				// get address "123.123.123.123" from "ip_ntp_server=123.123.123.123"
				ntp_server = line.substr(14);
				loinfo("agv_shell") << "ntp server address:" << ntp_server;
				break;
			}
		}
		read_file.close();
	}
	catch (...) {
		loinfo("agv_shell") << "read file error, file:" << file_name;
		return;
	}

	return;
}

int global_parameter::set_ntp_server(const std::string& address) {
	int ret = nsp::tcpip::endpoint::is_effective_ipv4(address);
	if( !ret ) {
		loerror("agv_shell") << "invalid ip address:" << address;
		return ret;
	}
#ifndef _WIN32
	std::string file_name;
	std::size_t pos;
	std::string cmd_str = "sed -i 's/^ip_ntp_server=.*/ip_ntp_server=";

	auto ap = global_parameter::vct_auto_run_list_.begin();
	while (ap != global_parameter::vct_auto_run_list_.end()) {
		pos = ap->find_last_of(AUTO_START_FILE_NAME);
		if (pos != std::string::npos) {
			file_name = *ap;
			break;
		}
		++ap;
	}
	if (0 == file_name.size()) {
		loinfo("agv_shell") << "can not find file " << AUTO_START_FILE_NAME;
		return -1;
	}
	// sed -i 's/^ip_ntp_server=.*/ip_ntp_server=address/' file_name
	cmd_str += address + "/' "+ file_name;
	ret = system(cmd_str.c_str());
	if (ret == 0) {
		ntp_server = address;
		loinfo("agv_shell") << "set ntp server address:" << address << " file:" << file_name;
	} else {
		loerror("agv_shell") << "set ntp server error:" << address << ret;
	}
#endif
	return ret;
}

int global_parameter::bash_command(const std::string &command, std::string &result, int max_length, const char *type){
#ifndef _WIN32
	if( max_length > 0){
		char * buffer = nullptr;
		try{
			buffer= new char[max_length];
			memset(buffer,0,sizeof(char)*(max_length));
		}
		catch(...){
			return -1;
		}

		FILE *p = nullptr;
		p = popen(command.c_str(), type);
		if (!p) {
			loerror("agv_shell") << "popen failure, cmd:" << command;
			delete [] buffer;
			return -1;
		}
		else {
			buffer = fgets(buffer, max_length, p);
			if(!buffer){
				loerror("agv_shell") << "fgets buffer error";
				delete [] buffer;
				pclose(p);
				return -1;
			}
			char * pchar = nullptr;
			if((pchar = strchr(buffer,'\n'))!= nullptr){
				*pchar = '\0';
			}
			result = buffer;
			delete [] buffer;
		}
		pclose(p);
	}
#endif
	return 0;
}

int global_parameter::bash_command(const std::string & command, const char * type ){
#ifndef _WIN32
	FILE *p = nullptr;
	p = popen(command.c_str(), type);
	if (!p) {
		loerror("agv_shell") << "popen failure, cmd:" << command;
		return -1;
	}
	//loinfo("agv_shell")<<"2";
	pclose(p);
#endif
	return 0;
}
