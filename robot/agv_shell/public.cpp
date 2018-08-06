#include "public.h"
#include "agv_shell_common.h"
#include "log.h"
#include "os_util.hpp"
#include "posix_time.h"    //posix__systime_t
#include "toolkit.h"

std::string get_process_name(std::string& path) {
    std::string name;
    std::size_t pos = path.find_last_of("/");
    std::size_t pos1 = path.find_last_of("\\");
    std::size_t p = 0;
    if (pos == std::string::npos
            && pos1 == std::string::npos) {
        return path;
    } else if (pos == std::string::npos) {
        p = pos1;
    } else if (pos1 == std::string::npos) {
        p = pos;
    } else {
        p = pos1 > pos ? pos1 : pos;
    }

    name = path.substr(p + 1, path.length() - p - 1);
    return name;

}

std::string get_process_path(std::string& path) {
    std::string name = "";
    std::size_t pos = path.find_last_of("/");
    std::size_t pos1 = path.find_last_of("\\");
    if (pos == std::string::npos) {
        pos = 0;
    }
    if (pos1 == std::string::npos) {
        pos1 = 0;
    }
    std::size_t p = pos1 > pos ? pos1 : pos;
    if (p > 0) {
        name += path.substr(0, p + 1);

    } else {
        name = "";
    }
    return name;

}

std::size_t split_symbol_string(const std::basic_string<char> &source, const char symbol, std::vector<std::basic_string<char>> &vct_substr) {
	std::basic_string<char> tmp;
	std::size_t idx_previous = 0, idx_found = 0;
	while (std::basic_string<char>::npos != (idx_found = source.find_first_of(symbol, idx_found))) {
		tmp.assign(&source[idx_previous], idx_found - idx_previous);
		if (tmp.size() > 0) {
			vct_substr.push_back(tmp);
		}
		++idx_found;
		while (0x20 == source[idx_found] && 0 != source[idx_found]) ++idx_found;
		idx_previous = idx_found;
	}
	if (0 == idx_previous) {
		return 0;
	}
	if (source.size() != idx_previous) {
		if( '\n' == source[source.size() - 1] ) { //delete \n 
			tmp.assign(&source[idx_previous], source.size() - idx_previous - 1);
		} else {
			tmp.assign(&source[idx_previous], source.size() - idx_previous);
		}
		vct_substr.push_back(tmp);
	}
	return vct_substr.size();
}

std::string convert_positive(const std::string& str, const char preview_cr, const  char new_cr){
	std::string result = str;
	for (size_t i = 0; i < result.size(); ++i){
		if (result.at(i) == preview_cr){
			result.at(i) = new_cr;
		}
	}
	return result;
}

#ifndef _WIN32
static int tar_cmd_exec(std::string& cmd_str)
{
	int ret = system(cmd_str.c_str());
	if (256 != ret && 0 != ret) {
		loerror("agv_shell") << "create tar file error:" << ret << " cmd:" << cmd_str.substr(0, 1024);
		return -EIO;
	}
	return 0;
}

static void get_relative_path(std::string &path_new, std::string &file_name)
{
	size_t pos = std::string::npos;

	pos = file_name.find("/log/");
	if (pos != std::string::npos) {
		path_new = file_name.substr(0, pos);
		file_name = file_name.substr(pos + 1);
	} else {
		path_new = "";
	}
}

static int set_cmd_str_head(
	std::string &cmd_str, 
	std::string &path_old, std::string &path_new,
	const std::string& compressed_file_name)
{
	int ret = 0;
	if (path_new != path_old) {
		if (!path_old.empty()) {
			ret = tar_cmd_exec(cmd_str);
			if (ret) {
				return ret;
			}
		}
		// example: "cd /gzrobot/; tar --warning=no-file-changed -rf log_12345678.tar "
		cmd_str = "cd " + path_new + "; tar --warning=no-file-changed -rf " + compressed_file_name + " ";
		path_old = path_new;
	}
	return 0;
}

static int check_cmd_length(
	std::string &cmd_str, const long max_size,
	const std::string& file_name, const std::string& path_new,
	const std::string& compressed_file_name)
{
	int ret = 0;
	long len = cmd_str.size() + file_name.size();
	if (len >= max_size) {
		ret = tar_cmd_exec(cmd_str);
		if (ret) {
			return ret;
		}
		if (path_new.empty()) {
			cmd_str = "tar --warning=no-file-changed -rf " + compressed_file_name + " ";
		} else {
			cmd_str = "cd " + path_new + "; tar --warning=no-file-changed -rf " + compressed_file_name + " ";
		}
	}
	return 0;
}

int compress_files(const std::vector<std::string>& files, std::string& compressed_file_name)
{
	long arg_max = -1;
	int ret = 0;

	if (files.empty()) {
		return -ENOENT;
	}
	// get the command line's max length 
	arg_max = sysconf(_SC_ARG_MAX);
	if (arg_max < 0) {
		arg_max = 4096;
	}

	std::string path_old;
	std::string path_new;
	std::string file_name;
	int need_run = 0;
	std::string cmd_str = "tar --warning=no-file-changed -rf " + compressed_file_name + " ";
	for (size_t i = 0; i < files.size(); i++) {
		file_name = files[i];
		// file_name:"/dir1/dir2/agv_shell/log/nshost_20180724_084756.log"
		//    get-> path_new:"/dir1/dir2/agv_shell" file_name:"log/nshost_20180724_084756.log"
		get_relative_path(path_new, file_name);
		ret = set_cmd_str_head(cmd_str, path_old, path_new, compressed_file_name);
		if(ret) {
			break;
		}
		ret = check_cmd_length(cmd_str, arg_max, file_name, path_new, compressed_file_name);
		if(ret) {
			break;
		}
		cmd_str += file_name + " ";
		need_run = 1;
	}
	if (0 == ret && need_run) {
		ret = tar_cmd_exec(cmd_str);
	}

	if (0 == ret) {
		cmd_str = "xz " + compressed_file_name;
		// compress a tar file to xz file(tar file will be deleted). example: "/bin/xz log_20180308_143537.tar"
		ret = system(cmd_str.c_str());
		if (ret) {
			loerror("agv_shell") << "create xz file error:" << ret << " cmd:" << cmd_str;
		} else {
			// return a xz file name
			compressed_file_name += ".xz";
			loinfo("agv_shell") << "create xz file:" << compressed_file_name << " finished";
		}
	}
	if(ret) {
		// delete xxx.tar file
		nsp::os::rmfile(compressed_file_name);
		compressed_file_name += ".xz";
		//delete xxx.tar.xz file
		nsp::os::rmfile(compressed_file_name);
		return -EIO;
	}

	return 0;
}
#endif

int get_time_zone()
{
#ifndef _WIN32
    time_t t = time(NULL);
    struct tm local = *(localtime(&t));
    struct tm gm = *(gmtime(&t));
	
    int hour = local.tm_hour - gm.tm_hour;
    int zone = 0;
    if(hour > 0) {
        if(local.tm_mday == gm.tm_mday) {
            zone = hour;
		} else {
            zone = hour - 24;
		}
    } else {
        if(local.tm_mday == gm.tm_mday) {
            zone = hour;
        } else {
            zone = hour + 24;
        }
    }
	return zone;
#else
	return 8;
#endif
}


int reboot_os() {
#ifdef _WIN32
    system("shutdown -r -t 3");
#else
    system("reboot");
#endif // _WIN32
    return 0;
}

int shutdown_os() {
#ifdef _WIN32
    system("shutdown -s -t 3");
#else
    system("shutdown -h now");
#endif // _WIN32
    return 0;
}


