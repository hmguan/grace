#include <getopt.h>
#include <stdio.h>
#include <string.h>

#include "args.h"
#include "vartypes.h"
#include "logger.h"
#include "getopt.h"

#include "posix_string.h"

#pragma pack(push,1)

struct  {
    posix__boolean_t __sim_flag;
    position_t __original_simodo;
    char __tcp_ipv4[posix__ipv4_length];
    uint16_t __tcp_port;
	uint16_t __udp_port;
	char __canio_driver_file[RUN_FILENAME_LENGTH];
    int __must_login;
    int __load_on_exec;
} __startup_parameters;

#pragma pack(pop)

/* 不可见短参数索引定义 */
enum opt_invisible_indx {
    kInvisibleOptIndex_GetHelp = 'h',
    kInvisibleOptIndex_ChangeCanbusLink = 'l',
    kInvisibleOptIndex_GetVersion = 'v',
    kInvisibleOptIndex_SetSimulate = 's',
    kInvisibleOptIndex_StartupSimOdo = 0x301,
    kInvisibleOptIndex_ServiceHost,
    kInvisibleOptIndex_ServiceTcpPort,
	kInvisibleOptIndex_ServiceUdpPort,
	kInvisibleOptIndex_NoLogin,
    kInvisibleOptIndex_LoadOnExec,
};

static const struct option long_options[] = {
    {"help", no_argument, NULL, kInvisibleOptIndex_GetHelp},
    {"link", required_argument, NULL, kInvisibleOptIndex_ChangeCanbusLink},
    {"version", no_argument, NULL, kInvisibleOptIndex_GetVersion},
    {"simulate", no_argument, NULL, kInvisibleOptIndex_SetSimulate},
    {"startup-simodo", required_argument, NULL, kInvisibleOptIndex_StartupSimOdo},
    {"service-host", required_argument, NULL, kInvisibleOptIndex_ServiceHost},
    {"service-tcp-port", required_argument, NULL, kInvisibleOptIndex_ServiceTcpPort},
	{"service-udp-port", required_argument, NULL, kInvisibleOptIndex_ServiceUdpPort },
	{"no-login", no_argument, NULL, kInvisibleOptIndex_NoLogin },
    {"load-on-exec", no_argument, NULL, kInvisibleOptIndex_LoadOnExec},
    {NULL, 0, NULL, 0}
};

/* 显示命令行参数及其详情 */
static
void run__dispay_usage() {
    static const char *usage_context =
            "NAME\n"
            "\tmotion_template - agv motion template\n"
            "\n"
            "SYNOPSIS\n"
            "\tmotion_template\n"
            "\t\t[-l|--link] [%%FILE%%] relocated canbus shared library linker path\n"
            "\t\t[-h|--help] display usage context and help informations\n"
            "\t\t[-v|--version] display versions of executable archive\n"
            "\t\t[-s|--simulate] simulate motion driver/device.\n"
            "\t\t[-r|--recover] recover calibration from xml to binary\n"
            "\t\t[-a|--apply] apply calibration from binary to xml.WARNING,this operation will destory\n"
            "\t\t\tThis operation will overwrite the existing xml files. The system will backup before covering, but it will not guarantee success.\n"
            "\t\t[--startup-simodo] initialize odo data like x:y:angle in simulate mode.\n"
            "\t\t[--service-host][%%IPV4] specify IPv4 address for local TCP/TCP network service.\n"
            "\t\t[--service-tcp-port][%%TCP PORT] specify port for local TCP network service.\n"
			"\t\t[--service-udp-port][%%UDP PORT] specify port for local UDP network service.\n"
			"\t\t[--no-login] do not use login pakcet to verifity network connection.\n"
            "\t\t[--load-on-exec] load storage data from file on process startup\n"
            ;

    printf("%s", usage_context);
}

/* 打印版权信息和版本信息 */
static
void run__display_author_information() {
    static const char *author_context =
            "motion_template 2,2,0,0\n"
            "Copyright (C) 2018 Zhejiang Guozi RoboticsCo., Ltd.\n"
            "For bug reporting instructions, please see:\n"
            "<http://www.gzrobot.com/>.\n"
            "Find the AGV manual and other documentation resources online at:\n"
            "<http://www.gzrobot.com/productCenter/>.\n"
            "For help, type \"help\".\n"
            ;

    printf("%s", author_context);
}

static
int run__parse_simodo(char *pars, position_t *odo) {
    char *mid = NULL, *save;
    double *odo_ptr[3] = {&odo->x_, &odo->y_, &odo->angle_};
    int i = 0;
    if (!pars || !odo) {
        return -1;
    }

    while ((NULL != (mid = posix__strtok(mid ? NULL : pars, ":", &save))) && (i < 3)) {
        *(odo_ptr[i]) = strtod(mid, NULL);
        log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "simodo startup parameters set:%f", *(odo_ptr[i]));
        i++;
    }
    return 0;
}

int run__check_args(int argc, char **argv) {
    int opt_index;
    int opt;
    int retval = 0;
    char shortopts[128];
    
    /* 默认不启用驱动模拟 */
    __startup_parameters.__sim_flag = posix__false;
    /* 默认不提供初始定位 */
    __startup_parameters.__original_simodo.x_ = __startup_parameters.__original_simodo.y_ = __startup_parameters.__original_simodo.angle_ = 0;
    /* 默认的服务 endpoint 从配置文件获取 */
    memset(__startup_parameters.__tcp_ipv4, 0, sizeof(__startup_parameters.__tcp_ipv4));
    __startup_parameters.__tcp_port = 0;
    /* no need to load storage form file on process startup by default */
    __startup_parameters.__load_on_exec = 0;
    
    /* 默认的 CANIO 驱动执行文件 */
#if _WIN32
    posix__strcpy(__startup_parameters.__canio_driver_file, cchof(__startup_parameters.__canio_driver_file), "canbus_driver.dll");
#else
    posix__strcpy(__startup_parameters.__canio_driver_file, cchof(__startup_parameters.__canio_driver_file), "canbus_driver.so");
#endif
    __startup_parameters.__must_login = 1;
    
    /* 组合长短启动参数， 执行综合判定 */
    posix__sprintf(shortopts, cchof(shortopts), "svhl:%d:%d:%d:%d:%d:%d:%d", 
		kInvisibleOptIndex_StartupSimOdo, kInvisibleOptIndex_ServiceHost, kInvisibleOptIndex_ServiceTcpPort, kInvisibleOptIndex_ServiceUdpPort, \
		kInvisibleOptIndex_NoLogin, kInvisibleOptIndex_LoadOnExec );
    opt = getopt_long(argc, argv, shortopts, long_options, &opt_index);
    while (opt != -1) {
        switch (opt) {
            case 'l':
                if (optarg) {
                    posix__strcpy(__startup_parameters.__canio_driver_file, cchof(__startup_parameters.__canio_driver_file), optarg);
                    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,
                            "canio target shared library has been changed to %s", __startup_parameters.__canio_driver_file);
                }
                break;
            case 'v':
                retval = -1;
                run__display_author_information();
                break;
            case 'h':
                retval = -1;
                run__dispay_usage();
                break;
            case 's':
                log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,
                        "application siwtch to simulate mode.");
                __startup_parameters.__sim_flag = posix__true;
                break;
            case kInvisibleOptIndex_StartupSimOdo:
                if (run__parse_simodo(optarg, &__startup_parameters.__original_simodo) < 0) {
                    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,
                            "--startup-simodo initialize odo data like x:y:angle in simulate mode");
                }
                break;
            case kInvisibleOptIndex_ServiceHost:
                if (optarg){
                    posix__strcpy(__startup_parameters.__tcp_ipv4, cchof(__startup_parameters.__tcp_ipv4), optarg);
                }
                break;
            case kInvisibleOptIndex_ServiceTcpPort:
                if (optarg){
                    __startup_parameters.__tcp_port = (uint16_t)strtoul(optarg, NULL, 10);
                }
                break;
			case kInvisibleOptIndex_ServiceUdpPort:
				if (optarg){
					__startup_parameters.__udp_port = (uint16_t)strtoul(optarg, NULL, 10);
				}
				break;
            case kInvisibleOptIndex_NoLogin:
                __startup_parameters.__must_login = 0;
                break;
            case kInvisibleOptIndex_LoadOnExec:
                __startup_parameters.__load_on_exec = 1;
                break;
            case '?':
                printf("?\n");
                break;
            case 0:
                printf("0\n");
                break;
            default:
                retval = -1;
                run__dispay_usage();
                break;
        }
        opt = getopt_long(argc, argv, /*"svhl:"*/shortopts, long_options, &opt_index);
    }

    return retval;
}

char *run__getarg_tcphost(char *host){
    if (host){
        posix__strcpy(host, posix__ipv4_length, __startup_parameters.__tcp_ipv4);
    }
    
    return host;
}

uint16_t run__getarg_tcpport(){
    return __startup_parameters.__tcp_port;
}

uint16_t run__getarg_udpport(){
	return __startup_parameters.__udp_port;
}

posix__boolean_t run__getarg_simflag(){
    return __startup_parameters.__sim_flag;
}

void run__getarg_simodo(double *x, double *y, double *w){
    if (x){
        *x = __startup_parameters.__original_simodo.x_;
    }
    if (y){
        *y = __startup_parameters.__original_simodo.y_;
    }
    if (w){
        *w = __startup_parameters.__original_simodo.angle_;
    }
}

char *run__getarg_canio_driver(char *file){
    if (file){
        posix__strcpy(file, RUN_FILENAME_LENGTH, __startup_parameters.__canio_driver_file);
    }
    return file;
}

posix__boolean_t run__if_must_login() {
    return (posix__boolean_t)__startup_parameters.__must_login;
}

posix__boolean_t run__if_loadonexec() {
    return (posix__boolean_t)__startup_parameters.__load_on_exec;
}