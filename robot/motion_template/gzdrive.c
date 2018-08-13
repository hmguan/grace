#include "gzdrive.h"
#include "nsp.h"

#include "var.h"
#include "vartypes.h"
#include "vehicle.h"
#include "dio.h"
#include "drive_unit.h"
#include "wheel.h"

#include "xml_loaded.h"
#include "private_udp.h"
#include "private_udp_cache.h"

#include "posix_string.h"
#include "posix_atomic.h"
#include "posix_time.h"
#include "logger.h"
#include "posix_wait.h"
#include "posix_ifos.h"
#include "nis.h"
#include "logger.h"
#include "elmo.h"

#include "error.h"
#include "errtypes.h"

#include <stdio.h>

//static const uint16_t BIT_MASK_UINT16[] = 
//   {1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000};

#pragma pack(push, 1)

////////////////////////////////////        设备驱动(电机)命令/应答部分协议       /////////////////////////////////////////////////////////////////////////////
struct vcu_command {
    uint16_t sequence;
    uint8_t version;
    uint8_t reserve1;
    uint8_t command;
    uint8_t reserve2[3];
    uint8_t op;
    uint8_t subop;
    uint16_t par;
    uint16_t length;
    uint8_t data[0];
};

struct vcu_response {
    uint16_t sequence;
    uint8_t version;
    uint8_t reserve1;
    uint8_t command;
    uint8_t ack;
    uint8_t reserve2[2];
    uint8_t op;
    uint8_t subop;
    uint16_t par;
    uint16_t length;
    uint8_t data[0];
};

struct vcu_swheel_config {
    int object_type;
    int min_angle;
    int max_angle;
    int zero_angle;
    int zero_angle_enc;
    int max_w;
    int control_mode;
    int scale_control;
    int scale_feedback;
    int control_cp;
    int control_ci;
    int control_cd;
};

struct vcu_dwheel_config {
    int object_type;
    int max_speed;
    int max_acc;
    int max_dec;
    int control_mode;
    int scale_control;
    int scale_feedback;
    int roll_weight;
    int slide_weight;
};

struct vcu_sdd_config {
    int object_type;
    int gauge;
};

struct vcu_driveunit_config {
    int drive_unit_type;
    int install_x;
    int install_y;
    int install_theta;
    int calibrated_x;
    int calibrated_y;
    int calibrated_theta;
    int count_of_wheel;
    char wheels[0];
};

struct vcu_vehicle_config {
    int chassis_type;
    int max_speed;
    int creep_speed;
    int max_acc;
    int max_dec;
    int max_w;
    int creep_w;
    int max_acc_w;
    int max_dec_w;
    int steer_angle_error;
    int count_of_drive_unit;
    char units[0];
};

struct vcu_vehicle_misc_config {
    char object_name[16];
    uint32_t object_type;
    uint8_t object_data[8];
};

////////////////////////////////////        驱动协议和通信约定部分     /////////////////////////////////////////////////////////////////////////////
#define COMMON_DATA_SIZE        (64)  // 主要数据段长度， 业务数据段偏移
#define BUSSINESS_OFFSET        (COMMON_DATA_SIZE)

struct vcu_protocol_head {
    uint16_t id;
    uint8_t ver;
    uint8_t seg;    /* total segment for this packet */
    uint32_t off; /* target address byte offset */
    uint16_t len; // 不计包头的字节长度（即dat的字节长度）
    uint8_t dat[0];
};/* 10 bytes */

struct vcu_common_command {
    uint16_t mask;
    int16_t x; // 线速度 m/s * 1000
    int16_t y; // 线速度 m/s * 1000
    int16_t angular_velocity; // 角速度 rad/s * 1000
}; /* 8 bytes */

struct vcu_battery_protocol {
    uint16_t voltage; // 总电压 v * 100
    uint16_t current; // 总电流 a * 100
    uint16_t total_capacity; // 电池总容量
    uint16_t capacity; // 电池剩余容量
    uint16_t cycle; // 电池循环充电次数(寿命标志) 
    uint16_t status; // 电池状态
}; /* 12 bytes */

struct vcu_common_report {
    struct vcu_common_command command;
    uint32_t time_stamp_second;
    uint32_t time_stamp_microsecond;
    uint16_t driver_error[8]; // 最多允许反馈8个轮子的错误码
    uint16_t vcu_status; // 底盘整体故障
    uint16_t vcu_error;
    struct vcu_battery_protocol battery; // 电池参数
    int32_t coordinate_x; // M核递推的坐标x
    int32_t coordinate_y; // M核递推的坐标y
    int16_t coordinate_angle; // M核递推的坐标角度
}; /* 58 bytes */ 

////////////////////////////////////        对象和功能结构定义部分     /////////////////////////////////////////////////////////////////////////////
#define GZD_UDP_PORT                (0x5000)
//#define GZD_UDP_TARGET            ("10.10.100.169")
#define GZD_UDP_TARGET              ("192.168.0.2")
#define GZD_UDP_LOCAL               ("192.168.0.3")
#define GZD_DEVICE_CONFIG_PORT      (0x4002)
#define GZD_MAX_MISC_NUM            (50)

struct vcu_exchange_node {
    uint32_t var_id;
    uint32_t var_offset;
    uint32_t length;
};

struct vcu_exchange_mapping {
    uint32_t driver_address;
    uint32_t total_length;
    uint32_t mapping_count;
    struct vcu_exchange_node mapping[20]; /* 20 unidirectional mappings are provided at most */
};

struct vcu_udp_object {
    char local_ipv4_[16];
    uint16_t local_port_;
    int access_;
    posix__pthread_t vcu_timer;
    struct vcu_exchange_mapping to_driver; /* send these var values to VCU */
    struct vcu_protocol_head *pkt_write_customer_to_drver;
    struct vcu_exchange_mapping to_var; /* get these var values from VCU  */
    struct vcu_exchange_mapping to_init; /* get these var values from VCU one shot */
    struct vcu_vehicle_config *vehicle_config; /* device configution */
    int vehicle_config_size; /* size of device configution in bytes  */
    int vcu_pktid;
    HUDPLINK link; /* device configution UDP session */
};

struct vcu_vehicle_misc_object {
    int vehicle_misc_config_size; /* 配置部分数据包大小  */
    struct vcu_vehicle_misc_config *vehicle_misc_config;
};

#pragma pack(pop)

#define SUBOP_QUERY_VEHICLE_CONFIG_DATA     (0)     // 请求底盘配置数据 VCU->Linux
#define SUBOP_WRITE_VEHICLE_CONFIG_DATA     (1)     // 写底盘配置数据  Linux->VCU
#define SUBOP_READ_VEHICLE_CONFIG_DATA      (2)     // 读取底盘配置数据 Linux->VCU

#define SUBOP_QUERY_VEHICLE_CONFIG_MISC     (3)     // 请求底盘杂项数据 VCU->Linux
#define SUBOP_WRITE_VEHICLE_CONFIG_MISC     (4)     // 写底盘杂项数据  Linux->VCU
#define SUBOP_READ_VEHICLE_CONFIG_MISC      (5)     // 读取底盘杂项数据 Linux->VCU

static
int __atomic_gzdid = 1;

static
struct vcu_udp_object *__gzd_node = NULL;

static
struct vcu_vehicle_misc_object *__gzd_vehicle_misc_config = NULL;

static
posix__waitable_handle_t __waiter;
static
posix__waitable_handle_t __init_completed; /* completion event for VCU report. */

static
int __keep_loop = 1; // 循环发送请求退出

static const unsigned char BIT_MASK_BYTE[] = {1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80};

////////////////////////////////////        自主底盘UDP交互部分     ///////////////////////;//////////////////////////////////////////////////////
static
int nspi__on_vcu_common_report(struct vcu_common_report *common_report) {
    var__vehicle_t *veh;
    var__dio_t *dio;
    int i;
    int hardware_error = 0;

    dio = var__get_dio_byid(kVarFixedObject_InternalDIO);
    if (dio) {
        // do 的全部16位数据都作为参考数据使用
        dio->i.di_ = common_report->command.mask;

        // copy all driver errors to dio object, if any error existed, mark fault stop status
        for (i = 0; (i < (sizeof (common_report->driver_error) / sizeof (common_report->driver_error[0]))); i++) {
            dio->i.ai_[0].data_[i] = common_report->driver_error[i];
            if (0 != common_report->driver_error[i]) {
                log__save("motion_template", kLogLevel_Warning, kLogTarget_Stdout | kLogTarget_Filesystem, 
                            "driver error report from VCU,index=%d,error=%d", i, common_report->driver_error[i]);
                hardware_error |= BIT_MASK_BYTE[i];
            }
        }

        /* fatal index from device driver report, save at 24-31 bits of vehide's hardware error object. */
        hardware_error <<= 24;

        // VCU整体错误记录
        dio->i.ai_[1].data_[0] = common_report->vcu_status;
        dio->i.ai_[1].data_[1] = common_report->vcu_error;
        if (0 != common_report->vcu_error ) {
            log__save("motion_template", kLogLevel_Warning, kLogTarget_Stdout | kLogTarget_Filesystem, 
                            "VCU error : %u", common_report->vcu_error );

            /* VCU error code save at 0-15 bits of vehide's hardware error object */
            hardware_error |= common_report->vcu_error;
            
        }

        // 电池电量相关
        dio->i.ai_[2].data_[0] = common_report->battery.voltage;
        dio->i.ai_[2].data_[1] = common_report->battery.current;
        dio->i.ai_[2].data_[2] = common_report->battery.total_capacity;
        dio->i.ai_[2].data_[3] = common_report->battery.capacity;
        dio->i.ai_[2].data_[4] = common_report->battery.cycle;
        dio->i.ai_[2].data_[5] = common_report->battery.status;
        var__release_object_reference(dio);
    }

    /* due the featuer of the NTP protocol,
       there is a possible reverse time difference within 20 milliseconds between VCU and Linux.
       Importantly, 
       the VCU time may be 20 milliseconds later than Linux time, 
       because the navigation cycle uses (current system time - previous VCU time)to calculate the delta T, 
       so it may lead to the time difference is negative.
     */
    veh = var__get_vehicle();
    if (veh) {
        uint64_t abs_time = (uint64_t) common_report->time_stamp_second;
        abs_time *= 1000; // seconds to milliseconds
        abs_time += (common_report->time_stamp_microsecond / 1000); // additional microseconds to milliseconds
        
        // this timestamp is a piece of shit.
        if (abs_time < veh->i.time_stamp_) {
            log__save("motion_template", kLogLevel_Warning, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "abs_time="UINT64_STRFMT", veh time="UINT64_STRFMT, abs_time, veh->i.time_stamp_);
            var__release_object_reference(veh);
            return 0;
        }

        if (common_report->command.mask & 1) {
            veh->i.enabled_ |= 1;
        } else {
            veh->i.enabled_ &= ~1;
        }

        /* emergency stop button has down, save at bit[19] of vehide's hardware error object */
        if (common_report->command.mask & 8) {
            hardware_error |= (8 << 16);
            if ( VEH_HEALTHY == veh->fault_stop_) {
                log__save("motion_template", kLogLevel_Warning, kLogTarget_Stdout | kLogTarget_Filesystem, "hardware emergency stop button down.");
            }
        }else{
            var__error_handler_t *err = var__get_error_handler();
            if (err) {
                if (err->error_[kVarFixedObject_Vehide].hardware & (8 << 16) ) {
                    log__save("motion_template", kLogLevel_Warning, kLogTarget_Stdout | kLogTarget_Filesystem, "hardware emergency stop button release.");
                }
                var__release_object_reference(err);
            }
        }

        int16_t actual_x = common_report->command.x;
        veh->i.actual_velocity_.x_ = (double) actual_x / 1000;

        int16_t actual_y = common_report->command.y;
        veh->i.actual_velocity_.y_ = (double) actual_y / 1000;

        int16_t actual_w = common_report->command.angular_velocity;
        veh->i.actual_velocity_.w_ = (double) actual_w / 1000;

        uint64_t localMs = posix__clock_epoch();
        localMs /= 10000;
        int64_t delta = abs_time - localMs;
        if (delta < -10000 || delta > 10000) {
            veh->i.time_stamp_ = localMs;
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "gz drive timestamp checking error, abs time="UINT64_STRFMT", local ms="UINT64_STRFMT, abs_time, localMs);
        } else {
            veh->i.time_stamp_ = abs_time;
        }

        // odo meter from VCU calculator result.
        veh->i.odo_meter_.x_ = (double) common_report->coordinate_x / 1000;
        veh->i.odo_meter_.y_ = (double) common_report->coordinate_y / 1000;
        veh->i.odo_meter_.w_ = (double) common_report->coordinate_angle / 1000;

        /*
        no error now, and status is clearing-fault
        clear hardware fault flag and remove clear-fault order.
        overwise, mark error to vehide object's hardware field.
        */
        if (0 != hardware_error) {
            var__mark_hardware_error(kVarFixedObject_Vehide, hardware_error);
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                "hardware error=0x%08X fault stop=%d", hardware_error, veh->fault_stop_);
        } else {
            if (veh->fault_stop_ & VEH_HARDWARE_HANDLEING) {
                veh->fault_stop_ &= ~VEH_HARDWARE_FAULT;
                veh->fault_stop_ &= ~VEH_HARDWARE_HANDLEING;
            }
        }
        var__release_object_reference(veh);
    }
    
    return 0;
}

static
int nspi__on_vcu_customer(const struct vcu_exchange_mapping *map_target, const unsigned char *bussiness, int size) {
    int retval;
    int i;
    char *from = (char *) bussiness;
    var__functional_object_t *object;
    int effective_size = size;

    if (!map_target) {
        return -EINVAL;
    }

    retval = 0;
    for (i = 0; i < map_target->mapping_count; i++) {
        const struct vcu_exchange_node *mapping = &map_target->mapping[i];

        // 包数据剩余长度不足以解释xml中要求的数据
        if (mapping->length > effective_size) {
            log__save("motion_template", kLogLevel_Warning, kLogTarget_Stdout | kLogTarget_Filesystem, 
                "mapping length too large,%u,%u", mapping->length, effective_size);
            retval = -1;
            break;
        }

        objhld_t hld = var__getobj_handle_byid(mapping->var_id);
        if (hld < 0) {
            from += mapping->length;
            effective_size -= mapping->length;
            continue;
        }
        object = objrefr(hld);
        if (!object) {
            retval = -1;
            break;
        }

        var__acquire_lock(object);

        // 要锁后直接针对偏移拷贝长度
        if (mapping->var_offset + mapping->length > object->body_length_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "drive mapping memory error, offset=%u, length=%u, body=%u", mapping->var_offset, mapping->length, object->body_length_);
        } else {
            memcpy(object->body_ + mapping->var_offset, from, mapping->length);
        }

        var__release_lock(object);
        objdefr(hld);

        from += mapping->length;
        effective_size -= mapping->length;
    }

    return retval;
}

static
int nspi_vcu_callback(const unsigned char *data, int cb) {
    struct vcu_protocol_head *head = (struct vcu_protocol_head *) data;
    struct vcu_common_report *report;
    unsigned char *bussiness;
    int bussiness_size;

    /* the packet data length received but not enough to fill the (common packet length + bussiness packet length). */
    if (cb < (sizeof (struct vcu_protocol_head) + sizeof (struct vcu_common_report))) {
        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "Invalid packet size:%u, offset:0", head->len);
        return -1;
    }

    if (0 != head->off) {
        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "Invalid packet offset:%u", head->off);
        return -1;
    }

    /* parse the report data by protocol. */
    report = (struct vcu_common_report *) head->dat;
    bussiness = &head->dat[BUSSINESS_OFFSET];

    nspi__on_vcu_common_report(report);

    bussiness_size = cb - COMMON_DATA_SIZE - sizeof (struct vcu_protocol_head);
    if (bussiness_size > 0) {

        /*
         * If the reverse initialization is specified in the configuration information(mnt.xml node init),
         * the VCU's first reported data must be transferred to the local variable according to the requirements of @to_init.
         * Otherwise, these reports will be transferred to the local variables according to the requirements of @to_var. 
         * Note that @to_init is only used once.
        */
        if ((__gzd_node->to_init.total_length > 0) && (__gzd_node->to_init.mapping_count > 0)){
            nspi__on_vcu_customer(&__gzd_node->to_init, bussiness, bussiness_size);
            __gzd_node->to_init.total_length = 0;
            __gzd_node->to_init.mapping_count = 0;
            posix__sig_waitable_handle(&__init_completed);
        }else{
            nspi__on_vcu_customer(&__gzd_node->to_var, bussiness, bussiness_size);
        }
    }
    return 0;
}

static
int nspi__write_customer() {
    uint32_t i;
    uint32_t pkt_offset;
    var__functional_object_t *obj;
    char *p_var, *p_pkt_data;

    /* 还没有为发送给驱动的数据构建统一的数据包 */
    if (!__gzd_node->pkt_write_customer_to_drver) {
        uint32_t total_length = sizeof (struct vcu_protocol_head);
        total_length += __gzd_node->to_driver.total_length;
        __gzd_node->pkt_write_customer_to_drver = (struct vcu_protocol_head *) malloc(total_length);
        if (!__gzd_node->pkt_write_customer_to_drver) {
            return -1;
        }

        memset(__gzd_node->pkt_write_customer_to_drver, 0, total_length);
        __gzd_node->pkt_write_customer_to_drver->ver = 0;
        __gzd_node->pkt_write_customer_to_drver->seg = 1;
        __gzd_node->pkt_write_customer_to_drver->off = __gzd_node->to_driver.driver_address;
        __gzd_node->pkt_write_customer_to_drver->len = __gzd_node->to_driver.total_length;
    }

    /* 动态数据 */
    pkt_offset = 0;
    __gzd_node->pkt_write_customer_to_drver->id = posix__atomic_inc(&__atomic_gzdid);
    for (i = 0; i < __gzd_node->to_driver.mapping_count; i++) {
        p_pkt_data = (char *) __gzd_node->pkt_write_customer_to_drver->dat;
        objhld_t hld = var__getobj_handle_byid(__gzd_node->to_driver.mapping[i].var_id);
        if (hld > 0) {
            obj = (var__functional_object_t *) objrefr(hld);
            if (obj) {
                p_var = (char *) obj->body_;
                var__acquire_lock(obj);

                // 内存访问越界保护
                if (__gzd_node->to_driver.mapping[i].var_offset + __gzd_node->to_driver.mapping[i].length > obj->body_length_) {
                    log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                            "common read request memory error, offset=%u, length=%u, body=%u", __gzd_node->to_driver.mapping[i].var_offset, __gzd_node->to_driver.mapping[i].length, obj->body_length_);
                } else {
                    memcpy(p_pkt_data + pkt_offset, p_var + __gzd_node->to_driver.mapping[i].var_offset, __gzd_node->to_driver.mapping[i].length);
                }

                var__release_lock(obj);
                objdefr(hld);
            }
        }
        pkt_offset += __gzd_node->to_driver.mapping[i].length;
    }

    return private_udp_write((const unsigned char *) __gzd_node->pkt_write_customer_to_drver, __gzd_node->pkt_write_customer_to_drver->len + sizeof (struct vcu_protocol_head));
}

static
void *nspi_vcu_timerproc(void *p) {
    static const uint32_t interval = 15; // 和windows的控制节奏相同， 15毫秒一个IO包
    struct vcu_protocol_head *pkt_command;
    struct vcu_common_command *command;
    var__vehicle_t *veh;
    var__dio_t *dio;

    pkt_command = (struct vcu_protocol_head *) malloc(sizeof (struct vcu_protocol_head) + sizeof (struct vcu_common_command));
    if (!pkt_command) {
        return NULL;
    }
    memset(pkt_command, 0, sizeof (struct vcu_protocol_head) + sizeof (struct vcu_common_command));

    pkt_command->ver = 0;
    pkt_command->seg = 1;
    pkt_command->off = 0;
    pkt_command->len = sizeof (struct vcu_common_command);
    command = (struct vcu_common_command *) &pkt_command->dat;

    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "VCU timer procedure startup.");

    while (1) {
        // 内置系统调用直接休眠
        posix__waitfor_waitable_handle(&__waiter, interval);

        // 通用包ID维护自增
        pkt_command->id = posix__atomic_inc(&__atomic_gzdid);

        // 保证任何对象读取错误的情况下， 下发基准为0
        memset(command, 0, sizeof (struct vcu_common_command));

        // 从内部DIO读取命令信号
        dio = var__get_dio_byid(kVarFixedObject_InternalDIO);
        if (dio) {
            // do 的8位数据都作为参考数据使用
            command->mask = (uint8_t)dio->do_;
            var__release_object_reference(dio);
        } else {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "can not found dio object kVarFixedObject_InternalDIO");
        }

        // 从底盘获取enable状态和和速度
        veh = var__get_vehicle();
        if (veh) {
            if (veh->enable_ & 1) {
                command->mask |= 1;
            } else {
                command->mask &= ~1;
            }

            // if need to clear fault, set bit 8-15 to 1
            if ( veh->fault_stop_ & VEH_MANUAL_CLEAR) {
                veh->fault_stop_ |= VEH_HARDWARE_HANDLEING;
                veh->fault_stop_ &= ~VEH_MANUAL_CLEAR;
                command->mask |= 0xff00;
                log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "mark command mask to clear VCU error");
            }

            int tmp = veh->i.command_velocity_.x_ * 1000;
            command->x = (uint16_t) tmp;
            tmp = veh->i.command_velocity_.y_ * 1000;
            command->y = (uint16_t) tmp;
            tmp = veh->i.command_velocity_.w_ * 1000;
            command->angular_velocity = (uint16_t) tmp;

            var__release_object_reference(veh);
        }

        private_udp_write(pkt_command, sizeof (struct vcu_protocol_head) + sizeof (struct vcu_common_command));

        // 投递自定义包
        nspi__write_customer();
    }

    free(pkt_command);
    nsp__uninit_gzd_object();
    
    return NULL;
}

static
int mnti__fill_binding(xmlNode *node) {
    struct vcu_exchange_mapping *mnt_node;
    xmlAttr *attr;
    const char *sz;
    xmlNode *next;

    attr = node->properties;
    mnt_node = NULL;
    while (attr) {
        if (0 == xmlStrcmp(attr->name, BAD_CAST"from")) {
            sz = get_first_text_properties(attr);
            if (sz) {
                mnt_node = &__gzd_node->to_var;
                memset(mnt_node, 0, sizeof (struct vcu_exchange_mapping));
                mnt_node->driver_address = strtoul(sz, NULL, 10);
            }
        } else if (0 == xmlStrcmp(attr->name, BAD_CAST"to")) {
            sz = get_first_text_properties(attr);
            if (sz) {
                mnt_node = &__gzd_node->to_driver;
                memset(mnt_node, 0, sizeof (struct vcu_exchange_mapping));
                mnt_node->driver_address = strtoul(sz, NULL, 10);
            }
        } else if (0 == xmlStrcmp(attr->name, BAD_CAST"init")) {
            sz = get_first_text_properties(attr);
            if (sz) {
                mnt_node = &__gzd_node->to_init;
                memset(mnt_node, 0, sizeof (struct vcu_exchange_mapping));
                mnt_node->driver_address = strtoul(sz, NULL, 10);
            }
        } else {
            ;
        }
        attr = attr->next;
    }

    if (!mnt_node) {
        return -1;
    }

    next = node->children;
    while (next) {
        if (0 == xmlStrcmp(next->name, BAD_CAST"sub")) {
            attr = next->properties;
            while (attr) {
                if (0 == xmlStrcmp(attr->name, BAD_CAST"id")) {
                    sz = get_first_text_properties(attr);
                    if (sz) {
                        mnt_node->mapping[mnt_node->mapping_count].var_id = strtoul(sz, NULL, 10);
                    }
                } else if (0 == xmlStrcmp(attr->name, BAD_CAST"offset")) {
                    sz = get_first_text_properties(attr);
                    if (sz) {
                        mnt_node->mapping[mnt_node->mapping_count].var_offset = strtoul(sz, NULL, 10);
                    }
                } else if (0 == xmlStrcmp(attr->name, BAD_CAST"len")) {
                    sz = get_first_text_properties(attr);
                    if (sz) {
                        mnt_node->mapping[mnt_node->mapping_count].length = strtoul(sz, NULL, 10);
                    }
                } else {
                    ;
                }

                attr = attr->next;
            }
            mnt_node->total_length += mnt_node->mapping[mnt_node->mapping_count].length;
            mnt_node->mapping_count++;
        }
        next = next->next;
    }

    return 0;
}

static
int nspi_load_vcu_settings() {
    xmlDocPtr doc = NULL;
    xmlNodePtr root = NULL;
    xmlNodePtr element = NULL;
    char xml_path[MAXPATH];
    const char *pedir;

    pedir = posix__getpedir();
    if (!pedir) {
        return -1;
    }
    posix__sprintf(xml_path, cchof(xml_path), "/etc/agv/mnt.xml", pedir);
    if (NULL == (doc = xmlReadFile(xml_path, "GB2312", 256))) {
        return -1;
    }

    if (NULL == (root = xmlDocGetRootElement(doc))) {
        return -1;
    }

    element = root->xmlChildrenNode;
    while (element) {
        if (0 == xmlStrcmp(BAD_CAST"node", element->name)) {
            mnti__fill_binding(element);
        }
        element = element->next;
    }

    xmlFreeDoc(doc);
    return 0;
}

static
int nspi_load_vehicle_config_settings() {
    int i;

    /* 确定全部配置信息的长度 */
    uint32_t length = 0;
    length += sizeof (struct vcu_vehicle_config);

    /* 驱动单元个数 */
    int count;
    const struct list_head *drive_unit_head = var__get_driveunit(&count);
    length += count * sizeof (struct vcu_driveunit_config);

    /* 遍历各个驱动单元的轮子，确认最终大小 */
    struct list_head *pos;

    list_for_each(pos, drive_unit_head) {
        var__functional_object_t *object;
        object = containing_record(pos, var__functional_object_t, local_);
        var__drive_unit_t *driver_unit;
        driver_unit = (var__drive_unit_t *) object->body_;
        switch (driver_unit->unit_type_) {
            case kOnlySWheel:
                length += sizeof (struct vcu_swheel_config);
                break;
            case kOnlyDWheel:
                length += sizeof (struct vcu_dwheel_config);
                break;
            case kSDWheel:
                length += sizeof (struct vcu_swheel_config);
                length += sizeof (struct vcu_dwheel_config);
                break;
            case kSDDWheel:
                length += sizeof (struct vcu_swheel_config);
                length += sizeof (struct vcu_dwheel_config);
                length += sizeof (struct vcu_dwheel_config);
                length += sizeof (struct vcu_sdd_config);
                break;
            default:
                break;
        }
    }

    /* 可以分配配置信息最终缓冲区 */
    __gzd_node->vehicle_config_size = length;
    if (__gzd_node->vehicle_config) {
        free(__gzd_node->vehicle_config);
    }
    __gzd_node->vehicle_config = (struct vcu_vehicle_config *) malloc(length);
    if (!__gzd_node->vehicle_config) {
        return -ENOMEM;
    }

    /* 填充底盘部分配置 */
    var__vehicle_t *veh = var__get_vehicle();
    __gzd_node->vehicle_config->chassis_type = veh->chassis_type_;
    __gzd_node->vehicle_config->max_speed = veh->max_speed_ * 1000;
    __gzd_node->vehicle_config->creep_speed = veh->creep_speed_ * 1000;
    __gzd_node->vehicle_config->max_acc = veh->max_acc_ * 1000;
    __gzd_node->vehicle_config->max_dec = veh->max_dec_ * 1000;
    __gzd_node->vehicle_config->max_w = veh->max_w_ * 1000;
    __gzd_node->vehicle_config->creep_w = veh->creep_w_ * 1000;
    __gzd_node->vehicle_config->max_acc_w = veh->max_acc_w_ * 1000;
    __gzd_node->vehicle_config->max_dec_w = veh->max_dec_w_ * 1000;
    __gzd_node->vehicle_config->steer_angle_error = veh->steer_angle_error_tolerance_ * 1000;
    __gzd_node->vehicle_config->count_of_drive_unit = count;
    var__release_object_reference(veh);
    log__save("motion_template", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "load config vehicle for VCU completed.");

    /* 填充驱动单元配置 */
    char *drive_unit_address = __gzd_node->vehicle_config->units;

    list_for_each(pos, drive_unit_head) {
        var__functional_object_t *object;
        object = containing_record(pos, var__functional_object_t, local_);
        var__drive_unit_t *drive_unit;
        drive_unit = (var__drive_unit_t *) object->body_;

        struct vcu_driveunit_config *drive_unit_config = (struct vcu_driveunit_config *) drive_unit_address;
        drive_unit_config->drive_unit_type = drive_unit->unit_type_;
        drive_unit_config->install_x = drive_unit->install_.x_ * 1000;
        drive_unit_config->install_y = drive_unit->install_.y_ * 1000;
        drive_unit_config->install_theta = drive_unit->install_.w_ * 1000;
        drive_unit_config->calibrated_x = drive_unit->calibrated_.x_ * 1000;
        drive_unit_config->calibrated_y = drive_unit->calibrated_.y_ * 1000;
        drive_unit_config->calibrated_theta = drive_unit->calibrated_.w_ * 1000;
        drive_unit_config->count_of_wheel = drive_unit->associated_cnt_;
        drive_unit_address += sizeof (struct vcu_driveunit_config);
        log__save("motion_template", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "load config drive unit %d for VCU completed.", object->object_id_);

        /* 填充轮子配置 */
        char *internel_address = drive_unit_config->wheels;
        var__functional_object_t **funcs = (var__functional_object_t **) malloc(sizeof (void *) * drive_unit->associated_cnt_);
        var__driveunit_parse_to_functional(drive_unit, (void **) funcs, NULL);
        for (i = 0; i < drive_unit->associated_cnt_; i++) {
            switch (funcs[i]->type_) {
                case kVarType_SWheel:
                {
                    struct vcu_swheel_config *swheel_config = (struct vcu_swheel_config *) internel_address;
                    var__swheel_t *swheel = (var__swheel_t *) funcs[i]->body_;
                    swheel_config->object_type = 'S';
                    swheel_config->min_angle = swheel->min_angle_ * 1000;
                    swheel_config->max_angle = swheel->max_angle_ * 1000;
                    swheel_config->zero_angle = swheel->zero_angle_ * 1000;
                    swheel_config->zero_angle_enc = swheel->zero_angle_enc_ * 1000;
                    swheel_config->max_w = swheel->max_w_ * 1000;
                    swheel_config->control_mode = swheel->control_mode_;
                    swheel_config->scale_control = swheel->scale_control_ * 1000 * 1000;
                    swheel_config->scale_feedback = swheel->scale_feedback_ * 1000 * 1000;
                    swheel_config->control_cp = swheel->control_cp_ * 1000;
                    swheel_config->control_ci = swheel->control_ci_ * 1000;
                    swheel_config->control_cd = swheel->control_cd_ * 1000;
                    internel_address += sizeof (struct vcu_swheel_config);
                    drive_unit_address += sizeof (struct vcu_swheel_config);
                    log__save("motion_template", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "load config s-wheel %d for VCU completed.", funcs[i]->object_id_);
                }
                    break;
                case kVarType_DWheel:
                {
                    struct vcu_dwheel_config *dwheel_config = (struct vcu_dwheel_config *) internel_address;
                    var__dwheel_t *dwheel = (var__dwheel_t *) funcs[i]->body_;
                    dwheel_config->object_type = 'D';
                    dwheel_config->max_speed = dwheel->max_speed_ * 1000;
                    dwheel_config->max_acc = dwheel->max_acc_ * 1000;
                    dwheel_config->max_dec = dwheel->max_dec_ * 1000;
                    dwheel_config->control_mode = dwheel->control_mode_;
                    dwheel_config->scale_control = dwheel->scale_control_ * 1000 * 1000;
                    dwheel_config->scale_feedback = dwheel->scale_feedback_ * 1000 * 1000;
                    dwheel_config->roll_weight = dwheel->roll_weight_ * 1000;
                    ;
                    dwheel_config->slide_weight = dwheel->slide_weight_ * 1000;
                    ;
                    internel_address += sizeof (struct vcu_dwheel_config);
                    drive_unit_address += sizeof (struct vcu_dwheel_config);
                    log__save("motion_template", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "load config d-wheel %d for VCU completed.", funcs[i]->object_id_);
                }
                    break;
                case kVarType_SDDExtra:
                {
                    struct vcu_sdd_config *sdd_config = (struct vcu_sdd_config *) internel_address;
                    var__sdd_extra_t *sdd = (var__sdd_extra_t *) funcs[i]->body_;
                    sdd_config->object_type = 'X';
                    sdd_config->gauge = sdd->gauge_ * 1000;
                    internel_address += sizeof (struct vcu_sdd_config);
                    drive_unit_address += sizeof (struct vcu_sdd_config);
                    log__save("motion_template", kLogLevel_Info, kLogTarget_Stdout | kLogTarget_Filesystem, "load config sdd-ex %d for VCU completed.", funcs[i]->object_id_);
                }
                    break;
                default:
                    break;
            }
        }
        for (i = 0; i < drive_unit->associated_cnt_; i++) {
            var__release_wheel_dup(funcs[i]);
        }
        free(funcs);
    }

    return 0;
}

static
int nspi_load_vehicle_misc_settings() {
    xmlDocPtr doc = NULL;
    xmlNodePtr root = NULL;
    xmlNodePtr element = NULL;
    char xml_path[MAXPATH];
    xmlAttr *attr;
    const char *sz;
    struct vcu_vehicle_misc_config *misc_node = NULL;
    int node_len = 0;
    char t_object_data[16];
    memset(t_object_data, 0, 16);
    
    posix__sprintf(xml_path, cchof(xml_path), "/etc/agv/misc.xml");
    if (NULL == (doc = xmlReadFile(xml_path, "UTF-8", 256))) {
        if (NULL == (doc = xmlReadFile(xml_path, "GB2312", 256))) {
            return -1;
        }
    }

    if (NULL == (root = xmlDocGetRootElement(doc))) {
        return -1;
    }

    element = root->xmlChildrenNode;
    while (element) {
        //获取节点总长度
        if (0 == xmlStrcmp(BAD_CAST"node", element->name)) {
            if(++node_len >= GZD_MAX_MISC_NUM) break;
        }
        element = element->next;
    }
    
    __gzd_vehicle_misc_config->vehicle_misc_config = (struct vcu_vehicle_misc_config*) malloc( node_len * sizeof(struct vcu_vehicle_misc_config));
    if (!__gzd_vehicle_misc_config->vehicle_misc_config) {
        return -ENOMEM;
    }
    __gzd_vehicle_misc_config->vehicle_misc_config_size = node_len * sizeof(struct vcu_vehicle_misc_config);
    memset(__gzd_vehicle_misc_config->vehicle_misc_config, 0, __gzd_vehicle_misc_config->vehicle_misc_config_size);
    misc_node = __gzd_vehicle_misc_config->vehicle_misc_config;
    
    element = root->xmlChildrenNode;
    while ( element ) {
        if (0 == xmlStrcmp(BAD_CAST"node", element->name)) {
            attr = element->properties;
            while (attr) {
                if (0 == xmlStrcmp(attr->name, BAD_CAST"name")) {
                    sz = get_first_text_properties(attr);
                    if (sz) {
                        strncpy(misc_node->object_name, sz, 15);
                    }
                } else if (0 == xmlStrcmp(attr->name, BAD_CAST"type")) {
                    sz = get_first_text_properties(attr);
                    if (sz) {
                        if ( strcmp(sz, "string") == 0 || strcmp(sz, "String") == 0 || strcmp(sz, "STRING") == 0 ) {
							misc_node->object_type = 0x00;
						} else if ( strcmp(sz, "U8") == 0 || strcmp(sz, "u8") == 0 ) {
							misc_node->object_type = 0x01;
						} else if ( strcmp(sz, "U16") == 0 || strcmp(sz, "u16") == 0 ) {
							misc_node->object_type = 0x02;
						} else if ( strcmp(sz, "U32") == 0 || strcmp(sz, "u32") == 0 ) {
							misc_node->object_type = 0x04;
						} else if ( strcmp(sz, "U64") == 0 || strcmp(sz, "u64") == 0 ) {
							misc_node->object_type = 0x08;
						} else if ( strcmp(sz, "s8") == 0 || strcmp(sz, "S8") == 0 ) {
							misc_node->object_type = 0x11;
						} else if ( strcmp(sz, "s16") == 0 || strcmp(sz, "S16") == 0 ) {
							misc_node->object_type = 0x12;
						} else if ( strcmp(sz, "s32") == 0 || strcmp(sz, "S32") == 0 ) {
							misc_node->object_type = 0x14;
						} else if ( strcmp(sz, "s64") == 0 || strcmp(sz, "S64") == 0 ) {
							misc_node->object_type = 0x18;
						} else if ( strcmp(sz, "float") == 0 || strcmp(sz, "Float") == 0 || strcmp(sz, "FLOAT") == 0 ) {
							misc_node->object_type = 0x24;
						} else if ( strcmp(sz, "double") == 0 || strcmp(sz, "Double") == 0 || strcmp(sz, "DOUBLE") == 0 ) {
							misc_node->object_type = 0x28;
						} else {
							log__save("motion_template", kLogLevel_Error, kLogTarget_Stdout | kLogTarget_Filesystem, "Unsupport type[%s].", sz);
						}
                    }
                } else if (0 == xmlStrcmp(attr->name, BAD_CAST"data")) {
                    sz = get_first_text_properties(attr);
                    if (sz) {
                        posix__strncpy(t_object_data, cchof(t_object_data), sz, 15);
                    }
                } else {
                    ;
                }
                attr = attr->next;
            }
			
			if(0 == misc_node->object_type) {
				memcpy(misc_node->object_data, t_object_data, 8);
			} else if (misc_node->object_type > 0 && misc_node->object_type <= 8) {
				uint64_t uint64_data = strtoull(t_object_data, NULL, sizeof(uint64_data));
				memcpy(misc_node->object_data, &uint64_data, 8);
			} else if (misc_node->object_type > 8 && misc_node->object_type <= 24) {
				int64_t int64_data = strtoll(t_object_data, NULL, 10);
				memcpy(misc_node->object_data, &int64_data, sizeof(int64_data));
			} else if (misc_node->object_type == 36) {
				float flo_data = strtof(t_object_data, NULL);
				memcpy(misc_node->object_data, &flo_data, sizeof(flo_data));
			} else if (misc_node->object_type == 40) {
				double dou_data = strtod(t_object_data, NULL);
				memcpy(misc_node->object_data, &dou_data, sizeof(dou_data));
			}
        
            misc_node = (struct vcu_vehicle_misc_config*) ( (char*)misc_node + sizeof(struct vcu_vehicle_misc_config) );
        }
        
        element = element->next;
    }

    xmlFreeDoc(doc);
    return 0;
}

static
int nspi_query_vcu_vehicle_config() {
    int retval = 0;

    if (INVALID_HUDPLINK == __gzd_node->link ) {
        return -EINVAL;
    }

    struct vcu_command *query_vcu_vehicle_config = (struct vcu_command *) malloc(sizeof (struct vcu_command) );
    if (!query_vcu_vehicle_config) {
        return -ENOMEM;
    }
    memset(query_vcu_vehicle_config, 0, sizeof (struct vcu_command) );
    query_vcu_vehicle_config->sequence = (uint16_t)++__gzd_node->vcu_pktid;
    query_vcu_vehicle_config->version = 0;
    query_vcu_vehicle_config->reserve1 = 0;
    query_vcu_vehicle_config->command = 6;
    query_vcu_vehicle_config->reserve2[0] = query_vcu_vehicle_config->reserve2[1] = query_vcu_vehicle_config->reserve2[2] = 0;
    query_vcu_vehicle_config->op = 5;
    query_vcu_vehicle_config->subop = SUBOP_READ_VEHICLE_CONFIG_DATA;
    query_vcu_vehicle_config->par = 0;
    query_vcu_vehicle_config->length = 0;
    
    if (udp_write(__gzd_node->link, sizeof (struct vcu_command), &nsp__packet_maker, 
            query_vcu_vehicle_config, GZD_UDP_TARGET, GZD_DEVICE_CONFIG_PORT) < 0) {
        log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "nspi_query_vcu_vehicle_config failed.");
        retval = -1;
    }
    private_udp_write(query_vcu_vehicle_config, sizeof (struct vcu_command));
    free(query_vcu_vehicle_config);

    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "read vcu config...");
    return retval;
}

static
int nspi_send_vehicle_config() {
    int retval = 0;

    if (INVALID_HUDPLINK == __gzd_node->link || !__gzd_node->vehicle_config || __gzd_node->vehicle_config_size <= 0) {
        return -EINVAL;
    }

    struct vcu_command *vcu_vehicle_config = (struct vcu_command *) malloc(sizeof (struct vcu_command) +__gzd_node->vehicle_config_size);
    if (!vcu_vehicle_config) {
        return -ENOMEM;
    }
    memset(vcu_vehicle_config, 0, sizeof (struct vcu_command) +__gzd_node->vehicle_config_size);
    vcu_vehicle_config->sequence = (uint16_t)++__gzd_node->vcu_pktid;
    vcu_vehicle_config->version = 0;
    vcu_vehicle_config->reserve1 = 0;
    vcu_vehicle_config->command = 6;
    vcu_vehicle_config->reserve2[0] = vcu_vehicle_config->reserve2[1] = vcu_vehicle_config->reserve2[2] = 0;
    vcu_vehicle_config->op = 5;
    vcu_vehicle_config->subop = SUBOP_WRITE_VEHICLE_CONFIG_DATA;
    vcu_vehicle_config->par = 0;
    vcu_vehicle_config->length = __gzd_node->vehicle_config_size;
    memcpy(vcu_vehicle_config->data, __gzd_node->vehicle_config, __gzd_node->vehicle_config_size);
    if (udp_write(__gzd_node->link, sizeof (struct vcu_command) +__gzd_node->vehicle_config_size,
            &nsp__packet_maker, vcu_vehicle_config, GZD_UDP_TARGET, GZD_DEVICE_CONFIG_PORT) < 0) {
        log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "nspi_send_vehicle_config failed.");
        retval = -1;
    }
    private_udp_write(vcu_vehicle_config, sizeof (struct vcu_command) +__gzd_node->vehicle_config_size);
    free(vcu_vehicle_config);

    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "write vcu config...");
    return retval;
}

static
int nspi_query_vcu_vehicle_misc() {
    int retval = 0;

    if (INVALID_HUDPLINK == __gzd_node->link ) {
        return -EINVAL;
    }

    struct vcu_command *query_vcu_vehicle_misc_config = (struct vcu_command *) malloc(sizeof (struct vcu_command) );
    if (!query_vcu_vehicle_misc_config) {
        return -ENOMEM;
    }
    memset(query_vcu_vehicle_misc_config, 0, sizeof (struct vcu_command) );
    query_vcu_vehicle_misc_config->sequence = (uint16_t)++__gzd_node->vcu_pktid;
    query_vcu_vehicle_misc_config->version = 0;
    query_vcu_vehicle_misc_config->reserve1 = 0;
    query_vcu_vehicle_misc_config->command = 6;
    query_vcu_vehicle_misc_config->reserve2[0] = query_vcu_vehicle_misc_config->reserve2[1] = query_vcu_vehicle_misc_config->reserve2[2] = 0;
    query_vcu_vehicle_misc_config->op = 5;
    query_vcu_vehicle_misc_config->subop = SUBOP_READ_VEHICLE_CONFIG_MISC;
    query_vcu_vehicle_misc_config->par = 0;
    query_vcu_vehicle_misc_config->length = 0;
    
    if (udp_write(__gzd_node->link, sizeof (struct vcu_command),
            &nsp__packet_maker, query_vcu_vehicle_misc_config, GZD_UDP_TARGET, GZD_DEVICE_CONFIG_PORT) < 0) {
        log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "nspi_query_vcu_vehicle_misc failed.");
        retval = -1;
    }
    private_udp_write(query_vcu_vehicle_misc_config, sizeof (struct vcu_command) );
    free(query_vcu_vehicle_misc_config);

    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "read vcu misc config...");
    return retval;
}

static
int nspi_send_vehicle_misc() {
    int retval = 0;

    if (INVALID_HUDPLINK == __gzd_node->link ) {
        return -EINVAL;
    }

    struct vcu_command *vcu_vehicle_misc_config = (struct vcu_command *) malloc(sizeof (struct vcu_command) +__gzd_vehicle_misc_config->vehicle_misc_config_size);
    if (!vcu_vehicle_misc_config) {
        return -ENOMEM;
    }
    memset(vcu_vehicle_misc_config, 0, sizeof (struct vcu_command) +__gzd_vehicle_misc_config->vehicle_misc_config_size);
    vcu_vehicle_misc_config->sequence = (uint16_t)++__gzd_node->vcu_pktid;
    vcu_vehicle_misc_config->version = 0;
    vcu_vehicle_misc_config->reserve1 = 0;
    vcu_vehicle_misc_config->command = 6;
    vcu_vehicle_misc_config->reserve2[0] = vcu_vehicle_misc_config->reserve2[1] = vcu_vehicle_misc_config->reserve2[2] = 0;
    vcu_vehicle_misc_config->op = 5;
    vcu_vehicle_misc_config->subop = SUBOP_WRITE_VEHICLE_CONFIG_MISC;
    vcu_vehicle_misc_config->par = 0;
    vcu_vehicle_misc_config->length = __gzd_vehicle_misc_config->vehicle_misc_config_size;
    if(__gzd_vehicle_misc_config->vehicle_misc_config_size > 0 && __gzd_vehicle_misc_config->vehicle_misc_config) {
        memcpy(vcu_vehicle_misc_config->data, __gzd_vehicle_misc_config->vehicle_misc_config, __gzd_vehicle_misc_config->vehicle_misc_config_size);
    }
    if (udp_write(__gzd_node->link, sizeof (struct vcu_command) +__gzd_vehicle_misc_config->vehicle_misc_config_size,
            &nsp__packet_maker, vcu_vehicle_misc_config, GZD_UDP_TARGET, GZD_DEVICE_CONFIG_PORT) < 0) {
        log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "nspi_send_vehicle_misc failed.");
        retval = -1;
    }
    private_udp_write(vcu_vehicle_misc_config, sizeof (struct vcu_command) +__gzd_vehicle_misc_config->vehicle_misc_config_size);
    free(vcu_vehicle_misc_config);

    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "write vcu misc config...");
    return retval;
}

static
void STD_CALL nspi_vcu_udp_callback(const nis_event_t *naio_event, const void *pParam2) {
    udp_data_t *udp_data = (udp_data_t *) pParam2;

    switch (naio_event->Event) {
        case EVT_RECEIVEDATA:
        {
            if (udp_data->e.Packet.Size < sizeof (struct vcu_command)) {
                return;
            }
            struct vcu_response *response = (struct vcu_response *) udp_data->e.Packet.Data;
            if (response->ack != 0) {
                log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                    "vcu response error ack[%d], subop [%d].", response->ack, response->subop);
                    return;
            }
            
            if (134 == response->command && 5 == response->op) {
                switch (response->subop) {
                    case SUBOP_QUERY_VEHICLE_CONFIG_DATA:// VCU 请求底盘数据
                        nspi_send_vehicle_config();
                        break;
                    case SUBOP_WRITE_VEHICLE_CONFIG_DATA:// 写底盘数据的VCU应答
                        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "write vcu config successful.");
                        __keep_loop = 0;
                        break;
                    case SUBOP_READ_VEHICLE_CONFIG_DATA:
                        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                            "vcu ack read config data length [%d], storage data len [%d].", response->length, __gzd_node->vehicle_config_size);
                        if (response->length != __gzd_node->vehicle_config_size
                            || 0 != memcmp(__gzd_node->vehicle_config, response->data, response->length) ) {
                                nspi_send_vehicle_config();
                        } else {
                            __keep_loop = 0;
                        }
                        break;
                        
                    case SUBOP_QUERY_VEHICLE_CONFIG_MISC: // VCU 请求底盘杂项
                        nspi_send_vehicle_misc();
                        break;
                    case SUBOP_WRITE_VEHICLE_CONFIG_MISC:
                        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "write vcu misc config successful.");
                        __keep_loop = 0;
                        break;
                    case SUBOP_READ_VEHICLE_CONFIG_MISC:
                        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                            "vcu ack read misc config data length [%d], storage data len [%d].", response->length, __gzd_vehicle_misc_config->vehicle_misc_config_size);
                        if (response->length != __gzd_vehicle_misc_config->vehicle_misc_config_size
                            || 0 != memcmp(__gzd_vehicle_misc_config->vehicle_misc_config, response->data, response->length) ) {
                                nspi_send_vehicle_misc();
                        } else {
                            __keep_loop = 0;
                        }
                        break;
                    default:
                        break;
                }
                return;
            }
            break;
        }
        default:
            break;
    }
}

void nsp__uninit_gzd_object() {
    if (__gzd_node) {
        if(__gzd_node->vehicle_config) {
            free(__gzd_node->vehicle_config);
            __gzd_node->vehicle_config = NULL;
        }

        posix__uninit_waitable_handle(&__init_completed);
        free(__gzd_node);
        __gzd_node = NULL;
    }
    
    if (__gzd_vehicle_misc_config) {
        if(__gzd_vehicle_misc_config->vehicle_misc_config) {
            free(__gzd_vehicle_misc_config->vehicle_misc_config);
            __gzd_vehicle_misc_config->vehicle_misc_config = NULL;
        }

        free(__gzd_vehicle_misc_config);
        __gzd_vehicle_misc_config = NULL;
    }
    
    posix__uninit_waitable_handle(&__waiter);
}

int nsp__init_gzd_object() {
    if (__gzd_node) {
        return 0;
    }

    __gzd_node = (struct vcu_udp_object *) malloc(sizeof ( struct vcu_udp_object));
    if (!__gzd_node) {
        return -1;
    }
    memset(__gzd_node, 0, sizeof ( struct vcu_udp_object));
    
    __gzd_vehicle_misc_config = (struct vcu_vehicle_misc_object*) malloc( sizeof(struct vcu_vehicle_misc_object));
    if (!__gzd_vehicle_misc_config) {
        return -1;
    }
    memset(__gzd_vehicle_misc_config, 0, sizeof(struct vcu_vehicle_misc_object));
    
    if (posix__init_synchronous_waitable_handle(&__waiter) < 0) {
        return -1;
    }
    if (posix__init_synchronous_waitable_handle(&__init_completed) < 0) {
        posix__uninit_waitable_handle(&__waiter);
        return -1;
    }

    // 读取底盘配置和杂项
    nspi_load_vehicle_config_settings();
    nspi_load_vehicle_misc_settings();
    
    // 创建底盘配置参数交互UDP对象
    __gzd_node->link = udp_create(&nspi_vcu_udp_callback, "0.0.0.0", GZD_DEVICE_CONFIG_PORT, 0);
    if (__gzd_node->link == INVALID_HUDPLINK) {
        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "failed to create UDP binding for vehicle config.");
    }else{
        // 查询底盘配置参数和底盘配置杂项是否与当前一致
        int loop_num = 100;
        while(__keep_loop && loop_num-- > 0) {
            nspi_query_vcu_vehicle_config();
            posix__waitfor_waitable_handle(&__waiter, 100);
        }
        
        __keep_loop = 1;
        loop_num = 100;
        while(__keep_loop && loop_num-- > 0) {
            nspi_query_vcu_vehicle_misc();
            posix__waitfor_waitable_handle(&__waiter, 100);
        }
    }
    
    // 读取自主驱动的内存映射
    nspi_load_vcu_settings();

    // 创建 private udp
    __gzd_node->local_port_ = GZD_UDP_PORT;
    posix__strcpy(__gzd_node->local_ipv4_, cchof(__gzd_node->local_ipv4_), GZD_UDP_LOCAL);
    private_udp_startup(GZD_UDP_TARGET, &nspi_vcu_callback);

    // 启动线程用于实时投递速度数据
    if (__gzd_node->to_init.total_length > 0 && __gzd_node->to_init.mapping_count > 0) {
        log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "waitting for VCU initialization report.");
        if (ETIMEDOUT == posix__waitfor_waitable_handle(&__init_completed, 5000) ) {
            log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "VCU initialization report. timedout");
        }
    }
    posix__pthread_create(&__gzd_node->vcu_timer, &nspi_vcu_timerproc, &nspi_vcu_callback);
    
    return 0;
}
