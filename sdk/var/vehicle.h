#if !defined VAR_VEHICLE_H
#define VAR_VEHICLE_H

#define  ROTATEION  (1)
#define  TRANSLATION (2)

#include "vartypes.h"

enum vehicle_type_t {
    kVehicleAgv_Type_PickingAgv = 0,
    kVehicleAgv_Type_CartAgv,
    kVehicleAgv_Type_ForkAgv,
};

enum vehicle_control_mode_t {
    kVehicleControlMode_Navigation = 0,
    kVehicleControlMode_Manual,
    kVehicleControlMode_Calibrated, // 微调模式
};

#define VEH_HEALTHY             ((int)0) // vehicle status is healthy
#define VEH_SOFTWARE_FAULT      ((int)1) // this bit means any software running errors has been found
#define VEH_MANUAL_CLEAR        ((int)2) // calling thread or server program need and confirmed to cleanup all errors
#define VEH_HARDWARE_FAULT      ((int)4) // this bit means any errors has been detected on hardware.
#define VEH_HARDWARE_HANDLEING  ((int)8) // hareware is running on clear-fault command


#pragma pack(push, 1)

struct var__vehicle_config_t {
    /*+000*/ int vehicle_id_; // 机器人ID
    /*+004*/ char vehicle_name_[VAR_NAME_LENGTH]; // 机器人名字
    /*+004*/ enum vehicle_type_t vehicle_type_; // 车辆类型
    /*+012*/ char chassis_type_; // 底盘类型, 0位表示能否旋转， 1位表示能否平移
    /*+013*/ double max_speed_; // 底盘最大速度
    /*+021*/ double creep_speed_; // 底盘最小速度（龟速)
    /*+029*/ double max_acc_; // 底盘最大加速度
    /*+037*/ double max_dec_; // 底盘最大减速度
    /*+045*/ double max_w_; // 最大角速度
    /*+053*/ double creep_w_; // 最小角速度（龟速）
    /*+061*/ double max_acc_w_; // 最大角加速度(需要约定单位, 弧度)
    /*+069*/ double max_dec_w_; // 最大角减速度       
    /*+077*/ double steer_angle_error_tolerance_; // 转向轮角度偏差容忍值
};

typedef struct _var__vehicle_t {
    // 全局只读
    /*+000*/ int vehicle_id_; // 机器人ID
    /*+004*/ char vehicle_name_[VAR_NAME_LENGTH]; // 机器人名字
    /*+004*/ enum vehicle_type_t vehicle_type_; // 车辆类型
    /*+012*/ char chassis_type_; // 底盘类型, 0位表示能否旋转， 1位表示能否平移
    /*+013*/ double max_speed_; // 底盘最大速度
    /*+021*/ double creep_speed_; // 底盘最小速度（龟速)
    /*+029*/ double max_acc_; // 底盘最大加速度
    /*+037*/ double max_dec_; // 底盘最大减速度
    /*+045*/ double max_w_; // 最大角速度
    /*+053*/ double creep_w_; // 最小角速度（龟速）
    /*+061*/ double max_acc_w_; // 最大角加速度(需要约定单位, 弧度)
    /*+069*/ double max_dec_w_; // 最大角减速度		
    /*+077*/ double steer_angle_error_tolerance_; // 转向轮角度偏差容忍值

    // 外部可写(数据下行,可作为绑定目标)
    /*+085*/ velocity_t manual_velocity_; // 遥控速度
    /*+109*/ int stop_normal_; // 普通停止,被设置后，车以正常减速度停车
    /*+113*/ int stop_emergency_; // 紧急停止,被设置后，车以紧急减速度停车
    /*+117*/ int fault_stop_; // 故障求停
    /*+121*/ int slow_down_; // 减速,被设置后，车以正常减速度减至慢速
    /*+125*/ int enable_; // 底盘使能，如果设为false，则需要将各个驱动单元disable
    /*+129*/ enum vehicle_control_mode_t control_mode_; // 启动手动遥控模式,导航模式 or 遥控模式

    // 内部可提交(数据上行， 可作为绑定源)
    union {
#if __cplusplus
        struct {
#else
        struct _i_vehicle_internal_t {
#endif 
            int enabled_; // 反馈使能状态
            /*+153*/ velocity_t command_velocity_; // 目标速度,导航算出来经过限制的
            /*+177*/ velocity_t ref_velocity_; // 参考速度,导航算出来的原始速度
            /*+201*/ velocity_t actual_command_velocity_; // 实际速度,导航算出来经过最详细处理的速度

            /*+205*/ velocity_t actual_velocity_; // 实际反馈速度
            /*+249*/ position_t odo_meter_; // 里程计( 导航写， 定位读 )
            /*+257*/ uint64_t time_stamp_; // 反馈（里程计、瞬时速度）的时间戳
            /*+261*/ int is_moving_; // 当前机器人是否在移动,通过驱动反馈判断，而非通过上面几个标志位判断

            /*+265*/ int normal_stopped_; // stop_normal_ 的上行数据
            /*+269*/ int emergency_stopped_; // stop_emergency_ 的上行数据
            /*+273*/ int slow_done_; // slow_down_ 的上行数据
            
            /*+277*/ double total_odo_meter_;    // 行驶总里程
        } i;
        
        char u_;
    };

} var__vehicle_t;

#pragma pack(pop)

#if !defined __cplusplus

extern
int var__load_vehicle();

/* 在线程锁保护下获取 vehicle 对象原始数据指针, 使用完成后手动调用 var__release_object_reference 释放
 */
extern
var__vehicle_t *var__get_vehicle();

/* 创建一份 vehicle 对象的拷贝， 使用完成后手动调用 var__release_vehicle_dup 释放
 */
extern
var__vehicle_t *var__create_vehicle_dup();

/* 将 vehicle 拷贝对象提交到原始变量区
 */
extern
int var__commit_vehicle_dup(const var__vehicle_t *veh);

/* 释放由 var__get_vehicle 成功调用所得的 vehicle 对象拷贝
 */
extern
void var__release_vehicle_dup(var__vehicle_t *dup);

#endif

#endif