#if !defined VAR_WHEEL_H
#define VAR_WHEEL_H

#include "vartypes.h"
#include "object.h"

// 角度调整模式
enum swheel_ctrlmod_t {
    kSWheelCtrlMod_Angle = 0, // 目标角度
    kSWheelCtrlMod_DiscreteRate, // 离散变化率
    kSWheelCtrlMod_ContinueRate, // 连续变化率
};

enum dwheel_ctrlmod_t {
    kDWheelCtrlMod_Speed = 0, // 速度模式
    kDWheelCtrlMod_Dist, // 电流模式
    kDWheelCtrlMod_Position, // 位置模式
};

#pragma pack (push, 1)

struct var__swheel_config_t {
    /*+000*/ double min_angle_; // 行驶的最小角度         rad
    /*+008*/ double max_angle_; // 最大角度            rad
    /*+016*/ double zero_angle_; // 零偏                     rad
    /*+024*/ int64_t zero_angle_enc_; // 零偏对应的反馈编码值        counts
    /*+032*/ double max_w_; // 最大角速度                     rad/s
    /*+040*/ enum swheel_ctrlmod_t control_mode_; // 控制模式
    /*+044*/ double scale_control_; // 传动系数       counts*s/m(rad) *1000
    /*+052*/ double scale_feedback_; // 反馈系数      m(rad)/(s*counts)           *1000
    /*+060*/ double control_cp_; // 控制角度时所用的pid参数，控制PID（用于离散变化率控制或连续变化率控制）      *1000
    /*+068*/ double control_ci_;     //  *1000
    /*+076*/ double control_cd_;     //  *1000
};

typedef struct _var__swheel_t {
    // 全局只读
    /*+000*/ double min_angle_; // 行驶的最小角度         rad
    /*+008*/ double max_angle_; // 最大角度            rad
    /*+016*/ double zero_angle_; // 零偏                     rad
    /*+024*/ int64_t zero_angle_enc_; // 零偏对应的反馈编码值        counts
    /*+032*/ double max_w_; // 最大角速度                     rad/s
    /*+040*/ enum swheel_ctrlmod_t control_mode_; // 控制模式
    /*+044*/ double scale_control_; // 传动系数       counts*s/m(rad) *1000
    /*+052*/ double scale_feedback_; // 反馈系数      m(rad)/(s*counts)           *1000
    /*+060*/ double control_cp_; // 控制角度时所用的pid参数，控制PID（用于离散变化率控制或连续变化率控制）      *1000
    /*+068*/ double control_ci_;     //  *1000
    /*+076*/ double control_cd_;     //  *1000

    // 外部可写 (数据下行,可作为绑定目标)
    /*+084*/ int enabled_; // 使能状态
    /*+088*/ double actual_angle_; // 反馈角度
    /*+096*/ int64_t actual_angle_enc_; // 反馈角度(Scaled)
    /*+104*/ uint64_t time_stamp_; // 时间戳
    /*+112*/ int error_code_; // 错误码

    // 提交 (数据上行， 可作为绑定源)
    union {
#if __cplusplus
        struct {
#else
        struct _i_swheel_internal_t {
#endif
            /*+116*/ int enable_; // 使能
            /*+120*/ double command_angle_; // 目标角度
            /*+128*/ int64_t command_angle_enc_; // 目标角度scale
            /*+136*/ double command_rate_; // 目标变化率
            /*+144*/ double command_rate_enc_; // Scaled
        } i;

        char u_;
    };
} var__swheel_t;

struct var__dwheel_config_t {
    /*+000*/ double max_speed_;
    /*+008*/ double max_acc_; //最大加速度
    /*+016*/ double max_dec_; //最大减速度
    /*+024*/ enum dwheel_ctrlmod_t control_mode_;
    /*+028*/ double scale_control_; //传动系数，和反馈系数这两个值一般是一样的，由减速比，轮子直径，码盘线数等决定
    /*+036*/ double scale_feedback_; //反馈系数
    /*+044*/ double roll_weight_; //滚动约束权重          *1000
    /*+052*/ double slide_weight_; //侧滑约束权重                   *1000
};

typedef struct _var__dwheel_t {
    // 全局只读
    /*+000*/ double max_speed_;
    /*+008*/ double max_acc_; //最大加速度
    /*+016*/ double max_dec_; //最大减速度
    /*+024*/ enum dwheel_ctrlmod_t control_mode_;
    /*+028*/ double scale_control_; //传动系数，和反馈系数这两个值一般是一样的，由减速比，轮子直径，码盘线数等决定
    /*+036*/ double scale_feedback_; //反馈系数
    /*+044*/ double roll_weight_; //滚动约束权重          *1000
    /*+052*/ double slide_weight_; //侧滑约束权重                   *1000

    // 外部可写 (数据下行,可作为绑定目标)
    /*+060*/ int enabled_; //底盘反馈，当前使能状态，即使不主动设置，也可能因为急停信号连锁导致该状态变为disable
    /*+064*/ double actual_velocity_; //反馈速度
    /*+072*/ int64_t actual_velocity_enc_; //反馈速度（Scaled）
    /*+080*/ double actual_position_; //行进距离反馈
    /*+088*/ int64_t actual_position_enc_; //行进距离反馈（Scaled）
    /*+096*/ double actual_current_; //反馈电流
    /*+104*/ uint64_t time_stamp_; //时间戳
    /*+112*/ int error_code_; //错误码

    // 内部可提交
    union {
#if __cplusplus
        struct {
#else
        struct _i_dwheel_internal_t {
#endif
            /*+116*/ int enable_; //使enable或disable
            /*+120*/ double command_velocity_; //目标速度，m/s
            /*+128*/ int64_t command_velocity_enc_; //乘以传动系数以后，发给驱动的速度
            /*+136*/ double command_position_; //目标位置
            /*+144*/ int64_t command_position_enc_; //目标位置（Scaled）
            /*+152*/ double command_current_; //目标电流
        } i;

        char u_;
    };

} var__dwheel_t;

typedef struct __var__sdd_extra {
    /*+000*/ double gauge_; // 轮距  m    *1000
} var__sdd_extra_t;

#pragma pack(pop)

#if !defined __cplusplus

extern
int var__load_dwheel();
extern
int var__load_swheel();
extern
int var__load_sddext();

/*得到 var__functional_object_t *
  手动调用 var__release_object_reference 释放
 */
extern
void *var__get_wheel(objhld_t hld);
extern
void *var__get_wheel_byid(int id);

/*得到复制后的 var__functional_object_t*/
extern
void *var__create_wheel_dup(objhld_t hld);
extern
void *var__create_wheel_dup_byid(int id);

/* 将 var__create_wheel_dup或var__create_wheel_dup_byid所得对象提交到原始数据区
 */
extern
int var__commit_wheel_dup(const void *dup);

/* 释放 var__create_wheel_dup或var__create_wheel_dup_byid所得对象拷
 */
extern
void var__release_wheel_dup(void *dup);

#endif
#endif