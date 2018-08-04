#if !defined DEF_NAVIGATION_H
#define DEF_NAVIGATION_H

#include "vartypes.h"

#define MAXIMUM_TRAJ_REF_COUNT  (2000)       // 最大参考轨迹指定

#pragma pack(push, 1)

typedef struct _trail {
    int edge_id_;
    int wop_id_;
} trail_t;

struct var__navigation_config_t {
    double max_speed_; // 导航最大速度
    double creep_speed_; // 导航龟速,当距离目标点creep_distance时，发送龟速
    double max_w_; // 导航最大角速度
    double creep_w_; // 导航龟角速度
    double slow_down_speed_; // 慢速,当nav.slow_down被置位时的速度限制
    double acc_; // 导航加速度
    double dec_; // 导航减速度
    double dec_estop_; // 急停减速度,在急停时导航的减速度
    double acc_w_; // 最大角加速度
    double dec_w_; // 最大角减速度
    double creep_distance_; // 在离目标多远时发龟速
    double creep_theta_; // 在离目标角度多少度时发龟角速度
    double upl_mapping_angle_tolerance_; // 路径映射角度容忍偏差,用于计算当前的定位坐标属于哪一条边
    double upl_mapping_dist_tolerance_; // 路径映射距离容忍偏差
    double upl_mapping_angle_weight_; // 路径映射角度权重
    double upl_mapping_dist_weight_; // 路径映射角度权重
    double tracking_error_tolerance_dist_; // 导航轨迹跟踪的距离偏差容忍值
    double tracking_error_tolerance_angle_; // 导航轨迹跟踪的角度偏差容忍值
    double aim_dist_; // 预瞄距离
    float  predict_time_; // 预测时间(秒)
    uint32_t is_traj_whole_; // 路径是否完整,0不允许导航真正完成, 1允许完成(POC后， 这个字段将被独立出来)
    double aim_angle_p_; // 预瞄角度偏差的PID参数
    double aim_angle_i_;
    double aim_angle_d_;
    double stop_tolerance_; // 停止距离,当距离目标点多远的时候发送速度0
    double stop_tolerance_angle_; // 角度上的停止阈值
    double stop_point_trim_; // 停止点调整,对停止点进行前后修正的值，即改变目标点
    double aim_ey_p_; // 侧向偏差的PID参数
    double aim_ey_i_;
    double aim_ey_d_;
};

typedef struct _var__navigation_t {
    // 全局只读
    double max_speed_; // 导航最大速度
    double creep_speed_; // 导航龟速,当距离目标点creep_distance时，发送龟速
    double max_w_; // 导航最大角速度
    double creep_w_; // 导航龟角速度
    double slow_down_speed_; // 慢速,当nav.slow_down被置位时的速度限制
    double acc_; // 导航加速度
    double dec_; // 导航减速度
    double dec_estop_; // 急停减速度,在急停时导航的减速度
    double acc_w_; // 最大角加速度
    double dec_w_; // 最大角减速度
    double creep_distance_; // 在离目标多远时发龟速
    double creep_theta_; // 在离目标角度多少度时发龟角速度
    double upl_mapping_angle_tolerance_; // 路径映射角度容忍偏差,用于计算当前的定位坐标属于哪一条边
    double upl_mapping_dist_tolerance_; // 路径映射距离容忍偏差
    double upl_mapping_angle_weight_; // 路径映射角度权重
    double upl_mapping_dist_weight_; // 路径映射角度权重
    double tracking_error_tolerance_dist_; // 导航轨迹跟踪的距离偏差容忍值
    double tracking_error_tolerance_angle_; // 导航轨迹跟踪的角度偏差容忍值
    double aim_dist_; // 预瞄距离
    float  predict_time_; // 预测时间(秒)
    uint32_t is_traj_whole_; // 路径是否完整,0不允许导航真正完成, 1允许完成(POC后， 这个字段将被独立出来)
    double aim_angle_p_; // 预瞄角度偏差的PID参数
    double aim_angle_i_;
    double aim_angle_d_;
    double stop_tolerance_; // 停止距离,当距离目标点多远的时候发送速度0
    double stop_tolerance_angle_; // 角度上的停止阈值
    double stop_point_trim_; // 停止点调整,对停止点进行前后修正的值，即改变目标点
    double aim_ey_p_; // 侧向偏差的PID参数
    double aim_ey_i_;
    double aim_ey_d_;

    // 状态迁移
    var__status_describe_t track_status_; // 当前导航状态
    
    // 导航任务ID
    uint64_t user_task_id_;
    uint64_t ato_task_id_;

    // 外部可写(数据下行,可作为绑定目标)
    upl_t dest_upl_; // 目标upl_t
    position_t dest_pos_; // 目标Point,目标位置姿态，可以在目标upl_t基础上进行一定偏移
    var__vector_t traj_ref_; // 参考轨迹 trail_t []
    position_t pos_; // 当前定位坐标
    uint64_t pos_time_stamp_; // 当前定位时间戳
    uint32_t pos_status;        // 定位状态 
    
    double pos_confidence_; // 当前定位置信度

    // 提交(数据上行， 可作为绑定源)
    union {
#if __cplusplus
        struct {
#else
        struct _i_navigation_internal_t {
#endif 
            // 内部读写，外部只读
            int traj_ref_index_curr_; // 当前处于参考轨迹边的数组下标
            upl_t upl_; // 当前upl_t,导航程序维护

            // 常规提交
            int tracking_error_; // 是否触发了跟踪保护,导航程序置位，代表轨迹跟踪误差超限
            position_t base_point_; // 当前到轨迹最近点
            position_t aim_point_; // 预瞄点
            double aim_heading_error_; // 到预瞄点的航向偏差
            position_t predict_point_; // 预测点
            double predict_point_curvature_; // 预测点曲率
            int on_last_segment_; // 导航程序置位，代表已处于最后一条边上
            double dist_to_partition_; // 到下一个切分点的距离
            double dist_to_dest_; // 到终点的距离（路程），以base_point为基准计算，而不是机器人位置	 
            
            var__edge_wop_properties_t current_edge_wop_properties_;
            uint64_t current_task_id_; // current running task id
        } i;

        char u_;
    };
	
	double runtime_limiting_velocity_;	// 导航运行时速度限制
	
} var__navigation_t;

#pragma pack(pop)

#if !defined __cplusplus

/* 创建一份导航对象的拷贝，使用完成后调用 var__release_navigation_dup 释放
 */
extern
var__navigation_t *var__create_navigation_dup(int ignore_trajs);

/* 将[nav]指向的导航对象拷贝数据提交到原始变量区
        2016-12-02 参考轨迹一定不会提交到原始变量区
 */
extern
int var__commit_navigation_dup(const var__navigation_t *nav);

/* 释放由 var__create_navigation_dup 成功调用所得的导航结构拷贝
 */
extern
void var__release_navigation_dup(var__navigation_t *nav);

/* 在线程安全前提下，获取导航结构的原始数据区指针, 完成使用后手动调用 var__release_object_reference 释放
 */
extern
var__navigation_t *var__get_navigation();

extern
int var__load_navigation();

#endif
#endif