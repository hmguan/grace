#if !defined DEF_OFFLINETASK_H
#define DEF_OFFLINETASK_H
#include "navigation.h"

#pragma pack(push ,1)

// 最大离线参考轨迹个数 
#define OFFLINE_MAX_TRAIL 500
// 最大离线操作个数 
#define OFFLINE_MAX_OPER  10
// 最大离线任务节点个数 
#define OFFLINE_MAX_NODE  5000

// 数组数据 
typedef struct var_offline_vector {
	int count_;
	char data[0];
} var_offline_vector_t;

// 离线操作 
typedef struct var_offline_oper {
	uint64_t task_id_;
	int  code_;
	uint64_t params_[10];
} var_offline_oper_t;

// 离线节点(导航和操作) 
typedef struct var_offline_task_node {
	uint64_t task_id_;
	upl_t dest_upl_;
	position_t dest_pos_;
	// 参考轨迹个数 
	int cnt_trails_;
	trail_t trails_[OFFLINE_MAX_TRAIL];
	// 离线操作个数 
	int cnt_opers_;
	var_offline_oper_t opers_[OFFLINE_MAX_OPER];
} var_offline_task_node_t;

typedef struct var_offline_task {
	// 当前离线导航状态 
	var__status_describe_t track_status_;

	// 离线导航任务ID
	uint64_t user_task_id_;
	uint64_t ato_task_id_;

	// 离线任务节点个数
	int task_count_;
	// 当前执行到的离线任务(0:初始值，未执行)
	int task_current_exec_index_;

	// 离线任务节点数据 
	var_offline_task_node_t tasks_[OFFLINE_MAX_NODE];
} var_offline_task_t;

#pragma pack(pop)

extern int var__load_offlinetask();
extern var_offline_task_t *var__get_offline_task();
extern int var__set_offline_task(const char *data, int cb);

#endif // DEF_OFFLINETASK_H
