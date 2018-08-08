#include "offlinetask.h"
#include "navigation.h"
#include "var.h"
#include "proto.h"
#include "logger.h"

//free heap memory when next incoming offline task  arrived
//notify customer / rex navigation completed
//decline post_navigation_task(not customer) / post_add_navigation_task_traj / post_allocate_operation_task when offline task is existed
//decline offline task when either navigation task or operation task is existed

static objhld_t __local = -1;

int var__load_offlinetask() {

	var__functional_object_t *object;
	var_offline_task_t *target;

	if (var__allocate_functional_object(sizeof (var_offline_task_t), kVarType_OfflineTask, &object) < 0) {
		return -1;
	}
	object->object_id_ = kVarFixedObject_OfflineTask;

	target = var__object_body_ptr(var_offline_task_t, object);
	memset(target, 0, sizeof (var_offline_task_t));
	var__init_status_describe(&target->track_status_);

	// 插入本地管理列表 
	__local = object->handle_;
	// 插入全局的对象管理列表 
	var__insert_object(object);
	return 0;
}

// 取得离线任务对象 
var_offline_task_t *var__get_offline_task() {
	var__functional_object_t *obj;
	var_offline_task_t *task;

	if (__local > 0) {
		obj = objrefr(__local);
		if (obj) {
			task = var__object_body_ptr(var_offline_task_t, obj);
			var__acquire_lock(obj);
			return task;
		}
	}

	return NULL;
}

// 构建离线任务节点 
static int build_task_node(const char *data, const int cb, const int count , var_offline_task_t *task_new)
{
	int ret_value = 0;
	var_offline_task_node_t *tmp_node = NULL;
	var_offline_vector_t *tmp_vctor = NULL;
	const char *src_data = data;
	int tmp_len = 0;
	int left_len = cb;

	if (!data || !cb || !task_new) {
		return -EINVAL;
	}
	if (count <= 0) {
		log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
			"invalid offline task node count: %d", count);
		return -EINVAL;
	}

	for (int i = 0; i < count; i++) {
		tmp_node = &task_new->tasks_[i];
		tmp_len = offsetof(var_offline_task_node_t, trails_);
		if (left_len < tmp_len) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task node data, node: %d", i + 1);
			ret_value = -EINVAL;
			break;
		}
		memcpy(tmp_node, src_data, tmp_len);
		// 读取参考轨迹个数
		tmp_len = offsetof(var_offline_task_node_t, cnt_trails_);
		src_data += tmp_len; left_len -= tmp_len;
		tmp_vctor = (var_offline_vector_t *)src_data;
		tmp_node->cnt_trails_ = tmp_vctor->count_;
		src_data += sizeof(int); left_len -= sizeof(int);
		tmp_len = tmp_node->cnt_trails_ * sizeof(trail_t);
		if (tmp_node->cnt_trails_ < 1 || tmp_node->cnt_trails_ > OFFLINE_MAX_TRAIL || left_len < tmp_len) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task node trail data, node: %d", i + 1);
			ret_value = -EINVAL;
			break;
		}
		// 读取所有参考轨迹数据 
		memcpy(tmp_node->trails_, tmp_vctor->data, tmp_len);
		src_data += tmp_len; left_len -= tmp_len;

		if (left_len < (sizeof(int)+sizeof(var_offline_oper_t))) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task node operation data, node: %d", i + 1);
			ret_value = -EINVAL;
			break;
		}
		// 读取操作个数 
		tmp_vctor = (var_offline_vector_t *)src_data;
		tmp_node->cnt_opers_ = tmp_vctor->count_;
		src_data += sizeof(int); left_len -= sizeof(int);
		tmp_len = tmp_node->cnt_opers_ * sizeof(var_offline_oper_t);
		if (tmp_node->cnt_opers_ < 1 || tmp_node->cnt_opers_ > OFFLINE_MAX_OPER || left_len < tmp_len) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task var_offline_oper_t count: %d", tmp_node->cnt_opers_);
			ret_value = -EINVAL;
			break;
		}
		// 读取所有操作数据 
		memcpy(tmp_node->opers_, tmp_vctor->data, tmp_len);
		src_data += tmp_len; left_len -= tmp_len;
	}

	return ret_value;
}

// 设置新的离线任务对象 
int var__set_offline_task(const char *data, int cb) {
	var_offline_task_t *offline_task = NULL;
	nsp__allocate_offline_task_t *task_pkt = (nsp__allocate_offline_task_t *)data;
	int ret_value = 0;
	int len = 0;

	offline_task = var__get_offline_task();
	if (!offline_task) {
		return -ENOENT;
	}

	if (((offline_task->track_status_.response_ > kStatusDescribe_PendingFunction)
		&& (offline_task->track_status_.response_ < kStatusDescribe_FinalFunction))
		|| (offline_task->track_status_.middle_ != kStatusDescribe_Idle)
		) {
		// 非空闲时，不允许新的离线任务下达 
		var__release_object_reference(offline_task);
		return -EBUSY;
	}

	if (cb < sizeof(nsp__allocate_offline_task_t)) {
		log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
			"invalid request size for nsp__allocate_offline_task_t: %d", cb);
		return -EINVAL;
	}
	if (task_pkt->task_id_ == offline_task->user_task_id_) {
		// 新的离线任务ID，不允许与既存的离线任务ID一致 
		log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
			"offline new task id equal to current."UINT64_STRFMT, offline_task->task_count_);
		return -EINVAL;
	}
	offline_task->user_task_id_ = task_pkt->task_id_;
	offline_task->task_count_ = task_pkt->cnt_nodes_;
	len = cb - sizeof(nsp__allocate_offline_task_t);
	ret_value = build_task_node(task_pkt->data, len, task_pkt->cnt_nodes_, offline_task);
	++offline_task->ato_task_id_;

	if (offline_task) {
		var__release_object_reference(offline_task);
	}
	return ret_value;
}

