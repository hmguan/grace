#include "offlinetask.h"
#include "navigation.h"
#include "var.h"
#include "proto.h"
#include "logger.h"

//free heap memory when next incoming offline task  arrived
//notify customer / rex navigation completed
//decline post_navigation_task(not customer) / post_add_navigation_task_traj / post_allocate_operation_task when offline task is existed
//decline offline task when either navigation task or operation task is existed

static
objhld_t __local = -1;

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
static int build_task_node(const char *data, int *cb, int count , var_offline_task_t *task_new)
{
	int ret_value = 0;
	int len;

	if (!data || !cb || !task_new) {
		return -EINVAL;
	}
	if (count <= 0) {
		log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
			"invalid offline task node count: %d", count);
		return -EINVAL;
	}

	char *dest_data = NULL;
	var_offline_task_node_t *tmp_node = (var_offline_task_node_t *)(data);
	for (int i = 0; i < count; i++) {
		if (*cb < sizeof(var_offline_task_node_t)) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task node data, count: %d", count);
			ret_value = -EINVAL;
			break;
		}
		memcpy(&task_new->tasks_[i].task_id_, data, sizeof(uint64_t));
		data += sizeof(uint64_t); *cb -= sizeof(uint64_t);
		memcpy(&task_new->tasks_[i].dest_upl_, data, sizeof(upl_t));
		data += sizeof(upl_t); *cb -= sizeof(upl_t);
		memcpy(&task_new->tasks_[i].dest_pos_, data, sizeof(position_t));
		data += sizeof(position_t); *cb -= sizeof(position_t);
		// 读取参考轨迹个数 
		memcpy(&task_new->tasks_[i].cnt_trals_, data, sizeof(int));
		data += sizeof(int); *cb -= sizeof(int);
		len = task_new->tasks_[i].cnt_trals_ * sizeof(trail_t);
		if (task_new->tasks_[i].cnt_trals_ <= 0 || task_new->tasks_[i].cnt_trals_ >= MAXIMUM_TRAJ_REF_COUNT \
			|| *cb < len) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task trail_t count: %d", task_new->tasks_[i].cnt_trals_);
			ret_value = -EINVAL;
			break;
		}
		dest_data = &task_new->tasks_[i].data[0];
		// 读取所有参考轨迹数据 
		memcpy(dest_data, data, len);
		data += len; *cb -= len; dest_data += len;

		if (*cb < sizeof(int)) {
			ret_value = -EINVAL;
			break;
		}
		// 读取操作个数 
		memcpy(&task_new->tasks_[i].cnt_opers_, data, sizeof(int));
		data += sizeof(int); *cb -= sizeof(int); dest_data += sizeof(int);
		len = task_new->tasks_[i].cnt_opers_ * sizeof(var_offline_oper_t);
		if (task_new->tasks_[i].cnt_opers_ <= 0 || task_new->tasks_[i].cnt_opers_ >= MAXIMUM_TRAJ_REF_COUNT \
			|| *cb < len) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task var_offline_oper_t count: %d", task_new->tasks_[i].cnt_opers_);
			ret_value = -EINVAL;
			break;
		}
		// 读取所有操作数据 
		memcpy(&dest_data, data, len);
		data += len; *cb -= len; dest_data += len;
		// 下一个 var_offline_task_node_t 的读取 
		tmp_node = (var_offline_task_node_t *)(dest_data);
	}

	return ret_value;
}

// 设置新的离线任务对象 
int var__set_offline_task(const char *data, int cb) {
	var_offline_task_t *task_new = NULL;
	var_offline_task_t *task_old = NULL;
	var__functional_object_t *object_new = NULL;
	var__functional_object_t *object_old = NULL;
	nsp__allocate_offline_task_t *task_pkt = (nsp__allocate_offline_task_t *)data;
	int ret_value = 0;

	if (__local > 0) {
		task_old = var__get_offline_task();
		if (((task_old->track_status_.response_ > kStatusDescribe_PendingFunction)
			&& (task_old->track_status_.response_ < kStatusDescribe_FinalFunction))
			|| (task_old->track_status_.middle_ != kStatusDescribe_Idle)
			) {
			// 非空闲时，不允许新的离线任务下达 
			var__release_object_reference(task_old);
			return -EBUSY;
		}
	}

	int len = cb - sizeof(nsp__allocate_offline_task_t)+ (sizeof(var_offline_task_t) - sizeof(var_offline_task_node_t));
	if (cb < sizeof(nsp__allocate_offline_task_t)) {
		log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
			"invalid request size for nsp__allocate_offline_task_t: %d", cb);
		return -EINVAL;
	}
	if (var__allocate_functional_object(len, kVarType_OfflineTask, &object_new) < 0) {
		return -ENOMEM;
	}
	task_new = var__object_body_ptr(var_offline_task_t, object_new);


	task_new->user_task_id_ = task_pkt->task_id_;
	task_new->task_count_ = task_pkt->cnt_nodes_;
	len = cb - sizeof(nsp__allocate_offline_task_t);
	do {
		if (task_old) {
			if (task_new->user_task_id_ == task_old->user_task_id_) {
				// 新的离线任务ID，不允许与既存的离线任务ID一致 
				log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
					"offline new task id equal to current."UINT64_STRFMT, object_new->object_id_);
				ret_value = -EINVAL;
				break;
			}
			task_new->ato_task_id_ = task_old->ato_task_id_;
		}
		ret_value = build_task_node(task_pkt->data, &len, task_new->task_count_, task_new);
		if (ret_value) {
			break;
		}
		object_new->object_id_ = kVarFixedObject_OfflineTask;
		task_new = var__object_body_ptr(var_offline_task_t, object_new);
		var__init_status_describe(&task_new->track_status_);
		// 从全局的对象管理列表中删除旧的离线任务对象 
		var__delete_object(object_old);
		// 删除旧的离线任务对象 
		var__delete_functional_object(object_old);
	} while (0);
	if (ret_value) {
		if (task_old) {
			var__release_object_reference(object_old);
		}
		var__delete_functional_object(object_new);
		return ret_value;
	}

	// 插入本地管理列表 
	__local = object_new->handle_;
	// 插入全局的对象管理列表 
	var__insert_object(object_new);
	return 0;
}

