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
static int build_task_node(const char *data, const int cb, const int count , var_offline_task_t *task_new)
{
	int ret_value = 0;
	var_offline_task_node_t *tmp_node = NULL;
	var_offline_vector_t *tmp_vctor = NULL;
	const char *src_data = data;
	char *dest_data = NULL;
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

	dest_data = (char *)&task_new->tasks_[0];
	for (int i = 0; i < count; i++) {
		if (left_len < sizeof(var_offline_task_node_t)) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task node data, count: %d", count);
			ret_value = -EINVAL;
			break;
		}
		// 下一个 var_offline_task_node_t 的读取 
		tmp_node = (var_offline_task_node_t *)(dest_data);
		// 9 = sizeof(cnt_trals_) + sizeof(cnt_opers_) + sizeof(data[1]) 
		tmp_len = sizeof(var_offline_task_node_t) - 9;
		memcpy(dest_data, src_data, tmp_len);
		src_data += tmp_len; left_len -= tmp_len;
		tmp_vctor = (var_offline_vector_t *)src_data;
		// 读取参考轨迹个数 
		tmp_node->cnt_trals_ = tmp_vctor->count_;
		src_data += sizeof(int); left_len -= sizeof(int);
		tmp_len = tmp_node->cnt_trals_ * sizeof(trail_t);
		if (tmp_node->cnt_trals_ <= 0 || tmp_node->cnt_trals_ >= MAXIMUM_TRAJ_REF_COUNT \
			|| left_len < tmp_len) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task trail_t count: %d", tmp_node->cnt_trals_);
			ret_value = -EINVAL;
			break;
		}
		// 读取所有参考轨迹数据 
		dest_data = &tmp_node->data[0];
		memcpy(dest_data, tmp_vctor->data, tmp_len);
		src_data += tmp_len; left_len -= tmp_len; dest_data += tmp_len;

		if (left_len < (sizeof(int)+sizeof(var_offline_oper_t))) {
			ret_value = -EINVAL;
			break;
		}
		// 读取操作个数 
		tmp_vctor = (var_offline_vector_t *)src_data;
		tmp_node->cnt_opers_ = tmp_vctor->count_;
		src_data += sizeof(int); left_len -= sizeof(int);
		tmp_len = tmp_node->cnt_opers_ * sizeof(var_offline_oper_t);
		if (tmp_node->cnt_opers_ <= 0 || tmp_node->cnt_opers_ >= MAXIMUM_TRAJ_REF_COUNT \
			|| left_len < tmp_len) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"invalid offline task var_offline_oper_t count: %d", tmp_node->cnt_opers_);
			ret_value = -EINVAL;
			break;
		}
		// 读取所有操作数据 
		memcpy(dest_data, tmp_vctor->data, tmp_len);
		src_data += tmp_len; left_len -= tmp_len; dest_data += tmp_len;
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
	int len = 0;

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

	if (cb < sizeof(nsp__allocate_offline_task_t)) {
		log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
			"invalid request size for nsp__allocate_offline_task_t: %d", cb);
		return -EINVAL;
	}
	len = cb - sizeof(nsp__allocate_offline_task_t);
	len += sizeof(var_offline_task_t) - sizeof(var_offline_task_node_t);
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
		ret_value = build_task_node(task_pkt->data, len, task_new->task_count_, task_new);
		if (ret_value) {
			break;
		}
		object_new->object_id_ = kVarFixedObject_OfflineTask;
		task_new = var__object_body_ptr(var_offline_task_t, object_new);
		var__init_status_describe(&task_new->track_status_);
		if (task_old) {
			var__release_object_reference(task_old);
			object_old = objrefr(__local);
			// 从全局的对象管理列表中删除旧的离线任务对象 
			var__delete_object(object_old);
			// 删除旧的离线任务对象 
			var__delete_functional_object(object_old);
			task_old = NULL;
		}
	} while (0);
	if (ret_value) {
		if (task_old) {
			var__release_object_reference(task_old);
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

