#include "proto.h"
#include "var.h"
#include "vehicle.h"
#include "navigation.h"
#include "nsp.h"
#include "operation.h"
#include "usrdef.h"
#include "proto.h"
#include "optpar.h"
#include "error.h"
#include "regist_event.h"
#include "safety.h"
#include "posix_string.h"
#include "args.h"
#include "posix_atomic.h"
#include "logger.h"
#include "offlinetask.h"

#include <assert.h>

#if _WIN32
#else
    #include <signal.h>
#endif

static
int nspi__on_login(HTCPLINK link, const char *data, int cb) {
    nsp__login_robot_t *pkt_login_robot = (nsp__login_robot_t *) data;
    int retval;
    nsp__login_robot_ack_t ack_login_robot;

    if (cb < sizeof( nsp__login_robot_t)) {
        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, " invalid request size for nsp__login_robot_t:%d", cb);
        return -1;
    }

    if (16 != pkt_login_robot->encrypted_md5_size_ ||
        PROTO_SESSION_KEY_LENGTH != pkt_login_robot->original_buffer_size_ ||
        pkt_login_robot->head_.size_ != sizeof ( nsp__login_robot_t))
    {
        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
            "invalid request parameters for login, encrypt size=%d,original size=%d, head size=%d", 
            pkt_login_robot->encrypted_md5_size_, pkt_login_robot->original_buffer_size_,pkt_login_robot->head_.size_);
        return -1;
    }

    retval = -1;

    // 设置登入属性
    if (pkt_login_robot->cct_ <= kControlorType_MaximumFunctions) {
        retval = nsp__on_login(link,
                pkt_login_robot->cct_,
                pkt_login_robot->acquire_access_,
                pkt_login_robot->original_buffer_,
                sizeof (pkt_login_robot->original_buffer_),
                pkt_login_robot->encrypted_md5_);
        if (retval < 0) {
            return -1;
        }
    }

    // 应答
    memcpy(&ack_login_robot, &pkt_login_robot->head_, sizeof ( nsp__packet_head_t));
    ack_login_robot.size_ = sizeof ( nsp__login_robot_ack_t);
    ack_login_robot.type_ = PKTTYPE_LOGIN_ROBOT_ACK;
    ack_login_robot.err_ = retval;
    return tcp_write(link, ack_login_robot.size_, &nsp__packet_maker, &ack_login_robot);
}

static HTCPLINK previous_link = INVALID_HTCPLINK;

static
int nspi__on_common_write_byid(HTCPLINK link, const char *data, int cb) {
    nsp__common_protocol_item_t *item;
    nsp__common_protocol_t *pkt_common_write = (nsp__common_protocol_t *) data;
    int i;
    char *p_var;
    nsp__common_protocol_write_ack_t ack_common_write;
    objhld_t hld;
    var__functional_object_t *obj;
    int usable;

    p_var = NULL;
    usable = cb;
    
    /* build a acknowledge packet */
    memcpy(&ack_common_write, &pkt_common_write->head_, sizeof ( nsp__packet_head_t));
    ack_common_write.err_ = 0;
    ack_common_write.size_ = sizeof ( nsp__packet_head_t);
    ack_common_write.type_ = PKTTYPE_COMMON_WRITE_BYID_ACK;

    /* source data length security checking */
    if (usable < sizeof(nsp__common_protocol_t)) {
        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
            "invalid common write byid request size %d", cb);
        return -1;
    }
    usable -= sizeof(nsp__common_protocol_t);

    /* try to write all request node into functional objects */
    item = (nsp__common_protocol_item_t *) pkt_common_write->items;
    for (i = 0; i < pkt_common_write->count_; i++) {

        /* data length security checking */
        usable -= sizeof(nsp__common_protocol_item_t);
        usable -= item->length_;
        if (usable < 0) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "invalid common write byid request size");
            ack_common_write.err_ = -1;
            break;
        }

        hld = var__getobj_handle_byid(item->var_id_);
        if (hld > 0) {
            obj = (var__functional_object_t *) objrefr(hld);
            if (obj) {
                p_var = (char *) obj->body_;

                var__acquire_lock(obj);

                if (obj->object_id_ == kVarFixedObject_Operation) {
					if (link != posix__atomic_xchange(&previous_link, link)) {
                        log__save("motion_template", 
                            kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout, "operation source changed to:%ld", link);
                    }
                }

                // protect memory access fatal
                if (item->offset_ + item->length_ > obj->body_length_) {
                    log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                        "common wrtie by id request memory error, offset=%u, length=%u, ID=%u", item->offset_, item->length_, item->var_id_);
                    ack_common_write.err_ = -1;
                } else {
                    memcpy(p_var + item->offset_, item->data_, item->length_);
                }

                var__release_lock(obj);
                objdefr(hld);

                /* no matter which step of security check failed, 
                    subsequent requests are no longer executed */
                if (ack_common_write.err_ < 0) {
                    break;
                }

                p_var = NULL;
                hld = -1;
            }
        }
        item = (nsp__common_protocol_item_t *) (((char *) item) + (sizeof ( nsp__common_protocol_item_t) + item->length_));
    }

    return tcp_write(link, ack_common_write.size_, &nsp__packet_maker, &ack_common_write);
}

static
int nspi__on_common_read_byid(HTCPLINK link, const char *data, int cb) {
    nsp__common_protocol_t *pkt_common_read = (nsp__common_protocol_t *) data;
    nsp__common_protocol_item_t *items;
    int i;
    int total_ack_length = 0;
    nsp__common_protocol_t *ack_common_read;
    nsp__common_protocol_item_t *ack_item;
    char *p_var;
    int retval;
    objhld_t hld;
    var__functional_object_t *obj;
    int usable;
	int32_t read_err = 0;

    p_var = NULL;
    hld = -1;
    usable = cb; 
    total_ack_length = sizeof ( nsp__common_protocol_t);

    // 请求解析过程的可用字节长度基本检查
    if (usable < sizeof(nsp__common_protocol_t)) {
        log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
            "invalid common read byid request size %d", cb);
        return -1;
    }
    usable -= sizeof(nsp__common_protocol_t);

    // 请求内容详细检查, 同时求出应答包长度
    items = (nsp__common_protocol_item_t *) pkt_common_read->items;
    for (i = 0; i < pkt_common_read->count_; i++) {
        usable -= sizeof(nsp__common_protocol_item_t);
        // 必须保证请求包长度完整
        if (usable < 0) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "invalid common read request size");
            return -1;
        }
        if (items[i].offset_ < 0 || items[i].length_ <= 0) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invalid common read request for object %d, offset=%d, length=%d", items[i].var_id_, items[i].offset_, items[i].length_);
            return -1;
        }
        total_ack_length += sizeof ( nsp__common_protocol_item_t);
        total_ack_length += items[i].length_;
    }

    // 构建应答包
    ack_common_read = (nsp__common_protocol_t *) malloc(total_ack_length);
    if (!ack_common_read) {
        return -1;
    }
    memcpy(&ack_common_read->head_, &pkt_common_read->head_, sizeof ( nsp__packet_head_t));
    ack_common_read->head_.size_ = total_ack_length;
    ack_common_read->head_.type_ = PKTTYPE_COMMON_READ_BYID_ACK;
    ack_common_read->count_ = pkt_common_read->count_;

    // 在合适的地址偏移构建应答包数据
    items = (nsp__common_protocol_item_t *) pkt_common_read->items;
    ack_item = (nsp__common_protocol_item_t *) ack_common_read->items;
    for (i = 0; i < pkt_common_read->count_; i++) {
        ack_item->offset_ = items[i].offset_;
        ack_item->length_ = items[i].length_;
        ack_item->var_id_ = items[i].var_id_;

        hld = var__getobj_handle_byid(items[i].var_id_);
        if (hld > 0) {
            obj = (var__functional_object_t *) objrefr(hld);
            if (obj) {
                ack_item->var_type_ = obj->type_;
                p_var = (char *) obj->body_;
                var__acquire_lock(obj);

                // 内存访问越界保护
                if (items[i].offset_ + items[i].length_ > obj->body_length_) {
                    log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                        "common read request memory error, offset=%u, length=%u, body=%u", items[i].offset_, items[i].length_, obj->body_length_);
					read_err = -ERANGE;
                } else {
                    memcpy(ack_item->data_, p_var + items[i].offset_, items[i].length_);
                }

                var__release_lock(obj);
                objdefr(hld);
                p_var = NULL;
                hld = -1;
				if (read_err) {
					break;
				}
            }
        } else {
			read_err = -EFAULT;
			break;
		}
        ack_item = (nsp__common_protocol_item_t *) (((char *) ack_item) + (sizeof ( nsp__common_protocol_item_t) + items[i].length_));
    }

	if (read_err) {
		free(ack_common_read);
		ack_common_read = (nsp__common_protocol_t *)malloc(sizeof(nsp__common_protocol_t));
		if (!ack_common_read) {
			return -1;
		}
		memcpy(&ack_common_read->head_, &pkt_common_read->head_, sizeof (nsp__packet_head_t));
		ack_common_read->head_.size_ = sizeof(nsp__common_protocol_t);
		ack_common_read->head_.type_ = PKTTYPE_COMMON_READ_BYID_ACK;
		ack_common_read->head_.err_ = read_err;
		ack_common_read->count_ = 0;
	}
    // 发送应答包
    retval = tcp_write(link, ack_common_read->head_.size_, &nsp__packet_maker, ack_common_read);

    free(ack_common_read);
    return retval;
}

static
int nspi__on_common_compare_write(HTCPLINK link, const char *data, int cb) {
    struct nsp__common_compare_write *pkt_common_compare_write = (struct nsp__common_compare_write *) data;
    uint8_t *request_parse_ptr = (uint8_t *) pkt_common_compare_write;
    int condition_count;
    nsp__common_protocol_item_t *conditions;
    int write_count;
    nsp__common_protocol_item_t *writes;
    nsp__packet_head_t ack_common_compare_write;
    int i;
    var__functional_object_t *obj;
    objhld_t hld;
    char *p_var;

    /* build base acknowledge packet */
    memcpy(&ack_common_compare_write, &pkt_common_compare_write->head_, sizeof ( pkt_common_compare_write->head_));
    ack_common_compare_write.type_ = PKTTYPE_COMMON_COMPARE_WRITE_ACK;
    ack_common_compare_write.size_ = sizeof (ack_common_compare_write);
    ack_common_compare_write.err_ = 0;

    request_parse_ptr += sizeof (struct nsp__common_compare_write);

    /* parse request condition */
    condition_count = *((int *) request_parse_ptr);
    if (condition_count <= 0) {
        ack_common_compare_write.err_ = -EINVAL;
        return tcp_write(link, ack_common_compare_write.size_, &nsp__packet_maker, &ack_common_compare_write);
    }
    request_parse_ptr += sizeof (condition_count);
    conditions = (nsp__common_protocol_item_t *) request_parse_ptr;
    for (i = 0; i < condition_count; i++) {
        request_parse_ptr += sizeof ( nsp__common_protocol_item_t);
        request_parse_ptr += conditions[i].length_;
    }

    /* parse reqeust data */
    write_count = *((int *) request_parse_ptr);
    if (write_count <= 0) {
        ack_common_compare_write.err_ = -EINVAL;
        return tcp_write(link, ack_common_compare_write.size_, &nsp__packet_maker, &ack_common_compare_write);
    }
    request_parse_ptr += sizeof (write_count);
    writes = (nsp__common_protocol_item_t *) request_parse_ptr;
    
    do {
        ack_common_compare_write.err_ = 0;
        obj = NULL;
        hld = -1;
    
        hld = var__getobj_handle_byid(pkt_common_compare_write->var_id_);
        if (hld <= 0) {
            ack_common_compare_write.err_ = -EFAULT;
            break;
        }

        obj = (var__functional_object_t *) objrefr(hld);
        if (!obj) {
            ack_common_compare_write.err_ = -EFAULT;
            break;
        }

        p_var = (char *) obj->body_;
        var__acquire_lock(obj);

        /* 预设为条件不成立 */
        /* 条件检查 */
        for (i = 0; i < condition_count; i++) {
            if (conditions[i].offset_ + conditions[i].length_ > obj->body_length_) {
                log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                        "common compare write request memory error, offset=%u, length=%u, body=%u", conditions[i].offset_, conditions[i].length_, obj->body_length_);
                ack_common_compare_write.err_ = -EFAULT;
                break;
            }

            /* 条件不成立， 且请求要求与判断，则本次操作无法继续 */
            if (0 != memcmp(conditions[i].data_, p_var + conditions[i].offset_, conditions[i].length_)) {
                ack_common_compare_write.err_ = -EPERM;
                if (pkt_common_compare_write->method_ == kConditionCheck_And) {
                    break;
                }
            } else {
                /* 条件成立， 且条件检查方式为或， 则整体条件成立 */
                if (pkt_common_compare_write->method_ == kConditionCheck_Or) {
                    ack_common_compare_write.err_ = 0;
                    break;
                }
            }
        }

        /* 条件不成立或内存错误 */
        if (ack_common_compare_write.err_ < 0) {
            break;
        }

        /* 可以执行写入 */
        for (i = 0; i < write_count; i++) {
            if (writes[i].offset_ + writes[i].length_ > obj->body_length_) {
                log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                        "common compare write request memory error, offset=%u, length=%u, body=%u", writes[i].offset_, writes[i].length_, obj->body_length_);
                ack_common_compare_write.err_ = -EFAULT;
                break;
            }
            memcpy(p_var + writes[i].offset_, writes[i].data_, writes[i].length_);
        }
    } while (0);

    if (obj) {
        var__release_lock(obj);
    }
    if (hld > 0) {
        objdefr(hld);
    }
    return tcp_write(link, ack_common_compare_write.size_, &nsp__packet_maker, &ack_common_compare_write);
}

static
int nspi__on_allocate_navigation_task(HTCPLINK link, const char *data, int cb) {
    nsp__allocate_navigation_task_t *pkt_allocate_nav_task = (nsp__allocate_navigation_task_t *) data;
    var__navigation_t *nav;
    var__vehicle_t *veh;
    nsp__allocate_navigation_task_ack_t ack_control_navigation_task;

    memcpy(&ack_control_navigation_task.head_, &pkt_allocate_nav_task->head_, sizeof ( pkt_allocate_nav_task->head_));
    ack_control_navigation_task.head_.type_ = PKTTYPE_ALLOC_NAVIGATION_TASK_ACK;
    ack_control_navigation_task.head_.size_ = sizeof ( ack_control_navigation_task);
    ack_control_navigation_task.status_ = 0;
    ack_control_navigation_task.head_.err_ = 0;

    do {
        if (cb < sizeof(nsp__allocate_navigation_task_t)) {
            log__save("motion_template",  kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invalid request size for nsp__allocate_navigation_task_t: %d", cb);
            ack_control_navigation_task.head_.err_ = -EINVAL;
            break;
        }

        // invalid reference trajectory specified in packet
        if ((pkt_allocate_nav_task->cnt_trajs_ >= MAXIMUM_TRAJ_REF_COUNT) || (pkt_allocate_nav_task->cnt_trajs_  <= 0)) {
            log__save("motion_template",  kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                    "invalid trajs when allocate navigation task, traj count:%d", pkt_allocate_nav_task->cnt_trajs_ );
            ack_control_navigation_task.head_.err_ = -EINVAL;
            break;
        }

        // invalid data length of reference trajectory field in packet
        if (cb < (int)(pkt_allocate_nav_task->cnt_trajs_ * sizeof(trail_t) + sizeof(nsp__allocate_navigation_task_t))) {
            log__save("motion_template",  kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invalid request size for nsp__allocate_navigation_task_t: %d traj count:%d", cb, pkt_allocate_nav_task->cnt_trajs_ );
            ack_control_navigation_task.head_.err_ = -EINVAL;
            break;
        }

        nav = var__get_navigation();
        if (!nav) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "navigation object is not existed.");
            ack_control_navigation_task.head_.err_ = -ENOENT;
            break;
        }

        // the new task id equal to current, request will be declined
        if (pkt_allocate_nav_task->set_task_id_ == nav->i.current_task_id_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "navigation new task id equal to current."UINT64_STRFMT, nav->i.current_task_id_);
            var__release_object_reference(nav);

            ack_control_navigation_task.head_.err_ = -EINVAL;
            break;
        }

        // dispatcher cannot cover navigation task before cancel existing one
        // also cannot cancel task when track status is in middle
        if (((nav->track_status_.response_ > kStatusDescribe_PendingFunction) &&
                (nav->track_status_.response_ < kStatusDescribe_FinalFunction)) ||
                (nav->track_status_.middle_ != kStatusDescribe_Idle)
                ) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "failed to allocate navigation task,status response=%u, middle=%u", nav->track_status_.response_, nav->track_status_.middle_);
            var__release_object_reference(nav);

            ack_control_navigation_task.head_.err_ = -EBUSY;
            break;
        }

        nav->is_traj_whole_ = pkt_allocate_nav_task->is_traj_whole_;
        memcpy(&nav->dest_upl_, &pkt_allocate_nav_task->dest_upl_, sizeof ( pkt_allocate_nav_task->dest_upl_));
        memcpy(&nav->dest_pos_, &pkt_allocate_nav_task->dest_pos_, sizeof ( pkt_allocate_nav_task->dest_pos_));

        // release previous navigation trajectorys, reestablish the navigation sequence 
        if (nav->traj_ref_.data_ && nav->traj_ref_.count_ > 0) {
            free(nav->traj_ref_.data_);
        }
        nav->traj_ref_.count_ = 0;
        nav->traj_ref_.data_ = NULL;

        /* protect the source request and count of trajs  */
        nav->traj_ref_.data_ = malloc(pkt_allocate_nav_task->cnt_trajs_ * sizeof ( trail_t));
        if (nav->traj_ref_.data_) {
            nav->traj_ref_.count_ = pkt_allocate_nav_task->cnt_trajs_;
            memcpy(nav->traj_ref_.data_, pkt_allocate_nav_task->traj_, pkt_allocate_nav_task->cnt_trajs_ * sizeof ( trail_t));
            var__xchange_command_status(&nav->track_status_, kStatusDescribe_Startup, NULL);
            nav->user_task_id_ = pkt_allocate_nav_task->set_task_id_; // using the new task id
            ack_control_navigation_task.task_id_ = nav->user_task_id_;  // acknowledge using current task id
            posix__atomic_inc(&nav->ato_task_id_); // atomic increase inner task id
            log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,
                "successful allocate navigation task. user_id=%lld, ato_id=%lld", nav->user_task_id_, nav->ato_task_id_);
        }else{
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                "failed to allocate navigation task because insuffcient memory.");
            ack_control_navigation_task.head_.err_ = -ENOMEM;
        }

        var__release_object_reference(nav);

        // change control mode to navigation
        veh = var__get_vehicle();
        veh->control_mode_ = kVehicleControlMode_Navigation;
        var__release_object_reference(veh);
    }while(0);

    return tcp_write(link, ack_control_navigation_task.head_.size_, &nsp__packet_maker, &ack_control_navigation_task);
}

static
int nspi__on_additional_navigation_trajectory(HTCPLINK link, const char *data, int cb) {
    var__navigation_t *nav;
    nsp__additional_navigation_trajectory_ack_t ack_additional_navigation_trajectory;
    nsp__additional_navigation_trajectory_t *pkt_additional_navigation_trajectory = (nsp__additional_navigation_trajectory_t *) data;
    trail_t *realloc_trajectory;
    int realloc_traj_cnt;
    uint32_t previous_traj_status;
    
    memcpy(&ack_additional_navigation_trajectory, &pkt_additional_navigation_trajectory->head_,
            sizeof ( pkt_additional_navigation_trajectory->head_));
    ack_additional_navigation_trajectory.size_ = sizeof ( ack_additional_navigation_trajectory);
    ack_additional_navigation_trajectory.type_ = PKTTYPE_ADDITIONAL_NAVIGATION_TRAJECTORY_ACK;
    ack_additional_navigation_trajectory.err_ = 0;

    previous_traj_status = 0xFFFFFFFF;

    do {
        nav = NULL;

        if (cb < sizeof( nsp__additional_navigation_trajectory_t)) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invlaid request size for nsp__additional_navigation_trajectory_t:%d", cb);
            ack_additional_navigation_trajectory.err_ = -EINVAL;
            break;
        }

        // 分配导航任务但是没有指定有效的参考轨迹
        if (pkt_additional_navigation_trajectory->cnt_trajs_  <= 0) {
            log__save("motion_template",  kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                    "invalid trajs when allocate navigation task, traj count:%d", pkt_additional_navigation_trajectory->cnt_trajs_ );
            ack_additional_navigation_trajectory.err_ = -EINVAL;
            break;
        }

        // 指定了参考轨迹， 但是参考轨迹的数据并不符合请求包长度
        if (cb < (int)(pkt_additional_navigation_trajectory->cnt_trajs_ * sizeof(trail_t) + sizeof(nsp__additional_navigation_trajectory_t))) {
            log__save("motion_template",  kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invalid request size for nsp__additional_navigation_trajectory_t: %d traj count:%d", cb, pkt_additional_navigation_trajectory->cnt_trajs_ );
            ack_additional_navigation_trajectory.err_ = -EINVAL;
            break;
        }

        // 直接变更 nav 对象中的数据
        nav = var__get_navigation();
        if (!nav) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "navigation object is not existed.");
            ack_additional_navigation_trajectory.err_ = -ENOENT;
            break;
        }

        // request task id not equal to current, decliend.
        if (nav->i.current_task_id_ != pkt_additional_navigation_trajectory->task_id_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "navigation task mismatch for add trajectory, current:"UINT64_STRFMT", request:"UINT64_STRFMT
                    , nav->i.current_task_id_, pkt_additional_navigation_trajectory->task_id_);

            // 任务存在且ID不符
            ack_additional_navigation_trajectory.err_ = -EEXIST;
            break;
        }

        // 当前没有执行中的导航任务， 无法完成追加导航轨迹操作
        if ((nav->track_status_.response_ < kStatusDescribe_PendingFunction) ||
                (nav->track_status_.response_ >= kStatusDescribe_FinalFunction)) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "failed to additional navigation task,status response=%u", nav->track_status_.response_);

            // 导航正忙
            ack_additional_navigation_trajectory.err_ = -EBUSY;
            break;
        }

        // 追加导航序列索引必须位于当前导航序列内
        if (pkt_additional_navigation_trajectory->begin_index_ > nav->traj_ref_.count_) {
            // 不指定有效的起始索引， 则从当前队列尾开始追加
            if (pkt_additional_navigation_trajectory->begin_index_ == -1) {
                pkt_additional_navigation_trajectory->begin_index_ = nav->traj_ref_.count_;
            } else {
                log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                        "failed to additional navigation task because begin index error, begin index=%d, traj count=%d", 
                        pkt_additional_navigation_trajectory->begin_index_, nav->traj_ref_.count_);
                // 区域不符
                ack_additional_navigation_trajectory.err_ = -ERANGE;
                break;
            }
        }

        // 上层确认本次导航参考轨迹是否完整
        previous_traj_status = nav->is_traj_whole_;
        nav->is_traj_whole_ = pkt_additional_navigation_trajectory->is_traj_whole_;

#if 0
        char log_nav_pkt_traj[1024];
        int i;
        int off = 0;
        off += posix__sprintf(&log_nav_pkt_traj[off], sizeof(log_nav_pkt_traj)-off, "additional nav traj from:%u edges:",
            pkt_additional_navigation_trajectory->begin_index_);
        for (i = 0; i < pkt_additional_navigation_trajectory->cnt_trajs_; i++) {
            trail_t *p_trail = (trail_t *)pkt_additional_navigation_trajectory->traj_;
            off += posix__sprintf(&log_nav_pkt_traj[off], sizeof(log_nav_pkt_traj)-off, "%d ", p_trail->edge_id_);
        }
        log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem, "%s", log_nav_pkt_traj);
#endif

        // 如果: 追加起始索引 + 追加边数量  <= 现有总量
        // 不需要重新分配内存，直接覆盖 所引@数量 的数据即可
        if ((pkt_additional_navigation_trajectory->begin_index_ + pkt_additional_navigation_trajectory->cnt_trajs_) <= nav->traj_ref_.count_) {
            if (nav->is_traj_whole_ > 0) {
                log__save("motion_template", kLogLevel_Warning, kLogTarget_Filesystem | kLogTarget_Stdout,
                        "recover navigation trajectory, but whole feild is %d", nav->is_traj_whole_);
            }
            realloc_trajectory = (trail_t *)nav->traj_ref_.data_;
            memcpy(&realloc_trajectory[pkt_additional_navigation_trajectory->begin_index_], 
                pkt_additional_navigation_trajectory->traj_, pkt_additional_navigation_trajectory->cnt_trajs_ * sizeof (trail_t));
            break;
        }

        // 如果当前没有任何的导航边信息， 则全新创建, 全量覆盖
        if (0 == nav->traj_ref_.count_ || !nav->traj_ref_.data_) {
            assert(0 == nav->traj_ref_.count_ );
            assert(!nav->traj_ref_.data_);
            realloc_traj_cnt = pkt_additional_navigation_trajectory->cnt_trajs_;
            realloc_trajectory = (trail_t *)malloc(realloc_traj_cnt * sizeof ( trail_t));
            if (realloc_trajectory) {
                memcpy(&realloc_trajectory[0],
                    pkt_additional_navigation_trajectory->traj_, pkt_additional_navigation_trajectory->cnt_trajs_ * sizeof (trail_t));
                nav->traj_ref_.count_ = realloc_traj_cnt;
                nav->traj_ref_.data_ = realloc_trajectory;
            }else{
                ack_additional_navigation_trajectory.err_ = -ENOMEM;
            }
            break;
        }

        // 覆盖索引到现有队尾的距离 = 现有总量 - 追加起始索引
        // 重新分配的总量 = 现有总量 + (追加边数量 - 覆盖索引到现有队尾的距离)
        realloc_traj_cnt = (nav->traj_ref_.count_ + pkt_additional_navigation_trajectory->cnt_trajs_) -
                           (nav->traj_ref_.count_ - pkt_additional_navigation_trajectory->begin_index_);
        if ((realloc_traj_cnt > 0) && (realloc_traj_cnt < MAXIMUM_TRAJ_REF_COUNT) /*总量控制*/) {
            realloc_trajectory = (trail_t *)malloc(realloc_traj_cnt * sizeof ( trail_t));
            if (realloc_trajectory) {
                // 保持全部现有部分数据
                memcpy(realloc_trajectory, nav->traj_ref_.data_, nav->traj_ref_.count_ * sizeof ( trail_t));
                // 使用新数据的覆盖部分
                memcpy(&realloc_trajectory[pkt_additional_navigation_trajectory->begin_index_], 
                    pkt_additional_navigation_trajectory->traj_, pkt_additional_navigation_trajectory->cnt_trajs_ * sizeof (trail_t));
                // 释放原有的导航边数据
                free(nav->traj_ref_.data_);
                // 重置导航边序列
                nav->traj_ref_.count_ = realloc_traj_cnt;
                nav->traj_ref_.data_ = realloc_trajectory;
            }else{
                ack_additional_navigation_trajectory.err_ = -ENOMEM;
            }
            break;
        }

#if 0
        /* 记录内存中的全部导航轨迹 */
        if (nav->traj_ref_.count_ > 0){
            uint32_t log_len = 1024 + nav->traj_ref_.count_ * 8;
            if (log_len < (2 << 20)){         /* 不对超过20MB的轨迹进行跟踪 */
                char *log_nav_memory_traj = (char *)malloc(log_len);
                if (log_nav_memory_traj) {
                    trail_t *p_trail = (trail_t *)nav->traj_ref_.data_;
                    off = 0;
                    off += posix__sprintf(&log_nav_memory_traj[off], log_len - off, "navigation memory traj:");
                    for (i = 0; i < nav->traj_ref_.count_; i++){
                        off += posix__sprintf(&log_nav_memory_traj[off], log_len - off, "%d ", p_trail->edge_id_);
                        p_trail++;
                    }
                    log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem, "%s", log_nav_memory_traj);
                    free(log_nav_memory_traj);
                }
            }
        }
#endif
    }while(0);

    if (nav) {
        if ((ack_additional_navigation_trajectory.err_ < 0 ) && (0xFFFFFFFF != previous_traj_status)) {
            nav->is_traj_whole_ = previous_traj_status;
        }

        log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem, "add nav trajs, whole flag = %d", nav->is_traj_whole_);
        
        var__release_object_reference(nav);
    }

    return tcp_write(link, ack_additional_navigation_trajectory.size_, &nsp__packet_maker, &ack_additional_navigation_trajectory);
}

static
int nspi__on_control_navigation_task_cancel(HTCPLINK link, const nsp__packet_head_t *head) {
    var__navigation_t *nav;
    nsp__control_navigation_task_ack_t ack_control_navigation_task;
    nsp__cancel_navigation_task_t *pkt_cancel_navigation_task = (nsp__cancel_navigation_task_t *) head;

    memcpy(&ack_control_navigation_task.head_, head, sizeof ( nsp__packet_head_t));
    ack_control_navigation_task.head_.type_ = head->type_ | PKTTYPE_ACK;
    ack_control_navigation_task.head_.size_ = sizeof ( ack_control_navigation_task);
    ack_control_navigation_task.status_ = 0;
    ack_control_navigation_task.head_.err_ = 0;

    do {
         // 直接变更 nav 对象中的数据
        nav = var__get_navigation();
        if (!nav) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "navigation object is not existed.");
            ack_control_navigation_task.head_.err_ = -ENOENT;
            break;
        }

        // 如果指定的 task id 和当前的 task id 不符，则返回错误
        if (nav->i.current_task_id_ != pkt_cancel_navigation_task->task_id_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                "navigation task mismatch for cancel, current:"UINT64_STRFMT", request:"UINT64_STRFMT
                , nav->i.current_task_id_, pkt_cancel_navigation_task->task_id_);
            ack_control_navigation_task.head_.err_ = -EEXIST;
            break;
        }

        // 直接将命令状态置为取消
        var__xchange_command_status(&nav->track_status_, kStatusDescribe_Cancel, NULL);

        // report the current task id to calling thread
        ack_control_navigation_task.task_id_ = nav->i.current_task_id_;

        // 应答状态置为 cancel
        ack_control_navigation_task.status_ = (int) kPRC_Cancel;
    }while(0);

    if (nav) {
        var__release_object_reference(nav);
    }

    return tcp_write(link, ack_control_navigation_task.head_.size_, &nsp__packet_maker, &ack_control_navigation_task);
}

static
int nspi__on_control_navigation_task_pause(HTCPLINK link, const nsp__packet_head_t *head) {
    var__navigation_t *nav;
    nsp__control_navigation_task_ack_t ack_control_navigation_task;
    nsp__pause_navigation_task_t *pkt_pause_navigation_task = (nsp__pause_navigation_task_t *) head;

    memcpy(&ack_control_navigation_task.head_, head, sizeof ( nsp__packet_head_t));
    ack_control_navigation_task.head_.type_ = head->type_ | PKTTYPE_ACK;
    ack_control_navigation_task.head_.size_ = sizeof ( ack_control_navigation_task);
    ack_control_navigation_task.status_ = 0;
    ack_control_navigation_task.head_.err_ = 0;

    do {
        nav = var__get_navigation();
        if (!nav) {
            ack_control_navigation_task.head_.err_ = -ENOENT;
            break;
        }

        // 如果指定的 task id 和当前的 task id 不符，则返回错误
        if (nav->i.current_task_id_ != pkt_pause_navigation_task->task_id_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "navigation task mismatch for pause, current:"UINT64_STRFMT", request:"UINT64_STRFMT
                    , nav->i.current_task_id_, pkt_pause_navigation_task->task_id_);
            ack_control_navigation_task.head_.err_ = -EEXIST;
            break;
        }

        // 直接变更 nav 对象中的数据
        var__xchange_command_status(&nav->track_status_, kStatusDescribe_Pause, NULL);

        // report the current task id to calling thread
        ack_control_navigation_task.task_id_ = nav->i.current_task_id_;
        // 应答状态置为 Pause
        ack_control_navigation_task.status_ = kPRC_Pause;
    }while(0);

    if (nav) {
        var__release_object_reference(nav);
    }
    
    return tcp_write(link, ack_control_navigation_task.head_.size_, &nsp__packet_maker, &ack_control_navigation_task);
}

static
int nspi__on_control_navigation_task_resume(HTCPLINK link, const nsp__packet_head_t *head) {
    var__navigation_t *nav;
    var__vehicle_t *veh;
    nsp__resume_navigation_task_t *pkt_resume_navigation_task = (nsp__resume_navigation_task_t *) head;
    nsp__control_navigation_task_ack_t ack_control_navigation_task;
    var__status_describe_t previous_status;

    memcpy(&ack_control_navigation_task.head_, head, sizeof ( nsp__packet_head_t));
    ack_control_navigation_task.head_.type_ = head->type_ | PKTTYPE_ACK;
    ack_control_navigation_task.head_.size_ = sizeof ( ack_control_navigation_task);
    ack_control_navigation_task.head_.err_ = 0;
    ack_control_navigation_task.status_ = 0;

    do {
        nav = var__get_navigation();
        if (!nav) {
            ack_control_navigation_task.head_.err_ = -ENOENT;
            break;
        }

        // 如果指定的 task id 和当前的 task id 不符，则返回错误
        if (nav->i.current_task_id_ != pkt_resume_navigation_task->task_id_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "navigation task mismatch for resume, current:"UINT64_STRFMT", request:"UINT64_STRFMT
                    , nav->i.current_task_id_, pkt_resume_navigation_task->task_id_);
            var__release_object_reference(nav);
            ack_control_navigation_task.head_.err_ = -EEXIST;
            break;
        }

        /* 记录当前状态， 变更 nav 对象中的数据 */
        previous_status = nav->track_status_;
        var__xchange_command_status(&nav->track_status_, kStatusDescribe_Resume, NULL);
        // report the current task id to calling thread
        ack_control_navigation_task.task_id_ = nav->i.current_task_id_;
        var__release_object_reference(nav);

        /* 任务恢复，需要将底盘中的控制状态置换为导航模式 */
        veh = var__get_vehicle();
        if (!veh) {
            /* 还原导航任务状态 */
            nav = var__get_navigation();
            if (nav) {
                nav->track_status_ = previous_status;
                var__release_object_reference(nav);
            }
            ack_control_navigation_task.head_.err_ = -ENOENT;
            break;
        }
        veh->control_mode_ = kVehicleControlMode_Navigation;
        var__release_object_reference(veh);

        // 应答状态置为 Resume
        ack_control_navigation_task.status_ = kPRC_Resume;
    }while(0);
    
    return tcp_write(link, ack_control_navigation_task.head_.size_, &nsp__packet_maker, &ack_control_navigation_task);
}

static
int nspi__on_query_navigation_task_status(HTCPLINK link, const char *data, int cb) {
    nsp__query_navigation_task_status_t *pkt_query_navigation_task_status = (nsp__query_navigation_task_status_t *) data;
    var__navigation_t *nav;
    var__vehicle_t *veh;
    nsp__query_navigation_task_status_ack_t ack_query_navigation_task_status;

    memcpy(&ack_query_navigation_task_status.head_, pkt_query_navigation_task_status, sizeof ( nsp__packet_head_t));
    ack_query_navigation_task_status.head_.type_ = PKTTYPE_QUERY_NAVIGATION_TASK_STATUS_ACK;
    ack_query_navigation_task_status.head_.size_ = sizeof ( ack_query_navigation_task_status);
    ack_query_navigation_task_status.head_.err_ = 0;

    do {
        // 直接获取 nav 对象中的数据
        nav = var__get_navigation();
        if (!nav) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "navigation object is not existed.");
            ack_query_navigation_task_status.head_.err_ = -ENOENT;
            break;
        }

        // task id not equal to current, request delined.
        if (nav->i.current_task_id_ != pkt_query_navigation_task_status->task_id_) {
            var__release_object_reference(nav);
            ack_query_navigation_task_status.head_.err_ = -EEXIST;
            break;
        }

        ack_query_navigation_task_status.task_id_ = nav->i.current_task_id_;
        ack_query_navigation_task_status.traj_index_current_ = nav->i.traj_ref_index_curr_;
        ack_query_navigation_task_status.track_eno_ = nav->track_status_.response_;
        ack_query_navigation_task_status.pos_ = nav->pos_;
        ack_query_navigation_task_status.pos_time_stamp_ = nav->pos_time_stamp_;
        ack_query_navigation_task_status.pos_confidence_ = nav->pos_confidence_;
        ack_query_navigation_task_status.upl_ = nav->i.upl_;
        var__release_object_reference(nav);

        // 直接获取 veh 对象中的数据
        veh = var__get_vehicle();
        if (!veh) {
            ack_query_navigation_task_status.head_.err_ = -ENOENT;
            break;
        }
        ack_query_navigation_task_status.vehcile_control_mode_ = veh->control_mode_;
        var__release_object_reference(veh);
    }while(0);

    return tcp_write(link, ack_query_navigation_task_status.head_.size_, &nsp__packet_maker, &ack_query_navigation_task_status);
}

static
int nspi__on_query_navigation_task_trajs(HTCPLINK link, const char *data, int cb) {
    nsp__query_navigation_task_traj_t *pkt_query_navigation_task_traj = (nsp__query_navigation_task_traj_t *) data;
    var__navigation_t *nav;
    nsp__query_navigation_task_traj_ack_t ack, *ack_query_navigation_task_traj = NULL;
    int retval;

    do {
        ack_query_navigation_task_traj = &ack;
        ack_query_navigation_task_traj->cnt_trajs_ = 0;

        // 直接获取 nav 对象中的数据
        nav = var__get_navigation();
        if (!nav) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "navigation object is not existed.");
            ack_query_navigation_task_traj->head_.err_ = -ENOENT;
            return -1;
        }

        // report the current task id to calling thread
        ack_query_navigation_task_traj->task_id_ = nav->i.current_task_id_;

        // 没有任何导航轨迹
        if (!nav->traj_ref_.data_ || nav->traj_ref_.count_ <= 0) {
            ack_query_navigation_task_traj->head_.err_ = 0;
            break;
        }

        ack_query_navigation_task_traj = (nsp__query_navigation_task_traj_ack_t *) malloc(
                sizeof ( nsp__query_navigation_task_traj_ack_t) + sizeof ( trail_t) * nav->traj_ref_.count_);
        if (!ack_query_navigation_task_traj) {
            ack_query_navigation_task_traj->head_.err_ = -ENOMEM;
            break;
        }

        ack_query_navigation_task_traj->cnt_trajs_ = nav->traj_ref_.count_;
        memcpy(ack_query_navigation_task_traj->traj_, nav->traj_ref_.data_, sizeof ( trail_t) * nav->traj_ref_.count_);
    }while(0);

    if (nav) {
        var__release_object_reference(nav);
    }

    memcpy(&ack_query_navigation_task_traj->head_, pkt_query_navigation_task_traj, sizeof ( nsp__packet_head_t));
    ack_query_navigation_task_traj->head_.type_ = PKTTYPE_QUERY_NAVIGATION_TASK_TRAJ_ACK;
    ack_query_navigation_task_traj->head_.size_ = sizeof ( nsp__query_navigation_task_traj_ack_t) + ack_query_navigation_task_traj->cnt_trajs_ * sizeof ( trail_t);

    retval = tcp_write(link, ack_query_navigation_task_traj->head_.size_, &nsp__packet_maker, ack_query_navigation_task_traj);

    if (ack_query_navigation_task_traj && (ack_query_navigation_task_traj != &ack)) {
       free(ack_query_navigation_task_traj);
    }

    return retval;
}

////////////////////////////////////////////////////         操作任务相关协议      ////////////////////////////////////////////////////////////////////////////////
static
int nspi__on_allocate_operation_task(HTCPLINK link, const char *data, int cb) {
    nsp__allocate_operation_task_t *pkt_allocate_op_task = (nsp__allocate_operation_task_t *) data;
    nsp__allocate_operation_task_ack_t ack_allocate_op_task;
    var__operation_t *opt;

    memcpy(&ack_allocate_op_task.head_, &pkt_allocate_op_task->head_, sizeof (nsp__packet_head_t));
    ack_allocate_op_task.head_.type_ = PKTTYPE_ALLOC_OPERATION_TASK_ACK;
    ack_allocate_op_task.head_.size_ = sizeof (ack_allocate_op_task);
    ack_allocate_op_task.head_.err_ = -1;

    do {
        opt = NULL;

        if (cb < sizeof( nsp__allocate_operation_task_t)) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invalid request size for ansp__allocate_operation_task_t:%d", cb);
            ack_allocate_op_task.head_.err_ = -EINVAL;
            break;
        }

        opt = var__get_opt();
        if (!opt) {
            ack_allocate_op_task.head_.err_ = -ENOENT;
            break;
        }

        // new task id must not equal to current
        if (pkt_allocate_op_task->task_id_ == opt->i.current_task_id_) {
            log__save("motion_template", kLogLevel_Warning, kLogTarget_Filesystem | kLogTarget_Stdout, "operation new task id equal to current."UINT64_STRFMT, opt->i.current_task_id_);
            ack_allocate_op_task.head_.err_ = -EINVAL;
            break;
        }

        // 当前有导航任务正在执行， 不能在取消任务前执行任务覆盖( 中间状态非空闲也不能接收任何导航任务 )
        if (((opt->status_.response_ > kStatusDescribe_PendingFunction) && (opt->status_.response_ < kStatusDescribe_FinalFunction))
             || (opt->status_.middle_ != kStatusDescribe_Idle)) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "failed to allocate operation task,status response=%u, middle=%u", opt->status_.response_, opt->status_.middle_);
            ack_allocate_op_task.head_.err_ = -EBUSY;
            break;
        } 

        opt->code_ = pkt_allocate_op_task->optcode_;
        opt->status_.command_ = kStatusDescribe_Startup;
        opt->user_task_id_ = pkt_allocate_op_task->task_id_;
        ack_allocate_op_task.task_id_ = opt->user_task_id_;
        memcpy(&opt->param0_, pkt_allocate_op_task->params_, sizeof (pkt_allocate_op_task->params_));
        ++opt->ato_task_id_;
        ack_allocate_op_task.head_.err_ = 0;
    } while(0);

    if (opt) {
        var__release_object_reference( opt );
    }

    return tcp_write(link, ack_allocate_op_task.head_.size_, &nsp__packet_maker, &ack_allocate_op_task);
}

static
int nspi__on_cancel_operation_task(HTCPLINK link, const char *data, int cb) {
    nsp__cancel_operation_task_t *pkt_cancel_op_task = (nsp__cancel_operation_task_t *) data;
    nsp__control_operation_task_ack_t ack_cancel_op_task;
    var__operation_t *opt;

    memcpy(&ack_cancel_op_task.head_, &pkt_cancel_op_task->head_, sizeof (nsp__packet_head_t));
    ack_cancel_op_task.head_.type_ = PKTTYPE_CANCEL_OPERATION_TASK_ACK;
    ack_cancel_op_task.head_.size_ = sizeof (ack_cancel_op_task);
    ack_cancel_op_task.head_.err_ = 0;

    do {
        opt = NULL;

        if (cb < sizeof(nsp__cancel_operation_task_t)) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invalid request size for nsp__cancel_operation_task_t:%d", cb);
            ack_cancel_op_task.head_.err_ = -EINVAL;
            break;
        }

        opt = var__get_opt();
        if (!opt) {
            ack_cancel_op_task.head_.err_ = -ENOENT;
            break;
        }

        // 请求处理的ID必须符合正在执行的ID
        if (pkt_cancel_op_task->task_id_ != opt->i.current_task_id_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "operation task mismatch for cancel, current:"UINT64_STRFMT", request:"UINT64_STRFMT
                    , opt->i.current_task_id_, pkt_cancel_op_task->task_id_);
            ack_cancel_op_task.head_.err_ = -EEXIST;
            break;
        }

        opt->status_.command_ = kStatusDescribe_Cancel;
        ack_cancel_op_task.task_id_ = opt->i.current_task_id_;
        ack_cancel_op_task.optcode_ = opt->code_;
    }while(0);

    if (opt) {
        var__release_object_reference(opt);
    }

    return tcp_write(link, ack_cancel_op_task.head_.size_, &nsp__packet_maker, &ack_cancel_op_task);
}

static
int nspi__on_pause_operation_task(HTCPLINK link, const char *data, int cb) {
    nsp__pause_operation_task_t *pkt_pause_op_task = (nsp__pause_operation_task_t *) data;
    nsp__control_operation_task_ack_t ack_pause_op_task;
    var__operation_t *opt;

    memcpy(&ack_pause_op_task.head_, &pkt_pause_op_task->head_, sizeof (nsp__packet_head_t));
    ack_pause_op_task.head_.type_ = PKTTYPE_PAUSE_OPERATION_TASK_ACK;
    ack_pause_op_task.head_.size_ = sizeof (ack_pause_op_task);
    ack_pause_op_task.head_.err_ = 0;

    do {
        opt = NULL;

        if (cb < sizeof(nsp__pause_operation_task_t)) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invalid request size for nsp__pause_operation_task_t:%d", cb);
            ack_pause_op_task.head_.err_ = -EINVAL;
            break;
        }

        opt = var__get_opt();
        if (!opt) {
            ack_pause_op_task.head_.err_ = -ENOENT;
            break;
        }

        // 请求处理的ID必须符合正在执行的ID
        if (pkt_pause_op_task->task_id_ != opt->i.current_task_id_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "operation task mismatch for pause, current:"UINT64_STRFMT", request:"UINT64_STRFMT
                    , opt->i.current_task_id_, pkt_pause_op_task->task_id_);
            ack_pause_op_task.head_.err_ = -EEXIST;
            break;
        }

        opt->status_.command_ = kStatusDescribe_Pause;
        ack_pause_op_task.task_id_ = opt->i.current_task_id_;
        ack_pause_op_task.optcode_ = opt->code_;
    }while(0);

    if (opt) {
        var__release_object_reference(opt);
    }

    return tcp_write(link, ack_pause_op_task.head_.size_, &nsp__packet_maker, &ack_pause_op_task);
}

static
int nspi__on_resume_operation_task(HTCPLINK link, const char *data, int cb) {
    nsp__resume_operation_task_t *pkt_resume_op_task = (nsp__resume_operation_task_t *) data;
    nsp__control_operation_task_ack_t ack_resume_op_task;
    var__operation_t *opt;

    memcpy(&ack_resume_op_task.head_, &pkt_resume_op_task->head_, sizeof (nsp__packet_head_t));
    ack_resume_op_task.head_.type_ = PKTTYPE_RESUME_OPERATION_TASK_ACK;
    ack_resume_op_task.head_.size_ = sizeof (ack_resume_op_task);
    ack_resume_op_task.head_.err_ = 0;

    do {
        opt = NULL;
        
        if (cb < sizeof(nsp__resume_operation_task_t)) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, 
                "invalid request size for pkt_resume_op_task:%d", cb);
            ack_resume_op_task.head_.err_ = -EINVAL;
            break;
        }

        opt = var__get_opt();
        if (!opt) {
            ack_resume_op_task.head_.err_ = -ENOENT;
            break;
        }

        // 请求处理的ID必须符合正在执行的ID
        if (pkt_resume_op_task->task_id_ != opt->i.current_task_id_) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "operation task mismatch for resume, current:"UINT64_STRFMT", request:"UINT64_STRFMT
                    , opt->i.current_task_id_, pkt_resume_op_task->task_id_);
            ack_resume_op_task.head_.err_ = -EEXIST;
            break;
        }

        opt->status_.command_ = kStatusDescribe_Resume;
        ack_resume_op_task.task_id_ = opt->i.current_task_id_;
        ack_resume_op_task.optcode_ = opt->code_;
    }while(0);

    if (opt) {
        var__release_object_reference(opt);
    }

    return tcp_write(link, ack_resume_op_task.head_.size_, &nsp__packet_maker, &ack_resume_op_task);
}

static
int nspi__on_query_operation_parameter(HTCPLINK link, const char *data, int cb) {
    nsp__query_operation_parameters_ack_t ack_query_optpar;
    var__operation_t *opt;

    memcpy(&ack_query_optpar, data, sizeof ( nsp__packet_head_t));
    ack_query_optpar.type_ = PKTTYPE_QUERY_OPERATION_PARAMETER_ACK;
    ack_query_optpar.size_ = sizeof(nsp__query_operation_parameters_ack_t);
    ack_query_optpar.err_ = 0;

    do {
        opt = var__get_opt();
        if (!opt) {
            ack_query_optpar.err_ = -ENOENT;
            break;
        }

        // 当前有导航任务正在执行， 不能在取消任务前执行任务覆盖
        if ((opt->status_.response_ > kStatusDescribe_PendingFunction) 
            && (opt->status_.response_ < kStatusDescribe_FinalFunction)) {
            ack_query_optpar.err_ = -EBUSY;
            break;
        }

        opt->code_ = kOperationCode_QueryParameter;
        var__xchange_command_status(&opt->status_, kStatusDescribe_Startup, NULL);
    }while(0);

    if (opt) {
        var__release_object_reference(opt);
    }

    return tcp_write(link, ack_query_optpar.size_, &nsp__packet_maker, &ack_query_optpar);
}

/* 查询当前下位机运行内存中的所有对象简易信息(ID+TYPE) */
static
int nspi__on_query_var_list(HTCPLINK link, const char *data, int cb) {
    int varcnt;
    nsp__query_varlist_ack_t ack, *ack_query_varlst;
    int *ids;
    enum var__types *types;
    int i;
    nsp__query_varlist_t *req = ((nsp__query_varlist_t *) data);
    int retval;
    
    do {
        ack_query_varlst = &ack;
        ack_query_varlst->item_count_ = 0;
        
        varcnt = var__query_global_count();
        if (varcnt < 0) {
            ack_query_varlst->head_.err_ = -ERANGE;
            break;
        }
        
        ack_query_varlst = (nsp__query_varlist_ack_t *) malloc(sizeof (nsp__query_varlist_ack_t) + varcnt * sizeof (nsp__var_report_item_t));
        if (!ack_query_varlst) {
            ack_query_varlst->head_.err_ = -ENOMEM;
            break;
        }
        ack_query_varlst->item_count_ = varcnt;
        
        /* 构建 ID 序列  */
        ids = (int *) malloc(ack_query_varlst->item_count_ * sizeof (int));
        if (!ids) {
            ack_query_varlst->head_.err_ = -ENOMEM;
            break;
        }
        var__query_global_object_ids(ack_query_varlst->item_count_, ids);
        for (i = 0; i < ack_query_varlst->item_count_; i++) {
            ack_query_varlst->items[i].id_ = ids[i];
        }
        free(ids);

        /* 构建 TYPE 序列  */
        types = (enum var__types *)malloc(ack_query_varlst->item_count_ * sizeof (enum var__types));
        if (!types) {
            ack_query_varlst->head_.err_ = -ENOMEM;
            break;
        }
        var__query_global_object_types(ack_query_varlst->item_count_, types);
        for (i = 0; i < ack_query_varlst->item_count_; i++) {
            ack_query_varlst->items[i].type_ = (uint32_t) types[i];
        }
        free(types);

    }while(0);

    ack_query_varlst->head_.size_ = sizeof ( nsp__query_varlist_ack_t) + ack_query_varlst->item_count_ * sizeof ( nsp__var_report_item_t);
    ack_query_varlst->head_.id_ = req->id_;
    ack_query_varlst->head_.type_ = PKTTYPE_DBG_VARLS_ACK;
    ack_query_varlst->head_.err_ = 0;
    ack_query_varlst->item_count_ = varcnt;

    retval = tcp_write(link, ack_query_varlst->head_.size_, &nsp__packet_maker, ack_query_varlst);
    if (ack_query_varlst && ack_query_varlst != &ack) {
        free(ack_query_varlst);
    }
    return retval;
}

/* 清除错误结构中的所有 condition */
static
int nspi__on_clear_fault(HTCPLINK link, const char *data, int cb) {
    nsp__dbg_clear_fault_ack_t ack;
    nsp__dbg_clear_fault_t *req = ((nsp__dbg_clear_fault_t *) data);

    ack.id_ = req->id_;
    ack.type_ = PKTTYPE_DBG_CLEAR_FAULT_ACK;
    ack.size_ = sizeof ( nsp__dbg_clear_fault_ack_t);
    ack.err_ = var__clear_fault();
    return tcp_write(link, ack.size_, &nsp__packet_maker, &ack);
}

static
int nspi__on_report_status_msg(HTCPLINK link, const char *data, int cb) {
    struct nsp__report_status_msg *msg = (struct nsp__report_status_msg*) data;
    nsp__report_status_msg_ack ack;
    memset(&ack, 0, sizeof (ack));

    ack.id_ = msg->head_.id_;
    ack.size_ = sizeof (nsp__packet_head_t);
    ack.type_ = PKTTYPE_QUERY_REPORT_STATUS_ACK;
    ack.err_ = 0;
    
    if (cb < sizeof(struct nsp__report_status_msg)) {
        ack.err_ = -EINVAL;
    }else{
        nsp__report_status(msg->task_id_, msg->vartype_, msg->status_, msg->usrdat_, msg->usrcb_);
    }

    return tcp_write(link, ack.size_, &nsp__packet_maker, &ack);
}

static
int nspi__on_get_mtver(HTCPLINK link, const char *data, int cb) {
    static const uint16_t mt_major_version = 1;
    static const uint16_t mt_sub_version = 0;

    nsp__packet_head_t *request = (nsp__packet_head_t *)data;
    nsp__get_mtver_ack_t ack;

    ack.head_.id_ = request->id_;
    ack.head_.size_ = sizeof(nsp__get_mtver_ack_t);
    ack.head_.type_ = PKTTYPE_DBG_GET_MTVER_ACK;
    ack.head_.err_ = 0;

    ack.major = mt_major_version;
    ack.sub = mt_sub_version;
    ack.extended = 0;

    ack.namlen = sizeof(ack.uname);
    memset(ack.uname, 0, ack.namlen);
    ack.vcunamlen = sizeof(ack.vcu);
    memset(ack.vcu, 0, ack.vcunamlen);

    return tcp_write(link, ack.head_.size_, &nsp__packet_maker, &ack);
}

static
int nspi__on_keepalive_tcp(HTCPLINK link, const char *data, int cb) {
    nsp__packet_head_t *request = (nsp__packet_head_t *)data;
    nsp__packet_head_t ack;

    if (sizeof(nsp__packet_head_t) != cb ) {
        return -1;
    }

    memcpy(&ack, request, sizeof(nsp__packet_head_t));
    ack.type_ = request->type_ | PKTTYPE_ACK;

    return tcp_write(link, ack.size_, &nsp__packet_maker, &ack);
}

static
void nspi__on_raise_segmentfault(HTCPLINK link) {
    log__write("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "link 0x%08X initiative trigger segment fault.", link);
#if _WIN32
    __asm int 3    
#else
    raise(SIGSEGV);
#endif
}

static int nspi__on_allocate_offline_task(HTCPLINK link, const char *data, int cb) {
	nsp__allocate_offline_task_t *pkt_offline_task = (nsp__allocate_offline_task_t *)data;
	nsp__allocate_offline_task_ack_t ack_offline_task;
	var__navigation_t *nav;
	var__operation_t *opt;

	memcpy(&ack_offline_task.head_, &pkt_offline_task->head_, sizeof(nsp__packet_head_t));
	ack_offline_task.head_.type_ = PKTTYPE_ALLOC_OFFLINE_TASK_ACK;
	ack_offline_task.head_.size_ = sizeof (ack_offline_task);

	do {
		nav = var__get_navigation();
		if (nav) {
			if (((nav->track_status_.response_ > kStatusDescribe_PendingFunction) && (nav->track_status_.response_ < kStatusDescribe_FinalFunction))
				|| (nav->track_status_.middle_ != kStatusDescribe_Idle)
				) { // 如果已经有非离线导航任务在执行，那么拒绝接受离线任务 
				log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
					"failed to allocate offline task, navigation status response=%u, middle=%u", \
					nav->track_status_.response_, nav->track_status_.middle_);
				ack_offline_task.head_.err_ = -EBUSY;
				break;
			}
			var__release_object_reference(nav);
		}
		opt = var__get_opt();
		if (opt) {
			if (((opt->status_.response_ > kStatusDescribe_PendingFunction) && (opt->status_.response_ < kStatusDescribe_FinalFunction))
				|| (opt->status_.middle_ != kStatusDescribe_Idle)
				) { // 如果已经有非离线operation在执行，那么拒绝接受离线任务 
				log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
					"failed to allocate offline task, operation status response=%u, middle=%u", opt->status_.response_, opt->status_.middle_);
				ack_offline_task.head_.err_ = -EBUSY;
				break;
			}
			var__release_object_reference(opt);
		}
		// 设置新的离线任务对象 
		ack_offline_task.head_.err_ = var__set_offline_task(data, cb);
	} while (0);

	return tcp_write(link, ack_offline_task.head_.size_, &nsp__packet_maker, &ack_offline_task);
}

static int nspi__on_cancel_offline_task(HTCPLINK link, const char *data, int cb) {
	nsp__cancel_offline_task_t *pkt_cancel_offline_task = (nsp__cancel_offline_task_t *)data;
	nsp__cancel_offline_task_t ack_cancel_offline_task;
	var_offline_task_t *offline_task = NULL;

	memcpy(&ack_cancel_offline_task, pkt_cancel_offline_task, sizeof(nsp__cancel_offline_task_t));
	ack_cancel_offline_task.head_.type_ = PKTTYPE_CANCEL_OFFLINE_TASK_ACK;

	offline_task = var__get_offline_task();
	if (offline_task) {
		if (offline_task->user_task_id_ != pkt_cancel_offline_task->task_id_) {
			ack_cancel_offline_task.head_.err_ = -EEXIST;
		} else {
			// 直接设置离线任务状态未calcel，由customer取消正在执行的导航和operation后置为空闲状态 
			var__xchange_command_status(&offline_task->track_status_, kStatusDescribe_Cancel, NULL);
		}
		var__release_object_reference(offline_task);
	} else {
		ack_cancel_offline_task.head_.err_ = -ENOENT;
	}

	return tcp_write(link, ack_cancel_offline_task.head_.size_, &nsp__packet_maker, &ack_cancel_offline_task);
}

static int nspi__on_offline_navigation_task(const var_offline_task_t *offline_task) {
	var_offline_task_node_t *task_node = NULL;
	trail_t *trail_array = NULL;
	var__navigation_t *nav = NULL;
	var__vehicle_t *veh = NULL;
	int ret_value = 0;
	char *tmp_data = (char *)&offline_task->tasks_[0];
	int i, len = 0;

	for (i = 0; i < offline_task->task_current_exec_index_; i++) {
		task_node = (var_offline_task_node_t *)tmp_data;
		len += sizeof(var_offline_task_node_t) - 1;
		len += task_node->cnt_trals_ * sizeof(trail_t);
		len += task_node->cnt_opers_ * sizeof(var_offline_oper_t);
	}
	tmp_data += len;
	task_node = (var_offline_task_node_t *)tmp_data;
	trail_array = (trail_t *)&task_node->data[0];

	nav = var__get_navigation();
	do {
		if (!nav) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "navigation object is not existed.");
			ret_value = -ENOENT;
			break;
		}

		// the new task id equal to current, request will be declined
		if (task_node->task_id_ == nav->i.current_task_id_) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"offline navigation new task id equal to current."UINT64_STRFMT, nav->i.current_task_id_);
			ret_value = -EINVAL;
			break;
		}

		// dispatcher cannot cover navigation task before cancel existing one
		// also cannot cancel task when track status is in middle
		if (((nav->track_status_.response_ > kStatusDescribe_PendingFunction) &&
			(nav->track_status_.response_ < kStatusDescribe_FinalFunction)) ||
			(nav->track_status_.middle_ != kStatusDescribe_Idle)
			) {
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"failed to allocate navigation task,status response=%u, middle=%u", nav->track_status_.response_, nav->track_status_.middle_);
			ret_value = -EBUSY;
			break;
		}

		nav->is_traj_whole_ = 1;
		memcpy(&nav->dest_upl_, &task_node->dest_upl_, sizeof (task_node->dest_upl_));
		memcpy(&nav->dest_pos_, &task_node->dest_pos_, sizeof (task_node->dest_pos_));

		// release previous navigation trajectorys, reestablish the navigation sequence 
		if (nav->traj_ref_.data_ && nav->traj_ref_.count_ > 0) {
			free(nav->traj_ref_.data_);
		}
		nav->traj_ref_.count_ = 0;
		nav->traj_ref_.data_ = NULL;

		// protect the source request and count of trajs 
		nav->traj_ref_.data_ = malloc(task_node->cnt_trals_ * sizeof (trail_t));
		if (nav->traj_ref_.data_) {
			nav->traj_ref_.count_ = task_node->cnt_trals_;
			memcpy(nav->traj_ref_.data_, &trail_array, task_node->cnt_trals_ * sizeof (trail_t));
			var__xchange_command_status(&nav->track_status_, kStatusDescribe_Startup, NULL);
			nav->user_task_id_ = task_node->task_id_; // using the new task id
			posix__atomic_inc(&nav->ato_task_id_); // atomic increase inner task id
			log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,
				"successful allocate navigation task. user_id=%lld, ato_id=%lld", nav->user_task_id_, nav->ato_task_id_);
		} else{
			log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
				"failed to allocate navigation task because insuffcient memory.");
			ret_value = -ENOMEM;
			break;
		}

		// change control mode to navigation
		veh = var__get_vehicle();
		if (veh) {
			veh->control_mode_ = kVehicleControlMode_Navigation;
			var__release_object_reference(veh);
		}
	} while (0);
	if (nav) {
		var__release_object_reference(nav);
	}

	return ret_value;
}

static int nspi__on_offline_next_step(HTCPLINK link, const char *data, int cb) {
	nsp__cancel_offline_task_t *pkt_cancel_offline_task = (nsp__cancel_offline_task_t *)data;
	nsp__cancel_offline_task_t ack_cancel_offline_task;
	var_offline_task_t *offline_task = NULL;

	memcpy(&ack_cancel_offline_task, pkt_cancel_offline_task, sizeof(nsp__cancel_offline_task_t));
	ack_cancel_offline_task.head_.type_ = PKTTYPE_CANCEL_OFFLINE_TASK_ACK;

	offline_task = var__get_offline_task();
	if (offline_task) {
		if (offline_task->track_status_.command_ == kStatusDescribe_Idle) {
			var__xchange_command_status(&offline_task->track_status_, kStatusDescribe_Startup, NULL);
		}
		ack_cancel_offline_task.head_.err_ = nspi__on_offline_navigation_task(offline_task);
		offline_task->task_current_exec_index_ += 1;
		var__release_object_reference(offline_task);
	} else {
		ack_cancel_offline_task.head_.err_ = -ENOENT;
	}

	return tcp_write(link, ack_cancel_offline_task.head_.size_, &nsp__packet_maker, &ack_cancel_offline_task);
}

static
int nspi__on_nonsupport(HTCPLINK link, const char *data, int cb) {
    nsp__packet_head_t *head = (nsp__packet_head_t *)data;
    nsp__packet_head_t ack;

    ack.id_ = head->id_;
    ack.size_ = sizeof(nsp__packet_head_t);
    ack.type_ = head->type_ | PKTTYPE_ACK;
    ack.err_ = -ENOSYS;

    log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout, "unknown request packet 0x%08X recved", head->type_);
    return tcp_write(link, ack.size_, &nsp__packet_maker, &ack);
}

int nsp__on_tcp_recvdata(HTCPLINK link, const char *data, int cb) {
    nsp__packet_head_t *head;
    int retval;

    /*请求包不符合包头基本长度*/
    if (cb < sizeof ( nsp__packet_head_t)) {
        return -1;
    }

    /* 包头所载数据长度与收包长度不符 */
    head = (nsp__packet_head_t *) data;
    if (head->size_ != cb) {
        return -1;
    }

    // 登入操作不需要对链接作有效性判定
    if (run__if_must_login()) {
        if (PKTTYPE_LOGIN_ROBOT == head->type_) {
            return nspi__on_login(link, data, cb);
        }

        // 链接必须成功登入，并获得授权， 才能进行其他操作
        if (nsp__check_link_access_tcp(link) <= 0) {
            log__save("motion_template", kLogLevel_Error, kLogTarget_Filesystem | kLogTarget_Stdout,
                    "link 0x%08X access check failed, request declined.", link);
            return -1;
        }
    } else {
        nsp__refurbish_keepalive_status(link);
    }

    // 藉由包头对具体的包类型进行判定
    retval = -1;
    switch (head->type_) {
        case PKTTYPE_KEEPALIVE_TCP: // 不需要特殊处理, nsp__check_link_access 内已经重置了心跳计数
            retval = nspi__on_keepalive_tcp(link, data, cb);
            break;
        case PKTTYPE_COMMON_READ_BYID:
            retval = nspi__on_common_read_byid(link, data, cb);
            break;
        case PKTTYPE_COMMON_WRITE_BYID:
            retval = nspi__on_common_write_byid(link, data, cb);
            break;
        case PKTTYPE_COMMON_COMPARE_WRITE:
            retval = nspi__on_common_compare_write(link, data, cb);
            break;
        case PKTTYPE_INITIACTIVE_COMMON_READ:
            retval = nspi__regist_cycle_event(link, data, cb);
            break;
        case PKTTYPE_INITIACTIVE_COMMON_READ_CANCLE:
            retval = nspi__unregist_cycle_event(link, data, cb);
            break;
        case PKTTYPE_ALLOC_NAVIGATION_TASK:
            retval = nspi__on_allocate_navigation_task(link, data, cb);
            break;
        case PKTTYPE_CANCEL_NAVIGATION_TASK:
            retval = nspi__on_control_navigation_task_cancel(link, (const nsp__packet_head_t *) data);
            break;
        case PKTTYPE_PAUSE_NAVIGATION_TASK:
            retval = nspi__on_control_navigation_task_pause(link, (const nsp__packet_head_t *) data);
            break;
        case PKTTYPE_RESUME_NAVIGATION_TASK:
            retval = nspi__on_control_navigation_task_resume(link, (const nsp__packet_head_t *) data);
            break;
        case PKTTYPE_QUERY_NAVIGATION_TASK_TRAJ:
            retval = nspi__on_query_navigation_task_trajs(link, data, cb);
            break;
        case PKTTYPE_QUERY_NAVIGATION_TASK_STATUS:
            retval = nspi__on_query_navigation_task_status(link, data, cb);
            break;
        case PKTTYPE_ADDITIONAL_NAVIGATION_TRAJECTORY:
            retval = nspi__on_additional_navigation_trajectory(link, data, cb);
            break;

            // 分配， 取消， 暂停， 恢复操作任务
        case PKTTYPE_ALLOC_OPERATION_TASK:
            retval = nspi__on_allocate_operation_task(link, data, cb);
            break;
        case PKTTYPE_CANCEL_OPERATION_TASK:
            retval = nspi__on_cancel_operation_task(link, data, cb);
            break;
        case PKTTYPE_PAUSE_OPERATION_TASK:
            retval = nspi__on_pause_operation_task(link, data, cb);
            break;
        case PKTTYPE_RESUME_OPERATION_TASK:
            retval = nspi__on_resume_operation_task(link, data, cb);
            break;

        case PKTTYPE_COMMON_STATUS_REPORT_ACK:
            break;

            /*3段*/
        case PKTTYPE_QUERY_OPERATION_PARAMETER:
            retval = nspi__on_query_operation_parameter(link, data, cb);
            break;
        case PKTTYPE_QUERY_REPORT_STATUS:
            retval = nspi__on_report_status_msg(link, data, cb);
            break;
            /* DBG 段 */
        case PKTTYPE_DBG_VARLS:
            retval = nspi__on_query_var_list(link, data, cb);
            break;
        case PKTTYPE_DBG_CLEAR_FAULT:
            retval = nspi__on_clear_fault(link, data, cb);
            break;
        case PKTTYPE_DBG_GET_MTVER:
            retval = nspi__on_get_mtver(link, data, cb);
            break;
        case PKTTYPE_DBG_SIGSEGV:
            nspi__on_raise_segmentfault(link);
            break;

		case PKTTYPE_ALLOC_OFFLINE_TASK:
			retval = nspi__on_allocate_offline_task(link, data, cb);
			break;
		case PKTTYPE_CANCEL_OFFLINE_TASK:
			retval = nspi__on_cancel_offline_task(link, data, cb);
		case PKTTYPE_OFFLINE_NEXT_STEP:
			retval = nspi__on_offline_next_step(link, data, cb);
			break;

        default:
            retval = nspi__on_nonsupport(link, data, cb);
            break;
    }

    return retval;
}

// 处理UDP心跳报文 
static int nspi__on_keepalive_udp(HTCPLINK link, char *udp_addr, uint16_t udp_port, const char *data, int cb) {
	nsp__keepalive_udp_t *request = (nsp__keepalive_udp_t *)data;
	nsp__keepalive_udp_ack_t *ack = request;

	if (!udp_addr || sizeof(nsp__keepalive_udp_t) != cb) {
		return -1;
	}

	ack->head_.type_ |= PKTTYPE_ACK;
	ack->head_.err_ = 0;

	log__save("motion_template", kLogLevel_Info, kLogTarget_Filesystem | kLogTarget_Stdout,
		"return a keepalive udp packet to %s:%d.", udp_addr, udp_port);
	return udp_write(link, cb, &nsp__packet_maker, ack, udp_addr, udp_port);
}

// 接收UDP报文 
int nsp__on_udp_recvdata(HTCPLINK link, char *udp_addr, uint16_t udp_port, const char *data, int cb) {
	nsp__packet_head_t *head;
	int retval = 0;

	/*请求包不符合包头基本长度*/
	if (cb < sizeof (nsp__packet_head_t)) {
		return -1;
	}

	/* 包头所载数据长度与收包长度不符 */
	head = (nsp__packet_head_t *)data;
	if (head->size_ != cb) {
		return -1;
	}
	// 藉由包头对具体的包类型进行判定
	retval = -1;
	switch (head->type_) {
		case PKTTYPE_KEEPALIVE_UDP:
			retval = nspi__on_keepalive_udp(link, udp_addr, udp_port, data, cb);
			break;
	}

	return retval;
}
