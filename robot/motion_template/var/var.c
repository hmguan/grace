#include "var.h"
#include "elmo.h"
#include "voice.h"
#include "curtis.h"
#include "moons.h"
#include "wheel.h"
#include "bus.h"
#include "vehicle.h"
#include "drive_unit.h"
#include "navigation.h"
#include "operation.h"
#include "usrdef.h"
#include "angle_encoder.h"
#include "map.h"
#include "optpar.h"
#include "dio.h"
#include "memnetdev.h"
#include "safety.h"
#include "posix_string.h"
#include "posix_thread.h"
#include "posix_atomic.h"
#include "offlinetask.h"

typedef int( *config_proc_t)();

static struct {
    struct list_head all_objects_;
    int hld_mapping_[MAXIMUM_OBJECT_INDEX];
    posix__pthread_mutex_t mutex_;
    long inited_;
    int actived_object_count_;
}__var__kernal_manager;

int var__object_init(void *udata, void *ctx, int ctxcb) {
    return 0;
}

void var__object_uninit(int hld, void *udata) {
    ;
}

static 
int var__load_localization_object(){
    int retval;
    var__functional_object_t *object;
    retval = var__allocate_functional_object(0, kVarType_Localization, &object);
    if ((retval >= 0) && (object)) {
        object->object_id_ = kVarFixedObject_Localization;
        var__insert_object(object);
    }

    return retval;
}

static
void var__load_private_driver_object() {
    int i;
    var__functional_object_t *object;
    int retval;

    for ( i = kVarFixedObject_Driver0; i <= kVarFixedObject_Driver6; i++) {
        retval = var__allocate_functional_object(0, kVarType_PrivateDriver, &object);
        if ((retval >= 0) && (object)) {
            object->object_id_ = i;
            var__insert_object(object);
        }

        object = NULL;
    }
}

int var__load_dev_configure() {
    int retval = 0;
    int i = 0;
    
    static const config_proc_t config_procs[] = {
        &var__load_canbus,
        &var__load_elmo,
        &var__load_voice_device,
        &var__load_moos,
        &var__load_angle_encoder,
        &var__load_dio,
        &var__load_curtis,
        &var__load_memory_netdev,
    };

    for (i = 0; (i < sizeof ( config_procs) / sizeof ( config_procs[0])); i++) {
        retval = config_procs[i]();
    }
    return retval;
}

int var__load_var_configure() {
    int retval = 0;
    int i = 0;
    
    static const config_proc_t config_procs[] = {
        &var__load_vehicle,
        &var__load_swheel,
        &var__load_dwheel,
        &var__load_sddext,
        &var__load_driveunit,
        &var__load_navigation,
        &var__load_opt,
        &var__load_usrbuf,
        &var__load_layout,
        &var__load_optpar,
		&var__load_offlinetask,
        &var__load_safety,
    };
    for (i = 0; (i < sizeof ( config_procs) / sizeof ( config_procs[0])); i++) {
        retval = config_procs[i]();
    }

    /* create virtual object for localization and PSD/VCU driver */
    var__load_localization_object();
    var__load_private_driver_object();
    return retval;
}

int var__allocate_functional_object(int size, enum var__types type, var__functional_object_t **output) {
    objhld_t hld;
    var__functional_object_t *object;

    if (!output) return -1;
    *output = NULL;

    hld = objallo(size + sizeof ( var__functional_object_t), &var__object_init, &var__object_uninit, NULL, 0);
    if (hld < 0) {
        return -1;
    }

    // 创建阶段的引用不会被外部反引用， 只有触发 var__delete_functional_object 才会被真正反引用并释放
    object = objrefr(hld);
    if (!object) {
        return -1;
    }

    memset(object, 0, size + sizeof ( var__functional_object_t));
    object->type_ = type;
    object->length_ = size + sizeof ( var__functional_object_t);
    object->body_length_ = size;
    object->handle_ = hld;
    object->dup_ = -1;
    INIT_LIST_HEAD(&object->pagefault_);
    INIT_LIST_HEAD(&object->global_);
    INIT_LIST_HEAD(&object->local_);
    
    posix__pthread_mutex_init(&object->locker_);

    *output = object;
    return 0;
}

void var__delete_functional_object(var__functional_object_t *object) {
    if (object) {
        objdefr(object->handle_);
        objclos(object->handle_);
    }
}

int var__query_global_count(){
    int count = -1;
    
    if (__var__kernal_manager.inited_ <= 0){
        return -1;
    }
    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
    count = __var__kernal_manager.actived_object_count_;
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);
    return count;
}

int var__query_global_object_ids(int count, int *ids){
    int retval = -1;
    struct list_head *loop;
    int i;
    var__functional_object_t *obj;
    
    i = 0;
    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
    if (count == var__query_global_count() && ids){
        retval = 0;
        list_for_each(loop, &__var__kernal_manager.all_objects_){
            obj = containing_record(loop, var__functional_object_t, global_);
            if (obj) {
                ids[i] = obj->object_id_;
                if (++i >= count){
                    break;
                }
            }
        }
    }
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);
    return retval;
}

int var__query_global_object_types(int count, enum var__types *types){
    int retval = -1;
    struct list_head *loop;
    int i;
    var__functional_object_t *obj;
    
    i = 0;
    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
    if (count == var__query_global_count() && types){
        retval = 0;
        list_for_each(loop, &__var__kernal_manager.all_objects_){
            obj = containing_record(loop, var__functional_object_t, global_);
            if (obj) {
                types[i] = obj->type_;
                if (++i >= count){
                    break;
                }
            }
        }
    }
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);
    return retval;
}

int var__traversal_global_object_ids(int (*todo)(int fixed_object_id, void *, int), void *usrptr, int usrcb){
    struct list_head *loop;
    var__functional_object_t *obj;
    
    if (!todo){
        return -1;
    }
    
    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
    list_for_each(loop,  &__var__kernal_manager.all_objects_){
        obj = containing_record(loop, var__functional_object_t, global_);
        if (obj) {
            if (todo(obj->object_id_, usrptr, usrcb) < 0){
                break;
            }
        }
    }
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);
    
    return 0;
}

void var__insert_object(var__functional_object_t *ob) {
    if (1 == posix__atomic_inc(&__var__kernal_manager.inited_)) {
        posix__pthread_mutex_init(&__var__kernal_manager.mutex_);
        INIT_LIST_HEAD(&__var__kernal_manager.all_objects_);
        memset(__var__kernal_manager.hld_mapping_, -1, sizeof(__var__kernal_manager.hld_mapping_));
        __var__kernal_manager.actived_object_count_ = 0;
    }

    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
    
    /* 建立全局对象表 */
    list_add_tail(&ob->global_, &__var__kernal_manager.all_objects_);

    /* 为全局对象表建立快速索引, 此处硬性限制了最大对象ID（总数）不得超过 MAXIMUM_OBJECT_INDEX */
    if (ob->object_id_ < MAXIMUM_OBJECT_INDEX) {
        __var__kernal_manager.hld_mapping_[ob->object_id_] = ob->handle_;
    }
    
    /* 记录全局对象个数 */
    ++__var__kernal_manager.actived_object_count_;
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);
}

void var__delete_object(var__functional_object_t *ob) {

	posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);

	/* 从全局对象表删除 */
	__list_del_entry(&ob->global_);

	/* 为全局对象表建立快速索引, 此处硬性限制了最大对象ID（总数）不得超过 MAXIMUM_OBJECT_INDEX */
	if (ob->object_id_ < MAXIMUM_OBJECT_INDEX) {
		__var__kernal_manager.hld_mapping_[ob->object_id_] = -1;
	}

	/* 记录全局对象个数 */
	--__var__kernal_manager.actived_object_count_;
	posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);
}

objhld_t var__getobj_handle_byid(int object_id) {
    objhld_t hld;

    if ((object_id >= MAXIMUM_OBJECT_INDEX) || (__var__kernal_manager.inited_ <= 0) || (object_id < 0) ){
        return -1;
    }
    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
    hld = __var__kernal_manager.hld_mapping_[object_id];
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);

    return hld;
}

objhld_t var__getobj_handle_byname(const char *objname) {
    objhld_t hld;
    struct list_head *loop;
    var__functional_object_t *object;

    if ((!objname) ||  (__var__kernal_manager.inited_ <= 0)) {
        return -1;
    }

    hld = -1;

    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
    list_for_each(loop, &__var__kernal_manager.all_objects_){
        object = containing_record(loop, var__functional_object_t, global_);
        if (0 == posix__strcmp(objname, object->name_)) {
            hld = object->handle_;
            break;
        }
    }
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);

    return hld;
}

// 遍历全部对象
void var__traverse_global_objects(int( *trfn)(var__functional_object_t *object, void *usrdat), void *usrdat) {
    struct list_head *loop;
    var__functional_object_t *obj;

    if ((!trfn) ||  (__var__kernal_manager.inited_ <= 0) ) return;

    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
    list_for_each(loop, &__var__kernal_manager.all_objects_){
        obj = containing_record(loop, var__functional_object_t, global_);
        if (obj) {
            if (trfn(obj, usrdat) < 0) {
                break;
            }
        }
    }
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);
}

void var__release_object_reference(void *ref) {

    var__functional_object_t *obj;
    objhld_t hld;

    if (!ref) {
        return;
    }
    
    obj = var__object_head_ptr(ref);
    hld = obj->handle_;
    var__release_lock(obj);
    objdefr(hld);
}

void var__release_object_reference_byhead(var__functional_object_t *objhrd){
    
    objhld_t hld;
    
    if (!objhrd){
        return;
    }
    
    hld = objhrd->handle_;
    var__release_lock(objhrd);
    objdefr(hld);
}

void var__acquire_global_lock() {
    if  (__var__kernal_manager.inited_ <= 0){
        return;
    }
    posix__pthread_mutex_lock(&__var__kernal_manager.mutex_);
}

void var__release_global_lock() {
    if  (__var__kernal_manager.inited_ <= 0) {
        return;
    }
    posix__pthread_mutex_unlock(&__var__kernal_manager.mutex_);
}

void var__xchange_command_status(var__status_describe_t *sd, enum status_describe_t status, enum status_describe_t *previous) {
    if (previous){
        *previous = sd->command_;
    }
    sd->command_ = status;
}

void var__xchange_middle_status(var__status_describe_t *sd, enum status_describe_t status, enum status_describe_t *previous) {
    if (previous){
        *previous = sd->middle_;
    }
    sd->middle_ = status;
}

void var__xchange_response_status(var__status_describe_t *sd, enum status_describe_t status, enum status_describe_t *previous) {
    if (previous){
        *previous = sd->response_;
    }
    sd->response_ = status;
}

// 提交状态描述的拷贝对象
void var__commit_status_describe_dup(var__status_describe_t *ori, const var__status_describe_t *dup) {
    if (!ori || !dup) {
        return;
    }
    
    if (ori->response_ < kStatusDescribe_FinalFunction && dup->response_ > kStatusDescribe_FinalFunction) {
        // 当报告状态被置为最终状态时，命令/中间状态被强制置为空闲 
        ori->command_ = kStatusDescribe_Idle;
        ori->middle_ = kStatusDescribe_Idle;
        ori->response_ = dup->response_;
    } else {
        // 中间状态和报告状态均需要从拷贝对象提交到原始数据区
        ori->middle_ = dup->middle_;
        ori->response_ = dup->response_;
    }
}

void var__init_status_describe(var__status_describe_t *sd) {
    if (sd) {
        sd->command_ = sd->middle_ = sd->response_ = kStatusDescribe_Idle;
    }
}