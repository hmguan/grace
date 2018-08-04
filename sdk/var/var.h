#if !defined VAR_LAYOUT_H
#define VAR_LAYOUT_H

#if __cplusplus
#include "icom/object.h"
#include "icom/clist.h"
#include "icom/posix_thread.h"
#else
#include "object.h"
#include "clist.h"
#include "posix_thread.h"
#endif

#include "vartypes.h"

#pragma pack(push,1)

enum var__pagefault_mode_t {
    kPageFaultMode_ByteAlign = 0,
    kPageFaultMode_Bits,
};

/* 本对象处理数据换入的偏移列表(有序,安全) */
typedef struct {
    enum var__pagefault_mode_t mode_;      /* 按位交换或字节对齐 */
    objhld_t from_; // 换出数据对象
    int src_byte_offset_;      // 换入数据字节偏移
    int src_bit_offset_;      // 换入数据的字节内位偏移      
    int dst_byte_offset_;      // 换出数据字节偏移
    int dst_bit_offset_;      // 换出数据的字节内位偏移
    int len_;       //  字节长度 或 换位个数
    int bfill_;     /* 按位缺页的填充数据 */
    struct list_head entry_; /* 插入换入目标的 var__functional_object_t::pagefault_ */
} var__pagefault_mapping_t;

/* 功能性对象基本定义 */
typedef struct _var__functional_object {
    objhld_t handle_;
    objhld_t dup_; // (如果本对象复制于某对象)来源对象的句柄
    int object_id_;
    enum var__types type_;
    int length_;
    int body_length_;
    char name_[VAR_NAME_LENGTH];
    posix__pthread_mutex_t locker_; // 对象的安全锁定
    struct list_head pagefault_; // 内存换入序列
    struct list_head global_; // 全局所有的设备对象
    struct list_head local_; // 本地所有的设备对象(按类型)
#if _WIN32
#pragma warning(disable:4200)
#endif
    char body_[0];
#if _WIN32
#pragma warning(default:4200)
#endif
} var__functional_object_t;

#pragma pack(pop)

// 通过对象获取具体的数据指针
#define var__object_body_ptr(type, object) (type *)(&(((var__functional_object_t *)object)->body_))
#define var__object_head_ptr(ptr) containing_record(ptr, var__functional_object_t, body_)

// 获取和释放全局锁
void var__acquire_global_lock();
void var__release_global_lock();

// 获取和释放对象锁
#define var__acquire_lock(object)  ((!object) ? 0 : posix__pthread_mutex_lock(&object->locker_))
#define var__release_lock(object)  ((!object) ? 0 : posix__pthread_mutex_unlock(&object->locker_))
#define var__acquire_object_lock(ptr) ((!ptr) ? 0 : var__acquire_lock(var__object_head_ptr(ptr)))
#define var__release_object_lock(ptr) ((!ptr) ? 0 : var__release_lock(var__object_head_ptr(ptr)))

extern
int var__object_init(void *udata, void *ctx, int ctxcb);
extern
void var__object_uninit(int hld, void *udata);

/* 创建和销毁一个指定类型的对象(原始数据区)
 */
extern
int var__allocate_functional_object(int size, enum var__types type, var__functional_object_t **output);
extern
void var__delete_functional_object(var__functional_object_t *object);

/* 查询全局管理对象列表中维护的有效对象总量
 */
extern
int var__query_global_count();

/* 查询全局管理对象列表中，所有对象的基本信息 ID/TYPE
 */
extern
int var__query_global_object_ids(int count, int *ids);
extern
int var__query_global_object_types(int count, enum var__types *types);

/* 允许携带用户变量，遍历所有的对象ID, 遍历例程返回-1， 则遍历终止*/
extern
int var__traversal_global_object_ids(int (*todo)(int fixed_object_id, void *, int), void *usrptr, int usrcb);

/*插入一个对象到全局管理队列
 */
extern
void var__insert_object(var__functional_object_t *ob);

/* 根据对象ID, 快速索引对象句柄
 */
extern
objhld_t var__getobj_handle_byid(int object_id);

/* 根据对象名, 慢速索引对象句柄, 得到对象后， 应用程序应该至少记住对象ID, 后续可使用ID快速查找对象
 */
extern
objhld_t var__getobj_handle_byname(const char *objname);

/*  遍历所有的对象
        回调例程返回值定义:
        0.  继续遍历
        1.  终止遍历
        -1. 移除节点后继续遍历
        -2. 移除节点后终止遍历
 */
extern
void var__traverse_global_objects(int( *trfn)(var__functional_object_t *object, void *usrdat), void *usrdat);

/* 加载驱动配置信息 和 逻辑配置信息
 */
extern
int var__load_dev_configure();
extern
int var__load_var_configure();

/* 释放任何籍由 var__get_******() 函数所取得的原始数据指针引用
 */
extern
void var__release_object_reference(void *ref);
extern
void var__release_object_reference_byhead(var__functional_object_t *objhrd);

/* 置换状态描述体，得到前一个维护状态
 */
extern
void var__xchange_command_status(var__status_describe_t *sd, enum status_describe_t status, enum status_describe_t *previous);
extern
void var__xchange_middle_status(var__status_describe_t *sd, enum status_describe_t status, enum status_describe_t *previous);
extern
void var__xchange_response_status(var__status_describe_t *sd, enum status_describe_t status, enum status_describe_t *previous);
extern
void var__commit_status_describe_dup(var__status_describe_t *ori, const var__status_describe_t *dup);
extern
void var__init_status_describe(var__status_describe_t *sd);

/* convert configure by mmap
*/
extern
void var__convert_by_mapping();


#endif