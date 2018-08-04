#if !defined VAR_DRIVE_UNIT_H
#define VAR_DRIVE_UNIT_H

#include "object.h"
#include "vartypes.h"
#include "clist.h"

enum drive_unit_type_t {
    kUnknown_DriveUnitType = -1,
    kOnlySWheel,
    kOnlyDWheel,
    kSDWheel,
    kSDDWheel,
};

#pragma pack(push, 1)

typedef struct _ASSOICATED {
    int object_id_;
    struct list_head link_;
} associated_object_t;

struct var__driveunit_config_t {
    /*+000*/ enum drive_unit_type_t unit_type_; // 驱动单元的类型[]
    /*+004*/ position_t install_; // 安装位置
    /*+028*/ position_t calibrated_; // 经过校准的安装位置
};

typedef struct _var__drive_unit {
    /*+000*/ enum drive_unit_type_t unit_type_; // 驱动单元的类型[]
    /*+004*/ position_t install_; // 安装位置
    /*+028*/ position_t calibrated_; // 经过校准的安装位置
    /*+052*/ struct list_head associated_; // 驱动单元关联的设备ID链表(associated_object_t)
    /*+060*/ int associated_cnt_; // 挂载个数
} var__drive_unit_t;

#pragma pack(pop)

#if !defined __cplusplus

extern
int var__load_driveunit();

/*得到 var__functional_object_t *
  手动调用 var__release_object_reference 释放
 */
extern
var__drive_unit_t *var__getunit(objhld_t hld);
extern
var__drive_unit_t *var__getunit_byid(int id);

/* 因为 drive unit 的对象数据区域不可变更， 因此这个引用为 const 属性返回， 不需要反引用
 */
extern
const struct list_head *var__get_driveunit(int *count);

/* drive unit 不存在可提交数据， 因此没有 commit 过程
 */
extern
var__drive_unit_t *var__create_driveunit_dup(objhld_t hld);
extern
void var__release_driveunit_dup(var__drive_unit_t *unit);

// 将 drive unit 下挂载的轮子解析为 var__functional_object_t 指针数组
// [functionals] 参数通过 var__drive_unit_t::associated_cnt_ 进行二维指针分配后， 再传参给本过程
// [functionals] 使用完成后， 逐个调用  var__release_wheel_dup 进行释放
extern
int var__driveunit_parse_to_functional(const var__drive_unit_t *unit, void **functionals, int *count);

#endif
#endif