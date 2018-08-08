#if !defined VAR_TYPES_H
#define VAR_TYPES_H

#if __cplusplus
#include "icom/compiler.h"
#else
#include "compiler.h"
#endif

#if !defined MAXIMUM_IPV4STR
#define MAXIMUM_IPV4STR         (16)
#endif

// 控制模式

enum control_mode_t {
    kDriverControlMode_Unknown = 0,
    kDriverControlMode_SpeedMode,
    kDriverControlMode_PositionMode,
    kDriverControlMode_CurrentMode,
};

// 预留对象ID, 兼容原有的网络协议宏， 废弃 common read 接口

enum var__fixed_object_id {
    kVarFixedObject_MotionTemplateFramwork = 0, // MT 框架预留
    kVarFixedObject_Navigation = 1, // 导航
    kVarFixedObject_Vehide, // 底盘
    kVarFixedObject_Operation, // 操作
    kVarFixedObject_UserDefinition, // 用户定义区
    kVarFixedObject_OptPar, // 操作参数
    kVarFixedObject_Map, // 地图
    kVarFixedObject_ErrorCollecter, // 错误采集
    kVarFixedObject_SaftyProtec, // 安全防护
    kVarFixedObject_InternalDIO, // 内部DIO/充电信息和指令

    kVarFixedObject_Virtual = 10, // virtual object with only functional head
    kVarFixedObject_Localization = kVarFixedObject_Virtual,
    kVarFixedObject_Driver0, 
    kVarFixedObject_Driver1,
    kVarFixedObject_Driver2,
    kVarFixedObject_Driver3,
    kVarFixedObject_Driver4,
    kVarFixedObject_Driver5,
    kVarFixedObject_Driver6,
    kVarFixedObject_Driver7,
    kVarFixedObject_VirtualLast = kVarFixedObject_Driver7,

    kVarReserved_MaximumFunction = 20, // 最大预留区间
};

// 全部的对象类型定义

enum var__types {
    kVarType_MotionTemplateFramwork = 0, /* MT 框架预留 */

    // var
    kVarType_Vehicle = 1, // 整车
    kVarType_Navigation, // 导航
    kVarType_Operation, // 定制操作
    kVarType_UserDefined, // 用户指定
    kVarType_SWheel, // 逻辑转向轮
    kVarType_DWheel, // 逻辑驱动轮
    kVarType_SDDExtra, // 逻辑 SDD 架构附加信息
    kVarType_DriveUnit, // 驱动单元
    kVarType_Map, // 地图
    kVarType_OperationTarget, // 操作目标参数表
    kVarType_Localization,

    // dev
    kVarType_CanBus = 0x100, // CAN 总线设备
    kVarType_Copley, // COPLEY 设备
    kVarType_Elmo, // ELMO 设备
    kVarType_DIO, // DIO 设备
    kVarType_Moons, // MOONS (步进电机)
    kVarType_AngleEncoder, // 角度编码器
    kVarType_Curtis, // 柯蒂斯控制器 设备
    kVarType_VoiceDevice, // 音响设备
    kVarType_PrivateDriver,

    // memnetdev
    kVarType_OmronPLC = 0x500, // 欧姆龙PLC网络设备
    kVarType_ModBus_TCP, // 网络协议的 modbus-TCP

    // logic
    kVarType_ErrorHandler = 0x1000,
    kVarType_SafetyProtec,

    // maximum
    kVarType_MaximumFunction = 0x7FFF,
};

enum prc_t {
    kPRC_NoAction = 0,
    kPRC_Pause,
    kPRC_Resume,
    kPRC_Cancel,
};

enum status_describe_t {
    kStatusDescribe_Unknown = 0,

    // 可发任务状态
    kStatusDescribe_Idle,
    kStatusDescribe_PrepareFinish, // 预完成状态， 提交阶段忽略
    kStatusDescribe_PendingFunction, // 以下为不可接受任务状态(直到最终状态触发)

    // 通常状态
    kStatusDescribe_Running,
    kStatusDescribe_Hang,
    kStatusDescribe_Alertable, // 可唤醒，但是是阻塞中， 等待重新调度

    // 命令
    kStatusDescribe_Startup,
    kStatusDescribe_Cancel,
    kStatusDescribe_Pause,
    kStatusDescribe_Resume,
    kStatusDescribe_Warning,

    // 最终状态， 当 response_ 处于最终状态时， 框架将 command_ 置为 Idle
    kStatusDescribe_FinalFunction,
    kStatusDescribe_Completed,
    kStatusDescribe_Terminated,
    kStatusDescribe_Error,
};

#pragma pack(push,1)

/* 定位和速度描述体 */
typedef struct _POSITION {
    double x_;
    double y_;

    union {
        double theta_;
        double angle_;
        double w_;
        double distance_;
    };
} position_t, velocity_t;

/* upl_t 结构 */
typedef struct _UPL {
    int edge_id_;
    double percentage_;

    union {
        int wop_id_;
        double angle_;
    };
} upl_t;

/* 边 WOP 属性， 包括WOP_ID */
typedef struct {
    int wop_id_;
    posix__boolean_t enabled_; /* 是否启用 */
    int wop_properties_[9]; /* 总共 9 个INT的边WOP参考属性，最后一个按位解释 */
} var__edge_wop_properties_t;

/* 简易向量表/通用不定长数组 */
typedef struct _var__vector {
    int count_;

    union {
        void *data_;
        uint64_t data64_;
    };
} var__vector_t;

/* 双通道状态迁移描述体 */
typedef struct {
    /*+000*/ enum status_describe_t command_;
    /*+004*/ enum status_describe_t middle_;
    /*+008*/ enum status_describe_t response_;
} var__status_describe_t; /*+012*/

typedef struct {
    /*+000*/ int canbus_; // CAN BUS ID
    /*+004*/ int canport_; // CAN PORT ID
    /*+008*/ int cannode_; // CAN NODE ID
    /*+012*/ uint64_t latency_; // 收到反馈瞬间的系统时间减去延迟即为反馈时间戳
    /*+020*/ int merge_; // 合并到 bus 的IO线程

    union {
        /*+024*/ void * self_rw_; // (如果不合并到bus的IO线程)自己的IO线程句柄
        uint64_t self_rw64_;
    };
    /*+032*/ int pdocnt_; // node 中的 PDO 数量
} var__can_device_t; /*+036*/

enum var__network_proto_types_t {
    kNetworkProto_Unknown = 0,
    kNetworkProto_TCP,
    kNetworkProto_UDP,
};

enum var__byte_orders_t {
    kNetworkByteOrder_Unknown = 0,
    kNetworkByteOrder_LittleEndian,
    kNetworkByteOrder_BigEndian,
};

typedef struct {
    /*+000*/ char ipv4_[MAXIMUM_IPV4STR];
    /*+016*/ uint16_t port_;
} var__endpoint_t;

typedef struct {
    /*+000*/ var__endpoint_t local_;
    /*+018*/ var__endpoint_t remote_;
    /*+036*/ enum var__network_proto_types_t protocol_;
    /*+040*/ enum var__byte_orders_t byte_order_;
} var__network_device_t; /*+044*/

#pragma pack(pop)

#define VAR_NAME_LENGTH  (64)

/* 最大对象容器容量 */
#define MAXIMUM_OBJECT_INDEX  (500)

enum nsp__controlor_type_t {
    kControlorType_Unknown = 0x100,
    kControlorType_NetworkIdentity = kControlorType_Unknown,
    kControlorType_Manual = kControlorType_NetworkIdentity,
    kControlorType_LSS = (kControlorType_NetworkIdentity << 1),                 /* 0x200 */
    kControlorType_LayoutDesigner = (kControlorType_NetworkIdentity << 2),      /* 0x400 */
    kControlorType_Localization = (kControlorType_NetworkIdentity << 3),        /* 0x800 */
    kControlorType_Robot = (kControlorType_NetworkIdentity << 4),               /* 0x1000 */
    kControlorType_TrafficControl = (kControlorType_NetworkIdentity << 5),      /* 0x2000 */
    kControlorType_RealtimeViewer = (kControlorType_NetworkIdentity << 6),      /* 0x4000 */
    kControlorType_Dispatcher = (kControlorType_NetworkIdentity << 7),          /* 0x8000 */
    kControlorType_Customer = (kControlorType_NetworkIdentity << 8),            /* 0x10000 */
    kControlorType_Guest = (kControlorType_NetworkIdentity << 9),               /* 0x20000 */
    kControlorType_MaximumFunctions = kControlorType_Guest,
};

#endif