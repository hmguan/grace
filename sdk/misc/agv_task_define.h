/*!
 * file agv_task_define.h
 * date 2017/08/03 16:53
 *
 * author chenkai
 *
 * brief 
 *
 * note
*/

#ifndef __AGV_TASK_DEFINE_H_CK__
#define __AGV_TASK_DEFINE_H_CK__
#include "operation.h"
#include <vector>
enum agv_task_types
{
    AgvTaskType_None = -1,
    AgvTaskType_Atom = 0,
    AgvTaskType_Combine,
    AgvTaskType_Offline,
};

enum agv_task_phase
{
    AgvTaskPhase_None = -1,
    AgvTaskPhase_Send = 0,
    AgvTaskPhase_WaitAck,
    AgvTaskPhase_CheckSend,
    AgvTaskPhase_WaitTraffic,
    AgvTaskPhase_Fin,
};

enum agv_taskstatus_check_types
{
    AgvTaskStatus_CheckStatus_None = -1,
    AgvTaskStatus_CheckStatus_Avaliable = 0,
    AgvTaskStatus_CheckStatus_Idle,
    AgvTaskStatus_CheckStatus_Final,
    AgvTaskStatus_CheckStatus_Running,
    AgvTaskStatus_CheckStatus_EqualStatus,
    AgvTaskStatus_CheckStatus_MultiAnd,
    AgvTaskStatus_CheckStatus_MultiOr,
};

enum agv_atom_task_types
{
    AgvAtomTaskType_None = -1,
    AgvAtomTaskType_Nav = 0,
    AgvAtomTaskType_Operation,
    AgvAtomTaskType_Multi,
};

enum agv_atom_taskdata_types
{
    AgvAtomTaskData_Header = 0,

    AgvAtomTaskData_Internal_Send = 0x100,//internal type

    AgvAtomTaskData_NewNav ,  
    AgvAtomTaskData_AddNav,
    AgvAtomTaskData_PauseNav,
    AgvAtomTaskData_ResumeNav,
    AgvAtomTaskData_CancelNav,
    AgvAtomTaskData_PathSearch,

    AgvAtomTaskData_NewOpt,
    AgvAtomTaskData_ModifyOpt,
    AgvAtomTaskData_PauseOpt,
    AgvAtomTaskData_ResumeOpt,
    AgvAtomTaskData_CancelOpt,

    AgvAtomTaskData_CheckStatus = 0x200,
    AgvAtomTaskData_CheckOpt,


};

enum agv_combine_task_types
{
    AgvCombineTaskType_None = -1,
    AgvCombineTaskType_Goto = 0,
    AgvCombineTaskType_Action,
    AgvCombineTaskType_GotoAction,
};

enum agv_combine_taskdata_types
{
    AgvCombineTaskData_Header = 0,

    AgvCombineTaskData_Internal_Send = 0x100,//internal type

    AgvCombineTaskData_NewGoto,
    AgvCombineTaskData_PauseGoto,
    AgvCombineTaskData_ResumeGoto,
    AgvCombineTaskData_CancelGoto,

    AgvCombineTaskData_NewAction,
    AgvCombineTaskData_PauseAction,
    AgvCombineTaskData_ResumeAction,
    AgvCombineTaskData_CancelAction,

    AgvCombineTaskData_NewGotoAction,
    AgvCombineTaskData_PauseGotoAction,
    AgvCombineTaskData_ResumeGotoAction,
    AgvCombineTaskData_CancelGotoAction,
    AgvCombineTaskData_CheckStatus = 0x200,


};

struct offline_task_item{
    int dock_id;
    std::vector<var__operation_t> ops;
};

#endif