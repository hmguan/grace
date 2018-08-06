#include "agv_offline_taskdata.h"


agv_offline_taskdata::agv_offline_taskdata()  :
agv_taskdata_base(AgvTaskType_Offline)
{
}


agv_offline_taskdata::~agv_offline_taskdata()
{
}

void agv_offline_taskdata::callback_status(status_describe_t status, int err)
{
    if (!__fn_result)
    {
        return;
    }
    if (status > kStatusDescribe_FinalFunction)
    {
        this->set_task_phase(AgvTaskPhase_None);
    }
    __fn_result(__task_id, status, err, __user);
}

void agv_offline_taskdata::callback_rate(int per)
{
     if (__fn_asyn_send_rate)
     {
         __fn_asyn_send_rate(per);
     }
}
