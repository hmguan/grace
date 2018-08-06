#ifndef __AGV_OFFLINE_TASKDATA__H_
#define __AGV_OFFLINE_TASKDATA__H_

#include "agv_atom_taskdata_base.h"  
#include <vector>

class agv_offline_taskdata :public agv_taskdata_base
{
public:
    agv_offline_taskdata();
    ~agv_offline_taskdata();

    uint16_t __task_id;
    std::vector<offline_task_item> __vct_task;
    std::function<void(uint64_t taskid, status_describe_t status, int err, void* __user)> __fn_result = nullptr;
    void* __user = nullptr;
    std::function<void(int percent)> __fn_asyn_send_rate = nullptr;

    void callback_status(status_describe_t status, int err);
    void callback_rate(int per);
};
#endif


