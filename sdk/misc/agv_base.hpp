/*!
 * file agv_base.hpp
 * date 2017/08/02 14:55
 *
 * author chenkai
 *
 * brief
 *
 * note
 */
#ifndef _AGV_BASE_H__
#define _AGV_BASE_H__
#include <vector>
#include "navigation.h"
#include <string>
#include "operation.h"
#include <functional>
#include "vehicle.h"
#include <mutex>
#include "net_impls.h"
#include "layout_info.h"
#include "vehicle.h"
#include "optpar.h"
#include "task_scheduler.hpp"
#include <memory>
#include "agv_driver.h"
#include "safety.h"
#include "error.h"
#include "dio.h"
#include "log.h"
#include "mntypes.h"

#include "agv_atom_taskdata_base.h"
#include "agv_atom_taskdata_nav.h"
#include "agv_atom_taskdata_opt.h"
#include "agv_combine_taskdata_base.h"
#include "agv_combine_taskdata_gotoaction.h"
#include "agv_offline_taskdata.h"
#include "offlinetask.h"
class agv_base;


class agv_tp_task_base
{
public:
    agv_tp_task_base(agv_base* __agv, std::shared_ptr<agv_taskdata_base> taskdata);
    ~agv_tp_task_base();

    virtual void on_task() = 0;
protected:
    agv_base* __agv = nullptr;
    std::shared_ptr<agv_taskdata_base> __taskdata = nullptr;
};

class agv_base
{
public:
    agv_base(vehicle_type_t t, int is_gz_arm_core = 1);
    virtual ~agv_base();

public:
    void set_layoutinfo(const std::shared_ptr<layout_info> &layoutptr);
    vehicle_type_t get_agv_type() const;

    void set_agv_id(int id);
    int get_agv_id() const;

    int login(const std::string &ip, uint16_t port);
    int login(const char *epstr); // string of standard endpoint like 192.168.0.1:1234
    virtual int on_login();
    int logout();

    int update_endpoint(const std::string &ip, uint16_t port);

    //  status < 0 is offline
    int get_net_status(int& status) const;
    int is_ready(bool& rd);
    int clear_logout_flag();

    //must be called outside loop
    void update_agv_data();

    template<class T, class ...MEMBERS_T>
    int subscribe_data(int id, void *begin, T head, MEMBERS_T...members);

    virtual int sleep_agv_syn();
    virtual int wakeup_agv_syn();
    virtual int get_agv_sleep_status(int& sleep);//1=sleep  0=wakeup
public:
    struct via_dock_info
    {
        int dock_id = 0;
        int wait = 0;//等待周期次数（等待时间=update_agv_data的周期*wait ）  ，如果<0表示无限等待，需要手动触发
    };
    int detour_dock(std::vector<via_dock_info> via_docks);
    int go_on_detour(int dock);//触发继续绕路, 输入参数输入正在wait的dock点
    int get_detour_blocked_dock();//获取当前正在wait的dock点
public://get data
    int get_current_upl(upl_t& curUpl) const;
    int get_current_traj_idex(int& idex) const;
    int get_current_pos(position_t& pos, double& confident) const;

    //child should has it's own implement if it is not gz_arm_core
    virtual int get_voltage(double& vol) const;
    virtual int get_battery_power(double&)const;
    virtual int get_battery_capacity_current(double& cur);
    virtual int get_battery_capacity_total(double& total);
    int get_nav_dis2dest(double& dis) const;
    int get_nav_dis2partition(double& dis) const;
    int nav_is_on_last_segment(bool& on) const;
    int get_emergent_stop_state(bool& stop) const;
    virtual int get_fault_stop_state(bool& stop) const;
    int get_vehcile_enable_state(bool &enabled) const;

    int get_vehcile(var__vehicle_t& v) const;
    int get_nav_info(var__navigation_t& nav) const;
    int get_operation(var__operation_t& op) const;
    int get_fault(var__error_handler_t& f) const;
    int get_safety_info(var__safety_t& s) const;
    int get_fault_describe(std::vector<std::string> & v);
	int get_agv_obj_list_asyn(std::vector<mn::var_item>& obj);

    int get_internal_dio_info(var__dio_t& dio)const;

public: //task
    int goto_charge_task(int dock_id, int charge_on, uint64_t& task_id, std::function<void(uint64_t taskid, status_describe_t status, int err, void* user)> fn, void* user = nullptr);
    int pause_gotocharge();
    int resume_gotocharge();
    int cancel_gotocharge();

public://offline task

    int new_offline_task(const std::vector<offline_task_item>& vct_task,     //离线任务列表
        uint64_t& task_id, std::function<void(uint64_t taskid, status_describe_t status, int err, void* user)> fn, void* user = nullptr,
        std::function<void(int percent)> fn_asyn_send_rate = nullptr //离线任务下载进度 (0 ~ 100)，如果没有注册回调，则此接口同步阻塞式执行
        );
    int cancel_offline_task(uint64_t& task_id);

public://syn      
    int pause_nav_syn();
    int resume_nav_syn();
    int cancel_nav_syn();
    int emergent_stop_syn(bool stop);
    int get_nav_traj_syn(std::vector<trail_t>& pathUpl);
    int clear_fault();
    int enable_safety_syn(bool  enable);

    int new_operation_syn(const var__operation_t &op);
    virtual int modify_operation_syn(const var__operation_t &op);
    virtual int retry_operation_syn();
    int pause_operation_syn();
    int resume_operation_syn();
    int cancel_operation_syn();

	int get_agv_obj_list_syn(std::vector<mn::var_item>& obj);
protected: //child must have implement
    virtual int ready(bool& ready) = 0;
    // get children data，thread use father’s ,trriggered every cycle
    virtual int get_elongate_variable() = 0;
    //get enlongate variable ,if base not support
    virtual void get_elongate_variable_ack(uint32_t id, const void *data) = 0;

    virtual int goto_charge(int dock_id, int charge_on, uint64_t& task_id, std::function<void(uint64_t taskid, status_describe_t status, int err, void* user)> fn, void* user = nullptr) = 0;
    
    //  external  fault ,childrens' own fault info
    virtual int get_external_fault_describe(std::vector<std::string> & v) ;


    // var error handler
    virtual int convert_varobj_type2str(mn::var_item obj, std::string& type_str);
    virtual int convert_varobj_error2str(mn::var_item obj, var__error_item_t e, std::string& error_detail);

    //gz arm error
    virtual int convert_gzarm_driver_id2str(int gz_driver_id, std::string& driver_name);
    virtual int convert_gzarm_driver_error2str(int gz_driver_id, int driver_err, std::string& driver_err_str);
    virtual int convert_gzarm_vcu_error2str_bybit(int bit, std::string& vcu_err_str);
public:
    int push_task(uint64_t &task_id, std::shared_ptr<agv_combine_taskdata_gotoaction> task, std::function<void(uint64_t taskid, status_describe_t status, int err, void* user)> fn, void* user = nullptr);
    int pause_taskqueue();
    int resume_taskqueue();
    int cancel_taskqueue();

    int post_task2threadpool(std::shared_ptr<agv_tp_task_base> task);

public://do task 
    int goto_dock(int dock_id, uint64_t& goto_task_id,
		const std::function<void(uint64_t taskid, status_describe_t status, int err, void* user)> &fn, void* user = nullptr);
    int pause_goto();
    int resume_goto();
    int cancel_goto();
	void on_upl_task( const upl_t &dest_upl, const position_t &dest_pos, int is_traj_whole, const std::vector<trail_t> &vec_trail);
	int on_upl_nav_task(const upl_t &dest_upl, const position_t &dest_pos, int is_traj_whole, const std::vector<trail_t> &vec_trail);
	int query_task_opera_parm(uint32_t robot_id);
private:
    int do_opt_senquence(int dock, int opt, 
		const std::function<int(int dock, int opt, std::vector<std::shared_ptr<agv_atom_taskdata_base>>& v, void*)> &opt_logic,
		void* opt_logic_user, uint64_t& optseq_task_id, 
		const std::function<void(uint64_t taskid, status_describe_t status, int err, void* user)> &fn, void* user = nullptr);
    int pause_optseq();
    int resume_optseq();
    int cancel_optseq();

    int goto_action(int dock_id, int opt_id, uint64_t task_id
        , const std::function<void(int dock_id, int opt_id, std::vector<std::shared_ptr<agv_combine_taskdata_base>>& v, void* user)> &fn_locgic, void* logic_user
        , const std::function<void(uint64_t taskid, status_describe_t status, int err, void* user)> &fn_result, void* user = nullptr);
    int pause_gotoaction();
    int resume_gotoaction();
    int cancel_gotoaction();
public:
    template<typename T>
    int get_var_info_by_id_syn(int id, T& var);

    /*
    可变参数列表 模板函数  通用写

    示例：
    motion_interface mi;
    mi.init("127.0.0.1", 10001);

    var__operation_t op;
    op.code_ = 1;
    op.param0_ = 1000;

    std::vector<common_write_t> v;
    mi.common_write_by_id(kVarFixedObject_Operation, v, &op, &op.code_, &op.param0_);

    */
	template<class T, class ...MEMBERS_T> int common_write_by_id(int id, mn::common_data &vct_write, void *begin, T head, MEMBERS_T...members);
	template<class T, class ...MEMBERS_T> int make_common_writedata(int id, std::vector<mn::common_data_item> &vct_write, void *begin, T head, MEMBERS_T...members);

private:
    //为可变参数模板 特化一个空参数列表函数
	int common_write_by_id(int id, mn::common_data &, void*);
	int make_common_writedata(int id, std::vector<mn::common_data_item> &, void*);
    int subscribe_data(int id, void *begin);
protected:
    template<typename T>int get_var_info_by_id_asyn(int type);

public: //thread pool task ,base only
    virtual void on_task(std::shared_ptr<agv_taskdata_base>& taskdata);
	
public:
    std::shared_ptr<layout_info> __layout = nullptr;
    std::shared_ptr<AgvDriver> __agv_driver = nullptr;

private:
    void recv_event( int32_t net_id, const void *data, enum mn::notift_type_t type );
    void th_atom_task(std::shared_ptr<agv_atom_taskdata_base> task);
    void callback_navseq_status(status_describe_t status, int err);
    void post_navigation_task_ack(uint32_t id, const void *data);
    void common_read_ack(uint32_t id, const void *data);
    void set_nav_sequnece_prc(status_describe_t s);
    void pause_navigation_task_ack(uint32_t id, const void *data);
    void cancel_navigation_task_ack(uint32_t id, const void *data);
    void resume_navigation_task_ack(uint32_t id, const void *data);
    void add_navigation_task_ack(uint32_t id, const void *data);
    void callback_optseq_status(status_describe_t status, int err);
    void set_opt_sequnece_prc(status_describe_t s);
    void pause_opt_task_ack(uint32_t id, const void *data);
    void resume_opt_task_ack(uint32_t id, const void *data);
    void cancel_opt_task_ack(uint32_t id, const void *data);
    void post_opt_task_ack(uint32_t id, const void *data);
    void callback_combineseq_status(status_describe_t status, int err);
    void set_combine_sequence_prc(status_describe_t s);
    void th_combine_task(std::shared_ptr<agv_combine_taskdata_base> task);
    void goto_fin_cb(uint64_t task_id, status_describe_t status, int err, void* user);
    void action_fin_cb(uint64_t task_id, status_describe_t status, int err, void* user);
    void gotoaction_fin_cb(uint64_t task_id, status_describe_t status, int err, void* user);
    void check_nav_status_from_navseq(status_describe_t s);
    void check_nav_status_from_optseq(status_describe_t s);
    void check_opt_status(const var__operation_t& opt);
    void post_modopt_task_ack(uint32_t id, const void *data);
    void check_report_opt_status(status_describe_t s, uint64_t task_id);
    void check_report_nav_status_from_optseq(status_describe_t s, uint64_t task_id);
    void check_report_nav_status_from_navseq(status_describe_t s, uint64_t task_id);

    status_describe_t get_nav_sequence_prc();
    uint64_t get_autoinc_nav_id();
    status_describe_t get_combine_sequence_prc();
    status_describe_t get_opt_sequence_prc();

    int get_task_from_navseq(std::shared_ptr<agv_atom_taskdata_base>& task);
    int pathsearch(std::shared_ptr<agv_atom_taskdata_pathsearch> t);
    int newnav(std::shared_ptr<agv_atom_taskdata_newnav> task);
    int set_navseq_curtask_phase(agv_task_phase p, status_describe_t status = kStatusDescribe_Completed);
    int get_nav_sequence_cur_idex();
    int set_nav_task_id(uint64_t id);
    int get_cur_nav_task_id(uint64_t& id);
    int pausenav();
    int resumenav();
    int cancelnav();
    int addnav(std::shared_ptr<agv_atom_taskdata_addnav> task);
    int traffic_path_available_cb(int has_path);
    int update_navseq_cur_taskdata(std::shared_ptr<agv_atom_taskdata_base> task);
    int get_cur_opt_task_id(uint64_t& id);
    int get_task_from_optseq(std::shared_ptr<agv_atom_taskdata_base>& task);
    int set_optseq_curtask_phase(agv_task_phase p, status_describe_t status = kStatusDescribe_Completed, int idex = -1);
    int get_opt_sequence_cur_idex();
    int pauseopt();
    int resumeopt();
    int cancelopt();
    int newopt(std::shared_ptr<agv_atom_taskdata_newopt> task);
    int get_task_from_combineseq(std::shared_ptr<agv_combine_taskdata_base>& task);
    int set_combineseq_curtask_phase(agv_task_phase p, status_describe_t status = kStatusDescribe_Unknown, int err = 0);
    int get_cur_combine_gototask_id(uint64_t& id);
    int get_cur_combine_actiontask_id(uint64_t& id);
    int set_cur_combine_gototask_id(uint64_t id);
    int set_cur_combine_actiontask_id(uint64_t id);
    int get_cur_combine_queue_task(std::shared_ptr<agv_combine_taskdata_gotoaction>& task);
    int update_optseq_cur_taskdata(std::shared_ptr<agv_atom_taskdata_base>& task);
    int modifyopt(std::shared_ptr<agv_atom_taskdata_modifyopt>& task);

	std::string convert_error2str(mn::var_item obj, var__error_item_t e);
    std::string convert_obj_type2str(mn::var_item obj);
	std::string make_string_by_errorcode(mn::var_item obj, var__error_item_t e);
    status_describe_t get_combine_queue_task_prc();
    void set_combine_queue_task_prc(status_describe_t s);
    int callback_cur_combine_queue_task_status(uint64_t task_id, status_describe_t status, int err);

    void post_task(const std::shared_ptr<agv_taskdata_base>& t);
    int wait_task_tp_fin();
    int get_combineseq_header(std::shared_ptr<agv_combine_taskdata_header>& task);

    int get_gzarm_error_detail(std::vector<std::string>& v);
    int th_offline_task(std::shared_ptr<agv_offline_taskdata> p);
protected:
    int __agv_id;
    vehicle_type_t __agv_type;

    mutable std::mutex __mtx_nav;
    var__navigation_t __nav;

    mutable std::mutex __mtx_veh;
    var__vehicle_t __veh;

    mutable std::mutex __mtx_opt;
    var__operation_t __opr;

    std::mutex __mtx_op_par;
    var__operation_parameter_t __op_par;

    mutable std::mutex __mtx_safety;
    int __safety_stop = -1;
    var__safety_t __safety;

    mutable std::mutex __mtx_fault;
    int __fault_stop = -1;
    var__error_handler_t __fault;
	std::vector<mn::var_item> __vct_agv_obj_list;

    const int __kCommReadUniqueThres = 2;
    std::mutex __mtx_comm_read_unique;
    std::map<int, int> __map_comm_read_unique;

private:
    bool __logout_flag = false;

public:
    int32_t __net_id = 0;

private:
    std::mutex __mtx_nav_sequence;
    //std::mutex __mtx_nav_sequence_phase;
    std::mutex __mtx_nav_sequence_newnav;
    std::mutex __mtx_nav_sequence_pausenav;
    std::mutex __mtx_nav_sequence_resumenav;
    std::mutex __mtx_nav_sequence_cancelnav;
    std::mutex __mtx_nav_sequence_addnav;
    std::mutex __mtx_nav_sequence_traffic;
    std::vector<std::shared_ptr<agv_atom_taskdata_base>> __nav_sequence;
    int __cur_nav_sequence_idx = -1;
    status_describe_t __nav_sequence_prc = kStatusDescribe_Resume;
    std::atomic<uint64_t> __nav_task_id{ 0 };
    int __nav_path_iswhole = 0;

    std::atomic<uint64_t> __goto_task_id{ 0 };

    std::atomic<int> __net_status{ -1};
    std::atomic<int> __exit{ 1 };
    std::atomic<int> __opt_report_credible{ 0 };
    std::atomic<int> __nav_report_credible{ 0 };

    std::mutex __mtx_opt_sequence;
    //std::mutex __mtx_opt_sequence_phase;
    std::mutex __mtx_opt_sequence_newopt;
    std::mutex __mtx_opt_sequence_modopt;
    std::mutex __mtx_opt_sequence_cancelopt;
    std::mutex __mtx_opt_sequence_resumeopt;
    std::mutex __mtx_opt_sequence_pauseopt;
    std::vector<std::shared_ptr<agv_atom_taskdata_base>> __opt_sequence;
    int __cur_opt_sequence_idx = -1;
    status_describe_t __opt_sequence_prc = kStatusDescribe_Resume;

    std::atomic<int> __tp_task_counter{ 0 };
    std::atomic<int> __cb_counter{ 0 };
private://combine task sequence

    class combineseq_cancel_status
    {
    public:
        void init(bool hasgoto,bool hasaction)
        {
            has_goto = hasgoto;
            has_action = hasaction;
        }

        void update_goto_status(bool canceled)
        {
            goto_canceled = canceled;
        }

        void update_action_status(bool canceled)
        {
            action_canceled = canceled;
        }

        bool need_callback()
        {
             if (has_goto == goto_canceled
                 && has_action == action_canceled)
             {
                 return true;
             }
             return false;
        }

    private:

        bool goto_canceled = false;
        bool action_canceled = false;

        bool has_goto = false;
        bool has_action = false;
    };
    std::mutex __mtx_combine_sequence;
    std::mutex __mtx_combine_sequence_phase;
    std::vector<std::shared_ptr<agv_combine_taskdata_base>> __combine_sequence;
    int __cur_combine_sequence_idx = -1;
    status_describe_t __combine_sequence_prc = kStatusDescribe_Resume;
    combineseq_cancel_status __combineseq_cancel_status;
protected:
    std::atomic<uint64_t> __opt_task_id{ 0 };

private:
    std::mutex __mtx_combine_queue_task;
    std::vector<std::shared_ptr<agv_combine_taskdata_base>> __combine_queue_task;
    std::atomic<uint64_t> __combine_task_id{ 0 };
    int __cur_combine_queue_task_idx = -1;
    status_describe_t __combine_queue_task_prc = kStatusDescribe_Resume;

    private:
        int __is_gz_arm_core;
        mutable std::mutex __mtx_dio_main;
        var__dio_t __dio_main;
private:
    struct detour_wait_info
    {
        int dock_id = 0;
        int wait = 0;
        int wait_counter = 0;
        unsigned int wait_idx = 0;
    };
    std::mutex __mtx_map_detour_wait;
    std::map<int, std::shared_ptr<detour_wait_info>> __map_detour_wait;
    int get_cur_detourwait_dockinf(int idx, int size, std::shared_ptr<detour_wait_info>& dwi);


    int __detour_blocked_dock = -1;
    int __cache_idex = 0;
    std::vector<trail_t> __cache_path;

private:
    enum AGVINTERFACE_ERROR
    {
         kAgvInterfaceError_OK = 0,
         kAgvInterfaceError_NoDock,
         kAgvInterfaceError_PathSearch,
         kAgvInterfaceError_SendFailed,
    };
    AGVINTERFACE_ERROR __last_interface_error = kAgvInterfaceError_OK;
    std::string __last_interface_error_str;

private:
    void common_read_ack_subscrbe(uint32_t id, const void *data);

    mn::common_title __vec_post_common;
    std::map<int, mn::common_title>  __map_subscribe_data;

private:
    std::atomic<uint64_t> __offline_task_id{ 0 };
    status_describe_t __offline_prc = kStatusDescribe_Resume;
    std::mutex __mtx_offtask_ptr;
    std::shared_ptr<agv_offline_taskdata> __offline_task = nullptr;
    mutable std::mutex __mtx_offtask;
    var_offline_task __var_offtask;
};

template<typename T>
int agv_base::get_var_info_by_id_syn(int id, T& var)
{
	mn::common_title vec_post_common;
	mn::common_title_item post_common;
	post_common.varid = id;
	post_common.offset = 0;
	post_common.length = sizeof(T);
	vec_post_common.items.push_back(post_common);

	int callback_ret = -1;
	nsp::os::waitable_handle wait_request(0);
	mn::common_data* asio_data;
	int ret = post_common_read_request_by_id(__net_id, vec_post_common, 
		[&](uint32_t robot_id, const void *data) {
		if (!data)
		{
            loerror("agvbase") << "AgvBase:" << __agv_id << " post_common_read_request_by_id failed，!data，id = " << id;
			wait_request.sig();
			return;
		}
		 asio_data = (mn::common_data*)data;
		if (asio_data->err_ < 0)
		{                                                 
            loerror("agvbase") << "AgvBase:" << __agv_id << " post_common_read_request_by_id failed，asio_data->get_err() < 0，id = " << id;
			wait_request.sig();
			return;
		}
		if (((mn::common_data*)data)->items.size() == 0)
		{
			wait_request.sig();
			return;
		}
		var = *(T*)(((mn::common_data*)data)->items[0].data.c_str());
		callback_ret = 0;
		wait_request.sig();
	});
	if (ret < 0){//如果接口调用失败，那么直接返回
        	loerror("agvbase") << "AgvBase:" << __agv_id << " post_common_read_request_by_id failed，ret < 0，id = " << id;
		return callback_ret;
	}
	wait_request.wait();

	return callback_ret;
}

template<class T, class ...MEMBERS_T>
int agv_base::common_write_by_id(int id, mn::common_data &vct_write, void *begin, T head, MEMBERS_T...members) {
	mn::common_data_item node;
    node.varid = id;
    //node.length = sizeof(*head);
    node.offset = (char*)head - (char*)begin;
	node.data = std::string((const char *)head, sizeof(*head));
    vct_write.items.push_back(node);
    int argc = sizeof...(members);
    if (argc > 0) {
        return common_write_by_id(id, vct_write, begin, members...);
    }
    else
    {
        mn::asio_t asio_data_;
        nsp::os::waitable_handle water(0);
		int err = -1;
		int iRet = post_common_write_request_by_id(__net_id, vct_write,
			[&](uint32_t robot_id, const void *data) {
            if (!data) {
                err = -1;
                loerror("agvbase") << "AgvBase:" << __agv_id << " post_common_write_request failed，!data，id = " << id;
                water.sig();
                return;
            }

			asio_data_ = *(mn::asio_t*)(data);
            if (asio_data_.err_ < 0) {
                loerror("agvbase") << "AgvBase:" << __agv_id << " post_common_write_request failed，asio_data->get_err() < 0，id = " << id;
                err = -2;
                water.sig();
                return;
            }
			err = asio_data_.err_;
            water.sig();
        });
        if (iRet < 0)
        {
            loerror("agvbase") << "AgvBase:" << __agv_id << " post_common_write_request failed，ret < 0，id = " << id;
            return -1;
        }
        water.wait();
        return err;

    }

    return 0;
}


template<class T, class ...MEMBERS_T>
int agv_base::make_common_writedata(int id, std::vector<mn::common_data_item> &vct_write, void *begin, T head, MEMBERS_T...members)
{
	mn::common_data_item node;
    node.varid = id;
   // node.length = sizeof(*head);
    node.offset = (char*)head - (char*)begin;
	node.data = std::string((const char *)head, sizeof(*head));
    vct_write.push_back(node);
    int argc = sizeof...(members);
    if (argc > 0) {
        return make_common_writedata(id, vct_write, begin, members...);
    }

    return 0;
}

template<typename T>
int agv_base::get_var_info_by_id_asyn(int id)
{
    {
        std::lock_guard<decltype(__mtx_comm_read_unique)> guard(__mtx_comm_read_unique);
        std::map<int, int>::iterator itr = __map_comm_read_unique.find(id);
        if (itr == __map_comm_read_unique.end())
        {
            __map_comm_read_unique[id] = 0;
        } 
        else
        {
            if (__kCommReadUniqueThres < itr->second)
            {
                return -10;    //wait last ack
            }
        }

    }

	mn::common_title vec_post_common;
	mn::common_title_item post_common;
    post_common.varid = id;
    post_common.offset = 0;
	if (id == kVarFixedObject_OfflineTask) {
		post_common.length = offsetof(var_offline_task, tasks_[0]);
	} else {
		post_common.length = sizeof(T);
	}
    vec_post_common.items.push_back(post_common);


	//mn::common_data* comm_data;
	int r = mn::post_common_read_request_by_id(__net_id, vec_post_common, std::bind(&agv_base::common_read_ack, this, std::placeholders::_1, std::placeholders::_2));
    //int ret = post_common_read_request_by_id(__net_id, vec_post_common, asio);
    if (r < 0){//如果接口调用失败，那么直接返回
        loerror("agvbase") << "AgvBase:" << __agv_id << " post_common_read_request_by_id failed，ret < 0，id = " << id << " err=" << r;
        return -1;
    }

    {
        std::lock_guard<decltype(__mtx_comm_read_unique)> guard(__mtx_comm_read_unique);
        //__map_comm_read_unique[id]++;
    }
    return 0;
}


template<class T, class ...MEMBERS_T>
int agv_base::subscribe_data(int id, void *begin, T head, MEMBERS_T...members)
{
    auto& it = __map_subscribe_data.find(id);
    if (it != __map_subscribe_data.end())
    {
        __vec_post_common = it->second;
    }

    mn::common_title_item post_common;
    post_common.varid = id;
    post_common.offset = (char*)head - (char*)begin;
    post_common.length = sizeof(*head);
    __vec_post_common.items.push_back(post_common);
    
    int argc = sizeof...(members);
    //if (argc > 0)
    {
        return subscribe_data(id, begin, members...);
    }

    return 0;
}

#endif

