#include "agv_base.hpp"
#include "os_util.hpp"
#include "endpoint.h"
#include "net_impls.h"
#include "mntypes.h"


#include <iostream>
#include "usrdef.h"
#include "agv_atom_taskdata_nav.h"
#include "agv_atom_taskdata_opt.h"
#include "agv_interface_error_define.h"
#include "agv_combine_taskdata_action.h"
#include "agv_combine_taskdata_goto.h"
#include "singleton.hpp"
#include "task_threadpool.h"

#include "toolkit.h"

using namespace mn;

#define NAV_TASK_ID_UPPER  10
#define NAV_TASK_ID_LOWER  1
#define GZ_ARM_DRIVER_NUM  8

agv_tp_task_base::agv_tp_task_base( agv_base* agv, std::shared_ptr<agv_taskdata_base> taskdata )
	:__agv( agv ), __taskdata( taskdata ) {
}

agv_tp_task_base::~agv_tp_task_base() {
}

class agv_tp_task4agvbase :public agv_tp_task_base {
public:
	agv_tp_task4agvbase( agv_base* agv, std::shared_ptr<agv_taskdata_base> taskdata )
		:agv_tp_task_base( agv, taskdata ) {
	}
	~agv_tp_task4agvbase() {}

	void on_task();
protected:
private:
};

void agv_tp_task4agvbase::on_task() {
	if ( __agv ) {
		__agv->on_task( __taskdata );
	}
}


//static nsp::toolkit::task_thread_pool<agv_tp_task_base> g_task_threadpool( 100 );
static bool g_net_inited = false;

agv_base::agv_base(vehicle_type_t t ,int is_gz_arm_core)
	:__agv_type( t ) ,__is_gz_arm_core(is_gz_arm_core){

	if ( !g_net_inited ) {
		//init_broadcast();
		g_net_inited = true;
	}

    memset(&__nav, 0, sizeof(__nav));
    memset(&__veh, 0, sizeof(__veh));
    memset(&__opr, 0, sizeof(__opr));
    memset(&__safety, 0, sizeof(__safety));
    memset(&__fault, 0, sizeof(__fault));
    memset(&__dio_main, 0, sizeof(__dio_main));
}


agv_base::~agv_base() {
	cancel_taskqueue();
	__exit = 1;
	wait_task_tp_fin();
	
	loinfo("agvbase") << "AgvBase:" << __agv_id << " wait cb fin ing...";
	
	nsp::os::waitable_handle time_wait;
	while (__cb_counter > 0) {
        	loinfo("agvbase") << "AgvBase:" << __agv_id << " wait cb fin counter=" << __cb_counter;
        	time_wait.wait(1);
    	}
    	loinfo("agvbase") << "AgvBase:" << __agv_id << " wait cb fin ok";
    	logout();
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " agv_base::~agv_base()";
}


void agv_base::recv_event( int32_t net_id, const void *data, enum mn::notift_type_t type ) {
    if (__exit)
    {
        return;
    }
    ++__cb_counter;
	switch ( type ) {
		case mn::kNET_STATUS:
			{
				mn::net_status_t net_status = *( mn::net_status_t* )data;
				if ( net_status < 0 ) {
					loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " __net_status " << net_status << ",offline";
					__opt_report_credible = 0;
					__nav_report_credible = 0;
					__net_status = net_status;
				} else {
					loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " __net_status " << net_status << ",online";

					if ( __net_status < 0 ) {
                        __net_status = net_status;
                        {   //resume navseq
                            std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
                            get_task_from_navseq(task);
                            if (task
                                && task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
                                && task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus
                                && task->get_task_phase() == AgvTaskPhase_Send) {
                                loinfo("agvbase") << "AgvBase:" << __agv_id << " resume nav task, task->get_atomtaskdata_type()=" << task->get_atomtaskdata_type()
                                    << " task->get_task_phase()=" << task->get_task_phase();
                                post_task(task);
                            }
                        }

                        {   //resume optseq
                            std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
                            get_task_from_optseq(task);
                            if (task
                                && task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
                                && task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus
                                && task->get_task_phase() == AgvTaskPhase_Send) {
                                loinfo("agvbase") << "AgvBase:" << __agv_id << " resume opt task, task->get_atomtaskdata_type()=" << task->get_atomtaskdata_type()
                                    << " task->get_task_phase()=" << task->get_task_phase();
                                post_task(task);
                            }
                        }
					}

				}
			}
			break;
		case mn::kTASK_STATUS_REPORT:
			{
				mn::motion_report_t *r = ( mn::motion_report_t* )data;
				switch ( int( r->var_type_ ) ) {
					case kVarFixedObject_Operation:
						{
							__opt_report_credible = 1;

							loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " event : __opt_status" << r->status_ << " taskid=" << r->task_id_;
							check_report_opt_status( ( status_describe_t ) r->status_, r->task_id_ );

						}
						break;
					case kVarFixedObject_Navigation:
						{

							__nav_report_credible = 1;
							loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " event : __nav_status" << r->status_ << " taskid=" << r->task_id_;
							check_report_nav_status_from_navseq( ( status_describe_t ) r->status_, r->task_id_ );
							check_report_nav_status_from_optseq( ( status_describe_t ) r->status_, r->task_id_ );

						}
						break;
					default:
						break;
				}

			}
			break;
		default:
			break;
	}
    --__cb_counter;
}

void agv_base::set_layoutinfo( const std::shared_ptr<layout_info> &layoutptr ) {
	__layout = layoutptr;
}

vehicle_type_t agv_base::get_agv_type() const {
	return __agv_type;
}

void agv_base::set_agv_id( int id ) {
	__agv_id = id;
}

int agv_base::get_agv_id() const {
	return __agv_id;
}

int agv_base::login( const std::string &ip, uint16_t port ) {
	char epstr[32];
	sprintf( epstr, "%s:%u", ip.c_str(), port );
	return login(epstr);
}

int agv_base::login(const char *epstr) {
	mn::disconnect_host( __net_id );
	__net_id = mn::init_net();
	if ( __net_id <= 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " init_net failed";
		return -1;
	}

	if ( mn::login_to_host( __net_id, epstr, kControlorType_Dispatcher ) < 0 ) {
        loerror("agvbase") << "AgvBase:" << __agv_id << " net_id=" << __net_id << " login_to_host " << epstr<<" failed";
		return -1;
	}

	mn::register_callback_to_notify( __net_id, std::bind( &agv_base::recv_event, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );
	__net_status = mn::kNetworkStatus_Connected;
	__exit = 0;
	set_nav_sequnece_prc( kStatusDescribe_Resume );
	set_opt_sequnece_prc( kStatusDescribe_Resume );
	set_combine_sequence_prc( kStatusDescribe_Resume );
	set_combine_queue_task_prc( kStatusDescribe_Resume );

    on_login();
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " net_id=" << __net_id << " login_to_host "<<epstr<<" success!";

	return 0;
}

int agv_base::logout() {
	cancel_taskqueue();

	__exit = 1;

	wait_task_tp_fin();

	mn::disconnect_host( __net_id );
	__net_status = -1;

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " net_id=" << __net_id << " logout success!";
	return 0;
}

int agv_base::wait_task_tp_fin() {
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " wait_task_tp_fin ing...";

	nsp::os::waitable_handle time_wait;
	while ( __tp_task_counter > 0 ) {
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " wait_task_tp_fin counter=" << __tp_task_counter;
		time_wait.wait( 1 );
	}
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " wait_task_tp_fin ok";
	return 0;
}

int agv_base::is_ready( bool& rd ) {
	rd = false;
	do {
		if ( agv_base::__logout_flag == true ) {
			break;
		}

		upl_t cur_upl;
		if ( agv_base::get_current_upl( cur_upl ) < 0 ) {
			break;
		}

		if ( cur_upl.edge_id_ <= 0 ) {
			break;
		}

		position_t cur_pos;
		double pos_confident = 0;
		if ( agv_base::get_current_pos( cur_pos, pos_confident ) < 0 ) {
			break;
		}

		if ( pos_confident < 0.5 ) {
			break;
		}


		if ( ready( rd ) < 0 ) {
			break;
		}

		__opt_report_credible = 1;
		__nav_report_credible = 1;
		return 0;
	} while ( 0 );


	return 0;
}

int agv_base::clear_logout_flag() {
	__logout_flag = false;
	return 0;
}

int agv_base::get_current_upl( upl_t& curUpl ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_nav )> lock( __mtx_nav );
	curUpl = __nav.i.upl_;
	curUpl.percentage_ /= 100.0;
	return 0;
}

int agv_base::get_current_traj_idex( int& idex ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_nav )> lock( __mtx_nav );
	idex = __nav.i.traj_ref_index_curr_;
	return 0;
}

int agv_base::get_current_pos( position_t& pos, double& confident ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_nav )> lock( __mtx_nav );
	pos = __nav.pos_;
	confident = __nav.pos_confidence_;
	return 0;
}

int agv_base::common_write_by_id( int id, common_data &, void* ) {
	return 0;
}

int agv_base::goto_dock( int dock_id, uint64_t& goto_task_id, 
	const std::function<void( uint64_t taskid, status_describe_t status, int err, void* user )> &fn, void* user ) {
	static bool print_log = true;
	{
		std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
		if ( __cur_nav_sequence_idx != -1 ) {
			if ( print_log ) {
				loerror( "agvbase" ) << "AgvBase:" << __agv_id << " goto_dock dock= " << dock_id << " last task not finished!";
				print_log = false;
			}
			return -1;
		}
	}
	if ( !print_log ) {
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " goto_dock dock= " << dock_id << " last task finished!";
	}
	print_log = true;
	upl_t destUpl;
	position_t dest_pos;
	if ( __layout->get_dest_by_dockid( dock_id, destUpl, dest_pos ) < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " dock= " << dock_id << " not exist!";
        __last_interface_error = kAgvInterfaceError_NoDock;
        __last_interface_error_str = std::string("agvbase:dock ") + std::to_string(dock_id) + std::string(" not exist");
		return -2;
	}

	{
		std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
		__nav_sequence.clear();
		__nav_path_iswhole = 0;

		{
			goto_task_id = ++__goto_task_id;
			std::shared_ptr<agv_atom_nav_taskdata_header> p_header = std::shared_ptr<agv_atom_nav_taskdata_header>( new agv_atom_nav_taskdata_header );
			p_header->set_task_id( goto_task_id );
			p_header->set_atomtask_id( get_autoinc_nav_id() );
			p_header->set_task_status_callback( fn, user );
			__nav_sequence.push_back( p_header );
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new gotodock: idx=0 agv_atom_nav_taskdata_header";
		}

		{
			std::shared_ptr<agv_atom_taskdata_checkstatus_available> p_check_navavi = std::shared_ptr<agv_atom_taskdata_checkstatus_available>( new agv_atom_taskdata_checkstatus_available( AgvAtomTaskType_Nav ) );
			p_check_navavi->set_task_phase( AgvTaskPhase_WaitAck );
            p_check_navavi->set_atomtaskseq_idex(__nav_sequence.size());
			__nav_sequence.push_back( p_check_navavi );
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new gotodock: idx=1 agv_atom_taskdata_checkstatus_available";
		}

		{
			std::shared_ptr<agv_atom_taskdata_pathsearch> p_path = std::shared_ptr<agv_atom_taskdata_pathsearch>( new agv_atom_taskdata_pathsearch );
			p_path->destUpl = destUpl;
			p_path->set_task_phase( AgvTaskPhase_Send );
			__nav_sequence.push_back( p_path );
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new gotodock: idx=2 agv_atom_taskdata_pathsearch";

		}


		{
			std::shared_ptr<agv_atom_taskdata_newnav> p_newnav = std::shared_ptr<agv_atom_taskdata_newnav>( new agv_atom_taskdata_newnav );
			p_newnav->destPos = dest_pos;
			p_newnav->destUpl = destUpl;
			p_newnav->path_is_whole = 0;
			p_newnav->task_id = __nav_task_id;
			p_newnav->set_task_phase( AgvTaskPhase_WaitTraffic );
			__nav_sequence.push_back( p_newnav );
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new gotodock: idx=3 agv_atom_taskdata_newnav";

		}

		{
			std::shared_ptr<agv_atom_taskdata_addnav> p_addnav = std::shared_ptr<agv_atom_taskdata_addnav>( new agv_atom_taskdata_addnav );
			p_addnav->set_task_phase( AgvTaskPhase_WaitTraffic );
			__nav_sequence.push_back( p_addnav );
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new gotodock: idx=4 agv_atom_taskdata_addnav";
		}

		{
			std::shared_ptr<agv_atom_taskdata_checkstatus_final> p_check_navfin = std::shared_ptr<agv_atom_taskdata_checkstatus_final>( new agv_atom_taskdata_checkstatus_final( AgvAtomTaskType_Nav ) );
			p_check_navfin->set_task_phase( AgvTaskPhase_WaitAck );
            p_check_navfin->set_atomtaskseq_idex(__nav_sequence.size());
			__nav_sequence.push_back( p_check_navfin );
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new gotodock: idx=5 agv_atom_taskdata_checkstatus_final";
		}

		__cur_nav_sequence_idx = 1;
		__nav_sequence_prc = kStatusDescribe_Resume;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new gotodock= " << dock_id << " task_id=" << __goto_task_id << " idx=" << __cur_nav_sequence_idx << " navid=" << __nav_task_id;
	}

	return 0;
}

int agv_base::get_task_from_navseq( std::shared_ptr<agv_atom_taskdata_base>& task ) {
	task = nullptr;
	{
		if ( __net_status < 0 ) {
			return -1;
		}
		if ( __exit ) {
			return -1;
		}
		std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
		if ( __cur_nav_sequence_idx < 0
			|| __cur_nav_sequence_idx >= ( int ) __nav_sequence.size() ) {
			return -1;
		}

		switch ( __nav_sequence_prc ) {
			case kStatusDescribe_Resume:
				task = __nav_sequence[__cur_nav_sequence_idx];
				break;
			case kStatusDescribe_Pause:
				loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " get_task_from_navseq null, task paused";

				return 1;
			case kStatusDescribe_Cancel:
				loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " get_task_from_navseq null, task canceled";
				return -1;
			default:
				return -1;
		}

	}
	return 0;
}

int agv_base::set_navseq_curtask_phase( agv_task_phase p, status_describe_t s ) {
	bool navseq_complete = false;
	{
		std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
		if ( __cur_nav_sequence_idx < 0
			|| __cur_nav_sequence_idx >= ( int ) __nav_sequence.size() ) {
			return -1;
		}
        if (__nav_sequence[__cur_nav_sequence_idx]->get_task_phase() == p&& p==AgvTaskPhase_Fin)
        {
            lowarn("agvbase") << "AgvBase:" << __agv_id << " set_navseq_curtask_phase, __nav_sequence[__cur_nav_sequence_idx]->get_task_phase() == p ==AgvTaskPhase_Fin,muti set return 0";
            return 0;
        }

		__nav_sequence[__cur_nav_sequence_idx]->set_task_phase( p );

		if ( p == AgvTaskPhase_Fin ) {
			__cur_nav_sequence_idx++;
			if ( __cur_nav_sequence_idx >= ( int ) __nav_sequence.size() )    //finish
			{
				if ( __agv_driver ) {
					__agv_driver->FinishTaskPath();
				}
				navseq_complete = true;
			} else {
				loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " gotodock prepare trrigger next atomtask ,idx=" << __cur_nav_sequence_idx << " ";
			}
		}
	}

	if ( navseq_complete ) {
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " gotodock complete";

		callback_navseq_status( s, AgvInterfaceError_OK );
	} else {
		//trrigger next task
		if ( p == AgvTaskPhase_Fin ) {
			std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
			get_task_from_navseq( task );
			if ( task
				&& task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
				&& task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus ) {
				post_task( task );
			}
		}
	}

	return 0;
}

int agv_base::get_cur_nav_task_id( uint64_t& id ) {
	id = -1;
	{
		std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
		if ( __nav_sequence.empty() ) {
			return -1;
		}

		std::shared_ptr<agv_atom_nav_taskdata_header> task = std::static_pointer_cast< agv_atom_nav_taskdata_header, agv_atom_taskdata_base >( __nav_sequence[0] );
		id = task->get_atomtask_id();
	}

	return 0;
}

void agv_base::on_upl_task(const upl_t &dest_upl, const position_t &dest_pos, int is_traj_whole, const std::vector<trail_t> &vec_trail)
{
	// __goto_task_id++;
	__nav_task_id++;
	std::shared_ptr<agv_atom_taskdata_newnav> task = std::make_shared<agv_atom_taskdata_newnav>();
	//task->set_task_id(goto_task_id);
	task->set_task_phase(AgvTaskPhase_Send);
	task->task_id = __nav_task_id;
	task->path_is_whole = is_traj_whole;
	task->destPos = dest_pos;
	task->destUpl = dest_upl;
	task->pathUpl = vec_trail;

	post_task(task);
}

int agv_base::on_upl_nav_task(const upl_t &dest_upl, const position_t &dest_pos, int is_traj_whole, const std::vector<trail_t> &vec_trail)
{
	__nav_task_id++;

	int err = 0;
	nsp::os::waitable_handle wait_request(0);
	task_status_t* asio_data;
	int r = post_navigation_task(__net_id, __nav_task_id, dest_upl, dest_pos, is_traj_whole, vec_trail,
		[&](uint32_t robot_id, const void *data) {

		asio_data = (task_status_t*)data;
		if (!data) {
			err = asio_data->err_;
			wait_request.sig();
			return;
		}
		if (asio_data->err_ < 0) {
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		wait_request.sig();
	});

	if (r < 0) {
		return -1;
	}

	wait_request.wait();
	return err;
}

int agv_base::query_task_opera_parm(uint32_t robot_id)
{
	asio_t *asio;
	nsp::os::waitable_handle waiter(0);
	int retval = -1;
	int ret = query_task_opera_parm_order(__net_id, [&](uint32_t robot_id, const void *data) {
		if (!data) {
			waiter.sig();
			return;
		}

		asio = (asio_t*)data;
		retval = asio->err_;
		waiter.sig();
	});
	if (ret < 0) {
		return retval;
	}
	waiter.wait();
	if (retval < 0)	{
		return -2;
	}
	return 0;
}

void agv_base::on_task( std::shared_ptr<agv_taskdata_base>& taskdata ) {
	switch ( taskdata->get_taskdata_type() ) {
		case AgvTaskType_Atom:
			{
				std::shared_ptr<agv_atom_taskdata_base> task = std::static_pointer_cast< agv_atom_taskdata_base, agv_taskdata_base >( taskdata );
				th_atom_task( task );
			}

			break;
		case AgvTaskType_Combine:
			{
				std::shared_ptr<agv_combine_taskdata_base> task = std::static_pointer_cast< agv_combine_taskdata_base, agv_taskdata_base >( taskdata );
				th_combine_task( task );
			}
			break;
        case AgvTaskType_Offline:
        
            {
                std::shared_ptr<agv_offline_taskdata> task = std::static_pointer_cast<agv_offline_taskdata, agv_taskdata_base>(taskdata);
                th_offline_task(task);
            }
        
            break;
		default:
			break;
	}

	--__tp_task_counter;
}

void agv_base::goto_fin_cb( uint64_t task_id, status_describe_t status, int err, void* user ) {
	std::lock_guard<decltype( __mtx_combine_sequence_phase )> lock( __mtx_combine_sequence_phase );
	//check task
	uint64_t nav_id = -1;
	if ( get_cur_combine_gototask_id( nav_id ) < 0 ) {
		return;
	}

	if ( get_combine_sequence_prc() == kStatusDescribe_Cancel ) {
		__combineseq_cancel_status.update_goto_status( status > kStatusDescribe_FinalFunction );
		if ( __combineseq_cancel_status.need_callback() ) {
			callback_combineseq_status( kStatusDescribe_Terminated, err );
		}
	} else {
		std::shared_ptr<agv_combine_taskdata_base> task = nullptr;
		get_task_from_combineseq( task );
		if ( task ) {
			if ( task->get_combine_taskdata_type() == AgvCombineTaskData_CheckStatus ) {
				std::shared_ptr<agv_combine_taskdata_check_base> ck = std::static_pointer_cast< agv_combine_taskdata_check_base, agv_combine_taskdata_base >( task );
				if ( ck->get_combine_task_type() == AgvCombineTaskType_Goto ) {
					if ( ck->check_status( status, nav_id == task_id ) == 0 ) {
						set_combineseq_curtask_phase( AgvTaskPhase_Fin, status, err );
					}
				}
			}
		}
	}
}

void agv_base::action_fin_cb( uint64_t task_id, status_describe_t status, int err, void* user ) {
	std::lock_guard<decltype( __mtx_combine_sequence_phase )> lock( __mtx_combine_sequence_phase );
	//check task
	uint64_t action_id = -1;
	if ( get_cur_combine_actiontask_id( action_id ) < 0 ) {
		return;
	}
	if ( get_combine_sequence_prc() == kStatusDescribe_Cancel ) {
		__combineseq_cancel_status.update_action_status( status > kStatusDescribe_FinalFunction );
		if ( __combineseq_cancel_status.need_callback() ) {
			callback_combineseq_status( kStatusDescribe_Terminated, err );
		}
	} else {
		std::shared_ptr<agv_combine_taskdata_base> task = nullptr;
		get_task_from_combineseq( task );
		if ( task ) {
			if ( task->get_combine_taskdata_type() == AgvCombineTaskData_CheckStatus ) {
				std::shared_ptr<agv_combine_taskdata_check_base> ck = std::static_pointer_cast< agv_combine_taskdata_check_base, agv_combine_taskdata_base >( task );
				if ( ck->get_combine_task_type() == AgvCombineTaskType_Action ) {
					if ( ck->check_status( status, action_id == task_id ) == 0 ) {
						set_combineseq_curtask_phase( AgvTaskPhase_Fin, status, err );
					}
				}
			}
		}
	}
}

void agv_base::gotoaction_fin_cb( uint64_t task_id, status_describe_t status, int err, void* user ) {
	callback_cur_combine_queue_task_status( task_id, status, err );

	std::shared_ptr<agv_combine_taskdata_gotoaction> ct = nullptr;
	if ( get_cur_combine_queue_task( ct ) < 0 ) {
		return;
	}
	if ( ct && task_id == ct->get_task_id() ) {
		if ( status > kStatusDescribe_FinalFunction ) {

			{
				std::lock_guard<decltype( __mtx_combine_queue_task )> lock( __mtx_combine_queue_task );
				__combine_queue_task[__cur_combine_queue_task_idx] = nullptr;
				__cur_combine_queue_task_idx++;
			}

			if ( status != kStatusDescribe_Error ) {
				//trrigger next task 

				std::shared_ptr<agv_combine_taskdata_gotoaction> t = nullptr;
				if ( get_cur_combine_queue_task( t ) < 0 ) {
					return;
				}

				if ( t ) {
					post_task( t );
				}
			}

		}
	}
}


void agv_base::th_combine_task( std::shared_ptr<agv_combine_taskdata_base> task ) {
	if ( !task ) {
		return;
	}
	switch ( task->get_combine_taskdata_type() ) {
		case AgvCombineTaskData_NewGoto:
			{
				std::shared_ptr<agv_combine_taskdata_goto> t = std::static_pointer_cast< agv_combine_taskdata_goto, agv_combine_taskdata_base >( task );
				do {
					std::lock_guard<decltype( __mtx_combine_sequence_phase )> lock( __mtx_combine_sequence_phase );
					uint64_t goto_id = 0;
					int e = goto_dock( t->dock_id, goto_id, std::bind( &agv_base::goto_fin_cb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 ) );
					if ( e < 0 ) {
						post_task( task );
						break;
					}

					set_cur_combine_gototask_id( goto_id );
					set_combineseq_curtask_phase( AgvTaskPhase_Fin );
				} while ( 0 );

			}
			break;
		case AgvCombineTaskData_NewAction:
			{
				std::shared_ptr<agv_combine_taskdata_action> t = std::static_pointer_cast< agv_combine_taskdata_action, agv_combine_taskdata_base >( task );
				do {
					std::lock_guard<decltype( __mtx_combine_sequence_phase )> lock( __mtx_combine_sequence_phase );
					uint64_t task_id = 0;
					int e = do_opt_senquence( t->dock_id, t->opt_id, t->fn_opt_logic, t->user, task_id, std::bind( &agv_base::action_fin_cb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 ) );
					if ( e < 0 ) {
						post_task( task );
						break;
					}

					set_cur_combine_actiontask_id( task_id );
					set_combineseq_curtask_phase( AgvTaskPhase_Fin );
				} while ( 0 );
			}
			break;
		case AgvCombineTaskData_NewGotoAction:
			{
				std::shared_ptr<agv_combine_taskdata_gotoaction> t = std::static_pointer_cast< agv_combine_taskdata_gotoaction, agv_combine_taskdata_base >( task );
				if ( goto_action( t->dock_id, t->opt_id, t->get_task_id(), t->fn_action_logic, t->logic_user
					, std::bind( &agv_base::gotoaction_fin_cb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 )
					) < 0
					) {
					post_task( task );
					break;
				}
			}
			break;
		case AgvCombineTaskData_CancelGotoAction:
			{
                                                    int ret = -1;
                                                    do
                                                    {
                                                        if (cancel_goto() < 0)
                                                        {
                                                            break;
                                                        }
                                                        if (cancel_optseq() < 0)
                                                        {
                                                            break;
                                                        }
                                                        ret = 0;
                                                    } while (0);

                                                    if (ret < 0)
                                                    {
                                                        post_task(task);
                                                    }
			}
			break;
		case AgvCombineTaskData_ResumeGotoAction:
			{
				resume_optseq();
				resume_goto();
			}
			break;
		case AgvCombineTaskData_PauseGotoAction:
			{
				pause_optseq();
				pause_goto();
			}
			break;
		default:
			break;
	}
}

void agv_base::th_atom_task( std::shared_ptr<agv_atom_taskdata_base> task ) {
	if ( !task ) {
		return;
	}
	switch ( task->get_atomtaskdata_type() ) {
		case AgvAtomTaskData_PathSearch:
			{
				std::shared_ptr<agv_atom_taskdata_pathsearch> t = std::static_pointer_cast< agv_atom_taskdata_pathsearch, agv_atom_taskdata_base >( task );
				pathsearch( t );
			}
			break;
		case AgvAtomTaskData_NewNav:
			{
				std::shared_ptr<agv_atom_taskdata_newnav> t = std::static_pointer_cast< agv_atom_taskdata_newnav, agv_atom_taskdata_base >( task );
				newnav( t );
			}
			break;
		case AgvAtomTaskData_AddNav:
			{
				std::shared_ptr<agv_atom_taskdata_addnav> t = std::static_pointer_cast< agv_atom_taskdata_addnav, agv_atom_taskdata_base >( task );
				addnav( t );
			}
			break;
		case AgvAtomTaskData_PauseNav:
			{
				pausenav();
			}
			break;
		case AgvAtomTaskData_ResumeNav:
			{
				resumenav();
			}
			break;
		case AgvAtomTaskData_CancelNav:
			{
				cancelnav();
			}
			break;
		case AgvAtomTaskData_NewOpt:
			{
				std::shared_ptr<agv_atom_taskdata_newopt> t = std::static_pointer_cast< agv_atom_taskdata_newopt, agv_atom_taskdata_base >( task );
				newopt( t );
			}
			break;
		case AgvAtomTaskData_PauseOpt:
			{
				pauseopt();
			}
			break;
		case AgvAtomTaskData_ResumeOpt:
			{
				resumeopt();
			}
			break;
		case AgvAtomTaskData_CancelOpt:
			{
				cancelopt();
			}
			break;
		case AgvAtomTaskData_ModifyOpt:
			{
				std::shared_ptr<agv_atom_taskdata_modifyopt> t = std::static_pointer_cast< agv_atom_taskdata_modifyopt, agv_atom_taskdata_base >( task );
				modifyopt( t );
			}
			break;
		default:
			break;
	}
}

int agv_base::pathsearch( std::shared_ptr<agv_atom_taskdata_pathsearch> t ) {
	//std::lock_guard<decltype( __mtx_nav_sequence_phase )> lock( __mtx_nav_sequence_phase );
	if ( t->get_task_phase() != AgvTaskPhase_Send ) {
		return 0;
	}
	if ( __net_status < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " __net_status < 0,path_search paused";
		return -1;
	}
	upl_t cur_upl = __nav.i.upl_;
	cur_upl.percentage_ /= 100;
	double dis = 0;
	std::vector<trail_t> pathUpl;
	if ( __layout->path_search( cur_upl, t->destUpl, pathUpl, dis ) < 0 ) {
		//err
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " path_search from (" << cur_upl.edge_id_ << " " << cur_upl.percentage_ << " " << cur_upl.angle_ << ") to("
			<< t->destUpl.edge_id_ << " " << t->destUpl.percentage_ << " " << t->destUpl.angle_ << ") failed!";

		callback_navseq_status( kStatusDescribe_Error, AgvInterfaceError_PathSearch );
        __last_interface_error = kAgvInterfaceError_PathSearch;
        __last_interface_error_str = std::string("agvbase: path_search failed, from edge ") + std::to_string(cur_upl.edge_id_) + std::string(" to")
            + std::string(" edge ")+ std::to_string(t->destUpl.edge_id_);
		return -1;
	}

	//set traffic
	if ( __agv_driver ) {
		__agv_driver->SetAllocatedAvailabeCallback( std::bind( &agv_base::traffic_path_available_cb, this, std::placeholders::_1 ) );
		__agv_driver->StartTaskPath( pathUpl );
	}

	set_navseq_curtask_phase( AgvTaskPhase_Fin );

	return 0;
}

void agv_base::callback_navseq_status( status_describe_t status, int err ) {
	std::shared_ptr<agv_atom_taskdata_header> task = nullptr;
	{
		std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
		if ( !__nav_sequence.empty() ) {
			task = std::static_pointer_cast< agv_atom_taskdata_header, agv_atom_taskdata_base >( __nav_sequence[0] );
		}

		//sequence final
        if (status > kStatusDescribe_FinalFunction &&task) {
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " gotodock task_id=" << task->get_task_id() << " final, status=" << status << " err=" << err << " !";
			__cur_nav_sequence_idx = -1;
		}
	}

	if ( task
		&& task->get_atomtaskdata_type() == AgvAtomTaskData_Header ) {
		task->callback_task_status( status, err );
	}

}

void agv_base::post_navigation_task_ack( uint32_t id, const void *data ) {

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " recv post_navigation_task_ack, id=" << id;

	std::lock_guard<decltype( __mtx_nav_sequence_newnav )> lock( __mtx_nav_sequence_newnav );

	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_task_ack failed, err = !data, id=" << id;
			break;
		}

		mn::task_status_t *asio_data_ = ( mn::task_status_t* )( data );
		if ( asio_data_->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_task_ack failed, asio_data_.get_err() = " << asio_data_->err_ << ", id=" << id;
			break;
		}

		ret = asio_data_->err_;
        loinfo("agvbase") << "AgvBase:" << __agv_id << " post_navigation_task_ack ok, id=" << __nav_task_id;
        set_navseq_curtask_phase(AgvTaskPhase_Fin);

	} while ( 0 );

	if ( ret < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_task_ack failed, AgvTaskPhase_CheckSend navid=" << __nav_task_id;

		set_navseq_curtask_phase( AgvTaskPhase_CheckSend );
	}
    --__cb_counter;
}

int agv_base::newnav( std::shared_ptr<agv_atom_taskdata_newnav> task ) {

	if ( task->get_task_phase() != AgvTaskPhase_Send ) {
		return 0;
	}

	std::lock_guard<decltype( __mtx_nav_sequence_newnav )> lock( __mtx_nav_sequence_newnav );

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_task" << task->task_id;
	//std::shared_ptr<motion::asio_partnet> asio = std::shared_ptr<motion::asio_partnet>(new motion::asio_partnet(std::bind(&agv_base::post_navigation_task_ack, this, std::placeholders::_1, std::placeholders::_2)));
	int r = post_navigation_task( __net_id, task->task_id, task->destUpl, task->destPos, task->path_is_whole, task->pathUpl,
		std::bind( &agv_base::post_navigation_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );

	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_task failed, r < 0,task_id=" << task->task_id;

		if ( __net_status < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " __net_status < 0,newnav paused";
			return -1;
		}
		if ( __exit ) {
			return -1;
		}
		switch ( get_nav_sequence_prc() ) {
			case kStatusDescribe_Cancel:
			case kStatusDescribe_Pause:
				break;
			case kStatusDescribe_Resume:
				{
					post_task( task );
				}
				break;
			default:
				break;
		}

		return -1;
	}

	set_navseq_curtask_phase( AgvTaskPhase_WaitAck );
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_task ok, AgvTaskPhase_WaitAck navid=" << task->task_id;
    ++__cb_counter;
	return 0;
}

void agv_base::pause_navigation_task_ack( uint32_t id, const void *data ) {
	std::lock_guard<decltype( __mtx_nav_sequence_pausenav )> lock( __mtx_nav_sequence_pausenav );

	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " pause_navigation_task_ack failed, err = !data, id=" << id;
			break;
		}

		task_status_t* status = ( task_status_t* ) ( data );
		if ( status->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " pause_navigation_task_ack failed, asio_data_.get_err() = " << status->err_ << ", id=" << id;
			break;
		}

		ret = status->err_;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " pause_navigation_task_ack ok, id=" << id;
		callback_navseq_status( kStatusDescribe_Hang, AgvInterfaceError_OK );

	} while ( 0 );

	if ( ret < 0 ) {
        do 
        {
            if (__exit) {
                break;
            }
            //if (__net_status < 0) {
            //    loerror("agvbase") << "AgvBase:" << __agv_id << "in pause_navigation_task_ack, __net_status < 0,pasuenav paused";
            //    return;
            //}
            std::shared_ptr<agv_atom_taskdata_pausenav> pn = std::shared_ptr<agv_atom_taskdata_pausenav>(new agv_atom_taskdata_pausenav);
            post_task(pn);
        } while (0);
         
	}
    --__cb_counter;
}

int agv_base::pausenav() {
    var__navigation_t nav;
    int e = get_nav_info(nav);
    if (e<0)
    {
        return 0;
    }
    if (nav.track_status_.response_ == kStatusDescribe_Hang
        || (
        nav.track_status_.response_ < kStatusDescribe_PendingFunction
                || nav.track_status_.response_ > kStatusDescribe_FinalFunction)
        )
    {
        return 0;
    }


	std::lock_guard<decltype( __mtx_nav_sequence_pausenav )> lock( __mtx_nav_sequence_pausenav );

	uint64_t nav_id = -1;
	if ( get_cur_nav_task_id( nav_id ) < 0 ) //no task
	{
		return 0;
	}

	int r = post_navigation_pause_task( __net_id, nav_id, std::bind( &agv_base::pause_navigation_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_navigation_pause_task( __net_id, nav_id, asio );

	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_pause_task failed, r < 0, taskid=" << nav_id;

		//if ( __net_status < 0 ) {
		//          loerror("agvbase") << "AgvBase:" << __agv_id << " __net_status < 0,pasuenav paused";
		//	return -1;
		//}
		if ( __exit ) {
			return 0;
		}
		std::shared_ptr<agv_atom_taskdata_pausenav> pn = std::shared_ptr<agv_atom_taskdata_pausenav>( new agv_atom_taskdata_pausenav );
		post_task( pn );

		return 0;
	}
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " pause_navigation_task ok";
    ++__cb_counter;
	return 0;
}

int agv_base::pause_goto() {
    if (__net_status < 0)
    {
        return -1;
    }
	set_nav_sequnece_prc( kStatusDescribe_Pause );

	std::shared_ptr<agv_atom_taskdata_pausenav> pn = std::shared_ptr<agv_atom_taskdata_pausenav>( new agv_atom_taskdata_pausenav );
	post_task( pn );

	return 0;
}

void agv_base::resume_navigation_task_ack( uint32_t id, const void *data ) {
	std::lock_guard<decltype( __mtx_nav_sequence_resumenav )> lock( __mtx_nav_sequence_resumenav );

	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " resume_navigation_task_ack failed, err = !data, id=" << id;
			break;
		}

		task_status_t* status = ( task_status_t* ) ( data );
		if ( status->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " resume_navigation_task_ack failed, asio_data_.get_err() = " << status->err_ << ", id=" << id;
			break;
		}

		ret = status->err_;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " resume_navigation_task_ack ok";
		callback_navseq_status( kStatusDescribe_Running, AgvInterfaceError_OK );

		std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
		get_task_from_navseq( task );
		if ( task
			&& task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
			&& task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus ) {
			post_task( task );
		}


	} while ( 0 );

	if ( ret < 0 ) {
        do 
        {
            if (__exit) {
                break;
            }
            //if (__net_status < 0) {
            //    loerror("agvbase") << "AgvBase:" << __agv_id << "in resume_navigation_task_ack, __net_status < 0,resumenav paused";
            //    return;
            //}
            std::shared_ptr<agv_atom_taskdata_resumenav> pn = std::shared_ptr<agv_atom_taskdata_resumenav>(new agv_atom_taskdata_resumenav);
            post_task(pn);


        } while (0);

	}
    --__cb_counter;
}

int agv_base::resumenav() {
    var__navigation_t nav;
    int e = get_nav_info(nav);
    if (e < 0)
    {
        return 0;
    }
    if (nav.track_status_.response_ == kStatusDescribe_Running
        || (
        nav.track_status_.response_ < kStatusDescribe_PendingFunction
        || nav.track_status_.response_ > kStatusDescribe_FinalFunction)
        )
    {
        return 0;
    }

	std::lock_guard<decltype( __mtx_nav_sequence_resumenav )> lock( __mtx_nav_sequence_resumenav );

	uint64_t nav_id = -1;
	if ( get_cur_nav_task_id( nav_id ) < 0 ) //no task
	{
		return 0;
	}

	int r = post_navigation_resume_task( __net_id, nav_id, std::bind( &agv_base::resume_navigation_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_navigation_resume_task( __net_id, nav_id, asio );
	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_resume_task failed, r < 0  taskid=" << nav_id;

		//if ( __net_status < 0 ) {
		//          loerror("agvbase") << "AgvBase:" << __agv_id << " __net_status < 0,resumenav paused";
		//	return -1;
		//}
		if ( __exit ) {
			return 0;
		}
		std::shared_ptr<agv_atom_taskdata_resumenav> pn = std::shared_ptr<agv_atom_taskdata_resumenav>( new agv_atom_taskdata_resumenav );
		post_task( pn );

		return 0;
	}

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_resume_task ok";
    ++__cb_counter;
	return 0;
}

int agv_base::resume_goto() {
    if (__net_status < 0)
    {
        return -1;
    }
	set_nav_sequnece_prc( kStatusDescribe_Resume );

	std::shared_ptr<agv_atom_taskdata_resumenav> pn = std::shared_ptr<agv_atom_taskdata_resumenav>( new agv_atom_taskdata_resumenav );
	post_task( pn );
	return 0;
}

void agv_base::cancel_navigation_task_ack( uint32_t id, const void *data ) {
	std::lock_guard<decltype( __mtx_nav_sequence_cancelnav )> lock( __mtx_nav_sequence_cancelnav );

	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " cancel_navigation_task_ack failed, err = !data, id=" << id;
			break;
		}

		task_status_t* status = ( task_status_t* ) ( data );
		if ( status->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " cancel_navigation_task_ack failed, asio_data_.get_err() = " << status->err_ << ", id=" << id;
			break;
		}
		ret = status->err_;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " cancel_navigation_task_ack ok";
		callback_navseq_status( kStatusDescribe_Terminated, AgvInterfaceError_OK );
	} while ( 0 );

	if ( ret < 0 ) {
        do 
        {

            if (__exit) {
                callback_navseq_status(kStatusDescribe_Terminated, AgvInterfaceError_OK);
                break;
            }
            std::shared_ptr<agv_atom_taskdata_cancelnav> pn = std::shared_ptr<agv_atom_taskdata_cancelnav>(new agv_atom_taskdata_cancelnav);
            post_task(pn);

        } while (0);


	}
    --__cb_counter;

}

int agv_base::cancelnav() {

    var__navigation_t nav;
    int e = get_nav_info(nav);
    if (e < 0)
    {
        return 0;
    }


    uint64_t nav_id = -1;
    if (get_cur_nav_task_id(nav_id) < 0) //no task
    {
        return 0;
    }

    if ((
        nav.track_status_.response_ < kStatusDescribe_PendingFunction
        || nav.track_status_.response_ > kStatusDescribe_FinalFunction)
        && nav_id == nav.i.current_task_id_)
    {
        callback_navseq_status(kStatusDescribe_Terminated, AgvInterfaceError_OK);
        return 0;
    }

    //if (nav_id != nav.i.current_task_id_)
    //{
    //    //callback_navseq_status(kStatusDescribe_Terminated, AgvInterfaceError_OK);
    //    //return 0;
    //    nav_id = nav.i.current_task_id_;
    //}

	std::lock_guard<decltype( __mtx_nav_sequence_cancelnav )> lock( __mtx_nav_sequence_cancelnav );



	int r = post_navigation_cancel_task( __net_id, nav_id, std::bind( &agv_base::cancel_navigation_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_navigation_cancel_task( __net_id, nav_id, asio );
	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_cancel_task failed, r < 0 id=" << nav_id;

		//if ( __net_status < 0 ) {
		//          loerror("agvbase") << "AgvBase:" << __agv_id << " __net_status < 0,cancelnav paused";
		//	return -1;
		//}
		if ( __exit ) {
			return 0;
		}
		std::shared_ptr<agv_atom_taskdata_cancelnav> pn = std::shared_ptr<agv_atom_taskdata_cancelnav>( new agv_atom_taskdata_cancelnav );
		post_task( pn );

		return 0;
	}
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_navigation_cancel_task ok";
    ++__cb_counter;
	return 0;
}

int agv_base::cancel_goto() {
    if (__net_status < 0)
    {
        return -1;
    }
    {
        std::lock_guard<decltype(__mtx_nav_sequence)> lock(__mtx_nav_sequence);
        if (__cur_nav_sequence_idx <= 3) {
            return -1;
        }
    }

    //if (get_nav_sequence_prc() != kStatusDescribe_Cancel)
    {
        if (__agv_driver) {
            __agv_driver->CancelTaskPath();
        }

	set_nav_sequnece_prc( kStatusDescribe_Cancel );

        std::shared_ptr<agv_atom_taskdata_cancelnav> pn = std::shared_ptr<agv_atom_taskdata_cancelnav>(new agv_atom_taskdata_cancelnav);
        post_task(pn);
    }

	return 0;
}

void agv_base::add_navigation_task_ack( uint32_t id, const void *data ) {

	std::lock_guard<decltype( __mtx_nav_sequence_addnav )> lock( __mtx_nav_sequence_addnav );

	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " add_navigation_task_ack failed, err = !data,  id=" << id;
			break;
		}

		asio_t* status = ( asio_t* ) ( data );
		if ( status->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " add_navigation_task_ack failed, asio_data_.get_err() = " << status->err_ << ",  id=" << id;
			break;
		}

		ret = status->err_;

		std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
		get_task_from_navseq( task );
		if ( task
			&& task->get_atomtaskdata_type() == AgvAtomTaskData_AddNav ) {
			std::shared_ptr<agv_atom_taskdata_addnav> nav = std::static_pointer_cast< agv_atom_taskdata_addnav, agv_atom_taskdata_base >( task );
			if ( nav->path_is_whole == 1 ) {
				loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " add_navigation_task_ack ok, AgvTaskPhase_Fin";

				set_navseq_curtask_phase( AgvTaskPhase_Fin );
			} else {
				loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " add_navigation_task_ack ok, AgvTaskPhase_WaitTraffic";
				set_navseq_curtask_phase( AgvTaskPhase_WaitTraffic );
			}
		}

	} while ( 0 );

	if ( ret < 0 ) {
        do 
        {
            if (__exit) {
                break;
            }
            set_navseq_curtask_phase(AgvTaskPhase_Send);
            if (__net_status < 0) {
                loerror("agvbase") << "AgvBase:" << __agv_id << " in add_navigation_task_ack, __net_status < 0,addnav paused";
                break;
            }
            switch (get_nav_sequence_prc()) {
            case kStatusDescribe_Cancel:
            case kStatusDescribe_Pause:
                break;
            case kStatusDescribe_Resume:
            {
                                           std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
                                           get_task_from_navseq(task);
                                           if (task
                                               && task->get_atomtaskdata_type() == AgvAtomTaskData_AddNav) {

                                               post_task(task);
                                           }
            }
                break;
            default:
                break;
            }

        } while (0);
	}

    --__cb_counter;
}

int agv_base::addnav( std::shared_ptr<agv_atom_taskdata_addnav> task ) {
	if ( task->get_task_phase() != AgvTaskPhase_Send ) {
		if ( task->get_task_phase() == AgvTaskPhase_WaitTraffic ) {
			//need get traffic first
			if ( __nav_path_iswhole ) {
				loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " path is whole need not addnav anymore";
				set_navseq_curtask_phase( AgvTaskPhase_Fin );
			}
			return 0;
		}
		return 0;
	}

	if ( task->path_is_whole == 1 && __nav.track_status_.response_ > kStatusDescribe_FinalFunction ) {

		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " path is whole and nav is finished,not addnav anymore";
		set_navseq_curtask_phase( AgvTaskPhase_Fin );
		return 0;
	}

	std::lock_guard<decltype( __mtx_nav_sequence_addnav )> lock( __mtx_nav_sequence_addnav );

	uint64_t nav_id = -1;
	if ( get_cur_nav_task_id( nav_id ) < 0 ) //no task
	{
		return 0;
	}
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_add_navigation_task_traj: taskid=" << nav_id << " index=" << task->index << " path_is_whole=" << task->path_is_whole << " path.size()=" << task->pathUpl.size();

	int r = post_add_navigation_task_traj( __net_id, nav_id, task->index, task->pathUpl, task->path_is_whole, std::bind( &agv_base::add_navigation_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_add_navigation_task_traj( __net_id, nav_id, task->index, task->pathUpl, task->path_is_whole, asio );
	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_add_navigation_task_traj failed, r < 0 taskid=" << nav_id;

		if ( __net_status < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " __net_status < 0,addnav paused";
			return -1;
		}
		if ( __exit ) {
			return -1;
		}
		switch ( get_nav_sequence_prc() ) {
			case kStatusDescribe_Cancel:
			case kStatusDescribe_Pause:
				break;
			case kStatusDescribe_Resume:
				{
					post_task( task );
				}
				break;
			default:
				break;
		}

		return -1;
	}
	set_navseq_curtask_phase( AgvTaskPhase_WaitAck );
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_add_navigation_task_traj ok";
    ++__cb_counter;
	return 0;
}

void agv_base::check_report_nav_status_from_navseq( status_describe_t s, uint64_t task_id ) {
	//std::lock_guard<decltype( __mtx_nav_sequence_phase )> lock( __mtx_nav_sequence_phase );

	//check task
	uint64_t nav_id = -1;
	if ( get_cur_nav_task_id( nav_id ) < 0 ) {
		return;
	}
	std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
	get_task_from_navseq( task );
	if ( task ) {
		if ( task->get_atomtaskdata_type() == AgvAtomTaskData_CheckStatus ) {
			std::shared_ptr<agv_atom_taskdata_check_base> ck = std::static_pointer_cast< agv_atom_taskdata_check_base, agv_atom_taskdata_base >( task );
            bool need_check_idex = true;
            if (task->get_atomtaskseq_idex() >= 0)
            {
                if (get_nav_sequence_cur_idex() == task->get_atomtaskseq_idex())
                {
                    need_check_idex = true;
                }
                else
                {
                    loinfo("agvbase") << "AgvBase:" << __agv_id << " check_report_nav_status_from_navseq,need_check_idex,cur_idex= " << get_nav_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                    need_check_idex = false;
                }
            }
            if ( ck->check_status( s, nav_id == task_id && need_check_idex, AgvAtomTaskType_Nav ) == 0 ) {
                loinfo("agvbase") << "AgvBase:" << __agv_id << " check_report_nav_status_from_navseq,check_status = true ,check_type = " << ck->get_check_types() << " task_id=" << task_id;

				set_navseq_curtask_phase( AgvTaskPhase_Fin, s );
			}
		}
	}
}

void agv_base::check_report_nav_status_from_optseq( status_describe_t s, uint64_t task_id ) {
	//std::lock_guard<decltype( __mtx_opt_sequence_phase )> lock( __mtx_opt_sequence_phase );
	//check task
	uint64_t nav_id = -1;
	if ( get_cur_nav_task_id( nav_id ) < 0 ) {
		return;
	}
	uint64_t opt_id = -1;
	if ( get_cur_opt_task_id( opt_id ) < 0 ) {
		return;
	}
	std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
	get_task_from_optseq( task );
	if ( task ) {
		if ( task->get_atomtaskdata_type() == AgvAtomTaskData_CheckStatus ) {
			std::shared_ptr<agv_atom_taskdata_check_base> ck = std::static_pointer_cast< agv_atom_taskdata_check_base, agv_atom_taskdata_base >( task );
            bool need_check_idex = true;
            if (task->get_atomtaskseq_idex() >= 0)
            {
                if (get_opt_sequence_cur_idex() == task->get_atomtaskseq_idex())
                {
                    need_check_idex = true;
                }
                else
                {
                    loinfo("agvbase") << "AgvBase:" << __agv_id << " check_report_nav_status_from_optseq,need_check_idex,cur_idex= " << get_opt_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                    need_check_idex = false;
                }
            }
			if ( ck->check_status( s, nav_id == task_id && need_check_idex, AgvAtomTaskType_Nav ) == 0 ) {
                loinfo("agvbase") << "AgvBase:" << __agv_id << " check_report_nav_status_from_optseq,check_status = true ,check_type = " << ck->get_check_types() << " task_id=" << task_id;

				set_optseq_curtask_phase( AgvTaskPhase_Fin, s ,task->get_atomtaskseq_idex());
			}
		}
	}
}

void agv_base::check_report_opt_status( status_describe_t s, uint64_t task_id ) {
	//std::lock_guard<decltype( __mtx_opt_sequence_phase )> lock( __mtx_opt_sequence_phase );
	uint64_t opt_id = -1;
	if ( get_cur_opt_task_id( opt_id ) < 0 ) {
		return;
	}
	std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
	get_task_from_optseq( task );
	if ( task ) {
		if ( task->get_atomtaskdata_type() == AgvAtomTaskData_CheckStatus
			) {
			std::shared_ptr<agv_atom_taskdata_check_base> ck = std::static_pointer_cast< agv_atom_taskdata_check_base, agv_atom_taskdata_base >( task );
            bool need_check_idex = true;
            if (task->get_atomtaskseq_idex() >= 0)
            {
                if (get_opt_sequence_cur_idex() == task->get_atomtaskseq_idex())
                {
                    need_check_idex = true;
                }
                else
                {
                    loinfo("agvbase") << "AgvBase:" << __agv_id << " check_report_opt_status,need_check_idex,cur_idex= " << get_opt_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                    need_check_idex = false;
                }
            }
			if ( ck->check_status( s, opt_id == task_id && need_check_idex, AgvAtomTaskType_Operation ) == 0 ) {
                loinfo("agvbase") << "AgvBase:" << __agv_id << " check_report_opt_status,check_status = true ,check_type = " << ck->get_check_types() << " task_id=" << task_id;

                set_optseq_curtask_phase(AgvTaskPhase_Fin, s, task->get_atomtaskseq_idex());
			}
		}
	}
}

void agv_base::check_nav_status_from_navseq( status_describe_t s ) {
	//std::lock_guard<decltype( __mtx_nav_sequence_phase )> lock( __mtx_nav_sequence_phase );

	//check task
	uint64_t nav_id = -1;
	if ( get_cur_nav_task_id( nav_id ) < 0 ) {
		return;
	}
	std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
	get_task_from_navseq( task );
	if ( task ) {
		if ( task->get_atomtaskdata_type() == AgvAtomTaskData_CheckStatus ) {
			std::shared_ptr<agv_atom_taskdata_check_base> ck = std::static_pointer_cast< agv_atom_taskdata_check_base, agv_atom_taskdata_base >( task );

			if ( __nav_report_credible ) {
				if ( !( ck->get_check_types() == AgvTaskStatus_CheckStatus_EqualStatus
					|| ck->get_check_types() == AgvTaskStatus_CheckStatus_Final )
					) {
                    bool need_check_idex = true;
                        if (task->get_atomtaskseq_idex() >= 0)
                        {
                            if (get_nav_sequence_cur_idex() == task->get_atomtaskseq_idex())
                            {
                                need_check_idex = true;
                            }
                            else
                            {
                                loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_navseq,need_check_idex,cur_idex= " << get_nav_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                                need_check_idex = false;
                            }
                        }
					if ( ck->check_status( s, nav_id == __nav.i.current_task_id_ &&need_check_idex, AgvAtomTaskType_Nav ) == 0 ) {
                        loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_navseq, __nav_report_credible = 1,check_status = true ,check_type = " << ck->get_check_types();

						set_navseq_curtask_phase( AgvTaskPhase_Fin, s );
					}
				} else {
					if ( ck->check_timeout() ) {
                        bool need_check_idex = true;
                        if (task->get_atomtaskseq_idex() >= 0)
                        {
                            if (get_nav_sequence_cur_idex() == task->get_atomtaskseq_idex())
                            {
                                need_check_idex = true;
                            }
                            else
                            {
                                loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_navseq,need_check_idex,cur_idex= " << get_nav_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                                need_check_idex = false;
                            }
                        }
                        if (ck->check_status(s, nav_id == __nav.i.current_task_id_&&need_check_idex, AgvAtomTaskType_Nav) == 0) {
							loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " check_nav_status_from_navseq check_timeout trriggred, check_status = true ,check_type = "<<ck->get_check_types();
							set_navseq_curtask_phase( AgvTaskPhase_Fin, s );
						}
					}
				}
			} else {
                bool need_check_idex = true;
                if (task->get_atomtaskseq_idex() >= 0)
                {
                    if (get_nav_sequence_cur_idex() == task->get_atomtaskseq_idex())
                    {
                        need_check_idex = true;
                    }
                    else
                    {
                        loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_navseq,need_check_idex,cur_idex= " << get_nav_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                        need_check_idex = false;
                    }
                }
                if (ck->check_status(s, nav_id == __nav.i.current_task_id_&&need_check_idex, AgvAtomTaskType_Nav) == 0) {
                    loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_navseq, __nav_report_credible = 0,check_status = true ,check_type = " << ck->get_check_types();

					set_navseq_curtask_phase( AgvTaskPhase_Fin, s );
				}
			}
		} else if ( task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus
			&& task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send ) {
			if ( task->get_task_phase() == AgvTaskPhase_CheckSend ) {
				if ( nav_id == __nav.user_task_id_ )//send ok
				{
                    loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_navseq, check send ,check_status = true ,task_id = " << nav_id;

					set_navseq_curtask_phase( AgvTaskPhase_Fin, s );
				} else  //retry send
				{
					set_navseq_curtask_phase( AgvTaskPhase_Send );
					task->set_task_phase( AgvTaskPhase_Send );
					post_task( task );
				}
			}
		}
	}
}

void agv_base::check_nav_status_from_optseq( status_describe_t s ) {
	//std::lock_guard<decltype( __mtx_nav_sequence_phase )> lock( __mtx_nav_sequence_phase );
	//check task
	uint64_t nav_id = -1;
	if ( get_cur_nav_task_id( nav_id ) < 0 ) {
		return;
	}
	uint64_t opt_id = -1;
	if ( get_cur_opt_task_id( opt_id ) < 0 ) {
		return;
	}
	std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
	get_task_from_optseq( task );
	if ( task ) {
		if ( task->get_atomtaskdata_type() == AgvAtomTaskData_CheckStatus ) {
			std::shared_ptr<agv_atom_taskdata_check_base> ck = std::static_pointer_cast< agv_atom_taskdata_check_base, agv_atom_taskdata_base >( task );

			if ( __nav_report_credible ) {
				if ( !( ck->get_check_types() == AgvTaskStatus_CheckStatus_EqualStatus
					|| ck->get_check_types() == AgvTaskStatus_CheckStatus_Final )
					) {
                    bool need_check_idex = true;
                    if (task->get_atomtaskseq_idex() >= 0)
                    {
                        if (get_opt_sequence_cur_idex() == task->get_atomtaskseq_idex())
                        {
                            need_check_idex = true;
                        }
                        else
                        {
                            loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_optseq,need_check_idex,cur_idex= " << get_opt_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                            need_check_idex = false;
                        }
                    }
                    if (ck->check_status(s, nav_id == __nav.i.current_task_id_&&need_check_idex, AgvAtomTaskType_Nav) == 0) {
                        loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_optseq, __nav_report_credible = 1,check_status = true ,check_type = " << ck->get_check_types();

                        set_optseq_curtask_phase(AgvTaskPhase_Fin, s, task->get_atomtaskseq_idex());
					}
				} else {
					if ( ck->check_timeout() ) {
                        bool need_check_idex = true;
                        if (task->get_atomtaskseq_idex() >= 0)
                        {
                            if (get_opt_sequence_cur_idex() == task->get_atomtaskseq_idex())
                            {
                                need_check_idex = true;
                            }
                            else
                            {
                                loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_optseq,need_check_idex,cur_idex= " << get_opt_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                                need_check_idex = false;
                            }
                        }
                        if (ck->check_status(s, nav_id == __nav.i.current_task_id_&&need_check_idex, AgvAtomTaskType_Nav) == 0) {
                            loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_optseq check_timeout trriggred, check_status = true ,check_type = " << ck->get_check_types();

                            set_optseq_curtask_phase(AgvTaskPhase_Fin, s, task->get_atomtaskseq_idex());
						}
					}
				}
			} else {
                bool need_check_idex = true;
                if (task->get_atomtaskseq_idex() >= 0)
                {
                    if (get_opt_sequence_cur_idex() == task->get_atomtaskseq_idex())
                    {
                        need_check_idex = true;
                    }
                    else
                    {
                        loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_optseq,need_check_idex,cur_idex= " << get_opt_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                        need_check_idex = false;
                    }
                }
                if (ck->check_status(s, nav_id == __nav.i.current_task_id_&&need_check_idex, AgvAtomTaskType_Nav) == 0) {
                    loinfo("agvbase") << "AgvBase:" << __agv_id << " check_nav_status_from_optseq, __nav_report_credible = 0,check_status = true ,check_type = " << ck->get_check_types();

                    set_optseq_curtask_phase(AgvTaskPhase_Fin, s, task->get_atomtaskseq_idex());
				}
			}
		}

	}


}

void agv_base::check_opt_status( const var__operation_t& opt ) {
	//std::lock_guard<decltype( __mtx_opt_sequence_phase )> lock( __mtx_opt_sequence_phase );
	uint64_t opt_id = -1;
	if ( get_cur_opt_task_id( opt_id ) < 0 ) {
		return;
	}
	std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
	get_task_from_optseq( task );
	if ( task ) {
		if ( task->get_atomtaskdata_type() == AgvAtomTaskData_CheckStatus
			) {
			std::shared_ptr<agv_atom_taskdata_check_base> ck = std::static_pointer_cast< agv_atom_taskdata_check_base, agv_atom_taskdata_base >( task );

			if ( __opt_report_credible ) {
				if ( !( ck->get_check_types() == AgvTaskStatus_CheckStatus_EqualStatus
					|| ck->get_check_types() == AgvTaskStatus_CheckStatus_Final )
					) {
                    bool need_check_idex = true;
                    if (task->get_atomtaskseq_idex() >= 0)
                    {
                        if (get_opt_sequence_cur_idex() == task->get_atomtaskseq_idex())
                        {
                            need_check_idex = true;
                        }
                        else
                        {
                            loinfo("agvbase") << "AgvBase:" << __agv_id << " check_opt_status,need_check_idex,cur_idex= " << get_opt_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                            need_check_idex = false;
                        }
                    }
					if ( ck->check_status( opt.status_.response_, opt_id == __opr.i.current_task_id_&&need_check_idex, AgvAtomTaskType_Operation ) == 0 ) {

                        loinfo("agvbase") << "AgvBase:" << __agv_id << " check_opt_status ,__opt_report_credible =1,check_status = true ,check_type=" << ck->get_check_types();
                        set_optseq_curtask_phase(AgvTaskPhase_Fin, opt.status_.response_, task->get_atomtaskseq_idex());
					}
				} else {
					if ( ck->check_timeout() ) {
                        bool need_check_idex = true;
                        if (task->get_atomtaskseq_idex() >= 0)
                        {
                            if (get_opt_sequence_cur_idex() == task->get_atomtaskseq_idex())
                            {
                                need_check_idex = true;
                            }
                            else
                            {
                                loinfo("agvbase") << "AgvBase:" << __agv_id << " check_opt_status,need_check_idex,cur_idex= " << get_opt_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                                need_check_idex = false;
                            }
                        }
                        if (ck->check_status(opt.status_.response_, opt_id == __opr.i.current_task_id_&&need_check_idex, AgvAtomTaskType_Operation) == 0) {
                            loinfo("agvbase") << "AgvBase:" << __agv_id << " check_opt_status check_timeout trriggred, check_status = true ,check_type=" << ck->get_check_types();

                            set_optseq_curtask_phase(AgvTaskPhase_Fin, opt.status_.response_, task->get_atomtaskseq_idex());
						}
					}
				}
			} else {
                bool need_check_idex = true;
                if (task->get_atomtaskseq_idex() >= 0)
                {
                    if (get_opt_sequence_cur_idex() == task->get_atomtaskseq_idex())
                    {                           
                        need_check_idex = true;
                    }
                    else
                    {
                        loinfo("agvbase") << "AgvBase:" << __agv_id << " check_opt_status,need_check_idex,cur_idex= " << get_opt_sequence_cur_idex() << " task_idx=" << task->get_atomtaskseq_idex();
                        need_check_idex = false;
                    }
                }
                if (ck->check_status(opt.status_.response_, opt_id == __opr.i.current_task_id_&&need_check_idex, AgvAtomTaskType_Operation) == 0) {
                    loinfo("agvbase") << "AgvBase:" << __agv_id << " check_opt_status ,__opt_report_credible =0,check_status = true ,check_type=" << ck->get_check_types();

                    set_optseq_curtask_phase(AgvTaskPhase_Fin, opt.status_.response_, task->get_atomtaskseq_idex());
				}
			}
		} else if ( task->get_atomtaskdata_type() == AgvAtomTaskData_CheckOpt ) {
			std::shared_ptr<agv_atom_taskdata_checkopt> ck = std::static_pointer_cast< agv_atom_taskdata_checkopt, agv_atom_taskdata_base >( task );
			if ( ck->check( opt ) == 0 ) {
                loinfo("agvbase") << "AgvBase:" << __agv_id << " check_opt_status ,check_opt = true,task_id="<<opt.i.current_task_id_;

                set_optseq_curtask_phase(AgvTaskPhase_Fin, opt.status_.response_, task->get_atomtaskseq_idex());
			}
		} else if ( task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus
			&& task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send ) {
			if ( task->get_task_phase() == AgvTaskPhase_CheckSend ) {
				if ( opt_id == __opr.user_task_id_ )//send ok
				{
                    loinfo("agvbase") << "AgvBase:" << __agv_id << " check_opt_status ,check send = true,task_id="<<opt_id;

                    set_optseq_curtask_phase(AgvTaskPhase_Fin, opt.status_.response_, task->get_atomtaskseq_idex());
				} else  //retry send
				{
                    set_optseq_curtask_phase(AgvTaskPhase_Send, opt.status_.response_, task->get_atomtaskseq_idex());
					task->set_task_phase( AgvTaskPhase_Send );
					post_task( task );
				}
			}
		}

	}
}

void agv_base::common_read_ack( uint32_t id, const void *data ) {
	do {

		mn::common_data *asio_data_ = ( mn::common_data* )( data );
		if ( asio_data_->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " common_read_ack failed, asio_data_.get_err() = " << asio_data_->err_ << ", id=" << id;
			break;
		}
		if (asio_data_->items.size() == 0){
			break;
		}
		common_data_item ca = asio_data_->items[0];

		switch ( ca.vartype ) {
			case kVarType_Vehicle:
				{
					std::lock_guard<decltype( __mtx_veh )> lock( __mtx_veh );
					var__vehicle_t* vh = ( var__vehicle_t* ) ( ca.data.c_str() );
					memcpy( &__veh, vh, sizeof( __veh ) );
				}
				break;
			case kVarType_Navigation:
				{
					{
						std::lock_guard<decltype( __mtx_nav )> lock( __mtx_nav );
						var__navigation_t* nv = ( var__navigation_t* ) ( ca.data.c_str() );
						memcpy( &__nav, nv, sizeof( __nav ) );
					}

					//::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d readnav and do traffic begin", __agv_id);
					upl_t cur_upl;
					int idex = 0;
					if ( get_current_upl( cur_upl ) >= 0 ) {
						if ( get_current_traj_idex( idex ) >= 0 ) {
							if ( __agv_driver ) {
								__agv_driver->UpdateUPL( cur_upl, idex );
								__agv_driver->DoAllocatedAnddRelease();
							}
						}
					}

					check_nav_status_from_navseq( __nav.track_status_.response_ );
                    check_nav_status_from_optseq(__nav.track_status_.response_);
                    //::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d readnav and do traffic end", __agv_id);

				}
				break;
			case kVarType_Operation:
				{
					{
						std::lock_guard<decltype( __mtx_opt )> lock( __mtx_opt );
						var__operation_t* op = ( var__operation_t* ) ( ca.data.c_str() );
						memcpy( &__opr, op, sizeof( var__operation_t ) );
					}
					//::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d check_opt_status begin", __agv_id);
					check_opt_status( __opr );
					//::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d check_opt_status end", __agv_id);

				}
				break;
			case kVarType_ErrorHandler:
				{
					std::lock_guard<decltype( __mtx_fault )> lock( __mtx_fault );
					var__error_handler_t* op = ( var__error_handler_t* ) ( ca.data.c_str() );
					memcpy( &__fault, op, sizeof( var__error_handler_t ) );
				}
				break;
			case kVarType_SafetyProtec:
				{
					std::lock_guard<decltype( __mtx_safety )> lock( __mtx_safety );
					var__safety_t* op = ( var__safety_t* ) ( ca.data.c_str() );
					memcpy( &__safety, op, sizeof( var__safety_t ) );
				}
				break;
            case kVarType_DIO:
            {
                                 if (ca.varid == kVarFixedObject_InternalDIO)
                                 {
                                     std::lock_guard<decltype(__mtx_dio_main)> lock(__mtx_dio_main);
                                     var__dio_t* vh = (var__dio_t*)(ca.data.c_str());
                                     memcpy(&__dio_main, vh, sizeof(var__dio_t));
                                 }
                                 else
                                 {
                                     get_elongate_variable_ack(id, data);
                                 }
            }
                break;
            case kVarType_OfflineTask:
            {
                {
                    std::lock_guard<decltype(__mtx_offtask)> lock(__mtx_offtask);
                    var_offline_task* vh = (var_offline_task*)(ca.data.c_str());
					memcpy(&__var_offtask, vh, offsetof(var_offline_task, tasks_[0]));
                }

                                         if (__var_offtask.track_status_.response_ > kStatusDescribe_FinalFunction)
                                         {
                                             std::lock_guard<decltype(__mtx_offtask_ptr)> lock(__mtx_offtask_ptr);
                                             if (__offline_task && __offline_task->get_task_phase() == AgvTaskPhase_Fin)
                                             {
                                                 __offline_task->callback_status(__var_offtask.track_status_.response_, kAgvInterfaceError_OK);
                                             }
                                         }
            }
                break;
			default:
				get_elongate_variable_ack( id, data );
				break;
		}

		{
			std::lock_guard<decltype( __mtx_comm_read_unique )> guard( __mtx_comm_read_unique );
			__map_comm_read_unique[ca.varid]--;
		}

	} while ( 0 );


}

int agv_base::get_agv_obj_list_syn( std::vector<var_item>& obj ) {
	obj.clear();
	int err = -1;
	var_list*asio;
	nsp::os::waitable_handle wait_request(0);
	int r = post_dbg_varls_request( __net_id,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			wait_request.sig();
			err = -1;
			return;
		}
		asio = ( var_list* ) data;
		err = asio->err_;
		if ( asio->err_ < 0 ) {
			wait_request.sig();
			return;
		}
		if ( !asio ) {
			wait_request.sig();
			return;
		}
		obj = asio->items_;
		wait_request.sig();
	} );
	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();
	return err;
}

void agv_base::update_agv_data() {
	if ( __exit ) {
		return;
	}
	if ( __net_status < 0 ) {
		return;
	}
    //loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 1";

	if ( __vct_agv_obj_list.empty() ) {
		std::vector<var_item> v;
		if ( get_agv_obj_list_syn( v ) >= 0 ) {
			__vct_agv_obj_list = v;
		}
	}

    if (__map_subscribe_data.empty())//无订阅，读全量数据
    {
        //loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 2";

        get_var_info_by_id_asyn<var__navigation_t>(kVarFixedObject_Navigation);

        //loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 3";
        get_var_info_by_id_asyn<var__vehicle_t>(kVarFixedObject_Vehide);
        //loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 4";

        get_var_info_by_id_asyn<var__operation_t>(kVarFixedObject_Operation);
        //loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 5";

        get_var_info_by_id_asyn<var__safety_t>(kVarFixedObject_SaftyProtec);
        //loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 6";

        get_var_info_by_id_asyn<var__error_handler_t>(kVarFixedObject_ErrorCollecter);
        //loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 7";
        get_var_info_by_id_asyn<var__dio_t>(kVarFixedObject_InternalDIO);

        get_var_info_by_id_asyn<var_offline_task>(kVarFixedObject_OfflineTask);
        loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 8";

        get_elongate_variable();
        //loinfo("agvbase_debug") << "AgvBase:" << __agv_id << " update_agv_data 9";
    } 
    else
    {
        for (auto& item:__map_subscribe_data)
        {
            int r = mn::post_common_read_request_by_id(__net_id, item.second, std::bind(&agv_base::common_read_ack_subscrbe, this, std::placeholders::_1, std::placeholders::_2));
            if (r < 0){//如果接口调用失败，那么直接返回
                loerror("agvbase") << "AgvBase:" << __agv_id << "subscribe post_common_read_request_by_id failed，ret < 0，id = " << item.second.items[0].varid << " err=" << r;
            }
        }
    }


}

status_describe_t agv_base::get_nav_sequence_prc() {
	std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
	return __nav_sequence_prc;
}


int agv_base::get_nav_sequence_cur_idex()
{
    std::lock_guard<decltype(__mtx_nav_sequence)> lock(__mtx_nav_sequence);
    return __cur_nav_sequence_idx;
}

void agv_base::set_nav_sequnece_prc( status_describe_t s ) {
	std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
	__nav_sequence_prc = s;
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " set_nav_sequnece_prc=" << s;
}

uint64_t agv_base::get_autoinc_nav_id() {

	if ( ++__nav_task_id > NAV_TASK_ID_UPPER ) {
		__nav_task_id = NAV_TASK_ID_LOWER;
	}

	if ( __nav_task_id < NAV_TASK_ID_LOWER ) {
		__nav_task_id = NAV_TASK_ID_LOWER;
	}

	return __nav_task_id;
}

int agv_base::traffic_path_available_cb( int has_path ) {

	//::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d traffic_path_available_cb", __agv_id);

    
    int idex_from_cache = 0;
    std::vector<trail_t> path_from_cache;

    std::shared_ptr<detour_wait_info> dwi;
    if (get_cur_detourwait_dockinf(__cache_idex, __cache_path.size(), dwi) >= 0)
    {
        if (dwi)//停止边在缓存中
        {
            int new_path_size = dwi->wait_idx - __cache_idex;
            if (new_path_size<=0)
            {
                __detour_blocked_dock = dwi->dock_id;
                loinfo("agvbase") << "AgvBase:" << __agv_id << " detour traffic need wait,dwi.wait=" << dwi->wait << " counter=" << dwi->wait_counter<<" dwi.wait_idex="<<dwi->wait_idx<<" dock="<<dwi->dock_id;
                dwi->wait_counter++;
                return 0;
            }
            else    //需要取出部分缓存
            {
                idex_from_cache = __cache_idex;
                for (int i = 0; i < new_path_size; i++)
                {
                    path_from_cache.push_back(__cache_path[i]);
                }
                if (new_path_size == __cache_path.size())
                {
                    __cache_path.clear();
                }
                else
                {
                    std::vector<trail_t> tmp;
                    tmp.assign(__cache_path.begin() + new_path_size, __cache_path.end());
                    __cache_path = tmp;
                }

                __cache_idex += new_path_size;

                loinfo("agvbase") << "AgvBase:" << __agv_id << " detour traffic get part path,new path idx="<<idex_from_cache<<" size="<<path_from_cache.size()<<" cache left idx="<<__cache_idex<<" size="<<__cache_path.size()<<" ,need wait for dwi.wait=" << dwi->wait << " counter=" << dwi->wait_counter << " dwi.wait_idex=" << dwi->wait_idx << " dock=" << dwi->dock_id;

            }

        }
    }

    if (!has_path)
    {
        return 0;
    }
    loinfo("agvbase") << "AgvBase:" << __agv_id << " traffic_path_available_cb start!";

	std::lock_guard<decltype( __mtx_nav_sequence_traffic )> lock( __mtx_nav_sequence_traffic );
	do {

		std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
		get_task_from_navseq( task );
		if ( task
			&& task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
			&& task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus
			&& task->get_task_phase() == AgvTaskPhase_WaitTraffic ) {

            int idex = 0;
            std::vector<trail_t> path;
            bool path_is_whole = false;
            if (path_from_cache.empty())
            {
                if (__nav_path_iswhole) {
                    task->set_task_phase(AgvTaskPhase_Fin);
                    break;
                }                    

                if (__agv_driver) {
                    path = __agv_driver->GetAllocatedTaskPath(path_is_whole, idex);
                }
                if (path.empty()) {
                    break;
                }
                __nav_path_iswhole = path_is_whole ? 1 : 0;
                if (!__nav_path_iswhole)
                {
                    __cache_idex = idex;
                    __cache_path = path;
                }
            }
            else
            {
                idex = idex_from_cache;
                path = path_from_cache;
                path_is_whole = false;
            }


			switch ( task->get_atomtaskdata_type() ) {
				case AgvAtomTaskData_NewNav:
					{
						loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post agv_atom_taskdata_newnav! path_is_whole=" << path_is_whole;
						std::shared_ptr<agv_atom_taskdata_newnav> ck = std::static_pointer_cast< agv_atom_taskdata_newnav, agv_atom_taskdata_base >( task );
						ck->path_is_whole = path_is_whole;
						ck->pathUpl = path;
						ck->set_task_phase( AgvTaskPhase_Send );
						update_navseq_cur_taskdata( ck );
						post_task( ck );
					}
					break;
				case AgvAtomTaskData_AddNav:
					{

						loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post agv_atom_taskdata_addnav! path_is_whole=" << path_is_whole;
						std::shared_ptr<agv_atom_taskdata_addnav> ck = std::static_pointer_cast< agv_atom_taskdata_addnav, agv_atom_taskdata_base >( task );
						ck->path_is_whole = path_is_whole;
						ck->pathUpl = path;
						ck->index = idex;
						ck->set_task_phase( AgvTaskPhase_Send );
						update_navseq_cur_taskdata( ck );
						post_task( ck );
					}
					break;
				default:
					break;
			}

		}

	} while ( 0 );

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " traffic_path_available_cb fin!";
	return 0;
}

int agv_base::update_navseq_cur_taskdata( std::shared_ptr<agv_atom_taskdata_base> task ) {
	std::lock_guard<decltype( __mtx_nav_sequence )> lock( __mtx_nav_sequence );
	if ( __cur_nav_sequence_idx < 0
		|| __cur_nav_sequence_idx >= ( int ) __nav_sequence.size() ) {
		return -1;
	}
	__nav_sequence[__cur_nav_sequence_idx] = task;
	return 0;
}

int agv_base::update_optseq_cur_taskdata( std::shared_ptr<agv_atom_taskdata_base>& task ) {
	std::lock_guard<decltype( __mtx_opt_sequence )> lock( __mtx_opt_sequence );
	if ( __cur_opt_sequence_idx < 0
		|| __cur_opt_sequence_idx >= ( int ) __opt_sequence.size() ) {
		return -1;
	}
	__opt_sequence[__cur_opt_sequence_idx] = task;
	return 0;
}

int agv_base::get_opt_sequence_cur_idex()
{
    std::lock_guard<decltype(__mtx_opt_sequence)> lock(__mtx_opt_sequence);
    if (__cur_opt_sequence_idx < 0
        || __cur_opt_sequence_idx >= (int)__opt_sequence.size()) {
        return -1;
    }
    return __cur_opt_sequence_idx;
}

int agv_base::make_common_writedata( int id, std::vector<mn::common_data_item> &, void* ) {
	return 0;
}

int agv_base::do_opt_senquence( int dock, int opt,
	const std::function<int( int dock, int opt, std::vector<std::shared_ptr<agv_atom_taskdata_base>>& v, void* )> &opt_logic, 
	void* opt_logic_user, uint64_t& optseq_task_id, 
	const std::function<void( uint64_t taskid, status_describe_t status, int err, void* user )> &fn, void* user ) {
	{
		std::lock_guard<decltype( __mtx_opt_sequence )> lock( __mtx_opt_sequence );
        static int printlog = 1;
		if ( __cur_opt_sequence_idx != -1 ) {
            if (printlog)
            {
                loerror("agvbase") << "AgvBase:" << __agv_id << " do_opt_senquence "  " last task not finished!";
                printlog = 0;
            }
			return -1;
		}

        printlog = 1;

		__opt_sequence.clear();
		{
			optseq_task_id = ++__opt_task_id;
			std::shared_ptr<agv_atom_opt_taskdata_header> p_header = std::shared_ptr<agv_atom_opt_taskdata_header>( new agv_atom_opt_taskdata_header );
			p_header->set_task_id( optseq_task_id );
			p_header->set_atomtask_id( optseq_task_id );
			p_header->set_task_status_callback( fn, user );
			__opt_sequence.push_back( p_header );

			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new do_opt_senquence: idx=" << __opt_sequence.size() - 1 << " agv_atom_opt_taskdata_header";
		}

		if ( opt_logic ) {
			opt_logic( dock, opt, __opt_sequence, opt_logic_user );
		} else {
			return 0;
		}

		__cur_opt_sequence_idx = 1;
		__opt_sequence_prc = kStatusDescribe_Resume;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new do_opt_senquence= " << __opt_task_id << " idx=" << __cur_opt_sequence_idx << " optid=" << optseq_task_id;

		if ( __cur_opt_sequence_idx < 0
			|| __cur_opt_sequence_idx >= ( int ) __opt_sequence.size() ) {
			return -1;
		}
		{
			std::shared_ptr<agv_atom_taskdata_base> task = __opt_sequence[__cur_opt_sequence_idx];
			if ( task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
				&& task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus ) {
				post_task( task );
			}
		}

	}

	return 0;
}

int agv_base::get_task_from_optseq( std::shared_ptr<agv_atom_taskdata_base>& task ) {
	task = nullptr;
	{
		if ( __net_status < 0 ) {
			return -1;
		}
		if ( __exit ) {
			return -1;
		}
		std::lock_guard<decltype( __mtx_opt_sequence )> lock( __mtx_opt_sequence );
		if ( __cur_opt_sequence_idx < 0
			|| __cur_opt_sequence_idx >= ( int ) __opt_sequence.size() ) {
			return -1;
		}

		switch ( __opt_sequence_prc ) {
			case kStatusDescribe_Resume:
				task = __opt_sequence[__cur_opt_sequence_idx];
				break;
			case kStatusDescribe_Pause:
				return 1;
			case kStatusDescribe_Cancel:
				return -1;
			default:
				return -1;
		}
	}
	return 0;
}

int agv_base::set_optseq_curtask_phase(agv_task_phase p, status_describe_t s, int idex) {
	bool seq_complete = false;
	{
		std::lock_guard<decltype( __mtx_opt_sequence )> lock( __mtx_opt_sequence );
		if ( __cur_opt_sequence_idx < 0
			|| __cur_opt_sequence_idx >= ( int ) __opt_sequence.size() ) {
			return -1;
		}

        if (__opt_sequence[__cur_opt_sequence_idx]->get_task_phase() == p
            &&  p == AgvTaskPhase_Fin)
        {
            lowarn("agvbase") << "AgvBase:" << __agv_id << " set_optseq_curtask_phase,__opt_sequence[__cur_opt_sequence_idx]->get_task_phase() == p == AgvTaskPhase_Fin,muti set return 0";

            return 0;
        }

        if (idex >= 0)
        {
            if (idex != __cur_opt_sequence_idx)
            {
                lowarn("agvbase") << "AgvBase:" << __agv_id << " set_optseq_curtask_phase,idex unmatch,idex=" << idex << " cur_idex=" << __cur_opt_sequence_idx;

                return 0;
            }
        }


		__opt_sequence[__cur_opt_sequence_idx]->set_task_phase( p );

		if ( p == AgvTaskPhase_Fin ) {
			__cur_opt_sequence_idx++;
			if ( __cur_opt_sequence_idx == ( int ) __opt_sequence.size() )    //finish
			{
				seq_complete = true;
			} else {
				loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " optseq trrigger next atomtask ,idx=" << __cur_opt_sequence_idx << " ";
			}
		}
	}

	if ( seq_complete ) {
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " optseq complete";

		callback_optseq_status( s, AgvInterfaceError_OK );
		return 0;
	} else {
		//trrigger next task
		if ( p == AgvTaskPhase_Fin ) {
			std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
			get_task_from_optseq( task );
			if ( task
				&& task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
				&& task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus ) {
				post_task( task );
			}
		}
	}

	return 0;
}

int agv_base::get_cur_opt_task_id( uint64_t& id ) {
	id = -1;
	{
		std::lock_guard<decltype( __mtx_opt_sequence )> lock( __mtx_opt_sequence );
		if ( __opt_sequence.empty() ) {
			return -1;
		}

		std::shared_ptr<agv_atom_opt_taskdata_header> task = std::static_pointer_cast< agv_atom_opt_taskdata_header, agv_atom_taskdata_base >( __opt_sequence[0] );
		id = task->get_atomtask_id();
	}
	return 0;
}

void agv_base::callback_optseq_status( status_describe_t status, int err ) {
	std::shared_ptr<agv_atom_taskdata_header> task = nullptr;
	{
		std::lock_guard<decltype( __mtx_opt_sequence )> lock( __mtx_opt_sequence );
		if ( !__opt_sequence.empty() ) {
			task = std::static_pointer_cast< agv_atom_taskdata_header, agv_atom_taskdata_base >( __opt_sequence[0] );
		}

		//sequence final
        if (status > kStatusDescribe_FinalFunction &&task) {
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " action task_id=" << task->get_task_id() << " final, status=" << status << " err=" << err << " !";

			__cur_opt_sequence_idx = -1;
		}
	}
	if ( task
		&& task->get_atomtaskdata_type() == AgvAtomTaskData_Header ) {
		task->callback_task_status( status, err );
	}

}

status_describe_t agv_base::get_opt_sequence_prc() {
	std::lock_guard<decltype( __mtx_opt_sequence )> lock( __mtx_opt_sequence );
	return __opt_sequence_prc;
}

void agv_base::set_opt_sequnece_prc( status_describe_t s ) {
	std::lock_guard<decltype( __mtx_opt_sequence )> lock( __mtx_opt_sequence );
	__opt_sequence_prc = s;
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " set_opt_sequnece_prc=" << s;
}


void agv_base::pause_opt_task_ack( uint32_t id, const void *data ) {
	std::lock_guard<decltype( __mtx_opt_sequence_pauseopt )> lock( __mtx_opt_sequence_pauseopt );
	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " pause_opt_task_ack failed, err = !data,  id=" << id;
			break;
		}

		task_status_t* status = ( task_status_t* ) ( data );
		if ( status->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " pause_opt_task_ack failed, asio_data_.get_err() = " << status->err_
				<< ",  id=" << id;
			break;
		}

		ret = status->err_;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " pause_opt_task_ack ok";
		callback_optseq_status( kStatusDescribe_Hang, AgvInterfaceError_OK );

	} while ( 0 );

	if ( ret < 0 ) {
        do 
        {
            if (__exit) {
                break;
            }
            std::shared_ptr<agv_atom_taskdata_pauseopt> pn = std::shared_ptr<agv_atom_taskdata_pauseopt>(new agv_atom_taskdata_pauseopt);
            post_task(pn);
        } while (0);

	}
    --__cb_counter;
}
int agv_base::pauseopt() {
    var__operation_t opt;
    int e = get_operation(opt);
    if (e < 0)
    {
        return 0;
    }
    if (opt.status_.response_ == kStatusDescribe_Hang
        || (
        opt.status_.response_ < kStatusDescribe_PendingFunction
        || opt.status_.response_ > kStatusDescribe_FinalFunction)
        )
    {
        return 0;
    }

	std::lock_guard<decltype( __mtx_opt_sequence_pauseopt )> lock( __mtx_opt_sequence_pauseopt );
	uint64_t nav_id = -1;
	if ( get_cur_opt_task_id( nav_id ) < 0 ) //no task
	{
		return 0;
	}

	int r = post_operation_pause_task( __net_id, nav_id, std::bind( &agv_base::pause_opt_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_operation_pause_task( __net_id, nav_id, asio );
	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_operation_pause_task failed, r < 0 taskid=" << nav_id;

		//if ( __net_status < 0 ) {
		//          loerror("agvbase") << "AgvBase:" << __agv_id << " __net_status < 0,pauseopt paused";
		//	return -1;
		//}
		if ( __exit ) {
			return 0;
		}
		std::shared_ptr<agv_atom_taskdata_pauseopt> pn = std::shared_ptr<agv_atom_taskdata_pauseopt>( new agv_atom_taskdata_pauseopt );
		post_task( pn );

		return 0;
	}

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_operation_pause_task ok";
    ++__cb_counter;
	return 0;
}

int agv_base::pause_optseq() {
	set_opt_sequnece_prc( kStatusDescribe_Pause );

	std::shared_ptr<agv_atom_taskdata_pauseopt> pn = std::shared_ptr<agv_atom_taskdata_pauseopt>( new agv_atom_taskdata_pauseopt );
	post_task( pn );
	return 0;
}

void agv_base::resume_opt_task_ack( uint32_t id, const void *data ) {
	std::lock_guard<decltype( __mtx_opt_sequence_resumeopt )> lock( __mtx_opt_sequence_resumeopt );
	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " resume_opt_task_ack failed, err = !data, id=" << id;
			break;
		}

		task_status_t* status = ( task_status_t* ) ( data );
		if ( status->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " resume_opt_task_ack failed, asio_data_.get_err() = " << status->err_ << ", id=" << id;
			break;
		}
		ret = status->err_;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " resume_opt_task_ack ok";
		callback_optseq_status( kStatusDescribe_Running, AgvInterfaceError_OK );

		std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
		get_task_from_optseq( task );
		if ( task
			&& task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
			&& task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus ) {
			post_task( task );
		}


	} while ( 0 );

	if ( ret < 0 ) {
        do 
        {
            if (__exit) {
                break;
            }
            std::shared_ptr<agv_atom_taskdata_resumeopt> pn = std::shared_ptr<agv_atom_taskdata_resumeopt>(new agv_atom_taskdata_resumeopt);
            post_task(pn);
        } while (0);


	}
    --__cb_counter;
}

int agv_base::resumeopt() {
    var__operation_t opt;
    int e = get_operation(opt);
    if (e < 0)
    {
        return 0;
    }
    if (opt.status_.response_ == kStatusDescribe_Running
        ||(
        opt.status_.response_ < kStatusDescribe_PendingFunction
        || opt.status_.response_ > kStatusDescribe_FinalFunction)
        )
    {
        return 0;
    }

	std::lock_guard<decltype( __mtx_opt_sequence_resumeopt )> lock( __mtx_opt_sequence_resumeopt );
	uint64_t nav_id = -1;
	if ( get_cur_opt_task_id( nav_id ) < 0 ) //no task
	{
		return 0;
	}

	int r = post_operation_resume_task( __net_id, nav_id, std::bind( &agv_base::resume_opt_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_operation_resume_task( __net_id, nav_id, asio );
	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_operation_resume_task failed, r < 0 taskid=" << nav_id;

		//if ( __net_status < 0 ) {
		//          loerror("agvbase") << "AgvBase:" << __agv_id << " __net_status < 0,resumeopt paused";
		//	return -1;
		//}
		if ( __exit ) {
			return 0;
		}
		std::shared_ptr<agv_atom_taskdata_resumeopt> pn = std::shared_ptr<agv_atom_taskdata_resumeopt>( new agv_atom_taskdata_resumeopt );
		post_task( pn );

		return 0;
	}

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_operation_resume_task ok";
    ++__cb_counter;
	return 0;
}

int agv_base::resume_optseq() {
	set_opt_sequnece_prc( kStatusDescribe_Resume );

	std::shared_ptr<agv_atom_taskdata_resumeopt> pn = std::shared_ptr<agv_atom_taskdata_resumeopt>( new agv_atom_taskdata_resumeopt );
	post_task( pn );
	return 0;
}

void agv_base::cancel_opt_task_ack( uint32_t id, const void *data ) {
	std::lock_guard<decltype( __mtx_opt_sequence_cancelopt )> lock( __mtx_opt_sequence_cancelopt );
	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " cancel_opt_task_ack failed, err = !data,  id=" << id;
			break;
		}

		task_status_t* status = ( task_status_t* ) ( data );
		if ( status->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id
				<< " cancel_opt_task_ack failed, asio_data_.get_err() = " << status->err_ << ",  id=" << id;
			break;
		}

		ret = status->err_;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " cancel_opt_task_ack ok";
		callback_optseq_status( kStatusDescribe_Terminated, AgvInterfaceError_OK );

	} while ( 0 );

	if ( ret < 0 ) {
        do 
        {
            if (__exit) {
                callback_optseq_status(kStatusDescribe_Terminated, AgvInterfaceError_OK);
                break;
            }
            std::shared_ptr<agv_atom_taskdata_cancelopt> pn = std::shared_ptr<agv_atom_taskdata_cancelopt>(new agv_atom_taskdata_cancelopt);
            post_task(pn);
        } while (0);


	}

    if (__agv_driver)
    {
        __agv_driver->CancelTaskPath();
    }
    --__cb_counter;

}

int agv_base::cancelopt() {
    var__operation_t opt;
    int e = get_operation(opt);
    if (e < 0)
    {
        return 0;
    }
    uint64_t nav_id = -1;
    if (get_cur_opt_task_id(nav_id) < 0) //no task
    {
        return 0;
    }
    if ((
        opt.status_.response_ < kStatusDescribe_PendingFunction
        || opt.status_.response_ > kStatusDescribe_FinalFunction)
        && nav_id == opt.i.current_task_id_)
    {
        callback_optseq_status(kStatusDescribe_Terminated, AgvInterfaceError_OK);
        return 0;
    }

    //if (nav_id != opt.i.current_task_id_)
    //{
    //    //callback_optseq_status(kStatusDescribe_Terminated, AgvInterfaceError_OK);
    //    //return 0;
    //    nav_id = opt.i.current_task_id_;
    //}             

	std::lock_guard<decltype( __mtx_opt_sequence_cancelopt )> lock( __mtx_opt_sequence_cancelopt );   

	int r = post_operation_cancle_task( __net_id, nav_id, std::bind( &agv_base::cancel_opt_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_operation_cancle_task( __net_id, nav_id, asio );
	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_operation_cancle_task failed, r < 0 taskid=" << nav_id;

		//if ( __net_status < 0 ) {
		//          loerror("agvbase") << "AgvBase:" << __agv_id << " __net_status < 0,cancelopt paused";
		//	return -1;
		//}
		if ( __exit ) {
			return 0;
		}
		std::shared_ptr<agv_atom_taskdata_cancelopt> pn = std::shared_ptr<agv_atom_taskdata_cancelopt>( new agv_atom_taskdata_cancelopt );
		post_task( pn );

		return 0;
	}

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_operation_cancle_task ok";
    ++__cb_counter;
	return 0;
}

int agv_base::cancel_optseq() {
	set_opt_sequnece_prc( kStatusDescribe_Cancel );

	std::shared_ptr<agv_atom_taskdata_cancelopt> pn = std::shared_ptr<agv_atom_taskdata_cancelopt>( new agv_atom_taskdata_cancelopt );
	post_task( pn );
	return 0;
}

int agv_base::get_voltage( double& vol ) const{

    if (__is_gz_arm_core)
    {
        vol = (double)__dio_main.i.ai_[2].data_[0] / 100;
    }
    else
    {
        lowarn("agvbase") << "AgvBase:" << __agv_id << " get_voltage not support, it is not gz arm, child shoud has its own get_voltage() function";  
        return -2;
    }
	
    return 0;
}

int agv_base::post_task2threadpool( std::shared_ptr<agv_tp_task_base> task ) {
	nsp::toolkit::singleton<task_threadpool>::instance()->g_task_threadpool.post( task );
	return 0;
}

void agv_base::post_modopt_task_ack( uint32_t id, const void *data ) {
	std::lock_guard<decltype( __mtx_opt_sequence_modopt )> lock( __mtx_opt_sequence_modopt );
	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_modopt_task_ack failed, err = !data, id=" << id;
			break;
		}

		mn::asio_t *asio_data_ = ( mn::asio_t* )( data );
		if ( asio_data_->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id
				<< " post_modopt_task_ack failed, asio_data_.get_err() = " << asio_data_->err_ << ", id=" << id;
			break;
		}
		ret = asio_data_->err_;
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_modopt_task_ack ok";
		set_optseq_curtask_phase( AgvTaskPhase_Fin );

	} while ( 0 );

	if ( ret < 0 ) {
        do 
        {
            set_optseq_curtask_phase(AgvTaskPhase_Send);
            if (__net_status < 0) {
                loerror("agvbase") << "AgvBase:" << __agv_id << " in modoptack, __net_status < 0,modopt paused";
                break;
            }
            if (__exit) {
                break;
            }

            std::shared_ptr<agv_atom_taskdata_base> task = nullptr;
            get_task_from_optseq(task);
            if (task) {
                if (task->get_atomtaskdata_type() > AgvAtomTaskData_Internal_Send
                    && task->get_atomtaskdata_type() < AgvAtomTaskData_CheckStatus) {
                    post_task(task);
                }
            }
        } while (0);

	}
    --__cb_counter;
}

void agv_base::post_opt_task_ack( uint32_t id, const void *data ) {
	std::lock_guard<decltype( __mtx_opt_sequence_newopt )> lock( __mtx_opt_sequence_newopt );
	int ret = -1;
	do {
		if ( !data ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_opt_task_ack failed, err = !data,  id=" << id;
			break;
		}
		mn::task_status_t* asio_data_ = (mn::task_status_t*)(data);
		if ( asio_data_->err_ < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_opt_task_ack failed, asio_data_.get_err() = "
				<< asio_data_->err_ << ",  id=" << id;
			break;
		}

		ret = asio_data_->err_;
        loinfo("agvbase") << "AgvBase:" << __agv_id << " post_opt_task_ack ok "<< asio_data_->task_id_;

        set_optseq_curtask_phase(AgvTaskPhase_Fin);

	} while ( 0 );

	if ( ret < 0 ) {
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_opt_task_ack failed! AgvTaskPhase_CheckSend";
		set_optseq_curtask_phase( AgvTaskPhase_CheckSend );
	}

    --__cb_counter;
}

int agv_base::modifyopt( std::shared_ptr<agv_atom_taskdata_modifyopt>& task ) {
	if ( task->get_task_phase() != AgvTaskPhase_Send ) {
		return 0;
	}

	std::lock_guard<decltype( __mtx_opt_sequence_modopt )> lock( __mtx_opt_sequence_modopt );
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " send modifyopt ";

	mn::common_data comm_data;
	comm_data.items = task->__vec_common;
	int r = post_common_write_request_by_id( __net_id, comm_data, std::bind( &agv_base::post_modopt_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_common_write_request_by_id( __net_id, task->__vec_common, asio );
	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " modifyopt failed, r < 0 ";

		if ( __net_status < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " __net_status < 0,modopt paused";
			return -1;
		}
		if ( __exit ) {
			return -1;
		}
		switch ( get_opt_sequence_prc() ) {
			case kStatusDescribe_Cancel:
			case kStatusDescribe_Pause:
				break;
			case kStatusDescribe_Resume:
				{
					post_task( task );
				}
				break;
			default:
				break;
		}

		return -1;
	}
	set_optseq_curtask_phase( AgvTaskPhase_WaitAck );
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " send modifyopt ok";
    ++__cb_counter;
	return 0;
}

int agv_base::newopt( std::shared_ptr<agv_atom_taskdata_newopt> task ) {
	if ( task->get_task_phase() != AgvTaskPhase_Send ) {
		return 0;
	}

	uint64_t opt_id = -1;
	if ( get_cur_opt_task_id( opt_id ) < 0 ) {
		return -1;
	}

	std::lock_guard<decltype( __mtx_opt_sequence_newopt )> lock( __mtx_opt_sequence_newopt );
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_allocate_operation_task "<<opt_id;

	int r = post_allocate_operation_task( __net_id, opt_id, task->__opt_data.code_, &( task->__opt_data.param0_ ), std::bind( &agv_base::post_opt_task_ack, this, std::placeholders::_1, std::placeholders::_2 ) );
	//int r = post_allocate_operation_task( __net_id, opt_id, task->__opt_data.code_, &( task->__opt_data.param0_ ), asio );
	if ( r < 0 ) {
		loerror( "agvbase" ) << "AgvBase:" << __agv_id << " post_allocate_operation_task failed, r < 0 id=" << opt_id;

		if ( __net_status < 0 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " __net_status < 0,new opt paused";
			return -1;
		}
		if ( __exit ) {
			return -1;
		}
		switch ( get_opt_sequence_prc() ) {
			case kStatusDescribe_Cancel:
			case kStatusDescribe_Pause:
				break;
			case kStatusDescribe_Resume:
				{
					post_task( task );
				}
				break;
			default:
				break;
		}

		return -1;
	}

	set_optseq_curtask_phase( AgvTaskPhase_WaitAck );
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " post_allocate_operation_task ok "<<opt_id;
    ++__cb_counter;
	return 0;
}

int agv_base::goto_action( int dock_id, int opt_id, uint64_t task_id
	, const std::function<void( int dock_id, int opt_id, std::vector<std::shared_ptr<agv_combine_taskdata_base>>& v, void* user )> &fn_locgic, void* logic_user
	, const std::function<void( uint64_t taskid, status_describe_t status, int err, void* user )> &fn_result, void* user /*= nullptr*/ ) {
	{
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( __cur_combine_sequence_idx != -1 ) {
			loerror( "agvbase" ) << "AgvBase:" << __agv_id << " goto_action " << dock_id << " " << opt_id << " last task not finished!";
			return -1;
		}
		__combine_sequence.clear();

		{
			std::shared_ptr<agv_combine_taskdata_header> p_header = std::shared_ptr<agv_combine_taskdata_header>( new agv_combine_taskdata_header( AgvCombineTaskType_GotoAction ) );
			p_header->set_task_id( task_id );
			p_header->set_goto_task_id( task_id );
			p_header->set_action_task_id( task_id );
			p_header->set_has_goto_task( dock_id > 0 );
			p_header->set_has_action_task( ( dock_id >= 0 ) && ( opt_id > 0 ) );
			p_header->set_task_status_callback( fn_result, user );
			__combine_sequence.push_back( p_header );
			__combineseq_cancel_status.init( p_header->has_goto_task(), p_header->has_action_task() );
		}

		if ( dock_id > 0 ) {
			std::shared_ptr<agv_combine_taskdata_goto> p_goto = std::shared_ptr<agv_combine_taskdata_goto>( new agv_combine_taskdata_goto );
			p_goto->dock_id = dock_id;
			__combine_sequence.push_back( p_goto );
		}
		if ( fn_locgic ) {
			fn_locgic( dock_id, opt_id, __combine_sequence, logic_user );
		} else {
			return 0;
		}

		__cur_combine_sequence_idx = 1;
		__combine_sequence_prc = kStatusDescribe_Resume;
	}

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " goto_action " << dock_id << " " << opt_id << " task_id=" << task_id;

	do {
		std::shared_ptr<agv_combine_taskdata_base> task = nullptr;
		get_task_from_combineseq( task );
		if ( task ) {
			if ( task->get_combine_taskdata_type() < AgvCombineTaskData_Internal_Send ) {
				continue;
			}

			if ( task->get_combine_taskdata_type() > AgvCombineTaskData_Internal_Send
				&& task->get_combine_taskdata_type() < AgvCombineTaskData_CheckStatus ) {
				post_task( task );
			}
		}


	} while ( 0 );

	return 0;
}

int agv_base::get_task_from_combineseq( std::shared_ptr<agv_combine_taskdata_base>& task ) {
	task = nullptr;
	{
		if ( __exit ) {
			return -1;
		}
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( __cur_combine_sequence_idx < 0
			|| __cur_combine_sequence_idx >= ( int ) __combine_sequence.size() ) {
			return -1;
		}

		switch ( __combine_sequence_prc ) {
			case kStatusDescribe_Resume:
				task = __combine_sequence[__cur_combine_sequence_idx];
				break;
			case kStatusDescribe_Pause:
				return 1;
			case kStatusDescribe_Cancel:
				//__cur_nav_sequence_idx = -1;
				return -1;
			default:
				return -1;
		}

	}
	return 0;
}


int agv_base::set_combineseq_curtask_phase( agv_task_phase p, status_describe_t status, int err ) {
	bool seq_complete = false;
	{
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( __cur_combine_sequence_idx < 0
			|| __cur_combine_sequence_idx >= ( int ) __combine_sequence.size() ) {
			return -1;
		}

		__combine_sequence[__cur_combine_sequence_idx]->set_task_phase( p );

		if ( p == AgvTaskPhase_Fin ) {
			__cur_combine_sequence_idx++;

			if ( __cur_combine_sequence_idx == ( int ) __combine_sequence.size() )    //finish
			{
				seq_complete = true;
			} else {
				loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " gotoaction trrigger next task ,idx=" << __cur_combine_sequence_idx << " ";

			}
		}
	}

	if ( seq_complete ) {
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " gotoaction complete, s=" << status << " e=" << err;

		callback_combineseq_status( status, err );
		return 0;
	} else {
		//trrigger next task
		if ( p == AgvTaskPhase_Fin ) {
			std::shared_ptr<agv_combine_taskdata_base> task = nullptr;
			get_task_from_combineseq( task );
			if ( task
				&& task->get_combine_taskdata_type() > AgvCombineTaskData_Internal_Send
				&& task->get_combine_taskdata_type() < AgvCombineTaskData_CheckStatus ) {
				post_task( task );
			}
		}
	}

	return 0;
}

int agv_base::get_cur_combine_gototask_id( uint64_t& id ) {
	id = -1;
	{
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( __combine_sequence.empty() ) {
			return -1;
		}

		std::shared_ptr<agv_combine_taskdata_header> task = std::static_pointer_cast< agv_combine_taskdata_header, agv_combine_taskdata_base >( __combine_sequence[0] );
		id = task->get_goto_task_id();
	}
	return 0;
}

int agv_base::set_cur_combine_gototask_id( uint64_t id ) {
	{
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( __combine_sequence.empty() ) {
			return -1;
		}

		std::shared_ptr<agv_combine_taskdata_header> task = std::static_pointer_cast< agv_combine_taskdata_header, agv_combine_taskdata_base >( __combine_sequence[0] );
		task->set_goto_task_id( id );
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " set_cur_combine_gototask_id =" << id;
	}
	return 0;
}

int agv_base::get_cur_combine_actiontask_id( uint64_t& id ) {
	id = -1;
	{
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( __combine_sequence.empty() ) {
			return -1;
		}

		std::shared_ptr<agv_combine_taskdata_header> task = std::static_pointer_cast< agv_combine_taskdata_header, agv_combine_taskdata_base >( __combine_sequence[0] );
		id = task->get_action_task_id();
	}
	return 0;
}

int agv_base::set_cur_combine_actiontask_id( uint64_t id ) {
	{
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( __combine_sequence.empty() ) {
			return -1;
		}

		std::shared_ptr<agv_combine_taskdata_header> task = std::static_pointer_cast< agv_combine_taskdata_header, agv_combine_taskdata_base >( __combine_sequence[0] );
		task->set_action_task_id( id );
		loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " set_cur_combine_actiontask_id =" << id;

	}
	return 0;
}

int agv_base::get_combineseq_header( std::shared_ptr<agv_combine_taskdata_header>& task ) {
	{
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( !__combine_sequence.empty() ) {
			task = std::static_pointer_cast< agv_combine_taskdata_header, agv_combine_taskdata_base >( __combine_sequence[0] );
		}
	}

	if ( task
		&& task->get_combine_taskdata_type() == AgvCombineTaskData_Header ) {
		return 0;
	}
	task = nullptr;
	return -1;
}
void agv_base::callback_combineseq_status( status_describe_t status, int err ) {
	std::shared_ptr<agv_combine_taskdata_header> task = nullptr;
	{
		std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
		if ( !__combine_sequence.empty() ) {
			task = std::static_pointer_cast< agv_combine_taskdata_header, agv_combine_taskdata_base >( __combine_sequence[0] );
		}

		//sequence final
		if ( status > kStatusDescribe_FinalFunction ) {
			loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " gotoaction task_id=" << task->get_task_id() << " final, status=" << status << " err=" << err << " ";
			//if (status != kStatusDescribe_Error) //Èç¹ûÊÇError£¬ÔòÈÎÎñÎÞ·¨Íê³É£¬Ö±µ½È¡ÏûÈÎÎñ
			{
				__cur_combine_sequence_idx = -1;
			}

		}
	}
	if ( task
		&& task->get_combine_taskdata_type() == AgvCombineTaskData_Header ) {
		task->callback_task_status( status, err );
	}

}

status_describe_t agv_base::get_combine_sequence_prc() {
	std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
	return __combine_sequence_prc;
}

void agv_base::set_combine_sequence_prc( status_describe_t s ) {
	std::lock_guard<decltype( __mtx_combine_sequence )> lock( __mtx_combine_sequence );
	__combine_sequence_prc = s;
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " set_combine_sequence_prc=" << s;
}

int agv_base::get_cur_combine_queue_task( std::shared_ptr<agv_combine_taskdata_gotoaction>& task ) {
	std::lock_guard<decltype( __mtx_combine_queue_task )> lock( __mtx_combine_queue_task );
	if ( __cur_combine_queue_task_idx >= ( int ) __combine_queue_task.size()
		|| __cur_combine_queue_task_idx < 0 ) {
		return -1;
	}
	switch ( __combine_queue_task_prc ) {
		case kStatusDescribe_Resume:
			task = std::static_pointer_cast< agv_combine_taskdata_gotoaction, agv_combine_taskdata_base >( __combine_queue_task[__cur_combine_queue_task_idx] );
			break;
		case kStatusDescribe_Pause:
			return 1;
		case kStatusDescribe_Cancel:
			//__cur_nav_sequence_idx = -1;
			return -1;
		default:
			return -1;
	}
	return 0;
}

int agv_base::callback_cur_combine_queue_task_status( uint64_t task_id, status_describe_t status, int err ) {
	std::lock_guard<decltype( __mtx_combine_queue_task )> lock( __mtx_combine_queue_task );
	if ( status == kStatusDescribe_Terminated && __combine_queue_task_prc == kStatusDescribe_Cancel ) {
		while ( __cur_combine_queue_task_idx < ( int ) __combine_queue_task.size()
			&& __cur_combine_queue_task_idx >= 0 ) {

			std::shared_ptr<agv_combine_taskdata_gotoaction> task = std::static_pointer_cast< agv_combine_taskdata_gotoaction, agv_combine_taskdata_base >( __combine_queue_task[__cur_combine_queue_task_idx] );
			if ( task ) {
				task->callback_task_status( kStatusDescribe_Terminated, err );
			}

			++__cur_combine_queue_task_idx;
		}
	} else {
		if ( __cur_combine_queue_task_idx >= ( int ) __combine_queue_task.size()
			|| __cur_combine_queue_task_idx < 0 ) {
			return -1;
		}

		std::shared_ptr<agv_combine_taskdata_gotoaction> task = std::static_pointer_cast< agv_combine_taskdata_gotoaction, agv_combine_taskdata_base >( __combine_queue_task[__cur_combine_queue_task_idx] );
		if ( task && task->get_task_id() == task_id ) {
			task->callback_task_status( status, err );
		}
	}

	return 0;
}

int agv_base::push_task( uint64_t &task_id, std::shared_ptr<agv_combine_taskdata_gotoaction> task, std::function<void( uint64_t taskid, status_describe_t status, int err, void* user )> fn, void* user /*= nullptr*/ ) {
	int need_trrigger = 0;
	{
		std::lock_guard<decltype( __mtx_combine_queue_task )> lock( __mtx_combine_queue_task );

		if ( __cur_combine_queue_task_idx == __combine_queue_task.size() )  //last queue all finished
		{
			need_trrigger = 1;
		}
		if ( __cur_combine_queue_task_idx < 0 )//first time
		{
			__cur_combine_queue_task_idx = 0;
			need_trrigger = 1;
		}
		if ( need_trrigger ) {
			__combine_queue_task_prc = kStatusDescribe_Resume;
		}

		task_id = ++__combine_task_id;
		task->set_task_status_callback( fn, user );
		task->set_task_id( task_id );
		__combine_queue_task.push_back( task );
	}

	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " new push_task id=" << task_id << " need_trrigger=" << need_trrigger;

	if ( need_trrigger ) {
		std::shared_ptr<agv_combine_taskdata_gotoaction> t;
		if ( get_cur_combine_queue_task( t ) < 0 ) {
			return -1;
		}
		post_task( t );

	}

	return 0;
}

int agv_base::get_nav_dis2dest( double& dis ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_nav )> lock( __mtx_nav );
	dis = __nav.i.dist_to_dest_;
	return 0;
}

int agv_base::get_nav_dis2partition( double& dis ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_nav )> lock( __mtx_nav );
	dis = __nav.i.dist_to_partition_;
	return 0;
}

int agv_base::nav_is_on_last_segment( bool& on ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_nav )> lock( __mtx_nav );
	on = ( __nav.i.on_last_segment_ == 1 );
	return 0;
}

int agv_base::get_fault( var__error_handler_t& f ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_fault )> lock( __mtx_fault );
	f = __fault;
	return 0;
}

int agv_base::get_safety_info( var__safety_t& s ) const {
	if ( __net_status < 0 ) {
		return -1;
	}

	std::lock_guard<decltype( __mtx_safety )> lock( __mtx_safety );
	s = __safety;
	return 0;
}

int agv_base::get_fault_stop_state( bool& stop ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_veh )> lock( __mtx_veh );
	stop = ( __veh.fault_stop_ != 0 ?1:0);
    if (__last_interface_error)
    {
        stop = true;
    }
	return 0;
}

int agv_base::get_nav_traj_syn( std::vector<trail_t>& pathUpl ) {
	pathUpl.clear();
	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	recv_nav_task_traj* asio_data;
	int r = query_navigation_task_traj( __net_id,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( recv_nav_task_traj* ) data;
		if ( asio_data->err_ < 0 ) {
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		pathUpl = asio_data->trajs_;
		err = 0;
		wait_request.sig();
	} );


	//int iRet = query_navigation_task_traj; 

	if ( r < 0 ) {
		return -1;
	}

	wait_request.wait();
	return err;
}

int agv_base::retry_operation_syn() {
	return -2;
}
int agv_base::clear_fault() {
    //__last_interface_error = kAgvInterfaceError_OK;

	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	asio_t* asio_data;
	int r = post_clear_fault_request( __net_id,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( asio_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}

	wait_request.wait();
	return err;

}
int agv_base::enable_safety_syn( bool  enable ) {
	std::vector<mn::common_data_item> vec_common;
	mn::common_data_item common;
	
	int enable_safety = enable ? 1 : 0;
	common.varid = kVarFixedObject_SaftyProtec;
	common.offset = offsetof( var__safety_t, enable_ );
	common.data.assign( ( char * ) &enable_safety, sizeof( enable_safety ) );
	vec_common.push_back( common );
	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	asio_t* asio_data;
	mn::common_data comm_data;
	comm_data.items = vec_common;
	int r = post_common_write_request_by_id( __net_id, comm_data,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( asio_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();

	return err;
}

int agv_base::get_net_status( int& status ) const {
	status = __net_status;
	return status;
}


int agv_base::get_agv_obj_list_asyn( std::vector<var_item>& obj ) {
	if ( __net_status < 0 ) {
		return -1;
	}
	obj.clear();
	if ( __vct_agv_obj_list.empty() ) {
		return -1;
	}
	obj = __vct_agv_obj_list;
	return 0;
}

int agv_base::get_fault_describe( std::vector<std::string> & v ) {
	if ( __net_status < 0 ) {
        v.push_back(std::string("offline"));
		return 0;
	}
	std::vector<var_item> obj;
	if ( get_agv_obj_list_asyn( obj ) < 0 ) {
		return -1;
	}
	var__error_handler_t f;
	if ( get_fault( f ) < 0 ) {
		return -1;
	}

	for ( int k = 0; k < ( int ) obj.size(); ++k ) {
		if ( f.error_[obj[k].id_].status < 0 )
			v.push_back( make_string_by_errorcode( obj[k], f.error_[obj[k].id_] ) );
	}

    if (__is_gz_arm_core)
    {
        get_gzarm_error_detail(v);
    }

    if (__last_interface_error)
    {
        v.push_back(__last_interface_error_str);
    }

    get_external_fault_describe(v);
	return 0;
}

std::string agv_base::make_string_by_errorcode( var_item obj, var__error_item_t e ) {
	char err_buf[1024] = { 0 };
	sprintf( err_buf, "id: %03d\t type: %s\t detail: %s"
		, obj.id_
		, convert_obj_type2str( obj ).c_str()
		, convert_error2str( obj, e ).c_str()
		);

	return std::string( err_buf );
}

std::string agv_base::convert_obj_type2str(var_item obj) {
    std::string typestr;
    if (convert_varobj_type2str(obj, typestr)>=0)
    {
        return typestr;
    }
    switch (obj.type_) {
		case kVarType_MotionTemplateFramwork:
			return "Framework       ";
		case kVarType_Navigation:
			return "Navigation      ";
		case kVarType_Vehicle:
			return "Vechile         ";
		case kVarType_Operation:
			return "Operation       ";
		case kVarType_UserDefined:
			return "User Buffer     ";
		case kVarType_SWheel:
			return "SWheel          ";
		case kVarType_DWheel:
			return "DWheel          ";
		case kVarType_SDDExtra:
			return "SDDExtra        ";
		case kVarType_DriveUnit:
			return "DriveUnit       ";
		case kVarType_Map:
			return "Map             ";
		case kVarType_OperationTarget:
			return "Operation param ";
		case kVarType_CanBus:
			return "Canbus          ";
		case kVarType_Copley:
			return "Copley          ";
		case kVarType_Elmo:
			return "Elmo            ";
		case kVarType_DIO:
			return "Dio             ";
		case kVarType_Moons:
			return "Moons           ";
		case kVarType_AngleEncoder:
			return "Angle encoder   ";
		case kVarType_Curtis:
			return "Curtis          ";
		case kVarType_VoiceDevice:
			return "Voice           ";
		case kVarType_OmronPLC:
			return "Omron PLC       ";
		case kVarType_ErrorHandler:
			return "Error handler   ";
		case kVarType_SafetyProtec:
			return "Safety protect  ";
        case kVarType_Localization:
            return "Localization    ";
        case kVarType_PrivateDriver:
            return "Gz Driver   ";
		default:
			break;
	}
	return "Unkown object";
}

int agv_base::convert_varobj_type2str(mn::var_item obj, std::string& type_str)
{
    return -1;
}

std::string agv_base::convert_error2str( var_item obj, var__error_item_t e ) {
    std::string error_detail;

    if (convert_varobj_error2str(obj,e,error_detail)>=0)
    {
        return error_detail;
    }
	if ( e.status < 0 ) {
		if ( obj.id_ == kVarFixedObject_MotionTemplateFramwork ) {
			switch ( e.software  ) {
				case kFramworkFatal_Common:
					return "Common error                      ";
				case kFramworkFatal_WorkerThreadNonResponse:
					return "Thread no response                ";
				case kFramworkFatal_AverageIOTimeout:
					return "IO timeout                        ";
				case kFramworkFatal_NavigationExecutiveFault:
					return "Navigation error                  ";
				case kFramworkFatal_TcpMasterServicesHalted:
					return "Tcp service halted                ";
				case kFramworkFatal_CanbusNodeNonResponse:
					return "Canbus node failed                ";
				default:
					break;
			}
		}
        else if ( obj.id_ == kVarFixedObject_Navigation ) {
			switch ( e.software  ) {
				case kNav_ErrorLocConfidence:
					return "Low localization condidence         ";
				case kNav_ErrorLocTimeOut:
					return "Localization timeout                ";
				case kNav_ErrorLocAverageTimeOut:
					return "Localization average timeout        ";
				case kNav_ErrorTraj:
					return "Navigation path error               ";
				case kNav_ErrorTracking:
					return "Navigation tracking error           ";
				case kNav_ErrorNoMap:
					return "Map error                           ";
				case kNav_ErrorOdoTimeOut:
					return "Odometer timeout                    ";
				case kNav_ErrorOdoAverageTimeOut:
					return "Odometer average timeout            ";
				case kNav_ErrorLocNotInit:
					return "Localization not initialized        ";
				default:
					break;
			}
		} 
        else if (obj.id_ == kVarFixedObject_Localization) {
            switch (e.software) {
            case kLoc_ErrorOdoException:
                return "Odo exception                       ";
            case kLoc_ErrorOdoTimeOut:
                return "Odo timeout                         ";
            case kLoc_ErrorImuException:
                return "IMU exception                       ";
            case kLoc_ErrorImuTimeOut:
                return "IMU timeout                         ";
            case kLoc_ObserveSensorException:
                return "Observe sensor exception            ";
            case kLoc_ObserveSensorTimeOut:
                return "Observe sensor timeout              ";
            case kLoc_ObserveException:
                return "Observe exception                   ";
            case kLoc_ObserveNoneInThres:
                return "Observe nothing in threshold        ";
            case kLoc_MissCode:
                return "Miss code                           ";
            case kLoc_SensorDataExtend:
                return "Sensor data have piled up           ";
            default:
                break;
            }
        }
        else  if ( obj.type_ >= kVarType_CanBus
            && obj.type_ < kVarType_OmronPLC ) {
			switch ( e.software ) {
				case kCanbusDriver_ErrorHardware:
					return "Canbus hardware error               ";
				case kCanbusDriver_ErrorFrameCountExtend:
					return "Too many canbus errorframes in total";
				case kCanbusDriver_ErrorFrameWindowExtend:
					return "High errorframes frequence          ";
				case kCanbusDriver_ErrorFrameSpeedFollowErr:
					return "Motion follow error                 ";
				case kCanbusDriver_WarnEmergencyStop:
					return "Emergency stop button               ";
				case kCanbusDriver_ErrorHoming:
					return "Homing error                        ";
				default:
					break;
			}
		}
        else  if ( obj.id_ == kVarFixedObject_Vehide) {
            std::string err;
            unsigned char drv_state = (e.hardware >> 24) & 0xFF;
            for (int i = 0; i < 8;i++)
            {
                unsigned char mask = (1 << i);
                if ((drv_state & mask) == mask)
                {
                    err += std::string("driver") + std::to_string(i) + std::string(" fault;");
                }
            }

            if (e.hardware & (8 << 16))
            {
                err += std::string("emergency stop button on;");
            }

            unsigned char vcu_err = (e.hardware) & 0xFFFF;
            if (vcu_err)
            {
                err += std::string("VCU error ,code=") + std::to_string(vcu_err) + std::string(";");
            }

            return err;
        }

	} else {
		return "ok                       ";
	}

	char err[100] = { 0 };
	sprintf( err, "unkown error,id:%d software err:%08X hardware err:%08X",obj.id_, e.software, e.hardware );

	return err;
}

int agv_base::convert_varobj_error2str(mn::var_item obj, var__error_item_t e, std::string& error_detail)
{
    return -1;
}

int agv_base::emergent_stop_syn( bool stop ) {
	std::vector<mn::common_data_item> vec_common;
	mn::common_data_item common;
		
	int emergency_stop = stop ? 1 : 0;
	common.varid = kVarFixedObject_Vehide;
	common.offset = offsetof( var__vehicle_t, stop_emergency_ );
	
	common.data.assign( ( char * ) &emergency_stop, sizeof( emergency_stop ) );
	vec_common.push_back( common );

	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	asio_t* asio_data;
	mn::common_data comm_data;
	comm_data.items = vec_common;
	int r = post_common_write_request_by_id( __net_id, comm_data,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( asio_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();

	return err;
}

int agv_base::get_emergent_stop_state( bool& stop ) const {
	if ( __net_status < 0 ) {
		return -1;
	}
	std::lock_guard<decltype( __mtx_veh )> lock( __mtx_veh );
	stop = ( __veh.stop_emergency_ == 1 );
	return 0;
}

int agv_base::new_operation_syn( const var__operation_t &op ) {
	int err = 0;
	nsp::os::waitable_handle wait_request(0);
	task_status_t* asio_data;
	int r = post_allocate_operation_task( __net_id, ++__opt_task_id, op.code_, ( uint64_t* ) ( &( op.param0_ ) ),
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( task_status_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		if (asio_data->status_ < 0) {
			wait_request.sig();
			return;
		}
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();
	return err;
}

int agv_base::modify_operation_syn( const var__operation_t &op ) {
	std::vector<mn::common_data_item> vec_common;
	mn::common_data_item common;

	common.varid = kVarFixedObject_Operation;
	common.vartype = kVarType_Operation;
	common.offset = offsetof( var__operation_t, param0_ );
	common.data.assign( ( const char * ) &op.param0_, offsetof( var__operation_t, i.param10_ ) - common.offset );
	vec_common.push_back( common );

	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	asio_t* asio_data;
	mn::common_data comm_data;
	comm_data.items = vec_common;
	int r = post_common_write_request_by_id( __net_id, comm_data,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( asio_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();

	return err;
}

int agv_base::pause_operation_syn() {
	std::vector<mn::common_data_item> vec_common;
	mn::common_data_item common;
	var__operation_t  opset;

	memset( &opset, 0, sizeof( opset ) );
	opset.status_.command_ = kStatusDescribe_Pause;

	common.varid = kVarFixedObject_Operation;
	common.vartype = kVarType_Operation;
	common.offset = 0;
	
	common.data.assign( ( const char * ) &opset.status_.command_, sizeof( opset.status_.command_ ) );
	vec_common.push_back( common );

	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	asio_t* asio_data;
	mn::common_data comm_data;
	comm_data.items = vec_common;
	int r = post_common_write_request_by_id( __net_id, comm_data,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( asio_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();

	return err;
}

int agv_base::resume_operation_syn() {
	std::vector<mn::common_data_item> vec_common;
	mn::common_data_item common;
	var__operation_t  opset;

	memset( &opset, 0, sizeof( opset ) );
	opset.status_.command_ = kStatusDescribe_Resume;

	common.varid = kVarFixedObject_Operation;
	common.vartype = kVarType_Operation;
	common.offset = 0;
	common.data.assign( ( const char * ) &opset.status_.command_, sizeof( opset.status_.command_ ) );
	vec_common.push_back( common );

	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	asio_t* asio_data;
	mn::common_data comm_data;
	comm_data.items = vec_common;
	int r = post_common_write_request_by_id( __net_id, comm_data,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( asio_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();

	return err;
}

int agv_base::cancel_operation_syn() {
	std::vector<mn::common_data_item> vec_common;
	mn::common_data_item common;
	var__operation_t  opset;

	memset( &opset, 0, sizeof( opset ) );
	opset.status_.command_ = kStatusDescribe_Cancel;

	common.varid = kVarFixedObject_Operation;
	common.vartype = kVarType_Operation;
	common.offset = 0;
	
	common.data.assign( ( const char * ) &opset.status_.command_, sizeof( opset.status_.command_ ) );
	vec_common.push_back( common );

	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	asio_t* asio_data;
	mn::common_data comm_data;
	comm_data.items = vec_common;
	int r = post_common_write_request_by_id( __net_id, comm_data,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( asio_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();

	return err;
}

int agv_base::pause_nav_syn() {
	int err = -1;
	nsp::os::waitable_handle wait_request(0);
	task_status_t* asio_data;
	int r = post_navigation_pause_task( __net_id, __nav.user_task_id_,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio_data = ( task_status_t* ) data;
		if ( asio_data->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio_data) {
			wait_request.sig();
			return;
		}
		if (asio_data->status_ < 0) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();
	return err;

}

int agv_base::resume_nav_syn() {
	int err = -1;
	task_status_t*asio;
	nsp::os::waitable_handle wait_request(0);
	int r = post_navigation_resume_task( __net_id, __nav.user_task_id_,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio = ( task_status_t* ) data;
		if ( asio->err_ < 0 ) {
			err = -1;
			wait_request.sig();
			return;
		}
		if (!asio) {
			wait_request.sig();
			return;
		}
		if (asio->status_ < 0) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();

	return err;

}

int agv_base::cancel_nav_syn() {
	int err = -1;
	asio_t*asio;
	nsp::os::waitable_handle wait_request(0);
	int r = post_navigation_cancel_task( __net_id, __nav.user_task_id_,
		[&] ( uint32_t robot_id, const void *data ) {
		if ( !data ) {
			err = -1;
			wait_request.sig();
			return;
		}
		asio = ( asio_t* ) data;
		if ( asio->err_ < 0 ) {
			err = -2;
			wait_request.sig();
			return;
		}
		if (!asio) {
			wait_request.sig();
			return;
		}
		if (asio->err_ < 0) {
			wait_request.sig();
			return;
		}
		err = 0;
		wait_request.sig();
	} );

	if ( r < 0 ) {
		return -1;
	}
	wait_request.wait();

	return err;

}


int agv_base::get_vehcile(var__vehicle_t& v) const
{
    if (__net_status < 0) {
        return -1;
    }

    std::lock_guard<decltype(__mtx_veh)> lock(__mtx_veh);
    v = __veh;
    return 0;
}


int agv_base::get_nav_info( var__navigation_t& nav ) const {
	if ( __net_status < 0 ) {
		return -1;
	}

	std::lock_guard<decltype( __mtx_nav )> lock( __mtx_nav );
	nav = __nav;
	return 0;
}

int agv_base::get_operation( var__operation_t& op ) const {
	if ( __net_status < 0 ) {
		return -1;
	}

	std::lock_guard<decltype( __mtx_opt )> lock( __mtx_opt );
	op = __opr;
	return 0;
}

int agv_base::goto_charge_task( int dock_id, int charge_on, uint64_t& task_id, std::function<void( uint64_t taskid, status_describe_t status, int err, void* user )> fn, void* user /*= nullptr*/ ) {
	return goto_charge( dock_id, charge_on, task_id, fn, user );
}



int agv_base::pause_gotoaction() {
    if (__net_status < 0)
    {
        return -1;
    }
	set_combine_sequence_prc( kStatusDescribe_Pause );

	std::shared_ptr<agv_combine_taskdata_pausegotoaction> pn =
		std::shared_ptr<agv_combine_taskdata_pausegotoaction>( new agv_combine_taskdata_pausegotoaction );
	post_task( pn );

	return 0;
}

int agv_base::resume_gotoaction() {
    if (__net_status < 0)
    {
        return -1;
    }
	set_combine_sequence_prc( kStatusDescribe_Resume );

	std::shared_ptr<agv_combine_taskdata_resumegotoaction> pn =
		std::shared_ptr<agv_combine_taskdata_resumegotoaction>( new agv_combine_taskdata_resumegotoaction );
	post_task( pn );

	return 0;
}

int agv_base::cancel_gotoaction() {
    if (__net_status < 0)
    {
        return -1;
    }
	set_combine_sequence_prc( kStatusDescribe_Cancel );

	std::shared_ptr<agv_combine_taskdata_cancelgotoaction> pn =
		std::shared_ptr<agv_combine_taskdata_cancelgotoaction>( new agv_combine_taskdata_cancelgotoaction );
	post_task( pn );

	return 0;
}

int agv_base::pause_gotocharge() {
	return pause_gotoaction();
}

int agv_base::resume_gotocharge() {
	return resume_gotoaction();
}

int agv_base::cancel_gotocharge() {
	return cancel_gotoaction();
}

status_describe_t agv_base::get_combine_queue_task_prc() {
	std::lock_guard<decltype( __mtx_combine_queue_task )> lock( __mtx_combine_queue_task );
	return __combine_queue_task_prc;
}

void agv_base::set_combine_queue_task_prc( status_describe_t s ) {
	std::lock_guard<decltype( __mtx_combine_queue_task )> lock( __mtx_combine_queue_task );
	__combine_queue_task_prc = s;
	loinfo( "agvbase" ) << "AgvBase:" << __agv_id << " set_combine_queue_task_prc=" << s;
}

int agv_base::pause_taskqueue() {
    if (__net_status < 0)
    {
        return -1;
    }
	set_combine_queue_task_prc( kStatusDescribe_Pause );
	return pause_gotoaction();
}

int agv_base::resume_taskqueue() {
    if (__net_status < 0)
    {
        return -1;
    }
	set_combine_queue_task_prc( kStatusDescribe_Resume );
	return resume_gotoaction();
}

int agv_base::cancel_taskqueue() {

    if (__net_status < 0)
    {
        return -1;
    }
	{

		std::lock_guard<decltype( __mtx_combine_queue_task )> lock( __mtx_combine_queue_task );

		if ( __cur_combine_queue_task_idx == __combine_queue_task.size() )  //last queue all finished
		{
			__combine_queue_task.clear();
			__cur_combine_queue_task_idx = 0;
			return 0;
		}
	}

	set_combine_queue_task_prc( kStatusDescribe_Cancel );
	return cancel_gotoaction();
}

void agv_base::post_task(const std::shared_ptr<agv_taskdata_base>& t ) {
	if ( __exit ) {
		return;
	}
	++__tp_task_counter;
	std::shared_ptr<agv_tp_task_base> tp = std::shared_ptr<agv_tp_task_base>( new agv_tp_task4agvbase( this, t ) );
	nsp::toolkit::singleton<task_threadpool>::instance()->g_task_threadpool.post( tp );
}

int agv_base::get_internal_dio_info(var__dio_t& dio)const  {
    if (__net_status < 0) {
        return -1;
    }
    std::lock_guard<decltype(__mtx_dio_main)> lock(__mtx_dio_main);
    dio = __dio_main;
    return 0;
}

int agv_base::get_gzarm_error_detail(std::vector<std::string>& v)
{
    if (__net_status < 0) {
        return -1;
    }
    var__dio_t internal_dio;
    get_internal_dio_info(internal_dio);
    int vcu_err = internal_dio.i.ai_[1].data_[1];
    if (vcu_err != 0)
    {
        std::string err;

        for (int i = 0;
            i < 32  // DWORD
            ; i++)
        {
            int err_mask = (0x0001 << i);
            if ((vcu_err&err_mask) == err_mask)
            {
                if (convert_gzarm_vcu_error2str_bybit(i, err) < 0)
                {
                    switch (i)//bit
                    {
                    case 0:
                    {
                              err = "vcu err:not calibrated";
                    }
                        break;
                    case 1:
                    {
                              err = "vcu err:time syn failed";
                    }
                        break;
                    case 2:
                    {
                              err = "vcu err:#0 can error";
                    }
                        break;
                    case 3:
                    {
                              err = "vcu err:#1 can error";
                    }
                        break;
                    case 4:
                    {
                              err = "vcu err:#0 rs485 error";
                    }
                        break;
                    case 5:
                    {
                              err = "vcu err:#1 rs485 error";
                    }
                        break;
                    default:
                    {
                               err = "vcu err: bit" + std::to_string(i) + " unkown error";

                    }
                        break;
                    }
                }
                
                v.push_back(err);
            }
        }

    }


    for (int i = 0; i < GZ_ARM_DRIVER_NUM;++i)
    {
        int drv_err = internal_dio.i.ai_[0].data_[i];
        if (drv_err)
        {
            std::string driver_name;
            std::string driver_err;
            if (convert_gzarm_driver_id2str(i, driver_name) < 0)
            {
                driver_name = std::string("driver#") + std::to_string(i) ;
            }

            if (convert_gzarm_driver_error2str(i,drv_err, driver_err)<0)
            {
                driver_err = std::string(" error:") + std::to_string(drv_err);
            }

            std::string err = driver_name + std::string(" ") + driver_err;
            
            v.push_back(err);
        } 
    }

    {
        bool enabled = false;
        if (get_vehcile_enable_state(enabled) >= 0)
        {
            if (!enabled)
            {
                std::string err = std::string("warning: vechile not enabled");
                v.push_back(err);
            }
        }

    }

    {
        if ((internal_dio.i.di_&0x08) == 0x08)
        {
            std::string err = std::string("warning: stop button on");
            v.push_back(err);
        }
    }
    return 0;
}

int agv_base::get_vehcile_enable_state(bool &enabled) const
{
    if (__net_status < 0) {
        return -1;
    }
    std::lock_guard<decltype(__mtx_veh)> lock(__mtx_veh);
    enabled = (__veh.i.enabled_ == 1);
    return 0;
}

int agv_base::detour_dock(std::vector<via_dock_info> via_docks)
{
    {
        std::lock_guard<decltype(__mtx_map_detour_wait)> lock(__mtx_map_detour_wait);
        __map_detour_wait.clear();
    }

    if (via_docks.empty())
    {
        loerror("agvbase") << "AgvBase:" << __agv_id << " detour_dock failed,via_docks.empty()";
        return -1;
    }

    for (auto& dock : via_docks)
    {
        loinfo("agvbase") << "AgvBase:" << __agv_id << " detour_dock:" << dock.dock_id << " wait:" << dock.wait;
    }

    upl_t final_destUpl;
    {//取出最终的目的点
        std::lock_guard<decltype(__mtx_nav_sequence)> lock(__mtx_nav_sequence);
        if (__cur_nav_sequence_idx != 4) {
            loerror("agvbase") << "AgvBase:" << __agv_id << " __cur_nav_sequence_idx != 4 ,detour_dock failed!";
            return -1;
        }
        std::shared_ptr<agv_atom_taskdata_pathsearch> p_path = std::static_pointer_cast< agv_atom_taskdata_pathsearch, agv_atom_taskdata_base >(__nav_sequence[2]);
        final_destUpl = p_path->destUpl;
    }

    std::vector<trail_t> pathUpl_whole;
    if (get_nav_traj_syn(pathUpl_whole) < 0)
    {
        loerror("agvbase") << "AgvBase:" << __agv_id << " get_nav_traj_syn < 0,detour_dock failed!";
        return -1;
    }

    if (pathUpl_whole.empty())
    {
        loerror("agvbase") << "AgvBase:" << __agv_id << " pathUpl_whole.empty(),detour_dock failed!";
        return -1;
    }

    upl_t cur_upl = __nav.i.upl_;
    cur_upl.percentage_ /= 100;

    upl_t last_destUpl = cur_upl;
    std::map<int, std::shared_ptr<detour_wait_info> > map_detour_wait;
    for (auto& dock:via_docks)
    {
        //连接所有中途点的搜路结果
        upl_t destUpl;
        position_t dest_pos;
        if (__layout->get_dest_by_dockid(dock.dock_id, destUpl, dest_pos) < 0) {
            loerror("agvbase") << "AgvBase:" << __agv_id << " dock= " << dock.dock_id << " not exist,detour_dock failed!";
            return -2;
        }


        double dis = 0;
        std::vector<trail_t> pathUpl;
        if (__layout->path_search(last_destUpl, destUpl, pathUpl, dis) < 0) {
            //err
            loerror("agvbase") << "AgvBase:" << __agv_id << " path_search from (" << cur_upl.edge_id_ << " " << cur_upl.percentage_ << " " << cur_upl.angle_ << ") to("
                << destUpl.edge_id_ << " " << destUpl.percentage_ << " " << destUpl.angle_ << ") failed! detour_dock failed";
            return -1;
        } 

        trail_t l = pathUpl_whole.back();
        trail_t r = pathUpl.front();
        if (l.edge_id_ == r.edge_id_ && l.wop_id_ == r.wop_id_)
        {
            if (pathUpl.size() > 1)
            {
                pathUpl_whole.insert(pathUpl_whole.end(), pathUpl.begin() + 1, pathUpl.end());
            } 
        }
        else
        {
            pathUpl_whole.insert(pathUpl_whole.end(), pathUpl.begin(), pathUpl.end());

        }             

        std::shared_ptr<detour_wait_info> dwi = std::shared_ptr<detour_wait_info>(new detour_wait_info);
        dwi->dock_id = dock.dock_id;
        dwi->wait = dock.wait;
        dwi->wait_idx = pathUpl_whole.size()-1;
        map_detour_wait[dwi->dock_id] = dwi;
        last_destUpl = destUpl;
        loinfo("agvbase") << "AgvBase:" << __agv_id << " detour_dock block poiont:" << dock.dock_id << " wait:" << dock.wait << " idx=" << dwi->wait_idx;
    }       

    {
        //连接到最终点
        double dis = 0;
        std::vector<trail_t> pathUpl;
        if (__layout->path_search(last_destUpl, final_destUpl, pathUpl, dis) < 0) {
            //err
            loerror("agvbase") << "AgvBase:" << __agv_id << " path_search from (" << last_destUpl.edge_id_ << " " << last_destUpl.percentage_ << " " << last_destUpl.angle_ << ") to("
                << final_destUpl.edge_id_ << " " << final_destUpl.percentage_ << " " << final_destUpl.angle_ << ") failed,detour_dock failed!";
            return -1;
        }
        //pathUpl_whole.insert(pathUpl_whole.end(), pathUpl.begin()+1, pathUpl.end());
        trail_t l = pathUpl_whole.back();
        trail_t r = pathUpl.front();
        if (l.edge_id_ == r.edge_id_ && l.wop_id_ == r.wop_id_)
        {
            if (pathUpl.size() > 1)
            {
                pathUpl_whole.insert(pathUpl_whole.end(), pathUpl.begin() + 1, pathUpl.end());
            }
        }
        else
        {
            pathUpl_whole.insert(pathUpl_whole.end(), pathUpl.begin(), pathUpl.end());

        }
    }

    //set traffic
    if (__agv_driver) {
		if (false == __agv_driver->UpdateTaskPath(pathUpl_whole)) {
			loerror("agvbase") << "AgvBase:" << __agv_id << " update task path failed...";
			return -3;
		}
    }


    //log    
    for (auto& path : pathUpl_whole)
    {
        loinfo("agvbase") << "AgvBase:" << __agv_id << " detour_dock path:" << path.edge_id_ << " " << path.wop_id_;
    }

    {
        std::lock_guard<decltype(__mtx_map_detour_wait)> lock(__mtx_map_detour_wait);
        __map_detour_wait = map_detour_wait;
    }
    return 0;
}

int agv_base::get_detour_blocked_dock()
{
    return __detour_blocked_dock;
}

int agv_base::go_on_detour(int dock)
{
    {
        std::lock_guard<decltype(__mtx_map_detour_wait)> lock(__mtx_map_detour_wait);
        auto itr = __map_detour_wait.find(dock);
        if (itr != __map_detour_wait.end())
        {
            __map_detour_wait.erase(itr);
        }
    }

    return 0;

}

int agv_base::get_cur_detourwait_dockinf(int idx,int size,std::shared_ptr<detour_wait_info>& dwi)
{
    dwi = nullptr;
    if (size<=0)
    {
        return -1;
    }
    {
        std::lock_guard<decltype(__mtx_map_detour_wait)> lock(__mtx_map_detour_wait);
        for (auto& itr:__map_detour_wait)
        {
            std::shared_ptr<detour_wait_info>& tmp = itr.second;
            if (tmp)
            {
                if (tmp->wait_idx >= idx && tmp->wait_idx <= (idx + size))
                {
                    if (tmp->wait_counter >= tmp->wait && tmp->wait >= 0)
                    {
                        tmp = nullptr;
                        break;
                    }
                    else
                    {
                        dwi = tmp;
                        return 0;
                    }                                    
                }
                else   if (tmp->wait_idx < idx)
                {
                    tmp = nullptr;
                }
            }
            else
            {
                continue;
            }

        }
    }
    return -1;
}

int agv_base::get_external_fault_describe(std::vector<std::string> & v)
{
    return 0;
}

int agv_base::convert_gzarm_driver_id2str(int gz_driver_id, std::string& driver_name)
{
    return -1;
}

int agv_base::convert_gzarm_driver_error2str(int gz_driver_id, int driver_err, std::string& driver_err_str)
{
    return -1;
}

int agv_base::convert_gzarm_vcu_error2str_bybit(int vcu_err, std::string& vcu_err_str)
{
    return -1;
}

int agv_base::get_battery_power(double& p) const
{
    if (__net_status < 0) {
        return -1;
    }
    p = 0;
    if (__is_gz_arm_core)
    {
        var__dio_t internal_dio;
        get_internal_dio_info(internal_dio);
        int total = internal_dio.i.ai_[2].data_[2];
        if (total != 0)
            p = (double)internal_dio.i.ai_[2].data_[3] / total;
    }
    else
    {
        lowarn("agvbase") << "AgvBase:" << __agv_id << " get_battery_power not support, it is not gz arm, child shoud has its own get_battery_power() function";
        return -2;
    }

    return 0;
}

int agv_base::get_agv_sleep_status(int& sleep)
{
     if (__is_gz_arm_core)
     {
         if (__net_status < 0) {
             return -1;
         }
         var__dio_t internal_dio;
         get_internal_dio_info(internal_dio);
         sleep = (internal_dio.i.di_&(0x01 << 5)) == (0x01 << 5) ? 1 : 0;
     } 
     else
     {
         lowarn("agvbase") << "AgvBase:" << __agv_id << " get_agv_sleep_status not support, it is not gz arm, child shoud has its own get_agv_sleep_status() function";

         return -2;
     }

     return 0;
}

int agv_base::sleep_agv_syn()
{
    if (__is_gz_arm_core)
    {
        var__dio_t dio;
        if (agv_base::get_internal_dio_info(dio) < 0)
        {
            return -1;
        }
        dio.do_ |= (1 << 5);
        mn::common_data vct_write;
        return agv_base::common_write_by_id(kVarFixedObject_InternalDIO, vct_write, &dio, &dio.do_);
    }
    else
    {
        lowarn("agvbase") << "AgvBase:" << __agv_id << " sleep_agv_syn not support, it is not gz arm, child shoud has its own get_agv_sleep_status() function";

        return -2;
    }

    return 0;


}

int agv_base::wakeup_agv_syn()
{
    if (__is_gz_arm_core)
    {
        var__dio_t dio;
        if (agv_base::get_internal_dio_info(dio) < 0)
        {
            return -1;
        }
        dio.do_ &= ~(1 << 5);
        mn::common_data vct_write;
        return agv_base::common_write_by_id(kVarFixedObject_InternalDIO, vct_write, &dio, &dio.do_);
    }
    else
    {
        lowarn("agvbase") << "AgvBase:" << __agv_id << " wakeup_agv_syn not support, it is not gz arm, child shoud has its own get_agv_sleep_status() function";

        return -2;
    }

    return 0;

}

int agv_base::update_endpoint(const std::string &ip, uint16_t port)
{
    char epstr[32];
    sprintf(epstr, "%s:%u", ip.c_str(), port);
    return change_connect_host(__net_id, epstr);
}

int agv_base::get_battery_capacity_current(double& cur)
{
    if (__net_status < 0) {
        return -1;
    }
    cur = 0;
    if (__is_gz_arm_core)
    {
        var__dio_t internal_dio;
        get_internal_dio_info(internal_dio);
        cur = internal_dio.i.ai_[2].data_[3];
    }
    else
    {
        lowarn("agvbase") << "AgvBase:" << __agv_id << " get_battery_capacity_current not support, it is not gz arm, child shoud has its own get_battery_power() function";
        return -2;
    }

    return 0;
}

int agv_base::get_battery_capacity_total(double& total)
{
    if (__net_status < 0) {
        return -1;
    }
    total = 0;
    if (__is_gz_arm_core)
    {
        var__dio_t internal_dio;
        get_internal_dio_info(internal_dio);
        total = internal_dio.i.ai_[2].data_[2];
    }
    else
    {
        lowarn("agvbase") << "AgvBase:" << __agv_id << " get_battery_capacity_current not support, it is not gz arm, child shoud has its own get_battery_power() function";
        return -2;
    }

    return 0;
}

int agv_base::subscribe_data(int id, void *begin)
{
    __map_subscribe_data[id] = __vec_post_common;
    __vec_post_common.items.clear();
    return 0;
}

void agv_base::common_read_ack_subscrbe(uint32_t id, const void *data) {
    do {

        mn::common_data *asio_data_ = (mn::common_data*)(data);
        if (asio_data_->err_ < 0) {
            loerror("agvbase") << "AgvBase:" << __agv_id << " common_read_ack_subscrbe failed, asio_data_.get_err() = " << asio_data_->err_ << ", id=" << id;
            break;
        }
        if (asio_data_->items.size() == 0){
            break;
        }
        common_data_item ca = asio_data_->items[0];

        switch (ca.vartype) {
        case kVarType_Vehicle:
        {
                                 std::lock_guard<decltype(__mtx_veh)> lock(__mtx_veh);
                                 for (auto& itm:asio_data_->items)
                                 {
                                     memcpy((char*)&__veh + itm.offset, itm.data.c_str(), itm.data.size());
                                 }

                                 //var__vehicle_t* vh = (var__vehicle_t*)(ca.data.c_str());
                                 //memcpy(&__veh, vh, sizeof(__veh));
        }
            break;
        case kVarType_Navigation:
        {
            {
                std::lock_guard<decltype(__mtx_nav)> lock(__mtx_nav);
                for (auto& itm : asio_data_->items)
                {
                    memcpy((char*)&__nav + itm.offset, itm.data.c_str(), itm.data.size());
                }
                //var__navigation_t* nv = (var__navigation_t*)(ca.data.c_str());
                //memcpy(&__nav, nv, sizeof(__nav));
            }

                                    //::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d readnav and do traffic begin", __agv_id);
                                    upl_t cur_upl;
                                    int idex = 0;
                                    if (get_current_upl(cur_upl) >= 0) {
                                        if (get_current_traj_idex(idex) >= 0) {
                                            if (__agv_driver) {
                                                __agv_driver->UpdateUPL(cur_upl, idex);
                                                __agv_driver->DoAllocatedAnddRelease();
                                            }
                                        }
                                    }

                                    check_nav_status_from_navseq(__nav.track_status_.response_);
                                    check_nav_status_from_optseq(__nav.track_status_.response_);
                                    //::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d readnav and do traffic end", __agv_id);

        }
            break;
        case kVarType_Operation:
        {
            {
                std::lock_guard<decltype(__mtx_opt)> lock(__mtx_opt);
                for (auto& itm : asio_data_->items)
                {
                    memcpy((char*)&__opr + itm.offset, itm.data.c_str(), itm.data.size());
                }
                //var__operation_t* op = (var__operation_t*)(ca.data.c_str());
                //memcpy(&__opr, op, sizeof(var__operation_t));
            }
                                   //::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d check_opt_status begin", __agv_id);
                                   check_opt_status(__opr);
                                   //::log__save("agvbase_debug", kLogLevel_Trace, kLogTarget_Filesystem, "AgvBase:%d check_opt_status end", __agv_id);

        }
            break;
        case kVarType_ErrorHandler:
        {
                                      std::lock_guard<decltype(__mtx_fault)> lock(__mtx_fault);
                                      for (auto& itm : asio_data_->items)
                                      {
                                          memcpy((char*)&__fault + itm.offset, itm.data.c_str(), itm.data.size());
                                      }
                                      //var__error_handler_t* op = (var__error_handler_t*)(ca.data.c_str());
                                      //memcpy(&__fault, op, sizeof(var__error_handler_t));
        }
            break;
        case kVarType_SafetyProtec:
        {
                                      std::lock_guard<decltype(__mtx_safety)> lock(__mtx_safety);
                                      for (auto& itm : asio_data_->items)
                                      {
                                          memcpy((char*)&__safety + itm.offset, itm.data.c_str(), itm.data.size());
                                      }
                                      //var__safety_t* op = (var__safety_t*)(ca.data.c_str());
                                      //memcpy(&__safety, op, sizeof(var__safety_t));
        }
            break;
        case kVarType_DIO:
        {
                             if (ca.varid == kVarFixedObject_InternalDIO)
                             {
                                 std::lock_guard<decltype(__mtx_dio_main)> lock(__mtx_dio_main);
                                 for (auto& itm : asio_data_->items)
                                 {
                                     memcpy((char*)&__dio_main + itm.offset, itm.data.c_str(), itm.data.size());
                                 }
                                 //var__dio_t* vh = (var__dio_t*)(ca.data.c_str());
                                 //memcpy(&__dio_main, vh, sizeof(var__dio_t));
                             }
                             else
                             {
                                 get_elongate_variable_ack(id, data);
                             }
        }
            break;
        case kVarType_OfflineTask:
        {
            {
                std::lock_guard<decltype(__mtx_offtask)> lock(__mtx_offtask);
                for (auto& itm : asio_data_->items)
                {
                    memcpy((char*)&__var_offtask + itm.offset, itm.data.c_str(), itm.data.size());
                }
			}

									 if (__var_offtask.track_status_.response_ > kStatusDescribe_FinalFunction)
                                     {
                                         std::lock_guard<decltype(__mtx_offtask_ptr)> lock(__mtx_offtask_ptr);
                                         if (__offline_task && __offline_task->get_task_phase() == AgvTaskPhase_Fin)
                                         {
                                             __offline_task->callback_status(__var_offtask.track_status_.response_, kAgvInterfaceError_OK);
                                         }
                                     }
        }
        default:
            get_elongate_variable_ack(id, data);
            break;
        }


    } while (0);


}

int agv_base::on_login()
{
    return 0;
}

int agv_base::new_offline_task(const std::vector<offline_task_item>& vct_task, /*离线任务列表 */ uint64_t& task_id, std::function<void(uint64_t taskid, status_describe_t status, int err, void* user)> fn, void* user /*= nullptr*/, std::function<void(int percent)> fn_asyn_send_rate /*= nullptr /*离线任务下载进度，如果没有注册回调，则此接口同步阻塞 */)
{
    if (__net_status < 0) {
        return -1;
    }

    if (__offline_task)
    {
        if (__offline_task->get_task_phase() != AgvTaskPhase_None)
        {
            return -99;
        }
    }

    __offline_task = std::shared_ptr<agv_offline_taskdata>(new agv_offline_taskdata());

    std::shared_ptr<agv_offline_taskdata> p = __offline_task;
    p->__vct_task = vct_task;
    p->__task_id = ++__offline_task_id;
    p->__fn_result = fn;
    p->__user = user;
    p->__fn_asyn_send_rate = fn_asyn_send_rate;
    task_id = p->__task_id;
    p->set_task_phase(AgvTaskPhase_Send);

    if (fn_asyn_send_rate)
    {
        post_task(p);
    }
    else
    {
        return th_offline_task(p);
    }

    return 0;
}

int agv_base::cancel_offline_task(uint64_t& task_id)
{
    int err = 0;
    nsp::os::waitable_handle wait_request(0);
    task_status_t* asio_data;
    int r = mn::cancel_offline_task(__net_id, task_id,
        [&](uint32_t robot_id, const void *data) {

        asio_data = (task_status_t*)data;
        if (!data) {
            err = asio_data->err_;
            wait_request.sig();
            return;
        }
        if (asio_data->err_ < 0) {
            wait_request.sig();
            return;
        }
        if (!asio_data) {
            wait_request.sig();
            return;
        }
        wait_request.sig();
    });

    if (r < 0) {
        return -1;
    }

    wait_request.wait();
    return err;
}

int agv_base::th_offline_task(std::shared_ptr<agv_offline_taskdata> p) 
{
    upl_t cur_upl = __nav.i.upl_;
    cur_upl.percentage_ /= 100;

    mn::mn_offline_task ot;
    int cur_cnt = 0;
    int total_cnt = p->__vct_task.size() + 1;
    p->callback_rate(0);
    for (auto item:p->__vct_task)
    {
        int dock_id = item.dock_id;
        upl_t destUpl;
        position_t dest_pos;
        if (__layout->get_dest_by_dockid(dock_id, destUpl, dest_pos) < 0) {
            loerror("agvbase") << "AgvBase:" << __agv_id << " dock= " << dock_id << " not exist!";
            __last_interface_error = kAgvInterfaceError_NoDock;
            __last_interface_error_str = std::string("agvbase:dock ") + std::to_string(dock_id) + std::string(" not exist");
            p->callback_status(kStatusDescribe_Error, kAgvInterfaceError_NoDock);
            return -2;
        }

        double dis = 0;
        std::vector<trail_t> pathUpl;
        if (__layout->path_search(cur_upl, destUpl, pathUpl, dis) < 0) {
            //err
            loerror("agvbase") << "AgvBase:" << __agv_id << " path_search from (" << cur_upl.edge_id_ << " " << cur_upl.percentage_ << " " << cur_upl.angle_ << ") to("
                << destUpl.edge_id_ << " " << destUpl.percentage_ << " " << destUpl.angle_ << ") failed!";

            p->callback_status(kStatusDescribe_Error, kAgvInterfaceError_PathSearch);
            __last_interface_error = kAgvInterfaceError_PathSearch;
            __last_interface_error_str = std::string("agvbase: path_search failed, from edge ") + std::to_string(cur_upl.edge_id_) + std::string(" to")
                + std::string(" edge ") + std::to_string(destUpl.edge_id_);
            return -1;
        }

        cur_upl = destUpl;

        mn::offline_task_node_t   otn;
        otn.dest_pos_ = dest_pos;
        otn.dest_upl_ = destUpl;
        otn.trails_ = pathUpl;
        
        for (auto& op:item.ops)
        {
            offline_operator_node_t oon;
			oon.code_ = op.code_;
			oon.params_[0] = op.param0_;
			oon.params_[1] = op.param1_;
			oon.params_[2] = op.param2_;
			oon.params_[3] = op.param3_;
			oon.params_[4] = op.param4_;
			oon.params_[5] = op.param5_;
			oon.params_[6] = op.param6_;
			oon.params_[7] = op.param7_;
			oon.params_[8] = op.param8_;
			oon.params_[9] = op.param9_;
            otn.opers_.push_back(oon);
        }

        ot.push_back(otn);

        p->callback_rate(100 * (cur_cnt++) / total_cnt);
    }

    int err = 0;
    nsp::os::waitable_handle wait_request(0);
    task_status_t* asio_data;
    int r = post_offline_task(__net_id, p->__task_id, ot,
        [&](uint32_t robot_id, const void *data) {

        asio_data = (task_status_t*)data;
        if (!data) {
            err = asio_data->err_;
            wait_request.sig();
            return;
        }
        if (asio_data->err_ < 0) {
            wait_request.sig();
            return;
        }
        if (!asio_data) {
            wait_request.sig();
            return;
        }
        wait_request.sig();
    });


    if (r < 0) {
        err = -1;
    }
    else
    {
       wait_request.wait();
    }

    if (err < 0)
    {
        p->callback_rate(99); 
        p->callback_status(kStatusDescribe_Error, kAgvInterfaceError_SendFailed);
    }
    else
    {
        p->callback_rate(100);
        p->set_task_phase(AgvTaskPhase_Fin);
    }
    
    
    return err;

}