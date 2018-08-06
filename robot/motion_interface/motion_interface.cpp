#include "motion_interface.hpp"
#include "vehicle.h"
#include "log.h"
#include "safety.h"
#include "os_util.hpp"
#include "toolkit.h"

motion_interface::motion_interface() {
}


motion_interface::~motion_interface() {
	;
}

int motion_interface::simple_common_write_byid( const mn::common_data &data_array, const char *origin ) const {
	int retval = -1;
	nsp::os::waitable_handle w(0);
	mn::asio_t* asio;

	retval = mn::post_common_write_request_by_id( __net_id, data_array, [&] ( uint32_t robot_id, const void *data ) {
		asio = ( mn::asio_t* )data;
		retval = asio->err_;
		w.sig();
	} );
	if ( retval < 0 ) {//如果接口调用失败，那么直接返回
		loerror( "motion_interface" ) << "mn call fail.callstack= " << origin;
		return retval;
	}
	w.wait();
	if ( retval < 0 ) {
		loerror( "motion_interface" ) << "common write asio fail.callstack= " << origin << " error= " << retval;
	}
	return retval;
}

#define interface_simple_write_call(data_array) \
			this->simple_common_write_byid(data_array, __FUNCTION__)

int motion_interface::control_speed( double vx, double vy, double vw ) {
	mn::common_data data_array;

	{
		mn::common_data_item data_item;
		data_item.varid = kVarFixedObject_Vehide;
		data_item.offset = offsetof( var__vehicle_t, manual_velocity_ );

		velocity_t set_v;
		set_v.x_ = vx;
		set_v.y_ = vy;
		set_v.w_ = vw;
		data_item.data = std::string( ( char* ) &set_v, sizeof( set_v ) );
		data_array.items.push_back( data_item );
	}

	{
		mn::common_data_item data_item;
		data_item.varid = kVarFixedObject_Vehide;
		data_item.offset = offsetof( var__vehicle_t, control_mode_ );
		enum vehicle_control_mode_t ctrl_mode = kVehicleControlMode_Calibrated;
		data_item.data = std::string( ( char* ) &ctrl_mode, sizeof( ctrl_mode ) );
		data_array.items.push_back( data_item );
	}

	{
		mn::common_data_item data_item;
		data_item.varid = kVarFixedObject_Vehide;
		data_item.offset = offsetof( var__vehicle_t, enable_ );
		int enable = 1;
		data_item.data = std::string( ( char* ) &enable, sizeof( enable ) );
		data_array.items.push_back( data_item );
	}
	return interface_simple_write_call( data_array );
}

int motion_interface::cancel_nav() {
	var__navigation_t nav;
	if ( get_var_info_by_id( kVarFixedObject_Navigation, nav ) < 0 ) {
		return -1;
	}

	mn::task_status_t *asio;
	nsp::os::waitable_handle w(0);
	int retval = mn::post_navigation_cancel_task( __net_id, nav.user_task_id_, [&] ( uint32_t id, const void *data ) {
		asio = ( mn::task_status_t * )data;
		retval = asio->err_;
		if ( retval < 0 ) {
			loerror( "motion_interface" ) << "post post_navigation_cancle_task command failed.asio_data_.get_err() < 0";
			w.sig();
			return;
		}
		w.sig();
	} );
	if ( retval < 0 ) {
		loerror( "motion_interface" ) << "post post_navigation_cancle_task command failed.iRet < 0";
		return -1;
	}
	w.wait();
	return retval;
}

int motion_interface::stop_normal( int stop ) {
	mn::common_data data_array;

	{
		mn::common_data_item data_item;
		data_item.varid = kVarFixedObject_Vehide;
		data_item.offset = offsetof( var__vehicle_t, stop_normal_ );
		data_item.data = std::string( ( char* ) &stop, sizeof( stop ) );
		data_array.items.push_back( data_item );
	}
	return interface_simple_write_call( data_array );
}


int motion_interface::set_usrdef_buf_syn( int offet, char* data_usr, int len ) {
	mn::common_data data_array;

	{
		mn::common_data_item data_item;
		data_item.varid = kVarFixedObject_UserDefinition;
		data_item.offset = offet;
		data_item.data = std::string( data_usr, len );
		data_array.items.push_back( data_item );
	}
	return interface_simple_write_call( data_array );
}

int motion_interface::safety_enable( int enable ) {
	mn::common_data data_array;

	{
		mn::common_data_item data_item;
		data_item.varid = kVarFixedObject_SaftyProtec;
		data_item.offset = offsetof( var__safety_t, enable_ );
		data_item.data = std::string( ( const char * ) &enable, sizeof( enable ) );
		data_array.items.push_back( data_item );
	}
	return interface_simple_write_call( data_array );
}

void motion_interface::recv_event( int32_t net_id, const void *data, int type ) {
	switch ( type ) {
		case mn::kNET_STATUS:
			{
				mn::net_status_t *net_status = ( mn::net_status_t * ) data;
				if ( *net_status < mn::kNetworkStatus_Established ) {
					loerror( "motion_interface" ) << "__net_status " << net_status << ",offline";
				} else {
					loerror( "motion_interface" ) << "__net_status " << net_status << ",online";
				}
			}
			break;
		case mn::kCANIO_MSG:
			{
				mn::canio_msg_t* canio = ( mn::canio_msg_t* ) data;
				if ( notify_canio_ ) {
					notify_canio_( *canio );
				}
			}
		default:
			break;
	}
}
int motion_interface::init(std::string ip, int port, nsp__controlor_type_t id_type) {
	if ( __net_id >= 0 ) {
		mn::disconnect_host( __net_id );
	}

    __net_id = mn::init_net();
	//连接网络
    char epstr[32] = { 0 };
#if _WIN32
	sprintf_s( epstr, cchof( epstr ), "%s:%u", ip.c_str(), port );
#else
	sprintf( epstr, "%s:%u", ip.c_str(), port );
#endif
    if (mn::login_to_host(__net_id, epstr, id_type) < 0) {
        loerror("motion_interface") << " net_id=" << __net_id << " id_type:" << id_type << " login_to_host failed";
		return -1;
	}

	mn::register_callback_to_notify( __net_id, std::bind( &motion_interface::recv_event, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );
	return 0;
}

int motion_interface::nsp__report_status( uint64_t task_id, int id, status_describe_t sd, int u, int i ) {

	mn::asio_t *asio;
	nsp::os::waitable_handle w(0);
	int retval = -1;
	retval = mn::post_report_status_order( __net_id, task_id, id, sd, "",  [&] ( uint32_t id, const void *data ) {
		asio = ( mn::asio_t* )( data );
		retval = asio->err_;
		w.sig();
	}  );
	if ( retval < 0 ) {
		return -1;
	}
	w.wait();
	return retval;
}

void motion_interface::bind_canio_fn( std::function<void( const mn::canio_msg_t &)> fn ) {
	notify_canio_ = fn;
}

int motion_interface::uninit() {
	mn::disconnect_host( __net_id );
	return 0;
}

int motion_interface::common_write_by_id(int id, mn::common_data &, void*) {
    return 0;
}

int motion_interface::common_read_by_id(int id, mn::common_title &vct_write, void*)
{
    return 0;
}

int motion_interface::do_offline_nextstep_syn(uint64_t task_id)
{
    mn::task_status_t *asio;
    nsp::os::waitable_handle w(0);
    int retval = mn::post_offline_nextstep(__net_id, task_id, [&](uint32_t id, const void *data) {
        asio = (mn::task_status_t *)data;
        retval = asio->err_;
        if (retval < 0) {
            loerror("motion_interface") << "post post_offline_nextstep command failed.asio_data_.get_err() < 0";
            w.sig();
            return;
        }
        w.sig();
    });
    if (retval < 0) {
        loerror("motion_interface") << "post post_offline_nextstep command failed.iRet < 0";
        return -1;
    }
    w.wait();
    return retval;
}
