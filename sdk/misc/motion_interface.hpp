//////////////////////////////////////////////////////////////////////////
/*!
 * file motion_interface_base.hpp
 * date 2017/07/28 13:51
 *
 * author chenkai
 *
 * brief motion_interface  ,for customize coding
 *
 * note
 */
#ifndef __MOTION_INTERFACE_H_CK__
#define __MOTION_INTERFACE_H_CK__
#include <string>

#include "mntypes.h"
#include "net_impls.h"
#include "log.h"
#include "operation.h"

#include "os_util.hpp"

class motion_interface {
	int simple_common_write_byid( const mn::common_data &data_array, const char *origin ) const;
public:
	motion_interface();
	~motion_interface();

	//connect mt
    int init(std::string ip, int port, nsp__controlor_type_t id_type);
	int uninit();
public:
	//get total variable
	template<typename T> int get_var_info_by_id( int id, T& var );
	//set total variable
	template<typename T> int set_var_info_by_id( int id, const T& var );

	/*
	可变参数列表 模板函数  通用写

	示例：
	motion_interface mi;
	mi.init("127.0.0.1", 10001);

	var__operation_t op;
	op.code_ = 1;
	op.param0_ = 1000;

	mn::common_data v;
	mi.common_write_by_id(kVarFixedObject_Operation, v, &op, &op.code_, &op.param0_);

	*/
	template<class T, class ...MEMBERS_T> int common_write_by_id( int id, mn::common_data &vct_write, void *begin, T head, MEMBERS_T...members );


    template<class T, class ...MEMBERS_T> int common_read_by_id(int id, mn::common_title &vct_write, void *begin, T head, MEMBERS_T...members);
public://not template
	int control_speed( double vx, double vy, double vw );
	int cancel_nav();
	int stop_normal( int stop );
	int set_usrdef_buf_syn( int offet, char* data_usr, int len );
	int safety_enable( int enable );
	int nsp__report_status( uint64_t task_id, int id, status_describe_t sd, int u = 0, int i = 0 );

    int do_offline_nextstep_syn(uint64_t task_id);
public:
	void bind_canio_fn( std::function<void( const mn::canio_msg_t & )> fn );

private:
	void recv_event( int32_t net_id, const void *data, int type );
    int common_write_by_id(int id, mn::common_data &vct_write, void*); 
    int common_read_by_id(int id, mn::common_title &vct_write, void*);
protected:
	int   __net_id = -1;
	std::function<void( const mn::canio_msg_t & )> notify_canio_ = nullptr;
};

template<class T, class ...MEMBERS_T>
int motion_interface::common_read_by_id(int id, mn::common_title &title, void *begin, T head, MEMBERS_T...members)
{
    mn::common_title_item node;
    node.varid = id;
    node.length = sizeof(*head);
    node.offset = (char*)head - (char*)begin;
    title.items.push_back(node);
    int argc = sizeof...(members);
    if (argc > 0) {
        return common_read_by_id(id, title, begin, members...);
    }
    else
    {

        int retval = -1;
        nsp::os::waitable_handle w(0);
        mn::common_data *asio;
        retval = mn::post_common_read_request_by_id(__net_id, title, [&](uint32_t, const void *data) {
            asio = (mn::common_data *)data;
            retval = asio->err_;
            if (retval < 0) {
                loerror("motion_interface") << "common_read_by_id failed，error= " << retval << " id = " << id;
                w.sig();
                return;
            }

            for (int i = 0; i < asio->items.size(); i++) {

                memcpy((char*)begin + asio->items[i].offset, asio->items[i].data.data(), asio->items[i].data.size());

            }
            w.sig();
        });
        if (retval < 0) {//如果接口调用失败，那么直接返回
            loerror("motion_interface") << "common_read_by_id failed，ret < 0，id = " << id;
            return retval;
        }
        w.wait();
        return retval;

    }

    return 0;
}

template<typename T>
int motion_interface::get_var_info_by_id( int id, T& var ) {
	mn::common_title title;
	{
		mn::common_title_item title_item;
		title_item.varid = id;
		title_item.offset = 0;
		title_item.length = sizeof( T );
		title.items.push_back( title_item );
	}

	int retval = -1;
	nsp::os::waitable_handle w(0);
	mn::common_data *asio;
	retval = mn::post_common_read_request_by_id( __net_id, title, [&] ( uint32_t, const void *data ) {
		asio = ( mn::common_data * )data;
		retval = asio->err_;
		if ( retval < 0 ) {
			loerror( "motion_interface" ) << "get_var_info_by_id failed，error= " << retval << " id = " << id;
			w.sig();
			return;
		}

		if ( asio->items.size() > 0 ) {
			if ( asio->items[0].data.size() == sizeof( T ) ) {
				memcpy( &var, asio->items[0].data.data(), asio->items[0].data.size() );
			}
		}
		w.sig();
	} );
	if ( retval < 0 ) {//如果接口调用失败，那么直接返回
		loerror( "motion_interface" ) << "get_var_info_by_id failed，ret < 0，id = " << id;
		return retval;
	}
	w.wait();
	return retval;
}

template<class T, class ...MEMBERS_T>
int motion_interface::common_write_by_id(int id, mn::common_data &vct_write, void *begin, T head, MEMBERS_T...members) {
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
                loerror("motion_interface") << "net_id:" << __net_id << " post_common_write_request failed,id = " << id;
                water.sig();
                return;
            }

			asio_data_ = *(mn::asio_t*)(data);
            if (asio_data_.err_ < 0) {
                loerror("motion_interface") << "net_id:" << __net_id << " post_common_write_request failedï¼Œasio_data->get_err() < 0,id = " << id;
                err = -2;
                water.sig();
                return;
            }
			err = asio_data_.err_;
            water.sig();
        });
        if (iRet < 0)
        {
            loerror("motion_interface") << "net_id:" << __net_id << " post_common_write_request failed,ret ="<<iRet<<",id = " << id;

            return -1;
        }
        water.wait();
        return err;
    }

    return 0;
}

#endif
