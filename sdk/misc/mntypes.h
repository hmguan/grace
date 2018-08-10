#if !defined MOTION_NET_DEFINE_H
#define MOTION_NET_DEFINE_H

#include <vector>
#include <functional>

#include "vartypes.h"
#include "navigation.h"

namespace mn {
	
	enum net_status_t : int {
		kNetworkStatus_Closed = -1,      // this session is unusable
		kNetworkStatus_Actived = 0,      // link is already created
		kNetworkStatus_Connecting,       // connecting to target host
		kNetworkStatus_Connected,        // TCP is established,but session must login to target host befor use it.
		kNetworkStatus_Ready,            // login key has been received,login request canbe active on next step
		kNetworkStatus_Landing,        	 // try to login host server
		kNetworkStatus_Established,      // this session is usable.
	};

	enum notift_type_t : int {
		KNAVI_POS = 1,

		/* setup by @register_callback_to_notify, force convert @data to type pointer <net_status_t> */
		kNET_STATUS,

		/* setup by @register_callback_to_notify, force convert @data to type pointer <motion_report_t> */
		kTASK_STATUS_REPORT,

		/* setup by @register_callback_to_notify, force convert @data to type pointer <canio_msg_t> */
		kCANIO_MSG,

		/* setup by @registry_periodic_report, force convert @data to type pointer <periodic_data> */
		kPERIODIC_REPORT,
	};

	typedef std::function<void( uint32_t robot_id, const void *data, enum notift_type_t type)> notify_t;
	typedef const std::function<void( uint32_t, const void * ) > asynchronous_procedure_call;
	typedef asynchronous_procedure_call apc_t;

	struct asio_t {
		int err_;
	};

	struct common_title_item {
		int varid;
		int vartype; /* the latest protocol ignores this field  */
		int offset;
		int length;
	};

	/*
	post_common_read_request_by_id
	*/
	struct common_title {
		std::vector<struct common_title_item> items;
	};

	struct common_data_item {
		int varid;
		int vartype; /* the latest protocol ignores this field  */
		int offset;
		std::string data;
	};

	/*
	post_common_read_request_by_id
	*/
	struct common_data : public asio_t {
		std::vector<struct common_data_item> items;
	};

	struct periodic_title_item : public common_title_item {
		uint32_t interval;		/* interval in milliseconds between two reports */
	};
	
	/*
	registry_periodic_report <request>
	*/
	struct periodic_title {
		/* current system timestamp.
		 * on request, this field will be set by inner, ignore if you're using motion-net module
		 * on response, this field will be set by motion template */
		uint64_t timestamp;		
		std::vector<struct periodic_title_item> items;
	};

	struct periodic_data_item : public common_data_item {
		uint32_t interval;
	};
	
	/*
	registry_periodic_report <response>
	*/
	struct periodic_data : public asio_t {
		/* current system timestamp.
		 * on request, this field will be set by inner, ignore if you're using motion-net module
		 * on response, this field will be set by motion template */
		uint64_t timestamp;
		std::vector<struct periodic_data_item> items;
	};

	/*
	query_navigation_task_status
	*/
	struct navigation_task_status_ack : public asio_t {
		int traj_index_current_;
		int track_eno_;
		prc_t track_prc_;
		position_t pos_;
		uint64_t pos_time_stamp_;
		double pos_confidence_;
		upl_t upl_;
		int vehcile_control_mode_;
	};

	/*
	post_navigation_task
	post_navigation_pause_task
	post_navigation_resume_task
	post_navigation_cancle_task
	*/
	struct task_status_t : public asio_t {
		uint64_t task_id_;
		union {
			int status_;
			int optcode_;
		};
	};

	/*
	query_navigation_task_traj
	*/
	struct recv_nav_task_traj : public asio_t {
		std::vector<trail_t> trajs_;
	};

	struct var_item : public asio_t {
		int id_;
		uint32_t type_;
	};

	/*
	post_dbg_varls_request
	*/
	struct var_list : public asio_t {
		std::vector<struct var_item> items_;
	};

	/*
	post_query_mtver
	*/
	struct mtver_t : public asio_t {
		uint16_t major;
		uint16_t sub;
		uint16_t extended;
		char uname[128];
    	char vcu[32];
	};

	typedef enum __nsp__common_condition_checking_method {
		kConditionCheck_And = 0,
		kConditionCheck_Or = 1,
	} nsp__common_condition_checking_method;

	struct common_compare_write {
		int var_id_;
		nsp__common_condition_checking_method condition_checking_method_;
		common_data condition_;
		common_data writes_value_;
	};

	/*
	recv_common_status_report
	*/
	struct motion_report_t: public asio_t {
		uint64_t task_id_;
		int var_type_;
		int status_;
		std::string usrdat_;
	};

	/*
	recv_canio_msg
	*/
	struct canio_msg_t : public asio_t {
		uint32_t id;
		uint32_t var_type_;
		uint32_t command_;
		uint32_t middle_;
		uint32_t response_;
	};

	struct offline_operator_node_t {
		int32_t code_;
		uint64_t params_[10];
	};
	struct offline_task_node_t {
		upl_t dest_upl_;
		position_t dest_pos_;
		std::vector<trail_t> trails_;
		std::vector<offline_operator_node_t> opers_;
	};
	typedef std::vector<offline_task_node_t> mn_offline_task;

}

#endif /* !MOTION_NET_DEFINE_H */
