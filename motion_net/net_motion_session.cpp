#include "net_motion_session.h"
#include "proto_definion.h"
#include "proto_login.h"
#include <limits.h>
#include "net_manager.h"
#include "log.h"
#include "os_util.hpp"
#include "toolkit.h"
#include "proto_report_status.h"
#include "proto_task_status.h"
#include "proto_allocate_operation_task.h"
#include "proto_keep_alive.h"
#include "proto_offline_task.h"

#include "rapidxml_iterators.hpp"
#include "rapidxml_print.hpp"
#include "rapidxml_utils.hpp"

#define PROTO_HEAD_LEN 24
namespace mn {
	int net_motion_udp_session::init_session() {
		int retval = -1;

		if (initialized_) {
			return 0;
		}

		// create udp socket
		retval = create();
		if (retval < 0) {
			return -1;
		}

		// read xml config file
#if _WIN32
		std::string xml_path = nsp::os::get_module_directory<char>();
		xml_path += "\\etc\\motion_net.xml";
#else
		std::string xml_path = "/etc/agv/motion_net.xml";
#endif
		rapidxml::file<char> *xml_file = nullptr;
		rapidxml::xml_document<> doc;
		rapidxml::xml_node<>* root = nullptr;
		rapidxml::xml_node<>* node_map = nullptr;
		try {
			xml_file = new rapidxml::file<char>(xml_path.c_str());
			doc.parse<0>(xml_file->data());
			// read root node "motion_net "
			root = doc.first_node("motion_net");
			if (root) {
				for (node_map = root->first_node(); node_map; node_map = node_map->next_sibling()) {
					// read node "map"
					if (strcmp(node_map->name(), "map") != 0) {
						continue;
					}
					nsp::tcpip::endpoint tcp_ep;
					nsp::tcpip::endpoint udp_ep;
					std::string ipv4_str;
					uint16_t tcp_port = 0;
					uint16_t udp_port = 0;
					// read node's attribute
					for (auto anattr = node_map->first_attribute(); anattr != NULL; anattr = anattr->next_attribute()) {
						unsigned long ul = 0;
						char *endptr = NULL;
						if (strcmp(anattr->name(), "ipv4") == 0) {
							ipv4_str = anattr->value();
						} else if (strcmp(anattr->name(), "tcp_port") == 0) {
							errno = 0;
							ul = strtoul(anattr->value(), &endptr, 10);
							if (errno || ul < 1 || ul > 65535 || (endptr && *endptr != '\0')) {
								// error, do nothing
							} else {
								tcp_port = (uint16_t)ul;
							}
						} else if (strcmp(anattr->name(), "udp_port") == 0) {
							errno = 0;
							ul = strtoul(anattr->value(), &endptr, 10);
							if (errno || ul < 1 || ul > 65535 || (endptr && *endptr != '\0')) {
								// error, do nothing
							} else {
								udp_port = (uint16_t)ul;
							}
						}
					}
					if (ipv4_str.size() && tcp_port && udp_port) {
						// get all value, build endpoint & map_xml_udp_ep
						nsp::tcpip::endpoint::build(ipv4_str.c_str(), tcp_port, tcp_ep);
						nsp::tcpip::endpoint::build(ipv4_str.c_str(), udp_port, udp_ep);
						map_xml_udp_ep_[tcp_ep] = udp_ep;
					}
				}
			}
		}
		catch (...) {
			if (xml_file) {
				delete xml_file;
				xml_file = nullptr;
			}
		}

		initialized_ = 1;
		return retval;
	}

	void net_motion_udp_session::stop_keep_alive(uint32_t robot_id) {
		std::lock_guard<decltype(mutex_locker_)> guard(mutex_locker_);
		uint32_t old_pktid;
		auto iter_robotid = map_robotid_pktid_.find(robot_id);
		if (iter_robotid != map_robotid_pktid_.end()) {
			old_pktid = iter_robotid->second;
			map_pktid_robotid_.erase(old_pktid);
			map_robotid_pktid_.erase(iter_robotid);
		}
	}

	int net_motion_udp_session::send_alive_packet(uint32_t robot_id, nsp::tcpip::endpoint &mt_ep, nsp::tcpip::endpoint &mn_ep) {
		if (!initialized_) {
			// is not initialized return error.
			return -1;
		}

		std::lock_guard<decltype(mutex_locker_)> guard(mutex_locker_);

		nsp::proto::proto_keep_alive_t pkt_keep_alive;
		uint32_t new_pktid = ++pktid_;
		pkt_keep_alive.head_.type_ = PKTTYPE_KEEPALIVE_UDP;
		pkt_keep_alive.head_.id_ = new_pktid;
		posix__strcpy(pkt_keep_alive.address_, cchof(pkt_keep_alive.address_), mt_ep.ipv4());
		pkt_keep_alive.port_ = (uint16_t)mt_ep.port();
		pkt_keep_alive.calc_size();

		int retval = -1;
		uint32_t old_pktid;
		auto iter_robotid = map_robotid_pktid_.find(robot_id);
		if (iter_robotid != map_robotid_pktid_.end()) {
			// old packet is ignored
			old_pktid = iter_robotid->second;
			map_pktid_robotid_.erase(old_pktid);
			iter_robotid->second = new_pktid;
		} else {
			map_robotid_pktid_[robot_id] = new_pktid;
		}
		map_pktid_robotid_[new_pktid] = robot_id;

		auto iter_xml = map_xml_udp_ep_.find(mn_ep);
		if (iter_xml == map_xml_udp_ep_.end()) {
			// use parameter
			retval = obudp::sendto(pkt_keep_alive.head_.size_, [&](void*buffer, int cb)->int {
				return (pkt_keep_alive.serialize((unsigned char*)buffer) < 0) ? -1 : 0;
			}, mn_ep);
		} else {
			// the ip address and udp port is specified in the file motion_net.xml
			retval = obudp::sendto(pkt_keep_alive.head_.size_, [&](void*buffer, int cb)->int {
				return (pkt_keep_alive.serialize((unsigned char*)buffer) < 0) ? -1 : 0;
			}, iter_xml->second);
		}

		if (retval < 0) {
			map_robotid_pktid_.erase(robot_id);
			map_pktid_robotid_.erase(new_pktid);
		}
		return retval;
	}

	int net_motion_udp_session::recv_keep_alive(const unsigned char *buffer, int &cb, const nsp::tcpip::endpoint &r_ep) {
		nsp::proto::proto_keep_alive_t keep_alive_ack;
		if (keep_alive_ack.build(buffer, cb) < 0) {
			return -1;
		}

		uint32_t robot_id_ = 0;
		uint32_t pktid = keep_alive_ack.head_.id_;
		{
			std::lock_guard<decltype(mutex_locker_) > guard(mutex_locker_);
			auto iter_pktid = map_pktid_robotid_.find(pktid);
			if (iter_pktid != map_pktid_robotid_.end()) {
				robot_id_ = iter_pktid->second;
				auto iter_robotid = map_robotid_pktid_.find(robot_id_);
				if (iter_robotid != map_robotid_pktid_.end()) {
					map_robotid_pktid_.erase(iter_robotid);
				}
				map_pktid_robotid_.erase(iter_pktid);
			}
		} 
		if (robot_id_) {
			mnlog_info << "recv a udp keepalive packet from " << r_ep.to_string();
			nsp::toolkit::singleton<net_manager>::instance()->set_alive_session(robot_id_);
		}
		return 0;
	}

	void net_motion_udp_session::on_recvdata(const std::string &data, const nsp::tcpip::endpoint &r_ep) {
		const unsigned char *buffer = (const unsigned char *)data.data();
		int cb = data.size();
		auto len = nsp::proto::proto_head::type_length();
		if (cb < len) {
			return;
		}

		uint32_t type = *(uint32_t *)(buffer + sizeof(uint32_t));
		switch (type) {
		case PKTTYPE_KEEPALIVE_UDP_ACK:
			recv_keep_alive(buffer, cb, r_ep);
			break;
		}
	}

	net_motion_session::net_motion_session(int robot_id) : 
		nsp::tcpip::tcp_application_client<nsp::proto::nspdef::protocol>(), pool_(robot_id) {
			robot_id_ = robot_id;
	}

	net_motion_session::net_motion_session( HTCPLINK lnk, int robot_id ) : 
		nsp::tcpip::tcp_application_client<nsp::proto::nspdef::protocol>( lnk ),pool_(robot_id) {
	}

	net_motion_session::~net_motion_session() {
	}

	int net_motion_session::check_connection_status() const {
		if ( this->net_status_ < kNetworkStatus_Established ) {
			return -ECONNABORTED;
		}
		return 0;
	}

	void net_motion_session::notify_it(const void *data, enum notift_type_t type) {
		notify_t notify_to;

		{
			std::lock_guard<decltype(lock_notify_change_)> guard(lock_notify_change_);
			notify_to = notify_;
		}

		if ( notify_to ) {
			notify_to( robot_id_, data, type );
		}
	}

	void net_motion_session::on_disconnected( const HTCPLINK previous ) {
		// reset packet time to zero when session disconnected.
		tcp_pkt_timed_ = 0;

		if (kNetworkStatus_Closed != net_status_){
			net_status_ = kNetworkStatus_Closed;
			net_status_t stat = net_status_;
			notify_it( ( const void* ) &stat, kNET_STATUS );
		}
		this->pool_.cleanall();

		mnlog_info << "session disconnected, target:" << this->ep_.to_string();
	}

	void net_motion_session::per_close() {
		net_status_ = kNetworkStatus_Closed;
		pool_.cleanall();
	}

	void net_motion_session::get_notify(notify_t &func) const {
		std::lock_guard<decltype(lock_notify_change_)> guard(lock_notify_change_);
		func = notify_;
	}

	void net_motion_session::set_udp_alive() {
		udp_pkt_timed_ = nsp::os::clock_gettime();
	}

	void net_motion_session::register_notify_callback( const notify_t& to_notify ) {
		std::lock_guard<decltype(lock_notify_change_)> guard(lock_notify_change_);
		notify_ = to_notify;
	}

	int net_motion_session::recv_sync_connect_ack(const unsigned char *buffer, int &cb ) {
		std::shared_ptr<nsp::proto::proto_key> pkt;
		if (this->build<nsp::proto::proto_key>(buffer, cb, pkt) < 0) {
			return -1;
		}
		encrypt_key_ = pkt->key_;
		this->net_status_ = kNetworkStatus_Ready;
		mnlog_info << "session received key, target: " << ep_.to_string();
		// when status is ready, try to login ... 
		try_login();
		return 0;
	}

	int net_motion_session::recv_login_ack( const unsigned char *buffer, int &cb) {
		std::shared_ptr<nsp::proto::proto_head> pkt;
		if (this->build<nsp::proto::proto_head>(buffer, cb, pkt) < 0) {
			return -1;
		}
		
		return pool_.async_complete( pkt->id_, pkt->type_, ( const char* ) &pkt->err_ );
	}

	int net_motion_session::recv_navigation_status_ack(const unsigned char *buffer, int &cb ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}
		
		std::shared_ptr<nsp::proto::proto_query_navigation_task_status_ack> pkt;
		if (build<nsp::proto::proto_query_navigation_task_status_ack>(buffer, cb, pkt) < 0) {
			return -1;
		}

		navigation_task_status_ack ack;
		ack.traj_index_current_ = pkt->traj_index_current_;
		ack.track_eno_ = pkt->track_eno_;
		ack.track_prc_ = pkt->track_prc_;
		ack.pos_.x_ = pkt->pos_.x_;
		ack.pos_.y_ = pkt->pos_.y_;
		ack.pos_.w_ = pkt->pos_.y_;
		ack.pos_time_stamp_ = pkt->pos_time_stamp_;
		ack.pos_confidence_ = pkt->pos_confidence_;
		ack.upl_.edge_id_ = pkt->upl_.edge_id_;
		ack.upl_.percentage_ = pkt->upl_.percentage_;
		ack.upl_.wop_id_ = pkt->upl_.wop_id_;
		ack.vehcile_control_mode_ = pkt->vehcile_control_mode_;

		return pool_.async_complete( pkt->head.id_, pkt->head.type_, ( const char* ) &ack );
	}

	int net_motion_session::recv_navigation_traj_ack(const unsigned char *pos, int &cb) {
		std::shared_ptr<nsp::proto::proto_navigation_task_traj_ack_t> pkt;
		if (build<nsp::proto::proto_navigation_task_traj_ack_t>(pos, cb, pkt) < 0) {
			
		}

		recv_nav_task_traj asio;
		asio.err_ = pkt->head_.err_;

		if ( asio.err_ >= 0 && pkt->trajs_.size() > 0 ) {
			for ( auto &iter : pkt->trajs_ ) {
				trail_t traj;
				traj.edge_id_ = iter.edge_id_;
				traj.wop_id_ = iter.wop_id_;
				asio.trajs_.push_back( traj );
			}
		}

		return pool_.async_complete( pkt->head_.id_, pkt->head_.type_, ( const char* ) &asio );
	}

	int net_motion_session::recv_navigation_operation_ack( const unsigned char *buffer, int &cb) {
		std::shared_ptr<nsp::proto::proto_control_navi_task_status_ack_t> pkt;
		if (this->build<nsp::proto::proto_control_navi_task_status_ack_t>(buffer, cb, pkt) < 0 ) {
			return -1;
		}
		
		task_status_t status;
		status.err_ = pkt->head_.err_;
		status.status_ = pkt->status_;
		status.task_id_ = pkt->task_id_;

		return pool_.async_complete( pkt->head_.id_, pkt->head_.type_, ( const char* ) &status );
	}

	int net_motion_session::recv_comm_read_requst_asio(const unsigned char *pos, int &cb) {
		std::shared_ptr<nsp::proto::proto_vec_commom_ack> pkt;
		if (build<nsp::proto::proto_vec_commom_ack>(pos, cb, pkt) < 0) {
			return -1;
		}

		common_data data;
		data.err_ = pkt->head.err_;
		if ( pkt->head.err_ >= 0 && pkt->vec_common_ack.size() > 0 ) {
			for ( auto &iter : pkt->vec_common_ack ) {
				struct common_data_item item;
				item.varid = iter.var_id_;
				item.vartype = iter.var_type_;
				item.offset = iter.offset_;
				item.data = iter.value_;
				data.items.push_back( item );
			}
		}

		return pool_.async_complete( pkt->head.id_, pkt->head.type_, ( const char* ) &data );
	}

	int net_motion_session::recv_common_status_report(const unsigned char *pos, int &cb) {
		std::shared_ptr<nsp::proto::proto_common_report_t> pkt;
		if (build<nsp::proto::proto_common_report_t>(pos, cb, pkt) < 0) {
			return -1;
		}
		
		motion_report_t report;
		report.err_ = pkt->head.err_;
		report.task_id_ = pkt->task_id_;
		report.status_ = pkt->status_;
		report.usrdat_ = pkt->usrdat_;
		report.var_type_ = pkt->var_type_;

		if ( ( type_ & kControlorType_Dispatcher ) || ( type_ & kControlorType_LayoutDesigner ) ) {
			notify_it( ( const void* ) &report, kTASK_STATUS_REPORT );
		}
		return 0;
	}

	int net_motion_session::recv_canio_msg(const unsigned char *pos, int &cb ) {
		std::shared_ptr<nsp::proto::proto_canio_msg_t> pkt;
		if (build<nsp::proto::proto_canio_msg_t>(pos, cb, pkt) < 0) {
			return -1;
		}
		
		canio_msg_t msg;
		msg.err_ = pkt->head_.err_;
		msg.id = pkt->id_;
		msg.var_type_ = pkt->var_type_;
		msg.command_ = pkt->command_;
		msg.middle_ = pkt->middle_;
		msg.response_ = pkt->response_;

		if ( ( type_ & kControlorType_Dispatcher ) || ( type_ & kControlorType_LayoutDesigner ) ) {
			notify_it( ( const void* ) &msg, kCANIO_MSG );
		}

		return 0;
	}

	int net_motion_session::recv_varls_ack(const unsigned char *pos, int &cb) {
		std::shared_ptr<nsp::proto::proto_var_report_items_t> pkt;
		if (build<nsp::proto::proto_var_report_items_t>(pos, cb, pkt) < 0 ) {
			return -1;
		}

		var_list asio;
		asio.err_ = pkt->head.err_;
		for ( size_t i = 0; i < pkt->items.size(); i++ ) {
			struct var_item item;
			item.id_ = pkt->items[i].id_;
			item.type_ = pkt->items[i].type_;
			asio.items_.push_back( item );
		}

		return pool_.async_complete( pkt->head.id_, pkt->head.type_, ( const char* ) &asio );
	}

	int net_motion_session::recv_allocate_operation_task_ack(const unsigned char *pos, int &cb ){
		std::shared_ptr<nsp::proto::proto_allocate_operation_task_ack_t> pkt;
		if (build<nsp::proto::proto_allocate_operation_task_ack_t>(pos, cb, pkt) < 0) {
			return -1;
		}

		task_status_t asio;
		asio.err_ = pkt->head_.err_;
		asio.task_id_ = pkt->task_id_;

		return pool_.async_complete( pkt->head_.id_, pkt->head_.type_, ( const char* ) &asio );
	}

	int net_motion_session::recv_operation_task_status_ack(const unsigned char *pos, int &cb) {
		std::shared_ptr<nsp::proto::proto_operator_task_action_ack_t> pkt;
		if (build<nsp::proto::proto_operator_task_action_ack_t>(pos, cb, pkt) < 0) {
			return -1;
		}
		
		task_status_t asio;
		asio.err_ = pkt->head_.err_;
		asio.task_id_ = pkt->task_id_;
		asio.optcode_ = pkt->optcode_;

		return pool_.async_complete( pkt->head_.id_, pkt->head_.type_, ( const char* ) &asio );
	}

	int net_motion_session::recv_normal_complete( const unsigned char *pos, int &cb ) {
		std::shared_ptr<nsp::proto::proto_head> pkt;
		if (build<nsp::proto::proto_head>(pos, cb, pkt) < 0 ){
			return -1;
		}
		
		mn::asio_t asio;
		asio.err_ = pkt->err_;

		return pool_.async_complete( pkt->id_, pkt->type_, ( const char* ) &asio );
	}

	int net_motion_session::recv_periodic_report( const unsigned char *pos, int &cb ) {
		std::shared_ptr<nsp::proto::proto_periodic_report<nsp::proto::proto_periodic_data>> pkt;
		if (build<nsp::proto::proto_periodic_report<nsp::proto::proto_periodic_data>>(pos, cb, pkt) < 0) {
			return -1;
		}

		struct periodic_data msg;
		msg.err_ = pkt->head.err_;
		msg.timestamp = pkt->timestamp;
		for (auto &iter : pkt->parameters) {
			periodic_data_item item;
			item.varid = iter.var_id_;
			item.vartype = iter.var_type_;
			item.offset = iter.offset_;
			item.interval = iter.event_cycle_;
			item.data = iter.data_;
			msg.items.push_back( item );
		}

		notify_it( ( const void* )&msg, kPERIODIC_REPORT );
		return 0;
	}

	int net_motion_session::recv_mtver(const unsigned char *pos, int &cb ) {
		std::shared_ptr<nsp::proto::proto_get_mtver_ack> pkt;
		if (build<nsp::proto::proto_get_mtver_ack>(pos, cb, pkt) < 0) {
			return -1;
		}

		struct mtver_t mtver;
		mtver.err_ = pkt->head.err_;

		if (mtver.err_ >= 0 ) {
			mtver.major = pkt->major;
			mtver.sub = pkt->sub;
			mtver.extended = pkt->extended;

			if (pkt->uname.size() < sizeof(mtver.uname)) {
				posix__strcpy(mtver.uname, cchof(mtver.uname), pkt->uname.c_str());
			}

			if (pkt->vcu.size() < sizeof(mtver.vcu)) {
				posix__strcpy(mtver.vcu, cchof(mtver.vcu), pkt->vcu.c_str());
			}
		}
		return pool_.async_complete( pkt->head.id_, pkt->head.type_, ( const char* ) &mtver );
	}

	void net_motion_session::recv_dispatch(const unsigned char *buffer, int &cb ) {
		auto len = nsp::proto::proto_head::type_length();
		if (cb < len) {
			return;
		}
		
		// this is type
		uint32_t type = *(uint32_t *)(buffer + sizeof(uint32_t));
		
		const unsigned char *pos = (const unsigned char  *)buffer; 
		switch ( type ) {
			case PKTTYPE_KEEPALIVE_TCP_ACK:
				break;
			case PKTTYPE_PRE_LOGIN:
				recv_sync_connect_ack( pos, cb );
				break;
			case PKTTYPE_LOGIN_ROBOT_ACK:
				recv_login_ack( pos, cb );
				break;
			case PKTTYPE_ALLOC_NAVIGATION_TASK_ACK:
			case PKTTYPE_CANCEL_NAVIGATION_TASK_ACK:
			case PKTTYPE_PAUSE_NAVIGATION_TASK_ACK:
			case PKTTYPE_RESUME_NAVIGATION_TASK_ACK:
			case PKTTYPE_ALLOC_OFFLINE_TASK_ACK:
			case PKTTYPE_CANCEL_OFFLINE_TASK_ACK:
			case PKTTYPE_OFFLINE_NEXT_STEP_ACK:
				recv_navigation_operation_ack(pos, cb);
				break;
			case PKTTYPE_QUERY_NAVIGATION_TASK_STATUS_ACK:
				recv_navigation_status_ack(pos, cb);
				break;
			case PKTTYPE_QUERY_NAVIGATION_TASK_TRAJ_ACK:
				recv_navigation_traj_ack(pos, cb);
				break;
			case PKTTYPE_COMMON_READ_BYID_ACK:
				recv_comm_read_requst_asio( pos,cb );
				break;
			case PKTTYPE_COMMON_STATUS_REPORT:
				recv_common_status_report( pos, cb);
				break;
			case PKTTYPE_DBG_VARLS_ACK:
				recv_varls_ack( pos, cb );
				break;
			case PKTTYPE_CANIO_MSG:
				recv_canio_msg( pos, cb );
				break;
			case PKTTYPE_ALLOC_OPERATION_TASK_ACK:
				recv_allocate_operation_task_ack( pos, cb);
				break;
			case PKTTYPE_CANCEL_OPERATION_TASK_ACK:
			case PKTTYPE_RESUME_OPERATION_TASK_ACK:
			case PKTTYPE_PAUSE_OPERATION_TASK_ACK:
				recv_operation_task_status_ack( pos, cb );
				break;
			case PKTTYPE_COMMON_WRITE_BYID_ACK:
			case PKTTYPE_COMMON_COMPARE_WRITE_ACK:
			case PKTTYPE_QUERY_REPORT_STATUS_ACK:
			case PKTTYPE_ADDITIONAL_NAVIGATION_TRAJECTORY_ACK:
			case PKTTYPE_QUERY_OPERATION_PARAMETER_ACK:
			case PKTTYPE_DBG_CLEAR_FAULT_ACK:
			case PKTTYPE_INITIACTIVE_COMMON_READ_ACK:
			case PKTTYPE_INITIACTIVE_COMMON_READ_CANCLE_ACK:
				recv_normal_complete( pos, cb );
				break;
			case PKTTYPE_INITIACTIVE_READ_ACK:
				recv_periodic_report( pos, cb );
				break;
			case PKTTYPE_DBG_GET_MTVER_ACK:
				recv_mtver( pos, cb );
				break;
			default:
				mnlog_error << "unknown packet type received. " << type;
				break;
		}
	}

	void net_motion_session::on_recvdata( const std::string &pkt ) {
		tcp_pkt_timed_ = nsp::os::clock_gettime();
		nsp::toolkit::singleton<net_manager>::instance()->schedule_async_receive(pkt,
			std::bind(&net_motion_session::recv_dispatch, this, std::placeholders::_1, std::placeholders::_2));
	}

	void net_motion_session::on_connected() {
		net_status_t exp = kNetworkStatus_Connecting;
		tcp_pkt_timed_ = nsp::os::clock_gettime();
		if (net_status_.compare_exchange_strong(exp, kNetworkStatus_Connected)) {
			mnlog_info << "session successful connected to target host " << remote_.to_string();
		} else {
			mnlog_info << "session successful connected to target host " << remote_.to_string() << ", status is " << net_status_;
		}
	}

	int net_motion_session::try_connect() {
		if ( !ep_ ) {
			return -1;
		}

		do {
			// not inited/has been closed
			net_status_t exp = kNetworkStatus_Closed;
			if ( net_status_.compare_exchange_strong( exp, kNetworkStatus_Actived ) ) {
				if ( this->create() < 0 ) {
					mnlog_error << "failed to call ns API create.this link will be destory,robot:" << this->robot_id_;
					break;
				}
			}

			// inited but not established
			exp = kNetworkStatus_Actived;
			if ( net_status_.compare_exchange_strong( exp, kNetworkStatus_Connecting ) ) {
				mnlog_info << "try connect to host " << ep_.to_string();
				tcp_pkt_timed_ = nsp::os::clock_gettime(); // begin of timedout check
				if (connect2(ep_) < 0) {
					close();
					mnlog_error << "failed to call ns API connect2.this link will be destory,robot:" << this->robot_id_;
					break;
				}
			}

			return 0;
		}while ( 0 );
		
		return -1;
	}

	void net_motion_session::login_notify(uint32_t robot_id, const void *data) {
		if (!data) {
			return;
		}

		int err = *(int *)data;
		if (err < 0) {
			mnlog_error << "server decline login request,error=" << err;
			nsp::toolkit::singleton<net_manager>::instance()->set_status(robot_id_, kNetworkStatus_Closed, err);
		}
		else {
			mnlog_info << "session successful login to host " << ep_.to_string();
			nsp::toolkit::singleton<net_manager>::instance()->set_status(robot_id_, kNetworkStatus_Established, 0);
			notify_it((const void*)&net_status_, kNET_STATUS);
		}
	}

	int net_motion_session::try_login() {
		net_status_t exp = kNetworkStatus_Ready;
		if ( !this->net_status_.compare_exchange_strong( exp, kNetworkStatus_Landing ) ) {
			close();
			mnlog_error << "network status must be kNetworkStatus_Ready befor post login request. ";
			return -1;
		}

		//生成原文
		unsigned char original_buffer[32];
		for ( int i = 0; i < 32; i++ ) {
			original_buffer[i] = nsp::toolkit::random( 10, UCHAR_MAX );
		}

		std::string original_str( ( const char* ) original_buffer, 32 );
		std::string encrypt_str;

		if ( nsp::toolkit::encrypt( original_str, encrypt_key_, encrypt_str ) < 0 ) {
			mnlog_error << " failed post login request to service. access declined.";
			close();
			return -1;
		}

		unsigned char md5_trans[16];
		nsp::toolkit::md5( encrypt_str.c_str(), encrypt_str.length(), md5_trans );
		std::string md5_str( ( const char* ) md5_trans, 16 );

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_login_robot pkt_login( PKTTYPE_LOGIN_ROBOT, pktid );
		pkt_login.cct_ = type_;
		pkt_login.acquire_access_ = 0;
		pkt_login.original_buffer_ = original_str;
		pkt_login.encrypted_buffer_ = md5_str;
		pkt_login.calc_size();

		login_error_ = 0;
		int retval = pool_.queue_packet(pktid, std::make_shared<asio_partnet>(this->robot_id_,
			std::bind(&net_motion_session::login_notify, this, std::placeholders::_1, std::placeholders::_2)
			), [&]() ->int {
			return psend( &pkt_login );
		} );

		if (retval < 0) {
			close();
			return -1;
		}

		return 0;
	}

	int net_motion_session::post_navigation_task( uint64_t task_id, const upl_t &dest_upl, const position_t &dest_pos, int is_traj_whole,
		const std::vector<trail_t> &vec_trail, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_navigation_task_t packet( PKTTYPE_ALLOC_NAVIGATION_TASK, pktid );

		packet.set_task_id_ = task_id;
		packet.is_traj_whole_ = is_traj_whole;
		packet.dest_upl_.edge_id_ = dest_upl.edge_id_;
		packet.dest_upl_.percentage_ = dest_upl.percentage_;
		packet.dest_upl_.angle_ = dest_upl.angle_;

		packet.dest_pos_.x_ = dest_pos.x_;
		packet.dest_pos_.y_ = dest_pos.y_;
		packet.dest_pos_.w_ = dest_pos.w_;

		for ( auto &iter : vec_trail ) {
			nsp::proto::trail_t trail;
			trail.edge_id_ = iter.edge_id_;
			trail.wop_id_ = iter.wop_id_;
			packet.vec_trail_t.push_back( trail );
		}
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_navigation_cancle_task( uint64_t task_id, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_task_status_t packet( PKTTYPE_CANCEL_NAVIGATION_TASK, pktid );
		packet.task_id_ = task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_navigation_pause_task( uint64_t task_id, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_task_status_t packet( PKTTYPE_PAUSE_NAVIGATION_TASK, pktid );
		packet.task_id_ = task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_navigation_resume_task( uint64_t task_id, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_task_status_t packet( PKTTYPE_RESUME_NAVIGATION_TASK, pktid );
		packet.task_id_ = task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::query_task_opera_parm_order( const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_head packet( PKTTYPE_QUERY_OPERATION_PARAMETER, pktid, PROTO_HEAD_LEN );

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::query_navigation_task_detail_status( uint64_t task_id, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_control_navigation_task packet( PKTTYPE_QUERY_NAVIGATION_TASK_STATUS, pktid );
		packet.task_id_ = task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::query_navigation_task_traj( const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_head packet( PKTTYPE_QUERY_NAVIGATION_TASK_TRAJ, pktid );

		packet.size_ = packet.length();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_add_navigation_task_traj(
		uint64_t task_id, uint32_t index, const std::vector<trail_t> &vec_trail, int is_traj_whole, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_add_navigation_task_traj_ack_t packet( PKTTYPE_ADDITIONAL_NAVIGATION_TRAJECTORY, pktid );

		packet.task_id_ = task_id;
		packet.index_ = index;
		packet.is_traj_whole_ = is_traj_whole;
		for ( const auto &iter : vec_trail ) {
			nsp::proto::navi_task_add_trail_t trail;
			trail.edge_id_ = iter.edge_id_;
			trail.wop_id_ = iter.wop_id_;
			packet.cnt_trajs_.push_back( trail );
		}
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_common_read_request_by_id(
		const struct common_title &read_data, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_vec_comm_t packet( PKTTYPE_COMMON_READ_BYID, pktid );

		for ( const auto &iter : read_data.items ) {
			nsp::proto::common_t common_par;
			common_par.var_id_ = iter.varid;
			common_par.var_type_ = iter.vartype;
			common_par.offset_ = iter.offset;
			common_par.length_ = iter.length;
			packet.vec_common_par.push_back( common_par );
		}
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_common_write_request_by_id( const struct common_data &write_data, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_vec_commom_ack_t packet( PKTTYPE_COMMON_WRITE_BYID, pktid );

		for ( const auto &iter : write_data.items ) {
			nsp::proto::proto_common_ack item;
			item.var_id_ = iter.varid;
			item.var_type_ = iter.vartype;
			item.offset_ = iter.offset;
			item.value_ = iter.data;
			packet.vec_common_ack.push_back( item );
		}
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_common_compare_write_request( 
		const struct common_compare_write& compare_write, const std::shared_ptr<asio_partnet>&asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_common_compare_write_t packet( PKTTYPE_COMMON_COMPARE_WRITE, pktid );

		packet.var_id_ = compare_write.var_id_;
		packet.nsp__controlor_type_t_ = compare_write.condition_checking_method_;

		for ( auto & iter : compare_write.condition_.items ) {
			nsp::proto::proto_common_ack item;
			item.var_id_ = iter.varid;
			item.var_type_ = iter.vartype;
			item.offset_ = iter.offset;
			item.value_ = iter.data;
			packet.vct_condition_.push_back( item );
		}

		for ( auto&iter : compare_write.writes_value_.items ) {
			nsp::proto::proto_common_ack item;
			item.var_id_ = iter.varid;
			item.var_type_ = iter.vartype;
			item.offset_ = iter.offset;
			item.value_ = iter.data;
			packet.vct_write_.push_back( item );
		}
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_report_status_order( 
		uint64_t task_id, uint32_t vartype_, uint32_t status, const std::string &usr_msg, const std::shared_ptr<asio_partnet>&asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_report_status_t packet( PKTTYPE_QUERY_REPORT_STATUS, pktid );

		packet.task_id_ = task_id;
		packet.vartype_ = vartype_;
		packet.status_ = status;
		packet.usr_msg_ = usr_msg;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_clear_fault_request( const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_head packet( PKTTYPE_DBG_CLEAR_FAULT, pktid, PROTO_HEAD_LEN );

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_dbg_varls_request( const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_head packet( PKTTYPE_DBG_VARLS, pktid, PROTO_HEAD_LEN );

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_dbg_segmentfault(const std::shared_ptr<asio_partnet> &asio){
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_head packet( PKTTYPE_DBG_SIGSEGV, pktid, PROTO_HEAD_LEN );

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_query_mtver( const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_head packet( PKTTYPE_DBG_GET_MTVER, pktid, PROTO_HEAD_LEN );

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_allocate_operation_task( 
		uint64_t task_id, int32_t task_code, uint64_t task_params[10], const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_allocate_operation_task_t packet( PKTTYPE_ALLOC_OPERATION_TASK, pktid );
		packet.task_id_ = task_id;
		packet.code_ = task_code;
		packet.params0_ = task_params[0];
		packet.params1_ = task_params[1];
		packet.params2_ = task_params[2];
		packet.params3_ = task_params[3];
		packet.params4_ = task_params[4];
		packet.params5_ = task_params[5];
		packet.params6_ = task_params[6];
		packet.params7_ = task_params[7];
		packet.params8_ = task_params[8];
		packet.params9_ = task_params[9];
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_operation_cancle_task( uint64_t operator_task_id, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_operator_task_action_t packet( PKTTYPE_CANCEL_OPERATION_TASK, pktid );
		packet.task_id_ = operator_task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_operation_pause_task( uint64_t operator_task_id, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_operator_task_action_t packet( PKTTYPE_PAUSE_OPERATION_TASK, pktid );
		packet.task_id_ = operator_task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::post_operation_resume_task( uint64_t operator_task_id, const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_operator_task_action_t packet( PKTTYPE_RESUME_OPERATION_TASK, pktid );
		packet.task_id_ = operator_task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	int net_motion_session::registry_periodic_report(const struct periodic_title &read_periodic_data, const std::shared_ptr<asio_partnet> &asio) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}

		if (0 == read_periodic_data.items.size()) {
			return -EINVAL;
		}
		
		uint32_t pktid = pool_.get_periodic_pktid();
		nsp::proto::proto_periodic_report<nsp::proto::proto_periodic_title> packet(PKTTYPE_INITIACTIVE_COMMON_READ, pktid);

		packet.timestamp = nsp::os::clock_gettime();
		for (auto &iter : read_periodic_data.items) {
			nsp::proto::proto_periodic_title item;
			item.var_id_ = iter.varid;
			item.event_cycle_ = iter.interval;
			item.var_type_ = iter.vartype;
			item.offset_ = iter.offset;
			item.length_ = iter.length;
			packet.parameters.push_back(item);
		}

		packet.calc_size();

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}
	
	int net_motion_session::unregistry_periodic_report( const std::shared_ptr<asio_partnet> &asio ) {
		int err = check_connection_status();
		if ( err < 0 ) {
			return err;
		}
		
		pool_.periodic_pktid_cancel();

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_head packet( PKTTYPE_INITIACTIVE_COMMON_READ_CANCLE, pktid, PROTO_HEAD_LEN );

		return pool_.queue_packet(pktid, asio,  [&] () ->int {
			return psend( &packet );
		} );
	}

	// 周期性检查链接状态，并投递心跳包
	int net_motion_session::keepalive() {
		if (!enable_keepalive_) {
			return 0;
		}

		do {
			// this link has been established, then send the keepalive package to target host
			if ( kNetworkStatus_Established == net_status_ ) {
				nsp::proto::proto_head pkt_keepalive( PKTTYPE_KEEPALIVE_TCP, 0, PROTO_HEAD_LEN );
				pkt_keepalive.id_ = pool_.get_pktid();
				if ( psend( &pkt_keepalive ) < 0 ) {
					mnlog_error << "keepalive send error";
					close();
					break;
				}
			}

			if (net_status_ < kNetworkStatus_Connected) {
				nsp::tcpip::endpoint mn_ep;
				if (kNetworkStatus_Established == net_status_) {
					// connection is Established, use local ip:port
					mn_ep = local_;
				}
				else {
					// connection is not Established, use 0.0.0.0:0
					nsp::tcpip::endpoint::build("0.0.0.0", 0, mn_ep);
				}
				if (!udp_need_send) {
					mnlog_info << "start to send udp keepalive packet to " << ep_.to_string() << ", status is " << net_status_;
					udp_need_send = 1;
				}				// send udp keepalive package to target host
				nsp::toolkit::singleton<net_manager>::instance()->send_alive_packet(robot_id_, mn_ep, ep_);
			} else {
				if (udp_need_send) {
					mnlog_info << "stop to send udp keepalive packet to " << ep_.to_string() << ", status is " << net_status_;
					udp_need_send = 0;
				}
				udp_pkt_timed_ = 0;
			}
			uint64_t now;
			now = nsp::os::clock_gettime();
			if (udp_pkt_timed_ && (now - udp_pkt_timed_) > 20000000) {
				// didn't recv ack packet in 2 seconds, don't need retry connect/login.
				return 0;
			} else if (0 == udp_pkt_timed_ && udp_need_send) {
				// ack packet didn't recv, don't need retry connect/login.
				return 0;
			}

			// try to creat and reconnect to target host, when current status is closed.
			//		if this operation failed. status will be restore to closed.
			if (kNetworkStatus_Closed == net_status_ ) {
				mnlog_info << "retry connect to host " << ep_.to_string();
				if (try_connect() < 0) {
					break;
				}
			}

			// if key-pair is ready, try to login/relogin to target host next
			if (kNetworkStatus_Ready == net_status_) {
				mnlog_info << "retry login to host " << ep_.to_string();
				if (try_login() < 0) {
					break;
				}
			}
			return 0;
		}while( 0 );

		return -1;
	}

	const nsp::tcpip::endpoint &net_motion_session::getep() const {
		return ep_;
	}

	void net_motion_session::setep( const nsp::tcpip::endpoint &ep ) {
		ep_ = ep;
	}

	const nsp__controlor_type_t net_motion_session::gettype() const {
		return type_;
	}

	void net_motion_session::settype( const nsp__controlor_type_t &type ) {
		this->type_ = type;
	}

	void net_motion_session::enable_keepalive() {
		this->enable_keepalive_ = posix__true;
	}

	int net_motion_session::check_timeout(uint64_t tick) {
		static const uint64_t LAST_PACKET_CHECK_TIMEOUT = 50000000; // in 100ns
		uint64_t last_packet_time = tcp_pkt_timed_;
		if (last_packet_time > 0) {
			uint64_t now = nsp::os::clock_gettime();
			if (now > last_packet_time) {
				uint64_t dur = now - last_packet_time;
				// if no any packet transfer during 5 seconds, this link may be dead.
				if (dur > LAST_PACKET_CHECK_TIMEOUT && kNetworkStatus_Closed != net_status_) {
					mnlog_warn << "no any pakcet transfer during " << dur << " counts,this link " \
						<< lnk_ << " maybe dead. local=" << local_.to_string() << " remote=" << remote_.to_string() \
						<< ", status is " << net_status_ << ", now:" << now;
					this->close();
					return -1;
				}
			}
		}
		pool_.check_timeout(tick);
		return 0;
	}

	void net_motion_session::get_status(net_status_t &net_stat, int &login_err) const {
		net_stat = net_status_;
		login_err = login_error_;
	}

	void net_motion_session::set_status(net_status_t net_stat, int login_err) {
		if (kNetworkStatus_Closed == net_stat) {
			close();
		}
		this->net_status_ = net_stat;
		this->login_error_ = login_err;
	}

	int net_motion_session::post_offline_task(uint64_t task_id, const mn_offline_task &task, const std::shared_ptr<asio_partnet> &asio) {
		int err = check_connection_status();
		if (err < 0) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_offline_task packet(PKTTYPE_ALLOC_OFFLINE_TASK, pktid);
		packet.task_id_ = task_id;

		for (const auto &iter_node : task) {
			nsp::proto::proto_offline_node_t task_node;
			task_node.dest_upl_.edge_id_ = iter_node.dest_upl_.edge_id_;
			task_node.dest_upl_.percentage_ = iter_node.dest_upl_.percentage_;
			task_node.dest_upl_.angle_ = iter_node.dest_upl_.angle_;
			task_node.dest_pos_.x_ = iter_node.dest_pos_.x_;
			task_node.dest_pos_.y_ = iter_node.dest_pos_.y_;
			task_node.dest_pos_.w_ = iter_node.dest_pos_.w_;
			for (const auto &iter_traj : iter_node.trails_) {
				nsp::proto::trail_t task_trail;
				task_trail.edge_id_ = iter_traj.edge_id_;
				task_trail.wop_id_ = iter_traj.wop_id_;
				task_node.trails_.push_back(std::move(task_trail));
			}
			for (const auto &iter_oper : iter_node.opers_) {
				nsp::proto::proto_offline_operation_t task_oper;
				task_oper.code_ = iter_oper.code_;
				task_oper.params0_ = iter_oper.params_[0];
				task_oper.params1_ = iter_oper.params_[1];
				task_oper.params2_ = iter_oper.params_[2];
				task_oper.params3_ = iter_oper.params_[3];
				task_oper.params4_ = iter_oper.params_[4];
				task_oper.params5_ = iter_oper.params_[5];
				task_oper.params6_ = iter_oper.params_[6];
				task_oper.params7_ = iter_oper.params_[7];
				task_oper.params8_ = iter_oper.params_[8];
				task_oper.params9_ = iter_oper.params_[9];
				task_node.opers_.push_back(std::move(task_oper));
			}
			packet.nodes_.push_back(std::move(task_node));
		}
		packet.calc_size();
		return pool_.queue_packet(pktid, asio, [&]() ->int {
			return psend(&packet);
		});
	}

	int net_motion_session::cancel_offline_task(uint64_t task_id, const std::shared_ptr<asio_partnet> &asio) {
		int err = check_connection_status();
		if (err < 0) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_task_status_t packet(PKTTYPE_CANCEL_OFFLINE_TASK, pktid);
		packet.task_id_ = task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio, [&]() ->int {
			return psend(&packet);
		});
	}

	int net_motion_session::post_offline_nextstep(uint64_t task_id, const std::shared_ptr<asio_partnet> &asio) {
		int err = check_connection_status();
		if (err < 0) {
			return err;
		}

		uint32_t pktid = pool_.get_pktid();
		nsp::proto::proto_task_status_t packet(PKTTYPE_OFFLINE_NEXT_STEP, pktid);
		packet.task_id_ = task_id;
		packet.calc_size();

		return pool_.queue_packet(pktid, asio, [&]() ->int {
			return psend(&packet);
		});
	}

}
