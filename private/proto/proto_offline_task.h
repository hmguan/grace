#ifndef PROTO_OFFLINE_TASK_H
#define PROTO_OFFLINE_TASK_H

#include "proto_navigation_task.h"

namespace nsp {
	namespace proto {
		typedef struct proto_offline_operation : public nsp::proto::proto_interface
		{
			nsp::proto::proto_crt_t<int32_t> code_;
			nsp::proto::proto_crt_t<uint64_t> params0_;
			nsp::proto::proto_crt_t<uint64_t> params1_;
			nsp::proto::proto_crt_t<uint64_t> params2_;
			nsp::proto::proto_crt_t<uint64_t> params3_;
			nsp::proto::proto_crt_t<uint64_t> params4_;
			nsp::proto::proto_crt_t<uint64_t> params5_;
			nsp::proto::proto_crt_t<uint64_t> params6_;
			nsp::proto::proto_crt_t<uint64_t> params7_;
			nsp::proto::proto_crt_t<uint64_t> params8_;
			nsp::proto::proto_crt_t<uint64_t> params9_;

			unsigned char *serialize(unsigned char *bytes) const override
			{
				unsigned char *pos = bytes;
				pos = code_.serialize(pos);
				pos = params0_.serialize(pos);
				pos = params1_.serialize(pos);
				pos = params2_.serialize(pos);
				pos = params3_.serialize(pos);
				pos = params4_.serialize(pos);
				pos = params5_.serialize(pos);
				pos = params6_.serialize(pos);
				pos = params7_.serialize(pos);
				pos = params8_.serialize(pos);
				pos = params9_.serialize(pos);
				return pos;
			}

			const unsigned char *build(const unsigned char *bytes, int &cb) override
			{
				const unsigned char *pos = bytes;
				pos = code_.build(pos, cb);
				pos = params0_.build(pos, cb);
				pos = params1_.build(pos, cb);
				pos = params2_.build(pos, cb);
				pos = params3_.build(pos, cb);
				pos = params4_.build(pos, cb);
				pos = params5_.build(pos, cb);
				pos = params6_.build(pos, cb);
				pos = params7_.build(pos, cb);
				pos = params8_.build(pos, cb);
				pos = params9_.build(pos, cb);
				return pos;
			}

			const int length() const override
			{
				return code_.length() + params0_.length() * 10;
			}
		} proto_offline_operation_t;

		typedef struct proto_offline_node : public nsp::proto::proto_interface
		{
			nsp::proto::upl_t dest_upl_;
			nsp::proto::position_t dest_pos_;
			nsp::proto::proto_vector_t<trail_t> trails_;
			nsp::proto::proto_vector_t<proto_offline_operation_t> opers_;

			unsigned char *serialize(unsigned char *bytes) const override
			{
				unsigned char *pos = bytes;
				pos = dest_upl_.serialize(pos);
				pos = dest_pos_.serialize(pos);
				pos = trails_.serialize(pos);
				pos = opers_.serialize(pos);
				return pos;
			}

			const unsigned char *build(const unsigned char *bytes, int &cb) override
			{
				const unsigned char *pos = bytes;
				pos = dest_upl_.build(pos, cb);
				pos = dest_pos_.build(pos, cb);
				pos = trails_.build(pos, cb);
				pos = opers_.build(pos, cb);
				return pos;
			}

			const int length() const override
			{
				return dest_upl_.length() + dest_pos_.length() + trails_.length() + opers_.length();
			}
		} proto_offline_node_t;

		typedef struct proto_offline_task : public nsp::proto::proto_interface
		{
			proto_offline_task(uint32_t type, uint32_t id) :head_(type, id) {}

			nsp::proto::proto_head head_;
			nsp::proto::proto_crt_t<uint64_t> task_id_;
			nsp::proto::proto_vector_t<proto_offline_node_t> nodes_;

			unsigned char *serialize(unsigned char *bytes) const override
			{
				unsigned char *pos = bytes;
				pos = head_.serialize(pos);
				pos = task_id_.serialize(pos);
				pos = nodes_.serialize(pos);
				return pos;
			}

			const unsigned char *build(const unsigned char *bytes, int &cb) override
			{
				const unsigned char *pos = bytes;
				pos = head_.build(pos, cb);
				pos = task_id_.build(pos, cb);
				pos = nodes_.build(pos, cb);
				return pos;
			}

			const int length() const override
			{
				return head_.length() + task_id_.length() + nodes_.length();
			}

			void calc_size()
			{
				head_.size_ = length();
			}
		} proto_offline_task_t;
	}
}

#endif // PROTO_OFFLINE_TASK_H 
