#ifndef PROTO_LOCALIZATION_MSG
#define PROTO_LOCALIZATION_MSG

#include <memory.h>
#include <stdlib.h>
#include "proto_definion.h"
#include "serialize.hpp"

namespace nsp {
    namespace proto {
        struct proto_localization_cfgwrite : public nsp::proto::proto_interface
        {
            proto_localization_cfgwrite(uint32_t type,uint32_t id) :head_(type,id), blob_(128) {}

            nsp::proto::proto_head                      head_;
            nsp::proto::proto_blob_t<unsigned char>     blob_;

            unsigned char *serialize(unsigned char * byte_stream) const
            {
                unsigned char *pos = byte_stream;
                pos = head_.serialize(pos);
                pos = blob_.serialize(pos);
                return pos;
            }

            const unsigned char *build(const  unsigned char * byte_stream, int &cb)
            {
                const unsigned char *pos = byte_stream;
                pos = head_.build(pos, cb);
                pos = blob_.build(pos, cb);
                return pos;
            }

            const int length() const
            {
                return head_.length() + blob_.length();
            }

            void calc_size()
            {
                head_.size_ = length();
            }
        };

        struct proto_localization_cfgread_ack :public nsp::proto::proto_interface
        {
            proto_localization_cfgread_ack() : blob_(128) {}
            
            nsp::proto::proto_head head_;
            nsp::proto::proto_blob_t<unsigned char>     blob_;

            unsigned char *serialize(unsigned char * byte_stream) const override
            {
                unsigned char *pos = byte_stream;
                pos = head_.serialize(pos);
                pos = blob_.serialize(pos);
                return pos;
            }

            const unsigned char *build(const  unsigned char * byte_stream, int &cb) override
            {
                const unsigned char *pos = byte_stream;
                pos = head_.build(pos, cb);
                pos = blob_.build(pos, cb);
                return pos;
            }

            const int length() const override
            {
                return  head_.length() + blob_.length();
            }
        };
    }
}

#endif