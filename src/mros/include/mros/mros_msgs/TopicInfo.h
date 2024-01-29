#ifndef _MROS_MSG_mros_msgs_TopicInfo_h
#define _MROS_MSG_mros_msgs_TopicInfo_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"

namespace mros
{
namespace mros_msgs
{

  class TopicInfo : public mros::Msg
  {
    public:
      typedef uint32_t _topic_id_type;
      _topic_id_type topic_id;
      typedef std::string _topic_name_type;
      _topic_name_type topic_name;
      typedef std::string _message_type_type;
      _message_type_type message_type;
      typedef std::string _md5sum_type;
      _md5sum_type md5sum;
      typedef int32_t _buffer_size_type;
      _buffer_size_type buffer_size;
      typedef bool _negotiated_type;
      _negotiated_type negotiated;
      typedef std::string _node_type;
      _node_type node;
      typedef std::string _definition_type;
      _definition_type definition;
      typedef bool _latch_type;
      _latch_type latch;
      typedef bool _reliable_type;
      _reliable_type reliable;
      typedef int32_t _queue_size_type;
      _queue_size_type queue_size;
      typedef int32_t _enabled_type;
      _enabled_type enabled;

      #if defined(_WIN32) && defined(ID_PUBLISHER)
        #undef ID_PUBLISHER
      #endif
      #if defined(_WIN32) && defined(ID_SUBSCRIBER)
        #undef ID_SUBSCRIBER
      #endif
      #if defined(_WIN32) && defined(ID_SERVICE_SERVER)
        #undef ID_SERVICE_SERVER
      #endif
      #if defined(_WIN32) && defined(ID_SERVICE_CLIENT)
        #undef ID_SERVICE_CLIENT
      #endif
      #if defined(_WIN32) && defined(ID_MROSTOPIC_REQUEST)
        #undef ID_MROSTOPIC_REQUEST
      #endif
      #if defined(_WIN32) && defined(ID_REMOVE_PUBLISHER)
        #undef ID_REMOVE_PUBLISHER
      #endif
      #if defined(_WIN32) && defined(ID_REMOVE_SUBSCRIBER)
        #undef ID_REMOVE_SUBSCRIBER
      #endif
      #if defined(_WIN32) && defined(ID_REMOVE_SERVICE_SERVER)
        #undef ID_REMOVE_SERVICE_SERVER
      #endif
      #if defined(_WIN32) && defined(ID_REMOVE_SERVICE_CLIENT)
        #undef ID_REMOVE_SERVICE_CLIENT
      #endif
      #if defined(_WIN32) && defined(ID_MROSSERVICE_REQUEST)
        #undef ID_MROSSERVICE_REQUEST
      #endif
      #if defined(_WIN32) && defined(ID_LOG)
        #undef ID_LOG
      #endif
      #if defined(_WIN32) && defined(ID_TIME)
        #undef ID_TIME
      #endif
      #if defined(_WIN32) && defined(ID_NEGOTIATED)
        #undef ID_NEGOTIATED
      #endif
      #if defined(_WIN32) && defined(ID_SESSION_ID)
        #undef ID_SESSION_ID
      #endif
      #if defined(_WIN32) && defined(ID_ASHMEM_INFO)
        #undef ID_ASHMEM_INFO
      #endif
      #if defined(_WIN32) && defined(ID_DIAG)
        #undef ID_DIAG
      #endif

      enum { ID_PUBLISHER = 0 };
      enum { ID_SUBSCRIBER = 1 };
      enum { ID_SERVICE_SERVER = 2 };
      enum { ID_SERVICE_CLIENT = 4 };
      enum { ID_MROSTOPIC_REQUEST = 6 };
      enum { ID_REMOVE_PUBLISHER = 7 };
      enum { ID_REMOVE_SUBSCRIBER = 8 };
      enum { ID_REMOVE_SERVICE_SERVER = 9 };
      enum { ID_REMOVE_SERVICE_CLIENT = 11 };
      enum { ID_MROSSERVICE_REQUEST = 13 };
      enum { ID_LOG = 14 };
      enum { ID_TIME = 15 };
      enum { ID_NEGOTIATED = 16 };
      enum { ID_SESSION_ID = 17 };
      enum { ID_ASHMEM_INFO = 18 };
      enum { ID_DIAG = 19 };

    TopicInfo():
      topic_id(0),
      topic_name(""),
      message_type(""),
      md5sum(""),
      buffer_size(0),
      negotiated(0),
      node(""),
      definition(""),
      latch(0),
      reliable(0),
      queue_size(0),
      enabled(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(__outbuffer__ + __offset__, &topic_id, 4);
      __offset__ += 4;
      uint32_t __length_topic_name__ = this->topic_name.size();
      memcpy(__outbuffer__ + __offset__, &__length_topic_name__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->topic_name.c_str(), __length_topic_name__);
      __offset__ += __length_topic_name__;
      uint32_t __length_message_type__ = this->message_type.size();
      memcpy(__outbuffer__ + __offset__, &__length_message_type__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->message_type.c_str(), __length_message_type__);
      __offset__ += __length_message_type__;
      uint32_t __length_md5sum__ = this->md5sum.size();
      memcpy(__outbuffer__ + __offset__, &__length_md5sum__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->md5sum.c_str(), __length_md5sum__);
      __offset__ += __length_md5sum__;
      memcpy(__outbuffer__ + __offset__, &buffer_size, 4);
      __offset__ += 4;
      union {
        bool real__;
        uint8_t base__;
      } __u_negotiated__;
      __u_negotiated__.real__ = this->negotiated;
      *(__outbuffer__ + __offset__ + 0) = (__u_negotiated__.base__ >> (8 * 0)) & 0xFF;
      __offset__ += 1;
      uint32_t __length_node__ = this->node.size();
      memcpy(__outbuffer__ + __offset__, &__length_node__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->node.c_str(), __length_node__);
      __offset__ += __length_node__;
      uint32_t __length_definition__ = this->definition.size();
      memcpy(__outbuffer__ + __offset__, &__length_definition__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->definition.c_str(), __length_definition__);
      __offset__ += __length_definition__;
      union {
        bool real__;
        uint8_t base__;
      } __u_latch__;
      __u_latch__.real__ = this->latch;
      *(__outbuffer__ + __offset__ + 0) = (__u_latch__.base__ >> (8 * 0)) & 0xFF;
      __offset__ += 1;
      union {
        bool real__;
        uint8_t base__;
      } __u_reliable__;
      __u_reliable__.real__ = this->reliable;
      *(__outbuffer__ + __offset__ + 0) = (__u_reliable__.base__ >> (8 * 0)) & 0xFF;
      __offset__ += 1;
      memcpy(__outbuffer__ + __offset__, &queue_size, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, &enabled, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(&topic_id, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      uint32_t __length_topic_name__ = 0;
      memcpy(&__length_topic_name__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->topic_name.assign((const char*)&__inbuffer__[__offset__], __length_topic_name__);
      __offset__ += __length_topic_name__;
      uint32_t __length_message_type__ = 0;
      memcpy(&__length_message_type__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->message_type.assign((const char*)&__inbuffer__[__offset__], __length_message_type__);
      __offset__ += __length_message_type__;
      uint32_t __length_md5sum__ = 0;
      memcpy(&__length_md5sum__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->md5sum.assign((const char*)&__inbuffer__[__offset__], __length_md5sum__);
      __offset__ += __length_md5sum__;
      memcpy(&buffer_size, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      union {
        bool real__;
        uint8_t base__;
      } __u_negotiated__;
      __u_negotiated__.base__ = 0;
      __u_negotiated__.base__ |= ((uint8_t) (*(__inbuffer__ + __offset__ + 0))) << (8 * 0);
      this->negotiated = __u_negotiated__.real__;
      __offset__ += 1;
      uint32_t __length_node__ = 0;
      memcpy(&__length_node__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->node.assign((const char*)&__inbuffer__[__offset__], __length_node__);
      __offset__ += __length_node__;
      uint32_t __length_definition__ = 0;
      memcpy(&__length_definition__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->definition.assign((const char*)&__inbuffer__[__offset__], __length_definition__);
      __offset__ += __length_definition__;
      union {
        bool real__;
        uint8_t base__;
      } __u_latch__;
      __u_latch__.base__ = 0;
      __u_latch__.base__ |= ((uint8_t) (*(__inbuffer__ + __offset__ + 0))) << (8 * 0);
      this->latch = __u_latch__.real__;
      __offset__ += 1;
      union {
        bool real__;
        uint8_t base__;
      } __u_reliable__;
      __u_reliable__.base__ = 0;
      __u_reliable__.base__ |= ((uint8_t) (*(__inbuffer__ + __offset__ + 0))) << (8 * 0);
      this->reliable = __u_reliable__.real__;
      __offset__ += 1;
      memcpy(&queue_size, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      memcpy(&enabled, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += 4;
      uint32_t __length_topic_name__ = this->topic_name.size();
      __length_ += 4;
      __length_ += __length_topic_name__;
      uint32_t __length_message_type__ = this->message_type.size();
      __length_ += 4;
      __length_ += __length_message_type__;
      uint32_t __length_md5sum__ = this->md5sum.size();
      __length_ += 4;
      __length_ += __length_md5sum__;
      __length_ += 4;
      __length_ += 1;
      uint32_t __length_node__ = this->node.size();
      __length_ += 4;
      __length_ += __length_node__;
      uint32_t __length_definition__ = this->definition.size();
      __length_ += 4;
      __length_ += __length_definition__;
      __length_ += 1;
      __length_ += 1;
      __length_ += 4;
      __length_ += 4;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        ((TopicInfo*)__msg__)->topic_id = mros::json::Number(__root__["topic_id"]).Value();
        ((TopicInfo*)__msg__)->topic_name = mros::json::String(__root__["topic_name"]).Value();
        ((TopicInfo*)__msg__)->message_type = mros::json::String(__root__["message_type"]).Value();
        ((TopicInfo*)__msg__)->md5sum = mros::json::String(__root__["md5sum"]).Value();
        ((TopicInfo*)__msg__)->buffer_size = mros::json::Number(__root__["buffer_size"]).Value();
        ((TopicInfo*)__msg__)->negotiated = mros::json::Number(__root__["negotiated"]).Value();
        ((TopicInfo*)__msg__)->node = mros::json::String(__root__["node"]).Value();
        ((TopicInfo*)__msg__)->definition = mros::json::String(__root__["definition"]).Value();
        ((TopicInfo*)__msg__)->latch = mros::json::Number(__root__["latch"]).Value();
        ((TopicInfo*)__msg__)->reliable = mros::json::Number(__root__["reliable"]).Value();
        ((TopicInfo*)__msg__)->queue_size = mros::json::Number(__root__["queue_size"]).Value();
        ((TopicInfo*)__msg__)->enabled = mros::json::Number(__root__["enabled"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      std::stringstream ss_topic_id; ss_topic_id << "\"topic_id\":" << topic_id <<",";
      __echo__ += ss_topic_id.str();
      __echo__ += "\"topic_name\":";
      __echo__ += matchString2(topic_name);
      __echo__ += ",";
      __echo__ += "\"message_type\":";
      __echo__ += matchString2(message_type);
      __echo__ += ",";
      __echo__ += "\"md5sum\":";
      __echo__ += matchString2(md5sum);
      __echo__ += ",";
      std::stringstream ss_buffer_size; ss_buffer_size << "\"buffer_size\":" << buffer_size <<",";
      __echo__ += ss_buffer_size.str();
      std::stringstream ss_negotiated; ss_negotiated << "\"negotiated\":" << negotiated <<",";
      __echo__ += ss_negotiated.str();
      __echo__ += "\"node\":";
      __echo__ += matchString2(node);
      __echo__ += ",";
      __echo__ += "\"definition\":";
      __echo__ += matchString2(definition);
      __echo__ += ",";
      std::stringstream ss_latch; ss_latch << "\"latch\":" << latch <<",";
      __echo__ += ss_latch.str();
      std::stringstream ss_reliable; ss_reliable << "\"reliable\":" << reliable <<",";
      __echo__ += ss_reliable.str();
      std::stringstream ss_queue_size; ss_queue_size << "\"queue_size\":" << queue_size <<",";
      __echo__ += ss_queue_size.str();
      std::stringstream ss_enabled; ss_enabled << "\"enabled\":" << enabled <<"";
      __echo__ += ss_enabled.str();
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "mros_msgs/TopicInfo"; }
    static std::string getTypeStatic(){ return "mros_msgs/TopicInfo"; }
    virtual const std::string getMD5() const { return "567256ed4d5848e1d4168b3a242e6ab4"; }
    static std::string getMD5Static(){ return "567256ed4d5848e1d4168b3a242e6ab4"; }
    virtual const std::string getDefinition() const { return "uint32 ID_PUBLISHER=0\nuint32 ID_SUBSCRIBER=1\nuint32 ID_SERVICE_SERVER=2\nuint32 ID_SERVICE_CLIENT=4\nuint32 ID_MROSTOPIC_REQUEST=6\nuint32 ID_REMOVE_PUBLISHER=7\nuint32 ID_REMOVE_SUBSCRIBER=8\nuint32 ID_REMOVE_SERVICE_SERVER=9\nuint32 ID_REMOVE_SERVICE_CLIENT=11\nuint32 ID_MROSSERVICE_REQUEST=13\nuint32 ID_LOG=14\nuint32 ID_TIME=15\nuint32 ID_NEGOTIATED=16\nuint32 ID_SESSION_ID=17\nuint32 ID_ASHMEM_INFO=18\nuint32 ID_DIAG=19\nuint32 topic_id\nstring topic_name\nstring message_type\nstring md5sum\nint32 buffer_size\nbool negotiated\nstring node\nstring definition\nbool latch\nbool reliable\nint32 queue_size\nint32 enabled\n"; }
    static std::string getDefinitionStatic(){ return "uint32 ID_PUBLISHER=0\nuint32 ID_SUBSCRIBER=1\nuint32 ID_SERVICE_SERVER=2\nuint32 ID_SERVICE_CLIENT=4\nuint32 ID_MROSTOPIC_REQUEST=6\nuint32 ID_REMOVE_PUBLISHER=7\nuint32 ID_REMOVE_SUBSCRIBER=8\nuint32 ID_REMOVE_SERVICE_SERVER=9\nuint32 ID_REMOVE_SERVICE_CLIENT=11\nuint32 ID_MROSSERVICE_REQUEST=13\nuint32 ID_LOG=14\nuint32 ID_TIME=15\nuint32 ID_NEGOTIATED=16\nuint32 ID_SESSION_ID=17\nuint32 ID_ASHMEM_INFO=18\nuint32 ID_DIAG=19\nuint32 topic_id\nstring topic_name\nstring message_type\nstring md5sum\nint32 buffer_size\nbool negotiated\nstring node\nstring definition\nbool latch\nbool reliable\nint32 queue_size\nint32 enabled\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::mros_msgs::TopicInfo> Ptr;
    typedef std::shared_ptr<mros::mros_msgs::TopicInfo const> ConstPtr;
  };
typedef std::shared_ptr<mros::mros_msgs::TopicInfo> TopicInfoPtr;
typedef std::shared_ptr<mros::mros_msgs::TopicInfo const> TopicInfoConstPtr;

}
}
#endif
