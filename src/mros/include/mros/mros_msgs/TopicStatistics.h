#ifndef _MROS_MSG_mros_msgs_TopicStatistics_h
#define _MROS_MSG_mros_msgs_TopicStatistics_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/mros_msgs/TopicInfo.h"

namespace mros
{
namespace mros_msgs
{

  class TopicStatistics : public mros::Msg
  {
    public:
      typedef std::string _node_type;
      _node_type node;
      typedef mros::mros_msgs::TopicInfo _subscribers_type;
      std::vector<_subscribers_type> subscribers;
      typedef mros::mros_msgs::TopicInfo _publishers_type;
      std::vector<_publishers_type> publishers;



    TopicStatistics():
      node(""),
      subscribers(0),
      publishers(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      uint32_t __length_node__ = this->node.size();
      memcpy(__outbuffer__ + __offset__, &__length_node__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->node.c_str(), __length_node__);
      __offset__ += __length_node__;
      uint32_t __subscribers_length__ = this->subscribers.size();
      memcpy(__outbuffer__ + __offset__, &__subscribers_length__, 4);
      __offset__ += 4;
      for( uint32_t i = 0; i < __subscribers_length__; i++) {
        __offset__ += this->subscribers[i].serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      }
      uint32_t __publishers_length__ = this->publishers.size();
      memcpy(__outbuffer__ + __offset__, &__publishers_length__, 4);
      __offset__ += 4;
      for( uint32_t i = 0; i < __publishers_length__; i++) {
        __offset__ += this->publishers[i].serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      }
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      uint32_t __length_node__ = 0;
      memcpy(&__length_node__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->node.assign((const char*)&__inbuffer__[__offset__], __length_node__);
      __offset__ += __length_node__;
      uint32_t __subscribers_length__ = 0; 
      memcpy(&__subscribers_length__, __inbuffer__ + __offset__, 4);
      this->subscribers.resize(__subscribers_length__); 
      __offset__ += 4;
      for( uint32_t i = 0; i < __subscribers_length__; i++) {
        __offset__ += this->subscribers[i].deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      }
      uint32_t __publishers_length__ = 0; 
      memcpy(&__publishers_length__, __inbuffer__ + __offset__, 4);
      this->publishers.resize(__publishers_length__); 
      __offset__ += 4;
      for( uint32_t i = 0; i < __publishers_length__; i++) {
        __offset__ += this->publishers[i].deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      }
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      uint32_t __length_node__ = this->node.size();
      __length_ += 4;
      __length_ += __length_node__;
      uint32_t __subscribers_length__ = this->subscribers.size();
      __length_ += 4;
      for( uint32_t i = 0; i < __subscribers_length__; i++) {
        __length_ += this->subscribers[i].serializedLength();
      }
      uint32_t __publishers_length__ = this->publishers.size();
      __length_ += 4;
      for( uint32_t i = 0; i < __publishers_length__; i++) {
        __length_ += this->publishers[i].serializedLength();
      }
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        ((TopicStatistics*)__msg__)->node = mros::json::String(__root__["node"]).Value();
        mros::json::Array __array_subscribers__ = mros::json::Array(__root__["subscribers"]);
        ((TopicStatistics*)__msg__)->subscribers.resize(__array_subscribers__.Size()); 
        for( uint32_t i = 0; i < __array_subscribers__.Size(); i++) {
          std::stringstream __stream_subscribers__;
          mros::json::Writer::Write(__root__["subscribers"][i], __stream_subscribers__);
          ((TopicStatistics*)__msg__)->subscribers[i].parse(&((TopicStatistics*)__msg__)->subscribers[i], __stream_subscribers__.str());
        }
        mros::json::Array __array_publishers__ = mros::json::Array(__root__["publishers"]);
        ((TopicStatistics*)__msg__)->publishers.resize(__array_publishers__.Size()); 
        for( uint32_t i = 0; i < __array_publishers__.Size(); i++) {
          std::stringstream __stream_publishers__;
          mros::json::Writer::Write(__root__["publishers"][i], __stream_publishers__);
          ((TopicStatistics*)__msg__)->publishers[i].parse(&((TopicStatistics*)__msg__)->publishers[i], __stream_publishers__.str());
        }
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      __echo__ += "\"node\":";
      __echo__ += matchString2(node);
      __echo__ += ",";
      uint32_t __subscribers_length__ = this->subscribers.size();
      __echo__ += "\"subscribers\":[";
      for( uint32_t i = 0; i < __subscribers_length__; i++) {
        if( i == (__subscribers_length__ - 1)) {
          __echo__ += this->subscribers[i].echo();
          __echo__ += "";
        } else {
          __echo__ += this->subscribers[i].echo();
          __echo__ += ",";
        }
      }
      __echo__ += "],";
      uint32_t __publishers_length__ = this->publishers.size();
      __echo__ += "\"publishers\":[";
      for( uint32_t i = 0; i < __publishers_length__; i++) {
        if( i == (__publishers_length__ - 1)) {
          __echo__ += this->publishers[i].echo();
          __echo__ += "";
        } else {
          __echo__ += this->publishers[i].echo();
          __echo__ += ",";
        }
      }
      __echo__ += "]";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "mros_msgs/TopicStatistics"; }
    static std::string getTypeStatic(){ return "mros_msgs/TopicStatistics"; }
    virtual const std::string getMD5() const { return "305cd1761c721e504c75082ef964a898"; }
    static std::string getMD5Static(){ return "305cd1761c721e504c75082ef964a898"; }
    virtual const std::string getDefinition() const { return "string node\nmros_msgs/TopicInfo[] subscribers\nmros_msgs/TopicInfo[] publishers\n================================================================================\nMSG: mros_msgs/TopicInfo\nuint32 ID_PUBLISHER=0\nuint32 ID_SUBSCRIBER=1\nuint32 ID_SERVICE_SERVER=2\nuint32 ID_SERVICE_CLIENT=4\nuint32 ID_MROSTOPIC_REQUEST=6\nuint32 ID_REMOVE_PUBLISHER=7\nuint32 ID_REMOVE_SUBSCRIBER=8\nuint32 ID_REMOVE_SERVICE_SERVER=9\nuint32 ID_REMOVE_SERVICE_CLIENT=11\nuint32 ID_MROSSERVICE_REQUEST=13\nuint32 ID_LOG=14\nuint32 ID_TIME=15\nuint32 ID_NEGOTIATED=16\nuint32 ID_SESSION_ID=17\nuint32 ID_ASHMEM_INFO=18\nuint32 ID_DIAG=19\nuint32 topic_id\nstring topic_name\nstring message_type\nstring md5sum\nint32 buffer_size\nbool negotiated\nstring node\nstring definition\nbool latch\nbool reliable\nint32 queue_size\nint32 enabled\n"; }
    static std::string getDefinitionStatic(){ return "string node\nmros_msgs/TopicInfo[] subscribers\nmros_msgs/TopicInfo[] publishers\n================================================================================\nMSG: mros_msgs/TopicInfo\nuint32 ID_PUBLISHER=0\nuint32 ID_SUBSCRIBER=1\nuint32 ID_SERVICE_SERVER=2\nuint32 ID_SERVICE_CLIENT=4\nuint32 ID_MROSTOPIC_REQUEST=6\nuint32 ID_REMOVE_PUBLISHER=7\nuint32 ID_REMOVE_SUBSCRIBER=8\nuint32 ID_REMOVE_SERVICE_SERVER=9\nuint32 ID_REMOVE_SERVICE_CLIENT=11\nuint32 ID_MROSSERVICE_REQUEST=13\nuint32 ID_LOG=14\nuint32 ID_TIME=15\nuint32 ID_NEGOTIATED=16\nuint32 ID_SESSION_ID=17\nuint32 ID_ASHMEM_INFO=18\nuint32 ID_DIAG=19\nuint32 topic_id\nstring topic_name\nstring message_type\nstring md5sum\nint32 buffer_size\nbool negotiated\nstring node\nstring definition\nbool latch\nbool reliable\nint32 queue_size\nint32 enabled\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::mros_msgs::TopicStatistics> Ptr;
    typedef std::shared_ptr<mros::mros_msgs::TopicStatistics const> ConstPtr;
  };
typedef std::shared_ptr<mros::mros_msgs::TopicStatistics> TopicStatisticsPtr;
typedef std::shared_ptr<mros::mros_msgs::TopicStatistics const> TopicStatisticsConstPtr;

}
}
#endif
