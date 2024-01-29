#ifndef _MROS_SRV_mros_msgs_GetDiscoveryInfo_h
#define _MROS_SRV_mros_msgs_GetDiscoveryInfo_h
#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/mros_msgs/TopicInfo.h"
#include "mros/mros_msgs/NodeInfo.h"

namespace mros
{
namespace mros_msgs
{

static const char GETDISCOVERYINFO[] = "mros_msgs/GetDiscoveryInfo";

  class GetDiscoveryInfoRequest : public mros::Msg
  {
    private:
      typedef uint32_t ___id___type;
      ___id___type __id__;

    public:



    GetDiscoveryInfoRequest()
    {
      this->__id__ = 0;
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      *(__outbuffer__ + __offset__ + 0) = (this->__id__ >> (8 * 0)) & 0xFF;
      *(__outbuffer__ + __offset__ + 1) = (this->__id__ >> (8 * 1)) & 0xFF;
      *(__outbuffer__ + __offset__ + 2) = (this->__id__ >> (8 * 2)) & 0xFF;
      *(__outbuffer__ + __offset__ + 3) = (this->__id__ >> (8 * 3)) & 0xFF;
      __offset__ += 4;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      this->__id__ =  ((uint32_t) (*(__inbuffer__ + __offset__)));
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 1))) << (8 * 1);
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 2))) << (8 * 2);
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 3))) << (8 * 3);
      __offset__ += 4;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += 4;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return GETDISCOVERYINFO; }
    static std::string getTypeStatic(){ return GETDISCOVERYINFO; }
    virtual const std::string getMD5() const { return "f99a4277b416fe4b7604474c82a5e8f8"; }
    static std::string getMD5Static(){ return "f99a4277b416fe4b7604474c82a5e8f8"; }
    virtual const std::string getDefinition() const { return ""; }
    static std::string getDefinitionStatic(){ return ""; }
    static bool hasHeader(){ return false; }
    uint32_t const getID() const { return this->__id__; }
    void setID(uint32_t id){ this->__id__ = id; }
    typedef std::shared_ptr<mros::mros_msgs::GetDiscoveryInfoRequest> Ptr;
    typedef std::shared_ptr<mros::mros_msgs::GetDiscoveryInfoRequest const> ConstPtr;
  };
typedef std::shared_ptr<mros::mros_msgs::GetDiscoveryInfoRequest> GetDiscoveryInfoRequestPtr;
typedef std::shared_ptr<mros::mros_msgs::GetDiscoveryInfoRequest const> GetDiscoveryInfoRequestConstPtr;

  class GetDiscoveryInfoResponse : public mros::Msg
  {
    private:
      typedef uint32_t ___id___type;
      ___id___type __id__;

    public:
      typedef mros::mros_msgs::NodeInfo _nodes_type;
      std::vector<_nodes_type> nodes;
      typedef mros::mros_msgs::TopicInfo _subscribers_type;
      std::vector<_subscribers_type> subscribers;
      typedef mros::mros_msgs::TopicInfo _publishers_type;
      std::vector<_publishers_type> publishers;
      typedef mros::mros_msgs::TopicInfo _services_type;
      std::vector<_services_type> services;



    GetDiscoveryInfoResponse():
      nodes(0),
      subscribers(0),
      publishers(0),
      services(0)
    {
      this->__id__ = 0;
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      *(__outbuffer__ + __offset__ + 0) = (this->__id__ >> (8 * 0)) & 0xFF;
      *(__outbuffer__ + __offset__ + 1) = (this->__id__ >> (8 * 1)) & 0xFF;
      *(__outbuffer__ + __offset__ + 2) = (this->__id__ >> (8 * 2)) & 0xFF;
      *(__outbuffer__ + __offset__ + 3) = (this->__id__ >> (8 * 3)) & 0xFF;
      __offset__ += 4;
      uint32_t __nodes_length__ = this->nodes.size();
      memcpy(__outbuffer__ + __offset__, &__nodes_length__, 4);
      __offset__ += 4;
      for( uint32_t i = 0; i < __nodes_length__; i++) {
        __offset__ += this->nodes[i].serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      }
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
      uint32_t __services_length__ = this->services.size();
      memcpy(__outbuffer__ + __offset__, &__services_length__, 4);
      __offset__ += 4;
      for( uint32_t i = 0; i < __services_length__; i++) {
        __offset__ += this->services[i].serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      }
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      this->__id__ =  ((uint32_t) (*(__inbuffer__ + __offset__)));
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 1))) << (8 * 1);
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 2))) << (8 * 2);
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 3))) << (8 * 3);
      __offset__ += 4;
      uint32_t __nodes_length__ = 0; 
      memcpy(&__nodes_length__, __inbuffer__ + __offset__, 4);
      this->nodes.resize(__nodes_length__); 
      __offset__ += 4;
      for( uint32_t i = 0; i < __nodes_length__; i++) {
        __offset__ += this->nodes[i].deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      }
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
      uint32_t __services_length__ = 0; 
      memcpy(&__services_length__, __inbuffer__ + __offset__, 4);
      this->services.resize(__services_length__); 
      __offset__ += 4;
      for( uint32_t i = 0; i < __services_length__; i++) {
        __offset__ += this->services[i].deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      }
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += 4;
      uint32_t __nodes_length__ = this->nodes.size();
      __length_ += 4;
      for( uint32_t i = 0; i < __nodes_length__; i++) {
        __length_ += this->nodes[i].serializedLength();
      }
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
      uint32_t __services_length__ = this->services.size();
      __length_ += 4;
      for( uint32_t i = 0; i < __services_length__; i++) {
        __length_ += this->services[i].serializedLength();
      }
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        mros::json::Array __array_nodes__ = mros::json::Array(__root__["nodes"]);
        ((GetDiscoveryInfoResponse*)__msg__)->nodes.resize(__array_nodes__.Size()); 
        for( uint32_t i = 0; i < __array_nodes__.Size(); i++) {
          std::stringstream __stream_nodes__;
          mros::json::Writer::Write(__root__["nodes"][i], __stream_nodes__);
          ((GetDiscoveryInfoResponse*)__msg__)->nodes[i].parse(&((GetDiscoveryInfoResponse*)__msg__)->nodes[i], __stream_nodes__.str());
        }
        mros::json::Array __array_subscribers__ = mros::json::Array(__root__["subscribers"]);
        ((GetDiscoveryInfoResponse*)__msg__)->subscribers.resize(__array_subscribers__.Size()); 
        for( uint32_t i = 0; i < __array_subscribers__.Size(); i++) {
          std::stringstream __stream_subscribers__;
          mros::json::Writer::Write(__root__["subscribers"][i], __stream_subscribers__);
          ((GetDiscoveryInfoResponse*)__msg__)->subscribers[i].parse(&((GetDiscoveryInfoResponse*)__msg__)->subscribers[i], __stream_subscribers__.str());
        }
        mros::json::Array __array_publishers__ = mros::json::Array(__root__["publishers"]);
        ((GetDiscoveryInfoResponse*)__msg__)->publishers.resize(__array_publishers__.Size()); 
        for( uint32_t i = 0; i < __array_publishers__.Size(); i++) {
          std::stringstream __stream_publishers__;
          mros::json::Writer::Write(__root__["publishers"][i], __stream_publishers__);
          ((GetDiscoveryInfoResponse*)__msg__)->publishers[i].parse(&((GetDiscoveryInfoResponse*)__msg__)->publishers[i], __stream_publishers__.str());
        }
        mros::json::Array __array_services__ = mros::json::Array(__root__["services"]);
        ((GetDiscoveryInfoResponse*)__msg__)->services.resize(__array_services__.Size()); 
        for( uint32_t i = 0; i < __array_services__.Size(); i++) {
          std::stringstream __stream_services__;
          mros::json::Writer::Write(__root__["services"][i], __stream_services__);
          ((GetDiscoveryInfoResponse*)__msg__)->services[i].parse(&((GetDiscoveryInfoResponse*)__msg__)->services[i], __stream_services__.str());
        }
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      uint32_t __nodes_length__ = this->nodes.size();
      __echo__ += "\"nodes\":[";
      for( uint32_t i = 0; i < __nodes_length__; i++) {
        if( i == (__nodes_length__ - 1)) {
          __echo__ += this->nodes[i].echo();
          __echo__ += "";
        } else {
          __echo__ += this->nodes[i].echo();
          __echo__ += ",";
        }
      }
      __echo__ += "],";
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
      __echo__ += "],";
      uint32_t __services_length__ = this->services.size();
      __echo__ += "\"services\":[";
      for( uint32_t i = 0; i < __services_length__; i++) {
        if( i == (__services_length__ - 1)) {
          __echo__ += this->services[i].echo();
          __echo__ += "";
        } else {
          __echo__ += this->services[i].echo();
          __echo__ += ",";
        }
      }
      __echo__ += "]";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return GETDISCOVERYINFO; }
    static std::string getTypeStatic(){ return GETDISCOVERYINFO; }
    virtual const std::string getMD5() const { return "f99a4277b416fe4b7604474c82a5e8f8"; }
    static std::string getMD5Static(){ return "f99a4277b416fe4b7604474c82a5e8f8"; }
    virtual const std::string getDefinition() const { return "mros_msgs/NodeInfo[] nodes\nmros_msgs/TopicInfo[] subscribers\nmros_msgs/TopicInfo[] publishers\nmros_msgs/TopicInfo[] services\n================================================================================\nMSG: mros_msgs/NodeInfo\nstring node_name\n================================================================================\nMSG: mros_msgs/TopicInfo\nuint32 ID_PUBLISHER=0\nuint32 ID_SUBSCRIBER=1\nuint32 ID_SERVICE_SERVER=2\nuint32 ID_SERVICE_CLIENT=4\nuint32 ID_MROSTOPIC_REQUEST=6\nuint32 ID_REMOVE_PUBLISHER=7\nuint32 ID_REMOVE_SUBSCRIBER=8\nuint32 ID_REMOVE_SERVICE_SERVER=9\nuint32 ID_REMOVE_SERVICE_CLIENT=11\nuint32 ID_MROSSERVICE_REQUEST=13\nuint32 ID_LOG=14\nuint32 ID_TIME=15\nuint32 ID_NEGOTIATED=16\nuint32 ID_SESSION_ID=17\nuint32 ID_ASHMEM_INFO=18\nuint32 ID_DIAG=19\nuint32 topic_id\nstring topic_name\nstring message_type\nstring md5sum\nint32 buffer_size\nbool negotiated\nstring node\nstring definition\nbool latch\nbool reliable\nint32 queue_size\nint32 enabled\n"; }
    static std::string getDefinitionStatic(){ return "mros_msgs/NodeInfo[] nodes\nmros_msgs/TopicInfo[] subscribers\nmros_msgs/TopicInfo[] publishers\nmros_msgs/TopicInfo[] services\n================================================================================\nMSG: mros_msgs/NodeInfo\nstring node_name\n================================================================================\nMSG: mros_msgs/TopicInfo\nuint32 ID_PUBLISHER=0\nuint32 ID_SUBSCRIBER=1\nuint32 ID_SERVICE_SERVER=2\nuint32 ID_SERVICE_CLIENT=4\nuint32 ID_MROSTOPIC_REQUEST=6\nuint32 ID_REMOVE_PUBLISHER=7\nuint32 ID_REMOVE_SUBSCRIBER=8\nuint32 ID_REMOVE_SERVICE_SERVER=9\nuint32 ID_REMOVE_SERVICE_CLIENT=11\nuint32 ID_MROSSERVICE_REQUEST=13\nuint32 ID_LOG=14\nuint32 ID_TIME=15\nuint32 ID_NEGOTIATED=16\nuint32 ID_SESSION_ID=17\nuint32 ID_ASHMEM_INFO=18\nuint32 ID_DIAG=19\nuint32 topic_id\nstring topic_name\nstring message_type\nstring md5sum\nint32 buffer_size\nbool negotiated\nstring node\nstring definition\nbool latch\nbool reliable\nint32 queue_size\nint32 enabled\n"; }
    static bool hasHeader(){ return false; }
    uint32_t const getID() const { return this->__id__; }
    void setID(uint32_t id){ this->__id__ = id; }
    typedef std::shared_ptr<mros::mros_msgs::GetDiscoveryInfoResponse> Ptr;
    typedef std::shared_ptr<mros::mros_msgs::GetDiscoveryInfoResponse const> ConstPtr;
  };
typedef std::shared_ptr<mros::mros_msgs::GetDiscoveryInfoResponse> GetDiscoveryInfoResponsePtr;
typedef std::shared_ptr<mros::mros_msgs::GetDiscoveryInfoResponse const> GetDiscoveryInfoResponseConstPtr;

  class GetDiscoveryInfo {
    public:
    typedef GetDiscoveryInfoRequest Request;
    typedef GetDiscoveryInfoResponse Response;
    Request request;
    Response response;
  };

}
}
#endif
