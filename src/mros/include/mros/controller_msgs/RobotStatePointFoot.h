#ifndef _MROS_MSG_controller_msgs_RobotStatePointFoot_h
#define _MROS_MSG_controller_msgs_RobotStatePointFoot_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/std_msgs/Header.h"

namespace mros
{
namespace controller_msgs
{

  class RobotStatePointFoot : public mros::Msg
  {
    public:
      typedef mros::std_msgs::Header _header_type;
      _header_type header;
      typedef double _qState_type;
      std::vector<_qState_type> qState;
      typedef double _qdState_type;
      std::vector<_qdState_type> qdState;
      typedef double _currentState_type;
      std::vector<_currentState_type> currentState;



    RobotStatePointFoot():
      header(),
      qState(6, 0.0),
      qdState(6, 0.0),
      currentState(6, 0.0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      memcpy(__outbuffer__ + __offset__, this->qState.data(), 6 * 8);
      __offset__ += 6 * 8;
      memcpy(__outbuffer__ + __offset__, this->qdState.data(), 6 * 8);
      __offset__ += 6 * 8;
      memcpy(__outbuffer__ + __offset__, this->currentState.data(), 6 * 8);
      __offset__ += 6 * 8;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      memcpy(this->qState.data(), __inbuffer__ + __offset__, 6 * 8);
      __offset__ += 6 * 8;
      memcpy(this->qdState.data(), __inbuffer__ + __offset__, 6 * 8);
      __offset__ += 6 * 8;
      memcpy(this->currentState.data(), __inbuffer__ + __offset__, 6 * 8);
      __offset__ += 6 * 8;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += this->header.serializedLength();
      __length_ += 6 * 8;
      __length_ += 6 * 8;
      __length_ += 6 * 8;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        std::stringstream __stream_header__;
        mros::json::Writer::Write(__root__["header"], __stream_header__);
        ((RobotStatePointFoot*)__msg__)->header.parse(&((RobotStatePointFoot*)__msg__)->header, __stream_header__.str());
        mros::json::Array __array_qState__ = mros::json::Array(__root__["qState"]);
        ((RobotStatePointFoot*)__msg__)->qState.resize(6); 
        for( uint32_t i = 0; i < __array_qState__.Size() && i < 6; i++) {
          ((RobotStatePointFoot*)__msg__)->qState[i] = mros::json::Number(__root__["qState"][i]).Value();
        }
        mros::json::Array __array_qdState__ = mros::json::Array(__root__["qdState"]);
        ((RobotStatePointFoot*)__msg__)->qdState.resize(6); 
        for( uint32_t i = 0; i < __array_qdState__.Size() && i < 6; i++) {
          ((RobotStatePointFoot*)__msg__)->qdState[i] = mros::json::Number(__root__["qdState"][i]).Value();
        }
        mros::json::Array __array_currentState__ = mros::json::Array(__root__["currentState"]);
        ((RobotStatePointFoot*)__msg__)->currentState.resize(6); 
        for( uint32_t i = 0; i < __array_currentState__.Size() && i < 6; i++) {
          ((RobotStatePointFoot*)__msg__)->currentState[i] = mros::json::Number(__root__["currentState"][i]).Value();
        }
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      __echo__ += "\"header\":";
      __echo__ += this->header.echo();
      __echo__ += ",";
      __echo__ += "\"qState\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_qStatei; ss_qStatei << qState[i] <<"";
          __echo__ += ss_qStatei.str();
        } else {
          std::stringstream ss_qStatei; ss_qStatei << qState[i] <<",";
          __echo__ += ss_qStatei.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"qdState\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_qdStatei; ss_qdStatei << qdState[i] <<"";
          __echo__ += ss_qdStatei.str();
        } else {
          std::stringstream ss_qdStatei; ss_qdStatei << qdState[i] <<",";
          __echo__ += ss_qdStatei.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"currentState\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_currentStatei; ss_currentStatei << currentState[i] <<"";
          __echo__ += ss_currentStatei.str();
        } else {
          std::stringstream ss_currentStatei; ss_currentStatei << currentState[i] <<",";
          __echo__ += ss_currentStatei.str();
        }
      }
      __echo__ += "]";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return &header; }
    virtual const std::string getType() const { return "controller_msgs/RobotStatePointFoot"; }
    static std::string getTypeStatic(){ return "controller_msgs/RobotStatePointFoot"; }
    virtual const std::string getMD5() const { return "62f1fff8dc6bd3e9cfd0a57d9854672b"; }
    static std::string getMD5Static(){ return "62f1fff8dc6bd3e9cfd0a57d9854672b"; }
    virtual const std::string getDefinition() const { return "std_msgs/Header header\nfloat64[6] qState\nfloat64[6] qdState\nfloat64[6] currentState\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static std::string getDefinitionStatic(){ return "std_msgs/Header header\nfloat64[6] qState\nfloat64[6] qdState\nfloat64[6] currentState\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static bool hasHeader(){ return true; }
    typedef std::shared_ptr<mros::controller_msgs::RobotStatePointFoot> Ptr;
    typedef std::shared_ptr<mros::controller_msgs::RobotStatePointFoot const> ConstPtr;
  };
typedef std::shared_ptr<mros::controller_msgs::RobotStatePointFoot> RobotStatePointFootPtr;
typedef std::shared_ptr<mros::controller_msgs::RobotStatePointFoot const> RobotStatePointFootConstPtr;

}
}
#endif
