#ifndef _MROS_MSG_controller_msgs_RobotCmdPointFoot_h
#define _MROS_MSG_controller_msgs_RobotCmdPointFoot_h

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

  class RobotCmdPointFoot : public mros::Msg
  {
    public:
      typedef mros::std_msgs::Header _header_type;
      _header_type header;
      typedef float _qCmd_type;
      std::vector<_qCmd_type> qCmd;
      typedef float _qdCmd_type;
      std::vector<_qdCmd_type> qdCmd;
      typedef float _kpCmd_type;
      std::vector<_kpCmd_type> kpCmd;
      typedef float _kdCmd_type;
      std::vector<_kdCmd_type> kdCmd;
      typedef float _currentCmd_type;
      std::vector<_currentCmd_type> currentCmd;
      typedef uint8_t _mode_type;
      std::vector<_mode_type> mode;



    RobotCmdPointFoot():
      header(),
      qCmd(6, 0.0),
      qdCmd(6, 0.0),
      kpCmd(6, 0.0),
      kdCmd(6, 0.0),
      currentCmd(6, 0.0),
      mode(6, 0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      memcpy(__outbuffer__ + __offset__, this->qCmd.data(), 6 * 4);
      __offset__ += 6 * 4;
      memcpy(__outbuffer__ + __offset__, this->qdCmd.data(), 6 * 4);
      __offset__ += 6 * 4;
      memcpy(__outbuffer__ + __offset__, this->kpCmd.data(), 6 * 4);
      __offset__ += 6 * 4;
      memcpy(__outbuffer__ + __offset__, this->kdCmd.data(), 6 * 4);
      __offset__ += 6 * 4;
      memcpy(__outbuffer__ + __offset__, this->currentCmd.data(), 6 * 4);
      __offset__ += 6 * 4;
      memcpy(__outbuffer__ + __offset__, this->mode.data(), 6 * 1);
      __offset__ += 6 * 1;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      memcpy(this->qCmd.data(), __inbuffer__ + __offset__, 6 * 4);
      __offset__ += 6 * 4;
      memcpy(this->qdCmd.data(), __inbuffer__ + __offset__, 6 * 4);
      __offset__ += 6 * 4;
      memcpy(this->kpCmd.data(), __inbuffer__ + __offset__, 6 * 4);
      __offset__ += 6 * 4;
      memcpy(this->kdCmd.data(), __inbuffer__ + __offset__, 6 * 4);
      __offset__ += 6 * 4;
      memcpy(this->currentCmd.data(), __inbuffer__ + __offset__, 6 * 4);
      __offset__ += 6 * 4;
      memcpy(this->mode.data(), __inbuffer__ + __offset__, 6 * 1);
      __offset__ += 6 * 1;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += this->header.serializedLength();
      __length_ += 6 * 4;
      __length_ += 6 * 4;
      __length_ += 6 * 4;
      __length_ += 6 * 4;
      __length_ += 6 * 4;
      __length_ += 6 * 1;
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
        ((RobotCmdPointFoot*)__msg__)->header.parse(&((RobotCmdPointFoot*)__msg__)->header, __stream_header__.str());
        mros::json::Array __array_qCmd__ = mros::json::Array(__root__["qCmd"]);
        ((RobotCmdPointFoot*)__msg__)->qCmd.resize(6); 
        for( uint32_t i = 0; i < __array_qCmd__.Size() && i < 6; i++) {
          ((RobotCmdPointFoot*)__msg__)->qCmd[i] = mros::json::Number(__root__["qCmd"][i]).Value();
        }
        mros::json::Array __array_qdCmd__ = mros::json::Array(__root__["qdCmd"]);
        ((RobotCmdPointFoot*)__msg__)->qdCmd.resize(6); 
        for( uint32_t i = 0; i < __array_qdCmd__.Size() && i < 6; i++) {
          ((RobotCmdPointFoot*)__msg__)->qdCmd[i] = mros::json::Number(__root__["qdCmd"][i]).Value();
        }
        mros::json::Array __array_kpCmd__ = mros::json::Array(__root__["kpCmd"]);
        ((RobotCmdPointFoot*)__msg__)->kpCmd.resize(6); 
        for( uint32_t i = 0; i < __array_kpCmd__.Size() && i < 6; i++) {
          ((RobotCmdPointFoot*)__msg__)->kpCmd[i] = mros::json::Number(__root__["kpCmd"][i]).Value();
        }
        mros::json::Array __array_kdCmd__ = mros::json::Array(__root__["kdCmd"]);
        ((RobotCmdPointFoot*)__msg__)->kdCmd.resize(6); 
        for( uint32_t i = 0; i < __array_kdCmd__.Size() && i < 6; i++) {
          ((RobotCmdPointFoot*)__msg__)->kdCmd[i] = mros::json::Number(__root__["kdCmd"][i]).Value();
        }
        mros::json::Array __array_currentCmd__ = mros::json::Array(__root__["currentCmd"]);
        ((RobotCmdPointFoot*)__msg__)->currentCmd.resize(6); 
        for( uint32_t i = 0; i < __array_currentCmd__.Size() && i < 6; i++) {
          ((RobotCmdPointFoot*)__msg__)->currentCmd[i] = mros::json::Number(__root__["currentCmd"][i]).Value();
        }
        mros::json::Array __array_mode__ = mros::json::Array(__root__["mode"]);
        ((RobotCmdPointFoot*)__msg__)->mode.resize(6); 
        for( uint32_t i = 0; i < __array_mode__.Size() && i < 6; i++) {
          ((RobotCmdPointFoot*)__msg__)->mode[i] = mros::json::Number(__root__["mode"][i]).Value();
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
      __echo__ += "\"qCmd\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_qCmdi; ss_qCmdi << qCmd[i] <<"";
          __echo__ += ss_qCmdi.str();
        } else {
          std::stringstream ss_qCmdi; ss_qCmdi << qCmd[i] <<",";
          __echo__ += ss_qCmdi.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"qdCmd\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_qdCmdi; ss_qdCmdi << qdCmd[i] <<"";
          __echo__ += ss_qdCmdi.str();
        } else {
          std::stringstream ss_qdCmdi; ss_qdCmdi << qdCmd[i] <<",";
          __echo__ += ss_qdCmdi.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"kpCmd\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_kpCmdi; ss_kpCmdi << kpCmd[i] <<"";
          __echo__ += ss_kpCmdi.str();
        } else {
          std::stringstream ss_kpCmdi; ss_kpCmdi << kpCmd[i] <<",";
          __echo__ += ss_kpCmdi.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"kdCmd\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_kdCmdi; ss_kdCmdi << kdCmd[i] <<"";
          __echo__ += ss_kdCmdi.str();
        } else {
          std::stringstream ss_kdCmdi; ss_kdCmdi << kdCmd[i] <<",";
          __echo__ += ss_kdCmdi.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"currentCmd\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_currentCmdi; ss_currentCmdi << currentCmd[i] <<"";
          __echo__ += ss_currentCmdi.str();
        } else {
          std::stringstream ss_currentCmdi; ss_currentCmdi << currentCmd[i] <<",";
          __echo__ += ss_currentCmdi.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"mode\":[";
      for( uint32_t i = 0; i < 6; i++) {
        if( i == (6 - 1)) {
          std::stringstream ss_modei; ss_modei << (uint16_t)mode[i] <<"";
          __echo__ += ss_modei.str();
        } else {
          std::stringstream ss_modei; ss_modei << (uint16_t)mode[i] <<",";
          __echo__ += ss_modei.str();
        }
      }
      __echo__ += "]";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return &header; }
    virtual const std::string getType() const { return "controller_msgs/RobotCmdPointFoot"; }
    static std::string getTypeStatic(){ return "controller_msgs/RobotCmdPointFoot"; }
    virtual const std::string getMD5() const { return "9977052abb2b90a4721c3735646307ec"; }
    static std::string getMD5Static(){ return "9977052abb2b90a4721c3735646307ec"; }
    virtual const std::string getDefinition() const { return "std_msgs/Header header\nfloat32[6] qCmd\nfloat32[6] qdCmd\nfloat32[6] kpCmd\nfloat32[6] kdCmd\nfloat32[6] currentCmd\nuint8[6] mode\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static std::string getDefinitionStatic(){ return "std_msgs/Header header\nfloat32[6] qCmd\nfloat32[6] qdCmd\nfloat32[6] kpCmd\nfloat32[6] kdCmd\nfloat32[6] currentCmd\nuint8[6] mode\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static bool hasHeader(){ return true; }
    typedef std::shared_ptr<mros::controller_msgs::RobotCmdPointFoot> Ptr;
    typedef std::shared_ptr<mros::controller_msgs::RobotCmdPointFoot const> ConstPtr;
  };
typedef std::shared_ptr<mros::controller_msgs::RobotCmdPointFoot> RobotCmdPointFootPtr;
typedef std::shared_ptr<mros::controller_msgs::RobotCmdPointFoot const> RobotCmdPointFootConstPtr;

}
}
#endif
