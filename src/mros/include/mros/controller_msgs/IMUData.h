#ifndef _MROS_MSG_controller_msgs_IMUData_h
#define _MROS_MSG_controller_msgs_IMUData_h

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

  class IMUData : public mros::Msg
  {
    public:
      typedef mros::std_msgs::Header _header_type;
      _header_type header;
      typedef double _euler_type;
      std::vector<_euler_type> euler;
      typedef double _quat_type;
      std::vector<_quat_type> quat;
      typedef double _acc_type;
      std::vector<_acc_type> acc;
      typedef double _gyro_type;
      std::vector<_gyro_type> gyro;



    IMUData():
      header(),
      euler(3, 0.0),
      quat(4, 0.0),
      acc(3, 0.0),
      gyro(3, 0.0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      memcpy(__outbuffer__ + __offset__, this->euler.data(), 3 * 8);
      __offset__ += 3 * 8;
      memcpy(__outbuffer__ + __offset__, this->quat.data(), 4 * 8);
      __offset__ += 4 * 8;
      memcpy(__outbuffer__ + __offset__, this->acc.data(), 3 * 8);
      __offset__ += 3 * 8;
      memcpy(__outbuffer__ + __offset__, this->gyro.data(), 3 * 8);
      __offset__ += 3 * 8;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      memcpy(this->euler.data(), __inbuffer__ + __offset__, 3 * 8);
      __offset__ += 3 * 8;
      memcpy(this->quat.data(), __inbuffer__ + __offset__, 4 * 8);
      __offset__ += 4 * 8;
      memcpy(this->acc.data(), __inbuffer__ + __offset__, 3 * 8);
      __offset__ += 3 * 8;
      memcpy(this->gyro.data(), __inbuffer__ + __offset__, 3 * 8);
      __offset__ += 3 * 8;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += this->header.serializedLength();
      __length_ += 3 * 8;
      __length_ += 4 * 8;
      __length_ += 3 * 8;
      __length_ += 3 * 8;
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
        ((IMUData*)__msg__)->header.parse(&((IMUData*)__msg__)->header, __stream_header__.str());
        mros::json::Array __array_euler__ = mros::json::Array(__root__["euler"]);
        ((IMUData*)__msg__)->euler.resize(3); 
        for( uint32_t i = 0; i < __array_euler__.Size() && i < 3; i++) {
          ((IMUData*)__msg__)->euler[i] = mros::json::Number(__root__["euler"][i]).Value();
        }
        mros::json::Array __array_quat__ = mros::json::Array(__root__["quat"]);
        ((IMUData*)__msg__)->quat.resize(4); 
        for( uint32_t i = 0; i < __array_quat__.Size() && i < 4; i++) {
          ((IMUData*)__msg__)->quat[i] = mros::json::Number(__root__["quat"][i]).Value();
        }
        mros::json::Array __array_acc__ = mros::json::Array(__root__["acc"]);
        ((IMUData*)__msg__)->acc.resize(3); 
        for( uint32_t i = 0; i < __array_acc__.Size() && i < 3; i++) {
          ((IMUData*)__msg__)->acc[i] = mros::json::Number(__root__["acc"][i]).Value();
        }
        mros::json::Array __array_gyro__ = mros::json::Array(__root__["gyro"]);
        ((IMUData*)__msg__)->gyro.resize(3); 
        for( uint32_t i = 0; i < __array_gyro__.Size() && i < 3; i++) {
          ((IMUData*)__msg__)->gyro[i] = mros::json::Number(__root__["gyro"][i]).Value();
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
      __echo__ += "\"euler\":[";
      for( uint32_t i = 0; i < 3; i++) {
        if( i == (3 - 1)) {
          std::stringstream ss_euleri; ss_euleri << euler[i] <<"";
          __echo__ += ss_euleri.str();
        } else {
          std::stringstream ss_euleri; ss_euleri << euler[i] <<",";
          __echo__ += ss_euleri.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"quat\":[";
      for( uint32_t i = 0; i < 4; i++) {
        if( i == (4 - 1)) {
          std::stringstream ss_quati; ss_quati << quat[i] <<"";
          __echo__ += ss_quati.str();
        } else {
          std::stringstream ss_quati; ss_quati << quat[i] <<",";
          __echo__ += ss_quati.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"acc\":[";
      for( uint32_t i = 0; i < 3; i++) {
        if( i == (3 - 1)) {
          std::stringstream ss_acci; ss_acci << acc[i] <<"";
          __echo__ += ss_acci.str();
        } else {
          std::stringstream ss_acci; ss_acci << acc[i] <<",";
          __echo__ += ss_acci.str();
        }
      }
      __echo__ += "],";
      __echo__ += "\"gyro\":[";
      for( uint32_t i = 0; i < 3; i++) {
        if( i == (3 - 1)) {
          std::stringstream ss_gyroi; ss_gyroi << gyro[i] <<"";
          __echo__ += ss_gyroi.str();
        } else {
          std::stringstream ss_gyroi; ss_gyroi << gyro[i] <<",";
          __echo__ += ss_gyroi.str();
        }
      }
      __echo__ += "]";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return &header; }
    virtual const std::string getType() const { return "controller_msgs/IMUData"; }
    static std::string getTypeStatic(){ return "controller_msgs/IMUData"; }
    virtual const std::string getMD5() const { return "82c73fbe9665d091c171f7e183970344"; }
    static std::string getMD5Static(){ return "82c73fbe9665d091c171f7e183970344"; }
    virtual const std::string getDefinition() const { return "std_msgs/Header header\nfloat64[3] euler\nfloat64[4] quat\nfloat64[3] acc\nfloat64[3] gyro\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static std::string getDefinitionStatic(){ return "std_msgs/Header header\nfloat64[3] euler\nfloat64[4] quat\nfloat64[3] acc\nfloat64[3] gyro\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static bool hasHeader(){ return true; }
    typedef std::shared_ptr<mros::controller_msgs::IMUData> Ptr;
    typedef std::shared_ptr<mros::controller_msgs::IMUData const> ConstPtr;
  };
typedef std::shared_ptr<mros::controller_msgs::IMUData> IMUDataPtr;
typedef std::shared_ptr<mros::controller_msgs::IMUData const> IMUDataConstPtr;

}
}
#endif
