#ifndef _MROS_MSG_std_msgs_Int64Array_h
#define _MROS_MSG_std_msgs_Int64Array_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/std_msgs/Header.h"

namespace mros
{
namespace std_msgs
{

  class Int64Array : public mros::Msg
  {
    public:
      typedef mros::std_msgs::Header _header_type;
      _header_type header;
      typedef int64_t _data_type;
      std::vector<_data_type> data;



    Int64Array():
      header(),
      data(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      uint32_t __data_length__ = this->data.size();
      memcpy(__outbuffer__ + __offset__, &__data_length__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->data.data(), __data_length__ * 8);
      __offset__ += __data_length__ * 8;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      uint32_t __data_length__ = 0; 
      memcpy(&__data_length__, __inbuffer__ + __offset__, 4);
      this->data.resize(__data_length__); 
      __offset__ += 4;
      memcpy(this->data.data(), __inbuffer__ + __offset__, __data_length__ * 8);
      __offset__ += __data_length__ * 8;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += this->header.serializedLength();
      uint32_t __data_length__ = this->data.size();
      __length_ += 4;
      __length_ += __data_length__ * 8;
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
        ((Int64Array*)__msg__)->header.parse(&((Int64Array*)__msg__)->header, __stream_header__.str());
        mros::json::Array __array_data__ = mros::json::Array(__root__["data"]);
        ((Int64Array*)__msg__)->data.resize(__array_data__.Size()); 
        for( uint32_t i = 0; i < __array_data__.Size(); i++) {
          ((Int64Array*)__msg__)->data[i] = mros::json::Number(__root__["data"][i]).Value();
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
      uint32_t __data_length__ = this->data.size();
      __echo__ += "\"data\":[";
      for( uint32_t i = 0; i < __data_length__; i++) {
        if( i == (__data_length__ - 1)) {
          std::stringstream ss_datai; ss_datai << data[i] <<"";
          __echo__ += ss_datai.str();
        } else {
          std::stringstream ss_datai; ss_datai << data[i] <<",";
          __echo__ += ss_datai.str();
        }
      }
      __echo__ += "]";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return &header; }
    virtual const std::string getType() const { return "std_msgs/Int64Array"; }
    static std::string getTypeStatic(){ return "std_msgs/Int64Array"; }
    virtual const std::string getMD5() const { return "3c62002f0defb2682656f1d354eeffd4"; }
    static std::string getMD5Static(){ return "3c62002f0defb2682656f1d354eeffd4"; }
    virtual const std::string getDefinition() const { return "std_msgs/Header header\nint64[] data\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static std::string getDefinitionStatic(){ return "std_msgs/Header header\nint64[] data\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static bool hasHeader(){ return true; }
    typedef std::shared_ptr<mros::std_msgs::Int64Array> Ptr;
    typedef std::shared_ptr<mros::std_msgs::Int64Array const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::Int64Array> Int64ArrayPtr;
typedef std::shared_ptr<mros::std_msgs::Int64Array const> Int64ArrayConstPtr;

}
}
#endif
