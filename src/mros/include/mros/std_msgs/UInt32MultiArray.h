#ifndef _MROS_MSG_std_msgs_UInt32MultiArray_h
#define _MROS_MSG_std_msgs_UInt32MultiArray_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/std_msgs/MultiArrayLayout.h"

namespace mros
{
namespace std_msgs
{

  class UInt32MultiArray : public mros::Msg
  {
    public:
      typedef mros::std_msgs::MultiArrayLayout _layout_type;
      _layout_type layout;
      typedef uint32_t _data_type;
      std::vector<_data_type> data;



    UInt32MultiArray():
      layout(),
      data(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->layout.serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      uint32_t __data_length__ = this->data.size();
      memcpy(__outbuffer__ + __offset__, &__data_length__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->data.data(), __data_length__ * 4);
      __offset__ += __data_length__ * 4;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->layout.deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      uint32_t __data_length__ = 0; 
      memcpy(&__data_length__, __inbuffer__ + __offset__, 4);
      this->data.resize(__data_length__); 
      __offset__ += 4;
      memcpy(this->data.data(), __inbuffer__ + __offset__, __data_length__ * 4);
      __offset__ += __data_length__ * 4;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += this->layout.serializedLength();
      uint32_t __data_length__ = this->data.size();
      __length_ += 4;
      __length_ += __data_length__ * 4;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        std::stringstream __stream_layout__;
        mros::json::Writer::Write(__root__["layout"], __stream_layout__);
        ((UInt32MultiArray*)__msg__)->layout.parse(&((UInt32MultiArray*)__msg__)->layout, __stream_layout__.str());
        mros::json::Array __array_data__ = mros::json::Array(__root__["data"]);
        ((UInt32MultiArray*)__msg__)->data.resize(__array_data__.Size()); 
        for( uint32_t i = 0; i < __array_data__.Size(); i++) {
          ((UInt32MultiArray*)__msg__)->data[i] = mros::json::Number(__root__["data"][i]).Value();
        }
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      __echo__ += "\"layout\":";
      __echo__ += this->layout.echo();
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

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/UInt32MultiArray"; }
    static std::string getTypeStatic(){ return "std_msgs/UInt32MultiArray"; }
    virtual const std::string getMD5() const { return "4d6a180abc9be191b96a7eda6c8a233d"; }
    static std::string getMD5Static(){ return "4d6a180abc9be191b96a7eda6c8a233d"; }
    virtual const std::string getDefinition() const { return "std_msgs/MultiArrayLayout layout\nuint32[] data\n================================================================================\nMSG: std_msgs/MultiArrayLayout\nstd_msgs/MultiArrayDimension[] dim\nuint32 data_offset\n================================================================================\nMSG: std_msgs/MultiArrayDimension\nstring label\nuint32 size\nuint32 stride\n"; }
    static std::string getDefinitionStatic(){ return "std_msgs/MultiArrayLayout layout\nuint32[] data\n================================================================================\nMSG: std_msgs/MultiArrayLayout\nstd_msgs/MultiArrayDimension[] dim\nuint32 data_offset\n================================================================================\nMSG: std_msgs/MultiArrayDimension\nstring label\nuint32 size\nuint32 stride\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::UInt32MultiArray> Ptr;
    typedef std::shared_ptr<mros::std_msgs::UInt32MultiArray const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::UInt32MultiArray> UInt32MultiArrayPtr;
typedef std::shared_ptr<mros::std_msgs::UInt32MultiArray const> UInt32MultiArrayConstPtr;

}
}
#endif
