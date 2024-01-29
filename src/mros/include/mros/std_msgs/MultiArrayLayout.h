#ifndef _MROS_MSG_std_msgs_MultiArrayLayout_h
#define _MROS_MSG_std_msgs_MultiArrayLayout_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/std_msgs/MultiArrayDimension.h"

namespace mros
{
namespace std_msgs
{

  class MultiArrayLayout : public mros::Msg
  {
    public:
      typedef mros::std_msgs::MultiArrayDimension _dim_type;
      std::vector<_dim_type> dim;
      typedef uint32_t _data_offset_type;
      _data_offset_type data_offset;



    MultiArrayLayout():
      dim(0),
      data_offset(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      uint32_t __dim_length__ = this->dim.size();
      memcpy(__outbuffer__ + __offset__, &__dim_length__, 4);
      __offset__ += 4;
      for( uint32_t i = 0; i < __dim_length__; i++) {
        __offset__ += this->dim[i].serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      }
      memcpy(__outbuffer__ + __offset__, &data_offset, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      uint32_t __dim_length__ = 0; 
      memcpy(&__dim_length__, __inbuffer__ + __offset__, 4);
      this->dim.resize(__dim_length__); 
      __offset__ += 4;
      for( uint32_t i = 0; i < __dim_length__; i++) {
        __offset__ += this->dim[i].deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      }
      memcpy(&data_offset, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      uint32_t __dim_length__ = this->dim.size();
      __length_ += 4;
      for( uint32_t i = 0; i < __dim_length__; i++) {
        __length_ += this->dim[i].serializedLength();
      }
      __length_ += 4;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        mros::json::Array __array_dim__ = mros::json::Array(__root__["dim"]);
        ((MultiArrayLayout*)__msg__)->dim.resize(__array_dim__.Size()); 
        for( uint32_t i = 0; i < __array_dim__.Size(); i++) {
          std::stringstream __stream_dim__;
          mros::json::Writer::Write(__root__["dim"][i], __stream_dim__);
          ((MultiArrayLayout*)__msg__)->dim[i].parse(&((MultiArrayLayout*)__msg__)->dim[i], __stream_dim__.str());
        }
        ((MultiArrayLayout*)__msg__)->data_offset = mros::json::Number(__root__["data_offset"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      uint32_t __dim_length__ = this->dim.size();
      __echo__ += "\"dim\":[";
      for( uint32_t i = 0; i < __dim_length__; i++) {
        if( i == (__dim_length__ - 1)) {
          __echo__ += this->dim[i].echo();
          __echo__ += "";
        } else {
          __echo__ += this->dim[i].echo();
          __echo__ += ",";
        }
      }
      __echo__ += "],";
      std::stringstream ss_data_offset; ss_data_offset << "\"data_offset\":" << data_offset <<"";
      __echo__ += ss_data_offset.str();
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/MultiArrayLayout"; }
    static std::string getTypeStatic(){ return "std_msgs/MultiArrayLayout"; }
    virtual const std::string getMD5() const { return "0fed2a11c13e11c5571b4e2a995a91a3"; }
    static std::string getMD5Static(){ return "0fed2a11c13e11c5571b4e2a995a91a3"; }
    virtual const std::string getDefinition() const { return "std_msgs/MultiArrayDimension[] dim\nuint32 data_offset\n================================================================================\nMSG: std_msgs/MultiArrayDimension\nstring label\nuint32 size\nuint32 stride\n"; }
    static std::string getDefinitionStatic(){ return "std_msgs/MultiArrayDimension[] dim\nuint32 data_offset\n================================================================================\nMSG: std_msgs/MultiArrayDimension\nstring label\nuint32 size\nuint32 stride\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::MultiArrayLayout> Ptr;
    typedef std::shared_ptr<mros::std_msgs::MultiArrayLayout const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::MultiArrayLayout> MultiArrayLayoutPtr;
typedef std::shared_ptr<mros::std_msgs::MultiArrayLayout const> MultiArrayLayoutConstPtr;

}
}
#endif
