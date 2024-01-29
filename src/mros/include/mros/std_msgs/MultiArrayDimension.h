#ifndef _MROS_MSG_std_msgs_MultiArrayDimension_h
#define _MROS_MSG_std_msgs_MultiArrayDimension_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"

namespace mros
{
namespace std_msgs
{

  class MultiArrayDimension : public mros::Msg
  {
    public:
      typedef std::string _label_type;
      _label_type label;
      typedef uint32_t _size_type;
      _size_type size;
      typedef uint32_t _stride_type;
      _stride_type stride;



    MultiArrayDimension():
      label(""),
      size(0),
      stride(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      uint32_t __length_label__ = this->label.size();
      memcpy(__outbuffer__ + __offset__, &__length_label__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->label.c_str(), __length_label__);
      __offset__ += __length_label__;
      memcpy(__outbuffer__ + __offset__, &size, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, &stride, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      uint32_t __length_label__ = 0;
      memcpy(&__length_label__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->label.assign((const char*)&__inbuffer__[__offset__], __length_label__);
      __offset__ += __length_label__;
      memcpy(&size, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      memcpy(&stride, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      uint32_t __length_label__ = this->label.size();
      __length_ += 4;
      __length_ += __length_label__;
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
        ((MultiArrayDimension*)__msg__)->label = mros::json::String(__root__["label"]).Value();
        ((MultiArrayDimension*)__msg__)->size = mros::json::Number(__root__["size"]).Value();
        ((MultiArrayDimension*)__msg__)->stride = mros::json::Number(__root__["stride"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      __echo__ += "\"label\":";
      __echo__ += matchString2(label);
      __echo__ += ",";
      std::stringstream ss_size; ss_size << "\"size\":" << size <<",";
      __echo__ += ss_size.str();
      std::stringstream ss_stride; ss_stride << "\"stride\":" << stride <<"";
      __echo__ += ss_stride.str();
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/MultiArrayDimension"; }
    static std::string getTypeStatic(){ return "std_msgs/MultiArrayDimension"; }
    virtual const std::string getMD5() const { return "4cd0c83a8683deae40ecdac60e53bfa8"; }
    static std::string getMD5Static(){ return "4cd0c83a8683deae40ecdac60e53bfa8"; }
    virtual const std::string getDefinition() const { return "string label\nuint32 size\nuint32 stride\n"; }
    static std::string getDefinitionStatic(){ return "string label\nuint32 size\nuint32 stride\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::MultiArrayDimension> Ptr;
    typedef std::shared_ptr<mros::std_msgs::MultiArrayDimension const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::MultiArrayDimension> MultiArrayDimensionPtr;
typedef std::shared_ptr<mros::std_msgs::MultiArrayDimension const> MultiArrayDimensionConstPtr;

}
}
#endif
