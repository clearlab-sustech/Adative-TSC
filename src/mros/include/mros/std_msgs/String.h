#ifndef _MROS_MSG_std_msgs_String_h
#define _MROS_MSG_std_msgs_String_h

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

  class String : public mros::Msg
  {
    public:
      typedef std::string _data_type;
      _data_type data;



    String():
      data("")
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      uint32_t __length_data__ = this->data.size();
      memcpy(__outbuffer__ + __offset__, &__length_data__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->data.c_str(), __length_data__);
      __offset__ += __length_data__;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      uint32_t __length_data__ = 0;
      memcpy(&__length_data__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->data.assign((const char*)&__inbuffer__[__offset__], __length_data__);
      __offset__ += __length_data__;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      uint32_t __length_data__ = this->data.size();
      __length_ += 4;
      __length_ += __length_data__;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        ((String*)__msg__)->data = mros::json::String(__root__["data"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      __echo__ += "\"data\":";
      __echo__ += matchString2(data);
      __echo__ += "";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/String"; }
    static std::string getTypeStatic(){ return "std_msgs/String"; }
    virtual const std::string getMD5() const { return "992ce8a1687cec8c8bd883ec73ca41d1"; }
    static std::string getMD5Static(){ return "992ce8a1687cec8c8bd883ec73ca41d1"; }
    virtual const std::string getDefinition() const { return "string data\n"; }
    static std::string getDefinitionStatic(){ return "string data\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::String> Ptr;
    typedef std::shared_ptr<mros::std_msgs::String const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::String> StringPtr;
typedef std::shared_ptr<mros::std_msgs::String const> StringConstPtr;

}
}
#endif
