#ifndef _MROS_MSG_std_msgs_UInt32_h
#define _MROS_MSG_std_msgs_UInt32_h

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

  class UInt32 : public mros::Msg
  {
    public:
      typedef uint32_t _data_type;
      _data_type data;



    UInt32():
      data(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(__outbuffer__ + __offset__, &data, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(&data, __inbuffer__ + __offset__, 4);
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
        ((UInt32*)__msg__)->data = mros::json::Number(__root__["data"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      std::stringstream ss_data; ss_data << "\"data\":" << data <<"";
      __echo__ += ss_data.str();
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/UInt32"; }
    static std::string getTypeStatic(){ return "std_msgs/UInt32"; }
    virtual const std::string getMD5() const { return "304a39449588c7f8ce2df6e8001c5fce"; }
    static std::string getMD5Static(){ return "304a39449588c7f8ce2df6e8001c5fce"; }
    virtual const std::string getDefinition() const { return "uint32 data\n"; }
    static std::string getDefinitionStatic(){ return "uint32 data\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::UInt32> Ptr;
    typedef std::shared_ptr<mros::std_msgs::UInt32 const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::UInt32> UInt32Ptr;
typedef std::shared_ptr<mros::std_msgs::UInt32 const> UInt32ConstPtr;

}
}
#endif
