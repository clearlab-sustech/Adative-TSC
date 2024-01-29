#ifndef _MROS_MSG_std_msgs_Int16_h
#define _MROS_MSG_std_msgs_Int16_h

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

  class Int16 : public mros::Msg
  {
    public:
      typedef int16_t _data_type;
      _data_type data;



    Int16():
      data(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(__outbuffer__ + __offset__, &data, 2);
      __offset__ += 2;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(&data, __inbuffer__ + __offset__, 2);
      __offset__ += 2;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += 2;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        ((Int16*)__msg__)->data = mros::json::Number(__root__["data"]).Value();
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
    virtual const std::string getType() const { return "std_msgs/Int16"; }
    static std::string getTypeStatic(){ return "std_msgs/Int16"; }
    virtual const std::string getMD5() const { return "8524586e34fbd7cb1c08c5f5f1ca0e57"; }
    static std::string getMD5Static(){ return "8524586e34fbd7cb1c08c5f5f1ca0e57"; }
    virtual const std::string getDefinition() const { return "int16 data\n"; }
    static std::string getDefinitionStatic(){ return "int16 data\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::Int16> Ptr;
    typedef std::shared_ptr<mros::std_msgs::Int16 const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::Int16> Int16Ptr;
typedef std::shared_ptr<mros::std_msgs::Int16 const> Int16ConstPtr;

}
}
#endif
