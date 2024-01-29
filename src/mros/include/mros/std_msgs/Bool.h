#ifndef _MROS_MSG_std_msgs_Bool_h
#define _MROS_MSG_std_msgs_Bool_h

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

  class Bool : public mros::Msg
  {
    public:
      typedef bool _data_type;
      _data_type data;



    Bool():
      data(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      union {
        bool real__;
        uint8_t base__;
      } __u_data__;
      __u_data__.real__ = this->data;
      *(__outbuffer__ + __offset__ + 0) = (__u_data__.base__ >> (8 * 0)) & 0xFF;
      __offset__ += 1;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      union {
        bool real__;
        uint8_t base__;
      } __u_data__;
      __u_data__.base__ = 0;
      __u_data__.base__ |= ((uint8_t) (*(__inbuffer__ + __offset__ + 0))) << (8 * 0);
      this->data = __u_data__.real__;
      __offset__ += 1;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += 1;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        ((Bool*)__msg__)->data = mros::json::Number(__root__["data"]).Value();
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
    virtual const std::string getType() const { return "std_msgs/Bool"; }
    static std::string getTypeStatic(){ return "std_msgs/Bool"; }
    virtual const std::string getMD5() const { return "8b94c1b53db61fb6aed406028ad6332a"; }
    static std::string getMD5Static(){ return "8b94c1b53db61fb6aed406028ad6332a"; }
    virtual const std::string getDefinition() const { return "bool data\n"; }
    static std::string getDefinitionStatic(){ return "bool data\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::Bool> Ptr;
    typedef std::shared_ptr<mros::std_msgs::Bool const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::Bool> BoolPtr;
typedef std::shared_ptr<mros::std_msgs::Bool const> BoolConstPtr;

}
}
#endif
