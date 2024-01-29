#ifndef _MROS_MSG_std_msgs_Time_h
#define _MROS_MSG_std_msgs_Time_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/os/time.h"

namespace mros
{
namespace std_msgs
{

  class Time : public mros::Msg
  {
    public:
      typedef mros::Time _data_type;
      _data_type data;



    Time():
      data()
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(__outbuffer__ + __offset__, &data.sec, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, &data.nsec, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(&data.sec, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      memcpy(&data.nsec, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
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
        ((Time*)__msg__)->data.sec = mros::json::Number(__root__["data"]["sec"]).Value();
        ((Time*)__msg__)->data.nsec = mros::json::Number(__root__["data"]["nsec"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      std::stringstream ss_data;
      ss_data << "\"data\":{\"sec\":" << data.sec;
      ss_data << ",\"nsec\":" << data.nsec << "}";
      __echo__ += ss_data.str();
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/Time"; }
    static std::string getTypeStatic(){ return "std_msgs/Time"; }
    virtual const std::string getMD5() const { return "cd7166c74c552c311fbcc2fe5a7bc289"; }
    static std::string getMD5Static(){ return "cd7166c74c552c311fbcc2fe5a7bc289"; }
    virtual const std::string getDefinition() const { return "time data\n"; }
    static std::string getDefinitionStatic(){ return "time data\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::Time> Ptr;
    typedef std::shared_ptr<mros::std_msgs::Time const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::Time> TimePtr;
typedef std::shared_ptr<mros::std_msgs::Time const> TimeConstPtr;

}
}
#endif
