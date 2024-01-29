#ifndef _MROS_MSG_std_msgs_Empty_h
#define _MROS_MSG_std_msgs_Empty_h

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

  class Empty : public mros::Msg
  {
    public:



    Empty()
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/Empty"; }
    static std::string getTypeStatic(){ return "std_msgs/Empty"; }
    virtual const std::string getMD5() const { return "d41d8cd98f00b204e9800998ecf8427e"; }
    static std::string getMD5Static(){ return "d41d8cd98f00b204e9800998ecf8427e"; }
    virtual const std::string getDefinition() const { return ""; }
    static std::string getDefinitionStatic(){ return ""; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::Empty> Ptr;
    typedef std::shared_ptr<mros::std_msgs::Empty const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::Empty> EmptyPtr;
typedef std::shared_ptr<mros::std_msgs::Empty const> EmptyConstPtr;

}
}
#endif
