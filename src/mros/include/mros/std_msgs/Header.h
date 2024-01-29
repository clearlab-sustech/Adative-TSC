#ifndef _MROS_MSG_std_msgs_Header_h
#define _MROS_MSG_std_msgs_Header_h

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

  class Header : public mros::Msg
  {
    public:
      typedef uint32_t _seq_type;
      _seq_type seq;
      typedef mros::Time _stamp_type;
      _stamp_type stamp;
      typedef std::string _frame_id_type;
      _frame_id_type frame_id;



    Header():
      seq(0),
      stamp(),
      frame_id("")
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(__outbuffer__ + __offset__, &seq, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, &stamp.sec, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, &stamp.nsec, 4);
      __offset__ += 4;
      uint32_t __length_frame_id__ = this->frame_id.size();
      memcpy(__outbuffer__ + __offset__, &__length_frame_id__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->frame_id.c_str(), __length_frame_id__);
      __offset__ += __length_frame_id__;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(&seq, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      memcpy(&stamp.sec, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      memcpy(&stamp.nsec, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      uint32_t __length_frame_id__ = 0;
      memcpy(&__length_frame_id__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->frame_id.assign((const char*)&__inbuffer__[__offset__], __length_frame_id__);
      __offset__ += __length_frame_id__;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += 4;
      __length_ += 4;
      __length_ += 4;
      uint32_t __length_frame_id__ = this->frame_id.size();
      __length_ += 4;
      __length_ += __length_frame_id__;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        ((Header*)__msg__)->seq = mros::json::Number(__root__["seq"]).Value();
        ((Header*)__msg__)->stamp.sec = mros::json::Number(__root__["stamp"]["sec"]).Value();
        ((Header*)__msg__)->stamp.nsec = mros::json::Number(__root__["stamp"]["nsec"]).Value();
        ((Header*)__msg__)->frame_id = mros::json::String(__root__["frame_id"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      std::stringstream ss_seq; ss_seq << "\"seq\":" << seq <<",";
      __echo__ += ss_seq.str();
      std::stringstream ss_stamp;
      ss_stamp << "\"stamp\":{\"sec\":" << stamp.sec;
      ss_stamp << ",\"nsec\":" << stamp.nsec << "},";
      __echo__ += ss_stamp.str();
      __echo__ += "\"frame_id\":";
      __echo__ += matchString2(frame_id);
      __echo__ += "";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/Header"; }
    static std::string getTypeStatic(){ return "std_msgs/Header"; }
    virtual const std::string getMD5() const { return "2176decaecbce78abc3b96ef049fabed"; }
    static std::string getMD5Static(){ return "2176decaecbce78abc3b96ef049fabed"; }
    virtual const std::string getDefinition() const { return "uint32 seq\ntime stamp\nstring frame_id\n"; }
    static std::string getDefinitionStatic(){ return "uint32 seq\ntime stamp\nstring frame_id\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::Header> Ptr;
    typedef std::shared_ptr<mros::std_msgs::Header const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::Header> HeaderPtr;
typedef std::shared_ptr<mros::std_msgs::Header const> HeaderConstPtr;

}
}
#endif
