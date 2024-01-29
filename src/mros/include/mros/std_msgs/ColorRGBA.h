#ifndef _MROS_MSG_std_msgs_ColorRGBA_h
#define _MROS_MSG_std_msgs_ColorRGBA_h

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

  class ColorRGBA : public mros::Msg
  {
    public:
      typedef float _r_type;
      _r_type r;
      typedef float _g_type;
      _g_type g;
      typedef float _b_type;
      _b_type b;
      typedef float _a_type;
      _a_type a;



    ColorRGBA():
      r(0),
      g(0),
      b(0),
      a(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(__outbuffer__ + __offset__, &r, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, &g, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, &b, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, &a, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      memcpy(&r, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      memcpy(&g, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      memcpy(&b, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      memcpy(&a, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += 4;
      __length_ += 4;
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
        ((ColorRGBA*)__msg__)->r = mros::json::Number(__root__["r"]).Value();
        ((ColorRGBA*)__msg__)->g = mros::json::Number(__root__["g"]).Value();
        ((ColorRGBA*)__msg__)->b = mros::json::Number(__root__["b"]).Value();
        ((ColorRGBA*)__msg__)->a = mros::json::Number(__root__["a"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      std::stringstream ss_r; ss_r << "\"r\":" << r <<",";
      __echo__ += ss_r.str();
      std::stringstream ss_g; ss_g << "\"g\":" << g <<",";
      __echo__ += ss_g.str();
      std::stringstream ss_b; ss_b << "\"b\":" << b <<",";
      __echo__ += ss_b.str();
      std::stringstream ss_a; ss_a << "\"a\":" << a <<"";
      __echo__ += ss_a.str();
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return "std_msgs/ColorRGBA"; }
    static std::string getTypeStatic(){ return "std_msgs/ColorRGBA"; }
    virtual const std::string getMD5() const { return "a29a96539573343b1310c73607334b00"; }
    static std::string getMD5Static(){ return "a29a96539573343b1310c73607334b00"; }
    virtual const std::string getDefinition() const { return "float32 r\nfloat32 g\nfloat32 b\nfloat32 a\n"; }
    static std::string getDefinitionStatic(){ return "float32 r\nfloat32 g\nfloat32 b\nfloat32 a\n"; }
    static bool hasHeader(){ return false; }
    typedef std::shared_ptr<mros::std_msgs::ColorRGBA> Ptr;
    typedef std::shared_ptr<mros::std_msgs::ColorRGBA const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_msgs::ColorRGBA> ColorRGBAPtr;
typedef std::shared_ptr<mros::std_msgs::ColorRGBA const> ColorRGBAConstPtr;

}
}
#endif
