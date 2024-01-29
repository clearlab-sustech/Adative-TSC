#ifndef _MROS_MSG_diagnostic_msgs_DiagnosticValue_h
#define _MROS_MSG_diagnostic_msgs_DiagnosticValue_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/std_msgs/Header.h"

namespace mros
{
namespace diagnostic_msgs
{

  class DiagnosticValue : public mros::Msg
  {
    public:
      typedef mros::std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _level_type;
      _level_type level;
      typedef std::string _name_type;
      _name_type name;
      typedef int32_t _code_type;
      _code_type code;
      typedef std::string _message_type;
      _message_type message;

      #if defined(_WIN32) && defined(OK)
        #undef OK
      #endif
      #if defined(_WIN32) && defined(WARN)
        #undef WARN
      #endif
      #if defined(_WIN32) && defined(ERROR)
        #undef ERROR
      #endif

      enum { OK = 0 };
      enum { WARN = 1 };
      enum { ERROR = 2 };

    DiagnosticValue():
      header(),
      level(0),
      name(""),
      code(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      memcpy(__outbuffer__ + __offset__, &level, 4);
      __offset__ += 4;
      uint32_t __length_name__ = this->name.size();
      memcpy(__outbuffer__ + __offset__, &__length_name__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->name.c_str(), __length_name__);
      __offset__ += __length_name__;
      memcpy(__outbuffer__ + __offset__, &code, 4);
      __offset__ += 4;
      uint32_t __length_message__ = this->message.size();
      memcpy(__outbuffer__ + __offset__, &__length_message__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->message.c_str(), __length_message__);
      __offset__ += __length_message__;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      memcpy(&level, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      uint32_t __length_name__ = 0;
      memcpy(&__length_name__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->name.assign((const char*)&__inbuffer__[__offset__], __length_name__);
      __offset__ += __length_name__;
      memcpy(&code, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      uint32_t __length_message__ = 0;
      memcpy(&__length_message__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->message.assign((const char*)&__inbuffer__[__offset__], __length_message__);
      __offset__ += __length_message__;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += this->header.serializedLength();
      __length_ += 4;
      uint32_t __length_name__ = this->name.size();
      __length_ += 4;
      __length_ += __length_name__;
      __length_ += 4;
      uint32_t __length_message__ = this->message.size();
      __length_ += 4;
      __length_ += __length_message__;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        std::stringstream __stream_header__;
        mros::json::Writer::Write(__root__["header"], __stream_header__);
        ((DiagnosticValue*)__msg__)->header.parse(&((DiagnosticValue*)__msg__)->header, __stream_header__.str());
        ((DiagnosticValue*)__msg__)->level = mros::json::Number(__root__["level"]).Value();
        ((DiagnosticValue*)__msg__)->name = mros::json::String(__root__["name"]).Value();
        ((DiagnosticValue*)__msg__)->code = mros::json::Number(__root__["code"]).Value();
        ((DiagnosticValue*)__msg__)->message = mros::json::String(__root__["message"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      __echo__ += "\"header\":";
      __echo__ += this->header.echo();
      __echo__ += ",";
      std::stringstream ss_level; ss_level << "\"level\":" << level <<",";
      __echo__ += ss_level.str();
      __echo__ += "\"name\":";
      __echo__ += matchString2(name);
      __echo__ += ",";
      std::stringstream ss_code; ss_code << "\"code\":" << code <<",";
      __echo__ += ss_code.str();
      __echo__ += "\"message\":";
      __echo__ += matchString2(message);
      __echo__ += "";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return &header; }
    virtual const std::string getType() const { return "diagnostic_msgs/DiagnosticValue"; }
    static std::string getTypeStatic(){ return "diagnostic_msgs/DiagnosticValue"; }
    virtual const std::string getMD5() const { return "879d5c821998ca0ab6e3520e8c63e5fc"; }
    static std::string getMD5Static(){ return "879d5c821998ca0ab6e3520e8c63e5fc"; }
    virtual const std::string getDefinition() const { return "std_msgs/Header header\nint32 OK=0\nint32 WARN=1\nint32 ERROR=2\nint32 level\nstring name\nint32 code\nstring message\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static std::string getDefinitionStatic(){ return "std_msgs/Header header\nint32 OK=0\nint32 WARN=1\nint32 ERROR=2\nint32 level\nstring name\nint32 code\nstring message\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static bool hasHeader(){ return true; }
    typedef std::shared_ptr<mros::diagnostic_msgs::DiagnosticValue> Ptr;
    typedef std::shared_ptr<mros::diagnostic_msgs::DiagnosticValue const> ConstPtr;
  };
typedef std::shared_ptr<mros::diagnostic_msgs::DiagnosticValue> DiagnosticValuePtr;
typedef std::shared_ptr<mros::diagnostic_msgs::DiagnosticValue const> DiagnosticValueConstPtr;

}
}
#endif
