#ifndef _MROS_MSG_mros_msgs_Log_h
#define _MROS_MSG_mros_msgs_Log_h

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"
#include "mros/std_msgs/Header.h"

namespace mros
{
namespace mros_msgs
{

  class Log : public mros::Msg
  {
    public:
      typedef mros::std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _level_type;
      _level_type level;
      typedef std::string _name_type;
      _name_type name;
      typedef std::string _msg_type;
      _msg_type msg;
      typedef std::string _file_type;
      _file_type file;
      typedef std::string _function_type;
      _function_type function;
      typedef uint32_t _line_type;
      _line_type line;
      typedef std::string _topics_type;
      std::vector<_topics_type> topics;
      typedef uint64_t _pid_type;
      _pid_type pid;
      typedef uint64_t _tid_type;
      _tid_type tid;

      #if defined(_WIN32) && defined(DEBUG)
        #undef DEBUG
      #endif
      #if defined(_WIN32) && defined(INFO)
        #undef INFO
      #endif
      #if defined(_WIN32) && defined(WARN)
        #undef WARN
      #endif
      #if defined(_WIN32) && defined(ERROR)
        #undef ERROR
      #endif
      #if defined(_WIN32) && defined(FATAL)
        #undef FATAL
      #endif

      enum { DEBUG = 1  };
      enum { INFO = 2   };
      enum { WARN = 4   };
      enum { ERROR = 8  };
      enum { FATAL = 16  };

    Log():
      header(),
      level(0),
      name(""),
      msg(""),
      file(""),
      function(""),
      line(0),
      topics(0),
      pid(0),
      tid(0)
    {
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.serialize(__outbuffer__ + __offset__, __count__ - __offset__);
      memcpy(__outbuffer__ + __offset__, &level, 1);
      __offset__ += 1;
      uint32_t __length_name__ = this->name.size();
      memcpy(__outbuffer__ + __offset__, &__length_name__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->name.c_str(), __length_name__);
      __offset__ += __length_name__;
      uint32_t __length_msg__ = this->msg.size();
      memcpy(__outbuffer__ + __offset__, &__length_msg__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->msg.c_str(), __length_msg__);
      __offset__ += __length_msg__;
      uint32_t __length_file__ = this->file.size();
      memcpy(__outbuffer__ + __offset__, &__length_file__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->file.c_str(), __length_file__);
      __offset__ += __length_file__;
      uint32_t __length_function__ = this->function.size();
      memcpy(__outbuffer__ + __offset__, &__length_function__, 4);
      __offset__ += 4;
      memcpy(__outbuffer__ + __offset__, this->function.c_str(), __length_function__);
      __offset__ += __length_function__;
      memcpy(__outbuffer__ + __offset__, &line, 4);
      __offset__ += 4;
      uint32_t __topics_length__ = this->topics.size();
      memcpy(__outbuffer__ + __offset__, &__topics_length__, 4);
      __offset__ += 4;
      for( uint32_t i = 0; i < __topics_length__; i++) {
        uint32_t __length_topicsi__ = this->topics[i].size();
        memcpy(__outbuffer__ + __offset__, &__length_topicsi__, 4);
        __offset__ += 4;
        memcpy(__outbuffer__ + __offset__, this->topics[i].c_str(), __length_topicsi__);
        __offset__ += __length_topicsi__;
      }
      memcpy(__outbuffer__ + __offset__, &pid, 8);
      __offset__ += 8;
      memcpy(__outbuffer__ + __offset__, &tid, 8);
      __offset__ += 8;
      return __offset__;
    }

    virtual int deserialize(unsigned char *__inbuffer__, int __count__)
    {
      (void)__count__;
      int __offset__ = 0;
      __offset__ += this->header.deserialize(__inbuffer__ + __offset__, __count__ - __offset__);
      memcpy(&level, __inbuffer__ + __offset__, 1);
      __offset__ += 1;
      uint32_t __length_name__ = 0;
      memcpy(&__length_name__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->name.assign((const char*)&__inbuffer__[__offset__], __length_name__);
      __offset__ += __length_name__;
      uint32_t __length_msg__ = 0;
      memcpy(&__length_msg__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->msg.assign((const char*)&__inbuffer__[__offset__], __length_msg__);
      __offset__ += __length_msg__;
      uint32_t __length_file__ = 0;
      memcpy(&__length_file__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->file.assign((const char*)&__inbuffer__[__offset__], __length_file__);
      __offset__ += __length_file__;
      uint32_t __length_function__ = 0;
      memcpy(&__length_function__, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      this->function.assign((const char*)&__inbuffer__[__offset__], __length_function__);
      __offset__ += __length_function__;
      memcpy(&line, __inbuffer__ + __offset__, 4);
      __offset__ += 4;
      uint32_t __topics_length__ = 0; 
      memcpy(&__topics_length__, __inbuffer__ + __offset__, 4);
      this->topics.resize(__topics_length__); 
      __offset__ += 4;
      for( uint32_t i = 0; i < __topics_length__; i++) {
        uint32_t __length_topicsi__ = 0;
        memcpy(&__length_topicsi__, __inbuffer__ + __offset__, 4);
        __offset__ += 4;
        this->topics[i].assign((const char*)&__inbuffer__[__offset__], __length_topicsi__);
        __offset__ += __length_topicsi__;
      }
      memcpy(&pid, __inbuffer__ + __offset__, 8);
      __offset__ += 8;
      memcpy(&tid, __inbuffer__ + __offset__, 8);
      __offset__ += 8;
      return __offset__;
    }

    virtual int serializedLength() const
    {
      int __length_ = 0;
      __length_ += this->header.serializedLength();
      __length_ += 1;
      uint32_t __length_name__ = this->name.size();
      __length_ += 4;
      __length_ += __length_name__;
      uint32_t __length_msg__ = this->msg.size();
      __length_ += 4;
      __length_ += __length_msg__;
      uint32_t __length_file__ = this->file.size();
      __length_ += 4;
      __length_ += __length_file__;
      uint32_t __length_function__ = this->function.size();
      __length_ += 4;
      __length_ += __length_function__;
      __length_ += 4;
      uint32_t __topics_length__ = this->topics.size();
      __length_ += 4;
      for( uint32_t i = 0; i < __topics_length__; i++) {
        uint32_t __length_topicsi__ = this->topics[i].size();
        __length_ += 4;
        __length_ += __length_topicsi__;
      }
      __length_ += 8;
      __length_ += 8;
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
        ((Log*)__msg__)->header.parse(&((Log*)__msg__)->header, __stream_header__.str());
        ((Log*)__msg__)->level = mros::json::Number(__root__["level"]).Value();
        ((Log*)__msg__)->name = mros::json::String(__root__["name"]).Value();
        ((Log*)__msg__)->msg = mros::json::String(__root__["msg"]).Value();
        ((Log*)__msg__)->file = mros::json::String(__root__["file"]).Value();
        ((Log*)__msg__)->function = mros::json::String(__root__["function"]).Value();
        ((Log*)__msg__)->line = mros::json::Number(__root__["line"]).Value();
        mros::json::Array __array_topics__ = mros::json::Array(__root__["topics"]);
        ((Log*)__msg__)->topics.resize(__array_topics__.Size()); 
        for( uint32_t i = 0; i < __array_topics__.Size(); i++) {
          ((Log*)__msg__)->topics[i] = mros::json::String(__root__["topics"][i]).Value();
        }
        ((Log*)__msg__)->pid = mros::json::Number(__root__["pid"]).Value();
        ((Log*)__msg__)->tid = mros::json::Number(__root__["tid"]).Value();
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
      std::stringstream ss_level; ss_level << "\"level\":" << (int16_t)level <<",";
      __echo__ += ss_level.str();
      __echo__ += "\"name\":";
      __echo__ += matchString2(name);
      __echo__ += ",";
      __echo__ += "\"msg\":";
      __echo__ += matchString2(msg);
      __echo__ += ",";
      __echo__ += "\"file\":";
      __echo__ += matchString2(file);
      __echo__ += ",";
      __echo__ += "\"function\":";
      __echo__ += matchString2(function);
      __echo__ += ",";
      std::stringstream ss_line; ss_line << "\"line\":" << line <<",";
      __echo__ += ss_line.str();
      uint32_t __topics_length__ = this->topics.size();
      __echo__ += "\"topics\":[";
      for( uint32_t i = 0; i < __topics_length__; i++) {
        if( i == (__topics_length__ - 1)) {
          __echo__ += matchString2(topics[i]);
          __echo__ += "";
        } else {
          __echo__ += matchString2(topics[i]);
          __echo__ += ",";
        }
      }
      __echo__ += "],";
      std::stringstream ss_pid; ss_pid << "\"pid\":" << pid <<",";
      __echo__ += ss_pid.str();
      std::stringstream ss_tid; ss_tid << "\"tid\":" << tid <<"";
      __echo__ += ss_tid.str();
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return &header; }
    virtual const std::string getType() const { return "mros_msgs/Log"; }
    static std::string getTypeStatic(){ return "mros_msgs/Log"; }
    virtual const std::string getMD5() const { return "c455707f8997b806ee5cc58566a8ed1c"; }
    static std::string getMD5Static(){ return "c455707f8997b806ee5cc58566a8ed1c"; }
    virtual const std::string getDefinition() const { return "byte DEBUG=1\nbyte INFO=2\nbyte WARN=4\nbyte ERROR=8\nbyte FATAL=16\nstd_msgs/Header header\nbyte level\nstring name\nstring msg\nstring file\nstring function\nuint32 line\nstring[] topics\nuint64 pid\nuint64 tid\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static std::string getDefinitionStatic(){ return "byte DEBUG=1\nbyte INFO=2\nbyte WARN=4\nbyte ERROR=8\nbyte FATAL=16\nstd_msgs/Header header\nbyte level\nstring name\nstring msg\nstring file\nstring function\nuint32 line\nstring[] topics\nuint64 pid\nuint64 tid\n================================================================================\nMSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n"; }
    static bool hasHeader(){ return true; }
    typedef std::shared_ptr<mros::mros_msgs::Log> Ptr;
    typedef std::shared_ptr<mros::mros_msgs::Log const> ConstPtr;
  };
typedef std::shared_ptr<mros::mros_msgs::Log> LogPtr;
typedef std::shared_ptr<mros::mros_msgs::Log const> LogConstPtr;

}
}
#endif
