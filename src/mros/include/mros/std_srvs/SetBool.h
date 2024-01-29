#ifndef _MROS_SRV_std_srvs_SetBool_h
#define _MROS_SRV_std_srvs_SetBool_h
#include <stdint.h>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mros/os/msg.h"

namespace mros
{
namespace std_srvs
{

static const char SETBOOL[] = "std_srvs/SetBool";

  class SetBoolRequest : public mros::Msg
  {
    private:
      typedef uint32_t ___id___type;
      ___id___type __id__;

    public:
      typedef bool _data_type;
      _data_type data;



    SetBoolRequest():
      data(0)
    {
      this->__id__ = 0;
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      *(__outbuffer__ + __offset__ + 0) = (this->__id__ >> (8 * 0)) & 0xFF;
      *(__outbuffer__ + __offset__ + 1) = (this->__id__ >> (8 * 1)) & 0xFF;
      *(__outbuffer__ + __offset__ + 2) = (this->__id__ >> (8 * 2)) & 0xFF;
      *(__outbuffer__ + __offset__ + 3) = (this->__id__ >> (8 * 3)) & 0xFF;
      __offset__ += 4;
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
      this->__id__ =  ((uint32_t) (*(__inbuffer__ + __offset__)));
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 1))) << (8 * 1);
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 2))) << (8 * 2);
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 3))) << (8 * 3);
      __offset__ += 4;
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
      __length_ += 4;
      __length_ += 1;
      return __length_;
    }

    void parse(Msg* __msg__, const std::string& __json__)
    {
      try {
        mros::json::Object __root__;
        std::istringstream __stream__(__json__);
        mros::json::Reader::Read(__root__, __stream__);
        ((SetBoolRequest*)__msg__)->data = mros::json::Number(__root__["data"]).Value();
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
    virtual const std::string getType() const { return SETBOOL; }
    static std::string getTypeStatic(){ return SETBOOL; }
    virtual const std::string getMD5() const { return "09fb03525b03e7ea1fd3992bafd87e16"; }
    static std::string getMD5Static(){ return "09fb03525b03e7ea1fd3992bafd87e16"; }
    virtual const std::string getDefinition() const { return "bool data\n"; }
    static std::string getDefinitionStatic(){ return "bool data\n"; }
    static bool hasHeader(){ return false; }
    uint32_t const getID() const { return this->__id__; }
    void setID(uint32_t id){ this->__id__ = id; }
    typedef std::shared_ptr<mros::std_srvs::SetBoolRequest> Ptr;
    typedef std::shared_ptr<mros::std_srvs::SetBoolRequest const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_srvs::SetBoolRequest> SetBoolRequestPtr;
typedef std::shared_ptr<mros::std_srvs::SetBoolRequest const> SetBoolRequestConstPtr;

  class SetBoolResponse : public mros::Msg
  {
    private:
      typedef uint32_t ___id___type;
      ___id___type __id__;

    public:
      typedef bool _success_type;
      _success_type success;
      typedef std::string _message_type;
      _message_type message;



    SetBoolResponse():
      success(0),
      message("")
    {
      this->__id__ = 0;
    }

    virtual int serialize(unsigned char *__outbuffer__, int __count__) const
    {
      (void)__count__;
      int __offset__ = 0;
      *(__outbuffer__ + __offset__ + 0) = (this->__id__ >> (8 * 0)) & 0xFF;
      *(__outbuffer__ + __offset__ + 1) = (this->__id__ >> (8 * 1)) & 0xFF;
      *(__outbuffer__ + __offset__ + 2) = (this->__id__ >> (8 * 2)) & 0xFF;
      *(__outbuffer__ + __offset__ + 3) = (this->__id__ >> (8 * 3)) & 0xFF;
      __offset__ += 4;
      union {
        bool real__;
        uint8_t base__;
      } __u_success__;
      __u_success__.real__ = this->success;
      *(__outbuffer__ + __offset__ + 0) = (__u_success__.base__ >> (8 * 0)) & 0xFF;
      __offset__ += 1;
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
      this->__id__ =  ((uint32_t) (*(__inbuffer__ + __offset__)));
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 1))) << (8 * 1);
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 2))) << (8 * 2);
      this->__id__ |= ((uint32_t) (*(__inbuffer__ + __offset__ + 3))) << (8 * 3);
      __offset__ += 4;
      union {
        bool real__;
        uint8_t base__;
      } __u_success__;
      __u_success__.base__ = 0;
      __u_success__.base__ |= ((uint8_t) (*(__inbuffer__ + __offset__ + 0))) << (8 * 0);
      this->success = __u_success__.real__;
      __offset__ += 1;
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
      __length_ += 4;
      __length_ += 1;
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
        ((SetBoolResponse*)__msg__)->success = mros::json::Number(__root__["success"]).Value();
        ((SetBoolResponse*)__msg__)->message = mros::json::String(__root__["message"]).Value();
      } catch (const std::exception& e) {
        std::cout << "Caught mros::json::Exception: " << e.what() << std::endl;
      }
    }
    virtual const std::string echo() const
    {
      std::string __echo__ = "{";
      std::stringstream ss_success; ss_success << "\"success\":" << success <<",";
      __echo__ += ss_success.str();
      __echo__ += "\"message\":";
      __echo__ += matchString2(message);
      __echo__ += "";
      __echo__ += "}";
      return __echo__;
    }

    virtual const Msg* getHeader() const { return nullptr; }
    virtual const std::string getType() const { return SETBOOL; }
    static std::string getTypeStatic(){ return SETBOOL; }
    virtual const std::string getMD5() const { return "09fb03525b03e7ea1fd3992bafd87e16"; }
    static std::string getMD5Static(){ return "09fb03525b03e7ea1fd3992bafd87e16"; }
    virtual const std::string getDefinition() const { return "bool success\nstring message\n"; }
    static std::string getDefinitionStatic(){ return "bool success\nstring message\n"; }
    static bool hasHeader(){ return false; }
    uint32_t const getID() const { return this->__id__; }
    void setID(uint32_t id){ this->__id__ = id; }
    typedef std::shared_ptr<mros::std_srvs::SetBoolResponse> Ptr;
    typedef std::shared_ptr<mros::std_srvs::SetBoolResponse const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_srvs::SetBoolResponse> SetBoolResponsePtr;
typedef std::shared_ptr<mros::std_srvs::SetBoolResponse const> SetBoolResponseConstPtr;

  class SetBool {
    public:
    typedef SetBoolRequest Request;
    typedef SetBoolResponse Response;
    Request request;
    Response response;
  };

}
}
#endif
