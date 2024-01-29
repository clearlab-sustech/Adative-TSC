#ifndef _MROS_SRV_std_srvs_Empty_h
#define _MROS_SRV_std_srvs_Empty_h
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

static const char EMPTY[] = "std_srvs/Empty";

  class EmptyRequest : public mros::Msg
  {
    private:
      typedef uint32_t ___id___type;
      ___id___type __id__;

    public:



    EmptyRequest()
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
    virtual const std::string getType() const { return EMPTY; }
    static std::string getTypeStatic(){ return EMPTY; }
    virtual const std::string getMD5() const { return "d41d8cd98f00b204e9800998ecf8427e"; }
    static std::string getMD5Static(){ return "d41d8cd98f00b204e9800998ecf8427e"; }
    virtual const std::string getDefinition() const { return ""; }
    static std::string getDefinitionStatic(){ return ""; }
    static bool hasHeader(){ return false; }
    uint32_t const getID() const { return this->__id__; }
    void setID(uint32_t id){ this->__id__ = id; }
    typedef std::shared_ptr<mros::std_srvs::EmptyRequest> Ptr;
    typedef std::shared_ptr<mros::std_srvs::EmptyRequest const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_srvs::EmptyRequest> EmptyRequestPtr;
typedef std::shared_ptr<mros::std_srvs::EmptyRequest const> EmptyRequestConstPtr;

  class EmptyResponse : public mros::Msg
  {
    private:
      typedef uint32_t ___id___type;
      ___id___type __id__;

    public:



    EmptyResponse()
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
    virtual const std::string getType() const { return EMPTY; }
    static std::string getTypeStatic(){ return EMPTY; }
    virtual const std::string getMD5() const { return "d41d8cd98f00b204e9800998ecf8427e"; }
    static std::string getMD5Static(){ return "d41d8cd98f00b204e9800998ecf8427e"; }
    virtual const std::string getDefinition() const { return ""; }
    static std::string getDefinitionStatic(){ return ""; }
    static bool hasHeader(){ return false; }
    uint32_t const getID() const { return this->__id__; }
    void setID(uint32_t id){ this->__id__ = id; }
    typedef std::shared_ptr<mros::std_srvs::EmptyResponse> Ptr;
    typedef std::shared_ptr<mros::std_srvs::EmptyResponse const> ConstPtr;
  };
typedef std::shared_ptr<mros::std_srvs::EmptyResponse> EmptyResponsePtr;
typedef std::shared_ptr<mros::std_srvs::EmptyResponse const> EmptyResponseConstPtr;

  class Empty {
    public:
    typedef EmptyRequest Request;
    typedef EmptyResponse Response;
    Request request;
    Response response;
  };

}
}
#endif
