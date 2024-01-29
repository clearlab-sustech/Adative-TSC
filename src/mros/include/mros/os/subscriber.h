#ifndef MROS_SUBSCRIBER_H_
#define MROS_SUBSCRIBER_H_
#include <functional>
#include <mros/macros.h>
#include "mros/os/log.h"
#include "mros/mros_msgs/TopicInfo.h"
#include "mros/os/common.h"

namespace mroseprosima
{
namespace fastrtps
{
	class Participant;
	class Publisher;
	class Subscriber;
}
}

namespace mros
{
class SubListener;
class NodeHandleBase;

typedef enum {
    CallbackType0,
    CallbackType1,
    CallbackType2,
    CallbackType3,
} CallbackType;

/* Base class for objects subscribers. */
class MROS_DllAPI SubscriberBase
{
public:
  SubscriberBase();
  
  virtual ~SubscriberBase();
  
  virtual void callback(unsigned char *data, int count) = 0;
  virtual void intraProcess(const mros::MsgConstPtr& msg) {}
  virtual bool negotiated() { return negotiated_; }
  virtual std::string getMsgType() { return type_; }
  virtual std::string getMsgMD5() { return md5_; }
  virtual std::string getMsgDef() { return def_; }
  virtual void setCount(int32_t count) { count_ = count; }
  virtual int32_t getCount() { return count_; }
  virtual int32_t getQueueSize() { return queue_size_sub_; }
  virtual bool getReliable() { return reliable_sub_; }
  virtual bool getLatch() { return latch_sub_; }
  void setNegotiated(bool negotiated) { negotiated_ = negotiated; }
  uint32_t getTopicId() { return topic_id_; }
  void setTopicId(uint32_t id) { topic_id_ = id; }
  int getEndpointType() { return endpoint_; }
  void setEndpointType(int endpoint ) { endpoint_ = endpoint; }

  /**
   * \brief Disable or enable subscription of specific topic messages.
   *
   * Note: You cannot call this function in the callback specified by the Subscriber constructor,
   *       because it will cause the program to deadlock and block. But you can call it through an asynchronous task.
   *
   * \param enable Disable or enable subscription of specific topic messages.
   */
  virtual void setEnabled(bool enable);
  virtual bool getEnabled() {return enable_; }

  /**
   * \brief Set topic name. Subscribe to messages through this topic.
   *
   * Note: You cannot call this function in the callback specified by the Subscriber constructor,
   *       because it will cause the program to deadlock and block. But you can call it through an asynchronous task.
   *
   * \param topic_name topic name.
   */
  virtual void setTopic(const std::string & topic_name);

  /**
   * \brief Get topic name.
   *
   * \return std::string topic name.
   */
  virtual std::string getTopic();
  
  virtual bool subscribe(NodeHandleBase* nh, bool latch = false, bool reliable = false, int32_t queue_size = -1);

protected:
  NodeHandleBase* node_handle_;
  int endpoint_;
  uint32_t topic_id_;
  std::string topic_; ///< Datatype of the topic
  std::string type_; ///< Name of the topic
  std::string md5_; ///< md5sum of the topic
  std::string def_; ///< definition of the topic
  bool negotiated_; ///< negotiated
  bool srv_flag_; ///< Is it a service
  CallbackType cb_type_; ///< CallbackType
  bool enable_; ///< enabled
  int32_t count_; ///< message count
  bool latch_sub_; ///< message latch
  bool reliable_sub_; ///< message reliable
  int queue_size_sub_; ///< message queue_size
  int32_t intra_process_id1_; ///< intra_process_id for signal1
  int32_t intra_process_id2_; ///< intra_process_id for signal2
  
  std::shared_ptr<mroseprosima::fastrtps::Participant*> participant_; ///< participant

  std::shared_ptr<mroseprosima::fastrtps::Subscriber*> subscriber_; ///< subscriber
  
  friend class SubListener;
  SubListener* listener_; ///< subscriber's listener
};

/* Bound function subscriber. */
template<typename MsgT, typename ObjT = void>
class MROS_DllAPI Subscriber: public SubscriberBase
{
public:
  typedef void(ObjT::*CallbackT0)(const MsgT&);
  typedef void(ObjT::*CallbackT1)(const MsgT&, SubscriberBase*);
  typedef void(ObjT::*CallbackT2)(const std::shared_ptr<MsgT const>&);
  typedef void(ObjT::*CallbackT3)(const std::shared_ptr<MsgT const>&, SubscriberBase*);
  MsgT msg;

  Subscriber() = delete;
  Subscriber(std::string topic_name, CallbackT0 cb, ObjT* obj)
    : cb0_(cb)
    , obj_(obj) { 
      cb_type_ = CallbackType0;
      type_ = msg.getType();
      md5_ = msg.getMD5();
      def_ = msg.getDefinition();

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT0 cb, std::string type, std::string md5, std::string def, ObjT* obj) 
    : cb0_(cb)
    , obj_(obj) { 
      cb_type_ = CallbackType0;
      type_ = type;
      md5_ = type;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT1 cb, ObjT* obj)
    : cb1_(cb)
    , obj_(obj) { 
      cb_type_ = CallbackType1;
      type_ = msg.getType();
      md5_ = msg.getMD5();
      def_ = msg.getDefinition();

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT1 cb, std::string type, std::string md5, std::string def, ObjT* obj)
    : cb1_(cb)
    , obj_(obj) { 
      cb_type_ = CallbackType1;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT2 cb, ObjT* obj)
    : cb2_(cb)
    , obj_(obj) { 
      cb_type_ = CallbackType2;
      type_ = msg.getType();
      md5_ = msg.getMD5();
      def_ = msg.getDefinition();

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT2 cb, std::string type, std::string md5, std::string def, ObjT* obj)
    : cb2_(cb)
    , obj_(obj) { 
      cb_type_ = CallbackType2;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT3 cb, ObjT* obj)
    : cb3_(cb)
    , obj_(obj) { 
      cb_type_ = CallbackType3;
      type_ = msg.getType();
      md5_ = msg.getMD5();
      def_ = msg.getDefinition();

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT3 cb, std::string type, std::string md5, std::string def, ObjT* obj)
    : cb3_(cb)
    , obj_(obj) { 
      cb_type_ = CallbackType3;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  virtual void callback(unsigned char* data, int count)
  {
    if (!enable_)
      return;

    switch(cb_type_) {
      case CallbackType0: {
        MsgT tmsg;
        tmsg.callerid = mros::getCallerId(data, count);
        tmsg.deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
        (obj_->*cb0_)(tmsg);
        break;
      }
      case CallbackType1: {
        MsgT tmsg;
        tmsg.callerid = mros::getCallerId(data, count);
        tmsg.deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
        (obj_->*cb1_)(tmsg, this);
        break;
      }
      case CallbackType2: {
        std::shared_ptr<MsgT> tmsg = std::shared_ptr<MsgT>(new MsgT);
        tmsg->callerid = mros::getCallerId(data, count);
        tmsg->deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
        (obj_->*cb2_)(tmsg);
        break;
      }
      case CallbackType3: {
        std::shared_ptr<MsgT> tmsg = std::shared_ptr<MsgT>(new MsgT);
        tmsg->callerid = mros::getCallerId(data, count);
        tmsg->deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
        (obj_->*cb3_)(tmsg, this);
        break;
      }
      default: break;
    }
  }

  void intraProcess(const mros::MsgConstPtr& msg) {
    if (!enable_)
      return;

    switch(cb_type_) {
      case CallbackType0: {
        (obj_->*cb0_)((const MsgT&)*msg);
        break;
      }
      case CallbackType1: {
        (obj_->*cb1_)((const MsgT&)*msg, this);
        break;
      }
      case CallbackType2: {
        (obj_->*cb2_)((const std::shared_ptr<MsgT const>&)msg);
        break;
      }
      case CallbackType3: {
        (obj_->*cb3_)((const std::shared_ptr<MsgT const>&)msg, this);
        break;
      }
      default: break;
    }
  }

private:
  CallbackT0 cb0_;
  CallbackT1 cb1_;
  CallbackT2 cb2_;
  CallbackT3 cb3_;
  ObjT* obj_;
};

/* Standalone function subscriber. */
template<typename MsgT>
class MROS_DllAPI Subscriber<MsgT, void>: public SubscriberBase
{
public:
  typedef std::function<void(const MsgT&)> CallbackT0;
  typedef std::function<void(const MsgT&, SubscriberBase*)> CallbackT1;
  typedef std::function<void(const std::shared_ptr<MsgT const>&)> CallbackT2;
  typedef std::function<void(const std::shared_ptr<MsgT const>&, SubscriberBase*)> CallbackT3;

  MsgT msg;

  Subscriber() = delete;
  Subscriber(std::string topic_name, CallbackT0 cb)
    : cb0_(cb) { 
      cb_type_ = CallbackType0;
      type_ = msg.getType();
      md5_ = msg.getMD5();
      def_ = msg.getDefinition();

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT0 cb, std::string type, std::string md5, std::string def)
    : cb0_(cb) { 
      cb_type_ = CallbackType0;
      topic_ = topic_name;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }
  
  Subscriber(std::string topic_name, CallbackT1 cb)
    : cb1_(cb) { 
      cb_type_ = CallbackType1;
      type_ = msg.getType();
      md5_ = msg.getMD5();
      def_ = msg.getDefinition();

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT1 cb, std::string type, std::string md5, std::string def)
    : cb1_(cb) { 
      cb_type_ = CallbackType1;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }
  
  Subscriber(std::string topic_name, CallbackT2 cb)
    : cb2_(cb) { 
      cb_type_ = CallbackType2;
      type_ = msg.getType();
      md5_ = msg.getMD5();
      def_ = msg.getDefinition();

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT2 cb, std::string type, std::string md5, std::string def)
    : cb2_(cb) { 
      cb_type_ = CallbackType2;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }
  
  Subscriber(std::string topic_name, CallbackT3 cb)
    : cb3_(cb) { 
      cb_type_ = CallbackType3;
      type_ = msg.getType();
      md5_ = msg.getMD5();
      def_ = msg.getDefinition();

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT3 cb, std::string type, std::string md5, std::string def)
    : cb3_(cb) { 
      cb_type_ = CallbackType3;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }
  
  virtual void callback(unsigned char* data, int count)
  {
    if (!enable_)
      return;

    switch(cb_type_) {
      case CallbackType0: {
        MsgT tmsg;
        tmsg.callerid = mros::getCallerId(data, count);
        tmsg.deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
        this->cb0_(tmsg);
        break;
      }
      case CallbackType1: {
        MsgT tmsg;
        tmsg.callerid = mros::getCallerId(data, count);
        tmsg.deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
        this->cb1_(tmsg, this);
        break;
      }
      case CallbackType2: {
        std::shared_ptr<MsgT> tmsg = std::shared_ptr<MsgT>(new MsgT);
        tmsg->callerid = mros::getCallerId(data, count);
        tmsg->deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
        this->cb2_(tmsg);
        break;
      }
      case CallbackType3: {
        std::shared_ptr<MsgT> tmsg = std::shared_ptr<MsgT>(new MsgT);
        tmsg->callerid = mros::getCallerId(data, count);
        tmsg->deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
        this->cb3_(tmsg, this);
        break;
      }
      default: break;
    }
  }

  void intraProcess(const mros::MsgConstPtr& msg) {
    if (!enable_)
      return;

    switch(cb_type_) {
      case CallbackType0: {
        this->cb0_((const MsgT&)*msg);
        break;
      }
      case CallbackType1: {
        this->cb1_((const MsgT&)*msg, this);
        break;
      }
      case CallbackType2: {
        this->cb2_((const std::shared_ptr<MsgT const>&)msg);
        break;
      }
      case CallbackType3: {
        this->cb3_((const std::shared_ptr<MsgT const>&)msg, this);
        break;
      }
      default: break;
    }
  }

private:
  CallbackT0 cb0_;
  CallbackT1 cb1_;
  CallbackT2 cb2_;
  CallbackT3 cb3_;
};

template<>
class MROS_DllAPI Subscriber<void>: public SubscriberBase
{
public:
  typedef void(*CallbackT0)(const unsigned char* data, int count, void* userdata);
  typedef void(*CallbackT1)(const unsigned char* data, int count, void* userdata, SubscriberBase*);

  std::string callerid;

  Subscriber() = delete;

  Subscriber(std::string topic_name, CallbackT0 cb, std::string type, std::string md5, std::string def, void* userdata)
    : cb0_(cb) {
      userdata_ = userdata;
      cb_type_ = CallbackType0;
      topic_ = topic_name;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }

  Subscriber(std::string topic_name, CallbackT1 cb, std::string type, std::string md5, std::string def, void* userdata)
    : cb1_(cb) { 
      userdata_ = userdata;
      cb_type_ = CallbackType1;
      topic_ = topic_name;
      type_ = type;
      md5_ = md5;
      def_ = def;

      /* Note: finally set topic name*/
      setTopic(topic_name);
  }
  
  virtual void callback(unsigned char* data, int count)
  {
    if (!enable_)
      return;

    switch(cb_type_) {
      case CallbackType0: {
        this->callerid = mros::getCallerId(data, count);
        this->cb0_(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE, userdata_);
        break;
      }
      case CallbackType1: {
        this->callerid = mros::getCallerId(data, count);
        this->cb1_(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE, userdata_, this);
        break;
      }
      default: break;
    }
  }

private:
  CallbackT0 cb0_;
  CallbackT1 cb1_;
  void* userdata_;
};


}

#endif
