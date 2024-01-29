#ifndef _MROS_SERVICE_SERVER_H_
#define _MROS_SERVICE_SERVER_H_

#include <functional>
#include <mros/macros.h>
#include "mros/mros_msgs/TopicInfo.h"
#include "mros/os/publisher.h"
#include "mros/os/subscriber.h"

namespace mros
{
class NodeHandleBase;
template<typename M, typename ObjT = void>
class MROS_DllAPI ServiceServer : public SubscriberBase
{
public:
  typedef void(ObjT::*CallbackT)(const typename M::Request&, typename M::Response&);

  ServiceServer() = delete;
  ServiceServer(std::string topic_name, CallbackT cb, ObjT* obj) :
    pub(topic_name + "|srv", typename M::Response()),
    obj_(obj)
  {
    pub.setEndpointType(mros::mros_msgs::TopicInfo::ID_SERVICE_SERVER + mros::mros_msgs::TopicInfo::ID_PUBLISHER);
    this->endpoint_ = mros::mros_msgs::TopicInfo::ID_SERVICE_SERVER + mros::mros_msgs::TopicInfo::ID_SUBSCRIBER;
    this->negotiated_ = false;
    this->srv_flag_ = true;
    this->cb_ = cb;
    this->type_ = req.getType();
    this->md5_ = req.getMD5();
    setTopic(topic_name + "|svc");
  }

  // these refer to the subscriber
  virtual void callback(unsigned char *data, int count)
  {
    if (!enable_) {
      return;
    }
    
    typename M::Request treq; typename M::Response tresp;
    treq.callerid = mros::getCallerId(data, count);
    treq.deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
    (obj_->*cb_)(treq, tresp);
    tresp.setID(treq.getID());
    pub.publish(&tresp);
  }
  
  virtual bool advertiseService(NodeHandleBase* nh) {
    return SubscriberBase::subscribe(nh) && pub.advertise(nh);
  }

  virtual bool negotiated()
  { 
    return (negotiated_ && pub.negotiated()); 
  }

  typename M::Request req;
  typename M::Response resp;
  Publisher pub;
private:
  CallbackT cb_;
  ObjT* obj_;
};

template<typename M>
class MROS_DllAPI ServiceServer<M, void> : public SubscriberBase
{
public:
  typedef std::function<void(const typename M::Request&, typename M::Response&)> CallbackT;

  ServiceServer() = delete;
  ServiceServer(std::string topic_name, CallbackT cb) :
    pub(topic_name + "|srv", typename M::Response())
  {
    pub.setEndpointType(mros::mros_msgs::TopicInfo::ID_SERVICE_SERVER + mros::mros_msgs::TopicInfo::ID_PUBLISHER);
    this->endpoint_ = mros::mros_msgs::TopicInfo::ID_SERVICE_SERVER + mros::mros_msgs::TopicInfo::ID_SUBSCRIBER;
    this->negotiated_ = false;
    this->srv_flag_ = true;
    this->cb_ = cb;
    this->type_ = req.getType();
    this->md5_ = req.getMD5();
    setTopic(topic_name + "|svc");
  }

  // these refer to the subscriber
  virtual void callback(unsigned char *data, int count)
  {
    if (!enable_) {
      return;
    }
    
    typename M::Request treq; typename M::Response tresp;
    treq.callerid = mros::getCallerId(data, count);
    treq.deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
    cb_(treq, tresp);
    tresp.setID(treq.getID());
    pub.publish(&tresp);
  }
  
  virtual bool advertiseService(NodeHandleBase* nh) {
    return SubscriberBase::subscribe(nh) && pub.advertise(nh);
  }
  
  virtual bool negotiated()
  { 
    return (negotiated_ && pub.negotiated()); 
  }

  typename M::Request req;
  typename M::Response resp;
  Publisher pub;
private:
  CallbackT cb_;
};

}

#endif
