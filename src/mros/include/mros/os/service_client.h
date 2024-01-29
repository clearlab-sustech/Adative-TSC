#ifndef _MROS_SERVICE_CLIENT_H_
#define _MROS_SERVICE_CLIENT_H_
#include <stdint.h>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <mros/macros.h>
#include "mros/mros.h"
#include "mros/os/log.h"
#include "mros/mros_msgs/TopicInfo.h"
#include "mros/os/publisher.h"
#include "mros/os/subscriber.h"

namespace mros
{
class NodeHandleBase;
template<typename M>
class MROS_DllAPI ServiceClient : public SubscriberBase
{
public:
  ServiceClient() = delete;
  ServiceClient(std::string topic_name)
    : pub(topic_name + "|svc", typename M::Request()) {
    pub.setEndpointType(mros::mros_msgs::TopicInfo::ID_SERVICE_CLIENT + mros::mros_msgs::TopicInfo::ID_PUBLISHER);
    this->endpoint_ = mros::mros_msgs::TopicInfo::ID_SERVICE_CLIENT + mros::mros_msgs::TopicInfo::ID_SUBSCRIBER;
    this->negotiated_ = false;
    this->srv_flag_ = true;
    this->call_resp = NULL;
    this->call_req = NULL;
    this->type_ = resp.getType();
    this->md5_ = resp.getMD5();
    setTopic(topic_name + "|srv");
  }

  bool call(typename M::Request & request, std::shared_ptr<typename M::Response>& response, double duration = 3.0 /* seconds */) {
    if (response == nullptr) {
      return false;
    }
    return call(request, *response, duration);
  }

  bool call(M& m, double duration = 3.0 /* seconds */) {
    return call(m.request, m.response, duration);
  }
  
  bool call(typename M::Request & request, typename M::Response & response, double duration = 3.0 /* seconds */) {
    if (!enable_) {
      return false;
    }

    std::string srvname = pub.getTopic();
    mros::V_ShapeTopicInfo srvlist;
    mros::getServiceList(srvlist);
    auto it = std::find_if(srvlist.begin(), srvlist.end(), [&srvname](const mros::ShapeTopicInfo& info) { return info.name == srvname; });
    if ((it != srvlist.end()) && (it->md5sum != request.getMD5())) {
      mros_log_warn("Service MD5 does not match [srv: %s, svc: %s].", it->md5sum.c_str(), request.getMD5().c_str());
    } else if (pub.getNumSubscribers() <= 0) {
      mros_log_warn("Cannot find service [%s].", srvname.c_str());
    } 

    std::unique_lock<std::mutex> g_lock(g_mutex_);
    std::unique_lock<std::mutex> lock(mutex_);
    
    call_req = &request;
    call_resp = &response;
    call_req->setID(mros::guid::uint32());
    
    if (pub.publish(&request) <= 0) {
      call_req = NULL; call_resp = NULL;
      return false;
    }
    if (cond_.wait_until(lock, std::chrono::system_clock::now() +
      std::chrono::milliseconds((int64_t)(duration * 1000))) == std::cv_status::timeout) {
      if (this->getTopic() != std::string(MROS_DISCOVERY)) {
        mros_log_warn("Service[%s] call_req.id: %u, call timeout", this->getTopic().c_str(), call_req->getID());
      }
      call_req = NULL; call_resp = NULL;
      return false;
    }
    call_req = NULL; call_resp = NULL;
    return true;
  }

  // these refer to the subscriber
  virtual void callback(unsigned char *data, int count) {
    if (call_resp && call_req) {
      std::unique_lock<std::mutex> lock(mutex_);
      if (call_resp && call_req) {
        uint32_t req_id  = call_req->getID();
        uint32_t resp_id =  ((uint32_t) (*(data + MROS_CALLERID_SIZE + 0)));
        resp_id |= ((uint32_t) (*(data + MROS_CALLERID_SIZE + 1))) << (8 * 1);
        resp_id |= ((uint32_t) (*(data + MROS_CALLERID_SIZE + 2))) << (8 * 2);
        resp_id |= ((uint32_t) (*(data + MROS_CALLERID_SIZE + 3))) << (8 * 3);

        if (req_id == resp_id) {
          call_resp->callerid = mros::getCallerId(data, count);
          call_resp->deserialize(data + MROS_CALLERID_SIZE, count - MROS_CALLERID_SIZE);
          cond_.notify_all();
        }
      }
    }
  }
  
  virtual bool serviceClient(NodeHandleBase* nh) {
    return SubscriberBase::subscribe(nh) && pub.advertise(nh);
  }
  
  virtual bool negotiated() {
    return (negotiated_ && pub.getNumSubscribers() > 0);
  }

  typename M::Request req;
  typename M::Response resp;
  typename M::Request * call_req;
  typename M::Response * call_resp;
  Publisher pub;
  std::mutex mutex_;
  std::mutex g_mutex_;
  std::condition_variable cond_;
};
}

#endif
