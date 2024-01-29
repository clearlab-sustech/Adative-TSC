#ifndef MROS_NODE_HANDLE_BASE_H_
#define MROS_NODE_HANDLE_BASE_H_
#include "mros/os/common.h"
#include "mros/os/transport.h"
#include "mros/os/msg.h"
#include "mros/std_msgs/String.h"
#include "mros/mros_msgs/Log.h"
#include "mros/diagnostic_msgs/DiagnosticValue.h"
#include "mros/mros_msgs/TopicInfo.h"
#include <mros/mros_msgs/TopicStatistics.h>
#include "mros/os/log.h"
#include "mros/os/threadpool.h"
#include "mros/os/publisher.h"
#include "mros/os/subscriber.h"
#include "mros/os/service_server.h"
#include "mros/os/service_client.h"
#include "mros/os/signals.h"
#include <mros/macros.h>

namespace mroseprosima
{
  namespace fastrtps
  {
    class Participant;
    class Publisher;
    class Subscriber;
  }
}

namespace mros {
  class MROS_DllAPI NodeHandleBase
  {
  public:
    NodeHandleBase() {
      topic_statistics_ = std::make_shared<mros::mros_msgs::TopicStatistics>();
    }

    virtual ~NodeHandleBase() {}
  
  protected:
    std::recursive_mutex node_handle_mutex_;
    std::string node_name_{""};
    std::string ip_addr_{""};
    bool exited_{false};
    bool inited_{false};
    NodeType node_type_ {NodeType::NODE_TYPE_FASTRTPS}; 
    mroseprosima::fastrtps::Participant* participant_{nullptr};

    std::recursive_mutex topic_statistics_mutex_;
    mros::mros_msgs::TopicStatistics::Ptr topic_statistics_{nullptr};
    mros::Publisher* topic_statistics_publisher_{nullptr};

    std::map<uint32_t, Publisher*> publishers;
    std::map<uint32_t, SubscriberBase *> subscribers;
    std::map<uint32_t, std::shared_ptr<ThreadPool>> tunnel_pool;
  
  public:
    virtual bool exited() { return exited_; }
    virtual std::string& ip_addr() { return ip_addr_; }
    virtual std::string& node_name() { return node_name_; }
    virtual NodeType node_type() { return node_type_; }
    virtual mroseprosima::fastrtps::Participant* participant() { return participant_; }
    virtual bool inited() { return inited_; }

  public:
  /** @brief NodeHandle initialization function.
   *
   * \param node_name Name of this node.
   */
  virtual bool initNode(const std::string& node_name, const std::string& ip) { return false; }

  /**
   * \brief Advertise a Publisher
   *
   * This call to publicize that the node will be
   * publishing messages on the given Publisher.
   *
   * \param p The Publisher that allows you to publish a message on the given topic.
   *
   * \param intra_process If true, messages on topics which are published and subscribed to within
   * this context will go through a special intra-process communication code
   * code path which can avoid serialization and deserialization, unnecessary
   * copies, and achieve lower latencies in some cases.
   
   * \param queue_size Size of the queue.
   *
   * \return On success return true
   */
  bool advertise(Publisher & p, bool intra_process = false, int32_t queue_size = 50) {
    if (inited_) {
      switch (node_type_) {
        case NodeType::NODE_TYPE_FASTRTPS: {
          return p.advertise(this, intra_process, queue_size);
        }
        case NodeType::NODE_TYPE_UDP:
        case NodeType::NODE_TYPE_TCP: {
          std::unique_lock<std::recursive_mutex> lock(node_handle_mutex_);
          uint32_t topic_id = mros::guid::uint32();
          p.setTopicId(topic_id);
          if (node_type_ == NodeType::NODE_TYPE_UDP) {
            p.setNegotiated(true);
          }
          publishers[topic_id] = &p;
          lock.unlock();
          p.advertise(this, intra_process, queue_size);
          negotiateTopics(publishers[topic_id]);
          mros_log_debug("Publishers[%u] topic_id: %u, topic_name: %s", topic_id, p.getTopicId(), p.getTopic().c_str());
          return true;
        }
      
        default: {
          return false;
        }
      }
    }
    mros_log_error("MROS no inited!");
    return false;
  }
  
  /**
   * @brief Subscribes to a topic.
   *
   * This function is used to subscribe to a specific topic. The subscriber receives and processes
   * the messages published on this topic. The type of the SubscriberT parameter determines the message type.
   *
   * @tparam SubscriberT The type of the subscriber. This template parameter determines the type of the messages 
   *                     that the subscriber will receive and process.
   *                     
   * @param s Reference to the subscriber object, which will receive and process the messages published on the topic.
   * @param latch If set to true, enables the latching behavior. Latched topics remember the last value 
   *              published and automatically send it to any future subscribers. Default value is false.
   * @param reliable If set to true, enables the reliable communication mode. In reliable mode, the system ensures 
   *                 that all sent messages are received, even if this means slowing down the message flow or retrying 
   *                 failed deliveries. Default value is false.
   * @param queue_size The size of the incoming message queue. If the number of incoming messages exceeds this size, 
   *                   the oldest ones will start to be discarded. If set to -1 (the default), an unlimited queue size is used.
   *
   * @return Returns true if the subscription succeeds, false otherwise.
   */
  template<typename SubscriberT>
  bool subscribe(SubscriberT& s, bool latch = false, bool reliable = false, int32_t queue_size = -1) {
    if (inited_) {
      switch (node_type_) {
        case NodeType::NODE_TYPE_FASTRTPS: {
          return s.subscribe(this, latch, reliable, queue_size);
        }
        case NodeType::NODE_TYPE_UDP:
        case NodeType::NODE_TYPE_TCP: {
          std::unique_lock<std::recursive_mutex> lock(node_handle_mutex_);
          uint32_t topic_id = mros::guid::uint32();
          s.setTopicId(topic_id);
          if (node_type_ == NodeType::NODE_TYPE_UDP) {
            s.setNegotiated(true);
          }
          tunnel_pool[topic_id] = std::shared_ptr<ThreadPool>(new ThreadPool(1));
          subscribers[topic_id] = &s;
          lock.unlock();
          s.subscribe(this, latch, reliable, queue_size);
          negotiateTopics(subscribers[topic_id]);
          mros_log_debug("Subscribers[%u] topic_id: %u, topic_name: %s", topic_id, s.getTopicId(), s.getTopic().c_str());
          return true;
        }
      
        default: {
          return false;
        }
      }
    }
    mros_log_error("MROS no inited!");
    return false;
  }

  /**
 * @brief Advertises a service on the mros system.
 *
 * This function enables a node to provide a service using specific message types. 
 * The service will be advertised over mros, and any callbacks associated with this service will
 * be automatically called when requests for the service are received.
 *
 * @tparam M  The request message type used by the service.
 * @tparam T  The response message type used by the service.
 * @param srv An instance of ServiceServer which encapsulates the service details.
 *
 * @return True if the service is successfully advertised; false otherwise.
 */
  template<typename M, typename T>
  bool advertiseService(ServiceServer<M, T>& srv) {
    if (inited_) {
      switch (node_type_) {
        case NodeType::NODE_TYPE_FASTRTPS: {
          return srv.advertiseService(this);
        }
        case NodeType::NODE_TYPE_UDP:
        case NodeType::NODE_TYPE_TCP: {
          std::unique_lock<std::recursive_mutex> lock(node_handle_mutex_);
          uint32_t topic_id = mros::guid::uint32();
          if (node_type_ == NodeType::NODE_TYPE_UDP) {
            srv.setNegotiated(true);
            srv.pub.setNegotiated(true);
          }
          srv.setTopicId(topic_id);
          subscribers[topic_id] = &srv;
          tunnel_pool[topic_id] = std::shared_ptr<ThreadPool>(new ThreadPool(1));
          srv.pub.setTopicId(topic_id);
          publishers[topic_id] = &srv.pub;
          lock.unlock();
          srv.subscribe(this);
          srv.pub.advertise(this);
          negotiateTopics(subscribers[topic_id]);
          negotiateTopics(publishers[topic_id]);
          mros_log_debug("advertiseService Subscribers[%u] topic_id: %u, topic_name: %s | Publishers[%d] topic_id: %u, topic_name: %s",
            topic_id, srv.getTopicId(), srv.getTopic().c_str(), topic_id, srv.pub.getTopicId(), srv.pub.getTopic().c_str());
          return true;
        }
        default: {
          return false;
        }
      }
    }
    mros_log_error("MROS no inited!");
    return false;
  }

  /**
   * @brief This function serves as an interface to use a service client in mros.
   *
   * The serviceClient function is a template function where M is any type of 
   * service message in the mros framework. This function is commonly used for 
   * making requests to a service and waiting for its response.
   *
   * @tparam M - The service message type, defined within the mros framework.
   * @param svc - A reference to the ServiceClient object which is used to interact with the service.
   *
   * @return Returns a boolean value indicating the success or failure of the service request.
   */
  template<typename M>
  bool serviceClient(ServiceClient<M>& svc) {
    if (inited_) {
      switch (node_type_) {
        case NodeType::NODE_TYPE_FASTRTPS: {
          return svc.serviceClient(this);
        }
        case NodeType::NODE_TYPE_UDP:
        case NodeType::NODE_TYPE_TCP: {
          std::unique_lock<std::recursive_mutex> lock(node_handle_mutex_);
          uint32_t topic_id = mros::guid::uint32();
          if (node_type_ == NodeType::NODE_TYPE_UDP) {
            svc.setNegotiated(true);
            svc.pub.setNegotiated(true);
          }
          subscribers[topic_id] = &svc;
          tunnel_pool[topic_id] = std::shared_ptr<ThreadPool>(new ThreadPool(1));
          svc.setTopicId(topic_id);
          svc.pub.setTopicId(topic_id);
          publishers[topic_id] = &svc.pub;
          lock.unlock();
          svc.subscribe(this);
          svc.pub.advertise(this);
          negotiateTopics(subscribers[topic_id]);
          negotiateTopics(publishers[topic_id]);
          mros_log_debug("serviceClient Subscribers[%u] topic_id: %u, topic_name: %s | Publishers[%d] topic_id: %u, topic_name: %s", 
            topic_id, svc.getTopicId(), svc.getTopic().c_str(), topic_id, svc.pub.getTopicId(), svc.pub.getTopic().c_str());
          return true;
        }
        default: {
          return false;
        }
      }
    }
    mros_log_error("MROS no inited!");
    return false;
  }

  /**
   * \brief Log to a given verbosity level , with printf-style formatting.
   *
   * \param byte The verbosity level.
   * \param msg The message that will be output.
   */
  virtual void log(const mros::mros_msgs::LogConstPtr& msg) { }

  virtual void diag(const mros::diagnostic_msgs::DiagnosticValue& msg) { }
  
  /**
   * \brief Returns topic list.
   */
  virtual std::string getTopicList() { return ""; }
  
  /**
   * \brief Returns subscribed topic list.
   */
  virtual std::string getSubscribedTopicList() { return ""; }

  /**
   * \brief Returns published topic list.
   */
  virtual std::string getPublishedTopicList() { return ""; }

  /**
   * \brief Returns service list.
   */
  virtual std::string getServiceList() { return ""; }
  
  /**
   * \brief Close every handle created through this NodeHandle.
   *
   * This method will unadvertise every topic and service advertisement,
   * and unsubscribe every subscription created through this NodeHandle.
   */
  virtual void exit() {}
  
  /** \brief Check whether it's time to exit.
   *
   * This method checks to see if both mros::nh()::ok() is true and shutdown() has not been called, to see whether it's yet time
   * to exit.  ok() is false once either mros::shutdown() or mros::nh()::exit() have been called
   *
   * \return true if we're still OK, false if it's time to exit
   */
  virtual bool ok() { return false; }

  virtual TransportBase* getTransport() { return nullptr; }

  /**
   * \brief Returns node list.
   */
  virtual std::string getNodeList() { return ""; }
  
  /** \brief Defines the location of the default profile configuration XML file. 
   * If this variable is set and its value corresponds with an existing file, Fast DDS will load its profiles. 
   * Linux: export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/profiles.xml
   *
   * \return true If FASTRTPS_DEFAULT_PROFILES_FILE variable is set
   */
  virtual bool useQosFromXml() { return false; }

 /** \brief Set mros discoveryt participant
   *
   * \return true If success
   */
  virtual bool setDiscoverytParticipant(mroseprosima::fastrtps::Participant* participant) { return false; }

 /** \brief publish topic statistics info
   *
   * \return true If success
   */
  virtual bool publishTopicStatisticsInfo() {
    if (!inited_ || exited_ || node_name_ == "mrosagent") {
      return false;
    }

    std::unique_lock<std::recursive_mutex> lock(topic_statistics_mutex_);
    if (!topic_statistics_publisher_) {
      std::string suffix = "";
      if (node_type_ == NodeType::NODE_TYPE_TCP || node_type_ == NodeType::NODE_TYPE_UDP) {
        suffix = "(mrosagent)";
      }
      topic_statistics_->node = node_name_ + suffix + "@" + std::to_string(mros::guid::uint32());
      topic_statistics_publisher_ = new Publisher(MROS_STATISTICS, mros::mros_msgs::TopicStatistics());
      this->advertise(*topic_statistics_publisher_);
    }
    topic_statistics_publisher_->publish(topic_statistics_);
    return true;
  }

 /** \brief addPublisherTopicStatisticsInfo
   *
   * \return true If success
   */
  virtual bool addPublisherTopicStatisticsInfo(mros::mros_msgs::TopicInfo & topic_info) {
    if (!inited_ || exited_) {
      return false;
    }
    std::unique_lock<std::recursive_mutex> lock(topic_statistics_mutex_);
    for (auto & p: topic_statistics_->publishers) {
      if (p.topic_name == topic_info.topic_name) {
        return false;
      }
    }
    topic_statistics_->publishers.push_back(topic_info);
    return true;
  }

 /** \brief delPublisherTopicStatisticsInfo
   *
   * \return true If success
   */
  virtual bool delPublisherTopicStatisticsInfo(const std::string& topic_name) {
    if (!inited_ || exited_) {
      return false;
    }

    std::unique_lock<std::recursive_mutex> lock(topic_statistics_mutex_);
    std::vector<mros::mros_msgs::TopicInfo>::iterator it;
    for (it = topic_statistics_->publishers.begin(); it != topic_statistics_->publishers.end(); ) {
        if ((*it).topic_name == topic_name) {
          it = topic_statistics_->publishers.erase(it);
        } else {
          ++it;
        }
    }
    return true;
  }

 /** \brief addSubscriberTopicStatisticsInfo
   *
   * \return true If success
   */
  virtual bool addSubscriberTopicStatisticsInfo(mros::mros_msgs::TopicInfo & topic_info) {
    if (!inited_ || exited_) {
      return false;
    }

    std::unique_lock<std::recursive_mutex> lock(topic_statistics_mutex_);
    for (auto & s: topic_statistics_->subscribers) {
      if (s.topic_name == topic_info.topic_name) {
        return false;
      }
    }
    topic_statistics_->subscribers.push_back(topic_info);
    return true;
  }

 /** \brief delSubscriberTopicStatisticsInfo
   *
   * \return true If success
   */
  virtual bool delSubscriberTopicStatisticsInfo(const std::string& topic_name) {
    if (!inited_ || exited_) {
      return false;
    }

    std::unique_lock<std::recursive_mutex> lock(topic_statistics_mutex_);
    std::vector<mros::mros_msgs::TopicInfo>::iterator it;
    for (it = topic_statistics_->subscribers.begin(); it != topic_statistics_->subscribers.end(); ) {
        if ((*it).topic_name == topic_name) {
          it = topic_statistics_->subscribers.erase(it);
        } else {
          ++it;
        }
    }
    return true;
  }

  /** \brief remove publisher from publishers
   *
   * \return true If success
   */
  virtual void removePublisher(uint32_t id) {
    std::unique_lock<std::recursive_mutex> lock(node_handle_mutex_);
    if (publishers.count(id) > 0) {
      publishers.erase(id);
    }
  }

  /** \brief remove subscriber from subscribers
   *
   * \return true If success
   */
  virtual void removeSubscriber(uint32_t id) {
    std::unique_lock<std::recursive_mutex> lock(node_handle_mutex_);
    if (subscribers.count(id) > 0) {
      subscribers[id]->setEnabled(false);
      if (tunnel_pool.count(id) > 0) {
        tunnel_pool[id]->shutdown();
        tunnel_pool.erase(id);
      }
      subscribers.erase(id);
    }
  }

  virtual void negotiateTopics(Publisher * p) {}

  virtual void negotiateTopics(SubscriberBase * s) {}

  virtual void negotiateTopics() {}

  virtual int spin() { return 0; }
  virtual int publish(uint32_t, const Msg *) { return 0; }
  virtual int publish(uint32_t, const unsigned char *, int, const std::string&) { return 0; }

  };

//////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Wait for a single message to arrive on a topic
   *
   * \param M <template> The message type
   * \param topic The topic to subscribe on
   * \param timeout The amount of time to wait before returning if no message is received
   * \return The message.  Empty std::shared_ptr if waitForMessage is interrupted by the mros shutting down
   */
  template<class M>
  std::shared_ptr<M const> waitForMessage(const std::string& topic, mros::Duration timeout) {
    static std::shared_ptr<M const> m;
    m = nullptr;
    mros::Subscriber<M> sub(topic, [](const std::shared_ptr<M const>& msg) {
      m = msg;
    });
    mros::nh()->subscribe(sub);
    mros::Time end = mros::Time::now() + timeout;
    while(!m && mros::nh()->ok()) {
      usleep(100 * 1000);
      if (!timeout.isZero() && mros::Time::now() >= end) {
        break;
      }
    }

    if (!m) {
      return std::shared_ptr<M>(new M);
    }

    return m;
  }

  /**
   * \brief Wait for a single message to arrive on a topic
   *
   * \param M <template> The message type
   * \param topic The topic to subscribe on
   * \return The message.  Empty std::shared_ptr if waitForMessage is interrupted by the mros shutting down
   */
  template<class M>
  std::shared_ptr<M const> waitForMessage(const std::string& topic) {
    static std::shared_ptr<M const> m;

    m = nullptr;

    mros::Subscriber<M> sub(topic, [](const std::shared_ptr<M const>& msg) {
      m = msg;
    });

    mros::nh()->subscribe(sub);

    while(!m && mros::nh()->ok()) {
      usleep(100 * 1000);
    }

    if (!m) {
      return std::shared_ptr<M>(new M);
    }

    return m;
  }
}

////////////////////////////////////////////////////////////////////////////

namespace mros{
struct nh {
nh() = delete;
struct Publisher {
  std::shared_ptr<mros::Publisher> pub;
  Publisher(): pub(nullptr) { }
  Publisher(const std::shared_ptr<mros::Publisher>& _pub): pub(_pub) { }

  void setEnabled(bool enable) {
    if (pub == nullptr) {
      return;
    }
    pub->setEnabled(enable);
  }
  bool getEnabled() {
    if (pub == nullptr) {
      return false;
    }
    return pub->getEnabled();
  }
  bool negotiated() {
    if (pub == nullptr) {
      return false;
    }
    return pub->negotiated();
  }
  std::string getTopic() {
    if (pub == nullptr) {
      return "";
    }
    return pub->getTopic();
  }
  std::string getMsgType() {
    if (pub == nullptr) {
      return "";
    }
    return pub->getMsgType();
  }
  std::string getMsgMD5() {
    if (pub == nullptr) {
      return "";
    }
    return pub->getMsgMD5();
  }
  std::string getMsgDef() {
    if (pub == nullptr) {
      return "";
    }
    return pub->getMsgDef();
  }
  int publish(const unsigned char *buffer, int size, const std::string& callerid = "") {
    if (pub == nullptr) {
      return -1;
    }
    return pub->publish(buffer, size, callerid);
  }

  int publish(const mros::MsgConstPtr& msg) {
    if (pub == nullptr) {
      return -1;
    }
    return pub->publish(msg);
  }
  int publish(const Msg* msg) {
    if (pub == nullptr) {
      return -1;
    }
    return pub->publish(msg);
  }

  int publish(const Msg& msg)  {
    if (pub == nullptr) {
      return -1;
    }
    return pub->publish(msg);
  }

  int getNumSubscribers() {
    if (pub == nullptr) {
      return 0;
    }
    return pub->getNumSubscribers();
  }
};

/**
 * @brief Advertise a specific topic.
 *
 * This function is used to advertise a specific topic. It does not publish any message,
 * but uses the provided `msg` parameter as a type holder to extract the message type information.
 * The function creates and returns a shared pointer to the Publisher object.
 * @tparam M The message type associated with the given topic_name.
 * @param topic_name The name of the topic to advertise.
 * @param intra_process If set to true, enables the intra-process communication mode 
 *                      which allows for passing messages within the same process via shared memory, default value is false.
 * @param queue_size   The size of the message queue, if the number of incoming messages 
 *                     exceeds this size, the oldest ones will start to be discarded. Default value is 50.
 *
 * @return A shared pointer to the Publisher object that can be used to publish messages.
 *
 * @throw SomeExceptionType If any error occurs during the operation, an exception of type SomeExceptionType is thrown.
 */
template<typename M>
static mros::nh::Publisher advertise(const std::string& topic_name, bool intra_process = false, int32_t queue_size = 50) {
  std::shared_ptr<mros::Publisher> pub = std::make_shared<mros::Publisher>(topic_name, M());
  mros::nh()->advertise(*pub, intra_process, queue_size);
  mros::nh::Publisher publisher(pub);
  return publisher;
}

struct Subscriber {
  std::shared_ptr<mros::SubscriberBase> sub;
  Subscriber(): sub(nullptr) { }
  Subscriber(const std::shared_ptr<mros::SubscriberBase>& _sub): sub(_sub) { }

  void setEnabled(bool enable) {
    if (sub == nullptr) {
      return;
    }
    sub->setEnabled(enable);
  }
  bool getEnabled() {
    if (sub == nullptr) {
      return false;
    }
    return sub->getEnabled();
  }
  
  bool negotiated() {
    if (sub == nullptr) {
      return false;
    }
    return sub->negotiated();
  }
  std::string getTopic() {
    if (sub == nullptr) {
      return "";
    }
    return sub->getTopic();
  }
  std::string getMsgType() {
    if (sub == nullptr) {
      return "";
    }
    return sub->getMsgType();
  }
  std::string getMsgMD5() {
    if (sub == nullptr) {
      return "";
    }
    return sub->getMsgMD5();
  }
  std::string getMsgDef() {
    if (sub == nullptr) {
      return "";
    }
    return sub->getMsgDef();
  }
};

/**
 * @brief A function template for subscribing to a specific topic.
 *
 * This function provides the ability for an object to subscribe to a particular topic. 
 * This is commonly used in publish-subscribe model where messages are sent by publishers and received by subscribers. 
 * The function requires the message type, callback function and the object which contains the callback function.
 *
 * @tparam M The message type associated with the given topic_name.
 * @tparam T An optional template parameter that defaults to void. 
 *         It represents the type of the object that has the method to be called when a new message arrives on this topic.
 *
 * @param[in] topic_name A string specifying the name of the topic to subscribe to.
 * @param[in] fb A pointer to the member function that will be called back when a message arrives on the subscribed topic.
 * @param[in] obj Pointer to the object that the above method will be invoked on. This parameter is ignored if T is void.
 * @param[in] latch Optional boolean parameter. If set to true, late joiners will receive last published message.
 * @param[in] reliable Optional boolean parameter. If set to true, the system ensures the delivery of the messages. This may add overheads.
 * @param[in] queue_size Optional parameter to set the size of the message queue. If not provided or set to -1, the queue size will be unlimited.
 *
 * @return Returns a shared_ptr to a Subscriber object which can be saved and later used to unsubscribe.
 */
template<typename M, typename T = void>
static mros::nh::Subscriber subscribe(const std::string& topic_name, void(T::*fb)(const M&), T* obj, bool latch = false, bool reliable = false, int32_t queue_size = -1) {
  std::shared_ptr<mros::Subscriber<M, T>> sub = std::make_shared<mros::Subscriber<M, T>>(topic_name, fb, obj);
  mros::nh()->subscribe(*sub, latch, reliable, queue_size);
  mros::nh::Subscriber subscriber(sub);
  return subscriber;
}

/**
 * @brief A function template for subscribing to a specific topic.
 *
 * This function provides the ability for an object to subscribe to a particular topic. 
 * This is commonly used in publish-subscribe model where messages are sent by publishers and received by subscribers. 
 * The function requires the message type, callback function and the object which contains the callback function.
 *
 * @tparam M The message type associated with the given topic_name.
 * @tparam T An optional template parameter that defaults to void. 
 *         It represents the type of the object that has the method to be called when a new message arrives on this topic.
 *
 * @param[in] topic_name A string specifying the name of the topic to subscribe to.
 * @param[in] fb A pointer to the member function that will be called back when a message arrives on the subscribed topic.
 * @param[in] obj Pointer to the object that the above method will be invoked on. This parameter is ignored if T is void.
 * @param[in] latch Optional boolean parameter. If set to true, late joiners will receive last published message.
 * @param[in] reliable Optional boolean parameter. If set to true, the system ensures the delivery of the messages. This may add overheads.
 * @param[in] queue_size Optional parameter to set the size of the message queue. If not provided or set to -1, the queue size will be unlimited.
 *
 * @return Returns a shared_ptr to a Subscriber object which can be saved and later used to unsubscribe.
 */
template<typename M, typename T = void>
static mros::nh::Subscriber subscribe(const std::string& topic_name, void(T::*fb)(const std::shared_ptr<M const>&), T* obj, bool latch = false, bool reliable = false, int32_t queue_size = -1) {
  std::shared_ptr<mros::Subscriber<M, T>> sub = std::make_shared<mros::Subscriber<M, T>>(topic_name, fb, obj);
  mros::nh()->subscribe(*sub, latch, reliable, queue_size);
  mros::nh::Subscriber subscriber(sub);
  return subscriber;
}

/**
 * @brief A function template for subscribing to a specific topic.
 *
 * This function provides the ability for an object to subscribe to a particular topic. 
 * This is commonly used in publish-subscribe model where messages are sent by publishers and received by subscribers. 
 * The function requires the message type, callback function and the object which contains the callback function.
 *
 * @tparam M The message type associated with the given topic_name.
 * @tparam T An optional template parameter that defaults to void. 
 *         It represents the type of the object that has the method to be called when a new message arrives on this topic.
 *
 * @param[in] topic_name A string specifying the name of the topic to subscribe to.
 * @param[in] fb A pointer to the member function that will be called back when a message arrives on the subscribed topic.
 * @param[in] obj Pointer to the object that the above method will be invoked on. This parameter is ignored if T is void.
 * @param[in] latch Optional boolean parameter. If set to true, late joiners will receive last published message.
 * @param[in] reliable Optional boolean parameter. If set to true, the system ensures the delivery of the messages. This may add overheads.
 * @param[in] queue_size Optional parameter to set the size of the message queue. If not provided or set to -1, the queue size will be unlimited.
 *
 * @return Returns a shared_ptr to a Subscriber object which can be saved and later used to unsubscribe.
 */
template<typename M>
static mros::nh::Subscriber subscribe(const std::string& topic_name, std::function<void(const M&)> fb, bool latch = false, bool reliable = false, int32_t queue_size = -1) {
  std::shared_ptr<mros::Subscriber<M>> sub = std::make_shared<mros::Subscriber<M>>(topic_name, fb);
  mros::nh()->subscribe(*sub, latch, reliable, queue_size);
  mros::nh::Subscriber subscriber(sub);
  return subscriber;
}

/**
 * @brief A function template for subscribing to a specific topic.
 *
 * This function provides the ability for an object to subscribe to a particular topic. 
 * This is commonly used in publish-subscribe model where messages are sent by publishers and received by subscribers. 
 * The function requires the message type, callback function and the object which contains the callback function.
 *
 * @tparam M The message type associated with the given topic_name.
 * @tparam T An optional template parameter that defaults to void. 
 *         It represents the type of the object that has the method to be called when a new message arrives on this topic.
 *
 * @param[in] topic_name A string specifying the name of the topic to subscribe to.
 * @param[in] fb A pointer to the member function that will be called back when a message arrives on the subscribed topic.
 * @param[in] obj Pointer to the object that the above method will be invoked on. This parameter is ignored if T is void.
 * @param[in] latch Optional boolean parameter. If set to true, late joiners will receive last published message.
 * @param[in] reliable Optional boolean parameter. If set to true, the system ensures the delivery of the messages. This may add overheads.
 * @param[in] queue_size Optional parameter to set the size of the message queue. If not provided or set to -1, the queue size will be unlimited.
 *
 * @return Returns a shared_ptr to a Subscriber object which can be saved and later used to unsubscribe.
 */
template<typename M>
static mros::nh::Subscriber subscribe(const std::string& topic_name, std::function<void(const std::shared_ptr<M const>&)> fb, bool latch = false, bool reliable = false, int32_t queue_size = -1) {
  std::shared_ptr<mros::Subscriber<M>> sub = std::make_shared<mros::Subscriber<M>>(topic_name, fb);
  mros::nh()->subscribe(*sub, latch, reliable, queue_size);
  mros::nh::Subscriber subscriber(sub);
  return subscriber;
}

struct ServiceServer {
  std::shared_ptr<mros::SubscriberBase> srv;
  ServiceServer(): srv(nullptr) { }
  ServiceServer(const std::shared_ptr<mros::SubscriberBase>& _srv): srv(_srv) { }

  void setEnabled(bool enable) {
    if (srv == nullptr) {
      return;
    }
    srv->setEnabled(enable);
  }
  bool getEnabled() {
    if (srv == nullptr) {
      return false;
    }
    return srv->getEnabled();
  }
  
  bool negotiated() {
    if (srv == nullptr) {
      return false;
    }
    return srv->negotiated();
  }
  std::string getTopic() {
    if (srv == nullptr) {
      return "";
    }
    return srv->getTopic();
  }
  std::string getMsgType() {
    if (srv == nullptr) {
      return "";
    }
    return srv->getMsgType();
  }
  std::string getMsgMD5() {
    if (srv == nullptr) {
      return "";
    }
    return srv->getMsgMD5();
  }
  std::string getMsgDef() {
    if (srv == nullptr) {
      return "";
    }
    return srv->getMsgDef();
  }
};
  
/**
 * @brief Advertises a service on the mros system.
 *
 * This function enables a node to provide a service for a specific topic with given message types. 
 * The service will be advertised over mros, and the provided member function callback will 
 * automatically be invoked when requests for the service are received.
 *
 * @tparam M  The request message type used by the service.
 * @tparam T  The object type that provides the callback method.
 * @param topic_name The name of the topic/service as a string.
 * @param fb A pointer to the class member function to be used as a callback.
 *           This callback will be invoked when a service request is received.
 * @param obj A pointer to the instance of the object on which the callback method should be invoked.
 *
 * @return A shared_ptr to the advertised ServiceServer instance.
 */
template<typename M, typename T>
static mros::nh::ServiceServer advertiseService(const std::string& topic_name, void(T::*fb)(const typename M::Request&,  typename M::Response&), T* obj) {
  std::string srvname = topic_name;
  std::shared_ptr<mros::ServiceServer<M, T>> srv = std::make_shared<mros::ServiceServer<M, T>>(srvname, fb, obj);
  mros::nh()->advertiseService(*srv);
  mros::nh::ServiceServer server(srv);
  return server;
}

/**
 * @brief Advertises a service on the mros system.
 *
 * This function allows a node to provide a service for a specific topic using a specific message type. 
 * The service will be advertised over mros, and the provided std::function callback will 
 * automatically be invoked when requests for the service are received.
 *
 * @tparam M  The message type utilized by the service.
 * @param topic_name The name of the topic/service as a string.
 * @param fb A std::function instance to be used as a callback.
 *           This callback will be triggered when a service request is received.
 *           The function should accept two parameters: a const reference to the request message 
 *           and a reference to the response message (which it can modify).
 *
 * @return A shared_ptr to the advertised ServiceServer instance.
 */
template<typename M>
static mros::nh::ServiceServer advertiseService(const std::string& topic_name, std::function<void(const typename M::Request&,  typename M::Response&)> fb) {
  std::string srvname = topic_name;
  std::shared_ptr<mros::ServiceServer<M>> srv = std::make_shared<mros::ServiceServer<M>>(srvname, fb);
  mros::nh()->advertiseService(*srv);
  mros::nh::ServiceServer server(srv);
  return server;
}

template<typename M>
struct ServiceClient {
  std::shared_ptr<mros::ServiceClient<M>> svc;
  ServiceClient(): svc(nullptr) { }
  ServiceClient(const std::shared_ptr<mros::ServiceClient<M>>& _svc): svc(_svc) { }

  void setEnabled(bool enable) {
    if (svc == nullptr) {
      return;
    }
    svc->setEnabled(enable);
  }
  bool getEnabled() {
    if (svc == nullptr) {
      return false;
    }
    return svc->getEnabled();
  }
  
  bool negotiated() {
    if (svc == nullptr) {
      return false;
    }
    return svc->negotiated();
  }
  std::string getTopic() {
    if (svc == nullptr) {
      return "";
    }
    return svc->getTopic();
  }
  std::string getMsgType() {
    if (svc == nullptr) {
      return "";
    }
    return svc->getMsgType();
  }
  std::string getMsgMD5() {
    if (svc == nullptr) {
      return "";
    }
    return svc->getMsgMD5();
  }
  std::string getMsgDef() {
    if (svc == nullptr) {
      return "";
    }
    return svc->getMsgDef();
  }

  bool call(typename M::Request & request, std::shared_ptr<typename M::Response>& response, double duration = 3.0 /* seconds */) {
    if (svc == nullptr) {
      return false;
    }
    return svc->call(request, *response, duration);
  }

  bool call(M& m, double duration = 3.0 /* seconds */) {
    if (svc == nullptr) {
      return false;
    }
    return svc->call(m, duration);
  }
  
  bool call(typename M::Request & request, typename M::Response & response, double duration = 3.0 /* seconds */) {
    if (svc == nullptr) {
      return false;
    }
    return svc->call(request, response, duration);
  }
};

/**
 * @brief This function is responsible for creating a ServiceClient object in the mros framework.
 *
 * The serviceClient function is templated on any type of service message 'M' within the mros system. It takes a string as an argument, 
 * which represents the name of the topic, and returns a shared pointer to a ServiceClient object associated with that topic. This allows 
 * for efficient handling and management of resources, as the ServiceClient object can be shared among various components without 
 * duplicating it.
 *
 * @tparam M - The service message type, defined within the mros framework.
 * @param topic_name - The name of the topic for which the ServiceClient object is being created.
 *
 * @return Returns a std::shared_ptr to the ServiceClient object associated with the topic.
 */
template<typename M>
static mros::nh::ServiceClient<M> serviceClient(const std::string& topic_name) {
  std::string srvname = topic_name;
  std::shared_ptr<mros::ServiceClient<M>> svc = std::make_shared<mros::ServiceClient<M>>(srvname);
  mros::nh()->serviceClient(*svc);
  mros::nh::ServiceClient<M> client(svc);
  return client;
}
};
}
#endif