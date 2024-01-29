#ifndef _MROS_PUBLISHER_H_
#define _MROS_PUBLISHER_H_

#include <mros/os/msg.h>
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


namespace mros
{
class PubListener;

class NodeHandleBase;

class MROS_DllAPI Publisher
{
public:
  Publisher() = delete;
  Publisher(const std::string& topic_name, const Msg& msg);
  Publisher(const std::string& topic_name, const std::string& type, const std::string& md5, const std::string& def);
  
  ~Publisher();
  bool advertise(NodeHandleBase* nh, bool intra_process = false, int32_t queue_size = 50);
  void setEnabled(bool enable);
  bool getEnabled() {return enable_; }
  int publish(const unsigned char *buffer, int size, const std::string& callerid = "");
  int publish(const mros::MsgConstPtr& msg);
  int publish(const Msg* msg);
  int publish(const Msg& msg);
  int getNumSubscribers();
  bool negotiated();
  std::string getTopic();
  std::string getMsgType();
  std::string getMsgMD5();
  std::string getMsgDef();
  void setNegotiated(bool negotiated) { negotiated_ = negotiated; }
  uint32_t getTopicId() { return topic_id_; }
  void setTopicId(uint32_t id) { topic_id_ = id; }
  int getEndpointType() { return endpoint_; }
  void setEndpointType(int endpoint ) { endpoint_ = endpoint; }

private:
  friend class PubListener;
  NodeHandleBase* node_handle_;
  int endpoint_;
  uint32_t topic_id_;
  std::string type_; ///< Datatype of the topic
  std::string md5_; ///< md5sum of the topic
  std::string def_; ///< definition of the topic
  std::string topic_; ///< Name of the topic
  bool negotiated_; ///< negotiated
  bool enable_; ///< enabled
  std::shared_ptr<mroseprosima::fastrtps::Participant*> participant_; ///< participant
  std::shared_ptr<mroseprosima::fastrtps::Publisher*> publisher_; ///< publisher
  bool intra_process_; ///< use intra_process
  PubListener* listener_; ///< publisher listener
  void initialize(const std::string& topic_name, const std::string& type, 
    const std::string& md5, const std::string& def);
};
}

#endif
