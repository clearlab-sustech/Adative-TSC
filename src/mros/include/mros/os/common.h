#ifndef _MROS_MROS_COMMON_H_
#define _MROS_MROS_COMMON_H_

#include <future>
#include <stdint.h>
#include <mutex>
#include <memory>
#include <map>
#include <stddef.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <mros/macros.h>

namespace mros {
  class NodeListener;
  class NodeHandleBase;

  enum NodeType {
    NODE_TYPE_FASTRTPS, 
    NODE_TYPE_TCP, 
    NODE_TYPE_UDP
  };

  /**
   * \brief Contains information about a topic
   */
  struct ShapeTopicInfo
  {
    ShapeTopicInfo() {}
    ShapeTopicInfo(const std::string& _name, const std::string& _datatype ,
      const std::string& _md5sum, const std::string& _definition)
    : name(_name)
    , datatype(_datatype)
    , md5sum(_md5sum)
    , definition(_definition)
    {}
    std::string name;        ///< Name of the topic
    std::string datatype;    ///< Datatype of the topic
    std::string md5sum;      ///< md5sum of the topic
    std::string definition;  ///< definition of the topic
  };

  typedef std::vector<ShapeTopicInfo> V_ShapeTopicInfo;

  /** @brief Get the list of topics that are being published by all nodes.
   *
   * @param topics A place to store the resulting list.  Each item in the
   * list is a pair <string topic, string datatype, std::string md5sum, std::string definition>.  
   * The type is represented in the format "package_name/MessageName".
   *
   * @return true on success, false otherwise (topics not filled in)
   */
  MROS_DllAPI bool getServiceList(V_ShapeTopicInfo& topics);
  MROS_DllAPI bool getTopics(V_ShapeTopicInfo& topics, int duration = 3 /* seconds */);
  MROS_DllAPI bool getPublishedTopics(V_ShapeTopicInfo& topics, int duration = 3 /* seconds */);
  MROS_DllAPI bool getSubscribedTopics(V_ShapeTopicInfo& topics, int duration = 3 /* seconds */);
  MROS_DllAPI std::vector<std::string> topicSplit(const std::string& s, const std::string& delim);

  /**
   * \brief Get the singleton object of NodeHandleBase
   */
  MROS_DllAPI NodeHandleBase* nh();

  /**
   * \brief Disconnects everything and unregisters from the DDS.  It is generally not
   * necessary to call this function, as the node will automatically shutdown when all
   * NodeHandles destruct. However, if you want to break out of a DDS Participant explicitly,
   * this function allows that.
   */
  MROS_DllAPI void shutdown();

  /** @brief MROS initialization function.
   *
   * \param node_name Name of this node.
   * \param signal_callback
   * \param node_type
   * \param install_signal If true install the global signal handler for mros. 
   *        The signal handler will shutdown all initialized context.
   */
  MROS_DllAPI void init(std::string node_name, 
            const std::function<void()>& signal_callback = nullptr, 
            NodeType node_type = NodeType::NODE_TYPE_FASTRTPS, 
            const std::string& ip_addr = "127.0.0.1", 
            bool install_signal = true);

  /** \brief Enter simple event loop
   *
   * This method enters a loop, processing callbacks.  This method should only be used
   * if the NodeHandle API is being used.
   *
   * This method is mostly useful when your node does all of its work in
   * subscription callbacks.
   */
  MROS_DllAPI void spin();

  /**
   * \brief Returns the name of NodeHandle.
   */
  MROS_DllAPI const std::string& getNodeName();

  /**
   * \brief Set the name of NodeHandle.
   */
  MROS_DllAPI void setNodeName(const std::string& name);

  /**
   * \brief Get the CallerId.
   */
  MROS_DllAPI std::string getCallerId(unsigned char* data, int count);

  /**
   * \brief Set the CallerId.
   */
  MROS_DllAPI void setCallerId(unsigned char* data, int count, const std::string& _callerid = "");

  /**
   * \brief Returns the mros domain id.
   *  Only nodes with the same domain id can send and receive messages.
   *  You can configure it through the environment variable "MROS_DOMAIN_ID", 
   *  for example: export MROS_DOMAIN_ID="00:00:00:00:00:00"
   */
  MROS_DllAPI const std::string& getDomainID();

  namespace tf {
    class CallbackQueue;
  }
  std::shared_ptr<mros::tf::CallbackQueue> getGlobalCallbackQueue();


  namespace package {
    /**
     * \brief Returns the fully-qualified path to a package.
     */
    MROS_DllAPI std::string getPath(const std::string& package);

    
    /**
     * \brief Return the storage path of the package.
     */
    MROS_DllAPI std::string getExecPath();
  }

  namespace path {
    /**
     * \brief Return path "xxx/install".
     */
    MROS_DllAPI std::string root();

    /**
     * \brief Return path "xxx/install/bin".
     */    
    MROS_DllAPI std::string bin();

    /**
     * \brief Return path "xxx/install/lib".
     */    
    MROS_DllAPI std::string lib();

    /**
     * \brief Return path "xxx/install/etc".
     */    
    MROS_DllAPI std::string etc();
  }
  
  namespace guid {
    /**
     * \brief Produce a uuid string.
     */
    MROS_DllAPI std::string uuid();
    
    /**
     * \brief Produce a uuid value.
     */
    MROS_DllAPI uint32_t uint32();
    MROS_DllAPI uint32_t uint32(const std::string& text);
  }

  namespace cpu {
    /**
     * \brief Set CPU affinity.
     */
    MROS_DllAPI bool setaffinity(int cpu);
  }
}
#endif