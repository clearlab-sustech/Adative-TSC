#ifndef MROS_THREADPOOL_H_
#define MROS_THREADPOOL_H_
#include <stdint.h>
#include <vector>
#include <utility>
#include <deque>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <mros/macros.h>
#include "mros/os/log.h"
#include "mros/os/duration.h"
#ifndef _WIN32
#include <unistd.h>
#include <sys/prctl.h>
#endif

namespace mros {
class MROS_DllAPI ThreadPool{
public:
  typedef std::function<void()> Task;

  ThreadPool(int init_threads, const std::string& name = "mros_pool") {
    tasking_ = 0;
    started_ = true;
    name_ = name;
    threads_count_ = init_threads;
    for (int i = 0; i < threads_count_; i++) {
      std::thread tid(std::bind(&ThreadPool::thread_loop, this, i));
      tid.detach();
    }
  }
  
  ~ThreadPool() {
    shutdown();
  }

  void shutdown(int timeout = 3 /* seconds */) {
    {
      started_ = false;
      std::unique_lock<std::mutex> lock(mutex_);
      tasks_.clear();
      cond_.notify_all();
    }

    wait_completed(timeout);
  }
  
  void wait_completed(int timeout = 3 /* seconds */) {
    int count = (timeout * 1000 * 1000) / 1000;
    while(tasking_ > 0 && count > 0) {
      mros::Duration(1.0 / 1000).wallSleep();
      count--;
    }
    if (tasking_ > 0) {
      mros_log_warn("Blocked - %s wait_completed timeout=%ds", name_.c_str(), timeout);
      std::cout << "Warning: " << "Blocked - " << name_ <<  " wait_completed timeout=" << timeout << "s" << std::endl;
    }
  }

  bool empty() {
    return (tasks_.size() == 0);
  }

  uint32_t size() {
    return tasks_.size();
  }
  
  void schedule(const Task& task, int32_t queue_size = -1) {
    if (started_) {
      std::unique_lock<std::mutex> lock(mutex_);
      while ((queue_size > 0 && tasks_.size() >= queue_size) 
        || (queue_size == -1 && tasks_.size() >= 10000)) {
        tasks_.pop_front();
      }
      tasks_.push_back(task);
      cond_.notify_one();
    }
  }

  void clear() {
    std::unique_lock<std::mutex> lock(mutex_);
    tasks_.clear();
    cond_.notify_all();
  }

private:
  void thread_loop(int idx) {
    std::string pname = name_;
    if (threads_count_ > 1) {
      pname += "_" + std::to_string(idx);
    }

    if (pname.length() > 15) {
      pname = pname.substr(pname.length() - 15);
    }

#ifndef _WIN32
    prctl(PR_SET_NAME, pname.c_str());
#endif

    while(started_) {
      std::unique_lock<std::mutex> tasking_lock(tasking_mutex_);
      tasking_++;
      tasking_lock.unlock();

      Task task = take();
      if(started_ && task) {
        mros::Time now = mros::Time::now(); 

        task();
        
        std::size_t task_size = tasks_.size();
        mros::Duration diff = mros::Time::now() - now;
        if (diff.toMSec() >= 50.0) {
          mros_log_once_warn("%s timeescape %.2f ms, remained task: %ld", name_.c_str(), diff.toMSec(), task_size);
        }
      }

      tasking_lock.lock();
      tasking_--;
      tasking_lock.unlock();
    }
  }
  
  Task take() {
    std::unique_lock<std::mutex> lock(mutex_);
    while(tasks_.empty() && started_) {
      cond_.wait(lock);
    }
    Task task = nullptr;
    if(started_ && !tasks_.empty()) {
      task = tasks_.front();
      tasks_.pop_front();
    }
    return task;
  }

  typedef std::deque<Task> Tasks;

  Tasks tasks_;

  std::mutex mutex_;
  std::condition_variable cond_;
  bool started_;
  std::mutex tasking_mutex_;
  int tasking_;
  std::string name_;
  int threads_count_;
};

}

#endif //MROS_THREADPOOL_H_

