#ifndef MROS_SIGNALS_H
#define MROS_SIGNALS_H
#include <functional>
#include <map>
#include <mutex>

namespace mros
{
template <typename... Args>
class Signals2 {
 public:
  Signals2() : current_id_(0) {}
 
  Signals2(Signals2 const& other) : current_id_(0) {}
 
  template <typename F, typename... A>
  int connect_member(F&& f, A&& ... a) const {
    std::unique_lock<std::mutex> lock(mutex_);
    slots_.insert({++current_id_, std::bind(f, a...)});
    return current_id_;
  }
 
  int connect(std::function<void(Args...)> const& slot) const {
    std::unique_lock<std::mutex> lock(mutex_);
    slots_.insert(std::make_pair(++current_id_, slot));
    return current_id_;
  }
 
  void disconnect(int id) const {
    std::unique_lock<std::mutex> lock(mutex_);
    slots_.erase(id);
  }
 
  void disconnect_all() const {
    std::unique_lock<std::mutex> lock(mutex_);
    slots_.clear();
  }
 
  void emitter(Args... p) {
    std::unique_lock<std::mutex> lock(mutex_);
    for(auto it : slots_) {
      it.second(p...);
    }
  }
 
 private:
  mutable std::map<int, std::function<void(Args...)> > slots_;
  mutable int current_id_;
  mutable std::mutex mutex_;
};
}
#endif // MROS_SIGNALS_H
