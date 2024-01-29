#ifndef _MROS_H_
#define _MROS_H_
#include <thread>
#include <stdint.h>
#include <mutex>
#include <memory>
#include <sstream>
#include <random>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <functional>
#ifdef WIN32
#define NOMINMAX
#include <Windows.h>
#include <tchar.h>
#else
#include <unistd.h>
#endif
#include <mros/macros.h>
#include "mros/os/time.h"
#include "mros/os/rate.h"
#include "mros/os/node_handle_base.h"

#ifdef MROS_POINTER_COMPATIBILITY_IMPLEMENTED
#include <boost/shared_ptr.hpp>
namespace mros {

template<class SharedPointer>
struct Holder
{
  SharedPointer p;

  explicit Holder(const SharedPointer & p)
  : p(p) {}
  Holder(const Holder & other)
  : p(other.p) {}
  Holder(Holder && other)
  : p(std::move(other.p)) {}

  void operator()(...) {p.reset();}
};

template<class T>
inline std::shared_ptr<T> to_std_ptr(const boost::shared_ptr<T> & p)
{
  typedef Holder<std::shared_ptr<T>> H;
  if (H * h = boost::get_deleter<H>(p)) {
    return h->p;
  } else {
    return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
  }
}

template<class T>
inline boost::shared_ptr<T> to_boost_ptr(const std::shared_ptr<T> & p)
{
  typedef Holder<boost::shared_ptr<T>> H;
  if (H * h = std::get_deleter<H>(p)) {
    return h->p;
  } else {
    return boost::shared_ptr<T>(p.get(), Holder<std::shared_ptr<T>>(p));
  }
}

}
#endif
#endif

