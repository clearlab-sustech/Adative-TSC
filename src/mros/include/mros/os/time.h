#ifndef MROS_TIME_H_
#define MROS_TIME_H_

#include <math.h>
#include <stdint.h>
#include <string>
#include <mutex>
#include <limits>
#include <iostream>
#include <mros/macros.h>
#include "mros/os/duration.h"

namespace mros
{
MROS_DllAPI void normalizeSecNSec(uint32_t &sec, uint32_t &nsec);
MROS_DllAPI void normalizeSecNSec(uint64_t &sec, uint64_t &nsec);
MROS_DllAPI void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec);

enum TimeType { SYS_TIME, SIM_TIME, WALL_TIME };

class MROS_DllAPI Time
{
public:
  uint32_t sec, nsec;

  Time() : sec(0), nsec(0) {}
  Time(double t) { fromSec(t); }
  Time(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
  {
    normalizeSecNSec(sec, nsec);
  }

  double toMSec() const
  {
    return (double)sec * 1000 + 1e-6 * (double)nsec;
  };

  double toSec() const
  {
    return (double)sec + 1e-9 * (double)nsec;
  }
  
  Time& fromSec(double t)
  {
    sec = (uint32_t) floor(t);
    nsec = (uint32_t) round((t - sec) * 1e9);
    return *this;
  }

  uint64_t toNsec()
  {
    return (uint64_t)sec * 1000000000ull + (uint64_t)nsec;
  }
  
  bool isZero() const { return sec == 0 && nsec == 0; }
  
  Time& fromNSec(int64_t t);
  Duration operator-(const Time &rhs) const;
  Time operator-(const Duration &rhs) const;
  Time operator+(const Duration &rhs) const;
  Time& operator+=(const Duration &rhs);
  Time& operator-=(const Duration &rhs);
  bool operator==(const Time &rhs) const;
  bool operator!=(const Time &rhs) const { return !((*this) == rhs); }
  bool operator<(const Time &rhs) const;
  bool operator>(const Time &rhs) const;
  bool operator<=(const Time &rhs) const;
  bool operator>=(const Time &rhs) const;

  static std::mutex sim_time_mutex_;
  static TimeType use_time_type_;
  static Time sim_time_;
  static Time now();
  static Time WallTime();
  static Time monotonic();
  static void setTime(const Time& now);
  static void setType(TimeType type);
  static TimeType getType();
  static bool sleepUntil(const Time& end, bool wall = false);
  static std::string getStringWallTime(bool ms = false);
};

std::ostream &operator <<(std::ostream &os, const Time &rhs);

const Time TIME_MAX(std::numeric_limits<uint32_t>::max(), 999999999);
const Time TIME_MIN(0, 1);

}

#endif
