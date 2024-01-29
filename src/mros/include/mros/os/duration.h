#ifndef _MROS_DURATION_H_
#define _MROS_DURATION_H_

#include <math.h>
#include <stdint.h>
#include <limits>
#include <iostream>
#include <mros/macros.h>

namespace mros
{

MROS_DllAPI void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec);

class MROS_DllAPI Duration
{
public:
  int32_t sec, nsec;

  Duration() : sec(0), nsec(0) {}
  Duration(double t) { fromSec(t); }
  Duration(int32_t _sec, int32_t _nsec) : sec(_sec), nsec(_nsec)
  {
    normalizeSecNSecSigned(sec, nsec);
  }

  double toSec() const
  {
    return (double)sec + 1e-9 * (double)nsec;
  }

  double toMSec() const
  {
    return (double)sec * 1000 + 1e-6 * (double)nsec;
  }

  uint64_t toNsec()
  {
    return (uint64_t)sec * 1000000000ull + (uint64_t)nsec;
  }
  
  Duration& fromSec(double t)
  {
    sec = (int32_t) floor(t);
    nsec = (int32_t) ((t - (double)sec) * 1e9);
    return *this;
  }

  Duration& fromNSec(int64_t t)
  {
    sec  = (int32_t)(t / 1000000000);
    nsec = (int32_t)(t % 1000000000);

    normalizeSecNSecSigned(sec, nsec);

    return *this;
  }
  
  Duration operator+(const Duration &rhs) const;
  Duration operator*(double scale) const;
  Duration operator-(const Duration &rhs) const;
  Duration operator-() const;
  Duration& operator+=(const Duration &rhs);
  Duration& operator-=(const Duration &rhs);
  Duration& operator*=(double scale);
  bool operator==(const Duration &rhs) const;
  bool operator!=(const Duration &rhs) const { return !((*this) == rhs); }
  bool operator<(const Duration &rhs) const;
  bool operator>(const Duration &rhs) const;
  bool operator<=(const Duration &rhs) const;
  bool operator>=(const Duration &rhs) const;
  bool isZero() const;
  bool sleep() const;
  bool wallSleep() const;
};

MROS_DllAPI std::ostream &operator <<(std::ostream &os, const Duration &rhs);

const Duration MROS_DURATION_MAX(std::numeric_limits<int32_t>::max(), 999999999);
const Duration MROS_DURATION_MIN(std::numeric_limits<int32_t>::min(), 0);

}

#endif

