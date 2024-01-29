#ifndef _MROS_RATE_H_
#define _MROS_RATE_H_

#include <math.h>
#include <stdint.h>
#include <limits>
#include <iostream>
#include <mros/macros.h>
#include "mros/os/time.h"
#include "mros/os/duration.h"

namespace mros
{
class MROS_DllAPI Rate
{
public:
  /**
   * @brief  Constructor, creates a Rate
   * @param  frequency The desired rate to run at in Hz
   */
  Rate(double frequency);
  explicit Rate(const mros::Duration&);

  /**
   * @brief  Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
   * @return True if the desired rate was met for the cycle, false otherwise.
   */
  bool sleep();

  /**
   * @brief  Sets the start time for the rate to now
   */
  void reset();

  /**
   * @brief  Get the actual run time of a cycle from start to sleep
   * @return The runtime of the cycle
   */
  mros::Duration cycleTime() const;

  /**
   * @brief Get the expected cycle time -- one over the frequency passed in to the constructor
   */
  mros::Duration expectedCycleTime() const { return expected_cycle_time_; }

private:
  mros::Time start_;
  mros::Duration expected_cycle_time_, actual_cycle_time_;
};
}

#endif