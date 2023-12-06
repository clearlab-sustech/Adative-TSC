#pragma once

#include <asserts/gait/ModeSchedule.h>
#include <asserts/gait/MotionPhaseDefinition.h>

#include <memory>
#include <vector>

namespace clear {
namespace legged_robot {

struct TimeInterval {
  scalar_t start;
  scalar_t end;
};

std::vector<scalar_t>
getTimeOfNextTouchDown(scalar_t time_cur,
                       const std::shared_ptr<ModeSchedule> mode_schedule);

std::vector<scalar_t>
getTimeOfNextLiftOff(scalar_t time_cur,
                     const std::shared_ptr<ModeSchedule> mode_schedule);
} // namespace legged_robot
} // namespace clear
