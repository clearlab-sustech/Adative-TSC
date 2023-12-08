
#include "generation/LegLogic.h"

namespace clear {
namespace legged_robot {

std::vector<scalar_t>
getTimeOfNextTouchDown(scalar_t time_cur,
                       const std::shared_ptr<ModeSchedule> mode_schedule) {
  auto contact_flag_last = modeNumber2StanceLeg(
      mode_schedule->getModeFromPhase(mode_schedule->eventPhases().front()));
  std::vector<scalar_t> td_time_array;
  td_time_array.resize(contact_flag_last.size());

  for (size_t i = 0; i < contact_flag_last.size(); i++) {
    td_time_array[i] = time_cur + mode_schedule->duration();
    for (const auto &phase : mode_schedule->eventPhases()) {
      auto contact_flag =
          modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
      if (contact_flag[i] && !contact_flag_last[i]) {
        td_time_array[i] = time_cur + phase * mode_schedule->duration();
        break;
      }
      contact_flag_last = contact_flag;
    }
  }
  return td_time_array;
}

std::vector<scalar_t>
getTimeOfNextLiftOff(scalar_t time_cur,
                     const std::shared_ptr<ModeSchedule> mode_schedule) {

  auto contact_flag_last = modeNumber2StanceLeg(
      mode_schedule->getModeFromPhase(mode_schedule->eventPhases().front()));
  std::vector<scalar_t> lift_time_array;
  lift_time_array.resize(contact_flag_last.size());

  for (size_t i = 0; i < contact_flag_last.size(); i++) {
    lift_time_array[i] = time_cur + mode_schedule->duration();
    for (const auto &phase : mode_schedule->eventPhases()) {
      auto contact_flag =
          modeNumber2StanceLeg(mode_schedule->getModeFromPhase(phase));
      if (!contact_flag[i] && contact_flag_last[i]) {
        lift_time_array[i] = time_cur + phase * mode_schedule->duration();
        break;
      }
      contact_flag_last = contact_flag;
    }
  }
  return lift_time_array;
}
} // namespace legged_robot
} // namespace clear
