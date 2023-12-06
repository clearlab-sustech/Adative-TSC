#pragma once

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <core/types.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace clear {

namespace legged_robot {
template <typename T> using feet_array_t = std::array<T, 2>;
using contact_flag_t = feet_array_t<bool>;

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

enum ModeNumber { // {LF, RF}
  FLY = 0,
  LF = 1,
  RF = 2,
  STANCE = 3,
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline contact_flag_t modeNumber2StanceLeg(const size_t &modeNumber) {
  contact_flag_t stanceLegs; // {LF, RF, LH, RH}

  switch (modeNumber) {
  case 0:
    stanceLegs = contact_flag_t{false, false};
    break; // 0:  0-leg-stance
  case 1:
    stanceLegs = contact_flag_t{true, false};
    break; // 1:  LF
  case 2:
    stanceLegs = contact_flag_t{false, true};
    break; // 2:  RF
  case 3:
    stanceLegs = contact_flag_t{true, true};
    break; // 3:  STANCE
  }

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t stanceLeg2ModeNumber(const contact_flag_t &stanceLegs) {
  return static_cast<size_t>(stanceLegs[0]) +
         2 * static_cast<size_t>(stanceLegs[1]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::string modeNumber2String(const size_t &modeNumber) {
  // build the map from mode number to name
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[LF] = "LF";
  modeToName[RF] = "RF";
  modeToName[STANCE] = "STANCE";
  return modeToName[modeNumber];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t string2ModeNumber(const std::string &modeString) {
  // build the map from name to mode number
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["LF"] = LF;
  nameToMode["RF"] = RF;
  nameToMode["STANCE"] = STANCE;
  return nameToMode[modeString];
}

} // namespace legged_robot

} // end of namespace clear
