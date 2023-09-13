//
// Created by wenchun on 3/26/21.
//

#ifndef XIAOTIANHYBRID_USERINTERFACE_H
#define XIAOTIANHYBRID_USERINTERFACE_H

#include "DataSets.h"
#include "joystick.h"
#include <memory>

class UserInterface {
public:
  UserInterface(UserParameterHandler *param, UserCmd *userCmd);

  void update(const EstimatedState* estimatedState);


private:
  UserParameterHandler *param;
  UserCmd *userCmd;
  GamepadCommand gameCmd;
  JoystickEvent event;
  std::shared_ptr<Joystick> joystick;
  int iter = 0;
  bool fixed_vel = false;
};


#endif //XIAOTIANHYBRID_USERINTERFACE_H
