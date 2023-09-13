// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#include "joystick.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include "unistd.h"

Joystick::Joystick()
{
  openPath("/dev/input/js0");
}

Joystick::Joystick(int joystickNumber)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath)
{
  openPath(devicePath);
}

Joystick::Joystick(std::string devicePath, bool blocking)
{
  openPath(devicePath, blocking);
}

void Joystick::openPath(std::string devicePath, bool blocking)
{
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool Joystick::sample(JoystickEvent* event)
{
  int bytes = read(_fd, event, sizeof(*event)); 

  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return bytes == sizeof(*event);
}

bool Joystick::isFound()
{
  return _fd >= 0;
}

Joystick::~Joystick()
{
  close(_fd);
}

void Joystick::updateCommand(JoystickEvent* event, GamepadCommand& cmd) {
  if (sample(event)) { // Attempt to sample an event from the joystick
    // button
    if (event->isButton() && event->number == 0)
      cmd.A = event->value;
    
    if (event->isButton() && event->number == 1)
      cmd.B = event->value;
    
    if (event->isButton() && event->number == 2)
      cmd.X = event->value;
    
    if (event->isButton() && event->number == 3)
      cmd.Y = event->value;
    
    if (event->isButton() && event->number == 4)
      cmd.LB = event->value;
    
    if (event->isButton() && event->number == 5)
      cmd.RB = event->value;

    if (event->isButton() && event->number == 6)
      cmd.BACK = event->value;
    
    if (event->isButton() && event->number == 7)
      cmd.START = event->value;
    
    // Axis
    if (event->isAxis() && event->number == 1)
      cmd.leftStickAnalog[0] = -(event->value)/double(event->MAX_AXES_VALUE);

    if (event->isAxis() && event->number == 0)
      cmd.leftStickAnalog[1] = -(event->value)/double(event->MAX_AXES_VALUE);

    if (event->isAxis() && event->number == 4)
      cmd.rightStickAnalog[0] = -(event->value)/double(event->MAX_AXES_VALUE);

    if (event->isAxis() && event->number == 3)
      cmd.rightStickAnalog[1] = -(event->value)/double(event->MAX_AXES_VALUE);

    cmd.applyDeadband(0.001);
    // printf("%s\n", cmd.toString().c_str());
  }
}

std::ostream& operator<<(std::ostream& os, const JoystickEvent& e)
{
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}


