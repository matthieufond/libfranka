#pragma once

#include <research_interface/robot/rbk_types.h>

namespace aescape {
struct BackendInterface {
  virtual bool updateRobotState(research_interface::robot::RobotState& state) = 0;
  virtual bool updateRobotCommand(const research_interface::robot::RobotCommand& cmd) = 0;
};
}  // namespace aescape