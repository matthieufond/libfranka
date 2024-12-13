#include <fmt/core.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <iostream>

/*
 * This is a minimal client example to test the mock server example.
 * It contains all the libfranka calls used in the controls repo.
 *
 * */

#define LOG_INFO(str, ...) logMessage(__FILE__, __LINE__, "INFO", str, ##__VA_ARGS__)

template <typename... T>
void logMessage(const char* filename,
                int line,
                const char* level,
                const std::string& format_str,
                T&&... args) {
  std::cout << fmt::format("[{}]{}:{}: ", level, filename, line)
            << fmt::format(format_str, std::forward<T>(args)...) << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    LOG_INFO("Usage: {} <server_ip>\nUse 127.0.0.1 if unsure.", argv[0]);
    return 1;
  }

  try {
    auto robotip = std::string(argv[1]);
    auto real_time_config =
        franka::RealtimeConfig::kIgnore;  // TODO (Cat): Need this when updating controls driver

    auto robot = std::make_shared<franka::Robot>(robotip, real_time_config);
    LOG_INFO("Create franka robot");

    auto model = std::make_shared<franka::Model>(robot->loadModel());
    LOG_INFO("Create franka model");

    robot->automaticErrorRecovery();
    LOG_INFO("automaticErrorRecovery");

    robot->setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 25.0, 25.0, 25.0}},
        {{100.0, 100.0, 100.0, 100.0, 25.0, 25.0, 25.0}}, {{100, 100, 100, 100, 25, 25, 25}},
        {{100, 100, 100, 100, 25, 25, 25}}, {{300.0, 300.0, 300.0, 300.0, 300.0, 300.0}},
        {{300.0, 300.0, 300.0, 300.0, 300.0, 300.0}}, {{300.0, 300.0, 300.0, 300.0, 300.0, 300.0}},
        {{300.0, 300.0, 300.0, 300.0, 300.0, 300.0}});
    LOG_INFO("setCollisionBehavior");

    robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    LOG_INFO("setJointImpedance");

    robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    LOG_INFO("setCartesianImpedance");

    robot->readOnce();
    LOG_INFO("Read once");

    LOG_INFO("Start Torque control callback");
    static constexpr int kDOF = 7;
    std::array<double, kDOF> tau_command{};
    franka::Torques torques(tau_command);
    size_t count = 0;
    auto torque_control_cb = [&](const franka::RobotState&, franka::Duration) -> franka::Torques {
      //LOG_INFO("FrankaDriver::torque_control_cb");
      if (++count > 0) {
        return torques;
      }
      return franka::MotionFinished(torques);
    };
    robot->control(torque_control_cb, true);
    LOG_INFO("End Torque control callback");

    robot->stop();
    LOG_INFO("Stop");

  } catch (const franka::Exception& e) {
    LOG_INFO(e.what());
    LOG_INFO("Not able to initialize robot for Franka_driver due to exception!");
  }
}
