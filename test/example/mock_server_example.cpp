#include <fstream>
#include <mutex>

#include <backend/forward_dynamics_backend.h>
#include <backend/proportional_controller_backend.h>
#include <franka/robot_model.h>
#include <magic_enum.hpp>
#include <tracy/Tracy.hpp>
#include "aescape_mock_server.h"

/*
 * Mock Server Example
 * This example demonstrates how to use the MockServer class and respond to different commands
 * from the franka::robot class.
 */

const std::string kUrdfPath =
    std::string(FRANKA_TEST_SOURCE_DIR) + "/aescape_simulation_panda.urdf";

int main(int argc, char* argv[]) {
  if (argc < 2) {
    LOG_INFO("Usage: {} <server_ip>\nUse 127.0.0.1 if unsure.", argv[0]);
    return 1;
  }
  // Set initial state
  Eigen::VectorXd q(kNumJoints);
  q << 0.054, -1.225, -0.008, -1.820, -0.019, 0.697, -0.021;  //Idle pose
  AescapeMockServer<RobotTypes> server(std::make_shared<aescape::ProportionalControllerBackend>(q),
                                       1000, on_conn, 0, std::string(argv[1]));
  server.run();
}
