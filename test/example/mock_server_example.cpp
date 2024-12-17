#include <fmt/core.h>
#include <fstream>
#include <iostream>
#include <mutex>

#include <franka/robot_model.h>
#include "aescape_mock_server.hpp"
#include <magic_enum.hpp>
#include <tracy/Tracy.hpp>

/*
 * Mock Server Example
 * This example demonstrates how to use the MockServer class and respond to different commands
 * from the franka::robot class.
 */


const std::string kUrdfPath = std::string(FRANKA_TEST_SOURCE_DIR) + "/fr3.urdf";
constexpr size_t kControlLoopCount = 4;
/*
 * Read a file to a string
 */
std::string readFileToString(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return "";
  }

  std::ostringstream oss;
  oss << file.rdbuf();
  file.close();
  return oss.str();
}

/*
 * Struct to hold the modes of the robot
 */
int main(int argc, char* argv[]) {
  if (argc < 2) {
    LOG_INFO("Usage: {} <server_ip>\nUse 127.0.0.1 if unsure.", argv[0]);
    return 1;
  }
  bool connected = false;
  std::condition_variable cv;
  std::mutex mut;

  LOG_INFO("Starting server on port: {}", argv[1]);
  auto on_conn = [&connected, &cv, &mut](const RobotTypes::Connect::Request& req) {
    LOG_INFO("Connected! port: {}", req.udp_port);
    std::unique_lock<std::mutex> lck(mut);
    connected = true;
    cv.notify_one();
    return RobotTypes::Connect::Response(RobotTypes::Connect::Status::kSuccess);
  };

  AescapeMockServer<RobotTypes> server(
    std::make_shared<DummyBackend>(),
    1000,
    on_conn,
    0,
    std::string(argv[1]));

  std::unique_lock<std::mutex> lck(mut);
  while (!connected) {
    cv.wait(lck);
  }
  lck.unlock();

  // Prepare for load model
  std::ifstream model_library_stream(
      FRANKA_TEST_BINARY_DIR + "/libfcimodels.so"s,
      std::ios_base::in | std::ios_base::binary | std::ios_base::ate);
  std::vector<char> buffer;
  buffer.resize(model_library_stream.tellg());
  model_library_stream.seekg(0, std::ios::beg);
  if (!model_library_stream.read(buffer.data(), buffer.size())) {
    throw std::runtime_error("Model test: Cannot load mock libfcimodels.so");
  }

  // Test methods used in the controls repo
  server
      .waitForCommand<r::GetRobotModel>([](const typename r::GetRobotModel::Request& /*request*/) {
        ZoneScopedN("LoadModelCommand");
        LOG_INFO("--- Receive load model request -> loadModel");
        LOG_INFO(" ++ GetRobotModel");
        const auto urdf_string = readFileToString(kUrdfPath);
        auto model = std::make_unique<franka::RobotModel>(urdf_string);
        return r::GetRobotModel::Response(r::GetRobotModel::Status::kSuccess, urdf_string);
      })
      .spinOnce()
      .generic([&buffer, &server](decltype(server)::Socket& tcp_socket, decltype(server)::Socket&) {
        ZoneScopedN("LoadModelResponse");
        LOG_INFO(" ++ LoadModelLibrary");
        r::CommandHeader header;
        server.receiveRequest<r::LoadModelLibrary>(tcp_socket, &header);
        server.sendResponse<r::LoadModelLibrary>(
            tcp_socket,
            r::CommandHeader(
                r::Command::kLoadModelLibrary, header.command_id,
                sizeof(r::CommandMessage<r::LoadModelLibrary::Response>) + buffer.size()),
            r::LoadModelLibrary::Response(r::LoadModelLibrary::Status::kSuccess));
        tcp_socket.sendBytes(buffer.data(), buffer.size());
      })
      .spinOnce()
      .waitForCommand<r::AutomaticErrorRecovery>(
          [](const typename r::AutomaticErrorRecovery::Request&) {
            ZoneScopedN("ErrorRecoveryCmd");
            LOG_INFO("--- Automatic Error Recovery");
            return r::AutomaticErrorRecovery::Response(r::AutomaticErrorRecovery::Status::kSuccess);
          })
      .spinOnce()
      .waitForCommand<r::SetCollisionBehavior>(
          [](const typename r::SetCollisionBehavior::Request&) {
            ZoneScopedN("CollisonBehaviorCmd");
            LOG_INFO("--- SetCollisionBehavior");
            return r::SetCollisionBehavior::Response(r::SetCollisionBehavior::Status::kSuccess);
          })
      .spinOnce()
      .waitForCommand<r::SetJointImpedance>([](const typename r::SetJointImpedance::Request&) {
        ZoneScopedN("JointImpedanceCmd");
        LOG_INFO("--- SetJointImpedance");
        return r::SetJointImpedance::Response(r::SetJointImpedance::Status::kSuccess);
      })
      .spinOnce()
      .waitForCommand<r::SetCartesianImpedance>(
          [](const typename r::SetCartesianImpedance::Request&) {
            ZoneScopedN("CartesianImpedanceCmd");
            LOG_INFO("--- SetCartesianImpedance");
            return r::SetCartesianImpedance::Response(r::SetCartesianImpedance::Status::kSuccess);
          })
      .spinOnce();

  LOG_INFO("--- Read current state ->readOnce");  // Can miss. Try twice here.
  server.handleReadOnce().spinOnce();
  server.handleReadOnce().spinOnce();

  LOG_INFO("--- Run control loop ->control");
  LOG_INFO(" ++ ControlLoop Constructor");
  LOG_INFO("    -- Handle robot.startMotion()");
  // Send Move response again for startMotion haven't done a RobotState update yet
  // and it checks for a Move response. It does not send another request so simply send a response
  server.waitForMove().spinOnce().respondMove().spinOnce();
  LOG_INFO("    -- Handle robot.update()");
  server.handleUpdate().spinOnce();
  LOG_INFO(" ++ ControlLoop Constructor DONE");

  LOG_INFO(" ++ operator()()");
  
  server.stateLoop();
  LOG_INFO("    -- Finish control callback loop");

  LOG_INFO("    -- Start robot.finishMotion");
  server.handleReadOnce()
      .spinOnce()
      .respondMove()  // No request. Only waiting for response.
      .spinOnce();
  LOG_INFO("    -- Finish robot.finishMotion");

  LOG_INFO("--- Stop");
  server.waitForCommand<r::StopMove>([](const r::StopMove::Request&) {
    LOG_INFO(" ++ Received StopMove");
    return r::StopMove::Response(r::StopMove::Status::kSuccess);
  });
  LOG_INFO("DONE");
}
