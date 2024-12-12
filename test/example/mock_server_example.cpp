#include <fmt/core.h>
#include <fstream>
#include <iostream>
#include <mutex>

#include <franka/robot_model.h>
#include <magic_enum.hpp>
#include "mock_server.h"

/*
 * Mock Server Example
 * This example demonstrates how to use the MockServer class and respond to different commands
 * from the franka::robot class.
 */

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

using namespace std::string_literals;
namespace r = research_interface::robot;
using move_ctrl_mode_t = r::Move::ControllerMode;
using move_motion_mode_t = r::Move::MotionGeneratorMode;

const std::string kUrdfPath = std::string(FRANKA_TEST_SOURCE_DIR) + "/fr3.urdf";
constexpr size_t kControlLoopCount = 4;

/*
 * Convert enums from research_interface::robot::Move::ControllerMode to
 * research_interface::robot::ControllerMode
 */
r::ControllerMode convertMode(const move_ctrl_mode_t& mode) {
  switch (mode) {
    case move_ctrl_mode_t::kCartesianImpedance:
      return r::ControllerMode::kCartesianImpedance;
    case move_ctrl_mode_t::kExternalController:
      return r::ControllerMode::kExternalController;
    case move_ctrl_mode_t::kJointImpedance:
      return r::ControllerMode::kJointImpedance;
    default:
      LOG_INFO("Unknown Controller Mode: {}", mode);
      return r::ControllerMode::kOther;
  }
}

/*
 * Convert enums from research_interface::robot::Move::MotionGeneratorMode to
 * research_interface::robot::MotionGeneratorMode
 */

r::MotionGeneratorMode convertMode(const move_motion_mode_t& mode) {
  switch (mode) {
    case move_motion_mode_t::kCartesianPosition:
      return r::MotionGeneratorMode::kCartesianPosition;
    case move_motion_mode_t::kCartesianVelocity:
      return r::MotionGeneratorMode::kCartesianVelocity;
    case move_motion_mode_t::kJointPosition:
      return r::MotionGeneratorMode::kJointPosition;
    case move_motion_mode_t::kJointVelocity:
      return r::MotionGeneratorMode::kJointVelocity;
    default:
      return r::MotionGeneratorMode::kIdle;
  }
}

/*
 * Create a success move response
 */
r::Move::Response moveSuccessResp() {
  return r::Move::Response(r::Move::Status::kSuccess);
}

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
struct Modes {
  std::mutex mut;
  r::ControllerMode ctrl_mode{r::ControllerMode::kOther};
  r::MotionGeneratorMode motion_mode{r::MotionGeneratorMode::kIdle};
  r::RobotMode robot_mode{r::RobotMode::kIdle};
};

/*
 * Aescape MockServer example
 */
template <typename C>
class AescapeMockServer : public MockServer<C> {
 public:
  AescapeMockServer(
      typename MockServer<C>::ConnectCallbackT on_connect = MockServer<C>::ConnectCallbackT(),
      uint32_t sequence_number = 0,
      std::string ip = "127.0.0.1")
      : MockServer<C>(on_connect, sequence_number, std::move(ip)) {}

  /*
   * Handle robot.readOnce() by responding with the robot state
   */
  AescapeMockServer& handleReadOnce() {
    MockServer<C>::template onSendUDP<r::RobotState>(
        [&modes = this->modes_](r::RobotState& robot_state) {
          std::lock_guard<std::mutex> modes_lck(modes.mut);
          robot_state.robot_mode = modes.robot_mode;
          robot_state.controller_mode = modes.ctrl_mode;
          robot_state.motion_generator_mode = modes.motion_mode;
          LOG_INFO("    -- Send robot_state");
        });
    return *this;
  }

  /*
   * Handle robot.update() by sending the robot state and receiving the robot command
   */
  AescapeMockServer& handleUpdate() {
    handleReadOnce().spinOnce().onReceiveRobotCommand(
        [&modes = this->modes_](const r::RobotCommand& cmd) {
          LOG_INFO("    -- Received control robot command");
          if (cmd.motion.motion_generation_finished) {
            LOG_INFO("    -- Motion generation finished");
            std::lock_guard<std::mutex> modes_lck(modes.mut);
            modes.motion_mode = r::MotionGeneratorMode::kIdle;
            modes.ctrl_mode = r::ControllerMode::kOther;
          }
        });
    return *this;
  }

  /*
   * Wait for a move command and responding with a success
   */
  AescapeMockServer& waitForMove() {
    MockServer<C>::template waitForCommand<r::Move>(
        [&modes = this->modes_](const r::Move::Request& req) {
          std::lock_guard<std::mutex> modes_lck(modes.mut);
          modes.motion_mode = convertMode(req.motion_generator_mode);
          modes.ctrl_mode = convertMode(req.controller_mode);
          modes.robot_mode = r::RobotMode::kMove;
          LOG_INFO("    -- Move Received: Controller Mode: {}, Motion Mode: {}",
                   magic_enum::enum_name(modes.ctrl_mode),
                   magic_enum::enum_name(modes.motion_mode));
          return moveSuccessResp();
        },
        &move_id_);
    return *this;
  }

  /*
   * Respond to the move command with a success without waiting for a request
   */
  AescapeMockServer& respondMove() {
    MockServer<C>::template sendResponse<r::Move>(move_id_, moveSuccessResp);
    return *this;
  }

  /*
   * Override MockServer SpinOnce to return the derived AescapeMockServer class instead
   */
  AescapeMockServer& spinOnce() override {
    MockServer<C>::spinOnce();
    return *this;
  }

  ~AescapeMockServer() = default;

 private:
  Modes modes_;
  uint32_t move_id_;
};

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

  AescapeMockServer<RobotTypes> server(on_conn, 0, std::string(argv[1]));

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
        LOG_INFO("--- Receive load model request -> loadModel");
        LOG_INFO(" ++ GetRobotModel");
        const auto urdf_string = readFileToString(kUrdfPath);
        auto model = std::make_unique<franka::RobotModel>(urdf_string);
        return r::GetRobotModel::Response(r::GetRobotModel::Status::kSuccess, urdf_string);
      })
      .spinOnce()
      .generic([&buffer, &server](decltype(server)::Socket& tcp_socket, decltype(server)::Socket&) {
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
            LOG_INFO("--- Automatic Error Recovery");
            return r::AutomaticErrorRecovery::Response(r::AutomaticErrorRecovery::Status::kSuccess);
          })
      .spinOnce()
      .waitForCommand<r::SetCollisionBehavior>(
          [](const typename r::SetCollisionBehavior::Request&) {
            LOG_INFO("--- SetCollisionBehavior");
            return r::SetCollisionBehavior::Response(r::SetCollisionBehavior::Status::kSuccess);
          })
      .spinOnce()
      .waitForCommand<r::SetJointImpedance>([](const typename r::SetJointImpedance::Request&) {
        LOG_INFO("--- SetJointImpedance");
        return r::SetJointImpedance::Response(r::SetJointImpedance::Status::kSuccess);
      })
      .spinOnce()
      .waitForCommand<r::SetCartesianImpedance>(
          [](const typename r::SetCartesianImpedance::Request&) {
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
  server.handleUpdate().spinOnce();
  // throwOnMotionError not triggered and go into control feedback loop
  LOG_INFO("    -- Start control callback loop");
  size_t count = 0;
  while (++count < kControlLoopCount) {
    server.handleUpdate().spinOnce();
  }
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
