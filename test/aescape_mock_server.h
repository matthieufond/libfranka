#include "mock_server.h"
#include "helpers.h"
#include <robot_impl.h>
#include "logging_macros.h"

#include <backend/forward_dynamics_backend.h>
#include <magic_enum.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

/*
 * Mock Server Example
 * This example demonstrates how to use the MockServer class and respond to different commands
 * from the franka::robot class.
 */

using namespace std::string_literals;
using namespace std::placeholders;
namespace r = research_interface::robot;
using move_ctrl_mode_t = r::Move::ControllerMode;
using move_motion_mode_t = r::Move::MotionGeneratorMode;

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

struct Modes {
  r::ControllerMode ctrl_mode{r::ControllerMode::kOther};
  r::MotionGeneratorMode motion_mode{r::MotionGeneratorMode::kIdle};
  r::RobotMode robot_mode{r::RobotMode::kIdle};
};

/*
 * Create a success move response
 */
r::Move::Response moveSuccessResp() {
  return r::Move::Response(r::Move::Status::kSuccess);
}

constexpr int kNumJoints = 7;

/*
 * Aescape MockServer example
 */
template <typename C>
class AescapeMockServer : public MockServer<C> {
 public:
  AescapeMockServer(
      std::shared_ptr<aescape::BackendInterface> robot_backend,
      double state_send_rate_hz,
      const std::string& urdfPath,
      uint32_t sequence_number = 0,
      std::string ip = "127.0.0.1")
      : MockServer<C>(std::bind(&AescapeMockServer::onConnect, this, _1),
        sequence_number, std::move(ip)),
        robot_backend_(robot_backend),
        state_send_rate_hz_(state_send_rate_hz),
        kUrdf_str_(loadUrdf(urdfPath)),
        model_buffer_(loadModelStream())
      {
        // Grab initial state from backend
        robot_backend_->updateRobotState(state_);

      }

  void startUpdateThread(){
    update_thread_ = std::thread(&AescapeMockServer::updateLoop, this);
  }
  /*
   * Handle robot.readOnce() by responding with the robot state
   */
  AescapeMockServer& handleReadOnce() {
    ZoneScoped;

    MockServer<C>::template onSendUDPSeparateQueue<r::RobotState>(
        [this](r::RobotState& robot_state) {
          ZoneScopedN("SendState");
          std::lock_guard<std::mutex> lck(mut_);
          // Set server local state_.message_id to the server's sequence number (which is what is being sent to client)
          // the message_id is set from the sequence number in onSendUDP()
          TRACY_MESSAGE(msg, "sequence_number:{},robot_state.message_id:{}", MockServer<C>::sequenceNumber(), robot_state.message_id)

          state_.message_id = robot_state.message_id;
          robot_state = state_;
          // Mode
          robot_state.robot_mode = modes_.robot_mode;
          robot_state.controller_mode = modes_.ctrl_mode;
          robot_state.motion_generator_mode = modes_.motion_mode;
        });
    return *this;
  }

  /*
   * Handle robot.update() by sending the robot state and receiving the robot command
   */
  AescapeMockServer& handleUpdate() {
    ZoneScoped;

    handleReadOnce().onReceiveRobotCommandSeparateQueue(
        [this](const r::RobotCommand& cmd) {
          ZoneScopedN("OnReceiveRobotCommand");
          //LOG_INFO("    -- Received control robot command");
          std::lock_guard<std::mutex> state_lck(mut_);
          if (cmd.message_id == 0) {
            // TODO(matthieu): Handle timeout
            LOG_INFO("Command timed out, ignoring");
            return;
          }
          if (cmd.motion.motion_generation_finished) {
            LOG_INFO("    -- Motion generation finished");
            modes_.motion_mode = r::MotionGeneratorMode::kIdle;
            modes_.ctrl_mode = r::ControllerMode::kOther;
            stopped_ = true;
          } else {
            cmd_ = cmd;
          }
          cv_.notify_all();
        });
    return *this;
  }

  /*
   * Wait for a move command and responding with a success
   */
  AescapeMockServer& waitForMove() {
    ZoneScoped;

    MockServer<C>::template waitForCommand<r::Move>(
        [&](const r::Move::Request& req) {
          ZoneScopedN("WaitForMove");
          std::lock_guard<std::mutex> state_lck(mut_);
          modes_.motion_mode = convertMode(req.motion_generator_mode);
          modes_.ctrl_mode = convertMode(req.controller_mode);
          modes_.robot_mode = r::RobotMode::kMove;
          LOG_INFO("    -- Move Received: Controller Mode: {}, Motion Mode: {}",
                   magic_enum::enum_name(modes_.ctrl_mode),
                   magic_enum::enum_name(modes_.motion_mode));

          // Stop Update Thread
          stopUpdateLoop();

          return moveSuccessResp();
        },
        &move_id_);

    return *this;
  }

  /*
   * Respond to the move command with a success without waiting for a request
   */
  AescapeMockServer& respondMove() {
    ZoneScoped;
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

  void updateLoop() {
    ZoneScoped;
    // Run without update if we are not in state loop and not stopped
    while (!in_state_loop_) {
      handleReadOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void stateLoop() {
    in_state_loop_ = true;
    ZoneScoped;
    using clock = std::chrono::steady_clock;
    uint64_t cycles = 0;
    auto init_time = clock::now();
    uint64_t dt_us = 1e6 / state_send_rate_hz_;
    //state_.message_id = MockServer<C>::sequenceNumber() + 1;

    std::unique_lock<std::mutex> mut(mut_);


    while (!stopped_) {
      ZoneScopedN("SendStateLoop");

      mut.unlock();

      auto deadline = std::chrono::microseconds(++cycles*dt_us) + init_time;

      handleUpdate();

      // Wait until we receive a new command or timeout

      mut.lock();

      {
        ZoneScopedN("Wait for command notify");
        TRACY_MESSAGE(msg, "seq_num:{},cmd.message_id:{}", MockServer<C>::sequenceNumber(), cmd_.message_id);

        cv_.wait_until(mut, deadline, [this]{return cmd_.message_id == MockServer<C>::sequenceNumber();});
      }

      // Figure out how to cleanly handle late arriving commands
      r::RobotCommand cmd = cmd_;
      r::RobotState state = state_;
      
      mut.unlock();

      if (cmd.message_id < state.message_id) {
        // Do something if we didn't get a command a time. Just update state still ?
      } else {
        // Update State
        robot_backend_->updateRobotCommand(cmd);
        robot_backend_->updateRobotState(state);
      }

      // Sleep until appropriate to send the computed state

      auto prev_time = clock::now();
      auto tts = std::chrono::duration_cast<std::chrono::microseconds>(deadline - prev_time);

      std::this_thread::sleep_for(tts);

      if (std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - prev_time).count() > std::abs(1.2*tts.count())) {
        std::cout << "Missed deadline by more than 20%, expected " << tts.count() << "us slept " << std::chrono::duration_cast<std::chrono::microseconds>(clock::now()-prev_time).count() << "us" << std::endl;
      }

      mut.lock();
      state_ = state;
    }
    LOG_INFO("getting out of state loop");
    in_state_loop_ = false;

  }

  void stopUpdateLoop() {
    in_state_loop_ = true;
    //update_thread_.join();
  }

  void run() {
    std::unique_lock<std::mutex> lck(mut_);

    while (!connected_) {
      LOG_INFO("Waiting for connection...");
      cv_.wait(lck);
    }
    lck.unlock();

    // ReadOnce() needs states to be sent, but trying to sync on timing with the client is tricky
    // Easier to have a thread send states at init and then kill it once we get into Move mode
    startUpdateThread();

    // Handle init methods from the controls stack
    MockServer<C>::template waitForCommand<r::GetRobotModel>([this](const typename r::GetRobotModel::Request& /*request*/) {
          ZoneScopedN("LoadModelCommand");
          LOG_INFO("--- Receive load model request -> loadModel");
          LOG_INFO(" ++ GetRobotModel");
          return r::GetRobotModel::Response(r::GetRobotModel::Status::kSuccess, kUrdf_str_);
        })
        .spinOnce()
        .generic([this](typename MockServer<C>::Socket& tcp_socket, typename MockServer<C>::Socket&) {
          ZoneScopedN("LoadModelResponse");
          LOG_INFO(" ++ LoadModelLibrary");
          r::CommandHeader header;
          this->template receiveRequest<r::LoadModelLibrary>(tcp_socket, &header);
          this->template sendResponse<r::LoadModelLibrary>(
              tcp_socket,
              r::CommandHeader(
                  r::Command::kLoadModelLibrary, header.command_id,
                  sizeof(r::CommandMessage<r::LoadModelLibrary::Response>) + model_buffer_.size()),
              r::LoadModelLibrary::Response(r::LoadModelLibrary::Status::kSuccess));
          tcp_socket.sendBytes(model_buffer_.data(), model_buffer_.size());
        })
        .spinOnce()
        .template waitForCommand<r::AutomaticErrorRecovery>(
            [](const typename r::AutomaticErrorRecovery::Request&) {
              ZoneScopedN("ErrorRecoveryCmd");
              LOG_INFO("--- Automatic Error Recovery");
              return r::AutomaticErrorRecovery::Response(r::AutomaticErrorRecovery::Status::kSuccess);
            })
        .spinOnce()
        .template waitForCommand<r::SetCollisionBehavior>(
            [](const typename r::SetCollisionBehavior::Request&) {
              ZoneScopedN("CollisonBehaviorCmd");
              LOG_INFO("--- SetCollisionBehavior");
              return r::SetCollisionBehavior::Response(r::SetCollisionBehavior::Status::kSuccess);
            })
        .spinOnce()
        .template waitForCommand<r::SetJointImpedance>([](const typename r::SetJointImpedance::Request&) {
          ZoneScopedN("JointImpedanceCmd");
          LOG_INFO("--- SetJointImpedance");
          return r::SetJointImpedance::Response(r::SetJointImpedance::Status::kSuccess);
        })
        .spinOnce()
        .template waitForCommand<r::SetCartesianImpedance>(
            [](const typename r::SetCartesianImpedance::Request&) {
              ZoneScopedN("CartesianImpedanceCmd");
              LOG_INFO("--- SetCartesianImpedance");
              return r::SetCartesianImpedance::Response(r::SetCartesianImpedance::Status::kSuccess);
            })
        .spinOnce();

    LOG_INFO("--- Read current state ->readOnce (Handle by update thread)");  // Can miss. Try twice here.

    LOG_INFO("--- Run control loop ->control");
    LOG_INFO(" ++ ControlLoop Constructor");
    LOG_INFO("    -- Handle robot.startMotion()");
    // Send Move response again for startMotion haven't done a RobotState update yet
    // and it checks for a Move response. It does not send another request so simply send a response
    waitForMove().spinOnce().respondMove().spinOnce();
    LOG_INFO("    -- Handle robot.update() (Handle by update thread)");
    LOG_INFO(" ++ ControlLoop Constructor DONE");

    LOG_INFO(" ++ operator()()");

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    
    stateLoop();

    LOG_INFO("    -- Finish control callback loop");

    LOG_INFO("    -- Start robot.finishMotion");
    handleReadOnce()
        .respondMove()  // No request. Only waiting for response.
        .spinOnce();
    LOG_INFO("    -- Finish robot.finishMotion");

    LOG_INFO("--- Stop");
    MockServer<C>::template waitForCommand<r::StopMove>([](const r::StopMove::Request&) {
      LOG_INFO(" ++ Received StopMove");
      return r::StopMove::Response(r::StopMove::Status::kSuccess);
    }).spinOnce();
    LOG_INFO("DONE");
      
  }

  ~AescapeMockServer() 
  {
    MockServer<C>::ignoreUdpBuffer();
    destructor_called_ = true;
    update_thread_.join();
    std::cout << "Destructor call finished" << std::endl;
  }

 private:
  RobotTypes::Connect::Response onConnect(const RobotTypes::Connect::Request& req) {
    ZoneScoped;
    LOG_INFO("Connected! port: {}", req.udp_port);
    std::unique_lock<std::mutex> lck(mut_);
    connected_ = true;
    cv_.notify_one();
    return RobotTypes::Connect::Response(RobotTypes::Connect::Status::kSuccess);
  }

  std::vector<char> loadModelStream() {
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

    return buffer;
  }

  std::string loadUrdf(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      std::cerr << "Error opening file: " << path << std::endl;
      return "";
    }

    std::ostringstream oss;
    oss << file.rdbuf();
    file.close();

    return oss.str();
  }
  
  std::shared_ptr<aescape::BackendInterface> robot_backend_;
  double state_send_rate_hz_;
  std::string kUrdf_str_;
  std::vector<char> model_buffer_;
  std::mutex mut_;
  std::condition_variable cv_;
  Modes modes_{};
  uint32_t move_id_;
  r::RobotCommand cmd_{};
  r::RobotState state_{};
  bool stopped_{false};
  bool connected_{false};
  std::atomic_bool in_state_loop_{false};
  std::atomic_bool destructor_called_{false};
  std::thread update_thread_;
};
