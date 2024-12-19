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
      typename MockServer<C>::ConnectCallbackT on_connect = MockServer<C>::ConnectCallbackT(),
      uint32_t sequence_number = 0,
      std::string ip = "127.0.0.1")
      : MockServer<C>(on_connect, sequence_number, std::move(ip)), robot_backend_(robot_backend), state_send_rate_hz_(state_send_rate_hz)  {
        testRobotStateIsZero(franka::convertRobotState(state_));
        startUpdateThread();

      }

  void startUpdateThread(){
    update_thread_ = std::thread(&AescapeMockServer::updateLoop, this);
  }
  /*
   * Handle robot.readOnce() by responding with the robot state
   */
  AescapeMockServer& handleReadOnce() {
    ZoneScoped;

    MockServer<C>::template onSendUDPSeparateThread<r::RobotState>(
        [this](r::RobotState& robot_state) {
          ZoneScopedN("SendState");
          std::lock_guard<std::mutex> lck(mut_);
          // Set server local state_.message_id to the server's sequence number (which is what is being sent to client)
          // the message_id is set from the sequence number in onSendUDP()
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

    handleReadOnce().spinOnce().onReceiveRobotCommandSeparateThread(
        [this](const r::RobotCommand& cmd) {
          ZoneScopedN("OnReceiveRobotCommand");
          // TODO(matthieu): Handle timeout
          //LOG_INFO("    -- Received control robot command");
          std::lock_guard<std::mutex> state_lck(mut_);
          if (cmd.motion.motion_generation_finished) {
            LOG_INFO("    -- Motion generation finished");
            modes_.motion_mode = r::MotionGeneratorMode::kIdle;
            modes_.ctrl_mode = r::ControllerMode::kOther;
            stopped_ = true;
          } else {
            cmd_ = cmd;
            std::cout << "cmd.msg_id " << cmd.message_id << " state.msg_id " << state_.message_id << std::endl;
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
    while (!destructor_called_) {
      // Run without update if we are not in state loop and not stopped
      while (!in_state_loop_ && !destructor_called_) {
        LOG_INFO("    -- updateLoop");
        handleReadOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      LOG_INFO("    -- state loop running destructor_called_: {}, in_state_loop_: {}", destructor_called_, in_state_loop_);

      std::this_thread::sleep_for(std::chrono::microseconds(10));

    }
    LOG_INFO("    -- updateLoop finished");
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

      handleUpdate().spinOnce();

      // Wait until we receive a new command or timeout

      mut.lock();

      {
        ZoneScopedN("Wait for command notify");
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

  ~AescapeMockServer() 
  {
    MockServer<C>::ignoreUdpBuffer();
    destructor_called_ = true;
    update_thread_.join();
    std::cout << "Destructor call finished" << std::endl;
  }

 private:
  std::shared_ptr<aescape::BackendInterface> robot_backend_;
  double state_send_rate_hz_;
  std::mutex mut_;
  std::condition_variable cv_;
  Modes modes_{};
  uint32_t move_id_;
  r::RobotCommand cmd_{};
  r::RobotState state_{};
  bool stopped_{false};
  std::atomic_bool in_state_loop_{false};
  std::atomic_bool destructor_called_{false};
  std::thread update_thread_;
};

