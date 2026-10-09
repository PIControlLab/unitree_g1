#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <csignal>
#include <thread>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/h2/loco/h2_loco_client.hpp>

namespace {

using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using unitree::robot::ChannelFactory;
using unitree::robot::ChannelPublisher;
using unitree::robot::ChannelPublisherPtr;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;

constexpr char kArmSdkTopic[] = "rt/arm_sdk";
constexpr char kLowStateTopic[] = "rt/lowstate";
constexpr int kArmSdkWeightJoint = 31;
constexpr float kControlDt = 0.02f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kSmallShoulderRoll = 20.0f * kPi / 180.0f;
constexpr float kSmallElbow = 30.0f * kPi / 180.0f;
constexpr float kSmallHeadYaw = 20.0f * kPi / 180.0f;
constexpr float kSmallHeadPitch = 15.0f * kPi / 180.0f;

volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) { g_stop_requested = 1; }

enum H2JointIndex : int {
  kLeftShoulderPitch = 15,
  kLeftShoulderRoll = 16,
  kLeftShoulderYaw = 17,
  kLeftElbow = 18,
  kLeftWristRoll = 19,
  kLeftWristPitch = 20,
  kLeftWristYaw = 21,
  kRightShoulderPitch = 22,
  kRightShoulderRoll = 23,
  kRightShoulderYaw = 24,
  kRightElbow = 25,
  kRightWristRoll = 26,
  kRightWristPitch = 27,
  kRightWristYaw = 28,
  kHeadPitch = 29,
  kHeadYaw = 30,
};

constexpr std::array<int, 16> kUpperBodyJoints = {
    kLeftShoulderPitch,  kLeftShoulderRoll,  kLeftShoulderYaw,
    kLeftElbow,          kLeftWristRoll,     kLeftWristPitch,
    kLeftWristYaw,       kRightShoulderPitch, kRightShoulderRoll,
    kRightShoulderYaw,   kRightElbow,        kRightWristRoll,
    kRightWristPitch,    kRightWristYaw,     kHeadPitch,
    kHeadYaw,
};

class StateBuffer {
 public:
  void Set(const LowState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
    has_state_ = true;
  }

  bool Get(LowState& state) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_state_) {
      return false;
    }
    state = state_;
    return true;
  }

 private:
  mutable std::mutex mutex_;
  LowState state_{};
  bool has_state_ = false;
};

void SetJointCommand(LowCmd& command, int joint, float q) {
  auto& motor = command.motor_cmd().at(joint);
  motor.tau(0.0f);
  motor.q(q);
  motor.dq(0.0f);
  if (joint == kHeadPitch || joint == kHeadYaw) {
    motor.kp(30.0f);
    motor.kd(1.0f);
  } else {
    motor.kp(80.0f);
    motor.kd(1.5f);
  }
}

void Publish(LowCmd& command,
             const ChannelPublisherPtr<LowCmd>& publisher) {
  command.crc() = crc32_core(reinterpret_cast<uint32_t*>(&command),
                             (sizeof(LowCmd) >> 2) - 1);
  publisher->Write(command);
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  std::cout << "WARNING: Ensure there are no obstacles around the robot.\n"
            << "Press ENTER to continue..." << std::endl;
  std::cin.get();

  const std::string network_interface = argc > 1 ? argv[1] : "";
  ChannelFactory::Instance()->Init(0, network_interface);

  unitree::robot::h2::LocoClient loco_client;
  loco_client.SetTimeout(5.0f);
  loco_client.Init();

  int fsm_id = -1;
  int ret = loco_client.GetFsmId(fsm_id);
  if (ret != 0) {
    std::cerr << "Failed to get H2 FSM ID, error code: " << ret << std::endl;
    return 1;
  }
  if (fsm_id != 4 && fsm_id != 703) {
    std::cerr << "Current FSM " << fsm_id
              << " does not accept rt/arm_sdk commands. Supported FSMs: 4, 703."
              << std::endl;
    return 1;
  }

  ret = loco_client.EnableArmSDK();
  if (ret != 0) {
    std::cerr << "Failed to enable the external Arm SDK, error code: " << ret
              << std::endl;
    return 1;
  }

  auto disable_arm_sdk = [&]() {
    const int disable_ret = loco_client.DisableArmSDK();
    if (disable_ret != 0) {
      std::cerr << "Failed to disable the external Arm SDK, error code: "
                << disable_ret << std::endl;
    }
  };

  ChannelPublisherPtr<LowCmd> publisher =
      std::make_shared<ChannelPublisher<LowCmd>>(kArmSdkTopic);
  publisher->InitChannel();

  auto state_buffer = std::make_shared<StateBuffer>();
  ChannelSubscriberPtr<LowState> subscriber =
      std::make_shared<ChannelSubscriber<LowState>>(kLowStateTopic);
  subscriber->InitChannel(
      [state_buffer](const void* message) {
        state_buffer->Set(*static_cast<const LowState*>(message));
      },
      10);

  LowState state;
  for (int i = 0; i < 250 && !g_stop_requested && !state_buffer->Get(state);
       ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (g_stop_requested) {
    disable_arm_sdk();
    return 0;
  }
  if (!state_buffer->Get(state)) {
    std::cerr << "Timed out waiting for rt/lowstate." << std::endl;
    disable_arm_sdk();
    return 1;
  }

  const std::array<float, 16> target_pos = {
      0.0f,  kSmallShoulderRoll, 0.0f, kSmallElbow, 0.0f, 0.0f, 0.0f,
      0.0f, -kSmallShoulderRoll,  0.0f, kSmallElbow, 0.0f, 0.0f, 0.0f,
      kSmallHeadPitch, kSmallHeadYaw};
  LowCmd command{};

  const auto sleep_time = std::chrono::milliseconds(20);
  // Match the Python example: every control cycle blends against the latest
  // measured position instead of using a fixed starting pose.
  const auto publish_joint_stage = [&](float duration, int stage) {
    const int steps = static_cast<int>(duration / kControlDt);
    for (int step = 0; step < steps; ++step) {
      if (g_stop_requested) {
        return false;
      }
      const float ratio = static_cast<float>(step + 1) / steps;
      LowState latest_state;
      if (!state_buffer->Get(latest_state)) {
        return false;
      }
      for (size_t i = 0; i < kUpperBodyJoints.size(); ++i) {
        const float measured_q =
            latest_state.motor_state().at(kUpperBodyJoints[i]).q();
        float q = (1.0f - ratio) * measured_q;
        if (stage == 2) {
          q += ratio * target_pos[i];
        }
        SetJointCommand(command, kUpperBodyJoints[i], q);
      }
      command.motor_cmd().at(kArmSdkWeightJoint).q(1.0f);
      Publish(command, publisher);
      std::this_thread::sleep_for(sleep_time);
    }
    return true;
  };

  std::cout << "[Stage 1]: set arms and head to zero posture." << std::endl;
  if (!publish_joint_stage(3.0f, 1)) {
    disable_arm_sdk();
    return 0;
  }

  std::cout << "[Stage 2]: lift arms and move head." << std::endl;
  if (!publish_joint_stage(6.0f, 2)) {
    disable_arm_sdk();
    return 0;
  }

  std::cout << "[Stage 3]: set arms and head back to zero posture." << std::endl;
  if (!publish_joint_stage(9.0f, 3)) {
    disable_arm_sdk();
    return 0;
  }

  std::cout << "[Stage 4]: release arm_sdk." << std::endl;
  const int release_steps = static_cast<int>(3.0f / kControlDt);
  for (int step = 0; step < release_steps; ++step) {
    if (g_stop_requested) {
      break;
    }
    command.motor_cmd().at(kArmSdkWeightJoint).q(
        1.0f - static_cast<float>(step + 1) / release_steps);
    Publish(command, publisher);
    std::this_thread::sleep_for(sleep_time);
  }
  command.motor_cmd().at(kArmSdkWeightJoint).q(0.0f);
  Publish(command, publisher);
  disable_arm_sdk();
  std::cout << "Done!" << std::endl;
  return 0;
}
