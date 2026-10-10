#pragma once

#include "FSM/FSMState.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "policy/RobotProfile.h"
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

class State_PolicyBase : public FSMState
{
public:
    using ObservationSources = isaaclab::ManagerBasedRLEnv::ObservationSources;

    State_PolicyBase(int state_mode, const std::string& state_string);
    ~State_PolicyBase() override;

    void enter() override;
    void run() override;
    void exit() override;
    void prepare() override;
    bool can_enter() const override;

protected:
    void initialize(
        ObservationSources observation_sources = {},
        std::function<void(long)> before_step = {},
        std::function<void(long)> after_step = {},
        std::function<void()> on_enter = {});
    void register_finished_transition(std::function<bool()> finished, const std::string& target);

    YAML::Node deploy_cfg_;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_;

private:
    std::unique_ptr<isaaclab::OrtRunner> create_runner() const;
    void configure_motors();
    void policy_loop();

    std::unique_ptr<policy::RobotProfile> robot_profile_;
    policy::JointLayout layout_;
    std::function<void()> on_enter_;
    std::thread policy_thread_;
    std::thread preload_thread_;
    std::atomic<bool> preload_ready_{false};
    std::atomic<bool> policy_running_{false};
    std::atomic<bool> inference_failed_{false};
    std::atomic<int> warmup_ticks_{0};
    std::vector<float> entry_joint_pos_;
    std::chrono::steady_clock::time_point enter_time_;
    float position_blend_duration_ = 0.0f;
    std::string effort_limit_mode_;
    int control_mode_ = 1;
    bool run_first_step_on_enter_ = false;
    bool preload_on_start_ = false;
};
