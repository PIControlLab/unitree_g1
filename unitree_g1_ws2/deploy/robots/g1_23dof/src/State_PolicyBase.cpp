#include "FSM/State_PolicyBase.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "param.h"
#include "unitree_articulation.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

State_PolicyBase::State_PolicyBase(int state_mode, const std::string& state_string)
    : FSMState(state_mode, state_string)
{
    const auto state_cfg = param::config["FSM"][state_string];
    deploy_cfg_ = YAML::LoadFile(
        param::resolve_path(state_cfg["deploy_file"].as<std::string>()).string());
    robot_profile_ = std::make_unique<policy::RobotProfile>(
        param::resolve_path(deploy_cfg_["robot_profile"].as<std::string>()));
    layout_ = robot_profile_->layout(
        deploy_cfg_["joints"]["joint_names"].as<std::vector<std::string>>());

    position_blend_duration_ =
        deploy_cfg_["handover"]["position_blend_duration"].as<float>();
    effort_limit_mode_ = deploy_cfg_["actions"]["effort_limit_mode"].as<std::string>();
    control_mode_ = deploy_cfg_["control"]["mode"].as<int>();
    run_first_step_on_enter_ = deploy_cfg_["timing"]["run_first_step_on_enter"].as<bool>();
    preload_on_start_ = deploy_cfg_["model"]["preload_on_start"]
        && deploy_cfg_["model"]["preload_on_start"].as<bool>();
}

State_PolicyBase::~State_PolicyBase()
{
    exit();
    if (preload_thread_.joinable()) preload_thread_.join();
}

void State_PolicyBase::prepare()
{
    if (!preload_on_start_ || preload_thread_.joinable() || preload_ready_) return;
    preload_thread_ = std::thread([this]() {
        try
        {
            env_->alg = create_runner();
            preload_ready_ = true;
            spdlog::info("[{}] policy preloaded", getStateString());
        }
        catch (const std::exception& error)
        {
            spdlog::critical("[{}] policy preload failed: {}", getStateString(), error.what());
        }
    });
}

bool State_PolicyBase::can_enter() const
{
    return !preload_on_start_ || preload_ready_.load();
}

void State_PolicyBase::initialize(
    ObservationSources observation_sources,
    std::function<void(long)> before_step,
    std::function<void(long)> after_step,
    std::function<void()> on_enter)
{
    auto articulation =
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);
    env_ = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        deploy_cfg_, articulation, layout_, std::move(observation_sources));
    env_->before_step = std::move(before_step);
    env_->after_step = std::move(after_step);
    on_enter_ = std::move(on_enter);

    registered_checks.insert(registered_checks.begin(), {
        [this]() { return inference_failed_.load(); },
        FSMStringMap.right.at("Passive")
    });

    const auto safety = deploy_cfg_["safety"]["bad_orientation"];
    if (safety["enabled"].as<bool>())
    {
        const float threshold = safety["threshold_rad"].as<float>();
        const int warmup = safety["warmup_steps"].as<int>();
        const int target = FSMStringMap.right.at(safety["target_state"].as<std::string>());
        registered_checks.emplace_back(
            [this, threshold, warmup]() {
                if (warmup_ticks_.fetch_add(1) < warmup) return false;
                std::lock_guard<std::mutex> lock(lowstate->mutex_);
                const auto& quat = lowstate->msg_.imu_state().quaternion();
                const Eigen::Quaternionf q(quat[0], quat[1], quat[2], quat[3]);
                const float gravity_z =
                    (q.conjugate() * Eigen::Vector3f(0.0f, 0.0f, -1.0f))[2];
                return std::acos(std::clamp(-gravity_z, -1.0f, 1.0f)) > threshold;
            },
            target);
    }
}

void State_PolicyBase::register_finished_transition(
    std::function<bool()> finished, const std::string& target)
{
    registered_checks.insert(
        registered_checks.begin(), {std::move(finished), FSMStringMap.right.at(target)});
}

std::unique_ptr<isaaclab::OrtRunner> State_PolicyBase::create_runner() const
{
    std::vector<std::filesystem::path> files;
    for (const auto& file : deploy_cfg_["model"]["policy"]["files"])
        files.push_back(param::resolve_path(file.as<std::string>()));
    return std::make_unique<isaaclab::OrtRunner>(files);
}

void State_PolicyBase::configure_motors()
{
    entry_joint_pos_.resize(layout_.names.size());
    for (const auto& joint : robot_profile_->motor_joints())
    {
        auto& command = command_msg.motor_cmd()[joint.motor_id];
        command.mode() = control_mode_;
        command.q() = lowstate->msg_.motor_state()[joint.motor_id].q();
        command.dq() = 0.0f;
        command.tau() = 0.0f;
        command.kp() = robot_profile_->inactive_stiffness;
        command.kd() = robot_profile_->inactive_damping;
    }

    const auto& stiffness = env_->robot->data.joint_stiffness;
    const auto& damping = env_->robot->data.joint_damping;
    for (size_t i = 0; i < layout_.names.size(); ++i)
    {
        const int motor_id = layout_.motor_ids[i];
        entry_joint_pos_[i] = lowstate->msg_.motor_state()[motor_id].q();
        if (!layout_.active[i]) continue;
        auto& command = command_msg.motor_cmd()[motor_id];
        command.kp() = stiffness[i];
        command.kd() = damping[i];
    }
}

void State_PolicyBase::enter()
{
    exit();
    inference_failed_ = false;
    warmup_ticks_ = 0;
    if (!preload_on_start_) env_->alg = create_runner();
    configure_motors();
    if (on_enter_) on_enter_();
    env_->reset();
    if (run_first_step_on_enter_) env_->step();
    enter_time_ = std::chrono::steady_clock::now();

    policy_running_ = true;
    policy_thread_ = std::thread(&State_PolicyBase::policy_loop, this);
    spdlog::info("[{}] policy started at {:.1f} Hz", getStateString(), 1.0f / env_->step_dt);
}

void State_PolicyBase::policy_loop()
{
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(env_->step_dt));
    auto next = Clock::now() + period;
    try
    {
        while (policy_running_)
        {
            env_->step();
            std::this_thread::sleep_until(next);
            next += period;
        }
    }
    catch (const std::exception& error)
    {
        spdlog::critical("[{}] inference failed: {}", getStateString(), error.what());
        inference_failed_ = true;
        policy_running_ = false;
    }
}

void State_PolicyBase::run()
{
    const auto target = env_->action_manager->processed_actions();
    const float blend = position_blend_duration_ == 0.0f
        ? 1.0f
        : std::clamp(
            std::chrono::duration<float>(std::chrono::steady_clock::now() - enter_time_).count()
                / position_blend_duration_,
            0.0f, 1.0f);

    const auto& stiffness = env_->robot->data.joint_stiffness;
    const auto& damping = env_->robot->data.joint_damping;
    const auto& effort_limit = env_->robot->data.joint_effort_limit;
    for (size_t i = 0; i < layout_.names.size(); ++i)
    {
        if (!layout_.active[i]) continue;
        const int motor_id = layout_.motor_ids[i];
        const float desired = entry_joint_pos_[i] + blend * (target[i] - entry_joint_pos_[i]);
        auto& command = command_msg.motor_cmd()[motor_id];
        if (effort_limit_mode_ == "equivalent_position")
        {
            const float q = lowstate->msg_.motor_state()[motor_id].q();
            const float dq = lowstate->msg_.motor_state()[motor_id].dq();
            const float torque = std::clamp(
                stiffness[i] * (desired - q) - damping[i] * dq,
                -effort_limit[i], effort_limit[i]);
            command.q() = q + (torque + damping[i] * dq) / stiffness[i];
        }
        else
        {
            command.q() = desired;
        }
        command.dq() = 0.0f;
        command.tau() = 0.0f;
        command.kp() = stiffness[i];
        command.kd() = damping[i];
    }

    for (const auto& joint : robot_profile_->motor_joints())
    {
        if (joint.active) continue;
        auto& command = command_msg.motor_cmd()[joint.motor_id];
        command.q() = lowstate->msg_.motor_state()[joint.motor_id].q();
        command.kp() = robot_profile_->inactive_stiffness;
        command.kd() = robot_profile_->inactive_damping;
    }
}

void State_PolicyBase::exit()
{
    policy_running_ = false;
    if (policy_thread_.joinable()) policy_thread_.join();
}
