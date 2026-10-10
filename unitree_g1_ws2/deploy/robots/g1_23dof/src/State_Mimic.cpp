#include "State_Mimic.h"
#include "isaaclab/utils/utils.h"
#include "param.h"
#include <algorithm>

State_Mimic::State_Mimic(int state_mode, const std::string& state_string)
    : State_PolicyBase(state_mode, state_string)
{
    const auto reference_cfg = deploy_cfg_["reference"];
    reference_ = std::make_shared<policy::MotionReference>(
        param::resolve_path(reference_cfg["manifest"].as<std::string>()));
    start_time_ = reference_cfg["start_time"].as<float>();
    end_time_ = std::min(reference_cfg["end_time"].as<float>(), reference_->duration);
    const auto& names = reference_->joint_names();
    waist_index_ = static_cast<int>(
        std::find(names.begin(), names.end(), "waist_yaw_joint") - names.begin());

    ObservationSources sources;
    sources["reference_joint_state"] =
        [this](isaaclab::ManagerBasedRLEnv*, YAML::Node) { return reference_joint_state(); };
    sources["reference_anchor_orientation_6d"] =
        [this](isaaclab::ManagerBasedRLEnv* env, YAML::Node) {
            return reference_anchor_orientation(env);
        };

    initialize(
        std::move(sources),
        [this](long step) { update_reference(step); },
        {},
        [this]() { reset_reference(); });

    register_finished_transition(
        [this]() { return env_->episode_length * env_->step_dt > end_time_ - start_time_; },
        "Velocity");
}

void State_Mimic::reset_reference()
{
    reference_->update(start_time_);
    env_->robot->update();
    const auto reference_yaw =
        isaaclab::yawQuaternion(reference_anchor_quaternion()).toRotationMatrix();
    const auto robot_yaw =
        isaaclab::yawQuaternion(torso_quaternion(env_.get())).toRotationMatrix();
    yaw_alignment_ = Eigen::Quaternionf(robot_yaw * reference_yaw.transpose());
}

void State_Mimic::update_reference(long step)
{
    reference_->update(start_time_ + step * env_->step_dt);
}

std::vector<float> State_Mimic::reference_joint_state() const
{
    const auto position = reference_->joint_pos();
    const auto velocity = reference_->joint_vel();
    std::vector<float> result;
    result.reserve(position.size() + velocity.size());
    result.insert(result.end(), position.data(), position.data() + position.size());
    result.insert(result.end(), velocity.data(), velocity.data() + velocity.size());
    return result;
}

Eigen::Quaternionf State_Mimic::torso_quaternion(isaaclab::ManagerBasedRLEnv* env) const
{
    return env->robot->data.root_quat_w
        * Eigen::AngleAxisf(env->robot->data.joint_pos[waist_index_], Eigen::Vector3f::UnitZ());
}

Eigen::Quaternionf State_Mimic::reference_anchor_quaternion() const
{
    return reference_->root_quaternion()
        * Eigen::AngleAxisf(reference_->joint_pos()[waist_index_], Eigen::Vector3f::UnitZ());
}

std::vector<float> State_Mimic::reference_anchor_orientation(
    isaaclab::ManagerBasedRLEnv* env) const
{
    const Eigen::Quaternionf relative =
        (yaw_alignment_ * reference_anchor_quaternion()).conjugate()
        * torso_quaternion(env);
    const Eigen::Matrix3f rotation = relative.toRotationMatrix().transpose();
    return {
        rotation(0, 0), rotation(0, 1),
        rotation(1, 0), rotation(1, 1),
        rotation(2, 0), rotation(2, 1),
    };
}
