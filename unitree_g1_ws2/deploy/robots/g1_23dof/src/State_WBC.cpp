#include "FSM/State_WBC.h"
#include "common/mathTools.h"
#include "param.h"
#include <algorithm>
#include <cmath>

State_WBC::State_WBC(int state_mode, const std::string& state_string)
    : State_PolicyBase(state_mode, state_string)
{
    const auto reference_cfg = deploy_cfg_["reference"];
    reference_ = std::make_shared<policy::ReferenceData>(
        param::resolve_path(reference_cfg["manifest"].as<std::string>()));
    start_frame_ = reference_cfg["start_frame"].as<int>();
    end_frame_ = reference_cfg["end_frame"].as<int>();
    frame_interval_ = reference_cfg["frame_interval"].as<int>();
    sample_offset_steps_ = reference_cfg["sample_offset_steps"].as<int>();
    const auto reference_safety = deploy_cfg_["safety"]["reference_gravity_error"];
    gravity_error_threshold_ = reference_safety["threshold"].as<float>();

    ObservationSources sources;
    sources["reference_joint_pos"] = [this](isaaclab::ManagerBasedRLEnv*, YAML::Node) {
        return reference_slice(0, 29);
    };
    sources["reference_joint_vel"] = [this](isaaclab::ManagerBasedRLEnv*, YAML::Node) {
        return reference_slice(29, 58);
    };
    sources["reference_anchor_pos"] = [this](isaaclab::ManagerBasedRLEnv*, YAML::Node) {
        return reference_slice(58, 61);
    };
    sources["reference_anchor_orientation_6d"] =
        [this](isaaclab::ManagerBasedRLEnv*, YAML::Node) {
            return reference_slice(61, 67);
        };

    initialize(
        std::move(sources),
        [this](long step) { update_reference(step); },
        {},
        [this]() { reset_reference(); });

    register_finished_transition(
        [this]() { return current_frame_ >= end_frame_; }, "Velocity");
    registered_checks.insert(registered_checks.begin(), {
        [this]() { return unsafe_reference_.load(); },
        FSMStringMap.right.at(reference_safety["target_state"].as<std::string>())
    });
}

void State_WBC::reset_reference()
{
    unsafe_reference_ = false;
    current_frame_ = start_frame_;
    compute_reference_observation();
}

void State_WBC::update_reference(long step)
{
    current_frame_ = std::min(
        start_frame_ + static_cast<int>(step) + sample_offset_steps_, end_frame_);
    compute_reference_observation();
}

std::vector<float> State_WBC::reference_slice(size_t begin, size_t end) const
{
    return std::vector<float>(
        reference_observation_.begin() + begin,
        reference_observation_.begin() + end);
}

void State_WBC::compute_reference_observation()
{
    const int last_frame = std::max(1, current_frame_ - frame_interval_);
    const auto body_pos = reference_->frame("body_pos_w", current_frame_);
    const auto body_quat = reference_->frame("body_quat_w", current_frame_);
    const auto last_body_pos = reference_->frame("body_pos_w", last_frame);
    const auto last_body_quat = reference_->frame("body_quat_w", last_frame);
    const auto joint_pos = reference_->frame("joint_pos", current_frame_);
    const auto joint_vel = reference_->frame("joint_vel", current_frame_);

    const std::vector<float> anchor_pos(body_pos.begin(), body_pos.begin() + 3);
    const std::vector<float> anchor_quat(body_quat.begin(), body_quat.begin() + 4);
    const std::vector<float> last_anchor_pos(last_body_pos.begin(), last_body_pos.begin() + 3);
    const std::vector<float> last_anchor_quat(last_body_quat.begin(), last_body_quat.begin() + 4);

    const Eigen::Quaternionf robot_quaternion = env_->robot->data.root_quat_w;
    const std::vector<float> robot_quat = {
        robot_quaternion.w(), robot_quaternion.x(), robot_quaternion.y(), robot_quaternion.z()};
    const auto yaw_delta = quat_multiply(yaw_quat(robot_quat), quat_conjugate(yaw_quat(anchor_quat)));
    const auto aligned_anchor_quat = quat_multiply(yaw_delta, anchor_quat);
    const auto motion_gravity = quat_apply_inverse(aligned_anchor_quat, {0.0f, 0.0f, -1.0f});

    const auto [target_pos, target_quat] = subtract_frame_transforms(
        last_anchor_pos, last_anchor_quat, &anchor_pos, &anchor_quat);
    const Eigen::Matrix3f rotation = matrix_from_quat(target_quat);

    reference_observation_.clear();
    reference_observation_.reserve(67);
    reference_observation_.insert(reference_observation_.end(), joint_pos.begin(), joint_pos.end());
    reference_observation_.insert(reference_observation_.end(), joint_vel.begin(), joint_vel.end());
    reference_observation_.insert(reference_observation_.end(), target_pos.begin(), target_pos.end());
    reference_observation_.insert(reference_observation_.end(), {
        rotation(0, 0), rotation(0, 1),
        rotation(1, 0), rotation(1, 1),
        rotation(2, 0), rotation(2, 1)});

    unsafe_reference_ = std::abs(
        motion_gravity[2] - env_->robot->data.projected_gravity_b[2])
        > gravity_error_threshold_;
}
