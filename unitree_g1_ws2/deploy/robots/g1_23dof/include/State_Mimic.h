#pragma once

#include "FSM/State_PolicyBase.h"
#include "policy/MotionReference.h"
#include <Eigen/Geometry>
#include <memory>

class State_Mimic : public State_PolicyBase
{
public:
    State_Mimic(int state_mode, const std::string& state_string);

private:
    void reset_reference();
    void update_reference(long step);
    std::vector<float> reference_joint_state() const;
    std::vector<float> reference_anchor_orientation(isaaclab::ManagerBasedRLEnv* env) const;
    Eigen::Quaternionf torso_quaternion(isaaclab::ManagerBasedRLEnv* env) const;
    Eigen::Quaternionf reference_anchor_quaternion() const;

    std::shared_ptr<policy::MotionReference> reference_;
    Eigen::Quaternionf yaw_alignment_ = Eigen::Quaternionf::Identity();
    float start_time_ = 0.0f;
    float end_time_ = 0.0f;
    int waist_index_ = 0;
};

REGISTER_FSM(State_Mimic)
