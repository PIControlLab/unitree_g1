#pragma once

#include "FSM/FSMState.h"
#include "param.h"
#include "policy/RobotProfile.h"
#include <memory>
#include <string>
#include <vector>

class State_CommandBase : public FSMState
{
public:
    State_CommandBase(int state_mode, const std::string& state_string)
        : FSMState(state_mode, state_string)
    {
        const auto state_cfg = param::config["FSM"][state_string];
        config_ = YAML::LoadFile(
            param::resolve_path(state_cfg["deploy_file"].as<std::string>()).string());
        profile_ = std::make_unique<policy::RobotProfile>(
            param::resolve_path(config_["robot_profile"].as<std::string>()));
        layout_ = profile_->layout(
            config_["joints"]["joint_names"].as<std::vector<std::string>>());
        stiffness_ = config_["control"]["stiffness"].as<std::vector<float>>();
        damping_ = config_["control"]["damping"].as<std::vector<float>>();
        mode_ = config_["control"]["mode"].as<int>();
    }

protected:
    void configure_motors()
    {
        for (const auto& joint : profile_->motor_joints())
        {
            auto& command = lowcmd->msg_.motor_cmd()[joint.motor_id];
            command.mode() = mode_;
            command.q() = lowstate->msg_.motor_state()[joint.motor_id].q();
            command.dq() = 0.0f;
            command.tau() = 0.0f;
            command.kp() = profile_->inactive_stiffness;
            command.kd() = profile_->inactive_damping;
        }
        for (size_t i = 0; i < layout_.names.size(); ++i)
        {
            auto& command = lowcmd->msg_.motor_cmd()[layout_.motor_ids[i]];
            command.kp() = stiffness_[i];
            command.kd() = damping_[i];
        }
    }

    std::vector<float> current_joint_pos() const
    {
        std::vector<float> result(layout_.names.size());
        for (size_t i = 0; i < layout_.names.size(); ++i)
            result[i] = lowstate->msg_.motor_state()[layout_.motor_ids[i]].q();
        return result;
    }

    void write_joint_pos(const std::vector<float>& position)
    {
        for (size_t i = 0; i < layout_.names.size(); ++i)
            lowcmd->msg_.motor_cmd()[layout_.motor_ids[i]].q() = position[i];
    }

    YAML::Node config_;

private:
    std::unique_ptr<policy::RobotProfile> profile_;
    policy::JointLayout layout_;
    std::vector<float> stiffness_;
    std::vector<float> damping_;
    int mode_ = 1;
};
