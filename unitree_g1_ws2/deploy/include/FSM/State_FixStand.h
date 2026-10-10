#pragma once

#include "FSM/State_CommandBase.h"
#include <algorithm>

class State_FixStand : public State_CommandBase
{
public:
    State_FixStand(int state_mode, const std::string& state_string = "FixStand")
        : State_CommandBase(state_mode, state_string)
    {
        target_ = config_["joints"]["default_joint_pos"].as<std::vector<float>>();
        duration_ = config_["trajectory"]["duration"].as<float>();
    }

    void enter() override
    {
        configure_motors();
        start_ = current_joint_pos();
        start_time_ = unitree::common::GetCurrentTimeMillisecond() * 1.0e-3;
    }

    void run() override
    {
        const float elapsed =
            unitree::common::GetCurrentTimeMillisecond() * 1.0e-3 - start_time_;
        const float blend = std::clamp(elapsed / duration_, 0.0f, 1.0f);
        std::vector<float> position(target_.size());
        for (size_t i = 0; i < position.size(); ++i)
            position[i] = start_[i] + blend * (target_[i] - start_[i]);
        write_joint_pos(position);
    }

private:
    std::vector<float> start_;
    std::vector<float> target_;
    double start_time_ = 0.0;
    float duration_ = 0.0f;
};

REGISTER_FSM(State_FixStand)
