// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "LinearInterpolator.h"
#include <chrono>

class State_FixStand : public FSMState
{
public:
    State_FixStand(int state, std::string state_string = "FixStand")
    : FSMState(state, state_string)
    {
        ts_ = param::config["FSM"]["FixStand"]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"]["FixStand"]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());
    }

    void enter()
    {
        // set gain
        static auto kp = param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>();
        for(int i(0); i < kp.size(); ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = kp[i];
            motor.kd() = kd[i];
            motor.dq() = motor.tau() = 0;
        }


        // set initial position
        std::vector<float> q0;
        for(int i(0); i < kp.size(); ++i) {
            q0.push_back(lowcmd->msg_.motor_cmd()[i].q());
        }
        qs_[0] = q0;
        t0_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
        spdlog::info("FixStand enter: kp_count={} first_target_q={:.4f} current_q0={:.4f}", kp.size(), qs_[1].empty() ? 0.0f : qs_[1][0], q0.empty() ? 0.0f : q0[0]);
    }

    void run()
    {
        float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_;
        auto q = linear_interpolate(t, ts_, qs_);

        for(int i(0); i < q.size(); ++i) {
            lowcmd->msg_.motor_cmd()[i].q() = q[i];
        }

        const auto now = std::chrono::steady_clock::now();
        if(!last_log_time_.has_value() || std::chrono::duration<double>(now - *last_log_time_).count() > 0.5) {
            // removed
            last_log_time_ = now;
        }
    }

private:
    double t0_;
    std::optional<std::chrono::steady_clock::time_point> last_log_time_;
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
};

REGISTER_FSM(State_FixStand)