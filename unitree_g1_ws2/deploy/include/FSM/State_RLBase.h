// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include "isaaclab/algorithms/algorithms.h"

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);

    static constexpr int TOTAL_MOTORS = 29;
    static constexpr int LEG_END = 11;
    static constexpr float DEFAULT_KP = 40.0f;
    static constexpr float DEFAULT_KD = 1.0f;
    static constexpr float SOFT_LEG = 1.0f;   
    static constexpr float SOFT_UPPER = 1.0f; 
    static constexpr float RAMP_DURATION = 3.0f;

    void enter()
    {
        _warmup = 0;

        // 重建 ONNX session，清零 GRU/LSTM 隐状态
        env->alg = std::make_unique<isaaclab::OrtRunner>(_policy_onnx_path);
        spdlog::info("[Velocity] ONNX session recreated, GRU state reset");

        // 仅在 WBC→Velocity 时启用软接手：腿 30% / 上身 10%，1s ramp 到 100%
        // 参考 FixStand→Velocity 的思路：FixStand 用满刚度 + 当前位置起步的平滑插值，
        // 过渡丝滑。WBC 末态离站立位有偏差，不能直接满刚度，但腿需要一定刚度维持支撑。
        bool do_soft_ramp = test::velocity_soft_arms;
        test::velocity_soft_arms = false;
        if (do_soft_ramp) {
            _soft_arm_ramp = true;
            _arm_ramp_start = std::chrono::steady_clock::now();
            spdlog::info("[Velocity] Soft takeover: legs {:.0f}% / upper {:.0f}%, ramp {:.0f}s",
                         SOFT_LEG * 100, SOFT_UPPER * 100, RAMP_DURATION);
        }

        // set gain — 遍历全部 29 个电机
        // mode = 0x01 (MIT 模式): τ = kp*(q_des - q) + kd*(dq_des - dq)
        // 与 g1_merger 的锁定逻辑保持一致
        for (int i = 0; i < TOTAL_MOTORS; ++i)
        {
            float kp = (i < static_cast<int>(env->robot->data.joint_stiffness.size()))
                       ? env->robot->data.joint_stiffness[i]
                       : DEFAULT_KP;
            float kd = (i < static_cast<int>(env->robot->data.joint_damping.size()))
                       ? env->robot->data.joint_damping[i]
                       : DEFAULT_KD;
            if (do_soft_ramp) {
                float factor = (i <= LEG_END) ? SOFT_LEG : SOFT_UPPER;
                kp *= factor;
                kd *= factor;
            }
            lowcmd->msg_.motor_cmd()[i].mode() = 0x01;
            lowcmd->msg_.motor_cmd()[i].kp() = kp;
            lowcmd->msg_.motor_cmd()[i].kd() = kd;
            lowcmd->msg_.motor_cmd()[i].dq() = 0;
            lowcmd->msg_.motor_cmd()[i].tau() = 0;
            lowcmd->msg_.motor_cmd()[i].q() =
                lowstate->msg_.motor_state()[i].q();
        }

        env->robot->update();

        // 先同步执行一步 env->step()，确保 processed_actions 有
        // 带 offset 的有效值，避免 run() 首次调用读到全零 actions
        // 导致关节目标位置被错误覆盖为 0
        env->reset();
        env->step();
        spdlog::info("[Velocity] First policy step done, actions ready");

        // Start policy thread
        policy_thread_running = true;
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);
            auto sleepTill = clock::now() + dt;
            while (policy_thread_running)
            {
                env->step();
                std::this_thread::sleep_until(sleepTill);
                sleepTill += dt;
            }
        });
    }

    void run();
    int _warmup = 0;

    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
    }

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::string _policy_onnx_path;

    std::thread policy_thread;
    bool policy_thread_running = false;

    // 手臂刚度 ramp-up（仅 WBC→Velocity 时启用）
    bool _soft_arm_ramp = false;
    std::chrono::steady_clock::time_point _arm_ramp_start;
};

REGISTER_FSM(State_RLBase)
