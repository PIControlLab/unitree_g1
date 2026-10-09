#pragma once

#include "FSM/FSMState.h"
#include <onnxruntime_cxx_api.h>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class State_UFO : public FSMState
{
public:
    State_UFO(int state_mode, std::string state_string);
    ~State_UFO();

    void enter() override;
    void run() override;
    void exit() override;

private:
    static constexpr int kDof = 23;
    static constexpr int kStateDim = 52;
    static constexpr int kPrivilegedDim = 358;
    static constexpr int kReferenceDim = 410;
    static constexpr int kHistoryFrames = 4;
    static constexpr int kActorObsDim = 631;
    static constexpr int kZDim = 256;

    void load_reference(const std::filesystem::path& path);
    void load_models(const YAML::Node& cfg);
    void precompute_latents();
    std::array<float, kStateDim> read_robot_state();
    void policy_loop();

    Ort::Env ort_env_{ORT_LOGGING_LEVEL_WARNING, "ufo"};
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> policy_session_;
    std::unique_ptr<Ort::Session> encoder_session_;
    std::vector<char> policy_model_;

    std::vector<float> reference_;
    std::vector<float> latents_;
    uint32_t reference_frames_ = 0;
    float reference_hz_ = 50.0f;

    std::array<float, kDof> target_q_{};
    std::array<float, kDof> entry_q_{};
    std::mutex target_mutex_;
    std::thread policy_thread_;
    std::atomic<bool> policy_running_{false};
    std::atomic<bool> motion_done_{false};
    std::atomic<bool> inference_failed_{false};
    std::atomic<int> warmup_ticks_{0};
    std::chrono::steady_clock::time_point enter_time_;
    float entry_blend_s_ = 1.0f;

    static const std::array<int, kDof> motor_ids_;
    static const std::array<float, kDof> default_q_;
    static const std::array<float, kDof> kp_;
    static const std::array<float, kDof> kd_;
    static const std::array<float, kDof> effort_limit_;
    static const std::array<float, kDof> action_scale_;
};

REGISTER_FSM(State_UFO)
