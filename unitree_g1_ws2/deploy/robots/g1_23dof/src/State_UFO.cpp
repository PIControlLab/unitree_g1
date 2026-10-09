#include "State_UFO.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>

const std::array<int, State_UFO::kDof> State_UFO::motor_ids_ = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    15, 16, 17, 18, 19, 22, 23, 24, 25, 26
};
const std::array<float, State_UFO::kDof> State_UFO::default_q_ = {
    -0.1f, 0, 0, 0.3f, -0.2f, 0, -0.1f, 0, 0, 0.3f, -0.2f, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
const std::array<float, State_UFO::kDof> State_UFO::kp_ = {
    99.09843f, 99.09843f, 40.17924f, 99.09843f, 28.50125f, 28.50125f,
    99.09843f, 99.09843f, 40.17924f, 99.09843f, 28.50125f, 28.50125f,
    300.0f, 14.25062f, 14.25062f, 14.25062f, 14.25062f, 14.25062f,
    14.25062f, 14.25062f, 14.25062f, 14.25062f, 14.25062f
};
const std::array<float, State_UFO::kDof> State_UFO::kd_ = {
    6.3088f, 6.3088f, 2.55789f, 6.3088f, 1.81445f, 1.81445f,
    6.3088f, 6.3088f, 2.55789f, 6.3088f, 1.81445f, 1.81445f, 5.0f,
    0.90722f, 0.90722f, 0.90722f, 0.90722f, 0.90722f, 0.90722f,
    0.90722f, 0.90722f, 0.90722f, 0.90722f
};
const std::array<float, State_UFO::kDof> State_UFO::effort_limit_ = {
    139, 139, 88, 139, 50, 50, 139, 139, 88, 139, 50, 50, 88,
    25, 25, 25, 25, 25, 25, 25, 25, 25, 25
};
const std::array<float, State_UFO::kDof> State_UFO::action_scale_ = {
    .350661459f, .350661459f, .547546444f, .350661459f, .438577255f, .438577255f,
    .350661459f, .350661459f, .547546444f, .350661459f, .438577255f, .438577255f,
    .0733333333f, .438577409f, .438577409f, .438577409f, .438577409f, .438577409f,
    .438577409f, .438577409f, .438577409f, .438577409f, .438577409f
};

namespace {
template <typename T>
void read_exact(std::ifstream& stream, T& value) {
    stream.read(reinterpret_cast<char*>(&value), sizeof(value));
    if (!stream) throw std::runtime_error("UFO: truncated reference header");
}

std::filesystem::path resolve_path(const std::string& value) {
    std::filesystem::path path(value);
    return path.is_relative() ? param::proj_dir / path : path;
}
}

State_UFO::State_UFO(int state_mode, std::string state_string)
    : FSMState(state_mode, std::move(state_string))
{
    const auto cfg = param::config["FSM"][getStateString()];
    entry_blend_s_ = cfg["entry_blend_s"] ? cfg["entry_blend_s"].as<float>() : 1.0f;
    load_reference(resolve_path(cfg["reference_file"].as<std::string>()));
    load_models(cfg);
    precompute_latents();

    registered_checks.insert(registered_checks.begin(), {
        [this]() { return inference_failed_.load(); }, FSMStringMap.right.at("Passive")
    });
    registered_checks.insert(registered_checks.begin(), {
        [this]() { return motion_done_.load(); }, FSMStringMap.right.at("Velocity")
    });
    registered_checks.emplace_back(
        [this]() {
            if (warmup_ticks_.fetch_add(1) < 1000) return false;
            const auto& quat = lowstate->msg_.imu_state().quaternion();
            Eigen::Quaternionf q(quat[0], quat[1], quat[2], quat[3]);
            const float gravity_z = (q.conjugate() * Eigen::Vector3f(0, 0, -1))[2];
            if (std::acos(std::clamp(-gravity_z, -1.0f, 1.0f)) > 1.0f) {
                spdlog::critical("[UFO] unsafe tilt detected; switching to Passive");
                return true;
            }
            return false;
        },
        FSMStringMap.right.at("Passive")
    );
}

State_UFO::~State_UFO() { exit(); }

void State_UFO::load_reference(const std::filesystem::path& path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("UFO: cannot open reference " + path.string());
    std::array<char, 8> magic{};
    stream.read(magic.data(), magic.size());
    uint32_t version = 0, width = 0;
    read_exact(stream, version);
    read_exact(stream, reference_frames_);
    read_exact(stream, width);
    read_exact(stream, reference_hz_);
    if (std::string(magic.data(), 7) != "UFOREF1" || version != 1 || width != kReferenceDim || reference_frames_ < 2 || std::abs(reference_hz_ - 50.0f) > 1e-4f)
        throw std::runtime_error("UFO: invalid reference contract");
    reference_.resize(static_cast<size_t>(reference_frames_) * kReferenceDim);
    stream.read(reinterpret_cast<char*>(reference_.data()), reference_.size() * sizeof(float));
    if (!stream || stream.peek() != std::ifstream::traits_type::eof())
        throw std::runtime_error("UFO: invalid reference payload length");
    spdlog::info("[UFO] Loaded {} reference frames from {}", reference_frames_, path.string());
}

void State_UFO::load_models(const YAML::Node& cfg)
{
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    for (const auto& node : cfg["policy_parts"]) {
        const auto path = resolve_path(node.as<std::string>());
        std::ifstream stream(path, std::ios::binary);
        if (!stream) throw std::runtime_error("UFO: cannot open model part " + path.string());
        policy_model_.insert(policy_model_.end(), std::istreambuf_iterator<char>(stream), {});
    }
    if (policy_model_.empty()) throw std::runtime_error("UFO: policy model is empty");
    policy_session_ = std::make_unique<Ort::Session>(ort_env_, policy_model_.data(), policy_model_.size(), session_options_);
    const auto encoder_path = resolve_path(cfg["encoder_path"].as<std::string>());
    encoder_session_ = std::make_unique<Ort::Session>(ort_env_, encoder_path.c_str(), session_options_);

    const auto policy_in = policy_session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto policy_out = policy_session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (policy_in.size() != 2 || policy_in[1] != kActorObsDim || policy_out.size() != 2 || policy_out[1] != kDof)
        throw std::runtime_error("UFO: unexpected policy tensor shape");
    spdlog::info("[UFO] Loaded policy ({} bytes) and backward encoder", policy_model_.size());
}

void State_UFO::precompute_latents()
{
    std::vector<float> states(static_cast<size_t>(reference_frames_) * kStateDim);
    std::vector<float> privileged(static_cast<size_t>(reference_frames_) * kPrivilegedDim);
    for (uint32_t frame = 0; frame < reference_frames_; ++frame) {
        const float* src = reference_.data() + static_cast<size_t>(frame) * kReferenceDim;
        std::copy_n(src, kStateDim, states.data() + static_cast<size_t>(frame) * kStateDim);
        std::copy_n(src + kStateDim, kPrivilegedDim, privileged.data() + static_cast<size_t>(frame) * kPrivilegedDim);
    }
    const std::array<int64_t, 2> state_shape{reference_frames_, kStateDim};
    const std::array<int64_t, 2> privileged_shape{reference_frames_, kPrivilegedDim};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
    std::array<Ort::Value, 2> inputs{
        Ort::Value::CreateTensor<float>(memory, states.data(), states.size(), state_shape.data(), state_shape.size()),
        Ort::Value::CreateTensor<float>(memory, privileged.data(), privileged.size(), privileged_shape.data(), privileged_shape.size())
    };
    const std::array<const char*, 2> input_names{"state", "privileged_state"};
    const std::array<const char*, 1> output_names{"z"};
    auto outputs = encoder_session_->Run(Ort::RunOptions{nullptr}, input_names.data(), inputs.data(), inputs.size(), output_names.data(), 1);
    const auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (shape.size() != 2 || shape[0] != reference_frames_ || shape[1] != kZDim)
        throw std::runtime_error("UFO: unexpected encoder output shape");
    const float* z = outputs[0].GetTensorData<float>();
    latents_.assign(z, z + static_cast<size_t>(reference_frames_) * kZDim);
    for (float value : latents_) if (!std::isfinite(value)) throw std::runtime_error("UFO: encoder produced non-finite latent");
    spdlog::info("[UFO] Precomputed {} task latents", reference_frames_);
}

std::array<float, State_UFO::kStateDim> State_UFO::read_robot_state()
{
    std::array<float, kStateDim> state{};
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    for (int i = 0; i < kDof; ++i) {
        const auto& motor = lowstate->msg_.motor_state()[motor_ids_[i]];
        state[i] = motor.q() - default_q_[i];
        state[kDof + i] = motor.dq();
    }
    const auto& imu = lowstate->msg_.imu_state();
    Eigen::Quaternionf q(imu.quaternion()[0], imu.quaternion()[1], imu.quaternion()[2], imu.quaternion()[3]);
    Eigen::Vector3f gravity = q.conjugate() * Eigen::Vector3f(0, 0, -1);
    for (int i = 0; i < 3; ++i) {
        state[46 + i] = gravity[i];
        state[49 + i] = static_cast<float>(imu.gyroscope()[i]) * 0.25f;
    }
    return state;
}

void State_UFO::enter()
{
    exit();
    motion_done_ = false;
    inference_failed_ = false;
    warmup_ticks_ = 0;
    enter_time_ = std::chrono::steady_clock::now();
    for (int i = 0; i < kDof; ++i) {
        const int motor_id = motor_ids_[i];
        entry_q_[i] = lowstate->msg_.motor_state()[motor_id].q();
        target_q_[i] = entry_q_[i];
        auto& cmd = lowcmd->msg_.motor_cmd()[motor_id];
        cmd.mode() = 0x01; cmd.q() = entry_q_[i]; cmd.dq() = 0; cmd.tau() = 0;
        cmd.kp() = kp_[i]; cmd.kd() = kd_[i];
    }
    for (int motor_id : {13, 14, 20, 21, 27, 28}) {
        auto& cmd = lowcmd->msg_.motor_cmd()[motor_id];
        cmd.mode() = 0x01; cmd.q() = lowstate->msg_.motor_state()[motor_id].q();
        cmd.dq() = 0; cmd.tau() = 0; cmd.kp() = 0; cmd.kd() = 0.5f;
    }
    policy_running_ = true;
    policy_thread_ = std::thread(&State_UFO::policy_loop, this);
    spdlog::info("[UFO] Entered: {:.2f}s reference, {:.2f}s entry blend", (reference_frames_ - 1) / reference_hz_, entry_blend_s_);
}

void State_UFO::policy_loop()
{
    std::array<float, kDof> last_action{};
    std::array<std::array<float, kDof>, kHistoryFrames> action_history{}, pos_history{}, vel_history{};
    std::array<std::array<float, 3>, kHistoryFrames> angular_history{}, gravity_history{};
    auto next = std::chrono::steady_clock::now();
    try {
        for (uint32_t frame = 1; policy_running_ && frame < reference_frames_; ++frame) {
            next += std::chrono::milliseconds(20);
            const auto state = read_robot_state();
            std::array<float, kActorObsDim> obs{};
            size_t offset = 0;
            std::copy(state.begin(), state.end(), obs.begin() + offset); offset += kStateDim;
            std::copy(last_action.begin(), last_action.end(), obs.begin() + offset); offset += kDof;
            for (const auto& history : action_history) { std::copy(history.begin(), history.end(), obs.begin() + offset); offset += kDof; }
            for (const auto& history : angular_history) { std::copy(history.begin(), history.end(), obs.begin() + offset); offset += 3; }
            for (const auto& history : pos_history) { std::copy(history.begin(), history.end(), obs.begin() + offset); offset += kDof; }
            for (const auto& history : vel_history) { std::copy(history.begin(), history.end(), obs.begin() + offset); offset += kDof; }
            for (const auto& history : gravity_history) { std::copy(history.begin(), history.end(), obs.begin() + offset); offset += 3; }
            const float* z = latents_.data() + static_cast<size_t>(frame) * kZDim;
            std::copy_n(z, kZDim, obs.begin() + offset);

            const std::array<int64_t, 2> shape{1, kActorObsDim};
            auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
            auto input = Ort::Value::CreateTensor<float>(memory, obs.data(), obs.size(), shape.data(), shape.size());
            const std::array<const char*, 1> input_names{"actor_obs"}, output_names{"action"};
            auto outputs = policy_session_->Run(Ort::RunOptions{nullptr}, input_names.data(), &input, 1, output_names.data(), 1);
            const float* raw = outputs[0].GetTensorData<float>();
            std::array<float, kDof> next_target{};
            for (int i = 0; i < kDof; ++i) {
                last_action[i] = std::clamp(raw[i] * 5.0f, -5.0f, 5.0f);
                next_target[i] = default_q_[i] + last_action[i] * action_scale_[i];
                if (!std::isfinite(next_target[i])) throw std::runtime_error("non-finite policy action");
            }
            for (int h = kHistoryFrames - 1; h > 0; --h) {
                action_history[h] = action_history[h - 1]; pos_history[h] = pos_history[h - 1];
                vel_history[h] = vel_history[h - 1]; angular_history[h] = angular_history[h - 1];
                gravity_history[h] = gravity_history[h - 1];
            }
            action_history[0] = last_action;
            for (int i = 0; i < kDof; ++i) { pos_history[0][i] = state[i]; vel_history[0][i] = state[kDof + i]; }
            for (int i = 0; i < 3; ++i) { gravity_history[0][i] = state[46 + i]; angular_history[0][i] = state[49 + i]; }
            { std::lock_guard<std::mutex> lock(target_mutex_); target_q_ = next_target; }
            std::this_thread::sleep_until(next);
        }
        if (policy_running_) motion_done_ = true;
    } catch (const std::exception& error) {
        spdlog::critical("[UFO] Inference failed: {}", error.what());
        inference_failed_ = true;
    }
}

void State_UFO::run()
{
    std::array<float, kDof> target;
    { std::lock_guard<std::mutex> lock(target_mutex_); target = target_q_; }
    const float blend = entry_blend_s_ <= 0 ? 1.0f : std::clamp(std::chrono::duration<float>(std::chrono::steady_clock::now() - enter_time_).count() / entry_blend_s_, 0.0f, 1.0f);
    for (int i = 0; i < kDof; ++i) {
        const int motor_id = motor_ids_[i];
        const float desired = entry_q_[i] + blend * (target[i] - entry_q_[i]);
        const float q = lowstate->msg_.motor_state()[motor_id].q();
        const float dq = lowstate->msg_.motor_state()[motor_id].dq();
        const float tau = std::clamp(kp_[i] * (desired - q) - kd_[i] * dq, -effort_limit_[i], effort_limit_[i]);
        auto& cmd = lowcmd->msg_.motor_cmd()[motor_id];
        cmd.q() = q + (tau + kd_[i] * dq) / kp_[i];
        cmd.dq() = 0; cmd.tau() = 0; cmd.kp() = kp_[i]; cmd.kd() = kd_[i];
    }
}

void State_UFO::exit()
{
    const bool was_running = policy_running_.exchange(false);
    if (policy_thread_.joinable()) policy_thread_.join();
    if (was_running) {
        test::velocity_soft_arms = true;
        spdlog::info("[UFO] Exited (soft handover to Velocity)");
    }
}
