#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"

REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    return {0.0f, 0.0f, 0.0f};//

    /*
    if (!FSMState::keyboard) { //
        return {0.0f, 0.0f, 0.0f};
    }
    */
    /*
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];
       
    static std::string last_logged_key = "";
    if(key != last_lo/root/unitree_rl_labgged_key && !key.empty()) {
        spdlog::info("Key detected: '{}' -> Command will be generated", key);
        last_logged_key = key;
    }

    // Optimized keyboard values based on curriculum training analysis
    // Forward/backward: policy generalizes well beyond training range (trained 0.1, works at 0.4)
    // Lateral/turning: limited to 50% of training max (NO curriculum, stayed at 0.1 entire training)
    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {0.4f, 0.0f, 0.0f}},    // Walk forward - generalizes well
        {"s", {-0.3f, 0.0f, 0.0f}},   // Walk backward - generalizes well  
        {"a", {0.0f, 0.05f, 0.0f}},   // Strafe left (50% of training max)
        {"d", {0.0f, -0.05f, 0.0f}},  // Strafe right (50% of training max)
        {"q", {0.0f, 0.0f, 0.05f}},   // Turn left (50% max - CRITICAL: no ang curriculum)
        {"e", {0.0f, 0.0f, -0.05f}}   // Turn right (50% max - CRITICAL: no ang curriculum)
    // return {0.0f, 0.0f, 0.0f};
    };
    
    // Smooth velocity command with exponential smoothing
    static std::vector<float> current_cmd = {0.0f, 0.0f, 0.0f};
    static std::vector<float> target_cmd = {0.0f, 0.0f, 0.0f};
    const float smoothing = 0.15f;  // Smooth acceleration (lower = smoother)
    
    // Update target based on key press
    if (key_commands.find(key) != key_commands.end())
    {
        target_cmd = key_commands[key];
        spdlog::info("Command: [{:.3f}, {:.3f}, {:.3f}]", target_cmd[0], target_cmd[1], target_cmd[2]);
    }
    else
    {
        target_cmd = {0.0f, 0.0f, 0.0f};  // Stop when no key pressed
    }
    
    // Smooth interpolation to target
    for(size_t i = 0; i < 3; i++) {
        current_cmd[i] += (target_cmd[i] - current_cmd[i]) * smoothing;
        // Deadzone for near-zero values
        if(std::abs(current_cmd[i]) < 0.01f) current_cmd[i] = 0.0f;
    }
    
    return current_cmd;
    */
/*
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];
    std::cout << "here /n";

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        // TODO: smooth and limit the velocity commands
        cmd = key_commands[key];
    }
    return cmd;
*/
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    spdlog::info("========================================");
    spdlog::info("Loading RL Policy from:");
    spdlog::info("  Policy Directory: {}", policy_dir.string());
    
    auto deploy_yaml = policy_dir / "params" / "deploy.yaml";
    auto policy_onnx = policy_dir / "exported" / "policy.onnx"; //
    
    spdlog::info("  Deploy Config: {}", deploy_yaml.string());
    spdlog::info("  Policy ONNX: {}", policy_onnx.string());
    
    // Check if files exist
    if(!std::filesystem::exists(deploy_yaml)) {
        spdlog::critical("Deploy YAML not found: {}", deploy_yaml.string());
        throw std::runtime_error("Deploy YAML file missing!");
    }
    if(!std::filesystem::exists(policy_onnx)) {
        spdlog::critical("Policy ONNX not found: {}", policy_onnx.string());
        throw std::runtime_error("Policy ONNX file missing!");
    }
    
    // Log file sizes and timestamps
    auto onnx_size = std::filesystem::file_size(policy_onnx);
    auto onnx_time = std::filesystem::last_write_time(policy_onnx);
    spdlog::info("  ONNX File Size: {} bytes ({:.2f} MB)", onnx_size, onnx_size / (1024.0 * 1024.0));
    spdlog::info("========================================");

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(deploy_yaml),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_onnx);
    _policy_onnx_path = policy_onnx.string();

    spdlog::info("Policy loaded successfully!");
    
    // bad_orientation → Passive (预热计数器在 enter() 里重置)
    this->registered_checks.emplace_back(
        std::make_pair(
            [&]() -> bool {
                if (_warmup < 1000) { _warmup++; return false; }
                float proj_g = env->robot->data.projected_gravity_b[2];
                float angle = std::fabs(std::acos(-proj_g));
                if (angle > 1.0) {
                    spdlog::critical("[bad_orientation] RLBase → Passive");
                    spdlog::critical("  projected_gravity_z = {:.4f}", proj_g);
                    spdlog::critical("  tilt_angle = {:.4f} rad ({:.1f} deg), limit = 1.0 rad",
                                     angle, angle * 180.0 / std::acos(-1.0));
                    spdlog::critical("  robot orientation is too tilted, entering Passive for safety!");
                    return true;
                }
                return false;
            },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    // ── 刚度 ramp：腿 30%→100%，上身 10%→100%（仅 WBC→Velocity 时生效）──
    if (_soft_arm_ramp) {
        auto elapsed = std::chrono::steady_clock::now() - _arm_ramp_start;
        float t = std::min(1.0f, std::chrono::duration<float>(elapsed).count() / RAMP_DURATION);
        float leg_factor   = SOFT_LEG   + (1.0f - SOFT_LEG)   * t;
        float upper_factor = SOFT_UPPER + (1.0f - SOFT_UPPER) * t;
        for (int i = 0; i < TOTAL_MOTORS; i++) {
            float factor = (i <= LEG_END) ? leg_factor : upper_factor;
            float full_kp = (i < static_cast<int>(env->robot->data.joint_stiffness.size()))
                            ? env->robot->data.joint_stiffness[i] : DEFAULT_KP;
            float full_kd = (i < static_cast<int>(env->robot->data.joint_damping.size()))
                            ? env->robot->data.joint_damping[i] : DEFAULT_KD;
            lowcmd->msg_.motor_cmd()[i].kp() = full_kp * factor;
            lowcmd->msg_.motor_cmd()[i].kd() = full_kd * factor;
        }
        if (t >= 1.0f) {
            _soft_arm_ramp = false;
            spdlog::info("[Velocity] Stiffness restored to 100%");
        }
    }

    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}