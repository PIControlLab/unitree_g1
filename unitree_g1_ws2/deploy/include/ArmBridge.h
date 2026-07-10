#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

// Reads arm joint commands from shared memory JSON (written by arm_ctrl.py).
// Overrides motor commands in the LowCmd message for specified joints.
//
// JSON format expected:
// {
//   "joints": {
//     "22": {"q": 0.5, "kp": 60.0, "kd": 1.5, "tau": 0.0},
//     "23": {"q": -0.3, "kp": 60.0, "kd": 1.5, "tau": 0.0}
//   },
//   "seq": 1,
//   "timeout": 0.3
// }
class ArmBridge {
public:
    explicit ArmBridge(std::string json_path = "/dev/shm/unitree_arm_bridge.json",
                       double default_timeout = 0.3)
        : json_path_(std::move(json_path)), timeout_(default_timeout) {}

    // Apply arm joint overrides to the LowCmd message.
    // Returns true if external arm commands are active.
    template <typename LowCmdMsg>
    bool apply(LowCmdMsg& lowcmd_msg) {
        auto payload = load_payload();
        if (!payload || payload->joints.empty()) {
            return false;
        }

        // Check if data is fresh (timeout-based)
        auto now = std::chrono::steady_clock::now();
        double age = std::chrono::duration<double>(now - payload->recv_time).count();
        double effective_timeout = payload->timeout > 0 ? payload->timeout : timeout_;

        if (age > effective_timeout) {
            // stale data – don't override, let policy control arms
            if (was_active_) {
                spdlog::info("[ArmBridge] Timed out ({:.2f}s > {:.2f}s), releasing arm control",
                             age, effective_timeout);
                was_active_ = false;
            }
            return false;
        }

        if (!was_active_) {
            spdlog::info("[ArmBridge] Active — {} joints, age={:.3f}s",
                         payload->joints.size(), age);
            was_active_ = true;
        }

        // Override motor commands for specified joints
        for (const auto& [motor_idx_str, cmd] : payload->joints) {
            int motor_idx = std::stoi(motor_idx_str);
            if (motor_idx < 0 || motor_idx >= static_cast<int>(lowcmd_msg.motor_cmd().size())) {
                continue;
            }

            auto& m = lowcmd_msg.motor_cmd()[motor_idx];
            m.mode() = 0x01;  // servo mode
            m.q() = cmd.q;
            m.dq() = cmd.dq;
            m.kp() = cmd.kp;
            m.kd() = cmd.kd;
            m.tau() = cmd.tau;
        }

        return true;
    }

private:
    struct JointCmd {
        double q = 0.0;
        double dq = 0.0;
        double kp = 0.0;
        double kd = 0.0;
        double tau = 0.0;
    };

    struct Payload {
        std::unordered_map<std::string, JointCmd> joints;
        long long seq = -1;
        double timeout = 0.3;
        std::chrono::steady_clock::time_point recv_time;
    };

    std::optional<Payload> load_payload() {
        namespace fs = std::filesystem;
        std::error_code ec;
        auto mtime = fs::last_write_time(json_path_, ec);
        if (ec) {
            return std::nullopt;
        }
        if (last_mtime_.has_value() && mtime == *last_mtime_) {
            return last_payload_;
        }

        std::ifstream file(json_path_);
        if (!file.good()) {
            return std::nullopt;
        }

        nlohmann::json j;
        try {
            file >> j;
        } catch (const std::exception& exc) {
            spdlog::warn("[ArmBridge] Failed to parse JSON: {}", exc.what());
            return std::nullopt;
        }

        Payload payload;
        payload.recv_time = std::chrono::steady_clock::now();

        if (j.contains("timeout") && j["timeout"].is_number()) {
            payload.timeout = j["timeout"].get<double>();
        }

        if (j.contains("joints") && j["joints"].is_object()) {
            for (auto& [key, value] : j["joints"].items()) {
                JointCmd cmd;
                if (value.contains("q")) cmd.q = value["q"].get<double>();
                if (value.contains("dq")) cmd.dq = value["dq"].get<double>();
                if (value.contains("kp")) cmd.kp = value["kp"].get<double>();
                if (value.contains("kd")) cmd.kd = value["kd"].get<double>();
                if (value.contains("tau")) cmd.tau = value["tau"].get<double>();
                payload.joints[key] = cmd;
            }
        }

        if (j.contains("seq") && j["seq"].is_number_integer()) {
            payload.seq = j["seq"].get<long long>();
        }

        last_mtime_ = mtime;
        last_payload_ = payload;
        return last_payload_;
    }

    std::string json_path_;
    double timeout_;
    std::optional<std::filesystem::file_time_type> last_mtime_;
    std::optional<Payload> last_payload_;
    bool was_active_ = false;
};
