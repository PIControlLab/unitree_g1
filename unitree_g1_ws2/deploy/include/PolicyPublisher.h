#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

// Writes the complete LowCmd message to shared memory JSON at each control tick.
// A Python merger node reads this and publishes via ROS 2 to /lowcmd.
class PolicyPublisher {
public:
    explicit PolicyPublisher(std::string json_path = "/dev/shm/unitree_policy_bridge.json")
        : json_path_(std::move(json_path))
    {
        spdlog::info("[PolicyPublisher] Writing to {}", json_path_);
    }

    template <typename LowCmdMsg>
    void write(const LowCmdMsg& msg) {
        ++seq_;
        nlohmann::json j;
        j["mode_pr"] = msg.mode_pr();
        j["mode_machine"] = msg.mode_machine();
        j["seq"] = seq_;

        auto& motors_json = j["motors"];
        for (size_t i = 0; i < msg.motor_cmd().size(); ++i) {
            const auto& m = msg.motor_cmd()[i];
            // Only include motors that have non-default mode (to keep JSON small)
            nlohmann::json mj;
            mj["mode"] = m.mode();
            mj["q"] = m.q();
            mj["dq"] = m.dq();
            mj["kp"] = m.kp();
            mj["kd"] = m.kd();
            mj["tau"] = m.tau();
            motors_json[std::to_string(i)] = mj;
        }

        // Atomic write
        std::string tmp = json_path_ + ".tmp";
        std::ofstream ofs(tmp);
        ofs << j.dump();
        ofs.close();
        std::filesystem::rename(tmp, json_path_);

        if (first_write_) {
            spdlog::info("[PolicyPublisher] First write OK, {} motors", msg.motor_cmd().size());
            first_write_ = false;
        }
    }

private:
    std::string json_path_;
    long long seq_ = 0;
    bool first_write_ = true;
};
