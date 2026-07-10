#pragma once

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

// This adapter reads the latest JSON state written by receiver_bridge.py and
// folds it into the Unitree joystick object already used by unitree_rl_lab.
//
// Assumptions based on unitree_rl_lab's joystick DSL:
// - joystick has members: back/start/LS/RS/LB/RB/A/B/X/Y/up/down/left/right/lx/ly/rx/ry/LT/RT
// - each member exposes: pressed, on_pressed, on_released, pressed_time
//
// You can call bridge.apply(lowstate->joystick) once per control loop after
// lowstate->update() to override button states with the forwarded controller.
class UnitreeJoystickBridge {
public:
    explicit UnitreeJoystickBridge(
        std::string json_path = "/dev/shm/unitree_joystick_bridge.json",
        double axis_press_threshold = 0.2)
        : json_path_(std::move(json_path)), axis_press_threshold_(axis_press_threshold) {}

    template <typename JoystickT>
    void apply(JoystickT& joystick) {
        auto payload = load_payload();
        if (!payload) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto dt = last_apply_time_.has_value()
            ? std::chrono::duration<double>(now - *last_apply_time_).count()
            : 0.0;
        last_apply_time_ = now;

        update_button(joystick.A, "a", payload->buttons, dt);
        update_button(joystick.B, "b", payload->buttons, dt);
        update_button(joystick.X, "x", payload->buttons, dt);
        update_button(joystick.Y, "y", payload->buttons, dt);
        update_button(joystick.LB, "lb", payload->buttons, dt);
        update_button(joystick.RB, "rb", payload->buttons, dt);
        update_button(joystick.back, "back", payload->buttons, dt);
        update_button(joystick.start, "start", payload->buttons, dt);
        update_button(joystick.LS, "ls", payload->buttons, dt);
        update_button(joystick.RS, "rs", payload->buttons, dt);
        update_button(joystick.up, "up", payload->buttons, dt);
        update_button(joystick.down, "down", payload->buttons, dt);
        update_button(joystick.left, "left", payload->buttons, dt);
        update_button(joystick.right, "right", payload->buttons, dt);

        joystick.lx(static_cast<float>(payload->axes.at("lx")));
        joystick.ly(static_cast<float>(payload->axes.at("ly")));
        joystick.rx(static_cast<float>(payload->axes.at("rx")));
        joystick.ry(static_cast<float>(payload->axes.at("ry")));
        joystick.LT(static_cast<float>(payload->axes.at("lt")));
        joystick.RT(static_cast<float>(payload->axes.at("rt")));

    }

    template <typename EnvT>
    std::vector<float> velocity_command(const EnvT& env) const {
        std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
        auto payload = last_payload_;
        if (!payload) {
            return cmd;
        }

        auto ranges = env->cfg["commands"]["base_velocity"]["ranges"];
        const float lin_x = payload->axes.find("ly") != payload->axes.end()
            ? static_cast<float>(-payload->axes.at("ly")) * ranges["lin_vel_x"][1].template as<float>()
            : 0.0f;
        const float lin_y = payload->axes.find("lx") != payload->axes.end()
            ? static_cast<float>(-payload->axes.at("lx")) * ranges["lin_vel_y"][1].template as<float>()
            : 0.0f;
        const float ang_z = payload->axes.find("rx") != payload->axes.end()
            ? static_cast<float>(-payload->axes.at("rx")) * ranges["ang_vel_z"][1].template as<float>()
            : 0.0f;
        cmd[0] = lin_x;
        cmd[1] = lin_y;
        cmd[2] = ang_z;
        return cmd;
    }

private:
    struct Payload {
        std::unordered_map<std::string, double> axes;
        std::unordered_map<std::string, bool> buttons;
        long long seq = -1;
        double source_ts = 0.0;
        double recv_ts = 0.0;
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
            spdlog::warn("Failed to parse joystick bridge JSON: {}", exc.what());
            return std::nullopt;
        }

        Payload payload;
        if (j.contains("axes") && j["axes"].is_object()) {
            for (auto& [key, value] : j["axes"].items()) {
                if (value.is_number()) {
                    payload.axes[key] = value.get<double>();
                }
            }
        }
        if (j.contains("buttons") && j["buttons"].is_object()) {
            for (auto& [key, value] : j["buttons"].items()) {
                payload.buttons[key] = value.get<int>() != 0;
            }
        }
        if (j.contains("seq") && j["seq"].is_number_integer()) {
            payload.seq = j["seq"].get<long long>();
        }
        if (j.contains("source_ts") && j["source_ts"].is_number()) {
            payload.source_ts = j["source_ts"].get<double>();
        }
        if (j.contains("recv_ts") && j["recv_ts"].is_number()) {
            payload.recv_ts = j["recv_ts"].get<double>();
        }

        last_mtime_ = mtime;
        last_payload_ = payload;
        return last_payload_;
    }

    template <typename KeyT>
    void update_button(KeyT& key, const std::string& name, const std::unordered_map<std::string, bool>& buttons, double dt) {
        auto it = buttons.find(name);
        const bool pressed = it != buttons.end() ? it->second : false;
        set_key_state(key, pressed, dt);
    }

    template <typename KeyT>
    void update_axis_as_key(KeyT& key, bool pressed, double dt) {
        set_key_state(key, pressed, dt);
    }

    template <typename KeyT>
    void set_key_state(KeyT& key, bool pressed, double dt) {
        const bool was_pressed = key.pressed;
        key.pressed = pressed;
        key.on_pressed = pressed && !was_pressed;
        key.on_released = !pressed && was_pressed;
        key.pressed_time = pressed ? (was_pressed ? key.pressed_time + static_cast<float>(dt) : 0.0f) : 0.0f;
    }

    std::string json_path_;
    double axis_press_threshold_;
    std::optional<std::filesystem::file_time_type> last_mtime_;
    std::optional<std::chrono::steady_clock::time_point> last_apply_time_;
    std::optional<Payload> last_payload_;
};
