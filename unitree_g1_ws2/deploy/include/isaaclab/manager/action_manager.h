#pragma once

#include <algorithm>
#include <mutex>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace isaaclab
{

class ManagerBasedRLEnv;

class ActionManager
{
public:
    ActionManager(YAML::Node cfg, const std::vector<float>& default_joint_pos)
        : default_joint_pos_(default_joint_pos),
          raw_action_scale_(cfg["raw_action_scale"].as<float>()),
          action_target_scale_(cfg["action_target_scale"].as<std::vector<float>>()),
          add_default_joint_pos_(cfg["add_default_joint_pos"].as<bool>())
    {
        if (cfg["raw_action_clip"] && !cfg["raw_action_clip"].IsNull())
            raw_action_clip_ = cfg["raw_action_clip"].as<std::vector<float>>();
        if (cfg["position_clip"] && !cfg["position_clip"].IsNull())
            position_clip_ = cfg["position_clip"].as<std::vector<std::vector<float>>>();
        reset();
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        raw_actions_.assign(action_target_scale_.size(), 0.0f);
        normalized_actions_.assign(action_target_scale_.size(), 0.0f);
        position_targets_ = add_default_joint_pos_
            ? default_joint_pos_
            : std::vector<float>(action_target_scale_.size(), 0.0f);
    }

    void process_action(const std::vector<float>& actions)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        raw_actions_ = actions;
        for (size_t i = 0; i < actions.size(); ++i)
        {
            normalized_actions_[i] = actions[i] * raw_action_scale_;
            if (!raw_action_clip_.empty())
                normalized_actions_[i] = std::clamp(
                    normalized_actions_[i], raw_action_clip_[0], raw_action_clip_[1]);
            position_targets_[i] = normalized_actions_[i] * action_target_scale_[i];
            if (add_default_joint_pos_) position_targets_[i] += default_joint_pos_[i];
            if (!position_clip_.empty())
                position_targets_[i] = std::clamp(
                    position_targets_[i], position_clip_[i][0], position_clip_[i][1]);
        }
    }

    std::vector<float> raw_actions() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return raw_actions_;
    }

    std::vector<float> normalized_actions() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return normalized_actions_;
    }

    std::vector<float> processed_actions() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return position_targets_;
    }

    int action_dim() const { return static_cast<int>(action_target_scale_.size()); }

private:
    std::vector<float> default_joint_pos_;
    float raw_action_scale_;
    std::vector<float> raw_action_clip_;
    std::vector<float> action_target_scale_;
    bool add_default_joint_pos_;
    std::vector<std::vector<float>> position_clip_;

    mutable std::mutex mutex_;
    std::vector<float> raw_actions_;
    std::vector<float> normalized_actions_;
    std::vector<float> position_targets_;
};

} // namespace isaaclab
