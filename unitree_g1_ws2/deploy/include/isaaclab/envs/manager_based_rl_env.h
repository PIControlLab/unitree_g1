#pragma once

#include "isaaclab/algorithms/algorithms.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/manager/observation_manager.h"
#include "policy/RobotProfile.h"
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace isaaclab
{

class ManagerBasedRLEnv
{
public:
    using ObservationSources = std::unordered_map<std::string, ObsFunc>;

    ManagerBasedRLEnv(
        YAML::Node cfg,
        std::shared_ptr<Articulation> robot,
        policy::JointLayout layout,
        ObservationSources external_sources = {})
        : cfg(std::move(cfg)), robot(std::move(robot)), external_sources_(std::move(external_sources))
    {
        step_dt = this->cfg["timing"]["policy_dt"].as<float>();
        this->robot->data.joint_names = std::move(layout.names);
        this->robot->data.joint_ids_map = std::move(layout.motor_ids);
        this->robot->data.joint_active = std::move(layout.active);
        this->robot->data.joint_pos.resize(this->robot->data.joint_names.size());
        this->robot->data.joint_vel.resize(this->robot->data.joint_names.size());

        const auto joint_cfg = this->cfg["joints"];
        this->robot->data.default_joint_pos = to_eigen(joint_cfg["default_joint_pos"]);
        this->robot->data.joint_stiffness = this->cfg["control"]["stiffness"].as<std::vector<float>>();
        this->robot->data.joint_damping = this->cfg["control"]["damping"].as<std::vector<float>>();
        this->robot->data.joint_effort_limit = this->cfg["control"]["effort_limit"].as<std::vector<float>>();

        this->robot->update();
        const auto defaults = joint_cfg["default_joint_pos"].as<std::vector<float>>();
        action_manager = std::make_unique<ActionManager>(this->cfg["actions"], defaults);
        observation_manager = std::make_unique<ObservationManager>(
            this->cfg["observations"], this,
            [this](const std::string& source) { return resolve_observation(source); });
    }

    void reset()
    {
        global_phase = 0.0f;
        episode_length = 0;
        robot->update();
        action_manager->reset();
        observation_manager->reset();
    }

    void step()
    {
        robot->update();
        if (before_step) before_step(episode_length);
        auto obs = observation_manager->compute();
        auto action = alg->act(obs);
        action_manager->process_action(action);
        observation_manager->update_after_inference();
        ++episode_length;
        if (after_step) after_step(episode_length);
    }

    float step_dt = 0.0f;
    YAML::Node cfg;
    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    float global_phase = 0.0f;
    std::function<void(long)> before_step;
    std::function<void(long)> after_step;

private:
    static Eigen::VectorXf to_eigen(const YAML::Node& node)
    {
        const auto values = node.as<std::vector<float>>();
        return Eigen::VectorXf::Map(values.data(), values.size());
    }

    ObsFunc resolve_observation(const std::string& source)
    {
        const auto external = external_sources_.find(source);
        if (external != external_sources_.end()) return external->second;
        return observations_map().at(source);
    }

    ObservationSources external_sources_;
};

} // namespace isaaclab
