// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

inline std::vector<int> observation_joint_ids(const YAML::Node& params)
{
    const auto asset_cfg = params["asset_cfg"];
    return asset_cfg && asset_cfg["joint_ids"]
        ? asset_cfg["joint_ids"].as<std::vector<int>>()
        : std::vector<int>{};
}

REGISTER_OBSERVATION(base_ang_vel)
{
    auto & asset = env->robot;
    auto & data = asset->data.root_ang_vel_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(projected_gravity)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(joint_pos)
{
    auto & asset = env->robot;
    std::vector<float> data;

    const auto joint_ids = observation_joint_ids(params);

    if(joint_ids.empty())
    {
        data.resize(asset->data.joint_pos.size());
        for(size_t i = 0; i < asset->data.joint_pos.size(); ++i)
        {
            data[i] = asset->data.joint_pos[i];
        }
    }
    else
    {
        data.resize(joint_ids.size());
        for(size_t i = 0; i < joint_ids.size(); ++i)
        {
            data[i] = asset->data.joint_pos[joint_ids[i]];
        }
    }

    return data;
}

REGISTER_OBSERVATION(joint_pos_rel)
{
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for(size_t i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    const auto joint_ids = observation_joint_ids(params);
    if(!joint_ids.empty()) {
        std::vector<float> selected(joint_ids.size());
        for(size_t i = 0; i < joint_ids.size(); ++i){
            selected[i] = data[joint_ids[i]];
        }
        data = std::move(selected);
    }

    return data;
}

REGISTER_OBSERVATION(joint_vel)
{
    auto & asset = env->robot;
    auto data = asset->data.joint_vel;

    const auto joint_ids = observation_joint_ids(params);
    if(!joint_ids.empty()) {
        data.resize(joint_ids.size());
        for(size_t i = 0; i < joint_ids.size(); ++i) {
            data[i] = asset->data.joint_vel[joint_ids[i]];
        }
    }
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(last_action)
{
    auto data = env->action_manager->normalized_actions();
    return std::vector<float>(data.data(), data.data() + data.size());
};

REGISTER_OBSERVATION(velocity_commands)
{
    std::vector<float> obs(3);
    auto & joystick = env->robot->data.joystick;

    const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    const float raw_ly = joystick->ly();
    const float raw_lx = joystick->lx();
    const float raw_rx = joystick->rx();

    obs[0] = std::clamp(-raw_ly, cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
    obs[1] = std::clamp(-raw_lx, cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    obs[2] = std::clamp(-raw_rx, cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());

    return obs;
}

REGISTER_OBSERVATION(gait_phase)
{
    float period = params["period"].as<float>();
    float delta_phase = env->step_dt * (1.0f / period);

    env->global_phase += delta_phase;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2 * M_PI);
    obs[1] = std::cos(env->global_phase * 2 * M_PI);
    return obs;
}

}
}
