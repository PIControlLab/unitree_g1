// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/common/thread/recurrent_thread.hpp>
#include "BaseState.h"
#include "FSMState.h"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstdio>
#include <fstream>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }
    }

    void start() 
    {
        // Start From State_Passive
        currentState = states[0];
        currentState->enter();
        publish_state();

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
        spdlog::info("FSM: Start {}", currentState->getStateString());
        for (const auto& state : states) state->prepare();
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;
    void publish_state() const
    {
        static const std::string path = "/dev/shm/unitree_fsm_state.json";
        const std::string temporary = path + ".tmp";
        {
            std::ofstream stream(temporary, std::ios::trunc);
            if (!stream)
            {
                spdlog::warn("FSM: cannot publish state to {}", temporary);
                return;
            }
            stream << "{\"state\":\"" << currentState->getStateString() << "\"}\n";
        }
        if (std::rename(temporary.c_str(), path.c_str()) != 0)
            spdlog::warn("FSM: cannot publish state to {}", path);
    }

    void run_()
    {
        currentState->pre_run();

        // Check transitions immediately after input/state refresh so edge-triggered
        // conditions like `on_pressed` are evaluated in the same control tick.
        int nextStateMode = pending_state_mode_;
        for(int i(0); i<currentState->registered_checks.size(); i++)
        {
            const bool matched = currentState->registered_checks[i].first();
            if(matched)
            {
                nextStateMode = currentState->registered_checks[i].second;
                break;
            }
        }
        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    if (!state->can_enter())
                    {
                        if (pending_state_mode_ != nextStateMode)
                            spdlog::info("FSM: Waiting for {} preparation", state->getStateString());
                        pending_state_mode_ = nextStateMode;
                        break;
                    }
                    spdlog::info("FSM: Change state from {} to {}  ",
                        currentState->getStateString(), state->getStateString());
                    pending_state_mode_ = 0;
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    publish_state();
                    break;
                }
            }
        }

        if (nextStateMode != 0 && currentState->isState(nextStateMode))
            pending_state_mode_ = 0;

        currentState->run();
        currentState->post_run();
    }

    std::shared_ptr<BaseState> currentState;
    int pending_state_mode_ = 0;
    unitree::common::RecurrentThreadPtr fsm_thread_;
};
