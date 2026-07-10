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

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
        spdlog::info("FSM: Start {}", currentState->getStateString());
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

    void run_()
    {
        currentState->pre_run();

        // Check transitions immediately after input/state refresh so edge-triggered
        // conditions like `on_pressed` are evaluated in the same control tick.
        int nextStateMode = 0;
        const auto now = std::chrono::steady_clock::now();
        const bool trace_checks = false; // const bool trace_checks = test::last_combo_log_time_.has_value() &&
            std::chrono::duration<double>(now - *test::last_combo_log_time_).count() < 1.0;
        if (trace_checks) {
            spdlog::info("FSM debug: current state={} check_count={}", currentState->getStateString(), currentState->registered_checks.size());
        }
        for(int i(0); i<currentState->registered_checks.size(); i++)
        {
            const bool matched = currentState->registered_checks[i].first();
            if (trace_checks) {
                const int target = currentState->registered_checks[i].second;
                std::string target_name = FSMStringMap.left.count(target) ? FSMStringMap.left.at(target) : std::to_string(target);
                spdlog::info("FSM debug: check[{}] matched={} target={}", i, matched, target_name);
            }
            if(matched)
            {
                nextStateMode = currentState->registered_checks[i].second;
                break;
            }
        }
        if (trace_checks) {
            if(nextStateMode == 0) {
                spdlog::info("FSM debug: no transition matched");
            } else {
                std::string next_name = FSMStringMap.left.count(nextStateMode) ? FSMStringMap.left.at(nextStateMode) : std::to_string(nextStateMode);
                spdlog::info("FSM debug: nextStateMode={}", next_name);
            }
        }

        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    spdlog::info("FSM: Change state from {} to {}  ",
                        currentState->getStateString(), state->getStateString());
                    // 打印全部 29 个关节位置，方便调切换时的姿态偏差
                    if (FSMState::lowstate) {
                        std::string joints;
                        for (int j = 0; j < 29; j++) {
                            char buf[32];
                            snprintf(buf, sizeof(buf), " %d:%.4f", j,
                                     FSMState::lowstate->msg_.motor_state()[j].q());
                            joints += buf;
                        }
                        spdlog::info("FSM joint positions (29):{}", joints);
                    }
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    break;
                }
            }
        }

        currentState->run();
        currentState->post_run();
    }

    std::shared_ptr<BaseState> currentState;
    unitree::common::RecurrentThreadPtr fsm_thread_;
};