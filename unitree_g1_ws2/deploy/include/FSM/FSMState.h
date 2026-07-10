#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"
#include "UnitreeJoystickBridge.h"
#include "ArmBridge.h"
#include "PolicyPublisher.h"
#include "memory"
#include <chrono>

namespace test {
    inline std::unique_ptr<UnitreeJoystickBridge> remote_joystick;
    inline std::unique_ptr<ArmBridge> arm_bridge;
    inline std::optional<std::chrono::steady_clock::time_point> last_combo_log_time_;
    inline bool velocity_soft_arms = false;  // WBC/Mimic 退出前设 true，Velocity 据此决定是否软臂
}


class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // timeout 检查已禁用（使用 joystick_emu.py JSON 桥接时不适用）
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
        if(test::remote_joystick) {//
            test::remote_joystick->apply(lowstate->joystick);
        }

        auto now = std::chrono::steady_clock::now();
        if(lowstate->joystick.LT.pressed && lowstate->joystick.up.on_pressed) {
            test::last_combo_log_time_ = now;
        }
        if(lowstate->joystick.RB.pressed && lowstate->joystick.X.on_pressed) {
            test::last_combo_log_time_ = now;
        }
        if(lowstate->joystick.LT.pressed && lowstate->joystick.B.on_pressed) {
            test::last_combo_log_time_ = now;
        }
    }

    void post_run()
    {
        if (policy_publisher) {
            policy_publisher->write(lowcmd->msg_);
        } else {
            lowcmd->unlockAndPublish();
        }
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
    static std::unique_ptr<PolicyPublisher> policy_publisher;
};