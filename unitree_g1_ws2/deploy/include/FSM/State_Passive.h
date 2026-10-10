#pragma once

#include "FSM/State_CommandBase.h"

class State_Passive : public State_CommandBase
{
public:
    State_Passive(int state_mode, const std::string& state_string = "Passive")
        : State_CommandBase(state_mode, state_string) {}

    void enter() override
    {
        configure_motors();
    }

    void run() override
    {
        write_joint_pos(current_joint_pos());
    }
};

REGISTER_FSM(State_Passive)
