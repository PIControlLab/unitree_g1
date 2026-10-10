#pragma once

#include "FSM/State_PolicyBase.h"

class State_Velocity : public State_PolicyBase
{
public:
    State_Velocity(int state_mode, const std::string& state_string)
        : State_PolicyBase(state_mode, state_string)
    {
        initialize();
    }
};

REGISTER_FSM(State_Velocity)
