#pragma once

#include "FSM/State_PolicyBase.h"
#include "policy/UfoLatentReference.h"
#include <memory>

class State_UFO : public State_PolicyBase
{
public:
    State_UFO(int state_mode, const std::string& state_string);

private:
    void reset_reference();
    void update_reference(long step);

    std::shared_ptr<policy::UfoLatentReference> reference_;
    std::string hold_after_finished_;
    bool holding_finished_pose_ = false;
    long active_start_step_ = 0;
};

REGISTER_FSM(State_UFO)