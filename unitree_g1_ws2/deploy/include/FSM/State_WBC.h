#pragma once

#include "FSM/State_PolicyBase.h"
#include "policy/ReferenceData.h"
#include <atomic>
#include <memory>
#include <vector>

class State_WBC : public State_PolicyBase
{
public:
    State_WBC(int state_mode, const std::string& state_string);

private:
    void reset_reference();
    void update_reference(long step);
    void compute_reference_observation();
    std::vector<float> reference_slice(size_t begin, size_t end) const;

    std::shared_ptr<policy::ReferenceData> reference_;
    std::vector<float> reference_observation_;
    std::atomic<bool> unsafe_reference_{false};
    int start_frame_ = 0;
    int end_frame_ = 0;
    int current_frame_ = 0;
    int frame_interval_ = 1;
    int sample_offset_steps_ = 0;
    float gravity_error_threshold_ = 0.0f;
};

REGISTER_FSM(State_WBC)
