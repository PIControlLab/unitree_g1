#pragma once

#include <algorithm>
#include <deque>
#include <functional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace isaaclab
{

class ManagerBasedRLEnv;
using ObsFunc = std::function<std::vector<float>(ManagerBasedRLEnv*, YAML::Node)>;

enum class HistoryOrder
{
    OldestToNewest,
    NewestToOldest,
};

enum class HistoryReset
{
    Zeros,
    RepeatFirst,
};

enum class HistoryUpdatePhase
{
    BeforeInference,
    AfterInference,
};

struct ObservationTermCfg
{
    std::string source;
    YAML::Node params;
    ObsFunc func;
    std::vector<float> clip;
    std::vector<float> scale;
    int history_length = 1;
    HistoryOrder history_order = HistoryOrder::OldestToNewest;
    HistoryReset history_reset = HistoryReset::RepeatFirst;
    HistoryUpdatePhase update_phase = HistoryUpdatePhase::BeforeInference;
    bool scale_first = false;

    std::vector<float> sample(ManagerBasedRLEnv* env) const
    {
        auto value = func(env, params);
        for (size_t i = 0; i < value.size(); ++i)
        {
            if (scale_first)
            {
                if (!scale.empty()) value[i] *= scale[i];
                if (!clip.empty()) value[i] = std::clamp(value[i], clip[0], clip[1]);
            }
            else
            {
                if (!clip.empty()) value[i] = std::clamp(value[i], clip[0], clip[1]);
                if (!scale.empty()) value[i] *= scale[i];
            }
        }
        return value;
    }

    void reset(const std::vector<float>& first)
    {
        history_.clear();
        const std::vector<float> initial = history_reset == HistoryReset::Zeros
            ? std::vector<float>(first.size(), 0.0f)
            : first;
        for (int i = 0; i < history_length; ++i) history_.push_back(initial);
    }

    void update(std::vector<float> value)
    {
        history_.push_back(std::move(value));
        history_.pop_front();
    }

    const std::vector<float>& frame(int index) const
    {
        const int resolved = history_order == HistoryOrder::OldestToNewest
            ? index
            : history_length - 1 - index;
        return history_[resolved];
    }

private:
    std::deque<std::vector<float>> history_;
};

} // namespace isaaclab
