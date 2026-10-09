#pragma once

#include "isaaclab/manager/manager_term_cfg.h"
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaaclab
{

using ObsMap = std::map<std::string, ObsFunc>;

inline ObsMap& observations_map()
{
    static ObsMap instance;
    return instance;
}

#define REGISTER_OBSERVATION(name) \
    inline std::vector<float> name(ManagerBasedRLEnv* env, YAML::Node params); \
    inline struct name##_registrar { \
        name##_registrar() { observations_map()[#name] = name; } \
    } name##_registrar_instance; \
    inline std::vector<float> name(ManagerBasedRLEnv* env, YAML::Node params)

class ObservationManager
{
public:
    using Resolver = std::function<ObsFunc(const std::string&)>;

    ObservationManager(YAML::Node cfg, ManagerBasedRLEnv* env, Resolver resolver)
        : env_(env)
    {
        for (auto input = cfg.begin(); input != cfg.end(); ++input)
        {
            InputCfg input_cfg;
            input_cfg.name = input->first.as<std::string>();
            for (const auto& block_node : input->second["blocks"])
            {
                BlockCfg block;
                block.layout = block_node["layout"].as<std::string>();
                for (const auto& term_node : block_node["terms"])
                {
                    ObservationTermCfg term;
                    term.source = term_node["source"].as<std::string>();
                    term.params = term_node["params"]
                        ? term_node["params"]
                        : YAML::Node(YAML::NodeType::Map);
                    term.func = resolver(term.source);
                    term.scale = read_vector(term_node["scale"]);
                    term.clip = read_vector(term_node["clip"]);
                    term.scale_first = term_node["scale_first"].as<bool>(false);

                    if (const auto history = term_node["history"])
                    {
                        term.history_length = history["length"].as<int>(1);
                        term.history_order = history["order"].as<std::string>("oldest_to_newest") == "newest_to_oldest"
                            ? HistoryOrder::NewestToOldest
                            : HistoryOrder::OldestToNewest;
                        term.history_reset = history["reset"].as<std::string>("repeat_first") == "zeros"
                            ? HistoryReset::Zeros
                            : HistoryReset::RepeatFirst;
                        term.update_phase = history["update_phase"].as<std::string>("before_inference") == "after_inference"
                            ? HistoryUpdatePhase::AfterInference
                            : HistoryUpdatePhase::BeforeInference;
                    }
                    block.terms.push_back(std::move(term));
                }
                input_cfg.blocks.push_back(std::move(block));
            }
            inputs_.push_back(std::move(input_cfg));
        }
    }

    void reset()
    {
        for (auto& input : inputs_)
            for (auto& block : input.blocks)
                for (auto& term : block.terms)
                    term.reset(term.sample(env_));
    }

    std::unordered_map<std::string, std::vector<float>> compute()
    {
        std::unordered_map<std::string, std::vector<float>> result;
        for (auto& input : inputs_)
        {
            for (auto& block : input.blocks)
                for (auto& term : block.terms)
                    if (term.update_phase == HistoryUpdatePhase::BeforeInference)
                        term.update(term.sample(env_));

            auto& values = result[input.name];
            for (const auto& block : input.blocks) append_block(block, values);
        }
        return result;
    }

    void update_after_inference()
    {
        for (auto& input : inputs_)
            for (auto& block : input.blocks)
                for (auto& term : block.terms)
                    if (term.update_phase == HistoryUpdatePhase::AfterInference)
                        term.update(term.sample(env_));
    }

private:
    struct BlockCfg
    {
        std::string layout;
        std::vector<ObservationTermCfg> terms;
    };

    struct InputCfg
    {
        std::string name;
        std::vector<BlockCfg> blocks;
    };

    static std::vector<float> read_vector(const YAML::Node& node)
    {
        return node && !node.IsNull() ? node.as<std::vector<float>>() : std::vector<float>{};
    }

    static void append_block(const BlockCfg& block, std::vector<float>& output)
    {
        if (block.layout == "frame_major")
        {
            const int frames = block.terms.front().history_length;
            for (int frame = 0; frame < frames; ++frame)
                for (const auto& term : block.terms)
                    append(term.frame(frame), output);
        }
        else
        {
            for (const auto& term : block.terms)
                for (int frame = 0; frame < term.history_length; ++frame)
                    append(term.frame(frame), output);
        }
    }

    static void append(const std::vector<float>& values, std::vector<float>& output)
    {
        output.insert(output.end(), values.begin(), values.end());
    }

    ManagerBasedRLEnv* env_;
    std::vector<InputCfg> inputs_;
};

} // namespace isaaclab
