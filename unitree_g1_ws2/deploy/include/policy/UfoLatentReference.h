#pragma once

#include "cnpy.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace policy
{

// Runtime library for precomputed UFO task latents. Tracking entries contain
// one z per control step; goal/reward entries contain a single constant z.
class UfoLatentReference
{
public:
    UfoLatentReference(
        const std::filesystem::path& catalog_path,
        std::filesystem::path selection_path)
        : catalog_path_(catalog_path), selection_path_(std::move(selection_path))
    {
        const auto catalog = YAML::LoadFile(catalog_path_.string());
        default_id_ = catalog["default_id"].as<std::string>();
        sample_offset_steps_ = catalog["sample_offset_steps"]
            ? catalog["sample_offset_steps"].as<size_t>() : 0;
        for (const auto& node : catalog["entries"])
        {
            Entry entry;
            entry.id = node["id"].as<std::string>();
            entry.function = node["function"].as<std::string>();
            entry.label = node["label"].as<std::string>();
            entry.playback = node["playback"].as<std::string>();
            entry.sample_rate_hz = node["sample_rate_hz"]
                ? node["sample_rate_hz"].as<float>() : 50.0f;
            entry.duration_s = node["duration_s"]
                ? node["duration_s"].as<float>() : 0.0f;
            auto latent_file = std::filesystem::path(node["latent_file"].as<std::string>());
            entry.latent_file = latent_file.is_absolute()
                ? latent_file : catalog_path_.parent_path() / latent_file;
            entries_.emplace(entry.id, std::move(entry));
        }
        if (!entries_.count(default_id_))
            throw std::runtime_error("UFO latent catalog default_id is not an entry: " + default_id_);
        load(default_id_);
    }

    void select_on_entry(const std::string& id)
    {
        load(id);
        update_selection_stamp();
    }

    bool refresh_from_file()
    {
        if (!selection_file_changed()) return false;
        const std::string requested = requested_from_file();
        load(requested);
        update_selection_stamp();
        return true;
    }

    void update(size_t step)
    {
        elapsed_steps_ = step;
        current_frame_ = active_->playback == "sequence"
            ? std::min(step + sample_offset_steps_, frame_count_ - 1)
            : 0;
    }

    std::vector<float> latent() const
    {
        const float* begin = latents_.data() + current_frame_ * latent_dim_;
        return std::vector<float>(begin, begin + latent_dim_);
    }

    bool finished() const { return active_->playback == "sequence" && elapsed_steps_ + 1 >= duration_steps_; }
    size_t frame_count() const { return frame_count_; }
    size_t current_frame() const { return current_frame_; }
    const std::string& active_id() const { return active_->id; }
    const std::string& active_function() const { return active_->function; }
    const std::string& active_label() const { return active_->label; }

    void select(const std::string& id)
    {
        if (!entries_.count(id))
            throw std::runtime_error("Unknown UFO latent id: " + id);
        load(id);
    }

private:
    struct Entry
    {
        std::string id;
        std::string function;
        std::string label;
        std::string playback;
        std::filesystem::path latent_file;
        float sample_rate_hz = 50.0f;
        float duration_s = 0.0f;
    };

    bool selection_file_changed() const
    {
        try
        {
            const bool exists = std::filesystem::exists(selection_path_);
            if (!selection_stamp_initialized_) return true;
            if (exists != selection_file_exists_) return true;
            return exists
                && std::filesystem::last_write_time(selection_path_) != selection_write_time_;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    void update_selection_stamp()
    {
        selection_stamp_initialized_ = true;
        try
        {
            selection_file_exists_ = std::filesystem::exists(selection_path_);
            if (selection_file_exists_)
                selection_write_time_ = std::filesystem::last_write_time(selection_path_);
        }
        catch (const std::exception&)
        {
            selection_file_exists_ = false;
        }
    }

    std::string requested_from_file() const
    {
        std::string requested = default_id_;
        try
        {
            if (std::filesystem::exists(selection_path_))
            {
                const auto selection = YAML::LoadFile(selection_path_.string());
                if (selection["id"]) requested = selection["id"].as<std::string>();
            }
        }
        catch (const std::exception&)
        {
            requested = default_id_;
        }
        return entries_.count(requested) ? requested : default_id_;
    }

    void load(const std::string& id)
    {
        active_ = &entries_.at(id);
        const auto arrays = cnpy::npz_load(active_->latent_file.string());
        const auto found = arrays.find("z");
        if (found == arrays.end())
            throw std::runtime_error("UFO latent file has no z array: " + active_->latent_file.string());
        const auto& z = found->second;
        if (z.word_size != sizeof(float) || z.shape.size() != 2 || z.shape[1] != 256 || z.shape[0] == 0)
            throw std::runtime_error("UFO z must be a non-empty float32 [N, 256] array: " + active_->latent_file.string());
        frame_count_ = z.shape[0];
        latent_dim_ = z.shape[1];
        const float* values = z.data<float>();
        latents_.assign(values, values + frame_count_ * latent_dim_);
        current_frame_ = 0;
        elapsed_steps_ = 0;
        duration_steps_ = active_->playback == "sequence"
            ? std::max<size_t>(1, frame_count_ > sample_offset_steps_
                ? frame_count_ - sample_offset_steps_ : 1)
            : std::max<size_t>(1, static_cast<size_t>(std::lround(
                active_->duration_s * active_->sample_rate_hz)));
    }

    std::filesystem::path catalog_path_;
    std::filesystem::path selection_path_;
    std::unordered_map<std::string, Entry> entries_;
    const Entry* active_ = nullptr;
    bool selection_stamp_initialized_ = false;
    bool selection_file_exists_ = false;
    std::filesystem::file_time_type selection_write_time_{};
    std::string default_id_;
    std::vector<float> latents_;
    size_t sample_offset_steps_ = 0;
    size_t frame_count_ = 0;
    size_t latent_dim_ = 0;
    size_t current_frame_ = 0;
    size_t elapsed_steps_ = 0;
    size_t duration_steps_ = 1;
};

} // namespace policy