#pragma once

#include "cnpy.h"
#include <filesystem>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace policy
{

class ReferenceData
{
public:
    explicit ReferenceData(const std::filesystem::path& manifest_path)
    {
        const auto cfg = YAML::LoadFile(manifest_path.string());
        sample_rate_hz = cfg["sample_rate_hz"].as<float>();
        joint_names = cfg["joint_names"].as<std::vector<std::string>>();
        arrays_ = cnpy::npz_load((manifest_path.parent_path() / cfg["data_file"].as<std::string>()).string());
    }

    const cnpy::NpyArray& array(const std::string& name) const
    {
        return arrays_.at(name);
    }

    std::vector<float> frame(const std::string& name, size_t index) const
    {
        const auto& value = array(name);
        size_t width = 1;
        for (size_t axis = 1; axis < value.shape.size(); ++axis) width *= value.shape[axis];
        const float* begin = value.data<float>() + index * width;
        return std::vector<float>(begin, begin + width);
    }

    size_t frames(const std::string& name) const
    {
        return array(name).shape.front();
    }

    float sample_rate_hz = 0.0f;
    std::vector<std::string> joint_names;

private:
    cnpy::npz_t arrays_;
};

} // namespace policy
