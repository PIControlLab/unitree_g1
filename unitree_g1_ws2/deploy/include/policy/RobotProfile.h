#pragma once

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace policy
{

struct JointSpec
{
    std::string name;
    int motor_id;
    bool active;
};

struct JointLayout
{
    std::vector<std::string> names;
    std::vector<int> motor_ids;
    std::vector<bool> active;
};

class RobotProfile
{
public:
    explicit RobotProfile(const std::filesystem::path& path)
    {
        const auto cfg = YAML::LoadFile(path.string());
        motor_count = cfg["motor_count"].as<int>();
        inactive_stiffness = cfg["inactive_joint_control"]["stiffness"].as<float>();
        inactive_damping = cfg["inactive_joint_control"]["damping"].as<float>();

        for (const auto& joint : cfg["joints"])
        {
            JointSpec spec{
                joint["name"].as<std::string>(),
                joint["motor_id"].as<int>(),
                joint["active"].as<bool>()
            };
            joints_.emplace(spec.name, spec);
            motor_joints_.push_back(spec);
        }
    }

    JointLayout layout(const std::vector<std::string>& names) const
    {
        JointLayout result;
        result.names = names;
        result.motor_ids.reserve(names.size());
        result.active.reserve(names.size());
        for (const auto& name : names)
        {
            const auto& joint = joints_.at(name);
            result.motor_ids.push_back(joint.motor_id);
            result.active.push_back(joint.active);
        }
        return result;
    }

    const std::vector<JointSpec>& motor_joints() const { return motor_joints_; }

    int motor_count = 0;
    float inactive_stiffness = 0.0f;
    float inactive_damping = 0.0f;

private:
    std::unordered_map<std::string, JointSpec> joints_;
    std::vector<JointSpec> motor_joints_;
};

} // namespace policy
