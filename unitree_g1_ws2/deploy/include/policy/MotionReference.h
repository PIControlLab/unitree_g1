#pragma once

#include "policy/ReferenceData.h"
#include <Eigen/Geometry>
#include <algorithm>

namespace policy
{

class MotionReference
{
public:
    explicit MotionReference(const std::filesystem::path& manifest)
        : data_(manifest)
    {
        duration = (data_.frames("joint_pos") - 1) / data_.sample_rate_hz;
        update(0.0f);
    }

    void update(float time)
    {
        const float frame = std::clamp(time * data_.sample_rate_hz, 0.0f,
            static_cast<float>(data_.frames("joint_pos") - 1));
        lower_ = static_cast<size_t>(frame);
        upper_ = std::min(lower_ + 1, data_.frames("joint_pos") - 1);
        blend_ = frame - static_cast<float>(lower_);
    }

    Eigen::VectorXf joint_pos() const { return interpolate_vector("joint_pos"); }
    Eigen::VectorXf joint_vel() const { return interpolate_vector("joint_vel"); }

    Eigen::Quaternionf root_quaternion() const
    {
        const auto lower = data_.frame("root_quat_wxyz", lower_);
        const auto upper = data_.frame("root_quat_wxyz", upper_);
        const Eigen::Quaternionf q0(lower[0], lower[1], lower[2], lower[3]);
        const Eigen::Quaternionf q1(upper[0], upper[1], upper[2], upper[3]);
        return q0.slerp(blend_, q1);
    }

    const std::vector<std::string>& joint_names() const { return data_.joint_names; }

    float duration;

private:
    Eigen::VectorXf interpolate_vector(const std::string& name) const
    {
        const auto lower = data_.frame(name, lower_);
        const auto upper = data_.frame(name, upper_);
        Eigen::Map<const Eigen::VectorXf> a(lower.data(), lower.size());
        Eigen::Map<const Eigen::VectorXf> b(upper.data(), upper.size());
        return a * (1.0f - blend_) + b * blend_;
    }

    ReferenceData data_;
    size_t lower_ = 0;
    size_t upper_ = 0;
    float blend_ = 0.0f;
};

} // namespace policy
