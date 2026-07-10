#pragma once

#include "FSM/State_RLBase.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class State_Mimic : public FSMState
{
public:
    State_Mimic(int state_mode, std::string state_string);

    void enter();
    void run();

    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
    }

    class MotionLoader_;
    static std::shared_ptr<MotionLoader_> motion; // for obs computation

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::shared_ptr<MotionLoader_> motion_;
    std::string _policy_onnx_path;

    std::thread policy_thread;
    bool policy_thread_running = false;
    std::array<float, 2> time_range_;
};

class State_Mimic::MotionLoader_
{
public:
    MotionLoader_(std::string motion_file, float fps)
    : dt(1.0f / fps)
    {
        auto data = isaaclab::load_csv(motion_file);

        num_frames = static_cast<int>(data.size());
        duration = num_frames * dt;

        // 29DoF CSV -> 23DoF deploy order
        // 必须与 deploy.yaml 的 joint_ids_map 保持一致
        keep_joint_indices = {
            0, 6, 12,
            1, 7, 15, 22,
            2, 8, 16, 23,
            3, 9, 17, 24,
            4, 10, 18, 25,
            5, 11, 19, 26
        };

        for (int i = 0; i < num_frames; ++i)
        {
            root_positions.push_back(Eigen::VectorXf::Map(data[i].data(), 3));
            root_quaternions.push_back(
                Eigen::Quaternionf(data[i][6], data[i][3], data[i][4], data[i][5])
            );

            Eigen::VectorXf joint_pos(23);
            for (int j = 0; j < 23; ++j) {
                joint_pos[j] = data[i][7 + keep_joint_indices[j]];
            }
            dof_positions.push_back(joint_pos);
        }

        dof_velocities = _compute_raw_derivative(dof_positions);
        update(0.0f);
    }

    void update(float time)
    {
        float phase = std::clamp(time / duration, 0.0f, 1.0f);
        float frame_f = phase * (num_frames - 1);

        index_0_ = static_cast<int>(std::floor(frame_f));
        index_1_ = std::min(index_0_ + 1, num_frames - 1);
        blend_ = frame_f - static_cast<float>(index_0_);
    }

    void reset(const isaaclab::ArticulationData& data, float t = 0.0f)
    {
        update(t);
        auto init_to_anchor = isaaclab::yawQuaternion(this->root_quaternion()).toRotationMatrix();
        auto world_to_anchor = isaaclab::yawQuaternion(data.root_quat_w).toRotationMatrix();
        world_to_init_ = world_to_anchor * init_to_anchor.transpose();
    }

    Eigen::VectorXf joint_pos()
    {
        return dof_positions[index_0_] * (1.0f - blend_) + dof_positions[index_1_] * blend_;
    }

    Eigen::VectorXf root_position()
    {
        return root_positions[index_0_] * (1.0f - blend_) + root_positions[index_1_] * blend_;
    }

    Eigen::VectorXf joint_vel()
    {
        return dof_velocities[index_0_] * (1.0f - blend_) + dof_velocities[index_1_] * blend_;
    }

    Eigen::Quaternionf root_quaternion()
    {
        return root_quaternions[index_0_].slerp(blend_, root_quaternions[index_1_]);
    }

    float dt;
    int num_frames;
    float duration;

    std::vector<int> keep_joint_indices;
    std::vector<Eigen::VectorXf> root_positions;
    std::vector<Eigen::Quaternionf> root_quaternions;
    std::vector<Eigen::VectorXf> dof_positions;
    std::vector<Eigen::VectorXf> dof_velocities;

    Eigen::Matrix3f world_to_init_;

private:
    int index_0_ = 0;
    int index_1_ = 0;
    float blend_ = 0.0f;

    std::vector<Eigen::VectorXf> _compute_raw_derivative(const std::vector<Eigen::VectorXf>& data)
    {
        std::vector<Eigen::VectorXf> derivative;
        derivative.reserve(data.size());

        for (int i = 0; i < static_cast<int>(data.size()) - 1; ++i) {
            derivative.push_back((data[i + 1] - data[i]) / dt);
        }

        if (!derivative.empty()) {
            derivative.push_back(derivative.back());
        } else {
            derivative.push_back(Eigen::VectorXf::Zero(23));
        }

        return derivative;
    }
};

REGISTER_FSM(State_Mimic)