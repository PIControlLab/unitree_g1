#include "State_Mimic.h"



#include "isaaclab/envs/mdp/actions/joint_actions.h"

#include "isaaclab/envs/mdp/observations/observations.h"

#include "unitree_articulation.h"



#include <spdlog/spdlog.h>



static Eigen::Quaternionf init_quat = Eigen::Quaternionf::Identity();

std::shared_ptr<State_Mimic::MotionLoader_> State_Mimic::motion = nullptr;



// ================================

// 23DoF torso / anchor orientation

// ================================

// 29DoF 官方版使用 root + waist_yaw + waist_roll + waist_pitch。

// 23DoF 中 waist_roll / waist_pitch 已删除，因此这里严格只保留 waist_yaw。

// 同时 real/ref 两侧使用完全一致的定义，避免静止阶段出现持续偏航补偿。



namespace

{



constexpr int kWaistYawIdx = 12;   // in 23DoF deploy order

constexpr int kWaistYawMotor = 12; // in lowstate motor_state order / deploy mapping



Eigen::Quaternionf torso_quat_w(isaaclab::ManagerBasedRLEnv* env)

{

    using G1Type = unitree::BaseArticulation<LowState_t::SharedPtr>;

    G1Type* robot = dynamic_cast<G1Type*>(env->robot.get());



    const auto root_quat = env->robot->data.root_quat_w;

    const auto& motors = robot->lowstate->msg_.motor_state();



    Eigen::Quaternionf torso_quat =

        root_quat *

        Eigen::AngleAxisf(motors[kWaistYawMotor].q(), Eigen::Vector3f::UnitZ());



    return torso_quat;

}



Eigen::Quaternionf anchor_quat_w(const std::shared_ptr<State_Mimic::MotionLoader_>& loader)

{

    const auto root_quat = loader->root_quaternion();

    const auto joint_pos = loader->joint_pos();



    Eigen::Quaternionf torso_quat =

        root_quat *

        Eigen::AngleAxisf(joint_pos[kWaistYawIdx], Eigen::Vector3f::UnitZ());



    return torso_quat;

}



} // namespace



namespace isaaclab

{

namespace mdp

{



REGISTER_OBSERVATION(motion_joint_pos)

{

    auto& loader = State_Mimic::motion;



    auto data_dfs = loader->joint_pos();

    return std::vector<float>(data_dfs.data(), data_dfs.data() + data_dfs.size());

}



REGISTER_OBSERVATION(motion_joint_vel)

{

    auto& loader = State_Mimic::motion;



    auto data_dfs = loader->joint_vel();

    return std::vector<float>(data_dfs.data(), data_dfs.data() + data_dfs.size());

}



REGISTER_OBSERVATION(motion_command)

{

    auto& loader = State_Mimic::motion;



    auto pos_dfs = loader->joint_pos();

    auto vel_dfs = loader->joint_vel();



    std::vector<float> data;

    data.reserve(pos_dfs.size() + vel_dfs.size());

    data.insert(data.end(), pos_dfs.data(), pos_dfs.data() + pos_dfs.size());

    data.insert(data.end(), vel_dfs.data(), vel_dfs.data() + vel_dfs.size());

    return data;

}


REGISTER_OBSERVATION(motion_anchor_ori_b)

{

    // real/ref 两侧都使用同一套 torso(anchor) 定义

    auto real_quat_w = torso_quat_w(env);

    auto ref_quat_w = anchor_quat_w(State_Mimic::motion);



    auto rot_ = (init_quat * ref_quat_w).conjugate() * real_quat_w;

    auto rot = rot_.toRotationMatrix().transpose();



    Eigen::Matrix<float, 6, 1> data;

    data << rot(0, 0), rot(0, 1),

            rot(1, 0), rot(1, 1),

            rot(2, 0), rot(2, 1);



    return std::vector<float>(data.data(), data.data() + data.size());

}



} // namespace mdp

} // namespace isaaclab



State_Mimic::State_Mimic(int state_mode, std::string state_string)

: FSMState(state_mode, state_string)

{

    auto cfg = param::config["FSM"][state_string];

    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());



    auto articulation =

        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate);



    std::filesystem::path motion_file = cfg["motion_file"].as<std::string>();

    if (!motion_file.is_absolute()) {

        motion_file = param::proj_dir / motion_file;

    }



    motion_ = std::make_shared<MotionLoader_>(motion_file.string(), cfg["fps"].as<float>());

    spdlog::info(

        "[23DOF] Loaded motion file '{}' with duration {:.2f}s",

        motion_file.stem().string(),

        motion_->duration

    );



    motion = motion_;



    if (cfg["time_start"]) {

        float time_start = cfg["time_start"].as<float>();

        time_range_[0] = std::clamp(time_start, 0.0f, motion_->duration);

    } else {

        time_range_[0] = 0.0f;

    }



    if (cfg["time_end"]) {

        float time_end = cfg["time_end"].as<float>();

        time_range_[1] = std::clamp(time_end, 0.0f, motion_->duration);

    } else {

        time_range_[1] = motion_->duration;

    }



    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(

        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),

        articulation

    );

    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");
    _policy_onnx_path = (policy_dir / "exported" / "policy.onnx").string();

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]() -> bool {
                float proj_g = env->robot->data.projected_gravity_b[2];
                float angle = std::fabs(std::acos(-proj_g));
                if (angle > 1.0) {
                    spdlog::critical("[bad_orientation] Mimic → Passive");
                    spdlog::critical("  projected_gravity_z = {:.4f}", proj_g);
                    spdlog::critical("  tilt_angle = {:.4f} rad ({:.1f} deg), limit = 1.0 rad",
                                     angle, angle * 180.0 / std::acos(-1.0));
                    spdlog::critical("  robot orientation is too tilted, entering Passive for safety!");
                    return true;
                }
                return false;
            },
            FSMStringMap.right.at("Passive")
        )
    );

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]() -> bool { return (env->episode_length * env->step_dt) > time_range_[1]; },
            FSMStringMap.right.at("Velocity")

        )

    );

}



void State_Mimic::enter()
{
    // 重建 ONNX session，清零 GRU/LSTM 隐状态
    env->alg = std::make_unique<isaaclab::OrtRunner>(_policy_onnx_path);
    spdlog::info("[Mimic] ONNX session recreated, GRU state reset");

    // set gain — 遍历全部 29 个电机，mode = 0x01 (MIT PD 控制)
    // 与 g1_merger / Velocity / WBC 保持一致
    static constexpr int TOTAL_MOTORS = 29;
    static constexpr float DEFAULT_KP = 40.0f;
    static constexpr float DEFAULT_KD = 1.0f;
    for (int i = 0; i < TOTAL_MOTORS; ++i)
    {
        float kp = (i < static_cast<int>(env->robot->data.joint_stiffness.size()))
                   ? env->robot->data.joint_stiffness[i] : DEFAULT_KP;
        float kd = (i < static_cast<int>(env->robot->data.joint_damping.size()))
                   ? env->robot->data.joint_damping[i] : DEFAULT_KD;
        lowcmd->msg_.motor_cmd()[i].mode() = 0x01;
        lowcmd->msg_.motor_cmd()[i].kp() = kp;
        lowcmd->msg_.motor_cmd()[i].kd() = kd;
        lowcmd->msg_.motor_cmd()[i].dq() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].tau() = 0.0f;
        lowcmd->msg_.motor_cmd()[i].q() = lowstate->msg_.motor_state()[i].q();
    }
    // 清零 23DoF 假关节
    static const int dummy_23dof[] = {13, 14, 20, 21, 27, 28};
    for (int idx : dummy_23dof) {
        lowcmd->msg_.motor_cmd()[idx].kp() = 0;
        lowcmd->msg_.motor_cmd()[idx].kd() = 0.5f;
        lowcmd->msg_.motor_cmd()[idx].tau() = 0;
    }



    motion = motion_;

    env->reset();



    policy_thread_running = true;

    policy_thread = std::thread([this] {

        using clock = std::chrono::high_resolution_clock;

        const std::chrono::duration<double> desiredDuration(env->step_dt);

        const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);



        auto sleepTill = clock::now() + dt;



        // 先把 motion 对齐到起始时刻

        motion->reset(env->robot->data, time_range_[0]);



        // 关键修正：

        // init_quat 两侧都使用 anchor/torso 的定义，而不是 ref 用 root、real 用 torso

        auto ref_yaw = isaaclab::yawQuaternion(anchor_quat_w(motion)).toRotationMatrix();

        auto robot_yaw = isaaclab::yawQuaternion(torso_quat_w(env.get())).toRotationMatrix();

        init_quat = robot_yaw * ref_yaw.transpose();



        // 不再第二次 env->reset()，避免 init_quat 与当前状态脱节

        while (policy_thread_running)

        {

            env->robot->update();

            motion->update(env->episode_length * env->step_dt + time_range_[0]);

            env->step();



            std::this_thread::sleep_until(sleepTill);

            sleepTill += dt;

        }

    });

}



void State_Mimic::run()

{

    auto action = env->action_manager->processed_actions();



    // 与官方部署逻辑一致：action[i] 写回 joint_ids_map[i]

    for (int i = 0; i < env->robot->data.joint_ids_map.size(); ++i) {

        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];

    }

}