#pragma once

#include "FSMState.h"
#include "common/read_traj.h"
#include "common/mathTools.h"
#include <onnxruntime_cxx_api.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <cmath>

#define NUM_DOF 29

using namespace ArmatureConstants;

class State_WBC : public FSMState
{
public:
    State_WBC(int state, std::string state_string = "WBC");
    ~State_WBC() = default;
    void enter() override;
    void run() override;
    void exit() override;

private:
    void loadPolicy();
    void observations_compute();
    void action_compute();

    Ort::Env _env;
    Ort::SessionOptions _session_options;
    std::unique_ptr<Ort::Session> _session;
    Ort::AllocatorWithDefaultOptions _allocator;

    const std::vector<const char*> _input_names = {"obs"};
    const std::vector<const char*> _output_names = {"actions"};

    std::vector<int64_t> _input_shape;
    std::vector<int64_t> _output_shape;
    int64_t _obs_size_;
    int64_t _action_size_;

    float _targetPos_rl[29];
    float _last_targetPos_rl[29];

    const float clip_observations = 100.0;
    const float clip_actions = 100.0;
    const float action_scale = 0.25;

    float scale_dof_pos = 1.0;
    float scale_dof_vel = 1.0;
    float _joint_q[29];

    std::vector<float> _action;
    std::vector<float> _observation;

    // motion data
    std::vector<float> _body_ang_vel_w;
    std::vector<uint32_t> _body_ang_vel_w_shape;
    std::vector<float> _body_lin_vel_w;
    std::vector<uint32_t> _body_lin_vel_w_shape;
    std::vector<float> _body_pos_w;
    std::vector<uint32_t> _body_pos_w_shape;
    std::vector<float> _body_quat_w;
    std::vector<uint32_t> _body_quat_w_shape;
    std::vector<int64_t> _fps;
    std::vector<uint32_t> _fps_shape;
    std::vector<float> _joint_pos;
    std::vector<uint32_t> _joint_pos_shape;
    std::vector<float> _joint_vel;
    std::vector<uint32_t> _joint_vel_shape;

    bool _bin_data_loaded;
    std::string _model_path;
    std::string _folder_path;

    const int _mimic_obs_predictive_horizon = 1;
    const int _frame_interval = 5;
    const int _actor_state_history_length = 4;
    const int _robot_state_dim = 93;
    const int _reference_dim = 67;
    unsigned int _refer_idx = 0;
    unsigned int _last_refer_idx = 0;
    int _step_counter = 0;   // 降频 1000Hz→50Hz
    int _recovery_idx = -1;  // -1=播放中, >=0=恢复中
    static const int RECOVERY_FRAMES = 75; // 1.5s 平滑过渡
    const int _anchor_idx = 0;
    const int _waist_yrp_idx[3] = {12, 13, 14};

    int _start_refer_idx = 0;
    int _end_refer_idx = -1;
    int _motion_frame_count = 0;
    const std::vector<float> _gravity_vec = {0.0f, 0.0f, -1.0f};
    float _anchor_terminate_thresh = 0.5f;
    bool _terminate_flag = false;
    bool _motion_done = false;  // 动作播完立刻切 Velocity

    std::vector<float> _robot_state_obs_buf =
        std::vector<float>(_robot_state_dim * _actor_state_history_length, 0.0f);

    // 29DoF 模型训练时的零位参考（observations_compute 中做差使用，不可修改）
    const float _default_dof_pos[NUM_DOF] = {
        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
        0.0, 0.0, 0.0,
        0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
        0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    };

    // 23DoF 站立姿态（对齐 Velocity deploy.yaml 的 default_joint_pos/offset）
    // recovery 和 enter 初始化使用此姿态，确保切回 Velocity 时姿态兼容
    const float _stand_pos[NUM_DOF] = {
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,        // L leg
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,        // R leg
        0.0, 0.0, 0.0,                           // waist (13,14 dummy)
        0.3, 0.25, 0.0, 0.97, 0.15, 0.0, 0.0,  // L arm: shoulder_pitch=0.3 (对齐Velocity)
        0.3, -0.25, 0.0, 0.97, -0.15, 0.0, 0.0, // R arm: shoulder_pitch=0.3 (对齐Velocity)
    };

    const int dof_mapping[NUM_DOF] = {
        0,  6, 12,
        1,  7, 13,
        2,  8, 14,
        3,  9, 15, 22,
        4, 10, 16, 23,
        5, 11, 17, 24,
        18, 25,
        19, 26,
        20, 27,
        21, 28,
    };

    const double dof_Kps[NUM_DOF] = {
        STIFFNESS_7520_22, STIFFNESS_7520_22, STIFFNESS_7520_14, STIFFNESS_7520_22, 2.0*STIFFNESS_5020, 2.0*STIFFNESS_5020,
        STIFFNESS_7520_22, STIFFNESS_7520_22, STIFFNESS_7520_14, STIFFNESS_7520_22, 2.0*STIFFNESS_5020, 2.0*STIFFNESS_5020,
        STIFFNESS_7520_14, 2.0*STIFFNESS_5020, 2.0*STIFFNESS_5020,
        STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5010_16, STIFFNESS_5010_16,
        STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5010_16, STIFFNESS_5010_16,
    };

    const double dof_Kds[NUM_DOF] = {
        DAMPING_7520_22, DAMPING_7520_22, DAMPING_7520_14, DAMPING_7520_22, 2.0*DAMPING_5020, 2.0*DAMPING_5020,
        DAMPING_7520_22, DAMPING_7520_22, DAMPING_7520_14, DAMPING_7520_22, 2.0*DAMPING_5020, 2.0*DAMPING_5020,
        DAMPING_7520_14, 2.0*DAMPING_5020, 2.0*DAMPING_5020,
        DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5010_16, DAMPING_5010_16,
        DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5010_16, DAMPING_5010_16,
    };
};

REGISTER_FSM(State_WBC)
