#include "FSM/State_WBC.h"
#include "common/read_traj.h"
#include "param.h"
#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

State_WBC::State_WBC(int state, std::string state_string)
    : FSMState(state, state_string)
{
    // 从 config.yaml 读取 WBC 配置
    auto cfg = param::config["FSM"]["WBC"];
    _model_path = (param::proj_dir / cfg["policy_path"].as<std::string>()).string();
    _folder_path = (param::proj_dir / cfg["motion_path"].as<std::string>()).string();
    _anchor_terminate_thresh = cfg["safe_projgravity_threshold"]
        ? cfg["safe_projgravity_threshold"].as<float>() : 0.5f;
    _start_refer_idx = cfg["start_idx"] ? cfg["start_idx"].as<int>() : 0;
    _end_refer_idx = cfg["end_idx"] ? cfg["end_idx"].as<int>() : -1;

    spdlog::info("[State_WBC] Model path: {}", _model_path);
    spdlog::info("[State_WBC] Motion path: {}", _folder_path);
    spdlog::info("[State_WBC] Frame range: {} -> {}", _start_refer_idx, _end_refer_idx);

    // 加载二进制 motion data
    _bin_data_loaded = BinaryArrayReader::readBinFilesFromFolder(
        _folder_path,
        _body_ang_vel_w, _body_ang_vel_w_shape,
        _body_lin_vel_w, _body_lin_vel_w_shape,
        _body_pos_w, _body_pos_w_shape,
        _body_quat_w, _body_quat_w_shape,
        _fps, _fps_shape,
        _joint_pos, _joint_pos_shape,
        _joint_vel, _joint_vel_shape
    );
    if (!_bin_data_loaded || _joint_pos_shape.empty()) {
        spdlog::error("[State_WBC] Failed to load motion data from: {}", _folder_path);
        throw std::runtime_error("WBC: Cannot load motion data");
    }
    _motion_frame_count = _joint_pos_shape[0];
    spdlog::info("[State_WBC] Loaded {} motion frames", _motion_frame_count);

    loadPolicy();

    // 动作播完立刻切 Velocity，不做 recovery（recovery 站不稳）
    registered_checks.insert(registered_checks.begin(),
        std::make_pair(
            [this]() { return _motion_done; },
            FSMStringMap.right.at("Velocity")
        )
    );
}

void State_WBC::loadPolicy()
{
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    _session = std::make_unique<Ort::Session>(_env, _model_path.c_str(), _session_options);

    Ort::TypeInfo input_type = _session->GetInputTypeInfo(0);
    auto input_shapes = input_type.GetTensorTypeAndShapeInfo().GetShape();
    Ort::TypeInfo output_type = _session->GetOutputTypeInfo(0);
    auto output_shapes = output_type.GetTensorTypeAndShapeInfo().GetShape();

    _obs_size_ = input_shapes[1];
    _action_size_ = output_shapes[1];
    _action = std::vector<float>(_action_size_, 0.0f);
    spdlog::info("[State_WBC] Policy loaded, obs_size={}, action_size={}", _obs_size_, _action_size_);
}

void State_WBC::observations_compute()
{
    // --- IMU / base quaternion ---
    std::vector<float> base_quat = {
        lowstate->msg_.imu_state().quaternion()[0],
        lowstate->msg_.imu_state().quaternion()[1],
        lowstate->msg_.imu_state().quaternion()[2],
        lowstate->msg_.imu_state().quaternion()[3],
    };
    std::vector<float> projected_gravity = QuatRotateInverse(base_quat, _gravity_vec);
    auto obs_projected_gravity = projected_gravity;

    // --- waist RPY ---
    std::vector<float> waist_yrp(3);
    for (int i = 0; i < 3; i++) {
        waist_yrp[i] = lowstate->msg_.motor_state()[_waist_yrp_idx[i]].q();
    }

    Eigen::Matrix3f R_b2w = rotz(waist_yrp[0]) * rotx(waist_yrp[1]) * roty(waist_yrp[2]);
    Eigen::Matrix3f R_base = matrix_from_quat(base_quat);
    Eigen::Matrix3f R_waist = R_base * R_b2w;
    std::vector<float> waist_quat = quat_from_matrix(R_waist);

    // --- dof pos/vel ---
    std::vector<float> dof_pos_vec;
    dof_pos_vec.reserve(NUM_DOF);
    for (int i = 0; i < NUM_DOF; ++i) {
        dof_pos_vec.push_back(
            lowstate->msg_.motor_state()[dof_mapping[i]].q() - _default_dof_pos[dof_mapping[i]]);
    }

    std::vector<float> dof_vel_vec;
    dof_vel_vec.reserve(NUM_DOF);
    for (int i = 0; i < NUM_DOF; ++i) {
        dof_vel_vec.push_back(lowstate->msg_.motor_state()[dof_mapping[i]].dq());
    }

    // 23DoF 假关节观察替换: 将 dummy 关节的 pos/vel 设为 0,
    // 让 ONNX 模型(29DoF训练)看到正确的"虚拟"关节状态
    // dummy motor: 13(waist_roll), 14(waist_pitch),
    //   20(l_wrist_pitch), 21(l_wrist_yaw), 27(r_wrist_pitch), 28(r_wrist_yaw)
    // dof_mapping 反向找: 5→13, 8→14, 25→20, 27→21, 26→27, 28→28
    static const int dummy_policy_idx[] = {5, 8, 25, 26, 27, 28};
    for (int idx : dummy_policy_idx) {
        dof_pos_vec[idx] = 0.0f;
        dof_vel_vec[idx] = 0.0f;
    }

    // --- body angular velocity (gyro) ---
    auto body_ang_vel = std::vector<float>({
        static_cast<float>(lowstate->msg_.imu_state().gyroscope()[0]),
        static_cast<float>(lowstate->msg_.imu_state().gyroscope()[1]),
        static_cast<float>(lowstate->msg_.imu_state().gyroscope()[2]),
    });

    for (int i = 0; i < NUM_DOF; i++) {
        dof_pos_vec[i] *= scale_dof_pos;
        dof_vel_vec[i] *= scale_dof_vel;
    }

    // --- build robot state buffer ---
    std::vector<float> current_robot_state;
    current_robot_state.reserve(3 + 3 + dof_pos_vec.size() + dof_vel_vec.size() + _action.size());
    current_robot_state.insert(current_robot_state.end(), body_ang_vel.begin(), body_ang_vel.end());
    current_robot_state.insert(current_robot_state.end(), obs_projected_gravity.begin(), obs_projected_gravity.end());
    current_robot_state.insert(current_robot_state.end(), dof_pos_vec.begin(), dof_pos_vec.end());
    current_robot_state.insert(current_robot_state.end(), dof_vel_vec.begin(), dof_vel_vec.end());
    current_robot_state.insert(current_robot_state.end(), _action.begin(), _action.end());

    _robot_state_obs_buf.erase(_robot_state_obs_buf.begin(),
                                _robot_state_obs_buf.begin() + _robot_state_dim);
    _robot_state_obs_buf.insert(_robot_state_obs_buf.end(),
                                current_robot_state.begin(), current_robot_state.end());

    // --- motion trajectory helpers ---
    auto get_pos = [this](int frame_idx, int link_idx) -> std::vector<float> {
        int num_links = _body_pos_w_shape[1];
        int base = frame_idx * num_links * 3 + link_idx * 3;
        return {_body_pos_w[base], _body_pos_w[base + 1], _body_pos_w[base + 2]};
    };
    auto get_quat = [this](int frame_idx, int link_idx) -> std::vector<float> {
        int num_links = _body_quat_w_shape[1];
        int base = frame_idx * num_links * 4 + link_idx * 4;
        return {_body_quat_w[base], _body_quat_w[base + 1], _body_quat_w[base + 2], _body_quat_w[base + 3]};
    };
    auto get_dof_pos = [this](int frame_idx) -> std::vector<float> {
        int num_dofs = _joint_pos_shape[1];
        int base = frame_idx * num_dofs;
        std::vector<float> dof_pos_(num_dofs);
        for (int i = 0; i < num_dofs; ++i) dof_pos_[i] = _joint_pos[base + i];
        return dof_pos_;
    };
    auto get_dof_vel = [this](int frame_idx) -> std::vector<float> {
        int num_dofs = _joint_vel_shape[1];
        int base = frame_idx * num_dofs;
        std::vector<float> dof_vel_(num_dofs);
        for (int i = 0; i < num_dofs; ++i) dof_vel_[i] = _joint_vel[base + i];
        return dof_vel_;
    };

    // --- reference trajectory ---
    std::vector<float> cur_refer_anchor_pos, cur_refer_dof_pos, cur_refer_dof_vel;
    std::vector<float> cur_refer_anchor_quat, ref_yaw_quat, ref_yaw_quat_conj;
    std::vector<float> yaw_quat_delta, aligned_cur_refer_anchor_quat;
    std::vector<float> motion_projected_gravity;
    std::vector<float> base_yaw_quat = yaw_quat(base_quat);
    std::vector<float> last_refer_anchor_pos, last_refer_anchor_quat;

    std::vector<float> tgt_anchor_ori_b_flat, tgt_anchor_pos_b_flat;
    std::vector<float> tgt_dof_pos_flat, tgt_dof_vel_flat;

    for (int i = 0; i < _mimic_obs_predictive_horizon; i++) {
        int idx = _refer_idx + i * _frame_interval;
        int last_idx = idx - _frame_interval;
        if (idx >= _end_refer_idx) idx = _end_refer_idx;
        else if (idx <= 1) idx = 1;
        if (last_idx >= _end_refer_idx) last_idx = _end_refer_idx;
        else if (last_idx <= 1) last_idx = 1;

        cur_refer_anchor_pos = get_pos(idx, _anchor_idx);
        cur_refer_dof_pos = get_dof_pos(idx);
        cur_refer_dof_vel = get_dof_vel(idx);
        cur_refer_anchor_quat = get_quat(idx, _anchor_idx);
        ref_yaw_quat = yaw_quat(cur_refer_anchor_quat);
        ref_yaw_quat_conj = quat_conjugate(ref_yaw_quat);
        yaw_quat_delta = quat_multiply(base_yaw_quat, ref_yaw_quat_conj);
        aligned_cur_refer_anchor_quat = quat_multiply(yaw_quat_delta, cur_refer_anchor_quat);

        last_refer_anchor_pos = get_pos(last_idx, _anchor_idx);
        last_refer_anchor_quat = get_quat(last_idx, _anchor_idx);
        if (i == 0)
            motion_projected_gravity = quat_apply_inverse(aligned_cur_refer_anchor_quat, _gravity_vec);

        auto [cur_target_pos, cur_target_quat] = subtract_frame_transforms(
            last_refer_anchor_pos, last_refer_anchor_quat,
            &cur_refer_anchor_pos, &cur_refer_anchor_quat);

        Eigen::Matrix3f mat = matrix_from_quat(cur_target_quat);
        std::vector<float> tgt_anchor_ori_b(6);
        tgt_anchor_ori_b[0] = mat(0,0); tgt_anchor_ori_b[1] = mat(0,1);
        tgt_anchor_ori_b[2] = mat(1,0); tgt_anchor_ori_b[3] = mat(1,1);
        tgt_anchor_ori_b[4] = mat(2,0); tgt_anchor_ori_b[5] = mat(2,1);

        tgt_dof_pos_flat.insert(tgt_dof_pos_flat.end(), cur_refer_dof_pos.begin(), cur_refer_dof_pos.end());
        tgt_dof_vel_flat.insert(tgt_dof_vel_flat.end(), cur_refer_dof_vel.begin(), cur_refer_dof_vel.end());
        tgt_anchor_pos_b_flat.insert(tgt_anchor_pos_b_flat.end(), cur_target_pos.begin(), cur_target_pos.end());
        tgt_anchor_ori_b_flat.insert(tgt_anchor_ori_b_flat.end(), tgt_anchor_ori_b.begin(), tgt_anchor_ori_b.end());
    }

    std::vector<float> mimic_obs;
    mimic_obs.insert(mimic_obs.end(), tgt_dof_pos_flat.begin(), tgt_dof_pos_flat.end());
    mimic_obs.insert(mimic_obs.end(), tgt_dof_vel_flat.begin(), tgt_dof_vel_flat.end());
    mimic_obs.insert(mimic_obs.end(), tgt_anchor_pos_b_flat.begin(), tgt_anchor_pos_b_flat.end());
    mimic_obs.insert(mimic_obs.end(), tgt_anchor_ori_b_flat.begin(), tgt_anchor_ori_b_flat.end());

    float anchor_proj_gravity_error = std::abs(motion_projected_gravity[2] - projected_gravity[2]);
    if (anchor_proj_gravity_error > _anchor_terminate_thresh) {
        _terminate_flag = true;
        static int warn_count = 0;
        if (++warn_count % 100 == 1)  // 每100次警告才打印一次
            spdlog::warn("[State_WBC] Anchor gravity error: {:.4f} > {:.2f}",
                         anchor_proj_gravity_error, _anchor_terminate_thresh);
    }

    _observation.clear();
    _observation.reserve(_robot_state_obs_buf.size() + mimic_obs.size());
    _observation.insert(_observation.end(), mimic_obs.begin(), mimic_obs.end());
    _observation.insert(_observation.end(), _robot_state_obs_buf.begin(), _robot_state_obs_buf.end());

    for (size_t i = 0; i < _observation.size(); i++) {
        _observation[i] = std::max(-clip_observations, std::min(_observation[i], clip_observations));
    }
}

void State_WBC::action_compute()
{
    try {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

        std::vector<Ort::Value> input_tensors;
        std::vector<int64_t> obs_shape = {1,
            _robot_state_dim * _actor_state_history_length
            + _reference_dim * _mimic_obs_predictive_horizon};

        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, _observation.data(), _observation.size(),
            obs_shape.data(), obs_shape.size()));

        auto output_tensors = _session->Run(
            Ort::RunOptions{nullptr}, _input_names.data(), input_tensors.data(),
            input_tensors.size(), _output_names.data(), 1);

        float* actions = output_tensors[0].GetTensorMutableData<float>();
        std::memcpy(_action.data(), actions, _action.size() * sizeof(float));

        for (size_t i = 0; i < _action.size(); i++) {
            _action[i] = std::max(-clip_actions, std::min(_action[i], clip_actions));
            float scaled = _action[i] * action_scale + _default_dof_pos[dof_mapping[i]];
            _joint_q[dof_mapping[i]] = scaled;
        }
    } catch (const Ort::Exception& e) {
        spdlog::error("[State_WBC] ONNX Runtime error: {}", e.what());
    } catch (const std::exception& e) {
        spdlog::error("[State_WBC] Exception: {}", e.what());
    }
}

void State_WBC::enter()
{
    _terminate_flag = false;
    _motion_done = false;
    _step_counter = 0;
    _recovery_idx = -1;
    _refer_idx = _start_refer_idx;
    _last_refer_idx = _refer_idx;

    if (_end_refer_idx < 0 || _end_refer_idx < _start_refer_idx) {
        if (_end_refer_idx < _start_refer_idx) {
            spdlog::warn("[State_WBC] end_idx < start_idx, using full motion");
        }
        _end_refer_idx = _motion_frame_count - 1;
    }

    for (int i = 0; i < NUM_DOF; i++) {
        lowcmd->msg_.motor_cmd()[i].mode() = 0x01;
        lowcmd->msg_.motor_cmd()[i].q() = lowstate->msg_.motor_state()[i].q();
        lowcmd->msg_.motor_cmd()[i].dq() = 0;
        lowcmd->msg_.motor_cmd()[i].tau() = 0;
        lowcmd->msg_.motor_cmd()[i].kp() = dof_Kps[i];
        lowcmd->msg_.motor_cmd()[i].kd() = dof_Kds[i];
        _targetPos_rl[i] = _stand_pos[i];
        _last_targetPos_rl[i] = lowstate->msg_.motor_state()[i].q();
        _joint_q[i] = _stand_pos[i];
    }
    // 清零 23DoF 假关节
    static const int dummy_23dof[] = {13, 14, 20, 21, 27, 28};
    for (int idx : dummy_23dof) {
        lowcmd->msg_.motor_cmd()[idx].kp() = 0;
        lowcmd->msg_.motor_cmd()[idx].kd() = 0.5f;  // 最小阻尼防漂移（motor 27 曾漂到 -50）
        lowcmd->msg_.motor_cmd()[idx].tau() = 0;
    }

    spdlog::info("[State_WBC] Entered, frames {}/{}", _start_refer_idx, _end_refer_idx);
}

void State_WBC::run()
{
    const int steps_per_frame = 20;
    static const int dummy_23dof[] = {13, 14, 20, 21, 27, 28};

    // 手动按 RB+X 则直接切 Velocity
    if (_recovery_idx < 0 && lowstate->joystick.RB.pressed &&
        lowstate->joystick.X.on_pressed) {
        spdlog::info("[State_WBC] RB+X pressed, switching to Velocity...");
        _motion_done = true;
    }

    if (_recovery_idx >= 0) {
        // ── 平滑过渡到站立（保留 recovery 代码，但不再由此路径触发）──
        if (++_step_counter >= steps_per_frame) {
            _step_counter = 0;
            _recovery_idx++;
        }
        float t = std::min(1.0f, (float)_recovery_idx / RECOVERY_FRAMES);

        for (int j = 0; j < NUM_DOF; j++) {
            float q = _targetPos_rl[j] + t * (_stand_pos[j] - _targetPos_rl[j]);
            lowcmd->msg_.motor_cmd()[j].mode() = 0x01;
            lowcmd->msg_.motor_cmd()[j].q() = q;
            lowcmd->msg_.motor_cmd()[j].dq() = 0;
            lowcmd->msg_.motor_cmd()[j].tau() = 0;
            lowcmd->msg_.motor_cmd()[j].kp() = dof_Kps[j];
            lowcmd->msg_.motor_cmd()[j].kd() = dof_Kds[j];
        }
        for (int idx : dummy_23dof) {
            lowcmd->msg_.motor_cmd()[idx].kp() = 0;
            lowcmd->msg_.motor_cmd()[idx].kd() = 0.5f;
            lowcmd->msg_.motor_cmd()[idx].tau() = 0;
        }
        return;
    }

    // ── 正常播放 ─────────────────────────────────
    if (++_step_counter >= steps_per_frame) {
        _step_counter = 0;
        if (_refer_idx < static_cast<unsigned int>(_end_refer_idx))
            _refer_idx++;
        else {
            // 动作播完，立刻切 Velocity，不做 recovery
            spdlog::info("[State_WBC] Motion finished at frame {}, switching to Velocity...", _refer_idx);
            _motion_done = true;
        }

        // ONNX 推理按 50Hz 运行（与 Velocity/Mimic 一致，模型按此频率训练）
        observations_compute();
        action_compute();

        // 安全检测：投影重力误差过大时触发 recovery
        if (_terminate_flag) {
            spdlog::warn("[State_WBC] Terminate flag set, starting recovery...");
            _recovery_idx = 0;
            _step_counter = 0;
            _terminate_flag = false;
            std::memcpy(_targetPos_rl, _joint_q, sizeof(_joint_q));
            // 本次仍使用当前 action，下一 tick 进入 recovery 分支
        }

        std::memcpy(_targetPos_rl, _joint_q, sizeof(_joint_q));
    }
    // 电机指令按 1kHz 写入，使用上次推理缓存的 _targetPos_rl

    for (int j = 0; j < NUM_DOF; j++) {
        lowcmd->msg_.motor_cmd()[j].mode() = 0x01;
        lowcmd->msg_.motor_cmd()[j].q() = _targetPos_rl[j];
        lowcmd->msg_.motor_cmd()[j].dq() = 0;
        lowcmd->msg_.motor_cmd()[j].tau() = 0;
        lowcmd->msg_.motor_cmd()[j].kp() = dof_Kps[j];
        lowcmd->msg_.motor_cmd()[j].kd() = dof_Kds[j];
        _last_targetPos_rl[j] = _targetPos_rl[j];
    }
    for (int idx : dummy_23dof) {
        lowcmd->msg_.motor_cmd()[idx].kp() = 0;
        lowcmd->msg_.motor_cmd()[idx].kd() = 0.5f;  // 最小阻尼防漂移（motor 27 曾漂到 -50）
        lowcmd->msg_.motor_cmd()[idx].tau() = 0;
    }
    _last_refer_idx = _refer_idx;
}

void State_WBC::exit()
{
    // 通知 Velocity 软臂接手（WBC 末态手臂离站立位远，需软过渡）
    test::velocity_soft_arms = true;
    spdlog::info("[State_WBC] Exiting WBC state (soft arm handover).");
}
