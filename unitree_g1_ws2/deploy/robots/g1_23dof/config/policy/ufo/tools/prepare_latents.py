#!/usr/bin/env python3
"""Precompute every UFO z used by the deployment catalog.

Tracking and goal latents follow UFO FB exactly: B(s) is produced by the
exported backward encoder, tracking averages the next ``seq_length`` values,
and every result is projected to norm sqrt(z_dim). Reward latents use UFO's
weighted-regression formula. By default the safe representative motion NPZs
are the inference sample corpus; pass ``--reward-samples`` to use an NPZ with
arrays ``z`` and ``reward_<task-id>`` generated from the training replay buffer.
"""

from __future__ import annotations

import argparse
import json
import re
import struct
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
from prepare_reference import (
    BODY_NAMES, DEFAULT_Q, JOINT_NAMES, load_kinematic_model, quat_mul_xyzw, quat_rotate_xyzw,
)


MOTIONS = {
    "walk": ("walk1_subject1_130_140.uforef", "walk1_subject1.npz", 130.0),
    "run": ("tracking_run.uforef", "run1_subject2.npz", 120.0),
    "jump": ("tracking_jump.uforef", "jumps1_subject1.npz", 120.0),
    "dance": ("tracking_dance.uforef", "dance1_subject1.npz", 70.0),
    "get_up": ("tracking_get_up.uforef", "fallAndGetUp1_subject1.npz", 80.0),
}
GOAL_FRAMES = {"walk": 250, "run": 250, "jump": 250, "dance": 250, "get_up": 400}
REWARD_TASKS = {
    "forward_0_3": (0.0, 0.3),
    "forward_0_7": (0.0, 0.7),
    "forward_1_0": (0.0, 1.0),
}


def fix_stand_encoder_input(config_path: Path, xml_path: Path):
    text = config_path.read_text(encoding="utf-8")
    names_match = re.search(r"joint_names:\s*\n\s*\[([^\]]+)\]", text, re.DOTALL)
    position_match = re.search(r"default_joint_pos:\s*\n\s*\[([^\]]+)\]", text, re.DOTALL)
    if not names_match or not position_match:
        raise ValueError(f"Cannot parse FixStand joints from {config_path}")
    names = [token.strip() for token in names_match.group(1).replace("\n", " ").split(",")]
    target = np.fromstring(position_match.group(1).replace("\n", " "), sep=",", dtype=np.float64)
    if names != JOINT_NAMES or target.shape != (len(JOINT_NAMES),):
        raise ValueError("FixStand joint order does not match the UFO contract")

    model = load_kinematic_model(xml_path)
    data = mujoco.MjData(model)
    qpos_idx = np.asarray([
        model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        for name in JOINT_NAMES
    ])
    body_ids = np.asarray([
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in BODY_NAMES
    ])
    data.qpos[qpos_idx] = target
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    positions = data.xpos[body_ids].copy()
    quats = data.xquat[body_ids][:, [1, 2, 3, 0]].copy()
    forward = quat_rotate_xyzw(quats[0], np.asarray([1.0, 0.0, 0.0]))
    heading = np.arctan2(forward[1], forward[0])
    heading_inv = np.asarray([0.0, 0.0, -np.sin(heading / 2.0), np.cos(heading / 2.0)])
    headings = np.repeat(heading_inv[None], len(body_ids), axis=0)
    local_pos = quat_rotate_xyzw(headings, positions - positions[0])
    local_rot = quat_mul_xyzw(headings, quats)
    tangent = quat_rotate_xyzw(local_rot, np.tile([1.0, 0.0, 0.0], (len(body_ids), 1)))
    normal = quat_rotate_xyzw(local_rot, np.tile([0.0, 0.0, 1.0], (len(body_ids), 1)))
    zero_body_velocity = np.zeros((len(body_ids), 3), dtype=np.float64)
    privileged = np.concatenate((
        positions[0, 2:3], local_pos[1:].ravel(),
        np.concatenate((tangent, normal), axis=1).ravel(),
        zero_body_velocity.ravel(), zero_body_velocity.ravel(),
    ))
    state = np.concatenate((target - DEFAULT_Q, np.zeros(23), [0.0, 0.0, -1.0], np.zeros(3)))
    if state.shape != (52,) or privileged.shape != (358,):
        raise AssertionError("unexpected FixStand encoder feature dimensions")
    return state[None].astype(np.float32), privileged[None].astype(np.float32), target.astype(np.float32)


def read_uforef(path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    with path.open("rb") as stream:
        magic = stream.read(8)
        version, frames, width = struct.unpack("<III", stream.read(12))
        sample_rate_hz = struct.unpack("<f", stream.read(4))[0]
        if magic[:7] != b"UFOREF1" or version != 1 or width != 410:
            raise ValueError(f"Invalid UFO reference header: {path}")
        values = np.frombuffer(stream.read(), dtype="<f4").reshape(frames, width)
    return values[:, :52], values[:, 52:], float(sample_rate_hz)


def project_z(z: np.ndarray) -> np.ndarray:
    z = np.asarray(z, dtype=np.float32)
    norm = np.linalg.norm(z, axis=-1, keepdims=True)
    return (np.sqrt(z.shape[-1]) * z / np.maximum(norm, 1.0e-12)).astype(np.float32)


def tracking_z(backward: np.ndarray, seq_length: int) -> np.ndarray:
    result = np.empty_like(backward)
    for step in range(len(backward)):
        result[step] = backward[step : min(step + seq_length, len(backward))].mean(axis=0)
    return project_z(result)


def softmax(x: np.ndarray) -> np.ndarray:
    shifted = x - np.max(x)
    exp = np.exp(shifted)
    return exp / exp.sum()


def reward_wr_z(backward: np.ndarray, reward: np.ndarray) -> np.ndarray:
    reward = np.asarray(reward, dtype=np.float32).reshape(-1)
    if len(reward) != len(backward):
        raise ValueError("reward and backward sample counts differ")
    weight = softmax(10.0 * reward)
    raw = np.sum((reward * weight)[:, None] * backward, axis=0, keepdims=True)
    return project_z(raw)


def quat_rotate_xyzw(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv, qw = q[..., :3], q[..., 3:4]
    return v * (2.0 * qw * qw - 1.0) + 2.0 * qw * np.cross(qv, v) + 2.0 * qv * np.sum(qv * v, axis=-1, keepdims=True)


def interpolate_motion_metrics(path: Path, start_s: float, frames: int, sample_rate_hz: float):
    source = np.load(path, allow_pickle=False)
    fps = float(source["fps"])
    source_pos = np.asarray(source["root_pos"], dtype=np.float64)
    source_quat = np.asarray(source["root_quat"], dtype=np.float64)
    sample = (start_s + np.arange(frames) / sample_rate_hz) * fps
    lo = np.floor(sample).astype(int)
    hi = np.minimum(lo + 1, len(source_pos) - 1)
    alpha = (sample - lo)[:, None]
    pos = (1.0 - alpha) * source_pos[lo] + alpha * source_pos[hi]
    quat_hi = source_quat[hi].copy()
    flip = np.sum(source_quat[lo] * quat_hi, axis=1) < 0.0
    quat_hi[flip] *= -1.0
    quat = (1.0 - alpha) * source_quat[lo] + alpha * quat_hi
    quat /= np.maximum(np.linalg.norm(quat, axis=1, keepdims=True), 1.0e-12)
    world_vel = np.gradient(pos, 1.0 / sample_rate_hz, axis=0)
    forward = quat_rotate_xyzw(quat, np.tile([1.0, 0.0, 0.0], (frames, 1)))
    heading = np.arctan2(forward[:, 1], forward[:, 0])
    c, s = np.cos(heading), np.sin(heading)
    ego_vel = np.stack((c * world_vel[:, 0] + s * world_vel[:, 1], -s * world_vel[:, 0] + c * world_vel[:, 1]), axis=1)
    up = quat_rotate_xyzw(quat, np.tile([0.0, 0.0, 1.0], (frames, 1)))[:, 2]
    return ego_vel.astype(np.float32), pos[:, 2].astype(np.float32), up.astype(np.float32)


def gaussian_tolerance(value: np.ndarray, lower: float, upper: float, margin: float, value_at_margin: float) -> np.ndarray:
    distance = np.maximum(lower - value, 0.0) + np.maximum(value - upper, 0.0)
    scaled = distance / max(margin, 1.0e-6)
    scale = np.sqrt(-2.0 * np.log(value_at_margin))
    return np.exp(-0.5 * np.square(scaled * scale))


def locomotion_reward(ego_vel: np.ndarray, height: np.ndarray, up: np.ndarray, angle_deg: float, speed: float) -> np.ndarray:
    standing = np.clip((height - 0.25) / 0.25, 0.0, 1.0)
    orientation = np.clip((up - 0.5) / 0.5, 0.0, 1.0)
    stand_reward = standing * orientation
    horizontal_speed = np.linalg.norm(ego_vel, axis=1)
    if speed <= 0.01:
        dont_move = gaussian_tolerance(np.abs(ego_vel), 0.0, 0.0, 0.2, 0.1).mean(axis=1)
        return (stand_reward * dont_move).astype(np.float32)
    move = gaussian_tolerance(horizontal_speed, 0.9 * speed, 1.1 * speed, 0.5 * speed, 0.5)
    move = (5.0 * move + 1.0) / 6.0
    radians = np.deg2rad(angle_deg)
    target = np.asarray([np.cos(radians), np.sin(radians)])
    direction = ego_vel / np.maximum(horizontal_speed[:, None], 1.0e-6)
    angle_reward = (direction @ target + 1.0) / 2.0
    angle_reward[horizontal_speed < 1.0e-6] = 1.0
    return (stand_reward * move * angle_reward).astype(np.float32)


def save_z(path: Path, z: np.ndarray, **metadata) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, z=np.ascontiguousarray(z, dtype=np.float32), **metadata)
    norms = np.linalg.norm(z, axis=1)
    if z.ndim != 2 or z.shape[1] != 256 or not np.all(np.isfinite(z)):
        raise ValueError(f"Invalid z written to {path}: {z.shape}")
    print(f"{path.name}: {z.shape}, norm=[{norms.min():.6f}, {norms.max():.6f}]")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--encoder", type=Path, required=True)
    parser.add_argument("--reference-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--fix-stand-config", type=Path, required=True)
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--seq-length", type=int, default=8)
    parser.add_argument("--reward-samples", type=Path, default=None)
    args = parser.parse_args()

    encoder = ort.InferenceSession(str(args.encoder), providers=["CPUExecutionProvider"])
    stand_state, stand_privileged, stand_target = fix_stand_encoder_input(
        args.fix_stand_config, args.xml
    )
    stand_backward = encoder.run(
        ["z"], {"state": stand_state, "privileged_state": stand_privileged}
    )[0].astype(np.float32)
    save_z(
        args.output_dir / "goal_stand_pose.npz", project_z(stand_backward),
        source=np.asarray("fix_stand"), target_joint_pos=stand_target,
    )
    reward_backward, reward_velocity, reward_height, reward_up = [], [], [], []

    for name, (feature_file, motion_file, start_s) in MOTIONS.items():
        state, privileged, sample_rate_hz = read_uforef(args.reference_dir / feature_file)
        backward = encoder.run(["z"], {"state": state, "privileged_state": privileged})[0].astype(np.float32)
        tracking = tracking_z(backward, args.seq_length)
        save_z(args.output_dir / f"tracking_{name}.npz", tracking, sample_rate_hz=np.float32(sample_rate_hz))
        goal = project_z(backward[GOAL_FRAMES[name] : GOAL_FRAMES[name] + 1])
        save_z(args.output_dir / f"goal_{name}_pose.npz", goal, source_frame=np.int32(GOAL_FRAMES[name]))
        velocity, height, up = interpolate_motion_metrics(
            args.reference_dir / motion_file, start_s, len(backward), sample_rate_hz
        )
        reward_backward.append(backward)
        reward_velocity.append(velocity)
        reward_height.append(height)
        reward_up.append(up)

    if args.reward_samples:
        samples = np.load(args.reward_samples, allow_pickle=False)
        reward_b = np.asarray(samples["z"], dtype=np.float32)
        for task in REWARD_TASKS:
            key = f"reward_{task}"
            if key not in samples:
                raise KeyError(f"{args.reward_samples} has no {key}")
            save_z(args.output_dir / f"reward_{task}.npz", reward_wr_z(reward_b, samples[key]))
    else:
        reward_b = np.concatenate(reward_backward)
        velocity = np.concatenate(reward_velocity)
        height = np.concatenate(reward_height)
        up = np.concatenate(reward_up)
        for task, (angle, speed) in REWARD_TASKS.items():
            reward = locomotion_reward(velocity, height, up, angle, speed)
            save_z(
                args.output_dir / f"reward_{task}.npz",
                reward_wr_z(reward_b, reward),
                sample_count=np.int32(len(reward)),
                reward_min=np.float32(reward.min()),
                reward_max=np.float32(reward.max()),
            )
        print("WARNING: reward z used the bundled representative-motion corpus, not the missing training replay buffer.")

    catalog = json.loads((args.output_dir.parent / "latent_catalog.json").read_text(encoding="utf-8"))
    missing = [entry["latent_file"] for entry in catalog["entries"] if not (args.output_dir.parent / entry["latent_file"]).exists()]
    if missing:
        raise FileNotFoundError(f"Catalog outputs missing: {missing}")


if __name__ == "__main__":
    main()