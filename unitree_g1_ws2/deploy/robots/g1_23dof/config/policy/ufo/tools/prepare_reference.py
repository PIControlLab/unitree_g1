#!/usr/bin/env python3
"""Convert a safe UFO reference NPZ into the compact C++ deployment format."""

from __future__ import annotations

import argparse
import struct
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
]
BODY_NAMES = [
    "pelvis", "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link",
    "left_knee_link", "left_ankle_pitch_link", "left_ankle_roll_link",
    "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link",
    "right_knee_link", "right_ankle_pitch_link", "right_ankle_roll_link",
    "torso_link", "left_shoulder_pitch_link", "left_shoulder_roll_link",
    "left_shoulder_yaw_link", "left_elbow_link", "left_wrist_roll_rubber_hand",
    "right_shoulder_pitch_link", "right_shoulder_roll_link",
    "right_shoulder_yaw_link", "right_elbow_link", "right_wrist_roll_rubber_hand",
]
DEFAULT_Q = np.asarray([
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
])


def quat_rotate_xyzw(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv, qw = q[..., :3], q[..., 3:4]
    return v * (2.0 * qw * qw - 1.0) + 2.0 * qw * np.cross(qv, v) + 2.0 * qv * np.sum(qv * v, axis=-1, keepdims=True)


def quat_mul_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    av, aw, bv, bw = a[..., :3], a[..., 3:4], b[..., :3], b[..., 3:4]
    return np.concatenate((aw * bv + bw * av + np.cross(av, bv), aw * bw - np.sum(av * bv, axis=-1, keepdims=True)), axis=-1)


def qpos_at(root_pos, root_quat, dof_pos, fps, time_s):
    p = np.clip(time_s * fps, 0.0, len(root_pos) - 1.0)
    lo, hi, alpha = int(np.floor(p)), min(int(np.floor(p)) + 1, len(root_pos) - 1), p - np.floor(p)
    quat = root_quat[lo].copy()
    other = root_quat[hi].copy()
    if np.dot(quat, other) < 0.0:
        other *= -1.0
    quat = quat * (1.0 - alpha) + other * alpha
    quat /= np.linalg.norm(quat)
    return np.concatenate(((1.0 - alpha) * root_pos[lo] + alpha * root_pos[hi], np.roll(quat, 1), (1.0 - alpha) * dof_pos[lo] + alpha * dof_pos[hi]))


def load_kinematic_model(xml_path: Path):
    root = ET.parse(xml_path).getroot()
    for asset in root.findall("asset"):
        for child in list(asset):
            if child.tag == "mesh":
                asset.remove(child)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "geom" and child.get("mesh"):
                parent.remove(child)
    with tempfile.TemporaryDirectory() as tmp:
        stripped = Path(tmp) / "kinematics.xml"
        ET.ElementTree(root).write(stripped)
        return mujoco.MjModel.from_xml_path(str(stripped))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--start", type=float, default=130.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source = np.load(args.reference, allow_pickle=False)
    if source["joint_names"].tolist() != JOINT_NAMES:
        raise ValueError("reference joint order does not match the UFO contract")
    root_pos, root_quat, dof_pos = source["root_pos"], source["root_quat"], source["dof_pos"]
    fps = float(source["fps"])
    source_duration = (len(root_pos) - 1) / fps
    if args.start < 0 or args.duration <= 0 or args.start + args.duration > source_duration:
        raise ValueError("requested time range is outside the reference")

    model = load_kinematic_model(args.xml)
    data = mujoco.MjData(model)
    qpos_idx = np.asarray([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in JOINT_NAMES])
    qvel_idx = np.asarray([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in JOINT_NAMES])
    body_ids = np.asarray([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in BODY_NAMES])
    times = args.start + np.arange(int(round(args.duration * 50.0)) + 1) * 0.02
    frames = []
    velocity = np.empty(6)

    for time_s in times:
        qpos = qpos_at(root_pos, root_quat, dof_pos, fps, time_s)
        dt = min(0.002, 0.25 / fps)
        before = qpos_at(root_pos, root_quat, dof_pos, fps, max(0.0, time_s - dt))
        after = qpos_at(root_pos, root_quat, dof_pos, fps, min(source_duration, time_s + dt))
        interval = min(source_duration, time_s + dt) - max(0.0, time_s - dt)
        qvel = np.zeros(model.nv)
        mujoco.mj_differentiatePos(model, qvel, interval, before, after)
        data.qpos[:], data.qvel[:] = qpos, qvel
        mujoco.mj_forward(model, data)

        positions, quats, linear, angular = data.xpos[body_ids].copy(), data.xquat[body_ids][:, [1, 2, 3, 0]].copy(), [], []
        for body_id in body_ids:
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, int(body_id), velocity, 0)
            angular.append(velocity[:3].copy())
            linear.append(velocity[3:].copy())
        linear, angular = np.asarray(linear), np.asarray(angular)
        forward = quat_rotate_xyzw(quats[0], np.asarray([1.0, 0.0, 0.0]))
        heading = np.arctan2(forward[1], forward[0])
        heading_inv = np.asarray([0.0, 0.0, -np.sin(heading / 2.0), np.cos(heading / 2.0)])
        headings = np.repeat(heading_inv[None], len(body_ids), axis=0)
        local_pos = quat_rotate_xyzw(headings, positions - positions[0])
        local_rot = quat_mul_xyzw(headings, quats)
        tangent = quat_rotate_xyzw(local_rot, np.tile([1.0, 0.0, 0.0], (len(body_ids), 1)))
        normal = quat_rotate_xyzw(local_rot, np.tile([0.0, 0.0, 1.0], (len(body_ids), 1)))
        privileged = np.concatenate((positions[0, 2:3], local_pos[1:].ravel(), np.concatenate((tangent, normal), axis=1).ravel(), quat_rotate_xyzw(headings, linear).ravel(), quat_rotate_xyzw(headings, angular).ravel()))
        quat_wxyz = qpos[3:7]
        quat_conj = np.asarray([-quat_wxyz[1], -quat_wxyz[2], -quat_wxyz[3], quat_wxyz[0]])
        gravity = quat_rotate_xyzw(quat_conj, np.asarray([0.0, 0.0, -1.0]))
        state = np.concatenate((qpos[qpos_idx] - DEFAULT_Q, qvel[qvel_idx], gravity, qvel[3:6] * 0.25))
        if state.shape != (52,) or privileged.shape != (358,):
            raise AssertionError("unexpected reference feature dimensions")
        frames.append(np.concatenate((state, privileged)).astype("<f4"))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("wb") as stream:
        stream.write(struct.pack("<8sIIIf", b"UFOREF1\0", 1, len(frames), 410, 50.0))
        np.stack(frames).astype("<f4").tofile(stream)
    print(f"wrote {len(frames)} frames ({args.duration:.2f}s) to {args.output}")


if __name__ == "__main__":
    main()
