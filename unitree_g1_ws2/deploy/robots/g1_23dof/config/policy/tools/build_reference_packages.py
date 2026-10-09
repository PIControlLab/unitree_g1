#!/usr/bin/env python3
"""Build the deployment reference packages from the original exported data."""

from pathlib import Path
import io
import struct
import zipfile

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def save_npz(path: Path, **arrays):
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED) as archive:
        for name, value in arrays.items():
            buffer = io.BytesIO()
            np.lib.format.write_array(buffer, np.ascontiguousarray(value), allow_pickle=False)
            entry = zipfile.ZipInfo(f"{name}.npy")
            entry.compress_type = zipfile.ZIP_STORED
            archive.writestr(entry, buffer.getvalue())


def read_bin(path: Path):
    with path.open("rb") as stream:
        if stream.read(4) != b"NPZ\0":
            raise ValueError(path)
        ndim = struct.unpack("<I", stream.read(4))[0]
        shape = struct.unpack(f"<{ndim}I", stream.read(4 * ndim))
        word_size = struct.unpack("<I", stream.read(4))[0]
        dtype_code = stream.read(1).decode()
        stream.read(3)
        dtype = {("f", 4): "<f4", ("i", 8): "<i8"}[(dtype_code, word_size)]
        return np.frombuffer(stream.read(), dtype=dtype).reshape(shape)


def build_mimic():
    folder = ROOT / "mimic/dance_102/reference"
    source = ROOT / "mimic/dance_102/params/G1_Take_102.bvh_60hz.csv"
    data = np.loadtxt(source, delimiter=",", dtype=np.float32)
    keep = np.asarray([
        0, 6, 12, 1, 7, 15, 22, 2, 8, 16, 23, 3, 9, 17, 24, 4, 10, 18,
        25, 5, 11, 19, 26,
    ])
    joint_pos = data[:, 7 + keep]
    joint_vel = np.empty_like(joint_pos)
    joint_vel[:-1] = np.diff(joint_pos, axis=0) * 60.0
    joint_vel[-1] = joint_vel[-2]
    root_quat_wxyz = data[:, [6, 3, 4, 5]]
    folder.mkdir(parents=True, exist_ok=True)
    save_npz(
        folder / "data.npz",
        root_pos=data[:, :3],
        root_quat_wxyz=root_quat_wxyz,
        joint_pos=joint_pos,
        joint_vel=joint_vel,
    )


def build_wbc():
    source = ROOT / "wbc/motion_data"
    target = ROOT / "wbc/reference"
    target.mkdir(parents=True, exist_ok=True)
    names = [
        "body_ang_vel_w", "body_lin_vel_w", "body_pos_w", "body_quat_w",
        "joint_pos", "joint_vel",
    ]
    save_npz(target / "data.npz", **{name: read_bin(source / f"{name}.bin") for name in names})


def build_ufo():
    source = ROOT / "ufo/reference/walk1_subject1_130_140.uforef"
    target = ROOT / "ufo/reference/walk1_subject1_130_140"
    with source.open("rb") as stream:
        magic = stream.read(8)
        version, frames, width = struct.unpack("<III", stream.read(12))
        sample_rate_hz = struct.unpack("<f", stream.read(4))[0]
        if magic[:7] != b"UFOREF1" or version != 1 or width != 410 or sample_rate_hz != 50.0:
            raise ValueError(source)
        values = np.frombuffer(stream.read(), dtype="<f4").reshape(frames, width)
    target.mkdir(parents=True, exist_ok=True)
    save_npz(
        target / "data.npz",
        expert_state=values[:, :52],
        privileged_state=values[:, 52:],
    )


if __name__ == "__main__":
    build_mimic()
    build_wbc()
    build_ufo()
