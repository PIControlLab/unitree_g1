#!/usr/bin/env python3
"""Install exported motion-goal and replay-reward latents; preserve tracking/stand."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


UFO_DIR = Path(__file__).resolve().parent.parent
ARCHIVE = "ufo_g1_23dof_latents.npz"
REWARD_LABELS = {
    "move-ego-0-0": "保持静止",
    "move-ego-low0.5-0-0": "低姿态静止",
    "move-ego-0-0.7": "前进 0.7 m/s",
    "move-ego-0-0.3": "前进 0.3 m/s",
    "move-ego-90-0.3": "左移 0.3 m/s",
    "move-ego-180-0.3": "后退 0.3 m/s",
    "move-ego--90-0.3": "右移 0.3 m/s",
    "rotate-z-5-0.5": "左转 5 rad/s",
    "rotate-z--5-0.5": "右转 5 rad/s",
}
GOAL_LABELS = {
    "fallAndGetUp1_subject4_2193": "低姿态 1",
    "fallAndGetUp1_subject4_2230": "低姿态 2",
    "fallAndGetUp1_subject4_3350": "低姿态 3",
    "fallAndGetUp1_subject4_5015": "低姿态 4",
    "dance1_subject3_505": "舞蹈 1",
    "dance2_subject4_700": "舞蹈 2",
    "dance2_subject4_670": "舞蹈 3",
    "dance1_subject2_1150": "舞蹈 4",
    "dance1_subject2_2877": "舞蹈 5",
    "fightAndSports1_subject1_252": "格斗 1",
    "fightAndSports1_subject1_745": "格斗 2",
    "fightAndSports1_subject1_1009": "格斗 3",
    "fight1_subject3_5559": "格斗 4",
    "walk2_subject1_2588": "行走",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def model_sha256() -> str:
    digest = hashlib.sha256()
    for part in ("FBcprAuxModel.onnx.part00", "FBcprAuxModel.onnx.part01"):
        with (UFO_DIR / "model" / part).open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


def validate_vectors(values: np.ndarray, names: list[str], items: list[dict]) -> None:
    expected_names = [item["name"] for item in items]
    if names != expected_names or values.shape != (len(names), 256):
        raise ValueError("Archive names or latent shape disagree with manifest")
    if values.dtype != np.float32 or not np.isfinite(values).all():
        raise ValueError("Latents must be finite float32 values")
    if not np.allclose(np.linalg.norm(values, axis=1), 16.0, atol=1e-3):
        raise ValueError("Latent norm differs from 16")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, default=UFO_DIR / "source")
    args = parser.parse_args()
    package = args.package
    manifest = json.loads((package / "latent_manifest.json").read_text(encoding="utf-8"))
    archive = package / ARCHIVE
    source_sha = sha256(archive)
    if source_sha != manifest["files"][ARCHIVE]["sha256"]:
        raise ValueError("Archive SHA-256 disagrees with manifest")
    if model_sha256() != manifest["files"]["FBcprAuxModel.onnx"]["sha256"]:
        raise ValueError("Existing UFO policy does not match replay latents")
    definitions = package / "goal_definitions_g1_23dof.json"
    if sha256(definitions) != manifest["files"][definitions.name]["sha256"]:
        raise ValueError("Goal definitions SHA-256 disagrees with manifest")

    with np.load(archive, allow_pickle=False) as data:
        if int(data["checkpoint_global_steps"]) != manifest["checkpoint"]["global_env_steps"]:
            raise ValueError("Checkpoint step disagrees with manifest")
        if int(data["latent_dim"]) != 256:
            raise ValueError("Expected 256-dimensional latent")
        goals = np.asarray(data["goal_latents"])
        goal_names = data["goal_names"].tolist()
        rewards = np.asarray(data["reward_latents"])
        reward_names = data["reward_names"].tolist()
    validate_vectors(goals, goal_names, manifest["goal"]["items"])
    validate_vectors(rewards, reward_names, manifest["reward"]["items"])
    if set(goal_names) != set(GOAL_LABELS):
        raise ValueError("Goal labels do not match the exported goal names")


    catalog_path = UFO_DIR / "latent_catalog.json"
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    tracking = [item for item in catalog["entries"] if item["function"] == "tracking"]
    stand = next(item for item in catalog["entries"] if item["id"] == "goal_reaching/stand_pose")
    stale = [
        UFO_DIR / item["latent_file"] for item in catalog["entries"]
        if item["function"] != "tracking" and item["id"] != stand["id"]
    ]
    entries = tracking + [stand]
    latent_dir = UFO_DIR / "latents"

    for function, values, names, items, prefix in (
        ("goal_reaching", goals, goal_names, manifest["goal"]["items"], "goal"),
        ("reward_inference", rewards, reward_names, manifest["reward"]["items"], "reward"),
    ):
        for index, (name, z, item) in enumerate(zip(names, values, items)):
            filename = f"{prefix}_replay_{index:02d}.npz"
            path = latent_dir / filename
            np.savez(
                path,
                z=np.ascontiguousarray(z[None, :]),
                source_name=np.asarray(name),
                source_archive_sha256=np.asarray(source_sha),
                checkpoint_global_steps=np.int64(manifest["checkpoint"]["global_env_steps"]),
            )
            label = (
                GOAL_LABELS[name]
                if function == "goal_reaching" else REWARD_LABELS[name]
            )
            entries.append({
                "id": f"{function}/{name}",
                "function": function,
                "label": label,
                "playback": "constant",
                "latent_file": f"latents/{filename}",
                "sample_rate_hz": 50.0,
            })

    goal_entries = {
        entry["id"]: entry for entry in entries
        if entry["function"] == "goal_reaching" and entry["id"] != stand["id"]
    }
    reward_entries = [entry for entry in entries if entry["function"] == "reward_inference"]
    entries = tracking + [stand] + [
        goal_entries[f"goal_reaching/{name}"] for name in GOAL_LABELS
    ] + reward_entries
    catalog["entries"] = entries
    catalog_path.write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    new_paths = {UFO_DIR / item["latent_file"] for item in entries}
    for path in stale:
        if path not in new_paths and path.is_file():
            path.unlink()
    print(f"Installed {len(goals)} goal and {len(rewards)} reward latents.")
    print(f"Preserved {len(tracking)} tracking entries and the FixStand-derived stand goal.")


if __name__ == "__main__":
    main()
