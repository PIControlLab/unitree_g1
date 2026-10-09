#!/usr/bin/env python3
"""Validate deployed UFO task files and compare with the source archive if present."""

import json
from pathlib import Path

import numpy as np
from install_replay_latents import GOAL_LABELS


ROOT = Path(__file__).resolve().parent.parent
catalog = json.loads((ROOT / "latent_catalog.json").read_text(encoding="utf-8"))
entries = catalog["entries"]
assert len(entries) == 29
assert len({entry["id"] for entry in entries}) == len(entries)
assert sum(entry["function"] == "tracking" for entry in entries) == 5
assert sum(entry["function"] == "goal_reaching" for entry in entries) == 15
assert sum(entry["function"] == "reward_inference" for entry in entries) == 9

goal_entries = [
    entry for entry in entries
    if entry["function"] == "goal_reaching" and entry["id"] != "goal_reaching/stand_pose"
]
assert [entry["id"] for entry in goal_entries] == [
    f"goal_reaching/{name}" for name in GOAL_LABELS
]
assert [entry["label"] for entry in goal_entries] == list(GOAL_LABELS.values())

for entry in entries:
    path = ROOT / entry["latent_file"]
    assert path.is_file(), path
    with np.load(path, allow_pickle=False) as item:
        z = item["z"]
        assert z.ndim == 2 and z.shape[1] == 256 and z.dtype == np.float32
        assert np.isfinite(z).all()
        if entry["function"] != "tracking":
            assert z.shape == (1, 256)
        if entry["function"] in ("goal_reaching", "reward_inference") and entry["id"] != "goal_reaching/stand_pose":
            assert str(item["source_name"]) == entry["id"].split("/", 1)[1]

source_path = ROOT / "source/ufo_g1_23dof_latents.npz"
if source_path.is_file():
    with np.load(source_path, allow_pickle=False) as source:
        for function, key in (("goal_reaching", "goal"), ("reward_inference", "reward")):
            selected = [
                entry for entry in entries
                if entry["function"] == function and entry["id"] != "goal_reaching/stand_pose"
            ]
            names = [str(name) for name in source[key + "_names"].tolist()]
            assert len(selected) == len(names)
            source_index = {name: index for index, name in enumerate(names)}
            assert len(source_index) == len(names)
            for entry in selected:
                name = entry["id"].split("/", 1)[1]
                with np.load(ROOT / entry["latent_file"], allow_pickle=False) as item:
                    np.testing.assert_array_equal(
                        item["z"][0], source[key + "_latents"][source_index[name]]
                    )
    print("PASS: 29 catalog entries and 23 exact exported vectors")
else:
    print("PASS: 29 catalog entries and all task files; source comparison skipped")
