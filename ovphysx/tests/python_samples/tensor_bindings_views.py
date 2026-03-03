# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""
Sample: "TensorAPI-like" views built on TensorBindingsAPI (ctypes).

Purpose
-------
This file is intentionally copy/paste-friendly for downstream users (e.g. IsaacLab)
who want some of the legacy TensorAPI ergonomics (SimulationView/ArticulationView)
without depending on the CPython-minor-specific pybind11 bindings from
`omni.physics.tensors`.

This is *not* an ovphysx-maintained library module; it is a sample that shows
how easy it is to build these helpers on top of the official TensorBindingsAPI.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from ovphysx import (
    OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32,
    OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32,
    PhysX,
)


def _find_usd_path() -> str:
    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir.parent.parent / "tests" / "data" / "links_chain_sample.usda",
        script_dir / "links_chain_sample.usda",
        script_dir.parent.parent.parent / "ovphysx" / "tests" / "data" / "links_chain_sample.usda",
    ]
    for c in candidates:
        if c.exists():
            return str(c)
    raise RuntimeError(f"Test data not found. Tried: {candidates}")


@dataclass(frozen=True)
class ArticulationView:
    """Minimal "view" wrapper around a couple of tensor bindings."""

    dof_positions: object
    dof_position_targets: object

    @property
    def count(self) -> int:
        return int(self.dof_positions.count)

    @property
    def shape(self) -> tuple[int, ...]:
        return tuple(self.dof_positions.shape)

    def get_dof_positions(self) -> np.ndarray:
        out = np.zeros(self.shape, dtype=np.float32)
        self.dof_positions.read(out)
        return out

    def set_dof_position_targets(self, targets: np.ndarray) -> None:
        if targets.shape != self.shape:
            raise ValueError(f"targets shape mismatch: expected {self.shape}, got {targets.shape}")
        self.dof_position_targets.write(targets.astype(np.float32, copy=False))


@dataclass(frozen=True)
class SimulationView:
    """Minimal simulation view that can manufacture other views."""

    physx: PhysX

    def create_articulation_view(self, pattern: str) -> ArticulationView:
        dof_pos = self.physx.create_tensor_binding(
            pattern=pattern,
            tensor_type=OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32,
        )
        dof_targets = self.physx.create_tensor_binding(
            pattern=pattern,
            tensor_type=OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32,
        )
        return ArticulationView(dof_positions=dof_pos, dof_position_targets=dof_targets)


def create_simulation_view(physx: PhysX) -> SimulationView:
    return SimulationView(physx=physx)


if __name__ == "__main__":
    print("=" * 60)
    print("SAMPLE: TensorBindingsAPI views helper (ctypes)")
    print("=" * 60)

    physx = PhysX(device="cpu")
    usd_path = _find_usd_path()
    print(f"Loading USD scene: {usd_path}")
    physx.add_usd(usd_path)
    physx.wait_all()

    sim_view = create_simulation_view(physx)
    art_view = sim_view.create_articulation_view("/World/articulation/articulationLink*")
    print(f"Found {art_view.count} articulations, shape={art_view.shape}")
    if art_view.count == 0:
        print("No articulations found in scene")
        sys.exit(1)

    pos0 = art_view.get_dof_positions()
    print(f"Initial DOF positions (first row): {pos0[0][:5]}...")

    art_view.set_dof_position_targets(pos0)
    print("Position targets set (echoing current positions)")

    for i in range(5):
        physx.step(1.0 / 60.0, i / 60.0)
    physx.wait_all()

    pos1 = art_view.get_dof_positions()
    delta = float(np.abs(pos1 - pos0).max())
    print(f"Max DOF position change after 5 steps: {delta:.6f}")

    physx.release()
    print("[OK] DONE")
