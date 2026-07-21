# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

# [tutorial-start]
import ovphysx
from ovphysx import PhysX
from pathlib import Path

print("Using ovphysx version: ", ovphysx.__version__)

def attach_scene(physx, usd_path):
    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    stage = ovstage.Stage("ovphysx-hello-world")
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.PHYSICS)
        physx.attach_ovstage(stage, read_ordinal=ordinal)
        print("Loaded scene through ovstage")
        return stage
    except Exception:
        stage.destroy()
        raise

script_dir = Path(__file__).resolve().parent
usd_path = script_dir / ".." / "data" / "links_chain_sample.usda"

# Initialize PhysX
physx = PhysX()
stage = attach_scene(physx, usd_path)

try:
    # Run a simulation step
    dt = 1.0 / 60.0
    physx.step(dt)
    print("Simulation step completed successfully")
finally:
    if stage is not None:
        physx.detach_ovstage()
        stage.destroy()
    physx.release()
    print("Cleanup complete")
# [tutorial-end]
