# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

# [tutorial-start]
# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import PhysX
from pathlib import Path

print("Using ovphysx version: ", ovphysx.__version__)

_physx_schemas_registered = False


def attach_scene(physx, usd_path):
    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    # Register external PhysX and Newton schemas before the first population.
    global _physx_schemas_registered
    if not _physx_schemas_registered:
        ovstage.population.register_usd_schemas(
            [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
        )
        _physx_schemas_registered = True
    stage = ovstage.Stage("ovphysx-hello-world")
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=ordinal))
        print("Loaded scene through ovstage")
        return stage
    except Exception:
        if physx.is_stage_attached(stage):
            physx.wait_task(physx.detach_ovstage())
        stage.destroy()
        raise

# Prefer package data so a copied sample works. Fall back to the checked-in
# sample's adjacent data directory when package data is absent.
usd_path = Path(ovphysx.__file__).resolve().parent / "samples" / "data" / "links_chain_sample.usda"
if not usd_path.is_file():
    usd_path = Path(__file__).resolve().parent.parent / "data" / "links_chain_sample.usda"
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

# Initialize PhysX
physx = PhysX()
stage = attach_scene(physx, usd_path)

try:
    # Run a simulation step
    dt = 1.0 / 60.0
    physx.step_sync(dt)
    print("Simulation step completed successfully")
finally:
    if stage is not None:
        physx.wait_task(physx.detach_ovstage())
        stage.destroy()
    physx.destroy()
    print("Cleanup complete")
# [tutorial-end]
