# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

# [tutorial-start]
import ovphysx
from ovphysx import PhysX
from pathlib import Path

print("Using ovphysx version: ", ovphysx.__version__)

_physx_schemas_registered = False


def attach_scene(physx, usd_path):
    import ovstage

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")

    # ovphysx ships its PhysX USD schemas as codeless resources and does not register
    # them itself. Register them with ovstage once, before the first population
    # call in the process.
    global _physx_schemas_registered
    if not _physx_schemas_registered:
        ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
        _physx_schemas_registered = True
    stage = ovstage.Stage("ovphysx-hello-world")
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.PHYSICS)
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.attach_ovstage(stage, read_ordinal=ordinal)
        print("Loaded scene through ovstage")
        return stage
    except Exception:
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
        physx.detach_ovstage()
        stage.destroy()
    physx.destroy()
    print("Cleanup complete")
# [tutorial-end]
