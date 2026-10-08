# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""
NVTX profiling sample: annotate a simulation for NVIDIA Nsight Systems.

This sample demonstrates how to:
1. Turn NVTX range emission on before the PhysX instance is created
2. Confirm it is active through the public config getter
3. Run a simulation whose phases then appear in an Nsight Systems capture

Run it under Nsight Systems to get the trace:

    nsys profile -t nvtx,cuda python nvtx_profiling.py

Without a profiler attached the ranges cost almost nothing, so this sample
runs and passes normally outside of Nsight.
"""

# [tutorial-start]
from pathlib import Path

import ovphysx
from ovphysx import ConfigBool, PhysX, PhysXConfig


_physx_schemas_registered = False


def attach_scene(physx, usd_path, stage_name):
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
    stage = ovstage.Stage(stage_name)
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.PHYSICS)
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.attach_ovstage(stage, read_ordinal=ordinal)
        return stage
    except Exception:
        stage.destroy()
        raise

# Enable NVTX range emission.
#
# IMPORTANT: this has to be set at initialization, before the physics engine is
# created internally, because that is when the PhysX profiler callback is
# installed. There is no way to turn it on later in the process.
#
# Setting OVPHYSX_NVTX=1 in the environment does the same thing without any code
# change, which is the usual way to profile an application you did not write.
physx = PhysX(config=PhysXConfig(nvtx_enabled=True))

# The effective state is readable, so a harness can log what it is doing rather
# than guess from an empty capture.
print(f"NVTX enabled: {physx.get_config_bool(ConfigBool.NVTX_ENABLED)}")

script_dir = Path(__file__).resolve().parent
usd_path = script_dir / ".." / "data" / "links_chain_sample.usda"
print(f"Loading USD scene: {usd_path}")

# Teardown runs even if attach or stepping raises: leaving a stage attached or an
# instance alive would leak the runtime singleton and mask the original error.
stage = None
try:
    stage = attach_scene(physx, usd_path, "ovphysx-nvtx-sample")
    physx.wait_all()

    # Every call below opens an NVTX range in the "ovphysx" domain, and the PhysX
    # SDK zones nested inside them land in the "PhysX" domain.
    dt = 1.0 / 60.0
    n_steps = 120  # 2 seconds at 60 Hz
    print(f"Simulating {n_steps} steps...")

    for i in range(n_steps):
        physx.step_sync(dt)

    print("Simulation complete.")
finally:
    if stage is not None:
        physx.detach_ovstage()
        stage.destroy()
    physx.destroy()
    print("Cleanup complete")

print(
    "\nRun this sample under Nsight Systems to see the ranges:\n"
    "  nsys profile -t nvtx,cuda python nvtx_profiling.py\n"
    "The timeline then shows an 'ovphysx' domain with the API calls and a\n"
    "'PhysX' domain with the SDK simulation phases, correlated with CUDA work."
)
# [tutorial-end]
