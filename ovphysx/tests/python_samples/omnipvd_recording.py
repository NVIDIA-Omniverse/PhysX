# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""
OmniPVD recording sample: capture physics internals to .ovd files.

This sample demonstrates how to:
1. Enable recording capability before runtime creation
2. Load a USD scene and run simulation
3. Record to an exact .ovd file for offline inspection in Kit

The resulting .ovd file can be opened in a compatible Kit application with the
OmniPVD extension (Window > Extensions > omni.physx.pvd) to inspect
simulation internals frame-by-frame. See docs/tutorials/omnipvd_recording.md
for the OVD format, OmniPVD stream version, and PhysX OVD integration version
compatibility policy.
"""

# [tutorial-start]
# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1

import glob
import os
import tempfile
from pathlib import Path

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, set_global_config


_physx_schemas_registered = False


def attach_scene(physx, usd_path, stage_name):
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
    stage = ovstage.Stage(stage_name)
    ordinal = 1
    try:
        ovstage.population.open_usd(stage, str(usd_path), ordinal=ordinal, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=ordinal).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=ordinal))
        return stage
    except Exception:
        if physx.is_stage_attached(stage):
            physx.wait_task(physx.detach_ovstage())
        stage.destroy()
        raise

# Use a temporary directory for recording output.
# Replace with your own path for persistent recordings.
output_dir = tempfile.mkdtemp(prefix="ovphysx_pvd_")
print(f"OmniPVD recording directory: {output_dir}")

# Capability must exist before the shared runtime creates PxPhysics.
set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
physx = PhysX()

# Prefer package data so a copied sample works. Fall back to the checked-in
# sample's adjacent data directory when package data is absent.
usd_path = (
    Path(ovphysx.__file__).resolve().parent
    / "samples"
    / "data"
    / "links_chain_sample.usda"
)
if not usd_path.is_file():
    usd_path = Path(__file__).resolve().parent.parent / "data" / "links_chain_sample.usda"
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")
print(f"Loading USD scene: {usd_path}")

stage = attach_scene(physx, usd_path, "ovphysx-omnipvd-sample")
physx.wait_all()
physx.start_recording(OmniPvdDestination.file(str(Path(output_dir) / "capture.ovd")))

# Run the simulation. OmniPVD captures each frame automatically.
dt = 1.0 / 60.0
n_steps = 120  # 2 seconds at 60 Hz
print(f"Simulating {n_steps} steps...")

for i in range(n_steps):
    physx.step_sync(dt)

print("Simulation complete.")

# Stop finalizes the exact destination before releasing the borrowed stage.
physx.stop_recording()
physx.wait_task(physx.detach_ovstage())
stage.destroy()
physx.destroy()
print("Runtime cleanup complete; recording retained for inspection")

# List the produced .ovd files
ovd_files = glob.glob(os.path.join(output_dir, "*.ovd"))
if ovd_files:
    print(f"\nRecorded {len(ovd_files)} OVD file(s):")
    for f in ovd_files:
        size_kb = os.path.getsize(f) / 1024
        print(f"  {os.path.basename(f)}  ({size_kb:.1f} KB)")
    print("\nOpen these files in a Kit app with OmniPVD to inspect simulation data.")
else:
    print("\nWARNING: No .ovd files found. Check runtime logs for errors.")
# [tutorial-end]
