# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""
OmniPVD recording sample: capture physics internals to .ovd files.

This sample demonstrates how to:
1. Configure OmniPVD recording via settings
2. Load a USD scene and run simulation
3. Produce a timestamped .ovd file for offline inspection in Kit

The resulting .ovd file can be opened in a compatible Kit application with the
OmniPVD extension (Window > Extensions > omni.physx.pvd) to inspect
simulation internals frame-by-frame. See docs/tutorials/omnipvd_recording.md
for the OVD format, OmniPVD stream version, and PhysX OVD integration version
compatibility policy.
"""

# [tutorial-start]
import glob
import os
import tempfile
from pathlib import Path

import ovphysx
from ovphysx import PhysX, PhysXConfig


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

# Use a temporary directory for recording output.
# Replace with your own path for persistent recordings.
output_dir = tempfile.mkdtemp(prefix="ovphysx_pvd_")
print(f"OmniPVD recording directory: {output_dir}")

# Initialize PhysX with OmniPVD recording enabled.
# IMPORTANT: Both config fields must be passed at initialization, before the
# physics engine is created internally. The recording directory must be
# a valid, writable path.
physx = PhysX(
    config=PhysXConfig(
        omnipvd_ovd_recording_directory=output_dir,
        omnipvd_output_enabled=True,
    )
)

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

# Run the simulation. OmniPVD captures each frame automatically.
dt = 1.0 / 60.0
n_steps = 120  # 2 seconds at 60 Hz
print(f"Simulating {n_steps} steps...")

for i in range(n_steps):
    physx.step_sync(dt)

print("Simulation complete.")

# Destroying the instance finalizes the recording:
# the runtime renames tmp.ovd -> <timestamp>_rec.ovd.
physx.detach_ovstage()
stage.destroy()
physx.destroy()
print("Runtime cleanup complete; recording retained for inspection")

# List the produced .ovd files
ovd_files = glob.glob(os.path.join(output_dir, "*_rec.ovd"))
if ovd_files:
    print(f"\nRecorded {len(ovd_files)} OVD file(s):")
    for f in ovd_files:
        size_kb = os.path.getsize(f) / 1024
        print(f"  {os.path.basename(f)}  ({size_kb:.1f} KB)")
    print("\nOpen these files in a Kit app with OmniPVD to inspect simulation data.")
else:
    print("\nWARNING: No .ovd files found. Check runtime logs for errors.")
# [tutorial-end]
