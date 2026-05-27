# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# NOTE: This file is included verbatim in documentation via literalinclude.
# Tutorial marker comments below define the included range.

"""
OmniPVD recording sample -- capture physics internals to .ovd files.

This sample demonstrates how to:
1. Configure OmniPVD recording via settings
2. Load a USD scene and run simulation
3. Produce a timestamped .ovd file for offline inspection in Kit

The resulting .ovd file can be opened in any Kit application with the
OmniPVD extension (Window > Extensions > omni.physx.pvd) to inspect
simulation internals frame-by-frame.
"""

# [tutorial-start]
import glob
import os
import tempfile
from pathlib import Path

from ovphysx import PhysX, PhysXConfig

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

# Load a USD scene with physics objects
script_dir = Path(__file__).resolve().parent
usd_path = script_dir / ".." / "data" / "links_chain_sample.usda"
print(f"Loading USD scene: {usd_path}")

physx.add_usd(str(usd_path))
physx.wait_all()

# Run simulation -- OmniPVD captures each frame automatically
dt = 1.0 / 60.0
n_steps = 120  # 2 seconds at 60 Hz
print(f"Simulating {n_steps} steps...")

for i in range(n_steps):
    physx.step_sync(dt, i * dt)

print("Simulation complete.")

# Destroying the instance finalizes the recording:
# the runtime renames tmp.ovd -> <timestamp>_rec.ovd.
physx.release()
print("Cleanup complete")

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
