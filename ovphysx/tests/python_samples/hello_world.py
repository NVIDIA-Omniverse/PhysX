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

# Initialize PhysX
physx = PhysX()

# Load USD scene with physics setup
script_dir = Path(__file__).resolve().parent
usd_path = script_dir / ".." / "data" / "links_chain_sample.usda"
physx.add_usd(str(usd_path))

# Run a simulation step
dt = 1.0 / 60.0
elapsed_time = 0.0
physx.step(dt, elapsed_time)

print("Simulation step completed successfully")

physx.release()
print("Cleanup complete")
# [tutorial-end]
