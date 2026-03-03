# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Verify ovphysx installation: ``python -m ovphysx``."""

import ovphysx
from ovphysx import PhysX

print(f"ovphysx {ovphysx.__version__}")

physx = PhysX()
physx.step(1.0 / 60.0, 0.0)
physx.release()

print("OK")
