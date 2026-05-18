# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#


# omni.physx public API
from .scripts.ifaces import *
from .scripts import physicsUtils as physics_utils

# make extension class privately visible
from .scripts.extension import PhysxExtension as _PhysxExtension
