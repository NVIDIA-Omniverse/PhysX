# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# Import from bindings directly to avoid circular import when used outside Kit.
# All types, enums, and dtype constants must be available before .api is
# imported, since api.py does ``from omni.physics.tensors import float32, ...``.
from omni.physics.tensors.bindings._physicsTensors import (
    acquire_tensor_api,
    # enums
    ObjectType, JointType, DofType, DofMotion, DofDriveType, TensorDataType,
    # data classes
    TensorDesc, DofProperties, Transform, Velocity,
    # dtype constants (exported by TensorDataType enum)
    float32, float64,
    int8, int16, int32, int64,
    uint8, uint16, uint32, uint64,
)
from .api import create_simulation_view, reset, SimulationView

# Kit extension lifecycle - only needed inside Kit
try:
    from .extension import Extension
except ImportError:
    pass
