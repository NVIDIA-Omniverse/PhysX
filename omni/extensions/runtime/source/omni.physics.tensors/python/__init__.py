# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# Import from bindings directly to avoid circular import when used outside Kit
from omni.physics.tensors.bindings._physicsTensors import acquire_tensor_api
from .api import create_simulation_view, reset

# Kit extension lifecycle - only needed inside Kit
try:
    from .extension import Extension
except ImportError:
    pass
