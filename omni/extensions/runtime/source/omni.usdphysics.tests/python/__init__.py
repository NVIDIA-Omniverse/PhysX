# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .bindings._usdPhysicsTests import acquire_usd_physics_tests_interface, IUsdPhysicsTests

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_usd_physics_test_interface() -> IUsdPhysicsTests:
    return _get_interface(get_usd_physics_test_interface, acquire_usd_physics_tests_interface)


from .scripts.extension import *
