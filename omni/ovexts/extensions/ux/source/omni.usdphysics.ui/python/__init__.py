# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from omni.usdphysicsui.bindings._usdphysicsUI import acquire_usdphysics_ui_interface, IUsdPhysicsUI
from omni.usdphysicsui.bindings._usdphysicsUI import acquire_usdphysics_ui_private_interface, IUsdPhysicsUIPrivate
from .ifacecache import cached_interface

def get_usdphysicsui_interface() -> IUsdPhysicsUI:
    return cached_interface(get_usdphysicsui_interface, acquire_usdphysics_ui_interface)

def get_usdphysicsuiprivate_interface() -> IUsdPhysicsUIPrivate:
    return cached_interface(get_usdphysicsuiprivate_interface, acquire_usdphysics_ui_private_interface)

from .scripts.extension import *
