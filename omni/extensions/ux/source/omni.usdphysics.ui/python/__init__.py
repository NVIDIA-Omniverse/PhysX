# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.usdphysicsui.bindings._usdphysicsUI import acquire_usdphysics_ui_interface, IUsdPhysicsUI
from omni.usdphysicsui.bindings._usdphysicsUI import acquire_usdphysics_ui_private_interface, IUsdPhysicsUIPrivate

def get_usdphysicsui_interface() -> IUsdPhysicsUI:
    if not hasattr(get_usdphysicsui_interface, "iface"):
        get_usdphysicsui_interface.iface = acquire_usdphysics_ui_interface()
    return get_usdphysicsui_interface.iface

def get_usdphysicsuiprivate_interface() -> IUsdPhysicsUIPrivate:
    if not hasattr(get_usdphysicsuiprivate_interface, "iface"):
        get_usdphysicsuiprivate_interface.iface = acquire_usdphysics_ui_private_interface()
    return get_usdphysicsuiprivate_interface.iface

from .scripts.extension import *
