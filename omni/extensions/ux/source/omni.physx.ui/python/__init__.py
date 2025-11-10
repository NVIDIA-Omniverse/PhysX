# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .bindings._physxUI import acquire_physx_ui_interface, acquire_physx_ui_private_interface, IPhysxUI, IPhysxUIPrivate

def get_physxui_interface() -> IPhysxUI:
    if not hasattr(get_physxui_interface, "iface"):
        get_physxui_interface.iface = acquire_physx_ui_interface()
    return get_physxui_interface.iface


def get_physxui_private_interface() -> IPhysxUIPrivate:
    if not hasattr(get_physxui_private_interface, "iface"):
        get_physxui_private_interface.iface = acquire_physx_ui_private_interface()
    return get_physxui_private_interface.iface

import omni.physxuicommon
from .scripts.extension import *
