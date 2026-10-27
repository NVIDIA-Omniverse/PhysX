# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

__all__ = ["get_physxui_interface"]

from .bindings._physxUI import acquire_physx_ui_interface, acquire_physx_ui_private_interface, IPhysxUI, IPhysxUIPrivate
from .ifacecache import cached_interface

def get_physxui_interface() -> IPhysxUI:
    return cached_interface(get_physxui_interface, acquire_physx_ui_interface)


def get_physxui_private_interface() -> IPhysxUIPrivate:
    return cached_interface(get_physxui_private_interface, acquire_physx_ui_private_interface)

import omni.physxuicommon
from .scripts.extension import PhysxUIExtension
from .scripts.extension import get_input_manager, get_physicsui_instance, PhysxUIMouseInteraction, PhysicsMenu
