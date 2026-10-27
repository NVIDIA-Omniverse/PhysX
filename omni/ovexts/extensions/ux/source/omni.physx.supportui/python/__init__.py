# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

__all__ = ["get_physx_supportui_interface"]

from .bindings._physxSupportUi import acquire_physx_supportui_interface, IPhysxSupportUi
from .bindings._physxSupportUi import acquire_physx_supportui_private_interface, IPhysxSupportUiPrivate

from .ifacecache import cached_interface

def get_physx_supportui_interface() -> IPhysxSupportUi:
    return cached_interface(get_physx_supportui_interface, acquire_physx_supportui_interface)

def get_physx_supportui_private_interface() -> IPhysxSupportUiPrivate:
    return cached_interface(get_physx_supportui_private_interface, acquire_physx_supportui_private_interface)

from .scripts.extension import PhysxSupportUiExtension
import omni.physx
omni.physx.scripts.safe_import_tests("omni.physxsupportui.scripts.tests")
