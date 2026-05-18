# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

__all__ = ["get_physx_supportui_interface"]

from .bindings._physxSupportUi import acquire_physx_supportui_interface, IPhysxSupportUi
from .bindings._physxSupportUi import acquire_physx_supportui_private_interface, IPhysxSupportUiPrivate

def _get_interface(func, acq):
    if not hasattr(func, "iface"):
        func.iface = acq()
    return func.iface

def get_physx_supportui_interface() -> IPhysxSupportUi:
    return _get_interface(get_physx_supportui_interface, acquire_physx_supportui_interface)

def get_physx_supportui_private_interface() -> IPhysxSupportUiPrivate:
    return _get_interface(get_physx_supportui_private_interface, acquire_physx_supportui_private_interface)

from .scripts.extension import PhysxSupportUiExtension
import omni.physx
omni.physx.scripts.safe_import_tests("omni.physxsupportui.scripts.tests")
