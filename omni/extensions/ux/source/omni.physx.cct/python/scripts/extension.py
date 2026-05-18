# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.ext
from .physxCct import PhysXCct
import omni.physxdemos as demo
from omni.physxuicommon import windowmenuitem
from .. import get_physx_cct_interface
from ..bindings._physxCct import release_physx_cct_interface

DEMO_MODULE = "omni.physxcct.scripts.scenes"


class PhysxCctExtension(omni.ext.IExt):
    def on_startup(self):
        self._physx_cct_interface = get_physx_cct_interface()
        self._menu = windowmenuitem.WindowMenuItem("Character Controller", lambda: PhysXCct())
        demo.register(DEMO_MODULE)

    def on_shutdown(self):
        self._menu.on_shutdown()
        self._menu = None
        demo.unregister(DEMO_MODULE)
        release_physx_cct_interface(self._physx_cct_interface)
        self._physx_cct_interface = None
