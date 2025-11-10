# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from .physxCct import PhysXCct
import omni.physxdemos as demo
from omni.physxuicommon import windowmenuitem
from omni.physx.scripts.utils import safe_import_tests
from .. import get_physx_cct_interface
from ..bindings._physxCct import release_physx_cct_interface

DEMO_MODULE = "omni.physxcct.scripts.scenes"
safe_import_tests("omni.physxcct.scripts.tests")


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
