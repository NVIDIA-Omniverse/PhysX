# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from .. import get_physx_fabric_interface
from omni.physxfabric.bindings._physxFabric import release_physx_fabric_interface
from omni.physxfabric.bindings._physxFabric import release_physx_fabric_interface_scripting

class PhysxFabricExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()

    def on_startup(self):
        # getting plugin to be alive
        self._fabric_iface = get_physx_fabric_interface()
        self._settings = None
        try:
            from .settings import PhysicsFcSettings
            import omni.kit.window.preferences.scripts.preferences_window as preferences_window
            self._settings = preferences_window.register_page(PhysicsFcSettings())
        except:
            pass        

    def on_shutdown(self):
        if self._settings is not None:
            import omni.kit.window.preferences.scripts.preferences_window as preferences_window
            preferences_window.unregister_page(self._settings)
            self._settings = None
        
        release_physx_fabric_interface(self._fabric_iface)
        release_physx_fabric_interface_scripting(self._fabric_iface) # OM-60917
        self._fabric_iface = None
