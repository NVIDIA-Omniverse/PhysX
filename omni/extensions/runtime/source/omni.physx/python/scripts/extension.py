# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from .. import get_physx_interface, get_physxunittests_interface, get_physx_visualization_interface, get_physx_scene_query_interface, get_physx_cooking_interface, get_physx_cooking_private_interface, get_physx_simulation_interface, get_physx_benchmarks_interface, get_physx_attachment_private_interface, get_physx_property_query_interface, get_physx_stage_update_interface, get_physx_statistics_interface
from ..bindings._physx import release_physx_interface, release_physx_interface_scripting, release_physxunittests_interface, release_physx_visualization_interface, release_physx_scene_query_interface, release_physx_cooking_interface, release_physx_cooking_private_interface, release_physx_simulation_interface, release_physx_attachment_private_interface, release_physx_benchmarks_interface, release_physx_property_query_interface, release_physx_stage_update_interface, release_physx_statistics_interface
import os, sys
import ctypes

class PhysxExtension(omni.ext.IExt):
    def on_startup(self):
        # Preload libraries plugin depends on to make the discoverable
        if sys.platform == "win32":
            ext_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../.."))
            # PVDRuntime shared library also loaded by carbonite, see extension.toml [[native.library]]
            # we need to load here, too, to ensure it's not unloaded too early
            pvd_dll_path = os.path.join(ext_folder, "bin/PVDRuntime_64.dll")
            ctypes.WinDLL(pvd_dll_path)

        # Init interface cache
        self._physx_interface = get_physx_interface()
        self._physxunittests_interface = get_physxunittests_interface()
        self._physx_visualization_interface = get_physx_visualization_interface()
        self._physx_scene_query_interface = get_physx_scene_query_interface()
        self._physx_cooking_interface = get_physx_cooking_interface()
        self._physx_cooking_private_interface = get_physx_cooking_private_interface()
        self._physx_simulation_interface = get_physx_simulation_interface()
        self._physx_benchmarks_interface = get_physx_benchmarks_interface()
        self._physx_attachment_private_interface = get_physx_attachment_private_interface()
        self._physx_property_query_interface = get_physx_property_query_interface()
        self._physx_stage_update_interface = get_physx_stage_update_interface()
        self._physx_statistics_interface = get_physx_statistics_interface()

    def on_shutdown(self):
        release_physx_statistics_interface(self._physx_statistics_interface)
        self._physx_statistics_interface = None
        release_physx_stage_update_interface(self._physx_stage_update_interface)
        self._physx_stage_update_interface = None
        release_physx_attachment_private_interface(self._physx_attachment_private_interface)
        self._physx_attachment_private_interface = None
        release_physx_benchmarks_interface(self._physx_benchmarks_interface)
        self._physx_benchmarks_interface = None
        release_physx_simulation_interface(self._physx_simulation_interface)
        self._physx_simulation_interface = None
        release_physx_cooking_interface(self._physx_cooking_interface)
        self._physx_cooking_interface = None
        release_physx_cooking_private_interface(self._physx_cooking_private_interface)
        self._physx_cooking_private_interface = None
        release_physx_scene_query_interface(self._physx_scene_query_interface)
        self._physx_scene_query_interface = None
        release_physx_visualization_interface(self._physx_visualization_interface)
        self._physx_visualization_interface = None
        release_physxunittests_interface(self._physxunittests_interface)
        self._physxunittests_interface = None
        release_physx_property_query_interface(self._physx_property_query_interface)
        self._physx_property_query_interface = None
        release_physx_interface(self._physx_interface)
        release_physx_interface_scripting(self._physx_interface) # OM-60917
        self._physx_interface = None
