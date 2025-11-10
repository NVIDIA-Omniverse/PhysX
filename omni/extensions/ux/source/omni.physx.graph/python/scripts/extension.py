# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
import omni.physx
import carb
from omni.physxgraph.bindings import _physxGraph
import omni.physxdemos as demo


DEMO_SCENES = "omni.physxgraph.scripts.samples"

class PhysxGraphExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()
        self._instance_name = "Graph"
        self._physxGraphInterface = None
        return

    def on_startup(self):
        self._physxGraphInterface = _physxGraph.acquire_physx_graph_interface()
        demo.register(DEMO_SCENES)

    def on_shutdown(self):
        if self._physxGraphInterface is not None:
            _physxGraph.release_physx_graph_interface(self._physxGraphInterface)
            _physxGraph.release_physx_graph_interface_scripting(self._physxGraphInterface) # OM-60917
            self._physxGraphInterface = None

        demo.unregister(DEMO_SCENES)
