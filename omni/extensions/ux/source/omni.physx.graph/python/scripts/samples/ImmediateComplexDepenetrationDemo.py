# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import os
import omni.physxdemos as demo
from .BasicSetup import get_usd_asset_path
from pxr import Gf, Sdf, UsdGeom, PhysxSchema, UsdPhysics, UsdLux

class OmniGraphImmediateComplexDepenetrationDemo(demo.Base):
    title = "Objects de-penetration"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "OmniGraph demo using immediate physics nodes to depenetrate objects."
    description = ("""Demo showcasing how to use the Immediate Nodes:
 - Compute Geometry Bounds
 - Compute Bounds Overlap
 - Compute Geometry Penetration
 - Generate Geometry Contacts

Immediate nodes do not need simulation running or USD physics in order to be used.
On Omnigraph play, the objects on the first shelf will be de-penetrated using "Generate Geometry Contacts" node.
The objects on the second shelf will be de-penetrated using "Generate Geometry Contacts" node."""
    )
    
    def create(self, stage):
        default_prim_path, scene = demo.setup_physics_scene(self, stage, primPathAsString = False)

        main_file_path = get_usd_asset_path("ogn_immediate_depenetration_complex.usd")
        main_path = default_prim_path.AppendChild("main")
        assert (stage.DefinePrim(main_path).GetReferences().AddReference(main_file_path) )

        basePath = str(default_prim_path) + "/main"
        room = demo.get_demo_room(self, stage, zoom = 0.4, camElevation = 100.0, camPitch = 0.4, hasTable = False, pathsToHide = [
            # basePath + "/SphereLight", # this light should be removed in the future since the room scene and museum demo are supposed to provide default lighting
            basePath + "/Camera"
        ])
