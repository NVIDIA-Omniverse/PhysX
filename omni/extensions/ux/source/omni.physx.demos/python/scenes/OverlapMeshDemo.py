# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.commands
from omni.physx.scripts.physicsUtils import set_or_add_translate_op
from pxr import UsdGeom, Sdf, Gf, Vt, PhysicsSchemaTools
import omni.physxdemos as demo
from omni.physx import get_physx_scene_query_interface
from .SceneQueryBaseDemo import createSceneQueryBase


class OverlapMeshDemo(demo.Base):
    title = "Overlap Mesh"
    category = demo.Categories.SCENE_QUERY
    short_description = "Demo showing overlap torus mesh usage"
    description = "Demo showing overlap usage. Overlap on a torus position (move the torus with a gizmo), if it hits a box it turns green. Press play (space) to run the simulation."

    def create(self, stage):
        self._stage = stage
        self.defaultPrimPath, scene = demo.setup_physics_scene(self, stage, gravityMod=0.0)
        createSceneQueryBase(stage, self.defaultPrimPath)

        self._room = demo.get_demo_room(self, stage, hasTable=False, floorOffset=-30.0)

        result, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Torus", select_new_prim=False)
        self._path = self.defaultPrimPath + "/meshPath"
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=self._path)

        torus_mesh = UsdGeom.Mesh.Get(stage, self._path)
        set_or_add_translate_op(torus_mesh, translate=Gf.Vec3f(0.0, 0.0, 50.0))
        torus_mesh.CreateDisplayColorAttr().Set([demo.get_primary_color(1.0)])

    def report_hit(self, hit):
        hitColor = Vt.Vec3fArray([demo.get_hit_color()])
        # check if the hit object is one of the three rigid cubes
        # that were created in createSceneQueryBase
        # If the hit is a cube, update its color
        usdGeom = UsdGeom.Cube.Get(self._stage, hit.rigid_body)
        if usdGeom:
            usdGeom.GetDisplayColorAttr().Set(hitColor)
        return True

    def update(self, stage, dt, viewport, physxIFace):
        self._stage = stage

        origColor = Vt.Vec3fArray([demo.get_primary_color()])
        usdGeom = UsdGeom.Cube.Get(stage, self.defaultPrimPath + "/boxActor0")
        usdGeom.GetDisplayColorAttr().Set(origColor)
        usdGeom = UsdGeom.Cube.Get(stage, self.defaultPrimPath + "/boxActor1")
        usdGeom.GetDisplayColorAttr().Set(origColor)
        usdGeom = UsdGeom.Cube.Get(stage, self.defaultPrimPath + "/boxActor2")
        usdGeom.GetDisplayColorAttr().Set(origColor)

        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self._path))

        _ = get_physx_scene_query_interface().overlap_mesh(path_tuple[0], path_tuple[1], self.report_hit, False)
