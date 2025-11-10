# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import omni.kit.commands
from omni.physx.scripts.physicsUtils import set_or_add_translate_op
from pxr import UsdGeom, Sdf, Gf, Vt, PhysicsSchemaTools
import omni.physxdemos as demo
from omni.physx import get_physx_scene_query_interface
from .SceneQueryBaseDemo import createSceneQueryBase
import omni.kit.viewport.utility as vp_utils
from omni.physx.scripts.utils import CameraTransformHelper


class OverlapShapeDemo(demo.Base):
    title = "Overlap Shape"
    category = demo.Categories.SCENE_QUERY
    short_description = "Demo showing overlap shape (UsdGeom.Rprim) usage"
    description = "Demo showing overlap usage. Overlap on a various UsdGeom.Rprim positions (move the rprims (pink) with a gizmo), if it hits a blue box (collision shape) it turns green. Press play (space) to run the simulation."

    def create(self, stage):
        self._stage = stage
        self.defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        createSceneQueryBase(stage, self.defaultPrimPath)
        self._room = demo.get_demo_room(self, stage, hasTable=False, floorOffset=-30.0)

        shapeColor = demo.get_primary_color(1.0)

        result, tmp_path = omni.kit.commands.execute("CreateMeshPrim", prim_type="Torus", select_new_prim=False)
        self._mesh_path = self.defaultPrimPath + "/meshPath"
        omni.kit.commands.execute("MovePrim", path_from=tmp_path, path_to=self._mesh_path)
        torus_prim = stage.GetPrimAtPath(self._mesh_path)
        torus_mesh = UsdGeom.Mesh(torus_prim)
        set_or_add_translate_op(torus_mesh, translate=Gf.Vec3f(0.0, 0.0, 80.0))
        torus_mesh.CreateDisplayColorAttr().Set([shapeColor])

        self._sphere_path = Sdf.Path(self.defaultPrimPath + "/Sphere")
        prim = UsdGeom.Sphere.Define(stage, self._sphere_path)
        prim.CreateRadiusAttr().Set(50)
        prim.AddTranslateOp().Set(Gf.Vec3f(-150.0, 0.0, 80.0))
        prim.CreateDisplayColorAttr().Set([shapeColor])

        self._cube_path = Sdf.Path(self.defaultPrimPath + "/Cube")
        prim = UsdGeom.Cube.Define(stage, self._cube_path)
        prim.CreateSizeAttr().Set(50)
        prim.AddTranslateOp().Set(Gf.Vec3f(-250.0, 0.0, 80.0))
        prim.CreateDisplayColorAttr().Set([shapeColor])

        self._capsule_path = Sdf.Path(self.defaultPrimPath + "/Capsule")
        prim = UsdGeom.Capsule.Define(stage, self._capsule_path)
        prim.CreateRadiusAttr().Set(25)
        prim.CreateHeightAttr().Set(50)
        prim.AddTranslateOp().Set(Gf.Vec3f(150.0, 0.0, 80.0))
        prim.CreateDisplayColorAttr().Set([shapeColor])

        self._cylinder_path = Sdf.Path(self.defaultPrimPath + "/Cylinder")
        prim = UsdGeom.Cylinder.Define(stage, self._cylinder_path)
        prim.CreateRadiusAttr().Set(25)
        prim.CreateHeightAttr().Set(50)
        prim.AddTranslateOp().Set(Gf.Vec3f(250.0, 0.0, 80.0))
        prim.CreateDisplayColorAttr().Set([shapeColor])

        self._cone_path = Sdf.Path(self.defaultPrimPath + "/Cone")
        prim = UsdGeom.Cone.Define(stage, self._cone_path)
        prim.CreateRadiusAttr().Set(25)
        prim.CreateHeightAttr().Set(50)
        prim.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 200.0))
        prim.CreateDisplayColorAttr().Set([shapeColor])

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
        activeCamera = stage.GetPrimAtPath(vp_utils.get_active_viewport().camera_path)
        cameraHelper = CameraTransformHelper(activeCamera)

        cameraPos = cameraHelper.get_pos()
        cameraForward = cameraHelper.get_forward()

        origColor = Vt.Vec3fArray([demo.get_primary_color()])
        usdGeom = UsdGeom.Cube.Get(stage, self.defaultPrimPath + "/boxActor0")
        usdGeom.GetDisplayColorAttr().Set(origColor)
        usdGeom = UsdGeom.Cube.Get(stage, self.defaultPrimPath + "/boxActor1")
        usdGeom.GetDisplayColorAttr().Set(origColor)
        usdGeom = UsdGeom.Cube.Get(stage, self.defaultPrimPath + "/boxActor2")
        usdGeom.GetDisplayColorAttr().Set(origColor)

        origin = carb.Float3(cameraPos[0], cameraPos[1], cameraPos[2])
        forwardDir = carb.Float3(cameraForward[0], cameraForward[1], cameraForward[2])
        origin[0] = origin[0] + forwardDir[0] * 800.0
        origin[1] = origin[1] + forwardDir[1] * 800.0
        origin[2] = origin[2] + forwardDir[2] * 800.0

        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self._mesh_path))
        _ = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], self.report_hit, False)

        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self._sphere_path))
        _ = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], self.report_hit, False)

        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self._cube_path))
        _ = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], self.report_hit, False)

        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self._capsule_path))
        _ = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], self.report_hit, False)

        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self._cylinder_path))
        _ = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], self.report_hit, False)

        path_tuple = PhysicsSchemaTools.encodeSdfPath(Sdf.Path(self._cone_path))
        _ = get_physx_scene_query_interface().overlap_shape(path_tuple[0], path_tuple[1], self.report_hit, False)
