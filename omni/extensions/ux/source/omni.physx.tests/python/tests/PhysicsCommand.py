# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
import omni.physx.scripts.utils
from omni.physxtests import utils
from pxr import Gf, UsdPhysics, UsdGeom, UsdShade, Sdf, Vt, PhysxSchema
from omni.kit.commands import execute
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase, TestCategory


def get_pos(self):
    cubeGeom = UsdGeom.Cube(self._prim)
    for op in cubeGeom.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op.Get()
    return Gf.Vec3d(0)


class PhysicsCommandTestAsync(PhysicsBaseAsyncTestCase):
    category = TestCategory.Kit
    async def base_setup(self):
        self.fail_on_log_error = True
        stage = await utils.new_stage_setup()
        path = "/physicsScene"
        execute("AddPhysicsScene", stage=stage, path=path)
        return stage

    async def test_physics_userpath_add_ground_plane(self):
        stage = await self.base_setup()

        upAxis = UsdGeom.GetStageUpAxis(stage)
        scaleFactor = omni.physx.scripts.utils.getUnitScaleFactor(stage)
        plane_path = "/staticPlaneActor"
        position = Gf.Vec3f(0.0)
        color = Gf.Vec3f(0.5)
        size = 25.0 * scaleFactor
        utils.execute_and_check(self, "AddGroundPlane", stage=stage, planePath=plane_path, axis=upAxis,
                                size=size, position=position, color=color)

        await utils.play_and_step_and_stop(self, 1)

        plane_path = str(stage.GetDefaultPrim().GetPath()) + plane_path
        mesh_inst = UsdGeom.Mesh.Get(stage, plane_path + "/CollisionMesh")
        self.assertTrue(mesh_inst)
        self.assertTrue(mesh_inst.GetDisplayColorAttr().Get() == Vt.Vec3fArray([color]))
        plane_inst = UsdGeom.Plane.Get(stage, plane_path + "/CollisionPlane")
        self.assertTrue(plane_inst)
        self.assertTrue(plane_inst.GetPurposeAttr().Get() == "guide")
        self.assertTrue(plane_inst.GetAxisAttr().Get() == upAxis)
        self.assertTrue(UsdPhysics.CollisionAPI.Get(stage, plane_path + "/CollisionPlane"))

    async def test_physics_userpath_add_rigidbody_material(self):
        stage = await self.base_setup()

        mat_path = "/physicsMaterial"
        utils.execute_and_check(self, "AddRigidBodyMaterial", stage=stage, path=mat_path, staticFriction=0.0, dynamicFriction=0.0,
                                restitution=0.0, density=1000)

        await utils.play_and_step_and_stop(self, 1)

        self.assertTrue(UsdShade.Material.Get(stage, mat_path))
        api_inst = UsdPhysics.MaterialAPI.Get(stage, mat_path)
        self.assertTrue(api_inst)
        self.assertTrue(api_inst.GetStaticFrictionAttr().Get() == 0.0)
        self.assertTrue(api_inst.GetDynamicFrictionAttr().Get() == 0.0)
        self.assertTrue(api_inst.GetRestitutionAttr().Get() == 0.0)
        self.assertTrue(api_inst.GetDensityAttr().Get() == 1000)
    
    async def test_physics_userpath_add_collision_group(self):
        stage = await self.base_setup()

        path = "/collisionGroup"
        utils.execute_and_check(self, "AddCollisionGroup", stage=stage, path=path)

        await utils.play_and_step_and_stop(self, 1)

        self.assertTrue(UsdPhysics.CollisionGroup.Get(stage, path))

    async def test_physics_userpath_add_pair_filter(self):
        stage = await self.base_setup()

        path1 = "/cube1"
        utils.execute_and_check(self, "CreatePrimWithDefaultXform", prim_type="Cube", prim_path=path1)

        path2 = "/cube2"
        utils.execute_and_check(self, "CreatePrimWithDefaultXform", prim_type="Cube", prim_path=path2)

        utils.execute_and_check(self, "AddPairFilter", stage=stage, primPaths=[path1, path2])

        await utils.play_and_step_and_stop(self, 1)

        api_inst = UsdPhysics.FilteredPairsAPI.Get(stage, path1)
        self.assertTrue(api_inst)
        self.assertTrue(api_inst.GetFilteredPairsRel().GetTargets() == [Sdf.Path(path2)])
