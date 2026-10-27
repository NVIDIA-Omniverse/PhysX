# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import omni.kit.test
import omni.kit.undo
import omni.physx.scripts.utils
import omni.physx.scripts.utils as physxUtils
from omni.physx.scripts import physicsUtils
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


class PhysicsJointCreateCommandTestAsync(PhysicsBaseAsyncTestCase):
    """
    Locks in the OMPE-49347 contract that CreateJoint{,s}Command produces the
    same default limits as the property-panel '+ Add Component' flow.
    """
    category = TestCategory.Kit

    async def _two_boxes(self):
        stage = await utils.new_stage_setup()
        b0 = physicsUtils.add_rigid_box(stage, "/b0", Gf.Vec3f(1.0), Gf.Vec3f(0.0))
        b1 = physicsUtils.add_rigid_box(stage, "/b1", Gf.Vec3f(1.0), Gf.Vec3f(0.0, 0.0, 5.0))
        return stage, b0.GetPrim(), b1.GetPrim()

    async def _create(self, joint_type):
        stage, p0, p1 = await self._two_boxes()
        ret, prim = execute("CreateJoint", stage=stage, joint_type=joint_type,
                            from_prim=p0, to_prim=p1)
        self.assertTrue(ret)
        self.assertTrue(prim and prim.IsValid())
        return stage, prim

    async def test_create_revolute_default_limits(self):
        _, prim = await self._create("Revolute")
        rev = UsdPhysics.RevoluteJoint(prim)
        self.assertEqual(rev.GetLowerLimitAttr().Get(), -90.0)
        self.assertEqual(rev.GetUpperLimitAttr().Get(),  90.0)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxLimitAPI, "angular"))

    async def test_create_prismatic_default_limits(self):
        _, prim = await self._create("Prismatic")
        pris = UsdPhysics.PrismaticJoint(prim)
        self.assertEqual(pris.GetLowerLimitAttr().Get(), -1.0)
        self.assertEqual(pris.GetUpperLimitAttr().Get(),  1.0)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxLimitAPI, "linear"))

    async def test_create_spherical_default_cone_in_degrees(self):
        _, prim = await self._create("Spherical")
        sph = UsdPhysics.SphericalJoint(prim)
        self.assertEqual(sph.GetConeAngle0LimitAttr().Get(), 90.0)
        self.assertEqual(sph.GetConeAngle1LimitAttr().Get(), 90.0)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxLimitAPI, "cone"))

    async def test_create_distance_authors_no_limit_attrs(self):
        _, prim = await self._create("Distance")
        self.assertFalse(prim.GetAttribute("physics:minDistance").IsAuthored())
        self.assertFalse(prim.GetAttribute("physics:maxDistance").IsAuthored())

    async def test_create_fixed_authors_no_limits(self):
        _, prim = await self._create("Fixed")
        for inst in ("angular", "linear", "cone", "distance"):
            self.assertFalse(prim.HasAPI(PhysxSchema.PhysxLimitAPI, inst))
        self.assertFalse(prim.HasAttribute("physics:lowerLimit"))
        self.assertFalse(prim.HasAttribute("physics:upperLimit"))

    async def test_create_d6_keeps_six_free_axes(self):
        _, prim = await self._create("")  # blank -> D6 branch in createJoint
        for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
            self.assertTrue(prim.HasAPI(UsdPhysics.LimitAPI, axis))
            api = UsdPhysics.LimitAPI.Get(prim, axis)
            self.assertEqual(api.GetLowAttr().Get(),  1.0)
            self.assertEqual(api.GetHighAttr().Get(), -1.0)

    async def test_create_joint_single_undo_removes_prim_and_limits(self):
        stage, prim = await self._create("Revolute")
        path = prim.GetPath().pathString
        self.assertTrue(stage.GetPrimAtPath(path).IsValid())
        omni.kit.undo.undo()
        self.assertFalse(stage.GetPrimAtPath(path).IsValid())

    async def test_create_joints_spherical_applies_default_cone(self):
        stage, _, p1 = await self._two_boxes()
        ret, prims = execute("CreateJoints", stage=stage, joint_type="Spherical",
                             paths=[p1.GetPath().pathString], join_to_parent=False)
        self.assertTrue(ret)
        self.assertEqual(len(prims), 1)
        sph = UsdPhysics.SphericalJoint(prims[0])
        self.assertEqual(sph.GetConeAngle0LimitAttr().Get(), 90.0)
        self.assertEqual(sph.GetConeAngle1LimitAttr().Get(), 90.0)
        self.assertTrue(prims[0].HasAPI(PhysxSchema.PhysxLimitAPI, "cone"))


class PhysicsJointAddCommandTestAsync(PhysicsBaseAsyncTestCase):
    """Regression test for the Spherical cone-angle units bug fix in
    AddSphericalPhysicsJointComponent (OMPE-49347).
    Pre-fix value was 1.57 stored as degrees (near-locked cone); post-fix
    is 90 degrees, matching the PhysX SDK pi/2 rad default.
    """
    category = TestCategory.Kit

    async def test_add_spherical_limit_writes_90_degrees(self):
        stage = await utils.new_stage_setup()
        b0 = physicsUtils.add_rigid_box(stage, "/b0", Gf.Vec3f(1.0), Gf.Vec3f(0.0))
        b1 = physicsUtils.add_rigid_box(stage, "/b1", Gf.Vec3f(1.0), Gf.Vec3f(0.0, 0.0, 5.0))

        joint_prim = physxUtils.createJoint(stage, "Spherical", b0.GetPrim(), b1.GetPrim())
        self.assertFalse(joint_prim.GetAttribute("physics:coneAngle0Limit").IsAuthored())
        self.assertFalse(joint_prim.GetAttribute("physics:coneAngle1Limit").IsAuthored())

        ret, _ = execute("AddSphericalPhysicsJointComponent",
                         usd_prim=joint_prim, component="PhysicsLimit")
        self.assertTrue(ret)

        sph = UsdPhysics.SphericalJoint(joint_prim)
        self.assertEqual(sph.GetConeAngle0LimitAttr().Get(), 90.0)
        self.assertEqual(sph.GetConeAngle1LimitAttr().Get(), 90.0)
        self.assertTrue(joint_prim.HasAPI(PhysxSchema.PhysxLimitAPI, "cone"))
