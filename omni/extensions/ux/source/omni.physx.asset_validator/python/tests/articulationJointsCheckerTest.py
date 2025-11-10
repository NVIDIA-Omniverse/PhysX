# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .utils import BaseValidatorTest, ExpectedResult
from omni.physx.scripts import physicsUtils
from pxr import Usd, PhysxSchema, UsdPhysics, Gf, Sdf
from ..scripts.articulationJointsChecker import ArticulationJointsChecker

class ArticulationJointsCheckerTestCase(BaseValidatorTest):

    rules = [ArticulationJointsChecker]

    async def test_articulation_joints_checker(self):
        self._stage = await self.new_stage()
        self.set_stage(self._stage)
        self.create_validation_engine()

        actor_prims = [physicsUtils.add_rigid_box(self._stage, "/rigid_body_root")]
        fixed_joint = physicsUtils.add_joint_fixed(self._stage, "/fixed_joint", "", "/rigid_body_root", Gf.Vec3f(), Gf.Quatf(), Gf.Vec3f(), Gf.Quatf(), 1e20, 1e20)
        UsdPhysics.ArticulationRootAPI.Apply(fixed_joint.GetPrim())

        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(fixed_joint.GetPrim())
        articulation_api.CreateSleepThresholdAttr(0.0)
        articulation_api.CreateEnabledSelfCollisionsAttr().Set(False)

        def add_joint_revolute(
            jointPath: str | Sdf.Path, 
            actor0: str | Sdf.Path, 
            actor1: str | Sdf.Path, 
            localPos0: Gf.Vec3f, 
            localRot0: Gf.Quatf, 
            localPos1: Gf.Vec3f, 
            localRot1: Gf.Quatf,
            axis: str | Usd.Tokens,
            limit_lower: float,
            limit_upper: float,
        ) -> UsdPhysics.RevoluteJoint:
            revolute_joint = UsdPhysics.RevoluteJoint.Define(self._stage, jointPath)

            revolute_joint.CreateAxisAttr(axis)
            revolute_joint.CreateLowerLimitAttr(limit_lower)
            revolute_joint.CreateUpperLimitAttr(limit_upper)

            if actor0:
                revolute_joint.CreateBody0Rel().SetTargets([actor0])
            if actor1:
                revolute_joint.CreateBody1Rel().SetTargets([actor1])

            revolute_joint.CreateLocalPos0Attr().Set(localPos0)
            revolute_joint.CreateLocalRot0Attr().Set(localRot0)

            revolute_joint.CreateLocalPos1Attr().Set(localPos1)
            revolute_joint.CreateLocalRot1Attr().Set(localRot1)

            return revolute_joint


        for n in range(4):
            actor_prims.append(physicsUtils.add_rigid_box(self._stage, f"/rigid_body_{n}", position=Gf.Vec3f(5.0 * n, 0.0, 0.0)))
    
        revolute_joints = []
        physx_limit_apis = []
        for n in range(4):
            revolute_joints.append(add_joint_revolute(f"/revolute_joint_{n}", actor_prims[n].GetPath(), actor_prims[n + 1].GetPath(),
                                        Gf.Vec3f(), Gf.Quatf(), Gf.Vec3f(0.0, -5.0, 0.0), Gf.Quatf(), "X", -20.0, 20.0))
            physx_limit_apis.append(PhysxSchema.PhysxLimitAPI.Apply(revolute_joints[-1].GetPrim(), UsdPhysics.Tokens.angular))

        physx_limit_apis[0].CreateBounceThresholdAttr().Set(10.0)
        physx_limit_apis[1].CreateDampingAttr().Set(10.0)
        physx_limit_apis[2].CreateRestitutionAttr().Set(10.0)
        physx_limit_apis[3].CreateStiffnessAttr().Set(10.0)

        self.validate_and_check_and_fix(ExpectedResult(code=ArticulationJointsChecker.ARTICULATION_JOINT_UNSUPPORTED_ATTRIBUTE_CODE, expect_suggestion=True, issues_num=4))
