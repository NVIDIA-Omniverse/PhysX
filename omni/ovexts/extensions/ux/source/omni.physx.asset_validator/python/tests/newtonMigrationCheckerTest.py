# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .utils import BaseValidatorTest, ExpectedResult
from ..scripts.newtonMigrationChecker import NewtonMigrationChecker, NewtonMimicMigrationChecker
from pxr import Usd, UsdPhysics, PhysxSchema, Gf, Sdf


class NewtonMigrationCheckerTestCase(BaseValidatorTest):
    rules = [NewtonMigrationChecker]

    async def setUp(self):
        await super().setUp()
        self._stage = self.create_stage_in_memory()
        self.create_validation_engine()

    # ---- Scene: timeStepsPerSecond ----

    def test_deprecated_scene_time_steps_detected(self):
        """Authored physxScene:timeStepsPerSecond should be flagged as deprecated."""
        prim = self._stage.DefinePrim(Sdf.Path("/Scene"), "PhysicsScene")
        api = PhysxSchema.PhysxSceneAPI.Apply(prim)
        api.CreateTimeStepsPerSecondAttr().Set(120)

        self.run_validation_and_assert_result(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_SCENE_ATTR_CODE)
        )

    def test_scene_time_steps_fix_migrates_to_newton(self):
        """Fix should migrate physxScene:timeStepsPerSecond to newton:timeStepsPerSecond."""
        prim = self._stage.DefinePrim(Sdf.Path("/Scene"), "PhysicsScene")
        api = PhysxSchema.PhysxSceneAPI.Apply(prim)
        api.CreateTimeStepsPerSecondAttr().Set(120)

        self.validate_and_check_and_fix(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_SCENE_ATTR_CODE)
        )

        # After fix: Newton attr should have the value, PhysX attr should be gone
        newton_attr = prim.GetAttribute("newton:timeStepsPerSecond")
        self.assertTrue(newton_attr)
        self.assertEqual(newton_attr.Get(), 120)

        physx_attr = prim.GetAttribute("physxScene:timeStepsPerSecond")
        self.assertFalse(physx_attr.HasAuthoredValue())

    def test_scene_no_authored_time_steps_passes(self):
        """PhysxSceneAPI without authored timeStepsPerSecond should pass."""
        prim = self._stage.DefinePrim(Sdf.Path("/Scene"), "PhysicsScene")
        PhysxSchema.PhysxSceneAPI.Apply(prim)
        # Don't author timeStepsPerSecond

        self.run_validation_and_assert_result(None)

    def test_scene_no_physx_api_passes(self):
        """Scene without PhysxSceneAPI should pass."""
        self._stage.DefinePrim(Sdf.Path("/Scene"), "PhysicsScene")

        self.run_validation_and_assert_result(None)

    # ---- Collision: contactOffset / restOffset ----

    def test_deprecated_collision_contact_offset_detected(self):
        """Authored physxCollision:contactOffset should be flagged."""
        prim = self._stage.DefinePrim(Sdf.Path("/Box"), "Cube")
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr().Set(2.0)

        self.run_validation_and_assert_result(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_COLLISION_ATTR_CODE)
        )

    def test_deprecated_collision_rest_offset_detected(self):
        """Authored physxCollision:restOffset should be flagged."""
        prim = self._stage.DefinePrim(Sdf.Path("/Box"), "Cube")
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateRestOffsetAttr().Set(0.5)

        self.run_validation_and_assert_result(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_COLLISION_ATTR_CODE)
        )

    def test_both_collision_attrs_detected(self):
        """Both contactOffset and restOffset authored should produce 2 issues."""
        prim = self._stage.DefinePrim(Sdf.Path("/Box"), "Cube")
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr().Set(2.0)
        api.CreateRestOffsetAttr().Set(0.5)

        self.run_validation_and_assert_result(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_COLLISION_ATTR_CODE, issues_num=2)
        )

    def test_collision_contact_offset_fix_migrates_to_newton(self):
        """Fix should migrate physxCollision:contactOffset to newton:contactGap.
        gap = contactOffset - restOffset. With no restOffset authored, restOffset=0."""
        prim = self._stage.DefinePrim(Sdf.Path("/Box"), "Cube")
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr().Set(2.0)

        self.validate_and_check_and_fix(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_COLLISION_ATTR_CODE)
        )

        # gap = contactOffset(2.0) - restOffset(0.0) = 2.0
        newton_attr = prim.GetAttribute("newton:contactGap")
        self.assertTrue(newton_attr)
        self.assertAlmostEqual(newton_attr.Get(), 2.0, places=5)

        physx_attr = prim.GetAttribute("physxCollision:contactOffset")
        self.assertFalse(physx_attr.HasAuthoredValue())

    def test_collision_rest_offset_fix_migrates_to_newton(self):
        """Fix should migrate physxCollision:restOffset to newton:contactMargin."""
        prim = self._stage.DefinePrim(Sdf.Path("/Box"), "Cube")
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateRestOffsetAttr().Set(0.5)

        self.validate_and_check_and_fix(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_COLLISION_ATTR_CODE)
        )

        newton_attr = prim.GetAttribute("newton:contactMargin")
        self.assertTrue(newton_attr)
        self.assertAlmostEqual(newton_attr.Get(), 0.5, places=5)

        physx_attr = prim.GetAttribute("physxCollision:restOffset")
        self.assertFalse(physx_attr.HasAuthoredValue())

    def test_collision_both_offsets_fix_computes_gap(self):
        """When both contactOffset and restOffset are authored, gap = contactOffset - restOffset."""
        prim = self._stage.DefinePrim(Sdf.Path("/Box"), "Cube")
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr().Set(2.0)
        api.CreateRestOffsetAttr().Set(0.5)

        self.validate_and_check_and_fix(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_COLLISION_ATTR_CODE, issues_num=2)
        )

        # margin = restOffset = 0.5
        margin_attr = prim.GetAttribute("newton:contactMargin")
        self.assertTrue(margin_attr)
        self.assertAlmostEqual(margin_attr.Get(), 0.5, places=5)

        # gap = contactOffset(2.0) - restOffset(0.5) = 1.5
        gap_attr = prim.GetAttribute("newton:contactGap")
        self.assertTrue(gap_attr)
        self.assertAlmostEqual(gap_attr.Get(), 1.5, places=5)

        # Both PhysX attrs should be removed
        self.assertFalse(prim.GetAttribute("physxCollision:contactOffset").HasAuthoredValue())
        self.assertFalse(prim.GetAttribute("physxCollision:restOffset").HasAuthoredValue())

    def test_collision_no_authored_offsets_passes(self):
        """PhysxCollisionAPI without authored offsets should pass."""
        prim = self._stage.DefinePrim(Sdf.Path("/Box"), "Cube")
        UsdPhysics.CollisionAPI.Apply(prim)
        PhysxSchema.PhysxCollisionAPI.Apply(prim)

        self.run_validation_and_assert_result(None)

    # ---- Articulation: enabledSelfCollisions ----

    def test_deprecated_articulation_self_collisions_detected(self):
        """Authored physxArticulation:enabledSelfCollisions should be flagged."""
        prim = self._stage.DefinePrim(Sdf.Path("/Articulation"))
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        api.CreateEnabledSelfCollisionsAttr().Set(False)

        self.run_validation_and_assert_result(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_ARTICULATION_ATTR_CODE)
        )

    def test_articulation_self_collisions_fix_migrates_to_newton(self):
        """Fix should migrate physxArticulation:enabledSelfCollisions to newton:selfCollisionEnabled."""
        prim = self._stage.DefinePrim(Sdf.Path("/Articulation"))
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        api.CreateEnabledSelfCollisionsAttr().Set(False)

        self.validate_and_check_and_fix(
            ExpectedResult(code=NewtonMigrationChecker.DEPRECATED_ARTICULATION_ATTR_CODE)
        )

        newton_attr = prim.GetAttribute("newton:selfCollisionEnabled")
        self.assertTrue(newton_attr)
        self.assertEqual(newton_attr.Get(), False)

        physx_attr = prim.GetAttribute("physxArticulation:enabledSelfCollisions")
        self.assertFalse(physx_attr.HasAuthoredValue())

    def test_articulation_no_authored_self_collisions_passes(self):
        """PhysxArticulationAPI without authored enabledSelfCollisions should pass."""
        prim = self._stage.DefinePrim(Sdf.Path("/Articulation"))
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        PhysxSchema.PhysxArticulationAPI.Apply(prim)

        self.run_validation_and_assert_result(None)

    # ---- Mixed: multiple deprecated attrs on different prims ----

    def test_multiple_prims_with_deprecated_attrs(self):
        """Multiple prims with different deprecated attrs should all be detected."""
        scene = self._stage.DefinePrim(Sdf.Path("/Scene"), "PhysicsScene")
        scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene)
        scene_api.CreateTimeStepsPerSecondAttr().Set(120)

        box = self._stage.DefinePrim(Sdf.Path("/Box"), "Cube")
        UsdPhysics.CollisionAPI.Apply(box)
        col_api = PhysxSchema.PhysxCollisionAPI.Apply(box)
        col_api.CreateContactOffsetAttr().Set(2.0)

        art = self._stage.DefinePrim(Sdf.Path("/Articulation"))
        UsdPhysics.ArticulationRootAPI.Apply(art)
        art_api = PhysxSchema.PhysxArticulationAPI.Apply(art)
        art_api.CreateEnabledSelfCollisionsAttr().Set(False)

        # 3 prims, each with 1 deprecated attr = 3 issues total
        self.run_validation_and_assert_result(ExpectedResult(issues_num=3))


class NewtonMimicMigrationCheckerTestCase(BaseValidatorTest):
    rules = [NewtonMimicMigrationChecker]

    async def setUp(self):
        await super().setUp()
        self._stage = self.create_stage_in_memory()
        self.create_validation_engine()

    def _make_prismatic(self, stage, path):
        joint = UsdPhysics.PrismaticJoint.Define(stage, Sdf.Path(path))
        joint.CreateAxisAttr("X")
        joint.CreateLowerLimitAttr(-10.0)
        joint.CreateUpperLimitAttr(10.0)
        return joint

    def _make_revolute(self, stage, path):
        joint = UsdPhysics.RevoluteJoint.Define(stage, Sdf.Path(path))
        joint.CreateAxisAttr("X")
        joint.CreateLowerLimitAttr(-90.0)
        joint.CreateUpperLimitAttr(90.0)
        return joint

    def _apply_mimic(self, follower_joint, leader_path, gearing=1.0, offset=0.0,
                     instance=UsdPhysics.Tokens.rotX):
        api = PhysxSchema.PhysxMimicJointAPI.Apply(follower_joint.GetPrim(), instance)
        api.CreateReferenceJointRel().SetTargets([Sdf.Path(leader_path)])
        api.CreateReferenceJointAxisAttr().Set(instance)
        api.CreateGearingAttr().Set(gearing)
        api.CreateOffsetAttr().Set(offset)
        return api

    # ---- Positive cases: single-DOF follower + single-DOF leader -----------

    def test_prismatic_follower_prismatic_leader_detected(self):
        leader = self._make_prismatic(self._stage, "/Leader")
        follower = self._make_prismatic(self._stage, "/Follower")
        self._apply_mimic(follower, "/Leader", gearing=1.5, offset=0.05)

        self.run_validation_and_assert_result(
            ExpectedResult(code=NewtonMimicMigrationChecker.DEPRECATED_MIMIC_API_CODE)
        )

    def test_revolute_follower_revolute_leader_detected(self):
        leader = self._make_revolute(self._stage, "/Leader")
        follower = self._make_revolute(self._stage, "/Follower")
        self._apply_mimic(follower, "/Leader", gearing=2.0, offset=10.0)

        self.run_validation_and_assert_result(
            ExpectedResult(code=NewtonMimicMigrationChecker.DEPRECATED_MIMIC_API_CODE)
        )

    def test_prismatic_fix_migrates_to_newton(self):
        leader = self._make_prismatic(self._stage, "/Leader")
        follower = self._make_prismatic(self._stage, "/Follower")
        api = self._apply_mimic(follower, "/Leader", gearing=1.5, offset=0.05)
        # Author the remaining PhysxMimicJointAPI attributes to verify the fix
        # strips every namespaced property, not just gearing / offset.
        api.CreateNaturalFrequencyAttr().Set(5.0)
        api.CreateDampingRatioAttr().Set(0.5)

        self.validate_and_check_and_fix(
            ExpectedResult(code=NewtonMimicMigrationChecker.DEPRECATED_MIMIC_API_CODE)
        )

        follower_prim = follower.GetPrim()

        # Newton schema applied with sign-flipped coefficients.
        self.assertTrue("NewtonMimicAPI" in follower_prim.GetAppliedSchemas())
        self.assertEqual(follower_prim.GetAttribute("newton:mimicEnabled").Get(), True)
        self.assertAlmostEqual(follower_prim.GetAttribute("newton:mimicCoef1").Get(), -1.5, places=5)
        self.assertAlmostEqual(follower_prim.GetAttribute("newton:mimicCoef0").Get(), -0.05, places=5)
        rel_targets = follower_prim.GetRelationship("newton:mimicJoint").GetTargets()
        self.assertEqual([str(p) for p in rel_targets], ["/Leader"])

        # PhysX schema and its namespaced properties are gone.
        self.assertFalse(any(s.startswith("PhysxMimicJointAPI:")
                             for s in follower_prim.GetAppliedSchemas()))
        self.assertFalse(follower_prim.GetAttribute("physxMimicJoint:rotX:gearing").HasAuthoredValue())
        self.assertFalse(follower_prim.GetAttribute("physxMimicJoint:rotX:offset").HasAuthoredValue())
        self.assertFalse(follower_prim.GetAttribute("physxMimicJoint:rotX:naturalFrequency").HasAuthoredValue())
        self.assertFalse(follower_prim.GetAttribute("physxMimicJoint:rotX:dampingRatio").HasAuthoredValue())

    def test_revolute_fix_preserves_defaults_when_not_authored(self):
        """Unauthored gearing/offset fall back to PhysX defaults (1.0 / 0.0)."""
        leader = self._make_revolute(self._stage, "/Leader")
        follower = self._make_revolute(self._stage, "/Follower")
        api = PhysxSchema.PhysxMimicJointAPI.Apply(follower.GetPrim(), UsdPhysics.Tokens.rotX)
        api.CreateReferenceJointRel().SetTargets([Sdf.Path("/Leader")])
        # gearing and offset deliberately not authored.

        self.validate_and_check_and_fix(
            ExpectedResult(code=NewtonMimicMigrationChecker.DEPRECATED_MIMIC_API_CODE)
        )

        follower_prim = follower.GetPrim()
        self.assertAlmostEqual(follower_prim.GetAttribute("newton:mimicCoef1").Get(), -1.0, places=5)
        self.assertAlmostEqual(follower_prim.GetAttribute("newton:mimicCoef0").Get(), 0.0, places=5)

    # ---- Negative cases: should be silently skipped ------------------------

    def test_multi_instance_not_migrated(self):
        """Joints with multiple PhysxMimicJointAPI instances are skipped."""
        leader = self._make_prismatic(self._stage, "/Leader")
        follower_prim = self._stage.DefinePrim(Sdf.Path("/Follower"), "PhysicsJoint")
        apiX = PhysxSchema.PhysxMimicJointAPI.Apply(follower_prim, UsdPhysics.Tokens.rotX)
        apiX.CreateReferenceJointRel().SetTargets([Sdf.Path("/Leader")])
        apiY = PhysxSchema.PhysxMimicJointAPI.Apply(follower_prim, UsdPhysics.Tokens.rotY)
        apiY.CreateReferenceJointRel().SetTargets([Sdf.Path("/Leader")])

        self.run_validation_and_assert_result(None)

    def test_multi_dof_follower_not_migrated(self):
        """A D6 follower joint is multi-DOF and cannot be migrated."""
        leader = self._make_prismatic(self._stage, "/Leader")
        follower = UsdPhysics.Joint.Define(self._stage, Sdf.Path("/Follower"))
        self._apply_mimic(follower, "/Leader")

        self.run_validation_and_assert_result(None)

    def test_multi_dof_leader_not_migrated(self):
        """A D6 leader joint is multi-DOF and cannot be migrated."""
        leader = UsdPhysics.Joint.Define(self._stage, Sdf.Path("/Leader"))
        follower = self._make_prismatic(self._stage, "/Follower")
        self._apply_mimic(follower, "/Leader")

        self.run_validation_and_assert_result(None)

    def test_no_mimic_api_passes(self):
        """Plain single-DOF joint without PhysxMimicJointAPI should pass."""
        self._make_prismatic(self._stage, "/Plain")

        self.run_validation_and_assert_result(None)
