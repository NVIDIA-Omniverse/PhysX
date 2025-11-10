# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .utils import BaseValidatorTest, ExpectedResult
from ..scripts.apiConflictChecker import APIConflictChecker
from pxr import UsdPhysics, PhysxSchema


class APIConflictCheckerTestCase(BaseValidatorTest):
    rules = [APIConflictChecker]

    async def setUp(self):
        await super().setUp()
        self._stage = self.create_stage_in_memory()
        self.create_validation_engine()

    def test_rigidbody_api_conflict(self):
        """Test that RigidBodyAPI conflicts are detected"""
        # Create a prim with RigidBodyAPI
        prim = self._stage.DefinePrim("/RigidBodyPrim")
        UsdPhysics.RigidBodyAPI.Apply(prim)

        # Apply a conflicting API (PhysxDeformableBodyAPI)
        PhysxSchema.PhysxDeformableBodyAPI.Apply(prim)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.RB_API_CONFLICT_CODE))

    def test_collision_api_conflict(self):
        """Test that CollisionAPI conflicts are detected"""
        # Create a prim with CollisionAPI
        prim = self._stage.DefinePrim("/CollisionPrim")
        UsdPhysics.CollisionAPI.Apply(prim)

        # Apply a conflicting API (PhysxParticleSetAPI)
        PhysxSchema.PhysxParticleSetAPI.Apply(prim)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.COLLISION_API_CONFLICT_CODE))

    def test_articulation_root_api_conflict(self):
        """Test that ArticulationRootAPI conflicts are detected"""
        # Create a prim with ArticulationRootAPI
        prim = self._stage.DefinePrim("/ArticulationRootPrim")
        UsdPhysics.ArticulationRootAPI.Apply(prim)

        # Apply a conflicting API (PhysxDeformableSurfaceAPI)
        PhysxSchema.PhysxDeformableSurfaceAPI.Apply(prim)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.ARTICULATION_ROOT_API_CONFLICT_CODE))

    def test_no_api_conflict(self):
        """Test that non-conflicting APIs pass validation"""
        # Create a prim with RigidBodyAPI but no conflicting APIs
        prim = self._stage.DefinePrim("/NonConflictingPrim")
        UsdPhysics.RigidBodyAPI.Apply(prim)

        # Run validation and expect no errors
        self.run_validation_and_assert_result(None)

    # Ancestor-descendant tests for RigidBodyAPI
    def test_rigidbody_api_ancestor_conflict(self):
        """Test that RigidBodyAPI conflicts with ancestor APIs are detected"""
        # Create parent prim with a conflicting API
        parent = self._stage.DefinePrim("/RigidBodyAncestorTest")
        PhysxSchema.PhysxDeformableBodyAPI.Apply(parent)

        # Create child prim with RigidBodyAPI
        child = self._stage.DefinePrim("/RigidBodyAncestorTest/ChildPrim")
        UsdPhysics.RigidBodyAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.RB_API_CONFLICT_CODE))

    def test_rigidbody_api_descendant_conflict(self):
        """Test that RigidBodyAPI conflicts with descendant APIs are detected"""
        # Create parent prim with RigidBodyAPI
        parent = self._stage.DefinePrim("/RigidBodyDescendantTest")
        UsdPhysics.RigidBodyAPI.Apply(parent)

        # Create child prim with a conflicting API
        child = self._stage.DefinePrim("/RigidBodyDescendantTest/ChildPrim")
        PhysxSchema.PhysxParticleSetAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.RB_API_CONFLICT_CODE))

    # Ancestor-descendant tests for CollisionAPI
    def test_collision_api_ancestor_conflict(self):
        """Test that CollisionAPI conflicts with ancestor APIs are detected"""
        # Create parent prim with a conflicting API
        parent = self._stage.DefinePrim("/CollisionAncestorTest")
        PhysxSchema.PhysxDeformableSurfaceAPI.Apply(parent)

        # Create child prim with CollisionAPI
        child = self._stage.DefinePrim("/CollisionAncestorTest/ChildPrim")
        UsdPhysics.CollisionAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.COLLISION_API_CONFLICT_CODE))

    def test_collision_api_descendant_conflict(self):
        """Test that CollisionAPI conflicts with descendant APIs are detected"""
        # Create parent prim with CollisionAPI
        parent = self._stage.DefinePrim("/CollisionDescendantTest")
        UsdPhysics.CollisionAPI.Apply(parent)

        # Create child prim with a conflicting API
        child = self._stage.DefinePrim("/CollisionDescendantTest/ChildPrim")
        PhysxSchema.PhysxParticleClothAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.COLLISION_API_CONFLICT_CODE))

    # Ancestor-descendant tests for ArticulationRootAPI
    def test_articulation_root_api_ancestor_conflict(self):
        """Test that ArticulationRootAPI conflicts with ancestor APIs are detected"""
        # Create parent prim with a conflicting API
        parent = self._stage.DefinePrim("/ArticulationAncestorTest")
        PhysxSchema.PhysxDeformableBodyAPI.Apply(parent)

        # Create child prim with ArticulationRootAPI
        child = self._stage.DefinePrim("/ArticulationAncestorTest/ChildPrim")
        UsdPhysics.ArticulationRootAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.ARTICULATION_ROOT_API_CONFLICT_CODE))

    def test_articulation_root_api_descendant_conflict(self):
        """Test that ArticulationRootAPI conflicts with descendant APIs are detected"""
        # Create parent prim with ArticulationRootAPI
        parent = self._stage.DefinePrim("/ArticulationDescendantTest")
        UsdPhysics.ArticulationRootAPI.Apply(parent)

        # Create child prim with a conflicting API
        child = self._stage.DefinePrim("/ArticulationDescendantTest/ChildPrim")
        PhysxSchema.PhysxParticleSamplingAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.ARTICULATION_ROOT_API_CONFLICT_CODE))

    # Tests for deep hierarchy conflicts
    def test_deep_hierarchy_api_conflict(self):
        """Test that API conflicts are detected in deep hierarchies"""
        # Create root prim with RigidBodyAPI
        root = self._stage.DefinePrim("/DeepHierarchyTest")
        UsdPhysics.RigidBodyAPI.Apply(root)

        # Create nested child prim with a conflicting API
        child = self._stage.DefinePrim("/DeepHierarchyTest/Level1/Level2/Level3")
        PhysxSchema.PhysxDeformableSurfaceAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.RB_API_CONFLICT_CODE))

    def test_hierarchical_collision_api_conflict(self):
        """Test that CollisionAPI conflicts in the hierarchy are detected"""
        # Create parent prim with CollisionAPI
        parent = self._stage.DefinePrim("/CollisionParentPrim")
        UsdPhysics.CollisionAPI.Apply(parent)

        # Create child prim with a conflicting API
        child = self._stage.DefinePrim("/CollisionParentPrim/ChildPrim")
        PhysxSchema.PhysxDeformableSurfaceAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.COLLISION_API_CONFLICT_CODE))

    def test_hierarchical_articulation_root_api_conflict(self):
        """Test that ArticulationRootAPI conflicts in the hierarchy are detected"""
        # Create parent prim with ArticulationRootAPI
        parent = self._stage.DefinePrim("/ArticulationParentPrim")
        UsdPhysics.ArticulationRootAPI.Apply(parent)

        # Create child prim with a conflicting API
        child = self._stage.DefinePrim("/ArticulationParentPrim/ChildPrim")
        PhysxSchema.PhysxParticleSetAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.ARTICULATION_ROOT_API_CONFLICT_CODE))

    def test_hierarchical_child_to_parent_api_conflict(self):
        """Test that API conflicts are detected when child has API that conflicts with parent"""
        # Create parent prim with a deformable API
        parent = self._stage.DefinePrim("/DeformableParentPrim")
        PhysxSchema.PhysxDeformableBodyAPI.Apply(parent)

        # Create child prim with a conflicting API
        child = self._stage.DefinePrim("/DeformableParentPrim/ChildPrim")
        UsdPhysics.RigidBodyAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.RB_API_CONFLICT_CODE))

    def test_hierarchical_api_conflict(self):
        """Test that API conflicts in the hierarchy are detected"""
        # Create parent prim with RigidBodyAPI
        parent = self._stage.DefinePrim("/ParentPrim")
        UsdPhysics.RigidBodyAPI.Apply(parent)

        # Create child prim with a conflicting API
        child = self._stage.DefinePrim("/ParentPrim/ChildPrim")
        PhysxSchema.PhysxDeformableBodyAPI.Apply(child)

        # Run validation and check for API conflict error
        self.run_validation_and_assert_result(ExpectedResult(code=APIConflictChecker.RB_API_CONFLICT_CODE))
