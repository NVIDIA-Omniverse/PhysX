# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .utils import BaseValidatorTest, ExpectedResult
from ..scripts.simulationOwnerChecker import SimulationOwnerChecker
from pxr import UsdGeom, UsdPhysics


class SimulationOwnerCheckerTestCase(BaseValidatorTest):
    rules = [SimulationOwnerChecker]

    def test_simulation_owner_no_owner(self):
        """Test case where both bodies have no simulation owner (should pass)."""
        stage = self.create_stage_in_memory()
        self.create_validation_engine()

        # Create two rigid bodies
        body0 = UsdGeom.Xform.Define(stage, "/World/body0")
        body1 = UsdGeom.Xform.Define(stage, "/World/body1")

        # Add RigidBodyAPI to both bodies
        UsdPhysics.RigidBodyAPI.Apply(body0.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(body1.GetPrim())

        # Create a joint between the bodies
        joint = UsdPhysics.Joint.Define(stage, "/World/joint")
        joint.CreateBody0Rel().SetTargets([body0.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPrim().GetPath()])

        self.run_validation_and_assert_result(None)

    def test_simulation_owner_same_owner(self):
        """Test case where both bodies have the same simulation owner (should pass)."""
        stage = self.create_stage_in_memory()
        self.create_validation_engine()

        # Create two rigid bodies
        body0 = UsdGeom.Xform.Define(stage, "/World/body0")
        body1 = UsdGeom.Xform.Define(stage, "/World/body1")

        # Add RigidBodyAPI to both bodies
        rb0 = UsdPhysics.RigidBodyAPI.Apply(body0.GetPrim())
        rb1 = UsdPhysics.RigidBodyAPI.Apply(body1.GetPrim())

        # Create a joint between the bodies
        joint = UsdPhysics.Joint.Define(stage, "/World/joint")
        joint.CreateBody0Rel().SetTargets([body0.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPrim().GetPath()])

        # Create a simulation owner (PhysicsScene)
        sim_owner = UsdPhysics.Scene.Define(stage, "/World/sim_owner")

        # Set same simulation owner for both bodies
        rb0.CreateSimulationOwnerRel().SetTargets([sim_owner.GetPrim().GetPath()])
        rb1.CreateSimulationOwnerRel().SetTargets([sim_owner.GetPrim().GetPath()])

        self.run_validation_and_assert_result(None)

    def test_simulation_owner_different_owners(self):
        """Test case where bodies have different simulation owners (should fail)."""
        stage = self.create_stage_in_memory()
        self.create_validation_engine()

        # Create two rigid bodies
        body0 = UsdGeom.Xform.Define(stage, "/World/body0")
        body1 = UsdGeom.Xform.Define(stage, "/World/body1")

        # Add RigidBodyAPI to both bodies
        rb0 = UsdPhysics.RigidBodyAPI.Apply(body0.GetPrim())
        rb1 = UsdPhysics.RigidBodyAPI.Apply(body1.GetPrim())

        # Create a joint between the bodies
        joint = UsdPhysics.Joint.Define(stage, "/World/joint")
        joint.CreateBody0Rel().SetTargets([body0.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPrim().GetPath()])

        # Create two different simulation owners (PhysicsScenes)
        sim_owner1 = UsdPhysics.Scene.Define(stage, "/World/sim_owner1")
        sim_owner2 = UsdPhysics.Scene.Define(stage, "/World/sim_owner2")

        # Set different simulation owners for the bodies
        rb0.CreateSimulationOwnerRel().SetTargets([sim_owner1.GetPrim().GetPath()])
        rb1.CreateSimulationOwnerRel().SetTargets([sim_owner2.GetPrim().GetPath()])

        self.run_validation_and_assert_result(ExpectedResult(code=SimulationOwnerChecker.JOINT_BAD_SIM_OWNERS_CODE))

    def test_simulation_owner_missing_owner(self):
        """Test case where only one body has a simulation owner (should fail)."""
        stage = self.create_stage_in_memory()
        self.create_validation_engine()

        # Create two rigid bodies
        body0 = UsdGeom.Xform.Define(stage, "/World/body0")
        body1 = UsdGeom.Xform.Define(stage, "/World/body1")

        # Add RigidBodyAPI to both bodies
        UsdPhysics.RigidBodyAPI.Apply(body0.GetPrim())
        rb1 = UsdPhysics.RigidBodyAPI.Apply(body1.GetPrim())

        # Create a joint between the bodies
        joint = UsdPhysics.Joint.Define(stage, "/World/joint")
        joint.CreateBody0Rel().SetTargets([body0.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPrim().GetPath()])

        # Create a simulation owner (PhysicsScene)
        sim_owner = UsdPhysics.Scene.Define(stage, "/World/sim_owner")

        # Set simulation owner only for body1
        rb1.CreateSimulationOwnerRel().SetTargets([sim_owner.GetPrim().GetPath()])

        self.run_validation_and_assert_result(ExpectedResult(code=SimulationOwnerChecker.JOINT_BAD_SIM_OWNERS_CODE))

    def test_simulation_owner_collision_bodies(self):
        """Test case with collision bodies having the same simulation owner (should pass)."""
        stage = self.create_stage_in_memory()
        self.create_validation_engine()

        # Create two collision bodies
        body0 = UsdGeom.Xform.Define(stage, "/World/body0")
        body1 = UsdGeom.Xform.Define(stage, "/World/body1")

        # Add CollisionAPI to both bodies
        col0 = UsdPhysics.CollisionAPI.Apply(body0.GetPrim())
        col1 = UsdPhysics.CollisionAPI.Apply(body1.GetPrim())

        # Create a joint between the bodies
        joint = UsdPhysics.Joint.Define(stage, "/World/joint")
        joint.CreateBody0Rel().SetTargets([body0.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPrim().GetPath()])

        # Create a simulation owner (PhysicsScene)
        sim_owner = UsdPhysics.Scene.Define(stage, "/World/sim_owner")

        # Set same simulation owner for both collision bodies
        col0.CreateSimulationOwnerRel().SetTargets([sim_owner.GetPrim().GetPath()])
        col1.CreateSimulationOwnerRel().SetTargets([sim_owner.GetPrim().GetPath()])

        self.run_validation_and_assert_result(None)

    def test_simulation_owner_not_physics_scene(self):
        """Test case where simulation owners are not PhysicsScene prims (should fail)."""
        stage = self.create_stage_in_memory()
        self.create_validation_engine()

        # Create two rigid bodies
        body0 = UsdGeom.Xform.Define(stage, "/World/body0")
        body1 = UsdGeom.Xform.Define(stage, "/World/body1")

        # Add RigidBodyAPI to both bodies
        rb0 = UsdPhysics.RigidBodyAPI.Apply(body0.GetPrim())
        rb1 = UsdPhysics.RigidBodyAPI.Apply(body1.GetPrim())

        # Create a joint between the bodies
        joint = UsdPhysics.Joint.Define(stage, "/World/joint")
        joint.CreateBody0Rel().SetTargets([body0.GetPrim().GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPrim().GetPath()])

        # Create simulation owners that are not PhysicsScene prims
        sim_owner0 = UsdGeom.Xform.Define(stage, "/World/sim_owner0")
        sim_owner1 = UsdGeom.Xform.Define(stage, "/World/sim_owner1")

        # Set non-PhysicsScene simulation owners for the bodies
        rb0.CreateSimulationOwnerRel().SetTargets([sim_owner0.GetPrim().GetPath()])
        rb1.CreateSimulationOwnerRel().SetTargets([sim_owner1.GetPrim().GetPath()])

        self.run_validation_and_assert_result(ExpectedResult(code=SimulationOwnerChecker.SIM_OWNER_NOT_PHYSICS_SCENE_CODE))
