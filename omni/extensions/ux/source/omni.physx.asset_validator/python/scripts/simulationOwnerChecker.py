# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
__all__ = ["SimulationOwnerChecker"]

from omni.asset_validator.core import BaseRuleChecker, registerRule
from pxr import Usd, UsdPhysics, UsdGeom, Sdf
from .utils import check_timeline_playing

@registerRule("Omni:Physx")
class SimulationOwnerChecker(BaseRuleChecker):
    __doc__ = """
    Check if the simulation owner is set correctly.
    """

    JOINT_BAD_SIM_OWNERS_CODE = "SimulationOwnerChecker:JointBadSimOwners"
    SIM_OWNER_NOT_PHYSICS_SCENE_CODE = "SimulationOwnerChecker:SimOwnerNotPhysicsScene"

    def get_bodies(self, joint: UsdPhysics.Joint, stage: Usd.Stage) -> tuple[Usd.Prim, Usd.Prim]:
        body0Targets = joint.GetBody0Rel().GetTargets()
        body1Targets = joint.GetBody1Rel().GetTargets()
        if len(body0Targets) > 0 and len(body1Targets) > 0:
            body0 = self.get_body(body0Targets[0], stage)
            body1 = self.get_body(body1Targets[0], stage)
            return body0, body1
        return None, None

    def get_body(self, body_target: Sdf.Path, stage: Usd.Stage):
        body = UsdPhysics.RigidBodyAPI.Get(stage, body_target)
        if body:
            return body
        body = UsdPhysics.CollisionAPI.Get(stage, body_target)
        if body:
            return body
        return None

    def check_simulation_owners(self, body0, body1):
        body0_sim_owner_targets = body0.GetSimulationOwnerRel().GetTargets()
        body1_sim_owner_targets = body1.GetSimulationOwnerRel().GetTargets()
        body0_sim_owner_targets_len = len(body0_sim_owner_targets)
        body1_sim_owner_targets_len = len(body1_sim_owner_targets)

        # both bodies have the default owner
        if body0_sim_owner_targets_len == 0 and body1_sim_owner_targets_len == 0:
            return True

        # both bodies have a simulation owner
        if body0_sim_owner_targets_len > 0 and body1_sim_owner_targets_len > 0:
            if body0_sim_owner_targets[0] != body1_sim_owner_targets[0]:
                # different simulation owners
                return False
            else:
                # same simulation owner
                return True

        # one body has a simulation owner, the other doesn't
        # TODO: this might be valid if the default owner is the same as the set one, that would require to search
        # for the default stage
        return False

    def check_simulation_owner_type(self, body, stage):
        sim_owner_targets = body.GetSimulationOwnerRel().GetTargets()
        if len(sim_owner_targets) > 0:
            sim_owner_prim = stage.GetPrimAtPath(sim_owner_targets[0])
            if not sim_owner_prim.IsA(UsdPhysics.Scene):
                return False
        return True

    def CheckStage(self, stage: Usd.Stage):
        if check_timeline_playing(self, stage):
            return

    def CheckPrim(self, prim: Usd.Prim):
        joint = UsdPhysics.Joint(prim)
        stage = prim.GetStage()
        if joint:
            body0, body1 = self.get_bodies(joint, stage)
            if body0 and body1:
                # Check if simulation owners are PhysicsScene prims
                if not self.check_simulation_owner_type(body0, stage) or not self.check_simulation_owner_type(body1, stage):
                    self._AddFailedCheck(
                        message=f'Joint "{prim.GetPath()}" bodies have simulation owners that are not PhysicsScene prims.',
                        at=prim,
                        code=self.SIM_OWNER_NOT_PHYSICS_SCENE_CODE
                    )
                    return

                if not self.check_simulation_owners(body0, body1):
                    self._AddFailedCheck(
                        message=f'Joint "{prim.GetPath()}" bodies have different simulation owners or only one has a defined owner.',
                        at=prim,
                        code=self.JOINT_BAD_SIM_OWNERS_CODE
                    )
