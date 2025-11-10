# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
__all__ = ["ArticulationJointsChecker"]

from omni.asset_validator.core import BaseRuleChecker, registerRule, Suggestion
from pxr import Usd, UsdPhysics, PhysxSchema

@registerRule("Omni:Physx")
class ArticulationJointsChecker(BaseRuleChecker):
    __doc__ = """
    Validate the configuration of articulation joints
    """

    ARTICULATION_JOINT_UNSUPPORTED_ATTRIBUTE_CODE = "ArticulationJointsChecker:JointUnsupportedAttribute"

    @classmethod
    def get_physx_limit_api_unsupported_attributes(cls, physx_limit_api : PhysxSchema.PhysxLimitAPI):
        return (physx_limit_api.GetStiffnessAttr(), physx_limit_api.GetDampingAttr(), 
                              physx_limit_api.GetBounceThresholdAttr(), physx_limit_api.GetRestitutionAttr())

    @classmethod
    def get_joint_limit_types(cls, joint_prim : Usd.Prim):
        if joint_prim.IsA(UsdPhysics.RevoluteJoint):
            return (UsdPhysics.Tokens.angular,)
        elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
            return (UsdPhysics.Tokens.linear,)
        elif joint_prim.IsA(UsdPhysics.SphericalJoint):
            return (UsdPhysics.Tokens.cone,)
        elif joint_prim.HasAPI(UsdPhysics.LimitAPI):
            return (UsdPhysics.Tokens.transX, UsdPhysics.Tokens.transY,  UsdPhysics.Tokens.transZ,
            UsdPhysics.Tokens.rotX,  UsdPhysics.Tokens.rotY,  UsdPhysics.Tokens.rotZ)
        return []

    def clear_articulation_joint_unsupported_attributes(self, stage: Usd.Stage, location: Usd.Prim) -> None:
        joint_prim : Usd.Prim = location
        if not joint_prim.IsValid():
            return

        limit_types = self.get_joint_limit_types(joint_prim)
        for limit_type in limit_types:
            if joint_prim.HasAPI(PhysxSchema.PhysxLimitAPI, limit_type):
                physx_limit_api = PhysxSchema.PhysxLimitAPI(joint_prim, limit_type)
                attributes = self.get_physx_limit_api_unsupported_attributes(physx_limit_api)
                for attribute in attributes:
                    if attribute.HasAuthoredValue():
                        attribute.Clear()

    def CheckStage(self, stage : Usd.Stage):
        stage_usd_physics = UsdPhysics.LoadUsdPhysicsFromRange(stage, ["/"])

        for key, value in stage_usd_physics.items():
            prim_paths, descs = value
            if key == UsdPhysics.ObjectType.Articulation:
                for prim_path, desc in zip(prim_paths, descs):
                    for articulatedJoint in desc.articulatedJoints:
                        joint_prim : Usd.Prim = stage.GetPrimAtPath(articulatedJoint)
                        if not joint_prim.IsValid():
                            continue

                        limit_types = self.get_joint_limit_types(joint_prim)

                        # Determine what unsupported attributes have been set for this
                        unsupported_attributes = []
                        for limit_type in limit_types:
                            if not joint_prim.HasAPI(PhysxSchema.PhysxLimitAPI, limit_type):
                                continue
                        
                            physx_limit_api = PhysxSchema.PhysxLimitAPI(joint_prim, limit_type)
                            attributes = self.get_physx_limit_api_unsupported_attributes(physx_limit_api)
                            for attribute in attributes:
                                if attribute.HasAuthoredValue():
                                    unsupported_attributes.append(attribute.GetName())
                        

                        if len(unsupported_attributes) > 0:
                            plural = "s" if len(unsupported_attributes) > 1 else ""
                            if len(unsupported_attributes) == 1:
                                unsupported_attributes = unsupported_attributes[0]
                            self._AddWarning(
                                message="Unsupported limit attribute" + plural + f" for articulation joint: {unsupported_attributes} (attribute" + plural+" will be ignored)",
                                at=joint_prim,
                                suggestion=Suggestion(
                                    message="Clear value"+ plural + " of the unsupported attribute" + plural + ".",
                                    callable=self.clear_articulation_joint_unsupported_attributes,
                                    at=[joint_prim],
                                ),
                                code=self.ARTICULATION_JOINT_UNSUPPORTED_ATTRIBUTE_CODE
                            )
