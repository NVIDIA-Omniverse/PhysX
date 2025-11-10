# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
__all__ = ["JointPoseChecker"]

from omni.asset_validator.core import BaseRuleChecker, registerRule, Suggestion
from omni.usdphysics.scripts import jointUtils
from pxr import Usd

@registerRule("Omni:Physx")
class JointPoseChecker(BaseRuleChecker):
    __doc__ = """
    Check if the joint pose satisfies the constraints.
    """

    JOINT_POSE_INVALID_CODE = "JointPoseChecker:JointPoseInvalid"

    def fix_joint_pose(self, stage: Usd.Stage, location: Usd.Prim) -> None:
        joint_validator = jointUtils.make_joint_validator(location)
        if joint_validator is None:
            return

        joint_validator.make_pose_valid(location)

    def CheckPrim(self, prim: Usd.Prim) -> None:
        """Validate joint poses for individual prims"""
        joint_validator = jointUtils.make_joint_validator(prim)
        if joint_validator is None:
            return

        if not joint_validator.validate_pose():
            self._AddFailedCheck(
                message=f"Joint pose exceeding limits for {prim.GetPath()}",
                at=prim,
                suggestion=Suggestion(
                    message="Change joint attachment offsets to fit within limits",
                    callable=self.fix_joint_pose,
                    at=[prim],
                ),
                code=self.JOINT_POSE_INVALID_CODE
            )
