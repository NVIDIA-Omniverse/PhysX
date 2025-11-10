# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
__all__ = ["APIConflictChecker"]

from omni.asset_validator.core import BaseRuleChecker, registerRule
from pxr import Usd, UsdPhysics 
import omni.physx.bindings._physx as pxb

@registerRule("Omni:Physx")
class APIConflictChecker(BaseRuleChecker):
    __doc__ = """
    Check if applied RigidBodyAPI, CollisionAPI, and ArticulationRootAPI have conflicting APIs in the hierarchy.
    """

    RB_API_CONFLICT_CODE = "APIConflictChecker:APIConflict"
    COLLISION_API_CONFLICT_CODE = "APIConflictChecker:CollisionAPIConflict"
    ARTICULATION_ROOT_API_CONFLICT_CODE = "APIConflictChecker:ArticulationRootAPIConflict"

    def _check_api_conflict_on_prim(self, prim: Usd.Prim) -> None:
        """Validate mass configuration for individual prims"""

        if UsdPhysics.RigidBodyAPI(prim):
            fail, where = pxb.hasconflictingapis_RigidBodyAPI_WRet(prim, True, False)
            print(prim, fail, where)
            if fail:
                self._AddFailedCheck(
                    message=f"RigidBodyAPI has conflicting API(s) at {where.GetPath()}",
                    at=prim,
                    code=self.RB_API_CONFLICT_CODE
                )

        if UsdPhysics.CollisionAPI(prim):
            fail, where = pxb.hasconflictingapis_CollisionAPI_WRet(prim, False, False)
            if fail:
                self._AddFailedCheck(
                    message=f"CollisionAPI has conflicting API(s) at {where.GetPath()}",
                    at=prim,
                    code=self.COLLISION_API_CONFLICT_CODE
                )

        if UsdPhysics.ArticulationRootAPI(prim):
            fail, where = pxb.hasconflictingapis_ArticulationRoot_WRet(prim, True, False)
            if fail:
                self._AddFailedCheck(
                    message=f"ArticulationRootAPI has conflicting API(s) at {where.GetPath()}",
                    at=prim,
                    code=self.ARTICULATION_ROOT_API_CONFLICT_CODE
                )

    def CheckPrim(self, prim: Usd.Prim):
        self._check_api_conflict_on_prim(prim)
