# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
__all__ = ["MassChecker"]

from omni.asset_validator.core import BaseRuleChecker, registerRule
from pxr import Usd, UsdPhysics

@registerRule("Omni:Physx")
class MassChecker(BaseRuleChecker):
    __doc__ = """
    Check if the MassAPI is set correctly or can be succesfully computed.
    Validates both prim-level mass properties and stage-level physics requirements.

    Mass Validation Rules

    No MassAPI Present
    Rigid bodies without MassAPI must have collision shapes to compute mass properties from geometry.

    MassAPI Present
    If MassAPI exists but no collision shapes are found, explicit mass and inertia values must be provided.
    """

    MISSING_MASS_API_AND_COLLIDERS_CODE = "MassChecker:MissingMassAPIAndColliders"
    MISSING_MASS_API_PROPERTIES_CODE = "MassChecker:MissingMassAPIProperties"

    def _check_mass_on_prim(self, prim: Usd.Prim) -> None:
        """Validate mass configuration for individual prims"""
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        if not rigid_body_api:
            return

        has_mass_api = prim.HasAPI(UsdPhysics.MassAPI)

        has_colliders = False
        prim_range = Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
        for _prim in prim_range:
            if _prim.HasAPI(UsdPhysics.CollisionAPI):
                has_colliders = True
                break

        # Case 1: Missing MassAPI with no colliders
        if not has_mass_api:
            if not has_colliders:
                self._AddFailedCheck(
                    message=f"Missing MassAPI and colliders on {prim.GetPath()}",
                    at=prim,
                    code=self.MISSING_MASS_API_AND_COLLIDERS_CODE
                )
            return

        # Case 2: MassAPI with missing properties and no colliders
        mass_api = UsdPhysics.MassAPI(prim)
        if not has_colliders:
            required = {
                "mass": mass_api.GetMassAttr(),
                "diagonal inertia": mass_api.GetDiagonalInertiaAttr(),
            }

            missing = [name for name, attr in required.items() if not attr.HasAuthoredValue() or not attr.Get()]
            if missing:
                self._AddFailedCheck(
                    f"Prim has MassAPI but no colliders so {' and '.join(missing)} must be provided.",
                    prim,
                    code=self.MISSING_MASS_API_PROPERTIES_CODE
                )

    def CheckPrim(self, prim: Usd.Prim):
        self._check_mass_on_prim(prim)
