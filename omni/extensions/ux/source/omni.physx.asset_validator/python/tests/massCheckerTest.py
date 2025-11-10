# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from .utils import BaseValidatorTest, ExpectedResult
from ..scripts.massChecker import MassChecker
from pxr import UsdPhysics, Gf


class MassCheckerTestCase(BaseValidatorTest):
    rules = [MassChecker]

    async def setUp(self):
        await super().setUp()
        self._stage = self.create_stage_in_memory()
        self.create_validation_engine()

    def _create_prim_with_apis(self, path, rigid_body=False, mass_api=False, colliders=0):
        prim = self._stage.DefinePrim(path)
        if rigid_body:
            UsdPhysics.RigidBodyAPI.Apply(prim)
        if mass_api:
            UsdPhysics.MassAPI.Apply(prim)
        for i in range(colliders):
            child = self._stage.DefinePrim(f"{path}/collider_{i}")
            UsdPhysics.CollisionAPI.Apply(child)
        return prim

    def test_mass_non_rigidbody_prim(self):
        """Prim without RigidBodyAPI should pass validation"""
        self._create_prim_with_apis("/TestPrim")
        self.run_validation_and_assert_result(None)

    def test_mass_missing_massapi_and_colliders(self):
        """RigidBody with neither MassAPI nor colliders should fail"""
        self._create_prim_with_apis("/RigidBody", rigid_body=True)
        self.run_validation_and_assert_result(ExpectedResult(code=MassChecker.MISSING_MASS_API_AND_COLLIDERS_CODE))

    def test_mass_massapi_missing_properties(self):
        """MassAPI with missing properties and no colliders should fail"""
        self._create_prim_with_apis("/PhysicsBody", rigid_body=True, mass_api=True)
        self.run_validation_and_assert_result(ExpectedResult(code=MassChecker.MISSING_MASS_API_PROPERTIES_CODE))

    def test_mass_valid_massapi_with_colliders(self):
        """MassAPI with colliders should pass without properties"""
        self._create_prim_with_apis("/ValidBody", rigid_body=True, mass_api=True, colliders=2)
        self.run_validation_and_assert_result(None)

    def test_mass_configured_massapi_without_colliders(self):
        """Fully configured MassAPI without colliders should pass"""
        prim = self._create_prim_with_apis("/ConfiguredBody", rigid_body=True, mass_api=True)
        mass_api = UsdPhysics.MassAPI(prim)
        mass_api.CreateMassAttr(5.0)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(2.0, 2.0, 2.0))
        self.run_validation_and_assert_result(None)

    def test_mass_partially_configured_massapi(self):
        """MassAPI with some properties configured should report missing ones"""
        prim = self._create_prim_with_apis("/PartialBody", rigid_body=True, mass_api=True)
        mass_api = UsdPhysics.MassAPI(prim)
        mass_api.CreateMassAttr(5.0)
        self.run_validation_and_assert_result(ExpectedResult(code=MassChecker.MISSING_MASS_API_PROPERTIES_CODE))
