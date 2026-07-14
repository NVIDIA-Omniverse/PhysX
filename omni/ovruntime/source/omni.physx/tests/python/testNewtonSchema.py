# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Basic tests for Newton USD schema registration and attribute defaults.

import _physics_setup  # noqa: F401 - loads Carbonite + physics plugins + Newton schemas

import math
import pytest
from pxr import Plug, Tf, Usd, Sdf


# ---------------------------------------------------------------------------
# Schema registration
# ---------------------------------------------------------------------------

class TestNewtonSchemaRegistration:
    """Verify the Newton schema plugin is registered and declares expected types."""

    def test_plugin_registered(self):
        plugin = Plug.Registry().GetPluginWithName("newton")
        assert plugin is not None, "Newton schema plugin not registered"

    @pytest.mark.parametrize("type_name", [
        "NewtonPhysicsSceneAPI",
        "NewtonPhysicsXpbdSceneAPI",
        "NewtonPhysicsKaminoSceneAPI",
        "NewtonPhysicsArticulationRootAPI",
        "NewtonPhysicsCollisionAPI",
        "NewtonPhysicsMeshCollisionAPI",
        "NewtonPhysicsMaterialAPI",
        "NewtonPhysicsMimicAPI",
    ])
    def test_schema_type_declared(self, type_name):
        tf_type = Tf.Type.FindByName(type_name)
        assert not tf_type.isUnknown, f"{type_name} not found in type registry"


# ---------------------------------------------------------------------------
# Apply and default values
# ---------------------------------------------------------------------------

class TestNewtonSceneAPI:
    """Test NewtonSceneAPI application and default attribute values."""

    def _make_scene(self):
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim(Sdf.Path("/TestScene"), "PhysicsScene")
        return stage, prim

    def test_apply(self):
        stage, prim = self._make_scene()
        assert prim.ApplyAPI("NewtonSceneAPI")

    def test_default_max_solver_iterations(self):
        stage, prim = self._make_scene()
        prim.ApplyAPI("NewtonSceneAPI")
        val = prim.GetAttribute("newton:maxSolverIterations").Get()
        assert val == -1

    def test_default_time_steps_per_second(self):
        stage, prim = self._make_scene()
        prim.ApplyAPI("NewtonSceneAPI")
        val = prim.GetAttribute("newton:timeStepsPerSecond").Get()
        assert val == 1000

    def test_default_gravity_enabled(self):
        stage, prim = self._make_scene()
        prim.ApplyAPI("NewtonSceneAPI")
        val = prim.GetAttribute("newton:gravityEnabled").Get()
        assert val is True


class TestNewtonCollisionAPI:
    """Test NewtonCollisionAPI on a Mesh prim."""

    def _make_mesh(self):
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim(Sdf.Path("/TestMesh"), "Mesh")
        return stage, prim

    def test_apply(self):
        stage, prim = self._make_mesh()
        assert prim.ApplyAPI("NewtonCollisionAPI")

    def test_default_contact_margin(self):
        stage, prim = self._make_mesh()
        prim.ApplyAPI("NewtonCollisionAPI")
        val = prim.GetAttribute("newton:contactMargin").Get()
        assert val == pytest.approx(0.0)

    def test_default_contact_gap(self):
        stage, prim = self._make_mesh()
        prim.ApplyAPI("NewtonCollisionAPI")
        val = prim.GetAttribute("newton:contactGap").Get()
        assert val == float("-inf")


class TestNewtonMaterialAPI:
    """Test NewtonMaterialAPI on a Material prim."""

    def _make_material(self):
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim(Sdf.Path("/TestMaterial"), "Material")
        return stage, prim

    def test_default_torsional_friction(self):
        stage, prim = self._make_material()
        prim.ApplyAPI("NewtonMaterialAPI")
        val = prim.GetAttribute("newton:torsionalFriction").Get()
        assert val == pytest.approx(0.25)

    def test_default_rolling_friction(self):
        stage, prim = self._make_material()
        prim.ApplyAPI("NewtonMaterialAPI")
        val = prim.GetAttribute("newton:rollingFriction").Get()
        assert val == pytest.approx(0.0005)


class TestNewtonXpbdSceneAPI:
    """Test NewtonXpbdSceneAPI attributes."""

    def _make_scene(self):
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim(Sdf.Path("/TestScene"), "PhysicsScene")
        return stage, prim

    def test_default_soft_body_relaxation(self):
        stage, prim = self._make_scene()
        prim.ApplyAPI("NewtonXpbdSceneAPI")
        val = prim.GetAttribute("newton:xpbd:softBodyRelaxation").Get()
        assert val == pytest.approx(0.9)

    def test_default_rigid_contact_relaxation(self):
        stage, prim = self._make_scene()
        prim.ApplyAPI("NewtonXpbdSceneAPI")
        val = prim.GetAttribute("newton:xpbd:rigidContactRelaxation").Get()
        assert val == pytest.approx(0.8)
