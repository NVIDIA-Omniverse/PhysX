# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# PARTIALLY DEPRECATED (tensor-binding-deprecation): the shape tensor-binding tests here retire with the binding. The material-pool regression stays.

"""Tests for shape-level tensor bindings: material properties, contact offsets, rest offsets.

Scenes:
  - simple_physics_scene.usda: rigid body (Cube1) with at least 1 shape
  - two_articulations.usda: 2 articulations with 3 links each
"""

import os

import numpy as np
import pytest
from ovphysx.types import ObjectScope, SimObjectType, TensorType
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


RB_PATTERN = "/World/Cube*"
ARTI_PATTERN = "/World/articulation*"


# ---------------------------------------------------------------------------
# Rigid body shape-level tensors
# ---------------------------------------------------------------------------


class TestRigidBodyShapeTensors:

    def _make_binding(self, sdk, tensor_type):
        load_usd_with_ovstage(sdk, data_path("simple_physics_scene.usda"))
        sdk.wait_all()
        return sdk.create_tensor_binding(pattern=RB_PATTERN, tensor_type=tensor_type)

    def test_material_properties_shape(self, physx_sdk_cpu):
        """Material properties binding should have shape [N, S, 3]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION)
        assert b.ndim == 3
        N, S, C = b.shape
        assert N >= 1
        assert S >= 1
        assert C == 3
        b.destroy()

    def test_material_properties_read(self, physx_sdk_cpu):
        """Material properties should be readable and contain finite values."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out))
        b.destroy()

    def test_contact_offset_shape(self, physx_sdk_cpu):
        """Contact offset binding should have shape [N, S]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_CONTACT_OFFSET)
        assert b.ndim == 2
        N, S = b.shape
        assert N >= 1
        assert S >= 1
        b.destroy()

    def test_contact_offset_read(self, physx_sdk_cpu):
        """Contact offsets should be readable and contain finite values."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_CONTACT_OFFSET)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out))
        b.destroy()

    def test_rest_offset_shape(self, physx_sdk_cpu):
        """Rest offset binding should have shape [N, S]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_REST_OFFSET)
        assert b.ndim == 2
        N, S = b.shape
        assert N >= 1
        assert S >= 1
        b.destroy()

    def test_rest_offset_read(self, physx_sdk_cpu):
        """Rest offsets should be readable and contain finite values."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_REST_OFFSET)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out))
        b.destroy()

    def test_material_properties_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original.copy()
        modified[:, :, 0] = 0.7  # static_friction
        modified[:, :, 1] = 0.5  # dynamic_friction
        modified[:, :, 2] = 0.3  # restitution
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    def test_material_properties_pool_reuse(self, physx_sdk_cpu):
        """Regression for NVBugs 6489465 / OMPE-102536.

        Writing shape friction/restitution interns PxMaterials in a refcounted
        pool (BaseSimulationView::mMaterials) keyed by a formatted value string.
        A prior key-drift bug erased pool entries with a reconstructed 3-component
        key that never matched the stored key, so a released material was recycled
        and repurposed for a new tuple while the stale pool entry still pointed at
        it. Re-requesting the original tuple then returned the recycled material
        with foreign values (the write reported success and read-back was wrong).

        Drive the A -> B -> C -> A write sequence that reproduced the corruption:
        after B every shape is off tuple A (its material refcount hits 0 and is
        recycled), C repurposes that material, and re-requesting A must return A's
        values, not C's.
        """
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION)

        def uniform(static_friction, dynamic_friction, restitution):
            props = np.zeros(b.shape, dtype=np.float32)
            props[:, :, 0] = static_friction
            props[:, :, 1] = dynamic_friction
            props[:, :, 2] = restitution
            return props

        mat_a = uniform(0.1, 0.2, 0.3)
        mat_b = uniform(0.4, 0.5, 0.6)
        mat_c = uniform(0.7, 0.8, 0.9)

        b.write(mat_a)
        b.write(mat_b)
        b.write(mat_c)
        b.write(mat_a)  # re-request the recycled tuple

        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(
            result, mat_a, rtol=1e-5,
            err_msg="Re-requesting a previously-used material tuple returned the "
            "wrong material (NVBugs 6489465)",
        )
        b.destroy()

    def test_contact_offset_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_CONTACT_OFFSET)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 0.01
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    def test_rest_offset_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_REST_OFFSET)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 0.005
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()


# ---------------------------------------------------------------------------
# Articulation shape-level tensors
# ---------------------------------------------------------------------------


class TestArticulationShapeTensors:

    def _make_binding(self, sdk, tensor_type):
        load_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
        sdk.wait_all()
        return sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)

    def test_material_properties_shape(self, physx_sdk_cpu):
        """Articulation material properties binding should have shape [N, S, 3]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION)
        assert b.ndim == 3
        N, S, C = b.shape
        assert N >= 1
        assert S >= 1
        assert C == 3
        b.destroy()

    def test_material_properties_read(self, physx_sdk_cpu):
        """Articulation material properties should be readable and finite."""
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out))
        b.destroy()

    def test_contact_offset_shape(self, physx_sdk_cpu):
        """Articulation contact offset binding should have shape [N, S]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_CONTACT_OFFSET)
        assert b.ndim == 2
        N, S = b.shape
        assert N >= 1
        assert S >= 1
        b.destroy()

    def test_rest_offset_shape(self, physx_sdk_cpu):
        """Articulation rest offset binding should have shape [N, S]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_REST_OFFSET)
        assert b.ndim == 2
        N, S = b.shape
        assert N >= 1
        assert S >= 1
        b.destroy()

    def test_material_properties_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original.copy()
        modified[:, :, 0] = 0.6  # static_friction
        modified[:, :, 1] = 0.4  # dynamic_friction
        modified[:, :, 2] = 0.2  # restitution
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    def test_material_properties_pool_reuse(self, physx_sdk_cpu):
        """Regression for NVBugs 6489465 / OMPE-102536 on the articulation path.

        Articulation shape material writes share the same refcounted material
        pool and release path (BaseSimulationView::releaseSharedMaterial) that
        the rigid-body writes use, so the same A -> B -> C -> A key-drift repro
        must return A's values on re-request rather than the recycled C material.
        """
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION)

        def uniform(static_friction, dynamic_friction, restitution):
            props = np.zeros(b.shape, dtype=np.float32)
            props[:, :, 0] = static_friction
            props[:, :, 1] = dynamic_friction
            props[:, :, 2] = restitution
            return props

        mat_a = uniform(0.1, 0.2, 0.3)
        mat_b = uniform(0.4, 0.5, 0.6)
        mat_c = uniform(0.7, 0.8, 0.9)

        b.write(mat_a)
        b.write(mat_b)
        b.write(mat_c)
        b.write(mat_a)  # re-request the recycled tuple

        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(
            result, mat_a, rtol=1e-5,
            err_msg="Re-requesting a previously-used material tuple returned the "
            "wrong material (NVBugs 6489465)",
        )
        b.destroy()

    def test_contact_offset_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_CONTACT_OFFSET)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 0.01
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    def test_rest_offset_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_REST_OFFSET)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 0.005
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()


# ---------------------------------------------------------------------------
# Material writes keep the rest of the material
# ---------------------------------------------------------------------------

DT = 1.0 / 60.0

# Each case authors one material on 1 kg cubes and the initial velocity that shows it on ground with friction 1:
#   - friction 0.2 with the "min" combine mode: a slider decelerates at 0.2 g, not the 0.6 g of "average";
#   - a damped compliant contact: a cube dropped onto the ground comes to rest instead of bouncing;
#   - the same contact as an acceleration spring: the rigid body sinks about 10 mm, not the 2 mm of a force spring.
COMPLIANT = [
    "float physics:staticFriction = 1",
    "float physics:dynamicFriction = 1",
    "float physxMaterial:compliantContactStiffness = 1000",
    "float physxMaterial:compliantContactDamping = 63",
]
MATERIAL_CASES = {
    "friction combine mode": (
        [
            "float physics:staticFriction = 0.2",
            "float physics:dynamicFriction = 0.2",
            'uniform token physxMaterial:frictionCombineMode = "min"',
        ],
        "(2, 0, 0)",
    ),
    "compliant contact damping": (COMPLIANT, "(0, 0, -1)"),
    "compliant acceleration spring": (
        COMPLIANT + ["bool physxMaterial:compliantContactAccelerationSpring = 1"],
        "(0, 0, 0)",
    ),
}


def _material(name, attrs):
    return [
        f'    def Material "{name}" (',
        '        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]',
        "    )",
        "    {",
        *[f"        {attr}" for attr in attrs],
        "    }",
    ]


def _cube(name, x, y, velocity):
    return [
        f'    def Cube "{name}" (',
        '        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsCollisionAPI",'
        ' "MaterialBindingAPI"]',
        "    )",
        "    {",
        "        double size = 0.2",
        "        float physics:mass = 1",
        f"        vector3f physics:velocity = {velocity}",
        "        rel material:binding:physics = </World/body_material>",
        f"        double3 xformOp:translate = ({x}, {y}, 0.1)",
        '        uniform token[] xformOpOrder = ["xformOp:translate"]',
        "    }",
    ]


def _material_scene(tmp_path, attrs, velocity):
    """A rigid body and an articulation of two cubes joined by a fixed joint, resting on a ground box."""
    lines = [
        "#usda 1.0",
        '(\n    defaultPrim = "World"\n    metersPerUnit = 1\n    upAxis = "Z"\n)',
        'def Xform "World"',
        "{",
        '    def PhysicsScene "physicsScene"',
        "    {",
        "    }",
        *_material("ground_material", ["float physics:staticFriction = 1", "float physics:dynamicFriction = 1"]),
        *_material("body_material", attrs),
        '    def Cube "ground" (',
        '        prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]',
        "    )",
        "    {",
        "        double size = 1",
        "        rel material:binding:physics = </World/ground_material>",
        "        double3 xformOp:translate = (0, 0, -0.5)",
        "        float3 xformOp:scale = (20, 20, 1)",
        '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
        "    }",
        *_cube("body", 0, 0, velocity),
        '    def Xform "articulation" (',
        '        prepend apiSchemas = ["PhysicsArticulationRootAPI"]',
        "    )",
        "    {",
        *["    " + line for line in _cube("link0", 0, 2, velocity) + _cube("link1", 0.25, 2, velocity)],
        '        def PhysicsFixedJoint "joint"',
        "        {",
        "            rel physics:body0 = </World/articulation/link0>",
        "            rel physics:body1 = </World/articulation/link1>",
        "            point3f physics:localPos0 = (0.25, 0, 0)",
        "        }",
        "    }",
        "}",
    ]
    path = tmp_path / "material_write.usda"
    path.write_text("\n".join(lines) + "\n")
    return str(path)


def _speeds(sdk, object_type):
    with sdk.read(object_type, ["linearVelocity"], scope=ObjectScope.ALL) as result:
        return np.concatenate([np.linalg.norm(g.tensors[0].numpy().reshape(-1, 3), axis=1) for g in result.groups])


def _speeds_after(sdk, usda, write_back):
    """Body and link speeds at each of 30 steps, optionally after writing back the material values read."""
    load_usd_with_ovstage(sdk, usda)
    sdk.wait_all()
    if write_back:
        for pattern, tensor_type in (
            ("/World/body", TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION),
            ("/World/articulation", TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION),
        ):
            b = sdk.create_tensor_binding(pattern=pattern, tensor_type=tensor_type)
            values = np.zeros(b.shape, dtype=np.float32)
            b.read(values)
            b.write(values)
            b.destroy()
    bodies, links = [], []
    for _ in range(30):
        sdk.step(DT)
        sdk.wait_all()
        bodies.append(_speeds(sdk, SimObjectType.RIGID_BODY))
        links.append(_speeds(sdk, SimObjectType.ARTICULATION_LINK))
    return np.array(bodies), np.array(links)


@pytest.mark.parametrize("case", MATERIAL_CASES)
def test_material_write_back_keeps_the_rest_of_the_material(physx_sdk_cpu, tmp_path, case):
    """Writing back the friction and restitution just read leaves the motion unchanged.

    The write sets friction and restitution only, so the combine modes, the compliant-contact damping and the
    material flags of the material it replaces must survive it, on rigid-body and articulation-link shapes alike.
    """
    usda = _material_scene(tmp_path, *MATERIAL_CASES[case])
    unchanged = _speeds_after(physx_sdk_cpu, usda, write_back=False)
    written = _speeds_after(physx_sdk_cpu, usda, write_back=True)
    for name, expected, actual in zip(("rigid body", "articulation links"), unchanged, written):
        np.testing.assert_allclose(
            actual, expected, atol=1e-3, err_msg=f"{case}: the write-back changed the {name} speeds"
        )
