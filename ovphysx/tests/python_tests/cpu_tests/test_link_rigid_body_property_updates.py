# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Runtime changes of PhysxRigidBodyAPI attributes reach articulation links.

The rigid-body property-update handlers accept articulation links as well as rigid bodies. A damping or
maximum-velocity value written after the stage is attached must therefore slow a moving link down, as it
slows a rigid body down. The scene has no gravity and no contacts: a free cube and a floating articulation
of two cubes joined by a fixed joint along x move at 1 m/s along x, or spin at 1 rad/s about x.
"""

import numpy as np
import ovstage
import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import attach_usd_with_ovstage

DT = 1.0 / 60.0
BODY = "/World/body"
ARTICULATION = "/World/articulation"
LINKS = (f"{ARTICULATION}/link0", f"{ARTICULATION}/link1")

# Initial velocity of every body, and the column that reads it back. USD angular velocities are in deg/s.
LINEAR = ("vector3f physics:velocity = (1, 0, 0)", "linearVelocity")
ANGULAR = ("vector3f physics:angularVelocity = (57.29578, 0, 0)", "angularVelocity")

# Attribute -> (value written at runtime, motion it slows down). 17.19 deg/s = 0.3 rad/s.
CASES = {
    "physxRigidBody:linearDamping": (5.0, LINEAR),
    "physxRigidBody:angularDamping": (5.0, ANGULAR),
    "physxRigidBody:maxLinearVelocity": (0.3, LINEAR),
    "physxRigidBody:maxAngularVelocity": (17.19, ANGULAR),
}


def _cube(name, x, y, velocity):
    return [
        f'    def Cube "{name}" (',
        '        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]',
        "    )",
        "    {",
        "        double size = 0.2",
        "        float physics:mass = 1",
        f"        {velocity}",
        "        float physxRigidBody:linearDamping = 0",
        "        float physxRigidBody:angularDamping = 0",
        f"        double3 xformOp:translate = ({x}, {y}, 0)",
        '        uniform token[] xformOpOrder = ["xformOp:translate"]',
        "    }",
    ]


def _usda(tmp_path, velocity):
    lines = [
        "#usda 1.0",
        '(\n    defaultPrim = "World"\n    metersPerUnit = 1\n    upAxis = "Z"\n)',
        'def Xform "World"',
        "{",
        '    def PhysicsScene "physicsScene"',
        "    {",
        "        float physics:gravityMagnitude = 0",
        "    }",
        *_cube("body", 0, 0, velocity),
        '    def Xform "articulation" (',
        '        prepend apiSchemas = ["PhysicsArticulationRootAPI"]',
        "    )",
        "    {",
        *["    " + line for line in _cube("link0", 0, 2, velocity) + _cube("link1", 0.25, 2, velocity)],
        '        def PhysicsFixedJoint "joint"',
        "        {",
        f"            rel physics:body0 = <{LINKS[0]}>",
        f"            rel physics:body1 = <{LINKS[1]}>",
        "            point3f physics:localPos0 = (0.25, 0, 0)",
        "        }",
        "    }",
        "}",
    ]
    path = tmp_path / "link_rigid_body_property_updates.usda"
    path.write_text("\n".join(lines) + "\n")
    return str(path)


def _max_speed(physx_sdk, object_type, column):
    with physx_sdk.read(object_type, [column], scope=ObjectScope.ALL) as result:
        return max(float(np.linalg.norm(g.tensors[0].numpy().reshape(-1, 3), axis=1).max()) for g in result.groups)


@pytest.mark.parametrize("attribute", CASES)
def test_runtime_rigid_body_property_reaches_links(physx_sdk, tmp_path, attribute):
    """A runtime damping or maximum-velocity change slows the articulation links down like the rigid body."""
    value, (velocity, column) = CASES[attribute]
    stage = attach_usd_with_ovstage(physx_sdk, _usda(tmp_path, velocity))
    physx_sdk.step(DT)
    physx_sdk.wait_all()

    # Write the attribute on the body and on both links at a new ordinal and apply it.
    paths = ovstage.PathDictionary(stage)
    query = stage.query_from_path_list(paths.create_path_list_from_strings([BODY, *LINKS]))
    stage.write_attribute(
        query, attribute, 2, np.full((3, 1), value, np.float32), is_array=False, prim_mode=ovstage.PrimMode.UPSERT
    ).wait()
    query.release()
    stage.advance_write_floor(ordinal=2).wait()
    physx_sdk.update_from_ovstage(2, 2)

    for _ in range(30):
        physx_sdk.step(DT)
    physx_sdk.wait_all()
    body = _max_speed(physx_sdk, SimObjectType.RIGID_BODY, column)
    links = _max_speed(physx_sdk, SimObjectType.ARTICULATION_LINK, column)

    # Unchanged, every body would still move at 1; damping 5 leaves about 0.07 after 30 steps, the limit 0.3.
    assert body < 0.5, f"{attribute} did not reach the rigid body: {body:.3f}"
    assert links < 0.5, f"{attribute} did not reach the articulation links: {links:.3f} (rigid body {body:.3f})"
