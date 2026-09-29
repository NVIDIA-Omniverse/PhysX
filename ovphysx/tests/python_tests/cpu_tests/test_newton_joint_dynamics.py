# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""NewtonJointAPI passive joint dynamics reach the PhysX articulation joints.

`newton:armature`, `newton:friction` and `newton:damping` are joint-level Newton attributes,
broadcast to every DOF of the joint. They are read as fallbacks for the PhysxJointAxisAPI
armature, static and dynamic friction effort, and viscous friction coefficient, the same way
`newton:velocityLimit` falls back for maxJointVelocity: a PhysX value of the same field wins (a
strictly positive one, see ParseJoint.cpp), then the Newton value, then the PhysX default. The
Isaac Sim URDF and MJCF importers write joint friction and damping (and, from MJCF, armature) to
the engine-neutral physics layer of an asset this way.

The values are read back through the joint property read API, which reports an angular axis in
USD units (per degree), so an angular `newton:damping` reads back as authored only if the
parser converted it to per radian on the way in.

ovstage population publishes the properties of registered schemas only, so the Newton schemas
have to be registered, like the PhysX ones, before the first population in the process. The
scene therefore runs in a fresh interpreter, with the newton-usd-schemas package that every
source build fetches for ovruntime (ovruntime/deps/pip_newton.toml).
"""

import json
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest

ROOT = "/World/articulation"
OVPHYSX_ROOT = Path(__file__).resolve().parents[3]
NEWTON_SCHEMAS = OVPHYSX_ROOT / "ovruntime" / "_build" / "target-deps" / "newton_prebundle" / "newton_usd_schemas"

# Joint name -> (joint type, NewtonJointAPI (armature, friction, damping) or None, PhysX attributes).
# Every joint connects the fixed base link to a child link of its own. The values differ per joint,
# so that a mix-up shows.
JOINTS = {
    # NewtonJointAPI only, for every joint type the articulation parser creates DOFs for.
    "revolute": ("PhysicsRevoluteJoint", (0.02, 0.2, 0.5), {}),
    "prismatic": ("PhysicsPrismaticJoint", (0.05, 1.5, 2.0), {}),
    "spherical": ("PhysicsSphericalJoint", (0.015, 0.14, 0.25), {}),
    "d6": ("PhysicsJoint", (0.011, 0.11, 0.125), {}),
    # Authored per-axis PhysX values win over the Newton ones.
    "physx_axis": (
        "PhysicsRevoluteJoint",
        (0.02, 0.2, 0.5),
        {
            "physxJointAxis:angular:armature": 0.09,
            "physxJointAxis:angular:staticFrictionEffort": 0.9,
            "physxJointAxis:angular:dynamicFrictionEffort": 0.6,
            "physxJointAxis:angular:viscousFrictionCoefficient": 0.3,
        },
    ),
    # The two friction efforts come from one source, so that static >= dynamic still holds.
    "physx_static": ("PhysicsRevoluteJoint", (0.02, 0.2, 0.5), {"physxJointAxis:angular:staticFrictionEffort": 0.1}),
    # An authored joint-level physxJoint:armature wins over newton:armature.
    "physx_joint": ("PhysicsRevoluteJoint", (0.02, 0.2, 0.5), {"physxJoint:armature": 0.07}),
    # A PhysX API applied for an unrelated field leaves the Newton values in place.
    "axis_max_velocity": ("PhysicsRevoluteJoint", (0.03, 0.3, 0.75), {"physxJointAxis:angular:maxJointVelocity": 90}),
    "joint_max_velocity": ("PhysicsRevoluteJoint", (0.04, 0.4, 1.0), {"physxJoint:maxJointVelocity": 120}),
    # Control: nothing authored.
    "plain": ("PhysicsRevoluteJoint", None, {}),
}

# Joint name -> expected (armature, static friction, dynamic friction, viscous friction) of every
# DOF, as the read API reports them: effort*s/deg for the viscous coefficient on an angular axis.
EXPECTED = {
    "revolute": (0.02, 0.2, 0.2, 0.5),
    "prismatic": (0.05, 1.5, 1.5, 2.0),
    "spherical": (0.015, 0.14, 0.14, 0.25),
    "d6": (0.011, 0.11, 0.11, 0.125),
    "physx_axis": (0.09, 0.9, 0.6, 0.3),
    "physx_static": (0.02, 0.1, 0.0, 0.5),
    "physx_joint": (0.07, 0.2, 0.2, 0.5),
    "axis_max_velocity": (0.03, 0.3, 0.3, 0.75),
    "joint_max_velocity": (0.04, 0.4, 0.4, 1.0),
    "plain": (0.0, 0.0, 0.0, 0.0),
}
ATTRIBUTES = ("jointArmature", "jointStaticFriction", "jointDynamicFriction", "jointViscousFriction")

# The unrelated fields, in deg/s: proof that the PhysX API of those cases is in effect.
EXPECTED_MAX_VELOCITY = {"axis_max_velocity": 90.0, "joint_max_velocity": 120.0}

DOFS = {"PhysicsSphericalJoint": 3, "PhysicsJoint": 3}

TRANSLATIONS = ("transX", "transY", "transZ")
TYPE_ATTRIBUTES = {
    "PhysicsRevoluteJoint": ['uniform token physics:axis = "Z"'],
    "PhysicsPrismaticJoint": [
        'uniform token physics:axis = "X"',
        "float physics:lowerLimit = -0.5",
        "float physics:upperLimit = 0.5",
    ],
    "PhysicsSphericalJoint": ['uniform token physics:axis = "X"'],
    # A D6 joint in an articulation: translations locked (low > high), rotations free.
    "PhysicsJoint": [
        f"float limit:{axis}:physics:{bound} = {v}" for axis in TRANSLATIONS for bound, v in (("low", 1), ("high", -1))
    ],
}


def _link(name, y):
    return [
        f'        def Cube "{name}" (',
        '            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]',
        "        )",
        "        {",
        "            double size = 0.1",
        "            float physics:mass = 1",
        "            float3 physics:diagonalInertia = (0.01, 0.01, 0.01)",
        f"            double3 xformOp:translate = (0, {y}, 1)",
        '            uniform token[] xformOpOrder = ["xformOp:translate"]',
        "        }",
    ]


def _usda(tmp_path):
    lines = [
        "#usda 1.0",
        '(\n    defaultPrim = "World"\n    metersPerUnit = 1\n    upAxis = "Z"\n)',
        'def Xform "World"',
        "{",
        '    def PhysicsScene "physicsScene"',
        "    {",
        "    }",
        '    def Xform "articulation" (',
        '        prepend apiSchemas = ["PhysicsArticulationRootAPI"]',
        "    )",
        "    {",
        *_link("base", 0),
        '        def PhysicsFixedJoint "rootJoint"',
        "        {",
        f"            rel physics:body1 = <{ROOT}/base>",
        "            point3f physics:localPos0 = (0, 0, 1)",
        "        }",
    ]
    for i, (name, (joint_type, newton, physx)) in enumerate(JOINTS.items()):
        y = 0.5 * (i + 1)
        schemas = [f"PhysicsLimitAPI:{axis}" for axis in TRANSLATIONS] if joint_type == "PhysicsJoint" else []
        attributes = list(TYPE_ATTRIBUTES[joint_type])
        if newton:
            schemas.append("NewtonJointAPI")
            attributes += [
                f"float newton:{key} = {value}" for key, value in zip(("armature", "friction", "damping"), newton)
            ]
        if any(key.startswith("physxJointAxis:angular:") for key in physx):
            schemas.append("PhysxJointAxisAPI:angular")
        if any(key.startswith("physxJoint:") for key in physx):
            schemas.append("PhysxJointAPI")
        attributes += [f"float {key} = {value}" for key, value in physx.items()]
        lines += _link(f"{name}_link", y)
        if schemas:
            lines += [
                f'        def {joint_type} "{name}" (',
                f"            prepend apiSchemas = {json.dumps(schemas)}",
                "        )",
            ]
        else:
            lines.append(f'        def {joint_type} "{name}"')
        lines += [
            "        {",
            f"            rel physics:body0 = <{ROOT}/base>",
            f"            rel physics:body1 = <{ROOT}/{name}_link>",
            f"            point3f physics:localPos0 = (0, {y}, 0)",
            *(f"            {attribute}" for attribute in attributes),
            "        }",
        ]
    lines += ["    }", "}"]
    path = tmp_path / "newton_joint_dynamics.usda"
    path.write_text("\n".join(lines) + "\n")
    return path


_CHILD_SCRIPT = textwrap.dedent(
    """
    import json
    import sys

    import numpy as np
    import ovphysx
    import ovstage
    from ovphysx.types import ObjectScope, SimObjectType

    usd_path, newton_schemas, attributes = sys.argv[1], sys.argv[2], sys.argv[3].split(",")
    if not ovstage.population.available():
        print("SKIP|ovstage population bridge is unavailable", flush=True)
        sys.exit(0)
    ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root()), newton_schemas])
    stage = ovstage.Stage("newton-joint-dynamics")
    ovstage.population.open_usd(stage, usd_path, ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
    stage.advance_write_floor(ordinal=1).wait()

    ovphysx.PhysX.set_cpu_mode(True)
    physx = ovphysx.PhysX()
    try:
        physx.attach_ovstage(stage, read_ordinal=1)
        physx.step_sync(1.0 / 60.0)
        # Joint groups are array groups: the value of prim i of a group is tensors[i].
        values = {}
        with ovstage.PathDictionary(stage) as paths:
            for attribute in attributes:
                with physx.read(SimObjectType.ARTICULATION_JOINT, [attribute], scope=ObjectScope.ALL) as result:
                    for g in result.groups:
                        for path, t in zip(paths.get_path_strings(g.prim_list), g.tensors):
                            values.setdefault(path, {})[attribute] = np.asarray(t.numpy()).reshape(-1).tolist()
        physx.detach_ovstage()
        print("VALUES|" + json.dumps(values), flush=True)
    finally:
        physx.destroy()
        stage.destroy()
    """
)


def test_newton_joint_dynamics_reach_physx(tmp_path):
    if not (NEWTON_SCHEMAS / "plugInfo.json").is_file():
        pytest.skip(f"newton-usd-schemas not found at {NEWTON_SCHEMAS}")
    attributes = [*ATTRIBUTES, "jointMaxVelocity"]
    result = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT, str(_usda(tmp_path)), str(NEWTON_SCHEMAS), ",".join(attributes)],
        check=False,
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert result.returncode == 0, f"child failed\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    skip = [line for line in result.stdout.splitlines() if line.startswith("SKIP|")]
    if skip:
        pytest.skip(skip[0][5:])
    line = next(line for line in result.stdout.splitlines() if line.startswith("VALUES|"))
    read = json.loads(line[len("VALUES|") :])

    mismatches = []
    for name, (joint_type, _, _) in JOINTS.items():
        expected = dict(zip(ATTRIBUTES, EXPECTED[name]))
        if name in EXPECTED_MAX_VELOCITY:
            expected["jointMaxVelocity"] = EXPECTED_MAX_VELOCITY[name]
        for attribute, value in expected.items():
            got = read.get(f"{ROOT}/{name}", {}).get(attribute)
            want = [value] * DOFS.get(joint_type, 1)
            if got is None or len(got) != len(want) or not np.allclose(got, want, rtol=1e-5, atol=1e-7):
                mismatches.append(f"{name} {attribute}: expected {want}, read {got}")
    assert not mismatches, "\n".join(mismatches)
