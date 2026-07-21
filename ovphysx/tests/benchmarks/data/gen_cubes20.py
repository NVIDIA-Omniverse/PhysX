#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Generate a deliberately-minimal benchmark fixture: 20 falling cubes
with random positions, orientations, and angular velocities + one
static ground plane.

The intent is an "overhead-probe" scene: small enough that the actual
solver / contact work is trivial, so any time spent stepping reveals
fixed per-step overhead (dispatch, USD attribute readback, etc).
Pairs with LowLoad.empty_step (no scene, ~5 us) and
Step.basic_simulation_cpu (1 body, ~1 ms) to triangulate which costs
are body-count-independent vs scale with body count.

Initial state uses a fixed random seed so the fixture is reproducible.
Cubes spawn in a small X/Z footprint at random heights so some
overlap and collide during fall.

Usage:
    python3 tests/benchmarks/data/gen_cubes20.py > \
        tests/benchmarks/data/cubes20.usda

    python3 tests/benchmarks/data/gen_cubes20.py --envs > \
        tests/benchmarks/data/cubes20_envs.usda
"""

import argparse
import math
import random
import sys

NUM_CUBES = 20
CUBE_SIZE = 0.3
GROUND_HALF = 10.0
SEED = 20260603  # fixed seed: same output every regeneration


def quat_from_axis_angle(ax, ay, az, theta):
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    s = math.sin(theta / 2.0)
    return (math.cos(theta / 2.0), ax * s, ay * s, az * s)


HEADER = f"""#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
    doc = "20 falling cubes + ground plane. Minimal scene for overhead-floor benchmarking. See gen_cubes20.py."
)

def Xform "World"
{{
    def PhysicsScene "physicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {{
        vector3f physics:gravityDirection = (0, -1, 0)
        float physics:gravityMagnitude = 9.81
        uint physxScene:timeStepsPerSecond = 240
    }}

    def Cube "GroundPlane" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        double size = 1.0
        double3 xformOp:scale = ({GROUND_HALF * 2}, 0.02, {GROUND_HALF * 2})
        double3 xformOp:translate = (0, -0.01, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}
"""

CUBE_TEMPLATE = """
    def Cube "cube_{idx:02d}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
    )
    {{
        double size = {size}
        float physics:mass = 1.0
        vector3f physics:angularVelocity = ({wx}, {wy}, {wz})
        double3 xformOp:translate = ({tx}, {ty}, {tz})
        quatd xformOp:orient = ({qw}, {qx}, {qy}, {qz})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
    }}
"""

FOOTER = "}\n"


def _envs_header() -> str:
    return """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
    doc = "Cubes20 template fixture for cloning. Source body lives at /World/envs/template so TensorIo benches can clone() it into env1..envN and bind /World/envs/env*/* against the clones (not the template)."
)

def Xform "World"
{
    def PhysicsScene "physicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {
        vector3f physics:gravityDirection = (0, -1, 0)
        float physics:gravityMagnitude = 9.81
        uint physxScene:timeStepsPerSecond = 240
    }

    def Cube "GroundPlane" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        double size = 1.0
        double3 xformOp:scale = (200.0, 0.02, 200.0)
        double3 xformOp:translate = (0, -0.01, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }

    def Xform "envs"
    {
        def Xform "template"
        {
"""


_ENVS_CUBE_TEMPLATE = """
            def Cube "cube_{idx:02d}" (
                prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
            )
            {{
                double size = {size}
                float physics:mass = 1.0
                vector3f physics:angularVelocity = ({wx}, {wy}, {wz})
                double3 xformOp:translate = ({tx}, {ty}, {tz})
                quatd xformOp:orient = ({qw}, {qx}, {qy}, {qz})
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
            }}
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envs", action="store_true",
                        help="emit clonable variant with template under /World/envs/template")
    args = parser.parse_args()

    if args.envs:
        rng = random.Random(SEED)
        out = [_envs_header()]
        for i in range(NUM_CUBES):
            tx = rng.uniform(-1.0, 1.0)
            tz = rng.uniform(-1.0, 1.0)
            ty = rng.uniform(0.5, 5.0)
            ax = rng.uniform(-1.0, 1.0)
            ay = rng.uniform(-1.0, 1.0)
            az = rng.uniform(-1.0, 1.0)
            theta = rng.uniform(0.0, 2.0 * math.pi)
            qw, qx, qy, qz = quat_from_axis_angle(ax, ay, az, theta)
            wx = rng.uniform(-3.0, 3.0)
            wy = rng.uniform(-3.0, 3.0)
            wz = rng.uniform(-3.0, 3.0)
            out.append(_ENVS_CUBE_TEMPLATE.format(
                idx=i, size=CUBE_SIZE,
                tx=tx, ty=ty, tz=tz,
                qw=qw, qx=qx, qy=qy, qz=qz,
                wx=wx, wy=wy, wz=wz,
            ))
        out.append("        }\n    }\n}\n")
        sys.stdout.write("".join(out))
        sys.stderr.write(f"cubes20_envs: {NUM_CUBES} dynamic under /World/envs/template + ground\n")
        return 0

    rng = random.Random(SEED)
    out = [HEADER]

    # Spawn cubes in a 2 m × 2 m footprint so they're close enough to
    # collide on the way down. Height range chosen so the highest
    # cubes land last; varying heights → staggered impact times.
    for i in range(NUM_CUBES):
        tx = rng.uniform(-1.0, 1.0)
        tz = rng.uniform(-1.0, 1.0)
        ty = rng.uniform(0.5, 5.0)

        # Random orientation: pick a random unit axis + angle.
        ax = rng.uniform(-1.0, 1.0)
        ay = rng.uniform(-1.0, 1.0)
        az = rng.uniform(-1.0, 1.0)
        theta = rng.uniform(0.0, 2.0 * math.pi)
        qw, qx, qy, qz = quat_from_axis_angle(ax, ay, az, theta)

        # Random angular velocity (rad/s), bounded so the sim stays stable.
        wx = rng.uniform(-3.0, 3.0)
        wy = rng.uniform(-3.0, 3.0)
        wz = rng.uniform(-3.0, 3.0)

        out.append(CUBE_TEMPLATE.format(
            idx=i, size=CUBE_SIZE,
            tx=tx, ty=ty, tz=tz,
            qw=qw, qx=qx, qy=qy, qz=qz,
            wx=wx, wy=wy, wz=wz,
        ))

    out.append(FOOTER)
    sys.stdout.write("".join(out))
    sys.stderr.write(f"cubes20: {NUM_CUBES} dynamic + 1 static (ground)\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
