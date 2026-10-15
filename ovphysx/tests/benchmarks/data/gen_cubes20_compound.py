#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-006
# @covers AC-1

"""Generate the COMPOUND-body variant of the cubes20 env fixture: 20 rigid bodies per env, each
carrying SHAPES_PER_BODY colliders instead of one.

Why a separate fixture rather than reusing cubes20_envs.usda: every body there has exactly one
collider, which is the degenerate case for a per-shape read. A per-shape column is padded to the
widest body in the read, so at one shape per body there is no padding, no nested bodies x shapes
loop depth, and no way to see how the column scales with shape count -- i.e. it would measure
everything about that path except the part that is per-shape.

The spawn layout matches gen_cubes20.py: same seed, draw order, ranges, ground plane,
positions, orientations and angular velocities. Each body carries SHAPES_PER_BODY
colliders instead of being a collider itself. This fixture explicitly selects its
dynamics mode; compare scene settings before comparing timings with another family.

If you change the draw order or ranges, update gen_cubes20.py too or document the
different spawn layout.

Colliders are offset along X inside the body so they do not coincide.

Usage:
    python3 tests/benchmarks/data/gen_cubes20_compound.py > \
        tests/benchmarks/data/cubes20_compound_envs.usda

    python3 tests/benchmarks/data/gen_cubes20_compound.py --gpu > \
        tests/benchmarks/data/cubes20_compound_envs_gpu.usda
"""

import argparse
import math
import random

NUM_BODIES = 20            # == gen_cubes20.NUM_CUBES
SHAPES_PER_BODY = 4
CUBE_SIZE = 0.3            # == gen_cubes20.CUBE_SIZE
GROUND_HALF = 10.0         # == gen_cubes20.GROUND_HALF
SEED = 20260603            # == gen_cubes20.SEED -- must match, see the module docstring


def quat_from_axis_angle(ax, ay, az, theta):
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    s = math.sin(theta / 2.0)
    return (math.cos(theta / 2.0), ax * s, ay * s, az * s)


GPU_SCENE_SETTINGS = """        bool physxScene:enableGPUDynamics = true
        token physxScene:broadphaseType = "GPU"
"""


CPU_SCENE_SETTINGS = '''        bool physxScene:enableGPUDynamics = false
        token physxScene:broadphaseType = "MBP"
'''


def header(gpu: bool) -> str:
    return f"""#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
    doc = "Compound-body cubes20 template for cloning: {NUM_BODIES} bodies x {SHAPES_PER_BODY} colliders each. Pairs with cubes20_envs.usda, which is the same scene at one collider per body. See gen_cubes20_compound.py."
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
{GPU_SCENE_SETTINGS if gpu else CPU_SCENE_SETTINGS}    }}

    def Cube "GroundPlane" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        double size = 1.0
        double3 xformOp:scale = (200.0, 0.02, 200.0)
        double3 xformOp:translate = (0, -0.01, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}

    def Xform "envs"
    {{
        def Xform "template"
        {{
"""


BODY = """
            def Xform "body_{idx:02d}" (
                prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
            )
            {{
                float physics:mass = 1.0
                vector3f physics:angularVelocity = ({wx}, {wy}, {wz})
                double3 xformOp:translate = ({tx}, {ty}, {tz})
                quatd xformOp:orient = ({qw}, {qx}, {qy}, {qz})
                uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
{shapes}            }}
"""

SHAPE = """
                def Cube "col_{s}" (
                    prepend apiSchemas = ["PhysicsCollisionAPI"]
                )
                {{
                    double size = {size}
                    double3 xformOp:translate = ({ox}, 0, 0)
                    uniform token[] xformOpOrder = ["xformOp:translate"]
                }}
"""


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gpu", action="store_true", help="emit the GPU-dynamics scene settings")
    args = ap.parse_args()

    rng = random.Random(SEED)
    out = [header(args.gpu)]
    for i in range(NUM_BODIES):
        # Draw order and ranges are gen_cubes20.py's, exactly -- tx, tz, ty, then the axis/angle, then
        # the angular velocity. Reordering even two draws off the same seed gives different bodies and
        # silently breaks the control this fixture exists to provide.
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
        shapes = "".join(
            SHAPE.format(s=s, size=CUBE_SIZE, ox=round(s * CUBE_SIZE * 1.1, 4))
            for s in range(SHAPES_PER_BODY)
        )
        out.append(
            BODY.format(idx=i, wx=wx, wy=wy, wz=wz, tx=tx, ty=ty, tz=tz,
                        qw=qw, qx=qx, qy=qy, qz=qz, shapes=shapes)
        )
    out.append("        }\n    }\n}\n")
    print("".join(out), end="")


if __name__ == "__main__":
    main()
