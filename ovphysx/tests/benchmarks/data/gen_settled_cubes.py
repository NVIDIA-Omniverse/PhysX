#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Generate a benchmark fixture with a STABLE active subset: per env, most
cubes are kinematic (never active), a few fall and stay active.

The point is OVPHYSX_SCOPE_ACTIVE. cubes20 drops every cube from a height,
so all of them are moving and SCOPE_ACTIVE returns the whole scene -- no
subset to fetch. This scene splits each env into two populations whose
membership does not drift across the benchmark window:

  - KINEMATIC (physics:kinematicEnabled = true): resting on the ground and
    never given a target, so they never move. A kinematic body is excluded
    from the active set by rule -- the read's SCOPE_ACTIVE check drops
    eKINEMATIC -- so a SCOPE_ACTIVE read never fetches them.

  - FALLING: dynamic, spawned high with zero velocity and left to fall. A
    falling body is unambiguously moving, so it is in the active set on
    every step.

Kinematic rather than asleep for the inactive population because a
DirectGPU scene (GPU dynamics + suppressReadback) raises
eDISABLE_SLEEPING and clears eENABLE_ACTIVE_ACTORS: nothing can sleep, and
the active set falls back to `!(kinematic || sleeping)`. So on DirectGPU a
dynamic body is ALWAYS active and the only way to be inactive is to be
kinematic. Kinematic works on CPU too (a stationary kinematic is not in
getActiveActors), so one asset serves both device passes. And free fall
rather than "dynamic at rest" for the active population because a resting
dynamic sleeps on CPU (dropping out of the active set) even though it stays
awake on the sleepless DirectGPU scene; a falling one is active on both.

The spawn height keeps the falling cubes airborne well past the longest
run (getNbSteps() is 20 at 8192 envs, 0.33 s; --steps could push it), and
they are offset from the kinematics so a long run lands them on empty
ground rather than on a kinematic.

So SCOPE_ALL reads NUM_CUBES per env and SCOPE_ACTIVE reads NUM_FALLING,
a fixed fraction, on ONE scene: the readonly_settled_rb / readonly_active_rb
pair reads the same bodies through two scopes and their ratio is the
fetch-reduction win of "fetch only the requested rows" (TASK-16).

Initial state uses a fixed random seed so the fixture is reproducible.

Usage:
    python3 tests/benchmarks/data/gen_settled_cubes.py --envs > \
        tests/benchmarks/data/settled_cubes_envs.usda

    python3 tests/benchmarks/data/gen_settled_cubes.py --envs --gpu > \
        tests/benchmarks/data/settled_cubes_envs_gpu.usda
"""

import argparse
import math
import random
import sys

NUM_CUBES = 20  # per env, matching cubes20 so the row counts line up
NUM_FALLING = 4  # the rest are kinematic; 4/20 = a 20% active subset
CUBE_SIZE = 0.3
REST_Y = CUBE_SIZE / 2.0  # a cube resting on the ground plane at y=0
FALL_Y = 20.0  # high enough to stay airborne past the longest run
SEED = 20260910  # fixed seed: same output every regeneration


def quat_from_axis_angle(ax, ay, az, theta):
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    s = math.sin(theta / 2.0)
    return (math.cos(theta / 2.0), ax * s, ay * s, az * s)


# The GPU variant differs from the CPU one ONLY by these scene settings, as in gen_cubes20.py.
GPU_SCENE_SETTINGS = """        bool physxScene:enableGPUDynamics = true
        string physxScene:broadphaseType = "GPU"
"""


def _envs_header(gpu: bool = False) -> str:
    gpu_settings = GPU_SCENE_SETTINGS if gpu else ""
    return f"""#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
    doc = "Settled-subset fixture for cloning. Per env: {NUM_CUBES - NUM_FALLING} kinematic cubes on the ground, {NUM_FALLING} falling dynamics, so SCOPE_ACTIVE reads a fixed {NUM_FALLING}/{NUM_CUBES} subset. Template lives at /World/envs/template for clone() into env1..envN. See gen_settled_cubes.py."
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
{gpu_settings}    }}

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


# Kinematic on the ground: physics:kinematicEnabled and never given a target, so it never moves and
# the SCOPE_ACTIVE check (which drops eKINEMATIC) never fetches it -- on CPU and DirectGPU alike.
_KINEMATIC_CUBE_TEMPLATE = """
            def Cube "cube_{idx:02d}" (
                prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
            )
            {{
                double size = {size}
                float physics:mass = 1.0
                bool physics:kinematicEnabled = 1
                double3 xformOp:translate = ({tx}, {ty}, {tz})
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }}
"""


# Falling from a height with zero initial velocity: awake and moving on every step, so always active.
_FALLING_CUBE_TEMPLATE = """
            def Cube "cube_{idx:02d}" (
                prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
            )
            {{
                double size = {size}
                float physics:mass = 1.0
                double3 xformOp:translate = ({tx}, {ty}, {tz})
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }}
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envs", action="store_true",
                        help="emit clonable variant with template under /World/envs/template")
    parser.add_argument("--gpu", action="store_true",
                        help="with --envs: enable GPU dynamics and GPU broadphase on the scene")
    args = parser.parse_args()

    if not args.envs:
        sys.stderr.write("gen_settled_cubes only emits the --envs (clonable) variant\n")
        return 2

    rng = random.Random(SEED)
    out = [_envs_header(gpu=args.gpu)]
    for i in range(NUM_CUBES):
        if i < NUM_FALLING:
            # Falling cubes share an x-band well clear of the kinematics, so even a long run lands
            # them on empty ground rather than on a kinematic.
            tx = 2.0 + rng.uniform(-0.4, 0.4)
            tz = rng.uniform(-0.4, 0.4)
            out.append(_FALLING_CUBE_TEMPLATE.format(
                idx=i, size=CUBE_SIZE, tx=tx, ty=FALL_Y, tz=tz))
        else:
            # Kinematics on a small grid on the ground; spacing is cosmetic (they never move or
            # collide), it just keeps them from sharing a location.
            slot = i - NUM_FALLING
            gx = slot % 4
            gz = slot // 4
            tx = -2.0 + gx * 0.5
            tz = -1.0 + gz * 0.5
            out.append(_KINEMATIC_CUBE_TEMPLATE.format(
                idx=i, size=CUBE_SIZE, tx=tx, ty=REST_Y, tz=tz))
    out.append("        }\n    }\n}\n")
    sys.stdout.write("".join(out))
    sys.stderr.write(f"settled_cubes_envs{'_gpu' if args.gpu else ''}: {NUM_CUBES - NUM_FALLING} kinematic + "
                     f"{NUM_FALLING} falling per env under /World/envs/template + ground\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
