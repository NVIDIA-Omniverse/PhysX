#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Generate a synthetic cartpole USDA fixture for ovphysx Lab.* benchmarks.

Matches the topology IsaacLab uses for its cartpole environment: a single
articulation with a static base, a prismatic cart joint, and a revolute
pole joint. The articulation lives under /World/envs/template/cartpole so the
benchmark can clone() it into env1..envN (grid-separated) for the
Lab.cartpole_<N>_* benchmarks.

Usage:
    python3 tests/benchmarks/data/gen_cartpole.py > \
        tests/benchmarks/data/cartpole.usda
"""

import sys

CART_SIZE = 0.4       # cube edge for the cart body
POLE_LENGTH = 1.0     # cylinder length for the pole
POLE_RADIUS = 0.04
TRACK_LIMIT = 2.5     # +/- prismatic joint limit, meters


USDA = f"""#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
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
        # GPU dynamics + GPU broadphase: the Lab.* benches are GPU-only (forceGpu()), and
        # this is the batched-RL path they measure. Env copies are grid-separated by the
        # clone() per-env transforms, so cross-env collisions need no env-id filtering.
        bool physxScene:enableGPUDynamics = true
        token physxScene:broadphaseType = "GPU"
    }}

    def Scope "envs"
    {{
        def Xform "template"
        {{
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]

            def Xform "cartpole" (
                prepend apiSchemas = ["PhysicsArticulationRootAPI"]
            )
            {{
                def Cube "base" (
                    prepend apiSchemas = ["PhysicsCollisionAPI"]
                )
                {{
                    double size = 0.2
                    double3 xformOp:translate = (0, 0.1, 0)
                    uniform token[] xformOpOrder = ["xformOp:translate"]
                }}

                def Cube "cart" (
                    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
                )
                {{
                    double size = {CART_SIZE}
                    float physics:mass = 1.0
                    double3 xformOp:translate = (0, 0.5, 0)
                    uniform token[] xformOpOrder = ["xformOp:translate"]
                }}

                def Cylinder "pole" (
                    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"]
                )
                {{
                    uniform token axis = "Y"
                    double height = {POLE_LENGTH}
                    double radius = {POLE_RADIUS}
                    float physics:mass = 0.1
                    double3 xformOp:translate = (0, {0.5 + POLE_LENGTH / 2.0}, 0)
                    uniform token[] xformOpOrder = ["xformOp:translate"]
                }}

                def PhysicsPrismaticJoint "cartJoint"
                {{
                    rel physics:body0 = </World/envs/template/cartpole/base>
                    rel physics:body1 = </World/envs/template/cartpole/cart>
                    uniform token physics:axis = "X"
                    float physics:lowerLimit = {-TRACK_LIMIT}
                    float physics:upperLimit = {TRACK_LIMIT}
                }}

                def PhysicsRevoluteJoint "poleJoint"
                {{
                    rel physics:body0 = </World/envs/template/cartpole/cart>
                    rel physics:body1 = </World/envs/template/cartpole/pole>
                    uniform token physics:axis = "Z"
                }}
            }}
        }}
    }}
}}
"""


def main() -> int:
    sys.stdout.write(USDA)
    return 0


if __name__ == "__main__":
    sys.exit(main())
