#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Generate the synthetic Cartpole-4096 OVPhysX probe fixture.

The source environment lives at /World/envs/env_0 and is cloned into 4095
targets by the benchmark. The two joint names and drive parameters match the
IsaacLab Cartpole configuration. The compact Y-up geometry remains synthetic.

Usage:
    python3 tests/benchmarks/data/gen_cartpole_probe.py > \
        tests/benchmarks/data/cartpole_probe.usda
"""

import sys

CART_SIZE = 0.4  # cube edge for the cart body
POLE_LENGTH = 1.0  # cylinder length for the pole
POLE_RADIUS = 0.04
TRACK_LIMIT = 2.5  # +/- prismatic joint limit, meters


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
        uint physxScene:timeStepsPerSecond = 120
        custom int physxScene:envIdInBoundsBitCount = 4
        # GPU dynamics and broadphase match the batched control-step probe.
        bool physxScene:enableGPUDynamics = true
        token physxScene:broadphaseType = "GPU"
    }}

    def Scope "envs"
    {{
        def Xform "env_0"
        {{
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]

            def Xform "Robot" (
                prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
            )
            {{
                bool physxArticulation:enabledSelfCollisions = false
                int physxArticulation:solverPositionIterationCount = 4
                int physxArticulation:solverVelocityIterationCount = 0
                float physxArticulation:sleepThreshold = 0.005
                float physxArticulation:stabilizationThreshold = 0.001

                def Cube "base" (
                    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
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

                def PhysicsFixedJoint "root_joint"
                {{
                    rel physics:body1 = </World/envs/env_0/Robot/base>
                }}

                def PhysicsPrismaticJoint "slider_to_cart" (
                    prepend apiSchemas = ["PhysicsDriveAPI:linear"]
                )
                {{
                    rel physics:body0 = </World/envs/env_0/Robot/base>
                    rel physics:body1 = </World/envs/env_0/Robot/cart>
                    uniform token physics:axis = "X"
                    float physics:lowerLimit = {-TRACK_LIMIT}
                    float physics:upperLimit = {TRACK_LIMIT}
                    float drive:linear:physics:damping = 10
                    float drive:linear:physics:maxForce = 400
                    float drive:linear:physics:stiffness = 0
                    float drive:linear:physics:targetPosition = 0
                    float drive:linear:physics:targetVelocity = 0
                    uniform token drive:linear:physics:type = "force"
                }}

                def PhysicsRevoluteJoint "cart_to_pole" (
                    prepend apiSchemas = ["PhysicsDriveAPI:angular"]
                )
                {{
                    rel physics:body0 = </World/envs/env_0/Robot/cart>
                    rel physics:body1 = </World/envs/env_0/Robot/pole>
                    uniform token physics:axis = "Z"
                    float drive:angular:physics:damping = 0
                    float drive:angular:physics:maxForce = 400
                    float drive:angular:physics:stiffness = 0
                    float drive:angular:physics:targetPosition = 0
                    float drive:angular:physics:targetVelocity = 0
                    uniform token drive:angular:physics:type = "force"
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
