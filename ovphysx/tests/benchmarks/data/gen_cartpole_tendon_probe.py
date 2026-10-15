#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-006
# @covers AC-1

"""Generate the Cartpole tendon OVPhysX probe fixture.

The cartpole of gen_cartpole_probe.py with one fixed and one spatial tendon added, so a
single articulation carries both kinds. Kept as a separate asset rather than folded into
cartpole_probe.usda because tendons are constraints: adding them there would change the
scene the existing artilink / artidof lanes measure.

Both tendons are authored with stiffness/damping/limits that leave the cartpole's motion
essentially unchanged. The fixture supplies both fixed and spatial tendon properties.

The source environment lives at /World/envs/env_0 and is cloned by the benchmark.

Usage:
    python3 tests/benchmarks/data/gen_cartpole_tendon_probe.py > \
        tests/benchmarks/data/cartpole_tendon_probe.usda
    python3 tests/benchmarks/data/gen_cartpole_tendon_probe.py --cpu > \
        tests/benchmarks/data/cartpole_tendon_probe_cpu.usda
"""

import sys

CART_SIZE = 0.4  # cube edge for the cart body
POLE_LENGTH = 1.0  # cylinder length for the pole
POLE_RADIUS = 0.04
TRACK_LIMIT = 2.5  # +/- prismatic joint limit, meters

# Select the dynamics and broadphase explicitly; omitted fields inherit schema defaults.
GPU_SCENE_ATTRS = """        bool physxScene:enableGPUDynamics = true
        token physxScene:broadphaseType = "GPU"
"""


CPU_SCENE_ATTRS = '''        bool physxScene:enableGPUDynamics = false
        token physxScene:broadphaseType = "MBP"
'''


def usda(gpu: bool) -> str:
    scene_attrs = GPU_SCENE_ATTRS if gpu else CPU_SCENE_ATTRS
    return f"""#usda 1.0
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
{scene_attrs}    }}

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

                # Root of the spatial tendon. The root attachment is the tendon: it is the prim
                # the read reports for OVPHYSX_QUERY_TARGET_SPATIAL_TENDON, and it carries the
                # stiffness / damping / limitStiffness / offset the lane reads.
                def Cube "cart" (
                    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI",
                                          "PhysxTendonAttachmentRootAPI:spatial0"]
                )
                {{
                    double size = {CART_SIZE}
                    float physics:mass = 1.0
                    float physxTendon:spatial0:stiffness = 0.5
                    float physxTendon:spatial0:damping = 0.1
                    float physxTendon:spatial0:limitStiffness = 0.2
                    float physxTendon:spatial0:offset = 0.0
                    # No gearing here: the schema ignores it on a root attachment.
                    point3f physxTendon:spatial0:localPos = (0, 0, 0)
                    double3 xformOp:translate = (0, 0.5, 0)
                    uniform token[] xformOpOrder = ["xformOp:translate"]
                }}

                # Leaf of the spatial tendon: a root alone is not a tendon, it needs somewhere to
                # pull to. parentLink names the PRIM carrying the parent attachment, and
                # parentAttachment its instance name.
                def Cylinder "pole" (
                    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI",
                                          "PhysxTendonAttachmentLeafAPI:spatial0Leaf"]
                )
                {{
                    uniform token axis = "Y"
                    double height = {POLE_LENGTH}
                    double radius = {POLE_RADIUS}
                    float physics:mass = 0.1
                    float physxTendon:spatial0Leaf:gearing = 1.0
                    point3f physxTendon:spatial0Leaf:localPos = (0, 0, 0)
                    uniform token physxTendon:spatial0Leaf:parentAttachment = "spatial0"
                    rel physxTendon:spatial0Leaf:parentLink = </World/envs/env_0/Robot/cart>
                    float physxTendon:spatial0Leaf:restLength = 0.5
                    float physxTendon:spatial0Leaf:lowerLimit = -1.0
                    float physxTendon:spatial0Leaf:upperLimit = 1.0
                    double3 xformOp:translate = (0, {0.5 + POLE_LENGTH / 2.0}, 0)
                    uniform token[] xformOpOrder = ["xformOp:translate"]
                }}

                def PhysicsFixedJoint "root_joint"
                {{
                    rel physics:body1 = </World/envs/env_0/Robot/base>
                }}

                # Root axis of the fixed tendon, on the first movable joint. As with the spatial
                # root, this prim is what the read reports for OVPHYSX_QUERY_TARGET_FIXED_TENDON, and it
                # carries the tendon-level properties; the child axis below only adds gearing.
                def PhysicsPrismaticJoint "slider_to_cart" (
                    prepend apiSchemas = ["PhysicsDriveAPI:linear", "PhysxTendonAxisRootAPI:fixed0"]
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
                    float physxTendon:fixed0:stiffness = 0.5
                    float physxTendon:fixed0:damping = 0.1
                    float physxTendon:fixed0:limitStiffness = 0.2
                    float physxTendon:fixed0:offset = 0.0
                    float physxTendon:fixed0:restLength = 0.0
                    float physxTendon:fixed0:lowerLimit = -1.0
                    float physxTendon:fixed0:upperLimit = 1.0
                    float[] physxTendon:fixed0:gearing = [1.0]
                }}

                def PhysicsRevoluteJoint "cart_to_pole" (
                    prepend apiSchemas = ["PhysicsDriveAPI:angular", "PhysxTendonAxisAPI:fixed0"]
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
                    float[] physxTendon:fixed0:gearing = [1.0]
                }}
            }}
        }}
    }}
}}
"""


def main() -> int:
    sys.stdout.write(usda(gpu="--cpu" not in sys.argv[1:]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
