# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import math

import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# A 1 m arm that lies along the x axis. A hinge at the origin lets it turn
# around the vertical z axis, and a drive turns it to 90 degrees.
SCENE = """#usda 1.0
(
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene" {}

    def Capsule "Arm" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {
        double radius = 0.05
        double height = 0.9
        uniform token axis = "X"
        double3 xformOp:translate = (0.5, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def PhysicsRevoluteJoint "Hinge" (
        prepend apiSchemas = ["PhysicsDriveAPI:angular"]
    )
    {
        rel physics:body1 = </World/Arm>
        uniform token physics:axis = "Z"
        point3f physics:localPos0 = (0, 0, 0)
        point3f physics:localPos1 = (-0.5, 0, 0)
        float drive:angular:physics:targetPosition = 90
        float drive:angular:physics:stiffness = 2
        float drive:angular:physics:damping = 0.6
    }
}
"""


# [scene-end]
def arm_angle(physx):
    """Read the arm's center and return its angle around the hinge in degrees."""
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        x, y, _ = result.groups[0].tensors[0].numpy()[0]
        return math.degrees(math.atan2(y, x))


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

stage = ovstage.Stage("driven_arm")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

print(f"t=0.00 s  angle = {arm_angle(physx):3.0f} deg")
for step in range(1, 97):
    physx.step_sync(1.0 / 60.0)
    if step % 12 == 0:
        print(f"t={step / 60:.2f} s  angle = {arm_angle(physx):3.0f} deg")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
