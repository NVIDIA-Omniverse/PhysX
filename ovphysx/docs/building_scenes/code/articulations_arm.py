# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# A robot arm that turns in a horizontal plane, like a pick-and-place robot:
# a base fixed to the world and two links on vertical revolute joints.
# The links are written in the zero pose, with the arm pointing along +X.
SCENE = """#usda 1.0
(
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene" {}

    def Xform "Arm" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    )
    {
        def Cube "Base" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
        )
        {
            double size = 0.2
            double3 xformOp:translate = (0, 0, 0.1)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def PhysicsFixedJoint "BaseJoint"
        {
            rel physics:body1 = </World/Arm/Base>
        }

        def Capsule "UpperArm" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
        )
        {
            uniform token axis = "X"
            double height = 0.4
            double radius = 0.05
            double3 xformOp:translate = (0.25, 0, 0.25)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Capsule "Forearm" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
        )
        {
            uniform token axis = "X"
            double height = 0.4
            double radius = 0.05
            double3 xformOp:translate = (0.75, 0, 0.25)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def PhysicsRevoluteJoint "Shoulder" (
            prepend apiSchemas = ["PhysicsDriveAPI:angular"]
        )
        {
            uniform token physics:axis = "Z"
            rel physics:body0 = </World/Arm/Base>
            rel physics:body1 = </World/Arm/UpperArm>
            point3f physics:localPos0 = (0, 0, 0.15)
            point3f physics:localPos1 = (-0.25, 0, 0)
            float drive:angular:physics:stiffness = 1000
            float drive:angular:physics:damping = 200
        }

        def PhysicsRevoluteJoint "Elbow" (
            prepend apiSchemas = ["PhysicsDriveAPI:angular",
                                  "PhysicsJointStateAPI:angular"]
        )
        {
            uniform token physics:axis = "Z"
            rel physics:body0 = </World/Arm/UpperArm>
            rel physics:body1 = </World/Arm/Forearm>
            point3f physics:localPos0 = (0.25, 0, 0)
            point3f physics:localPos1 = (-0.25, 0, 0)
            float drive:angular:physics:stiffness = 1000
            float drive:angular:physics:damping = 200
            float drive:angular:physics:targetPosition = 90
            float state:angular:physics:position = 90
        }
    }
}
"""


# [scene-end]
def joint_angle(physx, joint):
    """Read the angle of one joint, in degrees."""
    with physx.read_sync(
        SimObjectType.ARTICULATION_JOINT,
        ["jointPosition"],
        pattern=f"/World/Arm/{joint}",
    ) as result:
        angles = result.groups[0].tensors[0].numpy()  # one row per joint axis
        return float(angles[0])


def set_target(physx, joint, degrees):
    """Write the drive target of one joint, in degrees."""
    with physx.write_sync(
        SimObjectType.ARTICULATION_JOINT,
        ["jointPositionTarget"],
        pattern=f"/World/Arm/{joint}",
    ) as session:
        group = session.groups[0]
        group.tensors[0].assign([degrees])  # one row: the joint's only axis
        session.commit_sync(group)


def print_angles(physx, t):
    shoulder = joint_angle(physx, "Shoulder")
    elbow = joint_angle(physx, "Elbow")
    print(f"t={t:.2f} s  shoulder = {shoulder:5.1f} deg  elbow = {elbow:5.1f} deg")


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

stage = ovstage.Stage("articulation_arm")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

# Set the drive targets once. They stay until the program writes new ones.
set_target(physx, "Shoulder", 45.0)
set_target(physx, "Elbow", 60.0)

print_angles(physx, 0.0)
for step in range(1, 91):
    physx.step_sync(1.0 / 60.0)
    if step % 15 == 0:
        print_angles(physx, step / 60)

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
