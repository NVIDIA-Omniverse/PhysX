# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# A 2 kg box, 0.5 m wide, falls onto a static plate. The box is a contact sensor.
SCENE = """#usda 1.0
(
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {
        bool physxScene:disableSleeping = 1
    }

    def Cube "Plate" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        double size = 1
        double3 xformOp:translate = (0, 0, -0.05)
        float3 xformOp:scale = (2, 2, 0.1)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }

    def Cube "Box" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
                              "PhysicsMassAPI", "PhysxContactReportAPI"]
    )
    {
        double size = 0.5
        float physics:mass = 2
        float physxContactReport:threshold = 0
        double3 xformOp:translate = (0, 0, 1)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""


def contact_force(physx):
    """Read the net contact force on every sensor.

    Return the box's z, in newtons.
    """
    with physx.read_sync(
        SimObjectType.CONTACT_SENSOR, ["netNormalContactForce"]
    ) as result:
        forces = result.groups[0].tensors[0].numpy()  # one row per sensor
        return float(forces[0, 2])


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

stage = ovstage.Stage("contacts")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

print(f"t=0.00 s  contact force z = {contact_force(physx):7.2f} N")
for step in range(1, 61):
    physx.step_sync(1.0 / 60.0)
    if step % 5 == 0:
        force = contact_force(physx)
        print(f"t={step / 60:.2f} s  contact force z = {force:7.2f} N")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
