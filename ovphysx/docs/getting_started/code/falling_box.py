# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# A ground plane and a 1 m box whose center starts 2 m above it.
SCENE = """#usda 1.0
(
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene" {}

    def Plane "Ground" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        uniform token axis = "Z"
    }

    def Cube "Box" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {
        double size = 1
        double3 xformOp:translate = (0, 0, 2)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""


def box_height(physx):
    """Read the position of every rigid body and return the box's z."""
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        positions = result.groups[0].tensors[0].numpy()  # one row per body
        return float(positions[0, 2])


# ovstage must know the PhysX USD schemas before it reads a scene.
ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

# Load the scene into a stage, then seal the edits at ordinal 1.
stage = ovstage.Stage("falling_box")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

# Create the simulation from the sealed stage.
physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

print(f"t=0.00 s  box z = {box_height(physx):.3f} m")
for step in range(1, 61):
    physx.step_sync(1.0 / 60.0)
    if step % 10 == 0:
        print(f"t={step / 60:.2f} s  box z = {box_height(physx):.3f} m")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
