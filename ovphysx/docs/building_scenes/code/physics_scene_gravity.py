# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# A 1 m box whose center starts 2 m above the ground.
# GRAVITY is replaced by a number before the scene is loaded.
SCENE = """#usda 1.0
(
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene"
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = GRAVITY
    }

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
        return float(result.groups[0].tensors[0].numpy()[0, 2])


def drop_box(gravity):
    """Simulate 1.5 s with this gravity and return the height every 0.25 s."""
    stage = ovstage.Stage("drop")
    scene = SCENE.replace("GRAVITY", str(gravity))
    ovstage.population.open_usd_from_string_with_desc(
        stage, scene, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
    )
    stage.advance_write_floor(ordinal=1).wait()
    physx = PhysX()
    physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

    heights = [box_height(physx)]
    for step in range(1, 91):
        physx.step_sync(1.0 / 60.0)
        if step % 15 == 0:
            heights.append(box_height(physx))

    physx.wait_task(physx.detach_ovstage())
    physx.destroy()
    stage.destroy()
    return heights


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

earth = drop_box(9.81)
moon = drop_box(1.62)
print("time    Earth    Moon")
for i, (earth_z, moon_z) in enumerate(zip(earth, moon)):
    print(f"{i * 0.25:.2f} s  {earth_z:.3f} m  {moon_z:.3f} m")
# [example-end]
