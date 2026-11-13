# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# BoxA rests on the ground, BoxB is thrown up and to the side, and BoxC falls.
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

    def Cube "BoxA" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {
        double size = 0.4
        double3 xformOp:translate = (-1, 0, 0.2)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def Cube "BoxB" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {
        double size = 0.4
        vector3f physics:velocity = (1, 0, 3)
        double3 xformOp:translate = (0, 0, 1)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def Cube "BoxC" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {
        double size = 0.4
        double3 xformOp:translate = (1.5, 0, 3)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""


# [scene-end]
def read_bodies(physx):
    """Return {path: {attribute: [x, y, z]}} for every rigid body."""
    bodies = {}
    with physx.read_sync(
        SimObjectType.RIGID_BODY, ["position", "linearVelocity"]
    ) as result:
        names = result.dictionary  # turns ids into attribute names and paths
        for group in result.groups:
            attribute = names.token_to_string(group.attribute)
            paths = names.get_path_strings(group.prim_list)
            for path, row in zip(paths, group.tensors[0].numpy()):
                bodies.setdefault(path, {})[attribute] = row.tolist()  # a copy
    return bodies


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
stage = ovstage.Stage("reading_values")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

for step in range(30):  # simulate 0.5 s
    physx.step_sync(1.0 / 60.0)

print("object       position (m)            linear velocity (m/s)")
for path, values in read_bodies(physx).items():
    x, y, z = values["position"]
    vx, vy, vz = values["linearVelocity"]
    print(f"{path}  {x:6.3f} {y:6.3f} {z:6.3f}    {vx:6.3f} {vy:6.3f} {vz:6.3f}")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
