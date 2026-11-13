# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# Two boxes under Shelf, and the ground and a ball under Floor.
# The boxes and the ball are rigid bodies; the ground is not.
SCENE = """#usda 1.0
(
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene" {}

    def Xform "Shelf"
    {
        def Cube "BoxA" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
        )
        {
            double size = 0.2
            double3 xformOp:translate = (-0.3, 0, 1.1)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Cube "BoxB" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
        )
        {
            double size = 0.2
            double3 xformOp:translate = (0.3, 0, 1.1)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }
    }

    def Xform "Floor"
    {
        def Plane "Ground" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {
            uniform token axis = "Z"
        }

        def Sphere "Ball" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
        )
        {
            double radius = 0.1
            double3 xformOp:translate = (0, 0, 0.1)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }
    }
}
"""


# [scene-end]
def selected_paths(physx, pattern):
    """Return the path of every rigid body that the pattern selects."""
    paths = []
    with physx.read_sync(
        SimObjectType.RIGID_BODY, ["position"], pattern=pattern
    ) as result:
        for group in result.groups:
            paths += result.dictionary.get_path_strings(group.prim_list)
    return paths


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
stage = ovstage.Stage("selecting_objects")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

for pattern in ["/World/Shelf/*", "/World/**", "/World/Floor/Ball", "/World/Box*"]:
    print(f"{pattern:<18} {', '.join(selected_paths(physx, pattern))}")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
