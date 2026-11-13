# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx import PhysX
from ovphysx.population import physics_population_descs
from ovphysx.population import create_rigid_body_cubes
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


# [scene-end]
def box_heights(physx):
    """Read the position of every rigid body and return its z, keyed by path."""
    heights = {}
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        for group in result.groups:
            paths = result.dictionary.get_path_strings(group.prim_list)
            for path, position in zip(paths, group.tensors[0].numpy()):
                heights[path] = float(position[2])
    return heights


def add_second_box(stage, physx):
    """Create a 1 m box in the stage at ordinal 2, seal it, and apply it."""
    # The starting pose, a 4 by 4 matrix written row by row. Its last row
    # holds the position: 1.5 m beside the first box and 2 m up.
    start = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1.5, 0, 2, 1]
    create_rigid_body_cubes(
        stage,
        ovstage.PathDictionary(stage),  # maps path names to stage IDs
        ["/World/Box2"],
        ordinal=2,
        cube={"size": [1.0], "local_matrix": [start]},
        ancestors=False,  # /World is already in the stage
    )
    stage.advance_write_floor(ordinal=2).wait()
    physx.wait_task(physx.update_from_ovstage(2, 2))


def print_heights(time, heights):
    boxes = "  ".join(
        f"{path} z = {z:.3f} m" for path, z in sorted(heights.items())
    )
    print(f"t={time:.2f} s  {boxes}")


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

# Load the scene and seal it at ordinal 1, as in the first program.
stage = ovstage.Stage("changing_the_scene")
ovstage.population.open_usd_from_string_with_desc(
    stage,
    SCENE,
    ordinal=1,
    time=ovstage.population.DEFAULT_TIME,
    descs=physics_population_descs(rendering=True),
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

print_heights(0.0, box_heights(physx))
for step in range(1, 121):
    physx.step_sync(1.0 / 60.0)
    if step == 60:  # halfway through the 2 s run
        add_second_box(stage, physx)
        print("Added /World/Box2 at ordinal 2")
    if step % 15 == 0:
        print_heights(step / 60, box_heights(physx))

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
