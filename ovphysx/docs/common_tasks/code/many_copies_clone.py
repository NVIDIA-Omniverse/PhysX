# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# A ground plane and one environment, env0, with a 1 m box 2 m above the ground.
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

    def Xform "envs"
    {
        def Xform "env0"
        {
            def Cube "Box" (
                prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
            )
            {
                double size = 1
                double3 xformOp:translate = (0, 0, 2)
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }
        }
    }
}
"""


def box_heights(physx):
    """Read the box of every copy with one path pattern.

    Return its z, keyed by path.
    """
    heights = {}
    with physx.read_sync(
        SimObjectType.RIGID_BODY, ["position"], pattern="/World/envs/*/Box"
    ) as result:
        for group in result.groups:
            paths = result.dictionary.get_path_strings(group.prim_list)
            for path, position in zip(paths, group.tensors[0].numpy()):
                heights[path] = float(position[2])
    return heights


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

stage = ovstage.Stage("many_copies")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

# Copy env0 to env1 through env15 on a 4 by 4 grid, 2.5 m apart,
# before the first step.
# Each pose is a position (x, y, z) and a rotation quaternion (x, y, z, w).
targets = [f"/World/envs/env{i}" for i in range(1, 16)]
poses = [
    (2.5 * (i % 4), 2.5 * (i // 4), 0.0, 0.0, 0.0, 0.0, 1.0)
    for i in range(1, 16)
]
physx.wait_task(physx.clone("/World/envs/env0", targets, anchor_transforms=poses))

print(f"One read returns {len(box_heights(physx))} boxes.")
for step in range(60):
    physx.step_sync(1.0 / 60.0)

heights = box_heights(physx)
print("Box heights after 1 s, laid out like the grid:")
for i in range(16):
    z = heights[f"/World/envs/env{i}/Box"]
    print(f"  env{i:<2} {z:.3f} m", end="\n" if i % 4 == 3 else " ")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
