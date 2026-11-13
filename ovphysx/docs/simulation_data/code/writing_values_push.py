# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import numpy as np
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# Two 1 kg boxes side by side. Gravity is off, so only the writes move them.
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
        float physics:gravityMagnitude = 0
    }

    def Cube "VelocityBox" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
                              "PhysicsMassAPI"]
    )
    {
        float physics:mass = 1
        double size = 0.2
        double3 xformOp:translate = (0, -0.5, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def Cube "ForceBox" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
                              "PhysicsMassAPI"]
    )
    {
        float physics:mass = 1
        double size = 0.2
        double3 xformOp:translate = (0, 0.5, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""


# [scene-end]
def write_vector(physx, pattern, attribute, vector):
    """Write the same vector to every rigid body that the pattern selects."""
    with physx.write_sync(
        SimObjectType.RIGID_BODY, [attribute], pattern=pattern
    ) as session:
        for group in session.groups:
            rows = np.full((group.prim_count, 3), vector, dtype=np.float32)
            group.tensors[0].assign(rows)  # fill every row of the open array
            session.commit_sync(group)  # apply the values now


def print_motion(physx, step):
    """Print the x position and x velocity of both boxes."""
    x = {}
    with physx.read_sync(
        SimObjectType.RIGID_BODY, ["position", "linearVelocity"]
    ) as result:
        for group in result.groups:
            attribute = result.dictionary.token_to_string(group.attribute)
            paths = result.dictionary.get_path_strings(group.prim_list)
            for path, row in zip(paths, group.tensors[0].numpy()):
                x[path, attribute] = float(row[0])
    print(
        f"{step:4}   {x['/World/VelocityBox', 'position']:6.3f}"
        f"  {x['/World/VelocityBox', 'linearVelocity']:6.3f}"
        f"      {x['/World/ForceBox', 'position']:6.3f}"
        f"  {x['/World/ForceBox', 'linearVelocity']:6.3f}"
    )


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
stage = ovstage.Stage("writing_values")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

# Set VelocityBox moving at 1 m/s. Push ForceBox with 30 N, once.
write_vector(physx, "/World/VelocityBox", "linearVelocity", [1.0, 0.0, 0.0])
write_vector(physx, "/World/ForceBox", "force", [30.0, 0.0, 0.0])

print("        VelocityBox         ForceBox")
print("step    x (m)  vx (m/s)     x (m)  vx (m/s)")
print_motion(physx, 0)
for step in range(1, 61):
    physx.step_sync(1.0 / 60.0)
    if step in (1, 2, 3, 30, 60):
        print_motion(physx, step)

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
