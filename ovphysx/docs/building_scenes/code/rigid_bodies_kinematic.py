# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import math

import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# A static ground, a kinematic platform 1 m above it,
# and a dynamic box on the platform.
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

    def Cube "Platform" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {
        bool physics:kinematicEnabled = true
        double size = 1
        double3 xformOp:translate = (1, 0, 1)
        double3 xformOp:scale = (2, 1, 0.2)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }

    def Cube "Box" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {
        double size = 0.4
        double3 xformOp:translate = (1, 0, 1.3)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""


# [scene-end]
def print_x(physx, t):
    """Print the x position of the platform and of the box."""
    x = {}
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        for group in result.groups:
            names = result.dictionary.get_path_strings(group.prim_list)
            for name, position in zip(names, group.tensors[0].numpy()):
                x[name] = float(position[0])
    platform_x, box_x = x["/World/Platform"], x["/World/Box"]
    print(f"t={t:.0f} s  platform x = {platform_x:.2f} m  box x = {box_x:.2f} m")


def move_platform(physx, x):
    """Write where the platform must be at the end of the next step."""
    with physx.write_sync(
        SimObjectType.RIGID_BODY,
        ["kinematicTargetPosition"],
        pattern="/World/Platform",
    ) as session:
        for group in session.groups:
            group.tensors[0].assign([[x, 0.0, 1.0]])
            session.commit_sync(group)


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

stage = ovstage.Stage("platform")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

# Slide the platform from x = 1 m to 2 m and back in 4 s, one target per step.
print_x(physx, 0)
for step in range(1, 241):
    t = step / 60.0
    move_platform(physx, 1.5 - 0.5 * math.cos(math.pi * t / 2.0))
    physx.step_sync(1.0 / 60.0)
    if step % 60 == 0:
        print_x(physx, t)

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
