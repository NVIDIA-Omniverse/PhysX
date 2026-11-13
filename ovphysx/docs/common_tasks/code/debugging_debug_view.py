# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import numpy as np
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# The scene from Your First Simulation: a ground plane and a 1 m box whose
# center starts 2 m above it.
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


def set_debug(physx, name, value):
    """Set one debug visualization setting, such as "collisionShapes"."""
    with physx.write_sync(SimObjectType.DEBUG_RENDER, [name]) as session:
        for group in session.groups:
            dtype = np.uint8 if name == "enabled" else np.float32
            group.tensors[0].assign(np.array([value], dtype=dtype))
            session.commit_sync(group)


def lines_by_color(physx):
    """Read the debug lines and count them by color (0xAARRGGBB)."""
    with physx.read_sync(SimObjectType.DEBUG_RENDER, ["lineColor"]) as result:
        colors = result.groups[0].tensors[0].numpy()[:, 0]  # first end of each line
        values, counts = np.unique(colors, return_counts=True)
        return ", ".join(f"{n} in {int(v):08x}" for v, n in zip(values, counts))


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
stage = ovstage.Stage("debug_view")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

# Turn on the debug view, then three of its flags.
set_debug(physx, "enabled", 1)
set_debug(physx, "collisionShapes", 1.0)
set_debug(physx, "contactPoint", 1.0)
set_debug(physx, "contactNormal", 1.5)  # 1.5 m long normals, past the 1 m box

for step in range(1, 46):
    physx.step_sync(1.0 / 60.0)
    if step in (30, 45):  # still falling, then resting on the ground
        print(f"step {step}: lines {lines_by_color(physx)}")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
