# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import time

import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

COUNT = 400

HEADER = """#usda 1.0
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
"""

BOX = """
    def Cube "Box{i}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {{
        double size = 0.5
        double3 xformOp:translate = ({x}, {y}, 1)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
"""

# 400 boxes in a 20 x 20 grid, 1 m apart and 1 m above the ground.
BOXES = "".join(BOX.format(i=i, x=i % 20, y=i // 20) for i in range(COUNT))
SCENE = HEADER + BOXES + "}\n"


def read_all(physx):
    """Read the positions of all boxes with one read."""
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        return result.groups[0].tensors[0].numpy().copy()


def read_one(physx, i):
    """Read the position of one box."""
    with physx.read_sync(
        SimObjectType.RIGID_BODY, ["position"], pattern=f"/World/Box{i}"
    ) as result:
        return result.groups[0].tensors[0].numpy().copy()


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

stage = ovstage.Stage("fast_reads")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
# The first read also loads NVIDIA Warp, so time the reads after it.
read_all(physx)

start = time.perf_counter()
read_all(physx)
one_read = time.perf_counter() - start

start = time.perf_counter()
for i in range(COUNT):
    read_one(physx, i)
many_reads = time.perf_counter() - start

print(f"one read of {COUNT} boxes: {one_read * 1000:7.1f} ms")
print(f"{COUNT} reads of one box:  {many_reads * 1000:7.1f} ms")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
