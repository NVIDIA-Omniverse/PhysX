# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType


def box(i):
    """A 0.5 m box, the i-th from the bottom of the stack."""
    return f"""
    def Cube "Box{i}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
    )
    {{
        double size = 0.5
        double3 xformOp:translate = (0, 0, {0.25 + 0.5 * i})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
"""


# Six boxes stacked on the ground. The scene limits the solver to one
# position iteration, which makes the stack fragile.
SCENE = """#usda 1.0
(
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {
        uniform uint physxScene:maxPositionIterationCount = 1
    }

    def Plane "Ground" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        uniform token axis = "Z"
    }
""" + "".join(box(i) for i in range(6)) + "}\n"


def top_box(physx):
    """Return the x, y, z position of the top box."""
    with physx.read_sync(
        SimObjectType.RIGID_BODY, ["position"], pattern="/World/Box5"
    ) as result:
        x, y, z = result.groups[0].tensors[0].numpy()[0]
        return float(x), float(y), float(z)


def simulate(steps_per_second):
    """Simulate 5 s in steps of 1 / steps_per_second s.

    Report how far the top box moved.
    """
    stage = ovstage.Stage("stack")
    ovstage.population.open_usd_from_string_with_desc(
        stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
    )
    stage.advance_write_floor(ordinal=1).wait()
    physx = PhysX()
    physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

    x0, y0, z0 = top_box(physx)
    for _ in range(5 * steps_per_second):
        physx.step_sync(1.0 / steps_per_second)
    x, y, z = top_box(physx)
    sideways = ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
    print(
        f"1/{steps_per_second} s steps: top box moved {sideways:.3f} m sideways, "
        f"height {z0:.3f} m -> {z:.3f} m"
    )

    physx.wait_task(physx.detach_ovstage())
    physx.destroy()
    stage.destroy()


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
simulate(steps_per_second=60)
simulate(steps_per_second=240)
# [example-end]
