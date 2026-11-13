# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType
from ovphysx.utils import step_and_write_to_ovstage

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
# [stage-read-start]


def stage_height(stage, ordinal):
    """Read the box's world transform back from the stage and return its z."""
    with (
        ovstage.PathDictionary(stage) as names,
        names.create_path_list_from_strings(["/World/Box"]) as box,
        stage.query_from_path_list(box) as query,
        stage.read_attributes(
            query,
            [names.intern_token("omni:fabric:worldMatrix")],
            ovstage.OrdinalRange.latest(ordinal),
        ) as read,
    ):
        for group in read.wait().groups():
            with group:
                # A 4x4 matrix; its last row holds the position.
                matrix = group.columns[0].array(0)[0]
                return float(matrix[3, 2])


# [stage-read-end]
# ovstage must know the PhysX USD schemas before it reads a scene.
ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

# Load the scene, including what a renderer needs, and seal it at ordinal 1.
stage = ovstage.Stage("write_back")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

print(f"t=0.00 s  box z in the stage = {stage_height(stage, 1):.3f} m")
for step in range(1, 61):
    # Step 1/60 s, write each rigid body's pose into the stage at a new
    # ordinal, and seal that ordinal.
    ordinal = 1 + step
    step_and_write_to_ovstage(
        physx,
        dt=1.0 / 60.0,
        output_ordinal=ordinal,
        outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
    )
    if step % 10 == 0:
        z = stage_height(stage, ordinal)
        print(f"t={step / 60:.2f} s  box z in the stage = {z:.3f} m")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
