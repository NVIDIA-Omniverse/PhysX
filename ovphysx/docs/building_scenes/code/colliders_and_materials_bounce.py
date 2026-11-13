# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# Three balls with a radius of 0.1 m. Each ball's lowest point starts 1 m
# above the ground. Each ball has its own material; only the restitution
# differs.
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

    def Material "Clay" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {
        float physics:restitution = 0.0
        uniform token physxMaterial:restitutionCombineMode = "max"
    }

    def Material "Rubber" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {
        float physics:restitution = 0.5
        uniform token physxMaterial:restitutionCombineMode = "max"
    }

    def Material "Superball" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {
        float physics:restitution = 0.9
        uniform token physxMaterial:restitutionCombineMode = "max"
    }

    def Sphere "ClayBall" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
                              "MaterialBindingAPI"]
    )
    {
        double radius = 0.1
        double3 xformOp:translate = (-0.5, 0, 1.1)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        rel material:binding:physics = </World/Clay>
    }

    def Sphere "RubberBall" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
                              "MaterialBindingAPI"]
    )
    {
        double radius = 0.1
        double3 xformOp:translate = (0, 0, 1.1)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        rel material:binding:physics = </World/Rubber>
    }

    def Sphere "SuperBall" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
                              "MaterialBindingAPI"]
    )
    {
        double radius = 0.1
        double3 xformOp:translate = (0.5, 0, 1.1)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        rel material:binding:physics = </World/Superball>
    }
}
"""


# [scene-end]
def lowest_points(physx):
    """Return the height of each ball's lowest point, keyed by the ball's path."""
    heights = {}
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        for group in result.groups:
            paths = result.dictionary.get_path_strings(group.prim_list)
            centers = group.tensors[0].numpy()[:, 2]
            for path, center in zip(paths, centers):
                heights[path] = float(center) - 0.1  # subtract the radius
    return heights


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

stage = ovstage.Stage("bouncing_balls")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

# The balls reach the ground after about 0.45 s. From 0.5 s on, keep each
# ball's highest point: the top of its first bounce.
peaks = {}
for step in range(1, 91):
    physx.step_sync(1.0 / 60.0)
    if step >= 30:
        for path, height in lowest_points(physx).items():
            peaks[path] = max(peaks.get(path, 0.0), height)

for path in sorted(peaks):
    print(f"{path} bounces back to {peaks[path]:.3f} m")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
