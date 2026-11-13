# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import PhysX
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# A soft cube with 0.5 m sides, whose bottom starts 0.75 m above the ground.
# Its simulation mesh joins the 8 corners into 5 tetrahedra.
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

    def Material "SoftMaterial" (
        prepend apiSchemas = ["OmniPhysicsDeformableMaterialAPI",
                              "PhysxDeformableMaterialAPI"]
    )
    {
        float omniphysics:youngsModulus = 20000
        float omniphysics:poissonsRatio = 0.45
        float physxDeformableMaterial:elasticityDamping = 0.05
    }

    def TetMesh "SoftCube" (
        prepend apiSchemas = ["OmniPhysicsDeformableBodyAPI",
                              "OmniPhysicsVolumeDeformableSimAPI",
                              "PhysicsCollisionAPI", "MaterialBindingAPI"]
    )
    {
        point3f[] points = [
            (0, 0, 0.75), (0.5, 0, 0.75), (0.5, 0.5, 0.75), (0, 0.5, 0.75),
            (0, 0, 1.25), (0.5, 0, 1.25), (0.5, 0.5, 1.25), (0, 0.5, 1.25)
        ]
        int4[] tetVertexIndices = [
            (0, 1, 3, 4), (1, 2, 3, 6), (1, 4, 5, 6), (3, 4, 6, 7), (1, 3, 4, 6)
        ]
        int3[] surfaceFaceVertexIndices = [
            (0, 3, 1), (0, 1, 4), (0, 4, 3), (1, 3, 2), (1, 2, 6), (2, 3, 6),
            (1, 5, 4), (1, 6, 5), (4, 5, 6), (3, 4, 7), (3, 7, 6), (4, 6, 7)
        ]
        point3f[] omniphysics:restShapePoints = [
            (0, 0, 0.75), (0.5, 0, 0.75), (0.5, 0.5, 0.75), (0, 0.5, 0.75),
            (0, 0, 1.25), (0.5, 0, 1.25), (0.5, 0.5, 1.25), (0, 0.5, 1.25)
        ]
        int4[] omniphysics:restTetVtxIndices = [
            (0, 1, 3, 4), (1, 2, 3, 6), (1, 4, 5, 6), (3, 4, 6, 7), (1, 3, 4, 6)
        ]
        rel material:binding:physics = </World/SoftMaterial>
    }
}
"""


def cube_height(physx):
    """Read the points of every volume soft body and return the cube's height."""
    with physx.read_sync(SimObjectType.DEFORMABLE_VOLUME, ["points"]) as result:
        points = result.groups[0].tensors[0].numpy()  # this body: one row per point
        return float(points[:, 2].max() - points[:, 2].min())


ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

stage = ovstage.Stage("soft_cube")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

print(f"t=0.00 s  height = {cube_height(physx):.3f} m")
for step in range(1, 91):
    physx.step_sync(1.0 / 60.0)
    if step % 15 == 0:
        print(f"t={step / 60:.2f} s  height = {cube_height(physx):.3f} m")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
