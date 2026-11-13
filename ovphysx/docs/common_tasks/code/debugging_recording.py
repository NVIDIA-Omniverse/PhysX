# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
import os

import ovstage
from ovphysx.population import physics_population_descs
from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, set_global_config
from ovphysx.samples.python_samples.schema_support import physx_schema_paths

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

ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
stage = ovstage.Stage("recorded_box")
ovstage.population.open_usd_from_string_with_desc(
    stage, SCENE, ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

# Allow recording. This must happen before the first PhysX object exists.
set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

# Record one second of simulation, starting before the first step.
physx.start_recording(OmniPvdDestination.file("falling_box.ovd"))
for step in range(60):
    physx.step_sync(1.0 / 60.0)
physx.stop_recording()  # finishes the file

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
size_kb = os.path.getsize("falling_box.ovd") / 1024
print(f"Recorded 60 steps to falling_box.ovd ({size_kb:.0f} KB)")
# [example-end]
