# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
from pathlib import Path

import ovstage
from ovphysx import PhysX
from ovphysx.population import physics_population_descs
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx.types import SimObjectType

# The scene file is in the same folder as this program.
SCENE_FILE = Path(__file__).with_name("loading_scenes_crates.usda")


def rigid_bodies(physx):
    """Return the path and position of every rigid body in the simulation."""
    bodies = []
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        for group in result.groups:
            names = result.dictionary.get_path_strings(group.prim_list)
            for name, position in zip(names, group.tensors[0].numpy()):
                bodies.append((name, position.tolist()))
    return bodies


# Register the PhysX schemas once per process, before the first load.
ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])

# Load the file into a stage at ordinal 1, then seal ordinal 1.
stage = ovstage.Stage("crates")
ovstage.population.open_usd_with_desc(
    stage, str(SCENE_FILE), ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True)
)
stage.advance_write_floor(ordinal=1).wait()

# Attach the sealed stage. ovphysx builds the simulation from it.
physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))

bodies = rigid_bodies(physx)
print(f"ovphysx found {len(bodies)} rigid bodies:")
for path, (x, y, z) in sorted(bodies):
    print(f"  {path}  at ({x:.2f}, {y:.2f}, {z:.2f}) m")

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
# [example-end]
