# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OWNER-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-DEBUG-001
# @covers AC-2

"""Statistics-extension flow backed by a real CPU simulation and native bridge.

Run in a simulation process without customer pxr. A host with an existing PhysX
owner starts at ToolsSession(physics) and retains its existing timeline policy.
"""

from pathlib import Path

from ovphysx_companion.native import ToolsSession


def run():
    import ovphysx
    import ovstage

    from ovphysx_companion.schemas import physx_schema_paths
    schemas = [str(p) for p in physx_schema_paths()]
    newton = ovphysx.newton_schema_root()
    if newton:
        schemas.append(str(newton))
    ovstage.population.register_usd_schemas(schemas)
    ovphysx.PhysX.set_cpu_mode(True)
    physics = ovphysx.PhysX()
    stage = ovstage.Stage("companion-statistics-sample")
    try:
        ovstage.population.open_usd(stage, str(Path(__file__).parent / "scenes/two_boxes.usda"),
                                    ordinal=1, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        physics.wait_task(physics.attach_ovstage(stage, read_ordinal=1))
        with ToolsSession(physics) as tools:
            identity = tools.identity
            for _ in range(3):
                physics.step(1 / 60)
                physics.wait_all()
                snapshot = tools.scene_statistics("/World/PhysicsScene")
                assert snapshot.owner == identity
                assert snapshot.dynamic_bodies == 2
                assert snapshot.static_bodies == 1
                print(snapshot)
        # Session close precedes owner destruction. No companion attach or step
        # route is involved; the existing host remains the simulation scheduler.
    finally:
        physics.destroy()
        stage.destroy()


if __name__ == "__main__":
    run()
