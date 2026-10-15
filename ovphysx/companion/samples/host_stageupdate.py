# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-HOST-001
# @covers AC-1 AC-2 AC-4
"""Two stageupdate entry points sharing one native host coordinator."""
from pathlib import Path


def run():
    import ovphysx
    import ovstage
    from ovphysx_companion.native import ToolsSession
    from ovphysx_companion.host import FrameCoordinator, cpu_tool_availability
    from ovphysx_companion.cct import CharacterController
    from ovphysx_companion.debug import DebugDraw
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX()
    stage=ovstage.Stage('host-stageupdate-sample')
    frames=[]
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).with_name('scenes')/'character.usda'),
                                    ordinal=1,domains=ovstage.PopulationDomain.PHYSICS|ovstage.PopulationDomain.RENDERING)
        stage.advance_write_floor(ordinal=1).wait()
        physics.wait_task(physics.attach_ovstage(stage,read_ordinal=1))
        with ToolsSession(physics) as owner, FrameCoordinator(owner) as host:
            cpu_tool_availability(owner,'character-controller').require()
            cpu_tool_availability(owner,'debug-geometry').require()
            with CharacterController(owner,'/World/Character') as character, DebugDraw(owner) as debug:
                character.set_gravity('scene')
                debug.configure()
                for tick in range(1,121):
                    def input():character.move(dt=1/60)
                    def output(frame):frames.append((frame,debug.capture()))
                    first=host.tick(tick,1/60,before_step=input,after_step=output)
                    second=host.tick(tick,1/60,before_step=input,after_step=output)
                    assert first is second # umbrella + PhysX adapters cannot double-step
                assert len(frames)==120
                # Coordinator-local and native step counters have distinct origins.
                assert all(host_frame.owner.attachment==draw_frame.attachment for host_frame,draw_frame in frames)
                assert character.state.position[2]<1.25
    finally:
        physics.destroy();stage.destroy()
    return len(frames)


if __name__=='__main__':print(f'{run()} frames, one native step per frame')
