# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-INTERACTION-001
# @covers AC-2 AC-3 AC-4 AC-5 AC-6
"""Viewport gesture: ray pick -> native force grab -> host steps -> cancel cleanup.

An extension routes pointer/timeline events to these same calls on its approved
thread. The host owns stepping; Interaction never advances playback itself.
"""
from pathlib import Path


def main():
    import ovphysx,ovstage
    from ovphysx_companion.native import ToolsSession
    from ovphysx_companion.queries import SceneQueries
    from ovphysx_companion.interaction import Interaction,EndVelocity,GrabMode
    from ovphysx_companion.host import cpu_tool_availability
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX();stage=ovstage.Stage('viewport-drag')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).parent/'scenes/interaction.usda'),ordinal=1,
                                    domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        physics.wait_task(physics.attach_ovstage(stage,read_ordinal=1))
        with ToolsSession(physics) as owner,SceneQueries(owner) as queries,Interaction(owner) as tool:
            physics.step_sync(.01)
            assert cpu_tool_availability(owner,'interaction').available
            hit=queries.raycast((-5,0,2),(1,0,0),10).hits[0]
            with tool.pick((-5,0,2),(1,0,0),end_velocity=EndVelocity.ZERO) as gesture:
                assert gesture.mode==GrabMode.FORCE
                assert gesture.state.body==hit.body
                # Pointer motion retains the original hit distance and body.
                gesture.move_ray((-1,0,2),(1,0,0))
                # The host routes release, pause/stop, focus loss and input disable
                # to gesture.cancel(). Busy cancellation is retried after fetch.
                for _ in range(120):physics.step_sync(1/120)
                assert .8<tool.read(hit.body).pose[0]<1.05
            assert tool.read(hit.body).linear_velocity==(0,0,0)
            print('Default force grab collided with the wall; release restored damping and zeroed velocity.')
    finally:physics.destroy();stage.destroy()


if __name__=='__main__':main()
