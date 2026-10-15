# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEBUG-001
# @covers AC-1 AC-2 AC-4
"""Viewport adapter flow: configure, host step, copy a frame, render elsewhere."""
from pathlib import Path


def run():
    import ovphysx
    import ovstage
    from ovphysx_companion.native import ToolsSession
    from ovphysx_companion.debug import DebugDraw, Parameter
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX()
    stage=ovstage.Stage('debug-geometry-sample')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).with_name('scenes')/'two_boxes.usda'),
                                    ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        physics.wait_task(physics.attach_ovstage(stage,read_ordinal=1))
        with ToolsSession(physics) as owner, DebugDraw(owner) as debug:
            debug.configure(Parameter.COLLISION_SHAPES|Parameter.BODY_MASS_AXES)
            physics.step(1/60);physics.wait_all()
            frame=debug.capture()
            assert frame.lines
            # Queue this immutable object onto the renderer's approved thread.
    finally:
        physics.destroy();stage.destroy()
    return frame


if __name__=='__main__':
    frame=run()
    print(f'step={frame.step} lines={len(frame.lines)} units={frame.units} frame={frame.frame}')
