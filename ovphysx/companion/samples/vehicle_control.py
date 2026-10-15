# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VEHICLE-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Vehicle extension flow: stopped computation, pre-step input, copied telemetry.

Host input/UI stays outside this module. --math-only loads the shared library but
never initializes a physics instance, attaches a stage or imports customer USD.
"""
from pathlib import Path


def stopped():
    import ctypes,os
    from ovphysx_companion.vehicle import VehicleMath
    service=VehicleMath(ctypes.CDLL(os.environ['OVPHYSX_LIB']))
    masses=service.sprung_masses(((-1,0,-2),(1,0,-2),(-1,0,2),(1,0,2)),1200)
    assert masses==(300,)*4
    frame=service.suspension_frame((1,0,0,0,0,0,1),(2,0,3,0,0,0,1))
    assert frame==(1,0,3,0,0,0,1)
    return {'sprung_masses':masses,'suspension_frame':frame,'outer_steer_angle':service.ackermann(3,1.5,.4)}


def run():
    import ovphysx,ovstage
    from ovphysx_companion.vehicle import population_desc
    from ovphysx_companion.preview import PreviewSession
    from ovphysx_companion.vehicle import VehicleController,VehicleInput
    from ovphysx_companion.host import FrameCoordinator
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    ovphysx.PhysX.set_cpu_mode(True)
    physics=ovphysx.PhysX();stage=ovstage.Stage('vehicle-extension-sample')
    try:
        ovstage.population.open_usd_with_desc(stage,str(Path(__file__).with_name('scenes')/'vehicle.usda'),ordinal=1,time=float("nan"),descs=[population_desc()])
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as preview:
            with VehicleController(preview,'/World/Car',response_rate=3) as vehicle,FrameCoordinator(preview) as loop:
                frames=[]
                for frame in range(180):
                    controls=VehicleInput(throttle=.8,steer=.1 if frame>60 else 0)
                    loop.tick(frame,1/60,before_step=lambda:vehicle.pre_step(controls,dt=1/60),
                              after_step=lambda completed:frames.append(vehicle.state))
                assert frames[-1].forward_velocity>1 and any(w.ground for w in frames[-1].wheels)
                assert frames[-1].engine is None # Direct drive has no invented engine metrics.
                vehicle.pause();loop.pause() # Host routes its pause action to both adapters.
                assert vehicle.state.inputs==VehicleInput()
                return {'steps':len(frames),'forward_velocity':frames[-1].forward_velocity,'wheels':len(frames[-1].wheels)}
    finally:
        physics.destroy();stage.destroy()


if __name__=='__main__':
    import sys
    print(stopped() if '--math-only' in sys.argv else run())
