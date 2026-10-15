# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VEHICLE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-VEHICLE-CONTROL-001
# @covers AC-1 AC-2 AC-3 AC-4
import ctypes as C
from pathlib import Path
import math
import pytest
from ovphysx_companion.vehicle import VehicleController,VehicleInput,VehicleMath,_Api,_State,_Command
from ovphysx_companion.preview import PreviewSession
from ovphysx_companion.native import CompanionError,Status


@pytest.fixture
def vehicle_stage(physics):
    import ovstage
    from ovphysx_companion.vehicle import population_desc
    stage=ovstage.Stage('native-vehicle')
    ovstage.population.open_usd_with_desc(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/vehicle.usda'),
        ordinal=1,time=float("nan"),descs=[population_desc()])
    stage.advance_write_floor(ordinal=1).wait()
    yield stage
    stage.destroy()


def test_vehicle_drive_real_wheels_and_clear(physics,vehicle_stage):
    with PreviewSession(physics,vehicle_stage,read_ordinal=1) as preview:
        with VehicleController(preview,'/World/Car') as vehicle:
            from ovphysx_companion.host import cpu_tool_availability
            assert cpu_tool_availability(preview,'vehicle-controller').available
            initial=vehicle.state
            assert initial.engine is None and len(initial.wheels)==4
            assert all(w.identity and w.identity.attachment==initial.identity.attachment for w in initial.wheels)
            for _ in range(180):
                vehicle.pre_step(VehicleInput(.8,steer=.1),dt=1/60)
                preview.step(1/60)
            state=vehicle.state
            assert state.inputs.throttle==pytest.approx(.8) and state.step>initial.step
            assert state.forward_velocity>1
            assert any(abs(w.rotation_speed)>1 and w.ground for w in state.wheels)
            assert all(w.frame=='actor-local' and w.ground.actor and w.ground.shape for w in state.wheels if w.ground), [(w.frame,w.ground) for w in state.wheels]
            assert any(w.suspension_jounce>0 and w.tire_friction>0 for w in state.wheels)
            vehicle.pause()
            assert vehicle.state.inputs==VehicleInput()
            assert vehicle.state.forward_velocity==pytest.approx(state.forward_velocity)
            with pytest.raises(RuntimeError,match='paused'):vehicle.pre_step(VehicleInput(1),dt=.1)
            vehicle.resume();vehicle.pre_step(VehicleInput(.5,.2,.1,-.3),dt=.1)
        with VehicleController(preview,'/World/Car') as replacement:
            assert replacement.state.inputs==VehicleInput()
            assert replacement.state.identity==state.identity


def test_vehicle_filter_pending_retry_and_abi_canaries(physics,vehicle_stage):
    api=_Api();acquire=physics._lib.ovphysx_companion_vehicle_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    with PreviewSession(physics,vehicle_stage,read_ordinal=1) as preview:
        with VehicleController(preview,'/World/Car',response_rate=2) as vehicle:
            with pytest.raises(RuntimeError,match='already'):VehicleController(preview,'/World/Car')
            assert vehicle.pre_step(VehicleInput(1),dt=.1).inputs.throttle==pytest.approx(.2)
            preview.simulate(.01)
            for operation in (lambda:vehicle.pre_step(VehicleInput(1),dt=.1),vehicle.close):
                with pytest.raises(CompanionError) as error:operation()
                assert error.value.status==Status.BUSY
            preview.fetch()
            assert vehicle.pre_step(VehicleInput(1),dt=.1).inputs.throttle==pytest.approx(.4)
            command=_Command(vehicle.state.identity.generation,1,-2,float('nan'),0,0,0)
            output=_State();C.memset(C.byref(output),0xa5,C.sizeof(output));before=bytes(output)
            assert api.execute(preview.identity.attachment,b'/World/Car',C.byref(command),C.sizeof(command),C.byref(output),C.sizeof(output))==Status.INVALID_ARGUMENT
            assert bytes(output)==before
            command.action=0;command.generation+=1
            assert api.execute(preview.identity.attachment,b'/World/Car',C.byref(command),C.sizeof(command),C.byref(output),C.sizeof(output))==Status.STALE
            assert bytes(output)==before
            command.generation=vehicle.state.identity.generation
            assert api.execute(preview.identity.attachment,b'/World/Car',C.byref(command),C.sizeof(command)-1,C.byref(output),C.sizeof(output))==Status.INVALID_ARGUMENT
            assert bytes(output)==before


def test_vehicle_auto_reverse_and_owner_replacement(physics,vehicle_stage):
    preview=PreviewSession(physics,vehicle_stage,read_ordinal=1)
    vehicle=VehicleController(preview,'/World/Car',auto_reverse=True)
    for _ in range(120):
        state=vehicle.pre_step(VehicleInput(brake=.8),dt=1/60);preview.step(1/60)
    assert vehicle.state.forward_velocity < -1
    assert state.inputs.throttle==pytest.approx(.8) and state.inputs.brake==0
    reverse=vehicle.pre_step(VehicleInput(throttle=.8),dt=1/60)
    assert reverse.inputs.throttle==0 and reverse.inputs.brake==pytest.approx(.8)
    old=vehicle.state.identity
    preview.close();vehicle.close()
    with PreviewSession(physics,vehicle_stage,read_ordinal=1) as newer:
        with VehicleController(newer,'/World/Car') as new:
            assert new.state.identity.attachment!=old.attachment
            assert new.state.inputs==VehicleInput()
            vehicle.close()
            assert new.state.inputs==VehicleInput()


def test_vehicle_math_bounds_frames_and_units(physics):
    math_service=VehicleMath(physics._lib)
    points=((-1,0,-2),(1,0,-2),(-1,0,2),(1,0,2))
    assert math_service.sprung_masses(points,1200)==pytest.approx((300,)*4)
    assert math_service.sprung_masses(tuple((x,y,z-.5) for x,y,z in points),1200)==pytest.approx((225,225,375,375))
    assert math_service.suspension_frame((1,2,3,0,0,0,1),(4,6,8,0,0,0,1))==pytest.approx((3,4,5,0,0,0,1))
    outer=math_service.ackermann(3,1.5,.4)
    assert 0<outer<.4 and math_service.ackermann(3,1.5,-.4)==pytest.approx(-outer)
    for op in (lambda:math_service.sprung_masses(points,-1),lambda:math_service.ackermann(0,1,.4),
               lambda:math_service.suspension_frame((0,0,0,0,0,0,0),(0,0,0,0,0,0,1))):
        with pytest.raises(CompanionError) as error:op()
        assert error.value.status==Status.INVALID_ARGUMENT
    api=math_service._api;out=(C.c_float*4)(9,9,9,9);xyz=(C.c_float*12)(*(v for p in points for v in p))
    assert api.sprung_masses(4,xyz,1200,1,out,3)==Status.INVALID_ARGUMENT and tuple(out)==(9,)*4
    assert api.sprung_masses(4,xyz,float('inf'),1,out,4)==Status.INVALID_ARGUMENT and tuple(out)==(9,)*4


def test_vehicle_peer_isolation(physics,vehicle_stage):
    import ovstage
    from ovphysx_companion.vehicle import population_desc
    peer=ovstage.Stage('vehicle-peer')
    ovstage.population.open_usd_with_desc(peer,str(Path(__file__).resolve().parents[1]/'samples/scenes/vehicle.usda'),ordinal=1,time=float("nan"),descs=[population_desc()])
    peer.advance_write_floor(ordinal=1).wait()
    try:
        with PreviewSession(physics,vehicle_stage,read_ordinal=1) as first,PreviewSession(physics,peer,read_ordinal=1) as second:
            with VehicleController(first,'/World/Car') as a,VehicleController(second,'/World/Car') as b:
                first.simulate(.01)
                assert b.pre_step(VehicleInput(.7),dt=.1).inputs.throttle==pytest.approx(.7)
                second.step(.01);first.fetch()
                assert a.state.inputs==VehicleInput()
                b.close();assert a.state.inputs==VehicleInput()
    finally:peer.destroy()


def test_vehicle_sample_and_stopped_math_in_workers():
    import os,subprocess,sys,ovphysx_companion
    env=dict(os.environ);env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    sample=Path(__file__).resolve().parents[1]/'samples/vehicle_control.py'
    for args in ([],['--math-only']):
        result=subprocess.run([sys.executable,str(sample),*args],env=env,capture_output=True,text=True,timeout=60)
        assert result.returncode==0,result.stdout+result.stderr


def test_vehicle_forward_speed_uses_world_center_of_mass_frame(physics,tmp_path):
    import ovstage
    from ovphysx_companion.vehicle import population_desc
    from ovphysx_companion.inspection import Inspector
    source=Path(__file__).resolve().parents[1]/'samples/scenes/vehicle.usda'
    text=source.read_text().replace('quatf physics:principalAxes = (1, 0, 0, 0)',
        'quatf physics:principalAxes = (0, 0, 1, 0)\n        vector3f physics:velocity = (0, 0, 10)')
    scene=tmp_path/'vehicle-com.usda';scene.write_text(text)
    stage=ovstage.Stage('vehicle-com-frame')
    try:
        ovstage.population.open_usd_with_desc(stage,str(scene),ordinal=1,time=float('nan'),descs=[population_desc()])
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as preview:
            preview.step(.001)
            inspector=Inspector(preview);body=inspector.read(inspector.resolve('/World/Car'))
            def rotate(q,v):
                x,y,z,w=q;vx,vy,vz=v
                cross=(y*vz-z*vy,z*vx-x*vz,x*vy-y*vx)
                cross2=(y*cross[2]-z*cross[1],z*cross[0]-x*cross[2],x*cross[1]-y*cross[0])
                return tuple(v[i]+2*w*cross[i]+2*cross2[i] for i in range(3))
            forward=rotate(body.pose[3:],rotate(body.center_of_mass[3:],(0,0,1)))
            expected=sum(v*f for v,f in zip(body.linear_velocity,forward))
            assert expected < -1
            with VehicleController(preview,'/World/Car',auto_reverse=True) as controller:
                assert controller.state.forward_velocity==pytest.approx(expected)
                inputs=controller.pre_step(VehicleInput(throttle=.8),dt=.01).inputs
                assert inputs.throttle==0 and inputs.brake==pytest.approx(.8)
    finally:stage.destroy()
