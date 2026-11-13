# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEBUG-001
# @covers AC-1 AC-4
# @implements REQ-DEBUG-SNAPSHOT-001
# @covers AC-1 AC-2 AC-3
import ctypes as C
from dataclasses import FrozenInstanceError
from pathlib import Path
import pytest
from ovphysx_companion.debug import DebugDraw, Parameter, SnapshotLimitError, _Api, _Config, _Info, _Vertex
from ovphysx_companion.inspection import Inspector
from ovphysx_companion.native import ToolsSession, CompanionError, Status
from ovphysx_companion.preview import PreviewSession


def test_debug_completed_frame_units_color_identity_and_immutable_lifetime(attached):
    with ToolsSession(attached) as session:
        with DebugDraw(session) as debug:
            debug.configure(Parameter.COLLISION_SHAPES|Parameter.BODY_AXES)
            attached.step(1/60);attached.wait_all()
            frame=debug.capture()
            assert frame.attachment==session.identity.attachment and frame.step>0
            assert frame.lines and frame.frame=='world' and frame.units=='stage' and frame.color_format=='AARRGGBB'
            assert all(len(line)==2 for line in frame.lines)
            assert all(len(triangle)==3 for triangle in frame.triangles)
            assert all(0<=v.color<=0xffffffff and len(v.position)==3 for line in frame.lines for v in line)
            with pytest.raises(FrozenInstanceError):frame.lines[0][0].color=0
            with pytest.raises(SnapshotLimitError):debug.capture(max_vertices=1)
            retained=frame.lines
        assert frame.lines==retained
    attached.wait_task(attached.detach_ovstage())
    assert frame.lines==retained


def test_debug_frame_draws_moving_bodies_at_step_start_poses(attached):
    # Intended PhysX timing: shapes use pre-step poses so they line up with that step's contacts.
    with ToolsSession(attached) as session,Inspector(session) as inspector,DebugDraw(session) as debug:
        debug.configure(Parameter.COLLISION_SHAPES)
        boxes=[inspector.resolve(path) for path in ('/World/BoxA','/World/BoxB')]
        for _ in range(3):
            before=[inspector.read(box).pose[:3] for box in boxes]
            attached.step(1/60);attached.wait_all()
            frame=debug.capture()
            for box,start in zip(boxes,before):
                after=inspector.read(box)
                # Both boxes are still falling, 2 m apart in x; the floor's edges stay below z=0.
                points=[v.position for line in frame.lines for v in line if v.position[2]>1 and abs(v.position[0]-start[0])<1]
                assert len(points)==24 and frame.step==after.step
                assert tuple(sum(axis)/len(points) for axis in zip(*points))==pytest.approx(start,abs=1e-5)
                assert start[2]-after.pose[2]>1e-3


def test_debug_pending_rejection_and_disable(attached):
    with ToolsSession(attached) as session:
        with DebugDraw(session) as debug:
            debug.configure()
            attached.step(1/60) # core task scheduling can finish eagerly; use native preview for guaranteed pending
            attached.wait_all()
            assert debug.capture().lines
            debug.configure(enabled=False)
            assert not debug.capture().lines
            with pytest.raises(ValueError):debug.configure(scale=float('nan'))
            with pytest.raises(ValueError):debug.configure(culling=((1,0,0),(0,1,1)))
            with pytest.raises(ValueError):debug.configure(parameters=1<<63)
            debug.configure(culling=((-100,-100,-100),(100,100,100)))
            attached.step(1/60);attached.wait_all()
            assert debug.capture().lines


def test_debug_preview_pending_peer_isolation_and_retirement(physics,attached):
    import ovstage
    from ovphysx.population import physics_population_descs
    stage=ovstage.Stage('debug-preview')
    ovstage.population.open_usd_with_desc(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),
                                ordinal=1,time=ovstage.population.DEFAULT_TIME,descs=physics_population_descs(rendering=True))
    stage.advance_write_floor(ordinal=1).wait()
    try:
        with ToolsSession(attached) as playback, DebugDraw(playback) as a:
            a.configure()
            attached.step(1/60);attached.wait_all()
            before=a.capture()
            preview=PreviewSession(physics,stage,read_ordinal=1)
            b=DebugDraw(preview)
            b.configure(Parameter.BODY_AXES)
            preview.simulate(1/60)
            for call in (b.capture,b.configure,b.close):
                with pytest.raises(CompanionError) as error:call()
                assert error.value.status==Status.BUSY
            assert a.capture()==before
            preview.fetch()
            frame=b.capture()
            assert frame.lines and frame.attachment!=before.attachment
            preview.close();b.close()
            assert frame.lines
            assert a.capture()==before
    finally:stage.destroy()


def test_debug_native_copy_canaries_and_frame_after_owner_close(physics):
    import ovstage
    from ovphysx.population import physics_population_descs
    stage=ovstage.Stage('debug-native-copy')
    ovstage.population.open_usd_with_desc(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),
                                ordinal=1,time=ovstage.population.DEFAULT_TIME,descs=physics_population_descs(rendering=True))
    stage.advance_write_floor(ordinal=1).wait()
    api=_Api();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    acquire=physics._lib.ovphysx_companion_debug_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    handle=C.c_uint64()
    try:
        with PreviewSession(physics,stage,read_ordinal=1) as preview:
            config=_Config(C.sizeof(_Config),1,int(Parameter.COLLISION_SHAPES),1,1)
            assert api.configure(preview.identity.attachment,C.byref(config))==Status.OK
            preview.step(1/60)
            handle.value=0xa5a5
            assert api.capture(preview.identity.attachment,1,C.byref(handle))==8 and handle.value==0xa5a5
            handle.value=0
            assert api.capture(preview.identity.attachment,1000000,C.byref(handle))==Status.OK
        info=_Info()
        assert api.describe(handle.value,C.sizeof(info),C.byref(info))==Status.OK and info.lines>0
        data=(_Vertex*1)();C.memset(data,0xa5,C.sizeof(data));before=bytes(data)
        required=C.c_uint64()
        assert api.copy(handle.value,1,data,1,C.byref(required))==Status.INVALID_ARGUMENT
        assert required.value==2*info.lines and bytes(data)==before
        assert api.copy(handle.value,1,None,0,C.byref(required))==Status.OK
        all_vertices=(_Vertex*required.value)()
        assert api.copy(handle.value,1,all_vertices,required.value,C.byref(required))==Status.OK
    finally:
        if handle.value:assert api.release(handle.value)==Status.OK
        stage.destroy()


def test_debug_extension_sample_in_native_worker():
    import os
    import subprocess
    import sys
    import ovphysx_companion
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    sample=Path(__file__).resolve().parents[1]/'samples/debug_geometry.py'
    result=subprocess.run([sys.executable,str(sample)],env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr


def test_stopped_collision_overlay_normals_and_export_are_separate(physics):
    from ovphysx_companion.cooking import CookingService,Kind
    from ovphysx_companion.debug import collision_overlay,export_availability
    import numpy as np
    with CookingService(physics) as cooking:
        cooked=cooking.cook(Kind.TRIANGLE,[(0,0,0),(1,0,0),(0,1,0)],[(0,1,2)],
                            frame='shape-local',units='stage',meters_per_unit=.01).result()
    overlay=collision_overlay(cooked,path='/Stopped',color=0xff123456,normal_length=2,
                              local_to_output=((2,0,0,0),(0,3,0,0),(0,0,4,0),(1,2,3,1)),output_frame='world')
    assert overlay.frame=='world' and overlay.meters_per_unit==.01
    assert overlay.topology_version==cooked.cache_key
    assert overlay.triangles[0][0].position==pytest.approx((1,2,3))
    start,end=overlay.normals[0]
    assert np.subtract(end.position,start.position)==pytest.approx((0,0,2))
    assert all(v.color==0xff123456 for v in overlay.triangles[0])
    for operation in ('repx','legacy-pvd','unknown'):
        with pytest.raises(CompanionError) as error:export_availability(operation).require()
        assert error.value.status==Status.UNAVAILABLE
    assert not physics._attached_ovstage
    with pytest.raises(ValueError,match='requires an explicit transform'):
        collision_overlay(cooked,path='/Stopped',output_frame='world')
