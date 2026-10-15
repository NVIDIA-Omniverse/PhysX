# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-INTERACTION-001
# @covers AC-2 AC-3 AC-4 AC-5 AC-6
# @implements REQ-NATIVE-INTERACTION-001
# @covers AC-2 AC-3 AC-4 AC-5 AC-6
import ctypes as C
from pathlib import Path
import pytest
from ovphysx_companion.interaction import Interaction,ForceMode,EndVelocity,GrabMode,_Api,_Options,_State,_PickingApi,_PickingOptions
from ovphysx_companion.inspection import _native_reference
from ovphysx_companion.native import ToolsSession,CompanionError,Status
from ovphysx_companion.preview import PreviewSession


@pytest.fixture
def stage(physics):
    import ovstage
    value=ovstage.Stage('interaction-test')
    ovstage.population.open_usd(value,str(Path(__file__).resolve().parents[1]/'samples/scenes/interaction.usda'),ordinal=1,
                                domains=ovstage.PopulationDomain.ALL)
    value.advance_write_floor(ordinal=1).wait()
    yield value
    value.destroy()


def test_impulse_at_point_and_exact_instance(physics,stage):
    with PreviewSession(physics,stage,read_ordinal=1) as owner,Interaction(owner) as tool:
        owner.step(.01)
        body=tool.resolve('/World/Body')
        tool.apply_force(body,(0,2,0),mode=ForceMode.IMPULSE,point=(1,0,2))
        tool.apply_force(body,(0,2,0),mode=ForceMode.IMPULSE,point=(1,0,0),frame='actor-local')
        a=tool.resolve('/World/Instances/Box',instance=0);b=tool.resolve('/World/Instances/Box',instance=1)
        tool.apply_force(a,(2,0,0),mode=ForceMode.IMPULSE)
        tool.apply_force(b,(2,0,0))
        owner.step(.01)
        assert tool.read(body).linear_velocity==pytest.approx((0,2,0))
        assert tool.read(body).angular_velocity==pytest.approx((0,0,4))
        assert tool.read(a).linear_velocity==pytest.approx((1,0,0))
        assert tool.read(b).linear_velocity[0]==pytest.approx(.01,abs=.0001)
        with pytest.raises(CompanionError) as error:tool.apply_force(tool.resolve('/World/Wall'),(1,0,0))
        assert error.value.status==Status.UNAVAILABLE


def test_drag_collision_end_velocity_and_abi(physics,stage):
    acquire=physics._lib.ovphysx_companion_interaction_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    api=_Api();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api)-1,C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    with PreviewSession(physics,stage,read_ordinal=1) as owner,Interaction(owner) as tool:
        owner.step(.01);ref=tool.resolve('/World/Body');native=_native_reference(ref)
        options=_Options();options.stiffness=-1;handle=C.c_uint64(987)
        assert tool._interaction_api.begin(C.byref(native),C.sizeof(native),C.byref(options),C.sizeof(options),C.byref(handle))==Status.INVALID_ARGUMENT
        assert handle.value==987
        with tool.joint_drag(ref,(0,0,2),end_velocity=EndVelocity.ZERO) as drag:
            assert drag.state.active
            with pytest.raises(CompanionError) as error:tool.drag(ref,(0,0,2))
            assert error.value.status==Status.BUSY
            drag.move((4,0,2))
            for _ in range(120):owner.step(1/120)
            # The body moves toward its target but remains on this side of the wall.
            assert .8<tool.read(ref).pose[0]<1.05
            assert drag.state.target==(4,0,2)
            output=_State();C.memset(C.byref(output),0xa5,C.sizeof(output));before=bytes(output)
            assert tool._interaction_api.read(drag._handle.value,C.byref(output),C.sizeof(output)-1)==Status.INVALID_ARGUMENT
            assert bytes(output)==before
        assert tool.read(ref).linear_velocity==pytest.approx((0,0,0))
        assert tool.read(ref).angular_velocity==pytest.approx((0,0,0))
        # A new drag proves that the old body's uniqueness entry was released.
        tool.apply_force(ref,(0,2,0),mode=ForceMode.IMPULSE)
        owner.step(.01)
        before=tool.read(ref).linear_velocity
        assert before[1]>.9
        with tool.joint_drag(ref,tool.read(ref).pose[:3],end_velocity=EndVelocity.RESTORE):
            tool.apply_force(ref,(0,2,0),mode=ForceMode.IMPULSE)
            owner.step(.01)
        assert tool.read(ref).linear_velocity==pytest.approx(before)


def test_drag_pending_peer_and_owner_retirement(attached,stage):
    from concurrent.futures import ThreadPoolExecutor
    with ToolsSession(attached) as playback,Interaction(playback) as live:
        with PreviewSession(attached,stage,read_ordinal=1) as preview,Interaction(preview) as tool:
            preview.step(.01);body=tool.resolve('/World/Body');drag=tool.drag(body,(0,0,2))
            preview.simulate(.01)
            for operation in (lambda:drag.move((1,0,2)),drag.close,lambda:tool.apply_force(body,(1,0,0))):
                with pytest.raises(CompanionError) as error:operation()
                assert error.value.status==Status.BUSY
            live.apply_force(live.resolve('/World/BoxA'),(1,0,0),mode=ForceMode.IMPULSE)
            preview.fetch();assert drag.state.active and drag.state.target==(0,0,2)
            with pytest.raises(CompanionError) as error:live.apply_force(body,(1,0,0))
            assert error.value.status==Status.STALE
            with ThreadPoolExecutor(1) as pool:
                with pytest.raises(RuntimeError,match='host thread'):pool.submit(drag.close).result()
        assert not drag.state.active
        with pytest.raises(CompanionError) as error:drag.move((1,0,2))
        assert error.value.status==Status.STALE
        drag.close();drag.close()


def test_interaction_extension_sample():
    import os,subprocess,sys,ovphysx_companion
    env=dict(os.environ);env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    result=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'samples/viewport_drag.py')],
                          env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr


def test_force_grab_default_pick_and_ray_updates(physics,stage):
    with PreviewSession(physics,stage,read_ordinal=1) as owner,Interaction(owner) as tool:
        owner.step(.01)
        body=tool.resolve('/World/Body')
        initial=tool.read(body)
        # Force is also the default for the existing drag entry point.
        with tool.drag(body,initial.pose[:3]) as default:
            assert default.mode==GrabMode.FORCE
        # Ray selection and move_ray retain the exact initial body and distance.
        with tool.pick((-5,0,2),(3,0,0),include=(body,)) as grab:
            assert grab.mode==GrabMode.FORCE and grab.state.body==body
            assert grab.pick_distance==pytest.approx(4.5)
            before=tool.read(body)
            assert grab.move_ray((-5,1,2),(7,0,0)).target==pytest.approx((-.5,1,2))
            assert tool.read(body).linear_velocity==before.linear_velocity
            with pytest.raises(CompanionError) as error:grab.move_ray((0,0,0),(0,0,0))
            assert error.value.status==Status.INVALID_ARGUMENT
            assert grab.state.target==pytest.approx((-.5,1,2))
            owner.step(.01)
            assert tool.read(body).linear_velocity[1]==pytest.approx(.5,rel=.01)
            # Force and joint modes share one exact-instance uniqueness rule.
            with pytest.raises(CompanionError) as error:tool.joint_drag(body,initial.pose[:3])
            assert error.value.status==Status.BUSY
        # Cancellation applies without another step and preserves final motion.
        released=tool.read(body).linear_velocity
        owner.step(.01)
        assert tool.read(body).linear_velocity==pytest.approx(released,abs=.001)
        assert tool.pick((-5,100,2),(1,0,0)) is None
        assert tool.pick((-5,0,2),(1,0,0),exclude=(body,)) is None
        with pytest.raises(ValueError,match='Joint options'):tool.drag(body,initial.pose[:3],stiffness=123)
        for bad in (-1,float('nan'),float('inf')):
            with pytest.raises(CompanionError) as error:tool.grab(body,initial.pose[:3],picking_force=bad)
            assert error.value.status==Status.INVALID_ARGUMENT


def test_force_picking_abi_preserves_rejected_outputs(physics,stage):
    acquire=physics._lib.ovphysx_companion_picking_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_PickingApi)];acquire.restype=C.c_uint32
    api=_PickingApi();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api)-1,C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    with PreviewSession(physics,stage,read_ordinal=1) as owner,Interaction(owner) as tool:
        owner.step(.01);body=tool.resolve('/World/Body');ref=_native_reference(body)
        options=_PickingOptions();options.pickingForce=-1;handle=C.c_uint64(987)
        assert api.begin(C.byref(ref),C.sizeof(ref),C.byref(options),C.sizeof(options),C.byref(handle))==Status.INVALID_ARGUMENT
        assert handle.value==987
        with tool.grab(body,(0,0,2)) as grab:
            output=_State();C.memset(C.byref(output),0xa5,C.sizeof(output));before=bytes(output)
            origin=(C.c_float*3)(0,0,0);zero=(C.c_float*3)(0,0,0)
            assert api.move_ray(grab._handle.value,origin,zero,C.byref(output),C.sizeof(output))==Status.INVALID_ARGUMENT
            assert bytes(output)==before


def test_force_grab_exact_point_instance_and_peer(attached,stage):
    with ToolsSession(attached) as live_owner,Interaction(live_owner) as live:
        with PreviewSession(attached,stage,read_ordinal=1) as preview,Interaction(preview) as tool:
            preview.step(.01)
            first=tool.resolve('/World/Instances/Box',instance=0);second=tool.resolve('/World/Instances/Box',instance=1)
            with tool.grab(first,tool.read(first).pose[:3]) as grab:
                start=grab.state.target;grab.move((start[0]+1,start[1],start[2]))
                attached.step_sync(.01)
                assert tool.read(first).linear_velocity[0]==pytest.approx(0)
                preview.step(.01)
                assert tool.read(first).linear_velocity[0]==pytest.approx(.5)
                assert tool.read(second).linear_velocity[0]==pytest.approx(0)
                with pytest.raises(CompanionError) as error:live.grab(first,start)
                assert error.value.status==Status.STALE


def test_force_picking_zero_and_live_tuning(physics,stage):
    with PreviewSession(physics,stage,read_ordinal=1) as owner,Interaction(owner) as tool:
        owner.step(.01);body=tool.resolve('/World/Body')
        with tool.grab(body,(0,0,2),picking_force=0) as grab:
            grab.move((1,0,2));owner.step(.01)
            assert tool.read(body).linear_velocity==pytest.approx((0,0,0))
            grab.set_picking_force(.5)
            with pytest.raises(CompanionError) as error:grab.set_picking_force(float('nan'))
            assert error.value.status==Status.INVALID_ARGUMENT
            assert tool.read(body).linear_velocity==pytest.approx((0,0,0))
            owner.step(.01)
            assert tool.read(body).linear_velocity[0]==pytest.approx(.25)
            grab.set_picking_force(0);owner.step(.01)
            assert tool.read(body).linear_velocity[0]==pytest.approx(.25)
            owner.simulate(.01)
            with pytest.raises(CompanionError) as error:grab.set_picking_force(1)
            assert error.value.status==Status.BUSY
            owner.fetch()


def test_joint_drag_wakes_sleeping_body_on_begin_and_move(physics,stage):
    with PreviewSession(physics,stage,read_ordinal=1) as owner,Interaction(owner) as tool:
        for _ in range(240):owner.step(1/120)
        body=tool.resolve('/World/Body')
        assert tool.read(body).sleeping
        with tool.joint_drag(body,(0,0,2)) as drag:
            assert not tool.read(body).sleeping
            for _ in range(240):owner.step(1/120)
            assert tool.read(body).sleeping
            drag.move((0,3,2))
            assert not tool.read(body).sleeping
            for _ in range(60):owner.step(1/120)
            assert tool.read(body).pose[1]>1


def test_force_ray_endpoint_rejects_joint_handle_without_mutation(physics,stage):
    from ovphysx_companion.interaction import _PickingApi,_State
    api=_PickingApi()
    acquire=physics._lib.ovphysx_companion_picking_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_PickingApi)];acquire.restype=C.c_uint32
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    with PreviewSession(physics,stage,read_ordinal=1) as owner,Interaction(owner) as tool:
        owner.step(.01);body=tool.resolve('/World/Body')
        with tool.joint_drag(body,(0,0,2)) as drag:
            target=drag.state.target
            output=_State();C.memset(C.byref(output),0xa5,C.sizeof(output));before=bytes(output)
            origin=(C.c_float*3)(10,0,2);direction=(C.c_float*3)(1,0,0)
            assert api.move_ray(drag._handle.value,origin,direction,C.byref(output),C.sizeof(output))==Status.INVALID_ARGUMENT
            assert bytes(output)==before and drag.state.target==target
