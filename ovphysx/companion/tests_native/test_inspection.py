# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-QUERY-001
# @covers AC-2 AC-3 AC-4
# @implements REQ-NATIVE-INSPECTION-001
# @covers AC-1 AC-2 AC-3 AC-4
import ctypes as C
from dataclasses import replace
from pathlib import Path
import pytest
from ovphysx_companion.inspection import Inspector,ObjectKind,_Api,_Body,_Object,_Articulation,_native_reference
from ovphysx_companion.native import ToolsSession,CompanionError,Status
from ovphysx_companion.preview import PreviewSession


@pytest.fixture
def inspection_stage(physics):
    import ovstage
    stage=ovstage.Stage('typed-inspection')
    ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/inspection.usda'),
                                ordinal=1,domains=ovstage.PopulationDomain.ALL)
    stage.advance_write_floor(ordinal=1).wait()
    yield stage
    stage.destroy()


def test_typed_body_joint_material_and_moving_articulation(physics,inspection_stage):
    with PreviewSession(physics,inspection_stage,read_ordinal=1) as preview,Inspector(preview) as inspector:
        preview.step(.01)
        ref=inspector.resolve('/World/Body');body=inspector.read(ref)
        assert inspector.path(ref)=='/World/Body'
        assert body.type=='dynamic' and body.mass==6
        assert body.inertia==pytest.approx((1,2,3)) and body.center_of_mass[:3]==pytest.approx((.1,0,0))
        assert body.pose[:3]==pytest.approx(preview.rigid_pose('/World/Body').position)
        hinge=inspector.read(inspector.resolve('/World/Hinge',ObjectKind.JOINT))
        # Runtime realizes this authored revolute joint as a native D6 constraint.
        assert hinge.type=='D6' and hinge.body0 is None and hinge.body1==ref
        assert hinge.local0[:3]==(4,0,5) and hinge.local1[:3]==(0,0,1)
        assert hinge.world0[:3]==pytest.approx(hinge.world1[:3],abs=.003)
        assert hinge.break_force==100000 and hinge.break_torque==200000
        material=inspector.read(inspector.resolve('/World/Material',ObjectKind.MATERIAL))
        assert (material.static_friction,material.dynamic_friction,material.restitution)==pytest.approx((.7,.4,.2))
        static=inspector.read(inspector.resolve('/World/Floor'))
        assert static.type=='static' and static.mass is None and static.linear_velocity is None
        robot=inspector.resolve('/World/Robot',ObjectKind.ARTICULATION)
        before=inspector.read(robot)
        assert len(before.links)==2 and len(before.dofs)==1
        assert before.dofs[0].axis=='twist' and before.dofs[0].index==0
        elbow=inspector.read(inspector.resolve('/World/Robot/Elbow',ObjectKind.JOINT))
        assert elbow.type=='articulation' and elbow.flags is None and elbow.break_force is None
        assert elbow.world0[:3]==pytest.approx(elbow.world1[:3],abs=.003)
        assert {v.identity for v in before.links}=={elbow.body0,elbow.body1}
        for _ in range(30):preview.step(.01)
        after=inspector.read(robot)
        assert after.step>before.step and abs(after.dofs[0].position-before.dofs[0].position)>.01
        assert before.links[1].pose!=after.links[1].pose and abs(after.dofs[0].velocity)>.01
        assert before.identity==after.identity and body.identity==inspector.resolve('/World/Body')
    assert before.dofs[0].link.kind==ObjectKind.BODY


def test_inspection_busy_peer_foreign_and_retired_references(attached,inspection_stage):
    with ToolsSession(attached) as tools,Inspector(tools) as playback:
        with PreviewSession(attached,inspection_stage,read_ordinal=1) as preview,Inspector(preview) as inspector:
            reference=inspector.resolve('/World/Body');copied=inspector.read(reference)
            preview.simulate(.01)
            with pytest.raises(CompanionError) as error:inspector.read(reference)
            assert error.value.status==Status.BUSY
            assert playback.read(playback.resolve('/World/BoxA')).type=='dynamic'
            preview.fetch()
            with pytest.raises(CompanionError) as error:playback.read(reference)
            assert error.value.status==Status.STALE
            with pytest.raises(CompanionError) as error:inspector.read(replace(reference,generation=reference.generation+1))
            assert error.value.status==Status.STALE
        with PreviewSession(attached,inspection_stage,read_ordinal=1) as newer,Inspector(newer) as inspector:
            with pytest.raises(CompanionError) as error:inspector.read(reference)
            assert error.value.status==Status.STALE
            assert inspector.resolve('/World/Body').attachment!=reference.attachment
        assert copied.mass==6


def test_inspection_abi_canaries_and_capacity(physics,inspection_stage):
    acquire=physics._lib.ovphysx_companion_inspection_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    api=_Api();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    with PreviewSession(physics,inspection_stage,read_ordinal=1) as preview,Inspector(preview) as inspector:
        preview.step(.01);api=inspector._api
        reference=_native_reference(inspector.resolve('/World/Body'))
        output=_Body();C.memset(C.byref(output),0xa5,C.sizeof(output));before=bytes(output)
        assert api.read(C.byref(reference),C.sizeof(reference)-1,C.byref(output),C.sizeof(output))==Status.INVALID_ARGUMENT
        assert bytes(output)==before
        reference.generation+=1
        assert api.read(C.byref(reference),C.sizeof(reference),C.byref(output),C.sizeof(output))==Status.STALE
        assert bytes(output)==before
        robot=_native_reference(inspector.resolve('/World/Robot',ObjectKind.ARTICULATION))
        value=_Articulation();C.memset(C.byref(value),0xa5,C.sizeof(value));before=bytes(value)
        nl=C.c_uint32();nd=C.c_uint32()
        assert api.articulation(C.byref(robot),C.sizeof(robot),C.byref(value),C.sizeof(value),None,0,C.byref(nl),None,0,C.byref(nd))==8
        assert (nl.value,nd.value)==(2,1) and bytes(value)==before
        for path,kind in [('/Absent',ObjectKind.BODY),('/World/Body',ObjectKind.MATERIAL)]:
            with pytest.raises(CompanionError) as error:inspector.resolve(path,kind)
            assert error.value.status==Status.NOT_FOUND


def test_inspector_extension_sample():
    import os,subprocess,sys,ovphysx_companion
    env=dict(os.environ);env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    result=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'samples/inspector_tools.py')],
                          env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr
