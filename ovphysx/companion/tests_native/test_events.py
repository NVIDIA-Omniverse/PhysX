# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-EVENTS-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-EVENT-QUEUE-001
# @covers AC-1 AC-2 AC-3 AC-4
import ctypes as C
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import pytest
from ovphysx_companion.events import EventQueue,Mask,Kind,Phase,ContactType,TriggerType,EventOverflow,_Api,_Event,_Batch
from ovphysx_companion.preview import PreviewSession
from ovphysx_companion.native import CompanionError,Status


@pytest.fixture
def event_stages(physics):
    import ovstage
    stages=[]
    try:
        for name in ('event-A','event-B'):
            stage=ovstage.Stage(name);stages.append(stage)
            ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/events.usda'),
                ordinal=1,domains=ovstage.PopulationDomain.ALL)
            stage.advance_write_floor(ordinal=1).wait()
        yield stages
    finally:
        for stage in reversed(stages):stage.destroy()


def test_native_contacts_triggers_stay_and_phase_order(physics,event_stages):
    with PreviewSession(physics,event_stages[0],read_ordinal=1) as preview:
        with EventQueue(preview,filter_path='/World/BoxA') as queue,EventQueue(preview,mask=Mask.CONTACT,filter_path='/World/BoxB') as other:
            events=[];other_events=[]
            for _ in range(120):
                preview.step(1/120);preview.step(1/120) # two native substeps per host delivery
                batch=queue.poll();events.extend(batch.events);other_events.extend(other.poll().events)
                assert batch.owner_live and not batch.overflow
                assert len([e for e in batch.events if e.kind==Kind.SUBSTEP_BEGIN])==2
                assert all(e.step<=batch.completed_step for e in batch.events)
            assert [e.sequence for e in events]==sorted(e.sequence for e in events)
            assert all(e.attachment==preview.identity.attachment and e.generation for e in events)
            for kind in (Kind.PRE_STEP,Kind.SUBSTEP_BEGIN,Kind.SUBSTEP_END,Kind.POST_STEP,Kind.SIMULATION_COMPLETE):
                assert any(e.kind==kind for e in events),kind
            begin=next(e for e in events if e.kind==Kind.SUBSTEP_BEGIN)
            end=next(e for e in events if e.kind==Kind.SUBSTEP_END)
            assert begin.sequence<end.sequence and begin.substep==end.substep
            assert begin.phase==Phase.SUBSTEP and end.phase==Phase.SUBSTEP
            triggers=[e for e in events if e.kind in (Kind.TRIGGER,Kind.TRIGGER_STAY)]
            enter=next(e for e in triggers if e.trigger_type==TriggerType.ENTER)
            leave=next(e for e in triggers if e.trigger_type==TriggerType.LEAVE)
            stays=[e for e in triggers if e.kind==Kind.TRIGGER_STAY]
            assert stays and enter.sequence<stays[0].sequence<leave.sequence
            assert enter.pair==leave.pair and enter.pair.instance0 is None
            assert all(e.sequence<leave.sequence for e in stays)
            contacts=[e for e in events if e.kind==Kind.CONTACT]
            assert contacts and other_events
            assert any(e.contact_type==ContactType.FOUND for e in contacts)
            assert any(e.point and e.point.position and abs(e.point.impulse[2])>0 for e in contacts)
            assert all(e.pair.actor0==enter.pair.actor1 or e.pair.actor1==enter.pair.actor1 for e in contacts)
            assert all(e.pair.actor0!=enter.pair.actor1 and e.pair.actor1!=enter.pair.actor1 for e in other_events)
            assert all(e.pair.collider0 and e.pair.collider1 for e in contacts)
            saved=contacts[0]
        assert saved.point is not None
        with pytest.raises(Exception):saved.sequence=0


def test_event_busy_peer_overflow_and_thread_delivery(physics,event_stages):
    with PreviewSession(physics,event_stages[0],read_ordinal=1) as a,PreviewSession(physics,event_stages[1],read_ordinal=1) as b:
        with EventQueue(a,capacity=1,mask=Mask.PRE_STEP|Mask.POST_STEP|Mask.SUBSTEPS) as small,EventQueue(b) as peer:
            a.simulate(.01)
            with pytest.raises(CompanionError) as error:small.poll()
            assert error.value.status==Status.BUSY
            b.step(.01);assert peer.poll().events
            a.fetch()
            with pytest.raises(EventOverflow) as error:small.poll()
            assert error.value.batch.overflow and error.value.batch.dropped>0
            assert small.poll(allow_incomplete=True).overflow
            with ThreadPoolExecutor(1) as pool:
                with pytest.raises(RuntimeError,match='host thread'):pool.submit(peer.poll).result()
            a.simulate(.01);small.close();a.fetch() # cancellation is metadata, not physics reentry
            with pytest.raises(RuntimeError,match='closed'):small.poll()


def test_event_disposal_stops_dispatch_and_retirement_keeps_owned_history(physics,event_stages):
    preview=PreviewSession(physics,event_stages[0],read_ordinal=1)
    queue=EventQueue(preview)
    preview.step(.01)
    called=[]
    def handler(event):
        called.append(event);queue.close()
    batch=queue.dispatch(handler)
    assert len(batch.events)>1 and len(called)==1
    queue2=EventQueue(preview)
    preview.step(.01);preview.close()
    history=queue2.poll()
    assert not history.owner_live and history.events
    assert all(e.attachment==history.attachment for e in history.events)
    queue2.close()
    assert batch.events[0]==called[0]


def test_event_abi_invalid_poll_does_not_consume(physics,event_stages):
    acquire=physics._lib.ovphysx_companion_events_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    api=_Api();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    with PreviewSession(physics,event_stages[0],read_ordinal=1) as preview,EventQueue(preview) as queue:
        preview.step(.01)
        output=_Event();info=_Batch();C.memset(C.byref(output),0xa5,C.sizeof(output));C.memset(C.byref(info),0xa5,C.sizeof(info))
        before=(bytes(output),bytes(info))
        assert queue._api.poll(queue._handle.value,C.sizeof(output)-1,C.byref(output),1,C.sizeof(info),C.byref(info))==Status.INVALID_ARGUMENT
        assert (bytes(output),bytes(info))==before
        assert queue.poll().events
        with pytest.raises(CompanionError):EventQueue(preview,filter_path='/Absent')



def test_event_monitor_sample():
    import os,subprocess,sys,ovphysx_companion
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    result=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'samples/event_monitor.py')],
                          env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr


def test_contact_point_instance_identity(physics,tmp_path):
    import ovstage
    source=tmp_path/'instances.usda'
    source.write_text('''#usda 1.0
(metersPerUnit = 1
 upAxis = "Z")
def Xform "World" {
    def PhysicsScene "Scene" (prepend apiSchemas = ["PhysxSceneAPI"]) {
        bool physxScene:enableGPUDynamics = false
        token physxScene:broadphaseType = "MBP"
    }
    def PointInstancer "Instances" {
        rel prototypes = </World/Instances/Box>
        int[] protoIndices = [0, 0]
        point3f[] positions = [(0, 0, 3), (2, 0, 3)]
        quath[] orientations = [(1, 0, 0, 0), (1, 0, 0, 0)]
        def Cube "Box" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysxContactReportAPI"]) {
            double size = 1
            float physxContactReport:threshold = 0
        }
    }
    def Cube "Trigger" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxTriggerAPI"]) {
        double size = 1
        double3 xformOp:translate = (1, 0, 2)
        double3 xformOp:scale = (5, 2, 0.2)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
    def Cube "Floor" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
        double size = 1
        double3 xformOp:translate = (0, 0, -0.5)
        double3 xformOp:scale = (10, 10, 1)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
}
''')
    stage=ovstage.Stage('contact-instances')
    try:
        ovstage.population.open_usd(stage,str(source),ordinal=1,
            domains=ovstage.PopulationDomain.PHYSICS|ovstage.PopulationDomain.RENDERING)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as preview,EventQueue(preview,mask=Mask.CONTACT|Mask.TRIGGER) as queue:
            indices=set();enters=set();leaves=set();stays=set()
            for _ in range(90):
                preview.step(1/60)
                for event in queue.poll().events:
                    if event.kind not in (Kind.CONTACT,Kind.TRIGGER,Kind.TRIGGER_STAY):continue
                    pair=event.pair
                    assert (pair.instance0 is None)!=(pair.instance1 is None)
                    index=pair.instance0 if pair.instance0 is not None else pair.instance1
                    if event.kind==Kind.CONTACT:indices.add(index)
                    elif event.kind==Kind.TRIGGER_STAY:stays.add(index)
                    elif event.trigger_type==TriggerType.ENTER:enters.add(index)
                    elif event.trigger_type==TriggerType.LEAVE:leaves.add(index)
            assert indices==enters==leaves==stays=={0,1}, (indices,enters,leaves,stays)
    finally:stage.destroy()


def test_event_queue_retains_runtime_after_last_owner_in_worker():
    import os,subprocess,sys,ovphysx_companion
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    code='''
import ovphysx,ovstage
from ovphysx_companion.preview import PreviewSession
from ovphysx_companion.events import EventQueue
from ovphysx_companion.schemas import physx_schema_paths
ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
ovphysx.PhysX.set_cpu_mode(True)
p=ovphysx.PhysX();s=ovstage.Stage('event-retention')
import sys
ovstage.population.open_usd(s,sys.argv[1],ordinal=1,domains=ovstage.PopulationDomain.ALL)
s.advance_write_floor(ordinal=1).wait()
preview=PreviewSession(p,s,read_ordinal=1);q=EventQueue(preview)
preview.step(.01);preview.close();p.destroy();s.destroy()
batch=q.poll();assert batch.events and not batch.owner_live
q.close();assert batch.events
'''
    result=subprocess.run([sys.executable,'-c',code,str(Path(__file__).resolve().parents[1]/'samples/scenes/events.usda')],
                          env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr


def test_trigger_leave_for_articulation_link(physics,tmp_path):
    import ovstage
    source=Path(__file__).resolve().parents[1]/'samples/scenes/events.usda'
    text=source.read_text().replace('"PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysxContactReportAPI"',
        '"PhysicsRigidBodyAPI", "PhysicsArticulationRootAPI", "PhysicsCollisionAPI", "PhysxContactReportAPI"',1)
    path=tmp_path/'articulation-trigger.usda';path.write_text(text)
    stage=ovstage.Stage('articulation-trigger')
    try:
        ovstage.population.open_usd(stage,str(path),ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as preview,EventQueue(preview,mask=Mask.TRIGGER) as queue:
            events=[]
            for _ in range(120):
                preview.step(1/60);events.extend(queue.poll().events)
            enter=next(e for e in events if e.trigger_type==TriggerType.ENTER)
            leave=next((e for e in events if e.trigger_type==TriggerType.LEAVE),None)
            assert leave is not None and leave.pair==enter.pair
            assert all(e.sequence<leave.sequence for e in events if e.kind==Kind.TRIGGER_STAY)
    finally:stage.destroy()


@pytest.mark.parametrize('approximation', ['convexDecomposition', 'sphereFill'])
def test_compound_collider_contact_filter(physics, tmp_path, approximation):
    import ovstage
    source = tmp_path / 'compound.usda'
    source.write_text('''#usda 1.0
(metersPerUnit = 1
 upAxis = "Z")
def Xform "World" {
    def PhysicsScene "Scene" (prepend apiSchemas = ["PhysxSceneAPI"]) {
        bool physxScene:enableGPUDynamics = false
    }
    def Xform "Body" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxContactReportAPI"]) {
        float physxContactReport:threshold = 0
        double3 xformOp:translate = (0, 0, 2)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Mesh "Collider" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]) {
            token physics:approximation = "APPROXIMATION"
            point3f[] points = [(-.5,-.5,-.5), (.5,-.5,-.5), (.5,.5,-.5), (-.5,.5,-.5),
                               (-.5,-.5,.5), (.5,-.5,.5), (.5,.5,.5), (-.5,.5,.5)]
            int[] faceVertexCounts = [4,4,4,4,4,4]
            int[] faceVertexIndices = [0,3,2,1, 4,5,6,7, 0,1,5,4, 1,2,6,5, 2,3,7,6, 3,0,4,7]
        }
    }
    def Cube "Floor" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {
        double size = 1
        double3 xformOp:translate = (0, 0, -.5)
        double3 xformOp:scale = (10, 10, 1)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
}
'''.replace('APPROXIMATION', approximation))
    with ovstage.Stage('compound-contact-filter') as stage:
        ovstage.population.open_usd(stage, str(source), ordinal=1, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics, stage, read_ordinal=1) as preview:
            with EventQueue(preview, mask=Mask.CONTACT, filter_path='/World/Body/Collider') as filtered, \
                    EventQueue(preview, mask=Mask.CONTACT) as all_contacts:
                selected, complete = [], []
                for _ in range(120):
                    preview.step(1/60)
                    selected.extend(filtered.poll().events)
                    complete.extend(all_contacts.poll().events)
                assert selected and complete
                assert [(event.pair, event.contact_type) for event in selected] == [
                    (event.pair, event.contact_type) for event in complete]


def test_raw_event_poll_marks_lost_history_with_diagnostic_batch(physics,event_stages):
    with PreviewSession(physics,event_stages[0],read_ordinal=1) as preview:
        with EventQueue(preview,capacity=1,mask=Mask.PRE_STEP|Mask.POST_STEP) as queue:
            preview.step(.01)
            records=(_Event*1)();batch=_Batch()
            status=queue._api.poll(queue._handle.value,C.sizeof(_Event),records,1,C.sizeof(batch),C.byref(batch))
            assert status==8 and batch.overflow and batch.dropped==1 and batch.count==1
            with pytest.raises(EventOverflow):queue.poll()
            assert queue.poll(allow_incomplete=True).overflow
