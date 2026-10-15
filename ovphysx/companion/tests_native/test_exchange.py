# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVSTAGE-001
# @covers AC-1 AC-2 AC-3
import numpy as np
import pytest
from ovphysx_companion.exchange import StageFlow,OutputColumn
from ovphysx_companion.inspection import Inspector
from ovphysx_companion.native import ToolsSession,MassShape,PrimitiveKind
from ovphysx_companion.jobs import AuthoringService
from ovphysx_companion.preview import PreviewSession


@pytest.fixture
def exchange_attached(physics):
    import ovstage
    from pathlib import Path
    stage=ovstage.Stage('exchange-core')
    ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/exchange.usda'),ordinal=1,
                                domains=ovstage.PopulationDomain.ALL)
    stage.advance_write_floor(ordinal=1).wait();physics.wait_task(physics.attach_ovstage(stage,read_ordinal=1))
    physics.wait_task(physics.step(.01))
    try:yield physics
    finally:physics.wait_task(physics.detach_ovstage());stage.destroy()


def write(stage,path,attribute,values,ordinal):
    import ovstage
    with ovstage.PathDictionary(stage) as dictionary:
        with dictionary.create_path_list_from_strings([path]) as selected:
            with stage.query_from_path_list(selected) as query:
                stage.write_attribute(query,attribute,ordinal,values,is_array=False).wait()


def read(stage,path,attribute,ordinal):
    import ovstage
    values=[]
    with ovstage.PathDictionary(stage) as dictionary:
        token=dictionary.intern_token(attribute)
        with dictionary.create_path_list_from_strings([path]) as selected:
            with stage.query_from_path_list(selected) as query:
                with stage.read_attributes(query,[token],ovstage.OrdinalRange.latest(ordinal)) as batch:
                    batch.wait()
                    for group in batch.groups():
                        with group:
                            for i in range(group.tensor_count):values.append(np.array(group.array(i),copy=True))
    return values


def test_core_authored_input_explicit_output_and_snapshot(exchange_attached):
    attached=exchange_attached
    stage=attached._attached_ovstage
    with ToolsSession(attached) as owner,Inspector(owner) as inspector,StageFlow(owner,stage,consumed_ordinal=1) as flow:
        original=read(stage,'/World/BoxA','omni:fabric:worldMatrix',1)
        write(stage,'/World/BoxA','physics:mass',np.array([7],dtype=np.float32),2)
        with pytest.raises(ValueError,match='Seal'):flow.consume(2)
        stage.advance_write_floor(ordinal=2).wait()
        assert flow.consume(2) and not flow.consume(2) and flow.input_updates==1
        body=inspector.resolve('/World/BoxA')
        task=attached.step(.01);attached.wait_task(task)
        state=inspector.read(body);assert state.mass==pytest.approx(7)
        assert np.array_equal(read(stage,'/World/BoxA','omni:fabric:worldMatrix',2)[0],original[0])
        # The explicit output column is an independently owned copy, not a live
        # tensor/SDK pointer. This host chooses an inspect-only world-space field.
        source=np.array([state.pose[:3]],dtype=np.float32)
        output=OutputColumn('companion:output:worldPosition',source,'world','stage',semantic=5)
        source[:]=999
        with pytest.raises(ValueError):output.values.flags.writeable=True
        flow.publish(3,['/World/BoxA'],[output,OutputColumn('companion:output:mass',np.array([100],dtype=np.float32),'body','stage')])
        assert flow.cursor==3 and flow.output_batches==1 and flow.input_updates==1
        assert not flow.consume(3)
        assert read(stage,'/World/BoxA','companion:output:worldPosition',3)[0].reshape(-1)==pytest.approx(state.pose[:3])
        with AuthoringService(attached._lib) as authoring:
            mass=authoring.compute_mass_properties([MassShape(PrimitiveKind.BOX,(1,1,1),1)]).result(10)
            assert mass.mass==8
        assert owner.identity==flow._identity and inspector.read(body).step==state.step
        attached.wait_task(attached.step(.01))
        assert inspector.read(body).mass==pytest.approx(7)  # output mass 100 was never an input
        write(stage,'/World/BoxA','physics:density',np.array([9],dtype=np.float32),4)
        stage.advance_write_floor(ordinal=4).wait()
        assert flow.consume(4) and flow.input_updates==2
        attached.wait_task(attached.step(.01))
        assert read(stage,'/World/BoxA','companion:output:mass',4)[0].reshape(-1)==pytest.approx([100])
        assert inspector.read(body).mass==pytest.approx(7)


def test_preview_flow_owner_election_sealing_and_retirement(physics):
    import ovstage
    from pathlib import Path
    from concurrent.futures import ThreadPoolExecutor
    stage=ovstage.Stage('exchange-preview');other=ovstage.Stage('unrelated')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/exchange.usda'),ordinal=1,
                                    domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with PreviewSession(physics,stage,read_ordinal=1) as owner,Inspector(owner) as inspector:
            with pytest.raises(ValueError,match='belong'):StageFlow(owner,other,consumed_ordinal=1)
            flow=StageFlow(owner,stage,consumed_ordinal=1)
            with pytest.raises(RuntimeError,match='Share'):StageFlow(owner,stage,consumed_ordinal=1)
            write(stage,'/World/BoxA','physics:mass',np.array([3],dtype=np.float32),2)
            stage.advance_write_floor(ordinal=2).wait()
            column=OutputColumn('companion:output:value',np.array([42],dtype=np.float32),'none','unitless')
            with pytest.raises(ValueError,match='Consume'):flow.publish(3,['/World/BoxA'],[column])
            assert flow.consume(2)
            owner.step(.01)
            assert inspector.read(inspector.resolve('/World/BoxA')).mass==3
            flow.publish(3,['/World/BoxA'],[column]);assert not flow.consume(3)
            with ThreadPoolExecutor(1) as pool:
                with pytest.raises(RuntimeError,match='host thread'):pool.submit(flow.consume,3).result()
        with pytest.raises(Exception):flow.consume(4)
        flow.close()
    finally:other.destroy();stage.destroy()


def test_exchange_preflight_and_failed_publication_latch(exchange_attached,monkeypatch):
    attached=exchange_attached
    import ovstage
    stage=attached._attached_ovstage
    with ToolsSession(attached) as owner,StageFlow(owner,stage,consumed_ordinal=1) as flow:
        column=OutputColumn('companion:output:value',np.array([1],dtype=np.float32),'none','unitless')
        for paths,columns in ((['/World/BoxA','/World/BoxA'],[column]),(['/World/BoxA'],[column,column]),([],[])):
            with pytest.raises((ValueError,TypeError)):flow.publish(2,paths,columns)
        assert flow.cursor==1
        def fail(*args,**kwargs):raise RuntimeError('injected output failure')
        monkeypatch.setattr(ovstage.Stage,'write_attributes',fail)
        with pytest.raises(RuntimeError,match='injected'):flow.publish(2,['/World/BoxA'],[column])
        with pytest.raises(RuntimeError,match='latched'):flow.consume(2)
        assert flow.cursor==1 and flow.output_batches==0


def test_explicit_flow_sample():
    import os,subprocess,sys,ovphysx_companion
    from pathlib import Path
    env=dict(os.environ);env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    result=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'samples/ovstage_flow.py')],env=env,
                          capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr


def test_ragged_output_explicit_lanes_and_cpu_copy(exchange_attached):
    stage=exchange_attached._attached_ovstage
    a=np.array([[1,2,3],[4,5,6]],dtype=np.float32);b=np.array([[7,8,9]],dtype=np.float32)
    with pytest.raises(ValueError,match='lanes'):OutputColumn('companion:output:points',(a,b),'world','stage',is_array=True)
    column=OutputColumn('companion:output:points',(a,b),'world','stage',is_array=True,semantic=5,lanes=3)
    a[:]=999
    with ToolsSession(exchange_attached) as owner,StageFlow(owner,stage,consumed_ordinal=1) as flow:
        flow.publish(2,['/World/BoxA','/World/BoxB'],[column])
        assert read(stage,'/World/BoxA','companion:output:points',2)[0].reshape(-1,3).tolist()==[[1,2,3],[4,5,6]]
        assert read(stage,'/World/BoxB','companion:output:points',2)[0].reshape(-1,3).tolist()==[[7,8,9]]
        assert flow.input_updates==0 and not flow.consume(2)
