# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVD-001
# @covers AC-2 AC-3 AC-4
import struct
from dataclasses import replace
import pytest
from pxr import Usd, UsdGeom
from ovphysx_companion.ovd import Document, Record
from ovphysx_companion.usd import EditSession, StaleResultError
from ovphysx_companion.usd.ovd import convert


def document():
    return Document((0,4,0),(
        Record(12,1,class_handle=1,name='PxRigidActor'),
        Record(30,3,class_handle=1,attribute=2,name='globalPose',data_type=8,elements=7),
        Record(50,5,class_handle=1,attribute=3,name='shapes',data_type=11),
        Record(70,9,context=1,object=42,class_handle=1,name='first'),
        Record(90,11,context=99,timestamp=500),
        Record(110,6,context=1,object=42,attribute=2,attribute_path=(2,),payload=struct.pack('<7f',0,0,0,1,2,3,4)),
        Record(140,7,context=1,object=42,attribute=3,attribute_path=(3,),payload=struct.pack('<Q',18)),
        Record(160,8,context=1,object=42,attribute=3,attribute_path=(3,),payload=struct.pack('<Q',18)),
        Record(180,12,context=99,timestamp=500),
        Record(200,10,context=1,object=42),
        Record(220,9,context=1,object=42,class_handle=1,name='replacement'),
        Record(240,6,context=1,object=42,attribute=2,attribute_path=(2,),payload=struct.pack('<7f',0,0,0,1,5,6,7)),
        Record(260,13,context=1,name='warning',file='source.cpp',line=12,message_type=2),
    ))


def test_ovd_customer_usd_lifetimes_typed_attributes_units_timeline_and_undo():
    stage=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage,.01)
    layer=stage.GetRootLayer();before=layer.ExportToString()
    with EditSession(stage) as session:
        result=convert(session,document(),root='/Capture',layer=layer,meters_per_unit=1)
        first,second=result.objects
        assert (first.handle,first.generation,second.handle,second.generation)==(42,1,42,2)
        assert first.path!=second.path and first.died==9
        assert result.frames[0].timestamp==500 and result.frames[0].time_code==4
        assert result.timeline=='record-index'
        assert result.object_at(1,42,8) is first
        assert result.object_at(1,42,10) is second
        assert result.object_at(1,42,9) is None
        prim=stage.GetPrimAtPath(first.path)
        image=UsdGeom.Imageable(prim)
        assert image.ComputeVisibility(2)==UsdGeom.Tokens.invisible
        assert image.ComputeVisibility(8)==UsdGeom.Tokens.inherited
        assert image.ComputeVisibility(9)==UsdGeom.Tokens.invisible
        pose=UsdGeom.XformCache(5).GetLocalToWorldTransform(prim).ExtractTranslation()
        assert tuple(pose)==pytest.approx((200,300,400))
        assert list(prim.GetAttribute('ovd:a3_shapes').Get(6))==[18]
        assert list(prim.GetAttribute('ovd:a3_shapes').Get(7))==[]
        assert 'warning' in stage.GetPrimAtPath('/Capture').GetCustomDataByKey('ovd:messages')
        after=layer.ExportToString()
        result.patch.undo();assert layer.ExportToString()==before
        result.patch.redo();assert layer.ExportToString()==after
        assert UsdGeom.GetStageMetersPerUnit(stage)==.01


def test_ovd_customer_conversion_preflights_schema_and_timeline_before_edit():
    stage=Usd.Stage.CreateInMemory();layer=stage.GetRootLayer()
    with EditSession(stage) as session:
        before=layer.ExportToString()
        cases=(Document((0,4,0),(Record(1,9,context=1,object=2,class_handle=100),)),
               Document((0,4,0),document().records[:5]+(replace(document().records[5],payload=b'bad'),)))
        for doc in cases:
            with pytest.raises(ValueError):convert(session,doc,root='/Capture',layer=layer,meters_per_unit=1)
            assert layer.ExportToString()==before
        with pytest.raises(ValueError):convert(session,document(),root='/Capture',layer=layer,meters_per_unit=1,time_for_record=lambda i,r:-i)
        assert layer.ExportToString()==before
        source=session.source
        def mutate(index,record):
            if index==0:stage.DefinePrim('/External')
            return index
        with pytest.raises(StaleResultError):
            convert(session,document(),root='/Capture',layer=layer,meters_per_unit=1,source=source,time_for_record=mutate)
        assert not stage.GetPrimAtPath('/Capture')


def test_ovd_real_native_capture_customer_usd_roundtrip():
    from ovd_recording import run
    assert run()>0


def test_ovd_flag_word_width_and_duplicate_retirement_diagnostic():
    from ovphysx_companion.ovd import attribute_values
    for width in (1,2,4,8):
        assert attribute_values(13,(3).to_bytes(width,'little'))==(3,)
    with pytest.raises(ValueError):attribute_values(13,b'bad')
    stage=Usd.Stage.CreateInMemory()
    source=document()
    source=Document(source.version,source.records+(Record(300,10,context=1,object=99),))
    with EditSession(stage) as session:
        result=convert(session,source,root='/Capture',layer=stage.GetRootLayer(),meters_per_unit=1)
        assert result.diagnostics==( ('OVD_DESTROY_WITHOUT_LIVE_OBJECT',13,1,99), )
