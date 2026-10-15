# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVD-001
# @covers AC-1 AC-2 AC-4
# @implements REQ-OVD-DECODE-001
# @covers AC-1 AC-2 AC-3
import ctypes as C
from pathlib import Path
import struct
import pytest
from ovphysx_companion.ovd import decode, DecodeError, Document, attribute_values, _Api, _Limits, _Diagnostic
from ovphysx_companion.native import Status


def string(text):
    data=text.encode();return struct.pack('<H',len(data))+data


def stream():
    header=struct.pack('<III',0,4,0)
    records=[struct.pack('<BII',1,1,0)+string('PxRigidActor'),
             struct.pack('<BIIHI',3,1,2,8,7)+string('globalPose'),
             struct.pack('<BQIQ',9,1,1,42)+string('body'),
             struct.pack('<BQQ',11,1,5),
             struct.pack('<BQQBIII',6,1,42,2,7,2,28)+struct.pack('<7f',0,0,0,1,2,3,4),
             struct.pack('<BQQ',12,1,5),struct.pack('<BQQ',10,1,42)]
    return header,records


def test_native_ovd_decode_schema_full_path_timestamps_and_copied_lifetime(physics):
    header,records=stream();data=bytearray(header+b''.join(records))
    document=decode(physics._lib,data)
    data[:]=b'\0'*len(data)
    assert document.version==(0,4,0)
    assert [r.command for r in document.records]==[1,3,9,11,6,12,10]
    assert document.records[4].attribute_path==(7,2)
    assert document.records[3].timestamp==5 and document.records[3].context==1
    assert attribute_values(8,document.records[4].payload)==(0,0,0,1,2,3,4)
    assert Document.from_json(document.to_json())==document


def test_native_ovd_truncation_limits_unknown_commands_and_versions(physics):
    header,records=stream();data=header+b''.join(records)
    boundaries={len(header)}
    cursor=len(header)
    for record in records:cursor+=len(record);boundaries.add(cursor)
    for length in range(len(data)):
        if length in boundaries:continue
        with pytest.raises(DecodeError):decode(physics._lib,data[:length])
    for malformed in (struct.pack('<III',1,0,0),header+b'\xff',header+b'\x02'):
        with pytest.raises(DecodeError) as error:decode(physics._lib,malformed)
        assert error.value.status==Status.UNAVAILABLE
    with pytest.raises(DecodeError) as error:decode(physics._lib,data,max_records=1)
    assert error.value.status==8
    with pytest.raises(DecodeError) as error:decode(physics._lib,data,max_string_bytes=2)
    assert error.value.status==8
    huge=header+struct.pack('<BQQBII',6,1,42,1,2,0xffffffff)
    with pytest.raises(DecodeError) as error:decode(physics._lib,huge)
    assert error.value.status==8 and error.value.offset<=len(huge)
    empty_path=header+struct.pack('<BQQB',6,1,42,0)
    with pytest.raises(DecodeError) as error:decode(physics._lib,empty_path)
    assert error.value.status==9


def test_native_ovd_c_table_negotiation_and_failed_document_canary(physics):
    acquire=physics._lib.ovphysx_companion_ovd_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    api=_Api();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    limits=_Limits(C.sizeof(_Limits),1,1024,100,1024,100)
    output=C.c_uint64(0xa5a5);diagnostic=_Diagnostic()
    data=struct.pack('<III',0,4,0)+b'\x01'
    assert api.decode(C.c_char_p(data),len(data),C.byref(limits),C.byref(output),C.byref(diagnostic))==9
    assert output.value==0xa5a5 and diagnostic.offset==13 and diagnostic.message


def test_native_ovd_real_recording_and_independent_preview_admission(tmp_path):
    import os
    import subprocess
    import sys
    import ovphysx_companion
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    sample=Path(__file__).resolve().parents[1]/'samples/ovd_recording.py'
    output=tmp_path/'record.json'
    result=subprocess.run([sys.executable,str(sample),'--worker',str(output)],env=env,capture_output=True,text=True,timeout=90)
    assert result.returncode==0,result.stdout+result.stderr
    document=Document.from_json(output.read_text())
    assert any(r.command==9 for r in document.records) and any(r.command==11 for r in document.records)


def test_preview_attach_does_not_start_or_restart_recording(tmp_path):
    # Recording capability is startup-only, so use a fresh process.
    import os
    import subprocess
    import sys
    import textwrap
    import ovphysx_companion
    companion=Path(__file__).resolve().parents[1]
    first=tmp_path/'first.ovd'
    late=tmp_path/'late.ovd'
    script=textwrap.dedent(f"""
        import ovphysx,ovstage
        from ovphysx import GlobalConfigKey, OmniPvdDestination, set_global_config
        from ovphysx_companion.preview import PreviewSession
        from ovphysx_companion.schemas import physx_schema_paths
        ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
        ovphysx.PhysX.set_cpu_mode(True)
        set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE,True)
        physics=ovphysx.PhysX()
        stages=[]
        def stage(name):
            s=ovstage.Stage(name);stages.append(s)
            ovstage.population.open_usd(s,{str(companion/'samples/scenes/two_boxes.usda')!r},ordinal=1,
                                        domains=ovstage.PopulationDomain.ALL)
            s.advance_write_floor(ordinal=1).wait()
            return s
        try:
            assert not physics.is_recording()
            physics.wait_task(physics.attach_ovstage(stage('core'),read_ordinal=1))
            assert not physics.is_recording()
            with PreviewSession(physics,stage('initial-preview'),read_ordinal=1) as preview:
                assert not physics.is_recording()
                preview.step(1/60)
                assert not physics.is_recording()
            physics.start_recording(OmniPvdDestination.file({str(first)!r}))
            physics.step_sync(1/60)
            assert physics.is_recording()
            physics.stop_recording()
            with PreviewSession(physics,stage('preview'),read_ordinal=1) as preview:
                preview.step(1/60)
                assert not physics.is_recording()
                physics.start_recording(OmniPvdDestination.file({str(late)!r}))
                assert physics.is_recording()
                physics.step_sync(1/60)
                physics.stop_recording()
                assert not physics.is_recording()
        finally:
            physics.destroy()
            for s in stages: s.destroy()
    """)
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    result=subprocess.run([sys.executable,'-c',script],env=env,capture_output=True,text=True,timeout=120)
    assert result.returncode==0,result.stdout+result.stderr
    assert first.stat().st_size>12
    assert late.stat().st_size>12
    assert {path.name for path in tmp_path.iterdir()}=={'first.ovd','late.ovd'}
