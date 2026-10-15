# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CONFIG-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-CONFIG-SNAPSHOT-001
# @covers AC-1 AC-2 AC-3
import ctypes as C
from dataclasses import fields
from pathlib import Path
import pytest
from ovphysx_companion.configuration import (
    CATALOG,Scope,AttachmentOptions,AttachmentConfiguration,AttachmentSnapshot,
    Pipeline,Solver,HostPreferences,set_global,get_global,set_instance_devices,get_instance_devices,
    _Api,_Snapshot)
from ovphysx_companion.native import ToolsSession,CompanionError,Status


@pytest.fixture
def stages(physics):
    import ovstage
    result=[]
    try:
        for name in ('settings-B','settings-C'):
            stage=ovstage.Stage(name);result.append(stage)
            ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),
                ordinal=1,domains=ovstage.PopulationDomain.ALL)
            stage.advance_write_floor(ordinal=1).wait()
        yield result
    finally:
        for stage in reversed(result):stage.destroy()


def test_configuration_copied_options_and_owner_local_mutation(attached,stages):
    old_default=get_global('disable_contact_processing')
    with ToolsSession(attached) as tools:
        a=AttachmentConfiguration(tools);old_a=a.snapshot
        with AttachmentOptions(threads=2,gpu_pipeline=Pipeline.CPU,solver=Solver.PGS).open_preview(attached,stages[0],read_ordinal=1) as b, \
             AttachmentOptions(threads=3,gpu_pipeline=Pipeline.CPU,solver=Solver.TGS).open_preview(attached,stages[1],read_ordinal=1) as c:
            bc,cc=AttachmentConfiguration(b),AttachmentConfiguration(c)
            assert (bc.snapshot.threads,bc.snapshot.solver)==(2,Solver.PGS)
            assert (cc.snapshot.threads,cc.snapshot.solver)==(3,Solver.TGS)
            old_c=cc.snapshot
            assert bc.set_contact_processing_disabled(not old_a.disable_contact_processing).disable_contact_processing!=old_a.disable_contact_processing
            assert a.snapshot==old_a and cc.snapshot==old_c
            assert get_global('disable_contact_processing')==old_default
            b.simulate(.01)
            with pytest.raises(CompanionError) as error:bc.set_contact_processing_disabled(False)
            assert error.value.status==Status.BUSY
            cc.set_contact_processing_disabled(True)  # A pending peer does not block C.
            b.fetch()
            assert bc.snapshot.step>0
            assert bc.snapshot.disable_contact_processing!=old_a.disable_contact_processing
        assert a.snapshot==old_a


def test_configuration_abi_stale_and_output_canaries(physics,stages):
    api=_Api();acquire=physics._lib.ovphysx_companion_configuration_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    with AttachmentOptions().open_preview(physics,stages[0],read_ordinal=1) as preview:
        config=AttachmentConfiguration(preview);owner=config.snapshot.attachment
        output=_Snapshot();C.memset(C.byref(output),0xa5,C.sizeof(output));before=bytes(output)
        for disabled,size in ((2,C.sizeof(output)),(0,C.sizeof(output)-1)):
            assert api.set_contact_processing_disabled(owner,disabled,size,C.byref(output))==Status.INVALID_ARGUMENT
            assert bytes(output)==before
        preview.simulate(.01)
        assert api.get_configuration(owner,C.sizeof(output),C.byref(output))==Status.BUSY and bytes(output)==before
        preview.fetch()
    assert api.set_contact_processing_disabled(owner,1,C.sizeof(output),C.byref(output))==Status.STALE and bytes(output)==before
    with pytest.raises(CompanionError):_ = config.snapshot


def test_configuration_core_lifecycle_and_hard_cpu_conflicts(attached,stages):
    for key,value in (('cpu_only',False),('num_threads',3),('profiler_backend',0)):
        before=get_global(key)
        with pytest.raises(Exception):set_global(key,value)
        assert get_global(key)==before
    devices=get_instance_devices(attached)
    with pytest.raises(Exception):set_instance_devices(attached,'0')
    assert get_instance_devices(attached)==devices
    with pytest.raises(CompanionError) as error:
        AttachmentOptions(gpu_pipeline=Pipeline.GPU).open_preview(attached,stages[0],read_ordinal=1)
    assert error.value.status==Status.UNAVAILABLE
    with ToolsSession(attached) as tools:assert tools.scene_statistics("/World/PhysicsScene").dynamic_bodies==2
    with pytest.raises(ValueError):AttachmentOptions(threads=True)
    with pytest.raises(ValueError):AttachmentOptions(solver=0.0)


def test_configuration_catalog_and_host_preferences():
    from concurrent.futures import ThreadPoolExecutor
    from ovphysx import GlobalConfigKey
    process=[s for s in CATALOG.values() if s.scope==Scope.PROCESS]
    assert len(process)==len(GlobalConfigKey)
    for item in CATALOG.values():assert item.phase and item.persistence and item.description
    for field in fields(AttachmentSnapshot):
        if field.name not in ('attachment','step'):assert 'attachment.'+field.name in CATALOG
    store={};prefs=HostPreferences(store)
    assert prefs.get('show_debug_geometry') is False
    prefs.set('show_debug_geometry',True)
    assert HostPreferences(store).get('show_debug_geometry') is True
    with pytest.raises(ValueError):prefs.set('solver',True)
    with ThreadPoolExecutor(1) as pool:
        with pytest.raises(RuntimeError,match='creating thread'):pool.submit(prefs.get,'show_debug_geometry').result()
    assert store=={'show_debug_geometry':True}


def test_configuration_ovd_recording_capability_default_and_core_delegation():
    import os,subprocess,sys,textwrap,ovphysx_companion
    setting=CATALOG['process.ovd_recording_capable']
    assert setting.scope==Scope.PROCESS and setting.default is False
    assert setting.phase=='before-first-instance'
    script=textwrap.dedent("""
        from ovphysx import PhysX,GlobalConfigKey,get_global_config,set_global_config
        from ovphysx_companion.configuration import get_global,set_global
        key=GlobalConfigKey.OVD_RECORDING_CAPABLE
        assert get_global('ovd_recording_capable') is False
        for invalid in (1,'true',None):
            try:set_global('ovd_recording_capable',invalid)
            except (TypeError,ValueError):pass
            else:raise AssertionError('Non-bool recording capability accepted')
            assert get_global('ovd_recording_capable') is False
        set_global('ovd_recording_capable',True)
        assert get_global_config(key) is True
        set_global_config(key,False)
        assert get_global('ovd_recording_capable') is False
        set_global('ovd_recording_capable',True)
        PhysX.set_cpu_mode(True)
        physics=PhysX()
        try:
            assert not physics.is_recording()
            for value in (False,True):
                try:set_global('ovd_recording_capable',value)
                except RuntimeError:pass
                else:raise AssertionError('Late recording capability write accepted')
                assert get_global('ovd_recording_capable') is True
        finally:physics.destroy()
        set_global('ovd_recording_capable',False)
        assert get_global_config(key) is False
    """)
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    result=subprocess.run([sys.executable,'-c',script],env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr


def test_settings_preview_sample():
    import os,subprocess,sys,ovphysx_companion
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    result=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'samples/scoped_settings.py')],
                          env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr
