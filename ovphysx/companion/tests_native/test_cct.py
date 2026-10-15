# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CCT-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-CCT-CONTROL-001
# @covers AC-1 AC-2 AC-3 AC-4
import ctypes as C
from pathlib import Path
import numpy as np
import pytest
from ovphysx_companion.cct import CharacterController, Collision, Filter, _Api, _Command, _State
from ovphysx_companion.preview import PreviewSession
from ovphysx_companion.native import CompanionError, Status


@pytest.fixture
def cct_stage(physics):
    import ovstage
    stage=ovstage.Stage('native-character')
    ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/character.usda'),
                                ordinal=1,domains=ovstage.PopulationDomain.PHYSICS|ovstage.PopulationDomain.RENDERING)
    stage.advance_write_floor(ordinal=1).wait()
    yield stage
    stage.destroy()


def test_native_character_gravity_collision_resize_teleport_and_close(physics,cct_stage):
    with PreviewSession(physics,cct_stage,read_ordinal=1) as preview:
        with CharacterController(preview,'/World/Character') as controller:
            first=controller.state
            assert first.height==1 and first.radius==.5 and first.up==(0,0,1)
            assert first.generation>0 and first.key>0
            controller.set_gravity('scene')
            for _ in range(120):
                state=controller.move(dt=1/60)
                preview.step(1/60)
            assert state.collisions&Collision.BELOW
            assert 1.0<state.position[2]<1.25
            foot=state.position[2]-state.height/2-state.radius
            resized=controller.resize(height=2,radius=.4)
            assert resized.height==2 and resized.radius==pytest.approx(.4)
            assert resized.position[2]-resized.height/2-resized.radius==pytest.approx(foot,abs=1e-5)
            assert controller.teleport((2,6,4)).position==(2,6,4)
            controller.set_gravity(None)
            np.testing.assert_allclose(controller.move((1,0,0),dt=.1).position,(3,6,4),atol=1e-5)
        with pytest.raises(CompanionError) as error:CharacterController(preview,'/World/Character')
        assert error.value.status==Status.NOT_FOUND
        preview.step(.01) # retiring the native CCT leaves its attachment usable


def test_character_filters_and_pending_owner_rejection(physics,cct_stage):
    with PreviewSession(physics,cct_stage,read_ordinal=1) as preview:
        with CharacterController(preview,'/World/Character') as controller:
            preview.simulate(.01)
            with pytest.raises(CompanionError) as error:controller.move((0,0,-1),dt=.1)
            assert error.value.status==Status.BUSY
            with pytest.raises(CompanionError) as error:controller.close()
            assert error.value.status==Status.BUSY
            preview.fetch()
            assert controller.state.position==(0,0,3)
            state=controller.move((0,0,-5),dt=.1,filters=Filter.DYNAMIC|Filter.CONTROLLERS)
            assert state.position[2]<-1 and not state.collisions&Collision.BELOW


def test_character_table_and_command_output_canaries(physics,cct_stage):
    api=_Api();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    acquire=physics._lib.ovphysx_companion_cct_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    with PreviewSession(physics,cct_stage,read_ordinal=1) as preview:
        with CharacterController(preview,'/World/Character') as controller:
            command=_Command(C.sizeof(_Command),1,1,0,controller.state.generation)
            command.dt=-1
            output=_State();C.memset(C.byref(output),0xa5,C.sizeof(output));before=bytes(output)
            assert api.execute(preview.identity.attachment,b'/World/Character',C.byref(command),C.sizeof(output),C.byref(output))==Status.INVALID_ARGUMENT
            assert bytes(output)==before
            command.action=0;command.generation+=1
            assert api.execute(preview.identity.attachment,b'/World/Character',C.byref(command),C.sizeof(output),C.byref(output))==Status.STALE
            assert bytes(output)==before


def test_character_close_after_owner_retirement_is_safe(physics,cct_stage):
    preview=PreviewSession(physics,cct_stage,read_ordinal=1)
    controller=CharacterController(preview,'/World/Character')
    preview.close()
    controller.close()
    with PreviewSession(physics,cct_stage,read_ordinal=1) as replacement:
        with CharacterController(replacement,'/World/Character') as new:
            assert new.state.position==(0,0,3)
            controller.close() # must not remove the new owner's controller
            assert new.state.height==1


def test_character_peer_isolation_and_wall(physics,cct_stage):
    import ovstage
    other=ovstage.Stage('character-peer')
    ovstage.population.open_usd(other,str(Path(__file__).resolve().parents[1]/'samples/scenes/character.usda'),
                                ordinal=1,domains=ovstage.PopulationDomain.PHYSICS|ovstage.PopulationDomain.RENDERING)
    other.advance_write_floor(ordinal=1).wait()
    try:
        with PreviewSession(physics,cct_stage,read_ordinal=1) as first, PreviewSession(physics,other,read_ordinal=1) as second:
            with CharacterController(first,'/World/Character') as a, CharacterController(second,'/World/Character') as b:
                first.simulate(.01)
                result=b.move((10,0,0),dt=.1)
                assert result.collisions&Collision.SIDES and result.position[0]<3
                first.fetch()
                assert a.state.position==(0,0,3)
                b.close()
                assert a.state.height==1
    finally:other.destroy()


def test_character_extension_sample_in_native_worker():
    import os
    import subprocess
    import sys
    import ovphysx_companion
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    sample=Path(__file__).resolve().parents[1]/'samples/character_controller.py'
    result=subprocess.run([sys.executable,str(sample)],env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr
