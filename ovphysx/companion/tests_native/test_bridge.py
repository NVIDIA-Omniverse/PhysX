# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-COMPANION-001
# @covers AC-1 AC-2 AC-3
# @implements REQ-COMP-OWNER-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-API-001
# @covers AC-1 AC-2 AC-3
# @implements REQ-COMP-DEBUG-001
# @covers AC-2

import ctypes as C
import gc

import pytest

from ovphysx_companion.native import Capability, CompanionError, Status, ToolsSession, _Api


def test_abi_negotiation(physics):
    acquire = physics._lib.ovphysx_companion_acquire
    acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_Api)]
    acquire.restype = C.c_uint32
    table = _Api()
    C.memset(C.byref(table), 0xA5, C.sizeof(table))
    before = bytes(table)
    assert acquire(99, C.sizeof(table), C.byref(table)) == Status.ABI_MISMATCH
    assert bytes(table) == before
    assert acquire(1, C.sizeof(table) - 1, C.byref(table)) == Status.ABI_MISMATCH
    assert bytes(table) == before
    assert acquire(1, C.sizeof(table), C.byref(table)) == Status.OK
    assert table.size == C.sizeof(table)
    assert table.capabilities & Capability.BORROWED_SESSION


def test_open_requires_existing_attachment(physics):
    with pytest.raises(CompanionError) as error:
        ToolsSession(physics)
    assert error.value.status == Status.STALE


def test_statistics_and_retryable_owner_destroy(attached):
    handle = attached._omni_physx_sdk_handle.value
    with ToolsSession(attached) as tools:
        identity = tools.identity
        stats = tools.scene_statistics("/World/PhysicsScene")
        assert stats.owner == identity
        assert stats.dynamic_bodies == 2
        assert stats.static_bodies == 1
        with pytest.raises(RuntimeError, match="leases.*retry"):
            attached.destroy()
        assert attached._omni_physx_sdk_handle.value == handle
        assert tools.identity == identity
        attached.step(1 / 60)
        attached.wait_all()
        assert tools.scene_statistics("/World/PhysicsScene").dynamic_bodies == 2
        with pytest.raises(ValueError, match="NUL"):
            tools.scene_statistics("/World/PhysicsScene\0other")
    tools.close()
    with pytest.raises(CompanionError) as error:
        _ = tools.identity
    assert error.value.status == Status.NOT_FOUND


def test_closed_peer_and_gc_leave_other_session_usable(attached):
    with ToolsSession(attached) as keeper:
        other = ToolsSession(attached)
        del other
        gc.collect()
        assert keeper.scene_statistics("/World/PhysicsScene").dynamic_bodies == 2


def test_detach_makes_session_stale(attached):
    with ToolsSession(attached) as tools:
        attached.wait_task(attached.detach_ovstage())
        with pytest.raises(CompanionError) as error:
            _ = tools.identity
        assert error.value.status == Status.STALE


def test_tools_session_closes_before_core_at_process_exit():
    import os
    from pathlib import Path
    import subprocess
    import sys
    companion = Path(__file__).resolve().parents[1]
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join([str(companion/'python'), env.get('PYTHONPATH', '')])
    code = '''
import atexit
from ovphysx import PhysX
from ovphysx_companion.native import ToolsSession
PhysX.set_cpu_mode(True)
physics = PhysX()
# Register before opening the tools lease: its cleanup must execute first.
def verify():
    physics.destroy()
    assert physics._omni_physx_sdk_handle is None
    print('tools-exit-clean')
atexit.register(verify)
import ovstage, ovphysx, sys
from ovphysx_companion.schemas import physx_schema_paths
schemas=[str(p) for p in physx_schema_paths()]
if ovphysx.newton_schema_root():schemas.append(str(ovphysx.newton_schema_root()))
ovstage.population.register_usd_schemas(schemas)
stage=ovstage.Stage('tools-exit')
ovstage.population.open_usd(stage,sys.argv[1],ordinal=1,domains=ovstage.PopulationDomain.ALL)
stage.advance_write_floor(ordinal=1).wait()
physics.wait_task(physics.attach_ovstage(stage,read_ordinal=1))
session=ToolsSession(physics)
'''
    result = subprocess.run([sys.executable, '-c', code, str(companion/'samples/scenes/two_boxes.usda')],
                            env=env, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert 'tools-exit-clean' in result.stdout, result.stderr
