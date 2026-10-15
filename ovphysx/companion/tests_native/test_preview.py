# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PREVIEW-001
# @covers AC-1 AC-2 AC-4 AC-5 AC-6 AC-7
# @implements REQ-CAPI-COMPANION-001
# @covers AC-4
import ctypes as C
from pathlib import Path
import pytest
from ovphysx_companion.native import CompanionError, Status, ToolsSession
from ovphysx_companion.preview import PreviewSession, _PreviewApi


@pytest.fixture
def preview_stage(physics):
    import ovstage
    stage = ovstage.Stage('independent-inspector')
    ovstage.population.open_usd(stage, str(Path(__file__).resolve().parents[1] / 'samples/scenes/two_boxes.usda'),
                                ordinal=1, domains=ovstage.PopulationDomain.ALL)
    stage.advance_write_floor(ordinal=1).wait()
    yield stage
    stage.destroy()


# @implements REQ-COMP-OVSTAGE-001
# @covers AC-1
def test_population_nested_transforms_preserved_by_live_and_preview(physics,tmp_path):
    import ovstage
    import numpy as np
    from ovphysx_companion.inspection import Inspector
    source=tmp_path/'nested.usda'
    source.write_text('''#usda 1.0
(
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "World" {
    double3 xformOp:translate = (10, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
    def Cube "Body" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]) {
        double size = 1
        double3 xformOp:translate = (1, 2, 3)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
''')
    stage=ovstage.Stage('nested-companion-population')
    preview_stage=ovstage.Stage('nested-companion-preview')
    try:
        for candidate in (stage,preview_stage):
            ovstage.population.open_usd(candidate,str(source),ordinal=1,domains=ovstage.PopulationDomain.ALL)
            candidate.advance_write_floor(ordinal=1).wait()
            with ovstage.PathDictionary(candidate) as dictionary:
                world=dictionary.intern_token('omni:fabric:worldMatrix')
                with dictionary.create_path_list_from_strings(['/World/Body']) as selected:
                    with candidate.query_from_path_list(selected) as query:
                        with candidate.read_attributes(query,[world],ovstage.OrdinalRange.latest(1)) as batch:
                            batch.wait()
                            matrices=[]
                            for group in batch.groups():
                                with group:
                                    for i in range(group.tensor_count):
                                        matrices.append(np.array(group.array(i),copy=True))
            assert len(matrices)==1
            assert matrices[0].reshape(4,4)[3,:3]==pytest.approx((11,2,3))
        physics.wait_task(physics.attach_ovstage(stage,read_ordinal=1))
        with ToolsSession(physics) as tools,Inspector(tools) as live:
            assert live.read(live.resolve('/World/Body')).pose[:3]==pytest.approx((11,2,3))
            with PreviewSession(physics,preview_stage,read_ordinal=1) as preview:
                assert preview.rigid_pose('/World/Body').position==pytest.approx((11,2,3))
    finally:
        physics.wait_task(physics.detach_ovstage())
        preview_stage.destroy()
        stage.destroy()


def test_cyclic_gc_closes_preview_before_its_stage_is_finalized(monkeypatch):
    # A host callback exception can leave preview and Stage in one unreachable cycle
    # without an explicit close. Without the module-scope pin the collector runs the
    # Stage finalizer (allocated first) before the preview finalizer closes native.
    import gc
    import ovphysx.api
    monkeypatch.setattr(ovphysx.api, '_acquire_process_lifecycle', lambda owner=None: None)
    monkeypatch.setattr(ovphysx.api, '_release_process_lifecycle', lambda: None)
    events = []

    class Stage:
        def handle(self):
            return 1

        def __del__(self):
            events.append('stage finalized')

    def open_preview(desc, handle):
        handle[0] = 7
        return Status.OK

    def close_preview(handle):
        events.append('preview closed')
        return Status.OK

    fields = dict(_PreviewApi._fields_)
    callbacks = {'open': open_preview, 'close': close_preview}

    def acquire(version, size, table):
        api = table._obj
        api.size, api.version = C.sizeof(_PreviewApi), 1
        for name in ('open', 'close', 'identity', 'simulate', 'fetch', 'statistics', 'rigid_pose', 'update'):
            setattr(api, name, fields[name](callbacks.get(name, lambda *args: Status.OK)))
        return Status.OK

    class Library:
        ovphysx_companion_preview_acquire = staticmethod(acquire)

    class Physics:
        _lib = Library()
        _omni_physx_sdk_handle = C.c_uint64(1)

    stage = Stage()
    preview = PreviewSession(Physics(), stage, read_ordinal=1)
    cycle = [stage, preview]  # e.g. a stored traceback's frames
    cycle.append(cycle)
    del stage, preview, cycle
    gc.collect()
    assert events == ['preview closed', 'stage finalized']


def test_preview_table_failure_preserves_output(physics):
    table = _PreviewApi()
    acquire = physics._lib.ovphysx_companion_preview_acquire
    acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_PreviewApi)]
    acquire.restype = C.c_uint32
    C.memset(C.byref(table), 0xa5, C.sizeof(table))
    before = bytes(table)
    assert acquire(8, C.sizeof(table), C.byref(table)) == Status.ABI_MISMATCH
    assert bytes(table) == before
    assert acquire(1, C.sizeof(table)-1, C.byref(table)) == Status.ABI_MISMATCH
    assert bytes(table) == before


def test_preview_pending_isolation_and_close_preserves_playback_receipt(attached, preview_stage):
    with ToolsSession(attached) as tools:
        preview = PreviewSession(attached, preview_stage, read_ordinal=1, threads=2, gpu_pipeline=0)
        try:
            assert preview.identity.attachment != tools.identity.attachment
            before = preview.rigid_pose('/World/BoxA')
            preview.simulate(0.1)
            with pytest.raises(CompanionError) as error:
                preview.simulate(0.1)
            assert error.value.status == Status.BUSY
            with pytest.raises(CompanionError) as error:
                preview.rigid_pose('/World/BoxA')
            assert error.value.status == Status.BUSY
            playback_task = attached.step(0.01)
            preview.fetch()
            after = preview.rigid_pose('/World/BoxA')
            assert after.position[2] < before.position[2]
            assert preview.scene_statistics('/World/PhysicsScene').dynamic_bodies == 2
            preview.simulate(0.01)
            preview.close()  # drains B only; A task must still be collectible
            attached.wait_task(playback_task)
            assert tools.scene_statistics('/World/PhysicsScene').dynamic_bodies == 2
        finally:
            preview.close()


# @implements REQ-COMP-OWNER-001
# @covers AC-3
def test_duplicate_sources_and_independent_public_owners(attached, preview_stage):
    import ovphysx
    import ovstage
    with pytest.raises(CompanionError) as error:
        PreviewSession(attached, attached._attached_ovstage, read_ordinal=1)
    assert error.value.status == Status.INVALID_ARGUMENT
    with PreviewSession(attached, preview_stage, read_ordinal=1) as preview:
        with pytest.raises(CompanionError) as error:
            PreviewSession(attached, preview_stage, read_ordinal=1)
        assert error.value.status == Status.INVALID_ARGUMENT
        peer = ovphysx.PhysX()
        peer_stage = ovstage.Stage('independent-public-peer')
        try:
            task = peer.attach_ovstage(preview_stage, read_ordinal=1)
            assert task.op_index != 0
            with pytest.raises(ovphysx.TaskError, match='already attached'):
                peer.wait_task(task)
            ovstage.population.open_usd(peer_stage,
                str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),
                ordinal=1,domains=ovstage.PopulationDomain.ALL)
            peer_stage.advance_write_floor(ordinal=1).wait()
            peer.wait_task(peer.attach_ovstage(peer_stage,read_ordinal=1))
            peer.step_sync(.01)
            with ToolsSession(peer) as peer_tools,ToolsSession(attached) as playback_tools:
                assert peer_tools.scene_statistics('/World/PhysicsScene').dynamic_bodies==2
                assert playback_tools.scene_statistics('/World/PhysicsScene').dynamic_bodies==2
                assert preview.scene_statistics('/World/PhysicsScene').dynamic_bodies==2
        finally:
            peer.destroy()
            peer_stage.destroy()
        preview.step(0.01)
        attached.step_sync(0.01)
        with ToolsSession(attached) as playback_tools:
            assert playback_tools.scene_statistics('/World/PhysicsScene').dynamic_bodies == 2


def test_bootstrap_owner_can_be_destroyed_before_preview(physics, preview_stage):
    import ovphysx
    bootstrap = ovphysx.PhysX()
    preview = PreviewSession(bootstrap, preview_stage, read_ordinal=1)
    try:
        identity = preview.identity
        bootstrap.destroy()
        preview.step(0.01)
        assert preview.identity == identity
        assert preview.scene_statistics('/World/PhysicsScene').dynamic_bodies == 2
        assert physics._lib.ovphysx_shutdown().status != 0
        preview.step(0.01)
    finally:
        preview.close()
        bootstrap.destroy()


@pytest.mark.parametrize('kwargs', [dict(threads=0), dict(threads=-1), dict(gpu_pipeline=2),
                                    dict(cuda_device=-2), dict(solver=3), dict(read_ordinal=0)])
def test_preview_rejects_bad_configuration_without_mutating_playback(attached, preview_stage, kwargs):
    with ToolsSession(attached) as tools:
        before = tools.identity
        with pytest.raises(ValueError):
            PreviewSession(attached, preview_stage, **(dict(read_ordinal=1) | kwargs))
        assert tools.identity == before
        attached.step(0.01)
        attached.wait_all()
        assert tools.scene_statistics('/World/PhysicsScene').dynamic_bodies == 2


@pytest.mark.parametrize('playback_first', [False, True])
@pytest.mark.parametrize('recording', [False, True])
def test_last_core_owner_and_recording_in_fresh_process(tmp_path, playback_first, recording):
    import os
    import subprocess
    import sys
    companion = Path(__file__).resolve().parents[1]
    command = [sys.executable, str(companion / 'samples/inspector_preview.py')]
    if playback_first:
        command.append('--playback-first')
    output = tmp_path / 'inspector.ovd'
    if recording:
        command += ['--recording', str(output)]
    env = os.environ.copy()
    env['PYTHONPATH'] = str(companion / 'python')
    result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=45)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'isolation passed' in result.stdout
    if recording:
        assert output.stat().st_size > 0


def test_hard_cpu_policy_rejects_explicit_gpu_preview(attached, preview_stage):
    with pytest.raises(CompanionError) as error:
        PreviewSession(attached, preview_stage, read_ordinal=1, gpu_pipeline=1)
    assert error.value.status == Status.UNAVAILABLE
    with PreviewSession(attached, preview_stage, read_ordinal=1, gpu_pipeline=0) as preview:
        preview.step(0.01)
    attached.step(0.01)
    attached.wait_all()


def test_independent_preview_threads(physics, preview_stage):
    import ovstage
    from concurrent.futures import ThreadPoolExecutor
    stage2 = ovstage.Stage('second-preview-thread')
    ovstage.population.open_usd(stage2, str(Path(__file__).resolve().parents[1] / 'samples/scenes/two_boxes.usda'),
                                ordinal=1, domains=ovstage.PopulationDomain.ALL)
    stage2.advance_write_floor(ordinal=1).wait()
    try:
        with PreviewSession(physics, preview_stage, read_ordinal=1, threads=2) as first, \
                PreviewSession(physics, stage2, read_ordinal=1, threads=3) as second:
            from threading import Barrier
            pending = Barrier(2)
            def run(preview, dt):
                for _ in range(4):
                    preview.simulate(dt)
                    pending.wait(timeout=5)
                    preview.fetch()
                    pending.wait(timeout=5)
                return preview.rigid_pose('/World/BoxA')
            with ThreadPoolExecutor(max_workers=2) as pool:
                a = pool.submit(run, first, .01)
                b = pool.submit(run, second, .02)
                assert b.result(timeout=10).position[2] < a.result(timeout=10).position[2]
    finally:
        stage2.destroy()

@pytest.mark.parametrize('retire_first', ['fetch', 'close'])
def test_recording_gate_tracks_every_pending_preview(physics, preview_stage, retire_first):
    import ovstage
    stage2 = ovstage.Stage('second-pending-preview')
    ovstage.population.open_usd(stage2, str(Path(__file__).resolve().parents[1] / 'samples/scenes/two_boxes.usda'),
                                ordinal=1, domains=ovstage.PopulationDomain.ALL)
    stage2.advance_write_floor(ordinal=1).wait()
    try:
        with PreviewSession(physics, preview_stage, read_ordinal=1) as first, \
                PreviewSession(physics, stage2, read_ordinal=1) as second:
            first.simulate(.01)
            second.simulate(.01)
            with pytest.raises(RuntimeError, match='safe point'):
                physics.is_recording()
            getattr(first, retire_first)()
            with pytest.raises(RuntimeError, match='safe point'):
                physics.is_recording()
            # A rejected recording transition did not fetch this owner's work.
            second.fetch()
            assert not physics.is_recording()
    finally:
        stage2.destroy()


@pytest.mark.parametrize('threads',[256,1024,2**32-1])
def test_preview_thread_limit_rejected_before_native_attach(physics,preview_stage,threads):
    from ovphysx_companion.configuration import AttachmentOptions
    with pytest.raises(ValueError):AttachmentOptions(threads=threads)
    with pytest.raises(ValueError):PreviewSession(physics,preview_stage,read_ordinal=1,threads=threads)


def test_parallel_preview_publication_and_queue_runtime_resolution(physics,preview_stage):
    import ovstage
    import time
    from concurrent.futures import ThreadPoolExecutor
    from threading import Barrier
    from ovphysx_companion.events import EventQueue,Mask
    from ovphysx_companion.native import CompanionError,Status
    peer=ovstage.Stage('publication-peer')
    ovstage.population.open_usd(peer,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),
        ordinal=1,domains=ovstage.PopulationDomain.ALL)
    peer.advance_write_floor(ordinal=1).wait()
    rendezvous=Barrier(2)
    def admitted(operation):
        deadline=time.monotonic()+10
        while True:
            try:return operation()
            except CompanionError as error:
                if error.status!=Status.BUSY or time.monotonic()>deadline:raise
                time.sleep(.001)
    def run(stage):
        for _ in range(20):
            rendezvous.wait(timeout=10)
            preview=admitted(lambda:PreviewSession(physics,stage,read_ordinal=1))
            queue=None
            try:
                queue=admitted(lambda:EventQueue(preview,mask=Mask.PRE_STEP|Mask.POST_STEP))
                admitted(lambda:preview.simulate(.001))
                admitted(preview.fetch)
                assert len(admitted(queue.poll).events)==2
            finally:
                if queue is not None:admitted(queue.close)
                # Peer attachment configuration can briefly reject retirement.
                # Busy retains the owner; retry its individual operation.
                admitted(preview.close)
    try:
        with ThreadPoolExecutor(2) as workers:
            first=workers.submit(run,preview_stage);second=workers.submit(run,peer)
            first.result(45);second.result(45)
    finally:peer.destroy()


def test_concurrent_preview_open_reports_busy_for_configuration_admission(attached,preview_stage,tmp_path):
    import logging
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event
    import ovphysx
    import ovstage
    peer=ovstage.Stage('contention-peer')
    entered,release=Event(),Event()
    logger=logging.getLogger('companion.preview.contention');logger.setLevel(logging.DEBUG)
    class PauseAttach(logging.Handler):
        def emit(self,record):
            if '/World/AdmissionProbe' in record.getMessage() and 'invalid inertia tensor' in record.getMessage() and not entered.is_set():
                entered.set();release.wait(10)
    handler=PauseAttach();logger.addHandler(handler)
    try:
        source=Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'
        candidate_scene=tmp_path/'admission.usda'
        # A valid shapeless body emits a deterministic mass-fallback warning
        # during attach. Do not rely on optional USD-backing lookup diagnostics.
        candidate_scene.write_text(source.read_text().replace('    def PhysicsScene',
            '    def Xform "AdmissionProbe" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {}\n    def PhysicsScene',1))
        ovstage.population.open_usd(peer,str(source),ordinal=1,domains=ovstage.PopulationDomain.ALL)
        ovstage.population.open_usd(preview_stage,str(candidate_scene),ordinal=2,domains=ovstage.PopulationDomain.ALL)
        preview_stage.advance_write_floor(ordinal=2).wait()
        peer.advance_write_floor(ordinal=1).wait()
        ovphysx.enable_python_logging(logger_name=logger.name)
        def candidate():
            with PreviewSession(attached,preview_stage,read_ordinal=2):pass
        with ThreadPoolExecutor(1) as pool:
            future=pool.submit(candidate)
            try:
                assert entered.wait(5)
                with pytest.raises(CompanionError) as caught:PreviewSession(attached,peer,read_ordinal=1)
                assert caught.value.status==Status.BUSY
            finally:release.set()
            future.result(timeout=10)
        with PreviewSession(attached,peer,read_ordinal=1) as owner:owner.step(.001)
    finally:
        release.set();ovphysx.disable_python_logging();logger.removeHandler(handler);peer.destroy()
