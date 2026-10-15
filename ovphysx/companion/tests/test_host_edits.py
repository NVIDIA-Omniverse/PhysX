# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-HOST-001
# @covers AC-1 AC-3 AC-4
from concurrent.futures import CancelledError, ThreadPoolExecutor
import pytest
from ovphysx_companion.host import DeferredEdit, Availability
from ovphysx_companion.usd import StaleResultError


class CompletedJob:
    done=True
    def result(self):return 2.0
    def cancel(self):return False # native work has already completed


def test_host_cancelled_completed_job_cannot_late_author(scene):
    stage,session=scene
    before=stage.GetRootLayer().ExportToString()
    calls=[]
    command=DeferredEdit(session,session.source,CompletedJob(),lambda value:calls.append(value))
    assert command.cancel()
    with pytest.raises(CancelledError):command.deliver()
    assert not calls and stage.GetRootLayer().ExportToString()==before
    assert not command.cancel()


def test_host_edit_delivery_revision_thread_undo_and_notice_guard(scene):
    stage,session=scene
    from pxr import UsdGeom
    layer=stage.GetSessionLayer()
    before=layer.ExportToString()
    guards=[]
    from contextlib import contextmanager
    @contextmanager
    def guard():
        guards.append('enter')
        yield
        guards.append('exit')
    session._output_guard=guard
    def apply(value):
        # A real helper would convert its input units before this atomic batch.
        return session._apply(layer,lambda:UsdGeom.SetStageMetersPerUnit(stage,value),output=True)
    command=DeferredEdit(session,session.source,CompletedJob(),apply)
    with ThreadPoolExecutor(1) as pool:
        with pytest.raises(RuntimeError,match='approved thread'):pool.submit(command.deliver).result()
    patch=command.deliver()
    assert command.deliver() is patch and guards==['enter','exit']
    assert UsdGeom.GetStageMetersPerUnit(stage)==2
    patch.undo();assert layer.ExportToString()==before
    patch.redo();assert UsdGeom.GetStageMetersPerUnit(stage)==2
    assert len(guards)==6


def test_host_stale_job_failure_is_terminal_and_diagnostic(scene):
    stage,session=scene
    calls=[]
    command=DeferredEdit(session,session.source,CompletedJob(),lambda value:calls.append(value))
    stage.DefinePrim('/Changed')
    for _ in range(2):
        with pytest.raises(StaleResultError):command.deliver()
    assert command.done and not calls
    with pytest.raises(ValueError,match='specific diagnostic'):Availability('viewer',False)
    Availability('ready',True).require()


def test_batch_colliders_preflight_cancel_units_and_atomic_undo():
    from pxr import Usd,UsdGeom,UsdPhysics
    from ovphysx_companion.usd import EditSession
    from ovphysx_companion.usd.colliders import ColliderBatch
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,.01)
    for p in ('/A','/B'):UsdGeom.Cube.Define(stage,p)
    with EditSession(stage) as session:
        layer=stage.GetSessionLayer();before=layer.ExportToString()
        with pytest.raises(ValueError,match='active loaded'):
            ColliderBatch(session,['/A','/Absent'],layer=layer)
        assert layer.ExportToString()==before
        batch=ColliderBatch(session,['/A','/B'],layer=layer,contact_offset_meters=.02)
        assert batch.advance(1).fraction==.5
        assert batch.world_bounds_meters[0][1]==pytest.approx((-.01,-.01,-.01))
        with pytest.raises(RuntimeError,match='Finish preparing'):batch.commit()
        assert layer.ExportToString()==before
        batch.cancel()
        with pytest.raises(CancelledError):batch.commit()
        assert layer.ExportToString()==before
        batch=ColliderBatch(session,['/A','/B'],layer=layer,contact_offset_meters=.02)
        batch.advance();patch=batch.commit();assert batch.commit() is patch
        assert stage.GetPrimAtPath('/B').GetAttribute('physxCollision:contactOffset').Get()==pytest.approx(2)
        assert stage.GetPrimAtPath('/A').HasAPI(UsdPhysics.CollisionAPI)
        patch.undo();assert layer.ExportToString()==before
        patch.redo();assert stage.GetPrimAtPath('/B').HasAPI(UsdPhysics.CollisionAPI)


def test_batch_colliders_stale_thread_failure_rollback_and_sample():
    from pxr import Usd,UsdGeom
    from ovphysx_companion.usd import EditSession
    from ovphysx_companion.usd.colliders import ColliderBatch
    stage=Usd.Stage.CreateInMemory()
    for p in ('/A','/B'):UsdGeom.Cube.Define(stage,p)
    with EditSession(stage) as session:
        layer=stage.GetSessionLayer();before=layer.ExportToString()
        batch=ColliderBatch(session,['/A','/B'],layer=layer)
        with ThreadPoolExecutor(1) as pool:
            with pytest.raises(RuntimeError,match='approved thread'):pool.submit(batch.advance).result()
        stage.DefinePrim('/Changed')
        with pytest.raises(StaleResultError):batch.advance()
        batch=ColliderBatch(session,['/A','/B'],layer=layer)
        original=batch._set_collider
        def fail_second(prim,approximation):
            if str(prim.GetPath())=='/B':raise RuntimeError('Injected second-item failure')
            original(prim,approximation)
        batch._set_collider=fail_second
        batch.advance()
        for _ in range(2):
            with pytest.raises(RuntimeError,match='second-item failure'):batch.commit()
        assert layer.ExportToString()==before
    from batch_colliders import main
    main()
