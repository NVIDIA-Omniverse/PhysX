# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PYTHON-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-USD-EDIT-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-IDENTITY-001
# @covers AC-3 AC-4
# @implements REQ-COMP-ATTACH-USD-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-DEFORM-WRITE-001
# @covers AC-1 AC-2 AC-3 AC-4

from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from ovphysx_companion.usd import (
    EditSession, MeshFrame, StaleResultError, VertexAttachment,
    apply_vertex_attachment, capture_mesh, write_simulation_output,
)


def _frame(session, path="/World/A", **kwargs):
    snapshot = capture_mesh(session, path)
    return MeshFrame(snapshot, np.array(snapshot.points, dtype=np.float32) + (0, 0, 1), 1, 0.1, **kwargs)


def _attachment(session):
    return VertexAttachment(capture_mesh(session, "/World/A"), capture_mesh(session, "/World/B"), (0, 1), (1, 2))


def test_lazy_import_and_missing_customer_usd():
    script = '''
import sys
class Block:
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in {'pxr', 'omni', 'ovphysx', 'ovstage', 'numpy'}:
            raise ImportError('blocked optional dependency: ' + fullname)
sys.meta_path.insert(0, Block())
import ovphysx_companion
assert 'ovphysx_companion.usd' not in sys.modules
from ovphysx_companion.native import ToolsSession, compute_mass_properties
from ovphysx_companion.jobs import AuthoringService
assert 'ovphysx_companion.usd' not in sys.modules
from ovphysx_companion.usd import EditSession, DependencyError
assert 'pxr' not in sys.modules
try:
    EditSession(None)
except DependencyError as e:
    assert 'customer-installed pxr' in str(e)
else:
    raise AssertionError('missing pxr accepted')
'''
    env = os.environ.copy()
    env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1] / "python")
    subprocess.run([sys.executable, "-c", script], env=env, check=True)


def test_unregistered_attachment_schema_fails_before_mutation():
    script = '''
from pxr import Usd
from ovphysx_companion.usd import EditSession, VertexAttachment, apply_vertex_attachment, DependencyError
s = Usd.Stage.CreateInMemory()
before = s.GetRootLayer().ExportToString()
with EditSession(s) as session:
    try:
        apply_vertex_attachment(session, VertexAttachment(None, None, (), ()),
                                path='/Missing', layer=s.GetRootLayer(), owner='test')
    except DependencyError as e:
        assert 'Register compatible codeless' in str(e)
    else:
        raise AssertionError('unregistered schema accepted')
assert s.GetRootLayer().ExportToString() == before
'''
    env = {k: v for k, v in os.environ.items() if "PXR_PLUGINPATH" not in k}
    env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1] / "python")
    subprocess.run([sys.executable, "-c", script], env=env, check=True)


def test_capture_contains_only_source_data(scene):
    from pxr import UsdGeom
    stage, session = scene
    UsdGeom.SetStageMetersPerUnit(stage, 0.001)
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/World/A"))
    mesh.AddTranslateOp().Set((2, 3, 4))
    record = capture_mesh(session, "/World/A")
    assert record.meters_per_unit == 0.001
    assert record.local_to_world[3] == (2, 3, 4, 1)
    assert record.points == ((0, 0, 0), (1, 0, 0), (0, 1, 0))


def test_attachment_apply_update_and_undo_preserve_unowned_content(scene):
    from pxr import Sdf
    stage, session = scene
    layer = stage.GetRootLayer()
    before = layer.ExportToString()
    patch = apply_vertex_attachment(session, _attachment(session), path="/World/Seam", layer=layer, owner="tool")
    seam = stage.GetPrimAtPath("/World/Seam")
    assert seam.GetTypeName() == "OmniPhysicsVtxVtxAttachment"
    assert seam.GetRelationship("omniphysics:src1").GetTargets() == [Sdf.Path("/World/B")]
    assert list(seam.GetAttribute("omniphysics:vtxIndicesSrc1").Get()) == [1, 2]
    patch.undo()
    assert layer.ExportToString() == before
    patch.redo()
    seam = stage.GetPrimAtPath("/World/Seam")
    seam.CreateAttribute("user:note", Sdf.ValueTypeNames.String).Set("keep")
    stage.DefinePrim("/World/Seam/UserChild", "Scope")
    authored = layer.ExportToString()
    apply_vertex_attachment(session, _attachment(session), path="/World/Seam", layer=layer, owner="tool")
    assert layer.ExportToString() == authored
    assert stage.GetPrimAtPath("/World/Seam/UserChild")


@pytest.mark.parametrize("failure", ["foreign", "deleted", "topology", "cancel", "recreated", "closed"])
def test_stale_results_fail_without_edits(scene, failure):
    stage, session = scene
    result = _attachment(session)
    if failure == "foreign":
        session = EditSession(stage)
    elif failure == "deleted":
        stage.RemovePrim("/World/A")
    elif failure == "topology":
        stage.GetPrimAtPath("/World/A").GetAttribute("faceVertexIndices").Set([2, 1, 0])
    elif failure == "recreated":
        stage.RemovePrim("/World/A")
        stage.DefinePrim("/World/A", "Mesh")
    elif failure == "closed":
        session.close()
    else:
        session.invalidate()
    before = stage.GetRootLayer().ExportToString()
    try:
        with pytest.raises((StaleResultError, RuntimeError)):
            apply_vertex_attachment(session, result, path="/World/Seam", layer=stage.GetRootLayer(), owner="tool")
        assert stage.GetRootLayer().ExportToString() == before
    finally:
        if failure == "foreign":
            session.close()


def test_attachment_preflight_and_ownership(scene):
    stage, session = scene
    layer = stage.GetRootLayer()
    stage.DefinePrim("/World/Seam", "Scope")
    result = _attachment(session)
    before = layer.ExportToString()
    with pytest.raises(ValueError, match="owned"):
        apply_vertex_attachment(session, result, path="/World/Seam", layer=layer, owner="tool")
    bad = VertexAttachment(result.source0, result.source1, (999,), (0,))
    with pytest.raises(ValueError, match="indices"):
        apply_vertex_attachment(session, bad, path="/World/New", layer=layer, owner="tool")
    assert layer.ExportToString() == before


def test_batch_output_preserves_rest_topology_and_explicit_time(scene):
    from pxr import Sdf, Usd
    stage, session = scene
    rest = stage.GetRootLayer().ExportToString()
    layer = stage.GetSessionLayer()
    frames = [_frame(session, p, velocities=np.zeros((3, 3), np.float32)) for p in ("/World/A", "/World/B")]
    patch = write_simulation_output(session, frames, layer=layer, time_code=12.5)
    assert stage.GetRootLayer().ExportToString() == rest
    for frame in frames:
        prim = stage.GetPrimAtPath(frame.destination.path)
        assert np.allclose(prim.GetAttribute("points").Get(Usd.TimeCode(12.5)), frame.positions)
        assert layer.ListTimeSamplesForPath(Sdf.Path(frame.destination.path + ".points")) == [12.5]
        assert not layer.GetAttributeAtPath(frame.destination.path + ".faceVertexIndices")
    patch.undo()
    assert layer.empty
    patch.redo()
    with pytest.raises(StaleResultError):
        write_simulation_output(session, frames, layer=layer, time_code=13)


@pytest.mark.parametrize("bad", ["shape", "nan", "gpu", "frame", "duplicate", "time", "mixed"])
def test_output_preflight_is_whole_batch(scene, bad):
    from dataclasses import replace
    stage, session = scene
    first, last = _frame(session), _frame(session, "/World/B")
    time = None
    if bad == "shape":
        last = replace(last, positions=np.zeros((2, 3), np.float32))
    elif bad == "nan":
        last = replace(last, positions=np.full((3, 3), np.nan))
    elif bad == "gpu":
        class DeviceOnly:
            def __array__(self):
                raise AssertionError("implicit GPU copy attempted")
        last = replace(last, positions=DeviceOnly())
    elif bad == "frame":
        last = replace(last, frame="world")
    elif bad == "duplicate":
        last = first
    elif bad == "mixed":
        last = replace(last, step=2)
    else:
        time = float("inf")
    layer = stage.GetSessionLayer()
    before = layer.ExportToString()
    with pytest.raises((ValueError, TypeError)):
        write_simulation_output(session, [first, last], layer=layer, time_code=time)
    assert layer.ExportToString() == before


def test_failed_application_rolls_back_and_restores_edit_target(scene, monkeypatch):
    from pxr import Usd
    stage, session = scene
    layer = stage.GetSessionLayer()
    before = layer.ExportToString()
    frames = [_frame(session, p) for p in ("/World/A", "/World/B")]
    original = Usd.Attribute.Set

    def fail_second(attr, *args, **kwargs):
        if str(attr.GetPath()) == "/World/B.points":
            raise RuntimeError("injected authoring failure")
        return original(attr, *args, **kwargs)

    monkeypatch.setattr(Usd.Attribute, "Set", fail_second)
    with pytest.raises(RuntimeError, match="injected"):
        write_simulation_output(session, frames, layer=layer, time_code=None)
    assert layer.ExportToString() == before
    assert stage.GetEditTarget().GetLayer() == stage.GetRootLayer()


def test_thread_layer_and_undo_conflict_checks(scene):
    from pxr import Sdf
    stage, session = scene
    frame = _frame(session)
    with ThreadPoolExecutor(max_workers=1) as executor:
        task = executor.submit(write_simulation_output, session, [frame], layer=stage.GetSessionLayer(), time_code=None)
        with pytest.raises(RuntimeError, match="approved thread"):
            task.result()
    with pytest.raises(ValueError, match="layer stack"):
        write_simulation_output(session, [frame], layer=Sdf.Layer.CreateAnonymous(), time_code=None)
    layer = stage.GetSessionLayer()
    layer.SetPermissionToEdit(False)
    try:
        with pytest.raises(ValueError, match="writable"):
            write_simulation_output(session, [frame], layer=layer, time_code=None)
    finally:
        layer.SetPermissionToEdit(True)
    patch = write_simulation_output(session, [frame], layer=layer, time_code=None)
    layer.comment = "another user's edit"
    before = layer.ExportToString()
    with pytest.raises(StaleResultError, match="unrelated"):
        patch.undo()
    assert layer.ExportToString() == before


def test_stage_time_converts_to_local_layer_time(scene):
    from pxr import Sdf
    stage, session = scene
    stage.SetTimeCodesPerSecond(60)
    layer = Sdf.Layer.CreateAnonymous("output.usda")
    layer.timeCodesPerSecond = 24
    stage.GetSessionLayer().subLayerPaths.append(layer.identifier)
    frame = _frame(session)
    write_simulation_output(session, [frame], layer=layer, time_code=5)
    assert layer.ListTimeSamplesForPath(Sdf.Path("/World/A.points")) == [2.0]
    assert np.allclose(stage.GetPrimAtPath("/World/A").GetAttribute("points").Get(5), frame.positions)


def test_output_rejects_changed_topology_and_instances(scene):
    stage, session = scene
    frame = _frame(session)
    stage.GetPrimAtPath("/World/A").GetAttribute("faceVertexIndices").Set([2, 0, 1])
    before = stage.GetSessionLayer().ExportToString()
    with pytest.raises(StaleResultError):
        write_simulation_output(session, [frame], layer=stage.GetSessionLayer(), time_code=None)
    assert stage.GetSessionLayer().ExportToString() == before
    instance = stage.DefinePrim("/Instance")
    instance.GetReferences().AddInternalReference("/World")
    instance.SetInstanceable(True)
    with pytest.raises(ValueError, match="Instance"):
        capture_mesh(session, "/Instance/A")


def test_guard_failure_returns_recoverable_layer(scene):
    from ovphysx_companion.usd import RecoveryError
    stage, _ = scene
    layer = stage.GetSessionLayer()
    before = layer.ExportToString()

    @contextmanager
    def failed_guard():
        yield
        raise RuntimeError("host guard failed")

    with EditSession(stage, output_guard=failed_guard) as session:
        with pytest.raises(RecoveryError) as failure:
            write_simulation_output(session, [_frame(session)], layer=layer, time_code=None)
        assert failure.value.layer == layer
        assert failure.value.before_text == before
        # Host decides when/how to restore after repairing its notice policy.
        assert layer.ImportFromString(failure.value.before_text)
        assert layer.ExportToString() == before


def test_false_usd_set_rolls_back(scene, monkeypatch):
    from pxr import Usd
    stage, session = scene
    frames = [_frame(session, p) for p in ("/World/A", "/World/B")]
    original = Usd.Attribute.Set

    def reject_second(attr, *args, **kwargs):
        return False if str(attr.GetPath()) == "/World/B.points" else original(attr, *args, **kwargs)

    monkeypatch.setattr(Usd.Attribute, "Set", reject_second)
    with pytest.raises(RuntimeError, match="rejected"):
        write_simulation_output(session, frames, layer=stage.GetSessionLayer(), time_code=None)
    assert stage.GetSessionLayer().empty


def test_output_notice_guard_and_reentrant_edits(scene):
    from pxr import Tf, Usd
    stage, _ = scene
    publishing, input_notices = [], []

    def notice(changes, sender):
        if not publishing:
            input_notices.append(changes)

    @contextmanager
    def guard():
        publishing.append(True)
        try:
            with pytest.raises(RuntimeError, match="Reentrant"):
                session.invalidate()
            yield
        finally:
            publishing.pop()

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, notice, stage)
    try:
        with EditSession(stage, output_guard=guard) as session:
            patch = write_simulation_output(session, [_frame(session)], layer=stage.GetSessionLayer(), time_code=None)
            patch.undo()
            patch.redo()
            assert not input_notices
    finally:
        listener.Revoke()


def test_extension_samples_run():
    import attachment_authoring
    import deformable_writeback
    stage = attachment_authoring.run()
    assert stage.GetPrimAtPath("/World/Seam")
    _, bake = deformable_writeback.run()
    assert not bake.empty


@pytest.mark.parametrize("sample", ["attachment_authoring", "deformable_writeback"])
def test_extension_sample_cli(sample):
    companion = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env["PYTHONPATH"] = str(companion / "python")
    script = """
import runpy, sys
runpy.run_path(sys.argv[1], run_name='__main__')
assert 'ovphysx._bindings' not in sys.modules
assert 'ovstage' not in sys.modules
"""
    result = subprocess.run([sys.executable, "-c", script, str(companion / "samples" / (sample + ".py"))],
                            env=env, cwd=companion, text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.startswith("#usda")


def test_attachment_matching_frames_units_and_invalid_results(scene):
    from dataclasses import replace
    from ovphysx_companion.usd import match_vertex_attachment
    stage, session = scene
    a, b = capture_mesh(session, '/World/A'), capture_mesh(session, '/World/B')
    shifted = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (10, 0, 0, 1))
    a = replace(a, local_to_world=shifted, meters_per_unit=0.1)
    calls = []
    def matcher(source, target, distance):
        calls.append((source, target, distance))
        return (2, -1, 0)
    result = match_vertex_attachment(a, b, distance=0.2, matcher=matcher)
    assert calls[0][0][0] == pytest.approx((1, 0, 0))
    assert calls[0][0][1] == pytest.approx((1.1, 0, 0))
    assert result.indices0 == (0, 2)
    assert result.indices1 == (2, 0)
    for indices in [(0,), (0, 1, 20), (-2, 0, 1)]:
        with pytest.raises(ValueError, match='invalid correspondence'):
            match_vertex_attachment(a, b, distance=1, matcher=lambda *_: indices)
    with pytest.raises(ValueError, match='No attachment points'):
        match_vertex_attachment(a, b, distance=0, matcher=lambda *_: (-1, -1, -1))
    session.invalidate()
    with pytest.raises(StaleResultError):
        apply_vertex_attachment(session, result, path='/World/Seam', layer=stage.GetRootLayer(), owner='test')
