# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""A write step captured into a CUDA graph through the public Python API, with Warp.

Capture needs the caller's stream in the scene's CUDA context. The suite's shared GPU instance runs physics in
its own context, which Warp's streams do not share, so it covers the refusals; the replay cases run in a fresh
process whose instance is created on CUDA device 0, the primary context Warp uses.
"""

# @implements REQ-CAPI-WRITE-001
# @covers AC-17
# @maps_to TEST-CAPI-WRITE-003

# @implements REQ-CAPI-OVSTAGE-UPDATE-001
# @covers AC-6
# @maps_to TEST-CAPI-OVSTAGE-UPDATE-004

import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import ovstage
import pytest
import warp as wp
from ovphysx import PhysX, PhysXConfig
from ovphysx.api import TaskError
from ovphysx.types import SimObjectType
from session_helpers import _group_paths, read_column
from test_utils import attach_usd_with_ovstage, data_path

_HERE = Path(__file__).resolve().parent
DT = 1.0 / 60.0


def _flat(group, device):
    """One float array over a group's column, which its tensors window."""
    base = group.tensors[0].ptr
    count = 0
    for tensor in group.tensors:
        assert tensor.ptr == base + count * 4, "the group's tensors do not window one column"
        count += tensor.size
    return wp.array(ptr=base, dtype=float, shape=(count,), device=device)


def _paths(stage, group):
    with ovstage.PathDictionary(stage) as dictionary:
        return _group_paths(dictionary, group)


def _open_write(physx, query, attribute):
    session = physx.write(query, [attribute]).wait()
    groups = []
    while (group := session.fetch_next().wait()) is not None:
        groups.append(group)
    return session, groups


def _attach_and_step(physx, scene):
    stage = attach_usd_with_ovstage(physx, data_path(scene))
    physx.wait_all()
    physx.warmup()
    physx.step(DT)
    physx.wait_all()
    return stage


def _gpu_instancer_scene(directory):
    """ovstage_output_instancer.usda on a DirectGPU scene, with a translate op on the instancer."""
    text = Path(data_path("ovstage_output_instancer.usda")).read_text()
    for old, new in (
        ("bool physxScene:enableGPUDynamics = false", "bool physxScene:enableGPUDynamics = true"),
        ('uniform token physxScene:broadphaseType = "MBP"', 'uniform token physxScene:broadphaseType = "GPU"'),
        ('def PointInstancer "Instances"\n    {',
         'def PointInstancer "Instances"\n    {\n        double3 xformOp:translate = (0, 0, 0)\n'
         '        uniform token[] xformOpOrder = ["xformOp:translate"]'),
    ):
        assert old in text, old
        text = text.replace(old, new)
    path = directory / "instancer_gpu.usda"
    path.write_text(text)
    return path


def _move_prim(physx, stage, path, x, ordinal):
    """Translate `path` to x through a stage update, which leaves every physics object in place."""
    matrix = np.asarray([[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, 0, 0, 1]], dtype=np.float64)
    with ovstage.PathDictionary(stage) as dictionary:
        query = stage.query_from_path_list(dictionary.create_path_list_from_strings([path]))
        stage.write_attribute(query, "omni:xform", ordinal, matrix, is_array=False,
                              prim_mode=ovstage.PrimMode.UPSERT).wait()
        query.release()
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.wait_task(physx.update_from_ovstage(ordinal, ordinal))


def _replay_scenarios():
    """Runs in a fresh process. Each graph is replayed with new inputs and checked through a read."""
    physx = PhysX(config=PhysXConfig(carbonite_overrides={"/physics/suppressReadback": True}), active_cuda_gpus="0")
    device = wp.get_device("cuda:0")
    stream = wp.get_stream(device)
    cuda_stream = int(stream.cuda_stream or 1)

    # A captured fill and commit write what the source holds when each replay runs.
    stage = _attach_and_step(physx, "boxes_falling_on_groundplane_gpu.usda")
    bodies = physx.query(SimObjectType.RIGID_BODY).wait()
    session, (group,) = _open_write(physx, bodies, "position")
    paths = _paths(stage, group)
    column = _flat(group, device)
    source = wp.zeros(column.shape[0], dtype=float, device=device)
    before = read_column(physx, stage, SimObjectType.RIGID_BODY, "position")
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        wp.copy(column, source, stream=stream)
        session.commit(group, cuda_stream=cuda_stream).wait()
    after = read_column(physx, stage, SimObjectType.RIGID_BODY, "position")
    assert all(np.allclose(after[p], before[p]) for p in paths), "capturing the commit ran it"
    for offset in (10.0, -7.0):
        values = np.concatenate([[offset + i, 2.0, 1.0 + i] for i in range(len(paths))]).astype(np.float32)
        source.assign(values)
        wp.capture_launch(capture.graph, stream=stream)
        wp.synchronize_stream(stream)
        after = read_column(physx, stage, SimObjectType.RIGID_BODY, "position")
        for i, path in enumerate(paths):
            assert np.allclose(after[path], values[3 * i:3 * i + 3], atol=1e-4), (offset, path, after[path])
    physx.step(DT)
    physx.wait_all()
    session.close().wait()

    # A captured masked commit reads the mask's contents when each replay runs.
    session, (group,) = _open_write(physx, bodies, "position")
    paths = _paths(stage, group)
    column = _flat(group, device)
    source = wp.zeros(column.shape[0], dtype=float, device=device)
    mask = wp.zeros(group.prim_count, dtype=wp.uint8, device=device)
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        wp.copy(column, source, stream=stream)
        session.commit(group, cuda_stream=cuda_stream, mask=mask).wait()
    for parity in (0, 1):
        selected = [i % 2 == parity for i in range(len(paths))]
        values = np.concatenate([[50.0 + i, 3.0, 2.0] for i in range(len(paths))]).astype(np.float32)
        before = read_column(physx, stage, SimObjectType.RIGID_BODY, "position")
        source.assign(values)
        mask.assign(np.asarray(selected, dtype=np.uint8))
        wp.capture_launch(capture.graph, stream=stream)
        wp.synchronize_stream(stream)
        after = read_column(physx, stage, SimObjectType.RIGID_BODY, "position")
        for i, path in enumerate(paths):
            expected = values[3 * i:3 * i + 3] if selected[i] else before[path]
            assert np.allclose(after[path], expected, atol=1e-4), (parity, path, after[path])
    session.close().wait()
    bodies.close().wait()

    # A captured joint write and kinematic refresh over a held query move the links on replay, with no step.
    stage = _attach_and_step(physx, "two_articulations.usda")
    joints = physx.query(SimObjectType.ARTICULATION_JOINT).wait()
    articulations = physx.query(SimObjectType.ARTICULATION).wait()
    session, groups = _open_write(physx, joints, "jointPosition")
    rows = [path for group in groups for path in _paths(stage, group)]
    columns = [_flat(group, device) for group in groups]
    source = wp.zeros(sum(c.shape[0] for c in columns), dtype=float, device=device)
    links_before = read_column(physx, stage, SimObjectType.ARTICULATION_LINK, "position")
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        offset = 0
        for group, column in zip(groups, columns):
            wp.copy(column, source, src_offset=offset, count=column.shape[0], stream=stream)
            offset += column.shape[0]
            session.commit(group, cuda_stream=cuda_stream).wait()
        physx.update_articulations_kinematic(query=articulations, cuda_stream=cuda_stream)
    # Five degrees, inside every joint's limits.
    source.fill_(5.0)
    wp.capture_launch(capture.graph, stream=stream)
    wp.synchronize_stream(stream)
    positions = read_column(physx, stage, SimObjectType.ARTICULATION_JOINT, "jointPosition")
    assert all(abs(positions[path][0] - 5.0) < 1e-2 for path in rows), positions
    links_after = read_column(physx, stage, SimObjectType.ARTICULATION_LINK, "position")
    moved = [p for p in links_after if np.linalg.norm(links_after[p] - links_before[p]) > 1e-3]
    assert len(moved) == 4, f"expected the two outer links of each articulation to move, moved: {moved}"
    # Each replay writes what the source holds when it runs, step after step.
    for degrees in (-4.0, 2.5):
        physx.step(DT)
        physx.wait_all()
        links_before = read_column(physx, stage, SimObjectType.ARTICULATION_LINK, "position")
        source.fill_(degrees)
        wp.capture_launch(capture.graph, stream=stream)
        wp.synchronize_stream(stream)
        positions = read_column(physx, stage, SimObjectType.ARTICULATION_JOINT, "jointPosition")
        assert all(abs(positions[path][0] - degrees) < 1e-2 for path in rows), (degrees, positions)
        links_after = read_column(physx, stage, SimObjectType.ARTICULATION_LINK, "position")
        moved = [p for p in links_after if np.linalg.norm(links_after[p] - links_before[p]) > 1e-3]
        assert len(moved) == 4, (degrees, moved)
    session.close().wait()
    joints.close().wait()
    articulations.close().wait()

    # A captured instancer write keeps the instancer frame it was opened with: after the instancer moves, a replay
    # still writes in the old frame, and only a write opened afterwards uses the new one.
    with tempfile.TemporaryDirectory() as directory:
        stage = attach_usd_with_ovstage(physx, str(_gpu_instancer_scene(Path(directory))))
        physx.wait_all()
        physx.warmup()
        physx.step(DT)
        physx.wait_all()
        instancers = physx.query(SimObjectType.RIGID_BODY, pattern="/World/Instances").wait()
        # A DirectGPU instancer is written through the view its first read builds.
        read_column(physx, stage, SimObjectType.RIGID_BODY, "position", pattern="/World/Instances")
        session, (group,) = _open_write(physx, instancers, "position")
        local = np.asarray([1.0, 0.0, 20.0, 6.0, 0.0, 20.0], dtype=np.float32)
        source = wp.array(local, dtype=float, device=device)
        with wp.ScopedCapture(device=device, stream=stream) as capture:
            wp.copy(_flat(group, device), source, stream=stream)
            session.commit(group, cuda_stream=cuda_stream).wait()
        wp.capture_launch(capture.graph, stream=stream)
        wp.synchronize_stream(stream)
        written = read_column(physx, stage, SimObjectType.RIGID_BODY, "position", pattern="/World/Instances")
        assert np.allclose(written["/World/Instances"], local, atol=1e-4), written
        _move_prim(physx, stage, "/World/Instances", 20.0, ordinal=2)
        wp.capture_launch(capture.graph, stream=stream)
        wp.synchronize_stream(stream)
        replayed = read_column(physx, stage, SimObjectType.RIGID_BODY, "position", pattern="/World/Instances")
        shifted = local - np.asarray([20.0, 0.0, 0.0, 20.0, 0.0, 0.0], dtype=np.float32)
        assert np.allclose(replayed["/World/Instances"], shifted, atol=1e-4), replayed
        session.close().wait()
        session, (group,) = _open_write(physx, instancers, "position")
        wp.copy(_flat(group, device), source, stream=stream)
        session.commit(group, cuda_stream=cuda_stream).wait()
        wp.synchronize_stream(stream)
        fresh = read_column(physx, stage, SimObjectType.RIGID_BODY, "position", pattern="/World/Instances")
        assert np.allclose(fresh["/World/Instances"], local, atol=1e-4), fresh
        session.close().wait()
        instancers.close().wait()


def _run_in_fresh_process(scenario):
    """Runs this module's function `scenario` in a fresh process, which can create an instance on an explicit CUDA
    device."""
    command = (
        "import sys; sys.path[:0] = [sys.argv[1], sys.argv[2]]; "
        f"import test_write_capture_gpu as t; t.{scenario}()"
    )
    result = subprocess.run(
        [sys.executable, "-c", command, str(_HERE), str(_HERE.parent / "python_samples")],
        cwd=_HERE,
        text=True,
        capture_output=True,
        timeout=600,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_write_step_replays_from_a_cuda_graph():
    """Fill, commit, masked commit and kinematic refresh captured on Warp's stream replay with new inputs, step after
    step; a replayed instancer write keeps the frame it was captured in."""
    _run_in_fresh_process("_replay_scenarios")


def _reused_column_scenario():
    """Runs in a fresh process, on the CUDA context Warp uses, so each commit is ordered on the producer's stream
    instead of waited for on the host."""
    physx = PhysX(config=PhysXConfig(carbonite_overrides={"/physics/suppressReadback": True}), active_cuda_gpus="0")
    stage = _attach_and_step(physx, "boxes_falling_on_groundplane_gpu.usda")
    device = wp.get_device("cuda:0")
    bodies = physx.query(SimObjectType.RIGID_BODY).wait()
    # Stands in for another framework's stream, reached through its handle.
    producer = wp.Stream(device)
    column_ptrs = []
    for offset in (10.0, 20.0):
        session, (group,) = _open_write(physx, bodies, "position")
        column = _flat(group, device)
        column_ptrs.append(column.ptr)
        paths = _paths(stage, group)
        values = np.concatenate([[offset + i, 2.0, 1.0 + i] for i in range(len(paths))]).astype(np.float32)
        source = wp.array(values, dtype=float, device=device)
        wp.Stream(device, cuda_stream=producer.cuda_stream).wait_stream(wp.get_stream(device))
        wp.copy(column, source, stream=producer)
        session.commit(group, cuda_stream=int(producer.cuda_stream)).wait()
        wp.synchronize_stream(producer)
        after = read_column(physx, stage, SimObjectType.RIGID_BODY, "position")
        for i, path in enumerate(paths):
            assert np.allclose(after[path], values[3 * i:3 * i + 3], atol=1e-4), (offset, path, after[path])
        session.close().wait()
    assert column_ptrs[0] == column_ptrs[1], "the second write was expected to reuse the column"
    bodies.close().wait()


def test_reused_column_filled_on_another_stream():
    """A reused column filled on another stream than Warp's, made to wait for Warp's stream first as the guide says,
    reads back each write's values when the instance shares Warp's CUDA context."""
    _run_in_fresh_process("_reused_column_scenario")


def test_capture_on_another_cuda_context_is_refused(physx_sdk):
    """A commit on a capturing stream of another context is refused before anything is queued on it."""
    _attach_and_step(physx_sdk, "boxes_falling_on_groundplane_gpu.usda")
    device = wp.get_device("cuda:0")
    stream = wp.get_stream(device)
    bodies = physx_sdk.query(SimObjectType.RIGID_BODY).wait()
    session, (group,) = _open_write(physx_sdk, bodies, "position")
    column = _flat(group, device)
    source = wp.zeros(column.shape[0], dtype=float, device=device)
    with wp.ScopedCapture(device=device, stream=stream):
        wp.copy(column, source, stream=stream)
        with pytest.raises(TaskError, match="another CUDA context"):
            session.commit(group, cuda_stream=int(stream.cuda_stream or 1)).wait()
    # The refused group is spent, and nothing proves the captured fill never runs: release keeps its column.
    with pytest.raises(TaskError, match="quarantined"):
        session.close().wait()
    bodies.close().wait()


def test_host_group_refused_inside_a_capture_commits_after_it(physx_sdk):
    """A group applied on the host is refused on a capturing stream when committed, and commits after the capture."""
    stage = _attach_and_step(physx_sdk, "boxes_falling_on_groundplane_gpu.usda")
    device = wp.get_device("cuda:0")
    stream = wp.get_stream(device)
    bodies = physx_sdk.query(SimObjectType.RIGID_BODY).wait()
    session, groups = _open_write(physx_sdk, bodies, "mass")
    for group in groups:
        for tensor in group.tensors:
            assert not tensor.device.is_cuda, "mass is expected to be a host-applied group"
            tensor.assign(np.full(tensor.shape, 3.0, dtype=np.float32))
    marker = wp.zeros(1, dtype=float, device=device)
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        marker.fill_(1.0)
        for group in groups:
            with pytest.raises(RuntimeError, match="being captured"):
                session.commit(group, cuda_stream=int(stream.cuda_stream or 1))
    # The refusal left the capture going: its graph replays.
    wp.capture_launch(capture.graph, stream=stream)
    wp.synchronize_stream(stream)
    assert marker.numpy()[0] == 1.0
    for group in groups:
        session.commit_sync(group)
    masses = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "mass")
    paths = [path for group in groups for path in _paths(stage, group)]
    assert paths and all(np.allclose(masses[path], 3.0) for path in paths), masses
    session.close().wait()
    bodies.close().wait()


def test_update_from_ovstage_refused_inside_a_capture(physx_sdk):
    """update_from_ovstage inside a Warp capture is refused, because a graph replay would not repeat it. The capture
    goes on, and the same update is accepted after it."""
    _attach_and_step(physx_sdk, "boxes_falling_on_groundplane_gpu.usda")
    device = wp.get_device("cuda:0")
    stream = wp.get_stream(device)
    marker = wp.zeros(1, dtype=float, device=device)
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        marker.fill_(1.0)
        with pytest.raises(RuntimeError, match="CUDA graph capture"):
            physx_sdk.update_from_ovstage(1, 1)
    # The refusal left the capture going: its graph replays.
    wp.capture_launch(capture.graph, stream=stream)
    wp.synchronize_stream(stream)
    assert marker.numpy()[0] == 1.0
    physx_sdk.wait_task(physx_sdk.update_from_ovstage(1, 1))
