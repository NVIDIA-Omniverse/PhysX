# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""A read step captured into a CUDA graph through the public Python API, with Warp.

Capture needs the caller's stream in the scene's CUDA context. The suite's shared GPU instance runs physics in
its own context, which Warp's streams do not share, so it covers that refusal; the replay cases run in a fresh
process whose instance is created on CUDA device 0, the primary context Warp uses.
"""

# @implements REQ-PYTHON-READ-001
# @covers AC-8
# @maps_to TEST-PYTHON-READ-002

import subprocess
import sys
import weakref
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


@wp.kernel
def _shift_positions(source: wp.array(dtype=float), shift: wp.array(dtype=float), out: wp.array(dtype=float)):
    i = wp.tid()
    out[i] = source[i] + shift[i % 3]


def _flat(tensors, device):
    """One float array over the column a group's tensors window."""
    base = tensors[0].ptr
    count = 0
    for tensor in tensors:
        assert tensor.ptr == base + count * 4, "the group's tensors do not window one column"
        count += tensor.size
    return wp.array(ptr=base, dtype=float, shape=(count,), device=device)


def _paths(stage, group):
    with ovstage.PathDictionary(stage) as dictionary:
        return _group_paths(dictionary, group)


def _rows(stage, group):
    """{prim path -> flat row} of a group: a fixed group's column rows, a prim's rows together (a joint has one
    row per DOF), or an array group's per-prim tensors."""
    index_map = group.index_map.numpy() if group.index_map is not None else None
    column = None if group.is_array else group.tensors[0].numpy().reshape(group.tensors[0].shape[0], -1)
    out = {}
    for row, path in enumerate(_paths(stage, group)):
        data_row = int(index_map[row]) if index_map is not None else row
        values = group.tensors[data_row].numpy() if group.is_array else column[data_row]
        values = np.asarray(values, dtype=np.float64).reshape(-1)
        out[path] = np.concatenate((out[path], values)) if path in out else values
    return out


def _open_read(physx, query, attributes, cuda_stream):
    result = physx.read(query, attributes, cuda_stream=cuda_stream).wait()
    groups = []
    while (group := result.fetch_next().wait()) is not None:
        groups.append(group)
    return result, groups


def _read_once(physx, query, attributes, cuda_stream):
    """A read outside a capture, closed, which leaves its plan held on the query for a captured read to take."""
    result, _groups = _open_read(physx, query, attributes, cuda_stream)
    result.close().wait()


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


def _refused_reads(physx, device, stream, cuda_stream):
    """A captured read with no plan held on its query, or holding a host column, is refused before anything is
    queued, and the capture goes on."""
    _attach_and_step(physx, "boxes_falling_on_groundplane_gpu.usda")
    bodies = physx.query(SimObjectType.RIGID_BODY).wait()
    marker = wp.zeros(1, dtype=float, device=device)
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        marker.fill_(1.0)
        with pytest.raises(TaskError, match="read once outside the capture"):
            physx.read(bodies, ["position"], cuda_stream=cuda_stream).wait()
        with pytest.raises(TaskError, match="read on the host"):
            physx.read(bodies, ["mass"], cuda_stream=cuda_stream).wait()
    wp.capture_launch(capture.graph, stream=stream)
    wp.synchronize_stream(stream)
    assert marker.numpy()[0] == 1.0
    bodies.close().wait()


def _read_policy_write(physx, device, stream, cuda_stream):
    """Read, policy and write captured in one graph: each replay writes the poses the bodies hold when it runs,
    shifted by what the shift array holds then."""
    stage = _attach_and_step(physx, "boxes_falling_on_groundplane_gpu.usda")
    bodies = physx.query(SimObjectType.RIGID_BODY).wait()
    _read_once(physx, bodies, ["position"], cuda_stream)
    session, (write_group,) = _open_write(physx, bodies, "position")
    paths = _paths(stage, write_group)
    shift = wp.zeros(3, dtype=float, device=device)
    target = _flat(write_group.tensors, device)
    # The kernel's module loads outside the capture.
    wp.launch(_shift_positions, dim=target.shape[0], inputs=[target, shift, target], device=device, stream=stream)
    wp.synchronize_stream(stream)
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        result, (read_group,) = _open_read(physx, bodies, ["position"], cuda_stream)
        assert read_group.index_map is None
        assert _paths(stage, read_group) == paths, "the read and the write were expected to share their row order"
        source = _flat(read_group.tensors, device)
        wp.launch(_shift_positions, dim=target.shape[0], inputs=[source, shift, target], device=device,
                  stream=stream)
        session.commit(write_group, cuda_stream=cuda_stream).wait()
    for offset in ((0.0, 0.0, 2.0), (1.5, -1.0, 0.5)):
        physx.step(DT)
        physx.wait_all()
        before = read_column(physx, stage, SimObjectType.RIGID_BODY, "position")
        shift.assign(np.asarray(offset, dtype=np.float32))
        wp.capture_launch(capture.graph, stream=stream)
        wp.synchronize_stream(stream)
        after = read_column(physx, stage, SimObjectType.RIGID_BODY, "position")
        for path in paths:
            assert np.allclose(after[path], before[path] + offset, atol=1e-4), (offset, path, after[path])
    session.close().wait()
    result.close().wait()
    bodies.close().wait()


def _articulation_reads(physx, device, stream, cuda_stream):
    """Link poses and joint positions captured from two held queries in one graph replay the state after each
    step."""
    stage = _attach_and_step(physx, "two_articulations_gpu.usda")
    links = physx.query(SimObjectType.ARTICULATION_LINK).wait()
    joints = physx.query(SimObjectType.ARTICULATION_JOINT).wait()
    reads = ((links, "position", SimObjectType.ARTICULATION_LINK),
             (joints, "jointPosition", SimObjectType.ARTICULATION_JOINT))
    for query, attribute, _ in reads:
        _read_once(physx, query, [attribute], cuda_stream)
    physx.wait_all()
    # The articulations move, so a replay that did not run would leave the values read before the capture.
    previous = {attribute: read_column(physx, stage, object_type, attribute) for _, attribute, object_type in reads}
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        captured = [(*_open_read(physx, query, [attribute], cuda_stream), object_type, attribute)
                    for query, attribute, object_type in reads]
    for _ in range(2):
        for _ in range(5):
            physx.step(DT)
        physx.wait_all()
        wp.capture_launch(capture.graph, stream=stream)
        wp.synchronize_stream(stream)
        for _result, groups, object_type, attribute in captured:
            expected = read_column(physx, stage, object_type, attribute)
            assert any(not np.allclose(expected[path], previous[attribute][path]) for path in expected), attribute
            previous[attribute] = expected
            for group in groups:
                for path, values in _rows(stage, group).items():
                    assert np.allclose(values, expected[path], atol=1e-5), (attribute, path, values)
    for result, _groups, _, _ in captured:
        result.close().wait()
    links.close().wait()
    joints.close().wait()


def _producer_event(physx, device, stream, cuda_stream):
    """A read given the event a producer stream recorded after a write reads the written values, and holds the
    event's owner until its open is collected. The write reaches PhysX before the read either way, so this does not
    show the read waiting for the event; the runtime test "A read waits on the device for the event it is given before
    reusing its query's columns" does."""
    stage = _attach_and_step(physx, "boxes_falling_on_groundplane_gpu.usda")
    bodies = physx.query(SimObjectType.RIGID_BODY).wait()
    session, (write_group,) = _open_write(physx, bodies, "position")
    producer = wp.Stream(device)
    with wp.ScopedStream(producer):
        _flat(write_group.tensors, device).fill_(7.0)
    session.commit(write_group, cuda_stream=int(producer.cuda_stream)).wait()
    event = producer.record_event()
    owner = weakref.ref(event)
    opening = physx.read(bodies, ["position"], cuda_stream=cuda_stream, cuda_wait_event=int(event.cuda_event),
                         event_owner=event)
    del event
    assert owner() is not None, "the open does not hold the event's owner"
    result = opening.wait()
    groups = []
    while (group := result.fetch_next().wait()) is not None:
        groups.append(group)
    wp.synchronize_stream(stream)
    assert groups
    for group in groups:
        for path, values in _rows(stage, group).items():
            assert np.allclose(values, 7.0), (path, values)
    result.close().wait()
    session.close().wait()
    bodies.close().wait()


def _run_scenarios(scenarios):
    """Runs `scenarios` on one instance whose physics shares Warp's primary context. Each scenario's arrays die when
    it returns, so the wait between scenarios releases its reads before the next capture starts."""
    physx = PhysX(config=PhysXConfig(carbonite_overrides={"/physics/suppressReadback": True}), active_cuda_gpus="0")
    device = wp.get_device("cuda:0")
    stream = wp.get_stream(device)
    cuda_stream = int(stream.cuda_stream or 1)
    for scenario in scenarios:
        scenario(physx, device, stream, cuda_stream)
        physx.wait_all()


def _replay_scenarios():
    _run_scenarios((_refused_reads, _read_policy_write, _articulation_reads))


def _handoff_scenarios():
    _run_scenarios((_producer_event,))


def _run_in_fresh_process(scenario):
    """Runs this module's function `scenario` in a fresh process, which can create an instance on an explicit CUDA
    device."""
    command = (
        "import sys; sys.path[:0] = [sys.argv[1], sys.argv[2]]; "
        f"import test_read_capture_gpu as t; t.{scenario}()"
    )
    result = subprocess.run(
        [sys.executable, "-c", command, str(_HERE), str(_HERE.parent / "python_samples")],
        cwd=_HERE,
        text=True,
        capture_output=True,
        timeout=600,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_read_step_replays_from_a_cuda_graph():
    """A read, a Warp policy and a commit captured on Warp's stream replay against the state after each step; reads
    of two held queries replay in one graph; a captured read without a held plan, or holding a host column, is
    refused and the capture goes on."""
    _run_in_fresh_process("_replay_scenarios")


def test_read_takes_a_producer_event_and_keeps_its_owner():
    """A read given a producer stream's event reads what that stream wrote, and keeps the event alive until its open
    is collected."""
    _run_in_fresh_process("_handoff_scenarios")


def test_read_capture_on_another_cuda_context_is_refused(physx_sdk):
    """A read on a capturing stream of another context is refused before anything is queued on it."""
    _attach_and_step(physx_sdk, "boxes_falling_on_groundplane_gpu.usda")
    device = wp.get_device("cuda:0")
    stream = wp.get_stream(device)
    bodies = physx_sdk.query(SimObjectType.RIGID_BODY).wait()
    marker = wp.zeros(1, dtype=float, device=device)
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        marker.fill_(1.0)
        with pytest.raises(TaskError, match="another CUDA context"):
            physx_sdk.read(bodies, ["position"], cuda_stream=int(stream.cuda_stream or 1)).wait()
    wp.capture_launch(capture.graph, stream=stream)
    wp.synchronize_stream(stream)
    assert marker.numpy()[0] == 1.0
    # Outside a capture, the same read waits for the stream on the host and reads on the scene's own stream.
    result, groups = _open_read(physx_sdk, bodies, ["position"], int(stream.cuda_stream or 1))
    assert groups and all(group.tensors[0].device.is_cuda for group in groups)
    result.close().wait()
    bodies.close().wait()
