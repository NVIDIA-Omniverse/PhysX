# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-FRAME-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-7

from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import ovstage
import pytest
import warp as wp
from ovphysx.api import PhysX
from ovphysx.types import ObjectScope, SimObjectType
from ovphysx.utils import OvStageOutputCache, split_joint_group, step_and_write_to_ovstage
from ovphysx.utils.simulation import _write_to_ovstage

import ovphysx


class _ReadResult:
    def __init__(self, groups):
        self.groups = groups

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return None


class _PhysX:
    def __init__(self, stage, groups_by_type=None, attach_handle=17):
        self._attached_ovstage = stage
        self._groups_by_type = groups_by_type or {}
        self._attach_handle = attach_handle
        self.steps = []
        self.ordered_groups = []
        self.read_names = []

    def is_stage_attached(self, stage=None):
        return stage is None or stage is getattr(self, "_native_stage", self._attached_ovstage)

    def _held_stages(self):
        return [] if self._attached_ovstage is None else [self._attached_ovstage]

    def _resolve_attached_stage(self):
        return next((stage for stage in self._held_stages() if self.is_stage_attached(stage)), None)

    def _capture_attachment_query(self):
        return self._attach_handle

    def _is_query_valid(self, query):
        return query == self._attach_handle

    def _release_attachment_query(self, query, *, defer=False):
        pass

    def step_sync(self, dt):
        self.steps.append(dt)

    def read_sync(self, object_type, names, *, scope):
        assert scope == ObjectScope.ALL
        self.read_names.append((object_type, list(names)))
        return _ReadResult(self._groups_by_type.get(object_type, []))

    def _order_warp_stream_after_group(self, group, _wp):
        self.ordered_groups.append(group)


def _record_stage_calls(stage, *, failing_ordinal=None):
    # Keep the real Stage owner: PathDictionary rejects forwarding proxies.
    stage.writes = []
    stage.write_floors = []
    stage.reads = 0
    stage.released_queries = []
    write_attribute = stage.write_attribute
    read_attributes = stage.read_attributes
    release_query = stage.release_query
    advance_write_floor = stage.advance_write_floor

    def record_write(query, attribute, ordinal, tensors, **kwargs):
        stage.writes.append((attribute, ordinal, tensors, kwargs))
        if ordinal == failing_ordinal:
            raise RuntimeError("write failed")
        return write_attribute(query, attribute, ordinal, tensors, **kwargs)

    def record_read(query, attributes, ordinal_range):
        stage.reads += 1
        return read_attributes(query, attributes, ordinal_range)

    def record_release(query):
        stage.released_queries.append(query)
        return release_query(query)

    def record_floor(*, ordinal):
        stage.write_floors.append(ordinal)
        return advance_write_floor(ordinal=ordinal)

    stage.write_attribute = record_write
    stage.read_attributes = record_read
    stage.release_query = record_release
    stage.advance_write_floor = record_floor
    return stage


def _group(
    *,
    attribute,
    prim_list,
    tensors,
    is_array=False,
    semantic=0,
    prim_count=1,
    index_map=None,
    prim_index_map=None,
    prim_offset=0,
    cuda_stream=0,
    cuda_wait_event=0,
):
    return SimpleNamespace(
        attribute=attribute,
        object_type=SimObjectType.RIGID_BODY,
        ordinal=0,
        is_array=is_array,
        is_delete=False,
        semantic=semantic,
        prim_list=prim_list,
        prim_offset=prim_offset,
        prim_count=prim_count,
        prim_index_map=prim_index_map,
        index_map=index_map,
        layout_generation=0,
        write_floor_ordinal=0,
        tensors=tensors,
        cuda_stream=cuda_stream,
        cuda_wait_event=cuda_wait_event,
    )


def _lane_tensor(values, bits, lanes):
    return ovstage.make_dltensor(
        values,
        dtype=ovstage.DLDataType(ovstage.DLDataTypeCode.kDLFloat, bits, lanes),
        shape=[len(values)],
        ndim=1,
    )


def _reject_numpy_view(*_args, **_kwargs):
    raise AssertionError("the helper must consume OVStage reads through DLPack")


def _read_attribute(stage, query, paths, attribute, ordinal):
    token = paths.intern_token(attribute)
    values = []
    with stage.read_attributes(query, [token], ovstage.OrdinalRange.latest(ordinal)) as read:
        read.wait()
        for group in read.groups():
            with group:
                for column in group.columns:
                    for tensor_index in range(column.tensor_count):
                        tensor = column.tensor(tensor_index)
                        array = np.array(column.array(tensor_index), copy=True)
                        if int(tensor.dtype.bits) == 16:
                            array = array.view(np.float16)
                        layout = ovstage.typed_tensor_to_write(tensor)
                        values.append(
                            (array, int(layout.dtype.bits), int(layout.dtype.lanes), int(column.raw.semantic))
                        )
    return values


def _seed_fixed_stage(stage, paths, path_list, matrices, reset_values=(False, True), device="cpu"):
    matrix_values = wp.array(matrices, dtype=wp.float64, device=device)
    cuda_stream = int(wp.get_stream(device).cuda_stream or 1) if device.startswith("cuda") else None
    with stage.query_from_path_list(path_list) as query:
        prim_type = np.full(len(matrices), paths.intern_token("Xform").raw_id, dtype=np.uint64)
        stage.write_attribute(
            query,
            "usd-prim-type",
            1,
            prim_type,
            is_array=False,
        ).wait()
        stage.write_attribute(
            query,
            "omni:resetXformStack",
            1,
            np.asarray(reset_values, dtype=np.bool_),
            is_array=False,
        ).wait()
        stage.write_attribute(
            query,
            "omni:fabric:worldMatrix",
            1,
            matrix_values,
            is_array=False,
            semantic=ovstage.AttributeSemantic.MATRIX,
            cuda_stream=cuda_stream,
        ).wait()
    stage.advance_write_floor(ordinal=1).wait()


def _seed_instancer_stage(stage, paths, path_list, positions, orientations, device="cpu"):
    position_values = wp.array(positions, dtype=wp.float32, device=device)
    orientation_values = wp.array(orientations, dtype=wp.float16, device=device)
    cuda_stream = int(wp.get_stream(device).cuda_stream or 1) if device.startswith("cuda") else None
    with stage.query_from_path_list(path_list) as query:
        stage.write_attribute(
            query,
            "usd-prim-type",
            1,
            np.asarray([paths.intern_token("PointInstancer").raw_id], dtype=np.uint64),
            is_array=False,
        ).wait()
        stage.write_attribute(
            query,
            "positions",
            1,
            [_lane_tensor(position_values, 32, 3)],
            is_array=True,
            semantic=ovstage.AttributeSemantic.POINT,
            cuda_stream=cuda_stream,
        ).wait()
        stage.write_attribute(
            query,
            "orientations",
            1,
            [_lane_tensor(orientation_values, 16, 4)],
            is_array=True,
            semantic=ovstage.AttributeSemantic.QUATERNION,
            cuda_stream=cuda_stream,
        ).wait()
    stage.advance_write_floor(ordinal=1).wait()


def test_utility_stays_outside_main_physx_api():
    assert not hasattr(PhysX, "step_and_write_to_ovstage")
    assert "step_and_write_to_ovstage" not in ovphysx.__all__
    assert not hasattr(ovphysx, "step_and_write_to_ovstage")


def test_cache_needs_no_application_scale_or_ordinal():
    with ovstage.Stage("output-cache-simple-construction") as stage:
        physx = _PhysX(stage)
        with OvStageOutputCache(physx):
            pass


def test_cache_rejects_wrong_type_and_physx_owner():
    with ovstage.Stage("output-cache-owner") as stage:
        owner = _PhysX(stage)
        other = _PhysX(stage)
        with pytest.raises(TypeError, match="OvStageOutputCache or None"):
            step_and_write_to_ovstage(owner, dt=0.1, output_ordinal=2, cache=object())
        with OvStageOutputCache(owner) as cache:
            with pytest.raises(RuntimeError, match="another PhysX instance"):
                step_and_write_to_ovstage(other, dt=0.1, output_ordinal=2, cache=cache)
        assert owner.steps == []
        assert other.steps == []


def test_pose_selection_is_rejected_before_step():
    with ovstage.Stage("output-cache-preflight") as stage:
        physx = _PhysX(stage)
        with OvStageOutputCache(physx) as cache:
            with pytest.raises(ValueError, match="selected together"):
                step_and_write_to_ovstage(
                    physx,
                    dt=0.1,
                    output_ordinal=2,
                    cache=cache,
                    outputs={SimObjectType.RIGID_BODY: ["position"]},
                )
        assert physx.steps == []


@pytest.mark.parametrize("device", ["cpu", pytest.param("cuda:0", marks=pytest.mark.cuda)])
@pytest.mark.parametrize("write_only", [False, True], ids=["step-and-write", "write-only"])
def test_wheel_pose_writes_only_world_matrix_and_reuses_buffers(device, write_only, monkeypatch):
    """Vehicle wheels have no stored scale, so the cache derives it once from the stage world matrix."""
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    with ovstage.Stage(f"output-cache-fixed-{device}") as raw_stage:
        stage = _record_stage_calls(raw_stage)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/A", "/World/B"]) as path_list:
                _seed_fixed_stage(
                    stage,
                    paths,
                    path_list,
                    [
                        [[-2, 0, 0, 0], [0, 3, 0, 0], [0, 0, 4, 0], [0, 0, 0, 1]],
                        [[2, 0, 0, 0], [0.5, 3, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                    ],
                    device=device,
                )
                position_token = paths.intern_token("position")
                orientation_token = paths.intern_token("orientation")
                positions = wp.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=wp.float32, device=device)
                orientations = wp.array(
                    [[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.70710677, 0.70710677]],
                    dtype=wp.float32,
                    device=device,
                )
                prim_list = ovstage.PrimPathList(path_list)
                groups = [
                    _group(attribute=orientation_token, prim_list=prim_list, tensors=[orientations], prim_count=2),
                    _group(attribute=position_token, prim_list=prim_list, tensors=[positions], prim_count=2),
                ]
                physx = _PhysX(stage, {SimObjectType.VEHICLE_WHEEL: groups})
                write_attribute = stage.write_attribute

                def check_order_and_write(*args, **kwargs):
                    assert physx.steps == expected_steps
                    assert stage.write_floors == expected_floors
                    return write_attribute(*args, **kwargs)

                with monkeypatch.context() as patch:
                    patch.setattr(ovstage.ReadColumn, "array", _reject_numpy_view)
                    patch.setattr(stage, "write_attribute", check_order_and_write)
                    with OvStageOutputCache(physx) as cache:
                        outputs = {SimObjectType.VEHICLE_WHEEL: ["position", "orientation"]}
                        for ordinal in (2, 3):
                            expected_steps = [] if write_only else [1.0 / 60.0] * (ordinal - 1)
                            expected_floors = list(range(1, ordinal))
                            if write_only:
                                written = _write_to_ovstage(physx, ordinal, cache, list(outputs.items()), retain=True)
                                assert physx.steps == []
                                assert stage.write_floors == expected_floors
                                stage.advance_write_floor(ordinal=ordinal).wait()
                            else:
                                written = step_and_write_to_ovstage(
                                    physx,
                                    dt=1.0 / 60.0,
                                    output_ordinal=ordinal,
                                    cache=cache,
                                    outputs=outputs,
                                )
                            assert written == 1
                            assert physx.steps == expected_steps
                            assert stage.write_floors == list(range(1, ordinal + 1))

                assert stage.reads == 1
                # A wheel has no stored scale, so worldScale is never requested for it.
                assert all("worldScale" not in names for _type, names in physx.read_names)

                matrix_writes = [
                    write for write in stage.writes if write[0] == paths.intern_token("omni:fabric:worldMatrix")
                ]
                assert len(matrix_writes) == 2
                assert matrix_writes[0][2] is matrix_writes[1][2]
                assert matrix_writes[0][3]["cuda_stream"] is None
                if device.startswith("cuda"):
                    assert matrix_writes[0][3]["cuda_event"] == matrix_writes[1][3]["cuda_event"] != 0
                else:
                    assert matrix_writes[0][3]["cuda_event"] is None

                helper_writes = [write for write in stage.writes if write[1] in (2, 3)]
                assert not any(write[0] in ("omni:xform", "omni:resetXformStack") for write in helper_writes)
                with stage.query_from_path_list(path_list) as query:
                    matrices = _read_attribute(stage, query, paths, "omni:fabric:worldMatrix", 3)
                    resets = _read_attribute(stage, query, paths, "omni:resetXformStack", 3)
                assert matrices[0][1:] == (64, 16, int(ovstage.AttributeSemantic.MATRIX))
                matrix = matrices[0][0].reshape(2, 4, 4)
                assert np.allclose(
                    matrix[0],
                    [[-2, 0, 0, 0], [0, 3, 0, 0], [0, 0, 4, 0], [1, 2, 3, 1]],
                    atol=1.0e-5,
                )
                assert np.allclose(
                    matrix[1],
                    [[0, 2, 0, 0], [-3, 0, 0, 0], [0, 0, 1, 0], [4, 5, 6, 1]],
                    atol=1.0e-5,
                )
                assert resets[0][0].tolist() == [False, True]


@pytest.mark.parametrize(
    ("stage_device", "physics_device"),
    [
        ("cpu", "cpu"),
        pytest.param("cpu", "cuda:0", marks=pytest.mark.cuda),
        pytest.param("cuda:0", "cpu", marks=pytest.mark.cuda),
        pytest.param("cuda:0", "cuda:0", marks=pytest.mark.cuda),
    ],
)
def test_default_reads_current_world_matrix_each_call(stage_device, physics_device, monkeypatch):
    if any(device.startswith("cuda") for device in (stage_device, physics_device)) and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    with ovstage.Stage(f"output-default-current-scale-{stage_device}-{physics_device}") as raw_stage:
        stage = _record_stage_calls(raw_stage)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Body"]) as path_list:
                _seed_fixed_stage(
                    stage,
                    paths,
                    path_list,
                    [[[2, 0, 0, 0], [0.5, 3, 0, 0], [0, 0, 4, 0], [0, 0, 0, 1]]],
                    reset_values=(False,),
                    device=stage_device,
                )
                groups = [
                    _group(
                        attribute=paths.intern_token("position"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32, device=physics_device)],
                    ),
                    _group(
                        attribute=paths.intern_token("orientation"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.array([[0, 0, 0.70710677, 0.70710677]], dtype=wp.float32, device=physics_device)],
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.VEHICLE_WHEEL: groups})
                monkeypatch.setattr(
                    OvStageOutputCache,
                    "__init__",
                    lambda *_args, **_kwargs: pytest.fail("default path constructed the persistent cache"),
                )
                monkeypatch.setattr(wp, "clone", lambda *_args, **_kwargs: pytest.fail("default path cloned data"))
                for ordinal in range(2, 7):
                    assert (
                        step_and_write_to_ovstage(
                            physx,
                            dt=0.1,
                            output_ordinal=ordinal,
                            outputs={SimObjectType.VEHICLE_WHEEL: ["position", "orientation"]},
                        )
                        == 1
                    )

                with stage.query_from_path_list(path_list) as query:
                    repeated_matrix = _read_attribute(stage, query, paths, "omni:fabric:worldMatrix", 6)[0][0].reshape(
                        4, 4
                    )
                assert np.allclose(
                    np.linalg.norm(repeated_matrix[:3, :3], axis=1),
                    [2, 3, 4],
                    atol=1.0e-12,
                )

                with stage.query_from_path_list(path_list) as query:
                    stage.write_attribute(
                        query,
                        "omni:fabric:worldMatrix",
                        7,
                        np.asarray(
                            [[[5, 0, 0, 0], [0, 6, 0, 0], [0, 0, 7, 0], [0, 0, 0, 1]]],
                            dtype=np.float64,
                        ),
                        is_array=False,
                        semantic=ovstage.AttributeSemantic.MATRIX,
                    ).wait()
                stage.advance_write_floor(ordinal=7).wait()
                assert (
                    step_and_write_to_ovstage(
                        physx,
                        dt=0.1,
                        output_ordinal=8,
                        outputs={SimObjectType.VEHICLE_WHEEL: ["position", "orientation"]},
                    )
                    == 1
                )

                assert stage.reads == 7
                with stage.query_from_path_list(path_list) as query:
                    matrix = _read_attribute(stage, query, paths, "omni:fabric:worldMatrix", 8)[0][0].reshape(4, 4)
                assert np.allclose(np.linalg.norm(matrix[:3, :3], axis=1), [5, 6, 7], atol=1.0e-12)


@pytest.mark.parametrize("device", ["cpu", pytest.param("cuda:0", marks=pytest.mark.cuda)])
@pytest.mark.parametrize("use_cache", [True, False], ids=["cache", "default"])
@pytest.mark.parametrize("object_type", [SimObjectType.RIGID_BODY, SimObjectType.ARTICULATION_LINK])
def test_body_pose_composes_stored_world_scale_without_stage_read(device, use_cache, object_type):
    """Rigid bodies and links compose with the physics-stored worldScale read beside the pose: the stage's
    own world matrix is never read, so a stage edit does not reach the output, and worldScale is written to
    sim:worldScale only when the caller selects it."""
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    with ovstage.Stage(f"output-stored-scale-{device}-{use_cache}-{object_type.name}") as raw_stage:
        stage = _record_stage_calls(raw_stage)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/A", "/World/B"]) as path_list:
                # The stage carries a different scale; it must not reach the output.
                _seed_fixed_stage(
                    stage,
                    paths,
                    path_list,
                    [
                        [[5, 0, 0, 0], [0, 6, 0, 0], [0, 0, 7, 0], [0, 0, 0, 1]],
                        [[5, 0, 0, 0], [0, 6, 0, 0], [0, 0, 7, 0], [0, 0, 0, 1]],
                    ],
                )
                groups = [
                    _group(
                        attribute=paths.intern_token("position"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.array([[1, 2, 3], [4, 5, 6]], dtype=wp.float32, device=device)],
                        prim_count=2,
                    ),
                    _group(
                        attribute=paths.intern_token("orientation"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[
                            wp.array([[0, 0, 0, 1], [0, 0, 0.70710677, 0.70710677]], dtype=wp.float32, device=device)
                        ],
                        prim_count=2,
                    ),
                    _group(
                        attribute=paths.intern_token("worldScale"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.array([[-2, -3, -4], [2, 3, 1]], dtype=wp.float32, device=device)],
                        prim_count=2,
                    ),
                ]
                physx = _PhysX(stage, {object_type: groups})
                with ExitStack() as stack:
                    cache = stack.enter_context(OvStageOutputCache(physx)) if use_cache else None
                    for ordinal, names in ((2, ["position", "orientation"]),
                                           (3, ["position", "orientation", "worldScale"])):
                        reads_before = stage.reads
                        written = step_and_write_to_ovstage(
                            physx, dt=0.1, output_ordinal=ordinal, cache=cache, outputs={object_type: names}
                        )
                        assert stage.reads == reads_before
                        # The matrix, plus sim:worldScale only once the caller selects it.
                        assert written == (1 if ordinal == 2 else 2)
                        # Latest-only storage serves the newest ordinal, so check each call's output now.
                        with stage.query_from_path_list(path_list) as query:
                            matrix = _read_attribute(stage, query, paths, "omni:fabric:worldMatrix", ordinal)[0][0]
                        matrix = matrix.reshape(2, 4, 4)
                        assert np.allclose(
                            matrix[0], [[-2, 0, 0, 0], [0, -3, 0, 0], [0, 0, -4, 0], [1, 2, 3, 1]], atol=1.0e-5
                        )
                        assert np.allclose(
                            matrix[1], [[0, 2, 0, 0], [-3, 0, 0, 0], [0, 0, 1, 0], [4, 5, 6, 1]], atol=1.0e-5
                        )

                # worldScale is requested once, whether or not the caller named it.
                assert [names.count("worldScale") for _type, names in physx.read_names] == [1, 1]
                sim_scale = paths.intern_token("sim:worldScale")
                sim_scale_writes = [write for write in stage.writes if write[0] in (sim_scale, "sim:worldScale")]
                assert [write[1] for write in sim_scale_writes] == [3]


def test_body_pose_without_world_scale_fails_without_writing_or_sealing():
    with ovstage.Stage("output-missing-world-scale") as raw_stage:
        stage = _record_stage_calls(raw_stage)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Body"]) as path_list:
                groups = [
                    _group(
                        attribute=paths.intern_token("position"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32)],
                    ),
                    _group(
                        attribute=paths.intern_token("orientation"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.array([[0, 0, 0, 1]], dtype=wp.float32)],
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
                with pytest.raises(RuntimeError, match="missing worldScale"):
                    step_and_write_to_ovstage(
                        physx,
                        dt=0.1,
                        output_ordinal=2,
                        outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                    )
                assert physx.steps == [0.1]
                assert stage.reads == 0
                assert not [write for write in stage.writes if write[1] == 2]
                assert 2 not in stage.write_floors


@pytest.mark.parametrize("device", ["cpu", pytest.param("cuda:0", marks=pytest.mark.cuda)])
def test_point_instancer_preserves_holes_and_authored_tail(device, monkeypatch):
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    baseline_positions = [[10, 20, 30], [40, 50, 60], [70, 80, 90]]
    baseline_orientations = [[0, 0, 0, 1], [0, 0.5, 0, 0.866], [0, 0, 1, 0]]
    with ovstage.Stage(f"output-cache-instancer-{device}") as raw_stage:
        stage = _record_stage_calls(raw_stage)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/PI"]) as path_list:
                _seed_instancer_stage(stage, paths, path_list, baseline_positions, baseline_orientations, device=device)
                positions = wp.array([[0, 0, 0], [999, 999, 999]], dtype=wp.float32, device=device)
                orientations = wp.array([[0, 0, 0.70710677, 0.70710677], [0, 0, 0, 0]], dtype=wp.float32, device=device)
                groups = [
                    _group(
                        attribute=paths.intern_token("orientations"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[orientations],
                        is_array=True,
                        prim_count=1,
                        semantic=ovstage.AttributeSemantic.QUATERNION,
                    ),
                    _group(
                        attribute=paths.intern_token("positions"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[positions],
                        is_array=True,
                        prim_count=1,
                        semantic=ovstage.AttributeSemantic.POINT,
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
                with monkeypatch.context() as patch:
                    patch.setattr(ovstage.ReadColumn, "array", _reject_numpy_view)
                    with OvStageOutputCache(physx) as cache:
                        assert (
                            step_and_write_to_ovstage(
                                physx,
                                dt=1.0 / 60.0,
                                output_ordinal=2,
                                cache=cache,
                                outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                            )
                            == 2
                        )
                        assert (
                            step_and_write_to_ovstage(
                                physx,
                                dt=1.0 / 60.0,
                                output_ordinal=3,
                                cache=cache,
                                outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                            )
                            == 2
                        )

                with stage.query_from_path_list(path_list) as query:
                    out_positions = _read_attribute(stage, query, paths, "positions", 3)[0]
                    out_orientations = _read_attribute(stage, query, paths, "orientations", 3)[0]
                assert out_positions[1:] == (32, 3, int(ovstage.AttributeSemantic.POINT))
                assert out_orientations[1:] == (16, 4, int(ovstage.AttributeSemantic.QUATERNION))
                assert np.allclose(out_positions[0].reshape(-1, 3), [[0, 0, 0], [40, 50, 60], [70, 80, 90]])
                expected_q = np.asarray(
                    [[0, 0, 0.70710677, 0.70710677], baseline_orientations[1], baseline_orientations[2]],
                    dtype=np.float16,
                )
                assert np.array_equal(out_orientations[0].reshape(-1, 4), expected_q)

                writes = [
                    write
                    for write in stage.writes
                    if write[0] in (paths.intern_token("positions"), paths.intern_token("orientations"))
                ]
                output_writes = [write for write in writes if write[1] in (2, 3)]
                assert len(output_writes) == 4
                first_ptrs = [write[2][0].data for write in output_writes[:2]]
                second_ptrs = [write[2][0].data for write in output_writes[2:]]
                assert first_ptrs == second_ptrs
                if device.startswith("cuda"):
                    assert output_writes[0][3]["cuda_event"] == output_writes[2][3]["cuda_event"] != 0
                else:
                    assert output_writes[0][3]["cuda_event"] is None


@pytest.mark.parametrize(
    ("stage_device", "physics_device"),
    [
        ("cpu", "cpu"),
        pytest.param("cpu", "cuda:0", marks=pytest.mark.cuda),
        pytest.param("cuda:0", "cpu", marks=pytest.mark.cuda),
        pytest.param("cuda:0", "cuda:0", marks=pytest.mark.cuda),
    ],
)
def test_point_instancer_grows_new_holes_with_defaults(stage_device, physics_device, monkeypatch):
    if any(device.startswith("cuda") for device in (stage_device, physics_device)) and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    with ovstage.Stage(f"output-cache-instancer-growth-{stage_device}-{physics_device}") as stage:
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/PI"]) as path_list:
                _seed_instancer_stage(stage, paths, path_list, [[10, 20, 30]], [[0, 0, 0, 1]], device=stage_device)
                positions = wp.array([[1, 2, 3], [8, 8, 8], [9, 9, 9]], dtype=wp.float32, device=physics_device)
                orientations = wp.array(
                    [[0, 0, 0, 1], [0, 0, 0, 0], [0, 0, 0, 0]], dtype=wp.float32, device=physics_device
                )
                groups = [
                    _group(
                        attribute=paths.intern_token("positions"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[positions],
                        is_array=True,
                    ),
                    _group(
                        attribute=paths.intern_token("orientations"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[orientations],
                        is_array=True,
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
                monkeypatch.setattr(wp, "clone", lambda *_args, **_kwargs: pytest.fail("default path cloned data"))
                step_and_write_to_ovstage(
                    physx,
                    dt=0.1,
                    output_ordinal=2,
                    outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                )

                with stage.query_from_path_list(path_list) as query:
                    out_positions = _read_attribute(stage, query, paths, "positions", 2)[0][0]
                    out_orientations = _read_attribute(stage, query, paths, "orientations", 2)[0][0]
                assert np.array_equal(out_positions.reshape(-1, 3), [[1, 2, 3], [0, 0, 0], [0, 0, 0]])
                assert np.array_equal(
                    out_orientations.reshape(-1, 4),
                    np.asarray([[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]], dtype=np.float16),
                )


def test_world_matrix_read_preserves_outer_stride():
    matrices = np.arange(3 * 16, dtype=np.float64).reshape(3, 4, 4)[::2]
    tensor = ovstage.DLTensor.from_dlpack(matrices)
    group = SimpleNamespace(
        is_array=False,
        raw=SimpleNamespace(data=SimpleNamespace(cuda_sync=SimpleNamespace(wait_event=0))),
        tensor=lambda _index: tensor,
        dlpack=lambda _index: matrices,
    )
    with ovstage.Stage("output-matrix-stride") as stage:
        with OvStageOutputCache(_PhysX(stage)) as cache:
            borrowed = cache._world_matrix_array(group)
            assert borrowed.shape == (2, 16)
            assert borrowed.strides == (matrices.strides[0], matrices.itemsize)
            assert np.array_equal(borrowed.numpy(), matrices.reshape(2, 16))


def test_cache_keeps_stored_scale_and_wheel_pose_buffers_apart(monkeypatch):
    # Bodies take their scale from the physics read and keep no scales; wheels keep the scales captured from the
    # stage. One prim set and device must not hand either mode the other's buffers.
    with ovstage.Stage("output-cache-scale-modes") as stage:
        with OvStageOutputCache(_PhysX(stage)) as cache:
            captured = wp.zeros(1, dtype=wp.vec3d, device="cpu")
            monkeypatch.setattr(cache, "_capture_world_scales", lambda *_args: captured)
            paths = ("/World/Prim",)
            stored = cache._fixed_pose_buffers(None, paths, "cpu", stored_scale=True)
            wheel = cache._fixed_pose_buffers(None, paths, "cpu", stored_scale=False)
            assert stored.scales is None
            assert wheel.scales is captured
            assert cache._fixed_pose_buffers(None, paths, "cpu", stored_scale=True) is stored
            assert cache._fixed_pose_buffers(None, paths, "cpu", stored_scale=False) is wheel


def test_dlpack_capture_orders_warp_after_ovstage_event():
    with ovstage.Stage("output-cache-event-order") as stage:
        physx = _PhysX(stage)
        with OvStageOutputCache(physx) as cache:
            source = wp.zeros((1, 3), dtype=wp.float32)
            raw = SimpleNamespace(data=SimpleNamespace(cuda_sync=SimpleNamespace(wait_event=123)))
            tensor = SimpleNamespace(dtype=SimpleNamespace(code=2, bits=32, lanes=3))
            column = SimpleNamespace(
                raw=raw,
                tensor=lambda _index: tensor,
                dlpack=lambda _index: wp.to_dlpack(source),
            )
            borrowed = cache._column_array(column, 0, lanes=3, bits=32, name="positions")
            assert borrowed.shape == (1, 3)
            assert physx.ordered_groups == [raw]


def test_point_instancer_rejects_mismatched_baseline_lengths():
    with ovstage.Stage("output-cache-instancer-mismatched-baseline") as stage:
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/PI"]) as path_list:
                _seed_instancer_stage(
                    stage,
                    paths,
                    path_list,
                    [[10, 20, 30]],
                    [[0, 0, 0, 1], [0, 0, 1, 0]],
                )
                groups = [
                    _group(
                        attribute=paths.intern_token("positions"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.array([[1, 2, 3]], dtype=wp.float32)],
                        is_array=True,
                    ),
                    _group(
                        attribute=paths.intern_token("orientations"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.array([[0, 0, 0, 1]], dtype=wp.float32)],
                        is_array=True,
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
                with pytest.raises(RuntimeError, match="positions and orientations must have the same length"):
                    step_and_write_to_ovstage(
                        physx,
                        dt=0.1,
                        output_ordinal=2,
                        outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                    )


def test_non_pose_array_output_keeps_shadow_attribute_behavior():
    with ovstage.Stage("output-cache-shadow-array") as stage:
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Particles"]) as path_list:
                group = _group(
                    attribute=paths.intern_token("points"),
                    prim_list=ovstage.PrimPathList(path_list),
                    tensors=[wp.array([[1, 2, 3], [4, 5, 6]], dtype=wp.float32)],
                    is_array=True,
                    semantic=ovstage.AttributeSemantic.POINT,
                )
                physx = _PhysX(stage, {SimObjectType.PARTICLE_SET: [group]})
                written = step_and_write_to_ovstage(
                    physx,
                    dt=0.1,
                    output_ordinal=2,
                    outputs={SimObjectType.PARTICLE_SET: ["points"]},
                )
                with stage.query_from_path_list(path_list) as query:
                    output = _read_attribute(stage, query, paths, "sim:points", 2)[0]
                assert written == 1
                assert output[1:] == (32, 3, int(ovstage.AttributeSemantic.POINT))
                assert np.array_equal(output[0].reshape(-1, 3), [[1, 2, 3], [4, 5, 6]])


def test_write_failure_releases_query_without_advancing_floor():
    with ovstage.Stage("output-write-failure") as raw_stage:
        stage = _record_stage_calls(raw_stage, failing_ordinal=2)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Particles"]) as path_list:
                group = _group(
                    attribute=paths.intern_token("points"),
                    prim_list=ovstage.PrimPathList(path_list),
                    tensors=[wp.array([[1, 2, 3]], dtype=wp.float32)],
                    is_array=True,
                    semantic=ovstage.AttributeSemantic.POINT,
                )
                physx = _PhysX(stage, {SimObjectType.PARTICLE_SET: [group]})
                with pytest.raises(RuntimeError, match="write failed"):
                    step_and_write_to_ovstage(
                        physx,
                        dt=0.1,
                        output_ordinal=2,
                        outputs={SimObjectType.PARTICLE_SET: ["points"]},
                    )

                assert stage.released_queries
                assert 2 not in stage.write_floors


def test_pose_write_failure_releases_queries_without_advancing_floor():
    with ovstage.Stage("output-pose-write-failure") as raw_stage:
        stage = _record_stage_calls(raw_stage, failing_ordinal=2)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Body"]) as path_list:
                _seed_fixed_stage(
                    stage,
                    paths,
                    path_list,
                    [[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]],
                    reset_values=(False,),
                )
                groups = [
                    _group(
                        attribute=paths.intern_token("position"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32)],
                    ),
                    _group(
                        attribute=paths.intern_token("orientation"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.array([[0, 0, 0, 1]], dtype=wp.float32)],
                    ),
                    _group(
                        attribute=paths.intern_token("worldScale"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.ones((1, 3), dtype=wp.float32)],
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
                with pytest.raises(RuntimeError, match="write failed"):
                    step_and_write_to_ovstage(
                        physx,
                        dt=0.1,
                        output_ordinal=2,
                        outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                    )

                assert len(stage.released_queries) >= 2
                assert 2 not in stage.write_floors


def test_missing_fixed_scale_fails_without_writing_or_sealing():
    with ovstage.Stage("output-cache-missing-scale") as raw_stage:
        stage = _record_stage_calls(raw_stage)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Body"]) as path_list:
                groups = [
                    _group(
                        attribute=paths.intern_token("position"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32)],
                    ),
                    _group(
                        attribute=paths.intern_token("orientation"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.array([[0, 0, 0, 1]], dtype=wp.float32)],
                    ),
                    _group(
                        attribute=paths.intern_token("linearVelocity"),
                        prim_list=ovstage.PrimPathList(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32)],
                    ),
                ]
                # A wheel's scale comes from the stage, so a missing world matrix fails there.
                physx = _PhysX(stage, {SimObjectType.VEHICLE_WHEEL: groups})
                with pytest.raises(RuntimeError, match="worldMatrix is missing"):
                    step_and_write_to_ovstage(
                        physx,
                        dt=0.1,
                        output_ordinal=2,
                        outputs={SimObjectType.VEHICLE_WHEEL: ["position", "orientation", "linearVelocity"]},
                    )
                assert physx.steps == [0.1]
                assert not [write for write in stage.writes if write[1] == 2]
                assert 2 not in stage.write_floors


def test_cache_rejects_reattach_and_close_is_idempotent():
    with ovstage.Stage("output-cache-lifetime") as stage:
        physx = _PhysX(stage)
        cache = OvStageOutputCache(physx)
        physx._attach_handle += 1
        with pytest.raises(RuntimeError, match="detach or reattach"):
            cache.refresh()
        cache.close()
        cache.close()
        with pytest.raises(RuntimeError, match="closed"):
            cache.refresh()


def test_cache_uses_native_query_lifetime_without_attachment_identity():
    with ovstage.Stage("output-cache-query-lifetime") as stage:
        physx = _PhysX(stage)

        def forbidden_identity():
            raise AssertionError("output caches must not retrieve runtime attach identities")

        physx.get_attach_handle = forbidden_identity
        released = []
        physx._release_attachment_query = released.append
        cache = OvStageOutputCache(physx)
        cache.refresh()
        physx._attach_handle += 1  # Native detach/reattach, same Python Stage.
        with pytest.raises(RuntimeError, match="detach or reattach"):
            cache.refresh()
        cache.close()
        cache.close()
        assert released == [17]


def test_cache_rejects_native_stage_replacement_with_stale_python_keepalive():
    with ovstage.Stage("old-stage") as original, ovstage.Stage("native-replacement") as replacement:
        physx = _PhysX(original)
        physx._native_stage = replacement
        released = []
        physx._release_attachment_query = released.append
        with pytest.raises(RuntimeError, match="current ovstage.Stage object"):
            OvStageOutputCache(physx)
        assert released == [17]


def test_cache_collection_defers_native_query_release():
    import gc
    with ovstage.Stage("collected-cache") as stage:
        physx = _PhysX(stage)
        releases = []
        physx._release_attachment_query = lambda query, *, defer=False: releases.append((query, defer))
        cache = OvStageOutputCache(physx)
        del cache
        gc.collect()
        assert releases == [(17, True)]


class _RecordingPhysX(_PhysX):
    def __init__(self, stage, groups_by_type=None):
        super().__init__(stage, groups_by_type)
        self.reads = []

    def read_sync(self, object_type, names, *, scope):
        self.reads.append((object_type, tuple(names)))
        return super().read_sync(object_type, names, scope=scope)


@pytest.mark.parametrize("skin_rows", [True, False], ids=["skin-rows", "no-skin-rows"])
def test_default_set_reads_every_type_and_writes_skin_points(skin_rows):
    skin_points = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    with ovstage.Stage(f"output-default-set-{int(skin_rows)}") as stage:
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Body/skin"]) as path_list:
                groups_by_type = {}
                if skin_rows:
                    groups_by_type[SimObjectType.DEFORMABLE_SKIN_MESH] = [
                        _group(
                            attribute=paths.intern_token("points"),
                            prim_list=ovstage.PrimPathList(path_list),
                            tensors=[wp.array(skin_points, dtype=wp.float32)],
                            is_array=True,
                            semantic=ovstage.AttributeSemantic.POINT,
                        )
                    ]
                physx = _RecordingPhysX(stage, groups_by_type)
                written = step_and_write_to_ovstage(physx, dt=0.1, output_ordinal=2)

                # Pins the whole default set in order, so a change to the default set fails this test.
                # Bodies and links also read the stored worldScale their poses are composed with.
                body_reads = ("position", "orientation", "linearVelocity", "angularVelocity", "worldScale")
                assert physx.reads == [
                    (SimObjectType.RIGID_BODY, body_reads),
                    (SimObjectType.ARTICULATION_LINK, body_reads),
                    (
                        SimObjectType.ARTICULATION,
                        ("rootPosition", "rootOrientation", "rootLinearVelocity", "rootAngularVelocity"),
                    ),
                    (SimObjectType.ARTICULATION_JOINT, ("jointPosition", "jointVelocity")),
                    (SimObjectType.VEHICLE_WHEEL, ("position", "orientation")),
                    (SimObjectType.DEFORMABLE_VOLUME, ("points", "velocities")),
                    (SimObjectType.DEFORMABLE_SURFACE, ("points", "velocities")),
                    (SimObjectType.PARTICLE_SET, ("points", "velocities")),
                    (SimObjectType.DEFORMABLE_SKIN_MESH, ("points",)),
                ]
                assert physx.steps == [0.1]
                if skin_rows:
                    with stage.query_from_path_list(path_list) as query:
                        output = _read_attribute(stage, query, paths, "sim:points", 2)[0]
                    assert written == 1
                    assert np.array_equal(output[0].reshape(-1, 3), skin_points)
                else:
                    assert written == 0


def _published_points(stage, paths, prim_path, ordinal):
    with paths.create_path_list_from_strings([prim_path]) as path_list:
        with stage.query_from_path_list(path_list) as query:
            values = _read_attribute(stage, query, paths, "sim:points", ordinal)
    return values[0][0].reshape(-1, 3) if values else None


def _prim_matrix(stage, paths, prim_path, attribute, ordinal):
    with paths.create_path_list_from_strings([prim_path]) as path_list:
        with stage.query_from_path_list(path_list) as query:
            values = _read_attribute(stage, query, paths, attribute, ordinal)
    return values[0][0].reshape(4, 4)


def test_parent_edit_after_publication_moves_kinematic_child(physx_sdk):
    """OMPE-120176: an edit of the parent after publication moves the child with it, in OVStage and in PhysX
    (REQ-PYTHON-FRAME-001 AC-7)."""
    from session_helpers import read_column
    from test_utils import attach_usd_with_ovstage, data_path

    parent = "/World/Outer/Parent"
    body = "/World/Outer/Parent/Body"
    stage = attach_usd_with_ovstage(physx_sdk, data_path("ovstage_output_parent_edit.usda"))
    physx_sdk.wait_all()
    stage.compute_hierarchy(1, 2)
    stage.advance_write_floor(ordinal=2).wait()
    step_and_write_to_ovstage(physx_sdk, dt=1.0 / 60.0, output_ordinal=3)

    # Outer turns the parent's (3, 4, 0) translation into (-8, 6, 0) in world space.
    translation = wp.array([[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [3, 4, 0, 1]]], dtype=wp.float64, device="cpu")
    with ovstage.PathDictionary(stage) as paths:
        with paths.create_path_list_from_strings([parent]) as path_list:
            with stage.query_from_path_list(path_list) as query:
                stage.write_attribute(
                    query, "omni:xform", 4, translation, is_array=False, semantic=ovstage.AttributeSemantic.MATRIX
                ).wait()
        stage.advance_write_floor(ordinal=4).wait()
        parent_world = _prim_matrix(stage, paths, parent, "omni:fabric:worldMatrix", 4)
        body_world = _prim_matrix(stage, paths, body, "omni:fabric:worldMatrix", 4)
        body_local = _prim_matrix(stage, paths, body, "omni:xform", 4)
    assert np.allclose(parent_world[3, :3], [42, 6, 0], atol=1.0e-4)
    assert np.allclose(body_world[3, :3], [42, 6, 8], atol=1.0e-4)
    assert np.allclose(body_local[3, :3], [0, 0, 4], atol=1.0e-4)

    physx_sdk.wait_task(physx_sdk.update_from_ovstage(4, 4))
    physx_sdk.step_sync(1.0 / 60.0)
    position = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")[body]
    assert np.allclose(position, [42, 6, 8], atol=1.0e-4)


@pytest.mark.cuda
@pytest.mark.parametrize("rendering", [True, False])
def test_default_set_publishes_skin_points_only_with_rendering_domain(physx_sdk, rendering):
    if not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    from session_helpers import read_column
    from test_utils import attach_usd_with_ovstage, data_path

    stage = attach_usd_with_ovstage(physx_sdk, data_path("volume_deformable_skinned.usda"), rendering=rendering)
    physx_sdk.wait_all()
    step_and_write_to_ovstage(physx_sdk, dt=1.0 / 60.0, output_ordinal=2)

    skin = read_column(physx_sdk, stage, SimObjectType.DEFORMABLE_SKIN_MESH, "points")
    volume = read_column(physx_sdk, stage, SimObjectType.DEFORMABLE_VOLUME, "points")
    with ovstage.PathDictionary(stage) as paths:
        published_sim = _published_points(stage, paths, "/World/Body/simMesh", 2)
        published_skin = _published_points(stage, paths, "/World/Body/skin", 2)

    # The volume's own output is published in both populations, which shows that the skin-mesh entry
    # does not disturb the existing outputs.
    assert published_sim is not None
    assert np.allclose(published_sim, volume["/World/Body/simMesh"].reshape(-1, 3))
    if rendering:
        assert list(skin) == ["/World/Body/skin"]
        assert published_skin is not None
        assert np.allclose(published_skin, skin["/World/Body/skin"].reshape(-1, 3))
    else:
        assert skin == {}
        assert published_skin is None


# The humanoid's joints have one, two and three axes, so its joint groups repeat paths (ADR-0059).
_HUMANOID = str(Path(__file__).resolve().parents[1] / "benchmarks" / "data" / "humanoid.usda")


def _path_strings(paths, path_list, offset=0, count=None):
    handles = paths.get_paths(path_list)
    count = len(handles) - offset if count is None else count
    return [paths.path_to_string(handle) for handle in handles[offset : offset + count]]


def _consecutive_unique(values):
    return [value for index, value in enumerate(values) if index == 0 or value != values[index - 1]]


def test_split_joint_group_views_a_joint_read_group_per_joint(physx_sdk):
    """A joint read group, one row per DOF, becomes one view per joint over its own column, with a path list naming
    each joint once (REQ-PYTHON-FRAME-001 AC-6)."""
    from test_utils import attach_usd_with_ovstage

    stage = attach_usd_with_ovstage(physx_sdk, _HUMANOID)
    physx_sdk.wait_all()
    physx_sdk.step_sync(1.0 / 60.0)
    with physx_sdk.read_sync(SimObjectType.ARTICULATION_JOINT, ["jointPosition"]) as result:
        group = result.groups[0]
        assert not group.is_array and len(group.tensors) == 1
        column = group.tensors[0]
        with ovstage.PathDictionary(stage) as paths:
            row_paths = _path_strings(paths, group.prim_list, group.prim_offset, group.prim_count)
            prim_list, tensors = split_joint_group(stage, group)
            with prim_list:
                joint_paths = _path_strings(paths, prim_list)
                assert joint_paths == _consecutive_unique(row_paths)
                assert len(tensors) == len(joint_paths) < len(row_paths)
                offset = 0
                for path, tensor in zip(joint_paths, tensors):
                    assert tensor.ptr == column.ptr + offset * 4, path
                    assert int(tensor.shape[0]) == row_paths.count(path)
                    offset += int(tensor.shape[0])
                assert offset == int(column.shape[0])


def test_split_joint_group_views_fill_a_joint_write_group(physx_sdk):
    """Writing through the views of a joint write group fills its column joint by joint (REQ-PYTHON-FRAME-001
    AC-6)."""
    from session_helpers import read_column
    from test_utils import attach_usd_with_ovstage

    stage = attach_usd_with_ovstage(physx_sdk, _HUMANOID)
    physx_sdk.wait_all()
    physx_sdk.step_sync(1.0 / 60.0)
    wrote = {}
    with physx_sdk.write_sync(SimObjectType.ARTICULATION_JOINT, "jointPosition") as session:
        for group in session.groups:
            prim_list, tensors = split_joint_group(stage, group)
            with ovstage.PathDictionary(stage) as paths, prim_list:
                for index, (path, tensor) in enumerate(zip(_path_strings(paths, prim_list), tensors)):
                    wrote[path] = 1.0 + index  # a small angle per joint, distinct from every other joint's
                    tensor.fill_(wrote[path])
            session.commit_sync(group)
    assert wrote
    for path, values in read_column(physx_sdk, stage, SimObjectType.ARTICULATION_JOINT, "jointPosition").items():
        np.testing.assert_allclose(values, np.full(values.size, wrote[path]), atol=1e-3)


def test_joint_output_is_published_as_one_array_per_joint(physx_sdk):
    """step_and_write_to_ovstage publishes a joint attribute as one array per joint prim holding that joint's DOFs in
    order, though the joint group has one row per DOF (REQ-PYTHON-FRAME-001 AC-6)."""
    from session_helpers import read_column
    from test_utils import attach_usd_with_ovstage

    stage = attach_usd_with_ovstage(physx_sdk, _HUMANOID)
    physx_sdk.wait_all()
    step_and_write_to_ovstage(
        physx_sdk,
        dt=1.0 / 60.0,
        output_ordinal=2,
        outputs={SimObjectType.ARTICULATION_JOINT: ("jointPosition", "jointVelocity")},
    )
    for attribute in ("jointPosition", "jointVelocity"):
        joints = read_column(physx_sdk, stage, SimObjectType.ARTICULATION_JOINT, attribute)
        assert any(values.size > 1 for values in joints.values()), "the fixture must have a multi-DOF joint"
        with ovstage.PathDictionary(stage) as paths:
            for path, values in joints.items():
                with paths.create_path_list_from_strings([path]) as path_list:
                    with stage.query_from_path_list(path_list) as query:
                        published = _read_attribute(stage, query, paths, f"sim:{attribute}", 2)
                assert len(published) == 1, path
                np.testing.assert_allclose(published[0][0].reshape(-1), values, rtol=1e-6, atol=1e-6, err_msg=path)


def test_joint_physx_pointer_is_published_as_one_value_per_joint(physx_sdk):
    """A joint's physxPointer has one row per joint, so step_and_write_to_ovstage publishes it as one value per joint
    prim, not as an array (REQ-PYTHON-FRAME-001 AC-6)."""
    from test_utils import attach_usd_with_ovstage

    stage = attach_usd_with_ovstage(physx_sdk, _HUMANOID)
    physx_sdk.wait_all()
    step_and_write_to_ovstage(
        physx_sdk,
        dt=1.0 / 60.0,
        output_ordinal=2,
        outputs={SimObjectType.ARTICULATION_JOINT: ("physxPointer",)},
    )
    with ovstage.PathDictionary(stage) as paths:
        with physx_sdk.read_sync(SimObjectType.ARTICULATION_JOINT, ["physxPointer"]) as result:
            (group,) = result.groups
            assert group.index_map is None
            pointers = [int(value) for value in np.asarray(group.tensors[0].numpy()).reshape(-1)]
            handles = paths.get_paths(group.prim_list)[group.prim_offset : group.prim_offset + group.prim_count]
            joint_paths = [paths.path_to_string(handle) for handle in handles]
        assert len(set(joint_paths)) == len(joint_paths) == len(pointers), "physxPointer has one row per joint"
        token = paths.intern_token("sim:physxPointer")
        for path, pointer in zip(joint_paths, pointers):
            with paths.create_path_list_from_strings([path]) as path_list:
                with stage.query_from_path_list(path_list) as query:
                    with stage.read_attributes(query, [token], ovstage.OrdinalRange.latest(2)) as read:
                        read.wait()
                        (published,) = list(read.groups())
                        with published:
                            (column,) = published.columns
                            assert not column.is_array, path
                            assert int(np.array(column.array(0)).reshape(-1)[0]) == pointer, path


def test_split_joint_group_refuses_a_group_not_shaped_as_a_joint_group():
    """A deleted group, two tensors, or a row count other than the prim count is not a joint group
    (REQ-CAPI-JOINT-ARRAYS-001 AC-4)."""
    column = wp.zeros(3, dtype=wp.float32, device="cpu")
    with pytest.raises(ValueError):
        split_joint_group(
            None, SimpleNamespace(is_delete=True, tensors=[column], prim_list=0, prim_offset=0, prim_count=3)
        )
    with pytest.raises(ValueError):
        split_joint_group(None, SimpleNamespace(tensors=[column, column], prim_list=0, prim_offset=0, prim_count=3))
    with pytest.raises(ValueError):
        split_joint_group(None, SimpleNamespace(tensors=[column], prim_list=0, prim_offset=0, prim_count=2))
