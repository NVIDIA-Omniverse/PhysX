# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-FRAME-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5

from types import SimpleNamespace

import numpy as np
import ovstage
import pytest
import warp as wp
from ovphysx.api import PhysX
from ovphysx.types import ObjectScope, SimObjectType
from ovphysx.utils import OvStageOutputCache, step_and_write_to_ovstage

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

    def get_attach_handle(self):
        return self._attach_handle

    def step_sync(self, dt):
        self.steps.append(dt)

    def read(self, object_type, names, *, scope):
        assert scope == ObjectScope.ALL
        return _ReadResult(self._groups_by_type.get(object_type, []))

    def _order_warp_stream_after_group(self, group, _wp):
        self.ordered_groups.append(group)


class _RecordingStage:
    def __init__(self, stage, *, failing_ordinal=None):
        self._stage = stage
        self._failing_ordinal = failing_ordinal
        self.writes = []
        self.write_floors = []
        self.reads = 0
        self.released_queries = []

    def __getattr__(self, name):
        return getattr(self._stage, name)

    def write_attribute(self, query, attribute, ordinal, tensors, **kwargs):
        self.writes.append((attribute, ordinal, tensors, kwargs))
        if ordinal == self._failing_ordinal:
            raise RuntimeError("write failed")
        return self._stage.write_attribute(query, attribute, ordinal, tensors, **kwargs)

    def read_attributes(self, query, attributes, ordinal_range):
        self.reads += 1
        return self._stage.read_attributes(query, attributes, ordinal_range)

    def release_query(self, query):
        self.released_queries.append(query)
        return self._stage.release_query(query)

    def advance_write_floor(self, *, ordinal):
        self.write_floors.append(ordinal)
        return self._stage.advance_write_floor(ordinal=ordinal)


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
                for tensor_index in range(group.tensor_count):
                    tensor = group.tensor(tensor_index)
                    array = np.array(group.array(tensor_index), copy=True)
                    if int(tensor.dtype.bits) == 16:
                        array = array.view(np.float16)
                    values.append(
                        (array, int(tensor.dtype.bits), int(tensor.dtype.lanes or 1), int(group.raw.semantic))
                    )
    return values


def _seed_fixed_stage(stage, paths, path_list, matrices, reset_values=(False, True), device="cpu"):
    matrix_values = wp.array(matrices, dtype=wp.float64, device=device)
    cuda_stream = int(wp.get_stream(device).cuda_stream or 1) if device.startswith("cuda") else None
    with stage.query_from_path_list(path_list) as query:
        prim_type = np.full(len(matrices), paths.intern_token("Xform"), dtype=np.uint64)
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
            np.asarray([paths.intern_token("PointInstancer")], dtype=np.uint64),
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
def test_fixed_pose_writes_only_world_matrix_and_reuses_buffers(device, monkeypatch):
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    with ovstage.Stage(f"output-cache-fixed-{device}") as raw_stage:
        stage = _RecordingStage(raw_stage)
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
                groups = [
                    _group(attribute=orientation_token, prim_list=int(path_list), tensors=[orientations], prim_count=2),
                    _group(attribute=position_token, prim_list=int(path_list), tensors=[positions], prim_count=2),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
                with monkeypatch.context() as patch:
                    patch.setattr(ovstage.ReadGroup, "array", _reject_numpy_view)
                    with OvStageOutputCache(physx) as cache:
                        assert (
                            step_and_write_to_ovstage(
                                physx,
                                dt=1.0 / 60.0,
                                output_ordinal=2,
                                cache=cache,
                                outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                            )
                            == 1
                        )
                        assert (
                            step_and_write_to_ovstage(
                                physx,
                                dt=1.0 / 60.0,
                                output_ordinal=3,
                                cache=cache,
                                outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                            )
                            == 1
                        )

                assert stage.reads == 1

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
        stage = _RecordingStage(raw_stage)
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
                        prim_list=int(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32, device=physics_device)],
                    ),
                    _group(
                        attribute=paths.intern_token("orientation"),
                        prim_list=int(path_list),
                        tensors=[wp.array([[0, 0, 0.70710677, 0.70710677]], dtype=wp.float32, device=physics_device)],
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
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
                            outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
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
                        outputs={SimObjectType.RIGID_BODY: ["position", "orientation"]},
                    )
                    == 1
                )

                assert stage.reads == 7
                with stage.query_from_path_list(path_list) as query:
                    matrix = _read_attribute(stage, query, paths, "omni:fabric:worldMatrix", 8)[0][0].reshape(4, 4)
                assert np.allclose(np.linalg.norm(matrix[:3, :3], axis=1), [5, 6, 7], atol=1.0e-12)


@pytest.mark.parametrize("device", ["cpu", pytest.param("cuda:0", marks=pytest.mark.cuda)])
def test_point_instancer_preserves_holes_and_authored_tail(device, monkeypatch):
    if device.startswith("cuda") and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    baseline_positions = [[10, 20, 30], [40, 50, 60], [70, 80, 90]]
    baseline_orientations = [[0, 0, 0, 1], [0, 0.5, 0, 0.866], [0, 0, 1, 0]]
    with ovstage.Stage(f"output-cache-instancer-{device}") as raw_stage:
        stage = _RecordingStage(raw_stage)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/PI"]) as path_list:
                _seed_instancer_stage(stage, paths, path_list, baseline_positions, baseline_orientations, device=device)
                positions = wp.array([[0, 0, 0], [999, 999, 999]], dtype=wp.float32, device=device)
                orientations = wp.array([[0, 0, 0.70710677, 0.70710677], [0, 0, 0, 0]], dtype=wp.float32, device=device)
                groups = [
                    _group(
                        attribute=paths.intern_token("orientations"),
                        prim_list=int(path_list),
                        tensors=[orientations],
                        is_array=True,
                        prim_count=1,
                        semantic=ovstage.AttributeSemantic.QUATERNION,
                    ),
                    _group(
                        attribute=paths.intern_token("positions"),
                        prim_list=int(path_list),
                        tensors=[positions],
                        is_array=True,
                        prim_count=1,
                        semantic=ovstage.AttributeSemantic.POINT,
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
                with monkeypatch.context() as patch:
                    patch.setattr(ovstage.ReadGroup, "array", _reject_numpy_view)
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
                        prim_list=int(path_list),
                        tensors=[positions],
                        is_array=True,
                    ),
                    _group(
                        attribute=paths.intern_token("orientations"),
                        prim_list=int(path_list),
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


def test_dlpack_capture_orders_warp_after_ovstage_event():
    with ovstage.Stage("output-cache-event-order") as stage:
        physx = _PhysX(stage)
        with OvStageOutputCache(physx) as cache:
            source = wp.zeros((1, 3), dtype=wp.float32)
            raw = SimpleNamespace(data=SimpleNamespace(cuda_sync=SimpleNamespace(wait_event=123)))
            tensor = SimpleNamespace(dtype=SimpleNamespace(bits=32, lanes=3))
            group = SimpleNamespace(
                raw=raw,
                tensor=lambda _index: tensor,
                dlpack=lambda _index: wp.to_dlpack(source),
            )
            borrowed = cache._group_array(group, 0, lanes=3, bits=32, name="positions")
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
                        prim_list=int(path_list),
                        tensors=[wp.array([[1, 2, 3]], dtype=wp.float32)],
                        is_array=True,
                    ),
                    _group(
                        attribute=paths.intern_token("orientations"),
                        prim_list=int(path_list),
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
                    prim_list=int(path_list),
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
        stage = _RecordingStage(raw_stage, failing_ordinal=2)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Particles"]) as path_list:
                group = _group(
                    attribute=paths.intern_token("points"),
                    prim_list=int(path_list),
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
        stage = _RecordingStage(raw_stage, failing_ordinal=2)
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
                        prim_list=int(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32)],
                    ),
                    _group(
                        attribute=paths.intern_token("orientation"),
                        prim_list=int(path_list),
                        tensors=[wp.array([[0, 0, 0, 1]], dtype=wp.float32)],
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
        stage = _RecordingStage(raw_stage)
        with ovstage.PathDictionary(stage) as paths:
            with paths.create_path_list_from_strings(["/World/Body"]) as path_list:
                groups = [
                    _group(
                        attribute=paths.intern_token("position"),
                        prim_list=int(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32)],
                    ),
                    _group(
                        attribute=paths.intern_token("orientation"),
                        prim_list=int(path_list),
                        tensors=[wp.array([[0, 0, 0, 1]], dtype=wp.float32)],
                    ),
                    _group(
                        attribute=paths.intern_token("linearVelocity"),
                        prim_list=int(path_list),
                        tensors=[wp.zeros((1, 3), dtype=wp.float32)],
                    ),
                ]
                physx = _PhysX(stage, {SimObjectType.RIGID_BODY: groups})
                with pytest.raises(RuntimeError, match="worldMatrix is missing"):
                    step_and_write_to_ovstage(
                        physx,
                        dt=0.1,
                        output_ordinal=2,
                        outputs={SimObjectType.RIGID_BODY: ["position", "orientation", "linearVelocity"]},
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
