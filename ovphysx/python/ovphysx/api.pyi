# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-1 AC-5
# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
# @implements REQ-PYTHON-KEYWORD-001
# @covers AC-1 AC-2
# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-4
# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @implements REQ-PYTHON-BINDING-DEVICE-001
# @covers AC-1 AC-2 AC-3 AC-4

from __future__ import annotations

from collections.abc import Sequence
from typing import Any, NamedTuple, TypedDict

from typing_extensions import deprecated

import warp as wp

from .config import OmniPvdDestination, PhysXConfig
from .dlpack import DLDataType, DLDevice
from .types import (
    LogLevel,
    ObjectScope,
    ObjectType,
    SceneQueryGeometryType,
    SceneQueryMode,
    SimObjectType,
)


class SceneQueryHit(TypedDict):
    collision: int
    rigid_body: int
    proto_index: int
    normal: tuple[float, float, float]
    position: tuple[float, float, float]
    distance: float
    face_index: int
    material: int


class ContactHeaderDict(TypedDict):
    type: int
    attachHandle: int
    actor0: int
    actor1: int
    collider0: int
    collider1: int
    contactDataOffset: int
    numContactData: int
    frictionAnchorsDataOffset: int
    numfrictionAnchorsData: int
    protoIndex0: int
    protoIndex1: int


class ContactPointDict(TypedDict):
    position: tuple[float, float, float]
    normal: tuple[float, float, float]
    impulse: tuple[float, float, float]
    separation: float
    faceIndex0: int
    faceIndex1: int
    material0: int
    material1: int


class FrictionAnchorDict(TypedDict):
    position: tuple[float, float, float]
    impulse: tuple[float, float, float]


class ContactReportDict(TypedDict, total=False):
    headers: list[ContactHeaderDict] | Any
    num_headers: int
    points: list[ContactPointDict] | Any
    num_points: int
    anchors: list[FrictionAnchorDict] | Any
    num_anchors: int


@deprecated("The tensor-binding API is deprecated; use PhysX.read for reads and PhysX.write for writes.")
class TensorBindingSpec(NamedTuple):
    """.. deprecated:: 0.6.0
    The tensor-binding API is deprecated; use :meth:`PhysX.read` for reads and
    :meth:`PhysX.write` for writes.
    """

    dtype: DLDataType
    ndim: int
    shape: tuple[int, ...]


class ReadGroup(NamedTuple):
    attribute: int
    object_type: SimObjectType
    ordinal: int
    is_array: bool
    is_delete: bool
    semantic: int
    prim_list: int
    prim_offset: int
    prim_count: int
    prim_index_map: wp.array | None
    index_map: wp.array | None
    layout_generation: int
    write_floor_ordinal: int
    tensors: list[wp.array]
    cuda_stream: int
    cuda_wait_event: int


class ReadResult:
    groups: list[ReadGroup]
    def __init__(
        self,
        sdk: PhysX,
        query: int,
        read: int,
        groups: list[ReadGroup],
        group_ids: list[int],
    ) -> None: ...
    @property
    def dictionary(self) -> int: ...
    def close(self) -> None: ...
    def __enter__(self) -> ReadResult: ...
    def __exit__(self, *exc: object) -> None: ...


class WriteGroup(NamedTuple):
    prim_list: int
    prim_offset: int
    prim_count: int
    tensors: list[wp.array]


class WriteSession:
    groups: list[WriteGroup]
    def __init__(
        self,
        sdk: PhysX,
        query: int,
        write: int,
        groups: list[WriteGroup],
        native: list[Any],
    ) -> None: ...
    def commit(
        self,
        group: WriteGroup,
        cuda_stream: int = ...,
        cuda_wait_event: int = ...,
    ) -> None: ...
    def close(self) -> None: ...
    def __enter__(self) -> WriteSession: ...
    def __exit__(self, *exc: object) -> None: ...


@deprecated("The tensor-binding API is deprecated; use PhysX.read for reads and PhysX.write for writes.")
class TensorBinding:
    def __init__(
        self,
        sdk: PhysX,
        handle: int,
        tensor_type: int,
        ndim: int,
        shape: tuple[int, ...],
        dtype: DLDataType | None = ...,
    ) -> None: ...
    @property
    def handle(self) -> int: ...
    @property
    def tensor_type(self) -> int: ...
    @property
    def ndim(self) -> int: ...
    @property
    def shape(self) -> tuple[int, ...]: ...
    @property
    def dtype(self) -> DLDataType: ...
    @property
    def dtype_name(self) -> str: ...
    @property
    def native_device(self) -> DLDevice: ...
    @property
    def spec(self) -> TensorBindingSpec: ...
    @property
    def count(self) -> int: ...
    @property
    def prim_paths(self) -> list[str]: ...
    @property
    def dof_count(self) -> int: ...
    @property
    def body_count(self) -> int: ...
    @property
    def is_fixed_base(self) -> bool: ...
    @property
    def dof_names(self) -> list[str]: ...
    @property
    def body_names(self) -> list[str]: ...
    @property
    def joint_count(self) -> int: ...
    @property
    def joint_names(self) -> list[str]: ...
    @property
    def fixed_tendon_count(self) -> int: ...
    @property
    def spatial_tendon_count(self) -> int: ...
    def read(self, tensor: Any) -> None: ...
    def write(
        self,
        tensor: Any,
        indices: Any | None = ...,
        mask: Any | None = ...,
    ) -> None: ...
    def wake_up(self, indices: Any | None = ...) -> None: ...
    def sleep(self, indices: Any | None = ...) -> None: ...
    def destroy(self) -> None: ...
    def __enter__(self) -> TensorBinding: ...
    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None: ...


class SdfView:
    def __init__(
        self,
        sdk: PhysX,
        handle: int,
        count: int,
        max_query_points: int,
    ) -> None: ...
    @property
    def count(self) -> int: ...
    @property
    def max_query_points(self) -> int: ...
    def evaluate(
        self,
        query_points: Any,
        out_distances_and_gradients: Any,
    ) -> None: ...
    def destroy(self) -> None: ...
    def __enter__(self) -> SdfView: ...
    def __exit__(self, *args: object) -> None: ...


class ContactBinding:
    def __init__(
        self,
        sdk: PhysX,
        handle: int,
        sensor_count: int,
        filter_count: int,
        max_contact_data_count: int,
    ) -> None: ...
    @property
    def sensor_count(self) -> int: ...
    @property
    def filter_count(self) -> int: ...
    @property
    def max_contact_data_count(self) -> int: ...
    @property
    def sensor_paths(self) -> list[str]: ...
    @property
    def filter_paths(self) -> list[list[str]]: ...
    def read_net_forces(self, output: Any) -> None: ...
    def read_force_matrix(self, output: Any) -> None: ...
    def read_contact_data(
        self,
        contact_forces: Any,
        positions: Any,
        normals: Any,
        separations: Any,
        counts: Any,
        start_indices: Any,
    ) -> None: ...
    def read_raw_contact_data(
        self,
        contact_forces: Any,
        positions: Any,
        normals: Any,
        separations: Any,
        sensor_layout: Any,
        actor_ids: Any,
    ) -> None: ...
    def get_other_actor_paths_from_ids(self, ids_array: Any) -> list[str]: ...
    def read_friction_data(
        self,
        friction_forces: Any,
        friction_points: Any,
        counts: Any,
        start_indices: Any,
    ) -> None: ...
    def destroy(self) -> None: ...
    def __enter__(self) -> ContactBinding: ...
    def __exit__(self, *args: object) -> None: ...


class PhysX:
    def __init__(
        self,
        *,
        config: PhysXConfig | None = ...,
        ignore_version_mismatch: bool = ...,
        active_cuda_gpus: str | None = ...,
    ) -> None: ...
    @property
    def handle(self) -> int: ...
    @staticmethod
    def set_cpu_mode(cpu_only: bool) -> None: ...
    @staticmethod
    def get_cpu_mode() -> bool: ...
    def reset_stage(self) -> int: ...
    def clone(
        self,
        source_path: str,
        target_paths: list[str],
        anchor_transforms: list[Sequence[float]] | None = ...,
        env_ids: list[int] | None = ...,
    ) -> int: ...
    def get_object_type(self, prim_path: str) -> ObjectType: ...
    def start_recording(self, destination: OmniPvdDestination) -> None: ...
    def stop_recording(self) -> None: ...
    def is_recording(self) -> bool: ...
    def step(self, dt: float) -> int: ...
    def step_sync(self, dt: float) -> None: ...
    def step_n_sync(self, n: int, dt: float) -> None: ...
    def update_articulations_kinematic(self) -> None: ...
    def wait_op(self, op_index: int, *, timeout_ns: int | None = ...) -> None: ...
    def wait_all(self, *, timeout_ns: int | None = ...) -> None: ...
    def attach_ovstage(self, stage: Any, *, read_ordinal: int = ...) -> None: ...
    def update_from_ovstage(self, from_ordinal: int, to_ordinal: int) -> None: ...
    def detach_ovstage(self) -> None: ...
    def get_attach_handle(self) -> int: ...
    def read(
        self,
        object_type: SimObjectType,
        attribute_names: list[str],
        *,
        scope: ObjectScope = ...,
    ) -> ReadResult: ...
    def read_tokens(
        self,
        object_type: SimObjectType,
        attribute_tokens: list[int],
        *,
        scope: ObjectScope = ...,
    ) -> ReadResult: ...
    def write(
        self,
        object_type: SimObjectType,
        attribute_name: str,
        *,
        scope: ObjectScope = ...,
    ) -> WriteSession: ...
    def query_shared_dictionary(self, query: int) -> int: ...
    def set_config(self, entry: Any) -> None: ...
    def set_config_bool(self, key: int, value: bool) -> None: ...
    def set_config_int32(self, key: int, value: int) -> None: ...
    def set_config_float(self, key: int, value: float) -> None: ...
    def get_config_bool(self, key: int) -> bool: ...
    def get_config_int32(self, key: int) -> int: ...
    def get_config_float(self, key: int) -> float: ...
    def get_config_string(self, key: int) -> str | None: ...
    def destroy(self) -> None: ...
    @deprecated("The tensor-binding API is deprecated; use PhysX.read for reads and PhysX.write for writes.")
    def create_tensor_binding(
        self,
        pattern: str | None = ...,
        prim_paths: list[str] | None = ...,
        tensor_type: int = ...,
        *,
        raise_if_empty: bool = ...,
    ) -> TensorBinding: ...
    def warmup(self) -> None: ...
    def get_contact_report(
        self,
        *,
        include_friction_anchors: bool = ...,
        copy: bool = ...,
    ) -> ContactReportDict: ...
    def raycast(
        self,
        origin: Sequence[float],
        direction: Sequence[float],
        distance: float,
        mode: SceneQueryMode = ...,
        both_sides: bool = ...,
    ) -> list[SceneQueryHit]: ...
    def sweep(
        self,
        geometry_type: SceneQueryGeometryType,
        direction: Sequence[float],
        distance: float,
        mode: SceneQueryMode = ...,
        both_sides: bool = ...,
        **kwargs: float | str | Sequence[float],
    ) -> list[SceneQueryHit]: ...
    def overlap(
        self,
        geometry_type: SceneQueryGeometryType,
        mode: SceneQueryMode = ...,
        **kwargs: float | str | Sequence[float],
    ) -> list[SceneQueryHit]: ...
    def get_scene_query_paths_from_ids(self, ids: Sequence[int]) -> list[str]: ...
    def create_contact_binding(
        self,
        sensor_patterns: list[str],
        filter_patterns: list[str] | None = ...,
        filters_per_sensor: int = ...,
        max_contact_data_count: int = ...,
    ) -> ContactBinding: ...
    def create_sdf_view(self, pattern: str, max_query_points: int) -> SdfView: ...


def set_log_level(level: int | LogLevel) -> None: ...
def get_log_level() -> int: ...
def enable_default_log_output(enable: bool = ...) -> None: ...
def flush_log(timeout_ns: int = ...) -> None: ...
def enable_python_logging(
    logger_name: str = ...,
    *,
    min_severity: int | LogLevel = ...,
    channel_filter: str | None = ...,
) -> None: ...
def disable_python_logging() -> None: ...
