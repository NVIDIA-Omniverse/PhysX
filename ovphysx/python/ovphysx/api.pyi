# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-DETACH-001
# @covers AC-3 AC-4
# @implements REQ-PYTHON-FRAME-001
# @covers AC-1
# @implements REQ-CAPI-CONTACT-003
# @covers AC-1 AC-2 AC-3 AC-4

# @implements REQ-PYTHON-TASK-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-8

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
# @implements REQ-CAPI-CONTACT-002
# @covers AC-2 AC-3 AC-4
# @implements REQ-CAPI-CONTACT-001
# @covers AC-5 AC-6 AC-7
# @implements REQ-CAPI-SDF-001
# @covers AC-4

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, NamedTuple, TypedDict, Generic, TypeVar
from enum import IntFlag
# The standalone stub check excludes this pinned runtime dependency.
from ovstage import PathDictionary, Stage  # pyright: ignore[reportMissingImports]

from typing_extensions import deprecated

import warp as wp

from .config import OmniPvdDestination, PhysXConfig
from .dlpack import DLDataType, DLDevice
from .types import (
    ApiStatus,
    ArticulationKinematicFlag,
    ConfigKey,
    ContactSelection,
    GlobalConfigKey,
    LogLevel,
    ObjectScope,
    ObjectType,
    SceneQueryGeometryType,
    SceneQueryMode,
    SimObjectType,
)


@dataclass(frozen=True, slots=True)
class Task:
    status: ApiStatus
    op_index: int
    instance: int


class TaskError(RuntimeError):
    def __init__(self, errors: tuple[tuple[Task, str], ...]) -> None: ...
    @property
    def errors(self) -> tuple[tuple[Task, str], ...]: ...


class TaskTimeoutError(TimeoutError):
    def __init__(self, errors: tuple[tuple[Task, str], ...], pending_task: Task) -> None: ...
    @property
    def errors(self) -> tuple[tuple[Task, str], ...]: ...
    @property
    def pending_task(self) -> Task: ...


T = TypeVar("T")

class Operation(Generic[T]):
    @property
    def tasks(self) -> tuple[Task, ...]: ...
    @property
    def task(self) -> Task: ...
    @property
    def done(self) -> bool: ...
    @property
    def result(self) -> T: ...
    def wait(self, *, timeout_ns: int | None = ...) -> T: ...

@dataclass(frozen=True, slots=True)
class QueryMetadata:
    total_prim_count: int
    attributes: tuple[int, ...]

class ReadOmission(IntFlag):
    NONE = 0
    UNSUPPORTED_ATTRIBUTE = 1
    UNSUPPORTED_CONFIGURATION = 2
    NOT_READY = 4

@dataclass(frozen=True, slots=True)
class ReadAttributeResult:
    attribute: int
    group_count: int
    omission_reasons: ReadOmission

@dataclass(frozen=True, slots=True)
class ReadMetadata:
    total_prim_count: int
    attributes: tuple[ReadAttributeResult, ...]

@dataclass(frozen=True, slots=True)
class WriteAttributeResult:
    attribute: int
    group_count: int

@dataclass(frozen=True, slots=True)
class WriteMetadata:
    total_prim_count: int
    attributes: tuple[WriteAttributeResult, ...]

class Query:
    object_type: SimObjectType
    metadata: QueryMetadata | None
    @property
    def handle(self) -> int: ...
    @property
    def dictionary(self) -> PathDictionary: ...
    def close(self) -> Operation[None]: ...
    def release(self) -> Operation[None]: ...
    def close_sync(self) -> None: ...
    def __enter__(self) -> Query: ...
    def __exit__(self, *exc: object) -> None: ...


class SceneQueryHit(TypedDict):
    collision: int
    rigid_body: int
    proto_index: int
    normal: tuple[float, float, float]
    position: tuple[float, float, float]
    distance: float
    face_index: int
    material: int


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
    metadata: ReadMetadata | None
    @property
    def dictionary(self) -> PathDictionary: ...
    def fetch_next(self) -> Operation[ReadGroup | None]: ...
    def close(self) -> Operation[None]: ...
    def release(self) -> Operation[None]: ...
    def close_sync(self) -> None: ...
    def __enter__(self) -> ReadResult: ...
    def __exit__(self, *exc: object) -> None: ...


class WriteGroup(NamedTuple):
    prim_list: int
    prim_offset: int
    prim_count: int
    tensors: list[wp.array]
    attribute_index: int
    attribute: int


class WriteSession:
    groups: list[WriteGroup]
    metadata: WriteMetadata | None
    def fetch_next(self) -> Operation[WriteGroup | None]: ...
    def commit(self, group: WriteGroup, cuda_stream: int = ..., cuda_wait_event: int = ...,
               *, event_owner: Any = ...) -> Operation[None]:
        """Publish with producer dependencies; both nonzero handles are honored.

        Keep handles valid through completion. Stream 1 uses the caller's current
        CUDA context; these handles do not select PhysX's execution stream.
        event_owner retains Python owners through collection.
        """
        ...
    def commit_sync(self, group: WriteGroup, cuda_stream: int = ..., cuda_wait_event: int = ...,
                    *, event_owner: Any = ...) -> None: ...
    def close(self) -> Operation[None]: ...
    def release(self) -> Operation[None]: ...
    def close_sync(self) -> None: ...
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


@deprecated("The contact-binding API is deprecated and retained for compatibility.")
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
    @deprecated("The contact-binding API is deprecated and retained for compatibility.")
    def read_net_normal_forces(self, output: Any) -> None: ...
    @deprecated("The contact-binding API is deprecated and retained for compatibility.")
    def read_net_friction_forces(self, output: Any) -> None: ...
    @deprecated("The contact-binding API is deprecated and retained for compatibility.")
    def read_normal_force_matrix(self, output: Any) -> None: ...
    @deprecated("The contact-binding API is deprecated and retained for compatibility.")
    def read_friction_force_matrix(self, output: Any) -> None: ...
    @deprecated("Use read_net_normal_forces(); returns normal forces only")
    def read_net_forces(self, output: Any) -> None: ...
    @deprecated("Use read_normal_force_matrix(); returns normal forces only")
    def read_force_matrix(self, output: Any) -> None: ...
    @deprecated("The contact-binding API is deprecated and retained for compatibility.")
    def read_normal_contact_data(
        self,
        contact_forces: Any,
        positions: Any,
        normals: Any,
        separations: Any,
        counts: Any,
        start_indices: Any,
    ) -> int: ...
    @deprecated("Use read_normal_contact_data(); forces contain only normal components")
    def read_contact_data(
        self,
        contact_forces: Any,
        positions: Any,
        normals: Any,
        separations: Any,
        counts: Any,
        start_indices: Any,
    ) -> int: ...
    def read_raw_contact_data(
        self,
        contact_forces: Any,
        positions: Any,
        normals: Any,
        separations: Any,
        sensor_layout: Any,
        actor_ids: Any,
    ) -> int: ...
    def get_other_actor_paths_from_ids(self, ids_array: Any) -> list[str]: ...
    @deprecated("The contact-binding API is deprecated and retained for compatibility.")
    def read_friction_contact_data(
        self,
        friction_forces: Any,
        friction_points: Any,
        counts: Any,
        start_indices: Any,
    ) -> int: ...
    @deprecated("Use read_friction_contact_data()")
    def read_friction_data(
        self,
        friction_forces: Any,
        friction_points: Any,
        counts: Any,
        start_indices: Any,
    ) -> int: ...
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
    def clone(
        self,
        source_path: str,
        target_paths: list[str],
        anchor_transforms: list[Sequence[float]] | None = ...,
        env_ids: list[int] | None = ...,
    ) -> Task: ...
    @deprecated("Object-type lookup is deprecated.")
    def get_object_type(self, prim_path: str) -> ObjectType: ...
    def start_recording(self, destination: OmniPvdDestination) -> None: ...
    def stop_recording(self) -> None: ...
    def is_recording(self) -> bool: ...
    def step(self, dt: float) -> Task: ...
    def step_sync(self, dt: float) -> None: ...
    def update_articulations_kinematic(
        self,
        *,
        flags: ArticulationKinematicFlag = ...,
        scope: ObjectScope = ...,
        prim_paths: list[int] | None = ...,
        pattern: str | None = ...,
    ) -> None:
        """Wait for the native update task; raise admission or execution failures."""
        ...
    def evaluate_sdf_shapes(
        self,
        points: Any,
        out: Any,
        *,
        prim_paths: list[int] | None = ...,
        pattern: str | None = ...,
        cuda_stream: int = ...,
        cuda_wait_event: int = ...,
    ) -> None:
        """Evaluate SDF_SHAPE query shapes at [N, Q, 3] points into [N, Q, 4] out; wait for the task."""
        ...
    def wait_task(self, task: Task, *, timeout_ns: int | None = ...) -> None: ...
    def wait_all(self, *, timeout_ns: int | None = ...) -> None: ...
    def subscribe_object_changes(
        self,
        *,
        on_created: Any = ...,
        on_destroyed: Any = ...,
        on_all_destroyed: Any = ...,
    ) -> int: ...
    def unsubscribe_object_changes(self, subscription_id: int) -> None: ...
    def attach_ovstage(self, stage: Stage, *, read_ordinal: int = ...) -> Task: ...
    def update_from_ovstage(self, from_ordinal: int, to_ordinal: int) -> Task: ...
    def detach_ovstage(self) -> Task: ...
    def is_stage_attached(self, stage: Any = None) -> bool: ...
    def query(self, object_type: SimObjectType, *, scope: ObjectScope = ...,
              prim_paths: Sequence[int] | None = ..., pattern: str | None = ...,
              contacts: Sequence[ContactSelection] | None = ..., metadata: bool = ...) -> Operation[Query]: ...
    def read(self, query: Query, attributes: Sequence[str | int], *,
             metadata: bool = ...) -> Operation[ReadResult]: ...
    def write(self, query: Query, attributes: Sequence[str | int], *,
              metadata: bool = ...) -> Operation[WriteSession]: ...
    def read_sync(self, object_type: SimObjectType, attribute_names: Sequence[str | int], *,
                  scope: ObjectScope = ..., prim_paths: Sequence[int] | None = ...,
                  pattern: str | None = ..., contacts: Sequence[ContactSelection] | None = ...,
                  metadata: bool = ...) -> ReadResult: ...
    def read_tokens_sync(self, object_type: SimObjectType, attribute_tokens: Sequence[int], *,
                         scope: ObjectScope = ..., prim_paths: Sequence[int] | None = ...,
                         pattern: str | None = ..., contacts: Sequence[ContactSelection] | None = ...,
                         metadata: bool = ...) -> ReadResult: ...
    def write_sync(self, object_type: SimObjectType, attributes: str | int | Sequence[str | int], *,
                   scope: ObjectScope = ..., prim_paths: Sequence[int] | None = ...,
                   pattern: str | None = ..., metadata: bool = ..., values: Sequence[int | float] | None = ...) -> WriteSession: ...
    def set_config(self, key: ConfigKey, value: bool | int | float | str) -> None: ...
    def get_config(self, key: ConfigKey) -> bool | int | float | str | None: ...
    def destroy(self) -> None:
        """Reject live read-array borrows before teardown; drop aliases and retry."""
        ...
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
    @deprecated("Scene queries are deprecated.")
    def raycast(
        self,
        origin: Sequence[float],
        direction: Sequence[float],
        distance: float,
        mode: SceneQueryMode = ...,
        both_sides: bool = ...,
    ) -> list[SceneQueryHit]: ...
    @deprecated("Scene queries are deprecated.")
    def sweep(
        self,
        geometry_type: SceneQueryGeometryType,
        direction: Sequence[float],
        distance: float,
        mode: SceneQueryMode = ...,
        both_sides: bool = ...,
        **kwargs: float | str | Sequence[float],
    ) -> list[SceneQueryHit]: ...
    @deprecated("Scene queries are deprecated.")
    def overlap(
        self,
        geometry_type: SceneQueryGeometryType,
        mode: SceneQueryMode = ...,
        **kwargs: float | str | Sequence[float],
    ) -> list[SceneQueryHit]: ...
    @deprecated("Scene queries are deprecated.")
    def get_scene_query_paths_from_ids(self, ids: Sequence[int]) -> list[str]: ...
    @deprecated("The contact-binding API is deprecated and retained for compatibility.")
    def create_contact_binding(
        self,
        sensor_patterns: list[str],
        filter_patterns: list[str] | None = ...,
        filters_per_sensor: int = ...,
        max_contact_data_count: int = ...,
    ) -> ContactBinding: ...
    @deprecated("SDF views are deprecated.")
    def create_sdf_view(self, pattern: str, max_query_points: int) -> SdfView: ...


def set_global_config(
    key: GlobalConfigKey, value: bool | int | float | str
) -> None: ...
def get_global_config(
    key: GlobalConfigKey,
) -> bool | int | float | str | None: ...
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
