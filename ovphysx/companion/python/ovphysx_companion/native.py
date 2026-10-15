# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-BUFFERS-001
# @covers AC-1 AC-2 AC-3

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2 AC-3

# @implements REQ-COMP-API-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-OWNER-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-DEBUG-001
# @covers AC-2

"""ctypes companion tables acquired from an existing PhysX object's library.

This module does not load a shared library, import USD or start simulation.
Callers serialize operations with the corresponding core instance. Close sessions
before destroying that instance. Returned records are copies with no native views.
"""

import atexit
import weakref
import warnings
import ctypes as C
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from numbers import Real


class Status(IntEnum):
    OK = 0
    INVALID_ARGUMENT = 1
    ABI_MISMATCH = 2
    UNAVAILABLE = 3
    NOT_FOUND = 4
    STALE = 5
    BUSY = 6
    ERROR = 7


class Capability(IntFlag):
    BORROWED_SESSION = 1
    SCENE_STATISTICS = 2
    PRIMITIVE_MASS = 4
    POINT_MATCHING = 8


class CompanionError(RuntimeError):
    def __init__(self, status, operation):
        self.status = Status(status)
        super().__init__(f"Companion {operation}: {self.status.name}")


class _Identity(C.Structure):
    _fields_ = [("instance", C.c_uint64), ("attachment", C.c_uint64)]


class _Statistics(C.Structure):
    _fields_ = [("size", C.c_uint32), ("version", C.c_uint32), ("owner", _Identity),
                ("dynamic_bodies", C.c_uint32), ("active_dynamic_bodies", C.c_uint32),
                ("static_bodies", C.c_uint32), ("kinematic_bodies", C.c_uint32),
                ("articulations", C.c_uint32), ("active_constraints", C.c_uint32)]


class _MassShape(C.Structure):
    _fields_ = [("kind", C.c_uint32), ("dimensions", C.c_float * 3),
                ("position", C.c_float * 3), ("rotation", C.c_float * 4), ("density", C.c_float)]


class _MassResult(C.Structure):
    _fields_ = [("size", C.c_uint32), ("version", C.c_uint32), ("mass", C.c_float),
                ("volume", C.c_float), ("center_of_mass", C.c_float * 3),
                ("diagonal_inertia", C.c_float * 3), ("principal_axes", C.c_float * 4),
                ("bounds_min", C.c_float * 3), ("bounds_max", C.c_float * 3)]


_ComputeMass = C.CFUNCTYPE(C.c_uint32, C.POINTER(_MassShape), C.c_uint32,
                         C.c_uint32, C.POINTER(_MassResult), C.POINTER(C.c_uint32))


_Open = C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.POINTER(C.c_uint64))
_Close = C.CFUNCTYPE(C.c_uint32, C.c_uint64)
_GetIdentity = C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.POINTER(_Identity))
_GetStatistics = C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.c_char_p, C.c_uint32, C.POINTER(_Statistics))


_MatchPoints = C.CFUNCTYPE(C.c_uint32, C.POINTER(C.c_float), C.c_uint32,
                          C.POINTER(C.c_float), C.c_uint32, C.c_float,
                          C.POINTER(C.c_int32), C.c_uint32)


class _Api(C.Structure):
    _fields_ = [("size", C.c_uint32), ("version", C.c_uint32), ("capabilities", C.c_uint64),
                ("open_session", _Open), ("close_session", _Close),
                ("get_identity", _GetIdentity), ("scene_statistics", _GetStatistics),
                ("compute_primitive_mass", _ComputeMass), ("match_points", _MatchPoints)]


@dataclass(frozen=True)
class Identity:
    instance: int
    attachment: int


@dataclass(frozen=True)
class SceneStatistics:
    owner: Identity
    dynamic_bodies: int
    active_dynamic_bodies: int
    static_bodies: int
    kinematic_bodies: int
    articulations: int
    active_constraints: int


def _check(status, operation):
    if status != Status.OK:
        raise CompanionError(status, operation)


def _acquire(library):
    api = _Api()
    try:
        acquire = library.ovphysx_companion_acquire
    except AttributeError as exc:
        raise CompanionError(Status.UNAVAILABLE, "acquire (core has no companion bridge)") from exc
    acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_Api)]
    acquire.restype = C.c_uint32
    _check(acquire(1, C.sizeof(_Api), C.byref(api)), "acquire")
    if api.version != 1 or api.size != C.sizeof(_Api):
        raise CompanionError(Status.ABI_MISMATCH, "table layout")
    return api


_live_sessions = weakref.WeakSet()
_retained_sessions = []
_cleanup_registered = False


def _cleanup_sessions():
    for session in list(_live_sessions) + list(_retained_sessions):
        try:
            session.close()
        except Exception as error:
            warnings.warn(f'Tools session cleanup failed: {error}', RuntimeWarning)


class ToolsSession:
    """Borrow a live core attachment, retaining its PhysX object and native owner.

    Detach/reattach makes the session stale; close it and acquire a new session.
    `with ToolsSession(physics) as tools:` is the preferred lifetime pattern.
    """

    def __init__(self, physics):
        global _cleanup_registered
        self._owner = physics
        self._handle = C.c_uint64()
        self._library = physics._lib
        self._api = _acquire(self._library)
        if not self._api.capabilities & Capability.BORROWED_SESSION or not all(
            (self._api.open_session, self._api.close_session, self._api.get_identity)
        ):
            raise CompanionError(Status.UNAVAILABLE, "borrowed session")
        handle = getattr(physics, "_omni_physx_sdk_handle", None)
        if handle is None or not handle.value:
            raise CompanionError(Status.NOT_FOUND, "core owner")
        _check(self._api.open_session(handle.value, C.byref(self._handle)), "open_session")
        _live_sessions.add(self)
        if not _cleanup_registered:
            atexit.register(_cleanup_sessions)
            _cleanup_registered = True

    @property
    def capabilities(self):
        return Capability(self._api.capabilities)

    @property
    def identity(self):
        result = _Identity()
        _check(self._api.get_identity(self._handle.value, C.byref(result)), "get_identity")
        return Identity(result.instance, result.attachment)

    def scene_statistics(self, scene_path):
        if not self.capabilities & Capability.SCENE_STATISTICS or not self._api.scene_statistics:
            raise CompanionError(Status.UNAVAILABLE, "scene_statistics")
        if not isinstance(scene_path, str) or "\0" in scene_path:
            raise ValueError("scene_path must be a path string without embedded NUL")
        result = _Statistics()
        _check(self._api.scene_statistics(self._handle.value, scene_path.encode("utf-8"),
                                          C.sizeof(result), C.byref(result)), "scene_statistics")
        if result.size != C.sizeof(result) or result.version != 1:
            raise CompanionError(Status.ABI_MISMATCH, "statistics layout")
        return SceneStatistics(Identity(result.owner.instance, result.owner.attachment),
                               result.dynamic_bodies, result.active_dynamic_bodies,
                               result.static_bodies, result.kinematic_bodies,
                               result.articulations, result.active_constraints)

    def close(self):
        if self._handle.value:
            _check(self._api.close_session(self._handle.value), "close_session")
            self._handle.value = 0
        self._owner = None
        _live_sessions.discard(self)
        if self in _retained_sessions:
            _retained_sessions.remove(self)

    def __enter__(self):
        self.identity
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        # The registry releases synchronously without entering the runtime.
        # Strong owner retention prevents normal finalization from preceding us.
        if getattr(self, "_handle", None) is not None and self._handle.value:
            try:
                self.close()
            except Exception:
                _retained_sessions.append(self)


class PrimitiveKind(IntEnum):
    BOX = 0
    SPHERE = 1
    CAPSULE = 2


@dataclass(frozen=True)
class MassShape:
    """SI primitive in body-local coordinates; rotation is xyzw, capsule axis X."""
    kind: PrimitiveKind
    dimensions: tuple
    density: float = 1000.0
    position: tuple = (0.0, 0.0, 0.0)
    rotation: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class MassProperties:
    mass: float
    volume: float
    center_of_mass: tuple
    diagonal_inertia: tuple
    principal_axes: tuple
    bounds_min: tuple
    bounds_max: tuple


def _host_sequence(value):
    # Never ask a device scalar to convert itself to a Python float: that could
    # hide a synchronization/transfer in an editor operation.
    if hasattr(value, "__cuda_array_interface__") or getattr(value, "is_cuda", False):
        raise TypeError("Expected ready CPU data; transfer device data explicitly")
    if hasattr(value, "__dlpack_device__") and value.__dlpack_device__()[0] != 1:
        raise TypeError("Expected ready CPU data; transfer device data explicitly")
    return tuple(value)


def _host_scalar(value):
    if not isinstance(value, Real):
        raise TypeError("Expected ready CPU scalar data; convert tensors explicitly")
    return float(value)


def _snapshot_mass_shapes(shapes):
    result = []
    for shape in _host_sequence(shapes):
        dimensions = tuple(_host_scalar(v) for v in _host_sequence(shape.dimensions))
        position = tuple(_host_scalar(v) for v in _host_sequence(shape.position))
        rotation = tuple(_host_scalar(v) for v in _host_sequence(shape.rotation))
        if len(dimensions) != 3 or len(position) != 3 or len(rotation) != 4:
            raise ValueError("Expected 3 dimensions, 3 position components and 4 rotation components")
        result.append(MassShape(PrimitiveKind(shape.kind), dimensions, _host_scalar(shape.density), position, rotation))
    if not result or len(result) > 0xFFFFFFFF:
        raise ValueError("A nonempty uint32-sized shape sequence is required")
    return tuple(result)


def compute_mass_properties(library, shapes):
    """Compute copied SI results without an attached scene or initialized runtime.

    Pass the already loaded core CDLL (e.g. physics._lib). This function never
    loads another library. Primitive volumes add, including overlaps. Authored
    overrides and mesh inputs are not accepted by this primitive operation.
    """
    api = _acquire(library)
    if not api.capabilities & Capability.PRIMITIVE_MASS or not api.compute_primitive_mass:
        raise CompanionError(Status.UNAVAILABLE, "primitive mass")
    shapes = _snapshot_mass_shapes(shapes)
    records = [_MassShape(shape.kind, (C.c_float * 3)(*shape.dimensions),
                          (C.c_float * 3)(*shape.position),
                          (C.c_float * 4)(*shape.rotation), shape.density) for shape in shapes]
    inputs = (_MassShape * len(records))(*records)
    result = _MassResult()
    invalid = C.c_uint32(0xFFFFFFFF)
    status = api.compute_primitive_mass(inputs, len(inputs), C.sizeof(result),
                                       C.byref(result), C.byref(invalid))
    if status != Status.OK:
        error = CompanionError(status, f"compute_mass_properties (shape {invalid.value})")
        error.invalid_shape = None if invalid.value == 0xFFFFFFFF else invalid.value
        raise error
    if result.size != C.sizeof(result) or result.version != 1:
        raise CompanionError(Status.ABI_MISMATCH, "mass result layout")
    return MassProperties(result.mass, result.volume, tuple(result.center_of_mass),
                          tuple(result.diagonal_inertia), tuple(result.principal_axes),
                          tuple(result.bounds_min), tuple(result.bounds_max))


def match_points(library, source, target, distance):
    """Copied nearest target indices (-1 for unmatched), in a shared frame/units.

    Coordinates are converted to float32. Distance is inclusive, ties pick the
    lowest target index and several source vertices can choose the same target.
    Empty inputs are supported. NaNs, infinities and negative distances fail.
    """
    api = _acquire(library)
    if not api.capabilities & Capability.POINT_MATCHING or not api.match_points:
        raise CompanionError(Status.UNAVAILABLE, "match_points")
    def capture(points):
        points = tuple(tuple(_host_scalar(v) for v in _host_sequence(p)) for p in _host_sequence(points))
        if len(points) > 0x7FFFFFFF or any(len(p) != 3 for p in points):
            raise ValueError("Expected a sequence of XYZ points with int32-sized count")
        return len(points), (C.c_float * (len(points) * 3))(*(v for p in points for v in p))
    n, a = capture(source)
    m, b = capture(target)
    result = (C.c_int32 * n)()
    _check(api.match_points(a, n, b, m, _host_scalar(distance), result, n), "match_points")
    return tuple(result)
