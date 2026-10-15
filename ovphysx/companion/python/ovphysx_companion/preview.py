# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PREVIEW-001
# @covers AC-1 AC-2 AC-4 AC-5 AC-6 AC-7
"""Inspector's independent native attachment. Imports neither USD nor ovstage.

Populate a distinct ovstage snapshot, seal its ordinal, then pass it together
with the existing PhysX settings source. The stage is retained through close;
explicitly destroying it while attached violates the source lifetime contract.
"""
import atexit
import ctypes as C
from dataclasses import dataclass
import threading
import warnings
import weakref
from .native import (CompanionError, Status, Identity, SceneStatistics, _Identity,
                     _Statistics, _GetStatistics, _Close, _GetIdentity, _check)


class _Desc(C.Structure):
    _fields_ = [("size", C.c_uint32), ("version", C.c_uint32), ("source_instance", C.c_uint64),
                ("stage", C.c_void_p), ("dictionary", C.c_void_p), ("read_ordinal", C.c_uint64),
                ("overrides", C.c_uint32), ("threads", C.c_uint32), ("cuda_device", C.c_int32),
                ("gpu_pipeline", C.c_int32), ("solver", C.c_int32),
                ("excluded_path_count", C.c_uint32), ("excluded_paths", C.POINTER(C.c_char_p))]


class _Pose(C.Structure):
    _fields_ = [("position", C.c_float * 3), ("orientation", C.c_float * 4)]


class _PreviewApi(C.Structure):
    _fields_ = [("size", C.c_uint32), ("version", C.c_uint32),
                ("open", C.CFUNCTYPE(C.c_uint32, C.POINTER(_Desc), C.POINTER(C.c_uint64))),
                ("close", _Close), ("identity", _GetIdentity),
                ("simulate", C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.c_float)),
                ("fetch", _Close), ("statistics", _GetStatistics),
                ("rigid_pose", C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.c_char_p, C.POINTER(_Pose))),
                ("update", C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.c_uint64, C.c_uint64))]


@dataclass(frozen=True)
class Pose:
    """World pose in stage units; orientation is xyzw. Values own their storage."""
    position: tuple
    orientation: tuple


_live = weakref.WeakSet()
_failed_close = []
# Stages of open previews, pinned from module scope as core owners pin theirs. A
# cyclic-GC pass can otherwise finalize the Stage, destroying the native stage,
# before this preview's finalizer closes the native preview reading it. Released
# only after a successful native close.
_pinned_stages = {}
_exit_registered = False


def _close_at_exit():
    for preview in list(_live) + list(_failed_close):
        try:
            preview.close()
        except Exception as error:
            warnings.warn(f"Preview cleanup failed: {error}", RuntimeWarning)
    _failed_close.clear()


def _path(value):
    if not isinstance(value, str) or not value.startswith("/") or "\0" in value:
        raise ValueError("Expected an absolute path without embedded NUL")
    return value.encode("utf-8")


def _ordinal(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value < 2**64:
        raise ValueError("Expected a nonzero uint64 sealed ordinal")
    return value


class PreviewSession:
    """Own a preview attachment independently of the bootstrap PhysX instance.

    `simulate` / `fetch` delimit one pending interval. `step` performs both.
    Queries require idle state. `close` drains B only, even while playback A has
    uncollected work. Use as a context manager or close explicitly. Resource
    overrides are copied at attach; they never temporarily rewrite defaults.
    update() consumes sealed inputs without stepping or promising immediate
    effective mass realization; rebuild a stopped preview when that is needed.
    Fresh invalid ordinal ranges report stale input without retiring this owner.
    """
    def __init__(self, physics, stage, *, read_ordinal, threads=None, cuda_device=None,
                 gpu_pipeline=None, solver=None, excluded_paths=()):
        global _exit_registered
        self._handle = C.c_uint64()
        self._lifecycle_acquired = False
        self._lock = threading.Lock()
        self._library = physics._lib
        self._stage = stage
        api = _PreviewApi()
        try:
            acquire = self._library.ovphysx_companion_preview_acquire
        except AttributeError as error:
            raise CompanionError(Status.UNAVAILABLE, "native preview (use an explicit worker)") from error
        acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_PreviewApi)]
        acquire.restype = C.c_uint32
        _check(acquire(1, C.sizeof(api), C.byref(api)), "acquire_preview")
        if api.size != C.sizeof(api) or api.version != 1 or not all(
                (api.open, api.close, api.identity, api.simulate, api.fetch, api.statistics, api.rigid_pose, api.update)):
            raise CompanionError(Status.ABI_MISMATCH, "preview table")
        self._api = api
        handle = getattr(physics, "_omni_physx_sdk_handle", None)
        if not handle or not handle.value:
            raise CompanionError(Status.NOT_FOUND, "preview bootstrap instance")
        desc = _Desc(size=C.sizeof(_Desc), version=1, source_instance=handle.value,
                     stage=(stage.handle() if hasattr(stage, "handle") else C.cast(stage._inst, C.c_void_p).value),
                     read_ordinal=_ordinal(read_ordinal))
        for bit, field, value, low, high in ((1, "threads", threads, 1, 255),
                                           (2, "cuda_device", cuda_device, -1, 2**31-1),
                                           (4, "gpu_pipeline", gpu_pipeline, -1, 1),
                                           (8, "solver", solver, -1, 1)):
            if value is not None:
                if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
                    raise ValueError(f"{field} must be an integer in {low}..{high}")
                desc.overrides |= bit
                setattr(desc, field, value)
        paths = (C.c_char_p * len(excluded_paths))(*map(_path, excluded_paths))
        desc.excluded_paths = paths
        desc.excluded_path_count = len(paths)
        # Same Python process token used by core owners and borrowed read arrays.
        # This lazy import does not affect base companion imports.
        from ovphysx.api import _acquire_process_lifecycle, _release_process_lifecycle
        self._release_lifecycle = _release_process_lifecycle
        _acquire_process_lifecycle(self)
        try:
            _check(api.open(C.byref(desc), C.byref(self._handle)), "open_preview")
        except BaseException:
            self._lifecycle_acquired = False
            self._release_lifecycle()
            raise
        _pinned_stages[id(self)] = stage
        _live.add(self)
        if not _exit_registered:
            atexit.register(_close_at_exit)  # after core registration: preview drains first
            _exit_registered = True

    def _call(self, operation, *args):
        if not self._lock.acquire(blocking=False):
            raise CompanionError(Status.BUSY, operation)
        try:
            _check(getattr(self._api, operation)(self._handle.value, *args), operation)
        finally:
            self._lock.release()

    @property
    def identity(self):
        result = _Identity()
        self._call("identity", C.byref(result))
        return Identity(result.instance, result.attachment)

    def simulate(self, dt):
        from .native import _host_scalar
        self._call("simulate", _host_scalar(dt))

    def fetch(self):
        self._call("fetch")

    def step(self, dt):
        self.simulate(dt)
        self.fetch()

    def rigid_pose(self, path):
        result = _Pose()
        self._call("rigid_pose", _path(path), C.byref(result))
        return Pose(tuple(result.position), tuple(result.orientation))

    def scene_statistics(self, path):
        result = _Statistics()
        self._call("statistics", _path(path), C.sizeof(result), C.byref(result))
        return SceneStatistics(Identity(result.owner.instance, result.owner.attachment),
                               result.dynamic_bodies, result.active_dynamic_bodies,
                               result.static_bodies, result.kinematic_bodies,
                               result.articulations, result.active_constraints)

    def update(self, from_ordinal, to_ordinal):
        self._call("update", _ordinal(from_ordinal), _ordinal(to_ordinal))

    def close(self):
        if not self._lock.acquire(blocking=False):
            raise CompanionError(Status.BUSY, "close")
        try:
            if self._handle.value:
                _check(self._api.close(self._handle.value), "close_preview")
                self._handle.value = 0
            _pinned_stages.pop(id(self), None)
            self._stage = None
            _live.discard(self)
            if self._lifecycle_acquired:
                self._lifecycle_acquired = False  # release consumes even if shutdown raises
                self._release_lifecycle()
        finally:
            self._lock.release()

    def __enter__(self):
        self.identity
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        if getattr(self, "_handle", None) is not None and (self._handle.value or self._lifecycle_acquired):
            try:
                self.close()
            except Exception:
                # Retain the stage/library/token until explicit process-exit retry.
                _failed_close.append(self)
