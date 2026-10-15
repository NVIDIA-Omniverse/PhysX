# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-NATIVE-ABI-001
# @covers AC-1 AC-3 AC-4
# @implements REQ-COMP-JOINT-001
# @covers AC-2 AC-4
# @implements REQ-COMP-GEOMETRY-001
# @covers AC-1 AC-4
"""Load compiled companion plugins; no solver/contact callbacks run in Python."""
import atexit
import ctypes as C
import os
from pathlib import Path
import threading
import warnings
import weakref
from .native import CompanionError, Status, Identity, _check


class _Api(C.Structure):
    _fields_ = [("size", C.c_uint32), ("version", C.c_uint32),
                ("load", C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.c_char_p, C.POINTER(C.c_uint64))),
                ("close", C.CFUNCTYPE(C.c_uint32, C.c_uint64)),
                ("joint_control", C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.c_uint64, C.c_char_p,
                                               C.c_uint32, C.POINTER(C.c_uint32)))]


class _CapabilitiesApi(C.Structure):
    _fields_ = [('size',C.c_uint32),('version',C.c_uint32),
                ('query',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(C.c_uint32)))]


_NATIVE_CAPABILITIES = {'joint_cpu':1, 'geometry_cpu':2, 'joint_refresh':4,
                        'raycast':8, 'overlap':16, 'sweep':32, 'debug':64}
# Provider v1 deliberately exposes no GPU, DirectGPU, articulation DOF, SDF,
# mesh-merge, particle/deformable contacts or tensor-integration capability.

_live = weakref.WeakSet()
_retained = []
_registered = False


def _cleanup():
    for module in list(_live) + list(_retained):
        try:
            module.close()
        except Exception as error:
            warnings.warn(f"Native plugin retained because retirement failed: {error}", RuntimeWarning)


class NativePlugin:
    """Retain a native module and runtime services until safe retirement.

    Register schema resources before population separately. Load the plugin before
    attaching the scene. Close all its objects/attachments before this module;
    Busy preserves the handle so close can be retried. Keep this object alive
    while used and close explicitly (or use a context manager).
    """
    def __init__(self, physics, path):
        global _registered
        path = os.fspath(path)
        if not isinstance(path, str) or "\0" in path or not Path(path).is_absolute():
            raise ValueError("Native plugin path must be absolute and contain no NUL")
        self._handle = C.c_uint64()
        self._lifecycle_acquired = False
        self._lock = threading.Lock()
        self._library = physics._lib
        self._api = _Api()
        try:
            acquire = self._library.ovphysx_companion_native_acquire
        except AttributeError as error:
            raise CompanionError(Status.UNAVAILABLE, "native plugin table") from error
        acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_Api)]
        acquire.restype = C.c_uint32
        _check(acquire(1, C.sizeof(_Api), C.byref(self._api)), "native plugin table")
        if self._api.size != C.sizeof(_Api) or self._api.version != 1 or not self._api.load or not self._api.close or not self._api.joint_control:
            raise CompanionError(Status.ABI_MISMATCH, "native plugin table")
        handle = getattr(physics, "_omni_physx_sdk_handle", None)
        if not handle or not handle.value:
            raise CompanionError(Status.NOT_FOUND, "native plugin bootstrap")
        from ovphysx.api import _acquire_process_lifecycle, _release_process_lifecycle
        self._release_lifecycle = _release_process_lifecycle
        _acquire_process_lifecycle(self)
        try:
            _check(self._api.load(handle.value, path.encode("utf-8"), C.byref(self._handle)), "load native plugin")
        except BaseException:
            self._lifecycle_acquired = False
            self._release_lifecycle()
            raise
        _live.add(self)
        if not _registered:
            atexit.register(_cleanup)
            _registered = True

    @property
    def capabilities(self):
        """Discover the loaded provider's optional operations; closed handles fail."""
        table = _CapabilitiesApi()
        try:
            acquire = self._library.ovphysx_companion_native_capabilities_acquire
        except AttributeError as error:
            raise CompanionError(Status.UNAVAILABLE,'native capability table') from error
        acquire.argtypes = [C.c_uint32,C.c_uint32,C.POINTER(_CapabilitiesApi)]
        acquire.restype = C.c_uint32
        _check(acquire(1,C.sizeof(table),C.byref(table)),'native capabilities')
        if table.size != C.sizeof(table) or table.version != 1 or not table.query:
            raise CompanionError(Status.ABI_MISMATCH,'native capability table')
        value = C.c_uint32()
        with self._lock:
            _check(table.query(self._handle.value,C.byref(value)),'native capabilities')
        return frozenset(name for name,bit in _NATIVE_CAPABILITIES.items() if value.value & bit)

    def require(self, operation):
        """Raise UNAVAILABLE before a host action uses an absent provider mode."""
        if operation not in self.capabilities:
            raise CompanionError(Status.UNAVAILABLE,'native plugin '+str(operation))

    def _joint(self, owner, path, action, flags=0):
        if not isinstance(owner, Identity) or not 0 < owner.attachment < 2**64:
            raise ValueError("Expected an exact native attachment identity")
        if not isinstance(path, str) or not path.startswith("/") or "\0" in path:
            raise ValueError("Expected an absolute authored joint path")
        if not self._lock.acquire(blocking=False):
            raise CompanionError(Status.BUSY, "joint control")
        try:
            value = C.c_uint32(flags)
            _check(self._api.joint_control(self._handle.value, owner.attachment, path.encode(), action,
                                           C.byref(value)), "joint control")
            return value.value
        finally:
            self._lock.release()

    def mark_joint_dirty(self, owner, path):
        self._joint(owner, path, 0)

    def set_joint_always_update(self, owner, path, enabled):
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be bool")
        self._joint(owner, path, 1, int(enabled))

    def joint_always_update(self, owner, path):
        return bool(self._joint(owner, path, 2))

    def refresh_joint(self, owner, path):
        """Copy current sealed-source parameters into the idle native joint.

        Update the owner's input ordinal first. The plugin may reject invalid
        values without changing its state. Success dirties data for next step.
        """
        self._joint(owner, path, 3)

    def close(self):
        if not self._lock.acquire(blocking=False):
            raise CompanionError(Status.BUSY, "close native plugin")
        try:
            if self._handle.value:
                _check(self._api.close(self._handle.value), "close native plugin")
                self._handle.value = 0
            _live.discard(self)
            if self._lifecycle_acquired:
                self._lifecycle_acquired = False
                self._release_lifecycle()
        finally:
            self._lock.release()

    def __enter__(self):
        if not self._handle.value:
            raise CompanionError(Status.NOT_FOUND, "closed native plugin")
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        if getattr(self, "_handle", None) is not None and (self._handle.value or self._lifecycle_acquired):
            try:
                self.close()
            except Exception:
                _retained.append(self)
