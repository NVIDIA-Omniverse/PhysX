# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import ctypes
import gc
import weakref
from threading import Thread

from ovphysx.dlpack import (
    DLDataTypeCode,
    DLDeviceType,
    DLManagedTensor,
    DLTensor,
    ManagedDLTensor,
    PyCapsule_GetPointer,
)

PyCapsule_SetName = ctypes.pythonapi.PyCapsule_SetName
PyCapsule_SetName.argtypes = [ctypes.py_object, ctypes.c_char_p]
PyCapsule_SetName.restype = ctypes.c_int

_DEFAULT_CONTEXT = object()


class _Context:
    def __init__(self):
        self.cleanup_count = 0


def _make_managed_tensor(context=_DEFAULT_CONTEXT, cleanup=None):
    storage = (ctypes.c_float * 1)(1.0)
    shape = (ctypes.c_int64 * 1)(1)
    if context is _DEFAULT_CONTEXT:
        context = _Context()

    tensor = DLTensor()
    tensor.data = ctypes.addressof(storage)
    tensor.device.device_type = DLDeviceType.kDLCPU
    tensor.device.device_id = 0
    tensor.ndim = 1
    tensor.dtype.code = DLDataTypeCode.kDLFloat
    tensor.dtype.bits = 32
    tensor.dtype.lanes = 1
    tensor.shape = shape
    tensor.strides = None
    tensor.byte_offset = 0
    tensor._test_keepalive = (storage, shape)
    if isinstance(context, _Context):
        context._test_keepalive = (storage, shape)

    if cleanup is None:

        def cleanup(manager_context):
            manager_context.cleanup_count += 1

    return ManagedDLTensor(tensor, context, cleanup), context


def test_cleanup_without_export():
    managed, context = _make_managed_tensor()

    del managed
    gc.collect()
    assert context.cleanup_count == 1


def test_cleanup_waits_for_last_unconsumed_capsule_owner():
    managed, context = _make_managed_tensor()
    capsule = managed.__dlpack__()

    del managed
    gc.collect()
    assert context.cleanup_count == 0

    del capsule
    gc.collect()
    assert context.cleanup_count == 1


def test_export_accepts_none_manager_context():
    cleanup_count = 0

    def cleanup(manager_context):
        nonlocal cleanup_count
        assert manager_context is None
        cleanup_count += 1

    managed, _ = _make_managed_tensor(context=None, cleanup=cleanup)
    capsule = managed.__dlpack__()

    del managed
    del capsule
    gc.collect()
    assert cleanup_count == 1


def test_cleanup_waits_for_wrapper_after_unconsumed_capsule_is_deleted():
    managed, context = _make_managed_tensor()
    capsule = managed.__dlpack__()

    del capsule
    gc.collect()
    assert context.cleanup_count == 0

    del managed
    gc.collect()
    assert context.cleanup_count == 1


def test_cleanup_waits_for_every_exported_capsule():
    managed, context = _make_managed_tensor()
    first_capsule = managed.__dlpack__()
    second_capsule = managed.__dlpack__()

    del managed
    del first_capsule
    gc.collect()
    assert context.cleanup_count == 0

    del second_capsule
    gc.collect()
    assert context.cleanup_count == 1


def test_cleanup_waits_for_consumed_tensor_deleter():
    managed, context = _make_managed_tensor()
    capsule = managed.__dlpack__()
    managed_ptr = PyCapsule_GetPointer(capsule, b"dltensor")
    consumer_deleter = DLManagedTensor.from_address(managed_ptr).deleter
    assert PyCapsule_SetName(capsule, b"used_dltensor") == 0

    del capsule
    del managed
    gc.collect()
    try:
        assert context.cleanup_count == 0
    finally:
        consumer_deleter(managed_ptr)

    assert context.cleanup_count == 1


def test_numpy_consumer_keeps_storage_alive_after_wrapper_is_deleted():
    import numpy as np

    managed, context = _make_managed_tensor()
    consumer = np.from_dlpack(managed)

    del managed
    gc.collect()
    assert context.cleanup_count == 0
    assert consumer.tolist() == [1.0]

    del consumer
    gc.collect()
    assert context.cleanup_count == 1


def test_cleanup_exception_does_not_retain_manager_context():
    cleanup_count = 0
    context = _Context()
    context_ref = weakref.ref(context)

    def cleanup(_manager_context):
        nonlocal cleanup_count
        cleanup_count += 1
        raise RuntimeError("expected cleanup failure")

    managed, returned_context = _make_managed_tensor(context=context, cleanup=cleanup)
    capsule = managed.__dlpack__()

    del managed
    del capsule
    del returned_context
    del context
    gc.collect()

    assert cleanup_count == 1
    assert context_ref() is None


def test_consumed_tensor_deleter_can_run_from_another_thread():
    managed, context = _make_managed_tensor()
    capsule = managed.__dlpack__()
    managed_ptr = PyCapsule_GetPointer(capsule, b"dltensor")
    consumer_deleter = DLManagedTensor.from_address(managed_ptr).deleter
    assert PyCapsule_SetName(capsule, b"used_dltensor") == 0

    del capsule
    del managed
    gc.collect()
    assert context.cleanup_count == 0

    thread = Thread(target=consumer_deleter, args=(managed_ptr,))
    thread.start()
    thread.join(timeout=10)

    assert not thread.is_alive()
    assert context.cleanup_count == 1
