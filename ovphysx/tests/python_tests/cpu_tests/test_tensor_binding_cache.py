# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# DEPRECATED (tensor-binding-deprecation): a deprecated tensor-binding test. Removed with the binding.

"""CPU-mode tests for the TensorBinding same-target read/write cache.

Mirrors the GPU cache tests in test_tensor_bindings.py but uses numpy arrays
with CPU-mode PhysX. The cache logic is device-agnostic. These tests confirm
it works correctly on the CPU path.
"""

import os
from types import SimpleNamespace

import numpy as np
import pytest
from ovphysx.api import TensorBinding
from ovphysx.types import ApiStatus, TensorType
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


# ---------------------------------------------------------------------------
# Read cache
# ---------------------------------------------------------------------------


def test_read_cache_populated(physx_sdk):
    """After the first read(), _read_cache should be populated."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)

    assert binding._read_cache is None
    binding.read(buf)
    assert binding._read_cache is not None
    assert binding._read_cache.tensor is buf
    assert "keepalive" not in binding._read_cache._fields
    assert binding._read_cache.data_ptr == buf.ctypes.data

    binding.destroy()


def test_read_different_buffer_replaces_cache(physx_sdk):
    """Switching to a different buffer object should replace the cached entry."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    buf_a = np.zeros(binding.shape, dtype=np.float32)
    buf_b = np.zeros(binding.shape, dtype=np.float32)

    binding.read(buf_a)
    assert binding._read_cache.tensor is buf_a

    binding.read(buf_b)
    assert binding._read_cache.tensor is buf_b

    binding.destroy()


def test_read_after_destroy_raises_with_cache(physx_sdk):
    """After warming the cache and then destroying, read() must still raise."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)
    binding.read(buf)

    binding.destroy()

    with pytest.raises(RuntimeError, match="(?i)(destroyed|invalid)"):
        binding.read(buf)


def test_read_cache_cleared_on_destroy(physx_sdk):
    """destroy() must clear _read_cache and _write_cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)
    binding.read(buf)
    assert binding._read_cache is not None

    binding.destroy()
    assert binding._read_cache is None
    assert binding._write_cache is None


def test_read_readonly_numpy_with_cache(physx_sdk):
    """Warming the cache then making the array read-only must raise on the fast path."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)

    binding.read(buf)
    assert binding._read_cache is not None

    buf.flags.writeable = False

    with pytest.raises(ValueError, match="Array passed to binding.read\\(\\) must be writeable"):
        binding.read(buf)

    binding.destroy()


def test_read_repeated_same_buffer(physx_sdk):
    """Multiple reads into the same buffer should all succeed and use the cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)

    for _ in range(10):
        binding.read(buf)
        assert binding._read_cache is not None
        assert binding._read_cache.tensor is buf

    binding.destroy()


# ---------------------------------------------------------------------------
# Write cache
# ---------------------------------------------------------------------------


def test_write_cache_populated(physx_sdk):
    """After a simple write(), _write_cache should be populated."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)

    assert binding._write_cache is None
    binding.write(buf)
    assert binding._write_cache is not None
    assert binding._write_cache.tensor is buf
    assert "keepalive" not in binding._write_cache._fields
    assert binding._write_cache.data_ptr == buf.ctypes.data

    binding.destroy()


def test_write_with_indices_no_cache(physx_sdk):
    """write() with indices should NOT populate _write_cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)
    indices = np.array([0], dtype=np.int32)

    binding.write(buf, indices=indices)
    assert binding._write_cache is None

    binding.destroy()


def test_write_with_mask_no_cache(physx_sdk):
    """write() with mask should NOT populate _write_cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)
    mask = np.ones(binding.shape[0], dtype=np.uint8)

    binding.write(buf, mask=mask)
    assert binding._write_cache is None

    binding.destroy()


def test_unknown_dlpack_provider_is_not_cached(physx_sdk):
    """Unknown providers export a fresh capsule on every synchronous call."""

    class CountingDLPackProvider:
        """A DLPack producer ovphysx does not recognise, so its storage cannot be cached."""

        def __init__(self, array):
            self._array = array
            self.export_count = 0

        def __dlpack__(self, stream=None):
            self.export_count += 1
            return self._array.__dlpack__()

    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    read_binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    read_buffer = np.zeros(read_binding.shape, dtype=np.float32)
    read_binding.read(read_buffer)
    assert read_binding._read_cache is not None
    read_provider = CountingDLPackProvider(read_buffer)
    read_binding.read(read_provider)
    read_binding.read(read_provider)
    assert read_binding._read_cache is None
    assert read_provider.export_count == 2

    write_binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    write_buffer = np.zeros(write_binding.shape, dtype=np.float32)
    write_binding.write(write_buffer)
    assert write_binding._write_cache is not None
    write_provider = CountingDLPackProvider(write_buffer)
    write_binding.write(write_provider)
    write_binding.write(write_provider)
    assert write_binding._write_cache is None
    assert write_provider.export_count == 2

    read_binding.destroy()
    write_binding.destroy()


def _dlpack_provider(dtype):
    return np.zeros((1,), dtype=dtype)


@pytest.fixture
def failing_tensor_binding():
    failure = SimpleNamespace(status=ApiStatus.INVALID_ARGUMENT)
    success = SimpleNamespace(status=ApiStatus.SUCCESS)
    sdk = SimpleNamespace(
        _omni_physx_sdk_handle=SimpleNamespace(value=1),
        _lib=SimpleNamespace(
            ovphysx_read_tensor_binding=lambda *args: failure,
            ovphysx_write_tensor_binding=lambda *args: failure,
            ovphysx_write_tensor_binding_masked=lambda *args: failure,
            ovphysx_destroy_tensor_binding=lambda *args: success,
        ),
        _get_last_error=lambda: "forced native failure",
    )
    binding = TensorBinding(
        sdk=sdk,
        handle=1,
        tensor_type=TensorType.RIGID_BODY_POSE,
        ndim=1,
        shape=(1,),
    )
    yield binding
    binding.destroy()


@pytest.mark.parametrize("operation", ("read", "write", "mask", "indices"))
def test_native_failure_is_reported_for_every_argument_shape(failing_tensor_binding, operation):
    tensor = _dlpack_provider(np.float32)
    kwargs = {}

    if operation == "mask":
        kwargs["mask"] = _dlpack_provider(np.uint8)
    elif operation == "indices":
        kwargs["indices"] = _dlpack_provider(np.int32)

    with pytest.raises(RuntimeError, match="forced native failure") as exc_info:
        if operation == "read":
            failing_tensor_binding.read(tensor)
        else:
            failing_tensor_binding.write(tensor, **kwargs)

    assert exc_info.traceback is not None


class _FailingDLPackProvider:
    def __dlpack__(self, stream=None):
        raise ValueError("forced DLPack acquisition failure")


@pytest.mark.parametrize("argument_name", ("mask", "indices"))
def test_secondary_acquisition_failure_propagates(failing_tensor_binding, argument_name):
    tensor = _dlpack_provider(np.float32)

    with pytest.raises(ValueError, match="forced DLPack acquisition failure") as exc_info:
        failing_tensor_binding.write(tensor, **{argument_name: _FailingDLPackProvider()})

    assert exc_info.traceback is not None


def test_write_repeated_same_buffer(physx_sdk):
    """Multiple simple writes from the same buffer should all succeed and use the cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)

    for _ in range(10):
        binding.write(buf)
        assert binding._write_cache is not None
        assert binding._write_cache.tensor is buf

    binding.destroy()


# Cache-after-destroy tests live in lifecycle_tests/ (separate subprocess)
# because Carbonite cannot be re-initialized after ovphysx_destroy_instance.

# ---------------------------------------------------------------------------
# Staleness guard: in-place numpy resize must invalidate the cache
# ---------------------------------------------------------------------------


def _force_data_pointer_change(buf, shape):
    """Resize *buf* in-place until its data pointer differs from the current one.

    ``numpy.ndarray.resize()`` calls ``realloc`` internally, and the C
    allocator is allowed to return the same address (e.g. when it can extend
    the block in-place).  Increasingly large intermediate sizes are tried, with
    "blocker" arrays allocated in between to prevent address reuse.  If the
    pointer still has not moved after all attempts, False is returned so the
    caller can ``pytest.skip`` instead of failing flakily.
    """
    old_ptr = buf.ctypes.data
    blockers = []
    for factor in (4, 16, 64, 256, 1024):
        buf.resize((shape[0] * factor, shape[1]), refcheck=False)
        blockers.append(np.empty(buf.shape, dtype=buf.dtype))
        buf.resize(shape, refcheck=False)
        if buf.ctypes.data != old_ptr:
            break
    del blockers
    return buf.ctypes.data != old_ptr


def test_read_cache_invalidated_by_numpy_resize(physx_sdk):
    """numpy resize() in place changes the data pointer while keeping identity.

    The staleness guard must detect this and fall back to the slow path
    instead of passing a stale DLTensor to the C layer.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)
    binding.read(buf)
    assert binding._read_cache is not None

    if not _force_data_pointer_change(buf, binding.shape):
        binding.destroy()
        pytest.skip(
            "numpy allocator reused the same data address despite resize; "
            "cannot test data-pointer staleness detection"
        )

    binding.read(buf)
    assert binding._read_cache is not None
    assert binding._read_cache.data_ptr == buf.ctypes.data

    binding.destroy()


def test_write_cache_invalidated_by_numpy_resize(physx_sdk):
    """Same staleness guard test for the write path."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    buf = np.zeros(binding.shape, dtype=np.float32)
    binding.write(buf)
    assert binding._write_cache is not None

    if not _force_data_pointer_change(buf, binding.shape):
        binding.destroy()
        pytest.skip(
            "numpy allocator reused the same data address despite resize; "
            "cannot test data-pointer staleness detection"
        )

    binding.write(buf)
    assert binding._write_cache is not None
    assert binding._write_cache.data_ptr == buf.ctypes.data

    binding.destroy()
