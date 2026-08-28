# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Python tests for the TensorBinding API.

TensorBinding provides batch/tensor-oriented access to physics data:
    - create_tensor_binding(pattern=None, prim_paths=None, tensor_type=TensorType.RIGID_BODY_POSE, *, raise_if_empty=False)
    - TensorBinding class with read(tensor), write(tensor, indices=None, mask=None)
    - Synchronous operations (no wait() needed)
    - Predefined tensor types for physics data
"""

import numpy as np
import pytest
from ovphysx.types import TensorType
from test_utils import CudaArray, data_path, verify_tensor_shape
from test_utils import load_usd_with_ovstage


def test_tensor_binding_properties(physx_sdk):
    """Test TensorBinding properties (shape, count, ndim, tensor_type, handle).

    Covered APIs:
        TensorBinding.shape
        TensorBinding.count
        TensorBinding.ndim
        TensorBinding.tensor_type
        TensorBinding.handle

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates tensor binding properties.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube[1-5]",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )

    assert isinstance(binding.shape, tuple)
    assert len(binding.shape) == 2  # [N, 7]
    assert binding.shape[1] == 7  # Pose components
    assert binding.count == binding.shape[0]
    assert binding.ndim == 2
    assert binding.tensor_type == TensorType.RIGID_BODY_POSE
    assert isinstance(binding.handle, int)
    assert binding.handle > 0

    binding.destroy()


def test_tensor_binding_pattern_matching(physx_sdk):
    """Test pattern parameter with glob syntax.

    Covered APIs:
        PhysX.create_tensor_binding (pattern parameter)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates pattern matching works.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    # Patterns known to be unsupported (may vary by implementation)
    known_unsupported_patterns = {
        "/World/Cube[1-3]",  # Range patterns may not be supported
    }

    patterns = [
        ("/World/Cube*", "wildcard"),
        ("/World/Cube[1-3]", "range"),
        ("/World/*", "all children"),
    ]

    for pattern, description in patterns:
        if pattern in known_unsupported_patterns:
            # Known unsupported pattern - expect failure or skip
            try:
                binding = physx_sdk.create_tensor_binding(
                    pattern=pattern,
                    tensor_type=TensorType.RIGID_BODY_POSE,
                )
                # If it succeeds unexpectedly, verify it works and clean up
                assert binding.count > 0, f"Pattern '{pattern}' ({description}) matched but returned zero count"
                binding.destroy()
            except (RuntimeError, ValueError) as e:
                # Expected: pattern not supported
                assert len(str(e)) > 0, "Exception should have meaningful message"
        else:
            # Pattern should be supported - validate it works
            binding = physx_sdk.create_tensor_binding(
                pattern=pattern,
                tensor_type=TensorType.RIGID_BODY_POSE,
            )
            assert binding.count > 0, f"Pattern '{pattern}' ({description}) should match objects"
            binding.destroy()


def test_tensor_binding_explicit_paths(physx_sdk):
    """Test prim_paths parameter with explicit list.

    Covered APIs:
        PhysX.create_tensor_binding (prim_paths parameter)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates explicit path list works.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    # Explicit paths (unordered list)
    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/Cube3", "/World/Cube1", "/World/Cube2"],
        tensor_type=TensorType.RIGID_BODY_POSE,
    )

    assert binding.count == 3
    assert binding.shape == (3, 7)

    binding.destroy()


def test_tensor_binding_invalid_arguments(physx_sdk):
    """Test create_tensor_binding with invalid arguments.

    Covered APIs:
        PhysX.create_tensor_binding (error handling)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates appropriate errors for invalid arguments.
    """
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    # Neither pattern nor prim_paths provided
    with pytest.raises(ValueError, match="Either 'pattern' or 'prim_paths' must be provided"):
        physx_sdk.create_tensor_binding(
            tensor_type=TensorType.RIGID_BODY_POSE,
        )

    # Both pattern and prim_paths provided
    with pytest.raises(ValueError, match="Cannot specify both 'pattern' and 'prim_paths'"):
        physx_sdk.create_tensor_binding(
            pattern="/World/*",
            prim_paths=["/World/Cube"],
            tensor_type=TensorType.RIGID_BODY_POSE,
        )


def test_tensor_binding_nonexistent_prims(physx_sdk):
    """Test tensor binding with nonexistent prims.

    Covered APIs:
        PhysX.create_tensor_binding (with invalid prims)
        TensorBinding.count (empty binding)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates behavior when binding to nonexistent prims.

    Note:
        API behavior: Binding creation succeeds but results in empty binding (count=0).
        This differs from pattern matching which may raise RuntimeError.
    """
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    # Nonexistent prim creates empty binding (count=0) instead of raising error
    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/DoesNotExist"],
        tensor_type=TensorType.RIGID_BODY_POSE,
    )

    assert binding.count == 0, "Binding to nonexistent prim should result in count=0"
    binding.destroy()


def test_tensor_binding_raise_if_empty_prim_paths(physx_sdk):
    """Test opt-in error for explicit paths that match no physics prims."""
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    with pytest.raises(ValueError, match=r"matched 0 prims.*prim_paths"):
        physx_sdk.create_tensor_binding(
            prim_paths=["/World/DoesNotExist"],
            tensor_type=TensorType.RIGID_BODY_POSE,
            raise_if_empty=True,
        )


def test_tensor_binding_raise_if_empty_pattern(physx_sdk):
    """Test opt-in error for patterns that match no physics prims."""
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    with pytest.raises(
        ValueError,
        match=r"matched 0 prims.*pattern '/World/DoesNotExist'.*optional.*raise_if_empty=False.*binding.count",
    ):
        physx_sdk.create_tensor_binding(
            pattern="/World/DoesNotExist",
            tensor_type=TensorType.RIGID_BODY_POSE,
            raise_if_empty=True,
        )


def test_tensor_binding_raise_if_empty_advanced_articulation(physx_sdk):
    """Empty advanced articulation tensors raise the opt-in ValueError."""
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    with pytest.raises(ValueError, match=r"matched 0 prims.*pattern '/World/DoesNotExist'"):
        physx_sdk.create_tensor_binding(
            pattern="/World/DoesNotExist",
            tensor_type=TensorType.ARTICULATION_JACOBIAN,
            raise_if_empty=True,
        )


def test_tensor_binding_raise_if_empty_valid_match(physx_sdk):
    """Valid matches are returned normally when raise_if_empty is enabled."""
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/envs/env0/table"],
        tensor_type=TensorType.RIGID_BODY_POSE,
        raise_if_empty=True,
    )
    assert binding.count == 1
    binding.destroy()


def test_tensor_binding_empty_prim_paths(physx_sdk):
    """Test tensor binding with empty prim_paths list.

    Covered APIs:
        PhysX.create_tensor_binding (empty list)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates error handling for empty prim list.
    """
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    with pytest.raises(ValueError, match="must not be empty"):
        physx_sdk.create_tensor_binding(
            prim_paths=[],
            tensor_type=TensorType.RIGID_BODY_POSE,
        )


def test_tensor_binding_invalid_tensor_type(physx_sdk):
    """Test tensor binding with invalid tensor_type.

    Covered APIs:
        PhysX.create_tensor_binding (invalid tensor_type)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates error handling for invalid tensor types.
    """
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    # Invalid tensor type (0 = TensorType.INVALID)
    with pytest.raises(RuntimeError):
        physx_sdk.create_tensor_binding(
            prim_paths=["/World/envs/env0/table"],
            tensor_type=0,  # INVALID
        )


def test_tensor_binding_read_readonly_array(physx_sdk):
    """Test TensorBinding.read() with a read-only array.

    Covered APIs:
        TensorBinding.read (error handling)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates that passing a read-only array raises a ValueError and prevents a crash.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )

    buf = np.zeros(binding.shape, dtype=np.float32)
    buf.flags.writeable = False

    with pytest.raises(ValueError, match="Array passed to binding.read\\(\\) must be writeable"):
        binding.read(buf)

    binding.destroy()


def test_tensor_write_shape_mismatch(physx_sdk):
    """Test TensorBinding.write() with mismatched tensor shape.

    Covered APIs:
        TensorBinding.write (error handling)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates shape validation in write().
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/Cube1", "/World/Cube2"],
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )

    # Correct shape is (2, 6), provide wrong shape
    wrong_shape = np.zeros((3, 6), dtype=np.float32)  # 3 instead of 2

    with pytest.raises(RuntimeError, match="(?i)(shape|mismatch|size)"):
        binding.write(wrong_shape)

    binding.destroy()


def test_tensor_write_non_contiguous_tensor(physx_sdk):
    """Test TensorBinding.write() rejects non-contiguous tensor input.

    Covered APIs:
        TensorBinding.write (contiguity validation)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates non-C-contiguous tensors are rejected with ValueError.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/Cube1", "/World/Cube2"],
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )

    non_contiguous = np.zeros((6, 2), dtype=np.float32).T
    assert non_contiguous.shape == binding.shape, "Test tensor shape must match binding shape"
    assert not non_contiguous.flags["C_CONTIGUOUS"], "Test tensor should be non-C-contiguous"

    with pytest.raises(Exception, match=r"(?i)C-contiguous|contiguous\(\)|contiguous input"):
        binding.write(non_contiguous)

    binding.destroy()


def test_tensor_read_shape_mismatch(physx_sdk):
    """Test TensorBinding.read() with mismatched tensor shape.

    Covered APIs:
        TensorBinding.read (error handling)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates shape validation in read().
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/Cube1"],
        tensor_type=TensorType.RIGID_BODY_POSE,
    )

    # Correct shape is (1, 7), provide wrong shape
    wrong_shape = np.zeros((1, 6), dtype=np.float32)  # 6 instead of 7

    with pytest.raises(RuntimeError, match="(?i)(shape|mismatch|size)"):
        binding.read(wrong_shape)

    binding.destroy()


def test_tensor_operations_after_destroy(physx_sdk):
    """Test that operations after destroy() raise appropriate errors.

    Covered APIs:
        TensorBinding.destroy
        TensorBinding.read (after destroy)
        TensorBinding.write (after destroy)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates destroyed binding rejects operations.
    """
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        prim_paths=["/World/envs/env0/table"],
        tensor_type=TensorType.RIGID_BODY_POSE,
    )

    binding.destroy()

    # Operations after destroy should fail
    data = np.zeros((1, 7), dtype=np.float32)

    with pytest.raises(RuntimeError, match="(?i)(destroyed|invalid)"):
        binding.read(data)

    with pytest.raises(RuntimeError, match="(?i)(destroyed|invalid)"):
        binding.write(data)


def test_multiple_bindings_same_prims(physx_sdk):
    """Test creating multiple bindings to same prims with different tensor types.

    Covered APIs:
        PhysX.create_tensor_binding (multiple)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates multiple bindings to same prims.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    pose_binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )

    velocity_binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )

    assert pose_binding.count == velocity_binding.count

    assert pose_binding.shape[1] == 7  # Pose
    assert velocity_binding.shape[1] == 6  # Velocity

    pose_binding.destroy()
    velocity_binding.destroy()


def test_tensor_binding_no_usd_loaded(physx_sdk):
    """Test tensor binding creation without USD loaded.

    Covered APIs:
        PhysX.create_tensor_binding (no USD)

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Validates appropriate error when no USD is loaded.
    """
    # Don't load USD - should fail
    with pytest.raises(RuntimeError, match="(?i)(usd|stage|not loaded)"):
        physx_sdk.create_tensor_binding(
            prim_paths=["/World/Cube"],
            tensor_type=TensorType.RIGID_BODY_POSE,
        )


# ---------------------------------------------------------------------------
# Same-target cache tests for TensorBinding.read() / write()
#
# GPU-state tensors (RIGID_BODY_POSE, RIGID_BODY_VELOCITY) require CUDA
# buffers via CudaArray. CPU-property tensors (ARTICULATION_DOF_STIFFNESS)
# accept numpy even in GPU mode, used for the numpy readonly guard test.
# ---------------------------------------------------------------------------


def test_read_cache_populated(physx_sdk):
    """After the first read(), the binding's internal _read_cache should be populated."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    ga = CudaArray(binding.shape)

    assert binding._read_cache is None
    binding.read(ga.dltensor)
    assert binding._read_cache is not None
    assert binding._read_cache.tensor is ga.dltensor

    binding.destroy()


def test_read_different_buffer_replaces_cache(physx_sdk):
    """Switching to a different buffer object should replace the cached entry."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    ga_a = CudaArray(binding.shape)
    ga_b = CudaArray(binding.shape)

    binding.read(ga_a.dltensor)
    assert binding._read_cache.tensor is ga_a.dltensor

    binding.read(ga_b.dltensor)
    assert binding._read_cache.tensor is ga_b.dltensor

    binding.destroy()


def test_read_after_destroy_raises_with_cache(physx_sdk):
    """After warming the cache and then destroying, read() must still raise."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    ga = CudaArray(binding.shape)
    binding.read(ga.dltensor)

    binding.destroy()

    with pytest.raises(RuntimeError, match="(?i)(destroyed|invalid)"):
        binding.read(ga.dltensor)


def test_read_cache_cleared_on_destroy(physx_sdk):
    """destroy() must clear _read_cache and _write_cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    ga = CudaArray(binding.shape)
    binding.read(ga.dltensor)
    assert binding._read_cache is not None

    binding.destroy()
    assert binding._read_cache is None
    assert binding._write_cache is None


def test_write_cache_populated(physx_sdk):
    """After a simple write(), _write_cache should be populated."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    ga = CudaArray(binding.shape)

    assert binding._write_cache is None
    binding.write(ga.dltensor)
    assert binding._write_cache is not None
    assert binding._write_cache.tensor is ga.dltensor

    binding.destroy()


def test_write_with_indices_no_cache(physx_sdk):
    """write() with indices should NOT populate _write_cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    ga = CudaArray(binding.shape)
    gi = CudaArray((1,), dtype=np.int32)

    binding.write(ga.dltensor, indices=gi.dltensor)
    assert binding._write_cache is None

    binding.destroy()


def test_write_with_mask_no_cache(physx_sdk):
    """write() with mask should NOT populate _write_cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    ga = CudaArray(binding.shape)
    gm = CudaArray((binding.shape[0],), dtype=np.uint8)

    binding.write(ga.dltensor, mask=gm.dltensor)
    assert binding._write_cache is None

    binding.destroy()


def test_read_readonly_numpy_with_cache(physx_sdk):
    """Warming the cache then making the array read-only must raise on the fast path.

    Uses a CPU-property tensor (DOF_STIFFNESS) which accepts numpy in GPU mode.
    """
    load_usd_with_ovstage(physx_sdk, data_path("two_articulations.usda"))
    physx_sdk.wait_all()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/articulation*",
        tensor_type=TensorType.ARTICULATION_DOF_STIFFNESS,
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
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    ga = CudaArray(binding.shape)

    for _ in range(10):
        binding.read(ga.dltensor)
        assert binding._read_cache is not None
        assert binding._read_cache.tensor is ga.dltensor

    binding.destroy()


def test_write_repeated_same_buffer(physx_sdk):
    """Multiple simple writes from the same buffer should all succeed and use the cache."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(
        pattern="/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_VELOCITY,
    )
    ga = CudaArray(binding.shape)

    for _ in range(10):
        binding.write(ga.dltensor)
        assert binding._write_cache is not None
        assert binding._write_cache.tensor is ga.dltensor

    binding.destroy()
