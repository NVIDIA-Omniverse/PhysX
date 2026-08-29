# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""GPU DirectGPU coverage for deformable tensor bindings."""

from contextlib import ExitStack

import numpy as np
import pytest
from ovphysx.types import TensorType
from test_utils import CudaArray, data_path, load_usd_with_ovstage

_BODY_PATTERN = "/World/DeformableBody"
_MAT_PATTERN = "/World/DeformableMaterial"
_MULTI_BODY_PATTERN = "/World/DeformableBody_*"
_MULTI_MAT_PATTERN = "/World/DeformableMaterial_*"

# ovstage population gap: the current ovpopulation build does not surface deformable
# *material* prims (OmniPhysicsDeformableMaterialAPI family) — the ovstage schema
# enumerate returns 0, so no PxDeformableMaterial is created and the material tensor
# views match 0 prims. Deformable *bodies* are surfaced and work. To be fixed in the
# ovstage repo (add the deformable-material schema family to ovpopulation), after
# which these unskip. ovphysx deliberately does NOT traverse the backing USD stage to
# work around this.
_SKIP_DEFORMABLE_MATERIAL = pytest.mark.skip(
    reason="ovstage population does not yet surface deformable-material prims; fix in ovstage repo (ovpopulation)"
)


def _load_deformable(sdk, name="volume_deformable_simple.usda"):
    load_usd_with_ovstage(sdk, data_path(name))
    sdk.wait_all()
    sdk.warmup_gpu()


def _binding_np_dtype(binding):
    return np.dtype(binding.dtype_name)


def _read_gpu(binding, dtype=None):
    use_dtype = np.dtype(dtype) if dtype is not None else _binding_np_dtype(binding)
    buf = CudaArray(binding.shape, dtype=use_dtype)
    binding.read(buf.dltensor)
    return buf.numpy()


def _write_gpu(binding, values, dtype=None, indices=None):
    use_dtype = np.dtype(dtype) if dtype is not None else _binding_np_dtype(binding)
    src = np.asarray(values, dtype=use_dtype)
    buf = CudaArray(binding.shape if indices is None else src.shape, dtype=src.dtype)
    buf.upload(src)
    binding.write(buf.dltensor, indices=indices)


@_SKIP_DEFORMABLE_MATERIAL
def test_deformable_material_cpu_importable_without_gpu(physx_sdk):
    """Deformable material tensors are CPU-resident and readable without DirectGPU."""
    _load_deformable(physx_sdk)

    with ExitStack() as stack:
        mat = physx_sdk.create_tensor_binding(
            pattern=_MAT_PATTERN,
            tensor_type=TensorType.DEFORMABLE_MATERIAL_DYNAMIC_FRICTION,
            raise_if_empty=True,
        )
        stack.callback(mat.destroy)

        values = np.zeros(mat.shape, dtype=np.float32)
        mat.read(values)
        np.testing.assert_allclose(values, [0.5], atol=1.0e-4)


def test_deformable_body_gpu_read_write_and_metadata(physx_sdk):
    _load_deformable(physx_sdk)

    with ExitStack() as stack:
        pos = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.DEFORMABLE_SIM_NODAL_POSITION,
            raise_if_empty=True,
        )
        stack.callback(pos.destroy)
        vel = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.DEFORMABLE_SIM_NODAL_VELOCITY,
            raise_if_empty=True,
        )
        stack.callback(vel.destroy)
        targets = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.DEFORMABLE_SIM_KINEMATIC_TARGET,
            raise_if_empty=True,
        )
        stack.callback(targets.destroy)
        rest = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.DEFORMABLE_REST_NODAL_POSITION,
            raise_if_empty=True,
        )
        stack.callback(rest.destroy)
        elements = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.DEFORMABLE_SIM_ELEMENT_INDICES,
            raise_if_empty=True,
        )
        stack.callback(elements.destroy)

        assert pos.count == 1
        assert pos.shape == (1, 5, 3)
        assert vel.shape == pos.shape
        assert targets.shape == (1, 5, 4)
        assert rest.shape == pos.shape
        assert elements.shape == (1, 2, 4)
        assert elements.dtype_name == "int32"
        assert pos.prim_paths == [_BODY_PATTERN]

        positions = _read_gpu(pos)
        np.testing.assert_allclose(positions[0, 0], [0.0, 0.0, 0.0], atol=1.0e-4)

        velocities = _read_gpu(vel)
        np.testing.assert_allclose(velocities, 0.0, atol=1.0e-4)

        rest_positions = _read_gpu(rest)
        np.testing.assert_allclose(rest_positions, positions, atol=1.0e-4)

        element_indices = _read_gpu(elements, dtype=np.int32)
        np.testing.assert_array_equal(element_indices[0], np.array([[0, 1, 2, 3], [1, 2, 3, 4]], dtype=np.int32))

        index = np.array([0], dtype=np.int32)
        new_positions = positions.copy()
        new_positions[0, :, 1] += 0.05
        _write_gpu(pos, new_positions, indices=index)
        np.testing.assert_allclose(_read_gpu(pos), new_positions, atol=1.0e-4)

        new_velocities = np.zeros_like(velocities)
        new_velocities[0, :, 0] = 0.25
        _write_gpu(vel, new_velocities, indices=index)
        np.testing.assert_allclose(_read_gpu(vel), new_velocities, atol=1.0e-4)

        new_targets = np.zeros(targets.shape, dtype=np.float32)
        new_targets[0, :, :3] = new_positions[0] + np.array([0.0, 0.03, 0.0], dtype=np.float32)
        new_targets[0, :, 3] = 1.0
        _write_gpu(targets, new_targets, indices=index)
        np.testing.assert_allclose(_read_gpu(targets), new_targets, atol=1.0e-4)

        with pytest.raises(RuntimeError, match="read-only"):
            _write_gpu(rest, rest_positions, indices=index)
        with pytest.raises(RuntimeError, match="read-only"):
            _write_gpu(elements, element_indices, dtype=np.int32, indices=index)

        mask = CudaArray((1,), dtype=np.uint8)
        mask.upload(np.array([1], dtype=np.uint8))
        masked_positions = new_positions.copy()
        masked_positions[0, :, 2] += 0.025
        masked_src = CudaArray(pos.shape, dtype=pos.dtype_name)
        masked_src.upload(masked_positions)
        pos.write(masked_src.dltensor, mask=mask.dltensor)
        np.testing.assert_allclose(_read_gpu(pos), masked_positions, atol=1.0e-4)


@_SKIP_DEFORMABLE_MATERIAL
def test_deformable_material_cpu_read_indexed_and_masked_write(physx_sdk):
    _load_deformable(physx_sdk)

    with ExitStack() as stack:
        dynamic_friction = physx_sdk.create_tensor_binding(
            pattern=_MAT_PATTERN,
            tensor_type=TensorType.DEFORMABLE_MATERIAL_DYNAMIC_FRICTION,
            raise_if_empty=True,
        )
        stack.callback(dynamic_friction.destroy)
        youngs_modulus = physx_sdk.create_tensor_binding(
            pattern=_MAT_PATTERN,
            tensor_type=TensorType.DEFORMABLE_MATERIAL_YOUNGS_MODULUS,
            raise_if_empty=True,
        )
        stack.callback(youngs_modulus.destroy)
        poissons_ratio = physx_sdk.create_tensor_binding(
            pattern=_MAT_PATTERN,
            tensor_type=TensorType.DEFORMABLE_MATERIAL_POISSONS_RATIO,
            raise_if_empty=True,
        )
        stack.callback(poissons_ratio.destroy)

        assert dynamic_friction.shape == (1,)
        assert dynamic_friction.dtype_name == "float32"

        friction = np.zeros(dynamic_friction.shape, dtype=np.float32)
        youngs = np.zeros(youngs_modulus.shape, dtype=np.float32)
        poisson = np.zeros(poissons_ratio.shape, dtype=np.float32)
        dynamic_friction.read(friction)
        youngs_modulus.read(youngs)
        poissons_ratio.read(poisson)
        np.testing.assert_allclose(friction, [0.5], atol=1.0e-4)
        np.testing.assert_allclose(youngs, [1000.0], atol=1.0e-3)
        np.testing.assert_allclose(poisson, [0.3], atol=1.0e-4)

        dynamic_friction.write(np.array([0.6], dtype=np.float32))
        dynamic_friction.read(friction)
        np.testing.assert_allclose(friction, [0.6], atol=1.0e-4)

        indices = np.array([0], dtype=np.int32)
        youngs_modulus.write(np.array([1200.0], dtype=np.float32), indices=indices)
        youngs_modulus.read(youngs)
        np.testing.assert_allclose(youngs, [1200.0], atol=1.0e-3)

        mask = np.array([1], dtype=np.uint8)
        poissons_ratio.write(np.array([0.25], dtype=np.float32), mask=mask)
        poissons_ratio.read(poisson)
        np.testing.assert_allclose(poisson, [0.25], atol=1.0e-4)

        with pytest.raises(RuntimeError, match="does not expose prim path metadata"):
            _ = dynamic_friction.prim_paths


def test_deformable_body_indexed_write_selects_requested_row(physx_sdk):
    _load_deformable(physx_sdk, "volume_deformable_multi.usda")

    with ExitStack() as stack:
        pos = physx_sdk.create_tensor_binding(
            pattern=_MULTI_BODY_PATTERN,
            tensor_type=TensorType.DEFORMABLE_SIM_NODAL_POSITION,
            raise_if_empty=True,
        )
        stack.callback(pos.destroy)

        assert pos.shape == (2, 5, 3)
        positions = _read_gpu(pos)
        np.testing.assert_allclose(positions[0, 0], [0.0, 0.0, 0.0], atol=1.0e-4)
        np.testing.assert_allclose(positions[1, 0], [2.0, 0.0, 0.0], atol=1.0e-4)

        updated = positions.copy()
        updated[1, :, 1] += 0.125
        _write_gpu(pos, updated, indices=np.array([1], dtype=np.int32))

        readback = _read_gpu(pos)
        np.testing.assert_allclose(readback[0], positions[0], atol=1.0e-4)
        np.testing.assert_allclose(readback[1], updated[1], atol=1.0e-4)


@_SKIP_DEFORMABLE_MATERIAL
def test_deformable_material_partial_mask_compacts_selection(physx_sdk):
    _load_deformable(physx_sdk, "volume_deformable_multi.usda")

    with ExitStack() as stack:
        poissons_ratio = physx_sdk.create_tensor_binding(
            pattern=_MULTI_MAT_PATTERN,
            tensor_type=TensorType.DEFORMABLE_MATERIAL_POISSONS_RATIO,
            raise_if_empty=True,
        )
        stack.callback(poissons_ratio.destroy)

        assert poissons_ratio.shape == (2,)

        values = np.zeros(poissons_ratio.shape, dtype=np.float32)
        poissons_ratio.read(values)
        np.testing.assert_allclose(values, [0.3, 0.35], atol=1.0e-4)

        # Mask selects only row 0, so row 1 must retain its authored value.
        poissons_ratio.write(np.array([0.22, 0.48], dtype=np.float32), mask=np.array([1, 0], dtype=np.uint8))
        poissons_ratio.read(values)
        np.testing.assert_allclose(values, [0.22, 0.35], atol=1.0e-4)


def test_volume_deformable_collision_element_indices_read(physx_sdk):
    _load_deformable(physx_sdk)

    with ExitStack() as stack:
        collision = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.DEFORMABLE_COLLISION_ELEMENT_INDICES,
            raise_if_empty=True,
        )
        stack.callback(collision.destroy)

        assert collision.count == 1
        assert collision.ndim == 3
        assert collision.shape[0] == 1
        assert collision.shape[2] > 0  # K = getNumNodesPerElement(); 4 for volume tetmesh
        assert collision.dtype_name == "int32"

        indices = _read_gpu(collision, dtype=np.int32)
        # All indices must be non-negative (valid node references)
        assert np.all(indices >= 0), f"Negative index in collision mesh: {indices}"

        with pytest.raises(RuntimeError, match="read-only"):
            _write_gpu(collision, indices, dtype=np.int32, indices=np.array([0], dtype=np.int32))


def test_surface_deformable_body_gpu_read_write_and_metadata(physx_sdk):
    _load_deformable(physx_sdk, "surface_deformable_simple.usda")

    with ExitStack() as stack:
        pos = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.SURFACE_DEFORMABLE_SIM_POSITION,
            raise_if_empty=True,
        )
        stack.callback(pos.destroy)
        vel = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.SURFACE_DEFORMABLE_SIM_VELOCITY,
            raise_if_empty=True,
        )
        stack.callback(vel.destroy)
        rest = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.SURFACE_DEFORMABLE_REST_POSITION,
            raise_if_empty=True,
        )
        stack.callback(rest.destroy)
        elems = physx_sdk.create_tensor_binding(
            pattern=_BODY_PATTERN,
            tensor_type=TensorType.SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES,
            raise_if_empty=True,
        )
        stack.callback(elems.destroy)

        # Shape checks: 4 nodes (square cloth), 2 triangles
        assert pos.count == 1
        assert pos.shape == (1, 4, 3)
        assert vel.shape == pos.shape
        assert rest.shape == (1, 4, 3)
        assert elems.shape == (1, 2, 3)   # K=3 trimesh
        assert elems.dtype_name == "int32"
        assert pos.prim_paths == [_BODY_PATTERN]

        positions = _read_gpu(pos)
        np.testing.assert_allclose(positions[0, 0], [0.0, 0.0, 0.0], atol=1.0e-4)

        velocities = _read_gpu(vel)
        np.testing.assert_allclose(velocities, 0.0, atol=1.0e-4)

        rest_positions = _read_gpu(rest)
        np.testing.assert_allclose(rest_positions, positions, atol=1.0e-4)

        element_indices = _read_gpu(elems, dtype=np.int32)
        assert np.all(element_indices >= 0)

        index = np.array([0], dtype=np.int32)
        new_positions = positions.copy()
        new_positions[0, :, 1] += 0.05
        _write_gpu(pos, new_positions, indices=index)
        np.testing.assert_allclose(_read_gpu(pos), new_positions, atol=1.0e-4)

        new_velocities = np.zeros_like(velocities)
        new_velocities[0, :, 0] = 0.1
        masked_src = CudaArray(vel.shape, dtype=np.float32)
        masked_src.upload(new_velocities)
        mask = CudaArray((1,), dtype=np.uint8)
        mask.upload(np.array([1], dtype=np.uint8))
        vel.write(masked_src.dltensor, mask=mask.dltensor)
        np.testing.assert_allclose(_read_gpu(vel), new_velocities, atol=1.0e-4)

        with pytest.raises(RuntimeError, match="read-only"):
            _write_gpu(rest, rest_positions, indices=index)
        with pytest.raises(RuntimeError, match="read-only"):
            _write_gpu(elems, element_indices, dtype=np.int32, indices=index)


@_SKIP_DEFORMABLE_MATERIAL
def test_deformable_material_elasticity_damping_read_write(physx_sdk):
    _load_deformable(physx_sdk)

    with ExitStack() as stack:
        damping = physx_sdk.create_tensor_binding(
            pattern=_MAT_PATTERN,
            tensor_type=TensorType.DEFORMABLE_MATERIAL_ELASTICITY_DAMPING,
            raise_if_empty=True,
        )
        stack.callback(damping.destroy)

        assert damping.shape == (1,)
        assert damping.dtype_name == "float32"

        values = np.zeros(damping.shape, dtype=np.float32)
        damping.read(values)
        np.testing.assert_allclose(values, [0.01], atol=1.0e-4)

        damping.write(np.array([0.05], dtype=np.float32))
        damping.read(values)
        np.testing.assert_allclose(values, [0.05], atol=1.0e-4)

        mask = np.array([1], dtype=np.uint8)
        damping.write(np.array([0.03], dtype=np.float32), mask=mask)
        damping.read(values)
        np.testing.assert_allclose(values, [0.03], atol=1.0e-4)


@_SKIP_DEFORMABLE_MATERIAL
def test_surface_deformable_material_bending_properties_read_write(physx_sdk):
    load_usd_with_ovstage(physx_sdk, data_path("surface_deformable_material.usda"))
    physx_sdk.wait_all()

    with ExitStack() as stack:
        bstiff = physx_sdk.create_tensor_binding(
            pattern="/World/SurfaceDeformableMaterial",
            tensor_type=TensorType.DEFORMABLE_MATERIAL_BENDING_STIFFNESS,
            raise_if_empty=True,
        )
        stack.callback(bstiff.destroy)
        thick = physx_sdk.create_tensor_binding(
            pattern="/World/SurfaceDeformableMaterial",
            tensor_type=TensorType.DEFORMABLE_MATERIAL_THICKNESS,
            raise_if_empty=True,
        )
        stack.callback(thick.destroy)
        bdamp = physx_sdk.create_tensor_binding(
            pattern="/World/SurfaceDeformableMaterial",
            tensor_type=TensorType.DEFORMABLE_MATERIAL_BENDING_DAMPING,
            raise_if_empty=True,
        )
        stack.callback(bdamp.destroy)

        assert bstiff.shape == (1,)

        bstiff_val = np.zeros(1, dtype=np.float32)
        thick_val = np.zeros(1, dtype=np.float32)
        bdamp_val = np.zeros(1, dtype=np.float32)

        bstiff.read(bstiff_val)
        np.testing.assert_allclose(bstiff_val, [100.0], atol=0.1)
        thick.read(thick_val)
        np.testing.assert_allclose(thick_val, [0.01], atol=1.0e-4)
        bdamp.read(bdamp_val)
        np.testing.assert_allclose(bdamp_val, [0.05], atol=1.0e-4)

        bstiff.write(np.array([200.0], dtype=np.float32))
        bstiff.read(bstiff_val)
        np.testing.assert_allclose(bstiff_val, [200.0], atol=0.1)

        thick.write(np.array([0.02], dtype=np.float32), mask=np.array([1], dtype=np.uint8))
        thick.read(thick_val)
        np.testing.assert_allclose(thick_val, [0.02], atol=1.0e-4)


# ---------------------------------------------------------------------------
# SDF evaluation
# ---------------------------------------------------------------------------

def test_sdf_view_cube_distance_signs_and_gradients(physx_sdk):
    """SDF distances are negative inside the box and positive outside; gradients point outward."""
    load_usd_with_ovstage(physx_sdk, data_path("sdf_cube.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    # 2 query points: one just inside the +x face, one just outside.
    max_q = 2
    sdf_view = physx_sdk.create_sdf_view(pattern="/World/Cube", max_query_points=max_q)
    assert sdf_view.count == 1

    # Points in the shape's local frame (cube half-extent 0.5).
    inside_x  = 0.45   # inside  +x face  -> dist < 0, gradient ~ (1, 0, 0)
    outside_x = 0.55   # outside +x face  -> dist > 0, gradient ~ (1, 0, 0)
    query_pts = np.array([[[inside_x,  0.0, 0.0],
                            [outside_x, 0.0, 0.0]]], dtype=np.float32)  # [1, 2, 3]
    out = np.zeros((1, max_q, 4), dtype=np.float32)                    # [1, 2, 4]

    in_gpu  = CudaArray(query_pts.shape, dtype=np.float32)
    out_gpu = CudaArray(out.shape,       dtype=np.float32)
    in_gpu.upload(query_pts)
    out_gpu.upload(out)

    sdf_view.evaluate(in_gpu.dltensor, out_gpu.dltensor)
    result = out_gpu.numpy()

    # result[0, p] = (grad.x, grad.y, grad.z, dist)
    dist_inside  = result[0, 0, 3]
    dist_outside = result[0, 1, 3]
    assert dist_inside  < 0, f"Expected negative distance inside cube, got {dist_inside}"
    assert dist_outside > 0, f"Expected positive distance outside cube, got {dist_outside}"

    # Gradient for the inside point should point mostly in +x (toward the nearest face).
    gx_inside = result[0, 0, 0]
    assert gx_inside > 0.5, f"Expected outward gradient ~(1,0,0) inside, got gx={gx_inside}"

    sdf_view.destroy()
