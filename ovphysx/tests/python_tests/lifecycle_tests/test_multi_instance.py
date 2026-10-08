# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Multi-instance attach-ownership regression, ONE destroy cycle per file.

Carbonite/Python cannot be re-initialized after destroy, so sequential
create/destroy cycling is not supported. This file validates that a second
instance cannot displace the first instance's process-wide live attach or
disrupt the first instance's GPU reads.

Requires a usable CUDA device and the ovstage population bridge.

@implements REQ-CAPI-ATTACH-OWNER-001
@covers AC-1 AC-2 AC-4

Runs in its own subprocess (see scripts/test_python_runtime.cmake).
"""

import ctypes

import numpy as np
import ovstage
import pytest
from ovphysx import PhysX, PhysXConfig, codeless_schema_root
from test_utils import data_path, get_cuda_driver, read_rigid_body_poses


def _read_poses(physx):
    """Every rigid body's pose [N,7] (position + orientation) via the session read API.

    Delegates to the shared helper so this multi-instance probe cannot re-introduce the keep-last
    read bug: /World/Cube* spans multiple scene partitions, so the read yields several position and
    orientation groups that must all be accumulated, not overwritten.
    """
    return read_rigid_body_poses(physx)


def _has_usable_cuda_device() -> bool:
    try:
        driver = get_cuda_driver()
        cu_init = driver.cuInit
        cu_init.argtypes = [ctypes.c_uint]
        cu_init.restype = ctypes.c_int
        if cu_init(0) != 0:
            return False

        device_count = ctypes.c_int(0)
        cu_device_get_count = driver.cuDeviceGetCount
        cu_device_get_count.argtypes = [ctypes.POINTER(ctypes.c_int)]
        cu_device_get_count.restype = ctypes.c_int
        return cu_device_get_count(ctypes.byref(device_count)) == 0 and device_count.value > 0
    except (AttributeError, OSError):
        return False


_skip_reason = None
if PhysX.get_cpu_mode():
    _skip_reason = "DirectGPU attach-owner regression requires GPU mode"
elif not _has_usable_cuda_device():
    _skip_reason = "DirectGPU attach-owner regression requires a usable CUDA device"
elif not ovstage.population.available():
    _skip_reason = "ovstage population bridge is unavailable"


def _populate_stage(name: str, usd_path: str):
    ovstage.population.register_usd_schemas([str(codeless_schema_root())])
    stage = ovstage.Stage(name)
    try:
        ovstage.population.open_usd(
            stage,
            usd_path,
            ordinal=1,
            domains=ovstage.PopulationDomain.PHYSICS,
        )
        stage.advance_write_floor(ordinal=1).wait()
        return stage
    except Exception:
        stage.destroy()
        raise


def _destroy_instance_and_stage(physx, stage) -> None:
    try:
        if physx is not None:
            try:
                physx.detach_ovstage()
            finally:
                physx.destroy()
    finally:
        if stage is not None:
            stage.destroy()


@pytest.mark.skipif(_skip_reason is not None, reason=_skip_reason or "")
def test_rejected_second_attach_keeps_first_gpu_binding_readable():
    """A rejected peer attach must leave the owner's existing binding usable."""
    config = PhysXConfig(
        carbonite_overrides={
            "/physics/suppressReadback": True,
        }
    )
    usd_path = data_path("boxes_falling_on_groundplane_gpu.usda")

    physx_a = PhysX(config=config)
    physx_b = None
    stage_a = None
    stage_b = None

    try:
        stage_a = _populate_stage("ovphysx-attach-owner-a", usd_path)
        physx_a.attach_ovstage(stage_a, read_ordinal=1)
        physx_a.wait_all()
        attach_a = physx_a.get_attach_handle()
        assert attach_a != 0

        physx_a.warmup()
        physx_a.wait_all()

        poses_before = _read_poses(physx_a)
        assert poses_before.shape[0] > 0
        assert poses_before.shape[1] == 7
        assert np.isfinite(poses_before).all()

        physx_b = PhysX(config=config)
        stage_b = _populate_stage("ovphysx-attach-owner-b", usd_path)

        attach_error = None
        try:
            physx_b.attach_ovstage(stage_b, read_ordinal=1)
        except RuntimeError as exc:
            attach_error = str(exc)

        # Keep this read before asserting the attach result. If the owner gate is
        # removed, B displaces A and this reaches the originally crashing stale
        # GPU tensor-view path instead of stopping at an expected-error check.
        poses_after_rejected_attach = _read_poses(physx_a)

        assert attach_error is not None
        assert physx_b.get_attach_handle() == 0
        assert physx_a.get_attach_handle() == attach_a
        np.testing.assert_array_equal(poses_after_rejected_attach, poses_before)

        # A detach on B, which never attached, is a no-op and must not disturb A.
        physx_b.detach_ovstage()
        assert physx_b.get_attach_handle() == 0
        assert physx_a.get_attach_handle() == attach_a
        poses_after_b_detach = _read_poses(physx_a)
        np.testing.assert_array_equal(poses_after_b_detach, poses_before)
        physx_a.step_sync(1.0 / 60.0)

        # Once A releases the process-wide owner latch, the already-populated B
        # stage can attach and read its own live tensor view.
        physx_a.detach_ovstage()
        physx_a.destroy()
        physx_a = None
        stage_a.destroy()
        stage_a = None

        physx_b.attach_ovstage(stage_b, read_ordinal=1)
        physx_b.wait_all()
        assert physx_b.get_attach_handle() != 0
        physx_b.warmup()
        physx_b.wait_all()
        poses_b = _read_poses(physx_b)
        assert poses_b.shape[0] > 0
        assert np.isfinite(poses_b).all()
    finally:
        try:
            _destroy_instance_and_stage(physx_a, stage_a)
        finally:
            _destroy_instance_and_stage(physx_b, stage_b)
