# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Multi-instance attach regression, ONE destroy cycle per file.

Carbonite/Python cannot be re-initialized after destroy, so sequential
create/destroy cycling is not supported. This file validates that two
instances can attach at once and that detaching one leaves the other's GPU
reads intact.

Requires a usable CUDA device and the ovstage population bridge.

@implements REQ-CAPI-ATTACH-OWNER-001
@covers AC-1 AC-2 AC-4
@implements REQ-CAPI-INSTANCE-SCOPE-001
@covers AC-4

Runs in its own subprocess (see scripts/test_python_runtime.cmake).
"""

import ctypes

import numpy as np
import ovstage
import pytest
from test_utils import data_path, get_cuda_driver, read_rigid_body_poses

from ovphysx import PhysX, PhysXConfig
from schema_support import physx_schema_paths


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
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
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
    if physx is not None:
        try:
            physx.wait_task(physx.detach_ovstage())
        finally:
            physx.destroy()
    if stage is not None:
        stage.destroy()


@pytest.mark.skipif(_skip_reason is not None, reason=_skip_reason or "")
def test_second_attach_leaves_first_gpu_binding_readable():
    """A second live attach must not displace the first instance's GPU reads."""
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
        physx_a.wait_task(physx_a.attach_ovstage(stage_a, read_ordinal=1))
        physx_a.wait_all()
        attach_a = physx_a._capture_attachment_query()
        assert attach_a != 0

        physx_a.warmup()
        physx_a.wait_all()

        poses_before = _read_poses(physx_a)
        assert poses_before.shape[0] > 0
        assert poses_before.shape[1] == 7
        assert np.isfinite(poses_before).all()

        physx_b = PhysX(config=config)
        stage_b = _populate_stage("ovphysx-attach-owner-b", usd_path)
        physx_b.wait_task(physx_b.attach_ovstage(stage_b, read_ordinal=1))
        physx_b.wait_all()
        assert physx_b.is_stage_attached(stage_b)
        assert physx_a._is_query_valid(attach_a)

        physx_b.warmup()
        physx_b.wait_all()
        poses_b = _read_poses(physx_b)
        assert poses_b.shape[0] > 0
        assert np.isfinite(poses_b).all()

        poses_after_second_attach = _read_poses(physx_a)
        np.testing.assert_array_equal(poses_after_second_attach, poses_before)

        physx_b.step_sync(1.0 / 60.0)
        np.testing.assert_array_equal(_read_poses(physx_a), poses_before)
        poses_b_after_own_step = _read_poses(physx_b)

        physx_a.step_sync(1.0 / 60.0)
        np.testing.assert_array_equal(_read_poses(physx_b), poses_b_after_own_step)
        poses_a_after_own_step = _read_poses(physx_a)

        physx_b.wait_task(physx_b.detach_ovstage())
        assert not physx_b.is_stage_attached()
        assert physx_a._is_query_valid(attach_a)
        np.testing.assert_array_equal(_read_poses(physx_a), poses_a_after_own_step)
        physx_a.step_sync(1.0 / 60.0)
        physx_a._release_attachment_query(attach_a)
    finally:
        try:
            _destroy_instance_and_stage(physx_a, stage_a)
        finally:
            _destroy_instance_and_stage(physx_b, stage_b)


def test_peer_unsubscribe_does_not_drop_owner_subscription():
    """A second instance cannot unsubscribe the first instance's object-change id."""
    physx_a = PhysX()
    physx_b = PhysX()
    try:
        sub = physx_a.subscribe_object_changes(on_created=lambda _path, _type: None)
        with pytest.raises(RuntimeError):
            physx_b.unsubscribe_object_changes(sub)
        physx_a.unsubscribe_object_changes(sub)
    finally:
        try:
            physx_a.destroy()
        finally:
            physx_b.destroy()


def test_destroy_clears_object_change_callback_keepalive():
    """Instance teardown releases Python callback captures after native cleanup."""
    physx = PhysX()
    try:
        subscription = physx.subscribe_object_changes(on_created=lambda _path, _type: None)
        assert subscription in physx._object_change_keepalive

        physx.destroy()

        assert not physx._object_change_keepalive
    finally:
        physx.destroy()
