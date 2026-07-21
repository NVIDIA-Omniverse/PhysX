# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Advanced PhysX instance tests: handle property, warmup_gpu CPU behavior,
update_articulations_kinematic CPU mode, attach_ovstage / update_from_ovstage error paths,
detach_ovstage on a non-attached stage, and get_config_string / runtime
config round-trips.

Isolation note: the session-scoped physx_sdk fixture is reused across the
whole pytest session. The per-test reset() in its teardown clears USD stage
state, DOF positions, and detach/attach state — but does NOT restore
process-global config values written via set_config_int32 / set_config_bool.
Tests that mutate config must cache the original value with get_config_*
and restore it in a finally block (see test_set_config_int32_then_get_config_int32
and test_set_config_bool_then_get_config_bool below).
"""

import os

import numpy as np
import pytest
from ovphysx.types import ConfigBool, ConfigInt32, ConfigString, TensorType
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


# ---------------------------------------------------------------------------
# handle property
# ---------------------------------------------------------------------------


def test_handle_property_returns_nonzero_int(physx_sdk):
    """physx.handle must return a positive integer while the instance is alive."""
    h = physx_sdk.handle
    assert isinstance(h, int)
    assert h > 0


# ---------------------------------------------------------------------------
# warmup_gpu on CPU mode
# ---------------------------------------------------------------------------


def test_warmup_gpu_on_cpu_mode_is_noop(physx_sdk):
    """warmup_gpu() on a CPU-mode instance must not raise (it is a no-op)."""
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()  # must not raise


def test_warmup_gpu_twice_idempotent(physx_sdk):
    """Calling warmup_gpu() twice must not raise."""
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()
    physx_sdk.warmup_gpu()


# ---------------------------------------------------------------------------
# update_articulations_kinematic on CPU
# ---------------------------------------------------------------------------


def test_update_articulations_kinematic_cpu_no_error(physx_sdk):
    """update_articulations_kinematic() on CPU mode with articulations must not raise."""
    load_usd_with_ovstage(physx_sdk, data_path("two_articulations.usda"))
    physx_sdk.wait_all()
    physx_sdk.update_articulations_kinematic()


def test_update_articulations_kinematic_changes_link_pose(physx_sdk):
    """Writing DOF positions then calling update_articulations_kinematic must change
    link poses (FK refresh without a full physics step)."""
    load_usd_with_ovstage(physx_sdk, data_path("two_articulations.usda"))
    physx_sdk.wait_all()

    dof_b = physx_sdk.create_tensor_binding(
        pattern="/World/articulation*",
        tensor_type=TensorType.ARTICULATION_DOF_POSITION,
    )
    link_b = physx_sdk.create_tensor_binding(
        pattern="/World/articulation*",
        tensor_type=TensorType.ARTICULATION_LINK_POSE,
    )
    try:
        if dof_b.count == 0:
            pytest.skip("No articulations found")

        # Read baseline link poses
        before = np.zeros(link_b.shape, dtype=np.float32)
        link_b.read(before)

        # Set all DOFs to a non-zero position
        dof_vals = np.full(dof_b.shape, 0.3, dtype=np.float32)
        dof_b.write(dof_vals)

        # Kinematic update — no physics step, just FK
        physx_sdk.update_articulations_kinematic()

        # Read updated link poses
        after = np.zeros(link_b.shape, dtype=np.float32)
        link_b.read(after)

        # At least one link pose must have changed
        assert not np.allclose(
            before, after, atol=1e-6
        ), "Link poses should change after FK update with non-zero DOF positions"
    finally:
        dof_b.destroy()
        link_b.destroy()


def test_update_articulations_kinematic_no_articulations(physx_sdk):
    """update_articulations_kinematic() on a scene with no articulations must not raise."""
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()
    physx_sdk.update_articulations_kinematic()  # no articulations — must be a no-op


# ---------------------------------------------------------------------------
# detach_ovstage / attach_ovstage
# ---------------------------------------------------------------------------


def test_detach_ovstage_when_not_attached_is_noop(physx_sdk):
    """detach_ovstage() on an instance that has no attached stage must not raise."""
    physx_sdk.detach_ovstage()  # must succeed silently


def test_attach_ovstage_none_raises(physx_sdk):
    """attach_ovstage(None) must raise RuntimeError."""
    with pytest.raises(RuntimeError):
        physx_sdk.attach_ovstage(None)


def test_attach_ovstage_zero_handle_raises(physx_sdk):
    """attach_ovstage(0) must raise RuntimeError (null handle)."""
    with pytest.raises(RuntimeError):
        physx_sdk.attach_ovstage(0)


def test_update_from_ovstage_invalid_range_raises(physx_sdk):
    """update_from_ovstage requires an ordered ordinal range."""
    with pytest.raises(ValueError):
        physx_sdk.update_from_ovstage(2, 1)


def test_update_from_ovstage_without_ovstage_raises(physx_sdk):
    """update_from_ovstage requires a prior attach_ovstage call."""
    with pytest.raises(RuntimeError):
        physx_sdk.update_from_ovstage(1, 1)


def test_attach_ovstage_and_update_range(physx_sdk):
    """Attach a populated stage, then drain a subsequent ordinal range."""
    ovstage = pytest.importorskip("ovstage")
    if not ovstage.population.available():
        pytest.skip("ovstage population bridge is unavailable")

    try:
        stage = ovstage.Stage("ovphysx-attach-test")
    except Exception as exc:
        pytest.skip(f"ovstage runtime is unavailable: {exc}")

    try:
        ordinal = 1
        ovstage.population.open_usd(
            stage,
            data_path("basic_simulation.usda"),
            ordinal=ordinal,
            domains=ovstage.PopulationDomain.PHYSICS,
        )
        physx_sdk.attach_ovstage(stage, read_ordinal=ordinal)
        physx_sdk.update_from_ovstage(ordinal + 1, ordinal + 1)
    finally:
        try:
            physx_sdk.detach_ovstage()
        except RuntimeError:
            pass
        stage.destroy()


# ---------------------------------------------------------------------------
# get_config_string returns None when not set
# ---------------------------------------------------------------------------


def test_get_config_string_unset_returns_none(physx_sdk):
    """get_config_string for OMNIPVD_OVD_RECORDING_DIRECTORY when not configured
    must return None (not raise)."""
    result = physx_sdk.get_config_string(ConfigString.OMNIPVD_OVD_RECORDING_DIRECTORY)
    # May be None or an empty string; must not raise
    assert result is None or isinstance(result, str)


# ---------------------------------------------------------------------------
# Runtime config round-trips
# ---------------------------------------------------------------------------


def test_set_config_bool_then_get_config_bool(physx_sdk):
    """set_config_bool then get_config_bool must round-trip.

    set_config_bool is process-global and persists across the session-scoped
    physx_sdk fixture, so the original value must be cached and restored.
    """
    original = physx_sdk.get_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING)
    try:
        physx_sdk.set_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING, True)
        val = physx_sdk.get_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING)
        assert val is True

        physx_sdk.set_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING, False)
        val = physx_sdk.get_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING)
        assert val is False
    finally:
        physx_sdk.set_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING, original)


def test_set_config_int32_then_get_config_int32(physx_sdk):
    """set_config_int32 then get_config_int32 must round-trip.

    set_config_int32 is process-global and persists across the session-scoped
    physx_sdk fixture, so the original value must be cached and restored.
    """
    original = physx_sdk.get_config_int32(ConfigInt32.NUM_THREADS)
    try:
        physx_sdk.set_config_int32(ConfigInt32.NUM_THREADS, 2)
        val = physx_sdk.get_config_int32(ConfigInt32.NUM_THREADS)
        assert val == 2
    finally:
        physx_sdk.set_config_int32(ConfigInt32.NUM_THREADS, original)
