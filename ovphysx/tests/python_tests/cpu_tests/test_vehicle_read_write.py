# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Vehicle controls and telemetry through the public session API on a CPU scene."""

# @implements REQ-CAPI-VEHICLE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @maps_to TEST-CAPI-VEHICLE-001

import numpy as np
import ovstage
import pytest
from ovphysx.api import ReadOmission, TaskError
from ovphysx.types import SimObjectType
from test_utils import attach_usd_with_ovstage, data_path

VEHICLE = SimObjectType.VEHICLE
WHEEL = SimObjectType.VEHICLE_WHEEL
BODY = SimObjectType.RIGID_BODY
VEHICLE_COLUMNS = {
    "accelerator": (np.float32, 1), "brake0": (np.float32, 1),
    "brake1": (np.float32, 1), "steer": (np.float32, 1),
    "clutch": (np.float32, 1), "currentGear": (np.int32, 1),
    "targetGear": (np.int32, 1), "engineRotationSpeed": (np.float32, 1),
    "automaticTransmission": (np.uint8, 1), "gearSwitchTime": (np.float32, 1),
    "autoboxTimeSinceLastShift": (np.float32, 1),
}
WHEEL_COLUMNS = {
    "position": (np.float32, 3), "orientation": (np.float32, 4),
    "rotationSpeed": (np.float32, 1), "rotationAngle": (np.float32, 1),
    "wheelSteerAngle": (np.float32, 1), "suspensionJounce": (np.float32, 1),
    "suspensionForce": (np.float32, 3), "tireFriction": (np.float32, 1),
    "tireLongitudinalDirection": (np.float32, 3), "tireLateralDirection": (np.float32, 3),
    "tireLongitudinalSlip": (np.float32, 1), "tireLateralSlip": (np.float32, 1),
    "tireForce": (np.float32, 3), "isOnGround": (np.uint8, 1),
    "groundPlane": (np.float32, 4), "groundHitPosition": (np.float32, 3),
    "groundActorPath": (np.uint64, 1), "groundShapePath": (np.uint64, 1),
    "groundMaterialPath": (np.uint64, 1),
}


@pytest.fixture
def vehicles(physx_sdk_cpu):
    stage = attach_usd_with_ovstage(physx_sdk_cpu, data_path("vehicle_read_write.usda"))
    return physx_sdk_cpu, stage


def _read(sdk, stage, kind, names, pattern):
    columns = {name: {} for name in names}
    with ovstage.PathDictionary(stage) as dictionary:
        with sdk.read_sync(kind, names, pattern=pattern, metadata=True) as result:
            metadata = result.metadata
            for group in result.groups:
                name = dictionary.token_to_string(group.attribute)
                assert not group.is_array
                assert len(group.tensors) == 1
                tensor = group.tensors[0]
                assert tensor.device.is_cpu
                values = tensor.numpy().copy()
                paths = dictionary.get_path_strings(group.prim_list)
                paths = paths[group.prim_offset:group.prim_offset + group.prim_count]
                assert len(values) == len(paths)
                columns[name].update(zip(paths, values))
    return columns, metadata


def _write(sdk, kind, name, value, dtype, pattern):
    with sdk.write_sync(kind, name, pattern=pattern) as session:
        assert session.groups
        for group in session.groups:
            tensor = group.tensors[0]
            assert tensor.device.is_cpu
            tensor.assign(np.full(tensor.shape, value, dtype=dtype))
            session.commit_sync(group)


def test_cold_filtered_controls_and_discovery(vehicles):
    sdk, stage = vehicles
    # The first data operation is a filtered write: no read or step primes vehicle caches.
    _write(sdk, VEHICLE, "accelerator", 0.6, np.float32, "/World/Basic")
    for _ in range(3):
        sdk.step_sync(1.0 / 60.0)
    values, _ = _read(sdk, stage, VEHICLE, ["accelerator"], "/World/**")
    assert values["accelerator"] == {
        "/World/Basic": pytest.approx(0.6), "/World/Standard": 0, "/World/Tank": 0,
    }
    for path, has_engine, has_controls in (("Basic", False, True), ("Standard", True, True),
                                            ("Raw", False, False), ("Tank", True, True)):
        with sdk.query(VEHICLE, pattern=f"/World/{path}").wait() as query:
            assert query.metadata.total_prim_count == 1
            names = {query.dictionary.token_to_string(t) for t in query.metadata.attributes}
            assert ("engineRotationSpeed" in names) == has_engine
            assert ("accelerator" in names) == has_controls
            assert "resetToRestState" not in names
            assert "requestedGear" not in names
        sdk.wait_all()
    with sdk.query(VEHICLE).wait() as query:
        assert query.metadata.total_prim_count == 4
    sdk.wait_all()
    empty, summary = _read(sdk, stage, VEHICLE, ["accelerator"], "/World/Missing")
    assert not empty["accelerator"]
    assert summary.total_prim_count == 0


def test_vehicle_column_types_and_partial_availability(vehicles):
    sdk, stage = vehicles
    sdk.step_sync(1.0 / 60.0)
    columns, summary = _read(sdk, stage, VEHICLE, list(VEHICLE_COLUMNS), "/World/Standard")
    assert summary.total_prim_count == 1
    for name, (dtype, _) in VEHICLE_COLUMNS.items():
        values = np.asarray(list(columns[name].values()))
        assert values.shape == (1,)
        assert values.dtype == dtype
        assert np.all(np.isfinite(values))
    columns, summary = _read(sdk, stage, VEHICLE, ["engineRotationSpeed"], "/World/**")
    assert set(columns["engineRotationSpeed"]) == {"/World/Standard", "/World/Tank"}
    assert summary.attributes[0].omission_reasons & ReadOmission.UNSUPPORTED_CONFIGURATION
    columns, summary = _read(sdk, stage, VEHICLE, ["requestedGear"], "/World/Standard")
    assert not columns["requestedGear"]
    assert summary.attributes[0].omission_reasons & ReadOmission.UNSUPPORTED_ATTRIBUTE


def test_wheel_columns_and_public_ground_identities(vehicles):
    sdk, stage = vehicles
    for _ in range(60):
        sdk.step_sync(1.0 / 60.0)
    columns, _ = _read(sdk, stage, WHEEL, list(WHEEL_COLUMNS), "/World/Standard/**")
    wheels = set(columns["position"])
    assert len(wheels) == 4
    for name, (dtype, width) in WHEEL_COLUMNS.items():
        assert set(columns[name]) == wheels
        values = np.asarray(list(columns[name].values()))
        assert values.shape == ((4,) if width == 1 else (4, width))
        assert values.dtype == dtype
        assert np.all(np.isfinite(values))
    assert any(columns["isOnGround"].values())
    with ovstage.PathDictionary(stage) as dictionary:
        for wheel in wheels:
            if columns["isOnGround"][wheel]:
                assert dictionary.path_to_string(int(columns["groundShapePath"][wheel])) == "/World/CollisionPlane"
                assert dictionary.path_to_string(int(columns["groundMaterialPath"][wheel])) == "/World/TarmacMaterial"
            else:
                for name in ("groundPlane", "groundHitPosition", "groundActorPath",
                             "groundShapePath", "groundMaterialPath"):
                    assert np.all(columns[name][wheel] == 0)


def test_controls_and_gear_requests_retain_native_domains(vehicles):
    sdk, stage = vehicles
    for name, value in (("accelerator", 0.4), ("brake0", 0.2), ("brake1", 0.3), ("steer", -0.5)):
        _write(sdk, VEHICLE, name, value, np.float32, "/World/Basic")
        columns, _ = _read(sdk, stage, VEHICLE, [name], "/World/Basic")
        assert columns[name]["/World/Basic"] == pytest.approx(value)
    for gear in (-1, 0, 1, 30, 255):
        _write(sdk, VEHICLE, "requestedGear", gear, np.int32, "/World/Basic")
        _write(sdk, VEHICLE, "requestedGear", gear, np.int32, "/World/Standard")
    _write(sdk, VEHICLE, "requestedGear", 2, np.int32, "/World/Standard")
    # The fixture's gear switch takes 0.5 seconds; complete the shift in manual mode.
    for _ in range(60):
        sdk.step_sync(1.0 / 60.0)
    names = ["currentGear", "targetGear", "automaticTransmission"]
    engaged, _ = _read(sdk, stage, VEHICLE, names, "/World/Standard")
    assert engaged == {
        "currentGear": {"/World/Standard": 2},
        "targetGear": {"/World/Standard": 2},
        "automaticTransmission": {"/World/Standard": 0},
    }
    # Gear 30 is accepted but absent from this gearbox, so it cannot become engaged.
    _write(sdk, VEHICLE, "requestedGear", 30, np.int32, "/World/Standard")
    for _ in range(60):
        sdk.step_sync(1.0 / 60.0)
    after, _ = _read(sdk, stage, VEHICLE, names, "/World/Standard")
    assert after == engaged


def test_raw_wheel_torque_and_tank_thrust_produce_motion(vehicles):
    sdk, stage = vehicles
    before, _ = _read(sdk, stage, BODY, ["position"], "/World/Tank")
    _write(sdk, VEHICLE, "requestedGear", 1, np.int32, "/World/Tank")
    _write(sdk, VEHICLE, "accelerator", 0.8, np.float32, "/World/Tank")
    for thrust in ("thrust0", "thrust1"):
        _write(sdk, VEHICLE, thrust, 1, np.float32, "/World/Tank")
    for name, value in (("driveTorque", 100), ("brakeTorque", 0), ("steerAngle", 0.1)):
        _write(sdk, WHEEL, name, value, np.float32, "/World/Raw/**")
    for _ in range(120):
        sdk.step_sync(1.0 / 60.0)
    wheels, _ = _read(sdk, stage, WHEEL, ["rotationSpeed", "wheelSteerAngle"], "/World/Raw/**")
    assert len(wheels["rotationSpeed"]) == 4
    assert set(wheels["wheelSteerAngle"]) == set(wheels["rotationSpeed"])
    assert all(abs(value) > 0.01 for value in wheels["rotationSpeed"].values())
    assert all(value == pytest.approx(0.1) for value in wheels["wheelSteerAngle"].values())
    tank, _ = _read(sdk, stage, BODY, ["position", "linearVelocity"], "/World/Tank")
    assert abs(tank["linearVelocity"]["/World/Tank"][2]) > 0.01
    assert abs(tank["position"]["/World/Tank"][2] - before["position"]["/World/Tank"][2]) > 0.01


@pytest.mark.parametrize("kind,name,pattern", [
    (VEHICLE, "accelerator", "/World/Raw"), (VEHICLE, "thrust0", "/World/Standard"),
    (VEHICLE, "engineRotationSpeed", "/World/Standard"),
    (WHEEL, "driveTorque", "/World/Basic/**"), (WHEEL, "wheelSteerAngle", "/World/Raw/**"),
])
def test_inapplicable_and_read_only_writes_are_rejected(vehicles, kind, name, pattern):
    sdk, _ = vehicles
    with pytest.raises(TaskError, match="[Ww]rite"):
        sdk.write_sync(kind, name, pattern=pattern)


@pytest.mark.parametrize("name,value,dtype", [
    ("accelerator", np.nan, np.float32), ("brake0", -0.1, np.float32),
    ("steer", 1.1, np.float32), ("requestedGear", 31, np.int32),
    ("resetToRestState", 2, np.uint8),
])
def test_invalid_values_do_not_change_controls(vehicles, name, value, dtype):
    sdk, stage = vehicles
    _write(sdk, VEHICLE, "accelerator", 0.3, np.float32, "/World/Standard")
    _write(sdk, VEHICLE, "requestedGear", 255, np.int32, "/World/Standard")
    _write(sdk, BODY, "linearVelocity", 3, np.float32, "/World/Standard")
    _write(sdk, BODY, "angularVelocity", 2, np.float32, "/World/Standard")
    names = ["accelerator", "brake0", "brake1", "steer", "targetGear", "automaticTransmission"]
    motion_names = ["position", "orientation", "linearVelocity", "angularVelocity"]
    motion_before, _ = _read(sdk, stage, BODY, motion_names, "/World/Standard")
    before, _ = _read(sdk, stage, VEHICLE, names, "/World/Standard")
    with pytest.raises(TaskError) as failure:
        _write(sdk, VEHICLE, name, value, dtype, "/World/Standard")
    if name == "accelerator":
        message = str(failure.value)
        assert "accelerator" in message
        assert "nothing was written" in message
        assert "finite" in message
    after, _ = _read(sdk, stage, VEHICLE, names, "/World/Standard")
    assert after == before
    motion_after, _ = _read(sdk, stage, BODY, motion_names, "/World/Standard")
    for column in motion_names:
        np.testing.assert_array_equal(motion_after[column]["/World/Standard"],
                                      motion_before[column]["/World/Standard"])



def test_invalid_gear_batch_preserves_pending_commands(vehicles):
    sdk, stage = vehicles
    paths = ["/World/Standard", "/World/Tank"]
    for path in paths:
        _write(sdk, VEHICLE, "requestedGear", 1, np.int32, path)
    for _ in range(60):
        sdk.step_sync(1.0 / 60.0)
    names = ["currentGear", "targetGear", "automaticTransmission"]
    before, _ = _read(sdk, stage, VEHICLE, names, "/World/**")
    assert before["currentGear"] == before["targetGear"] == dict.fromkeys(paths, 1)
    with ovstage.PathDictionary(stage) as dictionary:
        handles = [dictionary.intern_path(path) for path in paths]
    with sdk.write_sync(VEHICLE, "requestedGear", prim_paths=handles) as session:
        assert len(session.groups) == 1
        group = session.groups[0]
        assert group.prim_count == 2
        group.tensors[0].assign(np.array([-1, 31], dtype=np.int32))
        with pytest.raises(TaskError, match="requestedGear"):
            session.commit_sync(group)
    # An accidentally published reverse command would start shifting on this update.
    sdk.step_sync(1.0 / 60.0)
    after, _ = _read(sdk, stage, VEHICLE, names, "/World/**")
    assert after == before


def test_reset_preserves_pose_and_controls_and_is_not_replayed(vehicles):
    sdk, stage = vehicles
    sdk.step_sync(1.0 / 60.0)
    _write(sdk, VEHICLE, "accelerator", 0.4, np.float32, "/World/Standard")
    _write(sdk, BODY, "linearVelocity", 3, np.float32, "/World/Standard")
    before, _ = _read(sdk, stage, BODY, ["position"], "/World/Standard")
    _write(sdk, VEHICLE, "resetToRestState", 0, np.uint8, "/World/Standard")
    velocities, _ = _read(sdk, stage, BODY, ["linearVelocity"], "/World/Standard")
    np.testing.assert_allclose(velocities["linearVelocity"]["/World/Standard"], [3, 3, 3])
    with sdk.write_sync(VEHICLE, "resetToRestState", pattern="/World/Standard") as session:
        group = session.groups[0]
        group.tensors[0].fill_(1)
        session.commit_sync(group)
        with pytest.raises(RuntimeError, match="already been committed"):
            session.commit_sync(group)
    after, _ = _read(sdk, stage, BODY, ["position", "linearVelocity"], "/World/Standard")
    np.testing.assert_array_equal(after["position"]["/World/Standard"], before["position"]["/World/Standard"])
    np.testing.assert_array_equal(after["linearVelocity"]["/World/Standard"], [0, 0, 0])
    controls, _ = _read(sdk, stage, VEHICLE, ["accelerator"], "/World/Standard")
    assert controls["accelerator"]["/World/Standard"] == pytest.approx(0.4)
    _write(sdk, BODY, "linearVelocity", 5, np.float32, "/World/Standard")
    # Filling a command and closing without a commit discards it.
    with sdk.write_sync(VEHICLE, "resetToRestState", pattern="/World/Standard") as session:
        session.groups[0].tensors[0].fill_(1)
    sdk.step_sync(1.0 / 60.0)
    after, _ = _read(sdk, stage, BODY, ["linearVelocity"], "/World/Standard")
    assert np.linalg.norm(after["linearVelocity"]["/World/Standard"]) > 1


@pytest.mark.parametrize("reset_value", [0, 1])
def test_reset_requires_enabled_chassis_even_for_zero(vehicles, reset_value):
    sdk, _ = vehicles
    _write(sdk, BODY, "disableSimulation", 1, np.uint8, "/World/Standard")
    with pytest.raises(TaskError, match="simulation enabled"):
        _write(sdk, VEHICLE, "resetToRestState", reset_value, np.uint8, "/World/Standard")
