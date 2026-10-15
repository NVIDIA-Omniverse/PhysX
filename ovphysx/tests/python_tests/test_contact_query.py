# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Contact sensors selected through the read API, on a DirectGPU simulation (ADR-0044).

On the device, a pattern-selected read reduces the scene's full sensor set and compacts the selected rows
out of it, and a contact query reduces through the filtered reducer of its rows, shared by every contact query
over the same rows; both come back as CUDA columns in the requested order.
"""

# @implements REQ-CAPI-READ-CONTACT-001
# @covers AC-4
# @maps_to TEST-CAPI-READ-CONTACT-001
#
# @implements REQ-CAPI-READ-CONTACT-002
# @covers AC-1 AC-3 AC-4 AC-5
# @maps_to TEST-CAPI-READ-CONTACT-003

from pathlib import Path

from ovphysx.types import ContactSelection
from test_utils import (
    FILTERED_CONTACT_COUNT,
    FILTERED_CONTACT_FORCE,
    FILTERED_CONTACT_NORMAL,
    FILTERED_CONTACT_POINT,
    FRICTION_CONTACT_FORCE_MATRIX,
    NORMAL_CONTACT_FORCE_MATRIX,
    attach_usd_with_ovstage,
    check_filtered_contact_detail,
    read_contact_columns,
    read_contact_rows,
    slide,
    write_contact_sensor_usda,
)

LIGHT, HEAVY = 9.81, 19.62


def _assert_weights(rows, expected):
    assert [path for path, _ in rows] == [path for path, _ in expected]
    for (path, force), (_, weight) in zip(rows, expected):
        assert abs(force[2] - weight) < 0.05 * weight, f"{path}: {force}"


def test_contact_query_and_selection_device(physx_sdk, tmp_path):
    stage = attach_usd_with_ovstage(physx_sdk, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    _assert_weights(read_contact_rows(physx_sdk, stage, pattern="/World/Heavy"), [("/World/Heavy", HEAVY)])
    contacts = [
        ContactSelection("/World/Light", ["/World/Ground"]),
        ContactSelection("/World/Heavy", ["/World/Ground"]),
    ]
    _assert_weights(
        read_contact_rows(physx_sdk, stage, contacts=contacts), [("/World/Light", LIGHT), ("/World/Heavy", HEAVY)]
    )


def test_contact_query_force_matrices_device(physx_sdk, tmp_path):
    """The force matrices on DirectGPU: CUDA columns of F x 3 floats per sensor."""
    stage = attach_usd_with_ovstage(physx_sdk, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    contacts = [
        ContactSelection("/World/Heavy", ["/World/Ground", "/World/Light"]),
        ContactSelection("/World/Light", ["/World/Ground", "/World/Heavy"]),
    ]
    columns, meta = read_contact_columns(
        physx_sdk, stage, [NORMAL_CONTACT_FORCE_MATRIX, FRICTION_CONTACT_FORCE_MATRIX], contacts=contacts
    )
    assert meta[NORMAL_CONTACT_FORCE_MATRIX] == (True, (2, 6))
    normal = dict(columns[NORMAL_CONTACT_FORCE_MATRIX])
    for path, weight in (("/World/Heavy", HEAVY), ("/World/Light", LIGHT)):
        assert abs(normal[path][2] - weight) < 0.05 * weight, f"{path}: {normal[path]}"
        assert all(abs(v) < 1e-3 for v in normal[path][3:6]), f"{path}: {normal[path]}"


def test_contact_query_filtered_detail_device(physx_sdk, tmp_path):
    """The filtered detail on DirectGPU: CUDA arrays per sensor and a CUDA int32 count companion."""
    stage = attach_usd_with_ovstage(physx_sdk, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    contacts = [
        ContactSelection("/World/Heavy", ["/World/Ground", "/World/Light"]),
        ContactSelection("/World/Light", ["/World/Ground", "/World/Heavy"]),
    ]
    attributes = [FILTERED_CONTACT_COUNT, FILTERED_CONTACT_FORCE, FILTERED_CONTACT_NORMAL, FILTERED_CONTACT_POINT]
    columns, meta = read_contact_columns(physx_sdk, stage, attributes, contacts=contacts)
    check_filtered_contact_detail(columns, meta, True, {"/World/Heavy": HEAVY, "/World/Light": LIGHT})


def test_contact_query_repeated_reads_keep_their_arrays_device(physx_sdk, tmp_path):
    """Repeated reads with the same contacts, each read's array kept past its result: every call opens a fresh
    contact query over the same rows, so the reads share one device reducer, and each kept array still holds
    its own read's values -- /World/Light falling in the first, resting in the later ones."""
    import numpy
    import ovstage
    from ovphysx.types import SimObjectType

    usda = write_contact_sensor_usda(tmp_path)
    text = Path(usda).read_text()
    light = "double3 xformOp:translate = (0, 0, 0.5)"
    assert text.count(light) == 1  # /World/Light, the only resting box at the origin
    Path(usda).write_text(text.replace(light, "double3 xformOp:translate = (0, 0, 3)"))
    stage = attach_usd_with_ovstage(physx_sdk, usda)
    paths = ovstage.PathDictionary(stage)
    contacts = [
        ContactSelection("/World/Light", ["/World/Ground"]),
        ContactSelection("/World/Heavy", ["/World/Ground"]),
    ]

    kept = []
    for steps in (10, 140, 1, 1):
        for _ in range(steps):
            physx_sdk.step(1.0 / 60.0)
        physx_sdk.wait_all()
        with physx_sdk.read_sync(SimObjectType.CONTACT_SENSOR, [NORMAL_CONTACT_FORCE_MATRIX], contacts=contacts) as result:
            (group,) = result.groups
            assert paths.get_path_strings(group.prim_list) == ["/World/Light", "/World/Heavy"]
            tensor = group.tensors[0]
            assert tensor.device.is_cuda and tuple(tensor.shape) == (2, 3)
            kept.append((tensor, tensor.numpy().copy()))

    # One filter per sensor: lane 2 is the ground's normal force along z.
    falling, *resting = [values for _, values in kept]
    assert falling[0][2] == 0.0, falling
    for values in resting:
        assert abs(values[0][2] - LIGHT) < 0.05 * LIGHT, values
        assert abs(values[1][2] - HEAVY) < 0.05 * HEAVY, values
    for tensor, values in kept:
        numpy.testing.assert_array_equal(tensor.numpy(), values)


def test_contact_query_friction_detail_after_sliding_device(physx_sdk, tmp_path):
    """DirectGPU: sliding makes the device friction matrix and filtered-friction detail nonzero (at-rest ~0)."""
    stage = attach_usd_with_ovstage(physx_sdk, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()
    slide(physx_sdk, "/World/Heavy")
    contacts = [ContactSelection("/World/Heavy", ["/World/Ground"])]
    attributes = [FRICTION_CONTACT_FORCE_MATRIX, "filteredFrictionCount", "filteredFrictionForce"]
    columns, meta = read_contact_columns(physx_sdk, stage, attributes, contacts=contacts)
    assert meta[FRICTION_CONTACT_FORCE_MATRIX][0] is True  # a CUDA column
    matrix = dict(columns[FRICTION_CONTACT_FORCE_MATRIX])["/World/Heavy"]
    assert any(abs(v) > 0.05 * HEAVY for v in matrix), f"device friction matrix should be nonzero while sliding: {matrix}"
    count = int(dict(columns["filteredFrictionCount"])["/World/Heavy"][0])
    assert count > 0 and len(dict(columns["filteredFrictionForce"])["/World/Heavy"]) == 3 * count
