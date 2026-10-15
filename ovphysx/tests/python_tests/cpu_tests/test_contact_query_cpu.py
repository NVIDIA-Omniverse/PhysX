# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Contact sensors selected through the read API, on a CPU simulation (ADR-0044).

A pattern narrows the contact-sensor rows like any selected query, and ``contacts=`` opens a contact query
whose rows follow its selections in order. A filter the runtime cannot pair fails the read with the
runtime's message; a malformed ``contacts=`` is refused in Python before any native call.
"""

# @implements REQ-CAPI-READ-CONTACT-001
# @covers AC-4
# @maps_to TEST-CAPI-READ-CONTACT-001
#
# @implements REQ-CAPI-READ-CONTACT-002
# @covers AC-1 AC-2 AC-3 AC-4 AC-5
# @maps_to TEST-CAPI-READ-CONTACT-003
# @implements REQ-CAPI-CONFIG-001
# @covers AC-4 AC-7
# @maps_to TEST-CAPI-CONFIG-001
# @maps_to TEST-CAPI-CONFIG-002

import numpy as np
import pytest
from ovphysx import ApiStatus, GlobalConfigKey, set_global_config
from ovphysx.config import _make_bool_entry
from ovphysx.types import ConfigBool, ContactSelection, ObjectScope, SimObjectType
from test_utils import (
    FILTERED_CONTACT_COUNT,
    FILTERED_CONTACT_FORCE,
    FILTERED_CONTACT_NORMAL,
    FILTERED_CONTACT_POINT,
    FRICTION_CONTACT_FORCE_MATRIX,
    NET_NORMAL_CONTACT_FORCE,
    NORMAL_CONTACT_FORCE_MATRIX,
    attach_usd_with_ovstage,
    check_filtered_contact_detail,
    read_contact_columns,
    read_contact_rows,
    slide,
    write_contact_sensor_usda,
)

LIGHT, HEAVY = 9.81, 19.62


def _stepped_stage(physx, tmp_path):
    stage = attach_usd_with_ovstage(physx, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx.step(1.0 / 60.0)
    physx.wait_all()
    return stage


def _assert_weights(rows, expected):
    assert [path for path, _ in rows] == [path for path, _ in expected]
    for (path, force), (_, weight) in zip(rows, expected):
        assert abs(force[2] - weight) < 0.05 * weight, f"{path}: {force}"


def test_cpu_pattern_selects_contact_sensors(physx_sdk_cpu, tmp_path):
    """ovphysx_query_selection accepts the contact-sensor type, and the pattern narrows its rows."""
    stage = _stepped_stage(physx_sdk_cpu, tmp_path)
    _assert_weights(read_contact_rows(physx_sdk_cpu, stage, pattern="/World/Heavy"), [("/World/Heavy", HEAVY)])


def test_cpu_contact_query_rows_follow_selections(physx_sdk_cpu, tmp_path):
    """Rows come in selection order, each sensor paired with a shared filter; /World/Quiet reports nothing."""
    stage = _stepped_stage(physx_sdk_cpu, tmp_path)
    contacts = [
        ContactSelection("/World/Heavy", ["/World/Ground"]),
        ContactSelection("/World/Quiet", ["/World/Ground"]),
        ContactSelection("/World/Light", ["/World/Ground"]),
    ]
    rows = read_contact_rows(physx_sdk_cpu, stage, contacts=contacts)
    _assert_weights(rows, [("/World/Heavy", HEAVY), ("/World/Light", LIGHT)])


def test_cpu_contact_query_unpairable_filter_raises(physx_sdk_cpu, tmp_path):
    """One sensor, and a filter matching several objects: the native error names the rule."""
    _stepped_stage(physx_sdk_cpu, tmp_path)
    with pytest.raises(RuntimeError, match="one object per sensor"):
        physx_sdk_cpu.read_sync(
            SimObjectType.CONTACT_SENSOR,
            [NET_NORMAL_CONTACT_FORCE],
            contacts=[ContactSelection("/World/Heavy", ["/World/*"])],
        )


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"contacts": [ContactSelection("/World/Heavy")], "pattern": "/World/Heavy"}, "cannot be combined"),
        ({"contacts": [ContactSelection("/World/Heavy")], "scope": ObjectScope.ACTIVE}, "no scope"),
        ({"contacts": []}, "at least one"),
        ({"contacts": [ContactSelection("/World/Heavy", ["/World/Ground"]), ContactSelection("/World/Light")]},
         "same number of filters"),
        ({"contacts": [ContactSelection("")]}, "non-empty"),
        ({"contacts": [ContactSelection("/World/Heavy", "/World/Ground")]}, "wrap a single pattern in a list"),
        ({"contacts": [ContactSelection(["/World/Heavy"])]}, "holds a non-integer"),
    ],
)
def test_cpu_malformed_contacts_are_refused(physx_sdk_cpu, tmp_path, kwargs, message):
    _stepped_stage(physx_sdk_cpu, tmp_path)
    with pytest.raises(ValueError, match=message):
        physx_sdk_cpu.read_sync(SimObjectType.CONTACT_SENSOR, [NET_NORMAL_CONTACT_FORCE], **kwargs)


def test_cpu_contacts_require_the_contact_sensor_type(physx_sdk_cpu, tmp_path):
    _stepped_stage(physx_sdk_cpu, tmp_path)
    with pytest.raises(ValueError, match="CONTACT_SENSOR"):
        physx_sdk_cpu.read_sync(SimObjectType.RIGID_BODY, ["position"], contacts=[ContactSelection("/World/Heavy")])


def check_force_matrices(columns, meta, is_cuda):
    """Each resting sensor's ground column carries its weight; the other sensor's column and friction are ~0."""
    assert meta[NORMAL_CONTACT_FORCE_MATRIX] == (is_cuda, (2, 6))
    assert meta[FRICTION_CONTACT_FORCE_MATRIX] == (is_cuda, (2, 6))
    normal = dict(columns[NORMAL_CONTACT_FORCE_MATRIX])
    assert [path for path, _ in columns[NORMAL_CONTACT_FORCE_MATRIX]] == ["/World/Heavy", "/World/Light"]
    for path, weight in (("/World/Heavy", HEAVY), ("/World/Light", LIGHT)):
        ground, other = normal[path][0:3], normal[path][3:6]
        assert abs(ground[2] - weight) < 0.05 * weight, f"{path}: {normal[path]}"
        assert all(abs(v) < 1e-3 for v in other), f"{path}: {normal[path]}"
    for path, row in columns[FRICTION_CONTACT_FORCE_MATRIX]:
        assert all(abs(v) < 0.05 * LIGHT for v in row), f"{path}: {row}"


def test_cpu_contact_query_force_matrices(physx_sdk_cpu, tmp_path):
    """Filters make the force matrices readable: F x 3 floats per sensor, the ground column holding its weight."""
    stage = _stepped_stage(physx_sdk_cpu, tmp_path)
    contacts = [
        ContactSelection("/World/Heavy", ["/World/Ground", "/World/Light"]),
        ContactSelection("/World/Light", ["/World/Ground", "/World/Heavy"]),
    ]
    columns, meta = read_contact_columns(
        physx_sdk_cpu, stage, [NORMAL_CONTACT_FORCE_MATRIX, FRICTION_CONTACT_FORCE_MATRIX], contacts=contacts
    )
    check_force_matrices(columns, meta, is_cuda=False)


def test_cpu_contact_query_filtered_detail(physx_sdk_cpu, tmp_path):
    """Filters make the per-contact detail readable: an array per sensor, split by its int32 counts."""
    stage = _stepped_stage(physx_sdk_cpu, tmp_path)
    contacts = [
        ContactSelection("/World/Heavy", ["/World/Ground", "/World/Light"]),
        ContactSelection("/World/Light", ["/World/Ground", "/World/Heavy"]),
    ]
    attributes = [FILTERED_CONTACT_COUNT, FILTERED_CONTACT_FORCE, FILTERED_CONTACT_NORMAL, FILTERED_CONTACT_POINT]
    columns, meta = read_contact_columns(physx_sdk_cpu, stage, attributes, contacts=contacts)
    check_filtered_contact_detail(columns, meta, False, {"/World/Heavy": HEAVY, "/World/Light": LIGHT})


# ---------------------------------------------------------------------------
# Ports from the deprecated contact binding: the friction channel and the
# filters-required rule, through the CONTACT_SENSOR read (ADR-0044).
# ---------------------------------------------------------------------------


def test_cpu_contact_query_friction_detail_after_sliding(physx_sdk_cpu, tmp_path):
    """Sliding makes the friction matrix and filtered-friction detail nonzero (the at-rest tests leave them ~0)."""
    stage = _stepped_stage(physx_sdk_cpu, tmp_path)
    slide(physx_sdk_cpu, "/World/Heavy")
    contacts = [ContactSelection("/World/Heavy", ["/World/Ground"])]
    attributes = [FRICTION_CONTACT_FORCE_MATRIX, "filteredFrictionCount", "filteredFrictionForce"]
    columns, _meta = read_contact_columns(physx_sdk_cpu, stage, attributes, contacts=contacts)
    matrix = dict(columns[FRICTION_CONTACT_FORCE_MATRIX])["/World/Heavy"]  # one ground filter -> one vec3
    assert any(abs(v) > 0.05 * (2.0 * 9.81) for v in matrix), f"friction matrix should be nonzero while sliding: {matrix}"
    count = dict(columns["filteredFrictionCount"])["/World/Heavy"][0]
    assert count > 0, f"filtered friction count: {count}"
    assert len(dict(columns["filteredFrictionForce"])["/World/Heavy"]) == 3 * int(count)


def test_cpu_filtered_attributes_absent_on_an_unfiltered_query(physx_sdk_cpu, tmp_path):
    """The matrices and filtered detail need filters: an unfiltered CONTACT_SENSOR query omits them, not an error."""
    stage = _stepped_stage(physx_sdk_cpu, tmp_path)
    attributes = [NET_NORMAL_CONTACT_FORCE, NORMAL_CONTACT_FORCE_MATRIX, FILTERED_CONTACT_FORCE]
    columns, _meta = read_contact_columns(physx_sdk_cpu, stage, attributes, pattern="/World/Heavy")
    assert NET_NORMAL_CONTACT_FORCE in columns
    assert NORMAL_CONTACT_FORCE_MATRIX not in columns, "force matrix needs filters"
    assert FILTERED_CONTACT_FORCE not in columns, "filtered detail needs filters"


def test_cpu_contact_separation_is_finite(physx_sdk_cpu, tmp_path):
    """Per-contact separation reads finite: rawContactSeparation (unfiltered) and filteredContactSeparation (filtered)."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    raw, _m = read_contact_columns(physx_sdk_cpu, stage, ["rawContactSeparation"], scope=ObjectScope.ALL)
    assert any(len(sep) for _p, sep in raw["rawContactSeparation"])
    for _p, sep in raw["rawContactSeparation"]:
        assert np.all(np.isfinite(sep))
    filt, _m = read_contact_columns(
        physx_sdk_cpu, stage, ["filteredContactSeparation"],
        contacts=[ContactSelection("/World/Heavy", ["/World/Ground"])])
    assert any(len(sep) for _p, sep in filt["filteredContactSeparation"])
    for _p, sep in filt["filteredContactSeparation"]:
        assert np.all(np.isfinite(sep))


def test_cpu_disable_contact_processing_zeroes_the_contact_read(physx_sdk_cpu, tmp_path):
    """Disable after an uncollected step; the legacy setter restores live contacts (REQ-CAPI-CONFIG-001)."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    net = "netNormalContactForce"

    def resting_force():
        cols, _m = read_contact_columns(physx_sdk_cpu, stage, [net], scope=ObjectScope.ALL)
        return dict(cols[net])

    assert any(abs(v[2]) > 1.0 for v in resting_force().values()), "baseline: resting sensors carry a normal force"
    try:
        pending = physx_sdk_cpu.step(1.0 / 60.0)
        set_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING, True)
        physx_sdk_cpu.wait_task(pending)
        physx_sdk_cpu.step_sync(1.0 / 60.0)
        cols, _m = read_contact_columns(physx_sdk_cpu, stage, [net, "rawContactForce"], scope=ObjectScope.ALL)
        assert all(abs(v[2]) < 1e-3 for v in dict(cols[net]).values()), "net forces ~0 while contact processing is disabled"
        assert not cols.get("rawContactForce"), "no raw-contact array groups while disabled"
        entry = _make_bool_entry(ConfigBool.DISABLE_CONTACT_PROCESSING, False)
        assert physx_sdk_cpu._lib.ovphysx_set_global_config_entry(entry).status == ApiStatus.SUCCESS
        physx_sdk_cpu.step_sync(1.0 / 60.0)
        assert any(abs(v[2]) > 1.0 for v in resting_force().values()), "forces restored after re-enabling"
    finally:
        set_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING, False)
