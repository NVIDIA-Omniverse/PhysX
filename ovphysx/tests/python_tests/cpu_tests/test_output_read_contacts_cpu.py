# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Contact-sensor net contact force through the ovphysx read API, on a CPU simulation.

A non-DirectGPU scene reduces contacts on the host, so ``netNormalContactForce`` and
``netFrictionContactForce`` each come back as one CPU Warp column. The read opens through
``ovphysx_query`` with ``SimObjectType.CONTACT_SENSOR``, so this also pins that the public surface
accepts the type.
"""

# @implements REQ-CAPI-READ-CONTACT-001
# @covers AC-1 AC-2 AC-4 AC-5 AC-6
# @maps_to TEST-CAPI-READ-CONTACT-001
# @implements REQ-CAPI-TIMESTEP-002
# @covers AC-3
# @maps_to TEST-CAPI-TIMESTEP-002

import pytest
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import (
    CONTACT_EVENT_OTHER_BODY,
    CONTACT_EVENT_POINT_COUNT,
    CONTACT_EVENT_TYPE,
    NET_FRICTION_CONTACT_FORCE,
    NET_NORMAL_CONTACT_FORCE,
    RAW_CONTACT_FORCE,
    RAW_CONTACT_NORMAL,
    RAW_CONTACT_OTHER_BODY,
    assert_contact_sensor_forces,
    attach_usd_with_ovstage,
    check_raw_contacts,
    read_contact_columns,
    read_contact_rows,
    read_net_contact_forces,
    slide,
    write_contact_sensor_usda,
)


def test_cpu_contact_sensor_net_force_read(physx_sdk_cpu, tmp_path):
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()

    forces, columns = read_net_contact_forces(physx_sdk_cpu, stage)

    # One host-resident vec3 column per component, in request order, each covering the three sensors of
    # the single scene.
    assert columns == [(NET_NORMAL_CONTACT_FORCE, False, (3, 3)), (NET_FRICTION_CONTACT_FORCE, False, (3, 3))]
    assert_contact_sensor_forces(forces)

    # Exercise the async public selection gate and the same scene's changing reducer membership.
    attributes = [NET_FRICTION_CONTACT_FORCE, NET_NORMAL_CONTACT_FORCE]
    for selected_path in ("/World/Heavy", "/World/Float", "/World/Missing"):
        expected_count = int(selected_path in forces[NET_NORMAL_CONTACT_FORCE])
        query = physx_sdk_cpu.query(SimObjectType.CONTACT_SENSOR, pattern=selected_path).wait()
        try:
            assert query.metadata is not None
            assert query.metadata.total_prim_count == expected_count
            result = physx_sdk_cpu.read(query, attributes, metadata=True).wait()
            try:
                assert result.metadata is not None
                assert result.metadata.total_prim_count == expected_count
                assert [item.group_count for item in result.metadata.attributes] == [expected_count] * 2
                assert all(item.omission_reasons == 0 for item in result.metadata.attributes)
                emitted = []
                while (group := result.fetch_next().wait()) is not None:
                    name = query.dictionary.token_to_string(group.attribute)
                    emitted.append(name)
                    assert query.dictionary.get_path_strings(group.prim_list) == [selected_path]
                    assert group.prim_count == 1
                    assert group.tensors[0].numpy()[0] == pytest.approx(forces[name][selected_path])
                assert emitted == (attributes if expected_count else [])
            finally:
                result.close_sync()
        finally:
            query.close_sync()
        restored_forces, restored_columns = read_net_contact_forces(physx_sdk_cpu, stage)
        assert restored_forces == forces
        assert restored_columns == columns


def test_cpu_read_rejects_type_past_the_last(physx_sdk_cpu, tmp_path):
    """ovphysx_query bounds the type by OVPHYSX_QUERY_TARGET_COUNT: the last type reads (above), one past it is refused."""
    attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    with pytest.raises(RuntimeError, match="invalid object type"):
        physx_sdk_cpu.read_sync(int(max(SimObjectType)) + 1, ["netNormalContactForce"], scope=ObjectScope.ALL)
    with pytest.raises(RuntimeError, match="invalid object type"):
        physx_sdk_cpu.query(int(max(SimObjectType)) + 1, pattern="/World/Heavy").wait()


def test_cpu_contact_sensor_raw_contacts(physx_sdk_cpu, tmp_path):
    """Every contact-sensor query serves the raw contacts: an array per sensor, the other body a prim handle."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()

    attributes = [RAW_CONTACT_FORCE, RAW_CONTACT_NORMAL, RAW_CONTACT_OTHER_BODY]
    columns, meta = read_contact_columns(physx_sdk_cpu, stage, attributes, scope=ObjectScope.ALL)
    check_raw_contacts(columns, meta, False)


def test_cpu_contact_sensor_events(physx_sdk_cpu, tmp_path):
    """A CPU scene serves the contact events: each resting sensor's persisting pair with the ground."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()

    attributes = [CONTACT_EVENT_TYPE, CONTACT_EVENT_OTHER_BODY, CONTACT_EVENT_POINT_COUNT, NET_NORMAL_CONTACT_FORCE]
    columns, meta = read_contact_columns(physx_sdk_cpu, stage, attributes, scope=ObjectScope.ALL)
    resting = ["/World/Heavy", "/World/Light"]  # the hovering sensor has no event
    for attr in (CONTACT_EVENT_TYPE, CONTACT_EVENT_OTHER_BODY, CONTACT_EVENT_POINT_COUNT):
        assert sorted(path for path, _ in columns[attr]) == resting, attr
        assert meta[attr][0] is False
    types, others, points = (dict(columns[a]) for a in (CONTACT_EVENT_TYPE, CONTACT_EVENT_OTHER_BODY, CONTACT_EVENT_POINT_COUNT))
    for path in resting:
        assert types[path] and all(t == 2 for t in types[path]), f"{path}: {types[path]}"  # persist
        assert others[path] == ["/World/Ground"] * len(types[path]), f"{path}: {others[path]}"
        assert sum(points[path]) >= 1
    assert NET_NORMAL_CONTACT_FORCE in columns


# ---------------------------------------------------------------------------
# Ports from the deprecated contact binding unblocked by the CONTACT_SENSOR read (ADR-0044).
# ---------------------------------------------------------------------------


def test_cpu_contact_sensor_net_friction_after_sliding(physx_sdk_cpu, tmp_path):
    """Sliding a resting sensor makes netFrictionContactForce oppose the motion; the at-rest tests leave it ~0."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    slide(physx_sdk_cpu, "/World/Heavy")
    columns, _meta = read_contact_columns(physx_sdk_cpu, stage, [NET_FRICTION_CONTACT_FORCE], pattern="/World/Heavy")
    friction = dict(columns[NET_FRICTION_CONTACT_FORCE])["/World/Heavy"]
    weight = 2.0 * 9.81
    assert friction[0] < -0.05 * weight, f"friction should oppose the +x slide: {friction}"


def test_cpu_contact_force_uses_the_step_dt(physx_sdk_cpu, tmp_path):
    """netNormalContactForce converts impulse with the current step's dt, not a cached one.

    Seed a different async dt, then step_sync at another dt (mirroring the binding's
    test_sync_contact_force_uses_current_dt): a read that scaled the sync impulse by the stale
    cached dt would get the force wrong. Checking against the bodies' weight catches it, since
    force = impulse / dt = m*g only when the step's own dt is used.
    """
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    seed_dt, sync_dt = 1.0 / 60.0, 1.0 / 240.0
    physx_sdk_cpu.step(seed_dt)  # a different async dt, cached before the sync steps
    physx_sdk_cpu.wait_all()
    for _ in range(240):  # one simulated second at sync_dt lets the resting sensors settle
        physx_sdk_cpu.step_sync(sync_dt)
    sync = dict(read_contact_columns(physx_sdk_cpu, stage, [NET_NORMAL_CONTACT_FORCE], scope=ObjectScope.ALL)[0][NET_NORMAL_CONTACT_FORCE])
    physx_sdk_cpu.step(sync_dt)  # the documented async reference path, same dt
    physx_sdk_cpu.wait_all()
    stepped = dict(read_contact_columns(physx_sdk_cpu, stage, [NET_NORMAL_CONTACT_FORCE], scope=ObjectScope.ALL)[0][NET_NORMAL_CONTACT_FORCE])
    for path, weight in (("/World/Heavy", 2.0 * 9.81), ("/World/Light", 9.81)):
        assert abs(sync[path][2] - weight) < 0.05 * weight, f"{path} sync: {sync[path]}"
        assert abs(stepped[path][2] - weight) < 0.05 * weight, f"{path} step: {stepped[path]}"


def test_cpu_contact_force_survives_a_zero_dt_step(physx_sdk_cpu, tmp_path):
    """A zero-dt step simulates nothing, so netNormalContactForce still reads the bodies' weight.

    The other half of NVBug 6904332: this read divides by the scene's own timestep, which a
    zero-dt simulate() leaves unchanged, so it must agree with the contact binding across such
    a step (see test_contact_reads_unchanged_across_zero_dt_step). Both step paths are covered.
    """
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    step_dt = 1.0 / 240.0  # far from the 1.0 a clobbered divisor would substitute
    for _ in range(240):
        physx_sdk_cpu.step_sync(step_dt)

    def net_forces():
        columns, _meta = read_contact_columns(
            physx_sdk_cpu, stage, [NET_NORMAL_CONTACT_FORCE], scope=ObjectScope.ALL
        )
        return dict(columns[NET_NORMAL_CONTACT_FORCE])

    weights = (("/World/Heavy", 2.0 * 9.81), ("/World/Light", 9.81))
    before = net_forces()
    for path, weight in weights:  # guard: a zero baseline would make the comparison vacuous
        assert abs(before[path][2] - weight) < 0.05 * weight, f"{path} before: {before[path]}"

    physx_sdk_cpu.step_sync(0.0)
    after_sync = net_forces()
    physx_sdk_cpu.step(0.0)
    physx_sdk_cpu.wait_all()
    after_async = net_forces()

    for path, weight in weights:
        for label, forces in (("step_sync(0)", after_sync), ("step(0)", after_async)):
            assert abs(forces[path][2] - weight) < 0.05 * weight, f"{path} after {label}: {forces[path]}"


def test_cpu_contact_sensors_discovered_on_runtime_clones(physx_sdk_cpu, tmp_path):
    """clone()d reporting bodies are discovered as contact sensors and report their weight (runtime-only rows)."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    physx_sdk_cpu.clone(
        "/World/Light", ["/World/CloneA", "/World/CloneB"],
        anchor_transforms=[(-6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), (-12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)],
    )
    physx_sdk_cpu.wait_all()
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    rows = dict(read_contact_rows(physx_sdk_cpu, stage, pattern="/World/Clone*"))
    assert set(rows) == {"/World/CloneA", "/World/CloneB"}, rows
    for path, force in rows.items():
        assert abs(force[2] - 9.81) < 0.1 * 9.81, f"{path}: {force}"


def test_cpu_contact_sensor_before_first_step(physx_sdk_cpu, tmp_path):
    """Before any step: net forces read zero and the per-contact / event array groups are not emitted."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    physx_sdk_cpu.wait_all()
    attributes = [NET_NORMAL_CONTACT_FORCE, NET_FRICTION_CONTACT_FORCE, RAW_CONTACT_FORCE, CONTACT_EVENT_TYPE]
    columns, _meta = read_contact_columns(physx_sdk_cpu, stage, attributes, scope=ObjectScope.ALL)
    for attr in (NET_NORMAL_CONTACT_FORCE, NET_FRICTION_CONTACT_FORCE):
        # Pin the rows: before any step a schema-carrying sensor is still a row, valued zero,
        # so an empty read must fail rather than pass the force loop vacuously.
        assert sorted(p for p, _ in columns[attr]) == ["/World/Float", "/World/Heavy", "/World/Light"], attr
        for _path, row in columns[attr]:
            assert all(abs(v) < 1e-6 for v in row), f"{attr}: {row}"
    assert RAW_CONTACT_FORCE not in columns, "raw contacts should be absent before a step"
    assert CONTACT_EVENT_TYPE not in columns, "events should be absent before a step"


def test_cpu_contact_event_point_detail(physx_sdk_cpu, tmp_path):
    """Event point detail (position/normal/impulse) reads one array per resting sensor; counts split the points."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    attributes = ["contactEventPointCount", "contactEventPointPosition", "contactEventPointNormal", "contactEventPointImpulse"]
    columns, meta = read_contact_columns(physx_sdk_cpu, stage, attributes, scope=ObjectScope.ALL)
    resting = ["/World/Heavy", "/World/Light"]
    for attr in attributes:
        assert sorted(path for path, _ in columns[attr]) == resting, attr
        assert meta[attr][0] is False
    counts = dict(columns["contactEventPointCount"])
    positions, normals, impulses = (dict(columns[a]) for a in ("contactEventPointPosition", "contactEventPointNormal", "contactEventPointImpulse"))
    for path in resting:
        n = int(sum(counts[path]))  # contactEventPointCount splits the concatenated points per event
        assert n >= 1
        assert len(positions[path]) == 3 * n and len(normals[path]) == 3 * n and len(impulses[path]) == 3 * n
        assert all(abs(positions[path][3 * i + 2]) < 0.1 for i in range(n)), f"{path}: {positions[path]}"
        assert sum(normals[path][3 * i + 2] for i in range(n)) > 0, f"{path}: {normals[path]}"


def test_cpu_contact_event_anchor_detail(physx_sdk_cpu, tmp_path):
    """Sliding produces friction anchors in the event stream; count splits the anchor position/impulse arrays."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    slide(physx_sdk_cpu, "/World/Heavy")
    attributes = ["contactEventAnchorCount", "contactEventAnchorPosition", "contactEventAnchorImpulse"]
    columns, _meta = read_contact_columns(physx_sdk_cpu, stage, attributes, scope=ObjectScope.ALL)
    counts = dict(columns["contactEventAnchorCount"])
    positions, impulses = dict(columns["contactEventAnchorPosition"]), dict(columns["contactEventAnchorImpulse"])
    n = int(sum(counts["/World/Heavy"]))
    assert n >= 1, f"sliding Heavy should report friction anchors: {counts.get('/World/Heavy')}"
    assert len(positions["/World/Heavy"]) == 3 * n and len(impulses["/World/Heavy"]) == 3 * n
