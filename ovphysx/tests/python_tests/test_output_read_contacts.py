# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Contact-sensor net contact force through the ovphysx read API, on a DirectGPU simulation.

The ``physx_sdk`` fixture runs DirectGPU, where contacts are reduced on the device, so
``netNormalContactForce`` and ``netFrictionContactForce`` each come back as one CUDA Warp column.
The read opens through ``ovphysx_query`` with ``SimObjectType.CONTACT_SENSOR``, so this also pins
that the public surface accepts the type.
"""

# @implements REQ-CAPI-READ-CONTACT-001
# @covers AC-1 AC-2 AC-5 AC-6
# @maps_to TEST-CAPI-READ-CONTACT-001

from ovphysx.types import ObjectScope
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
    read_net_contact_forces,
    write_contact_sensor_usda,
)


def test_contact_sensor_net_force_read_device(physx_sdk, tmp_path):
    stage = attach_usd_with_ovstage(physx_sdk, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    forces, columns = read_net_contact_forces(physx_sdk, stage)

    # One device-resident vec3 column per component, in request order, each covering the three sensors of
    # the single scene.
    assert columns == [(NET_NORMAL_CONTACT_FORCE, True, (3, 3)), (NET_FRICTION_CONTACT_FORCE, True, (3, 3))]
    assert_contact_sensor_forces(forces)


def test_contact_sensor_raw_contacts_device(physx_sdk, tmp_path):
    """Every contact-sensor query serves the raw contacts: an array per sensor, the other body a prim handle."""
    stage = attach_usd_with_ovstage(physx_sdk, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    attributes = [RAW_CONTACT_FORCE, RAW_CONTACT_NORMAL, RAW_CONTACT_OTHER_BODY]
    columns, meta = read_contact_columns(physx_sdk, stage, attributes, scope=ObjectScope.ALL)
    check_raw_contacts(columns, meta, True)


def test_contact_sensor_events_omitted_device(physx_sdk, tmp_path):
    """A DirectGPU scene serves no contact event -- its GPU contact pairs carry no event type -- and still reads."""
    stage = attach_usd_with_ovstage(physx_sdk, write_contact_sensor_usda(tmp_path))
    for _ in range(60):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    attributes = [CONTACT_EVENT_TYPE, CONTACT_EVENT_OTHER_BODY, CONTACT_EVENT_POINT_COUNT, NET_NORMAL_CONTACT_FORCE]
    columns, _ = read_contact_columns(physx_sdk, stage, attributes, scope=ObjectScope.ALL)
    assert list(columns) == [NET_NORMAL_CONTACT_FORCE]
