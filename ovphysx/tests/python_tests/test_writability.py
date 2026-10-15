# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Programmatic writability of (object type, attribute) pairs (REQ-INPUT-COVERAGE-001 AC-2).

Drives the Python binding of ``ovphysx_writability``. Writability is a static property of the
write API (no instance, scene or step), so these run without a fixture, in a bare process.
Expected values are a snapshot of the C-layer table in ovphysxPhysXInterop.cpp (itself a snapshot
of the runtime write/read tables). This checks a representative subset, not all of them, and does
not re-derive from the runtime.
"""

# @implements REQ-INPUT-COVERAGE-001
# @covers AC-2
# @maps_to TEST-INPUT-COVERAGE-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-15
# @maps_to TEST-CAPI-WRITE-001
#
# @implements REQ-CAPI-SLEEP-001
# @covers AC-1
# @maps_to TEST-CAPI-SLEEP-001
#
# @implements REQ-CAPI-VEHICLE-001
# @covers AC-3
# @maps_to TEST-CAPI-VEHICLE-001
# @implements REQ-CAPI-READ-CONTACT-001
# @covers AC-3 AC-4 AC-5 AC-6
# @maps_to TEST-CAPI-READ-CONTACT-002
#
# @implements REQ-CAPI-READ-CONTACT-002
# @covers AC-4 AC-5
# @maps_to TEST-CAPI-READ-CONTACT-003

import pytest
from ovphysx._bindings import writability
from ovphysx.types import SimObjectType

# ovphysx_writability_t values.
UNCLASSIFIED = 0
WRITABLE = 1
CONDITIONAL = 2
WRITE_ONLY = 3
READ_ONLY = 4

# Representative (object type, attribute, expected) across every object type and every class.
CASES = [
    (SimObjectType.SDF_SHAPE, "position", UNCLASSIFIED),  # query-only: evaluated, never written
    (SimObjectType.RIGID_BODY, "position", WRITABLE),
    (SimObjectType.RIGID_BODY, "mass", WRITABLE),
    (SimObjectType.RIGID_BODY, "force", WRITE_ONLY),  # control input, no read-back
    (SimObjectType.RIGID_BODY, "linearAcceleration", READ_ONLY),
    (SimObjectType.RIGID_BODY, "inverseMass", READ_ONLY),
    (SimObjectType.RIGID_BODY, "isSleeping", READ_ONLY),
    # Conditional: driven on a CPU scene, refused on a DirectGPU scene (no device counterpart).
    (SimObjectType.RIGID_BODY, "kinematicTargetPosition", CONDITIONAL),
    # wake is a write-only command supported on CPU and GPU-with-CPU-readback scenes.
    # It is CONDITIONAL because DirectGPU requires eDISABLE_SLEEPING and rejects the command.
    (SimObjectType.RIGID_BODY, "wake", CONDITIONAL),
    (SimObjectType.ARTICULATION_LINK, "position", READ_ONLY),  # link pose is derived
    (SimObjectType.ARTICULATION_LINK, "isSleeping", READ_ONLY),
    (SimObjectType.ARTICULATION_LINK, "kinematicTargetPosition", READ_ONLY),  # reads pose, no target to drive
    (SimObjectType.ARTICULATION_LINK, "mass", WRITABLE),
    (SimObjectType.ARTICULATION_LINK, "force", WRITE_ONLY),
    # wake is deliberately UNCLASSIFIED on a link (write-only + refused-by-name = usable in neither
    # direction), like jointPosition on a rigid body -- not READ_ONLY, which would imply a readable column.
    (SimObjectType.ARTICULATION_LINK, "wake", UNCLASSIFIED),
    (SimObjectType.ARTICULATION_JOINT, "jointPosition", WRITABLE),
    (SimObjectType.ARTICULATION_JOINT, "jointVelocityTarget", WRITABLE),
    (SimObjectType.ARTICULATION_JOINT, "jointProjectedForce", READ_ONLY),
    # A limit interval exists only on an eLIMITED axis. A finite limit on a free axis is refused at
    # write time (AC-10), so the signal reports it as conditional, not unconditionally writable.
    (SimObjectType.ARTICULATION_JOINT, "jointLimit", CONDITIONAL),
    (SimObjectType.VEHICLE, "accelerator", CONDITIONAL),
    (SimObjectType.VEHICLE, "brake0", CONDITIONAL),
    (SimObjectType.VEHICLE, "brake1", CONDITIONAL),
    (SimObjectType.VEHICLE, "steer", CONDITIONAL),
    (SimObjectType.VEHICLE, "requestedGear", CONDITIONAL),
    (SimObjectType.VEHICLE, "thrust0", CONDITIONAL),
    (SimObjectType.VEHICLE, "thrust1", CONDITIONAL),
    (SimObjectType.VEHICLE, "resetToRestState", CONDITIONAL),
    (SimObjectType.VEHICLE, "clutch", READ_ONLY),
    (SimObjectType.VEHICLE, "currentGear", READ_ONLY),
    (SimObjectType.VEHICLE, "targetGear", READ_ONLY),
    (SimObjectType.VEHICLE, "engineRotationSpeed", READ_ONLY),
    (SimObjectType.VEHICLE, "automaticTransmission", READ_ONLY),
    (SimObjectType.VEHICLE, "gearSwitchTime", READ_ONLY),
    (SimObjectType.VEHICLE, "autoboxTimeSinceLastShift", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "rotationSpeed", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "rotationAngle", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "wheelSteerAngle", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "suspensionJounce", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "suspensionForce", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "tireFriction", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "tireLongitudinalDirection", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "tireLateralDirection", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "tireLongitudinalSlip", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "tireLateralSlip", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "tireForce", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "isOnGround", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "groundPlane", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "groundHitPosition", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "groundActorPath", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "groundShapePath", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "groundMaterialPath", READ_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "driveTorque", WRITE_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "position", READ_ONLY),  # wheel pose is derived
    (SimObjectType.DEFORMABLE_VOLUME, "points", WRITABLE),
    (SimObjectType.DEFORMABLE_VOLUME, "restPoints", READ_ONLY),
    # Volume-only, and CONDITIONAL: no device-mode restriction (it drives through a device buffer that
    # survives DirectGPU, unlike the rigid one), but object-state preconditions the write reports as unmet --
    # refused on a volume already kinematic via another path (eKINEMATIC or a view's ePARTIALLY_KINEMATIC),
    # and on an un-pin (moving an already-driven node back to free; driving is monotonic).
    (SimObjectType.DEFORMABLE_VOLUME, "kinematicTarget", CONDITIONAL),
    (SimObjectType.DEFORMABLE_SURFACE, "kinematicTarget", UNCLASSIFIED),  # no setKinematicTargetBufferD
    (SimObjectType.DEFORMABLE_SURFACE, "velocities", WRITABLE),
    (SimObjectType.PARTICLE_SET, "points", WRITABLE),
    (SimObjectType.FIXED_TENDON, "tendonLimit", WRITABLE),
    (SimObjectType.SPATIAL_TENDON, "tendonStiffness", WRITABLE),
    (SimObjectType.ARTICULATION, "rootPosition", WRITABLE),
    (SimObjectType.ARTICULATION, "jacobian", READ_ONLY),
    (SimObjectType.ARTICULATION, "isSleeping", UNCLASSIFIED),
    (SimObjectType.DEFORMABLE_MATERIAL, "deformableYoungsModulus", WRITABLE),
    (SimObjectType.DEBUG_RENDER, "bodyAxes", WRITABLE),
    (SimObjectType.DEBUG_RENDER, "pointPosition", READ_ONLY),
    # Every contact column (net forces, force matrices) is a per-step output; a write is refused.
    (SimObjectType.CONTACT_SENSOR, "netNormalContactForce", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "netFrictionContactForce", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "normalContactForceMatrix", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "frictionContactForceMatrix", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "filteredContactForce", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "filteredContactPoint", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "filteredContactNormal", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "filteredContactSeparation", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "filteredContactCount", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "filteredFrictionForce", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "filteredFrictionPoint", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "filteredFrictionCount", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "rawContactForce", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "rawContactPoint", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "rawContactNormal", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "rawContactSeparation", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "rawContactOtherBody", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventType", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventSensorSide", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventCollider", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventOtherBody", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventOtherCollider", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventProtoIndex", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventOtherProtoIndex", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointCount", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventAnchorCount", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointPosition", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointNormal", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointImpulse", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointSeparation", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointFaceIndex", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointOtherFaceIndex", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointMaterial", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventPointOtherMaterial", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventAnchorPosition", READ_ONLY),
    (SimObjectType.CONTACT_SENSOR, "contactEventAnchorImpulse", READ_ONLY),
    (SimObjectType.CHARACTER_CONTROLLER, "position", UNCLASSIFIED),
    (SimObjectType.CHARACTER_CONTROLLER, "orientation", UNCLASSIFIED),
    (SimObjectType.CHARACTER_CONTROLLER, "move", CONDITIONAL),
]


@pytest.mark.parametrize("obj, attr, expected", CASES)
def test_writability_matches_the_write_api(obj, attr, expected):
    """Each (object type, attribute) classifies as the write/read tables say (AC-2)."""
    assert writability(int(obj), attr) == expected


def test_unknown_attribute_is_unclassified():
    """A name the object type does not accept is a gap to close, not an error: UNCLASSIFIED."""
    assert writability(int(SimObjectType.RIGID_BODY), "definitelyNotAnAttribute") == UNCLASSIFIED
    # jointPosition is an articulation-joint attribute, not a rigid-body one.
    assert writability(int(SimObjectType.RIGID_BODY), "jointPosition") == UNCLASSIFIED


def test_unknown_object_type_is_rejected():
    """An object type outside SimObjectType (one past the last enum value) is rejected, not returned as
    UNCLASSIFIED, so it is distinguishable from an attribute the type does not accept (AC-2)."""
    # Derived from the enum's own max so it tracks a newly added type without a manual bump (the C side
    # keys the same check off OVPHYSX_QUERY_TARGET_COUNT).
    bad = int(max(SimObjectType)) + 1
    with pytest.raises(ValueError, match="unknown object type"):
        writability(bad, "position")


def test_writability_is_bare_process():
    """No instance, scene or step: the query answers from the static table alone (AC-2)."""
    assert writability(int(SimObjectType.RIGID_BODY), "position") == WRITABLE


def test_vehicle_selector_matches_public_c_value():
    assert int(SimObjectType.VEHICLE) == 13
    assert int(SimObjectType.DEBUG_RENDER) == 23
    assert int(max(SimObjectType)) == 24
