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
    (SimObjectType.RIGID_BODY, "position", WRITABLE),
    (SimObjectType.RIGID_BODY, "mass", WRITABLE),
    (SimObjectType.RIGID_BODY, "force", WRITE_ONLY),  # control input, no read-back
    (SimObjectType.RIGID_BODY, "linearAcceleration", READ_ONLY),
    (SimObjectType.RIGID_BODY, "inverseMass", READ_ONLY),
    (SimObjectType.ARTICULATION_LINK, "position", READ_ONLY),  # link pose is derived
    (SimObjectType.ARTICULATION_LINK, "mass", WRITABLE),
    (SimObjectType.ARTICULATION_LINK, "force", WRITE_ONLY),
    (SimObjectType.ARTICULATION_JOINT, "jointPosition", WRITABLE),
    (SimObjectType.ARTICULATION_JOINT, "jointVelocityTarget", WRITABLE),
    (SimObjectType.ARTICULATION_JOINT, "jointProjectedForce", READ_ONLY),
    # A limit interval exists only on an eLIMITED axis. A finite limit on a free axis is refused at
    # write time (AC-10), so the signal reports it as conditional, not unconditionally writable.
    (SimObjectType.ARTICULATION_JOINT, "jointLimit", CONDITIONAL),
    (SimObjectType.VEHICLE_WHEEL, "driveTorque", WRITE_ONLY),
    (SimObjectType.VEHICLE_WHEEL, "position", READ_ONLY),  # wheel pose is derived
    (SimObjectType.DEFORMABLE_VOLUME, "points", WRITABLE),
    (SimObjectType.DEFORMABLE_VOLUME, "restPoints", READ_ONLY),
    (SimObjectType.DEFORMABLE_SURFACE, "velocities", WRITABLE),
    (SimObjectType.PARTICLE_SET, "points", WRITABLE),
    (SimObjectType.FIXED_TENDON, "tendonLimit", WRITABLE),
    (SimObjectType.SPATIAL_TENDON, "tendonStiffness", WRITABLE),
    (SimObjectType.ARTICULATION, "rootPosition", WRITABLE),
    (SimObjectType.ARTICULATION, "jacobian", READ_ONLY),
    (SimObjectType.DEFORMABLE_MATERIAL, "deformableYoungsModulus", WRITABLE),
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
    """An object type outside SimObjectType (a value past DEFORMABLE_MATERIAL, e.g. a caller who
    mixed up the tensor-binding OVPHYSX_OBJECT_TYPE_* domain) is rejected, not returned as
    UNCLASSIFIED, so it is distinguishable from an attribute the type does not accept (AC-2)."""
    bad = int(SimObjectType.DEFORMABLE_MATERIAL) + 1
    with pytest.raises(ValueError, match="unknown object type"):
        writability(bad, "position")


def test_writability_is_bare_process():
    """No instance, scene or step: the query answers from the static table alone (AC-2)."""
    assert writability(int(SimObjectType.RIGID_BODY), "position") == WRITABLE
