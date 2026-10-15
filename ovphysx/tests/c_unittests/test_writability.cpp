// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Tests for ovphysx_writability: the (object type, attribute) classification (REQ-INPUT-COVERAGE-001
// AC-2). It is a static property of the write API, so no instance, query or step is needed and these
// run as plain TEST() with nothing set up.
//
// These are snapshot checks, not a cross-check against the runtime. The per-class TESTs assert
// ovphysx_writability returns the values in kWritabilityTable for a representative subset of pairs.
// Both sides mirror the runtime's answers, so they do not catch a classification drifting from
// OvxPhysicsWrite/Read.cpp. The table and these expectations are updated by hand when those change.
// EveryDeclaredAttributeIsClassifiedSomewhere adds a completeness guard: every attribute the header
// declares must classify for some object type, so an attribute added to the write API but missing
// from the table is caught (it would be UNCLASSIFIED everywhere).

/**
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-2
 * @maps_to TEST-INPUT-COVERAGE-001
 *
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-15
 * @maps_to TEST-CAPI-WRITE-001
 *
 * @implements REQ-CAPI-SLEEP-001
 * @covers AC-1
 * @maps_to TEST-CAPI-SLEEP-001
 *
 * @implements REQ-CAPI-VEHICLE-001
 * @covers AC-3
 * @maps_to TEST-CAPI-VEHICLE-001
 * @implements REQ-CAPI-READ-CONTACT-001
 * @covers AC-3 AC-4 AC-5 AC-6
 * @maps_to TEST-CAPI-READ-CONTACT-002
 *
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-4 AC-5
 * @maps_to TEST-CAPI-READ-CONTACT-003
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_attributes.h"

#include <cstddef>
#include <fstream>
#include <iterator>
#include <regex>
#include <string>
#include <vector>

namespace
{
// A string literal carries its length in its type, so a literal attribute name needs no std::strlen.
// A runtime name (the header-scan case) already knows its own length and passes it explicitly.
template <size_t N>
ovx_string_or_token_t makeAttr(const char (&name)[N])
{
    ovx_string_or_token_t a{};
    a.token = 0;
    a.string.ptr = name;
    a.string.length = N - 1;
    return a;
}

ovx_string_or_token_t makeAttr(const char* name, size_t len)
{
    ovx_string_or_token_t a{};
    a.token = 0;
    a.string.ptr = name;
    a.string.length = len;
    return a;
}

template <size_t N>
ovphysx_writability_t classify(ovphysx_query_target_t type, const char (&name)[N])
{
    ovx_string_or_token_t a = makeAttr(name);
    ovphysx_writability_t w = OVPHYSX_WRITABILITY_UNCLASSIFIED;
    EXPECT_EQ(ovphysx_writability(type, &a, &w).status, OVPHYSX_API_SUCCESS) << type << " / " << name;
    return w;
}
} // namespace

TEST(Writability, RigidBody)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_MASS), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_FORCE), OVPHYSX_WRITABILITY_WRITE_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WRENCH), OVPHYSX_WRITABILITY_WRITE_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_ACCELERATION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_INVERSE_MASS), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_IS_SLEEPING), OVPHYSX_WRITABILITY_READ_ONLY);
    // Conditional: driven on a CPU scene, refused on a DirectGPU scene (no device counterpart).
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_KINEMATIC_TARGET_POSITION), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_KINEMATIC_TARGET_ORIENTATION), OVPHYSX_WRITABILITY_CONDITIONAL);
    // wake has no read-back column, but it is CONDITIONAL rather than WRITE_ONLY: CPU-side wakeUp/
    // putToSleep drive on a CPU scene and are refused on a DirectGPU scene (the device owns sleep state).
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WAKE), OVPHYSX_WRITABILITY_CONDITIONAL);
}

TEST(Writability, ArticulationLink)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_IS_SLEEPING),
              OVPHYSX_WRITABILITY_READ_ONLY);
    // A link's pose follows from the root pose and DOFs, so it is readable but not writable.
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_POSITION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_MASS), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_FORCE), OVPHYSX_WRITABILITY_WRITE_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_SIMULATION),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_LINK_INCOMING_JOINT_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    // A link shares the rigid read table, so the kinematic target reads back (its current pose), but a
    // link has no target to drive: readable, not writable.
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_KINEMATIC_TARGET_POSITION),
              OVPHYSX_WRITABILITY_READ_ONLY);
    // wake is deliberately UNCLASSIFIED on a link, not READ_ONLY: it has no read column (write-only) and
    // is refused by name on a link, so it is usable in neither direction -- the jointPosition-on-rigid
    // case, not an oversight. READ_ONLY would falsely imply a readable wake column.
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_WAKE), OVPHYSX_WRITABILITY_UNCLASSIFIED);
}

TEST(Writability, ArticulationJoint)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY_TARGET),
              OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DRIVE_TYPE), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    // A limit interval exists only on an eLIMITED axis, so a finite limit on a free axis is refused
    // at write time. The attribute is writable only under that condition (AC-10).
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_LIMIT),
              OVPHYSX_WRITABILITY_CONDITIONAL);
}

TEST(Writability, Vehicle)
{
    EXPECT_EQ(OVPHYSX_QUERY_TARGET_VEHICLE, 13);
    EXPECT_EQ(OVPHYSX_QUERY_TARGET_DEBUG_RENDER, 23);
    EXPECT_EQ(OVPHYSX_QUERY_TARGET_COUNT, 25);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_ACCELERATOR), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_BRAKE0), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_BRAKE1), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_STEER), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_REQUESTED_GEAR), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_THRUST0), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_THRUST1), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_RESET_TO_REST_STATE), OVPHYSX_WRITABILITY_CONDITIONAL);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_CLUTCH), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_CURRENT_GEAR), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_TARGET_GEAR), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_ENGINE_ROTATION_SPEED), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_AUTOMATIC_TRANSMISSION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_GEAR_SWITCH_TIME), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_AUTOBOX_TIME_SINCE_LAST_SHIFT), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_ROTATION_SPEED), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_ROTATION_ANGLE), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_WHEEL_STEER_ANGLE), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_SUSPENSION_JOUNCE), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_SUSPENSION_FORCE), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_FRICTION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_LONGITUDINAL_DIRECTION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_LATERAL_DIRECTION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_LONGITUDINAL_SLIP), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_LATERAL_SLIP), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_FORCE), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_IS_ON_GROUND), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_PLANE), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_HIT_POSITION), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_ACTOR_PATH), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_SHAPE_PATH), OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_MATERIAL_PATH), OVPHYSX_WRITABILITY_READ_ONLY);
}

// SDF shapes are query-only: no attribute of the type is classified writable, so a stray table row would show here.
TEST(Writability, SdfShapeIsQueryOnly)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_SDF_SHAPE, OVPHYSX_ATTR_POSITION), OVPHYSX_WRITABILITY_UNCLASSIFIED);
}

TEST(Writability, VehicleWheel)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_DRIVE_TORQUE), OVPHYSX_WRITABILITY_WRITE_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_STEER_ANGLE), OVPHYSX_WRITABILITY_WRITE_ONLY);
    // A wheel's pose is derived from the chassis + suspension + steer, so it is read-only.
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_POSITION), OVPHYSX_WRITABILITY_READ_ONLY);
}

TEST(Writability, DeformablesAndParticles)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_POINTS), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_REST_POINTS), OVPHYSX_WRITABILITY_READ_ONLY);
    // kinematicTarget is CONDITIONAL: no device-mode restriction (it drives through a device buffer that
    // survives DirectGPU, unlike the rigid one), but object-state preconditions the write reports as unmet --
    // refused on a volume already kinematic via another path (eKINEMATIC or a view's ePARTIALLY_KINEMATIC),
    // and on a write that would move an already-driven node back to free (monotonic, no un-pinning).
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_KINEMATIC_TARGET),
              OVPHYSX_WRITABILITY_CONDITIONAL);
    // A surface deformable has no setKinematicTargetBufferD, so the name is not one of its attributes.
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE, OVPHYSX_ATTR_KINEMATIC_TARGET),
              OVPHYSX_WRITABILITY_UNCLASSIFIED);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE, OVPHYSX_ATTR_VELOCITIES), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_PARTICLE_SET, OVPHYSX_ATTR_POINTS), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_YOUNGS_MODULUS),
              OVPHYSX_WRITABILITY_WRITABLE);
}

TEST(Writability, Tendons)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_FIXED_TENDON, OVPHYSX_ATTR_TENDON_LIMIT), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_STIFFNESS), OVPHYSX_WRITABILITY_WRITABLE);
    // A spatial tendon's limit lives on the leaf attachment, so it is not a spatial-tendon attribute.
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_LIMIT), OVPHYSX_WRITABILITY_UNCLASSIFIED);
}

TEST(Writability, WholeArticulation)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION), OVPHYSX_WRITABILITY_WRITABLE);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_JACOBIAN), OVPHYSX_WRITABILITY_READ_ONLY);
}

TEST(Writability, ContactSensor)
{
    // Every contact column -- the net forces, the force matrices, the filtered detail, the raw contacts and the
    // events -- is a per-step simulation output, so it is READ_ONLY: a write is an error naming the type, not
    // UNCLASSIFIED (an unfilled gap in the table).
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_NET_NORMAL_CONTACT_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_NET_FRICTION_CONTACT_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_NORMAL_CONTACT_FORCE_MATRIX),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FRICTION_CONTACT_FORCE_MATRIX),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_POINT),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_NORMAL),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_SEPARATION),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_COUNT),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_FRICTION_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_FRICTION_POINT),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_FRICTION_COUNT),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_FORCE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_POINT),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_NORMAL),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_SEPARATION),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_OTHER_BODY),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_TYPE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_SENSOR_SIDE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_COLLIDER),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_OTHER_BODY),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_OTHER_COLLIDER),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_PROTO_INDEX),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_OTHER_PROTO_INDEX),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_COUNT),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_COUNT),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_POSITION),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_NORMAL),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_IMPULSE),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_SEPARATION),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_FACE_INDEX),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_OTHER_FACE_INDEX),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_MATERIAL),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_OTHER_MATERIAL),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_POSITION),
              OVPHYSX_WRITABILITY_READ_ONLY);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_IMPULSE),
              OVPHYSX_WRITABILITY_READ_ONLY);
}

TEST(Writability, CharacterController)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER, OVPHYSX_ATTR_POSITION),
              OVPHYSX_WRITABILITY_UNCLASSIFIED);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER, OVPHYSX_ATTR_ORIENTATION),
              OVPHYSX_WRITABILITY_UNCLASSIFIED);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER, OVPHYSX_ATTR_CCT_MOVE),
              OVPHYSX_WRITABILITY_CONDITIONAL);
}

TEST(Writability, UnknownAndCrossTypeAreUnclassified)
{
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_IS_SLEEPING), OVPHYSX_WRITABILITY_UNCLASSIFIED);
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, "definitelyNotAnAttribute"), OVPHYSX_WRITABILITY_UNCLASSIFIED);
    // jointPosition is an articulation-joint attribute, not a rigid-body one.
    EXPECT_EQ(classify(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_JOINT_POSITION), OVPHYSX_WRITABILITY_UNCLASSIFIED);
}

TEST(Writability, RejectsNullArguments)
{
    ovx_string_or_token_t a = makeAttr(OVPHYSX_ATTR_POSITION);
    EXPECT_EQ(ovphysx_writability(OVPHYSX_QUERY_TARGET_RIGID_BODY, &a, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_writability_t w = OVPHYSX_WRITABILITY_WRITABLE;
    EXPECT_EQ(ovphysx_writability(OVPHYSX_QUERY_TARGET_RIGID_BODY, nullptr, &w).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(w, OVPHYSX_WRITABILITY_UNCLASSIFIED) << "out must be reset before the null-attribute check";
}

TEST(Writability, RejectsTokenOnlyAttribute)
{
    // A token-only key (no string) is rejected outright, not silently classified as UNCLASSIFIED.
    // Resolving a token would need a path dictionary this scene-free query lacks.
    ovx_string_or_token_t tok{};
    tok.token = 12345; // a non-zero interned token with the string field left empty
    tok.string.ptr = nullptr;
    tok.string.length = 0;

    ovphysx_writability_t w = OVPHYSX_WRITABILITY_WRITABLE;
    EXPECT_EQ(ovphysx_writability(OVPHYSX_QUERY_TARGET_RIGID_BODY, &tok, &w).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(w, OVPHYSX_WRITABILITY_UNCLASSIFIED);
}

TEST(Writability, RejectsUnknownObjectType)
{
    // An object_type outside ovphysx_query_target_t (OVPHYSX_QUERY_TARGET_COUNT, one past the last enum
    // value) is rejected, not returned as SUCCESS/UNCLASSIFIED. Otherwise a caller cannot
    // tell an attribute the type does not accept from a type that does not exist.
    ovx_string_or_token_t a = makeAttr(OVPHYSX_ATTR_POSITION);
    const ovphysx_query_target_t bad = static_cast<ovphysx_query_target_t>(OVPHYSX_QUERY_TARGET_COUNT);
    ovphysx_writability_t w = OVPHYSX_WRITABILITY_WRITABLE;
    EXPECT_EQ(ovphysx_writability(bad, &a, &w).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(w, OVPHYSX_WRITABILITY_UNCLASSIFIED) << "out must be reset before the unknown-type check";
}

// Completeness: every OVPHYSX_ATTR_* the public header declares must classify as something other
// than UNCLASSIFIED for at least one object type. This catches an attribute added to the write API
// but missing from kWritabilityTable, which would be UNCLASSIFIED everywhere and slip past the
// spot-checks above. The attribute vocabulary is read from the header rather than hand-listed, so a
// newly declared attribute is picked up automatically. The check enumerates the public vocabulary
// against the C-layer snapshot and does not re-derive from the runtime.
TEST(Writability, EveryDeclaredAttributeIsClassifiedSomewhere)
{
    std::ifstream header(OVPHYSX_SOURCE_DIR "/include/ovphysx/ovphysx_attributes.h");
    ASSERT_TRUE(header.is_open()) << "cannot open ovphysx_attributes.h";
    const std::string text((std::istreambuf_iterator<char>(header)), std::istreambuf_iterator<char>());

    // Matches both the single-line macros and the one line-continued (\) multi-line macro.
    const std::regex re(R"RE(#\s*define\s+OVPHYSX_ATTR_\w+\s+(?:\\\s+)?"([^"]+)")RE");
    std::vector<std::string> orphans;
    size_t count = 0;
    for (std::sregex_iterator it(text.begin(), text.end(), re), end; it != end; ++it)
    {
        ++count;
        const std::string name = (*it)[1].str();
        bool classifiedSomewhere = false;
        for (int t = 0; t < OVPHYSX_QUERY_TARGET_COUNT && !classifiedSomewhere; ++t)
        {
            ovx_string_or_token_t a = makeAttr(name.c_str(), name.size());
            ovphysx_writability_t w = OVPHYSX_WRITABILITY_UNCLASSIFIED;
            if (ovphysx_writability(static_cast<ovphysx_query_target_t>(t), &a, &w).status == OVPHYSX_API_SUCCESS &&
                w != OVPHYSX_WRITABILITY_UNCLASSIFIED)
                classifiedSomewhere = true;
        }
        if (!classifiedSomewhere)
            orphans.push_back(name);
    }
    EXPECT_GT(count, 0u) << "no OVPHYSX_ATTR_* parsed from the header -- wrong path or regex?";
    EXPECT_TRUE(orphans.empty()) << orphans.size()
                                 << " declared attribute(s) are UNCLASSIFIED for every object type "
                                    "(missing from kWritabilityTable): "
                                 << (orphans.empty() ? std::string() : orphans.front());
}
