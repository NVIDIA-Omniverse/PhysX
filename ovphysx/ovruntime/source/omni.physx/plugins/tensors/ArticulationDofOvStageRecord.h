// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-6
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-11, AC-15
 */

#pragma once

#include <cstdint>

// The gather that consumes these records runs on both devices, and the device one is compiled by
// nvcc, so everything below has to be callable from a kernel.
#if defined(__CUDACC__)
#    define OVX_DOF_HD __host__ __device__
#else
#    define OVX_DOF_HD
#endif

namespace omni
{
namespace physx
{
namespace tensors
{

// Per-output-slot record for the ovstage joint read (the `…OvStage` view methods). Device-agnostic
// and dependency-free, so CUDA and host TUs share it. The read emits a flat list of (joint-prim,
// enabled-axis) scalars, so each slot names its source DOF and carries the per-axis facts a fold
// needs: dst[i] = dofScaleFor(record[i], policy) * dofScalar[viewArtiIdx][physxDofIdx].
//
// It carries the axis facts rather than one finished multiply because the fold is per attribute
// while the list is shared by all of them (see DofScalePolicy): a position is degrees times the
// body-order sign, a torque on the same axis is the sign alone, a stiffness converts the other way.
struct ArticulationDofOvStageRecord
{
    uint32_t viewArtiIdx = 0xffffffff; // articulation's row in the view
    uint32_t physxDofIdx = 0xffffffff; // DOF index within that articulation
    // rad->deg for an axis reported in degrees, 1 otherwise. Supplied by the reader so it is the same
    // constant the parse library converted with (ADR-0001 section 8.1), and so an authored
    // JointStateAPI overriding the per-axis convention is honoured where that is known.
    float angScale = 1.0f;
    // The inverse fold. Stored rather than derived as 1/angScale: the reciprocal of the float rad->deg
    // constant is not the float deg->rad one, so the value would not round-trip.
    float invAngScale = 1.0f;
    // +1 when the joint's body0 is the parent link, -1 otherwise. The DirectGPU buffer and the
    // articulation cache are unsigned-raw, so a signed quantity folds this in; a magnitude does not.
    float sign = 1.0f;
};

// How an attribute folds the record's axis facts into its gather. Named per attribute in the reader's
// table, so the choice is explicit rather than inferred from the quantity's name.
//
// The two angular directions are not symmetric spellings of one option. USD authors in degrees and the
// parse library converts to radians (ADR-0001 section 8.1), so a quantity measured per radian was
// multiplied by rad->deg on the way in and reading it back means multiplying by deg->rad. Picking the
// wrong direction is a 57.295^2 error.
enum class DofScalePolicy : uint32_t
{
    // Degrees on an angular axis, and the body-order sign. Joint position, velocity and their drive
    // targets: a generalized coordinate and its derivatives.
    eAngularSigned = 0,
    // The sign alone. Forces and torques on the axis: signed, but a newton-metre is not an angle.
    eSigned = 1,
    // Degrees on an angular axis, no sign. Anything whose generalized-coordinate units sit in the
    // numerator and which the body order cannot flip: the max joint and actuator velocities, and the
    // drive-envelope speed/effort gradient, which is a rate per unit effort rather than a magnitude.
    eAngularForward = 2,
    // Radians-to-degrees inverted, no sign: a quantity measured per unit of generalized coordinate,
    // authored per degree and stored per radian (drive stiffness and damping, viscous friction,
    // velocity-dependent resistance).
    eAngularInverse = 3,
    // No fold at all. Plain SI or dimensionless: max effort, armature, static/dynamic friction
    // effort, the drive type enum.
    eNone = 4
};

// The multiply a policy applies to the raw PhysX scalar. Callable from a CUDA kernel and from the host
// loop, so the two backends cannot drift on a unit.
OVX_DOF_HD inline float dofScaleFor(const ArticulationDofOvStageRecord& r, DofScalePolicy policy)
{
    switch (policy)
    {
    case DofScalePolicy::eAngularSigned:  return r.angScale * r.sign;
    case DofScalePolicy::eSigned:         return r.sign;
    case DofScalePolicy::eAngularForward: return r.angScale;
    case DofScalePolicy::eAngularInverse: return r.invAngScale;
    case DofScalePolicy::eNone:           return 1.0f;
    }
    return 1.0f;
}

// The WRITE direction's factor for the same policy (ADR-0012).
//
// It is the policy with the ANGULAR FACTOR SWAPPED -- not 1/dofScaleFor. The record stores angScale
// and invAngScale separately precisely because the float reciprocal of rad->deg is not the float
// deg->rad, so `value / dofScaleFor(...)` does not return an authored value to itself while
// `value * dofInverseScaleFor(...)` does.
//
// The sign is +-1, so it is its own inverse and appears unchanged.
//
// Kept beside dofScaleFor rather than at the call sites: the two have to stay exact inverses, and
// that is far easier to see when they are five lines apart than when one lives in a kernel.
OVX_DOF_HD inline float dofInverseScaleFor(const ArticulationDofOvStageRecord& r, DofScalePolicy policy)
{
    switch (policy)
    {
    case DofScalePolicy::eAngularSigned:  return r.invAngScale * r.sign;
    case DofScalePolicy::eSigned:         return r.sign;
    case DofScalePolicy::eAngularForward: return r.invAngScale;
    case DofScalePolicy::eAngularInverse: return r.angScale;
    case DofScalePolicy::eNone:           return 1.0f;
    }
    return 1.0f;
}

// Which per-DOF property an output column carries. One `case` in one host gather rather than a method
// each: every one comes out of a per-joint PhysX call on the (joint, axis) pair the record already
// names, differing only in which field of the result it takes.
//
// All are host-sourced on both devices, and that is not an unfinished device path: these are
// simulation inputs PhysX never writes back, so there is no device copy, and the tensor API's
// equivalents refuse a device tensor (REQ-TENSOR-CPU-ONLY-001).
enum class DofProperty : uint32_t
{
    eStiffness = 0,
    eDamping,
    eLimit, // 2 components: (lower, upper) as one interval
    eMaxVelocity,
    eMaxForce,
    eArmature,
    eStaticFriction,
    eDynamicFriction,
    eViscousFriction,
    eSpeedEffortGradient,
    eMaxActuatorVelocity,
    eVelocityDependentResistance,
    eDriveType // uint8, not float
};

// Component width of a property. Only the limit is a pair: (lower, upper) is one interval, so it
// stays one attribute with two lanes. Declared with the enum so the gather and the reader that sizes
// the column cannot disagree about a width.
inline uint32_t dofPropertyComponents(DofProperty prop)
{
    return prop == DofProperty::eLimit ? 2u : 1u;
}

// True when a property's column is uint8 rather than float32. Only the drive type, an enum the
// tensor API also reports as a byte.
inline bool dofPropertyIsByte(DofProperty prop)
{
    return prop == DofProperty::eDriveType;
}

// Which PhysX struct a property's value comes from. A bitmask because a set of properties needs the
// union: getDofPropertiesOvStage fetches each named struct once per DOF and every column then reads
// what it needs out of them.
//
// Stated once here so the two halves of the gather cannot drift: a property whose value case reads a
// struct its selection case forgot to name publishes an unfetched struct -- a plausible zero, or
// stack residue where the PhysX default ctor leaves the members alone (PxJointFrictionParams) --
// under the right attribute name, lane count and dtype, indistinguishable from a real reading.
enum DofPropertySource : uint32_t
{
    eDofSrcNone = 0u,
    eDofSrcDrive = 1u << 0,       // getDriveParams -- seven of the thirteen columns
    eDofSrcFriction = 1u << 1,    // getFrictionParams -- three of them
    eDofSrcLimit = 1u << 2,       // getMotion plus getLimitParams
    eDofSrcMaxVelocity = 1u << 3, // getMaxJointVelocity
    eDofSrcArmature = 1u << 4,    // getArmature
};

inline uint32_t dofPropertySource(DofProperty prop)
{
    switch (prop)
    {
    case DofProperty::eStiffness:
    case DofProperty::eDamping:
    case DofProperty::eMaxForce:
    case DofProperty::eDriveType:
    case DofProperty::eSpeedEffortGradient:
    case DofProperty::eMaxActuatorVelocity:
    case DofProperty::eVelocityDependentResistance: return eDofSrcDrive;
    case DofProperty::eStaticFriction:
    case DofProperty::eDynamicFriction:
    case DofProperty::eViscousFriction:             return eDofSrcFriction;
    case DofProperty::eLimit:                       return eDofSrcLimit;
    case DofProperty::eMaxVelocity:                 return eDofSrcMaxVelocity;
    case DofProperty::eArmature:                    return eDofSrcArmature;
    }
    return eDofSrcNone;
}

// The attribute name, for the check helpers' messages. Thirteen columns share one gather, so without
// this every refused read logs the same line. Spelled as the public attribute so the message names
// the thing the caller asked for.
inline const char* dofPropertyLabel(DofProperty prop)
{
    switch (prop)
    {
    case DofProperty::eStiffness:                   return "jointStiffness";
    case DofProperty::eDamping:                     return "jointDamping";
    case DofProperty::eLimit:                       return "jointLimit";
    case DofProperty::eMaxVelocity:                 return "jointMaxVelocity";
    case DofProperty::eMaxForce:                    return "jointMaxForce";
    case DofProperty::eArmature:                    return "jointArmature";
    case DofProperty::eStaticFriction:              return "jointStaticFriction";
    case DofProperty::eDynamicFriction:             return "jointDynamicFriction";
    case DofProperty::eViscousFriction:             return "jointViscousFriction";
    case DofProperty::eSpeedEffortGradient:         return "jointSpeedEffortGradient";
    case DofProperty::eMaxActuatorVelocity:         return "jointMaxActuatorVelocity";
    case DofProperty::eVelocityDependentResistance: return "jointVelocityDependentResistance";
    case DofProperty::eDriveType:                   return "jointDriveType";
    }
    return "joint DOF property";
}

} // namespace tensors
} // namespace physx
} // namespace omni

// Scoped to this header, as InstancerReframe.h does with OVX_INSTANCER_HD: every use is above, and
// leaving it defined puts a bare OVX_* macro in every translation unit that includes this.
#undef OVX_DOF_HD
