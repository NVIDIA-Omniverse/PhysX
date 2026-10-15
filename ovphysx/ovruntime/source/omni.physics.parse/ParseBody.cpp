// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-BODY-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-6
 *
 * @implements REQ-PARSE-BODY-002
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-8
 *
 * @implements REQ-SPLINE-TARGET-001
 * @covers AC-1
 *
 * @implements REQ-SIM-LOCALVELOCITY-001
 * @covers AC-3 AC-4
 */

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedBody.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>
#include <cmath>
#include <type_traits>

namespace omni::physics::parse
{

namespace
{

// PhysX rejects velocities whose square exceeds 1e16. Cap at
// SQRT_FLT_MAX = 1e16 (exactly the boundary) so descriptors produced
// here are accepted by the runtime.
constexpr float kSqrtFloatMax = 1e16f;
constexpr float kPi = 3.14159265358979323846f;
// Match omni/physx/PhysXConversions.h byte-for-byte under float32.
constexpr float kDegToRad = 0.01745329251994329547f;
constexpr float kRadToDeg = 57.29577951308232286465f;

template <typename T>
T readPreparedClamped(const PreparedAttribute<T>& input, T defaultVal, T minVal, T maxVal) noexcept
{
    if (!input.authored)
        return defaultVal;
    T value = input.valueOr(defaultVal);
    if (value < minVal) value = minVal;
    if (value > maxVal) value = maxVal;
    return value;
}

inline carb::Float3 degToRad(const carb::Float3& v)
{
    return { v.x * kDegToRad, v.y * kDegToRad, v.z * kDegToRad };
}

// Right-multiply a row-vector by a 3x3 row-major matrix: out = v * R.
// The rotation produced by IPhysicsSource::getLocalToWorldRotationAndScale
// is row-major (same convention as GfMatrix4d).
inline carb::Float3 rotateDir(const Matrix3d& rot, const carb::Float3& v)
{
    const double vx = v.x, vy = v.y, vz = v.z;
    const double* r = rot.data;
    return {
        static_cast<float>(vx * r[0] + vy * r[3] + vz * r[6]),
        static_cast<float>(vx * r[1] + vy * r[4] + vz * r[7]),
        static_cast<float>(vx * r[2] + vy * r[5] + vz * r[8]),
    };
}

inline carb::Float3 componentScale(const carb::Float3& s, const carb::Float3& v)
{
    return { s.x * v.x, s.y * v.y, s.z * v.z };
}

} // anonymous namespace

void setToDefault(DynamicPhysxRigidBodyDesc& desc, const SourceUnits& units)
{
    // Units-aware overlay only. All other defaults are in-class on
    // DynamicPhysxRigidBodyDesc (Descriptors.h).
    const float metersPerUnit = units.metersPerUnit;
    const float tolerancesSpeed = 10.0f / metersPerUnit;

    desc.sleepThreshold           = 5e-5f * tolerancesSpeed * tolerancesSpeed;
    desc.stabilizationThreshold   = 1e-5f * tolerancesSpeed * tolerancesSpeed;
    desc.maxDepenetrationVelocity = 3.0f / metersPerUnit;
}

DescPtr<DynamicPhysxRigidBodyDesc> parseDynamicBody(ParseContext& ctx, ObjectKey key)
{
    DescPtr<DynamicPhysxRigidBodyDesc> desc = allocateDesc<DynamicPhysxRigidBodyDesc>(ctx.descriptorAllocator());
    if (desc)
        fillDynamicBody(prepareDynamicBody(ctx, key), *desc);
    return desc;
}

PreparedDynamicBody prepareDynamicBody(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedDynamicBody input;
    input.units = ctx.units();
    input.outputVelocitiesLocalSpaceDefault = ctx.outputVelocitiesLocalSpaceDefault();
    input.physxRigidBodyAPI = src.hasSchema(key, tok.physxRigidBodyAPI);
    input.physxSurfaceVelocityAPI = src.hasSchema(key, tok.physxSurfaceVelocityAPI);
    input.physxSplinesSurfaceVelocityAPI = src.hasSchema(key, tok.physxSplinesSurfaceVelocityAPI);
    readDynamicBodyAttributes(src, key, tok, input);
    if (input.metadataLocalSpaceVelocities.valueOr(input.outputVelocitiesLocalSpaceDefault) ||
        (input.physxSurfaceVelocityAPI && input.physxSurfaceVelocityLocalSpace.valueOr(true)))
        src.getLocalToWorldRotationAndScale(key, input.rotation, input.scale);

    if (input.physxSplinesSurfaceVelocityAPI && input.physxSplinesSurfaceVelocityEnabled.valueOr(true))
    {
        std::vector<ObjectKey> targets;
        src.getRelationshipTargets(key, tok.physxSplinesSurfaceVelocityCurve, targets);
        if (!targets.empty() && src.isA(targets[0], tok.basisCurvesType))
        {
            ObjectKey parent = src.getParent(targets[0]);
            while (parent.valid())
            {
                if (parent == key)
                {
                    input.splineCurve = targets[0];
                    break;
                }
                const ObjectKey next = src.getParent(parent);
                if (next == parent) break;
                parent = next;
            }
        }
    }
    input.prepared = true;
    return input;
}

bool fillDynamicBody(const PreparedDynamicBody& input, DynamicPhysxRigidBodyDesc& output) noexcept
{
    if (!input.prepared)
        return false;
    DynamicPhysxRigidBodyDesc* desc = &output;
    setToDefault(*desc, input.units);

    // Basic UsdPhysicsRigidBodyAPI fields
    desc->kinematicBody  = input.physicsKinematicEnabled.valueOr(false);
    desc->startsAsleep   = input.physicsStartsAsleep.valueOr(false);

    // Local-space-velocity output mode: start from the consumer's global
    // setting (e.g. omni.physx /physics/outputVelocitiesLocalSpace) and
    // let per-prim metadata override.
    desc->localSpaceVelocities = input.outputVelocitiesLocalSpaceDefault;
    // `physics:localSpaceVelocities` is authored as customData metadata in
    // USD; UsdSource::getAttribute falls back to customData when the prim
    // has no schema-applied attribute by that name. Callers don't need to
    // know which storage the source uses.
    input.metadataLocalSpaceVelocities.read(desc->localSpaceVelocities);

    desc->linearVelocity  = input.physicsVelocity.valueOr({ 0.0f, 0.0f, 0.0f });
    // angularVelocity authored in deg/s in USD; engine-side is rad/s.
    desc->angularVelocity = degToRad(
        input.physicsAngularVelocity.valueOr({ 0.0f, 0.0f, 0.0f }));

    // Scale linear velocity in the body's local axes before rotating both
    // velocities into world space.
    if (desc->localSpaceVelocities)
    {
        Matrix3d rot;
        carb::Float3 scale;
        rot = input.rotation;
        scale = input.scale;
        desc->linearVelocity  = rotateDir(rot, componentScale(scale, desc->linearVelocity));
        desc->angularVelocity = rotateDir(rot, desc->angularVelocity);
    }

    // PhysxRigidBodyAPI extensions — only override when the API is applied.
    // For each field, the readScalar default is the current (set-to-default)
    // value, so unauthored attributes preserve the units-aware defaults.
    if (input.physxRigidBodyAPI)
    {
        // Clamping ranges enforce PhysX's documented valid bounds for
        // each field.
        desc->linearDamping             = readPreparedClamped(input.physxRigidBodyLinearDamping,  desc->linearDamping,  0.0f, FLT_MAX);
        desc->angularDamping            = readPreparedClamped(input.physxRigidBodyAngularDamping, desc->angularDamping, 0.0f, FLT_MAX);
        desc->maxLinearVelocity         = readPreparedClamped(input.physxRigidBodyMaxLinearVelocity,  desc->maxLinearVelocity,  0.0f, kSqrtFloatMax);
        // maxAngularVelocity authored in deg/s; clamp [0, SQRT_FLT_MAX] then convert to rad/s.
        {
            float deg = -1.0f;
            if (input.physxRigidBodyMaxAngularVelocity.read(deg) && deg >= 0.0f)
            {
                if (deg > kSqrtFloatMax) deg = kSqrtFloatMax;
                desc->maxAngularVelocity = deg * kDegToRad;
            }
        }
        desc->sleepThreshold            = readPreparedClamped(input.physxRigidBodySleepThreshold,  desc->sleepThreshold,  0.0f, FLT_MAX);
        desc->stabilizationThreshold    = readPreparedClamped(input.physxRigidBodyStabilizationThreshold, desc->stabilizationThreshold, 0.0f, FLT_MAX);
        desc->maxDepenetrationVelocity  = readPreparedClamped(input.physxRigidBodyMaxDepenetrationVelocity, desc->maxDepenetrationVelocity, 0.0f, FLT_MAX);
        desc->contactSlopCoefficient    = readPreparedClamped(input.physxRigidBodyContactSlopCoefficient, desc->contactSlopCoefficient, 0.0f, FLT_MAX);
        desc->maxContactImpulse         = readPreparedClamped(input.physxRigidBodyMaxContactImpulse, desc->maxContactImpulse, 0.0f, FLT_MAX);
        desc->cfmScale                  = readPreparedClamped(input.physxRigidBodyCfmScale, desc->cfmScale, 0.0f, 1.0f);

        desc->solverPositionIterationCount = readPreparedClamped(input.physxRigidBodySolverPositionIterationCount, desc->solverPositionIterationCount, 1, 255);
        desc->solverVelocityIterationCount = readPreparedClamped(input.physxRigidBodySolverVelocityIterationCount, desc->solverVelocityIterationCount, 0, 255);

        desc->enableCCD                = input.physxRigidBodyEnableCCD.valueOr(desc->enableCCD);
        desc->enableSpeculativeCCD     = input.physxRigidBodyEnableSpeculativeCCD.valueOr(desc->enableSpeculativeCCD);
        desc->disableGravity           = input.physxRigidBodyDisableGravity.valueOr(desc->disableGravity);
        desc->retainAccelerations      = input.physxRigidBodyRetainAccelerations.valueOr(desc->retainAccelerations);
        desc->enableGyroscopicForces   = input.physxRigidBodyEnableGyroscopicForces.valueOr(desc->enableGyroscopicForces);
        desc->solveContacts            = input.physxRigidBodySolveContact.valueOr(desc->solveContacts);

        desc->lockedPosAxis            = readPreparedClamped(input.physxRigidBodyLockedPosAxis, desc->lockedPosAxis, 0, 7);
        desc->lockedRotAxis            = readPreparedClamped(input.physxRigidBodyLockedRotAxis, desc->lockedRotAxis, 0, 7);
    }

    // Legacy compatibility: a kinematic body with non-zero authored velocities
    // is treated as a surface-velocity body. The new PhysxSurfaceVelocityAPI
    // overrides this if applied.
    if (desc->kinematicBody &&
        (desc->linearVelocity.x  != 0 || desc->linearVelocity.y  != 0 || desc->linearVelocity.z  != 0 ||
         desc->angularVelocity.x != 0 || desc->angularVelocity.y != 0 || desc->angularVelocity.z != 0))
    {
        desc->surfaceVelocityEnabled       = true;
        desc->surfaceVelocityLocalSpace    = false;
        desc->surfaceLinearVelocity        = desc->linearVelocity;
        desc->surfaceLinearVelocityAuthored = desc->linearVelocity;
        desc->surfaceAngularVelocity       = desc->angularVelocity;
    }

    if (input.physxSurfaceVelocityAPI)
    {
        desc->surfaceVelocityEnabled    = input.physxSurfaceVelocityEnabled.valueOr(true);
        desc->surfaceVelocityLocalSpace = input.physxSurfaceVelocityLocalSpace.valueOr(true);
        desc->surfaceLinearVelocity     = input.physxSurfaceVelocity.valueOr({ 0.0f, 0.0f, 0.0f });
        // surfaceAngularVelocity is authored in deg/s; convert to rad/s.
        desc->surfaceAngularVelocity    = degToRad(
            input.physxSurfaceAngularVelocity.valueOr({ 0.0f, 0.0f, 0.0f }));

        // Keep the authored value: the scale fold below is not invertible (a zero scale
        // component loses the axis), and the runtime needs the pre-fold quantity to
        // re-derive the effective velocity when surfaceVelocityLocalSpace changes alone.
        desc->surfaceLinearVelocityAuthored = desc->surfaceLinearVelocity;

        // Apply the prim's per-axis scale to surfaceLinearVelocity (no
        // rotation) when surfaceVelocityLocalSpace is true.
        if (desc->surfaceVelocityLocalSpace)
        {
            desc->surfaceLinearVelocity = componentScale(input.scale, desc->surfaceLinearVelocity);
        }
    }

    if (input.physxSplinesSurfaceVelocityAPI)
    {
        desc->splinesSurfaceVelocityEnabled = input.physxSplinesSurfaceVelocityEnabled.valueOr(true);

        // Surface velocity from a curve and from explicit components
        // are mutually exclusive.
        if (desc->surfaceVelocityEnabled && desc->splinesSurfaceVelocityEnabled)
            desc->splinesSurfaceVelocityEnabled = false;

        if (desc->splinesSurfaceVelocityEnabled)
        {
            desc->splinesSurfaceVelocityMagnitude = input.physxSplinesSurfaceVelocityMagnitude.valueOr(0.0f);

            if (input.splineCurve.valid())
                desc->splinesCurvePrimKey = input.splineCurve;
            else
                desc->splinesSurfaceVelocityEnabled = false;
        }
    }

    return true;
}

DescPtr<StaticPhysxRigidBodyDesc> parseStaticBody(ParseContext& ctx, ObjectKey /*key*/)
{
    // Static bodies have no PhysX-extension fields beyond what's on the base
    // PhysxRigidBodyDesc; sourceGPrimKey is filled in at create time by the
    // collider that promotes a shape-only prim to a static body.
    return allocateDesc<StaticPhysxRigidBodyDesc>(ctx.descriptorAllocator());
}

} // namespace omni::physics::parse
