// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-BODY-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-BODY-002
 * @covers AC-1
 */

#include <omni/physics/parse/ParseApi.h>
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
T readScalar(const IPhysicsSource& src, ObjectKey key, TokenId attr, T defaultVal)
{
    if constexpr (std::is_same_v<T, int>)
    {
        int64_t v = defaultVal;
        src.getAttribute(key, attr, v);
        return static_cast<int>(v);
    }
    else
    {
        T v = defaultVal;
        src.getAttribute(key, attr, v);
        return v;
    }
}

// Read an attribute with [min, max] clamping. Several PhysxRigidBodyAPI
// attributes have schema defaults of `inf` (e.g., maxLinearVelocity);
// without clamping we'd hand `inf` to PhysX, which rejects it as an
// invalid float.
//
// Gates on `hasAuthoredAttribute` — schema defaults are intentionally
// NOT applied because the caller pre-loaded `defaultVal` with a
// units-scaled default (e.g. sleepThreshold's
// `5e-5 * tolerancesSpeed^2`). Without the gate, an unauthored
// attribute would return its raw schema default and silently overwrite
// the units-scaled default.
template <typename T>
T readScalarClamped(const IPhysicsSource& src, ObjectKey key, TokenId attr,
                    T defaultVal, T minVal, T maxVal)
{
    if (!src.hasAuthoredAttribute(key, attr))
        return defaultVal;
    T v = readScalar<T>(src, key, attr, defaultVal);
    if (v < minVal) v = minVal;
    if (v > maxVal) v = maxVal;
    return v;
}

carb::Float3 readFloat3(const IPhysicsSource& src, ObjectKey key, TokenId attr,
                       const carb::Float3& defaultVal)
{
    carb::Float3 v = defaultVal;
    src.getAttribute(key, attr, v);
    return v;
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
    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);

    DescPtr<DynamicPhysxRigidBodyDesc> desc =
        allocateDesc<DynamicPhysxRigidBodyDesc>(ctx.descriptorAllocator());
    setToDefault(*desc, ctx.units());

    // Basic UsdPhysicsRigidBodyAPI fields
    desc->kinematicBody  = readScalar<bool>(src, key, tok.physicsKinematicEnabled, false);
    desc->startsAsleep   = readScalar<bool>(src, key, tok.physicsStartsAsleep, false);

    // Local-space-velocity output mode: start from the consumer's global
    // setting (e.g. omni.physx /physics/outputVelocitiesLocalSpace) and
    // let per-prim metadata override.
    desc->localSpaceVelocities = ctx.outputVelocitiesLocalSpaceDefault();
    // `physics:localSpaceVelocities` is authored as customData metadata in
    // USD; UsdSource::getAttribute falls back to customData when the prim
    // has no schema-applied attribute by that name. Callers don't need to
    // know which storage the source uses.
    src.getAttribute(key, tok.metadataLocalSpaceVelocities, desc->localSpaceVelocities);

    desc->linearVelocity  = readFloat3(src, key, tok.physicsVelocity,        { 0.0f, 0.0f, 0.0f });
    // angularVelocity authored in deg/s in USD; engine-side is rad/s.
    desc->angularVelocity = degToRad(
        readFloat3(src, key, tok.physicsAngularVelocity, { 0.0f, 0.0f, 0.0f }));

    // When localSpaceVelocities is on, apply the prim's rotation (and
    // per-axis scale, on linear only) so the authored vector becomes a
    // world-space velocity.
    if (desc->localSpaceVelocities)
    {
        Matrix3d rot;
        carb::Float3 scale;
        src.getLocalToWorldRotationAndScale(key, rot, scale);
        desc->linearVelocity  = componentScale(scale, rotateDir(rot, desc->linearVelocity));
        desc->angularVelocity = rotateDir(rot, desc->angularVelocity);
    }

    // PhysxRigidBodyAPI extensions — only override when the API is applied.
    // For each field, the readScalar default is the current (set-to-default)
    // value, so unauthored attributes preserve the units-aware defaults.
    if (src.hasSchema(key, tok.physxRigidBodyAPI))
    {
        // Clamping ranges enforce PhysX's documented valid bounds for
        // each field.
        desc->linearDamping             = readScalarClamped<float>(src, key, tok.physxRigidBodyLinearDamping,  desc->linearDamping,  0.0f, FLT_MAX);
        desc->angularDamping            = readScalarClamped<float>(src, key, tok.physxRigidBodyAngularDamping, desc->angularDamping, 0.0f, FLT_MAX);
        desc->maxLinearVelocity         = readScalarClamped<float>(src, key, tok.physxRigidBodyMaxLinearVelocity,  desc->maxLinearVelocity,  0.0f, kSqrtFloatMax);
        // maxAngularVelocity authored in deg/s; clamp [0, SQRT_FLT_MAX] then convert to rad/s.
        {
            float deg = -1.0f;
            if (src.getAttribute(key, tok.physxRigidBodyMaxAngularVelocity, deg) && deg >= 0.0f)
            {
                if (deg > kSqrtFloatMax) deg = kSqrtFloatMax;
                desc->maxAngularVelocity = deg * kDegToRad;
            }
        }
        desc->sleepThreshold            = readScalarClamped<float>(src, key, tok.physxRigidBodySleepThreshold,  desc->sleepThreshold,  0.0f, FLT_MAX);
        desc->stabilizationThreshold    = readScalarClamped<float>(src, key, tok.physxRigidBodyStabilizationThreshold, desc->stabilizationThreshold, 0.0f, FLT_MAX);
        desc->maxDepenetrationVelocity  = readScalarClamped<float>(src, key, tok.physxRigidBodyMaxDepenetrationVelocity, desc->maxDepenetrationVelocity, 0.0f, FLT_MAX);
        desc->contactSlopCoefficient    = readScalarClamped<float>(src, key, tok.physxRigidBodyContactSlopCoefficient, desc->contactSlopCoefficient, 0.0f, FLT_MAX);
        desc->maxContactImpulse         = readScalarClamped<float>(src, key, tok.physxRigidBodyMaxContactImpulse, desc->maxContactImpulse, 0.0f, FLT_MAX);
        desc->cfmScale                  = readScalarClamped<float>(src, key, tok.physxRigidBodyCfmScale, desc->cfmScale, 0.0f, 1.0f);

        desc->solverPositionIterationCount = readScalarClamped<int>(src, key, tok.physxRigidBodySolverPositionIterationCount, desc->solverPositionIterationCount, 1, 255);
        desc->solverVelocityIterationCount = readScalarClamped<int>(src, key, tok.physxRigidBodySolverVelocityIterationCount, desc->solverVelocityIterationCount, 0, 255);

        desc->enableCCD                = readScalar<bool>(src, key, tok.physxRigidBodyEnableCCD, desc->enableCCD);
        desc->enableSpeculativeCCD     = readScalar<bool>(src, key, tok.physxRigidBodyEnableSpeculativeCCD, desc->enableSpeculativeCCD);
        desc->disableGravity           = readScalar<bool>(src, key, tok.physxRigidBodyDisableGravity, desc->disableGravity);
        desc->retainAccelerations      = readScalar<bool>(src, key, tok.physxRigidBodyRetainAccelerations, desc->retainAccelerations);
        desc->enableGyroscopicForces   = readScalar<bool>(src, key, tok.physxRigidBodyEnableGyroscopicForces, desc->enableGyroscopicForces);
        desc->solveContacts            = readScalar<bool>(src, key, tok.physxRigidBodySolveContact, desc->solveContacts);

        desc->lockedPosAxis            = readScalarClamped<int>(src, key, tok.physxRigidBodyLockedPosAxis, desc->lockedPosAxis, 0, 7);
        desc->lockedRotAxis            = readScalarClamped<int>(src, key, tok.physxRigidBodyLockedRotAxis, desc->lockedRotAxis, 0, 7);
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
        desc->surfaceAngularVelocity       = desc->angularVelocity;
    }

    if (src.hasSchema(key, tok.physxSurfaceVelocityAPI))
    {
        desc->surfaceVelocityEnabled    = readScalar<bool>(src, key, tok.physxSurfaceVelocityEnabled, true);
        desc->surfaceVelocityLocalSpace = readScalar<bool>(src, key, tok.physxSurfaceVelocityLocalSpace, true);
        desc->surfaceLinearVelocity     = readFloat3(src, key, tok.physxSurfaceVelocity, { 0.0f, 0.0f, 0.0f });
        // surfaceAngularVelocity is authored in deg/s; convert to rad/s.
        desc->surfaceAngularVelocity    = degToRad(
            readFloat3(src, key, tok.physxSurfaceAngularVelocity, { 0.0f, 0.0f, 0.0f }));

        // Apply the prim's per-axis scale to surfaceLinearVelocity (no
        // rotation) when surfaceVelocityLocalSpace is true.
        if (desc->surfaceVelocityLocalSpace)
        {
            Matrix3d rot;
            carb::Float3 scale;
            src.getLocalToWorldRotationAndScale(key, rot, scale);
            desc->surfaceLinearVelocity = componentScale(scale, desc->surfaceLinearVelocity);
        }
    }

    if (src.hasSchema(key, tok.physxSplinesSurfaceVelocityAPI))
    {
        desc->splinesSurfaceVelocityEnabled = readScalar<bool>(src, key, tok.physxSplinesSurfaceVelocityEnabled, true);

        // Surface velocity from a curve and from explicit components
        // are mutually exclusive.
        if (desc->surfaceVelocityEnabled && desc->splinesSurfaceVelocityEnabled)
            desc->splinesSurfaceVelocityEnabled = false;

        if (desc->splinesSurfaceVelocityEnabled)
        {
            desc->splinesSurfaceVelocityMagnitude = readScalar<float>(
                src, key, tok.physxSplinesSurfaceVelocityMagnitude, 0.0f);

            std::vector<ObjectKey> curveTargets;
            src.getRelationshipTargets(key, tok.physxSplinesSurfaceVelocityCurve, curveTargets);
            if (curveTargets.empty())
            {
                desc->splinesSurfaceVelocityEnabled = false;
            }
            else
            {
                ObjectKey curve = curveTargets[0];
                // Legacy validations (PhysicsBody.cpp::parseRigidBody):
                //   1. Curve target must be a UsdGeomBasisCurves prim.
                //   2. Curve must be a descendant of the body in the hierarchy.
                bool isCurve = src.hasSchema(curve, tok.basisCurvesType);
                bool isDescendant = false;
                if (isCurve)
                {
                    ObjectKey parent = src.getParent(curve);
                    while (parent.handle != 0)
                    {
                        if (parent.handle == key.handle)
                        {
                            isDescendant = true;
                            break;
                        }
                        ObjectKey next = src.getParent(parent);
                        if (next.handle == parent.handle) break;
                        parent = next;
                    }
                }
                if (isCurve && isDescendant)
                    desc->splinesCurvePrimKey = curve;
                else
                    desc->splinesSurfaceVelocityEnabled = false;
            }
        }
    }

    return desc;
}

DescPtr<StaticPhysxRigidBodyDesc> parseStaticBody(ParseContext& ctx, ObjectKey /*key*/)
{
    // Static bodies have no PhysX-extension fields beyond what's on the base
    // PhysxRigidBodyDesc; sourceGPrimKey is filled in at create time by the
    // collider that promotes a shape-only prim to a static body.
    return allocateDesc<StaticPhysxRigidBodyDesc>(ctx.descriptorAllocator());
}

} // namespace omni::physics::parse
