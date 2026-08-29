// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-JOINT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-JOINT-002
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 */

// parse::parseJoint — minimum-viable port that consumes the already-resolved
// `JointInfo` (produced by Pixar's UsdPhysicsLoadStageFromPrimRange in the
// USD backend, or produced natively by URDF/MJCF/procedural backends) and
// adds the PhysX-extension fields on top.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>
#include <cmath>

namespace omni::physics::parse
{

namespace
{

constexpr float kPi = 3.14159265358979323846f;
// Precomputed rad↔deg constants matching omni/physx/PhysXConversions.h
// so parse-lib drive / state / property conversions are byte-equivalent
// under float32.
constexpr float kRadToDeg = 57.29577951308232286465f;
constexpr float kDegToRad = 0.01745329251994329547f;
inline float degToRad(float v) { return v * kDegToRad; }
inline float radToDeg(float v) { return v * kRadToDeg; }

template <typename T>
T readClampedScalar(const IPhysicsSource& src, ObjectKey key, TokenId attr,
                    T defaultVal, T minVal, T maxVal)
{
    // Gate on `hasAuthoredAttribute` so schema-fallback defaults (e.g.
    // `physxJoint:maxJointVelocity = 1000000`) don't leak into the
    // descriptor and override units-aware setToDefault values (e.g.
    // `FLT_MAX`).
    if (!src.hasAuthoredAttribute(key, attr))
        return defaultVal;
    T result;
    if (!src.getAttribute(key, attr, result))
        return defaultVal;
    if (result < minVal) result = minVal;
    if (result > maxVal) result = maxVal;
    return result;
}

void copyBaseFields(const JointInfo& info, PhysxJointDesc& desc)
{
    desc.body0 = info.body0;
    desc.body1 = info.body1;
    desc.rel0  = info.rel0;
    desc.rel1  = info.rel1;
    desc.localPose0Position    = info.localPose0Position;
    desc.localPose0Orientation = info.localPose0Orientation;
    desc.localPose1Position    = info.localPose1Position;
    desc.localPose1Orientation = info.localPose1Orientation;
    desc.jointEnabled          = info.jointEnabled;
    desc.breakForce            = info.breakForce;
    desc.breakTorque           = info.breakTorque;
    desc.enableCollision       = info.collisionEnabled;
    desc.excludedFromArticulation = info.excludeFromArticulation;
    // validBodyTransformations is a USD-runtime-side check; default true.
    desc.validBodyTransformations = true;
}

// PhysxJointAPI extension: jointFriction. Legacy `getAttribute(...,
// physxJointAPI.GetJointFrictionAttr(), 0.0f, FLT_MAX, ...)` clamps
// to [0, FLT_MAX] which we mirror.
void readPhysxJointApi(const IPhysicsSource& src, ObjectKey key,
                       const KnownTokens& tok, PhysxJointDesc& desc)
{
    if (!src.hasSchema(key, tok.physxJointAPI))
        return;
    desc.jointFriction = readClampedScalar<float>(
        src, key, tok.physxJointJointFriction, desc.jointFriction, 0.0f, FLT_MAX);
}

// Default extension fields to zero. The constructor only sets the
// union (angle0/lower) and `enabled`; without explicit defaulting
// these would carry garbage into PhysX.
void initLimitExtensions(PhysxJointLimit& l)
{
    l.restitution = 0.0f;
    l.bounceThreshold = 0.0f;
    l.stiffness = 0.0f;
    l.damping = 0.0f;
}

// Read PhysxLimitAPI:<instance> overrides on top of `limit`. `instance`
// is the multi-apply schema instance name (e.g. "rotX", "linear",
// "cone", "distance"). Each field is clamped to a valid PhysX range.
void readPhysxLimitApi(IPhysicsSource& src, ObjectKey key,
                       std::string_view instance, PhysxJointLimit& limit)
{
    std::string apiName = "PhysxLimitAPI:";
    apiName += instance;
    if (!src.hasSchema(key, src.internToken(apiName)))
        return;

    auto readClamped = [&](std::string_view attr, float minV, float maxV, float def) {
        std::string name = "physxLimit:";
        name += instance;
        name += ":";
        name += attr;
        float r;
        if (!src.getAttribute(key, src.internToken(name), r))
            return def;
        if (r < minV) r = minV;
        if (r > maxV) r = maxV;
        return r;
    };

    limit.restitution     = readClamped("restitution",     0.0f, FLT_MAX, limit.restitution);
    limit.bounceThreshold = readClamped("bounceThreshold", 0.0f, FLT_MAX, limit.bounceThreshold);
    limit.stiffness       = readClamped("stiffness",       0.0f, FLT_MAX, limit.stiffness);
    limit.damping         = readClamped("damping",         0.0f, FLT_MAX, limit.damping);
}

// Convert the USD-free JointInfo per-axis limit into the PhysX-extended
// PhysxJointLimit, applying a deg→rad conversion for rotational axes.
PhysxJointLimit makePhysxLimit(const JointLimitInfo& info, JointAxis ax)
{
    PhysxJointLimit out;
    initLimitExtensions(out);
    out.enabled = info.enabled;
    const bool isRotational = (ax == eRotX || ax == eRotY || ax == eRotZ);
    out.lower = isRotational ? degToRad(info.lower) : info.lower;
    out.upper = isRotational ? degToRad(info.upper) : info.upper;
    return out;
}

PhysxJointLimit makeAngularLimit(const JointLimitInfo& info)
{
    // Spherical limit: angle0 = degToRad(angle0), angle1 = degToRad(angle1).
    PhysxJointLimit out;
    initLimitExtensions(out);
    out.enabled = info.enabled;
    out.angle0 = degToRad(info.lower);  // schema::JointLimit::angle0 ↔ lower
    out.angle1 = degToRad(info.upper);
    return out;
}

PhysxJointLimit makeLinearLimit(const JointLimitInfo& info)
{
    PhysxJointLimit out;
    initLimitExtensions(out);
    out.enabled = info.enabled;
    out.lower = info.lower;
    out.upper = info.upper;
    return out;
}

// Multi-apply schema instance for the PhysxLimitAPI on a per-axis limit
// (D6 case). For non-D6 joints the instance is type-specific:
// Spherical=cone, Revolute=angular, Prismatic=linear, Distance=distance.
const char* limitInstanceForAxis(JointAxis ax)
{
    switch (ax)
    {
    case eDistance: return "distance";
    case eTransX:   return "transX";
    case eTransY:   return "transY";
    case eTransZ:   return "transZ";
    case eRotX:     return "rotX";
    case eRotY:     return "rotY";
    case eRotZ:     return "rotZ";
    }
    return "";
}

// Default PhysxJointAxisProperties' five extension fields.
void initAxisPropertiesDefaults(PhysxJointAxisProperties& p)
{
    p.armature = 0.0f;
    p.maxJointVelocity = FLT_MAX;
    p.staticFrictionEffort = 0.0f;
    p.dynamicFrictionEffort = 0.0f;
    p.viscousFrictionCoefficient = 0.0f;
}

// Read PhysxJointAxisAPI:<instance> overrides on top of `properties`.
// `instance` is the multi-apply schema instance ("linear", "angular",
// "rotX", "rotY", "rotZ"). When the API is *not* applied for the axis,
// falls back to PhysxJointAPI's `armature` and `maxJointVelocity` (the
// friction fields stay at default).
//
// The caller is responsible for the deg→rad / rad→deg conversions on
// rotational axes — different joint types apply them slightly differently
// (D6/Spherical: per axis; Revolute: once; Prismatic: never), and doing
// it here would force the caller to convert back for the linear axis
// case.
void readPhysxJointAxisApi(IPhysicsSource& src, ObjectKey key,
                           const KnownTokens& tok,
                           std::string_view instance,
                           PhysxJointAxisProperties& properties)
{
    initAxisPropertiesDefaults(properties);

    std::string apiName = "PhysxJointAxisAPI:";
    apiName += instance;
    const TokenId apiToken = src.internToken(apiName);

    if (src.hasSchema(key, apiToken))
    {
        auto readClamped = [&](std::string_view field, float minV, float maxV, float def) {
            std::string name = "physxJointAxis:";
            name += instance;
            name += ":";
            name += field;
            const TokenId nameTok = src.internToken(name);
            // Gate on `hasAuthoredAttribute` (mirrors `readClampedScalar`): only an
            // authored override should displace the default. USD's getAttribute returns
            // the schema fallback (e.g. maxJointVelocity = inf) for a declared-but-
            // unauthored attribute, while ovstage's population may materialize the type
            // default (0); gating keeps the intended default (FLT_MAX) under both.
            if (!src.hasAuthoredAttribute(key, nameTok))
                return def;
            float r;
            if (!src.getAttribute(key, nameTok, r))
                return def;
            if (r < minV) r = minV;
            if (r > maxV) r = maxV;
            return r;
        };

        properties.armature                  = readClamped("armature",                  0.0f, FLT_MAX, properties.armature);
        properties.maxJointVelocity          = readClamped("maxJointVelocity",          0.0f, FLT_MAX, properties.maxJointVelocity);
        properties.staticFrictionEffort      = readClamped("staticFrictionEffort",      0.0f, FLT_MAX, properties.staticFrictionEffort);
        properties.dynamicFrictionEffort     = readClamped("dynamicFrictionEffort",     0.0f, FLT_MAX, properties.dynamicFrictionEffort);
        properties.viscousFrictionCoefficient = readClamped("viscousFrictionCoefficient", 0.0f, FLT_MAX, properties.viscousFrictionCoefficient);
        return;
    }

    // PhysxJointAPI fallback: when the per-axis API isn't applied,
    // populate `armature` and `maxJointVelocity` from PhysxJointAPI.
    // The friction fields stay at their default.
    if (src.hasSchema(key, tok.physxJointAPI))
    {
        properties.maxJointVelocity = readClampedScalar<float>(
            src, key, tok.physxJointMaxJointVelocity, properties.maxJointVelocity, 0.0f, FLT_MAX);
        properties.armature = readClampedScalar<float>(
            src, key, tok.physxJointArmature, properties.armature, 0.0f, FLT_MAX);
    }
}

// Apply the deg→rad / rad→deg conversion required for rotational axes
// after reading PhysxJointAxisAPI overrides. Linear axes pass through
// unchanged.
void applyRotationalAxisConversions(PhysxJointAxisProperties& p)
{
    // FLT_MAX is the "unlimited" sentinel (PhysX's default maxJointVelocity, and
    // what the change-update path maps the unlimited case to). Running it through
    // degToRad would scale it to a finite, no-longer-unlimited value (~5.9e36),
    // which then disagrees with the change-update representation -- visible under
    // ovstage, where re-attach re-parses but no per-attribute change fires to
    // correct it. Only convert genuine authored (sub-sentinel) velocities deg/s -> rad/s.
    if (p.maxJointVelocity < FLT_MAX)
        p.maxJointVelocity = degToRad(p.maxJointVelocity);
    // viscousFrictionCoefficient is torque·s/deg in USD; convert to torque·s/rad
    // for runtime, which is multiplication by 180/pi (i.e. radToDeg).
    p.viscousFrictionCoefficient = p.viscousFrictionCoefficient * kRadToDeg;
}

PhysxJointDrive makePhysxDrive(const JointDriveInfo& info, bool convertToRad)
{
    PhysxJointDrive out;
    out.enabled = info.enabled;
    out.acceleration = info.acceleration;
    out.forceLimit = std::isfinite(info.forceLimit) ? info.forceLimit : FLT_MAX;
    out.stiffness = convertToRad ? (info.stiffness * kRadToDeg) : info.stiffness;
    out.damping   = convertToRad ? (info.damping   * kRadToDeg) : info.damping;
    out.targetPosition = convertToRad ? degToRad(info.targetPosition) : info.targetPosition;
    out.targetVelocity = convertToRad ? degToRad(info.targetVelocity) : info.targetVelocity;
    return out;
}

// Read PhysxDrivePerformanceEnvelopeAPI:<instance> overrides on top of
// `drive`. `instance` is the multi-apply schema instance ("linear",
// "angular", "rotX", "rotY", "rotZ"). When the API is applied:
//   - drive.isEnvelopeUsed = true,
//   - maxActuatorVelocity / velocityDependentResistance / speedEffortGradient
//     are read (clamped [0, FLT_MAX]),
//   - rotational instances (everything except "linear") apply a deg→rad /
//     rad→deg conversion after the read.
// When the API is *not* applied, the envelope fields stay at their
// constructor defaults (isEnvelopeUsed=false, maxActuatorVelocity=FLT_MAX,
// velocityDependentResistance=0, speedEffortGradient=0).
void readPhysxDrivePerformanceEnvelopeApi(IPhysicsSource& src, ObjectKey key,
                                          std::string_view instance,
                                          PhysxJointDrive& drive)
{
    std::string apiName = "PhysxDrivePerformanceEnvelopeAPI:";
    apiName += instance;
    if (!src.hasSchema(key, src.internToken(apiName)))
        return;

    drive.isEnvelopeUsed = true;

    auto readClamped = [&](std::string_view field, float minV, float maxV, float def) {
        std::string name = "physxDrivePerformanceEnvelope:";
        name += instance;
        name += ":";
        name += field;
        float r;
        if (!src.getAttribute(key, src.internToken(name), r))
            return def;
        if (r < minV) r = minV;
        if (r > maxV) r = maxV;
        return r;
    };

    drive.maxActuatorVelocity         = readClamped("maxActuatorVelocity",         0.0f, FLT_MAX, drive.maxActuatorVelocity);
    drive.velocityDependentResistance = readClamped("velocityDependentResistance", 0.0f, FLT_MAX, drive.velocityDependentResistance);
    drive.speedEffortGradient         = readClamped("speedEffortGradient",         0.0f, FLT_MAX, drive.speedEffortGradient);

    // Rotational instances ("angular", "rotX", "rotY", "rotZ") need deg→rad /
    // rad→deg conversion. "linear" passes through unchanged.
    if (instance != "linear")
    {
        if (drive.maxActuatorVelocity < FLT_MAX)
            drive.maxActuatorVelocity = degToRad(drive.maxActuatorVelocity);
        drive.velocityDependentResistance = drive.velocityDependentResistance * kRadToDeg; // torque*s/deg → torque*s/rad
        drive.speedEffortGradient         = degToRad(drive.speedEffortGradient);
    }
}

// Read PhysicsJointStateAPI:<instance> on top of `state`. Schema name:
// `PhysicsJointStateAPI:<instance>`; attributes:
// `state:<instance>:physics:position` / `:velocity`, clamped to
// [-FLT_MAX, FLT_MAX].
//
// `applyDegToRad` selects whether the read values are converted from
// schema-authored degrees to engine-native radians for rotational
// instances:
//   - Revolute "angular" / D6 "rotX|rotY|rotZ": rotational. Pass true.
//     Engine-side state is in radians, so degToRad is applied here.
//   - Prismatic "linear" / D6 "transX|transY|transZ": linear. Pass false.
//     No conversion needed.
//
// When the API is not applied, leaves `state` at its constructor default
// (enabled=false, position=0, velocity=0).
void readPhysicsJointStateApi(IPhysicsSource& src, ObjectKey key,
                              std::string_view instance,
                              PhysicsJointState& state,
                              bool applyDegToRad)
{
    std::string apiName = "PhysicsJointStateAPI:";
    apiName += instance;
    if (!src.hasSchema(key, src.internToken(apiName)))
        return;

    state.enabled = true;

    auto readClamped = [&](std::string_view field, float minV, float maxV, float def) {
        std::string name = "state:";
        name += instance;
        name += ":physics:";
        name += field;
        float r;
        if (!src.getAttribute(key, src.internToken(name), r))
            return def;
        if (r < minV) r = minV;
        if (r > maxV) r = maxV;
        return r;
    };

    state.position = readClamped("position", -FLT_MAX, FLT_MAX, state.position);
    state.velocity = readClamped("velocity", -FLT_MAX, FLT_MAX, state.velocity);

    if (applyDegToRad)
    {
        state.position = degToRad(state.position);
        state.velocity = degToRad(state.velocity);
    }
}

bool isAxisRotational(JointAxis ax)
{
    return ax == eRotX || ax == eRotY || ax == eRotZ;
}

} // anonymous namespace

DescPtr<PhysxJointDesc> parseJoint(ParseContext& ctx, ObjectKey key, const JointInfo& info)
{
    IPhysicsSource& src = ctx.source();
    KnownTokens tok;
    tok.intern(src);

    DescPtr<PhysxJointDesc> base;

    switch (info.type)
    {
    case eJointFixed:
    {
        DescPtr<FixedPhysxJointDesc> d = allocateDesc<FixedPhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    case eJointSpherical:
    {
        DescPtr<SphericalPhysxJointDesc> d = allocateDesc<SphericalPhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        d->axis = info.axis;
        d->limit = makeAngularLimit(info.limit);
        // Spherical joint's limit uses the multi-apply instance "cone".
        readPhysxLimitApi(src, key, "cone", d->limit);
        // PhysxJointAxisAPI per rotational axis (rotX, rotY, rotZ). Legacy
        // populates jointProperties for all three and applies the rotational
        // deg→rad / rad→deg conversion to each.
        for (JointAxis ax : { eRotX, eRotY, eRotZ })
        {
            PhysxJointAxisProperties props;
            readPhysxJointAxisApi(src, key, tok, limitInstanceForAxis(ax), props);
            applyRotationalAxisConversions(props);
            d->jointProperties.push_back(std::make_pair(ax, props));
        }
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    case eJointRevolute:
    {
        DescPtr<RevolutePhysxJointDesc> d = allocateDesc<RevolutePhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        d->axis = info.axis;
        d->limit = makeAngularLimit(info.limit);
        d->drive = makePhysxDrive(info.drive, /*convertToRad=*/true);
        // Revolute joint uses the "angular" instance for limit + axis-properties.
        readPhysxLimitApi(src, key, "angular", d->limit);
        readPhysxJointAxisApi(src, key, tok, "angular", d->properties);
        applyRotationalAxisConversions(d->properties);
        readPhysxDrivePerformanceEnvelopeApi(src, key, "angular", d->drive);
        // Joint state for "angular" instance. Rotational → convert
        // schema-authored degrees to engine-native radians.
        readPhysicsJointStateApi(src, key, "angular", d->state, /*applyDegToRad=*/true);
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    case eJointPrismatic:
    {
        DescPtr<PrismaticPhysxJointDesc> d = allocateDesc<PrismaticPhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        d->axis = info.axis;
        d->limit = makeLinearLimit(info.limit);
        d->drive = makePhysxDrive(info.drive, /*convertToRad=*/false);
        // Prismatic joint uses the "linear" instance.
        readPhysxLimitApi(src, key, "linear", d->limit);
        readPhysxJointAxisApi(src, key, tok, "linear", d->properties);
        // No rotational conversion for the linear axis.
        readPhysxDrivePerformanceEnvelopeApi(src, key, "linear", d->drive);
        // Joint state for "linear" instance, no conversion.
        readPhysicsJointStateApi(src, key, "linear", d->state, /*applyDegToRad=*/false);
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    case eJointDistance:
    {
        DescPtr<DistancePhysxJointDesc> d = allocateDesc<DistancePhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        d->minEnabled = info.minEnabled;
        d->maxEnabled = info.maxEnabled;
        d->limit = makeLinearLimit(info.limit);
        readPhysxLimitApi(src, key, "distance", d->limit);
        // PhysxPhysicsDistanceJointAPI extension: spring enabled +
        // stiffness + damping.
        const TokenId distApiTok = src.internToken("PhysxPhysicsDistanceJointAPI");
        if (src.hasSchema(key, distApiTok))
        {
            src.getAttribute(key, src.internToken("physxPhysicsDistanceJoint:springEnabled"), d->springEnabled);
            d->stiffness = readClampedScalar<float>(
                src, key, src.internToken("physxPhysicsDistanceJoint:springStiffness"),
                d->stiffness, 0.0f, FLT_MAX);
            d->damping = readClampedScalar<float>(
                src, key, src.internToken("physxPhysicsDistanceJoint:springDamping"),
                d->damping, 0.0f, FLT_MAX);
        }
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    case eJointD6:
    {
        DescPtr<D6PhysxJointDesc> d = allocateDesc<D6PhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        d->jointLimits.reserve(info.jointLimits.size());
        for (const auto& [ax, lim] : info.jointLimits)
        {
            PhysxJointLimit limit = makePhysxLimit(lim, ax);
            readPhysxLimitApi(src, key, limitInstanceForAxis(ax), limit);
            d->jointLimits.push_back(std::make_pair(ax, limit));
        }
        d->jointDrives.reserve(info.jointDrives.size());
        for (const auto& [ax, drv] : info.jointDrives)
        {
            PhysxJointDrive physxDrive = makePhysxDrive(drv, isAxisRotational(ax));
            // Legacy reads PhysxDrivePerformanceEnvelopeAPI only for the
            // rotational D6 drives — translational drives skip envelope.
            if (isAxisRotational(ax))
                readPhysxDrivePerformanceEnvelopeApi(src, key, limitInstanceForAxis(ax), physxDrive);
            d->jointDrives.push_back(std::make_pair(ax, physxDrive));
        }
        // PhysxJointAxisAPI per rotational axis (rotX, rotY, rotZ). Legacy
        // populates jointProperties only for rotational axes on D6 (the
        // translational axes are not authored through this API).
        for (JointAxis ax : { eRotX, eRotY, eRotZ })
        {
            PhysxJointAxisProperties props;
            readPhysxJointAxisApi(src, key, tok, limitInstanceForAxis(ax), props);
            applyRotationalAxisConversions(props);
            d->jointProperties.push_back(std::make_pair(ax, props));
        }
        // PhysicsJointStateAPI per axis on D6. Legacy iterates trans first
        // then rot (axisVector order) and only pushes entries where the API
        // is applied. Rotational axes apply degToRad in-line.
        for (JointAxis ax : { eTransX, eTransY, eTransZ, eRotX, eRotY, eRotZ })
        {
            PhysicsJointState state;
            const bool isRot = isAxisRotational(ax);
            readPhysicsJointStateApi(src, key, limitInstanceForAxis(ax), state, /*applyDegToRad=*/isRot);
            if (state.enabled)
                d->jointStates.push_back(std::make_pair(ax, state));
        }
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    case eJointGear:
    {
        // Built-in gear joint: capture gearRatio + hinge0/hinge1 targets so the
        // consumer needs no USD re-read (it converts the ObjectKeys to paths).
        DescPtr<GearPhysxJointDesc> d = allocateDesc<GearPhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        src.getAttribute(key, src.internToken("physics:gearRatio"), d->gearRatio);
        std::vector<ObjectKey> targets;
        src.getRelationshipTargets(key, src.internToken("physics:hinge0"), targets);
        if (!targets.empty())
            d->hingePrimPath0 = targets.front();
        targets.clear();
        src.getRelationshipTargets(key, src.internToken("physics:hinge1"), targets);
        if (!targets.empty())
            d->hingePrimPath1 = targets.front();
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    case eJointRackAndPinion:
    {
        // Built-in rack-and-pinion joint: capture ratio + hinge/prismatic targets.
        DescPtr<RackPhysxJointDesc> d = allocateDesc<RackPhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        src.getAttribute(key, src.internToken("physics:ratio"), d->ratio);
        std::vector<ObjectKey> targets;
        src.getRelationshipTargets(key, src.internToken("physics:hinge"), targets);
        if (!targets.empty())
            d->hingePrimKey = targets.front();
        targets.clear();
        src.getRelationshipTargets(key, src.internToken("physics:prismatic"), targets);
        if (!targets.empty())
            d->prismaticPrimKey = targets.front();
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    case eJointCustom:
    {
        // Third-party registered custom joints emit a CustomPhysxJointDesc with
        // default-empty customJointToken; the consumer fills the token in from
        // the prim type via the CustomJointManager. (Built-in gear/rack are
        // handled as typed joints above.)
        DescPtr<CustomPhysxJointDesc> d = allocateDesc<CustomPhysxJointDesc>(ctx.descriptorAllocator());
        copyBaseFields(info, *d);
        base = descPtrCast<PhysxJointDesc>(std::move(d));
        break;
    }
    default:
    {
        // Any unrecognised types fall through with the base.
        DescPtr<PhysxJointDesc> d = allocateDesc<PhysxJointDesc>(ctx.descriptorAllocator());
        d->type = info.type;
        copyBaseFields(info, *d);
        base = std::move(d);
        break;
    }
    }

    base->jointPrimKey = key;
    readPhysxJointApi(src, key, tok, *base);
    return base;
}

} // namespace omni::physics::parse
