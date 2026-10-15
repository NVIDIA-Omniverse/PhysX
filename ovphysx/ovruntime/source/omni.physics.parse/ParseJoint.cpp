// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-NEWTON-JOINT-001
 * @covers AC-1 AC-2 AC-3 AC-4
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-PARSE-JOINT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-JOINT-002
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-JOINT-005
 * @covers AC-1 AC-2 AC-5
 *
 * @implements REQ-PARSE-JOINT-006
 * @covers AC-6
 *
 * @implements REQ-SIM-ARTJOINTDRIVE-002
 * @covers AC-2
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-2
 */

// parse::parseJoint — minimum-viable port that consumes the already-resolved
// `JointInfo` (produced by Pixar's UsdPhysicsLoadStageFromPrimRange in the
// USD backend, or produced natively by URDF/MJCF/procedural backends) and
// adds the PhysX-extension fields on top.

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedJoint.h>
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

float clamped(const PreparedAttribute<float>& input, float fallback, float minVal = 0.0f, float maxVal = FLT_MAX) noexcept
{
    float value = input.valueOr(fallback);
    if (value < minVal) value = minVal;
    if (value > maxVal) value = maxVal;
    return value;
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
void fillLimit(const PreparedJointInstance& input, PhysxJointLimit& limit) noexcept
{
    if (!input.limit) return;
    limit.restitution = clamped(input.restitution, limit.restitution);
    limit.bounceThreshold = clamped(input.bounceThreshold, limit.bounceThreshold);
    limit.stiffness = clamped(input.stiffness, limit.stiffness);
    limit.damping = clamped(input.damping, limit.damping);
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
// "rotX", "rotY", "rotZ"). An axis without the API keeps the NewtonJointAPI
// joint-level values or the defaults.
//
// The caller is responsible for the deg→rad / rad→deg conversions on
// rotational axes — different joint types apply them slightly differently
// (D6/Spherical: per axis; Revolute: once; Prismatic: never), and doing
// it here would force the caller to convert back for the linear axis
// case.
void fillAxis(const PreparedJoint& joint, const PreparedJointInstance& input,
              PhysxJointAxisProperties& properties) noexcept
{
    initAxisPropertiesDefaults(properties);
    // Preserve the Newton seeds where the per-axis PhysX values have no opinion.
    properties.maxJointVelocity = clamped(joint.newtonVelocity, properties.maxJointVelocity);
    properties.armature = clamped(joint.newtonArmature, properties.armature);
    properties.staticFrictionEffort = clamped(joint.newtonFriction, properties.staticFrictionEffort);
    properties.dynamicFrictionEffort = properties.staticFrictionEffort;
    properties.viscousFrictionCoefficient = clamped(joint.newtonDamping, properties.viscousFrictionCoefficient);
    if (input.axis)
    {
        const float axisArmature = clamped(input.armature, 0.0f);
        if (hasAxisArmatureOpinion(axisArmature)) properties.armature = axisArmature;
        const float axisVelocity = clamped(input.maxJointVelocity, FLT_MAX);
        if (hasAxisVelocityLimitOpinion(axisVelocity)) properties.maxJointVelocity = axisVelocity;
        const float staticFriction = clamped(input.staticFrictionEffort, 0.0f);
        const float dynamicFriction = clamped(input.dynamicFrictionEffort, 0.0f);
        if (hasAxisFrictionOpinion(staticFriction, dynamicFriction))
        {
            properties.staticFrictionEffort = staticFriction;
            properties.dynamicFrictionEffort = dynamicFriction;
        }
        const float viscousFriction = clamped(input.viscousFrictionCoefficient, 0.0f);
        if (hasAxisViscousFrictionOpinion(viscousFriction)) properties.viscousFrictionCoefficient = viscousFriction;
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
//     are read (clamped [0, FLT_MAX]; NaN keeps the default, as in the
//     runtime envelope edits),
//   - rotational instances (everything except "linear") apply a deg→rad /
//     rad→deg conversion after the read.
// When the API is *not* applied, the envelope fields stay at their
// constructor defaults (isEnvelopeUsed=false, maxActuatorVelocity=FLT_MAX,
// velocityDependentResistance=0, speedEffortGradient=0).
void fillEnvelope(const PreparedJointInstance& input, bool rotational, PhysxJointDrive& drive) noexcept
{
    if (!input.envelope) return;
    auto envelopeTerm = [](const PreparedAttribute<float>& term, float fallback) {
        const float value = clamped(term, fallback);
        return std::isnan(value) ? fallback : value;
    };
    drive.isEnvelopeUsed = true;
    drive.maxActuatorVelocity = envelopeTerm(input.maxActuatorVelocity, drive.maxActuatorVelocity);
    drive.velocityDependentResistance = envelopeTerm(input.velocityDependentResistance, drive.velocityDependentResistance);
    drive.speedEffortGradient = envelopeTerm(input.speedEffortGradient, drive.speedEffortGradient);
    if (rotational)
    {
        if (drive.maxActuatorVelocity < FLT_MAX)
            drive.maxActuatorVelocity = degToRad(drive.maxActuatorVelocity);
        drive.velocityDependentResistance *= kRadToDeg;
        drive.speedEffortGradient = degToRad(drive.speedEffortGradient);
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
void fillState(const PreparedJointInstance& input, PhysicsJointState& state, bool rotational) noexcept
{
    if (!input.state) return;
    state.enabled = true;
    state.position = clamped(input.position, state.position, -FLT_MAX, FLT_MAX);
    state.velocity = clamped(input.velocity, state.velocity, -FLT_MAX, FLT_MAX);
    if (rotational)
    {
        state.position = degToRad(state.position);
        state.velocity = degToRad(state.velocity);
    }
}

bool isAxisRotational(JointAxis ax)
{
    return ax == eRotX || ax == eRotY || ax == eRotZ;
}

void prepareInstanceStructure(ParseContext& ctx, ObjectKey key, size_t index,
                              bool wantLimit, bool wantAxis, bool wantEnvelope, bool wantState,
                              PreparedJointInstance& input)
{
    const auto& src = ctx.source();
    const auto& names = ctx.jointReadTokens().instances[index];
    const auto& present = ctx.jointSchemaPresence();
    input.limit = wantLimit && present.physxLimit && src.hasSchema(key, names.limitAPI);
    input.axis = wantAxis && present.physxJointAxis && src.hasSchema(key, names.axisAPI);
    input.envelope = wantEnvelope && present.drivePerfEnvelope && src.hasSchema(key, names.envelopeAPI);
    input.state = wantState && present.jointState && src.hasSchema(key, names.stateAPI);
}

} // anonymous namespace

JointReadTokens::JointReadTokens(const IPhysicsSource& source)
{
    const char* instanceNames[] = {"distance", "transX", "transY", "transZ", "rotX", "rotY", "rotZ", "angular", "linear", "cone"};
    for (size_t index = 0; index < instances.size(); ++index)
    {
        const std::string instance = instanceNames[index];
        auto& tokens = instances[index];
        tokens.limitAPI = source.internToken("PhysxLimitAPI:" + instance);
        tokens.axisAPI = source.internToken("PhysxJointAxisAPI:" + instance);
        tokens.envelopeAPI = source.internToken("PhysxDrivePerformanceEnvelopeAPI:" + instance);
        tokens.stateAPI = source.internToken("PhysicsJointStateAPI:" + instance);
        tokens.physicsLimitAPI = source.internToken("PhysicsLimitAPI:" + instance);
        tokens.physicsDriveAPI = source.internToken("PhysicsDriveAPI:" + instance);
        tokens.restitution = source.internToken("physxLimit:" + instance + ":restitution");
        tokens.bounceThreshold = source.internToken("physxLimit:" + instance + ":bounceThreshold");
        tokens.stiffness = source.internToken("physxLimit:" + instance + ":stiffness");
        tokens.damping = source.internToken("physxLimit:" + instance + ":damping");
        tokens.armature = source.internToken("physxJointAxis:" + instance + ":armature");
        tokens.maxJointVelocity = source.internToken("physxJointAxis:" + instance + ":maxJointVelocity");
        tokens.staticFrictionEffort = source.internToken("physxJointAxis:" + instance + ":staticFrictionEffort");
        tokens.dynamicFrictionEffort = source.internToken("physxJointAxis:" + instance + ":dynamicFrictionEffort");
        tokens.viscousFrictionCoefficient = source.internToken("physxJointAxis:" + instance + ":viscousFrictionCoefficient");
        tokens.maxActuatorVelocity = source.internToken("physxDrivePerformanceEnvelope:" + instance + ":maxActuatorVelocity");
        tokens.velocityDependentResistance = source.internToken("physxDrivePerformanceEnvelope:" + instance + ":velocityDependentResistance");
        tokens.speedEffortGradient = source.internToken("physxDrivePerformanceEnvelope:" + instance + ":speedEffortGradient");
        tokens.position = source.internToken("state:" + instance + ":physics:position");
        tokens.velocity = source.internToken("state:" + instance + ":physics:velocity");
        tokens.low = source.internToken("limit:" + instance + ":physics:low");
        tokens.high = source.internToken("limit:" + instance + ":physics:high");
        tokens.driveStiffness = source.internToken("drive:" + instance + ":physics:stiffness");
        tokens.driveDamping = source.internToken("drive:" + instance + ":physics:damping");
        tokens.driveTargetPosition = source.internToken("drive:" + instance + ":physics:targetPosition");
        tokens.driveTargetVelocity = source.internToken("drive:" + instance + ":physics:targetVelocity");
        tokens.driveMaxForce = source.internToken("drive:" + instance + ":physics:maxForce");
        tokens.driveType = source.internToken("drive:" + instance + ":physics:type");
    }
    gearRatio = source.internToken("physics:gearRatio");
    rackRatio = source.internToken("physics:ratio");
}

PreparedJoint prepareJointStructure(ParseContext& ctx, ObjectKey key, const JointInfo& info)
{
    IPhysicsSource& src = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedJoint input;
    input.key = key;
    input.info = info;
    if (info.type == eJointRevolute || info.type == eJointPrismatic)
    {
        const bool rotational = info.type == eJointRevolute;
        prepareInstanceStructure(ctx, key, rotational ? JointReadTokens::angular : JointReadTokens::linear, true, true, true, true,
                        input.instances[rotational ? JointReadTokens::angular : JointReadTokens::linear]);
    }
    else if (info.type == eJointDistance)
    {
        prepareInstanceStructure(ctx, key, 0, true, false, false, false, input.instances[0]);
        input.distanceSpring = src.hasSchema(key, tok.physxPhysicsDistanceJointAPI);
    }
    else if (info.type == eJointSpherical)
    {
        prepareInstanceStructure(ctx, key, JointReadTokens::cone, true, false, false, false, input.instances[JointReadTokens::cone]);
        for (const JointAxis axis : { eRotX, eRotY, eRotZ })
            prepareInstanceStructure(ctx, key, size_t(axis), false, true, false, false, input.instances[size_t(axis)]);
    }
    else if (info.type == eJointD6)
    {
        for (const JointAxis axis : { eDistance, eTransX, eTransY, eTransZ, eRotX, eRotY, eRotZ })
        {
            const bool rotational = isAxisRotational(axis);
            const bool limit = std::any_of(info.jointLimits.begin(), info.jointLimits.end(),
                [=](const std::pair<JointAxis, JointLimitInfo>& value) { return value.first == axis; });
            const bool envelope = rotational && std::any_of(info.jointDrives.begin(), info.jointDrives.end(),
                [=](const std::pair<JointAxis, JointDriveInfo>& value) { return value.first == axis; });
            prepareInstanceStructure(ctx, key, size_t(axis), limit, rotational, envelope, axis != eDistance,
                            input.instances[size_t(axis)]);
        }
    }
    else if (info.type == eJointGear || info.type == eJointRackAndPinion)
    {
        const bool gear = info.type == eJointGear;
        std::vector<ObjectKey> targets;
        src.getRelationshipTargets(key, src.internToken(gear ? "physics:hinge0" : "physics:hinge"), targets);
        if (!targets.empty()) input.reference0 = targets.front();
        targets.clear();
        src.getRelationshipTargets(key, src.internToken(gear ? "physics:hinge1" : "physics:prismatic"), targets);
        if (!targets.empty()) input.reference1 = targets.front();
    }
    return input;
}

PreparedJoint prepareJoint(ParseContext& ctx, ObjectKey key, const JointInfo& info)
{
    PreparedJoint input = prepareJointStructure(ctx, key, info);
    readJointAttributes(ctx.source(), ctx.knownTokens(), ctx.jointReadTokens(), input);
    return input;
}

size_t jointStateCount(const PreparedJoint& input) noexcept
{
    size_t count = 0;
    for (const JointAxis axis : { eTransX, eTransY, eTransZ, eRotX, eRotY, eRotZ })
        if (input.instances[size_t(axis)].state) ++count;
    return count;
}

void prepareJointStorage(const PreparedJoint& input, PhysxJointDesc& output)
{
    if (input.info.type == eJointSpherical)
        static_cast<SphericalPhysxJointDesc&>(output).jointProperties.resize(3);
    else if (input.info.type == eJointD6)
    {
        D6PhysxJointDesc& desc = static_cast<D6PhysxJointDesc&>(output);
        desc.jointLimits.resize(input.info.jointLimits.size());
        desc.jointDrives.resize(input.info.jointDrives.size());
        desc.jointProperties.resize(3);
        desc.jointStates.resize(jointStateCount(input));
    }
}

DescPtr<PhysxJointDesc> parseJoint(ParseContext& ctx, ObjectKey key, const JointInfo& info)
{
    DescPtr<PhysxJointDesc> output;
    switch (info.type)
    {
    case eJointFixed: output = allocateDesc<FixedPhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    case eJointRevolute: output = allocateDesc<RevolutePhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    case eJointPrismatic: output = allocateDesc<PrismaticPhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    case eJointSpherical: output = allocateDesc<SphericalPhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    case eJointDistance: output = allocateDesc<DistancePhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    case eJointD6: output = allocateDesc<D6PhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    case eJointGear: output = allocateDesc<GearPhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    case eJointRackAndPinion: output = allocateDesc<RackPhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    case eJointCustom: output = allocateDesc<CustomPhysxJointDesc, PhysxJointDesc>(ctx.descriptorAllocator()); break;
    default:
        output = allocateDesc<PhysxJointDesc>(ctx.descriptorAllocator());
        if (output) output->type = info.type;
        break;
    }
    if (!output) return {};
    const PreparedJoint input = prepareJoint(ctx, key, info);
    prepareJointStorage(input, *output);
    if (!fillJoint(input, *output)) return {};
    return output;
}

std::vector<DescPtr<PhysxJointDesc>> allocateJointBatch(DescriptorBatchAllocator& allocator,
                                                     ObjectType type, size_t count)
{
    switch (type)
    {
    case eJointFixed: return allocateDescriptorBatch<FixedPhysxJointDesc, PhysxJointDesc>(allocator, count);
    case eJointRevolute: return allocateDescriptorBatch<RevolutePhysxJointDesc, PhysxJointDesc>(allocator, count);
    case eJointPrismatic: return allocateDescriptorBatch<PrismaticPhysxJointDesc, PhysxJointDesc>(allocator, count);
    case eJointSpherical: return allocateDescriptorBatch<SphericalPhysxJointDesc, PhysxJointDesc>(allocator, count);
    case eJointDistance: return allocateDescriptorBatch<DistancePhysxJointDesc, PhysxJointDesc>(allocator, count);
    case eJointD6: return allocateDescriptorBatch<D6PhysxJointDesc, PhysxJointDesc>(allocator, count);
    case eJointGear: return allocateDescriptorBatch<GearPhysxJointDesc, PhysxJointDesc>(allocator, count);
    case eJointRackAndPinion: return allocateDescriptorBatch<RackPhysxJointDesc, PhysxJointDesc>(allocator, count);
    case eJointCustom: return allocateDescriptorBatch<CustomPhysxJointDesc, PhysxJointDesc>(allocator, count);
    default:
    {
        std::vector<DescPtr<PhysxJointDesc>> result = allocateDescriptorBatch<PhysxJointDesc>(allocator, count);
        for (DescPtr<PhysxJointDesc>& desc : result) desc->type = type;
        return result;
    }
    }
}

bool fillJoint(const PreparedJoint& input, PhysxJointDesc& output) noexcept
{
    if (!input.prepared || output.type != input.info.type) return false;
    const JointInfo& info = input.info;
    if (info.type == eJointSpherical && static_cast<SphericalPhysxJointDesc&>(output).jointProperties.size() != 3)
        return false;
    if (info.type == eJointD6)
    {
        for (const std::pair<JointAxis, JointLimitInfo>& limit : info.jointLimits)
            if (limit.first < eDistance || limit.first > eRotZ) return false;
        for (const std::pair<JointAxis, JointDriveInfo>& drive : info.jointDrives)
            if (drive.first < eDistance || drive.first > eRotZ) return false;
        const D6PhysxJointDesc& desc = static_cast<const D6PhysxJointDesc&>(output);
        if (desc.jointLimits.size() != info.jointLimits.size() || desc.jointDrives.size() != info.jointDrives.size() ||
            desc.jointProperties.size() != 3 || desc.jointStates.size() != jointStateCount(input)) return false;
    }

    switch (info.type)
    {
    case eJointFixed:
    {
        FixedPhysxJointDesc* d = static_cast<FixedPhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        break;
    }
    case eJointSpherical:
    {
        SphericalPhysxJointDesc* d = static_cast<SphericalPhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        d->axis = info.axis;
        d->limit = makeAngularLimit(info.limit);
        // Spherical joint's limit uses the multi-apply instance "cone".
        fillLimit(input.instances[JointReadTokens::cone], d->limit);
        // PhysxJointAxisAPI per rotational axis (rotX, rotY, rotZ). Legacy
        // populates jointProperties for all three and applies the rotational
        // deg→rad / rad→deg conversion to each.
        for (JointAxis ax : { eRotX, eRotY, eRotZ })
        {
            PhysxJointAxisProperties props;
            fillAxis(input, input.instances[size_t(ax)], props);
            applyRotationalAxisConversions(props);
            d->jointProperties[size_t(ax) - size_t(eRotX)] = std::make_pair(ax, props);
        }
        break;
    }
    case eJointRevolute:
    {
        RevolutePhysxJointDesc* d = static_cast<RevolutePhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        d->axis = info.axis;
        d->limit = makeAngularLimit(info.limit);
        d->drive = makePhysxDrive(info.drive, /*convertToRad=*/true);
        // Revolute joint uses the "angular" instance for limit + axis-properties.
        fillLimit(input.instances[JointReadTokens::angular], d->limit);
        fillAxis(input, input.instances[JointReadTokens::angular], d->properties);
        applyRotationalAxisConversions(d->properties);
        fillEnvelope(input.instances[JointReadTokens::angular], true, d->drive);
        // Joint state for "angular" instance. Rotational → convert
        // schema-authored degrees to engine-native radians.
        fillState(input.instances[JointReadTokens::angular], d->state, /*applyDegToRad=*/true);
        break;
    }
    case eJointPrismatic:
    {
        PrismaticPhysxJointDesc* d = static_cast<PrismaticPhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        d->axis = info.axis;
        d->limit = makeLinearLimit(info.limit);
        d->drive = makePhysxDrive(info.drive, /*convertToRad=*/false);
        // Prismatic joint uses the "linear" instance.
        fillLimit(input.instances[JointReadTokens::linear], d->limit);
        fillAxis(input, input.instances[JointReadTokens::linear], d->properties);
        // No rotational conversion for the linear axis.
        fillEnvelope(input.instances[JointReadTokens::linear], false, d->drive);
        // Joint state for "linear" instance, no conversion.
        fillState(input.instances[JointReadTokens::linear], d->state, /*applyDegToRad=*/false);
        break;
    }
    case eJointDistance:
    {
        DistancePhysxJointDesc* d = static_cast<DistancePhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        d->minEnabled = info.minEnabled;
        d->maxEnabled = info.maxEnabled;
        d->limit = makeLinearLimit(info.limit);
        fillLimit(input.instances[0], d->limit);
        // PhysxPhysicsDistanceJointAPI extension: spring enabled + stiffness + damping.
        if (input.distanceSpring)
        {
            input.springEnabled.read(d->springEnabled);
            d->stiffness = clamped(input.springStiffness,
                d->stiffness, 0.0f, FLT_MAX);
            d->damping = clamped(input.springDamping,
                d->damping, 0.0f, FLT_MAX);
        }
        break;
    }
    case eJointD6:
    {
        D6PhysxJointDesc* d = static_cast<D6PhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        size_t limitIndex = 0;
        for (const auto& [ax, lim] : info.jointLimits)
        {
            PhysxJointLimit limit = makePhysxLimit(lim, ax);
            fillLimit(input.instances[size_t(ax)], limit);
            d->jointLimits[limitIndex++] = std::make_pair(ax, limit);
        }
        size_t driveIndex = 0;
        for (const auto& [ax, drv] : info.jointDrives)
        {
            PhysxJointDrive physxDrive = makePhysxDrive(drv, isAxisRotational(ax));
            // Legacy reads PhysxDrivePerformanceEnvelopeAPI only for the
            // rotational D6 drives — translational drives skip envelope.
            if (isAxisRotational(ax))
                fillEnvelope(input.instances[size_t(ax)], true, physxDrive);
            d->jointDrives[driveIndex++] = std::make_pair(ax, physxDrive);
        }
        // PhysxJointAxisAPI per rotational axis (rotX, rotY, rotZ). Legacy
        // populates jointProperties only for rotational axes on D6 (the
        // translational axes are not authored through this API).
        for (JointAxis ax : { eRotX, eRotY, eRotZ })
        {
            PhysxJointAxisProperties props;
            fillAxis(input, input.instances[size_t(ax)], props);
            applyRotationalAxisConversions(props);
            d->jointProperties[size_t(ax) - size_t(eRotX)] = std::make_pair(ax, props);
        }
        // PhysicsJointStateAPI per axis on D6. Legacy iterates trans first
        // then rot (axisVector order) and only pushes entries where the API
        // is applied. Rotational axes apply degToRad in-line.
        size_t stateIndex = 0;
        for (JointAxis ax : { eTransX, eTransY, eTransZ, eRotX, eRotY, eRotZ })
        {
            PhysicsJointState state;
            const bool isRot = isAxisRotational(ax);
            fillState(input.instances[size_t(ax)], state, /*applyDegToRad=*/isRot);
            if (state.enabled)
                d->jointStates[stateIndex++] = std::make_pair(ax, state);
        }
        break;
    }
    case eJointGear:
    {
        // Built-in gear joint: capture gearRatio + hinge0/hinge1 targets so the
        // consumer needs no USD re-read (it converts the ObjectKeys to paths).
        GearPhysxJointDesc* d = static_cast<GearPhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        input.ratio.read(d->gearRatio);
        d->hingePrimPath0 = input.reference0;
        d->hingePrimPath1 = input.reference1;
        break;
    }
    case eJointRackAndPinion:
    {
        // Built-in rack-and-pinion joint: capture ratio + hinge/prismatic targets.
        RackPhysxJointDesc* d = static_cast<RackPhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        input.ratio.read(d->ratio);
        d->hingePrimKey = input.reference0;
        d->prismaticPrimKey = input.reference1;
        break;
    }
    case eJointCustom:
    {
        // Third-party registered custom joints emit a CustomPhysxJointDesc with
        // default-empty customJointToken; the consumer fills the token in from
        // the prim type via the CustomJointManager. (Built-in gear/rack are
        // handled as typed joints above.)
        CustomPhysxJointDesc* d = static_cast<CustomPhysxJointDesc*>(&output);
        copyBaseFields(info, *d);
        break;
    }
    default:
    {
        // Any unrecognised types fall through with the base.
        PhysxJointDesc* d = static_cast<PhysxJointDesc*>(&output);
        d->type = info.type;
        copyBaseFields(info, *d);
        break;
    }
    }

    output.jointPrimKey = input.key;
    return true;
}

} // namespace omni::physics::parse
