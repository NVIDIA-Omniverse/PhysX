// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-14 AC-16
 */

#include "OvstageWalker.h"

#include "OvstageSource.h"

#include <omni/physics/parse/ArticulationGraph.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <private/omni/physics/CollisionShapeTransform.h>
#include <private/omni/physics/JointFrameTransform.h>

#include <pxr/base/gf/matrix4d.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physics::ovstage
{
namespace
{
ovx_string_t ovxStr(const char* s)
{
    return { s, std::string_view(s).size() };
}

PXR_NS::GfMatrix4d toGfMatrix4d(const Matrix4d& matrix)
{
    return PXR_NS::GfMatrix4d(
        matrix.data[0], matrix.data[1], matrix.data[2], matrix.data[3],
        matrix.data[4], matrix.data[5], matrix.data[6], matrix.data[7],
        matrix.data[8], matrix.data[9], matrix.data[10], matrix.data[11],
        matrix.data[12], matrix.data[13], matrix.data[14], matrix.data[15]);
}

void waitAndRelease(ovstage_instance_t* inst, ovstage_enqueue_result_t r)
{
    if (r.status != OVSTAGE_OK || r.op_index == OVSTAGE_INVALID_OP_ID)
        return;
    ovstage_wait_op(inst, r.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr);
    ovstage_release_op(inst, r.op_index);
}

// A concept's matched prims plus the attribute columns discovered on them — the
// latter feeds the columnar bulk read (prefetchBucket) so the per-concept parse
// loop reads from a cache rather than a per-prim round trip.
struct Bucket
{
    std::vector<ObjectKey> keys;
    std::vector<std::string> attrs; // discoverable data columns (no metadata / computed cols)
};

void appendBucketAttr(Bucket& bucket, const char* attr)
{
    if (!attr || attr[0] == '\0')
        return;
    if (std::find(bucket.attrs.begin(), bucket.attrs.end(), attr) == bucket.attrs.end())
        bucket.attrs.emplace_back(attr);
}

void appendBucketAttr(Bucket& bucket, const std::string& attr)
{
    if (attr.empty())
        return;
    if (std::find(bucket.attrs.begin(), bucket.attrs.end(), attr) == bucket.attrs.end())
        bucket.attrs.push_back(attr);
}

void appendBucketAttrs(Bucket& bucket, std::initializer_list<const char*> attrs)
{
    for (const char* attr : attrs)
        appendBucketAttr(bucket, attr);
}

std::vector<std::string> appendedAttrs(const Bucket& bucket, size_t previousSize)
{
    if (previousSize >= bucket.attrs.size())
        return {};
    return std::vector<std::string>(bucket.attrs.begin() + previousSize, bucket.attrs.end());
}

void appendTransformAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        conv::kFabricWorldMatrix,
        conv::kFabricLocalMatrix,
        conv::kLocalTransform,
        conv::kResetXformStack,
    });
}

std::vector<ObjectKey> collectTransformAncestors(const OvstageSource& src, const std::vector<ObjectKey>& keys)
{
    std::vector<ObjectKey> out;
    std::unordered_set<uint64_t> seen;
    seen.reserve(keys.size() * 2);
    for (const ObjectKey key : keys)
    {
        if (!key.valid())
            continue;
        seen.insert(key.handle);
        if (const uint64_t canonical = src.canonicalPath(key))
            seen.insert(canonical);
    }

    const ObjectKey root = src.getRootKey();
    for (const ObjectKey key : keys)
    {
        ObjectKey cur = src.getParent(key);
        for (int guard = 0; cur.valid() && guard < 64; ++guard)
        {
            const uint64_t canonical = src.canonicalPath(cur);
            const uint64_t dedupe = canonical ? canonical : cur.handle;
            if (dedupe && seen.insert(dedupe).second)
                out.push_back(cur);

            if (cur == root)
                break;
            const ObjectKey next = src.getParent(cur);
            if (next.handle == cur.handle)
                break;
            cur = next;
        }
    }
    return out;
}

void prefetchTransformAncestors(OvstageSource& src, const std::vector<ObjectKey>& keys)
{
    Bucket bucket;
    bucket.keys = collectTransformAncestors(src, keys);
    if (bucket.keys.empty())
        return;
    appendTransformAttrs(bucket);
    src.prefetchBucket(bucket.keys, bucket.attrs);
}

void prefetchTransformsForKeys(OvstageSource& src, const std::vector<ObjectKey>& keys)
{
    Bucket bucket;
    bucket.keys = keys;
    if (bucket.keys.empty())
        return;
    appendTransformAttrs(bucket);
    src.prefetchBucket(bucket.keys, bucket.attrs);
}

void appendSceneAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "physics:gravityDirection",
        "physics:gravityMagnitude",
        "physxScene:updateType",
        "physxScene:bounceThreshold",
        "physxScene:frictionOffsetThreshold",
        "physxScene:frictionCorrelationDistance",
        "physxScene:maxBiasCoefficient",
        "physxScene:timeStepsPerSecond",
        "physxScene:minPositionIterationCount",
        "physxScene:maxPositionIterationCount",
        "physxScene:minVelocityIterationCount",
        "physxScene:maxVelocityIterationCount",
        "physxScene:enableCCD",
        "physxScene:enableStabilization",
        "physxScene:enableGPUDynamics",
        "physxScene:enableEnhancedDeterminism",
        "physxScene:enableExternalForcesEveryIteration",
        "physxScene:invertCollisionGroupFilter",
        "physxScene:reportKinematicKinematicPairs",
        "physxScene:reportKinematicStaticPairs",
        "physxScene:enableSceneQuerySupport",
        "physxScene:solveArticulationContactLast",
        "physxScene:disableSleeping",
        "physxScene:collisionSystem",
        "physxScene:solverType",
        "physxScene:broadphaseType",
        "physxScene:frictionType",
        "physxScene:gpuTempBufferCapacity",
        "physxScene:gpuMaxRigidContactCount",
        "physxScene:gpuMaxRigidPatchCount",
        "physxScene:gpuHeapCapacity",
        "physxScene:gpuFoundLostPairsCapacity",
        "physxScene:gpuFoundLostAggregatePairsCapacity",
        "physxScene:gpuTotalAggregatePairsCapacity",
        "physxScene:gpuMaxDeformableVolumeContacts",
        "physxScene:gpuMaxDeformableSurfaceContacts",
        "physxScene:gpuMaxParticleContacts",
        "physxScene:gpuCollisionStackSize",
        "physxScene:gpuMaxNumPartitions",
        "physxScene:envIdInBoundsBitCount",
        "physxSceneQuasistatic:enableQuasistatic",
        "newton:timeStepsPerSecond",
        "newton:gravityEnabled",
    });
}

void appendMaterialAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "physics:staticFriction",
        "physics:dynamicFriction",
        "physics:restitution",
        "physics:density",
        "physxMaterial:frictionCombineMode",
        "physxMaterial:restitutionCombineMode",
        "physxMaterial:dampingCombineMode",
        "physxMaterial:compliantContactAccelerationSpring",
        "physxMaterial:compliantContactStiffness",
        "physxMaterial:compliantContactDamping",
    });
}

void appendRigidBodyAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "physics:rigidBodyEnabled",
        "physics:kinematicEnabled",
        "physics:startsAsleep",
        "physics:velocity",
        "physics:angularVelocity",
        "physics:localSpaceVelocities",
        "physics:simulationOwner",
        "physics:filteredPairs",
        "physxRigidBody:linearDamping",
        "physxRigidBody:angularDamping",
        "physxRigidBody:maxLinearVelocity",
        "physxRigidBody:maxAngularVelocity",
        "physxRigidBody:sleepThreshold",
        "physxRigidBody:stabilizationThreshold",
        "physxRigidBody:maxDepenetrationVelocity",
        "physxRigidBody:contactSlopCoefficient",
        "physxRigidBody:maxContactImpulse",
        "physxRigidBody:cfmScale",
        "physxRigidBody:solverPositionIterationCount",
        "physxRigidBody:solverVelocityIterationCount",
        "physxRigidBody:enableCCD",
        "physxRigidBody:enableSpeculativeCCD",
        "physxRigidBody:disableGravity",
        "physxRigidBody:retainAccelerations",
        "physxRigidBody:enableGyroscopicForces",
        "physxRigidBody:solveContact",
        "physxRigidBody:lockedPosAxis",
        "physxRigidBody:lockedRotAxis",
        "physxSurfaceVelocity:surfaceVelocityEnabled",
        "physxSurfaceVelocity:surfaceVelocityLocalSpace",
        "physxSurfaceVelocity:surfaceVelocity",
        "physxSurfaceVelocity:surfaceAngularVelocity",
        "physxSplinesSurfaceVelocity:surfaceVelocityEnabled",
        "physxSplinesSurfaceVelocity:surfaceVelocityMagnitude",
        "physxSplinesSurfaceVelocity:surfaceVelocityCurve",
    });
}

void appendMassAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "physics:mass",
        "physics:density",
        "physics:centerOfMass",
        "physics:diagonalInertia",
        "physics:principalAxes",
    });
}

void appendCollisionShapeAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "physics:collisionEnabled",
        "physics:simulationOwner",
        "physics:filteredPairs",
        "physics:approximation",
        "material:binding:physics",
        "material:binding",
        "size",
        "radius",
        "height",
        "axis",
        "doubleSided",
        "physxCollision:torsionalPatchRadius",
        "physxCollision:minTorsionalPatchRadius",
        "physxCollision:contactOffset",
        "physxCollision:restOffset",
        "physxConvexGeometry:margin",
        "physxConvexHullCollision:hullVertexLimit",
        "physxConvexHullCollision:minThickness",
        "physxConvexDecompositionCollision:minThickness",
        "physxConvexDecompositionCollision:maxConvexHulls",
        "physxConvexDecompositionCollision:hullVertexLimit",
        "physxConvexDecompositionCollision:voxelResolution",
        "physxConvexDecompositionCollision:errorPercentage",
        "physxConvexDecompositionCollision:shrinkWrap",
        "physxSphereFillCollision:maxSpheres",
        "physxSphereFillCollision:seedCount",
        "physxSphereFillCollision:voxelResolution",
        "physxSphereFillCollision:fillMode",
        "physxTriangleMeshCollision:weldTolerance",
        "physxTriangleMeshSimplificationCollision:metric",
        "physxTriangleMeshSimplificationCollision:weldTolerance",
        "physxSDFMeshCollision:sdfResolution",
        "physxSDFMeshCollision:sdfSubgridResolution",
        "physxSDFMeshCollision:sdfBitsPerSubgridPixel",
        "physxSDFMeshCollision:sdfNarrowBandThickness",
        "physxSDFMeshCollision:sdfMargin",
        "physxSDFMeshCollision:sdfEnableRemeshing",
        "physxSDFMeshCollision:sdfTriangleCountReductionFactor",
        "newton:maxHullVertices",
        "newton:contactMargin",
        "newton:contactGap",
    });
}

void appendDeformableBodyAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "omniphysics:deformableBodyEnabled",
        "omniphysics:mass",
        "omniphysics:kinematicEnabled",
        "omniphysics:startsAsleep",
        "omniphysics:simulationOwner",
        "physics:filteredPairs",
        "physxDeformableBody:linearDamping",
        "physxDeformableBody:maxLinearVelocity",
        "physxDeformableBody:sleepThreshold",
        "physxDeformableBody:settlingThreshold",
        "physxDeformableBody:settlingDamping",
        "physxDeformableBody:maxDepenetrationVelocity",
        "physxDeformableBody:selfCollisionFilterDistance",
        "physxDeformableBody:solverPositionIterationCount",
        "physxDeformableBody:enableSpeculativeCCD",
        "physxDeformableBody:selfCollision",
        "physxDeformableBody:disableGravity",
        "physxDeformableBody:collisionPairUpdateFrequency",
        "physxDeformableBody:collisionIterationMultiplier",
        "physxDeformableBody:autoDeformableBodyEnabled",
        "physxDeformableBody:cookingSourceMesh",
        "physxDeformableBody:autoDeformableMeshSimplificationEnabled",
        "physxDeformableBody:remeshingEnabled",
        "physxDeformableBody:remeshingResolution",
        "physxDeformableBody:targetTriangleCount",
        "physxDeformableBody:forceConforming",
        "physxDeformableBody:resolution",
    });
}

void appendCollisionGroupAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "physics:filteredGroups",
        "physics:invertFilteredGroups",
        "collection:colliders:includes",
        "collection:colliders:excludes",
    });
}

void appendArticulationAttrs(Bucket& bucket)
{
    appendTransformAttrs(bucket);
    appendBucketAttrs(bucket, {
        "physics:filteredPairs",
        "physxArticulation:articulationEnabled",
        "physxArticulation:sleepThreshold",
        "physxArticulation:stabilizationThreshold",
        "physxArticulation:solverPositionIterationCount",
        "physxArticulation:solverVelocityIterationCount",
        "physxArticulation:enabledSelfCollisions",
        "newton:selfCollisionEnabled",
    });
}

void appendJointCommonAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "physics:jointEnabled",
        "physics:collisionEnabled",
        "physics:excludeFromArticulation",
        "physics:breakForce",
        "physics:breakTorque",
        "physics:body0",
        "physics:body1",
        "physics:localPos0",
        "physics:localRot0",
        "physics:localPos1",
        "physics:localRot1",
        "physxJoint:jointFriction",
    });
}

void appendPhysxJointAxisAttrs(Bucket& bucket, const char* axis)
{
    const std::string axisBase = std::string("physxJointAxis:") + axis + ":";
    appendBucketAttr(bucket, axisBase + "armature");
    appendBucketAttr(bucket, axisBase + "maxJointVelocity");
    appendBucketAttr(bucket, axisBase + "staticFrictionEffort");
    appendBucketAttr(bucket, axisBase + "dynamicFrictionEffort");
    appendBucketAttr(bucket, axisBase + "viscousFrictionCoefficient");

    appendBucketAttrs(bucket, {
        "physxJoint:armature",
        "physxJoint:maxJointVelocity",
    });
}

void appendPhysxLimitAttrs(Bucket& bucket, const char* axis)
{
    const std::string limitBase = std::string("physxLimit:") + axis + ":";
    appendBucketAttr(bucket, limitBase + "restitution");
    appendBucketAttr(bucket, limitBase + "bounceThreshold");
    appendBucketAttr(bucket, limitBase + "stiffness");
    appendBucketAttr(bucket, limitBase + "damping");
}

void appendPhysicsLimitAttrs(Bucket& bucket, const char* axis)
{
    const std::string limitBase = std::string("limit:") + axis + ":physics:";
    appendBucketAttr(bucket, limitBase + "low");
    appendBucketAttr(bucket, limitBase + "high");
    appendPhysxLimitAttrs(bucket, axis);
}

void appendPhysicsDriveAttrs(Bucket& bucket, const char* axis)
{
    const std::string driveBase = std::string("drive:") + axis + ":physics:";
    appendBucketAttr(bucket, driveBase + "stiffness");
    appendBucketAttr(bucket, driveBase + "damping");
    appendBucketAttr(bucket, driveBase + "targetPosition");
    appendBucketAttr(bucket, driveBase + "targetVelocity");
    appendBucketAttr(bucket, driveBase + "maxForce");
    appendBucketAttr(bucket, driveBase + "type");
}

void appendPhysxDrivePerformanceEnvelopeAttrs(Bucket& bucket, const char* axis)
{
    const std::string envelopeBase = std::string("physxDrivePerformanceEnvelope:") + axis + ":";
    appendBucketAttr(bucket, envelopeBase + "maxActuatorVelocity");
    appendBucketAttr(bucket, envelopeBase + "velocityDependentResistance");
    appendBucketAttr(bucket, envelopeBase + "speedEffortGradient");
}

void appendPhysicsJointStateAttrs(Bucket& bucket, const char* axis)
{
    const std::string stateBase = std::string("state:") + axis + ":physics:";
    appendBucketAttr(bucket, stateBase + "position");
    appendBucketAttr(bucket, stateBase + "velocity");
}

void appendNewtonMimicAttrs(Bucket& bucket)
{
    appendBucketAttrs(bucket, {
        "newton:mimicJoint",
    });
}

void appendJointAttrs(Bucket& bucket, ObjectType type)
{
    appendJointCommonAttrs(bucket);

    switch (type)
    {
    case eJointFixed:
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointSpherical:
        appendBucketAttrs(bucket, {
            "physics:axis",
            "physics:coneAngle0Limit",
            "physics:coneAngle1Limit",
        });
        appendPhysxLimitAttrs(bucket, "cone");
        appendPhysxJointAxisAttrs(bucket, "rotX");
        appendPhysxJointAxisAttrs(bucket, "rotY");
        appendPhysxJointAxisAttrs(bucket, "rotZ");
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointRevolute:
        appendBucketAttrs(bucket, {
            "physics:axis",
            "physics:lowerLimit",
            "physics:upperLimit",
        });
        appendPhysxLimitAttrs(bucket, "angular");
        appendPhysxJointAxisAttrs(bucket, "angular");
        appendPhysicsDriveAttrs(bucket, "angular");
        appendPhysxDrivePerformanceEnvelopeAttrs(bucket, "angular");
        appendPhysicsJointStateAttrs(bucket, "angular");
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointPrismatic:
        appendBucketAttrs(bucket, {
            "physics:axis",
            "physics:lowerLimit",
            "physics:upperLimit",
        });
        appendPhysxLimitAttrs(bucket, "linear");
        appendPhysxJointAxisAttrs(bucket, "linear");
        appendPhysicsDriveAttrs(bucket, "linear");
        appendPhysxDrivePerformanceEnvelopeAttrs(bucket, "linear");
        appendPhysicsJointStateAttrs(bucket, "linear");
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointDistance:
        appendBucketAttrs(bucket, {
            "physics:minDistance",
            "physics:maxDistance",
            "physxPhysicsDistanceJoint:springEnabled",
            "physxPhysicsDistanceJoint:springStiffness",
            "physxPhysicsDistanceJoint:springDamping",
        });
        appendPhysxLimitAttrs(bucket, "distance");
        appendNewtonMimicAttrs(bucket);
        break;

    case eJointD6:
    {
        static const char* const kD6Axes[] = { "transX", "transY", "transZ", "rotX", "rotY", "rotZ", "distance" };
        for (const char* axis : kD6Axes)
        {
            appendPhysicsLimitAttrs(bucket, axis);
            appendPhysicsDriveAttrs(bucket, axis);
        }

        static const char* const kD6RotAxes[] = { "rotX", "rotY", "rotZ" };
        for (const char* axis : kD6RotAxes)
        {
            appendPhysxJointAxisAttrs(bucket, axis);
            appendPhysxDrivePerformanceEnvelopeAttrs(bucket, axis);
        }

        static const char* const kD6StateAxes[] = { "transX", "transY", "transZ", "rotX", "rotY", "rotZ" };
        for (const char* axis : kD6StateAxes)
            appendPhysicsJointStateAttrs(bucket, axis);

        appendNewtonMimicAttrs(bucket);
        break;
    }

    case eJointGear:
        appendBucketAttrs(bucket, {
            "physics:gearRatio",
            "physics:hinge0",
            "physics:hinge1",
        });
        break;

    case eJointRackAndPinion:
        appendBucketAttrs(bucket, {
            "physics:ratio",
            "physics:hinge",
            "physics:prismatic",
        });
        break;

    default:
        break;
    }
}

// Enumerate the ObjectKeys of prims matching a single-predicate filter, plus the
// data attributes discovered on the matched set. `probeAttr` must be an attribute
// every matched prim carries: the read group's prims.list materialises the
// matched set (ovstage gives no path list on the query result itself).
Bucket enumerate(ovstage_instance_t* inst,
                 ovx_path_dictionary_t* dict,
                 const char* predAttr,
                 ovstage_filter_op_t op,
                 const char* predValue,
                 const char* probeAttr,
                 ovstage_ordinal_t readOrdinal,
                 const char* metadataProbeAttr)
{
    Bucket out;
    std::vector<ObjectKey>& keys = out.keys;

    ovx_string_t value = ovxStr(predValue);
    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
    pred.attribute.string = ovxStr(predAttr);
    pred.op = op;
    pred.values = &value;
    pred.value_count = 1;

    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t qe = ovstage_query(inst, &filter, nullptr, 0, &q);
    if (qe.status != OVSTAGE_OK)
        return out;
    waitAndRelease(inst, qe);

    // ovstage gives no prim path list on the query result itself — the matched
    // prims materialise only as a read group's prims.list. Rather than assume a
    // specific probe attribute exists (different data models author different
    // attributes — the hand populator vs ovpopulation), read the query result's
    // own *discovered* attributes: those are exactly the attributes present on the
    // matched set, so reading them back yields every matched prim. `probeAttr` is
    // a fallback when the result reports no attributes. `usd-schemas` is readable
    // metadata, so schema/type predicates can use an explicit metadata column as
    // an enumeration probe even when the matched prim has no ordinary data column.
    ovstage_query_handle_t use = q;
    std::vector<ovx_token_t> probes; // all usable data columns; their union covers the matched set
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(inst, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
            use = qr.all_handle;
        // Probe with attributes the matched prims actually carry: discovered
        // attributes (data models differ — hand populator vs ovpopulation — so a
        // hardcoded probe name is wrong). Skip fabric-internal computed columns
        // (leading '_', e.g. "_worldExtent"/"_worldVisibility"): they aren't
        // readable through the data-plane read, so probing them yields no prims.
        for (size_t i = 0; i < qr.attribute_count; ++i)
        {
            ovx_string_t s{};
            if (ovx_path_dictionary_token_to_string(dict, qr.attributes[i], &s) != OVX_OK || !s.ptr || s.length == 0)
                continue;
            if (s.ptr[0] == '_')
                continue;

            // Built-in usd-* metadata is not scalar parse data. Some metadata
            // columns are readable, though, so use the explicit metadata probe
            // only for enumeration fallback, not as a bulk-prefetch attribute.
            if (s.length >= 4 && std::strncmp(s.ptr, "usd-", 4) == 0)
                continue;
            probes.push_back(qr.attributes[i]); // each usable column probes + bulk-reads
            out.attrs.emplace_back(s.ptr, s.length);
        }
        ovstage_release_query_result(inst, &qr);
    }
    if (probes.empty())
    {
        // No ordinary data column on the matched set: fall back to an explicit
        // metadata/probe column so schema/type predicates still enumerate.
        ovx_token_t fb = OVX_INVALID_TOKEN;
        const char* fallbackProbe = metadataProbeAttr ? metadataProbeAttr : probeAttr;
        if (ovx_path_dictionary_intern_token(dict, ovxStr(fallbackProbe), &fb) == OVX_OK &&
            fb != OVX_INVALID_TOKEN)
            probes.push_back(fb);
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = readOrdinal;
    range.has_start_ordinal = false;

    // Read EVERY usable column and union the matched prim lists. A single column
    // is only authored on the prims that carry it, so one probe under-counts the
    // matched set: a read group's prims.list holds only the prims with that
    // column, dropping matched prims whose authored column was not the first one
    // picked (e.g. Sphere/Mesh/Cube colliders, particle systems). The discovered
    // columns are the union over the matched set, so every matched prim carries at
    // least one — reading them all and unioning (dedup by handle) yields the
    // complete set, which is what let us drop the USD-Traverse supplement in
    // OvstageSource::collectPrimTypeKeys. Handles come from the data-column read
    // (the populator's canonical primpaths) so downstream key lookups still match.
    std::unordered_set<uint64_t> seen;
    for (const ovx_token_t probe : probes)
    {
        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re = ovstage_read_attributes(inst, use, &probe, 1, range, &rh);
        if (re.status != OVSTAGE_OK)
            continue;
        waitAndRelease(inst, re);
        ovstage_read_group_t g{};
        // Union prims from EVERY read group, not just the first. A scalar column
        // comes back as a single group whose prims.list covers all owning prims,
        // but an array/relationship column (e.g. `collection:colliders:includes`,
        // `physics:filteredGroups`) is emitted one group PER owning prim — so a
        // first-group-only scan captured just one owner and dropped the rest (a
        // second collision group with only a ragged membership column went missing).
        // Dedup via `seen` keeps scalar reads idempotent.
        while (ovstage_fetch_read_next(inst, rh, OVSTAGE_TIMEOUT_INFINITE, &g) == OVSTAGE_OK)
        {
            const ovx_primpath_t* paths = nullptr;
            size_t count = 0;
            if (ovx_path_dictionary_get_paths(dict, g.prims.list, &paths, &count) == OVX_OK)
                for (size_t i = 0; i < count; ++i)
                    if (seen.insert(paths[i]).second)
                        keys.push_back(ObjectKey{ paths[i] });
            ovstage_release_group(inst, &g);
        }
        waitAndRelease(inst, ovstage_release_read(inst, rh));
    }
    waitAndRelease(inst, ovstage_release_query(inst, q));
    return out;
}

// Quaternion (x,y,z,w) from the row-major, row-vector rotation matrix that
// IPhysicsSource::getLocalToWorldRotationAndScale returns. The row-vector form
// (v' = v R) is the transpose of the column-vector matrix M = R^T the standard
// trace-based extraction expects, so the transposed indices are fed in. Lets a
// bodyless collider's authored orientation survive into the shape's local pose.
carb::Float4 quatFromRotation(const Matrix3d& r)
{
    const double m00 = r.data[0], m01 = r.data[3], m02 = r.data[6];
    const double m10 = r.data[1], m11 = r.data[4], m12 = r.data[7];
    const double m20 = r.data[2], m21 = r.data[5], m22 = r.data[8];
    const double trace = m00 + m11 + m22;
    carb::Float4 q{ 0.0f, 0.0f, 0.0f, 1.0f };
    if (trace > 0.0)
    {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = static_cast<float>(0.25 * s);
        q.x = static_cast<float>((m21 - m12) / s);
        q.y = static_cast<float>((m02 - m20) / s);
        q.z = static_cast<float>((m10 - m01) / s);
    }
    else if (m00 > m11 && m00 > m22)
    {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q.w = static_cast<float>((m21 - m12) / s);
        q.x = static_cast<float>(0.25 * s);
        q.y = static_cast<float>((m01 + m10) / s);
        q.z = static_cast<float>((m02 + m20) / s);
    }
    else if (m11 > m22)
    {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q.w = static_cast<float>((m02 - m20) / s);
        q.x = static_cast<float>((m01 + m10) / s);
        q.y = static_cast<float>(0.25 * s);
        q.z = static_cast<float>((m12 + m21) / s);
    }
    else
    {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q.w = static_cast<float>((m10 - m01) / s);
        q.x = static_cast<float>((m02 + m20) / s);
        q.y = static_cast<float>((m12 + m21) / s);
        q.z = static_cast<float>(0.25 * s);
    }
    const double lenSq = static_cast<double>(q.x) * static_cast<double>(q.x) +
                         static_cast<double>(q.y) * static_cast<double>(q.y) +
                         static_cast<double>(q.z) * static_cast<double>(q.z) +
                         static_cast<double>(q.w) * static_cast<double>(q.w);
    if (!std::isfinite(lenSq) || lenSq <= 1.0e-24)
        return { 0.0f, 0.0f, 0.0f, 1.0f };
    const double invLen = 1.0 / std::sqrt(lenSq);
    q.x = static_cast<float>(static_cast<double>(q.x) * invLen);
    q.y = static_cast<float>(static_cast<double>(q.y) * invLen);
    q.z = static_cast<float>(static_cast<double>(q.z) * invLen);
    q.w = static_cast<float>(static_cast<double>(q.w) * invLen);
    return q;
}

float maxAbs2(float a, float b)
{
    return std::max(std::fabs(a), std::fabs(b));
}

float maxAbs3(float a, float b, float c)
{
    return std::max(maxAbs2(a, b), std::fabs(c));
}

void scaleRoundShape(Axis axis, const carb::Float3& scale, float& radius, float& halfHeight)
{
    if (axis == eX)
    {
        halfHeight *= scale.x;
        radius *= maxAbs2(scale.y, scale.z);
    }
    else if (axis == eY)
    {
        halfHeight *= scale.y;
        radius *= maxAbs2(scale.x, scale.z);
    }
    else
    {
        halfHeight *= scale.z;
        radius *= maxAbs2(scale.x, scale.y);
    }
}


// Does prim `path` have prim-type == `typeName`? (usd-prim-type is filter-only,
// like usd-schemas — so a scoped query, not a column read.)
bool isType(ovstage_instance_t* inst, const std::string& path, const char* typeName)
{
    ovx_string_t pathVal = ovxStr(path.c_str());
    ovx_string_t typeVal = ovxStr(typeName);

    ovstage_predicate_t preds[2]{};
    preds[0].attribute.string = ovxStr("usd-path");
    preds[0].op = OVSTAGE_FILTER_OP_IN;
    preds[0].values = &pathVal;
    preds[0].value_count = 1;
    preds[1].attribute.string = ovxStr("usd-prim-type");
    preds[1].op = OVSTAGE_FILTER_OP_IN;
    preds[1].values = &typeVal;
    preds[1].value_count = 1;

    ovstage_filter_t filter{};
    filter.predicates = preds;
    filter.count = 2;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t e = ovstage_query(inst, &filter, nullptr, 0, &q);
    if (e.status != OVSTAGE_OK)
        return false;
    waitAndRelease(inst, e);

    size_t count = 0;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(inst, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        count = qr.total_prim_count;
        ovstage_release_query_result(inst, &qr);
    }
    waitAndRelease(inst, ovstage_release_query(inst, q));
    return count >= 1;
}

bool isType(const OvstageSource& src, ovstage_instance_t* inst, ObjectKey key, const char* typeName)
{
    const std::string path(src.sourceKeyToString(key));
    return !path.empty() && isType(inst, path, typeName);
}

bool tokenColumnContains(ovstage_instance_t* inst,
                         ovx_path_dictionary_t* dict,
                         const OvstageSource& src,
                         ObjectKey key,
                         const char* attrName,
                         ovx_token_t wantedToken,
                         ovstage_ordinal_t readOrdinal)
{
    if (!inst || !dict || !key.valid() || !attrName || wantedToken == OVX_INVALID_TOKEN)
        return false;

    const uint64_t canonical = src.canonicalPath(key);
    ovx_primpath_t path = canonical ? canonical : key.handle;
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list(dict, &path, 1, &list) != OVX_OK)
        return false;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(inst, list, &q) != OVSTAGE_OK || q == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        ovx_path_dictionary_destroy_path_list(dict, list);
        return false;
    }

    ovx_token_t attr = OVX_INVALID_TOKEN;
    if (ovx_path_dictionary_intern_token(dict, ovxStr(attrName), &attr) != OVX_OK || attr == OVX_INVALID_TOKEN)
    {
        waitAndRelease(inst, ovstage_release_query(inst, q));
        ovx_path_dictionary_destroy_path_list(dict, list);
        return false;
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = readOrdinal;
    range.has_start_ordinal = false;

    bool found = false;
    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_read_attributes(inst, q, &attr, 1, range, &rh);
    if (re.status == OVSTAGE_OK)
    {
        waitAndRelease(inst, re);
        ovstage_read_group_t g{};
        while (!found && ovstage_fetch_read_next(inst, rh, OVSTAGE_TIMEOUT_INFINITE, &g) == OVSTAGE_OK)
        {
            if (g.data.tensor_count > 0 && g.data.tensors && g.data.tensors[0].data)
            {
                const DLTensor& t = g.data.tensors[0];
                size_t valueCount = 1;
                for (int i = 0; i < t.ndim; ++i)
                    valueCount *= static_cast<size_t>(t.shape[i]);

                const uint8_t* bytes = static_cast<const uint8_t*>(t.data) + t.byte_offset;
                if (t.dtype.code == kDLUInt && t.dtype.bits == 64)
                {
                    const uint64_t* values = reinterpret_cast<const uint64_t*>(bytes);
                    for (size_t i = 0; i < valueCount; ++i)
                    {
                        if (values[i] == static_cast<uint64_t>(wantedToken))
                        {
                            found = true;
                            break;
                        }
                    }
                }
                else if (t.dtype.code == kDLUInt && t.dtype.bits == 32)
                {
                    const uint32_t* values = reinterpret_cast<const uint32_t*>(bytes);
                    for (size_t i = 0; i < valueCount; ++i)
                    {
                        if (values[i] == static_cast<uint32_t>(wantedToken))
                        {
                            found = true;
                            break;
                        }
                    }
                }
            }
            ovstage_release_group(inst, &g);
        }
        waitAndRelease(inst, ovstage_release_read(inst, rh));
    }

    waitAndRelease(inst, ovstage_release_query(inst, q));
    ovx_path_dictionary_destroy_path_list(dict, list);
    return found;
}

bool isXformable(const OvstageSource& src, ObjectKey key)
{
    return src.isA(key, src.internToken("Xformable"));
}

void appendUnique(std::vector<ObjectKey>& dst, const std::vector<ObjectKey>& srcKeys)
{
    for (const ObjectKey key : srcKeys)
    {
        const auto it = std::find_if(dst.begin(), dst.end(), [&](ObjectKey existing)
        {
            return existing.handle == key.handle;
        });
        if (it == dst.end())
            dst.push_back(key);
    }
}

Bucket enumerateSchema(ovstage_instance_t* inst,
                       ovx_path_dictionary_t* dict,
                       const char* schemaName,
                       const char* probeAttr,
                       ovstage_ordinal_t readOrdinal)
{
    return enumerate(inst, dict, "usd-schemas", OVSTAGE_FILTER_OP_CONTAINS,
                     schemaName, probeAttr, readOrdinal, conv::kUsdSchemas);
}

Bucket enumerateSchemas(ovstage_instance_t* inst,
                        ovx_path_dictionary_t* dict,
                        const char* const* schemaNames,
                        size_t schemaNameCount,
                        const char* probeAttr,
                        ovstage_ordinal_t readOrdinal)
{
    Bucket out;
    for (size_t i = 0; i < schemaNameCount; ++i)
    {
        Bucket one = enumerateSchema(inst, dict, schemaNames[i], probeAttr, readOrdinal);
        appendUnique(out.keys, one.keys);
        for (const std::string& attr : one.attrs)
        {
            if (std::find(out.attrs.begin(), out.attrs.end(), attr) == out.attrs.end())
                out.attrs.push_back(attr);
        }
    }
    return out;
}

std::vector<ObjectKey> relationshipTargets(const OvstageSource& src, ObjectKey key, const char* relName)
{
    std::vector<ObjectKey> targets;
    src.getRelationshipTargets(key, src.internToken(relName), targets);
    return targets;
}

std::vector<ObjectKey> relationshipTargetsEither(const OvstageSource& src, ObjectKey key,
                                                 const char* primaryRel, const char* fallbackRel)
{
    std::vector<ObjectKey> targets = relationshipTargets(src, key, primaryRel);
    if (targets.empty() && fallbackRel && fallbackRel[0] != '\0')
        targets = relationshipTargets(src, key, fallbackRel);
    return targets;
}

ObjectKey resolveRelOrApi(const OvstageSource& src, ObjectKey key, const char* relName, const char* apiName)
{
    std::vector<ObjectKey> targets = relationshipTargets(src, key, relName);
    if (targets.size() == 1)
        return targets[0];
    if (targets.size() > 1)
        return {};
    if (src.hasSchema(key, src.internToken(apiName)))
        return key;
    return {};
}

template <typename T>
bool readTypedArray(const OvstageSource& src, ObjectKey key, const char* attrName,
                    BufferElemType type, int comps, std::vector<T>& out)
{
    out.clear();
    BufferHandle h = src.readArrayAttribute(key, attrName, type, comps);
    if (!h.valid())
        return false;

    size_t byteCount = 0;
    const void* data = src.resolveBuffer(h, byteCount);
    if (!data)
    {
        src.releaseBuffer(h);
        return false;
    }

    const size_t count = std::min<size_t>(h.elemCount, byteCount / sizeof(T));
    const T* typed = static_cast<const T*>(data);
    out.assign(typed, typed + count);
    src.releaseBuffer(h);
    return true;
}

bool readFloatArray(const OvstageSource& src, ObjectKey key, const char* attrName, std::vector<float>& out)
{
    return readTypedArray(src, key, attrName, BufferElemType::eFloat, 1, out);
}

bool readFloatArrayEither(const OvstageSource& src, ObjectKey key,
                          const char* primaryAttr, const char* fallbackAttr, std::vector<float>& out)
{
    if (readFloatArray(src, key, primaryAttr, out))
        return true;
    return fallbackAttr && fallbackAttr[0] != '\0' && readFloatArray(src, key, fallbackAttr, out);
}

bool readIntArray(const OvstageSource& src, ObjectKey key, const char* attrName, std::vector<int>& out)
{
    out.clear();
    std::vector<int32_t> tmp;
    if (!readTypedArray(src, key, attrName, BufferElemType::eInt32, 1, tmp))
        return false;
    out.reserve(tmp.size());
    for (int32_t v : tmp)
        out.push_back(static_cast<int>(v));
    return true;
}

bool readFloat2Array(const OvstageSource& src, ObjectKey key, const char* attrName, std::vector<carb::Float2>& out)
{
    return readTypedArray(src, key, attrName, BufferElemType::eVec2, 2, out);
}

bool readFloat4Array(const OvstageSource& src, ObjectKey key, const char* attrName, std::vector<carb::Float4>& out)
{
    return readTypedArray(src, key, attrName, BufferElemType::eVec4, 4, out);
}

bool hasCollisionInSubtree(const OvstageSource& src, ObjectKey key, TokenId collisionApi)
{
    if (src.hasSchema(key, collisionApi))
        return true;
    bool found = false;
    src.forEachChild(key, [&](ObjectKey child)
    {
        if (!found && hasCollisionInSubtree(src, child, collisionApi))
            found = true;
    });
    return found;
}

bool fillCommonShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info, PhysxShapeDesc& desc)
{
    setToDefault(desc, ctx.units());

    desc.collisionEnabled = info.collisionEnabled;
    desc.collisionGroup = info.collisionGroup;
    desc.rigidBody = info.rigidBody;
    desc.localPos = info.localPos;
    desc.localRot = info.localRot;
    desc.localScale = info.localScale;
    desc.sourceGprim = info.sourceGprim;

    CollisionExtFields fields;
    fields.contactOffset = desc.contactOffset;
    fields.restOffset = desc.restOffset;
    fields.torsionalPatchRadius = desc.torsionalPatchRadius;
    fields.minTorsionalPatchRadius = desc.minTorsionalPatchRadius;
    fields.isTrigger = desc.isTrigger;
    fields.isTriggerUsdOutput = desc.isTriggerUsdOutput;
    parseCollisionExt(ctx, key, fields);
    desc.contactOffset = fields.contactOffset;
    desc.restOffset = fields.restOffset;
    desc.torsionalPatchRadius = fields.torsionalPatchRadius;
    desc.minTorsionalPatchRadius = fields.minTorsionalPatchRadius;
    desc.isTrigger = fields.isTrigger;
    desc.isTriggerUsdOutput = fields.isTriggerUsdOutput;

    for (ObjectKey owner : info.simulationOwners)
    {
        ObjectId id = ctx.objects().findEntry(owner, eScene);
        if (id != kInvalidObjectId)
            desc.sceneIds.push_back(id);
    }
    return info.simulationOwners.empty() || !desc.sceneIds.empty();
}

// Mesh-shape descriptor builder — the ovstage peer of NativeWalker::readMeshShapeDesc.
// Reads the mesh approximation + cooking knobs via the shared parse functions, applies
// the TriangleMesh / MeshSimplification -> ConvexHull fallback for dynamic non-kinematic
// bodies (PhysX rejects non-SDF triangle meshes as a simulation shape on a dynamic body),
// and allocates the typed descriptor. meshPrimKey = gprimKey: the cooking service reads
// the geometry from there via IPhysicsSource::getMeshAttributes (FROM_PRIM_MESH_VIEW path).
// Bounding-sphere/cube are not handled (they need an eagerly-merged point buffer the scan
// owns) -> fall through to a triangle mesh.
DescPtr<PhysxShapeDesc> buildMeshShapeDesc(OvstageScanResult& out, ParseContext& ctx, const IPhysicsSource& src,
                                           ObjectKey shapeKey, ObjectKey gprimKey, ObjectKey rigidBodyKey,
                                           const carb::Float3& meshScale, const carb::Float3& signScale,
                                           bool doubleSided)
{
    MeshApproximation approx = parseMeshApproximation(ctx, shapeKey);

    if ((approx == MeshApproximation::eNone || approx == MeshApproximation::eMeshSimplification) &&
        rigidBodyKey.valid())
    {
        bool kinematic = false, enabled = true;
        src.getAttribute(rigidBodyKey, src.internToken("physics:kinematicEnabled"), kinematic);
        src.getAttribute(rigidBodyKey, src.internToken("physics:rigidBodyEnabled"), enabled);
        if (!kinematic && enabled)
            approx = MeshApproximation::eConvexHull;
    }

    switch (approx)
    {
    case MeshApproximation::eConvexHull:
    {
        DescPtr<ConvexMeshPhysxShapeDesc> cmd = allocateDesc<ConvexMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        cmd->meshScale = meshScale;
        cmd->meshPrimKey = gprimKey;
        cmd->convexCookingParams.signScale = signScale;
        parseConvexHullCookingExt(ctx, shapeKey, cmd->convexCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(cmd));
    }
    case MeshApproximation::eMeshSimplification:
    {
        DescPtr<TriangleMeshPhysxShapeDesc> tmd = allocateDesc<TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = gprimKey;
        tmd->doubleSided = doubleSided;
        tmd->triangleMeshCookingParams.mode = omni::physx::TriangleMeshMode::eQUADRIC_SIMPLIFICATION;
        tmd->sdfMeshCookingParams.sdfResolution = 0;
        parseTriangleMeshSimplificationCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(tmd));
    }
    case MeshApproximation::eConvexDecomposition:
    {
        DescPtr<ConvexMeshDecompositionPhysxShapeDesc> cdd =
            allocateDesc<ConvexMeshDecompositionPhysxShapeDesc>(ctx.descriptorAllocator());
        cdd->meshScale = meshScale;
        cdd->meshPrimKey = gprimKey;
        cdd->doubleSided = doubleSided;
        cdd->convexDecompositionCookingParams.signScale = signScale;
        cdd->sdfMeshCookingParams.sdfResolution = 0;
        parseConvexDecompositionCookingExt(ctx, shapeKey, cdd->convexDecompositionCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(cdd));
    }
    case MeshApproximation::eSphereFill:
    {
        DescPtr<SpherePointsPhysxShapeDesc> sfd = allocateDesc<SpherePointsPhysxShapeDesc>(ctx.descriptorAllocator());
        sfd->meshScale = meshScale;
        sfd->meshPrimKey = gprimKey;
        sfd->doubleSided = doubleSided;
        sfd->sphereFillCookingParams.signScale = signScale;
        sfd->sdfMeshCookingParams.sdfResolution = 0;
        parseSphereFillCookingExt(ctx, shapeKey, sfd->sphereFillCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(sfd));
    }
    case MeshApproximation::eSdf:
    {
        DescPtr<TriangleMeshPhysxShapeDesc> tmd = allocateDesc<TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = gprimKey;
        tmd->doubleSided = doubleSided;
        parseTriangleMeshCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        parseSdfMeshCookingExt(ctx, shapeKey, tmd->sdfMeshCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(tmd));
    }
    case MeshApproximation::eBoundingSphere:
    case MeshApproximation::eBoundingCube:
    {
        // Read the gprim's points into a mergedMesh buffer; the consumer computes
        // the bounding-sphere radius / bounding-box halfExtents from those points.
        // (Mirrors NativeWalker's USD path; needed so a dynamic body gets an analytic
        // bounding shape instead of an illegal triangle-mesh simulation shape.)
        DescPtr<MergeMeshDesc> mm = allocateDesc<MergeMeshDesc>(ctx.descriptorAllocator());
        const MeshGeometry geom = parseMeshGeometry(ctx, gprimKey);
        const BufferSpan<carb::Float3> pointsView = ctx.getBuffer<carb::Float3>(geom.points);
        if (pointsView.count > 0)
            mm->points.assign(pointsView.data, pointsView.data + pointsView.count);

        DescPtr<MergeMeshPhysxShapeDesc> typed;
        if (approx == MeshApproximation::eBoundingSphere)
        {
            DescPtr<BoundingSpherePhysxShapeDesc> bs =
                allocateDesc<BoundingSpherePhysxShapeDesc>(ctx.descriptorAllocator());
            typed = descPtrCast<MergeMeshPhysxShapeDesc>(std::move(bs));
        }
        else
        {
            DescPtr<BoundingBoxPhysxShapeDesc> bb =
                allocateDesc<BoundingBoxPhysxShapeDesc>(ctx.descriptorAllocator());
            typed = descPtrCast<MergeMeshPhysxShapeDesc>(std::move(bb));
        }
        typed->mergedMesh = mm.get();
        out.ownedMeshes.push_back(std::move(mm));
        return descPtrCast<PhysxShapeDesc>(std::move(typed));
    }
    case MeshApproximation::eNone:
    default:
    {
        DescPtr<TriangleMeshPhysxShapeDesc> tmd = allocateDesc<TriangleMeshPhysxShapeDesc>(ctx.descriptorAllocator());
        tmd->meshScale = meshScale;
        tmd->meshPrimKey = gprimKey;
        tmd->doubleSided = doubleSided;
        tmd->sdfMeshCookingParams.sdfResolution = 0;
        parseTriangleMeshCookingExt(ctx, shapeKey, tmd->triangleMeshCookingParams);
        return descPtrCast<PhysxShapeDesc>(std::move(tmd));
    }
    }
}

} // namespace

void emitTireFrictionTable(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    TireFrictionTableInfo info;
    info.defaultFrictionValue = 1.0f;
    if (!src.getAttribute(key, src.internToken("defaultFrictionValue"), info.defaultFrictionValue))
        src.getAttribute(key, src.internToken("physxVehicleTireFrictionTable:defaultFrictionValue"),
                         info.defaultFrictionValue);

    info.materialPaths = relationshipTargetsEither(src, key, "groundMaterials",
                                                   "physxVehicleTireFrictionTable:groundMaterials");
    readFloatArrayEither(src, key, "frictionValues", "physxVehicleTireFrictionTable:frictionValues",
                         info.frictionValues);

    if (DescPtr<TireFrictionTableDesc> desc = parseTireFrictionTable(ctx, key, info))
        out.tireFrictionTables.push_back(std::move(desc));
}

void emitVehicleContext(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    if (!src.isA(key, src.internToken("PhysicsScene")))
        return;
    if (DescPtr<VehicleContextDesc> desc = parseVehicleContext(ctx, key))
        out.vehicleContexts.push_back(std::move(desc));
}

void emitVehicleWheel(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<WheelDesc> desc = parseWheel(ctx, key))
        out.vehicleWheels.push_back(std::move(desc));
}

void emitVehicleTire(OvstageScanResult& out, ovstage_instance_t* inst, OvstageSource& src,
                     ParseContext& ctx, ObjectKey key, float massScale)
{
    TireInfo info;
    info.massScale = massScale;

    std::vector<carb::Float2> graph;
    if (readFloat2Array(src, key, "physxVehicleTire:frictionVsSlipGraph", graph))
    {
        if (graph.size() != 3)
            return;
        info.hasFrictionVsSlipGraph = true;
        for (uint32_t i = 0; i < 3; ++i)
            info.frictionVsSlipGraph[i] = graph[i];
    }

    const std::vector<ObjectKey> ftTargets = relationshipTargets(src, key, "physxVehicleTire:frictionTable");
    if (ftTargets.size() == 1)
    {
        if (isType(src, inst, ftTargets[0], "PhysxVehicleTireFrictionTable"))
            info.frictionTableKey = ftTargets[0];
        else
            info.frictionTableTooMany = true;
    }
    else if (ftTargets.size() > 1)
    {
        info.frictionTableTooMany = true;
    }

    if (DescPtr<TireDesc> desc = parseTire(ctx, key, info))
        out.vehicleTires.push_back(std::move(desc));
}

void emitVehicleSuspension(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<SuspensionDesc> desc = parseSuspension(ctx, key))
        out.vehicleSuspensions.push_back(std::move(desc));
}

void emitVehicleEngine(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx,
                       ObjectKey key, float kgmsScale)
{
    EngineInfo info;
    info.kgmsScale = kgmsScale;

    std::vector<carb::Float2> curve;
    if (readFloat2Array(src, key, "physxVehicleEngine:torqueCurve", curve))
    {
        info.hasTorqueCurve = true;
        info.torqueCurvePointCount = static_cast<uint32_t>(curve.size());
        const uint32_t copyCount = std::min(info.torqueCurvePointCount,
            static_cast<uint32_t>(EngineDesc::maxNumberOfTorqueCurvePoints));
        for (uint32_t i = 0; i < copyCount; ++i)
            info.torqueCurve[i] = curve[i];
    }

    if (DescPtr<EngineDesc> desc = parseEngine(ctx, key, info))
        out.vehicleEngines.push_back(std::move(desc));
}

void emitVehicleGears(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    GearsInfo info;
    info.hasRatios = readFloatArray(src, key, "physxVehicleGears:ratios", info.ratios);

    if (DescPtr<GearsDesc> desc = parseGears(ctx, key, info))
    {
        out.vehicleGears.push_back(std::move(desc));
        out.vehicleGearsPaths.push_back(key);
    }
}

void emitVehicleClutch(OvstageScanResult& out, ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    if (DescPtr<ClutchDesc> desc = parseClutch(ctx, key, kgmsScale))
    {
        out.vehicleClutches.push_back(std::move(desc));
        out.vehicleClutchPaths.push_back(key);
    }
}

void emitVehicleDriveBasic(OvstageScanResult& out, ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    if (DescPtr<DriveBasicDesc> desc = parseDriveBasic(ctx, key, kgmsScale))
        out.vehicleDrivesBasic.push_back(std::move(desc));
}

void emitVehicleDriveStandard(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DriveStandardInfo info;
    info.engineKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:engine", "PhysxVehicleEngineAPI");
    info.gearsKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:gears", "PhysxVehicleGearsAPI");
    info.autoGearBoxKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:autoGearBox",
                                          "PhysxVehicleAutoGearBoxAPI");
    info.clutchKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:clutch", "PhysxVehicleClutchAPI");

    if (DescPtr<DriveStandardDesc> desc = parseDriveStandard(ctx, key, info))
    {
        out.vehicleDrivesStandard.push_back(std::move(desc));
        out.vehicleDrivesStandardPaths.push_back(key);
        out.vehicleDrivesStandardCrossRefs.push_back(info);
    }
}

void readDifferentialInfo(OvstageSource& src, ObjectKey key, DifferentialInfo& info)
{
    readIntArray(src, key, "physxVehicleMultiWheelDifferential:wheels", info.wheels);
    info.hasTorqueRatios = readFloatArray(src, key, "physxVehicleMultiWheelDifferential:torqueRatios",
                                          info.torqueRatios);
    info.hasAverageWheelSpeedRatios = readFloatArray(
        src, key, "physxVehicleMultiWheelDifferential:averageWheelSpeedRatios", info.averageWheelSpeedRatios);
}

void emitVehicleMultiWheelDifferential(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DifferentialInfo info;
    readDifferentialInfo(src, key, info);
    if (DescPtr<MultiWheelDifferentialDesc> desc = parseMultiWheelDifferential(ctx, key, info))
    {
        out.vehicleMultiWheelDifferentials.push_back(std::move(desc));
        out.vehicleMultiWheelDifferentialPaths.push_back(key);
    }
}

void emitVehicleTankDifferential(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DifferentialInfo info;
    readDifferentialInfo(src, key, info);

    TankDifferentialInfo tankInfo;
    tankInfo.hasNumberOfWheelsPerTrack = readIntArray(
        src, key, "physxVehicleTankDifferential:numberOfWheelsPerTrack", tankInfo.numberOfWheelsPerTrack);
    tankInfo.hasThrustIndexPerTrack = readIntArray(
        src, key, "physxVehicleTankDifferential:thrustIndexPerTrack", tankInfo.thrustIndexPerTrack);
    tankInfo.hasWheelIndicesInTrackOrder = readIntArray(
        src, key, "physxVehicleTankDifferential:wheelIndicesInTrackOrder", tankInfo.wheelIndicesInTrackOrder);
    tankInfo.hasTrackToWheelIndices = readIntArray(
        src, key, "physxVehicleTankDifferential:trackToWheelIndices", tankInfo.trackToWheelIndices);

    if (DescPtr<TankDifferentialDesc> desc = parseTankDifferential(ctx, key, info, tankInfo))
    {
        out.vehicleTankDifferentials.push_back(std::move(desc));
        out.vehicleTankDifferentialPaths.push_back(key);
    }
}

void emitVehicleAutoGearBox(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    AutoGearBoxInfo info;
    info.hasUpRatios = readFloatArray(src, key, "physxVehicleAutoGearBox:upRatios", info.upRatios);
    info.hasDownRatios = readFloatArray(src, key, "physxVehicleAutoGearBox:downRatios", info.downRatios);

    if (DescPtr<AutoGearBoxDesc> desc = parseAutoGearBox(ctx, key, info))
    {
        out.vehicleAutoGearBoxes.push_back(std::move(desc));
        out.vehicleAutoGearBoxPaths.push_back(key);
    }
}

void emitVehicleBrakes(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    src.forEachMultiApplyInstance(key, "PhysxVehicleBrakesAPI", [&](std::string_view instance)
    {
        uint8_t brakesIndex = 0;
        if (instance == "brakes0")
            brakesIndex = 0;
        else if (instance == "brakes1")
            brakesIndex = 1;
        else
            return;

        const std::string attrBase = std::string("physxVehicleBrakes:") + std::string(instance) + ":";
        BrakesInfo info;
        readIntArray(src, key, (attrBase + "wheels").c_str(), info.wheels);
        info.hasTorqueMultipliers = readFloatArray(src, key, (attrBase + "torqueMultipliers").c_str(),
                                                   info.torqueMultipliers);

        if (DescPtr<BrakesDesc> desc = parseBrakes(ctx, key, instance, brakesIndex, info))
        {
            out.vehicleBrakes.push_back(std::move(desc));
            out.vehicleBrakesPaths.push_back(key);
            out.vehicleBrakesInstanceTokens.push_back(src.internToken(instance));
        }
    });
}

void emitVehicleSteeringBasic(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    SteeringBasicInfo info;
    readIntArray(src, key, "physxVehicleSteering:wheels", info.wheels);
    info.hasAngleMultipliers = readFloatArray(src, key, "physxVehicleSteering:angleMultipliers",
                                              info.angleMultipliers);

    if (DescPtr<SteeringBasicDesc> desc = parseSteeringBasic(ctx, key, info))
    {
        out.vehicleSteeringBasic.push_back(std::move(desc));
        out.vehicleSteeringBasicPaths.push_back(key);
    }
}

void emitVehicleSteeringAckermann(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<SteeringAckermannDesc> desc = parseSteeringAckermann(ctx, key))
    {
        out.vehicleSteeringAckermann.push_back(std::move(desc));
        out.vehicleSteeringAckermannPaths.push_back(key);
    }
}

void emitVehicleNonlinearCmdResponse(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    src.forEachMultiApplyInstance(key, "PhysxVehicleNonlinearCommandResponseAPI", [&](std::string_view instance)
    {
        const std::string attrBase = std::string("physxVehicleNonlinearCommandResponse:") +
                                     std::string(instance) + ":";
        NonlinearCmdResponseInfo info;
        info.hasCommandValues = readFloatArray(src, key, (attrBase + "commandValues").c_str(),
                                               info.commandValues);
        info.hasSpeedResponsesPerCommandValue = readIntArray(
            src, key, (attrBase + "speedResponsesPerCommandValue").c_str(), info.speedResponsesPerCommandValue);
        info.hasSpeedResponses = readFloat2Array(src, key, (attrBase + "speedResponses").c_str(),
                                                 info.speedResponses);

        if (DescPtr<NonlinearCmdResponseDesc> desc = parseNonlinearCmdResponse(ctx, key, instance, info))
        {
            out.vehicleNonlinearCmdResponses.push_back(std::move(desc));
            out.vehicleNonlinearCmdResponsePaths.push_back(key);
            out.vehicleNonlinearCmdResponseInstanceTokens.push_back(src.internToken(instance));
        }
    });
}

void emitVehicleSuspensionCompliance(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    SuspensionComplianceInfo info;
    readFloat2Array(src, key, "physxVehicleSuspensionCompliance:wheelToeAngle", info.wheelToeAngles);
    readFloat2Array(src, key, "physxVehicleSuspensionCompliance:wheelCamberAngle", info.wheelCamberAngles);
    readFloat4Array(src, key, "physxVehicleSuspensionCompliance:suspensionForceAppPoint",
                    info.suspensionForceAppPoints);
    readFloat4Array(src, key, "physxVehicleSuspensionCompliance:tireForceAppPoint", info.tireForceAppPoints);

    if (DescPtr<SuspensionComplianceDesc> desc = parseSuspensionCompliance(ctx, key, info))
    {
        out.vehicleSuspensionCompliances.push_back(std::move(desc));
        out.vehicleSuspensionCompliancePaths.push_back(key);
    }
}

void emitVehicleWheelAttachment(OvstageScanResult& out, ovstage_instance_t* inst, OvstageSource& src,
                                ParseContext& ctx, ObjectKey key)
{
    WheelAttachmentInfo info;
    info.wheelKey = resolveRelOrApi(src, key, "physxVehicleWheelAttachment:wheel", "PhysxVehicleWheelAPI");
    info.tireKey = resolveRelOrApi(src, key, "physxVehicleWheelAttachment:tire", "PhysxVehicleTireAPI");
    info.suspensionKey = resolveRelOrApi(src, key, "physxVehicleWheelAttachment:suspension",
                                         "PhysxVehicleSuspensionAPI");

    const TokenId vehicleApi = src.internToken("PhysxVehicleAPI");
    for (ObjectKey parent = src.getParent(key); parent.valid(); parent = src.getParent(parent))
    {
        if (src.hasSchema(parent, vehicleApi))
        {
            info.vehicleKey = parent;
            break;
        }
    }
    out.vehicleWheelAttachmentOwners.push_back({ info.vehicleKey, key });

    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:suspensionTravelDirection"),
                     info.suspensionTravelDirection);

    const TokenId suspForceTok = src.internToken("physxVehicleWheelAttachment:suspensionForceAppPointOffset");
    if (src.hasAuthoredAttribute(key, suspForceTok) && src.getAttribute(key, suspForceTok,
                                                                        info.suspensionForceAppPointOffset))
        info.state |= WheelAttachmentDesc::eHAS_SUSP_FORCE_APP_POINT;

    const TokenId suspFramePosTok = src.internToken("physxVehicleWheelAttachment:suspensionFramePosition");
    if (src.hasAuthoredAttribute(key, suspFramePosTok) && src.getAttribute(key, suspFramePosTok,
                                                                           info.suspensionFramePosition))
        info.state |= WheelAttachmentDesc::eHAS_SUSPENSION_FRAME;

    if (!(info.state & WheelAttachmentDesc::eHAS_SUSPENSION_FRAME))
    {
        const TokenId wheelComTok = src.internToken("physxVehicleWheelAttachment:wheelCenterOfMassOffset");
        if (src.hasAuthoredAttribute(key, wheelComTok) && src.getAttribute(key, wheelComTok,
                                                                           info.wheelCenterOfMassOffset))
            info.state |= WheelAttachmentDesc::eHAS_WHEEL_COM_OFFSET;
    }

    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:suspensionFrameOrientation"),
                     info.suspensionFrameOrientation);

    const TokenId tireForceTok = src.internToken("physxVehicleWheelAttachment:tireForceAppPointOffset");
    if (src.hasAuthoredAttribute(key, tireForceTok) && src.getAttribute(key, tireForceTok,
                                                                        info.tireForceAppPointOffset))
        info.state |= WheelAttachmentDesc::eHAS_TIRE_FORCE_APP_POINT;

    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:wheelFramePosition"),
                     info.wheelFramePosition);
    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:wheelFrameOrientation"),
                     info.wheelFrameOrientation);
    src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:driven"), info.driven);

    int64_t index = 0;
    if (src.getAttribute(key, src.internToken("physxVehicleWheelAttachment:index"), index))
        info.index = static_cast<int>(index);

    const std::vector<ObjectKey> cgTargets = relationshipTargets(src, key,
                                                                 "physxVehicleWheelAttachment:collisionGroup");
    if (cgTargets.size() == 1 && isType(src, inst, cgTargets[0], "PhysicsCollisionGroup"))
        info.collisionGroupKey = cgTargets[0];

    if (isXformable(src, key))
    {
        info.state |= WheelAttachmentDesc::eMANAGE_TRANSFORMS;
        const TokenId collisionApi = src.internToken("PhysicsCollisionAPI");
        bool foundShape = false;
        if (src.hasSchema(key, collisionApi))
        {
            info.shapeKey = key;
            info.state |= WheelAttachmentDesc::eHAS_SHAPE;
            foundShape = true;
        }

        bool sawNestedCollider = false;
        src.forEachChild(key, [&](ObjectKey child)
        {
            if (src.hasSchema(child, collisionApi))
            {
                if (foundShape)
                {
                    sawNestedCollider = true;
                    return;
                }
                info.shapeKey = child;
                info.state |= WheelAttachmentDesc::eHAS_SHAPE;
                foundShape = true;
            }
            src.forEachChild(child, [&](ObjectKey grandChild)
            {
                if (hasCollisionInSubtree(src, grandChild, collisionApi))
                    sawNestedCollider = true;
            });
        });
        if (sawNestedCollider)
            return;
    }

    if (DescPtr<WheelAttachmentDesc> desc = parseWheelAttachment(ctx, key, info))
    {
        out.vehicleWheelAttachments.push_back(std::move(desc));
        out.vehicleWheelAttachmentInfos.push_back(info);
    }
}

void emitVehicle(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx,
                 ObjectKey key, float lengthScale)
{
    VehicleInfo info;
    info.lengthScale = lengthScale;
    Matrix3d rotation;
    src.getLocalToWorldRotationAndScale(key, rotation, info.scale);
    if (info.scale.x == 0.0f || info.scale.y == 0.0f || info.scale.z == 0.0f)
        return;

    TokenId queryType;
    const TokenId sweepType = src.internToken("sweep");
    if (src.getAttribute(key, src.internToken("physxVehicle:suspensionLineQueryType"), queryType) &&
        queryType == sweepType)
        info.queryType = 1;

    if (DescPtr<VehicleDesc> desc = parseVehicle(ctx, key, info))
    {
        out.vehicles.push_back(std::move(desc));
        out.vehiclePaths.push_back(key);
    }
}


void emitDeformableMaterial(OvstageScanResult& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DescPtr<PhysxDeformableMaterialDesc> desc;
    if (src.hasSchema(key, src.internToken("OmniPhysicsSurfaceDeformableMaterialAPI")))
    {
        DescPtr<PhysxSurfaceDeformableMaterialDesc> surf = allocateDesc<PhysxSurfaceDeformableMaterialDesc>(ctx.descriptorAllocator());
        parseDeformableMaterial(ctx, key, *surf);
        parseSurfaceDeformableMaterial(ctx, key, *surf);
        desc = descPtrCast<PhysxDeformableMaterialDesc>(std::move(surf));
    }
    else
    {
        desc = allocateDesc<PhysxDeformableMaterialDesc>(ctx.descriptorAllocator());
        parseDeformableMaterial(ctx, key, *desc);
    }
    if (!desc)
        return;
    desc->poissonsRatio = std::min(desc->poissonsRatio, 0.4999f);
    desc->materialKey = key;
    out.deformableMaterials.push_back(std::move(desc));
}

void emitParticleSystem(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    DescPtr<ParticleSystemDesc> sys = parseParticleSystem(ctx, key);
    if (!sys)
        return;
    const float fluidRestOffset = sys->fluidRestOffset;
    out.particleSystems.push_back(std::move(sys));

    if (DescPtr<ParticleAnisotropyDesc> a = parseParticleAnisotropy(ctx, key))
        out.particleAnisotropies.push_back(std::move(a));
    if (DescPtr<ParticleSmoothingDesc> sm = parseParticleSmoothing(ctx, key))
        out.particleSmoothings.push_back(std::move(sm));
    if (DescPtr<ParticleIsosurfaceDesc> iso = parseParticleIsosurface(ctx, key, fluidRestOffset))
        out.particleIsosurfaces.push_back(std::move(iso));
}

void emitParticleSet(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<ParticleSetDesc> desc = parseParticleSet(ctx, key))
    {
        if (desc->particleSystemKey.valid() && ctx.source().exists(desc->particleSystemKey))
        {
            const ObjectKey canonicalSystemKey = ctx.source().canonicalKey(desc->particleSystemKey);
            const bool alreadyEmitted =
                std::any_of(out.particleSystems.begin(), out.particleSystems.end(),
                            [&](const DescPtr<ParticleSystemDesc>& existing)
                            {
                                return ctx.source().canonicalKey(existing->systemKey).handle == canonicalSystemKey.handle;
                            });
            if (!alreadyEmitted)
            {
                emitParticleSystem(out, ctx, desc->particleSystemKey);
            }
        }
        out.particleSets.push_back(std::move(desc));
    }
}

void emitParticleSampler(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<ParticleSamplingDesc> desc = parseParticleSampling(ctx, key))
    {
        out.particleSamplers.push_back(std::move(desc));
        out.particleSamplerKeys.push_back(key);
    }
}

ObjectType attachmentSubtypeForPrimType(std::string_view typeName)
{
    if (typeName == "OmniPhysicsVtxVtxAttachment") return eAttachmentVtxVtx;
    if (typeName == "OmniPhysicsVtxTriAttachment") return eAttachmentVtxTri;
    if (typeName == "OmniPhysicsVtxTetAttachment") return eAttachmentVtxTet;
    if (typeName == "OmniPhysicsVtxCrvAttachment") return eAttachmentVtxCrv;
    if (typeName == "OmniPhysicsVtxXformAttachment") return eAttachmentVtxXform;
    if (typeName == "OmniPhysicsTetXformAttachment") return eAttachmentTetXform;
    if (typeName == "OmniPhysicsTriTriAttachment") return eAttachmentTriTri;
    return eUndefined;
}

void emitAttachment(OvstageScanResult& out, ParseContext& ctx, ObjectKey key, ObjectType subtype)
{
    if (subtype == eUndefined)
        return;
    if (DescPtr<PhysxDeformableAttachmentDesc> desc = parseAttachment(ctx, key, subtype))
    {
        desc->primKey = key;
        out.attachments.push_back(std::move(desc));
    }
}

void emitElementCollisionFilter(OvstageScanResult& out, ParseContext& ctx, ObjectKey key)
{
    if (DescPtr<PhysxDeformableCollisionFilterDesc> desc = parseElementCollisionFilter(ctx, key))
    {
        desc->primKey = key;
        out.deformableCollisionFilters.push_back(std::move(desc));
    }
}

bool isPointBased(const OvstageSource& src, ObjectKey key)
{
    return src.isA(key, src.internToken("PointBased"));
}

bool isDeformableSimMesh(const OvstageSource& src, ovstage_instance_t* inst, ObjectKey key, ObjectType& outType)
{
    outType = eUndefined;
    if (src.hasSchema(key, src.internToken("OmniPhysicsVolumeDeformableSimAPI")))
    {
        if (!isType(src, inst, key, "TetMesh"))
            return false;
        outType = eVolumeDeformableBody;
        return true;
    }
    if (src.hasSchema(key, src.internToken("OmniPhysicsSurfaceDeformableSimAPI")))
    {
        if (!isType(src, inst, key, "Mesh"))
            return false;
        outType = eSurfaceDeformableBody;
        return true;
    }
    return false;
}

bool isEnabledCollisionGeom(const OvstageSource& src, ObjectKey key)
{
    if (!src.hasSchema(key, src.internToken("PhysicsCollisionAPI")))
        return false;
    if (!isPointBased(src, key))
        return false;
    bool enabled = true;
    src.getAttribute(key, src.internToken("physics:collisionEnabled"), enabled);
    return enabled;
}

TokenId findPoseNameFromPurpose(OvstageSource& src, ObjectKey key, std::string_view purpose)
{
    TokenId out;
    const TokenId purposeToken = src.internToken(purpose);
    src.forEachMultiApplyInstance(key, "OmniPhysicsDeformablePoseAPI", [&](std::string_view inst)
    {
        if (out.valid())
            return;
        // ovstage currently has no token-array source contract. Support the common
        // scalar-token case if a producer publishes a single purpose value.
        const std::string attr = std::string("omniphysics:") + std::string(inst) + ":purposes";
        TokenId tok;
        if (src.getAttribute(key, src.internToken(attr), tok) && tok == purposeToken)
            out = src.internToken(inst);
    });
    return out;
}

void collectPointBasedSubtree(OvstageSource& src, ObjectKey root, std::vector<ObjectKey>& out)
{
    if (isPointBased(src, root))
        out.push_back(root);
    src.forEachChild(root, [&](ObjectKey child)
    {
        collectPointBasedSubtree(src, child, out);
    });
}

void emitDeformableBody(OvstageScanResult& out, ovstage_instance_t* inst, OvstageSource& src,
                        ParseContext& ctx, ObjectKey key, const SourceUnits& units,
                        std::unordered_map<uint64_t, bool>& deformableBodyEnabledByCanonical)
{
    const bool validGeomType = isType(src, inst, key, "Mesh") || isType(src, inst, key, "TetMesh") ||
                               isType(src, inst, key, "BasisCurves");
    const bool validRootType = isXformable(src, key) && !src.isA(key, src.internToken("Gprim"));
    if (!validGeomType && !validRootType)
        return;

    const DeformableBodyParse parsed = parseDeformableBody(ctx, key);

    ObjectKey simMeshKey;
    ObjectType simMeshType = eUndefined;
    std::vector<ObjectKey> collisionGeomKeys;
    std::vector<TokenId> collisionGeomBindPoseTokens;
    std::vector<bool> collisionGeomLeftHandedOrientations;
    std::vector<ObjectKey> skinGeomKeys;
    std::vector<TokenId> skinGeomBindPoseTokens;
    std::vector<ObjectKey> hierarchyFilteredPairs;

    std::vector<ObjectKey> pointBased;
    collectPointBasedSubtree(src, key, pointBased);
    for (ObjectKey pKey : pointBased)
    {
        ObjectType candidateType = eUndefined;
        const bool isSim = isDeformableSimMesh(src, inst, pKey, candidateType);
        const bool isColl = isEnabledCollisionGeom(src, pKey);
        if (isSim)
        {
            if (simMeshKey.valid())
                return;
            const bool isBodyPrim = src.canonicalPath(pKey) == src.canonicalPath(key);
            const bool isImmediateChild = src.canonicalPath(src.getParent(pKey)) == src.canonicalPath(key);
            if (!isBodyPrim && !isImmediateChild)
                return;
            simMeshKey = pKey;
            simMeshType = candidateType;
        }
        if (isColl)
        {
            collisionGeomKeys.push_back(pKey);
            collisionGeomBindPoseTokens.push_back(findPoseNameFromPurpose(src, pKey, "bindPose"));
            TokenId orientation;
            const TokenId leftHandedToken = src.internToken("leftHanded");
            const bool leftHandedOrientation = src.getAttribute(pKey, src.internToken("orientation"), orientation) &&
                                               orientation == leftHandedToken;
            collisionGeomLeftHandedOrientations.push_back(leftHandedOrientation);

            std::vector<ObjectKey> childPairs = parseFilteredPairs(ctx, pKey);
            hierarchyFilteredPairs.insert(hierarchyFilteredPairs.end(), childPairs.begin(), childPairs.end());
        }
        if (!isSim && !isColl)
        {
            skinGeomKeys.push_back(pKey);
            skinGeomBindPoseTokens.push_back(findPoseNameFromPurpose(src, pKey, "bindPose"));
        }
    }

    if (!simMeshKey.valid())
        return;

    DescPtr<PhysxDeformableBodyDesc> desc;
    if (simMeshType == eSurfaceDeformableBody)
    {
        DescPtr<PhysxSurfaceDeformableBodyDesc> surf = allocateDesc<PhysxSurfaceDeformableBodyDesc>(ctx.descriptorAllocator());
        setToDefault(*surf, units);
        desc = descPtrCast<PhysxDeformableBodyDesc>(std::move(surf));
    }
    else
    {
        DescPtr<PhysxVolumeDeformableBodyDesc> vol = allocateDesc<PhysxVolumeDeformableBodyDesc>(ctx.descriptorAllocator());
        setToDefault(*vol, units);
        desc = descPtrCast<PhysxDeformableBodyDesc>(std::move(vol));
    }

    desc->primKey = key;
    desc->bodyEnabled = parsed.bodyEnabled;
    desc->kinematicBody = parsed.kinematicBody;
    desc->startsAsleep = parsed.startsAsleep;
    desc->mass = (parsed.mass <= 0.0f) ? -1.0f : parsed.mass;
    desc->sourceSimulationOwners = parsed.simulationOwners;
    desc->sourceFilteredCollisions = parsed.filteredCollisions;
    desc->sourceFilteredCollisions.insert(desc->sourceFilteredCollisions.end(),
                                          hierarchyFilteredPairs.begin(), hierarchyFilteredPairs.end());

    Matrix4d transform;
    src.getLocalToWorldTransform(key, transform);
    std::memcpy(desc->transform.data, transform.data, sizeof(double) * 16);

    desc->simMeshKey = simMeshKey;
    desc->simMeshBindPoseToken = findPoseNameFromPurpose(src, simMeshKey, "bindPose");
    {
        TokenId orientation;
        const TokenId leftHandedToken = src.internToken("leftHanded");
        desc->simMeshLeftHandedOrientation =
            src.getAttribute(simMeshKey, src.internToken("orientation"), orientation) &&
            orientation == leftHandedToken;
    }

    if (!collisionGeomKeys.empty())
    {
        desc->collisionMeshKey = collisionGeomKeys.front();
        if (!collisionGeomBindPoseTokens.empty())
            desc->collisionMeshBindPoseToken = collisionGeomBindPoseTokens.front();
        if (!collisionGeomLeftHandedOrientations.empty())
            desc->collisionMeshLeftHandedOrientation = collisionGeomLeftHandedOrientations.front();
    }

    desc->skinGeomPaths = std::move(skinGeomKeys);
    desc->skinGeomBindPoseTokens = std::move(skinGeomBindPoseTokens);

    const TokenId baseDBTok = src.internToken("PhysxBaseDeformableBodyAPI");
    if (src.hasSchema(key, baseDBTok))
    {
        auto rdF = [&](const char* name, float def) {
            float v = def;
            src.getAttribute(key, src.internToken(name), v);
            return v;
        };
        auto rdB = [&](const char* name, bool def) {
            bool v = def;
            src.getAttribute(key, src.internToken(name), v);
            return v;
        };
        auto rdU = [&](const char* name, uint32_t def) {
            int64_t iv = 0;
            if (src.getAttribute(key, src.internToken(name), iv))
                return uint32_t(iv);
            return def;
        };
        desc->linearDamping = rdF("physxDeformableBody:linearDamping", desc->linearDamping);
        desc->maxLinearVelocity = rdF("physxDeformableBody:maxLinearVelocity", desc->maxLinearVelocity);
        desc->sleepThreshold = rdF("physxDeformableBody:sleepThreshold", desc->sleepThreshold);
        desc->settlingThreshold = rdF("physxDeformableBody:settlingThreshold", desc->settlingThreshold);
        desc->settlingDamping = rdF("physxDeformableBody:settlingDamping", desc->settlingDamping);
        desc->maxDepenetrationVelocity = rdF("physxDeformableBody:maxDepenetrationVelocity", desc->maxDepenetrationVelocity);
        desc->selfCollisionFilterDistance = rdF("physxDeformableBody:selfCollisionFilterDistance", desc->selfCollisionFilterDistance);
        desc->solverPositionIterationCount = rdU("physxDeformableBody:solverPositionIterationCount", desc->solverPositionIterationCount);
        desc->enableSpeculativeCCD = rdB("physxDeformableBody:enableSpeculativeCCD", desc->enableSpeculativeCCD);
        desc->selfCollision = rdB("physxDeformableBody:selfCollision", desc->selfCollision);
        desc->disableGravity = rdB("physxDeformableBody:disableGravity", desc->disableGravity);
    }

    if (simMeshType == eSurfaceDeformableBody && src.hasSchema(key, src.internToken("PhysxSurfaceDeformableBodyAPI")))
    {
        PhysxSurfaceDeformableBodyDesc* surf = static_cast<PhysxSurfaceDeformableBodyDesc*>(desc.get());
        int64_t freq = 0;
        if (src.getAttribute(key, src.internToken("physxDeformableBody:collisionPairUpdateFrequency"), freq))
            surf->collisionPairUpdateFrequency = uint32_t(freq);
        int64_t mult = 0;
        if (src.getAttribute(key, src.internToken("physxDeformableBody:collisionIterationMultiplier"), mult))
            surf->collisionIterationMultiplier = uint32_t(mult);
    }

    if (src.hasSchema(key, src.internToken("PhysxAutoDeformableBodyAPI")))
    {
        src.getAttribute(key, src.internToken("physxDeformableBody:autoDeformableBodyEnabled"), desc->hasAutoAPI);
        std::vector<ObjectKey> cookingTargets;
        src.getRelationshipTargets(key, src.internToken("physxDeformableBody:cookingSourceMesh"), cookingTargets);
        if (!cookingTargets.empty())
        {
            desc->cookingSrcMeshKey = cookingTargets.front();
            desc->cookingSrcMeshBindPoseToken = findPoseNameFromPurpose(src, cookingTargets.front(), "bindPose");
        }

        if (src.hasSchema(key, src.internToken("PhysxAutoDeformableMeshSimplificationAPI")))
        {
            auto rdU = [&](const char* name, uint32_t def) {
                int64_t iv = 0;
                if (src.getAttribute(key, src.internToken(name), iv))
                    return uint32_t(iv);
                return def;
            };
            auto rdB = [&](const char* name, bool def) {
                bool v = def;
                src.getAttribute(key, src.internToken(name), v);
                return v;
            };
            desc->isAutoMeshSimplificationEnabled = rdB("physxDeformableBody:autoDeformableMeshSimplificationEnabled",
                                                        desc->isAutoMeshSimplificationEnabled);
            desc->isAutoRemeshingEnabled = rdB("physxDeformableBody:remeshingEnabled", desc->isAutoRemeshingEnabled);
            desc->autoRemeshingResolution = rdU("physxDeformableBody:remeshingResolution", desc->autoRemeshingResolution);
            desc->autoTriangleTargetCount = rdU("physxDeformableBody:targetTriangleCount", desc->autoTriangleTargetCount);
            desc->hasAutoForceConforming = rdB("physxDeformableBody:forceConforming", desc->hasAutoForceConforming);
        }
        if (simMeshType != eSurfaceDeformableBody && src.hasSchema(key, src.internToken("PhysxAutoDeformableHexahedralMeshAPI")))
        {
            PhysxVolumeDeformableBodyDesc* vol = static_cast<PhysxVolumeDeformableBodyDesc*>(desc.get());
            vol->isAutoHexahedralMeshEnabled = true;
            int64_t resolution = 0;
            if (src.getAttribute(key, src.internToken("physxDeformableBody:resolution"), resolution))
                vol->autoHexahedralResolution = uint32_t(resolution);
        }
    }

    deformableBodyEnabledByCanonical[src.canonicalPath(key)] = parsed.bodyEnabled;
    out.deformables.push_back(std::move(desc));
}

OvstageScanResult scanOvstage(ovstage_instance_t* instance,
                              ovx_path_dictionary_t* dict,
                              IDescriptorAllocator& allocator,
                              ovstage_ordinal_t readOrdinal,
                              const OvstageScanFilter* filter,
                              uint64_t usdStageId)
{
    OvstageScanResult out;
    if (!instance || !dict)
        return out;

    std::unique_ptr<OvstageSource> source = std::make_unique<OvstageSource>(instance, dict, readOrdinal, usdStageId);
    OvstageSource& src = *source;
    out.source = std::move(source);
    src.beginLoadCache();
    struct ScopedLoadCache
    {
        OvstageSource& source;
        ~ScopedLoadCache() { source.clearLoadCache(); }
    } scopedLoadCache{ src };

    ParseContext ctx(src, allocator);
    const SourceUnits units = src.getSourceUnits();
    const float massScale = (units.kilogramsPerUnit != 0.0f) ? (1.0f / units.kilogramsPerUnit) : 1.0f;
    const float lengthScale = (units.metersPerUnit != 0.0f) ? (1.0f / units.metersPerUnit) : 1.0f;
    const float kgmsScale = (lengthScale * lengthScale) * massScale;

    auto isRootPrimPath = [](std::string_view path) -> bool
    {
        return path.size() > 1 && path[0] == '/' && path.find('/', 1) == std::string_view::npos;
    };
    auto pathIsAtOrBelow = [](std::string_view path, std::string_view root) -> bool
    {
        if (root.empty() || root == "/")
            return true;
        return path == root || (path.size() > root.size() &&
                                path.compare(0, root.size(), root) == 0 &&
                                path[root.size()] == '/');
    };
    std::vector<std::string> pointInstancerSubtreeRoots;
    auto keyPassesFilter = [&](ObjectKey key) -> bool
    {
        if (src.isPrototypeBackingKey(key))
            return false;
        if (!filter || (filter->scanRoots.empty() && filter->excludePaths.empty()))
            return true;

        const std::string path(src.sourceKeyToString(key));
        bool inRoots = filter->scanRoots.empty();
        for (const std::string& root : filter->scanRoots)
        {
            if (pathIsAtOrBelow(path, root))
            {
                inRoots = true;
                break;
            }
        }
        if (!inRoots)
            return false;

        for (const std::string& exclude : filter->excludePaths)
        {
            if (isRootPrimPath(exclude))
                continue;
            if (pathIsAtOrBelow(path, exclude))
                return false;
        }
        return true;
    };
    auto isBelowPointInstancer = [&](ObjectKey key) -> bool
    {
        if (pointInstancerSubtreeRoots.empty())
            return false;
        const std::string path(src.sourceKeyToString(key));
        for (const std::string& root : pointInstancerSubtreeRoots)
            if (path != root && pathIsAtOrBelow(path, root))
                return true;
        return false;
    };

    // Ovstage enumeration and schema-cache iteration order can vary with
    // process history. Re-rank every bucket by one stable depth-first traversal
    // of the path-sorted source hierarchy so scan order depends only on the
    // populated stage.
    std::unordered_map<uint64_t, uint32_t> namespaceOrder;
    bool namespaceOrderBuilt = false;
    auto buildNamespaceOrder = [&]()
    {
        if (namespaceOrderBuilt)
            return;
        namespaceOrderBuilt = true;

        std::vector<ObjectKey> hierarchy;
        src.collectDescendantKeys(src.getRootKey(), hierarchy);
        namespaceOrder.reserve(hierarchy.size() * 2);
        uint32_t index = 0;
        for (const ObjectKey key : hierarchy)
        {
            namespaceOrder.emplace(key.handle, index);
            const uint64_t canonical = src.canonicalPath(key);
            if (canonical && canonical != key.handle)
                namespaceOrder.emplace(canonical, index);
            ++index;
        }
    };
    auto orderBucketKeys = [&](std::vector<ObjectKey>& keys)
    {
        if (keys.size() < 2)
            return;
        buildNamespaceOrder();

        struct RankedKey
        {
            uint32_t rank;
            std::string fallback;
            ObjectKey key;
        };

        std::vector<RankedKey> ranked;
        ranked.reserve(keys.size());
        for (const ObjectKey key : keys)
        {
            std::unordered_map<uint64_t, uint32_t>::const_iterator it = namespaceOrder.find(key.handle);
            if (it == namespaceOrder.end())
            {
                const uint64_t canonical = src.canonicalPath(key);
                if (canonical)
                    it = namespaceOrder.find(canonical);
            }
            if (it != namespaceOrder.end())
            {
                ranked.push_back(RankedKey{ it->second, std::string(), key });
            }
            else
            {
                ranked.push_back(
                    RankedKey{ std::numeric_limits<uint32_t>::max(), std::string(src.sourceKeyToString(key)), key });
            }
        }
        std::sort(ranked.begin(), ranked.end(),
                  [](const RankedKey& a, const RankedKey& b)
                  {
                      if (a.rank != b.rank)
                          return a.rank < b.rank;
                      return a.fallback < b.fallback;
                  });
        for (size_t i = 0; i < ranked.size(); ++i)
            keys[i] = ranked[i].key;
    };

    auto applyScanFilter = [&](Bucket bucket) -> Bucket
    {
        if (bucket.keys.empty())
            return bucket;
        bucket.keys.erase(std::remove_if(bucket.keys.begin(), bucket.keys.end(),
                                         [&](ObjectKey key)
                                         {
                                             return !keyPassesFilter(key) || isBelowPointInstancer(key);
                                         }),
                          bucket.keys.end());
        orderBucketKeys(bucket.keys);
        return bucket;
    };
    auto enumerateFiltered = [&](const char* predAttr, ovstage_filter_op_t op, const char* predValue,
                                 const char* probeAttr, ovstage_ordinal_t ordinal) -> Bucket
    {
        return applyScanFilter(enumerate(instance, dict, predAttr, op, predValue, probeAttr, ordinal, nullptr));
    };
    auto enumeratePrimTypeFiltered = [&](const char* typeName, const char* probeAttr,
                                         ovstage_ordinal_t ordinal) -> Bucket
    {
        Bucket bucket = enumerate(instance, dict, "usd-prim-type", OVSTAGE_FILTER_OP_IN,
                                  typeName, probeAttr, ordinal, conv::kUsdPath);
        return applyScanFilter(std::move(bucket));
    };
    auto enumerateSchemaFiltered = [&](const char* schemaName, const char* probeAttr,
                                       ovstage_ordinal_t ordinal) -> Bucket
    {
        const TokenId schemaToken = src.internToken(schemaName);
        std::vector<ObjectKey> cachedKeys;
        if (src.collectSchemaKeys(schemaToken, cachedKeys))
        {
            if (cachedKeys.empty())
                return applyScanFilter(Bucket{});

            (void)probeAttr;
            (void)ordinal;
            Bucket bucket;
            bucket.keys = std::move(cachedKeys);
            return applyScanFilter(std::move(bucket));
        }

        Bucket bucket = enumerateSchema(instance, dict, schemaName, probeAttr, ordinal);
        return applyScanFilter(bucket);
    };
    auto enumerateSchemasFiltered = [&](const char* const* schemaNames, size_t schemaCount, const char* probeAttr,
                                        ovstage_ordinal_t ordinal) -> Bucket
    {
        Bucket outBucket;
        for (size_t i = 0; i < schemaCount; ++i)
        {
            Bucket one = enumerateSchemaFiltered(schemaNames[i], probeAttr, ordinal);
            appendUnique(outBucket.keys, one.keys);
            for (const std::string& attr : one.attrs)
            {
                if (std::find(outBucket.attrs.begin(), outBucket.attrs.end(), attr) == outBucket.attrs.end())
                    outBucket.attrs.push_back(attr);
            }
        }
        // Re-order the union rather than preserving schema-bucket append order.
        orderBucketKeys(outBucket.keys);
        return outBucket;
    };

    const TokenId tokRigidBodyEnabled = src.internToken("physics:rigidBodyEnabled");
    const TokenId tokGravityDir = src.internToken("physics:gravityDirection");
    const TokenId tokGravityMag = src.internToken("physics:gravityMagnitude");
    const TokenId tokSize = src.internToken("size");
    const TokenId tokRadius = src.internToken("radius");
    const TokenId tokHeight = src.internToken("height");
    const TokenId tokAxis = src.internToken("axis");
    const TokenId tokAxisX = src.internToken("X");
    const TokenId tokAxisY = src.internToken("Y");
    const TokenId tokAxisZ = src.internToken("Z");
    const TokenId tokDoubleSided = src.internToken("doubleSided");
    const TokenId tokRigidBodyAPI = src.internToken("PhysicsRigidBodyAPI");
    const TokenId tokCollisionAPI = src.internToken("PhysicsCollisionAPI");
    const TokenId tokCollisionEnabled = src.internToken("physics:collisionEnabled");
    const TokenId tokSimulationOwner = src.internToken("physics:simulationOwner");

    const bool prunePointInstancerDescendants = !filter || filter->prunePointInstancerDescendants;
    auto collectPointInstancerPrimType = [&](const char* typeName)
    {
        const Bucket pointInstancerBucket = enumeratePrimTypeFiltered(typeName, conv::kLocalTransform, readOrdinal);
        if (!pointInstancerBucket.keys.empty())
            out.hasPointInstancerPrims = true;
        if (!prunePointInstancerDescendants)
            return;
        pointInstancerSubtreeRoots.reserve(pointInstancerSubtreeRoots.size() + pointInstancerBucket.keys.size());
        for (const ObjectKey key : pointInstancerBucket.keys)
            pointInstancerSubtreeRoots.emplace_back(src.sourceKeyToString(key));
    };
    collectPointInstancerPrimType("PointInstancer");
    collectPointInstancerPrimType("PhysxPhysicsJointInstancer");

    // --- Scenes (PRIM_TYPE == PhysicsScene) ---
    Bucket sceneBucket = enumeratePrimTypeFiltered("PhysicsScene", "physics:gravityMagnitude", readOrdinal);
    const std::vector<std::string> sceneDiscoveredAttrs = sceneBucket.attrs;
    const size_t sceneAttrCount = sceneBucket.attrs.size();
    appendSceneAttrs(sceneBucket);
    src.prefetchBucket(sceneBucket.keys, sceneDiscoveredAttrs);
    src.prefetchBucket(sceneBucket.keys, appendedAttrs(sceneBucket, sceneAttrCount));
    for (const ObjectKey key : sceneBucket.keys)
    {
        DescPtr<PhysxSceneDesc> desc = allocateDesc<PhysxSceneDesc>(allocator);
        if (!desc)
            continue;
        setToDefault(*desc, units);

        SceneInfo info;
        carb::Float3 dir{};
        float mag = 0.0f;
        if (src.getAttribute(key, tokGravityDir, dir))
            info.gravityDirection = dir;
        if (src.getAttribute(key, tokGravityMag, mag))
            info.gravityMagnitude = mag;

        parseScene(ctx, key, info, *desc);
        desc->primKey = key;
        out.scenes.push_back(std::move(desc));
    }
    src.clearBucket();

    // No authored PhysicsScene → synthesize a default scene descriptor so a lone
    // body still simulates (parity with the USD path, which authors a temp scene
    // prim). ovstage authors NO prim: this is an in-memory descriptor flagged
    // `synthetic`, with a stable synthetic path so pathFor/keyFor round-trip in the
    // consumer (which skips the source existence/ownership gates for it). See
    // makeDefaultSceneDesc + LoadStage processScannedDescs.
    //
    // "This stage authors no PhysicsScene" is a whole-stage fact and cannot be
    // concluded from a scoped scan. An incremental re-scan rooted at a newly added
    // prim normally contains no scene, so synthesizing one here published a
    // spurious extra `/__defaultPhysicsScene__` on every runtime prim add.
    //
    // An exclude-only filter also scopes the scan unless every excluded path is a
    // root prim. Root-prim excludes are ignored by keyPassesFilter for parity with
    // the legacy traversal path and therefore do not make the scan scoped.
    const bool excludesScopeTheScan =
        filter && std::any_of(filter->excludePaths.begin(), filter->excludePaths.end(),
                              [&](const std::string& exclude) { return !isRootPrimPath(exclude); });
    const bool wholeStageScan = !filter || (filter->scanRoots.empty() && !excludesScopeTheScan);
    if (out.scenes.empty() && wholeStageScan)
    {
        if (DescPtr<PhysxSceneDesc> def = makeDefaultSceneDesc(allocator, units))
        {
            def->primKey = src.findByPath(conv::kDefaultScenePath);
            out.scenes.push_back(std::move(def));
        }
    }

    // --- Materials (HAS_APPLIED_SCHEMA PhysicsMaterialAPI) ---
    // Emit the material descriptors (friction / restitution / density). The
    // material→shape binding is resolved by ovpopulation as a target-path
    // relationship and read back via OvstageSource::getMaterialBinding, wired
    // into each shape's sourceMaterials in the shape loop below.
    Bucket materialBucket = enumerateSchemaFiltered("PhysicsMaterialAPI",
                                                    "physics:dynamicFriction",
                                                    readOrdinal);
    const std::vector<std::string> materialDiscoveredAttrs = materialBucket.attrs;
    const size_t materialAttrCount = materialBucket.attrs.size();
    appendMaterialAttrs(materialBucket);
    src.prefetchBucket(materialBucket.keys, materialDiscoveredAttrs);
    src.prefetchBucket(materialBucket.keys, appendedAttrs(materialBucket, materialAttrCount));
    for (const ObjectKey key : materialBucket.keys)
    {
        DescPtr<PhysxMaterialDesc> mat = parseMaterial(ctx, key);
        if (!mat)
            continue;
        mat->materialKey = key; // parseMaterial sets this; keep explicit alongside the other concepts
        out.materials.push_back(std::move(mat));
    }
    src.clearBucket();

    // --- PBD materials (PhysxPBDMaterialAPI) ---
    // ovpopulation currently exposes material prims through PhysicsMaterialAPI;
    // probe those visible material keys for PBD attributes because usd-schemas may
    // not include PhysxPBDMaterialAPI under ovstage yet.
    const std::vector<std::string> pbdMaterialAttrs = {
        "physxPBDMaterial:friction",
        "physxPBDMaterial:particleFrictionScale",
        "physxPBDMaterial:damping",
        "physxPBDMaterial:viscosity",
        "physxPBDMaterial:vorticityConfinement",
        "physxPBDMaterial:surfaceTension",
        "physxPBDMaterial:cohesion",
        "physxPBDMaterial:adhesion",
        "physxPBDMaterial:particleAdhesionScale",
        "physxPBDMaterial:adhesionOffsetScale",
        "physxPBDMaterial:gravityScale",
        "physxPBDMaterial:cflCoefficient",
        "physxPBDMaterial:density",
    };
    src.prefetchBucket(materialBucket.keys, pbdMaterialAttrs);
    for (const ObjectKey key : materialBucket.keys)
    {
        DescPtr<PBDMaterialDesc> mat = parsePBDMaterial(ctx, key);
        if (!mat)
            continue;
        mat->materialKey = key;
        out.pbdMaterials.push_back(std::move(mat));
    }
    src.clearBucket();

    // --- Deformable materials (OmniPhysicsDeformableMaterialAPI family) ---
    const char* const deformableMaterialSchemas[] = {
        "OmniPhysicsDeformableMaterialAPI",
        "OmniPhysicsSurfaceDeformableMaterialAPI",
        "OmniPhysicsCurveDeformableMaterialAPI",
    };
    const Bucket deformableMaterialBucket = enumerateSchemasFiltered(deformableMaterialSchemas, 3,
                                                             "omniphysics:youngsModulus", readOrdinal);
    src.prefetchBucket(deformableMaterialBucket.keys, deformableMaterialBucket.attrs);
    for (const ObjectKey key : deformableMaterialBucket.keys)
        emitDeformableMaterial(out, src, ctx, key);
    src.clearBucket();

    // --- Rigid bodies (HAS_APPLIED_SCHEMA PhysicsRigidBodyAPI) ---
    std::unordered_set<ObjectKey, ObjectKey::Hash> bodySet;
    std::unordered_map<ObjectKey, PhysxRigidBodyDesc*, ObjectKey::Hash> bodyByKey;
    // canonical intern_path id → body's enumerate key. Populated relationship
    // targets (physics:body0/1) are canonical intern_path ids, which the ovstage
    // lib hands back DISTINCT from the enumerate/get_paths handles in `bodySet`
    // for the same path. This maps a canonical handle back to the enumerate body
    // key so a resolved body matches the body descriptor's primKey.
    std::unordered_map<uint64_t, ObjectKey> bodyByCanonical;

    Bucket bodyBucket = enumerateSchemaFiltered("PhysicsRigidBodyAPI",
                                                conv::kLocalTransform,
                                                readOrdinal);
    const std::vector<std::string> bodyDiscoveredAttrs = bodyBucket.attrs;
    const size_t bodyAttrCount = bodyBucket.attrs.size();
    appendRigidBodyAttrs(bodyBucket);
    appendMassAttrs(bodyBucket);
    prefetchTransformAncestors(src, bodyBucket.keys);
    prefetchTransformsForKeys(src, bodyBucket.keys);
    src.prefetchBucket(bodyBucket.keys, bodyDiscoveredAttrs);
    src.prefetchBucket(bodyBucket.keys, appendedAttrs(bodyBucket, bodyAttrCount));
    for (const ObjectKey key : bodyBucket.keys)
    {
        bool enabled = true;
        bool enabledVal = false;
        if (src.getAttribute(key, tokRigidBodyEnabled, enabledVal))
            enabled = enabledVal;

        DescPtr<PhysxRigidBodyDesc> base;
        if (enabled)
        {
            DescPtr<DynamicPhysxRigidBodyDesc> dyn = parseDynamicBody(ctx, key);
            if (!dyn)
                continue;
            base = descPtrCast<PhysxRigidBodyDesc>(std::move(dyn));
        }
        else
        {
            DescPtr<StaticPhysxRigidBodyDesc> stat = parseStaticBody(ctx, key);
            if (!stat)
                continue;
            stat->sourceGPrimKey = key;
            base = descPtrCast<PhysxRigidBodyDesc>(std::move(stat));
        }

        base->primKey = key;
        // World-space body pose from the resolved transform. The engine creates
        // actors from position + rotation, so preserve both here.
        Matrix4d m;
        src.getLocalToWorldTransform(key, m);
        base->position = { static_cast<float>(m.data[12]), static_cast<float>(m.data[13]),
                           static_cast<float>(m.data[14]) };
        Matrix3d rotation;
        carb::Float3 scale{ 1.0f, 1.0f, 1.0f };
        src.getLocalToWorldRotationAndScale(key, rotation, scale);
        base->rotation = quatFromRotation(rotation);
        base->scale = scale;

        // Per-body collision filtering (PhysxFilteredPairsAPI → physics:filteredPairs);
        // no-op when the API is absent.
        base->sourceFilteredCollisions = parseFilteredPairs(ctx, key);
        src.getRelationshipTargets(key, tokSimulationOwner, base->sourceSimulationOwners);

        bodySet.insert(key);
        bodyByKey.emplace(key, base.get());
        bodyByCanonical.emplace(src.canonicalPath(key), key);
        out.bodies.push_back(std::move(base));
    }
    src.clearBucket();

    // Resolve the nearest enclosing rigid body for a collision shape, walking
    // the path-parent chain (mirrors the native walker's body-of-shape walk).
    // Ovstage can hand back distinct ObjectKey handles for the same path across
    // schema queries, so compare canonical path ids and return the body query key.
    auto resolveBody = [&](ObjectKey shapeKey) -> ObjectKey
    {
        ObjectKey k = shapeKey;
        for (int guard = 0; k.valid() && guard < 64; ++guard)
        {
            const auto it = bodyByCanonical.find(src.canonicalPath(k));
            if (it != bodyByCanonical.end())
                return it->second;
            k = src.getParent(k);
        }
        return {};
    };

    // --- Deformable bodies (HAS_APPLIED_SCHEMA OmniPhysicsDeformableBodyAPI) ---
    // Parse before rigid collision shapes so deformable collision children can be
    // skipped by the rigid-shape scan, matching the native walker.
    std::unordered_map<uint64_t, bool> deformableBodyEnabledByCanonical;
    Bucket deformableBodyBucket = enumerateSchemaFiltered("OmniPhysicsDeformableBodyAPI",
                                                          conv::kLocalTransform, readOrdinal);
    const std::vector<std::string> deformableBodyDiscoveredAttrs = deformableBodyBucket.attrs;
    const size_t deformableBodyAttrCount = deformableBodyBucket.attrs.size();
    appendDeformableBodyAttrs(deformableBodyBucket);
    prefetchTransformAncestors(src, deformableBodyBucket.keys);
    prefetchTransformsForKeys(src, deformableBodyBucket.keys);
    src.prefetchBucket(deformableBodyBucket.keys, deformableBodyDiscoveredAttrs);
    src.prefetchBucket(deformableBodyBucket.keys, appendedAttrs(deformableBodyBucket, deformableBodyAttrCount));
    for (const ObjectKey key : deformableBodyBucket.keys)
        emitDeformableBody(out, instance, src, ctx, key, units, deformableBodyEnabledByCanonical);
    src.clearBucket();

    auto isDeformableCollider = [&](ObjectKey shapeKey) -> bool
    {
        ObjectKey k = shapeKey;
        for (int guard = 0; k.valid() && guard < 64; ++guard)
        {
            auto it = deformableBodyEnabledByCanonical.find(src.canonicalPath(k));
            if (it != deformableBodyEnabledByCanonical.end() && it->second)
                return true;
            k = src.getParent(k);
        }
        return false;
    };

    // Resolve a USD round-shape `axis` token (X/Y/Z) to a parse::Axis; USD's
    // default for capsule/cylinder/cone is Z. ovstage stores the token as an id
    // column, so keep it in source token space.
    auto readAxis = [&](ObjectKey key) -> Axis
    {
        TokenId at{};
        if (src.getAttribute(key, tokAxis, at) && at.valid())
        {
            if (at == tokAxisX)
                return eX;
            if (at == tokAxisY)
                return eY;
        }
        return eZ;
    };

    // --- Collision shapes (HAS_APPLIED_SCHEMA PhysicsCollisionAPI) ---
    // Analytic and mesh geometry keep logical identity and transforms on an
    // instance-proxy path while reading authored geometry from its prototype.
    Bucket shapeBucket = enumerateSchemaFiltered("PhysicsCollisionAPI",
                                                 conv::kLocalTransform,
                                                 readOrdinal);
    const std::vector<std::string> shapeDiscoveredAttrs = shapeBucket.attrs;
    const size_t shapeAttrCount = shapeBucket.attrs.size();
    appendCollisionShapeAttrs(shapeBucket);
    appendMassAttrs(shapeBucket);
    prefetchTransformAncestors(src, shapeBucket.keys);
    prefetchTransformsForKeys(src, shapeBucket.keys);
    std::vector<ObjectKey> shapeReadKeys = shapeBucket.keys;
    std::unordered_set<uint64_t> shapeReadCanonical;
    shapeReadCanonical.reserve(shapeBucket.keys.size() * 2);
    for (const ObjectKey key : shapeBucket.keys)
    {
        const uint64_t canonical = src.canonicalPath(key);
        if (canonical)
            shapeReadCanonical.insert(canonical);
    }
    for (const ObjectKey key : shapeBucket.keys)
    {
        const ObjectKey backingKey = src.geometryBackingKey(key);
        const uint64_t canonical = src.canonicalPath(backingKey);
        if (canonical && shapeReadCanonical.insert(canonical).second)
            shapeReadKeys.push_back(backingKey);
    }
    src.prefetchBucket(shapeReadKeys, shapeDiscoveredAttrs);
    src.prefetchBucket(shapeReadKeys, appendedAttrs(shapeBucket, shapeAttrCount));
    enum class PrimType : uint8_t
    {
        eCube,
        eSphere,
        eCapsule,
        eCylinder,
        eCone,
        ePlane,
        eMesh,
    };
    std::unordered_map<uint64_t, PrimType> primTypeByCanonical;
    auto rememberPrimType = [&](const char* typeName, PrimType primType)
    {
        const Bucket typeBucket = enumeratePrimTypeFiltered(typeName, conv::kLocalTransform, readOrdinal);
        for (const ObjectKey typeKey : typeBucket.keys)
        {
            const uint64_t canonical = src.canonicalPath(typeKey);
            if (canonical)
                primTypeByCanonical[canonical] = primType;
        }
    };
    rememberPrimType("Cube", PrimType::eCube);
    rememberPrimType("Sphere", PrimType::eSphere);
    rememberPrimType("Capsule", PrimType::eCapsule);
    rememberPrimType("Cylinder", PrimType::eCylinder);
    rememberPrimType("Cone", PrimType::eCone);
    rememberPrimType("Plane", PrimType::ePlane);
    rememberPrimType("Mesh", PrimType::eMesh);
    auto primTypeName = [](PrimType primType) -> const char*
    {
        switch (primType)
        {
        case PrimType::eCube: return "Cube";
        case PrimType::eSphere: return "Sphere";
        case PrimType::eCapsule: return "Capsule";
        case PrimType::eCylinder: return "Cylinder";
        case PrimType::eCone: return "Cone";
        case PrimType::ePlane: return "Plane";
        case PrimType::eMesh: return "Mesh";
        }
        return nullptr;
    };
    auto primTypeIs = [&](ObjectKey key, PrimType primType) -> bool
    {
        const uint64_t canonical = src.canonicalPath(key);
        if (!canonical)
            return false;
        std::unordered_map<uint64_t, PrimType>::const_iterator it = primTypeByCanonical.find(canonical);
        if (it != primTypeByCanonical.end())
            return it->second == primType;
        const char* typeName = primTypeName(primType);
        if (!typeName)
            return false;
        ovx_token_t typeToken = OVX_INVALID_TOKEN;
        if (ovx_path_dictionary_intern_token(dict, ovxStr(typeName), &typeToken) == OVX_OK &&
            tokenColumnContains(instance, dict, src, key, conv::kUsdPrimType, typeToken, readOrdinal))
        {
            return true;
        }
        return isType(src, instance, key, typeName);
    };

    // Build and register one shape for a (collider, logical gprim) pair.
    // colliderKey owns collision state and gprimKey owns runtime identity,
    // transforms, and material. geometryKey is private ovstage backing used only
    // for prototype-authored type and geometry values.
    auto emitShape = [&](ObjectKey colliderKey, ObjectKey gprimKey) -> bool
    {
        if (isDeformableCollider(colliderKey))
            return true;
        const ObjectKey geometryKey = src.geometryBackingKey(gprimKey);

        ShapeInfo info;
        info.rigidBody = resolveBody(colliderKey);
        info.sourceGprim = gprimKey;
        src.getAttribute(colliderKey, tokCollisionEnabled, info.collisionEnabled);
        // info.simulationOwners left empty so the parsers do not drop the shape
        // on unresolved scenes (M2 single-scene scope).

        Matrix3d gprimRot;
        carb::Float3 gprimScale{ 1.0f, 1.0f, 1.0f };
        src.getLocalToWorldRotationAndScale(gprimKey, gprimRot, gprimScale);

        // Shape local pose. Mirror UsdGeomXformCache::ComputeRelativeTransform by
        // accumulating local transforms from the gprim up to (but excluding) its
        // body, stopping at a reset-xform-stack. The shared helper decomposes
        // that matrix and bakes the body scale; transform sourcing remains
        // ovstage-specific. A bodyless collider uses the gprim's world pose.
        if (info.rigidBody.valid())
        {
            const uint64_t bodyCanonical = src.canonicalPath(info.rigidBody);
            if (src.canonicalPath(gprimKey) != bodyCanonical)
            {
                PXR_NS::GfMatrix4d relativeTransform(1.0);
                ObjectKey current = gprimKey;
                for (int guard = 0; current.valid() && guard < 64; ++guard)
                {
                    if (src.canonicalPath(current) == bodyCanonical)
                        break;

                    Matrix4d localTransform;
                    bool resetsXformStack = false;
                    src.getLocalTransform(current, ReadTime::defaultTime(), localTransform, resetsXformStack);
                    relativeTransform *= toGfMatrix4d(localTransform);
                    if (resetsXformStack)
                        break;
                    current = src.getParent(current);
                }

                Matrix4d bodyWorldTransform;
                src.getLocalToWorldTransform(info.rigidBody, bodyWorldTransform);

                PXR_NS::GfVec3f localPos;
                PXR_NS::GfQuatf localRot;
                PXR_NS::GfVec3f localScale;
                omni::physics::decomposeCollisionShapeLocalTransform(
                    relativeTransform, toGfMatrix4d(bodyWorldTransform), localPos, localRot, localScale);
                const PXR_NS::GfVec3f localRotImaginary = localRot.GetImaginary();
                info.localPos = { localPos[0], localPos[1], localPos[2] };
                info.localRot = { localRotImaginary[0], localRotImaginary[1], localRotImaginary[2],
                                  localRot.GetReal() };
                info.localScale = { localScale[0], localScale[1], localScale[2] };
            }
        }
        else
        {
            Matrix4d wm;
            src.getLocalToWorldTransform(gprimKey, wm);
            info.localPos = { static_cast<float>(wm.data[12]), static_cast<float>(wm.data[13]),
                              static_cast<float>(wm.data[14]) };
            info.localRot = quatFromRotation(gprimRot);
            info.localScale = gprimScale;
        }

        DescPtr<PhysxShapeDesc> shape;
        bool commonFilled = true;
        if (primTypeIs(geometryKey, PrimType::eCube))
        {
            double sizeAttr = 1.0;
            src.getAttribute(geometryKey, tokSize, sizeAttr);
            const float half = static_cast<float>(std::abs(sizeAttr)) * 0.5f;
            shape = descPtrCast<PhysxShapeDesc>(parseBoxShape(
                ctx, colliderKey, info,
                { half * std::fabs(gprimScale.x), half * std::fabs(gprimScale.y), half * std::fabs(gprimScale.z) }));
        }
        else if (primTypeIs(geometryKey, PrimType::eSphere))
        {
            double radius = 1.0;
            src.getAttribute(geometryKey, tokRadius, radius);
            shape = descPtrCast<PhysxShapeDesc>(
                parseSphereShape(ctx, colliderKey, info,
                                 static_cast<float>(std::abs(radius)) *
                                     maxAbs3(gprimScale.x, gprimScale.y, gprimScale.z)));
        }
        else if (primTypeIs(geometryKey, PrimType::eCapsule))
        {
            double radius = 0.5, height = 1.0;
            src.getAttribute(geometryKey, tokRadius, radius);
            src.getAttribute(geometryKey, tokHeight, height);
            float scaledRadius = static_cast<float>(radius);
            float scaledHalfHeight = static_cast<float>(height) * 0.5f;
            const Axis axis = readAxis(geometryKey);
            scaleRoundShape(axis, gprimScale, scaledRadius, scaledHalfHeight);
            shape = descPtrCast<PhysxShapeDesc>(parseCapsuleShape(
                ctx, colliderKey, info, std::fabs(scaledRadius), std::fabs(scaledHalfHeight), axis));
        }
        else if (primTypeIs(geometryKey, PrimType::eCylinder))
        {
            double radius = 1.0, height = 2.0;
            src.getAttribute(geometryKey, tokRadius, radius);
            src.getAttribute(geometryKey, tokHeight, height);
            float scaledRadius = static_cast<float>(radius);
            float scaledHalfHeight = static_cast<float>(height) * 0.5f;
            const Axis axis = readAxis(geometryKey);
            scaleRoundShape(axis, gprimScale, scaledRadius, scaledHalfHeight);
            shape = descPtrCast<PhysxShapeDesc>(parseCylinderShape(
                ctx, colliderKey, info, std::fabs(scaledRadius), std::fabs(scaledHalfHeight), axis));
        }
        else if (primTypeIs(geometryKey, PrimType::eCone))
        {
            double radius = 1.0, height = 2.0;
            src.getAttribute(geometryKey, tokRadius, radius);
            src.getAttribute(geometryKey, tokHeight, height);
            float scaledRadius = static_cast<float>(radius);
            float scaledHalfHeight = static_cast<float>(height) * 0.5f;
            const Axis axis = readAxis(geometryKey);
            scaleRoundShape(axis, gprimScale, scaledRadius, scaledHalfHeight);
            shape = descPtrCast<PhysxShapeDesc>(parseConeShape(
                ctx, colliderKey, info, std::fabs(scaledRadius), std::fabs(scaledHalfHeight), axis));
        }
        else if (primTypeIs(geometryKey, PrimType::ePlane))
        {
            shape = descPtrCast<PhysxShapeDesc>(parsePlaneShape(ctx, colliderKey, info, readAxis(geometryKey)));
        }
        else if (primTypeIs(geometryKey, PrimType::eMesh))
        {
            // Mesh collider: build the cooking-input descriptor (triangle / convex /
            // decomposition / sphere-fill / SDF per UsdPhysicsMeshCollisionAPI). The
            // mesh scale is the gprim's world scale; the actual geometry is read later
            // by the cooking service via IPhysicsSource::getMeshAttributes(gprimKey).
            Matrix3d meshRot;
            carb::Float3 meshScale{ 1.0f, 1.0f, 1.0f };
            src.getLocalToWorldRotationAndScale(gprimKey, meshRot, meshScale);
            const carb::Float3 signScale = scaleToSignScale(meshScale);
            bool doubleSided = false;
            src.getAttribute(geometryKey, tokDoubleSided, doubleSided);
            shape = buildMeshShapeDesc(out, ctx, src, colliderKey, gprimKey, info.rigidBody, meshScale, signScale,
                                       doubleSided);
            commonFilled = false;
        }
        else
        {
            return false; // convex / unhandled geometry — later slice
        }

        if (!shape)
            return false;
        if (!commonFilled && !fillCommonShape(ctx, colliderKey, info, *shape))
            return false;
        shape->primKey = colliderKey;
        shape->sourceGprim = gprimKey;

        // Read direct material relationships or the resolved instance-material
        // column through the source. Keep the lookup on the logical gprim so
        // per-instance overrides survive. Mirrors the native walker.
        const ObjectKey material = src.getMaterialBinding(gprimKey);
        if (material.valid())
            shape->sourceMaterials.push_back(material);

        // Per-shape collision filtering (PhysxFilteredPairsAPI on the collider).
        shape->sourceFilteredCollisions = parseFilteredPairs(ctx, colliderKey);

        if (info.rigidBody.valid())
        {
            auto it = bodyByKey.find(info.rigidBody);
            if (it != bodyByKey.end())
                it->second->sourceCollisions.insert(colliderKey);
        }
        out.shapes.push_back(std::move(shape));
        return true;
    };

    for (const ObjectKey key : shapeBucket.keys)
    {
        // A collider applied directly to an analytic gprim emits one shape. A
        // collider applied to a non-geometry prim (e.g. an Xform) parents its
        // geometry on descendant gprims — emit a shape per analytic child, with the
        // static body kept at the collider's path (mirrors the native walker's
        // Xform-collider descent). Child gprims carry no physics schema, so they are
        // only present when the rendering domain was populated too.
        if (emitShape(key, key))
            continue;

        bool emitted = false;
        src.forEachChild(key, [&](ObjectKey child)
        {
            if (emitShape(key, child))
                emitted = true;
        });
        if (!emitted)
            ++out.skippedShapes; // unsupported collider or no supported child geometry
    }
    src.clearBucket();

    // --- Joints (typed PhysicsJoint prims) ---
    // body0/body1 are resolved relationships (read back from ovpopulation).
    // Authored local frames are normalized, transformed from each relationship
    // target into its resolved body frame, and have the body's scale baked into
    // their translations. Joint drives and D6 per-axis data are populated below.
    struct JointType
    {
        const char* typeName;
        ObjectType type;
    };
    static const JointType kJointTypes[] = {
        { "PhysicsFixedJoint", eJointFixed },
        { "PhysicsRevoluteJoint", eJointRevolute },
        { "PhysicsPrismaticJoint", eJointPrismatic },
        { "PhysicsSphericalJoint", eJointSpherical },
        { "PhysicsDistanceJoint", eJointDistance },
        { "PhysicsJoint", eJointD6 },
        { "PhysxPhysicsGearJoint", eJointGear },
        { "PhysxPhysicsRackAndPinionJoint", eJointRackAndPinion },
    };

    const TokenId tokJointEnabled = src.internToken("physics:jointEnabled");
    const TokenId tokBreakForce = src.internToken("physics:breakForce");
    const TokenId tokBreakTorque = src.internToken("physics:breakTorque");
    const TokenId tokExcludeArt = src.internToken("physics:excludeFromArticulation");
    const TokenId tokBody0 = src.internToken("physics:body0");
    const TokenId tokBody1 = src.internToken("physics:body1");
    const TokenId tokLocalPos0 = src.internToken("physics:localPos0");
    const TokenId tokLocalRot0 = src.internToken("physics:localRot0");
    const TokenId tokLocalPos1 = src.internToken("physics:localPos1");
    const TokenId tokLocalRot1 = src.internToken("physics:localRot1");
    const TokenId tokJointAxis = src.internToken("physics:axis");
    const TokenId tokLowerLimit = src.internToken("physics:lowerLimit");
    const TokenId tokUpperLimit = src.internToken("physics:upperLimit");
    const TokenId tokCone0 = src.internToken("physics:coneAngle0Limit");
    const TokenId tokCone1 = src.internToken("physics:coneAngle1Limit");
    const TokenId tokMinDist = src.internToken("physics:minDistance");
    const TokenId tokMaxDist = src.internToken("physics:maxDistance");
    const TokenId tokDriveAcceleration = src.internToken("acceleration");

    auto firstTarget = [&](ObjectKey jointKey, TokenId rel) -> ObjectKey
    {
        std::vector<ObjectKey> t;
        src.getRelationshipTargets(jointKey, rel, t);
        return t.empty() ? ObjectKey{} : t.front();
    };
    // Resolve a relationship target to the nearest rigid-body ancestor. If none
    // exists, preserve the native walker's legacy fallback to the outermost
    // CollisionAPI ancestor. At each level the scan's body map is the fast path;
    // RigidBodyAPI membership is checked only after that level misses so a body
    // created in an earlier incremental ordinal still outranks a farther mapped
    // body. CollisionAPI membership is deferred to a second pass outside the
    // common rigid-body path.
    auto resolveBodyFromTarget = [&](ObjectKey relTarget) -> ObjectKey
    {
        ObjectKey k = relTarget;
        for (int guard = 0; k.valid() && guard < 64; ++guard)
        {
            const std::unordered_map<uint64_t, ObjectKey>::const_iterator it =
                bodyByCanonical.find(src.canonicalPath(k));
            if (it != bodyByCanonical.end())
                return it->second;
            if (src.hasSchema(k, tokRigidBodyAPI))
                return src.canonicalKey(k);
            k = src.getParent(k);
        }

        ObjectKey collisionBody;
        k = relTarget;
        for (int guard = 0; k.valid() && guard < 64; ++guard)
        {
            if (src.hasSchema(k, tokCollisionAPI))
                collisionBody = src.canonicalKey(k);
            k = src.getParent(k);
        }
        return collisionBody;
    };
    auto readJointAxis = [&](ObjectKey key) -> Axis
    {
        TokenId at{};
        if (src.getAttribute(key, tokJointAxis, at) && at.valid())
        {
            if (at == tokAxisY)
                return eY;
            if (at == tokAxisZ)
                return eZ;
        }
        return eX; // USD joint default axis is X
    };
    auto transformJointFrame = [&](ObjectKey relationshipTarget,
                                   ObjectKey body,
                                   carb::Float3& localPosition,
                                   carb::Float4& localOrientation) {
        PXR_NS::GfQuatf orientation(
            localOrientation.w, PXR_NS::GfVec3f(localOrientation.x, localOrientation.y, localOrientation.z));
        orientation.Normalize();
        if (!relationshipTarget.valid())
        {
            const PXR_NS::GfVec3f normalizedImaginary = orientation.GetImaginary();
            localOrientation = {
                normalizedImaginary[0], normalizedImaginary[1], normalizedImaginary[2], orientation.GetReal()
            };
            return;
        }

        Matrix4d relationshipWorld;
        src.getLocalToWorldTransform(relationshipTarget, relationshipWorld);

        const bool relationshipTargetsBody =
            body.valid() && src.canonicalPath(body) == src.canonicalPath(relationshipTarget);
        Matrix4d bodyWorld = relationshipWorld;
        if (!body.valid())
            bodyWorld = Matrix4d{};
        else if (!relationshipTargetsBody)
            src.getLocalToWorldTransform(body, bodyWorld);

        PXR_NS::GfVec3f position(localPosition.x, localPosition.y, localPosition.z);
        omni::physics::transformJointFrameToBody(
            toGfMatrix4d(relationshipWorld), toGfMatrix4d(bodyWorld), relationshipTargetsBody, position, orientation);

        const PXR_NS::GfVec3f imaginary = orientation.GetImaginary();
        localPosition = { position[0], position[1], position[2] };
        localOrientation = { imaginary[0], imaginary[1], imaginary[2], orientation.GetReal() };
    };

    struct ScannedJointEntry
    {
        PhysxJointDesc* desc = nullptr;
        bool jointEnabled = true;
        bool excludeFromArticulation = false;
        std::vector<std::pair<JointAxis, JointLimitInfo>> jointLimits;
    };
    using ScannedJointMap = std::unordered_map<ObjectKey, ScannedJointEntry, ObjectKey::Hash>;
    ScannedJointMap jointMap;
    std::vector<ObjectKey> jointOrder;

    for (const JointType& jt : kJointTypes)
    {
        Bucket jointBucket = enumeratePrimTypeFiltered(jt.typeName, "physics:localPos0", readOrdinal);
        const std::vector<std::string> jointDiscoveredAttrs = jointBucket.attrs;
        const size_t jointAttrCount = jointBucket.attrs.size();
        appendJointAttrs(jointBucket, jt.type);
        src.prefetchBucket(jointBucket.keys, jointDiscoveredAttrs);
        src.prefetchBucket(jointBucket.keys, appendedAttrs(jointBucket, jointAttrCount));
        for (const ObjectKey key : jointBucket.keys)
        {
            JointInfo info;
            info.type = jt.type;

            bool flag = true;
            if (src.getAttribute(key, tokJointEnabled, flag))
                info.jointEnabled = flag;
            flag = false;
            if (src.getAttribute(key, tokCollisionEnabled, flag))
                info.collisionEnabled = flag;
            // Default to "no break" (FLT_MAX), overwritten only if authored.
            info.breakForce = FLT_MAX;
            info.breakTorque = FLT_MAX;
            src.getAttribute(key, tokBreakForce, info.breakForce);
            src.getAttribute(key, tokBreakTorque, info.breakTorque);
            flag = false;
            if (src.getAttribute(key, tokExcludeArt, flag))
                info.excludeFromArticulation = flag;

            // Resolved body relationships → nearest enabling rigid body.
            const ObjectKey rel0 = firstTarget(key, tokBody0);
            const ObjectKey rel1 = firstTarget(key, tokBody1);
            const ObjectKey body0 = rel0.valid() ? resolveBodyFromTarget(rel0) : ObjectKey{};
            const ObjectKey body1 = rel1.valid() ? resolveBodyFromTarget(rel1) : ObjectKey{};
            info.rel0 = rel0;
            info.rel1 = rel1;
            info.body0 = body0;
            info.body1 = body1;

            // Authored local frames are relative to the relationship targets;
            // convert them to the resolved body frames after reading.
            src.getAttribute(key, tokLocalPos0, info.localPose0Position);
            src.getAttribute(key, tokLocalRot0, info.localPose0Orientation);
            src.getAttribute(key, tokLocalPos1, info.localPose1Position);
            src.getAttribute(key, tokLocalRot1, info.localPose1Orientation);
            transformJointFrame(rel0, body0, info.localPose0Position, info.localPose0Orientation);
            transformJointFrame(rel1, body1, info.localPose1Position, info.localPose1Orientation);

            // Per-type axis and single limit. D6 per-axis data and drives are
            // populated separately below.
            if (jt.type == eJointRevolute || jt.type == eJointPrismatic || jt.type == eJointSpherical)
                info.axis = readJointAxis(key);
            // A limit is active only when a bound is finite and inside the sentinel
            // range; USD's unlimited default (-inf/+inf, or the +-0.5e38 sentinel)
            // leaves it disabled. Mirrors NativeWalker — without the finite check an
            // unlimited revolute/prismatic joint feeds inf bounds to the articulation
            // joint's setLimitParams and PhysX rejects them.
            constexpr float kJointSentinelLimit = 0.5e38f;
            if (jt.type == eJointRevolute || jt.type == eJointPrismatic)
            {
                src.getAttribute(key, tokLowerLimit, info.limit.lower);
                src.getAttribute(key, tokUpperLimit, info.limit.upper);
                info.limit.enabled =
                    (std::isfinite(info.limit.lower) && info.limit.lower > -kJointSentinelLimit) ||
                    (std::isfinite(info.limit.upper) && info.limit.upper < kJointSentinelLimit);
            }
            else if (jt.type == eJointSpherical)
            {
                src.getAttribute(key, tokCone0, info.limit.lower);
                src.getAttribute(key, tokCone1, info.limit.upper);
                info.limit.enabled = std::isfinite(info.limit.lower) && std::isfinite(info.limit.upper) &&
                                     info.limit.lower >= 0.0f && info.limit.upper >= 0.0f;
            }
            else if (jt.type == eJointDistance)
            {
                float mn = -1.0f, mx = -1.0f;
                info.minEnabled = src.getAttribute(key, tokMinDist, mn) && mn >= 0.0f;
                info.maxEnabled = src.getAttribute(key, tokMaxDist, mx) && mx >= 0.0f;
                info.limit.lower = mn;
                info.limit.upper = mx;
            }
            else if (jt.type == eJointD6)
            {
                // D6 carries per-axis limits/drives as multi-apply UsdPhysicsLimitAPI:
                // <axis> / UsdPhysicsDriveAPI:<axis> instances (axis = transX/Y/Z,
                // rotX/Y/Z, distance). These axis names are fixed by the D6 schema,
                // so check the exact schema tokens directly.
                struct D6AxisSchema
                {
                    const char* instance;
                    JointAxis axis;
                };
                static const D6AxisSchema kD6AxisSchemas[] = {
                    { "transX", eTransX }, { "transY", eTransY }, { "transZ", eTransZ },
                    { "rotX", eRotX },     { "rotY", eRotY },     { "rotZ", eRotZ },
                    { "distance", eDistance },
                };
                for (const D6AxisSchema& axisSchema : kD6AxisSchemas)
                {
                    const std::string limitApi = std::string("PhysicsLimitAPI:") + axisSchema.instance;
                    if (!src.hasSchema(key, src.internToken(limitApi)))
                        continue;
                    const std::string pfx = std::string("limit:") + axisSchema.instance + ":physics:";
                    JointLimitInfo lim;
                    src.getAttribute(key, src.internToken(pfx + "low"), lim.lower);
                    src.getAttribute(key, src.internToken(pfx + "high"), lim.upper);
                    // A limit is active only for finite bounds inside the sentinel
                    // range; USD's unlimited (-inf/+inf or beyond +-0.5e38) leaves the
                    // axis FREE (enabled=false), matching the revolute/prismatic rule.
                    // Feeding inf to PxD6Joint's pyramid/limit setters is rejected.
                    lim.enabled =
                        (std::isfinite(lim.lower) && lim.lower > -kJointSentinelLimit) ||
                        (std::isfinite(lim.upper) && lim.upper < kJointSentinelLimit);
                    info.jointLimits.push_back({ axisSchema.axis, lim });
                }
                for (const D6AxisSchema& axisSchema : kD6AxisSchemas)
                {
                    const std::string driveApi = std::string("PhysicsDriveAPI:") + axisSchema.instance;
                    if (!src.hasSchema(key, src.internToken(driveApi)))
                        continue;
                    const std::string pfx = std::string("drive:") + axisSchema.instance + ":physics:";
                    JointDriveInfo drv;
                    drv.enabled = true;
                    drv.forceLimit = FLT_MAX;
                    src.getAttribute(key, src.internToken(pfx + "stiffness"), drv.stiffness);
                    src.getAttribute(key, src.internToken(pfx + "damping"), drv.damping);
                    src.getAttribute(key, src.internToken(pfx + "targetPosition"), drv.targetPosition);
                    src.getAttribute(key, src.internToken(pfx + "targetVelocity"), drv.targetVelocity);
                    src.getAttribute(key, src.internToken(pfx + "maxForce"), drv.forceLimit);
                    TokenId driveType{};
                    if (src.getAttribute(key, src.internToken(pfx + "type"), driveType) && driveType.valid())
                        drv.acceleration = (driveType == tokDriveAcceleration);
                    info.jointDrives.push_back({ axisSchema.axis, drv });
                }
            }

            // Revolute/prismatic single-axis drive (UsdPhysicsDriveAPI:<instance>).
            // The instance is fixed by joint type ("angular" for revolute,
            // "linear" for prismatic), so it is read directly without multi-apply
            // enumeration. The drive is active iff that DriveAPI instance is applied.
            if (jt.type == eJointRevolute || jt.type == eJointPrismatic)
            {
                const char* inst = (jt.type == eJointRevolute) ? "angular" : "linear";
                if (src.hasSchema(key, src.internToken(std::string("PhysicsDriveAPI:") + inst)))
                {
                    const std::string pfx = std::string("drive:") + inst + ":physics:";
                    info.drive.enabled = true;
                    info.drive.forceLimit = FLT_MAX;
                    src.getAttribute(key, src.internToken(pfx + "stiffness"), info.drive.stiffness);
                    src.getAttribute(key, src.internToken(pfx + "damping"), info.drive.damping);
                    src.getAttribute(key, src.internToken(pfx + "targetPosition"), info.drive.targetPosition);
                    src.getAttribute(key, src.internToken(pfx + "targetVelocity"), info.drive.targetVelocity);
                    src.getAttribute(key, src.internToken(pfx + "maxForce"), info.drive.forceLimit);
                    TokenId driveType{};
                    if (src.getAttribute(key, src.internToken(pfx + "type"), driveType) && driveType.valid())
                        info.drive.acceleration = (driveType == tokDriveAcceleration);
                }
            }

            DescPtr<PhysxJointDesc> joint = parseJoint(ctx, key, info);
            if (!joint)
                continue;

            auto bodyFromReferencedJoint = [&](ObjectKey referencedJoint) -> ObjectKey
            {
                ScannedJointMap::const_iterator refIt = jointMap.find(src.canonicalKey(referencedJoint));
                if (refIt == jointMap.end() || !refIt->second.desc)
                    return {};

                const PhysxJointDesc* refDesc = refIt->second.desc;
                return refDesc->body1.valid() ? refDesc->body1 : refDesc->body0;
            };

            // ovpopulation currently surfaces gear/rack custom-joint prims and their
            // cross-joint refs, but can omit their inherited PhysicsJoint body rels.
            // Derive the missing bodies from the referenced child joints so the
            // runtime second-pass gear/rack creation has valid actors.
            if (jt.type == eJointGear)
            {
                GearPhysxJointDesc* gear = static_cast<GearPhysxJointDesc*>(joint.get());
                if (!gear->body0.valid())
                    gear->body0 = bodyFromReferencedJoint(gear->hingePrimPath0);
                if (!gear->body1.valid())
                    gear->body1 = bodyFromReferencedJoint(gear->hingePrimPath1);
            }
            else if (jt.type == eJointRackAndPinion)
            {
                RackPhysxJointDesc* rack = static_cast<RackPhysxJointDesc*>(joint.get());
                if (!rack->body0.valid())
                    rack->body0 = bodyFromReferencedJoint(rack->hingePrimKey);
                if (!rack->body1.valid())
                    rack->body1 = bodyFromReferencedJoint(rack->prismaticPrimKey);
            }

            joint->jointPrimKey = key;

            ScannedJointEntry entry;
            entry.desc = joint.get();
            entry.jointEnabled = info.jointEnabled;
            entry.excludeFromArticulation = info.excludeFromArticulation;
            entry.jointLimits = info.jointLimits;
            const ObjectKey canonicalJointKey = src.canonicalKey(key);
            jointMap[canonicalJointKey] = std::move(entry);
            jointOrder.push_back(canonicalJointKey);

            out.joints.push_back(std::move(joint));
        }
        src.clearBucket();
    }

    if (!jointMap.empty())
    {
        JointTypeLookup jointTypeOf = [&src, &jointMap](ObjectKey key) -> ObjectType
        {
            const ObjectKey canonicalKey = src.canonicalKey(key);
            ScannedJointMap::const_iterator it = jointMap.find(canonicalKey);
            if (it == jointMap.end() || !it->second.desc)
                return eUndefined;
            return it->second.desc->type;
        };

        JointLimitLookup jointLimitOf = [&src, &jointMap](ObjectKey key, JointAxis axis) -> const JointLimitInfo*
        {
            const ObjectKey canonicalKey = src.canonicalKey(key);
            ScannedJointMap::const_iterator it = jointMap.find(canonicalKey);
            if (it == jointMap.end())
                return nullptr;
            for (const std::pair<JointAxis, JointLimitInfo>& limit : it->second.jointLimits)
                if (limit.first == axis)
                    return &limit.second;
            return nullptr;
        };

        for (const ObjectKey& jointKey : jointOrder)
        {
            ScannedJointMap::const_iterator it = jointMap.find(jointKey);
            if (it == jointMap.end() || !it->second.desc)
                continue;
            const ScannedJointEntry& jointEntry = it->second;

            MimicJointParseInfo info;
            info.jointType = jointEntry.desc->type;
            info.jointEnabled = jointEntry.jointEnabled;
            info.excludeFromArticulation = jointEntry.excludeFromArticulation;

            const bool physxMimicAllowed = info.jointType == eJointRevolute ||
                                           info.jointType == eJointPrismatic ||
                                           info.jointType == eJointD6;
            const bool newtonMimicAllowed = physxMimicAllowed ||
                                            info.jointType == eJointFixed ||
                                            info.jointType == eJointSpherical ||
                                            info.jointType == eJointDistance;
            if (physxMimicAllowed)
                parseMimicJoints(ctx, jointKey, info, jointTypeOf, jointLimitOf, out.mimicJoints);
            if (newtonMimicAllowed)
                parseNewtonMimicJoints(ctx, jointKey, info, jointTypeOf, out.mimicJoints);
        }

        for (const ObjectKey& jointKey : jointOrder)
        {
            ScannedJointMap::const_iterator it = jointMap.find(jointKey);
            if (it == jointMap.end() || !it->second.desc)
                continue;
            FixedTendonParseInfo info;
            info.jointType = it->second.desc->type;
            info.body0 = it->second.desc->body0;
            info.body1 = it->second.desc->body1;
            parseFixedTendons(ctx, jointKey, info, out.fixedTendonAxes, out.fixedTendons);
        }
    }

    // Spatial tendon attachments are arbitrary multi-apply schemas on articulation
    // links / rigid bodies. Use the cached usd-schemas index to check the base
    // schema families without probing possible instance names through ovstage.
    static const char* const kSpatialTendonBases[] = {
        "PhysxTendonAttachmentRootAPI",
        "PhysxTendonAttachmentLeafAPI",
        "PhysxTendonAttachmentAPI",
    };
    constexpr size_t kSpatialTendonBaseCount =
        sizeof(kSpatialTendonBases) / sizeof(kSpatialTendonBases[0]);
    std::vector<ObjectKey> spatialTendonKeys;
    for (const char* base : kSpatialTendonBases)
        src.collectMultiApplySchemaKeys(src.internToken(base), spatialTendonKeys);
    if (!spatialTendonKeys.empty())
    {
        for (const auto& bodyEntry : bodyByKey)
        {
            if (!bodyEntry.second)
                continue;
            SpatialTendonParseInfo info;
            info.linkWorldScale = bodyEntry.second->scale;
            parseSpatialTendons(ctx, bodyEntry.first, info, out.spatialTendonAttachments);
        }
    }

    // --- Collision groups (typed UsdPhysicsCollisionGroup prims) ---
    // filteredGroups (group↔group filtering) + the `colliders` collection
    // membership, both resolved through the source from ovpopulation-persisted
    // relationships (parseCollisionGroup → getRelationshipTargets / resolveCollection).
    Bucket cgBucket = enumeratePrimTypeFiltered("PhysicsCollisionGroup",
                                                "physics:invertFilteredGroups",
                                                readOrdinal);
    const std::vector<std::string> cgDiscoveredAttrs = cgBucket.attrs;
    const size_t cgAttrCount = cgBucket.attrs.size();
    appendCollisionGroupAttrs(cgBucket);
    src.prefetchBucket(cgBucket.keys, cgDiscoveredAttrs);
    src.prefetchBucket(cgBucket.keys, appendedAttrs(cgBucket, cgAttrCount));
    for (const ObjectKey key : cgBucket.keys)
    {
        CollisionGroupInfo info = parseCollisionGroup(ctx, key);
        DescPtr<CollisionGroupDesc> desc = allocateDesc<CollisionGroupDesc>(allocator);
        if (!desc)
            continue;
        desc->primKey = key;
        desc->sourceFilteredGroups = std::move(info.filteredGroups);
        desc->sourceMembers = std::move(info.members);
        out.collisionGroups.push_back(std::move(desc));
    }
    src.clearBucket();

    // --- Articulations (HAS_APPLIED_SCHEMA PhysicsArticulationRootAPI) ---
    // Collect the roots (config via parseArticulation) and hand them — with the
    // already-scanned bodies + joints — to the SHARED root-election / graph
    // algorithm (parse::buildArticulations); the native walker runs the same code.
    std::vector<ArticulationRootInput> artRoots;
    std::unordered_set<ObjectKey, ObjectKey::Hash> articulationRootKeys;
    Bucket artBucket = enumerateSchemaFiltered("PhysicsArticulationRootAPI",
                                               conv::kLocalTransform,
                                               readOrdinal);
    const std::vector<std::string> artDiscoveredAttrs = artBucket.attrs;
    const size_t artAttrCount = artBucket.attrs.size();
    appendArticulationAttrs(artBucket);
    prefetchTransformAncestors(src, artBucket.keys);
    prefetchTransformsForKeys(src, artBucket.keys);
    src.prefetchBucket(artBucket.keys, artDiscoveredAttrs);
    src.prefetchBucket(artBucket.keys, appendedAttrs(artBucket, artAttrCount));
    auto addArticulationRoot = [&](ObjectKey key)
    {
        if (!key.valid() || !keyPassesFilter(key) || isBelowPointInstancer(key))
            return;
        const ObjectKey canonical = src.canonicalKey(key);
        const ObjectKey dedupeKey = canonical.valid() ? canonical : key;
        if (!articulationRootKeys.insert(dedupeKey).second)
            return;

        ArticulationRootInput in;
        in.key = key;
        setToDefault(in.fields, units);
        parseArticulation(ctx, key, in.fields);
        in.sourceFilteredCollisions = parseFilteredPairs(ctx, key);
        artRoots.push_back(std::move(in));
    };
    for (const ObjectKey key : artBucket.keys)
        addArticulationRoot(key);

    src.clearBucket();

    buildArticulations(src, allocator, artRoots, out.bodies, out.joints, out.articulations);

    auto scanBucket = [&](const Bucket& bucket, auto emit)
    {
        src.prefetchBucket(bucket.keys, bucket.attrs);
        for (const ObjectKey key : bucket.keys)
            emit(key);
        src.clearBucket();
    };
    auto scanBucketWithTransform = [&](const Bucket& bucket, auto emit)
    {
        Bucket withTransforms = bucket;
        appendTransformAttrs(withTransforms);
        prefetchTransformAncestors(src, withTransforms.keys);
        prefetchTransformsForKeys(src, withTransforms.keys);
        src.prefetchBucket(bucket.keys, bucket.attrs);
        for (const ObjectKey key : withTransforms.keys)
            emit(key);
        src.clearBucket();
    };

    // --- Particles ---
    Bucket particleSystemBucket = enumeratePrimTypeFiltered("PhysxParticleSystem",
                                                           "particleSystemEnabled",
                                                           readOrdinal);
    appendBucketAttr(particleSystemBucket, "physics:simulationOwner");
    appendBucketAttr(particleSystemBucket, "physics:filteredPairs");
    scanBucket(particleSystemBucket, [&](ObjectKey key) { emitParticleSystem(out, ctx, key); });

    Bucket particleSetBucket = enumerateSchemaFiltered("PhysxParticleSetAPI",
                                                     "physxParticle:particleSystem", readOrdinal);
    appendBucketAttr(particleSetBucket, "physxParticle:particleSystem");
    scanBucket(particleSetBucket, [&](ObjectKey key) { emitParticleSet(out, ctx, key); });

    Bucket particleSamplerBucket = enumerateSchemaFiltered("PhysxParticleSamplingAPI",
                                                          "physxParticleSampling:samplingDistance", readOrdinal);
    appendBucketAttr(particleSamplerBucket, "physxParticleSampling:particles");
    scanBucket(particleSamplerBucket, [&](ObjectKey key) { emitParticleSampler(out, ctx, key); });

    // --- Deformable attachments and element collision filters ---
    static const char* const attachmentTypeNames[] = {
        "OmniPhysicsVtxVtxAttachment", "OmniPhysicsVtxTriAttachment",
        "OmniPhysicsVtxTetAttachment", "OmniPhysicsVtxCrvAttachment",
        "OmniPhysicsVtxXformAttachment", "OmniPhysicsTetXformAttachment",
        "OmniPhysicsTriTriAttachment",
    };
    for (const char* typeName : attachmentTypeNames)
    {
        const Bucket attachmentBucket = enumeratePrimTypeFiltered(typeName,
                                                                  "omniphysics:attachmentEnabled",
                                                                  readOrdinal);
        const ObjectType subtype = attachmentSubtypeForPrimType(typeName);
        scanBucket(attachmentBucket, [&](ObjectKey key) { emitAttachment(out, ctx, key, subtype); });
    }

    const Bucket filterBucket = enumeratePrimTypeFiltered("OmniPhysicsElementCollisionFilter",
                                                          "omniphysics:filterEnabled",
                                                          readOrdinal);
    scanBucket(filterBucket, [&](ObjectKey key) { emitElementCollisionFilter(out, ctx, key); });

    // --- Vehicles: scene context + tire friction tables ---
    const Bucket vehCtxBucket = enumerateSchemaFiltered("PhysxVehicleContextAPI",
                                                "physxVehicleContext:updateMode", readOrdinal);
    scanBucket(vehCtxBucket, [&](ObjectKey key) { emitVehicleContext(out, src, ctx, key); });

    const Bucket tftBucket = enumeratePrimTypeFiltered("PhysxVehicleTireFrictionTable",
                                                       "frictionValues",
                                                       readOrdinal);
    scanBucket(tftBucket, [&](ObjectKey key) { emitTireFrictionTable(out, src, ctx, key); });

    // --- Vehicles: shareable wheel/tire/suspension components ---
    const Bucket wheelBucket = enumerateSchemaFiltered("PhysxVehicleWheelAPI",
                                               "physxVehicleWheel:radius", readOrdinal);
    scanBucket(wheelBucket, [&](ObjectKey key) { emitVehicleWheel(out, ctx, key); });

    const Bucket tireBucket = enumerateSchemaFiltered("PhysxVehicleTireAPI",
                                              "physxVehicleTire:lateralStiffnessGraph", readOrdinal);
    scanBucket(tireBucket, [&](ObjectKey key) { emitVehicleTire(out, instance, src, ctx, key, massScale); });

    const Bucket suspensionBucket = enumerateSchemaFiltered("PhysxVehicleSuspensionAPI",
                                                    "physxVehicleSuspension:springStrength", readOrdinal);
    scanBucket(suspensionBucket, [&](ObjectKey key) { emitVehicleSuspension(out, ctx, key); });

    // --- Vehicles: drivetrain components ---
    const Bucket engineBucket = enumerateSchemaFiltered("PhysxVehicleEngineAPI",
                                                "physxVehicleEngine:peakTorque", readOrdinal);
    scanBucket(engineBucket, [&](ObjectKey key) { emitVehicleEngine(out, src, ctx, key, kgmsScale); });

    const Bucket gearsBucket = enumerateSchemaFiltered("PhysxVehicleGearsAPI",
                                               "physxVehicleGears:ratios", readOrdinal);
    scanBucket(gearsBucket, [&](ObjectKey key) { emitVehicleGears(out, src, ctx, key); });

    const Bucket clutchBucket = enumerateSchemaFiltered("PhysxVehicleClutchAPI",
                                                "physxVehicleClutch:strength", readOrdinal);
    scanBucket(clutchBucket, [&](ObjectKey key) { emitVehicleClutch(out, ctx, key, kgmsScale); });

    const Bucket driveBasicBucket = enumerateSchemaFiltered("PhysxVehicleDriveBasicAPI",
                                                    "physxVehicleDriveBasic:peakTorque", readOrdinal);
    scanBucket(driveBasicBucket, [&](ObjectKey key) { emitVehicleDriveBasic(out, ctx, key, kgmsScale); });

    const Bucket driveStandardBucket = enumerateSchemaFiltered("PhysxVehicleDriveStandardAPI",
                                                       "physxVehicleDriveStandard:engine", readOrdinal);
    scanBucket(driveStandardBucket, [&](ObjectKey key) { emitVehicleDriveStandard(out, src, ctx, key); });

    const Bucket tankDiffBucket = enumerateSchemaFiltered("PhysxVehicleTankDifferentialAPI",
                                                  "physxVehicleTankDifferential:numberOfWheelsPerTrack", readOrdinal);
    scanBucket(tankDiffBucket, [&](ObjectKey key) { emitVehicleTankDifferential(out, src, ctx, key); });

    const Bucket multiDiffBucket = enumerateSchemaFiltered("PhysxVehicleMultiWheelDifferentialAPI",
                                                   "physxVehicleMultiWheelDifferential:wheels", readOrdinal);
    const TokenId tokTankDiffApi = src.internToken("PhysxVehicleTankDifferentialAPI");
    scanBucket(multiDiffBucket, [&](ObjectKey key)
    {
        if (!src.hasSchema(key, tokTankDiffApi))
            emitVehicleMultiWheelDifferential(out, src, ctx, key);
    });

    const Bucket autoGearBucket = enumerateSchemaFiltered("PhysxVehicleAutoGearBoxAPI",
                                                  "physxVehicleAutoGearBox:upRatios", readOrdinal);
    scanBucket(autoGearBucket, [&](ObjectKey key) { emitVehicleAutoGearBox(out, src, ctx, key); });

    // --- Vehicles: brakes, steering, nonlinear response ---
    const char* const brakeSchemas[] = { "PhysxVehicleBrakesAPI:brakes0", "PhysxVehicleBrakesAPI:brakes1" };
    const Bucket brakesBucket = enumerateSchemasFiltered(brakeSchemas, 2,
                                                 "physxVehicleBrakes:brakes0:maxBrakeTorque", readOrdinal);
    scanBucket(brakesBucket, [&](ObjectKey key) { emitVehicleBrakes(out, src, ctx, key); });

    const Bucket steeringBucket = enumerateSchemaFiltered("PhysxVehicleSteeringAPI",
                                                  "physxVehicleSteering:wheels", readOrdinal);
    scanBucket(steeringBucket, [&](ObjectKey key) { emitVehicleSteeringBasic(out, src, ctx, key); });

    const Bucket ackermannBucket = enumerateSchemaFiltered("PhysxVehicleAckermannSteeringAPI",
                                                   "physxVehicleAckermannSteering:wheel0", readOrdinal);
    const TokenId tokSteeringApi = src.internToken("PhysxVehicleSteeringAPI");
    scanBucket(ackermannBucket, [&](ObjectKey key)
    {
        if (!src.hasSchema(key, tokSteeringApi))
            emitVehicleSteeringAckermann(out, ctx, key);
    });

    const char* const ncrSchemas[] = {
        "PhysxVehicleNonlinearCommandResponseAPI:drive",
        "PhysxVehicleNonlinearCommandResponseAPI:steer",
        "PhysxVehicleNonlinearCommandResponseAPI:brakes0",
        "PhysxVehicleNonlinearCommandResponseAPI:brakes1",
    };
    const Bucket ncrBucket = enumerateSchemasFiltered(ncrSchemas, 4,
                                              "physxVehicleNonlinearCommandResponse:drive:commandValues", readOrdinal);
    scanBucket(ncrBucket, [&](ObjectKey key) { emitVehicleNonlinearCmdResponse(out, src, ctx, key); });

    // --- Vehicles: wheel attachments, suspension compliance, chassis roots ---
    const Bucket wheelAttachmentBucket = enumerateSchemaFiltered("PhysxVehicleWheelAttachmentAPI",
                                                         "physxVehicleWheelAttachment:index", readOrdinal);
    scanBucket(wheelAttachmentBucket, [&](ObjectKey key) { emitVehicleWheelAttachment(out, instance, src, ctx, key); });

    const Bucket suspensionComplianceBucket = enumerateSchemaFiltered("PhysxVehicleSuspensionComplianceAPI",
                                                              "physxVehicleSuspensionCompliance:suspensionForceAppPoint",
                                                              readOrdinal);
    scanBucket(suspensionComplianceBucket, [&](ObjectKey key) { emitVehicleSuspensionCompliance(out, src, ctx, key); });

    const Bucket vehicleBucket = enumerateSchemaFiltered("PhysxVehicleAPI",
                                                 "physxVehicle:vehicleEnabled", readOrdinal);
    scanBucketWithTransform(vehicleBucket, [&](ObjectKey key) { emitVehicle(out, src, ctx, key, lengthScale); });

    return out;
}

} // namespace omni::physics::ovstage
