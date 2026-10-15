// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-MASS-004
 * @covers AC-1 AC-4
 */

/**
 * @implements REQ-PARSE-COL-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-PARSE-COL-003
 * @covers AC-1 AC-3
 *
 * @implements REQ-PARSE-SHAPE-006
 * @covers AC-1 AC-3
 *
 * @implements REQ-PARSE-COL-005
 * @covers AC-1 AC-5
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-5
 */

/**
 * @implements REQ-PARSE-INSTANCER-DISPATCH-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-NEWTON-JOINT-001
 * @covers AC-1 AC-3
 *
 * @implements REQ-PARSE-BODY-003
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-1 AC-14 AC-15 AC-16 AC-17 AC-18 AC-20
 *
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-24
 *
 * @implements REQ-PARSE-CCT-001
 * @covers AC-5
 *
 * @implements REQ-PARSE-MAT-001
 * @covers AC-6
 *
 * @implements REQ-PUBLICAPI-003
 * @covers AC-4
 *
 * @implements REQ-PARSE-SHAPE-004
 * @covers AC-1
 *
 * @implements REQ-PARSE-JOINT-005
 * @covers AC-3 AC-4
 *
 * @implements REQ-PARSE-FEED-003
 * @covers AC-10 AC-12 AC-13 AC-14 AC-17
 */

#include "OvstageWalker.h"

#include "OvstageSource.h"
#include "OvstageShapeBatch.h"
#include "OvstageJointBatch.h"
#include "OvstageVehicleBatch.h"
#include "DescriptorMemoryStats.h"
#include "OvstageDeformableBatch.h"



#include <carb/extras/ScopeExit.h>
#include <carb/logging/Log.h>

#include <omni/physics/parse/ArticulationGraph.h>
#include <omni/physics/parse/CustomTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedBody.h>
#include <omni/physics/parse/PreparedParticle.h>
#include <omni/physics/parse/PreparedCct.h>
#include <omni/physics/parse/PreparedAttachment.h>
#include <omni/physics/parse/PreparedJoint.h>
#include <omni/physics/parse/PreparedTendon.h>
#include <omni/physics/parse/PreparedMimicJoint.h>
#include <omni/physics/parse/PreparedMaterial.h>
#include <omni/physics/parse/PreparedScene.h>
#include <omni/physics/parse/ParseContext.h>
#include <private/omni/physics/CollisionShapeTransform.h>
#include <private/omni/physics/JointFrameTransform.h>

#include <foundation/PxMat44.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <limits>
#include <stdexcept>
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

// Test-only count of enumerate()'s data-column reads: a family with no matching prim must not
// pay one to materialise zero prims.
std::atomic_size_t& enumerateReadCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only count of enumerate()'s whole-stage filter queries: a prim-type family the source's
// type index knows to be empty must not pay one.
std::atomic_size_t& enumerateQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only: the next N batched probe reads in enumerate() fail, so the per-column fallback runs.
// Zero in production; set only by setOvstageWalkerEnumerateReadFaultForTest().
std::atomic_int& enumerateReadFaultCounter()
{
    static std::atomic_int counter{ 0 };
    return counter;
}

bool consumeEnumerateReadFault()
{
    int n = enumerateReadFaultCounter().load(std::memory_order_relaxed);
    while (n > 0)
    {
        if (enumerateReadFaultCounter().compare_exchange_weak(n, n - 1, std::memory_order_relaxed))
            return true;
    }
    return false;
}

// parse::Matrix4d is sixteen doubles in the layout PxMat44d/GfMatrix4d both use;
// this is an element copy, not a transpose. Only the *product* order differs,
// see common/foundation/MatrixTools.h.
::physx::PxMat44d toPxMat44d(const Matrix4d& matrix)
{
    double values[16];
    for (int i = 0; i < 16; ++i)
        values[i] = matrix.data[i];
    return ::physx::PxMat44d(values);
}

// This local helper reports completion for strict schema reads; OvstageChangeFeed keeps its lenient copy.
bool waitAndRelease(ovstage_instance_t* inst, ovstage_enqueue_result_t result) noexcept
{
    if (result.status != OVSTAGE_OK || result.op_index == OVSTAGE_INVALID_OP_ID)
        return false;

    const ovstage_api_status_t waitStatus =
        ovstage_wait_op(inst, result.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr);
    if (waitStatus == OVSTAGE_OK || waitStatus == OVSTAGE_ERROR_OP_FAILED)
        (void)ovstage_release_op(inst, result.op_index);
    return waitStatus == OVSTAGE_OK;
}

[[noreturn]] void throwDataError(ovstage_api_status_t status,
                                 const char* operation,
                                 ovstage_ordinal_t readOrdinal)
{
    throw std::runtime_error("OVStage schema scan failed during " + std::string(operation) +
                             " at read ordinal " + std::to_string(readOrdinal) +
                             " (status " + std::to_string(static_cast<int>(status)) + ")");
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

void prefetchTransformAncestors(OvstageSource& src, const std::vector<ObjectKey>& keys)
{
    Bucket bucket;
    bucket.keys = src.collectAncestors(keys);
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

void appendDeformableBodyAttrs(Bucket& bucket)
{
    if (bucket.keys.empty())
        return;
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
    if (bucket.keys.empty())
        return;
    appendBucketAttrs(bucket, {
        "physics:filteredGroups",
        "physics:invertFilteredGroups",
        "collection:colliders:includes",
        "collection:colliders:excludes",
    });
}

void appendArticulationAttrs(Bucket& bucket)
{
    if (bucket.keys.empty())
        return;
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

// Enumerate the ObjectKeys of prims matching a single-predicate filter, plus the
// data attributes discovered on the matched set. `probeAttr` must be an attribute
// every matched prim carries: the read group's prims.list materialises the
// matched set (ovstage gives no path list on the query result itself).
Bucket enumerate(const OvstageSource& src,
                 ovstage_instance_t* inst,
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
    const bool requiresSchemaProbe = std::string_view(predAttr) == conv::kUsdSchemas;

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
    CARB_SCOPE_EXIT
    {
        if (q != OVSTAGE_INVALID_QUERY_HANDLE)
            waitAndRelease(inst, ovstage_release_query(inst, q));
    };
    enumerateQueryCounter().fetch_add(1, std::memory_order_relaxed);
    const ovstage_enqueue_result_t qe = ovstage_query(inst, &filter, nullptr, 0, &q);
    if (!waitAndRelease(inst, qe) || q == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        if (requiresSchemaProbe)
            throwDataError(qe.status == OVSTAGE_OK ? OVSTAGE_ERROR_OP_FAILED : qe.status,
                           "ovstage_query", readOrdinal);
        return out;
    }

    // ovstage gives no prim path list on the query result itself — the matched
    // prims materialise only as a read group's prims.list. Rather than assume a
    // specific probe attribute exists (different data models author different
    // attributes — the hand populator vs ovpopulation), read the query result's
    // own *discovered* attributes: those are exactly the attributes present on the
    // matched set, so reading them back yields every matched prim. `probeAttr` is
    // a fallback when the result reports no attributes. Schema membership queries
    // are not ordinal-scoped, so `usd-schemas` itself is the required probe for a
    // schema predicate; reading an ordinary column would not prove that the schema
    // data is readable at `readOrdinal`.
    ovstage_query_handle_t use = q;
    std::vector<ovx_token_t> probes; // all usable data columns; their union covers the matched set
    ovstage_query_result_t qr{};
    const ovstage_api_status_t queryResultStatus =
        ovstage_fetch_query_result(inst, q, OVSTAGE_TIMEOUT_INFINITE, &qr);
    if (queryResultStatus != OVSTAGE_OK && requiresSchemaProbe)
        throwDataError(queryResultStatus, "ovstage_fetch_query_result", readOrdinal);
    bool noMatch = false;
    if (queryResultStatus == OVSTAGE_OK)
    {
        noMatch = qr.total_prim_count == 0;
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
            if (!requiresSchemaProbe)
                probes.push_back(qr.attributes[i]); // each usable column probes + bulk-reads
            out.attrs.emplace_back(s.ptr, s.length);
        }
        (void)ovstage_release_query_result(inst, &qr);
    }
    // The query's own count decides an empty family: no fallback read to materialise zero
    // prims. Schema predicates keep their read, which doubles as the readability proof.
    if (noMatch && !requiresSchemaProbe)
        return out;
    if (requiresSchemaProbe || probes.empty())
    {
        // Schema predicates must read their metadata column. Other predicates
        // use the explicit metadata/probe column only when no ordinary data
        // column can materialize the matched prims.
        ovx_token_t fb = OVX_INVALID_TOKEN;
        const char* fallbackProbe = metadataProbeAttr ? metadataProbeAttr : probeAttr;
        if (ovx_path_dictionary_intern_token(dict, ovxStr(fallbackProbe), &fb) == OVX_OK &&
            fb != OVX_INVALID_TOKEN)
            probes.push_back(fb);
        else if (requiresSchemaProbe)
            throw std::runtime_error("OVStage schema scan could not intern required usd-schemas token");
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = readOrdinal;
    range.has_start_ordinal = false;

    // Read EVERY usable column, in ONE multi-attribute read, and union the matched prim
    // lists. A single column is only authored on the prims that carry it, so one probe
    // under-counts the matched set: a read group's prims.list holds only the prims with that
    // column, dropping matched prims whose authored column was not the first one
    // picked (e.g. Sphere/Mesh/Cube colliders, particle systems). The discovered
    // columns are the union over the matched set, so every matched prim carries at
    // least one — reading them all and unioning (dedup by handle) yields the
    // complete set, which is what let us drop the USD-Traverse supplement in
    // OvstageSource::collectPrimTypeKeys. Handles come from the data-column read
    // (the populator's canonical primpaths) so downstream key lookups still match. The
    // groups come back tagged per column; only their prim lists matter here, so one round
    // trip serves every column. Should that batched read fail, the columns are read one by
    // one instead, so a single failing column drops only its own rows rather than emptying the
    // family (a schema predicate's failing column is still an error, as before).
    std::unordered_set<uint64_t> seen;
    // One read over `toks`, its groups' prims unioned into `keys`. False when the read itself
    // could not be issued (the batched caller then falls back per column).
    auto readProbeUnion = [&](const ovx_token_t* toks, size_t tokCount, bool batched) -> bool
    {
        if (batched && consumeEnumerateReadFault())
            return false;
        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        CARB_SCOPE_EXIT
        {
            if (rh != OVSTAGE_INVALID_READ_HANDLE)
                waitAndRelease(inst, ovstage_release_read(inst, rh));
        };
        enumerateReadCounter().fetch_add(1, std::memory_order_relaxed);
        const ovstage_enqueue_result_t re = ovstage_read_attributes(inst, use, toks, tokCount, range, &rh);
        if (!waitAndRelease(inst, re) || rh == OVSTAGE_INVALID_READ_HANDLE)
        {
            if (!batched && requiresSchemaProbe)
                throwDataError(re.status == OVSTAGE_OK ? OVSTAGE_ERROR_OP_FAILED : re.status,
                               "ovstage_read_attributes", readOrdinal);
            return false;
        }
        ovstage_read_group_t g{};
        // Union prims from EVERY read group, not just the first. A scalar column
        // comes back as a single group whose prims.list covers all owning prims,
        // but an array/relationship column (e.g. `collection:colliders:includes`,
        // `physics:filteredGroups`) is emitted one group PER owning prim — so a
        // first-group-only scan captured just one owner and dropped the rest (a
        // second collision group with only a ragged membership column went missing).
        // Dedup via `seen` keeps scalar reads idempotent.
        ovstage_api_status_t fetchStatus = OVSTAGE_OK;
        // A list is fetched and unioned once, however many groups (one per owning prim on an array
        // column) address it.
        ReadListMemo listMemo(dict);
        std::unordered_set<ovx_primpath_list_t> unioned;
        while ((fetchStatus = ovstage_fetch_read_next(inst, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
        {
            const ovx_primpath_t* paths = nullptr;
            size_t count = 0;
            if (listMemo.paths(g.prims.list, &paths, &count) && unioned.insert(g.prims.list).second)
                for (size_t i = 0; i < count; ++i)
                    if (seen.insert(paths[i]).second)
                        keys.push_back(src.internKey(paths[i]));
            ovstage_release_group(inst, &g);
        }
        if (fetchStatus != OVSTAGE_ERROR_END_OF_ITERATION && requiresSchemaProbe)
            throwDataError(fetchStatus, "ovstage_fetch_read_next", readOrdinal);
        return true;
    };
    if (!probes.empty() && !readProbeUnion(probes.data(), probes.size(), /*batched=*/true))
    {
        for (const ovx_token_t probe : probes)
            readProbeUnion(&probe, 1, /*batched=*/false);
    }
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

// Use the source's composed type, including geometry behind renderer proxies.
bool isType(const OvstageSource& src, ovstage_instance_t*, ObjectKey key, const char* typeName)
{
    return src.tokenToString(src.getTypeName(key)) == typeName;
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
    // key.handle is the packed, generation-tagged ObjectKey (ADR-0021), never
    // a valid ovx_primpath_t for the C API below -- rawHandle() is the true
    // raw-handle fallback when canonicalization fails.
    ovx_primpath_t path = canonical ? canonical : src.rawHandle(key);
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

// `src` provides the canonical identity used to dedup: `dst`/`srcKeys` may mix
// keys minted through different routes for the same logical prim (schema-cache
// keys are always canonical, but a raw enumerate()-sourced key is not
// guaranteed to pack to the same value as its canonical alias -- ADR-0021), so
// a raw ObjectKey.handle comparison alone is not a reliable identity test.
void appendUnique(const OvstageSource& src, std::vector<ObjectKey>& dst, const std::vector<ObjectKey>& srcKeys)
{
    for (const ObjectKey key : srcKeys)
    {
        const uint64_t canonical = src.canonicalPath(key);
        const uint64_t identity = canonical ? canonical : key.handle;
        const auto it = std::find_if(dst.begin(), dst.end(), [&](ObjectKey existing)
        {
            const uint64_t existingCanonical = src.canonicalPath(existing);
            return (existingCanonical ? existingCanonical : existing.handle) == identity;
        });
        if (it == dst.end())
            dst.push_back(key);
    }
}

Bucket enumerateSchema(const OvstageSource& src,
                       ovstage_instance_t* inst,
                       ovx_path_dictionary_t* dict,
                       const char* schemaName,
                       const char* probeAttr,
                       ovstage_ordinal_t readOrdinal)
{
    return enumerate(src, inst, dict, "usd-schemas", OVSTAGE_FILTER_OP_CONTAINS,
                     schemaName, probeAttr, readOrdinal, conv::kUsdSchemas);
}

Bucket enumerateSchemas(const OvstageSource& src,
                        ovstage_instance_t* inst,
                        ovx_path_dictionary_t* dict,
                        const char* const* schemaNames,
                        size_t schemaNameCount,
                        const char* probeAttr,
                        ovstage_ordinal_t readOrdinal)
{
    Bucket out;
    for (size_t i = 0; i < schemaNameCount; ++i)
    {
        Bucket one = enumerateSchema(src, inst, dict, schemaNames[i], probeAttr, readOrdinal);
        appendUnique(src, out.keys, one.keys);
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

void prepareMeshShape(ParseContext& ctx, const IPhysicsSource& src,
                      ObjectKey shapeKey, ObjectKey gprimKey,
                      const PhysxRigidBodyDesc* body, PreparedOvstageShape& input)
{
    MeshApproximation approximation = parseMeshApproximation(ctx, shapeKey);
    if ((approximation == MeshApproximation::eNone || approximation == MeshApproximation::eMeshSimplification) &&
        body && body->type == eDynamicBody && !static_cast<const DynamicPhysxRigidBodyDesc&>(*body).kinematicBody)
        approximation = MeshApproximation::eConvexHull;
    switch (approximation)
    {
    case MeshApproximation::eConvexHull:
        input.type = eConvexMeshShape;
        input.cooking = PreparedConvexHullCookingExt{};
        std::get<PreparedConvexHullCookingExt>(input.cooking).physxConvexHullCollisionAPI =
            src.hasSchema(shapeKey, ctx.knownTokens().physxConvexHullCollisionAPI);
        break;
    case MeshApproximation::eConvexDecomposition:
        input.type = eConvexMeshDecompositionShape;
        input.cooking = PreparedConvexDecompositionCookingExt{};
        break;
    case MeshApproximation::eSphereFill:
        input.type = eSpherePointsShape;
        input.cooking = PreparedSphereFillCookingExt{};
        std::get<PreparedSphereFillCookingExt>(input.cooking).fillModeFlood = ctx.knownTokens().fillModeFlood;
        std::get<PreparedSphereFillCookingExt>(input.cooking).fillModeRaycast = ctx.knownTokens().fillModeRaycast;
        std::get<PreparedSphereFillCookingExt>(input.cooking).fillModeSurface = ctx.knownTokens().fillModeSurface;
        break;
    case MeshApproximation::eBoundingSphere:
    case MeshApproximation::eBoundingCube:
    {
        input.type = approximation == MeshApproximation::eBoundingSphere ? eBoundingSphereShape : eBoundingBoxShape;
        const MeshGeometry geometry = parseMeshGeometry(ctx, gprimKey);
        input.points = ctx.getBuffer<carb::Float3>(geometry.points);
        break;
    }
    case MeshApproximation::eMeshSimplification:
        input.type = eTriangleMeshShape;
        input.cooking = PreparedTriangleMeshSimplificationCookingExt{};
        break;
    case MeshApproximation::eSdf:
        input.sdf = true;
        input.sdfCooking.physxSDFMeshCollisionAPI = src.hasSchema(shapeKey, ctx.knownTokens().physxSDFMeshCollisionAPI);
        input.sdfCooking.bitsPerPixel8 = ctx.knownTokens().bitsPerPixel8;
        input.sdfCooking.bitsPerPixel16 = ctx.knownTokens().bitsPerPixel16;
        input.sdfCooking.bitsPerPixel32 = ctx.knownTokens().bitsPerPixel32;
        [[fallthrough]];
    default:
        input.type = eTriangleMeshShape;
        input.cooking = PreparedTriangleMeshCookingExt{};
        break;
    }
}

} // namespace

// Not declared in the public header; test translation units forward-declare these.
void resetOvstageWalkerEnumerateReadCountForTest()
{
    enumerateReadCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageWalkerEnumerateReadCountForTest()
{
    return enumerateReadCounter().load(std::memory_order_relaxed);
}

void resetOvstageWalkerEnumerateQueryCountForTest()
{
    enumerateQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageWalkerEnumerateQueryCountForTest()
{
    return enumerateQueryCounter().load(std::memory_order_relaxed);
}

void setOvstageWalkerEnumerateReadFaultForTest(int count)
{
    enumerateReadFaultCounter().store(count, std::memory_order_relaxed);
}

int getOvstageWalkerEnumerateReadFaultForTest()
{
    return enumerateReadFaultCounter().load(std::memory_order_relaxed);
}

void emitTireFrictionTable(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
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

    out.tireFrictionTables.push_back(prepareTireFrictionTable(ctx, key, info));
}

void emitVehicleContext(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    if (!src.isA(key, src.internToken("PhysicsScene")))
        return;
    out.vehicleContexts.push_back(prepareVehicleContext(ctx, key));
}

void emitVehicleWheel(PreparedOvstageVehicles& out, ParseContext& ctx, ObjectKey key)
{
    out.vehicleWheels.push_back(prepareWheel(ctx, key));
}

void emitVehicleTire(PreparedOvstageVehicles& out, ovstage_instance_t* inst, OvstageSource& src,
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

    out.vehicleTires.push_back(prepareTire(ctx, key, info));
}

void emitVehicleSuspension(PreparedOvstageVehicles& out, ParseContext& ctx, ObjectKey key)
{
    out.vehicleSuspensions.push_back(prepareSuspension(ctx, key));
}

void emitVehicleEngine(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx,
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

    out.vehicleEngines.push_back(prepareEngine(ctx, key, info));
}

void emitVehicleGears(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    GearsInfo info;
    info.hasRatios = readFloatArray(src, key, "physxVehicleGears:ratios", info.ratios);

    out.vehicleGears.push_back(prepareGears(ctx, key, info));
}

void emitVehicleClutch(PreparedOvstageVehicles& out, ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    out.vehicleClutches.push_back(prepareClutch(ctx, key, kgmsScale));
}

void emitVehicleDriveBasic(PreparedOvstageVehicles& out, ParseContext& ctx, ObjectKey key, float kgmsScale)
{
    out.vehicleDrivesBasic.push_back(prepareDriveBasic(ctx, key, kgmsScale));
}

void emitVehicleDriveStandard(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DriveStandardInfo info;
    info.engineKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:engine", "PhysxVehicleEngineAPI");
    info.gearsKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:gears", "PhysxVehicleGearsAPI");
    info.autoGearBoxKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:autoGearBox",
                                          "PhysxVehicleAutoGearBoxAPI");
    info.clutchKey = resolveRelOrApi(src, key, "physxVehicleDriveStandard:clutch", "PhysxVehicleClutchAPI");

    out.vehicleDrivesStandard.push_back(prepareDriveStandard(ctx, key, info));
}

void readDifferentialInfo(OvstageSource& src, ObjectKey key, DifferentialInfo& info)
{
    readIntArray(src, key, "physxVehicleMultiWheelDifferential:wheels", info.wheels);
    info.hasTorqueRatios = readFloatArray(src, key, "physxVehicleMultiWheelDifferential:torqueRatios",
                                          info.torqueRatios);
    info.hasAverageWheelSpeedRatios = readFloatArray(
        src, key, "physxVehicleMultiWheelDifferential:averageWheelSpeedRatios", info.averageWheelSpeedRatios);
}

void emitVehicleMultiWheelDifferential(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    DifferentialInfo info;
    readDifferentialInfo(src, key, info);
    out.vehicleMultiWheelDifferentials.push_back(prepareMultiWheelDifferential(ctx, key, info));
}

void emitVehicleTankDifferential(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
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

    out.vehicleTankDifferentials.push_back(prepareTankDifferential(ctx, key, info, tankInfo));
}

void emitVehicleAutoGearBox(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    AutoGearBoxInfo info;
    info.hasUpRatios = readFloatArray(src, key, "physxVehicleAutoGearBox:upRatios", info.upRatios);
    info.hasDownRatios = readFloatArray(src, key, "physxVehicleAutoGearBox:downRatios", info.downRatios);

    out.vehicleAutoGearBoxes.push_back(prepareAutoGearBox(ctx, key, info));
}

void emitVehicleBrakes(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
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

        out.vehicleBrakes.push_back(prepareBrakes(ctx, key, instance, brakesIndex, info));
    });
}

void emitVehicleSteeringBasic(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    SteeringBasicInfo info;
    readIntArray(src, key, "physxVehicleSteering:wheels", info.wheels);
    info.hasAngleMultipliers = readFloatArray(src, key, "physxVehicleSteering:angleMultipliers",
                                              info.angleMultipliers);

    out.vehicleSteeringBasic.push_back(prepareSteeringBasic(ctx, key, info));
}

void emitVehicleSteeringAckermann(PreparedOvstageVehicles& out, ParseContext& ctx, ObjectKey key)
{
    out.vehicleSteeringAckermann.push_back(prepareSteeringAckermann(ctx, key));
}

void emitVehicleNonlinearCmdResponse(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
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

        out.vehicleNonlinearCmdResponses.push_back(prepareNonlinearCmdResponse(ctx, key, instance, info));
    });
}

void emitVehicleSuspensionCompliance(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    SuspensionComplianceInfo info;
    readFloat2Array(src, key, "physxVehicleSuspensionCompliance:wheelToeAngle", info.wheelToeAngles);
    readFloat2Array(src, key, "physxVehicleSuspensionCompliance:wheelCamberAngle", info.wheelCamberAngles);
    readFloat4Array(src, key, "physxVehicleSuspensionCompliance:suspensionForceAppPoint",
                    info.suspensionForceAppPoints);
    readFloat4Array(src, key, "physxVehicleSuspensionCompliance:tireForceAppPoint", info.tireForceAppPoints);

    out.vehicleSuspensionCompliances.push_back(prepareSuspensionCompliance(ctx, key, info));
}

void emitVehicleWheelAttachment(PreparedOvstageVehicles& out, ovstage_instance_t* inst, OvstageSource& src,
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

    out.vehicleWheelAttachments.push_back(prepareWheelAttachment(ctx, key, info));
}

// @implements REQ-PARSE-VEH-ROOT-001
// @covers AC-4
void emitVehicle(PreparedOvstageVehicles& out, OvstageSource& src, ParseContext& ctx,
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

    // Hosts include this metadata in their population descriptor. Preserve the
    // legacy COM default only when no opinion was populated.
    src.getAttribute(key, src.internToken("usd-metadata:customData:physxVehicle:referenceFrameIsCenterOfMass"),
                     info.referenceFrameIsCenterOfMass);

    out.vehicles.push_back(prepareVehicle(ctx, key, info));
}


void appendPreparedParticleSystems(OvstageScanResult& out, ParseContext& ctx, DescriptorBatchAllocator& batches,
                                   const DescriptorExecutionOptions& execution, const std::vector<ObjectKey>& keys)
{
        DescriptorPipelineStats* stats = execution.statistics;
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedParticleSystem> inputs;
        for (ObjectKey key : keys) inputs.push_back(prepareParticleSystem(ctx, key));
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<ParticleSystemDesc>> particleSystems = allocateDescriptorBatch<ParticleSystemDesc>(batches, inputs.size());
        for (size_t i = 0; i < inputs.size(); ++i) prepareParticleSystemStorage(inputs[i], *particleSystems[i]);
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) fillParticleSystem(inputs[i], *particleSystems[i]);
        });
        phase.transition(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedParticleAnisotropy> anisotropies;
        std::vector<PreparedParticleSmoothing> smoothings;
        std::vector<PreparedParticleIsosurface> isosurfaces;
        for (const DescPtr<ParticleSystemDesc>& system : particleSystems)
        {
            ctx.particleSystemCache()[system->systemKey] =
                { system->sceneKey, system->solidRestOffset, system->fluidRestOffset, system->particleContactOffset };
            PreparedParticleAnisotropy anisotropy = prepareParticleAnisotropy(ctx, system->systemKey);
            if (anisotropy.prepared) anisotropies.push_back(std::move(anisotropy));
            PreparedParticleSmoothing smoothing = prepareParticleSmoothing(ctx, system->systemKey);
            if (smoothing.prepared) smoothings.push_back(std::move(smoothing));
            PreparedParticleIsosurface isosurface = prepareParticleIsosurface(ctx, system->systemKey, system->fluidRestOffset);
            if (isosurface.prepared) isosurfaces.push_back(std::move(isosurface));
        }
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs) +
            descriptor_memory::nested(anisotropies) + descriptor_memory::nested(smoothings) + descriptor_memory::nested(isosurfaces));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<ParticleAnisotropyDesc>> particleAnisotropies = allocateDescriptorBatch<ParticleAnisotropyDesc>(batches, anisotropies.size());
        phase.finish();
        fillDescriptorRanges(anisotropies.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) fillParticleAnisotropy(anisotropies[i], *particleAnisotropies[i]);
        });
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<ParticleSmoothingDesc>> particleSmoothings = allocateDescriptorBatch<ParticleSmoothingDesc>(batches, smoothings.size());
        phase.finish();
        fillDescriptorRanges(smoothings.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) fillParticleSmoothing(smoothings[i], *particleSmoothings[i]);
        });
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<ParticleIsosurfaceDesc>> particleIsosurfaces = allocateDescriptorBatch<ParticleIsosurfaceDesc>(batches, isosurfaces.size());
        for (size_t i = 0; i < isosurfaces.size(); ++i) prepareParticleIsosurfaceStorage(isosurfaces[i], *particleIsosurfaces[i]);
        phase.finish();
        fillDescriptorRanges(isosurfaces.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) fillParticleIsosurface(isosurfaces[i], *particleIsosurfaces[i]);
        });
        phase.transition(stats ? &stats->finalizeSeconds : nullptr);
        for (DescPtr<ParticleSystemDesc>& desc : particleSystems) out.particleSystems.push_back(std::move(desc));
        for (DescPtr<ParticleAnisotropyDesc>& desc : particleAnisotropies) out.particleAnisotropies.push_back(std::move(desc));
        for (DescPtr<ParticleSmoothingDesc>& desc : particleSmoothings) out.particleSmoothings.push_back(std::move(desc));
        for (DescPtr<ParticleIsosurfaceDesc>& desc : particleIsosurfaces) out.particleIsosurfaces.push_back(std::move(desc));
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

// Character-controller emit — the ovstage peer of NativeWalker.cpp::emitCct.
// Requires the prim to be a Capsule gprim, reads its radius/height and world
// transform (translation + scale), and the simulationOwner relationship from
// PhysxCharacterControllerAPI. Per-axis scale baking matches the native walker
// exactly: radius scaled by scale.y, halfHeight by scale.z. parse-lib's
// source-agnostic parseCct resolves slopeLimit and the owner key.
void prepareOvstageCct(std::vector<PreparedCct>& inputs, OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    // A CCT on non-capsule geometry is dropped, as in the native walker. That one
    // logs "CCT prim must be a capsule geom"; this module links no logging backend
    // (there is no carb/logging dependency in omni.physics.ovstage), so the drop is
    // silent here.
    if (!src.isA(key, src.internToken("Capsule")))
        return;

    // UsdGeomCapsule schema fallbacks: radius 0.5, height 1.0 -- the same pair the
    // capsule SHAPE path below uses. `getAttribute` leaves `out` untouched on a miss,
    // so these initializers ARE the fallback whenever the source has no resident USD
    // stage to fall back through. The native walker gets away with initializing
    // radius to 1.0 because `GetRadiusAttr().Get()` on a typed capsule always writes
    // the schema fallback; here 1.0 would silently double an unauthored CCT's width.
    double radiusAttr = 0.5, heightAttr = 1.0;
    src.getAttribute(key, src.internToken("radius"), radiusAttr);
    src.getAttribute(key, src.internToken("height"), heightAttr);

    Matrix4d world;
    src.getLocalToWorldTransform(key, world);
    Matrix3d rotation;
    carb::Float3 scale{ 1.0f, 1.0f, 1.0f };
    src.getLocalToWorldRotationAndScale(key, rotation, scale);

    CctInfo info;
    info.radius = scale.y * static_cast<float>(radiusAttr);
    info.halfHeight = scale.z * static_cast<float>(heightAttr) * 0.5f;
    info.scale = scale;
    info.pos = { static_cast<float>(world.data[12]), static_cast<float>(world.data[13]),
                 static_cast<float>(world.data[14]) };
    src.getRelationshipTargets(key, src.internToken("physxCharacterController:simulationOwner"),
                               info.simulationOwners);

    inputs.push_back(prepareCct(ctx, key, info));
}

bool isPointBased(const OvstageSource& src, ObjectKey key)
{
    return src.isA(key, src.internToken("PointBased"));
}

// A tet sim mesh is identified by its TET DATA, not by its concrete prim type.
//
// ovstage's `usd-prim-type` column reports a UsdGeomTetMesh as plain "Mesh" — the
// populator has no TetMesh mapping — and isType() is an exact string match with no
// inheritance, so requiring "TetMesh" here rejected every volume deformable whose
// sim mesh came from ovstage. The surface branch only worked by luck: its required
// type IS the fallback the populator emits.
//
// Gating on the presence of readable tetVertexIndices is both weaker in the right
// way and stronger in practice: OmniPhysicsVolumeDeformableSimAPI already declares
// intent, and tet connectivity is what the consumer actually needs. USD still
// satisfies the fast path, since there the prim really is a TetMesh.
// Presence check only — resolve the buffer to prove it is readable, but never copy it. A tet
// mesh's connectivity is one of the largest arrays on the prim and this runs per candidate prim.
bool hasTetConnectivity(const OvstageSource& src, ObjectKey key)
{
    const BufferHandle h = src.readArrayAttribute(key, "tetVertexIndices", BufferElemType::eInt4, 4);
    if (!h.valid())
        return false;
    size_t byteCount = 0;
    const bool readable = src.resolveBuffer(h, byteCount) != nullptr;
    const bool nonEmpty = readable && h.elemCount > 0 && byteCount > 0;
    src.releaseBuffer(h);
    return nonEmpty;
}

bool isDeformableSimMesh(const OvstageSource& src, ovstage_instance_t* inst, ObjectKey key, ObjectType& outType)
{
    outType = eUndefined;
    if (src.hasSchema(key, src.internToken("OmniPhysicsVolumeDeformableSimAPI")))
    {
        if (!isType(src, inst, key, "TetMesh") && !hasTetConnectivity(src, key))
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

void prepareOvstageDeformable(std::vector<PreparedOvstageDeformable>& inputs, ovstage_instance_t* inst,
                              OvstageSource& src, ParseContext& ctx, ObjectKey key)
{
    const bool validGeomType = isType(src, inst, key, "Mesh") || isType(src, inst, key, "TetMesh") ||
                               isType(src, inst, key, "BasisCurves");
    const bool validRootType = isXformable(src, key) && !src.isA(key, src.internToken("Gprim"));
    if (!validGeomType && !validRootType)
        return;

    DeformableBodyParse parsed = parseDeformableBody(ctx, key);

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

    PreparedOvstageDeformable input;
    input.key = key;
    input.surface = simMeshType == eSurfaceDeformableBody;
    input.body = std::move(parsed);
    input.body.filteredCollisions.insert(input.body.filteredCollisions.end(), hierarchyFilteredPairs.begin(), hierarchyFilteredPairs.end());
    src.getLocalToWorldTransform(key, input.transform);
    input.simMesh = simMeshKey;
    input.simPose = findPoseNameFromPurpose(src, simMeshKey, "bindPose");
    TokenId orientation;
    input.simLeftHanded = src.getAttribute(simMeshKey, src.internToken("orientation"), orientation) && orientation == src.internToken("leftHanded");
    if (!collisionGeomKeys.empty())
    {
        assert(collisionGeomKeys.size() == collisionGeomBindPoseTokens.size());
        assert(collisionGeomKeys.size() == collisionGeomLeftHandedOrientations.size());
        input.collisionMesh = collisionGeomKeys.front();
        input.collisionPose = collisionGeomBindPoseTokens.front();
        input.collisionLeftHanded = collisionGeomLeftHandedOrientations.front();
    }
    input.skin = std::move(skinGeomKeys);
    input.skinPose = std::move(skinGeomBindPoseTokens);
    if (src.hasSchema(key, src.internToken("PhysxBaseDeformableBodyAPI")))
    {
        input.linearDamping = prepareAttribute<float>(src, key, src.internToken("physxDeformableBody:linearDamping"));
        input.maxLinearVelocity = prepareAttribute<float>(src, key, src.internToken("physxDeformableBody:maxLinearVelocity"));
        input.sleepThreshold = prepareAttribute<float>(src, key, src.internToken("physxDeformableBody:sleepThreshold"));
        input.settlingThreshold = prepareAttribute<float>(src, key, src.internToken("physxDeformableBody:settlingThreshold"));
        input.settlingDamping = prepareAttribute<float>(src, key, src.internToken("physxDeformableBody:settlingDamping"));
        input.maxDepenetrationVelocity = prepareAttribute<float>(src, key, src.internToken("physxDeformableBody:maxDepenetrationVelocity"));
        input.selfCollisionFilterDistance = prepareAttribute<float>(src, key, src.internToken("physxDeformableBody:selfCollisionFilterDistance"));
        input.solverPositionIterationCount = prepareAttribute<uint32_t>(src, key, src.internToken("physxDeformableBody:solverPositionIterationCount"));
        input.enableSpeculativeCCD = prepareAttribute<bool>(src, key, src.internToken("physxDeformableBody:enableSpeculativeCCD"));
        input.selfCollision = prepareAttribute<bool>(src, key, src.internToken("physxDeformableBody:selfCollision"));
        input.disableGravity = prepareAttribute<bool>(src, key, src.internToken("physxDeformableBody:disableGravity"));
    }
    if (input.surface && src.hasSchema(key, src.internToken("PhysxSurfaceDeformableBodyAPI")))
    {
        input.collisionPairUpdateFrequency = prepareAttribute<uint32_t>(src, key, src.internToken("physxDeformableBody:collisionPairUpdateFrequency"));
        input.collisionIterationMultiplier = prepareAttribute<uint32_t>(src, key, src.internToken("physxDeformableBody:collisionIterationMultiplier"));
    }
    if (src.hasSchema(key, src.internToken("PhysxAutoDeformableBodyAPI")))
    {
        input.hasAutoAPI = prepareAttribute<bool>(src, key, src.internToken("physxDeformableBody:autoDeformableBodyEnabled"));
        std::vector<ObjectKey> targets;
        src.getRelationshipTargets(key, src.internToken("physxDeformableBody:cookingSourceMesh"), targets);
        if (!targets.empty())
        {
            input.cookingMesh = targets.front();
            input.cookingPose = findPoseNameFromPurpose(src, targets.front(), "bindPose");
        }
        if (src.hasSchema(key, src.internToken("PhysxAutoDeformableMeshSimplificationAPI")))
        {
            input.isAutoMeshSimplificationEnabled = prepareAttribute<bool>(src, key, src.internToken("physxDeformableBody:autoDeformableMeshSimplificationEnabled"));
            input.isAutoRemeshingEnabled = prepareAttribute<bool>(src, key, src.internToken("physxDeformableBody:remeshingEnabled"));
            input.autoRemeshingResolution = prepareAttribute<uint32_t>(src, key, src.internToken("physxDeformableBody:remeshingResolution"));
            input.autoTriangleTargetCount = prepareAttribute<uint32_t>(src, key, src.internToken("physxDeformableBody:targetTriangleCount"));
            input.hasAutoForceConforming = prepareAttribute<bool>(src, key, src.internToken("physxDeformableBody:forceConforming"));
        }
        if (!input.surface && src.hasSchema(key, src.internToken("PhysxAutoDeformableHexahedralMeshAPI")))
        {
            input.hexahedral = true;
            input.hexahedralResolution = prepareAttribute<uint32_t>(src, key, src.internToken("physxDeformableBody:resolution"));
        }
    }
    input.prepared = true;
    inputs.push_back(std::move(input));
}

OvstageScanResult scanOvstage(ovstage_instance_t* instance,
                              ovx_path_dictionary_t* dict,
                              IDescriptorAllocator& allocator,
                              ovstage_ordinal_t readOrdinal,
                              const OvstageScanFilter* filter,
                              uint64_t usdStageId,
                              OvstageSource* attached)
{
    OvstageScanResult out;
    if (!instance || !dict)
        return out;

    out.descriptorPool = filter ? filter->descriptorPool : nullptr;
    if (!out.descriptorPool || &out.descriptorPool->upstream() != &allocator)
        out.descriptorPool = std::make_shared<DescriptorBatchAllocator>(allocator);
    DescriptorBatchAllocator& batches = *out.descriptorPool;
    const DescriptorExecutionOptions execution = filter ? filter->descriptorExecution : DescriptorExecutionOptions{};

    if (attached && attached->instance() != instance)
        attached = nullptr; // a foreign attach's source cannot mint keys for this instance
    std::unique_ptr<OvstageSource> owned;
    if (!attached)
        owned = std::make_unique<OvstageSource>(instance, dict, readOrdinal, usdStageId);
    OvstageSource& src = attached ? *attached : *owned;
    out.source = std::move(owned);
    out.borrowedSource = attached;
    // A consumer that opened a load-cache window on the attached source around its whole
    // incremental load (LoadStage::loadFromRange) shares it: the scan's prefetches then also
    // serve the consumer's follow-up reads, and the window is the consumer's to end.
    const bool ownWindow = !(attached && attached->loadCacheActive());
    if (ownWindow)
        src.beginLoadCache();
    // A borrowed source outlives the scan: drop the concept bucket with the load cache, or its
    // last prefetch would keep serving later reads. A caller that re-enters the scan with a
    // bucket of its own open (a drain's seeded read group, the mass update's prefetch, through
    // PointInstancer::parsePrototype or the collision-group scan) gets it back instead: the
    // scan's prefetches run on a suspended copy, mirroring the load-cache window join above.
    struct ScopedLoadCache
    {
        OvstageSource& source;
        bool endWindow;
        OvstageSource::BucketSnapshotPtr callerBucket;
        ~ScopedLoadCache()
        {
            if (endWindow)
                source.clearLoadCache();
            if (callerBucket)
                source.resumeBucket(std::move(callerBucket));
            else
                source.clearBucket();
        }
    } scopedLoadCache{ src, ownWindow, attached ? src.suspendBucket() : nullptr };

    DescriptorPipelineStats* stats = execution.statistics;
    if (stats) *stats = {}; // Options may be reused for several prototype scans.
    DescriptorPhaseTimer scanTimer(stats ? &stats->scanSeconds : nullptr);
    struct ReadTimings
    {
        double shapeSetup = 0, shapeClassify = 0, shapeTransforms = 0, shapeRelations = 0;
        double jointSetup = 0, jointTopology = 0, columns = 0;
    } readTimings;
    // Inclusive coordinator phase observations; none of these clocks run in workers.
    struct SetupTimings
    {
        double enumerate = 0, transformAncestors = 0, relationshipAncestors = 0;
        double transforms = 0, metadata = 0, attributes = 0, columns = 0, inputs = 0;
    } bodySetup, shapeSetup;
    struct JointTimings
    {
        double enumerate = 0, probe = 0, prefetch = 0, targets = 0, bodies = 0;
        double frames = 0, membership = 0, structure = 0, declarations = 0;
    } jointDetail;
    std::array<double, 4> shapeRelationSeconds{}; // Subsets, material binding, filters, owners.
    std::array<double, 10> waveSeconds{};
    DescriptorPhaseTimer waveTimer(stats ? &waveSeconds[0] : nullptr);
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
    // --- Deterministic enumeration order ---------------------------------------
    //
    // ovstage does NOT give a stage-derived enumeration order, from either of the
    // two routes a bucket's keys arrive by:
    //   - enumerate() unions the prims.list of every read group returned for a
    //     column, and ovstage emits those groups in an order that varies with
    //     process history. The same column with byte-identical group contents was
    //     observed coming back in three different orders depending on what had
    //     been attached earlier in the process.
    //   - collectSchemaKeys() serves from an unordered_set of prim handles, so its
    //     iteration order is hash order over handles that themselves migrate
    //     between id arenas as more ovstage instances are created.
    //
    // Scan order is load-bearing: Articulation.cpp::createArticulationLinks sorts
    // the joint vector by scan index ("required to maintain parsing order and
    // determinism") and createLinkHierarchy DFS-walks it, so scan order becomes
    // PhysX link and DOF indices. An unstable order therefore silently permutes
    // every tensor-view column for an identical asset — no error, matching counts,
    // only the meaning of the indices moves.
    //
    // Re-key every enumerated bucket to the source hierarchy's depth-first
    // traversal order. For valid absolute prim paths this is exactly one
    // decorate-sort by canonical path string: USD prim names are [A-Za-z0-9_]+ and
    // the '/' separator (0x2F) sorts below every legal name character, so
    // lexicographic full-path order IS preorder DFS with path-sorted siblings.
    // That guarantee needs no per-ancestor sibling query and no whole-stage walk,
    // and — unlike an ancestor-chain rank that routes each key to its immediate
    // parent's observed child set — it stays correct when an intermediate
    // hierarchy level has no queryable row of its own: a child's full path sorts
    // into position whether or not its typeless "/World"-style ancestors are
    // separately enumerable (e.g. "/World/a/body" before "/World/z" even when
    // "/World/a" is never authored as its own row).
    //
    // This makes ovstage agree with ITSELF across process histories. It does
    // NOT make it agree with the USD backend: ovstage exposes no authoring
    // order, and USD namespace order is authoring order, not path order.
    // Articulation link/DOF indices therefore still differ between the two
    // backends — closing that gap needs authoring order from ovstage.
    //
    // The sort key is the resolved canonical path string (never ObjectKey.handle
    // -- ADR-0021): ObjectKey packs a per-instance local index over whichever raw
    // handle happened to be interned first for a path, so two ObjectKeys naming
    // the same prim are not guaranteed to carry the same packed handle. The path
    // string is the one stable, alias-independent identity this ordering relies
    // on, and OvstageSource resolves it from an interned per-key cache (no ovstage
    // query, no stage walk). Keys whose path cannot be resolved (unparented /
    // private rows) have no namespace position and are parked after the ranked
    // keys, ordered among themselves by that same string, so the result is fully
    // determined either way (REQ-PARSE-SCAN-001 AC-14).
    auto orderBucketKeys = [&](std::vector<ObjectKey>& keys)
    {
        if (keys.size() < 2)
            return;
        std::vector<std::pair<std::string, ObjectKey>> decorated;
        decorated.reserve(keys.size());
        for (const ObjectKey key : keys)
            decorated.emplace_back(std::string(src.sourceKeyToString(key)), key);
        std::stable_sort(decorated.begin(), decorated.end(),
                         [](const std::pair<std::string, ObjectKey>& a,
                            const std::pair<std::string, ObjectKey>& b) -> bool
                         {
                             // Ranked (path-resolved) keys sort before parked
                             // (unresolved-path) keys; within each group, by path.
                             if (a.first.empty() != b.first.empty())
                                 return !a.first.empty();
                             return a.first < b.first;
                         });
        for (size_t i = 0; i < keys.size(); ++i)
            keys[i] = decorated[i].second;
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
        return applyScanFilter(enumerate(src, instance, dict, predAttr, op, predValue, probeAttr, ordinal, nullptr));
    };
    auto enumeratePrimTypeFiltered = [&](const char* typeName, const char* probeAttr,
                                         ovstage_ordinal_t ordinal) -> Bucket
    {
        // The authored-type column cannot be used in an IN filter. Its merged
        // type index provides the exact keys; older stages retain live enumeration.
        std::vector<ObjectKey> indexed;
        if (src.collectPrimTypeKeys(typeName, indexed))
        {
            // Nor does a populated family none of whose prims the scan's filter would keep (a
            // scoped rescan below a spawned prim, with the scene elsewhere): the live enumerate
            // could only return keys applyScanFilter drops.
            const bool anyInScope = std::any_of(indexed.begin(), indexed.end(), [&](ObjectKey key)
                                                { return keyPassesFilter(key) && !isBelowPointInstancer(key); });
            if (!anyInScope)
                return Bucket{};
            if (src.hasAuthoredTypeColumn() == std::optional<bool>(true))
            {
                Bucket bucket;
                bucket.keys = std::move(indexed);
                return applyScanFilter(std::move(bucket));
            }
        }
        if (src.hasAuthoredTypeColumn() != std::optional<bool>(false))
            throw std::runtime_error("OVStage schema scan could not complete the authored prim-type index "
                                     "for ovstage_read_attributes at read ordinal " + std::to_string(ordinal));
        Bucket bucket = enumerate(src, instance, dict, conv::kUsdPrimType, OVSTAGE_FILTER_OP_IN,
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

        Bucket bucket = enumerateSchema(src, instance, dict, schemaName, probeAttr, ordinal);
        return applyScanFilter(bucket);
    };
    auto enumerateSchemasFiltered = [&](const char* const* schemaNames, size_t schemaCount, const char* probeAttr,
                                        ovstage_ordinal_t ordinal) -> Bucket
    {
        Bucket outBucket;
        for (size_t i = 0; i < schemaCount; ++i)
        {
            Bucket one = enumerateSchemaFiltered(schemaNames[i], probeAttr, ordinal);
            appendUnique(src, outBucket.keys, one.keys);
            for (const std::string& attr : one.attrs)
            {
                if (std::find(outBucket.attrs.begin(), outBucket.attrs.end(), attr) == outBucket.attrs.end())
                    outBucket.attrs.push_back(attr);
            }
        }
        // Each per-schema bucket is namespace-ordered on its own, but appending
        // them yields [schema0 keys][schema1 keys]. Re-order the union so a prim
        // carrying several of the schemas lands at its namespace position rather
        // than at whichever schema happened to claim it first.
        orderBucketKeys(outBucket.keys);
        return outBucket;
    };

    const TokenId tokRigidBodyEnabled = src.internToken("physics:rigidBodyEnabled");
    const TokenId tokKinematicEnabled = src.internToken("physics:kinematicEnabled");
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
    const TokenId tokGeomSubset = src.internToken("GeomSubset");
    const TokenId tokElementType = src.internToken("elementType");
    const TokenId tokFace = src.internToken("face");
    const TokenId tokPhysicsMaterialAPI = src.internToken("PhysicsMaterialAPI");

    // Discover dispatch candidates before installing the ordinary descriptor
    // prune roots. These targeted type/API buckets include nested instancers and
    // invalid samplers without walking unrelated render prims in the consumer.
    std::vector<ObjectKey> particleObjectKeys;
    std::vector<ObjectKey> instancerKeys;
    std::unordered_set<uint64_t> candidateIdentities;
    std::unordered_set<uint64_t> instancerIdentities;
    auto appendCandidates = [&](const Bucket& bucket, bool instancer)
    {
        for (const ObjectKey key : bucket.keys)
        {
            const uint64_t canonical = src.canonicalPath(key);
            const uint64_t identity = canonical ? canonical : key.handle;
            if (candidateIdentities.insert(identity).second)
                particleObjectKeys.push_back(key);
            // A consumer may also register a built-in particle type as an
            // instancer, so descriptor pruning is independent of dispatch dedup.
            if (instancer && instancerIdentities.insert(identity).second)
                instancerKeys.push_back(key);
        }
    };
    for (const char* typeName : { "PhysxParticleSystem", "PointInstancer", "PhysxPhysicsJointInstancer" })
    {
        const Bucket bucket = enumeratePrimTypeFiltered(typeName, conv::kLocalTransform, readOrdinal);
        appendCandidates(bucket, std::string_view(typeName) != "PhysxParticleSystem");
    }
    for (const std::string& typeName : parse::customTokens(parse::CustomTokenKind::ePhysicsInstancer))
    {
        if (typeName == "PointInstancer" || typeName == "PhysxPhysicsJointInstancer")
            continue;
        const Bucket bucket = enumeratePrimTypeFiltered(typeName.c_str(), conv::kLocalTransform, readOrdinal);
        appendCandidates(bucket, true);
    }
    std::unordered_map<uint32_t, std::unordered_set<uint64_t>> candidateSchemaIdentities;
    for (const char* schemaName : { "PhysxParticleSetAPI", "PhysxParticleSamplingAPI", "InfiniteVoxelMapAPI" })
    {
        const Bucket bucket = enumerateSchemaFiltered(schemaName, conv::kLocalTransform, readOrdinal);
        appendCandidates(bucket, false);
        std::unordered_set<uint64_t>& identities = candidateSchemaIdentities[src.internToken(schemaName).id];
        for (const ObjectKey key : bucket.keys)
        {
            const uint64_t canonical = src.canonicalPath(key);
            identities.insert(canonical ? canonical : key.handle);
        }
    }
    // These three APIs are single-apply. Reuse their exact discovery buckets instead
    // of asking hasSchema's generic unqualified-base fallback to index every schema
    // on the stage when, for example, a visual PointInstancer has no ParticleSetAPI.
    auto hasCandidateSchema = [&](ObjectKey key, TokenId token) -> bool
    {
        const uint64_t canonical = src.canonicalPath(key);
        return candidateSchemaIdentities.at(token.id).count(canonical ? canonical : key.handle) != 0;
    };
    orderBucketKeys(particleObjectKeys);
    const KnownTokens& dispatchTokens = ctx.knownTokens();
    const TokenId voxelMapAPI = src.internToken("InfiniteVoxelMapAPI");
    for (const ObjectKey key : particleObjectKeys)
    {
        using Kind = ParticleObjectCandidate::Kind;
        // Classify once while the scan's load cache is active; the consumer must
        // not repeat isA() fallbacks against the render instancing graph.
        if (src.isA(key, dispatchTokens.physxParticleSystemType))
            out.particleObjectCandidates.push_back({ key, Kind::eParticleSystem });
        else if (src.isA(key, dispatchTokens.xformType))
        {
            if (hasCandidateSchema(key, voxelMapAPI))
                out.particleObjectCandidates.push_back({ key, Kind::eVoxelMap });
            else
            {
                const uint64_t canonical = src.canonicalPath(key);
                if (instancerIdentities.count(canonical ? canonical : key.handle))
                    out.particleObjectCandidates.push_back({ key, Kind::eCustomInstancer });
            }
        }
        else if (src.isA(key, dispatchTokens.meshType) && hasCandidateSchema(key, dispatchTokens.physxParticleSamplingAPI))
            out.particleObjectCandidates.push_back({ key, Kind::eParticleSampler });
        else if ((src.isA(key, dispatchTokens.pointBasedType) || src.isA(key, dispatchTokens.pointInstancerType)) &&
                 hasCandidateSchema(key, dispatchTokens.physxParticleSetAPI))
            out.particleObjectCandidates.push_back({ key, Kind::eParticleSet });
        else if (src.isA(key, dispatchTokens.pointInstancerType))
            out.particleObjectCandidates.push_back({ key, Kind::ePointInstancer });
        else if (src.isA(key, dispatchTokens.physxPhysicsJointInstancerType))
            out.particleObjectCandidates.push_back({ key, Kind::eJointInstancer });
        else
        {
            const uint64_t canonical = src.canonicalPath(key);
            if (instancerIdentities.count(canonical ? canonical : key.handle))
                out.particleObjectCandidates.push_back({ key, Kind::eCustomInstancer });
        }
    }

    out.hasPointInstancerPrims = !instancerKeys.empty();
    if (!filter || filter->prunePointInstancerDescendants)
    {
        pointInstancerSubtreeRoots.reserve(instancerKeys.size());
        for (const ObjectKey key : instancerKeys)
            pointInstancerSubtreeRoots.emplace_back(src.sourceKeyToString(key));
    }

    waveTimer.transition(stats ? &waveSeconds[1] : nullptr);
    // --- Scenes (PRIM_TYPE == PhysicsScene) ---
    Bucket sceneBucket = enumeratePrimTypeFiltered("PhysicsScene", "physics:gravityMagnitude", readOrdinal);
    // Scoped additions commonly have no scene or material. Avoid assembling
    // their column lists and preparation storage after membership filtering.
    if (!sceneBucket.keys.empty())
    {
        const std::vector<std::string> sceneDiscoveredAttrs = sceneBucket.attrs;
        const size_t sceneAttrCount = sceneBucket.attrs.size();
        appendSceneAttrs(sceneBucket);
        src.prefetchBucket(sceneBucket.keys, sceneDiscoveredAttrs);
        src.prefetchBucket(sceneBucket.keys, appendedAttrs(sceneBucket, sceneAttrCount));
        {
            DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
            std::vector<PreparedScene> inputs;
            inputs.reserve(sceneBucket.keys.size());
            for (const ObjectKey key : sceneBucket.keys)
            {
                SceneInfo info;
                src.getAttribute(key, tokGravityDir, info.gravityDirection);
                src.getAttribute(key, tokGravityMag, info.gravityMagnitude);
                inputs.push_back(prepareScene(ctx, key, info));
            }
            if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
            phase.transition(stats ? &stats->constructSeconds : nullptr);
            out.scenes = allocateDescriptorBatch<PhysxSceneDesc>(batches, inputs.size());
            std::vector<uint8_t> valid(inputs.size());
            phase.finish();
            fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
            {
                for (size_t i = begin; i < end; ++i)
                {
                    setToDefault(*out.scenes[i], units);
                    valid[i] = fillScene(inputs[i], *out.scenes[i]);
                    out.scenes[i]->primKey = sceneBucket.keys[i];
                }
            });
            if (std::find(valid.begin(), valid.end(), uint8_t(0)) != valid.end())
                throw std::logic_error("unprepared scene input");
        }
    }
    src.clearBucket();

    // No authored PhysicsScene → synthesize a default scene descriptor so a lone
    // body still simulates (parity with the USD path, which authors a temp scene
    // prim). ovstage authors NO prim: this is an in-memory descriptor flagged
    // `synthetic`, with a stable synthetic path so pathFor/keyFor round-trip in the
    // consumer (which skips the source existence/ownership gates for it). See
    // makeDefaultSceneDesc + LoadStage processScannedDescs.
    //
    // "This stage authors no PhysicsScene" is a WHOLE-STAGE fact and cannot be
    // concluded from a SCOPED scan. An incremental re-scan rooted at a newly added
    // prim (PrimUpdateMap::addPrim) normally contains no scene, so synthesizing one
    // here published a spurious extra `/__defaultPhysicsScene__` on every runtime
    // prim add — measured as a third object-created notification where USD reports
    // two. The USD path has no walker-side synthesis at all: LoadStage creates its
    // default scene only when `initialStageLoad && !mSceneFound`. Gating on an
    // unscoped scan is the ovstage mirror of that, and a scoped initial load still
    // gets LoadStage's own fallback.
    //
    // An exclude-only filter scopes the scan just as much as a `scanRoots` one does --
    // the excluded subtree is exactly where the authored scene may live -- so it must
    // not synthesize either. "Scoped" here has to mean the same thing it means to
    // `keyPassesFilter` above, which *ignores* a root-prim-level exclude (see
    // `LoadStage.cpp::forEachLoadObject` for why: the legacy `Traverse()` walk yielded
    // the root prim unchecked, so excluding e.g. "/World" was always a no-op). An
    // exclude list holding only such paths therefore prunes nothing and leaves the
    // scan genuinely whole-stage.
    //
    // Scope note: no in-repo runtime caller constructs an exclude-only filter today.
    // `loadFromStage` passes `scanRoots{SdfPath::AbsoluteRootPath()}` on every load,
    // selective or not (`LoadStage.cpp`), so the production selective-load path was
    // already scoped by its roots. This gate is the direct `scanOvstage` /
    // `IScanBackend` contract, pinned by `TestOvstageWalker.cpp`.
    const bool excludesScopeTheScan =
        filter && std::any_of(filter->excludePaths.begin(), filter->excludePaths.end(),
                              [&](const std::string& exclude) { return !isRootPrimPath(exclude); });
    const bool wholeStageScan = !filter || (filter->scanRoots.empty() && !excludesScopeTheScan);
    if (out.scenes.empty() && wholeStageScan)
    {
        if (DescPtr<PhysxSceneDesc> def = makeDefaultSceneDesc(batches, units))
        {
            def->primKey = src.findByPath(conv::kDefaultScenePath);
            out.scenes.push_back(std::move(def));
        }
    }

    waveTimer.transition(stats ? &waveSeconds[2] : nullptr);
    // --- Materials (HAS_APPLIED_SCHEMA PhysicsMaterialAPI) ---
    // Emit the material descriptors (friction / restitution / density). The
    // material→shape binding is resolved by ovpopulation as a target-path
    // relationship and read back via OvstageSource::getMaterialBinding, wired
    // into each shape's sourceMaterials in the shape loop below.
    Bucket materialBucket = enumerateSchemaFiltered("PhysicsMaterialAPI",
                                                    "physics:dynamicFriction",
                                                    readOrdinal);
    if (!materialBucket.keys.empty())
    {
        const std::vector<std::string> materialDiscoveredAttrs = materialBucket.attrs;
        const size_t materialAttrCount = materialBucket.attrs.size();
        appendMaterialAttrs(materialBucket);
        src.prefetchBucket(materialBucket.keys, materialDiscoveredAttrs);
        src.prefetchBucket(materialBucket.keys, appendedAttrs(materialBucket, materialAttrCount));
        {
            DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
            std::vector<PreparedMaterial> inputs(materialBucket.keys.size());
            for (size_t index = 0; index < inputs.size(); ++index)
                inputs[index] = prepareMaterial(ctx, materialBucket.keys[index]);
            if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
            phase.transition(stats ? &stats->constructSeconds : nullptr);
            out.materials = allocateDescriptorBatch<PhysxMaterialDesc>(batches, inputs.size());
            std::vector<uint8_t> valid(inputs.size(), 0);
            phase.finish();
            fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
            {
                for (size_t index = begin; index < end; ++index)
                    valid[index] = fillMaterial(inputs[index], *out.materials[index]) ? 1 : 0;
            });
            if (std::find(valid.begin(), valid.end(), uint8_t(0)) != valid.end())
                throw std::logic_error("unprepared material input");
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
        {
            DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
            std::vector<PreparedPBDMaterial> inputs;
            inputs.reserve(materialBucket.keys.size());
            for (const ObjectKey key : materialBucket.keys)
            {
                PreparedPBDMaterial input = preparePBDMaterial(ctx, key);
                if (input.enabled) inputs.push_back(input);
            }
            if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
            phase.transition(stats ? &stats->constructSeconds : nullptr);
            out.pbdMaterials = allocateDescriptorBatch<PBDMaterialDesc>(batches, inputs.size());
            phase.finish();
            fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
            {
                for (size_t i = begin; i < end; ++i)
                    fillPBDMaterial(inputs[i], *out.pbdMaterials[i]);
            });
        }
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
    {
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedDeformableMaterial> inputs;
        inputs.reserve(deformableMaterialBucket.keys.size());
        size_t surfaceCount = 0;
        for (const ObjectKey key : deformableMaterialBucket.keys)
        {
            inputs.push_back(prepareDeformableMaterial(ctx, key, true));
            surfaceCount += inputs.back().surface;
        }
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<PhysxDeformableMaterialDesc>> surface =
            allocateDescriptorBatch<PhysxSurfaceDeformableMaterialDesc, PhysxDeformableMaterialDesc>(batches, surfaceCount);
        std::vector<DescPtr<PhysxDeformableMaterialDesc>> volume =
            allocateDescriptorBatch<PhysxDeformableMaterialDesc>(batches, inputs.size() - surfaceCount);
        out.deformableMaterials.resize(inputs.size());
        size_t surfaceIndex = 0, volumeIndex = 0;
        for (size_t i = 0; i < inputs.size(); ++i)
            out.deformableMaterials[i] = inputs[i].surface ? std::move(surface[surfaceIndex++]) : std::move(volume[volumeIndex++]);
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i)
            {
                PhysxDeformableMaterialDesc& desc = *out.deformableMaterials[i];
                if (inputs[i].surface)
                    fillSurfaceDeformableMaterial(inputs[i], static_cast<PhysxSurfaceDeformableMaterialDesc&>(desc));
                else
                    fillDeformableMaterial(inputs[i], desc);
                desc.poissonsRatio = std::min(desc.poissonsRatio, 0.4999f);
            }
        });
    }
    src.clearBucket();

    waveTimer.transition(stats ? &waveSeconds[3] : nullptr);
    // --- Rigid bodies (HAS_APPLIED_SCHEMA PhysicsRigidBodyAPI) ---
    std::unordered_set<ObjectKey, ObjectKey::Hash> bodySet;
    std::unordered_map<ObjectKey, PhysxRigidBodyDesc*, ObjectKey::Hash> bodyByKey;
    // canonical intern_path id → body's enumerate key. Populated relationship
    // targets (physics:body0/1) are canonical intern_path ids, which the ovstage
    // lib hands back DISTINCT from the enumerate/get_paths handles in `bodySet`
    // for the same path. This maps a canonical handle back to the enumerate body
    // key so a resolved body matches the body descriptor's primKey.
    std::unordered_map<uint64_t, ObjectKey> bodyByCanonical;

    DescriptorPhaseTimer bodySetupTimer(stats ? &bodySetup.enumerate : nullptr);
    Bucket bodyBucket = enumerateSchemaFiltered("PhysicsRigidBodyAPI",
                                                conv::kLocalTransform,
                                                readOrdinal);
    const std::vector<std::string> bodyDiscoveredAttrs = bodyBucket.attrs;
    const size_t bodyAttrCount = bodyBucket.attrs.size();

    // The collision-shape family is enumerated here (the shape section below reuses it) so ONE
    // columnar read can cover both families: bodies, shapes, their geometry backings and every
    // ancestor, times every column the two sections and the consumer's follow-ups (mass update,
    // actor setup) ask. Complete load-cache coverage lets the per-section prefetches below
    // skip their preparation; usd-path in the set seeds the existence memo for the
    // ancestor chains the actor setup walks.
    Bucket shapeBucket = enumerateSchemaFiltered("PhysicsCollisionAPI", conv::kLocalTransform, readOrdinal);
    const std::vector<std::string> shapeDiscoveredAttrs = shapeBucket.attrs;
    const size_t shapeAttrCount = shapeBucket.attrs.size();
    std::vector<ObjectKey> shapeReadKeys = shapeBucket.keys;
    {
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
    }
    bool mergedReadComplete = false;
    {
        Bucket merged;
        std::unordered_set<uint64_t> mergedIdentity; // canonical prim id: one row per prim
        auto addKeys = [&](const std::vector<ObjectKey>& keys)
        {
            for (const ObjectKey key : keys)
            {
                const uint64_t canonical = src.canonicalPath(key);
                if (mergedIdentity.insert(canonical ? canonical : key.handle).second)
                    merged.keys.push_back(key);
            }
        };
        addKeys(bodyBucket.keys);
        std::vector<ObjectKey> bodyBackingKeys;
        for (const ObjectKey key : bodyBucket.keys)
        {
            const ObjectKey backing = src.collisionAttributeBackingKey(key);
            if (backing.valid())
                bodyBackingKeys.push_back(backing);
        }
        addKeys(bodyBackingKeys);
        addKeys(shapeReadKeys);
        addKeys(src.collectAncestors(merged.keys));
        // Cache only the fixed column names, never scene values or membership.
        // Borrow the plan unless this scan discovers additional family columns.
        static const std::vector<std::string> fixedColumns = []
        {
            Bucket all, body, shape;
            appendTransformAttrs(all);
            for (const std::string& attr : OvstageSource::relationshipPrefetchAttrs())
                appendBucketAttr(all, attr);
            appendRigidBodyAttrs(body);
            appendMassAttrs(body);
            appendCollisionShapeAttrs(shape);
            appendMassAttrs(shape);
            for (const Bucket* family : { &body, &shape })
                for (const std::string& attr : family->attrs)
                    appendBucketAttr(all, attr);
            appendBucketAttr(all, conv::kUsdPath);
            appendBucketAttr(all, "_instanceMaterialBinding");
            return all.attrs;
        }();
        const std::vector<std::string>* mergedAttrs = &fixedColumns;
        if (!bodyBucket.attrs.empty() || !shapeBucket.attrs.empty())
        {
            merged.attrs = fixedColumns;
            for (const Bucket* family : { &bodyBucket, &shapeBucket })
                for (const std::string& attr : family->attrs)
                    appendBucketAttr(merged, attr);
            mergedAttrs = &merged.attrs;
        }
        // Resolve collision scalars and collider density. Identity, transforms, schemas and
        // relationships keep their logical rows. The backing values join this SAME
        // body/shape read; parser getAttribute calls only inspect its cached rows.
        src.prefetchResolvedInstanceBucket(merged.keys, *mergedAttrs, shapeBucket.keys, {
            "physics:density",
            "physics:collisionEnabled",
            "physics:approximation",
            "physxCollision:contactOffset",
            "physxCollision:restOffset",
            "physxCollision:torsionalPatchRadius",
            "physxCollision:minTorsionalPatchRadius",
            "newton:contactMargin",
            "newton:contactGap",
        });
        mergedReadComplete = src.loadCacheCovers(merged.keys, *mergedAttrs);
    }

    if (!mergedReadComplete)
    {
        appendRigidBodyAttrs(bodyBucket);
        appendMassAttrs(bodyBucket);
        appendCollisionShapeAttrs(shapeBucket);
        appendMassAttrs(shapeBucket);
    }
    bodySetupTimer.transition(stats ? &bodySetup.transformAncestors : nullptr);
    if (!mergedReadComplete)
        prefetchTransformAncestors(src, bodyBucket.keys);
    bodySetupTimer.transition(stats ? &bodySetup.relationshipAncestors : nullptr);
    if (!mergedReadComplete)
        src.prefetchRelationshipAncestors(bodyBucket.keys);
    bodySetupTimer.transition(stats ? &bodySetup.transforms : nullptr);
    if (!mergedReadComplete)
        prefetchTransformsForKeys(src, bodyBucket.keys);
    bodySetupTimer.transition(stats ? &bodySetup.metadata : nullptr);
    const KnownTokens& bodyTokens = ctx.knownTokens();
    std::vector<uint8_t> columnBodies(bodyBucket.keys.size());
    bool commonBodiesOnly = true;
    for (size_t i = 0; i < bodyBucket.keys.size(); ++i)
    {
        columnBodies[i] = !src.hasSchema(bodyBucket.keys[i], bodyTokens.physxSurfaceVelocityAPI) &&
                          !src.hasSchema(bodyBucket.keys[i], bodyTokens.physxSplinesSurfaceVelocityAPI);
        commonBodiesOnly = commonBodiesOnly && columnBodies[i];
    }
    bodySetupTimer.transition(stats ? &bodySetup.attributes : nullptr);
    if (!mergedReadComplete)
    {
        if (commonBodiesOnly)
            src.prefetchBucket(bodyBucket.keys, {"physics:filteredPairs", "physics:simulationOwner"});
        else
        {
            src.prefetchBucket(bodyBucket.keys, bodyDiscoveredAttrs);
            src.prefetchBucket(bodyBucket.keys, appendedAttrs(bodyBucket, bodyAttrCount));
        }
    }
    {
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        struct BodyInput
        {
            ObjectKey key;
            Matrix4d world;
            std::vector<ObjectKey> filtered, owners;
            bool supported = true, kinematic = false;
            bool dynamic = false, columnRead = false, physxBodyAPI = false;
            size_t fallbackIndex = 0;
            size_t dynamicIndex = 0;
        };
        std::vector<BodyInput> inputs(bodyBucket.keys.size());
        std::vector<PreparedDynamicBody> dynamicInputs;
        const auto bodyScalarTokens = dynamicBodyAttributeTokens(bodyTokens);
        std::vector<TokenId> scalarTokens(bodyScalarTokens.begin(), bodyScalarTokens.end());
        scalarTokens.push_back(tokRigidBodyEnabled);
        bodySetupTimer.transition(stats ? &bodySetup.columns : nullptr);
        const FrozenScalarBatch scalarRows = commonBodiesOnly ?
            src.readScalarBatch(bodyBucket.keys, scalarTokens.data(), scalarTokens.size()) :
            src.freezeScalarBatch(bodyBucket.keys, scalarTokens.data(), scalarTokens.size());
        bodySetupTimer.transition(stats ? &bodySetup.inputs : nullptr);
        const bool localVelocityDefault = ctx.outputVelocitiesLocalSpaceDefault();
        size_t dynamicCount = 0;
        size_t staticCount = 0, preparedBytes = inputs.capacity() * sizeof(BodyInput);
        for (size_t index = 0; index < inputs.size(); ++index)
        {
            BodyInput& input = inputs[index];
            input.key = bodyBucket.keys[index];
            bool enabled = true;
            bool complete = true;
            const ObjectKey backing = src.collisionAttributeBackingKey(input.key);
            if (!scalarRows.row(index, complete).getAttribute(input.key, tokRigidBodyEnabled, enabled) && backing.valid())
                src.getAttribute(backing, tokRigidBodyEnabled, enabled);
            if (!src.getAttribute(input.key, tokKinematicEnabled, input.kinematic) && backing.valid())
                src.getAttribute(backing, tokKinematicEnabled, input.kinematic);
            if (backing.valid() && enabled && !input.kinematic)
            {
                const std::string path(src.sourceKeyToString(input.key));
                CARB_LOG_WARN("RigidBodyAPI on an instance proxy not supported, "
                              "unless set to kinematic or not enabled. %s", path.c_str());
                input.supported = false;
                continue;
            }
            input.dynamic = enabled;
            if (enabled)
            {
                input.dynamicIndex = dynamicCount++;
                input.columnRead = columnBodies[index];
                if (input.columnRead)
                    input.physxBodyAPI = src.hasSchema(input.key, bodyTokens.physxRigidBodyAPI);
                else
                {
                    input.fallbackIndex = dynamicInputs.size();
                    dynamicInputs.push_back(prepareDynamicBody(ctx, input.key));
                }
            }
            else
                ++staticCount;
            src.getLocalToWorldTransform(input.key, input.world);
            input.filtered = parseFilteredPairs(ctx, input.key);
            src.getRelationshipTargets(input.key, tokSimulationOwner, input.owners);
            preparedBytes += (input.filtered.capacity() + input.owners.capacity()) * sizeof(ObjectKey);
        }
        bodySetupTimer.finish();
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes,
            preparedBytes + scalarRows.capacityBytes() + dynamicInputs.capacity() * sizeof(PreparedDynamicBody));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<PhysxRigidBodyDesc>> dynamicOwners =
            allocateDescriptorBatch<DynamicPhysxRigidBodyDesc, PhysxRigidBodyDesc>(batches, dynamicCount);
        std::vector<DescPtr<PhysxRigidBodyDesc>> staticOwners =
            allocateDescriptorBatch<StaticPhysxRigidBodyDesc, PhysxRigidBodyDesc>(batches, staticCount);
        out.bodies.resize(inputs.size());
        size_t staticIndex = 0;
        for (size_t i = 0; i < inputs.size(); ++i)
        {
            if (!inputs[i].supported) continue;
            out.bodies[i] = inputs[i].dynamic ? std::move(dynamicOwners[inputs[i].dynamicIndex]) : std::move(staticOwners[staticIndex++]);
            out.bodies[i]->sourceFilteredCollisions.resize(inputs[i].filtered.size());
            out.bodies[i]->sourceSimulationOwners.resize(inputs[i].owners.size());
        }
        std::vector<uint8_t> valid(inputs.size());
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i)
            {
                const BodyInput& input = inputs[i];
                if (!input.supported) { valid[i] = true; continue; }
                PhysxRigidBodyDesc& output = *out.bodies[i];
                Matrix3d rotation;
                carb::Float3 scale;
                decomposeWorldTransform(input.world, rotation, scale);
                valid[i] = true;
                if (input.dynamic)
                {
                    if (input.columnRead)
                    {
                        PreparedDynamicBody decoded;
                        decoded.units = units;
                        decoded.outputVelocitiesLocalSpaceDefault = localVelocityDefault;
                        decoded.physxRigidBodyAPI = input.physxBodyAPI;
                        decoded.rotation = rotation;
                        decoded.scale = scale;
                        bool complete = true;
                        const auto row = scalarRows.row(i, complete);
                        readDynamicBodyAttributes(row, input.key, bodyTokens, decoded);
                        decoded.prepared = complete;
                        valid[i] = fillDynamicBody(decoded, static_cast<DynamicPhysxRigidBodyDesc&>(output));
                    }
                    else
                        valid[i] = fillDynamicBody(dynamicInputs[input.fallbackIndex], static_cast<DynamicPhysxRigidBodyDesc&>(output));
                    static_cast<DynamicPhysxRigidBodyDesc&>(output).kinematicBody = input.kinematic;
                }
                else
                    static_cast<StaticPhysxRigidBodyDesc&>(output).sourceGPrimKey = input.key;
                output.primKey = input.key;
                output.position = { static_cast<float>(input.world.data[12]), static_cast<float>(input.world.data[13]),
                                    static_cast<float>(input.world.data[14]) };
                output.rotation = quatFromRotation(rotation);
                output.scale = scale;
                std::copy(input.filtered.begin(), input.filtered.end(), output.sourceFilteredCollisions.begin());
                std::copy(input.owners.begin(), input.owners.end(), output.sourceSimulationOwners.begin());
            }
        });
        phase.transition(stats ? &stats->finalizeSeconds : nullptr);
        if (std::find(valid.begin(), valid.end(), uint8_t(0)) != valid.end())
            throw std::logic_error("unprepared body input");
        out.bodies.erase(std::remove_if(out.bodies.begin(), out.bodies.end(),
            [](const auto& body) { return !body; }), out.bodies.end());
        for (const DescPtr<PhysxRigidBodyDesc>& body : out.bodies)
        {
            bodySet.insert(body->primKey);
            bodyByKey.emplace(body->primKey, body.get());
            bodyByCanonical.emplace(src.canonicalPath(body->primKey), body->primKey);
        }
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

    waveTimer.transition(stats ? &waveSeconds[4] : nullptr);
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
    {
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedOvstageDeformable> inputs;
        for (ObjectKey key : deformableBodyBucket.keys) prepareOvstageDeformable(inputs, instance, src, ctx, key);
        const size_t surfaces = size_t(std::count_if(inputs.begin(), inputs.end(), [](const PreparedOvstageDeformable& input) { return input.surface; }));
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<PhysxDeformableBodyDesc>> surface = allocateDescriptorBatch<PhysxSurfaceDeformableBodyDesc, PhysxDeformableBodyDesc>(batches, surfaces);
        std::vector<DescPtr<PhysxDeformableBodyDesc>> volume = allocateDescriptorBatch<PhysxVolumeDeformableBodyDesc, PhysxDeformableBodyDesc>(batches, inputs.size() - surfaces);
        out.deformables.resize(inputs.size());
        size_t si = 0, vi = 0;
        for (size_t i = 0; i < inputs.size(); ++i)
        {
            out.deformables[i] = inputs[i].surface ? std::move(surface[si++]) : std::move(volume[vi++]);
            prepareOvstageDeformableStorage(inputs[i], *out.deformables[i]);
        }
        std::vector<uint8_t> valid(inputs.size());
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) valid[i] = fillOvstageDeformable(inputs[i], units, *out.deformables[i]);
        });
        for (size_t i = 0; i < inputs.size(); ++i)
        {
            if (!valid[i]) throw std::logic_error("unprepared deformable body input or storage");
            deformableBodyEnabledByCanonical[src.canonicalPath(inputs[i].key)] = inputs[i].body.bodyEnabled;
        }
    }
    src.clearBucket();

    auto isDeformableCollider = [&](ObjectKey shapeKey) -> bool
    {
        if (deformableBodyEnabledByCanonical.empty())
            return false;
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

    waveTimer.transition(stats ? &waveSeconds[5] : nullptr);
    // --- Collision shapes (HAS_APPLIED_SCHEMA PhysicsCollisionAPI) ---
    // Analytic and mesh geometry keep logical identity and transforms on an
    // instance-proxy path while reading authored geometry from its prototype. The family
    // (shapeBucket / shapeReadKeys) was enumerated with the rigid bodies above; the merged read
    // there already covers these prefetches under the load cache.
    DescriptorPhaseTimer shapeSetupTimer(stats ? &readTimings.shapeSetup : nullptr);
    DescriptorPhaseTimer shapeDetailTimer(stats ? &shapeSetup.enumerate : nullptr);
    shapeDetailTimer.transition(stats ? &shapeSetup.transformAncestors : nullptr);
    if (!mergedReadComplete)
        prefetchTransformAncestors(src, shapeBucket.keys);
    shapeDetailTimer.transition(stats ? &shapeSetup.relationshipAncestors : nullptr);
    if (!mergedReadComplete)
        src.prefetchRelationshipAncestors(shapeBucket.keys);
    shapeDetailTimer.transition(stats ? &shapeSetup.transforms : nullptr);
    if (!mergedReadComplete)
        prefetchTransformsForKeys(src, shapeBucket.keys);
    shapeDetailTimer.transition(stats ? &shapeSetup.attributes : nullptr);
    if (!mergedReadComplete)
    {
        src.prefetchBucket(shapeReadKeys, shapeDiscoveredAttrs);
        src.prefetchBucket(shapeReadKeys, appendedAttrs(shapeBucket, shapeAttrCount));
    }
    shapeDetailTimer.transition(stats ? &shapeSetup.inputs : nullptr);
    enum class PrimType : uint8_t
    {
        eCube,
        eSphere,
        eCapsule,
        eCylinder,
        eCone,
        ePlane,
        eMesh,
        eNone, // resolved, but not one of the analytic types above (e.g. Xform)
    };
    // Prefetch types only for collider and backing keys above. Descendant keys
    // discovered later retain the per-key fallback below; no stage-wide type
    // enumeration is needed. A prior version of this code
    // prefetched every prim of each analytic type stage-wide into
    // primTypeByCanonical up front -- including render-only geometry that has
    // nothing to do with physics -- which made attach cost scale with total
    // scene size instead of physics-relevant shape count (NVBug 6532970).
    // primTypeByCanonical is populated lazily by primTypeIs() the first
    // time a key's type resolves, so repeat probes for the same shape are served
    // from the map.
    std::unordered_map<uint64_t, PrimType> primTypeByCanonical;
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
        case PrimType::eNone: return nullptr;
        }
        return nullptr;
    };
    // Map an authored USD type name to one of the analytic PrimType buckets. Returns
    // eNone for any prim whose type is not analytic geometry (Xform colliders, etc.).
    auto primTypeFromName = [](std::string_view s) -> PrimType
    {
        if (s == "Cube") return PrimType::eCube;
        if (s == "Sphere") return PrimType::eSphere;
        if (s == "Capsule") return PrimType::eCapsule;
        if (s == "Cylinder") return PrimType::eCylinder;
        if (s == "Cone") return PrimType::eCone;
        if (s == "Plane") return PrimType::ePlane;
        if (s == "Mesh") return PrimType::eMesh;
        return PrimType::eNone;
    };
    auto primTypeIs = [&](ObjectKey key, PrimType primType) -> bool
    {
        const uint64_t canonical = src.canonicalPath(key);
        if (!canonical)
            return false;
        std::unordered_map<uint64_t, PrimType>::const_iterator it = primTypeByCanonical.find(canonical);
        if (it != primTypeByCanonical.end())
            return it->second == primType;

        // Resolve the prim type once (getTypeName reads the geometry backing; tetMesh
        // reports as "Mesh") so the analytic-type probes below hit the cache.
        const TokenId actualTypeToken = src.getTypeName(key);
        if (actualTypeToken.valid())
        {
            const std::string_view actualTypeName = src.tokenToString(actualTypeToken);
            if (!actualTypeName.empty())
            {
                const PrimType resolved = primTypeFromName(actualTypeName);
                primTypeByCanonical.emplace(canonical, resolved);
                return resolved == primType;
            }
        }

        // usd-prim-type unreadable: keep the uncached per-type probe.
        const char* typeName = primTypeName(primType);
        if (!typeName)
            return false;
        ovx_token_t typeToken = OVX_INVALID_TOKEN;
        if (ovx_path_dictionary_intern_token(dict, ovxStr(typeName), &typeToken) == OVX_OK &&
            tokenColumnContains(instance, dict, src, key, conv::kUsdPrimType, typeToken, readOrdinal))
        {
            // Remember the resolved type: the classifier calls primTypeIs() for
            // several candidate types per shape (and Mesh twice, for subset
            // materials), so caching the positive result answers the repeat and
            // every subsequent candidate from the map instead of paying another
            // synchronous usd-prim-type / isType() probe per shape.
            primTypeByCanonical.emplace(canonical, primType);
            return true;
        }
        if (isType(src, instance, key, typeName))
        {
            primTypeByCanonical.emplace(canonical, primType);
            return true;
        }
        return false;
    };

    // Consumer-registered custom-geometry tokens (REQ-PARSE-CORE-005). A token
    // matches either as an applied API on the collider or as its prim type, so
    // both are resolved here: applied APIs through the source's schema map, prim
    // types through one query per token (ADR-0002 open question 1) folded into a
    // canonical-path -> token index the way `rememberPrimType` does for the
    // analytic types.
    //
    // `PhysxMeshMergeCollisionAPI` is deliberately excluded. It is pre-registered
    // as a shape token but the native walker routes it to a *typed* mesh-merge
    // descriptor (`buildMeshMergeCustomShape`), not to a generic
    // CustomPhysxShapeDesc, and that builder has no ovstage counterpart. Letting
    // it through here would replace today's behaviour (children picked up as
    // ordinary colliders) with a custom shape whose hash no consumer has
    // registered — strictly worse. Mesh merge stays exactly as measured in
    // PLAN-ovstage-test-coverage-completion.md §6 gap #16.
    static const std::string kMeshMergeToken = "PhysxMeshMergeCollisionAPI";
    const std::vector<std::string> customShapeTokens = parse::customTokens(parse::CustomTokenKind::eShape);
    std::vector<TokenId> customShapeSchemaTokens;
    customShapeSchemaTokens.reserve(customShapeTokens.size());
    std::unordered_map<uint64_t, size_t> customShapeTokenByCanonical;
    for (size_t i = 0; i < customShapeTokens.size(); ++i)
    {
        const std::string& shapeToken = customShapeTokens[i];
        customShapeSchemaTokens.push_back(shapeToken == kMeshMergeToken ? TokenId{} :
                                                                          src.internToken(shapeToken));
        if (shapeToken == kMeshMergeToken)
            continue;
        const Bucket tokenBucket = enumeratePrimTypeFiltered(shapeToken.c_str(), conv::kLocalTransform, readOrdinal);
        for (const ObjectKey tokenKey : tokenBucket.keys)
        {
            const uint64_t canonical = src.canonicalPath(tokenKey);
            if (canonical)
                customShapeTokenByCanonical.emplace(canonical, i);
        }
    }
    // Index of the registered token that makes `colliderKey` a custom shape, or
    // npos. Applied APIs are tested first, matching the native walker's
    // `isCustomShapeApplied`, which scans the applied-API list before falling
    // back to the prim type.
    auto findCustomShapeToken = [&](ObjectKey colliderKey) -> size_t
    {
        if (customShapeTokens.empty())
            return std::string::npos;
        for (size_t i = 0; i < customShapeSchemaTokens.size(); ++i)
        {
            if (customShapeSchemaTokens[i].valid() && src.hasSchema(colliderKey, customShapeSchemaTokens[i]))
                return i;
        }
        const uint64_t canonical = src.canonicalPath(colliderKey);
        if (canonical)
        {
            const std::unordered_map<uint64_t, size_t>::const_iterator it =
                customShapeTokenByCanonical.find(canonical);
            if (it != customShapeTokenByCanonical.end())
                return it->second;
        }
        return std::string::npos;
    };

    {
        shapeDetailTimer.finish();
        shapeSetupTimer.finish();
        // Discovery owns all input storage; no descriptor exists until concrete
        // one-to-many emission counts are known. The source stays pinned through join.
        DescriptorPhaseTimer shapePhase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedOvstageShape> shapeInputs;
        shapeInputs.reserve(shapeBucket.keys.size());
        // Build and register one shape for a (collider, logical gprim) pair.
        // colliderKey owns collision state and gprimKey owns runtime identity,
        // transforms, and material. geometryKey is private ovstage backing used only
        // for prototype-authored type and geometry values.
        auto emitShape = [&](ObjectKey colliderKey, ObjectKey gprimKey) -> bool
        {
            DescriptorPhaseTimer detail(stats ? &readTimings.shapeClassify : nullptr);
            if (isDeformableCollider(colliderKey))
                return true;
            const ObjectKey geometryKey = src.geometryBackingKey(gprimKey);

            ShapeInfo info;
            info.rigidBody = resolveBody(colliderKey);
            info.sourceGprim = gprimKey;
            // info.simulationOwners stays empty so the parsers do not drop the shape on
            // unresolved scenes; the collider's owners are read onto the descriptor's
            // sourceSimulationOwners below, which is where the native walker puts them
            // and where the consumer resolves them.

            PreparedOvstageShape input;
            input.collider = colliderKey;
            input.geometry = geometryKey;
            detail.transition(stats ? &readTimings.shapeTransforms : nullptr);
            src.getLocalToWorldTransform(gprimKey, input.world);

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
                    ::physx::PxMat44d relativeTransform(::physx::PxIdentity);
                    ObjectKey current = gprimKey;
                    for (int guard = 0; current.valid() && guard < 64; ++guard)
                    {
                        if (src.canonicalPath(current) == bodyCanonical)
                            break;

                        Matrix4d localTransform;
                        bool resetsXformStack = false;
                        src.getLocalTransform(current, ReadTime::defaultTime(), localTransform, resetsXformStack);
                        // Gf order was `relativeTransform *= local`; operands swap in
                        // PhysX order (see common/foundation/MatrixTools.h).
                        relativeTransform = toPxMat44d(localTransform) * relativeTransform;
                        if (resetsXformStack)
                            break;
                        current = src.getParent(current);
                    }

                    Matrix4d bodyWorldTransform;
                    src.getLocalToWorldTransform(info.rigidBody, bodyWorldTransform);

                    input.relativePose = true;
                    std::memcpy(input.relative.data, &relativeTransform, sizeof(input.relative.data));
                    input.bodyWorld = bodyWorldTransform;
                }
            }
            detail.transition(stats ? &readTimings.shapeClassify : nullptr);
            // Precedence mirrors NativeWalker::emitShape: a Plane gprim is decided
            // before the custom-token check (which matters because "Plane" is itself a
            // pre-registered shape token, a legacy alias for a non-UsdGeomPlane prim
            // of that type), and the custom branch then outranks every analytic and
            // mesh branch below -- a custom-geometry API on a Sphere is a custom
            // shape, not a sphere.
            //
            // The two checks deliberately interrogate different keys, matching which
            // prim NativeWalker::emitShape tests for each: for a collider applied
            // directly to a gprim, colliderKey/gprimKey/geometryKey all collapse to
            // that one prim, same as native's single `shapePrim`. For a collider on a
            // non-gprim ancestor (the Xform-descent branch), native decides
            // `customShape` exactly once against the collider prim before
            // descending -- it is never re-tested per descendant child -- while the
            // Plane check runs per child against that child's own prim. So
            // `findCustomShapeToken` below is meant to see `colliderKey` (the
            // collider, fixed across children) and `planeGprim` is meant to see
            // `geometryKey` (the per-child geometry). Keep it this way rather than
            // unifying them onto one key; that would change which prim decides
            // custom-shape-ness for the Xform-descent case and diverge from native.
            const bool planeGprim = primTypeIs(geometryKey, PrimType::ePlane);
            const size_t customShapeToken = planeGprim ? std::string::npos : findCustomShapeToken(colliderKey);
            if (planeGprim)
            {
                input.type = ePlaneShape;
            }
            else if (customShapeToken != std::string::npos)
            {
                input.type = eCustomShape;
                input.customHash = customGeometryTokenHash(customShapeTokens[customShapeToken]);
            }
            else if (primTypeIs(geometryKey, PrimType::eCube))
            {
                input.type = eBoxShape;
            }
            else if (primTypeIs(geometryKey, PrimType::eSphere))
            {
                input.type = eSphereShape;
            }
            else if (primTypeIs(geometryKey, PrimType::eCapsule))
            {
                input.type = eCapsuleShape;
                input.radius = 0.5;
                input.height = 1.0;
            }
            else if (primTypeIs(geometryKey, PrimType::eCylinder) || primTypeIs(geometryKey, PrimType::eCone))
            {
                input.type = primTypeIs(geometryKey, PrimType::eCylinder) ? eCylinderShape : eConeShape;
            }
            else if (primTypeIs(geometryKey, PrimType::eMesh))
            {
                const auto body = bodyByKey.find(info.rigidBody);
                prepareMeshShape(ctx, src, colliderKey, gprimKey,
                    body == bodyByKey.end() ? nullptr : body->second, input);
            }
            else
                return false;
            input.common.units = units;
            input.common.info = std::move(info);
            input.common.prepared = true;
            input.common.collision.physxCollisionAPI = src.hasSchema(colliderKey, ctx.knownTokens().physxCollisionAPI);
            input.common.collision.physxTriggerAPI = src.hasSchema(colliderKey, ctx.knownTokens().physxTriggerAPI);
            input.common.collision.physxTriggerStateAPI = src.hasSchema(colliderKey, ctx.knownTokens().physxTriggerStateAPI);
            if (!input.common.ownersResolved)
                return false;

            detail.transition(stats ? &readTimings.shapeRelations : nullptr);
            DescriptorPhaseTimer relationTimer(stats ? &shapeRelationSeconds[0] : nullptr);
            // Per-face physics materials: a collider mesh expresses them as GeomSubset
            // children that each bind their own material, and that is the only way to
            // author more than one physics material on a mesh. They occupy
            // sourceMaterials [0..N) with the collider's own binding as the trailing
            // default slot, which is the order the consumer hands to
            // PxShape::setMaterials. Mirrors the native walker's mesh-subset branch.
            // Resolved on the geometry key so prototype-authored subsets are found for
            // an instance proxy too.
            if (primTypeIs(geometryKey, PrimType::eMesh))
            {
                src.forEachChild(geometryKey, [&](ObjectKey child)
                {
                    if (!src.isA(child, tokGeomSubset))
                        return;
                    TokenId elementType{};
                    if (!src.getAttribute(child, tokElementType, elementType) || elementType != tokFace)
                        return;
                    // A subset that binds no material of its own resolves the mesh's, so
                    // gate on the material actually being a physics material: a subset
                    // bound only to a render material must not take a slot.
                    const ObjectKey subsetMaterial = src.getMaterialBinding(child);
                    if (subsetMaterial.valid() && src.hasSchema(subsetMaterial, tokPhysicsMaterialAPI))
                        input.materials.push_back(subsetMaterial);
                });
            }

            // Read direct material relationships or the resolved instance-material
            // column through the source. Keep the lookup on the logical gprim so
            // per-instance overrides survive. Mirrors the native walker, including its
            // unconditional trailing slot: with per-face subsets present the collider's
            // own binding is the default-fallback entry at index N, and the consumer
            // indexes it positionally, so the slot has to exist even when nothing is
            // bound there.
            relationTimer.transition(stats ? &shapeRelationSeconds[1] : nullptr);
            const ObjectKey material = src.getMaterialBinding(gprimKey);
            if (material.valid() || !input.materials.empty())
                input.materials.push_back(material);

            // Per-shape collision filtering (PhysxFilteredPairsAPI on the collider).
            relationTimer.transition(stats ? &shapeRelationSeconds[2] : nullptr);
            input.filteredCollisions = parseFilteredPairs(ctx, colliderKey);

            // Collider-level simulationOwner (physics:simulationOwner on CollisionAPI),
            // which is what decides ownership for a bodyless static collider. Mirrors
            // the native walker: the owners ride on the descriptor and the consumer
            // resolves them, rather than going through ShapeInfo — whose
            // unresolved-scene path drops the shape outright.
            relationTimer.transition(stats ? &shapeRelationSeconds[3] : nullptr);
            src.getRelationshipTargets(colliderKey, tokSimulationOwner, input.simulationOwners);

            shapeInputs.push_back(std::move(input));
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
        const KnownTokens& shapeTokens = ctx.knownTokens();
        const ShapeColumnTokens shapeFields{tokCollisionEnabled, tokSize, tokRadius, tokHeight, tokAxis, tokDoubleSided, tokAxisX, tokAxisY};
        std::vector<ObjectKey> colliderRows, geometryRows;
        colliderRows.reserve(shapeInputs.size());
        geometryRows.reserve(shapeInputs.size());
        for (const auto& input : shapeInputs)
        {
            colliderRows.push_back(input.collider);
            geometryRows.push_back(input.geometry);
        }
        std::vector<TokenId> colliderAttributes{tokCollisionEnabled}, cookingTokens;
        appendCollisionAttributeTokens(PreparedCollisionExt{}, shapeTokens, colliderAttributes);
        std::array<bool, std::variant_size_v<PreparedShapeCooking>> declaredCooking{};
        bool declaredSdf = false, declaredMargin = false;
        for (const auto& input : shapeInputs)
        {
            if (!declaredCooking[input.cooking.index()])
            {
                std::visit([&](const auto& cooking)
                {
                    using T = std::decay_t<decltype(cooking)>;
                    if constexpr (!std::is_same_v<T, std::monostate>)
                        appendCollisionAttributeTokens(cooking, shapeTokens, colliderAttributes);
                    if constexpr (std::is_same_v<T, PreparedSphereFillCookingExt>)
                        cookingTokens.push_back(shapeTokens.physxSphereFillCollisionFillMode);
                }, input.cooking);
                declaredCooking[input.cooking.index()] = true;
            }
            if (input.sdf && !declaredSdf)
            {
                appendCollisionAttributeTokens(input.sdfCooking, shapeTokens, colliderAttributes);
                cookingTokens.push_back(shapeTokens.physxSDFMeshCollisionSdfBitsPerSubgridPixel);
                declaredSdf = true;
            }
            if ((input.type == eCylinderShape || input.type == eConeShape) && !declaredMargin)
            {
                colliderAttributes.push_back(shapeTokens.physxConvexGeometryMargin);
                declaredMargin = true;
            }
        }
        std::sort(colliderAttributes.begin(), colliderAttributes.end(), [](TokenId a, TokenId b) { return a.id < b.id; });
        colliderAttributes.erase(std::unique(colliderAttributes.begin(), colliderAttributes.end()), colliderAttributes.end());
        const auto geometryAttributes = shapeFields.geometryAttributes();
        DescriptorPhaseTimer columnTimer(stats ? &readTimings.columns : nullptr);
        const FrozenScalarBatch colliderColumns = src.readScalarBatch(colliderRows, colliderAttributes.data(), colliderAttributes.size(), cookingTokens.data(), cookingTokens.size());
        const FrozenScalarBatch geometryColumns = src.readScalarBatch(geometryRows, geometryAttributes.data(), geometryAttributes.size(), &tokAxis, 1);
        columnTimer.finish();
        if (stats)
        {
            size_t bytes = shapeInputs.capacity() * sizeof(PreparedOvstageShape) + colliderColumns.capacityBytes() +
                           geometryColumns.capacityBytes() + (colliderRows.capacity() + geometryRows.capacity()) * sizeof(ObjectKey);
            for (const PreparedOvstageShape& input : shapeInputs)
                bytes += (input.materials.capacity() + input.filteredCollisions.capacity() + input.simulationOwners.capacity() +
                          input.common.info.simulationOwners.capacity()) * sizeof(ObjectKey) +
                         input.common.sceneIds.capacity() * sizeof(ObjectId);
            stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, bytes);
        }
        shapePhase.transition(stats ? &stats->constructSeconds : nullptr);
        constructShapeBatches(batches, shapeInputs, out.shapes, out.ownedMeshes);
        std::vector<uint8_t> validShapes(shapeInputs.size(), 0);
        shapePhase.finish();
        fillDescriptorRanges(shapeInputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t index = begin; index < end; ++index)
            {
                auto& input = shapeInputs[index];
                bool complete = true;
                readOvstageShapeAttributes(colliderColumns.row(index, complete), geometryColumns.row(index, complete),
                                           shapeTokens, shapeFields, input);
                Matrix3d rotation;
                decomposeWorldTransform(input.world, rotation, input.geometryScale);
                if (input.relativePose)
                    omni::physics::decomposeCollisionShapeLocalTransform(toPxMat44d(input.relative), toPxMat44d(input.bodyWorld),
                        input.common.info.localPos, input.common.info.localRot, input.common.info.localScale);
                else if (!input.common.info.rigidBody.valid())
                {
                    input.common.info.localPos = {float(input.world.data[12]), float(input.world.data[13]), float(input.world.data[14])};
                    input.common.info.localRot = quatFromRotation(rotation);
                    input.common.info.localScale = input.geometryScale;
                }
                validShapes[index] = complete && fillOvstageShape(input, *out.shapes[index]);
            }
        });
        shapePhase.transition(stats ? &stats->finalizeSeconds : nullptr);
        if (std::find(validShapes.begin(), validShapes.end(), uint8_t(0)) != validShapes.end())
            throw std::logic_error("unprepared shape input or output storage");
        // Publish body-collider edges only after all disjoint descriptor writes join.
        for (const DescPtr<PhysxShapeDesc>& shape : out.shapes)
        {
            const auto body = bodyByKey.find(shape->rigidBody);
            if (body != bodyByKey.end())
                body->second->sourceCollisions.insert(shape->primKey);
        }
    }
    src.clearBucket();

    waveTimer.transition(stats ? &waveSeconds[6] : nullptr);
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

    // Consumer-registered custom joint prim types (REQ-PARSE-CORE-005). Mirrors
    // NativeWalker::jointPrimTypeToParse exactly: the typed schema variants win
    // on their schema bit, gear / rack win on their name, and the registry's only
    // effect there is to turn the eJointD6 fallback into eJointCustom. What the
    // native walker gets for free and this one does not is *reaching* a type the
    // list above does not name -- it visits every prim, while ovstage queries by
    // exact prim type -- so a registered type absent from the list needs its own
    // query (ADR-0002 open question 1).
    const std::vector<std::string> customJointTokens = parse::customTokens(parse::CustomTokenKind::eJoint);
    std::vector<JointType> jointTypes(std::begin(kJointTypes), std::end(kJointTypes));
    for (JointType& jt : jointTypes)
    {
        if (jt.type == eJointD6 &&
            parse::isCustomToken(parse::CustomTokenKind::eJoint, jt.typeName))
        {
            jt.type = eJointCustom;
        }
    }
    for (const std::string& jointToken : customJointTokens)
    {
        const bool alreadyQueried =
            std::any_of(std::begin(kJointTypes), std::end(kJointTypes),
                        [&](const JointType& jt) { return jointToken == jt.typeName; });
        if (!alreadyQueried)
            jointTypes.push_back(JointType{ jointToken.c_str(), eJointCustom });
    }

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
    struct JointFrameInput
    {
        Matrix4d relationshipWorld, bodyWorld;
        bool hasTarget = false, targetsBody = false;
    };
    auto captureJointFrame = [&](ObjectKey target, ObjectKey body)
    {
        JointFrameInput frame;
        frame.hasTarget = target.valid();
        if (!frame.hasTarget) return frame;
        src.getLocalToWorldTransform(target, frame.relationshipWorld);
        frame.targetsBody = body.valid() && src.canonicalPath(body) == src.canonicalPath(target);
        if (frame.targetsBody) frame.bodyWorld = frame.relationshipWorld;
        else if (body.valid()) src.getLocalToWorldTransform(body, frame.bodyWorld);
        return frame;
    };
    auto transformJointFrame = [](const JointFrameInput& frame, carb::Float3& position, carb::Float4& rotation) noexcept
    {
        ::physx::PxQuat q(rotation.x, rotation.y, rotation.z, rotation.w);
        q.normalize();
        rotation = {q.x, q.y, q.z, q.w};
        if (frame.hasTarget)
            omni::physics::transformJointFrameToBody(toPxMat44d(frame.relationshipWorld), toPxMat44d(frame.bodyWorld),
                frame.targetsBody, position, rotation);
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

    // Stage-wide joint sub-schema presence: one count-only membership query per family,
    // OR'd over every instance parseJoint could probe. Any applied instance anywhere
    // trips the flag, so skipping the per-joint probes is behaviour-preserving. Probed once,
    // on the first non-empty joint bucket: the flags are read only inside the joint loop, so a
    // stage without joints keeps the (conservative) defaults and pays nothing.
    bool jointSchemaPresenceProbed = false;
    auto probeJointSchemaPresence = [&]()
    {
        if (jointSchemaPresenceProbed)
            return;
        jointSchemaPresenceProbed = true;
        auto countOf = [](auto& arr) { return sizeof(arr) / sizeof(arr[0]); };
        static const char* const kLimit[] = {
            "PhysxLimitAPI:angular", "PhysxLimitAPI:linear", "PhysxLimitAPI:cone", "PhysxLimitAPI:distance",
            "PhysxLimitAPI:transX", "PhysxLimitAPI:transY", "PhysxLimitAPI:transZ",
            "PhysxLimitAPI:rotX", "PhysxLimitAPI:rotY", "PhysxLimitAPI:rotZ" };
        static const char* const kAxis[] = {
            "PhysxJointAxisAPI:angular", "PhysxJointAxisAPI:linear",
            "PhysxJointAxisAPI:rotX", "PhysxJointAxisAPI:rotY", "PhysxJointAxisAPI:rotZ" };
        static const char* const kEnv[] = {
            "PhysxDrivePerformanceEnvelopeAPI:angular", "PhysxDrivePerformanceEnvelopeAPI:linear",
            "PhysxDrivePerformanceEnvelopeAPI:rotX", "PhysxDrivePerformanceEnvelopeAPI:rotY",
            "PhysxDrivePerformanceEnvelopeAPI:rotZ" };
        static const char* const kState[] = {
            "PhysicsJointStateAPI:angular", "PhysicsJointStateAPI:linear",
            "PhysicsJointStateAPI:transX", "PhysicsJointStateAPI:transY", "PhysicsJointStateAPI:transZ",
            "PhysicsJointStateAPI:rotX", "PhysicsJointStateAPI:rotY", "PhysicsJointStateAPI:rotZ" };
        static const char* const kDrive[] = {
            "PhysicsDriveAPI:angular", "PhysicsDriveAPI:linear",
            "PhysicsDriveAPI:transX", "PhysicsDriveAPI:transY", "PhysicsDriveAPI:transZ",
            "PhysicsDriveAPI:rotX", "PhysicsDriveAPI:rotY", "PhysicsDriveAPI:rotZ", "PhysicsDriveAPI:distance" };
        static const char* const kDistApi[] = { "PhysxPhysicsDistanceJointAPI" };
        // An undeterminable answer keeps the fail-open default (probe per joint).
        JointSchemaPresence& present = ctx.jointSchemaPresence();
        auto known = [&](const char* const* names, size_t count, bool& flag)
        {
            if (const std::optional<bool> v = src.stageHasAnySchema(names, count))
                flag = *v;
        };
        known(kLimit, countOf(kLimit), present.physxLimit);
        known(kAxis, countOf(kAxis), present.physxJointAxis);
        known(kEnv, countOf(kEnv), present.drivePerfEnvelope);
        known(kState, countOf(kState), present.jointState);
        known(kDrive, countOf(kDrive), present.physicsDrive);
        known(kDistApi, countOf(kDistApi), present.physxDistanceJoint);
    };

    for (const JointType& jt : jointTypes)
    {
        DescriptorPhaseTimer jointSetupTimer(stats ? &readTimings.jointSetup : nullptr);
        JointTimings perType;
        double firstStructureSeconds = 0;
        DescriptorPhaseTimer jointDetailTimer(stats ? &perType.enumerate : nullptr);
        Bucket jointBucket = enumeratePrimTypeFiltered(jt.typeName, "physics:localPos0", readOrdinal);
        if (jointBucket.keys.empty())
        {
            src.clearBucket();
            continue;
        }
        jointDetailTimer.transition(stats ? &perType.probe : nullptr);
        probeJointSchemaPresence();
        const bool stageHasPhysicsDrive = ctx.jointSchemaPresence().physicsDrive;
        // Relationship topology is discovered serially. Numeric and token-valued
        // fields bind directly to retained rows after membership/storage is known.
        jointDetailTimer.transition(stats ? &perType.prefetch : nullptr);
        src.prefetchBucket(jointBucket.keys, {"physics:body0", "physics:body1", "physics:hinge0", "physics:hinge1", "physics:hinge", "physics:prismatic", "newton:mimicJoint"});
        jointDetailTimer.finish();
        jointSetupTimer.finish();
        DescriptorPhaseTimer topologyTimer(stats ? &readTimings.jointTopology : nullptr);
        DescriptorPhaseTimer jointPhase(stats ? &stats->prepareSeconds : nullptr);
        const KnownTokens& jointTokens = ctx.knownTokens();
        const JointReadTokens& jointNames = ctx.jointReadTokens();
        const JointColumnTokens jointFields{tokJointEnabled, tokCollisionEnabled, tokBreakForce, tokBreakTorque, tokExcludeArt,
            tokLocalPos0, tokLocalRot0, tokLocalPos1, tokLocalRot1, tokJointAxis, tokLowerLimit, tokUpperLimit,
            tokCone0, tokCone1, tokMinDist, tokMaxDist, tokAxisY, tokAxisZ, tokDriveAcceleration};
        std::vector<PreparedJoint> jointInputs;
        std::vector<std::array<JointFrameInput, 2>> jointFrames;
        jointInputs.reserve(jointBucket.keys.size());
        jointFrames.reserve(jointBucket.keys.size());
        for (const ObjectKey key : jointBucket.keys)
        {
            jointDetailTimer.transition(stats ? &perType.targets : nullptr);
            JointInfo info;
            info.type = jt.type;
            info.rel0 = firstTarget(key, tokBody0);
            info.rel1 = firstTarget(key, tokBody1);
            jointDetailTimer.transition(stats ? &perType.bodies : nullptr);
            info.body0 = info.rel0.valid() ? resolveBodyFromTarget(info.rel0) : ObjectKey{};
            info.body1 = info.rel1.valid() ? resolveBodyFromTarget(info.rel1) : ObjectKey{};
            jointDetailTimer.transition(stats ? &perType.frames : nullptr);
            jointFrames.push_back({captureJointFrame(info.rel0, info.body0), captureJointFrame(info.rel1, info.body1)});
            jointDetailTimer.transition(stats ? &perType.membership : nullptr);
            if (jt.type == eJointD6)
            {
                for (const JointAxis axis : {eTransX, eTransY, eTransZ, eRotX, eRotY, eRotZ, eDistance})
                {
                    const auto& names = jointNames.instances[size_t(axis)];
                    if (src.hasSchema(key, names.physicsLimitAPI)) info.jointLimits.push_back({axis, {}});
                    if (src.hasSchema(key, names.physicsDriveAPI))
                    {
                        JointDriveInfo drive;
                        drive.enabled = true;
                        info.jointDrives.push_back({axis, drive});
                    }
                }
            }
            else if ((jt.type == eJointRevolute || jt.type == eJointPrismatic) && stageHasPhysicsDrive)
                info.drive.enabled = src.hasSchema(key, jointNames.instances[jt.type == eJointRevolute ? 7 : 8].physicsDriveAPI);
            jointDetailTimer.transition(stats ? &perType.structure : nullptr);
            DescriptorPhaseTimer firstStructureTimer(stats && jointInputs.empty() ? &firstStructureSeconds : nullptr);
            jointInputs.push_back(prepareJointStructure(ctx, key, info));
        }
        jointDetailTimer.transition(stats ? &perType.declarations : nullptr);
        auto jointAttributes = jointFields.attributes(jt.type);
        std::vector<TokenId> jointTokenAttributes;
        if (jt.type == eJointRevolute || jt.type == eJointPrismatic || jt.type == eJointSpherical)
            jointTokenAttributes.push_back(tokJointAxis);
        appendJointAttributeTokens(jointTokens, jointNames, jointInputs, jointAttributes, jointTokenAttributes);
        jointDetailTimer.finish();
        topologyTimer.finish();
        DescriptorPhaseTimer columnTimer(stats ? &readTimings.columns : nullptr);
        const FrozenScalarBatch jointColumns = src.readScalarBatch(jointBucket.keys, jointAttributes.data(), jointAttributes.size(),
            jointTokenAttributes.data(), jointTokenAttributes.size());
        columnTimer.finish();
        if (stats)
        {
            jointDetail.enumerate += perType.enumerate;
            jointDetail.probe += perType.probe;
            jointDetail.prefetch += perType.prefetch;
            jointDetail.targets += perType.targets;
            jointDetail.bodies += perType.bodies;
            jointDetail.frames += perType.frames;
            jointDetail.membership += perType.membership;
            jointDetail.structure += perType.structure;
            jointDetail.declarations += perType.declarations;
            std::fprintf(stderr, "Descriptor joint type: type=%s keys=%zu columns=%zu enumerate=%.6f probe=%.6f prefetch=%.6f targets=%.6f bodies=%.6f frames=%.6f membership=%.6f structure=%.6f declarations=%.6f first_structure=%.6f\n",
                jt.typeName, jointBucket.keys.size(), jointAttributes.size(), perType.enumerate,
                perType.probe, perType.prefetch, perType.targets, perType.bodies, perType.frames,
                perType.membership, perType.structure, perType.declarations, firstStructureSeconds);
        }

        if (stats)
        {
            size_t bytes = jointInputs.capacity() * sizeof(PreparedJoint) + jointColumns.capacityBytes() +
                           jointFrames.capacity() * sizeof(std::array<JointFrameInput, 2>);
            for (const PreparedJoint& input : jointInputs)
                bytes += input.info.jointLimits.capacity() * sizeof(std::pair<JointAxis, JointLimitInfo>) +
                         input.info.jointDrives.capacity() * sizeof(std::pair<JointAxis, JointDriveInfo>);
            stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, bytes);
        }
        jointPhase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<PhysxJointDesc>> jointOutputs = allocateJointBatch(batches, jt.type, jointInputs.size());
        for (size_t i = 0; i < jointInputs.size(); ++i)
            prepareJointStorage(jointInputs[i], *jointOutputs[i]);
        std::vector<uint8_t> jointValid(jointInputs.size());
        jointPhase.finish();
        fillDescriptorRanges(jointInputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i)
            {
                auto& input = jointInputs[i];
                bool complete = true;
                readOvstageJointAttributes(jointColumns.row(i, complete), jointTokens, jointNames, jointFields, input);
                transformJointFrame(jointFrames[i][0], input.info.localPose0Position, input.info.localPose0Orientation);
                transformJointFrame(jointFrames[i][1], input.info.localPose1Position, input.info.localPose1Orientation);
                jointValid[i] = complete && fillJoint(input, *jointOutputs[i]);
            }
        });
        jointPhase.transition(stats ? &stats->finalizeSeconds : nullptr);
        for (size_t i = 0; i < jointInputs.size(); ++i)
        {
            if (!jointValid[i])
                throw std::logic_error("Incomplete prepared joint input or output storage");
            const ObjectKey key = jointInputs[i].key;
            const JointInfo& info = jointInputs[i].info;
            DescPtr<PhysxJointDesc>& joint = jointOutputs[i];

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

        DescriptorPhaseTimer tendonPhase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedMimicJoint> mimicInputs;
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
                prepareMimicJoints(ctx, jointKey, info, jointTypeOf, jointLimitOf, mimicInputs);
            if (newtonMimicAllowed)
                prepareNewtonMimicJoints(ctx, jointKey, info, jointTypeOf, mimicInputs);
        }

        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, mimicInputs.capacity() * sizeof(PreparedMimicJoint));
        tendonPhase.transition(stats ? &stats->constructSeconds : nullptr);
        out.mimicJoints = allocateDescriptorBatch<MimicJointDesc>(batches, mimicInputs.size());
        tendonPhase.finish();
        fillDescriptorRanges(mimicInputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) fillMimicJoint(mimicInputs[i], *out.mimicJoints[i]);
        });

        tendonPhase.transition(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedFixedTendon> fixedInputs;
        // parseFixedTendons probes every joint (one schema read each); skip the pass when no
        // prim applies a fixed-tendon axis schema. Membership, not authored attributes: an
        // applied instance with schema-default values is still a tendon.
        static const char* const kFixedTendonBases[] = { "PhysxTendonAxisRootAPI", "PhysxTendonAxisAPI" };
        if (src.stageHasAnySchema(kFixedTendonBases, sizeof(kFixedTendonBases) / sizeof(kFixedTendonBases[0]))
                .value_or(true))
        for (const ObjectKey& jointKey : jointOrder)
        {
            ScannedJointMap::const_iterator it = jointMap.find(jointKey);
            if (it == jointMap.end() || !it->second.desc)
                continue;
            FixedTendonParseInfo info;
            info.jointType = it->second.desc->type;
            info.body0 = it->second.desc->body0;
            info.body1 = it->second.desc->body1;
            prepareFixedTendons(ctx, jointKey, info, fixedInputs);
        }
        const size_t roots = size_t(std::count_if(fixedInputs.begin(), fixedInputs.end(), [](const PreparedFixedTendon& input) { return input.root; }));
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, mimicInputs.capacity() * sizeof(PreparedMimicJoint) + fixedInputs.capacity() * sizeof(PreparedFixedTendon));
        tendonPhase.transition(stats ? &stats->constructSeconds : nullptr);
        out.fixedTendonAxes = allocateDescriptorBatch<PhysxTendonAxisDesc>(batches, fixedInputs.size());
        out.fixedTendons = allocateDescriptorBatch<PhysxTendonFixedDesc>(batches, roots);
        std::vector<PhysxTendonFixedDesc*> rootSlots(fixedInputs.size(), nullptr);
        size_t rootIndex = 0;
        for (size_t i = 0; i < fixedInputs.size(); ++i)
            if (fixedInputs[i].root) rootSlots[i] = out.fixedTendons[rootIndex++].get();
        tendonPhase.finish();
        fillDescriptorRanges(fixedInputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i)
            {
                fillFixedTendonAxis(fixedInputs[i], *out.fixedTendonAxes[i]);
                if (rootSlots[i]) fillFixedTendonRoot(fixedInputs[i], *rootSlots[i]);
            }
        });
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
    // Spatial-tendon instance names are user-arbitrary, so collectMultiApplySchemaKeys needs
    // a whole-stage schema parse. Only emitted bodies can own the attachments we prepare
    // below, so an empty body set needs neither the presence probe nor that parse.
    if (!bodyByKey.empty() &&
        src.stageHasAnySchema(kSpatialTendonBases, kSpatialTendonBaseCount).value_or(true))
        for (const char* base : kSpatialTendonBases)
            src.collectMultiApplySchemaKeys(src.internToken(base), spatialTendonKeys);
    if (!spatialTendonKeys.empty())
    {
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedSpatialTendon> inputs;
        for (const auto& bodyEntry : bodyByKey)
        {
            if (!bodyEntry.second)
                continue;
            SpatialTendonParseInfo info;
            info.linkWorldScale = bodyEntry.second->scale;
            prepareSpatialTendons(ctx, bodyEntry.first, info, inputs);
        }
        size_t roots = 0, leaves = 0;
        for (const PreparedSpatialTendon& input : inputs)
        {
            roots += input.type == eTendonAttachmentRoot;
            leaves += input.type == eTendonAttachmentLeaf;
        }
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<PhysxTendonAttachmentDesc>> rootOwners = allocateDescriptorBatch<PhysxTendonSpatialDesc, PhysxTendonAttachmentDesc>(batches, roots);
        std::vector<DescPtr<PhysxTendonAttachmentDesc>> leafOwners = allocateDescriptorBatch<PhysxTendonAttachmentLeafDesc, PhysxTendonAttachmentDesc>(batches, leaves);
        std::vector<DescPtr<PhysxTendonAttachmentDesc>> middleOwners = allocateDescriptorBatch<PhysxTendonAttachmentDesc>(batches, inputs.size() - roots - leaves);
        out.spatialTendonAttachments.resize(inputs.size());
        size_t root = 0, leaf = 0, middle = 0;
        for (size_t i = 0; i < inputs.size(); ++i)
            out.spatialTendonAttachments[i] = inputs[i].type == eTendonAttachmentRoot ? std::move(rootOwners[root++]) :
                inputs[i].type == eTendonAttachmentLeaf ? std::move(leafOwners[leaf++]) : std::move(middleOwners[middle++]);
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) fillSpatialTendon(inputs[i], *out.spatialTendonAttachments[i]);
        });
    }

    waveTimer.transition(stats ? &waveSeconds[7] : nullptr);
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
    out.collisionGroups = allocateDescriptorBatch<CollisionGroupDesc>(batches, cgBucket.keys.size());
    for (size_t i = 0; i < cgBucket.keys.size(); ++i)
    {
        const ObjectKey key = cgBucket.keys[i];
        CollisionGroupInfo info = parseCollisionGroup(ctx, key);
        CollisionGroupDesc& desc = *out.collisionGroups[i];
        desc.primKey = key;
        desc.sourceFilteredGroups = std::move(info.filteredGroups);
        desc.sourceMembers = std::move(info.members);
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

    {
        DescriptorPhaseTimer timer(stats ? &stats->graphSeconds : nullptr);
        buildArticulations(src, batches, artRoots, out.bodies, out.joints, out.articulations);
    }

    PreparedOvstageVehicles vehicleInputs;
    auto scanBucket = [&](const Bucket& bucket, auto emit)
    {
        if (bucket.keys.empty())
        {
            src.clearBucket();
            return;
        }

        src.prefetchBucket(bucket.keys, bucket.attrs);
        for (const ObjectKey key : bucket.keys)
            emit(key);
        flushPreparedVehicles(vehicleInputs, out, batches, execution);
        src.clearBucket();
    };
    auto scanBucketWithTransform = [&](const Bucket& bucket, auto emit)
    {
        if (bucket.keys.empty())
        {
            src.clearBucket();
            return;
        }

        Bucket withTransforms = bucket;
        appendTransformAttrs(withTransforms);
        prefetchTransformAncestors(src, withTransforms.keys);
        prefetchTransformsForKeys(src, withTransforms.keys);
        src.prefetchBucket(bucket.keys, bucket.attrs);
        for (const ObjectKey key : withTransforms.keys)
            emit(key);
        flushPreparedVehicles(vehicleInputs, out, batches, execution);
        src.clearBucket();
    };

    waveTimer.transition(stats ? &waveSeconds[8] : nullptr);
    // --- Particles ---
    Bucket particleSystemBucket = enumeratePrimTypeFiltered("PhysxParticleSystem",
                                                           "particleSystemEnabled",
                                                           readOrdinal);
    appendBucketAttr(particleSystemBucket, "physics:simulationOwner");
    appendBucketAttr(particleSystemBucket, "physics:filteredPairs");
    src.prefetchBucket(particleSystemBucket.keys, particleSystemBucket.attrs);
    appendPreparedParticleSystems(out, ctx, batches, execution, particleSystemBucket.keys);
    src.clearBucket();

    Bucket particleSetBucket = enumerateSchemaFiltered("PhysxParticleSetAPI",
                                                     "physxParticle:particleSystem", readOrdinal);
    appendBucketAttr(particleSetBucket, "physxParticle:particleSystem");
    {
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedParticleSet> inputs;
        std::vector<ObjectKey> referencedSystems;
        std::unordered_set<ObjectKey, ObjectKey::Hash> emittedSystems;
        for (const DescPtr<ParticleSystemDesc>& system : out.particleSystems)
            emittedSystems.insert(src.canonicalKey(system->systemKey));
        scanBucket(particleSetBucket, [&](ObjectKey key)
        {
            PreparedParticleSet input = prepareParticleSet(ctx, key, false);
            if (!input.prepared) return;
            if (input.systemKey.valid() && src.exists(input.systemKey) &&
                emittedSystems.insert(src.canonicalKey(input.systemKey)).second)
                referencedSystems.push_back(input.systemKey);
            inputs.push_back(std::move(input));
        });
        phase.finish();
        appendPreparedParticleSystems(out, ctx, batches, execution, referencedSystems);
        phase.transition(stats ? &stats->prepareSeconds : nullptr);
        for (PreparedParticleSet& input : inputs)
            resolvePreparedParticleSetSystem(ctx, input);
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        out.particleSets = allocateDescriptorBatch<ParticleSetDesc>(batches, inputs.size());
        for (size_t i = 0; i < inputs.size(); ++i) prepareParticleSetStorage(inputs[i], *out.particleSets[i]);
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) fillParticleSet(inputs[i], *out.particleSets[i]);
        });
    }

    Bucket particleSamplerBucket = enumerateSchemaFiltered("PhysxParticleSamplingAPI",
                                                          "physxParticleSampling:samplingDistance", readOrdinal);
    appendBucketAttr(particleSamplerBucket, "physxParticleSampling:particles");
    {
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedParticleSampling> inputs;
        scanBucket(particleSamplerBucket, [&](ObjectKey key)
        {
            PreparedParticleSampling input = prepareParticleSampling(ctx, key);
            if (input.prepared) inputs.push_back(input);
        });
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        out.particleSamplers = allocateDescriptorBatch<ParticleSamplingDesc>(batches, inputs.size());
        out.particleSamplerKeys.resize(inputs.size());
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i)
            {
                fillParticleSampling(inputs[i], *out.particleSamplers[i]);
                out.particleSamplerKeys[i] = inputs[i].key;
            }
        });
    }

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
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedAttachment> inputs;
        scanBucket(attachmentBucket, [&](ObjectKey key)
        {
            PreparedAttachment input = prepareAttachment(ctx, key, subtype);
            if (input.valid) inputs.push_back(input);
        });
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        std::vector<DescPtr<PhysxDeformableAttachmentDesc>> outputs =
            allocateDescriptorBatch<PhysxDeformableAttachmentDesc>(batches, inputs.size());
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i)
            {
                fillAttachment(inputs[i], *outputs[i]);
                outputs[i]->primKey = inputs[i].key;
            }
        });
        for (DescPtr<PhysxDeformableAttachmentDesc>& desc : outputs)
            out.attachments.push_back(std::move(desc));
    }

    const Bucket filterBucket = enumeratePrimTypeFiltered("OmniPhysicsElementCollisionFilter",
                                                          "omniphysics:filterEnabled",
                                                          readOrdinal);
    {
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedAttachment> inputs;
        scanBucket(filterBucket, [&](ObjectKey key)
        {
            PreparedAttachment input = prepareElementCollisionFilter(ctx, key);
            if (input.valid) inputs.push_back(input);
        });
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        out.deformableCollisionFilters = allocateDescriptorBatch<PhysxDeformableCollisionFilterDesc>(batches, inputs.size());
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i)
            {
                fillElementCollisionFilter(inputs[i], *out.deformableCollisionFilters[i]);
                out.deformableCollisionFilters[i]->primKey = inputs[i].key;
            }
        });
    }

    // --- Character controllers ---
    // Peer of NativeWalker's ApiFlag::eCharacterControllerAPI dispatch. Needs the
    // resolved world transform (position + scale), so it rides scanBucketWithTransform.
    Bucket cctBucket = enumerateSchemaFiltered("PhysxCharacterControllerAPI",
                                               "physxCharacterController:slopeLimit", readOrdinal);
    appendBucketAttrs(cctBucket, {
        "physxCharacterController:slopeLimit",
        "physxCharacterController:simulationOwner",
        "radius",
        "height",
    });
    {
        DescriptorPhaseTimer phase(stats ? &stats->prepareSeconds : nullptr);
        std::vector<PreparedCct> inputs;
        scanBucketWithTransform(cctBucket, [&](ObjectKey key) { prepareOvstageCct(inputs, src, ctx, key); });
        if (stats) stats->preparedPeakBytes = std::max(stats->preparedPeakBytes, descriptor_memory::nested(inputs));
        phase.transition(stats ? &stats->constructSeconds : nullptr);
        out.ccts = allocateDescriptorBatch<CapsuleCctDesc>(batches, inputs.size());
        phase.finish();
        fillDescriptorRanges(inputs.size(), execution, [&](size_t begin, size_t end) noexcept
        {
            for (size_t i = begin; i < end; ++i) fillCct(inputs[i], *out.ccts[i]);
        });
    }

    waveTimer.transition(stats ? &waveSeconds[9] : nullptr);
    // --- Vehicles: scene context + tire friction tables ---
    // Descriptors in their own right (a vehicle may reference them later): always scanned.
    const Bucket vehCtxBucket = enumerateSchemaFiltered("PhysxVehicleContextAPI",
                                                "physxVehicleContext:updateMode", readOrdinal);
    scanBucket(vehCtxBucket, [&](ObjectKey key) { emitVehicleContext(vehicleInputs, src, ctx, key); });

    const Bucket tftBucket = enumeratePrimTypeFiltered("PhysxVehicleTireFrictionTable",
                                                       "frictionValues",
                                                       readOrdinal);
    scanBucket(tftBucket, [&](ObjectKey key) { emitTireFrictionTable(vehicleInputs, src, ctx, key); });

    // The ~16 component / vehicle enumerates below are each an ovstage query even when
    // empty; one count query over the whole vehicle schema family gates them. Every name
    // here is concrete (single-apply or qualified), so arbitrary-instance discovery would
    // read unrelated render schemas without adding a possible match. Unknown → scan.
    static const char* const kVehicleFamilySchemas[] = {
        "PhysxVehicleAPI", "PhysxVehicleWheelAPI", "PhysxVehicleTireAPI", "PhysxVehicleSuspensionAPI",
        "PhysxVehicleEngineAPI", "PhysxVehicleGearsAPI", "PhysxVehicleClutchAPI", "PhysxVehicleDriveBasicAPI",
        "PhysxVehicleDriveStandardAPI", "PhysxVehicleTankDifferentialAPI", "PhysxVehicleMultiWheelDifferentialAPI",
        "PhysxVehicleAutoGearBoxAPI", "PhysxVehicleBrakesAPI:brakes0", "PhysxVehicleBrakesAPI:brakes1",
        "PhysxVehicleSteeringAPI", "PhysxVehicleAckermannSteeringAPI",
        "PhysxVehicleNonlinearCommandResponseAPI:drive", "PhysxVehicleNonlinearCommandResponseAPI:steer",
        "PhysxVehicleNonlinearCommandResponseAPI:brakes0", "PhysxVehicleNonlinearCommandResponseAPI:brakes1",
        "PhysxVehicleWheelAttachmentAPI", "PhysxVehicleSuspensionComplianceAPI" };
    if (src.stageHasAnySchema(kVehicleFamilySchemas, sizeof(kVehicleFamilySchemas) / sizeof(kVehicleFamilySchemas[0]),
                              /*includeMultiApplyInstances=*/false)
            .value_or(true))
    {
    // --- Vehicles: shareable wheel/tire/suspension components ---
    const Bucket wheelBucket = enumerateSchemaFiltered("PhysxVehicleWheelAPI",
                                               "physxVehicleWheel:radius", readOrdinal);
    scanBucket(wheelBucket, [&](ObjectKey key) { emitVehicleWheel(vehicleInputs, ctx, key); });

    const Bucket tireBucket = enumerateSchemaFiltered("PhysxVehicleTireAPI",
                                              "physxVehicleTire:lateralStiffnessGraph", readOrdinal);
    scanBucket(tireBucket, [&](ObjectKey key) { emitVehicleTire(vehicleInputs, instance, src, ctx, key, massScale); });

    const Bucket suspensionBucket = enumerateSchemaFiltered("PhysxVehicleSuspensionAPI",
                                                    "physxVehicleSuspension:springStrength", readOrdinal);
    scanBucket(suspensionBucket, [&](ObjectKey key) { emitVehicleSuspension(vehicleInputs, ctx, key); });

    // --- Vehicles: drivetrain components ---
    const Bucket engineBucket = enumerateSchemaFiltered("PhysxVehicleEngineAPI",
                                                "physxVehicleEngine:peakTorque", readOrdinal);
    scanBucket(engineBucket, [&](ObjectKey key) { emitVehicleEngine(vehicleInputs, src, ctx, key, kgmsScale); });

    const Bucket gearsBucket = enumerateSchemaFiltered("PhysxVehicleGearsAPI",
                                               "physxVehicleGears:ratios", readOrdinal);
    scanBucket(gearsBucket, [&](ObjectKey key) { emitVehicleGears(vehicleInputs, src, ctx, key); });

    const Bucket clutchBucket = enumerateSchemaFiltered("PhysxVehicleClutchAPI",
                                                "physxVehicleClutch:strength", readOrdinal);
    scanBucket(clutchBucket, [&](ObjectKey key) { emitVehicleClutch(vehicleInputs, ctx, key, kgmsScale); });

    const Bucket driveBasicBucket = enumerateSchemaFiltered("PhysxVehicleDriveBasicAPI",
                                                    "physxVehicleDriveBasic:peakTorque", readOrdinal);
    scanBucket(driveBasicBucket, [&](ObjectKey key) { emitVehicleDriveBasic(vehicleInputs, ctx, key, kgmsScale); });

    const Bucket driveStandardBucket = enumerateSchemaFiltered("PhysxVehicleDriveStandardAPI",
                                                       "physxVehicleDriveStandard:engine", readOrdinal);
    scanBucket(driveStandardBucket, [&](ObjectKey key) { emitVehicleDriveStandard(vehicleInputs, src, ctx, key); });

    const Bucket tankDiffBucket = enumerateSchemaFiltered("PhysxVehicleTankDifferentialAPI",
                                                  "physxVehicleTankDifferential:numberOfWheelsPerTrack", readOrdinal);
    scanBucket(tankDiffBucket, [&](ObjectKey key) { emitVehicleTankDifferential(vehicleInputs, src, ctx, key); });

    const Bucket multiDiffBucket = enumerateSchemaFiltered("PhysxVehicleMultiWheelDifferentialAPI",
                                                   "physxVehicleMultiWheelDifferential:wheels", readOrdinal);
    const TokenId tokTankDiffApi = src.internToken("PhysxVehicleTankDifferentialAPI");
    scanBucket(multiDiffBucket, [&](ObjectKey key)
    {
        if (!src.hasSchema(key, tokTankDiffApi))
            emitVehicleMultiWheelDifferential(vehicleInputs, src, ctx, key);
    });

    const Bucket autoGearBucket = enumerateSchemaFiltered("PhysxVehicleAutoGearBoxAPI",
                                                  "physxVehicleAutoGearBox:upRatios", readOrdinal);
    scanBucket(autoGearBucket, [&](ObjectKey key) { emitVehicleAutoGearBox(vehicleInputs, src, ctx, key); });

    // --- Vehicles: brakes, steering, nonlinear response ---
    const char* const brakeSchemas[] = { "PhysxVehicleBrakesAPI:brakes0", "PhysxVehicleBrakesAPI:brakes1" };
    const Bucket brakesBucket = enumerateSchemasFiltered(brakeSchemas, 2,
                                                 "physxVehicleBrakes:brakes0:maxBrakeTorque", readOrdinal);
    scanBucket(brakesBucket, [&](ObjectKey key) { emitVehicleBrakes(vehicleInputs, src, ctx, key); });

    const Bucket steeringBucket = enumerateSchemaFiltered("PhysxVehicleSteeringAPI",
                                                  "physxVehicleSteering:wheels", readOrdinal);
    scanBucket(steeringBucket, [&](ObjectKey key) { emitVehicleSteeringBasic(vehicleInputs, src, ctx, key); });

    const Bucket ackermannBucket = enumerateSchemaFiltered("PhysxVehicleAckermannSteeringAPI",
                                                   "physxVehicleAckermannSteering:wheel0", readOrdinal);
    const TokenId tokSteeringApi = src.internToken("PhysxVehicleSteeringAPI");
    scanBucket(ackermannBucket, [&](ObjectKey key)
    {
        if (!src.hasSchema(key, tokSteeringApi))
            emitVehicleSteeringAckermann(vehicleInputs, ctx, key);
    });

    const char* const ncrSchemas[] = {
        "PhysxVehicleNonlinearCommandResponseAPI:drive",
        "PhysxVehicleNonlinearCommandResponseAPI:steer",
        "PhysxVehicleNonlinearCommandResponseAPI:brakes0",
        "PhysxVehicleNonlinearCommandResponseAPI:brakes1",
    };
    const Bucket ncrBucket = enumerateSchemasFiltered(ncrSchemas, 4,
                                              "physxVehicleNonlinearCommandResponse:drive:commandValues", readOrdinal);
    scanBucket(ncrBucket, [&](ObjectKey key) { emitVehicleNonlinearCmdResponse(vehicleInputs, src, ctx, key); });

    // --- Vehicles: wheel attachments, suspension compliance, chassis roots ---
    const Bucket wheelAttachmentBucket = enumerateSchemaFiltered("PhysxVehicleWheelAttachmentAPI",
                                                         "physxVehicleWheelAttachment:index", readOrdinal);
    scanBucket(wheelAttachmentBucket, [&](ObjectKey key) { emitVehicleWheelAttachment(vehicleInputs, instance, src, ctx, key); });

    const Bucket suspensionComplianceBucket = enumerateSchemaFiltered("PhysxVehicleSuspensionComplianceAPI",
                                                              "physxVehicleSuspensionCompliance:suspensionForceAppPoint",
                                                              readOrdinal);
    scanBucket(suspensionComplianceBucket, [&](ObjectKey key) { emitVehicleSuspensionCompliance(vehicleInputs, src, ctx, key); });

    const Bucket vehicleBucket = enumerateSchemaFiltered("PhysxVehicleAPI",
                                                 "physxVehicle:vehicleEnabled", readOrdinal);
    scanBucketWithTransform(vehicleBucket, [&](ObjectKey key) { emitVehicle(vehicleInputs, src, ctx, key, lengthScale); });
    }

    waveTimer.finish();
    scanTimer.finish();
    if (stats)
    {
        stats->coordinatorOtherSeconds = std::max(0.0, stats->scanSeconds - stats->prepareSeconds -
            stats->constructSeconds - stats->fillAndJoinSeconds - stats->finalizeSeconds - stats->graphSeconds);
        const DescriptorBatchStats pool = batches.stats();
        stats->outputMemberCapacityBytes = descriptorMemberCapacityBytes(out);
        stats->logicalRequests = pool.requests;
        stats->backingAllocations = pool.backingAllocations;
        stats->poolLiveBytes = pool.liveBytes;
        stats->poolPeakBytes = pool.peakBytes;
        stats->poolRetainedBytes = pool.retainedBytes;
        stats->poolSlackBytes = pool.slackBytes;
        std::fprintf(stderr, "Descriptor reads: shape_setup=%.6f shape_classify=%.6f shape_transforms=%.6f shape_relations=%.6f joint_setup=%.6f joint_topology=%.6f columns=%.6f\n",
            readTimings.shapeSetup, readTimings.shapeClassify, readTimings.shapeTransforms, readTimings.shapeRelations,
            readTimings.jointSetup, readTimings.jointTopology, readTimings.columns);
        std::fprintf(stderr, "Descriptor waves: init=%.6f scenes=%.6f materials=%.6f bodies=%.6f deformables=%.6f shapes=%.6f joints_dependents=%.6f groups_graph=%.6f particles_attachments=%.6f vehicles=%.6f\n",
            waveSeconds[0], waveSeconds[1], waveSeconds[2], waveSeconds[3], waveSeconds[4],
            waveSeconds[5], waveSeconds[6], waveSeconds[7], waveSeconds[8], waveSeconds[9]);
        auto printSetup = [](const char* family, const SetupTimings& timing, size_t keys)
        {
            std::fprintf(stderr, "Descriptor setup %s: keys=%zu enumerate=%.6f transform_ancestors=%.6f relationship_ancestors=%.6f transforms=%.6f metadata=%.6f attributes=%.6f columns=%.6f inputs=%.6f\n",
                family, keys, timing.enumerate, timing.transformAncestors, timing.relationshipAncestors,
                timing.transforms, timing.metadata, timing.attributes, timing.columns, timing.inputs);
        };
        std::fprintf(stderr, "Descriptor joint details: enumerate=%.6f probe=%.6f prefetch=%.6f targets=%.6f bodies=%.6f frames=%.6f membership=%.6f structure=%.6f declarations=%.6f\n",
            jointDetail.enumerate, jointDetail.probe, jointDetail.prefetch, jointDetail.targets,
            jointDetail.bodies, jointDetail.frames, jointDetail.membership, jointDetail.structure, jointDetail.declarations);
        std::fprintf(stderr, "Descriptor shape relations: subsets=%.6f material=%.6f filters=%.6f owners=%.6f\n",
            shapeRelationSeconds[0], shapeRelationSeconds[1], shapeRelationSeconds[2], shapeRelationSeconds[3]);
        printSetup("bodies", bodySetup, bodyBucket.keys.size());
        printSetup("shapes", shapeSetup, shapeBucket.keys.size());
        if (execution.reportStatistics) execution.reportStatistics(*stats);
    }
    return out;
}

} // namespace omni::physics::ovstage
