// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-28
 */
/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-3 AC-5 AC-7
 *
 * @implements REQ-PARSE-INSTANCER-001
 * @covers AC-3 AC-4 AC-5
 *
 * @implements REQ-PARSE-INSTANCER-003
 * @covers AC-1 AC-3
 */

#include "PointInstancer.h"

#include <limits>
#include <unordered_set>

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "Particles.h"
#include "Collision.h"
#include "PhysicsBody.h"
#include "Material.h"
#include "Mass.h"
#include "CollisionGroup.h"
#include "IceDescriptorAllocator.h"

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ScanBackend.h>
#include <omni/physics/parse/ScannedStage.h>
#include <PhysXTools.h>
#include <OmniPhysX.h>

#include <foundation/PxBitMap.h>

#include "ScannedShapeCookingDispatch.h"

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

using omni::physics::parse::ObjectKey;

// Out-of-line ctor/dtor/move for TargetDesc -- required because the
// unique_ptr<ScannedStage> field forward-declares ScannedStage in
// the header; defaulted definitions live here where the full type
// is visible via the ScannedStage.h include above.
TargetDesc::TargetDesc() : desc(nullptr), outsideInstancer(false), hasProtoTransformInverse(false) {}
TargetDesc::~TargetDesc() = default;
TargetDesc::TargetDesc(TargetDesc&&) noexcept = default;
TargetDesc& TargetDesc::operator=(TargetDesc&&) noexcept = default;

bool isOutsideInstancer(const omni::physics::parse::IPhysicsSource* src,
                        omni::physics::parse::ObjectKey primKey,
                        omni::physics::parse::ObjectKey instancerKey)
{
    if (!src)
        return true;
    for (omni::physics::parse::ObjectKey k = primKey; k.valid(); k = src->getParent(k))
    {
        if (k == instancerKey)
            return false;
    }
    return true;
}

// Parse materials authored under the point-instancer's children and
// register them with the attached stage via a per-child scanStage call.
static void parseInstancerMaterials(AttachedStage& attachedStage, ObjectKey instancerKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;

    // Enumerate the instancer's direct children via the source and scan each
    // child subtree through the backend-dispatched scan path.
    std::vector<ObjectKey> children;
    src->forEachChild(instancerKey, [&children](ObjectKey c) { children.push_back(c); });
    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScanOptions scanOptions;
    scanOptions.prunePointInstancerDescendants = false;
    for (const ObjectKey childKey : children)
    {
        const std::vector<std::string> scanRoots{ std::string(attachedStage.textViewFor(childKey)) };
        omni::physics::parse::ScannedStage scanned = omni::physics::parse::scanStage(
            attachedStage.attachTarget(), scanRoots, kNoExclude, scanOptions,
            omni::physx::usdparser::iceDescriptorAllocator());

        for (auto& matDesc : scanned.materials)
        {
            // Re-key the material's scan-space key into attachedStage-space via the
            // scanned source's own string identity (the same opaque-identity round trip
            // ScannedShapeCookingDispatch.cpp's ObjectKey-native functions use).
            const std::string_view pathText = scanned.source().sourceKeyToString(matDesc->materialKey);
            const ObjectKey key = attachedStage.keyFor(pathText);
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(
                attachedStage, key, *matDesc);
            if (id != kInvalidObjectId)
            {
                attachedStage.getObjectDatabase()->findOrCreateEntry(key, pathText, matDesc->type, id);
            }
            // matDesc auto-frees via unique_ptr at end of iteration.
        }
    }
}

void parsePrototype(AttachedStage& attachedStage,
                    ObjectKey targetKey,
                    CollisionPairVector& filteredPairs,
                    const ObjectInstance& objectInstance,
                    bool collectShapes, TargetDesc& out)
{
    // Drive prototype parsing via the parse-library scanStage —
    // scanStage produces typed parse-lib shape / body descs; this
    // function runs the consumer-side registration (parent-walk for
    // instanced bodies, static-body synthesis, in-line createShape vs.
    // shapeDescVector accumulation, etc.).
    //
    // Lifetime: TargetDesc::scannedStage owns the ScannedStage so its
    // non-owning data references (MergeMeshPhysxShapeDesc::mergedMesh
    // pointers, source-side ObjectKey lookups via the scan source) stay
    // valid through the per-instance scale + cook step in
    // parseRigidBodyInstancer.

    const std::vector<std::string> scanRoots{ std::string(attachedStage.textViewFor(targetKey)) };
    static const std::vector<std::string> kNoExclude;
    omni::physics::parse::ScanOptions scanOptions;
    scanOptions.prunePointInstancerDescendants = false;
    auto scanned = std::make_unique<omni::physics::parse::ScannedStage>();
    *scanned = omni::physics::parse::scanStage(attachedStage.attachTarget(), scanRoots, kNoExclude, scanOptions,
                                                omni::physx::usdparser::iceDescriptorAllocator());
    if (!scanned->sourcePtr())
    {
        // targetKey exists on the source (callers only reach here after
        // src->exists(targetKey)), but the scan itself came back empty -- no
        // usable prototype data. Leave `out` at its default (desc == nullptr)
        // rather than dereferencing the sourceless ScannedStage.
        out.scannedStage = std::move(scanned);
        return;
    }
    const omni::physics::parse::IPhysicsSource& scanSrc = scanned->source();

    PhysxObjectDesc* protoDesc = nullptr;
    ObjectKey protoKey;
    ObjectIdVector shapeIds;

    // Pass 1 — shapes.  scanStage iterates depth-first (matches the
    // legacy loadFromRange dispatch order), so children precede their
    // parent body, which means shapeIds accumulates fully before the
    // body branch runs.
    for (auto& shapeUPtr : scanned->shapes)
    {
        PhysxShapeDesc* desc = shapeUPtr.get();
        const ObjectKey key = attachedStage.keyFor(scanSrc.sourceKeyToString(desc->primKey));

        // Cooking dispatch — fills crc/meshKey on mesh types via the
        // cooking service.  No-op for simple shapes.
        scan::dispatchScannedShapeCooking(attachedStage, *scanned, desc);
        // Re-key ObjectKey fields from the scanStage source's namespace
        // into the attachedStage source's namespace.  scanStage's
        // internal source mints its own ObjectKeys; downstream legacy
        // helpers resolve via the attachedStage source, so translate at
        // the boundary (via the source's string identity) so the desc
        // is usable across the two namespaces.
        if (desc->rigidBody.valid())
            desc->rigidBody = attachedStage.keyFor(scanSrc.sourceKeyToString(desc->rigidBody));
        if (desc->sourceGprim.valid())
            desc->sourceGprim = attachedStage.keyFor(scanSrc.sourceKeyToString(desc->sourceGprim));
        // Mesh-cooking subclasses each carry a `meshPrimKey` key the
        // runtime uses to locate the source mesh prim.  Each subclass
        // (ConvexMesh / TriangleMesh / ...) declares its own meshPrimKey
        // field (name-shadowed across MergeMesh hierarchy), so cast
        // to the specific subclass for the translation.
        if (desc->type == eConvexMeshShape)
        {
            auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(desc);
            if (d->meshPrimKey.valid())
                d->meshPrimKey = attachedStage.keyFor(scanSrc.sourceKeyToString(d->meshPrimKey));
        }
        else if (desc->type == eTriangleMeshShape ||
                 desc->type == eConvexMeshDecompositionShape ||
                 desc->type == eSpherePointsShape)
        {
            auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(desc);
            if (d->meshPrimKey.valid())
                d->meshPrimKey = attachedStage.keyFor(scanSrc.sourceKeyToString(d->meshPrimKey));
        }

        // Translate the parse-lib desc's source-side lists to legacy
        // fields the registration layer reads.  Returns false in the
        // "no resolvable scene" case fillPhysxShapeDesc gates on.
        std::vector<ObjectKey> materials;
        if (!scan::resolveConsumerSideShapeState(attachedStage, *scanned, desc,
                                                  materials, filteredPairs))
            continue;

        // collisionGroup — ObjectDatabase lookup (consumer-side state
        // the parse library can't model).  Matches fillPhysxShapeDesc
        // line 331.
        desc->collisionGroup = getCollisionGroup(attachedStage, key);

        finalizeShape(attachedStage, desc, materials);

        // Parent-walk: detect a rigid body applied on a prim above the
        // shape.
        bool shouldCreateShape = true;
        if (!desc->rigidBody.valid())
        {
            bool bodyInstanced = false;
            // Walk the instancer's ancestors (inclusive) for a rigid body — source
            // routed (no UsdPrim); matches the legacy `!= GetPseudoRoot()` loop.
            const omni::physics::parse::IPhysicsSource* psrc = attachedStage.getSource();
            if (psrc)
            {
                const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
                const ObjectKey rootKey = psrc->getRootKey();
                for (ObjectKey parentKey = objectInstance.instancerKey;
                     parentKey.valid() && parentKey != rootKey; parentKey = psrc->getParent(parentKey))
                {
                    if (psrc->hasSchema(parentKey, tok.physicsRigidBodyAPI))
                    {
                        bodyInstanced = true;
                        desc->rigidBody = parentKey;
                        bool resetXformStack;
                        const ::physx::PxMat44d mat = internal::getLocalTransform(
                            attachedStage, key,
                            omni::physics::parse::ReadTime::at(std::numeric_limits<double>::lowest()), resetXformStack);
                        ::physx::PxTransform pose;
                        ::physx::PxVec3 scale;
                        decomposeMatrix(pose, scale, mat);
                        desc->localPos = toFloat3(pose.p);
                        desc->localRot = toFloat4(pose.q);
                        desc->localScale = toFloat3(scale);
                        break;
                    }
                }
            }
            if (bodyInstanced)
                shouldCreateShape = false;
        }

        if (shouldCreateShape)
        {
            if (collectShapes)
            {
                if (!desc->rigidBody.valid())
                {
                    PhysxRigidBodyDesc* bodyDesc = createStaticBody();
                    bodyDesc->position = desc->localPos;
                    bodyDesc->rotation = desc->localRot;
                    bodyDesc->scale = desc->localScale;

                    desc->localPos   = { 0.0f, 0.0f, 0.0f };
                    desc->localRot   = { 0.0f, 0.0f, 0.0f, 1.0f };
                    desc->localScale = { 1.0f, 1.0f, 1.0f };

                    protoDesc = bodyDesc;
                }
                out.shapeDescVector.push_back(std::make_pair(key, desc));
            }
            else
            {
                if (!desc->rigidBody.valid())
                {
                    if (!protoDesc || protoDesc->type != eStaticBody)
                    {
                        PhysxRigidBodyDesc* rbDesc = createStaticBody();
                        bool resetXformStack;
                        const ::physx::PxMat44d mat = internal::getLocalTransform(
                            attachedStage, targetKey,
                            omni::physics::parse::ReadTime::at(std::numeric_limits<double>::lowest()), resetXformStack);
                        ::physx::PxTransform pose;
                        ::physx::PxVec3 scale;
                        decomposeMatrix(pose, scale, mat);
                        rbDesc->position = toFloat3(pose.p);
                        rbDesc->rotation = toFloat4(pose.q);
                        rbDesc->scale = toFloat3(scale);
                        rbDesc->sceneIds = desc->sceneIds;

                        protoDesc = rbDesc;
                    }

                    CARB_ASSERT(protoDesc->type == eStaticBody);
                    PhysxRigidBodyDesc* rbDesc = static_cast<PhysxRigidBodyDesc*>(protoDesc);

                    // carb-typed overload: the Gf one this replaced was a pure
                    // re-type of these same three values (LoadTools.cpp), and the
                    // GfVec3ToFloat3 / GfQuatToFloat4 calls that followed undid it
                    // lane for lane. Writing the descriptor fields directly is
                    // bit-identical and drops the round trip.
                    getCollisionShapeLocalTransform(attachedStage, key,
                        targetKey, desc->localPos, desc->localRot,
                        desc->localScale);

                    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createShape(
                        key, *desc, kInvalidObjectId, &objectInstance);
                    attachedStage.getObjectDatabase()->findOrCreateEntry(key, attachedStage.textViewFor(key), eShape, id);
                    rbDesc->shapes.push_back(id);
                }
                else
                {
                    ObjectId shapeId = kInvalidObjectId;
                    protoDesc = createShape(attachedStage, key, desc, &objectInstance, &shapeId);
                    shapeIds.push_back(shapeId);
                }
            }
        }
        else
        {
            protoDesc = desc;
            protoKey = key;
        }
    }

    // Pass 2 — rigid body.  In a well-formed prototype subtree there's
    // at most one body emit (the prototype root).  If one is present
    // it overrides whatever protoDesc shape-pass synthesized.
    for (auto& bodyUPtr : scanned->bodies)
    {
        PhysxRigidBodyDesc* rbDesc = bodyUPtr.get();

        // sceneIds translation — sourceSimulationOwners → eScene
        // ObjectIds (ObjectDatabase lookup is consumer state).
        rbDesc->sceneIds.clear();
        for (const auto& sk : rbDesc->sourceSimulationOwners)
        {
            // Re-key from the scanStage source's key-space into attachedStage's,
            // then use the ObjectDb's ObjectKey-typed findEntry overload.
            const ObjectKey ownerKey = attachedStage.keyFor(scanSrc.sourceKeyToString(sk));
            if (!ownerKey.valid())
                continue;
            const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(ownerKey, eScene);
            if (entry != kInvalidObjectId)
                rbDesc->sceneIds.push_back(entry);
        }

        // filteredCollisions on bodies map to filteredPairs pairs (matches
        // parseRigidBody's accumulation).  rbDesc->primKey and each fk are
        // still keyed in the *scanned*-stage's own key space (minted by the
        // scan-time source), so -- like the sceneIds re-keying just above,
        // and the sibling fix in ScannedShapeCookingDispatch.cpp's
        // resolveConsumerSideShapeState() (REQ-PARSE-CONSUMER-001 AC-5) --
        // they must round-trip through the scan source's string identity
        // before landing in filteredPairs (a CollisionPairVector, consumed
        // downstream by createFilteredPairs() via attachedStage-space keys).
        // Left unresolved, the pair silently disappears in
        // createFilteredPairs() and the point-instancer prototype rigid
        // body's FilteredPairsAPI never reaches PhysX.
        const ObjectKey bodyKey = attachedStage.keyFor(scanSrc.sourceKeyToString(rbDesc->primKey));
        for (const auto& fk : rbDesc->sourceFilteredCollisions)
        {
            if (!fk.valid())
                continue;
            const std::string_view targetStr = scanSrc.sourceKeyToString(fk);
            if (targetStr.empty())
                continue;
            filteredPairs.push_back(std::make_pair(bodyKey, attachedStage.keyFor(targetStr)));
        }

        protoDesc = rbDesc;
        protoKey = bodyKey;
        if (!collectShapes && rbDesc)
        {
            rbDesc->shapes.clear();
            rbDesc->shapes = shapeIds;
        }
        // The legacy listener stops at the first body emit; subsequent
        // bodies in the subtree are ignored.  Match that.
        break;
    }

    out.desc = protoDesc;
    out.descKey = protoKey;
    out.scannedStage = std::move(scanned);
}

void parseRigidBodyInstancer(AttachedStage& attachedStage,
                             ObjectKey instancerKey,
                             CollisionPairVector& filteredPairs)
{
    // Diagnostics-only path text for the CARB_LOG_WARN calls below; textFor()
    // avoids materializing a path just to format a log string.
    const char* instancerText = attachedStage.textFor(instancerKey);

    // parse the objects below point instancer that we need, like materials, those dont belong to a prototype
    parseInstancerMaterials(attachedStage, instancerKey);

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();

    const ::physx::PxMat44d instancerMatrix = internal::getWorldTransform(attachedStage, instancerKey);
    const ::physx::PxMat44d instancerMatrixInverse = affineInverse(instancerMatrix);
    // Bit-exact GfTransform::GetScale() (gfmath::decomposeWithPivot), not the
    // PhysX-native polar getScale: instancerMatrix is a PointInstancer world
    // transform, so ancestor non-uniform scale plus rotation can shear it, and
    // the two decompositions disagree on that input (see MatrixTools.h). This
    // feeds bodyDesc->scale below, matching the Gf convention already used for
    // instancerActorMatrixInverse/protoWorld in this function.
    const gfmath::PivotTransform instancerXf = gfmath::decomposeWithPivot(instancerMatrix);
    const ::physx::PxVec3 sc(float(instancerXf.scale.x), float(instancerXf.scale.y), float(instancerXf.scale.z));
    // Bit-exact Gf::RemoveScaleShear() (gfmath::removeScaleShearGf), not the
    // PhysX-native removeScaleShear: this is a PointInstancer world transform,
    // so ancestor non-uniform scale plus rotation can shear it, and the two
    // decompositions disagree on that input (see MatrixTools.h). Feeds
    // cacheTargetPrototypeTransform below, and mirrors InternalActor.cpp's
    // fallback computation of the same quantity for an instance that reaches
    // it without a cached protoTransformInverse.
    const ::physx::PxMat44d instancerActorMatrixInverse = affineInverse(gfmath::removeScaleShearGf(
        internal::getWorldTransform(attachedStage, instancerKey, omni::physics::parse::ReadTime::defaultTime())));

    // prototypes relationship -> target prim keys, already in attachedStage's own
    // source key-space (getRelationshipTargets is read directly off `src`, the same
    // source attachedStage owns), so no re-key round trip is needed here -- unlike
    // parsePrototype's scanned-subtree descs below, which mint keys in a fresh,
    // separate scan source and must round-trip through that source's string identity.
    std::vector<ObjectKey> targets;
    TargetDescVector targetObjects;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (src)
        src->getRelationshipTargets(instancerKey, tok.prototypes, targets);

    // Prefer the default value, fall back to the earliest time sample
    // (mirrors getAttributeArrayTimedFallback) — all via the source.
    const auto readArray = [&](omni::physics::parse::TokenId attr, auto& out) -> bool
    {
        return internal::getArrayValue(attachedStage, instancerKey, attr,
                                        omni::physics::parse::ReadTime::defaultTime(), out) ||
               // std::numeric_limits<double>::lowest() is UsdTimeCode::EarliestTime()'s
               // exact underlying value (pxr/usd/usd/timeCode.h); ReadTime::at() round-trips
               // it straight back into UsdTimeCode(t) on the USD backend, so this is the
               // same earliest-time read without needing the pxr timeCode header here.
               internal::getArrayValue(attachedStage, instancerKey, attr,
                                        omni::physics::parse::ReadTime::at(std::numeric_limits<double>::lowest()),
                                        out);
    };

    // these attributes are required and must match in length
    std::vector<int32_t> indices;
    if (!readArray(tok.protoIndices, indices))
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) indices array not valid\n", instancerText);
    }

    if (indices.size() == 0)
    {
        return;
    }

    std::vector<carb::Float3> positions;
    if (!readArray(tok.positions, positions) || positions.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) positions array not valid\n", instancerText);
    }

    // these attributes are optional, but must match in length with 'indices' if they are defined.
    // carb::Float4 also serves the on-disk GfQuath storage -- PhysXTools.h's fillArray
    // dispatches on the source BufferElemType (eQuath) at runtime and widens half->float,
    // it is not a memcpy.
    std::vector<carb::Float4> orientations;
    if (readArray(tok.orientations, orientations) && orientations.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) orientations array size does not match instance count\n",
                      instancerText);
    }

    std::vector<carb::Float3> velocities;
    readArray(tok.velocities, velocities);
    if (velocities.size() > 0 && velocities.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) velocities defined but size does not match instance count\n",
                      instancerText);
    }

    std::vector<carb::Float3> angularVelocities;
    readArray(tok.angularVelocities, angularVelocities);
    if (angularVelocities.size() > 0  && angularVelocities.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) angularVelocities defined but size does not match instance count\n",
                      instancerText);
    }


    // attributes that aren't currently supported go here
    std::vector<carb::Float3> scales;
    if (readArray(tok.scales, scales) && scales.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) scales array size does not match instance count\n",
                      instancerText);
    }

    // support shared shapes only if we dont have scales provided. For scaled instances we cant share shapes
    const bool sharedShapes = scales.empty();

    auto cacheTargetPrototypeTransform = [&](TargetDesc& targetDesc, ObjectKey targetKey)
    {
        if (!src || !src->exists(targetKey))
            return;

        // As above: bit-exact Gf decomposition to match instancerActorMatrixInverse.
        const ::physx::PxMat44d protoWorld = gfmath::removeScaleShearGf(
            internal::getWorldTransform(attachedStage, targetKey, omni::physics::parse::ReadTime::defaultTime()));
        // Was, in Gf (row-vector) order: (protoWorld * instancerActorMatrixInverse).GetInverse().
        // Gf A * B is PhysX B * A on the same sixteen doubles, so the operands swap.
        targetDesc.protoTransformInverse =
            internal::toParseMatrix4d(affineInverse(instancerActorMatrixInverse * protoWorld));
        targetDesc.hasProtoTransformInverse = true;
    };

    std::vector<int64_t> inactiveItemsVector;
    if (src)
        src->getInactiveInstanceIds(instancerKey, inactiveItemsVector);
    ::physx::PxBitMap inactiveIds;
    ::physx::PxBitMap activeTargets;
    if (!inactiveItemsVector.empty())
    {
        // we dont want to parse all targets, only those used.
        activeTargets.resize(::physx::PxU32(targets.size()));
        inactiveIds.resize(::physx::PxU32(indices.size()));
        for (size_t i = 0; i < inactiveItemsVector.size(); i++)
        {
            // The source hands back instance positions, but they originate in backend data
            // (the USD `inactiveIds` metadata) and can name instances that do not exist.
            // PxBitMap::set() only asserts on range, so an out of range entry would corrupt the heap.
            if (inactiveItemsVector[i] < 0 || size_t(inactiveItemsVector[i]) >= indices.size())
            {
                CARB_LOG_WARN("Physics:PointInstancer: (%s) inactive instance %lld is out of range, ignoring\n",
                              instancerText, (long long)inactiveItemsVector[i]);
                continue;
            }

            inactiveIds.set(::physx::PxU32(inactiveItemsVector[i]));
        }

        // if target is used set the bit for activeTargets
        for (size_t index = 0; index < indices.size(); index++)
        {
            // protoIndices can be out of range - the instance loop below skips those - so the bit
            // must not be set here either, for the same reason as above.
            if (size_t(indices[index]) >= targets.size())
            {
                continue;
            }

            if (!inactiveIds.test(::physx::PxU32(index)))
            {
                activeTargets.set(::physx::PxU32(indices[index]));
            }
        }
    }

    // traverse bitmap only if needed, its slower
    if (!inactiveItemsVector.empty())
    {
        // targetObjects is indexed by indices[i] below, which is an index into `targets`. Only the
        // active targets get parsed here, so the entries have to be placed at their target index
        // instead of appended - appending would compact the vector and make every later
        // targetObjects[indices[i]] read past its end.
        targetObjects.resize(targets.size());

        ::physx::PxBitMap::Iterator it(activeTargets);
        for (::physx::PxU32 index = it.getNext(); index != ::physx::PxBitMap::Iterator::DONE; index = it.getNext())
        {
            ObjectInstance objectInstance = { instancerKey, index, omni::physics::parse::ObjectKey{}, false };
            const ObjectKey targetKey = targets[index];
            if (src && src->exists(targetKey))
            {
                // the unparsed targets keep their default constructed entry, which has desc == nullptr
                TargetDesc& targetDesc = targetObjects[index];
                parsePrototype(attachedStage, targetKey, filteredPairs, objectInstance,
                    !sharedShapes, targetDesc);
                targetDesc.outsideInstancer = isOutsideInstancer(src, targetKey, instancerKey);
                cacheTargetPrototypeTransform(targetDesc, targetKey);
            }
        }
    }
    else
    {
        for (size_t i = 0; i < targets.size(); i++)
        {
            ObjectInstance objectInstance = { instancerKey, (uint32_t)i, omni::physics::parse::ObjectKey{}, false };
            const ObjectKey targetKey = targets[i];
            if (src && src->exists(targetKey))
            {
                TargetDesc targetDesc;
                parsePrototype(attachedStage, targetKey, filteredPairs, objectInstance,
                    !sharedShapes, targetDesc);
                targetDesc.outsideInstancer = isOutsideInstancer(src, targetKey, instancerKey);
                cacheTargetPrototypeTransform(targetDesc, targetKey);
                targetObjects.push_back(std::move(targetDesc));
            }
            else
            {
                targetObjects.emplace_back();
            }
        }
    }

    ObjectKey topBodyKey;    // invalid until the shared top-level body is resolved (no UsdPrim)
    ::physx::PxMat44d topBodyMatrixInverse(::physx::PxIdentity);

    for (size_t i = 0; i < indices.size(); i++)
    {
        if (size_t(indices[i]) >= targets.size())
            continue;

        // A.B. we might consider traversing the BitMap here instead, though in most cases it might not be faster
        if (!inactiveItemsVector.empty())
        {
            if (inactiveIds.test(::physx::PxU32(i)))
            {
                continue; // skip inactive instances
            }
        }

        ObjectInstance objectInstance = { instancerKey, (uint32_t)i, targets[indices[i]], false };
        const TargetDesc& targetDesc = targetObjects[indices[i]];
        if (targetDesc.hasProtoTransformInverse)
        {
            objectInstance.protoTransformInverse = targetDesc.protoTransformInverse;
            objectInstance.hasProtoTransformInverse = true;
        }
        PhysxObjectDesc* objectDesc = targetDesc.desc;

        if (!objectDesc)
            continue;

        if (objectDesc->type == eStaticBody || objectDesc->type == eDynamicBody)
        {
            PhysxRigidBodyDesc* sourceBodyDesc = static_cast<PhysxRigidBodyDesc*>(objectDesc);

            // toPhysXQuat(GfQuath) is the same four floats the old
            // toPhysX(GfQuatf(GfQuath)) produced -- both read GetImaginary()/GetReal()
            // and GfHalf->float is exact -- so this is not a widening, just one hop
            // fewer. GfQuatf(1.0f) was the identity (real 1, imaginary 0).
            const ::physx::PxVec3 instancePos =
                i < positions.size() ? toPhysX(positions[i]) : ::physx::PxVec3(0.0f);
            const ::physx::PxQuat instanceOrient = i < orientations.size() ?
                                                       toPhysXQuat(orientations[i]) :
                                                       ::physx::PxQuat(::physx::PxIdentity);
            const carb::Float3 instanceScale =
                i < scales.size() ? scales[i] : carb::Float3{ 1.0f, 1.0f, 1.0f };

            const ShapeDescVector& shapesDescs = targetObjects[indices[i]].shapeDescVector;
            // create shapes if scaling is used
            if (!shapesDescs.empty())
            {
                sourceBodyDesc->shapes.clear();
                for (size_t shapeIndex = 0; shapeIndex < shapesDescs.size(); shapeIndex++)
                {
                    const ObjectKey shapeKey = shapesDescs[shapeIndex].first;
                    const PhysxShapeDesc& shapeDesc = *shapesDescs[shapeIndex].second;
                    PhysxShapeDesc* scaledShapeDesc = scaleShapeDesc(shapeDesc, instanceScale);
                    if (scaledShapeDesc)
                    {
                        ObjectId shapeId = kInvalidObjectId;
                        objectInstance.isExclusive = true;
                        createShape(attachedStage, shapeKey, scaledShapeDesc, &objectInstance, &shapeId);
                        if (shapeId != kInvalidObjectId)
                            sourceBodyDesc->shapes.push_back(shapeId);
                    }
                }
            }


            // Gf order was: localBody * instancerMatrixInverse, then
            // localBody * instanceMatrix * instancerMatrix. Gf A * B is PhysX B * A,
            // so each product is written with the operands reversed here.
            ::physx::PxMat44d localBodyMatrix =
                makeMatrix(toPhysX(sourceBodyDesc->position, sourceBodyDesc->rotation));
            localBodyMatrix = instancerMatrixInverse * localBodyMatrix;

            localBodyMatrix.setPosition(localBodyMatrix.getPosition().multiply(
                ::physx::PxVec3d(instanceScale.x, instanceScale.y, instanceScale.z)));

            const ::physx::PxMat44d instanceMatrix =
                makeMatrix(::physx::PxTransform(instancePos, instanceOrient));

            const ::physx::PxMat44d bodyMatrix = instancerMatrix * instanceMatrix * localBodyMatrix;

            PhysxRigidBodyDesc* bodyDesc = nullptr;
            if (sourceBodyDesc->type == eDynamicBody)
            {
                bodyDesc = ICE_PLACEMENT_NEW(DynamicPhysxRigidBodyDesc)();
                DynamicPhysxRigidBodyDesc* dynamicBody = (DynamicPhysxRigidBodyDesc*)bodyDesc;
                *dynamicBody = *(DynamicPhysxRigidBodyDesc*)sourceBodyDesc;

                // degToRad(PxVec3) applies the same float scalar as degToRad(GfVec3f).
                dynamicBody->angularVelocity =
                    i < angularVelocities.size() ?
                        toFloat3(degToRad(toPhysX(angularVelocities[i]))) :
                        carb::Float3{ 0.0f, 0.0f, 0.0f };

                ::physx::PxVec3d transformedVelocity(0.0);
                if (i < velocities.size())
                    transformedVelocity = ::physx::PxVec3d(velocities[i].x, velocities[i].y, velocities[i].z);
                if (dynamicBody->localSpaceVelocities)
                {
                    // GfMatrix4d::Transform is a point transform (translation included);
                    // PxMat44::transform matches it.
                    transformedVelocity = instancerMatrix.transform(transformedVelocity);
                }
                dynamicBody->linearVelocity = carb::Float3{ float(transformedVelocity.x), float(transformedVelocity.y),
                                                            float(transformedVelocity.z) };
            }
            else
            {
                bodyDesc = ICE_PLACEMENT_NEW(StaticPhysxRigidBodyDesc)();
                StaticPhysxRigidBodyDesc* staticBody = (StaticPhysxRigidBodyDesc*)bodyDesc;
                *staticBody = *(StaticPhysxRigidBodyDesc*)sourceBodyDesc;
            }

            const ::physx::PxTransform bodyPose = toTransform(bodyMatrix);
            bodyDesc->position = toFloat3(bodyPose.p);
            bodyDesc->scale = toFloat3(sc);
            bodyDesc->rotation = toFloat4(bodyPose.q);

            const ObjectId pointInstancerBodyId = attachedStage.getObjectDatabase()->findEntry(instancerKey, ePointInstancedBody);
            if (pointInstancerBodyId == kInvalidObjectId)
            {
                PointInstancedBodyDesc piDesc;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, instancerKey, piDesc, nullptr);
                attachedStage.getObjectDatabase()->findOrCreateEntry(instancerKey, instancerText, ePointInstancedBody, id);
            }

            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, targets[indices[i]], *bodyDesc, &objectInstance);
            attachedStage.getObjectDatabase()->findOrCreateEntry(
                targets[indices[i]], attachedStage.textViewFor(targets[indices[i]]), eBody, id);
            bodyDesc->shapes.clear();

            ICE_FREE(bodyDesc);
        }
        else if (objectDesc->type > eShape && objectDesc->type < eBody)
        {
            // separate shapes, that do belong to a top level already created body
            const PhysxShapeDesc* sourceShapeDesc = static_cast<PhysxShapeDesc*>(objectDesc);

            if (!topBodyKey.valid())
            {
                topBodyKey = sourceShapeDesc->rigidBody;
                CARB_ASSERT(topBodyKey.valid());

                topBodyMatrixInverse =
                    affineInverse(internal::getWorldTransform(attachedStage, sourceShapeDesc->rigidBody));
            }

            const ::physx::PxVec3 instancePos =
                i < positions.size() ? toPhysX(positions[i]) : ::physx::PxVec3(0.0f);
            const ::physx::PxQuat instanceOrient = i < orientations.size() ?
                                                       toPhysXQuat(orientations[i]) :
                                                       ::physx::PxQuat(::physx::PxIdentity);
            const carb::Float3 instanceScale =
                i < scales.size() ? scales[i] : carb::Float3{ 1.0f, 1.0f, 1.0f };

            ::physx::PxMat44d localShapeMatrix =
                internal::getWorldTransform(attachedStage, targetObjects[indices[i]].descKey);

            // Do this only if the object is below the instancer
            // (Gf order was localShapeMatrix * instancerMatrixInverse; operands swap.)
            if (!targetObjects[indices[i]].outsideInstancer)
                localShapeMatrix = instancerMatrixInverse * localShapeMatrix;

            const ::physx::PxMat44d instanceMatrix =
                makeMatrix(::physx::PxTransform(instancePos, instanceOrient));

            // Gf: localShapeMatrix * instanceMatrix * instancerMatrix, then
            // shapeWorldMatrix * topBodyMatrixInverse -- each product reversed here.
            const ::physx::PxMat44d shapeWorldMatrix = instancerMatrix * instanceMatrix * localShapeMatrix;
            const ::physx::PxMat44d newLocalShapeMatrix = topBodyMatrixInverse * shapeWorldMatrix;

            PhysxShapeDesc* scaledShapeDesc = scaleShapeDesc(*sourceShapeDesc, instanceScale);

            const ::physx::PxTransform newLocalShapePose = toTransform(newLocalShapeMatrix);
            scaledShapeDesc->localPos = toFloat3(newLocalShapePose.p);
            scaledShapeDesc->localRot = toFloat4(newLocalShapePose.q);

            const ObjectId pointInstancerBodyId = attachedStage.getObjectDatabase()->findEntry(instancerKey, ePointInstancedBody);
            if (pointInstancerBodyId == kInvalidObjectId)
            {
                PointInstancedBodyDesc piDesc;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, instancerKey, piDesc, nullptr);
                attachedStage.getObjectDatabase()->findOrCreateEntry(instancerKey, instancerText, ePointInstancedBody, id);
            }

            objectInstance.isExclusive = true;
            createShape(attachedStage, targetObjects[indices[i]].descKey, scaledShapeDesc, &objectInstance);
        }
    }

    // Shape descs in shapeDescVector are owned by each TargetDesc's
    // ScannedStage (raw pointers into the parse-library snapshot).
    // The unique_ptr<ScannedStage> in TargetDesc cleans up on
    // targetObjects vector destruction.  No explicit free needed.

    for (size_t i = 0; i < targets.size(); i++)
    {
        // bufferRequestRigidBodyMassUpdate is ObjectKey-native; targets[i] is
        // already in attachedStage's own key-space (see the re-key comment above).
        attachedStage.bufferRequestRigidBodyMassUpdate(targets[i]);
    }

    if (topBodyKey.valid())
    {
        attachedStage.bufferRequestRigidBodyMassUpdate(topBodyKey);
    }
}
} // namespace usdparser
} // namespace physx
} // namespace omni
