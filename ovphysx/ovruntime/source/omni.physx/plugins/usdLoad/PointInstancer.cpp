// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-5 AC-7
 */

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <omni/physics/usd/PrimIterator.h>
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
#include "PointInstancer.h"
#include "IceDescriptorAllocator.h"

#include <PhysXTools.h>
#include <OmniPhysX.h>

#include <foundation/PxBitMap.h>

#include <omni/physics/usd/StageScan.h>

#include "ScannedShapeCookingDispatch.h"

using namespace PXR_NS;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

// Out-of-line ctor/dtor/move for TargetDesc — required because the
// unique_ptr<ScannedStage> field forward-declares ScannedStage in
// the header; defaulted definitions live here where the full type
// is visible via the StageScan.h include above.
TargetDesc::TargetDesc() : desc(nullptr), outsideInstancer(false), hasProtoTransformInverse(false) {}
TargetDesc::~TargetDesc() = default;
TargetDesc::TargetDesc(TargetDesc&&) noexcept = default;
TargetDesc& TargetDesc::operator=(TargetDesc&&) noexcept = default;

static omni::physics::parse::Matrix4d toParseMatrix4d(const PXR_NS::GfMatrix4d& matrix)
{
    omni::physics::parse::Matrix4d out;
    const double* src = matrix.GetArray();
    for (size_t i = 0; i < 16; ++i)
        out.data[i] = src[i];
    return out;
}

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
static void parseInstancerMaterials(AttachedStage& attachedStage, omni::physics::parse::ObjectKey instancerKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;

    // Enumerate the instancer's direct children via the source and scan each
    // child subtree through the backend-dispatched scan path.
    std::vector<omni::physics::parse::ObjectKey> children;
    src->forEachChild(instancerKey, [&children](omni::physics::parse::ObjectKey c) { children.push_back(c); });
    static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
    omni::physics::parse::ScanOptions scanOptions;
    scanOptions.prunePointInstancerDescendants = false;
    for (const omni::physics::parse::ObjectKey childKey : children)
    {
        const std::vector<SdfPath> scanRoots{ attachedStage.pathFor(childKey) };
        omni::physics::usd::ScannedStage scanned = omni::physics::usd::scanStage(
            attachedStage.attachTarget(), scanRoots, kNoExclude,
            omni::physx::usdparser::iceDescriptorAllocator(), scanOptions);

        for (auto& matDesc : scanned.materials)
        {
            const PXR_NS::SdfPath path = scanned.pathFor(matDesc->materialKey);
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(
                attachedStage, path, *matDesc);
            if (id != kInvalidObjectId)
            {
                attachedStage.getObjectDatabase()->findOrCreateEntry(path, matDesc->type, id);
            }
            // matDesc auto-frees via unique_ptr at end of iteration.
        }
    }
}

void parsePrototype(AttachedStage& attachedStage,
                    const SdfPath& targetPath,
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
    // pointers, source-side ObjectKey lookups via pathFor) stay valid
    // through the per-instance scale + cook step in
    // parseRigidBodyInstancer.

    const std::vector<SdfPath> scanRoots{ targetPath };
    static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
    omni::physics::parse::ScanOptions scanOptions;
    scanOptions.prunePointInstancerDescendants = false;
    auto scanned = std::make_unique<omni::physics::usd::ScannedStage>(
        omni::physics::usd::scanStage(attachedStage.attachTarget(), scanRoots, kNoExclude,
                                      omni::physx::usdparser::iceDescriptorAllocator(), scanOptions));

    PhysxObjectDesc* protoDesc = nullptr;
    SdfPath protoPath;
    ObjectIdVector shapeIds;

    // Pass 1 — shapes.  scanStage iterates depth-first (matches the
    // legacy loadFromRange dispatch order), so children precede their
    // parent body, which means shapeIds accumulates fully before the
    // body branch runs.
    for (auto& shapeUPtr : scanned->shapes)
    {
        PhysxShapeDesc* desc = shapeUPtr.get();
        const SdfPath path = scanned->pathFor(desc->primKey);

        // Cooking dispatch — fills crc/meshKey on mesh types via the
        // cooking service.  No-op for simple shapes.
        scan::dispatchScannedShapeCooking(attachedStage, *scanned, desc);
        // Re-key ObjectKey fields from the scanStage source's namespace
        // into the attachedStage source's namespace.  scanStage's
        // internal UsdSource mints its own ObjectKeys; downstream
        // legacy helpers resolve via `attachedStage.pathFor()` which
        // uses a different intern table.  Translate at the boundary so
        // the desc is usable across the two namespaces.
        if (desc->rigidBody.valid())
            desc->rigidBody = attachedStage.keyFor(scanned->pathFor(desc->rigidBody));
        if (desc->sourceGprim.valid())
            desc->sourceGprim = attachedStage.keyFor(scanned->pathFor(desc->sourceGprim));
        // Mesh-cooking subclasses each carry a `meshPrimKey` key the
        // runtime uses to locate the source mesh prim.  Each subclass
        // (ConvexMesh / TriangleMesh / ...) declares its own meshPrimKey
        // field (name-shadowed across MergeMesh hierarchy), so cast
        // to the specific subclass for the translation.
        if (desc->type == eConvexMeshShape)
        {
            auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(desc);
            if (d->meshPrimKey.valid())
                d->meshPrimKey = attachedStage.keyFor(scanned->pathFor(d->meshPrimKey));
        }
        else if (desc->type == eTriangleMeshShape ||
                 desc->type == eConvexMeshDecompositionShape ||
                 desc->type == eSpherePointsShape)
        {
            auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(desc);
            if (d->meshPrimKey.valid())
                d->meshPrimKey = attachedStage.keyFor(scanned->pathFor(d->meshPrimKey));
        }

        // Translate the parse-lib desc's source-side lists to legacy
        // fields the registration layer reads.  Returns false in the
        // "no resolvable scene" case fillPhysxShapeDesc gates on.
        SdfPathVector materials;
        if (!scan::resolveConsumerSideShapeState(attachedStage, *scanned, desc,
                                                  materials, filteredPairs))
            continue;

        // collisionGroup — ObjectDatabase lookup (consumer-side state
        // the parse library can't model).  Matches fillPhysxShapeDesc
        // line 331.
        desc->collisionGroup = getCollisionGroup(attachedStage, path);

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
                const omni::physics::parse::TokenId rbApiTok =
                    internal::schemaTypeToken<UsdPhysicsRigidBodyAPI>(*psrc);
                const omni::physics::parse::ObjectKey rootKey = psrc->getRootKey();
                for (omni::physics::parse::ObjectKey parentKey = attachedStage.keyFor(objectInstance.instancerPath);
                     parentKey.valid() && parentKey != rootKey; parentKey = psrc->getParent(parentKey))
                {
                    if (psrc->hasSchema(parentKey, rbApiTok))
                    {
                        bodyInstanced = true;
                        desc->rigidBody = parentKey;
                        bool resetXformStack;
                        const GfMatrix4d mat = internal::getLocalTransform(attachedStage, attachedStage.keyFor(path), UsdTimeCode::EarliestTime(), resetXformStack);
                        const GfTransform tr(mat);
                        GfVec3ToFloat3(tr.GetTranslation(), desc->localPos);
                        GfQuatToFloat4(tr.GetRotation().GetQuat(), desc->localRot);
                        GfVec3ToFloat3(tr.GetScale(), desc->localScale);
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
                out.shapeDescVector.push_back(std::make_pair(path, desc));
            }
            else
            {
                if (!desc->rigidBody.valid())
                {
                    if (!protoDesc || protoDesc->type != eStaticBody)
                    {
                        PhysxRigidBodyDesc* rbDesc = createStaticBody();
                        bool resetXformStack;
                        const GfMatrix4d mat = internal::getLocalTransform(attachedStage, attachedStage.keyFor(targetPath), UsdTimeCode::EarliestTime(), resetXformStack);
                        const GfTransform tr(mat);
                        GfVec3ToFloat3(tr.GetTranslation(), rbDesc->position);
                        GfQuatToFloat4(tr.GetRotation().GetQuat(), rbDesc->rotation);
                        GfVec3ToFloat3(tr.GetScale(), rbDesc->scale);
                        rbDesc->sceneIds = desc->sceneIds;

                        protoDesc = rbDesc;
                    }

                    CARB_ASSERT(protoDesc->type == eStaticBody);
                    PhysxRigidBodyDesc* rbDesc = static_cast<PhysxRigidBodyDesc*>(protoDesc);

                    GfVec3f localPos, localScale;
                    GfQuatf localRot;
                    getCollisionShapeLocalTransform(attachedStage, attachedStage.keyFor(path),
                        attachedStage.keyFor(targetPath), localPos, localRot, localScale);
                    GfVec3ToFloat3(localPos, desc->localPos);
                    GfQuatToFloat4(localRot, desc->localRot);
                    GfVec3ToFloat3(localScale, desc->localScale);

                    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createShape(
                        path, *desc, kInvalidObjectId, &objectInstance);
                    attachedStage.getObjectDatabase()->findOrCreateEntry(path, eShape, id);
                    rbDesc->shapes.push_back(id);
                }
                else
                {
                    ObjectId shapeId = kInvalidObjectId;
                    protoDesc = createShape(attachedStage, path, desc, &objectInstance, &shapeId);
                    shapeIds.push_back(shapeId);
                }
            }
        }
        else
        {
            protoDesc = desc;
            protoPath = path;
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
            const SdfPath sp = scanned->pathFor(sk);
            if (sp.IsEmpty())
                continue;
            const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(sp, eScene);
            if (entry != kInvalidObjectId)
                rbDesc->sceneIds.push_back(entry);
        }

        // filteredCollisions on bodies map to filteredPairs pairs
        // (matches parseRigidBody's accumulation).
        const SdfPath bodyPath = scanned->pathFor(rbDesc->primKey);
        for (const auto& fk : rbDesc->sourceFilteredCollisions)
        {
            const SdfPath fp = scanned->pathFor(fk);
            if (!fp.IsEmpty())
                filteredPairs.push_back(std::make_pair(bodyPath, fp));
        }

        protoDesc = rbDesc;
        protoPath = bodyPath;
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
    out.descPath = protoPath;
    out.scannedStage = std::move(scanned);
}

void parseRigidBodyInstancer(AttachedStage& attachedStage,
                                                          const SdfPath& instancerPath,
                                                          CollisionPairVector& filteredPairs)
{

    // parse the objects below point instancer that we need, like materials, those dont belong to a prototype
    parseInstancerMaterials(attachedStage, attachedStage.keyFor(instancerPath));

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey instancerKey = attachedStage.keyFor(instancerPath);

    const GfMatrix4d instancerMatrix = internal::getWorldTransform(attachedStage, instancerKey);
    const GfMatrix4d instancerMatrixInverse = instancerMatrix.GetInverse();
    const GfVec3d sc = GfTransform(instancerMatrix).GetScale();
    const GfMatrix4d instancerActorMatrixInverse =
        internal::getWorldTransform(attachedStage, instancerKey, UsdTimeCode::Default()).RemoveScaleShear().GetInverse();

    // prototypes relationship -> target prim paths (source key-space -> SdfPath)
    SdfPathVector targets;
    TargetDescVector targetObjects;
    if (src)
    {
        std::vector<omni::physics::parse::ObjectKey> protoKeys;
        src->getRelationshipTargets(instancerKey, src->internToken(UsdGeomTokens->prototypes.GetString()), protoKeys);
        targets.reserve(protoKeys.size());
        for (const auto& k : protoKeys)
            targets.push_back(attachedStage.pathFor(k));
    }

    // Prefer the default value, fall back to the earliest time sample
    // (mirrors getAttributeArrayTimedFallback) — all via the source.
    const auto readArray = [&](const PXR_NS::TfToken& attr, auto& out) -> bool
    {
        return internal::getArrayValue(attachedStage, instancerKey, attr, UsdTimeCode::Default(), out) ||
               internal::getArrayValue(attachedStage, instancerKey, attr, UsdTimeCode::EarliestTime(), out);
    };

    // these attributes are required and must match in length
    VtArray<int> indices;
    if (!readArray(UsdGeomTokens->protoIndices, indices))
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) indices array not valid\n", instancerPath.GetText());
    }

    if (indices.size() == 0)
    {
        return;
    }

    VtArray<GfVec3f> positions;
    if (!readArray(UsdGeomTokens->positions, positions) || positions.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) positions array not valid\n", instancerPath.GetText());
    }

    // these attributes are optional, but must match in length with 'indices' if they are defined
    VtArray<GfQuath> orientations;
    if (readArray(UsdGeomTokens->orientations, orientations) && orientations.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) orientations array size does not match instance count\n",
                      instancerPath.GetText());
    }

    VtArray<GfVec3f> velocities;
    readArray(UsdGeomTokens->velocities, velocities);
    if (velocities.size() > 0 && velocities.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) velocities defined but size does not match instance count\n",
                      instancerPath.GetText());
    }

    VtArray<GfVec3f> angularVelocities;
    readArray(UsdGeomTokens->angularVelocities, angularVelocities);
    if (angularVelocities.size() > 0  && angularVelocities.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) angularVelocities defined but size does not match instance count\n",
                      instancerPath.GetText());
    }


    // attributes that aren't currently supported go here
    VtArray<GfVec3f> scales;
    if (readArray(UsdGeomTokens->scales, scales) && scales.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) scales array size does not match instance count\n",
                      instancerPath.GetText());
    }

    // support shared shapes only if we dont have scales provided. For scaled instances we cant share shapes
    const bool sharedShapes = scales.empty();

    auto cacheTargetPrototypeTransform = [&](TargetDesc& targetDesc, const SdfPath& targetPath)
    {
        const omni::physics::parse::ObjectKey targetKey = attachedStage.keyFor(targetPath);
        if (!src || !src->exists(targetKey))
            return;

        const GfMatrix4d protoWorld =
            internal::getWorldTransform(attachedStage, targetKey, UsdTimeCode::Default()).RemoveScaleShear();
        targetDesc.protoTransformInverse = toParseMatrix4d((protoWorld * instancerActorMatrixInverse).GetInverse());
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
            inactiveIds.set(::physx::PxU32(inactiveItemsVector[i]));
        }

        // if target is used set the bit for activeTargets        
        for (size_t index = 0; index < indices.size(); index++)
        {
            if (!inactiveIds.test(::physx::PxU32(index)))
            {
                activeTargets.set(::physx::PxU32(indices[index]));
            }            
        }
    }

    // traverse bitmap only if needed, its slower
    if (!inactiveItemsVector.empty())
    {
        ::physx::PxBitMap::Iterator it(activeTargets);
        for (::physx::PxU32 index = it.getNext(); index != ::physx::PxBitMap::Iterator::DONE; index = it.getNext())
        {
            ObjectInstance objectInstance = { instancerPath, index, SdfPath(), false };
            TargetDesc targetDesc;
            const SdfPath& targetPath = targets[index];
            if (src && src->exists(attachedStage.keyFor(targetPath)))
            {
                parsePrototype(attachedStage, targetPath, filteredPairs, objectInstance,
                    !sharedShapes, targetDesc);
                targetDesc.outsideInstancer = isOutsideInstancer(src, attachedStage.keyFor(targetPath), instancerKey);
                cacheTargetPrototypeTransform(targetDesc, targetPath);
                targetObjects.push_back(std::move(targetDesc));
            }
            else
            {
                targetObjects.emplace_back();
            }
        }
    }
    else
    {
        for (size_t i = 0; i < targets.size(); i++)
        {
            ObjectInstance objectInstance = { instancerPath, (uint32_t)i, SdfPath(), false };
            const SdfPath& targetPath = targets[i];
            if (src && src->exists(attachedStage.keyFor(targetPath)))
            {
                TargetDesc targetDesc;
                parsePrototype(attachedStage, targetPath, filteredPairs, objectInstance,
                    !sharedShapes, targetDesc);
                targetDesc.outsideInstancer = isOutsideInstancer(src, attachedStage.keyFor(targetPath), instancerKey);
                cacheTargetPrototypeTransform(targetDesc, targetPath);
                targetObjects.push_back(std::move(targetDesc));
            }
            else
            {
                targetObjects.emplace_back();
            }
        }
    }

    SdfPath topBodyPath;    // empty until the shared top-level body is resolved (no UsdPrim)
    GfMatrix4d topBodyMatrixInverse;

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

        ObjectInstance objectInstance = { instancerPath, (uint32_t)i, targets[indices[i]], false };
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

            const GfVec3f instancePos = i < positions.size() ? positions[i] : GfVec3f(0.0f);
            const GfQuatf instanceOrient = i < orientations.size() ? GfQuatf(orientations[i]) : GfQuatf(1.0f);
            const GfVec3f instanceScale = i < scales.size() ? scales[i] : GfVec3f(1.0f);

            const ShapeDescVector& shapesDescs = targetObjects[indices[i]].shapeDescVector;
            // create shapes if scaling is used
            if (!shapesDescs.empty())
            {
                sourceBodyDesc->shapes.clear();
                for (size_t shapeIndex = 0; shapeIndex < shapesDescs.size(); shapeIndex++)
                {
                    const SdfPath& shapeKey = shapesDescs[shapeIndex].first;
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
            

            GfMatrix4d localBodyMatrix;
            localBodyMatrix.SetTranslate(GfVec3d(sourceBodyDesc->position.x, sourceBodyDesc->position.y, sourceBodyDesc->position.z));
            localBodyMatrix.SetRotateOnly(GfQuatd(sourceBodyDesc->rotation.w, sourceBodyDesc->rotation.x, sourceBodyDesc->rotation.y, sourceBodyDesc->rotation.z));
            localBodyMatrix = localBodyMatrix * instancerMatrixInverse;

            localBodyMatrix.SetTranslateOnly(GfCompMult(localBodyMatrix.ExtractTranslation(), GfVec3d(instanceScale)));

            GfMatrix4d instanceMatrix;
            instanceMatrix.SetTranslate(GfVec3d(instancePos));
            instanceMatrix.SetRotateOnly(instanceOrient);

            const GfMatrix4d bodyMatrix = localBodyMatrix * instanceMatrix * instancerMatrix;
        
            PhysxRigidBodyDesc* bodyDesc = nullptr;
            if (sourceBodyDesc->type == eDynamicBody)
            {
                bodyDesc = ICE_PLACEMENT_NEW(DynamicPhysxRigidBodyDesc)();
                DynamicPhysxRigidBodyDesc* dynamicBody = (DynamicPhysxRigidBodyDesc*)bodyDesc;
                *dynamicBody = *(DynamicPhysxRigidBodyDesc*)sourceBodyDesc;

                GfVec3ToFloat3(i < angularVelocities.size() ? degToRad(angularVelocities[i]) : GfVec3f(0.0f), dynamicBody->angularVelocity);

                GfVec3f transformedVelocity = i < velocities.size() ? velocities[i] : GfVec3f(0.0f);
                if (dynamicBody->localSpaceVelocities)
                {
                    transformedVelocity = PXR_NS::GfVec3f(instancerMatrix.Transform(transformedVelocity));
                }
                GfVec3ToFloat3(transformedVelocity, dynamicBody->linearVelocity);
            }
            else
            {
                bodyDesc = ICE_PLACEMENT_NEW(StaticPhysxRigidBodyDesc)();
                StaticPhysxRigidBodyDesc* staticBody = (StaticPhysxRigidBodyDesc*)bodyDesc;
                *staticBody = *(StaticPhysxRigidBodyDesc*)sourceBodyDesc;
            }

            GfVec3ToFloat3(bodyMatrix.ExtractTranslation(), bodyDesc->position);
            GfVec3ToFloat3(sc, bodyDesc->scale);
            GfQuatToFloat4(GfQuatf(bodyMatrix.RemoveScaleShear().ExtractRotation().GetQuat()), bodyDesc->rotation);

            const ObjectId pointInstancerBodyId = attachedStage.getObjectDatabase()->findEntry(instancerPath, ePointInstancedBody);
            if (pointInstancerBodyId == kInvalidObjectId)
            {
                PointInstancedBodyDesc piDesc;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, instancerPath, piDesc, nullptr);
                attachedStage.getObjectDatabase()->findOrCreateEntry(instancerPath, ePointInstancedBody, id);
            }

            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, targets[indices[i]], *bodyDesc, &objectInstance);
            attachedStage.getObjectDatabase()->findOrCreateEntry(targets[indices[i]], eBody, id);
            bodyDesc->shapes.clear();

            ICE_FREE(bodyDesc);
        }
        else if (objectDesc->type > eShape && objectDesc->type < eBody)
        {
            // separate shapes, that do belong to a top level already created body            
            const PhysxShapeDesc* sourceShapeDesc = static_cast<PhysxShapeDesc*>(objectDesc);

            if (topBodyPath.IsEmpty())
            {
                topBodyPath = attachedStage.pathFor(sourceShapeDesc->rigidBody);
                CARB_ASSERT(!topBodyPath.IsEmpty());

                topBodyMatrixInverse = internal::getWorldTransform(attachedStage, sourceShapeDesc->rigidBody).GetInverse();
            }

            const GfVec3f instancePos = i < positions.size() ? positions[i] : GfVec3f(0.0f);
            const GfQuatf instanceOrient = i < orientations.size() ? GfQuatf(orientations[i]) : GfQuatf(1.0f);
            const GfVec3f instanceScale = i < scales.size() ? scales[i] : GfVec3f(1.0f);

            GfMatrix4d localShapeMatrix = internal::getWorldTransform(attachedStage, attachedStage.keyFor(targetObjects[indices[i]].descPath));

            // Do this only if the object is below the instancer
            if (!targetObjects[indices[i]].outsideInstancer)
                localShapeMatrix = localShapeMatrix * instancerMatrixInverse;

            GfMatrix4d instanceMatrix;
            instanceMatrix.SetTranslate(GfVec3d(instancePos));
            instanceMatrix.SetRotateOnly(instanceOrient);

            const GfMatrix4d shapeWorldMatrix = localShapeMatrix * instanceMatrix * instancerMatrix;
            const GfMatrix4d newLocalShapeMatrix = shapeWorldMatrix * topBodyMatrixInverse;

            PhysxShapeDesc* scaledShapeDesc = scaleShapeDesc(*sourceShapeDesc, instanceScale);

            GfVec3ToFloat3(newLocalShapeMatrix.ExtractTranslation(), scaledShapeDesc->localPos);
            GfQuatToFloat4(newLocalShapeMatrix.ExtractRotationQuat(), scaledShapeDesc->localRot);

            const ObjectId pointInstancerBodyId = attachedStage.getObjectDatabase()->findEntry(instancerPath, ePointInstancedBody);
            if (pointInstancerBodyId == kInvalidObjectId)
            {
                PointInstancedBodyDesc piDesc;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, instancerPath, piDesc, nullptr);
                attachedStage.getObjectDatabase()->findOrCreateEntry(instancerPath, ePointInstancedBody, id);
            }

            objectInstance.isExclusive = true;
            createShape(attachedStage, targetObjects[indices[i]].descPath, scaledShapeDesc, &objectInstance);
        }
    }

    // Shape descs in shapeDescVector are owned by each TargetDesc's
    // ScannedStage (raw pointers into the parse-library snapshot).
    // The unique_ptr<ScannedStage> in TargetDesc cleans up on
    // targetObjects vector destruction.  No explicit free needed.

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    for (size_t i = 0; i < targets.size(); i++)
    {
        attachedStage.bufferRequestRigidBodyMassUpdate(targets[i]);
    }

    if (!topBodyPath.IsEmpty())
    {
        attachedStage.bufferRequestRigidBodyMassUpdate(topBodyPath);
    }
}
} // namespace usdparser
} // namespace physx
} // namespace omni
