// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on
#include <carb/logging/Log.h>
#include <carb/Framework.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/PrimUtilities.h>
#include <common/utilities/Utilities.h>
#include <common/utilities/OmniPhysXUtilities.h>
#include <omni/log/ILog.h>
#include <carb/profiler/Profile.h>
#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "PhysXTools.h"
#include "Mass.h"
#include "Collision.h"
#include "Particles.h"
#include "Articulation.h"
#include "PointInstancer.h"
#include "JointInstancer.h"
#include "Material.h"
#include "Scene.h"
#include "Joint.h"
#include "PhysicsBody.h"
#include "CollisionGroup.h"
#include "LoadStage.h"
#include <CookingDataAsync.h>
#include <utils/Profile.h>
#include <PhysXScene.h>
#include <OmniPhysX.h>
#include <VoxelMap.h>
#include <attachment/PhysXAttachment.h>
#include <particles/PhysXParticleSampling.h>


// physx specific stuff
#include "FixedTendon.h"
#include "SpatialTendon.h"
#include "Vehicle.h"
#include "DeformableAttachment.h"
#include "MimicJoint.h"
#include <ChangeRegister.h>

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-1 AC-5 AC-9 AC-16 AC-17 AC-18 AC-19 AC-20 AC-21 AC-22
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-5
 *
 * LoadStage is the consumer of the parse-library `ScannedStage`.
 * `loadFromRange` runs `scanStage(attachTarget, roots)` first, then
 * `processScannedDescs` iterates the typed lists (materials, shapes,
 * bodies, articulations, joints, attachments, ...) and translates
 * descriptors into runtime objects.  Scenes still use the
 * `reportObjectDesc` callback path because the simulator-ownership
 * gate (`canSceneBeProcessedByPhysX`, ADR-0002 §4) must run before
 * synchronous body dispatch resolves `simulationOwners → sceneIds`
 * against ObjectDatabase.
 *
 * Descriptors are produced ICE-allocated by `scanStage` (the
 * `IceDescriptorAllocator` is injected at the entry point; see
 * ADR-0005), so each descriptor moves out of `ScannedStage` via
 * `.release()` with no deep copy.  Cross-namespace `ObjectKey`
 * re-keying still runs at the boundary via
 * `attachedStage.keyFor(scanned.pathFor(scanKey))`.
 */

#include <omni/physics/usd/StageScan.h>
#include <OvstageSource.h>
#include "IceDescriptorAllocator.h"
#include "ScannedShapeCookingDispatch.h"

#include "TimeSampledCallbacks.h"
#include "DeformableBodyConverter.h"
#include "Material.h"  // setToDefault overloads for default sub-material descs
#include <common/utilities/UsdMaterialParsing.h>  // usdmaterialutils::getMaterialBinding
#include <propertiesUpdate/PhysXPropertiesUpdate.h>  // updateDeformableContactOffset/RestOffset
#include <pxr/usd/usdGeom/tetMesh.h>
#include <common/foundation/TypeCast.h>  // toVec3f / toQuatf for joint tm check
#include <PhysXCustomJoint.h>  // CustomJointManager for eJointCustom registry lookup
#include <cstdlib>
#include <unordered_set>

#include <omni/physics/parse/IPhysicsSource.h>  // source-backed isA/hasSchema dispatch

using namespace PXR_NS;
using namespace carb;
using namespace carb::tasking;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{
// schemaTypeToken lives in PhysXTools.h (single boundary translation).
using omni::physx::internal::schemaTypeToken;

omni::physics::ovstage::OvstageSource* seedOvstageKnownKeysForInitialLoad(
    AttachedStage& attachedStage,
    const std::vector<SdfPath>& scanRoots)
{
    auto* ovstageSource = dynamic_cast<omni::physics::ovstage::OvstageSource*>(attachedStage.getSource());
    if (!ovstageSource)
        return nullptr;

    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return nullptr;

    std::vector<omni::physics::parse::ObjectKey> knownKeys;
    std::unordered_set<uint64_t> seen;

    auto addKey = [&](omni::physics::parse::ObjectKey key)
    {
        if (key.valid() && seen.insert(key.handle).second)
            knownKeys.push_back(key);
    };
    // Enumerate the subtree via the iterative, mChildCache-backed collector rather
    // than the recursive forEachDescendantPruned walk. scanStage has already built
    // the child cache by this point, so this is a pure in-memory traversal (no
    // per-node pathOf std::string copies, recursive-mutex re-locks, per-call
    // unordered_set / std::function allocations). It visits the same key set the
    // eActiveInstanced pruned walk did (forEachChild ignores scope and reads the
    // same cache), so known-key seeding is unchanged. This was the dominant cost
    // of ovstage attach (~13 s on FrankaCabinet128 -> a few ms).
    std::vector<omni::physics::parse::ObjectKey> subtreeKeys;
    auto seedRoot = [&](omni::physics::parse::ObjectKey rootKey)
    {
        if (!rootKey.valid())
            return;
        subtreeKeys.clear();
        ovstageSource->collectDescendantKeys(rootKey, subtreeKeys);
        for (const omni::physics::parse::ObjectKey key : subtreeKeys)
            addKey(key);
    };

    if (scanRoots.empty())
    {
        seedRoot(source->getRootKey());
    }
    else
    {
        for (const SdfPath& rootPath : scanRoots)
            seedRoot(rootPath.IsEmpty() ? source->getRootKey() : attachedStage.keyFor(rootPath));
    }

    if (knownKeys.empty())
        seedRoot(source->getRootKey());

    ovstageSource->clearKnownKeys();
    ovstageSource->seedKnownKeys(knownKeys);
    ovstageSource->prefetchBucket(
        knownKeys,
        { omni::physics::ovstage::conv::kFabricWorldMatrix,
          omni::physics::ovstage::conv::kFabricLocalMatrix,
          omni::physics::ovstage::conv::kLocalTransform,
          omni::physics::ovstage::conv::kResetXformStack });
    return ovstageSource;
}

bool doesBodyExist(AttachedStage& attachedStage, SdfPath& bodyPath)
{
    const ObjectId body = attachedStage.getObjectDatabase()->findEntry(bodyPath, eBody);
    if (body != kInvalidObjectId)
        return true;

    return false;
}

void createJoint(AttachedStage& attachedStage, const SdfPath& primKey, PhysxJointDesc* desc)
{
    if (desc != nullptr)
    {
        ObjectDb* objectDb = attachedStage.getObjectDatabase();
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();

        const SdfPath body0Path = attachedStage.pathFor(desc->body0);
        const SdfPath body1Path = attachedStage.pathFor(desc->body1);

        ObjectId body0 = objectDb->findEntry(body0Path, eBody);
        if (body0 == kInvalidObjectId)
        {
            body0 = objectDb->findEntry(body0Path, eArticulationLink);
        }
        const bool body0Dynamic = body0 == kInvalidObjectId ? false : isRigidBodyDynamic(body0);
        ObjectId body1 = objectDb->findEntry(body1Path, eBody);
        if (body1 == kInvalidObjectId)
        {
            body1 = objectDb->findEntry(body1Path, eArticulationLink);
        }
        const bool body1Dynamic = body1 == kInvalidObjectId ? false : isRigidBodyDynamic(body1);

        createJoint(attachedStage, primKey, desc, body0, body0Dynamic, body1, body1Dynamic);
        ICE_FREE(desc);
    }
}

void createBodies(AttachedStage& attachedStage, BodyMap& bodyMap, BodyVector& additonalBodies)
{
    // create rigid bodies
    BodyMap::const_iterator it = bodyMap.begin();
    while (it != bodyMap.end())
    {
        PhysxRigidBodyDesc* bodyDesc = it->second.desc;
        if (bodyDesc)
        {

            // we dont send articulation links, they already have been created
            if (bodyDesc->type == eDynamicBody || bodyDesc->type == eStaticBody)
            {
                const SdfPath& bodyPath = it->first;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyPath, *bodyDesc);
                // If PhysXUsdPhysicsInterface::setForceParseOnlySingleScene is used it may return kInvalidObjectId
                if (id != kInvalidObjectId)
                {
                    attachedStage.getObjectDatabase()->findOrCreateEntry(bodyPath, eBody, id);
                    if (bodyDesc->type == eDynamicBody)
                    {
                        attachedStage.bufferRequestRigidBodyMassUpdate(bodyPath);
                    }
                }

            }
            ICE_FREE(bodyDesc);
        }

        it++;
    }

    bodyMap.clear();

    // Additional bodies
    for (std::pair<SdfPath, BodyDescAndColliders>& ref : additonalBodies)
    {
        PhysxRigidBodyDesc* bodyDesc = ref.second.desc;

        // we should have only static bodies here
        if (bodyDesc->type == eStaticBody)
        {
            const SdfPath& bodyPath = ref.first;
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyPath, *bodyDesc);

            attachedStage.getObjectDatabase()->findOrCreateEntry(bodyPath, eBody, id);
        }
        ICE_FREE(bodyDesc);
    }
    additonalBodies.clear();
}

void createJoints(AttachedStage& attachedStage, const JointVector& joints, bool initialStageLoad)
{
    std::vector<JointVector::const_iterator> jointsToAnalize2ndPass;
    // create joints, exclude gear and rack and pinion, create them in a second pass
    const size_t nbJoints = joints.size();
    for(size_t i=0;i<nbJoints;i++)
    {
        bool skipJoint = joints[i].articulationJoint;  // they get created as part of the articulation
        if ((!initialStageLoad) && (!skipJoint))
        {
            // note: articulationJoint is false if a structural change on an articulation joint prim
            // happened because only that prim might get reparsed and not the whole articulation. If
            // there still is an articulation joint, re-create the internal joint object. This is needed
            // for mimic joints, for example, as they get applied to joints and can trigger structural
            // changes on the prim without affecting the regular joint setup as such. Think of the
            // following example scenario that would fail without this extra logic:
            // - mimic joint API is added => structural change to trigger parsing
            // - InternalJoint object gets deleted (but not the PhysX articulation joint)
            // - joint prim is parsed
            // - InternalJoint object is not re-created because not marked as articulation joint
            //   (plus the PhysX articulation joint is still there anyway)
            // - mimic joint creation fails because there is no InternalJoint object
            // OM-123103 was created to re-visit this

            PhysxJointDesc& jointDesc = *joints[i].desc;

            if ((!jointDesc.excludedFromArticulation) && jointDesc.jointEnabled)
            {
                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                ObjectId link0Id = objectDb->findEntry(attachedStage.pathFor(jointDesc.body0), eArticulationLink);
                if (link0Id != kInvalidObjectId)
                {
                    ObjectId link1Id = objectDb->findEntry(attachedStage.pathFor(jointDesc.body1), eArticulationLink);
                    if (link1Id != kInvalidObjectId)
                    {
                        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
                        physInt->recreateArticulationJoint(attachedStage, jointDesc, link0Id, link1Id);
                        ICE_FREE_BASIC(&jointDesc);
                        skipJoint = true;
                    }
                }
            }
        }
        else
        {
            if (skipJoint)
            {
                ICE_FREE_BASIC(joints[i].desc);
            }
        }

        if (!skipJoint)
        {
            if (joints[i].desc->jointFriction != 0.0)
            {
                CARB_LOG_WARN("Joint friction attribute is only applied for joints in articulations. (%s)",
                              joints[i].path.GetText());
            }
            if (joints[i].desc->type != eJointGear && joints[i].desc->type != eJointRackAndPinion)
            {
                createJoint(attachedStage, joints[i].path, joints[i].desc);
            }
            else
            {
                // If we happen to be doing a 2nd pass, we want to make sure we only allocate once
                jointsToAnalize2ndPass.reserve(joints.size());
                jointsToAnalize2ndPass.push_back(joints.begin() + i);
            }
        }
    }
    const size_t nbJoints2ndPass = jointsToAnalize2ndPass.size();
    for (size_t i = 0; i < nbJoints2ndPass; i++)
    {
        // Here we are not articulationJoint and we are either eJointGear or eJointRackAndPinion
        createJoint(attachedStage, jointsToAnalize2ndPass[i]->path, jointsToAnalize2ndPass[i]->desc);
    }
}

void createMimicJoints(AttachedStage& attachedStage, MimicJointVector& mimicJoints)
{
    for (MimicJointDesc& desc : mimicJoints)
    {
        createMimicJoint(attachedStage, desc);
    }
}

void createDeformableBody(AttachedStage& attachedStage, PhysxDeformableBodyDesc* bodyDesc, SdfPath bodyPath)
{
    if (bodyDesc)
    {
        if (bodyDesc->type == eVolumeDeformableBody ||
            bodyDesc->type == eSurfaceDeformableBody)
        {
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyPath, *bodyDesc);
            // If PhysXUsdPhysicsInterface::setForceParseOnlySingleScene is used it may return kInvalidObjectId
            if (id != kInvalidObjectId)
            {
                attachedStage.getObjectDatabase()->findOrCreateEntry(bodyPath, bodyDesc->type, id);
            }
        }
        ICE_FREE(bodyDesc);
    }
}

// Source-routed equivalent of common/utilities `canSceneBeProcessedByPhysX`
// (no UsdPrim). PhysX processes a scene that carries PhysxSceneAPI, or that has
// no applied API while PhysX is the default simulator, or whose applied APIs are
// all in the allow-list.
//
// The applied-API list is the source's plain applied-schema set (the only
// notion an arbitrary IPhysicsSource backend — e.g. ovstage — can offer; there
// is no authored-vs-built-in distinction). This matches the legacy authored-only
// read in practice: the only API the allow-list path inspects is the one
// authored on a scene prim, and `UsdPhysicsScene` contributes no built-in API
// schemas of its own, so its applied-schema set equals its authored set.
bool canSceneBeProcessedByPhysXSource(const omni::physics::parse::IPhysicsSource& src,
                                      omni::physics::parse::ObjectKey key, bool isDefaultSimulator)
{
    bool processScene = src.hasSchema(key, src.internToken(PhysxSchemaTokens->PhysxSceneAPI.GetString()));
    static const char* const allowedList[] = { "PhysxSceneQuasistaticAPI", "MaterialBindingAPI",
                                               "CollectionAPI", "VehicleContextAPI",
                                               "NewtonPhysicsSceneAPI", "NewtonPhysicsXpbdSceneAPI",
                                               "NewtonPhysicsKaminoSceneAPI" };
    std::vector<std::string> apis;
    src.forEachAppliedSchema(key, [&](omni::physics::parse::TokenId t)
                             { apis.emplace_back(src.tokenToString(t)); });
    if (apis.empty() && isDefaultSimulator)
    {
        processScene = true;
    }
    else if (!processScene && !apis.empty())
    {
        bool allowedApis = true;
        for (const std::string& tokenString : apis)
        {
            bool allowed = false;
            for (const char* allowedToken : allowedList)
            {
                if (tokenString.find(allowedToken) != std::string::npos)
                {
                    allowed = true;
                    break;
                }
            }
            if (!allowed)
            {
                allowedApis = false;
                break;
            }
        }
        if (allowedApis)
            processScene = true;
    }
    return processScene;
}

void refreshAutoDeformableAttachments(AttachedStage& attachedStage,
    DeformableAttachmentVector& attachments, DeformableCollisionFilterVector& collisionFilters)
{
    // Child prims (VtxXformAttachment, etc.) live under the parent Scope prim
    // that carries AutoDeformableAttachmentAPI. Collect unique parents first —
    // applied-API gate via the source (no UsdPrim).
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    SdfPathSet autoAttachmentPaths;
    if (src)
    {
        const omni::physics::parse::TokenId autoApiTok =
            src->internToken(PhysxSchemaTokens->PhysxAutoDeformableAttachmentAPI.GetString());
        for (const auto& attachment : attachments)
        {
            const SdfPath parentPath = attachment.path.GetParentPath();
            const omni::physics::parse::ObjectKey parentKey = attachedStage.keyFor(parentPath);
            if (src->exists(parentKey) && src->hasSchema(parentKey, autoApiTok))
            {
                autoAttachmentPaths.insert(parentPath);
            }
        }
    }

    auto invalidateDescsUnderPath = [](auto& vec, const SdfPath& parentPath)
    {
        for (auto& entry : vec)
        {
            if (entry.path.HasPrefix(parentPath))
            {
                ICE_FREE(entry.desc);
            }
        }
    };

    for (const SdfPath& autoAttachmentPath : autoAttachmentPaths)
    {
        bool attachmentDataRecomputed = false;
        if (!omni::physx::updateAutoDeformableAttachment(autoAttachmentPath, attachmentDataRecomputed))
        {
            CARB_LOG_WARN("refreshAutoDeformableAttachments: updateAutoDeformableAttachment failed for %s",
                autoAttachmentPath.GetText());
        }

        // Only invalidate pre-parsed descs if attachment data was actually recomputed
        if (attachmentDataRecomputed)
        {
            invalidateDescsUnderPath(attachments, autoAttachmentPath);
            invalidateDescsUnderPath(collisionFilters, autoAttachmentPath);
        }
    }
}

void createDeformableAttachments(AttachedStage& attachedStage, DeformableAttachmentVector& attachments)
{
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
    ObjectId id = kInvalidObjectId;

    const size_t nbAttachments = attachments.size();
    for (size_t i = 0; i < nbAttachments; i++)
    {
        if (attachments[i].desc == nullptr)
        {
            // Single-prim scanStage produces a typed parse-lib desc;
            // parseDeformableAttachment translates it into the
            // consumer-side usdparser::PhysxDeformableAttachmentDesc
            // (which keeps the SdfPath src0/src1 vocabulary).
            const omni::physics::parse::IPhysicsSource* dsrc = attachedStage.getSource();
            if (!dsrc || !dsrc->exists(attachedStage.keyFor(attachments[i].path)))
                return;

            // Subtree scan rooted at the attachment prim (default predicate, no
            // instance proxies), routed through the active scan backend.
            const std::vector<SdfPath> scanRoots{ attachments[i].path };
            static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
            omni::physics::parse::ScanOptions scanOptions;
            scanOptions.descendantScope = omni::physics::parse::DescendantScope::eActive;
            omni::physics::usd::ScannedStage scanned = omni::physics::usd::scanStage(
                attachedStage.attachTarget(), scanRoots, kNoExclude,
                omni::physx::usdparser::iceDescriptorAllocator(), scanOptions);
            if (scanned.attachments.empty())
                return;

            attachments[i].desc = parseDeformableAttachment(scanned, *scanned.attachments[0]);
            if (attachments[i].desc == nullptr)
                return;
        }

        id = physInt->createObject(attachedStage, attachments[i].path, *attachments[i].desc);

        if (id != kInvalidObjectId)
            objectDb->findOrCreateEntry(attachments[i].path, attachments[i].desc->type, id);

        ICE_FREE(attachments[i].desc);
    }
}

void createDeformableCollisionFilters(AttachedStage& attachedStage, DeformableCollisionFilterVector& collisionFilters)
{
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
    ObjectId id = kInvalidObjectId;

    const size_t nbCollisionFilters = collisionFilters.size();
    for (size_t i = 0; i < nbCollisionFilters; i++)
    {
        if (collisionFilters[i].desc == nullptr)
        {
            // Single-prim scanStage; same pattern as the attachment
            // branch above.
            const omni::physics::parse::IPhysicsSource* dsrc = attachedStage.getSource();
            if (!dsrc || !dsrc->exists(attachedStage.keyFor(collisionFilters[i].path)))
                return;

            // Subtree scan rooted at the filter prim (default predicate, no instance
            // proxies), routed through the active scan backend.
            const std::vector<SdfPath> scanRoots{ collisionFilters[i].path };
            static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;
            omni::physics::parse::ScanOptions scanOptions;
            scanOptions.descendantScope = omni::physics::parse::DescendantScope::eActive;
            omni::physics::usd::ScannedStage scanned = omni::physics::usd::scanStage(
                attachedStage.attachTarget(), scanRoots, kNoExclude,
                omni::physx::usdparser::iceDescriptorAllocator(), scanOptions);
            if (scanned.deformableCollisionFilters.empty())
                return;

            collisionFilters[i].desc =
                parseDeformableCollisionFilter(scanned, *scanned.deformableCollisionFilters[0]);
            if (collisionFilters[i].desc == nullptr)
                return;
        }

        id = physInt->createObject(attachedStage, collisionFilters[i].path, *collisionFilters[i].desc);

        if (id != kInvalidObjectId)
            objectDb->findOrCreateEntry(collisionFilters[i].path, collisionFilters[i].desc->type, id);

        ICE_FREE(collisionFilters[i].desc);
    }
}

void createFilteredPairs(AttachedStage& attachedStage, const CollisionPairVector& pairsVector, const SdfPathVector& filteredPairsPaths)
{
    ObjectDb& objectDb = *attachedStage.getObjectDatabase();
    std::unordered_map<SdfPath, FilteredPairDesc, SdfPath::Hash> blockDescMap;
    for (const SdfPath& path : filteredPairsPaths)
    {
        blockDescMap[path] = FilteredPairDesc();
    }

    for (size_t iPairs = pairsVector.size(); iPairs--;)
    {
        const CollisionBlockPair& pair = pairsVector[iPairs];        

        FilteredPairDesc& blockDesc = blockDescMap[pair.first];
        const ObjectIdMap* entriesFirst = objectDb.getEntries(pair.first);
        if (entriesFirst && !entriesFirst->empty())
        {
            auto itFirst = entriesFirst->begin();
            while (itFirst != entriesFirst->end())
            {
                const ObjectIdMap* entriesSecond = objectDb.getEntries(pair.second);
                if (entriesSecond && !entriesSecond->empty())
                {
                    auto itSecond = entriesSecond->begin();
                    while (itSecond != entriesSecond->end())
                    {
                        const ObjectId firstObject = itFirst->second;
                        const ObjectId secondObject = itSecond->second;
                        blockDesc.pairs.push_back(std::make_pair(firstObject, secondObject));
                        itSecond++;
                    }
                }
                else if (const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource())
                {
                    // traverse and find the childs — default-predicate subtree walk
                    // via the source (no UsdPrim), pruning a child's subtree once a
                    // body/shape pair is recorded there (mirrors PruneChildren).
                    src->forEachDescendantPruned(
                        attachedStage.keyFor(pair.second),
                        [&](omni::physics::parse::ObjectKey childKey) -> bool
                        {
                            bool pairFound = false;
                            const ObjectIdMap* entriesSecond = objectDb.getEntries(attachedStage.pathFor(childKey));
                            if (entriesSecond && !entriesSecond->empty())
                            {
                                auto itSecond = entriesSecond->begin();
                                while (itSecond != entriesSecond->end())
                                {
                                    if (itSecond->first == eBody || itSecond->first == eShape)
                                    {
                                        pairFound = true;
                                        const ObjectId firstObject = itFirst->second;
                                        const ObjectId secondObject = itSecond->second;
                                        blockDesc.pairs.push_back(std::make_pair(firstObject, secondObject));
                                    }
                                    itSecond++;
                                }
                            }
                            return pairFound; // prune this child's descendants
                        },
                        omni::physics::parse::DescendantScope::eActive);
                }
                itFirst++;
            }
        }
    }

    for (auto& blockDesc : blockDescMap)
    {
        const ObjectId retId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, blockDesc.first, blockDesc.second);
        if (retId != kInvalidObjectId)
        {
            objectDb.findOrCreateEntry(blockDesc.first, eFilteredPair, retId);
        }
    }
}

ObjectId createObject(AttachedStage& attachedStage, const SdfPath& primKey, PhysxObjectDesc* desc,
    PhysXUsdPhysicsInterface& physicsInterface, ObjectDb& objectDb)
{
    if (!desc)
        return kInvalidObjectId;

    const ObjectId id = physicsInterface.createObject(attachedStage, primKey, *desc);
    if (id != kInvalidObjectId)
        objectDb.findOrCreateEntry(primKey, desc->type, id);
    return id;
}

ObjectId createObject(AttachedStage& attachedStage, const SdfPath& primKey, PhysxObjectDesc* desc, bool deleteDesc = true)
{
    if (!desc)
        return kInvalidObjectId;

    const ObjectId id = createObject(attachedStage, primKey, desc, *attachedStage.getPhysXPhysicsInterface(), *attachedStage.getObjectDatabase());
    if (deleteDesc)
        ICE_FREE(desc);
    return id;
}

// Build (append into) the engine's path-keyed CollisionGroupsMap by
// inverting the per-group `sourceMembers` lists already resolved by
// `parse::parseCollisionGroup` via the source backend.  This is a pure
// data transform on already-resolved data — no UsdCollectionAPI calls
// happen here.  See ADR-0006 and REQ-PARSE-COLGROUP-002.
//
// `beginIdx` / `endIdx` slice `scanned.collisionGroups` so the same
// helper drives the serial path and the parallel-batched path.  Append
// semantics match the legacy `updateCollisionCollection` exactly: a
// re-invocation on the same group adds duplicate entries (callers
// either consult `mCollectionsPopulated` to skip re-entry or, in the
// change-notice path, accept the duplication that the existing
// `getCollisionGroup` warning surfaces).
//
// @implements REQ-PARSE-COLGROUP-002
// @covers AC-4 AC-5
void invertCollisionGroupMembers(const omni::physics::usd::ScannedStage& scanned,
                                 size_t beginIdx, size_t endIdx,
                                 CollisionGroupsMap& cgMap)
{
    for (size_t i = beginIdx; i < endIdx; ++i)
    {
        const auto& group = scanned.collisionGroups[i];
        const SdfPath groupPath = scanned.pathFor(group->primKey);
        for (const omni::physics::parse::ObjectKey member : group->sourceMembers)
        {
            const SdfPath memberPath = scanned.pathFor(member);
            cgMap[memberPath].push_back(groupPath);
        }
    }
}

void setupCollisionGroups(AttachedStage& attachedStage, const std::vector<SdfPath>& collisionGroupsPaths)
{
    const ObjectDb& db = *attachedStage.getObjectDatabase();

    std::vector<SdfPath>::const_iterator it = collisionGroupsPaths.begin();

    while (it != collisionGroupsPaths.end())
    {
        const SdfPath& collisionGroupKey = (*it);
        const ObjectIdMap* map = db.getEntries(collisionGroupKey);
        CollisionGroupDesc desc;
        CARB_ASSERT(map);
        if (map)
        {
            CARB_ASSERT(map->begin()->first == eCollisionGroup);
            const ObjectId groupId = map->begin()->second;
            desc.groupId = groupId;
            // physics:filteredGroups targets via the source (no UsdPhysicsCollisionGroup
            // handle). An absent relationship yields no targets, matching the prior
            // `if (GetFilteredGroupsRel())` gate.
            SdfPathVector targets;
            if (const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource())
            {
                std::vector<omni::physics::parse::ObjectKey> filterKeys;
                src->getRelationshipTargets(attachedStage.keyFor(collisionGroupKey),
                                            src->internToken(UsdPhysicsTokens->physicsFilteredGroups.GetString()), filterKeys);
                targets.reserve(filterKeys.size());
                for (const omni::physics::parse::ObjectKey& k : filterKeys)
                    targets.push_back(attachedStage.pathFor(k));
            }
            for (size_t iTargets = 0; iTargets < targets.size(); iTargets++)
            {
                const SdfPath& filterPath = targets[iTargets];
                const ObjectIdMap* filterMap = db.getEntries(filterPath);
                if (filterMap && !filterMap->empty())
                {
                    ObjectIdMap::const_iterator itSecond = filterMap->begin();
                    while (itSecond != filterMap->end())
                    {
                        if (itSecond->first == eCollisionGroup)
                            desc.filteredGroups.push_back(itSecond->second);
                        itSecond++;
                    }
                }
            }
            attachedStage.getPhysXPhysicsInterface()->setupCollisionGroup(collisionGroupKey, desc);
        }
        it++;
    }
}

void createParticleSystemsAndObjects(AttachedStage& attachedStage, const std::vector<ParticleSystemDesc*>& particleSysDescs, const std::vector<ParticleDesc*>& particleDescs,
    CollisionPairVector& filteredPairs)
{
    // create particle systems
    for (ParticleSystemDesc* particleSys : particleSysDescs)
    {
        createObject(attachedStage, particleSys->systemPath, particleSys, false);

        for (size_t i = 0; i < particleSys->filteredCollisions.size(); i++)
        {
            filteredPairs.push_back(std::make_pair(particleSys->systemPath, particleSys->filteredCollisions[i]));
        }
        ICE_FREE(particleSys)
    }

    // create particles
    for (ParticleDesc* particleDesc : particleDescs)
    {
        createObject(attachedStage, particleDesc->primPath, particleDesc);
    }
}

omni::physx::usdparser::ObjectId findOrCreatePhysXObject(AttachedStage& attachedStage, const SdfPath& path, PhysxObjectDesc* objectDesc,
    PhysXUsdPhysicsInterface& physicsInterface, ObjectDb& objectDb)
{
    ObjectId objectId = objectDb.findEntry(path, objectDesc->type);
    if (objectId != kInvalidObjectId)
        return objectId;
    else
        return createObject(attachedStage, path, objectDesc, physicsInterface, objectDb);
}

void loadVehicle(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primObjKey, VehicleComponentTracker& vehicleComponentTracker)
{
    const SdfPath primKey = attachedStage.pathFor(primObjKey);

    VehicleDesc vehicleDesc;
    VehicleControllerDesc vehicleControllerDesc;
    VehicleTankControllerDesc vehicleTankControllerDesc;
    ObjectType vehicleControllerType;
    if (parseVehicle(attachedStage, primObjKey, vehicleDesc,
        vehicleControllerDesc, vehicleTankControllerDesc, vehicleControllerType,
        vehicleComponentTracker))
    {
        PhysXUsdPhysicsInterface& physicsInterface = *attachedStage.getPhysXPhysicsInterface();
        ObjectDb& objectDb = *attachedStage.getObjectDatabase();

        // create components, if they do not exist yet
        if (vehicleDesc.drive)
        {
            if (vehicleDesc.drive->type == ObjectType::eVehicleDriveStandard)
            {
                DriveStandardDesc* driveDesc = static_cast<DriveStandardDesc*>(vehicleDesc.drive);
                driveDesc->engineId = findOrCreatePhysXObject(attachedStage, driveDesc->engine->path, driveDesc->engine,
                    physicsInterface, objectDb);
            }
            else
            {
                CARB_ASSERT(vehicleDesc.drive->type == ObjectType::eVehicleDriveBasic);
                DriveBasicDesc* driveDesc = static_cast<DriveBasicDesc*>(vehicleDesc.drive);
                driveDesc->id = findOrCreatePhysXObject(attachedStage, driveDesc->path, driveDesc,
                    physicsInterface, objectDb);
            }
        }

        for (WheelAttachmentDesc& wheelAttachment : vehicleDesc.wheelAttachments)
        {
            wheelAttachment.id = createObject(attachedStage, wheelAttachment.path, &wheelAttachment, physicsInterface, objectDb);

            WheelDesc* wheelDesc = wheelAttachment.wheel;
            wheelAttachment.wheelId = findOrCreatePhysXObject(attachedStage, wheelDesc->path, wheelDesc,
                physicsInterface, objectDb);

            TireDesc* tireDesc = wheelAttachment.tire;
            wheelAttachment.tireId = findOrCreatePhysXObject(attachedStage,tireDesc->path, tireDesc,
                physicsInterface, objectDb);

            SuspensionDesc* suspDesc = wheelAttachment.suspension;
            wheelAttachment.suspensionId = findOrCreatePhysXObject(attachedStage, suspDesc->path, suspDesc,
                physicsInterface, objectDb);

            if (!wheelAttachment.tire->frictionTablePath.IsEmpty())
                wheelAttachment.tire->frictionTableId = objectDb.findEntry(
                    wheelAttachment.tire->frictionTablePath, eVehicleTireFrictionTable);
            else
                wheelAttachment.tire->frictionTableId = kInvalidObjectId;

            if (!wheelAttachment.collisionGroupPath.IsEmpty())
            {
                wheelAttachment.collisionGroupId = objectDb.findEntry(
                    wheelAttachment.collisionGroupPath, eCollisionGroup);
            }
            else
                wheelAttachment.collisionGroupId = kInvalidObjectId;

            if (wheelAttachment.state & WheelAttachmentDesc::eHAS_SHAPE)
                wheelAttachment.shapeId =
                    objectDb.findEntry(wheelAttachment.shapePath, eShape);
            else
                wheelAttachment.shapeId = kInvalidObjectId;
        }

        for (WheelControllerDesc& wheelController : vehicleDesc.wheelControllers)
        {
            wheelController.id = createObject(attachedStage, wheelController.path, &wheelController, physicsInterface, objectDb);
        }

        vehicleDesc.bodyId = objectDb.findEntry(primKey, eBody);

        ObjectId vehicleId = createObject(attachedStage, primKey, &vehicleDesc, physicsInterface, objectDb);

        if (vehicleId != kInvalidObjectId)
        {
            if (vehicleControllerType != eUndefined)
            {
                if (vehicleControllerType == eVehicleControllerStandard)
                    createObject(attachedStage, primKey, &vehicleControllerDesc, physicsInterface, objectDb);
                else
                {
                    CARB_ASSERT(vehicleControllerType == eVehicleControllerTank);
                    createObject(attachedStage, primKey, &vehicleTankControllerDesc, physicsInterface, objectDb);
                }
            }
        }
        else
        {
            for (WheelControllerDesc& wheelController : vehicleDesc.wheelControllers)
            {
                if (wheelController.id != kInvalidObjectId)
                {
                    physicsInterface.releaseObject(attachedStage, wheelController.path, wheelController.id);
                    wheelController.id = kInvalidObjectId;
                }
            }

            for (WheelAttachmentDesc& wheelAttachment : vehicleDesc.wheelAttachments)
            {
                if (wheelAttachment.id != kInvalidObjectId)
                {
                    physicsInterface.releaseObject(attachedStage, wheelAttachment.path, wheelAttachment.id);
                    wheelAttachment.id = kInvalidObjectId;
                }
            }
        }
    }
}

struct ParsingFlag
{
    enum Enum
    {
        eParseInternal = 1 << 0,
        eParseVehicles = 1 << 1,
        eParseParticles = 1 << 2
    };
};

// Stage-load orchestration: invokes scanStage, runs per-prim side
// effects, then iterates ScannedStage's typed lists in
// processScannedDescs.  The class name dates back to an
// IUsdPhysicsListener subclass; renaming is deferred to limit churn.
class PhysxUsdPhysicsListener
{
public:

    PhysxUsdPhysicsListener(AttachedStage& attachedStage)
        : mSceneFound(false), mNoValidScene(false), mNoPhysXScene(false), mCollectionsPopulated(false), mNumScenes(0), mAttachedStage(attachedStage)
    {
        mNoPhysXScene = !attachedStage.isPhysXDefaultSimulator();
    }


    // Per-prim side-effect walk.  scanStage's descriptor lists cover
    // most of the data, but a handful of per-prim parse-flag and
    // schema-API-flag side effects (CCT, PBD material, particles,
    // vehicles, force-API, contact reports) aren't part of the
    // descriptor model and must still be visited per-prim here.
    // Source-driven replacement for the legacy PrimIterator re-walks. Visits
    // every load object under each scan root (root inclusive) via IPhysicsSource,
    // honoring the replicator exclude set; `visit` returns true to prune that
    // object's subtree (mirrors PrimIterator::pruneChildren). The eActiveInstanced
    // scope reproduces the UsdTraverseInstanceProxies walk the iterators used —
    // no UsdPrim, no PrimIterator.
    void forEachLoadObject(const std::vector<SdfPath>& scanRoots, const PathSet* excludePaths,
                           const std::function<bool(omni::physics::parse::ObjectKey)>& visit)
    {
        const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();
        if (!src)
            return;
        for (const SdfPath& root : scanRoots)
        {
            src->forEachDescendantPruned(
                mAttachedStage.keyFor(root),
                [&](omni::physics::parse::ObjectKey key) -> bool
                {
                    if (excludePaths && !excludePaths->empty())
                    {
                        const SdfPath keyPath = mAttachedStage.pathFor(key);
                        // A root-prim exclude path is never pruned (see
                        // PrimIteratorExcludeRange): the legacy stage->Traverse()
                        // walk yielded the stage's root prim unchecked, so a
                        // root-prim-level exclude (clone()'s derived env root,
                        // e.g. "/World") was a no-op and the subtree was still
                        // visited.  Keep the gather walk consistent with the
                        // backend scan: only prune non-root-prim exclude paths.
                        if (!keyPath.IsRootPrimPath() &&
                            excludePaths->find(keyPath) != excludePaths->end())
                            return true; // skip the excluded prim and prune its subtree
                    }
                    return visit(key);
                },
                omni::physics::parse::DescendantScope::eActiveInstanced);
        }
    }

    void gatherPerPrimSideEffects(const std::vector<SdfPath>& scanRoots, const PathSet* excludePaths)
    {
        static const TfToken gCctAPI("PhysxCharacterControllerAPI");
        static const TfToken gParticleSetAPI("PhysxParticleSetAPI");
        static const TfToken gParticleSamplingAPI("PhysxParticleSamplingAPI");
        static const TfToken gVehicleAPI("PhysxVehicleAPI");
        static const TfToken gVehicleContextAPI("PhysxVehicleContextAPI");
        static const TfToken gCollisionAPIToken("PhysicsCollisionAPI");
        static const TfToken gFilteredPairsAPIToken("PhysicsFilteredPairsAPI");
        static const TfToken gForceAPIToken("PhysxForceAPI");
        static const TfToken gContactReportAPIToken("PhysxContactReportAPI");

        DeformableAttachmentHistoryMap& attHistory =
            mAttachedStage.getDeformableAttachmentHistoryMap();
        DeformableCollisionFilterHistoryMap& filterHistory =
            mAttachedStage.getDeformableCollisionFilterHistoryMap();

        const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();

        forEachLoadObject(scanRoots, excludePaths, [&](omni::physics::parse::ObjectKey primObjKey) -> bool
        {
            const SdfPath primKey = mAttachedStage.pathFor(primObjKey);
            ObjectDb* objectDb = mAttachedStage.getObjectDatabase();

            // Applied-API list via the source (= prim.GetAppliedSchemas()).
            src->forEachAppliedSchema(primObjKey, [&](omni::physics::parse::TokenId tok)
            {
                const TfToken api{ std::string(src->tokenToString(tok)) };
                if (api == gCctAPI)
                {
                    mParsingFlags |= ParsingFlag::eParseInternal;
                }
                else if (api == gForceAPIToken)
                {
                    objectDb->addSchemaAPI(primKey, SchemaAPIFlag::ePhysxForceAPI);
                    PhysxForceDesc* desc = parsePhysxForce(mAttachedStage, primObjKey);
                    mPhysxForceDescs.push_back(std::make_pair(primKey, desc));
                }
                else if (api == gFilteredPairsAPIToken)
                {
                    mFilteredPairsPaths.push_back(primKey);
                    objectDb->addSchemaAPI(primKey, SchemaAPIFlag::eFilteredPairsAPI);
                }
                else if (api == gParticleSetAPI || api == gParticleSamplingAPI)
                {
                    mParsingFlags |= ParsingFlag::eParseParticles;
                }
                else if (api == gVehicleAPI)
                {
                    mParsingFlags |= ParsingFlag::eParseVehicles;
                }
                else if (api == gVehicleContextAPI)
                {
                    // Vehicle context parsing moved into the native
                    // walker in 7A.1 (ADR-0008); the descriptor flows
                    // through scanned.vehicleContexts.  We still flip
                    // the parsing flag here so the post-load vehicle
                    // creation block runs.
                    mParsingFlags |= ParsingFlag::eParseVehicles;
                }
                else if (api == gCollisionAPIToken)
                {
                    objectDb->addSchemaAPI(primKey, SchemaAPIFlag::eCollisionAPI);
                }
                else if (api == gContactReportAPIToken)
                {
                    objectDb->addSchemaAPI(primKey, SchemaAPIFlag::eContactReportAPI);
                }
            });

            if (src->isA(primObjKey, schemaTypeToken<UsdGeomPointInstancer>(*src)))
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (src->isA(primObjKey, schemaTypeToken<PhysxSchemaPhysxParticleSystem>(*src)))
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (src->isA(primObjKey, schemaTypeToken<PhysxSchemaPhysxPhysicsJointInstancer>(*src)))
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (src->isA(primObjKey, schemaTypeToken<UsdPhysicsCollisionGroup>(*src)))
            {
                mCollisionGroupsPrims.push_back(primKey);
            }
            else if (src->isA(primObjKey, schemaTypeToken<PhysxSchemaPhysxVehicleTireFrictionTable>(*src)))
            {
                // Tire friction table parsing moved into the native
                // walker in 7A.1 (ADR-0008); the descriptor flows
                // through scanned.tireFrictionTables.
                mParsingFlags |= ParsingFlag::eParseVehicles;
            }

            // Deformable attachment / filter history dispatch (matches
            // the deleted parsePrim block — replays history entries
            // whose key matches this prim into the to-be-created list).
            {
                auto it = attHistory.find(primKey);
                while (it != attHistory.end() && it->first == primKey)
                {
                    DeformableAttachmentDescAndPath attachmentDescAndPath;
                    attachmentDescAndPath.path = it->second;
                    attachmentDescAndPath.desc = nullptr;
                    mDeformableAttachmentVector.push_back(attachmentDescAndPath);
                    ++it;
                }
            }
            {
                auto it = filterHistory.find(primKey);
                while (it != filterHistory.end() && it->first == primKey)
                {
                    DeformableCollisionFilterDescAndPath collisionFilterDescAndPath;
                    collisionFilterDescAndPath.path = it->second;
                    collisionFilterDescAndPath.desc = nullptr;
                    mDeformableCollisionFilterVector.push_back(collisionFilterDescAndPath);
                    ++it;
                }
            }

            return false; // gather never prunes (legacy only pruned invalid prims, which the source walk omits)
        });
    }

    void gatherScannedSideEffects(omni::physics::usd::ScannedStage& scanned)
    {
        ObjectDb* objectDb = mAttachedStage.getObjectDatabase();
        if (!objectDb)
            return;

        if (!scanned.ccts.empty())
            mParsingFlags |= ParsingFlag::eParseInternal;

        if (scanned.hasPointInstancerPrims ||
            !scanned.particleSystems.empty() || !scanned.particleSets.empty() ||
            !scanned.particleSamplers.empty() || !scanned.particleAnisotropies.empty() ||
            !scanned.particleSmoothings.empty() || !scanned.particleIsosurfaces.empty())
        {
            mParsingFlags |= ParsingFlag::eParseParticles;
        }

        if (!scanned.vehicleContexts.empty() || !scanned.tireFrictionTables.empty() ||
            !scanned.vehicleWheels.empty() || !scanned.vehicleTires.empty() ||
            !scanned.vehicleSuspensions.empty() || !scanned.vehicleEngines.empty() ||
            !scanned.vehicleGears.empty() || !scanned.vehicleClutches.empty() ||
            !scanned.vehicleDrivesBasic.empty() || !scanned.vehicleDrivesStandard.empty() ||
            !scanned.vehicleMultiWheelDifferentials.empty() || !scanned.vehicleTankDifferentials.empty() ||
            !scanned.vehicleAutoGearBoxes.empty() || !scanned.vehicleBrakes.empty() ||
            !scanned.vehicleSteeringBasic.empty() || !scanned.vehicleSteeringAckermann.empty() ||
            !scanned.vehicleNonlinearCmdResponses.empty() || !scanned.vehicleWheelAttachments.empty() ||
            !scanned.vehicles.empty() || !scanned.vehicleSuspensionCompliances.empty())
        {
            mParsingFlags |= ParsingFlag::eParseVehicles;
        }

        for (const omni::physics::parse::DescPtr<PhysxShapeDesc>& shapeUPtr : scanned.shapes)
        {
            if (!shapeUPtr)
                continue;
            const SdfPath path = scanned.pathFor(shapeUPtr->primKey);
            objectDb->addSchemaAPI(path, SchemaAPIFlag::eCollisionAPI);
            if (!shapeUPtr->sourceFilteredCollisions.empty())
            {
                objectDb->addSchemaAPI(path, SchemaAPIFlag::eFilteredPairsAPI);
                mFilteredPairsPaths.push_back(path);
            }
        }

        for (const omni::physics::parse::DescPtr<PhysxRigidBodyDesc>& bodyUPtr : scanned.bodies)
        {
            if (!bodyUPtr)
                continue;
            const SdfPath path = scanned.pathFor(bodyUPtr->primKey);
            if (!bodyUPtr->sourceFilteredCollisions.empty())
            {
                objectDb->addSchemaAPI(path, SchemaAPIFlag::eFilteredPairsAPI);
                mFilteredPairsPaths.push_back(path);
            }
        }

        for (const omni::physics::parse::DescPtr<PhysxArticulationDesc>& articulationUPtr : scanned.articulations)
        {
            if (!articulationUPtr)
                continue;
            const SdfPath path = scanned.pathFor(articulationUPtr->rootPrim);
            if (!articulationUPtr->sourceFilteredCollisions.empty())
            {
                objectDb->addSchemaAPI(path, SchemaAPIFlag::eFilteredPairsAPI);
                mFilteredPairsPaths.push_back(path);
            }
        }

        for (const omni::physics::parse::DescPtr<omni::physics::parse::PhysxDeformableBodyDesc>& deformableUPtr : scanned.deformables)
        {
            if (!deformableUPtr)
                continue;
            const SdfPath path = scanned.pathFor(deformableUPtr->primKey);
            if (!deformableUPtr->sourceFilteredCollisions.empty())
            {
                objectDb->addSchemaAPI(path, SchemaAPIFlag::eFilteredPairsAPI);
                mFilteredPairsPaths.push_back(path);
            }
        }

        for (const omni::physics::parse::DescPtr<omni::physics::parse::CollisionGroupDesc>& collisionGroupUPtr : scanned.collisionGroups)
        {
            if (collisionGroupUPtr)
                mCollisionGroupsPrims.push_back(scanned.pathFor(collisionGroupUPtr->primKey));
        }
    }

    // Translate scanStage descriptors into runtime objects.  Runs
    // after the backend-dispatched scanStage returns; populates mShapes /
    // mBodyMap / mMaterials / etc. buffers consumed by the body /
    // shape / articulation creation paths.
    void processScannedDescs(omni::physics::usd::ScannedStage& scanned,
                             const std::vector<SdfPath>& scanRoots, const PathSet* excludePaths)
    {
        callbacks::TimeSampledCallbackList cbList;

        // ----- Scenes ------------------------------------------------
        // Process scenes first so they're in ObjectDatabase by the time
        // the later body / shape / articulation blocks resolve
        // `simulationOwners → sceneIds`.  The consumer-side
        // `canSceneBeProcessedByPhysX` simulator-ownership gate
        // (ADR-0002 §4 — engine-agnostic scan, consumer applies
        // filtering) decides whether each scanned scene becomes a
        // PhysX scene.  Mirrors legacy `reportObjectDesc:eScene`
        // behavior including mSceneFound / mNoPhysXScene / mNumScenes
        // bookkeeping.
        for (auto& scanScene : scanned.scenes)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:scene");
            const SdfPath path = scanned.pathFor(scanScene->primKey);
            const omni::physics::parse::IPhysicsSource* scnSrc = mAttachedStage.getSource();
            const omni::physics::parse::ObjectKey sceneKey = mAttachedStage.keyFor(path);
            if (!scnSrc)
                continue;
            // A synthetic default scene (makeDefaultSceneDesc — backend authored no
            // prim) has no source object, so skip the existence + simulator-ownership
            // gates that assume a real prim. Real scanned scenes still pass through them.
            if (!scanScene->synthetic)
            {
                if (!scnSrc->exists(sceneKey))
                    continue;
                const bool isDefaultSimulator = mAttachedStage.isPhysXDefaultSimulator();
                if (!canSceneBeProcessedByPhysXSource(*scnSrc, sceneKey, isDefaultSimulator))
                    continue;
            }
            PhysxSceneDesc* desc = scanScene.release();
            if (!desc)
                continue;

            // Cross-namespace ObjectKey re-keying (REQ-PARSE-CONSUMER-001
            // AC-5): scanStage populates `materialKey` fields with
            // scan-side ObjectKeys minted by scanStage's UsdSource.  The
            // PhysX runtime resolves these via `attachedStage.pathFor()`
            // (e.g. `PhysXScene::mMaterialPath` for the per-scene default
            // material), which uses the consumer-side UsdSource.  Convert
            // each materialKey through pathFor→keyFor so attachedStage
            // can later resolve it back to an SdfPath.  Without this the
            // scene's default-material listener never fires when the
            // bound material's restitution changes (TestContactReport
            // "Contact Report Lost - Plane Delete" reproduces).
            auto rekey = [&](omni::physics::parse::ObjectKey& k) {
                if (k.valid()) k = mAttachedStage.keyFor(scanned.pathFor(k));
            };
            rekey(desc->defaultMaterialDesc.materialKey);
            rekey(desc->defaultDeformableMaterialDesc.materialKey);
            rekey(desc->defaultSurfaceDeformableMaterialDesc.materialKey);
            rekey(desc->defaultPBDMaterialDesc.materialKey);

            // Scene-level nested material descriptor defaults + binding overlay.
            // Mirrors legacy `parseSceneDesc` at usdLoad/Scene.cpp:108-138:
            //   1. `setToDefault` each nested material desc (units-aware defaults
            //      — youngsModulus needs metersPerUnit/kilogramsPerUnit, surface
            //      thickness needs metersPerUnit).  The parse-lib `setToDefault`
            //      for PhysxSceneDesc doesn't touch the nested descs (SourceUnits
            //      lacks kilogramsPerUnit; the deformable defaults need it), so
            //      we re-apply the USD-side defaults here at the consumer
            //      boundary.  Without this, scene->defaultDeformableMaterialDesc
            //      ships with youngsModulus=0 — `createDeformableVolumeMaterial`
            //      makes a degenerate material and modified-rest-shape volume
            //      deformables collapse (TestDeformables.cpp:606).
            //   2. Overlay each nested desc from the scene prim's material
            //      binding (if any) via the source-backed `parse*ForPrim` helpers.
            //      These no-op when the source has no bound material key.  This replaces
            //      the binding overlay that `StageScan::emitScene` does — the
            //      consumer's overlay applies to a properly-defaulted desc and
            //      uses legacy `parseMaterialDescInt` paths.
            setToDefault(desc->defaultMaterialDesc);
            // Units-aware deformable-material defaults from the source units
            // (backend-agnostic; no USD stage needed under ovstage).
            const omni::physics::parse::SourceUnits sceneUnits = scnSrc->getSourceUnits();
            setToDefault(sceneUnits, desc->defaultDeformableMaterialDesc);
            setToDefault(sceneUnits, desc->defaultSurfaceDeformableMaterialDesc);
            setToDefault(desc->defaultPBDMaterialDesc);
            // Scene's bound default material via the source (no UsdPrim / UsdStage).
            const omni::physics::parse::ObjectKey sceneMatKey = scnSrc->getMaterialBinding(sceneKey);
            parseMaterialForPrim(mAttachedStage, sceneMatKey, desc->defaultMaterialDesc);
            parseDeformableMaterialForPrim(mAttachedStage, sceneMatKey, desc->defaultDeformableMaterialDesc);
            parseSurfaceDeformableMaterialForPrim(mAttachedStage, sceneMatKey, desc->defaultSurfaceDeformableMaterialDesc);
            parsePBDMaterialForPrim(mAttachedStage, sceneMatKey, desc->defaultPBDMaterialDesc);

            // PhysxSceneQuasistaticAPI: resolve the actors collection at
            // the consumer (USD-side work; parse-lib has no UsdCollectionAPI
            // dependency).  Mirrors `parseSceneDesc` at usdLoad/Scene.cpp:
            // 288-310 — when the includes rel has targets, compute the
            // membership and insert each resolved path as an attachedStage
            // key into `desc->quasistaticActors`.
            {
                // resolveCollection performs the include/exclude membership walk
                // (instance proxies included) internally and returns persistent-source
                // ObjectKeys, replacing the UsdCollectionAPI::ComputeIncludedPaths path.
                const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();
                if (src && src->hasSchema(mAttachedStage.keyFor(path),
                                          schemaTypeToken<PhysxSchemaPhysxSceneQuasistaticAPI>(*src)))
                {
                    std::vector<omni::physics::parse::ObjectKey> members;
                    src->resolveCollection(mAttachedStage.keyFor(path), src->internToken("quasistaticactors"), members);
                    desc->quasistaticActors.clear();
                    desc->quasistaticActors.reserve(members.size());
                    for (const omni::physics::parse::ObjectKey& k : members)
                        desc->quasistaticActors.insert(k);
                }
            }

            const ObjectId sceneId = createObject(mAttachedStage, path, desc);
            if (sceneId != kInvalidObjectId)
            {
                mSceneFound = true;
                mNoPhysXScene = false;
                mNumScenes++;
            }
            else if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
            {
                mNoValidScene = true;
            }
        }

        if (mNoValidScene)
            return;
        if (mNoPhysXScene)
            return;
        // ----- CollisionGroups --------------------------------------
        // First-time collection population: invert the per-group
        // `sourceMembers` lists (already resolved during scanStage by
        // `IPhysicsSource::resolveCollection`) into the engine's
        // path-keyed CollisionGroupsMap.  No UsdCollectionAPI calls
        // here — the backend already did the include/exclude/child
        // traversal.  See ADR-0006 / REQ-PARSE-COLGROUP-002.
        if (!scanned.collisionGroups.empty())
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:collisionGroup");
            if (!mCollectionsPopulated)
            {
                PHYSICS_PROFILE("invertCollisionGroupMembers");
                const size_t minBatchSize = 20;
                const size_t collisionGroupsSize = scanned.collisionGroups.size();
                const omni::physics::parse::AttachTarget attachTarget = mAttachedStage.attachTarget();
                const bool serialCollisionGroupInversion = attachTarget.nativeStage && attachTarget.stageId == 0;
                if (collisionGroupsSize < minBatchSize || serialCollisionGroupInversion)
                {
                    invertCollisionGroupMembers(scanned, 0, collisionGroupsSize,
                                                mAttachedStage.getCollisionGroupMap());
                }
                else
                {
                    const size_t numBatches = 24;
                    const size_t batchSize = collisionGroupsSize / numBatches;
                    if (!mAttachedStage.getAdditionalCollisionGroupMaps().empty())
                    {
                        PHYSICS_PROFILE("invertCollisionGroupMembers:merge");
                        CollisionGroupsMap& cgMap = mAttachedStage.getCollisionGroupMap();
                        for (const CollisionGroupsMap& m : mAttachedStage.getAdditionalCollisionGroupMaps())
                            cgMap.insert(m.begin(), m.end());
                        mAttachedStage.getAdditionalCollisionGroupMaps().clear();
                    }
                    std::vector<CollisionGroupsMap>& cgMaps = mAttachedStage.getAdditionalCollisionGroupMaps();
                    cgMaps.resize(numBatches);
                    auto&& computeFunc = [&scanned, numBatches, batchSize, collisionGroupsSize, &cgMaps](size_t batchIndex)
                    {
                        const size_t batchEnd =
                            batchIndex == (numBatches - 1) ? collisionGroupsSize : (batchIndex + 1) * batchSize;
                        invertCollisionGroupMembers(scanned, batchIndex * batchSize, batchEnd, cgMaps[batchIndex]);
                    };
                    {
                        PHYSICS_PROFILE("invertCollisionGroupMembers:parallelFor");
                        ITasking* tasking = carb::getCachedInterface<ITasking>();
                        tasking->parallelFor(size_t(0), numBatches, computeFunc);
                    }
                }
                mCollectionsPopulated = true;
            }
            for (auto& cgUPtr : scanned.collisionGroups)
            {
                const SdfPath path = scanned.pathFor(cgUPtr->primKey);
                if (std::find(mCollisionGroupsPrims.begin(), mCollisionGroupsPrims.end(), path) ==
                    mCollisionGroupsPrims.end())
                {
                    mCollisionGroupsPrims.push_back(path);
                }
                CollisionGroupDesc desc;
                createObject(mAttachedStage, path, &desc, false);
            }
        }

        // ----- Materials --------------------------------------------
        for (auto& matUPtr : scanned.materials)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:material");
            const SdfPath path = scanned.pathFor(matUPtr->materialKey);
            PhysxMaterialDesc* desc = matUPtr.release();
            if (desc)
                mMaterials.push_back(std::make_pair(path, desc));
        }

        // ----- PBDMaterials ----------------------------------------
        for (omni::physics::parse::DescPtr<PBDMaterialDesc>& matUPtr : scanned.pbdMaterials)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:pbdMaterial");
            const SdfPath path = scanned.pathFor(matUPtr->materialKey);
            PBDMaterialDesc* desc = matUPtr.release();
            if (desc)
                mPDBMatrialsDescs.push_back(std::make_pair(path, desc));
        }

        // ----- DeformableMaterials ---------------------------------
        for (auto& dmUPtr : scanned.deformableMaterials)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:deformableMaterial");
            const SdfPath path = scanned.pathFor(dmUPtr->materialKey);
            PhysxDeformableMaterialDesc* desc = dmUPtr.release();
            if (desc)
                mDeformableMaterials.push_back(std::make_pair(path, desc));
        }

        // ----- Shapes ----------------------------------------------
        // Per-shape boundary pipeline: cooking dispatch + bounding-shape
        // compute (AC-10) + ObjectKey re-keying + state translation +
        // instance-proxy fixups (AC-13) + collisionGroup lookup + ICE
        // clone.  Triangle-mesh → ConvexHull fallback for non-kinematic
        // dynamic bodies (AC-15) happens earlier in
        // scanStage::emitShape so the desc emitted here already has the
        // correct subclass.
        for (auto& shapeUPtr : scanned.shapes)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:shape");
            PhysxShapeDesc* scanDesc = shapeUPtr.get();
            const SdfPath path = scanned.pathFor(scanDesc->primKey);

            scan::dispatchScannedShapeCooking(mAttachedStage, scanned, scanDesc);

            if (scanDesc->type == eBoundingSphereShape)
            {
                auto* bs = static_cast<BoundingSpherePhysxShapeDesc*>(scanDesc);
                if (bs->mergedMesh && !bs->mergedMesh->points.empty())
                {
                    if (BoundingSpherePhysxShapeDesc* computed = computeBoundingSphereShape(bs->mergedMesh->points))
                    {
                        bs->radius         = computed->radius;
                        bs->positionOffset = computed->positionOffset;
                        ICE_FREE(computed);
                    }
                }
            }
            else if (scanDesc->type == eBoundingBoxShape)
            {
                auto* bb = static_cast<BoundingBoxPhysxShapeDesc*>(scanDesc);
                if (bb->mergedMesh && !bb->mergedMesh->points.empty())
                {
                    if (BoundingBoxPhysxShapeDesc* computed = computeBoundingBoxShape(bb->mergedMesh->points))
                    {
                        bb->halfExtents    = computed->halfExtents;
                        bb->positionOffset = computed->positionOffset;
                        bb->rotationOffset = computed->rotationOffset;
                        ICE_FREE(computed);
                    }
                }
            }

            if (scanDesc->rigidBody.valid())
                scanDesc->rigidBody = mAttachedStage.keyFor(scanned.pathFor(scanDesc->rigidBody));
            if (scanDesc->sourceGprim.valid())
                scanDesc->sourceGprim = mAttachedStage.keyFor(scanned.pathFor(scanDesc->sourceGprim));
            if (scanDesc->type == eConvexMeshShape)
            {
                auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(scanDesc);
                if (d->meshPrimKey.valid()) d->meshPrimKey = mAttachedStage.keyFor(scanned.pathFor(d->meshPrimKey));
            }
            else if (scanDesc->type == eTriangleMeshShape ||
                     scanDesc->type == eConvexMeshDecompositionShape ||
                     scanDesc->type == eSpherePointsShape)
            {
                auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(scanDesc);
                if (d->meshPrimKey.valid()) d->meshPrimKey = mAttachedStage.keyFor(scanned.pathFor(d->meshPrimKey));
            }

            SdfPathVector materials;
            if (!scan::resolveConsumerSideShapeState(mAttachedStage, scanned, scanDesc, materials, mFilteredPairs))
                continue;

            // Instance-proxy fixups (REQ-PARSE-CONSUMER-001 AC-13).
            // Mirrors legacy `parseCollisionDesc` master/instance branch
            // (Collision.cpp:1622-1653).
            {
                const omni::physics::parse::IPhysicsSource* isrc = mAttachedStage.getSource();
                const omni::physics::parse::ObjectKey shapeKey = mAttachedStage.keyFor(path);
                if (isrc && isrc->isInstanceProxy(shapeKey))
                {
                    if (scanDesc->sourceMaterials.size() < 2)
                    {
                        // Instance-proxy material binding via the source (no UsdPrim).
                        const omni::physics::parse::ObjectKey matKey = isrc->getMaterialBinding(shapeKey);
                        const SdfPath instMatPath = matKey.valid() ? mAttachedStage.pathFor(matKey) : SdfPath();
                        if (!instMatPath.IsEmpty())
                        {
                            materials.clear();
                            materials.push_back(instMatPath);
                        }
                    }
                    if (scanDesc->rigidBody.valid())
                    {
                        const SdfPath bodyPath = mAttachedStage.pathFor(scanDesc->rigidBody);
                        if (isrc->exists(mAttachedStage.keyFor(bodyPath)))
                        {
                            GfVec3f localPos, localScale;
                            GfQuatf localRot;
                            getCollisionShapeLocalTransform(mAttachedStage, shapeKey,
                                scanDesc->rigidBody, localPos, localRot, localScale);
                            scanDesc->localPos   = { localPos[0], localPos[1], localPos[2] };
                            scanDesc->localRot   = { localRot.GetImaginary()[0], localRot.GetImaginary()[1],
                                                     localRot.GetImaginary()[2], localRot.GetReal() };
                            scanDesc->localScale = { localScale[0], localScale[1], localScale[2] };
                        }
                    }
                }
            }

            scanDesc->collisionGroup = getCollisionGroup(mAttachedStage, path);

            PhysxShapeDesc* desc = shapeUPtr.release();
            if (!desc)
                continue;

            // Keep runtime property-update requirements in sync with the scanned
            // descriptor model. The scanner only emits shapes for CollisionAPI
            // prims, so this mirrors the rigid-body eRigidBodyAPI flag stored in
            // the body loop below and makes contact/rest offset live updates
            // independent of the side-effect gather pass.
            mAttachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eCollisionAPI);

            callbacks::collectShapeTimeSampledCallbacks(mAttachedStage, mAttachedStage.keyFor(path), cbList);

            mShapes.push_back({ path, desc, materials });
        }

        // ----- RigidBodies -----------------------------------------
        // AC-16 investigation: bodies routed through scanStage emit.
        // Mirrors the pre-revert block from 8ef89730ca~1.  Translates
        // ObjectKeys back to SdfPath via scanned.pathFor() and clones
        // into ICE storage.
        for (auto& bodyUPtr : scanned.bodies)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:rigidBody");
            PhysxRigidBodyDesc* scanDesc = bodyUPtr.get();
            const SdfPath path = scanned.pathFor(scanDesc->primKey);

            // Translate sourceSimulationOwners → sceneIds.
            scanDesc->sceneIds.clear();
            for (const auto& sk : scanDesc->sourceSimulationOwners)
            {
                const SdfPath sp = scanned.pathFor(sk);
                if (sp.IsEmpty()) continue;
                const ObjectId entry = mAttachedStage.getObjectDatabase()->findEntry(sp, eScene);
                if (entry != kInvalidObjectId)
                    scanDesc->sceneIds.push_back(entry);
            }
            // Reject bodies whose explicit simulationOwners all failed to
            // resolve (e.g. the named scene is owned by a non-PhysX
            // simulator).  Mirrors `PhysicsBody.cpp::fillRigidBodyDesc`
            // lines 303-306 + `parseRigidBody` line 674: when no scene
            // resolves, the body must be dropped, not created.
            if (!scanDesc->sourceSimulationOwners.empty() && scanDesc->sceneIds.empty())
                continue;

            // Translate sourceFilteredCollisions → filteredPairs.
            for (const auto& fk : scanDesc->sourceFilteredCollisions)
            {
                const SdfPath fp = scanned.pathFor(fk);
                if (!fp.IsEmpty())
                    mFilteredPairs.push_back(std::make_pair(path, fp));
            }

            // Time-varying transform → kinematic auto-conversion.
            // Mirrors `PhysicsBody.cpp::parseRigidBody` lines 642-659:
            // a dynamic body with an animated parent xformOp gets force-
            // converted to kinematic so it follows the xform sample
            // stream.  Must happen pre-clone so the ICE-stored desc
            // carries the corrected flag.  The animated-kinematic map
            // is keyed by SdfPath, populated regardless of the original
            // kinematic flag.
            if (scanDesc->type == eDynamicBody)
            {
                const omni::physics::parse::IPhysicsSource* ssrc = mAttachedStage.getSource();
                const omni::physics::parse::ObjectKey bodyKey = mAttachedStage.keyFor(path);

                // Splines surface velocity validation warnings.  The
                // parse library silently disables splines when its
                // checks fail; legacy `PhysicsBody.cpp::parseRigidBody`
                // also CARB_LOG_ERRORs the reason.  Reproduce the
                // user-facing warnings here so test_physxSurfaceVelocityAPI
                // (which uses ExpectMessage) still observes them. Source-routed (no UsdPrim).
                DynamicPhysxRigidBodyDesc* dyn = static_cast<DynamicPhysxRigidBodyDesc*>(scanDesc);
                // Gate on the schema being applied, NOT on the post-parse
                // `splinesSurfaceVelocityEnabled` flag: the parser DISABLES that flag
                // for exactly the invalid configurations these warnings describe
                // (conflict with regular surface velocity, missing/!child curve), so
                // keying off it would suppress every warning. The inner checks below
                // re-derive the specific reason from the source.
                if (ssrc && ssrc->exists(bodyKey) &&
                    ssrc->hasSchema(bodyKey, ssrc->internToken(
                        PhysxSchemaTokens->PhysxSplinesSurfaceVelocityAPI.GetString())))
                {
                    bool splinesEnabledAuthored = true;
                    internal::getValue<bool>(mAttachedStage, bodyKey, PhysxSchemaTokens->physxSplinesSurfaceVelocitySurfaceVelocityEnabled,
                                             UsdTimeCode::Default(), splinesEnabledAuthored);

                    if (dyn->surfaceVelocityEnabled && splinesEnabledAuthored)
                    {
                        CARB_LOG_ERROR(
                            "Detected rigid body (%s) with both surface velocity and splines surface velocity, please disable one.",
                            path.GetText());
                    }
                    else if (splinesEnabledAuthored)
                    {
                        // Distinguish an absent relationship from a defined-but-empty one
                        // (matches the prior `!splinesRel || splinesList.empty()` gate).
                        const omni::physics::parse::TokenId curveRelTok =
                            ssrc->internToken(PhysxSchemaTokens->physxSplinesSurfaceVelocitySurfaceVelocityCurve.GetString());
                        const bool hasCurveRel = ssrc->hasRelationship(bodyKey, curveRelTok);
                        std::vector<omni::physics::parse::ObjectKey> curveKeys;
                        if (hasCurveRel)
                            ssrc->getRelationshipTargets(bodyKey, curveRelTok, curveKeys);
                        if (!hasCurveRel || curveKeys.empty())
                        {
                            CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined.",
                                           path.GetText());
                        }
                        else
                        {
                            const omni::physics::parse::ObjectKey curveKey = curveKeys[0];
                            if (!ssrc->exists(curveKey) ||
                                !ssrc->isA(curveKey, schemaTypeToken<UsdGeomBasisCurves>(*ssrc)))
                            {
                                CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined.",
                                               path.GetText());
                            }
                            else
                            {
                                // Walk the curve's ancestors for the body (no UsdPrim).
                                bool parentBodyFound = false;
                                const omni::physics::parse::ObjectKey root = ssrc->getRootKey();
                                for (omni::physics::parse::ObjectKey p = ssrc->getParent(curveKey);
                                     p.valid() && p != root; p = ssrc->getParent(p))
                                {
                                    if (p == bodyKey) { parentBodyFound = true; break; }
                                }
                                if (!parentBodyFound)
                                    CARB_LOG_ERROR("Splines surface velocity %s spline curve is not a child of the rigid body.",
                                                   path.GetText());
                            }
                        }
                    }
                }

                // Re-key the splines curve ObjectKey from the scan namespace into
                // the attached-stage namespace (mirrors the meshPrimKey re-key above).
                // The spline surface-velocity consumer in setupActor resolves the
                // curve prim through this key via getAttachedStage().pathFor().
                {
                    if (dyn->splinesCurvePrimKey.valid())
                        dyn->splinesCurvePrimKey =
                            mAttachedStage.keyFor(scanned.pathFor(dyn->splinesCurvePrimKey));
                }

                // Time-varying transform → kinematic auto-conversion (source-routed
                // equivalent of the legacy primutils::IsTransformTimeVarying walk).
                if (ssrc && ssrc->mightWorldTransformBeTimeVarying(bodyKey))
                {
                    DynamicPhysxRigidBodyDesc* dyn = static_cast<DynamicPhysxRigidBodyDesc*>(scanDesc);
                    dyn->hasTimeSampledXform = true;
                    if (!dyn->kinematicBody)
                    {
                        CARB_LOG_WARN("Detected rigid body that is not kinematic but does have parents with animated xformOps. Prim: %s, converting the body to kinematic body",
                                      path.GetText());
                        dyn->kinematicBody = true;
                    }
                    mAttachedStage.getAnimatedKinematicBodies()[path] = mAttachedStage.keyFor(path);
                }
            }

            PhysxRigidBodyDesc* desc = bodyUPtr.release();
            if (!desc)
                continue;

            // Schema-API flag for the rigid body, consumed by downstream
            // property-update paths (e.g. PhysXPointInstancerPropertiesUpdate
            // gates the per-instance shape-pose update on eRigidBodyAPI).
            mAttachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eRigidBodyAPI);

            BodyDescAndColliders& bdDesc = mBodyMap[path];
            bdDesc.desc = desc;
            // Translate sourceCollisions ObjectKeys → SdfPath set for
            // downstream body / collider lookups.
            bdDesc.collisions.clear();
            for (const auto& ck : scanDesc->sourceCollisions)
            {
                const SdfPath sp = scanned.pathFor(ck);
                if (!sp.IsEmpty())
                    bdDesc.collisions.insert(sp);
            }

            callbacks::collectRigidBodyTimeSampledCallbacks(mAttachedStage, mAttachedStage.keyFor(path), cbList);
        }

        // Time-sampled callback re-registration for tendon attributes
        // (6F.4).  Legacy parsers called `getAttribute(value, attr, min,
        // max, updateFn)` which both read the value AND registered a
        // time-sampled callback when `attr.ValueMightBeTimeVarying()`.
        // The 6F parse-lib parsers do the value read; this consumer-side
        // pass re-registers the callbacks using the engine's
        // `attachedStage.registerTimeSampledAttribute` API, matching
        // legacy semantics exactly:
        //   - Scalar / bool attributes (via `getAttribute` /
        //     `getBoolAttribute`): register when HasAuthoredValue AND
        //     ValueMightBeTimeVarying.
        //   - localPos / axis-gearing (registered inline in legacy
        //     parseAttachment / parseAxes): register on
        //     ValueMightBeTimeVarying alone, no HasAuthoredValue gate.
        //
        // See REQ-PARSE-TENDON-001 AC-6 and REQ-PARSE-TENDON-002 AC-6.
        // Authored/time-varying gates route through the source
        // (hasAuthoredAttribute == attr && HasAuthoredValue;
        // mightBeTimeVarying == attr && ValueMightBeTimeVarying). The prim is
        // used only for the attribute path (registerTimeSampledAttribute is the
        // engine-side sink, keyed by SdfPath), not for a value read.
        const omni::physics::parse::IPhysicsSource* tsSrc = mAttachedStage.getSource();
        auto regScalarIfAuthored = [&](const SdfPath& primPath, const std::string& attrName,
                                       OnUpdateObjectFn fn) {
            if (!tsSrc)
                return;
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(primPath);
            const omni::physics::parse::TokenId tok = tsSrc->internToken(attrName);
            if (tsSrc->hasAuthoredAttribute(key, tok) && tsSrc->mightBeTimeVarying(key, tok))
                mAttachedStage.registerTimeSampledAttribute(primPath.AppendProperty(TfToken(attrName)), fn);
        };
        auto regAnyIfTimeVarying = [&](const SdfPath& primPath, const std::string& attrName,
                                       OnUpdateObjectFn fn) {
            if (!tsSrc)
                return;
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(primPath);
            if (tsSrc->mightBeTimeVarying(key, tsSrc->internToken(attrName)))
                mAttachedStage.registerTimeSampledAttribute(primPath.AppendProperty(TfToken(attrName)), fn);
        };

        // ----- Spatial tendon attachments --------------------------
        // Walker pass-3c emitted a flat list of attachment descriptors
        // (root + intermediate + leaf, distinguished by `type`) into
        // scanned.spatialTendonAttachments.  Translate to the engine
        // descriptor types (SdfPath / TfToken instead of ObjectKey /
        // TokenId) and populate the legacy maps `mTendonAttachmentMap` +
        // `mSpatialTendons` that `createSpatialTendons` consumes.
        // Hierarchy resolution + change-tracking registration also run
        // here.  See REQ-PARSE-TENDON-001 AC-6.
        for (const auto& scanAtt : scanned.spatialTendonAttachments)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:spatialTendon");
            const TfToken instanceTok = scanned.tfTokenFor(scanAtt->instanceToken);
            const std::string instanceStr = instanceTok.GetString();
            const SdfPath linkPath = scanned.pathFor(scanAtt->linkKey);

            if (scanAtt->type == eTendonAttachmentRoot)
            {
                const auto& srcRoot = static_cast<const omni::physics::parse::PhysxTendonSpatialDesc&>(*scanAtt);
                auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonSpatialDesc)();
                engineDesc->type           = eTendonAttachmentRoot;
                engineDesc->gearing        = srcRoot.gearing;
                engineDesc->localPos       = srcRoot.localPos;
                engineDesc->linkPath       = linkPath;
                engineDesc->parentPath     = scanned.pathFor(srcRoot.parentKey);
                engineDesc->parentToken    = scanned.tfTokenFor(srcRoot.parentToken);
                engineDesc->instanceToken  = instanceTok;
                engineDesc->isEnabled      = srcRoot.isEnabled;
                engineDesc->stiffness      = srcRoot.stiffness;
                engineDesc->limitStiffness = srcRoot.limitStiffness;
                engineDesc->damping        = srcRoot.damping;
                engineDesc->offset         = srcRoot.offset;
                std::shared_ptr<PhysxTendonSpatialDesc> ptr(
                    engineDesc, [](PhysxTendonSpatialDesc* p) { ICE_FREE(p); });
                mSpatialTendons.push_back(ptr);
                registerSpatialTendonChangeParams(mAttachedStage, instanceStr);
                // PhysxTendonAttachmentRootAPI auto-applies
                // PhysxTendonAttachmentAPI with the same instance name
                // (see schema `apiSchemas`); register gearing + localPos
                // change tracking for the inherited attrs.
                registerTendonAttachmentChangeParams(mAttachedStage, instanceStr);

                // Time-sampled callbacks (root tendon attributes + localPos).
                if (tsSrc && tsSrc->exists(mAttachedStage.keyFor(linkPath)))
                {
                    const std::string base = "physxTendon:" + instanceStr + ":";
                    regScalarIfAuthored(linkPath, base + "stiffness",      updateSpatialTendonStiffness);
                    regScalarIfAuthored(linkPath, base + "limitStiffness", updateSpatialTendonLimitStiffness);
                    regScalarIfAuthored(linkPath, base + "damping",        updateSpatialTendonDamping);
                    regScalarIfAuthored(linkPath, base + "offset",         updateSpatialTendonOffset);
                    regScalarIfAuthored(linkPath, base + "tendonEnabled",  updateSpatialTendonEnabled);
                    regAnyIfTimeVarying(linkPath, base + "localPos",       updateTendonAttachmentLocalPos);
                }
            }
            else if (scanAtt->type == eTendonAttachmentLeaf)
            {
                const auto& srcLeaf = static_cast<const omni::physics::parse::PhysxTendonAttachmentLeafDesc&>(*scanAtt);
                auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonAttachmentLeafDesc)();
                engineDesc->type          = eTendonAttachmentLeaf;
                engineDesc->gearing       = srcLeaf.gearing;
                engineDesc->localPos      = srcLeaf.localPos;
                engineDesc->linkPath      = linkPath;
                engineDesc->parentPath    = scanned.pathFor(srcLeaf.parentKey);
                engineDesc->parentToken   = scanned.tfTokenFor(srcLeaf.parentToken);
                engineDesc->instanceToken = instanceTok;
                engineDesc->restLength    = srcLeaf.restLength;
                engineDesc->lowLimit      = srcLeaf.lowLimit;
                engineDesc->highLimit     = srcLeaf.highLimit;
                std::shared_ptr<PhysxTendonAttachmentLeafDesc> ptr(
                    engineDesc, [](PhysxTendonAttachmentLeafDesc* p) { ICE_FREE(p); });
                mTendonAttachmentMap[engineDesc->parentPath].push_back(ptr);
                registerTendonAttachmentLeafChangeParams(mAttachedStage, instanceStr);
                // PhysxTendonAttachmentLeafAPI auto-applies
                // PhysxTendonAttachmentAPI with the same instance name
                // (see schema `apiSchemas`); register gearing + localPos
                // change tracking for the inherited attrs.
                registerTendonAttachmentChangeParams(mAttachedStage, instanceStr);

                // Time-sampled callbacks: leaf shares gearing + localPos
                // with intermediate attachments, plus its own restLength /
                // lowerLimit / upperLimit (matches legacy
                // parseLeafAttachment → parseAttachment chain).
                if (tsSrc && tsSrc->exists(mAttachedStage.keyFor(linkPath)))
                {
                    const std::string base = "physxTendon:" + instanceStr + ":";
                    regScalarIfAuthored(linkPath, base + "gearing",     updateTendonAttachmentGearing);
                    regAnyIfTimeVarying(linkPath, base + "localPos",    updateTendonAttachmentLocalPos);
                    regScalarIfAuthored(linkPath, base + "restLength",  updateTendonAttachmentLeafRestLength);
                    regScalarIfAuthored(linkPath, base + "lowerLimit",  updateTendonAttachmentLeafLowLimit);
                    regScalarIfAuthored(linkPath, base + "upperLimit",  updateTendonAttachmentLeafHighLimit);
                }
            }
            else
            {
                auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonAttachmentDesc)();
                engineDesc->type          = scanAtt->type;
                engineDesc->gearing       = scanAtt->gearing;
                engineDesc->localPos      = scanAtt->localPos;
                engineDesc->linkPath      = linkPath;
                engineDesc->parentPath    = scanned.pathFor(scanAtt->parentKey);
                engineDesc->parentToken   = scanned.tfTokenFor(scanAtt->parentToken);
                engineDesc->instanceToken = instanceTok;
                std::shared_ptr<PhysxTendonAttachmentDesc> ptr(
                    engineDesc, [](PhysxTendonAttachmentDesc* p) { ICE_FREE(p); });
                mTendonAttachmentMap[engineDesc->parentPath].push_back(ptr);
                registerTendonAttachmentChangeParams(mAttachedStage, instanceStr);

                // Time-sampled callbacks: gearing (scalar) + localPos
                // (any).  Matches legacy parseAttachment exactly.
                if (tsSrc && tsSrc->exists(mAttachedStage.keyFor(linkPath)))
                {
                    const std::string base = "physxTendon:" + instanceStr + ":";
                    regScalarIfAuthored(linkPath, base + "gearing",  updateTendonAttachmentGearing);
                    regAnyIfTimeVarying(linkPath, base + "localPos", updateTendonAttachmentLocalPos);
                }
            }
        }

        // ----- DeformableBodies ------------------------------------
        // AC-17: route deformable bodies through scanStage emit +
        // convertScannedDeformableBody.  Mirrors the legacy
        // `parseDeformableBody` listener path closely; consumer-side
        // work that the parse library can't do (PhysX-extension
        // collision-mesh overlays, surface restBendAnglesDefault,
        // simMeshMaterial path, sceneId / filteredPair translation,
        // ObjectDatabase schema-flag tagging) is done here.
        for (size_t i = 0; i < scanned.deformables.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:deformableBody");
            const auto& scanDesc = scanned.deformables[i];
            const SdfPath path = scanned.pathFor(scanDesc->primKey);

            PhysxDeformableBodyDesc* desc = convert::convertScannedDeformableBody(scanned, i, mAttachedStage.getSourceUnits());
            if (!desc)
                continue;

            // Source-backed type/schema dispatch (single-apply schemas via
            // hasSchema, prim types via isA). Attribute-value reads below
            // (restBendAnglesDefault, PhysxCollisionAPI offsets) stay direct.
            const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();

            // Collision-geom path validation — mirrors
            // PhysicsBody.cpp::parseDeformableBody:856-899.  parse-lib
            // takes the first collisionGeomPath verbatim; the consumer
            // does the type-of-prim sanity check.
            if (desc->collisionMeshPath.IsEmpty())
            {
                CARB_LOG_WARN("No UsdPhysics.CollisionAPI found on deformable body prim or sub-tree, which is currently unsupported. "
                              "Parsing failed. Prim: %s", path.GetText());
                ICE_FREE(desc);
                continue;
            }
            const omni::physics::parse::ObjectKey collKey =
                src ? mAttachedStage.keyFor(desc->collisionMeshPath) : omni::physics::parse::ObjectKey{};
            if (desc->type == eVolumeDeformableBody)
            {
                if (!src || !src->isA(collKey, schemaTypeToken<UsdGeomTetMesh>(*src)))
                {
                    CARB_LOG_WARN("UsdPhysics.CollisionAPI on UsdGeomPointBased that are not UsdGeomTetMesh "
                                  "is currently not supported for volume deformables. Parsing failed. Prim: %s",
                                  path.GetText());
                    ICE_FREE(desc);
                    continue;
                }
            }
            else if (desc->type == eSurfaceDeformableBody)
            {
                if (!src || !src->isA(collKey, schemaTypeToken<UsdGeomMesh>(*src)))
                {
                    CARB_LOG_WARN("UsdPhysics.CollisionAPI on UsdGeomPointBased that are not UsdGeomMesh "
                                  "is currently not supported for surface deformables. Parsing failed. Prim: %s",
                                  path.GetText());
                    ICE_FREE(desc);
                    continue;
                }
                if (desc->collisionMeshPath != desc->simMeshPath)
                {
                    CARB_LOG_WARN("UsdPhysics.CollisionAPI found on different prim than UsdPhysics.SurfaceDeformableSimAPI, "
                                  "which is currently not supported for surface deformables. Parsing failed. Prim: %s",
                                  path.GetText());
                    ICE_FREE(desc);
                    continue;
                }
            }

            // Surface restBendAnglesDefault — read from sim-mesh prim's
            // SurfaceDeformableSimAPI attribute (a token).  Routed
            // through the source; mirrors PhysicsBody.cpp:842-854.
            if (desc->type == eSurfaceDeformableBody && src)
            {
                const omni::physics::parse::ObjectKey simKey = mAttachedStage.keyFor(desc->simMeshPath);
                if (src->hasSchema(simKey,
                                   src->internToken(OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsSurfaceDeformableSimAPI.GetString())))
                {
                    const auto rbaTok = src->internToken(OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestBendAnglesDefault.GetString());
                    omni::physics::parse::TokenId val{};
                    if (src->getAttribute(simKey, rbaTok, val) && val.valid())
                        static_cast<PhysxSurfaceDeformableBodyDesc*>(desc)->restBendAnglesDefault =
                            PXR_NS::TfToken(std::string(src->tokenToString(val)));
                }
            }

            // PhysxCollisionAPI overlay on collision-mesh prim —
            // contactOffset / restOffset with time-sample callbacks.
            // Routed through the source; mirrors PhysicsBody.cpp:904-965.
            if (src)
            {
                const auto coTok = src->internToken(PhysxSchemaTokens->physxCollisionContactOffset.GetString());
                const auto roTok = src->internToken(PhysxSchemaTokens->physxCollisionRestOffset.GetString());
                float contactOffset = desc->contactOffset;
                float restOffset = desc->restOffset;
                if (src->hasAuthoredAttribute(collKey, coTok))
                {
                    float v = contactOffset;
                    src->getAttribute(collKey, coTok, v);
                    if (!std::isinf(v) && v >= 0.0f && v <= FLT_MAX)
                        contactOffset = v;
                    if (src->isAttributeTimeSampled(collKey, coTok))
                        mAttachedStage.registerTimeSampledAttribute(
                            desc->collisionMeshPath.AppendProperty(PhysxSchemaTokens->physxCollisionContactOffset),
                            updateDeformableContactOffset);
                }
                if (src->hasAuthoredAttribute(collKey, roTok))
                {
                    float v = restOffset;
                    src->getAttribute(collKey, roTok, v);
                    if (!std::isinf(v) && v >= -FLT_MAX && v <= FLT_MAX)
                        restOffset = v;
                    if (src->isAttributeTimeSampled(collKey, roTok))
                        mAttachedStage.registerTimeSampledAttribute(
                            desc->collisionMeshPath.AppendProperty(PhysxSchemaTokens->physxCollisionRestOffset),
                            updateDeformableRestOffset);
                }
                if (contactOffset >= restOffset) desc->contactOffset = contactOffset;
                if (restOffset < contactOffset) desc->restOffset = restOffset;
                // collisionGroup is resolved later in the
                // createDeformableBody loop, after processScannedDescs
                // fully populates the CollisionGroupsMap.
            }

            // simMeshMaterial out-param: legacy reads from the schema
            // desc (populated by `getDeformableMaterialBinding`).  Re-
            // resolve via the source's material binding — parse-lib
            // doesn't snapshot this.
            SdfPath simMeshMaterial;
            if (src && !desc->simMeshPath.IsEmpty())
            {
                const omni::physics::parse::ObjectKey matKey =
                    src->getMaterialBinding(mAttachedStage.keyFor(desc->simMeshPath));
                if (matKey.valid())
                    simMeshMaterial = mAttachedStage.pathFor(matKey);
            }

            // Translate sourceFilteredCollisions → mFilteredPairs.
            for (const auto& fk : scanDesc->sourceFilteredCollisions)
            {
                const SdfPath fp = scanned.pathFor(fk);
                if (!fp.IsEmpty())
                    mFilteredPairs.push_back(std::make_pair(path, fp));
            }

            // Resolve first simulationOwner → sceneId.  Legacy ignores
            // owners beyond the first (PhysicsBody.cpp:988).
            if (!scanDesc->sourceSimulationOwners.empty())
            {
                const SdfPath sp = scanned.pathFor(scanDesc->sourceSimulationOwners[0]);
                if (!sp.IsEmpty())
                {
                    const ObjectId entry = mAttachedStage.getObjectDatabase()->findEntry(sp, eScene);
                    if (entry != kInvalidObjectId)
                        desc->sceneId = entry;
                }
            }

            // ObjectDatabase schema-flag tagging — duplicates the
            // legacy listener case (LoadStage.cpp:1043-1048) that ran
            // right after parseDeformableBody returned.
            if (desc->type == eVolumeDeformableBody)
                mAttachedStage.getObjectDatabase()->addSchemaAPI(desc->simMeshPath, SchemaAPIFlag::eVolumeDeformableSimAPI);
            else if (desc->type == eSurfaceDeformableBody)
                mAttachedStage.getObjectDatabase()->addSchemaAPI(desc->simMeshPath, SchemaAPIFlag::eSurfaceDeformableSimAPI);
            mAttachedStage.getObjectDatabase()->addSchemaAPI(desc->simMeshPath, SchemaAPIFlag::eDeformablePoseAPI);
            for (SdfPath skinGeomPath : desc->skinGeomPaths)
                mAttachedStage.getObjectDatabase()->addSchemaAPI(skinGeomPath, SchemaAPIFlag::eDeformablePoseAPI);

            // Schema-API flags on the body prim itself.  Mirrors trunk
            // LoadStage.cpp:869-895 (the per-prim switch case for
            // eVolumeDeformableBody / eSurfaceDeformableBody — deleted
            // wholesale in 6C, never restored).  Without
            // `eDeformableBodyAPI` on the body's path, callbacks like
            // `physicsDeformableBodyRequirementCheck` (ChangeRegister.cpp:204)
            // reject property-update routes for `mass` / `density` and
            // the runtime updates never reach PhysX (TestDeformables.cpp:1272).
            {
                mAttachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eDeformableBodyAPI);
                const omni::physics::parse::ObjectKey bodyKey =
                    src ? mAttachedStage.keyFor(path) : omni::physics::parse::ObjectKey{};
                if (src && src->exists(bodyKey))
                {
                    if (src->hasSchema(bodyKey, src->internToken(PhysxSchemaTokens->PhysxAutoDeformableBodyAPI.GetString())))
                    {
                        mAttachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eAutoDeformableBodyAPI);
                        if (src->hasSchema(bodyKey, src->internToken(PhysxSchemaTokens->PhysxAutoDeformableMeshSimplificationAPI.GetString())))
                            mAttachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eAutoDeformableMeshSimplificationAPI);
                        if (desc->type == eVolumeDeformableBody &&
                            src->hasSchema(bodyKey, src->internToken(PhysxSchemaTokens->PhysxAutoDeformableHexahedralMeshAPI.GetString())))
                            mAttachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eAutoDeformableHexahedralMeshAPI);
                    }
                }
            }

            mDeformableBodies.push_back({ path, desc, simMeshMaterial });
        }

        // ----- Articulations --------------------------------------
        // Mirrors legacy reportObjectDesc(eArticulation) +
        // parseArticulation in Articulation.cpp.  Each
        // ArticulationRootAPI-bearing prim emits one or more
        // PhysxArticulationDescs (one per rootPrim).  Multi-scene
        // owner check + body/joint/filter re-keying applied here so
        // downstream createArticulationLinks consumes engine-native
        // values.
        for (auto& scanArt : scanned.articulations)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:articulation");
            const SdfPath path = scanned.pathFor(scanArt->articulationPrim);

            if (mNumScenes > 1)
            {
                // Inlined checkArticulatonBodySimulationOwners — verify
                // all articulatedBodies share the same simulationOwner
                // and that the owner resolves to a registered scene.
                bool firstBody = true;
                SdfPath owner;
                bool ownerConsistent = true;
                for (const auto& bk : scanArt->articulatedBodies)
                {
                    const SdfPath bodyPath = scanned.pathFor(bk);
                    if (bodyPath.IsEmpty()) continue;
                    const SdfPath bodyOwner = getRigidBodySimulationOwner(mAttachedStage, bodyPath);
                    if (firstBody) { owner = bodyOwner; firstBody = false; }
                    else if (owner != bodyOwner)
                    {
                        CARB_LOG_ERROR("Articulation contains bodies with different simulation owners. Articulation: %s",
                                       path.GetText());
                        ownerConsistent = false;
                        break;
                    }
                }
                if (!ownerConsistent)
                    continue;
                if (!owner.IsEmpty() &&
                    mAttachedStage.getObjectDatabase()->findEntry(owner, eScene) == kInvalidObjectId)
                    continue;
            }

            PhysxArticulationDesc* desc = scanArt.release();
            if (!desc)
                continue;

            // Cross-namespace ObjectKey re-keying (AC-5).  Required for
            // every key flowing past the migration boundary — downstream
            // articulation creation does `attachedStage.pathFor(...)` and
            // would get empty paths from scan-source keys.
            auto rekey = [&](omni::physics::parse::ObjectKey k) {
                return k.valid() ? mAttachedStage.keyFor(scanned.pathFor(k)) : omni::physics::parse::ObjectKey{};
            };
            desc->articulationPrim = rekey(desc->articulationPrim);
            desc->rootPrim         = rekey(desc->rootPrim);
            desc->fixBaseKey      = rekey(desc->fixBaseKey);
            std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> rekJoints;
            std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> rekBodies;
            rekJoints.reserve(desc->articulatedJoints.size());
            rekBodies.reserve(desc->articulatedBodies.size());
            for (const auto& k : desc->articulatedJoints) rekJoints.insert(rekey(k));
            for (const auto& k : desc->articulatedBodies) rekBodies.insert(rekey(k));
            desc->articulatedJoints = std::move(rekJoints);
            desc->articulatedBodies = std::move(rekBodies);

            // Articulation-level filteredCollisions → mFilteredPairs.
            // Mirrors legacy `Articulation.cpp:193-196`.  Read from `desc`
            // (the released raw pointer), not `scanArt` (its DescPtr is
            // empty after release()).
            for (const auto& fk : desc->sourceFilteredCollisions)
            {
                const SdfPath fp = scanned.pathFor(fk);
                if (!fp.IsEmpty())
                    mFilteredPairs.push_back(std::make_pair(path, fp));
            }

            mArticulationMap[path].push_back(desc);
        }

        // ----- Joints ---------------------------------------------
        // Mirrors legacy reportObjectDesc(eJoint*) + parseJoint, plus
        // the multi-scene owner check from
        // PhysicsBody.cpp::checkJointBodySimulationOwners and the
        // body-transform-equality check legacy applies per joint type
        // (parse-lib can't compute that — it has no xformCache).
        for (auto& scanJoint : scanned.joints)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:joint");
            const SdfPath path = scanned.pathFor(scanJoint->jointPrimKey);
            const SdfPath body0Path = scanned.pathFor(scanJoint->body0);
            const SdfPath body1Path = scanned.pathFor(scanJoint->body1);

            if (mNumScenes > 1)
            {
                const SdfPath simOwner0 = getRigidBodySimulationOwner(mAttachedStage, body0Path);
                const SdfPath simOwner1 = getRigidBodySimulationOwner(mAttachedStage, body1Path);
                if (!body0Path.IsEmpty() && !body1Path.IsEmpty() && simOwner0 != simOwner1)
                {
                    CARB_LOG_ERROR("Cannot create joint between bodies that belong to different owners. Joint: %s",
                                   path.GetText());
                    continue;
                }
                const SdfPath owner = !simOwner0.IsEmpty() ? simOwner0 : simOwner1;
                if (!owner.IsEmpty() &&
                    mAttachedStage.getObjectDatabase()->findEntry(owner, eScene) == kInvalidObjectId)
                    continue;
            }

            PhysxJointDesc* desc = nullptr;
            // For eJointCustom, parse-lib's CustomPhysxJointDesc has a
            // different field layout than the consumer-side
            // `usdparser::CustomPhysxJointDesc` (TokenId vs TfToken,
            // ObjectKey vs SdfPath on Gear/Rack subclasses), so a
            // `static_cast` from the parse-lib subclass to the consumer
            // subclass would read past the parse-lib allocation. Detect
            // gear / rack / registered-custom from the prim type and
            // allocate the consumer-side typed desc directly from the
            // base fields. Mirrors Joint.cpp:713-779.
            // Gear / rack are now first-class typed joints captured by the
            // walker (parse-lib GearPhysxJointDesc/RackPhysxJointDesc, ObjectKey
            // targets). Convert to the engine desc (SdfPath targets) without any
            // USD re-read. Third-party custom joints still resolve via the
            // CustomJointManager below.
            if (scanJoint->type == eJointGear)
            {
                auto* gearDesc = ICE_PLACEMENT_NEW(GearPhysxJointDesc)();
                static_cast<PhysxJointDesc&>(*gearDesc) = *scanJoint;
                gearDesc->type = eJointGear;
                const auto* scanGear = static_cast<const omni::physics::parse::GearPhysxJointDesc*>(&*scanJoint);
                gearDesc->gearRatio = scanGear->gearRatio;
                // hinge keys are parse-time ScannedStage keys -> resolve via scanned.pathFor
                // (NOT the persistent AttachedStage source; different intern tables).
                gearDesc->hingePrimPath0 = scanned.pathFor(scanGear->hingePrimPath0);
                gearDesc->hingePrimPath1 = scanned.pathFor(scanGear->hingePrimPath1);
                desc = gearDesc;
            }
            else if (scanJoint->type == eJointRackAndPinion)
            {
                auto* rackDesc = ICE_PLACEMENT_NEW(RackPhysxJointDesc)();
                static_cast<PhysxJointDesc&>(*rackDesc) = *scanJoint;
                rackDesc->type = eJointRackAndPinion;
                const auto* scanRack = static_cast<const omni::physics::parse::RackPhysxJointDesc*>(&*scanJoint);
                rackDesc->ratio = scanRack->ratio;
                rackDesc->hingePrimPath = scanned.pathFor(scanRack->hingePrimKey);
                rackDesc->prismaticPrimPath = scanned.pathFor(scanRack->prismaticPrimKey);
                desc = rackDesc;
            }
            else if (scanJoint->type == eJointCustom)
            {
                // Custom joints are keyed by the prim's raw type name (not a schema
                // hierarchy), so read it through the source (no UsdPrim).
                const omni::physics::parse::IPhysicsSource* csrc = mAttachedStage.getSource();
                if (csrc && csrc->exists(mAttachedStage.keyFor(path)))
                {
                    const TfToken typeName(
                        std::string(csrc->tokenToString(csrc->getTypeName(mAttachedStage.keyFor(path)))));
                    const auto& customMap = OmniPhysX::getInstance().getCustomJointManager().getCustomJointTypeMap();
                    if (customMap.find(typeName) != customMap.end())
                    {
                        auto* customDesc = ICE_PLACEMENT_NEW(CustomPhysxJointDesc)();
                        static_cast<PhysxJointDesc&>(*customDesc) = *scanJoint;
                        customDesc->type = eJointCustom;
                        customDesc->customJointToken = typeName;
                        desc = customDesc;
                    }
                }
                if (!desc)
                {
                    // Unrecognised custom joint — drop. Mirrors legacy
                    // default of returning nullptr from parseJoint.
                    continue;
                }
            }
            else
            {
                desc = scanJoint.release();
                if (!desc)
                    continue;
            }

            // Re-key body0/body1/rel0/rel1/jointPrimKey from scanStage's
            // internal source into the consumer's AttachedStage source —
            // downstream createJoint does attachedStage.pathFor() and would
            // get empty paths without this (REQ-PARSE-CONSUMER-001 AC-5).
            auto rekey = [&](omni::physics::parse::ObjectKey k) {
                return k.valid() ? mAttachedStage.keyFor(scanned.pathFor(k)) : omni::physics::parse::ObjectKey{};
            };
            desc->jointPrimKey = rekey(desc->jointPrimKey);
            desc->body0 = rekey(desc->body0);
            desc->body1 = rekey(desc->body1);
            desc->rel0  = rekey(desc->rel0);
            desc->rel1  = rekey(desc->rel1);

            // Per-joint-type body-transform equality check. Spherical,
            // prismatic, and revolute joints check position only; fixed and
            // D6 joints also check rotation. Revolute rotation is unconstrained
            // around the authored axis, so localRot mismatches are not treated
            // as disjointed-body diagnostics.
            bool checkPos = false, checkRot = false;
            unsigned char tmAxis = 0xff;
            switch (desc->type)
            {
            case eJointFixed:     checkPos = true; checkRot = true; break;
            case eJointSpherical: checkPos = true; break;
            case eJointPrismatic: checkPos = true; break;
            case eJointD6:        checkPos = true; checkRot = true; break;
            case eJointRevolute:
                checkPos = true;
                tmAxis = static_cast<const RevolutePhysxJointDesc*>(desc)->axis;
                break;
            default: break;
            }
            if (checkPos || checkRot)
            {
                // Body world transforms via the source (EarliestTime, matching
                // the replaced mXfCache); validity via the source mirrors the
                // prior prim.IsValid() gate (no UsdPrim).
                const omni::physics::parse::IPhysicsSource* jbSrc = mAttachedStage.getSource();
                const bool body0Valid = jbSrc && jbSrc->exists(mAttachedStage.keyFor(body0Path));
                const bool body1Valid = jbSrc && jbSrc->exists(mAttachedStage.keyFor(body1Path));
                const GfMatrix4d body0World =
                    body0Valid ? internal::getWorldTransform(mAttachedStage, mAttachedStage.keyFor(body0Path)) : GfMatrix4d(1.0);
                const GfMatrix4d body1World =
                    body1Valid ? internal::getWorldTransform(mAttachedStage, mAttachedStage.keyFor(body1Path)) : GfMatrix4d(1.0);
                desc->validBodyTransformations = primutils::isBodyTransformEqual(
                    body0World, body0Valid, body1World, body1Valid,
                    toVec3f(desc->localPose0Position), toQuatf(desc->localPose0Orientation),
                    toVec3f(desc->localPose1Position), toQuatf(desc->localPose1Orientation),
                    OmniPhysX::getInstance().getCachedSettings().jointBodyTransformCheckTolerance,
                    checkPos, checkRot, tmAxis);
            }

            JointDescAndPath jd;
            jd.path = path;
            jd.desc = desc;
            jd.articulationJoint = false;
            jd.index = uint32_t(mJointVector.size());
            mJointPathIndexMap[path] = mJointVector.size();
            mJointVector.push_back(jd);

            // Mimic + tendon parsing moved into the native walker
            // (6F.1 mimic, 6F.3 fixed tendons); the unsupported-joint
            // tendon-axis warning is preserved here since only this loop
            // has the joint type bucketing in front of it. Source-routed (no UsdPrim).
            const omni::physics::parse::IPhysicsSource* jsrc = mAttachedStage.getSource();
            if (jsrc && jsrc->exists(mAttachedStage.keyFor(path)))
            {
                switch (desc->type)
                {
                case eJointFixed:
                case eJointSpherical:
                case eJointDistance:
                {
                    if (jsrc->hasSchema(mAttachedStage.keyFor(path),
                                        schemaTypeToken<PhysxSchemaPhysxTendonAxisAPI>(*jsrc)))
                    {
                        CARB_LOG_WARN("A Tendon Axis API was applied to an unsupported joint type at %s!",
                                      path.GetText());
                    }
                    break;
                }
                default: break;
                }
            }
        }

        // ----- Mimic joints ----------------------------------------
        // Walker pass-3b emitted parse-lib MimicJointDescs into
        // scanned.mimicJoints (with ObjectKey paths).  Translate to the
        // engine descriptor type (SdfPath paths) and append to mMimicJoints
        // for createMimicJoints to consume.  See REQ-PARSE-MIMIC-001 AC-6.
        for (const auto& scanMimic : scanned.mimicJoints)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:mimicJoint");
            MimicJointDesc engineDesc;
            engineDesc.type                 = scanMimic->type;
            engineDesc.mimicJointPath       = scanned.pathFor(scanMimic->mimicJointKey);
            engineDesc.mimicJointAxis       = scanMimic->mimicJointAxis;
            engineDesc.referenceJointPath   = scanned.pathFor(scanMimic->referenceJointKey);
            engineDesc.referenceJointAxis   = scanMimic->referenceJointAxis;
            engineDesc.gearing              = scanMimic->gearing;
            engineDesc.offset               = scanMimic->offset;
            engineDesc.naturalFrequency     = scanMimic->naturalFrequency;
            engineDesc.dampingRatio         = scanMimic->dampingRatio;
            mMimicJoints.push_back(engineDesc);
        }

        // ----- Fixed tendons --------------------------------------
        // Walker pass-3d emitted parse-lib PhysxTendonAxisDescs +
        // PhysxTendonFixedDescs into scanned.fixedTendonAxes /
        // scanned.fixedTendons (with ObjectKey paths + TokenId
        // instance names).  Translate to engine descriptor types
        // (SdfPath / TfToken) and populate the legacy maps that
        // `createFixedTendons` consumes.  Cross-reference
        // `PhysxTendonFixedDesc::rootAxis` is wired by matching
        // jointKey + instanceToken against mTendonAxisMap.
        // See REQ-PARSE-TENDON-002 AC-6.
        std::unordered_map<TfToken, std::shared_ptr<PhysxTendonAxisDesc>, TfToken::HashFunctor>
            axisByInstance; // jointKey:instance → engine axis ptr (for rootAxis wiring)
        for (const auto& scanAxis : scanned.fixedTendonAxes)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:fixedTendonAxis");
            const TfToken instanceTok = scanned.tfTokenFor(scanAxis->instanceToken);
            const SdfPath jointKey = scanned.pathFor(scanAxis->jointKey);
            const SdfPath link0     = scanned.pathFor(scanAxis->link0);
            const SdfPath link1     = scanned.pathFor(scanAxis->link1);

            auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonAxisDesc)();
            engineDesc->type              = scanAxis->type;
            engineDesc->instanceToken     = instanceTok;
            engineDesc->jointPath         = jointKey;
            engineDesc->link0             = link0;
            engineDesc->link1             = link1;
            engineDesc->gearings          = scanAxis->gearings;
            engineDesc->forceCoefficients = scanAxis->forceCoefficients;
            engineDesc->axes              = scanAxis->axes;
            engineDesc->parentAxisId      = kInvalidObjectId;
            engineDesc->wasVisited        = false;

            std::shared_ptr<PhysxTendonAxisDesc> ptr(
                engineDesc, [](PhysxTendonAxisDesc* p) { ICE_FREE(p); });

            mTendonAxisMap[link0].push_back(ptr);
            mTendonAxisMap[link1].push_back(ptr);
            mTendonAxisMap[jointKey].push_back(ptr);

            // The "axis by instance on this joint" lookup is used below
            // to wire PhysxTendonFixedDesc::rootAxis.  Use a composite
            // key of jointKey + ":" + instance.
            const TfToken composite(jointKey.GetString() + ":" + instanceTok.GetString());
            axisByInstance.emplace(composite, ptr);

            registerTendonAxisChangeParam(mAttachedStage, instanceTok.GetString());

            // Time-sampled callback for the axis gearing.  Matches legacy
            // parseAxes: ValueMightBeTimeVarying only (no HasAuthoredValue
            // gate — legacy registers via `gearingAttr.ValueMightBeTimeVarying()`).
            if (tsSrc && tsSrc->exists(mAttachedStage.keyFor(jointKey)))
            {
                const std::string base = "physxTendon:" + instanceTok.GetString() + ":";
                regAnyIfTimeVarying(jointKey, base + "gearing", updateTendonAxisSingleGearing);
            }
        }

        for (const auto& scanTendon : scanned.fixedTendons)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:fixedTendon");
            const TfToken instanceTok = scanned.tfTokenFor(scanTendon->instanceToken);
            const SdfPath jointKey   = scanned.pathFor(scanTendon->jointKey);

            auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonFixedDesc)();
            engineDesc->type            = eTendonFixed;
            engineDesc->instanceToken   = instanceTok;
            engineDesc->jointPath       = jointKey;
            engineDesc->stiffness       = scanTendon->stiffness;
            engineDesc->damping         = scanTendon->damping;
            engineDesc->restLength      = scanTendon->restLength;
            engineDesc->offset          = scanTendon->offset;
            engineDesc->limitStiffness  = scanTendon->limitStiffness;
            engineDesc->lowLimit        = scanTendon->lowLimit;
            engineDesc->highLimit       = scanTendon->highLimit;
            engineDesc->isEnabled       = scanTendon->isEnabled;
            engineDesc->rootAxis        = nullptr;

            // Wire rootAxis by matching jointKey + instance.
            const TfToken composite(jointKey.GetString() + ":" + instanceTok.GetString());
            auto axisIt = axisByInstance.find(composite);
            if (axisIt != axisByInstance.end())
                engineDesc->rootAxis = axisIt->second.get();

            std::shared_ptr<PhysxTendonFixedDesc> ptr(
                engineDesc, [](PhysxTendonFixedDesc* p) { ICE_FREE(p); });
            mFixedTendons.push_back(ptr);

            registerFixedTendonChangeParams(mAttachedStage, instanceTok.GetString());

            // Time-sampled callbacks for the fixed tendon root attributes.
            // Matches legacy parseFixedTendon's getAttribute chain
            // (HasAuthoredValue + ValueMightBeTimeVarying).
            if (tsSrc && tsSrc->exists(mAttachedStage.keyFor(jointKey)))
            {
                const std::string base = "physxTendon:" + instanceTok.GetString() + ":";
                regScalarIfAuthored(jointKey, base + "stiffness",      updateFixedTendonStiffness);
                regScalarIfAuthored(jointKey, base + "limitStiffness", updateFixedTendonLimitStiffness);
                regScalarIfAuthored(jointKey, base + "damping",        updateFixedTendonDamping);
                regScalarIfAuthored(jointKey, base + "offset",         updateFixedTendonOffset);
                regScalarIfAuthored(jointKey, base + "tendonEnabled",  updateFixedTendonEnabled);
                regScalarIfAuthored(jointKey, base + "restLength",     updateFixedTendonRestLength);
                regScalarIfAuthored(jointKey, base + "lowerLimit",     updateFixedTendonLowLimit);
                regScalarIfAuthored(jointKey, base + "upperLimit",     updateFixedTendonHighLimit);
            }
        }

        // ----- DeformableAttachments -------------------------------
        // Per-prim attachment desc, source-key cross-refs translated
        // to SdfPath via scanned.pathFor().  Mirrors legacy
        // reportObjectDesc(eAttachment*) case + parseDeformableAttachment.
        for (const auto& scanAtt : scanned.attachments)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:deformableAttachment");
            const SdfPath path = scanned.pathFor(scanAtt->primKey);
            auto* outDesc = ICE_PLACEMENT_NEW(PhysxDeformableAttachmentDesc)();
            outDesc->type      = scanAtt->type;
            outDesc->enabled   = scanAtt->enabled;
            outDesc->src0      = scanned.pathFor(scanAtt->src0);
            outDesc->src1      = scanned.pathFor(scanAtt->src1);
            outDesc->stiffness = scanAtt->stiffness;
            outDesc->damping   = scanAtt->damping;

            DeformableAttachmentDescAndPath entry;
            entry.path = path;
            entry.desc = outDesc;
            mDeformableAttachmentVector.push_back(entry);
        }

        // ----- DeformableCollisionFilters --------------------------
        for (const auto& scanFilt : scanned.deformableCollisionFilters)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:deformableCollisionFilter");
            const SdfPath path = scanned.pathFor(scanFilt->primKey);
            auto* outDesc = ICE_PLACEMENT_NEW(PhysxDeformableCollisionFilterDesc)();
            outDesc->enabled = scanFilt->enabled;
            outDesc->src0    = scanned.pathFor(scanFilt->src0);
            outDesc->src1    = scanned.pathFor(scanFilt->src1);

            DeformableCollisionFilterDescAndPath entry;
            entry.path = path;
            entry.desc = outDesc;
            mDeformableCollisionFilterVector.push_back(entry);
        }

        // ----- CCTs -----------------------------------------------
        // Character controllers — scanned.ccts is populated by the
        // native walker for every prim with PhysxCharacterControllerAPI
        // applied (and capsule geometry).  Gated by `eParseInternal` so
        // the inspector / single-scene-force settings (which drop
        // mParsingFlags = 0) skip CCT creation, matching legacy.  Scene
        // resolution happens here, after scenes have been created and
        // registered with ObjectDatabase via the scene loop above.
        if (mParsingFlags & ParsingFlag::eParseInternal)
        {
            const ObjectDb& db = *mAttachedStage.getObjectDatabase();
            for (const auto& scanCct : scanned.ccts)
            {
                CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:cct");
                const SdfPath path = scanned.pathFor(scanCct->primKey);

                if (scanCct->sourceSimulationOwner.valid())
                {
                    const SdfPath ownerPath = scanned.pathFor(scanCct->sourceSimulationOwner);
                    const ObjectId sceneId = db.findEntry(ownerPath, eScene);
                    if (sceneId == kInvalidObjectId)
                    {
                        CARB_LOG_ERROR("parseCct: Failed to find physics simulation owner \"%s\".",
                                       ownerPath.GetText());
                        continue;
                    }
                    scanCct->sceneId = sceneId;
                }
                createObject(mAttachedStage, path, scanCct.get(), /*deleteDesc=*/false);
            }
        }

        // ----- Tire friction tables (7A.1) -------------------------
        // Walker emitted parse-lib TireFrictionTableDescs into
        // scanned.tireFrictionTables (with ObjectKey paths).  Translate
        // to engine descriptor type (SdfPath paths) and stash in
        // mTireFrictionTableDescList for the post-load creation block
        // (which still owns ObjectDatabase material-id resolution +
        // engine createObject).  See REQ-PARSE-VEH-TIREFRICTION-001 AC-5.
        for (const auto& scanTft : scanned.tireFrictionTables)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:tireFrictionTable");
            auto* engineDesc = ICE_PLACEMENT_NEW(TireFrictionTableDesc)();
            engineDesc->path                 = scanned.pathFor(scanTft->key);
            engineDesc->defaultFrictionValue = scanTft->defaultFrictionValue;
            engineDesc->frictionValues       = scanTft->frictionValues;
            engineDesc->materialPaths.reserve(scanTft->materialPaths.size());
            for (const omni::physics::parse::ObjectKey matKey : scanTft->materialPaths)
                engineDesc->materialPaths.push_back(scanned.pathFor(matKey));
            mTireFrictionTableDescList.push_back(engineDesc);
        }

        // ----- Vehicle contexts (7A.1) -----------------------------
        // Walker emitted parse-lib VehicleContextDescs.  Translate to
        // engine descriptor type and stash for the post-load
        // setVehicleContext block.  See REQ-PARSE-VEH-CONTEXT-001 AC-5.
        for (const auto& scanCtx : scanned.vehicleContexts)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleContext");
            VehicleContextDesc engineDesc;
            engineDesc.scenePath          = scanned.pathFor(scanCtx->sceneKey);
            engineDesc.vehicleUpdateMode  = static_cast<VehicleUpdateMode>(scanCtx->vehicleUpdateMode);
            engineDesc.upAxis             = scanCtx->upAxis;
            engineDesc.forwardAxis        = scanCtx->forwardAxis;
            engineDesc.verticalAxis       = static_cast<VehicleContextDesc::AxisDir>(scanCtx->verticalAxis);
            engineDesc.longitudinalAxis   = static_cast<VehicleContextDesc::AxisDir>(scanCtx->longitudinalAxis);
            mVehicleContextDescList.push_back(engineDesc);
        }

        // ----- Vehicle shareable components (7A.2) -----------------
        // Walker emitted parse-lib WheelDesc / TireDesc / SuspensionDesc
        // into scanned.vehicleWheels / vehicleTires / vehicleSuspensions
        // (ObjectKey paths).  Translate to engine descriptors and
        // pre-populate the stage-level VehicleComponentTracker.  The
        // legacy parseVehicle path looks up entries via SdfPath and
        // short-circuits when found, so per-component legacy parsers
        // (parseWheel/parseTire/parseSuspension) are unreachable during
        // vehicle parsing.
        for (const auto& scanWheel : scanned.vehicleWheels)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleWheel");
            const SdfPath path = scanned.pathFor(scanWheel->key);
            if (mVehicleComponentTracker.mWheels.find(path) != mVehicleComponentTracker.mWheels.end())
                continue;
            WheelDesc* engineDesc = ICE_PLACEMENT_NEW(WheelDesc)();
            engineDesc->path                  = path;
            engineDesc->radius                = scanWheel->radius;
            engineDesc->width                 = scanWheel->width;
            engineDesc->mass                  = scanWheel->mass;
            engineDesc->moi                   = scanWheel->moi;
            engineDesc->dampingRate           = scanWheel->dampingRate;
            engineDesc->maxBrakeTorque        = scanWheel->maxBrakeTorque;
            engineDesc->maxHandBrakeTorque    = scanWheel->maxHandBrakeTorque;
            engineDesc->maxSteerAngle         = scanWheel->maxSteerAngle;
            engineDesc->toeAngle              = scanWheel->toeAngle;
            mVehicleComponentTracker.mWheels.insert({ path, engineDesc });
        }
        for (const auto& scanTire : scanned.vehicleTires)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleTire");
            const SdfPath path = scanned.pathFor(scanTire->key);
            if (mVehicleComponentTracker.mTires.find(path) != mVehicleComponentTracker.mTires.end())
                continue;
            TireDesc* engineDesc = ICE_PLACEMENT_NEW(TireDesc)();
            engineDesc->path                                = path;
            engineDesc->latStiffX                           = scanTire->latStiffX;
            engineDesc->latStiffY                           = scanTire->latStiffY;
            engineDesc->lateralStiffnessGraph               = scanTire->lateralStiffnessGraph;
            engineDesc->longitudinalStiffnessPerUnitGravity = scanTire->longitudinalStiffnessPerUnitGravity;
            engineDesc->longitudinalStiffness               = scanTire->longitudinalStiffness;
            engineDesc->camberStiffnessPerUnitGravity       = scanTire->camberStiffnessPerUnitGravity;
            engineDesc->camberStiffness                     = scanTire->camberStiffness;
            for (uint32_t i = 0; i < 3; ++i)
                engineDesc->frictionVsSlipGraph[i]          = scanTire->frictionVsSlipGraph[i];
            engineDesc->frictionTableId                     = kInvalidObjectId;
            engineDesc->frictionTablePath                   = scanned.pathFor(scanTire->frictionTableKey);
            engineDesc->restLoad                            = scanTire->restLoad;
            mVehicleComponentTracker.mTires.insert({ path, engineDesc });
        }
        for (const auto& scanSusp : scanned.vehicleSuspensions)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleSuspension");
            const SdfPath path = scanned.pathFor(scanSusp->key);
            if (mVehicleComponentTracker.mSuspensions.find(path) != mVehicleComponentTracker.mSuspensions.end())
                continue;
            SuspensionDesc* engineDesc = ICE_PLACEMENT_NEW(SuspensionDesc)();
            engineDesc->path                    = path;
            engineDesc->springStrength          = scanSusp->springStrength;
            engineDesc->springDamperRate        = scanSusp->springDamperRate;
            engineDesc->travelDistance          = scanSusp->travelDistance;
            engineDesc->maxCompression          = scanSusp->maxCompression;
            engineDesc->maxDroop                = scanSusp->maxDroop;
            engineDesc->camberAtRest            = scanSusp->camberAtRest;
            engineDesc->camberAtMaxCompression  = scanSusp->camberAtMaxCompression;
            engineDesc->camberAtMaxDroop        = scanSusp->camberAtMaxDroop;
            engineDesc->sprungMass              = scanSusp->sprungMass;
            mVehicleComponentTracker.mSuspensions.insert({ path, engineDesc });
        }

        // ----- Vehicle drivetrain components (7A.3) ----------------
        // Engine / Gears / Clutch.  AutoGearBox stays legacy until 7A.4
        // because parseAutoGearBox needs forwardGearCount from the
        // parent DriveStandard.gears rel target.
        for (const auto& scanEng : scanned.vehicleEngines)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleEngine");
            const SdfPath path = scanned.pathFor(scanEng->key);
            if (mVehicleComponentTracker.mEngines.find(path) != mVehicleComponentTracker.mEngines.end())
                continue;
            EngineDesc* engineDesc = ICE_PLACEMENT_NEW(EngineDesc)();
            engineDesc->path                                = path;
            engineDesc->moi                                 = scanEng->moi;
            engineDesc->peakTorque                          = scanEng->peakTorque;
            engineDesc->maxRotationSpeed                    = scanEng->maxRotationSpeed;
            engineDesc->idleRotationSpeed                   = scanEng->idleRotationSpeed;
            for (uint32_t i = 0; i < omni::physics::parse::EngineDesc::maxNumberOfTorqueCurvePoints; ++i)
                engineDesc->torqueCurve[i]                  = scanEng->torqueCurve[i];
            engineDesc->torqueCurvePointCount               = scanEng->torqueCurvePointCount;
            engineDesc->dampingRateFullThrottle             = scanEng->dampingRateFullThrottle;
            engineDesc->dampingRateZeroThrottleClutchEngaged    = scanEng->dampingRateZeroThrottleClutchEngaged;
            engineDesc->dampingRateZeroThrottleClutchDisengaged = scanEng->dampingRateZeroThrottleClutchDisengaged;
            mVehicleComponentTracker.mEngines.insert({ path, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleGears.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleGears");
            const SdfPath path = scanned.pathFor(scanned.vehicleGearsPaths[i]);
            if (mVehicleComponentTracker.mGears.find(path) != mVehicleComponentTracker.mGears.end())
                continue;
            GearsDesc* engineDesc = ICE_PLACEMENT_NEW(GearsDesc)();
            engineDesc->ratios     = scanned.vehicleGears[i]->ratios;
            engineDesc->ratioScale = scanned.vehicleGears[i]->ratioScale;
            engineDesc->switchTime = scanned.vehicleGears[i]->switchTime;
            mVehicleComponentTracker.mGears.insert({ path, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleClutches.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleClutch");
            const SdfPath path = scanned.pathFor(scanned.vehicleClutchPaths[i]);
            if (mVehicleComponentTracker.mClutches.find(path) != mVehicleComponentTracker.mClutches.end())
                continue;
            ClutchDesc* engineDesc = ICE_PLACEMENT_NEW(ClutchDesc)();
            engineDesc->strength = scanned.vehicleClutches[i]->strength;
            mVehicleComponentTracker.mClutches.insert({ path, engineDesc });
        }

        // ----- Vehicle NonlinearCmdResponse (7A.6) -----------------
        // Multi-apply per command instance.  Build a side-table keyed
        // by (ownerPath, instanceToken) so the Drive / Steering /
        // Brakes loops below can wire the nonlinearCmdResponse
        // pointer.  Engine-side descriptors land in the legacy
        // mNonlinearCmdResponses vector for cleanup.
        std::map<std::pair<SdfPath, TfToken>, NonlinearCmdResponseDesc*> ncrByOwnerAndInstance;
        for (size_t i = 0; i < scanned.vehicleNonlinearCmdResponses.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleNonlinearCmdResponse");
            const SdfPath path  = scanned.pathFor(scanned.vehicleNonlinearCmdResponsePaths[i]);
            const TfToken inst  = scanned.tfTokenFor(scanned.vehicleNonlinearCmdResponseInstanceTokens[i]);
            const auto& scanNcr = scanned.vehicleNonlinearCmdResponses[i];

            NonlinearCmdResponseDesc* engineDesc = ICE_PLACEMENT_NEW(NonlinearCmdResponseDesc)();
            engineDesc->commandValues                = scanNcr->commandValues;
            engineDesc->speedResponsesPerCommandValue = scanNcr->speedResponsesPerCommandValue;
            engineDesc->speedResponses               = scanNcr->speedResponses;

            mVehicleComponentTracker.mNonlinearCmdResponses.push_back(engineDesc);
            ncrByOwnerAndInstance.insert({ { path, inst }, engineDesc });
        }
        auto findNcr = [&ncrByOwnerAndInstance](const SdfPath& owner, const TfToken& inst) -> NonlinearCmdResponseDesc* {
            auto it = ncrByOwnerAndInstance.find({ owner, inst });
            return (it != ncrByOwnerAndInstance.end()) ? it->second : nullptr;
        };

        // ----- Vehicle DriveBasic + Differential + AutoGearBox (7A.4) ----------
        // DriveStandard with its cross-references to Engine/Gears/
        // AutoGearBox/Clutch is deferred to 7A.5 alongside Nonlinear-
        // CmdResponse migration.  Pre-populating DriveBasic here leaves
        // nonlinearCmdResponse=nullptr; the legacy create path tolerates
        // that (no nonlinear response curve applied at engine create
        // time).  7A.5 will wire it from the multi-apply schema.
        for (const auto& scanDrive : scanned.vehicleDrivesBasic)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleDriveBasic");
            const SdfPath path = scanned.pathFor(scanDrive->key);
            if (mVehicleComponentTracker.mDrivesBasic.find(path) != mVehicleComponentTracker.mDrivesBasic.end())
                continue;
            DriveBasicDesc* engineDesc = ICE_PLACEMENT_NEW(DriveBasicDesc)();
            engineDesc->path                = path;
            engineDesc->peakTorque          = scanDrive->peakTorque;
            engineDesc->nonlinearCmdResponse = findNcr(path, PhysxSchemaTokens->drive);  // 7A.6
            mVehicleComponentTracker.mDrivesBasic.insert({ path, engineDesc });
        }
        // DriveStandard pre-population.  Cross-references (engine /
        // gears / autoGearBox / clutch) resolve through the tracker
        // maps populated by 7A.3 / 7A.4 above.  If ANY required
        // cross-ref (engine / gears / clutch) failed to resolve —
        // typically because the per-component parser rejected the
        // authored data — skip pre-population entirely so the legacy
        // parseRelationshipOrAPI flow runs and reports the error path
        // identical to pre-7A.6 behavior.  AutoGearBox is optional, so
        // null pointer is fine there.
        for (size_t i = 0; i < scanned.vehicleDrivesStandard.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleDriveStandard");
            const SdfPath path = scanned.pathFor(scanned.vehicleDrivesStandardPaths[i]);
            if (mVehicleComponentTracker.mDrivesStandard.find(path) != mVehicleComponentTracker.mDrivesStandard.end())
                continue;
            const auto& refs = scanned.vehicleDrivesStandardCrossRefs[i];

            EngineDesc* engine = nullptr;
            GearsDesc*  gears  = nullptr;
            ClutchDesc* clutch = nullptr;
            AutoGearBoxDesc* autoGearBox = nullptr;

            if (refs.engineKey.valid())
            {
                auto it = mVehicleComponentTracker.mEngines.find(scanned.pathFor(refs.engineKey));
                if (it != mVehicleComponentTracker.mEngines.end()) engine = it->second;
            }
            if (refs.gearsKey.valid())
            {
                auto it = mVehicleComponentTracker.mGears.find(scanned.pathFor(refs.gearsKey));
                if (it != mVehicleComponentTracker.mGears.end()) gears = it->second;
            }
            if (refs.clutchKey.valid())
            {
                auto it = mVehicleComponentTracker.mClutches.find(scanned.pathFor(refs.clutchKey));
                if (it != mVehicleComponentTracker.mClutches.end()) clutch = it->second;
            }
            if (refs.autoGearBoxKey.valid())
            {
                auto it = mVehicleComponentTracker.mAutoGearBoxes.find(scanned.pathFor(refs.autoGearBoxKey));
                if (it != mVehicleComponentTracker.mAutoGearBoxes.end()) autoGearBox = it->second;
            }

            // Required refs missing → defer to legacy parser path.
            if (!engine || !gears || !clutch)
                continue;

            DriveStandardDesc* engineDesc = ICE_PLACEMENT_NEW(DriveStandardDesc)();
            engineDesc->engine       = engine;
            engineDesc->engineId     = kInvalidObjectId;
            engineDesc->gears        = gears;
            engineDesc->autoGearBox  = autoGearBox;
            engineDesc->clutch       = clutch;
            mVehicleComponentTracker.mDrivesStandard.insert({ path, engineDesc });
        }
        // ----- Vehicle SuspensionCompliance (7A.7) ------------------
        // SuspensionCompliance is per-attachment-prim.  Pre-populate
        // a SdfPath-keyed side-table and push to the legacy
        // mSuspensionCompliances vector for cleanup lifetime.  Legacy
        // parseWheelAttachment consults the side-table when the
        // PhysxVehicleSuspensionComplianceAPI is applied on the
        // attachment prim; short-circuits on hit.
        for (size_t i = 0; i < scanned.vehicleSuspensionCompliances.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleSuspensionCompliance");
            const SdfPath path = scanned.pathFor(scanned.vehicleSuspensionCompliancePaths[i]);
            if (mVehicleComponentTracker.mSuspensionComplianceByPath.find(path) !=
                mVehicleComponentTracker.mSuspensionComplianceByPath.end())
                continue;
            const auto& scanSc = scanned.vehicleSuspensionCompliances[i];
            SuspensionComplianceDesc* engineDesc = ICE_PLACEMENT_NEW(SuspensionComplianceDesc)();
            engineDesc->wheelToeAngleList           = scanSc->wheelToeAngleList;
            engineDesc->wheelCamberAngleList        = scanSc->wheelCamberAngleList;
            engineDesc->suspensionForceAppPointList = scanSc->suspensionForceAppPointList;
            engineDesc->tireForceAppPointList       = scanSc->tireForceAppPointList;
            mVehicleComponentTracker.mSuspensionCompliances.push_back(engineDesc);
            mVehicleComponentTracker.mSuspensionComplianceByPath.insert({ path, engineDesc });
        }
        // WheelAttachment full pre-population (7A.8).  Walker emitted
        // a descriptor + parallel WheelAttachmentInfo side-table with
        // the three rel-or-API cross-ref ObjectKeys (wheel / tire /
        // suspension).  Consumer adapter resolves to engine pointers
        // via the tracker maps populated in 7A.2; SuspensionCompliance
        // (when applied alongside) resolves through mSuspension-
        // ComplianceByPath populated above.  Required-refs missing →
        // skip pre-population so the legacy parser path runs and
        // reports the error identically (same safety pattern as 7A.6
        // DriveStandard).
        for (size_t i = 0; i < scanned.vehicleWheelAttachments.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleWheelAttachment");
            const SdfPath path = scanned.pathFor(scanned.vehicleWheelAttachments[i]->key);
            if (mVehicleComponentTracker.mWheelAttachmentByPath.find(path) !=
                mVehicleComponentTracker.mWheelAttachmentByPath.end())
                continue;
            const auto& info = scanned.vehicleWheelAttachmentInfos[i];

            WheelDesc* wheel = nullptr;
            TireDesc* tire = nullptr;
            SuspensionDesc* suspension = nullptr;
            if (info.wheelKey.valid())
            {
                auto it = mVehicleComponentTracker.mWheels.find(scanned.pathFor(info.wheelKey));
                if (it != mVehicleComponentTracker.mWheels.end()) wheel = it->second;
            }
            if (info.tireKey.valid())
            {
                auto it = mVehicleComponentTracker.mTires.find(scanned.pathFor(info.tireKey));
                if (it != mVehicleComponentTracker.mTires.end()) tire = it->second;
            }
            if (info.suspensionKey.valid())
            {
                auto it = mVehicleComponentTracker.mSuspensions.find(scanned.pathFor(info.suspensionKey));
                if (it != mVehicleComponentTracker.mSuspensions.end()) suspension = it->second;
            }
            if (!wheel || !tire || !suspension)
            {
                // Required component ref unresolved: leave it out of
                // mWheelAttachmentByPath. The owners list still records it, so
                // parseVehicle counts it and marks the vehicle invalid.
                CARB_LOG_ERROR("Usd Physics: wheel attachment \"%s\": a required %s%s%s reference is missing or does "
                               "not point to a prim with the matching vehicle component API applied.",
                               path.GetText(), wheel ? "" : "wheel ", tire ? "" : "tire ", suspension ? "" : "suspension ");
                continue;
            }

            const auto& scanWa = scanned.vehicleWheelAttachments[i];
            WheelAttachmentDesc* engineDesc = ICE_PLACEMENT_NEW(WheelAttachmentDesc)();
            engineDesc->path                         = path;
            engineDesc->id                           = kInvalidObjectId;
            engineDesc->state                        = scanWa->state;
            engineDesc->wheel                        = wheel;
            engineDesc->wheelId                      = kInvalidObjectId;
            engineDesc->tire                         = tire;
            engineDesc->tireId                       = kInvalidObjectId;
            engineDesc->suspension                   = suspension;
            engineDesc->suspensionId                 = kInvalidObjectId;
            // SuspensionCompliance: applied alongside on the same prim;
            // look up by attachment path.
            {
                auto it = mVehicleComponentTracker.mSuspensionComplianceByPath.find(path);
                engineDesc->suspensionCompliance =
                    (it != mVehicleComponentTracker.mSuspensionComplianceByPath.end()) ? it->second : nullptr;
            }
            engineDesc->suspensionTravelDirection     = scanWa->suspensionTravelDirection;
            engineDesc->suspensionForceAppPointOffset = scanWa->suspensionForceAppPointOffset;
            engineDesc->wheelCenterOfMassOffset       = scanWa->wheelCenterOfMassOffset;
            engineDesc->tireForceAppPointOffset       = scanWa->tireForceAppPointOffset;
            engineDesc->suspensionFramePosition       = scanWa->suspensionFramePosition;
            engineDesc->suspensionFrameOrientation    = scanWa->suspensionFrameOrientation;
            engineDesc->wheelFramePosition            = scanWa->wheelFramePosition;
            engineDesc->wheelFrameOrientation         = scanWa->wheelFrameOrientation;
            engineDesc->index                         = scanWa->index;
            engineDesc->driven                        = scanWa->driven;
            engineDesc->collisionGroupId              = kInvalidObjectId;
            engineDesc->collisionGroupPath           = scanned.pathFor(scanWa->collisionGroupKey);
            engineDesc->shapePath                    = scanned.pathFor(scanWa->shapeKey);
            engineDesc->shapeId                       = kInvalidObjectId;

            mVehicleComponentTracker.mWheelAttachmentsOwned.push_back(engineDesc);
            mVehicleComponentTracker.mWheelAttachmentByPath.insert({ path, engineDesc });
        }

        // Group EVERY scanned wheel attachment (valid or malformed) by its
        // owning vehicle, so parseVehicle can enumerate a vehicle's attachments
        // without a USD descendant walk — and still reject the vehicle when one
        // is malformed (present here but absent from mWheelAttachmentByPath).
        for (const auto& owner : scanned.vehicleWheelAttachmentOwners)
        {
            if (!owner.first.valid())
                continue;  // attachment with no vehicle ancestor (malformed) — skip
            mVehicleComponentTracker.mVehicleWheelAttachments[scanned.pathFor(owner.first)].push_back(
                scanned.pathFor(owner.second));
        }

        for (size_t i = 0; i < scanned.vehicleMultiWheelDifferentials.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleMultiWheelDifferential");
            const SdfPath path = scanned.pathFor(scanned.vehicleMultiWheelDifferentialPaths[i]);
            if (mVehicleComponentTracker.mDifferentialsByPath.find(path) != mVehicleComponentTracker.mDifferentialsByPath.end())
                continue;
            const auto& scanDiff = scanned.vehicleMultiWheelDifferentials[i];
            MultiWheelDifferentialDesc* engineDesc = ICE_PLACEMENT_NEW(MultiWheelDifferentialDesc)();
            engineDesc->wheels                  = scanDiff->wheels;
            engineDesc->torqueRatios            = scanDiff->torqueRatios;
            engineDesc->averageWheelSpeedRatios = scanDiff->averageWheelSpeedRatios;
            mVehicleComponentTracker.mMultiWheelDifferentials.push_back(engineDesc);
            mVehicleComponentTracker.mDifferentialsByPath.insert({ path, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleTankDifferentials.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleTankDifferential");
            const SdfPath path = scanned.pathFor(scanned.vehicleTankDifferentialPaths[i]);
            if (mVehicleComponentTracker.mDifferentialsByPath.find(path) != mVehicleComponentTracker.mDifferentialsByPath.end())
                continue;
            const auto& scanDiff = scanned.vehicleTankDifferentials[i];
            TankDifferentialDesc* engineDesc = ICE_PLACEMENT_NEW(TankDifferentialDesc)();
            engineDesc->wheels                  = scanDiff->wheels;
            engineDesc->torqueRatios            = scanDiff->torqueRatios;
            engineDesc->averageWheelSpeedRatios = scanDiff->averageWheelSpeedRatios;
            engineDesc->numberOfWheelsPerTrack  = scanDiff->numberOfWheelsPerTrack;
            engineDesc->thrustIndexPerTrack     = scanDiff->thrustIndexPerTrack;
            engineDesc->wheelIndicesInTrackOrder = scanDiff->wheelIndicesInTrackOrder;
            engineDesc->trackToWheelIndices     = scanDiff->trackToWheelIndices;
            mVehicleComponentTracker.mTankDifferentials.push_back(engineDesc);
            mVehicleComponentTracker.mDifferentialsByPath.insert({ path, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleAutoGearBoxes.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleAutoGearBox");
            const SdfPath path = scanned.pathFor(scanned.vehicleAutoGearBoxPaths[i]);
            if (mVehicleComponentTracker.mAutoGearBoxes.find(path) != mVehicleComponentTracker.mAutoGearBoxes.end())
                continue;
            const auto& scanAg = scanned.vehicleAutoGearBoxes[i];
            AutoGearBoxDesc* engineDesc = ICE_PLACEMENT_NEW(AutoGearBoxDesc)();
            engineDesc->upRatios   = scanAg->upRatios;
            engineDesc->downRatios = scanAg->downRatios;
            engineDesc->latency    = scanAg->latency;
            mVehicleComponentTracker.mAutoGearBoxes.insert({ path, engineDesc });
        }

        // ----- Vehicle brakes + steering (7A.5) ----------------------
        // Brakes are multi-apply (one BrakesDesc per (path, brakesIndex)).
        // Steering is single-apply (basic + Ackermann variants; consumer
        // adapter picks the variant from the walker emit).  Both leave
        // nonlinearCmdResponse=nullptr — multi-apply NonlinearCmdResponse
        // migration ships in 7A.6.
        for (size_t i = 0; i < scanned.vehicleBrakes.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleBrakes");
            const SdfPath path = scanned.pathFor(scanned.vehicleBrakesPaths[i]);
            const auto& scanBrake = scanned.vehicleBrakes[i];
            const std::pair<SdfPath, uint8_t> mapKey{ path, scanBrake->brakesIndex };
            if (mVehicleComponentTracker.mBrakesByPathIndex.find(mapKey) !=
                mVehicleComponentTracker.mBrakesByPathIndex.end())
                continue;
            BrakesDesc* engineDesc = ICE_PLACEMENT_NEW(BrakesDesc)();
            // 7A.6: brakes instance token (e.g. "brakes0"/"brakes1") doubles
            // as the NonlinearCmdResponse instance lookup key.
            const TfToken brakesInst = scanned.tfTokenFor(scanned.vehicleBrakesInstanceTokens[i]);
            engineDesc->nonlinearCmdResponse = findNcr(path, brakesInst);
            engineDesc->wheels               = scanBrake->wheels;
            engineDesc->torqueMultipliers    = scanBrake->torqueMultipliers;
            engineDesc->maxBrakeTorque       = scanBrake->maxBrakeTorque;
            engineDesc->brakesIndex          = scanBrake->brakesIndex;
            mVehicleComponentTracker.mBrakes.push_back(engineDesc);
            mVehicleComponentTracker.mBrakesByPathIndex.insert({ mapKey, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleSteeringBasic.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleSteeringBasic");
            const SdfPath path = scanned.pathFor(scanned.vehicleSteeringBasicPaths[i]);
            if (mVehicleComponentTracker.mSteeringByPath.find(path) != mVehicleComponentTracker.mSteeringByPath.end())
                continue;
            const auto& scanSt = scanned.vehicleSteeringBasic[i];
            SteeringBasicDesc* engineDesc = ICE_PLACEMENT_NEW(SteeringBasicDesc)();
            engineDesc->nonlinearCmdResponse = findNcr(path, PhysxSchemaTokens->steer);  // 7A.6
            engineDesc->wheels               = scanSt->wheels;
            engineDesc->angleMultipliers     = scanSt->angleMultipliers;
            engineDesc->maxSteerAngle        = scanSt->maxSteerAngle;
            mVehicleComponentTracker.mSteeringBasic.push_back(engineDesc);
            mVehicleComponentTracker.mSteeringByPath.insert({ path, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleSteeringAckermann.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleSteeringAckermann");
            const SdfPath path = scanned.pathFor(scanned.vehicleSteeringAckermannPaths[i]);
            if (mVehicleComponentTracker.mSteeringByPath.find(path) != mVehicleComponentTracker.mSteeringByPath.end())
                continue;
            const auto& scanSt = scanned.vehicleSteeringAckermann[i];
            SteeringAckermannDesc* engineDesc = ICE_PLACEMENT_NEW(SteeringAckermannDesc)();
            engineDesc->nonlinearCmdResponse = findNcr(path, PhysxSchemaTokens->steer);  // 7A.6
            engineDesc->wheel0               = scanSt->wheel0;
            engineDesc->wheel1               = scanSt->wheel1;
            engineDesc->maxSteerAngle        = scanSt->maxSteerAngle;
            engineDesc->wheelBase            = scanSt->wheelBase;
            engineDesc->trackWidth           = scanSt->trackWidth;
            engineDesc->strength             = scanSt->strength;
            mVehicleComponentTracker.mSteeringAckermann.push_back(engineDesc);
            mVehicleComponentTracker.mSteeringByPath.insert({ path, engineDesc });
        }

        // ----- Vehicle chassis root pre-population (7A.13) ----------
        // Walker emitted parse-lib VehicleDescs into scanned.vehicles
        // with scalar chassis fields filled.  Mint engine-side
        // VehicleDescs, copy scalars, and resolve the four cross-refs
        // (drive / differential / steering / brakes) via the tracker
        // maps populated above:
        //   - drive: rel-or-API on the vehicle prim; either authored
        //     rel target or vehicle prim itself when DriveBasicAPI /
        //     DriveStandardAPI is applied locally.  Probe
        //     mDrivesStandard then mDrivesBasic.
        //   - differential / steering / brakes: applied directly on the
        //     vehicle prim, so the tracker maps are keyed by the
        //     vehicle path.
        //
        // Required-refs guard: if any cross-ref's API is applied but
        // the tracker lookup misses (the per-component parser rejected
        // the data), skip pre-pop entirely so legacy parseVehicle runs
        // and reports the error identically to pre-7A.13 behavior.
        for (size_t i = 0; i < scanned.vehicles.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicle");
            const SdfPath vehiclePath = scanned.pathFor(scanned.vehiclePaths[i]);
            if (mVehicleComponentTracker.mVehicleByPath.find(vehiclePath) !=
                mVehicleComponentTracker.mVehicleByPath.end())
                continue;

            // Component-applied checks route through the source (single-apply via
            // hasSchema; the multi-apply Brakes via hasSchema [any instance] +
            // forEachMultiApplyInstance [specific instance]). A null source skips
            // pre-pop so legacy parseVehicle runs — the same safe fallback the
            // tracker-miss `continue`s rely on. No UsdPrim.
            const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();
            const omni::physics::parse::ObjectKey vehKey =
                src ? mAttachedStage.keyFor(vehiclePath) : omni::physics::parse::ObjectKey{};
            if (!src || !src->exists(vehKey) || !src->hasSchema(vehKey, schemaTypeToken<PhysxSchemaPhysxVehicleAPI>(*src)))
                continue;  // legacy parser will report any error path

            // ---- Drive: rel-or-API on vehicle prim --------------------
            // Mirror legacy parseDrive semantics: prefer the authored
            // rel target; on missing or empty rel, fall back to the API
            // applied on the vehicle prim itself.  Error paths
            // (rel with >1 targets, both DriveBasic+DriveStandard on
            // the same prim) are logged here and the vehicle is dropped.
            SdfPath drivePath;
            bool driveAuthored = false;
            {
                // physxVehicle:drive relationship targets via the source (an
                // unauthored/absent rel reports empty — matches the legacy
                // `if (GetDriveRel().HasAuthoredTargets()) GetTargets(...)`).
                SdfPathVector paths;
                {
                    std::vector<omni::physics::parse::ObjectKey> driveKeys;
                    src->getRelationshipTargets(
                        vehKey, src->internToken(PhysxSchemaTokens->physxVehicleDrive.GetString()), driveKeys);
                    paths.reserve(driveKeys.size());
                    for (const omni::physics::parse::ObjectKey& k : driveKeys)
                        paths.push_back(mAttachedStage.pathFor(k));
                }
                if (paths.size() > 1)
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: \"%s\" must not have more than 1 \"drive\" relationship defined.",
                        vehiclePath.GetName().c_str());
                    continue;
                }
                if (paths.size() == 1)
                {
                    drivePath = paths[0];
                    driveAuthored = true;
                }
                else
                {
                    const bool standardOnSelf = src->hasSchema(vehKey, schemaTypeToken<PhysxSchemaPhysxVehicleDriveStandardAPI>(*src));
                    const bool basicOnSelf    = src->hasSchema(vehKey, schemaTypeToken<PhysxSchemaPhysxVehicleDriveBasicAPI>(*src));
                    if (standardOnSelf && basicOnSelf)
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: vehicle \"%s\" has both PhysxSchemaPhysxVehicleDriveStandardAPI and "
                            "PhysxSchemaPhysxVehicleDriveBasicAPI applied. Only one is allowed.",
                            vehiclePath.GetName().c_str());
                        continue;
                    }
                    if (standardOnSelf || basicOnSelf)
                    {
                        drivePath = vehiclePath;
                        driveAuthored = true;
                    }
                }
            }
            DriveDesc* drive = nullptr;
            if (driveAuthored)
            {
                auto stdIt = mVehicleComponentTracker.mDrivesStandard.find(drivePath);
                if (stdIt != mVehicleComponentTracker.mDrivesStandard.end())
                {
                    drive = stdIt->second;
                }
                else
                {
                    auto basicIt = mVehicleComponentTracker.mDrivesBasic.find(drivePath);
                    if (basicIt != mVehicleComponentTracker.mDrivesBasic.end())
                        drive = basicIt->second;
                }
                if (!drive)
                    continue;  // per-component parser already logged the validation error
            }

            // ---- Differential: applied directly on vehicle prim -------
            const bool diffApiApplied =
                src->hasSchema(vehKey, schemaTypeToken<PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI>(*src)) ||
                src->hasSchema(vehKey, schemaTypeToken<PhysxSchemaPhysxVehicleTankDifferentialAPI>(*src));
            MultiWheelDifferentialDesc* differential = nullptr;
            if (diffApiApplied)
            {
                auto it = mVehicleComponentTracker.mDifferentialsByPath.find(vehiclePath);
                if (it == mVehicleComponentTracker.mDifferentialsByPath.end())
                    continue;
                differential = it->second;
            }

            // ---- Steering: applied directly on vehicle prim -----------
            const bool steeringApiApplied =
                src->hasSchema(vehKey, schemaTypeToken<PhysxSchemaPhysxVehicleSteeringAPI>(*src)) ||
                src->hasSchema(vehKey, schemaTypeToken<PhysxSchemaPhysxVehicleAckermannSteeringAPI>(*src));
            SteeringDesc* steering = nullptr;
            if (steeringApiApplied)
            {
                auto it = mVehicleComponentTracker.mSteeringByPath.find(vehiclePath);
                if (it == mVehicleComponentTracker.mSteeringByPath.end())
                    continue;
                steering = it->second;
            }

            // ---- Brakes: multi-apply (brakes0 / brakes1) --------------
            std::vector<const BrakesDesc*> brakes;
            const bool brakesApiApplied = src->hasSchema(vehKey, schemaTypeToken<PhysxSchemaPhysxVehicleBrakesAPI>(*src));
            if (brakesApiApplied)
            {
                bool brakesPrePopOk = true;
                TfToken brakesTokens[] = { PhysxSchemaTokens->brakes0, PhysxSchemaTokens->brakes1 };
                for (uint32_t bi = 0; bi < 2; ++bi)
                {
                    auto it = mVehicleComponentTracker.mBrakesByPathIndex.find(
                        { vehiclePath, static_cast<uint8_t>(bi) });
                    if (it != mVehicleComponentTracker.mBrakesByPathIndex.end())
                    {
                        brakes.push_back(it->second);
                        continue;
                    }
                    // Tracker miss: only counts as a failure when the
                    // instance API was actually applied (matches legacy
                    // parseBrakes). Per-instance check via the source's
                    // multi-apply instance enumeration.
                    static const std::string brakesBase =
                        UsdSchemaRegistry::GetSchemaTypeName(TfType::Find<PhysxSchemaPhysxVehicleBrakesAPI>()).GetString();
                    const std::string appliedSchema = brakesBase + ":" + brakesTokens[bi].GetString();
                    const bool instanceApplied = src->hasSchema(vehKey, src->internToken(appliedSchema));
                    if (instanceApplied)
                    {
                        brakesPrePopOk = false;
                        break;
                    }
                }
                if (!brakesPrePopOk)
                    continue;  // per-component parser already logged the validation error
                if (brakes.empty())
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: \"%s\": PhysxVehicleBrakesAPI is applied but no valid instance token could be found.",
                        vehiclePath.GetName().c_str());
                    continue;
                }
            }

            // ---- Mint engine-side VehicleDesc + copy fields -----------
            const auto& scanVeh = scanned.vehicles[i];
            VehicleDesc* engineDesc = ICE_PLACEMENT_NEW(VehicleDesc)();
            engineDesc->bodyId                              = kInvalidObjectId;
            engineDesc->drive                               = drive;
            engineDesc->differential                        = differential;
            engineDesc->steering                            = steering;
            engineDesc->brakes                              = std::move(brakes);
            engineDesc->scale                               = scanVeh->scale;
            engineDesc->subStepThresholdLongitudinalSpeed   = scanVeh->subStepThresholdLongitudinalSpeed;
            engineDesc->lowForwardSpeedSubStepCount         = scanVeh->lowForwardSpeedSubStepCount;
            engineDesc->highForwardSpeedSubStepCount        = scanVeh->highForwardSpeedSubStepCount;
            engineDesc->minLongitudinalSlipDenominator      = scanVeh->minLongitudinalSlipDenominator;
            engineDesc->minPassiveLongitudinalSlipDenominator = scanVeh->minPassiveLongitudinalSlipDenominator;
            engineDesc->minActiveLongitudinalSlipDenominator  = scanVeh->minActiveLongitudinalSlipDenominator;
            engineDesc->minLateralSlipDenominator           = scanVeh->minLateralSlipDenominator;
            engineDesc->longitudinalStickyTireThresholdSpeed = scanVeh->longitudinalStickyTireThresholdSpeed;
            engineDesc->longitudinalStickyTireThresholdTime  = scanVeh->longitudinalStickyTireThresholdTime;
            engineDesc->longitudinalStickyTireDamping        = scanVeh->longitudinalStickyTireDamping;
            engineDesc->lateralStickyTireThresholdSpeed     = scanVeh->lateralStickyTireThresholdSpeed;
            engineDesc->lateralStickyTireThresholdTime      = scanVeh->lateralStickyTireThresholdTime;
            engineDesc->lateralStickyTireDamping            = scanVeh->lateralStickyTireDamping;
            engineDesc->enabled                             = scanVeh->enabled;
            engineDesc->queryType                           = scanVeh->queryType;
            engineDesc->hasUserDefinedSprungMassValues      = false;  // computed during wheel-attachment iteration
            engineDesc->hasUserDefinedMaxDroopValues        = false;
            engineDesc->hasUserDefinedRestLoadValues        = false;
            engineDesc->isUsingDeprecatedLatStiffY          = false;
            engineDesc->referenceFrameIsCenterOfMass        = scanVeh->referenceFrameIsCenterOfMass;
            engineDesc->limitSuspensionExpansionVelocity    = scanVeh->limitSuspensionExpansionVelocity;

            mVehicleComponentTracker.mVehiclesOwned.push_back(engineDesc);
            mVehicleComponentTracker.mVehicleByPath.insert({ vehiclePath, engineDesc });
        }

        // ----- Apply collected time-sampled callbacks --------------
        callbacks::applyTimeSampledCallbacks(mAttachedStage, cbList);
    }

    // Source-driven load entry. `scanRoots` are the subtree roots to scan (the
    // pseudo-root for a whole-stage load; the changed prims for an incremental
    // update). `excludePaths` (replicator selective load) skips those subtrees.
    // No UsdPrim / PrimIterator: the backend scanStage builds the USD ranges and
    // the post-scan passes walk via IPhysicsSource.
    void loadFromRange(const std::vector<SdfPath>& scanRoots, const PathSet* excludePaths, bool initialStageLoad)
    {
        mStage = mAttachedStage.getStage();
        mSceneFound = false;
        mNoValidScene = false;
        mParsingFlags = 0;
        mFilteredPairsPaths.clear();

        static const std::unordered_set<SdfPath, SdfPath::Hash> kNoExclude;

        omni::physics::usd::ScannedStage scanned;
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:scanStage");
            // Single switch point (ADR-0002 M2c): scanStage(AttachTarget, ...)
            // routes to the registered scan backend (e.g. ovstage) or the native
            // USD walk. No backend branching here. (Subtree / incremental scans
            // elsewhere stay on the USD-stage overloads.)
            scanned = omni::physics::usd::scanStage(mAttachedStage.attachTarget(), scanRoots,
                excludePaths ? *excludePaths : kNoExclude, omni::physx::usdparser::iceDescriptorAllocator());
        }
        if (!scanned.particleSystems.empty() || !scanned.particleSets.empty() ||
            !scanned.particleSamplers.empty() || !scanned.particleAnisotropies.empty() ||
            !scanned.particleSmoothings.empty() || !scanned.particleIsosurfaces.empty())
        {
            mParsingFlags |= ParsingFlag::eParseParticles;
        }

        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:gatherPerPrimSideEffects");
            const omni::physics::parse::AttachTarget attachTarget = mAttachedStage.attachTarget();
            if (attachTarget.nativeStage && attachTarget.stageId == 0)
            {
                gatherScannedSideEffects(scanned);
            }
            else
                gatherPerPrimSideEffects(scanRoots, excludePaths);
        }

        struct ScopedOvstageKnownKeys
        {
            omni::physics::ovstage::OvstageSource* source = nullptr;
            ~ScopedOvstageKnownKeys()
            {
                if (source)
                {
                    source->clearKnownKeys();
                    source->clearBucket();
                }
            }
        } scopedKnownKeys;
        if (initialStageLoad)
            scopedKnownKeys.source = seedOvstageKnownKeysForInitialLoad(mAttachedStage, scanRoots);

        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs");
            processScannedDescs(scanned, scanRoots, excludePaths);
        }

        if (mNoValidScene)
            return;

        // We dont have a PhysX scene, PhysX should not simulate anything early exit
        if (mNoPhysXScene)
            return;

        if(OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
        {
            // scristiano: do not parse particles, vehicles or internal objects when forcing single scene for inspector (for now)
            mParsingFlags = 0;
        }

        // require always a scene
        if (initialStageLoad && !mSceneFound)
        {
            auto createDefaultSceneObject = [&](const SdfPath& scenePath)
            {
                PhysxSceneDesc sceneDesc;
                const omni::physics::parse::SourceUnits sceneUnits = mAttachedStage.getSourceUnits();
                omni::physics::parse::setToDefault(sceneDesc, sceneUnits);
                setToDefault(sceneDesc.defaultMaterialDesc);
                setToDefault(sceneUnits, sceneDesc.defaultDeformableMaterialDesc);
                setToDefault(sceneUnits, sceneDesc.defaultSurfaceDeformableMaterialDesc);
                setToDefault(sceneDesc.defaultPBDMaterialDesc);
                const ObjectId sceneId = createObject(mAttachedStage, scenePath, &sceneDesc, false); // deleteDesc == false
                if (sceneId != kInvalidObjectId)
                {
                    mSceneFound = true;
                    mNoPhysXScene = false;
                    mNumScenes++;
                }
            };

            const SdfPath tempPhysicsScenePath = OmniPhysX::getInstance().getTempPhysicsScenePath();
            const bool updateToUsd = OmniPhysX::getInstance().getCachedSettings().updateToUsd;
            if (updateToUsd)
            {
                PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eInfo,
                    "Physics USD: Physics scene not found. A temporary default PhysicsScene prim was added automatically!");

                const SdfPath authoredPath =
                    omni::physics::usd::createDefaultPhysicsScene(mStage, tempPhysicsScenePath);
                createDefaultSceneObject(authoredPath.IsEmpty() ? tempPhysicsScenePath : authoredPath);
                OmniPhysX::getInstance().setHasTempPhysicsScene(true);
            }
            else
            {
                createDefaultSceneObject(tempPhysicsScenePath);
            }
        }

        // create deformable materials first
        {
            for (const std::pair<SdfPath, PBDMaterialDesc*>& pair : mPDBMatrialsDescs)
            {
                createObject(mAttachedStage, pair.first, pair.second);
            }
            mPDBMatrialsDescs.clear();
        }

        // create materials first
        {
            for (size_t i = 0; i < mMaterials.size(); i++)
            {
                createObject(mAttachedStage, mMaterials[i].first, mMaterials[i].second);
            }
        }
        mMaterials.clear();

        // create shapes next
        {
            for (const ShapeDescAndMaterials& shapeDesc : mShapes)
            {
                finalizeShape(mAttachedStage, shapeDesc.desc, shapeDesc.materials);
                PhysxRigidBodyDesc* bodyDesc = createShape(mAttachedStage, shapeDesc.path, shapeDesc.desc, nullptr);
                // standalone collision -> static body
                if (bodyDesc)
                {
                    if (mBodyMap.find(shapeDesc.path) == mBodyMap.end())
                    {
                        BodyDescAndColliders& bdCol = mBodyMap[shapeDesc.path];
                        bdCol.desc = bodyDesc;
                    }
                    else
                    {
                        BodyDescAndColliders bdCol;
                        bdCol.desc = bodyDesc;
                        mAdditionalBodyVector.push_back(std::make_pair(shapeDesc.path, bdCol));
                    }
                }
            }
        }
        mShapes.clear();

        // finalize bodies
        {
            for (BodyMap::reference ref : mBodyMap)
            {
                finalizeRigidBody(mAttachedStage, ref.second);
            }
            for (std::pair<SdfPath, BodyDescAndColliders>& ref : mAdditionalBodyVector)
            {
                finalizeRigidBody(mAttachedStage, ref.second);
            }
        }

        // create articulation links
        {
            CARB_PROFILE_ZONE(0,"OmniPhysX:createArticulationLinks");
            SdfChangeBlock changeBlock; // add change block to get transforms sanitation changes grouped
            createArticulationLinks(mAttachedStage, mBodyMap, mJointVector, mArticulationMap, mJointPathIndexMap);
        }

        // create tendons (depends on prior articulation creation)
        {
            createFixedTendons(mAttachedStage, mTendonAxisMap, mFixedTendons);
            createSpatialTendons(mAttachedStage, mTendonAttachmentMap, mSpatialTendons);
        }

        // create rigid bodies
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createBodies");
            SdfChangeBlock changeBlock; // add change block to get transforms sanitation changes grouped
            createBodies(mAttachedStage, mBodyMap, mAdditionalBodyVector);
        }

        // deformable materials
        {
            for (size_t i = 0; i < mDeformableMaterials.size(); i++)
            {
                createObject(mAttachedStage, mDeformableMaterials[i].first, mDeformableMaterials[i].second);
            }
        }
        mDeformableMaterials.clear();

        // finalize and create deformable bodies
        {
            for (const DeformableDescAndMaterials& deformableDesc : mDeformableBodies)
            {
                // Re-resolve collisionGroup now that processScannedDescs has
                // populated CollisionGroupsMap.  Collision groups are
                // emitted into the map AFTER deformable descs, so the
                // group lookup during scanStage returns kInvalidObjectId;
                // this late re-resolution still lands before
                // createDeformableBody wires the filter group up in PhysX.
                if (!deformableDesc.desc->collisionMeshPath.IsEmpty())
                    deformableDesc.desc->collisionGroup = getCollisionGroup(mAttachedStage, deformableDesc.desc->collisionMeshPath);
                finalizeDeformableBody(mAttachedStage, deformableDesc.desc, deformableDesc.simMeshMaterial);
                createDeformableBody(mAttachedStage, deformableDesc.desc, deformableDesc.path);
            }
        }

        // create forces
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createForces");
            for (const std::pair<SdfPath, PhysxForceDesc*>& pair : mPhysxForceDescs)
            {
                finalizePhysxForce(mAttachedStage, mAttachedStage.keyFor(pair.first), *pair.second);
                createObject(mAttachedStage, pair.first, pair.second);
            }
            mPhysxForceDescs.clear();
        }

        // create joints
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createJoints");
            createJoints(mAttachedStage, mJointVector, initialStageLoad);
        }

        // create mimic joints (has to be done after the joints are created)
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createMimicJoints");
            createMimicJoints(mAttachedStage, mMimicJoints);
        }

        // parse all instancers and particle objects
        // A.B. TODO merge some traversals
        std::vector<ParticleSystemDesc*> particleSysDescs;
        std::vector<ParticleDesc*> particleDescs;

        if (mParsingFlags & ParsingFlag::eParseParticles)
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:particles");
            std::vector<SdfPath> jointInstancerPaths;

            // Source-backed type/schema dispatch keyed by ObjectKey (no UsdPrim).
            const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();

            // The particle systems / sets / samplers were already scanned into
            // typed parse descriptors by scanStage.  Index them by prim path
            // (parse-time table) so the dispatch below can build the engine
            // descriptor from scanned data instead of re-reading USD.
            std::unordered_map<SdfPath, const omni::physics::parse::ParticleSystemDesc*, SdfPath::Hash> scannedSystems;
            for (const auto& sysUPtr : scanned.particleSystems)
                scannedSystems[scanned.pathFor(sysUPtr->systemKey)] = sysUPtr.get();
            std::unordered_map<SdfPath, const omni::physics::parse::ParticleSetDesc*, SdfPath::Hash> scannedSets;
            for (const auto& setUPtr : scanned.particleSets)
                scannedSets[scanned.pathFor(setUPtr->primKey)] = setUPtr.get();
            std::unordered_map<SdfPath, const omni::physics::parse::ParticleSamplingDesc*, SdfPath::Hash> scannedSamplers;
            for (size_t i = 0; i < scanned.particleSamplers.size(); ++i)
                scannedSamplers[scanned.pathFor(scanned.particleSamplerKeys[i])] = scanned.particleSamplers[i].get();
            std::unordered_set<SdfPath, SdfPath::Hash> queuedParticleSystems;
            std::unordered_set<SdfPath, SdfPath::Hash> queuedParticleSets;
            // Voxel-map subtrees are pruned by forEachLoadObject below (it stops descending under
            // an InfiniteVoxelMapAPI Xform). scanStage has no voxel-map awareness, so the scanned
            // particle lists still contain prims nested under a voxel map; the catch-all loops
            // after the walk must replicate the prune and skip them.
            std::vector<SdfPath> voxelMapRoots;

            forEachLoadObject(scanRoots, excludePaths, [&](omni::physics::parse::ObjectKey primObjKey) -> bool
            {
                const SdfPath primPath = mAttachedStage.pathFor(primObjKey);

                if (src->isA(primObjKey, schemaTypeToken<PhysxSchemaPhysxParticleSystem>(*src)))
                {
                    auto sysIt = scannedSystems.find(primPath);
                    if (sysIt != scannedSystems.end())
                    {
                        particleSysDescs.push_back(buildParticleSystemDesc(mAttachedStage, scanned, *sysIt->second));
                        queuedParticleSystems.insert(primPath);
                    }
                }
                else if (src->isA(primObjKey, schemaTypeToken<UsdGeomXform>(*src)))
                {
                    if (src->hasSchema(primObjKey, src->internToken(gInfiniteVoxelMapAPI.GetString())))
                    {
                        InfiniteVoxelMapDesc desc(primPath);
                        createObject(mAttachedStage, primPath, &desc, false);
                        voxelMapRoots.push_back(primPath);
                        return true; // avoid parsing point instancers below (prune subtree)
                    }
                    return false;
                }
                else if (src->isA(primObjKey, schemaTypeToken<UsdGeomMesh>(*src)) &&
                         src->hasSchema(primObjKey, schemaTypeToken<PhysxSchemaPhysxParticleSamplingAPI>(*src)))
                {
                    cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
                    // Build the engine sampling descriptor from the scanned
                    // parse descriptor (no USD re-read).
                    auto samplerIt = scannedSamplers.find(primPath);
                    const omni::physics::parse::ParticleSamplingDesc* scanSampling =
                        samplerIt != scannedSamplers.end() ? samplerIt->second : nullptr;
                    ParticleSamplingDesc samplingDesc;
                    SdfPath particleSetPath;
                    if (scanSampling)
                    {
                        samplingDesc.samplingDistance = scanSampling->samplingDistance;
                        samplingDesc.sampleVolume = scanSampling->sampleVolume;
                        samplingDesc.maxSamples = scanSampling->maxSamples;
                        samplingDesc.pointWidth = scanSampling->pointWidth;
                        particleSetPath = scanSampling->particleSetKey.valid() ?
                            scanned.pathFor(scanSampling->particleSetKey) : SdfPath();
                        samplingDesc.particleSetPath = particleSetPath;
                    }
                    // check if it points to a valid particle prim
                    const omni::physics::parse::ObjectKey particleKey = mAttachedStage.keyFor(particleSetPath);
                    if (!scanSampling || !src->exists(particleKey) || !src->hasSchema(particleKey, schemaTypeToken<PhysxSchemaPhysxParticleSetAPI>(*src)) ||
                        !(src->isA(particleKey, schemaTypeToken<UsdGeomPoints>(*src)) || src->isA(particleKey, schemaTypeToken<UsdGeomPointInstancer>(*src))))
                    {
                        CARB_LOG_WARN("%s: particle sampler does not point to a valid particle prim, needs to be set before stage parsing starts.", primPath.GetText());
                    }
                    else if (!cookingDataAsync)
                    {
                        CARB_LOG_WARN("%s: particle sampling failed.", primPath.GetText());
                    }
                    else
                    {
                        // AD: OM-89182 - we need to disallow adding new samplers during simulation: it could have been just a path change, and we don't have the setup to clean things up.
                        // we never allowed processing changes in particleAuthoring and this code is just here in case there is no PhysXUI or we need to block at the sim start for async
                        // tasks to finish.
                        if (OmniPhysX::getInstance().getSimulationStepCount() == 0)
                        {
                            omni::physx::particles::createParticleSampler(primPath, samplingDesc.particleSetPath);
                            cookingDataAsync->poissonSampleMesh(primObjKey, mAttachedStage, samplingDesc, false, false);
                        }
                        else
                        {
                            CARB_LOG_WARN("%s: it is not allowed to add new particle samplers once simulation has been started, ignoring.", primPath.GetText());
                        }
                    }
                }
                else if (src->isA(primObjKey, schemaTypeToken<UsdGeomPointBased>(*src)) &&
                         src->hasSchema(primObjKey, schemaTypeToken<PhysxSchemaPhysxParticleSetAPI>(*src)))
                {
                    auto setIt = scannedSets.find(primPath);
                    if (setIt != scannedSets.end())
                    {
                        if (ParticleSetDesc* desc = buildParticleSetDesc(mAttachedStage, scanned, *setIt->second))
                        {
                            particleDescs.push_back(desc);
                            queuedParticleSets.insert(primPath);
                        }
                    }
                }
                else if (src->isA(primObjKey, schemaTypeToken<UsdGeomPointInstancer>(*src)))
                {
                    if (src->hasSchema(primObjKey, schemaTypeToken<PhysxSchemaPhysxParticleSetAPI>(*src)))
                    {
                        auto setIt = scannedSets.find(primPath);
                        if (setIt != scannedSets.end())
                        {
                            if (ParticleSetDesc* desc = buildParticleSetDesc(mAttachedStage, scanned, *setIt->second))
                            {
                                particleDescs.push_back(desc);
                                queuedParticleSets.insert(primPath);
                            }
                        }
                    }
                    else if (!src->isInstance(primObjKey) && !src->isInstanceProxy(primObjKey))
                    {
                        parseRigidBodyInstancer(mAttachedStage, primPath, mFilteredPairs);
                    }
                }
                else if (src->isA(primObjKey, schemaTypeToken<PhysxSchemaPhysxPhysicsJointInstancer>(*src)))
                {
                    if (!src->isInstance(primObjKey) && !src->isInstanceProxy(primObjKey))
                    {
                        jointInstancerPaths.push_back(primPath);
                    }
                }
                return false;
            });

            // Skip scanned particle prims nested under a pruned voxel-map subtree so the catch-all
            // does not create particle systems/sets that forEachLoadObject deliberately excluded.
            auto underVoxelMap = [&](const SdfPath& path) -> bool
            {
                for (const SdfPath& root : voxelMapRoots)
                    if (path.HasPrefix(root))
                        return true;
                return false;
            };

            for (const auto& entry : scannedSystems)
            {
                if (queuedParticleSystems.find(entry.first) == queuedParticleSystems.end() &&
                    !underVoxelMap(entry.first))
                    particleSysDescs.push_back(buildParticleSystemDesc(mAttachedStage, scanned, *entry.second));
            }

            for (const auto& entry : scannedSets)
            {
                if (queuedParticleSets.find(entry.first) == queuedParticleSets.end() &&
                    !underVoxelMap(entry.first))
                {
                    if (ParticleSetDesc* desc = buildParticleSetDesc(mAttachedStage, scanned, *entry.second))
                        particleDescs.push_back(desc);
                }
            }

            for (const SdfPath& jointInstancerPath : jointInstancerPaths)
            {
                parseJointInstancer(mAttachedStage, jointInstancerPath);
            }

            createParticleSystemsAndObjects(mAttachedStage, particleSysDescs, particleDescs, mFilteredPairs);
        }

        // Re-cooking the collision mesh above may produce a different topology
        // (cross-platform non-determinism, OMPE-90043), invalidating the pre-baked attachment data.
        // AttachmentAuthoring cannot fix this in time because ScopedBlockUSDUpdates suppresses
        // USD notices during cooking.
        {
            refreshAutoDeformableAttachments(mAttachedStage, mDeformableAttachmentVector, mDeformableCollisionFilterVector);
        }

        // create deformable attachments and filters
        {
            createDeformableAttachments(mAttachedStage, mDeformableAttachmentVector);
        }
        {
            createDeformableCollisionFilters(mAttachedStage, mDeformableCollisionFilterVector);
        }

        // create filtered collision groups.
        {
            PHYSICS_PROFILE("SetupCollisionGroups");
            setupCollisionGroups(mAttachedStage, mCollisionGroupsPrims);
        }

        // create collision blocks
        {
            createFilteredPairs(mAttachedStage, mFilteredPairs, mFilteredPairsPaths);
        }

        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:updateMass");
            UsdLoad::getUsdLoad()->updateRigidBodyMass();
        }

        //
        // parse all vehicles
        //
        // note: done after the rigid body masses have been updated such that
        //       vehicle creation can work with the actual body mass.
        //
		if (mParsingFlags & ParsingFlag::eParseVehicles)
		{
	        // all the scenes are expected to be loaded now -> set the vehicle contexts
	        PhysXUsdPhysicsInterface& physicsInterface = *mAttachedStage.getPhysXPhysicsInterface();
	        for (VehicleContextDesc& vehicleContextDesc : mVehicleContextDescList)
	        {
	            physicsInterface.setVehicleContext(mAttachedStage, vehicleContextDesc);
	        }

	        // all the materials are expected to be loaded now -> create the tire friction tables (which reference materials).
	        // The vehicles need the tables to be created already.
	        for (TireFrictionTableDesc*& tireFrictionTableDesc : mTireFrictionTableDescList)
	        {
	            uint32_t materialPathCount = static_cast<uint32_t>(tireFrictionTableDesc->materialPaths.size());
	            tireFrictionTableDesc->materialIds.resize(materialPathCount);

	            for (size_t j = 0; j < materialPathCount; j++)
	            {
	                const SdfPath& materialKey = tireFrictionTableDesc->materialPaths[j];
	                ObjectId materialId = mAttachedStage.getObjectDatabase()->findEntry(materialKey, eMaterial);
	                tireFrictionTableDesc->materialIds[j] = materialId;
	            }

	            createObject(mAttachedStage, tireFrictionTableDesc->path, tireFrictionTableDesc, true);
	        }
	
	        VehicleComponentTracker& vehicleComponentTracker = mVehicleComponentTracker;
	        const omni::physics::parse::IPhysicsSource* vehSrc = mAttachedStage.getSource();
	        forEachLoadObject(scanRoots, excludePaths, [&](omni::physics::parse::ObjectKey primObjKey) -> bool
	        {
	            // note: PhysxVehicleContextAPI needs to be set before creating vehicles. However, checking here is not that easy since
                //       the necessary data is not readily available. Thus, the check is done as part of vehicle creation.
	            loadVehicle(mAttachedStage, primObjKey, vehicleComponentTracker);

	            // skip the instancer parsing: prune the subtree rooted at a point instancer.
	            return vehSrc && vehSrc->isA(primObjKey, schemaTypeToken<UsdGeomPointInstancer>(*vehSrc));
	        });
    	}

        {
            for (PathPhysXDescMap::reference ref : mPhysXDescCache)
            {
                PhysxObjectDesc* desc = (PhysxObjectDesc*)ref.second;
                ICE_FREE(desc);
            }
        }
        mPhysXDescCache.clear();
    }

private:
    bool mSceneFound;
    bool mNoValidScene;
    bool mNoPhysXScene;
    bool mCollectionsPopulated;
    uint32_t mNumScenes;

    uint32_t mParsingFlags;

    AttachedStage&  mAttachedStage;
    UsdStageWeakPtr mStage;
    PathPhysXDescMap  mPhysXDescCache;

    // maps holding the descs
    MaterialsVector mMaterials;
    DeformableMaterialsVector mDeformableMaterials;
    ShapeDescsVector mShapes;
    ArticulationMap mArticulationMap;
    BodyMap mBodyMap;
    BodyVector mAdditionalBodyVector;
    JointVector mJointVector;
    JointPathIndexMap mJointPathIndexMap;
    DeformableBodyDescsVector mDeformableBodies;

    SdfPathVector mFilteredPairsPaths;
    CollisionPairVector mFilteredPairs;

    // forces
    std::vector<std::pair<SdfPath, PhysxForceDesc*>> mPhysxForceDescs;

    // particle materials
    std::vector<std::pair<SdfPath, PBDMaterialDesc*>> mPDBMatrialsDescs;

    std::vector<SdfPath> mCollisionGroupsPrims;

    std::vector<VehicleContextDesc> mVehicleContextDescList;
    std::vector<TireFrictionTableDesc*> mTireFrictionTableDescList;
    // Shareable vehicle components pre-populated by processScannedDescs
    // (7A.2 onward). Owned for the duration of the load — components
    // are referenced by parseVehicle via this tracker.
    VehicleComponentTracker mVehicleComponentTracker;

    // tendon data structures
    SpatialTendonVector mSpatialTendons;
    TendonAttachmentMap mTendonAttachmentMap;
    FixedTendonVector mFixedTendons;
    TendonAxisMap mTendonAxisMap;

    MimicJointVector mMimicJoints;

    // deformable attachments and collision filters
    DeformableAttachmentVector mDeformableAttachmentVector;
    DeformableCollisionFilterVector mDeformableCollisionFilterVector;
};

void loadFromStage(AttachedStage& attachedStage, const PathSet* excludePaths)
{
    PHYSICS_PROFILE("Physics Load Stage");
    CARB_PROFILE_ZONE(0, "Physics Load Stage");
    // Whole-stage load: the scan root is the pseudo-root; the backend builds the
    // UsdTraverseInstanceProxies range internally and the post-scan passes walk
    // via IPhysicsSource. `excludePaths` (replicator selective load) prunes the
    // listed subtrees.
    const std::vector<SdfPath> scanRoots{ SdfPath::AbsoluteRootPath() };
    PhysxUsdPhysicsListener usdPhysicsListener(attachedStage);
    usdPhysicsListener.loadFromRange(scanRoots, excludePaths, true);
}

void loadPhysicsFromPrimitive(AttachedStage& attachedStage, const std::set<SdfPath>& updateRoots)
{
    PHYSICS_PROFILE("Physics Load Prims");
    CARB_PROFILE_ZONE(0, "Physics Load Prims");
    if (updateRoots.empty())
        return;

    // Incremental re-parse: the changed-prim paths are the scan roots; the
    // backend walks each subtree (instance proxies) and the post-scan passes
    // walk via IPhysicsSource. No PrimIterator.
    const std::vector<SdfPath> scanRoots(updateRoots.begin(), updateRoots.end());
    PhysxUsdPhysicsListener usdPhysicsListener(attachedStage);
    usdPhysicsListener.loadFromRange(scanRoots, nullptr, false);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
