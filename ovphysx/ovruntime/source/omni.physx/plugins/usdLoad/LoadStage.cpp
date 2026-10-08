// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-9 AC-16 AC-17 AC-18 AC-19 AC-20 AC-21 AC-22 AC-23 AC-24
 *
 * @implements REQ-PARSE-FEED-003
 * @covers AC-12
 *
 * @implements REQ-SIM-MULTISCENE-001
 * @covers AC-3
 *
 * @implements REQ-SIM-OBJECTDB-001
 * @covers AC-2
 *
 * @implements REQ-WRITE-LOCALXFORM-001
 * @covers AC-4
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-23 AC-27 AC-29 AC-30
 *
 * @implements REQ-SPLINE-TARGET-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-SIM-AUTOATTACH-001
 * @covers AC-2 AC-4
 *
 * @implements REQ-PARSE-FEED-002
 * @covers AC-5
 */

#include <carb/logging/Log.h>
#include <carb/Framework.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/Allocator.h>
#include <carb/profiler/Profile.h>
#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include "LoadTools.h"
#include "Joint.h"
#include "MimicJoint.h"
#include "DeformableAttachment.h"
#include "IceDescriptorAllocator.h"
#include <attachment/PhysXAttachment.h>
#include <usdBridge/AttachmentAuthoringBridge.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ScanBackend.h> // pxr-free scanStage(AttachTarget, ...) sibling
#include <omni/physics/parse/ScannedStage.h>
#include <unordered_map>
#include <unordered_set>
#include "Vehicle.h"
#include <usdInterface/UsdInterface.h>

// Forward-declared rather than pulling in PhysXScene.h, a heavy header, for one function.
namespace omni { namespace physx { bool isRigidBodyDynamic(omni::physx::usdparser::ObjectId id); } }

#include "LoadUsd.h"
#include "PhysXTools.h"
#include <common/foundation/CarbPhysXCast.h> // toPhysX/toPhysXQuat for the Joints loop's body-transform check
#include <common/foundation/MatrixTools.h> // getScale/makeMatrix/toTransform for isJointBodyTransformEqual
#include "Mass.h"
#include "Collision.h"
#include "Particles.h"
#include "Articulation.h"
#include "PointInstancer.h"
#include "JointInstancer.h"
#include "Material.h"
#include "Scene.h"
#include "PhysicsBody.h"
#include "CollisionGroup.h"
#include "LoadStage.h"
#include <CookingDataAsync.h>
#include <utils/Profile.h>
#include <PhysXScene.h>
#include <OmniPhysX.h>
#include <VoxelMap.h>
#include <particles/PhysXParticleSampling.h>


// physx specific stuff
#include "FixedTendon.h"
#include "SpatialTendon.h"
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

#include <OvstageSource.h>
#include "ScannedShapeCookingDispatch.h"



#include "TimeSampledCallbacks.h"
#include "DeformableBodyConverter.h"
#include "Material.h"  // setToDefault overloads for default sub-material descs
#include <propertiesUpdate/PhysXPropertiesUpdate.h>  // updateDeformableContactOffset/RestOffset
#include <PhysXCustomJoint.h>  // CustomJointManager for eJointCustom registry lookup
#include <cstdlib>
#include <unordered_set>

#include <omni/physics/parse/IPhysicsSource.h>  // source-backed isA/hasSchema dispatch
#include <omni/physics/parse/KnownTokens.h>

using namespace carb;
using namespace carb::tasking;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{
omni::physics::ovstage::OvstageSource* seedOvstageKnownKeysForInitialLoad(
    AttachedStage& attachedStage,
    const std::vector<std::string>& scanRoots)
{
    auto* ovstageSource = dynamic_cast<omni::physics::ovstage::OvstageSource*>(attachedStage.getSource());
    if (!ovstageSource)
        return nullptr;

    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return nullptr;

    (void)scanRoots;

    // Seed and transform prefetch share one prim set: bodies, colliders and their ancestors
    // (ancestors for joint-frame world transforms). exists() falls back to a live query for
    // unseeded keys, so joints -- never exists()-checked at load -- are left out.
    std::vector<omni::physics::parse::ObjectKey> physicsKeys;
    ovstageSource->collectSchemaKeys(ovstageSource->internToken("PhysicsRigidBodyAPI"), physicsKeys);
    ovstageSource->collectSchemaKeys(ovstageSource->internToken("PhysicsCollisionAPI"), physicsKeys);
    const std::vector<omni::physics::parse::ObjectKey> ancestors = ovstageSource->collectAncestors(physicsKeys);
    physicsKeys.insert(physicsKeys.end(), ancestors.begin(), ancestors.end());

    ovstageSource->clearKnownKeys();
    ovstageSource->seedKnownKeys(physicsKeys);

    ovstageSource->prefetchBucket(physicsKeys,
                                  { omni::physics::ovstage::conv::kFabricWorldMatrix,
                                    omni::physics::ovstage::conv::kFabricLocalMatrix,
                                    omni::physics::ovstage::conv::kLocalTransform,
                                    omni::physics::ovstage::conv::kResetXformStack });
    return ovstageSource;
}

bool doesBodyExist(AttachedStage& attachedStage, omni::physics::parse::ObjectKey bodyKey)
{
    return attachedStage.getObjectDatabase()->findEntry(bodyKey, eBody) != kInvalidObjectId;
}

void createJoint(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, PhysxJointDesc* desc)
{
    if (desc != nullptr)
    {
        ObjectDb* objectDb = attachedStage.getObjectDatabase();

        ObjectId body0 = objectDb->findEntry(desc->body0, eBody);
        if (body0 == kInvalidObjectId)
        {
            body0 = objectDb->findEntry(desc->body0, eArticulationLink);
        }
        const bool body0Dynamic = body0 == kInvalidObjectId ? false : isRigidBodyDynamic(body0);
        ObjectId body1 = objectDb->findEntry(desc->body1, eBody);
        if (body1 == kInvalidObjectId)
        {
            body1 = objectDb->findEntry(desc->body1, eArticulationLink);
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
                const omni::physics::parse::ObjectKey bodyKey = it->first;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyKey, *bodyDesc);
                // If PhysXUsdPhysicsInterface::setForceParseOnlySingleScene is used it may return kInvalidObjectId
                if (id != kInvalidObjectId)
                {
                    // The pathText overload also feeds PrimHierarchyStorage; the bare-key one
                    // skips it, breaking cascade-delete-on-parent-removal for these bodies.
                    attachedStage.getObjectDatabase()->findOrCreateEntry(bodyKey, attachedStage.textFor(bodyKey), eBody, id);
                    if (bodyDesc->type == eDynamicBody)
                    {
                        attachedStage.bufferRequestRigidBodyMassUpdate(bodyKey);
                    }
                }

            }
            ICE_FREE(bodyDesc);
        }

        it++;
    }

    bodyMap.clear();

    // Additional bodies
    for (std::pair<omni::physics::parse::ObjectKey, BodyDescAndColliders>& ref : additonalBodies)
    {
        PhysxRigidBodyDesc* bodyDesc = ref.second.desc;

        // we should have only static bodies here
        if (bodyDesc->type == eStaticBody)
        {
            const omni::physics::parse::ObjectKey bodyKey = ref.first;
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyKey, *bodyDesc);

            // Guarded: findOrCreateEntry is backed by map::insert (no overwrite), so an entry
            // once written as kInvalidObjectId can never be replaced (REQ-SIM-OBJECTDB-001 AC-2).
            if (id != kInvalidObjectId)
            {
                attachedStage.getObjectDatabase()->findOrCreateEntry(bodyKey, attachedStage.textFor(bodyKey), eBody, id);
            }
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
                ObjectId link0Id = objectDb->findEntry(jointDesc.body0, eArticulationLink);
                if (link0Id != kInvalidObjectId)
                {
                    ObjectId link1Id = objectDb->findEntry(jointDesc.body1, eArticulationLink);
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
                              attachedStage.textFor(joints[i].path));
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

void createDeformableBody(AttachedStage& attachedStage, PhysxDeformableBodyDesc* bodyDesc, omni::physics::parse::ObjectKey bodyKey)
{
    if (bodyDesc)
    {
        if (bodyDesc->type == eVolumeDeformableBody ||
            bodyDesc->type == eSurfaceDeformableBody)
        {
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyKey, *bodyDesc);
            // If PhysXUsdPhysicsInterface::setForceParseOnlySingleScene is used it may return kInvalidObjectId
            if (id != kInvalidObjectId)
            {
                // The pathText overload also feeds PrimHierarchyStorage.
                attachedStage.getObjectDatabase()->findOrCreateEntry(bodyKey, attachedStage.textFor(bodyKey), bodyDesc->type, id);
            }
        }
        ICE_FREE(bodyDesc);
    }
}

// Compose each joint's local pose (scaled to remove the body's own scale) with the body's
// world matrix, then compare the two resulting world translations/rotations within tolerance.
// Relies on the Gf/PhysX matrix-operand-order swap documented in MatrixTools.h.
bool isJointBodyTransformEqual(const ::physx::PxMat44d& body0World, bool body0Valid,
                                const ::physx::PxMat44d& body1World, bool body1Valid,
                                const ::physx::PxVec3& localPose0Position, const ::physx::PxQuat& localPose0Orientation,
                                const ::physx::PxVec3& localPose1Position, const ::physx::PxQuat& localPose1Orientation,
                                double jointBodyTransformCheckTolerance,
                                bool checkPosition, bool checkRotation,
                                unsigned char axis)
{
    auto getJointBodyPose = [](const ::physx::PxMat44d& bodyWorld, bool bodyValid,
                                const ::physx::PxVec3& locPos, const ::physx::PxQuat& locRot) -> ::physx::PxTransform {
        if (!bodyValid)
        {
            return ::physx::PxTransform(locPos, locRot);
        }
        const ::physx::PxVec3 scale = getScale(bodyWorld);
        const ::physx::PxVec3 scaledPos(locPos.x / scale.x, locPos.y / scale.y, locPos.z / scale.z);
        const ::physx::PxMat44d localPose = makeMatrix(::physx::PxTransform(scaledPos, locRot));
        // Gf composes local*bodyWorld (row-vector convention: local applied first); the
        // PhysX (column-vector) equivalent of the same element-copied matrices swaps the
        // operand order, so bodyWorld*localPose still applies localPose first.
        return toTransform(bodyWorld * localPose);
    };

    const ::physx::PxTransform body0tm = getJointBodyPose(body0World, body0Valid, localPose0Position, localPose0Orientation);
    const ::physx::PxTransform body1tm = getJointBodyPose(body1World, body1Valid, localPose1Position, localPose1Orientation);

    const double eps = jointBodyTransformCheckTolerance;

    if (checkPosition)
    {
        if (axis < 3)
        {
            const ::physx::PxVec3 tran = body0tm.p - body1tm.p;
            static const unsigned char axes[3][2] = { { 1, 2 }, { 0, 2 }, { 0, 1 } };
            const double a1 = tran[axes[axis][0]];
            const double a2 = tran[axes[axis][1]];
            if (!(fabs(a1) < eps && fabs(a2) < eps))
            {
                return false;
            }
        }
        else
        {
            const ::physx::PxVec3 tran = body0tm.p - body1tm.p;
            if (!(tran.magnitudeSquared() <= eps * eps))
            {
                return false;
            }
        }
    }

    if (checkRotation)
    {
        const ::physx::PxQuat& rot0 = body0tm.q;
        const ::physx::PxQuat& rot1 = body1tm.q;
        // Double-cover-aware closeness (GfIsClose(GfQuatd,GfQuatd) semantics): compare
        // real+imaginary parts against +rot1 OR -rot1, both scalar-eps on the real part
        // and distance-eps on the imaginary part.
        auto imagClose = [eps](const ::physx::PxQuat& a, const ::physx::PxQuat& b) {
            const ::physx::PxVec3 d(a.x - b.x, a.y - b.y, a.z - b.z);
            return d.magnitudeSquared() <= eps * eps;
        };
        const bool same = fabs(rot0.w - rot1.w) < eps && imagClose(rot0, rot1);
        const bool opposite = fabs(rot0.w + rot1.w) < eps &&
                               imagClose(rot0, ::physx::PxQuat(-rot1.x, -rot1.y, -rot1.z, -rot1.w));
        if (!(same || opposite))
        {
            return false;
        }
    }

    return true;
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
    omni::physics::parse::KnownTokens tok;
    tok.intern(src);
    bool processScene = src.hasSchema(key, tok.physxSceneAPI);
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
    // applied-API gate via the source (no UsdPrim). Parent lookup routes
    // through IPhysicsSource::getParent rather than SdfPath::GetParentPath
    // (ADR-0019 decision 2).
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> autoAttachmentKeys;
    if (src)
    {
        const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
        const omni::physics::parse::TokenId autoApiTok = tok.PhysxAutoDeformableAttachmentAPI;
        for (const auto& attachment : attachments)
        {
            const omni::physics::parse::ObjectKey attachmentKey = attachment.path;
            const omni::physics::parse::ObjectKey parentKey = src->getParent(attachmentKey);
            if (src->exists(parentKey) && src->hasSchema(parentKey, autoApiTok))
            {
                autoAttachmentKeys.insert(parentKey);
            }
        }
        // In-memory layouts (attach that cannot author sub-prims) name their parent directly.
        for (const auto& layout : attachedStage.getGeneratedAutoAttachmentLayouts())
        {
            if (src->exists(layout.first) && src->hasSchema(layout.first, autoApiTok))
                autoAttachmentKeys.insert(layout.first);
        }
    }

    // Ancestry test routes through AttachedStage::isAncestorOrSelf (source
    // parent-chain walk) rather than SdfPath::HasPrefix (ADR-0019 decision 2).
    auto invalidateDescsUnderKey = [&attachedStage](auto& vec, omni::physics::parse::ObjectKey parentKey)
    {
        for (auto& entry : vec)
        {
            if (attachedStage.isAncestorOrSelf(parentKey, entry.path))
            {
                ICE_FREE(entry.desc);
            }
        }
    };

    for (const omni::physics::parse::ObjectKey autoAttachmentKey : autoAttachmentKeys)
    {
        bool attachmentDataRecomputed = false;
        if (!omni::physx::updateAutoDeformableAttachment(autoAttachmentKey, attachmentDataRecomputed))
        {
            CARB_LOG_WARN("refreshAutoDeformableAttachments: updateAutoDeformableAttachment failed for %s",
                attachedStage.textFor(autoAttachmentKey));
        }

        // Only invalidate pre-parsed descs if attachment data was actually recomputed
        if (attachmentDataRecomputed)
        {
            invalidateDescsUnderKey(attachments, autoAttachmentKey);
            invalidateDescsUnderKey(collisionFilters, autoAttachmentKey);
        }
    }
}

// Auto-attachment prims whose sub-prims cannot be authored (no live USD stage, i.e. an ovstage
// attach) get in-memory generated children instead of nothing. Prims that already carry
// authored sub-prims (scenes generated elsewhere) keep those.
void generateAutoDeformableAttachmentLayouts(AttachedStage& attachedStage,
    const KeySet& autoAttachmentKeys,
    DeformableAttachmentVector& attachments, DeformableCollisionFilterVector& collisionFilters)
{
    if (autoAttachmentKeys.empty() || omni::physx::attachmentauthoring::canAuthor(attachedStage))
        return;
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;

    // authored sub-prim count per parent
    std::unordered_map<omni::physics::parse::ObjectKey, uint32_t, omni::physics::parse::ObjectKey::Hash> authoredParents;
    for (const auto& attachment : attachments)
        ++authoredParents[src->canonicalKey(src->getParent(attachment.path))];
    for (const auto& filter : collisionFilters)
        ++authoredParents[src->canonicalKey(src->getParent(filter.path))];

    for (const omni::physics::parse::ObjectKey autoAttachmentKey : autoAttachmentKeys)
    {
        auto authored = authoredParents.find(src->canonicalKey(autoAttachmentKey));
        if (authored != authoredParents.end())
        {
            // Authored children win, but an incomplete set (interrupted or older producer) is
            // not completed here, so make the gap visible.
            const uint32_t expected = omni::physx::expectedAutoDeformableAttachmentSubPrimCount(attachedStage, autoAttachmentKey);
            if (expected != 0 && expected != authored->second)
            {
                CARB_LOG_WARN("Auto deformable attachment %s carries %u authored sub prims where %u are expected; "
                              "the missing ones are not generated, re-create the attachment to refresh them",
                    attachedStage.textFor(autoAttachmentKey), authored->second, expected);
            }
            continue;
        }
        if (!omni::physx::buildGeneratedAutoDeformableAttachmentLayout(attachedStage, autoAttachmentKey))
            continue;
        const GeneratedAutoAttachmentLayout* layout = attachedStage.getGeneratedAutoAttachmentLayout(autoAttachmentKey);
        for (const GeneratedAutoAttachmentChild& child : layout->children)
        {
            if (child.type == eDeformableCollisionFilter)
            {
                DeformableCollisionFilterDescAndPath entry;
                entry.path = child.key;
                entry.desc = omni::physx::makeGeneratedDeformableCollisionFilterDesc(child);
                collisionFilters.push_back(entry);
            }
            else
            {
                DeformableAttachmentDescAndPath entry;
                entry.path = child.key;
                entry.desc = omni::physx::makeGeneratedDeformableAttachmentDesc(child);
                attachments.push_back(entry);
            }
        }
    }
}

// When a desc is null (invalidated by refreshAutoDeformableAttachments above), does a
// single-prim scanStage() rescan to rebuild it.
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
            // A generated (in-memory) child has no prim to rescan; rebuild its desc from the layout.
            if (const GeneratedAutoAttachmentChild* generated = attachedStage.findGeneratedAutoAttachmentChild(attachments[i].path))
                attachments[i].desc = omni::physx::makeGeneratedDeformableAttachmentDesc(*generated);
        }
        if (attachments[i].desc == nullptr)
        {
            // Single-prim scanStage produces a typed parse-lib desc;
            // parseDeformableAttachment translates it into the
            // consumer-side usdparser::PhysxDeformableAttachmentDesc.
            const omni::physics::parse::IPhysicsSource* dsrc = attachedStage.getSource();
            if (!dsrc || !dsrc->exists(attachments[i].path))
                return;

            // Subtree scan rooted at the attachment prim (default predicate, no
            // instance proxies), routed through the active scan backend.
            const std::vector<std::string> scanRoots{ std::string(attachedStage.textViewFor(attachments[i].path)) };
            static const std::vector<std::string> kNoExclude;
            omni::physics::parse::ScanOptions scanOptions;
            scanOptions.descendantScope = omni::physics::parse::DescendantScope::eActive;
            omni::physics::parse::ScannedStage scanned = omni::physics::parse::scanStage(
                attachedStage.attachTarget(), scanRoots, kNoExclude, scanOptions,
                omni::physx::usdparser::iceDescriptorAllocator());
            if (scanned.attachments.empty())
                return;

            attachments[i].desc = parseDeformableAttachment(scanned, *scanned.attachments[0], attachedStage);
            if (attachments[i].desc == nullptr)
                return;
        }

        id = physInt->createObject(attachedStage, attachments[i].path, *attachments[i].desc);

        if (id != kInvalidObjectId)
            // The pathText overload also feeds PrimHierarchyStorage, needed for cascade-delete
            // when an ancestor prim is removed.
            objectDb->findOrCreateEntry(
                attachments[i].path, attachedStage.textViewFor(attachments[i].path), attachments[i].desc->type, id);

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
            if (const GeneratedAutoAttachmentChild* generated = attachedStage.findGeneratedAutoAttachmentChild(collisionFilters[i].path))
                collisionFilters[i].desc = omni::physx::makeGeneratedDeformableCollisionFilterDesc(*generated);
        }
        if (collisionFilters[i].desc == nullptr)
        {
            // Single-prim scanStage; same pattern as the attachment
            // branch above.
            const omni::physics::parse::IPhysicsSource* dsrc = attachedStage.getSource();
            if (!dsrc || !dsrc->exists(collisionFilters[i].path))
                return;

            // Subtree scan rooted at the filter prim (default predicate, no instance
            // proxies), routed through the active scan backend.
            const std::vector<std::string> scanRoots{ std::string(
                attachedStage.textViewFor(collisionFilters[i].path)) };
            static const std::vector<std::string> kNoExclude;
            omni::physics::parse::ScanOptions scanOptions;
            scanOptions.descendantScope = omni::physics::parse::DescendantScope::eActive;
            omni::physics::parse::ScannedStage scanned = omni::physics::parse::scanStage(
                attachedStage.attachTarget(), scanRoots, kNoExclude, scanOptions,
                omni::physx::usdparser::iceDescriptorAllocator());
            if (scanned.deformableCollisionFilters.empty())
                return;

            collisionFilters[i].desc =
                parseDeformableCollisionFilter(scanned, *scanned.deformableCollisionFilters[0], attachedStage);
            if (collisionFilters[i].desc == nullptr)
                return;
        }

        id = physInt->createObject(attachedStage, collisionFilters[i].path, *collisionFilters[i].desc);

        if (id != kInvalidObjectId)
            // The pathText overload, as in createDeformableAttachments above.
            objectDb->findOrCreateEntry(collisionFilters[i].path,
                                        attachedStage.textViewFor(collisionFilters[i].path),
                                        collisionFilters[i].desc->type, id);

        ICE_FREE(collisionFilters[i].desc);
    }
}

void createFilteredPairs(AttachedStage& attachedStage, const CollisionPairVector& pairsVector, const KeySet& filteredPairsPaths)
{
    ObjectDb& objectDb = *attachedStage.getObjectDatabase();
    std::unordered_map<omni::physics::parse::ObjectKey, FilteredPairDesc, omni::physics::parse::ObjectKey::Hash> blockDescMap;
    for (const omni::physics::parse::ObjectKey key : filteredPairsPaths)
    {
        blockDescMap[key] = FilteredPairDesc();
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
                        pair.second,
                        [&](omni::physics::parse::ObjectKey childKey) -> bool
                        {
                            bool pairFound = false;
                            const ObjectIdMap* entriesSecond = objectDb.getEntries(childKey);
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

ObjectId createObject(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, PhysxObjectDesc* desc,
    PhysXUsdPhysicsInterface& physicsInterface, ObjectDb& objectDb)
{
    if (!desc)
        return kInvalidObjectId;

    const ObjectId id = physicsInterface.createObject(attachedStage, primKey, *desc);
    if (id != kInvalidObjectId)
        // The pathText overload, not the bare-key one: objects created here (e.g. materials)
        // are still looked up by SdfPath elsewhere, so mPathMap must stay populated too.
        objectDb.findOrCreateEntry(primKey, attachedStage.textFor(primKey), desc->type, id);
    return id;
}

ObjectId createObject(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, PhysxObjectDesc* desc, bool deleteDesc = true)
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
void invertCollisionGroupMembers(const AttachedStage& attachedStage,
                                 const omni::physics::parse::ScannedStage& scanned,
                                 size_t beginIdx, size_t endIdx,
                                 CollisionGroupsMap& cgMap)
{
    // Re-key from scanStage's source into attachedStage's: they are different intern tables
    // even when scanning the full stage. keyFor(string_view) populates its table lazily on a
    // miss; the source serializes that internally, so the caller's parallelFor can't race.
    for (size_t i = beginIdx; i < endIdx; ++i)
    {
        const auto& group = scanned.collisionGroups[i];
        const omni::physics::parse::ObjectKey groupKey =
            attachedStage.keyFor(scanned.source().sourceKeyToString(group->primKey));
        for (const omni::physics::parse::ObjectKey member : group->sourceMembers)
        {
            const omni::physics::parse::ObjectKey memberKey =
                attachedStage.keyFor(scanned.source().sourceKeyToString(member));
            cgMap[memberKey].push_back(groupKey);
        }
    }
}

void setupCollisionGroups(AttachedStage& attachedStage, const KeySet& collisionGroupsPaths)
{
    const ObjectDb& db = *attachedStage.getObjectDatabase();

    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();

    for (const omni::physics::parse::ObjectKey collisionGroupKey : collisionGroupsPaths)
    {
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
            std::vector<omni::physics::parse::ObjectKey> targets;
            if (const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource())
            {
                src->getRelationshipTargets(collisionGroupKey, tok.physicsFilteredGroups, targets);
            }
            for (const omni::physics::parse::ObjectKey filterKey : targets)
            {
                const ObjectIdMap* filterMap = db.getEntries(filterKey);
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
    }
}

void createParticleSystemsAndObjects(AttachedStage& attachedStage, const std::vector<ParticleSystemDesc*>& particleSysDescs, const std::vector<ParticleDesc*>& particleDescs,
    CollisionPairVector& filteredPairs)
{
    // create particle systems
    for (ParticleSystemDesc* particleSys : particleSysDescs)
    {
        createObject(attachedStage, particleSys->systemKey, particleSys, false);

        for (size_t i = 0; i < particleSys->filteredCollisions.size(); i++)
        {
            filteredPairs.push_back(std::make_pair(particleSys->systemKey, particleSys->filteredCollisions[i]));
        }
        ICE_FREE(particleSys)
    }

    // create particles
    for (ParticleDesc* particleDesc : particleDescs)
    {
        createObject(attachedStage, particleDesc->primKey, particleDesc);
    }
}

omni::physx::usdparser::ObjectId findOrCreatePhysXObject(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, PhysxObjectDesc* objectDesc,
    PhysXUsdPhysicsInterface& physicsInterface, ObjectDb& objectDb)
{
    ObjectId objectId = objectDb.findEntry(key, objectDesc->type);
    if (objectId != kInvalidObjectId)
        return objectId;
    else
        return createObject(attachedStage, key, objectDesc, physicsInterface, objectDb);
}

void loadVehicle(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primObjKey, VehicleComponentTracker& vehicleComponentTracker)
{
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
                driveDesc->engineId = findOrCreatePhysXObject(attachedStage, driveDesc->engine->key, driveDesc->engine,
                    physicsInterface, objectDb);
            }
            else
            {
                CARB_ASSERT(vehicleDesc.drive->type == ObjectType::eVehicleDriveBasic);
                DriveBasicDesc* driveDesc = static_cast<DriveBasicDesc*>(vehicleDesc.drive);
                driveDesc->id = findOrCreatePhysXObject(attachedStage, driveDesc->key, driveDesc,
                    physicsInterface, objectDb);
            }
        }

        for (WheelAttachmentDesc& wheelAttachment : vehicleDesc.wheelAttachments)
        {
            wheelAttachment.id = createObject(attachedStage, wheelAttachment.key, &wheelAttachment, physicsInterface, objectDb);

            WheelDesc* wheelDesc = wheelAttachment.wheel;
            wheelAttachment.wheelId = findOrCreatePhysXObject(attachedStage, wheelDesc->key, wheelDesc,
                physicsInterface, objectDb);

            TireDesc* tireDesc = wheelAttachment.tire;
            wheelAttachment.tireId = findOrCreatePhysXObject(attachedStage, tireDesc->key, tireDesc,
                physicsInterface, objectDb);

            SuspensionDesc* suspDesc = wheelAttachment.suspension;
            wheelAttachment.suspensionId = findOrCreatePhysXObject(attachedStage, suspDesc->key, suspDesc,
                physicsInterface, objectDb);

            if (wheelAttachment.tire->frictionTableKey.valid())
                wheelAttachment.tire->frictionTableId = objectDb.findEntry(
                    wheelAttachment.tire->frictionTableKey, eVehicleTireFrictionTable);
            else
                wheelAttachment.tire->frictionTableId = kInvalidObjectId;

            if (wheelAttachment.collisionGroupKey.valid())
            {
                wheelAttachment.collisionGroupId = objectDb.findEntry(
                    wheelAttachment.collisionGroupKey, eCollisionGroup);
            }
            else
                wheelAttachment.collisionGroupId = kInvalidObjectId;

            if (wheelAttachment.state & WheelAttachmentDesc::eHAS_SHAPE)
                wheelAttachment.shapeId =
                    objectDb.findEntry(wheelAttachment.shapeKey, eShape);
            else
                wheelAttachment.shapeId = kInvalidObjectId;
        }

        for (WheelControllerDesc& wheelController : vehicleDesc.wheelControllers)
        {
            wheelController.id = createObject(attachedStage, wheelController.key, &wheelController, physicsInterface, objectDb);
        }

        vehicleDesc.bodyId = objectDb.findEntry(primObjKey, eBody);

        ObjectId vehicleId = createObject(attachedStage, primObjKey, &vehicleDesc, physicsInterface, objectDb);

        if (vehicleId != kInvalidObjectId)
        {
            if (vehicleControllerType != eUndefined)
            {
                if (vehicleControllerType == eVehicleControllerStandard)
                    createObject(attachedStage, primObjKey, &vehicleControllerDesc, physicsInterface, objectDb);
                else
                {
                    CARB_ASSERT(vehicleControllerType == eVehicleControllerTank);
                    createObject(attachedStage, primObjKey, &vehicleTankControllerDesc, physicsInterface, objectDb);
                }
            }
        }
        else
        {
            for (WheelControllerDesc& wheelController : vehicleDesc.wheelControllers)
            {
                if (wheelController.id != kInvalidObjectId)
                {
                    physicsInterface.releaseObject(attachedStage, wheelController.key, wheelController.id);
                    wheelController.id = kInvalidObjectId;
                }
            }

            for (WheelAttachmentDesc& wheelAttachment : vehicleDesc.wheelAttachments)
            {
                if (wheelAttachment.id != kInvalidObjectId)
                {
                    physicsInterface.releaseObject(attachedStage, wheelAttachment.key, wheelAttachment.id);
                    wheelAttachment.id = kInvalidObjectId;
                }
            }
        }
    }
}

// RAII IPhysicsDataWrite batch: closes on every early return, including exceptions.
// `dw` may be null (no write sink), in which case both calls are skipped.
struct DataWriteScope
{
    omni::physics::parse::IPhysicsDataWrite* dw;
    explicit DataWriteScope(omni::physics::parse::IPhysicsDataWrite* d) : dw(d) { if (dw) dw->beginWrite(); }
    ~DataWriteScope() { if (dw) dw->endWrite(); }
};

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
    // Visits every load object under each scan root (root inclusive) via IPhysicsSource,
    // honoring the replicator exclude set; `visit` returns true to prune that object's
    // subtree. The eActiveInstanced scope reproduces the UsdTraverseInstanceProxies walk.
    void forEachLoadObject(const std::vector<std::string>& scanRoots, const PathSet* excludePaths,
                           const std::function<bool(omni::physics::parse::ObjectKey)>& visit)
    {
        const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();
        if (!src)
            return;
        for (const std::string& root : scanRoots)
        {
            src->forEachDescendantPruned(
                mAttachedStage.keyFor(root),
                [&](omni::physics::parse::ObjectKey key) -> bool
                {
                    if (excludePaths && !excludePaths->empty())
                    {
                        // A top-level (root-prim) exclude entry is never pruned: the legacy
                        // stage->Traverse() walk yielded the root prim unchecked, so a
                        // root-prim-level exclude (e.g. "/World") was a no-op and the subtree
                        // was still visited.
                        const bool isTopLevel = src->getParent(key) == src->getRootKey();
                        if (!isTopLevel && excludePaths->find(key) != excludePaths->end())
                            return true; // skip the excluded prim and prune its subtree
                    }
                    return visit(key);
                },
                omni::physics::parse::DescendantScope::eActiveInstanced);
        }
    }

    // Applied-API side effects that the SCANNED lists do not carry.
    //
    // gatherScannedSideEffects() reconstructs its side effects from ScannedStage's
    // typed descriptor lists (shapes, bodies, articulations, vehicles, particles,
    // ccts). PhysxForceAPI and PhysxContactReportAPI have NO descriptor list in the
    // parse library at all, so nothing on the scanned path can observe them and both
    // were silently dropped for every non-USD source: no PhysxForceDesc was ever
    // built, so ForceAPI forces were never applied under ovstage.
    //
    // They are recovered here with the same source-routed applied-schema walk the USD
    // path uses. This is a targeted sweep, not the full gatherPerPrimSideEffects: the
    // other branches there are already covered by the scanned lists and running them
    // twice would double-register.
    void gatherSourceOnlySideEffects(const std::vector<std::string>& scanRoots, const PathSet* excludePaths)
    {
        const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();
        if (!src)
            return;

        ObjectDb* objectDb = mAttachedStage.getObjectDatabase();
        if (!objectDb)
            return;

        // Intern the two names once. TokenId compare is an integer compare
        // against the source's own intern table.
        const omni::physics::parse::TokenId forceApiId = src->internToken("PhysxForceAPI");
        const omni::physics::parse::TokenId contactReportApiId = src->internToken("PhysxContactReportAPI");
        // Same story for the auto-attachment prims: no descriptor list carries them, and the
        // in-memory sub-prim generation needs the set (REQ-SIM-AUTOATTACH-001 AC-2).
        const omni::physics::parse::TokenId autoAttachmentApiId = src->internToken("PhysxAutoDeformableAttachmentAPI");
        if (!forceApiId.valid() && !contactReportApiId.valid() && !autoAttachmentApiId.valid())
            return;

        auto onForceKey = [&](omni::physics::parse::ObjectKey primObjKey)
        {
            objectDb->addSchemaAPI(primObjKey, SchemaAPIFlag::ePhysxForceAPI);
            PhysxForceDesc* desc = parsePhysxForce(mAttachedStage, primObjKey);
            mPhysxForceDescs.push_back(std::make_pair(primObjKey, desc));
        };
        auto onContactReportKey = [&](omni::physics::parse::ObjectKey primObjKey)
        {
            objectDb->addSchemaAPI(primObjKey, SchemaAPIFlag::eContactReportAPI);
        };

        // This is called on every attach with an external (e.g. ovstage) source,
        // regardless of physics relevance -- so the O(stage size) route below
        // (forEachLoadObject -> forEachAppliedSchema per prim) dominated attach
        // cost on scenes with a lot of physics-irrelevant content (NVBug 6532970):
        // it was the single largest contributor to ovstage attach time by a wide
        // margin, well above anything in the ovstage scan/ordering path itself.
        // ovstage exposes schema membership as an O(matching-prim-count) query
        // (collectSchemaKeys, backed by a schema-membership cache or a single
        // targeted query -- see OvstageSource::collectSchemaKeysRaw), so prefer
        // that: it visits only prims that actually carry the schema, never the
        // rest of the stage.
        // collectSchemaKeys() reports EVERY prim on the stage that carries the
        // schema, with no way to restrict to scanRoots / excludePaths. That is
        // exactly right for a whole-stage initial load, but a scoped re-parse
        // (loadPhysicsFromPrimitive passes the changed subtrees as scanRoots,
        // AC-15) or a load with a non-empty exclude set would then re-apply
        // PhysxForceAPI / PhysxContactReportAPI side effects for out-of-scope
        // prims. Gate the fast path to an unscoped whole-stage scan with no
        // excludes; anything scoped falls through to the scope-honoring
        // forEachLoadObject walk below.
        const bool wholeStageScan =
            scanRoots.size() == 1 && scanRoots[0] == "/" && (!excludePaths || excludePaths->empty());
        if (auto* ovstageSource =
                wholeStageScan ? dynamic_cast<const omni::physics::ovstage::OvstageSource*>(src) : nullptr)
        {
            bool ok = true;
            if (forceApiId.valid())
            {
                std::vector<omni::physics::parse::ObjectKey> keys;
                ok = ok && ovstageSource->collectSchemaKeys(forceApiId, keys);
                if (ok)
                    for (const omni::physics::parse::ObjectKey key : keys)
                        onForceKey(key);
            }
            if (ok && contactReportApiId.valid())
            {
                std::vector<omni::physics::parse::ObjectKey> keys;
                ok = ovstageSource->collectSchemaKeys(contactReportApiId, keys);
                if (ok)
                    for (const omni::physics::parse::ObjectKey key : keys)
                        onContactReportKey(key);
            }
            if (ok && autoAttachmentApiId.valid())
            {
                std::vector<omni::physics::parse::ObjectKey> keys;
                ok = ovstageSource->collectSchemaKeys(autoAttachmentApiId, keys);
                if (ok)
                    mAutoDeformableAttachmentKeys.insert(keys.begin(), keys.end());
            }
            if (ok)
                return;
            // collectSchemaKeys() only reports failure when the source has no
            // live instance/dict to query at all -- an attach-time inconsistency
            // rather than an ordinary cache miss. Fall through to the walk below
            // so results are still correct in that case.
        }

        forEachLoadObject(scanRoots, excludePaths, [&](omni::physics::parse::ObjectKey primObjKey) -> bool
        {
            src->forEachAppliedSchema(primObjKey, [&](omni::physics::parse::TokenId tok)
            {
                if (forceApiId.valid() && tok == forceApiId)
                    onForceKey(primObjKey);
                else if (contactReportApiId.valid() && tok == contactReportApiId)
                    onContactReportKey(primObjKey);
                else if (autoAttachmentApiId.valid() && tok == autoAttachmentApiId)
                    mAutoDeformableAttachmentKeys.insert(primObjKey);
            });
            return false;
        });
    }

    void gatherPerPrimSideEffects(const std::vector<std::string>& scanRoots, const PathSet* excludePaths)
    {
        DeformableAttachmentHistoryMap& attHistory =
            mAttachedStage.getDeformableAttachmentHistoryMap();
        DeformableCollisionFilterHistoryMap& filterHistory =
            mAttachedStage.getDeformableCollisionFilterHistoryMap();

        const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();

        // Intern the known API names once (see gatherSourceOnlySideEffects): this
        // walk runs over every prim on every attach, so round-tripping each
        // applied schema through std::string into a TfToken just to compare
        // against a handful of constants would put registry and string work on
        // the hot path. TokenId compare is an integer compare.
        omni::physics::parse::KnownTokens knownTok;
        if (src)
            knownTok.intern(*src);

        forEachLoadObject(scanRoots, excludePaths, [&](omni::physics::parse::ObjectKey primObjKey) -> bool
        {
            ObjectDb* objectDb = mAttachedStage.getObjectDatabase();

            // Applied-API list via the source (= prim.GetAppliedSchemas()).
            src->forEachAppliedSchema(primObjKey, [&](omni::physics::parse::TokenId tok)
            {
                if (tok == knownTok.physxCharacterControllerAPI)
                {
                    mParsingFlags |= ParsingFlag::eParseInternal;
                }
                else if (tok == knownTok.physxForceAPI)
                {
                    objectDb->addSchemaAPI(primObjKey, SchemaAPIFlag::ePhysxForceAPI);
                    PhysxForceDesc* desc = parsePhysxForce(mAttachedStage, primObjKey);
                    mPhysxForceDescs.push_back(std::make_pair(primObjKey, desc));
                }
                else if (tok == knownTok.physicsFilteredPairsAPI)
                {
                    mFilteredPairsPaths.insert(primObjKey);
                    objectDb->addSchemaAPI(primObjKey, SchemaAPIFlag::eFilteredPairsAPI);
                }
                else if (tok == knownTok.physxParticleSetAPI || tok == knownTok.physxParticleSamplingAPI)
                {
                    mParsingFlags |= ParsingFlag::eParseParticles;
                }
                else if (tok == knownTok.physxVehicleAPI)
                {
                    mParsingFlags |= ParsingFlag::eParseVehicles;
                }
                else if (tok == knownTok.physxVehicleContextAPI)
                {
                    // Vehicle context parsing moved into the native
                    // walker in 7A.1 (ADR-0008); the descriptor flows
                    // through scanned.vehicleContexts.  We still flip
                    // the parsing flag here so the post-load vehicle
                    // creation block runs.
                    mParsingFlags |= ParsingFlag::eParseVehicles;
                }
                else if (tok == knownTok.PhysxAutoDeformableAttachmentAPI)
                {
                    mAutoDeformableAttachmentKeys.insert(primObjKey);
                }
                else if (tok == knownTok.physicsCollisionAPI)
                {
                    objectDb->addSchemaAPI(primObjKey, SchemaAPIFlag::eCollisionAPI);
                }
                else if (tok == knownTok.physxContactReportAPI)
                {
                    objectDb->addSchemaAPI(primObjKey, SchemaAPIFlag::eContactReportAPI);
                }
            });

            if (src->isA(primObjKey, knownTok.pointInstancerType))
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (src->isA(primObjKey, knownTok.physxParticleSystemType))
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (src->isA(primObjKey, knownTok.physxPhysicsJointInstancerType))
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (src->isA(primObjKey, knownTok.physicsCollisionGroupType))
            {
                mCollisionGroupsPrims.insert(primObjKey);
            }
            else if (src->isA(primObjKey, knownTok.physxVehicleTireFrictionTableType))
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
                auto it = attHistory.find(primObjKey);
                while (it != attHistory.end() && it->first == primObjKey)
                {
                    DeformableAttachmentDescAndPath attachmentDescAndPath;
                    attachmentDescAndPath.path = it->second;
                    attachmentDescAndPath.desc = nullptr;
                    mDeformableAttachmentVector.push_back(attachmentDescAndPath);
                    ++it;
                }
            }
            {
                auto it = filterHistory.find(primObjKey);
                while (it != filterHistory.end() && it->first == primObjKey)
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

    void gatherScannedSideEffects(omni::physics::parse::ScannedStage& scanned)
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
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(shapeUPtr->primKey));
            objectDb->addSchemaAPI(path, SchemaAPIFlag::eCollisionAPI);
            if (!shapeUPtr->sourceFilteredCollisions.empty())
            {
                objectDb->addSchemaAPI(path, SchemaAPIFlag::eFilteredPairsAPI);
                mFilteredPairsPaths.insert(path);
            }
        }

        for (const omni::physics::parse::DescPtr<PhysxRigidBodyDesc>& bodyUPtr : scanned.bodies)
        {
            if (!bodyUPtr)
                continue;
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(bodyUPtr->primKey));
            if (!bodyUPtr->sourceFilteredCollisions.empty())
            {
                objectDb->addSchemaAPI(path, SchemaAPIFlag::eFilteredPairsAPI);
                mFilteredPairsPaths.insert(path);
            }
        }

        for (const omni::physics::parse::DescPtr<PhysxArticulationDesc>& articulationUPtr : scanned.articulations)
        {
            if (!articulationUPtr)
                continue;
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(articulationUPtr->rootPrim));
            if (!articulationUPtr->sourceFilteredCollisions.empty())
            {
                objectDb->addSchemaAPI(path, SchemaAPIFlag::eFilteredPairsAPI);
                mFilteredPairsPaths.insert(path);
            }
        }

        for (const omni::physics::parse::DescPtr<omni::physics::parse::PhysxDeformableBodyDesc>& deformableUPtr : scanned.deformables)
        {
            if (!deformableUPtr)
                continue;
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(deformableUPtr->primKey));
            if (!deformableUPtr->sourceFilteredCollisions.empty())
            {
                objectDb->addSchemaAPI(path, SchemaAPIFlag::eFilteredPairsAPI);
                mFilteredPairsPaths.insert(path);
            }
        }

        for (const omni::physics::parse::DescPtr<omni::physics::parse::CollisionGroupDesc>& collisionGroupUPtr : scanned.collisionGroups)
        {
            if (collisionGroupUPtr)
                mCollisionGroupsPrims.insert(
                    mAttachedStage.keyFor(scanned.source().sourceKeyToString(collisionGroupUPtr->primKey)));
        }
    }

    // Translate scanStage descriptors into runtime objects.  Runs
    // after the backend-dispatched scanStage returns; populates mShapes /
    // mBodyMap / mMaterials / etc. buffers consumed by the body /
    // shape / articulation creation paths.
    void processScannedDescs(omni::physics::parse::ScannedStage& scanned)
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
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanScene->primKey));
            const omni::physics::parse::IPhysicsSource* scnSrc = mAttachedStage.getSource();
            const omni::physics::parse::ObjectKey sceneKey = path;
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
                if (k.valid()) k = mAttachedStage.keyFor(scanned.source().sourceKeyToString(k));
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
                const omni::physics::parse::KnownTokens& tok = mAttachedStage.getKnownTokens();
                if (src && src->hasSchema(path, tok.physxSceneQuasistaticAPI))
                {
                    std::vector<omni::physics::parse::ObjectKey> members;
                    src->resolveCollection(path, src->internToken("quasistaticactors"), members);
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
                const bool serialCollisionGroupInversion = mAttachedStage.hasExternalSource();
                if (collisionGroupsSize < minBatchSize || serialCollisionGroupInversion)
                {
                    invertCollisionGroupMembers(mAttachedStage, scanned, 0, collisionGroupsSize,
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
                    auto&& computeFunc = [this, &scanned, numBatches, batchSize, collisionGroupsSize, &cgMaps](size_t batchIndex)
                    {
                        const size_t batchEnd =
                            batchIndex == (numBatches - 1) ? collisionGroupsSize : (batchIndex + 1) * batchSize;
                        invertCollisionGroupMembers(mAttachedStage, scanned, batchIndex * batchSize, batchEnd, cgMaps[batchIndex]);
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
                const omni::physics::parse::ObjectKey path =
                    mAttachedStage.keyFor(scanned.source().sourceKeyToString(cgUPtr->primKey));
                mCollisionGroupsPrims.insert(path);
                CollisionGroupDesc desc;
                createObject(mAttachedStage, path, &desc, false);
            }
        }

        // ----- Materials --------------------------------------------
        for (auto& matUPtr : scanned.materials)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:material");
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(matUPtr->materialKey));
            PhysxMaterialDesc* desc = matUPtr.release();
            if (desc)
                mMaterials.push_back(std::make_pair(path, desc));
        }

        // ----- PBDMaterials ----------------------------------------
        for (omni::physics::parse::DescPtr<PBDMaterialDesc>& matUPtr : scanned.pbdMaterials)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:pbdMaterial");
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(matUPtr->materialKey));
            PBDMaterialDesc* desc = matUPtr.release();
            if (desc)
                mPDBMatrialsDescs.push_back(std::make_pair(path, desc));
        }

        // ----- DeformableMaterials ---------------------------------
        for (auto& dmUPtr : scanned.deformableMaterials)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:deformableMaterial");
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(dmUPtr->materialKey));
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
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanDesc->primKey));

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
                scanDesc->rigidBody = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanDesc->rigidBody));
            if (scanDesc->sourceGprim.valid())
                scanDesc->sourceGprim = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanDesc->sourceGprim));
            if (scanDesc->type == eConvexMeshShape)
            {
                auto* d = static_cast<ConvexMeshPhysxShapeDesc*>(scanDesc);
                if (d->meshPrimKey.valid())
                    d->meshPrimKey = mAttachedStage.keyFor(scanned.source().sourceKeyToString(d->meshPrimKey));
            }
            else if (scanDesc->type == eTriangleMeshShape ||
                     scanDesc->type == eConvexMeshDecompositionShape ||
                     scanDesc->type == eSpherePointsShape)
            {
                auto* d = static_cast<TriangleMeshPhysxShapeDesc*>(scanDesc);
                if (d->meshPrimKey.valid())
                    d->meshPrimKey = mAttachedStage.keyFor(scanned.source().sourceKeyToString(d->meshPrimKey));
            }

            std::vector<omni::physics::parse::ObjectKey> materials;
            if (!scan::resolveConsumerSideShapeState(mAttachedStage, scanned, scanDesc, materials, mFilteredPairs))
                continue;

            // Instance-proxy fixups (REQ-PARSE-CONSUMER-001 AC-13).
            // Mirrors legacy `parseCollisionDesc` master/instance branch
            // (Collision.cpp:1622-1653).
            {
                const omni::physics::parse::IPhysicsSource* isrc = mAttachedStage.getSource();
                const omni::physics::parse::ObjectKey shapeKey = path;
                if (isrc && isrc->isInstanceProxy(shapeKey))
                {
                    if (scanDesc->sourceMaterials.size() < 2)
                    {
                        // Instance-proxy material binding via the source (no UsdPrim).
                        const omni::physics::parse::ObjectKey matKey = isrc->getMaterialBinding(shapeKey);
                        if (matKey.valid())
                        {
                            materials.clear();
                            materials.push_back(matKey);
                        }
                    }
                    if (scanDesc->rigidBody.valid() && isrc->exists(scanDesc->rigidBody))
                    {
                        getCollisionShapeLocalTransform(mAttachedStage, shapeKey, scanDesc->rigidBody,
                                                        scanDesc->localPos, scanDesc->localRot,
                                                        scanDesc->localScale);
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

            callbacks::collectShapeTimeSampledCallbacks(mAttachedStage, path, cbList);

            mShapes.push_back({ path, desc, materials });
        }

        // ----- RigidBodies -----------------------------------------
        // Resolves each scanned body's source key to an attached-stage ObjectKey and
        // clones the descriptor into ICE storage.
        for (auto& bodyUPtr : scanned.bodies)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:rigidBody");
            PhysxRigidBodyDesc* scanDesc = bodyUPtr.get();
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanDesc->primKey));

            // Translate sourceSimulationOwners → sceneIds.
            scanDesc->sceneIds.clear();
            for (const auto& sk : scanDesc->sourceSimulationOwners)
            {
                if (!sk.valid()) continue;
                const omni::physics::parse::ObjectKey spKey =
                    mAttachedStage.keyFor(scanned.source().sourceKeyToString(sk));
                const ObjectId entry = mAttachedStage.getObjectDatabase()->findEntry(spKey, eScene);
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
                if (!fk.valid()) continue;
                const omni::physics::parse::ObjectKey fpKey =
                    mAttachedStage.keyFor(scanned.source().sourceKeyToString(fk));
                mFilteredPairs.push_back(std::make_pair(path, fpKey));
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
                const omni::physics::parse::ObjectKey bodyKey = path;
                const omni::physics::parse::KnownTokens& tok = mAttachedStage.getKnownTokens();

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
                    ssrc->hasSchema(bodyKey, tok.physxSplinesSurfaceVelocityAPI))
                {
                    bool splinesEnabledAuthored = true;
                    internal::getValue<bool>(mAttachedStage, bodyKey, tok.physxSplinesSurfaceVelocityEnabled,
                                             omni::physics::parse::ReadTime::defaultTime(), splinesEnabledAuthored);

                    if (dyn->surfaceVelocityEnabled && splinesEnabledAuthored)
                    {
                        CARB_LOG_ERROR(
                            "Detected rigid body (%s) with both surface velocity and splines surface velocity, please disable one.",
                            mAttachedStage.textFor(path));
                    }
                    else if (splinesEnabledAuthored)
                    {
                        // Distinguish an absent relationship from a defined-but-empty one
                        // (matches the prior `!splinesRel || splinesList.empty()` gate).
                        const omni::physics::parse::TokenId curveRelTok = tok.physxSplinesSurfaceVelocityCurve;
                        const bool hasCurveRel = ssrc->hasRelationship(bodyKey, curveRelTok);
                        std::vector<omni::physics::parse::ObjectKey> curveKeys;
                        if (hasCurveRel)
                            ssrc->getRelationshipTargets(bodyKey, curveRelTok, curveKeys);
                        // Each rejection names its cause: an unresolved target, a target that
                        // never made it into the attached stage (a physics-only ovstage
                        // population that skipped the curve), and a target of the wrong type
                        // are fixed in different places.
                        if (!hasCurveRel || curveKeys.empty())
                        {
                            CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined: "
                                           "the physxSplinesSurfaceVelocity:surfaceVelocityCurve relationship has no target.",
                                           mAttachedStage.textFor(path));
                        }
                        else
                        {
                            const omni::physics::parse::ObjectKey curveKey = curveKeys[0];
                            const std::string_view curveText = ssrc->sourceKeyToString(curveKey);
                            if (!ssrc->exists(curveKey))
                            {
                                CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined: "
                                               "the curve target %.*s is not present in the attached stage. When populating "
                                               "with ovstage, the physics population must include the referenced BasisCurves prim.",
                                               mAttachedStage.textFor(path), int(curveText.size()), curveText.data());
                            }
                            else if (!ssrc->isA(curveKey, tok.basisCurvesType))
                            {
                                const std::string_view typeText = ssrc->tokenToString(ssrc->getTypeName(curveKey));
                                CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined: "
                                               "the curve target %.*s is not a BasisCurves prim (type '%.*s').",
                                               mAttachedStage.textFor(path), int(curveText.size()), curveText.data(),
                                               int(typeText.size()), typeText.data());
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
                                                   mAttachedStage.textFor(path));
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
                        dyn->splinesCurvePrimKey = mAttachedStage.keyFor(
                            scanned.source().sourceKeyToString(dyn->splinesCurvePrimKey));
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
                                      mAttachedStage.textFor(path));
                        dyn->kinematicBody = true;
                    }
                    mAttachedStage.getAnimatedKinematicBodies().insert(path);
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
            // Translate sourceCollisions ObjectKeys → the attachedStage-space
            // KeySet for downstream body / collider lookups.
            bdDesc.collisions.clear();
            for (const auto& ck : scanDesc->sourceCollisions)
            {
                if (!ck.valid()) continue;
                bdDesc.collisions.insert(mAttachedStage.keyFor(scanned.source().sourceKeyToString(ck)));
            }

            callbacks::collectRigidBodyTimeSampledCallbacks(mAttachedStage, path, cbList);
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
        // mightBeTimeVarying == attr && ValueMightBeTimeVarying).
        const omni::physics::parse::IPhysicsSource* tsSrc = mAttachedStage.getSource();
        auto regScalarIfAuthored = [&](omni::physics::parse::ObjectKey primKey, const std::string& attrName,
                                       OnUpdateObjectFn fn) {
            if (!tsSrc)
                return;
            const omni::physics::parse::TokenId tok = tsSrc->internToken(attrName);
            if (tsSrc->hasAuthoredAttribute(primKey, tok) && tsSrc->mightBeTimeVarying(primKey, tok))
                mAttachedStage.registerTimeSampledAttribute(primKey, tok, fn);
        };
        auto regAnyIfTimeVarying = [&](omni::physics::parse::ObjectKey primKey, const std::string& attrName,
                                       OnUpdateObjectFn fn) {
            if (!tsSrc)
                return;
            const omni::physics::parse::TokenId tok = tsSrc->internToken(attrName);
            if (tsSrc->mightBeTimeVarying(primKey, tok))
                mAttachedStage.registerTimeSampledAttribute(primKey, tok, fn);
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
            const std::string instanceStr = std::string(scanned.source().tokenToString(scanAtt->instanceToken));

            // linkKey/parentKey/parentToken/instanceToken are minted by the
            // SCAN's own source and must be re-keyed/re-interned into
            // mAttachedStage's namespace (ADR-0019 increment 7; mirrors the
            // `rekey` pattern used above for articulations/vehicles).
            const omni::physics::parse::IPhysicsSource* asSrc = mAttachedStage.getSource();
            auto internStr = [asSrc](const std::string& s) -> omni::physics::parse::TokenId
            {
                return asSrc ? asSrc->internToken(s) : omni::physics::parse::TokenId{};
            };
            const omni::physics::parse::ObjectKey linkKey =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanAtt->linkKey));
            const omni::physics::parse::TokenId instanceTokId = internStr(instanceStr);

            if (scanAtt->type == eTendonAttachmentRoot)
            {
                const auto& srcRoot = static_cast<const omni::physics::parse::PhysxTendonSpatialDesc&>(*scanAtt);
                auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonSpatialDesc)();
                engineDesc->type           = eTendonAttachmentRoot;
                engineDesc->gearing        = srcRoot.gearing;
                engineDesc->localPos       = srcRoot.localPos;
                engineDesc->linkKey        = linkKey;
                engineDesc->parentKey      = mAttachedStage.keyFor(scanned.source().sourceKeyToString(srcRoot.parentKey));
                engineDesc->parentToken    = internStr(std::string(scanned.source().tokenToString(srcRoot.parentToken)));
                engineDesc->instanceToken  = instanceTokId;
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
                if (tsSrc && tsSrc->exists(linkKey))
                {
                    const std::string base = "physxTendon:" + instanceStr + ":";
                    regScalarIfAuthored(linkKey, base + "stiffness",      updateSpatialTendonStiffness);
                    regScalarIfAuthored(linkKey, base + "limitStiffness", updateSpatialTendonLimitStiffness);
                    regScalarIfAuthored(linkKey, base + "damping",        updateSpatialTendonDamping);
                    regScalarIfAuthored(linkKey, base + "offset",         updateSpatialTendonOffset);
                    regScalarIfAuthored(linkKey, base + "tendonEnabled",  updateSpatialTendonEnabled);
                    regAnyIfTimeVarying(linkKey, base + "localPos",       updateTendonAttachmentLocalPos);
                }
            }
            else if (scanAtt->type == eTendonAttachmentLeaf)
            {
                const auto& srcLeaf = static_cast<const omni::physics::parse::PhysxTendonAttachmentLeafDesc&>(*scanAtt);
                auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonAttachmentLeafDesc)();
                engineDesc->type          = eTendonAttachmentLeaf;
                engineDesc->gearing       = srcLeaf.gearing;
                engineDesc->localPos      = srcLeaf.localPos;
                engineDesc->linkKey       = linkKey;
                engineDesc->parentKey     = mAttachedStage.keyFor(scanned.source().sourceKeyToString(srcLeaf.parentKey));
                engineDesc->parentToken   = internStr(std::string(scanned.source().tokenToString(srcLeaf.parentToken)));
                engineDesc->instanceToken = instanceTokId;
                engineDesc->restLength    = srcLeaf.restLength;
                engineDesc->lowLimit      = srcLeaf.lowLimit;
                engineDesc->highLimit     = srcLeaf.highLimit;
                std::shared_ptr<PhysxTendonAttachmentLeafDesc> ptr(
                    engineDesc, [](PhysxTendonAttachmentLeafDesc* p) { ICE_FREE(p); });
                mTendonAttachmentMap[engineDesc->parentKey].push_back(ptr);
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
                if (tsSrc && tsSrc->exists(linkKey))
                {
                    const std::string base = "physxTendon:" + instanceStr + ":";
                    regScalarIfAuthored(linkKey, base + "gearing",     updateTendonAttachmentGearing);
                    regAnyIfTimeVarying(linkKey, base + "localPos",    updateTendonAttachmentLocalPos);
                    regScalarIfAuthored(linkKey, base + "restLength",  updateTendonAttachmentLeafRestLength);
                    regScalarIfAuthored(linkKey, base + "lowerLimit",  updateTendonAttachmentLeafLowLimit);
                    regScalarIfAuthored(linkKey, base + "upperLimit",  updateTendonAttachmentLeafHighLimit);
                }
            }
            else
            {
                auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonAttachmentDesc)();
                engineDesc->type          = scanAtt->type;
                engineDesc->gearing       = scanAtt->gearing;
                engineDesc->localPos      = scanAtt->localPos;
                engineDesc->linkKey       = linkKey;
                engineDesc->parentKey     = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanAtt->parentKey));
                engineDesc->parentToken   = internStr(std::string(scanned.source().tokenToString(scanAtt->parentToken)));
                engineDesc->instanceToken = instanceTokId;
                std::shared_ptr<PhysxTendonAttachmentDesc> ptr(
                    engineDesc, [](PhysxTendonAttachmentDesc* p) { ICE_FREE(p); });
                mTendonAttachmentMap[engineDesc->parentKey].push_back(ptr);
                registerTendonAttachmentChangeParams(mAttachedStage, instanceStr);

                // Time-sampled callbacks: gearing (scalar) + localPos
                // (any).  Matches legacy parseAttachment exactly.
                if (tsSrc && tsSrc->exists(linkKey))
                {
                    const std::string base = "physxTendon:" + instanceStr + ":";
                    regScalarIfAuthored(linkKey, base + "gearing",  updateTendonAttachmentGearing);
                    regAnyIfTimeVarying(linkKey, base + "localPos", updateTendonAttachmentLocalPos);
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
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanDesc->primKey));

            PhysxDeformableBodyDesc* desc = convert::convertScannedDeformableBody(scanned, i, mAttachedStage.getSourceUnits(), mAttachedStage);
            if (!desc)
                continue;

            // Source-backed type/schema dispatch (single-apply schemas via
            // hasSchema, prim types via isA). Attribute-value reads below
            // (restBendAnglesDefault, PhysxCollisionAPI offsets) stay direct.
            const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();
            const omni::physics::parse::KnownTokens& tok = mAttachedStage.getKnownTokens();

            // Collision-geom path validation — mirrors
            // PhysicsBody.cpp::parseDeformableBody:856-899.  parse-lib
            // takes the first collisionGeomPath verbatim; the consumer
            // does the type-of-prim sanity check.
            if (!desc->collisionMeshKey.valid())
            {
                CARB_LOG_WARN("No UsdPhysics.CollisionAPI found on deformable body prim or sub-tree, which is currently unsupported. "
                              "Parsing failed. Prim: %s", mAttachedStage.textFor(path));
                ICE_FREE(desc);
                continue;
            }
            const omni::physics::parse::ObjectKey collKey = desc->collisionMeshKey;
            if (desc->type == eVolumeDeformableBody)
            {
                // isTetMeshLike, not isA(UsdGeomTetMesh): ovstage reports a UsdGeomTetMesh as
                // plain "Mesh" (its populator has no TetMesh mapping). See
                // PhysXTools.h::isTetMeshLike.
                if (!src || !omni::physx::internal::isTetMeshLike(mAttachedStage, collKey))
                {
                    CARB_LOG_WARN("UsdPhysics.CollisionAPI on UsdGeomPointBased that are not UsdGeomTetMesh "
                                  "is currently not supported for volume deformables. Parsing failed. Prim: %s",
                                  mAttachedStage.textFor(path));
                    ICE_FREE(desc);
                    continue;
                }
            }
            else if (desc->type == eSurfaceDeformableBody)
            {
                if (!src || !src->isA(collKey, tok.meshType))
                {
                    CARB_LOG_WARN("UsdPhysics.CollisionAPI on UsdGeomPointBased that are not UsdGeomMesh "
                                  "is currently not supported for surface deformables. Parsing failed. Prim: %s",
                                  mAttachedStage.textFor(path));
                    ICE_FREE(desc);
                    continue;
                }
                if (desc->collisionMeshKey != desc->simMeshKey)
                {
                    CARB_LOG_WARN("UsdPhysics.CollisionAPI found on different prim than UsdPhysics.SurfaceDeformableSimAPI, "
                                  "which is currently not supported for surface deformables. Parsing failed. Prim: %s",
                                  mAttachedStage.textFor(path));
                    ICE_FREE(desc);
                    continue;
                }
            }

            // Surface restBendAnglesDefault — read from sim-mesh prim's
            // SurfaceDeformableSimAPI attribute (a token).  Routed
            // through the source; mirrors PhysicsBody.cpp:842-854.
            if (desc->type == eSurfaceDeformableBody && src)
            {
                const omni::physics::parse::ObjectKey simKey = desc->simMeshKey;
                if (src->hasSchema(simKey, tok.OmniPhysicsSurfaceDeformableSimAPI))
                {
                    const auto rbaTok = tok.omniphysicsRestBendAnglesDefault;
                    omni::physics::parse::TokenId val{};
                    if (src->getAttribute(simKey, rbaTok, val) && val.valid())
                        static_cast<PhysxSurfaceDeformableBodyDesc*>(desc)->restBendAnglesDefault = val;
                }
            }

            // PhysxCollisionAPI overlay on collision-mesh prim —
            // contactOffset / restOffset with time-sample callbacks.
            // Routed through the source; mirrors PhysicsBody.cpp:904-965.
            if (src)
            {
                const auto coTok = tok.physxCollisionContactOffset;
                const auto roTok = tok.physxCollisionRestOffset;
                float contactOffset = desc->contactOffset;
                float restOffset = desc->restOffset;
                if (src->hasAuthoredAttribute(collKey, coTok))
                {
                    float v = contactOffset;
                    src->getAttribute(collKey, coTok, v);
                    if (!std::isinf(v) && v >= 0.0f && v <= FLT_MAX)
                        contactOffset = v;
                    if (src->isAttributeTimeSampled(collKey, coTok))
                        mAttachedStage.registerTimeSampledAttribute(collKey, coTok, updateDeformableContactOffset);
                }
                if (src->hasAuthoredAttribute(collKey, roTok))
                {
                    float v = restOffset;
                    src->getAttribute(collKey, roTok, v);
                    if (!std::isinf(v) && v >= -FLT_MAX && v <= FLT_MAX)
                        restOffset = v;
                    if (src->isAttributeTimeSampled(collKey, roTok))
                        mAttachedStage.registerTimeSampledAttribute(collKey, roTok, updateDeformableRestOffset);
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
            omni::physics::parse::ObjectKey simMeshMaterial;
            if (src && desc->simMeshKey.valid())
            {
                const omni::physics::parse::ObjectKey matKey =
                    src->getMaterialBinding(desc->simMeshKey);
                if (matKey.valid())
                    simMeshMaterial = matKey;
            }

            // Translate sourceFilteredCollisions -> mFilteredPairs.  `fk` is minted by
            // `scanned`'s own scan-time source, so it needs the same opaque-identity
            // rekey as `path` above before joining mAttachedStage's namespace.
            for (const omni::physics::parse::ObjectKey fk : scanDesc->sourceFilteredCollisions)
            {
                const omni::physics::parse::ObjectKey fpKey =
                    mAttachedStage.keyFor(scanned.source().sourceKeyToString(fk));
                if (fpKey.valid())
                    mFilteredPairs.push_back(std::make_pair(path, fpKey));
            }

            // Resolve first simulationOwner → sceneId.  Legacy ignores
            // owners beyond the first (PhysicsBody.cpp:988).
            if (!scanDesc->sourceSimulationOwners.empty())
            {
                const omni::physics::parse::ObjectKey sceneKey =
                    mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanDesc->sourceSimulationOwners[0]));
                if (sceneKey.valid())
                {
                    const ObjectId entry = mAttachedStage.getObjectDatabase()->findEntry(sceneKey, eScene);
                    if (entry != kInvalidObjectId)
                        desc->sceneId = entry;
                }
            }

            // ObjectDatabase schema-flag tagging — duplicates the
            // legacy listener case (LoadStage.cpp:1043-1048) that ran
            // right after parseDeformableBody returned.
            const omni::physics::parse::ObjectKey simMeshKey = desc->simMeshKey;
            if (desc->type == eVolumeDeformableBody)
                mAttachedStage.getObjectDatabase()->addSchemaAPI(simMeshKey, SchemaAPIFlag::eVolumeDeformableSimAPI);
            else if (desc->type == eSurfaceDeformableBody)
                mAttachedStage.getObjectDatabase()->addSchemaAPI(simMeshKey, SchemaAPIFlag::eSurfaceDeformableSimAPI);
            mAttachedStage.getObjectDatabase()->addSchemaAPI(simMeshKey, SchemaAPIFlag::eDeformablePoseAPI);
            for (const omni::physics::parse::ObjectKey skinGeomKey : desc->skinGeomPaths)
                mAttachedStage.getObjectDatabase()->addSchemaAPI(skinGeomKey, SchemaAPIFlag::eDeformablePoseAPI);

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
                const omni::physics::parse::ObjectKey bodyKey = src ? path : omni::physics::parse::ObjectKey{};
                if (src && src->exists(bodyKey))
                {
                    if (src->hasSchema(bodyKey, tok.PhysxAutoDeformableBodyAPI))
                    {
                        mAttachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eAutoDeformableBodyAPI);
                        if (src->hasSchema(bodyKey, tok.PhysxAutoDeformableMeshSimplificationAPI))
                            mAttachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eAutoDeformableMeshSimplificationAPI);
                        if (desc->type == eVolumeDeformableBody &&
                            src->hasSchema(bodyKey, tok.PhysxAutoDeformableHexahedralMeshAPI))
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
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanArt->articulationPrim));

            if (mNumScenes > 1)
            {
                // Inlined checkArticulatonBodySimulationOwners — verify
                // all articulatedBodies share the same simulationOwner
                // and that the owner resolves to a registered scene.
                bool firstBody = true;
                omni::physics::parse::ObjectKey owner;
                bool ownerConsistent = true;
                for (const auto& bk : scanArt->articulatedBodies)
                {
                    if (!bk.valid()) continue;
                    const omni::physics::parse::ObjectKey bodyKey =
                        mAttachedStage.keyFor(scanned.source().sourceKeyToString(bk));
                    // getRigidBodySimulationOwner is ObjectKey-native (ADR-0019).
                    const omni::physics::parse::ObjectKey bodyOwner =
                        getRigidBodySimulationOwner(mAttachedStage, bodyKey);
                    if (firstBody) { owner = bodyOwner; firstBody = false; }
                    else if (owner != bodyOwner)
                    {
                        CARB_LOG_ERROR("Articulation contains bodies with different simulation owners. Articulation: %s",
                                       mAttachedStage.textFor(path));
                        ownerConsistent = false;
                        break;
                    }
                }
                if (!ownerConsistent)
                    continue;
                if (owner.valid() &&
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
                return k.valid() ? mAttachedStage.keyFor(scanned.source().sourceKeyToString(k)) : omni::physics::parse::ObjectKey{};
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
                const omni::physics::parse::ObjectKey fpKey = rekey(fk);
                if (fpKey.valid())
                    mFilteredPairs.push_back(std::make_pair(desc->articulationPrim, fpKey));
            }

            mArticulationMap[desc->articulationPrim].push_back(desc);
        }

        // ----- Joints ---------------------------------------------
        // Mirrors legacy reportObjectDesc(eJoint*) + parseJoint, plus
        // the multi-scene owner check from
        // PhysicsBody.cpp::checkJointBodySimulationOwners and the
        // body-transform-equality check legacy applies per joint type
        // (parse-lib can't compute that — it has no xformCache).
        // USD keeps the per-joint tendon-axis check; ovstage skips it when no prim applies the API.
        bool stageHasTendons = true;
        // Under ovstage the scan source and the attach source share one intern table, so a
        // scan key canonicalizes directly via canonicalKey() (no SdfPath round-trip). USD
        // attaches keep keyFor(pathFor()): their two UsdSource keyspaces differ.
        omni::physics::ovstage::OvstageSource* ovAttachSrc =
            dynamic_cast<omni::physics::ovstage::OvstageSource*>(mAttachedStage.getSource());
        if (ovAttachSrc)
        {
            static const char* const kTendonAxisApi[] = { "PhysxTendonAxisAPI" };
            stageHasTendons = ovAttachSrc->stageHasAnySchema(kTendonAxisApi, 1).value_or(true);
        }
        for (auto& scanJoint : scanned.joints)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:joint");

            // Re-key a scan-source key into the AttachedStage source (ovstage: canonicalKey;
            // USD: path round-trip).
            auto rekey = [&](omni::physics::parse::ObjectKey k) -> omni::physics::parse::ObjectKey {
                if (!k.valid())
                    return {};
                return ovAttachSrc ? ovAttachSrc->canonicalKey(k)
                                   : mAttachedStage.keyFor(scanned.source().sourceKeyToString(k));
            };

            if (mNumScenes > 1)
            {
                // getRigidBodySimulationOwner is ObjectKey-native (ADR-0019).
                const omni::physics::parse::ObjectKey simOwner0 =
                    getRigidBodySimulationOwner(mAttachedStage, rekey(scanJoint->body0));
                const omni::physics::parse::ObjectKey simOwner1 =
                    getRigidBodySimulationOwner(mAttachedStage, rekey(scanJoint->body1));
                if (scanJoint->body0.valid() && scanJoint->body1.valid() && simOwner0 != simOwner1)
                {
                    CARB_LOG_ERROR("Cannot create joint between bodies that belong to different owners. Joint: %s",
                                   mAttachedStage.textFor(rekey(scanJoint->jointPrimKey)));
                    continue;
                }
                const omni::physics::parse::ObjectKey owner = simOwner0.valid() ? simOwner0 : simOwner1;
                if (owner.valid() &&
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
                // hinge keys are parse-time ScannedStage keys -> re-key into
                // mAttachedStage's namespace (ADR-0019 increment 7).
                gearDesc->hingePrimPath0 = rekey(scanGear->hingePrimPath0);
                gearDesc->hingePrimPath1 = rekey(scanGear->hingePrimPath1);
                desc = gearDesc;
            }
            else if (scanJoint->type == eJointRackAndPinion)
            {
                auto* rackDesc = ICE_PLACEMENT_NEW(RackPhysxJointDesc)();
                static_cast<PhysxJointDesc&>(*rackDesc) = *scanJoint;
                rackDesc->type = eJointRackAndPinion;
                const auto* scanRack = static_cast<const omni::physics::parse::RackPhysxJointDesc*>(&*scanJoint);
                rackDesc->ratio = scanRack->ratio;
                rackDesc->hingePrimKey = rekey(scanRack->hingePrimKey);
                rackDesc->prismaticPrimKey = rekey(scanRack->prismaticPrimKey);
                desc = rackDesc;
            }
            else if (scanJoint->type == eJointCustom)
            {
                // Custom joints are keyed by the prim's raw type name (not a schema
                // hierarchy), so read it through the source (no UsdPrim).
                const omni::physics::parse::IPhysicsSource* csrc = mAttachedStage.getSource();
                const omni::physics::parse::ObjectKey jointKey = rekey(scanJoint->jointPrimKey);
                if (csrc && csrc->exists(jointKey))
                {
                    const omni::physics::parse::TokenId typeNameId = csrc->getTypeName(jointKey);
                    // mCustomJointTypeMap is plain-string-keyed (PhysXCustomJoint.h); no
                    // TfToken interning available/needed here.
                    const std::string typeName(csrc->tokenToString(typeNameId));
                    const auto& customMap = OmniPhysX::getInstance().getCustomJointManager().getCustomJointTypeMap();
                    if (customMap.find(typeName) != customMap.end())
                    {
                        auto* customDesc = ICE_PLACEMENT_NEW(CustomPhysxJointDesc)();
                        static_cast<PhysxJointDesc&>(*customDesc) = *scanJoint;
                        customDesc->type = eJointCustom;
                        customDesc->customJointToken = typeNameId;
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

            // Re-key body0/body1/rel0/rel1/jointPrimKey into the AttachedStage source
            // (REQ-PARSE-CONSUMER-001 AC-5).
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
            // Body-transform-equality diagnostic.
            if (checkPos || checkRot)
            {
                // Body world transforms via the source (EarliestTime, matching
                // the replaced mXfCache); validity via the source mirrors the
                // prior prim.IsValid() gate (no UsdPrim).
                // desc->body0/body1 are already the rekeyed AttachedStage keys.
                const omni::physics::parse::IPhysicsSource* jbSrc = mAttachedStage.getSource();
                const bool body0Valid = jbSrc && jbSrc->exists(desc->body0);
                const bool body1Valid = jbSrc && jbSrc->exists(desc->body1);
                // World matrices stay PhysX-typed into primutils::isBodyTransformEqual; an invalid
                // body contributes identity (ignored there).
                const ::physx::PxMat44d body0World =
                    body0Valid ? internal::getWorldTransform(mAttachedStage, desc->body0) :
                                 ::physx::PxMat44d(::physx::PxIdentity);
                const ::physx::PxMat44d body1World =
                    body1Valid ? internal::getWorldTransform(mAttachedStage, desc->body1) :
                                 ::physx::PxMat44d(::physx::PxIdentity);
                desc->validBodyTransformations = isJointBodyTransformEqual(
                    body0World, body0Valid, body1World, body1Valid,
                    toPhysX(desc->localPose0Position), toPhysXQuat(desc->localPose0Orientation),
                    toPhysX(desc->localPose1Position), toPhysXQuat(desc->localPose1Orientation),
                    OmniPhysX::getInstance().getCachedSettings().jointBodyTransformCheckTolerance,
                    checkPos, checkRot, tmAxis);
            }

            JointDescAndPath jd;
            jd.path = desc->jointPrimKey;
            jd.desc = desc;
            jd.articulationJoint = false;
            jd.index = uint32_t(mJointVector.size());
            mJointPathIndexMap[desc->jointPrimKey] = mJointVector.size();
            mJointVector.push_back(jd);

            // Mimic + tendon parsing moved into the native walker
            // (6F.1 mimic, 6F.3 fixed tendons); the unsupported-joint
            // tendon-axis warning is preserved here since only this loop
            // has the joint type bucketing in front of it. Source-routed (no UsdPrim).
            const omni::physics::parse::IPhysicsSource* jsrc = mAttachedStage.getSource();
            // Only the tendon-axis warning needs a token; read it from the attach-scoped cache.
            const bool jointTypeTakesTendonWarn = desc->type == eJointFixed ||
                desc->type == eJointSpherical || desc->type == eJointDistance;
            // desc->jointPrimKey is already the rekeyed AttachedStage key.
            if (stageHasTendons && jointTypeTakesTendonWarn && jsrc && jsrc->exists(desc->jointPrimKey))
            {
                switch (desc->type)
                {
                case eJointFixed:
                case eJointSpherical:
                case eJointDistance:
                {
                    if (jsrc->hasSchema(desc->jointPrimKey,
                                        mAttachedStage.getKnownTokens().physxTendonAxisAPI))
                    {
                        CARB_LOG_WARN("A Tendon Axis API was applied to an unsupported joint type at %s!",
                                      mAttachedStage.textFor(desc->jointPrimKey));
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
            // mimicJointKey/referenceJointKey are minted by the SCAN's own
            // source and must be re-keyed into mAttachedStage's namespace
            // (ADR-0019 increment 7; mirrors the tire-friction-table and
            // articulation `rekey` pattern above) -- MimicJoint.cpp's
            // createMimicJoint reads them against mAttachedStage's own
            // ObjectDatabase.
            engineDesc.mimicJointKey        = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanMimic->mimicJointKey));
            engineDesc.mimicJointAxis       = scanMimic->mimicJointAxis;
            engineDesc.referenceJointKey    = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanMimic->referenceJointKey));
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
        // Composite key is jointKey-string + ":" + instance-name-string, not a
        // TfToken: both halves are only ever produced/consumed as strings here
        // (tokenToString / SdfPath::GetString) for the axisByInstance lookup;
        // mTendonAxisMap itself is ObjectKey-keyed (LoadTools.h retype), keyed
        // below by the already-rekeyed engineDesc->link0/link1/jointKey.
        std::unordered_map<std::string, std::shared_ptr<PhysxTendonAxisDesc>>
            axisByInstance; // jointKey:instance → engine axis ptr (for rootAxis wiring)
        for (const auto& scanAxis : scanned.fixedTendonAxes)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:fixedTendonAxis");
            const std::string instanceTokStr = std::string(scanned.source().tokenToString(scanAxis->instanceToken));
            // Both this loop and the fixedTendons loop below derive the composite
            // axisByInstance key from sourceKeyToString, so they always agree.
            const std::string jointKeyStr = std::string(scanned.source().sourceKeyToString(scanAxis->jointKey));
            const omni::physics::parse::ObjectKey jointKey = mAttachedStage.keyFor(jointKeyStr);
            const omni::physics::parse::ObjectKey link0 =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanAxis->link0));
            const omni::physics::parse::ObjectKey link1 =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanAxis->link1));

            auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonAxisDesc)();
            engineDesc->type              = scanAxis->type;
            // instanceToken/jointKey/link0/link1 are minted by the SCAN's
            // own source and must be re-interned/re-keyed into
            // mAttachedStage's namespace (ADR-0019 increment 7).
            engineDesc->instanceToken     = mAttachedStage.getSource() ? mAttachedStage.getSource()->internToken(instanceTokStr) : omni::physics::parse::TokenId{};
            engineDesc->jointKey          = jointKey;
            engineDesc->link0             = link0;
            engineDesc->link1             = link1;
            engineDesc->gearings          = scanAxis->gearings;
            engineDesc->forceCoefficients = scanAxis->forceCoefficients;
            engineDesc->axes              = scanAxis->axes;
            engineDesc->parentAxisId      = kInvalidObjectId;
            engineDesc->wasVisited        = false;

            std::shared_ptr<PhysxTendonAxisDesc> ptr(
                engineDesc, [](PhysxTendonAxisDesc* p) { ICE_FREE(p); });

            mTendonAxisMap[engineDesc->link0].push_back(ptr);
            mTendonAxisMap[engineDesc->link1].push_back(ptr);
            mTendonAxisMap[engineDesc->jointKey].push_back(ptr);

            // The "axis by instance on this joint" lookup is used below
            // to wire PhysxTendonFixedDesc::rootAxis.  Use a composite
            // key of jointKey + ":" + instance.
            const std::string composite = jointKeyStr + ":" + instanceTokStr;
            axisByInstance.emplace(composite, ptr);

            registerTendonAxisChangeParam(mAttachedStage, instanceTokStr);

            // Time-sampled callback for the axis gearing.  Matches legacy
            // parseAxes: ValueMightBeTimeVarying only (no HasAuthoredValue
            // gate — legacy registers via `gearingAttr.ValueMightBeTimeVarying()`).
            if (tsSrc && tsSrc->exists(jointKey))
            {
                const std::string base = "physxTendon:" + instanceTokStr + ":";
                regAnyIfTimeVarying(jointKey, base + "gearing", updateTendonAxisSingleGearing);
            }
        }

        for (const auto& scanTendon : scanned.fixedTendons)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:fixedTendon");
            const std::string instanceTokStr = std::string(scanned.source().tokenToString(scanTendon->instanceToken));
            const std::string jointKeyStr = std::string(scanned.source().sourceKeyToString(scanTendon->jointKey));
            const omni::physics::parse::ObjectKey jointKey = mAttachedStage.keyFor(jointKeyStr);

            auto* engineDesc = ICE_PLACEMENT_NEW(PhysxTendonFixedDesc)();
            engineDesc->type            = eTendonFixed;
            // instanceToken/jointKey are minted by the SCAN's own source and
            // must be re-interned/re-keyed into mAttachedStage's namespace
            // (ADR-0019 increment 7).
            engineDesc->instanceToken   = mAttachedStage.getSource() ? mAttachedStage.getSource()->internToken(instanceTokStr) : omni::physics::parse::TokenId{};
            engineDesc->jointKey        = jointKey;
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
            const std::string composite = jointKeyStr + ":" + instanceTokStr;
            auto axisIt = axisByInstance.find(composite);
            if (axisIt != axisByInstance.end())
                engineDesc->rootAxis = axisIt->second.get();

            std::shared_ptr<PhysxTendonFixedDesc> ptr(
                engineDesc, [](PhysxTendonFixedDesc* p) { ICE_FREE(p); });
            mFixedTendons.push_back(ptr);

            registerFixedTendonChangeParams(mAttachedStage, instanceTokStr);

            // Time-sampled callbacks for the fixed tendon root attributes.
            // Matches legacy parseFixedTendon's getAttribute chain
            // (HasAuthoredValue + ValueMightBeTimeVarying).
            if (tsSrc && tsSrc->exists(jointKey))
            {
                const std::string base = "physxTendon:" + instanceTokStr + ":";
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
        // Per-prim attachment desc; src0/src1/primKey are minted by
        // `scanned`'s own throwaway, parse-time source (ADR-0004 key-space
        // invariant) and must be re-keyed into mAttachedStage's persistent
        // namespace via a path round-trip -- every downstream consumer
        // (InternalDeformableAttachment.cpp) resolves them through
        // mAttachedStage. Mirrors legacy reportObjectDesc(eAttachment*) case
        // + parseDeformableAttachment.
        for (const auto& scanAtt : scanned.attachments)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:deformableAttachment");
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanAtt->primKey));
            auto* outDesc = ICE_PLACEMENT_NEW(PhysxDeformableAttachmentDesc)();
            outDesc->type      = scanAtt->type;
            outDesc->primKey   = path;
            outDesc->enabled   = scanAtt->enabled;
            outDesc->src0      = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanAtt->src0));
            outDesc->src1      = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanAtt->src1));
            outDesc->stiffness = scanAtt->stiffness;
            outDesc->damping   = scanAtt->damping;

            DeformableAttachmentDescAndPath entry;
            entry.path = outDesc->primKey;
            entry.desc = outDesc;
            mDeformableAttachmentVector.push_back(entry);
        }

        // ----- DeformableCollisionFilters --------------------------
        for (const auto& scanFilt : scanned.deformableCollisionFilters)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:deformableCollisionFilter");
            const omni::physics::parse::ObjectKey path =
                mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanFilt->primKey));
            auto* outDesc = ICE_PLACEMENT_NEW(PhysxDeformableCollisionFilterDesc)();
            outDesc->primKey = path;
            outDesc->enabled = scanFilt->enabled;
            outDesc->src0    = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanFilt->src0));
            outDesc->src1    = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanFilt->src1));

            DeformableCollisionFilterDescAndPath entry;
            entry.path = outDesc->primKey;
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
                const omni::physics::parse::ObjectKey path =
                    mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanCct->primKey));

                if (scanCct->sourceSimulationOwner.valid())
                {
                    const omni::physics::parse::ObjectKey ownerKey =
                        mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanCct->sourceSimulationOwner));
                    const ObjectId sceneId = db.findEntry(ownerKey, eScene);
                    if (sceneId == kInvalidObjectId)
                    {
                        CARB_LOG_ERROR("parseCct: Failed to find physics simulation owner \"%s\".",
                                       mAttachedStage.textFor(ownerKey));
                        continue;
                    }
                    scanCct->sceneId = sceneId;
                }
                createObject(mAttachedStage, path, scanCct.get(), /*deleteDesc=*/false);
            }
        }

        // ----- Tire friction tables (7A.1) -------------------------
        // Walker emitted parse-lib TireFrictionTableDescs into
        // scanned.tireFrictionTables. The engine descriptor is now the
        // same aliased type (ADR-0019 increment 7), so most fields are a
        // plain copy -- except `key`/`materialPaths`, which are minted by
        // the SCAN's own source and must be re-keyed into mAttachedStage's
        // namespace via a path round-trip before being stashed for the
        // post-load creation block (which still owns ObjectDatabase
        // material-id resolution + engine createObject, consumed later via
        // UsdLoad::getActiveAttachedStage(), i.e. this same mAttachedStage).
        // Mirrors the articulation `rekey` lambda above.
        // See REQ-PARSE-VEH-TIREFRICTION-001 AC-5.
        for (const auto& scanTft : scanned.tireFrictionTables)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:tireFrictionTable");
            auto* engineDesc = ICE_PLACEMENT_NEW(TireFrictionTableDesc)();
            engineDesc->key                  = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanTft->key));
            engineDesc->defaultFrictionValue = scanTft->defaultFrictionValue;
            engineDesc->frictionValues       = scanTft->frictionValues;
            // materialIds is resolved below, post-load, once all materials exist.
            engineDesc->materialPaths.reserve(scanTft->materialPaths.size());
            for (const omni::physics::parse::ObjectKey matKey : scanTft->materialPaths)
                engineDesc->materialPaths.push_back(mAttachedStage.keyFor(scanned.source().sourceKeyToString(matKey)));
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
            engineDesc.sceneKey           = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanCtx->sceneKey));
            engineDesc.vehicleUpdateMode  = static_cast<VehicleUpdateMode>(scanCtx->vehicleUpdateMode);
            engineDesc.upAxis             = scanCtx->upAxis;
            engineDesc.forwardAxis        = scanCtx->forwardAxis;
            engineDesc.verticalAxis       = static_cast<VehicleContextDesc::AxisDir>(scanCtx->verticalAxis);
            engineDesc.longitudinalAxis   = static_cast<VehicleContextDesc::AxisDir>(scanCtx->longitudinalAxis);
            mVehicleContextDescList.push_back(engineDesc);
        }

        // ----- Vehicle shareable components (7A.2) through ----------
        // ----- Vehicles (7A.13) -------------------------------------
        // Walker emitted parse-lib WheelDesc / TireDesc / SuspensionDesc
        // into scanned.vehicleWheels / vehicleTires / vehicleSuspensions
        // (ObjectKey paths).  Translate to engine descriptors and
        // pre-populate the stage-level VehicleComponentTracker.  The
        // legacy parseVehicle path looks up entries via ObjectKey and
        // short-circuits when found, so per-component legacy parsers
        // (parseWheel/parseTire/parseSuspension) are unreachable during
        // vehicle parsing.
        //
        // Scan-time ObjectKeys are re-keyed into mAttachedStage's namespace: scanned and
        // mAttachedStage are different intern tables even when scanning the full stage.
        for (const auto& scanWheel : scanned.vehicleWheels)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleWheel");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanWheel->key));
            if (mVehicleComponentTracker.mWheels.find(key) != mVehicleComponentTracker.mWheels.end())
                continue;
            WheelDesc* engineDesc = ICE_PLACEMENT_NEW(WheelDesc)();
            engineDesc->key                   = key;
            engineDesc->radius                = scanWheel->radius;
            engineDesc->width                 = scanWheel->width;
            engineDesc->mass                  = scanWheel->mass;
            engineDesc->moi                   = scanWheel->moi;
            engineDesc->dampingRate           = scanWheel->dampingRate;
            engineDesc->maxBrakeTorque        = scanWheel->maxBrakeTorque;
            engineDesc->maxHandBrakeTorque    = scanWheel->maxHandBrakeTorque;
            engineDesc->maxSteerAngle         = scanWheel->maxSteerAngle;
            engineDesc->toeAngle              = scanWheel->toeAngle;
            mVehicleComponentTracker.mWheels.insert({ key, engineDesc });
        }
        for (const auto& scanTire : scanned.vehicleTires)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleTire");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanTire->key));
            if (mVehicleComponentTracker.mTires.find(key) != mVehicleComponentTracker.mTires.end())
                continue;
            TireDesc* engineDesc = ICE_PLACEMENT_NEW(TireDesc)();
            engineDesc->key                                 = key;
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
            engineDesc->frictionTableKey                    = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanTire->frictionTableKey));
            engineDesc->restLoad                            = scanTire->restLoad;
            mVehicleComponentTracker.mTires.insert({ key, engineDesc });
        }
        for (const auto& scanSusp : scanned.vehicleSuspensions)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleSuspension");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanSusp->key));
            if (mVehicleComponentTracker.mSuspensions.find(key) != mVehicleComponentTracker.mSuspensions.end())
                continue;
            SuspensionDesc* engineDesc = ICE_PLACEMENT_NEW(SuspensionDesc)();
            engineDesc->key                     = key;
            engineDesc->springStrength          = scanSusp->springStrength;
            engineDesc->springDamperRate        = scanSusp->springDamperRate;
            engineDesc->travelDistance          = scanSusp->travelDistance;
            engineDesc->maxCompression          = scanSusp->maxCompression;
            engineDesc->maxDroop                = scanSusp->maxDroop;
            engineDesc->camberAtRest            = scanSusp->camberAtRest;
            engineDesc->camberAtMaxCompression  = scanSusp->camberAtMaxCompression;
            engineDesc->camberAtMaxDroop        = scanSusp->camberAtMaxDroop;
            engineDesc->sprungMass              = scanSusp->sprungMass;
            mVehicleComponentTracker.mSuspensions.insert({ key, engineDesc });
        }

        // ----- Vehicle drivetrain components (7A.3) ----------------
        // Engine / Gears / Clutch.  AutoGearBox stays legacy until 7A.4
        // because parseAutoGearBox needs forwardGearCount from the
        // parent DriveStandard.gears rel target.
        for (const auto& scanEng : scanned.vehicleEngines)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleEngine");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanEng->key));
            if (mVehicleComponentTracker.mEngines.find(key) != mVehicleComponentTracker.mEngines.end())
                continue;
            EngineDesc* engineDesc = ICE_PLACEMENT_NEW(EngineDesc)();
            engineDesc->key                                 = key;
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
            mVehicleComponentTracker.mEngines.insert({ key, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleGears.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleGears");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleGearsPaths[i]));
            if (mVehicleComponentTracker.mGears.find(key) != mVehicleComponentTracker.mGears.end())
                continue;
            GearsDesc* engineDesc = ICE_PLACEMENT_NEW(GearsDesc)();
            engineDesc->ratios     = scanned.vehicleGears[i]->ratios;
            engineDesc->ratioScale = scanned.vehicleGears[i]->ratioScale;
            engineDesc->switchTime = scanned.vehicleGears[i]->switchTime;
            mVehicleComponentTracker.mGears.insert({ key, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleClutches.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleClutch");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleClutchPaths[i]));
            if (mVehicleComponentTracker.mClutches.find(key) != mVehicleComponentTracker.mClutches.end())
                continue;
            ClutchDesc* engineDesc = ICE_PLACEMENT_NEW(ClutchDesc)();
            engineDesc->strength = scanned.vehicleClutches[i]->strength;
            mVehicleComponentTracker.mClutches.insert({ key, engineDesc });
        }

        // ----- Vehicle NonlinearCmdResponse (7A.6) -----------------
        // Multi-apply per command instance.  Build a side-table keyed
        // by (ownerPath, instanceToken) so the Drive / Steering /
        // Brakes loops below can wire the nonlinearCmdResponse
        // pointer.  Engine-side descriptors land in the legacy
        // mNonlinearCmdResponses vector for cleanup.  The instance half of
        // the key is a TokenId, not a TfToken: every producer/consumer here
        // (scan vectors, the "drive"/"steer" literals) already lives in the
        // scan source's own TokenId namespace, so comparing TfTokens instead
        // was a pure round-trip through pxr with no functional purpose.
        struct OwnerInstanceKey
        {
            omni::physics::parse::ObjectKey owner;
            omni::physics::parse::TokenId inst;
            bool operator==(const OwnerInstanceKey& o) const { return owner == o.owner && inst == o.inst; }
        };
        struct OwnerInstanceKeyHash
        {
            size_t operator()(const OwnerInstanceKey& k) const
            {
                return omni::physics::parse::ObjectKey::Hash()(k.owner) ^ (omni::physics::parse::TokenId::Hash()(k.inst) << 1);
            }
        };
        std::unordered_map<OwnerInstanceKey, NonlinearCmdResponseDesc*, OwnerInstanceKeyHash> ncrByOwnerAndInstance;
        for (size_t i = 0; i < scanned.vehicleNonlinearCmdResponses.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleNonlinearCmdResponse");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleNonlinearCmdResponsePaths[i]));
            const omni::physics::parse::TokenId inst = scanned.vehicleNonlinearCmdResponseInstanceTokens[i];
            const auto& scanNcr = scanned.vehicleNonlinearCmdResponses[i];

            NonlinearCmdResponseDesc* engineDesc = ICE_PLACEMENT_NEW(NonlinearCmdResponseDesc)();
            engineDesc->commandValues                = scanNcr->commandValues;
            engineDesc->speedResponsesPerCommandValue = scanNcr->speedResponsesPerCommandValue;
            engineDesc->speedResponses               = scanNcr->speedResponses;

            mVehicleComponentTracker.mNonlinearCmdResponses.push_back(engineDesc);
            ncrByOwnerAndInstance.insert({ { key, inst }, engineDesc });
        }
        auto findNcr = [&ncrByOwnerAndInstance](omni::physics::parse::ObjectKey owner, omni::physics::parse::TokenId inst) -> NonlinearCmdResponseDesc* {
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
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanDrive->key));
            if (mVehicleComponentTracker.mDrivesBasic.find(key) != mVehicleComponentTracker.mDrivesBasic.end())
                continue;
            DriveBasicDesc* engineDesc = ICE_PLACEMENT_NEW(DriveBasicDesc)();
            engineDesc->key                 = key;
            engineDesc->peakTorque          = scanDrive->peakTorque;
            // "drive" is the fixed multi-apply-schema instance name PhysxSchemaTokens->drive
            // names; interned straight off this scan's own source (whose tokens `key`/`inst`
            // above are already keyed by) rather than a generated pxr token-table constant.
            engineDesc->nonlinearCmdResponse =
                findNcr(key, scanned.source().internToken("drive"));  // 7A.6
            mVehicleComponentTracker.mDrivesBasic.insert({ key, engineDesc });
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
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleDrivesStandardPaths[i]));
            if (mVehicleComponentTracker.mDrivesStandard.find(key) != mVehicleComponentTracker.mDrivesStandard.end())
                continue;
            const auto& refs = scanned.vehicleDrivesStandardCrossRefs[i];

            EngineDesc* engine = nullptr;
            GearsDesc*  gears  = nullptr;
            ClutchDesc* clutch = nullptr;
            AutoGearBoxDesc* autoGearBox = nullptr;

            if (refs.engineKey.valid())
            {
                auto it = mVehicleComponentTracker.mEngines.find(mAttachedStage.keyFor(scanned.source().sourceKeyToString(refs.engineKey)));
                if (it != mVehicleComponentTracker.mEngines.end()) engine = it->second;
            }
            if (refs.gearsKey.valid())
            {
                auto it = mVehicleComponentTracker.mGears.find(mAttachedStage.keyFor(scanned.source().sourceKeyToString(refs.gearsKey)));
                if (it != mVehicleComponentTracker.mGears.end()) gears = it->second;
            }
            if (refs.clutchKey.valid())
            {
                auto it = mVehicleComponentTracker.mClutches.find(mAttachedStage.keyFor(scanned.source().sourceKeyToString(refs.clutchKey)));
                if (it != mVehicleComponentTracker.mClutches.end()) clutch = it->second;
            }
            if (refs.autoGearBoxKey.valid())
            {
                auto it = mVehicleComponentTracker.mAutoGearBoxes.find(mAttachedStage.keyFor(scanned.source().sourceKeyToString(refs.autoGearBoxKey)));
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
            mVehicleComponentTracker.mDrivesStandard.insert({ key, engineDesc });
        }
        // ----- Vehicle SuspensionCompliance (7A.7) ------------------
        // SuspensionCompliance is per-attachment-prim.  Pre-populate
        // an ObjectKey-keyed side-table and push to the legacy
        // mSuspensionCompliances vector for cleanup lifetime.  Legacy
        // parseWheelAttachment consults the side-table when the
        // PhysxVehicleSuspensionComplianceAPI is applied on the
        // attachment prim; short-circuits on hit.
        for (size_t i = 0; i < scanned.vehicleSuspensionCompliances.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleSuspensionCompliance");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleSuspensionCompliancePaths[i]));
            if (mVehicleComponentTracker.mSuspensionComplianceByPath.find(key) !=
                mVehicleComponentTracker.mSuspensionComplianceByPath.end())
                continue;
            const auto& scanSc = scanned.vehicleSuspensionCompliances[i];
            SuspensionComplianceDesc* engineDesc = ICE_PLACEMENT_NEW(SuspensionComplianceDesc)();
            engineDesc->wheelToeAngleList           = scanSc->wheelToeAngleList;
            engineDesc->wheelCamberAngleList        = scanSc->wheelCamberAngleList;
            engineDesc->suspensionForceAppPointList = scanSc->suspensionForceAppPointList;
            engineDesc->tireForceAppPointList       = scanSc->tireForceAppPointList;
            mVehicleComponentTracker.mSuspensionCompliances.push_back(engineDesc);
            mVehicleComponentTracker.mSuspensionComplianceByPath.insert({ key, engineDesc });
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
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleWheelAttachments[i]->key));
            if (mVehicleComponentTracker.mWheelAttachmentByPath.find(key) !=
                mVehicleComponentTracker.mWheelAttachmentByPath.end())
                continue;
            const auto& info = scanned.vehicleWheelAttachmentInfos[i];

            WheelDesc* wheel = nullptr;
            TireDesc* tire = nullptr;
            SuspensionDesc* suspension = nullptr;
            if (info.wheelKey.valid())
            {
                auto it = mVehicleComponentTracker.mWheels.find(mAttachedStage.keyFor(scanned.source().sourceKeyToString(info.wheelKey)));
                if (it != mVehicleComponentTracker.mWheels.end()) wheel = it->second;
            }
            if (info.tireKey.valid())
            {
                auto it = mVehicleComponentTracker.mTires.find(mAttachedStage.keyFor(scanned.source().sourceKeyToString(info.tireKey)));
                if (it != mVehicleComponentTracker.mTires.end()) tire = it->second;
            }
            if (info.suspensionKey.valid())
            {
                auto it = mVehicleComponentTracker.mSuspensions.find(mAttachedStage.keyFor(scanned.source().sourceKeyToString(info.suspensionKey)));
                if (it != mVehicleComponentTracker.mSuspensions.end()) suspension = it->second;
            }
            if (!wheel || !tire || !suspension)
            {
                // Required component ref unresolved: leave it out of
                // mWheelAttachmentByPath. The owners list still records it, so
                // parseVehicle counts it and marks the vehicle invalid.
                CARB_LOG_ERROR("Usd Physics: wheel attachment \"%s\": a required %s%s%s reference is missing or does "
                               "not point to a prim with the matching vehicle component API applied.",
                               mAttachedStage.textFor(key), wheel ? "" : "wheel ", tire ? "" : "tire ", suspension ? "" : "suspension ");
                continue;
            }

            const auto& scanWa = scanned.vehicleWheelAttachments[i];
            WheelAttachmentDesc* engineDesc = ICE_PLACEMENT_NEW(WheelAttachmentDesc)();
            // key/collisionGroupKey/shapeKey are minted by the SCAN's own source
            // and must be re-keyed into mAttachedStage's namespace via the
            // sourceKeyToString + keyFor round-trip (ADR-0019 increment 7; mirrors
            // the tire-friction-table and articulation `rekey` pattern above).
            engineDesc->key                          = key;
            engineDesc->id                           = kInvalidObjectId;
            engineDesc->state                        = scanWa->state;
            engineDesc->wheel                        = wheel;
            engineDesc->wheelId                      = kInvalidObjectId;
            engineDesc->tire                         = tire;
            engineDesc->tireId                       = kInvalidObjectId;
            engineDesc->suspension                   = suspension;
            engineDesc->suspensionId                 = kInvalidObjectId;
            // SuspensionCompliance: applied alongside on the same prim;
            // look up by attachment key.
            {
                auto it = mVehicleComponentTracker.mSuspensionComplianceByPath.find(key);
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
            engineDesc->collisionGroupKey            = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanWa->collisionGroupKey));
            engineDesc->shapeKey                     = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanWa->shapeKey));
            engineDesc->shapeId                       = kInvalidObjectId;

            mVehicleComponentTracker.mWheelAttachmentsOwned.push_back(engineDesc);
            mVehicleComponentTracker.mWheelAttachmentByPath.insert({ key, engineDesc });
        }

        // Group EVERY scanned wheel attachment (valid or malformed) by its
        // owning vehicle, so parseVehicle can enumerate a vehicle's attachments
        // without a USD descendant walk — and still reject the vehicle when one
        // is malformed (present here but absent from mWheelAttachmentByPath).
        for (const auto& owner : scanned.vehicleWheelAttachmentOwners)
        {
            if (!owner.first.valid())
                continue;  // attachment with no vehicle ancestor (malformed) — skip
            const omni::physics::parse::ObjectKey ownerKey = mAttachedStage.keyFor(scanned.source().sourceKeyToString(owner.first));
            const omni::physics::parse::ObjectKey attachmentKey = mAttachedStage.keyFor(scanned.source().sourceKeyToString(owner.second));
            mVehicleComponentTracker.mVehicleWheelAttachments[ownerKey].push_back(attachmentKey);
        }

        for (size_t i = 0; i < scanned.vehicleMultiWheelDifferentials.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleMultiWheelDifferential");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleMultiWheelDifferentialPaths[i]));
            if (mVehicleComponentTracker.mDifferentialsByPath.find(key) != mVehicleComponentTracker.mDifferentialsByPath.end())
                continue;
            const auto& scanDiff = scanned.vehicleMultiWheelDifferentials[i];
            MultiWheelDifferentialDesc* engineDesc = ICE_PLACEMENT_NEW(MultiWheelDifferentialDesc)();
            engineDesc->wheels                  = scanDiff->wheels;
            engineDesc->torqueRatios            = scanDiff->torqueRatios;
            engineDesc->averageWheelSpeedRatios = scanDiff->averageWheelSpeedRatios;
            mVehicleComponentTracker.mMultiWheelDifferentials.push_back(engineDesc);
            mVehicleComponentTracker.mDifferentialsByPath.insert({ key, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleTankDifferentials.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleTankDifferential");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleTankDifferentialPaths[i]));
            if (mVehicleComponentTracker.mDifferentialsByPath.find(key) != mVehicleComponentTracker.mDifferentialsByPath.end())
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
            mVehicleComponentTracker.mDifferentialsByPath.insert({ key, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleAutoGearBoxes.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleAutoGearBox");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleAutoGearBoxPaths[i]));
            if (mVehicleComponentTracker.mAutoGearBoxes.find(key) != mVehicleComponentTracker.mAutoGearBoxes.end())
                continue;
            const auto& scanAg = scanned.vehicleAutoGearBoxes[i];
            AutoGearBoxDesc* engineDesc = ICE_PLACEMENT_NEW(AutoGearBoxDesc)();
            engineDesc->upRatios   = scanAg->upRatios;
            engineDesc->downRatios = scanAg->downRatios;
            engineDesc->latency    = scanAg->latency;
            mVehicleComponentTracker.mAutoGearBoxes.insert({ key, engineDesc });
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
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleBrakesPaths[i]));
            const auto& scanBrake = scanned.vehicleBrakes[i];
            const std::pair<omni::physics::parse::ObjectKey, uint8_t> mapKey{ key, scanBrake->brakesIndex };
            if (mVehicleComponentTracker.mBrakesByPathIndex.find(mapKey) !=
                mVehicleComponentTracker.mBrakesByPathIndex.end())
                continue;
            BrakesDesc* engineDesc = ICE_PLACEMENT_NEW(BrakesDesc)();
            // 7A.6: brakes instance token (e.g. "brakes0"/"brakes1") doubles
            // as the NonlinearCmdResponse instance lookup key.
            const omni::physics::parse::TokenId brakesInst = scanned.vehicleBrakesInstanceTokens[i];
            engineDesc->nonlinearCmdResponse = findNcr(key, brakesInst);
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
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleSteeringBasicPaths[i]));
            if (mVehicleComponentTracker.mSteeringByPath.find(key) != mVehicleComponentTracker.mSteeringByPath.end())
                continue;
            const auto& scanSt = scanned.vehicleSteeringBasic[i];
            SteeringBasicDesc* engineDesc = ICE_PLACEMENT_NEW(SteeringBasicDesc)();
            // "steer" is the fixed single-apply-schema instance name PhysxSchemaTokens->steer
            // names; interned straight off this scan's own source, matching the "drive" swap above.
            engineDesc->nonlinearCmdResponse =
                findNcr(key, scanned.source().internToken("steer"));  // 7A.6
            engineDesc->wheels               = scanSt->wheels;
            engineDesc->angleMultipliers     = scanSt->angleMultipliers;
            engineDesc->maxSteerAngle        = scanSt->maxSteerAngle;
            mVehicleComponentTracker.mSteeringBasic.push_back(engineDesc);
            mVehicleComponentTracker.mSteeringByPath.insert({ key, engineDesc });
        }
        for (size_t i = 0; i < scanned.vehicleSteeringAckermann.size(); ++i)
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs:vehicleSteeringAckermann");
            const omni::physics::parse::ObjectKey key = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehicleSteeringAckermannPaths[i]));
            if (mVehicleComponentTracker.mSteeringByPath.find(key) != mVehicleComponentTracker.mSteeringByPath.end())
                continue;
            const auto& scanSt = scanned.vehicleSteeringAckermann[i];
            SteeringAckermannDesc* engineDesc = ICE_PLACEMENT_NEW(SteeringAckermannDesc)();
            // "steer" is the fixed single-apply-schema instance name PhysxSchemaTokens->steer
            // names; interned straight off this scan's own source, matching the "drive" swap above.
            engineDesc->nonlinearCmdResponse =
                findNcr(key, scanned.source().internToken("steer"));  // 7A.6
            engineDesc->wheel0               = scanSt->wheel0;
            engineDesc->wheel1               = scanSt->wheel1;
            engineDesc->maxSteerAngle        = scanSt->maxSteerAngle;
            engineDesc->wheelBase            = scanSt->wheelBase;
            engineDesc->trackWidth           = scanSt->trackWidth;
            engineDesc->strength             = scanSt->strength;
            mVehicleComponentTracker.mSteeringAckermann.push_back(engineDesc);
            mVehicleComponentTracker.mSteeringByPath.insert({ key, engineDesc });
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
            const omni::physics::parse::ObjectKey vehicleKey = mAttachedStage.keyFor(scanned.source().sourceKeyToString(scanned.vehiclePaths[i]));
            if (mVehicleComponentTracker.mVehicleByPath.find(vehicleKey) !=
                mVehicleComponentTracker.mVehicleByPath.end())
                continue;

            // Component-applied checks route through the source (single-apply via
            // hasSchema; the multi-apply Brakes via hasSchema [any instance] +
            // forEachMultiApplyInstance [specific instance]). A null source skips
            // pre-pop so legacy parseVehicle runs — the same safe fallback the
            // tracker-miss `continue`s rely on. No UsdPrim.
            const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();
            if (!src || !src->exists(vehicleKey))
                continue;  // legacy parser will report any error path
            const omni::physics::parse::KnownTokens& tok = mAttachedStage.getKnownTokens();
            if (!src->hasSchema(vehicleKey, tok.physxVehicleAPI))
                continue;  // legacy parser will report any error path

            // ---- Drive: rel-or-API on vehicle prim --------------------
            // Mirror legacy parseDrive semantics: prefer the authored
            // rel target; on missing or empty rel, fall back to the API
            // applied on the vehicle prim itself.  Error paths
            // (rel with >1 targets, both DriveBasic+DriveStandard on
            // the same prim) are logged here and the vehicle is dropped.
            omni::physics::parse::ObjectKey driveKey;
            bool driveAuthored = false;
            {
                // physxVehicle:drive relationship targets via the source (an
                // unauthored/absent rel reports empty — matches the legacy
                // `if (GetDriveRel().HasAuthoredTargets()) GetTargets(...)`). These
                // are already live keys (src is mAttachedStage's own source), so no
                // rekey round-trip is needed.
                std::vector<omni::physics::parse::ObjectKey> driveKeys;
                src->getRelationshipTargets(vehicleKey, tok.physxVehicleDrive, driveKeys);
                if (driveKeys.size() > 1)
                {
                    CARB_LOG_ERROR(
                        "Usd Physics: \"%s\" must not have more than 1 \"drive\" relationship defined.",
                        mAttachedStage.textFor(vehicleKey));
                    continue;
                }
                if (driveKeys.size() == 1)
                {
                    driveKey = driveKeys[0];
                    driveAuthored = true;
                }
                else
                {
                    const bool standardOnSelf = src->hasSchema(vehicleKey, tok.physxVehicleDriveStandardAPI);
                    const bool basicOnSelf    = src->hasSchema(vehicleKey, tok.physxVehicleDriveBasicAPI);
                    if (standardOnSelf && basicOnSelf)
                    {
                        CARB_LOG_ERROR(
                            "Usd Physics: vehicle \"%s\" has both PhysxSchemaPhysxVehicleDriveStandardAPI and "
                            "PhysxSchemaPhysxVehicleDriveBasicAPI applied. Only one is allowed.",
                            mAttachedStage.textFor(vehicleKey));
                        continue;
                    }
                    if (standardOnSelf || basicOnSelf)
                    {
                        driveKey = vehicleKey;
                        driveAuthored = true;
                    }
                }
            }
            DriveDesc* drive = nullptr;
            if (driveAuthored)
            {
                auto stdIt = mVehicleComponentTracker.mDrivesStandard.find(driveKey);
                if (stdIt != mVehicleComponentTracker.mDrivesStandard.end())
                {
                    drive = stdIt->second;
                }
                else
                {
                    auto basicIt = mVehicleComponentTracker.mDrivesBasic.find(driveKey);
                    if (basicIt != mVehicleComponentTracker.mDrivesBasic.end())
                        drive = basicIt->second;
                }
                if (!drive)
                    continue;  // per-component parser already logged the validation error
            }

            // ---- Differential: applied directly on vehicle prim -------
            const bool diffApiApplied =
                src->hasSchema(vehicleKey, tok.physxVehicleMultiWheelDifferentialAPI) ||
                src->hasSchema(vehicleKey, tok.physxVehicleTankDifferentialAPI);
            MultiWheelDifferentialDesc* differential = nullptr;
            if (diffApiApplied)
            {
                auto it = mVehicleComponentTracker.mDifferentialsByPath.find(vehicleKey);
                if (it == mVehicleComponentTracker.mDifferentialsByPath.end())
                    continue;
                differential = it->second;
            }

            // ---- Steering: applied directly on vehicle prim -----------
            const bool steeringApiApplied =
                src->hasSchema(vehicleKey, tok.physxVehicleSteeringAPI) ||
                src->hasSchema(vehicleKey, tok.physxVehicleAckermannSteeringAPI);
            SteeringDesc* steering = nullptr;
            if (steeringApiApplied)
            {
                auto it = mVehicleComponentTracker.mSteeringByPath.find(vehicleKey);
                if (it == mVehicleComponentTracker.mSteeringByPath.end())
                    continue;
                steering = it->second;
            }

            // ---- Brakes: multi-apply (brakes0 / brakes1) --------------
            std::vector<const BrakesDesc*> brakes;
            const bool brakesApiApplied = src->hasSchema(vehicleKey, tok.physxVehicleBrakesAPI);
            if (brakesApiApplied)
            {
                bool brakesPrePopOk = true;
                const omni::physics::parse::TokenId brakesTokens[] = { tok.brakes0, tok.brakes1 };
                for (uint32_t bi = 0; bi < 2; ++bi)
                {
                    auto it = mVehicleComponentTracker.mBrakesByPathIndex.find(
                        { vehicleKey, static_cast<uint8_t>(bi) });
                    if (it != mVehicleComponentTracker.mBrakesByPathIndex.end())
                    {
                        brakes.push_back(it->second);
                        continue;
                    }
                    // Tracker miss: only counts as a failure when the
                    // instance API was actually applied (matches legacy
                    // parseBrakes). Per-instance check via the source's
                    // multi-apply instance enumeration.
                    const std::string appliedSchema =
                        std::string(src->tokenToString(tok.physxVehicleBrakesAPI)) + ":" +
                        std::string(src->tokenToString(brakesTokens[bi]));
                    const bool instanceApplied = src->hasSchema(vehicleKey, src->internToken(appliedSchema));
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
                        mAttachedStage.textFor(vehicleKey));
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
            mVehicleComponentTracker.mVehicleByPath.insert({ vehicleKey, engineDesc });
        }

        // ----- Apply collected time-sampled callbacks --------------
        callbacks::applyTimeSampledCallbacks(mAttachedStage, cbList);
    }

    // Source-driven load entry. `scanRoots` are the subtree roots to scan (the
    // pseudo-root for a whole-stage load; the changed prims for an incremental
    // update). `excludePaths` (replicator selective load) skips those subtrees.
    // No UsdPrim / PrimIterator: the backend scanStage builds the USD ranges and
    // the post-scan passes walk via IPhysicsSource.
    bool loadFromRange(const std::vector<std::string>& scanRoots, const PathSet* excludePaths, bool initialStageLoad)
    {
        mSceneFound = false;
        mNoValidScene = false;
        mParsingFlags = 0;
        mFilteredPairsPaths.clear();

        // parse::scanStage takes source-path strings, not ObjectKeys.
        std::vector<std::string> excludeStrings;
        if (excludePaths)
        {
            excludeStrings.reserve(excludePaths->size());
            for (const omni::physics::parse::ObjectKey excludeKey : *excludePaths)
                excludeStrings.push_back(std::string(mAttachedStage.textViewFor(excludeKey)));
        }

        // Incremental ovstage load: one load-cache window over the scan AND everything after it
        // (actor setup, the mass update). The scan's merged columnar read then also serves the
        // follow-up reads that would otherwise go back to ovstage per prim; the source's covered-miss
        // answers stay exact because nothing in this range is written while it runs. The initial
        // load keeps its scan-scoped window (its whole-stage read groups would otherwise be held
        // through processScannedDescs).
        struct ScopedOvstageLoadCache
        {
            omni::physics::ovstage::OvstageSource* source = nullptr;
            ScopedOvstageLoadCache(omni::physics::parse::IPhysicsSource* src, bool open)
                : source(open ? dynamic_cast<omni::physics::ovstage::OvstageSource*>(src) : nullptr)
            {
                if (source && !source->loadCacheActive())
                    source->beginLoadCache();
                else
                    source = nullptr; // already someone else's window
            }
            ~ScopedOvstageLoadCache()
            {
                if (source)
                {
                    source->clearLoadCache();
                    source->clearBucket();
                }
            }
        } scopedLoadCache(mAttachedStage.hasExternalSource() ? mAttachedStage.getSource() : nullptr,
                          !initialStageLoad);

        omni::physics::parse::ScannedStage scanned;
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:scanStage");
            // UsdLoad::attach()/attachOvstage() always register a scan backend before
            // loadFromRange can run, so scanStage always has a backend to dispatch to;
            // without one it silently returns an empty scan and every attach bails out
            // at `!scanned.sourcePtr()` below.
            scanned = omni::physics::parse::scanStage(mAttachedStage.attachTarget(), scanRoots,
                excludeStrings, omni::physics::parse::ScanOptions{}, omni::physx::usdparser::iceDescriptorAllocator());
        }
        if (!scanned.sourcePtr())
            return false;

        if (!scanned.particleSystems.empty() || !scanned.particleSets.empty() ||
            !scanned.particleSamplers.empty() || !scanned.particleAnisotropies.empty() ||
            !scanned.particleSmoothings.empty() || !scanned.particleIsosurfaces.empty())
        {
            mParsingFlags |= ParsingFlag::eParseParticles;
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

        {
            // Bulk hierarchy-read window: forEachChild() trusts the complete child cache's leaf
            // answer only inside one. Spans the scoped side-effects walk (which asks every leaf of
            // a re-parsed subtree) and processScannedDescs (getMeshAttributes hits that path once
            // per mesh). beginHierarchyBulkRead(), not beginLoadCache(): a load cache would also
            // arm the attribute covered-miss machinery and seal attributes a prefetch did not return.
            struct ScopedOvstageHierarchyBulkRead
            {
                omni::physics::ovstage::OvstageSource* source = nullptr;
                explicit ScopedOvstageHierarchyBulkRead(omni::physics::parse::IPhysicsSource* src)
                    : source(dynamic_cast<omni::physics::ovstage::OvstageSource*>(src))
                {
                    if (source)
                        source->beginHierarchyBulkRead();
                }
                ~ScopedOvstageHierarchyBulkRead()
                {
                    if (source)
                        source->endHierarchyBulkRead();
                }
            } scopedHierarchyBulkRead(mAttachedStage.getSource());

            {
                CARB_PROFILE_ZONE(0, "UsdPhysics:gatherPerPrimSideEffects");
                if (mAttachedStage.hasExternalSource())
                {
                    gatherScannedSideEffects(scanned);
                    gatherSourceOnlySideEffects(scanRoots, excludePaths);
                }
                else
                    gatherPerPrimSideEffects(scanRoots, excludePaths);
            }

            if (initialStageLoad)
            {
                scopedKnownKeys.source = seedOvstageKnownKeysForInitialLoad(mAttachedStage, scanRoots);
            }

            CARB_PROFILE_ZONE(0, "UsdPhysics:processScannedDescs");
            processScannedDescs(scanned);
        }

        if (mNoValidScene)
            return true;

        // We dont have a PhysX scene, PhysX should not simulate anything early exit
        if (mNoPhysXScene)
            return true;

        if(OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
        {
            // scristiano: do not parse particles, vehicles or internal objects when forcing single scene for inspector (for now)
            mParsingFlags = 0;
        }

        // require always a scene
        if (initialStageLoad && !mSceneFound)
        {
            // ObjectKey-native (ADR-0019): the engine-side creation below only ever needs the
            // opaque source key, never a resolvable SdfPath, so it goes through
            // PhysXUsdPhysicsInterface::createObject's ObjectKey-taking overload directly
            // rather than the SdfPath-taking createObject() free function above.
            auto createDefaultSceneObject = [&](omni::physics::parse::ObjectKey sceneKey)
            {
                PhysxSceneDesc sceneDesc;
                const omni::physics::parse::SourceUnits sceneUnits = mAttachedStage.getSourceUnits();
                omni::physics::parse::setToDefault(sceneDesc, sceneUnits);
                setToDefault(sceneDesc.defaultMaterialDesc);
                setToDefault(sceneUnits, sceneDesc.defaultDeformableMaterialDesc);
                setToDefault(sceneUnits, sceneDesc.defaultSurfaceDeformableMaterialDesc);
                setToDefault(sceneDesc.defaultPBDMaterialDesc);
                const ObjectId sceneId =
                    mAttachedStage.getPhysXPhysicsInterface()->createObject(mAttachedStage, sceneKey, sceneDesc);
                if (sceneId != kInvalidObjectId)
                {
                    mAttachedStage.getObjectDatabase()->findOrCreateEntry(sceneKey, sceneDesc.type, sceneId);
                    mSceneFound = true;
                    mNoPhysXScene = false;
                    mNumScenes++;
                }
            };

            // Opaque ObjectDb bookkeeping identity. PhysXStageUpdate.cpp's physXReset()
            // inlines the same literal as an SdfPath for its UsdStage::RemovePrim().
            const omni::physics::parse::ObjectKey tempPhysicsSceneKey =
                mAttachedStage.keyFor("/PhysicsScene_16e12ee3daea");
            const bool updateToUsd = OmniPhysX::getInstance().getCachedSettings().updateToUsd;
            if (updateToUsd)
            {
                // The temp-scene flag schedules the matching session-layer removal in
                // physXReset(), so it must record whether the prim was ACTUALLY authored:
                // setting it after a failed author leaves every later reset retrying a
                // removal that can never find its prim.
                const bool authored = mAttachedStage.createDefaultPhysicsScenePlaceholder(tempPhysicsSceneKey);
                if (authored)
                {
                    PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eInfo,
                        "Physics USD: Physics scene not found. A temporary default PhysicsScene prim was added automatically!");
                    createDefaultSceneObject(tempPhysicsSceneKey);
                    OmniPhysX::getInstance().setHasTempPhysicsScene(true);
                    OmniPhysX::getInstance().setTempPhysicsSceneKey(tempPhysicsSceneKey);
                }
                else
                {
                    // No stage to author into: the default scene still exists in the
                    // simulation, it just has no USD counterpart to remove later.
                    PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eInfo,
                        "Physics USD: Physics scene not found. A default PhysicsScene was created for the simulation only (no stage to author it into).");
                    createDefaultSceneObject(tempPhysicsSceneKey);
                }
            }
            else
            {
                createDefaultSceneObject(tempPhysicsSceneKey);
            }
        }

        // create deformable materials first
        {
            for (const std::pair<omni::physics::parse::ObjectKey, PBDMaterialDesc*>& pair : mPDBMatrialsDescs)
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
                // createShape's own registration still resolves attachedStage.pathFor(key)
                // under USD so ObjectDb's SdfPath-keyed maps stay populated.
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
            for (std::pair<omni::physics::parse::ObjectKey, BodyDescAndColliders>& ref : mAdditionalBodyVector)
            {
                finalizeRigidBody(mAttachedStage, ref.second);
            }
        }

        // create articulation links
        {
            CARB_PROFILE_ZONE(0,"OmniPhysX:createArticulationLinks");
            // Group transform-sanitation changes into one write batch (a USD backend's
            // SdfChangeBlock); no-op without a write sink.
            DataWriteScope writeScope(mAttachedStage.getDataWrite());
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
            // See the createArticulationLinks block above.
            DataWriteScope writeScope(mAttachedStage.getDataWrite());
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
                if (deformableDesc.desc->collisionMeshKey.valid())
                    deformableDesc.desc->collisionGroup = getCollisionGroup(mAttachedStage, deformableDesc.desc->collisionMeshKey);
                // finalizeDeformableBody is ObjectKey-native (ADR-0019); simMeshMaterial is
                // already an ObjectKey, so no path round-trip needed.
                finalizeDeformableBody(mAttachedStage, deformableDesc.desc, deformableDesc.simMeshMaterial);
                createDeformableBody(mAttachedStage, deformableDesc.desc, deformableDesc.path);
            }
        }

        // create forces
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createForces");
            for (const std::pair<omni::physics::parse::ObjectKey, PhysxForceDesc*>& pair : mPhysxForceDescs)
            {
                finalizePhysxForce(mAttachedStage, pair.first, *pair.second);
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
            KeySet jointInstancerKeys;

            // Source-backed type/schema dispatch keyed by ObjectKey (no UsdPrim).
            const omni::physics::parse::IPhysicsSource* src = mAttachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (src)
                tok.intern(*src);

            // Index by SOURCE-TEXT, not ObjectKey: the scan's source and attachedStage's
            // persistent source are different IPhysicsSource instances, and re-keying
            // scan-space text is not guaranteed to yield the ObjectKey forEachLoadObject
            // hands back for the same prim (ovstage keys can be non-canonical -- see
            // OvstageSource::canonicalHandleRaw). Comparing ObjectKeys here silently
            // double-created particle systems/sets under ovstage. Text is the one identity
            // stable across both sources (ADR-0004).
            const omni::physics::parse::IPhysicsSource& scanSrc = scanned.source();
            std::unordered_map<std::string_view, const omni::physics::parse::ParticleSystemDesc*> scannedSystems;
            for (const auto& sysUPtr : scanned.particleSystems)
                scannedSystems[scanSrc.sourceKeyToString(sysUPtr->systemKey)] = sysUPtr.get();
            std::unordered_map<std::string_view, const omni::physics::parse::ParticleSetDesc*> scannedSets;
            for (const auto& setUPtr : scanned.particleSets)
                scannedSets[scanSrc.sourceKeyToString(setUPtr->primKey)] = setUPtr.get();
            std::unordered_map<std::string_view, const omni::physics::parse::ParticleSamplingDesc*> scannedSamplers;
            for (size_t i = 0; i < scanned.particleSamplers.size(); ++i)
                scannedSamplers[scanSrc.sourceKeyToString(scanned.particleSamplerKeys[i])] = scanned.particleSamplers[i].get();
            std::unordered_set<std::string_view> queuedParticleSystems;
            std::unordered_set<std::string_view> queuedParticleSets;
            // Voxel-map subtrees are pruned by forEachLoadObject below (it stops descending under
            // an InfiniteVoxelMapAPI Xform). scanStage has no voxel-map awareness, so the scanned
            // particle lists still contain prims nested under a voxel map; the catch-all loops
            // after the walk must replicate the prune and skip them. Text-prefix (mirrors
            // SdfPath::HasPrefix) for the same cross-source-identity reason as the maps above.
            std::vector<std::string_view> voxelMapRoots;
            const auto hasTextPrefix = [](std::string_view path, std::string_view root) -> bool
            {
                if (path == root)
                    return true;
                if (root.size() == 1) // root == "/" (the pseudo-root): every path is under it.
                    return path.size() > root.size() && path[0] == '/';
                return path.size() > root.size() && path.compare(0, root.size(), root) == 0 &&
                       path[root.size()] == '/';
            };

            forEachLoadObject(scanRoots, excludePaths, [&](omni::physics::parse::ObjectKey primObjKey) -> bool
            {
                if (src->isA(primObjKey, tok.physxParticleSystemType))
                {
                    auto sysIt = scannedSystems.find(mAttachedStage.textViewFor(primObjKey));
                    if (sysIt != scannedSystems.end())
                    {
                        particleSysDescs.push_back(buildParticleSystemDesc(mAttachedStage, scanned, *sysIt->second));
                        queuedParticleSystems.insert(sysIt->first);
                    }
                }
                else if (src->isA(primObjKey, tok.xformType))
                {
                    if (src->hasSchema(primObjKey, src->internToken("InfiniteVoxelMapAPI")))
                    {
                        // Unsupported in the USD-free runtime: no descriptor, no object. The subtree
                        // is still pruned so its Chunk_* PointInstancers are not parsed as bodies.
                        CARB_LOG_WARN("InfiniteVoxelMapAPI on '%s' is not supported by the USD-free runtime; the voxel map is ignored",
                                      mAttachedStage.textFor(primObjKey));
                        voxelMapRoots.push_back(mAttachedStage.textViewFor(primObjKey));
                        return true; // prune subtree
                    }
                    return false;
                }
                else if (src->isA(primObjKey, tok.meshType) &&
                         src->hasSchema(primObjKey, tok.physxParticleSamplingAPI))
                {
                    cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
                    // Build the engine sampling descriptor from the scanned
                    // parse descriptor (no USD re-read).
                    auto samplerIt = scannedSamplers.find(mAttachedStage.textViewFor(primObjKey));
                    const omni::physics::parse::ParticleSamplingDesc* scanSampling =
                        samplerIt != scannedSamplers.end() ? samplerIt->second : nullptr;
                    ParticleSamplingDesc samplingDesc;
                    omni::physics::parse::ObjectKey particleKey;
                    if (scanSampling)
                    {
                        samplingDesc.samplingDistance = scanSampling->samplingDistance;
                        samplingDesc.sampleVolume = scanSampling->sampleVolume;
                        samplingDesc.maxSamples = scanSampling->maxSamples;
                        samplingDesc.pointWidth = scanSampling->pointWidth;
                        particleKey = scanSampling->particleSetKey.valid() ?
                            mAttachedStage.keyFor(scanSrc.sourceKeyToString(scanSampling->particleSetKey)) : omni::physics::parse::ObjectKey{};
                    }
                    // check if it points to a valid particle prim
                    samplingDesc.particleSetKey = particleKey;
                    if (!scanSampling || !src->exists(particleKey) || !src->hasSchema(particleKey, tok.physxParticleSetAPI) ||
                        !(src->isA(particleKey, tok.pointsType) || src->isA(particleKey, tok.pointInstancerType)))
                    {
                        CARB_LOG_WARN("%s: particle sampler does not point to a valid particle prim, needs to be set before stage parsing starts.", mAttachedStage.textFor(primObjKey));
                    }
                    else if (!cookingDataAsync)
                    {
                        CARB_LOG_WARN("%s: particle sampling failed.", mAttachedStage.textFor(primObjKey));
                    }
                    else
                    {
                        // AD: OM-89182 - we need to disallow adding new samplers during simulation: it could have been just a path change, and we don't have the setup to clean things up.
                        // we never allowed processing changes in particleAuthoring and this code is just here in case there is no PhysXUI or we need to block at the sim start for async
                        // tasks to finish.
                        if (OmniPhysX::getInstance().getSimulationStepCount() == 0)
                        {
                            omni::physx::particles::createParticleSampler(primObjKey, particleKey);
                            cookingDataAsync->poissonSampleMesh(primObjKey, mAttachedStage, samplingDesc, false, false);
                        }
                        else
                        {
                            CARB_LOG_WARN("%s: it is not allowed to add new particle samplers once simulation has been started, ignoring.", mAttachedStage.textFor(primObjKey));
                        }
                    }
                }
                else if (src->isA(primObjKey, tok.pointBasedType) &&
                         src->hasSchema(primObjKey, tok.physxParticleSetAPI))
                {
                    auto setIt = scannedSets.find(mAttachedStage.textViewFor(primObjKey));
                    if (setIt != scannedSets.end())
                    {
                        if (ParticleSetDesc* desc = buildParticleSetDesc(mAttachedStage, scanned, *setIt->second))
                        {
                            particleDescs.push_back(desc);
                            queuedParticleSets.insert(setIt->first);
                        }
                    }
                }
                else if (src->isA(primObjKey, tok.pointInstancerType))
                {
                    if (src->hasSchema(primObjKey, tok.physxParticleSetAPI))
                    {
                        auto setIt = scannedSets.find(mAttachedStage.textViewFor(primObjKey));
                        if (setIt != scannedSets.end())
                        {
                            if (ParticleSetDesc* desc = buildParticleSetDesc(mAttachedStage, scanned, *setIt->second))
                            {
                                particleDescs.push_back(desc);
                                queuedParticleSets.insert(setIt->first);
                            }
                        }
                    }
                    else if (!src->isInstance(primObjKey) && !src->isInstanceProxy(primObjKey))
                    {
                        parseRigidBodyInstancer(mAttachedStage, primObjKey, mFilteredPairs);
                    }
                }
                else if (src->isA(primObjKey, tok.physxPhysicsJointInstancerType))
                {
                    if (!src->isInstance(primObjKey) && !src->isInstanceProxy(primObjKey))
                    {
                        jointInstancerKeys.insert(primObjKey);
                    }
                }
                return false;
            });

            // Skip scanned particle prims nested under a pruned voxel-map subtree so the catch-all
            // does not create particle systems/sets that forEachLoadObject deliberately excluded.
            // Text-prefix rather than an ObjectKey ancestry walk: re-keying scan-space text
            // just for this check would reintroduce the cross-source identity hazard above.
            auto underVoxelMap = [&](std::string_view path) -> bool
            {
                for (const std::string_view root : voxelMapRoots)
                    if (hasTextPrefix(path, root))
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

            for (const omni::physics::parse::ObjectKey jointInstancerKey : jointInstancerKeys)
            {
                parseJointInstancer(mAttachedStage, jointInstancerKey);
            }

            createParticleSystemsAndObjects(mAttachedStage, particleSysDescs, particleDescs, mFilteredPairs);
        }

        // Re-cooking the collision mesh above may produce a different topology
        // (cross-platform non-determinism, OMPE-90043), invalidating the pre-baked attachment data.
        // AttachmentAuthoring cannot fix this in time because ScopedBlockUSDUpdates suppresses
        // USD notices during cooking.
        {
            generateAutoDeformableAttachmentLayouts(mAttachedStage, mAutoDeformableAttachmentKeys,
                mDeformableAttachmentVector, mDeformableCollisionFilterVector);
            { refreshAutoDeformableAttachments(mAttachedStage, mDeformableAttachmentVector, mDeformableCollisionFilterVector); }
        }

        // create deformable attachments and filters
        {
            { createDeformableAttachments(mAttachedStage, mDeformableAttachmentVector); }
        }
        {
            { createDeformableCollisionFilters(mAttachedStage, mDeformableCollisionFilterVector); }
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
	                const omni::physics::parse::ObjectKey materialKey = tireFrictionTableDesc->materialPaths[j];
	                ObjectId materialId = mAttachedStage.getObjectDatabase()->findEntry(materialKey, eMaterial);
	                tireFrictionTableDesc->materialIds[j] = materialId;
	            }

	            createObject(mAttachedStage, tireFrictionTableDesc->key, tireFrictionTableDesc, true);
	        }
	
	        VehicleComponentTracker& vehicleComponentTracker = mVehicleComponentTracker;
	        const omni::physics::parse::IPhysicsSource* vehSrc = mAttachedStage.getSource();
	        omni::physics::parse::KnownTokens vehTok;
	        if (vehSrc)
	            vehTok.intern(*vehSrc);
	        forEachLoadObject(scanRoots, excludePaths, [&](omni::physics::parse::ObjectKey primObjKey) -> bool
	        {
	            // note: PhysxVehicleContextAPI needs to be set before creating vehicles. However, checking here is not that easy since
                //       the necessary data is not readily available. Thus, the check is done as part of vehicle creation.
	            loadVehicle(mAttachedStage, primObjKey, vehicleComponentTracker);

	            // skip the instancer parsing: prune the subtree rooted at a point instancer.
	            return vehSrc && vehSrc->isA(primObjKey, vehTok.pointInstancerType);
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
        return true;
    }

private:
    bool mSceneFound;
    bool mNoValidScene;
    bool mNoPhysXScene;
    bool mCollectionsPopulated;
    uint32_t mNumScenes;

    uint32_t mParsingFlags;

    AttachedStage&  mAttachedStage;
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

    // A set: every consumer only ever needed distinct prims, not order or duplicates.
    KeySet mFilteredPairsPaths;
    CollisionPairVector mFilteredPairs;

    // forces
    std::vector<std::pair<omni::physics::parse::ObjectKey, PhysxForceDesc*>> mPhysxForceDescs;

    // particle materials
    std::vector<std::pair<omni::physics::parse::ObjectKey, PBDMaterialDesc*>> mPDBMatrialsDescs;

    KeySet mCollisionGroupsPrims;

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
    // prims carrying PhysxAutoDeformableAttachmentAPI, for in-memory sub-prim generation
    KeySet mAutoDeformableAttachmentKeys;
};

bool loadFromStage(AttachedStage& attachedStage, const PathSet* excludePaths)
{
    PHYSICS_PROFILE("Physics Load Stage");
    CARB_PROFILE_ZONE(0, "Physics Load Stage");
    // Whole-stage load: the scan root is the pseudo-root, "/". `excludePaths`
    // (replicator selective load) prunes the listed subtrees.
    const std::vector<std::string> scanRoots{ "/" };
    PhysxUsdPhysicsListener usdPhysicsListener(attachedStage);
    return usdPhysicsListener.loadFromRange(scanRoots, excludePaths, true);
}

void loadPhysicsFromPrimitive(AttachedStage& attachedStage, const std::vector<std::string>& updateRoots)
{
    PHYSICS_PROFILE("Physics Load Prims");
    CARB_PROFILE_ZONE(0, "Physics Load Prims");
    if (updateRoots.empty())
        return;

    // Incremental re-parse: the changed-prim paths are the scan roots; the
    // backend walks each subtree (instance proxies) and the post-scan passes
    // walk via IPhysicsSource. No PrimIterator.
    PhysxUsdPhysicsListener usdPhysicsListener(attachedStage);
    usdPhysicsListener.loadFromRange(updateRoots, nullptr, false);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
