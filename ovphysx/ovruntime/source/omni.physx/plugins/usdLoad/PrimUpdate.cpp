// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-6
 *
 * The per-prim update path runs from the source alone. `processUpdates` takes its
 * time-code factor from `AttachedStage::getSourceUnits().timeCodesPerSecond`
 * (REQ-PARSE-CORE-003 AC-13) rather than from `UsdStage::GetTimeCodesPerSecond()`,
 * which was this file's last `getStage()` call; and the incremental change handlers
 * gate individual stage *reads*, never the work around them.
 *
 * @implements REQ-PARSE-COL-004
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-XFORM-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-SIM-AUTOATTACH-001
 * @covers AC-6
 */

/**
 * @implements REQ-PARSE-INSTANCER-002
 * @covers AC-2
 */

/**
 * @implements REQ-PARSE-SCHEMAAPI-001
 * @covers AC-1 AC-2 AC-3
 *
 * The apiSchemas-change dispatch in `PrimChangeMap::checkPrimChange` reads the
 * previously-applied schema-API bitmask through `ObjectDb::getSchemaAPIs(ObjectKey)`,
 * not the `SdfPath`-keyed overload: every non-structural `changeSchemaAPI` writer
 * (`ContactReport.cpp` and siblings) writes only the `ObjectKey`-keyed map, so the
 * `SdfPath`-keyed read never observed those writes and add/remove detection desynced.
 */

/**
 * @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
 * @covers AC-11 AC-12 AC-13
 *
 * `onSourceChange` routes a value ChangeBatch to `applyOvstageValueBatch`, but only when
 * USD-update deferral is off (`!getAsyncUSDUpdate()`): the drain mutates PhysX immediately,
 * so it must not run while an async simulation step is in flight. It returns `false` only for
 * `DrainResult::Failed` (a partial commit -> hold the cursor, AC-12), and on `Applied` runs the
 * per-object parse path for exactly the rows the drain reported as unresolved (the split, AC-13).
 */

#include <atomic>
#include <cmath>

#include <carb/Types.h>
#include <carb/Defines.h>
#include <carb/profiler/Profile.h>
#include <carb/logging/Log.h>

#include <omni/core/ITypeFactory.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "PrimUpdate.h"
#include "LoadStage.h"
#include <OmniPhysX.h>

#include <omni/physics/parse/IChangeFeed.h>
#include <omni/physics/parse/KnownTokens.h>
#include <PhysXTools.h>

#include <omni/physx/PhysxTokens.h>
#include <omni/physx/IOvxPhysicsWrite.h> // applyOvstageValueBatch -- the ovstage drain value-apply

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{
namespace
{
// Test-only count of transform changes handled at group complete, i.e. past the pending-
// (re)parse guard in handleTransformChange. The guard has no other observable: skipping the
// walk changes no pose, so this is what pins it.
std::atomic_size_t& handledTransformChangeCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}
} // namespace

// Not declared in any header; test translation units forward-declare these.
void resetHandledTransformChangeCountForTest()
{
    handledTransformChangeCounter().store(0, std::memory_order_relaxed);
}

size_t getHandledTransformChangeCountForTest()
{
    return handledTransformChangeCounter().load(std::memory_order_relaxed);
}

bool isMovableBody(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey)
{
    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            if (it->first == eBody || it->first == ePhysxForce || it->first == eArticulationLink ||
                it->first == eParticleSet ||
                it->first == eXformActor ||
                it->first == eVolumeDeformableBody || it->first == eSurfaceDeformableBody)
            {
                return true;
            }
            it++;
        }
    }

    return false;
}

// Mirrors pxr::UsdGeomXformable::IsTransformationAffectedByAttrNamed, which delegates to
// UsdGeomXformOp::IsXformOp: true for the `xformOpOrder` attribute itself, or any attribute
// in the `xformOp:` namespace.
bool isTransformationAffectedByAttrNamed(std::string_view attrName)
{
    static const std::string_view kXformOpOrder = "xformOpOrder";
    static const std::string_view kXformOpPrefix = "xformOp:";
    return attrName == kXformOpOrder ||
           (attrName.size() >= kXformOpPrefix.size() &&
            attrName.compare(0, kXformOpPrefix.size(), kXformOpPrefix) == 0);
}

bool isNonMovable(const AttachedStage& attachedStage,
                  omni::physics::parse::ObjectKey primKeyObj,
                  omni::physics::parse::ObjectKey& resyncKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const ObjectDb* objectDb = attachedStage.getObjectDatabase();

    const uint64_t primAPIs = objectDb->getSchemaAPIs(primKeyObj);
    if (primAPIs & SchemaAPIFlag::eDeformableBodyAPI)
    {
        resyncKey = primKeyObj;
        return true;
    }

    const omni::physics::parse::ObjectKey root = src->getRootKey();
    for (omni::physics::parse::ObjectKey parent = src->getParent(primKeyObj);
         parent.valid() && parent != root; parent = src->getParent(parent))
    {
        const uint64_t parentAPIs = objectDb->getSchemaAPIs(parent);
        if (parentAPIs & SchemaAPIFlag::eRigidBodyAPI)
        {
            if (primAPIs & SchemaAPIFlag::eCollisionAPI)
            {
                resyncKey = primKeyObj;
                return true;
            }
            bool foundCollision = false;
            src->forEachDescendantPruned(
                primKeyObj,
                [&](omni::physics::parse::ObjectKey k) -> bool
                {
                    if (objectDb->getSchemaAPIs(k) & SchemaAPIFlag::eCollisionAPI)
                        foundCollision = true;
                    return false;
                },
                omni::physics::parse::DescendantScope::eActiveInstanced);
            if (foundCollision)
            {
                resyncKey = primKeyObj;
                return true;
            }
            return false;
        }
        else if (parentAPIs & SchemaAPIFlag::eDeformableBodyAPI)
        {
            resyncKey = parent;
            return true;
        }
    }
    return false;
}

void handleRemovedPrim(AttachedStage& attachedStage, omni::physics::parse::ObjectKey removedKey)
{
    if (removedKey.valid())
    {
        attachedStage.clearCookedGeometryUnderPath(removedKey);
        attachedStage.clearGeneratedAutoAttachmentLayoutsUnderPath(removedKey);
    }

    attachedStage.getPrimChangeMap().removePrim(removedKey);

    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(removedKey);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            attachedStage.getPhysXPhysicsInterface()->releaseObject(attachedStage, removedKey, it->second);
            it++;
        }

        attachedStage.getObjectDatabase()->removeEntries(removedKey);
        attachedStage.getObjectDatabase()->removeSchemaAPIs(removedKey);
    }
}

PrimChangeMap::PrimChangeMap()
{
}

PrimChangeMap::~PrimChangeMap()
{
    m_propertyChanges.clear();
}

// Only stages the ChangeParams: no TokenId can be minted yet, since a TokenId is valid only
// for the IPhysicsSource that minted it and no source need exist at this point.
// internRegisteredChanges does the interning from AttachedStage::rebuildSource(), so a source
// swap never leaves a stale TokenId behind.
void PrimChangeMap::registerPrimChange(const ChangeParams& changeParam)
{
    CARB_ASSERT(!changeParam.changeAttribute.empty());
    m_registeredChanges.push_back(changeParam);
}

void PrimChangeMap::internRegisteredChanges(const omni::physics::parse::IPhysicsSource& source)
{
    m_propertyChanges.clear();
    for (const ChangeParams& changeParam : m_registeredChanges)
    {
        PropertyChange pc;
        pc.onUpdate = changeParam.onUpdate;
        pc.onPrimCheckKey = changeParam.onPrimCheckKey;
        pc.onPrimCheckExtKey = changeParam.onPrimCheckExtKey;
        m_propertyChanges.insert(std::pair<omni::physics::parse::TokenId, PropertyChange>(
            source.internToken(changeParam.changeAttribute), pc));
    }
}

void PrimChangeMap::clearRegisteredChanges()
{
    m_propertyChanges.clear();
}

// Unlike registerPrimChange above, this interns immediately: the caller
// (AttachedStage::registerStageSpecificAttribute) always has a live source by
// the time it runs (mid-parse, long after the source is attached), so there is
// no ordering problem to defer.
void PrimChangeMap::registerStageSpecificChange(omni::physics::parse::TokenId attributeId, const ChangeParams& changeParam)
{
    PropertyChange pc;
    pc.onUpdate = changeParam.onUpdate;
    pc.onPrimCheckKey = changeParam.onPrimCheckKey;
    pc.onPrimCheckExtKey = changeParam.onPrimCheckExtKey;
    m_stageSpecificChanges.insert(std::pair<omni::physics::parse::TokenId, PropertyChange>(attributeId, pc));
}

void PrimChangeMap::clearStageSpecificChanges()
{
    m_stageSpecificChanges.clear();
}

void moveBody(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    PhysXUsdPhysicsInterface::Transform fcTransform;

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();

    bool scaleProvided = true;
    {
        // Guard against expired/invalid prims that may have been deleted during
        // this notification cycle (source-routed validity, no UsdPrim).
        if (!src || !src->exists(key))
            return;

        // Source-routed world transform at the current (Default) edit-notice
        // frame; per-call read (this runs on the USD change-notice path, not the
        // per-frame sim write-back).
        const ::physx::PxMat44d mat =
            internal::getWorldTransform(attachedStage, key, omni::physics::parse::ReadTime::defaultTime());
        ::physx::PxTransform pose;
        ::physx::PxVec3 sc;
        decomposeMatrix(pose, sc, mat);

        fcTransform.position = toFloat3(pose.p);
        fcTransform.orientation = toFloat4(pose.q.getNormalized());
        fcTransform.scale = toFloat3(sc);
    }

    bool structChange = false;
    bool articulationLinkChange = false;

    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(key);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            bool updateOk = attachedStage.getPhysXPhysicsInterface()->updateTransform(
                attachedStage, key, it->second, fcTransform, true, scaleProvided);
            if (!updateOk)
            {
                structChange = true;
                if (it->first == eArticulationLink)
                {
                    CARB_LOG_ERROR("Articulation scale changed for %s in runtime, this does trigger full stage reparse.",
                                   attachedStage.textFor(key));
                    articulationLinkChange = true;
                }
            }
            it++;
        }
    }

    if (structChange)
    {
        if (articulationLinkChange)
        {
            // A.B. we could do eventually better, but we might not want to reconstruct the hierarchy here atm
            UsdLoad::getUsdLoad()->releasePhysicsObjects(attachedStage.getStageId());
        }
        else
        {
            // Guard against expired/invalid prims that may have been deleted
            // during this notification cycle (source-routed validity, no UsdPrim).
            if (!src->exists(key))
                return;

            {
                PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
                PrimHierarchyStorage::Iterator iterator(primStorage, std::string(attachedStage.textFor(key)));
                for (size_t i = iterator.getDescendentsPaths().size(); i--;)
                {
                    const omni::physics::parse::ObjectKey childKey =
                        attachedStage.keyFor(iterator.getDescendentsPaths()[i]);
                    handleRemovedPrim(attachedStage, childKey);
                }
                primStorage.removeIteration(iterator);
                attachedStage.getPrimUpdateMap().addPrim(attachedStage, key);
            }
        }
    }
}

void movePointInstancer(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    // Prototypes relationship + per-instance arrays + the instancer world
    // transform are all read through IPhysicsSource (no UsdPrim).
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();

    std::vector<omni::physics::parse::ObjectKey> targets;
    internal::getRelationshipValue(attachedStage, key, tok.prototypes, targets);
    std::vector<ObjectIdMap::const_iterator> targetEntries;

    std::vector<int32_t> indices;
    std::vector<carb::Float3> positions;
    // Orientations are authored as half-precision quaternions; the array read
    // widens them to carb::Float4 lanes x,y,z,w with w == the real part.
    std::vector<carb::Float4> orientations;
    internal::getArrayValue(
        attachedStage, key, tok.positions, omni::physics::parse::ReadTime::defaultTime(), positions);
    internal::getArrayValue(
        attachedStage, key, tok.orientations, omni::physics::parse::ReadTime::defaultTime(), orientations);
    internal::getArrayValue(
        attachedStage, key, tok.protoIndices, omni::physics::parse::ReadTime::defaultTime(), indices);

    PhysXUsdPhysicsInterface::Transform fcTransform;

    const ::physx::PxMat44d instancerMatrix =
        internal::getWorldTransform(attachedStage, key, omni::physics::parse::ReadTime::defaultTime());
    ::physx::PxTransform instancerPose;
    ::physx::PxVec3 instancerScale;
    decomposeMatrix(instancerPose, instancerScale, instancerMatrix);
    const ::physx::PxQuat instancerRotation = instancerPose.q.getNormalized();
    fcTransform.scale = toFloat3(instancerScale);

    bool structChange = false;

    targetEntries.clear();

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();

    for (size_t iProt = 0; iProt < targets.size(); iProt++)
    {
        const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(targets[iProt]);
        if (!entries)
        {
            structChange = true;
            break;
        }
        targetEntries.push_back(entries->begin());
    }

    if (!structChange)
    {
        for (size_t i = 0; i < indices.size(); i++)
        {
            if (size_t(indices[i]) < targets.size())
            {
                // positions/orientations come straight off the source arrays, so
                // they are converted here, at the boundary, and the rest is PhysX math.
                const carb::Float3 instancePos =
                    i < positions.size() ? positions[i] : carb::Float3{ 0.0f, 0.0f, 0.0f };
                const ::physx::PxVec3d transfPos =
                    instancerMatrix.transform(::physx::PxVec3d(instancePos.x, instancePos.y, instancePos.z));

                const ::physx::PxQuat instanceOrient =
                    i < orientations.size() ? toPhysXQuat(orientations[i]) : ::physx::PxQuat(::physx::PxIdentity);
                // Quaternion products do NOT swap operands the way matrix
                // products do: the Gf and PhysX Hamilton products have the same
                // meaning (verified numerically -- in Gf, M(q1*q2) == M(q2)*M(q1),
                // which cancels the matrix swap). The half-precision authored
                // input is not unit-length, hence the normalization.
                const ::physx::PxQuat transfOrient =
                    (instanceOrient * instancerRotation).getNormalized();

                fcTransform.position = { float(transfPos.x), float(transfPos.y), float(transfPos.z) };
                fcTransform.orientation = toFloat4(transfOrient);

                ObjectIdMap::const_iterator& it = targetEntries[indices[i]];
                if (!attachedStage.getPhysXPhysicsInterface()->updateTransform(attachedStage,
                    targets[indices[i]], it->second, fcTransform))
                {
                    structChange = true;
                    break;
                }
                it++;

                if (structChange)
                    break;
            }
        }
    }


    if (structChange)
    {
        attachedStage.getPrimUpdateMap().removePrim(attachedStage, key);
        attachedStage.getPrimUpdateMap().addPrim(attachedStage, key);
    }
}

void PrimChangeMap::handleTransformChange(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    CARB_PROFILE_ZONE(0, "PrimChangeMap::handleTransformChange(ObjectKey)");
    if (attachedStage.getObjectDatabase()->empty())
        return;

    // A prim queued for (re)parse reads its pose at parse time; moving it now would only walk
    // its not-yet-loaded subtree through live source queries (cf. the apiSchemas guard in
    // onSourceChange).
    if (attachedStage.getPrimUpdateMap().isInPrimAddMap(attachedStage, key))
        return;
    handledTransformChangeCounter().fetch_add(1, std::memory_order_relaxed);

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();

    bool bodyMoved = false;
    if (isMovableBody(attachedStage, key))
    {
        bodyMoved = true;
        moveBody(attachedStage, key);
    }

    if (!bodyMoved || OmniPhysX::getInstance().getInternalPhysXDatabase().getNestedBodiesUsed())
    {
        if (!src || !src->exists(key) || src->isPrototype(key))
            return;

        const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
        if (src->isA(key, tok.pointInstancerType))
        {
            movePointInstancer(attachedStage, key);
        }
        else
        {
            omni::physics::parse::ObjectKey resyncKey;
            if (isNonMovable(attachedStage, key, resyncKey))
            {
                PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
                PrimHierarchyStorage::Iterator iterator(primStorage, std::string(attachedStage.textFor(resyncKey)));
                for (size_t i = iterator.getDescendentsPaths().size(); i--;)
                {
                    const omni::physics::parse::ObjectKey childKey =
                        attachedStage.keyFor(iterator.getDescendentsPaths()[i]);
                    handleRemovedPrim(attachedStage, childKey);
                }
                primStorage.removeIteration(iterator);
                attachedStage.getPrimUpdateMap().addPrim(attachedStage, resyncKey);
            }
            else
            {
                // eActiveInstanced: a collider can live only in a referenced/prototype subtree
                // reached through instance proxies, same as isNonMovable's own subtree scan above.
                src->forEachDescendantPruned(
                    key,
                    [&](omni::physics::parse::ObjectKey childKey) -> bool
                    {
                        bool resetXformOpStack = false;
                        internal::getLocalTransform(attachedStage, childKey,
                                                    omni::physics::parse::ReadTime::defaultTime(), resetXformOpStack);
                        if (resetXformOpStack)
                            return true;

                        if (isMovableBody(attachedStage, childKey))
                        {
                            moveBody(attachedStage, childKey);
                        }
                        return false;
                    },
                    omni::physics::parse::DescendantScope::eActiveInstanced);
            }
        }
    }
}

static const SchemaAPIFlag::Enum gMimicJointSchemaAPIFlagList[] = {
    SchemaAPIFlag::eMimicJointRotXAPI, SchemaAPIFlag::eMimicJointRotYAPI, SchemaAPIFlag::eMimicJointRotZAPI
};
static const uint32_t gMimicJointAPITokenCount =
    sizeof(gMimicJointSchemaAPIFlagList) / sizeof(gMimicJointSchemaAPIFlagList[0]);

static const SchemaAPIFlag::Enum  gDrivePerformanceEnvelopAPIFlagList[] =
{
 SchemaAPIFlag::eDrivePerformanceEnvelopeAngularAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeLinearAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotXAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotYAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotZAPI
};

static const SchemaAPIFlag::Enum  gJointAxisAPIFlagList[] =
{
 SchemaAPIFlag::eJointAxisAngularAPI,
 SchemaAPIFlag::eJointAxisLinearAPI,
 SchemaAPIFlag::eJointAxisRotXAPI,
 SchemaAPIFlag::eJointAxisRotYAPI,
 SchemaAPIFlag::eJointAxisRotZAPI
};

// Schema-API bookkeeping, called by checkPrimChange()'s apiSchemas branch. The resync
// target is reported as the parent ObjectKey.
bool checkForStructuralSchemaAPIChanges(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key,
    const std::vector<omni::physics::parse::TokenId>& apiSchemas, uint32_t categories, omni::physics::parse::ObjectKey& resyncKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    const omni::physics::parse::TokenId mimicJointAPITokens[] = {
        tok.physxMimicJointAPIRotX, tok.physxMimicJointAPIRotY, tok.physxMimicJointAPIRotZ
    };

    std::vector<ObjectCategory> appliedCategories;
    for (const omni::physics::parse::TokenId apiSchema : apiSchemas)
    {
        if (apiSchema == tok.physicsRigidBodyAPI)
        {
            if (!(categories & SchemaAPIFlag::eRigidBodyAPI))
            {
                return true;
            }
            appliedCategories.push_back(eBody);
        }
        else if (apiSchema == tok.physicsCollisionAPI)
        {
            if (!(categories & SchemaAPIFlag::eCollisionAPI))
            {
                return true;
            }
            appliedCategories.push_back(eShape);
        }
        if (apiSchema == tok.physxForceAPI)
        {
            if (!(categories & SchemaAPIFlag::ePhysxForceAPI))
            {
                return true;
            }
            appliedCategories.push_back(ePhysxForce);
        }
        else if (apiSchema == tok.physxParticleSetAPI)
        {
            if (!(categories & SchemaAPIFlag::eParticleSetAPI))
            {
                return true;
            }
            appliedCategories.push_back(eParticleSet);
        }
        else if (apiSchema == tok.omniphysicsDeformableBodyAPI)
        {
            if (!(categories & SchemaAPIFlag::eDeformableBodyAPI))
            {
                return true;
            }
            appliedCategories.push_back(eDeformableBody);
        }
        else if (apiSchema == tok.OmniPhysicsSurfaceDeformableSimAPI)
        {
            if (!(categories & SchemaAPIFlag::eSurfaceDeformableSimAPI))
            {
                return true;
            }
            appliedCategories.push_back(eSurfaceDeformableBody);
        }
        else if (apiSchema == tok.OmniPhysicsVolumeDeformableSimAPI)
        {
            if (!(categories & SchemaAPIFlag::eVolumeDeformableSimAPI))
            {
                return true;
            }
            appliedCategories.push_back(eVolumeDeformableBody);
        }
        else
        {
            // adding the mimic joint schema API should be treated as structural change

            for (uint32_t i = 0; i < gMimicJointAPITokenCount; i++)
            {
                if (apiSchema == mimicJointAPITokens[i])
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        return true;
                    }
                    // Removing the mimic joint API is not treated as structural change for now, thus appliedCategories
                    // is not used here.
                }
            }

            // NewtonMimicAPI is single-apply; add/remove is treated as structural change.
            if (apiSchema == tok.NewtonMimicAPI)
            {
                if (!(categories & SchemaAPIFlag::eNewtonMimicAPI))
                {
                    return true;
                }
            }
        }
    }

    if (categories & SchemaAPIFlag::eRigidBodyAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eBody) == appliedCategories.end())
            return true;
    }

    if (categories & SchemaAPIFlag::eCollisionAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eShape) == appliedCategories.end())
            return true;
    }

    if (categories & SchemaAPIFlag::ePhysxForceAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), ePhysxForce) == appliedCategories.end())
            return true;
    }

    if (categories & SchemaAPIFlag::eDeformableBodyAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eVolumeDeformableBody) == appliedCategories.end())
            return true;
    }

    if (categories & SchemaAPIFlag::eSurfaceDeformableSimAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eSurfaceDeformableBody) == appliedCategories.end())
        {
            resyncKey = src->getParent(key);
            return true;
        }
    }

    if (categories & SchemaAPIFlag::eVolumeDeformableSimAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eVolumeDeformableBody) == appliedCategories.end())
        {
            resyncKey = src->getParent(key);
            return true;
        }
    }

    if (categories & SchemaAPIFlag::eParticleSetAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eParticleSet) == appliedCategories.end())
            return true;
    }

    return false;
}

void processNonstructuralSchemaAPIChanges(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key,
    const std::vector<omni::physics::parse::TokenId>& apiSchemas, uint32_t categories)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    const omni::physics::parse::TokenId mimicJointAPITokens[] = {
        tok.physxMimicJointAPIRotX, tok.physxMimicJointAPIRotY, tok.physxMimicJointAPIRotZ
    };
    const omni::physics::parse::TokenId drivePerfEnvAPITokens[] = {
        tok.physxDrivePerformanceEnvelopeAPIAngular, tok.physxDrivePerformanceEnvelopeAPILinear,
        tok.physxDrivePerformanceEnvelopeAPIRotX, tok.physxDrivePerformanceEnvelopeAPIRotY,
        tok.physxDrivePerformanceEnvelopeAPIRotZ
    };
    const omni::physics::parse::TokenId jointAxisAPITokens[] = {
        tok.physxJointAxisAPIAngular, tok.physxJointAxisAPILinear,
        tok.physxJointAxisAPIRotX, tok.physxJointAxisAPIRotY, tok.physxJointAxisAPIRotZ
    };

    std::vector<SchemaAPIFlag::Enum> appliedCategories;

    // iterate over added APIs & create if not existing
    for (const omni::physics::parse::TokenId apiSchema : apiSchemas)
    {
        if (apiSchema == tok.physxParticleIsosurfaceAPI)
        {
            if (!(categories & SchemaAPIFlag::eParticleIsosurfaceAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eParticleIsosurfaceAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleIsosurfaceAPI);
        }
        else if (apiSchema == tok.physxParticleAnisotropyAPI)
        {
            if (!(categories & SchemaAPIFlag::eParticleAnisotropyAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eParticleAnisotropyAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleAnisotropyAPI);
        }
        else if (apiSchema == tok.physxParticleSmoothingAPI)
        {
            if (!(categories & SchemaAPIFlag::eParticleSmoothingAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eParticleSmoothingAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleSmoothingAPI);
        }
        else if (apiSchema == tok.physxDiffuseParticlesAPI)
        {
            if (!(categories & SchemaAPIFlag::eDiffuseParticlesAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eDiffuseParticlesAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eDiffuseParticlesAPI);
        }
        else if (apiSchema == tok.physicsFilteredPairsAPI)
        {
            if (!(categories & SchemaAPIFlag::eFilteredPairsAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eFilteredPairsAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eFilteredPairsAPI);
        }
        else if (apiSchema == tok.physxContactReportAPI)
        {
            if (!(categories & SchemaAPIFlag::eContactReportAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eContactReportAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eContactReportAPI);
        }
        else
        {
            for (uint32_t i = 0; i < gMimicJointAPITokenCount; i++)
            {
                if (apiSchema == mimicJointAPITokens[i])
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

                    // note: adding is treated as structural change for now, thus no call to
                    // attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, schemaAPIFlag, false);

                    appliedCategories.push_back(schemaAPIFlag);
                }
            }

            if (apiSchema == tok.NewtonMimicAPI)
            {
                // note: adding is treated as structural change, thus no call to
                // attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eNewtonMimicAPI, false);

                appliedCategories.push_back(SchemaAPIFlag::eNewtonMimicAPI);
            }

            const uint32_t nbPerfEnvTokens = sizeof(drivePerfEnvAPITokens)/sizeof(drivePerfEnvAPITokens[0]);
            for (uint32_t i = 0; i < nbPerfEnvTokens; i++)
            {
                if (apiSchema == drivePerfEnvAPITokens[i])
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gDrivePerformanceEnvelopAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, schemaAPIFlag, false);
                    }
                    appliedCategories.push_back(schemaAPIFlag);
                }
            }

            const uint32_t nbJointAxisTokens = sizeof(jointAxisAPITokens)/sizeof(jointAxisAPITokens[0]);
            for (uint32_t i = 0; i < nbJointAxisTokens; i++)
            {
                if (apiSchema == jointAxisAPITokens[i])
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gJointAxisAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, schemaAPIFlag, false);
                    }
                    appliedCategories.push_back(schemaAPIFlag);
                }
            }
        }
    }

    // check for removed APIs
    if (categories & SchemaAPIFlag::eParticleIsosurfaceAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eParticleIsosurfaceAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eParticleIsosurfaceAPI, true);
    }

    if (categories & SchemaAPIFlag::eParticleAnisotropyAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eParticleAnisotropyAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eParticleAnisotropyAPI, true);
    }

    if (categories & SchemaAPIFlag::eParticleSmoothingAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eParticleSmoothingAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eParticleSmoothingAPI, true);
    }

    if (categories & SchemaAPIFlag::eDiffuseParticlesAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eDiffuseParticlesAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eDiffuseParticlesAPI, true);
    }

    if (categories & SchemaAPIFlag::eFilteredPairsAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eFilteredPairsAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eFilteredPairsAPI, true);
    }

    if (categories & SchemaAPIFlag::eContactReportAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eContactReportAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eContactReportAPI, true);
    }

    for (uint32_t i = 0; i < gMimicJointAPITokenCount; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, schemaAPIFlag, true);
        }
    }

    if (categories & SchemaAPIFlag::eNewtonMimicAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eNewtonMimicAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, SchemaAPIFlag::eNewtonMimicAPI, true);
    }

    const uint32_t nbPerfEnvTokens = sizeof(gDrivePerformanceEnvelopAPIFlagList)/sizeof(gDrivePerformanceEnvelopAPIFlagList[0]);
    for (uint32_t i = 0; i < nbPerfEnvTokens; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gDrivePerformanceEnvelopAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, schemaAPIFlag, true);
        }
    }


    const uint32_t nbJointAxisTokens = sizeof(gJointAxisAPIFlagList)/sizeof(gJointAxisAPIFlagList[0]);
    for (uint32_t i = 0; i < nbJointAxisTokens; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gJointAxisAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, key, schemaAPIFlag, true);
        }
    }

}

bool PrimChangeMap::getPropertyChange(omni::physics::parse::TokenId token, PropertyChangeMap::const_iterator& iterator, PropertyChangeMap::const_iterator& outItEnd) const
{
    PropertyChangeMap::const_iterator itChange = m_propertyChanges.find(token);
    PropertyChangeMap::const_iterator itEnd = m_propertyChanges.end();
    if (itChange != itEnd)
    {
        iterator = itChange;
        outItEnd = itEnd;
        return true;
    }
    else
    {
        // look into stage-specific changemap if nothing found on persistent
        itChange = m_stageSpecificChanges.find(token);
        itEnd = m_stageSpecificChanges.end();
        if (itChange != itEnd)
        {
            iterator = itChange;
            outItEnd = itEnd;
            return true;
        }
    }

    return false;
}

// Dispatch through PropertyChangeMap; every registered entry carries a check function
// (onPrimCheckKey/onPrimCheckExtKey -- see ChangeRegister.cpp's REGISTER_CHANGE macros).
void PrimChangeMap::checkPrimChange(AttachedStage& attachedStage,
                                    omni::physics::parse::ObjectKey primKey,
                                    omni::physics::parse::TokenId propertyTokenId)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    const bool deferUpdates = usdLoad->getAsyncUSDUpdate();
    PropertyChangeMap::const_iterator itChange;
    PropertyChangeMap::const_iterator itChangeEnd;
    bool propertyChangeFound = getPropertyChange(propertyTokenId, itChange, itChangeEnd);
    omni::physics::parse::ObjectKey resyncKey;
    bool structuralChange = false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();

    while (propertyChangeFound && itChange != itChangeEnd && itChange->first == propertyTokenId)
    {
        const PropertyChange& change = itChange->second;
        const bool primCheck = change.onPrimCheckKey && change.onPrimCheckKey(attachedStage, primKey, propertyTokenId);
        bool primCheckExt = true;
        if (primCheck && change.onPrimCheckExtKey != nullptr)
            primCheckExt = change.onPrimCheckExtKey(attachedStage, primKey, propertyTokenId, resyncKey);
        if (primCheck && primCheckExt)
        {
            // check for structural change, onUpdate is not defined and resyncKey not set
            // we will recreate the objects
            if (!change.onUpdate || resyncKey.valid())
            {
                structuralChange = true;
                break;
            }

            // Cooked-geometry carrier (ADR-0022): inert on USD-backed attaches (the carrier is
            // only ever populated when getDataWrite() == nullptr).
            if (src && internal::physxtools_detail::isCookedGeometryAttribute(src->tokenToString(propertyTokenId)))
                internal::clearCookedValue(attachedStage, primKey, propertyTokenId);

            if (!deferUpdates)
            {
                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
                if (entries && !entries->empty())
                {
                    ObjectIdMap::const_iterator it = entries->begin();
                    while (it != entries->end())
                    {
                        change.onUpdate(attachedStage, it->second, propertyTokenId, omni::physics::parse::ReadTime::defaultTime());
                        it++;
                    }
                }
                return;
            }
            else
            {
                bool attributeSet = false;
                KeyChangeMap::iterator it = m_keyChangeMap.find(primKey);
                while (it != m_keyChangeMap.end() && it->first == primKey)
                {
                    ChangeData& changeData = it->second;
                    if (changeData.second == propertyTokenId)
                    {
                        attributeSet = true;
                        break;
                    }
                    it++;
                }

                if (!attributeSet)
                {
                    m_keyChangeMap.insert(std::pair<omni::physics::parse::ObjectKey, ChangeData>(
                        primKey, { change.onUpdate, propertyTokenId }));
                }
                return;
            }
        }

        itChange++;
    }

    if (!structuralChange)
    {
        // Handle transform changes. USD xform-op attributes are recognized by
        // isTransformationAffectedByAttrNamed(); ovstage/fabric transform columns are
        // routed here too via the four KnownTokens members.
        if ((src && isTransformationAffectedByAttrNamed(src->tokenToString(propertyTokenId))) ||
            propertyTokenId == tok.omniXform || propertyTokenId == tok.omniResetXformStack ||
            propertyTokenId == tok.omniFabricLocalMatrix || propertyTokenId == tok.omniFabricWorldMatrix)
        {
            if (!deferUpdates)
            {
                addTransformChange(primKey);
            }
            else
                m_transformKeyUpdates.push_back(primKey);
            return;
        }

        // If an API schema was added/remove, check whether this triggers a structural change
        if (propertyTokenId == tok.apiSchemas)
        {
            // Guard against expired/invalid prims (source-routed validity, no UsdPrim).
            if (!src || !src->exists(primKey))
                return;
            std::vector<omni::physics::parse::TokenId> apiSchemas;
            src->forEachAppliedSchema(primKey, [&](omni::physics::parse::TokenId t) { apiSchemas.push_back(t); });
            const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
            structuralChange = checkForStructuralSchemaAPIChanges(attachedStage, primKey, apiSchemas, storedAPIs, resyncKey);
            if (!structuralChange) // handle API changes that don't trigger structural changes here
                processNonstructuralSchemaAPIChanges(attachedStage, primKey, apiSchemas, storedAPIs);
        }
        else if (propertyTokenId == tok.inactiveIds)
        {
            structuralChange = true;
        }
        else if (propertyTokenId == tok.physicsBody0 || propertyTokenId == tok.physicsBody1)
        {
            // A joint body relationship re-target requires recreating the joint
            // (a PxJoint's actors are immutable). ovstage surfaces this as a named
            // value change with no per-property update handler, so flag it
            // structural here to force a re-parse.
            structuralChange = true;
        }
    }

    // check if we need to reconstruct the prim
    if (structuralChange)
    {
        const omni::physics::parse::ObjectKey resyncTarget = resyncKey.valid() ? resyncKey : primKey;
        // Guard against expired/invalid prims (source-routed validity, no UsdPrim).
        if (!src || !src->exists(resyncTarget))
            return;
        PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
        PrimHierarchyStorage::Iterator iterator(primStorage, std::string(attachedStage.textFor(resyncTarget)));
        for (size_t i = iterator.getDescendentsPaths().size(); i--;)
        {
            const omni::physics::parse::ObjectKey childKey = attachedStage.keyFor(iterator.getDescendentsPaths()[i]);
            handleRemovedPrim(attachedStage, childKey);
        }
        primStorage.removeIteration(iterator);
        attachedStage.getPrimUpdateMap().addPrim(attachedStage, resyncTarget);
    }
}

void PrimChangeMap::clearMap()
{
    m_keyChangeMap.clear();
    m_keyTransformChangesSet.clear();
}

// Drops any pending m_keyChangeMap entry for this key -- see PrimUpdate.h's own comment on
// KeyChangeMap. A single find+erase (not all matching entries -- established behavior, not
// something to opportunistically fix here).
void PrimChangeMap::removePrim(omni::physics::parse::ObjectKey key)
{
    KeyChangeMap::iterator it = m_keyChangeMap.find(key);
    if (it != m_keyChangeMap.end())
        m_keyChangeMap.erase(it);
}

void PrimChangeMap::processTransformUpdates(AttachedStage& attachedStage)
{
    if (!m_transformKeyUpdates.empty())
    {
        for (size_t i = 0; i < m_transformKeyUpdates.size(); i++)
        {
            handleTransformChange(attachedStage, m_transformKeyUpdates[i]);
        }

        m_transformKeyUpdates.clear();
    }
}

void PrimChangeMap::processTransformChanges(AttachedStage& attachedStage)
{
    for (PrimKeySet::const_reference key : m_keyTransformChangesSet)
    {
        handleTransformChange(attachedStage, key);
    }
    m_keyTransformChangesSet.clear();
}

bool PrimUpdateMap::needsSceneReset(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    return source && source->isA(key, attachedStage.getKnownTokens().physicsScene);
}

void PrimUpdateMap::addPrim(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    if (m_isNewScene)
        return;

    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;

    if (!isInPrimAddMap(attachedStage, key))
    {
        // Nothing to demote from an empty map; skipping the walk spares a live hierarchy
        // query per new leaf on the drain path.
        if (!m_primAddMap.empty())
        {
            std::vector<omni::physics::parse::ObjectKey> descendants;
            source->forEachDescendant(key,
                                      [&descendants](omni::physics::parse::ObjectKey k) { descendants.push_back(k); });
            for (const omni::physics::parse::ObjectKey childKey : descendants)
            {
                m_primAddMap.erase(childKey);
            }
        }
        m_primAddMap.insert(key);
        if (needsSceneReset(attachedStage, key))
            UsdLoad::getUsdLoad()->releasePhysicsObjects(attachedStage.getStageId());
    }
}

void PrimUpdateMap::removePrim(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey)
{
    m_primAddMap.erase(primKey);

    PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
    PrimHierarchyStorage::Iterator iterator(primStorage, std::string(attachedStage.textFor(primKey)));
    for (size_t i = iterator.getDescendentsPaths().size(); i--;)
    {
        const omni::physics::parse::ObjectKey childKey = attachedStage.keyFor(iterator.getDescendentsPaths()[i]);
        handleRemovedPrim(attachedStage, childKey);
    }
    handleRemovedPrim(attachedStage, primKey);
    primStorage.removeIteration(iterator);
}

bool PrimUpdateMap::isInPrimAddMap(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key) const
{
    if (m_isNewScene)
        return true;

    if (m_primAddMap.empty())
        return false;

    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;

    // Walk `key` (inclusive) up its ancestors through the source; a hit means an
    // ancestor (or the object itself) is already queued for re-parse.
    const omni::physics::parse::ObjectKey root = source->getRootKey();
    for (omni::physics::parse::ObjectKey p = key; p.valid() && p != root; p = source->getParent(p))
    {
        if (m_primAddMap.find(p) != m_primAddMap.end())
            return true;
    }

    return false;
}

// remove invalid (no-longer-present) keys
void PrimUpdateMap::checkMap(const AttachedStage& attachedStage)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    for (PrimKeySet::iterator it = m_primAddMap.begin(); it != m_primAddMap.end();)
    {
        if (!source || !source->exists(*it))
            it = m_primAddMap.erase(it);
        else
            ++it;
    }
}

void processChangeMap(AttachedStage& attachedStage)
{
    // Guards both drains below against re-entrant dispatch while applying a
    // deferred update (same ChangeSource::eUsd marker onSourceChange itself
    // uses for every source, not just a literal USD one).
    auto changeSourceBlock = attachedStage.getChangeSourceBlock(ChangeSource::eUsd);

    KeyChangeMap::const_iterator itKeyCh = attachedStage.getPrimChangeMap().getKeyMap().begin();
    KeyChangeMap::const_iterator itKeyChEnd = attachedStage.getPrimChangeMap().getKeyMap().end();
    while (itKeyCh != itKeyChEnd)
    {
        const omni::physics::parse::ObjectKey primKey = itKeyCh->first;
        const ChangeData& changeData = itKeyCh->second;

        const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
        if (entries && !entries->empty())
        {
            auto it = entries->begin();
            while (it != entries->end())
            {
                changeData.first(attachedStage, it->second, changeData.second, omni::physics::parse::ReadTime::defaultTime());
                it++;
            }
        }
        itKeyCh++;
    }

    attachedStage.getPrimChangeMap().processTransformUpdates(attachedStage);
    attachedStage.getPrimChangeMap().clearMap();
}

void flushBufferedChanges(AttachedStage& attachedStage, float currentTime)
{
    // No stage guard: nothing below reads USD, and this is the body of the public
    // IPhysxSimulation::flushChanges(), which was a silent no-op on a stageless attach.

    if (attachedStage.getPrimUpdateMap().isEmptyScene())
    {
        attachedStage.getPrimUpdateMap().clearMap();
        attachedStage.getPrimChangeMap().clearMap();

        if (attachedStage.isReplicatorStage())
        {
            // Two identities, deliberately: the replicator is *registered* under an attach handle
            // (ADR-0016), while PhysXReplicator::attach still takes a stage id because on the USD
            // path that call performs the attach itself and there is no AttachedStage to name yet.
            //
            // The lookup is guarded because it can legitimately miss: isReplicatorStage() records
            // that this attach was set up by a replicator, but the registration is unregistered
            // independently (the clone path's RAII guard, or an explicit unregisterReplicator), so
            // the flag can outlive the entry. Dereferencing unguarded here crashed the flush.
            if (PhysXReplicator* replicator =
                    OmniPhysX::getInstance().getReplicator(attachedStage.getAttachHandle()))
            {
                replicator->attach(attachedStage.getStageId(), attachedStage.getPhysXPhysicsInterface(), false);
            }
            else
            {
                CARB_LOG_ERROR(
                    "flushBufferedChanges: no replicator is registered for this attach; the buffered "
                    "changes for a replicator stage cannot be flushed.");
            }
        }
        else
        {
            // NOTE: deliberately NOT wrapped in InitialStagePopulationScope. This
            // empty-scene branch also handles re-population after
            // releasePhysicsObjects() during a running simulation, whose creation
            // notifications subscribers rely on; the "initial population is not
            // notified" contract only covers attach/update traversal.
            loadFromStage(attachedStage);
        }
        attachedStage.getPrimUpdateMap().setEmptyScene(false);
    }

    // don't allow updates while processing the current batch
    // that would modify g_PrimUpdateMap while we are iterating over the contents
    if (!attachedStage.getPrimUpdateMap().getMap().empty())
    {
        UsdLoad::getUsdLoad()->blockUSDUpdate(true);

        attachedStage.getPrimUpdateMap().checkMap(attachedStage);

        // loadPhysicsFromPrimitive() takes source-path strings; convert at this boundary.
        std::vector<std::string> updateRoots;
        updateRoots.reserve(attachedStage.getPrimUpdateMap().getMap().size());
        for (const omni::physics::parse::ObjectKey primKey : attachedStage.getPrimUpdateMap().getMap())
            updateRoots.push_back(std::string(attachedStage.textFor(primKey)));
        loadPhysicsFromPrimitive(attachedStage, updateRoots);

        // it is safe to allow updates again
        UsdLoad::getUsdLoad()->blockUSDUpdate(false);

        attachedStage.getPrimUpdateMap().clearMap();
    }

    processChangeMap(attachedStage);
}

void processUpdates(AttachedStage& attachedStage, float currentTime)
{
    CARB_PROFILE_ZONE(0, "physx::usdparser::processUpdates");

    // The time base comes from the source alongside the other stage units, so both
    // loops below -- animated kinematic bodies (populated from the source,
    // PhysicsBody.cpp) and time-sampled attributes (ChangeRegister.cpp) -- are
    // reachable with no backing stage. Reproduces UsdTimeCode's NaN-as-Default()
    // sentinel (physxtools_detail::toReadTime) without constructing a UsdTimeCode.
    const double timeCodeValue = double(currentTime) * attachedStage.getSourceUnits().timeCodesPerSecond;
    const omni::physics::parse::ReadTime readTime = std::isnan(timeCodeValue) ?
        omni::physics::parse::ReadTime::defaultTime() : omni::physics::parse::ReadTime::at(timeCodeValue);

    if (!attachedStage.getAnimatedKinematicBodies().empty())
    {
        CARB_PROFILE_ZONE(0, "KinematicBodiesUpdate");
        for (const omni::physics::parse::ObjectKey& bodyKey : attachedStage.getAnimatedKinematicBodies())
        {
            if (!bodyKey.valid())
                continue;

            // Time-aware world transform at the current sim frame, read
            // through the source (per-call; source-side caching can be
            // reintroduced to restore the previous xform-cache amortisation).
            const ::physx::PxMat44d localToWorld =
                internal::getWorldTransform(attachedStage, bodyKey, readTime);

            ::physx::PxTransform pose;
            ::physx::PxVec3 sc;
            decomposeMatrix(pose, sc, localToWorld);

            PhysXUsdPhysicsInterface::Transform fcTransform;

            fcTransform.position = toFloat3(pose.p);
            fcTransform.orientation = toFloat4(pose.q.getNormalized());
            fcTransform.scale = toFloat3(sc);


            const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(bodyKey);
            if (entries && !entries->empty())
            {
                auto it = entries->begin();
                while (it != entries->end())
                {
                    attachedStage.getPhysXPhysicsInterface()->updateTransform(attachedStage,
                        bodyKey, it->second, fcTransform);
                    it++;
                }
            }
        }
    }

    if (!attachedStage.getTimeSampleMap().empty())
    {
        CARB_PROFILE_ZONE(0, "KinematicAttributesUpdate");
        for (auto& iterator : attachedStage.getTimeSampleMap())
        {
            const omni::physics::parse::ObjectKey primKey = iterator.first.primKey;
            const omni::physics::parse::TokenId attrTokenId = iterator.first.attr;

            const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
            if (entries && !entries->empty())
            {
                auto it = entries->begin();
                while (it != entries->end())
                {
                    attachedStage.getPhysXPhysicsInterface()->updateObject(attachedStage, primKey, it->second, iterator.second, attrTokenId, readTime);
                    it++;
                }
            }
        }
    }

    flushBufferedChanges(attachedStage, currentTime);
}

// Per-batch change-feed consumer (ADR-0003). Reproduces the
// per-path dispatch of the legacy UsdNotice::ObjectsChanged handler, driven by a
// single ChangeBatch instead of the raw notice. The USD feed (UsdChangeFeed)
// classifies each notice into batches:
//   - isDelete            : prim removed                  → removePrim
//   - invalid property    : structural resync; changed-field tokens in `values`
//                           → reproduce the typeName/apiSchemas/kind tree
//   - valid property      : value change                 → checkPrimChange
// Returns false ONLY when the ovstage drain committed part of a value batch and then a scatter failed: the
// caller (the feed's drainRange) must then hold the cursor so the batch is retried, and must NOT let the
// external ordinal advance. Every other outcome -- structural change handled, value applied, or fell back to
// the parse path -- returns true.
bool onSourceChange(AttachedStage& attachedStage, const omni::physics::parse::ChangeBatch& batch)
{
    CARB_PROFILE_ZONE(0, "physx::usdparser::onSourceChange");

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    if (usdLoad->usdUpdateIsBlocked())
        return true;

    // Early exit if scene is empty; we will parse the whole scene.
    if (attachedStage.getPrimUpdateMap().isEmptyScene())
        return true;

    // No stage guard. This is registered as the wildcard change consumer on the
    // ovstage branch of AttachedStage::rebuildSource(), so guarding it discarded every
    // incremental ovstage change on a stageless attach -- and did so SILENTLY:
    // updateFromOvStage still returned true and the external read ordinal still
    // advanced past the dropped batch, making the loss permanent rather than deferred.
    // That is ovphysx's whole data path after initial load. The function already
    // checks the thing it actually needs (getSource()) below.
    CARB_PROFILE_ZONE(0, "PhysicsUsdNoticleListener");

    auto changeSourceBlock = attachedStage.getChangeSourceBlock(ChangeSource::eUsd);

    if (batch.keys.count == 0 || !batch.keys.data)
        return true;
    const omni::physics::parse::ObjectKey* keys =
        static_cast<const omni::physics::parse::ObjectKey*>(batch.keys.data);

    // Removal (resync, prim removed or deactivated).
    if (batch.isDelete)
    {
        for (size_t i = 0; i < batch.keys.count; ++i)
        {
            attachedStage.getPrimUpdateMap().removePrim(attachedStage, keys[i]);
        }
        return true;
    }

    // Structural resync: invalid property, changed-field tokens carried in `values`.
    if (!batch.property.valid())
    {
        const omni::physics::parse::IPhysicsSource* psrc = attachedStage.getSource();
        if (!psrc)
            return true;
        const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();

        // Resync field tokens are source-space TokenIds naming the changed metadata
        // field (typeName / apiSchemas / kind). Intern the three field names once
        // per batch and compare TokenId to TokenId — an integer compare rather
        // than a per-field TfToken round trip through the source's intern table
        // (mirrors the applied-schema hot-path pattern in LoadStage.cpp's
        // gatherSourceOnlySideEffects).
        const omni::physics::parse::TokenId typeNameFieldId = psrc->internToken("typeName");
        const omni::physics::parse::TokenId apiSchemasFieldId = psrc->internToken("apiSchemas");
        const omni::physics::parse::TokenId kindFieldId = psrc->internToken("kind");
        bool typeNameChange = false, apiSchemasChange = false, kindChange = false;
        const omni::physics::parse::TokenId* fields =
            static_cast<const omni::physics::parse::TokenId*>(batch.values.data);
        for (size_t i = 0; i < batch.values.count; ++i)
        {
            const omni::physics::parse::TokenId f = fields[i];
            if (f == typeNameFieldId) typeNameChange = true;
            else if (f == apiSchemasFieldId) apiSchemasChange = true;
            else if (f == kindFieldId) kindChange = true;
        }
        for (size_t i = 0; i < batch.keys.count; ++i)
        {
            const omni::physics::parse::ObjectKey key = keys[i];
            if (!psrc->exists(key))
            {
                continue;
            }

            if (!typeNameChange && apiSchemasChange)
            {
                if (!attachedStage.getPrimUpdateMap().isInPrimAddMap(attachedStage, key))
                {
                    attachedStage.getPrimChangeMap().checkPrimChange(attachedStage, key, tok.apiSchemas);
                }
            }
            else if (!typeNameChange && kindChange)
            {
                // We ignore 'kind' changes
            }
            else
            {
                if (key == psrc->getRootKey())
                {
                    usdLoad->releasePhysicsObjects(static_cast<uint64_t>(attachedStage.getStageId()));
                }
                else if (!psrc->isPrototype(key))
                {
                    attachedStage.getPrimUpdateMap().removePrim(attachedStage, key);
                    attachedStage.getPrimUpdateMap().addPrim(attachedStage, key);
                }
            }
        }
        return true;
    }

    // Value change. Scatter it through the write backend (columnar, reusing the write session's
    // planGroup + scatterGroup) for every attribute that backend covers -- this replaces the legacy
    // per-object host-scalar apply for the ovstage path. applyOvstageValueBatch reports NotHandled for a
    // structural change or an attribute with no backend setter (transforms, apiSchemas, per-shape
    // rows), which falls through to the per-object property-change path below unchanged.
    omni::physics::parse::IPhysicsSource* psrc = attachedStage.getSource();
    if (!psrc)
        return true;

    // The drain mutates PhysX immediately, so it must not run while USD-update deferral is active (an async
    // simulation step is in flight and value changes are meant to be queued for processChanges). The ovstage
    // drain reaches here only via update_from_ovstage, which waits for all pending ops first, so async is
    // already off -- the guard just makes that invariant explicit and keeps it should UsdChangeFeed ever
    // start delivering a value column (a USD-fed value batch would then defer correctly rather than apply
    // mid-step).
    const UsdLoad* ul = UsdLoad::getUsdLoad();
    if (ul && !ul->getAsyncUSDUpdate())
    {
        // Passing a sink for the rows the drain skips opts into PARTIAL handling: the drain scatters the
        // keys it can service and reports the rest (a kinematic body, a local-frame velocity, a
        // not-yet-created body, a link an attribute can't take), which still need the per-object path
        // below. Splitting rather than falling back whole keeps the drain's DirectGPU-correct path for
        // every serviceable key in a mixed batch -- on DirectGPU the per-object velocity setter is inert,
        // so a whole-batch fallback would silently drop the serviceable bodies. In the common all-resolved
        // batch nothing is appended, so the vector makes no heap allocation.
        std::vector<uint32_t> unresolvedRows;
        const omni::physx::DrainResult drained =
            omni::physx::applyOvstageValueBatch(*psrc, batch, &unresolvedRows);
        // A partial commit followed by a scatter failure: the parse-path fallback would re-apply the rows
        // that DID reach the solver (a double-apply), so we must NOT fall through. Signal failure so
        // drainRange holds the cursor and the whole batch is retried on the next drain.
        if (drained == omni::physx::DrainResult::Failed)
            return false;
        if (drained == omni::physx::DrainResult::Applied)
        {
            // Only the rows the drain could not service (never committed, so no double-apply) still go
            // through the per-object property path.
            for (uint32_t i : unresolvedRows)
                attachedStage.getPrimChangeMap().checkPrimChange(attachedStage, keys[i], batch.property);
            return true;
        }
        // NotHandled: nothing was committed, fall through to the parse-layer property path for all keys.
    }

    for (size_t i = 0; i < batch.keys.count; ++i)
    {
        attachedStage.getPrimChangeMap().checkPrimChange(attachedStage, keys[i], batch.property);
    }
    return true;
}

// Per-group finalization: flush the transform changes accumulated across this
// notice's batches, exactly once — mirroring the single processTransformChanges()
// call at the tail of the legacy handler. Honors the same guards so nothing is
// flushed when the whole notice would have been skipped.
void onSourceGroupComplete(AttachedStage& attachedStage)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    if (usdLoad->usdUpdateIsBlocked())
        return;
    if (attachedStage.getPrimUpdateMap().isEmptyScene())
        return;
    auto changeSourceBlock = attachedStage.getChangeSourceBlock(ChangeSource::eUsd);
    attachedStage.getPrimChangeMap().processTransformChanges(attachedStage);
}


} // namespace usdparser
} // namespace physx
} // namespace omni
