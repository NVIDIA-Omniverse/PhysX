// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/Defines.h>
#include <carb/profiler/Profile.h>
#include <carb/logging/Log.h>

#include <omni/core/ITypeFactory.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "PrimUpdate.h"
#include "LoadStage.h"
#include "Collision.h"
#include "NewtonCompat.h"
#include <OmniPhysX.h>

#include <UsdSource.h>
#include <omni/physics/parse/IChangeFeed.h>
#include <PhysXTools.h>

#include <omni/physx/PhysxTokens.h>

using namespace PXR_NS;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

namespace
{

} // namespace

template <typename T>
bool checkSchemaAttribute(const PXR_NS::TfToken& attributeName)
{
    const TfTokenVector& apiAttributes = T::GetSchemaAttributeNames();
    for (size_t i = apiAttributes.size(); i--;)
    {
        if (apiAttributes[i] == attributeName)
            return true;
    }

    return false;
}

bool isMovableBody(const AttachedStage& attachedStage, const PXR_NS::SdfPath& primKey)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
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

bool isNonMovable(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKeyObj, PXR_NS::SdfPath& resyncPath)
{
    // Fully source-routed (no UsdPrim): the schema-API flags are keyed by path,
    // and the parent / subtree walks go through IPhysicsSource. The resync target
    // is reported as a path (the caller feeds it to PrimUpdateMap::addPrim).
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const ObjectDb* objectDb = attachedStage.getObjectDatabase();

    const PXR_NS::SdfPath primPath = attachedStage.pathFor(primKeyObj);
    const uint64_t primAPIs = objectDb->getSchemaAPIs(primPath);
    if (primAPIs & SchemaAPIFlag::eDeformableBodyAPI)
    {
        resyncPath = primPath;
        return true;
    }

    // if we changed the transform of a shape or a transform between the
    // body and the shape we cannot move that, we reconstruct
    const omni::physics::parse::ObjectKey root = src->getRootKey();
    for (omni::physics::parse::ObjectKey parent = src->getParent(primKeyObj);
         parent.valid() && parent != root; parent = src->getParent(parent))
    {
        const uint64_t parentAPIs = objectDb->getSchemaAPIs(attachedStage.pathFor(parent));
        if (parentAPIs & SchemaAPIFlag::eRigidBodyAPI)
        {
            if (primAPIs & SchemaAPIFlag::eCollisionAPI)
            {
                resyncPath = primPath;
                return true;
            }
            // Scan the subtree (root inclusive, all prims) for any collider.
            bool foundCollision = false;
            src->forEachDescendant(primKeyObj,
                [&](omni::physics::parse::ObjectKey k)
                {
                    if (objectDb->getSchemaAPIs(attachedStage.pathFor(k)) & SchemaAPIFlag::eCollisionAPI)
                        foundCollision = true;
                });
            if (foundCollision)
            {
                resyncPath = primPath;
                return true;
            }
            return false;
        }
        else if (parentAPIs & SchemaAPIFlag::eDeformableBodyAPI)
        {
            resyncPath = attachedStage.pathFor(parent);
            return true;
        }
    }
    return false;
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
            src->forEachDescendant(primKeyObj,
                [&](omni::physics::parse::ObjectKey k)
                {
                    if (objectDb->getSchemaAPIs(k) & SchemaAPIFlag::eCollisionAPI)
                        foundCollision = true;
                });
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

void handleRemovedPrim(AttachedStage& attachedStage, const SdfPath& primKey)
{
    attachedStage.getPrimChangeMap().removePrim(primKey);

    if (!attachedStage.getStage())
        return;

    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            attachedStage.getPhysXPhysicsInterface()->releaseObject(attachedStage, primKey, it->second);
            it++;
        }

        attachedStage.getObjectDatabase()->removeEntries(primKey);
        attachedStage.getObjectDatabase()->removeSchemaAPIs(primKey);
    }
}

SdfPath findDeformableBodyAncestor(AttachedStage& attachedStage, const SdfPath primKey)
{
    // Walk ancestors through the source (no UsdPrim): stop at the nearest
    // DeformableBody, bail at a RigidBody, and stop at a prim that resets the
    // xform stack (its subtree is detached from the parent frame).
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return SdfPath();
    const omni::physics::parse::ObjectKey root = src->getRootKey();
    omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    while (key.valid() && key != root)
    {
        if (internal::hasAppliedSchema(*src, key, OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI))
            return attachedStage.pathFor(key);
        if (internal::hasAppliedSchema<PXR_NS::UsdPhysicsRigidBodyAPI>(*src, key))
            return SdfPath();

        omni::physics::parse::Matrix4d local;
        bool resetXformStack = false;
        src->getLocalTransform(key, omni::physics::parse::ReadTime::defaultTime(), local, resetXformStack);
        if (resetXformStack)
            return SdfPath();
        key = src->getParent(key);
    }
    return SdfPath();
}

PrimChangeMap::PrimChangeMap()
{
}

PrimChangeMap::~PrimChangeMap()
{
    m_propertyChanges.clear();
}

void PrimChangeMap::registerPrimChange(const ChangeParams& changeParam)
{
    CARB_ASSERT(!changeParam.changeAttribute.empty());
    PropertyChange pc;
    pc.onPrimCheck = changeParam.onPrimCheck;
    pc.onPrimCheckExt = changeParam.onPrimCheckExt;
    pc.onUpdate = changeParam.onUpdate;
    pc.onPrimCheckKey = changeParam.onPrimCheckKey;
    m_propertyChanges.insert(std::pair<PXR_NS::TfToken, PropertyChange>(TfToken(changeParam.changeAttribute.c_str()), pc));
}

void PrimChangeMap::clearRegisteredChanges()
{
    m_propertyChanges.clear();
}

void PrimChangeMap::registerStageSpecificChange(const ChangeParams& changeParam)
{
    PropertyChange pc;
    pc.onPrimCheck = changeParam.onPrimCheck;
    pc.onPrimCheckExt = changeParam.onPrimCheckExt;
    pc.onUpdate = changeParam.onUpdate;
    pc.onPrimCheckKey = changeParam.onPrimCheckKey;
    m_stageSpecificChanges.insert(std::pair<PXR_NS::TfToken, PropertyChange>(TfToken(changeParam.changeAttribute.c_str()), pc));
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
        const GfMatrix4d mat = internal::getWorldTransform(attachedStage, key, UsdTimeCode::Default());
        const GfTransform tr(mat);
        const GfVec3d pos = tr.GetTranslation();
        const GfQuatd rot = tr.GetRotation().GetQuat();
        const GfVec3d sc = tr.GetScale();

        fcTransform.position = { float(pos[0]), float(pos[1]), float(pos[2]) };
        fcTransform.orientation = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]),
                                    float(rot.GetImaginary()[2]), float(rot.GetReal()) };
        fcTransform.scale = { float(sc[0]), float(sc[1]), float(sc[2]) };
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

            PXR_NS::SdfPath primKey = attachedStage.pathFor(key);
            if (primKey.IsEmpty())
                return;
            {
                PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
                PrimHierarchyStorage::Iterator iterator(primStorage, primKey);
                for (size_t i = iterator.getDescendentsPaths().size(); i--;)
                {
                    const PXR_NS::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
                    handleRemovedPrim(attachedStage, primCPath);
                }
                primStorage.removeIteration(iterator);
                attachedStage.getPrimUpdateMap().addPrim(attachedStage, primKey);
            }
        }
    }
}

void moveBody(AttachedStage& attachedStage, const PXR_NS::SdfPath& primKey)
{
    moveBody(attachedStage, attachedStage.keyFor(primKey));
}

void movePointInstancer(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    // Prototypes relationship + per-instance arrays + the instancer world
    // transform are all read through IPhysicsSource (no UsdPrim).
    SdfPathVector targets;
    internal::getRelationshipValue(attachedStage, key, UsdGeomTokens->prototypes, targets);
    std::vector<ObjectIdMap::const_iterator> targetEntries;

    VtArray<int> indices;
    VtArray<GfVec3f> positions;
    VtArray<GfQuath> orientations;
    internal::getArrayValue<VtVec3fArray>(attachedStage, key, UsdGeomTokens->positions, UsdTimeCode::Default(), positions);
    internal::getArrayValue<VtQuathArray>(attachedStage, key, UsdGeomTokens->orientations, UsdTimeCode::Default(), orientations);
    internal::getArrayValue<VtIntArray>(attachedStage, key, UsdGeomTokens->protoIndices, UsdTimeCode::Default(), indices);

    PhysXUsdPhysicsInterface::Transform fcTransform;

    const GfMatrix4d instancerMatrix = internal::getWorldTransform(attachedStage, key, UsdTimeCode::Default());
    const GfQuatf instancerRotation = GfQuatf(instancerMatrix.ExtractRotation().GetQuat());
    const GfVec3d sc = GfTransform(instancerMatrix).GetScale();
    fcTransform.scale = { float(sc[0]), float(sc[1]), float(sc[2]) };

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
                const GfVec3f instancePos = i < positions.size() ? positions[i] : GfVec3f(0.0f);
                const GfVec3f transfPos = PXR_NS::GfVec3f(instancerMatrix.Transform(instancePos));

                const GfQuatf instanceOrient = i < orientations.size() ? GfQuatf(orientations[i]) : GfQuatf(1.0f);
                const GfQuatf transfOrient = instanceOrient * instancerRotation;

                fcTransform.position = { float(transfPos[0]), float(transfPos[1]), float(transfPos[2]) };
                fcTransform.orientation = { float(transfOrient.GetImaginary()[0]), float(transfOrient.GetImaginary()[1]),
                                            float(transfOrient.GetImaginary()[2]), float(transfOrient.GetReal()) };

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
        const SdfPath instancerPath = attachedStage.pathFor(key);
        attachedStage.getPrimUpdateMap().removePrim(attachedStage, instancerPath);
        attachedStage.getPrimUpdateMap().addPrim(attachedStage, instancerPath);
    }
}

void PrimChangeMap::handleTransformChange(AttachedStage& attachedStage, const PXR_NS::SdfPath& primKey, const PXR_NS::UsdPrim* primIn)
{
    TRACE_FUNCTION();
    if (attachedStage.getObjectDatabase()->empty())
        return;

    bool bodyMoved = false;
    if (isMovableBody(attachedStage, primKey))
    {
        bodyMoved = true;
        moveBody(attachedStage, primKey);
    }

    // Do a quick check if there is actually something in the hierarchy
    PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
    {
        PrimHierarchyStorage::Iterator iterator(primStorage, primKey);
        if (iterator.getDescendentsPaths().empty())
        {
            return;
        }
    }
    
    if (!bodyMoved || OmniPhysX::getInstance().getInternalPhysXDatabase().getNestedBodiesUsed())
    {
        // `primIn` (a caller-supplied prim hint) is no longer needed here — the
        // dispatch + descendant walk are fully source-routed by ObjectKey. The
        // parameter is retained until the remaining caller sites are decoupled.
        (void)primIn;

        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);

        // Skip invalid keys and instancing prototype roots (edits propagate via
        // the instance proxies, not the synthetic prototype prim) — no UsdPrim.
        if (!src || !src->exists(key) || src->isPrototype(key))
            return;

        if (src->isA(key, internal::schemaTypeToken<UsdGeomPointInstancer>(*src)))
        {
            movePointInstancer(attachedStage, key);
        }
        else
        {
            // check the parents
            PXR_NS::SdfPath resyncPath;
            if (isNonMovable(attachedStage, key, resyncPath))
            {
                PrimHierarchyStorage::Iterator iterator(primStorage, resyncPath);
                for (size_t i = iterator.getDescendentsPaths().size(); i--;)
                {
                    const PXR_NS::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
                    handleRemovedPrim(attachedStage, primCPath);
                }
                primStorage.removeIteration(iterator);
                attachedStage.getPrimUpdateMap().addPrim(attachedStage, resyncPath);
            }
            else
            {
                // We could have changed a parent node for more prims, lets update.
                // Default-predicate subtree walk (active/defined/loaded/non-abstract, no
                // instance proxies) routed through the source. Prune at any object that
                // resets the inherited xform stack, since its subtree is unaffected.
                src->forEachDescendantPruned(
                    key,
                    [&](omni::physics::parse::ObjectKey childKey) -> bool
                    {
                        bool resetXformOpStack = false;
                        internal::getLocalTransform(attachedStage, childKey, UsdTimeCode::Default(),
                                                    resetXformOpStack);
                        if (resetXformOpStack)
                            return true; // prune this object's descendants

                        const PXR_NS::SdfPath childPath = attachedStage.pathFor(childKey);
                        if (isMovableBody(attachedStage, childPath))
                        {
                            // We cant anymore early exit here, as we do support now nested bodies...
                            moveBody(attachedStage, childPath);
                        }
                        return false;
                    },
                    omni::physics::parse::DescendantScope::eActive);
            }
        }
    }
}

void PrimChangeMap::handleTransformChange(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    TRACE_FUNCTION();
    if (attachedStage.getObjectDatabase()->empty())
        return;

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

        if (src->isA(key, internal::schemaTypeToken<UsdGeomPointInstancer>(*src)))
        {
            movePointInstancer(attachedStage, key);
        }
        else
        {
            omni::physics::parse::ObjectKey resyncKey;
            if (isNonMovable(attachedStage, key, resyncKey))
            {
                PXR_NS::SdfPath resyncPath = attachedStage.pathFor(resyncKey);
                if (resyncPath.IsEmpty())
                    return;

                {
                    PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
                    PrimHierarchyStorage::Iterator iterator(primStorage, resyncPath);
                    for (size_t i = iterator.getDescendentsPaths().size(); i--;)
                    {
                        const PXR_NS::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
                        handleRemovedPrim(attachedStage, primCPath);
                    }
                    primStorage.removeIteration(iterator);
                    attachedStage.getPrimUpdateMap().addPrim(attachedStage, resyncPath);
                }
            }
            else
            {
                src->forEachDescendantPruned(
                    key,
                    [&](omni::physics::parse::ObjectKey childKey) -> bool
                    {
                        bool resetXformOpStack = false;
                        internal::getLocalTransform(attachedStage, childKey, UsdTimeCode::Default(),
                                                    resetXformOpStack);
                        if (resetXformOpStack)
                            return true;

                        if (isMovableBody(attachedStage, childKey))
                        {
                            moveBody(attachedStage, childKey);
                        }
                        return false;
                    },
                    omni::physics::parse::DescendantScope::eActive);
            }
        }
    }
}

static const TfToken gMimicJointRotXAPIToken("PhysxMimicJointAPI:rotX");
static const TfToken gMimicJointRotYAPIToken("PhysxMimicJointAPI:rotY");
static const TfToken gMimicJointRotZAPIToken("PhysxMimicJointAPI:rotZ");
static const TfToken gMimicJointAPITokenList[] = {
    gMimicJointRotXAPIToken, gMimicJointRotYAPIToken, gMimicJointRotZAPIToken
};
static const uint32_t gMimicJointAPITokenCount = sizeof(gMimicJointAPITokenList) / sizeof(gMimicJointAPITokenList[0]);
static const SchemaAPIFlag::Enum gMimicJointSchemaAPIFlagList[gMimicJointAPITokenCount] = {
    SchemaAPIFlag::eMimicJointRotXAPI, SchemaAPIFlag::eMimicJointRotYAPI, SchemaAPIFlag::eMimicJointRotZAPI
};

static const PXR_NS::TfToken gDrivePerformanceEnvelopeAPINameTokensInstanced[5] =
{
    PXR_NS::TfToken("PhysxDrivePerformanceEnvelopeAPI:angular"),
    PXR_NS::TfToken("PhysxDrivePerformanceEnvelopeAPI:linear"),
    PXR_NS::TfToken("PhysxDrivePerformanceEnvelopeAPI:rotX"),
    PXR_NS::TfToken("PhysxDrivePerformanceEnvelopeAPI:rotY"),
    PXR_NS::TfToken("PhysxDrivePerformanceEnvelopeAPI:rotZ")
};
static const SchemaAPIFlag::Enum  gDrivePerformanceEnvelopAPIFlagList[] =
{
 SchemaAPIFlag::eDrivePerformanceEnvelopeAngularAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeLinearAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotXAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotYAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotZAPI
};

static const PXR_NS::TfToken gJointAxisAPINameTokensInstanced[5] =
{
    PXR_NS::TfToken("PhysxJointAxisAPI:angular"),
    PXR_NS::TfToken("PhysxJointAxisAPI:linear"),
    PXR_NS::TfToken("PhysxJointAxisAPI:rotX"),
    PXR_NS::TfToken("PhysxJointAxisAPI:rotY"),
    PXR_NS::TfToken("PhysxJointAxisAPI:rotZ")
};
static const SchemaAPIFlag::Enum  gJointAxisAPIFlagList[] =
{
 SchemaAPIFlag::eJointAxisAngularAPI,
 SchemaAPIFlag::eJointAxisLinearAPI,
 SchemaAPIFlag::eJointAxisRotXAPI,
 SchemaAPIFlag::eJointAxisRotYAPI,
 SchemaAPIFlag::eJointAxisRotZAPI
};

bool checkForStructuralSchemaAPIChanges(AttachedStage& attachedStage, const PXR_NS::SdfPath& path, const TfTokenVector& apiSchemas,
    uint32_t categories, SdfPath& resyncPath)
{
    // known physics schemas
    static const TfToken gPhysxForceAPIToken("PhysxForceAPI");
    static const TfToken gPhysicsAPIToken("PhysicsRigidBodyAPI");
    static const TfToken gCollisionAPIToken("PhysicsCollisionAPI");
    static const TfToken gParticleSetAPIToken("PhysxParticleSetAPI");
    static const TfToken gDeformableBodyAPIToken("OmniPhysicsDeformableBodyAPI");
    static const TfToken gSurfaceDeformableSimAPIToken("OmniPhysicsSurfaceDeformableSimAPI");
    static const TfToken gVolumeDeformableSimAPIToken("OmniPhysicsVolumeDeformableSimAPI");

    std::vector<ObjectCategory> appliedCategories;
    for (const TfToken& apiSchema : apiSchemas)
    {
        if (apiSchema == gPhysicsAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eRigidBodyAPI))
            {
                return true;
            }
            appliedCategories.push_back(eBody);
        }
        else if (apiSchema == gCollisionAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eCollisionAPI))
            {
                return true;
            }
            appliedCategories.push_back(eShape);
        }
        if (apiSchema == gPhysxForceAPIToken)
        {
            if (!(categories & SchemaAPIFlag::ePhysxForceAPI))
            {
                return true;
            }
            appliedCategories.push_back(ePhysxForce);
        }
        else if (apiSchema == gParticleSetAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eParticleSetAPI))
            {
                return true;
            }
            appliedCategories.push_back(eParticleSet);
        }
        else if (apiSchema == gDeformableBodyAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eDeformableBodyAPI))
            {
                return true;
            }
            appliedCategories.push_back(eDeformableBody);
        }
        else if (apiSchema == gSurfaceDeformableSimAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eSurfaceDeformableSimAPI))
            {
                return true;
            }
            appliedCategories.push_back(eSurfaceDeformableBody);
        }
        else if (apiSchema == gVolumeDeformableSimAPIToken)
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
                const TfToken token = gMimicJointAPITokenList[i];

                if (apiSchema == token)
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        return true;
                    }
                    // appliedCategories.push_back();
                    // Removing the mimic joint API is not treated as structural change for now, thus appliedCategories
                    // is not used here.
                }
            }

            // NewtonMimicAPI is single-apply; add/remove is treated as structural change.
            if (apiSchema == NewtonSchemaTokens->NewtonMimicAPI)
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
            resyncPath = path.GetParentPath();
            return true;
        }
    }

    if (categories & SchemaAPIFlag::eVolumeDeformableSimAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eVolumeDeformableBody) == appliedCategories.end())
        {
            resyncPath = path.GetParentPath();
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

void processNonstructuralSchemaAPIChanges(AttachedStage& attachedStage, const PXR_NS::SdfPath& path, const TfTokenVector& apiSchemas, uint32_t categories)
{
    static const TfToken gParticleSmoothingAPIToken("PhysxParticleSmoothingAPI");
    static const TfToken gParticleAnisotropyAPIToken("PhysxParticleAnisotropyAPI");
    static const TfToken gParticleIsosurfaceAPIToken("PhysxParticleIsosurfaceAPI");
    static const TfToken gDiffuseParticlesAPIToken("PhysxDiffuseParticlesAPI");
    static const TfToken gFilteredPairsAPIToken("PhysicsFilteredPairsAPI");
    static const TfToken gPhysxContactReportAPIToken("PhysxContactReportAPI");

    std::vector<SchemaAPIFlag::Enum> appliedCategories;

    // iterate over added APIs & create if not existing
    for (const TfToken& apiSchema : apiSchemas)
    {
        if (apiSchema == gParticleIsosurfaceAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eParticleIsosurfaceAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleIsosurfaceAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleIsosurfaceAPI);
        }
        else if (apiSchema == gParticleAnisotropyAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eParticleAnisotropyAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleAnisotropyAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleAnisotropyAPI);
        }
        else if (apiSchema == gParticleSmoothingAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eParticleSmoothingAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleSmoothingAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleSmoothingAPI);
        }
        else if (apiSchema == gDiffuseParticlesAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eDiffuseParticlesAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eDiffuseParticlesAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eDiffuseParticlesAPI);
        }
        else if (apiSchema == gFilteredPairsAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eFilteredPairsAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eFilteredPairsAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eFilteredPairsAPI);
        }
        else if (apiSchema == gPhysxContactReportAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eContactReportAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eContactReportAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eContactReportAPI);
        }
        else
        {
            for (uint32_t i = 0; i < gMimicJointAPITokenCount; i++)
            {
                const TfToken token = gMimicJointAPITokenList[i];

                if (apiSchema == token)
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

                    // note: adding is treated as structural change for now, thus no call to
                    // attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, false);

                    appliedCategories.push_back(schemaAPIFlag);
                }
            }

            if (apiSchema == NewtonSchemaTokens->NewtonMimicAPI)
            {
                // note: adding is treated as structural change, thus no call to
                // attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eNewtonMimicAPI, false);

                appliedCategories.push_back(SchemaAPIFlag::eNewtonMimicAPI);
            }

            const uint32_t nbPerfEnvTokens = sizeof(gDrivePerformanceEnvelopeAPINameTokensInstanced)/sizeof(gDrivePerformanceEnvelopeAPINameTokensInstanced[0]);
            for (uint32_t i = 0; i < nbPerfEnvTokens; i++)
            {
                const TfToken token = gDrivePerformanceEnvelopeAPINameTokensInstanced[i];

                if (apiSchema == token)
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gDrivePerformanceEnvelopAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, false);
                    }
                    appliedCategories.push_back(schemaAPIFlag);
                }
            }

            const uint32_t nbJointAxisTokens = sizeof(gJointAxisAPINameTokensInstanced)/sizeof(gJointAxisAPINameTokensInstanced[0]);
            for (uint32_t i = 0; i < nbJointAxisTokens; i++)
            {
                const TfToken token = gJointAxisAPINameTokensInstanced[i];

                if (apiSchema == token)
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gJointAxisAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, false);
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
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleIsosurfaceAPI, true);
    }

    if (categories & SchemaAPIFlag::eParticleAnisotropyAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eParticleAnisotropyAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleAnisotropyAPI, true);
    }

    if (categories & SchemaAPIFlag::eParticleSmoothingAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eParticleSmoothingAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleSmoothingAPI, true);
    }

    if (categories & SchemaAPIFlag::eDiffuseParticlesAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eDiffuseParticlesAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eDiffuseParticlesAPI, true);
    }

    if (categories & SchemaAPIFlag::eFilteredPairsAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eFilteredPairsAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eFilteredPairsAPI, true);
    }

    if (categories & SchemaAPIFlag::eContactReportAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eContactReportAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eContactReportAPI, true);
    }

    for (uint32_t i = 0; i < gMimicJointAPITokenCount; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, true);
        }
    }

    if (categories & SchemaAPIFlag::eNewtonMimicAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eNewtonMimicAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eNewtonMimicAPI, true);
    }

    const uint32_t nbPerfEnvTokens = sizeof(gDrivePerformanceEnvelopeAPINameTokensInstanced)/sizeof(gDrivePerformanceEnvelopeAPINameTokensInstanced[0]);
    for (uint32_t i = 0; i < nbPerfEnvTokens; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gDrivePerformanceEnvelopAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, true);
        }
    }


    const uint32_t nbJointAxisTokens = sizeof(gJointAxisAPINameTokensInstanced)/sizeof(gJointAxisAPINameTokensInstanced[0]);
    for (uint32_t i = 0; i < nbJointAxisTokens; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gJointAxisAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, true);
        }
    }

}

bool PrimChangeMap::getPropertyChange(const PXR_NS::TfToken& token, PropertyChangeMap::const_iterator& iterator, PropertyChangeMap::const_iterator& outItEnd) const
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
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

void PrimChangeMap::checkPrimChange(AttachedStage& attachedStage, const PXR_NS::SdfPath& primKey, const PXR_NS::TfToken& propertyName, const PXR_NS::UsdPrim* primIn)
{
    bool structuralChange = false;
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    PropertyChangeMap::const_iterator itChange;
    PropertyChangeMap::const_iterator itChangeEnd;
    bool propertyChangeFound = getPropertyChange(propertyName, itChange, itChangeEnd);
    PXR_NS::SdfPath resyncPrimPath;
    while (propertyChangeFound && itChange != itChangeEnd && itChange->first == propertyName)
    {
        const PropertyChange& change = itChange->second;
        bool primCheck = change.onPrimCheck(attachedStage, primKey, propertyName, primIn);
        bool primCheckExt = true;
        if (primCheck && change.onPrimCheckExt != nullptr)
            primCheckExt = change.onPrimCheckExt(attachedStage, primKey, propertyName, primIn, resyncPrimPath);
        if (primCheck && primCheckExt)
        {
            // check for structural change, onUpdate is not defined and resyncPrimPath not set 
            // we will recreate the objects
            if (!change.onUpdate || !resyncPrimPath.IsEmpty())
            {
                structuralChange = true;
                break;
            }

            if (!usdLoad->getAsyncUSDUpdate())
            {
                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
                if (entries && !entries->empty())
                {
                    ObjectIdMap::const_iterator it = entries->begin();
                    while (it != entries->end())
                    {
                        attachedStage.getPhysXPhysicsInterface()->updateObject(
                            attachedStage, primKey, it->second, change.onUpdate, propertyName,
                            PXR_NS::UsdTimeCode::Default());
                        it++;
                    }
                }
                return;
            }
            else
            {
                bool attributeSet = false;
                ChangeMap::iterator it = m_changeMap.find(primKey);
                while (it != m_changeMap.end() && it->first == primKey)
                {
                    ChangeData& changeData = it->second;
                    if (changeData.second == propertyName)
                    {
                        attributeSet = true;
                        break;
                    }
                    it++;
                }

                if (!attributeSet)
                {
                    attributeSet = true;
                    ChangeMap::iterator it =
                        m_changeMap.insert(std::pair<const PXR_NS::SdfPath, ChangeData>(primKey, { change.onUpdate, propertyName }));
                }
                return;
            }
        }

        itChange++;
    }

    if (!structuralChange)
    {
        // Handle transform changes. USD xform-op attributes are recognized by
        // UsdGeomXformable; ovstage/fabric transform columns are routed here too.
        // moveBody reads the resolved world transform through the source.
        static const PXR_NS::TfToken kOmniXform("omni:xform");
        static const PXR_NS::TfToken kOmniResetXformStack("omni:resetXformStack");
        static const PXR_NS::TfToken kFabricLocalMatrix("omni:fabric:localMatrix");
        static const PXR_NS::TfToken kFabricWorldMatrix(gWorldMatrixTokenString);
        if (UsdGeomXformable::IsTransformationAffectedByAttrNamed(propertyName) ||
            propertyName == kOmniXform || propertyName == kOmniResetXformStack ||
            propertyName == kFabricLocalMatrix || propertyName == kFabricWorldMatrix)
        {
            if (!usdLoad->getAsyncUSDUpdate())
            {
                addTransformChange(primKey, primIn);
            }
            else
                m_transformUpdates.push_back(primKey);
            return;
        }

        // If an API schema was added/remove, check whether this triggers a structural change
        if (propertyName == UsdTokens->apiSchemas)
        {
            // Guard against expired/invalid prims (source-routed validity, no UsdPrim).
            const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
            const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
            if (!src || !src->exists(key))
                return;
            PXR_NS::TfTokenVector apiSchemas;
            src->forEachAppliedSchema(key, [&](omni::physics::parse::TokenId t)
                { apiSchemas.push_back(PXR_NS::TfToken(std::string(src->tokenToString(t)))); });
            const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
            structuralChange = checkForStructuralSchemaAPIChanges(attachedStage, primKey, apiSchemas, storedAPIs, resyncPrimPath);
            if (!structuralChange) // handle API changes that don't trigger structural changes here
                processNonstructuralSchemaAPIChanges(attachedStage, primKey, apiSchemas, storedAPIs);
        }
        else if (propertyName == UsdGeomTokens->inactiveIds)
        {
            structuralChange = true;
        }
        else if (propertyName == UsdPhysicsTokens->physicsBody0 || propertyName == UsdPhysicsTokens->physicsBody1)
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
        const PXR_NS::SdfPath resyncPath = resyncPrimPath.IsEmpty() ? primKey : resyncPrimPath;
        // Guard against expired/invalid prims (source-routed validity, no UsdPrim).
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (!src || !src->exists(attachedStage.keyFor(resyncPath)))
            return;
        PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
        PrimHierarchyStorage::Iterator iterator(primStorage, resyncPath);
        for (size_t i = iterator.getDescendentsPaths().size(); i--;)
        {
            const PXR_NS::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
            handleRemovedPrim(attachedStage, primCPath);
        }
        primStorage.removeIteration(iterator);

        attachedStage.getPrimUpdateMap().addPrim(attachedStage, resyncPath);
    }
}

void PrimChangeMap::checkPrimChange(AttachedStage& attachedStage,
                                    omni::physics::parse::ObjectKey primKey,
                                    const PXR_NS::TfToken& propertyName)
{
    auto fallbackToPath = [&]()
    {
        PXR_NS::SdfPath path = attachedStage.pathFor(primKey);
        if (!path.IsEmpty())
        {
            checkPrimChange(attachedStage, path, propertyName, nullptr);
        }
    };

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    PropertyChangeMap::const_iterator itChange;
    PropertyChangeMap::const_iterator itChangeEnd;
    bool propertyChangeFound = getPropertyChange(propertyName, itChange, itChangeEnd);

    if (propertyChangeFound)
    {
        bool canUseKeyPath = !usdLoad->getAsyncUSDUpdate();
        for (PropertyChangeMap::const_iterator it = itChange; it != itChangeEnd && it->first == propertyName; ++it)
        {
            const PropertyChange& change = it->second;
            if (!change.onPrimCheckKey || change.onPrimCheckExt || !change.onUpdate)
            {
                canUseKeyPath = false;
                break;
            }
        }

        if (!canUseKeyPath)
        {
            fallbackToPath();
            return;
        }

        while (itChange != itChangeEnd && itChange->first == propertyName)
        {
            const PropertyChange& change = itChange->second;
            bool primCheck = false;
            primCheck = change.onPrimCheckKey(attachedStage, primKey, propertyName);
            if (primCheck)
            {
                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
                if (entries && !entries->empty())
                {
                    ObjectIdMap::const_iterator it = entries->begin();
                    while (it != entries->end())
                    {
                        change.onUpdate(attachedStage, it->second, propertyName, PXR_NS::UsdTimeCode::Default());
                        it++;
                    }
                }
                return;
            }

            itChange++;
        }
    }

    // Structural/schema and transform handling is still path-oriented. Those are
    // uncommon on the cached ovstage value-update path; keep correctness by
    // falling back only when such a property arrives.
    static const PXR_NS::TfToken kOmniXform("omni:xform");
    static const PXR_NS::TfToken kOmniResetXformStack("omni:resetXformStack");
    static const PXR_NS::TfToken kFabricLocalMatrix("omni:fabric:localMatrix");
    static const PXR_NS::TfToken kFabricWorldMatrix(gWorldMatrixTokenString);
    if (UsdGeomXformable::IsTransformationAffectedByAttrNamed(propertyName) ||
        propertyName == kOmniXform || propertyName == kOmniResetXformStack ||
        propertyName == kFabricLocalMatrix || propertyName == kFabricWorldMatrix)
    {
        addTransformChange(primKey);
        return;
    }

    if (propertyName == UsdTokens->apiSchemas || propertyName == UsdGeomTokens->inactiveIds ||
        propertyName == UsdPhysicsTokens->physicsBody0 || propertyName == UsdPhysicsTokens->physicsBody1)
    {
        // Joint body relationship re-targets need the path-oriented structural
        // handling (recreate the joint); the ovstage value-update path surfaces
        // them as named property changes with no per-property update handler.
        fallbackToPath();
    }
}

void PrimChangeMap::clearMap()
{
    m_changeMap.clear();
    m_keyTransformChangesSet.clear();
}

void PrimChangeMap::removePrim(const PXR_NS::SdfPath& primKey)
{
    ChangeMap::iterator itCh = m_changeMap.find(primKey);
    if (itCh != m_changeMap.end())
        m_changeMap.erase(itCh);
}

void PrimChangeMap::processTransformUpdates(AttachedStage& attachedStage)
{
    if (!m_transformUpdates.empty())
    {
        for (size_t i = 0; i < m_transformUpdates.size(); i++)
        {
            handleTransformChange(attachedStage, m_transformUpdates[i], nullptr);
        }

        m_transformUpdates.clear();
    }
}

void PrimChangeMap::processTransformChanges(AttachedStage& attachedStage)
{
    for (PrimSet::const_reference ref : m_usdTransformChangesSet)
    {
        handleTransformChange(attachedStage, ref.first, ref.second);
    }
    m_usdTransformChangesSet.clear();

    for (PrimKeySet::const_reference key : m_keyTransformChangesSet)
    {
        handleTransformChange(attachedStage, key);
    }
    m_keyTransformChangesSet.clear();
}

bool PrimUpdateMap::needsSceneReset(const omni::physics::parse::IPhysicsSource& source, omni::physics::parse::ObjectKey key)
{
    return source.isA(key, internal::schemaTypeToken<UsdPhysicsScene>(source));
}

void PrimUpdateMap::addPrim(const AttachedStage& attachedStage, const PXR_NS::SdfPath& primPath)
{
    if (m_isNewScene)
        return;

    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;

    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primPath);

    // if a parent is in the map, we dont need to add the prim
    if (!isInPrimAddMap(attachedStage, key))
    {
        // before we add we need to check if some child is not already in the map
        // (scoped query over the subtree, root inclusive).
        std::vector<omni::physics::parse::ObjectKey> descendants;
        source->forEachDescendant(key,
                                  [&descendants](omni::physics::parse::ObjectKey k) { descendants.push_back(k); });
        for (const omni::physics::parse::ObjectKey childKey : descendants)
        {
            m_primAddMap.erase(attachedStage.pathFor(childKey));
        }
        m_primAddMap.insert(primPath);
        if (needsSceneReset(*source, key))
            UsdLoad::getUsdLoad()->releasePhysicsObjects(attachedStage.getStageId());
    }
}

void PrimUpdateMap::removePrim(AttachedStage& attachedStage, const PXR_NS::SdfPath& primKey)
{
    m_primAddMap.erase(primKey);

    PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
    PrimHierarchyStorage::Iterator iterator(primStorage, primKey);
    for (size_t i = iterator.getDescendentsPaths().size(); i--;)
    {
        const PXR_NS::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
        handleRemovedPrim(attachedStage, primCPath);
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
        if (m_primAddMap.find(attachedStage.pathFor(p)) != m_primAddMap.end())
            return true;
    }

    return false;
}

// remove invalid (no-longer-present) paths
void PrimUpdateMap::checkMap(const AttachedStage& attachedStage)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    for (std::set<PXR_NS::SdfPath>::iterator it = m_primAddMap.begin(); it != m_primAddMap.end();)
    {
        if (!source || !source->exists(attachedStage.keyFor(*it)))
            it = m_primAddMap.erase(it);
        else
            ++it;
    }
}

void processChangeMap(AttachedStage& attachedStage)
{
    auto changeSourceBlock = attachedStage.getChangeSourceBlock(ChangeSource::eUsd); // ChangeMap is populated by USD change notice
    ChangeMap::const_iterator itCh = attachedStage.getPrimChangeMap().getMap().begin();
    ChangeMap::const_iterator itChEnd = attachedStage.getPrimChangeMap().getMap().end();
    while (itCh != itChEnd)
    {
        const SdfPath& primKey = itCh->first;
        const ChangeData& changeData = itCh->second;

        const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
        if (entries && !entries->empty())
        {
            auto it = entries->begin();
            while (it != entries->end())
            {
                attachedStage.getPhysXPhysicsInterface()->updateObject(attachedStage, primKey, it->second, changeData.first, changeData.second, PXR_NS::UsdTimeCode::Default());
                it++;
            }
        }
        itCh++;
    }

    attachedStage.getPrimChangeMap().processTransformUpdates(attachedStage);
    attachedStage.getPrimChangeMap().clearMap();
}

void checkChange(AttachedStage& attachedStage, const PropertyChange& change, const SdfPath& primKey, const TfToken& attributeName)
{
    if (change.onPrimCheck(attachedStage, primKey, attributeName, nullptr))
    {
        // check for structural change, onUpdate is not defined
        // we will recreate the objects
        if (change.onUpdate)
        {
            const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
            if (entries && !entries->empty())
            {
                ObjectIdMap::const_iterator it = entries->begin();
                while (it != entries->end())
                {
                    attachedStage.getPhysXPhysicsInterface()->updateObject(attachedStage, primKey, it->second, change.onUpdate, attributeName, PXR_NS::UsdTimeCode::Default());
                    it++;
                }
            }
        }
    }
}


void flushBufferedChanges(AttachedStage& attachedStage, float currentTime)
{
    UsdStageWeakPtr stage = attachedStage.getStage();
    if (stage == nullptr)
        return;

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();

    if (attachedStage.getPrimUpdateMap().isEmptyScene())
    {
        attachedStage.getPrimUpdateMap().clearMap();
        attachedStage.getPrimChangeMap().clearMap();

        if (attachedStage.isReplicatorStage())
        {
            OmniPhysX::getInstance()
                .getReplicator(attachedStage.getStageId())
                ->attach(attachedStage.getStageId(), attachedStage.getPhysXPhysicsInterface(), false);
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
        loadPhysicsFromPrimitive(attachedStage, attachedStage.getPrimUpdateMap().getMap());

        // it is safe to allow updates again
        UsdLoad::getUsdLoad()->blockUSDUpdate(false);

        attachedStage.getPrimUpdateMap().clearMap();
    }

    processChangeMap(attachedStage);
}

void processUpdates(AttachedStage& attachedStage, float currentTime)
{
    UsdStageWeakPtr stage = attachedStage.getStage();
    if (stage == nullptr)
        return;

    TRACE_FUNCTION();

    if (!attachedStage.getAnimatedKinematicBodies().empty() || !attachedStage.getTimeSampleMap().empty())
    {
        PXR_NS::UsdTimeCode timeCode(currentTime * stage->GetTimeCodesPerSecond());

        {
            CARB_PROFILE_ZONE(0, "KinematicBodiesUpdate");
            for (PathKeyMap::const_reference& entry : attachedStage.getAnimatedKinematicBodies())
            {
                if (!entry.second.valid())
                    continue;

                const SdfPath& bodyPath = entry.first;
                const omni::physics::parse::ObjectKey bodyKey = entry.second;

                // Time-aware world transform at the current sim frame, read
                // through the source (per-call; source-side caching can be
                // reintroduced to restore the previous xform-cache amortisation).
                PXR_NS::GfMatrix4d localToWorld =
                    internal::getWorldTransform(attachedStage, bodyKey, timeCode);

                const GfTransform tr(localToWorld);
                const GfVec3d pos = tr.GetTranslation();
                const GfQuatd rot = tr.GetRotation().GetQuat();
                const GfVec3d sc = tr.GetScale();

                PhysXUsdPhysicsInterface::Transform fcTransform;

                fcTransform.position = { float(pos[0]), float(pos[1]), float(pos[2]) };
                fcTransform.orientation = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]),
                                            float(rot.GetImaginary()[2]), float(rot.GetReal()) };
                fcTransform.scale = { float(sc[0]), float(sc[1]), float(sc[2]) };


                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(bodyPath);
                if (entries && !entries->empty())
                {
                    auto it = entries->begin();
                    while (it != entries->end())
                    {
                        attachedStage.getPhysXPhysicsInterface()->updateTransform(attachedStage,
                            bodyPath, it->second, fcTransform);
                        it++;
                    }
                }
            }
        }

        {
            CARB_PROFILE_ZONE(0, "KinematicAttributesUpdate");
            for (auto& iterator : attachedStage.getTimeSampleMap())
            {
                const SdfPath primKey = iterator.first.GetPrimPath();
                const TfToken& attrToken = iterator.first.GetNameToken();

                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primKey);
                if (entries && !entries->empty())
                {
                    auto it = entries->begin();
                    while (it != entries->end())
                    {
                        attachedStage.getPhysXPhysicsInterface()->updateObject(attachedStage, primKey, it->second, iterator.second, attrToken, timeCode);
                        it++;
                    }
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
void onSourceChange(AttachedStage& attachedStage, const omni::physics::parse::ChangeBatch& batch)
{
    TRACE_FUNCTION();

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    if (usdLoad->usdUpdateIsBlocked())
        return;

    // Early exit if scene is empty; we will parse the whole scene.
    if (attachedStage.getPrimUpdateMap().isEmptyScene())
        return;

    UsdStageWeakPtr stage = attachedStage.getStage();
    if (!stage)
        return;

    CARB_PROFILE_ZONE(0, "PhysicsUsdNoticleListener");

    auto changeSourceBlock = attachedStage.getChangeSourceBlock(ChangeSource::eUsd);

    if (batch.keys.count == 0 || !batch.keys.data)
        return;
    const omni::physics::parse::ObjectKey* keys =
        static_cast<const omni::physics::parse::ObjectKey*>(batch.keys.data);

    // Removal (resync, prim removed or deactivated).
    if (batch.isDelete)
    {
        for (size_t i = 0; i < batch.keys.count; ++i)
        {
            const SdfPath primKey = attachedStage.pathFor(keys[i]);
            if (!primKey.IsEmpty())
                attachedStage.getPrimUpdateMap().removePrim(attachedStage, primKey);
        }
        return;
    }

    // Structural resync: invalid property, changed-field tokens carried in `values`.
    if (!batch.property.valid())
    {
        const omni::physics::parse::IPhysicsSource* psrc = attachedStage.getSource();
        if (!psrc)
            return;

        // Resync field tokens are source-space TokenIds naming the changed metadata
        // field (typeName / apiSchemas / kind). The USD source has a cached fast path;
        // any other source (ovstage) resolves the interned string and reconstructs the
        // field TfToken — same comparison either way (mirrors the value-change path).
        const omni::physics::usd::UsdSource* src = omni::physics::usd::asUsdSource(psrc);
        auto fieldToken = [&](omni::physics::parse::TokenId id) -> TfToken
        {
            return src ? src->tfTokenFor(id) : TfToken(std::string(psrc->tokenToString(id)));
        };
        static const TfToken gkindToken("kind");
        bool typeNameChange = false, apiSchemasChange = false, kindChange = false;
        const omni::physics::parse::TokenId* fields =
            static_cast<const omni::physics::parse::TokenId*>(batch.values.data);
        for (size_t i = 0; i < batch.values.count; ++i)
        {
            const TfToken f = fieldToken(fields[i]);
            if (f == SdfFieldKeys->TypeName) typeNameChange = true;
            else if (f == PXR_NS::UsdTokens->apiSchemas) apiSchemasChange = true;
            else if (f == gkindToken) kindChange = true;
        }
        for (size_t i = 0; i < batch.keys.count; ++i)
        {
            const omni::physics::parse::ObjectKey key = keys[i];
            if (!psrc->exists(key))
            {
                continue;
            }
            const SdfPath primKey = attachedStage.pathFor(key);
            if (primKey.IsEmpty())
            {
                continue;
            }

            if (!typeNameChange && apiSchemasChange)
            {
                if (!attachedStage.getPrimUpdateMap().isInPrimAddMap(attachedStage, key))
                {
                    attachedStage.getPrimChangeMap().checkPrimChange(
                        attachedStage, primKey, PXR_NS::UsdTokens->apiSchemas, nullptr);
                }
            }
            else if (!typeNameChange && kindChange)
            {
                // We ignore 'kind' changes
            }
            else
            {
                if (primKey == stage->GetPseudoRoot().GetPath())
                {
                    usdLoad->releasePhysicsObjects(static_cast<uint64_t>(attachedStage.getStageId()));
                }
                else if (!psrc->isPrototype(key))
                {
                    attachedStage.getPrimUpdateMap().removePrim(attachedStage, primKey);
                    attachedStage.getPrimUpdateMap().addPrim(attachedStage, primKey);
                }
            }
        }
        return;
    }

    // Value change. The property is the changed attribute, identified by TokenId.
    // The change-property dispatch map is keyed by TfToken, so resolve the name:
    // the USD source has a cached fast path; any other source (ovstage) resolves
    // the interned string and reconstructs the (interned) TfToken — same key.
    const omni::physics::parse::IPhysicsSource* psrc = attachedStage.getSource();
    if (!psrc)
        return;
    TfToken token;
    if (const omni::physics::usd::UsdSource* src = omni::physics::usd::asUsdSource(psrc))
        token = src->tfTokenFor(batch.property);
    else
        token = TfToken(std::string(psrc->tokenToString(batch.property)));

    const bool useObjectKeyDispatch = omni::physics::usd::asUsdSource(psrc) == nullptr;
    for (size_t i = 0; i < batch.keys.count; ++i)
    {
        if (useObjectKeyDispatch)
        {
            attachedStage.getPrimChangeMap().checkPrimChange(attachedStage, keys[i], token);
        }
        else
        {
            SdfPath primKey = attachedStage.pathFor(keys[i]);
            if (!primKey.IsEmpty())
            {
                attachedStage.getPrimChangeMap().checkPrimChange(attachedStage, primKey, token);
            }
        }
    }
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
