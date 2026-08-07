// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "ChangeRegister.h"
#include "PhysXTools.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/CollisionGroup.h"
#include "usdLoad/ChangeParams.h"
#include "usdLoad/NewtonCompat.h"

#include "propertiesUpdate/PhysXPropertiesUpdate.h"
#include "internal/InternalScene.h"
#include "usdInterface/UsdInterface.h"

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysxTokens.h>
#include <physxSchema/axisInstanceTokens.h>
#include <omniUsdPhysicsDeformableSchema/tokens.h>
#include <PxPhysicsAPI.h>

using namespace omni::physx::usdparser;
using namespace PXR_NS;

static TfToken g_rotX("rotX");
static TfToken g_rotY("rotY");
static TfToken g_rotZ("rotZ");
static TfToken g_transX("transX");
static TfToken g_transY("transY");
static TfToken g_transZ("transZ");
static TfToken g_distance("distance");

extern bool updateMaterialDensity(AttachedStage& attachedStage, ObjectId objectId, const TfToken&, const UsdTimeCode&);

using namespace omni::physx::internal;
using namespace physx;

namespace omni
{
namespace physx
{

//
// note: if updateObjectFn is set to nullptr, the object will get released and the prim parsed again
//
#define REGISTER_CHANGE(changeParams, attribute, primCheckFn, updateObjectFn)                                          \
    {                                                                                                                  \
        ChangeParams cp{ attribute, primCheckFn, nullptr, updateObjectFn,                                             \
                         primCheckFn == emptyRequirementCheck ? emptyKeyRequirementCheck : nullptr };                  \
        changeParams.push_back(cp);                                                                                    \
    }

#define REGISTER_CHANGE_EXT(changeParams, attribute, primCheckFn, primCheckExtFn, updateObjectFn)                      \
    {                                                                                                                  \
        ChangeParams cp{ attribute, primCheckFn, primCheckExtFn, updateObjectFn };                                     \
        changeParams.push_back(cp);                                                                                    \
    }

static bool emptyRequirementCheck(AttachedStage& attachedStage, const SdfPath&, const TfToken&, const UsdPrim*)
{
    return true;
}

static bool emptyKeyRequirementCheck(AttachedStage& attachedStage,
                                     omni::physics::parse::ObjectKey,
                                     const TfToken&)
{
    return true;
}

using omni::physx::internal::schemaTypeToken;

// True iff any instance of multi-apply schema SchemaT is applied to `key` --
// the source-side equivalent of HasAPI<SchemaT>() (no instance), which
// hasSchema cannot express (it matches an exact applied name, but multi-apply
// applied names carry an instance suffix). Uses forEachMultiApplyInstance.
template <typename SchemaT>
static bool hasAnyMultiApplyInstance(const omni::physics::parse::IPhysicsSource& src,
                                     omni::physics::parse::ObjectKey key)
{
    static const std::string baseName =
        PXR_NS::UsdSchemaRegistry::GetSchemaTypeName(PXR_NS::TfType::Find<SchemaT>()).GetString();
    bool any = false;
    src.forEachMultiApplyInstance(key, baseName, [&any](std::string_view) { any = true; });
    return any;
}

static bool bodyRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(attachedStage.keyFor(primKey)) && (storedAPIs & SchemaAPIFlag::eRigidBodyAPI) != 0;
}

static bool noBodyRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(attachedStage.keyFor(primKey)) && !(storedAPIs & SchemaAPIFlag::eRigidBodyAPI);
}


static bool bodyPointInstancerRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    return src->isA(key, schemaTypeToken<UsdGeomPointInstancer>(*src)) &&
           !src->hasSchema(key, schemaTypeToken<PhysxSchemaPhysxParticleAPI>(*src));
}

static bool collisionRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(attachedStage.keyFor(primKey)) && (storedAPIs & SchemaAPIFlag::eCollisionAPI) != 0;
}

static bool jointRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (src && src->isA(attachedStage.keyFor(primKey), schemaTypeToken<UsdPhysicsJoint>(*src)))
        return true;

    // Non-USD sources may not be able to answer the full USD schema hierarchy for
    // every typed joint. If the prim already owns a joint runtime object, the
    // joint-property updater can handle this change safely.
    const ObjectIdMap* entries = attachedStage.getObjectIds(primKey);
    if (!entries || entries->empty())
        return false;

    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    for (ObjectIdMap::const_iterator it = entries->begin(); it != entries->end(); ++it)
    {
        if (it->second >= db.getRecords().size())
            continue;
        const InternalDatabase::Record& rec = db.getRecords()[size_t(it->second)];
        if (rec.mType == ePTJoint || rec.mType == ePTCustomJoint)
            return true;
    }
    return false;
}

static bool materialRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    return src && src->hasSchema(attachedStage.keyFor(primKey), schemaTypeToken<UsdPhysicsMaterialAPI>(*src));
}

static bool physxMaterialRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    return src->hasSchema(key, schemaTypeToken<UsdPhysicsMaterialAPI>(*src)) &&
           src->hasSchema(key, schemaTypeToken<PhysxSchemaPhysxMaterialAPI>(*src));
}

static bool jointEnableDisableRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->isA(attachedStage.keyFor(primKey), schemaTypeToken<UsdPhysicsJoint>(*src)))
        return false;

    bool val;
    if (!getValue<bool>(attachedStage, primKey, propName, UsdTimeCode(), val))
        return false;

    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const ObjectIdMap* entries = attachedStage.getObjectIds(primKey);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            if (it->second < db.getRecords().size())
            {
                const InternalDatabase::Record& rec = db.getRecords()[size_t(it->second)];
                if (rec.mType == ePTJoint || rec.mType == ePTLinkJoint || rec.mType == ePTLink)
                {
                    if (!val)
                        return true;
                    else
                        return false;
                }
                else if (rec.mType == ePTArticulationFixedBase && rec.mPtr)
                {
                    PxArticulationReducedCoordinate* art = (PxArticulationReducedCoordinate*)rec.mPtr;
                    art->setArticulationFlag(PxArticulationFlag::eFIX_BASE, val);
                    if (val)
                        art->wakeUp();
                    return false;
                }
            }
            it++;
        }
    }

    if (val)
        return true;

    return false;
}


static bool physxParticleSetRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(attachedStage.keyFor(primKey)) && (storedAPIs & SchemaAPIFlag::eParticleSetAPI) != 0;
}

static bool physxParticleSetPositionRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);

    if (src->exists(key) && (storedAPIs & SchemaAPIFlag::eParticleSetAPI))
    {
        // Block point/position updates only when simulationPoints is NOT authored.
        return !src->hasAuthoredAttribute(key, src->internToken(PhysxSchemaTokens.Get()->physxParticleSimulationPoints.GetString()));
    }
    return false;
}

static bool physicsDeformableBodyRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(attachedStage.keyFor(primKey)) && (storedAPIs & SchemaAPIFlag::eDeformableBodyAPI) != 0;
}

static bool physicsDeformableBodyResyncCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn, SdfPath& resyncPrimPath)
{
    //deformables require resync if disabled/enabled
    resyncPrimPath = primKey;
    return true;
}

static bool physicsDeformableBodyHierarchyCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    if (!src->exists(key))
        return false;

    const auto* db = attachedStage.getObjectDatabase();
    if ((db->getSchemaAPIs(primKey) & SchemaAPIFlag::eDeformableBodyAPI) > 0)
        return true;

    // Walk ancestors via the source (getParent terminates at the root, which
    // carries no flags) -- mirrors the prim.GetParent() chain.
    for (omni::physics::parse::ObjectKey p = src->getParent(key); p.valid(); p = src->getParent(p))
    {
        if (db->getSchemaAPIs(attachedStage.pathFor(p)) & SchemaAPIFlag::eDeformableBodyAPI)
            return true;
    }
    return false;
}

static bool physicsDeformableCollisionCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    bool isDeformableBodyHier = physicsDeformableBodyHierarchyCheck(attachedStage, primKey, propName, primIn);
    if (isDeformableBodyHier)
    {
        uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
        if ((storedAPIs & SchemaAPIFlag::eCollisionAPI) > 0)
        {
            return true;
        }
    }
    return false;
}

static bool physicsRigidCollisionCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    bool isDeformableBodyHier = physicsDeformableBodyHierarchyCheck(attachedStage, primKey, propName, primIn);
    if (!isDeformableBodyHier)
    {
        uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
        if ((storedAPIs & SchemaAPIFlag::eCollisionAPI) > 0)
        {
            return true;
        }
    }
    return false;
}

static bool physicsDeformableBodyHierarchyResyncCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn, SdfPath& resyncPrimPath)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const auto* db = attachedStage.getObjectDatabase();

    const uint64_t storedAPIs = db->getSchemaAPIs(primKey);
    if ((storedAPIs & SchemaAPIFlag::eDeformableBodyAPI) > 0)
    {
        resyncPrimPath = primKey;
        return true;
    }

    if ((storedAPIs & (SchemaAPIFlag::eDeformablePoseAPI | SchemaAPIFlag::eVolumeDeformableSimAPI | SchemaAPIFlag::eSurfaceDeformableSimAPI | SchemaAPIFlag::eCollisionAPI)) > 0)
    {
        const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
        for (omni::physics::parse::ObjectKey p = src->getParent(key); p.valid(); p = src->getParent(p))
        {
            const SdfPath ancestorPath = attachedStage.pathFor(p);
            if (db->getSchemaAPIs(ancestorPath) & SchemaAPIFlag::eDeformableBodyAPI)
            {
                resyncPrimPath = ancestorPath;
                return true;
            }
        }
    }
    return false;
}

static bool physicsDeformableSimRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    // Gated purely on cached schema-API flags (the prim was never inspected).
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return (storedAPIs & (SchemaAPIFlag::eVolumeDeformableSimAPI | SchemaAPIFlag::eSurfaceDeformableSimAPI)) != 0;
}

static bool cctRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    return src->hasSchema(key, schemaTypeToken<PhysxSchemaPhysxCharacterControllerAPI>(*src)) &&
           src->isA(key, schemaTypeToken<UsdGeomCapsule>(*src));
}

static bool collisionGroupRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;

    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    if (src->isA(key, schemaTypeToken<UsdPhysicsCollisionGroup>(*src)))
    {
        // The gate decision is source-agnostic (isA); the scoped parse scan is
        // rooted by source path and routed through the active scan backend.
        appendCollisionGroupFromPath(attachedStage, primKey, attachedStage.getCollisionGroupMap());
        return true;
    }
    return false;
}

// The tendon attachment/axis API schemas are MULTI-APPLY, so HasAPI<X>()
// ("any instance applied") maps to hasAnyMultiApplyInstance, not hasSchema.
static bool physxSpatialTendonRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src->isA(key, schemaTypeToken<UsdGeomXformable>(*src)) && (storedAPIs & SchemaAPIFlag::eRigidBodyAPI) &&
           hasAnyMultiApplyInstance<PhysxSchemaPhysxTendonAttachmentRootAPI>(*src, key);
}

static bool physxRigidBodyAttachmentRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src->isA(key, schemaTypeToken<UsdGeomXformable>(*src)) && (storedAPIs & SchemaAPIFlag::eRigidBodyAPI) &&
           hasAnyMultiApplyInstance<PhysxSchemaPhysxTendonAttachmentAPI>(*src, key);
}

static bool physxTendonAttachmentLeafRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src->isA(key, schemaTypeToken<UsdGeomXformable>(*src)) && (storedAPIs & SchemaAPIFlag::eRigidBodyAPI) &&
           hasAnyMultiApplyInstance<PhysxSchemaPhysxTendonAttachmentLeafAPI>(*src, key);
}

static bool physxFixedTendonRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    return src->isA(key, schemaTypeToken<UsdPhysicsJoint>(*src)) &&
           hasAnyMultiApplyInstance<PhysxSchemaPhysxTendonAxisRootAPI>(*src, key);
}

static bool physxTendonAxisRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::ObjectKey key = attachedStage.keyFor(primKey);
    return src->isA(key, schemaTypeToken<UsdPhysicsJoint>(*src)) &&
           hasAnyMultiApplyInstance<PhysxSchemaPhysxTendonAxisAPI>(*src, key);
}

template<SchemaAPIFlag::Enum tSchemaFlag>
static bool physxMimicJointRequirementCheck(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& propName, const UsdPrim* primIn)
{
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);

    // If the mimic joint API schema gets applied after the stage has been attached, no update
    // callback should run for properties set in the same frame (internal objects not yet created).

    if (storedAPIs & tSchemaFlag)
        return true;
    else
        return false;
}

void testTimeSampledAttribute(AttachedStage& attachedStage, const SdfPath& primKey, const TfToken& attributeName, usdparser::OnUpdateObjectFn onUpdate)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (src && src->isAttributeTimeSampled(attachedStage.keyFor(primKey), src->internToken(attributeName.GetString())))
    {
        attachedStage.registerTimeSampledAttribute(primKey.AppendProperty(attributeName), onUpdate);
    }
}




void registerDriveTimeSampledChanges(AttachedStage& attachedStage, const SdfPath& jointPrimKey, std::string driveAxis)
{
    testTimeSampledAttribute(attachedStage, jointPrimKey, TfToken(driveAxis + std::string(":physics:targetPosition")), updateDriveTargetPosition);
    testTimeSampledAttribute(attachedStage, jointPrimKey, TfToken(driveAxis + std::string(":physics:targetVelocity")), updateDriveTargetVelocity);
    testTimeSampledAttribute(attachedStage, jointPrimKey, TfToken(driveAxis + std::string(":physics:maxForce")), updateDriveMaxForce);
    testTimeSampledAttribute(attachedStage, jointPrimKey, TfToken(driveAxis + std::string(":physics:damping")), updateDriveDamping);
    testTimeSampledAttribute(attachedStage, jointPrimKey, TfToken(driveAxis + std::string(":physics:stiffness")), updateDriveStiffness);
    testTimeSampledAttribute(attachedStage, jointPrimKey, TfToken(driveAxis + std::string(":physics:type")), updateDriveType);
}

void registerJointTimeSampledChanges(AttachedStage& attachedStage, const SdfPath& jointPrimKey)
{
    testTimeSampledAttribute(attachedStage, jointPrimKey, UsdPhysicsTokens.Get()->physicsLocalPos0, updateLocalPos0);
    testTimeSampledAttribute(attachedStage, jointPrimKey, UsdPhysicsTokens.Get()->physicsLocalPos1, updateLocalPos1);
    testTimeSampledAttribute(attachedStage, jointPrimKey, UsdPhysicsTokens.Get()->physicsLocalRot0, updateLocalRot0);
    testTimeSampledAttribute(attachedStage, jointPrimKey, UsdPhysicsTokens.Get()->physicsLocalRot1, updateLocalRot1);
}

void registerSceneTimeSampledChanges(AttachedStage& attachedStage, const SdfPath& scenePrimKey)
{
    testTimeSampledAttribute(attachedStage, scenePrimKey, UsdPhysicsTokens.Get()->physicsGravityMagnitude, updateGravityMagnitude);
    testTimeSampledAttribute(attachedStage, scenePrimKey, UsdPhysicsTokens.Get()->physicsGravityDirection, updateGravityDirection);
}

static void addToStageSpecificAttributeMap(AttachedStage& attachedStage, std::string attributeName, OnPrimRequirementCheckFn checkFn,
    OnUpdateObjectFn updateFn)
{
    ChangeParams cp{ attributeName, checkFn, nullptr, updateFn,
                     checkFn == emptyRequirementCheck ? emptyKeyRequirementCheck : nullptr };
    attachedStage.registerStageSpecificAttribute(cp);
}

static void addToStageSpecificAttributeMapExt(AttachedStage& attachedStage, std::string attributeName, OnPrimRequirementCheckFn checkFn,
    OnPrimRequirementCheckExtFn checkExtFn, OnUpdateObjectFn updateFn)
{
    ChangeParams cp{ attributeName, checkFn, checkExtFn, updateFn };
    attachedStage.registerStageSpecificAttribute(cp);
}

// spatial tendon setup
void registerSpatialTendonChangeParams(AttachedStage& attachedStage, const std::string& instanceName)
{
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":stiffness", physxSpatialTendonRequirementCheck, updateSpatialTendonStiffness);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":limitStiffness", physxSpatialTendonRequirementCheck, updateSpatialTendonLimitStiffness);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":damping", physxSpatialTendonRequirementCheck, updateSpatialTendonDamping);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":offset", physxSpatialTendonRequirementCheck, updateSpatialTendonOffset);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":tendonEnabled", physxSpatialTendonRequirementCheck, updateSpatialTendonEnabled);
}

void registerTendonAttachmentChangeParams(AttachedStage& attachedStage, const std::string& instanceName)
{
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":gearing", physxRigidBodyAttachmentRequirementCheck, updateTendonAttachmentGearing);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":localPos", physxRigidBodyAttachmentRequirementCheck, updateTendonAttachmentLocalPos);
}

void registerTendonAttachmentLeafChangeParams(AttachedStage& attachedStage, const std::string& instanceName)
{
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":restLength", physxTendonAttachmentLeafRequirementCheck, updateTendonAttachmentLeafRestLength);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":lowerLimit", physxTendonAttachmentLeafRequirementCheck, updateTendonAttachmentLeafLowLimit);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":upperLimit", physxTendonAttachmentLeafRequirementCheck, updateTendonAttachmentLeafHighLimit);
}

// fixed tendon setup
void registerFixedTendonChangeParams(AttachedStage& attachedStage, const std::string& instanceName)
{
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":stiffness", physxFixedTendonRequirementCheck, updateFixedTendonStiffness);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":limitStiffness", physxFixedTendonRequirementCheck, updateFixedTendonLimitStiffness);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":damping", physxFixedTendonRequirementCheck, updateFixedTendonDamping);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":offset", physxFixedTendonRequirementCheck, updateFixedTendonOffset);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":tendonEnabled", physxFixedTendonRequirementCheck, updateFixedTendonEnabled);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":restLength", physxFixedTendonRequirementCheck, updateFixedTendonRestLength);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":lowerLimit", physxFixedTendonRequirementCheck, updateFixedTendonLowLimit);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":upperLimit", physxFixedTendonRequirementCheck, updateFixedTendonHighLimit);
}

void registerTendonAxisChangeParam(AttachedStage& attachedStage, const std::string& instanceName)
{
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":gearing", physxTendonAxisRequirementCheck, updateTendonAxisSingleGearing);
    addToStageSpecificAttributeMap(attachedStage, "physxTendon:" + instanceName + ":forceCoefficient", physxTendonAxisRequirementCheck, updateTendonAxisSingleForceCoefficient);
}

// deformable pose setup
void registerDeformablePoseChangeParams(AttachedStage& attachedStage, const std::string& instanceName)
{
    TfToken pointsAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPoints, instanceName);
    TfToken purposesAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(OmniUsdPhysicsDeformableSchemaTokens->deformablePose_MultipleApplyTemplate_OmniphysicsPurposes, instanceName);
    addToStageSpecificAttributeMapExt(attachedStage, pointsAttrName, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr);
    addToStageSpecificAttributeMapExt(attachedStage, purposesAttrName, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr);
}

// setup persistent change listeners
void registerChangeParams(std::vector<usdparser::ChangeParams>& changeParams)
{
    // scene live changes
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsGravityMagnitude, emptyRequirementCheck, updateGravityMagnitude)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsGravityDirection, emptyRequirementCheck, updateGravityDirection)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxSceneTimeStepsPerSecond, emptyRequirementCheck, updateTimeStepsPerSecond)
    // Newton scene fallbacks (apply only when the corresponding PhysX attribute is not authored).
    REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonTimeStepsPerSecond, emptyRequirementCheck, updateNewtonTimeStepsPerSecond)
    REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonGravityEnabled, emptyRequirementCheck, updateNewtonGravityEnabled)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxSceneUpdateType, emptyRequirementCheck, updateSceneUpdateType)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxSceneQuasistaticEnableQuasistatic, emptyRequirementCheck, updateQuasistaticEnabled)
    REGISTER_CHANGE(changeParams, "collection:quasistaticactors:includes", emptyRequirementCheck, updateQuasistaticCollection)
    REGISTER_CHANGE(changeParams, "collection:quasistaticactors:excludes", emptyRequirementCheck, updateQuasistaticCollection)

    // body live changes
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsRigidBodyEnabled, emptyRequirementCheck, updateBodyEnabled)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsMass, bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsDensity, bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsCenterOfMass, bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsDiagonalInertia, bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsPrincipalAxes, bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsVelocity, emptyRequirementCheck, updateBodyLinearVelocity)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsAngularVelocity, emptyRequirementCheck, updateBodyAngularVelocity)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsKinematicEnabled, emptyRequirementCheck, updateBodyEnableKinematics)
    REGISTER_CHANGE(changeParams, UsdGeomTokens->xformOpOrder, emptyRequirementCheck, updateBodyTransformStack)

    REGISTER_CHANGE(
        changeParams, PhysxSchemaTokens.Get()->physxRigidBodyLinearDamping, emptyRequirementCheck, updateBodyLinearDamping)
    REGISTER_CHANGE(
        changeParams, PhysxSchemaTokens.Get()->physxRigidBodyAngularDamping, emptyRequirementCheck, updateBodyAngularDamping)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyMaxLinearVelocity, emptyRequirementCheck,
                    updateBodyMaxLinearVelocity)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyMaxAngularVelocity, emptyRequirementCheck,
                    updateBodyMaxAngularVelocity)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyMaxContactImpulse, emptyRequirementCheck,
                    updateBodyMaxContactImpulse)
    REGISTER_CHANGE(
        changeParams, PhysxSchemaTokens.Get()->physxRigidBodySleepThreshold, emptyRequirementCheck, updateBodySleepThreshold)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyStabilizationThreshold, emptyRequirementCheck,
                    updateBodyStabilizationThreshold)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyMaxDepenetrationVelocity, emptyRequirementCheck,
                    updateBodyMaxDepenetrationVelocity)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyContactSlopCoefficient, emptyRequirementCheck,
                    updateBodyContactSlopCoefficient)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodySolverPositionIterationCount, emptyRequirementCheck,
                    updateBodySolverPositionIterationCount)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodySolverVelocityIterationCount, emptyRequirementCheck,
                    updateBodySolverVelocityIterationCount)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyEnableCCD, emptyRequirementCheck, updateBodyEnableCCD)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyEnableSpeculativeCCD, emptyRequirementCheck,
                    updateBodyEnableSpeculativeCCD)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyRetainAccelerations, emptyRequirementCheck,
                    updateBodyRetainAccelerations)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyEnableGyroscopicForces, emptyRequirementCheck,
                    updateBodyGyroscopicForces)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyDisableGravity, emptyRequirementCheck,
                    updateBodyDisableGravity)
    REGISTER_CHANGE(
        changeParams, PhysxSchemaTokens.Get()->physxRigidBodyLockedPosAxis, emptyRequirementCheck, updateBodyLockedPosAxis)
    REGISTER_CHANGE(
        changeParams, PhysxSchemaTokens.Get()->physxRigidBodyLockedRotAxis, emptyRequirementCheck, updateBodyLockedRotAxis)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodyCfmScale, emptyRequirementCheck, updateBodyCfmScale)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxRigidBodySolveContact, emptyRequirementCheck, updateBodySolveContacts)

    // contact report
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxContactReportThreshold, emptyRequirementCheck, updatePhysxContactReportThreshold)

    // force
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxForceForceEnabled, emptyRequirementCheck, updatePhysxForceEnabled)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxForceMode, emptyRequirementCheck, updatePhysxForceMode)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxForceTorque, emptyRequirementCheck, updatePhysxTorque)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxForceWorldFrameEnabled, emptyRequirementCheck, updatePhysxForceWorldFrameEnabled)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxForceForce, emptyRequirementCheck, updatePhysxForce)

    // surface velocity
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxSurfaceVelocitySurfaceVelocityEnabled, emptyRequirementCheck, updateBodySurfaceVelocityEnabled)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxSurfaceVelocitySurfaceVelocity, emptyRequirementCheck, updateBodySurfaceLinearVelocity)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxSurfaceVelocitySurfaceAngularVelocity, emptyRequirementCheck, updateBodySurfaceAngularVelocity)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxSplinesSurfaceVelocitySurfaceVelocityMagnitude, emptyRequirementCheck, updateBodySplineSurfaceVelocityMagnitude)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxSplinesSurfaceVelocitySurfaceVelocityEnabled, emptyRequirementCheck, updateBodySplineSurfaceVelocityEnabled)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxSurfaceVelocitySurfaceVelocity, emptyRequirementCheck,
                    updateBodySurfaceLinearVelocity)

    // body point instancer changes
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->positions, bodyPointInstancerRequirementCheck, updateBodyInstancedPositions)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->orientations, bodyPointInstancerRequirementCheck, updateBodyInstancedOrientations)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->velocities, bodyPointInstancerRequirementCheck, updateBodyInstancedVelocities)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->angularVelocities, bodyPointInstancerRequirementCheck, updateBodyInstancedAngularVelocities)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->protoIndices, bodyPointInstancerRequirementCheck, nullptr)

    // collision live changes
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsMass, collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsDensity, collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsCenterOfMass, collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsDiagonalInertia, collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsPrincipalAxes, collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsCollisionEnabled, collisionRequirementCheck, updateShapeEnabled)

    REGISTER_CHANGE(
        changeParams, PhysxSchemaTokens.Get()->physxCollisionContactOffset, physicsRigidCollisionCheck, updateShapeContactOffset)
    REGISTER_CHANGE(
        changeParams, PhysxSchemaTokens.Get()->physxCollisionRestOffset, physicsRigidCollisionCheck, updateShapeRestOffset)
    // Newton collision fallbacks (rigid).
    REGISTER_CHANGE(
        changeParams, NewtonSchemaTokens->newtonContactMargin, physicsRigidCollisionCheck, updateNewtonShapeContactMargin)
    REGISTER_CHANGE(
        changeParams, NewtonSchemaTokens->newtonContactGap, physicsRigidCollisionCheck, updateNewtonShapeContactGap)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxCollisionTorsionalPatchRadius, emptyRequirementCheck,
                    updateShapeTorsionalPatchRadius)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxCollisionMinTorsionalPatchRadius, emptyRequirementCheck,
                    updateShapeMinTorsionalPatchRadius)

    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsApproximation, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxConvexHullCollisionHullVertexLimit, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxConvexHullCollisionMinThickness, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxConvexDecompositionCollisionHullVertexLimit, emptyRequirementCheck, nullptr)
    // Newton fallback: newton:maxHullVertices feeds convex hull and decomposition vertex limits.
    // Mirrors PhysX (release+reparse on change).
    REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonMaxHullVertices, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxConvexDecompositionCollisionErrorPercentage, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxConvexDecompositionCollisionMaxConvexHulls, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxConvexDecompositionCollisionMinThickness, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxConvexDecompositionCollisionVoxelResolution, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxConvexDecompositionCollisionShrinkWrap, emptyRequirementCheck, nullptr)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxTriangleMeshSimplificationCollisionMetric, emptyRequirementCheck, nullptr)

    // simulation owner changes, fast path for rigid bodies, reparse otherwise
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsSimulationOwner, bodyRequirementCheck, updateBodySimulationOwner)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsSimulationOwner, noBodyRequirementCheck, nullptr)

    // filtered pairs rel
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsFilteredPairs, emptyRequirementCheck, updateFilteredPairs)

    // material live changes
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsDensity, materialRequirementCheck, updateMaterialDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsDynamicFriction, materialRequirementCheck,
                    updateMaterialDynamicFriction)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsStaticFriction, materialRequirementCheck, updateMaterialStaticFriction)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsRestitution, materialRequirementCheck, updateMaterialRestitution)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxMaterialFrictionCombineMode, physxMaterialRequirementCheck,
                    updateMaterialFrictionCombineMode)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxMaterialRestitutionCombineMode, physxMaterialRequirementCheck,
                    updateMaterialRestitutionCombineMode)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxMaterialDampingCombineMode, physxMaterialRequirementCheck,
                    updateMaterialDampingCombineMode)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxMaterialCompliantContactAccelerationSpring,
                    physxMaterialRequirementCheck, updateCompliantMaterial)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxMaterialCompliantContactStiffness,
                    physxMaterialRequirementCheck, updateCompliantMaterial)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxMaterialCompliantContactDamping,
                    physxMaterialRequirementCheck, updateCompliantMaterial)


    // collision groups live changes
    REGISTER_CHANGE(changeParams, "collection:colliders:includes", collisionGroupRequirementCheck, updateCollisionGroup)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsFilteredGroups, collisionGroupRequirementCheck, updateCollisionGroup)


    // joint enable/disable
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsJointEnabled, jointEnableDisableRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsBody0, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsBody1, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(
            changeParams, UsdPhysicsTokens.Get()->physicsCollisionEnabled, jointRequirementCheck, updateEnableCollision)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsBreakForce, emptyRequirementCheck, updateBreakForce)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsBreakTorque, emptyRequirementCheck, updateBreakTorque)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsLocalPos0, emptyRequirementCheck, updateLocalPos0)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsLocalRot0, emptyRequirementCheck, updateLocalRot0)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsLocalPos1, emptyRequirementCheck, updateLocalPos1)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsLocalRot1, emptyRequirementCheck, updateLocalRot1)
    REGISTER_CHANGE(
        changeParams, PhysxSchemaTokens.Get()->physxJointArmature, emptyRequirementCheck, updateArmature)

    // revolute joint
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsLowerLimit, emptyRequirementCheck, updateLimitLow)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsUpperLimit, emptyRequirementCheck, updateLimitHigh)
    REGISTER_CHANGE(changeParams, "drive:angular:physics:targetPosition", emptyRequirementCheck, updateDriveTargetPosition)
    REGISTER_CHANGE(changeParams, "drive:angular:physics:targetVelocity", emptyRequirementCheck, updateDriveTargetVelocity)
    REGISTER_CHANGE(changeParams, "drive:angular:physics:maxForce", emptyRequirementCheck, updateDriveMaxForce)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->maxActuatorVelocityAngular, emptyRequirementCheck, updateDriveMaxActuatorVelocity)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->velocityDependentResistanceAngular, emptyRequirementCheck, updateDriveVelocityDependentResistance)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->speedEffortGradientAngular, emptyRequirementCheck, updateDriveSpeedEffortGradient)
    
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->armatureAngular, emptyRequirementCheck, updateArmaturePerAxis)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->maxJointVelocityAngular, emptyRequirementCheck, updateArticulationMaxJointVelocityPerAxis)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->staticFrictionEffortAngular, emptyRequirementCheck, updateArticulationStaticFrictionEffort)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->dynamicFrictionEffortAngular, emptyRequirementCheck, updateArticulationDynamicFrictionEffort)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->viscousFrictionCoefficientAngular, emptyRequirementCheck, updateArticulationViscousFrictionCoefficient)
    
    

    REGISTER_CHANGE(changeParams, "drive:angular:physics:damping", emptyRequirementCheck, updateDriveDamping)
    REGISTER_CHANGE(changeParams, "drive:angular:physics:stiffness", emptyRequirementCheck, updateDriveStiffness)
    REGISTER_CHANGE(changeParams, "drive:angular:physics:type", emptyRequirementCheck, updateDriveType)    
    REGISTER_CHANGE(changeParams, "state:angular:physics:position", emptyRequirementCheck, updateJointStatePosition)
    REGISTER_CHANGE(changeParams, "state:angular:physics:velocity", emptyRequirementCheck, updateJointStateVelocity)

    REGISTER_CHANGE(changeParams, "physxLimit:angular:bounceThreshold", emptyRequirementCheck, updateLimitBounceThreshold)
    REGISTER_CHANGE(changeParams, "physxLimit:angular:damping", emptyRequirementCheck, updateLimitDamping)
    REGISTER_CHANGE(changeParams, "physxLimit:angular:restitution", emptyRequirementCheck, updateLimitRestitution)
    REGISTER_CHANGE(changeParams, "physxLimit:angular:stiffness", emptyRequirementCheck, updateLimitStiffness)

    // prismatic joint
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsLowerLimit, emptyRequirementCheck, updateLimitLow)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsUpperLimit, emptyRequirementCheck, updateLimitHigh)
    REGISTER_CHANGE(changeParams, "drive:linear:physics:targetPosition", emptyRequirementCheck, updateDriveTargetPosition)
    REGISTER_CHANGE(changeParams, "drive:linear:physics:targetVelocity", emptyRequirementCheck, updateDriveTargetVelocity)
    REGISTER_CHANGE(changeParams, "drive:linear:physics:maxForce", emptyRequirementCheck, updateDriveMaxForce)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->maxActuatorVelocityLinear, emptyRequirementCheck, updateDriveMaxActuatorVelocity)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->velocityDependentResistanceLinear, emptyRequirementCheck, updateDriveVelocityDependentResistance)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->speedEffortGradientLinear, emptyRequirementCheck, updateDriveSpeedEffortGradient)
    
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->armatureLinear, emptyRequirementCheck, updateArmaturePerAxis)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->maxJointVelocityLinear, emptyRequirementCheck, updateArticulationMaxJointVelocityPerAxis)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->staticFrictionEffortLinear, emptyRequirementCheck, updateArticulationStaticFrictionEffort)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->dynamicFrictionEffortLinear, emptyRequirementCheck, updateArticulationDynamicFrictionEffort)
    REGISTER_CHANGE(changeParams, PhysxAxisInstanceTokens->viscousFrictionCoefficientLinear, emptyRequirementCheck, updateArticulationViscousFrictionCoefficient)
    

    REGISTER_CHANGE(changeParams, "drive:linear:physics:damping", emptyRequirementCheck, updateDriveDamping)
    REGISTER_CHANGE(changeParams, "drive:linear:physics:stiffness", emptyRequirementCheck, updateDriveStiffness)
    REGISTER_CHANGE(changeParams, "drive:linear:physics:type", emptyRequirementCheck, updateDriveType)    
    REGISTER_CHANGE(changeParams, "state:linear:physics:position", emptyRequirementCheck, updateJointStatePosition)
    REGISTER_CHANGE(changeParams, "state:linear:physics:velocity", emptyRequirementCheck, updateJointStateVelocity)

    REGISTER_CHANGE(changeParams, "physxLimit:linear:bounceThreshold", emptyRequirementCheck, updateLimitBounceThreshold)
    REGISTER_CHANGE(changeParams, "physxLimit:linear:damping", emptyRequirementCheck, updateLimitDamping)
    REGISTER_CHANGE(changeParams, "physxLimit:linear:restitution", emptyRequirementCheck, updateLimitRestitution)
    REGISTER_CHANGE(changeParams, "physxLimit:linear:stiffness", emptyRequirementCheck, updateLimitStiffness)

    // spherical joint
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsConeAngle0Limit, emptyRequirementCheck, updateLimitLow)
    REGISTER_CHANGE(
        changeParams, UsdPhysicsTokens.Get()->physicsConeAngle1Limit, emptyRequirementCheck, updateLimitHigh)

    // distance joint
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsMinDistance, emptyRequirementCheck, updateLimitLow)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsMaxDistance, emptyRequirementCheck, updateLimitHigh)

    // gear joint
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physicsGearRatio, emptyRequirementCheck, updateGearRatio)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physicsHinge0, emptyRequirementCheck, updateGearHinge0)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physicsHinge1, emptyRequirementCheck, updateGearHinge1)

    // rack and pinion joint
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physicsRatio, emptyRequirementCheck, updateRackPinionRatio)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physicsHinge, emptyRequirementCheck, updateRackHinge)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physicsPrismatic, emptyRequirementCheck, updateRackPrismatic)


    // D6 joints
    std::vector<std::string> driveAxis({ "rotX", "rotY", "rotZ", "transX", "transY", "transZ" });
    for (uint32_t iAxis = 0; iAxis < driveAxis.size(); iAxis++)
    {
        const std::string driveTargetNamePos = std::string("drive:") + driveAxis[iAxis] + std::string(":physics:targetPosition");
        const std::string driveTargetNameVel = std::string("drive:") + driveAxis[iAxis] + std::string(":physics:targetVelocity");
        const std::string driveMaxForceName = std::string("drive:") + driveAxis[iAxis] + std::string(":physics:maxForce");
        const std::string driveDampingName = std::string("drive:") + driveAxis[iAxis] + std::string(":physics:damping");
        const std::string driveStiffnesName = std::string("drive:") + driveAxis[iAxis] + std::string(":physics:stiffness");
        const std::string driveTypeName = std::string("drive:") + driveAxis[iAxis] + std::string(":physics:type");        
        // A.B. with heavy usage we might want to move to a separate callback per axis,
        // so that we dont have to compare axis strings
        REGISTER_CHANGE(changeParams, driveTargetNamePos.c_str(), emptyRequirementCheck, updateDriveTargetPosition)
        REGISTER_CHANGE(changeParams, driveTargetNameVel.c_str(), emptyRequirementCheck, updateDriveTargetVelocity)
        REGISTER_CHANGE(changeParams, driveMaxForceName.c_str(), emptyRequirementCheck, updateDriveMaxForce)
        REGISTER_CHANGE(changeParams, driveDampingName.c_str(), emptyRequirementCheck, updateDriveDamping)
        REGISTER_CHANGE(changeParams, driveStiffnesName.c_str(), emptyRequirementCheck, updateDriveStiffness)
        REGISTER_CHANGE(changeParams, driveTypeName.c_str(), emptyRequirementCheck, updateDriveType)
        //rotational axes for which envelope can be defined
        if (iAxis < 3) {

        
            REGISTER_CHANGE(changeParams, gDrivePerformanceEnvelopeMaxActuatorVelocityAttributeNameToken[iAxis], emptyRequirementCheck, updateDriveMaxActuatorVelocity)     
            REGISTER_CHANGE(changeParams, gDrivePerformanceEnvelopeVelocityDependentResistanceAttributeNameToken[iAxis], emptyRequirementCheck, updateDriveVelocityDependentResistance)   
            REGISTER_CHANGE(changeParams, gDrivePerformanceEnvelopeSpeedEffortGradientAttributeNameToken[iAxis], emptyRequirementCheck,updateDriveSpeedEffortGradient)
        }       
    }

    // spherical or D6 (which map to spherical px joints with drives) joints 
    std::vector<std::string> physxPropertiesAxis({ "rotX", "rotY", "rotZ"});
    for (uint32_t iAxis = 0; iAxis < physxPropertiesAxis.size(); iAxis++)
    {
        REGISTER_CHANGE(changeParams, gPhysxJointAxisMaxJointVelocityAttributeNameToken[iAxis], emptyRequirementCheck, updateArticulationMaxJointVelocityPerAxis)
        REGISTER_CHANGE(changeParams, gPhysxJointAxisArmatureAttributeNameToken[iAxis], emptyRequirementCheck, updateArmaturePerAxis)
        REGISTER_CHANGE(changeParams, gPhysxJointAxisStaticFrictionEffortAttributeNameToken[iAxis], emptyRequirementCheck, updateArticulationStaticFrictionEffort)
        REGISTER_CHANGE(changeParams, gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[iAxis], emptyRequirementCheck, updateArticulationDynamicFrictionEffort)
        REGISTER_CHANGE(changeParams, gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[iAxis], emptyRequirementCheck, updateArticulationViscousFrictionCoefficient)
    }

    // Distance joints
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPhysicsDistanceJointSpringDamping, emptyRequirementCheck, updateDistanceJointSpringDamping)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPhysicsDistanceJointSpringStiffness, emptyRequirementCheck, updateDistanceJointSpringStiffness)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPhysicsDistanceJointSpringEnabled, emptyRequirementCheck, updateDistanceJointSpringEnabled)
    

    std::vector<std::string> limitAxis({ "rotX", "rotY", "rotZ", "transX", "transY", "transZ", "distance" });
    for (uint32_t iAxis = 0; iAxis < limitAxis.size(); iAxis++)
    {
        const std::string limitHighName = std::string("limit:") + limitAxis[iAxis] + std::string(":physics:high");
        const std::string limitLowName = std::string("limit:") + limitAxis[iAxis] + std::string(":physics:low");
        REGISTER_CHANGE(changeParams, limitHighName.c_str(), emptyRequirementCheck, updateLimitHigh)
        REGISTER_CHANGE(changeParams, limitLowName.c_str(), emptyRequirementCheck, updateLimitLow)

        REGISTER_CHANGE(changeParams, std::string("physxLimit:") + limitAxis[iAxis] + std::string(":bounceThreshold"), emptyRequirementCheck, updateLimitBounceThreshold)
        REGISTER_CHANGE(changeParams, std::string("physxLimit:") + limitAxis[iAxis] + std::string(":damping"), emptyRequirementCheck, updateLimitDamping)
        REGISTER_CHANGE(changeParams, std::string("physxLimit:") + limitAxis[iAxis] + std::string(":restitution"), emptyRequirementCheck, updateLimitRestitution)
        REGISTER_CHANGE(changeParams, std::string("physxLimit:") + limitAxis[iAxis] + std::string(":stiffness"), emptyRequirementCheck, updateLimitStiffness)
    }

    std::vector<std::string> joinsStateAxis({ "rotX", "rotY", "rotZ", "transX", "transY", "transZ" });
    for (uint32_t iAxis = 0; iAxis < joinsStateAxis.size(); iAxis++)
    {
        const std::string jointStatePosition = std::string("state:") + joinsStateAxis[iAxis] + std::string(":physics:position");
        const std::string jointStateVelocity = std::string("state:") + joinsStateAxis[iAxis] + std::string(":physics:velocity");
        REGISTER_CHANGE(changeParams, jointStatePosition.c_str(), emptyRequirementCheck, updateJointStatePosition)
        REGISTER_CHANGE(changeParams, jointStateVelocity.c_str(), emptyRequirementCheck, updateJointStateVelocity)
    }

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxJointMaxJointVelocity, emptyRequirementCheck,
        updateArticulationMaxJointVelocity)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxJointJointFriction, emptyRequirementCheck,
        updateArticulationFrictionCoefficient)

    // articulation    
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxArticulationSolverPositionIterationCount,
                    emptyRequirementCheck, updateArticulationSolverPositionIterationCount)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxArticulationSolverVelocityIterationCount,
                    emptyRequirementCheck, updateArticulationSolverVelocityIterationCount)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxArticulationSleepThreshold, emptyRequirementCheck,
                    updateArticulationSleepThreshold)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxArticulationStabilizationThreshold, emptyRequirementCheck,
                    updateArticulationStabilizationThreshold)

    // PBD material
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialCohesion, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialAdhesion, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialParticleAdhesionScale, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialAdhesionOffsetScale, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialFriction, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialParticleFrictionScale, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialDamping, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialSurfaceTension, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialViscosity, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialVorticityConfinement, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialGravityScale, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialCflCoefficient, emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxPBDMaterialDensity, emptyRequirementCheck, updatePBDMaterialAttribute)

    // particle system
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->particleSystemEnabled, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->enableCCD, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->contactOffset, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->fluidRestOffset, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->maxDepenetrationVelocity, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->maxVelocity, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->particleContactOffset, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->restOffset, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->solidRestOffset, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->solverPositionIterationCount, emptyRequirementCheck, updateParticleSystemAttribute)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->wind, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->maxNeighborhood, emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->neighborhoodScale, emptyRequirementCheck, updateParticleSystemAttribute)

    // particle sets
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleParticleEnabled, physxParticleSetRequirementCheck, updateParticleSetEnabled)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleSelfCollision, physxParticleSetRequirementCheck, updateParticleSetSelfCollision)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleFluid, emptyRequirementCheck, updateParticleSetFluid)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleParticleGroup, physxParticleSetRequirementCheck, updateParticleSetParticleGroup)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleSimulationPoints, emptyRequirementCheck, updateParticleSimPositions)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->points, physxParticleSetPositionRequirementCheck, updateParticlePositions)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->positions, physxParticleSetPositionRequirementCheck, updateParticlePositions)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->velocities, physxParticleSetRequirementCheck, updateParticleVelocities)

    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsMass, physxParticleSetRequirementCheck, updateParticleDensity)
    REGISTER_CHANGE(changeParams, UsdPhysicsTokens.Get()->physicsDensity, physxParticleSetRequirementCheck, updateParticleDensity)

    // diffuse particles
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesDiffuseParticlesEnabled, emptyRequirementCheck, updateDiffuseParticlesEnabledAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesMaxDiffuseParticleMultiplier, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesThreshold, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesLifetime, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesAirDrag, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesBubbleDrag, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesBuoyancy, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesKineticEnergyWeight, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesPressureWeight, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesDivergenceWeight, emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxDiffuseParticlesCollisionDecay, emptyRequirementCheck, updateDiffuseParticlesAttribute)

    // anisotropy
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleAnisotropyParticleAnisotropyEnabled, emptyRequirementCheck, updateParticleAnisotropyEnabledAttribute)

    // smoothing
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleSmoothingParticleSmoothingEnabled, emptyRequirementCheck, updateParticleSmoothingEnabledAttribute)

    // isosurface
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleIsosurfaceIsosurfaceEnabled, emptyRequirementCheck, updateParticleIsosurfaceEnabledAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleIsosurfaceSurfaceDistance, emptyRequirementCheck, updateParticleIsosurfaceAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleIsosurfaceGridFilteringPasses, emptyRequirementCheck, updateParticleIsosurfaceAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleIsosurfaceGridSmoothingRadius, emptyRequirementCheck, updateParticleIsosurfaceAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleIsosurfaceNumMeshSmoothingPasses, emptyRequirementCheck, updateParticleIsosurfaceAttribute)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxParticleIsosurfaceNumMeshNormalSmoothingPasses, emptyRequirementCheck, updateParticleIsosurfaceAttribute)

    // deformables
    REGISTER_CHANGE_EXT(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsDeformableBodyEnabled, emptyRequirementCheck, physicsDeformableBodyResyncCheck, nullptr)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsAttachmentEnabled, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsFilterEnabled, emptyRequirementCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, UsdGeomTokens->faceVertexCounts, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, UsdGeomTokens->faceVertexIndices, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, UsdGeomTokens->tetVertexIndices, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->points, physicsDeformableSimRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->velocities, physicsDeformableSimRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE_EXT(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestTetVtxIndices, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsRestTriVtxIndices, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, PhysxSchemaTokens->physxDeformableBodyAutoDeformableBodyEnabled, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, PhysxSchemaTokens->physxDeformableBodyResolution, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, PhysxSchemaTokens->physxDeformableBodyAutoDeformableMeshSimplificationEnabled, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, PhysxSchemaTokens->physxDeformableBodyRemeshingEnabled, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, PhysxSchemaTokens->physxDeformableBodyRemeshingResolution, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, PhysxSchemaTokens->physxDeformableBodyTargetTriangleCount, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, PhysxSchemaTokens->physxDeformableBodyForceConforming, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsMass, physicsDeformableBodyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodySolverPositionIterationCount, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodyLinearDamping, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodyMaxLinearVelocity, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodySettlingDamping, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodySleepThreshold, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodySettlingThreshold, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodyMaxDepenetrationVelocity, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodySelfCollision, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodySelfCollisionFilterDistance, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodyEnableSpeculativeCCD, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodyDisableGravity, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxCollisionContactOffset, physicsDeformableCollisionCheck, updateDeformableContactOffset)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxCollisionRestOffset, physicsDeformableCollisionCheck, updateDeformableRestOffset)
    // Newton collision fallbacks (deformable).
    REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonContactMargin, physicsDeformableCollisionCheck, updateNewtonDeformableContactMargin)
    REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonContactGap, physicsDeformableCollisionCheck, updateNewtonDeformableContactGap)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodyCollisionPairUpdateFrequency, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableBodyCollisionIterationMultiplier, emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsDensity, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsDynamicFriction, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsStaticFriction, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsYoungsModulus, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsPoissonsRatio, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSurfaceThickness, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSurfaceStretchStiffness, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSurfaceShearStiffness, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSurfaceBendStiffness, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableMaterialElasticityDamping, emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens->physxDeformableMaterialBendDamping, emptyRequirementCheck, updateDeformableMaterial)


    // CCT
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxCharacterControllerSlopeLimit, emptyRequirementCheck, updateCctSlopeLimit)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->height, cctRequirementCheck, updateCctHeight)
    REGISTER_CHANGE(changeParams, UsdGeomTokens.Get()->radius, cctRequirementCheck, updateCctRadius)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxCharacterControllerContactOffset, emptyRequirementCheck, updateCctContactOffset)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxCharacterControllerStepOffset, emptyRequirementCheck, updateCctStepOffset)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxCharacterControllerUpAxis, emptyRequirementCheck, updateCctUpAxis)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxCharacterControllerNonWalkableMode, emptyRequirementCheck, updateCctNonWalkableMode)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxCharacterControllerClimbingMode, emptyRequirementCheck, updateCctClimbingMode)

    // vehicle
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleContextUpdateMode, emptyRequirementCheck, updateVehicleContextUpdateMode)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleContextVerticalAxis, emptyRequirementCheck, updateVehicleContextVerticalAxis)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleContextLongitudinalAxis, emptyRequirementCheck, updateVehicleContextLongitudinalAxis)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleEngineMoi, emptyRequirementCheck, updateVehicleEngineMomentOfInertia)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleEnginePeakTorque, emptyRequirementCheck, updateVehicleEnginePeakTorque)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleEngineMaxRotationSpeed, emptyRequirementCheck, updateVehicleEngineMaxRotationSpeed)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleEngineIdleRotationSpeed, emptyRequirementCheck, updateVehicleEngineIdleRotationSpeed)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleEngineTorqueCurve, emptyRequirementCheck, updateVehicleEngineTorqueCurve)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleEngineDampingRateFullThrottle, emptyRequirementCheck, updateVehicleEngineDampingRateFullThrottle)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleEngineDampingRateZeroThrottleClutchEngaged, emptyRequirementCheck, updateVehicleEngineDampingRateZeroThrottleClutchEngaged)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleEngineDampingRateZeroThrottleClutchDisengaged, emptyRequirementCheck, updateVehicleEngineDampingRateZeroThrottleClutchDisengaged)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->frictionValues, emptyRequirementCheck, updateVehicleTireFrictionTableFrictionValues)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->groundMaterials, emptyRequirementCheck, updateVehicleTireFrictionTableGroundMaterials)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->defaultFrictionValue, emptyRequirementCheck, updateVehicleTireFrictionTableDefaultFrictionValue)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionSpringStrength, emptyRequirementCheck, updateVehicleSuspensionSpringStrength)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionSpringDamperRate, emptyRequirementCheck, updateVehicleSuspensionSpringDamperRate)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionMaxCompression, emptyRequirementCheck, updateVehicleSuspensionMaxCompression)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionMaxDroop, emptyRequirementCheck, updateVehicleSuspensionMaxDroop)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionTravelDistance, emptyRequirementCheck, updateVehicleSuspensionTravelDistance)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionSprungMass, emptyRequirementCheck, updateVehicleSuspensionSprungMass)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionCamberAtRest, emptyRequirementCheck, updateVehicleSuspensionCamberAtRest)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionCamberAtMaxCompression, emptyRequirementCheck, updateVehicleSuspensionCamberAtMaxCompression)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionCamberAtMaxDroop, emptyRequirementCheck, updateVehicleSuspensionCamberAtMaxDroop)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireLatStiffX, emptyRequirementCheck, updateVehicleTireLatStiffX)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireLatStiffY, emptyRequirementCheck, updateVehicleTireLatStiffY)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireLateralStiffnessGraph, emptyRequirementCheck, updateVehicleTireLateralStiffnessGraph)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireLongitudinalStiffnessPerUnitGravity, emptyRequirementCheck, updateVehicleTireLongStiffPerGrav)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireLongitudinalStiffness, emptyRequirementCheck, updateVehicleTireLongitudinalStiffness)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireCamberStiffnessPerUnitGravity, emptyRequirementCheck, updateVehicleTireCamberStiffPerGrav)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireCamberStiffness, emptyRequirementCheck, updateVehicleTireCamberStiffness)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireFrictionVsSlipGraph, emptyRequirementCheck, updateVehicleTireFrictionVsSlip)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireFrictionTable, emptyRequirementCheck, updateVehicleTireFrictionTableRel)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTireRestLoad, emptyRequirementCheck, updateVehicleTireRestLoad)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelRadius, emptyRequirementCheck, updateVehicleWheelRadius)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelWidth, emptyRequirementCheck, updateVehicleWheelWidth)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelMass, emptyRequirementCheck, updateVehicleWheelMass)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelMoi, emptyRequirementCheck, updateVehicleWheelMomentOfInertia)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelDampingRate, emptyRequirementCheck, updateVehicleWheelDampingRate)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelMaxBrakeTorque, emptyRequirementCheck, updateVehicleWheelMaxBrakeTorque)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelMaxHandBrakeTorque, emptyRequirementCheck, updateVehicleWheelMaxHandBrakeTorque)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelMaxSteerAngle, emptyRequirementCheck, updateVehicleWheelMaxSteerAngle)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelToeAngle, emptyRequirementCheck, updateVehicleWheelToeAngle)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentIndex, emptyRequirementCheck, updateVehicleWheelAttachmentIndex)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentWheel, emptyRequirementCheck, updateVehicleWheelAttachmentWheel)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentTire, emptyRequirementCheck, updateVehicleWheelAttachmentTire)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentSuspension, emptyRequirementCheck, updateVehicleWheelAttachmentSuspension)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentSuspensionTravelDirection, emptyRequirementCheck, updateVehicleWheelAttachmentSuspensionTravelDirection)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentSuspensionForceAppPointOffset, emptyRequirementCheck, updateVehicleWheelAttachmentSuspensionForceAppPointOffset)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentWheelCenterOfMassOffset, emptyRequirementCheck, updateVehicleWheelAttachmentWheelCenterOfMassOffset)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentTireForceAppPointOffset, emptyRequirementCheck, updateVehicleWheelAttachmentTireForceAppPointOffset)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentSuspensionFramePosition, emptyRequirementCheck, updateVehicleWheelAttachmentSuspensionFramePosition)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentSuspensionFrameOrientation, emptyRequirementCheck, updateVehicleWheelAttachmentSuspensionFrameOrientation)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentWheelFramePosition, emptyRequirementCheck, updateVehicleWheelAttachmentWheelFramePosition)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentWheelFrameOrientation, emptyRequirementCheck, updateVehicleWheelAttachmentWheelFrameOrientation)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentDriven, emptyRequirementCheck, updateVehicleWheelAttachmentDriven)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelAttachmentCollisionGroup, emptyRequirementCheck, updateVehicleWheelAttachmentCollisionGroup)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionComplianceWheelToeAngle, emptyRequirementCheck, updateVehicleSuspensionComplWheelToeAngle)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionComplianceWheelCamberAngle, emptyRequirementCheck, updateVehicleSuspensionComplWheelCamberAngle)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionComplianceSuspensionForceAppPoint, emptyRequirementCheck, updateVehicleSuspensionComplSuspForceAppPoint)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSuspensionComplianceTireForceAppPoint, emptyRequirementCheck, updateVehicleSuspensionComplTireForceAppPoint)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleVehicleEnabled, emptyRequirementCheck, updateVehicleEnabled)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleLimitSuspensionExpansionVelocity, emptyRequirementCheck, updateVehicleLimitSuspensionExpansionVelocity)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleMinPassiveLongitudinalSlipDenominator, emptyRequirementCheck, updateVehicleMinPassiveLongslipDenom)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleMinActiveLongitudinalSlipDenominator, emptyRequirementCheck, updateVehicleMinActiveLongslipDenom)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleMinLateralSlipDenominator, emptyRequirementCheck, updateVehicleMinLateralSlipDenom)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleLongitudinalStickyTireThresholdSpeed, emptyRequirementCheck, updateVehicleLongitudinalStickyTireThresholdSpeed)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleLongitudinalStickyTireThresholdTime, emptyRequirementCheck, updateVehicleLongitudinalStickyTireThresholdTime)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleLongitudinalStickyTireDamping, emptyRequirementCheck, updateVehicleLongitudinalStickyTireDamping)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleLateralStickyTireThresholdSpeed, emptyRequirementCheck, updateVehicleLateralStickyTireThresholdSpeed)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleLateralStickyTireThresholdTime, emptyRequirementCheck, updateVehicleLateralStickyTireThresholdTime)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleLateralStickyTireDamping, emptyRequirementCheck, updateVehicleLateralStickyTireDamping)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerAccelerator, emptyRequirementCheck, updateVehicleControllerAccelerator)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerBrake0, emptyRequirementCheck, updateVehicleControllerBrake0)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerBrake1, emptyRequirementCheck, updateVehicleControllerBrake1)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerBrake, emptyRequirementCheck, updateVehicleControllerBrake)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerHandbrake, emptyRequirementCheck, updateVehicleControllerHandbrake)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerSteer, emptyRequirementCheck, updateVehicleControllerSteer)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerSteerLeft, emptyRequirementCheck, updateVehicleControllerSteerLeft)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerSteerRight, emptyRequirementCheck, updateVehicleControllerSteerRight)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleControllerTargetGear, emptyRequirementCheck, updateVehicleControllerTargetGear)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTankControllerThrust0, emptyRequirementCheck, updateVehicleTankControllerThrust0)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTankControllerThrust1, emptyRequirementCheck, updateVehicleTankControllerThrust1)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleDriveBasicPeakTorque, emptyRequirementCheck, updateVehicleDriveBasicPeakTorque)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelControllerDriveTorque, emptyRequirementCheck, updateVehicleWheelControllerDriveTorque)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelControllerBrakeTorque, emptyRequirementCheck, updateVehicleWheelControllerBrakeTorque)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleWheelControllerSteerAngle, emptyRequirementCheck, updateVehicleWheelControllerSteerAngle)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleMultiWheelDifferentialWheels, emptyRequirementCheck, updateVehicleMultiWheelDifferentialWheels)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleMultiWheelDifferentialTorqueRatios, emptyRequirementCheck, updateVehicleMultiWheelDifferentialTorqueRatios)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleMultiWheelDifferentialAverageWheelSpeedRatios, emptyRequirementCheck, updateVehicleMultiWheelDifferentialAverageWheelSpeedRatios)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTankDifferentialNumberOfWheelsPerTrack, emptyRequirementCheck, updateVehicleTankDifferentialNumberOfWheelsPerTrack)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTankDifferentialThrustIndexPerTrack, emptyRequirementCheck, updateVehicleTankDifferentialThrustIndexPerTrack)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTankDifferentialTrackToWheelIndices, emptyRequirementCheck, updateVehicleTankDifferentialTrackToWheelIndices)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleTankDifferentialWheelIndicesInTrackOrder, emptyRequirementCheck, updateVehicleTankDifferentialWheelIndicesInTrackOrder)

    UsdSchemaRegistry& schemaRegistry = UsdSchemaRegistry::GetInstance();
    const TfToken& brakes0 = PhysxSchemaTokens.Get()->brakes0;
    const TfToken& brakes1 = PhysxSchemaTokens.Get()->brakes1;
    const TfToken& wheels = PhysxSchemaTokens.Get()->physxVehicleBrakes_MultipleApplyTemplate_Wheels;
    const TfToken& maxBrakeTorque = PhysxSchemaTokens.Get()->physxVehicleBrakes_MultipleApplyTemplate_MaxBrakeTorque;
    const TfToken& torqueMultipliers = PhysxSchemaTokens.Get()->physxVehicleBrakes_MultipleApplyTemplate_TorqueMultipliers;
    const std::string vehicleBrakes0Wheels = schemaRegistry.MakeMultipleApplyNameInstance(wheels, brakes0);
    const std::string vehicleBrakes1Wheels = schemaRegistry.MakeMultipleApplyNameInstance(wheels, brakes1);
    const std::string vehicleBrakes0MaxBrakeTorque = schemaRegistry.MakeMultipleApplyNameInstance(maxBrakeTorque, brakes0);
    const std::string vehicleBrakes1MaxBrakeTorque = schemaRegistry.MakeMultipleApplyNameInstance(maxBrakeTorque, brakes1);
    const std::string vehicleBrakes0TorqueMultipliers = schemaRegistry.MakeMultipleApplyNameInstance(torqueMultipliers, brakes0);
    const std::string vehicleBrakes1TorqueMultipliers = schemaRegistry.MakeMultipleApplyNameInstance(torqueMultipliers, brakes1);

    REGISTER_CHANGE(changeParams, vehicleBrakes0Wheels, emptyRequirementCheck, updateVehicleBrakes0Wheels)
    REGISTER_CHANGE(changeParams, vehicleBrakes1Wheels, emptyRequirementCheck, updateVehicleBrakes1Wheels)
    REGISTER_CHANGE(changeParams, vehicleBrakes0MaxBrakeTorque, emptyRequirementCheck, updateVehicleBrakes0MaxBrakeTorque)
    REGISTER_CHANGE(changeParams, vehicleBrakes1MaxBrakeTorque, emptyRequirementCheck, updateVehicleBrakes1MaxBrakeTorque)
    REGISTER_CHANGE(changeParams, vehicleBrakes0TorqueMultipliers, emptyRequirementCheck, updateVehicleBrakes0TorqueMultipliers)
    REGISTER_CHANGE(changeParams, vehicleBrakes1TorqueMultipliers, emptyRequirementCheck, updateVehicleBrakes1TorqueMultipliers)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSteeringWheels, emptyRequirementCheck, updateVehicleSteeringWheels)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSteeringMaxSteerAngle, emptyRequirementCheck, updateVehicleSteeringMaxSteerAngle)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleSteeringAngleMultipliers, emptyRequirementCheck, updateVehicleSteeringAngleMultipliers)

    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleAckermannSteeringWheel0, emptyRequirementCheck, updateVehicleAckermannSteeringWheel0)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleAckermannSteeringWheel1, emptyRequirementCheck, updateVehicleAckermannSteeringWheel1)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleAckermannSteeringMaxSteerAngle, emptyRequirementCheck, updateVehicleAckermannSteeringMaxSteerAngle)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleAckermannSteeringWheelBase, emptyRequirementCheck, updateVehicleAckermannSteeringWheelBase)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleAckermannSteeringTrackWidth, emptyRequirementCheck, updateVehicleAckermannSteeringTrackWidth)
    REGISTER_CHANGE(changeParams, PhysxSchemaTokens.Get()->physxVehicleAckermannSteeringStrength, emptyRequirementCheck, updateVehicleAckermannSteeringStrength)

    const TfToken& drive = PhysxSchemaTokens.Get()->drive;
    const TfToken& steer = PhysxSchemaTokens.Get()->steer;
    const TfToken& commandValues = PhysxSchemaTokens.Get()->physxVehicleNCR_MultipleApplyTemplate_CommandValues;
    const TfToken& speedResponsesPerCommandValue = PhysxSchemaTokens.Get()->physxVehicleNCR_MultipleApplyTemplate_SpeedResponsesPerCommandValue;
    const TfToken& speedResponses = PhysxSchemaTokens.Get()->physxVehicleNCR_MultipleApplyTemplate_SpeedResponses;

    const TfToken* ncrInstanceTokens[] = { &drive, &steer, &brakes0, &brakes1 };
    constexpr uint32_t ncrInstanceTokenCount = sizeof(ncrInstanceTokens) / sizeof(ncrInstanceTokens[0]);
    OnUpdateObjectFn ncrCommandValuesUpdateMethods[ncrInstanceTokenCount] = {
        updateVehicleNCRDriveCommandValues,
        updateVehicleNCRSteerCommandValues,
        updateVehicleNCRBrakes0CommandValues,
        updateVehicleNCRBrakes1CommandValues
    };
    OnUpdateObjectFn ncrSpeedResponsesPerCommandValueUpdateMethods[ncrInstanceTokenCount] = {
        updateVehicleNCRDriveSpeedResponsesPerCommandValue,
        updateVehicleNCRSteerSpeedResponsesPerCommandValue,
        updateVehicleNCRBrakes0SpeedResponsesPerCommandValue,
        updateVehicleNCRBrakes1SpeedResponsesPerCommandValue
    };
    OnUpdateObjectFn ncrSpeedResponsesUpdateMethods[ncrInstanceTokenCount] = {
        updateVehicleNCRDriveSpeedResponses,
        updateVehicleNCRSteerSpeedResponses,
        updateVehicleNCRBrakes0SpeedResponses,
        updateVehicleNCRBrakes1SpeedResponses
    };
    
    for (uint32_t i = 0; i < ncrInstanceTokenCount; i++)
    {
        const TfToken& instanceToken = *ncrInstanceTokens[i];

        const std::string vehicleNCRCommandValues = schemaRegistry.MakeMultipleApplyNameInstance(commandValues, instanceToken);
        REGISTER_CHANGE(changeParams, vehicleNCRCommandValues, emptyRequirementCheck, ncrCommandValuesUpdateMethods[i]);

        const std::string vehicleNCRSpeedResponsesPerCommandValue = schemaRegistry.MakeMultipleApplyNameInstance(speedResponsesPerCommandValue, instanceToken);
        REGISTER_CHANGE(changeParams, vehicleNCRSpeedResponsesPerCommandValue, emptyRequirementCheck, ncrSpeedResponsesPerCommandValueUpdateMethods[i]);

        const std::string vehicleNCRSpeedResponses = schemaRegistry.MakeMultipleApplyNameInstance(speedResponses, instanceToken);
        REGISTER_CHANGE(changeParams, vehicleNCRSpeedResponses, emptyRequirementCheck, ncrSpeedResponsesUpdateMethods[i]);
    }

    // mimic joint

    // extra complexity because mimic joints are using a multiple apply API schema
    const TfToken& mimicJointRotX = PhysxSchemaTokens.Get()->rotX;
    const TfToken& mimicJointRotY = PhysxSchemaTokens.Get()->rotY;
    const TfToken& mimicJointRotZ = PhysxSchemaTokens.Get()->rotZ;
    const TfToken& mimicJointGearing = PhysxSchemaTokens.Get()->physxMimicJoint_MultipleApplyTemplate_Gearing;
    const TfToken& mimicJointOffset = PhysxSchemaTokens.Get()->physxMimicJoint_MultipleApplyTemplate_Offset;
    const TfToken& mimicJointNaturalFrequency = PhysxSchemaTokens.Get()->physxMimicJoint_MultipleApplyTemplate_NaturalFrequency;
    const TfToken& mimicJointDampingRatio = PhysxSchemaTokens.Get()->physxMimicJoint_MultipleApplyTemplate_DampingRatio;
    const TfToken& mimicJointReferenceJoint = PhysxSchemaTokens.Get()->physxMimicJoint_MultipleApplyTemplate_ReferenceJoint;
    const TfToken& mimicJointReferenceJointAxis = PhysxSchemaTokens.Get()->physxMimicJoint_MultipleApplyTemplate_ReferenceJointAxis;

    const TfToken* mimicJointInstanceTokens[] = { &mimicJointRotX, &mimicJointRotY, &mimicJointRotZ };
    constexpr uint32_t mimicJointInstanceTokenCount = sizeof(mimicJointInstanceTokens) / sizeof(mimicJointInstanceTokens[0]);

    OnPrimRequirementCheckFn mimicJointRequirementCheckMethods[mimicJointInstanceTokenCount] = {
        physxMimicJointRequirementCheck<SchemaAPIFlag::eMimicJointRotXAPI>,
        physxMimicJointRequirementCheck<SchemaAPIFlag::eMimicJointRotYAPI>,
        physxMimicJointRequirementCheck<SchemaAPIFlag::eMimicJointRotZAPI>
    };

    OnUpdateObjectFn mimicJointGearingUpdateMethods[mimicJointInstanceTokenCount] = {
        updateMimicJointGearing,
        updateMimicJointGearing,
        updateMimicJointGearing
    };

    OnUpdateObjectFn mimicJointOffsetUpdateMethods[mimicJointInstanceTokenCount] = {
        updateMimicJointOffset,
        updateMimicJointOffset,
        updateMimicJointOffset
    };

    OnUpdateObjectFn mimicJointNaturalFrequencyUpdateMethods[mimicJointInstanceTokenCount] = {
        updateMimicJointNaturalFrequency,
        updateMimicJointNaturalFrequency,
        updateMimicJointNaturalFrequency
    };

    OnUpdateObjectFn mimicJointDampingRatioUpdateMethods[mimicJointInstanceTokenCount] = {
        updateMimicJointDampingRatio,
        updateMimicJointDampingRatio,
        updateMimicJointDampingRatio
    };

    
    for (uint32_t i = 0; i < mimicJointInstanceTokenCount; i++)
    {
        const TfToken& instanceToken = *mimicJointInstanceTokens[i];

        const std::string mimicJointGearingName = schemaRegistry.MakeMultipleApplyNameInstance(mimicJointGearing, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointGearingName, mimicJointRequirementCheckMethods[i], mimicJointGearingUpdateMethods[i]);

        const std::string mimicJointOffsetName = schemaRegistry.MakeMultipleApplyNameInstance(mimicJointOffset, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointOffsetName, mimicJointRequirementCheckMethods[i], mimicJointOffsetUpdateMethods[i]);

        const std::string mimicJointNaturalFrequencyName = schemaRegistry.MakeMultipleApplyNameInstance(mimicJointNaturalFrequency, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointNaturalFrequencyName, mimicJointRequirementCheckMethods[i], mimicJointNaturalFrequencyUpdateMethods[i]);

        const std::string mimicJointDampingRatioName = schemaRegistry.MakeMultipleApplyNameInstance(mimicJointDampingRatio, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointDampingRatioName, mimicJointRequirementCheckMethods[i], mimicJointDampingRatioUpdateMethods[i]);

        // changing the reference joint relationship should trigger a release and re-parsing
        const std::string mimicJointReferenceJointName = schemaRegistry.MakeMultipleApplyNameInstance(mimicJointReferenceJoint, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointReferenceJointName, mimicJointRequirementCheckMethods[i], nullptr);

        // changing the reference joint axis should trigger a release and re-parsing
        const std::string mimicJointReferenceJointAxisName = schemaRegistry.MakeMultipleApplyNameInstance(mimicJointReferenceJointAxis, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointReferenceJointAxisName, mimicJointRequirementCheckMethods[i], nullptr);
    }

    // Newton mimic API (single-apply)
    {
        OnPrimRequirementCheckFn newtonMimicRequirementCheck =
            physxMimicJointRequirementCheck<SchemaAPIFlag::eNewtonMimicAPI>;

        REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonMimicCoef1, newtonMimicRequirementCheck, updateNewtonMimicJointCoef1);
        REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonMimicCoef0, newtonMimicRequirementCheck, updateNewtonMimicJointCoef0);

        // mimicEnabled and the mimicJoint relationship changes trigger a structural release+reparse.
        REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonMimicEnabled, newtonMimicRequirementCheck, nullptr);
        REGISTER_CHANGE(changeParams, NewtonSchemaTokens->newtonMimicJoint, newtonMimicRequirementCheck, nullptr);
    }
}

#undef REGISTER_CHANGE

} // namespace physx
} // namespace omni
