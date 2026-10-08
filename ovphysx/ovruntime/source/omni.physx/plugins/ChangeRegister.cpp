// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BODY-001
 * @covers AC-5
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-1
 */

// The requirement-check functions here are ObjectKey/TokenId-keyed, matching
// PrimChangeMap's TokenId-keyed dispatch map (PropertyChangeMap, PrimUpdate.h).
// Prim-TYPE (isA) gates go through KnownTokens rather than a C++ schema type.
//
// testTimeSampledAttribute()/register*TimeSampledChanges() are a separate mechanism: they
// feed AttachedStage::mTimeSampledAttributes, consumed directly by PrimUpdate.cpp's
// processUpdates(), not by PrimChangeMap.

#include "ChangeRegister.h"
#include "PhysXTools.h"

#include <omni/physics/parse/KnownTokens.h>
#include "usdLoad/LoadUsd.h"
#include "usdLoad/CollisionGroup.h"
#include "usdLoad/ChangeParams.h"

#include "propertiesUpdate/PhysXPropertiesUpdate.h"
#include "internal/InternalScene.h"
#include "usdInterface/UsdInterface.h"

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysxTokens.h>

#include <omni/physics/parse/KnownTokens.h>

#include <PxPhysicsAPI.h>

using namespace omni::physx::usdparser;

extern bool updateMaterialDensity(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId, omni::physics::parse::ReadTime);

using namespace omni::physx::internal;
using namespace physx;

namespace omni
{
namespace physx
{

// Per-axis attribute-name tables for the rotational drive/joint axes. Kept in
// this TU (they are only consumed by registerChangeParams below) so the seven
// arrays are not duplicated into every translation unit that includes
// ChangeRegister.h. Literal values are the physxJointAxis/physxDrivePerformanceEnvelope
// multiple-apply instance names (rotX/rotY/rotZ), verified against
// physxSchema/tokens.h's ..._MultipleApplyTemplate_... doc comments.
static const char* gDrivePerformanceEnvelopeMaxActuatorVelocityAttributeNameToken[3] = {
    "physxDrivePerformanceEnvelope:rotX:maxActuatorVelocity",
    "physxDrivePerformanceEnvelope:rotY:maxActuatorVelocity",
    "physxDrivePerformanceEnvelope:rotZ:maxActuatorVelocity"
};

static const char* gDrivePerformanceEnvelopeVelocityDependentResistanceAttributeNameToken[3] = {
    "physxDrivePerformanceEnvelope:rotX:velocityDependentResistance",
    "physxDrivePerformanceEnvelope:rotY:velocityDependentResistance",
    "physxDrivePerformanceEnvelope:rotZ:velocityDependentResistance",
};

static const char* gDrivePerformanceEnvelopeSpeedEffortGradientAttributeNameToken[3] = {
    "physxDrivePerformanceEnvelope:rotX:speedEffortGradient",
    "physxDrivePerformanceEnvelope:rotY:speedEffortGradient",
    "physxDrivePerformanceEnvelope:rotZ:speedEffortGradient",
};

static const char* gPhysxJointAxisMaxJointVelocityAttributeNameToken[3] = {
    "physxJointAxis:rotX:maxJointVelocity",
    "physxJointAxis:rotY:maxJointVelocity",
    "physxJointAxis:rotZ:maxJointVelocity",
};
static const char* gPhysxJointAxisArmatureAttributeNameToken[3] = {
    "physxJointAxis:rotX:armature",
    "physxJointAxis:rotY:armature",
    "physxJointAxis:rotZ:armature",
};
static const char* gPhysxJointAxisStaticFrictionEffortAttributeNameToken[3] = {
    "physxJointAxis:rotX:staticFrictionEffort",
    "physxJointAxis:rotY:staticFrictionEffort",
    "physxJointAxis:rotZ:staticFrictionEffort",
};
static const char* gPhysxJointAxisDynamicFrictionEffortAttributeNameToken[3] = {
    "physxJointAxis:rotX:dynamicFrictionEffort",
    "physxJointAxis:rotY:dynamicFrictionEffort",
    "physxJointAxis:rotZ:dynamicFrictionEffort",
};
static const char* gPhysxJointAxisViscousFrictionCoefficientAttributeNameToken[3] = {
    "physxJointAxis:rotX:viscousFrictionCoefficient",
    "physxJointAxis:rotY:viscousFrictionCoefficient",
    "physxJointAxis:rotZ:viscousFrictionCoefficient",
};

//
// note: if updateObjectFn is set to nullptr, the object will get released and the prim parsed again
//
#define REGISTER_CHANGE(changeParams, attribute, primCheckFn, updateObjectFn)                                          \
    {                                                                                                                  \
        ChangeParams cp{ attribute, updateObjectFn, primCheckFn, nullptr };                                            \
        changeParams.push_back(cp);                                                                                    \
    }

#define REGISTER_CHANGE_EXT(changeParams, attribute, primCheckFn, primCheckExtFn, updateObjectFn)                      \
    {                                                                                                                  \
        ChangeParams cp{ attribute, updateObjectFn, primCheckFn, primCheckExtFn };                                     \
        changeParams.push_back(cp);                                                                                    \
    }

// Requirement-check functions. REGISTER_CHANGE/REGISTER_CHANGE_EXT and
// addToStageSpecificAttributeMap(Ext) below assign these into
// ChangeParams::onPrimCheckKey/onPrimCheckExtKey.
static bool emptyRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey, omni::physics::parse::TokenId)
{
    return true;
}

static bool bodyRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(primKey) && (storedAPIs & SchemaAPIFlag::eRigidBodyAPI) != 0;
}

static bool noBodyRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(primKey) && !(storedAPIs & SchemaAPIFlag::eRigidBodyAPI);
}

static bool bodyPointInstancerRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->isA(primKey, tok.pointInstancerType) && !src->hasSchema(primKey, tok.physxParticleAPI);
}

static bool collisionRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(primKey) && (storedAPIs & SchemaAPIFlag::eCollisionAPI) != 0;
}

static bool jointRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (src)
    {
        const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
        if (src->isA(primKey, tok.physicsJoint))
            return true;
    }

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

static bool materialRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->hasSchema(primKey, tok.physicsMaterialAPI);
}

static bool physxMaterialRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->hasSchema(primKey, tok.physicsMaterialAPI) && src->hasSchema(primKey, tok.physxMaterialAPI);
}

static bool jointEnableDisableRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (!src->isA(primKey, tok.physicsJoint))
        return false;

    bool val;
    if (!getValue<bool>(attachedStage, primKey, propName, omni::physics::parse::ReadTime::defaultTime(), val))
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

static bool physxParticleSetRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(primKey) && (storedAPIs & SchemaAPIFlag::eParticleSetAPI) != 0;
}

static bool physxParticleSetPositionRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);

    if (src->exists(primKey) && (storedAPIs & SchemaAPIFlag::eParticleSetAPI))
    {
        const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
        return !src->hasAuthoredAttribute(primKey, tok.physxParticleSimulationPoints);
    }
    return false;
}

static bool physicsDeformableBodyRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return src && src->exists(primKey) && (storedAPIs & SchemaAPIFlag::eDeformableBodyAPI) != 0;
}

static bool physicsDeformableBodyResyncCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName, omni::physics::parse::ObjectKey& resyncKey)
{
    // deformables require resync if disabled/enabled
    resyncKey = primKey;
    return true;
}

static bool physicsDeformableBodyHierarchyCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    if (!src->exists(primKey))
        return false;

    const auto* db = attachedStage.getObjectDatabase();
    if ((db->getSchemaAPIs(primKey) & SchemaAPIFlag::eDeformableBodyAPI) > 0)
        return true;

    for (omni::physics::parse::ObjectKey p = src->getParent(primKey); p.valid(); p = src->getParent(p))
    {
        if (db->getSchemaAPIs(p) & SchemaAPIFlag::eDeformableBodyAPI)
            return true;
    }
    return false;
}

static bool physicsDeformableCollisionCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    bool isDeformableBodyHier = physicsDeformableBodyHierarchyCheck(attachedStage, primKey, propName);
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

static bool physicsRigidCollisionCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    bool isDeformableBodyHier = physicsDeformableBodyHierarchyCheck(attachedStage, primKey, propName);
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

static bool physicsDeformableBodyHierarchyResyncCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName, omni::physics::parse::ObjectKey& resyncKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const auto* db = attachedStage.getObjectDatabase();

    const uint64_t storedAPIs = db->getSchemaAPIs(primKey);
    if ((storedAPIs & SchemaAPIFlag::eDeformableBodyAPI) > 0)
    {
        resyncKey = primKey;
        return true;
    }

    if ((storedAPIs & (SchemaAPIFlag::eDeformablePoseAPI | SchemaAPIFlag::eVolumeDeformableSimAPI | SchemaAPIFlag::eSurfaceDeformableSimAPI | SchemaAPIFlag::eCollisionAPI)) > 0)
    {
        for (omni::physics::parse::ObjectKey p = src->getParent(primKey); p.valid(); p = src->getParent(p))
        {
            if (db->getSchemaAPIs(p) & SchemaAPIFlag::eDeformableBodyAPI)
            {
                resyncKey = p;
                return true;
            }
        }
    }
    return false;
}

static bool physicsDeformableSimRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    return (storedAPIs & (SchemaAPIFlag::eVolumeDeformableSimAPI | SchemaAPIFlag::eSurfaceDeformableSimAPI)) != 0;
}

static bool cctRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->hasSchema(primKey, tok.physxCharacterControllerAPI) && src->isA(primKey, tok.capsuleType);
}

static bool collisionGroupRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;

    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (src->isA(primKey, tok.physicsCollisionGroupType))
    {
        appendCollisionGroupFromPath(attachedStage, primKey, attachedStage.getCollisionGroupMap());
        return true;
    }
    return false;
}

static bool physxSpatialTendonRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->isA(primKey, tok.xformableType) && (storedAPIs & SchemaAPIFlag::eRigidBodyAPI) &&
           src->hasSchema(primKey, tok.physxTendonAttachmentRootAPI);
}

static bool physxRigidBodyAttachmentRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->isA(primKey, tok.xformableType) && (storedAPIs & SchemaAPIFlag::eRigidBodyAPI) &&
           src->hasSchema(primKey, tok.physxTendonAttachmentAPI);
}

static bool physxTendonAttachmentLeafRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->isA(primKey, tok.xformableType) && (storedAPIs & SchemaAPIFlag::eRigidBodyAPI) &&
           src->hasSchema(primKey, tok.physxTendonAttachmentLeafAPI);
}

static bool physxFixedTendonRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->isA(primKey, tok.physicsJoint) && src->hasSchema(primKey, tok.physxTendonAxisRootAPI);
}

static bool physxTendonAxisRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    return src->isA(primKey, tok.physicsJoint) && src->hasSchema(primKey, tok.physxTendonAxisAPI);
}

template<SchemaAPIFlag::Enum tSchemaFlag>
static bool physxMimicJointRequirementCheck(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId propName)
{
    const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primKey);

    if (storedAPIs & tSchemaFlag)
        return true;
    else
        return false;
}

// Live-notify time-sampled attribute registration (ChangeRegister.h's own top comment).
// A backend that never reports an attribute as time-sampled registers nothing here.
void testTimeSampledAttribute(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, omni::physics::parse::TokenId attributeId, usdparser::OnUpdateObjectFn onUpdate)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (src && src->isAttributeTimeSampled(primKey, attributeId))
    {
        attachedStage.registerTimeSampledAttribute(primKey, attributeId, onUpdate);
    }
}

void registerDriveTimeSampledChanges(AttachedStage& attachedStage, omni::physics::parse::ObjectKey jointPrimKey, std::string driveAxis)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    testTimeSampledAttribute(attachedStage, jointPrimKey, src->internToken(driveAxis + ":physics:targetPosition"), updateDriveTargetPosition);
    testTimeSampledAttribute(attachedStage, jointPrimKey, src->internToken(driveAxis + ":physics:targetVelocity"), updateDriveTargetVelocity);
    testTimeSampledAttribute(attachedStage, jointPrimKey, src->internToken(driveAxis + ":physics:maxForce"), updateDriveMaxForce);
    testTimeSampledAttribute(attachedStage, jointPrimKey, src->internToken(driveAxis + ":physics:damping"), updateDriveDamping);
    testTimeSampledAttribute(attachedStage, jointPrimKey, src->internToken(driveAxis + ":physics:stiffness"), updateDriveStiffness);
    testTimeSampledAttribute(attachedStage, jointPrimKey, src->internToken(driveAxis + ":physics:type"), updateDriveType);
}

// @implements REQ-LOAD-TOKENS-001
void registerJointTimeSampledChanges(AttachedStage& attachedStage, omni::physics::parse::ObjectKey jointPrimKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    testTimeSampledAttribute(attachedStage, jointPrimKey, tok.physicsLocalPos0, updateLocalPos0);
    testTimeSampledAttribute(attachedStage, jointPrimKey, tok.physicsLocalPos1, updateLocalPos1);
    testTimeSampledAttribute(attachedStage, jointPrimKey, tok.physicsLocalRot0, updateLocalRot0);
    testTimeSampledAttribute(attachedStage, jointPrimKey, tok.physicsLocalRot1, updateLocalRot1);
}

void registerSceneTimeSampledChanges(AttachedStage& attachedStage, omni::physics::parse::ObjectKey scenePrimKey)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    testTimeSampledAttribute(attachedStage, scenePrimKey, tok.gravityMagnitude, updateGravityMagnitude);
    testTimeSampledAttribute(attachedStage, scenePrimKey, tok.gravityDirection, updateGravityDirection);
}

static void addToStageSpecificAttributeMap(AttachedStage& attachedStage, std::string attributeName, OnPrimRequirementKeyCheckFn checkFn,
    OnUpdateObjectFn updateFn)
{
    ChangeParams cp{ attributeName, updateFn, checkFn, nullptr };
    attachedStage.registerStageSpecificAttribute(cp);
}

static void addToStageSpecificAttributeMapExt(AttachedStage& attachedStage, std::string attributeName, OnPrimRequirementKeyCheckFn checkFn,
    OnPrimRequirementExtKeyCheckFn checkExtFn, OnUpdateObjectFn updateFn)
{
    ChangeParams cp{ attributeName, updateFn, checkFn, checkExtFn };
    attachedStage.registerStageSpecificAttribute(cp);
}

// Substitutes the instance-name placeholder in a multi-apply schema attribute-name
// template (e.g. "physxMimicJoint:__INSTANCE_NAME__:gearing", the literal value of the
// schema's ..._MultipleApplyTemplate_... token) with the given instance name -- the
// source-agnostic equivalent of pxr's UsdSchemaRegistry::MakeMultipleApplyNameInstance.
static std::string makeMultiApplyAttributeName(const char* nameTemplate, const std::string& instanceName)
{
    static constexpr char kInstanceNamePlaceholder[] = "__INSTANCE_NAME__";
    std::string result(nameTemplate);
    const size_t pos = result.find(kInstanceNamePlaceholder);
    if (pos != std::string::npos)
        result.replace(pos, sizeof(kInstanceNamePlaceholder) - 1, instanceName);
    return result;
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
    const std::string pointsAttrName = makeMultiApplyAttributeName("deformablePose:__INSTANCE_NAME__:omniphysics:points", instanceName);
    const std::string purposesAttrName = makeMultiApplyAttributeName("deformablePose:__INSTANCE_NAME__:omniphysics:purposes", instanceName);
    addToStageSpecificAttributeMapExt(attachedStage, pointsAttrName, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr);
    addToStageSpecificAttributeMapExt(attachedStage, purposesAttrName, emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr);
}

// setup persistent change listeners
void registerChangeParams(std::vector<usdparser::ChangeParams>& changeParams)
{
    // scene live changes
    REGISTER_CHANGE(changeParams, "physics:gravityMagnitude", emptyRequirementCheck, updateGravityMagnitude)
    REGISTER_CHANGE(changeParams, "physics:gravityDirection", emptyRequirementCheck, updateGravityDirection)
    REGISTER_CHANGE(changeParams, "physxScene:timeStepsPerSecond", emptyRequirementCheck, updateTimeStepsPerSecond)
    // Newton scene fallbacks (apply only when the corresponding PhysX attribute is not authored).
    REGISTER_CHANGE(changeParams, "newton:timeStepsPerSecond", emptyRequirementCheck, updateNewtonTimeStepsPerSecond)
    REGISTER_CHANGE(changeParams, "newton:gravityEnabled", emptyRequirementCheck, updateNewtonGravityEnabled)
    REGISTER_CHANGE(changeParams, "physxScene:updateType", emptyRequirementCheck, updateSceneUpdateType)
    REGISTER_CHANGE(changeParams, "physxSceneQuasistatic:enableQuasistatic", emptyRequirementCheck, updateQuasistaticEnabled)
    REGISTER_CHANGE(changeParams, "collection:quasistaticactors:includes", emptyRequirementCheck, updateQuasistaticCollection)
    REGISTER_CHANGE(changeParams, "collection:quasistaticactors:excludes", emptyRequirementCheck, updateQuasistaticCollection)

    // body live changes
    REGISTER_CHANGE(changeParams, "physics:rigidBodyEnabled", emptyRequirementCheck, updateBodyEnabled)
    REGISTER_CHANGE(changeParams, "physics:mass", bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, "physics:density", bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, "physics:centerOfMass", bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, "physics:diagonalInertia", bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, "physics:principalAxes", bodyRequirementCheck, updateBodyDensity)
    REGISTER_CHANGE(changeParams, "physics:velocity", emptyRequirementCheck, updateBodyLinearVelocity)
    REGISTER_CHANGE(
        changeParams, "physics:angularVelocity", emptyRequirementCheck, updateBodyAngularVelocity)
    REGISTER_CHANGE(
        changeParams, "physics:kinematicEnabled", emptyRequirementCheck, updateBodyEnableKinematics)
    REGISTER_CHANGE(changeParams, "xformOpOrder", emptyRequirementCheck, updateBodyTransformStack)

    REGISTER_CHANGE(
        changeParams, "physxRigidBody:linearDamping", emptyRequirementCheck, updateBodyLinearDamping)
    REGISTER_CHANGE(
        changeParams, "physxRigidBody:angularDamping", emptyRequirementCheck, updateBodyAngularDamping)
    REGISTER_CHANGE(changeParams, "physxRigidBody:maxLinearVelocity", emptyRequirementCheck,
                    updateBodyMaxLinearVelocity)
    REGISTER_CHANGE(changeParams, "physxRigidBody:maxAngularVelocity", emptyRequirementCheck,
                    updateBodyMaxAngularVelocity)
    REGISTER_CHANGE(changeParams, "physxRigidBody:maxContactImpulse", emptyRequirementCheck,
                    updateBodyMaxContactImpulse)
    REGISTER_CHANGE(
        changeParams, "physxRigidBody:sleepThreshold", emptyRequirementCheck, updateBodySleepThreshold)
    REGISTER_CHANGE(changeParams, "physxRigidBody:stabilizationThreshold", emptyRequirementCheck,
                    updateBodyStabilizationThreshold)
    REGISTER_CHANGE(changeParams, "physxRigidBody:maxDepenetrationVelocity", emptyRequirementCheck,
                    updateBodyMaxDepenetrationVelocity)
    REGISTER_CHANGE(changeParams, "physxRigidBody:contactSlopCoefficient", emptyRequirementCheck,
                    updateBodyContactSlopCoefficient)
    REGISTER_CHANGE(changeParams, "physxRigidBody:solverPositionIterationCount", emptyRequirementCheck,
                    updateBodySolverPositionIterationCount)
    REGISTER_CHANGE(changeParams, "physxRigidBody:solverVelocityIterationCount", emptyRequirementCheck,
                    updateBodySolverVelocityIterationCount)
    REGISTER_CHANGE(changeParams, "physxRigidBody:enableCCD", emptyRequirementCheck, updateBodyEnableCCD)
    REGISTER_CHANGE(changeParams, "physxRigidBody:enableSpeculativeCCD", emptyRequirementCheck,
                    updateBodyEnableSpeculativeCCD)
    REGISTER_CHANGE(changeParams, "physxRigidBody:retainAccelerations", emptyRequirementCheck,
                    updateBodyRetainAccelerations)
    REGISTER_CHANGE(changeParams, "physxRigidBody:enableGyroscopicForces", emptyRequirementCheck,
                    updateBodyGyroscopicForces)
    REGISTER_CHANGE(changeParams, "physxRigidBody:disableGravity", emptyRequirementCheck,
                    updateBodyDisableGravity)
    REGISTER_CHANGE(
        changeParams, "physxRigidBody:lockedPosAxis", emptyRequirementCheck, updateBodyLockedPosAxis)
    REGISTER_CHANGE(
        changeParams, "physxRigidBody:lockedRotAxis", emptyRequirementCheck, updateBodyLockedRotAxis)
    REGISTER_CHANGE(changeParams, "physxRigidBody:cfmScale", emptyRequirementCheck, updateBodyCfmScale)
    REGISTER_CHANGE(changeParams, "physxRigidBody:solveContact", emptyRequirementCheck, updateBodySolveContacts)

    // contact report
    REGISTER_CHANGE(changeParams, "physxContactReport:threshold", emptyRequirementCheck, updatePhysxContactReportThreshold)

    // force
    REGISTER_CHANGE(changeParams, "physxForce:forceEnabled", emptyRequirementCheck, updatePhysxForceEnabled)
    REGISTER_CHANGE(changeParams, "physxForce:mode", emptyRequirementCheck, updatePhysxForceMode)
    REGISTER_CHANGE(changeParams, "physxForce:torque", emptyRequirementCheck, updatePhysxTorque)
    REGISTER_CHANGE(changeParams, "physxForce:worldFrameEnabled", emptyRequirementCheck, updatePhysxForceWorldFrameEnabled)
    REGISTER_CHANGE(changeParams, "physxForce:force", emptyRequirementCheck, updatePhysxForce)

    // surface velocity
    REGISTER_CHANGE(changeParams, "physxSurfaceVelocity:surfaceVelocityEnabled", emptyRequirementCheck, updateBodySurfaceVelocityEnabled)
    REGISTER_CHANGE(changeParams, "physxSurfaceVelocity:surfaceVelocity", emptyRequirementCheck, updateBodySurfaceLinearVelocity)
    REGISTER_CHANGE(changeParams, "physxSurfaceVelocity:surfaceAngularVelocity", emptyRequirementCheck, updateBodySurfaceAngularVelocity)
    REGISTER_CHANGE(changeParams, "physxSplinesSurfaceVelocity:surfaceVelocityMagnitude", emptyRequirementCheck, updateBodySplineSurfaceVelocityMagnitude)
    REGISTER_CHANGE(changeParams, "physxSplinesSurfaceVelocity:surfaceVelocityEnabled", emptyRequirementCheck, updateBodySplineSurfaceVelocityEnabled)
    REGISTER_CHANGE(changeParams, "physxSurfaceVelocity:surfaceVelocityLocalSpace", emptyRequirementCheck, updateBodySurfaceVelocityLocalSpace)

    // body point instancer changes
    REGISTER_CHANGE(changeParams, "positions", bodyPointInstancerRequirementCheck, updateBodyInstancedPositions)
    REGISTER_CHANGE(changeParams, "orientations", bodyPointInstancerRequirementCheck, updateBodyInstancedOrientations)
    REGISTER_CHANGE(changeParams, "velocities", bodyPointInstancerRequirementCheck, updateBodyInstancedVelocities)
    REGISTER_CHANGE(changeParams, "angularVelocities", bodyPointInstancerRequirementCheck, updateBodyInstancedAngularVelocities)
    REGISTER_CHANGE(changeParams, "protoIndices", bodyPointInstancerRequirementCheck, nullptr)
    // `ids` names the instances that `inactiveIds` refers to, so the parsed activation state depends
    // on it. No onUpdate: like protoIndices this is structural and needs a re-parse, which is what
    // re-reads the id -> position mapping (see IPhysicsSource::getInactiveInstanceIds).
    REGISTER_CHANGE(changeParams, "ids", bodyPointInstancerRequirementCheck, nullptr)

    // collision live changes
    REGISTER_CHANGE(changeParams, "physics:mass", collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(changeParams, "physics:density", collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(changeParams, "physics:centerOfMass", collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(changeParams, "physics:diagonalInertia", collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(changeParams, "physics:principalAxes", collisionRequirementCheck, updateShapeDensity)
    REGISTER_CHANGE(
        changeParams, "physics:collisionEnabled", collisionRequirementCheck, updateShapeEnabled)

    REGISTER_CHANGE(
        changeParams, "physxCollision:contactOffset", physicsRigidCollisionCheck, updateShapeContactOffset)
    REGISTER_CHANGE(
        changeParams, "physxCollision:restOffset", physicsRigidCollisionCheck, updateShapeRestOffset)
    // Newton collision fallbacks (rigid).
    REGISTER_CHANGE(
        changeParams, "newton:contactMargin", physicsRigidCollisionCheck, updateNewtonShapeContactMargin)
    REGISTER_CHANGE(
        changeParams, "newton:contactGap", physicsRigidCollisionCheck, updateNewtonShapeContactGap)
    REGISTER_CHANGE(changeParams, "physxCollision:torsionalPatchRadius", emptyRequirementCheck,
                    updateShapeTorsionalPatchRadius)
    REGISTER_CHANGE(changeParams, "physxCollision:minTorsionalPatchRadius", emptyRequirementCheck,
                    updateShapeMinTorsionalPatchRadius)

    REGISTER_CHANGE(changeParams, "physics:approximation", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physxConvexHullCollision:hullVertexLimit", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physxConvexHullCollision:minThickness", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physxConvexDecompositionCollision:hullVertexLimit", emptyRequirementCheck, nullptr)
    // Newton fallback: newton:maxHullVertices feeds convex hull and decomposition vertex limits.
    // Mirrors PhysX (release+reparse on change).
    REGISTER_CHANGE(changeParams, "newton:maxHullVertices", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physxConvexDecompositionCollision:errorPercentage", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physxConvexDecompositionCollision:maxConvexHulls", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physxConvexDecompositionCollision:minThickness", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physxConvexDecompositionCollision:voxelResolution", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physxConvexDecompositionCollision:shrinkWrap", emptyRequirementCheck, nullptr)

    REGISTER_CHANGE(changeParams, "physxTriangleMeshSimplificationCollision:metric", emptyRequirementCheck, nullptr)

    // simulation owner changes, fast path for rigid bodies, reparse otherwise
    REGISTER_CHANGE(changeParams, "physics:simulationOwner", bodyRequirementCheck, updateBodySimulationOwner)
    REGISTER_CHANGE(changeParams, "physics:simulationOwner", noBodyRequirementCheck, nullptr)

    // filtered pairs rel
    REGISTER_CHANGE(changeParams, "physics:filteredPairs", emptyRequirementCheck, updateFilteredPairs)

    // material live changes
    REGISTER_CHANGE(changeParams, "physics:density", materialRequirementCheck, updateMaterialDensity)
    REGISTER_CHANGE(changeParams, "physics:dynamicFriction", materialRequirementCheck,
                    updateMaterialDynamicFriction)
    REGISTER_CHANGE(
        changeParams, "physics:staticFriction", materialRequirementCheck, updateMaterialStaticFriction)
    REGISTER_CHANGE(
        changeParams, "physics:restitution", materialRequirementCheck, updateMaterialRestitution)

    REGISTER_CHANGE(changeParams, "physxMaterial:frictionCombineMode", physxMaterialRequirementCheck,
                    updateMaterialFrictionCombineMode)
    REGISTER_CHANGE(changeParams, "physxMaterial:restitutionCombineMode", physxMaterialRequirementCheck,
                    updateMaterialRestitutionCombineMode)
    REGISTER_CHANGE(changeParams, "physxMaterial:dampingCombineMode", physxMaterialRequirementCheck,
                    updateMaterialDampingCombineMode)
    REGISTER_CHANGE(changeParams, "physxMaterial:compliantContactAccelerationSpring",
                    physxMaterialRequirementCheck, updateCompliantMaterial)
    REGISTER_CHANGE(changeParams, "physxMaterial:compliantContactStiffness",
                    physxMaterialRequirementCheck, updateCompliantMaterial)
    REGISTER_CHANGE(changeParams, "physxMaterial:compliantContactDamping",
                    physxMaterialRequirementCheck, updateCompliantMaterial)


    // collision groups live changes
    REGISTER_CHANGE(changeParams, "collection:colliders:includes", collisionGroupRequirementCheck, updateCollisionGroup)
    REGISTER_CHANGE(changeParams, "physics:filteredGroups", collisionGroupRequirementCheck, updateCollisionGroup)


    // joint enable/disable
    REGISTER_CHANGE(changeParams, "physics:jointEnabled", jointEnableDisableRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physics:body0", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "physics:body1", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(
            changeParams, "physics:collisionEnabled", jointRequirementCheck, updateEnableCollision)
    REGISTER_CHANGE(
        changeParams, "physics:breakForce", emptyRequirementCheck, updateBreakForce)
    REGISTER_CHANGE(
        changeParams, "physics:breakTorque", emptyRequirementCheck, updateBreakTorque)
    REGISTER_CHANGE(
        changeParams, "physics:localPos0", emptyRequirementCheck, updateLocalPos0)
    REGISTER_CHANGE(
        changeParams, "physics:localRot0", emptyRequirementCheck, updateLocalRot0)
    REGISTER_CHANGE(
        changeParams, "physics:localPos1", emptyRequirementCheck, updateLocalPos1)
    REGISTER_CHANGE(
        changeParams, "physics:localRot1", emptyRequirementCheck, updateLocalRot1)
    REGISTER_CHANGE(
        changeParams, "physxJoint:armature", emptyRequirementCheck, updateArmature)

    // revolute joint
    REGISTER_CHANGE(changeParams, "physics:lowerLimit", emptyRequirementCheck, updateLimitLow)
    REGISTER_CHANGE(changeParams, "physics:upperLimit", emptyRequirementCheck, updateLimitHigh)
    REGISTER_CHANGE(changeParams, "drive:angular:physics:targetPosition", emptyRequirementCheck, updateDriveTargetPosition)
    REGISTER_CHANGE(changeParams, "drive:angular:physics:targetVelocity", emptyRequirementCheck, updateDriveTargetVelocity)
    REGISTER_CHANGE(changeParams, "drive:angular:physics:maxForce", emptyRequirementCheck, updateDriveMaxForce)
    REGISTER_CHANGE(changeParams, "physxDrivePerformanceEnvelope:angular:maxActuatorVelocity", emptyRequirementCheck, updateDriveMaxActuatorVelocity)
    REGISTER_CHANGE(changeParams, "physxDrivePerformanceEnvelope:angular:velocityDependentResistance", emptyRequirementCheck, updateDriveVelocityDependentResistance)
    REGISTER_CHANGE(changeParams, "physxDrivePerformanceEnvelope:angular:speedEffortGradient", emptyRequirementCheck, updateDriveSpeedEffortGradient)
    
    REGISTER_CHANGE(changeParams, "physxJointAxis:angular:armature", emptyRequirementCheck, updateArmaturePerAxis)
    REGISTER_CHANGE(changeParams, "physxJointAxis:angular:maxJointVelocity", emptyRequirementCheck, updateArticulationMaxJointVelocityPerAxis)
    REGISTER_CHANGE(changeParams, "physxJointAxis:angular:staticFrictionEffort", emptyRequirementCheck, updateArticulationStaticFrictionEffort)
    REGISTER_CHANGE(changeParams, "physxJointAxis:angular:dynamicFrictionEffort", emptyRequirementCheck, updateArticulationDynamicFrictionEffort)
    REGISTER_CHANGE(changeParams, "physxJointAxis:angular:viscousFrictionCoefficient", emptyRequirementCheck, updateArticulationViscousFrictionCoefficient)
    
    

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
    REGISTER_CHANGE(changeParams, "physics:lowerLimit", emptyRequirementCheck, updateLimitLow)
    REGISTER_CHANGE(changeParams, "physics:upperLimit", emptyRequirementCheck, updateLimitHigh)
    REGISTER_CHANGE(changeParams, "drive:linear:physics:targetPosition", emptyRequirementCheck, updateDriveTargetPosition)
    REGISTER_CHANGE(changeParams, "drive:linear:physics:targetVelocity", emptyRequirementCheck, updateDriveTargetVelocity)
    REGISTER_CHANGE(changeParams, "drive:linear:physics:maxForce", emptyRequirementCheck, updateDriveMaxForce)
    REGISTER_CHANGE(changeParams, "physxDrivePerformanceEnvelope:linear:maxActuatorVelocity", emptyRequirementCheck, updateDriveMaxActuatorVelocity)
    REGISTER_CHANGE(changeParams, "physxDrivePerformanceEnvelope:linear:velocityDependentResistance", emptyRequirementCheck, updateDriveVelocityDependentResistance)
    REGISTER_CHANGE(changeParams, "physxDrivePerformanceEnvelope:linear:speedEffortGradient", emptyRequirementCheck, updateDriveSpeedEffortGradient)
    
    REGISTER_CHANGE(changeParams, "physxJointAxis:linear:armature", emptyRequirementCheck, updateArmaturePerAxis)
    REGISTER_CHANGE(changeParams, "physxJointAxis:linear:maxJointVelocity", emptyRequirementCheck, updateArticulationMaxJointVelocityPerAxis)
    REGISTER_CHANGE(changeParams, "physxJointAxis:linear:staticFrictionEffort", emptyRequirementCheck, updateArticulationStaticFrictionEffort)
    REGISTER_CHANGE(changeParams, "physxJointAxis:linear:dynamicFrictionEffort", emptyRequirementCheck, updateArticulationDynamicFrictionEffort)
    REGISTER_CHANGE(changeParams, "physxJointAxis:linear:viscousFrictionCoefficient", emptyRequirementCheck, updateArticulationViscousFrictionCoefficient)
    

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
        changeParams, "physics:coneAngle0Limit", emptyRequirementCheck, updateLimitLow)
    REGISTER_CHANGE(
        changeParams, "physics:coneAngle1Limit", emptyRequirementCheck, updateLimitHigh)

    // distance joint
    REGISTER_CHANGE(changeParams, "physics:minDistance", emptyRequirementCheck, updateLimitLow)
    REGISTER_CHANGE(changeParams, "physics:maxDistance", emptyRequirementCheck, updateLimitHigh)

    // gear joint
    REGISTER_CHANGE(changeParams, "physics:gearRatio", emptyRequirementCheck, updateGearRatio)
    REGISTER_CHANGE(changeParams, "physics:hinge0", emptyRequirementCheck, updateGearHinge0)
    REGISTER_CHANGE(changeParams, "physics:hinge1", emptyRequirementCheck, updateGearHinge1)

    // rack and pinion joint
    REGISTER_CHANGE(changeParams, "physics:ratio", emptyRequirementCheck, updateRackPinionRatio)
    REGISTER_CHANGE(changeParams, "physics:hinge", emptyRequirementCheck, updateRackHinge)
    REGISTER_CHANGE(changeParams, "physics:prismatic", emptyRequirementCheck, updateRackPrismatic)


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
    REGISTER_CHANGE(changeParams, "physxPhysicsDistanceJoint:springDamping", emptyRequirementCheck, updateDistanceJointSpringDamping)
    REGISTER_CHANGE(changeParams, "physxPhysicsDistanceJoint:springStiffness", emptyRequirementCheck, updateDistanceJointSpringStiffness)
    REGISTER_CHANGE(changeParams, "physxPhysicsDistanceJoint:springEnabled", emptyRequirementCheck, updateDistanceJointSpringEnabled)
    

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

    REGISTER_CHANGE(changeParams, "physxJoint:maxJointVelocity", emptyRequirementCheck,
        updateArticulationMaxJointVelocity)
    REGISTER_CHANGE(changeParams, "newton:velocityLimit", emptyRequirementCheck,
        updateNewtonJointVelocityLimit)
    REGISTER_CHANGE(changeParams, "physxJoint:jointFriction", emptyRequirementCheck,
        updateArticulationFrictionCoefficient)

    // articulation    
    REGISTER_CHANGE(changeParams, "physxArticulation:solverPositionIterationCount",
                    emptyRequirementCheck, updateArticulationSolverPositionIterationCount)
    REGISTER_CHANGE(changeParams, "physxArticulation:solverVelocityIterationCount",
                    emptyRequirementCheck, updateArticulationSolverVelocityIterationCount)
    REGISTER_CHANGE(changeParams, "physxArticulation:sleepThreshold", emptyRequirementCheck,
                    updateArticulationSleepThreshold)
    REGISTER_CHANGE(changeParams, "physxArticulation:stabilizationThreshold", emptyRequirementCheck,
                    updateArticulationStabilizationThreshold)

    // PBD material
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:cohesion", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:adhesion", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:particleAdhesionScale", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:adhesionOffsetScale", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:friction", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:particleFrictionScale", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:damping", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:surfaceTension", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:viscosity", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:vorticityConfinement", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:gravityScale", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:cflCoefficient", emptyRequirementCheck, updatePBDMaterialAttribute)
    REGISTER_CHANGE(changeParams, "physxPBDMaterial:density", emptyRequirementCheck, updatePBDMaterialAttribute)

    // particle system
    REGISTER_CHANGE(changeParams, "particleSystemEnabled", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "enableCCD", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "contactOffset", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "fluidRestOffset", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "maxDepenetrationVelocity", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "maxVelocity", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "particleContactOffset", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "restOffset", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "solidRestOffset", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "solverPositionIterationCount", emptyRequirementCheck, updateParticleSystemAttribute)

    REGISTER_CHANGE(changeParams, "wind", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "maxNeighborhood", emptyRequirementCheck, updateParticleSystemAttribute)
    REGISTER_CHANGE(changeParams, "neighborhoodScale", emptyRequirementCheck, updateParticleSystemAttribute)

    // particle sets
    REGISTER_CHANGE(changeParams, "physxParticle:particleEnabled", physxParticleSetRequirementCheck, updateParticleSetEnabled)
    REGISTER_CHANGE(changeParams, "physxParticle:selfCollision", physxParticleSetRequirementCheck, updateParticleSetSelfCollision)
    REGISTER_CHANGE(changeParams, "physxParticle:fluid", emptyRequirementCheck, updateParticleSetFluid)
    REGISTER_CHANGE(changeParams, "physxParticle:particleGroup", physxParticleSetRequirementCheck, updateParticleSetParticleGroup)
    REGISTER_CHANGE(changeParams, "physxParticle:simulationPoints", emptyRequirementCheck, updateParticleSimPositions)
    REGISTER_CHANGE(changeParams, "points", physxParticleSetPositionRequirementCheck, updateParticlePositions)
    REGISTER_CHANGE(changeParams, "positions", physxParticleSetPositionRequirementCheck, updateParticlePositions)
    REGISTER_CHANGE(changeParams, "velocities", physxParticleSetRequirementCheck, updateParticleVelocities)

    REGISTER_CHANGE(changeParams, "physics:mass", physxParticleSetRequirementCheck, updateParticleDensity)
    REGISTER_CHANGE(changeParams, "physics:density", physxParticleSetRequirementCheck, updateParticleDensity)

    // diffuse particles
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:diffuseParticlesEnabled", emptyRequirementCheck, updateDiffuseParticlesEnabledAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:maxDiffuseParticleMultiplier", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:threshold", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:lifetime", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:airDrag", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:bubbleDrag", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:buoyancy", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:kineticEnergyWeight", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:pressureWeight", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:divergenceWeight", emptyRequirementCheck, updateDiffuseParticlesAttribute)
    REGISTER_CHANGE(changeParams, "physxDiffuseParticles:collisionDecay", emptyRequirementCheck, updateDiffuseParticlesAttribute)

    // anisotropy
    REGISTER_CHANGE(changeParams, "physxParticleAnisotropy:particleAnisotropyEnabled", emptyRequirementCheck, updateParticleAnisotropyEnabledAttribute)

    // smoothing
    REGISTER_CHANGE(changeParams, "physxParticleSmoothing:particleSmoothingEnabled", emptyRequirementCheck, updateParticleSmoothingEnabledAttribute)

    // isosurface
    REGISTER_CHANGE(changeParams, "physxParticleIsosurface:isosurfaceEnabled", emptyRequirementCheck, updateParticleIsosurfaceEnabledAttribute)
    REGISTER_CHANGE(changeParams, "physxParticleIsosurface:surfaceDistance", emptyRequirementCheck, updateParticleIsosurfaceAttribute)
    REGISTER_CHANGE(changeParams, "physxParticleIsosurface:gridFilteringPasses", emptyRequirementCheck, updateParticleIsosurfaceAttribute)
    REGISTER_CHANGE(changeParams, "physxParticleIsosurface:gridSmoothingRadius", emptyRequirementCheck, updateParticleIsosurfaceAttribute)
    REGISTER_CHANGE(changeParams, "physxParticleIsosurface:numMeshSmoothingPasses", emptyRequirementCheck, updateParticleIsosurfaceAttribute)
    REGISTER_CHANGE(changeParams, "physxParticleIsosurface:numMeshNormalSmoothingPasses", emptyRequirementCheck, updateParticleIsosurfaceAttribute)

    // deformables
    REGISTER_CHANGE_EXT(changeParams, "omniphysics:deformableBodyEnabled", emptyRequirementCheck, physicsDeformableBodyResyncCheck, nullptr)
    REGISTER_CHANGE(changeParams, "omniphysics:attachmentEnabled", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE(changeParams, "omniphysics:filterEnabled", emptyRequirementCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "faceVertexCounts", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "faceVertexIndices", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "tetVertexIndices", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE(changeParams, "points", physicsDeformableSimRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "velocities", physicsDeformableSimRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE_EXT(changeParams, "omniphysics:restTetVtxIndices", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "omniphysics:restTriVtxIndices", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "physxDeformableBody:autoDeformableBodyEnabled", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "physxDeformableBody:resolution", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "physxDeformableBody:autoDeformableMeshSimplificationEnabled", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "physxDeformableBody:remeshingEnabled", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "physxDeformableBody:remeshingResolution", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "physxDeformableBody:targetTriangleCount", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE_EXT(changeParams, "physxDeformableBody:forceConforming", emptyRequirementCheck, physicsDeformableBodyHierarchyResyncCheck, nullptr)
    REGISTER_CHANGE(changeParams, "omniphysics:mass", physicsDeformableBodyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:solverPositionIterationCount", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:linearDamping", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:maxLinearVelocity", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:settlingDamping", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:sleepThreshold", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:settlingThreshold", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:maxDepenetrationVelocity", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:selfCollision", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:selfCollisionFilterDistance", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:enableSpeculativeCCD", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:disableGravity", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxCollision:contactOffset", physicsDeformableCollisionCheck, updateDeformableContactOffset)
    REGISTER_CHANGE(changeParams, "physxCollision:restOffset", physicsDeformableCollisionCheck, updateDeformableRestOffset)
    // Newton collision fallbacks (deformable).
    REGISTER_CHANGE(changeParams, "newton:contactMargin", physicsDeformableCollisionCheck, updateNewtonDeformableContactMargin)
    REGISTER_CHANGE(changeParams, "newton:contactGap", physicsDeformableCollisionCheck, updateNewtonDeformableContactGap)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:collisionPairUpdateFrequency", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "physxDeformableBody:collisionIterationMultiplier", emptyRequirementCheck, updateDeformableBody)
    REGISTER_CHANGE(changeParams, "omniphysics:density", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "omniphysics:dynamicFriction", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "omniphysics:staticFriction", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "omniphysics:youngsModulus", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "omniphysics:poissonsRatio", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "omniphysics:surfaceThickness", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "omniphysics:surfaceStretchStiffness", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "omniphysics:surfaceShearStiffness", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "omniphysics:surfaceBendStiffness", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "physxDeformableMaterial:elasticityDamping", emptyRequirementCheck, updateDeformableMaterial)
    REGISTER_CHANGE(changeParams, "physxDeformableMaterial:bendDamping", emptyRequirementCheck, updateDeformableMaterial)


    // CCT
    REGISTER_CHANGE(changeParams, "physxCharacterController:slopeLimit", emptyRequirementCheck, updateCctSlopeLimit)
    REGISTER_CHANGE(changeParams, "height", cctRequirementCheck, updateCctHeight)
    REGISTER_CHANGE(changeParams, "radius", cctRequirementCheck, updateCctRadius)
    REGISTER_CHANGE(changeParams, "physxCharacterController:contactOffset", emptyRequirementCheck, updateCctContactOffset)
    REGISTER_CHANGE(changeParams, "physxCharacterController:stepOffset", emptyRequirementCheck, updateCctStepOffset)
    REGISTER_CHANGE(changeParams, "physxCharacterController:upAxis", emptyRequirementCheck, updateCctUpAxis)
    REGISTER_CHANGE(changeParams, "physxCharacterController:nonWalkableMode", emptyRequirementCheck, updateCctNonWalkableMode)
    REGISTER_CHANGE(changeParams, "physxCharacterController:climbingMode", emptyRequirementCheck, updateCctClimbingMode)

    // vehicle
    REGISTER_CHANGE(changeParams, "physxVehicleContext:updateMode", emptyRequirementCheck, updateVehicleContextUpdateMode)
    REGISTER_CHANGE(changeParams, "physxVehicleContext:verticalAxis", emptyRequirementCheck, updateVehicleContextVerticalAxis)
    REGISTER_CHANGE(changeParams, "physxVehicleContext:longitudinalAxis", emptyRequirementCheck, updateVehicleContextLongitudinalAxis)

    REGISTER_CHANGE(changeParams, "physxVehicleEngine:moi", emptyRequirementCheck, updateVehicleEngineMomentOfInertia)
    REGISTER_CHANGE(changeParams, "physxVehicleEngine:peakTorque", emptyRequirementCheck, updateVehicleEnginePeakTorque)
    REGISTER_CHANGE(changeParams, "physxVehicleEngine:maxRotationSpeed", emptyRequirementCheck, updateVehicleEngineMaxRotationSpeed)
    REGISTER_CHANGE(changeParams, "physxVehicleEngine:idleRotationSpeed", emptyRequirementCheck, updateVehicleEngineIdleRotationSpeed)
    REGISTER_CHANGE(changeParams, "physxVehicleEngine:torqueCurve", emptyRequirementCheck, updateVehicleEngineTorqueCurve)
    REGISTER_CHANGE(changeParams, "physxVehicleEngine:dampingRateFullThrottle", emptyRequirementCheck, updateVehicleEngineDampingRateFullThrottle)
    REGISTER_CHANGE(changeParams, "physxVehicleEngine:dampingRateZeroThrottleClutchEngaged", emptyRequirementCheck, updateVehicleEngineDampingRateZeroThrottleClutchEngaged)
    REGISTER_CHANGE(changeParams, "physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged", emptyRequirementCheck, updateVehicleEngineDampingRateZeroThrottleClutchDisengaged)

    REGISTER_CHANGE(changeParams, "frictionValues", emptyRequirementCheck, updateVehicleTireFrictionTableFrictionValues)
    REGISTER_CHANGE(changeParams, "groundMaterials", emptyRequirementCheck, updateVehicleTireFrictionTableGroundMaterials)
    REGISTER_CHANGE(changeParams, "defaultFrictionValue", emptyRequirementCheck, updateVehicleTireFrictionTableDefaultFrictionValue)

    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:springStrength", emptyRequirementCheck, updateVehicleSuspensionSpringStrength)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:springDamperRate", emptyRequirementCheck, updateVehicleSuspensionSpringDamperRate)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:maxCompression", emptyRequirementCheck, updateVehicleSuspensionMaxCompression)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:maxDroop", emptyRequirementCheck, updateVehicleSuspensionMaxDroop)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:travelDistance", emptyRequirementCheck, updateVehicleSuspensionTravelDistance)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:sprungMass", emptyRequirementCheck, updateVehicleSuspensionSprungMass)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:camberAtRest", emptyRequirementCheck, updateVehicleSuspensionCamberAtRest)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:camberAtMaxCompression", emptyRequirementCheck, updateVehicleSuspensionCamberAtMaxCompression)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspension:camberAtMaxDroop", emptyRequirementCheck, updateVehicleSuspensionCamberAtMaxDroop)

    REGISTER_CHANGE(changeParams, "physxVehicleTire:latStiffX", emptyRequirementCheck, updateVehicleTireLatStiffX)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:latStiffY", emptyRequirementCheck, updateVehicleTireLatStiffY)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:lateralStiffnessGraph", emptyRequirementCheck, updateVehicleTireLateralStiffnessGraph)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:longitudinalStiffnessPerUnitGravity", emptyRequirementCheck, updateVehicleTireLongStiffPerGrav)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:longitudinalStiffness", emptyRequirementCheck, updateVehicleTireLongitudinalStiffness)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:camberStiffnessPerUnitGravity", emptyRequirementCheck, updateVehicleTireCamberStiffPerGrav)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:camberStiffness", emptyRequirementCheck, updateVehicleTireCamberStiffness)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:frictionVsSlipGraph", emptyRequirementCheck, updateVehicleTireFrictionVsSlip)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:frictionTable", emptyRequirementCheck, updateVehicleTireFrictionTableRel)
    REGISTER_CHANGE(changeParams, "physxVehicleTire:restLoad", emptyRequirementCheck, updateVehicleTireRestLoad)

    REGISTER_CHANGE(changeParams, "physxVehicleWheel:radius", emptyRequirementCheck, updateVehicleWheelRadius)
    REGISTER_CHANGE(changeParams, "physxVehicleWheel:width", emptyRequirementCheck, updateVehicleWheelWidth)
    REGISTER_CHANGE(changeParams, "physxVehicleWheel:mass", emptyRequirementCheck, updateVehicleWheelMass)
    REGISTER_CHANGE(changeParams, "physxVehicleWheel:moi", emptyRequirementCheck, updateVehicleWheelMomentOfInertia)
    REGISTER_CHANGE(changeParams, "physxVehicleWheel:dampingRate", emptyRequirementCheck, updateVehicleWheelDampingRate)
    REGISTER_CHANGE(changeParams, "physxVehicleWheel:maxBrakeTorque", emptyRequirementCheck, updateVehicleWheelMaxBrakeTorque)
    REGISTER_CHANGE(changeParams, "physxVehicleWheel:maxHandBrakeTorque", emptyRequirementCheck, updateVehicleWheelMaxHandBrakeTorque)
    REGISTER_CHANGE(changeParams, "physxVehicleWheel:maxSteerAngle", emptyRequirementCheck, updateVehicleWheelMaxSteerAngle)
    REGISTER_CHANGE(changeParams, "physxVehicleWheel:toeAngle", emptyRequirementCheck, updateVehicleWheelToeAngle)

    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:index", emptyRequirementCheck, updateVehicleWheelAttachmentIndex)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:wheel", emptyRequirementCheck, updateVehicleWheelAttachmentWheel)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:tire", emptyRequirementCheck, updateVehicleWheelAttachmentTire)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:suspension", emptyRequirementCheck, updateVehicleWheelAttachmentSuspension)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:suspensionTravelDirection", emptyRequirementCheck, updateVehicleWheelAttachmentSuspensionTravelDirection)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:suspensionForceAppPointOffset", emptyRequirementCheck, updateVehicleWheelAttachmentSuspensionForceAppPointOffset)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:wheelCenterOfMassOffset", emptyRequirementCheck, updateVehicleWheelAttachmentWheelCenterOfMassOffset)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:tireForceAppPointOffset", emptyRequirementCheck, updateVehicleWheelAttachmentTireForceAppPointOffset)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:suspensionFramePosition", emptyRequirementCheck, updateVehicleWheelAttachmentSuspensionFramePosition)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:suspensionFrameOrientation", emptyRequirementCheck, updateVehicleWheelAttachmentSuspensionFrameOrientation)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:wheelFramePosition", emptyRequirementCheck, updateVehicleWheelAttachmentWheelFramePosition)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:wheelFrameOrientation", emptyRequirementCheck, updateVehicleWheelAttachmentWheelFrameOrientation)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:driven", emptyRequirementCheck, updateVehicleWheelAttachmentDriven)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelAttachment:collisionGroup", emptyRequirementCheck, updateVehicleWheelAttachmentCollisionGroup)

    REGISTER_CHANGE(changeParams, "physxVehicleSuspensionCompliance:wheelToeAngle", emptyRequirementCheck, updateVehicleSuspensionComplWheelToeAngle)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspensionCompliance:wheelCamberAngle", emptyRequirementCheck, updateVehicleSuspensionComplWheelCamberAngle)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspensionCompliance:suspensionForceAppPoint", emptyRequirementCheck, updateVehicleSuspensionComplSuspForceAppPoint)
    REGISTER_CHANGE(changeParams, "physxVehicleSuspensionCompliance:tireForceAppPoint", emptyRequirementCheck, updateVehicleSuspensionComplTireForceAppPoint)

    REGISTER_CHANGE(changeParams, "physxVehicle:vehicleEnabled", emptyRequirementCheck, updateVehicleEnabled)
    REGISTER_CHANGE(changeParams, "physxVehicle:limitSuspensionExpansionVelocity", emptyRequirementCheck, updateVehicleLimitSuspensionExpansionVelocity)
    REGISTER_CHANGE(changeParams, "physxVehicle:minPassiveLongitudinalSlipDenominator", emptyRequirementCheck, updateVehicleMinPassiveLongslipDenom)
    REGISTER_CHANGE(changeParams, "physxVehicle:minActiveLongitudinalSlipDenominator", emptyRequirementCheck, updateVehicleMinActiveLongslipDenom)
    REGISTER_CHANGE(changeParams, "physxVehicle:minLateralSlipDenominator", emptyRequirementCheck, updateVehicleMinLateralSlipDenom)
    REGISTER_CHANGE(changeParams, "physxVehicle:longitudinalStickyTireThresholdSpeed", emptyRequirementCheck, updateVehicleLongitudinalStickyTireThresholdSpeed)
    REGISTER_CHANGE(changeParams, "physxVehicle:longitudinalStickyTireThresholdTime", emptyRequirementCheck, updateVehicleLongitudinalStickyTireThresholdTime)
    REGISTER_CHANGE(changeParams, "physxVehicle:longitudinalStickyTireDamping", emptyRequirementCheck, updateVehicleLongitudinalStickyTireDamping)
    REGISTER_CHANGE(changeParams, "physxVehicle:lateralStickyTireThresholdSpeed", emptyRequirementCheck, updateVehicleLateralStickyTireThresholdSpeed)
    REGISTER_CHANGE(changeParams, "physxVehicle:lateralStickyTireThresholdTime", emptyRequirementCheck, updateVehicleLateralStickyTireThresholdTime)
    REGISTER_CHANGE(changeParams, "physxVehicle:lateralStickyTireDamping", emptyRequirementCheck, updateVehicleLateralStickyTireDamping)

    REGISTER_CHANGE(changeParams, "physxVehicleController:accelerator", emptyRequirementCheck, updateVehicleControllerAccelerator)
    REGISTER_CHANGE(changeParams, "physxVehicleController:brake0", emptyRequirementCheck, updateVehicleControllerBrake0)
    REGISTER_CHANGE(changeParams, "physxVehicleController:brake1", emptyRequirementCheck, updateVehicleControllerBrake1)
    REGISTER_CHANGE(changeParams, "physxVehicleController:brake", emptyRequirementCheck, updateVehicleControllerBrake)
    REGISTER_CHANGE(changeParams, "physxVehicleController:handbrake", emptyRequirementCheck, updateVehicleControllerHandbrake)
    REGISTER_CHANGE(changeParams, "physxVehicleController:steer", emptyRequirementCheck, updateVehicleControllerSteer)
    REGISTER_CHANGE(changeParams, "physxVehicleController:steerLeft", emptyRequirementCheck, updateVehicleControllerSteerLeft)
    REGISTER_CHANGE(changeParams, "physxVehicleController:steerRight", emptyRequirementCheck, updateVehicleControllerSteerRight)
    REGISTER_CHANGE(changeParams, "physxVehicleController:targetGear", emptyRequirementCheck, updateVehicleControllerTargetGear)

    REGISTER_CHANGE(changeParams, "physxVehicleTankController:thrust0", emptyRequirementCheck, updateVehicleTankControllerThrust0)
    REGISTER_CHANGE(changeParams, "physxVehicleTankController:thrust1", emptyRequirementCheck, updateVehicleTankControllerThrust1)

    REGISTER_CHANGE(changeParams, "physxVehicleDriveBasic:peakTorque", emptyRequirementCheck, updateVehicleDriveBasicPeakTorque)

    REGISTER_CHANGE(changeParams, "physxVehicleWheelController:driveTorque", emptyRequirementCheck, updateVehicleWheelControllerDriveTorque)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelController:brakeTorque", emptyRequirementCheck, updateVehicleWheelControllerBrakeTorque)
    REGISTER_CHANGE(changeParams, "physxVehicleWheelController:steerAngle", emptyRequirementCheck, updateVehicleWheelControllerSteerAngle)

    REGISTER_CHANGE(changeParams, "physxVehicleMultiWheelDifferential:wheels", emptyRequirementCheck, updateVehicleMultiWheelDifferentialWheels)
    REGISTER_CHANGE(changeParams, "physxVehicleMultiWheelDifferential:torqueRatios", emptyRequirementCheck, updateVehicleMultiWheelDifferentialTorqueRatios)
    REGISTER_CHANGE(changeParams, "physxVehicleMultiWheelDifferential:averageWheelSpeedRatios", emptyRequirementCheck, updateVehicleMultiWheelDifferentialAverageWheelSpeedRatios)

    REGISTER_CHANGE(changeParams, "physxVehicleTankDifferential:numberOfWheelsPerTrack", emptyRequirementCheck, updateVehicleTankDifferentialNumberOfWheelsPerTrack)
    REGISTER_CHANGE(changeParams, "physxVehicleTankDifferential:thrustIndexPerTrack", emptyRequirementCheck, updateVehicleTankDifferentialThrustIndexPerTrack)
    REGISTER_CHANGE(changeParams, "physxVehicleTankDifferential:trackToWheelIndices", emptyRequirementCheck, updateVehicleTankDifferentialTrackToWheelIndices)
    REGISTER_CHANGE(changeParams, "physxVehicleTankDifferential:wheelIndicesInTrackOrder", emptyRequirementCheck, updateVehicleTankDifferentialWheelIndicesInTrackOrder)

    const std::string vehicleBrakes0Wheels = makeMultiApplyAttributeName("physxVehicleBrakes:__INSTANCE_NAME__:wheels", "brakes0");
    const std::string vehicleBrakes1Wheels = makeMultiApplyAttributeName("physxVehicleBrakes:__INSTANCE_NAME__:wheels", "brakes1");
    const std::string vehicleBrakes0MaxBrakeTorque = makeMultiApplyAttributeName("physxVehicleBrakes:__INSTANCE_NAME__:maxBrakeTorque", "brakes0");
    const std::string vehicleBrakes1MaxBrakeTorque = makeMultiApplyAttributeName("physxVehicleBrakes:__INSTANCE_NAME__:maxBrakeTorque", "brakes1");
    const std::string vehicleBrakes0TorqueMultipliers = makeMultiApplyAttributeName("physxVehicleBrakes:__INSTANCE_NAME__:torqueMultipliers", "brakes0");
    const std::string vehicleBrakes1TorqueMultipliers = makeMultiApplyAttributeName("physxVehicleBrakes:__INSTANCE_NAME__:torqueMultipliers", "brakes1");

    REGISTER_CHANGE(changeParams, vehicleBrakes0Wheels, emptyRequirementCheck, updateVehicleBrakes0Wheels)
    REGISTER_CHANGE(changeParams, vehicleBrakes1Wheels, emptyRequirementCheck, updateVehicleBrakes1Wheels)
    REGISTER_CHANGE(changeParams, vehicleBrakes0MaxBrakeTorque, emptyRequirementCheck, updateVehicleBrakes0MaxBrakeTorque)
    REGISTER_CHANGE(changeParams, vehicleBrakes1MaxBrakeTorque, emptyRequirementCheck, updateVehicleBrakes1MaxBrakeTorque)
    REGISTER_CHANGE(changeParams, vehicleBrakes0TorqueMultipliers, emptyRequirementCheck, updateVehicleBrakes0TorqueMultipliers)
    REGISTER_CHANGE(changeParams, vehicleBrakes1TorqueMultipliers, emptyRequirementCheck, updateVehicleBrakes1TorqueMultipliers)

    REGISTER_CHANGE(changeParams, "physxVehicleSteering:wheels", emptyRequirementCheck, updateVehicleSteeringWheels)
    REGISTER_CHANGE(changeParams, "physxVehicleSteering:maxSteerAngle", emptyRequirementCheck, updateVehicleSteeringMaxSteerAngle)
    REGISTER_CHANGE(changeParams, "physxVehicleSteering:angleMultipliers", emptyRequirementCheck, updateVehicleSteeringAngleMultipliers)

    REGISTER_CHANGE(changeParams, "physxVehicleAckermannSteering:wheel0", emptyRequirementCheck, updateVehicleAckermannSteeringWheel0)
    REGISTER_CHANGE(changeParams, "physxVehicleAckermannSteering:wheel1", emptyRequirementCheck, updateVehicleAckermannSteeringWheel1)
    REGISTER_CHANGE(changeParams, "physxVehicleAckermannSteering:maxSteerAngle", emptyRequirementCheck, updateVehicleAckermannSteeringMaxSteerAngle)
    REGISTER_CHANGE(changeParams, "physxVehicleAckermannSteering:wheelBase", emptyRequirementCheck, updateVehicleAckermannSteeringWheelBase)
    REGISTER_CHANGE(changeParams, "physxVehicleAckermannSteering:trackWidth", emptyRequirementCheck, updateVehicleAckermannSteeringTrackWidth)
    REGISTER_CHANGE(changeParams, "physxVehicleAckermannSteering:strength", emptyRequirementCheck, updateVehicleAckermannSteeringStrength)

    static constexpr const char* kNCRCommandValuesTemplate = "physxVehicleNCR:__INSTANCE_NAME__:commandValues";
    static constexpr const char* kNCRSpeedResponsesPerCommandValueTemplate = "physxVehicleNCR:__INSTANCE_NAME__:speedResponsesPerCommandValue";
    static constexpr const char* kNCRSpeedResponsesTemplate = "physxVehicleNCR:__INSTANCE_NAME__:speedResponses";

    static const char* ncrInstanceTokens[] = { "drive", "steer", "brakes0", "brakes1" };
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
        const std::string instanceToken = ncrInstanceTokens[i];

        const std::string vehicleNCRCommandValues = makeMultiApplyAttributeName(kNCRCommandValuesTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, vehicleNCRCommandValues, emptyRequirementCheck, ncrCommandValuesUpdateMethods[i]);

        const std::string vehicleNCRSpeedResponsesPerCommandValue = makeMultiApplyAttributeName(kNCRSpeedResponsesPerCommandValueTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, vehicleNCRSpeedResponsesPerCommandValue, emptyRequirementCheck, ncrSpeedResponsesPerCommandValueUpdateMethods[i]);

        const std::string vehicleNCRSpeedResponses = makeMultiApplyAttributeName(kNCRSpeedResponsesTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, vehicleNCRSpeedResponses, emptyRequirementCheck, ncrSpeedResponsesUpdateMethods[i]);
    }

    // mimic joint

    // extra complexity because mimic joints are using a multiple apply API schema
    static constexpr const char* kMimicJointGearingTemplate = "physxMimicJoint:__INSTANCE_NAME__:gearing";
    static constexpr const char* kMimicJointOffsetTemplate = "physxMimicJoint:__INSTANCE_NAME__:offset";
    static constexpr const char* kMimicJointNaturalFrequencyTemplate = "physxMimicJoint:__INSTANCE_NAME__:naturalFrequency";
    static constexpr const char* kMimicJointDampingRatioTemplate = "physxMimicJoint:__INSTANCE_NAME__:dampingRatio";
    static constexpr const char* kMimicJointReferenceJointTemplate = "physxMimicJoint:__INSTANCE_NAME__:referenceJoint";
    static constexpr const char* kMimicJointReferenceJointAxisTemplate = "physxMimicJoint:__INSTANCE_NAME__:referenceJointAxis";

    static const char* mimicJointInstanceTokens[] = { "rotX", "rotY", "rotZ" };
    constexpr uint32_t mimicJointInstanceTokenCount = sizeof(mimicJointInstanceTokens) / sizeof(mimicJointInstanceTokens[0]);

    OnPrimRequirementKeyCheckFn mimicJointRequirementCheckMethods[mimicJointInstanceTokenCount] = {
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
        const std::string instanceToken = mimicJointInstanceTokens[i];

        const std::string mimicJointGearingName = makeMultiApplyAttributeName(kMimicJointGearingTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointGearingName, mimicJointRequirementCheckMethods[i], mimicJointGearingUpdateMethods[i]);

        const std::string mimicJointOffsetName = makeMultiApplyAttributeName(kMimicJointOffsetTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointOffsetName, mimicJointRequirementCheckMethods[i], mimicJointOffsetUpdateMethods[i]);

        const std::string mimicJointNaturalFrequencyName = makeMultiApplyAttributeName(kMimicJointNaturalFrequencyTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointNaturalFrequencyName, mimicJointRequirementCheckMethods[i], mimicJointNaturalFrequencyUpdateMethods[i]);

        const std::string mimicJointDampingRatioName = makeMultiApplyAttributeName(kMimicJointDampingRatioTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointDampingRatioName, mimicJointRequirementCheckMethods[i], mimicJointDampingRatioUpdateMethods[i]);

        // changing the reference joint relationship should trigger a release and re-parsing
        const std::string mimicJointReferenceJointName = makeMultiApplyAttributeName(kMimicJointReferenceJointTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointReferenceJointName, mimicJointRequirementCheckMethods[i], nullptr);

        // changing the reference joint axis should trigger a release and re-parsing
        const std::string mimicJointReferenceJointAxisName = makeMultiApplyAttributeName(kMimicJointReferenceJointAxisTemplate, instanceToken);
        REGISTER_CHANGE(changeParams, mimicJointReferenceJointAxisName, mimicJointRequirementCheckMethods[i], nullptr);
    }

    // Newton mimic API (single-apply)
    {
        OnPrimRequirementKeyCheckFn newtonMimicRequirementCheck =
            physxMimicJointRequirementCheck<SchemaAPIFlag::eNewtonMimicAPI>;

        REGISTER_CHANGE(changeParams, "newton:mimicCoef1", newtonMimicRequirementCheck, updateNewtonMimicJointCoef1);
        REGISTER_CHANGE(changeParams, "newton:mimicCoef0", newtonMimicRequirementCheck, updateNewtonMimicJointCoef0);

        // mimicEnabled and the mimicJoint relationship changes trigger a structural release+reparse.
        REGISTER_CHANGE(changeParams, "newton:mimicEnabled", newtonMimicRequirementCheck, nullptr);
        REGISTER_CHANGE(changeParams, "newton:mimicJoint", newtonMimicRequirementCheck, nullptr);
    }
}

#undef REGISTER_CHANGE

} // namespace physx
} // namespace omni
