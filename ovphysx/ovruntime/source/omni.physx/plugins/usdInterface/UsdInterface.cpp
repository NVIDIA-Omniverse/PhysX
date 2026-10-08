// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-BODY-001
 * @covers AC-6
 *
 * @implements REQ-SIM-MULTISCENE-001
 * @covers AC-4
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-7 AC-9 AC-10 AC-27 AC-29
 *
 * @implements REQ-SIM-ACTIVEACTOR-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-COL-004
 * @covers AC-3
 *
 * @implements REQ-SPLINE-TARGET-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-6
 *
 * @implements REQ-PARSE-MASS-003
 * @covers AC-1 AC-2
 *
 * @implements REQ-WRITE-AUTHORING-001
 * @covers AC-5
 */

#include "UsdInterface.h"
#include <ChangeRegister.h>
#include <CookingDataAsync.h>
#include <internal/Internal.h>
#include <internal/InternalScene.h>
#include <internal/InternalParticle.h>
#include <internal/InternalDeformable.h>
#include <internal/InternalDeformableAttachment.h>
#include <internal/InternalFilteredPairs.h>
#include <internal/InternalMimicJoint.h>
#include <particles/PhysXParticlePost.h>
#include <PhysXScene.h>
#include <PhysXTools.h>
#include <PhysXUSDProperties.h>
#include <PhysXSimulationCallbacks.h>
#include <Setup.h>
#include <OmniPhysX.h>
#include <Trigger.h>
#include <PhysXCustomJoint.h>
#include <PhysXCustomGeometry.h>
#include <Raycast.h>
#include <PhysXMirror.h>
#include <VehicleGenerator.h>
#include <ContactReport.h>
#include <ConeCylinderConvexMesh.h>
#include <usdLoad/LoadUsd.h>
#include <usdLoad/Scene.h>
#include <usdLoad/Mass.h>
#include <usdLoad/Material.h>
#include <usdLoad/MimicJoint.h>
#include <PhysXSettings.h>
#include <PhysXUpdate.h>

#include <omni/physics/parse/KnownTokens.h>

#include <private/omni/physx/PhysxUsd.h>
#include <omni/log/ILog.h>

#include <PxPhysicsAPI.h>
#include <omnipvd/PxOmniPvd.h>  // note: remove as soon as PxPhysicsAPI.h is fixed up
#include <extensions/PxCollectionExt.h>
#include <deformables/PhysXDeformablePost.h>

#include <common/utilities/MemoryMacros.h>

extern void* getPhysXPtr(omni::physics::parse::ObjectKey key, omni::physx::PhysXType type);
extern void* getInternalPtr(omni::physics::parse::ObjectKey key, omni::physx::PhysXType type);

using namespace ::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;
using namespace cookingdataasync;


OMNI_LOG_DECLARE_CHANNEL(kRoboticsLogChannel)

static std::vector<std::pair<ObjectId, PxArticulationLink*>> gTempParent;

namespace omni
{
namespace physx
{

PhysXUsdPhysicsInterface gPhysicsUsdInterface;

PhysXUsdPhysicsInterface& getPhysXUsdPhysicsInterface()
{
    return gPhysicsUsdInterface;
}

// Strip an unsupported (multi- or single-apply) API schema instance off the
// object's backing destination. Scene-description authoring, so it goes through the
// authoring sink (REQ-WRITE-AUTHORING-001) and still reaches a resident backing USD
// stage on an ovstage attach, where getDataWrite() is null by design. A no-op only
// when there is no backing destination at all (a USD-free consumer has nothing to strip).
// This is a cleanup/downgrade side effect that accompanies a warning ("this
// schema is ignored here"), not a read -- callers keep emitting their warning either way.
static void removeAppliedAPI(usdparser::AttachedStage& attachedStage,
                             omni::physics::parse::ObjectKey key,
                             std::string_view apiName,
                             std::string_view instanceName = {})
{
    if (omni::physics::parse::IPhysicsDataWrite* write = attachedStage.getAuthoringDataWrite())
        write->removeAppliedAPI(key, apiName, instanceName);
}

// Resolve an instance TokenId to InternalTendonAxis/InternalTendonAttachment's
// instanceName (TendonInstanceNameHandle, pinned to std::string in both configs
// -- InternalScene.h). Same pxr-free source-interning pattern as tokenText()
// below.
static std::string tendonInstanceNameFor(const usdparser::AttachedStage& attachedStage,
                                         omni::physics::parse::TokenId instanceToken)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    return source ? std::string(source->tokenToString(instanceToken)) : std::string();
}

// pxr-free diagnostic mirror: resolve a TokenId to plain text for %s log formatting,
// without materializing a TfToken. Returns "" when the source/token is invalid.
static std::string tokenText(const usdparser::AttachedStage& attachedStage, omni::physics::parse::TokenId token)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    return source ? std::string(source->tokenToString(token)) : std::string();
}

void clearFiltering()
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    physxSetup.getFilteredPairs().clear();
    physxSetup.getCollisionGroupFilteredPairs().clear();
}


// pxr-free diagnostic mirror: resolve an ObjectKey to its path text on the
// currently attached stage, without materializing an SdfPath. Used at record/
// error sites that lack an AttachedStage in scope; returns "" when no stage
// is attached.
static const char* textForActiveStage(omni::physics::parse::ObjectKey key)
{
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    return attachedStage ? attachedStage->textFor(key) : "";
}

// `key` is the shape's source identity: the record key, the PhysX debug name
// and every diagnostic come from it (via attachedStage.textFor), never from a
// live UsdPrim -- the optional USD trigger-state write-back below (Trigger.h's
// preloadTrigger) resolves its own prim from `key`, and is a no-op when there
// is nothing to author into (no backing stage).
ObjectId createShape(const PxGeometry& geom,
                     const PhysxShapeDesc& desc,
                     omni::physics::parse::ObjectKey key,
                     PhysXScene* physxScene,
                     uint32_t collisionGroup,
                     PxRigidActor* rigidActor,
                     bool exposePrimNames,
                     const PxTransform* poseOffset = nullptr,
                     ObjectId* compoundId = nullptr,
                     PhysXUsdPhysicsInterface::MassInformation* massInfoOut = nullptr,
                     const ObjectInstance* instance = nullptr,
                     const PxBounds3* geomBounds = nullptr)
{
    PxMassProperties massProperties(geom);
    PhysXUsdPhysicsInterface::MassInformation massInfo;

    // A.B. we need to take into account the units
    const float scalar = 1.0;
    massInfo.volume = massProperties.mass * scalar; // density is 1 so this is fine.
    const PxMat33 inertiaMatrix = (massProperties.inertiaTensor * scalar);

    massInfo.inertia[0] = inertiaMatrix.column0[0];
    massInfo.inertia[1] = inertiaMatrix.column0[1];
    massInfo.inertia[2] = inertiaMatrix.column0[2];

    massInfo.inertia[3] = inertiaMatrix.column1[0];
    massInfo.inertia[4] = inertiaMatrix.column1[1];
    massInfo.inertia[5] = inertiaMatrix.column1[2];

    massInfo.inertia[6] = inertiaMatrix.column2[0];
    massInfo.inertia[7] = inertiaMatrix.column2[1];
    massInfo.inertia[8] = inertiaMatrix.column2[2];

    massInfo.centerOfMass = { massProperties.centerOfMass.x, massProperties.centerOfMass.y,
                              massProperties.centerOfMass.z };

    if (poseOffset)
    {
        const PxQuat rot = toPhysXQuat(desc.localRot);
        const PxVec3 pos = toPhysX(desc.localPos);
        const PxTransform shapeTrans  = PxTransform(pos, rot) * (*poseOffset);
        massInfo.geometryToSourcePos = fromPhysX(poseOffset->p);
        massInfo.geometryToSourceRot = fromPhysX(poseOffset->q);
        massInfo.localPos = fromPhysX(shapeTrans.p);
        massInfo.localRot = fromPhysX(shapeTrans.q);
    }
    else
    {
        massInfo.localPos = desc.localPos;
        massInfo.localRot = desc.localRot;
    }

    PxBounds3 aabbLocalBounds;
    if (geomBounds)
    {
        aabbLocalBounds = *geomBounds;
    }
    else
    {
        PxGeometryQuery::computeGeomBounds(aabbLocalBounds, geom, PxTransform(PxIdentity));
    }

    massInfo.aabbLocalMin = fromPhysX(aabbLocalBounds.minimum);
    massInfo.aabbLocalMax = fromPhysX(aabbLocalBounds.maximum);
    if (massInfoOut)
        *massInfoOut = massInfo;

    if (physxScene == nullptr)
    {
        // This is where we return when doing authoring mass computations to avoid creating objects in physx
        return kInvalidObjectId;
    }

    PxMaterial* material = physxScene->getDefaultMaterial();
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    InternalMaterial* intMat = nullptr;
    if (desc.materials.size() == 1 && desc.materials[0] != kInvalidObjectId)
    {
        InternalDatabase::Record& materialRecord = db.getRecords()[desc.materials[0]];
        material = getPtr<PxMaterial>(ePTMaterial, desc.materials[0]);
        intMat = (InternalMaterial*)materialRecord.mInternalPtr;
    }
    CARB_ASSERT(material);

    // A.B. TODO we need to create exclusive shapes for now
    // to be able to move them around in Kit, instanced shapes might be shared
    const bool isExclusive = instance ? (instance->isExclusive ? true : false) : true;
    PxShape* shape = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createShape(geom, *material, isExclusive);
    if (!shape)
    {
        CARB_LOG_ERROR("PhysX Shape failed to be created on a prim: %s!", physxScene->getAttachedStage().textFor(key));
        return kInvalidObjectId;
    }

    PxFilterData fd;
    convertCollisionGroupToPxFilterData(collisionGroup, fd);
    shape->setSimulationFilterData(fd);
    shape->setQueryFilterData(fd);

    // setup triangle mesh materials
    if (geom.getType() == PxGeometryType::eTRIANGLEMESH && desc.materials.size() > 1)
    {
        if (desc.type == eTriangleMeshShape)
        {
            const TriangleMeshPhysxShapeDesc& trimeshDesc = ((const TriangleMeshPhysxShapeDesc&)desc);
            if (trimeshDesc.triangleMeshCookingParams.mode == TriangleMeshMode::eORIGINAL_TRIANGLES)
            {
                std::vector<PxMaterial*> materialArray;
                materialArray.resize(desc.materials.size());
                for (size_t i = 0; i < desc.materials.size(); i++)
                {
                    material = physxScene->getDefaultMaterial();
                    if (desc.materials[i] != kInvalidObjectId)
                    {
                        InternalDatabase::Record& materialRecord = db.getRecords()[desc.materials[i]];
                        material = getPtr<PxMaterial>(ePTMaterial, desc.materials[i]);
                    }
                    materialArray[i] = material;
                }
                shape->setMaterials(materialArray.data(), PxU16(materialArray.size()));
            }
            else
            {
                CARB_LOG_WARN("Multiple materials supported only for original triangle mesh (no approximation), prim: %s", physxScene->getAttachedStage().textFor(key));
            }
        }
        else
        {
            CARB_LOG_WARN("Multiple materials supported only for original triangle mesh (no approximation), prim: %s", physxScene->getAttachedStage().textFor(key));
        }
    }

    shape->setLocalPose(toPhysX(massInfo.localPos, massInfo.localRot));

    {
        float contactOffset = desc.contactOffset;
        if (desc.contactOffset >= 0.0f)
        {
            if (desc.contactOffset <= desc.restOffset)
            {
                CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", physxScene->getAttachedStage().textFor(key));
                contactOffset = desc.contactOffset + desc.restOffset + 1e-3f;
                shape->setContactOffset(contactOffset);
            }
            else
                shape->setContactOffset(desc.contactOffset);
        }
        else
        {
            const PxVec3 extents = aabbLocalBounds.getDimensions();

            const PxReal g = physxScene->getScene()->getGravity().magnitude();
            const PxReal dt = 1.0f / physxScene->getTimeStepsPerSeconds();
            const PxReal dynamicLowerThreshold = 2.0f * dt * dt * PxMax(g, 1.0f); //Make sure the lower bound is not exacly zero in case of zero gravity

            const PxReal minContactOffset = extents.minElement() * 0.02f;
            contactOffset = fmaxf(dynamicLowerThreshold, minContactOffset);

            if (isfinite(desc.restOffset) && desc.restOffset > 0.0f)
                contactOffset += desc.restOffset;

            shape->setContactOffset(contactOffset);
        }
        if (!isfinite(desc.restOffset))
            shape->setRestOffset(0.0f);
        else
        {
            if (desc.restOffset > contactOffset)
            {
                CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", physxScene->getAttachedStage().textFor(key));
                shape->setRestOffset(0.0f);
            }
            shape->setRestOffset(desc.restOffset);
        }
        shape->setTorsionalPatchRadius(desc.torsionalPatchRadius);
        shape->setMinTorsionalPatchRadius(desc.minTorsionalPatchRadius);
    }

    bool isTrigger = desc.isTrigger;
    if (isTrigger && desc.type == eTriangleMeshShape)
    {
        CARB_LOG_WARN("Triggers are not supported for triangle mesh shapes, prim: %s", physxScene->getAttachedStage().textFor(key));
        isTrigger = false;
    }

    if (isTrigger)
    {
        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
        shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, desc.collisionEnabled);
        shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
        // USD-only trigger state write-back; preloadTrigger no-ops when there is
        // nothing to author into (no backing stage).
        OmniPhysX::getInstance().getTriggerManager()->preloadTrigger(physxScene->getAttachedStage(), key, desc.isTriggerUsdOutput);
    }
    else
    {
        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, desc.collisionEnabled);
        shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, physxScene->isSceneQuerySupported() ? desc.collisionEnabled : false);
    }

    if (desc.collisionEnabled)
    {
        shape->setFlag(PxShapeFlag::eVISUALIZATION, OmniPhysX::getInstance().isDebugVisualizationEnabled());
    }
    else
    {
        shape->setFlag(PxShapeFlag::eVISUALIZATION, false);
    }

    if (rigidActor)
    {
        const bool retVal = rigidActor->attachShape(*shape);
        CARB_ASSERT(retVal);
    }

    ObjectId outId = kInvalidObjectId;

    if (!compoundId)
    {
        InternalShape* internalShape =
            ICE_NEW(InternalShape)(physxScene, desc.localScale, intMat ? desc.materials[0] : kInvalidObjectId);
        internalShape->mMassInfo = massInfo;
        if (instance)
            internalShape->mInstanceIndex = instance->index;
        if (intMat)
        {
            internalShape->mMaterialId = (usdparser::ObjectId)material->userData;
            intMat->addShapeId(db.getRecords().size());
        }
        outId = db.addRecord(ePTShape, shape, internalShape, key);
        shape->userData = (void*)(outId);
        if (exposePrimNames)
            shape->setName(physxScene->getAttachedStage().textFor(key));
    }
    else
    {
        PhysXType internalType;
        PhysXCompoundShape* cs = (PhysXCompoundShape*)db.getRecord(internalType, *compoundId);
        std::vector<::physx::PxShape*>& cShapes = const_cast<std::vector<::physx::PxShape*>&>(cs->getShapes());
        cShapes.push_back(shape);
        shape->userData = (void*)(*compoundId);
        if (exposePrimNames)
            shape->setName(physxScene->getAttachedStage().textFor(key));
    }

    return outId;
}

InternalActor* setupActor(PhysXScene* ps, PxRigidActor& actor, const PhysxRigidBodyDesc& desc, const ObjectInstance* instance, omni::physics::parse::ObjectKey key)
{
    const uint32_t nbShapes = uint32_t(desc.shapes.size());
    for (uint32_t i = 0; i < nbShapes; i++)
    {
        PxShape* shape = getPtr<PxShape>(ePTShape, desc.shapes[i]);
        if (shape)
        {
            const bool retVal = actor.attachShape(*shape);
            CARB_ASSERT(retVal);
        }
        else
        {
            const PhysXCompoundShape* compoundShape = getPtr<const PhysXCompoundShape>(ePTCompoundShape, desc.shapes[i]);
            if (compoundShape)
            {
                for (size_t k = 0; k < compoundShape->getShapes().size(); k++)
                {
                    PxShape* s = (PxShape*)compoundShape->getShapes()[k];
                    if (s->isExclusive())
                        s->setFlag(PxShapeFlag::eVISUALIZATION, OmniPhysX::getInstance().isDebugVisualizationEnabled());
                    const bool retVal = actor.attachShape(*s);
                    CARB_ASSERT(retVal);
                }
            }
        }
    }

    actor.setActorFlag(PxActorFlag::eVISUALIZATION, OmniPhysX::getInstance().isDebugVisualizationEnabled());

    InternalActor* internalActor = nullptr;
    if (desc.type != eArticulationLink)
    {
        bool localSpaceVelocities = false;

        if (actor.is<PxRigidDynamic>())
        {
            const DynamicPhysxRigidBodyDesc& dynDesc = static_cast<const DynamicPhysxRigidBodyDesc&>(desc);
            localSpaceVelocities = dynDesc.localSpaceVelocities;
        }

        internalActor = ICE_NEW(InternalActor)(ps,
            (actor.is<PxRigidBody>() && !(actor.is<PxRigidBody>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) ? true : false,
            instance, localSpaceVelocities, key);
    }
    else
    {
        internalActor = ICE_NEW(InternalLink)(ps, instance, key);
    }
    internalActor->mScale = desc.scale;
    ps->getInternalScene()->addActor(*internalActor);
    internalActor->mActor = &actor;

    // set flag if dynamic body has time sampled xform
    if (desc.type == eDynamicBody)
    {
        const DynamicPhysxRigidBodyDesc& dynDesc = static_cast<const DynamicPhysxRigidBodyDesc&>(desc);
        if (dynDesc.hasTimeSampledXform)
        {
            internalActor->mFlags |= InternalActorFlag::eHAS_TIME_SAMPLED_XFORM;
        }
    }

    // setup contact solve and surface velocity
    if (desc.type == eDynamicBody || desc.type == eArticulationLink)
    {
        const DynamicPhysxRigidBodyDesc& dynDesc = static_cast<const DynamicPhysxRigidBodyDesc&>(desc);
        internalActor->enableContactSolve(dynDesc.solveContacts, &actor);

        internalActor->mSurfaceVelocity = toPhysX(dynDesc.surfaceLinearVelocity);
        internalActor->mSurfaceVelocityAuthored = toPhysX(dynDesc.surfaceLinearVelocityAuthored);
        internalActor->mSurfaceAngularVelocity = toPhysX(dynDesc.surfaceAngularVelocity);
        internalActor->mSurfaceAngularVelocityPivot = actor.getGlobalPose();
        internalActor->mSurfaceVelocityLocalSpace = dynDesc.surfaceVelocityLocalSpace;
        internalActor->enableSurfaceVelocity(dynDesc.surfaceVelocityEnabled, actor);
        if (dynDesc.splinesSurfaceVelocityEnabled)
        {
            const AttachedStage& attachedStage = ps->getAttachedStage();
            const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
            const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
            if (src && dynDesc.splinesCurvePrimKey.valid() &&
                src->isA(dynDesc.splinesCurvePrimKey, tok.basisCurvesType))
            {
                internalActor->enableSplineSurfaceVelocity(dynDesc.splinesSurfaceVelocityEnabled, actor,
                                                           attachedStage, dynDesc.splinesCurvePrimKey);
            }
        }
        internalActor->mSplinesSurfaceVelocityMagnitude = dynDesc.splinesSurfaceVelocityMagnitude;
    }

    // set back joints broken by resync
    const PxJointMap& jointMap = OmniPhysX::getInstance().getInternalPhysXDatabase().getPxJointMap();
    PxJointMap::const_iterator jointIt = jointMap.find(key);
    while (jointIt != jointMap.end() && jointIt->first == key)
    {
        PxRigidActor* actor0 = nullptr;
        PxRigidActor* actor1 = nullptr;
        jointIt->second.first->getActors(actor0, actor1);
        if (jointIt->second.second)
        {
            actor0 = &actor;
        }
        else
        {
            actor1 = &actor;
        }
        jointIt->second.first->setActors(actor0, actor1);
        jointIt++;
    }
    return internalActor;
}

void applyPhysXRigidBodyDesc(PxRigidBody& rigidDynamic, const DynamicPhysxRigidBodyDesc& physxBodyDesc)
{
    rigidDynamic.setLinearDamping(physxBodyDesc.linearDamping);
    rigidDynamic.setAngularDamping(physxBodyDesc.angularDamping);

    rigidDynamic.setMaxAngularVelocity(physxBodyDesc.maxAngularVelocity);
    rigidDynamic.setMaxLinearVelocity(physxBodyDesc.maxLinearVelocity);

    rigidDynamic.setMaxDepenetrationVelocity(physxBodyDesc.maxDepenetrationVelocity);
    rigidDynamic.setContactSlopCoefficient(physxBodyDesc.contactSlopCoefficient);

    rigidDynamic.setMaxContactImpulse(physxBodyDesc.maxContactImpulse);

    rigidDynamic.setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, physxBodyDesc.enableCCD);
    rigidDynamic.setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, physxBodyDesc.enableSpeculativeCCD);
    rigidDynamic.setRigidBodyFlag(PxRigidBodyFlag::eRETAIN_ACCELERATIONS, physxBodyDesc.retainAccelerations);
    rigidDynamic.setRigidBodyFlag(PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, physxBodyDesc.enableGyroscopicForces);

    rigidDynamic.setActorFlag(PxActorFlag::eDISABLE_GRAVITY, physxBodyDesc.disableGravity);
}

void applyRigidDynamicPhysxDesc(PhysXScene* ps, const DynamicPhysxRigidBodyDesc& desc, ::physx::PxRigidDynamic& rigidDynamic)
{
    applyPhysXRigidBodyDesc(rigidDynamic, desc);
    rigidDynamic.setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, desc.kinematicBody);

    rigidDynamic.setActorFlag(PxActorFlag::eVISUALIZATION, OmniPhysX::getInstance().isDebugVisualizationEnabled());

    rigidDynamic.setSolverIterationCounts(
        ps->getInternalScene()->clampPosIterationCount(desc.solverPositionIterationCount),
        ps->getInternalScene()->clampVelIterationCount(desc.solverVelocityIterationCount));

    rigidDynamic.setSleepThreshold(desc.sleepThreshold);
    rigidDynamic.setStabilizationThreshold(desc.stabilizationThreshold);

    if (!desc.kinematicBody)
    {
        rigidDynamic.setLinearVelocity(toPhysX(desc.linearVelocity));
        rigidDynamic.setAngularVelocity(toPhysX(desc.angularVelocity));
    }

    if (desc.lockedPosAxis)
    {
        if (desc.lockedPosAxis & 1 << 0)
            rigidDynamic.setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_X, true);
        if (desc.lockedPosAxis & 1 << 1)
            rigidDynamic.setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y, true);
        if (desc.lockedPosAxis & 1 << 2)
            rigidDynamic.setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Z, true);
    }

    if (desc.lockedRotAxis)
    {
        if (desc.lockedRotAxis & 1 << 0)
            rigidDynamic.setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_X, true);
        if (desc.lockedRotAxis & 1 << 1)
            rigidDynamic.setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y, true);
        if (desc.lockedRotAxis & 1 << 2)
            rigidDynamic.setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, true);
    }
}

bool checkScenes()
{
    return !OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes().empty();
}

void releaseDeformableAttachments(InternalActor* intActor)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    for (MirrorActor& mirror : intActor->mMirrors)
    {
        PxScene* scene = mirror.actor->getScene();
        InternalDatabase::Record& actorRec = db.getRecords()[(ObjectId)scene->userData];
        InternalScene* intScene = (InternalScene*)actorRec.mInternalPtr;
        intScene->removeDeformableAttachments(ObjectId(mirror.actor->userData));
    }
    intActor->mPhysXScene->getInternalScene()->removeDeformableAttachments(ObjectId(intActor->mActor->userData));
}

void releaseDeformableCollisionFilters(InternalActor* intActor)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    for (MirrorActor& mirror : intActor->mMirrors)
    {
        PxScene* scene = mirror.actor->getScene();
        InternalDatabase::Record& actorRec = db.getRecords()[(ObjectId)scene->userData];
        InternalScene* intScene = (InternalScene*)actorRec.mInternalPtr;
        intScene->removeDeformableCollisionFilters(ObjectId(mirror.actor->userData));
    }
    intActor->mPhysXScene->getInternalScene()->removeDeformableCollisionFilters(ObjectId(intActor->mActor->userData));
}

void updateDeformableAttachmentShapeEvents(InternalActor* intActor, ObjectId shapeId, omni::physics::parse::ObjectKey shapeKey, DirtyEventType eventType)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (intActor)
    {
        // Refresh all attachments that has the same rb which the shape is attached to
        std::vector<PxRigidActor*> actorList = { intActor->mActor };
        for (MirrorActor& mirror : intActor->mMirrors)
        {
            actorList.push_back(mirror.actor);
        }

        for (PxRigidActor* actor : actorList)
        {
            const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();
            for (PhysXScenesMap::const_reference ref : physxScenes)
            {
                PhysXScene* sc = ref.second;

                std::vector<class InternalDeformableAttachment*>& attachmentList = sc->getInternalScene()->mDeformableAttachments;
                for (size_t i = 0; i < attachmentList.size(); ++i)
                {
                    attachmentList[i]->setRefreshAttachmentEvent(actor);
                }
            }
        }
    }
}

void updateDeformableCollisionFilterShapeEvents(InternalActor* intActor, ObjectId shapeId, omni::physics::parse::ObjectKey shapeKey, DirtyEventType eventType)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (intActor)
    {
        // Refresh all collision filters that has the same rb which the shape is attached to
        std::vector<PxRigidActor*> actorList = { intActor->mActor };
        for (MirrorActor& mirror : intActor->mMirrors)
        {
            actorList.push_back(mirror.actor);
        }

        for (PxRigidActor* actor : actorList)
        {
            const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();
            for (PhysXScenesMap::const_reference ref : physxScenes)
            {
                PhysXScene* sc = ref.second;

                std::vector<class InternalDeformableCollisionFilter*>& collisionFilterList = sc->getInternalScene()->mDeformableCollisionFilters;
                for (size_t i = 0; i < collisionFilterList.size(); ++i)
                {
                    collisionFilterList[i]->setRefreshCollisionFilterEvent(actor);
                }
            }
        }
    }
}

PhysXUsdPhysicsInterface::PhysXUsdPhysicsInterface()
    : mDirty(true)
    , mObjectChangeNotificationsEnabled(false)
    , mExposePrimNames(true)
{
}

PhysXUsdPhysicsInterface::~PhysXUsdPhysicsInterface()
{
}

void PhysXUsdPhysicsInterface::sendObjectCreationNotification(omni::physics::parse::ObjectKey key, ObjectId objectId, PhysXType physxType)
{
    if ((objectId != kInvalidObjectId) && !mPhysicsObjectChangeSubscriptions.map.empty())
    {
        PhysicsObjectChangeSubscriptionRegistry::EventMap::const_iterator it = mPhysicsObjectChangeSubscriptions.map.begin();
        PhysicsObjectChangeSubscriptionRegistry::EventMap::const_iterator itEnd = mPhysicsObjectChangeSubscriptions.map.end();
        while (it != itEnd)
        {
            const IPhysicsObjectChangeCallback& callback = it->second;
            if (shouldDeliverObjectChangeNotification(callback))
            {
                if (callback.objectCreationNotifyFn)
                {
                    callback.objectCreationNotifyFn(key, objectId, physxType, callback.userData);
                }
            }
            it++;
        }
    }
}

void PhysXUsdPhysicsInterface::sendObjectDestructionNotification(omni::physics::parse::ObjectKey removedKey, ObjectId objectId, PhysXType physxType)
{
    if ((objectId != kInvalidObjectId) && !mPhysicsObjectChangeSubscriptions.map.empty())
    {
        PhysicsObjectChangeSubscriptionRegistry::EventMap::const_iterator it = mPhysicsObjectChangeSubscriptions.map.begin();
        PhysicsObjectChangeSubscriptionRegistry::EventMap::const_iterator itEnd = mPhysicsObjectChangeSubscriptions.map.end();
        while (it != itEnd)
        {
            const IPhysicsObjectChangeCallback& callback = it->second;

            if (shouldDeliverObjectChangeNotification(callback))
            {
                if (callback.objectDestructionNotifyFn)
                {
                    callback.objectDestructionNotifyFn(removedKey, objectId, physxType, callback.userData);
                }
            }
            it++;
        }
    }
}

// PhysxJointAxisAPI instance schemas are pre-composed by KnownTokens (e.g.
// physxJointAxisAPIAngular), so this is a plain hasSchema lookup.
static bool hasPhysxJointAxisAPI(const usdparser::AttachedStage& attachedStage,
                                 omni::physics::parse::ObjectKey jointKey,
                                 omni::physics::parse::TokenId instanceSchemaToken)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    return source && source->hasSchema(jointKey, instanceSchemaToken);
}

// note: tConfigurePhysXJoint=false is for scenarios, where the InternalJoint object got removed
//       but the PhysX joint remained (for example, if some non-joint API schema on the joint prim
//       got removed). In such a case, only the InternalJoint object has to be reconstructed later
//       on.
template<bool tConfigurePhysXJoint>
static void createArticulationJoint(usdparser::AttachedStage& attachedStage, const PhysxJointDesc& jointDesc,
    PxArticulationJointReducedCoordinate* pxJoint, omni::physics::parse::ObjectKey childLinkKey, InternalLink* internalLink)
{
    if (!tConfigurePhysXJoint)
    {
        CARB_ASSERT(internalLink == nullptr);
    }

    const omni::physics::parse::ObjectKey jointPrimKey = jointDesc.jointPrimKey;

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::KnownTokens& jointTok = attachedStage.getKnownTokens();

    InternalJoint* intJoint = ICE_NEW(InternalJoint);

    if (intJoint)
    {
        if (tConfigurePhysXJoint)
            pxJoint->setJointType(PxArticulationJointType::eFIX);

        intJoint->mJointType = jointDesc.type;

        intJoint->mBody0IsParentLink = jointDesc.body1 == childLinkKey;
        if (!intJoint->mBody0IsParentLink)
        {
            OMNI_LOG_WARN(
                kRoboticsLogChannel,
                "Physics USD: Joint %s body rel does not follow articulation hierarchy; consider swapping body0/body1 rels to match.",
                attachedStage.textFor(jointPrimKey));
        }

        PxTransform localPose0;
        PxTransform localPose1;

        if (tConfigurePhysXJoint)
        {
            localPose0 = intJoint->mBody0IsParentLink ?
                toPhysX(jointDesc.localPose0Position, jointDesc.localPose0Orientation) :
                toPhysX(jointDesc.localPose1Position, jointDesc.localPose1Orientation);
            localPose1 = intJoint->mBody0IsParentLink ?
                toPhysX(jointDesc.localPose1Position, jointDesc.localPose1Orientation) :
                toPhysX(jointDesc.localPose0Position, jointDesc.localPose0Orientation);
        }
        else
        {
            CARB_UNUSED(localPose0);
            CARB_UNUSED(localPose1);
        }

        if (jointDesc.type == eJointFixed)
        {
            if (tConfigurePhysXJoint)
                pxJoint->setJointType(PxArticulationJointType::eFIX);
            else
                CARB_ASSERT(pxJoint->getJointType() == PxArticulationJointType::eFIX);
        }
        else if (jointDesc.type == eJointRevolute)
        {
            const RevolutePhysxJointDesc& revJointDesc = static_cast<const RevolutePhysxJointDesc&>(jointDesc);
            intJoint->mAxis = revJointDesc.axis;

            const usdparser::PhysxJointDrive& driveData = revJointDesc.drive;
            intJoint->mJointDrive = driveData;

            if (tConfigurePhysXJoint)
            {
                const usdparser::PhysxJointLimit& limitData = revJointDesc.limit;
                if (limitData.enabled)
                {
                    pxJoint->setJointType(PxArticulationJointType::eREVOLUTE_UNWRAPPED);
                    intJoint->setArticulationJointLimits(pxJoint, ::physx::PxArticulationAxis::eTWIST, limitData.lower, limitData.upper);
                    pxJoint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eLIMITED);
                }
                else
                {
                    pxJoint->setJointType(PxArticulationJointType::eREVOLUTE);
                    pxJoint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eFREE);
                }

                pxJoint->setArmature(::physx::PxArticulationAxis::eTWIST, revJointDesc.properties.armature);
                pxJoint->setMaxJointVelocity(::physx::PxArticulationAxis::eTWIST, revJointDesc.properties.maxJointVelocity);
                pxJoint->setFrictionParams(::physx::PxArticulationAxis::eTWIST, PxJointFrictionParams(revJointDesc.properties.staticFrictionEffort, revJointDesc.properties.dynamicFrictionEffort, revJointDesc.properties.viscousFrictionCoefficient));
                if(hasPhysxJointAxisAPI(attachedStage, jointPrimKey, jointTok.physxJointAxisAPIAngular))
                {
                    ObjectDb* objectDb = attachedStage.getObjectDatabase();
                    objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eJointAxisAngularAPI);
                }


                if (driveData.enabled)
                {
                    registerDriveTimeSampledChanges(attachedStage, jointPrimKey, "drive:angular");
                    pxJoint->setDriveParams(::physx::PxArticulationAxis::eTWIST, toPhysX(driveData));
                    intJoint->setArticulationDrivePositionTarget(pxJoint, ::physx::PxArticulationAxis::eTWIST, driveData.targetPosition, jointPrimKey);
                    intJoint->setArticulationDriveVelocityTarget(pxJoint, ::physx::PxArticulationAxis::eTWIST, driveData.targetVelocity);
                    if(driveData.isEnvelopeUsed)
                    {
                        ObjectDb* objectDb = attachedStage.getObjectDatabase();
                        objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eDrivePerformanceEnvelopeAngularAPI);
                    }
                }
            }
            else
            {
                CARB_ASSERT((pxJoint->getJointType() == PxArticulationJointType::eREVOLUTE) ||
                    (pxJoint->getJointType() == PxArticulationJointType::eREVOLUTE_UNWRAPPED));
            }

            // Joint State
            const usdparser::PhysicsJointState& jointState = revJointDesc.state;
            if (jointState.enabled)
            {
                if (tConfigurePhysXJoint)
                {
                    internalLink->hasInboundJointWithStateAPI = true;
                    intJoint->setArticulationJointPosition(pxJoint, ::physx::PxArticulationAxis::eTWIST, jointState.position);
                    intJoint->setArticulationJointVelocity(pxJoint, ::physx::PxArticulationAxis::eTWIST, jointState.velocity);
                }

                intJoint->mJointStates[0].usdToken = "angular";
                intJoint->mJointStates[0].enabled = true;
                intJoint->mJointStates[0].convertToDegrees = true;
                intJoint->mJointStates[0].physxAxis = ::physx::PxArticulationAxis::eTWIST;
                intJoint->mJointStates[0].initialState.position = radToDeg(jointState.position);
                intJoint->mJointStates[0].initialState.velocity = radToDeg(jointState.velocity);
            }
        }
        else if (jointDesc.type == eJointPrismatic)
        {
            const PrismaticPhysxJointDesc& prisJointDesc = static_cast<const PrismaticPhysxJointDesc&>(jointDesc);
            intJoint->mAxis = prisJointDesc.axis;

            const usdparser::PhysxJointDrive& driveData = prisJointDesc.drive;
            intJoint->mJointDrive = driveData;

            if (tConfigurePhysXJoint)
            {
                pxJoint->setJointType(PxArticulationJointType::ePRISMATIC);

                const usdparser::PhysxJointLimit& limitData = prisJointDesc.limit;
                if (limitData.enabled)
                {
                    intJoint->setArticulationJointLimits(pxJoint, ::physx::PxArticulationAxis::eX, limitData.lower, limitData.upper);
                    pxJoint->setMotion(::physx::PxArticulationAxis::eX, ::physx::PxArticulationMotion::eLIMITED);
                }
                else
                {
                    pxJoint->setMotion(::physx::PxArticulationAxis::eX, ::physx::PxArticulationMotion::eFREE);
                }

                pxJoint->setArmature(::physx::PxArticulationAxis::eX, prisJointDesc.properties.armature);
                pxJoint->setMaxJointVelocity(::physx::PxArticulationAxis::eX, prisJointDesc.properties.maxJointVelocity);
                pxJoint->setFrictionParams(::physx::PxArticulationAxis::eX, PxJointFrictionParams(prisJointDesc.properties.staticFrictionEffort, prisJointDesc.properties.dynamicFrictionEffort, prisJointDesc.properties.viscousFrictionCoefficient));

                if(hasPhysxJointAxisAPI(attachedStage, jointPrimKey, jointTok.physxJointAxisAPILinear))
                {
                    ObjectDb* objectDb = attachedStage.getObjectDatabase();
                    objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eJointAxisLinearAPI);
                }

                if (driveData.enabled)
                {
                    registerDriveTimeSampledChanges(attachedStage, jointPrimKey, "drive:linear");
                    pxJoint->setDriveParams(::physx::PxArticulationAxis::eX, toPhysX(driveData));
                    intJoint->setArticulationDrivePositionTarget(pxJoint, ::physx::PxArticulationAxis::eX, driveData.targetPosition);
                    intJoint->setArticulationDriveVelocityTarget(pxJoint, ::physx::PxArticulationAxis::eX, driveData.targetVelocity);
                    if(driveData.isEnvelopeUsed)
                    {
                        ObjectDb* objectDb = attachedStage.getObjectDatabase();
                        objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eDrivePerformanceEnvelopeLinearAPI);
                    }
                }
            }
            else
            {
                CARB_ASSERT(pxJoint->getJointType() == PxArticulationJointType::ePRISMATIC);
            }

            // Joint State
            const usdparser::PhysicsJointState& jointState = prisJointDesc.state;
            if (jointState.enabled)
            {
                if (tConfigurePhysXJoint)
                {
                    internalLink->hasInboundJointWithStateAPI = true;
                    intJoint->setArticulationJointPosition(pxJoint, ::physx::PxArticulationAxis::eX, jointState.position);
                    intJoint->setArticulationJointVelocity(pxJoint, ::physx::PxArticulationAxis::eX, jointState.velocity);
                }

                intJoint->mJointStates[0].usdToken = "linear";
                intJoint->mJointStates[0].enabled = true;
                intJoint->mJointStates[0].convertToDegrees = false;
                intJoint->mJointStates[0].physxAxis = ::physx::PxArticulationAxis::eX;
                intJoint->mJointStates[0].initialState.position = jointState.position;
                intJoint->mJointStates[0].initialState.velocity = jointState.velocity;
            }
        }
        else if (jointDesc.type == eJointSpherical)
        {
            const SphericalPhysxJointDesc& sphJointDesc = static_cast<const SphericalPhysxJointDesc&>(jointDesc);
            intJoint->mAxis = sphJointDesc.axis;

            if (tConfigurePhysXJoint)
            {
                pxJoint->setJointType(PxArticulationJointType::eSPHERICAL);

                // for limits and motions below, consider that the joint frames will be rotated such that:
                // 1) the joint axis is always eTWIST in PhysX
                // 2) and the SWING1/SWING2 correspond to cone0/cone1 limits if applicable
                pxJoint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eFREE);

                const usdparser::PhysxJointLimit& limitData = sphJointDesc.limit;
                if (limitData.enabled)
                {
                    pxJoint->setMotion(::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationMotion::eLIMITED);
                    pxJoint->setMotion(::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationMotion::eLIMITED);
                    pxJoint->setLimitParams(::physx::PxArticulationAxis::eSWING1, PxArticulationLimit(-limitData.angle0, limitData.angle0));
                    pxJoint->setLimitParams(::physx::PxArticulationAxis::eSWING2, PxArticulationLimit(-limitData.angle1, limitData.angle1));
                }
                else
                {
                    pxJoint->setMotion(::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationMotion::eFREE);
                    pxJoint->setMotion(::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationMotion::eFREE);
                }

                for (size_t i = 0; i < sphJointDesc.jointProperties.size(); i++)
                {
                    const JointAxis axis = sphJointDesc.jointProperties[i].first;
                    const PhysxJointAxisProperties& properties = sphJointDesc.jointProperties[i].second;
                
                    switch (axis)
                    {
                        case eTransX:
                        case eTransY:
                        case eTransZ:
                        case eDistance:
                            break;
                        case eRotX:
                            pxJoint->setArmature(::physx::PxArticulationAxis::eTWIST, properties.armature);
                            pxJoint->setMaxJointVelocity(::physx::PxArticulationAxis::eTWIST, properties.maxJointVelocity);
                            pxJoint->setFrictionParams(::physx::PxArticulationAxis::eTWIST, PxJointFrictionParams(properties.staticFrictionEffort, properties.dynamicFrictionEffort, properties.viscousFrictionCoefficient));
                            if(hasPhysxJointAxisAPI(attachedStage, jointPrimKey, jointTok.physxJointAxisAPIRotX))
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eJointAxisRotXAPI);
                            }
                            break;
                        case eRotY:
                            pxJoint->setArmature(::physx::PxArticulationAxis::eSWING1, properties.armature);
                            pxJoint->setMaxJointVelocity(::physx::PxArticulationAxis::eSWING1, properties.maxJointVelocity);
                            pxJoint->setFrictionParams(::physx::PxArticulationAxis::eSWING1, PxJointFrictionParams(properties.staticFrictionEffort, properties.dynamicFrictionEffort, properties.viscousFrictionCoefficient));
                            if(hasPhysxJointAxisAPI(attachedStage, jointPrimKey, jointTok.physxJointAxisAPIRotY))
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eJointAxisRotYAPI);
                            }
                              break;
                        case eRotZ:
                            pxJoint->setArmature(::physx::PxArticulationAxis::eSWING2, properties.armature);
                            pxJoint->setMaxJointVelocity(::physx::PxArticulationAxis::eSWING2, properties.maxJointVelocity);
                            pxJoint->setFrictionParams(::physx::PxArticulationAxis::eSWING2, PxJointFrictionParams(properties.staticFrictionEffort, properties.dynamicFrictionEffort, properties.viscousFrictionCoefficient));
                            if(hasPhysxJointAxisAPI(attachedStage, jointPrimKey, jointTok.physxJointAxisAPIRotZ))
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eJointAxisRotZAPI);
                            }
                            break;
                    }
                }
            }
        }
        else
        {
            const D6PhysxJointDesc& d6JointDesc = static_cast<const D6PhysxJointDesc&>(jointDesc);

            if (tConfigurePhysXJoint)
            {
                // D6 definition, we support only custom 3 dof in rotation axis
                pxJoint->setJointType(PxArticulationJointType::eSPHERICAL);
                pxJoint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eFREE);
                pxJoint->setMotion(::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationMotion::eFREE);
                pxJoint->setMotion(::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationMotion::eFREE);

                for (size_t i = 0; i < d6JointDesc.jointProperties.size(); i++)
                {
                    const JointAxis axis = d6JointDesc.jointProperties[i].first;
                    const PhysxJointAxisProperties& properties = d6JointDesc.jointProperties[i].second;
                
                    switch (axis)
                    {
                        case eTransX:
                        case eTransY:
                        case eTransZ:
                        case eDistance:
                            break;
                        case eRotX:
                            pxJoint->setArmature(::physx::PxArticulationAxis::eTWIST, properties.armature);
                            pxJoint->setMaxJointVelocity(::physx::PxArticulationAxis::eTWIST, properties.maxJointVelocity);
                            pxJoint->setFrictionParams(::physx::PxArticulationAxis::eTWIST, PxJointFrictionParams(properties.staticFrictionEffort, properties.dynamicFrictionEffort, properties.viscousFrictionCoefficient));
                            if(hasPhysxJointAxisAPI(attachedStage, jointPrimKey, jointTok.physxJointAxisAPIRotX))
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eJointAxisRotXAPI);
                            }
                            break;
                        case eRotY:
                            pxJoint->setArmature(::physx::PxArticulationAxis::eSWING1, properties.armature);
                            pxJoint->setMaxJointVelocity(::physx::PxArticulationAxis::eSWING1, properties.maxJointVelocity);
                            pxJoint->setFrictionParams(::physx::PxArticulationAxis::eSWING1, PxJointFrictionParams(properties.staticFrictionEffort, properties.dynamicFrictionEffort, properties.viscousFrictionCoefficient));
                            if(hasPhysxJointAxisAPI(attachedStage, jointPrimKey, jointTok.physxJointAxisAPIRotY))
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eJointAxisRotYAPI);
                            }
                            break;
                        case eRotZ:
                            pxJoint->setArmature(::physx::PxArticulationAxis::eSWING2, properties.armature);
                            pxJoint->setMaxJointVelocity(::physx::PxArticulationAxis::eSWING2, properties.maxJointVelocity);
                            pxJoint->setFrictionParams(::physx::PxArticulationAxis::eSWING2, PxJointFrictionParams(properties.staticFrictionEffort, properties.dynamicFrictionEffort, properties.viscousFrictionCoefficient));
                            if(hasPhysxJointAxisAPI(attachedStage, jointPrimKey, jointTok.physxJointAxisAPIRotZ))
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eJointAxisRotZAPI);
                            }
                            break;
                    }
                }
                             
                uint32_t lockedTransAxis = 0;
                for (size_t i = 0; i < d6JointDesc.jointLimits.size(); i++)
                {
                    const PhysxJointLimit& currentLimit = d6JointDesc.jointLimits[i].second;
                    if (d6JointDesc.jointLimits[i].first == eRotX)
                    {
                        if (currentLimit.enabled)
                        {
                            if (currentLimit.lower > currentLimit.upper)
                            {
                                pxJoint->setMotion(::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eLOCKED);
                            }
                            else
                            {
                                intJoint->setArticulationJointLimits(pxJoint, ::physx::PxArticulationAxis::eTWIST, currentLimit.lower, currentLimit.upper);
                                pxJoint->setMotion(
                                    ::physx::PxArticulationAxis::eTWIST, ::physx::PxArticulationMotion::eLIMITED);
                            }
                        }
                    }
                    else if (d6JointDesc.jointLimits[i].first == eRotY)
                    {
                        if (currentLimit.enabled)
                        {
                            if (currentLimit.lower > currentLimit.upper)
                            {
                                pxJoint->setMotion(
                                    ::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationMotion::eLOCKED);
                            }
                            else
                            {
                                intJoint->setArticulationJointLimits(pxJoint, ::physx::PxArticulationAxis::eSWING1, currentLimit.lower, currentLimit.upper);
                                pxJoint->setMotion(
                                    ::physx::PxArticulationAxis::eSWING1, ::physx::PxArticulationMotion::eLIMITED);
                            }
                        }
                    }
                    else if (d6JointDesc.jointLimits[i].first == eRotZ)
                    {
                        if (currentLimit.enabled)
                        {
                            if (currentLimit.lower > currentLimit.upper)
                            {
                                pxJoint->setMotion(
                                    ::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationMotion::eLOCKED);
                            }
                            else
                            {
                                intJoint->setArticulationJointLimits(pxJoint, ::physx::PxArticulationAxis::eSWING2, currentLimit.lower, currentLimit.upper);
                                pxJoint->setMotion(
                                    ::physx::PxArticulationAxis::eSWING2, ::physx::PxArticulationMotion::eLIMITED);
                            }
                        }
                    }
                    else if (d6JointDesc.jointLimits[i].first == eTransX)
                    {
                        if (currentLimit.enabled)
                        {
                            if (currentLimit.lower > currentLimit.upper)
                            {
                                lockedTransAxis |= 1 << 0;
                            }
                        }
                    }
                    else if (d6JointDesc.jointLimits[i].first == eTransY)
                    {
                        if (currentLimit.enabled)
                        {
                            if (currentLimit.lower > currentLimit.upper)
                            {
                                lockedTransAxis |= 1 << 1;
                            }
                        }
                    }
                    else if (d6JointDesc.jointLimits[i].first == eTransZ)
                    {
                        if (currentLimit.enabled)
                        {
                            if (currentLimit.lower > currentLimit.upper)
                            {
                                lockedTransAxis |= 1 << 2;
                            }
                        }
                    }
                }

                if (lockedTransAxis != 7)
                {
                    CARB_LOG_WARN("Translation DOF is expected to be locked for D6 joint in articulation hierarchy, the resulting joint in articulation will have all translation DOF locked.");
                }
            }

            bool hasUnsupportedLinearDrives = false;
            // setup joint drives:
            for (size_t i = 0; i < d6JointDesc.jointDrives.size(); i++)
            {
                const JointAxis axis = d6JointDesc.jointDrives[i].first;
                const PhysxJointDrive& driveData = d6JointDesc.jointDrives[i].second;

                switch (axis)
                {
                case eTransX:
                case eTransY:
                case eTransZ:
                    hasUnsupportedLinearDrives = true;
                    break;
                case eDistance:
                    break;
                case eRotX:
                    if (driveData.enabled)
                    {
                        const ::physx::PxArticulationAxis::Enum pxAxis = ::physx::PxArticulationAxis::eTWIST;
                        intJoint->mJointDrives[pxAxis] = driveData;

                        if (tConfigurePhysXJoint)
                        {
                            registerDriveTimeSampledChanges(attachedStage, jointPrimKey, "drive:rotX");
                            pxJoint->setDriveParams(pxAxis, toPhysX(driveData));
                            intJoint->setArticulationDrivePositionTarget(pxJoint, pxAxis, driveData.targetPosition);
                            intJoint->setArticulationDriveVelocityTarget(pxJoint, pxAxis, driveData.targetVelocity);
                            if(driveData.isEnvelopeUsed)
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eDrivePerformanceEnvelopeRotXAPI);
                            }
                        }
                    }
                    break;
                case eRotY:
                    if (driveData.enabled)
                    {
                        const ::physx::PxArticulationAxis::Enum pxAxis = ::physx::PxArticulationAxis::eSWING1;
                        intJoint->mJointDrives[pxAxis] = driveData;

                        if (tConfigurePhysXJoint)
                        {
                            registerDriveTimeSampledChanges(attachedStage, jointPrimKey, "drive:rotY");
                            pxJoint->setDriveParams(pxAxis, toPhysX(driveData));
                            intJoint->setArticulationDrivePositionTarget(pxJoint, pxAxis, driveData.targetPosition);
                            intJoint->setArticulationDriveVelocityTarget(pxJoint, pxAxis, driveData.targetVelocity);
                            if(driveData.isEnvelopeUsed)
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eDrivePerformanceEnvelopeRotYAPI);
                            }
                        }
                    }
                    break;
                case eRotZ:
                    if (driveData.enabled)
                    {
                        const ::physx::PxArticulationAxis::Enum pxAxis = ::physx::PxArticulationAxis::eSWING2;
                        intJoint->mJointDrives[pxAxis] = driveData;

                        if (tConfigurePhysXJoint)
                        {
                            registerDriveTimeSampledChanges(attachedStage, jointPrimKey, "drive:rotZ");
                            pxJoint->setDriveParams(pxAxis, toPhysX(driveData));
                            intJoint->setArticulationDrivePositionTarget(pxJoint, pxAxis, driveData.targetPosition);
                            intJoint->setArticulationDriveVelocityTarget(pxJoint, pxAxis, driveData.targetVelocity);
                            if(driveData.isEnvelopeUsed)
                            {
                                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                                objectDb->addSchemaAPI(jointPrimKey, SchemaAPIFlag::eDrivePerformanceEnvelopeRotZAPI);
                            }
                        }
                    }
                    break;
                }
            }
            if (hasUnsupportedLinearDrives)
            {
                CARB_LOG_WARN("Linear drives on articulation D6 joints are not supported - drive is ignored. Joint: %s.", attachedStage.textFor(jointPrimKey));
            }

            for (size_t i = 0; i < d6JointDesc.jointStates.size(); ++i)
            {
                ::physx::PxArticulationAxis::Enum axis = ::physx::PxArticulationAxis::eTWIST;
                const usdparser::PhysicsJointState& jointState = d6JointDesc.jointStates[i].second;
                bool hasUnsupportedLinearJointStates = false;
                switch (d6JointDesc.jointStates[i].first)
                {
                case eRotX: axis = ::physx::PxArticulationAxis::eTWIST; break;
                case eRotY: axis = ::physx::PxArticulationAxis::eSWING1; break;
                case eRotZ: axis = ::physx::PxArticulationAxis::eSWING2; break;
                default:
                    hasUnsupportedLinearJointStates = true;
                }

                if (hasUnsupportedLinearJointStates)
                {
                    CARB_LOG_WARN("Linear joint states on articulation D6 joints are not supported - state is ignored. Joint: %s.", attachedStage.textFor(jointPrimKey));
                }
                else if (jointState.enabled)
                {
                    if (tConfigurePhysXJoint)
                    {
                        internalLink->hasInboundJointWithStateAPI = true;
                        intJoint->setArticulationJointPosition(pxJoint, axis, jointState.position);
                        intJoint->setArticulationJointVelocity(pxJoint, axis, jointState.velocity);
                    }

                    switch (d6JointDesc.jointStates[i].first)
                    {
                    case eRotX: intJoint->mJointStates[i].usdToken = "rotX"; break;
                    case eRotY: intJoint->mJointStates[i].usdToken = "rotY"; break;
                    case eRotZ: intJoint->mJointStates[i].usdToken = "rotZ"; break;
                    }

                    intJoint->mJointStates[i].enabled = true;
                    intJoint->mJointStates[i].convertToDegrees = true;
                    intJoint->mJointStates[i].physxAxis = axis;
                    intJoint->mJointStates[i].initialState.position = radToDeg(jointState.position);
                    intJoint->mJointStates[i].initialState.velocity = radToDeg(jointState.velocity);
                }
            }
        }

        if (tConfigurePhysXJoint)
        {
            // take joint axis into account, if necessary
            intJoint->fixupLocalPose(localPose0);
            intJoint->fixupLocalPose(localPose1);
            pxJoint->setParentPose(localPose0);
            pxJoint->setChildPose(localPose1);

            pxJoint->setFrictionCoefficient(jointDesc.jointFriction);
        }

        InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        const ObjectId jointObjId = db.addRecord(ePTLinkJoint, pxJoint, intJoint, jointPrimKey);
        pxJoint->userData = (void*)jointObjId;
        pxJoint->setName(attachedStage.textFor(jointPrimKey));
        attachedStage.getObjectDatabase()->findOrCreateEntry(jointPrimKey, attachedStage.textFor(jointPrimKey), eArticulationJoint, jointObjId);
    }
    else
    {
        OMNI_LOG_ERROR(
            kRoboticsLogChannel,
            "Physics USD: joint %s: memory allocation for internal joint object failed.",
            attachedStage.textFor(jointPrimKey));
    }
}

ObjectId PhysXUsdPhysicsInterface::createObject(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key,
                                                const PhysxObjectDesc& objectDesc,
                                                const ObjectInstance* instance)
{
    if (objectDesc.type != eScene && objectDesc.type !=  eCollisionGroup)
    {
        if (!checkScenes())
        {
            PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
            return kInvalidObjectId;
        }
    }
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
    InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    // scristiano: this is temporary code to allow physics inspector. we should filter simulation
    // owners at parsing stage. Kit-inspector-only debug filter, opt-in via an explicit Kit
    // setting; runs under any backend since findEntry/textFor are the key-native overloads.
    if(!mForceParseOnlySingleScenePath.empty())
    {
        switch (objectDesc.type)
        {
            case eScene:
            {
                if (attachedStage.textFor(key) != mForceParseOnlySingleScenePath)
                {
                    return kInvalidObjectId;
                }
            }
            break;
            case eDynamicBody:
            {
                const DynamicPhysxRigidBodyDesc& desc = static_cast<const DynamicPhysxRigidBodyDesc&>(objectDesc);
                const ObjectId sceneId = desc.sceneIds.empty() ? kInvalidObjectId : desc.sceneIds[0];
                if(sceneId != mForceParseOnlySingleSceneObjectId)
                    return kInvalidObjectId;
            }
            break;
            case eStaticBody:
            {
                const StaticPhysxRigidBodyDesc& desc = static_cast<const StaticPhysxRigidBodyDesc&>(objectDesc);
                const ObjectId sceneId = desc.sceneIds.empty() ? kInvalidObjectId : desc.sceneIds[0];
                if(sceneId != mForceParseOnlySingleSceneObjectId)
                    return kInvalidObjectId;
            }
            break;
            case eArticulationLink:
            {
                const PhysxArticulationLinkDesc& desc = static_cast<const PhysxArticulationLinkDesc&>(objectDesc);
                const ObjectId sceneId = desc.sceneIds.empty() ? kInvalidObjectId : desc.sceneIds[0];
                if(sceneId != mForceParseOnlySingleSceneObjectId)
                    return kInvalidObjectId;
            }
            break;
            case eArticulation:
            {
                const PhysxArticulationDesc& desc = static_cast<const PhysxArticulationDesc&>(objectDesc);
                if(desc.sceneId != mForceParseOnlySingleSceneObjectId)
                    return kInvalidObjectId;
            }
            break;
            case eTendonFixed:
            {
                const PhysxTendonFixedDesc& desc = static_cast<const PhysxTendonFixedDesc&>(objectDesc);

                // Find parent joint of tendon root joint (needed for dummy joint)
                ObjectId parentLinkJointId = attachedStage.getObjectDatabase()->findEntry(key, eArticulationJoint);

                if (parentLinkJointId == kInvalidObjectId)
                {
                    return kInvalidObjectId;
                }
            }
            break;
            case eTendonAxis:
            {
                const PhysxTendonAxisDesc& desc = static_cast<const PhysxTendonAxisDesc&>(objectDesc);

                // Get link joint corresponding to tendon axis
                ObjectId linkJointId = attachedStage.getObjectDatabase()->findEntry(key, eArticulationJoint);

                if (linkJointId == kInvalidObjectId)
                {
                    return kInvalidObjectId;
                }
            }
            break;
            case eTendonAttachmentRoot:
            {
                const PhysxTendonSpatialDesc& desc = static_cast<const PhysxTendonSpatialDesc&>(objectDesc);

                // find articulation link that root attachment should attach to
                const ObjectId linkId = attachedStage.getObjectDatabase()->findEntry(key, eArticulationLink);

                if (linkId == kInvalidObjectId)
                {
                    return kInvalidObjectId;
                }
            }
            break;
            case eTendonAttachmentLeaf:
            case eTendonAttachment:
            {
                const PhysxTendonAttachmentDesc& desc = static_cast<const PhysxTendonAttachmentDesc&>(objectDesc);

                // find corresponding link that attachment belongs to
                const ObjectId linkId = attachedStage.getObjectDatabase()->findEntry(key, eArticulationLink);

                if (linkId == kInvalidObjectId)
                {
                    return kInvalidObjectId;
                }
            }
            break;
            case eCapsuleCct:
            {
                const CapsuleCctDesc& desc = static_cast<const CapsuleCctDesc&>(objectDesc);
                if(desc.sceneId != mForceParseOnlySingleSceneObjectId)
                    return kInvalidObjectId;
            }
            break;
            case eMimicJointRotX:
            case eMimicJointRotY:
            case eMimicJointRotZ:
            case eNewtonMimicJoint:
            {
                // find corresponding joint that mimic joint belongs to
                const ObjectId articulationJointID = attachedStage.getObjectDatabase()->findEntry(key, eArticulationJoint);

                if (articulationJointID == kInvalidObjectId)
                {
                    return kInvalidObjectId;
                }
            }
            break;
            case eInfiniteVoxelMap:
            case eParticleSystem:
            case eParticleSet:
            {
                // scristiano: No multiple scenes support for these objects, so we don't know if we can create them
                return kInvalidObjectId;
            }
            break;
            case ePBDMaterial:
            case ePointInstancedBody:
            case eVehicle:
            case eVehicleControllerStandard:
            case eVehicleControllerTank:
            case eVehicleEngine:
            case eVehicleTireFrictionTable:
            case eVehicleSuspension:
            case eVehicleTire:
            case eVehicleWheel:
            case eVehicleWheelAttachment:
            case eVehicleWheelController:
            case eVehicleDriveBasic:
            case eFilteredPair:
            case eDeformableMaterial:
            case eSurfaceDeformableMaterial:
            case eVolumeDeformableBody:
            case eSurfaceDeformableBody:
            {
                // scristiano: Figure out what to do with these object types as well
                return kInvalidObjectId;
            }
            break;
            case eMaterial:
            case eCollisionGroup:
            {
                // scristiano: Objects that we create anyway in any case
            }
            break;
            default:
            {
                CARB_LOG_ERROR("Unsupported object type %d to filter when force parsing single scene", objectDesc.type);
                return kInvalidObjectId;
            }
            break;
        }
    }

    ObjectId outId = kInvalidObjectId;
    PhysXType physxType = ePTRemoved;

    switch (objectDesc.type)
    {
    default:
    {
        CARB_ASSERT(!"Unsupported type");
        return kInvalidObjectId;
    }
    break;

    case eInfiniteVoxelMap:
    {
        // Unsupported in the USD-free runtime: no ePTInfiniteVoxelMap record is ever published.
        CARB_LOG_WARN("InfiniteVoxelMapAPI on '%s' is not supported by the USD-free runtime; the voxel map is ignored",
                      attachedStage.textFor(key));
        return kInvalidObjectId;
    }
    break;
    case eMaterial:
    {
        const usdparser::PhysxMaterialDesc& desc = static_cast<const usdparser::PhysxMaterialDesc&>(objectDesc);
        PxMaterial* material = nullptr;
        // special case, we cant create the default scene material first
        PhysXScene* physxScene = nullptr;
        const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();

        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            PhysXScene* sc = ref.second;
            if (key == sc->getDefaultMaterialPath())
            {
                physxScene = sc;
                break;
            }
        }
        if (!physxScene)
        {
            material = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createMaterial(desc.staticFriction, desc.dynamicFriction, desc.restitution);

            if (material)
            {
                material->setFrictionCombineMode((PxCombineMode::Enum)desc.frictionCombineMode);
                material->setRestitutionCombineMode((PxCombineMode::Enum)desc.restitutionCombineMode);
                material->setDampingCombineMode((PxCombineMode::Enum)desc.dampingCombineMode);
                material->setFlag(PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING, desc.compliantAccelerationSpring);
                if (desc.compliantStiffness > 0.0f)
                {
                    material->setRestitution(-desc.compliantStiffness);
                }
                // preist: set damping in any case to get error from Np layer if nonzero damping is set while stiffness
                // zero and compliance is off
                material->setDamping(desc.compliantDamping);
            }
        }
        else
        {
            material = physxScene->getDefaultMaterial();
        }

        if (material)
        {
            InternalMaterial* internalMat = ICE_NEW(InternalMaterial)(desc.density);
            physxType = ePTMaterial;
            outId = db.addRecord(physxType, material, internalMat, key);
            material->userData = (void*)(outId);
        }
    }
    break;
    case eDeformableMaterial:
    {
        const usdparser::PhysxDeformableMaterialDesc& desc =
            static_cast<const usdparser::PhysxDeformableMaterialDesc&>(objectDesc);

        PxDeformableVolumeMaterial* material = nullptr;
        PhysXScene* physxScene = nullptr;
        const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();

        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            PhysXScene* sc = ref.second;
            if (key == sc->getDefaultVolumeDeformableMaterialPath())
            {
                physxScene = sc;
                break;
            }
        }

        if (!physxScene)
        {
            material = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createDeformableVolumeMaterial(
                desc.youngsModulus, desc.poissonsRatio, desc.dynamicFriction, desc.elasticityDamping);
        }
        else
        {
            material = physxScene->getDefaultVolumeDeformableMaterial();
        }

        if (material)
        {
            InternalDeformableMaterial* internalMat = ICE_NEW(InternalDeformableMaterial)(desc.density);
            physxType = ePTDeformableVolumeMaterial;
            // Key by the descriptor `path`, like every other addRecord here. The
            // previous usdPrim.GetPrimPath() is identical under the USD backend but
            // EMPTY under the descriptor-driven ovstage backend (usdPrim is invalid
            // there, see usdPrim init above), so the material was registered under an
            // empty key and getPhysXPtr(/path, ePTDeformable*Material) could never
            // find it -> deformable-material tensor views matched 0 prims.
            outId = db.addRecord(physxType, material, internalMat, key);
            material->userData = (void*)(outId);
        }
    }
    break;
    case eSurfaceDeformableMaterial:
    {
        const usdparser::PhysxSurfaceDeformableMaterialDesc& desc =
            static_cast<const usdparser::PhysxSurfaceDeformableMaterialDesc&>(objectDesc);

        PxDeformableSurfaceMaterial* material = nullptr;
        PhysXScene* physxScene = nullptr;
        const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();

        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            PhysXScene* sc = ref.second;
            if (key == sc->getDefaultSurfaceDeformableMaterialPath())
            {
                physxScene = sc;
                break;
            }
        }

        if (!physxScene)
        {
            material = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createDeformableSurfaceMaterial(
                desc.youngsModulus, desc.poissonsRatio, desc.dynamicFriction, desc.surfaceThickness,
                desc.surfaceBendStiffness, desc.elasticityDamping, desc.bendDamping);
        }
        else
        {
            material = physxScene->getDefaultSurfaceDeformableMaterial();
        }

        if (material)
        {
            InternalDeformableMaterial* internalMat = ICE_NEW(InternalDeformableMaterial)(desc.density);
            physxType = ePTDeformableSurfaceMaterial;
            // Key by the descriptor `path`, like every other addRecord here. The
            // previous usdPrim.GetPrimPath() is identical under the USD backend but
            // EMPTY under the descriptor-driven ovstage backend (usdPrim is invalid
            // there, see usdPrim init above), so the material was registered under an
            // empty key and getPhysXPtr(/path, ePTDeformable*Material) could never
            // find it -> deformable-material tensor views matched 0 prims.
            outId = db.addRecord(physxType, material, internalMat, key);
            material->userData = (void*)(outId);
        }
    }
    break;
    case ePBDMaterial:
    {
        const usdparser::PBDMaterialDesc& desc = static_cast<const usdparser::PBDMaterialDesc&>(objectDesc);

        PxPBDMaterial* material = nullptr;
        PhysXScene* physxScene = nullptr;
        const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();

        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            PhysXScene* sc = ref.second;
            if (key == sc->getDefaultPBDMaterialPath())
            {
                physxScene = sc;
                break;
            }
        }

        if (!physxScene)
        {
            material = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createPBDMaterial(
                desc.friction, desc.damping, desc.adhesion, desc.viscosity, desc.vorticityConfinement,
                desc.surfaceTension, desc.cohesion, 0.0f, 0.0f, desc.cflCoefficient, desc.gravityScale);

            if (material)
            {
                material->setParticleFrictionScale(desc.particleFrictionScale);
                material->setParticleAdhesionScale(desc.particleAdhesionScale);
                material->setAdhesionRadiusScale(desc.adhesionOffsetScale);
            }
        }
        else
        {
            material = physxScene->getDefaultPBDMaterial();
        }

        if (material)
        {
            InternalPBDParticleMaterial* internalMat = ICE_NEW(InternalPBDParticleMaterial)(desc.density);
            outId = db.addRecord(ePTPBDMaterial, material, internalMat, key);
            material->userData = (void*)(outId);
        }
    }
    break;
    case ePointInstancedBody:
    {
        physxType = ePTPointInstancer;
        outId = db.addRecord(physxType, nullptr, nullptr, key);
    }
    break;
    case eScene:
    {
        // physx scene params
        const usdparser::PhysxSceneDesc& desc = static_cast<const usdparser::PhysxSceneDesc&>(objectDesc);

        physxType = ePTScene;
        outId = db.addRecord(physxType, nullptr, nullptr, key);
        // Kit-inspector-only debug filter -- see setForceParseOnlySingleScene's comment.
        if(!mForceParseOnlySingleScenePath.empty())
        {
            mForceParseOnlySingleSceneObjectId = outId;
        }
        // Units come from the attached source (works under any backend — USD or
        // ovstage — without a live USD stage), not the USD stage directly.
        const omni::physics::parse::SourceUnits sceneUnits =
            attachedStage.getSource() ? attachedStage.getSource()->getSourceUnits() : omni::physics::parse::SourceUnits{};
        PhysXScene* physxScene = physxSetup.createPhysXScene(
            attachedStage, outId, sceneUnits.metersPerUnit, sceneUnits.kilogramsPerUnit, desc);
        if (physxScene)
        {
            PxScene* scene = physxScene->getScene();
            db.getRecords()[outId].mPtr = scene;
            db.getRecords()[outId].mInternalPtr = physxScene->getInternalScene();

            // Creation-time replicator env-ids (opt-in, /physics/replicatorEnvIdsOnAttach): flip
            // the AttachedStage flag before any body parses so every body created under this
            // attach receives its environment id at creation (scene-partition primvar id, or 0)
            // — outside the scene, where assignment is legal. This is how a cloneEnvironments()
            // source becomes collision-isolated from co-located copies WITHOUT remove/re-add of
            // live scene objects (remove+re-add within one step window corrupts
            // eENABLE_DIRECT_GPU_API scenes — PxgBodySimManager has no rigid-body unstaging).
            // Same GPU gate as replicate(): env-ids only act under GPU dynamics + GPU broadphase.
            {
                carb::settings::ISettings* iSettings = OmniPhysX::getInstance().getISettings();
                if (iSettings && iSettings->getAsBool(kSettingReplicatorEnvIdsOnAttach))
                {
                    const bool gpuDynamics = scene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;
                    const bool gpuBroadphase = scene->getBroadPhaseType() == PxBroadPhaseType::eGPU;
                    if (gpuDynamics && gpuBroadphase)
                        attachedStage.setUseReplicatorEnvIds(true);
                }
            }
            // physics scene params
            {
                InternalScene* intScene = physxScene->getInternalScene();
                intScene->mGravityDirection = toPhysX(desc.gravityDirection);
                intScene->mGravityMagnitude = desc.gravityMagnitude;
                // registerSceneTimeSampledChanges reads through
                // IPhysicsSource::isAttributeTimeSampled, so it naturally registers
                // nothing on a backend that never reports time samples.
                registerSceneTimeSampledChanges(attachedStage, key);
                scene->setGravity(PxVec3(desc.gravityDirection.x * desc.gravityMagnitude, desc.gravityDirection.y * desc.gravityMagnitude, desc.gravityDirection.z * desc.gravityMagnitude));
            }
            scene->userData = (void*)(outId);
        }
        else
        {
            CARB_LOG_ERROR("Failed to create PhysX Scene (prim: %s), no simulation will happen. Please report this issue ideally with a log and repro.", attachedStage.textFor(key));
        }
    }
    break;

    case eParticleSystem:
    {
        // Re-entrant-scan guard: attachOvstage's initial-stage-load has been observed to run
        // loadFromRange twice with identical scan roots (TestParticles.cpp's "Particle System
        // Scene Ownership Resolves Second Scene" caught it -- getNbPBDParticleSystems() came
        // back 2, not 1), so createObject can be asked to create the same particle-system prim
        // twice. createPbdParticleSystem has no existence check of its own and would spawn a
        // second, leaked PxPBDParticleSystem; unlike most other object types (whose duplicate
        // engine object is merely unused and overwritten in the ObjectDb, invisible to a test),
        // a particle system's actor count is directly observable
        // (PxScene::getNbPBDParticleSystems), so this is the one spot worth guarding
        // explicitly here. The double full-load itself is a separate, open issue.
        const ObjectId existingParticleSystemId = attachedStage.getObjectDatabase()->findEntry(key, eParticleSystem);
        if (existingParticleSystemId != kInvalidObjectId)
            return existingParticleSystemId;

        const ParticleSystemDesc& desc = static_cast<const ParticleSystemDesc&>(objectDesc);
        const ObjectId sceneId = attachedStage.getObjectDatabase()->findEntry(desc.sceneKey, eScene);
        PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(sceneId);

        if (!physxScene || !physxScene->isFullGpuPipelineAvailable())
        {
            PhysXUsdPhysicsInterface::reportLoadError(
                usdparser::ErrorCode::eError,
                "Particles feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!");
            return kInvalidObjectId;
        }

        physxType = ePTParticleSystem;
        outId = createPbdParticleSystem(attachedStage, key, desc);
    }
    break;

    case eParticleSet:
    {
        // Re-entrant-scan guard: same rationale as eParticleSystem above.
        const ObjectId existingParticleSetId = attachedStage.getObjectDatabase()->findEntry(key, eParticleSet);
        if (existingParticleSetId != kInvalidObjectId)
            return existingParticleSetId;

        const ParticleSetDesc& desc = static_cast<const ParticleSetDesc&>(objectDesc);
        const ObjectId sceneId = attachedStage.getObjectDatabase()->findEntry(desc.sceneKey, eScene);
        PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(sceneId);

        if (!physxScene || !physxScene->isFullGpuPipelineAvailable())
        {
            PhysXUsdPhysicsInterface::reportLoadError(
                usdparser::ErrorCode::eError,
                "Particles feature is only supported on GPU. Please enable GPU dynamics flag in Property/Scene of physics scene!");
            return kInvalidObjectId;
        }

        physxType = ePTParticleSet;
        outId = createParticleSet(attachedStage, key, desc);
        // Cleanup-on-failure: removes the applied API so a failed particle set
        // doesn't linger as a schema with no engine object.
        if (outId == kInvalidObjectId)
            removeAppliedAPI(attachedStage, key, "PhysxParticleSetAPI");
    }
    break;
    case eAttachmentVtxVtx:
    case eAttachmentVtxTri:
    case eAttachmentVtxTet:
    case eAttachmentVtxXform:
    case eAttachmentTetXform:
    {
        // No need to check for GPU dynamic flag on the scene because this is already done by the deformable
        const PhysxDeformableAttachmentDesc& desc = static_cast<const PhysxDeformableAttachmentDesc&>(objectDesc);
        physxType = ePTDeformableAttachment;
        outId = createDeformableAttachment(attachedStage, key, desc);
    }
    break;
    case eDeformableCollisionFilter:
    {
        // No need to check for GPU dynamic flag on the scene because this is already done by the deformable
        const PhysxDeformableCollisionFilterDesc& desc = static_cast<const PhysxDeformableCollisionFilterDesc&>(objectDesc);
        physxType = ePTDeformableCollisionFilter;
        outId = createDeformableCollisionFilter(attachedStage, key, desc);
    }
    break;

    case eVehicle:
    {
        const VehicleDesc& desc = static_cast<const VehicleDesc&>(objectDesc);
        physxType = ePTVehicle;
        outId = createVehicle(attachedStage, key, desc);
    }
    break;

    case eVehicleControllerStandard:
    case eVehicleControllerTank:
    {
        const VehicleControllerDesc& desc = static_cast<const VehicleControllerDesc&>(objectDesc);
        physxType = ePTVehicleController;
        outId = createVehicleController(attachedStage, key, desc);
    }
    break;

    case eVehicleEngine:
    {
        physxType = ePTVehicleEngine;
        outId = registerVehicleComponent(key, physxType);
    }
    break;

    case eVehicleTireFrictionTable:
    {
        const TireFrictionTableDesc& desc = static_cast<const TireFrictionTableDesc&>(objectDesc);
        physxType = ePTVehicleTireFrictionTable;
        outId = createTireFrictionTable(desc);
    }
    break;

    case eVehicleSuspension:
    {
        physxType = ePTVehicleSuspension;
        outId = registerVehicleWheelComponent(key, physxType);
    }
    break;

    case eVehicleTire:
    {
        physxType = ePTVehicleTire;
        outId = registerVehicleWheelComponent(key, physxType);
    }
    break;

    case eVehicleWheel:
    {
        physxType = ePTVehicleWheel;
        outId = registerVehicleWheelComponent(key, physxType);
    }
    break;

    case eVehicleWheelAttachment:
    {
        InternalVehicleWheelAttachment* vehicleWheelAttachment = ICE_NEW(InternalVehicleWheelAttachment);
        physxType = ePTVehicleWheelAttachment;
        outId = db.addRecord(physxType, nullptr, vehicleWheelAttachment, key);
    }
    break;

    case eVehicleWheelController:
    {
        const WheelControllerDesc& desc = static_cast<const WheelControllerDesc&>(objectDesc);
        physxType = ePTVehicleWheelController;
        outId = createVehicleWheelController(attachedStage, key, desc);
    }
    break;

    case eVehicleDriveBasic:
    {
        physxType = ePTVehicleDriveBasic;
        outId = registerVehicleComponent(key, physxType);
    }
    break;

    case eDynamicBody:
    {
        const DynamicPhysxRigidBodyDesc& desc = static_cast<const DynamicPhysxRigidBodyDesc&>(objectDesc);
        const ObjectId sceneId = desc.sceneIds.empty() ? kInvalidObjectId : desc.sceneIds[0];
        PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);

        const PxTransform rboTransform = toPhysX(desc.position, desc.rotation);
        if (!rboTransform.isValid())
        {
            CARB_LOG_ERROR("Dynamic body transformation not valid, prim (%s)", attachedStage.textFor(key));
            break;
        }

        PxRigidDynamic* rigidDynamic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidDynamic(rboTransform);
        if(!rigidDynamic)
        {
            CARB_LOG_ERROR("Failed to create rigid dynamic body, prim (%s)", attachedStage.textFor(key));
            break;
        }
        applyRigidDynamicPhysxDesc(physxScene, desc, *rigidDynamic);

        if (attachedStage.isUsingReplicatorEnvIds())
        {
            // Creation-time env-id for a dynamic body: scene-partition primvar id, else 0 (the clone
            // source's env). Only dynamics + articulation aggregates are stamped; statics are left
            // at PX_INVALID_U32 so a shared ground collides with every env (see the static branch).
            // Caveat: a *shared dynamic* object with no primvar also lands in env 0, so it only
            // interacts with env 0 -- a shared dynamic in a cloned scene is unusual and unsupported.
            uint32_t partition_id = 0;
            // Read (and look up) through the TokenId path, not TfToken: replicate()'s
            // registerEnvIdFromToken(TokenId) call (PhysXReplicator.cpp) is the only writer of
            // the scene-partition -> envId map, so a TfToken-keyed lookup here would always miss.
            if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
            {
                const omni::physics::parse::TokenId scenePartitionAttr = source->internToken(kScenePartitionPrimvar);
                omni::physics::parse::TokenId scenePartitionToken;
                if (getValue<omni::physics::parse::TokenId>(attachedStage, key,
                                                             scenePartitionAttr, omni::physics::parse::ReadTime::defaultTime(),
                                                             scenePartitionToken))
                {
                    partition_id = attachedStage.getEnvIdFromToken(scenePartitionToken);
                }
            }
            rigidDynamic->setEnvironmentID(partition_id);
        }

        InternalActor* intActor = setupActor(physxScene, *rigidDynamic, desc, instance, key);

        setupContactReport(physxScene, attachedStage, *rigidDynamic, key);
        physxType = ePTActor;
        outId = db.addRecord(physxType, rigidDynamic, intActor, key);
        rigidDynamic->userData = (void*)(outId);
        if (mExposePrimNames)
            rigidDynamic->setName(attachedStage.textFor(key));

        PxScene* scene = physxScene->getScene();
        scene->addActor(*rigidDynamic);

        PxU32 posIters, velIters;
        rigidDynamic->getSolverIterationCounts(posIters, velIters);
        if (scene->getSolverType() == PxSolverType::eTGS && velIters > 4)
        {
            CARB_LOG_WARN_ONCE(
                "Detected a rigid at %s with more than 4 velocity iterations being added to a TGS scene."
                "The related behavior changed recently, please consult the changelog. This warning will only print once.",
                attachedStage.textFor(key));
        }

        if (desc.startsAsleep)
        {
            rigidDynamic->putToSleep();
        }

        // handle mirrors
        if (desc.sceneIds.size() > 1 && intActor)
        {
            void* mirrorMemBlock = nullptr;
            uint32_t memSize = 0;
            PxCollection* sharedCollection = nullptr;
            mirrorActor(*rigidDynamic, mirrorMemBlock, memSize, *physxSetup.getSerializationRegistry(), sharedCollection);
            CARB_ASSERT(sharedCollection);
            intActor->mMirrorSharedCollection = sharedCollection;
            intActor->mMirrorMemsize = memSize;
            intActor->mMirrorMemory = mirrorMemBlock;

            physxScene->getInternalScene()->mMirorredActors.push_back(intActor);

            for (size_t i = 1; i < desc.sceneIds.size(); i++)
            {
                void* nm = copyAlignedMemory(mirrorMemBlock, memSize);
                PhysXScene* mirrorScene = physxSetup.getPhysXScene(desc.sceneIds[i]);
                PxCollection* col = nullptr;
                PxRigidDynamic* dynamicBody = static_cast<PxRigidDynamic*>(instantiateMirrorActor(nm, *physxSetup.getSerializationRegistry(),
                    *mirrorScene->getScene(), col, sharedCollection));
                // mirrored body is always kinematic
                dynamicBody->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
                intActor->mMirrors.push_back({ nm, col, dynamicBody, mirrorScene->getInternalScene() });
            }
        }
    }
    break;

    case eStaticBody:
    {
        const StaticPhysxRigidBodyDesc& desc = static_cast<const StaticPhysxRigidBodyDesc&>(objectDesc);
        const ObjectId sceneId = desc.sceneIds.empty() ? kInvalidObjectId : desc.sceneIds[0];
        PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);

        const PxTransform rboTransform = toPhysX(desc.position, desc.rotation);
        if (!rboTransform.isValid())
        {
            CARB_LOG_ERROR("Static body transformation not valid, prim (%s)", attachedStage.textFor(key));
            break;
        }

        PxRigidStatic* rigidStatic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidStatic(rboTransform);
        if (!rigidStatic)
        {
            CARB_LOG_ERROR("Failed to create rigid static body, prim (%s)", attachedStage.textFor(key));
            break;
        }

        // Creation-time replicator env-ids are intentionally NOT assigned to static bodies. Statics
        // never collide with each other, so an env-id on a static only affects static<->dynamic
        // pairs. Leaving a shared static (e.g. a ground plane parsed alongside the clone source) at
        // PX_INVALID_U32 keeps it colliding with EVERY environment's dynamics, so cloned envs keep
        // resting on it -- stamping it env id 0 would make clones (env 1..N) fall through. Only
        // dynamics (above) and articulation aggregates (below) carry env-ids. Residual: co-located
        // clones with per-env static obstacles are not isolated from each other (those statics
        // overlap and each collides with all envs' dynamics); use explicit transforms for that layout.

        InternalActor* intActor = setupActor(physxScene, *rigidStatic, desc, instance, key);
        setupContactReport(physxScene, attachedStage, *rigidStatic, key);
        physxType = ePTActor;
        outId = db.addRecord(physxType, rigidStatic, intActor, key);
        rigidStatic->userData = (void*)(outId);
        if (mExposePrimNames)
            rigidStatic->setName(attachedStage.textFor(key));

        PxScene* scene = physxScene->getScene();
        scene->addActor(*rigidStatic);

        intActor->mSourceGPrimKey = desc.sourceGPrimKey;

        // handle mirrors
        if (desc.sceneIds.size() > 1)
        {
            void* mirrorMemBlock = nullptr;
            uint32_t memSize = 0;
            PxCollection* sharedCollection = nullptr;
            mirrorActor(*rigidStatic, mirrorMemBlock, memSize, *physxSetup.getSerializationRegistry(), sharedCollection);
            CARB_ASSERT(sharedCollection);
            intActor->mMirrorSharedCollection = sharedCollection;
            intActor->mMirrorMemsize = memSize;
            intActor->mMirrorMemory = mirrorMemBlock;

            for (size_t i = 1; i < desc.sceneIds.size(); i++)
            {
                void* nm = copyAlignedMemory(mirrorMemBlock, memSize);                
                PhysXScene* mirrorScene = physxSetup.getPhysXScene(desc.sceneIds[i]);
                PxCollection* col = nullptr;
                PxRigidStatic* staticBody = static_cast<PxRigidStatic *>(instantiateMirrorActor(nm, *physxSetup.getSerializationRegistry(),
                    *mirrorScene->getScene(), col, sharedCollection));
                intActor->mMirrors.push_back({ nm, col, staticBody, mirrorScene->getInternalScene() });
            }
        }
    }
    break;
    case eVolumeDeformableBody:
    {
        const PhysxVolumeDeformableBodyDesc& desc = static_cast<const PhysxVolumeDeformableBodyDesc&>(objectDesc);
        outId = createVolumeDeformableBody(attachedStage, key, desc);
        physxType = ePTDeformableVolume;
    }
    break;
    case eSurfaceDeformableBody:
    {
        const PhysxSurfaceDeformableBodyDesc& desc = static_cast<const PhysxSurfaceDeformableBodyDesc&>(objectDesc);
        outId = createSurfaceDeformableBody(attachedStage, key, desc);
        physxType = ePTDeformableSurface;
    }
    break;
    case ePhysxForce:
    {
        const PhysxForceDesc& desc = static_cast<const PhysxForceDesc&>(objectDesc);
        if (desc.body != kInvalidObjectId && desc.body < db.getRecords().size())
        {
            InternalDatabase::Record& bodyRecord = db.getRecords()[(size_t)desc.body];
            if (bodyRecord.mType == ePTActor || bodyRecord.mType == ePTLink)
            {
                PxRigidActor* rbo = reinterpret_cast<::physx::PxRigidActor*>(bodyRecord.mPtr);
                InternalForce* intForce = ICE_NEW(InternalForce);

                intForce->mAccelerationMode = desc.accelerationMode;
                intForce->mCoMApplied = bodyRecord.mKey == key ? true : false;
                intForce->mEnabled = desc.enabled;
                intForce->setForce(toPhysX(desc.force));
                intForce->setTorque(toPhysX(desc.torque));
                intForce->mWorldFrame = desc.worldFrame;
                intForce->mRigidActor = rbo;
                intForce->mLocalRot = toPhysXQuat(desc.localRot);

                intForce->mBodyPrimDifferent = bodyRecord.mKey == key ? false : true;

                if (!intForce->mCoMApplied)
                {
                    intForce->mLocalPos = intForce->mRigidActor->getGlobalPose().transformInv(toPhysX(desc.worldPos));
                }
                else
                {
                    intForce->mLocalPos = PxVec3(0.0f, 0.0f, 0.0f);
                }

                PhysXScene* physxScene = physxSetup.getPhysXScene(desc.scene);
                physxScene->getInternalScene()->mForces.push_back(intForce);
                intForce->mPhysXScene = physxScene;

                physxType = ePTForce;
                outId = db.addRecord(physxType, nullptr, intForce, key);
            }
            else
            {
                CARB_LOG_INFO("PhysX Force cant belong to a static body, make sure its applied to a body or a child prim of a body. (force prim: %s)", attachedStage.textFor(key));
            }
        }
        else
        {
            CARB_LOG_WARN("PhysX Force does not belong to any body, make sure its applied to a body or a child prim of a body. (force prim: %s)", attachedStage.textFor(key));
        }
    }
    break;
    case eArticulationLink:
    {
        const PhysxArticulationLinkDesc& desc = static_cast<const PhysxArticulationLinkDesc&>(objectDesc);
        const ObjectId sceneId = desc.sceneIds.empty() ? kInvalidObjectId : desc.sceneIds[0];
        // A.B. add validity checks if all links belong to the same scene
        PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);

        PxArticulationReducedCoordinate* articulation =
            getPtr<PxArticulationReducedCoordinate>(ePTArticulation, desc.articulation);

        if (!articulation)
        {
            CARB_LOG_WARN("Articulation not found for link creation, prim (%s)", attachedStage.textFor(key));
            break;
        }

        PxArticulationLink* parent = getPtr<PxArticulationLink>(ePTLink, desc.parent);
        if (!gTempParent.empty())
        {
            for (size_t i = 0; i < gTempParent.size(); i++)
            {
                if (desc.parent == gTempParent[i].first)
                {
                    parent = gTempParent[i].second;
                    break;
                }
            }
        }

        const PxTransform rboTransform = toPhysX(desc.position, desc.rotation);
        if (!rboTransform.isValid())
        {
            CARB_LOG_WARN("Articulation link transformation not valid, prim (%s)", attachedStage.textFor(key));
            break;
        }

        PxArticulationLink* link = articulation->createLink(parent, rboTransform);

        setupContactReport(physxScene, attachedStage, *link, key);

        InternalLink* internalActor = nullptr;
        InternalJoint* intJoint = nullptr;

        if (link)
        {
            internalActor = (InternalLink*)setupActor(physxScene, *link, desc, instance, key);

            physxType = ePTLink;
            outId = db.addRecord(physxType, link, internalActor, key);

            link->setCfmScale(desc.cfmScale);

            PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();

            if (joint && desc.articulationJointType == eStandardJoint && desc.articulationJoint) // Will be null for root link
            {
                // Must be `key`: mBody0IsParentLink is derived from this identity, and
                // getting it wrong silently mirrors local poses and sign-flips limits,
                // drive targets, joint states and mimic offsets.
                createArticulationJoint<true>(attachedStage, *desc.articulationJoint, joint, key, internalActor);
            }

            if (link)
            {
                applyPhysXRigidBodyDesc(*link, desc);
                // can only set the velocities on the root link:
                if(!parent)
                {
                    articulation->setRootLinearVelocity(toPhysX(desc.linearVelocity));
                    articulation->setRootAngularVelocity(toPhysX(desc.angularVelocity));
                }
            }
        }

        if (link)
        {
            link->userData = (void*)(outId);
            if (mExposePrimNames)
                link->setName(attachedStage.textFor(key));
        }
    }
    break;

    case eArticulation:
    {
        gTempParent.clear();
        mDirty = true;
        const PhysxArticulationDesc& desc = static_cast<const PhysxArticulationDesc&>(objectDesc);

        PhysXScene* physxScene = physxSetup.getPhysXScene(desc.sceneId);

        PxArticulationReducedCoordinate* articulation = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createArticulationReducedCoordinate();
        if (mExposePrimNames)
            articulation->setName(attachedStage.textFor(key));

        physxScene->getInternalScene()->mArticulations.push_back(articulation);

        articulation->setSleepThreshold(desc.sleepThreshold);
        articulation->setStabilizationThreshold(desc.stabilizationThreshold);

        articulation->setSolverIterationCounts(
            physxScene->getInternalScene()->clampPosIterationCount(desc.solverPositionIterationCount),
            physxScene->getInternalScene()->clampVelIterationCount(desc.solverVelocityIterationCount));

        if (desc.fixBase)
            articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
        articulation->setArticulationFlag(PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES, true);

        InternalArticulation* intArt = ICE_NEW(InternalArticulation)(physxScene);
        intArt->mEnableSelfCollision = desc.selfCollision;
        intArt->mStaticRootBodyKey = desc.staticRootBodyPrim;

        physxType = ePTArticulation;
        outId = db.addRecord(physxType, articulation, intArt, key);
        mArticulations.push_back(outId);
        articulation->userData = (void*)(outId);

        if (desc.fixBaseKey.valid())
        {
            // fixed joints are converted into fixed articulation, we need the pointer to that.
            physxType = ePTArticulationFixedBase;
            const ObjectId outIdNew = db.addRecord(physxType, articulation, nullptr, desc.fixBaseKey);
            CARB_ASSERT(outId + 1 == outIdNew);
        }
    }
    break;

    case eTendonFixed:
    {
        const PhysxTendonFixedDesc& desc = static_cast<const PhysxTendonFixedDesc&>(objectDesc);

        // Find parent joint of tendon root joint (needed for dummy joint)
        ObjectId parentLinkJointId = attachedStage.getObjectDatabase()->findEntry(key, eArticulationJoint);

        if (parentLinkJointId == kInvalidObjectId)
        {
            CARB_LOG_WARN("Could not parse tendon at %s because its root-axis joint is not part of any articulation.", attachedStage.textFor(key));
            return kInvalidObjectId;
        }

        const PxArticulationJointReducedCoordinate* parentLinkJoint = getPtr<PxArticulationJointReducedCoordinate>(ePTLinkJoint, parentLinkJointId);
        if (!parentLinkJoint)
        {
            CARB_LOG_WARN("Could not parse tendon at %s because its root-axis joint is not available.", attachedStage.textFor(key));
            return kInvalidObjectId;
        }
        PxArticulationLink& parentLink = parentLinkJoint->getParentArticulationLink();
        PxArticulationReducedCoordinate& articulation = static_cast<PxArticulationReducedCoordinate&>(parentLink.getArticulation());

        // create tendon
        PxArticulationFixedTendon* tendon = articulation.createFixedTendon();

        // set stiffnesses & damping to zero if tendon is disabled
        if (desc.isEnabled)
        {
            tendon->setStiffness(desc.stiffness);
            tendon->setDamping(desc.damping);
            tendon->setLimitStiffness(desc.limitStiffness);
        }
        else
        {
            tendon->setStiffness(0.f);
            tendon->setLimitStiffness(0.f);
            tendon->setDamping(0.f);
        }
        tendon->setOffset(desc.offset);
        tendon->setRestLength(desc.restLength);
        PxArticulationTendonLimit para;
        para.lowLimit = desc.lowLimit;
        para.highLimit = desc.highLimit;
        tendon->setLimitParameters(para);

        // create dummy root joint
        PxArticulationTendonJoint* tJoint = tendon->createTendonJoint(nullptr, PxArticulationAxis::eTWIST, 0.f, 0.f, &parentLink);
        InternalTendonAxis* internalTJoint = ICE_NEW(InternalTendonAxis);
        internalTJoint->instanceName = tendonInstanceNameFor(attachedStage, desc.instanceToken);

        physxType = ePTFixedTendonAxis;
        outId =
            ObjectId(size_t(db.addRecord(physxType, tJoint, internalTJoint, key)));
        tJoint->userData = (void*)outId;
    }
    break;

    case eTendonAxis:
    {
        const PhysxTendonAxisDesc& desc = static_cast<const PhysxTendonAxisDesc&>(objectDesc);

        // Get link joint corresponding to tendon axis
        ObjectId linkJointId = attachedStage.getObjectDatabase()->findEntry(key, eArticulationJoint);

        if (linkJointId == kInvalidObjectId)
        {
            CARB_LOG_WARN("Could not parse tendon axis at %s because its joint is not part of any articulation.", attachedStage.textFor(key));
            return kInvalidObjectId;
        }

        PxArticulationTendonJoint* parentTendonJoint = nullptr;

        const PxArticulationJointReducedCoordinate* linkJoint = getPtr<PxArticulationJointReducedCoordinate>(ePTLinkJoint, linkJointId);
        if (!linkJoint)
        {
            CARB_LOG_WARN("Could not get tendon axis joint for tendon %s.", attachedStage.textFor(key));
            return kInvalidObjectId;
        }
        PxArticulationLink& parentLink = linkJoint->getParentArticulationLink();

        CARB_ASSERT(desc.parentAxisId != kInvalidObjectId);

        // get parent axis information and checking for topology correctness
        const InternalDatabase::Record& rec = db.getRecords()[size_t(desc.parentAxisId)];
        if (rec.mType == ePTFixedTendonAxis)
        {
            parentTendonJoint = static_cast<PxArticulationTendonJoint*>(rec.mPtr);
        }

        CARB_ASSERT(parentTendonJoint);
        if (!parentTendonJoint)
        {
            CARB_LOG_WARN("Could not get parent tendon joint at %s.", attachedStage.textFor(key));
            return kInvalidObjectId;
        }

        if (&parentLink != parentTendonJoint->getLink())
        {
            CARB_LOG_WARN("Could not parse tendon axis at %s due to a topology issue: Refer to the topology constraints in the USD schema doc for PhysxTendonAxisAPI.", attachedStage.textFor(key));
            return kInvalidObjectId;
        }

        // we only support prismatic and revolute joints at the moment
        PxArticulationJointType::Enum jointType = linkJoint->getJointType();
        PxArticulationAxis::Enum axis;
        switch (jointType)
        {
        case PxArticulationJointType::ePRISMATIC:
            axis = PxArticulationAxis::eX;
            break;
        case PxArticulationJointType::eREVOLUTE:
        case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            axis = PxArticulationAxis::eTWIST;
            break;
        default:
            return kInvalidObjectId;
        }

        // create tendon axis
        PxArticulationTendonJoint* tJoint = parentTendonJoint->getTendon()->createTendonJoint(parentTendonJoint, axis, desc.gearings[0], desc.forceCoefficients[0], &linkJoint->getChildArticulationLink());
        InternalTendonAxis* internalTJoint = ICE_NEW(InternalTendonAxis);
        internalTJoint->instanceName = tendonInstanceNameFor(attachedStage, desc.instanceToken);

        physxType = ePTFixedTendonAxis;
        outId =
            ObjectId(size_t(db.addRecord(physxType, tJoint, internalTJoint, key)));
        tJoint->userData = (void*)outId;
    }
    break;

    case eTendonAttachmentRoot:
    {
        const PhysxTendonSpatialDesc& desc = static_cast<const PhysxTendonSpatialDesc&>(objectDesc);

        // find articulation link that root attachment should attach to
        const ObjectId linkId = attachedStage.getObjectDatabase()->findEntry(key, eArticulationLink);

        if (linkId == kInvalidObjectId)
        {
            CARB_LOG_WARN("Could not parse tendon with root attachment %s because its link %s is not part of any articulation.", tokenText(attachedStage, desc.instanceToken).c_str(), attachedStage.textFor(key));
            return kInvalidObjectId;
        }

        // create spatial tendon
        PxArticulationLink* link = getPtr<PxArticulationLink>(ePTLink, linkId);
        PxArticulationReducedCoordinate& articulation = static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
        PxArticulationSpatialTendon* tendon = articulation.createSpatialTendon();

        if (desc.isEnabled)
        {
            tendon->setStiffness(desc.stiffness);
            tendon->setLimitStiffness(desc.limitStiffness);
            tendon->setDamping(desc.damping);
        }
        else
        {
            tendon->setStiffness(0.f);
            tendon->setLimitStiffness(0.f);
            tendon->setDamping(0.f);
        }
        tendon->setOffset(desc.offset);

        // create root attachment and internal data structure
        PxTransform linkTrans = link->getGlobalPose();
        PxArticulationAttachment* attachment = tendon->createAttachment(nullptr, 1.f, toPhysX(desc.localPos), link); //ignore gearing attribute
        InternalTendonAttachment* internalAttachment = ICE_NEW(InternalTendonAttachment);
        internalAttachment->globalPos = linkTrans.p + linkTrans.rotate(toPhysX(desc.localPos));
        internalAttachment->initLength = 0.f;
        internalAttachment->instanceName = tendonInstanceNameFor(attachedStage, desc.instanceToken);

        physxType = ePTTendonAttachment;
        outId =
            ObjectId(size_t(db.addRecord(physxType, attachment, internalAttachment, key)));
        attachment->userData = (void*)outId;
    }
    break;

    case eTendonAttachmentLeaf:
    case eTendonAttachment:
    {
        const PhysxTendonAttachmentDesc& desc = static_cast<const PhysxTendonAttachmentDesc&>(objectDesc);

        // find corresponding link that attachment belongs to
        const ObjectId linkId = attachedStage.getObjectDatabase()->findEntry(key, eArticulationLink);

        if (linkId == kInvalidObjectId)
        {
            CARB_LOG_WARN("Could not parse attachment %s because its link %s is not part of any articulation.", tokenText(attachedStage, desc.instanceToken).c_str(), attachedStage.textFor(key));
            return kInvalidObjectId;
        }

        PxArticulationAttachment* parentAttachment = nullptr;
        InternalTendonAttachment* parentIntAttachment = nullptr;

        // get parent information & ensure consistent topology
        const InternalDatabase::Record& rec = db.getRecords()[size_t(desc.parentId)];
        if (rec.mType == ePTTendonAttachment)
        {
            parentIntAttachment = static_cast<InternalTendonAttachment*>(rec.mInternalPtr);
            parentAttachment = static_cast<PxArticulationAttachment*>(rec.mPtr);
        }

        CARB_ASSERT(parentAttachment && parentIntAttachment);
        if (!parentAttachment)
        {
            CARB_LOG_WARN("Could not get parent attachment at %s.", attachedStage.textFor(key));
            return kInvalidObjectId;
        }

        PxArticulationLink* link = getPtr<PxArticulationLink>(ePTLink, linkId);
        if (parentAttachment->getTendon()->getArticulation() != &link->getArticulation())
        {
            CARB_LOG_WARN("Could not parse attachment %s at %s because its parent attachment %s at %s is not part of the same articulation.",
                          tokenText(attachedStage, desc.instanceToken).c_str(), attachedStage.textFor(key), tokenText(attachedStage, desc.parentToken).c_str(), attachedStage.textFor(desc.parentKey));
            return kInvalidObjectId;
        }

        // create attachment and keep track of global length metrics at initialization
        InternalTendonAttachment* internalAttachment = ICE_NEW(InternalTendonAttachment);
        PxTransform linkTrans = link->getGlobalPose();
        PxArticulationAttachment* attachment = parentAttachment->getTendon()->createAttachment(parentAttachment, desc.gearing, toPhysX(desc.localPos), link);
        internalAttachment->globalPos = linkTrans.p + linkTrans.rotate(toPhysX(desc.localPos));
        internalAttachment->initLength = parentIntAttachment->initLength +
            (internalAttachment->globalPos - parentIntAttachment->globalPos).magnitude() * desc.gearing;
        internalAttachment->instanceName = tendonInstanceNameFor(attachedStage, desc.instanceToken);

        // set leaf parameters if needed
        if (desc.type == eTendonAttachmentLeaf)
        {
            const PhysxTendonAttachmentLeafDesc& leafDesc = static_cast<const PhysxTendonAttachmentLeafDesc&>(desc);
            if (leafDesc.restLength < 0.f)
            {
                attachment->setRestLength(internalAttachment->initLength);
            }
            else
            {
                attachment->setRestLength(leafDesc.restLength);
            }
            PxArticulationTendonLimit limits;
            limits.lowLimit = leafDesc.lowLimit;
            limits.highLimit = leafDesc.highLimit;
            attachment->setLimitParameters(limits);
        }

        physxType = ePTTendonAttachment;
        outId =
            ObjectId(size_t(db.addRecord(physxType, attachment, internalAttachment, key)));
        attachment->userData = (void*)outId;
    }
    break;

    case eFilteredPair:
    {
        const FilteredPairDesc& desc = static_cast<const FilteredPairDesc&>(objectDesc);
        physxType = ePTFilteredPair;
        InternalFilteredPairs* intPairs = ICE_NEW(InternalFilteredPairs);
        intPairs->mPairs = desc.pairs;
        intPairs->createFilteredPairs();
        outId = db.addRecord(physxType, nullptr, intPairs, key);
    }
    break;

    case eCollisionGroup:
    {
        physxType = ePTCollisionGroup;
        outId = db.addRecord(physxType, nullptr, nullptr, key);
        PxFilterData* fd = new PxFilterData();
        const uint32_t index = convertToCollisionGroup(outId);
        convertCollisionGroupToPxFilterData(index, *fd);
        db.getRecords()[index].mPtr = fd;
    }
    break;

    case eCapsuleCct:
    {
        const CapsuleCctDesc& desc = static_cast<const CapsuleCctDesc&>(objectDesc);

        PhysXScene* physxScene = physxSetup.getPhysXScene(desc.sceneId);

        PxCapsuleControllerDesc cctDesc = parsePhysXCharacterControllerDesc(attachedStage, key, desc.radius, desc.height * 2.0f);

        cctDesc.slopeLimit = desc.slopeLimit;
        cctDesc.position = PxExtendedVec3(desc.pos.x, desc.pos.y, desc.pos.z);
        cctDesc.material = physxScene->getDefaultMaterial();

        PxCapsuleController* ctrl = static_cast<PxCapsuleController*>(physxScene->getControllerManager()->createController(cctDesc));
        if (!ctrl) {
            CARB_LOG_WARN("Could not parse capsule cct at %s : failed to create a PxCapsuleController.", attachedStage.textFor(key));
            return kInvalidObjectId;
        }

        // remove controller shape from scene query for standup overlap test
        PxRigidDynamic* actor = ctrl->getActor();
        if (actor->getNbShapes()) {
            PxShape* ctrlShape;
            actor->getShapes(&ctrlShape, 1);
            ctrlShape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
        }

        const PxQuat fixupQ = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), cctDesc.upDirection);

        InternalCct* intCct = ICE_NEW(InternalCct)(physxScene, nullptr, key);
        intCct->mScale = desc.scale;
        intCct->mActor = ctrl->getActor();
        intCct->mFixupQ = fixupQ.getConjugate();

        physxScene->getInternalScene()->mCctMap[key] = intCct;

        physxType = ePTCct;
        outId = db.addRecord(physxType, ctrl, intCct, key);
        ctrl->getActor()->userData = (void*)(outId);
    }
    break;

    case eMimicJointRotX:
    case eMimicJointRotY:
    case eMimicJointRotZ:
    case eNewtonMimicJoint:
    {
        const MimicJointDesc& desc = static_cast<const MimicJointDesc&>(objectDesc);
        physxType = ePTMimicJoint;
        outId = createMimicJoint(desc);
    }
    break;

    };

    sendObjectCreationNotification(key, outId, physxType);

    return outId;
}

// ObjectKey-native entry point: resolves the active attach directly (no
// UsdStageWeakPtr/stage-cache round trip needed).
ObjectId PhysXUsdPhysicsInterface::createShape(omni::physics::parse::ObjectKey key,
                                               const PhysxObjectDesc& objectDesc,
                                               usdparser::ObjectId bodyId,
                                               const ObjectInstance* instance)
{
    checkScenes();
    const PhysxShapeDesc& shapeDesc = static_cast<const PhysxShapeDesc&>(objectDesc);
    const ObjectId sceneId = shapeDesc.sceneIds.empty() ? kInvalidObjectId : shapeDesc.sceneIds[0];

    // scristiano: this is temporary code to allow physics inspector. we should filter simulation
    // owners at parsing stage. Kit-inspector-only debug filter, opt-in via an explicit Kit
    // setting; runs under any backend.
    if (!mForceParseOnlySingleScenePath.empty())
    {
        if (sceneId != mForceParseOnlySingleSceneObjectId)
            return kInvalidObjectId;
    }
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);
    InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    usdparser::AttachedStage* activeAttachedStage = usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
    PhysXType physxType = ePTRemoved;
    // create shape
    ObjectId outId = createShapeOrComputeMass(
        key, shapeDesc, bodyId, instance, activeAttachedStage, physxScene, mExposePrimNames, physxType, &db, nullptr);
    if (physxType != ePTRemoved)
    {
        sendObjectCreationNotification(key, outId, physxType);

        // clear shape removed event flag for attachments and collision filters
        InternalActor* intActor = getInternalPtr<InternalActor>(ePTActor, bodyId);
        updateDeformableAttachmentShapeEvents(intActor, outId, key, internal::DirtyEventType::eShapeAdded);
        updateDeformableCollisionFilterShapeEvents(intActor, outId, key, internal::DirtyEventType::eShapeAdded);
    }
    return outId;
}

void cleanupMultipleMaterials(const PhysxShapeDesc& desc,
                              const char* approxName,
                              const usdparser::AttachedStage* attachedStage,
                              omni::physics::parse::ObjectKey key)
{
    if (desc.materials.size() > 1)
    {
        bool materialsOk = true;
        const ObjectId matIndex = desc.materials[0];
        for (size_t k = 1; k < desc.materials.size(); k++)
        {
            if (matIndex != desc.materials[k])
            {
                materialsOk = false;
                break;
            }
        }

        if (materialsOk)
        {
            PhysxShapeDesc& descWr = const_cast<PhysxShapeDesc&>(desc);
            descWr.materials.clear();
            descWr.materials.push_back(matIndex);
        }
        else
        {
            CARB_LOG_ERROR(
                "Triangle mesh %s collision does not support multiple materials, default material will be used - prim: %s!",
                approxName, attachedStage ? attachedStage->textFor(key) : "");
            PhysxShapeDesc& descWr = const_cast<PhysxShapeDesc&>(desc);
            descWr.materials.clear();
        }
    }
}

// ObjectKey-native entry point (the source-agnostic identity boundary): no
// UsdStageWeakPtr/UsdPrim needed internally -- every shape-type branch below
// dispatches off `objectDesc`'s own fields (meshPrimKey etc., resolved through
// `attachedStage`), and the removed `stage`/UsdPrim were passthrough-only
// (diagnostics, and the now ObjectKey-native createShape/cleanupMultipleMaterials
// free functions).
ObjectId PhysXUsdPhysicsInterface::createShapeOrComputeMass(omni::physics::parse::ObjectKey key,
                                                            const PhysxShapeDesc& objectDesc,
                                                            usdparser::ObjectId bodyId,
                                                            const ObjectInstance* instance,
                                                            usdparser::AttachedStage* createShapeAttachedStage,
                                                            PhysXScene* physxScene,
                                                            bool exposePrimNames,
                                                            PhysXType& physxType,
                                                            InternalPhysXDatabase* db,
                                                            PhysXUsdPhysicsInterface::MassInformation* massInfoOut)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();

    CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();
    CARB_ASSERT(cookingDataAsync);

    // Diagnostic-only path text (never null-derefs createShapeAttachedStage,
    // which can be null in the same edge case the old stage-cache lookup left
    // unresolved -- see the SdfPath overload above).
    const char* const pathText = createShapeAttachedStage ? createShapeAttachedStage->textFor(key) : "";

    PxRigidActor* rigidActor = nullptr;
    if (bodyId != kInvalidObjectId)
    {
        rigidActor = getPtr<PxRigidActor>(ePTActor, bodyId);
        if (!rigidActor)
        {
            rigidActor = getPtr<PxRigidActor>(ePTLink, bodyId);
        }
    }

    ObjectId outId = kInvalidObjectId;
    physxType = ePTRemoved;

    switch (objectDesc.type)
    {
    default:
    {
        CARB_ASSERT(!"Unsupported type");
        return kInvalidObjectId;
    }
    break;
    case eSpherePointsShape:
        {
            const SpherePointsPhysxShapeDesc &desc = static_cast<const SpherePointsPhysxShapeDesc&>(objectDesc);
            const SpherePointsPhysxShapeDesc *sdesc = &desc;
            if ( desc.spheres.empty() )
            {
                const SpherePointsPhysxShapeDesc *spp = createShapeAttachedStage ? cookingDataAsync->getSpherePoints(desc, desc.meshPrimKey, *createShapeAttachedStage, false) : nullptr;
                if ( spp )
                {
                    sdesc = spp;
                }
            }
            const carb::Float3& meshScale = desc.meshScale;
            // A.B. we can really support only uniform scale
            const float maxScale = PxMax(meshScale.x, PxMax(meshScale.y, meshScale.z));
            const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
            PxU32 scount = uint32_t(sdesc->spheres.size());
            PxMassProperties* massProps = new PxMassProperties[scount];
            PxTransform* transforms = new PxTransform[scount];
            PhysXUsdPhysicsInterface::MassInformation* massInfos = new PhysXUsdPhysicsInterface::MassInformation[scount];
            physxType = ePTCompoundShape;
            CompoundShape *shape = nullptr;
            if (db)
            {
                shape = new CompoundShape(physxScene);
                InternalShape* intShape = ICE_NEW(InternalShape)(physxScene, desc.localScale);
                if (desc.materials.size() == 1 && desc.materials[0] != kInvalidObjectId)
                {
                    InternalDatabase::Record& materialRecord = db->getRecords()[(size_t)desc.materials[0]];
                    InternalMaterial* intMat = (InternalMaterial*)materialRecord.mInternalPtr;
                    intMat->addShapeId(db->getRecords().size());
                    shape->mMaterialId = (size_t)desc.materials[0];
                }
                outId = db->addRecord(physxType, shape, intShape, key);
            }

            PxBounds3 sumAabbLocalBounds;
            sumAabbLocalBounds.setEmpty();
            for (PxU32 c = 0; c < scount; c++)
            {
                PxTransform poseOffset(PxIdentity);
                poseOffset.p.x = sdesc->spheres[c].position.x * maxScale;
                poseOffset.p.y = sdesc->spheres[c].position.y * maxScale;
                poseOffset.p.z = sdesc->spheres[c].position.z * maxScale;
                const float radius = sdesc->spheres[c].radius * maxScale;                
                PxBounds3 aabbLocalBounds;
                PxGeometryQuery::computeGeomBounds(aabbLocalBounds, PxSphereGeometry(radius), poseOffset);
                sumAabbLocalBounds.include(aabbLocalBounds);
            }

            for (PxU32 c = 0; c < scount; c++)
            {
                PhysXUsdPhysicsInterface::MassInformation& massInfo = massInfos[c];
                PxTransform poseOffset(PxIdentity);
                poseOffset.p.x = sdesc->spheres[c].position.x * maxScale;
                poseOffset.p.y = sdesc->spheres[c].position.y * maxScale;
                poseOffset.p.z = sdesc->spheres[c].position.z * maxScale;
                const float radius = sdesc->spheres[c].radius * maxScale;

                omni::physx::createShape(PxSphereGeometry(radius), desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames,
                    &poseOffset, &outId, &massInfo, instance, &sumAabbLocalBounds);
                massProps[c].centerOfMass = { float(massInfo.centerOfMass.x+poseOffset.p.x), float(massInfo.centerOfMass.y+poseOffset.p.y), float(massInfo.centerOfMass.z+poseOffset.p.z) };
                massProps[c].mass = float(massInfo.volume) * maxScale;

                massProps[c].inertiaTensor.column0[0] = float(massInfo.inertia[0]) * maxScale;
                massProps[c].inertiaTensor.column0[1] = float(massInfo.inertia[1]) * maxScale;
                massProps[c].inertiaTensor.column0[2] = float(massInfo.inertia[2]) * maxScale;

                massProps[c].inertiaTensor.column1[0] = float(massInfo.inertia[3]) * maxScale;
                massProps[c].inertiaTensor.column1[1] = float(massInfo.inertia[4]) * maxScale;
                massProps[c].inertiaTensor.column1[2] = float(massInfo.inertia[5]) * maxScale;

                massProps[c].inertiaTensor.column2[0] = float(massInfo.inertia[6]) * maxScale;
                massProps[c].inertiaTensor.column2[1] = float(massInfo.inertia[7]) * maxScale;
                massProps[c].inertiaTensor.column2[2] = float(massInfo.inertia[8]) * maxScale;

                transforms[c] = PxTransform(PxIdentity);
            }
            PxMassProperties outMassProps = PxMassProperties::sum(massProps, transforms, scount);

            delete[] massProps;
            delete[] transforms;

            PhysXUsdPhysicsInterface::MassInformation massInfo;
            const PxMat33 inertiaMatrix = (outMassProps.inertiaTensor);

            massInfo.inertia[0] = inertiaMatrix.column0[0];
            massInfo.inertia[1] = inertiaMatrix.column0[1];
            massInfo.inertia[2] = inertiaMatrix.column0[2];

            massInfo.inertia[3] = inertiaMatrix.column1[0];
            massInfo.inertia[4] = inertiaMatrix.column1[1];
            massInfo.inertia[5] = inertiaMatrix.column1[2];

            massInfo.inertia[6] = inertiaMatrix.column2[0];
            massInfo.inertia[7] = inertiaMatrix.column2[1];
            massInfo.inertia[8] = inertiaMatrix.column2[2];

            massInfo.volume = outMassProps.mass; // density is 1 so this if fine.
            massInfo.centerOfMass = { outMassProps.centerOfMass.x, outMassProps.centerOfMass.y, outMassProps.centerOfMass.z };
            massInfo.localPos = desc.localPos;
            massInfo.localRot = desc.localRot;
            massInfo.aabbLocalMax = fromPhysX(sumAabbLocalBounds.maximum);
            massInfo.aabbLocalMin = fromPhysX(sumAabbLocalBounds.minimum);
            delete[] massInfos;
            if (shape)
            {
                shape->mMassInfo = massInfo;
            }
            if (massInfoOut)
            {
                *massInfoOut = massInfo;
            }

        }
        break;
    case eSphereShape:
    {
        const SpherePhysxShapeDesc& desc = static_cast<const SpherePhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
        physxType = ePTShape;
        outId = omni::physx::createShape(
            PxSphereGeometry(desc.radius), desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames, nullptr, nullptr, massInfoOut, instance);
    }
    break;

    case eBoxShape:
    {
        const BoxPhysxShapeDesc& desc = static_cast<const BoxPhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
        physxType = ePTShape;
        outId = omni::physx::createShape(PxBoxGeometry(toPhysX(desc.halfExtents)), desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames,
                                          nullptr, nullptr, massInfoOut, instance);
    }
    break;
    case eBoundingSphereShape:
    {
        const BoundingSpherePhysxShapeDesc& desc = static_cast<const BoundingSpherePhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
        const PxTransform transform(toPhysX(desc.positionOffset));
        physxType = ePTShape;
        cleanupMultipleMaterials(desc, "BoundingSphere", createShapeAttachedStage, key);
        outId = omni::physx::createShape(
            PxSphereGeometry(desc.radius), desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames, &transform, nullptr, massInfoOut, instance);
    }
    break;

    case eBoundingBoxShape:
    {
        const BoundingBoxPhysxShapeDesc& desc = static_cast<const BoundingBoxPhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
        const PxTransform transform(toPhysX(desc.positionOffset), toPhysXQuat(desc.rotationOffset));
        physxType = ePTShape;
        cleanupMultipleMaterials(desc, "BoundingBox", createShapeAttachedStage, key);
        outId = omni::physx::createShape(PxBoxGeometry(toPhysX(desc.halfExtents)), desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames,
                                          &transform, nullptr, massInfoOut, instance);
    }
    break;

    case eCapsuleShape:
    {
        const CapsulePhysxShapeDesc& desc = static_cast<const CapsulePhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
        CapsulePhysxShapeDesc desc2 = desc;

        const PxQuat fixupQ = fixupCapsuleQuat(desc.axis);
        PxQuat q = toPhysXQuat(desc2.localRot);
        q = q * fixupQ;
        desc2.localRot.x = q.x;
        desc2.localRot.y = q.y;
        desc2.localRot.z = q.z;
        desc2.localRot.w = q.w;

        physxType = ePTShape;
        outId = omni::physx::createShape(PxCapsuleGeometry(desc.radius, desc.halfHeight), desc2, key, physxScene,
                                          collisionGroup, rigidActor, exposePrimNames, nullptr, nullptr, massInfoOut, instance);
        if (massInfoOut)
        {
            auto newBounds = PxBounds3::transformFast(PxTransform(fixupQ), PxBounds3(toPhysX(massInfoOut->aabbLocalMin), toPhysX(massInfoOut->aabbLocalMax)));
            massInfoOut->aabbLocalMin = toFloat3 (newBounds.minimum);
            massInfoOut->aabbLocalMax = toFloat3 (newBounds.maximum);
        }
        if (outId != kInvalidObjectId)
        {
            InternalShape* intShape = (InternalShape*)db->getRecords()[outId].mInternalPtr;
            auto newBounds = PxBounds3::transformFast(PxTransform(fixupQ), PxBounds3(toPhysX(intShape->mMassInfo.aabbLocalMin), toPhysX(intShape->mMassInfo.aabbLocalMax)));
            intShape->mMassInfo.aabbLocalMin = toFloat3 (newBounds.minimum);
            intShape->mMassInfo.aabbLocalMax = toFloat3 (newBounds.maximum);
            intShape->mAxis = desc.axis;
        }
    }
    break;

    case eCylinderShape:
    {
        const CylinderPhysxShapeDesc& desc = static_cast<const CylinderPhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);

        physxType = ePTShape;

        if (omniPhysX.getCachedSettings().approximateCylinders)
        {
            const PxVec3 cylScale = getConeOrCylinderScale(desc.halfHeight, desc.radius, desc.axis);
            PxMeshScale scale = PxMeshScale(cylScale);
            PxConvexMeshGeometryFlags flags = PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;
            PxConvexMesh* cylinderMesh = physxSetup.getCylinderConvexMesh(desc.axis);
            PxConvexMeshGeometry convexGeom(cylinderMesh, scale, flags);
            if (convexGeom.isValid())
            {
                outId = omni::physx::createShape(convexGeom, desc, key, physxScene,
                    collisionGroup, rigidActor, exposePrimNames, nullptr, nullptr, massInfoOut, instance);
            }
            else
            {
                CARB_LOG_ERROR("Cylinder shape geometry not valid for:%s", pathText);
            }
        }
        else
        {
            CylinderPhysxShapeDesc desc2 = desc;

            const PxQuat fixupQ = fixupConeAndCylinderQuat(desc.axis);
            desc2.localRot = toFloat4(toPhysXQuat(desc2.localRot) * fixupQ);

            outId = omni::physx::createShape(PxConvexCoreGeometry(PxConvexCore::Cylinder(desc.halfHeight * 2.0f, desc.radius), desc.margin), desc2,
                key, physxScene, collisionGroup, rigidActor, exposePrimNames, nullptr, nullptr, massInfoOut, instance);

            if (massInfoOut)
            {
                auto newBounds = PxBounds3::transformFast(PxTransform(fixupQ), PxBounds3(toPhysX(massInfoOut->aabbLocalMin), toPhysX(massInfoOut->aabbLocalMax)));
                massInfoOut->aabbLocalMin = toFloat3(newBounds.minimum);
                massInfoOut->aabbLocalMax = toFloat3(newBounds.maximum);
            }

            if (outId != kInvalidObjectId)
            {
                InternalShape* intShape = (InternalShape*)db->getRecords()[outId].mInternalPtr;
                auto newBounds = PxBounds3::transformFast(PxTransform(fixupQ), PxBounds3(toPhysX(intShape->mMassInfo.aabbLocalMin), toPhysX(intShape->mMassInfo.aabbLocalMax)));
                intShape->mMassInfo.aabbLocalMin = toFloat3(newBounds.minimum);
                intShape->mMassInfo.aabbLocalMax = toFloat3(newBounds.maximum);
                intShape->mAxis = desc.axis;
            }
        }

        if (outId != kInvalidObjectId)
        {
            InternalShape* intShape = (InternalShape*)db->getRecords()[outId].mInternalPtr;
            intShape->mAxis = desc.axis;
        }
    }
    break;

    case eConeShape:
    {
        const ConePhysxShapeDesc& desc = static_cast<const ConePhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);

        physxType = ePTShape;

        if (omniPhysX.getCachedSettings().approximateCones)
        {
            const PxVec3 coneScale = getConeOrCylinderScale(desc.halfHeight, desc.radius, desc.axis);
            PxMeshScale scale = PxMeshScale(coneScale);
            PxConvexMeshGeometryFlags flags = PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;

            PxConvexMesh* coneMesh = physxSetup.getConeConvexMesh(desc.axis);

            PxConvexMeshGeometry convexGeom(coneMesh, scale, flags);
            if (convexGeom.isValid())
            {
                outId = omni::physx::createShape(convexGeom, desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames,
                    nullptr, nullptr, massInfoOut, instance);
            }
            else
            {
                CARB_LOG_ERROR("Cone shape geometry not valid for:%s", pathText);
            }
        }
        else
        {
            ConePhysxShapeDesc desc2 = desc;

            const PxQuat fixupQ = fixupConeAndCylinderQuat(desc.axis);
            desc2.localRot = toFloat4(toPhysXQuat(desc2.localRot) * fixupQ);

            outId = omni::physx::createShape(PxConvexCoreGeometry(PxConvexCore::Cone(desc.halfHeight * 2.0f, desc.radius), desc.margin), desc2,
                key, physxScene, collisionGroup, rigidActor, exposePrimNames, nullptr, nullptr, massInfoOut, instance);

            if (massInfoOut)
            {
                auto newBounds = PxBounds3::transformFast(PxTransform(fixupQ), PxBounds3(toPhysX(massInfoOut->aabbLocalMin), toPhysX(massInfoOut->aabbLocalMax)));
                massInfoOut->aabbLocalMin = toFloat3(newBounds.minimum);
                massInfoOut->aabbLocalMax = toFloat3(newBounds.maximum);
            }

            if (outId != kInvalidObjectId)
            {
                InternalShape* intShape = (InternalShape*)db->getRecords()[outId].mInternalPtr;
                auto newBounds = PxBounds3::transformFast(PxTransform(fixupQ), PxBounds3(toPhysX(intShape->mMassInfo.aabbLocalMin), toPhysX(intShape->mMassInfo.aabbLocalMax)));
                intShape->mMassInfo.aabbLocalMin = toFloat3(newBounds.minimum);
                intShape->mMassInfo.aabbLocalMax = toFloat3(newBounds.maximum);
                intShape->mAxis = desc.axis;
            }
        }

        if (outId != kInvalidObjectId)
        {
            InternalShape* intShape = (InternalShape*)db->getRecords()[outId].mInternalPtr;
            intShape->mAxis = desc.axis;
        }
    }
    break;

    case eCustomShape:
    {
        const CustomPhysxShapeDesc& desc = static_cast<const CustomPhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);

        physxType = ePTShape;

        outId = omni::physx::createShape(
            PxCustomGeometry(*OmniPhysX::getInstance().getCustomGeometryManager().createCustomGeometry(key, desc)),
            desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames, nullptr, nullptr, massInfoOut, instance);
    }
    break;

    case eConvexMeshShape:
    {
        const ConvexMeshPhysxShapeDesc& desc = static_cast<const ConvexMeshPhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
        PxConvexMesh* newConvex = createShapeAttachedStage ? cookingDataAsync->getConvexMesh(desc, desc.meshPrimKey, *createShapeAttachedStage, false) : nullptr;
        PxMeshScale meshScale(PxVec3(desc.meshScale.x, desc.meshScale.y, desc.meshScale.z).abs());
        PxConvexMeshGeometry convexGeom(newConvex, meshScale);
        if (convexGeom.isValid())
        {
            cleanupMultipleMaterials(desc, "ConvexHull", createShapeAttachedStage, key);

            if (!newConvex->isGpuCompatible())
            {
                CARB_LOG_INFO("ConvexMesh not GPU compatible, fall back to CPU, Object path: %s", pathText);
            }

            physxType = ePTShape;
            outId = omni::physx::createShape(convexGeom, desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames,
                                              nullptr, nullptr, massInfoOut, instance);
        }
        else
        {
            CARB_LOG_ERROR("Unable to create convex mesh for:%s", pathText);
            return kInvalidObjectId;
        }

    }
    break;

    case eConvexMeshDecompositionShape:
    {
        const ConvexMeshDecompositionPhysxShapeDesc& desc = static_cast<const ConvexMeshDecompositionPhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);

        std::vector<PxConvexMesh*> convexMeshes = createShapeAttachedStage ? cookingDataAsync->getConvexMeshDecomposition(desc, desc.meshPrimKey, *createShapeAttachedStage, false) : std::vector<PxConvexMesh*>{};
        const PxU32 cdNumConvexes = (PxU32)convexMeshes.size();
        PxMassProperties* massProps = new PxMassProperties[cdNumConvexes];
        PxTransform* transforms = new PxTransform[cdNumConvexes];
        PhysXUsdPhysicsInterface::MassInformation* massInfos = new PhysXUsdPhysicsInterface::MassInformation[cdNumConvexes];
        physxType = ePTCompoundShape;
        CompoundShape* shape = nullptr;

        cleanupMultipleMaterials(desc, "ConvexDecomposition", createShapeAttachedStage, key);

        if (db)
        {
            shape = new CompoundShape(physxScene);
            InternalShape* intShape = ICE_NEW(InternalShape)(physxScene, desc.localScale);
            if (desc.materials.size() == 1 && desc.materials[0] != kInvalidObjectId)
            {
                InternalDatabase::Record& materialRecord = db->getRecords()[(size_t)desc.materials[0]];
                InternalMaterial* intMat = (InternalMaterial*)materialRecord.mInternalPtr;
                intMat->addShapeId(db->getRecords().size());
                shape->mMaterialId = (size_t)desc.materials[0];
                intShape->mMaterialId = (size_t)desc.materials[0];
            }
            outId = db->addRecord(physxType, shape, intShape, key);
        }

        PxBounds3 sumAabbLocalBounds;
        sumAabbLocalBounds.setEmpty();
        for (PxU32 c = 0; c < cdNumConvexes; c++)
        {
            PhysXUsdPhysicsInterface::MassInformation& massInfo = massInfos[c];
            PxMeshScale meshScale(PxVec3(desc.meshScale.x, desc.meshScale.y, desc.meshScale.z).abs());
            PxConvexMeshGeometry convexGeom(convexMeshes[c], meshScale);
            if (convexGeom.isValid())
            {
                PxBounds3 aabbLocalBounds;
                PxGeometryQuery::computeGeomBounds(aabbLocalBounds, convexGeom, PxTransform(PxIdentity));
                sumAabbLocalBounds.include(aabbLocalBounds);
            }
        }

        for (PxU32 c = 0; c < cdNumConvexes; c++)
        {
            PhysXUsdPhysicsInterface::MassInformation& massInfo = massInfos[c];
            PxMeshScale meshScale(PxVec3(desc.meshScale.x, desc.meshScale.y, desc.meshScale.z).abs());
            PxConvexMeshGeometry convexGeom(convexMeshes[c], meshScale);
            if (convexGeom.isValid())
            {
                omni::physx::createShape(convexGeom, desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames,
                    nullptr, &outId, &massInfo, instance, &sumAabbLocalBounds);
            }

            massProps[c].centerOfMass = { float(massInfo.centerOfMass.x), float(massInfo.centerOfMass.y), float(massInfo.centerOfMass.z) };
            massProps[c].mass = float(massInfo.volume);

            massProps[c].inertiaTensor.column0[0] = float(massInfo.inertia[0]);
            massProps[c].inertiaTensor.column0[1] = float(massInfo.inertia[1]);
            massProps[c].inertiaTensor.column0[2] = float(massInfo.inertia[2]);

            massProps[c].inertiaTensor.column1[0] = float(massInfo.inertia[3]);
            massProps[c].inertiaTensor.column1[1] = float(massInfo.inertia[4]);
            massProps[c].inertiaTensor.column1[2] = float(massInfo.inertia[5]);

            massProps[c].inertiaTensor.column2[0] = float(massInfo.inertia[6]);
            massProps[c].inertiaTensor.column2[1] = float(massInfo.inertia[7]);
            massProps[c].inertiaTensor.column2[2] = float(massInfo.inertia[8]);

            transforms[c] = PxTransform(PxIdentity);
        }
        PxMassProperties outMassProps = PxMassProperties::sum(massProps, transforms, cdNumConvexes);

        delete[] massProps;
        delete[] transforms;

        PhysXUsdPhysicsInterface::MassInformation massInfo;
        const PxMat33 inertiaMatrix = (outMassProps.inertiaTensor);

        massInfo.inertia[0] = inertiaMatrix.column0[0];
        massInfo.inertia[1] = inertiaMatrix.column0[1];
        massInfo.inertia[2] = inertiaMatrix.column0[2];

        massInfo.inertia[3] = inertiaMatrix.column1[0];
        massInfo.inertia[4] = inertiaMatrix.column1[1];
        massInfo.inertia[5] = inertiaMatrix.column1[2];

        massInfo.inertia[6] = inertiaMatrix.column2[0];
        massInfo.inertia[7] = inertiaMatrix.column2[1];
        massInfo.inertia[8] = inertiaMatrix.column2[2];


        massInfo.volume = outMassProps.mass; // density is 1 so this if fine.
        massInfo.centerOfMass = { outMassProps.centerOfMass.x, outMassProps.centerOfMass.y, outMassProps.centerOfMass.z };
        massInfo.localPos = desc.localPos;
        massInfo.localRot = desc.localRot;
        massInfo.aabbLocalMax = fromPhysX(sumAabbLocalBounds.maximum);
        massInfo.aabbLocalMin = fromPhysX(sumAabbLocalBounds.minimum);
        delete[] massInfos;
        if (shape)
        {
            shape->mMassInfo = massInfo;
        }
        if (massInfoOut)
        {
            *massInfoOut = massInfo;
        }
    }
    break;

    case eTriangleMeshShape:
    {
        const TriangleMeshPhysxShapeDesc& desc = static_cast<const TriangleMeshPhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
        uint16_t maxMaterialIndex = 0;
        PxTriangleMesh* newMesh = createShapeAttachedStage ? cookingDataAsync->getTriangleMesh(desc, desc.meshPrimKey, *createShapeAttachedStage, false, nullptr, &maxMaterialIndex) : nullptr;
        if (newMesh)
        {
            PxMeshScale meshScale(PxVec3(desc.meshScale.x, desc.meshScale.y, desc.meshScale.z));
            if (desc.doubleSided)
            {
                CARB_LOG_INFO("Double sided attribute not supported in physics on a triangle mesh %s", pathText);
            }
            if (desc.sdfMeshCookingParams.sdfResolution > 0)
            {
                cleanupMultipleMaterials(desc, "SDF", createShapeAttachedStage, key);
            }
            else
            {
                if (newMesh->getTriangleMaterialIndex(0) == 0xffff)
                {
                    if (desc.materials.size() > 1)
                    {
                        PhysxShapeDesc& descWr = const_cast<PhysxShapeDesc&>(static_cast<const PhysxShapeDesc&>(objectDesc));
                        descWr.materials.clear();
                    }
                }
                else
                {
                    auto numDescMaterials = desc.materials.size();
                    if (maxMaterialIndex >= numDescMaterials)
                    {
                        PhysxShapeDesc& descWr = const_cast<PhysxShapeDesc&>(static_cast<const PhysxShapeDesc&>(objectDesc));
                        descWr.materials.clear();
                        for (PxU16 k = 0; k < maxMaterialIndex + 1; k++)
                        {
                            descWr.materials.push_back(kInvalidObjectId);
                        }                        
                    }
                }
            }

            physxType = ePTShape;
            outId = omni::physx::createShape(PxTriangleMeshGeometry(newMesh, meshScale), desc, key, physxScene, collisionGroup, rigidActor, exposePrimNames,
                                              nullptr, nullptr, massInfoOut, instance);
        }
        else
        {
            CARB_LOG_WARN("Unable to create triangle mesh for:%s",pathText );
            return kInvalidObjectId;
        }
    }
    break;

    case ePlaneShape:
    {
        const PlanePhysxShapeDesc& desc = static_cast<const PlanePhysxShapeDesc&>(objectDesc);
        const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);

        PlanePhysxShapeDesc desc2 = desc;
        {
            if (desc.axis == 1)
            {
                const float hRt2 = sqrt(2.0f) / 2.0f;
                const PxQuat fixupQ(hRt2, hRt2, 0.0f, 0.0f);
                desc2.localRot.x = fixupQ.x;
                desc2.localRot.y = fixupQ.y;
                desc2.localRot.z = fixupQ.z;
                desc2.localRot.w = fixupQ.w;
            }
            else if (desc.axis == 2)
            {
                const float hRt2 = sqrt(2.0f) / 2.0f;
                const PxQuat fixupQ(hRt2, 0.0f, hRt2, 0.0f);
                desc2.localRot.x = fixupQ.x;
                desc2.localRot.y = fixupQ.y;
                desc2.localRot.z = fixupQ.z;
                desc2.localRot.w = fixupQ.w;
            }
        }

        physxType = ePTShape;
        outId = omni::physx::createShape(
            PxPlaneGeometry(), desc2, key, physxScene, collisionGroup, rigidActor, exposePrimNames, nullptr, nullptr, massInfoOut, instance);
    }
    break;
    };
    if (massInfoOut && createShapeAttachedStage)
    {
        // World transform via the source (per-call at Default; replaces a
        // caller-supplied UsdGeomXformCache that was default-time-pinned).
        const ::physx::PxVec3 scale = omni::physx::getScale(getWorldTransform(
            *createShapeAttachedStage, key, omni::physics::parse::ReadTime::defaultTime()));
        massInfoOut->aabbLocalMin.x /= scale[0];
        massInfoOut->aabbLocalMin.y /= scale[1];
        massInfoOut->aabbLocalMin.z /= scale[2];
        massInfoOut->aabbLocalMax.x /= scale[0];
        massInfoOut->aabbLocalMax.y /= scale[1];
        massInfoOut->aabbLocalMax.z /= scale[2];
    }
    if (outId != kInvalidObjectId)
    {
        PhysXUsdPhysicsInterface::MassInformation* massInfo = nullptr;
        InternalDatabase::Record shapeRecord = db->getRecords()[outId];
        switch(shapeRecord.mType)
        {
            case ePTShape:
            {
                internal::InternalShape* intShape = (internal::InternalShape*)shapeRecord.mInternalPtr;
                massInfo = &intShape->mMassInfo;
            }
            break;
            case ePTCompoundShape:
            {
                internal::CompoundShape* shape = (internal::CompoundShape*)shapeRecord.mPtr;
                massInfo = &shape->mMassInfo;
            }
        }

        if(massInfo)
        {
            // World transform via the source (per-call at Default; the engine
            // xform cache will be reintroduced as a source-side cache later).
            // No direct-USD fallback: shape creation always runs with an attached
            // stage, so identity is the safe degenerate when one is somehow absent.
            const ::physx::PxMat44d worldMatrix = createShapeAttachedStage
                ? getWorldTransform(*createShapeAttachedStage, key, omni::physics::parse::ReadTime::defaultTime())
                : ::physx::PxMat44d(::physx::PxIdentity);
            const ::physx::PxVec3 scale = omni::physx::getScale(worldMatrix);
            massInfo->aabbLocalMin.x /= scale[0];
            massInfo->aabbLocalMin.y /= scale[1];
            massInfo->aabbLocalMin.z /= scale[2];
            massInfo->aabbLocalMax.x /= scale[0];
            massInfo->aabbLocalMax.y /= scale[1];
            massInfo->aabbLocalMax.z /= scale[2];
        }
    }

    return outId;
}

bool PhysXUsdPhysicsInterface::getRigidBodyShapes(const usdparser::AttachedStage& attachedStage, usdparser::ObjectId rbId, usdparser::ObjectIdPathMap& shapes) const
{
    bool hasTriggers = false;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    CARB_ASSERT(size_t(rbId) < db.getRecords().size());
    const InternalDatabase::Record& objectRecord = db.getRecords()[size_t(rbId)];
    if (objectRecord.mType == ePTActor || objectRecord.mType == ePTLink)
    {
        const PxRigidActor* actor = (const PxRigidActor * )objectRecord.mPtr;
        PxU32 numShapes = actor->getNbShapes();
        PxShape* shapePtr = nullptr;
        for (PxU32 i = 0; i < numShapes; i++)
        {
            actor->getShapes(&shapePtr, 1, i);
            // ignore trigger shapes for rigid shapes report
            if (shapePtr->getFlags() & PxShapeFlag::eTRIGGER_SHAPE)
            {
                hasTriggers = true;
                continue;
            }
            CARB_ASSERT(size_t(shapePtr->userData) < db.getRecords().size());
            const InternalDatabase::Record& shapeRecord = db.getRecords()[size_t(shapePtr->userData)];
            // ObjectKey directly (not a path/UsdPrim): resolves under any backend, and is all
            // the mass path needs (getShapeMassInfo is keyed purely by ObjectId).
            shapes[usdparser::ObjectId(shapePtr->userData)] = shapeRecord.mKey;
        }
    }
    return hasTriggers;
}

PhysXUsdPhysicsInterface::MassInformation PhysXUsdPhysicsInterface::getShapeMassInfo(ObjectId objectId) const
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    CARB_ASSERT(size_t(objectId) < db.getRecords().size());
    InternalDatabase::Record& objectRecord = db.getRecords()[size_t(objectId)];
    switch (objectRecord.mType)
    {
    case ePTShape:
    {
        InternalShape* intShape = (InternalShape*)objectRecord.mInternalPtr;
        return intShape->mMassInfo;
    }
    break;
    case ePTCompoundShape:
    {
        CompoundShape* shape = (CompoundShape*)objectRecord.mPtr;
        return shape->mMassInfo;
    }
    break;
    default:
    break;
    }

    MassInformation invalidMass;
    invalidMass.volume = -1.0f;
    return invalidMass;
}

PxJointLinearLimitPair getLinearLimitPair(PhysxJointLimit limitData)
{
    // soft limit
    if (limitData.stiffness > 0.0f || limitData.damping > 0.0f)
    {
        PxJointLinearLimitPair limit(limitData.lower, limitData.upper, PxSpring(limitData.stiffness, limitData.damping));
        limit.restitution = limitData.restitution;
        limit.bounceThreshold = limitData.bounceThreshold;
        return limit;
    }
    else
    // hard limit
    {
        const PxTolerancesScale tolerancesScale = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getTolerancesScale();
        PxJointLinearLimitPair limit(tolerancesScale, limitData.lower, limitData.upper);
        limit.restitution = limitData.restitution;
        limit.bounceThreshold = limitData.bounceThreshold;
        return limit;
    }
}

// `pathText` is purely the clamp-warning's diagnostic text -- ObjectKey-native callers pass
// attachedStage.textFor(key), no SdfPath needed.
PxJointAngularLimitPair getAngularLimitPair(PhysxJointLimit limitData, const char* pathText)
{
    const float twoPiEpsilon = (float)(2.0f * M_PI - 0.0000001f);
    if(limitData.lower < -twoPiEpsilon || limitData.upper > twoPiEpsilon)
    {
        OMNI_LOG_WARN(
        kRoboticsLogChannel,
        "Limit data for joint %s currently set to [%.1f - %.1f] will be clamped inside [-360, 360].",
        pathText, radToDeg(limitData.lower), radToDeg(limitData.upper));
    }
    // soft limit
    if (limitData.stiffness > 0.0f || limitData.damping > 0.0f)
    {
        PxJointAngularLimitPair limit(limitData.lower, limitData.upper, PxSpring(limitData.stiffness, limitData.damping));
        limit.restitution = limitData.restitution;
        limit.bounceThreshold = limitData.bounceThreshold;
        // we must clamp the limit *exclusively* between -2pi and 2pi
        limit.lower = PxMax(limit.lower, -twoPiEpsilon);
        limit.upper = PxMin(limit.upper, twoPiEpsilon);
        return limit;
    }
    else
        // hard limit
    {
        PxJointAngularLimitPair limit(limitData.lower, limitData.upper);
        limit.restitution = limitData.restitution;
        limit.bounceThreshold = limitData.bounceThreshold;
        // we must clamp the limit *exclusively* between -2pi and 2pi
        limit.lower = PxMax(limit.lower, -twoPiEpsilon);
        limit.upper = PxMin(limit.upper, twoPiEpsilon);
        return limit;
    }
}

PxJointLimitCone getConeLimit(PhysxJointLimit limitData)
{
    // soft limit
    if (limitData.stiffness > 0.0f || limitData.damping > 0.0f)
    {
        PxJointLimitCone limit(limitData.angle0, limitData.angle1, PxSpring(limitData.stiffness, limitData.damping));
        limit.restitution = limitData.restitution;
        limit.bounceThreshold = limitData.bounceThreshold;
        return limit;
    }
    else
        // hard limit
    {
        PxJointLimitCone limit(limitData.angle0, limitData.angle1);
        limit.restitution = limitData.restitution;
        limit.bounceThreshold = limitData.bounceThreshold;
        return limit;
    }
}

ObjectId PhysXUsdPhysicsInterface::createJoint(AttachedStage& attachedStage, omni::physics::parse::ObjectKey jointKey, const PhysxJointDesc& desc, ObjectId body0, ObjectId body1)
{
    checkScenes();
    if (desc.jointEnabled == false)
        return kInvalidObjectId;

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PxRigidActor* actor0 = getPtr<PxRigidActor>(ePTActor, body0);
    PxRigidActor* actor1 = getPtr<PxRigidActor>(ePTActor, body1);

    if (!actor0)
    {
        actor0 = getPtr<PxRigidActor>(ePTLink, body0);
    }
    if (!actor1)
    {
        actor1 = getPtr<PxRigidActor>(ePTLink, body1);
    }
    // This also filter the case of joints trying to be created on a different scene that is not forced scene (when active)
    if (!actor0 && !actor1)
        return kInvalidObjectId;

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();

    PxTransform localPose0 = toPhysX(desc.localPose0Position, desc.localPose0Orientation);
    PxTransform localPose1 = toPhysX(desc.localPose1Position, desc.localPose1Orientation);

    registerJointTimeSampledChanges(attachedStage, jointKey);

    InternalJoint* intJoint = ICE_NEW(InternalJoint);
    intJoint->mJointType = desc.type;  // must be set before any calls to fixupLocalPose
    PxJoint* j = nullptr;

    ObjectId retId = kInvalidObjectId;
    PhysXType physxType = ePTJoint;

    // Attach-scoped token cache (interned once in rebuildSource()); createJoint runs per joint.
    // No exists(jointKey) guard: the desc comes from a scan so the prim is live, and
    // hasSchema() answers by schema membership without the key being in the known-key seed.
    const omni::physics::parse::KnownTokens& jointTok = attachedStage.getKnownTokens();
    if(src && src->hasSchema(jointKey, jointTok.physicsJointStateAPI))
    {
        OMNI_LOG_WARN(
        kRoboticsLogChannel,
        "Physics USD: JointStateAPI applied to Joint %s it's not supported, and it will be ignored.",
        attachedStage.textFor(jointKey));
    }

    switch (desc.type)
    {
    case eJointFixed:
    {
        PxD6Joint* pj = PxD6JointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localPose0, actor1, localPose1);
        j = pj;

        if (!j)
        {
            break;
        }

        pj->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);

        pj->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
    }
    break;

    case eJointRevolute:
    {
        const RevolutePhysxJointDesc& jointDesc = static_cast<const RevolutePhysxJointDesc&>(desc);
        intJoint->mAxis = jointDesc.axis;
        intJoint->fixupLocalPose(localPose0);
        intJoint->fixupLocalPose(localPose1);

        PxD6Joint* pj = PxD6JointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localPose0, actor1, localPose1);
        j = pj;
        if (!j)
        {
            break;
        }

        pj->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);

        pj->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);

        usdparser::PhysxJointLimit limitData = jointDesc.limit;

        intJoint->mD6JointDrive = PxD6Drive::eTWIST;
        intJoint->mAxisIndex = 0;

        if (limitData.enabled)
        {
            PxJointAngularLimitPair limitPair = getAngularLimitPair(limitData, attachedStage.textFor(jointKey));
            checkRevoluteJointLimits(limitPair, attachedStage.textFor(jointKey));

            pj->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
            pj->setTwistLimit(limitPair);

            const bool enableExtendedJointAngles = OmniPhysX::getInstance().getCachedSettings().enableExtendedJointAngles;
            pj->setConstraintFlag(PxConstraintFlag::eENABLE_EXTENDED_LIMITS, enableExtendedJointAngles);
        }
        else
            pj->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);

        if ((src && src->hasSchema(jointKey, jointTok.physxJointAxisAPIAngular)))
        {
            removeAppliedAPI(attachedStage, jointKey, "PhysxJointAxisAPI", "angular");
            CARB_LOG_WARN("PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(jointKey));
        }

            const usdparser::PhysxJointDrive& driveData = jointDesc.drive;
        if (driveData.enabled)
        {
            registerDriveTimeSampledChanges(attachedStage, jointKey, "drive:angular");
            PxD6JointDrive drive(driveData.stiffness, driveData.damping, driveData.forceLimit, driveData.acceleration);
            pj->setDrive(PxD6Drive::eTWIST, drive);
            if(driveData.isEnvelopeUsed)
            {
                removeAppliedAPI(attachedStage, jointKey, "PhysxDrivePerformanceEnvelopeAPI", "angular");
                CARB_LOG_WARN("Performance envelope is supported only for joints that are part of an articulation. Envelope will be ignored for joint: %s", attachedStage.textFor(jointKey));
            }

            intJoint->mJointDrive = driveData;            

            // A.B. Switch the sign to be consistent with articulations and position drive behavior. Remove when changed in the SDK
            // OM-42441
            intJoint->mJointDrive.targetVelocity = -intJoint->mJointDrive.targetVelocity;
            const PxVec3 angVel(intJoint->mJointDrive.targetVelocity, 0.0f, 0.0f);
            const PxVec3 driveVel(0.0f, 0.0f, 0.0f);
            pj->setDriveVelocity(driveVel, angVel);

            const PxVec3 drivePos(0.0f, 0.0f, 0.0f);
            const PxVec3 rotAxis(1.0f, 0.0f, 0.0f);
            PxQuat rot(driveData.targetPosition, rotAxis);
            pj->setDrivePosition(PxTransform(drivePos, rot));
        }
    }
    break;

    case eJointPrismatic:
    {
        const PrismaticPhysxJointDesc& jointDesc = static_cast<const PrismaticPhysxJointDesc&>(desc);

        intJoint->mAxis = jointDesc.axis;
        intJoint->fixupLocalPose(localPose0);
        intJoint->fixupLocalPose(localPose1);
        PxD6Joint* pj = PxD6JointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localPose0, actor1, localPose1);
        j = pj;
        if (!j)
        {
            break;
        }

        pj->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);

        pj->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);

        const usdparser::PhysxJointLimit& limitData = jointDesc.limit;

        const PxD6Axis::Enum d6Axis = PxD6Axis::eX;
        const PxD6Drive::Enum d6drive = PxD6Drive::eX;
        const uint32_t axisIndex = 0;
        intJoint->mD6JointDrive = d6drive;
        intJoint->mAxisIndex = axisIndex;

        if (limitData.enabled)
        {
            pj->setMotion(d6Axis, PxD6Motion::eLIMITED);
            pj->setLinearLimit(d6Axis, getLinearLimitPair(limitData));
        }
        else
            pj->setMotion(d6Axis, PxD6Motion::eFREE);

        if ((src && src->hasSchema(jointKey, jointTok.physxJointAxisAPILinear)))
        {
            removeAppliedAPI(attachedStage, jointKey, "PhysxJointAxisAPI", "linear");
            CARB_LOG_WARN("PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(jointKey));
        }
        const usdparser::PhysxJointDrive& driveData = jointDesc.drive;
        if (driveData.enabled)
        {
            registerDriveTimeSampledChanges(attachedStage, jointKey, "drive:linear");
            PxD6JointDrive drive(driveData.stiffness, driveData.damping, driveData.forceLimit, driveData.acceleration);
            pj->setDrive(d6drive, drive);

            intJoint->mJointDrive = driveData;

            const PxVec3 angVel(0.0f, 0.0f, 0.0f);
            PxVec3 driveVel(0.0f, 0.0f, 0.0f);
            driveVel[axisIndex] = driveData.targetVelocity;
            pj->setDriveVelocity(driveVel, angVel);

            PxVec3 drivePos(0.0f, 0.0f, 0.0f);
            drivePos[axisIndex] = driveData.targetPosition;
            pj->setDrivePosition(PxTransform(drivePos));
            if(driveData.isEnvelopeUsed)
            {
                removeAppliedAPI(attachedStage, jointKey, "PhysxDrivePerformanceEnvelopeAPI", "linear");
                CARB_LOG_WARN("Performance envelope is supported only for joints that are part of an articulation. Envelope will be ignored for joint: %s", attachedStage.textFor(jointKey));
            }
        }
    }
    break;

    case eJointSpherical:
    {
        const SphericalPhysxJointDesc& jointDesc = static_cast<const SphericalPhysxJointDesc&>(desc);
        intJoint->mAxis = jointDesc.axis;
        intJoint->fixupLocalPose(localPose0);
        intJoint->fixupLocalPose(localPose1);

        PxD6Joint* pj = PxD6JointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localPose0, actor1, localPose1);
        j = pj;

        if (!j)
        {
            break;
        }

        pj->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
        pj->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
        pj->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);

        pj->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
        pj->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);

        const usdparser::PhysxJointLimit& limitData = jointDesc.limit;
        if (limitData.enabled)
        {
            pj->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
            pj->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
            pj->setSwingLimit(getConeLimit(limitData));
        }
        for (size_t i = 0; i < jointDesc.jointProperties.size(); i++)
        {
            const JointAxis axis = jointDesc.jointProperties[i].first;
        
            switch (axis)
            {
                case eTransX:
                case eTransY:
                case eTransZ:
                case eDistance:
                    break;
                case eRotX:
                    if ((src && src->hasSchema(jointKey, jointTok.physxJointAxisAPIRotX)))
                    {
                        removeAppliedAPI(attachedStage, jointKey, "PhysxJointAxisAPI", "rotX");
                        CARB_LOG_WARN("PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(jointKey));
                    }
                    break;
                case eRotY:
                    if ((src && src->hasSchema(jointKey, jointTok.physxJointAxisAPIRotY)))
                    {
                        removeAppliedAPI(attachedStage, jointKey, "PhysxJointAxisAPI", "rotY");
                        CARB_LOG_WARN("PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(jointKey));
                    }
                    break;
                case eRotZ:
                    if ((src && src->hasSchema(jointKey, jointTok.physxJointAxisAPIRotZ)))
                    {
                        removeAppliedAPI(attachedStage, jointKey, "PhysxJointAxisAPI", "rotZ");
                        CARB_LOG_WARN("PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(jointKey));
                    }
                    break;
            }
            
        }
    }
    break;

    case eJointDistance:
    {
        const DistancePhysxJointDesc& jointDesc = static_cast<const DistancePhysxJointDesc&>(desc);

        PxDistanceJoint* dj = PxDistanceJointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localPose0, actor1, localPose1);
        j = dj;

        if (!j)
        {
            break;
        }

        const float minDist = jointDesc.limit.lower;
        const float maxDist = jointDesc.limit.upper;

        dj->setDamping(jointDesc.limit.damping);
        dj->setStiffness(jointDesc.limit.stiffness);

        if (jointDesc.maxEnabled)
        {
            dj->setMaxDistance(maxDist);
            dj->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
        }
        else
            dj->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, false);

        if (jointDesc.minEnabled)
        {
            dj->setMinDistance(minDist);
            dj->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
        }
        else
            dj->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, false);

        dj->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, jointDesc.springEnabled);
        if (jointDesc.springEnabled)
        {
            dj->setDamping(jointDesc.damping);
            dj->setStiffness(jointDesc.stiffness);
        }
    }
    break;

    case eJointGear:
    {
        const GearPhysxJointDesc& jointDesc = static_cast<const GearPhysxJointDesc&>(desc);

        PxTransform localFrame0 = localPose0;
        PxTransform localFrame1 = localPose1;

        PxBase* hinge0 = nullptr;
        if (!getJointAndLocalPose(attachedStage, jointDesc.hingePrimPath0, actor0, hinge0, localFrame0))
        {
            CARB_LOG_ERROR("Invalid configuration for gear joint(%s) - neither parent nor child link of hinge 0 (%s) refer to body 0.", attachedStage.textFor(jointKey), attachedStage.textFor(jointDesc.hingePrimPath0));
            return kInvalidObjectId;
        }

        PxBase* hinge1 = nullptr;
        if (!getJointAndLocalPose(attachedStage, jointDesc.hingePrimPath1, actor1, hinge1, localFrame1))
        {
            CARB_LOG_ERROR("Invalid configuration for gear joint(%s) - neither parent nor child link of hinge 0 (%s) refer to body 0.", attachedStage.textFor(jointKey), attachedStage.textFor(jointDesc.hingePrimPath1));
            return kInvalidObjectId;
        }

        if (!hinge0 || !hinge1)
        {
            if (!hinge0)
                CARB_LOG_WARN("Cannot simulate gear joint (%s) without two valid Revolute joint rels.", attachedStage.textFor(jointKey));
            if (!hinge1)
                CARB_LOG_WARN("Cannot simulate gear joint (%s) without two valid Revolute joint rels.", attachedStage.textFor(jointKey));
        }


        PxGearJoint* gj = PxGearJointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localFrame0, actor1, localFrame1);
        j = gj;
        if (!j)
        {
            break;
        }

	    gj->setHinges(hinge0, hinge1);
	    gj->setGearRatio(jointDesc.gearRatio);
        break;
    };

    case eJointRackAndPinion:
    {
        const RackPhysxJointDesc& jointDesc = static_cast<const RackPhysxJointDesc&>(desc);

        PxTransform localFrame0 = localPose0;
        PxTransform localFrame1 = localPose1;

        PxBase* hinge = nullptr;
        if (!getJointAndLocalPose(attachedStage, jointDesc.hingePrimKey, actor0, hinge, localFrame0))
        {
            CARB_LOG_ERROR(
                "Invalid configuration for rack and pinion joint(%s) - neither parent nor child link of hinge 0 (%s) refer to body 0.",
                attachedStage.textFor(jointKey), attachedStage.textFor(jointDesc.hingePrimKey));
            return kInvalidObjectId;
        }

        PxBase* prismatic = nullptr;
        if (!getJointAndLocalPose(attachedStage, jointDesc.prismaticPrimKey, actor1, prismatic, localFrame1))
        {
            CARB_LOG_ERROR(
                "Invalid configuration for rack and pinion joint(%s) - neither parent nor child link of prismatic 1 (%s) refer to body 1.",
                attachedStage.textFor(jointKey), attachedStage.textFor(jointDesc.prismaticPrimKey));
            return kInvalidObjectId;
        }

        if (!hinge || !prismatic)
        {
            if (!hinge)
                CARB_LOG_WARN("Cannot simulate gear joint (%s) without two valid Revolute joint rel.", attachedStage.textFor(jointKey));
            if (!prismatic)
                CARB_LOG_WARN("Cannot simulate gear joint (%s) without two valid Prismatic joint rel.", attachedStage.textFor(jointKey));         
        }

        PxRackAndPinionJoint* rj = PxRackAndPinionJointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localFrame0, actor1, localFrame1);
        j = rj;
        if (!j)
        {
            break;
        }

	    rj->setJoints(hinge, prismatic);
        // USD ratio is deg / distance - PhysX is rad / distance
        rj->setRatio(degToRad(jointDesc.ratio));
        break;
    };

    case eJointD6:
    {
        const D6PhysxJointDesc& jointDesc = static_cast<const D6PhysxJointDesc&>(desc);

        PxD6Joint* d6j = PxD6JointCreate(*OmniPhysX::getInstance().getPhysXSetup().getPhysics(), actor0, localPose0, actor1, localPose1);
        j = d6j;
        if (!j)
        {
            break;
        }

        d6j->setAngularDriveConfig(PxD6AngularDriveConfig::eSWING_TWIST);

        d6j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
        d6j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
        d6j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);

        d6j->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
        d6j->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
        d6j->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);

        bool usePyramidalLimit = false;
        PxJointLimitPyramid limitPyramidal(0.0f, 0.0f, 0.0f, 0.0f);

        bool hasRotationalLimit = false;

        // parse limits
        for (size_t i = 0; i < jointDesc.jointLimits.size(); i++)
        {
            const JointAxis axis = jointDesc.jointLimits[i].first;
            const PhysxJointLimit& limitData = jointDesc.jointLimits[i].second;
            const bool locked = (limitData.lower > limitData.upper) ? true : false;

            switch (axis)
            {
            case eTransX:
                if (locked)
                    d6j->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
                else if (limitData.enabled)
                {
                    d6j->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
                    d6j->setLinearLimit(PxD6Axis::eX, getLinearLimitPair(limitData));
                }
                break;
            case eTransY:
                if (locked)
                    d6j->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
                else if (limitData.enabled)
                {
                    d6j->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
                    d6j->setLinearLimit(PxD6Axis::eY, getLinearLimitPair(limitData));
                }
                break;
            case eTransZ:
                if (locked)
                    d6j->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
                else if (limitData.enabled)
                {
                    d6j->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
                    d6j->setLinearLimit(PxD6Axis::eZ, getLinearLimitPair(limitData));
                }
                break;
            case eRotX:
                if (locked)
                    d6j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLOCKED);
                else if (limitData.enabled)
                {
                    d6j->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
                    d6j->setTwistLimit(getAngularLimitPair(limitData, attachedStage.textFor(jointKey)));

                    hasRotationalLimit = true;
                }
                break;
            case eRotY:
                if (locked)
                    d6j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
                else if (limitData.enabled)
                {
                    usePyramidalLimit = true;
                    limitPyramidal.yAngleMin = limitData.lower;
                    limitPyramidal.yAngleMax = limitData.upper;
                    limitPyramidal.damping = limitData.damping;
                    limitPyramidal.stiffness = limitData.stiffness;
                    limitPyramidal.restitution = limitData.restitution;
                    limitPyramidal.bounceThreshold = limitData.bounceThreshold;
                    d6j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);

                    hasRotationalLimit = true;
                }
                break;
            case eRotZ:
                if (locked)
                    d6j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);
                else if (limitData.enabled)
                {
                    usePyramidalLimit = true;
                    limitPyramidal.zAngleMin = limitData.lower;
                    limitPyramidal.zAngleMax = limitData.upper;
                    limitPyramidal.damping = limitData.damping;
                    limitPyramidal.stiffness = limitData.stiffness;
                    limitPyramidal.restitution = limitData.restitution;
                    limitPyramidal.bounceThreshold = limitData.bounceThreshold;
                    d6j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);

                    hasRotationalLimit = true;
                }
                break;
            case eDistance:
                if (limitData.lower > 0.0f)
                {
                    CARB_LOG_WARN("Distance limit on a D6 joint with a min value is not support currently. %s", attachedStage.textFor(jointKey));
                }
                const PxJointLinearLimit linearLimit(limitData.upper, PxSpring(limitData.stiffness, limitData.damping));
                d6j->setDistanceLimit(linearLimit);
                if (d6j->getMotion(PxD6Axis::eX) == PxD6Motion::eFREE)
                    d6j->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
                if (d6j->getMotion(PxD6Axis::eY) == PxD6Motion::eFREE)
                    d6j->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
                if (d6j->getMotion(PxD6Axis::eZ) == PxD6Motion::eFREE)
                    d6j->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
                break;
            }
        }

        if (usePyramidalLimit)
        {
            if (limitPyramidal.stiffness > 0.0f || limitPyramidal.damping > 0.0f)
            {
                PxJointLimitPyramid limit(limitPyramidal.yAngleMin, limitPyramidal.yAngleMax, limitPyramidal.zAngleMin, limitPyramidal.zAngleMax, PxSpring(limitPyramidal.stiffness, limitPyramidal.damping));
                limit.restitution = limitPyramidal.restitution;
                limit.bounceThreshold = limitPyramidal.bounceThreshold;
                d6j->setPyramidSwingLimit(limit);
            }
            else
            {
                PxJointLimitPyramid limit(limitPyramidal.yAngleMin, limitPyramidal.yAngleMax, limitPyramidal.zAngleMin, limitPyramidal.zAngleMax);
                limit.restitution = limitPyramidal.restitution;
                limit.bounceThreshold = limitPyramidal.bounceThreshold;
                d6j->setPyramidSwingLimit(limit);
            }
        }

        if (hasRotationalLimit)
        {
            const bool enableExtendedJointAngles = OmniPhysX::getInstance().getCachedSettings().enableExtendedJointAngles;
            d6j->setConstraintFlag(PxConstraintFlag::eENABLE_EXTENDED_LIMITS, enableExtendedJointAngles);
        }

        for (int i = 0; i < 6; i++)
        {
            intJoint->mJointDrives[i].enabled = false;
            intJoint->mJointDrives[i].targetVelocity = 0.0f;
            intJoint->mJointDrives[i].targetPosition = 0.0f;
        }

        // parse drives
        PxVec3 posDrive(0.0f);
        PxVec3 angDrive(0.0f);
        PxVec3 linVelDrive(0.0f);
        PxVec3 angVelDrive(0.0f);
        for (size_t i = 0; i < jointDesc.jointProperties.size(); i++)
        {
            const JointAxis axis = jointDesc.jointProperties[i].first;
        
            switch (axis)
            {
                case eTransX:
                case eTransY:
                case eTransZ:
                case eDistance:
                    break;
                case eRotX:
                    if ((src && src->hasSchema(jointKey, jointTok.physxJointAxisAPIRotX)))
                    {
                        removeAppliedAPI(attachedStage, jointKey, "PhysxJointAxisAPI", "rotX");
                        CARB_LOG_WARN("PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(jointKey));
                    }
                    break;
                case eRotY:
                    if ((src && src->hasSchema(jointKey, jointTok.physxJointAxisAPIRotY)))
                    {
                        removeAppliedAPI(attachedStage, jointKey, "PhysxJointAxisAPI", "rotY");
                        CARB_LOG_WARN("PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(jointKey));
                    }
                    break;
                case eRotZ:
                    if ((src && src->hasSchema(jointKey, jointTok.physxJointAxisAPIRotZ)))
                    {
                        removeAppliedAPI(attachedStage, jointKey, "PhysxJointAxisAPI", "rotZ");
                        CARB_LOG_WARN("PhysxJointAxisAPI is supported only for joints that are part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(jointKey));
                    }
                    break;
            }
            
        }
        for (size_t i = 0; i < jointDesc.jointDrives.size(); i++)
        {
            const JointAxis axis = jointDesc.jointDrives[i].first;
            const PhysxJointDrive& driveData = jointDesc.jointDrives[i].second;

            PxD6JointDrive drive(driveData.stiffness, driveData.damping, driveData.forceLimit, driveData.acceleration);
            switch (axis)
            {
            case eTransX:
                registerDriveTimeSampledChanges(attachedStage, jointKey, "drive:transX");
                intJoint->mJointDrives[0] = driveData;
                d6j->setDrive(PxD6Drive::eX, drive);
                linVelDrive.x = driveData.targetVelocity;
                posDrive.x = driveData.targetPosition;
                if(driveData.isEnvelopeUsed)
                {
                    removeAppliedAPI(attachedStage, jointKey, "PhysxDrivePerformanceEnvelopeAPI", "transX");
                    CARB_LOG_WARN("Performance envelope is supported only for joints that are part of an articulation. Envelope will be ignored for joint: %s", attachedStage.textFor(jointKey));
                }
                break;
            case eTransY:
                registerDriveTimeSampledChanges(attachedStage, jointKey, "drive:transY");
                intJoint->mJointDrives[1] = driveData;
                d6j->setDrive(PxD6Drive::eY, drive);
                linVelDrive.y = driveData.targetVelocity;
                posDrive.y = driveData.targetPosition;
                if(driveData.isEnvelopeUsed)
                {
                    removeAppliedAPI(attachedStage, jointKey, "PhysxDrivePerformanceEnvelopeAPI", "transY");
                    CARB_LOG_WARN("Performance envelope is supported only for joints that are part of an articulations. Envelope will be ignored for joint: %s", attachedStage.textFor(jointKey));
                }
                break;
            case eTransZ:
                registerDriveTimeSampledChanges(attachedStage, jointKey, "drive:transZ");
                intJoint->mJointDrives[2] = driveData;
                d6j->setDrive(PxD6Drive::eZ, drive);
                linVelDrive.z = driveData.targetVelocity;
                posDrive.z = driveData.targetPosition;
                if(driveData.isEnvelopeUsed)
                {
                    removeAppliedAPI(attachedStage, jointKey, "PhysxDrivePerformanceEnvelopeAPI", "transZ");
                    CARB_LOG_WARN("Performance envelope is supported only for joints that are part of an articulation. Envelope will be ignored for joint: %s", attachedStage.textFor(jointKey));
                }
                break;
            case eRotX:
                registerDriveTimeSampledChanges(attachedStage, jointKey, "drive:rotX");
                intJoint->mJointDrives[3] = driveData;
                d6j->setDrive(PxD6Drive::eTWIST, drive);
                // preist: Switch the sign to be consistent with articulations and position drive behavior. Remove when changed in the SDK
                // OM-42441
                angVelDrive.x = -driveData.targetVelocity;
                angDrive.x = driveData.targetPosition;
                if(driveData.isEnvelopeUsed)
                {
                    removeAppliedAPI(attachedStage, jointKey, "PhysxDrivePerformanceEnvelopeAPI", "rotX");
                    CARB_LOG_WARN("Performance envelope is supported only for joints that are part of an articulation. Envelope will be ignored for joint: %s", attachedStage.textFor(jointKey));
                }
                break;
            case eRotY:
                registerDriveTimeSampledChanges(attachedStage, jointKey, "drive:rotY");
                intJoint->mJointDrives[4] = driveData;
                d6j->setDrive(PxD6Drive::eSWING1, drive);
                // preist: Switch the sign to be consistent with articulations and position drive behavior. Remove when changed in the SDK
                // OM-42441
                angVelDrive.y = -driveData.targetVelocity;
                angDrive.y = driveData.targetPosition;
                if(driveData.isEnvelopeUsed)
                {
                    removeAppliedAPI(attachedStage, jointKey, "PhysxDrivePerformanceEnvelopeAPI", "rotY");
                    CARB_LOG_WARN("Performance envelope is supported only for joints that are part of an articulation. Envelope will be ignored for joint: %s", attachedStage.textFor(jointKey));
                }
                break;
            case eRotZ:
                registerDriveTimeSampledChanges(attachedStage, jointKey, "drive:rotZ");
                intJoint->mJointDrives[5] = driveData;
                d6j->setDrive(PxD6Drive::eSWING2, drive);
                // preist: Switch the sign to be consistent with articulations and position drive behavior. Remove when changed in the SDK
                // OM-42441
                angVelDrive.z = -driveData.targetVelocity;
                angDrive.z = driveData.targetPosition;
                if(driveData.isEnvelopeUsed)
                {
                    removeAppliedAPI(attachedStage, jointKey, "PhysxDrivePerformanceEnvelopeAPI", "rotZ");
                    CARB_LOG_WARN("Performance envelope is supported only for joints that are part of an articulation. Envelope will be ignored for joint: %s", attachedStage.textFor(jointKey));
                }
                break;
            case eDistance:
                break;
            }
        }
        const PxQuat qx(angDrive.x, PxVec3(1.0f, 0.0f, 0.0f));
        const PxQuat qy(angDrive.y, PxVec3(0.0f, 1.0f, 0.0f));
        const PxQuat qz(angDrive.z, PxVec3(0.0f, 0.0f, 1.0f));
        const PxQuat rot = qz * qy * qx;

        d6j->setDrivePosition(PxTransform(posDrive, rot));
        d6j->setDriveVelocity(linVelDrive, angVelDrive);
    }
    break;
    case eJointCustom:
    {
        const CustomPhysxJointDesc& jointDesc = static_cast<const CustomPhysxJointDesc&>(desc);

        CustomPhysXJoint* customPhysXJoint = OmniPhysX::getInstance().getCustomJointManager().createCustomJoint(
            attachedStage, jointKey, jointDesc, actor0, localPose0, actor1, localPose1);

        if (customPhysXJoint)
        {
            physxType = ePTCustomJoint;
            retId = db.addRecord(physxType, customPhysXJoint, intJoint, jointKey);
        }
        break;
    };

    default:
    {
        CARB_ASSERT(0);
    }
    break;
    };

    if (j)
    {
        j->setBreakForce(isfinite(desc.breakForce) ? desc.breakForce : FLT_MAX,
            isfinite(desc.breakTorque) ? desc.breakTorque : FLT_MAX);

        j->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, desc.enableCollision);

        j->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);

        if (mExposePrimNames)
            j->setName(attachedStage.textFor(jointKey));

        if (OmniPhysX::getInstance().isDebugVisualizationEnabled())
            j->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, OmniPhysX::getInstance().isDebugVisualizationEnabled());

        retId = db.addRecord(physxType, j, intJoint, jointKey);
        j->userData = (void*)retId;
    }

    sendObjectCreationNotification(jointKey, retId, physxType);

    return retId;
}

void PhysXUsdPhysicsInterface::recreateArticulationJoint(AttachedStage& attachedStage,
    const PhysxJointDesc& jointDesc, ObjectId link0Id, ObjectId link1Id)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    const PxArticulationLink* link0 = reinterpret_cast<const PxArticulationLink*>(db.getTypedRecord(ePTLink, link0Id));
    if (link0)
    {
        const PxArticulationLink* link1 = reinterpret_cast<const PxArticulationLink*>(db.getTypedRecord(ePTLink, link1Id));
        if (link1)
        {
            omni::physics::parse::ObjectKey childLinkKey;

            PxArticulationJointReducedCoordinate* joint = link1->getInboundJoint();
            if (joint)
            {
                const PxArticulationLink& link = joint->getParentArticulationLink();
                if (&link == link0)
                {
                    childLinkKey = jointDesc.body1;
                }
                else
                {
                    joint = link0->getInboundJoint();
                    if (joint)
                    {
                        const PxArticulationLink& link = joint->getParentArticulationLink();
                        if (&link == link1)
                        {
                            childLinkKey = jointDesc.body0;
                        }
                        else
                        {
                            joint = nullptr;
                        }
                    }
                }
            }
            else
            {
                joint = link0->getInboundJoint();
                if (joint)
                {
                    const PxArticulationLink& link = joint->getParentArticulationLink();
                    if (&link == link1)
                    {
                        childLinkKey = jointDesc.body0;
                    }
                    else
                    {
                        joint = nullptr;
                    }
                }
            }

            if (joint)
            {
                createArticulationJoint<false>(attachedStage, jointDesc, joint, childLinkKey, nullptr);
            }
        }
    }
}


void PhysXUsdPhysicsInterface::releaseObject(AttachedStage& attachedStage, omni::physics::parse::ObjectKey removedKey, ObjectId objectId)
{
    if(!OmniPhysX::getInternalPhysXDatabaseCheck())
    {
        CARB_ASSERT(0); // Should not happen.
        return;
    }

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    if (objectId >= db.getRecords().size())
    {
        CARB_LOG_ERROR("Removed path does point to non-existing objectId, path: %s", attachedStage.textFor(removedKey));
        return;
    }

    InternalDatabase::Record& objectRecord = db.getRecords()[objectId];

    // make sure we are not running simulation when releasing objects
    waitForSimulationCompletion(false);

    sendObjectDestructionNotification(objectRecord.mKey, objectId, objectRecord.mType);

    switch (objectRecord.mType)
    {
    case ePTRemoved:
    break;
    case ePTShape:
    {
        PxShape* shape = (PxShape*)objectRecord.mPtr;
        InternalShape* intShape = (InternalShape*)objectRecord.mInternalPtr;
        intShape->mPhysicsScene->getContactReport()->removeShape(shape, removedKey);
        OmniPhysX::getInstance().getTriggerManager()->clearBufferedShape(shape);
        removeFilteredObject<uint32_t>(internal::convertFilterPairFromPxFilterData(shape->getSimulationFilterData()), physxSetup.getFilteredPairs());
        PxRigidActor* pxActor = shape->getActor();
        if (pxActor && !intShape->mDetached)
        {
            const size_t actorIndex = (const size_t)(pxActor->userData);
            InternalDatabase::Record& actorRecord = db.getRecords()[actorIndex];
            if (actorRecord.mType != ePTRemoved)
            {
                db.addDirtyMassActor((InternalActor*)actorRecord.mInternalPtr);
                pxActor->detachShape(*shape);

                intShape->mPhysicsScene->getInternalScene()->updateVehicleOnRemovedShape(*pxActor, shape);

                // set shape removed event flag for attachments and collision filters
                updateDeformableAttachmentShapeEvents((InternalActor*)actorRecord.mInternalPtr, objectId, removedKey, internal::DirtyEventType::eShapeRemoved);
                updateDeformableCollisionFilterShapeEvents((InternalActor*)actorRecord.mInternalPtr, objectId, removedKey, internal::DirtyEventType::eShapeRemoved);

                shape->release();
            }
        }
        if (intShape && intShape->mMaterialId != kInvalidObjectId)
        {
            InternalDatabase::Record& materialRecord = db.getRecords()[intShape->mMaterialId];
            InternalMaterial* mat = (InternalMaterial*)materialRecord.mInternalPtr;
            if (mat)
            {
                mat->removeShapeId(objectId);
            }           
        }
        SAFE_DELETE_ALLOCABLE_SINGLE(intShape);
        objectRecord.setRemoved();
    }
    break;
    case ePTCompoundShape:
    {
        CompoundShape* shape = (CompoundShape*)objectRecord.mPtr;
        PhysXScene* ps = shape->mPhysXScene;
        for (size_t i = 0; i < shape->getShapes().size(); i++)
        {
            PxShape* pxShape = (PxShape*)shape->getShapes()[i];
            ps->getContactReport()->removeShape(pxShape, removedKey);
            OmniPhysX::getInstance().getTriggerManager()->clearBufferedShape(pxShape);
            removeFilteredObject<uint32_t>(internal::convertFilterPairFromPxFilterData(pxShape->getSimulationFilterData()), physxSetup.getFilteredPairs());
            PxRigidActor* pxActor = pxShape->getActor();
            if (pxActor)
            {
                const size_t actorIndex = (const size_t)(pxActor->userData);
                InternalDatabase::Record& actorRecord = db.getRecords()[actorIndex];
                if (actorRecord.mType != ePTRemoved)
                {
                    db.addDirtyMassActor((InternalActor*)actorRecord.mInternalPtr);
                    pxActor->detachShape(*pxShape);

                    ps->getInternalScene()->updateVehicleOnRemovedShape(*pxActor, pxShape);

                    pxShape->release();
                }
            }
        }
        if (shape->mMaterialId != kInvalidObjectId)
        {
            InternalDatabase::Record& materialRecord = db.getRecords()[shape->mMaterialId];
            InternalMaterial* mat = (InternalMaterial*)materialRecord.mInternalPtr;
            if (mat)
            {
                mat->removeShapeId(objectId);
            }        
        }
        SAFE_DELETE_SINGLE(shape);
        objectRecord.setRemoved();
    }
    break;
    case ePTActor:
    {
        PxRigidActor* actor = (PxRigidActor*)objectRecord.mPtr;
        InternalActor* intActor = (InternalActor*)objectRecord.mInternalPtr;
        PhysXScene* ps = intActor->mPhysXScene;
        OmniPhysX::getInstance().getRaycastManager().clearPicker(actor);
        ps->getContactReport()->removeActor(actor, removedKey);
        PxShape* shapePtr = nullptr;
        for (PxU32 i = 0; i < actor->getNbShapes(); i++)
        {
            actor->getShapes(&shapePtr, 1, i);
            removeFilteredObject<uint32_t>(internal::convertFilterPairFromPxFilterData(shapePtr->getSimulationFilterData()), physxSetup.getFilteredPairs());
            const size_t shapeIndex = (size_t)shapePtr->userData;
            if (shapeIndex < db.getRecords().size())
            {
                InternalDatabase::Record& shapeRecord = db.getRecords()[shapeIndex];
                if (shapeRecord.mType == ePTShape)
                {
                    InternalShape* intShape = (InternalShape*)shapeRecord.mInternalPtr;
                    intShape->mDetached = true;
                }
            }
        }

        // release the deformable attachments and collision filters before rigidbodies (including mirrored) are released
        releaseDeformableAttachments(intActor);
        releaseDeformableCollisionFilters(intActor);

        for (MirrorActor& mirror : intActor->mMirrors)
        {
            mirror.release();
        }
        SAFE_RELEASE(intActor->mMirrorSharedCollection);

        db.storePxJoint(actor, intActor->mKey);

        ps->getInternalScene()->trackReleasedActiveActor(actor);
        actor->release();
        db.removeDirtyMassActor(intActor);
        // A.B. this can be easily a bottleneck
        if (!ps->getInternalScene()->removeActor(*intActor))
        {
            // The entry is looked up through intActor->mPhysXScene. If it is not there the actor is
            // about to be deleted while some other scene still references it (NVBugs 6504495).
            CARB_LOG_ERROR("Removed rigid body %s was not registered with its own scene.",
                           attachedStage.textFor(removedKey));
        }
        {
            std::vector<InternalActor*>& actorsList = ps->getInternalScene()->mMirorredActors;
            for (size_t i = 0; i < actorsList.size(); i++)
            {
                if (actorsList[i] == intActor)
                {
                    actorsList[i] = actorsList.back();
                    actorsList.pop_back();
                    break;
                }
            }
        }
        SAFE_DELETE_ALLOCABLE_SINGLE(intActor);
        attachedStage.removeAnimatedKinematicBody(removedKey);
        objectRecord.setRemoved();
    }
    break;
    case ePTForce:
    {
        InternalForce* intForce = (InternalForce*)objectRecord.mInternalPtr;
        PhysXScene* ps = intForce->mPhysXScene;

        std::vector<InternalForce*>& forces = ps->getInternalScene()->mForces;
        for (size_t i = 0; i < forces.size(); i++)
        {
            if (forces[i] == intForce)
            {
                forces[i] = forces.back();
                forces.pop_back();
                break;
            }
        }
        attachedStage.removeAnimatedKinematicBody(removedKey);
        SAFE_DELETE_ALLOCABLE_SINGLE(intForce);
        objectRecord.setRemoved();
    }
    break;
    case ePTFilteredPair:
    {
        InternalFilteredPairs* intPairs = (InternalFilteredPairs*)objectRecord.mInternalPtr;
        intPairs->removeFilteredPairs();
        SAFE_DELETE_ALLOCABLE_SINGLE(intPairs);
        objectRecord.setRemoved();
    }
    break;
    case ePTScene:
    {
        objectRecord.setRemoved();
    }
    break;
    case ePTMaterial:
    {
        PxMaterial* material = (PxMaterial*)objectRecord.mPtr;

        // release from default material on a scene
        const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            PhysXScene* sc = ref.second;
            if (sc->getDefaultMaterial() == material)
            {
                sc->resetDefaultMaterial();
            }
        }

        material->release();
        InternalMaterial* intMat = (InternalMaterial*)objectRecord.mInternalPtr;
        for (size_t i = 0; i < intMat->mShapeIds.size(); i++)
        {
            InternalDatabase::Record& shapeRecord = db.getRecords()[intMat->mShapeIds[i]];
            if (shapeRecord.mType == ePTShape)
            {
                InternalShape* intShape = (InternalShape*)shapeRecord.mInternalPtr;
                intShape->mMaterialId = kInvalidObjectId;
            }
            else if (shapeRecord.mType == ePTCompoundShape)
            {
                CompoundShape* shapePtr = (CompoundShape*)shapeRecord.mPtr;
                shapePtr->mMaterialId = kInvalidObjectId;

                InternalShape* intShape = (InternalShape*)shapeRecord.mInternalPtr;
                intShape->mMaterialId = kInvalidObjectId;
            }
        }
        SAFE_DELETE_ALLOCABLE_SINGLE(intMat);
        objectRecord.setRemoved();
    }
    break;
    case ePTCct:
    {
        PxController* cct = (PxController*)objectRecord.mPtr;
        InternalCct* intCct = (InternalCct*)objectRecord.mInternalPtr;
        PhysXScene* ps = intCct->mPhysXScene;
        ps->getInternalScene()->trackReleasedActiveActor(cct->getActor());
        cct->release();
        CctMap::iterator it = ps->getInternalScene()->mCctMap.begin();
        while (it != ps->getInternalScene()->mCctMap.end())
        {
            if (it->second == intCct)
            {
                ps->getInternalScene()->mCctMap.erase(it);
                break;
            }
            it++;
        }
        SAFE_DELETE_ALLOCABLE_SINGLE(intCct);
        objectRecord.setRemoved();
    }
    break;

    case ePTVehicle:
    {
        InternalVehicle* internalVehicle = static_cast<InternalVehicle*>(objectRecord.mInternalPtr);
        internalVehicle->mInternalScene.removeVehicle(*internalVehicle);

        SAFE_DELETE_ALLOCABLE_SINGLE(internalVehicle);
        objectRecord.setRemoved();
    }
    break;

    case ePTVehicleController:
    {
        // nothing to delete as the entry just points to InternalVehicle
        objectRecord.setRemoved();
    }
    break;

    case ePTVehicleEngine:
    {
        InternalVehicleReferenceList* vehicleRefList = static_cast<InternalVehicleReferenceList*>(objectRecord.mInternalPtr);

        vehicleRefList->unlinkFromVehicles(ePTVehicleEngine);

        SAFE_DELETE_ALLOCABLE_SINGLE(vehicleRefList);
        objectRecord.setRemoved();
    }
    break;

    case ePTVehicleTireFrictionTable:
    {
        CARB_LOG_ERROR("PhysX Vehicle: \"%s\": removing tire friction tables while playing is not permitted.\n",
            attachedStage.textFor(removedKey));
    }
    break;

    case ePTVehicleSuspension:
    case ePTVehicleTire:
    case ePTVehicleWheel:
    {
        InternalVehicleWheelReferenceList* vehicleWheelRefList = static_cast<InternalVehicleWheelReferenceList*>(objectRecord.mInternalPtr);

        SAFE_DELETE_ALLOCABLE_SINGLE(vehicleWheelRefList);
        objectRecord.setRemoved();
    }
    break;

    case ePTVehicleWheelAttachment:
    {
        InternalVehicleWheelAttachment* vehicleWheelAttachment = static_cast<InternalVehicleWheelAttachment*>(objectRecord.mInternalPtr);

        vehicleWheelAttachment->release(true);
        objectRecord.setRemoved();
    }
    break;

    case ePTVehicleWheelController:
    {
        // nothing to delete as the entry just points to InternalVehicleWheelAttachment
        objectRecord.setRemoved();
    }
    break;

    case ePTVehicleDriveBasic:
    {
        InternalVehicleReferenceList* vehicleRefList = static_cast<InternalVehicleReferenceList*>(objectRecord.mInternalPtr);

        vehicleRefList->unlinkFromVehicles(ePTVehicleDriveBasic);

        SAFE_DELETE_ALLOCABLE_SINGLE(vehicleRefList);
        objectRecord.setRemoved();
    }
    break;

    case ePTJoint:
    {
        PxJoint* joint = (PxJoint*)objectRecord.mPtr;
        db.removePxJoint(joint);
        // A.B. this could easily be a bottleneck.
        for (InternalDatabase::Record& jointRec : db.getRecords())
        {
            if (jointRec.mType == ePTJoint)
            {
                PxJoint* jr = (PxJoint*)jointRec.mPtr;
                if (jr && jr->getConcreteType() == ::physx::PxJointConcreteType::eRACK_AND_PINION)
                {
                    ::physx::PxRackAndPinionJoint* rpJoint = (::physx::PxRackAndPinionJoint*)jr;
                    const ::physx::PxBase* hingeJoint = nullptr;
                    const ::physx::PxBase* prismaticJoint = nullptr;
                    rpJoint->getJoints(hingeJoint, prismaticJoint);
                    if (joint == hingeJoint || joint == prismaticJoint)
                    {
                        db.removePxJoint(rpJoint);
                        rpJoint->release();
                        InternalJoint* intJoint = (InternalJoint*)jointRec.mInternalPtr;
                        SAFE_DELETE_ALLOCABLE_SINGLE(intJoint);
                        jointRec.setRemoved();
                    }
                }
                else if (jr && jr->getConcreteType() == ::physx::PxJointConcreteType::eGEAR)
                {
                    ::physx::PxGearJoint* gearJoint = (::physx::PxGearJoint*)jr;
                    const ::physx::PxBase* hinge0Joint = nullptr;
                    const ::physx::PxBase* hinge1Joint = nullptr;
                    gearJoint->getHinges(hinge0Joint, hinge1Joint);
                    if (joint == hinge0Joint || joint == hinge1Joint)
                    {
                        db.removePxJoint(gearJoint);
                        gearJoint->release();
                        InternalJoint* intJoint = (InternalJoint*)jointRec.mInternalPtr;
                        SAFE_DELETE_ALLOCABLE_SINGLE(intJoint);
                        jointRec.setRemoved();
                    }
                }
            }
        }
        joint->release();
        InternalJoint* intJoint = (InternalJoint*)objectRecord.mInternalPtr;
        SAFE_DELETE_ALLOCABLE_SINGLE(intJoint);
        objectRecord.setRemoved();
    }
    break;
    case ePTArticulation:
    {
        InternalArticulation* internalArticulation = (InternalArticulation*)objectRecord.mInternalPtr;
        PxArticulationReducedCoordinate* art = (PxArticulationReducedCoordinate*)objectRecord.mPtr;
        if (internalArticulation->mAggregate)
        {
            internalArticulation->mAggregate->release();
            internalArticulation->mAggregate = nullptr;
        }

        InternalScene* internalScene = internalArticulation->mPhysxScene->getInternalScene();
        {
            const PxU32 numLinks = art->getNbLinks();
            std::vector<PxArticulationLink*> links(numLinks);
            if (numLinks)
            {
                art->getLinks(links.data(), numLinks);
                for (PxArticulationLink* link : links)
                    internalScene->trackReleasedActiveActor(link);
            }
        }
        std::vector<::physx::PxArticulationReducedCoordinate*>& articulationsList = internalScene->mArticulations;
        for (size_t i = 0; i < articulationsList.size(); i++)
        {
            if (art == articulationsList[i])
            {
                articulationsList[i] = articulationsList.back();
                articulationsList.pop_back();
                break;
            }
        }

        if (art->getScene())
        {
            art->getScene()->removeArticulation(*art);
        }

        // if we released the articulation, we have to release the links first and tendons
        // fixed tendon
        {
            PxArticulationFixedTendon* tendon = nullptr;
            const PxU32 numTendons = art->getNbFixedTendons();
            for (PxU32 i = 0; i < numTendons; i++)
            {
                art->getFixedTendons(&tendon, 1, i);
                const size_t tendonIndex = (size_t)tendon->userData;
                if (tendonIndex < db.getRecords().size())
                {
                    InternalDatabase::Record& tendonRecord = db.getRecords()[tendonIndex];
                    if (tendonRecord.mType == ePTFixedTendonAxis)
                    {
                        releaseObject(attachedStage, tendonRecord.mKey, tendonIndex);
                    }
                }
            }
        }
        // spatial tendon
        {
            PxArticulationSpatialTendon* tendon = nullptr;
            const PxU32 numTendons = art->getNbSpatialTendons();
            for (PxU32 i = 0; i < numTendons; i++)
            {
                art->getSpatialTendons(&tendon, 1, i);
                const size_t tendonIndex = (size_t)tendon->userData;
                if (tendonIndex < db.getRecords().size())
                {
                    InternalDatabase::Record& tendonRecord = db.getRecords()[tendonIndex];
                    if (tendonRecord.mType == ePTTendonAttachment)
                    {
                        releaseObject(attachedStage, tendonRecord.mKey, tendonIndex);
                    }
                }
            }
        }
        // links
        {
            PxArticulationLink* link = nullptr;
            const PxU32 numLinks = art->getNbLinks();
            for (PxU32 i = 0; i < numLinks; i++)
            {
                art->getLinks(&link, 1, i);

                PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
                if (joint)
                {
                    const ObjectId jointIndex = (ObjectId)joint->userData;
                    if (jointIndex < db.getRecords().size())
                    {
                        InternalDatabase::Record& jointRecord = db.getRecords()[jointIndex];
                        if (jointRecord.mType == ePTLinkJoint)
                        {
                            jointRecord.mPtr = nullptr;
                            // marking the PhysX pointer as invalid to ensure future access
                            // of the record knows that the PhysX part has been removed
                        }
                    }

                    internalScene->releasePhysXMimicJoints(*joint);
                }

                const size_t linkIndex = (size_t)link->userData;
                if (linkIndex < db.getRecords().size())
                {
                    InternalDatabase::Record& linkRecord = db.getRecords()[linkIndex];
                    if (linkRecord.mType == ePTLink)
                    {
                        releaseObject(attachedStage, linkRecord.mKey, linkIndex);
                    }
                }
            }
        }

        for (size_t i = 0; i < mArticulations.size(); i++)
        {
            if (mArticulations[i] == (ObjectId)art->userData)
            {
                mArticulations[i] = mArticulations.back();
                mArticulations.pop_back();
                break;
            }
        }

        art->release();
        SAFE_DELETE_ALLOCABLE_SINGLE(internalArticulation);
        objectRecord.setRemoved();
    }
    break;
    case ePTLink:
    {
        PxArticulationLink* actor = (PxArticulationLink*)objectRecord.mPtr;
        InternalLink* intActor = (InternalLink*)objectRecord.mInternalPtr;
        PhysXScene* ps = intActor->mPhysXScene;
        InternalScene* intScene = ps->getInternalScene();

        PxArticulationJointReducedCoordinate* joint = actor->getInboundJoint();
        if (joint && intScene->hasMimicJoint(*joint))
        {
            removeArticulationFromSceneAndScheduleForReAdd(actor->getArticulation());
            intScene->releasePhysXMimicJoints(*joint);
        }

        OmniPhysX::getInstance().getRaycastManager().clearPicker(actor);
        ps->getContactReport()->removeActor(actor, removedKey);
        PxShape* shapePtr = nullptr;
        for (PxU32 i = 0; i < actor->getNbShapes(); i++)
        {
            actor->getShapes(&shapePtr, 1, i);
            removeFilteredObject<uint32_t>(internal::convertFilterPairFromPxFilterData(shapePtr->getSimulationFilterData()), physxSetup.getFilteredPairs());
        }
        if (!intScene->removeActor(*intActor))
        {
            // See the equivalent check on the rigid body removal path (NVBugs 6504495).
            CARB_LOG_ERROR("Removed articulation link %s was not registered with its own scene.",
                           attachedStage.textFor(removedKey));
        }
        db.removeDirtyMassActor(intActor);
        SAFE_DELETE_ALLOCABLE_SINGLE(intActor);
        objectRecord.setRemoved();
    }
    break;
    case ePTLinkJoint:
    {
        PxArticulationJointReducedCoordinate* pxJoint = (PxArticulationJointReducedCoordinate*)objectRecord.mPtr;
        if (pxJoint)  // the PhysX joint might have been deleted as part of an articulation root deletion
                      // and the joint prim is deleted later (or because the joint is on a descendant prim)
        {
            PxArticulationLink& pxLink = pxJoint->getChildArticulationLink();
            InternalActor* intLink = (InternalActor*)db.getInternalTypedRecord(ePTLink, (ObjectId)pxLink.userData);
            if (intLink)
            {
                PhysXScene* ps = intLink->mPhysXScene;
                InternalScene* intScene = ps->getInternalScene();
                if (intScene->hasMimicJoint(*pxJoint))
                {
                    removeArticulationFromSceneAndScheduleForReAdd(pxLink.getArticulation());
                    intScene->releasePhysXMimicJoints(*pxJoint);
                }
            }
        }

        InternalJoint* intJoint = static_cast<InternalJoint*>(objectRecord.mInternalPtr);
        SAFE_DELETE_ALLOCABLE_SINGLE(intJoint);
        objectRecord.setRemoved();
    }
    break;
    case ePTFixedTendonAxis:
    {
        InternalTendonAxis* intAxis = static_cast<InternalTendonAxis*>(objectRecord.mInternalPtr);
        SAFE_DELETE_ALLOCABLE_SINGLE(intAxis);
        objectRecord.setRemoved();
    }
    break;
    case ePTTendonAttachment:
    {
        InternalTendonAttachment* intAttachment = static_cast<InternalTendonAttachment*>(objectRecord.mInternalPtr);
        SAFE_DELETE_ALLOCABLE_SINGLE(intAttachment);
        objectRecord.setRemoved();
    }
    break;
    case ePTCollisionGroup:
    {
        removeFilteredObject<uint32_t>(uint32_t(objectId), physxSetup.getCollisionGroupFilteredPairs());
        objectRecord.setRemoved();

        CARB_LOG_ERROR("Physics USD: Removing a CollisionGroup while the simulation is running results in undefined behavior!");
    }
    break;

    case ePTParticleSystem:
    {
        InternalPbdParticleSystem* internalParticleSystem = (InternalPbdParticleSystem*)objectRecord.mInternalPtr;
        if (internalParticleSystem)
        {
            // remove from internalScene and release
            InternalScene& internalScene = *internalParticleSystem->mPhysXScene->getInternalScene();
            auto it = std::find(internalScene.mParticleSystems.begin(), internalScene.mParticleSystems.end(), internalParticleSystem);
            if (it != internalScene.mParticleSystems.end())
            {
                std::swap(*it, internalScene.mParticleSystems.back());
                internalScene.mParticleSystems.pop_back();
            }
            internalParticleSystem->removeParticleObjectsFromDB();
            SAFE_DELETE_ALLOCABLE_SINGLE(internalParticleSystem);
        }
        objectRecord.setRemoved();
    }
    break;

    case ePTParticleSet:
    {
        InternalParticleSet* internalParticleSet = (InternalParticleSet*)objectRecord.mInternalPtr;
        if (internalParticleSet)
        {
            if (internalParticleSet->mParentParticleSystem)
            {
                internalParticleSet->mParentParticleSystem->removeParticleSet(internalParticleSet);
            }
            SAFE_DELETE_ALLOCABLE_SINGLE(internalParticleSet);
        }
        objectRecord.setRemoved();
    }
    break;
    case ePTDeformableSurface:
    {
        InternalSurfaceDeformableBody* intActor = (InternalSurfaceDeformableBody*)objectRecord.mInternalPtr;
        PhysXScene* ps = intActor->mPhysXScene;
        InternalScene* internalScene = ps->getInternalScene();
        const uint64_t removedPrimAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(removedKey);
        const bool isRoot = (removedPrimAPIs & SchemaAPIFlag::eDeformableBodyAPI) > 0;

        // Only process the root node, not sub objects
        if (intActor && isRoot)
        {
            // Remove from mSurfaceDeformableBodies
            auto it = std::find(internalScene->mSurfaceDeformableBodies.begin(), internalScene->mSurfaceDeformableBodies.end(), intActor);
            if (it != internalScene->mSurfaceDeformableBodies.end())
            {
                // Remove skinning data before the surface deformable body is released
                internalScene->mSurfaceDeformablePostSolveCallback->removeSurfaceDeformableSkinningData((*it)->mDeformableSurface);

                *it = internalScene->mSurfaceDeformableBodies.back();
                internalScene->mSurfaceDeformableBodies.pop_back();
            }

            // Remove material reference
            if (intActor->mMaterialId != kInvalidObjectId)
            {
                InternalDatabase::Record& materialRecord = OmniPhysX::getInstance().getInternalPhysXDatabase().getRecords()[intActor->mMaterialId];
                InternalDeformableMaterial* mat = (InternalDeformableMaterial*)materialRecord.mInternalPtr;
                if (mat)
                {
                    mat->removeDeformableId(objectId);
                }
            }

            // Remove deformable attachments and collision filters
            internalScene->removeDeformableAttachments(objectId);
            internalScene->removeDeformableCollisionFilters(objectId);

            // Delete instance
            SAFE_DELETE_ALLOCABLE_SINGLE(intActor);
        }

        objectRecord.setRemoved();
    }
    break;
    case ePTDeformableVolume:
    {
        InternalVolumeDeformableBody* intActor = (InternalVolumeDeformableBody*)objectRecord.mInternalPtr;
        PhysXScene* ps = intActor->mPhysXScene;
        InternalScene* internalScene = ps->getInternalScene();
        const uint64_t removedPrimAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(removedKey);
        const bool isRoot = (removedPrimAPIs & SchemaAPIFlag::eDeformableBodyAPI) > 0;

        // Only process the root node, not sub objects
        if (intActor && isRoot)
        {
            // Remove from mVolumeDeformableBodies
            auto it = std::find(internalScene->mVolumeDeformableBodies.begin(), internalScene->mVolumeDeformableBodies.end(), intActor);
            if (it != internalScene->mVolumeDeformableBodies.end())
            {
                // Remove skinning data before the volume deformable body is released
                internalScene->mVolumeDeformablePostSolveCallback->removeVolumeDeformableSkinningData((*it)->mDeformableVolume);

                *it = internalScene->mVolumeDeformableBodies.back();
                internalScene->mVolumeDeformableBodies.pop_back();
            }

            // Remove material reference
            if (intActor->mMaterialId != kInvalidObjectId)
            {
                InternalDatabase::Record& materialRecord = OmniPhysX::getInstance().getInternalPhysXDatabase().getRecords()[intActor->mMaterialId];
                InternalDeformableMaterial* mat = (InternalDeformableMaterial*)materialRecord.mInternalPtr;
                if (mat)
                {
                    mat->removeDeformableId(objectId);
                }
            }

            // Remove deformable attachments and collision filters
            internalScene->removeDeformableAttachments(objectId);
            internalScene->removeDeformableCollisionFilters(objectId);

            // Delete instance
            SAFE_DELETE_ALLOCABLE_SINGLE(intActor);
        }

        objectRecord.setRemoved();
    }
    break;

    case ePTDeformableAttachment:
    {
        InternalDeformableAttachment* internalDeformableAttachment = (InternalDeformableAttachment*)objectRecord.mInternalPtr;

        const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            InternalScene* internalScene = ref.second->getInternalScene();
            if (internalScene->removeDeformableAttachment(*internalDeformableAttachment))
                break;
        }

        objectRecord.setRemoved();
    }
    break;
    case ePTDeformableCollisionFilter:
    {
        InternalDeformableCollisionFilter* internalDeformableCollisionFilter = (InternalDeformableCollisionFilter*)objectRecord.mInternalPtr;

        const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            InternalScene* internalScene = ref.second->getInternalScene();
            if (internalScene->removeDeformableCollisionFilter(*internalDeformableCollisionFilter))
                break;
        }

        objectRecord.setRemoved();
    }
    break;
    case ePTXformActor:
    {
        const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            InternalScene* internalScene = ref.second->getInternalScene();

            // Remove deformable attachments and collision filters
            internalScene->removeDeformableAttachments(objectId);
            internalScene->removeDeformableCollisionFilters(objectId);
        }

        objectRecord.setRemoved();
    }
    break;

    case ePTMimicJoint:
    {
        InternalMimicJoint* internalMimicJoint = (InternalMimicJoint*)objectRecord.mInternalPtr;

        if (internalMimicJoint)
        {
            PxArticulationReducedCoordinate* articulation = internalMimicJoint->getArticulation();

            // remove the articulation from the scene since mimic joints can only be removed if
            // not part of a scene

            if (articulation)
            {
                removeArticulationFromSceneAndScheduleForReAdd(*articulation);
            }

            constexpr bool removeFromTrackers = true;
            constexpr bool releasePhysXObject = true;
            internalMimicJoint->release(removeFromTrackers, releasePhysXObject);
        }
        objectRecord.setRemoved();
    }
    break;
    }
}

void PhysXUsdPhysicsInterface::fillChangeParams(std::vector<ChangeParams>& changeParams)
{
    registerChangeParams(changeParams);
}

inline bool identicalPose(const float* data, const carb::Float3& pos, const carb::Float4& orient)
{
    const float epsylon = 1e-3f;
    if (fabsf(data[0] - pos.x) > epsylon || fabsf(data[1] - pos.y) > epsylon || fabsf(data[2] - pos.z) > epsylon)
        return false;

    const float dot = (data[3] * orient.x) + (data[4] * orient.y) + (data[5] * orient.z) + (data[6] * orient.w);
    if (fabsf(dot) < (1.0f - epsylon))
        return false;

    return true;
}

bool PhysXUsdPhysicsInterface::isReady(void)
{
    bool ret = true;
    CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
    if (cookingDataAsync)
    {
        if (cookingDataAsync->pump())
        {
            ret = false;
        }
    }
    return ret;
}

::physx::PxMat44d getSourceGPrimTransform(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    // Source gprim transforms are read from default-time authored state.
    return getWorldTransform(attachedStage, key, omni::physics::parse::ReadTime::defaultTime());
}

bool PhysXUsdPhysicsInterface::updateTransform(const AttachedStage& attachedStage,
                                               omni::physics::parse::ObjectKey key,
                                               ObjectId objectId,
                                               const Transform& transform,
                                               bool resetVelocity,
                                               bool scaleProvided)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    PhysXType internalType;
    const InternalDatabase::Record* fullRecord = db.getFullRecord(internalType, objectId);
    if (!fullRecord)
        return true;

    void* objectRecord = fullRecord->mPtr;
    

    PxVec3 translation = toPhysX(transform.position);
    PxQuat orientation = toPhysXQuat(transform.orientation);
    PxVec3 scale = toPhysX(transform.scale);
    if (internalType == ePTActor)
    {
        PxRigidActor* actor = (PxRigidActor*)objectRecord;
        if (actor->getScene() && actor->getScene()->getFlags().isSet(::physx::PxSceneFlag::eENABLE_DIRECT_GPU_API))
            return true;
        InternalDatabase::Record& actorRec = db.getRecords()[objectId];
        InternalActor* intActor = (InternalActor*)actorRec.mInternalPtr;
        const float scaleEpsilon = 1e-3f;

        PxRigidDynamic* dynamicActor = actor->is<PxRigidDynamic>();
        PxTransform pose(translation, orientation);
        if (dynamicActor && (dynamicActor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
        {
            dynamicActor->setKinematicTarget(pose);
        }
        else
        {
            if (!dynamicActor && intActor->mSourceGPrimKey.valid())
            {
                // Instanced colliders dedup their descriptor from the shared USD
                // prototype, so mSourceGPrimKey can be inside it — a key with no
                // live instance context. Fall back to this actor's own key, which
                // resolves through instancing correctly.
                omni::physics::parse::ObjectKey sourceGPrimKey = intActor->mSourceGPrimKey;
                const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
                if (src && src->isInPrototype(sourceGPrimKey))
                    sourceGPrimKey = key;

                // We need to get the transform from the actual gprim
                PxTransform gprimPose;
                // `scale` stays signed, as GfTransform::GetScale() was.
                decomposeMatrix(gprimPose, scale, getSourceGPrimTransform(attachedStage, sourceGPrimKey));

                translation = gprimPose.p;
                orientation = gprimPose.q;

                pose.p = translation;
                pose.q = orientation;
            }
            actor->setGlobalPose(pose);
            if (resetVelocity && dynamicActor)
            {
                dynamicActor->setLinearVelocity(PxVec3(0.0f));
                dynamicActor->setAngularVelocity(PxVec3(0.0f));
            }

            if (!dynamicActor)
            {
                // A.B. would this not cause slowdown easily?
                std::vector<class InternalDeformableAttachment*>& attachmentList = intActor->mPhysXScene->getInternalScene()->mDeformableAttachments;
                for (size_t i = 0; i < attachmentList.size(); ++i)
                {
                    attachmentList[i]->setRefreshAttachmentEvent(actor);
                }
            }
        }

        // update mirrors
        for (const MirrorActor& mirror : intActor->mMirrors)
        {
            mirror.actor->setGlobalPose(pose);

            if (!dynamicActor)
            {
                PxScene* scene = mirror.actor->getScene();
                InternalDatabase::Record& actorRec = db.getRecords()[(ObjectId)scene->userData];
                InternalScene* intScene = (InternalScene*)actorRec.mInternalPtr;

                std::vector<class InternalDeformableAttachment*>& attachmentList = intScene->mDeformableAttachments;
                for (size_t i = 0; i < attachmentList.size(); ++i)
                {
                    attachmentList[i]->setRefreshAttachmentEvent(actor);
                }
            }
        }

        if (scaleProvided)
        {
            if ((fabsf(scale.x - intActor->mScale.x) > scaleEpsilon) ||
                (fabsf(scale.y - intActor->mScale.y) > scaleEpsilon) ||
                (fabsf(scale.z - intActor->mScale.z) > scaleEpsilon))
            {
                // A.B. if scale changed we need to reconstruct
                return false;
            }
        }
    }
    else if (internalType == ePTLink)
    {
        PxArticulationLink* actor = (PxArticulationLink*)objectRecord;
        if (actor->getLinkIndex())
        {
            return true;
        }
        else
        {
            if (actor->getScene() && actor->getScene()->getFlags().isSet(::physx::PxSceneFlag::eENABLE_DIRECT_GPU_API))
                return true;
            const PxTransform pose(translation, orientation);
            PxArticulationReducedCoordinate& articulation = actor->getArticulation();
            articulation.setRootGlobalPose(pose, true);
            return true;
        }
    }
    else if (internalType == ePTArticulation)
    {
        return true;
    }
    else if (internalType == ePTForce)
    {
        InternalForce* force = reinterpret_cast<InternalForce*> (fullRecord->mInternalPtr);
        if (force->mRigidActor)
        {
            const PxTransform bodyTransform = force->mRigidActor->getGlobalPose();
            force->mLocalPos = bodyTransform.transformInv(translation);
            if (force->mBodyPrimDifferent)
            {
                force->mLocalRot = bodyTransform.q.getConjugate() * orientation;
            }
        }
        return true;
    }
    else if (internalType == ePTParticleSet)
    {
        InternalDatabase::Record& record = db.getRecords()[(size_t)objectId];
        InternalParticleSet* internalParticleSet = (InternalParticleSet*)record.mInternalPtr;

        // get geometry transform
        const ::physx::PxMat44d localToWorld =
            getWorldTransform(attachedStage, internalParticleSet->mKey, omni::physics::parse::ReadTime::defaultTime());
        const ::physx::PxMat44d worldToLocalOld = internalParticleSet->mWorldToLocal;

        // Gf `localToWorld * worldToLocalOld` is PhysX `worldToLocalOld * localToWorld`
        // (same matrices, operands swap -- see MatrixTools.h).
        const ::physx::PxMat44d deltaTransform = worldToLocalOld * localToWorld;

        for (uint32_t index = 0; index < internalParticleSet->mNumParticles; ++index)
        {
            const PxVec4& oldPos = internalParticleSet->mPositions[index];
            const ::physx::PxVec3d newPos =
                deltaTransform.transform(::physx::PxVec3d(oldPos.x, oldPos.y, oldPos.z));
            internalParticleSet->mPositions[index] =
                PxVec4(float(newPos.x), float(newPos.y), float(newPos.z), oldPos.w);
        }

        internalParticleSet->mUploadDirtyFlags |= ParticleBufferFlags::ePOSITIONS;
        internalParticleSet->mWorldToLocal = omni::physx::affineInverse(localToWorld);
    }
    else if (internalType == ePTCct)
    {
        PxController* cct = (PxController*)objectRecord;
        InternalDatabase::Record& cctRec = db.getRecords()[(size_t)objectId];
        InternalCct* intCct = (InternalCct*)cctRec.mInternalPtr;
        const float scaleEpsilon = 1e-3f;

        if (scaleProvided)
        {
            if ((fabsf(scale.x - intCct->mScale.x) > scaleEpsilon) ||
                (fabsf(scale.y - intCct->mScale.y) > scaleEpsilon) ||
                (fabsf(scale.z - intCct->mScale.z) > scaleEpsilon))
            {
                // A.B. if scale changed we need to reconstruct
                return false;
            }
        }

        cct->setPosition(PxExtendedVec3(translation.x, translation.y, translation.z));
    }
    else if (internalType == ePTXformActor)
    {
        const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            InternalScene* internalScene = ref.second->getInternalScene();

            std::vector<class InternalDeformableAttachment*>& attachmentList = internalScene->mDeformableAttachments;
            for (size_t i = 0; i < attachmentList.size(); ++i)
            {
                attachmentList[i]->setUpdateXformEvent(objectId);
            }
        }
    }

    return true;
}

// getJoint / updateDrivePerformanceEnvelope / updateJointAxis: reachable only from
// changeSchemaAPI below.
PxArticulationJointReducedCoordinate* getJoint(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, SchemaAPIFlag::Enum flag)
{
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    ObjectId jointId = objectDb->findEntry(key, ObjectType::eArticulationJoint);
    if (jointId == kInvalidObjectId)
        return NULL;

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    const InternalDatabase::Record* objectRecord = db.getFullTypedRecord(PhysXType::ePTLinkJoint, jointId);
    if (!objectRecord)
        return NULL;
    PxArticulationJointReducedCoordinate* pxjoint = reinterpret_cast<PxArticulationJointReducedCoordinate*>(objectRecord->mPtr);
    return pxjoint;
}

void updateDrivePerformanceEnvelope
(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, const char* axisName,
 omni::physics::parse::TokenId driveApiInstanceToken, omni::physics::parse::TokenId driveMaxForceToken,
 const SchemaAPIFlag::Enum flag, const PxArticulationAxis::Enum pxAxis,
 omni::physics::parse::TokenId maxActuatorVelocityToken, omni::physics::parse::TokenId velocityDependentResistanceToken,
 omni::physics::parse::TokenId envelopeSpeedEffortGradientToken,
 const bool removed)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (source && source->hasSchema(key, driveApiInstanceToken))
    {
        PxArticulationJointReducedCoordinate* pxjoint = getJoint(attachedStage, key, flag);

        float driveMaxForce = -1.0f;
        getValue<float>(attachedStage, key, driveMaxForceToken, omni::physics::parse::ReadTime::defaultTime(), driveMaxForce);

        if(pxjoint)
        {
            PxArticulationDrive	driveParams = pxjoint->getDriveParams(pxAxis);

            ObjectDb* objectDb = attachedStage.getObjectDatabase();

            if(removed)
            {
                driveParams.maxForce = driveMaxForce;
                driveParams.envelope.maxActuatorVelocity = 0.0f;
                driveParams.envelope.maxEffort = 0.0f;
                driveParams.envelope.speedEffortGradient = 0.0f;
                driveParams.envelope.velocityDependentResistance = 0.0f;

                objectDb->removeSchemaAPI(key, flag);
            }
            else
            {
                driveParams.envelope.maxEffort = driveMaxForce;
                getValue<float>(attachedStage, key, maxActuatorVelocityToken, omni::physics::parse::ReadTime::defaultTime(), driveParams.envelope.maxActuatorVelocity);
                getValue<float>(attachedStage, key, velocityDependentResistanceToken, omni::physics::parse::ReadTime::defaultTime(), driveParams.envelope.velocityDependentResistance);
                getValue<float>(attachedStage, key, envelopeSpeedEffortGradientToken, omni::physics::parse::ReadTime::defaultTime(), driveParams.envelope.speedEffortGradient);

                objectDb->addSchemaAPI(key, flag);
            }

            if(PxArticulationAxis::eTWIST == pxAxis || PxArticulationAxis::eSWING1 == pxAxis || PxArticulationAxis::eSWING2 == pxAxis)
            {
                driveParams.envelope.maxActuatorVelocity = degToRad(driveParams.envelope.maxActuatorVelocity);
                driveParams.envelope.velocityDependentResistance = radToDeg(driveParams.envelope.velocityDependentResistance);
                driveParams.envelope.speedEffortGradient = degToRad(driveParams.envelope.speedEffortGradient);
            }

            pxjoint->setDriveParams(pxAxis, driveParams);
        }
        else if (!removed)
        {
            removeAppliedAPI(attachedStage, key, "PhysxDrivePerformanceEnvelopeAPI", axisName);
            CARB_LOG_WARN("Please ensure that the joint is part of an articulation. Performance envelope will be ignored for joint: %s", attachedStage.textFor(key));
        }
    }
}

void updateJointAxis
(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, const char* axisName,
 const SchemaAPIFlag::Enum flag, const PxArticulationAxis::Enum pxAxis,
 omni::physics::parse::TokenId armatureToken, omni::physics::parse::TokenId maxJointVelocityToken,
 omni::physics::parse::TokenId staticFrictionToken, omni::physics::parse::TokenId dynamicFrictionToken,
 omni::physics::parse::TokenId viscousFrictionToken,
 const bool removed)
{
    PxArticulationJointReducedCoordinate* pxjoint = getJoint(attachedStage, key, flag);

    if(pxjoint)
    {
        ObjectDb* objectDb = attachedStage.getObjectDatabase();

        float armature = 0.0f;
        float maxJointVelocity = PX_MAX_F32;
        float frictionCoefficient = 0.0f;
        float staticFriction = 0.0f;
        float dynamicFriction = 0.0f;
        float viscousFriction = 0.0f;

        if(removed)
        {
            //Get armature and joint velocity from PhysxSchemaPhysxJointAPI
            const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
            const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
            if(src && src->hasSchema(key, tok.physxJointAPI))
            {
                getValue(attachedStage, key, tok.physxJointArmature, omni::physics::parse::ReadTime::defaultTime(), armature);
                getValue(attachedStage, key, tok.physxJointMaxJointVelocity, omni::physics::parse::ReadTime::defaultTime(), maxJointVelocity);
                getValue(attachedStage, key, tok.physxJointJointFriction, omni::physics::parse::ReadTime::defaultTime(), frictionCoefficient);
            }

            objectDb->removeSchemaAPI(key, flag);
        }
        else
        {
            getValue<float>(attachedStage, key, armatureToken, omni::physics::parse::ReadTime::defaultTime(), armature);
            getValue<float>(attachedStage, key, maxJointVelocityToken, omni::physics::parse::ReadTime::defaultTime(), maxJointVelocity);
            getValue<float>(attachedStage, key, staticFrictionToken, omni::physics::parse::ReadTime::defaultTime(), staticFriction);
            getValue<float>(attachedStage, key, dynamicFrictionToken, omni::physics::parse::ReadTime::defaultTime(), dynamicFriction);
            getValue<float>(attachedStage, key, viscousFrictionToken, omni::physics::parse::ReadTime::defaultTime(), viscousFriction);

            objectDb->addSchemaAPI(key, flag);
        }

        if(PxArticulationAxis::eTWIST == pxAxis || PxArticulationAxis::eSWING1 == pxAxis || PxArticulationAxis::eSWING2 == pxAxis)
        {
            maxJointVelocity = degToRad(maxJointVelocity);
            viscousFriction = radToDeg(viscousFriction);
        }

        pxjoint->setArmature(pxAxis, armature);
        pxjoint->setMaxJointVelocity(pxAxis, maxJointVelocity);
        pxjoint->setFrictionCoefficient(frictionCoefficient);
        pxjoint->setFrictionParams(pxAxis, PxJointFrictionParams(staticFriction, dynamicFriction, viscousFriction));
    }
    else if (!removed)
    {
        removeAppliedAPI(attachedStage, key, "PhysxJointAxisAPI", axisName);
        CARB_LOG_WARN("Please ensure that the joint is part of an articulation. Properties from PhysxJointAxisAPI will be ignored for joint: %s", attachedStage.textFor(key));
    }
}

void PhysXUsdPhysicsInterface::changeSchemaAPI(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, SchemaAPIFlag::Enum flag, bool removed)
{
    // Note: only non-structural changes are allowed to be handled in this function.

    const omni::physics::parse::IPhysicsSource* changeSchemaAPISrc = attachedStage.getSource();
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();

    switch (flag)
    {
        case SchemaAPIFlag::eParticleIsosurfaceAPI:
        case SchemaAPIFlag::eParticleAnisotropyAPI:
        case SchemaAPIFlag::eParticleSmoothingAPI:
        {
            changeParticlePostProcess(attachedStage, key, removed, flag);
        }
        break;

        case SchemaAPIFlag::eDiffuseParticlesAPI:
        {
            changeParticleDiffuseParticles(attachedStage, key, removed);
        }
        break;

        case SchemaAPIFlag::eFilteredPairsAPI:
        {
            changeFilteredPairs(attachedStage, key, removed);
        }
        break;

        case SchemaAPIFlag::eContactReportAPI:
        {
            changeContactReport(attachedStage, key, removed);
        }
        break;

        case SchemaAPIFlag::eMimicJointRotXAPI:
        case SchemaAPIFlag::eMimicJointRotYAPI:
        case SchemaAPIFlag::eMimicJointRotZAPI:
        case SchemaAPIFlag::eNewtonMimicAPI:
        {
            if (removed)
                releaseMimicJoint(attachedStage, key, flag);
        }
        break;
       case SchemaAPIFlag::eDrivePerformanceEnvelopeAngularAPI:
            updateDrivePerformanceEnvelope(
                attachedStage, key, "angular",
                tok.physicsDriveAPIAngular, tok.driveMaxForceAngular, flag, PxArticulationAxis::eTWIST,
                tok.maxActuatorVelocityAngular,
                tok.velocityDependentResistanceAngular,
                tok.speedEffortGradientAngular,
                removed);
       break;
       case SchemaAPIFlag::eDrivePerformanceEnvelopeLinearAPI:
            updateDrivePerformanceEnvelope(
                attachedStage, key, "linear",
                tok.physicsDriveAPILinear, tok.driveMaxForceLinear, flag, PxArticulationAxis::eX,
                tok.maxActuatorVelocityLinear,
                tok.velocityDependentResistanceLinear,
                tok.speedEffortGradientLinear,
                removed);
       break;
       case SchemaAPIFlag::eDrivePerformanceEnvelopeRotXAPI:
            updateDrivePerformanceEnvelope(
                attachedStage, key, "rotX",
                tok.physicsDriveAPIRotX, tok.driveMaxForceRotX, flag, PxArticulationAxis::eTWIST,
                tok.maxActuatorVelocityRotX,
                tok.velocityDependentResistanceRotX,
                tok.speedEffortGradientRotX,
                removed);
       break;
       case SchemaAPIFlag::eDrivePerformanceEnvelopeRotYAPI:
            updateDrivePerformanceEnvelope(
                attachedStage, key, "rotY",
                tok.physicsDriveAPIRotY, tok.driveMaxForceRotY, flag, PxArticulationAxis::eSWING1,
                tok.maxActuatorVelocityRotY,
                tok.velocityDependentResistanceRotY,
                tok.speedEffortGradientRotY,
                removed);
       break;
       case SchemaAPIFlag::eDrivePerformanceEnvelopeRotZAPI:
            updateDrivePerformanceEnvelope(
                attachedStage, key, "rotZ",
                tok.physicsDriveAPIRotZ, tok.driveMaxForceRotZ, flag, PxArticulationAxis::eSWING2,
                tok.maxActuatorVelocityRotZ,
                tok.velocityDependentResistanceRotZ,
                tok.speedEffortGradientRotZ,
                removed);
       break;
       case SchemaAPIFlag::eJointAxisAngularAPI:
            updateJointAxis(
                attachedStage, key, "angular", flag, PxArticulationAxis::eTWIST,
                tok.armatureAngular,
                tok.maxJointVelocityAngular,
                tok.staticFrictionEffortAngular,
                tok.dynamicFrictionEffortAngular,
                tok.viscousFrictionCoefficientAngular,
                removed);
       break;
       case SchemaAPIFlag::eJointAxisLinearAPI:
            updateJointAxis(
                attachedStage, key, "linear", flag, PxArticulationAxis::eX,
                tok.armatureLinear,
                tok.maxJointVelocityLinear,
                tok.staticFrictionEffortLinear,
                tok.dynamicFrictionEffortLinear,
                tok.viscousFrictionCoefficientLinear,
                removed);
       break;
       case SchemaAPIFlag::eJointAxisRotXAPI:
            updateJointAxis(
                attachedStage, key, "rotX", flag, PxArticulationAxis::eTWIST,
                tok.armatureRotX,
                tok.maxJointVelocityRotX,
                tok.staticFrictionEffortRotX,
                tok.dynamicFrictionEffortRotX,
                tok.viscousFrictionCoefficientRotX,
                removed);
       break;
       case SchemaAPIFlag::eJointAxisRotYAPI:
            updateJointAxis(
                attachedStage, key, "rotY", flag, PxArticulationAxis::eSWING1,
                tok.armatureRotY,
                tok.maxJointVelocityRotY,
                tok.staticFrictionEffortRotY,
                tok.dynamicFrictionEffortRotY,
                tok.viscousFrictionCoefficientRotY,
                removed);
       break;
       case SchemaAPIFlag::eJointAxisRotZAPI:
            updateJointAxis(
                attachedStage, key, "rotZ", flag, PxArticulationAxis::eSWING2,
                tok.armatureRotZ,
                tok.maxJointVelocityRotZ,
                tok.staticFrictionEffortRotZ,
                tok.dynamicFrictionEffortRotZ,
                tok.viscousFrictionCoefficientRotZ,
                removed);
       break;
       default:
       break;
    }
}

bool isNonDynamicMesh(PxShape* shape)
{
    const PxGeometry& geom = shape->getGeometry();
    if (geom.getType() == PxGeometryType::eTRIANGLEMESH)
    {
        const PxTriangleMeshGeometry& trigeom = static_cast<const PxTriangleMeshGeometry&>(geom);
        CARB_ASSERT(trigeom.triangleMesh);
        if(trigeom.triangleMesh)
        {
            return trigeom.triangleMesh->getSDF() == NULL;
        }
    }
    return false;
}

bool hasMeshShape(const PxRigidActor& actor)
{
    PxShape* shape = nullptr;
    for (PxU32 i = 0; i < actor.getNbShapes(); i++)
    {
        actor.getShapes(&shape, 1, i);
        if (isNonDynamicMesh(shape) || shape->getGeometry().getType() == PxGeometryType::eHEIGHTFIELD)
            return true;
    }
    return false;
}

bool PhysXUsdPhysicsInterface::updateMass(omni::physics::parse::ObjectKey key,
                                          ObjectId objectId,
                                          float mass,
                                          const carb::Float3& diagInertia,
                                          const carb::Float3& com,
                                          const carb::Float4& principalAxes)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);
    if (!objectFullRecord || !objectFullRecord->mPtr)
        return true;

    void* objectRecord = objectFullRecord->mPtr;

    const PxVec3 diagIn(diagInertia.x, diagInertia.y, diagInertia.z);
    const PxTransform comTr = toPhysX(com, principalAxes);

    if (internalType == ePTActor)
    {
        PxRigidActor* actor = (PxRigidActor*)objectRecord;
        InternalActor* intActor = (InternalActor*)objectFullRecord->mInternalPtr;
        if (actor->is<PxRigidDynamic>() && !hasMeshShape(*actor))
        {
            PxRigidDynamic* rigidDynamic = actor->is<PxRigidDynamic>();
            if (mass > 0.0f)
            {
                // check if the body is part of a vehicle and make adjustments if so.
                // Note: has to be done before the new parameters are set on the body
                intActor->mPhysXScene->getInternalScene()->updateVehicleOnMassChange(*rigidDynamic, mass, diagIn, comTr);

                rigidDynamic->setMass(mass);
                rigidDynamic->setMassSpaceInertiaTensor(diagIn);
                rigidDynamic->setCMassLocalPose(comTr);
            }
        }
    }
    if (internalType == ePTLink)
    {
        PxArticulationLink* actor = (PxArticulationLink*)objectRecord;
        if (mass > 0.0f)
        {
            actor->setMass(mass);
            actor->setMassSpaceInertiaTensor(diagIn);
            actor->setCMassLocalPose(comTr);
        }
    }
    return true;
}

bool PhysXUsdPhysicsInterface::updateObject(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key,
                                            ObjectId objectId,
                                            OnUpdateObjectFn updateFn,
                                            omni::physics::parse::TokenId propertyName,
                                            omni::physics::parse::ReadTime timeCode)
{
    return updateFn(attachedStage, objectId, propertyName, timeCode);
}

void PhysXUsdPhysicsInterface::releaseAllObjects()
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    // If we are in physics authoring mode and a scene reset occurs, then we need to
    // notify it to release any handles it has active to actors

    if (!mPhysicsObjectChangeSubscriptions.map.empty())
    {
        PhysicsObjectChangeSubscriptionRegistry::EventMap::const_iterator it = mPhysicsObjectChangeSubscriptions.map.begin();
        PhysicsObjectChangeSubscriptionRegistry::EventMap::const_iterator itEnd = mPhysicsObjectChangeSubscriptions.map.end();
        while (it != itEnd)
        {
            const IPhysicsObjectChangeCallback& callback = it->second;

            if (shouldDeliverObjectChangeNotification(callback))
            {
                if (callback.allObjectsDestructionNotifyFn)
                {
                    try
                    {
                        callback.allObjectsDestructionNotifyFn(callback.userData);
                    }
                    catch (const std::exception& error)
                    {
                        CARB_LOG_ERROR("Physics object destruction callback failed: %s", error.what());
                    }
                    catch (...)
                    {
                        CARB_LOG_ERROR("Physics object destruction callback failed with an unknown exception");
                    }
                }
            }
            it++;
        }
    }

    std::vector<InternalDatabase::Record>& records = db.getRecords();
    for (size_t i = 0; i < records.size(); i++)
    {
        InternalDatabase::Record& rec = records[i];
        if (rec.mType == ePTCompoundShape || rec.mType == ePTCollisionGroup)
        {
            delete rec.mPtr;
            rec.mPtr = nullptr;
        }
    }

    gTempParent.clear();

    clearFiltering();

    OmniPhysX::getInstance().getTriggerManager()->clearTriggers();

    // clear mArticulations so stale articulations won't be added in the next usd open with finishSetup
    mArticulations.clear();

    mParticleSystems.clear();

    OmniPhysX::getInstance().releasePhysXScenes();
}

static void addArticulationToScene(const usdparser::AttachedStage& attachedStage, InternalDatabase::Record& articulationRecord)
{
    InternalArticulation* intArt = (InternalArticulation*)articulationRecord.mInternalPtr;
    PxArticulationReducedCoordinate* art = (PxArticulationReducedCoordinate*)articulationRecord.mPtr;
    PhysXScene* ps = intArt->mPhysxScene;

    // only create aggregate if it does not exist already
    if (!intArt->mAggregate)
    {
        const PxU32 nbLinks = intArt->mStaticRootBodyKey.valid() ? art->getNbLinks() + 1 : art->getNbLinks();

        PxArticulationLink* link = nullptr;
        PxU32 numShapes = 0;
        for (PxU32 linkIndex = 0; linkIndex < art->getNbLinks(); linkIndex++)
        {
            art->getLinks(&link, 1, linkIndex);
            numShapes += link->getNbShapes();
        }
        // Just to be consistent with previous releases
        numShapes = numShapes < 1024 ? 1024 : numShapes;
        
        intArt->mAggregate = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createAggregate(
            nbLinks,
            numShapes,
            PxGetAggregateFilterHint(PxAggregateType::eGENERIC, intArt->mEnableSelfCollision)
        );
        intArt->mAggregate->addArticulation(*art);

        if (attachedStage.isUsingReplicatorEnvIds())
        {
            // setup envId for the aggregate check for the prim var
            uint32_t partition_id = 0;
            // Read (and look up) through the TokenId path, not TfToken -- see the matching
            // comment on the dynamic-body env-id lookup above.
            if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
            {
                const omni::physics::parse::TokenId scenePartitionAttr = source->internToken(kScenePartitionPrimvar);
                omni::physics::parse::TokenId scenePartitionToken;
                if (getValue<omni::physics::parse::TokenId>(attachedStage, articulationRecord.mKey,
                                                             scenePartitionAttr, omni::physics::parse::ReadTime::defaultTime(),
                                                             scenePartitionToken))
                {
                    partition_id = attachedStage.getEnvIdFromToken(scenePartitionToken);
                }
            }
            intArt->mAggregate->setEnvironmentID(partition_id);
        }

        if (intArt->mStaticRootBodyKey.valid())
        {
            PxBase* physxPtr = reinterpret_cast<PxBase*>(getPhysXPtr(intArt->mStaticRootBodyKey, ePTActor));
            if (physxPtr)
            {
                PxRigidStatic* staticBody = physxPtr->is<PxRigidStatic>();
                if (staticBody && staticBody->getScene() == ps->getScene())
                {
                    ps->getScene()->removeActor(*staticBody);
                    intArt->mAggregate->addActor(*staticBody);
                }
            }
        }
    }
    CARB_ASSERT(intArt->mAggregate == art->getAggregate());

    ps->getScene()->addAggregate(*intArt->mAggregate);

    PxU32 posIters, velIters;
    art->getSolverIterationCounts(posIters, velIters);
    if(ps->getScene()->getSolverType() == PxSolverType::eTGS && velIters > 4)
    {
        CARB_LOG_WARN_ONCE(
            "Detected an articulation at %s with more than 4 velocity iterations being added to a TGS scene."
            "The related behavior changed recently, please consult the changelog. This warning will only print once.",
            attachedStage.textFor(articulationRecord.mKey));
    }

    setupContactReportToArticulation(ps, attachedStage, articulationRecord.mKey, *art);
}

void PhysXUsdPhysicsInterface::finalizeArticulations(const usdparser::AttachedStage& attachedStage)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    const PxU32 nb = PxU32(mArticulations.size());
    for (PxU32 i = 0; i < nb; i++)
    {
        addArticulationToScene(attachedStage, db.getRecords()[mArticulations[i]]);
    }
    mArticulations.clear();
}

void PhysXUsdPhysicsInterface::finishSetup(const usdparser::AttachedStage& attachedStage)
{
    if (!mDirty)
        return;
    mDirty = false;

    finalizeArticulations(attachedStage);

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    const PxU32 nb = PxU32(mArticulations.size());
    for (PxU32 i = 0; i < nb; i++)
    {
        addArticulationToScene(attachedStage, db.getRecords()[mArticulations[i]]);
    }
    mArticulations.clear();
    const PhysXScenesMap& scenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : scenes)
    {
        ref.second->getContactReport()->resolvePairs();
    }
    db.clearPxJointMap();

    // Kit-viewport-rendering-only: notifies the particles:: postprocess pipeline (isosurface
    // mesh / anisotropy / smoothed-position generators feeding Hydra), which has no
    // ovstage/tensor-API consumer -- raw particle simulation data is read directly, not
    // through this postprocess pipeline. A no-op when the postprocess registry is empty.
    for (size_t i = 0; i < mParticleSystems.size(); ++i)
    {
        particles::notifyParticleSystemResize(mParticleSystems[i]->mKey);
    }
    mParticleSystems.clear();
}

// computes the OBB for this set of points relative to this transform matrix. SIMD version
void computeOBB(PxU32 vcount, const PxVec3* points, PxVec3& sides, const PxQuat& rot, PxVec3& trans)
{
    CARB_ASSERT(vcount);

    PxVec3 minV(FLT_MAX);
    PxVec3 maxV(-FLT_MAX);
    for (PxU32 i = 0; i < vcount; i++)
    {
        const PxVec3& vertexV = points[i];
        const PxVec3 v = rot.rotate(vertexV);

        for (PxU32 j = 0; j < 3; j++)
        {
            minV[j] = fminf(minV[j], v[j]);
            maxV[j] = fmaxf(maxV[j], v[j]);
        }
    }

    sides = (maxV - minV);

    PxMat33 tmpMat(rot);
    const PxVec3 center = (maxV + minV) * 0.5f;
    trans = rot.rotateInv(center);
}

bool PhysXUsdPhysicsInterface::createOBB(const void* inputPoints,
                                         const size_t nbPoints,
                                         carb::Float3& halfExtent,
                                         carb::Float3& offsetPos,
                                         carb::Float4& offsetRot)
{
    if (!inputPoints)
        return false;

    const PxVec3* verts = reinterpret_cast<const PxVec3*>(inputPoints);

    PxConvexMeshDesc desc;
    desc.points.data = verts;
    desc.points.count = (PxU32)nbPoints;
    desc.points.stride = sizeof(PxVec3);
    desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
    // desc.vertexLimit = 8;

    PxConvexMesh* obb = PxCreateConvexMesh(OmniPhysX::getInstance().getPhysXSetup().getDefaultCookingParams(), desc,
        OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getPhysicsInsertionCallback());

    if (!obb)
        return false;

    const PxVec3* points = obb->getVertices();
    PxU32 numPoints = obb->getNbVertices();

    float mass;
    PxMat33 localInertia;
    PxVec3 com;
    obb->getMassInformation(mass, localInertia, com);
    PxQuat inertiaQuat;
    PxDiagonalize(localInertia, inertiaQuat);
    const PxMat33 baseAxis(inertiaQuat);

    PxVec3 halfExt(0.0f);
    PxQuat localRot(PxIdentity);
    PxVec3 localPos(0.0f);

    const PxU32 numSteps = 360;
    const float subStep = 0.01745329251994329547f * (float(360 / numSteps));

    float bestVolume = FLT_MAX;

    for (PxU32 axis = 0; axis < 3; axis++)
    {
        for (PxU32 iStep = 0; iStep < numSteps; iStep++)
        {
            const PxQuat quat(iStep * subStep, baseAxis[axis]);
            PxVec3 psides;
            PxVec3 offs;

            computeOBB(numPoints, points, psides, quat, offs);

            const float volume = psides[0] * psides[1] * psides[2]; // the volume of the cube
            if (volume < bestVolume)
            {
                bestVolume = volume;
                halfExt = psides * 0.5f;
                localRot = quat.getConjugate();
                localPos = offs;
            }
        }
    }


    halfExtent.x = halfExt.x;
    halfExtent.y = halfExt.y;
    halfExtent.z = halfExt.z;

    offsetPos.x = localPos.x;
    offsetPos.y = localPos.y;
    offsetPos.z = localPos.z;

    offsetRot.x = localRot.x;
    offsetRot.y = localRot.y;
    offsetRot.z = localRot.z;
    offsetRot.w = localRot.w;

    obb->release();

    return true;
}

bool PhysXUsdPhysicsInterface::createBoundingSphere(const void* inputPoints,
    const size_t nbPoints,
    carb::Float3& sphereCenter_out,
    float& radius_out)
{
    if (!inputPoints)
        return false;

    const PxVec3* verts = reinterpret_cast<const PxVec3*>(inputPoints);

    PxConvexMeshDesc desc;
    desc.points.data = verts;
    desc.points.count = (PxU32)nbPoints;
    desc.points.stride = sizeof(PxVec3);
    desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    // generate convex hull aproximation and work on the approximation
    PxConvexMesh* obb = PxCreateConvexMesh(OmniPhysX::getInstance().getPhysXSetup().getDefaultCookingParams(), desc,
        OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getPhysicsInsertionCallback());

    if (!obb)
        return false;

    const PxVec3* points = obb->getVertices();
    PxU32 numPoints = obb->getNbVertices();

    float mass;
    PxMat33 localInertia;
    PxVec3 com;
    obb->getMassInformation(mass, localInertia, com);
    PxQuat inertiaQuat;
    PxDiagonalize(localInertia, inertiaQuat);
    const PxMat33 baseAxis(inertiaQuat);

    const float tolEpsilon = 0.001f;

    PxVec3 sphereCenters[3];
    float radius[3];

    // for symmetric objects its beneficial to make an approximation
    // around the inertia base axis
    for (int j = 0; j < 3; j++)
    {
        float maxDist = -FLT_MAX;
        float minDist = FLT_MAX;

        PxVec3 maxVec(0.0f);
        PxVec3 minVec(0.0f);

        PxVec3 maxV(0.0f);
        PxVec3 minV(0.0f);

        for (PxU32 i = 0; i < numPoints; i++)
        {
            const PxVec3& v = points[i];
            const float dist = (v - com).dot(baseAxis[j]);

            if (dist > maxDist)
            {
                maxDist = dist;
                maxVec = com + dist * baseAxis[j];
                maxV = v;
            }

            if (dist < minDist)
            {
                minDist = dist;
                minVec = com + dist * baseAxis[j];
                minV = v;
            }
        }

        sphereCenters[j] = (maxVec + minVec) * 0.5f;
        const float maxRadius = (maxV - sphereCenters[j]).magnitude();
        const float minRadius = (minV - sphereCenters[j]).magnitude();
        radius[j] = maxRadius < minRadius ? maxRadius : minRadius;
    }

    for (int j = 0; j < 3; j++)
    {
        for (PxU32 i = 0; i < numPoints; i++)
        {
            const PxVec3& v = points[i];
            const float dist = (v - sphereCenters[j]).magnitude();

            if (dist > radius[j] + tolEpsilon)
            {
                const PxVec3 k = v - sphereCenters[j];
                const PxVec3 d = baseAxis[j];
                const float r = radius[j];

                const float kRadius = dist;

                const float dot = k.dot(d);
                if (fabsf(dot) < tolEpsilon)
                {
                    radius[j] = kRadius;
                }
                else
                {
                    const float sign = dot > 0.0f ? -1.0f : 1.0f;
                    const float t = (r*r - kRadius * kRadius)/ (sign*2*r*(d.x + d.y + d.z) - 2*(k.x*d.x + k.y * d.y+ k.z * d.z));

                    sphereCenters[j] = sphereCenters[j] + d * t;
                    radius[j] = (v - sphereCenters[j]).magnitude();
                }
            }
        }
    }

    // fall back to the original algorithm
    PxVec3 initialVec = points[0];
    PxVec3 furthersVec0 = points[0];
    float distSqMax = 0.0f;

    for (PxU32 i = 0; i < numPoints; i++)
    {
        const PxVec3& v = points[i];

        const float distSq = (initialVec - v).magnitudeSquared();
        if (distSq > distSqMax)
        {
            furthersVec0 = v;
            distSqMax = distSq;
        }
    }

    PxVec3 furthersVec1 = furthersVec0;
    distSqMax = 0.0f;
    for (PxU32 i = 0; i < numPoints; i++)
    {
        const PxVec3& v = points[i];

        const float distSq = (furthersVec1 - v).magnitudeSquared();
        if (distSq > distSqMax)
        {
            furthersVec1 = v;
            distSqMax = distSq;
        }
    }

    PxVec3 sphereCenter = (furthersVec1 + furthersVec0) * 0.5f;
    float radiusSq = (furthersVec1 - sphereCenter).magnitudeSquared();

    for (PxU32 i = 0; i < numPoints; i++)
    {
        const PxVec3& v = points[i];

        const float distSq = (v - sphereCenter).magnitudeSquared();
        if (distSq > radiusSq + tolEpsilon)
        {
            // compute new sphere
            PxVec3 dir = (v - sphereCenter);
            dir.normalize();
            const PxVec3 opV = sphereCenter - dir * sqrtf(radiusSq);
            sphereCenter = (v + opV) * 0.5f;
            radiusSq = (v - sphereCenter).magnitudeSquared();
        }
    }

    const float qsradius = sqrtf(radiusSq);
    const int index = (radius[0] < radius[1] && radius[0] < radius[2]) ? 0 : ((radius[1] < radius[2]) ? 1 : 2);

    if (radius[index] < qsradius)
    {
        sphereCenter_out = (const carb::Float3&)sphereCenters[index];
        radius_out = radius[index];
    }
    else
    {
        sphereCenter_out = (const carb::Float3&)sphereCenter;
        radius_out = qsradius;
    }

    obb->release();

    return true;
}

void PhysXUsdPhysicsInterface::setupCollisionGroup(omni::physics::parse::ObjectKey key, const CollisionGroupDesc& desc)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    for (size_t i = 0; i < desc.filteredGroups.size(); i++)
    {
        Pair<uint32_t> groupPair(uint32_t(size_t(desc.groupId)), uint32_t(size_t(desc.filteredGroups[i])));
        physxSetup.getCollisionGroupFilteredPairs().insert(groupPair);
    }
}

bool PhysXUsdPhysicsInterface::setVehicleContext(const usdparser::AttachedStage& attachedStage, const VehicleContextDesc& contextDesc)
{
    const ObjectId sceneId = attachedStage.getObjectDatabase()->findEntry(contextDesc.sceneKey, eScene);
    if (sceneId != kInvalidObjectId)
    {
        InternalScene* internalScene = getInternalPtr<InternalScene>(PhysXType::ePTScene, sceneId);

        if (internalScene)
        {
            internalScene->setVehicleContext(contextDesc);
            return true;
        }
    }

    CARB_LOG_ERROR("Physics USD: Physics scene object can not be found for the scene prim at "
        "\"%s\". Vehicle context can not be set.\n",
        attachedStage.textFor(contextDesc.sceneKey));

    return false;
}

// Vehicle creation/registration. USD xform-op authoring of wheel/shape scale is fenced in
// InternalVehicle.cpp's WheelTransformManagementEntry::init; everything below reads through
// the physics source.
ObjectId PhysXUsdPhysicsInterface::createTireFrictionTable(const TireFrictionTableDesc& tireFrictionTableDesc)
{
    return OmniPhysX::getInstance().getInternalPhysXDatabase().createTireFrictionTable(tireFrictionTableDesc);
}

static void registerInWheelComponent(InternalVehicle& vehicle, uint32_t wheelIndex,
    PhysXType componentType, ObjectId componentId)
{
    InternalVehicleWheelReferenceList* wheelRefList = getInternalPtr<InternalVehicleWheelReferenceList>(
        componentType, componentId);
    CARB_ASSERT(wheelRefList);
    wheelRefList->addWheel(vehicle, wheelIndex);
    vehicle.addWheelComponent(*wheelRefList);
}

inline ::physx::PxTransform computeWheelShapeLocalPose(const ::physx::PxMat44d& wheelPose,
                                                      const ::physx::PxMat44d& shapePose)
{
    // note: since world transforms are computed, scale is already taken into account
    // (toTransform() discards it, as the previous toPhysX(pose, scale, m) did).
    return omni::physx::toTransform(wheelPose).getInverse() * omni::physx::toTransform(shapePose);
}

static void prepareWheelTransforms(usdparser::AttachedStage& attachedStage,
                                   InternalVehicle::WheelTransformManagementEntry& wheelTMEntry,
                                   omni::physics::parse::ObjectKey wheelKey,
                                   omni::physics::parse::ObjectKey shapeKey,
                                   ::physx::PxTransform& wheelShapeLocalPose)
{
    bool resetsXformStack;
    wheelTMEntry.initialTransform =
        getLocalTransform(attachedStage, wheelKey, omni::physics::parse::ReadTime::defaultTime(), resetsXformStack);

    if (wheelTMEntry.shape)
    {
        if (wheelKey != shapeKey)
        {
            wheelTMEntry.initialShapeTransform =
                getLocalTransform(attachedStage, shapeKey, omni::physics::parse::ReadTime::defaultTime(), resetsXformStack);
            const ::physx::PxMat44d wheelWorld = getWorldTransform(attachedStage, wheelKey, omni::physics::parse::ReadTime::defaultTime());
            const ::physx::PxMat44d shapeWorld = getWorldTransform(attachedStage, shapeKey, omni::physics::parse::ReadTime::defaultTime());
            wheelShapeLocalPose = computeWheelShapeLocalPose(wheelWorld, shapeWorld);
        }

        // Apply cylinder axis fixup
        const ::physx::PxQuat fixupQ = fixupConeAndCylinderQuat(wheelTMEntry.cylinderAxis);
        wheelShapeLocalPose.q = wheelShapeLocalPose.q * fixupQ;
    }
}

ObjectId PhysXUsdPhysicsInterface::createVehicle(AttachedStage& attachedStage,
                                                 omni::physics::parse::ObjectKey vehicleKey,
                                                 const VehicleDesc& vehicleDesc)
{
    ObjectId objectId = kInvalidObjectId;
    const char* vehiclePath = attachedStage.textFor(vehicleKey);

    // The rigid body for the vehicle must be created before calling createVehicle so mass and extents can be used.

    PxRigidActor* vehicleActor = getPtr<PxRigidActor>(PhysXType::ePTActor, vehicleDesc.bodyId);

    if (!vehicleActor)
    {
        CARB_LOG_ERROR("PhysX Vehicle: \"%s\": referenced rigid body could not be found. Vehicle creation failed.\n",
            vehiclePath);

        return objectId;
    }

    PxScene* pxScene = vehicleActor->getScene();
    if (!pxScene)
    {
        CARB_LOG_ERROR("PhysX Vehicle: \"%s\": referenced rigid body has no PhysX scene. Vehicle creation failed.\n",
            vehiclePath);

        return objectId;
    }

    PxRigidDynamic* vehicleBody = vehicleActor->is<PxRigidDynamic>();
    CARB_ASSERT(vehicleBody);  // the parser needs to ensure rigidBodyEnabled=true
    CARB_ASSERT(!((vehicleBody->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) && vehicleDesc.enabled));
    // the parser needs to ensure the vehicle is disabled if kinematic is enabled

    PxTransform bodyToActor = vehicleBody->getCMassLocalPose();
    const bool bodyScaleIsUniform = scaleIsUniform(vehicleDesc.scale.x, vehicleDesc.scale.y, vehicleDesc.scale.z);
    if ((!bodyScaleIsUniform) && (!bodyToActor.q.isIdentity()))
    {
        // some attributes are defined in the center-of-mass frame. If that one is rotated with respect to
        // the actor frame, we end up with additional complexity and shear scenarios etc. which
        // we want to ignore. Thus, only uniform scale is supported in such a case.

        CARB_LOG_WARN("PhysX Vehicle: \"%s\": ScaleOrientation in center-of-mass frame is not supported. "
            "You may ignore this if the scale is close to uniform.\n",
            vehiclePath);
    }

    if (!scaleIsIdentity(vehicleDesc.scale.x, vehicleDesc.scale.y, vehicleDesc.scale.z))
    {
        CARB_LOG_WARN("PhysX Vehicle: \"%s\": vehicle prim has a gobal scale that is not identity. It is recommended to avoid "
            "such configurations since not all vehicle related attributes are scale aware.\n", vehiclePath);
    }

    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PhysXScene* physxScene = physxSetup.getPhysXScene(size_t(pxScene->userData));
    InternalScene* internalScene = physxScene->getInternalScene();

    {
        const InternalVehicleContext& vehicleContext = internalScene->getVehicleContext();
        if (vehicleContext.getContext().physxScene == NULL)
        {
            // cover misconfigurations like:
            // - no physics scene prim at all (TempPhysicsScene being used)
            // - the respective physics scene prim not having PhysxVehicleContextAPI applied
            //
            // pointing to a non-existent physics scene prim should be covered as part of the rigid body pipeline

            CARB_LOG_ERROR("PhysX Vehicle: vehicle is being created but the stage has no matching physics scene prim with "
                "PhysxVehicleContextAPI applied. Default values are being used but might not match the required setup for the vehicle.\n");

            VehicleContextDesc defaultVehicleContext;
            defaultVehicleContext.setDefaultValues();
            internalScene->setVehicleContext(defaultVehicleContext);
        }
    }

    // Create and store the internal vehicle for the internal scene.
    InternalVehicle* internalVehicle = ICE_NEW(InternalVehicle)(*internalScene);
    if (internalVehicle)
    {
        internalVehicle->mScale = vehicleDesc.scale;

        if (vehicleDesc.hasUserDefinedSprungMassValues)
        {
            internalVehicle->mFlags |= InternalVehicleFlag::eUSER_DEFINED_SPRUNG_MASS;
        }

        if (vehicleDesc.hasUserDefinedMaxDroopValues)
        {
            internalVehicle->mFlags |= InternalVehicleFlag::eUSER_DEFINED_MAX_DROOP;
        }

        if (vehicleDesc.hasUserDefinedRestLoadValues)
        {
            internalVehicle->mFlags |= InternalVehicleFlag::eUSER_DEFINED_REST_LOAD;
        }

        if (vehicleDesc.isUsingDeprecatedLatStiffY)
        {
            internalVehicle->mFlags |= InternalVehicleFlag::eIS_USING_LAT_STIFF_Y;
        }

        if (vehicleDesc.referenceFrameIsCenterOfMass)
        {
            internalVehicle->mFlags |= InternalVehicleFlag::eREFERENCE_FRAME_IS_COM;
        }

        const uint32_t wheelCount = static_cast<uint32_t>(vehicleDesc.wheelAttachments.size());
        std::vector<::physx::PxShape*> wheelShapeMapping;
        wheelShapeMapping.reserve(wheelCount);
        std::vector<const ::physx::PxVehiclePhysXMaterialFrictionParams*> tireMaterialFrictionTables;
        tireMaterialFrictionTables.reserve(wheelCount);
        internalVehicle->mWheelAttachments.resize(wheelCount, nullptr);
        std::vector<::physx::PxTransform> wheelShapeLocalPoses;
        wheelShapeLocalPoses.resize(wheelCount, ::physx::PxTransform(PxIdentity));

        // the wheels should be ordered according to the defined wheel attachment indices but
        // here we collect the data based on the parsing order. The following map will track
        // how to get from wheel attachment index to data index.
        std::vector<uint32_t> wheelIndexToDataMap;
        wheelIndexToDataMap.resize(wheelCount);

        for (uint32_t i = 0; i < wheelCount; i++)
        {
            const WheelAttachmentDesc& wheelAttachmentDesc = vehicleDesc.wheelAttachments[i];
            CARB_ASSERT(wheelAttachmentDesc.index >= 0);
            CARB_ASSERT(wheelAttachmentDesc.index < static_cast<int>(wheelCount));
            const uint32_t wheelIndex = static_cast<uint32_t>(wheelAttachmentDesc.index);

            InternalVehicleWheelAttachment* wheelAttachment = getInternalPtr<InternalVehicleWheelAttachment>(
                PhysXType::ePTVehicleWheelAttachment, wheelAttachmentDesc.id);
            CARB_ASSERT(wheelAttachment);
            wheelAttachment->mVehicle = internalVehicle;
            wheelAttachment->mWheelIndex = wheelIndex;
            internalVehicle->mWheelAttachments[wheelIndex] = wheelAttachment;
            wheelIndexToDataMap[wheelIndex] = i;

            if (wheelAttachmentDesc.tire->frictionTableId != kInvalidObjectId)
            {
                ::physx::PxVehiclePhysXMaterialFrictionParams* tireMaterialFrictionTable = getPtr<::physx::PxVehiclePhysXMaterialFrictionParams>(
                    PhysXType::ePTVehicleTireFrictionTable, wheelAttachmentDesc.tire->frictionTableId);
                CARB_ASSERT(tireMaterialFrictionTable);
                tireMaterialFrictionTables.push_back(tireMaterialFrictionTable);
            }
            else
            {
                tireMaterialFrictionTables.push_back(InternalTireFrictionTable::getDefault());
            }

            registerInWheelComponent(*internalVehicle, wheelIndex, PhysXType::ePTVehicleWheel, wheelAttachmentDesc.wheelId);
            registerInWheelComponent(*internalVehicle, wheelIndex, PhysXType::ePTVehicleTire, wheelAttachmentDesc.tireId);
            registerInWheelComponent(*internalVehicle, wheelIndex, PhysXType::ePTVehicleSuspension, wheelAttachmentDesc.suspensionId);

            if (wheelAttachmentDesc.state & WheelAttachmentDesc::eMANAGE_TRANSFORMS)
            {
                PxShape* shape = nullptr;

                if (wheelAttachmentDesc.shapeId != kInvalidObjectId)
                {
                    shape = getPtr<PxShape>(PhysXType::ePTShape, wheelAttachmentDesc.shapeId);
                    wheelShapeMapping.push_back(shape);

                    if (!shape)
                    {
                        CARB_LOG_ERROR(
                            "PhysX Vehicle: \"%s\": referenced shape \"%s\" could not be found. Wheel will not be mapped to shape.\n",
                            vehiclePath, attachedStage.textFor(wheelAttachmentDesc.shapeKey));
                    }
                }
                else
                {
                    // it's legal for wheels to not have a shape mapping
                    wheelShapeMapping.push_back(nullptr);
                }

                if (internalVehicle->mWheelTransformManagementEntries.size() == 0)
                {
                    internalVehicle->mWheelTransformManagementEntries.resize(wheelCount);
                }
                InternalVehicle::WheelTransformManagementEntry& wheelTMEntry = internalVehicle->mWheelTransformManagementEntries[wheelIndex];
                const omni::physics::parse::ObjectKey wheelShapeKey =
                    shape ? wheelAttachmentDesc.shapeKey : omni::physics::parse::ObjectKey{};
                wheelTMEntry.init(wheelAttachmentDesc.key, wheelShapeKey, shape, wheelAttachmentDesc.shapeId);
                prepareWheelTransforms(attachedStage, wheelTMEntry, wheelAttachmentDesc.key, wheelShapeKey,
                                       wheelShapeLocalPoses[i]);

                // note: for now testing against identity is OK since this scale is the local one
                if (!scaleIsIdentity(wheelTMEntry.scale.x, wheelTMEntry.scale.y, wheelTMEntry.scale.z))
                {
                    CARB_LOG_WARN("PhysX Vehicle: \"%s\": wheel attachment prim has a gobal scale that is not identity. It is recommended to avoid "
                        "such configurations since neither the suspension frame nor other attributes are taking this scale into account.\n",
                        attachedStage.textFor(wheelAttachmentDesc.key));
                }
            }
            else
            {
                // if there is no prim representing the wheel transform, then shape mappings are not supported either
                wheelShapeMapping.push_back(nullptr);
            }
        }

        if (!(vehicleBody->getActorFlags() & ::physx::PxActorFlag::eDISABLE_GRAVITY))
        {
            const omni::physics::parse::IPhysicsSource* gravSrc = attachedStage.getSource();
            omni::physics::parse::KnownTokens gravTok;
            if (gravSrc)
                gravTok.intern(*gravSrc);
            if (gravSrc && gravSrc->hasSchema(vehicleKey, gravTok.physxRigidBodyAPI))
            {
                // do not warn if only UsdPhysicsRigidBodyAPI is applied as that one does not
                // have the disableGravity attribute
                CARB_LOG_WARN("PhysX Vehicle: \"%s\": vehicle rigid bodies need to have gravity disabled (see attribute disableGravity)! "
                    "Disabling gravity internally now.\n",
                    vehiclePath);
            }
            vehicleBody->setActorFlag(::physx::PxActorFlag::eDISABLE_GRAVITY, true);
        }

        const InternalVehicleContext& vehicleContext = internalScene->getVehicleContext();
        PxScene* pxScene = physxScene->getScene();
        const float gravityMagnitude = pxScene->getGravity().magnitude();
        PxPhysics& pxPhysics = pxScene->getPhysics();

        ::physx::PxAllocatorCallback* allocatorForPvd;
        const ::physx::PxVehiclePvdAttributeHandles* vehiclePvdAttributeHandles = physxSetup.getVehiclePvdRegistrationHandles();
        if (vehiclePvdAttributeHandles)
        {
            CARB_ASSERT(physxSetup.getOmniPvd());  // else vehiclePvdAttributeHandles should be nullptr
            CARB_ASSERT(physxSetup.getOmniPvd()->getWriter());  // else vehiclePvdAttributeHandles should be nullptr

            allocatorForPvd = &physxSetup.getAllocator();
        }
        else
        {
            allocatorForPvd = nullptr;
        }

        PhysXActorVehicleBase* vehicle = VehicleGenerator::createVehicle(*vehicleBody, pxPhysics,
            vehicleDesc, wheelShapeLocalPoses, wheelShapeMapping, tireMaterialFrictionTables, wheelIndexToDataMap,
            vehicleContext.getFrame(), gravityMagnitude,
            allocatorForPvd);

        if (vehicle)
        {
            internalVehicle->mPhysXVehicle = vehicle;

            if (vehicleDesc.drive)
            {
                if (vehicleDesc.drive->type == ObjectType::eVehicleDriveStandard)
                {
                    const DriveStandardDesc* driveDesc = static_cast<const DriveStandardDesc*>(vehicleDesc.drive);
                    if (driveDesc->engineId != kInvalidObjectId)
                    {
                        InternalVehicleReferenceList* vehicleRefList = getInternalPtr<InternalVehicleReferenceList>(
                            PhysXType::ePTVehicleEngine, driveDesc->engineId);
                        CARB_ASSERT(vehicleRefList);
                        vehicleRefList->addVehicle(*internalVehicle);
                        internalVehicle->mEngineOrDriveBasic = vehicleRefList;
                    }

                    if (driveDesc->autoGearBox)
                    {
                        internalVehicle->mFlags |= InternalVehicleFlag::eHAS_AUTO_GEAR_BOX;
                    }
                }
                else
                {
                    CARB_ASSERT(vehicleDesc.drive->type == ObjectType::eVehicleDriveBasic);
                    const DriveBasicDesc* driveDesc = static_cast<const DriveBasicDesc*>(vehicleDesc.drive);
                    if (driveDesc->id != kInvalidObjectId)
                    {
                        InternalVehicleReferenceList* vehicleRefList = getInternalPtr<InternalVehicleReferenceList>(
                            PhysXType::ePTVehicleDriveBasic, driveDesc->id);
                        CARB_ASSERT(vehicleRefList);
                        vehicleRefList->addVehicle(*internalVehicle);
                        internalVehicle->mEngineOrDriveBasic = vehicleRefList;
                    }
                }
            }

            for (const WheelControllerDesc& wheelControllerDesc : vehicleDesc.wheelControllers)
            {
                InternalVehicleWheelAttachment* wheelAttachment = getInternalPtr<InternalVehicleWheelAttachment>(
                    ePTVehicleWheelController, wheelControllerDesc.id);
                CARB_ASSERT(wheelAttachment);
                wheelAttachment->mInitialControllerValues.key = wheelControllerDesc.key;
                wheelAttachment->setControllerParams(wheelControllerDesc, true);
            }

            objectId = internalScene->addVehicle(*internalVehicle, wheelCount, vehicleKey, vehicleDesc.enabled);
        }
        else
        {
            CARB_LOG_ERROR("PhysX Vehicle: \"%s\": PhysX vehicle could not be created. Vehicle creation failed.\n",
            vehiclePath);

            delete internalVehicle;
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: \"%s\": internal vehicle object could not be allocated. Vehicle creation failed.\n",
            vehiclePath);
    }

    return objectId;
}

ObjectId PhysXUsdPhysicsInterface::createVehicleController(
    AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey vehicleControllerKey,
    const VehicleControllerDesc& vehicleControllerDesc)
{
    ObjectId objectId = kInvalidObjectId;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    InternalVehicle* vehicle = static_cast<InternalVehicle*>(::getInternalPtr(vehicleControllerKey, ePTVehicle));

    if (!vehicle)
    {
        CARB_LOG_ERROR("Could not create vehicle controller at path (%s)", attachedStage.textFor(vehicleControllerKey));
        return objectId;
    }
    InternalVehicle::InitialControllerValues* initialValues = vehicle->allocateInitialControllerValues();
    if (initialValues)
    {
        initialValues->key = vehicleControllerKey;
    }
    vehicle->setControllerParams(vehicleControllerDesc, initialValues);
    objectId = db.addRecord(ePTVehicleController, nullptr, vehicle, vehicleControllerKey);
    return objectId;
}

ObjectId PhysXUsdPhysicsInterface::registerVehicleComponent(omni::physics::parse::ObjectKey key, PhysXType type)
{
    ObjectId objectId = kInvalidObjectId;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    InternalVehicleReferenceList* vehicleRefList = ICE_NEW(InternalVehicleReferenceList);
    objectId = db.addRecord(type, nullptr, vehicleRefList, key);

    return objectId;
}

ObjectId PhysXUsdPhysicsInterface::registerVehicleWheelComponent(omni::physics::parse::ObjectKey key, PhysXType type)
{
    ObjectId objectId = kInvalidObjectId;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    InternalVehicleWheelReferenceList* vehicleWheelRefList = ICE_NEW(InternalVehicleWheelReferenceList);
    objectId = db.addRecord(type, nullptr, vehicleWheelRefList, key);

    return objectId;
}

ObjectId PhysXUsdPhysicsInterface::createVehicleWheelController(
    AttachedStage& attachedStage,
    omni::physics::parse::ObjectKey wheelControllerKey,
    const WheelControllerDesc& wheelControllerDesc)
{
    ObjectId objectId = kInvalidObjectId;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    InternalVehicleWheelAttachment* wheelAttachment = static_cast<InternalVehicleWheelAttachment*>(
        ::getInternalPtr(wheelControllerKey, ePTVehicleWheelAttachment));

    if (!wheelAttachment)
    {
        CARB_LOG_ERROR("Could not create vehicle wheel controller at path (%s)", attachedStage.textFor(wheelControllerKey));
        return objectId;
    }

    objectId = db.addRecord(ePTVehicleWheelController, nullptr, wheelAttachment, wheelControllerKey);
    return objectId;
}

ObjectId PhysXUsdPhysicsInterface::createMimicJoint(const usdparser::MimicJointDesc& mimicJointDesc)
{
    ObjectId objectId = kInvalidObjectId;

    PxArticulationJointReducedCoordinate* pxTargetJoint = getPtr<PxArticulationJointReducedCoordinate>(PhysXType::ePTLinkJoint, mimicJointDesc.mimicJointId);

    if (pxTargetJoint)
    {
        PxArticulationJointReducedCoordinate* pxReferenceJoint = getPtr<PxArticulationJointReducedCoordinate>(PhysXType::ePTLinkJoint, mimicJointDesc.referenceJointId);

        if (pxReferenceJoint)
        {
            const PxArticulationReducedCoordinate& pxTargetJointArt = pxTargetJoint->getParentArticulationLink().getArticulation();
            const PxArticulationReducedCoordinate& pxReferenceJointArt = pxReferenceJoint->getParentArticulationLink().getArticulation();

            if (&pxTargetJointArt == &pxReferenceJointArt)
            {
                InternalArticulation* internalArt = getInternalPtr<InternalArticulation>(ePTArticulation, ObjectId(pxTargetJointArt.userData));
                if (internalArt)
                {
                    CARB_ASSERT(internalArt->mPhysxScene);
                    CARB_ASSERT(internalArt->mPhysxScene->getInternalScene());

                    // mimic joints can only be added if the articulation is not part of a scene
                    // (this should, for example, cover the case where a mimic joint gets defined after
                    // the stage has been attached).
                    removeArticulationFromSceneAndScheduleForReAdd(pxTargetJointArt);

                    // Mimic-joint gearing/offset are authored against each joint's natural USD position
                    // convention (relative to body0 -> body1): PhysxMimicJointAPI as
                    // jointPosition + gearing * referenceJointPosition + offset = 0, and NewtonMimicAPI as
                    // q_follower = coef0 + coef1 * q_leader (parsed to gearing = -coef1, offset = -coef0).
                    // omni.physx, however, negates the PhysX articulation joint coordinate of any joint
                    // whose body0/body1 rel does not follow the articulation hierarchy
                    // (InternalJoint::mBody0IsParentLink == false; see
                    // InternalScene::getArticulationJointPosition and friends). To keep the authored
                    // relation valid in that convention, the gearing/offset must be flipped by the joints'
                    // flip parity: gearing by sign(target)*sign(reference) and offset by sign(target).
                    // These signs are handed to the InternalMimicJoint so they are reapplied on runtime
                    // gearing/offset (coef) changes too.
                    const InternalJoint* targetIntJoint =
                        getInternalPtr<InternalJoint>(ePTLinkJoint, mimicJointDesc.mimicJointId);
                    const InternalJoint* referenceIntJoint =
                        getInternalPtr<InternalJoint>(ePTLinkJoint, mimicJointDesc.referenceJointId);

                    const float targetSign = (targetIntJoint && !targetIntJoint->mBody0IsParentLink) ? -1.0f : 1.0f;
                    const float referenceSign = (referenceIntJoint && !referenceIntJoint->mBody0IsParentLink) ? -1.0f : 1.0f;

                    const float gearingSign = targetSign * referenceSign;
                    const float offsetSign = targetSign;

                    InternalMimicJoint* internalMimicJoint = ICE_NEW(InternalMimicJoint)(*internalArt->mPhysxScene->getInternalScene(), *pxTargetJoint, *pxReferenceJoint,
                        mimicJointDesc.mimicJointAxis, mimicJointDesc.referenceJointAxis,
                        mimicJointDesc.gearing, mimicJointDesc.offset, mimicJointDesc.naturalFrequency, mimicJointDesc.dampingRatio,
                        mimicJointDesc.type, gearingSign, offsetSign);

                    if (internalMimicJoint)
                    {
                        ::physx::PxArticulationMimicJoint* pxMimicJoint = internalMimicJoint->getPhysXMimicJoint();
                        if (pxMimicJoint)
                        {
                            InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

                            objectId = db.addRecord(ePTMimicJoint, pxMimicJoint, internalMimicJoint, mimicJointDesc.mimicJointKey);
                            pxMimicJoint->userData = (void*)(objectId);
                        }
                        else
                        {
                            CARB_LOG_ERROR("PhysX Mimic Joint: \"%s\": PhysX object could not be created.\n",
                                textForActiveStage(mimicJointDesc.mimicJointKey));

                            constexpr bool removeFromTrackers = false;
                            constexpr bool releasePhysXObject = false;

                            internalMimicJoint->release(removeFromTrackers, releasePhysXObject);
                        }
                    }
                    else
                    {
                        CARB_LOG_ERROR("PhysX Mimic Joint: \"%s\": allocation for internal mimic joint object failed.\n",
                            textForActiveStage(mimicJointDesc.mimicJointKey));
                    }
                }
                else
                {
                    CARB_LOG_ERROR("PhysX Mimic Joint: \"%s\": internal articulation object could not be found.\n",
                        textForActiveStage(mimicJointDesc.mimicJointKey));
                }
            }
            else
            {
                CARB_LOG_ERROR("PhysX Mimic Joint: \"%s\": the joints to connect are not part of the same articulation.\n",
                    textForActiveStage(mimicJointDesc.mimicJointKey));
            }
        }
        else
        {
            CARB_LOG_ERROR("PhysX Mimic Joint: \"%s\": PhysX instance of reference joint could not be found.\n",
                textForActiveStage(mimicJointDesc.referenceJointKey));
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Mimic Joint: \"%s\": PhysX instance of mimic joint could not be found.\n",
            textForActiveStage(mimicJointDesc.mimicJointKey));
    }
    
    return objectId;
}

void PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::Enum errorCode, const char* msg)
{
    if (errorCode == usdparser::ErrorCode::eError)
    {
        CARB_LOG_ERROR(msg);
        sendErrorEvent(OmniPhysX::getInstance().getErrorEventStream(), eUsdLoadError, std::make_pair("errorString", msg));
    }
    else if (errorCode == usdparser::ErrorCode::eWarning)
    {
        CARB_LOG_WARN(msg);
    }
    else if (errorCode == usdparser::ErrorCode::eInfo)
    {
        CARB_LOG_INFO(msg);
    }
}

omni::physics::parse::ObjectKey PhysXUsdPhysicsInterface::getParentJointPathInArticulation(
    const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey jointKey)
{
    const ObjectId jointId = attachedStage.getObjectDatabase()->findEntry(jointKey, eArticulationJoint);
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    if (jointId == kInvalidObjectId)
    {
        return jointKey;
    }

    PxArticulationJointReducedCoordinate* joint =
        static_cast<PxArticulationJointReducedCoordinate*>(db.getTypedRecord(ePTLinkJoint, jointId));

    CARB_ASSERT(joint); // the record should exist if its ID can be found in the database

    PxArticulationLink* link = &joint->getParentArticulationLink();

    if (link && link->getInboundJoint())
    {
        const ObjectId parentJointId = (ObjectId)(link->getInboundJoint()->userData);
        const InternalDatabase::Record* jointRecord = db.getFullTypedRecord(ePTLinkJoint, parentJointId);

        CARB_ASSERT(jointRecord); // should exist if PxJoint exists

        if (jointRecord)
        {
            return jointRecord->mKey;
        }
    }

    return jointKey;
}

SubscriptionId PhysXUsdPhysicsInterface::subscribeToObjectChangeNotifications(const IPhysicsObjectChangeCallback& callback)
{
    return mPhysicsObjectChangeSubscriptions.addEvent(callback);
}

void PhysXUsdPhysicsInterface::unsubscribeToObjectChangeNotifications(SubscriptionId subscriptionId)
{
    mPhysicsObjectChangeSubscriptions.removeEvent(subscriptionId);
}

void PhysXUsdPhysicsInterface::removeArticulationFromSceneAndScheduleForReAdd(
    const ::physx::PxArticulationReducedCoordinate& pxArticulation)
{
    if (pxArticulation.getScene() && pxArticulation.getAggregate())
    {
        pxArticulation.getScene()->removeAggregate(*(pxArticulation.getAggregate()));
        mArticulations.push_back((ObjectId)pxArticulation.userData);
        mDirty = true;  // trigger re-add to scene
    }
}

} // namespace physx
} // namespace omni
