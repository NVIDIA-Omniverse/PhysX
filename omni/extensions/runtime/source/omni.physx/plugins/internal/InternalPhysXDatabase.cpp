// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/profiler/Profile.h>


#include "InternalPhysXDatabase.h"
#include "PhysXUpdate.h"
#include "Setup.h"
#include "OmniPhysX.h"
#include "PhysXScene.h"
#include "InternalDeformableDeprecated.h"
#include "InternalFilteredPairs.h"

#include <CookingDataAsync.h>
#include <usdLoad/LoadUsd.h>
#include <common/utilities/MemoryMacros.h>

#include <ScopedNoticeLock.h>


using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace pxr;
using namespace carb;
using namespace ::physx;


InternalPhysXDatabase::InternalPhysXDatabase()
    :mInitialTransformsStored(false)
{
    // A.B. lets add one record so that we dont return the id of 0, which
    // is a bit confusing, though seems to still work
    addRecord(ePTRemoved, nullptr, nullptr, pxr::SdfPath());
}

InternalPhysXDatabase::~InternalPhysXDatabase()
{
    release();
}

void InternalPhysXDatabase::release()
{
    waitForSimulationCompletion(false);

    mPxJointMap.clear();

    const size_t nbRecords = getRecords().size();
    for (size_t i = 0; i < nbRecords; i++)
    {
        Record& rec = getRecords()[i];
        switch (rec.mType)
        {
        case ePTShape:
        {
            InternalShape* shape = (InternalShape*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(shape);
        }
        break;

        case ePTMaterial:
        {
            InternalMaterial* material = (InternalMaterial*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(material);
        }
        break;

        case ePTFilteredPair:
        {
            InternalFilteredPairs* intPairs = (InternalFilteredPairs*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(intPairs);
        }
        break;

        case ePTArticulation:
        {
            InternalArticulation* art = (InternalArticulation*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(art);
        }
        break;

        case ePTArticulationFixedBase:
        {
            // A.B. no internal part created
        }
        break;

        case ePTJoint:
        {
            InternalJoint* joint = (InternalJoint*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(joint);
        }
        break;

        case ePTLinkJoint:
        {
            InternalJoint* joint = (InternalJoint*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(joint);
        }
        break;

        case ePTVehicle:
        {
            InternalVehicle* vehicle = (InternalVehicle*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(vehicle);
        }
        break;

        case ePTVehicleController:
        {
            // nothing to do for now as all the logic is in InternalVehicle
        }
        break;

        case ePTVehicleEngine:
        {
            InternalVehicleReferenceList* vehicleRefList = (InternalVehicleReferenceList*)rec.mInternalPtr;
            vehicleRefList->unlinkFromVehicles(ePTVehicleEngine);
            SAFE_DELETE_ALLOCABLE_SINGLE(vehicleRefList);
        }
        break;

        case ePTVehicleTireFrictionTable:
        {
            InternalTireFrictionTable* tireFrictionTable = (InternalTireFrictionTable*)rec.mInternalPtr;
            InternalTireFrictionTable::release(*tireFrictionTable);
        }
        break;

        case ePTVehicleSuspension:
        case ePTVehicleTire:
        case ePTVehicleWheel:
        {
            InternalVehicleWheelReferenceList* wheelRefList = (InternalVehicleWheelReferenceList*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(wheelRefList);
        }
        break;

        case ePTVehicleWheelAttachment:
        {
            InternalVehicleWheelAttachment* wheelAttachment = (InternalVehicleWheelAttachment*)rec.mInternalPtr;
            wheelAttachment->release(false);
        }
        break;

        case ePTVehicleWheelController:
        {
            // nothing to do for now as all the logic is in InternalVehicleWheelAttachment
        }
        break;

        case ePTVehicleDriveBasic:
        {
            InternalVehicleReferenceList* vehicleRefList = (InternalVehicleReferenceList*)rec.mInternalPtr;
            vehicleRefList->unlinkFromVehicles(ePTVehicleDriveBasic);
            SAFE_DELETE_ALLOCABLE_SINGLE(vehicleRefList);
        }
        break;

        case ePTInfiniteVoxelMap:
        {
            InternalInfiniteVoxelMap* internalVoxelMap = (InternalInfiniteVoxelMap*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(internalVoxelMap);
        }
        break;

        case ePTSoftBodyMaterialDeprecated:
        case ePTFEMClothMaterialDeprecated:
        case ePTDeformableVolumeMaterial:
        case ePTDeformableSurfaceMaterial:
        {
            InternalDeformableMaterial* material = (InternalDeformableMaterial*)rec.mInternalPtr;
            SAFE_DELETE_ALLOCABLE_SINGLE(material);
        }
        break;

        default:
            break;
        }
    }

    for (SplinesCurveMap::reference splRef : mSplinesMap)
    {
        delete splRef.second;
    }
    mSplinesMap.clear();
}

SplinesCurve* InternalPhysXDatabase::addSplinesCurve(const pxr::UsdGeomBasisCurves& curve, bool& added)
{
    const SdfPath curvePath = curve.GetPrim().GetPrimPath();
    SplinesCurveMap::const_iterator fit = mSplinesMap.find(curvePath);
    if (fit != mSplinesMap.end())
    {
        added = false;
        return fit->second;
    }

    added = true;
    SplinesCurve* splinesCurve = new SplinesCurve(curve);
    if (!splinesCurve->isInitialized())
    {
        delete splinesCurve;
        return nullptr;
    }

    mSplinesMap[curvePath] = splinesCurve;
    return splinesCurve;
}

omni::physx::usdparser::ObjectId InternalPhysXDatabase::createTireFrictionTable(
        const omni::physx::usdparser::TireFrictionTableDesc& tireFrictionTableDesc,
        const pxr::UsdPrim& usdPrim)
{
    InternalTireFrictionTable* tireFrictionTable = InternalTireFrictionTable::create(tireFrictionTableDesc, *this);
    if (tireFrictionTable)
    {
        ObjectId objectId = addRecord(ePTVehicleTireFrictionTable, tireFrictionTable->getMaterialFrictionTable(),
            tireFrictionTable, usdPrim.GetPrimPath());
        return objectId;
    }
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: tire friction table: failed to create internal object.\n");
    }

    return kInvalidObjectId;
}

void InternalPhysXDatabase::addDirtyMassActor(size_t actorIndex)
{
    InternalDatabase::Record& objectRec = getRecords()[actorIndex];

    if (objectRec.mType == ePTActor || objectRec.mType == ePTLink)
    {
        PxRigidActor* actor = (PxRigidActor*)objectRec.mPtr;
        if (actor->is<PxRigidDynamic>() || actor->is<PxArticulationLink>())
        {
            addDirtyMassActor((InternalActor*)objectRec.mInternalPtr);
        }
    }
}
void InternalPhysXDatabase::addDirtyMassActor(InternalActor* actor)
{
    if (!(actor->mFlags & InternalActorFlag::eHAS_DIRTY_MASS))
    {
        actor->mFlags |= InternalActorFlag::eHAS_DIRTY_MASS;
        mDirtyMassActorList.push_back(actor);
    }
}
void InternalPhysXDatabase::removeDirtyMassActor(InternalActor* actor)
{
    if (actor->mFlags & InternalActorFlag::eHAS_DIRTY_MASS)
    {
        actor->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
        for (size_t i = mDirtyMassActorList.size(); i--;)
        {
            if (mDirtyMassActorList[i] == actor)
            {
                mDirtyMassActorList[i] = mDirtyMassActorList.back();
                mDirtyMassActorList.pop_back();
                break;
            }
        }
    }
}

void InternalPhysXDatabase::addDirtyMassDeformableBodyDeprecated(size_t deformableIndex)
{
    InternalDatabase::Record& objectRec = getRecords()[deformableIndex];

    addDirtyMassDeformableBodyDeprecated((InternalDeformableBodyDeprecated*)objectRec.mInternalPtr);
}

void InternalPhysXDatabase::addDirtyMassDeformableBodyDeprecated(InternalDeformableBodyDeprecated* deformable)
{
    if (!(deformable->mFlags & InternalActorFlag::eHAS_DIRTY_MASS))
    {
        deformable->mFlags |= InternalActorFlag::eHAS_DIRTY_MASS;
        mDirtyMassDeformableBodyListDeprecated.push_back(deformable);
    }
}

void InternalPhysXDatabase::removeDirtyMassDeformableBodyDeprecated(InternalDeformableBodyDeprecated* deformable)
{
    if (deformable->mFlags & InternalActorFlag::eHAS_DIRTY_MASS)
    {
        deformable->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
        for (size_t i = mDirtyMassDeformableBodyListDeprecated.size(); i--;)
        {
            if (mDirtyMassDeformableBodyListDeprecated[i] == deformable)
            {
                mDirtyMassDeformableBodyListDeprecated[i] = mDirtyMassDeformableBodyListDeprecated.back();
                mDirtyMassDeformableBodyListDeprecated.pop_back();
                break;
            }
        }
    }
}

void InternalPhysXDatabase::addDirtyMassDeformableSurfaceDeprecated(size_t deformableIndex)
{
    InternalDatabase::Record& objectRec = getRecords()[deformableIndex];

    addDirtyMassDeformableSurfaceDeprecated((InternalDeformableSurfaceDeprecated*)objectRec.mInternalPtr);
}

void InternalPhysXDatabase::addDirtyMassDeformableSurfaceDeprecated(InternalDeformableSurfaceDeprecated* deformable)
{
    if (!(deformable->mFlags & InternalActorFlag::eHAS_DIRTY_MASS))
    {
        deformable->mFlags |= InternalActorFlag::eHAS_DIRTY_MASS;
        mDirtyMassDeformableSurfaceListDeprecated.push_back(deformable);
    }
}

void InternalPhysXDatabase::removeDirtyMassDeformableSurfaceDeprecated(InternalDeformableSurfaceDeprecated* deformable)
{
    if (deformable->mFlags & InternalActorFlag::eHAS_DIRTY_MASS)
    {
        deformable->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
        for (size_t i = mDirtyMassDeformableSurfaceListDeprecated.size(); i--;)
        {
            if (mDirtyMassDeformableSurfaceListDeprecated[i] == deformable)
            {
                mDirtyMassDeformableSurfaceListDeprecated[i] = mDirtyMassDeformableSurfaceListDeprecated.back();
                mDirtyMassDeformableSurfaceListDeprecated.pop_back();
                break;
            }
        }
    }
}

void InternalPhysXDatabase::addDirtyMassParticle(size_t particleIndex)
{
    InternalDatabase::Record& objectRec = getRecords()[particleIndex];

    if (objectRec.mType == ePTParticleSet || objectRec.mType == ePTParticleClothDeprecated)
    {
        addDirtyMassParticle((InternalParticle*)objectRec.mInternalPtr);
    }
}

void InternalPhysXDatabase::addDirtyMassParticle(InternalParticle* particle)
{
    if (!(particle->mFlags & InternalActorFlag::eHAS_DIRTY_MASS))
    {
        particle->mFlags |= InternalActorFlag::eHAS_DIRTY_MASS;
        mDirtyMassParticleList.push_back(particle);
    }
}

void InternalPhysXDatabase::removeDirtyMassParticle(InternalParticle* particle)
{
    if (particle->mFlags & InternalActorFlag::eHAS_DIRTY_MASS)
    {
        particle->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
        for (size_t i = mDirtyMassParticleList.size(); i--;)
        {
            if (mDirtyMassParticleList[i] == particle)
            {
                mDirtyMassParticleList[i] = mDirtyMassParticleList.back();
                mDirtyMassParticleList.pop_back();
                break;
            }
        }
    }
}

void InternalPhysXDatabase::updateDirtyMassActors()
{
    for (size_t i = 0; i < mDirtyMassActorList.size(); i++)
    {
        InternalActor* internalActor = mDirtyMassActorList[i];
        internalActor->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
        if (internalActor->mActor)
            UsdLoad::getUsdLoad()->requestRigidBodyMassUpdate(internalActor->mPrim);

        // A.B. why was this needed?
        //for (size_t j = 0; j < mActors.size(); j++)
        //{
        //    const InternalActor* ia = mActors[j];
        //    if (ia->mActor == actor)
        //    {
        //        UsdLoad::getUsdLoad()->requestRigidBodyMassUpdate(internalActor->mPrim);
        //        break;
        //    }
        //}
    }

    mDirtyMassActorList.clear();

    // DEPRECATED
    for (size_t i = 0; i < mDirtyMassDeformableBodyListDeprecated.size(); i++)
    {
        InternalDeformableBodyDeprecated* internalActor = mDirtyMassDeformableBodyListDeprecated[i];

        if (internalActor->mSoftBody)
        {
            if (internalActor->mFlags & InternalActorFlag::eHAS_DIRTY_MASS)
            {
                UsdLoad::getUsdLoad()->requestDeformableBodyMassUpdateDeprecated(internalActor->mPrim);
                internalActor->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
            }
        }
    }

    mDirtyMassDeformableBodyListDeprecated.clear();

    // DEPRECATED
    for (size_t i = 0; i < mDirtyMassDeformableSurfaceListDeprecated.size(); i++)
    {
        InternalDeformableSurfaceDeprecated* internalActor = mDirtyMassDeformableSurfaceListDeprecated[i];

        if (internalActor->mDeformableSurface)
        {
            if (internalActor->mFlags & InternalActorFlag::eHAS_DIRTY_MASS)
            {
                UsdLoad::getUsdLoad()->requestDeformableSurfaceMassUpdateDeprecated(internalActor->mPrim);
                internalActor->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
            }
        }
    }

    mDirtyMassDeformableSurfaceListDeprecated.clear();

    for (size_t i = 0; i < mDirtyMassParticleList.size(); i++)
    {
        InternalParticle* internalActor = mDirtyMassParticleList[i];
        internalActor->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
        if (internalActor->mNumParticles > 0)
            UsdLoad::getUsdLoad()->requestParticleMassUpdate(internalActor->mPrim);
    }

    mDirtyMassParticleList.clear();
}

void InternalPhysXDatabase::storePxJoint(const ::physx::PxRigidActor* actor, const pxr::SdfPath& path)
{
    const PxU32 numConstraints = actor->getNbConstraints();
    std::vector<PxConstraint*> constaints;
    constaints.resize(size_t(numConstraints));
    actor->getConstraints(constaints.data(), numConstraints);
    for (size_t i = 0; i < constaints.size(); i++)
    {
        PxConstraint& constraint = *constaints[i];
        PxU32 typeId;
        PxJoint* joint = reinterpret_cast<PxJoint*>(constraint.getExternalReference(typeId));
        if (joint && (typeId == PxConstraintExtIDs::eJOINT))
        {
            PxRigidActor* actor0 = nullptr;
            PxRigidActor* actor1 = nullptr;
            joint->getActors(actor0, actor1);
            if (actor0 == actor)
                mPxJointMap.insert(std::pair<pxr::SdfPath, std::pair<PxJoint*, bool>>(path, std::make_pair(joint, true)));
            if (actor1 == actor)
                mPxJointMap.insert(std::pair<pxr::SdfPath, std::pair<PxJoint*, bool>>(path, std::make_pair(joint, false)));
        }
    }
}

void InternalPhysXDatabase::resetStartProperties(bool useUsdUpdate, bool useVelocitiesUSDUpdate, bool outputVelocitiesLocalSpace)
{
    mXformCache.Clear();

    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;

        sc->getInternalScene()->resetStartProperties(useUsdUpdate, useVelocitiesUSDUpdate, outputVelocitiesLocalSpace);
    }

    if (!mInitialTransformsStored)
        return;

    {
        PXR_NS::SdfChangeBlock changeBlock;
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();

        for (ActorInitialDataMap::const_reference ref : mInitialActorDataMap)
        {
            const SdfPath& primPath = ref.first;
            UsdPrim prim = stage->GetPrimAtPath(primPath);
            if (prim)
            {
                const ActorInitialData& initialData = ref.second;
                // write back xformOp
                {
                    UsdGeomXformable xform(prim);
                    initialData.xformOpStorage.restore(xform, true);
                }

                if (useVelocitiesUSDUpdate)
                {
                    if (initialData.velocityWritten)
                    {
                        UsdAttribute velAttr = prim.GetAttribute(pxr::UsdPhysicsTokens.Get()->physicsVelocity);
                        if (velAttr)
                        {
                            velAttr.Set(initialData.velocity);
                        }
                    }
                    else
                    {
                        prim.RemoveProperty(pxr::UsdPhysicsTokens.Get()->physicsVelocity);
                    }

                    if (initialData.angularVelocityWritten)
                    {
                        UsdAttribute angVelAttr = prim.GetAttribute(pxr::UsdPhysicsTokens.Get()->physicsAngularVelocity);
                        if (angVelAttr)
                        {
                            angVelAttr.Set(initialData.angularVelocity);
                        }
                    }
                    else
                    {
                        prim.RemoveProperty(pxr::UsdPhysicsTokens.Get()->physicsAngularVelocity);
                    }
                }
            }
        }

        for (TransformsInstanceMap::const_reference ref : mInitialPointInstancerTransforms)
        {
            const SdfPath& instancerPath = ref.first;
            UsdPrim instancerPrim = stage->GetPrimAtPath(instancerPath);
            if (instancerPrim && instancerPrim.IsA<UsdGeomPointInstancer>())
            {
                UsdGeomPointInstancer instancer(instancerPrim);
                const InitialInstancerData& data = ref.second;

                if (data.positions.size())
                {
                    if (instancer.GetPositionsAttr())
                        instancer.GetPositionsAttr().Set(data.positions);
                }
                if (data.orientations.size())
                {
                    if (instancer.GetOrientationsAttr())
                        instancer.GetOrientationsAttr().Set(data.orientations);
                }
                if (data.scales.size())
                {
                    if (instancer.GetScalesAttr())
                        instancer.GetScalesAttr().Set(data.scales);
                }
                if (data.velocities.size())
                {
                    if (instancer.GetVelocitiesAttr())
                        instancer.GetVelocitiesAttr().Set(data.velocities);
                }
                if (data.angularVelocities.size())
                {
                    if (instancer.GetAngularVelocitiesAttr())
                        instancer.GetAngularVelocitiesAttr().Set(data.angularVelocities);
                }
            }
        }

        for (size_t idx = 0; idx < getRecords().size(); idx++)
        {
            const InternalDatabase::Record& record = getRecords()[idx];
            if (record.mType == ePTLinkJoint)
            {
                InternalJoint* intJoint = (InternalJoint*)record.mInternalPtr;
                pxr::UsdPrim jointPrim = stage->GetPrimAtPath(record.mPath);
                for (size_t idx = 0; idx < 6; ++idx)
                {
                    InternalJoint::InternalJointState& intJointState = intJoint->mJointStates[idx];
                    if (!intJointState.enabled)
                        continue;
                    pxr::PhysxSchemaJointStateAPI cachedJointStateAPI =
                        intJointState.getCachedJointStateAPI(jointPrim, intJoint->mJointType);

                    cachedJointStateAPI.GetPositionAttr().Set(intJointState.initialState.position);
                    cachedJointStateAPI.GetVelocityAttr().Set(intJointState.initialState.velocity);
                }
            }
        }
    }

    mInitialPointInstancerTransforms.clear();
    mInitialActorDataMap.clear();
    mInitialTransformsStored = false;
}

void InternalPhysXDatabase::updateSimulationOutputs(bool updateToUsd)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    UsdStageWeakPtr stage = omniPhysX.getStage();
    if (!stage)
        return;

    ScopedNoticeBlock scopedNoticeBlock;

    if (omniPhysX.getSimulationLayer())
    {        
        pxr::UsdEditContext editContext(stage, UsdEditTarget(omniPhysX.getSimulationLayer()));

        PXR_NS::SdfChangeBlock changeBlock;

        {
            CARB_PROFILE_ZONE(0, "updateJointResiduals");        
            updateJointResiduals(updateToUsd);
        }
    }
    else
    {
        PXR_NS::SdfChangeBlock changeBlock;

        {
            CARB_PROFILE_ZONE(0, "updateJointResiduals");        
            updateJointResiduals(updateToUsd);
        }
    }

    if (omniPhysX.isUploadPhysXStatsToCarbEnabled())
    {
        PhysXStats* physxStats = omniPhysX.getPhysXStats();
        if (physxStats)
        {
            physxStats->update();
        }
    }
}

void InternalPhysXDatabase::updateJointResiduals(bool updateToUsd)
{
    SimulationCallbacks* cb = SimulationCallbacks::getSimulationCallbacks();
    const bool skipWriteResiduals = cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eRESIDUALS | GlobalSimulationFlag::eSKIP_WRITE);
    void* cbUserData = cb->getUserData();
    ResidualUpdateNotificationFn residualFn = cb->getResidualWriteFn();
    const bool notifyResiduals = residualFn && cb->checkGlobalSimulationFlags(GlobalSimulationFlag::eRESIDUALS | GlobalSimulationFlag::eNOTIFY_UPDATE);

    if (notifyResiduals || residualFn || (updateToUsd && !skipWriteResiduals))
    {
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        if (!stage)
            return;

        const InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
        const std::vector<InternalPhysXDatabase::Record>& records = db.getRecords();

        SdfLayerHandle currentLayer = stage->GetEditTarget().GetLayer();

        PXR_NS::SdfChangeBlock changeBlock;

        for (auto joint : mResitualPxJoints)
        {
            if (!(joint->getScene()->getFlags() & PxSceneFlag::eENABLE_SOLVER_RESIDUAL_REPORTING))
                continue;

            const size_t recordIndex = (size_t)joint->userData;
            if (recordIndex < db.getRecords().size())
            {
                const InternalDatabase::Record& record = db.getRecords()[recordIndex];
                if (record.mType == ePTJoint)
                {
                    PxConstraintResidual jointResiduals = joint->getConstraint()->getSolverResidual();
                    PxResiduals residuals;
                    residuals.positionIterationResidual.maxResidual = jointResiduals.positionIterationResidual;
                    residuals.positionIterationResidual.rmsResidual = jointResiduals.positionIterationResidual;
                    residuals.velocityIterationResidual.maxResidual = jointResiduals.velocityIterationResidual;
                    residuals.velocityIterationResidual.rmsResidual = jointResiduals.velocityIterationResidual;
                    updateResidualsImpl(updateToUsd, skipWriteResiduals, notifyResiduals, residualFn, record.mPath, residuals, currentLayer, cbUserData);
                }
            }
        }
    }
}
