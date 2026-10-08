// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */

#include "InternalPhysXDatabase.h"
#include "PhysXUpdate.h"
#include "Setup.h"
#include "OmniPhysX.h"
#include "PhysXScene.h"
#include "InternalFilteredPairs.h"

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Mass.h>
#include <omni/physics/parse/IPhysicsDataWrite.h>
#include <omni/physics/parse/KnownTokens.h>
#include <common/utilities/MemoryMacros.h>


using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace carb;
using namespace ::physx;


InternalPhysXDatabase::InternalPhysXDatabase() : mInitialTransformsStored(false), mDebugDrawFlags(0ull), mNestedBodiesUsed(false)
{
    // A.B. lets add one record so that we dont return the id of 0, which
    // is a bit confusing, though seems to still work
    addRecord(ePTRemoved, nullptr, nullptr, omni::physics::parse::ObjectKey{});

    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    if (omniPhysX.isDebugVisualizationEnabled())
    {
        const uint64_t visMask = omniPhysX.getVisualizationBitMask();
        for (uint64_t i = uint64_t(ePhysXNumValues); i < uint64_t(eNumValues); i++)
        {
            if (visMask & (1ull << i))
            {
                setVisualizationParameter(PhysXVisualizationParameter(i), true);
            }
        }
    }
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

SplinesCurve* InternalPhysXDatabase::addSplinesCurve(const AttachedStage& attachedStage,
                                                omni::physics::parse::ObjectKey curveKey,
                                                bool& added)
{
    if (!curveKey.valid())
    {
        added = false;
        return nullptr;
    }

    SplinesCurveMap::const_iterator fit = mSplinesMap.find(curveKey);
    if (fit != mSplinesMap.end())
    {
        added = false;
        return fit->second;
    }

    added = true;
    SplinesCurve* splinesCurve = new SplinesCurve(attachedStage, curveKey);
    if (!splinesCurve->isInitialized())
    {
        delete splinesCurve;
        return nullptr;
    }

    mSplinesMap[curveKey] = splinesCurve;
    return splinesCurve;
}

omni::physx::usdparser::ObjectId InternalPhysXDatabase::createTireFrictionTable(
        const omni::physx::usdparser::TireFrictionTableDesc& tireFrictionTableDesc)
{
    InternalTireFrictionTable* tireFrictionTable = InternalTireFrictionTable::create(tireFrictionTableDesc, *this);
    // No attached stage means there are no records to register against, so skip the add.
    const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    if (tireFrictionTable && attachedStage)
    {
        // Keyed from the descriptor's own key, not a UsdPrim path: identity that resolves
        // with no backing stage.
        ObjectId objectId = addRecord(ePTVehicleTireFrictionTable, tireFrictionTable->getMaterialFrictionTable(),
            tireFrictionTable, tireFrictionTableDesc.key);
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

void InternalPhysXDatabase::addDirtyMassParticle(size_t particleIndex)
{
    InternalDatabase::Record& objectRec = getRecords()[particleIndex];

    if (objectRec.mType == ePTParticleSet)
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
        {
            // Drive the mass recompute by the body's source ObjectKey: RequestRigidBodyMassUpdate
            // reads MassAPI/material/units through the source and needs no UsdPrim, so this works
            // under ovstage too (the prior UsdPrim round-trip threw "Used null prim" for sources
            // with no backing UsdStage).
            if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
                RequestRigidBodyMassUpdate(*as, internalActor->mKey);
        }
    }

    mDirtyMassActorList.clear();

    for (size_t i = 0; i < mDirtyMassParticleList.size(); i++)
    {
        InternalParticle* internalActor = mDirtyMassParticleList[i];
        internalActor->mFlags &= ~InternalActorFlag::eHAS_DIRTY_MASS;
        if (internalActor->mNumParticles > 0)
        {
            if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
                RequestParticleMassUpdate(*as, internalActor->mKey);
        }
    }

    mDirtyMassParticleList.clear();
}

void InternalPhysXDatabase::storePxJoint(const ::physx::PxRigidActor* actor, omni::physics::parse::ObjectKey key)
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
                mPxJointMap.insert(std::pair<omni::physics::parse::ObjectKey, std::pair<PxJoint*, bool>>(key, std::make_pair(joint, true)));
            if (actor1 == actor)
                mPxJointMap.insert(std::pair<omni::physics::parse::ObjectKey, std::pair<PxJoint*, bool>>(key, std::make_pair(joint, false)));
        }
    }
}

void InternalPhysXDatabase::resetStartProperties(bool useUsdUpdate, bool useVelocitiesUSDUpdate, bool outputVelocitiesLocalSpace)
{
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;

        sc->getInternalScene()->resetStartProperties(useUsdUpdate, useVelocitiesUSDUpdate, outputVelocitiesLocalSpace);
    }

    if (!mInitialTransformsStored)
        return;

    // Actor-initial-data velocity/angularVelocity restore, and point-instancer and joint-state
    // initial-value restore, through the source-agnostic write sink -- same
    // tok.physicsVelocity/physicsAngularVelocity tokens as InternalActor.cpp's capture site,
    // tok.positions/orientations/scales/velocities/angularVelocities tokens as
    // InternalScene.cpp's flushInstancerArrays, and the same "state:<axis>:physics:position/
    // velocity" tokens as InternalScene::updateJointState. Unconditional: a no-op when there is
    // no live write sink (a stageless/ovstage-without-sink attach), matching every
    // other write-sink call site in this codebase.
    if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
    {
        if (omni::physics::parse::IPhysicsDataWrite* dw = as->getDataWrite())
        {
            using omni::physics::parse::DataWriteView;
            using omni::physics::parse::DataType;

            if (omni::physics::parse::IPhysicsSource* source = as->getSource())
            {
                omni::physics::parse::KnownTokens tok;
                tok.intern(*source);

                dw->beginWrite();

                // Restore each actor's pre-simulation xform-op stack, snapshotted by
                // InternalActor::initializeDynamicActor's storeXformOpReset call.
                for (ActorInitialDataMap::const_reference ref : mInitialActorDataMap)
                    dw->restoreXformOpReset(ref.first, true);

                for (TransformsInstanceMap::const_reference ref : mInitialPointInstancerTransforms)
                {
                    const InitialInstancerData& data = ref.second;
                    if (!data.positions.empty())
                        dw->writeArray(ref.first, tok.positions,
                                      DataWriteView{ data.positions.data(), data.positions.size(), 0, -1, DataType::e32Bit });
                    if (!data.orientations.empty())
                        dw->writeArray(ref.first, tok.orientations,
                                      DataWriteView{ data.orientations.data(), data.orientations.size(), 0, -1, DataType::e32Bit });
                    if (!data.scales.empty())
                        dw->writeArray(ref.first, tok.scales,
                                      DataWriteView{ data.scales.data(), data.scales.size(), 0, -1, DataType::e32Bit });
                    if (!data.velocities.empty())
                        dw->writeArray(ref.first, tok.velocities,
                                      DataWriteView{ data.velocities.data(), data.velocities.size(), 0, -1, DataType::e32Bit });
                    if (!data.angularVelocities.empty())
                        dw->writeArray(ref.first, tok.angularVelocities,
                                      DataWriteView{ data.angularVelocities.data(), data.angularVelocities.size(), 0, -1, DataType::e32Bit });
                }

                if (useVelocitiesUSDUpdate)
                {
                    for (ActorInitialDataMap::const_reference ref : mInitialActorDataMap)
                    {
                        const ActorInitialData& initialData = ref.second;
                        if (initialData.velocityWritten)
                            dw->writeData(&ref.first, 1, tok.physicsVelocity,
                                         DataWriteView{ &initialData.velocity, 1, 0, -1, DataType::e32Bit });
                        else
                            dw->removeAttribute(ref.first, "physics:velocity");

                        if (initialData.angularVelocityWritten)
                            dw->writeData(&ref.first, 1, tok.physicsAngularVelocity,
                                         DataWriteView{ &initialData.angularVelocity, 1, 0, -1, DataType::e32Bit });
                        else
                            dw->removeAttribute(ref.first, "physics:angularVelocity");
                    }
                }

                for (size_t idx = 0; idx < getRecords().size(); idx++)
                {
                    const InternalDatabase::Record& record = getRecords()[idx];
                    if (record.mType == ePTLinkJoint)
                    {
                        InternalJoint* intJoint = (InternalJoint*)record.mInternalPtr;
                        for (size_t axisIdx = 0; axisIdx < 6; ++axisIdx)
                        {
                            InternalJoint::InternalJointState& intJointState = intJoint->mJointStates[axisIdx];
                            if (!intJointState.enabled)
                                continue;

                            const std::string axisName = jointStateAxisName(intJoint->mJointType, intJointState.physxAxis);
                            const omni::physics::parse::TokenId posAttr = source->internToken("state:" + axisName + ":physics:position");
                            dw->writeData(&record.mKey, 1, posAttr,
                                         DataWriteView{ &intJointState.initialState.position, 1, 0, -1, DataType::e32Bit });
                            const omni::physics::parse::TokenId velAttr = source->internToken("state:" + axisName + ":physics:velocity");
                            dw->writeData(&record.mKey, 1, velAttr,
                                         DataWriteView{ &intJointState.initialState.velocity, 1, 0, -1, DataType::e32Bit });
                        }
                    }
                }

                dw->endWrite();
            }
        }
    }

    mInitialPointInstancerTransforms.clear();
    mInitialActorDataMap.clear();
    mInitialTransformsStored = false;
}

void InternalPhysXDatabase::debugDraw()
{
    mRenderBuffer.clear();
    if (OmniPhysX::getInstance().isDebugVisualizationEnabled())
    {        
        const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();
        for (PhysXScenesMap::const_reference ref : physxScenes)
        {
            const PhysXScene* sc = ref.second;
            sc->getInternalScene()->debugDraw(mRenderBuffer, mDebugDrawFlags);
        }
    }
}

void InternalPhysXDatabase::setVisualizationParameter(PhysXVisualizationParameter param, bool val)
{
    switch (param)
    {
        case eSplinesSurfaceVelocitySegments:
            if (val)
                mDebugDrawFlags |= InternalDebugDrawFlags::eDEBUG_DRAW_SPLINES_SEGMENTS;
            else
                mDebugDrawFlags &= ~InternalDebugDrawFlags::eDEBUG_DRAW_SPLINES_SEGMENTS;
            break;
        case eSplinesSurfaceVelocity:
            if (val)
                mDebugDrawFlags |= InternalDebugDrawFlags::eDEBUG_DRAW_SPLINES;
            else
                mDebugDrawFlags &= ~InternalDebugDrawFlags::eDEBUG_DRAW_SPLINES;
            break;
        default:
            break;
    }
}
