// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <omni/physx/IPhysxSettings.h>

#include "PrimUpdate.h"
#include "AttachedStage.h"
#include "LoadTools.h"
#include "LoadUsd.h"
#include "Mass.h"

#include <OmniPhysX.h>

#include <omni/fabric/stage/interface/IChangeTrackingConfig.h>
#include <omni/fabric/stage/StageReaderWriter.h>
#include <carb/InterfaceUtils.h>
#include <common/utilities/OmniPhysXUtilities.h>


using namespace pxr;
using namespace omni::physics::schema;

extern omni::fabric::IChangeTrackerConfig* gChangeTrackerConfig;

namespace omni
{
namespace physx
{
namespace usdparser
{

ChangeSourceBlock::ChangeSourceBlock(AttachedStage& attachedStage, ChangeSource source) : mAttachedStage(attachedStage)
{
    mPrevSource = mAttachedStage.getChangeSource();
    mAttachedStage.setChangeSource(source);
}

ChangeSourceBlock::~ChangeSourceBlock()
{
    mAttachedStage.setChangeSource(mPrevSource);
}

AttachedStage::AttachedStage(pxr::UsdStageWeakPtr stage, PhysXUsdPhysicsInterface* iface)
    : mPhysicsInterface(iface),
      mStage(stage),
      mStageId(0),
      mObjectDatabase(nullptr),
      mChangeTrackingPaused(false),
      mReplicatorStage(false),
      mUseReplicatorEnvIds(false)
{
    setStage(stage);
    mObjectDatabase = new ObjectDb();
    mPhysXDefaultSim = omni::physx::isPhysXDefaultSimulator();

    mObjectDatabase->getPrimHierarchyStorage().init(mStage);

    std::vector<ChangeParams> changesToRegister;
    changesToRegister.reserve(1024);
    mPhysicsInterface->fillChangeParams(changesToRegister);
    for (size_t i = 0; i < changesToRegister.size(); i++)
    {
        mPrimChangeMap.registerPrimChange(changesToRegister[i]);
    }

    mListenerId.id = kInvalidFabricListenerId;
}

AttachedStage::AttachedStage(pxr::UsdStageWeakPtr stage)
    : AttachedStage()
{
    setStage(stage);
    mListenerId.id = kInvalidFabricListenerId;
    mObjectDatabase = new ObjectDb();
}

AttachedStage::AttachedStage()
    : mPhysicsInterface(nullptr),
      mStage(nullptr),
      mStageId(0),
      mObjectDatabase(nullptr),
      mChangeTrackingPaused(false),
      mReplicatorStage(false),
      mUseReplicatorEnvIds(false)
{
    mListenerId.id = kInvalidFabricListenerId;
    mObjectDatabase = new ObjectDb();
    mPhysXDefaultSim = true;
}

AttachedStage::~AttachedStage()
{
    freeReplicatorMemory();
    delete mObjectDatabase;
}

void AttachedStage::releasePhysicsObjects()
{
    mPhysicsInterface->releaseAllObjects();

    delete mObjectDatabase;
    mObjectDatabase = new ObjectDb();
    mObjectDatabase->getPrimHierarchyStorage().init(mStage);

    mPrimUpdateMap.setEmptyScene(true);

    mPrimUpdateMap.clearMap();
    mPrimChangeMap.clearMap();
    mPrimChangeMap.clearStageSpecificChanges();

    mAnimatedKinematicBodies.clear();
    mTimeSampledAttributes.clear();

    mCollisionGroupsMap.clear();
    mAdditionalCollisionGroupMaps.clear();
    mAttachmentHistoryMapDeprecated.clear();
    mDeformableAttachmentHistoryMap.clear();
    mDeformableCollisionFilterHistoryMap.clear();

    freeReplicatorMemory();
}

void AttachedStage::registerTimeSampledAttribute(const SdfPath& attributePath, usdparser::OnUpdateObjectFn onUpdate)
{
    mTimeSampledAttributes[attributePath] = onUpdate;
}


void AttachedStage::unregisterTimeSampledAttribute(const SdfPath& attributePath)
{
    mTimeSampledAttributes.erase(attributePath);
}

void AttachedStage::registerStageSpecificAttribute(ChangeParams& changeParam)
{
    mPrimChangeMap.registerStageSpecificChange(changeParam);
}

void AttachedStage::clearStageSpecificAttributes()
{
    mPrimChangeMap.clearStageSpecificChanges();
}

const usdparser::ObjectIdMap* AttachedStage::getObjectIds(const pxr::SdfPath& path) const
{
    return mObjectDatabase->getEntries(path);
}

void AttachedStage::registerObjectId(const pxr::SdfPath& path,
                               const usdparser::ObjectCategory& category,
                               const usdparser::ObjectId& newEntryId)
{
    mObjectDatabase->findOrCreateEntry(path, category, newEntryId);
}


void AttachedStage::updateRigidBodyMass()
{
    for (UsdPrimMap::const_reference ref : mRigidBodyMassUpdateMap)
    {
        if (mStage->GetPrimAtPath(ref.first))
            RequestRigidBodyMassUpdate(*this, ref.second);
    }

    mRigidBodyMassUpdateMap.clear();
}

void AttachedStage::fabricAttach()
{
    const omni::fabric::UsdStageId stageId = { uint64_t(mStageId) };

    if (!OmniPhysX::getInstance().getIStageReaderWriter())
        return;

    omni::fabric::StageReaderWriterId stageInProgressId = OmniPhysX::getInstance().getIStageReaderWriter()->get(stageId);
    if (stageInProgressId.id)
    {
        if (!omni::fabric::Token::iToken)
            omni::fabric::Token::iToken = carb::getCachedInterface<omni::fabric::IToken>();

        if (mListenerId.id == kInvalidFabricListenerId)
        {
            omni::fabric::IChangeTrackerConfig* ctc = carb::getCachedInterface<omni::fabric::IChangeTrackerConfig>();
            mListenerId = ctc->createListener();
        }

        // register all physics known attributes for our listener
        omni::fabric::StageReaderWriter stageRW = OmniPhysX::getInstance().getIStageReaderWriter()->get(stageId);

        {
            PropertyChangeMap::const_iterator it = mPrimChangeMap.getPropertyChangeMap().begin();
            PropertyChangeMap::const_iterator itEnd = mPrimChangeMap.getPropertyChangeMap().end();
            while (it != itEnd)
            {
                omni::fabric::Token fcToken(omni::fabric::asInt(it->first));
                stageRW.attributeEnableChangeTracking(fcToken, mListenerId);
                it++;
            }
        }

        {
            PropertyChangeMap::const_iterator it = mPrimChangeMap.getStageSpecificChangeMap().begin();
            PropertyChangeMap::const_iterator itEnd = mPrimChangeMap.getStageSpecificChangeMap().end();
            while (it != itEnd)
            {
                omni::fabric::Token fcToken(omni::fabric::asInt(it->first));
                stageRW.attributeEnableChangeTracking(fcToken, mListenerId);
                it++;
            }
        }

        const FabricTokens& fcTokens = UsdLoad::getUsdLoad()->getFabricTokens();
        // separate register for transforms, do not track worldmatrix, this should be handled via hierarchy.
        // stageRW.attributeEnableChangeTracking(*fcTokens.worldMatrix, mListenerId);
        stageRW.attributeEnableChangeTracking(*fcTokens.localMatrix, mListenerId);
        stageRW.attributeEnableChangeTracking(*fcTokens.worldForce, mListenerId);
        stageRW.attributeEnableChangeTracking(*fcTokens.positionInvMasses, mListenerId);
        stageRW.attributeEnableChangeTracking(*fcTokens.velocitiesFloat4, mListenerId);
    }
}

void AttachedStage::pauseChangeTracking(bool pause)
{
    if (gChangeTrackerConfig && mListenerId.id != kInvalidFabricListenerId && mStageId > 0)
    {
        const omni::fabric::UsdStageId stageId = { uint64_t(mStageId) };
        omni::fabric::StageReaderWriterId stageInProgressId =
            OmniPhysX::getInstance().getIStageReaderWriter()->get(stageId);
        if (stageInProgressId.id)
        {
            if (pause)
            {
                gChangeTrackerConfig->pause(stageInProgressId, mListenerId);
                mChangeTrackingPaused = true;
            }
            else
            {
                gChangeTrackerConfig->resume(stageInProgressId, mListenerId);
                mChangeTrackingPaused = false;
            }
        }
    }
}
}
}
}
