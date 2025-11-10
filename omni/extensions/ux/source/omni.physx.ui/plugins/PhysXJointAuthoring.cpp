// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Framework.h>
#include <common/utilities/Utilities.h> // intToPath

#include "PhysXJointAuthoring.h"

#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysx.h>


using namespace omni::physx::ui;
using namespace omni::physics::ui;
PhysXJointAuthoring::PhysXJointAuthoring()
{
}

PhysXJointAuthoring::~PhysXJointAuthoring()
{
}

void PhysXJointAuthoring::update()
{
    if (mIPhysx->isRunning())
    {
        mUsdPhysicsUI->setVisibilityFlags(omni::physics::ui::VisibilityFlags::eHIDE_ALL);
    }
    else
    {
        mUsdPhysicsUI->setVisibilityFlags(omni::physics::ui::VisibilityFlags::eSHOW_ALL);
    }
}

void PhysXJointAuthoring::init(const std::string& extensionPath)
{
    mIPhysx = carb::getCachedInterface<omni::physx::IPhysx>();
    mPrevGizmoScale = 1.0f;
    mUsdPhysicsUI = carb::getCachedInterface<omni::physics::ui::IUsdPhysicsUI>();
    IUsdPhysicsUICustomJointAuthoring customJointAuthoring;
    customJointAuthoring.userData = this;
    customJointAuthoring.onJointAuthoringScale =
        [](uint64_t stageID, uint64_t jointUSDPath,
           const IUsdPhysicsUICustomJointAuthoring::JointScaleData& jointScaleData, void* userData) {
            return static_cast<PhysXJointAuthoring*>(userData)->OnJointScale(stageID, jointUSDPath, jointScaleData);
        };
    customJointAuthoring.onJointGetCustomBillboard = [](uint64_t stageID, uint64_t jointUSDPath, uint32_t& assetId,
                                                         void* userData) {
        return static_cast<PhysXJointAuthoring*>(userData)->OnJointGetCustomBillboard(stageID, jointUSDPath, assetId);
    };
    customJointRegistrationID = mUsdPhysicsUI->registerCustomJointAuthoring(customJointAuthoring, "PhysXJointAuthoring");


    const char* jointStrings[] = { "GearJoint", "RackAndPinionJoint" };
    const int numStrings = sizeof(jointStrings) / sizeof(jointStrings[0]);
    mBillboardAssetIds.resize(numStrings);
    for (int i = 0; i < numStrings; ++i)
    {
        const std::string imagePath = extensionPath + "/icons/physicsJoint/" + jointStrings[i] + ".png";
        mBillboardAssetIds[i] = mUsdPhysicsUI->registerCustomJointBillboard(customJointRegistrationID, imagePath.c_str());
    }
}

void PhysXJointAuthoring::shutdown()
{
    mUsdPhysicsUI->unregisterCustomJointAuthoring(customJointRegistrationID);
    mUsdPhysicsUI = nullptr;
    mIPhysx = nullptr;
    customJointRegistrationID = omni::physics::ui::IUsdPhysicsUICustomJointAuthoring::InvalidRegistrationID;
}

bool PhysXJointAuthoring::OnJointScale(uint64_t stageID,
                                       uint64_t jointUSDPath,
                                       const IUsdPhysicsUICustomJointAuthoring::JointScaleData& jointScaleData)
{
    pxr::UsdStageWeakPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(long(stageID)));
    pxr::SdfPath jointPath = intToPath(jointUSDPath);
    pxr::UsdPrim jointPrim = stage->GetPrimAtPath(jointPath);
    auto ratio = jointScaleData.scaleRatio;
    const float epsilon = 1e-3f;
    const bool scaleAll = abs(ratio.x - ratio.y) < epsilon ? (abs(ratio.y - ratio.z) < epsilon ? true : false) : false;

    if (scaleAll && jointPrim.IsA<pxr::PhysxSchemaPhysxPhysicsRackAndPinionJoint>())
    {
        pxr::PhysxSchemaPhysxPhysicsRackAndPinionJoint rackJoint(jointPrim);
        float rackRatio;
        rackJoint.GetRatioAttr().Get(&rackRatio);
        rackRatio = rackRatio / jointScaleData.scaleRatio.x;
        rackJoint.GetRatioAttr().Set(rackRatio);
        return true; // handled
    }
    return false;
}

bool PhysXJointAuthoring::OnJointGetCustomBillboard(uint64_t stageID, uint64_t jointUSDPath, uint32_t& assetId)
{
    pxr::UsdStageWeakPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(long(stageID)));
    pxr::SdfPath jointPath = intToPath(jointUSDPath);
    pxr::UsdPrim jointPrim = stage->GetPrimAtPath(jointPath);
    if (jointPrim.IsA<pxr::PhysxSchemaPhysxPhysicsRackAndPinionJoint>())
    {
        assetId = mBillboardAssetIds[1];
        return true;
    }
    else if (jointPrim.IsA<pxr::PhysxSchemaPhysxPhysicsGearJoint>())
    {
        assetId = mBillboardAssetIds[0];
        return true;
    }
    return false;
}

