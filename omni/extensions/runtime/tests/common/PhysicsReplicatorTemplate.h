// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>

#include <omni/physx/IPhysxSettings.h>
#include <carb/settings/ISettings.h>
#include "Tools.h"

namespace omni
{
namespace physx
{

class USDReplicator
{
public:
    USDReplicator()
    {
        mStage = nullptr;
    }

    ~USDReplicator()
    {
        mStage = nullptr;
    }

    void init(long stageId, carb::Framework* framework)
    {
        mStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
        mStageId = stageId;

        mSettings = framework->acquireInterface<carb::settings::ISettings>();
        mUpdateUsd = mSettings->getAsBool(kSettingUpdateToUsd);        
        mSettings->setBool(kSettingUpdateToUsd, true);        
    }

    void destroy()
    {
        mStage = nullptr;
        mSettings->setBool(kSettingUpdateToUsd, mUpdateUsd);        
    }

    void clone(const pxr::SdfPath& envPath, const pxr::SdfPath& sourceEnvPath,  uint32_t numEnvs, uint32_t numCols, float offset);

    void replicate(uint64_t sourceEnvPath, uint32_t numReplication);

    bool isFabric()
    {
        return false;
    }

private:
    pxr::UsdStageRefPtr mStage;
    long mStageId;
    carb::settings::ISettings* mSettings;
    bool                mUpdateUsd;
    bool                mUpdateFastCache;
};

class FabricReplicator
{
public:
    FabricReplicator()
    {
        mStage = nullptr;
    }

    ~FabricReplicator()
    {
        mStage = nullptr;
    }

    void init(long stageId, carb::Framework* framework);

    void clone(const pxr::SdfPath& envPath, const pxr::SdfPath& sourceEnvPath,  uint32_t numEnvs, uint32_t numCols, float offset);

    void replicate(uint64_t sourceEnvPath, uint32_t numReplication);

    bool isFabric()
    {
        return true;
    }

    void destroy();

public:
    pxr::UsdStageRefPtr                 mStage;
    omni::fabric::UsdStageId            mStageId;
    carb::settings::ISettings*          mSettings;
    bool                                mUpdateUsd;
    bool                                mUpdateFastCache;
};

}
}
