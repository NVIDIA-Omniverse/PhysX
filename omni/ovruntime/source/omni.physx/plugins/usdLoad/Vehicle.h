// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

#include <common/foundation/Allocator.h>

#include "LoadTools.h"

namespace omni
{
namespace physx
{
namespace usdparser
{

class VehicleComponentTracker
{
public:
    VehicleComponentTracker();
    ~VehicleComponentTracker();

    template <typename T>
    static T* addComponent(const PXR_NS::SdfPath& path, std::map<PXR_NS::SdfPath, T*>& componentMap)
    {
        T* componentRef = ICE_PLACEMENT_NEW(T)();
        if (componentRef)
            componentMap.insert({ path, componentRef });

        return componentRef;
    }

    template <typename T>
    static void removeComponent(const PXR_NS::SdfPath& path, std::map<PXR_NS::SdfPath, T*>& componentMap)
    {
        typename std::map<PXR_NS::SdfPath, T*>::iterator iter = componentMap.find(path);
        if (iter != componentMap.end())
        {
            T* component = iter->second;
            component->~T();
            ICE_FREE(component);

            componentMap.erase(iter);
        }
    }

    template <typename T>
    static T* findComponent(const PXR_NS::SdfPath& path, std::map<PXR_NS::SdfPath, T*>& componentMap)
    {
        typename std::map<PXR_NS::SdfPath, T*>::iterator iter = componentMap.find(path);
        if (iter != componentMap.end())
        {
            return iter->second;
        }
        else
            return nullptr;
    }

    template <typename T>
    static void deleteComponents(std::map<PXR_NS::SdfPath, T*>& componentMap)
    {
        typename std::map<PXR_NS::SdfPath, T*>::iterator iter = componentMap.begin();
        while (iter != componentMap.end())
        {
            T* component = iter->second;
            component->~T();
            ICE_FREE(component);

            iter++;
        }
    }


    template <typename T>
    static T* addComponent(std::vector<T*>& componentList)
    {
        T* componentRef = ICE_PLACEMENT_NEW(T)();
        if (componentRef)
            componentList.push_back(componentRef);

        return componentRef;
    }

    template <typename T>
    static void removeLastComponent(std::vector<T*>& componentList)
    {
        if (!componentList.empty())
        {
            T* component = componentList.back();
            component->~T();
            ICE_FREE(component);

            componentList.pop_back();
        }
    }

    template <typename T>
    static void deleteComponents(std::vector<T*>& componentList)
    {
        for (T* component : componentList)
        {
            component->~T();
            ICE_FREE(component);
        }

        componentList.clear();
    }


public:
    std::map<PXR_NS::SdfPath, WheelDesc*> mWheels;
    std::map<PXR_NS::SdfPath, TireDesc*> mTires;
    std::map<PXR_NS::SdfPath, SuspensionDesc*> mSuspensions;
    std::map<PXR_NS::SdfPath, EngineDesc*> mEngines;
    std::map<PXR_NS::SdfPath, GearsDesc*> mGears;
    std::map<PXR_NS::SdfPath, AutoGearBoxDesc*> mAutoGearBoxes;
    std::map<PXR_NS::SdfPath, ClutchDesc*> mClutches;
    std::map<PXR_NS::SdfPath, DriveBasicDesc*> mDrivesBasic;
    std::map<PXR_NS::SdfPath, DriveStandardDesc*> mDrivesStandard;

    std::vector<MultiWheelDifferentialDesc*> mMultiWheelDifferentials;
    std::vector<TankDifferentialDesc*> mTankDifferentials;
    std::vector<BrakesDesc*> mBrakes;
    std::vector<SteeringBasicDesc*> mSteeringBasic;
    std::vector<SteeringAckermannDesc*> mSteeringAckermann;
    std::vector<SuspensionComplianceDesc*> mSuspensionCompliances;
    std::vector<NonlinearCmdResponseDesc*> mNonlinearCmdResponses;
};

bool parseVehicleContext(const PXR_NS::UsdPrim& usdPrim, VehicleContextDesc&);
TireFrictionTableDesc* parseTireFrictionTable(const PXR_NS::UsdStageWeakPtr stage, const PXR_NS::UsdPrim& usdPrim);
bool parseVehicle(const PXR_NS::UsdStageWeakPtr stage,
                  const PXR_NS::UsdPrim& usdPrim,
                  VehicleDesc& vehicleDesc,
                  VehicleControllerDesc& vehicleControllerDesc,
                  VehicleTankControllerDesc& vehicleTankControllerDesc,
                  ObjectType& vehicleControllerType, // will use eUndefined if no controller was defined
                  VehicleComponentTracker&,
                  PXR_NS::UsdGeomXformCache&);
} // namespace usdparser

} // namespace physx
} // namespace omni
