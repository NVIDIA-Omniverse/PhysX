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
    // SdfPath-keyed side-table for differentials so the consumer adapter
    // can pre-populate, and legacy parseDifferential can short-circuit on
    // a lookup hit. Storage lifetime is the legacy vectors above; this
    // map only holds borrow pointers.
    std::map<PXR_NS::SdfPath, MultiWheelDifferentialDesc*> mDifferentialsByPath;
    // Side-tables for brakes (keyed by (path, brakesIndex)) and steering
    // (single-apply, variant chosen by the consumer). Same pattern as
    // mDifferentialsByPath -- borrow pointers only; storage lifetime is
    // the legacy mBrakes / mSteeringBasic / mSteeringAckermann vectors above.
    std::map<std::pair<PXR_NS::SdfPath, uint8_t>, BrakesDesc*> mBrakesByPathIndex;
    std::map<PXR_NS::SdfPath, SteeringDesc*> mSteeringByPath;
    // Per-attachment-prim side-tables. WheelAttachmentDesc is per-vehicle
    // (consumer copies into VehicleDesc::wheelAttachments vector by-value).
    // SuspensionCompliance is owned by the legacy mSuspensionCompliances
    // vector for cleanup; this map is borrow-pointer only.
    std::map<PXR_NS::SdfPath, WheelAttachmentDesc*>  mWheelAttachmentByPath;
    // Vehicle prim path -> its wheel-attachment prim paths (in scan order),
    // built from the walker-resolved WheelAttachmentInfo::vehicleKey. Lets
    // parseVehicle enumerate a vehicle's attachments without a USD descendant
    // walk.
    std::map<PXR_NS::SdfPath, std::vector<PXR_NS::SdfPath>> mVehicleWheelAttachments;
    std::map<PXR_NS::SdfPath, SuspensionComplianceDesc*> mSuspensionComplianceByPath;
    // Storage for pre-populated WheelAttachmentDescs (legacy stores by-
    // value in VehicleDesc; we need lifetime management for the
    // pointers held in mWheelAttachmentByPath).
    std::vector<WheelAttachmentDesc*> mWheelAttachmentsOwned;
    std::vector<BrakesDesc*> mBrakes;
    std::vector<SteeringBasicDesc*> mSteeringBasic;
    std::vector<SteeringAckermannDesc*> mSteeringAckermann;
    std::vector<SuspensionComplianceDesc*> mSuspensionCompliances;
    std::vector<NonlinearCmdResponseDesc*> mNonlinearCmdResponses;
    // Pre-populated VehicleDesc side-table. Legacy parseVehicle consults
    // this map at entry; on hit, scalar + cross-ref fields (drive /
    // differential / steering / brakes) are copied into the caller's
    // VehicleDesc and the chassis-attr / cross-ref reads in legacy are
    // short-circuited. Wheel attachments + cross-validation + controllers
    // still run in legacy. mVehiclesOwned holds lifetime for the
    // side-table entries (mVehicleByPath is borrow-only).
    std::map<PXR_NS::SdfPath, VehicleDesc*> mVehicleByPath;
    std::vector<VehicleDesc*> mVehiclesOwned;
};

// parseVehicleContext + parseTireFrictionTable now live in
// omni.physics.parse/ParseVehicleContext.cpp + ParseTireFrictionTable.cpp.
// The walker emits both and the consumer adapter pre-populates the
// engine-side desc lists; no callers remain here.

bool parseVehicle(AttachedStage& attachedStage,
                  omni::physics::parse::ObjectKey vehicleKey,
                  VehicleDesc& vehicleDesc,
                  VehicleControllerDesc& vehicleControllerDesc,
                  VehicleTankControllerDesc& vehicleTankControllerDesc,
                  ObjectType& vehicleControllerType, // will use eUndefined if no controller was defined
                  VehicleComponentTracker&);
} // namespace usdparser

} // namespace physx
} // namespace omni
