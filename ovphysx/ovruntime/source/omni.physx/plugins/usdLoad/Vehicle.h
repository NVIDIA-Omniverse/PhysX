// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/PhysxUsd.h>

#include <common/foundation/Allocator.h>
#include <omni/physics/parse/Handles.h>

#include "LoadTools.h"

#include <unordered_map>
#include <utility>
#include <vector>

namespace omni
{
namespace physx
{
namespace usdparser
{

// ObjectKey-keyed side-table structure (15+ unordered_map<ObjectKey, ...> members), the
// vehicle-specific analogue of ObjectDb. Every map key is the ObjectKey the
// walker/consumer already resolved.
class VehicleComponentTracker
{
public:
    VehicleComponentTracker();
    ~VehicleComponentTracker();

    template <typename T>
    static T* addComponent(omni::physics::parse::ObjectKey key,
        std::unordered_map<omni::physics::parse::ObjectKey, T*, omni::physics::parse::ObjectKey::Hash>& componentMap)
    {
        T* componentRef = ICE_PLACEMENT_NEW(T)();
        if (componentRef)
            componentMap.insert({ key, componentRef });

        return componentRef;
    }

    template <typename T>
    static void removeComponent(omni::physics::parse::ObjectKey key,
        std::unordered_map<omni::physics::parse::ObjectKey, T*, omni::physics::parse::ObjectKey::Hash>& componentMap)
    {
        typename std::unordered_map<omni::physics::parse::ObjectKey, T*, omni::physics::parse::ObjectKey::Hash>::iterator iter = componentMap.find(key);
        if (iter != componentMap.end())
        {
            T* component = iter->second;
            component->~T();
            ICE_FREE(component);

            componentMap.erase(iter);
        }
    }

    template <typename T>
    static T* findComponent(omni::physics::parse::ObjectKey key,
        std::unordered_map<omni::physics::parse::ObjectKey, T*, omni::physics::parse::ObjectKey::Hash>& componentMap)
    {
        typename std::unordered_map<omni::physics::parse::ObjectKey, T*, omni::physics::parse::ObjectKey::Hash>::iterator iter = componentMap.find(key);
        if (iter != componentMap.end())
        {
            return iter->second;
        }
        else
            return nullptr;
    }

    template <typename T>
    static void deleteComponents(
        std::unordered_map<omni::physics::parse::ObjectKey, T*, omni::physics::parse::ObjectKey::Hash>& componentMap)
    {
        typename std::unordered_map<omni::physics::parse::ObjectKey, T*, omni::physics::parse::ObjectKey::Hash>::iterator iter = componentMap.begin();
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
    std::unordered_map<omni::physics::parse::ObjectKey, WheelDesc*, omni::physics::parse::ObjectKey::Hash> mWheels;
    std::unordered_map<omni::physics::parse::ObjectKey, TireDesc*, omni::physics::parse::ObjectKey::Hash> mTires;
    std::unordered_map<omni::physics::parse::ObjectKey, SuspensionDesc*, omni::physics::parse::ObjectKey::Hash> mSuspensions;
    std::unordered_map<omni::physics::parse::ObjectKey, EngineDesc*, omni::physics::parse::ObjectKey::Hash> mEngines;
    std::unordered_map<omni::physics::parse::ObjectKey, GearsDesc*, omni::physics::parse::ObjectKey::Hash> mGears;
    std::unordered_map<omni::physics::parse::ObjectKey, AutoGearBoxDesc*, omni::physics::parse::ObjectKey::Hash> mAutoGearBoxes;
    std::unordered_map<omni::physics::parse::ObjectKey, ClutchDesc*, omni::physics::parse::ObjectKey::Hash> mClutches;
    std::unordered_map<omni::physics::parse::ObjectKey, DriveBasicDesc*, omni::physics::parse::ObjectKey::Hash> mDrivesBasic;
    std::unordered_map<omni::physics::parse::ObjectKey, DriveStandardDesc*, omni::physics::parse::ObjectKey::Hash> mDrivesStandard;

    std::vector<MultiWheelDifferentialDesc*> mMultiWheelDifferentials;
    std::vector<TankDifferentialDesc*> mTankDifferentials;
    // ObjectKey-keyed side-table for differentials so the consumer adapter
    // can pre-populate, and legacy parseDifferential can short-circuit on
    // a lookup hit. Storage lifetime is the legacy vectors above; this
    // map only holds borrow pointers.
    std::unordered_map<omni::physics::parse::ObjectKey, MultiWheelDifferentialDesc*, omni::physics::parse::ObjectKey::Hash> mDifferentialsByPath;
    // Side-tables for brakes (keyed by (key, brakesIndex)) and steering
    // (single-apply, variant chosen by the consumer). Same pattern as
    // mDifferentialsByPath -- borrow pointers only; storage lifetime is
    // the legacy mBrakes / mSteeringBasic / mSteeringAckermann vectors above.
    struct KeyIndexHash
    {
        size_t operator()(const std::pair<omni::physics::parse::ObjectKey, uint8_t>& p) const
        {
            return omni::physics::parse::ObjectKey::Hash{}(p.first) ^ (static_cast<size_t>(p.second) << 1);
        }
    };
    std::unordered_map<std::pair<omni::physics::parse::ObjectKey, uint8_t>, BrakesDesc*, KeyIndexHash> mBrakesByPathIndex;
    std::unordered_map<omni::physics::parse::ObjectKey, SteeringDesc*, omni::physics::parse::ObjectKey::Hash> mSteeringByPath;
    // Per-attachment-prim side-tables. WheelAttachmentDesc is per-vehicle
    // (consumer copies into VehicleDesc::wheelAttachments vector by-value).
    // SuspensionCompliance is owned by the legacy mSuspensionCompliances
    // vector for cleanup; this map is borrow-pointer only.
    std::unordered_map<omni::physics::parse::ObjectKey, WheelAttachmentDesc*, omni::physics::parse::ObjectKey::Hash>  mWheelAttachmentByPath;
    // Vehicle key -> its wheel-attachment keys (in scan order), built from
    // the walker-resolved WheelAttachmentInfo::vehicleKey. Lets parseVehicle
    // enumerate a vehicle's attachments without a USD descendant walk.
    std::unordered_map<omni::physics::parse::ObjectKey, std::vector<omni::physics::parse::ObjectKey>, omni::physics::parse::ObjectKey::Hash> mVehicleWheelAttachments;
    std::unordered_map<omni::physics::parse::ObjectKey, SuspensionComplianceDesc*, omni::physics::parse::ObjectKey::Hash> mSuspensionComplianceByPath;
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
    std::unordered_map<omni::physics::parse::ObjectKey, VehicleDesc*, omni::physics::parse::ObjectKey::Hash> mVehicleByPath;
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
