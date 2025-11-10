// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <carb/Types.h>
#include "OmniPhysX.h"
#include <common/utilities/LockingList.h>

namespace omni
{
namespace physx
{

struct ManipCmd
{
    enum Type
    {
        ePush = 0,
        eGrabWithJoint,
        eGrabWithForce,
        eManipCmdTypeCount
    };
    enum State
    {
        eStartOrUpdate = 0,
        eRelease,
        eManipCmdStateCount
    };
    enum ExecutionType
    {
        eOnce = 0, // Removed after first execution.
        eRepeating, // Remains in the command buffer until cleared.
        eTerminal, // Removed after first execution along with any commands sitting in prior to it in the command buffer
                   // (including the repeating).
        eManipCmdExecTypeCount
    };

    ::physx::PxVec3 orig;
    ::physx::PxVec3 dir;
    Type type;
    State state;
    ExecutionType exec;
    float acceleration;
};

struct DeformableId
{
    DeformableId() : object(-1), subObject(-1), element(-1), bary(0.0f)
    {
    }
    int object;
    int subObject;
    int element;
    ::physx::PxVec4 bary;
};

struct PickType
{
    enum Enum
    {
        eNONE,
        eRIGID,
        ePARTICLE_CLOTH_DEPRECATED,
        eDEFORMABLE_BODY_DEPRECATED,
        eDEFORMABLE_SURFACE_DEPRECATED,
        eSURFACE_DEFORMABLE,
        eVOLUME_DEFORMABLE
    };
};

// Picker object when performing picking operations
struct Picker
{
    PickType::Enum type;

    // eRIGID data
    ::physx::PxRigidBody* hitRigidActor;
    ::physx::PxD6Joint* joint; // joint for joint grabbing.

    // ePARTICLE_CLOTH_DEPRECATED, eDEFORMABLE_BODY_DEPRECATED, eDEFORMABLE_SURFACE_DEPRECATED data
    DeformableId hitDeformableId;
    ::physx::PxDeformableAttachment* deformableAttachment;
    ::physx::PxDeformableElementFilter* deformableFilter;

    // general
    ::physx::PxRigidDynamic* targetActor; // actor for joint grabbing.
    PhysXScene* physXScene;

    Picker()
        : type(PickType::eNONE),
          hitRigidActor(nullptr),
          joint(nullptr),
          hitDeformableId(),
          deformableAttachment(nullptr),
          deformableFilter(nullptr),
          targetActor(nullptr),
          physXScene(nullptr)
    {
    }
};

class RaycastManager
{
public:
    RaycastManager();

    ~RaycastManager();

    void onUpdateRaycasts(float elapsedSecs) const;
    void handleInteractionEvent(const float* orig, const float* dir, PhysicsInteractionEvent interactionEvent);
    bool interactiveActorRaycast(const carb::Float3* origin, const carb::Float3* direction);
    void clearPicker(const ::physx::PxRigidActor* rb);
    void clearCommandBuffer();
    Picker& getPicker()
    {
        return picker;
    }

private:
    void push(const float* orig, const float* dir, float acceleration);
    void grab(const float* orig, const float* dir);

private:
    mutable LockingList<ManipCmd> mManipCmdBuffer;
    Picker picker;
};

void handleRaycast(const float* orig, const float* dir, bool input);

class raycastFilterExcludeInvisible : ::physx::PxQueryFilterCallback
{
public:
    raycastFilterExcludeInvisible(pxr::UsdStageWeakPtr stage) : mStage(stage){};
    ~raycastFilterExcludeInvisible(){};

    virtual ::physx::PxQueryHitType::Enum preFilter(const ::physx::PxFilterData& filterData0,
                                                    const ::physx::PxShape* shape,
                                                    const ::physx::PxRigidActor* actor,
                                                    ::physx::PxHitFlags&) override;

    virtual ::physx::PxQueryHitType::Enum postFilter(const ::physx::PxFilterData&,
                                                     const ::physx::PxQueryHit&,
                                                     const ::physx::PxShape*,
                                                     const ::physx::PxRigidActor*) override
    {
        return ::physx::PxQueryHitType::eBLOCK;
    }

private:
    pxr::UsdStageWeakPtr mStage;
};

} // namespace physx
} // namespace omni
