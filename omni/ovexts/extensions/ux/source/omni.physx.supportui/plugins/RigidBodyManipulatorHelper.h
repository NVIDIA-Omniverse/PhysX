// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/settings/ISettings.h>
#include <carb/extras/Timer.h>

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxSupportUiRigidBodyManipulator.h>

namespace omni
{
namespace physx
{

class RigidBodyManipulatorHelper final
{
public:
    static float constexpr kDistanceEpsilon = 0.00000001f;
    static float constexpr kAngleEpsilon = 0.01f;

    static float constexpr kRotateSmoothingFactor = 0.25f;
    static float constexpr kRotateMaxLinearVelocity = 2.0f;
    static float constexpr kRotateMaxAngularVelocity = float(M_PI * 4.0);

    static float constexpr kMoveSmoothingFactor = 0.25f;
    static float constexpr kMoveMaxLinearVelocity = 10000.0f;
    static float constexpr kMoveMaxAngularVelocity = float(M_PI);

    struct SceneBackup
    {
        bool ccdEnabled{ false };
    };

    enum class RigidBodyManipulationType
    {
        eMove,
        eRotate,
        eNone
    };

    struct ActorManipulator
    {
        ActorManipulator(){};
        ActorManipulator(RigidBodyManipulatorHelper* helper, const PXR_NS::SdfPath& path);
        ~ActorManipulator();

        RigidBodyManipulatorHelper* helper;
        PXR_NS::SdfPath path;
        ::physx::PxVec3 mTargetTranslation;
        ::physx::PxQuat mTargetRotation;
        ::physx::PxVec3 mRotationPivot;
        bool bLockTranslation;
        bool bLockRotation;
        RigidBodyManipulationType eType;

        struct
        {
            ::physx::PxActorFlags mPxActorFlags{ 0 };
            ::physx::PxRigidBodyFlags mPxRigidBodyFlags{ 0 };
            ::physx::PxRigidDynamicLockFlags mPxRigidDynamicLockFlags{ 0 };
            ::physx::PxTransform mPxCenterOfMass;

            float mAngularDamping;
            float mMaxAngularVelocity;
            float mLinearDamping;
            float mMaxLinearVelocity;
        } backUp;
    };

    typedef std::unordered_map<const PXR_NS::SdfPath, std::unique_ptr<ActorManipulator>, PXR_NS::SdfPath::Hash> TActorManipulatorMap;

public:
    RigidBodyManipulatorHelper();
    ~RigidBodyManipulatorHelper();

    void setStage(const PXR_NS::UsdStageWeakPtr& stage);
    void setPxScene(::physx::PxScene* physxScene);

    void onManipulationBegan(const PXR_NS::SdfPath& path);
    void onManipulationEnded(const PXR_NS::SdfPath& path);

    void update(float dt);

    bool move(const PXR_NS::SdfPath& path, const carb::Float3& deltaTranslation, bool lockRot, bool lockTrans);

    bool rotate(const PXR_NS::SdfPath& path,
                const carb::Float3& pivotWorldPos,
                const carb::Float4& deltaRotation,
                bool lockRot,
                bool lockTrans);

private:
    ::physx::PxActor* getPxActor(const PXR_NS::SdfPath& path) const;

    void lockRotation(::physx::PxActor* pa, bool stateX, bool stateY, bool stateZ);
    void lockTranslation(::physx::PxActor* pa, bool stateX, bool stateY, bool stateZ);

    void savePxSceneBackup();
    void restorePxSceneBackup();

private:
    omni::physx::IPhysx* mPhysX{ nullptr };
    carb::settings::ISettings* mSettings{ nullptr };
    SceneBackup mSceneBackup;
    PXR_NS::UsdStageWeakPtr mStage;
    ::physx::PxScene* mPhysxScene{ nullptr };

    TActorManipulatorMap mActorManipulators;
    // std::unordered_map<usdparser::ObjectId, RigidBodyManipulationData> mActorsManipulationData;
};

} // namespace physx
} // namespace omni
