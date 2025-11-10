// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#ifdef __linux__
#    define __forceinline __attribute__((always_inline))
#endif

#include "UsdPCH.h"

#include "Internal.h"
#include "InternalVehicle.h"
#include "InternalVoxelMap.h"
#include "InternalActor.h"

#include <MeshCache.h>

#include <VehicleGenerator.h>
#include <PhysXDefines.h>

#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physx/PhysXCompoundShape.h>

#include <PxPhysicsAPI.h>
#include "InternalParticle.h"
#include "InternalDeformableDeprecated.h"
#include "InternalAttachmentDeprecated.h"
#include "InternalDeformable.h"
#include "InternalDeformableAttachment.h"
#include <usdInterface/UsdInterface.h>

#include <PhysXSimulationCallbacks.h>
#include <common/utilities/Utilities.h>


#include <vector>

namespace omni
{
namespace physx
{
class PhysXScene;

namespace deformables
{
    class VolumeDeformablePostSolveCallback;
    class SurfaceDeformablePostSolveCallback;
}

namespace internal
{

inline void updateResidualsImpl(bool updateToUsd,
                                bool skipWriteResiduals,
                                bool notifyResiduals,
                                ResidualUpdateNotificationFn residualFn,
                                const pxr::SdfPath primPath,
                                const ::physx::PxResiduals& residuals,
                                const pxr::SdfLayerHandle& currentLayer,
                                void* cbUserData)
{
    if (updateToUsd && !skipWriteResiduals)
    {
        const pxr::SdfPath posMaxRes =
            primPath.AppendProperty(pxr::PhysxSchemaTokens->physxResidualReportingMaxResidualPositionIteration);
        const pxr::SdfPath posRmsRes =
            primPath.AppendProperty(pxr::PhysxSchemaTokens->physxResidualReportingRmsResidualPositionIteration);
        const pxr::SdfPath velMaxRes =
            primPath.AppendProperty(pxr::PhysxSchemaTokens->physxResidualReportingMaxResidualVelocityIteration);
        const pxr::SdfPath velRmsRes =
            primPath.AppendProperty(pxr::PhysxSchemaTokens->physxResidualReportingRmsResidualVelocityIteration);

        pxr::SdfAttributeSpecHandle posMaxResAttr = currentLayer->GetAttributeAtPath(posMaxRes);
        if (posMaxResAttr)
        {
            posMaxResAttr->SetDefaultValue(pxr::VtValue(residuals.positionIterationResidual.maxResidual));
        }
        else
        {
            pxr::SdfPrimSpecHandle primSpec = pxr::SdfCreatePrimInLayer(currentLayer, primPath);
            posMaxResAttr = pxr::SdfAttributeSpec::New(
                primSpec, pxr::PhysxSchemaTokens->physxResidualReportingMaxResidualPositionIteration.GetString(),
                pxr::SdfValueTypeNames->Float);
            posMaxResAttr->SetDefaultValue(pxr::VtValue(residuals.positionIterationResidual.maxResidual));
        }

        pxr::SdfAttributeSpecHandle posRmsResAttr = currentLayer->GetAttributeAtPath(posRmsRes);
        if (posRmsResAttr)
        {
            posRmsResAttr->SetDefaultValue(pxr::VtValue(residuals.positionIterationResidual.rmsResidual));
        }
        else
        {
            pxr::SdfPrimSpecHandle primSpec = pxr::SdfCreatePrimInLayer(currentLayer, primPath);
            posRmsResAttr = pxr::SdfAttributeSpec::New(
                primSpec, pxr::PhysxSchemaTokens->physxResidualReportingRmsResidualPositionIteration.GetString(),
                pxr::SdfValueTypeNames->Float);
            posRmsResAttr->SetDefaultValue(pxr::VtValue(residuals.positionIterationResidual.rmsResidual));
        }

        pxr::SdfAttributeSpecHandle velMaxResAttr = currentLayer->GetAttributeAtPath(velMaxRes);
        if (velMaxResAttr)
        {
            velMaxResAttr->SetDefaultValue(pxr::VtValue(residuals.velocityIterationResidual.maxResidual));
        }
        else
        {
            pxr::SdfPrimSpecHandle primSpec = pxr::SdfCreatePrimInLayer(currentLayer, primPath);
            velMaxResAttr = pxr::SdfAttributeSpec::New(
                primSpec, pxr::PhysxSchemaTokens->physxResidualReportingMaxResidualVelocityIteration.GetString(),
                pxr::SdfValueTypeNames->Float);
            velMaxResAttr->SetDefaultValue(pxr::VtValue(residuals.velocityIterationResidual.maxResidual));
        }

        pxr::SdfAttributeSpecHandle velRmsResAttr = currentLayer->GetAttributeAtPath(velRmsRes);
        if (velRmsResAttr)
        {
            velRmsResAttr->SetDefaultValue(pxr::VtValue(residuals.velocityIterationResidual.rmsResidual));
        }
        else
        {
            pxr::SdfPrimSpecHandle primSpec = pxr::SdfCreatePrimInLayer(currentLayer, primPath);
            velRmsResAttr = pxr::SdfAttributeSpec::New(
                primSpec, pxr::PhysxSchemaTokens->physxResidualReportingRmsResidualVelocityIteration.GetString(),
                pxr::SdfValueTypeNames->Float);
            velRmsResAttr->SetDefaultValue(pxr::VtValue(residuals.velocityIterationResidual.rmsResidual));
        }
    }

    if (notifyResiduals && residualFn)
    {
        residualFn(asInt(primPath), residuals.positionIterationResidual.maxResidual,
                   residuals.positionIterationResidual.rmsResidual, residuals.velocityIterationResidual.maxResidual,
                   residuals.velocityIterationResidual.rmsResidual, cbUserData);
    }
}

const uint32_t kInvalidUint32_t = 0xFFFFFFFF;
using CctMap = std::unordered_map<pxr::SdfPath, InternalCct*, pxr::SdfPath::Hash>;

class InternalMimicJoint;

struct CompoundShape : PhysXCompoundShape
{
public:
    CompoundShape(PhysXScene* ps)
        : mMaterialId(usdparser::kInvalidObjectId), mInstanceIndex(kInvalidUint32_t), mPhysXScene(ps)
    {
    }

    usdparser::ObjectId mMaterialId;
    PhysXUsdPhysicsInterface::MassInformation mMassInfo;
    uint32_t mInstanceIndex;
    PhysXScene* mPhysXScene;
};

class InternalShape : public Allocateable
{
public:
    InternalShape(PhysXScene* ps, const carb::Float3& scale, usdparser::ObjectId matId = usdparser::kInvalidObjectId)
        : mScale(scale),
          mMaterialId(usdparser::kInvalidObjectId),
          mInstanceIndex(kInvalidUint32_t),
          mDetached(false),
          mAxis(0),
          mPhysicsScene(ps)
    {
    }

    ~InternalShape() = default;

    carb::Float3 mScale;
    usdparser::ObjectId mMaterialId;
    PhysXUsdPhysicsInterface::MassInformation mMassInfo;
    uint32_t mInstanceIndex;
    bool mDetached;
    uint32_t mAxis; // axis information used for certain prim types
    PhysXScene* mPhysicsScene;
};

class InternalMaterial : public Allocateable
{
public:
    InternalMaterial(float density) : mDensity(density)
    {
    }

    ~InternalMaterial() = default;

    void addShapeId(usdparser::ObjectId id)
    {
        mShapeIds.push_back(id);
    }

    void removeShapeId(usdparser::ObjectId id)
    {
        for (size_t i = mShapeIds.size(); i--;)
        {
            if (mShapeIds[i] == id)
            {
                mShapeIds[i] = mShapeIds.back();
                mShapeIds.pop_back();
                break;
            }
        }
    }

    float mDensity;
    std::vector<usdparser::ObjectId> mShapeIds;
};

class InternalArticulation : public Allocateable
{
public:
    InternalArticulation(PhysXScene* ps)
        : mEnableSelfCollision(true), mAggregate(nullptr), mPhysxScene(ps), mReportResiduals(false)
    {
        mStaticRootBody = pxr::SdfPath();
    }

    ~InternalArticulation() = default;

    bool mEnableSelfCollision;
    pxr::SdfPath mStaticRootBody;
    ::physx::PxAggregate* mAggregate;
    PhysXScene* mPhysxScene;
    bool mReportResiduals;
};

class InternalJoint : public Allocateable
{
public:
    InternalJoint()
    {
        mJointType = usdparser::eJointFixed;
        mJointDrive.enabled = false;
    }

    ~InternalJoint() = default;

    usdparser::ObjectType mJointType;
    usdparser::PhysxJointDrive mJointDrive;
    ::physx::PxD6Drive::Enum mD6JointDrive;
    uint32_t mAxisIndex;

    // for MC D6, order is linear x,y,z then rot x,y,z
    // for RC D6, order is rot x,y,z, indexing directly with ::physx::PxArticulationAxis::[eTWIST, eSWING1, eSWING2] =
    // [0, 1, 2]
    usdparser::PhysxJointDrive mJointDrives[6];

    omni::physx::usdparser::Axis mAxis = omni::physx::usdparser::Axis::eX;
    // flag for articulations that indicates if the body0-body1 rel follows the articulation topology (true) or not
    // (false)
    bool mBody0IsParentLink = true;

    struct InternalJointInitialState
    {
        float position = 0.0f;
        float velocity = 0.0f;
    };

    struct InternalJointState
    {
        pxr::TfToken usdToken;
        bool enabled = false;
        bool convertToDegrees = false;
        ::physx::PxArticulationAxis::Enum physxAxis = ::physx::PxArticulationAxis::eTWIST;
        InternalJointInitialState initialState;

        pxr::PhysxSchemaJointStateAPI getCachedJointStateAPI(pxr::UsdPrim jointPrim, usdparser::ObjectType jointType);

    private:
        pxr::PhysxSchemaJointStateAPI cachedJointStateAPI;
    };
    InternalJointState mJointStates[6];

    void copy(const InternalJoint& cpJoint)
    {
        mJointType = cpJoint.mJointType;
        mJointDrive = cpJoint.mJointDrive;
        mD6JointDrive = cpJoint.mD6JointDrive;
        mAxisIndex = cpJoint.mAxisIndex;
        mAxis = cpJoint.mAxis;
        mBody0IsParentLink = cpJoint.mBody0IsParentLink;

        for (int i = 0; i < 6; i++)
        {
            mJointStates[i] = cpJoint.mJointStates[i];
            mJointDrives[i] = cpJoint.mJointDrives[i];
        }
    }

    // in-place fix of joint local pose given mAxis, and for articulation joints also given mBody0IsParentLink
    ::physx::PxQuat getLocalPoseFixupQuat() const;
    void fixupLocalPose(::physx::PxTransform& localPose) const;

    // helpers to set drive target and joint limits according to USD vs. articulation joint order, i.e.
    // mBody0IsParentLink
    void setArticulationJointLimits(::physx::PxArticulationJointReducedCoordinate* joint,
                                    ::physx::PxArticulationAxis::Enum axis,
                                    float usdLowLimit,
                                    float usdHighLimit) const;
    void updateArticulationJointLimitLow(::physx::PxArticulationJointReducedCoordinate* joint,
                                         ::physx::PxArticulationAxis::Enum axis,
                                         float usdLowLimit) const;
    void updateArticulationJointLimitHigh(::physx::PxArticulationJointReducedCoordinate* joint,
                                          ::physx::PxArticulationAxis::Enum axis,
                                          float usdHighLimit) const;
    void setArticulationDrivePositionTarget(::physx::PxArticulationJointReducedCoordinate* joint,
                                            ::physx::PxArticulationAxis::Enum axis,
                                            float positionTarget,
                                            const pxr::SdfPath jointPath = pxr::SdfPath()) const;
    void setArticulationDriveVelocityTarget(::physx::PxArticulationJointReducedCoordinate* joint,
                                            ::physx::PxArticulationAxis::Enum axis,
                                            float velocityTarget) const;
    void setArticulationJointPosition(::physx::PxArticulationJointReducedCoordinate* joint,
                                      ::physx::PxArticulationAxis::Enum axis,
                                      float position) const;
    void setArticulationJointVelocity(::physx::PxArticulationJointReducedCoordinate* joint,
                                      ::physx::PxArticulationAxis::Enum axis,
                                      float velocity) const;
    float getArticulationJointPosition(::physx::PxArticulationJointReducedCoordinate* joint,
                                       ::physx::PxArticulationAxis::Enum axis) const;
    float getArticulationJointVelocity(::physx::PxArticulationJointReducedCoordinate* joint,
                                       ::physx::PxArticulationAxis::Enum axis) const;
};

class InternalTendonAxis : public Allocateable
{
public:
    InternalTendonAxis() : instanceName()
    // mTendonPtr(nullptr)
    {
    }

    ~InternalTendonAxis() = default;

    pxr::TfToken instanceName;
    //::physx::PxArticulationFixedTendon* mTendonPtr;
};

class InternalTendonAttachment : public Allocateable
{
public:
    InternalTendonAttachment()
        : instanceName(),
          // mTendonPtr(nullptr),
          // mArticulationPtr(nullptr),
          globalPos(0.f),
          initLength(-FLT_MAX)
    {
    }

    ~InternalTendonAttachment() = default;

    pxr::TfToken instanceName;
    //::physx::PxArticulationSpatialTendon* mTendonPtr;
    //::physx::PxArticulationReducedCoordinate* mArticulationPtr;
    ::physx::PxVec3 globalPos;
    float initLength;
};

struct InternalInfiniteVoxelMap : public Allocateable
{
    InternalInfiniteVoxelMap(::physx::PxScene* scene,
                             ::pxr::UsdStageRefPtr stage,
                             const usdparser::InfiniteVoxelMapDesc& desc)
        : mInfiniteVoxelMap(scene, stage, desc){};

    virtual ~InternalInfiniteVoxelMap() = default;
    InfiniteVoxelMap mInfiniteVoxelMap;
};

class InternalScene : public Allocateable
{
public:
    InternalScene(const usdparser::PhysxSceneDesc& desc, ::physx::PxScene* scene);
    ~InternalScene();

    void release();

    const usdparser::PhysxSceneDesc& getSceneDesc() const
    {
        return mSceneDesc;
    }

    usdparser::PhysxSceneDesc& getSceneDesc()
    {
        return mSceneDesc;
    }

    void setSceneDesc(const usdparser::PhysxSceneDesc& inDesc)
    {
        mSceneDesc = inDesc;
    }

    ::physx::PxScene* getScene() const
    {
        return mScene;
    }

    uint32_t clampPosIterationCount(uint32_t inCount) const
    {
        return ::physx::PxClamp(inCount, mSceneDesc.minPosIterationCount, mSceneDesc.maxPosIterationCount);
    }

    uint32_t clampVelIterationCount(uint32_t inCount) const
    {
        return ::physx::PxClamp(inCount, mSceneDesc.minVelIterationCount, mSceneDesc.maxVelIterationCount);
    }

    void resetStartProperties(bool useUsdUpdate, bool useVelocitiesUSDUpdate, bool outputVelocitiesLocalSpace);

    void updateSimulationOutputs(bool updateToUSD,
                                 bool updateVelocitiesToUsd,
                                 bool outputVelocitiesLocalSpace,
                                 bool updateParticlesToUsd,
                                 bool updateResidualsToUsd);
    void updateRigidBodyTransforms(bool updateToUSD, bool updateVelocitiesToUsd, bool outputVelocitiesLocalSpace);
    void updateCctTransforms(bool updateToUSD);
    void updateVehicleTransforms(bool updateToUSD);
    void updateParticleTransforms(bool updateToUSD, bool updateVelocitiesToUsd, bool updateParticlesToUsd);
    void updateDeformableTransforms(bool updateToUSD, bool updateVelocitiesToUsd);

    // DEPRECATED attachement
    void addAttachmentDeprecated(InternalAttachmentDeprecated& internalAttachment);
    bool removeAttachmentDeprecated(InternalAttachmentDeprecated& internalAttachment);
    void removeAttachmentsDeprecated(usdparser::ObjectId objId);
    uint32_t updateEventAttachmentsDeprecated(usdparser::ObjectId shapeId,
                                              pxr::SdfPath shapePath,
                                              InternalAttachmentDeprecated::DirtyEventType eventType);
    void swapActorRigidAttachmentsDeprecated(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor);

    // deformable attachment
    void addDeformableAttachment(InternalDeformableAttachment& deformableAttachment);
    bool removeDeformableAttachment(InternalDeformableAttachment& deformableAttachment);
    void removeDeformableAttachments(usdparser::ObjectId objId);
    void swapDeformableAttachmentsRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor);

    // deformable collision filter
    void addDeformableCollisionFilter(InternalDeformableCollisionFilter& deformableCollisionFilter);
    bool removeDeformableCollisionFilter(InternalDeformableCollisionFilter& deformableCollisionFilter);
    void removeDeformableCollisionFilters(usdparser::ObjectId objId);
    void swapDeformableCollisionFiltersRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor);

    // vehicles
    InternalVehicle* getVehicleBody(const ::physx::PxRigidDynamic&) const; // returns nullptr if not part of a vehicle
    void updateVehicleOnMassChange(const ::physx::PxRigidDynamic&,
                                   const float mass,
                                   const ::physx::PxVec3& massSpaceInertiaTensor,
                                   const ::physx::PxTransform& centerOfMassFrame);
    void updateVehicleOnRemovedShape(const ::physx::PxRigidActor& rigidActor, const ::physx::PxShape* removedShape);

    void setVehicleContext(const usdparser::VehicleContextDesc&);
    const InternalVehicleContext& getVehicleContext() const
    {
        return mVehicleContext;
    }
    InternalVehicleContext& getVehicleContext()
    {
        return mVehicleContext;
    }

    usdparser::ObjectId addVehicle(InternalVehicle&, const uint32_t wheelCount, const pxr::UsdPrim&, const bool enabled);
    void removeVehicle(InternalVehicle&);
    void setVehicleEnabledState(InternalVehicle&, const bool enabled);
    __forceinline bool isVehicleEnabled(const InternalVehicle& internalVehicle) const
    {
        return (internalVehicle.mBufferIndex < mEnabledVehicleCount);
    }

    void swapForceActors(::physx::PxRigidActor* actor0, ::physx::PxRigidActor* actor1)
    {
        // A.B. not great, we might need to add some additional index to the actors
        for (InternalForce* force : mForces)
        {
            if (force && force->mRigidActor == actor0)
            {
                force->mRigidActor = actor1;
            }
        }
    }

    CUstream getDeformableCopyStream();
    void syncDeformableCopyStream(::physx::PxCudaContextManager* cudaContextManager);

    void updateResiduals(bool updateToUsd);

    // mimic joints
    void addMimicJoint(InternalMimicJoint&);
    void removeMimicJoint(InternalMimicJoint&);
    void releasePhysXMimicJoints(const ::physx::PxArticulationJointReducedCoordinate&);
    bool hasMimicJoint(const ::physx::PxArticulationJointReducedCoordinate&) const;

private:
    void setVehicleAtPosition(const uint32_t index, InternalVehicle&);
    void moveVehicleToBack(const uint32_t sourceIndex);
    void moveVehicleToPosition(const uint32_t sourceIndex, const uint32_t targetIndex);
    void updateJointState(pxr::UsdStageWeakPtr stage, const InternalDatabase::Record& record, bool updateVelocitiesToUsd);
    void addMimicJointMapEntries(InternalMimicJoint&);
    void removeMimicJointMapEntries(InternalMimicJoint&);
    void removeMimicJointMapEntry(const ::physx::PxArticulationJointReducedCoordinate*, InternalMimicJoint*);

private:
    usdparser::PhysxSceneDesc mSceneDesc;

public:
    std::vector<InternalPbdParticleSystem*> mParticleSystems;
    std::vector<InternalDeformableBodyDeprecated*> mDeformableBodiesDeprecated;
    std::vector<InternalDeformableSurfaceDeprecated*> mDeformableSurfacesDeprecated;
    std::vector<InternalAttachmentDeprecated*> mAttachmentsDeprecated;
    std::vector<InternalVolumeDeformableBody*> mVolumeDeformableBodies;
    std::vector<InternalSurfaceDeformableBody*> mSurfaceDeformableBodies;
    std::vector<InternalDeformableAttachment*> mDeformableAttachments;
    std::vector<InternalDeformableCollisionFilter*> mDeformableCollisionFilters;
    std::vector<InternalActor*> mActors;
    std::vector<InternalActor*> mMirorredActors;
    std::vector<::physx::PxArticulationReducedCoordinate*> mArticulations;
    std::vector<InternalForce*> mForces;

    CctMap mCctMap;

    // vehicles

    // for the vehicle array: enabled vehicles are at the front, disabled vehicles are at the back
    std::vector<InternalVehicle*> mVehicles;

    typedef std::unordered_map<const ::physx::PxRigidDynamic*, InternalVehicle*> ActorToVehicleMap;
    ActorToVehicleMap mVehicleActorToVehicle;

    uint32_t mEnabledVehicleCount;

    ::physx::PxVec3 mGravityDirection;
    float mGravityMagnitude;

    bool mReportResiduals;

    // For deformable skinning
    omni::physx::deformables::VolumeDeformablePostSolveCallback* mVolumeDeformablePostSolveCallback;
    omni::physx::deformables::SurfaceDeformablePostSolveCallback* mSurfaceDeformablePostSolveCallback;

private:
    InternalVehicleContext mVehicleContext;
    ::physx::PxScene* mScene;

    typedef std::unordered_set<InternalMimicJoint*> MimicJointSet;
    MimicJointSet mMimicJointSet;

    typedef std::unordered_multimap<const ::physx::PxArticulationJointReducedCoordinate*, InternalMimicJoint*> JointToMimicJointMap;
    JointToMimicJointMap mJointToMimicJointMap;

    CUstream mDeformableCopyStream;
    bool mDeformableCopyStreamDirty;
};


} // namespace internal
} // namespace physx
} // namespace omni
