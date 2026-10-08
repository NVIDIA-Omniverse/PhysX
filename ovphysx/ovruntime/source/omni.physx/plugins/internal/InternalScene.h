// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-ACTIVEACTOR-001
 * @covers AC-1 AC-3
 */

#pragma once

#ifdef __linux__
#    define __forceinline __attribute__((always_inline))
#endif

#include "Internal.h"
#include "InternalVehicle.h"
#include "InternalActor.h"

#include <MeshCache.h>

#include <VehicleGenerator.h>
#include <PhysXDefines.h>

#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physx/PhysXCompoundShape.h>

#include <PxPhysicsAPI.h>
#include "InternalDebugDraw.h"
#include "InternalParticle.h"
#include "InternalDeformable.h"
#include "InternalDeformableAttachment.h"
#include <usdInterface/UsdInterface.h>

#include <PhysXSimulationCallbacks.h>
#include <common/utilities/Utilities.h>


#include <string>
#include <unordered_set>
#include <vector>

namespace omni
{
namespace physx
{
class PhysXScene;

namespace usdparser
{
class AttachedStage;
}

namespace deformables
{
    class VolumeDeformablePostSolveCallback;
    class SurfaceDeformablePostSolveCallback;
}

namespace internal
{

const uint32_t kInvalidUint32_t = 0xFFFFFFFF;
using CctMap = std::unordered_map<omni::physics::parse::ObjectKey, InternalCct*, omni::physics::parse::ObjectKey::Hash>;

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
    InternalShape(PhysXScene* ps, const carb::Float3& scale, usdparser::ObjectId /*matId*/ = usdparser::kInvalidObjectId)
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
        : mEnableSelfCollision(true), mAggregate(nullptr), mPhysxScene(ps)
    {
        mStaticRootBodyKey = omni::physics::parse::ObjectKey{};
    }

    ~InternalArticulation() = default;

    bool mEnableSelfCollision;
    // ObjectKey of the static root body actor (invalid when the articulation has none);
    // resolve to a path on demand via AttachedStage::pathFor.
    omni::physics::parse::ObjectKey mStaticRootBodyKey;
    ::physx::PxAggregate* mAggregate;
    PhysXScene* mPhysxScene;
};

// Axis instance-name text for PhysxJointStateAPI's multi-apply "state:<axis>:..."
// properties. Shared so InternalScene.cpp's per-step updateJointState and
// InternalPhysXDatabase.cpp's initial-state restore agree on one definition.
const char* jointStateAxisName(usdparser::ObjectType jointType, ::physx::PxArticulationAxis::Enum physxAxis);

// InternalJoint::InternalJointState's usdToken member type. Pinned to std::string: this header
// is included directly by OvruntimeUnitTests, so the layout must match in every TU.
using JointUsdTokenHandle = std::string;
static_assert(sizeof(JointUsdTokenHandle) == sizeof(std::string),
              "JointUsdTokenHandle must stay ABI-identical to std::string");

// InternalTendonAxis/InternalTendonAttachment's instanceName, pinned to a single type for
// the same cross-target layout reason as JointUsdTokenHandle above.
using TendonInstanceNameHandle = std::string;
static_assert(sizeof(TendonInstanceNameHandle) == sizeof(std::string),
              "TendonInstanceNameHandle must stay ABI-identical to std::string");

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
        // usdToken is only ever written by createArticulationJoint; it stores a plain
        // axis-name literal ("angular"/"linear"/"rotX"/...). Nothing reads it currently,
        // but the member must stay declared unconditionally -- see JointUsdTokenHandle's
        // own comment for the cross-target layout reason.
        JointUsdTokenHandle usdToken;
        bool enabled = false;
        bool convertToDegrees = false;
        ::physx::PxArticulationAxis::Enum physxAxis = ::physx::PxArticulationAxis::eTWIST;
        InternalJointInitialState initialState;

        // Position/Velocity initial-state restore goes through IPhysicsDataWrite::writeData
        // rather than a cached PhysxSchemaJointStateAPI, so this struct needs no non-trivial
        // copy/assign/dtor and no per-instance cache member.
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
    // jointKey is diagnostic-logging-only: resolved to text via the active AttachedStage
    // (never null, empty string on miss/no attach -- AttachedStage::textFor's contract).
    void setArticulationDrivePositionTarget(::physx::PxArticulationJointReducedCoordinate* joint,
                                            ::physx::PxArticulationAxis::Enum axis,
                                            float positionTarget,
                                            omni::physics::parse::ObjectKey jointKey = omni::physics::parse::ObjectKey{}) const;
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

// instanceName is unconditionally TendonInstanceNameHandle (std::string) for the same
// cross-TU layout reason as InternalJointState::usdToken above.
class InternalTendonAxis : public Allocateable
{
public:
    InternalTendonAxis() = default;
    ~InternalTendonAxis() = default;

    TendonInstanceNameHandle instanceName;
};

class InternalTendonAttachment : public Allocateable
{
public:
    InternalTendonAttachment()
        : globalPos(0.f),
          initLength(-FLT_MAX)
    {
    }

    ~InternalTendonAttachment() = default;

    TendonInstanceNameHandle instanceName;
    ::physx::PxVec3 globalPos;
    float initLength;
};

// The Mineways voxel map (InfiniteVoxelMapAPI) is unsupported in the USD-free runtime;
// there is no internal record type for it.

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

    void trackReleasedActiveActor(const ::physx::PxActor* actor)
    {
        if (actor)
            mReleasedActiveActors.insert(actor);
    }

    bool hasReleasedActiveActors() const
    {
        return !mReleasedActiveActors.empty();
    }

    bool isReleasedActiveActor(const ::physx::PxActor* actor) const
    {
        return mReleasedActiveActors.find(actor) != mReleasedActiveActors.end();
    }

    void clearReleasedActiveActors()
    {
        mReleasedActiveActors.clear();
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
                                 bool updateParticlesToUsd);
    void updateRigidBodyTransforms(bool updateToUSD, bool updateVelocitiesToUsd, bool outputVelocitiesLocalSpace);
    void updateCctTransforms(bool updateToUSD);
    void updateVehicleTransforms(bool updateToUSD);
    void updateParticleTransforms(bool updateToUSD, bool updateVelocitiesToUsd, bool updateParticlesToUsd);
    void updateDeformableTransforms(bool updateToUSD, bool updateVelocitiesToUsd);

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

    usdparser::ObjectId addVehicle(InternalVehicle&, const uint32_t wheelCount,
                                   omni::physics::parse::ObjectKey vehicleKey, const bool enabled);
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

    // actors
    // mActors owns the InternalActor entries of this scene. Membership has to stay in sync with
    // InternalActor::mPhysXScene: the removal paths locate an actor through mPhysXScene, so an entry
    // left behind in another scene's list becomes a dangling pointer once the actor is deleted
    // (NVBugs 6504495).
    void addActor(InternalActor& actor);

    // Removes the actor from mActors. Returns false if it was not registered with this scene.
    bool removeActor(const InternalActor& actor);

    // mimic joints
    void addMimicJoint(InternalMimicJoint&);
    void removeMimicJoint(InternalMimicJoint&);
    void releasePhysXMimicJoints(const ::physx::PxArticulationJointReducedCoordinate&);
    bool hasMimicJoint(const ::physx::PxArticulationJointReducedCoordinate&) const;

    void debugDraw(omni::physx::OmniRenderBuffer& renderBuffer, uint64_t debugDrawFlags);

private:
    void setVehicleAtPosition(const uint32_t index, InternalVehicle&);
    void moveVehicleToBack(const uint32_t sourceIndex);
    void moveVehicleToPosition(const uint32_t sourceIndex, const uint32_t targetIndex);
    // Publishes InternalJointState's per-axis PhysxJointStateAPI Position/Velocity values
    // through IPhysicsDataWrite::writeData (attribute names built as literal
    // "state:<axis>:physics:position"/"...:velocity" strings, matching the schema's
    // propertyNamespacePrefix). A no-op when there is no write sink.
    void updateJointState(usdparser::AttachedStage* attachedStage, const InternalDatabase::Record& record, bool updateVelocitiesToUsd);
    void addMimicJointMapEntries(InternalMimicJoint&);
    void removeMimicJointMapEntries(InternalMimicJoint&);
    void removeMimicJointMapEntry(const ::physx::PxArticulationJointReducedCoordinate*, InternalMimicJoint*);

private:
    usdparser::PhysxSceneDesc mSceneDesc;

public:
    std::vector<InternalPbdParticleSystem*> mParticleSystems;
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

    // Bumped whenever what is enumerable from mVehicles changes: membership, order, or any vehicle's
    // set of live wheel attachments. None of those touches the object database, so the object-lifetime
    // epoch that validates every other such cache does not move for them, and a consumer caching rows
    // derived from this array has no other way to learn its rows were retired.
    uint64_t mVehicleSetEpoch;

    ::physx::PxVec3 mGravityDirection;
    float mGravityMagnitude;

    // For deformable skinning
    omni::physx::deformables::VolumeDeformablePostSolveCallback* mVolumeDeformablePostSolveCallback;
    omni::physx::deformables::SurfaceDeformablePostSolveCallback* mSurfaceDeformablePostSolveCallback;

private:
    InternalVehicleContext mVehicleContext;
    ::physx::PxScene* mScene;
    std::unordered_set<const ::physx::PxActor*> mReleasedActiveActors;

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
