// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "tensors/PhysicsTypes.h"
#include "tensors/gpu/CudaCommon.h"
#include "tensors/gpu/ThrustUtils.h"

#include <memory>
#include <unordered_map>

#define GPUAPI_CHECK_READY(gpuSimData_, retval_)                                                                       \
    if (!gpuSimData_ || !gpuSimData_->checkApiReady(__FUNCTION__))                                                     \
        return (retval_);

namespace omni
{
namespace physx
{
namespace tensors
{
class SimulationBackend;

struct CopyEvent
{
    enum
    {
        eArtiRootTransforms,
        eArtiRootLinVelocities,
        eArtiRootAngVelocities,
        eArtiLinkTransforms,
        eArtiLinkLinearVelocities,
        eArtiLinkAngularVelocities,
        eArtiDofPositions,
        eArtiDofVelocities,
        eArtiDofPositionTargets,
        eArtiDofVelocityTargets,
        eArtiDofActuationForces,
        eArtiJacobians,
        eArtiMassMatrices,
        eArtiMassCenter,
        eArtiCentroidalMomentum,
        eArtiCoriolisCentrifugal,
        eArtiGeneralizedGravity,
        eArtiLinkIncomingJointForce,
        eArtiTendonStiffnesses,
        eArtiTendonDampings,
        eArtiTendonLimitStiffnesses,
        eArtiTendonLimits,
        eArtiTendonRestLengths,
        eArtiTendonOffsets,
        eRdData,
        eSdfData,
        eCOUNT
    };
};

struct ApplyEvent
{
    enum
    {
        eArtiRootTransforms,
        eArtiRootVelocities,
        eArtiRootLinVelocities,
        eArtiRootAngVelocities,
        eArtiLinkForces,
        eArtiLinkTorques,
        eArtiDofPositions,
        eArtiDofVelocities,
        eArtiDofPositionTargets,
        eArtiDofVelocityTargets,
        eArtiDofForces,
        eArtiTendonProperties,
        eRdData,
        eRdLinVelocities,
        eRdAngVelocities,
        eRdForces,
        eRdTorques,
        eCOUNT
    };
};

struct DeformableBodyData
{
    enum Enum
    {
        eSimElementIndices,
        eSimNodalPosition,
        eSimNodalVelocity,
        eSimNodalKinematicTarget,
        eRestNodalPosition,
        eCollElementIndices,
        eCollNodalPosition,
        eCOUNT
    };
};

struct GpuRigidBodyRecord
{
    ::physx::PxU32 physxRdIdx = 0xffffffff; // rigid dynamic index in the physx buffers (only set if rb is a rigid
                                            // dynamic)
    ::physx::PxU32 physxLinkIdx = 0xffffffff; // articulation link index in the physx buffers (only set if rb is an
                                              // articulation link)
    ::physx::PxArticulationGPUIndex physxArtiIdx = 0xffffffff; // physx articulation index (only set if rb is an
                                                               // articulation link)
    ::physx::PxU32 tensorRdIdx = 0xffffffff; // rigid dynamic index in the tensor buffers (only set if rb is a rigid
                                             // dynamic)
    ::physx::PxU32 tensorArtiIdx = 0xffffffff; // rigid dynamic index in the tensor buffers (only set if rb is a rigid
                                               // dynamic)
    ::physx::PxU32 linkIdx = 0xffffffff; // articulation link index
    bool isRootLink = false; // whether this body is an articulation root link
    ::physx::PxVec3 origin{ 0.0f }; // subspace origin
};

struct GpuSdfShapeRecord
{
    ::physx::PxShape* sdfShape = nullptr;
    ::physx::PxShapeGPUIndex globalIndex = 0xffffffff; // gpu index
    ::physx::PxU32 numSamplePoints = 0; // number of sample points
    ::physx::PxVec3 origin{ 0.0f }; // subspace origin
};

struct GpuArticulationRootRecord
{
    ::physx::PxU32 physxArtiIdx = 0xffffffff; // PhysX articulation GPU index
    ::physx::PxVec3 origin{ 0.0f }; // subspace origin
};

struct GpuArticulationDofRecord
{
    ::physx::PxU32 physxDofIdx = 0xffffffff; // DOF index in the physx buffers
    ::physx::PxU32 physxArtiIdx = 0xffffffff; // PhysX articulation GPU index
    bool body0IsParent = true;
};

struct GpuArticulationLinkRecord
{
    ::physx::PxU32 physxLinkIdx = 0xffffffff; // Link index in the physx buffers
    ::physx::PxU32 physxArtiIdx = 0xffffffff; // PhysX articulation GPU index
    ::physx::PxVec3 origin{ 0.0f }; // subspace origin
    ::physx::PxQuat physxToUsdJointRotation = ::physx::PxQuat(0, 0, 0, 1); // inverse of the rotation that is applied to
                                                                           // the incoming joint local pose during USD
                                                                           // parsing.
    ::physx::PxTransform jointChild = ::physx::PxTransform(::physx::PxIdentity);
    ::physx::PxTransform jointParent = ::physx::PxTransform(::physx::PxIdentity);
    ::physx::PxU32 incomingLinkIdx = 0xffffffff; // Link index of the parent body
    ::physx::PxArticulationJointType::Enum incomingJointType = ::physx::PxArticulationJointType::eUNDEFINED;
    ::physx::PxU32 dofOffset = 0xffffffff;
    FreeD6RotationAxesFlags D6RotationAxes = FreeD6RotationAxesFlags(0);
    bool body0IsParent = true;
};

struct GpuArticulationFixedTendonRecord
{
    ::physx::PxU32 physxTendonIdx = 0xffffffff; // Tendon index in the physx buffers
    ::physx::PxU32 physxArtiIdx = 0xffffffff; // PhysX articulation GPU index
};

struct GpuArticulationSpatialTendonRecord
{
    ::physx::PxU32 physxTendonIdx = 0xffffffff; // Tendon index in the physx buffers
    ::physx::PxU32 physxArtiIdx = 0xffffffff; // PhysX articulation GPU index
};

struct GpuRigidContactFilterIdPair
{
    ::physx::PxActor* actor = nullptr;
    ::physx::PxU32 filterIndex = 0xffffffff;

    struct LessThan
    {
        bool operator()(const GpuRigidContactFilterIdPair& a, const GpuRigidContactFilterIdPair& b) const
        {
            return a.actor < b.actor;
        }
    };
};

// This is almost identical to the GpuRigidContactFilterIdPair struct, but it's used for the actor-to-pathId lookup for raw contact data.
// Since we need pathId to be uint64_t, we can't use the GpuRigidContactFilterIdPair struct.
// TODO: Use template to avoid code duplication?
struct GpuActorPathIdPair
{
    ::physx::PxActor* actor = nullptr;
    uint64_t pathId = 0;

    struct LessThan
    {
        __host__ __device__ bool operator()(const GpuActorPathIdPair& a, const GpuActorPathIdPair& b) const
        {
            return a.actor < b.actor;
        }
    };
};

struct GpuDeformableBodyRecord
{
    ::physx::PxDeformableBody* deformableBody;          // physX pointer (base class for PxDeformableVolume and PxDeformableSurface)
    ::physx::PxVec3 origin{ 0.f };                      // subspace origin

    // buffers are managed by tensor view or PxDeformableVolume / PxDeformableSurface instances
    ::physx::PxU32 numSimNodes = 0;                     // number of vertices of the simulation mesh
    ::physx::PxU32 numSimElements = 0;                  // number of elements of the simulation mesh
    ::physx::PxU32 numRestNodes = 0;                    // number of vertices of the rest shape
    ::physx::PxU32 numCollNodes = 0;                    // number of vertices of the collision mesh
    ::physx::PxU32 numCollElements = 0;                 // number of elements of the collision mesh
    ::physx::PxU32* simElementIndices = nullptr;        // managed by tensor view
    ::physx::PxVec4* simNodalPositions = nullptr;       // managed by PxDeformableVolume/PxDeformableSurface
    ::physx::PxVec4* simNodalVelocities = nullptr;      // managed by PxDeformableVolume/PxDeformableSurface
    ::physx::PxVec4* simNodalKinematicTargets = nullptr;// managed by PxDeformableVolume
    ::physx::PxVec3* restNodalPositions = nullptr;      // managed by tensor view
    ::physx::PxVec4* collNodalPositions = nullptr;      // managed by PxDeformableVolume
    ::physx::PxU32* collElementIndices = nullptr;       // managed by tensor view
};

class DeformableBodyBufferManager
{
public:
    DeformableBodyBufferManager() : buffersD(nullptr), bufferSizesD(nullptr), maxBufferSize(0), dataSize(0){};
    DeformableBodyBufferManager(::physx::PxU32 dataSize)
        : buffersD(nullptr), bufferSizesD(nullptr), maxBufferSize(0), dataSize(dataSize){};
    static void allocDevMemAndCopyH2D(void** dst, const void* src, size_t size, const char* name);
    static void resizeDeviceData(::physx::PxU32 previousSize,
                                 ::physx::PxU32 currentSize,
                                 ::physx::PxU32 elementSize,
                                 void* previousBufferDev);
    void uploadDeviceData(std::string debugMessage);
    void setMaxBufferSize();
    void releaseDeviceMem();
    void resetDeviceMem();

    void** buffersD; // device vector of device pointers
    ::physx::PxU32* bufferSizesD; // device vector of buffer sizes
    std::vector<void*> buffersH; // host vector of device pointers
    std::vector<::physx::PxU32> bufferSizesH; // host vector of buffer sizes
    ::physx::PxU32 maxBufferSize; // max buffer size
    ::physx::PxU32 dataSize; // max buffer size
};

// Does not mirror any PhysX enum (that mapping was removed); consider replacing with something better.
class ArticulationGpuFlag
{
public:
    enum Enum
    {
        eJOINT_POSITION = (1 << 0),
        eJOINT_VELOCITY = (1 << 1),
        eJOINT_ACCELERATION = (1 << 2),
        eJOINT_FORCE = (1 << 3),
        eJOINT_TARGET_VELOCITY = (1 << 4),
        eJOINT_TARGET_POSITION = (1 << 5),
        eROOT_TRANSFORM = (1 << 6),
        eROOT_VELOCITY = (1 << 7),
        eLINK_TRANSFORM = (1 << 8),
        eLINK_VELOCITY = (1 << 9),
        eLINK_ACCELERATION = (1 << 10),
        eLINK_FORCE = (1 << 11),
        eLINK_TORQUE = (1 << 12),
        eFIXED_TENDON = (1 << 13),
        eFIXED_TENDON_JOINT = (1 << 14),
        eSPATIAL_TENDON = (1 << 15),
        eSPATIAL_TENDON_ATTACHMENT = (1 << 16)
    };
};

class ActorGpuFlag
{
public:
    enum Enum
    {
        eACTOR_DATA = (1 << 0), // include transform and velocity
        eFORCE = (1 << 1),
        eTORQUE = (1 << 2)
    };
};

typedef ::physx::PxFlags<ArticulationGpuFlag::Enum, ::physx::PxU32> ArticulationGpuFlags;
typedef ::physx::PxFlags<ActorGpuFlag::Enum, ::physx::PxU16> ActorGpuFlags;

// data shared with child views that persist even if simulation view is deleted
struct GpuSimulationData
{
    GpuSimulationData(SimulationBackend& backend, long stageId);
    ~GpuSimulationData();

    bool init(::physx::PxScene* scene);

    // checks for illegal usage
    bool checkApiReady(const char* function) const;
    void clearForces();

    bool flush();

    void enableGpuUsageWarnings(bool enable);

    void updateContactReports();

    bool mGenerateWarning = true;

    SimulationBackend& mBackend;

    long mStageId = -1;

    ::physx::PxScene* mScene = nullptr;
    ::physx::PxCudaContextManager* mCudaContextManager = nullptr;

    CUcontext mCtx = 0;
    int mDevice = -1;

    ::physx::PxU32 mNumArtis = 0;
    ::physx::PxU32 mNumRds = 0;

    ::physx::PxU32 mMaxLinks = 0;
    ::physx::PxU32 mMaxDofs = 0;
    ::physx::PxU32 mMaxFixedTendons = 0;
    ::physx::PxU32 mMaxSpatialTendons = 0;

    ::physx::PxU32 mLinkBufSize = 0;
    ::physx::PxU32 mDofBufSize = 0;
    ::physx::PxU32 mFixedTendonBufSize = 0;
    ::physx::PxU32 mSpatialTendonBufSize = 0;
    ::physx::PxU32 mMaxArtiIndex = 0;

    ::physx::PxU32 mMaxRdIndex = 0;
    // Maps PxRigidDynamic* to global RD index in GPU buffers. The actor pointer
    // is stable across disable/enable cycles (unlike the island node index which
    // PhysX invalidates when eDISABLE_SIMULATION is set), so this is the sole
    // lookup used by GpuRigidBodyView::ctor.
    std::unordered_map<::physx::PxRigidDynamic*, uint32_t> mActor2RdIndexMap;
    // Scene-wide rigid-body disable/enable generation (OMPE-94459, multi-view +
    // body disable). This struct is shared by every rigid-body view over the
    // same scene, but each view's mRdIndexDirty flag is private to that view.
    // A disable issued through one view frees/reallocates the shared GPU island
    // indices for *all* views holding that body, so each successful
    // setDisableSimulations bumps this counter; sibling views compare it against
    // their last-synced value in refreshRdGpuIndices() and rebuild when it moved,
    // instead of reading stale/invalid GPU indices.
    uint64_t mRdDisableEpoch = 0;
    ::physx::PxTransform* mRdPoseDev = nullptr;
    ::physx::PxVec3* mRdLinearVelAccDev = nullptr;
    ::physx::PxVec3* mRdAngularVelAccDev = nullptr;
    ::physx::PxVec3* mRdForcesDev = nullptr;
    ::physx::PxVec3* mRdTorquesDev = nullptr;

    // maps articulation node indices to global articulation indices in GPU buffers
    std::unordered_map<uint32_t, ::physx::PxArticulationGPUIndex> mNodeIdx2ArtiIdxMap;
    ::physx::PxU32* mNodeIdx2ArtiGpuIdxDev = nullptr;

    ::physx::PxTransform* mLinkOrRootTransformsDev = nullptr;
    ::physx::PxVec3* mLinkOrRootLinearVelAccDev = nullptr;
    ::physx::PxVec3* mLinkOrRootAngularVelAccDev = nullptr;
    float* mDofScalarsDev = nullptr;

    ::physx::PxVec3* mLinkForcesDev = nullptr;
    ::physx::PxVec3* mLinkTorquesDev = nullptr;
    float* mDofActuationForcesDev = nullptr;
    PhysxGpuSpatialForces* mLinkIncomingJointForceDev = nullptr;
    ::physx::PxGpuFixedTendonData* mFixedTendonPropertiesDev = nullptr;
    ::physx::PxGpuSpatialTendonData* mSpatialTendonPropertiesDev = nullptr;

    // centroidal momentum/bias force buffers
    float* mCentroidalMomentumDataDev = nullptr;

    // jacobian buffer
    float* mJacobianDataDev = nullptr;
    ::physx::PxU32 mJacobianMaxCols;
    ::physx::PxU32 mJacobianMaxRows;

    // mass matrix buffer buffer
    float* mMassMatrixDataDev = nullptr;

    // gravity and Coriolis compensation force buffer
    float* mCoriolisGravityDataDev = nullptr;

    // contact pairs
    // Note: actual mMaxGpuContactPairs determined from scene
    uint32_t mMaxGpuContactPairs = 512 * 1024;
    ::physx::PxGpuContactPair* mGpuContactPairsDev = nullptr;
    ::physx::PxU32* mGpuContactPairCountDev = nullptr;
    ::physx::PxU32 mNumContactPairs = 0;

    // Actor-to-pathId lookup for raw contact data (shared by all contact views)
    GpuActorPathIdPair* mActorPathLookupDev = nullptr;
    ::physx::PxU32 mNumActorPathPairs = 0;

    // deformable bodies
    std::unordered_map<::physx::PxDeformableBody*, GpuDeformableBodyRecord> mDeformableBodyToRecord;

    // synchronization events
    CUevent mCopyEvents[CopyEvent::eCOUNT] = { 0 };
    CUevent* mCopyEventPointers[CopyEvent::eCOUNT] = { 0 };
    CUevent mApplyWaitEvents[ApplyEvent::eCOUNT] = { 0 };
    CUevent mApplySignalEvents[ApplyEvent::eCOUNT] = { 0 };
    CUevent mContactReadEvent = { 0 };

    // whether forces were applied by the user
    bool mRdForcesApplied = false;
    bool mRdTorquesApplied = false;
    bool mLinkForcesApplied = false;
    bool mLinkTorquesApplied = false;
    bool mArtiDofForcesApplied = false;

private:
    void dumpGpuContactData(::physx::PxGpuContactPair* contactPairsDev, ::physx::PxU32* numContactPairsDev);
    int64_t mRigidContactTimestamp = -1;
};

using GpuSimulationDataPtr = std::shared_ptr<GpuSimulationData>;

} // namespace tensors
} // namespace physx
} // namespace omni
