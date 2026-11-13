// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CCT-001
 * @covers AC-3
 *
 * @implements REQ-TENSOR-CONTACT-005
 * @covers AC-3
 *
 * @implements REQ-TENSOR-VIEW-001
 * @covers AC-14 AC-16 AC-17
 */

#pragma once

#include <omni/physics/tensors/TensorApi.h>

#include "tensors/InstancerReframe.h"
#include "tensors/PointSetReframe.h"
#include "tensors/PhysicsTypes.h"
#include "tensors/gpu/CudaCommon.h"
#include "tensors/gpu/ThrustUtils.h"

#include <memory>
#include <unordered_map>
#include <vector>

#define GPUAPI_CHECK_READY(gpuSimData_, retval_)                                                                       \
    if (!gpuSimData_ || !gpuSimData_->checkApiReady(__FUNCTION__))                                                     \
        return (retval_);

// GPUAPI_CHECK_READY, and for a call on data laid out with the articulation strides also that they are still PhysX's.
#define GPUAPI_CHECK_READY_ARTICULATIONS(gpuSimData_, usesArticulationStrides_, retval_)                               \
    if (!gpuSimData_ || !gpuSimData_->checkApiReady(__FUNCTION__) ||                                                   \
        ((usesArticulationStrides_) && !gpuSimData_->checkArticulationStrides(__FUNCTION__)))                          \
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

// The device buffers that a DirectGPU call and one of our kernels take turns touching, named so the
// ordering between those turns can be keyed by buffer (ADR-0008 Decision 7).
//
// Two streams reach each buffer: our kernels run on the null stream, PhysX runs its DirectGPU work
// on a non-blocking stream that does not implicitly synchronize with ours. Every access alternates
// between them, so each alternation needs an event -- PhysX signals its finishEvent when its side
// is done, mKernelDoneEvents below when ours is.
//
// Keyed by buffer rather than by attribute (unlike CopyEvent and ApplyEvent) to keep independent
// work parallel: a DOF read and a rigid-pose read share no event and overlap.
//
// This must stay the COMPLETE catalogue of GpuSimulationData buffers that both PhysX and our
// kernels touch. A buffer left out looks ordered while its safety actually rests on a host block,
// so anything added to GpuSimulationData that a DirectGPU call fills or reads belongs here too.
struct SharedDeviceBuffer
{
    enum
    {
        eRdPose,
        eRdLinearVel,
        eRdAngularVel,
        eLinkOrRootTransforms,
        eLinkOrRootLinearVel,
        eLinkOrRootAngularVel,
        eDofScalars,
        eLinkIncomingJointForce,
        eFixedTendonProperties,
        eSpatialTendonProperties,
        // Filled by computeArticulationData rather than get*Data, which changes nothing here: still
        // shared per scene, still read by one of our kernels afterwards. eCoriolisGravityData
        // carries two attributes and eCentroidalMomentumData three, so back-to-back reads of a pair
        // land in the same buffer -- the case Decision 7 calls out for eRdPose.
        eJacobianData,
        eMassMatrixData,
        eCentroidalMomentumData,
        eCoriolisGravityData,
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
    ::physx::PxQuat orientationFixup{ ::physx::PxIdentity }; // controller actor to prim frame
};

// One volume deformable as the ovstage kinematic-target commit writes it (submitKinematicTargetsOvStage): the
// runtime-owned buffer PhysX adopted (null leaves the body as it is), where the body's nodes start in the caller's
// column and how many there are, and the transform from the column's sim-mesh frame into the buffer's.
struct KinematicTargetBodyRecord
{
    ::physx::PxVec4* dst = nullptr;
    ::physx::PxU32 floatOffset = 0;
    ::physx::PxU32 nodeCount = 0;
    PointSetTransform columnToBuffer;
};

// Logs that kinematic-target commits kept `count` pinned nodes pinned although the write marked them free; nothing
// when `count` is 0.
void logKeptPinnedTargets(unsigned long long count);

// One POINT SET, as the ovstage output read needs it. A deformable body's simulation mesh and a
// particle set are both this shape, which is why the record and the view over it are named for the
// shape rather than for either type.
//
// Deliberately not GpuDeformableBodyRecord, the tensor BINDING's record: that one carries element
// indices, rest positions and kinematic-target buffers the read never touches, and the view that
// builds it flips ePARTIALLY_KINEMATIC on every body it covers. A read must not change how the
// solver behaves, so it brings its own record instead.
//
// `src` points straight at the source buffer, PhysX's own for a sim mesh or a particle set and the
// runtime's skinned-vertex buffer for a deformable skin mesh, so the fetch reads it in place and
// writes the destination column once, with nothing to copy first.
struct GpuPointSetReadRecord
{
    // The point buffer, device: PxVec4 (xyz + inverse mass) for a PhysX sim-mesh or particle buffer,
    // PxVec3 for the runtime's skinned-vertex buffer. Decided per VIEW (fetchPointSetColumnOvStage's
    // `layout`), not per record, so the record stays 16 bytes.
    const void* src = nullptr;
    ::physx::PxU32 numPoints = 0;        // points in this set; the kernel's per-record bounds test
    ::physx::PxU32 dstOffsetFloats = 0;  // start of this set's slice in the destination column
};


// One point-instancer instance, in the form the reframe kernel consumes.
//
// `protoInverse` is inlined rather than shared through an index because it is constant for the life
// of the view: uploaded once when the view is built, read straight from the record with no second
// indirection in the kernel.
//
// `proto` is the FORWARD transform, for the write (ADR-0012), stored rather than inverted on the
// device: inverting a general affine -- these carry scale -- in a kernel is real work, and the host
// already holds the matrix Gf can invert in one call. Costs 12 floats per instance.
struct GpuPointInstancerRecord
{
    ::physx::PxU32 rbRow = 0xffffffff; // row in the sibling superset rigid view's record array
    ::physx::PxU32 instancerIdx = 0; // indexes the per-read world-inverse and destination-offset arrays
    ::physx::PxU32 slot = 0; // instance index within that instancer's instance array
    ::physx::PxU32 pad = 0;
    InstancerAffine protoInverse; // InternalActor::mProtoTransformInverse, in row-vector form
    InstancerAffine proto; // the FORWARD proto transform, for the write (ADR-0012); rationale above
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

// Device tables that map a contact pair's node index to a sensor row. Lookups outside a table report no
// sensor, so bodies added after the tables were built cannot index past them.
struct RigidContactReferentTables
{
    const ::physx::PxU32* nodeIdx2ArtiGpuIdx = nullptr;
    const ::physx::PxU32* rdContactIndices = nullptr;
    const ::physx::PxU32* linkContactIndices = nullptr;
    ::physx::PxU32 nodeIdx2ArtiGpuIdxCount = 0;
    ::physx::PxU32 rdContactIndexCount = 0;
    ::physx::PxU32 linkContactIndexCount = 0;
    ::physx::PxU32 maxLinks = 0;
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
    // Both return false when an allocation or copy failed, leaving what was allocated for the caller to free.
    static bool allocDevMemAndCopyH2D(void** dst, const void* src, size_t size, const char* name);
    // Frees the buffers the tensor views allocate for a body (the others belong to PhysX) and nulls them.
    static void freeRecordBuffers(GpuDeformableBodyRecord& record);
    bool uploadDeviceData(std::string debugMessage);
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

// Rigid-body disable state of one scene. SimulationBackend keeps one per scene and gives it to every
// GpuSimulationData built for that scene, so a disable through a view on one copy reaches views on the others.
struct GpuRdDisableState
{
    uint64_t epoch = 0;
    bool mayHaveDisabledRd = false;
};

// The per-articulation strides GPU simulation data is sized and indexed by: the largest link, DOF, fixed- and
// spatial-tendon counts among the scene's articulations, each raised to the stride PhysX lays its DirectGPU
// articulation blocks out with. GpuSimulationData::init sizes its buffers with them and
// SimulationBackend::sceneTopology compares them to decide whether cached data still fits the scene, so both take
// them from here.
void articulationStrides(::physx::PxScene* scene,
                         ::physx::PxU32& maxLinks,
                         ::physx::PxU32& maxDofs,
                         ::physx::PxU32& maxFixedTendons,
                         ::physx::PxU32& maxSpatialTendons);

// data shared with child views that persist even if simulation view is deleted
struct GpuSimulationData
{
    GpuSimulationData(SimulationBackend& backend,
                      omni::physics::tensors::AttachHandle attachHandle,
                      std::shared_ptr<GpuRdDisableState> rdDisableState);
    ~GpuSimulationData();

    bool init(::physx::PxScene* scene);

    // checks for illegal usage: before the first step
    bool checkApiReady(const char* function) const;
    // false while the articulation strides differ from PhysX's (a larger articulation was inserted, so this data or
    // PhysX is out of date); checked only by calls on data laid out with them
    bool checkArticulationStrides(const char* function) const;
    // whether `arti` is one this data was built for (in the scene then, with the GPU index it has now); a view over
    // one it does not describe -- inserted after the data was built -- asks PhysX for more blocks than its buffers hold
    bool describesArticulation(const ::physx::PxArticulationReducedCoordinate& arti) const;
    void clearForces();

    bool flush();

    void enableGpuUsageWarnings(bool enable);

    void updateContactReports();

    bool mGenerateWarning = true;

    SimulationBackend& mBackend;

    // Attach handle (ADR-0013), not a USD stage id.
    omni::physics::tensors::AttachHandle mAttachHandle = omni::physics::tensors::kNoAttach;

    ::physx::PxScene* mScene = nullptr;
    ::physx::PxCudaContextManager* mCudaContextManager = nullptr;

    CUcontext mCtx = 0;
    int mDevice = -1;

    ::physx::PxU32 mNumArtis = 0;
    ::physx::PxU32 mNumRds = 0;

    // The strides of the DirectGPU articulation buffers (articulationStrides), not the scene's live maxima.
    ::physx::PxU32 mMaxLinks = 0;
    ::physx::PxU32 mMaxDofs = 0;
    ::physx::PxU32 mMaxFixedTendons = 0;
    ::physx::PxU32 mMaxSpatialTendons = 0;

    ::physx::PxU32 mLinkBufSize = 0;
    ::physx::PxU32 mDofBufSize = 0;
    ::physx::PxU32 mFixedTendonBufSize = 0;
    ::physx::PxU32 mSpatialTendonBufSize = 0;

    // Record-lifetime epoch of the attach when init() ran. The per-object maps below describe the
    // records of that epoch, so the data is not reused once the epoch moves.
    uint64_t mRecordEpoch = 0;

    // Maps PxRigidDynamic* to global RD index in GPU buffers. The actor pointer
    // is stable across disable/enable cycles (unlike the island node index which
    // PhysX invalidates when eDISABLE_SIMULATION is set), so this is the sole
    // lookup used by GpuRigidBodyView::ctor.
    std::unordered_map<::physx::PxRigidDynamic*, uint32_t> mActor2RdIndexMap;
    // The scene's disable state. The two references below name its fields.
    std::shared_ptr<GpuRdDisableState> mRdDisableState;
    // Scene-wide rigid-body disable/enable generation (OMPE-94459, multi-view +
    // body disable). It is shared by every rigid-body view over the same scene,
    // on every copy of this data, but each view's mRdIndexDirty flag is private to that view.
    // A disable issued through one view frees/reallocates the shared GPU island
    // indices for *all* views holding that body, so each successful
    // setDisableSimulations bumps this counter; sibling views compare it against
    // their last-synced value in refreshRdGpuIndices() and rebuild when it moved,
    // instead of reading stale/invalid GPU indices.
    uint64_t& mRdDisableEpoch;
    // Conservative scene-wide "at least one rigid dynamic may lack a DirectGPU row". Seeded by the
    // census below (free -- that loop already reads every getGPUIndex()), set by every notified
    // disable/enable toggle, and cleared only by the scene superset view's refreshRdGpuIndices()
    // once it has walked all entries and found none. A subset view must never clear it: finding no
    // disabled body among its own entries says nothing about the rest of the scene.
    //
    // False is a promise, so it may only be produced by something that saw the whole scene; true is
    // just "scan to find out". Lets the ovstage read/write filter skip its per-body actor-flag scan
    // on the disable-free steady state, which is the common case and was costing a cache miss per
    // matched body on every I/O.
    bool& mMayHaveDisabledRd;
    ::physx::PxTransform* mRdPoseDev = nullptr;
    ::physx::PxVec3* mRdLinearVelAccDev = nullptr;
    ::physx::PxVec3* mRdAngularVelAccDev = nullptr;
    ::physx::PxVec3* mRdForcesDev = nullptr;
    ::physx::PxVec3* mRdTorquesDev = nullptr;

    // maps articulation node indices to global articulation indices in GPU buffers
    std::unordered_map<uint32_t, ::physx::PxArticulationGPUIndex> mNodeIdx2ArtiIdxMap;
    ::physx::PxU32* mNodeIdx2ArtiGpuIdxDev = nullptr;
    ::physx::PxU32 mNodeIdx2ArtiGpuIdxCount = 0;

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

    // "our kernels have finished touching this buffer" -- one per shared device buffer (ADR-0008
    // Decision 7). Handed to a DirectGPU call as its startEvent so PhysX does not touch the buffer
    // while one of our kernels still is.
    CUevent mKernelDoneEvents[SharedDeviceBuffer::eCOUNT] = { 0 };

    // Sticky: set when the completion protocol could neither record this buffer's edge nor prove by
    // draining that our kernel had finished with it. Never cleared -- there is no operation that
    // re-establishes the lost ordering, so "recovered" could only ever be a guess.
    //
    // Mutable because drainDirectGpuFinish is const and must still be able to latch: constness here
    // describes the simulation data, and a quarantine records that the data can no longer be
    // trusted rather than changing what it holds.
    mutable bool mSharedBufferQuarantined[SharedDeviceBuffer::eCOUNT] = { false };

    // The startEvent a DirectGPU call touching `buf` must be issued with. Safe before anything has
    // been recorded: CUDA treats a created-but-never-recorded event as already complete, so the
    // first DirectGPU call waits for nothing.
    CUevent kernelDoneEvent(::physx::PxU32 buf) const
    {
        return buf < SharedDeviceBuffer::eCOUNT ? mKernelDoneEvents[buf] : nullptr;
    }

    // Record that our kernels are done with `buf`. Call AFTER the kernel is enqueued -- the event
    // marks where the buffer stops being touched, so recording it earlier lets the next DirectGPU
    // call overwrite it mid-kernel.
    //
    // Record UNCONDITIONALLY on every path whose kernel could touch the buffer: the record goes on
    // the null stream, so recording where no kernel of ours ran is conservative, never optimistic.
    // Gating it on what this view holds would be a claim about other views' kernels that no call
    // site can make.
    //
    // Returns false when NEITHER leg held: the event could not be recorded and the fallback drain
    // failed too. That is the one case where the postcondition above is untrue, and it also latches
    // the buffer (see sharedBufferQuarantined) because the immediate caller cannot undo an already
    // enqueued kernel -- what must change is what happens next.
    bool recordKernelDone(::physx::PxU32 buf);

    // True once a buffer's ordering edge has been lost and could not be re-established. A read over
    // quarantined scratch is refused rather than served: its contents may be being overwritten by a
    // producer nothing is holding back.
    bool sharedBufferQuarantined(::physx::PxU32 buf) const
    {
        return buf < SharedDeviceBuffer::eCOUNT && mSharedBufferQuarantined[buf];
    }

    // Quarantine every shared buffer. Used when the failure is not attributable to one of them --
    // a failed device-wide drain says nothing about which producer is still running.
    void quarantineAllSharedBuffers(const char* reason) const;

    // Order our stream behind a DirectGPU producer's finish event. False means the producer's
    // output MUST NOT be read.
    //
    // Not simply a wait: PhysX may have queued work and recorded the finish event even when it
    // reports failure, so a caller that returns without draining leaves that work in flight over
    // scratch it is about to release or hand to another user.
    bool drainDirectGpuFinish(CUevent finishEvent, const char* label) const;

    // One DirectGPU fetch, checked. `issued` is the getter's own return value -- PhysX refuses
    // pre-step and running-state reads outright and writes NOTHING when it does, so an unchecked
    // refusal gathers whatever the scratch last held and serves it as a fresh sample. Returns true
    // only when the fetch was accepted AND our stream is ordered behind it.
    bool awaitDirectGpuFetch(bool issued, CUevent finishEvent, const char* label) const;

    // Run a gather over shared scratch and hand the buffer back, in that order, always.
    //
    // The record is unconditional because the buffer was committed to a kernel the moment the
    // launch was issued, and a gather can report failure AFTER launching -- cudaGetLastError()
    // surfaces errors from earlier asynchronous work. Returning before the record would let the
    // next DirectGPU writer overwrite scratch a live kernel is still reading, which is a corrupt
    // read or a CUDA 700 somewhere else entirely.
    template <typename Gather>
    bool gatherThenRelease(::physx::PxU32 buf, Gather&& gather)
    {
        // Refused BEFORE the gather: a quarantined buffer has no one holding its producer back, so
        // launching a kernel over it is the race, not the reporting of it.
        if (sharedBufferQuarantined(buf))
            return false;
        const bool gathered = gather();
        // A lost edge invalidates the gather's result as well: the values may have been overwritten
        // while the kernel ran, so success is not reported on a buffer that just failed to release.
        return recordKernelDone(buf) && gathered;
    }

    // For a gather that reads TWO shared buffers. Recording only one of them is the same bug in
    // half: the unrecorded buffer stays open to the next DirectGPU writer while the kernel runs.
    template <typename Gather>
    bool gatherThenRelease(::physx::PxU32 bufA, ::physx::PxU32 bufB, Gather&& gather)
    {
        if (sharedBufferQuarantined(bufA) || sharedBufferQuarantined(bufB))
            return false;
        const bool gathered = gather();
        // BOTH recorded before the result is folded in: `&&` would short-circuit the second release
        // on the first one's failure, leaving that buffer open to the next producer -- the same bug
        // in half that this two-buffer form exists to prevent.
        const bool releasedA = recordKernelDone(bufA);
        const bool releasedB = recordKernelDone(bufB);
        return releasedA && releasedB && gathered;
    }

    // A stream-ordered session commit or read (ADR-0052, ADR-0055) reads the views' tables -- the rigid-body
    // DirectGPU index map and records, the centre-of-mass table, the point-instancer frames, the articulation
    // records -- on the caller's stream, which nothing orders against the host rewriting them in place or a view
    // freeing them. Call once the operation's work is queued on `stream`, outside a capture, with the scene's CUDA
    // context current. False only when neither the record nor the fallback wait for `stream` held.
    bool noteTableReader(CUstream stream);

    // Waits on the host until every operation noted above has finished reading the tables. A view calls it before
    // freeing them. False when a wait failed.
    bool waitForTableReaders();

    // The device address of the count of pinned nodes kinematic-target commits on this scene kept pinned although the
    // write marked them free (submitKinematicTargetsOvStage), in host memory the device writes directly, so the host
    // reads it without waiting. Allocated on the first call; null when that failed. Call outside a capture, with the
    // scene's CUDA context current.
    unsigned long long* keptPinnedTargetsCounter();

    // Logs the nodes counted since the last report, if any. Waits for nothing: a count a commit is still adding to is
    // reported by a later call.
    void reportKeptPinnedTargets();

    // Copy `bytes` from the host over one of those tables, once every operation noted above is done reading it.
    // Returns once the copy has reached the device, so work queued afterwards on any stream reads the new
    // contents. Call with the scene's CUDA context current.
    bool rewriteTable(void* dst, const void* src, size_t bytes);

    // One event per operation noted above that may still be running, recorded on its own stream so operations on
    // different streams stay independent. Events found complete go back to the idle pool for later operations.
    std::vector<CUevent> mTableReaders;
    std::vector<CUevent> mIdleTableReaders;

    // whether forces were applied by the user
    bool mRdForcesApplied = false;
    bool mRdTorquesApplied = false;
    bool mLinkForcesApplied = false;
    bool mLinkTorquesApplied = false;
    bool mArtiDofForcesApplied = false;

private:
    unsigned long long* mKeptPinnedTargetsHost = nullptr;
    unsigned long long* mKeptPinnedTargetsDev = nullptr;
    unsigned long long mKeptPinnedTargetsReported = 0;

    void dumpGpuContactData(::physx::PxGpuContactPair* contactPairsDev, ::physx::PxU32* numContactPairsDev);
    int64_t mRigidContactTimestamp = -1;
};

using GpuSimulationDataPtr = std::shared_ptr<GpuSimulationData>;

} // namespace tensors
} // namespace physx
} // namespace omni
