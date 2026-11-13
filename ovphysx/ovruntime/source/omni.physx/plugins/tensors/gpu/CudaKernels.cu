// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-TENSOR-CONTACT-004
// @covers AC-1 AC-2

/**
 * @implements REQ-READ-CCT-001
 * @covers AC-3
 *
 * @implements REQ-TENSOR-WRENCH-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-TENSOR-CONTACT-003
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-READ-CORE-001
 * @covers AC-6
 *
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-2, AC-6
 *
 * @implements REQ-READ-ARTICULATION-001
 * @covers AC-3 AC-4 AC-9
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-15, AC-17, AC-21, AC-30
 *
 * @implements REQ-READ-CONTACT-001
 * @covers AC-11
 *
 * @implements REQ-READ-CONTACT-005
 * @covers AC-2
 *
 * @implements REQ-READ-INVDYN-001
 * @covers AC-8, AC-10
 *
 * @implements REQ-READ-COVERAGE-001
 * @covers AC-4c, AC-4d
 *
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-16
 *
 * @implements REQ-INPUT-CORE-001
 * @covers AC-3 AC-5
 *
 * @implements REQ-TENSOR-CONTACT-002
 * @covers AC-2 AC-3
 *
 * @implements REQ-READ-SKIN-001
 * @covers AC-2
 * @implements REQ-READ-SDF-001
 * @covers AC-3
 *
 * @implements REQ-TENSOR-CONTACT-005
 * @covers AC-3
 *
 * @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
 * @covers AC-7 AC-14 AC-20 AC-21
 */

#include "tensors/gpu/CudaKernels.h"
#include "tensors/gpu/GpuSimulationData.h"
#include "tensors/gpu/GpuPointSetReadView.h"
#include "tensors/ArticulationDofOvStageRecord.h"
#include "tensors/ArticulationLinkOvStageRecord.h"
#include "tensors/ArticulationTendonOvStageRecord.h"
#include "tensors/InstancerReframe.h"
#include "tensors/PointSetReframe.h"
#include "tensors/ContactLayoutClamp.h"
#include "tensors/gpu/ThrustUtils.h"

#include <thrust/device_ptr.h>
#include <thrust/copy.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/scan.h>
#include <thrust/device_vector.h>

#include <PxContact.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{


// cudaMemset may reject a caller allocation from another CUDA context on Windows.
// Contact kernels already access that allocation through its unified device address;
// initialize it with the same access and stream ordering as those kernels.
__global__ void clearContactBufferKernel(unsigned char* destination, size_t bytes)
{
    for (size_t i = size_t(blockIdx.x) * blockDim.x + threadIdx.x; i < bytes;
         i += size_t(blockDim.x) * gridDim.x)
        destination[i] = 0;
}

bool clearContactBuffer(void* destination, size_t bytes)
{
    if (bytes == 0)
        return true;
    if (!destination)
        return false;
    const size_t blocks = (bytes - 1) / 256 + 1;
    clearContactBufferKernel<<<static_cast<unsigned>(blocks < 65535 ? blocks : 65535), 256>>>(
        static_cast<unsigned char*>(destination), bytes);
    return CHECK_CUDA(cudaGetLastError());
}

//
// Articulation dirty indices
//


namespace
{
class IsArtiDirtyFlagSet
{
public:
    explicit IsArtiDirtyFlagSet(ArticulationGpuFlag::Enum flag) : mFlag(flag)
    {
    }

    __host__ __device__ __forceinline__ bool operator()(const ArticulationGpuFlags& value) const
    {
        return (value & mFlag) == mFlag;
    }

private:
    const ArticulationGpuFlag::Enum mFlag;
};
}
// extract dirty indices from allIndices based on artiDirtyFlags and flag
PxU32 fillArtiDirtyIndices(SingleAllocPolicy& policy,
                           PxU32* indicesRet,
                           const PxU32* allIndices,
                           const ArticulationGpuFlags* artiDirtyFlags,
                           const ArticulationGpuFlag::Enum flag,
                           const PxU32 numArtis)
{
    thrust::device_ptr<const ArticulationGpuFlags> dirtyFlagsPtr(artiDirtyFlags);
    thrust::device_ptr<uint32_t> dirtyIndicesPtr(indicesRet);
    thrust::device_ptr<uint32_t> dirtyIndicesEnd = thrust::copy_if(
        policy, allIndices, allIndices + numArtis, dirtyFlagsPtr, dirtyIndicesPtr, IsArtiDirtyFlagSet(flag));
    PxU32 numDirtyIndices = PxU32(dirtyIndicesEnd - dirtyIndicesPtr);
    return numDirtyIndices;
}

// extract dirty indices from allIndices based on artiDirtyFlags and flag
PxU32 fillArtiTransforms(SingleAllocPolicy &policy, PxTransform *transformDev, PxArticulationGPUIndex *indicesRet,
                         const PxU32 *allIndices, const ArticulationGpuFlags *artiDirtyFlags,
                         const ArticulationGpuFlag::Enum flag, const PxU32 numArtis)
{
    thrust::device_ptr<const ArticulationGpuFlags> dirtyFlagsPtr(artiDirtyFlags);
    thrust::device_ptr<PxArticulationGPUIndex> dirtyIndicesPtr(indicesRet);
    thrust::device_ptr<PxTransform> dirtyTransformPtr(transformDev);
    thrust::device_ptr<PxArticulationGPUIndex> dirtyIndicesEnd = thrust::copy_if(
        policy, allIndices, allIndices + numArtis, dirtyFlagsPtr, dirtyIndicesPtr, IsArtiDirtyFlagSet(flag));
    thrust::device_ptr<PxTransform> dirtyTransformsEnd = thrust::copy_if(
        policy, transformDev, transformDev + numArtis, dirtyFlagsPtr, dirtyTransformPtr, IsArtiDirtyFlagSet(flag));
    PxU32 numDirtyIndices = PxU32(dirtyIndicesEnd - dirtyIndicesPtr);
    // printKernel<<<(numDirtyIndices + 1023) / 1024, 1024>>>(transformDev, indicesRet, numDirtyIndices);
    return numDirtyIndices;
}

// extract dirty indices from allIndices based on artiDirtyFlags and flag
PxU32 fillArtiVelocities(SingleAllocPolicy &policy, PxVec3 *linearVelDev, PxVec3 *angularVelDev,
                         PxArticulationGPUIndex *indicesRet, const PxU32 *allIndices,
                         const ArticulationGpuFlags *artiDirtyFlags, const ArticulationGpuFlag::Enum flag,
                         const PxU32 numArtis)
{
    thrust::device_ptr<const ArticulationGpuFlags> dirtyFlagsPtr(artiDirtyFlags);
    thrust::device_ptr<PxArticulationGPUIndex> dirtyIndicesPtr(indicesRet);
    thrust::device_ptr<PxVec3> dirtyLinVelPtr(linearVelDev);
    thrust::device_ptr<PxVec3> dirtyAngVelPtr(angularVelDev);
    thrust::device_ptr<PxArticulationGPUIndex> dirtyIndicesEnd = thrust::copy_if(
        policy, allIndices, allIndices + numArtis, dirtyFlagsPtr, dirtyIndicesPtr, IsArtiDirtyFlagSet(flag));
    thrust::device_ptr<PxVec3> dirtyLinVelEnd = thrust::copy_if(
        policy, linearVelDev, linearVelDev + numArtis, dirtyFlagsPtr, dirtyLinVelPtr, IsArtiDirtyFlagSet(flag));
    thrust::device_ptr<PxVec3> dirtyAngVelEnd = thrust::copy_if(
        policy, angularVelDev, angularVelDev + numArtis, dirtyFlagsPtr, dirtyAngVelPtr, IsArtiDirtyFlagSet(flag));
    PxU32 numDirtyIndices = PxU32(dirtyIndicesEnd - dirtyIndicesPtr);
    return numDirtyIndices;
}

// Post-launch status with a LIFETIME guarantee: a false return proves nothing from this launch is
// still running.
//
// cudaGetLastError() reports the last error from any prior runtime call on this thread and clears
// it, so it can surface a previously latched ASYNCHRONOUS error even when this launch was enqueued
// perfectly well. Returning false on that alone tells the caller "no work happened" while the
// kernel is still writing its destination -- and every caller here treats false as licence to free
// or reuse that destination.
//
// Draining on the failure path makes the contract true. Only on that path: draining on success
// would throw away the asynchrony the whole read design depends on.
//
// Best effort by nature. A sticky error fails the drain too, and nothing finer exists once the
// context is in that state; the caller still learns the launch failed, and anything that WAS
// enqueued has been waited for.
static bool launchOk(bool checkedStatus)
{
    if (!checkedStatus)
        cudaStreamSynchronize(nullptr);
    return checkedStatus;
}

// launchOk for a launch on `stream`. A stream being captured is not drained: synchronizing it would
// invalidate the capture, and the failed launch has already invalidated it anyway.
static bool drainOnLaunchFailure(bool checkedStatus, cudaStream_t stream)
{
    if (!checkedStatus)
    {
        cudaStreamCaptureStatus capture = cudaStreamCaptureStatusNone;
        if (cudaStreamIsCapturing(stream, &capture) == cudaSuccess && capture == cudaStreamCaptureStatusNone)
            cudaStreamSynchronize(stream);
    }
    return checkedStatus;
}

// A row selected by a masked commit (ADR-0052): one byte per row, nonzero selects it. No mask selects all.
__device__ static inline bool isRowCommitted(const uint8_t* mask, PxU32 row)
{
    return !mask || mask[row] != 0;
}

__global__ static void makeArtiLinksDirtyKernel(ArticulationGpuFlags *artiLinksDirtyFlags,
                                                const ArticulationGpuFlags *artiDirtyFlags,
                                                const ArticulationGpuFlag::Enum mFlag, const PxU32 numLinks,
                                                const PxU32 maxLinks)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 arti = i / maxLinks;
    if (i < numLinks)
    {
        if ((artiDirtyFlags[arti] & mFlag) == mFlag){
            artiLinksDirtyFlags[i] |= mFlag;
        }
    }
}

PxU32 fillArtiFT(SingleAllocPolicy &policy, PxVec3 *FT, PxArticulationGPUIndex *indicesRet, const PxU32 *allIndices,
                 const ArticulationGpuFlags *artiDirtyFlags, ArticulationGpuFlags *artiLinksDirtyFlags,
                 const ArticulationGpuFlag::Enum flag, const PxU32 numArtis, const PxU32 maxLinks)
{
    thrust::device_ptr<const ArticulationGpuFlags> dirtyFlagsPtr(artiDirtyFlags);
    thrust::device_ptr<const ArticulationGpuFlags> dirtyLinksFlagsPtr(artiLinksDirtyFlags);
    thrust::device_ptr<PxArticulationGPUIndex> dirtyIndicesPtr(indicesRet);
    thrust::device_ptr<PxVec3> dirtyFTPtr(FT);
    thrust::device_ptr<PxArticulationGPUIndex> dirtyIndicesEnd = thrust::copy_if(
        policy, allIndices, allIndices + numArtis, dirtyFlagsPtr, dirtyIndicesPtr, IsArtiDirtyFlagSet(flag));
    PxU32 numDirtyIndices = PxU32(dirtyIndicesEnd - dirtyIndicesPtr);
    // Make sure all links of an articulation made dirty when a single link is made dirty
        makeArtiLinksDirtyKernel<<<(numArtis * maxLinks + 1023) / 1024, 1024>>>(artiLinksDirtyFlags, artiDirtyFlags,
                                                                                flag, numArtis * maxLinks, maxLinks);
    thrust::device_ptr<PxVec3> dirtyForceEnd =
        thrust::copy_if(policy, FT, FT + numArtis * maxLinks, dirtyLinksFlagsPtr, dirtyFTPtr, IsArtiDirtyFlagSet(flag));
    CHECK_CUDA(cudaStreamSynchronize(nullptr));
    return numDirtyIndices;
}
//
// Rigid dynamic dirty indices
//

namespace
{
class IsRdDirtyFlagSet
{
public:
    explicit IsRdDirtyFlagSet(ActorGpuFlag::Enum flag) : mFlag(flag)
    {
    }

    __host__ __device__ __forceinline__ bool operator()(const ActorGpuFlags& value) const
    {
        return (value & mFlag) == mFlag;
    }

private:
    const ActorGpuFlag::Enum mFlag;
};
} // namespace

PxU32 fillRdTransforms(SingleAllocPolicy &policy, PxTransform *transformDev,
                       ::physx::PxRigidDynamicGPUIndex *indicesRet, const PxU32 *allRdIndices,
                       const ActorGpuFlags *rdDirtyFlags, const ActorGpuFlag::Enum flag, const PxU32 numRds)
{
    thrust::device_ptr<const ActorGpuFlags> dirtyFlagsPtr(rdDirtyFlags);
    thrust::device_ptr<PxTransform> dirtyTransformPtr(transformDev);
    thrust::device_ptr<::physx::PxRigidDynamicGPUIndex> dirtyIndicesPtr(indicesRet);
    thrust::device_ptr<::physx::PxRigidDynamicGPUIndex> dirtyIndicesEnd = thrust::copy_if(
        policy, allRdIndices, allRdIndices + numRds, dirtyFlagsPtr, dirtyIndicesPtr, IsRdDirtyFlagSet(flag));
    thrust::device_ptr<PxTransform> dirtyTrasnfromsEnd = thrust::copy_if(
        policy, transformDev, transformDev + numRds, dirtyFlagsPtr, dirtyTransformPtr, IsRdDirtyFlagSet(flag));
    PxU32 numDirtyIndices = PxU32(dirtyIndicesEnd - dirtyIndicesPtr);
    // printKernel<<<(numDirtyIndices + 1023) / 1024, 1024>>>(transformDev, indicesRet, numDirtyIndices);
    return numDirtyIndices;
}

PxU32 fillRdVelocities(SingleAllocPolicy &policy, PxVec3 *linVelDev, PxVec3 *angVelDev,
                       ::physx::PxRigidDynamicGPUIndex *indicesRet, const PxU32 *allRdIndices,
                       const ActorGpuFlags *rdDirtyFlags, const ActorGpuFlag::Enum flag, const PxU32 numRds)
{
    thrust::device_ptr<const ActorGpuFlags> dirtyFlagsPtr(rdDirtyFlags);
    thrust::device_ptr<::physx::PxRigidDynamicGPUIndex> dirtyIndicesPtr(indicesRet);
    thrust::device_ptr<PxVec3> dirtyLinVelPtr(linVelDev);
    thrust::device_ptr<PxVec3> dirtyAngVelPtr(angVelDev);
    thrust::device_ptr<::physx::PxRigidDynamicGPUIndex> dirtyIndicesEnd = thrust::copy_if(
        policy, allRdIndices, allRdIndices + numRds, dirtyFlagsPtr, dirtyIndicesPtr, IsRdDirtyFlagSet(flag));
    thrust::device_ptr<PxVec3> dirtyLinVelEnd =
        thrust::copy_if(policy, linVelDev, linVelDev + numRds, dirtyFlagsPtr, dirtyLinVelPtr, IsRdDirtyFlagSet(flag));
    thrust::device_ptr<PxVec3> dirtyAngVelEnd =
        thrust::copy_if(policy, angVelDev, angVelDev + numRds, dirtyFlagsPtr, dirtyAngVelPtr, IsRdDirtyFlagSet(flag));

    PxU32 numDirtyIndices = PxU32(dirtyIndicesEnd - dirtyIndicesPtr);
    // printKernel<<<(numDirtyIndices + 1023) / 1024, 1024>>>(transformDev, indicesRet, numDirtyIndices);
    return numDirtyIndices;
}

PxU32 fillRdFT(SingleAllocPolicy &policy, PxVec3 *FT, ::physx::PxRigidDynamicGPUIndex *indicesRet,
               const PxU32 *allRdIndices, const ActorGpuFlags *rdDirtyFlags, const ActorGpuFlag::Enum flag,
               const PxU32 numRds)
{
    thrust::device_ptr<const ActorGpuFlags> dirtyFlagsPtr(rdDirtyFlags);
    thrust::device_ptr<::physx::PxRigidDynamicGPUIndex> dirtyIndicesPtr(indicesRet);
    thrust::device_ptr<PxVec3> dirtyFTPtr(FT);
    thrust::device_ptr<::physx::PxRigidDynamicGPUIndex> dirtyIndicesEnd = thrust::copy_if(
        policy, allRdIndices, allRdIndices + numRds, dirtyFlagsPtr, dirtyIndicesPtr, IsRdDirtyFlagSet(flag));
    thrust::device_ptr<PxVec3> dirtyLinVelEnd =
        thrust::copy_if(policy, FT, FT + numRds, dirtyFlagsPtr, dirtyFTPtr, IsRdDirtyFlagSet(flag));
    PxU32 numDirtyIndices = PxU32(dirtyIndicesEnd - dirtyIndicesPtr);
    // printKernel<<<(numDirtyIndices + 1023) / 1024, 1024>>>(transformDev, indicesRet, numDirtyIndices);
    return numDirtyIndices;
}

void exclusiveScan(PxU32 *dstCounts, PxU32 *dstStartIndices, const PxU32 numElem)
{
    thrust::device_ptr<uint32_t> countPtr(dstCounts);
    thrust::device_ptr<uint32_t> StartPtr(dstStartIndices);
    thrust::exclusive_scan(countPtr, countPtr + numElem, StartPtr);
}

//
// Articulation root transforms
//

__global__ static void fetchArtiRootTransformsKernel(TensorTransform *dst, const PxTransform *src,
                                                     const PxU32 numArti, const GpuArticulationRootRecord *rootRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numArti)
    {
            const PxTransform &srcTransform = src[i];
            TensorTransform &dstTransform = dst[i];
            dstTransform.p = srcTransform.p - rootRecords[i].origin;
            dstTransform.q = srcTransform.q;
    }
}

bool fetchArtiRootTransforms(TensorTransform *dst, const PxTransform *src, const PxU32 numArti,
                             const GpuArticulationRootRecord *rootRecords)
{
    fetchArtiRootTransformsKernel<<<(numArti + 1023) / 1024, 1024>>>(dst, src, numArti, rootRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitArtiRootTransformsKernel(PxTransform* dst,
                                                      const TensorTransform* src,
                                                      const PxU32* srcIndices,
                                                      PxU32* dirtyArtiGpuIndices,
                                                      const PxU32 numIndices,
                                                      const PxU32 numArti,
                                                      const GpuArticulationRootRecord* rootRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numIndices)
    {
        PxU32 srcIdx = srcIndices[i];
        if(srcIdx< numArti)
        {
            dirtyArtiGpuIndices[i] = rootRecords[srcIdx].physxArtiIdx;
            const TensorTransform &srcTransform = src[srcIdx];
            PxTransform &dstTransform = dst[i];
            dstTransform.p = srcTransform.p + rootRecords[srcIdx].origin;
            dstTransform.q = srcTransform.q;
        }
    }
}

bool submitArtiRootTransforms(PxTransform* dst,
                              const TensorTransform* src,
                              const PxU32* srcIndices,
                              PxU32* dirtyArtiGpuIndices,
                              const PxU32 numIndices,
                              const PxU32 numArti,
                              const GpuArticulationRootRecord* rootRecords)
{
    submitArtiRootTransformsKernel<<<(numIndices + 1023) / 1024, 1024>>>(dst, src, srcIndices, dirtyArtiGpuIndices, numIndices, numArti, rootRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

//
// Articulation root velocities
//

__global__ static void fetchArtiRootVelocitiesKernel(TensorVelAcc *dst, const PxVec3 *srcLin, const PxVec3 *srcAng,
                                                     const PxU32 numArti)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numArti)
    {
        TensorVelAcc &v = dst[i];
        v.linear = srcLin[i];
        v.angular = srcAng[i];
    }
}

bool fetchArtiRootVelocities(TensorVelAcc *dst, const PxVec3 *srcLin, const PxVec3 *srcAng, const PxU32 numArti)
{
    fetchArtiRootVelocitiesKernel<<<(numArti + 1023) / 1024, 1024>>>(dst, srcLin, srcAng, numArti);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchArtiRootPoseColumnOvStageKernel(float* dst,
                                                            const PxTransform* src,
                                                            const PxU32 count,
                                                            const bool orientation)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count)
    {
        const PxTransform& pose = src[i];
        if (orientation)
        {
            float* const output = dst + size_t(i) * 4;
            output[0] = pose.q.x;
            output[1] = pose.q.y;
            output[2] = pose.q.z;
            output[3] = pose.q.w;
        }
        else
        {
            float* const output = dst + size_t(i) * 3;
            output[0] = pose.p.x;
            output[1] = pose.p.y;
            output[2] = pose.p.z;
        }
    }
}

bool fetchArtiRootPoseColumnOvStage(float* dst, const PxTransform* src, PxU32 count, bool orientation,
                                    cudaStream_t stream)
{
    if (count == 0)
    {
        return true;
    }
    fetchArtiRootPoseColumnOvStageKernel<<<(count + 1023) / 1024, 1024, 0, stream>>>(dst, src, count, orientation);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void fetchArtiRootVelocityColumnOvStageKernel(float* dst, const PxVec3* src, const PxU32 count)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count)
    {
        float* const output = dst + size_t(i) * 3;
        output[0] = src[i].x;
        output[1] = src[i].y;
        output[2] = src[i].z;
    }
}

bool fetchArtiRootVelocityColumnOvStage(float* dst, const PxVec3* src, PxU32 count, cudaStream_t stream)
{
    if (count == 0)
    {
        return true;
    }
    fetchArtiRootVelocityColumnOvStageKernel<<<(count + 1023) / 1024, 1024, 0, stream>>>(dst, src, count);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// Standalone subspace-origin subtraction: computeArticulationData writes the destination itself, so
// there is no gather to fold it into. Row-indexed by construction -- the sole caller reads every row
// and passes no record count, so slot i is record i; a selected-row caller would have to pass one.
__global__ static void applySubspaceOriginArtiMassCentersOvStageKernel(PxVec3* dst,
                                                                       const PxU32 count,
                                                                       const GpuArticulationRootRecord* rootRecords)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count)
    {
        dst[i] -= rootRecords[i].origin;
    }
}

bool applySubspaceOriginArtiMassCentersOvStage(PxVec3* dst,
                                               PxU32 count,
                                               const GpuArticulationRootRecord* rootRecords)
{
    if (count == 0)
    {
        return true;
    }
    applySubspaceOriginArtiMassCentersOvStageKernel<<<(count + 1023) / 1024, 1024>>>(dst, count, rootRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitArtiRootVelocitiesKernel(PxVec3 *dstLin, PxVec3 *dstAng, const TensorVelAcc *src,
                                                      const PxU32 *srcIndices, PxU32 *dirtyArtiGpuIndices,
                                                      const PxU32 numIndices, const PxU32 numArti,
                                                      const GpuArticulationRootRecord *rootRecords)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numIndices)
    {
        PxU32 srcIdx = srcIndices[i];
        if(srcIdx< numArti)
        {
            dirtyArtiGpuIndices[i] = rootRecords[srcIdx].physxArtiIdx;
            const TensorVelAcc &srcVels = src[srcIdx];
            dstLin[i] = srcVels.linear;
            dstAng[i] = srcVels.angular;
        }
    }
}

bool submitArtiRootVelocities(PxVec3 *dstLin, PxVec3 *dstAng, const TensorVelAcc *src, const PxU32 *srcIndices,
                              PxU32 *dirtyArtiGpuIndices, const PxU32 numIndices, const PxU32 numArti,
                              const GpuArticulationRootRecord *rootRecords)
{
    submitArtiRootVelocitiesKernel<<<(numIndices + 1023) / 1024, 1024>>>(
        dstLin, dstAng, src, srcIndices, dirtyArtiGpuIndices, numIndices, numArti, rootRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchArtiLinkTransformsKernel(TensorTransform *dst, const PxTransform *src, const PxU32 numLinks,
                                                     const PxU32 maxLinks, const PxU32 simMaxLinks,
                                                     const GpuArticulationLinkRecord *linkRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numLinks)
    {
        PxU32 linkIdx = i % maxLinks;
        PxU32 artiIndex = i / maxLinks;
        PxU32 srcIdx = artiIndex * simMaxLinks + linkIdx;
        const PxTransform &srcTransform = src[srcIdx];
        TensorTransform &dstTransform = dst[i];
        dstTransform.p = srcTransform.p - linkRecords[i].origin;
        dstTransform.q = srcTransform.q;
    }
}

bool fetchArtiLinkTransforms(TensorTransform *dst, const PxTransform *src, const PxU32 numLinks, const PxU32 maxLinks,
                             const PxU32 simMaxLinks, const GpuArticulationLinkRecord *linkRecords)
{
    fetchArtiLinkTransformsKernel<<<(numLinks + 1023) / 1024, 1024>>>(dst, src, numLinks, maxLinks, simMaxLinks,
                                                                      linkRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchArtiLinkVelocitiesAccelerationsKernel(TensorVelAcc *dst, const PxVec3 *linkLinVelAcc,
                                                                  const PxVec3 *linkAngVelAcc, const PxU32 numLinks,
                                                                  const PxU32 maxLinks, const PxU32 simMaxLinks)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numLinks)
    {
        PxU32 linkIdx = i % maxLinks;
        PxU32 artiIndex = i / maxLinks;
        PxU32 srcIdx = artiIndex * simMaxLinks + linkIdx;
        TensorVelAcc& dstVelAcc = dst[i];
        dstVelAcc.linear = linkLinVelAcc[srcIdx];
        dstVelAcc.angular = linkAngVelAcc[srcIdx];
    }
}

bool fetchArtiLinkVelocitiesAccelerations(TensorVelAcc *dst, const PxVec3 *linkLinVelAcc, const PxVec3 *linkAngVelAcc,
                                          const PxU32 numLinks, const PxU32 maxLinks, const PxU32 simMaxLinks)
{
    fetchArtiLinkVelocitiesAccelerationsKernel<<<(numLinks + 1023) / 1024, 1024>>>(dst, linkLinVelAcc, linkAngVelAcc,
                                                                                   numLinks, maxLinks, simMaxLinks);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__device__ __forceinline__ static float bodyOrderSign(const PxU32 generalizedCoord,
                                                      const PxU32 rootDofs,
                                                      const PxU32 dofRecordBase,
                                                      const GpuArticulationDofRecord* dofRecords)
{
    if (generalizedCoord < rootDofs)
    {
        return 1.0f;
    }
    return dofRecords[dofRecordBase + generalizedCoord - rootDofs].body0IsParent ? 1.0f : -1.0f;
}

__global__ static void fetchArtiMassMatricesKernel(float* dst,
                                                   const float* src,
                                                   const PxU32 numElements,
                                                   const PxU32 massMatrixSize,
                                                   const PxU32 simMassMatrixSize,
                                                   const PxU32 generalizedCoords,
                                                   const PxU32 rootDofs,
                                                   const PxU32 dofRecordBase,
                                                   const GpuArticulationDofRecord* dofRecords,
                                                   const bool applyBodyOrderSign)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numElements)
    {
        PxU32 elemIdx = i % massMatrixSize;
        PxU32 artiIndex = i / massMatrixSize;
        PxU32 srcIdx = artiIndex * simMassMatrixSize + elemIdx;
        float value = src[srcIdx];
        if (applyBodyOrderSign)
        {
            PxU32 row = elemIdx / generalizedCoords;
            PxU32 col = elemIdx % generalizedCoords;
            value *= bodyOrderSign(row, rootDofs, dofRecordBase, dofRecords) *
                     bodyOrderSign(col, rootDofs, dofRecordBase, dofRecords);
        }
        dst[i] = value;
    }
}

bool fetchArtiMassMatrices(float* dst,
                           const float* src,
                           const PxU32 numElements,
                           const PxU32 massMatrixSize,
                           const PxU32 simMassMatrixSize,
                           const PxU32 generalizedCoords,
                           const PxU32 rootDofs,
                           const PxU32 dofRecordBase,
                           const GpuArticulationDofRecord* dofRecords,
                           bool applyBodyOrderSign, cudaStream_t stream)
{
    fetchArtiMassMatricesKernel<<<(numElements + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, numElements, massMatrixSize, simMassMatrixSize, generalizedCoords, rootDofs, dofRecordBase,
        dofRecords, applyBodyOrderSign);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void fetchArtiDofAttributeGravityAndCoriolisKernel(float* dst,
                                                                     const float* src,
                                                                     const PxU32 numDofs,
                                                                     const PxU32 maxDofs,
                                                                     const PxU32 simMaxDofs,
                                                                     const GpuArticulationDofRecord* dofRecords,
                                                                     const bool hasRootDofs)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numDofs)
    {
        PxU32 srcDofIdx = i % maxDofs;
        PxU32 artiIndex = i / maxDofs;
        PxU32 srcIdx = artiIndex * simMaxDofs + srcDofIdx;
        if (hasRootDofs)
        {
            if (srcDofIdx < 6)
            {
                dst[i] = src[srcIdx];
            }
            else
            {
                PxU32 srcArtiDofIdx = artiIndex * (maxDofs - 6) + srcDofIdx - 6;
                bool body0IsParent = dofRecords[srcArtiDofIdx].body0IsParent;
                dst[i] = body0IsParent ? src[srcIdx] : -src[srcIdx];
            }
        }
        else
        {
            bool body0IsParent = dofRecords[i].body0IsParent;
            dst[i] = body0IsParent ? src[srcIdx] : -src[srcIdx];
        }
    }
}

// Selected-row twin of fetchArtiDofAttributeGravityAndCoriolis below: a cohort read selects a subset,
// so the output slot no longer indexes the view row. No row list is needed -- a cohort shares one
// interned metatype, so body0IsParent for dof j is common to every row and the caller passes a single
// record-block offset from its first row (asserted: a wrong offset flips a sign rather than failing).
__global__ static void fetchArtiGeneralizedForceColumnOvStageKernel(float* dst,
                                                                    const float* src,
                                                                    const PxU32 total,
                                                                    const PxU32 width,
                                                                    const PxU32 rootDofs,
                                                                    const PxU32 simMaxDofs,
                                                                    const PxU32 dofRecordBase,
                                                                    const GpuArticulationDofRecord* dofRecords)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < total)
    {
        const PxU32 slot = i / width;
        const PxU32 lane = i % width;
        const float value = src[slot * simMaxDofs + lane];
        // The leading six entries of a floating base are root terms, not dofs, so they carry no
        // parentage and take no sign flip.
        dst[i] = (lane < rootDofs) ? value :
                 (dofRecords[dofRecordBase + (lane - rootDofs)].body0IsParent ? value : -value);
    }
}

bool fetchArtiGeneralizedForceColumnOvStage(float* dst,
                                            const float* src,
                                            const PxU32 count,
                                            const PxU32 width,
                                            const PxU32 rootDofs,
                                            const PxU32 simMaxDofs,
                                            const PxU32 dofRecordBase,
                                            const GpuArticulationDofRecord* dofRecords, cudaStream_t stream)
{
    const PxU32 total = count * width;
    if (total == 0)
    {
        return true;
    }
    fetchArtiGeneralizedForceColumnOvStageKernel<<<(total + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, total, width, rootDofs, simMaxDofs, dofRecordBase, dofRecords);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

bool fetchArtiDofAttributeGravityAndCoriolis(float* dst,
                                             const float* src,
                                             const PxU32 numDofs,
                                             const PxU32 maxDofs,
                                             const PxU32 simMaxDofs,
                                             const GpuArticulationDofRecord* dofRecords,
                                             const bool hasRootDofs)
{
    fetchArtiDofAttributeGravityAndCoriolisKernel<<<(numDofs + 1023) / 1024, 1024>>>(dst, src, numDofs, maxDofs, simMaxDofs, dofRecords, hasRootDofs);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}


// Takes no simMaxDofs: source rows are (maxDofs + 6) wide because PhysX packs each articulation's own
// matrix at the start of its max-sized slot rather than at the scene's width.
__global__ static void fetchArtiCentroidalMomentumMatricesKernel(float* dst,
                                                                 const float* src,
                                                                 const PxU32 numElem,
                                                                 const PxU32 maxDofs,
                                                                 const PxU32 cenMomBlockSize,
                                                                 const PxU32 simCenMomBlockSize,
                                                                 const PxU32 startSimBiasForceBlock,
                                                                 const PxU32 dofRecordBase,
                                                                 const GpuArticulationDofRecord* dofRecords,
                                                                 const bool applyBodyOrderSign)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numElem)
    {
        PxU32 artiIndex = i / cenMomBlockSize;
        PxU32 elemIdx = i % cenMomBlockSize;
        PxU32 rowIdx = elemIdx / (maxDofs + 7);
        PxU32 colIdx = elemIdx % (maxDofs + 7);
        // Note: i = artiIndex * cenMomBlockSize + rowIdx * (maxDofs + 7) + colIdx;
        if (colIdx < 6 + maxDofs)
        {
            PxU32 srcIdx = artiIndex * simCenMomBlockSize + rowIdx * (maxDofs + 6) + colIdx;
            float value = src[srcIdx];
            if (applyBodyOrderSign)
            {
                value *= bodyOrderSign(colIdx, 6u, dofRecordBase, dofRecords);
            }
            dst[i] = value;
        }
        else if (colIdx == 6 + maxDofs){
            // SDK bias forces are aligned at the end of the data matrix block
            PxU32 srcIdx = startSimBiasForceBlock + artiIndex * 6 + rowIdx;
            dst[i] = src[srcIdx];
        }
    }
}

bool fetchArtiCentroidalMomentumMatrices(float* dst,
                                         const float* src,
                                         const PxU32 numElem,
                                         const PxU32 maxDofs,
                                         const PxU32 cenMomBlockSize,
                                         const PxU32 simCenMomBlockSize,
                                         const PxU32 startSimBiasForceBlock,
                                         const PxU32 dofRecordBase,
                                         const GpuArticulationDofRecord* dofRecords,
                                         bool applyBodyOrderSign, cudaStream_t stream)
{
    // Note:
    // cenMomBlockSize =  (maxDofs + 7) * 6;
    // simCenMomBlockSize =  (mMaxDofs + 6) * 6;
    // numElem = numArti * cenMomBlockSize;
    fetchArtiCentroidalMomentumMatricesKernel<<<(numElem + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, numElem, maxDofs, cenMomBlockSize, simCenMomBlockSize, startSimBiasForceBlock, dofRecordBase,
        dofRecords, applyBodyOrderSign);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void fetchArtiJacobianKernel(float* dst,
                                               const float* src,
                                               const PxU32 numElements,
                                               const PxU32 jacobianSize,
                                               const PxU32 simJacobianSize,
                                               const PxU32 jacobianCols,
                                               const PxU32 rootDofs,
                                               const PxU32 dofRecordBase,
                                               const GpuArticulationDofRecord* dofRecords,
                                               const bool applyBodyOrderSign)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numElements)
    {
        PxU32 elem = i % (jacobianSize);
        PxU32 artiIndex = i / jacobianSize;
        PxU32 srcIdx = artiIndex * simJacobianSize + elem;
        float value = src[srcIdx];
        if (applyBodyOrderSign)
        {
            PxU32 col = elem % jacobianCols;
            value *= bodyOrderSign(col, rootDofs, dofRecordBase, dofRecords);
        }
        dst[i] = value;
    }
}

bool fetchArtiJacobian(float* dst,
                       const float* src,
                       const PxU32 numElements,
                       const PxU32 jacobianSize,
                       const PxU32 simJacobianSize,
                       const PxU32 jacobianCols,
                       const PxU32 rootDofs,
                       const PxU32 dofRecordBase,
                       const GpuArticulationDofRecord* dofRecords,
                       bool applyBodyOrderSign, cudaStream_t stream)
{
    fetchArtiJacobianKernel<<<(numElements + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, numElements, jacobianSize, simJacobianSize, jacobianCols, rootDofs, dofRecordBase, dofRecords,
        applyBodyOrderSign);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

//
// Generic DOF attributes
//

__global__ static void fetchArtiDofAttributeKernel(float* dst,
                                                   const float* src,
                                                   const PxU32 numDofs,
                                                   const PxU32 maxDofs,
                                                   const PxU32 simMaxDofs,
                                                   const GpuArticulationDofRecord* dofRecords)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numDofs)
    {
        PxU32 srcDofIdx = i % maxDofs;
        PxU32 artiIndex = i / maxDofs;
        bool body0IsParent = dofRecords[i].body0IsParent;
        PxU32 srcIdx = artiIndex * simMaxDofs + srcDofIdx;
        dst[i] = body0IsParent ? src[srcIdx] : -src[srcIdx];
        // printf("%u -> %u: %f\n", srcDofIdx, i, src[srcDofIdx]);
    }
}

bool fetchArtiDofAttribute(float *dst, const float *src, const PxU32 numDofs,
                            const PxU32 maxDofs, const PxU32 simMaxDofs,const GpuArticulationDofRecord *dofRecords)
{
    fetchArtiDofAttributeKernel<<<(numDofs + 1023) / 1024, 1024>>>(dst, src, numDofs, maxDofs, simMaxDofs, dofRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

// One kernel for every device-sourced DOF scalar: the record names the source slot and the axis facts,
// while the scale policy differs per attribute (a joint position folds degrees and the body-order sign,
// a torque on the same axis folds the sign alone), so it is an argument rather than a kernel each.
__global__ static void fetchArtiDofAttributeOvStageKernel(float* dst,
                                                          const float* src,
                                                          const PxU32 numOutputs,
                                                          const PxU32 simMaxDofs,
                                                          const DofScalePolicy policy,
                                                          const ArticulationDofOvStageRecord* records)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs)
    {
        const ArticulationDofOvStageRecord r = records[i];
        const PxU32 srcIdx = r.viewArtiIdx * simMaxDofs + r.physxDofIdx;
        dst[i] = dofScaleFor(r, policy) * src[srcIdx];
    }
}

bool fetchArtiDofAttributeOvStage(float* dst, const float* src, const PxU32 numOutputs,
                                  const PxU32 simMaxDofs, const DofScalePolicy policy,
                                  const ArticulationDofOvStageRecord* records, cudaStream_t stream)
{
    if (numOutputs == 0)
        return true;
    fetchArtiDofAttributeOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, numOutputs, simMaxDofs, policy, records);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// Multiply a contiguous float column by a constant on the device: dst[i] = k * src[i]. The ovstage
// device drain uses it to apply the deg->rad scale to an angular-velocity column device-to-device, so
// a column authored in degrees reaches PhysX in engine-native radians with no host round trip.
__global__ static void scaleFloatColumnOvStageKernel(float* dst, const float* src, const PxU32 count, const float k)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count)
        dst[i] = k * src[i];
}

bool scaleFloatColumnOvStage(float* dst, const float* src, const PxU32 count, const float k, cudaStream_t stream)
{
    if (count == 0)
        return true;
    scaleFloatColumnOvStageKernel<<<(count + 1023) / 1024, 1024, 0, stream>>>(dst, src, count, k);
    // Drain the ACTUAL launch stream, not just the default (drainOnLaunchFailure): launchOk waits only on the
    // default stream, so a prior non-sticky error on another stream could falsely fail a valid launch still
    // queued here, whose dst then mutates after the caller saw the failure.
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// Scatter a joint DOF column from change/key order (src[i]) into the session's DOF-order column
// (dst[dstIdx[i]]), folding a wrapped-revolute drive position target into [-360, 360] on the flagged rows so
// the device joint drain matches the host path (InternalJoint::setArticulationDrivePositionTarget). dst is
// pre-zeroed by planJointGroup; the batch covers every single-DOF joint, so every dst row is written here.
__global__ static void scatterDofColumnOvStageKernel(float* dst, const float* src, const PxU32* dstIdx,
                                                     const PxU32 count)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count)
        return;
    // The DOF offset is the low 31 bits; the top bit flags a wrapped-revolute drive target to fold.
    const PxU32 packed = dstIdx[i];
    const bool doWrap = (packed & kScatterDofWrapBit) != 0;
    const PxU32 idx = packed & ~kScatterDofWrapBit;
    float v = src[i];
    if (doWrap && (v >= 360.0f || v <= -360.0f))
        v = fmodf(v, 360.0f);
    dst[idx] = v;
}

bool scatterDofColumnOvStage(float* dst, const float* src, const PxU32* dstIdx, const PxU32 count,
                             cudaStream_t stream)
{
    if (count == 0)
        return true;
    scatterDofColumnOvStageKernel<<<(count + 1023) / 1024, 1024, 0, stream>>>(dst, src, dstIdx, count);
    // Drain the ACTUAL launch stream, not just the default (see scaleFloatColumnOvStage above).
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// Decompose a column-major world transform (double[16], one row per prim) into a position (float[3]) and a
// quaternion (float[4], xyzw). The ovstage device pose drain uses it to convert authored world transforms
// into the pos/orientation the DirectGPU setters take, device-to-device with no host round trip.
//
// The math mirrors the host decomposeMatrix (common/foundation/MatrixTools.cpp): a non-finite translation
// yields the identity pose; otherwise translation is m[12..14] and the rotation is the
// Gram-Schmidt-orthonormalized 3x3 basis (columns X=m[0..2], Y=m[4..6], Z=m[8..10], so positive scale is
// stripped), with an improper (mirrored, det<0) basis flipped proper before the same branch-by-largest-
// diagonal (Shepperd) extraction -- exactly as the host does, which absorbs the reflection into a negative
// scale the pose setter drops. The host's Newton polar refinement runs too; it changes only a degenerate basis,
// since for a TRS transform (orthogonal columns, no shear) Gram-Schmidt already yields the exact rotation.
// `T` is the stored element type (double, or float widened to double on each read, as the host decode does).
template <typename T>
__global__ static void decomposeWorldMatrixOvStageKernel(const T* mat, const PxU32* srcRow, float* pos, float* quat,
                                                         const PxU32 rows)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= rows)
        return;
    // srcRow gathers from a shared (index-mapped) column: output row i reads storage row srcRow[i].
    // Null srcRow is the dense case (storage row i).
    const PxU32 sr = srcRow ? srcRow[i] : i;
    const T* m = mat + static_cast<size_t>(sr) * 16;

    // Non-finite translation yields the identity pose, matching the host decomposeMatrix: a NaN in the
    // authored transform must land as identity rather than reaching the GPU setter as a NaN pose.
    if (!isfinite(m[12]) || !isfinite(m[13]) || !isfinite(m[14]))
    {
        pos[i * 3 + 0] = 0.0f; pos[i * 3 + 1] = 0.0f; pos[i * 3 + 2] = 0.0f;
        quat[i * 4 + 0] = 0.0f; quat[i * 4 + 1] = 0.0f; quat[i * 4 + 2] = 0.0f; quat[i * 4 + 3] = 1.0f;
        return;
    }

    pos[i * 3 + 0] = static_cast<float>(m[12]);
    pos[i * 3 + 1] = static_cast<float>(m[13]);
    pos[i * 3 + 2] = static_cast<float>(m[14]);

    // Basis columns of the column-major matrix.
    double x0 = m[0], x1 = m[1], x2 = m[2];
    double y0 = m[4], y1 = m[5], y2 = m[6];
    double z0 = m[8], z1 = m[9], z2 = m[10];

    // Gram-Schmidt: normalize X; project X out of Y and normalize; project X and Y out of Z and normalize. Each
    // normalize guard matches the host safeNormalize (MatrixTools.cpp): L > 1e-12 AND isfinite(L). The finiteness
    // check matters -- an infinite basis column gives L = inf, which passes L > 1e-12 but makes 1/L = 0 and the
    // scaled column NaN; the host substitutes the fallback world axis instead, so the device must too (AC-20).
    double l = sqrt(x0 * x0 + x1 * x1 + x2 * x2);
    if (l > 1.0e-12 && isfinite(l)) { const double s = 1.0 / l; x0 *= s; x1 *= s; x2 *= s; }
    else { x0 = 1.0; x1 = 0.0; x2 = 0.0; }
    double d = x0 * y0 + x1 * y1 + x2 * y2;
    y0 -= d * x0; y1 -= d * x1; y2 -= d * x2;
    l = sqrt(y0 * y0 + y1 * y1 + y2 * y2);
    if (l > 1.0e-12 && isfinite(l)) { const double s = 1.0 / l; y0 *= s; y1 *= s; y2 *= s; }
    else { y0 = 0.0; y1 = 1.0; y2 = 0.0; }
    const double dx = x0 * z0 + x1 * z1 + x2 * z2;
    const double dy = y0 * z0 + y1 * z1 + y2 * z2;
    z0 -= dx * x0 + dy * y0; z1 -= dx * x1 + dy * y1; z2 -= dx * x2 + dy * y2;
    l = sqrt(z0 * z0 + z1 * z1 + z2 * z2);
    if (l > 1.0e-12 && isfinite(l)) { const double s = 1.0 / l; z0 *= s; z1 *= s; z2 *= s; }
    else { z0 = 0.0; z1 = 0.0; z2 = 1.0; }

    // Newton polar refinement (matching the host decomposeMatrix's Phase 2). From a Gram-Schmidt seed it is a
    // no-op for a well-conditioned basis, but for a DEGENERATE one -- a zero / below-threshold scale axis, where
    // Gram-Schmidt substituted a world axis -- it pulls the basis to the nearest orthogonal matrix, so the device
    // decompose matches the host for those too (the AC-20 host-parity contract). R <- 0.5 * (R + R^-T) per pass;
    // (x*, y*, z*) are columns X, Y, Z. Same 16-iteration cap and 1e-12 convergence tolerance as the host.
    for (int iter = 0; iter < 16; ++iter)
    {
        const double rdet = x0 * (y1 * z2 - y2 * z1) - y0 * (x1 * z2 - x2 * z1) + z0 * (x1 * y2 - x2 * y1);
        if (!isfinite(rdet) || fabs(rdet) < 1.0e-20)
            break;
        const double invD = 1.0 / rdet;
        // Columns of R^-T = cofactor columns of R / det(R).
        const double i00 = (y1 * z2 - y2 * z1) * invD, i01 = (y2 * z0 - y0 * z2) * invD, i02 = (y0 * z1 - y1 * z0) * invD;
        const double i10 = (z1 * x2 - z2 * x1) * invD, i11 = (z2 * x0 - z0 * x2) * invD, i12 = (z0 * x1 - z1 * x0) * invD;
        const double i20 = (x1 * y2 - x2 * y1) * invD, i21 = (x2 * y0 - x0 * y2) * invD, i22 = (x0 * y1 - x1 * y0) * invD;
        const double ax0 = 0.5 * (x0 + i00), ax1 = 0.5 * (x1 + i01), ax2 = 0.5 * (x2 + i02);
        const double ay0 = 0.5 * (y0 + i10), ay1 = 0.5 * (y1 + i11), ay2 = 0.5 * (y2 + i12);
        const double az0 = 0.5 * (z0 + i20), az1 = 0.5 * (z1 + i21), az2 = 0.5 * (z2 + i22);
        double maxDelta = 0.0;
        maxDelta = fmax(maxDelta, fmax(fabs(x0 - ax0), fmax(fabs(x1 - ax1), fabs(x2 - ax2))));
        maxDelta = fmax(maxDelta, fmax(fabs(y0 - ay0), fmax(fabs(y1 - ay1), fabs(y2 - ay2))));
        maxDelta = fmax(maxDelta, fmax(fabs(z0 - az0), fmax(fabs(z1 - az1), fabs(z2 - az2))));
        x0 = ax0; x1 = ax1; x2 = ax2;
        y0 = ay0; y1 = ay1; y2 = ay2;
        z0 = az0; z1 = az1; z2 = az2;
        if (maxDelta < 1.0e-12)
            break;
    }

    // Reflection: an improper (det < 0) basis -- an authored mirror, a negative scale on an odd number of
    // axes -- would make the Shepperd extraction below read out the wrong rotation (a pure mirror would
    // decompose to identity instead of the host's 180deg flip). Negate all three columns to flip it proper,
    // matching the host decomposeMatrix's det<0 branch (which absorbs the reflection into a fully negative
    // scale; the pose setter carries orientation only, so the scale is dropped either way).
    const double det = x0 * (y1 * z2 - y2 * z1) + x1 * (y2 * z0 - y0 * z2) + x2 * (y0 * z1 - y1 * z0);
    if (det < 0.0)
    {
        x0 = -x0; x1 = -x1; x2 = -x2;
        y0 = -y0; y1 = -y1; y2 = -y2;
        z0 = -z0; z1 = -z1; z2 = -z2;
    }

    // Shepperd extraction. n[r][c] = R[c][r]: rows read across the orthonormal basis columns X,Y,Z.
    const double n00 = x0, n01 = y0, n02 = z0;
    const double n10 = x1, n11 = y1, n12 = z1;
    const double n20 = x2, n21 = y2, n22 = z2;
    double qx, qy, qz, qw;
    const double trace = n00 + n11 + n22;
    if (trace > 0.0)
    {
        const double s = sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * s; qx = (n21 - n12) / s; qy = (n02 - n20) / s; qz = (n10 - n01) / s;
    }
    else if (n00 > n11 && n00 > n22)
    {
        const double s = sqrt(1.0 + n00 - n11 - n22) * 2.0;
        qw = (n21 - n12) / s; qx = 0.25 * s; qy = (n01 + n10) / s; qz = (n02 + n20) / s;
    }
    else if (n11 > n22)
    {
        const double s = sqrt(1.0 + n11 - n00 - n22) * 2.0;
        qw = (n02 - n20) / s; qx = (n01 + n10) / s; qy = 0.25 * s; qz = (n12 + n21) / s;
    }
    else
    {
        const double s = sqrt(1.0 + n22 - n00 - n11) * 2.0;
        qw = (n10 - n01) / s; qx = (n02 + n20) / s; qy = (n12 + n21) / s; qz = 0.25 * s;
    }
    const double mag = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (mag > 1.0e-12) { const double s = 1.0 / mag; qx *= s; qy *= s; qz *= s; qw *= s; }
    else { qx = 0.0; qy = 0.0; qz = 0.0; qw = 1.0; }

    quat[i * 4 + 0] = static_cast<float>(qx);
    quat[i * 4 + 1] = static_cast<float>(qy);
    quat[i * 4 + 2] = static_cast<float>(qz);
    quat[i * 4 + 3] = static_cast<float>(qw);
}

template <typename T>
static bool launchDecomposeWorldMatrixOvStage(const T* mat, const PxU32* srcRow, float* pos, float* quat,
                                              const PxU32 rows)
{
    if (rows == 0)
        return true;
    decomposeWorldMatrixOvStageKernel<T><<<(rows + 255) / 256, 256>>>(mat, srcRow, pos, quat, rows);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

bool decomposeWorldMatrixOvStage(const double* mat, const PxU32* srcRow, float* pos, float* quat, const PxU32 rows)
{
    return launchDecomposeWorldMatrixOvStage(mat, srcRow, pos, quat, rows);
}

bool decomposeWorldMatrixOvStage(const float* mat, const PxU32* srcRow, float* pos, float* quat, const PxU32 rows)
{
    return launchDecomposeWorldMatrixOvStage(mat, srcRow, pos, quat, rows);
}

// The tendon counterpart, one kernel for every attribute on both tendon kinds: the source structs are
// plain floats, so an attribute is a (stride, offset, width) triple. comp is 1 for every property
// except the fixed tendon's limit, which is the adjacent (lowLimit, highLimit) pair.
__global__ static void fetchArtiTendonPropertyOvStageKernel(float* dst,
                                                            const float* src,
                                                            const PxU32 numOutputs,
                                                            const PxU32 simMaxTendons,
                                                            const PxU32 structFloats,
                                                            const PxU32 fieldOffset,
                                                            const PxU32 comp,
                                                            const ArticulationTendonOvStageRecord* records)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs)
    {
        const ArticulationTendonOvStageRecord r = records[i];
        const PxU32 base = (r.viewArtiIdx * simMaxTendons + r.tendonIdx) * structFloats + fieldOffset;
        for (PxU32 c = 0; c < comp; ++c)
            dst[i * comp + c] = src[base + c];
    }
}

bool fetchArtiTendonPropertyOvStage(float* dst, const float* src, const PxU32 numOutputs,
                                    const PxU32 simMaxTendons, const PxU32 structFloats,
                                    const PxU32 fieldOffset, const PxU32 comp,
                                    const ArticulationTendonOvStageRecord* records, cudaStream_t stream)
{
    if (numOutputs == 0)
        return true;
    fetchArtiTendonPropertyOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, numOutputs, simMaxTendons, structFloats, fieldOffset, comp, records);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// Where one record's state sits in the fetched source: a rigid dynamic's slot in the rigid-dynamic block, or a link's
// in the [articulation x simMaxLinks] link block. A whole-view fetch holds every body at its view slot. A packed fetch
// (ADR-0055) holds only the read's rows: a rigid dynamic at its output slot `i` (`packedRd`), and a link's
// articulation at `artiSlot[i]`. False when the body has no GPU slot (a disabled rigid dynamic), which the callers
// write as a zeroed row, or when its slot lies past the `numFetched` rigid dynamics or articulations fetched: a graph
// replayed after a body was enabled reads a live index against the count fetched at its capture.
__device__ __forceinline__ static bool rbSourceSlot(const GpuRigidBodyRecord& r,
                                                    const PxU32 i,
                                                    const bool packedRd,
                                                    const PxU32* artiSlot,
                                                    const PxU32 simMaxLinks,
                                                    const PxU32 numFetched,
                                                    bool& isLink,
                                                    PxU32& slot)
{
    if (r.tensorRdIdx != 0xffffffff)
    {
        isLink = false;
        slot = packedRd ? i : r.tensorRdIdx;
        return slot < numFetched;
    }
    // Both fields must be valid: a disabled rigid dynamic has every index at the sentinel, and the
    // compound articulation * simMaxLinks + linkIdx would otherwise wrap to a live row.
    if (r.tensorArtiIdx != 0xffffffff && r.linkIdx != 0xffffffff)
    {
        isLink = true;
        const PxU32 articulation = artiSlot ? artiSlot[i] : r.tensorArtiIdx;
        slot = articulation * simMaxLinks + r.linkIdx;
        return articulation < numFetched;
    }
    return false;
}

__global__ static void fetchRbPoseColumnOvStageKernel(float* dst,
                                                      const PxTransform* actorData,
                                                      const PxTransform* linkTransforms,
                                                      const GpuRigidBodyRecord* records,
                                                      const PxU32* outRecordIdx,
                                                      const PxU32 numOutputs,
                                                      const PxU32 simMaxLinks,
                                                      const bool wantOrientation,
                                                      const bool packedRd,
                                                      const PxU32* artiSlot,
                                                      const PxU32 numFetched)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        const GpuRigidBodyRecord& r = records[recIdx];

        bool isLink = false;
        PxU32 slot = 0;
        const bool resolved = rbSourceSlot(r, i, packedRd, artiSlot, simMaxLinks, numFetched, isLink, slot);
        PxTransform pose(PxIdentity);
        if (resolved)
            pose = isLink ? linkTransforms[slot] : actorData[slot];

        if (wantOrientation)
        {
            pose.q = r.orientationFixup * pose.q;
            float* o = dst + static_cast<size_t>(i) * 4;
            o[0] = resolved ? pose.q.x : 0.0f;
            o[1] = resolved ? pose.q.y : 0.0f;
            o[2] = resolved ? pose.q.z : 0.0f;
            o[3] = resolved ? pose.q.w : 1.0f; // identity for an unresolved row
        }
        else
        {
            // Subspace origin is applied here, so the destination column is already stage-local.
            float* o = dst + static_cast<size_t>(i) * 3;
            o[0] = resolved ? pose.p.x - r.origin.x : 0.0f;
            o[1] = resolved ? pose.p.y - r.origin.y : 0.0f;
            o[2] = resolved ? pose.p.z - r.origin.z : 0.0f;
        }
    }
}

bool fetchRbPoseColumnOvStage(float* dst,
                              const PxTransform* actorData,
                              const PxTransform* linkTransforms,
                              const GpuRigidBodyRecord* records,
                              const PxU32* outRecordIdx,
                              const PxU32 numOutputs,
                              const PxU32 simMaxLinks,
                              const bool wantOrientation,
                              cudaStream_t stream,
                              const bool packedRd,
                              const PxU32* artiSlot,
                              const PxU32 numFetched)
{
    if (numOutputs == 0)
        return true;
    fetchRbPoseColumnOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        dst, actorData, linkTransforms, records, outRecordIdx, numOutputs, simMaxLinks, wantOrientation, packedRd,
        artiSlot, numFetched);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void fetchRbVelocityColumnOvStageKernel(float* dst,
                                                          const PxVec3* rdData,
                                                          const PxVec3* linkData,
                                                          const GpuRigidBodyRecord* records,
                                                          const PxU32* outRecordIdx,
                                                          const PxU32 numOutputs,
                                                          const PxU32 simMaxLinks,
                                                          const bool packedRd,
                                                          const PxU32* artiSlot,
                                                          const PxU32 numFetched)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        const GpuRigidBodyRecord& r = records[recIdx];

        bool isLink = false;
        PxU32 slot = 0;
        PxVec3 v(0.0f);
        if (rbSourceSlot(r, i, packedRd, artiSlot, simMaxLinks, numFetched, isLink, slot))
            v = isLink ? linkData[slot] : rdData[slot];

        float* o = dst + static_cast<size_t>(i) * 3;
        o[0] = v.x;
        o[1] = v.y;
        o[2] = v.z;
    }
}

bool fetchRbVelocityColumnOvStage(float* dst,
                                  const PxVec3* rdData,
                                  const PxVec3* linkData,
                                  const GpuRigidBodyRecord* records,
                                  const PxU32* outRecordIdx,
                                  const PxU32 numOutputs,
                                  const PxU32 simMaxLinks,
                                  cudaStream_t stream,
                                  const bool packedRd,
                                  const PxU32* artiSlot,
                                  const PxU32 numFetched)
{
    if (numOutputs == 0)
        return true;
    fetchRbVelocityColumnOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        dst, rdData, linkData, records, outRecordIdx, numOutputs, simMaxLinks, packedRd, artiSlot, numFetched);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void fetchRbWorldScaleColumnOvStageKernel(float* dst,
                                                            const PxVec3* scales,
                                                            const PxU32* outRecordIdx,
                                                            const PxU32 numOutputs)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs)
    {
        const PxVec3 s = scales[outRecordIdx ? outRecordIdx[i] : i];
        float* o = dst + static_cast<size_t>(i) * 3;
        o[0] = s.x;
        o[1] = s.y;
        o[2] = s.z;
    }
}

bool fetchRbWorldScaleColumnOvStage(float* dst,
                                    const PxVec3* scales,
                                    const PxU32* outRecordIdx,
                                    const PxU32 numOutputs,
                                    cudaStream_t stream)
{
    if (numOutputs == 0)
        return true;
    fetchRbWorldScaleColumnOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(dst, scales, outRecordIdx,
                                                                                         numOutputs);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// ovstage point-instancer columns. The destination is SCATTERED: one group per instancer holding its
// full instance array placed by index, so a thread writes at offsets[instancer] + slot * comp, leaving
// holes where an instance has no live body. Positions and orientations are instancer-LOCAL and get
// reframed out of world space; velocities are world-frame, a plain gather. Every instanced body is a
// PxRigidDynamic, never an articulation link, so there is no link source to resolve here.

__global__ static void submitInstancerPoseColumnOvStageKernel(PxTransform* packedPose,
                                                              const float* src,
                                                              const PxTransform* actorData,
                                                              const GpuRigidBodyRecord* rbRecords,
                                                              const GpuPointInstancerRecord* records,
                                                              const InstancerAffine* instancerInverses,
                                                              const InstancerAffine* instancerForwards,
                                                              const PxU32 numOutputs,
                                                              const PxU32 instancerIdx,
                                                              const bool wantOrientation,
                                                              const uint8_t* mask,
                                                              const PxTransform* currentByOutput)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numOutputs)
        return;

    // `records` already points at this instancer's range, so thread i IS output slot i and there is
    // no filter to run. Holes never appear here -- an instance with no live body was dropped when
    // the records were built.
    const GpuPointInstancerRecord& r = records[i];
    const GpuRigidBodyRecord& rb = rbRecords[r.rbRow];
    if (rb.tensorRdIdx == 0xffffffff)
        return; // disabled rigid dynamic: no DirectGPU row, so nothing to write to

    // Read-modify-write: recover the half this column does not carry from the body's CURRENT pose.
    // A session carries one attribute but a pose needs both halves -- the same reason the standalone
    // pose path pre-reads, reached again here. A session commit pre-reads exactly its own bodies, by
    // output slot; the drain reads the scene-wide buffer.
    const PxTransform pose = currentByOutput ? currentByOutput[i] : actorData[rb.tensorRdIdx];
    if (!isRowCommitted(mask, r.slot))
    {
        packedPose[i] = pose; // an unselected instance is written its current pose
        return;
    }
    const double worldQuat[4] = { double(pose.q.x), double(pose.q.y), double(pose.q.z), double(pose.q.w) };
    const double worldPos[3] = { double(pose.p.x), double(pose.p.y), double(pose.p.z) };
    double localPos[3];
    double localQuat[4];
    instancerReframe(r.protoInverse, instancerInverses[instancerIdx], worldQuat, worldPos, localPos, localQuat);

    // The caller's half, placed by INSTANCE INDEX: the column carries the instancer's full array,
    // holes included, which is how the read publishes it -- so it is r.slot, not i.
    if (wantOrientation)
    {
        const float* o = src + static_cast<size_t>(r.slot) * 4;
        localQuat[0] = double(o[0]);
        localQuat[1] = double(o[1]);
        localQuat[2] = double(o[2]);
        localQuat[3] = double(o[3]);
    }
    else
    {
        const float* o = src + static_cast<size_t>(r.slot) * 3;
        localPos[0] = double(o[0]);
        localPos[1] = double(o[1]);
        localPos[2] = double(o[2]);
    }

    double outPos[3];
    double outQuat[4];
    instancerUnreframe(r.proto, instancerForwards[instancerIdx], localQuat, localPos, outPos, outQuat);
    packedPose[i] = PxTransform(PxVec3(float(outPos[0]), float(outPos[1]), float(outPos[2])),
                                PxQuat(float(outQuat[0]), float(outQuat[1]), float(outQuat[2]),
                                       float(outQuat[3])));
}

bool submitInstancerPoseColumnOvStage(PxTransform* packedPose, const float* src, const PxTransform* actorData,
                                      const GpuRigidBodyRecord* rbRecords,
                                      const GpuPointInstancerRecord* records,
                                      const InstancerAffine* instancerInverses,
                                      const InstancerAffine* instancerForwards, const PxU32 numOutputs,
                                      const PxU32 instancerIdx, const bool wantOrientation, cudaStream_t stream,
                                      const uint8_t* mask, const PxTransform* currentByOutput)
{
    if (numOutputs == 0)
        return true;
    // 256, not 1024: like the read reframe this keeps several InstancerAffine (12 doubles each)
    // live, and a 1024-thread block caps a thread at 64 registers, past which the launch fails
    // outright with "too many resources requested". See fetchInstancerPoseColumnOvStage; check
    // `nvcc -Xptxas -v` before changing.
    submitInstancerPoseColumnOvStageKernel<<<(numOutputs + 255) / 256, 256, 0, stream>>>(
        packedPose, src, actorData, rbRecords, records, instancerInverses, instancerForwards, numOutputs,
        instancerIdx, wantOrientation, mask, currentByOutput);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitInstancerVelocityColumnOvStageKernel(PxVec3* packedVel,
                                                                  const float* src,
                                                                  const GpuPointInstancerRecord* records,
                                                                  const PxU32 numOutputs,
                                                                  const uint8_t* mask)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    // An unselected instance keeps the current velocity a masked commit pre-read into the block.
    if (i >= numOutputs || !isRowCommitted(mask, records[i].slot))
        return;
    // No reframe: world space in both directions.
    const float* o = src + static_cast<size_t>(records[i].slot) * 3;
    packedVel[i] = PxVec3(o[0], o[1], o[2]);
}

bool submitInstancerVelocityColumnOvStage(PxVec3* packedVel, const float* src,
                                          const GpuPointInstancerRecord* records, const PxU32 numOutputs,
                                          cudaStream_t stream, const uint8_t* mask)
{
    if (numOutputs == 0)
        return true;
    // 256 to match the pose kernel above; a plain gather like this one would run happily at 1024.
    submitInstancerVelocityColumnOvStageKernel<<<(numOutputs + 255) / 256, 256, 0, stream>>>(
        packedVel, src, records, numOutputs, mask);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void fetchInstancerPoseColumnOvStageKernel(float* dst,
                                                             const PxTransform* actorData,
                                                             const GpuRigidBodyRecord* rbRecords,
                                                             const GpuPointInstancerRecord* records,
                                                             const InstancerAffine* instancerInverses,
                                                             const PxU32* offsets,
                                                             const PxU32 numInstances,
                                                             const bool wantOrientation)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numInstances)
        return;

    const GpuPointInstancerRecord& r = records[i];
    const GpuRigidBodyRecord& rb = rbRecords[r.rbRow];
    // A disabled rigid dynamic has no DirectGPU row; leaving the slot zero-filled matches the host path.
    if (rb.tensorRdIdx == 0xffffffff)
        return;

    const PxTransform pose = actorData[rb.tensorRdIdx];
    const double worldQuat[4] = { double(pose.q.x), double(pose.q.y), double(pose.q.z), double(pose.q.w) };
    const double worldPos[3] = { double(pose.p.x), double(pose.p.y), double(pose.p.z) };

    // Compose once, extract only what this column publishes. The polar rotation costs a double sqrt
    // and divides, which dominate this kernel wherever fp64 throughput is a fraction of fp32 -- and
    // the position column never uses the result, so computing it unconditionally would pay that on
    // every instance for nothing.
    InstancerAffine local;
    instancerComposeLocal(r.protoInverse, instancerInverses[r.instancerIdx], worldQuat, worldPos, local);

    if (wantOrientation)
    {
        double localQuat[4];
        instancerPolarRotation(local, localQuat);
        float* o = dst + offsets[r.instancerIdx] + size_t(r.slot) * 4;
        o[0] = float(localQuat[0]);
        o[1] = float(localQuat[1]);
        o[2] = float(localQuat[2]);
        o[3] = float(localQuat[3]);
    }
    else
    {
        float* o = dst + offsets[r.instancerIdx] + size_t(r.slot) * 3;
        o[0] = float(local.column3.x);
        o[1] = float(local.column3.y);
        o[2] = float(local.column3.z);
    }
}

bool fetchInstancerPoseColumnOvStage(float* dst,
                                     const PxTransform* actorData,
                                     const GpuRigidBodyRecord* rbRecords,
                                     const GpuPointInstancerRecord* records,
                                     const InstancerAffine* instancerInverses,
                                     const PxU32* offsets,
                                     const PxU32 numInstances,
                                     const bool wantOrientation, cudaStream_t stream)
{
    if (numInstances == 0)
        return true;
    // 256, not the 1024 used elsewhere in this file: the reframe keeps several InstancerAffine (12
    // doubles each) live at once, and a 1024-thread block caps a thread at 64 registers, past which the
    // launch fails outright with "too many resources requested". Check `nvcc -Xptxas -v` before changing.
    fetchInstancerPoseColumnOvStageKernel<<<(numInstances + 255) / 256, 256, 0, stream>>>(
        dst, actorData, rbRecords, records, instancerInverses, offsets, numInstances, wantOrientation);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void fetchInstancerVelocityColumnOvStageKernel(float* dst,
                                                                 const PxVec3* rdData,
                                                                 const GpuRigidBodyRecord* rbRecords,
                                                                 const GpuPointInstancerRecord* records,
                                                                 const PxU32* offsets,
                                                                 const PxU32 numInstances)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numInstances)
        return;

    const GpuPointInstancerRecord& r = records[i];
    const GpuRigidBodyRecord& rb = rbRecords[r.rbRow];
    if (rb.tensorRdIdx == 0xffffffff)
        return;

    const PxVec3 v = rdData[rb.tensorRdIdx];
    float* o = dst + offsets[r.instancerIdx] + size_t(r.slot) * 3;
    o[0] = v.x;
    o[1] = v.y;
    o[2] = v.z;
}

bool fetchInstancerVelocityColumnOvStage(float* dst,
                                         const PxVec3* rdData,
                                         const GpuRigidBodyRecord* rbRecords,
                                         const GpuPointInstancerRecord* records,
                                         const PxU32* offsets,
                                         const PxU32 numInstances, cudaStream_t stream)
{
    if (numInstances == 0)
        return true;
    // 256 to match the pose kernel above; a plain gather like this one would run happily at 1024.
    fetchInstancerVelocityColumnOvStageKernel<<<(numInstances + 255) / 256, 256, 0, stream>>>(
        dst, rdData, rbRecords, records, offsets, numInstances);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// One thread per (set, point) of a RECTANGULAR numSets x maxPointsPerSet grid, so a thread whose
// point index is past its own set's length exits on the bounds test below. What that costs, and why
// it is the right shape anyway, is argued where the extent is computed, in
// fetchPointSetColumnOvStage.
//
// It costs no idle MEMORY, which is the part that would matter: the destination is compact. No write
// address derives from `maxPointsPerSet` -- `dstOffsetFloats` carries the layout, whatever the view
// chose. The extent itself is bounded by the view at construction.
//
// Templated on the source element rather than a runtime stride: PxVec4 keeps its aligned 16-byte load,
// and PxVec3 (the skinned-vertex buffer) gets its own instantiation instead of a per-thread branch.
template <typename TSrc>
__global__ static void fetchPointSetColumnOvStageKernel(float* dst,
                                                        const GpuPointSetReadRecord* records,
                                                        const PointSetTransform* transforms,
                                                        const PxU32 numSets,
                                                        const PxU32 maxPointsPerSet,
                                                        const bool reframe)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numSets * maxPointsPerSet)
        return;

    const PxU32 set = i / maxPointsPerSet;
    const PxU32 point = i % maxPointsPerSet;

    const GpuPointSetReadRecord& r = records[set];
    if (point >= r.numPoints || !r.src)
        return;

    const TSrc s = static_cast<const TSrc*>(r.src)[point];
    float* o = dst + r.dstOffsetFloats + size_t(point) * 3;

    if (reframe)
    {
        // On a PxVec4 source .w is inverse mass, not a coordinate, and dropping it here is the compaction
        // to packed xyz. A PxVec3 source has no fourth lane and passes through whole.
        const float p[3] = { s.x, s.y, s.z };
        pointSetTransformPoint(transforms[set], p, o);
    }
    else
    {
        o[0] = s.x;
        o[1] = s.y;
        o[2] = s.z;
    }
}

bool fetchPointSetColumnOvStage(float* dst,
                                const GpuPointSetReadRecord* records,
                                const PointSetTransform* transforms,
                                const PxU32 numSets,
                                const PxU32 maxPointsPerSet,
                                const bool reframe,
                                const PointSetSourceLayout layout,
                                cudaStream_t stream)
{
    if (numSets == 0 || maxPointsPerSet == 0)
        return true;

    // A RECTANGULAR extent, not one thread per point of a ragged set. The idle threads on the short
    // sets are the price of not building and uploading a per-set prefix sum every read -- host work
    // proportional to the set count, which is what this path exists to avoid. Do not "fix" it on
    // inspection: on a ragged scene holding the point total fixed, threads that do nothing but fail
    // the bounds test measure inside the uniform-scene spread.
    const PxU32 threads = numSets * maxPointsPerSet;
    // 1 + (threads - 1) / 256, not (threads + 255) / 256: the latter wraps for threads > 2^32 - 256
    // and produces a ZERO-block grid, which launches cleanly, writes nothing, and publishes the
    // destination's prior contents as a successful read. The caller refuses such extents, but the
    // arithmetic must not depend on that to be safe. threads is non-zero -- the early-out above
    // returns unless some set has points.
    const PxU32 blocks = 1u + (threads - 1u) / 256u;
    if (layout == PointSetSourceLayout::eVec3)
        fetchPointSetColumnOvStageKernel<PxVec3><<<blocks, 256, 0, stream>>>(
            dst, records, transforms, numSets, maxPointsPerSet, reframe);
    else
        fetchPointSetColumnOvStageKernel<PxVec4><<<blocks, 256, 0, stream>>>(
            dst, records, transforms, numSets, maxPointsPerSet, reframe);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// The inverse of fetchArtiDofAttributeOvStageKernel: it multiplies by the inverse scale -- see CudaKernels.h.
__global__ static void submitArtiDofAttributeOvStageKernel(float* dofScalars,
                                                           const float* src,
                                                           const PxU32 numOutputs,
                                                           const PxU32 simMaxDofs,
                                                           const ArticulationDofOvStageRecord* records,
                                                           const DofScalePolicy policy,
                                                           const uint8_t* mask)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    // One mask entry per output: a joint group has one row per DOF (ADR-0059).
    if (i < numOutputs && isRowCommitted(mask, i))
    {
        const ArticulationDofOvStageRecord r = records[i];
        const PxU32 dstIdx = r.viewArtiIdx * simMaxDofs + r.physxDofIdx;
        // MULTIPLY by invAngScale, not divide by angScale. The record carries both because the float
        // reciprocal of the rad->deg constant is not the float deg->rad one, so a value the read
        // published as `authored * angScale` returns to `authored` only when the write multiplies by
        // the inverse constant the parse library actually used. Dividing here was the old shape, and
        // it was exact only because the folded scale was a single multiply to undo.
        //
        // The sign is +-1, so folding it back is exact either way. Which folds apply is the
        // ATTRIBUTE's business, not this kernel's: position, velocity and the two drive targets are
        // eAngularSigned, but an actuation force is eSigned -- a torque is not a rad->deg quantity,
        // and folding one into it would scale every joint effort by 57.3.
        dofScalars[dstIdx] = src[i] * dofInverseScaleFor(r, policy);
    }
}

bool submitArtiDofAttributeOvStage(float* dofScalars,
                                   const float* src,
                                   const PxU32 numOutputs,
                                   const PxU32 simMaxDofs,
                                   const ArticulationDofOvStageRecord* records,
                                   const DofScalePolicy policy,
                                   cudaStream_t stream,
                                   const uint8_t* mask)
{
    if (numOutputs == 0)
        return true;
    submitArtiDofAttributeOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        dofScalars, src, numOutputs, simMaxDofs, records, policy, mask);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitPointSetColumnOvStageKernel(PxVec4* dst,
                                                        const float* src,
                                                        const PointSetTransform xf,
                                                        const PxU32 numPoints,
                                                        const bool reframe)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numPoints)
        return;

    const float p[3] = { src[i * 3 + 0], src[i * 3 + 1], src[i * 3 + 2] };
    float o[3];
    if (reframe)
    {
        pointSetTransformPoint(xf, p, o);
    }
    else
    {
        o[0] = p[0];
        o[1] = p[1];
        o[2] = p[2];
    }

    // Read-modify-write of the FOURTH lane alone. See the header: on a position buffer it is inverse
    // mass, and the caller's column has no fourth component to supply it from.
    PxVec4 v = dst[i];
    v.x = o[0];
    v.y = o[1];
    v.z = o[2];
    dst[i] = v;
}

bool submitPointSetColumnOvStage(PxVec4* const dst,
                                 const float* const src,
                                 const PointSetTransform& localToWorld,
                                 const PxU32 numPoints,
                                 const bool reframe)
{
    if (numPoints == 0)
        return true;
    submitPointSetColumnOvStageKernel<<<(numPoints + 255) / 256, 256>>>(dst, src, localToWorld, numPoints,
                                                                       reframe);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

// A volume's kinematic-target buffer as its setup leaves it: every node free (w = 1) at its current position, so the
// buffer changes nothing until a commit pins a node.
__global__ static void initKinematicTargetBufferOvStageKernel(PxVec4* dst,
                                                              const PxVec4* positions,
                                                              const PxU32 numNodes)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numNodes)
        dst[i] = PxVec4(positions[i].x, positions[i].y, positions[i].z, 1.0f);
}

bool initKinematicTargetBufferOvStage(PxVec4* const dst, const PxVec4* const positions, const PxU32 numNodes)
{
    if (numNodes == 0)
        return true;
    initKinematicTargetBufferOvStageKernel<<<(numNodes + 255) / 256, 256>>>(dst, positions, numNodes);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

// The kinematic-target commit: every body of the group in one launch, a row of blocks per body. A node pinned
// (w == 0) that the write marks free keeps its target and is counted, since PhysX never frees a pinned node.
__global__ static void submitKinematicTargetsOvStageKernel(const KinematicTargetBodyRecord* bodies,
                                                           const PxU32 numBodies,
                                                           const float* column,
                                                           const uint8_t* mask,
                                                           unsigned long long* keptPinned)
{
    for (PxU32 b = blockIdx.y; b < numBodies; b += gridDim.y)
    {
        const KinematicTargetBodyRecord& body = bodies[b];
        if (!body.dst || !isRowCommitted(mask, b))
            continue;
        for (PxU32 n = blockIdx.x * blockDim.x + threadIdx.x; n < body.nodeCount; n += gridDim.x * blockDim.x)
        {
            const float* const src = column + body.floatOffset + size_t(n) * 4;
            const float w = src[3];
            if (w != 0.0f && body.dst[n].w == 0.0f)
            {
                atomicAdd(keptPinned, 1ull);
                continue;
            }
            float o[3];
            pointSetTransformPoint(body.columnToBuffer, src, o);
            body.dst[n] = PxVec4(o[0], o[1], o[2], w);
        }
    }
}

bool submitKinematicTargetsOvStage(const KinematicTargetBodyRecord* const bodies,
                                   const PxU32 numBodies,
                                   const PxU32 maxNodes,
                                   const float* const column,
                                   const uint8_t* const mask,
                                   unsigned long long* const keptPinned,
                                   cudaStream_t stream)
{
    if (numBodies == 0 || maxNodes == 0)
        return true;
    const dim3 grid(PxMin((maxNodes + 255u) / 256u, 1024u), PxMin(numBodies, 65535u));
    submitKinematicTargetsOvStageKernel<<<grid, 256, 0, stream>>>(bodies, numBodies, column, mask, keptPinned);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// Kinematic-target gather: read one body's runtime-owned PxVec4 buffer into a four-float-per-node column
// (xyz + w flag). The inverse of the scatter above: xyz is carried world->sim-mesh by `worldToLocal`
// exactly as the point-set gather reframes points, and the w flag passes through unchanged.
__global__ static void fetchKinematicTargetColumnOvStageKernel(float* dst,
                                                               const PxVec4* src,
                                                               const PointSetTransform xf,
                                                               const PxU32 numNodes,
                                                               const bool reframe)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numNodes)
        return;

    const PxVec4 v = src[i];
    const float p[3] = { v.x, v.y, v.z };
    float o[3];
    if (reframe)
    {
        pointSetTransformPoint(xf, p, o);
    }
    else
    {
        o[0] = p[0];
        o[1] = p[1];
        o[2] = p[2];
    }
    dst[i * 4 + 0] = o[0];
    dst[i * 4 + 1] = o[1];
    dst[i * 4 + 2] = o[2];
    dst[i * 4 + 3] = v.w;
}

bool fetchKinematicTargetColumnOvStage(float* const dst,
                                       const PxVec4* const src,
                                       const PointSetTransform& worldToLocal,
                                       const PxU32 numNodes,
                                       const bool reframe, cudaStream_t stream)
{
    if (numNodes == 0)
        return true;
    fetchKinematicTargetColumnOvStageKernel<<<(numNodes + 255) / 256, 256, 0, stream>>>(
        dst, src, worldToLocal, numNodes, reframe);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// The packed write pair. See CudaKernels.h for why this replaces flags-and-compaction: that path
// derives its element count on the host and therefore cannot be non-blocking, which the read's
// contract requires.
__global__ static void submitRbPackedIndicesOvStageKernel(PxRigidDynamicGPUIndex* dstIdx,
                                                         const PxRigidDynamicGPUIndex* rdGpuIndices,
                                                         const GpuRigidBodyRecord* records,
                                                         const PxU32* outRecordIdx,
                                                         const PxU32 numOutputs)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        const PxU32 rdIdx = records[recIdx].tensorRdIdx;
        // The caller guarantees no sentinel here (see the header). Clamped rather than branched so a
        // violation writes a duplicate of slot 0's body instead of dereferencing out of bounds --
        // wrong, but bounded, and the host check is what actually prevents it.
        dstIdx[i] = rdGpuIndices[rdIdx != 0xffffffff ? rdIdx : 0u];
    }
}

bool submitRbPackedIndicesOvStage(PxRigidDynamicGPUIndex* dstIdx,
                                 const PxRigidDynamicGPUIndex* rdGpuIndices,
                                 const GpuRigidBodyRecord* records,
                                 const PxU32* outRecordIdx,
                                 const PxU32 numOutputs,
                                 cudaStream_t stream)
{
    if (numOutputs == 0)
        return true;
    submitRbPackedIndicesOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        dstIdx, rdGpuIndices, records, outRecordIdx, numOutputs);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitRbPackedPoseOvStageKernel(PxTransform* packedPose,
                                                        const float* src,
                                                        const GpuRigidBodyRecord* records,
                                                        const PxU32* outRecordIdx,
                                                        const PxU32 numOutputs,
                                                        const bool wantOrientation,
                                                        const uint8_t* mask)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    // An unselected row keeps the current pose the caller pre-read into the block.
    if (i < numOutputs && isRowCommitted(mask, i))
    {
        if (wantOrientation)
        {
            const float* o = src + static_cast<size_t>(i) * 4;
            packedPose[i].q = PxQuat(o[0], o[1], o[2], o[3]);
        }
        else
        {
            // Stage-local in, world out: the read subtracts this origin, so the round trip is exact.
            const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
            const PxVec3& origin = records[recIdx].origin;
            const float* o = src + static_cast<size_t>(i) * 3;
            packedPose[i].p = PxVec3(o[0] + origin.x, o[1] + origin.y, o[2] + origin.z);
        }
    }
}

bool submitRbPackedPoseOvStage(PxTransform* packedPose,
                                const float* src,
                                const GpuRigidBodyRecord* records,
                                const PxU32* outRecordIdx,
                                const PxU32 numOutputs,
                                const bool wantOrientation,
                                cudaStream_t stream,
                                const uint8_t* mask)
{
    if (numOutputs == 0)
        return true;
    submitRbPackedPoseOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        packedPose, src, records, outRecordIdx, numOutputs, wantOrientation, mask);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// Overlay a selected row's vec3 onto `dst`; an unselected row keeps `dst` or, for a quantity PhysX cannot
// read back (a force or a torque), becomes zero.
__global__ static void overlayVec3ColumnOvStageKernel(PxVec3* dst,
                                                      const float* src,
                                                      const uint8_t* mask,
                                                      const PxU32 numOutputs,
                                                      const bool zeroUnselected)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numOutputs)
        return;
    if (isRowCommitted(mask, i))
    {
        const float* o = src + static_cast<size_t>(i) * 3;
        dst[i] = PxVec3(o[0], o[1], o[2]);
    }
    else if (zeroUnselected)
    {
        dst[i] = PxVec3(0.0f);
    }
}

bool overlayVec3ColumnOvStage(PxVec3* dst,
                              const float* src,
                              const uint8_t* mask,
                              const PxU32 numOutputs,
                              const bool zeroUnselected,
                              cudaStream_t stream)
{
    if (numOutputs == 0)
        return true;
    overlayVec3ColumnOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(dst, src, mask, numOutputs,
                                                                                   zeroUnselected);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitLinkWrenchOvStageKernel(PxVec3* linkForces,
                                                     PxVec3* linkTorques,
                                                     const float* src,
                                                     const PxTransform* linkTransforms,
                                                     const PxVec3* comsByRecord,
                                                     const GpuRigidBodyRecord* records,
                                                     const PxU32* rows,
                                                     const PxU32* rowCompactArti,
                                                     const PxU32 numOutputs,
                                                     const PxU32 simMaxLinks,
                                                     const PxU32 comps,
                                                     const uint8_t* mask)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    // An unselected link keeps the zero its cleared block already holds.
    if (i < numOutputs && isRowCommitted(mask, i))
    {
        const PxU32 rec = rows[i];
        const GpuRigidBodyRecord& r = records[rec];
        // The link pose INPUT is always read from the whole-view transform buffer (filled in tensorArtiIdx
        // order). The force/torque OUTPUT lands in a COMPACT block keyed by the written articulation's slot
        // when the apply is sparse; on the write-ALL fast path (rowCompactArti == nullptr) every articulation
        // is applied, so the output uses the same whole-view slot as the input.
        const PxU32 inSlot = r.tensorArtiIdx * simMaxLinks + r.linkIdx;
        const PxU32 outSlot = rowCompactArti ? (rowCompactArti[i] * simMaxLinks + r.linkIdx) : inSlot;

        const float* w = src + static_cast<size_t>(i) * comps;
        const PxVec3 force(w[0], w[1], w[2]);
        linkForces[outSlot] = force;
        if (comps == 9)
        {
            const PxVec3 torque(w[3], w[4], w[5]);
            const PxVec3 point(w[6], w[7], w[8]);
            const PxTransform& pose = linkTransforms[inSlot];
            const PxVec3 comWorld = pose.transform(comsByRecord[rec]);
            linkTorques[outSlot] = torque + (point - comWorld).cross(force);
        }
    }
}

bool submitLinkWrenchOvStage(PxVec3* linkForces, PxVec3* linkTorques, const float* src,
                              const PxTransform* linkTransforms, const PxVec3* comsByRecord,
                              const GpuRigidBodyRecord* records, const PxU32* rows, const PxU32* rowCompactArti,
                              const PxU32 numOutputs, const PxU32 simMaxLinks, const PxU32 comps,
                              cudaStream_t stream, const uint8_t* mask)
{
    if (numOutputs == 0)
        return true;
    submitLinkWrenchOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        linkForces, linkTorques, src, linkTransforms, comsByRecord, records, rows, rowCompactArti, numOutputs,
        simMaxLinks, comps, mask);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitRbWrenchOvStageKernel(PxVec3* outForces,
                                                   PxVec3* outTorques,
                                                   const float* src,
                                                   const PxTransform* poses,
                                                   const PxVec3* comsByRecord,
                                                   const PxU32* rows,
                                                   const PxU32 numOutputs,
                                                   const uint8_t* mask)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs && !isRowCommitted(mask, i))
    {
        // PhysX cannot read an applied wrench back, so an unselected row applies none.
        outForces[i] = PxVec3(0.0f);
        outTorques[i] = PxVec3(0.0f);
    }
    else if (i < numOutputs)
    {
        const float* w = src + static_cast<size_t>(i) * 9;
        const PxVec3 force(w[0], w[1], w[2]);
        const PxVec3 torque(w[3], w[4], w[5]);
        const PxVec3 point(w[6], w[7], w[8]);

        // The application point is WORLD space, matching every other position this API accepts. The
        // binding's equivalent takes an isGlobal flag; the ovstage surface does not, because a
        // second convention for "where" would have to be discoverable and there is nowhere to say it.
        const PxTransform& pose = poses[i];
        const PxVec3 comWorld = pose.transform(comsByRecord[rows[i]]);
        outForces[i] = force;
        outTorques[i] = torque + (point - comWorld).cross(force);
    }
}

bool submitRbWrenchOvStage(PxVec3* outForces, PxVec3* outTorques, const float* src, const PxTransform* poses,
                            const PxVec3* comsByRecord, const PxU32* rows, const PxU32 numOutputs,
                            cudaStream_t stream, const uint8_t* mask)
{
    if (numOutputs == 0)
        return true;
    submitRbWrenchOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        outForces, outTorques, src, poses, comsByRecord, rows, numOutputs, mask);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitArtiRootPoseOvStageKernel(PxTransform* rootBlock,
                                                        const float* src,
                                                        const PxU32* rows,
                                                        const GpuArticulationRootRecord* rootRecords,
                                                        const PxU32 numOutputs,
                                                        const bool wantOrientation,
                                                        const uint8_t* mask)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    // An unselected articulation keeps the current root state pre-read into the block.
    if (i < numOutputs && isRowCommitted(mask, i))
    {
        const PxU32 row = rows[i];
        if (wantOrientation)
        {
            const float* o = src + static_cast<size_t>(i) * 4;
            rootBlock[row].q = PxQuat(o[0], o[1], o[2], o[3]);
        }
        else
        {
            // Stage-local in, world out: the read subtracts this origin, so the round trip is exact.
            const PxVec3& origin = rootRecords[row].origin;
            const float* o = src + static_cast<size_t>(i) * 3;
            rootBlock[row].p = PxVec3(o[0] + origin.x, o[1] + origin.y, o[2] + origin.z);
        }
    }
}

bool submitArtiRootPoseOvStage(PxTransform* rootBlock,
                                const float* src,
                                const PxU32* rows,
                                const GpuArticulationRootRecord* rootRecords,
                                const PxU32 numOutputs,
                                const bool wantOrientation,
                                cudaStream_t stream,
                                const uint8_t* mask)
{
    if (numOutputs == 0)
        return true;
    submitArtiRootPoseOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        rootBlock, src, rows, rootRecords, numOutputs, wantOrientation, mask);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitArtiRootVelocityOvStageKernel(PxVec3* velBlock,
                                                            const float* src,
                                                            const PxU32* rows,
                                                            const PxU32 numOutputs,
                                                            const uint8_t* mask)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs && isRowCommitted(mask, i))
    {
        const float* o = src + static_cast<size_t>(i) * 3;
        velBlock[rows[i]] = PxVec3(o[0], o[1], o[2]);
    }
}

bool submitArtiRootVelocityOvStage(PxVec3* velBlock, const float* src, const PxU32* rows, const PxU32 numOutputs,
                                   cudaStream_t stream, const uint8_t* mask)
{
    if (numOutputs == 0)
        return true;
    submitArtiRootVelocityOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(velBlock, src, rows,
                                                                                        numOutputs, mask);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitArtiTendonPropertyOvStageKernel(float* tendonStructs,
                                                             const float* src,
                                                             const PxU32 numOutputs,
                                                             const PxU32 simMaxTendons,
                                                             const PxU32 structFloats,
                                                             const PxU32 fieldOffset,
                                                             const PxU32 comp,
                                                             const ArticulationTendonOvStageRecord* records,
                                                             const uint8_t* mask,
                                                             const TendonFieldWrite mode)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs && isRowCommitted(mask, i))
    {
        const ArticulationTendonOvStageRecord r = records[i];
        const PxU32 base = (r.viewArtiIdx * simMaxTendons + r.tendonIdx) * structFloats + fieldOffset;
        switch (mode)
        {
        case TendonFieldWrite::eSet:
            for (PxU32 c = 0; c < comp; ++c)
                tendonStructs[base + c] = src[i * comp + c];
            break;
        case TendonFieldWrite::eLowPushHigh:
            tendonStructs[base] = src[i];
            tendonStructs[base + 1] = fmaxf(tendonStructs[base + 1], src[i]);
            break;
        case TendonFieldWrite::eHighPushLow:
            tendonStructs[base] = fminf(tendonStructs[base], src[i]);
            tendonStructs[base + 1] = src[i];
            break;
        }
    }
}

bool submitArtiTendonPropertyOvStage(float* tendonStructs, const float* src, const PxU32 numOutputs,
                                     const PxU32 simMaxTendons, const PxU32 structFloats,
                                     const PxU32 fieldOffset, const PxU32 comp,
                                     const ArticulationTendonOvStageRecord* records, cudaStream_t stream,
                                     const uint8_t* mask, const TendonFieldWrite mode)
{
    if (numOutputs == 0)
        return true;
    submitArtiTendonPropertyOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        tendonStructs, src, numOutputs, simMaxTendons, structFloats, fieldOffset, comp, records, mask, mode);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void submitArtiDofAttributeKernel(float* dst,
                                                    const float* src,
                                                    const PxU32* srcIndices,
                                                    PxU32* dirtyArtiGpuIndices,
                                                    const PxU32 numDofs,
                                                    const PxU32 maxDofs,
                                                    const PxU32 simMaxDofs,
                                                    const PxU32 numArtis,
                                                    const GpuArticulationDofRecord* dofRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numDofs)
    {
        PxU32 dofIndex = i % maxDofs;
        PxU32 artiIndex = i / maxDofs;
        PxU32 srcArtiIdx = srcIndices[artiIndex];
        if(srcArtiIdx < numArtis)
        {
            PxU32 srcDofIdx = srcArtiIdx * maxDofs + dofIndex;
            PxU32 dstDofIdx = artiIndex * simMaxDofs + dofIndex;
            PxU32 dstArtiIdx = dofRecords[srcDofIdx].physxArtiIdx;
            bool body0IsParent = dofRecords[srcDofIdx].body0IsParent;
            // printf("%u -> %u: %f\n", srcDofIdx, dstDofIdx, src[srcDofIdx]);
            dst[dstDofIdx] = body0IsParent ? src[srcDofIdx] : -src[srcDofIdx];
            if (dofIndex == 0)
                dirtyArtiGpuIndices[artiIndex] = dstArtiIdx;
        }
    }
}

bool submitArtiDofAttribute(float* dst,
                            const float* src,
                            const PxU32* srcArtiIndices,
                            PxU32* dirtyArtiGpuIndices,
                            const PxU32 numDofs,
                            const PxU32 maxDofs,
                            const PxU32 simMaxDofs,
                            const PxU32 numArtis,
                            const GpuArticulationDofRecord* dofRecords)
{
    submitArtiDofAttributeKernel<<<(numDofs + 1023) / 1024, 1024>>>(dst, src, srcArtiIndices, dirtyArtiGpuIndices,
                                                                    numDofs, maxDofs, simMaxDofs, numArtis, dofRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchFixedTendonStiffnessKernel(float* dst,
                                                       const ::physx::PxGpuFixedTendonData* tendonProperties,
                                                       const PxU32 numTendons,
                                                       const PxU32 maxTendons,
                                                       const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].stiffness;
    }
}

bool fetchFixedTendonStiffness(float* dst,
                               const ::physx::PxGpuFixedTendonData* tendonProperties,
                               const PxU32 numTendons,
                               const PxU32 maxTendons,
                               const PxU32 simMaxTendons)
{
    fetchFixedTendonStiffnessKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchFixedTendonDampingKernel(float* dst,
                                                     const ::physx::PxGpuFixedTendonData* tendonProperties,
                                                     const PxU32 numTendons,
                                                     const PxU32 maxTendons,
                                                     const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].damping;
    }
}

bool fetchFixedTendonDamping(float* dst,
                             const ::physx::PxGpuFixedTendonData* tendonProperties,
                             const PxU32 numTendons,
                             const PxU32 maxTendons,
                             const PxU32 simMaxTendons)
{
    fetchFixedTendonDampingKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchFixedTendonLimitStiffnessKernel(float* dst,
                                                            const ::physx::PxGpuFixedTendonData* tendonProperties,
                                                            const PxU32 numTendons,
                                                            const PxU32 maxTendons,
                                                            const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].limitStiffness;
    }
}

bool fetchFixedTendonLimitStiffness(float* dst,
                                    const ::physx::PxGpuFixedTendonData* tendonProperties,
                                    const PxU32 numTendons,
                                    const PxU32 maxTendons,
                                    const PxU32 simMaxTendons)
{
    fetchFixedTendonLimitStiffnessKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchFixedTendonLimitsKernel(float* dst,
                                                    const ::physx::PxGpuFixedTendonData* tendonProperties,
                                                    const PxU32 numTendons,
                                                    const PxU32 maxTendons,
                                                    const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[2 * i] = tendonProperties[srcIdx].lowLimit;
        dst[2 * i + 1] = tendonProperties[srcIdx].highLimit;
    }
}

bool fetchFixedTendonLimits(float* dst,
                            const ::physx::PxGpuFixedTendonData* tendonProperties,
                            const PxU32 numTendons,
                            const PxU32 maxTendons,
                            const PxU32 simMaxTendons)
{
    fetchFixedTendonLimitsKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchFixedTendonRestLengthKernel(float* dst,
                                                        const ::physx::PxGpuFixedTendonData* tendonProperties,
                                                        const PxU32 numTendons,
                                                        const PxU32 maxTendons,
                                                        const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].restLength;
    }
}

bool fetchFixedTendonRestLength(float* dst,
                                const ::physx::PxGpuFixedTendonData* tendonProperties,
                                const PxU32 numTendons,
                                const PxU32 maxTendons,
                                const PxU32 simMaxTendons)
{
    fetchFixedTendonRestLengthKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchFixedTendonOffsetKernel(float* dst,
                                                    const ::physx::PxGpuFixedTendonData* tendonProperties,
                                                    const PxU32 numTendons,
                                                    const PxU32 maxTendons,
                                                    const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].offset;
    }
}

bool fetchFixedTendonOffset(float* dst,
                            const ::physx::PxGpuFixedTendonData* tendonProperties,
                            const PxU32 numTendons,
                            const PxU32 maxTendons,
                            const PxU32 simMaxTendons)
{
    fetchFixedTendonOffsetKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchSpatialTendonStiffnessKernel(float* dst,
                                                         const ::physx::PxGpuSpatialTendonData* tendonProperties,
                                                         const PxU32 numTendons,
                                                         const PxU32 maxTendons,
                                                         const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].stiffness;
    }
}

bool fetchSpatialTendonStiffness(float* dst,
                                 const ::physx::PxGpuSpatialTendonData* tendonProperties,
                                 const PxU32 numTendons,
                                 const PxU32 maxTendons,
                                 const PxU32 simMaxTendons)
{
    fetchSpatialTendonStiffnessKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchSpatialTendonDampingKernel(float* dst,
                                                       const ::physx::PxGpuSpatialTendonData* tendonProperties,
                                                       const PxU32 numTendons,
                                                       const PxU32 maxTendons,
                                                       const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].damping;
    }
}

bool fetchSpatialTendonDamping(float* dst,
                               const ::physx::PxGpuSpatialTendonData* tendonProperties,
                               const PxU32 numTendons,
                               const PxU32 maxTendons,
                               const PxU32 simMaxTendons)
{
    fetchSpatialTendonDampingKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchSpatialTendonLimitStiffnessKernel(float* dst,
                                                              const ::physx::PxGpuSpatialTendonData* tendonProperties,
                                                              const PxU32 numTendons,
                                                              const PxU32 maxTendons,
                                                              const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].limitStiffness;
    }
}

bool fetchSpatialTendonLimitStiffness(float* dst,
                                      const ::physx::PxGpuSpatialTendonData* tendonProperties,
                                      const PxU32 numTendons,
                                      const PxU32 maxTendons,
                                      const PxU32 simMaxTendons)
{
    fetchSpatialTendonLimitStiffnessKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchSpatialTendonOffsetKernel(float* dst,
                                                      const ::physx::PxGpuSpatialTendonData* tendonProperties,
                                                      const PxU32 numTendons,
                                                      const PxU32 maxTendons,
                                                      const PxU32 simMaxTendons)
{

    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numTendons)
    {
        PxU32 srcTendonIdx = i % maxTendons;
        PxU32 artiIndex = i / maxTendons;
        PxU32 srcIdx = artiIndex * simMaxTendons + srcTendonIdx;
        dst[i] = tendonProperties[srcIdx].offset;
    }
}

bool fetchSpatialTendonOffset(float* dst,
                              const ::physx::PxGpuSpatialTendonData* tendonProperties,
                              const PxU32 numTendons,
                              const PxU32 maxTendons,
                              const PxU32 simMaxTendons)
{
    fetchSpatialTendonOffsetKernel<<<(numTendons + 1023) / 1024, 1024>>>(
        dst, tendonProperties, numTendons, maxTendons, simMaxTendons);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitArtiFixedTendonPropertiesKernel(::physx::PxGpuFixedTendonData* dst,
                                                             const float* stiffnesses,
                                                             const float* dampings,
                                                             const float* limitStiffnesses,
                                                             const float* limits,
                                                             const float* restLengths,
                                                             const float* offsets,
                                                             const PxU32* srcArtiIndices,
                                                             PxU32* dirtyArtiGpuIndices,
                                                             const PxU32 numArtiIndices,
                                                             const PxU32 maxTendons,
                                                             const PxU32 simMaxTendons,
                                                             const GpuArticulationFixedTendonRecord* tendonRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 artiIndex = i / maxTendons;

    if (artiIndex < numArtiIndices)
    {
        PxU32 tendonIndex = i % maxTendons;
        PxU32 srcArtiIdx = srcArtiIndices[artiIndex];
        PxU32 srcTendonIdx = srcArtiIdx * maxTendons + tendonIndex;
        PxU32 dstTendonIdx = artiIndex * simMaxTendons + tendonIndex;

        PxU32 dstArtiIdx = tendonRecords[srcTendonIdx].physxArtiIdx;
        if(tendonIndex==0)
            dirtyArtiGpuIndices[artiIndex] = dstArtiIdx;

        PxGpuFixedTendonData& dstTendonData = dst[dstTendonIdx];
        dstTendonData.stiffness = stiffnesses[srcTendonIdx];
        dstTendonData.damping = dampings[srcTendonIdx];
        dstTendonData.lowLimit = limits[srcTendonIdx * 2];
        dstTendonData.highLimit = limits[srcTendonIdx * 2 + 1];
        dstTendonData.limitStiffness = limitStiffnesses[srcTendonIdx];
        dstTendonData.restLength = restLengths[srcTendonIdx];
        dstTendonData.offset = offsets[srcTendonIdx];
    }
}

bool submitArtiFixedTendonProperties(::physx::PxGpuFixedTendonData* dst,
                                     const float* stiffnesses,
                                     const float* dampings,
                                     const float* limitStiffnesses,
                                     const float* limits,
                                     const float* restLengths,
                                     const float* offsets,
                                     const ::physx::PxU32* srcIndices,
                                     PxU32* dirtyArtiGpuIndices,
                                     const ::physx::PxU32 numIndices,
                                     const PxU32 maxTendons,
                                     const PxU32 simMaxTendons,
                                     const GpuArticulationFixedTendonRecord* tendonRecords)
{
    submitArtiFixedTendonPropertiesKernel<<<(numIndices * maxTendons + 1023) / 1024, 1024>>>(
        dst, stiffnesses, dampings, limitStiffnesses, limits, restLengths, offsets, srcIndices,
        dirtyArtiGpuIndices, numIndices, maxTendons, simMaxTendons, tendonRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitArtiSpatialTendonPropertiesKernel(::physx::PxGpuSpatialTendonData* dst,
                                                               const float* stiffnesses,
                                                               const float* dampings,
                                                               const float* limitStiffnesses,
                                                               const float* offsets,
                                                               const PxU32* srcArtiIndices,
                                                               PxU32* dirtyArtiGpuIndices,
                                                               const PxU32 numArtiIndices,
                                                               const PxU32 maxTendons,
                                                               const PxU32 simMaxTendons,
                                                               const GpuArticulationSpatialTendonRecord* tendonRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 artiIndex = i / maxTendons;

    if (artiIndex < numArtiIndices)
    {
        PxU32 tendonIndex = i % maxTendons;
        PxU32 srcArtiIdx = srcArtiIndices[artiIndex];
        PxU32 srcTendonIdx = srcArtiIdx * maxTendons + tendonIndex;
        PxU32 dstTendonIdx = artiIndex * simMaxTendons + tendonIndex;

        PxU32 dstArtiIdx = tendonRecords[srcTendonIdx].physxArtiIdx;
        if(tendonIndex==0)
            dirtyArtiGpuIndices[artiIndex] = dstArtiIdx;

        PxGpuSpatialTendonData& dstTendonData = dst[dstTendonIdx];
        dstTendonData.stiffness = stiffnesses[srcTendonIdx];
        dstTendonData.damping = dampings[srcTendonIdx];
        dstTendonData.limitStiffness = limitStiffnesses[srcTendonIdx];
        dstTendonData.offset = offsets[srcTendonIdx];
    }
}

bool submitArtiSpatialTendonProperties(::physx::PxGpuSpatialTendonData* dst,
                                       const float* stiffnesses,
                                       const float* dampings,
                                       const float* limitStiffnesses,
                                       const float* offsets,
                                       const ::physx::PxU32* srcIndices,
                                       PxU32* dirtyArtiGpuIndices,
                                       const ::physx::PxU32 numIndices,
                                       const PxU32 maxTendons,
                                       const PxU32 simMaxTendons,
                                       const GpuArticulationSpatialTendonRecord* tendonRecords)
{
    submitArtiSpatialTendonPropertiesKernel<<<(numIndices * maxTendons + 1023) / 1024, 1024>>>(
        dst, stiffnesses, dampings, limitStiffnesses, offsets, srcIndices,
        dirtyArtiGpuIndices, numIndices, maxTendons, simMaxTendons, tendonRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__device__ __forceinline__ static void transformToParentFrame(PxVec3 &force, PxVec3 &torque,
                                                              const GpuArticulationLinkRecord &linkRecord,
                                                              const PxU32 linkIdx,const PxU32 parentLinkIdx,
                                                              const PxTransform *linkTransforms)
{
    const PxTransform& transformC = linkIdx != 0xffffffff ? linkTransforms[linkIdx] : PxTransform(PxIdentity);
    const PxTransform& transformP =
    parentLinkIdx != 0xffffffff ? linkTransforms[parentLinkIdx] : PxTransform(PxIdentity);
    const PxTransform& jointC = linkRecord.jointChild;
    const PxTransform& jointP = linkRecord.jointParent;

    const PxTransform GpLp = transformP * jointP;
    const PxTransform GcLc = transformC * jointC;
    PxQuat J = GpLp.q.getConjugate() * GcLc.q;
    PxVec3 d = GpLp.p - GcLc.p; // global frame
    d = GcLc.q.rotateInv(d); // local frame
    torque = -1.0f * J.rotate(torque - d.cross(force));
    force = -1.0f * J.rotate(force);
}

__global__ static void fetchArtiLinkIncomingJointForceKernel(PhysxGpuSpatialForces *dst,
                                                             const PhysxGpuSpatialForces *src,
                                                             const PxTransform *linkTransforms, const PxU32 numLinks,
                                                             const PxU32 maxLinks, const PxU32 simMaxLinks,
                                                             const GpuArticulationLinkRecord *linkRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numLinks)
    {
        PxU32 linkIdx = i % maxLinks;
        PxU32 artiIndex = i / maxLinks;
        PxU32 srcIdx = artiIndex * simMaxLinks + linkIdx;
        PxQuat rotation = linkRecords[i].physxToUsdJointRotation;
        bool body0IsParent = linkRecords[i].body0IsParent;
        PhysxGpuSpatialForces sf = src[srcIdx];

        if (!body0IsParent)
            transformToParentFrame(sf.force, sf.torque, linkRecords[i], srcIdx,
                                   linkRecords[i].incomingLinkIdx, linkTransforms);

        sf.force = rotation.rotate(sf.force);
        sf.torque = rotation.rotate(sf.torque);

        dst[i] = sf;
    }
}

bool fetchArtiLinkIncomingJointForce(PhysxGpuSpatialForces *dst, const PhysxGpuSpatialForces *src,
                                     const PxTransform *linkTransforms, const PxU32 numLinks, const PxU32 maxLinks,
                                     const PxU32 simMaxLinks, const GpuArticulationLinkRecord *linkRecords,
                                     cudaStream_t stream)
{
    fetchArtiLinkIncomingJointForceKernel<<<(numLinks + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, linkTransforms, numLinks, maxLinks, simMaxLinks, linkRecords);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

// ovstage counterpart of the dense fetch above, for a fixed-width vector attribute: reorders its rows
// into ovstage prim order. No frame folding -- the USD joint rotation and the parent-frame swap when
// body0 is not the parent were resolved by the dense pass that produced `src`. `maxLinks` is the stride
// of THAT row (the view's), not the scene's.
__global__ static void fetchArtiLinkVectorOvStageKernel(float* dst,
                                                        const float* src,
                                                        const PxU32 numOutputs,
                                                        const PxU32 maxLinks,
                                                        const PxU32 comp,
                                                        const ArticulationLinkOvStageRecord* records)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numOutputs)
    {
        const ArticulationLinkOvStageRecord r = records[i];
        const PxU32 srcIdx = (r.viewArtiIdx * maxLinks + r.physxLinkIdx) * comp;
        for (PxU32 c = 0; c < comp; ++c)
            dst[i * comp + c] = src[srcIdx + c];
    }
}

bool fetchArtiLinkVectorOvStage(float* dst, const float* src, const PxU32 numOutputs, const PxU32 maxLinks,
                                const PxU32 comp, const ArticulationLinkOvStageRecord* records, cudaStream_t stream)
{
    if (numOutputs == 0)
        return true;
    fetchArtiLinkVectorOvStageKernel<<<(numOutputs + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, numOutputs, maxLinks, comp, records);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

__global__ static void compactContactSensorRowsOvStageKernel(float* dst,
                                                             const float* src,
                                                             const PxU32* rowIdx,
                                                             const PxU32 numRows,
                                                             const PxU32 comp)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numRows)
    {
        const PxU32 srcIdx = rowIdx[i] * comp;
        for (PxU32 c = 0; c < comp; ++c)
            dst[i * comp + c] = src[srcIdx + c];
    }
}

bool compactContactSensorRowsOvStage(
    float* dst, const float* src, const PxU32* rowIdx, const PxU32 numRows, const PxU32 comp)
{
    if (numRows == 0)
        return true;
    compactContactSensorRowsOvStageKernel<<<(numRows + 1023) / 1024, 1024>>>(dst, src, rowIdx, numRows, comp);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void packSdfQueryPointsOvStageKernel(PxVec4* dst, const PxVec3* src, const PxU32 count)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count)
        dst[i] = PxVec4(src[i].x, src[i].y, src[i].z, 0.0f);
}

bool packSdfQueryPointsOvStage(PxVec4* dst, const PxVec3* src, const PxU32 count)
{
    if (count == 0)
        return true;
    // Rounded up without forming count + 1023, which wraps for a count near the PxU32 limit.
    packSdfQueryPointsOvStageKernel<<<count / 1024 + (count % 1024 != 0 ? 1 : 0), 1024>>>(dst, src, count);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchRawContactOtherBodyOvStageKernel(uint64_t* dst, const uint64_t* pairs, const PxU32 count)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count)
        dst[i] = pairs[2 * size_t(i) + 1];
}

bool fetchRawContactOtherBodyOvStage(uint64_t* dst, const uint64_t* pairs, const PxU32 count)
{
    if (count == 0)
        return true;
    fetchRawContactOtherBodyOvStageKernel<<<(count + 1023) / 1024, 1024>>>(dst, pairs, count);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchDofProjectionForceKernel(float* dst,
                                                      const PhysxGpuSpatialForces* src,
                                                      const PxTransform* linkTransforms,
                                                      const PxU32 numLinks,const PxU32 maxLinks,
                                                      const PxU32 simMaxLinks,
                                                      const GpuArticulationLinkRecord* linkRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numLinks)
    {
        PxU32 linkIdx = i % maxLinks;
        PxU32 artiIndex = i / maxLinks;
        PxU32 srcIdx = artiIndex * simMaxLinks + linkIdx;

        const GpuArticulationLinkRecord& linkRecord = linkRecords[i];
        PxQuat rotation = linkRecord.physxToUsdJointRotation;
        PxArticulationJointType::Enum jointType = linkRecord.incomingJointType;
        PxU32 dofOffset = linkRecord.dofOffset;
        PhysxGpuSpatialForces sf = src[srcIdx];
        bool body0IsParent = linkRecord.body0IsParent;
        
        if (!body0IsParent)
            transformToParentFrame(sf.force, sf.torque, linkRecord, srcIdx, linkRecords[i].incomingLinkIdx,
                                   linkTransforms);

        switch (jointType)
        {
        case PxArticulationJointType::eFIX:
            break;
        case PxArticulationJointType::eREVOLUTE:
            dst[dofOffset] = sf.torque.x;
            break;
        case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
            dst[dofOffset] = sf.torque.x;
            break;
        case PxArticulationJointType::ePRISMATIC:
            dst[dofOffset] = sf.force.x;
            break;
        case PxArticulationJointType::eSPHERICAL:
            PxVec3 usdSpaceTorque = rotation.rotate(sf.torque);
            // dst[dofOffset + 0] = usdSpaceTorque.x;
            // dst[dofOffset + 1] = usdSpaceTorque.y;
            // dst[dofOffset + 2] = usdSpaceTorque.z;
            PxU32 DofIdx = 0;
            FreeD6RotationAxesFlags freeAxes = linkRecord.D6RotationAxes;
            if(freeAxes & FreeD6RotationAxesFlag::eTWIST)
                dst[dofOffset + DofIdx++] = usdSpaceTorque.x;
            if(freeAxes & FreeD6RotationAxesFlag::eSWING1)
                dst[dofOffset + DofIdx++] = usdSpaceTorque.y;
            if(freeAxes & FreeD6RotationAxesFlag::eSWING2)
                dst[dofOffset + DofIdx++] = usdSpaceTorque.z;
            // printf("gpu eSPHERICAL offset = %u, freeAxes=%u, torque= %f,%f,%f usdSpaceTorque= %f,%f,%f \n", dofOffset, (PxU32)freeAxes,
            //        sf.torque.x, sf.torque.y, sf.torque.z, usdSpaceTorque.x, usdSpaceTorque.y, usdSpaceTorque.z);
            break;
        }
    }
}

bool fetchDofProjectionForce(float *dst, const PhysxGpuSpatialForces *src, const PxTransform *linkTransforms,
                             const PxU32 numLinks, const PxU32 maxLinks, const PxU32 simMaxLinks,
                             const GpuArticulationLinkRecord *linkRecords, cudaStream_t stream)
{
    fetchDofProjectionForceKernel<<<(numLinks + 1023) / 1024, 1024, 0, stream>>>(
        dst, src, linkTransforms, numLinks, maxLinks, simMaxLinks, linkRecords);
    return drainOnLaunchFailure(CHECK_CUDA(cudaGetLastError()), stream);
}

//
// rigid bodies
//

__global__ static void fetchRbTransformsKernel(TensorTransform* dst,
                                               const PxTransform* actorData,
                                               const PxTransform* linkTransforms,
                                               const PxU32 numRbs,
                                               const PxU32 simMaxLinks,
                                               const GpuRigidBodyRecord* rbRecords)
{
    PxU32 rbIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (rbIdx < numRbs)
    {
        PxU32 tensorIdx = rbRecords[rbIdx].tensorRdIdx;
        const PxVec3 &origin = rbRecords[rbIdx].origin;
        if (tensorIdx != 0xffffffff)
        {
            // printf("RD %u -> RB %u %f\n", tensorIdx, rbIdx, actorData[tensorIdx].p.x);
            dst[rbIdx].p = actorData[tensorIdx].p - origin;
            dst[rbIdx].q = actorData[tensorIdx].q;
        }
        // OMPE-94459: gate the articulation path on tensorArtiIdx AND linkIdx
        // both being valid (same fix as fetchRbVelAccKernel). A disabled rigid
        // dynamic has tensorRdIdx == sentinel and both arti fields == sentinel;
        // the old compound `tensorArtiIdx * simMaxLinks + linkIdx` overflowed to
        // a non-sentinel index and read garbage from linkTransforms. For such a
        // body, leave the identity here -- getTransforms patches disabled rigid
        // dynamics with their frozen CPU pose afterwards.
        else if (rbRecords[rbIdx].tensorArtiIdx != 0xffffffff && rbRecords[rbIdx].linkIdx != 0xffffffff)
        {
            PxU32 linkIdx = rbRecords[rbIdx].tensorArtiIdx * simMaxLinks + rbRecords[rbIdx].linkIdx;
            // printf("Link %u -> RB %u %f\n", linkIdx, rbIdx, linkTransforms[linkIdx].p.x);
            dst[rbIdx].p = linkTransforms[linkIdx].p - origin;
            dst[rbIdx].q = linkTransforms[linkIdx].q;
        }
        else
        {
            dst[rbIdx].p = PxVec3(0.f);
            dst[rbIdx].q = PxQuat(PxIdentity);
        }
    }
}

bool fetchRbTransforms(TensorTransform* dst,
                       const PxTransform* actorData,
                       const PxTransform* linkTransforms,
                       const PxU32 numRbs,
                       const PxU32 simMaxLinks,
                       const GpuRigidBodyRecord* rbRecords)
{
    fetchRbTransformsKernel<<<(numRbs + 1023) / 1024, 1024>>>(dst, actorData, linkTransforms, numRbs, simMaxLinks, rbRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitRbTransformsKernel(PxTransform *dstActorData, PxTransform *dstRootTransforms,
                                                ActorGpuFlags *rdDirtyFlags, ArticulationGpuFlags *artiDirtyFlags,
                                                const TensorTransform *src, const PxU32 *srcRbIndices,
                                                const PxU32 numRbIndices, const PxU32 numRbs, const GpuRigidBodyRecord *rbRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numRbIndices)
    {
        PxU32 rbIdx = srcRbIndices[i];
        if (rbIdx < numRbs)
        {
            const PxVec3 &origin = rbRecords[rbIdx].origin;
            PxU32 tensorIdx = rbRecords[rbIdx].tensorRdIdx;
            if (tensorIdx != 0xffffffff)
            {
                dstActorData[tensorIdx].p = src[rbIdx].p + origin;
                dstActorData[tensorIdx].q = src[rbIdx].q;
                rdDirtyFlags[tensorIdx] |= ActorGpuFlag::eACTOR_DATA;
                // printf("rib %d %d %f\n", rbIdx, tensorIdx, dstActorData[tensorIdx].p.z);
            }
            else if (rbRecords[rbIdx].isRootLink)
            {
                PxU32 tensorIdx = rbRecords[rbIdx].tensorArtiIdx;
                dstRootTransforms[tensorIdx].p = src[rbIdx].p + origin;
                dstRootTransforms[tensorIdx].q = src[rbIdx].q;
                artiDirtyFlags[tensorIdx] |= ArticulationGpuFlag::eROOT_TRANSFORM;
                // printf("art %d %d %f\n", rbIdx, tensorIdx, dstRootTransforms[tensorIdx].p.z);
            }
        }
    }
}

bool submitRbTransforms(PxTransform *dstActorData, PxTransform *dstRootTransforms, ActorGpuFlags *rdDirtyFlags,
                        ArticulationGpuFlags *artiDirtyFlags, const TensorTransform *src, const PxU32 *srcRbIndices,
                        const PxU32 numRbIndices, const PxU32 numRbs, const GpuRigidBodyRecord *rbRecords)
{
    submitRbTransformsKernel<<<(numRbIndices + 1023) / 1024, 1024>>>(dstActorData, dstRootTransforms, rdDirtyFlags,
                                                                     artiDirtyFlags, src, srcRbIndices, numRbIndices,
                                                                     numRbs, rbRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchRbVelAccKernel(TensorVelAcc *dst, const PxVec3 *actorLinVel, const PxVec3 *actorAngVel,
                                           const PxVec3 *linkLinVel, const PxVec3 *linkAngVel, const PxU32 numRbs,
                                           const PxU32 simMaxLinks, const GpuRigidBodyRecord *rbRecords)
{
    PxU32 rbIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (rbIdx < numRbs)
    {
        // A currently-disabled body has tensorRdIdx == sentinel (set by
        // refreshRdGpuIndices, OMPE-94459), so it falls through to the else
        // branch and reads zero -- matching CPU getLinearVelocity, which
        // returns the zeroed core state for a disabled actor.
        PxU32 tensorIdx = rbRecords[rbIdx].tensorRdIdx;
        if (tensorIdx != 0xffffffff)
        {
            dst[rbIdx].linear = actorLinVel[tensorIdx];
            dst[rbIdx].angular = actorAngVel[tensorIdx];
        }
        // OMPE-94459 ask C2: gate the articulation path on tensorArtiIdx AND
        // linkIdx both being valid. Previously the compound `tensorArtiIdx *
        // simMaxLinks + linkIdx` was checked against 0xffffffff after
        // multiplication, which fails to detect the case where both fields
        // are sentinel (overflowed product != 0xffffffff in general). With
        // the GpuRigidBodyView ctor now marking disabled RBs as
        // tensorRdIdx=sentinel/tensorArtiIdx=sentinel, the kernel needs to
        // recognise that and write zero instead of reading garbage.
        else if (rbRecords[rbIdx].tensorArtiIdx != 0xffffffff &&
                 rbRecords[rbIdx].linkIdx != 0xffffffff)
        {
            PxU32 tensorIdx = rbRecords[rbIdx].tensorArtiIdx * simMaxLinks + rbRecords[rbIdx].linkIdx;
            dst[rbIdx].linear = linkLinVel[tensorIdx];
            dst[rbIdx].angular = linkAngVel[tensorIdx];
        }
        else
        {
            // Disabled-actor / unresolved-body path: explicit zero.
            dst[rbIdx].linear = PxVec3(0.f);
            dst[rbIdx].angular = PxVec3(0.f);
        }
    }
}

bool fetchRbVelAcc(TensorVelAcc *dst, const PxVec3 *actorLinVel, const PxVec3 *actorAngVel, const PxVec3 *linkLinVel,
                   const PxVec3 *linkAngVel, const PxU32 numRbs, const PxU32 simMaxLinks,
                   const GpuRigidBodyRecord *rbRecords)
{
    fetchRbVelAccKernel<<<(numRbs + 1023) / 1024, 1024>>>(dst, actorLinVel, actorAngVel, linkLinVel, linkAngVel, numRbs,
                                                          simMaxLinks, rbRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitRbVelocitiesKernel(PxVec3 *dstLinearVel, PxVec3 *dstAngularVel, PxVec3 *dstRootLinearVel,
                                                PxVec3 *dstRootAngularVel, ActorGpuFlags *rdDirtyFlags,
                                                ArticulationGpuFlags *artiDirtyFlags, const TensorVelAcc *src,
                                                const PxU32 *srcRbIndices, const PxU32 numRbIndices, const PxU32 numRbs,
                                                const GpuRigidBodyRecord *rbRecords)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numRbIndices)
    {
        PxU32 rbIdx = srcRbIndices[i];
        if (rbIdx < numRbs)
        {
            PxU32 tensorIdx = rbRecords[rbIdx].tensorRdIdx;
            if (tensorIdx != 0xffffffff)
            {
                dstLinearVel[tensorIdx] = src[rbIdx].linear;
                dstAngularVel[tensorIdx] = src[rbIdx].angular;
                rdDirtyFlags[tensorIdx] |= ActorGpuFlag::eACTOR_DATA;
                // printf("rib %d %d %f\n", rbIdx, tensorIdx, dstLinearVel[tensorIdx].z);
            }
            else if (rbRecords[rbIdx].isRootLink)
            {
                PxU32 tensorIdx = rbRecords[rbIdx].tensorArtiIdx;
                dstRootLinearVel[tensorIdx] = src[rbIdx].linear;
                dstRootAngularVel[tensorIdx] = src[rbIdx].angular;
                artiDirtyFlags[tensorIdx] |= ArticulationGpuFlag::eROOT_VELOCITY;
                // printf("art %d %d %f\n", rbIdx, tensorIdx, dstRootLinearVel[tensorIdx].z);
            }
        }
    }
}

bool submitRbVelocities(PxVec3 *dstLinearVel, PxVec3 *dstAngularVel, PxVec3 *dstRootLinearVel,
                        PxVec3 *dstRootAngularVel, ActorGpuFlags *rdDirtyFlags,
                        ArticulationGpuFlags *artiDirtyFlags, const TensorVelAcc *src, const PxU32 *srcRbIndices,
                        const PxU32 numRbIndices, const PxU32 numRbs, const GpuRigidBodyRecord *rbRecords)
{
    submitRbVelocitiesKernel<<<(numRbIndices + 1023) / 1024, 1024>>>(
        dstLinearVel, dstAngularVel, dstRootLinearVel, dstRootAngularVel, rdDirtyFlags, artiDirtyFlags, src,
        srcRbIndices, numRbIndices, numRbs, rbRecords);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

template <bool setDirtyFlag>
__device__ __forceinline__ void applyForces(PxVec3 *dstForces, PxVec3 *dstTorques, ActorGpuFlags *rdDirtyFlags,
                                            ArticulationGpuFlags *artiDirtyFlags,
                                            ArticulationGpuFlags *artiLinksDirtyFlags, const PxU32 rbIdx,
                                            const PxU32 rdOrLinkIdx, const PxU32 tensorIdx, const PxVec3 *srcForces,
                                            const PxVec3 *srcTorques, const PxVec3 *srcPositions,
                                            const PxVec3 &bodyPositions, const PxQuat &bodyRotation, PxVec3 com,
                                            const bool isArticulation, const bool isGlobal, const bool submitForces,
                                            const bool submitTorques, const bool applyAtPos)
{

    if (submitForces)
    {
        PxVec3 extForce = srcForces[rbIdx];
        if (!isGlobal)
            extForce = bodyRotation.rotate(extForce);

        dstForces[rdOrLinkIdx] = {extForce.x, extForce.y, extForce.z};
        if (setDirtyFlag)
        {
            if (!isArticulation)
                rdDirtyFlags[tensorIdx] |= ActorGpuFlag::eFORCE;
            else
            {
                artiDirtyFlags[tensorIdx] |= ArticulationGpuFlag::eLINK_FORCE;
                artiLinksDirtyFlags[rdOrLinkIdx] |= ArticulationGpuFlag::eLINK_FORCE;                
            }
        }

        if (applyAtPos)
        {
            PxTransform frame{bodyPositions, bodyRotation};
            PxVec3 position = srcPositions[rbIdx];
            if (!isGlobal)
                position = frame.transform(position);
            com = frame.transform(com);
            PxVec3 torque = (position - com).cross(extForce);
            dstTorques[rdOrLinkIdx] = {torque.x, torque.y, torque.z};
            if (setDirtyFlag)
            {
                if (!isArticulation)
                    rdDirtyFlags[tensorIdx] |= ActorGpuFlag::eTORQUE;
                else
                {
                    artiDirtyFlags[tensorIdx] |= ArticulationGpuFlag::eLINK_TORQUE;
                    artiLinksDirtyFlags[rdOrLinkIdx] |= ArticulationGpuFlag::eLINK_TORQUE;
                }
            }
        }
    }

    if (submitTorques)
    {
        PxVec3 extTorque = srcTorques[rbIdx];
        if (!isGlobal)
            extTorque = bodyRotation.rotate(extTorque);
        // add to previous torques if applyAtPos
        PxVec3 curr = {dstTorques[rdOrLinkIdx].x, dstTorques[rdOrLinkIdx].y, dstTorques[rdOrLinkIdx].z};
        PxVec3 extT = {extTorque.x, extTorque.y, extTorque.z};
        dstTorques[rdOrLinkIdx] = applyAtPos ? curr + extT : extT;
        if (setDirtyFlag)
        {
            if (!isArticulation)
                rdDirtyFlags[tensorIdx] |= ActorGpuFlag::eTORQUE;
            else
                artiDirtyFlags[tensorIdx] |= ArticulationGpuFlag::eLINK_TORQUE;
        }
    }
}

__global__ static void submitArtiForcesKernel(PxVec3 *dstLinkForces, PxVec3 *dstLinkTorques, PxU32 *dirtyArtiGpuIndices,
                                              const PxTransform *linkTransforms, const PxVec3 *linksComs,
                                              const PxVec3 *srcForces, const PxVec3 *srcTorques,
                                              const PxVec3 *srcPositions, const PxU32 *srcArtiIndices,
                                              const PxU32 numArtIndices, const PxU32 numLinks, const PxU32 simMaxLinks,
                                              const GpuArticulationLinkRecord *linkRecords, const bool isGlobal,
                                              const bool submitForces, const bool submitTorques,
                                              const bool applyAtPosition)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 maxLinks = numLinks / numArtIndices;
    if (i < numLinks)
    {
        PxU32 linkIndex = i % maxLinks;
        PxU32 artiIndex = i / maxLinks;
        PxU32 srcArtiIdx = srcArtiIndices[artiIndex];
        PxU32 srcLinkIdx = srcArtiIdx * maxLinks + linkIndex;
        PxVec3 com = linksComs[srcLinkIdx];
        PxU32 physxLinkIdx = linkRecords[srcLinkIdx].physxLinkIdx;
        PxU32 tensorLinkIdx = srcArtiIdx * simMaxLinks + linkIndex;
        // setArticulationData reads the data of dirtyArtiGpuIndices[artiIndex] from block artiIndex.
        PxU32 dstLinkIdx = artiIndex * simMaxLinks + linkIndex;

        // printf("i = %d, srcArtiIdx=%d, physxLinkIdx=%d, srcLinkIdx=%d\n", i, srcArtiIdx, physxLinkIdx, srcLinkIdx);
        if (linkIndex == 0)
        {
            dirtyArtiGpuIndices[artiIndex] = linkRecords[srcLinkIdx].physxArtiIdx;
        }
        if (physxLinkIdx != 0xffffffff)
        {
            applyForces<false>(dstLinkForces, dstLinkTorques, nullptr, nullptr, nullptr, srcLinkIdx, dstLinkIdx, 0,
                               srcForces, srcTorques, srcPositions, linkTransforms[tensorLinkIdx].p,
                               linkTransforms[tensorLinkIdx].q, com, true, isGlobal, submitForces, submitTorques,
                               applyAtPosition);
        }
    }
}

bool submitArtiLinkForces(PxVec3 *dstLinkForces, PxVec3 *dstLinkTorques, PxU32 *dirtyArtiGpuIndices,
                          const PxTransform *linkTransforms, const PxVec3 *linksComs, const PxVec3 *srcForces,
                          const PxVec3 *srcTorques, const PxVec3 *srcPositions, const PxU32 *srcArtiIndices,
                          const PxU32 numArtIndices, const PxU32 numLinks, const PxU32 simMaxLinks,
                          const GpuArticulationLinkRecord *linkRecords, const bool isGlobal, const bool submitForces,
                          const bool submitTorques, const bool applyAtPosition)
{
    submitArtiForcesKernel<<<(numLinks + 1023) / 1024, 1024>>>(
        dstLinkForces, dstLinkTorques, dirtyArtiGpuIndices, linkTransforms, linksComs, srcForces, srcTorques,
        srcPositions, srcArtiIndices, numArtIndices, numLinks, simMaxLinks, linkRecords, isGlobal, submitForces,
        submitTorques, applyAtPosition);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitRbForcesKernel(PxVec3* dstActorForces,
                                            PxVec3* dstActorTorques,
                                            PxVec3* dstLinkForces,
                                            PxVec3* dstLinkTorques,
                                            ActorGpuFlags* rdDirtyFlags,
                                            ArticulationGpuFlags* artiDirtyFlags,
                                            ArticulationGpuFlags* artiLinksDirtyFlags,
                                            const PxTransform* linkTransforms,
                                            const PxTransform* actorData,
                                            const PxVec3* coms,
                                            const PxVec3* srcForces,
                                            const PxVec3* srcTorques,
                                            const PxVec3* srcPositions,
                                            const PxU32* srcRbIndices,
                                            const PxU32 numRbIndices,
                                            const PxU32 simMaxLinks,
                                            const GpuRigidBodyRecord* rbRecords,
                                            const bool isGlobal,
                                            const bool submitForces,
                                            const bool submitTorques,
                                            const bool applyAtPos)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numRbIndices)
    {
        PxU32 rbIdx = srcRbIndices[i];
        PxVec3 com = coms[rbIdx];
        PxU32 rdIdx = rbRecords[rbIdx].physxRdIdx;
        if (rdIdx != 0xffffffff) {
          PxU32 tensorIdx = rbRecords[rbIdx].tensorRdIdx;
        //   printf("source index = %d, rbId=%d, tensorIdx=%d, rdIdx=%d\n", i, rbIdx, rdIdx, tensorIdx);
          applyForces<true>(dstActorForces, dstActorTorques, rdDirtyFlags, artiDirtyFlags, artiLinksDirtyFlags, rbIdx, tensorIdx,
                            tensorIdx, srcForces, srcTorques, srcPositions, actorData[tensorIdx].p,
                            actorData[tensorIdx].q, com, false, isGlobal, submitForces, submitTorques, applyAtPos);
        } else {
          PxU32 linkIdx = rbRecords[rbIdx].physxLinkIdx;
          if (linkIdx != 0xffffffff) {
            // PxU32 artiIdx = rbRecords[rbIdx].physxArtiIdx;
            PxU32 tensorArtiIdx = rbRecords[rbIdx].tensorArtiIdx;
            PxU32 tensorLinkIdx = tensorArtiIdx * simMaxLinks + rbRecords[rbIdx].linkIdx;
            // printf("rbIdx %u linkIdx = %u, artiIdx= %u, tensorIdx=%u \n",
            // rbIdx, linkIdx, artiIdx, tensorIdx);
            applyForces<true>(dstLinkForces, dstLinkTorques, rdDirtyFlags, artiDirtyFlags, artiLinksDirtyFlags, rbIdx,
                              tensorLinkIdx, tensorArtiIdx, srcForces, srcTorques, srcPositions,
                              linkTransforms[tensorLinkIdx].p, linkTransforms[tensorLinkIdx].q, com, true, isGlobal,
                              submitForces, submitTorques, applyAtPos);
          }
        }
    }
}

bool submitRbForces(PxVec3* dstActorForces,
                    PxVec3* dstActorTorques,
                    PxVec3* dstLinkForces,
                    PxVec3* dstLinkTorques,
                    ActorGpuFlags* rdDirtyFlags,
                    ArticulationGpuFlags* artiDirtyFlags,
                    ArticulationGpuFlags* artiLinksDirtyFlags,
                    const PxTransform* linkTransforms,
                    const PxTransform* actorData,
                    const PxVec3* actorAndLinksComs,
                    const PxVec3* srcForces,
                    const PxVec3* srcTorques,
                    const PxVec3* srcPositions,
                    const PxU32* srcRbIndices,
                    const PxU32 numRbIndices,
                    const PxU32 simMaxLinks,
                    const GpuRigidBodyRecord* rbRecords,
                    const bool isGlobal,
                    const bool submitForces,
                    const bool submitTorques,
                    const bool applyAtPosition)
{
    submitRbForcesKernel<<<(numRbIndices + 1023) / 1024, 1024>>>(
        dstActorForces, dstActorTorques, dstLinkForces, dstLinkTorques, rdDirtyFlags, artiDirtyFlags,
        artiLinksDirtyFlags, linkTransforms, actorData, actorAndLinksComs, srcForces, srcTorques, srcPositions,
        srcRbIndices, numRbIndices, simMaxLinks, rbRecords, isGlobal, submitForces, submitTorques, applyAtPosition);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}


//
// rigid contacts
//

constexpr static PxU32 INVALID_IDX = 0xffffffff;

__device__ __forceinline__ static PxU32 getRigidContactReferentIndex(const PxNodeIndex& nodeIndex,
                                                                     const RigidContactReferentTables& referents)
{
    if (nodeIndex.isArticulation())
    {
        if (nodeIndex.index() >= referents.nodeIdx2ArtiGpuIdxCount)
        {
            return INVALID_IDX;
        }
        PxU32 artiIdx = referents.nodeIdx2ArtiGpuIdx[nodeIndex.index()];
        PxU32 linkId = nodeIndex.articulationLinkId();
        if (artiIdx == INVALID_IDX || linkId >= referents.maxLinks)
        {
            return INVALID_IDX;
        }
        PxU64 linkIdx = PxU64(artiIdx) * referents.maxLinks + linkId;
        return linkIdx < referents.linkContactIndexCount ? referents.linkContactIndices[linkIdx] : INVALID_IDX;
    }
    else if (nodeIndex.isValid())
    {
        PxU32 rdIdx = nodeIndex.index();
        return rdIdx < referents.rdContactIndexCount ? referents.rdContactIndices[rdIdx] : INVALID_IDX;
    }

    // TODO: check shape id

    return INVALID_IDX;
}

__device__ __forceinline__ static PxVec3 getRigidContactPairNormalForce(const PxGpuContactPair& cp, float timeStepInv)
{
    PxVec3 impulse(0.0f);

    for (PxU32 i = 0; i < cp.nbPatches; i++)
    {
        const PxContactPatch& patch = reinterpret_cast<const PxContactPatch*>(cp.contactPatches)[i];
        float impulseMag = 0.0f;
        for (PxU32 j = patch.startContactIndex; j < patch.startContactIndex + patch.nbContacts; j++)
        {
            impulseMag += cp.contactForces[j];
        }
        impulse += impulseMag * patch.normal;
    }

    return timeStepInv * impulse;
}


__device__ __forceinline__ static PxVec3 getRigidContactPairFrictionForce(const PxGpuContactPair& cp, float timeStepInv)
{
    PxVec3 impulse(0.0f);
    if (cp.frictionPatches)
    {
        const PxFrictionPatch* patches = reinterpret_cast<const PxFrictionPatch*>(cp.frictionPatches);
        for (PxU32 i = 0; i < cp.nbPatches; ++i)
            for (PxU32 j = 0; j < patches[i].anchorCount; ++j)
                impulse += patches[i].anchorImpulses[j];
    }
    return timeStepInv * impulse;
}

template <ForceComponent component>
__global__ static void fetchNetRigidContactForcesKernel(PxVec3* netForces,
                                                        const PxGpuContactPair* contactPairs,
                                                        PxU32 numContactPairs,
                                                        RigidContactReferentTables referents,
                                                        float timeStepInv)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[i];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(cp.nodeIndex0, referents);
        PxU32 refIdx1 = getRigidContactReferentIndex(cp.nodeIndex1, referents);

        // printf("~!~!~! refIdx0: %u\n", refIdx0);
        // printf("~!~!~! refIdx1: %u\n", refIdx1);

        // check if we are interested in either of these objects
        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            const PxVec3 force = component == ForceComponent::eFriction ?
                                     getRigidContactPairFrictionForce(cp, timeStepInv) :
                                     getRigidContactPairNormalForce(cp, timeStepInv);

            if (refIdx0 != INVALID_IDX)
            {
                // race conditions should be rare, so atomicAdd won't block much
                atomicAdd(&netForces[refIdx0].x, force.x);
                atomicAdd(&netForces[refIdx0].y, force.y);
                atomicAdd(&netForces[refIdx0].z, force.z);
            }

            if (refIdx1 != INVALID_IDX)
            {
                // race conditions should be rare, so atomicAdd won't block much
                atomicAdd(&netForces[refIdx1].x, -force.x);
                atomicAdd(&netForces[refIdx1].y, -force.y);
                atomicAdd(&netForces[refIdx1].z, -force.z);
            }
        }
    }
}

bool fetchNetRigidContactForces(PxVec3* netForces,
                                const PxGpuContactPair* contactPairs,
                                PxU32 numContactPairs,
                                const RigidContactReferentTables& referents,
                                float timeStepInv,
                                ForceComponent component)
{
    if (numContactPairs > 0)
    {
        if (component == ForceComponent::eFriction)
            fetchNetRigidContactForcesKernel<ForceComponent::eFriction><<<(numContactPairs + 1023) / 1024, 1024>>>(
                netForces, contactPairs, numContactPairs, referents, timeStepInv);
        else
            fetchNetRigidContactForcesKernel<ForceComponent::eNormal><<<(numContactPairs + 1023) / 1024, 1024>>>(
                netForces, contactPairs, numContactPairs, referents, timeStepInv);
    }
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}


__device__ __forceinline__ static PxU32 getRigidContactFilterIndex(const GpuRigidContactFilterIdPair* filterIdPairs,
                                                                   PxU32 numFilters,
                                                                   const PxActor* actor)
{
#if 1
    // binary search
    int lo = 0;
    int hi = int(numFilters) - 1;
    while (hi >= lo)
    {
        int mid = lo + ((hi - lo) >> 1);
        if (filterIdPairs[mid].actor == actor)
        {
            return filterIdPairs[mid].filterIndex;
        }
        else if (filterIdPairs[mid].actor < actor)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid - 1;
        }
    }
#else
    // linear search
    for (PxU32 i = 0; i < numFilters; i++)
    {
        if (filterIdPairs[i].actor == actor)
        {
            return filterIdPairs[i].filterIndex;
        }
    }
#endif

    return INVALID_IDX;
}

__device__ __forceinline__ static uint64_t getActorPathId(const GpuActorPathIdPair* actorPathLookup,
                                                          PxU32 numPairs,
                                                          const PxActor* actor)
{
    if (!actorPathLookup || numPairs == 0 || !actor)
    {
        return 0;
    }
    
    // binary search
    int lo = 0;
    int hi = int(numPairs) - 1;
    while (hi >= lo)
    {
        int mid = lo + ((hi - lo) >> 1);
        if (actorPathLookup[mid].actor == actor)
        {
            return actorPathLookup[mid].pathId;
        }
        else if (actorPathLookup[mid].actor < actor)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid - 1;
        }
    }
    
    return 0;
}

template <ForceComponent component>
__global__ static void fetchRigidContactForceMatrixKernel(PxVec3* forceMatrix,
                                                          const PxGpuContactPair* contactPairs,
                                                          PxU32 numContactPairs,
                                                          PxU32 numFilters,
                                                          RigidContactReferentTables referents,
                                                          float timeStepInv,
                                                          const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[i];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(cp.nodeIndex0, referents);
        PxU32 refIdx1 = getRigidContactReferentIndex(cp.nodeIndex1, referents);

        // check if we are interested in either of these objects
        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            const PxVec3 force = component == ForceComponent::eFriction ?
                                     getRigidContactPairFrictionForce(cp, timeStepInv) :
                                     getRigidContactPairNormalForce(cp, timeStepInv);

            if (refIdx0 != INVALID_IDX && cp.actor1 != nullptr)
            {
                const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx0 * numFilters;
                PxU32 filterIdx0 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor1);
                if (filterIdx0 != INVALID_IDX)
                {
                    // printf("~!~! Sensor %u, filter %u (%f, %f, %f)\n", refIdx0, filterIdx0, force.x, force.y,
                    // force.z);
                    PxVec3* dst = forceMatrix + refIdx0 * numFilters + filterIdx0;
                    // race conditions should be rare, so atomicAdd won't block much
                    atomicAdd(&dst->x, force.x);
                    atomicAdd(&dst->y, force.y);
                    atomicAdd(&dst->z, force.z);
                }
            }

            if (refIdx1 != INVALID_IDX && cp.actor0 != nullptr)
            {
                const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx1 * numFilters;
                PxU32 filterIdx1 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor0);
                if (filterIdx1 != INVALID_IDX)
                {
                    // printf("~!~! Sensor %u, filter %u (%f, %f, %f)\n", refIdx1, filterIdx1, -force.x, -force.y,
                    // -force.z);
                    PxVec3* dst = forceMatrix + refIdx1 * numFilters + filterIdx1;
                    // race conditions should be rare, so atomicAdd won't block much
                    atomicAdd(&dst->x, -force.x);
                    atomicAdd(&dst->y, -force.y);
                    atomicAdd(&dst->z, -force.z);
                }
            }
        }
    }
}

bool fetchRigidContactForceMatrix(PxVec3* forceMatrix,
                                  const PxGpuContactPair* contactPairs,
                                  PxU32 numContactPairs,
                                  PxU32 numFilters,
                                  const RigidContactReferentTables& referents,
                                  float timeStepInv,
                                  const GpuRigidContactFilterIdPair* filterLookup,
                                  ForceComponent component)
{
    if (numContactPairs > 0)
    {
        if (component == ForceComponent::eFriction)
            fetchRigidContactForceMatrixKernel<ForceComponent::eFriction><<<(numContactPairs + 1023) / 1024, 1024>>>(
                forceMatrix, contactPairs, numContactPairs, numFilters, referents, timeStepInv, filterLookup);
        else
            fetchRigidContactForceMatrixKernel<ForceComponent::eNormal><<<(numContactPairs + 1023) / 1024, 1024>>>(
                forceMatrix, contactPairs, numContactPairs, numFilters, referents, timeStepInv, filterLookup);
    }
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}


__device__ __forceinline__ static void getFrictionData(PxVec3* forceBuffer,
                                                        PxVec3* pointBuffer,
                                                        PxU32* countMatrix,
                                                        const PxU32* startIndicesMatrix,
                                                        const PxFrictionPatch& frictionPatch,
                                                        PxU32 maxDataPoints,
                                                        PxU32 matrixIdx,
                                                        float multiplier)
{
    PxU32* count = countMatrix + matrixIdx;
    PxU32 currentCount = atomicAdd(count, frictionPatch.anchorCount);
    PxU32 elementIdx = startIndicesMatrix[matrixIdx] + currentCount;
    for (PxU32 i = 0; i < frictionPatch.anchorCount; i++)
    {
        if (elementIdx + i >= maxDataPoints)
            break;
        forceBuffer[elementIdx + i] = frictionPatch.anchorImpulses[i] * multiplier;
        pointBuffer[elementIdx + i] = frictionPatch.anchorPositions[i];
    }
}

__global__ static void fetchRigidFrictionDataKernel(PxVec3* forceBuffer,
                                                    PxVec3* pointBuffer,
                                                    PxU32* countMatrix,
                                                    const  PxU32* startIndicesMatrix,
                                                    const PxGpuContactPair* contactPairs,
                                                    PxU32 numContactPairs,
                                                    PxU32 numFilters,
                                                    PxU32 maxDataPoints,
                                                    RigidContactReferentTables referents,
                                                    float timeStepInv,
                                                    const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[c];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(cp.nodeIndex0, referents);
        PxU32 refIdx1 = getRigidContactReferentIndex(cp.nodeIndex1, referents);

        // check if we are interested in either of these objects
        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            for (PxU32 i = 0; i < cp.nbPatches; i++)
            {
                const PxFrictionPatch& frictionPatch = reinterpret_cast<const PxFrictionPatch*>(cp.frictionPatches)[i];

                if (refIdx0 != INVALID_IDX && cp.actor1 != nullptr)
                {
                    const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx0 * numFilters;
                    PxU32 filterIdx0 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor1);
                    if (filterIdx0 != INVALID_IDX)
                    {
                        getFrictionData(forceBuffer, pointBuffer, countMatrix, startIndicesMatrix, frictionPatch,
                            maxDataPoints, refIdx0 * numFilters + filterIdx0, timeStepInv);
                    }
                }

                if (refIdx1 != INVALID_IDX && cp.actor0 != nullptr)
                {
                    const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx1 * numFilters;
                    PxU32 filterIdx1 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor0);
                    if (filterIdx1 != INVALID_IDX)
                    {
                        getFrictionData(forceBuffer, pointBuffer, countMatrix, startIndicesMatrix, frictionPatch,
                            maxDataPoints, refIdx1 * numFilters + filterIdx1, -timeStepInv);
                    }
                }
            }
        }
    }
}

bool fetchRigidFrictionData(PxVec3* forceBuffer,
                            PxVec3* pointBuffer,
                            PxU32* countMatrix,
                            PxU32* startIndicesMatrix,
                            const PxGpuContactPair* contactPairs,
                            PxU32 numContactPairs,
                            PxU32 numFilters,
                            PxU32 maxDataPoints,
                            const RigidContactReferentTables& referents,
                            float timeStepInv,
                            const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchRigidFrictionDataKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            forceBuffer, pointBuffer, countMatrix, startIndicesMatrix, contactPairs, numContactPairs, numFilters,
            maxDataPoints, referents, timeStepInv, filterLookup);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchFrictionCountKernel(
    PxU32* countMatrix,
    const PxGpuContactPair* contactPairs,
    PxU32 numContactPairs,
    PxU32 numFilters,
    RigidContactReferentTables referents,
    const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs) {
        const PxGpuContactPair& cp = contactPairs[c];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(cp.nodeIndex0, referents);
        PxU32 refIdx1 = getRigidContactReferentIndex(cp.nodeIndex1, referents);

        // check if we are interested in either of these objects
        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX) {
            for (PxU32 i = 0; i < cp.nbPatches; i++) {
                const PxFrictionPatch& patch = reinterpret_cast<const PxFrictionPatch*>(cp.frictionPatches)[i];
                // one data point per patch for frictional forces
                PxU32 contactCount = patch.anchorCount;
                if (refIdx0 != INVALID_IDX && cp.actor1 != nullptr) {
                    const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx0 * numFilters;
                    PxU32 filterIdx0 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor1);
                    if (filterIdx0 != INVALID_IDX) {
                        PxU32* count = countMatrix + refIdx0 * numFilters + filterIdx0;
                        atomicAdd(count, contactCount);
                    }
                }
                if (refIdx1 != INVALID_IDX && cp.actor0 != nullptr) {
                    const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx1 * numFilters;
                    PxU32 filterIdx1 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor0);
                    if (filterIdx1 != INVALID_IDX) {
                        PxU32* count = countMatrix + refIdx1 * numFilters + filterIdx1;
                        atomicAdd(count, contactCount);
                    }
                }
            }
        }
    }
}

bool fetchFrictionCount(PxU32* countMatrix,
    const PxGpuContactPair* contactPairs,
    PxU32 numContactPairs,
    PxU32 numFilters,
    const RigidContactReferentTables& referents,
    const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchFrictionCountKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            countMatrix, contactPairs, numContactPairs, numFilters, referents, filterLookup);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchRigidContactDataKernel(PxReal* forceBuffer,
                                                   PxVec3* pointBuffer,
                                                   PxVec3* normalBuffer,
                                                   PxReal* separationBuffer,
                                                   PxU32* countMatrix,
                                                   const PxU32* startIndicesMatrix,
                                                   const PxGpuContactPair* contactPairs,
                                                   PxU32 numContactPairs,
                                                   PxU32 numFilters,
                                                   PxU32 maxDataPoints,
                                                   RigidContactReferentTables referents,
                                                   float timeStepInv,
                                                   const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[c];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(cp.nodeIndex0, referents);
        PxU32 refIdx1 = getRigidContactReferentIndex(cp.nodeIndex1, referents);

        // check if we are interested in either of these objects
        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            for (PxU32 i = 0; i < cp.nbPatches; i++)
            {
                const PxContactPatch& patch = reinterpret_cast<const PxContactPatch*>(cp.contactPatches)[i];
                PxVec3 normal = patch.normal;
                for (PxU32 j = patch.startContactIndex; j < patch.startContactIndex + patch.nbContacts; j++)
                {
                    PxReal normalImpulse = cp.contactForces[j];
                    const PxContact& contact = reinterpret_cast<const PxContact*>(cp.contactPoints)[j];
                    if (refIdx0 != INVALID_IDX && cp.actor1 != nullptr)
                    {
                        const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx0 * numFilters;
                        PxU32 filterIdx0 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor1);
                        if (filterIdx0 != INVALID_IDX)
                        {
                            PxU32* count = countMatrix + refIdx0 * numFilters + filterIdx0;
                            PxU32 currentCount = atomicAdd(count, 1);
                            PxU32 elementIdx = startIndicesMatrix[refIdx0 * numFilters + filterIdx0] + currentCount;
                            if (elementIdx < maxDataPoints)
                            {
                                forceBuffer[elementIdx] = normalImpulse * timeStepInv;
                                normalBuffer[elementIdx] = normal;
                                pointBuffer[elementIdx] = contact.contact;
                                separationBuffer[elementIdx] = contact.separation;
                            }
                        }
                    }

                    if (refIdx1 != INVALID_IDX && cp.actor0 != nullptr)
                    {
                        const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx1 * numFilters;
                        PxU32 filterIdx1 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor0);
                        if (filterIdx1 != INVALID_IDX)
                        {
                            PxU32* count = countMatrix + refIdx1 * numFilters + filterIdx1;
                            PxU32 currentCount = atomicAdd(count, 1);
                            PxU32 elementIdx = startIndicesMatrix[refIdx1 * numFilters + filterIdx1] + currentCount;
                            if (elementIdx < maxDataPoints)
                            {
                                forceBuffer[elementIdx] = -normalImpulse * timeStepInv;
                                normalBuffer[elementIdx] = normal;
                                pointBuffer[elementIdx] = contact.contact;
                                separationBuffer[elementIdx] = contact.separation;
                            }
                        }
                    }
                }
            }
        }
    }
}

bool fetchRigidContactData(PxReal* forceBuffer,
                           PxVec3* pointBuffer,
                           PxVec3* normalBuffer,
                           PxReal* separationBuffer,
                           PxU32* countMatrix,
                           PxU32* startIndicesMatrix,
                           const PxGpuContactPair* contactPairs,
                           PxU32 numContactPairs,
                           PxU32 numFilters,
                           PxU32 maxDataPoints,
                           const RigidContactReferentTables& referents,
                           float timeStepInv,
                           const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchRigidContactDataKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            forceBuffer, pointBuffer, normalBuffer, separationBuffer, countMatrix, startIndicesMatrix, contactPairs,
            numContactPairs, numFilters, maxDataPoints, referents, timeStepInv, filterLookup);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}


__global__ static void fetchRigidContactCountKernel(PxU32* countMatrix,
                                                    const PxGpuContactPair* contactPairs,
                                                    PxU32 numContactPairs,
                                                    PxU32 numFilters,
                                                    RigidContactReferentTables referents,
                                                    const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[c];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(cp.nodeIndex0, referents);
        PxU32 refIdx1 = getRigidContactReferentIndex(cp.nodeIndex1, referents);

        // check if we are interested in either of these objects
        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            for (PxU32 i = 0; i < cp.nbPatches; i++)
            {
                const PxContactPatch& patch = reinterpret_cast<const PxContactPatch*>(cp.contactPatches)[i];
                PxVec3 normal = patch.normal;
                // one data point per patch for frictional forces
                PxU32 contactCount = patch.nbContacts;
                if (refIdx0 != INVALID_IDX && cp.actor1 != nullptr)
                {
                    const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx0 * numFilters;
                    PxU32 filterIdx0 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor1);
                    if (filterIdx0 != INVALID_IDX)
                    {
                        PxU32* count = countMatrix + refIdx0 * numFilters + filterIdx0;
                        atomicAdd(count, contactCount);
                    }
                }
                if (refIdx1 != INVALID_IDX && cp.actor0 != nullptr)
                {
                    const GpuRigidContactFilterIdPair* filterIdPairs = filterLookup + refIdx1 * numFilters;
                    PxU32 filterIdx1 = getRigidContactFilterIndex(filterIdPairs, numFilters, cp.actor0);
                    if (filterIdx1 != INVALID_IDX)
                    {
                        PxU32* count = countMatrix + refIdx1 * numFilters + filterIdx1;
                        atomicAdd(count, contactCount);
                    }
                }
            }
        }
    }
}

bool fetchRigidContactCount(PxU32* countMatrix,
                            const PxGpuContactPair* contactPairs,
                            PxU32 numContactPairs,
                            PxU32 numFilters,
                            const RigidContactReferentTables& referents,
                            const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchRigidContactCountKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            countMatrix, contactPairs, numContactPairs, numFilters, referents, filterLookup);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

// Raw contact count kernel - counts all contacts per sensor (no filter matching)
__global__ static void fetchRawRigidContactCountKernel(PxU32* countBuffer,
                                                       const PxGpuContactPair* contactPairs,
                                                       PxU32 numContactPairs,
                                                       RigidContactReferentTables referents)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[c];

        // Get sensor indices for both bodies
        PxU32 refIdx0 = getRigidContactReferentIndex(cp.nodeIndex0, referents);
        PxU32 refIdx1 = getRigidContactReferentIndex(cp.nodeIndex1, referents);

        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            for (PxU32 i = 0; i < cp.nbPatches; i++)
            {
                const PxContactPatch& patch = reinterpret_cast<const PxContactPatch*>(cp.contactPatches)[i];
                PxU32 contactCount = patch.nbContacts;

                if (refIdx0 != INVALID_IDX && cp.actor1 != nullptr)
                {
                    atomicAdd(&countBuffer[refIdx0], contactCount);
                }
                if (refIdx1 != INVALID_IDX && cp.actor0 != nullptr)
                {
                    atomicAdd(&countBuffer[refIdx1], contactCount);
                }
            }
        }
    }
}

bool fetchRawRigidContactCount(PxU32* countBuffer,
                               const PxGpuContactPair* contactPairs,
                               PxU32 numContactPairs,
                               const RigidContactReferentTables& referents)
{
    if (numContactPairs > 0)
        fetchRawRigidContactCountKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            countBuffer, contactPairs, numContactPairs, referents);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

// Raw contact data kernel - fills contact data for sensors (no filter matching)
__global__ static void fetchRawRigidContactDataKernel(PxReal* forceBuffer,
                                                      PxVec3* pointBuffer,
                                                      PxVec3* normalBuffer,
                                                      PxReal* separationBuffer,
                                                      uint64_t* actorIdBuffer,
                                                      PxU32* countBuffer,
                                                      PxU32* startIndicesBuffer,
                                                      const PxGpuContactPair* contactPairs,
                                                      PxU32 numContactPairs,
                                                      PxU32 numDataPoints,
                                                      RigidContactReferentTables referents,
                                                      float timeStepInv,
                                                      const GpuActorPathIdPair* actorPathLookup,
                                                      PxU32 numActorPathPairs)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[c];

        PxU32 refIdx0 = getRigidContactReferentIndex(cp.nodeIndex0, referents);
        PxU32 refIdx1 = getRigidContactReferentIndex(cp.nodeIndex1, referents);

        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            for (PxU32 i = 0; i < cp.nbPatches; i++)
            {
                const PxContactPatch& patch = reinterpret_cast<const PxContactPatch*>(cp.contactPatches)[i];
                PxVec3 normal = patch.normal;

                for (PxU32 j = patch.startContactIndex; j < patch.startContactIndex + patch.nbContacts; j++)
                {
                    PxReal normalImpulse = cp.contactForces[j];
                    const PxContact& contact = reinterpret_cast<const PxContact*>(cp.contactPoints)[j];

                    if (refIdx0 != INVALID_IDX && cp.actor1 != nullptr)
                    {
                        PxU32 elementIdx = startIndicesBuffer[refIdx0] + atomicAdd(&countBuffer[refIdx0], 1);
                        if (elementIdx < numDataPoints)
                        {
                            forceBuffer[elementIdx] = normalImpulse * timeStepInv;
                            pointBuffer[elementIdx] = contact.contact;
                            normalBuffer[elementIdx] = normal;
                            separationBuffer[elementIdx] = contact.separation;
                            actorIdBuffer[elementIdx * 2 + 0] =
                                getActorPathId(actorPathLookup, numActorPathPairs, cp.actor0);
                            actorIdBuffer[elementIdx * 2 + 1] =
                                getActorPathId(actorPathLookup, numActorPathPairs, cp.actor1);
                        }
                    }
                    // Force sign inverted when sensor is body1
                    if (refIdx1 != INVALID_IDX && cp.actor0 != nullptr)
                    {
                        PxU32 elementIdx = startIndicesBuffer[refIdx1] + atomicAdd(&countBuffer[refIdx1], 1);
                        if (elementIdx < numDataPoints)
                        {
                            forceBuffer[elementIdx] = -normalImpulse * timeStepInv;
                            pointBuffer[elementIdx] = contact.contact;
                            normalBuffer[elementIdx] = normal;
                            separationBuffer[elementIdx] = contact.separation;
                            actorIdBuffer[elementIdx * 2 + 0] =
                                getActorPathId(actorPathLookup, numActorPathPairs, cp.actor1);
                            actorIdBuffer[elementIdx * 2 + 1] =
                                getActorPathId(actorPathLookup, numActorPathPairs, cp.actor0);
                        }
                    }
                }
            }
        }
    }
}

bool fetchRawRigidContactData(PxReal* forceBuffer,
                              PxVec3* pointBuffer,
                              PxVec3* normalBuffer,
                              PxReal* separationBuffer,
                              uint64_t* actorIdBuffer,
                              PxU32* countBuffer,
                              PxU32* startIndicesBuffer,
                              const PxGpuContactPair* contactPairs,
                              PxU32 numContactPairs,
                              PxU32 numDataPoints,
                              const RigidContactReferentTables& referents,
                              float timeStepInv,
                              const GpuActorPathIdPair* actorPathLookup,
                              PxU32 numActorPathPairs)
{
    if (numContactPairs > 0)
        fetchRawRigidContactDataKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            forceBuffer, pointBuffer, normalBuffer, separationBuffer, actorIdBuffer,
            countBuffer, startIndicesBuffer, contactPairs, numContactPairs, numDataPoints, referents, timeStepInv,
            actorPathLookup, numActorPathPairs);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

// Clamp per-sensor counts and start indices so that no entry references a
// position past the buffer cap.  fetchRawRigidContactData increments
// countBuffer[i] atomically before the
// bounds check, so after a kernel run countBuffer[i] may equal the first-pass
// count even when only a smaller prefix was actually written.  This kernel
// corrects both arrays so that start[i] + count[i] <= cap for all i.
__global__ static void clampContactLayoutKernel(
    PxU32* counts, PxU32* startIndices, PxU32 numSensors, PxU32 cap)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numSensors)
        clampContactLayoutEntry(counts, startIndices, i, cap);
}

// Interleave contiguous per-sensor counts/starts into a caller-facing (numSensors, 2)
// tensor. Kept as a separate step so exclusiveScan and the count/fill kernels above keep
// operating on plain arrays rather than growing a stride parameter.
__global__ static void packSensorLayoutKernel(
    PxU32* dstLayout, const PxU32* counts, const PxU32* startIndices, PxU32 numSensors)
{
    const PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numSensors)
        return;
    dstLayout[i * 2 + 0] = counts[i];
    dstLayout[i * 2 + 1] = startIndices[i];
}

bool packSensorLayout(PxU32* dstLayout, const PxU32* counts, const PxU32* startIndices, PxU32 numSensors)
{
    if (numSensors > 0)
        packSensorLayoutKernel<<<(numSensors + 255) / 256, 256>>>(dstLayout, counts, startIndices, numSensors);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

bool clampContactLayout(PxU32* counts, PxU32* startIndices, PxU32 numSensors, PxU32 cap)
{
    if (numSensors > 0)
        clampContactLayoutKernel<<<(numSensors + 255) / 256, 256>>>(counts, startIndices, numSensors, cap);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}


__global__ static void submitSdfQueryPointsKernel(PxVec4* dst,
                                                  const PxVec3* src,
                                                  const GpuSdfShapeRecord* sdfRecords,
                                                  const PxU32 maxPointsPerShape)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 sdfIndex = blockIdx.y;
    GpuSdfShapeRecord shape = sdfRecords[sdfIndex];

    if (i < shape.numSamplePoints)
    {
        PxU32 startIndex = sdfIndex * maxPointsPerShape;
        PxVec3 point = src[startIndex + i];
        dst[startIndex + i] = PxVec4(point.x, point.y, point.z, 0.0f);
    }
}

bool submitSdfQueryPoints(PxVec4* dst,
                          const PxVec3* src,
                          const GpuSdfShapeRecord* sdfRecords,
                          const PxU32 numIndices,
                          const PxU32 maxPointsPerShape)
{
    dim3 gridDim = dim3{ (maxPointsPerShape + 1023) / 1024, numIndices, 1 };
    submitSdfQueryPointsKernel<<<gridDim, 1024>>>(dst, src, sdfRecords, maxPointsPerShape);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}


//
// Deformable bodies
//

__global__ static void fetchDeformableBodyUInt4DataKernel(PxU32* dst,
                                                          const GpuDeformableBodyRecord* bodyRecords,
                                                          const DeformableBodyData::Enum dataFlag,
                                                          const PxU32 dstMaxElementsPerBody)
{
    PxU32 idx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 bodyIndex = blockIdx.y;
    GpuDeformableBodyRecord bodyRecord = bodyRecords[bodyIndex];
    PxU32* src = nullptr;
    PxU32 count = 0;
    switch (dataFlag)
    {
    case DeformableBodyData::eSimElementIndices:
        src = bodyRecord.simElementIndices;
        count = bodyRecord.numSimElements;
        break;
    case DeformableBodyData::eCollElementIndices:
        src = bodyRecord.collElementIndices;
        count = bodyRecord.numCollElements;
        break;
    }

    if (!src)
        return;

    if (idx < count)
    {
        PxU32 startIndex = bodyIndex * dstMaxElementsPerBody * 4;
        dst[startIndex + 4 * idx + 0] = src[4 * idx + 0];
        dst[startIndex + 4 * idx + 1] = src[4 * idx + 1];
        dst[startIndex + 4 * idx + 2] = src[4 * idx + 2];
        dst[startIndex + 4 * idx + 3] = src[4 * idx + 3];
    }
}

bool fetchDeformableBodyUInt4Data(PxU32* dst,
                                  const GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const PxU32 numBodies,
                                  const PxU32 dstMaxElementsPerBody)
{
    dim3 gridDim = dim3{ (dstMaxElementsPerBody + 1023) / 1024, numBodies, 1 };
    fetchDeformableBodyUInt4DataKernel<<<gridDim, 1024>>>(dst, bodyRecords, dataFlag, dstMaxElementsPerBody);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchDeformableBodyUInt3DataKernel(PxU32* dst,
                                                          const GpuDeformableBodyRecord* bodyRecords,
                                                          const DeformableBodyData::Enum dataFlag,
                                                          const PxU32 dstMaxElementsPerBody)
{
    PxU32 idx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 bodyIndex = blockIdx.y;
    GpuDeformableBodyRecord bodyRecord = bodyRecords[bodyIndex];
    PxU32* src = nullptr;
    PxU32 count = 0;
    switch (dataFlag)
    {
    case DeformableBodyData::eSimElementIndices:
        src = bodyRecord.simElementIndices;
        count = bodyRecord.numSimElements;
        break;
    case DeformableBodyData::eCollElementIndices:
        src = bodyRecord.collElementIndices;
        count = bodyRecord.numCollElements;
        break;
    }

    if (!src)
        return;

    if (idx < count)
    {
        PxU32 startIndex = bodyIndex * dstMaxElementsPerBody * 3;
        dst[startIndex + 3 * idx + 0] = src[3 * idx + 0];
        dst[startIndex + 3 * idx + 1] = src[3 * idx + 1];
        dst[startIndex + 3 * idx + 2] = src[3 * idx + 2];
    }
}

bool fetchDeformableBodyUInt3Data(PxU32* dst,
                                  const GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const PxU32 numBodies,
                                  const PxU32 dstMaxElementsPerBody)
{
    dim3 gridDim = dim3{ (dstMaxElementsPerBody + 1023) / 1024, numBodies, 1 };
    fetchDeformableBodyUInt3DataKernel<<<gridDim, 1024>>>(dst, bodyRecords, dataFlag, dstMaxElementsPerBody);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchDeformableBodyVec3DataKernel(PxVec3* dst,
                                                         const GpuDeformableBodyRecord* bodyRecords,
                                                         const DeformableBodyData::Enum dataFlag,
                                                         const PxU32 dstMaxElementsPerBody)
{
    PxU32 idx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 bodyIndex = blockIdx.y;
    GpuDeformableBodyRecord bodyRecord = bodyRecords[bodyIndex];

    PxReal* src = nullptr;
    PxU32 srcStride = 4;
    PxU32 count = 0;
    PxVec3 offset(0.0f);
    switch (dataFlag)
    {
    case DeformableBodyData::eSimNodalPosition:
        src = &bodyRecord.simNodalPositions->x;
        count = bodyRecord.numSimNodes;
        offset = bodyRecord.origin;
        break;
    case DeformableBodyData::eSimNodalVelocity:
        src = &bodyRecord.simNodalVelocities->x;
        count = bodyRecord.numSimNodes;
        break;
    case DeformableBodyData::eRestNodalPosition:
        src = &bodyRecord.restNodalPositions->x;
        srcStride = 3;
        count = bodyRecord.numRestNodes;
        break;
    case DeformableBodyData::eCollNodalPosition:
        src = &bodyRecord.collNodalPositions->x;
        count = bodyRecord.numCollNodes;
        offset = bodyRecord.origin;
    }

    if (!src)
        return;

    if (idx < count)
    {
        const PxReal* srcFlt = (src + idx * srcStride);
        const PxVec3* srcVec = reinterpret_cast<const PxVec3*>(srcFlt);
        PxU32 startIndex = bodyIndex * dstMaxElementsPerBody;
        dst[startIndex + idx] = *srcVec - offset;
    }
}

bool fetchDeformableBodyVec3Data(PxVec3* dst,
                                 const GpuDeformableBodyRecord* bodyRecords,
                                 const DeformableBodyData::Enum dataFlag,
                                 const PxU32 numBodies,
                                 const PxU32 dstMaxElementsPerBody)
{
    dim3 gridDim = dim3{ (dstMaxElementsPerBody + 1023) / 1024, numBodies, 1 };
    fetchDeformableBodyVec3DataKernel<<<gridDim, 1024>>>(dst, bodyRecords, dataFlag, dstMaxElementsPerBody);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void fetchDeformableBodyVec4DataKernel(PxVec4* dst,
                                                         const GpuDeformableBodyRecord* bodyRecords,
                                                         const DeformableBodyData::Enum dataFlag,
                                                         const PxU32 dstMaxElementsPerBody)
{
    PxU32 idx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 bodyIndex = blockIdx.y;
    GpuDeformableBodyRecord bodyRecord = bodyRecords[bodyIndex];
    PxVec4* src = nullptr;
    PxU32 count = 0;
    PxVec3 offset(0.0f);
    switch (dataFlag)
    {
    case DeformableBodyData::eSimNodalKinematicTarget:
        src = bodyRecord.simNodalKinematicTargets;
        count = bodyRecord.numSimNodes;
        offset = bodyRecord.origin;
        break;
    }

    if (!src)
        return;

    if (idx < count)
    {
        PxU32 startIndex = bodyIndex * dstMaxElementsPerBody;
        dst[startIndex + idx] = PxVec4(src[idx].getXYZ() - offset, src[idx].w);
    }
}

bool fetchDeformableBodyVec4Data(PxVec4* dst,
                                 const GpuDeformableBodyRecord* bodyRecords,
                                 const DeformableBodyData::Enum dataFlag,
                                 const PxU32 numBodies,
                                 const PxU32 dstMaxElementsPerBody)
{
    dim3 gridDim = dim3{ (dstMaxElementsPerBody + 1023) / 1024, numBodies, 1 };
    fetchDeformableBodyVec4DataKernel<<<gridDim, 1024>>>(dst, bodyRecords, dataFlag, dstMaxElementsPerBody);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitDeformableBodyVec3DataKernel(const PxVec3* src,
                                                          const PxU32* indices,
                                                          GpuDeformableBodyRecord* bodyRecords,
                                                          const DeformableBodyData::Enum dataFlag,
                                                          const PxU32 srcMaxElementsPerBody)
{
    PxU32 idx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 bodyIndex = indices[blockIdx.y];
    GpuDeformableBodyRecord bodyRecord = bodyRecords[bodyIndex];

    PxVec4* dst = nullptr;
    PxU32 count = 0;
    PxVec3 offset(0.0f);
    switch (dataFlag)
    {
    case DeformableBodyData::eSimNodalPosition:
        dst = bodyRecord.simNodalPositions;
        count = bodyRecord.numSimNodes;
        offset = bodyRecord.origin;
        break;
    case DeformableBodyData::eSimNodalVelocity:
        dst = bodyRecord.simNodalVelocities;
        count = bodyRecord.numSimNodes;
        break;
    case DeformableBodyData::eCollNodalPosition:
        dst = bodyRecord.collNodalPositions;
        count = bodyRecord.numCollNodes;
        offset = bodyRecord.origin;
        break;
    }

    if (dst && idx < count)
    {
        PxU32 startIndex = bodyIndex * srcMaxElementsPerBody;
        PxVec3 newVal = src[startIndex + idx] + offset;
        dst[idx] = PxVec4(newVal, dst[idx].w);
    }
}

bool submitDeformableBodyVec3Data(const PxVec3* src,
                                  const PxU32* indices,
                                  GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const PxU32 numBodies,
                                  const PxU32 srcMaxElementsPerBody)
{
    dim3 gridDim = dim3{ (srcMaxElementsPerBody + 1023) / 1024, numBodies, 1 };
    submitDeformableBodyVec3DataKernel<<<gridDim, 1024>>>(src, indices, bodyRecords, dataFlag, srcMaxElementsPerBody);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

__global__ static void submitDeformableBodyVec4DataKernel(const PxVec4* src,
                                                          const PxU32* indices,
                                                          GpuDeformableBodyRecord* bodyRecords,
                                                          const DeformableBodyData::Enum dataFlag,
                                                          const PxU32 srcMaxElementsPerBody)
{
    PxU32 idx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 bodyIndex = indices[blockIdx.y];
    GpuDeformableBodyRecord bodyRecord = bodyRecords[bodyIndex];

    PxVec4* dst = nullptr;
    PxU32 count = 0;
    PxVec3 offset(0.0f);
    switch (dataFlag)
    {
    case DeformableBodyData::eSimNodalKinematicTarget:
        dst = bodyRecord.simNodalKinematicTargets;
        count = bodyRecord.numSimNodes;
        offset = bodyRecord.origin;
        break;
    }

    if (dst && idx < count)
    {
        PxU32 startIndex = bodyIndex * srcMaxElementsPerBody;
        PxVec3 newVal = src[startIndex + idx].getXYZ() + offset;
        dst[idx] = PxVec4(newVal, src[startIndex + idx].w);
    }
}

bool submitDeformableBodyVec4Data(const PxVec4* src,
                                  const PxU32* indices,
                                  GpuDeformableBodyRecord* bodyRecords,
                                  const DeformableBodyData::Enum dataFlag,
                                  const PxU32 numBodies,
                                  const PxU32 srcMaxElementsPerBody)
{
    dim3 gridDim = dim3{ (srcMaxElementsPerBody + 1023) / 1024, numBodies, 1 };
    submitDeformableBodyVec4DataKernel<<<gridDim, 1024>>>(src, indices, bodyRecords, dataFlag, srcMaxElementsPerBody);
    return launchOk(CHECK_CUDA(cudaGetLastError()));
}

//
// mask -> indices compaction
//
// NOTE: This runs on the CUDA stream associated with SingleAllocPolicy (currently stream 0).
// If we ever move TensorAPI GPU ops to multi-stream execution, this should be plumbed through
// explicitly rather than relying on the default stream.
//
namespace
{
class IsMaskNonZero
{
public:
    __host__ __device__ __forceinline__ bool operator()(const uint8_t& value) const
    {
        return value != 0;
    }
};
} // namespace

bool compactMaskToIndices(SingleAllocPolicy& policy,
                          PxU32* indicesOut,
                          const uint8_t* maskDev,
                          PxU32 N,
                          PxU32& outK)
{
    thrust::device_ptr<const uint8_t> maskPtr(maskDev);
    thrust::device_ptr<PxU32> indicesOutPtr(indicesOut);
    thrust::device_ptr<PxU32> indicesEnd = thrust::copy_if(
        policy,
        thrust::make_counting_iterator<PxU32>(0),
        thrust::make_counting_iterator<PxU32>(N),
        maskPtr,
        indicesOutPtr,
        IsMaskNonZero());
    if (!CHECK_CUDA(cudaGetLastError()))
        return false;
    outK = PxU32(indicesEnd - indicesOutPtr);
    return true;
}

} // namespace tensors
} // namespace physx
} // namespace omni
