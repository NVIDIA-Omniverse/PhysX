// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "CudaKernels.h"
#include "GpuSimulationData.h"
#include "ThrustUtils.h"

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
// Particle Cloth dirty indices
//

namespace
{
class IsPcDirtyFlagSet
{
public:
    explicit IsPcDirtyFlagSet()
    {
    }

    __host__ __device__ __forceinline__ bool operator()(const PxParticleBufferFlags& value) const
    {
        return (uint32_t)value != 0;
    }
};
} // namespace

PxU32 getPcDirtyIndices(SingleAllocPolicy& policy,
                        PxU32* indicesRet,
                        const PxParticleBufferFlags* pcDirtyFlags,
                        const PxU32 numParticleCloths)
{
    thrust::device_ptr<const PxParticleBufferFlags> dirtyFlagsPtr(pcDirtyFlags);
    thrust::device_ptr<PxU32> dirtyIndicesPtr(indicesRet);
    thrust::device_ptr<PxU32> dirtyIndicesEnd = thrust::copy_if(
        policy, thrust::make_counting_iterator<uint32_t>(0),
        thrust::make_counting_iterator<uint32_t>(numParticleCloths), dirtyFlagsPtr, dirtyIndicesPtr, IsPcDirtyFlagSet());
    PxU32 numDirtyIndices = PxU32(dirtyIndicesEnd - dirtyIndicesPtr);
    return numDirtyIndices;
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchArtiMassMatricesKernel(float* dst, const float* src, const PxU32 numElements, const PxU32 massMatrixSize,
                                                 const PxU32 simMassMatrixSize)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numElements)
    {
        PxU32 srcDofIdx = i % massMatrixSize;
        PxU32 artiIndex = i / massMatrixSize;
        PxU32 srcIdx = artiIndex * simMassMatrixSize + srcDofIdx;
        dst[i] = src[srcIdx];
    }
}

bool fetchArtiMassMatrices(float* dst, const float* src, const PxU32 numElements, const PxU32 massMatrixSize, const PxU32 simMassMatrixSize)
{
    fetchArtiMassMatricesKernel<<<(numElements + 1023) / 1024, 1024>>>(dst, src, numElements, massMatrixSize, simMassMatrixSize);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchArtiDofAttributeGravityAndCoriolisKernel(float* dst,
                                                                     const float* src,
                                                                     const PxU32 numDofs,
                                                                     const PxU32 maxDofs,
                                                                     const PxU32 simMaxDofs,
                                                                     const GpuArticulationDofRecord* dofRecords,
                                                                     const bool rootDofs)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numDofs)
    {
        PxU32 srcDofIdx = i % maxDofs;
        PxU32 artiIndex = i / maxDofs;
        PxU32 srcIdx = artiIndex * simMaxDofs + srcDofIdx;
        if (rootDofs)
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

bool fetchArtiDofAttributeGravityAndCoriolis(float* dst,
                                             const float* src,
                                             const PxU32 numDofs,
                                             const PxU32 maxDofs,
                                             const PxU32 simMaxDofs,
                                             const GpuArticulationDofRecord* dofRecords,
                                             const bool rootDofs)
{
    fetchArtiDofAttributeGravityAndCoriolisKernel<<<(numDofs + 1023) / 1024, 1024>>>(dst, src, numDofs, maxDofs, simMaxDofs, dofRecords, rootDofs);
    return CHECK_CUDA(cudaGetLastError());
}


__global__ static void fetchArtiCentroidalMomentumMatricesKernel(float *dst, const float *src, const PxU32 numElem,
                                                                 const PxU32 maxDofs, const PxU32 simMaxDofs,
                                                                 const PxU32 cenMomBlockSize,
                                                                 const PxU32 simCenMomBlockSize,
                                                                 const PxU32 startSimBiasForceBlock)
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
            dst[i] = src[srcIdx];
        }
        else if (colIdx == 6 + maxDofs){
            // SDK bias forces are aligned at the end of the data matrix block
            PxU32 srcIdx = startSimBiasForceBlock + artiIndex * 6 + rowIdx;
            dst[i] = src[srcIdx];
        }
    }
}

bool fetchArtiCentroidalMomentumMatrices(float *dst, const float *src, const PxU32 numElem, const PxU32 maxDofs,
                                         const PxU32 simMaxDofs, const PxU32 cenMomBlockSize,
                                         const PxU32 simCenMomBlockSize, const PxU32 startSimBiasForceBlock)
{
    // Note:
    // cenMomBlockSize =  (maxDofs + 7) * 6;
    // simCenMomBlockSize =  (simMaxDofs + 6) * 6;
    // numElem = numArti * cenMomBlockSize;
    fetchArtiCentroidalMomentumMatricesKernel<<<(numElem + 1023) / 1024, 1024>>>(
        dst, src, numElem, maxDofs, simMaxDofs, cenMomBlockSize, simCenMomBlockSize, startSimBiasForceBlock);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchArtiJacobianKernel(float *dst, const float *src, const PxU32 numElements,
                                               const PxU32 jacobianSize, const PxU32 simJacobianSize)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numElements)
    {
        PxU32 elem = i % (jacobianSize);
        PxU32 artiIndex = i / jacobianSize;
        PxU32 srcIdx = artiIndex * simJacobianSize + elem;
        dst[i] = src[srcIdx];
    }
}

bool fetchArtiJacobian(float *dst, const float *src, const PxU32 numElements, const PxU32 jacobianSize,
                       const PxU32 simJacobianSize)
{
    fetchArtiJacobianKernel<<<(numElements + 1023) / 1024, 1024>>>(dst, src, numElements, jacobianSize,
                                                                   simJacobianSize);
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
                                     const PxU32 simMaxLinks, const GpuArticulationLinkRecord *linkRecords)
{
    fetchArtiLinkIncomingJointForceKernel<<<(numLinks + 1023) / 1024, 1024>>>(dst, src, linkTransforms, numLinks,
                                                                              maxLinks, simMaxLinks, linkRecords);
    return CHECK_CUDA(cudaGetLastError());
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
                             const GpuArticulationLinkRecord *linkRecords)
{
    fetchDofProjectionForceKernel<<<(numLinks + 1023) / 1024, 1024>>>(dst, src, linkTransforms, numLinks, maxLinks,
                                                                      simMaxLinks, linkRecords);
    return CHECK_CUDA(cudaGetLastError());
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
        else
        {
            PxU32 tensorIdx = rbRecords[rbIdx].tensorArtiIdx * simMaxLinks + rbRecords[rbIdx].linkIdx;
            if (tensorIdx != 0xffffffff)
            {
                // printf("Link %u -> RB %u %f\n", tensorIdx, rbIdx, linkTransforms[tensorIdx].p.x);
                dst[rbIdx].p = linkTransforms[tensorIdx].p - origin;
                dst[rbIdx].q = linkTransforms[tensorIdx].q;
            }
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchRbVelAccKernel(TensorVelAcc *dst, const PxVec3 *actorLinVel, const PxVec3 *actorAngVel,
                                           const PxVec3 *linkLinVel, const PxVec3 *linkAngVel, const PxU32 numRbs,
                                           const PxU32 simMaxLinks, const GpuRigidBodyRecord *rbRecords)
{
    PxU32 rbIdx = blockIdx.x * blockDim.x + threadIdx.x;
    if (rbIdx < numRbs)
    {
        PxU32 tensorIdx = rbRecords[rbIdx].tensorRdIdx;
        if (tensorIdx != 0xffffffff)
        {
            dst[rbIdx].linear = actorLinVel[tensorIdx];
            dst[rbIdx].angular = actorAngVel[tensorIdx];
        }
        else
        {
            PxU32 tensorIdx = rbRecords[rbIdx].tensorArtiIdx * simMaxLinks + rbRecords[rbIdx].linkIdx;
            if (tensorIdx != 0xffffffff)
            {
                dst[rbIdx].linear = linkLinVel[tensorIdx];
                dst[rbIdx].angular = linkAngVel[tensorIdx];
            }
        }
    }
}

bool fetchRbVelAcc(TensorVelAcc *dst, const PxVec3 *actorLinVel, const PxVec3 *actorAngVel, const PxVec3 *linkLinVel,
                   const PxVec3 *linkAngVel, const PxU32 numRbs, const PxU32 simMaxLinks,
                   const GpuRigidBodyRecord *rbRecords)
{
    fetchRbVelAccKernel<<<(numRbs + 1023) / 1024, 1024>>>(dst, actorLinVel, actorAngVel, linkLinVel, linkAngVel, numRbs,
                                                          simMaxLinks, rbRecords);
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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

        // printf("i = %d, srcArtiIdx=%d, physxLinkIdx=%d, srcLinkIdx=%d\n", i, srcArtiIdx, physxLinkIdx, srcLinkIdx);
        if (linkIndex == 0)
        {
            dirtyArtiGpuIndices[artiIndex] = linkRecords[srcLinkIdx].physxArtiIdx;
        }
        if (physxLinkIdx != 0xffffffff)
        {
            applyForces<false>(dstLinkForces, dstLinkTorques, nullptr, nullptr, nullptr, srcLinkIdx, tensorLinkIdx, 0,
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
}


//
// rigid contacts
//

constexpr static PxU32 INVALID_IDX = 0xffffffff;

__device__ __forceinline__ static PxU32 getRigidContactReferentIndex(const PxNodeIndex& nodeIndex,
                                                                     PxU32 maxLinks,
                                                                     const PxU32* nodeIdx2ArtiGpuIdx,
                                                                     const PxU32* rdContactIndices,
                                                                     const PxU32* linkContactIndices)
{
    if (nodeIndex.isArticulation())
    {
        PxU32 artiIdx = nodeIdx2ArtiGpuIdx[nodeIndex.index()];
        PxU32 linkIdx = artiIdx * maxLinks + nodeIndex.articulationLinkId();
        return linkContactIndices[linkIdx];
    }
    else if (nodeIndex.isValid())
    {
        PxU32 rdIdx = nodeIndex.index();
        return rdContactIndices[rdIdx];
    }

    // TODO: check shape id

    return INVALID_IDX;
}

__device__ __forceinline__ static PxVec3 getRigidContactPairForce(const PxGpuContactPair& cp, float timeStepInv)
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


__global__ static void fetchNetRigidContactForcesKernel(PxVec3* netForces,
                                                        const PxGpuContactPair* contactPairs,
                                                        PxU32 numContactPairs,
                                                        PxU32 maxLinks,
                                                        float timeStepInv,
                                                        const PxU32* nodeIdx2ArtiGpuIdx,
                                                        const PxU32* rdContactIndices,
                                                        const PxU32* linkContactIndices)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[i];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(
            cp.nodeIndex0, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);
        PxU32 refIdx1 = getRigidContactReferentIndex(
            cp.nodeIndex1, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);

        // printf("~!~!~! refIdx0: %u\n", refIdx0);
        // printf("~!~!~! refIdx1: %u\n", refIdx1);

        // check if we are interested in either of these objects
        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            PxVec3 force = getRigidContactPairForce(cp, timeStepInv);

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
                                PxU32 maxLinks,
                                float timeStepInv,
                                const PxU32* nodeIdx2ArtiGpuIdx,
                                const PxU32* rdContactIndices,
                                const PxU32* linkContactIndices)
{
    if (numContactPairs > 0)
        fetchNetRigidContactForcesKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            netForces, contactPairs, numContactPairs, maxLinks, timeStepInv, nodeIdx2ArtiGpuIdx, rdContactIndices,
            linkContactIndices);
    return CHECK_CUDA(cudaGetLastError());
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

__global__ static void fetchRigidContactForceMatrixKernel(PxVec3* forceMatrix,
                                                          const PxGpuContactPair* contactPairs,
                                                          PxU32 numContactPairs,
                                                          PxU32 numFilters,
                                                          PxU32 maxLinks,
                                                          float timeStepInv,
                                                          const PxU32* nodeIdx2ArtiGpuIdx,
                                                          const PxU32* rdContactIndices,
                                                          const PxU32* linkContactIndices,
                                                          const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[i];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(
            cp.nodeIndex0, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);
        PxU32 refIdx1 = getRigidContactReferentIndex(
            cp.nodeIndex1, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);

        // check if we are interested in either of these objects
        if (refIdx0 != INVALID_IDX || refIdx1 != INVALID_IDX)
        {
            PxVec3 force = getRigidContactPairForce(cp, timeStepInv);

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
                                  PxU32 maxLinks,
                                  float timeStepInv,
                                  const PxU32* nodeIdx2ArtiGpuIdx,
                                  const PxU32* rdContactIndices,
                                  const PxU32* linkContactIndices,
                                  const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchRigidContactForceMatrixKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            forceMatrix, contactPairs, numContactPairs, numFilters, maxLinks, timeStepInv, nodeIdx2ArtiGpuIdx,
            rdContactIndices, linkContactIndices, filterLookup);
    return CHECK_CUDA(cudaGetLastError());
}


__device__ __forceinline__ static PxU32 getFrictionData(PxVec3* forceBuffer,
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
    if (elementIdx < maxDataPoints)
    {
        for (PxU32 i = 0; i < frictionPatch.anchorCount; i++)
        {
            forceBuffer[elementIdx + i] = frictionPatch.anchorImpulses[i] * multiplier;
            pointBuffer[elementIdx + i] = frictionPatch.anchorPositions[i];
        }
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
                                                    PxU32 maxLinks,
                                                    float timeStepInv,
                                                    const PxU32* nodeIdx2ArtiGpuIdx,
                                                    const PxU32* rdContactIndices,
                                                    const PxU32* linkContactIndices,
                                                    const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[c];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(
            cp.nodeIndex0, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);
        PxU32 refIdx1 = getRigidContactReferentIndex(
            cp.nodeIndex1, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);

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
                            PxU32 maxLinks,
                            float timeStepInv,
                            const PxU32* nodeIdx2ArtiGpuIdx,
                            const PxU32* rdContactIndices,
                            const PxU32* linkContactIndices,
                            const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchRigidFrictionDataKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            forceBuffer, pointBuffer, countMatrix, startIndicesMatrix, contactPairs, numContactPairs, numFilters,
            maxDataPoints, maxLinks, timeStepInv, nodeIdx2ArtiGpuIdx, rdContactIndices,
            linkContactIndices, filterLookup);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchFrictionCountKernel(
    PxU32* countMatrix,
    const PxGpuContactPair* contactPairs,
    PxU32 numContactPairs,
    PxU32 numFilters,
    PxU32 maxLinks,
    const PxU32* nodeIdx2ArtiGpuIdx,
    const PxU32* rdContactIndices,
    const PxU32* linkContactIndices,
    const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs) {
        const PxGpuContactPair& cp = contactPairs[c];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(
            cp.nodeIndex0, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices,
            linkContactIndices);
        PxU32 refIdx1 = getRigidContactReferentIndex(
            cp.nodeIndex1, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices,
            linkContactIndices);

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
    PxU32 maxLinks,
    const PxU32* nodeIdx2ArtiGpuIdx,
    const PxU32* rdContactIndices,
    const PxU32* linkContactIndices,
    const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchFrictionCountKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            countMatrix, contactPairs, numContactPairs, numFilters, maxLinks,
            nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices, filterLookup);
    return CHECK_CUDA(cudaGetLastError());
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
                                                   PxU32 maxLinks,
                                                   float timeStepInv,
                                                   const PxU32* nodeIdx2ArtiGpuIdx,
                                                   const PxU32* rdContactIndices,
                                                   const PxU32* linkContactIndices,
                                                   const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[c];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(
            cp.nodeIndex0, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);
        PxU32 refIdx1 = getRigidContactReferentIndex(
            cp.nodeIndex1, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);

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
                           PxU32 maxLinks,
                           float timeStepInv,
                           const PxU32* nodeIdx2ArtiGpuIdx,
                           const PxU32* rdContactIndices,
                           const PxU32* linkContactIndices,
                           const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchRigidContactDataKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            forceBuffer, pointBuffer, normalBuffer, separationBuffer, countMatrix, startIndicesMatrix, contactPairs,
            numContactPairs, numFilters, maxDataPoints, maxLinks, timeStepInv, nodeIdx2ArtiGpuIdx, rdContactIndices,
            linkContactIndices, filterLookup);
    return CHECK_CUDA(cudaGetLastError());
}


__global__ static void fetchRigidContactCountKernel(PxU32* countMatrix,
                                                    const PxGpuContactPair* contactPairs,
                                                    PxU32 numContactPairs,
                                                    PxU32 numFilters,
                                                    PxU32 maxLinks,
                                                    const PxU32* nodeIdx2ArtiGpuIdx,
                                                    const PxU32* rdContactIndices,
                                                    const PxU32* linkContactIndices,
                                                    const GpuRigidContactFilterIdPair* filterLookup)
{
    PxU32 c = blockIdx.x * blockDim.x + threadIdx.x;
    if (c < numContactPairs)
    {
        const PxGpuContactPair& cp = contactPairs[c];

        // referent indices
        PxU32 refIdx0 = getRigidContactReferentIndex(
            cp.nodeIndex0, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);
        PxU32 refIdx1 = getRigidContactReferentIndex(
            cp.nodeIndex1, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices, linkContactIndices);

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
                            PxU32 maxLinks,
                            const PxU32* nodeIdx2ArtiGpuIdx,
                            const PxU32* rdContactIndices,
                            const PxU32* linkContactIndices,
                            const GpuRigidContactFilterIdPair* filterLookup)
{
    if (numContactPairs > 0)
        fetchRigidContactCountKernel<<<(numContactPairs + 1023) / 1024, 1024>>>(
            countMatrix, contactPairs, numContactPairs, numFilters, maxLinks, nodeIdx2ArtiGpuIdx, rdContactIndices,
            linkContactIndices, filterLookup);
    return CHECK_CUDA(cudaGetLastError());
}


__global__ static void fetchParticleClothPositionsKernel(PxVec3* dst,
                                                         const GpuParticleClothRecord* pcRecords,
                                                         const PxU32 maxParticlesPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    GpuParticleClothRecord cloth = pcRecords[blockIdx.y];

    if (i < cloth.numParticles)
    {
        const PxVec3& origin = cloth.origin;
        PxVec4 posInvMass = cloth.positions[i];

        PxU32 startIndex = blockIdx.y * maxParticlesPerCloth;
        dst[startIndex + i] = PxVec3(posInvMass.x, posInvMass.y, posInvMass.z) - origin;
    }
}

bool fetchParticleClothPositions(PxVec3* dst,
                                 const GpuParticleClothRecord* pcRecords,
                                 const ::physx::PxU32 numCloths,
                                 const ::physx::PxU32 maxParticlesPerCloth)
{
    dim3 gridDim = dim3{ (maxParticlesPerCloth + 1023) / 1024, numCloths, 1 };
    fetchParticleClothPositionsKernel<<<gridDim, 1024>>>(dst, pcRecords, maxParticlesPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void submitParticleClothPositionsKernel(const PxVec3* src,
                                                          const PxU32* indices,
                                                          PxParticleBufferFlags* clothDirtyFlags,
                                                          const GpuParticleClothRecord* pcRecords,
                                                          const PxU32 maxParticlesPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 clothIndex = indices[blockIdx.y];
    GpuParticleClothRecord cloth = pcRecords[clothIndex];

    if (i < cloth.numParticles)
    {
        const PxVec3& origin = cloth.origin;
        PxU32 startIndex = clothIndex * maxParticlesPerCloth;

        PxVec4 posInvMass = cloth.positions[i];
        PxVec3 newPos = src[startIndex + i];

        posInvMass.x = newPos.x + origin.x;
        posInvMass.y = newPos.y + origin.y;
        posInvMass.z = newPos.z + origin.z;

        cloth.positions[i] = posInvMass;
    }

    if (i == 0)
    {
        // raise the flags in the dirty list.
        PxU32 globalIndex = cloth.globalIndex;
        clothDirtyFlags[globalIndex] |= PxParticleBufferFlag::eUPDATE_POSITION;
    }
}

bool submitParticleClothPositions(const PxVec3* src,
                                  const PxU32* indices,
                                  PxU32 numIndices,
                                  PxParticleBufferFlags* clothDirtyFlags,
                                  const GpuParticleClothRecord* pcRecords,
                                  const PxU32 maxParticlesPerCloth)
{
    dim3 gridDim = dim3{ (maxParticlesPerCloth + 1023) / 1024, numIndices, 1 };
    submitParticleClothPositionsKernel<<<gridDim, 1024>>>(src, indices, clothDirtyFlags, pcRecords, maxParticlesPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchParticleClothVelocitiesKernel(PxVec3* dst,
                                                          const GpuParticleClothRecord* pcRecords,
                                                          const PxU32 maxParticlesPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    GpuParticleClothRecord cloth = pcRecords[blockIdx.y];

    if (i < cloth.numParticles)
    {
        PxVec4 velocity = cloth.velocities[i];

        PxU32 startIndex = blockIdx.y * maxParticlesPerCloth;
        dst[startIndex + i] = PxVec3(velocity.x, velocity.y, velocity.z);
    }
}

bool fetchParticleClothVelocities(PxVec3* dst,
                                  const GpuParticleClothRecord* pcRecords,
                                  const PxU32 numCloths,
                                  const PxU32 maxParticlesPerCloth)
{
    dim3 gridDim = dim3{ (maxParticlesPerCloth + 1023) / 1024, numCloths, 1 };
    fetchParticleClothVelocitiesKernel<<<gridDim, 1024>>>(dst, pcRecords, maxParticlesPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void submitParticleClothVelocitiesKernel(const PxVec3* src,
                                                           const PxU32* indices,
                                                           PxParticleBufferFlags* clothDirtyFlags,
                                                           const GpuParticleClothRecord* pcRecords,
                                                           const PxU32 maxParticlesPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 clothIndex = indices[blockIdx.y];
    GpuParticleClothRecord cloth = pcRecords[clothIndex];

    if (i < cloth.numParticles)
    {
        PxU32 startIndex = clothIndex * maxParticlesPerCloth;

        PxVec3 vel = src[startIndex + i];
        PxVec4 newVelocity = PxVec4(vel.x, vel.y, vel.z, 0.0f);

        cloth.velocities[i] = newVelocity;
    }

    if (i == 0)
    {
        // raise the flags in the dirty list.
        PxU32 globalIndex = cloth.globalIndex;
        clothDirtyFlags[globalIndex] |= PxParticleBufferFlag::eUPDATE_VELOCITY;
    }
}


bool submitParticleClothVelocities(const PxVec3* src,
                                   const PxU32* indices,
                                   const PxU32 numIndices,
                                   PxParticleBufferFlags* clothDirtyFlags,
                                   const GpuParticleClothRecord* pcRecords,
                                   const PxU32 maxParticlesPerCloth)
{
    dim3 gridDim = dim3{ (maxParticlesPerCloth + 1023) / 1024, numIndices, 1 };
    submitParticleClothVelocitiesKernel<<<gridDim, 1024>>>(
        src, indices, clothDirtyFlags, pcRecords, maxParticlesPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchParticleClothMassesKernel(float* dst,
                                                      const GpuParticleClothRecord* pcRecords,
                                                      const PxU32 numCloths,
                                                      const PxU32 maxParticlesPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    GpuParticleClothRecord cloth = pcRecords[blockIdx.y];

    if (i < cloth.numParticles)
    {
        float invMass = cloth.positions[i].w;
        float mass = 1.0f / invMass;

        PxU32 startIndex = blockIdx.y * maxParticlesPerCloth;
        dst[startIndex + i] = mass;
    }
}

bool fetchParticleClothMasses(float* dst,
                              const GpuParticleClothRecord* pcRecords,
                              const PxU32 numCloths,
                              const PxU32 maxParticlesPerCloth)
{
    dim3 gridDim = dim3{ (maxParticlesPerCloth + 1023) / 1024, numCloths, 1 };
    fetchParticleClothMassesKernel<<<gridDim, 1024>>>(dst, pcRecords, numCloths, maxParticlesPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void submitParticleClothMassesKernel(const float* src,
                                                       const PxU32* indices,
                                                       PxParticleBufferFlags* clothDirtyFlags,
                                                       const GpuParticleClothRecord* pcRecords,
                                                       const PxU32 maxParticlesPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 clothIndex = indices[blockIdx.y];
    GpuParticleClothRecord cloth = pcRecords[clothIndex];

    if (i < cloth.numParticles)
    {
        PxU32 startIndex = clothIndex * maxParticlesPerCloth;

        PxVec4 posInvMass = cloth.positions[i];
        float newMass = src[startIndex + i];

        posInvMass.w = newMass > 0.0f ? 1.f / newMass : 0.0f; // non-positive values are clamped to 0.0f
        cloth.positions[i] = posInvMass;
    }

    if (i == 0)
    {
        // raise the flags in the dirty list.
        PxU32 globalIndex = cloth.globalIndex;
        clothDirtyFlags[globalIndex] |= PxParticleBufferFlag::eUPDATE_POSITION;
    }
}

bool submitParticleClothMasses(const float* src,
                               const PxU32* indices,
                               PxU32 numIndices,
                               PxParticleBufferFlags* clothDirtyFlags,
                               const GpuParticleClothRecord* pcRecords,
                               const PxU32 maxParticlesPerCloth)
{
    dim3 gridDim = dim3{ (maxParticlesPerCloth + 1023) / 1024, numIndices, 1 };
    submitParticleClothMassesKernel<<<gridDim, 1024>>>(src, indices, clothDirtyFlags, pcRecords, maxParticlesPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchParticleClothSpringDampingKernel(float* dst,
                                                             const GpuParticleClothRecord* pcRecords,
                                                             const PxU32 maxSpringsPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    GpuParticleClothRecord cloth = pcRecords[blockIdx.y];

    if (i < cloth.numSprings)
    {
        PxParticleSpring spring = cloth.springs[i];

        PxU32 startIndex = blockIdx.y * maxSpringsPerCloth;
        dst[startIndex + i] = spring.damping;
    }
}

bool fetchParticleClothSpringDamping(float* dst,
                                     const GpuParticleClothRecord* pcRecords,
                                     const PxU32 numCloths,
                                     const PxU32 maxSpringsPerCloth)
{
    dim3 gridDim = dim3{ (maxSpringsPerCloth + 1023) / 1024, numCloths, 1 };
    fetchParticleClothSpringDampingKernel<<<gridDim, 1024>>>(dst, pcRecords, maxSpringsPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void submitParticleClothSpringDampingKernel(const float* src,
                                                              const PxU32* indices,
                                                              const GpuParticleClothRecord* pcRecords,
                                                              const PxU32 maxSpringsPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 clothIndex = indices[blockIdx.y];
    GpuParticleClothRecord cloth = pcRecords[clothIndex];

    if (i < cloth.numSprings)
    {
        PxU32 startIndex = clothIndex * maxSpringsPerCloth;

        float newDamping = src[startIndex + i];
        cloth.springs[i].damping = newDamping; // not coalesced, but more of a physx problem?
    }
}

bool submitParticleClothSpringDamping(const float* src,
                                      const PxU32* indices,
                                      PxU32 numIndices,
                                      const GpuParticleClothRecord* pcRecords,
                                      const PxU32 maxSpringsPerCloth)
{
    dim3 gridDim = dim3{ (maxSpringsPerCloth + 1023) / 1024, numIndices, 1 };
    submitParticleClothSpringDampingKernel<<<gridDim, 1024>>>(src, indices, pcRecords, maxSpringsPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchParticleClothSpringStiffnessKernel(float* dst,
                                                               const GpuParticleClothRecord* pcRecords,
                                                               const PxU32 maxSpringsPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    GpuParticleClothRecord cloth = pcRecords[blockIdx.y];

    if (i < cloth.numSprings)
    {
        PxParticleSpring spring = cloth.springs[i];

        PxU32 startIndex = blockIdx.y * maxSpringsPerCloth;
        dst[startIndex + i] = spring.stiffness;
    }
}

bool fetchParticleClothSpringStiffness(float* dst,
                                       const GpuParticleClothRecord* pcRecords,
                                       const PxU32 numCloths,
                                       const PxU32 maxSpringsPerCloth)
{
    dim3 gridDim = dim3{ (maxSpringsPerCloth + 1023) / 1024, numCloths, 1 };
    fetchParticleClothSpringStiffnessKernel<<<gridDim, 1024>>>(dst, pcRecords, maxSpringsPerCloth);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void submitParticleClothSpringStiffnessKernel(const float* src,
                                                                const PxU32* indices,
                                                                const GpuParticleClothRecord* pcRecords,
                                                                const PxU32 maxSpringsPerCloth)
{
    PxU32 i = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 clothIndex = indices[blockIdx.y];
    GpuParticleClothRecord cloth = pcRecords[clothIndex];

    if (i < cloth.numSprings)
    {
        PxU32 startIndex = clothIndex * maxSpringsPerCloth;

        float newStiffness = src[startIndex + i];
        cloth.springs[i].stiffness = newStiffness; // not coalesced, but more of a physx problem?
    }
}

bool submitParticleClothSpringStiffness(const float* src,
                                        const PxU32* indices,
                                        PxU32 numIndices,
                                        const GpuParticleClothRecord* pcRecords,
                                        const PxU32 maxSpringsPerCloth)
{
    dim3 gridDim = dim3{ (maxSpringsPerCloth + 1023) / 1024, numIndices, 1 };
    submitParticleClothSpringStiffnessKernel<<<gridDim, 1024>>>(src, indices, pcRecords, maxSpringsPerCloth);
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
}

//
// Soft bodies
//
__global__ static void fetchSbElementRotationsKernel(PxQuat* dst,
                                                     GpuSoftBodyRecord* mSbRecords,
                                                     const PxU32 maxElementsPerBody,
                                                     bool simulationMesh)
{
    PxU32 elementIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = blockIdx.y;
    GpuSoftBodyRecord softBody = mSbRecords[softBodyIndex];
    PxU32 numElements = simulationMesh ? softBody.numSimElements : softBody.numElements;
    PxQuat* buffer = simulationMesh ? softBody.simElementRotation : softBody.elementRotation;
    if (elementIdx < numElements)
    {
        PxU32 startIndex = softBodyIndex * maxElementsPerBody;
        dst[startIndex + elementIdx] = buffer[elementIdx];
    }
}

bool fetchSbElementRotations(
    PxQuat* dst, GpuSoftBodyRecord* mSbRecords, const PxU32 numSbs, const PxU32 maxElementsPerBody, bool simulationMesh)
{
    dim3 gridDim = dim3{ (maxElementsPerBody + 1023) / 1024, numSbs, 1 };
    fetchSbElementRotationsKernel<<<gridDim, 1024>>>(dst, mSbRecords, maxElementsPerBody, simulationMesh);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchSbElementRestPosesKernel(PxMat33* dst,
                                                      GpuSoftBodyRecord* mSbRecords,
                                                      const PxU32 maxElementsPerBody,
                                                      bool simulationMesh)
{
    PxU32 elementIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = blockIdx.y;
    GpuSoftBodyRecord softBody = mSbRecords[softBodyIndex];
    PxMat33* buffer = simulationMesh ? softBody.simElementRestPose : softBody.elementRestPose;
    PxU32 numElements = simulationMesh ? softBody.numSimElements : softBody.numElements;
    if (elementIdx < numElements)
    {
        PxU32 startIndex = softBodyIndex * maxElementsPerBody;
        dst[startIndex + elementIdx] = buffer[elementIdx];
    }
}

bool fetchSbElementRestPoses(PxMat33* dst, GpuSoftBodyRecord* mSbRecords, const PxU32 numSbs, const PxU32 maxElementsPerBody, bool simulationMesh)
{
    dim3 gridDim = dim3{ (maxElementsPerBody + 127) / 128, numSbs, 1 };
    fetchSbElementRestPosesKernel<<<gridDim, 128>>>(dst, mSbRecords, maxElementsPerBody, simulationMesh);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchSbElementIndicesKernel(PxU32* dst,
                                                   GpuSoftBodyRecord* mSbRecords,
                                                   const PxU32 maxElementsPerBody,
                                                   bool simulationMesh)
{
    PxU32 elementIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = blockIdx.y;
    GpuSoftBodyRecord softBody = mSbRecords[softBodyIndex];
    PxU32 numElements = simulationMesh ? softBody.numSimElements : softBody.numElements;
    PxU32* buffer = simulationMesh ? softBody.simElementIndices : softBody.elementIndices;
    if (elementIdx < numElements)
    {
        PxU32 startIndex = softBodyIndex * maxElementsPerBody * 4;
        dst[startIndex + 4 * elementIdx + 0] = buffer[4 * elementIdx + 0];
        dst[startIndex + 4 * elementIdx + 1] = buffer[4 * elementIdx + 1];
        dst[startIndex + 4 * elementIdx + 2] = buffer[4 * elementIdx + 2];
        dst[startIndex + 4 * elementIdx + 3] = buffer[4 * elementIdx + 3];
    }
}
bool fetchSbElementIndices(
    PxU32* dst, GpuSoftBodyRecord* mSbRecords, const PxU32 numSbs, const PxU32 maxElementsPerBody, bool simulationMesh)
{
    dim3 gridDim = dim3{ (maxElementsPerBody + 1023) / 1024, numSbs, 1 };
    fetchSbElementIndicesKernel<<<gridDim, 1024>>>(dst, mSbRecords, maxElementsPerBody, simulationMesh);
    return CHECK_CUDA(cudaGetLastError());
}


__device__ static PxMat33 computeDeformationGradient(PxMat33& Qinv, PxQuat& Rot, PxU32* indices, PxVec4* nodalPositions)
{
    const PxVec4 p0 = nodalPositions[indices[0]];
    const PxVec4 p1 = nodalPositions[indices[1]];
    const PxVec4 p2 = nodalPositions[indices[2]];
    const PxVec4 p3 = nodalPositions[indices[3]];

    const PxVec3 u1 = (p1 - p0).getXYZ();
    const PxVec3 u2 = (p2 - p0).getXYZ();
    const PxVec3 u3 = (p3 - p0).getXYZ();
    const PxMat33 P = PxMat33(u1, u2, u3);
    // element rest poses are loaded
    PxMat33 F = P * Qinv;
    // removing rotation from F
    PxMat33 R(Rot);
    PxMat33 RTranspose = R.getTranspose();
    return RTranspose * F;
};


__global__ static void computeSbElementDeformationGradientsKernel(PxMat33* dst,
                                                                  GpuSoftBodyRecord* mSbRecords,
                                                                  const PxU32 maxElementsPerBody,
                                                                  bool simulationMesh)
{
    PxU32 elementIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = blockIdx.y;
    GpuSoftBodyRecord softBody = mSbRecords[softBodyIndex];
    PxU32 numElements = simulationMesh ? softBody.numSimElements : softBody.numElements;
    PxQuat* rotationBuffer = simulationMesh ? softBody.simElementRotation : softBody.elementRotation;
    PxU32* indicesBuffer = simulationMesh ? softBody.simElementIndices : softBody.elementIndices;
    PxMat33* restPoseBuffer = simulationMesh ? softBody.simElementRestPose : softBody.elementRestPose;
    PxVec4* positionBuffer = simulationMesh ? softBody.simNodalValues : softBody.nodalPositions;
    if (elementIdx < numElements)
    {
        PxU32 startIndex = softBodyIndex * maxElementsPerBody;
        PxU32* indices = &indicesBuffer[4 * elementIdx];
        PxQuat& rotation = rotationBuffer[elementIdx];
        PxMat33& restPose = restPoseBuffer[elementIdx];
        dst[startIndex + elementIdx] = computeDeformationGradient(restPose, rotation, indices, positionBuffer);
    }
}

bool computeSbElementDeformationGradients(
    PxMat33* dst, GpuSoftBodyRecord* mSbRecords, const PxU32 numSbs, const PxU32 maxElementsPerBody, bool simulationMesh)
{
    dim3 gridDim = dim3{ (maxElementsPerBody + 127) / 128, numSbs, 1 };
    computeSbElementDeformationGradientsKernel<<<gridDim, 128>>>(dst, mSbRecords, maxElementsPerBody, simulationMesh);
    return CHECK_CUDA(cudaGetLastError());
}


__global__ static void computeSbElementStressesKernel(PxMat33* dst,
                                                      GpuSoftBodyRecord* mSbRecords,
                                                      PxVec4* matProps,
                                                      PxFEMSoftBodyMaterialModel::Enum* materialModels,
                                                      const PxU32 maxElementsPerBody,
                                                      bool simulationMesh)
{
    PxU32 elementIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = blockIdx.y;
    GpuSoftBodyRecord& softBody = mSbRecords[softBodyIndex];
    PxU32 numElements = simulationMesh ? softBody.numSimElements : softBody.numElements;
    PxQuat* rotationBuffer = simulationMesh ? softBody.simElementRotation : softBody.elementRotation;
    PxU32* indicesBuffer = simulationMesh ? softBody.simElementIndices : softBody.elementIndices;
    PxMat33* restPoseBuffer = simulationMesh ? softBody.simElementRestPose : softBody.elementRestPose;
    PxVec4* positionBuffer = simulationMesh ? softBody.simNodalValues : softBody.nodalPositions;

    if (elementIdx < numElements)
    {
        PxU32 startIndex = softBodyIndex * maxElementsPerBody;
        PxQuat& rotation = rotationBuffer[elementIdx];
        PxU32* indices = &indicesBuffer[4 * elementIdx];
        PxMat33& restPose = restPoseBuffer[elementIdx];
        PxMat33 F = computeDeformationGradient(restPose, rotation, indices, positionBuffer);

        PxReal mu = matProps[softBodyIndex].x / (1 + matProps[softBodyIndex].y) / 2.0f;
        PxReal lambda = matProps[softBodyIndex].x * matProps[softBodyIndex].y / (1 + matProps[softBodyIndex].y) /
                        (1 - 2 * matProps[softBodyIndex].y);
        PxMat33 I(PxIdentity);
        PxMat33 eps = (F.getTranspose() + F) * 0.5f - I;
        PxReal J = F.getDeterminant();
        PxMat33 P;

        if (materialModels[softBodyIndex] == PxFEMSoftBodyMaterialModel::Enum::eCO_ROTATIONAL)
        {
            PxReal trace = eps.column0.x + eps.column1.y + eps.column2.z;
            P = PxMat33(rotation) * eps * (2.0 * mu) + I * (lambda * trace);
            // dst[startIndex + elementIdx] = eps * (2.0 * mu) + I * (lambda * trace);
        }
        else if (materialModels[softBodyIndex] == PxFEMSoftBodyMaterialModel::Enum::eNEO_HOOKEAN)
        {
            PxMat33 FTInv = F.getTranspose().getInverse();
            P = (F - FTInv * mu) * mu + FTInv * lambda * logf(J);
        }
        dst[startIndex + elementIdx] = 1.0f / J * P * F.getTranspose();
    }
}

bool computeSbElementStresses(PxMat33* dst,
                              GpuSoftBodyRecord* mSbRecords,
                              PxVec4* materialProperties,
                              PxFEMSoftBodyMaterialModel::Enum* materialModels,
                              const PxU32 numSbs,
                              const PxU32 maxElementsPerBody,
                              bool simulationMesh)
{
    dim3 gridDim = dim3{ (maxElementsPerBody + 127) / 128, numSbs, 1 };
    computeSbElementStressesKernel<<<gridDim, 128>>>(
        dst, mSbRecords, materialProperties, materialModels, maxElementsPerBody, simulationMesh);
    return CHECK_CUDA(cudaGetLastError());
}

// fetch the nodal positions w.r.t. to the origin
__global__ static void fetchSbNodalValuesKernel(
    PxVec3* dst, GpuSoftBodyRecord* mSbRecords, const PxU32 maxVerticesPerBody, bool simulationMesh, bool transform)
{
    PxU32 vertexIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = blockIdx.y;
    GpuSoftBodyRecord softBody = mSbRecords[softBodyIndex];
    const PxVec3& origin = transform ? softBody.origin : PxVec3(0.0f);
    PxU32 numVertices = simulationMesh ? softBody.numSimVertices : softBody.numVertices;
    PxVec4* buffer = simulationMesh ? softBody.simNodalValues : softBody.nodalPositions;

    if (vertexIdx < numVertices)
    {
        PxU32 startIndex = softBodyIndex * maxVerticesPerBody;
        dst[startIndex + vertexIdx] = buffer[vertexIdx].getXYZ() - origin;
        // if (softBodyIndex == 0)
        //     printf("posMass[%u]= %f,%f,%f,%f\n", i, posInvMass.x, posInvMass.y, posInvMass.z, posInvMass.w);
    }
}

bool fetchSbNodalValues(PxVec3* dst,
                        GpuSoftBodyRecord* mSbRecords,
                        const PxU32 numSbs,
                        const PxU32 maxVerticesPerBody,
                        bool simulationMesh,
                        bool transform)
{
    dim3 gridDim = dim3{ (maxVerticesPerBody + 1023) / 1024, numSbs, 1 };
    fetchSbNodalValuesKernel<<<gridDim, 1024>>>(
        dst, mSbRecords, maxVerticesPerBody, simulationMesh, transform);
    return CHECK_CUDA(cudaGetLastError());
}

// set nodal value of either simulation or collision meshes
__global__ static void submitSbNodalValuesKernel(const PxVec3* src,
                                                 const PxU32* indices,
                                                 GpuSoftBodyRecord* mSbRecords,
                                                 const PxU32 maxVerticesPerBody,
                                                 bool transform)
{
    PxU32 vertexIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = indices[blockIdx.y];
    GpuSoftBodyRecord softBody = mSbRecords[softBodyIndex];
    const PxVec3& origin = transform ? softBody.origin : PxVec3(0.0f);
    if (vertexIdx < softBody.numSimVertices)
    {
        PxU32 startIndex = softBodyIndex * maxVerticesPerBody;
        PxVec3 newVal = src[startIndex + vertexIdx] + origin;
        // printf("Index=%u, vertexIdx=%u, applying pos=%f,%f,%f,%f, softBody %p\n", softBodyIndex, vertexIdx, newVal.x,
        //        newVal.y, newVal.z, softBody.simNodalValues[vertexIdx].w, softBody.simNodalValues);
        softBody.simNodalValues[vertexIdx] = PxVec4(newVal.x, newVal.y, newVal.z, softBody.simNodalValues[vertexIdx].w);
    }
}

bool submitSbNodalValues(const PxVec3* src,
                         const PxU32* indices,
                         GpuSoftBodyRecord* mSbRecords,
                         const PxU32 numSbs,
                         const PxU32 maxVerticesPerBody,
                         bool transform)
{
    dim3 gridDim = dim3{ (maxVerticesPerBody + 1023) / 1024, numSbs, 1 };
    submitSbNodalValuesKernel<<<gridDim, 1024>>>(src, indices, mSbRecords, maxVerticesPerBody, transform);
    return CHECK_CUDA(cudaGetLastError());
}

__global__ static void fetchSbKinematicTargetsKernel(PxVec4* dst,
                                                     GpuSoftBodyRecord* mSbRecords,
                                                     const PxU32 maxVerticesPerBody,
                                                     bool transform)
{
    PxU32 vertexIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = blockIdx.y;
    GpuSoftBodyRecord softBody = mSbRecords[softBodyIndex];
    const PxVec3& origin = transform ? softBody.origin : PxVec3(0.0f);
    if (vertexIdx < softBody.numSimVertices)
    {
        PxU32 startIndex = softBodyIndex * maxVerticesPerBody;
        PxVec3 value = softBody.simKinematicTargets[vertexIdx].getXYZ() - origin;
        dst[startIndex + vertexIdx] = PxVec4(value.x, value.y, value.z, softBody.simKinematicTargets[vertexIdx].w);
        // if (softBodyIndex == 0)
        //     printf("posMass[%u]= %f,%f,%f,%f\n", i, posInvMass.x, posInvMass.y, posInvMass.z, posInvMass.w);
    }
}

bool fetchSbKinematicTargets(PxVec4* dst,
                             GpuSoftBodyRecord* mSbRecords,
                             const PxU32 numSbs,
                             const PxU32 maxVerticesPerBody,
                             bool transform)
{
    dim3 gridDim = dim3{ (maxVerticesPerBody + 1023) / 1024, numSbs, 1 };
    fetchSbKinematicTargetsKernel<<<gridDim, 1024>>>(dst, mSbRecords, maxVerticesPerBody, transform);
    return CHECK_CUDA(cudaGetLastError());
}

// set nodal value of either simulation or collision meshes
__global__ static void submitSbKinematicTargetsKernel(const PxVec4* src,
                                                      const PxU32* indices,
                                                      GpuSoftBodyRecord* mSbRecords,
                                                      const PxU32 maxVerticesPerBody,
                                                      bool transform)
{
    PxU32 vertexIdx = blockIdx.x * blockDim.x + threadIdx.x;
    PxU32 softBodyIndex = indices[blockIdx.y];
    GpuSoftBodyRecord softBody = mSbRecords[softBodyIndex];
    const PxVec3& origin = transform ? softBody.origin : PxVec3(0.0f);
    if (vertexIdx < softBody.numSimVertices)
    {
        PxU32 startIndex = softBodyIndex * maxVerticesPerBody;
        PxVec3 newVal = src[startIndex + vertexIdx].getXYZ() + origin;
        softBody.simKinematicTargets[vertexIdx] = PxVec4(newVal.x, newVal.y, newVal.z, src[startIndex + vertexIdx].w);
        // if (softBody.simKinematicTargets[vertexIdx].w <= FLT_EPSILON )
        //     printf("R %f,%f,%f\n", softBody.simKinematicTargets[vertexIdx].x,
        //     softBody.simKinematicTargets[vertexIdx].y,
        //            softBody.simKinematicTargets[vertexIdx].z);
    }
}

bool submitSbKinematicTargets(const PxVec4* src,
                              const PxU32* indices,
                              GpuSoftBodyRecord* mSbRecords,
                              const PxU32 numSbs,
                              const PxU32 maxVerticesPerBody,
                              bool transform)
{
    dim3 gridDim = dim3{ (maxVerticesPerBody + 1023) / 1024, numSbs, 1 };
    submitSbKinematicTargetsKernel<<<gridDim, 1024>>>(src, indices, mSbRecords, maxVerticesPerBody, transform);
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
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
    return CHECK_CUDA(cudaGetLastError());
}

} // namespace tensors
} // namespace physx
} // namespace omni
