// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "PxgRadixSortCore.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelWrangler.h"
#include "CudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "PxgKernelIndices.h"
#include "PxgRadixSortKernelIndices.h"
#include "PxParticleSystem.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "DyParticleSystem.h"
#include "PxgKernelLauncher.h"

#define PS_GPU_SPARSE_GRID_CORE_DEBUG 0

using namespace physx;

PxgRadixSortCore::PxgRadixSortCore(PxgEssentialCore* core) :
	mRSDesc(core->mHeapMemoryManager->mMappedMemoryAllocators),
	mRadixSortDescBuf(core->mHeapMemoryManager, PxsHeapStats::eSHARED_PARTICLES),
	mRadixCountTotalBuf(core->mHeapMemoryManager, PxsHeapStats::eSHARED_PARTICLES)
{
	mEssentialCore = core;
}

void PxgRadixSortCore::allocate(PxU32 nbRequired)
{
	mRSDesc.resize(nbRequired * 2u);
	mRadixCountSize = sizeof(PxU32) * PxgRadixSortKernelGridDim::RADIX_SORT * 16;
	mRadixCountTotalBuf.allocate(mRadixCountSize * nbRequired, PX_FL);

	for (PxU32 i = 0; i < 2; ++i)
	{
		mRadixSortDescBuf[i].allocate(sizeof(PxgRadixSortBlockDesc)*nbRequired, PX_FL);
	}
}

void PxgRadixSortCore::updateGPURadixSortDesc(PxCudaContext*mCudaContext, const CUstream& stream, CUdeviceptr inputKeyd, CUdeviceptr inputRankd,
	CUdeviceptr outputKeyd, CUdeviceptr outputRankd, CUdeviceptr radixCountd, PxgRadixSortDesc* rsDescs,
	CUdeviceptr radixSortDescBuf0, CUdeviceptr radixSortDescBuf1, const PxU32 count)
{
	rsDescs[0].inputKeys = reinterpret_cast<PxU32*>(inputKeyd);
	rsDescs[0].inputRanks = reinterpret_cast<PxU32*>(inputRankd);
	rsDescs[0].outputKeys = reinterpret_cast<PxU32*>(outputKeyd);
	rsDescs[0].outputRanks = reinterpret_cast<PxU32*>(outputRankd);
	rsDescs[0].radixBlockCounts = reinterpret_cast<PxU32*>(radixCountd);
	rsDescs[0].count = count;

	rsDescs[1].outputKeys = reinterpret_cast<PxU32*>(inputKeyd);
	rsDescs[1].outputRanks = reinterpret_cast<PxU32*>(inputRankd);
	rsDescs[1].inputKeys = reinterpret_cast<PxU32*>(outputKeyd);
	rsDescs[1].inputRanks = reinterpret_cast<PxU32*>(outputRankd);
	rsDescs[1].radixBlockCounts = reinterpret_cast<PxU32*>(radixCountd);
	rsDescs[1].count = count;

	mCudaContext->memcpyHtoDAsync(radixSortDescBuf0, (void*)&rsDescs[0], sizeof(PxgRadixSortDesc), stream);
	mCudaContext->memcpyHtoDAsync(radixSortDescBuf1, (void*)&rsDescs[1], sizeof(PxgRadixSortDesc), stream);
}

void PxgRadixSortCore::sort(PxgCudaKernelWranglerManager* mGpuKernelWranglerManager, PxCudaContext*mCudaContext,
	const CUstream& stream, PxgCudaBuffer* radixSortDescBuf, const PxU32 numBits)
{
	CUfunction radixFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_MULTIBLOCK);
	CUfunction calculateRanksFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK);

	PxU32 startBit = 0;
	PxU32 numPass = (numBits + 3) / 4;
	numPass += numPass & 1; // ensure even number of passes to have results in final buffer

	for (PxU32 i = 0; i < numPass; ++i)
	{
		const PxU32 descIndex = i & 1;

		CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();

		PxCudaKernelParam radixSortKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(rsDesc),
			PX_CUDA_KERNEL_PARAM(startBit)
		};

		CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, stream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail to launch kernel!!\n");

		resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, stream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail to launch kernel!!\n");

		startBit += 4;
	}

#if PS_GPU_DEBUG
	CUresult result = mCudaContext->streamSynchronize(mStream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sort fail!!\n");
#endif
}

void PxgRadixSortCore::sort(PxgCudaKernelWranglerManager* mGpuKernelWranglerManager, PxCudaContext*mCudaContext, const CUstream& stream,
	const PxU32 numOfKeys, PxgCudaBuffer* radixSortDescBuf, const PxU32 numBits, PxgRadixSortDesc* rsDescs)
{
	CUfunction radixFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_MULTIBLOCK_NO_COUNT);
	CUfunction calculateRanksFunction = mGpuKernelWranglerManager->getKernelWrangler()->getCuFunction(PxgKernelIds::RS_CALCULATERANKS_MULTIBLOCK_NO_COUNT);

	PxU32 startBit = 0;
	const PxU32 numPass = (numBits + 3) / 4;

	for (PxU32 i = 0; i < numPass; ++i)
	{
		const PxU32 descIndex = i & 1;

		CUdeviceptr rsDesc = radixSortDescBuf[descIndex].getDevicePtr();

		PxCudaKernelParam radixSortKernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM(rsDesc),
			PX_CUDA_KERNEL_PARAM(startBit)
		};

		CUresult  resultR = mCudaContext->launchKernel(radixFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, stream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail to launch kernel!!\n");

		resultR = mCudaContext->launchKernel(calculateRanksFunction, PxgRadixSortKernelGridDim::RADIX_SORT, 1, 1, PxgRadixSortKernelBlockDim::RADIX_SORT, 1, 1, 0, stream, radixSortKernelParams, sizeof(radixSortKernelParams), 0, PX_FL);
		if (resultR != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail to launch kernel!!\n");

		startBit += 4;
	}

	if (numPass & 1)
	{
		//Odd number of passes performed, sorted results are in temp buffer, so copy data across to final buffer
		mCudaContext->memcpyDtoDAsync(CUdeviceptr(rsDescs[1].outputKeys), CUdeviceptr(rsDescs[1].inputKeys), sizeof(PxU32)*numOfKeys, stream);
		mCudaContext->memcpyDtoDAsync(CUdeviceptr(rsDescs[1].outputRanks), CUdeviceptr(rsDescs[1].inputRanks), sizeof(PxU32)*numOfKeys, stream);
	}

	/*CUresult result = mCudaContext->streamSynchronize(stream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail!!\n");*/

#if PS_GPU_SPARSE_GRID_CORE_DEBUG
	CUresult result = mCudaContext->streamSynchronize(stream);
	if (result != CUDA_SUCCESS)
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "GPU sortParticles fail!!\n");

	/*PxgParticleSystem* particleSystems = mSimController->getParticleSystems();

	PxgParticleSystem& particleSystem = particleSystems[0];
	PxgParticleSystemData& data = particleSystem.mData;
	const PxU32 numParticles = data.mNumParticles;
	PxArray<PxU32> hash;
	PxArray<PxU32> particleIndex;
	hash.reserve(numParticles);
	hash.forceSize_Unsafe(numParticles);

	particleIndex.reserve(numParticles);
	particleIndex.forceSize_Unsafe(numParticles);

	CUdeviceptr hashd = reinterpret_cast<CUdeviceptr>(particleSystems[0].mGridParticleHash);
	CUdeviceptr particleIndexd = reinterpret_cast<CUdeviceptr>(particleSystems[0].mGridParticleIndex);

	mCudaContext->memcpyDtoH(hash.begin(), hashd, sizeof(PxU32) * numParticles);
	mCudaContext->memcpyDtoH(particleIndex.begin(), particleIndexd, sizeof(PxU32) * numParticles);

	int bob = 0;
	PX_UNUSED(bob);*/
#endif
}
