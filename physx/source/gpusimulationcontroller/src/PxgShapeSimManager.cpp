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

#include "PxgShapeSimManager.h"
#include "PxgHeapMemAllocator.h"
#include "PxgNarrowphaseCore.h"
#include "GuBounds.h"
#include "CmTask.h"
#include "CmFlushPool.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgCudaMemoryAllocator.h"
#include "cudamanager/PxCudaContext.h"
#include "CudaKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgSimulationCoreKernelIndices.h"

#define SSM_GPU_DEBUG	0

using namespace physx;

PxgShapeSimManager::PxgShapeSimManager(PxgHeapMemoryAllocatorManager* heapMemoryManager) :
	mTotalNumShapes		(0),
	mNbTotalShapeSim	(0),
	mPxgShapeSimPool	(PxVirtualAllocator(heapMemoryManager->mMappedMemoryAllocators)),
	mShapeSimBuffer		(heapMemoryManager, PxsHeapStats::eSIMULATION),
	mNewShapeSimBuffer	(heapMemoryManager, PxsHeapStats::eSIMULATION)
{
}

void PxgShapeSimManager::addPxgShape(Sc::ShapeSimBase* shapeSimBase, const PxsShapeCore* shapeCore, PxNodeIndex nodeIndex, PxU32 index)
{
	if(mShapeSims.capacity() <= index)
	{
		mShapeSims.resize(2*index+1);
		mShapeSimPtrs.resize(2*index+1);
	}

	mShapeSims[index].mShapeCore = shapeCore;
	mShapeSims[index].mElementIndex_GPU = index;
	mShapeSims[index].mBodySimIndex_GPU = nodeIndex;

	mShapeSimPtrs[index] = shapeSimBase;
	
	mNewShapeSims.pushBack(index);
	mTotalNumShapes = PxMax(mTotalNumShapes, index+1);
}

// This method assigns the bodySimIndex for articulation links because the nodeIndex is not available for articulation links
// until after creation, when they are inserted into the articulation and receive their node index.
void PxgShapeSimManager::setPxgShapeBodyNodeIndex(PxNodeIndex nodeIndex, PxU32 index)
{
	mShapeSims[index].mBodySimIndex_GPU = nodeIndex;
}

void PxgShapeSimManager::removePxgShape(PxU32 index)
{
	mShapeSims[index].mBodySimIndex_GPU = PxNodeIndex(PX_INVALID_NODE);
	mShapeSims[index].mElementIndex_GPU = PX_INVALID_U32;

	mShapeSimPtrs[index] = NULL;

	mNewShapeSims.pushBack(index);
}

namespace physx	// PT: only in physx namespace for the friend access to work
{
	class PxgCopyToShapeSimTask : public Cm::Task
	{
		PxgShapeSimManager*		mShapeSimManager;
		PxgGpuNarrowphaseCore*	mNpCore;
		const PxU32				mStartIndex;
		const PxU32				mNbToProcess;

	public:
		PxgCopyToShapeSimTask(PxgShapeSimManager* shapeSimManager, PxgGpuNarrowphaseCore* npCore, PxU32 startIdx, PxU32 nbToProcess) :
			Cm::Task			(0),	// PT: TODO: add missing context ID ... but then again it's missing from most of the GPU code anyway
			mShapeSimManager	(shapeSimManager),
			mNpCore				(npCore),
			mStartIndex			(startIdx),
			mNbToProcess		(nbToProcess)
		{
		}

		virtual void runInternal()
		{
			PxgNewShapeSim* dst = mShapeSimManager->mPxgShapeSimPool.begin();
			const PxU32* newShapeSimsIndices = mShapeSimManager->mNewShapeSims.begin();
			const PxgShapeSimData* src = mShapeSimManager->mShapeSims.begin();
		
			PxgGpuNarrowphaseCore* npCore = mNpCore;
			const PxU32 shapeStartIndex = mStartIndex;
			const PxU32 endIndex = mNbToProcess + shapeStartIndex;
			for (PxU32 i = shapeStartIndex; i < endIndex; ++i)
			{
				PxgNewShapeSim& shapeSim = dst[i];

				const PxU32 shapeIndex = newShapeSimsIndices[i];

				const PxgShapeSimData& shapeLL = src[shapeIndex];

				const PxsShapeCore* shapeCore = shapeLL.mShapeCore;

				shapeSim.mTransform = shapeCore->getTransform();
				shapeSim.mElementIndex = shapeLL.mElementIndex_GPU;
				shapeSim.mBodySimIndex = shapeLL.mBodySimIndex_GPU;
				shapeSim.mShapeFlags = shapeCore->mShapeFlags;
				// ML: if the shape has been removed, we shouldn't calculate the bound (PT: otherwise it crashes, as the corresponding shape data has already been deleted)
				if (shapeSim.mElementIndex != PX_INVALID_U32)
					shapeSim.mLocalBounds = Gu::computeBounds(shapeCore->mGeometry.getGeometry(), PxTransform(PxIdentity));
				else
					shapeSim.mElementIndex = shapeIndex;

				shapeSim.mHullDataIndex = npCore->getShapeIndex(*shapeCore);
				shapeSim.mShapeType = PxU16(shapeCore->mGeometry.getType());
			}
		}

		virtual const char* getName() const
		{
			return "PxgCopyToShapeSimTask";
		}

	private:
		PX_NOCOPY(PxgCopyToShapeSimTask)
	};
}

void PxgShapeSimManager::copyToGpuShapeSim(PxgGpuNarrowphaseCore* npCore, PxBaseTask* continuation, Cm::FlushPool& flushPool)
{
	const PxU32 nbNewShapes = mNewShapeSims.size();

	// PT: ??? why not resize? you're not supposed to abuse forceSize_Unsafe
	mPxgShapeSimPool.forceSize_Unsafe(0);
	mPxgShapeSimPool.reserve(nbNewShapes);
	mPxgShapeSimPool.forceSize_Unsafe(nbNewShapes);
	//mPxgShapeSimPool.resize(nbNewShapes);

	// PT: TODO: better task management....
	const PxU32 maxElementsPerTask = 1024;

	for (PxU32 i = 0; i < nbNewShapes; i += maxElementsPerTask)
	{
		PxgCopyToShapeSimTask* task =
			PX_PLACEMENT_NEW(flushPool.allocate(sizeof(PxgCopyToShapeSimTask)), PxgCopyToShapeSimTask)(this, npCore, i, PxMin(maxElementsPerTask, nbNewShapes - i));

		startTask(task, continuation);
	}
}

void PxgShapeSimManager::gpuMemDmaUpShapeSim(PxCudaContext* cudaContext, CUstream stream, KernelWrangler* kernelWrangler)
{
	const PxU32 nbTotalShapes = mTotalNumShapes;
		
	const PxPinnedArray<PxgNewShapeSim>& newShapeSimPool = mPxgShapeSimPool;

	const PxU32 nbNewShapes = newShapeSimPool.size();

	//This will allocate PxgShapeSim 
	if (nbTotalShapes > mNbTotalShapeSim)
	{
		PxU64 oldCapacity = mShapeSimBuffer.getSize();
		mShapeSimBuffer.allocateCopyOldDataAsync(nbTotalShapes * sizeof(PxgShapeSim), cudaContext, stream, PX_FL);
		if (oldCapacity < mShapeSimBuffer.getSize())
			cudaContext->memsetD32Async(mShapeSimBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mShapeSimBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);

		mNbTotalShapeSim = nbTotalShapes;
	}

	if (nbNewShapes)
	{
		mNewShapeSimBuffer.allocate(nbNewShapes * sizeof(PxgNewShapeSim), PX_FL);
		cudaContext->memcpyHtoDAsync(mNewShapeSimBuffer.getDevicePtr(), newShapeSimPool.begin(), sizeof(PxgNewShapeSim)* nbNewShapes, stream);

		const PxgNewShapeSim* newShapeSimsBufferDeviceData = mNewShapeSimBuffer.getTypedPtr();
		PxgShapeSim* shapeSimsBufferDeviceData = mShapeSimBuffer.getTypedPtr();

		void* kernelParams[] =
		{
			PX_CUDA_KERNEL_PARAM2(newShapeSimsBufferDeviceData),
			PX_CUDA_KERNEL_PARAM2(shapeSimsBufferDeviceData),
			PX_CUDA_KERNEL_PARAM2(nbNewShapes)
		};

		const CUfunction kernelFunction = kernelWrangler->getCuFunction(PxgKernelIds::UPDATE_SHAPES);
		CUresult result = cudaContext->launchKernel(kernelFunction, PxgSimulationCoreKernelGridDim::UPDATE_BODIES_AND_SHAPES, 1, 1, PxgSimulationCoreKernelBlockDim::UPDATE_BODIES_AND_SHAPES, 1, 1, 0, stream, kernelParams, 0, PX_FL);
		PX_UNUSED(result);

#if SSM_GPU_DEBUG
		result = cudaContext->streamSynchronize(stream);
		if (result != CUDA_SUCCESS)
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "updateShapesLaunch kernel fail!\n");
#endif
	}

	mNewShapeSims.clear();
}
