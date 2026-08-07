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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PxgShapeManager.h"
#include "PxgCopyManager.h"
#include "PxgCudaUtils.h"
#include "PxgAllocatorDesc.h"
#include "PxsHeapStats.h"
#include "PxNodeIndex.h"

using namespace physx;

PxgShapeManager::PxgShapeManager(PxgAllocatorDesc& allocDesc) :
	mHostShapesMapped(allocDesc.hostMappedAlloc, PxsHeapStats::eNARROWPHASE, Cm::PinnableAllocatorFallback::eDISABLED),
	mHostShapesRemapTableMapped(allocDesc.hostMappedAlloc, PxsHeapStats::eNARROWPHASE, Cm::PinnableAllocatorFallback::eDISABLED),
	mHostTransformCacheIdToActorTableMapped(allocDesc.hostMappedAlloc, PxsHeapStats::eNARROWPHASE, Cm::PinnableAllocatorFallback::eDISABLED),
	mHostShapeIdTableMapped(allocDesc.hostMappedAlloc, PxsHeapStats::eNARROWPHASE, Cm::PinnableAllocatorFallback::eDISABLED),
	mGpuShapesBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mGpuShapesRemapTableBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mGpuTransformCacheIdToActorTableBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mGpuRigidIndiceBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mGpuShapeIndiceBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mGpuUnsortedShapeIndicesBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mGpuTempRigidBitIndiceBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mGpuTempRigidIndiceBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mAllocFailed(false)
{
	//allocate x4
	const PxU32 initialSize = 128;
	if(!mHostShapesMapped.reserve(initialSize))
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgShapeManager: failed to allocate pinned host buffer for mHostShapes");
		mAllocFailed = true;
		return;
	}
	mHostShapesMapped.forceSize_Unsafe(initialSize);

	if(!mHostShapesRemapTableMapped.reserve(initialSize))
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgShapeManager: failed to allocate pinned host buffer for mHostShapesRemapTable");
		mAllocFailed = true;
		return;
	}
	mHostShapesRemapTableMapped.forceSize_Unsafe(initialSize);

	if (!mHostShapeIdTableMapped.reserve(initialSize))
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgShapeManager: failed to allocate pinned host buffer for mHostShapeIdTable");
		mAllocFailed = true;
		return;
	}
	mHostShapeIdTableMapped.forceSize_Unsafe(initialSize);

	if(!mHostTransformCacheIdToActorTableMapped.reserve(initialSize))
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgShapeManager: failed to allocate pinned host buffer for mHostTransformCacheIdToActorTable");
		mAllocFailed = true;
		return;
	}
	mHostTransformCacheIdToActorTableMapped.forceSize_Unsafe(initialSize);

	mGpuShapesBuffer.allocate(sizeof(PxgShape)*initialSize, PX_FL);
	mGpuShapesRemapTableBuffer.allocate(sizeof(PxNodeIndex) * initialSize, PX_FL);
	mGpuTransformCacheIdToActorTableBuffer.allocate(sizeof(PxActor*) * initialSize, PX_FL);
	mGpuRigidIndiceBuffer.allocate(sizeof(PxNodeIndex) * initialSize, PX_FL);
	mGpuShapeIndiceBuffer.allocate(sizeof(PxU32) * initialSize, PX_FL);
	mGpuUnsortedShapeIndicesBuffer.allocate(sizeof(PxU32) * initialSize, PX_FL);
	mGpuTempRigidBitIndiceBuffer.allocate(sizeof(PxU32) * initialSize, PX_FL);
	mGpuTempRigidIndiceBuffer.allocate(sizeof(PxNodeIndex) * initialSize, PX_FL);

	mResizeRequired = false;
	mTransformCacheResizeRequired = false;
	mMaxShapeId = -1;
	mMaxTransformCacheID = -1;

	mHasShapeChanged = false;
	mHasShapeInstanceChanged = false;
}

void PxgShapeManager::initialize(PxCudaContext* cudaContext, CUstream stream)
{
	cudaContext->memsetD32Async(mGpuShapesRemapTableBuffer.getDevicePtr(), 0xFFFFFFFF, mGpuShapesRemapTableBuffer.getSize()/sizeof(PxU32), stream);
	cudaContext->memsetD32Async(mGpuRigidIndiceBuffer.getDevicePtr(), 0xFFFFFFFF, mGpuRigidIndiceBuffer.getSize() / sizeof(PxU32), stream);
	cudaContext->memsetD32Async(mGpuShapeIndiceBuffer.getDevicePtr(), 0xFFFFFFFF, mGpuShapeIndiceBuffer.getSize() / sizeof(PxU32), stream);
	cudaContext->memsetD32Async(mGpuUnsortedShapeIndicesBuffer.getDevicePtr(), 0xFFFFFFFF, mGpuUnsortedShapeIndicesBuffer.getSize() / sizeof(PxU32), stream);
	
}

PxU32 PxgShapeManager::registerShape(PxgShape& shape)
{
	if(mAllocFailed)
	{
		return PX_INVALID_U32;
	}

	const PxU32 shapeId = mIdPool.getNewID();

	if (shapeId >= mHostShapesMapped.capacity())
	{
		mResizeRequired = true;
		const PxU32 capacity = shapeId * 2;

		//make sure capacity is x4 because we need to use radix sort to sort shape id based on rigid body index later
		const PxU32 tempCapacity = (capacity + 3)&(~3);
		
		if(!mHostShapesMapped.resize(tempCapacity))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgShapeManager: failed to allocate pinned host buffer for mHostShapes");
			mAllocFailed = true;
			return PX_INVALID_U32;
		}

		mDirtyShapeMap.resize(tempCapacity);
	}

	mHostShapesMapped[shapeId] = shape;
	mDirtyShapeMap.growAndSet(shapeId);

	mMaxShapeId = PxMax(PxI32(shapeId), mMaxShapeId);

	mHasShapeChanged = true;

	return shapeId;
}

void PxgShapeManager::registerShapeInstance(const PxNodeIndex& nodeIndex, const PxU32 transformCacheID, PxActor* actor, bool aggregate)
{
	if(mAllocFailed)
	{
		return;
	}

	if (transformCacheID >= mHostShapesRemapTableMapped.capacity())
	{
		const PxU32 capacity = transformCacheID*2;
		//make sure capacity is x4 because we need to use radix sort to sort shape id based on rigid body index later
		const PxU32 tempCapacity = (capacity + 3)&(~3);
		mTransformCacheResizeRequired = true;
		if(!mHostShapesRemapTableMapped.resize(tempCapacity))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgShapeManager: failed to allocate pinned host buffer for mHostShapesRemapTable");
			mAllocFailed = true;
			return;
		}

		if(!mHostShapeIdTableMapped.resize(tempCapacity))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgShapeManager: failed to allocate pinned host buffer for mHostShapeIdTable");
			mAllocFailed = true;
			return;
		}
		
		if(!mHostTransformCacheIdToActorTableMapped.resize(tempCapacity))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgShapeManager: failed to allocate pinned host buffer for mHostTransformCacheIdToActorTable");
			mAllocFailed = true;
			return;
		}

		mDirtyTransformCacheMap.resize(tempCapacity);
	}
		
	mHostShapesRemapTableMapped[transformCacheID] = nodeIndex;
	mHostShapeIdTableMapped[transformCacheID] = aggregate ? 0xffffffff : transformCacheID;
	mHostTransformCacheIdToActorTableMapped[transformCacheID] = aggregate ? NULL : actor;
	mHasShapeInstanceChanged = true;
	mDirtyTransformCacheMap.growAndSet(transformCacheID);
	mMaxTransformCacheID = PxMax(PxI32(transformCacheID), mMaxTransformCacheID);
}

void PxgShapeManager::unregisterShape(const PxU32 id)
{
	if(mAllocFailed)
	{
		return;
	}

	mDirtyShapeMap.reset(id);
	mIdPool.deferredFreeID(id);
	mHasShapeChanged = true;
}

void PxgShapeManager::unregisterShapeInstance(const PxU32 transformCacheID)
{
	if(mAllocFailed)
	{
		return;
	}

	mDirtyTransformCacheMap.set(transformCacheID);
	mHostShapesRemapTableMapped[transformCacheID] = PxNodeIndex(PX_INVALID_NODE);
	mHostShapeIdTableMapped[transformCacheID] = 0xffffffff;
	mHostTransformCacheIdToActorTableMapped[transformCacheID] = NULL;
	mHasShapeInstanceChanged = true;
}

void PxgShapeManager::scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext, CUstream stream)
{
	PX_UNUSED(copyManager);
	PX_ASSERT(cudaContext);

	const PxU32 maxGrouping = 16;

	if (mAllocFailed)
	{
		cudaContext->setAbortMode(true);
		return;
	}

	if (mHasShapeChanged)
	{
		mHasShapeChanged = false;

		if (mResizeRequired)
		{
			//Allocate and copy data across
			mGpuShapesBuffer.allocateCopyOldDataAsync(sizeof(PxgShape)*mHostShapesMapped.capacity(), cudaContext, stream, PX_FL);

			mResizeRequired = false;
		}

		const PxU32* bits = mDirtyShapeMap.getWords();
		if (bits)
		{
			const PxU32 totalNumOfShapes = mMaxShapeId + 1;
			const PxU32 numShapes = (totalNumOfShapes + 3) &(~3);

			//make sure the dirty shape map cover x4 case and set those to invalid value
			for (PxU32 i = totalNumOfShapes; i < numShapes; ++i)
			{
				mDirtyShapeMap.growAndSet(i);
			}

			// PT: ### bitmap iterator pattern
			const PxU32 lastSetBit = mDirtyShapeMap.findLast();
			for (PxU32 w = 0; w <= lastSetBit >> 5; ++w)
			{
				//b&=b-1 will clear the lowest set bit in b
				for (PxU32 b = bits[w]; b; )
				{
					//dirtyId is the next bit that's set to 1!
					const PxU32 dirtyId = PxU32(w << 5 | PxLowestSetBit(b));

					void* hostPtr = mHostShapesMapped.begin() + dirtyId;

					PxgCopyDesc desc;
					desc.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostPtr));
					desc.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuShapesBuffer.getDevicePtr()) + dirtyId * sizeof(PxgShape));
					desc.bytes = sizeof(PxgShape);

					mDirtyShapeMap.reset(dirtyId);
					//Now we loop to try and find adjacent bits that are set...
					PxU32 currIdx = dirtyId + 1;
					PxU32 groupSize = 1;
					while (currIdx <= lastSetBit && mDirtyShapeMap.test(currIdx) && groupSize < maxGrouping)
					{
						groupSize++;
						mDirtyShapeMap.reset(currIdx);
						currIdx++;
						desc.bytes += sizeof(PxgShape);
					}

					if (currIdx != (dirtyId + 1))
					{
						//get the word from the current bit
						w = PxMin(currIdx, lastSetBit) >> 5;
						//reload the world
						b = bits[w]; //Set a 1 here to make sure the b &= (b-1) in the for loop doesn't remove the current bit we're interested in
					}
					else
					{
						b &= (b - 1);
					}

					copyManager.pushDeferredHtoD(desc);
				}
			}
		}

		mDirtyShapeMap.clear();
	}

	if (mHasShapeInstanceChanged)
	{
		//AD: mHasShapeInstanceChanged needs to persist because computeRigidsToShapes() needs to run if we use direct-API 
		//    we lower the flag in PxgNarrowphaseCore::prepareGpuNarrowphase.

		// AD: the resize of the GPU transform cache is inside PxgNarrowphaseCore::prepareGpuNarrowphase.
		if (mTransformCacheResizeRequired)
		{
			PxU64 oldCapacity = mGpuShapesRemapTableBuffer.getSize();
			mGpuShapesRemapTableBuffer.allocateCopyOldDataAsync(sizeof(PxNodeIndex)*mHostShapesRemapTableMapped.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuShapesRemapTableBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mGpuShapesRemapTableBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);

			oldCapacity = mGpuRigidIndiceBuffer.getSize();
			mGpuRigidIndiceBuffer.allocateCopyOldDataAsync(sizeof(PxNodeIndex) * mHostShapesRemapTableMapped.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuRigidIndiceBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mGpuRigidIndiceBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);

			mGpuTempRigidIndiceBuffer.allocate(sizeof(PxNodeIndex) * mHostShapesRemapTableMapped.capacity(), PX_FL);

			oldCapacity = mGpuShapeIndiceBuffer.getSize();
			mGpuShapeIndiceBuffer.allocateCopyOldDataAsync(sizeof(PxU32) * mHostShapeIdTableMapped.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuShapeIndiceBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mGpuShapeIndiceBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);

			oldCapacity = mGpuUnsortedShapeIndicesBuffer.getSize();
			mGpuUnsortedShapeIndicesBuffer.allocateCopyOldDataAsync(sizeof(PxU32) * mHostShapeIdTableMapped.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuUnsortedShapeIndicesBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mGpuUnsortedShapeIndicesBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);


			mGpuTempRigidBitIndiceBuffer.allocate(sizeof(PxU32) * mHostShapeIdTableMapped.capacity(), PX_FL);

			oldCapacity = mGpuTransformCacheIdToActorTableBuffer.getSize();
			mGpuTransformCacheIdToActorTableBuffer.allocateCopyOldDataAsync(sizeof(PxActor*) * mHostTransformCacheIdToActorTableMapped.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuTransformCacheIdToActorTableBuffer.getDevicePtr() + oldCapacity, 0, (mGpuTransformCacheIdToActorTableBuffer.getSize() - oldCapacity) / sizeof(PxActor*), stream);

			mTransformCacheResizeRequired = false;
		}

		const PxU32 totalNumOfShapeInstances = mMaxTransformCacheID + 1;
		const PxU32 numShapeInstances = (totalNumOfShapeInstances + 3) &(~3);

		//make sure the dirty shape map cover x4 case and set those to invalid value
		for (PxU32 i = totalNumOfShapeInstances; i < numShapeInstances; ++i)
		{
			if (!mHostShapesRemapTableMapped[i].isStaticBody())
			{
				mDirtyTransformCacheMap.growAndSet(i);
				mHostShapesRemapTableMapped[i] = PxNodeIndex(PX_INVALID_NODE);
				mHostShapeIdTableMapped[i] = 0xffffffff;
				mHostTransformCacheIdToActorTableMapped[i] = NULL;
			}
		}

		const PxU32* bits = mDirtyTransformCacheMap.getWords();

		if (bits)
		{
			// PT: ### bitmap iterator pattern
			const PxU32 lastSetBit = mDirtyTransformCacheMap.findLast();
			for (PxU32 w = 0; w <= lastSetBit >> 5; ++w)
			{
				//b&=b-1 will clear the lowest set bit in b
				for (PxU32 b = bits[w]; b; )
				{
					//dirtyId is the next bit that's set to 1!
					const PxU32 dirtyId = PxU32(w << 5 | PxLowestSetBit(b));

					void* hostRemapPtr = mHostShapesRemapTableMapped.begin() + dirtyId;

					void* hostShapeIdPtr = mHostShapeIdTableMapped.begin() + dirtyId;

					void* hostTransformCacheIdToActorPtr = mHostTransformCacheIdToActorTableMapped.begin() + dirtyId;

					PxgCopyDesc desc1;
					desc1.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostRemapPtr));
					desc1.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuShapesRemapTableBuffer.getDevicePtr()) + dirtyId * sizeof(PxNodeIndex));
					desc1.bytes = sizeof(PxNodeIndex);

					PxgCopyDesc desc2;
					desc2.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostRemapPtr));
					desc2.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuRigidIndiceBuffer.getDevicePtr()) + dirtyId * sizeof(PxNodeIndex));
					desc2.bytes = sizeof(PxNodeIndex);

					PxgCopyDesc desc3;
					desc3.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostShapeIdPtr));
					desc3.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuUnsortedShapeIndicesBuffer.getDevicePtr()) + dirtyId * sizeof(PxU32));
					desc3.bytes = sizeof(PxU32);

					PxgCopyDesc desc4;
					desc4.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostTransformCacheIdToActorPtr));
					desc4.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuTransformCacheIdToActorTableBuffer.getDevicePtr()) + dirtyId * sizeof(PxActor*));
					desc4.bytes = sizeof(PxActor*);

					mDirtyTransformCacheMap.reset(dirtyId);
					//Now we loop to try and find adjacent bits that are set...
					PxU32 currIdx = dirtyId + 1;
					PxU32 groupSize = 1;
					while (currIdx <= lastSetBit && mDirtyTransformCacheMap.test(currIdx) && groupSize < maxGrouping)
					{
						groupSize++;
						mDirtyTransformCacheMap.reset(currIdx);
						currIdx++;
						desc1.bytes += sizeof(PxNodeIndex);
						desc2.bytes += sizeof(PxNodeIndex);
						desc3.bytes += sizeof(PxU32);
						desc4.bytes += sizeof(PxActor*);
					}

					if (currIdx != (dirtyId + 1))
					{
						//get the word from the current bit
						w = PxMin(currIdx, lastSetBit) >> 5;
						//reload the world
						b = bits[w]; //Set a 1 here to make sure the b &= (b-1) in the for loop doesn't remove the current bit we're interested in
					}
					else
					{
						b &= (b - 1);
					}

					copyManager.pushDeferredHtoD(desc1);
					copyManager.pushDeferredHtoD(desc2);
					copyManager.pushDeferredHtoD(desc3);
					copyManager.pushDeferredHtoD(desc4);
				}
			}
		}
		mDirtyTransformCacheMap.clear();
	}
}

void PxgShapeManager::updateShapeMaterial(const PxU32 materialIndex, const PxU32 id)
{
	if(mAllocFailed)
	{
		return;
	}
	PX_ASSERT(id < mHostShapesMapped.size());
	mHostShapesMapped[id].materialIndex = materialIndex;
	mDirtyShapeMap.growAndSet(id);
	mHasShapeChanged = true;
}

////////////////////////////////////////////////////////////////////////////////////////////

PxgMaterialManager::PxgMaterialManager(PxgAllocatorDesc& allocDesc, const PxU32 elemSize) :
	mAllocFailed(false),
	mGpuMaterialBuffer(allocDesc.deviceAlloc, PxsHeapStats::eNARROWPHASE),
	mHostMaterialMapped(allocDesc.hostMappedAlloc, PxsHeapStats::eNARROWPHASE, Cm::PinnableAllocatorFallback::eDISABLED)
{
	const PxU32 originalSize = elemSize * 128;
	if(!mHostMaterialMapped.reserve(originalSize))
	{
		PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgMaterialManager: failed to allocate pinned host buffer for mHostMaterial");
		mAllocFailed = true;
		return;
	}
	mHostMaterialMapped.forceSize_Unsafe(originalSize);

	mGpuMaterialBuffer.allocate(originalSize, PX_FL);
	mResizeRequired = false;
}

PxU32 PxgMaterialManager::registerMaterial(const PxU8* materialData, const PxU32 elemSize)
{
	const PxU32 materialId = mIdPool.getNewID();
	PxU32 capacity = mHostMaterialMapped.capacity() / elemSize;
	
	if(materialId >= capacity)
	{
		capacity = PxMax(capacity * 2 + 1, materialId + 1);
		const PxU32 newBytes = capacity * elemSize;
		if(!mHostMaterialMapped.resize(newBytes))
		{
			PxGetFoundation().error(PxErrorCode::eOUT_OF_MEMORY, PX_FL, "PxgMaterialManager: failed to allocate pinned host buffer for mHostMaterial");
			mAllocFailed = true;
			return PX_INVALID_U32;
		}
		mResizeRequired = true;
	}

	PxU8* destPtr = mHostMaterialMapped.begin() + materialId * elemSize;
	PxMemCopy(destPtr, materialData, elemSize);
		
	mDirtyMaterialMap.growAndSet(materialId);


	return materialId;
}

void PxgMaterialManager::unregisterMaterial(const PxU32 id)
{
	if(mAllocFailed)
	{
		return;
	}

	mDirtyMaterialMap.reset(id);
	mIdPool.deferredFreeID(id);
}

void PxgMaterialManager::scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext, 
	CUstream stream, const PxU32 elemSize)
{
	if (mResizeRequired)
	{
		mGpuMaterialBuffer.allocateCopyOldDataAsync(mHostMaterialMapped.capacity(), cudaContext, stream, PX_FL);
		mResizeRequired = false;
	}

	if (mAllocFailed)
	{
		cudaContext->setAbortMode(true);
		return;
	}

	const PxU32* bits = mDirtyMaterialMap.getWords();

	const PxU32 maxGrouping = 16;

	if (bits)
	{
		// PT: ### bitmap iterator pattern
		const PxU32 lastSetBit = mDirtyMaterialMap.findLast();
		for (PxU32 w = 0; w <= lastSetBit >> 5; ++w)
		{
			//b&=b-1 will clear the lowest set bit in b
			for (PxU32 b = bits[w]; b; )
			{
				//dirtyId is the next bit that's set to 1!
				const PxU32 dirtyId = PxU32(w << 5 | PxLowestSetBit(b));

				void* hostPtr = mHostMaterialMapped.begin() + dirtyId * elemSize;

				PxgCopyDesc desc;
				desc.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostPtr));
				desc.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuMaterialBuffer.getDevicePtr()) + dirtyId * elemSize);
				desc.bytes = elemSize;

				mDirtyMaterialMap.reset(dirtyId);

				//Now we loop to try and find adjacent bits that are set...
				PxU32 currIdx = dirtyId + 1;
				PxU32 groupSize = 1;
				while (currIdx <= lastSetBit && mDirtyMaterialMap.test(currIdx) && (groupSize < maxGrouping))
				{
					groupSize++;
					mDirtyMaterialMap.reset(currIdx);
					currIdx++;
					desc.bytes += elemSize;
				}

				if (currIdx != (dirtyId + 1))
				{
					//get the word from the current bit
					w = PxMin(currIdx, lastSetBit) >> 5;
					//reload the world
					b = bits[w]; //Set a 1 here to make sure the b &= (b-1) in the for loop doesn't remove the current bit we're interested in
				}
				else
				{
					b &= (b - 1);
				}

				copyManager.pushDeferredHtoD(desc);
			}
		}
	}

	mDirtyMaterialMap.clear();
}

void PxgMaterialManager::updateMaterial(const PxU8* materialCore, const PxU32 elemSize, const PxU32 id)
{
	if(mAllocFailed)
	{
		return;
	}

	PX_ASSERT(id < mHostMaterialMapped.size());
	PxU8* destptr = reinterpret_cast<PxU8*>(mHostMaterialMapped.begin() + id * elemSize);
	PxMemCopy(destptr, materialCore, elemSize);
	mDirtyMaterialMap.growAndSet(id);
}

//////////////////////////////////////////////////////////////////////////////////////////////

void PxgFEMMaterialManager::scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext,
	CUstream stream, const PxU32 elemSize)
{
	if (mResizeRequired)
	{
		mGpuMaterialBuffer.allocateCopyOldDataAsync(mHostMaterialMapped.capacity(), cudaContext, stream, PX_FL);
		mResizeRequired = false;
	}

	if(mAllocFailed)
	{
		cudaContext->setAbortMode(true);
		return;
	}

	const PxU32* bits = mDirtyMaterialMap.getWords();

	const PxU32 maxGrouping = 16;

	if (bits)
	{
		// PT: ### bitmap iterator pattern
		const PxU32 lastSetBit = mDirtyMaterialMap.findLast();
		for (PxU32 w = 0; w <= lastSetBit >> 5; ++w)
		{
			//b&=b-1 will clear the lowest set bit in b
			for (PxU32 b = bits[w]; b; )
			{
				//dirtyId is the next bit that's set to 1!
				const PxU32 dirtyId = PxU32(w << 5 | PxLowestSetBit(b));

				void* hostPtr = mHostMaterialMapped.begin() + dirtyId * elemSize;

				PxgCopyDesc desc;
				desc.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostPtr));
				desc.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuMaterialBuffer.getDevicePtr()) + dirtyId * elemSize);
				desc.bytes = elemSize;

				mDirtyMaterialMap.reset(dirtyId);

				//Now we loop to try and find adjacent bits that are set...
				PxU32 currIdx = dirtyId + 1;
				PxU32 groupSize = 1;
				while (currIdx <= lastSetBit && mDirtyMaterialMap.test(currIdx) && (groupSize < maxGrouping))
				{
					groupSize++;
					mDirtyMaterialMap.reset(currIdx);
					currIdx++;
					desc.bytes += elemSize;
				}

				if (currIdx != (dirtyId + 1))
				{
					//get the word from the current bit
					w = PxMin(currIdx, lastSetBit) >> 5;
					//reload the world
					b = bits[w]; //Set a 1 here to make sure the b &= (b-1) in the for loop doesn't remove the current bit we're interested in
				}
				else
				{
					b &= (b - 1);
				}

				copyManager.pushDeferredHtoD(desc);
			}
		}
	}

	mDirtyMaterialMap.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////
	
