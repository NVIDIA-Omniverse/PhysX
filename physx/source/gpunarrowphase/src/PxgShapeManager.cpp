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

#include "PxgShapeManager.h"
#include "PxgCopyManager.h"
#include "PxgHeapMemAllocator.h"
#include "PxgCudaUtils.h"
#include "PxNodeIndex.h"

using namespace physx;

PxgShapeManager::PxgShapeManager(PxgHeapMemoryAllocatorManager* heapManager) :
	mHeapManager(heapManager),
	mHostShapes(PxVirtualAllocator(heapManager->mMappedMemoryAllocators, PxsHeapStats::eNARROWPHASE)),
	mHostShapesRemapTable(PxVirtualAllocator(heapManager->mMappedMemoryAllocators, PxsHeapStats::eNARROWPHASE)),
	mHostShapeIdTable(PxVirtualAllocator(heapManager->mMappedMemoryAllocators, PxsHeapStats::eNARROWPHASE)),
	mHostTransformCacheIdToActorTable(PxVirtualAllocator(heapManager->mMappedMemoryAllocators, PxsHeapStats::eNARROWPHASE)),
	mGpuShapesBuffer(heapManager, PxsHeapStats::eNARROWPHASE),
	mGpuShapesRemapTableBuffer(heapManager, PxsHeapStats::eNARROWPHASE),
	mGpuTransformCacheIdToActorTableBuffer(heapManager, PxsHeapStats::eNARROWPHASE),
	mGpuRigidIndiceBuffer(heapManager, PxsHeapStats::eNARROWPHASE),
	mGpuShapeIndiceBuffer(heapManager, PxsHeapStats::eNARROWPHASE),
	mGpuUnsortedShapeIndicesBuffer(heapManager, PxsHeapStats::eNARROWPHASE),
	mGpuTempRigidBitIndiceBuffer(heapManager, PxsHeapStats::eNARROWPHASE),
	mGpuTempRigidIndiceBuffer(heapManager, PxsHeapStats::eNARROWPHASE)
{
	//allocate x4
	const PxU32 initialSize = 128;
	mHostShapes.forceSize_Unsafe(0);
	mHostShapes.reserve(initialSize);
	mHostShapes.forceSize_Unsafe(initialSize);

	mHostShapesRemapTable.forceSize_Unsafe(0);
	mHostShapesRemapTable.reserve(initialSize);
	mHostShapesRemapTable.forceSize_Unsafe(initialSize);

	mHostShapeIdTable.forceSize_Unsafe(0);
	mHostShapeIdTable.reserve(initialSize);
	mHostShapeIdTable.forceSize_Unsafe(initialSize);

	mHostTransformCacheIdToActorTable.forceSize_Unsafe(0);
	mHostTransformCacheIdToActorTable.reserve(initialSize);
	mHostTransformCacheIdToActorTable.forceSize_Unsafe(initialSize);

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
	const PxU32 shapeId = mIdPool.getNewID();

	if (shapeId >= mHostShapes.capacity())
	{
		mResizeRequired = true;
		const PxU32 capacity = shapeId * 2;

		//make sure capacity is x4 because we need to use radix sort to sort shape id based on rigid body index later
		const PxU32 tempCapacity = (capacity + 3)&(~3);
		mHostShapes.resize(tempCapacity);
		mDirtyShapeMap.resize(tempCapacity);
	}

	mHostShapes[shapeId] = shape;
	mDirtyShapeMap.growAndSet(shapeId);

	mMaxShapeId = PxMax(PxI32(shapeId), mMaxShapeId);

	mHasShapeChanged = true;

	return shapeId;
}

void PxgShapeManager::registerShapeInstance(const PxNodeIndex& nodeIndex, const PxU32 transformCacheID, PxActor* actor, bool aggregate)
{
	if (transformCacheID >= mHostShapesRemapTable.capacity())
	{
		const PxU32 capacity = transformCacheID*2;
		//make sure capacity is x4 because we need to use radix sort to sort shape id based on rigid body index later
		const PxU32 tempCapacity = (capacity + 3)&(~3);
		mTransformCacheResizeRequired = true;
		mHostShapesRemapTable.resize(tempCapacity);
		mHostShapeIdTable.resize(tempCapacity);
		mHostTransformCacheIdToActorTable.resize(tempCapacity);
		mDirtyTransformCacheMap.resize(tempCapacity);
	}
		
	mHostShapesRemapTable[transformCacheID] = nodeIndex;
	mHostShapeIdTable[transformCacheID] = aggregate? 0xffffffff : transformCacheID;
	mHostTransformCacheIdToActorTable[transformCacheID] = aggregate ? NULL : actor;
	mHasShapeInstanceChanged = true;
	mDirtyTransformCacheMap.growAndSet(transformCacheID);
	mMaxTransformCacheID = PxMax(PxI32(transformCacheID), mMaxTransformCacheID);
}

void PxgShapeManager::unregisterShape(const PxU32 id)
{
	mDirtyShapeMap.reset(id);
	mIdPool.deferredFreeID(id);
	mHasShapeChanged = true;
}

void PxgShapeManager::unregisterShapeInstance(const PxU32 transformCacheID)
{
	mDirtyTransformCacheMap.set(transformCacheID);
	mHostShapesRemapTable[transformCacheID] = PxNodeIndex(PX_INVALID_NODE);
	mHostShapeIdTable[transformCacheID] = 0xffffffff;
	mHostTransformCacheIdToActorTable[transformCacheID] = NULL;
	mHasShapeInstanceChanged = true;
}

void PxgShapeManager::scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext, CUstream stream)
{
	PX_UNUSED(copyManager);

	const PxU32 maxGrouping = 16;

	if (mHasShapeChanged)
	{
		mHasShapeChanged = false;

		if (mResizeRequired)
		{
			//Allocate and copy data across
			mGpuShapesBuffer.allocateCopyOldDataAsync(sizeof(PxgShape)*mHostShapes.capacity(), cudaContext, stream, PX_FL);

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

					void* hostPtr = mHostShapes.begin() + dirtyId;

					PxgCopyManager::CopyDesc desc;
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
			mGpuShapesRemapTableBuffer.allocateCopyOldDataAsync(sizeof(PxNodeIndex)*mHostShapesRemapTable.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuShapesRemapTableBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mGpuShapesRemapTableBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);

			oldCapacity = mGpuRigidIndiceBuffer.getSize();
			mGpuRigidIndiceBuffer.allocateCopyOldDataAsync(sizeof(PxNodeIndex) * mHostShapesRemapTable.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuRigidIndiceBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mGpuRigidIndiceBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);

			mGpuTempRigidIndiceBuffer.allocate(sizeof(PxNodeIndex) * mHostShapesRemapTable.capacity(), PX_FL);

			oldCapacity = mGpuShapeIndiceBuffer.getSize();
			mGpuShapeIndiceBuffer.allocateCopyOldDataAsync(sizeof(PxU32) * mHostShapeIdTable.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuShapeIndiceBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mGpuShapeIndiceBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);

			oldCapacity = mGpuUnsortedShapeIndicesBuffer.getSize();
			mGpuUnsortedShapeIndicesBuffer.allocateCopyOldDataAsync(sizeof(PxU32) * mHostShapeIdTable.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuUnsortedShapeIndicesBuffer.getDevicePtr() + oldCapacity, 0xFFFFFFFF, (mGpuUnsortedShapeIndicesBuffer.getSize() - oldCapacity) / sizeof(PxU32), stream);


			mGpuTempRigidBitIndiceBuffer.allocate(sizeof(PxU32) * mHostShapeIdTable.capacity(), PX_FL);

			oldCapacity = mGpuTransformCacheIdToActorTableBuffer.getSize();
			mGpuTransformCacheIdToActorTableBuffer.allocateCopyOldDataAsync(sizeof(PxActor*) * mHostTransformCacheIdToActorTable.capacity(), cudaContext, stream, PX_FL);
			cudaContext->memsetD32Async(mGpuTransformCacheIdToActorTableBuffer.getDevicePtr() + oldCapacity, 0, (mGpuTransformCacheIdToActorTableBuffer.getSize() - oldCapacity) / sizeof(PxActor*), stream);

			mTransformCacheResizeRequired = false;
		}

		const PxU32 totalNumOfShapeInstances = mMaxTransformCacheID + 1;
		const PxU32 numShapeInstances = (totalNumOfShapeInstances + 3) &(~3);

		//make sure the dirty shape map cover x4 case and set those to invalid value
		for (PxU32 i = totalNumOfShapeInstances; i < numShapeInstances; ++i)
		{
			if (!mHostShapesRemapTable[i].isStaticBody())
			{
				mDirtyTransformCacheMap.growAndSet(i);
				mHostShapesRemapTable[i] = PxNodeIndex(PX_INVALID_NODE);
				mHostShapeIdTable[i] = 0xffffffff;
				mHostTransformCacheIdToActorTable[i] = NULL;
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

					void* hostRemapPtr = mHostShapesRemapTable.begin() + dirtyId;

					void* hostShapeIdPtr = mHostShapeIdTable.begin() + dirtyId;

					void* hostTransformCacheIdToActorPtr = mHostTransformCacheIdToActorTable.begin() + dirtyId;

					PxgCopyManager::CopyDesc desc1;
					desc1.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostRemapPtr));
					desc1.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuShapesRemapTableBuffer.getDevicePtr()) + dirtyId * sizeof(PxNodeIndex));
					desc1.bytes = sizeof(PxNodeIndex);

					PxgCopyManager::CopyDesc desc2;
					desc2.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostRemapPtr));
					desc2.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuRigidIndiceBuffer.getDevicePtr()) + dirtyId * sizeof(PxNodeIndex));
					desc2.bytes = sizeof(PxNodeIndex);

					PxgCopyManager::CopyDesc desc3;
					desc3.source = reinterpret_cast<size_t>(getMappedDevicePtr(cudaContext, hostShapeIdPtr));
					desc3.dest = reinterpret_cast<size_t>(reinterpret_cast<PxU8*>(mGpuUnsortedShapeIndicesBuffer.getDevicePtr()) + dirtyId * sizeof(PxU32));
					desc3.bytes = sizeof(PxU32);

					PxgCopyManager::CopyDesc desc4;
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
	PX_ASSERT(id < mHostShapes.size());
	mHostShapes[id].materialIndex = materialIndex;
	mDirtyShapeMap.growAndSet(id);
	mHasShapeChanged = true;
}

////////////////////////////////////////////////////////////////////////////////////////////

PxgMaterialManager::PxgMaterialManager(PxgHeapMemoryAllocatorManager* heapManager, const PxU32 elemSize) :
	mGpuMaterialBuffer(heapManager, PxsHeapStats::eNARROWPHASE),
	mHeapManager(heapManager),
	mHostMaterial(PxVirtualAllocator(heapManager->mMappedMemoryAllocators, PxsHeapStats::eNARROWPHASE))
{
	const PxU32 originalSize = elemSize * 128;
	mHostMaterial.forceSize_Unsafe(0);
	mHostMaterial.reserve(originalSize);
	mHostMaterial.forceSize_Unsafe(originalSize);

	mGpuMaterialBuffer.allocate(originalSize, PX_FL);
	mResizeRequired = false;
}

PxU32 PxgMaterialManager::registerMaterial(const PxU8* materialData, const PxU32 elemSize)
{
	const PxU32 shapeId = mIdPool.getNewID();
	PxU32 capacity = mHostMaterial.capacity() / elemSize;
	
	if (shapeId >= capacity)
	{
		capacity = PxMax(capacity * 2 + 1, shapeId + 1);
		mHostMaterial.resize(capacity * elemSize);
		mResizeRequired = true;
	}

	PxU8* destPtr = mHostMaterial.begin() + shapeId * elemSize;
	PxMemCopy(destPtr, materialData, elemSize);
		
	mDirtyMaterialMap.growAndSet(shapeId);

	return shapeId;
}

void PxgMaterialManager::unregisterMaterial(const PxU32 id)
{
	mDirtyMaterialMap.reset(id);
	mIdPool.deferredFreeID(id);
}

void PxgMaterialManager::scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext, 
	CUstream stream, const PxU32 elemSize)
{
	if (mResizeRequired)
	{
		mGpuMaterialBuffer.allocateCopyOldDataAsync(mHostMaterial.capacity(), cudaContext, stream, PX_FL);
		mResizeRequired = false;
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

				void* hostPtr = mHostMaterial.begin() + dirtyId * elemSize;

				PxgCopyManager::CopyDesc desc;
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
	PX_ASSERT(id < mHostMaterial.size());
	PxU8* destptr = reinterpret_cast<PxU8*>(mHostMaterial.begin() + id * elemSize);
	PxMemCopy(destptr, materialCore, elemSize);
	//mHostMaterial[id] = materialCore;
	mDirtyMaterialMap.growAndSet(id);
}

//////////////////////////////////////////////////////////////////////////////////////////////
	
PxgFEMMaterialManager::PxgFEMMaterialManager(PxgHeapMemoryAllocatorManager* heapManager, const PxU32 elemSize) :
	PxgMaterialManager(heapManager, elemSize)
{

}

void PxgFEMMaterialManager::scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext,
	CUstream stream, const PxU32 elemSize)
{
	if (mResizeRequired)
	{
		mGpuMaterialBuffer.allocateCopyOldDataAsync(mHostMaterial.capacity(), cudaContext, stream, PX_FL);
		mResizeRequired = false;
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

				void* hostPtr = mHostMaterial.begin() + dirtyId * elemSize;

				PxgCopyManager::CopyDesc desc;
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
	
PxgFEMSoftBodyMaterialManager::PxgFEMSoftBodyMaterialManager(PxgHeapMemoryAllocatorManager* heapManager) :
	PxgFEMMaterialManager(heapManager, sizeof(PxsDeformableVolumeMaterialData))
{
}
