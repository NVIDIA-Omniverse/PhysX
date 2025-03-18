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

#ifndef PXG_SHAPE_MANAGER_H
#define PXG_SHAPE_MANAGER_H

#include "foundation/PxPreprocessor.h"
#include "PxSceneDesc.h"
#include "foundation/PxPinnedArray.h"
#include "CmIDPool.h"
#include "PxgCudaBuffer.h"
#include "foundation/PxBitMap.h"
#include "PxsMaterialCore.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "PxsDeformableVolumeMaterialCore.h"
#include "PxsPBDMaterialCore.h"
#include "PxgConvexConvexShape.h"

namespace physx
{
	class PxCudaContext;
	class PxgHeapMemoryAllocatorManager;
	class PxgCopyManager;
	class PxNodeIndex;

	class PxgShapeManager
	{
	public:
		PxgShapeManager(PxgHeapMemoryAllocatorManager* heapManager);
		~PxgShapeManager(){}


		//this method push CopyDesc to PxgCopyManager
		void initialize(PxCudaContext* cudaContext, CUstream stream);
		void scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext, CUstream stream);

		PxU32 registerShape(PxgShape& shape);
		void registerShapeInstance(const PxNodeIndex& nodeIndex, const PxU32 transformCacheID, PxActor* actor, bool aggregate = false);
		void unregisterShape(const PxU32 shapeId);
		void unregisterShapeInstance(const PxU32 transformCacheID);

		void updateShapeMaterial(const PxU32 materalIndex, const PxU32 id);


		void releaseIDs() { mIdPool.processDeferredIds(); }

		Cm::DeferredIDPool				mIdPool;
		PxgHeapMemoryAllocatorManager*	mHeapManager;
		PxPinnedArray<PxgShape>			mHostShapes;
		PxPinnedArray<PxNodeIndex>		mHostShapesRemapTable;//remap table from shape to node index
		PxInt32ArrayPinned				mHostShapeIdTable;
		PxPinnedArray<PxActor*>			mHostTransformCacheIdToActorTable; // remap table from transformCacheId to PxActor
		PxgCudaBuffer					mGpuShapesBuffer;
		PxgCudaBuffer					mGpuShapesRemapTableBuffer;	// shape to rigid
		PxgCudaBuffer					mGpuTransformCacheIdToActorTableBuffer; // transformCacheId to PxActor
		PxgCudaBuffer					mGpuRigidIndiceBuffer; // 00011 nodeIndex 0 has shape(0, 2, 3), nodeIndex 1 has shape(1, 4)(it can be either articulation link or rigid body)
		PxgCudaBuffer					mGpuShapeIndiceBuffer; // 02314
		PxgCudaBuffer					mGpuUnsortedShapeIndicesBuffer; // needed when recomputing the shape to rigids mapping
		PxgCudaBuffer					mGpuTempRigidBitIndiceBuffer; //either the lower 32 bits or higher 32 bits of mGpuRigidIndiceBuffer
		PxgCudaBuffer					mGpuTempRigidIndiceBuffer;
		PxBitMap						mDirtyShapeMap;
		PxBitMap						mDirtyTransformCacheMap;
		bool							mResizeRequired;
		bool							mTransformCacheResizeRequired;
		PxI32							mMaxShapeId;
		PxI32							mMaxTransformCacheID;
		bool							mHasShapeChanged;
		bool							mHasShapeInstanceChanged;

	private:
		PX_NOCOPY(PxgShapeManager)
	};

	class PxgMaterialManager
	{
		PX_NOCOPY(PxgMaterialManager)
	public:
		PxgMaterialManager(PxgHeapMemoryAllocatorManager* heapManager, const PxU32 elemSize = sizeof(PxsMaterialData));
		virtual ~PxgMaterialManager(){}

		//this method push CopyDesc to PxgCopyManager
		virtual void scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext, CUstream stream, const PxU32 elemSize);

		PxU32 registerMaterial(const PxU8* materialData, const PxU32 elemSize);
		void updateMaterial(const PxU8* materialData, const PxU32 elemSize, const PxU32 id);
		
		void unregisterMaterial(const PxU32 materialId);
		void releaseIDs() { mIdPool.processDeferredIds(); }

		PxgCudaBuffer					mGpuMaterialBuffer;

	protected:
		Cm::DeferredIDPool				mIdPool;
		PxgHeapMemoryAllocatorManager*	mHeapManager;
		PxInt8ArrayPinned				mHostMaterial;
		PxBitMap						mDirtyMaterialMap;
		bool							mResizeRequired;
	};
	
	class PxgFEMMaterialManager : public PxgMaterialManager
	{
	public:

		PxgFEMMaterialManager(PxgHeapMemoryAllocatorManager* heapManager, const PxU32 elemSize);

		virtual void scheduleCopyHtoD(PxgCopyManager& copyManager, PxCudaContext* cudaContext, CUstream stream, const PxU32 elemSize);
	};

	class PxgFEMSoftBodyMaterialManager : public PxgFEMMaterialManager
	{
	public:
		PxgFEMSoftBodyMaterialManager(PxgHeapMemoryAllocatorManager* heapManager);
	};

	class PxgFEMClothMaterialManager : public PxgMaterialManager
	{
	public:
	    PxgFEMClothMaterialManager(PxgHeapMemoryAllocatorManager* heapManager) :
			PxgMaterialManager(heapManager, sizeof(PxsDeformableSurfaceMaterialData))
	    {
	    }
	    PxsDeformableSurfaceMaterialData* getCPUMaterialData()
	    {
		    return reinterpret_cast<PxsDeformableSurfaceMaterialData*>(mHostMaterial.begin());
	    }
	};

	class PxgPBDMaterialManager : public PxgMaterialManager
	{
	public:
		PxgPBDMaterialManager(PxgHeapMemoryAllocatorManager* heapManager) :
			PxgMaterialManager(heapManager, sizeof(PxsPBDMaterialData))
		{
		}
	};

}
#endif
