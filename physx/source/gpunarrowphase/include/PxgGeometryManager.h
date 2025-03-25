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

#ifndef PXG_GEOMETRY_MANAGER_H
#define PXG_GEOMETRY_MANAGER_H

#include "foundation/PxPreprocessor.h"

#include "foundation/PxArray.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxSimpleTypes.h"

#include "PxgCopyManager.h"

#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdisabled-macro-expansion"
#endif
#include "cuda.h"
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif

// AD: PxgGeometryManager manages the CPU/GPU data transfers and the lifetime of collision geometries: Convex Hulls, Trimeshes, SDFs, Heightfields.

/*
	Short overview: 

	- geometries are added using the add* functions.
	- during add, device memory is allocated and indices are given to geometries. We need to allocate the device memory here despite not writing to it
	  because the pointer is uploaded to GPU together with the shapeInstance data, which happens before we perform the final copy of the geometry in scheduleCopyHtoD.
	- We also accumulate the pinned memory requirements for the staging buffer during add.
	- scheduleCopyHtoD is performing the bulk of the work:
	    - we allocate the proper amount of pinned memory
		- for each scheduled geometry, we lay it out in the pinned memory
		- we push the copy descriptor to the copy manager.

	- the pinned memory is deallocated and the queues reset in resetAfterMemCopyCompleted().

	Future things:
	- look into the sequences of device uploads: we might be able to defer device memory allocations and use pools if we can rearrange things such that
	  we don't need the device pointer upfront.
	- The pinned memory is contiguous, so the copy manager taking pairs of (src, dest) could potentially be replaced by a linear thing.
*/

namespace physx
{

class PxgHeapMemoryAllocatorManager;
class PxgHeapMemoryAllocator;
class PxCudaContext;

namespace Gu
{
    struct ConvexHullData;
    class TriangleMesh;
    struct HeightFieldData;
};

template<typename ArrayT>
class PxgFreeIndicesProvider
{
	PX_NOCOPY(PxgFreeIndicesProvider)
public:
	PxgFreeIndicesProvider(ArrayT& ref): mDataArray(ref) {}

	PxU32							getFreeIndex()
	{
		if (!mFreeIndices.empty())
		{
			return mFreeIndices.popBack();
		}
		else
		{
			PxU32 currSz = mDataArray.size();
			
			if (currSz + 1 > mDataArray.capacity())
			{
				mDataArray.reserve(currSz * 2);
			}

			mDataArray.resize(currSz + 1);

			return currSz;
		}
	}

	void setFreeIndex(PxU32 idx)
	{
		PX_ASSERT(idx < mDataArray.size());
		mDeferredFreeIndices.pushBack(idx);
	}

	void releaseFreeIndices()
	{
		for (PxU32 a = 0; a < mDeferredFreeIndices.size(); ++a)
		{
			mFreeIndices.pushBack(mDeferredFreeIndices[a]);
		}
		mDeferredFreeIndices.forceSize_Unsafe(0);
	}

private:
	PxArray<PxU32>				mDeferredFreeIndices;
	PxArray<PxU32>				mFreeIndices;
	ArrayT&						mDataArray;
};

struct PxgMeshTextureData
{
	CUarray cuArray;
	CUtexObject cuTexRef;

	CUarray cuArraySubgrids;
	CUtexObject cuTexRefSubgrids;
};

struct HullOrMeshData
{
	void*		mDeviceMemPointer;
	PxU32		mCopyDescIndex;

	static const PxU32 INVALID_COPY_DESC_INDEX = 0xFFffFFff;
};
class PxgGeometryManager;
// Holds all the data needed to upload a geometry to the GPU. Created when calling add* functions, read during scheduleCopyHtoD.
struct ScheduledCopyData
{
private:
	// AD this is local, not intended for widespread usage.
	struct UploadGeometryType
	{
		enum Enum
		{
			eTRIANGLEMESH = 1,
			eCONVEXHULL   = 2,
			eHEIGHTFIELD  = 3,
			eBOXHULL      = 4
		};
	};
public:
	PxgCopyManager::CopyDesc         mCopyDesc;            //!< Copy descriptor for upload to GPU. lives until copy completed.
	PxU32                            mHullOrTrimeshIdx;    //!< Index into mGeometrydata
	UploadGeometryType::Enum         mType;                //!< Geometry type: see UploadGeometryType. Needed for deferred pinned memory allocation.
	PxU32                            mNumPolyVertices;     //!< Number of polygon vertices in case this is a convex hull
	const void*                      mGeometryPtr;         //!< Pointer to CPU geometry data.
	friend class PxgGeometryManager;
};

class PxgGeometryManager
{
	PX_NOCOPY(PxgGeometryManager)
	using UploadGeometryType = ScheduledCopyData::UploadGeometryType;

public:
	PxgGeometryManager(PxgHeapMemoryAllocatorManager* heapMemoryManager);

	virtual ~PxgGeometryManager();

	PxU32							addHull(const Gu::ConvexHullData& hull);
	PxU32							addTriMesh(const Gu::TriangleMesh& triMesh);
	PxU32							addHeightfield(const Gu::HeightFieldData& hf);
	void							addBoxHull();

	void							removeGeometry(PxU32 idx);
	
	void							scheduleCopyHtoD(PxgCopyManager& copyMan, PxCudaContext& cudaContext, CUstream stream);
	CUdeviceptr						getGeometryDevPtrByIndex(PxU32 idx) const;
	CUdeviceptr						getBoxHullDevPtr() const;
	
	void							resetAfterMemcpyCompleted();
	
private:
	PxU32 							addGeometryInternal(PxU64 byteSize, const void* geomPtr, UploadGeometryType::Enum type, PxU32 numPolyVertices = 0);

	PxgHeapMemoryAllocator*									mDeviceMemoryAllocator; //device memory
	PxgHeapMemoryAllocator*									mPinnedMemoryAllocator; //mapped memory
	void*													mPinnedMemoryBasePtr;   //stores the pointer to the current pinned memory allocation.
	PxU64													mPinnedHostMemoryRequirements; // accumulates pinned memory requirements for copies.
	
	PxArray<ScheduledCopyData>								mScheduledCopies;       // copies in current step, cleared after scheduleCopyHtoD

	PxArray<HullOrMeshData>									mGeometryData;          //persistent geometry data, holds idx -> device pointer mapping
	PxgFreeIndicesProvider<PxArray<HullOrMeshData> >		mFreeGeometryIndices;   //freeIndex provider for mGeometry.
	
	PxU32													mBoxHullIdx;            //index into mGeometryData for hardcoded convex hull for box.

	PxHashMap<void*, PxgMeshTextureData>					mMeshToTextureMap;      //maps meshes to SDF textures for lifetime management
};
};

#endif
