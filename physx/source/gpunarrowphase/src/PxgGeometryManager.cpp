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

#include "PxgGeometryManager.h"

#include "PxsHeapMemoryAllocator.h"
#include "foundation/PxAssert.h"
#include "foundation/PxBasicTemplates.h"
#include "foundation/PxMemory.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"

#include "PxgCopyManager.h"
#include "PxgCudaUtils.h"
#include "PxgHeapMemAllocator.h"

#include "GuBV32.h"
#include "convex/GuConvexMesh.h"
#include "mesh/GuTriangleMesh.h"
#include "hf/GuHeightField.h"
#include "GuSDF.h"
#include "cudaNpCommon.h"

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include <vector_types.h>
#include <vector_functions.h>

// AD: PxgGeometryManager manages the CPU/GPU data transfers and the lifetime of collision geometries: Convex Hulls, Trimeshes, SDFs, Heightfields.

using namespace physx;
using namespace Gu;

// forward declarations of static functions
static PxU64 computeBoxHullByteSize();
static PxU64 computeHullByteSize(const ConvexHullData& hull, PxU32& numPolyVertices);
static PxU64 computeTriMeshByteSize(const TriangleMesh& triMesh);
static PxU64 computeHeightfieldByteSize(const HeightFieldData& hf);
static void layOutBoxHull(void* mem);
static void layOutHull(void* mem, const ConvexHullData& hull, PxU32 numPolyVertices);
static PxgMeshTextureData layOutTriMesh(void* mem, const TriangleMesh& triMesh, CUstream stream);
static void layOutHeightfield(void* mem, const HeightFieldData& hf);
template<typename T> static void createTextureObject(CUarray_format format, CUtexObject*& texture, CUarray& cuArray, PxU32 width, PxU32 height, PxU32 depth, const T* data, CUstream stream);

// PxgGeometryManager definitions
PxgGeometryManager::PxgGeometryManager(PxgHeapMemoryAllocatorManager* heapMemoryManager): 
	mDeviceMemoryAllocator(heapMemoryManager->mDeviceMemoryAllocators),
	mPinnedMemoryAllocator(static_cast<PxgHeapMemoryAllocator*>(heapMemoryManager->mMappedMemoryAllocators)),
	mPinnedMemoryBasePtr(NULL),
	mPinnedHostMemoryRequirements(0),
	mFreeGeometryIndices(mGeometryData),
	mBoxHullIdx(0xFFffFFff)
{
}

PxgGeometryManager::~PxgGeometryManager()
{
	if (mBoxHullIdx != 0xFFFFFFFF)
		mDeviceMemoryAllocator->deallocate(reinterpret_cast<void*>(getGeometryDevPtrByIndex(mBoxHullIdx)));

	if (mPinnedMemoryBasePtr)
	{
		mPinnedMemoryAllocator->deallocate(mPinnedMemoryBasePtr);
		mPinnedMemoryBasePtr = NULL;
	}
}

PxU32 PxgGeometryManager::addGeometryInternal(PxU64 byteSize, const void* geomPtr, UploadGeometryType::Enum type, PxU32 numPolyVertices /*= 0*/)
{
	byteSize = (byteSize + 255) & ~255;

	mPinnedHostMemoryRequirements += byteSize;

	PxU32 idx = mFreeGeometryIndices.getFreeIndex();
	void* devicePtr = mDeviceMemoryAllocator->allocate(byteSize, PxsHeapStats::eNARROWPHASE, PX_FL);

	PxU32 copyIndex = mScheduledCopies.size();

	ScheduledCopyData scheduledCopy;
	scheduledCopy.mHullOrTrimeshIdx = idx;
	scheduledCopy.mGeometryPtr = geomPtr;
	scheduledCopy.mType = type;
	scheduledCopy.mNumPolyVertices = numPolyVertices;

	PxgCopyManager::CopyDesc desc;
	desc.dest = (size_t)devicePtr;
	desc.bytes = (size_t)byteSize;

	scheduledCopy.mCopyDesc = desc;
	mScheduledCopies.pushBack(scheduledCopy);
	
	HullOrMeshData newHullOrMesh;
	newHullOrMesh.mDeviceMemPointer = devicePtr;
	newHullOrMesh.mCopyDescIndex = copyIndex;

	PX_ASSERT(idx < mGeometryData.size());
	mGeometryData[idx] = newHullOrMesh;

	return idx;
}

void PxgGeometryManager::addBoxHull()
{
	if (mBoxHullIdx != 0xFFffFFff)
		return;

	PxU64 byteSize = computeBoxHullByteSize();

	mBoxHullIdx = addGeometryInternal(byteSize, NULL, UploadGeometryType::eBOXHULL);
}

PxU32 PxgGeometryManager::addHull(const ConvexHullData& hull)
{
	PxU32 numPolyVertices;
	PxU64 byteSize = computeHullByteSize(hull, numPolyVertices);

	return addGeometryInternal(byteSize, &hull, UploadGeometryType::eCONVEXHULL, numPolyVertices);
}

void PxgGeometryManager::removeGeometry(PxU32 idx)
{
	PX_ASSERT(idx < mGeometryData.size());
	HullOrMeshData geometryToRemove = mGeometryData[idx];

	PxU32 scheduledCopyIndex = geometryToRemove.mCopyDescIndex;

	if (scheduledCopyIndex != HullOrMeshData::INVALID_COPY_DESC_INDEX)
	{
		PxU32 lastScheduledCopyHullOrTrimeshIndex = mScheduledCopies.back().mHullOrTrimeshIdx;

		if (lastScheduledCopyHullOrTrimeshIndex != idx)
		{
			mScheduledCopies.replaceWithLast(scheduledCopyIndex);

			PX_ASSERT(lastScheduledCopyHullOrTrimeshIndex < mGeometryData.size());
			mGeometryData[lastScheduledCopyHullOrTrimeshIndex].mCopyDescIndex = scheduledCopyIndex;
		}
		else
		{
			mScheduledCopies.popBack();
		}
	}

	const PxPair<void*const, PxgMeshTextureData>* pair = mMeshToTextureMap.find(geometryToRemove.mDeviceMemPointer);

	if (pair)
	{
		cuArrayDestroy(pair->second.cuArray);
		cuTexObjectDestroy(pair->second.cuTexRef);

		if (pair->second.cuArraySubgrids)
		{
			cuArrayDestroy(pair->second.cuArraySubgrids);
			cuTexObjectDestroy(pair->second.cuTexRefSubgrids);
		}

		mMeshToTextureMap.erase(geometryToRemove.mDeviceMemPointer);
	}

	mDeviceMemoryAllocator->deallocate(geometryToRemove.mDeviceMemPointer);
	mFreeGeometryIndices.setFreeIndex(idx);
}

void PxgGeometryManager::scheduleCopyHtoD(PxgCopyManager& copyMan, PxCudaContext& cudaContext, CUstream stream)
{
	// allocate the proper amount of pinned memory
	mPinnedMemoryBasePtr = mPinnedMemoryAllocator->allocate(mPinnedHostMemoryRequirements, PxsHeapStats::eNARROWPHASE, PX_FL);
	PxU8* basePtr = reinterpret_cast<PxU8*>(mPinnedMemoryBasePtr);

	for (PxArray<ScheduledCopyData>::Iterator it = mScheduledCopies.begin(), end = mScheduledCopies.end(); it != end; ++it)
	{
		PxU64 byteSize = it->mCopyDesc.bytes;

		PxU8* mem = basePtr;
		basePtr += byteSize;

		PX_ASSERT((byteSize & 255) == 0);
		PX_ASSERT(((size_t)mem & 255) == 0);

		// layout the geometry in pinned memory depending on type
		switch (it->mType)
		{
			case UploadGeometryType::eTRIANGLEMESH:
				{
					PxgMeshTextureData texData = layOutTriMesh(mem, *reinterpret_cast<const TriangleMesh*>(it->mGeometryPtr), stream);

					if (texData.cuArray)
					{
						PX_ASSERT(it->mHullOrTrimeshIdx < mGeometryData.size());
						mMeshToTextureMap.insert(mGeometryData[it->mHullOrTrimeshIdx].mDeviceMemPointer, texData);
					}
				}
				break;
			case UploadGeometryType::eCONVEXHULL:
				layOutHull(mem, *reinterpret_cast<const ConvexHullData*>(it->mGeometryPtr), it->mNumPolyVertices);
				break;
			case UploadGeometryType::eHEIGHTFIELD:
				layOutHeightfield(mem, *reinterpret_cast<const HeightFieldData*>(it->mGeometryPtr));
				break;
			case UploadGeometryType::eBOXHULL:
				layOutBoxHull(mem);
				break;
			default:
				PX_ALWAYS_ASSERT();
		}

		// set the source pointer in the copy descriptor.
		it->mCopyDesc.source = reinterpret_cast<size_t>(getMappedDevicePtr(&cudaContext, mem));

		// schedule the copies.
		PX_ASSERT(it->mHullOrTrimeshIdx < mGeometryData.size());
		mGeometryData[it->mHullOrTrimeshIdx].mCopyDescIndex = HullOrMeshData::INVALID_COPY_DESC_INDEX;	
		copyMan.pushDeferredHtoD(it->mCopyDesc);
	}

	mScheduledCopies.forceSize_Unsafe(0);
	mPinnedHostMemoryRequirements = 0;
}

void PxgGeometryManager::resetAfterMemcpyCompleted()
{
	// AD: we basically never send geometry each frame, and they're usually sized differently anyway, so let's give this memory back
	// to keep pinned memory usage under control.
	if (mPinnedMemoryBasePtr)
	{
		mPinnedMemoryAllocator->deallocate(mPinnedMemoryBasePtr);
		mPinnedMemoryBasePtr = NULL;
	}

	mFreeGeometryIndices.releaseFreeIndices();
}

CUdeviceptr PxgGeometryManager::getGeometryDevPtrByIndex(PxU32 idx) const
{
	PX_ASSERT(idx < mGeometryData.size());

	return reinterpret_cast<CUdeviceptr>(mGeometryData[idx].mDeviceMemPointer);
}

CUdeviceptr PxgGeometryManager::getBoxHullDevPtr() const
{
	PX_ASSERT(mBoxHullIdx != 0xFFffFFff);
	PX_ASSERT(mBoxHullIdx < mGeometryData.size());

	return reinterpret_cast<CUdeviceptr>(mGeometryData[mBoxHullIdx].mDeviceMemPointer);
}

PxU32 PxgGeometryManager::addTriMesh(const TriangleMesh& triMesh)
{
	PxU64 byteSize = computeTriMeshByteSize(triMesh);
	
	return addGeometryInternal(byteSize, &triMesh, UploadGeometryType::eTRIANGLEMESH);
}

PxU32 PxgGeometryManager::addHeightfield(const HeightFieldData& hf)
{
	PxU64 byteSize = computeHeightfieldByteSize(hf);

	return addGeometryInternal(byteSize, &hf, UploadGeometryType::eHEIGHTFIELD);
}


// static functions
static PxU64 computeBoxHullByteSize()
{
	PxU64 byteSize = sizeof(float4) + //center of mass
		sizeof(PxU32) +               // NbEdgesNbHullVerticesNbPolygons
		3 * sizeof(PxU32) +           //pad
		sizeof(float4) +              //extents
		sizeof(float4) * 8 +          //vertices
		sizeof(float4) * 6 +          //planes
		sizeof(PxU32) * 6 +           //vRef8NbVertsMinIndex0 
		sizeof(PxU16) * 2 * 12 +      //mVerticesByEdges16
		sizeof(PxU8) * 2 * 12 +       //mFacesByEdges8
		sizeof(PxU8) * 3 * 8 +        //mFacesByVertices8
		sizeof(PxU8) * 24;            //vertexData8

	return byteSize;
}

static PxU64 computeHullByteSize(const ConvexHullData& hull, PxU32& numPolyVertices)
{
	numPolyVertices = 0;

	for (PxU32 i = 0; i < hull.mNbPolygons; ++i)
	{
		numPolyVertices += hull.mPolygons[i].mNbVerts;
	}

	return
		sizeof(float4) + //center of mass

		sizeof(PxU32) + // NbEdgesNbHullVerticesNbPolygons
		3 * sizeof(PxU32) + //pad
		sizeof(float4) + //extents
		sizeof(float4) * hull.mNbHullVertices +//vertices
		sizeof(float4) * hull.mNbPolygons +//planes
		sizeof(PxU32) * hull.mNbPolygons +//vRef8NbVertsMinIndex0 
		sizeof(PxU16) * 2 * hull.mNbEdges +//mVerticesByEdges16
		sizeof(PxU8) * 2 * hull.mNbEdges +//mFacesByEdges8
		sizeof(PxU8) * 3 * hull.mNbHullVertices +//mFacesByVertices8
		sizeof(PxU8) * numPolyVertices;//vertexData8
}

static PxU64 computeTriMeshByteSize(const TriangleMesh& triMesh)
{
	const PxU32 numTris = triMesh.getNbTrianglesFast();
	const PxU32 numVerts = triMesh.getNbVerticesFast();

	PxU64 meshDataSize =
		  sizeof(uint4)									// (nbVerts, nbTris, meshAdjVerticiesTotal, nbBv32TreeNodes)
		+ sizeof(float4) * numVerts						// meshVerts
		+ sizeof(uint4)  * numTris						// meshTriIndices
		+ sizeof(uint4)  * numTris						// meshTriAdjacencies
		+ sizeof(PxU32)  * numTris						// gpu to cpu remap table
		+ sizeof(PxU32)  * numTris * 3					// vertex to triangle remap table
		+ sizeof(PxU32)  * (numVerts + 1)				// vertex to triangle offset table
		;

	// Make sure memory is always 16-byte aligned
	meshDataSize = (meshDataSize + 15) & ~15;

	meshDataSize += sizeof(uint4);									// (sdfDimX, sdfDimY, sdfDimZ, 0)

	const SDF& sdfData = triMesh.getSdfDataFast();
	const PxU32 numSdfs = sdfData.mNumSdfs;
	if (numSdfs > 0)
	{
		meshDataSize +=
			+ sizeof(float4)				// (meshLower.x, meshLower.y, meshLower.z, spacing)
			+ sizeof(uint4)			
			+ sizeof(CUtexObject)			// SDF texture object reference
			+ sizeof(CUtexObject)
			+ sizeof(PxU32) * sdfData.mNumStartSlots;
	}

	// Make sure memory is always 16-byte aligned
	meshDataSize = (meshDataSize + 15) & ~15;

	//ML: don't know whether we need to have local bound
	const PxU64 bv32Size = triMesh.mGRB_BV32Tree->mNbPackedNodes * sizeof(BV32DataPacked);

	return meshDataSize +  bv32Size;
}

static PxU64 computeHeightfieldByteSize(const HeightFieldData& hf)
{
	
	/* Height field height data is 16 bit signed integers, followed by triangle materials. 
		
		Each sample is 32 bits wide arranged as follows:
		
		1) First there is a 16 bit height value.
		2) Next, two one byte material indices, with the high bit of each byte reserved for special use.
		(so the material index is only 7 bits).
		The high bit of material0 is the tess-flag.
		The high bit of material1 is reserved for future use.
	*/

	return sizeof(PxU32) //rows
			+ sizeof(PxU32) //columns
			+ sizeof(PxU32) * hf.columns * hf.rows
			+ sizeof(PxU16); //PxHeightFieldFlags
}

static void layOutBoxHull(void* mem)
{
	const float4 vertices[8] = { make_float4(-1.f, 1.f, -1.f, 0.f),
		make_float4(1.f, 1.f, -1.f, 0.f),
		make_float4(1.f, -1.f, -1.f, 0.f),
		make_float4(-1.f, -1.f, -1.f, 0.f),
		make_float4(-1.f, -1.f, 1.f, 0.f),
		make_float4(-1.f, 1.f, 1.f, 0.f),
		make_float4(1.f, -1.f, 1.f, 0.f),
		make_float4(1.f, 1.f, 1.f, 0.f)
	};

	const float4 planes[6] = {make_float4(0.f, 0.0f, -1.0f, -1.0f),
						make_float4(-1.0f, 0.0f, 0.0f, -1.0f),
						make_float4(0.0f, -1.0f, 0.0f, -1.0f),
						make_float4(0.0f, 1.0f, 0.0f, -1.0f),
						make_float4(0.0f, 0.0f, 1.0f, -1.0f),
						make_float4(1.0f, 0.0f, 0.0f, -1.0f)};
						
	const PxU32 polyData[6] = {merge(0, 4, 4),
						merge(4, 4, 1),
						merge(8, 4, 0),
						merge(12, 4, 2),
						merge(16, 4, 0),
						merge(20, 4, 0)};

	const PxU16 vertsByEdges[24] = { 0, 1, 3, 0, 5, 0, 1, 2, 7, 1, 2, 3, 2, 6, 3, 4, 4, 5, 6, 4, 5, 7, 6, 7 };

	const PxU8 facesByEdges[24] = {
								0, 3, 
								0, 1, 
								1, 3, 
								0, 5, 

								3, 5, 
								0, 2, 
								2, 5, 
								1, 2, 

								1, 4, 
								2, 4, 
								3, 4, 
								4, 5 
							};

	const PxU8 facesByVerts[24] =	{
								0, 1, 3,
								0, 3, 5,
								0, 2, 5,
								0, 1, 2,

								1, 2, 4,
								1, 3, 4,
								2, 4, 5,
								3, 4, 5
							};

	const PxU8 polyIndices[24] = {0, 1, 2, 3,
							4, 5, 0, 3, 
							4, 3, 2, 6,	
							7, 1, 0, 5,	
							7, 5, 4, 6,	
							7, 6, 2, 1	
	};

	PxU8* m = reinterpret_cast<PxU8*>(mem);
	*((float4*)m) = make_float4(0.0f, 0.0f, 0.0f, 0.0f);//center of mass
	m += sizeof(float4);
	*((PxU32*)m) = merge(12, merge((PxU8) 8, (PxU8) 6));
	m += 4 * sizeof(PxU32);
	*((float4*)m) = make_float4(1.0f, //half-extents and the insphere radius
							1.0f, 
							1.0f, 
							1.0f);

	m += sizeof(float4);
	PxMemCopy(m, vertices, sizeof(float4) * 8);
	m += sizeof(float4) * 8;
	PxMemCopy(m, planes, sizeof(float4) * 6);
	m += sizeof(float4) * 6;
	PxMemCopy(m, polyData, sizeof(PxU32) * 6);
	m += sizeof(PxU32) * 6;
	PxMemCopy(m, vertsByEdges, sizeof(PxU16) * 2 * 12);
	m += sizeof(PxU16) * 2 * 12;
	PxMemCopy(m, facesByEdges, sizeof(PxU8) * 2 * 12);
	m += sizeof(PxU8) * 2 * 12;
	PxMemCopy(m, facesByVerts, sizeof(PxU8) * 3 * 8);
	m += sizeof(PxU8) * 3 * 8;
	PxMemCopy(m, polyIndices, sizeof(PxU8) * 24);
}

static void layOutHull(void* mem, const ConvexHullData& hull, PxU32 numPolyVertices)
{
	PxU8* m = (PxU8*) mem;
	*((float4*)m) = make_float4(hull.mCenterOfMass.x, hull.mCenterOfMass.y, hull.mCenterOfMass.z, 0.f);
	m +=sizeof(float4);
	*((PxU32*)m) = merge(hull.mNbEdges, merge(hull.mNbHullVertices, hull.mNbPolygons));
	m += 4 * sizeof(PxU32);
	*((float4*)m) = make_float4(hull.mInternal.mInternalExtents.x, 
							hull.mInternal.mInternalExtents.y, 
							hull.mInternal.mInternalExtents.z, 
							hull.mInternal.mInternalRadius);

	m += sizeof(float4);

	const PxVec3* verts = hull.getHullVertices();

	for (const PxVec3* end = verts + hull.mNbHullVertices; verts < end; ++verts)
	{
		*((float4*)m) = make_float4(verts->x, verts->y, verts->z, 0); 
		m += sizeof(float4);
	}

	const HullPolygonData* polys = hull.mPolygons;
	float4* planes = (float4*)m;
	PxU32* vRef8NbVertsMinIndex = (PxU32*) (m + sizeof(float4) * hull.mNbPolygons);

	for (const HullPolygonData* end = polys + hull.mNbPolygons; polys < end; ++polys)
	{
		*(planes++) = make_float4(polys->mPlane.n.x, polys->mPlane.n.y, polys->mPlane.n.z, polys->mPlane.d);
		*(vRef8NbVertsMinIndex++) = merge(polys->mVRef8, polys->mNbVerts, polys->mMinIndex); 
	}

	m += (sizeof(float4) + sizeof(PxU32)) * hull.mNbPolygons;

	PxMemCopy(m, hull.getVerticesByEdges16(), sizeof(PxU16) * 2*hull.mNbEdges);
	m += sizeof(PxU16) * 2*hull.mNbEdges;

	PxMemCopy(m, hull.getFacesByEdges8(), sizeof(PxU8) * 2*hull.mNbEdges);
	m += sizeof(PxU8) * 2*hull.mNbEdges;

	PxMemCopy(m, hull.getFacesByVertices8(), sizeof(PxU8) * 3*hull.mNbHullVertices);
	m += sizeof(PxU8) * 3*hull.mNbHullVertices;

	PxMemCopy(m, hull.getVertexData8(), sizeof(PxU8) * numPolyVertices);
	//m += sizeof(PxU8) * numPolyVertices;
}

static PxgMeshTextureData layOutTriMesh(void* mem, const TriangleMesh& triMesh, CUstream stream)
{
	const PxU32 numTris = triMesh.getNbTrianglesFast();
	const PxU32 numVerts = triMesh.getNbVerticesFast();

	BV32Tree* bv32Tree = triMesh.mGRB_BV32Tree;

	PxU8* m = (PxU8*) mem;
	*((uint4*)m) = make_uint4(triMesh.getNbVerticesFast(), triMesh.getNbTrianglesFast(), 0, bv32Tree->mNbPackedNodes);
	m += sizeof(uint4);

	//Midphase
	PxMemCopy(m, bv32Tree->mPackedNodes, sizeof(BV32DataPacked) * bv32Tree->mNbPackedNodes);
	m += sizeof(BV32DataPacked) * bv32Tree->mNbPackedNodes;

	PX_ASSERT(bv32Tree->mNbPackedNodes > 0);

	// Core: Mesh data
	//construct gpu friendly verts
	float4* grbVerts = reinterpret_cast<float4*>(m);
	const PxVec3* verts = triMesh.getVerticesFast();

	for (PxU32 i = 0; i < numVerts; ++i)
	{
		grbVerts[i].x = verts[i].x;
		grbVerts[i].y = verts[i].y;
		grbVerts[i].z = verts[i].z;
		grbVerts[i].w = 0.f;
	}

	m += numVerts * sizeof(float4);
	
	uint4* grbTriInd = reinterpret_cast<uint4*>(m);
	//copy triangle indices
	if (triMesh.has16BitIndices())
	{
		const PxU16* triInds = reinterpret_cast<PxU16*>(triMesh.mGRB_triIndices);
		for (PxU32 i = 0; i < numTris; ++i)
		{
			grbTriInd[i].x = triInds[3 * i + 0];
			grbTriInd[i].y = triInds[3 * i + 1];
			grbTriInd[i].z = triInds[3 * i + 2];
			grbTriInd[i].w = 0;
		}
	}
	else
	{
		const PxU32* triInds = reinterpret_cast<PxU32*>(triMesh.mGRB_triIndices);
		for (PxU32 i = 0; i < numTris; ++i)
		{
			grbTriInd[i].x = triInds[3 * i + 0];
			grbTriInd[i].y = triInds[3 * i + 1];
			grbTriInd[i].z = triInds[3 * i + 2];
			grbTriInd[i].w = 0;
		}
	}

	//PxMemCopy(m, triMesh->mGRB_triIndices, numTris * sizeof(uint4));
	m += numTris * sizeof(uint4);

	//Adjacencies data for contact gen
	PxMemCopy(m, triMesh.mGRB_triAdjacencies, numTris * sizeof(uint4));
	m += numTris * sizeof(uint4);

	//GPU to CPU remap table
	PxMemCopy(m, triMesh.mGRB_faceRemap, numTris * sizeof(PxU32));
	m += numTris * sizeof(PxU32);

	//GPU to CPU remap table
	PxMemCopy(m, triMesh.mAccumulatedTrianglesRef, numVerts * sizeof(PxU32));
	reinterpret_cast<PxU32*>(m)[numVerts] = triMesh.mNbTrianglesReferences;
	m += (numVerts + 1) * sizeof(PxU32);

	//GPU to CPU remap table
	PxMemCopy(m, triMesh.mTrianglesReferences, (numTris*3) * sizeof(PxU32));
	m += (numTris * 3) * sizeof(PxU32);

	// Make sure m is 16-byte aligned
	m = (PxU8*)(((size_t)m + 15) & ~15);

	// Copy sdf values
	const SDF& sdfData = triMesh.getSdfDataFast();

	*((uint4*)m) = make_uint4(sdfData.mDims.x, sdfData.mDims.y, sdfData.mDims.z, triMesh.getPreferSDFProjection() ? 1 : 0);
	m += sizeof(uint4);

	const PxU32	 numSdfs = sdfData.mNumSdfs;

	PxgMeshTextureData result;
	PxMemZero(&result, sizeof(result));
	if (numSdfs)
	{
		const PxVec3 meshLower = sdfData.mMeshLower;
		const PxReal spacing = sdfData.mSpacing;
		*((float4*)m) = make_float4(meshLower.x, meshLower.y, meshLower.z, spacing);
		m += sizeof(float4);

		*((uint4*)m) = make_uint4(sdfData.mSubgridSize, reinterpret_cast<const PxU32&>(sdfData.mSubgridsMinSdfValue), reinterpret_cast<const PxU32&>(sdfData.mSubgridsMaxSdfValue), sdfData.mNumStartSlots);
		m += sizeof(uint4);

		CUtexObject* pTexture = (CUtexObject*)m;
		m += sizeof(CUtexObject);

		CUtexObject* pTextureSubgrids = (CUtexObject*)m;
		m += sizeof(CUtexObject);

		if (sdfData.mSubgridSize > 0)
			PxMemCopy(m, sdfData.mSubgridStartSlots, sdfData.mNumStartSlots * sizeof(PxU32));
		m += sizeof(PxU32) * sdfData.mNumStartSlots;

		if (sdfData.mSubgridSize == 0)
		{
			CUarray cuArray;
			createTextureObject<PxReal>(CU_AD_FORMAT_FLOAT, pTexture, cuArray, sdfData.mDims.x, sdfData.mDims.y, sdfData.mDims.z, sdfData.mSdf, stream);

			result.cuArray = cuArray;
			result.cuTexRef = *pTexture;

			result.cuArraySubgrids = 0;
			result.cuTexRefSubgrids = 0;
		}
		else
		{
			PxU32 x = sdfData.mDims.x / sdfData.mSubgridSize;
			PxU32 y = sdfData.mDims.y / sdfData.mSubgridSize;
			PxU32 z = sdfData.mDims.z / sdfData.mSubgridSize;

			CUarray cuArray;
			createTextureObject<PxReal>(CU_AD_FORMAT_FLOAT, pTexture, cuArray, x + 1, y + 1, z + 1, sdfData.mSdf, stream);

			CUarray cuArraySubgrids = 0;
			switch(sdfData.mBytesPerSparsePixel)
			{
			case 1:
				createTextureObject<PxU8>(CU_AD_FORMAT_UNSIGNED_INT8, pTextureSubgrids, cuArraySubgrids,
					sdfData.mSdfSubgrids3DTexBlockDim.x * (sdfData.mSubgridSize + 1),
					sdfData.mSdfSubgrids3DTexBlockDim.y * (sdfData.mSubgridSize + 1),
					sdfData.mSdfSubgrids3DTexBlockDim.z * (sdfData.mSubgridSize + 1), sdfData.mSubgridSdf, stream);
				break;
			case 2:
				createTextureObject<PxU16>(CU_AD_FORMAT_UNSIGNED_INT16, pTextureSubgrids, cuArraySubgrids,
					sdfData.mSdfSubgrids3DTexBlockDim.x * (sdfData.mSubgridSize + 1),
					sdfData.mSdfSubgrids3DTexBlockDim.y * (sdfData.mSubgridSize + 1),
					sdfData.mSdfSubgrids3DTexBlockDim.z * (sdfData.mSubgridSize + 1), reinterpret_cast<PxU16*>(sdfData.mSubgridSdf), stream);
				break;
			case 4:
				createTextureObject<PxReal>(CU_AD_FORMAT_FLOAT, pTextureSubgrids, cuArraySubgrids,
					sdfData.mSdfSubgrids3DTexBlockDim.x * (sdfData.mSubgridSize + 1),
					sdfData.mSdfSubgrids3DTexBlockDim.y * (sdfData.mSubgridSize + 1),
					sdfData.mSdfSubgrids3DTexBlockDim.z * (sdfData.mSubgridSize + 1), reinterpret_cast<PxReal*>(sdfData.mSubgridSdf), stream);
				break;
			}

			result.cuArray = cuArray;
			result.cuTexRef = *pTexture;

			if (cuArraySubgrids)
			{
				result.cuArraySubgrids = cuArraySubgrids;
				result.cuTexRefSubgrids = *pTextureSubgrids;
			}
			else
			{
				result.cuArraySubgrids = 0;
				result.cuTexRefSubgrids = 0;
			}
		}
	}

	return result;
}

static void layOutHeightfield(void* mem, const HeightFieldData& hf)
{
	PxU8* m = (PxU8*) mem;

	PxU32 nbRows = hf.rows;
	PxU32 nbCols = hf.columns;

	*((PxU32* ) m) = nbRows;
	m += sizeof(PxU32);

	*((PxU32* ) m) = nbCols;
	m += sizeof(PxU32);

	PX_ASSERT(sizeof(PxU32) == sizeof(PxHeightFieldSample));
	PxMemCopy(m, hf.samples, sizeof(PxU32) * nbRows * nbCols);

	m += sizeof(PxU32) * nbRows * nbCols;

	*((PxU16*)m) = hf.flags;
}

template<typename T>
static void createTextureObject(CUarray_format format, CUtexObject*& texture, CUarray& cuArray, PxU32 width, PxU32 height, PxU32 depth, const T* data, CUstream stream)
{
	if (width == 0 || height == 0 || depth == 0)
	{
		texture = 0;
		cuArray = 0;
		return;
	}

	CUDA_ARRAY3D_DESCRIPTOR_st arrDesc;
	arrDesc.Format = format;
	arrDesc.NumChannels = 1;
	arrDesc.Width = width; 
	arrDesc.Height = height;
	arrDesc.Depth = depth; 
	arrDesc.Flags = 0;
	CUresult r = cuArray3DCreate(&cuArray, &arrDesc);
	PX_UNUSED(r);
	PX_ASSERT(r == CUDA_SUCCESS);

	CUDA_MEMCPY3D copyParam;
	PxMemZero(&copyParam, sizeof(copyParam));
	copyParam.dstMemoryType = CU_MEMORYTYPE_ARRAY;
	copyParam.dstArray = cuArray;
	copyParam.srcMemoryType = CU_MEMORYTYPE_HOST;
	copyParam.srcHost = data;
	copyParam.srcPitch = width * sizeof(T);
	copyParam.srcHeight = height;
	copyParam.WidthInBytes = copyParam.srcPitch;
	copyParam.Height = height; 
	copyParam.Depth = depth; 

	cuMemcpy3DAsync(&copyParam, stream);

	CUDA_RESOURCE_DESC resDesc;
	PxMemZero(&resDesc, sizeof(resDesc));
	resDesc.resType = CU_RESOURCE_TYPE_ARRAY;
	resDesc.res.array.hArray = cuArray;	

	CUDA_TEXTURE_DESC texDesc;
	PxMemZero(&texDesc, sizeof(texDesc));
	texDesc.addressMode[0] = CU_TR_ADDRESS_MODE_CLAMP;
	texDesc.addressMode[1] = CU_TR_ADDRESS_MODE_CLAMP;
	texDesc.addressMode[2] = CU_TR_ADDRESS_MODE_CLAMP;
	
	texDesc.filterMode = CU_TR_FILTER_MODE_LINEAR;
	
	r = cuTexObjectCreate(texture, &resDesc, &texDesc, NULL);
	PX_ASSERT(r == CUDA_SUCCESS);
}
