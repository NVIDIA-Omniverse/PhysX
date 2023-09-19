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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "common/PxInsertionCallback.h"
#include "GuCooking.h"
#include "GuMeshFactory.h"
#include "GuTriangleMeshBV4.h"
#include "GuTriangleMeshRTree.h"
#include "GuTetrahedronMesh.h"
#include "GuConvexMesh.h"
#include "GuBVH.h"
#include "GuHeightField.h"

#if PX_SUPPORT_OMNI_PVD
#	define OMNI_PVD_NOTIFY_ADD(OBJECT) notifyListenersAdd(OBJECT)
#	define OMNI_PVD_NOTIFY_REMOVE(OBJECT) notifyListenersRemove(OBJECT)
#else
#	define OMNI_PVD_NOTIFY_ADD(OBJECT)
#	define OMNI_PVD_NOTIFY_REMOVE(OBJECT)
#endif

using namespace physx;
using namespace Gu;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

// PT: TODO: refactor all this with a dedicated container

MeshFactory::MeshFactory() :
	mTriangleMeshes		("mesh factory triangle mesh hash"),
	mConvexMeshes		("mesh factory convex mesh hash"),
	mHeightFields		("mesh factory height field hash"),
    mBVHs				("BVH factory hash"),
	mFactoryListeners	("FactoryListeners")
{
}

MeshFactory::~MeshFactory()
{
}

///////////////////////////////////////////////////////////////////////////////

template<class T>
static void releaseObjects(PxCoalescedHashSet<T*>& objects)
{
	while(objects.size())
	{
		T* object = objects.getEntries()[0];
		PX_ASSERT(RefCountable_getRefCount(*object)==1);
		object->release();
	}
}

// PT: needed because Gu::BVH is not a PxRefCounted object, although it derives from RefCountable
static void releaseObjects(PxCoalescedHashSet<Gu::BVH*>& objects)
{
	while(objects.size())
	{
		Gu::BVH* object = objects.getEntries()[0];
		PX_ASSERT(object->getRefCount()==1);
		object->release();
	}
}

void MeshFactory::release()
{
	// Release all objects in case the user didn't do it
	releaseObjects(mTriangleMeshes);
	releaseObjects(mTetrahedronMeshes);
	releaseObjects(mSoftBodyMeshes);
	releaseObjects(mConvexMeshes);
	releaseObjects(mHeightFields);
	releaseObjects(mBVHs);

	PX_DELETE_THIS;
}

template <typename T>
static void addToHash(PxCoalescedHashSet<T*>& hash, T* element, PxMutex* mutex)
{
	if(!element)
		return;

	if(mutex)
		mutex->lock();

	hash.insert(element);

	if(mutex)
		mutex->unlock();
}

///////////////////////////////////////////////////////////////////////////////

static void read8BitIndices(PxInputStream& stream, void* tris, PxU32 nbIndices, const bool has16BitIndices)
{
	PxU8 x;
	if(has16BitIndices)
	{
		PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
		for(PxU32 i=0;i<nbIndices;i++)
		{
			stream.read(&x, sizeof(PxU8));
			*tris16++ = x;
		}
	}
	else
	{
		PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
		for(PxU32 i=0;i<nbIndices;i++)
		{
			stream.read(&x, sizeof(PxU8));
			*tris32++ = x;
		}
	}
}

static void read16BitIndices(PxInputStream& stream, void* tris, PxU32 nbIndices, const bool has16BitIndices, const bool mismatch)
{
	if(has16BitIndices)
	{
		PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
		stream.read(tris16, nbIndices*sizeof(PxU16));
		if(mismatch)
		{
			for(PxU32 i=0;i<nbIndices;i++)
				flip(tris16[i]);
		}
	}
	else
	{
		PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
		PxU16 x;
		for(PxU32 i=0;i<nbIndices;i++)
		{
			stream.read(&x, sizeof(PxU16));
			if(mismatch)
				flip(x);

			*tris32++ = x;
		}
	}
}

static void read32BitIndices(PxInputStream& stream, void* tris, PxU32 nbIndices, const bool has16BitIndices, const bool mismatch)
{
	if(has16BitIndices)
	{
		PxU32 x;
		PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
		for(PxU32 i=0;i<nbIndices;i++)
		{
			stream.read(&x, sizeof(PxU32));
			if(mismatch)
				flip(x);
			*tris16++ = PxTo16(x);
		}
	}
	else
	{
		PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
		stream.read(tris32, nbIndices*sizeof(PxU32));

		if(mismatch)
		{
			for(PxU32 i=0;i<nbIndices;i++)
					flip(tris32[i]);
		}
	}
}

static TriangleMeshData* loadMeshData(PxInputStream& stream)
{
	// Import header
	PxU32 version;
	bool mismatch;
	if(!readHeader('M', 'E', 'S', 'H', version, mismatch, stream))
		return NULL;

	PxU32 midphaseID = PxMeshMidPhase::eBVH33;	// Default before version 14
	if(version>=14)	// this refers to PX_MESH_VERSION
		midphaseID = readDword(mismatch, stream);

	// Check if old (incompatible) mesh format is loaded
	if (version <= 9) // this refers to PX_MESH_VERSION
	{
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Loading triangle mesh failed: "
			"Deprecated mesh cooking format. Please recook your mesh in a new cooking format.");
		PX_ALWAYS_ASSERT_MESSAGE("Obsolete cooked mesh found. Mesh version has been updated, please recook your meshes.");
		return NULL;
	}

	// Import serialization flags
	const PxU32 serialFlags = readDword(mismatch, stream);

	// Import misc values	
	if (version <= 12) // this refers to PX_MESH_VERSION
	{
		// convexEdgeThreshold was removed in 3.4.0
		readFloat(mismatch, stream);		
	}

	TriangleMeshData* data;
	if(midphaseID==PxMeshMidPhase::eBVH33)
		data = PX_NEW(RTreeTriangleData);
	else if(midphaseID==PxMeshMidPhase::eBVH34)
		data = PX_NEW(BV4TriangleData);
	else return NULL;

	// Import mesh
	PxVec3* verts = data->allocateVertices(readDword(mismatch, stream));
	const PxU32 nbTris = readDword(mismatch, stream);
	const bool force32 = (serialFlags & (IMSF_8BIT_INDICES|IMSF_16BIT_INDICES)) == 0;

	//ML: this will allocate CPU triangle indices and GPU triangle indices if we have GRB data built
	void* tris = data->allocateTriangles(nbTris, force32, serialFlags & IMSF_GRB_DATA);

	stream.read(verts, sizeof(PxVec3)*data->mNbVertices);
	if(mismatch)
	{
		for(PxU32 i=0;i<data->mNbVertices;i++)
		{
			flip(verts[i].x);
			flip(verts[i].y);
			flip(verts[i].z);
		}
	}
	//TODO: stop support for format conversion on load!!
	const PxU32 nbIndices = 3*data->mNbTriangles;
	if(serialFlags & IMSF_8BIT_INDICES)
		read8BitIndices(stream, tris, nbIndices, data->has16BitIndices());
	else if(serialFlags & IMSF_16BIT_INDICES)
		read16BitIndices(stream, tris, nbIndices, data->has16BitIndices(), mismatch);
	else
		read32BitIndices(stream, tris, nbIndices, data->has16BitIndices(), mismatch);

	if(serialFlags & IMSF_MATERIALS)
	{
		PxU16* materials = data->allocateMaterials();
		stream.read(materials, sizeof(PxU16)*data->mNbTriangles);
		if(mismatch)
		{
			for(PxU32 i=0;i<data->mNbTriangles;i++)
				flip(materials[i]);
		}
	}
	if(serialFlags & IMSF_FACE_REMAP)
	{
		PxU32* remap = data->allocateFaceRemap();
		readIndices(readDword(mismatch, stream), data->mNbTriangles, remap, stream, mismatch);
	}

	if(serialFlags & IMSF_ADJACENCIES)
	{
		PxU32* adj = data->allocateAdjacencies();
		stream.read(adj, sizeof(PxU32)*data->mNbTriangles*3);
		if(mismatch)
		{
			for(PxU32 i=0;i<data->mNbTriangles*3;i++)
				flip(adj[i]);
		}		
	}

	// PT: TODO better
	if(midphaseID==PxMeshMidPhase::eBVH33)
	{
		if(!static_cast<RTreeTriangleData*>(data)->mRTree.load(stream, version, mismatch))
		{
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "RTree binary image load error.");
			PX_DELETE(data);
			return NULL;
		}
	}
	else if(midphaseID==PxMeshMidPhase::eBVH34)
	{
		BV4TriangleData* bv4data = static_cast<BV4TriangleData*>(data);
		if(!bv4data->mBV4Tree.load(stream, mismatch))
		{
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "BV4 binary image load error.");
			PX_DELETE(data);
			return NULL;
		}

		bv4data->mMeshInterface.setNbTriangles(nbTris);
		bv4data->mMeshInterface.setNbVertices(data->mNbVertices);
		if(data->has16BitIndices())
			bv4data->mMeshInterface.setPointers(NULL, reinterpret_cast<IndTri16*>(tris), verts);
		else
			bv4data->mMeshInterface.setPointers(reinterpret_cast<IndTri32*>(tris), NULL, verts);
		bv4data->mBV4Tree.mMeshInterface = &bv4data->mMeshInterface;
	}
	else PX_ASSERT(0);

	// Import local bounds
	data->mGeomEpsilon = readFloat(mismatch, stream);
	readFloatBuffer(&data->mAABB.minimum.x, 6, mismatch, stream);

	PxU32 nb = readDword(mismatch, stream);
	if(nb)
	{
		PX_ASSERT(nb==data->mNbTriangles);
		data->allocateExtraTrigData();
		// No need to convert those bytes
		stream.read(data->mExtraTrigData, nb*sizeof(PxU8));
	}

	if(serialFlags & IMSF_GRB_DATA)
	{
		PxU32 GRB_meshAdjVerticiesTotal = 0;
		if(version < 15)
			GRB_meshAdjVerticiesTotal = readDword(mismatch, stream);

		//read grb triangle indices
		PX_ASSERT(data->mGRB_primIndices);

		if(serialFlags & IMSF_8BIT_INDICES)
			read8BitIndices(stream, data->mGRB_primIndices, nbIndices, data->has16BitIndices());
		else if(serialFlags & IMSF_16BIT_INDICES)
			read16BitIndices(stream, data->mGRB_primIndices, nbIndices, data->has16BitIndices(), mismatch);
		else
			read32BitIndices(stream, data->mGRB_primIndices, nbIndices, data->has16BitIndices(), mismatch);

		data->mGRB_primAdjacencies = PX_ALLOCATE(PxU32, data->mNbTriangles*4, "mGRB_primAdjacencies");
		data->mGRB_faceRemap = PX_ALLOCATE(PxU32, data->mNbTriangles, "mGRB_faceRemap");

		if(serialFlags & IMSF_GRB_INV_REMAP)
			data->mGRB_faceRemapInverse = PX_ALLOCATE(PxU32, data->mNbTriangles, "mGRB_faceRemapInverse");

		stream.read(data->mGRB_primAdjacencies, sizeof(PxU32)*data->mNbTriangles*4);
		if (version < 15)
		{
			//stream.read(data->mGRB_vertValency, sizeof(PxU32)*data->mNbVertices);
			for (PxU32 i = 0; i < data->mNbVertices; ++i)
				readDword(mismatch, stream);
			//stream.read(data->mGRB_adjVertStart, sizeof(PxU32)*data->mNbVertices);
			for (PxU32 i = 0; i < data->mNbVertices; ++i)
				readDword(mismatch, stream);
			//stream.read(data->mGRB_adjVertices, sizeof(PxU32)*GRB_meshAdjVerticiesTotal);
			for (PxU32 i = 0; i < GRB_meshAdjVerticiesTotal; ++i)
				readDword(mismatch, stream);
		}
		stream.read(data->mGRB_faceRemap, sizeof(PxU32)*data->mNbTriangles);
		if(data->mGRB_faceRemapInverse)
			stream.read(data->mGRB_faceRemapInverse, sizeof(PxU32)*data->mNbTriangles);

		if(mismatch)
		{
			for(PxU32 i=0;i<data->mNbTriangles*4;i++)
				flip(reinterpret_cast<PxU32 *>(data->mGRB_primIndices)[i]);

			for(PxU32 i=0;i<data->mNbTriangles*4;i++)
				flip(reinterpret_cast<PxU32 *>(data->mGRB_primAdjacencies)[i]);
		}

		//read BV32
		data->mGRB_BV32Tree = PX_NEW(BV32Tree);
		if (!data->mGRB_BV32Tree->load(stream, mismatch))
		{
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "BV32 binary image load error.");
			PX_DELETE(data);
			return NULL;
		}

		if (serialFlags & IMSF_VERT_MAPPING)
		{
			//import vertex mapping data
			data->mNbTrianglesReferences = readDword(mismatch, stream);
			data->mAccumulatedTrianglesRef = PX_ALLOCATE(PxU32, data->mNbVertices, "mAccumulatedTrianglesRef");
			data->mTrianglesReferences = PX_ALLOCATE(PxU32, data->mNbTrianglesReferences, "mTrianglesReferences");

			stream.read(data->mAccumulatedTrianglesRef, data->mNbVertices * sizeof(PxU32));
			stream.read(data->mTrianglesReferences, data->mNbTrianglesReferences * sizeof(PxU32));
		}
	}

	if (serialFlags & IMSF_SDF)
	{
		// Import sdf
		SDF& sdfData = data->mSdfData;
		sdfData.mMeshLower.x = readFloat(mismatch, stream);
		sdfData.mMeshLower.y = readFloat(mismatch, stream);
		sdfData.mMeshLower.z = readFloat(mismatch, stream);
		sdfData.mSpacing = readFloat(mismatch, stream);
		sdfData.mDims.x = readDword(mismatch, stream);
		sdfData.mDims.y = readDword(mismatch, stream);
		sdfData.mDims.z = readDword(mismatch, stream);
		sdfData.mNumSdfs = readDword(mismatch, stream);
		
		sdfData.mNumSubgridSdfs = readDword(mismatch, stream);
		sdfData.mNumStartSlots = readDword(mismatch, stream);
		sdfData.mSubgridSize = readDword(mismatch, stream);
		sdfData.mSdfSubgrids3DTexBlockDim.x = readDword(mismatch, stream);
		sdfData.mSdfSubgrids3DTexBlockDim.y = readDword(mismatch, stream);
		sdfData.mSdfSubgrids3DTexBlockDim.z = readDword(mismatch, stream);
		
		sdfData.mSubgridsMinSdfValue = readFloat(mismatch, stream);
		sdfData.mSubgridsMaxSdfValue = readFloat(mismatch, stream);
		sdfData.mBytesPerSparsePixel = readDword(mismatch, stream);

		PxReal* sdf = sdfData.allocateSdfs(sdfData.mMeshLower, sdfData.mSpacing, sdfData.mDims.x, sdfData.mDims.y, sdfData.mDims.z,
			sdfData.mSubgridSize, sdfData.mSdfSubgrids3DTexBlockDim.x, sdfData.mSdfSubgrids3DTexBlockDim.y, sdfData.mSdfSubgrids3DTexBlockDim.z,
			sdfData.mSubgridsMinSdfValue, sdfData.mSubgridsMaxSdfValue, sdfData.mBytesPerSparsePixel);

		stream.read(sdf, sizeof(PxReal) * sdfData.mNumSdfs);
		readByteBuffer(sdfData.mSubgridSdf, sdfData.mNumSubgridSdfs, stream);
		readIntBuffer(sdfData.mSubgridStartSlots, sdfData.mNumStartSlots, mismatch, stream);		
	}

	if (serialFlags & IMSF_INERTIA)
	{
		// Import inertia
		stream.read(&data->mMass, sizeof(PxReal));
		readFloatBuffer(&data->mInertia(0, 0), 9, mismatch, stream);
		readFloatBuffer(&data->mLocalCenterOfMass.x, 3, mismatch, stream);
	}

	return data;
}

static void readIndices(const PxU32 serialFlags, void* indices, const PxU32 nbIndices,
	const bool has16BitIndices, const bool mismatch, PxInputStream& stream)
{
	if(serialFlags & IMSF_8BIT_INDICES)
		read8BitIndices(stream, indices, nbIndices, has16BitIndices);
	else if(serialFlags & IMSF_16BIT_INDICES)
		read16BitIndices(stream, indices, nbIndices, has16BitIndices, mismatch);
	else
		read32BitIndices(stream, indices, nbIndices, has16BitIndices, mismatch);
}

void MeshFactory::addTriangleMesh(TriangleMesh* np, bool lock)
{
	addToHash(mTriangleMeshes, np, lock ? &mTrackingMutex : NULL);
	OMNI_PVD_NOTIFY_ADD(np);
}

PxTriangleMesh* MeshFactory::createTriangleMesh(TriangleMeshData& data)
{	
	TriangleMesh* np;

	if(data.mType==PxMeshMidPhase::eBVH33)
	{
		PX_NEW_SERIALIZED(np, RTreeTriangleMesh)(this, data);
	}
	else if(data.mType==PxMeshMidPhase::eBVH34)
	{
		PX_NEW_SERIALIZED(np, BV4TriangleMesh)(this, data);
	}
	else return NULL;

	if(np)
		addTriangleMesh(np);

	return np;
}

// data injected by cooking lib for runtime cooking
PxTriangleMesh* MeshFactory::createTriangleMesh(void* data)
{	
	return createTriangleMesh(*reinterpret_cast<TriangleMeshData*>(data));
}

PxTriangleMesh* MeshFactory::createTriangleMesh(PxInputStream& desc)
{	
	TriangleMeshData* data = ::loadMeshData(desc);
	if(!data)
		return NULL;
	PxTriangleMesh* m = createTriangleMesh(*data);
	PX_DELETE(data);
	return m;
}

bool MeshFactory::removeTriangleMesh(PxTriangleMesh& m)
{
	TriangleMesh* gu = static_cast<TriangleMesh*>(&m);
	OMNI_PVD_NOTIFY_REMOVE(gu);
	PxMutex::ScopedLock lock(mTrackingMutex);
	bool found = mTriangleMeshes.erase(gu);
	return found;
}

PxU32 MeshFactory::getNbTriangleMeshes() const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return mTriangleMeshes.size();
}

PxU32 MeshFactory::getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mTriangleMeshes.getEntries(), mTriangleMeshes.size());
}

///////////////////////////////////////////////////////////////////////////////

static TetrahedronMeshData* loadTetrahedronMeshData(PxInputStream& stream)
{
	// Import header
	PxU32 version;
	bool mismatch;

	if (!readHeader('T', 'E', 'M', 'E', version, mismatch, stream))
		return NULL;

	// Import serialization flags
	const PxU32 serialFlags = readDword(mismatch, stream);

	TetrahedronMeshData* data = PX_NEW(TetrahedronMeshData);

	// Import mesh
	const PxU32 nbVerts = readDword(mismatch, stream);
	PxVec3* verts = data->allocateVertices(nbVerts);

	//const PxU32 nbSurfaceTriangles = readDword(mismatch, stream);

	const PxU32 nbTetrahedrons = readDword(mismatch, stream);

	//ML: this will allocate CPU tetrahedron indices and GPU tetrahedron indices and other GPU data if we have GRB data built
	//void* tets = data->allocateTetrahedrons(nbTetrahedrons, serialFlags & IMSF_GRB_DATA);
	data->allocateTetrahedrons(nbTetrahedrons, 1);	
	void* tets = data->mTetrahedrons;
	

	stream.read(verts, sizeof(PxVec3)*data->mNbVertices);
	//stream.read(restPoses, sizeof(PxMat33) * data->mNbTetrahedrons);

	if (mismatch)
	{
		for (PxU32 i = 0; i < data->mNbVertices; i++)
		{
			flip(verts[i].x);
			flip(verts[i].y);
			flip(verts[i].z);
		}
	}

	//TODO: stop support for format conversion on load!!
	const PxU32 nbTetIndices = 4 * data->mNbTetrahedrons;
	readIndices(serialFlags, tets, nbTetIndices, data->has16BitIndices(), mismatch, stream);
		
	// Import local bounds
	data->mGeomEpsilon = readFloat(mismatch, stream);
	readFloatBuffer(&data->mAABB.minimum.x, 6, mismatch, stream);

	return data;
}

static bool loadSoftBodyMeshData(PxInputStream& stream, SoftBodyMeshData& data)
{
	// Import header
	PxU32 version;
	bool mismatch;
	
	if (!readHeader('S', 'O', 'M', 'E', version, mismatch, stream))
		return false;

	// Import serialization flags
	const PxU32 serialFlags = readDword(mismatch, stream);

	// Import mesh
	const PxU32 nbVerts = readDword(mismatch, stream);
	PxVec3* verts = data.mCollisionMesh.allocateVertices(nbVerts);

	//const PxU32 nbSurfaceTriangles = readDword(mismatch, stream);

	const PxU32 nbTetrahedrons= readDword(mismatch, stream);
	
	//ML: this will allocate CPU tetrahedron indices and GPU tetrahedron indices and other GPU data if we have GRB data built
	//void* tets = data.allocateTetrahedrons(nbTetrahedrons, serialFlags & IMSF_GRB_DATA);
	data.mCollisionMesh.allocateTetrahedrons(nbTetrahedrons, 1);
	if (serialFlags & IMSF_GRB_DATA)
		data.mCollisionData.allocateCollisionData(nbTetrahedrons);
	void* tets = data.mCollisionMesh.mTetrahedrons;
	//void* surfaceTriangles = data.mCollisionData.allocateSurfaceTriangles(nbSurfaceTriangles);
	
	//void* restPoses = data.mTetraRestPoses;

	stream.read(verts, sizeof(PxVec3)*nbVerts);
	//stream.read(restPoses, sizeof(PxMat33) * data.mNbTetrahedrons);

	if (mismatch)
	{
		for (PxU32 i = 0; i< nbVerts; i++)
		{
			flip(verts[i].x);
			flip(verts[i].y);
			flip(verts[i].z);
		}
	}

	//TODO: stop support for format conversion on load!!
	const PxU32 nbTetIndices = 4 * nbTetrahedrons;
	readIndices(serialFlags, tets, nbTetIndices, data.mCollisionMesh.has16BitIndices(), mismatch, stream);
	
	//const PxU32 nbSurfaceTriangleIndices = 3 * nbSurfaceTriangles;
	//readIndices(serialFlags, surfaceTriangles, nbSurfaceTriangleIndices, data.mCollisionMesh.has16BitIndices(), mismatch, stream);

	////using IMSF_ADJACENCIES for tetMesh tetrahedron surface hint
	//if (serialFlags & IMSF_ADJACENCIES)
	//{
	//	PxU8* surfaceHints = reinterpret_cast<PxU8*>(data.mTetraSurfaceHint);
	//	stream.read(surfaceHints, sizeof(PxU8)*data.mNbTetrahedrons);
	//}

	if (serialFlags & IMSF_MATERIALS)
	{
		PxU16* materials = data.mCollisionMesh.allocateMaterials();
		stream.read(materials, sizeof(PxU16)*nbTetrahedrons);
		if (mismatch)
		{
			for (PxU32 i = 0; i < nbTetrahedrons; i++)
				flip(materials[i]);
		}
	}

	if (serialFlags & IMSF_FACE_REMAP)
	{
		PxU32* remap = data.mCollisionData.allocateFaceRemap(nbTetrahedrons);
		readIndices(readDword(mismatch, stream), nbTetrahedrons, remap, stream, mismatch);
	}

	/*if (serialFlags & IMSF_ADJACENCIES)
	{
		PxU32* adj = data.allocateAdjacencies();
		stream.read(adj, sizeof(PxU32)*data.mNbTetrahedrons * 4);
		if (mismatch)
		{
			for (PxU32 i = 0; i<data.mNbTetrahedrons * 4; i++)
				flip(adj[i]);
		}
	}*/
	
	SoftBodyMeshData* bv4data = &data;
	if (!bv4data->mCollisionData.mBV4Tree.load(stream, mismatch))
	{
		outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "BV4 binary image load error.");
		//PX_DELETE(data);
		return false;
	}

	bv4data->mCollisionData.mMeshInterface.setNbTetrahedrons(nbTetrahedrons);
	bv4data->mCollisionData.mMeshInterface.setNbVertices(nbVerts);
	if (data.mCollisionMesh.has16BitIndices())
		bv4data->mCollisionData.mMeshInterface.setPointers(NULL, reinterpret_cast<IndTetrahedron16*>(tets), verts);
	else
		bv4data->mCollisionData.mMeshInterface.setPointers(reinterpret_cast<IndTetrahedron32*>(tets), NULL, verts);
	
	bv4data->mCollisionData.mBV4Tree.mMeshInterface = &bv4data->mCollisionData.mMeshInterface;
	
	// Import local bounds
	data.mCollisionMesh.mGeomEpsilon = readFloat(mismatch, stream);
	readFloatBuffer(&data.mCollisionMesh.mAABB.minimum.x, 6, mismatch, stream);

	if (serialFlags & IMSF_GRB_DATA)
	{
		/*PxU32 GRB_meshAdjVerticiesTotal = 0;
		if (version < 15)
			GRB_meshAdjVerticiesTotal = readDword(mismatch, stream);*/

		//read grb tetrahedron indices
		PX_ASSERT(data.mCollisionData.mGRB_primIndices);

		//read tetrahedron indices
		readIndices(serialFlags, data.mCollisionData.mGRB_primIndices, nbTetIndices, data.mCollisionMesh.has16BitIndices(), mismatch, stream);
		
		//data.mGRB_primAdjacencies = static_cast<void *>(PX_NEW(PxU32)[data.mNbTetrahedrons * 4]);
		
		//data.mGRB_surfaceTriIndices = static_cast<void *>(PX_NEW(PxU32)[data.mNbTriangles * 3]);
		data.mCollisionData.mGRB_faceRemap = PX_ALLOCATE(PxU32, data.mCollisionMesh.mNbTetrahedrons, "mGRB_faceRemap");

		data.mCollisionData.mGRB_faceRemapInverse = PX_ALLOCATE(PxU32, data.mCollisionMesh.mNbTetrahedrons, "mGRB_faceRemapInverse");

		//data.mGRB_surfaceTriangleIndice = PX_NEW(PxU32)[data.mNbSurfaceTriangles * 3];

		//stream.read(data.mGRB_primAdjacencies, sizeof(PxU32)*data.mNbTetrahedrons * 4);		
		stream.read(data.mCollisionData.mGRB_tetraSurfaceHint, sizeof(PxU8) * data.mCollisionMesh.mNbTetrahedrons);
		stream.read(data.mCollisionData.mGRB_faceRemap, sizeof(PxU32) * data.mCollisionMesh.mNbTetrahedrons);
		stream.read(data.mCollisionData.mGRB_faceRemapInverse, sizeof(PxU32) * data.mCollisionMesh.mNbTetrahedrons);
		//stream.read(data.mGRB_surfaceTriangleIndice, sizeof(PxU32) * data.mNbSurfaceTriangles * 3);
		
		stream.read(data.mCollisionData.mTetraRestPoses, sizeof(PxMat33) * nbTetrahedrons);

		if (mismatch)
		{
			for (PxU32 i = 0; i<data.mCollisionMesh.mNbTetrahedrons * 4; i++)
				flip(reinterpret_cast<PxU32 *>(data.mCollisionData.mGRB_primIndices)[i]);
		}

		//read BV32
		data.mCollisionData.mGRB_BV32Tree = PX_NEW(BV32Tree);
		if (!data.mCollisionData.mGRB_BV32Tree->load(stream, mismatch))
		{
			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "BV32 binary image load error.");
			//PX_DELETE(data);
			return false;
		}

		const PxU32 nbGridModelTetrahedrons = readDword(mismatch, stream);
		const PxU32 nbGridModelVertices = readDword(mismatch, stream);
		const PxU32 nbGridModelPartitions = readDword(mismatch, stream);
		const PxU32 nbGMMaxTetsPerPartition = readDword(mismatch, stream);
		const PxU32 nbGMRemapOutputSize = readDword(mismatch, stream);
		PxU32 numTetsPerElement = 1;
		if(version >= 2)
			numTetsPerElement = readDword(mismatch, stream);
		const PxU32 nbGMTotalTetReferenceCount = readDword(mismatch, stream);
		const PxU32 nbTetRemapSize = readDword(mismatch, stream);

		const PxU32 numVertsPerElement = (numTetsPerElement == 5 || numTetsPerElement == 6) ? 8 : 4;
		const PxU32 numSimElements = nbGridModelTetrahedrons / numTetsPerElement;

		data.mSimulationData.mGridModelMaxTetsPerPartitions = nbGMMaxTetsPerPartition;
		data.mSimulationData.mNumTetsPerElement = numTetsPerElement;
		data.mMappingData.mTetsRemapSize = nbTetRemapSize;

		/*data.allocateGridModelData(nbGridModelTetrahedrons, nbGridModelVertices, 
			data.mCollisionMesh.mNbVertices, nbGridModelPartitions, nbGMRemapOutputSize,
			nbGMTotalTetReferenceCount, nbTetRemapSize, data.mCollisionMesh.mNbTetrahedrons,
			serialFlags & IMSF_GRB_DATA);*/
		data.mSimulationMesh.allocateTetrahedrons(nbGridModelTetrahedrons, serialFlags & IMSF_GRB_DATA);
		data.mSimulationMesh.allocateVertices(nbGridModelVertices, serialFlags & IMSF_GRB_DATA);
		data.mSimulationData.allocateGridModelData(nbGridModelTetrahedrons, nbGridModelVertices,
			data.mCollisionMesh.mNbVertices, nbGridModelPartitions, nbGMRemapOutputSize, numTetsPerElement, serialFlags & IMSF_GRB_DATA);
		data.mMappingData.allocatemappingData(data.mCollisionMesh.mNbVertices, nbTetRemapSize, data.mCollisionMesh.mNbTetrahedrons, serialFlags & IMSF_GRB_DATA);

		data.mMappingData.allocateTetRefData(nbGMTotalTetReferenceCount, data.mCollisionMesh.mNbVertices, serialFlags & IMSF_GRB_DATA);

		const PxU32 nbGridModelIndices = 4 * nbGridModelTetrahedrons;
		readIndices(serialFlags, data.mSimulationMesh.mTetrahedrons, nbGridModelIndices, data.mSimulationMesh.has16BitIndices(), mismatch, stream);

		//stream.read(data.mGridModelVerticesInvMass, sizeof(PxVec4) * nbGridModelVertices);
		stream.read(data.mSimulationMesh.mVertices, sizeof(PxVec3) * nbGridModelVertices);
		
		if (serialFlags & IMSF_MATERIALS)
		{
			PxU16* materials = data.mSimulationMesh.allocateMaterials();
			stream.read(materials, sizeof(PxU16)*nbGridModelTetrahedrons);
			if (mismatch)
			{
				for (PxU32 i = 0; i < nbTetrahedrons; i++)
					flip(materials[i]);
			}
		}
		stream.read(data.mSimulationData.mGridModelInvMass, sizeof(PxReal) * nbGridModelVertices);

		stream.read(data.mSimulationData.mGridModelTetraRestPoses, sizeof(PxMat33) * nbGridModelTetrahedrons);

		stream.read(data.mSimulationData.mGridModelOrderedTetrahedrons, sizeof(PxU32) * numSimElements);

		stream.read(data.mSimulationData.mGMRemapOutputCP, sizeof(PxU32) * numSimElements * numVertsPerElement);

		stream.read(data.mSimulationData.mGMAccumulatedPartitionsCP, sizeof(PxU32) * nbGridModelPartitions);
		
		stream.read(data.mSimulationData.mGMAccumulatedCopiesCP, sizeof(PxU32) * data.mSimulationMesh.mNbVertices);

		stream.read(data.mMappingData.mCollisionAccumulatedTetrahedronsRef, sizeof(PxU32) * data.mCollisionMesh.mNbVertices);

		stream.read(data.mMappingData.mCollisionTetrahedronsReferences, sizeof(PxU32) * data.mMappingData.mCollisionNbTetrahedronsReferences);
		
		stream.read(data.mMappingData.mCollisionSurfaceVertsHint, sizeof(PxU8) * data.mCollisionMesh.mNbVertices);

		stream.read(data.mMappingData.mCollisionSurfaceVertToTetRemap, sizeof(PxU32) * data.mCollisionMesh.mNbVertices);

		//stream.read(data->mVertsBarycentricInGridModel, sizeof(PxReal) * 4 * data->mNbVertices);
		stream.read(data.mSimulationData.mGMPullIndices, sizeof(PxU32) * numSimElements * numVertsPerElement);

		//stream.read(data->mVertsBarycentricInGridModel, sizeof(PxReal) * 4 * data->mNbVertices);
		stream.read(data.mMappingData.mVertsBarycentricInGridModel, sizeof(PxReal) * 4 * data.mCollisionMesh.mNbVertices);
	
		stream.read(data.mMappingData.mVertsRemapInGridModel, sizeof(PxU32) * data.mCollisionMesh.mNbVertices);

		stream.read(data.mMappingData.mTetsRemapColToSim, sizeof(PxU32) *nbTetRemapSize);

		stream.read(data.mMappingData.mTetsAccumulatedRemapColToSim, sizeof(PxU32) * data.mCollisionMesh.mNbTetrahedrons);
	}
	return true;
}

void MeshFactory::addTetrahedronMesh(TetrahedronMesh* np, bool lock)
{
	addToHash(mTetrahedronMeshes, np, lock ? &mTrackingMutex : NULL);
	OMNI_PVD_NOTIFY_ADD(np);
}

void MeshFactory::addSoftBodyMesh(SoftBodyMesh* np, bool lock)
{
	addToHash(mSoftBodyMeshes, np, lock ? &mTrackingMutex : NULL);
	OMNI_PVD_NOTIFY_ADD(np);
}

PxSoftBodyMesh* MeshFactory::createSoftBodyMesh(PxInputStream& desc)
{
	TetrahedronMeshData mSimulationMesh;
	SoftBodySimulationData mSimulationData;
	TetrahedronMeshData mCollisionMesh;
	SoftBodyCollisionData mCollisionData;
	CollisionMeshMappingData mMappingData;
	SoftBodyMeshData data(mSimulationMesh, mSimulationData, mCollisionMesh, mCollisionData, mMappingData);	
	if (!::loadSoftBodyMeshData(desc, data))
		return NULL;
	PxSoftBodyMesh* m = createSoftBodyMesh(data);
	//PX_DELETE(data);
	return m;
}

PxTetrahedronMesh* MeshFactory::createTetrahedronMesh(PxInputStream& desc)
{
	TetrahedronMeshData* data = ::loadTetrahedronMeshData(desc);
	if (!data)
		return NULL;
	PxTetrahedronMesh* m = createTetrahedronMesh(*data);
	PX_DELETE(data);
	return m;
}

PxTetrahedronMesh* MeshFactory::createTetrahedronMesh(TetrahedronMeshData& data)
{
	TetrahedronMesh* np = NULL;
	PX_NEW_SERIALIZED(np, TetrahedronMesh)(this, data);
	//PX_ASSERT(false);
	//PX_UNUSED(data);

	if (np)
		addTetrahedronMesh(np);

	return np;
}

// data injected by cooking lib for runtime cooking
PxTetrahedronMesh* MeshFactory::createTetrahedronMesh(void* data)
{
	return createTetrahedronMesh(*reinterpret_cast<TetrahedronMeshData*>(data));
}

PxSoftBodyMesh* MeshFactory::createSoftBodyMesh(Gu::SoftBodyMeshData& data)
{
	SoftBodyMesh* np = NULL;
	PX_NEW_SERIALIZED(np, SoftBodyMesh)(this, data);

	if (np) 	
		addSoftBodyMesh(np);	

	return np;
}

// data injected by cooking lib for runtime cooking
PxSoftBodyMesh* MeshFactory::createSoftBodyMesh(void* data)
{
	return createSoftBodyMesh(*reinterpret_cast<SoftBodyMeshData*>(data));
}

bool MeshFactory::removeSoftBodyMesh(PxSoftBodyMesh& tetMesh)
{
	SoftBodyMesh* gu = static_cast<SoftBodyMesh*>(&tetMesh);
	OMNI_PVD_NOTIFY_REMOVE(gu);
	PxMutex::ScopedLock lock(mTrackingMutex);
	bool found = mSoftBodyMeshes.erase(gu);
	return found;
}

bool MeshFactory::removeTetrahedronMesh(PxTetrahedronMesh& tetMesh)
{
	TetrahedronMesh* gu = static_cast<TetrahedronMesh*>(&tetMesh);
	OMNI_PVD_NOTIFY_REMOVE(gu);
	PxMutex::ScopedLock lock(mTrackingMutex);
	bool found = mTetrahedronMeshes.erase(gu);
	return found;
}

PxU32 MeshFactory::getNbSoftBodyMeshes()	const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return mSoftBodyMeshes.size();
}

PxU32 MeshFactory::getNbTetrahedronMeshes()	const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return mTetrahedronMeshes.size();
}

PxU32 MeshFactory::getTetrahedronMeshes(PxTetrahedronMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mTetrahedronMeshes.getEntries(), mTetrahedronMeshes.size());
}

PxU32 MeshFactory::getSoftBodyMeshes(PxSoftBodyMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mSoftBodyMeshes.getEntries(), mSoftBodyMeshes.size());
}

///////////////////////////////////////////////////////////////////////////////

void MeshFactory::addConvexMesh(ConvexMesh* np, bool lock)
{
	addToHash(mConvexMeshes, np, lock ? &mTrackingMutex : NULL);
	OMNI_PVD_NOTIFY_ADD(np);
}

// data injected by cooking lib for runtime cooking
PxConvexMesh* MeshFactory::createConvexMesh(void* data)
{
	return createConvexMesh(*reinterpret_cast<ConvexHullInitData*>(data));
}

PxConvexMesh* MeshFactory::createConvexMesh(ConvexHullInitData& data)
{
	ConvexMesh* np;
	PX_NEW_SERIALIZED(np, ConvexMesh)(this, data);
	if (np)
		addConvexMesh(np);

	return np;
}

PxConvexMesh* MeshFactory::createConvexMesh(PxInputStream& desc)
{
	ConvexMesh* np;
	PX_NEW_SERIALIZED(np, ConvexMesh)(this);
	if(!np)
		return NULL;

	if(!np->load(desc))
	{
		RefCountable_decRefCount(*np);
		return NULL;
	}

	addConvexMesh(np);
	return np;
}

bool MeshFactory::removeConvexMesh(PxConvexMesh& m)
{
	ConvexMesh* gu = static_cast<ConvexMesh*>(&m);
	OMNI_PVD_NOTIFY_REMOVE(gu);
	PxMutex::ScopedLock lock(mTrackingMutex);
	bool found = mConvexMeshes.erase(gu);
	return found;
}

PxU32 MeshFactory::getNbConvexMeshes() const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return mConvexMeshes.size();
}

PxU32 MeshFactory::getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mConvexMeshes.getEntries(), mConvexMeshes.size());
}

///////////////////////////////////////////////////////////////////////////////

void MeshFactory::addHeightField(HeightField* np, bool lock)
{
	addToHash(mHeightFields, np, lock ? &mTrackingMutex : NULL);
	OMNI_PVD_NOTIFY_ADD(np);
}

PxHeightField* MeshFactory::createHeightField(void* heightFieldMeshData)
{
	HeightField* np;
	PX_NEW_SERIALIZED(np, HeightField)(this, *reinterpret_cast<HeightFieldData*>(heightFieldMeshData));
	if(np)
		addHeightField(np);
	
	return np;
}

PxHeightField* MeshFactory::createHeightField(PxInputStream& stream)
{
	HeightField* np;
	PX_NEW_SERIALIZED(np, HeightField)(this);
	if(!np)
		return NULL;

	if(!np->load(stream))
	{
		RefCountable_decRefCount(*np);
		return NULL;
	}

	addHeightField(np);
	return np;
}

bool MeshFactory::removeHeightField(PxHeightField& hf)
{
	HeightField* gu = static_cast<HeightField*>(&hf);
	OMNI_PVD_NOTIFY_REMOVE(gu);
	PxMutex::ScopedLock lock(mTrackingMutex);
	bool found = mHeightFields.erase(gu);
	return found;
}

PxU32 MeshFactory::getNbHeightFields() const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return mHeightFields.size();
}

PxU32 MeshFactory::getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mHeightFields.getEntries(), mHeightFields.size());
}

///////////////////////////////////////////////////////////////////////////////

void MeshFactory::addFactoryListener(Gu::MeshFactoryListener& listener )
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	mFactoryListeners.pushBack( &listener );
}

void MeshFactory::removeFactoryListener(Gu::MeshFactoryListener& listener )
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	for ( PxU32 idx = 0; idx < mFactoryListeners.size(); ++idx )
	{
		if ( mFactoryListeners[idx] == &listener )
		{
			mFactoryListeners.replaceWithLast( idx );
			--idx;
		}
	}
}

void MeshFactory::notifyFactoryListener(const PxBase* base, PxType typeID)
{
	const PxU32 nbListeners = mFactoryListeners.size();
	for(PxU32 i=0; i<nbListeners; i++)
		mFactoryListeners[i]->onMeshFactoryBufferRelease(base, typeID);
}

#if PX_SUPPORT_OMNI_PVD
void MeshFactory::notifyListenersAdd(const PxBase* base)
{
	for (PxU32 i = 0; i < mFactoryListeners.size(); i++)
		mFactoryListeners[i]->onObjectAdd(base);
}
void MeshFactory::notifyListenersRemove(const PxBase* base)
{
	for (PxU32 i = 0; i < mFactoryListeners.size(); i++)
		mFactoryListeners[i]->onObjectRemove(base);
}
#endif

///////////////////////////////////////////////////////////////////////////////

void MeshFactory::addBVH(BVH* np, bool lock)
{
	addToHash(mBVHs, np, lock ? &mTrackingMutex : NULL);
	OMNI_PVD_NOTIFY_ADD(np);
}

// data injected by cooking lib for runtime cooking
PxBVH* MeshFactory::createBVH(void* data)
{
	return createBVH(*reinterpret_cast<BVHData*>(data));
}

PxBVH* MeshFactory::createBVH(BVHData& data)
{
	BVH* np;
	PX_NEW_SERIALIZED(np, BVH)(this, data);
	if (np)
		addBVH(np);

	return np;
}

PxBVH* MeshFactory::createBVH(PxInputStream& desc)
{
	BVH* np;
	PX_NEW_SERIALIZED(np, BVH)(this);
	if(!np)
		return NULL;

	if(!np->load(desc))
	{
		np->decRefCount();
		return NULL;
	}

	addBVH(np);
	return np;
}

bool MeshFactory::removeBVH(PxBVH& m)
{
	BVH* gu = static_cast<BVH*>(&m);
	OMNI_PVD_NOTIFY_REMOVE(gu);
	PxMutex::ScopedLock lock(mTrackingMutex);
	bool found = mBVHs.erase(gu);
	return found;
}

PxU32 MeshFactory::getNbBVHs() const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return mBVHs.size();
}

PxU32 MeshFactory::getBVHs(PxBVH** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mBVHs.getEntries(), mBVHs.size());
}

///////////////////////////////////////////////////////////////////////////////

bool MeshFactory::remove(PxBase& obj)
{
	const PxType type = obj.getConcreteType();
	if(type==PxConcreteType::eHEIGHTFIELD)
		return removeHeightField(static_cast<PxHeightField&>(obj));
	else if(type==PxConcreteType::eCONVEX_MESH)
		return removeConvexMesh(static_cast<PxConvexMesh&>(obj));
	else if(type==PxConcreteType::eTRIANGLE_MESH_BVH33 || type==PxConcreteType::eTRIANGLE_MESH_BVH34)
		return removeTriangleMesh(static_cast<PxTriangleMesh&>(obj));
	else if(type==PxConcreteType::eTETRAHEDRON_MESH)
		return removeTetrahedronMesh(static_cast<PxTetrahedronMesh&>(obj));
	else if (type == PxConcreteType::eSOFTBODY_MESH)
		return removeSoftBodyMesh(static_cast<PxSoftBodyMesh&>(obj));
	else if(type==PxConcreteType::eBVH)
		return removeBVH(static_cast<PxBVH&>(obj));
	return false;
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	class StandaloneInsertionCallback : public PxInsertionCallback
	{
	public:
		StandaloneInsertionCallback() {}

		virtual PxBase* buildObjectFromData(PxConcreteType::Enum type, void* data)
		{
			if(type == PxConcreteType::eTRIANGLE_MESH_BVH33)
			{
				TriangleMesh* np;
				PX_NEW_SERIALIZED(np, RTreeTriangleMesh)(NULL, *reinterpret_cast<TriangleMeshData*>(data));
				return np;
			}
			
			if(type == PxConcreteType::eTRIANGLE_MESH_BVH34)
			{
				TriangleMesh* np;
				PX_NEW_SERIALIZED(np, BV4TriangleMesh)(NULL, *reinterpret_cast<TriangleMeshData*>(data));
				return np;
			}

			if(type == PxConcreteType::eCONVEX_MESH)
			{
				ConvexMesh* np;
				PX_NEW_SERIALIZED(np, ConvexMesh)(NULL, *reinterpret_cast<ConvexHullInitData*>(data));
				return np;
			}

			if(type == PxConcreteType::eHEIGHTFIELD)
			{
				HeightField* np;
				PX_NEW_SERIALIZED(np, HeightField)(NULL, *reinterpret_cast<HeightFieldData*>(data));
				return np;
			}

			if(type == PxConcreteType::eBVH)
			{
				BVH* np;
				PX_NEW_SERIALIZED(np, BVH)(NULL, *reinterpret_cast<BVHData*>(data));
				return np;
			}

			if (type == PxConcreteType::eTETRAHEDRON_MESH)
			{
				TetrahedronMesh* np;
				PX_NEW_SERIALIZED(np, TetrahedronMesh)(NULL, *reinterpret_cast<TetrahedronMeshData*>(data));
				return np;
			}

			if (type == PxConcreteType::eSOFTBODY_MESH)
			{
				SoftBodyMesh* np;
				PX_NEW_SERIALIZED(np, SoftBodyMesh)(NULL, *reinterpret_cast<SoftBodyMeshData*>(data));
				return np;
			}

			outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Inserting object failed: "
				"Object type not supported for buildObjectFromData.");
			return NULL;
		}
	}gSAIC;
}

PxInsertionCallback* physx::immediateCooking::getInsertionCallback()
{
	return &gSAIC;
}

