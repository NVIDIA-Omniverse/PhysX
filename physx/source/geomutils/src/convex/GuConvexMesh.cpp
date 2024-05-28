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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxFoundation.h"
#include "GuConvexMesh.h"
#include "GuBigConvexData2.h"
#include "GuMeshFactory.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

bool ConvexMesh::getPolygonData(PxU32 i, PxHullPolygon& data) const
{
	if(i>=mHullData.mNbPolygons)
		return false;

	const HullPolygonData& poly = mHullData.mPolygons[i];
	data.mPlane[0]	= poly.mPlane.n.x;
	data.mPlane[1]	= poly.mPlane.n.y;
	data.mPlane[2]	= poly.mPlane.n.z;
	data.mPlane[3]	= poly.mPlane.d;
	data.mNbVerts	= poly.mNbVerts;
	data.mIndexBase	= poly.mVRef8;
	return true;
}

static void initConvexHullData(ConvexHullData& data)
{
	data.mAABB.setEmpty();
	data.mCenterOfMass = PxVec3(0);
	data.mNbEdges = PxBitAndWord();
	data.mNbHullVertices = 0;
	data.mNbPolygons = 0;
	data.mPolygons = NULL;
	data.mBigConvexRawData = NULL;
	data.mInternal.mInternalExtents = PxVec3(0.0f);
	data.mInternal.mInternalRadius= 0.0f;
}

ConvexMesh::ConvexMesh(MeshFactory* factory) :
	PxConvexMesh	(PxConcreteType::eCONVEX_MESH, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mNb				(0),
	mSdfData		(NULL),
	mBigConvexData	(NULL),
	mMass			(0),
	mInertia		(PxMat33(PxIdentity)),
	mMeshFactory	(factory)
{
	initConvexHullData(mHullData);
}

ConvexMesh::ConvexMesh(MeshFactory* factory, ConvexHullInitData& data) :
	PxConvexMesh	(PxConcreteType::eCONVEX_MESH, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mNb				(data.mNb),
	mSdfData		(data.mSdfData),
	mBigConvexData	(data.mBigConvexData),
	mMass			(data.mMass),
	mInertia		(data.mInertia),
	mMeshFactory	(factory)
{
	mHullData = data.mHullData;

	// this constructor takes ownership of memory from the data object
	data.mSdfData = NULL;
	data.mBigConvexData = NULL;
}

ConvexMesh::~ConvexMesh()
{
	if(getBaseFlags()&PxBaseFlag::eOWNS_MEMORY)
	{
		PX_FREE(mHullData.mPolygons);
		PX_DELETE(mBigConvexData);
		PX_DELETE(mSdfData);
	}
}

bool ConvexMesh::isGpuCompatible() const
{
	return mHullData.mNbHullVertices <= 64 &&
		mHullData.mNbPolygons <= 64 &&
		mHullData.mPolygons[0].mNbVerts <= 32 &&
		mHullData.mNbEdges.isBitSet() &&
		mHullData.checkExtentRadiusRatio();
}

void ConvexMesh::exportExtraData(PxSerializationContext& context)
{
	context.alignData(PX_SERIAL_ALIGN);
	const PxU32 bufferSize = computeBufferSize(mHullData, getNb());
	context.writeData(mHullData.mPolygons, bufferSize);

	if (mSdfData)
	{
		context.alignData(PX_SERIAL_ALIGN);
		context.writeData(mSdfData, sizeof(SDF));

		mSdfData->exportExtraData(context);
	}

	if(mBigConvexData)
	{
		context.alignData(PX_SERIAL_ALIGN);
		context.writeData(mBigConvexData, sizeof(BigConvexData));

		mBigConvexData->exportExtraData(context);
	}
}

void ConvexMesh::importExtraData(PxDeserializationContext& context)
{
	const PxU32 bufferSize = computeBufferSize(mHullData, getNb());
	mHullData.mPolygons = reinterpret_cast<HullPolygonData*>(context.readExtraData<PxU8, PX_SERIAL_ALIGN>(bufferSize));

	if (mSdfData)
	{
		mSdfData = context.readExtraData<SDF, PX_SERIAL_ALIGN>();
		PX_PLACEMENT_NEW(mSdfData, SDF(PxEmpty));
		mSdfData->importExtraData(context);
	}

	if(mBigConvexData)
	{
		mBigConvexData = context.readExtraData<BigConvexData, PX_SERIAL_ALIGN>();
		PX_PLACEMENT_NEW(mBigConvexData, BigConvexData(PxEmpty));
		mBigConvexData->importExtraData(context);
		mHullData.mBigConvexRawData = &mBigConvexData->mData;
	}
}

ConvexMesh* ConvexMesh::createObject(PxU8*& address, PxDeserializationContext& context)
{
	ConvexMesh* obj = PX_PLACEMENT_NEW(address, ConvexMesh(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(ConvexMesh);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

static bool convexHullLoad(ConvexHullData& data, PxInputStream& stream, PxBitAndDword& bufferSize)
{
	PxU32 version;
	bool Mismatch;
	if(!ReadHeader('C', 'L', 'H', 'L', version, Mismatch, stream))
		return false;

	if(version<=8)
	{
		if(!ReadHeader('C', 'V', 'H', 'L', version, Mismatch, stream))
			return false;
	}

	PxU32 Nb;

	// Import figures
	{
		PxU32 tmp[4];
		ReadDwordBuffer(tmp, 4, Mismatch, stream);
		data.mNbHullVertices	= PxTo8(tmp[0]);
		data.mNbEdges			= PxTo16(tmp[1]);
		data.mNbPolygons		= PxTo8(tmp[2]);
		Nb						= tmp[3];
	}

	//AM: In practice the old aligner approach wastes 20 bytes and there is no reason to 20 byte align this data.
	//I changed the code to just 4 align for the time being.  
	//On consoles if anything we will need to make this stuff 16 byte align vectors to have any sense, which will have to be done by padding data structures.
	PX_ASSERT(sizeof(HullPolygonData) % sizeof(PxReal) == 0);	//otherwise please pad it.
	PX_ASSERT(sizeof(PxVec3) % sizeof(PxReal) == 0);

	PxU32 bytesNeeded = computeBufferSize(data, Nb);

	PX_FREE(data.mPolygons);	// Load() can be called for an existing convex mesh. In that case we need to free the memory first.

	bufferSize = Nb;
	void* mDataMemory = PX_ALLOC(bytesNeeded, "ConvexHullData data");

	PxU8* address = reinterpret_cast<PxU8*>(mDataMemory);

	PX_ASSERT(address);

	data.mPolygons				= reinterpret_cast<HullPolygonData*>(address);	address += sizeof(HullPolygonData) * data.mNbPolygons;
	PxVec3* mDataHullVertices	= reinterpret_cast<PxVec3*>(address);			address += sizeof(PxVec3) * data.mNbHullVertices;
	PxU8* mDataFacesByEdges8	= address;										address += sizeof(PxU8) * data.mNbEdges * 2;
	PxU8* mDataFacesByVertices8 = address;										address += sizeof(PxU8) * data.mNbHullVertices * 3;
	PxU16* mEdges				= reinterpret_cast<PxU16*>(address);			address += data.mNbEdges.isBitSet() ? (sizeof(PxU16) * data.mNbEdges * 2) : 0;
	PxU8* mDataVertexData8		= address;										address += sizeof(PxU8) * Nb;	// PT: leave that one last, so that we don't need to serialize "Nb"

	PX_ASSERT(!(size_t(mDataHullVertices) % sizeof(PxReal)));
	PX_ASSERT(!(size_t(data.mPolygons) % sizeof(PxReal)));
	PX_ASSERT(size_t(address)<=size_t(mDataMemory)+bytesNeeded);

	// Import vertices
	readFloatBuffer(&mDataHullVertices->x, PxU32(3*data.mNbHullVertices), Mismatch, stream);

	if(version<=6)
	{
		PxU16 useUnquantizedNormals = readWord(Mismatch, stream);
		PX_UNUSED(useUnquantizedNormals);
	}

	// Import polygons
	stream.read(data.mPolygons, data.mNbPolygons*sizeof(HullPolygonData));

	if(Mismatch)
	{
		for(PxU32 i=0;i<data.mNbPolygons;i++)
			flipData(data.mPolygons[i]);
	}

	stream.read(mDataVertexData8, Nb);
	stream.read(mDataFacesByEdges8, PxU32(data.mNbEdges*2));
	if(version <= 5)
	{
		//KS - we need to compute faces-by-vertices here

		bool noPlaneShift = false;
		for(PxU32 i=0; i< data.mNbHullVertices; ++i)
		{
			PxU32 count = 0;
			PxU8 inds[3];
			for(PxU32 j=0; j<data.mNbPolygons; ++j)
			{
				HullPolygonData& polygon = data.mPolygons[j];
				for(PxU32 k=0; k< polygon.mNbVerts; ++k)
				{
					PxU8 index = mDataVertexData8[polygon.mVRef8 + k];
					if(i == index)
					{
						//Found a polygon
						inds[count++] = PxTo8(j);
						break;
					}
				}
				if(count == 3)
					break;
			}
			//We have 3 indices
			//PX_ASSERT(count == 3);
			//Do something here
			if(count == 3)
			{
				mDataFacesByVertices8[i*3+0] = inds[0];
				mDataFacesByVertices8[i*3+1] = inds[1];
				mDataFacesByVertices8[i*3+2] = inds[2];
			}
			else
			{
				noPlaneShift = true;
				break;
			}
		}


		if(noPlaneShift)
		{
			for(PxU32 a = 0; a < data.mNbHullVertices; ++a)
			{
				mDataFacesByVertices8[a*3] = 0xFF;
				mDataFacesByVertices8[a*3+1] = 0xFF;
				mDataFacesByVertices8[a*3+2] = 0xFF;
			}
		}

	}
	else
		stream.read(mDataFacesByVertices8, PxU32(data.mNbHullVertices * 3)); 

	if (data.mNbEdges.isBitSet())
	{
		if (version <= 7)
		{
			for (PxU32 a = 0; a < PxU32(data.mNbEdges * 2); ++a)
			{
				mEdges[a] = 0xFFFF;
			}
		}
		else
		{
			readWordBuffer(mEdges, PxU32(data.mNbEdges * 2), Mismatch, stream);
		}
	}
	return true;
}

bool ConvexMesh::load(PxInputStream& stream)
{
	// Import header
	PxU32 version;
	bool mismatch;
	if(!readHeader('C', 'V', 'X', 'M', version, mismatch, stream))
		return false;

	// Check if old (incompatible) mesh format is loaded
	if (version < PX_CONVEX_VERSION)
		return PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Loading convex mesh failed: Deprecated mesh cooking format.");

	// Import serialization flags
	PxU32 serialFlags	= readDword(mismatch, stream);
	PX_UNUSED(serialFlags);

	if(!convexHullLoad(mHullData, stream, mNb))
		return false;

	// Import local bounds
	float tmp[8];
	readFloatBuffer(tmp, 8, mismatch, stream);
//	geomEpsilon				= tmp[0];
	mHullData.mAABB = PxBounds3(PxVec3(tmp[1], tmp[2], tmp[3]), PxVec3(tmp[4],tmp[5],tmp[6]));

	// Import mass info
	mMass = tmp[7];
	if(mMass!=-1.0f)
	{
		readFloatBuffer(&mInertia(0,0), 9, mismatch, stream);
		readFloatBuffer(&mHullData.mCenterOfMass.x, 3, mismatch, stream);
	}

	// Import gaussmaps
	PxF32 gaussMapFlag = readFloat(mismatch, stream);
	if(gaussMapFlag != -1.0f)
	{
		PX_ASSERT(gaussMapFlag == 1.0f);	//otherwise file is corrupt

		PX_DELETE(mBigConvexData);
		PX_NEW_SERIALIZED(mBigConvexData, BigConvexData);	
      
		if(mBigConvexData)	
		{
			mBigConvexData->Load(stream);
			mHullData.mBigConvexRawData = &mBigConvexData->mData;
		}
	}

	//Import Sdf data
	PxF32 sdfFlag = readFloat(mismatch, stream);
	if (sdfFlag != -1.0f)
	{
		PX_ASSERT(sdfFlag == 1.0f);	//otherwise file is corrupt

		PX_DELETE(mSdfData);
		PX_NEW_SERIALIZED(mSdfData, SDF);

		if (mSdfData)
		{
			// Import sdf values
			mSdfData->mMeshLower.x = readFloat(mismatch, stream);
			mSdfData->mMeshLower.y = readFloat(mismatch, stream);
			mSdfData->mMeshLower.z = readFloat(mismatch, stream);
			mSdfData->mSpacing = readFloat(mismatch, stream);
			mSdfData->mDims.x = readDword(mismatch, stream);
			mSdfData->mDims.y = readDword(mismatch, stream);
			mSdfData->mDims.z = readDword(mismatch, stream);
			mSdfData->mNumSdfs = readDword(mismatch, stream);

			mSdfData->mNumSubgridSdfs = readDword(mismatch, stream);
			mSdfData->mNumStartSlots = readDword(mismatch, stream);
			mSdfData->mSubgridSize = readDword(mismatch, stream);
			mSdfData->mSdfSubgrids3DTexBlockDim.x = readDword(mismatch, stream);
			mSdfData->mSdfSubgrids3DTexBlockDim.y = readDword(mismatch, stream);
			mSdfData->mSdfSubgrids3DTexBlockDim.z = readDword(mismatch, stream);
			
			mSdfData->mSubgridsMinSdfValue = readFloat(mismatch, stream);
			mSdfData->mSubgridsMaxSdfValue = readFloat(mismatch, stream);
			mSdfData->mBytesPerSparsePixel = readDword(mismatch, stream);

			//allocate sdf 
			mSdfData->allocateSdfs(mSdfData->mMeshLower, mSdfData->mSpacing, mSdfData->mDims.x, mSdfData->mDims.y, mSdfData->mDims.z,
				mSdfData->mSubgridSize, mSdfData->mSdfSubgrids3DTexBlockDim.x, mSdfData->mSdfSubgrids3DTexBlockDim.y, mSdfData->mSdfSubgrids3DTexBlockDim.z,
				mSdfData->mSubgridsMinSdfValue, mSdfData->mSubgridsMaxSdfValue, mSdfData->mBytesPerSparsePixel);
						
			readFloatBuffer(mSdfData->mSdf, mSdfData->mNumSdfs, mismatch, stream);
			readByteBuffer(mSdfData->mSubgridSdf, mSdfData->mNumSubgridSdfs, stream);
			readIntBuffer(mSdfData->mSubgridStartSlots, mSdfData->mNumStartSlots, mismatch, stream);

			mHullData.mSdfData = mSdfData;
		}
	}
	
	
/*
	printf("\n\n");
	printf("COM: %f %f %f\n", massInfo.centerOfMass.x, massInfo.centerOfMass.y, massInfo.centerOfMass.z);
	printf("BND: %f %f %f\n", mHullData.aabb.getCenter().x, mHullData.aabb.getCenter().y, mHullData.aabb.getCenter().z);
	printf("CNT: %f %f %f\n", mHullData.mCenterxx.x, mHullData.mCenterxx.y, mHullData.mCenterxx.z);
	printf("COM-BND: %f BND-CNT: %f, CNT-COM: %f\n", (massInfo.centerOfMass - mHullData.aabb.getCenter()).magnitude(), (mHullData.aabb.getCenter() - mHullData.mCenterxx).magnitude(), (mHullData.mCenterxx - massInfo.centerOfMass).magnitude());
*/

// TEST_INTERNAL_OBJECTS
	// PT: this data was saved in ConvexMeshBuilder::save(), in this order: 'radius' first then 'extents'.
	// That order matched the previous in-memory data structure, but it changed later. So we read the data to a temp buffer and copy it afterwards.
	float internalObjectsData[4];
	readFloatBuffer(internalObjectsData, 4, mismatch, stream);
	mHullData.mInternal.mInternalRadius	= internalObjectsData[0];
	mHullData.mInternal.mInternalExtents.x = internalObjectsData[1];
	mHullData.mInternal.mInternalExtents.y = internalObjectsData[2];
	mHullData.mInternal.mInternalExtents.z = internalObjectsData[3];

	PX_ASSERT(mHullData.mInternal.mInternalExtents.isFinite());
	PX_ASSERT(mHullData.mInternal.mInternalExtents.x != 0.0f);
	PX_ASSERT(mHullData.mInternal.mInternalExtents.y != 0.0f);
	PX_ASSERT(mHullData.mInternal.mInternalExtents.z != 0.0f);
//~TEST_INTERNAL_OBJECTS
	return true;
}

void ConvexMesh::release()
{
	RefCountable_decRefCount(*this);
}

void ConvexMesh::onRefCountZero()
{
	// when the mesh failed to load properly, it will not have been added to the convex array
	::onRefCountZero(this, mMeshFactory, !getBufferSize(), "PxConvexMesh::release: double deletion detected!");
}

void ConvexMesh::acquireReference()
{
	RefCountable_incRefCount(*this);
}

PxU32 ConvexMesh::getReferenceCount() const
{
	return RefCountable_getRefCount(*this);
}

void ConvexMesh::getMassInformation(PxReal& mass, PxMat33& localInertia, PxVec3& localCenterOfMass) const
{
	mass = ConvexMesh::getMass();
	localInertia = ConvexMesh::getInertia();
	localCenterOfMass = ConvexMesh::getHull().mCenterOfMass;
}

PxBounds3 ConvexMesh::getLocalBounds() const
{
	PX_ASSERT(mHullData.mAABB.isValid());
	return PxBounds3::centerExtents(mHullData.mAABB.mCenter, mHullData.mAABB.mExtents);
}

const PxReal* ConvexMesh::getSDF() const
{
	if(mSdfData)
		return mSdfData->mSdf;

	return NULL;
}
