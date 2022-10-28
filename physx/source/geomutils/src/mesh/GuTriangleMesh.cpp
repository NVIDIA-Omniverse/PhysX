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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuMidphaseInterface.h"
#include "GuMeshFactory.h"
#include "GuConvexEdgeFlags.h"
#include "GuEdgeList.h"
#include "geometry/PxGeometryInternal.h"

using namespace physx;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static PxConcreteType::Enum gTable[] = {	PxConcreteType::eTRIANGLE_MESH_BVH33,
											PxConcreteType::eTRIANGLE_MESH_BVH34
										};

TriangleMesh::TriangleMesh(MeshFactory* factory, TriangleMeshData& d) :
	PxTriangleMesh				(PxType(gTable[d.mType]), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mNbVertices					(d.mNbVertices),
	mNbTriangles				(d.mNbTriangles),
	mVertices					(d.mVertices),
	mTriangles					(d.mTriangles),
	mAABB						(d.mAABB),
	mExtraTrigData				(d.mExtraTrigData),
	mGeomEpsilon				(d.mGeomEpsilon),
	mFlags						(d.mFlags),
	mMaterialIndices			(d.mMaterialIndices),
	mFaceRemap					(d.mFaceRemap),
	mAdjacencies				(d.mAdjacencies),
	mMeshFactory				(factory),
	mEdgeList					(NULL),
	mMass						(d.mMass),
	mInertia					(d.mInertia),
	mLocalCenterOfMass			(d.mLocalCenterOfMass),
	mGRB_triIndices				(d.mGRB_primIndices),
	mGRB_triAdjacencies			(d.mGRB_primAdjacencies),
	mGRB_faceRemap				(d.mGRB_faceRemap),
	mGRB_faceRemapInverse		(d.mGRB_faceRemapInverse),
	mGRB_BV32Tree				(d.mGRB_BV32Tree),
	mSdfData					(d.mSdfData),
	mAccumulatedTrianglesRef	(d.mAccumulatedTrianglesRef),
    mTrianglesReferences		(d.mTrianglesReferences),
    mNbTrianglesReferences		(d.mNbTrianglesReferences)
{
	// this constructor takes ownership of memory from the data object
	d.mVertices = NULL;
	d.mTriangles = NULL;
	d.mExtraTrigData = NULL;
	d.mFaceRemap = NULL;
	d.mAdjacencies = NULL;
	d.mMaterialIndices = NULL;

	d.mGRB_primIndices = NULL;

	d.mGRB_primAdjacencies = NULL;
	d.mGRB_faceRemap = NULL;
	d.mGRB_faceRemapInverse = NULL;
	d.mGRB_BV32Tree = NULL;

	d.mSdfData.mSdf = NULL;
	d.mSdfData.mSubgridStartSlots = NULL;
	d.mSdfData.mSubgridSdf = NULL;

	d.mAccumulatedTrianglesRef = NULL;
	d.mTrianglesReferences = NULL;
	
	// PT: 'getPaddedBounds()' is only safe if we make sure the bounds member is followed by at least 32bits of data
	PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(TriangleMesh, mExtraTrigData)>=PX_OFFSET_OF(TriangleMesh, mAABB)+4);	
}

// PT: temporary for Kit
TriangleMesh::TriangleMesh(const PxTriangleMeshInternalData& data) :
	PxTriangleMesh			(PxConcreteType::eTRIANGLE_MESH_BVH34, PxBaseFlags(0)),
	mNbVertices				(data.mNbVertices),
	mNbTriangles			(data.mNbTriangles),
	mVertices				(data.mVertices),
	mTriangles				(data.mTriangles),
	mExtraTrigData			(NULL),
	mGeomEpsilon			(data.mGeomEpsilon),
	mFlags					(data.mFlags),
	mMaterialIndices		(NULL),
	mFaceRemap				(data.mFaceRemap),
	mAdjacencies			(NULL),
	mMeshFactory			(NULL),
	mEdgeList				(NULL),
	mGRB_triIndices			(NULL),
	mGRB_triAdjacencies		(NULL),
	mGRB_faceRemap			(NULL),
	mGRB_faceRemapInverse	(NULL),
	mGRB_BV32Tree			(NULL),
	mAccumulatedTrianglesRef(NULL),
	mTrianglesReferences	(NULL),
	mNbTrianglesReferences	(0)
{
	mAABB.mCenter = data.mAABB_Center;
	mAABB.mExtents = data.mAABB_Extents;
}
//~ PT: temporary for Kit

TriangleMesh::~TriangleMesh() 
{ 	
	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		PX_FREE(mExtraTrigData);
		PX_FREE(mFaceRemap);
		PX_FREE(mAdjacencies);
		PX_FREE(mMaterialIndices);
		PX_FREE(mTriangles);
		PX_FREE(mVertices);

		PX_FREE(mGRB_triIndices); 

		PX_FREE(mGRB_triAdjacencies);
		PX_FREE(mGRB_faceRemap);
		PX_FREE(mGRB_faceRemapInverse);

		PX_DELETE(mGRB_BV32Tree);

		PX_FREE(mAccumulatedTrianglesRef);
		PX_FREE(mTrianglesReferences);
	}

	PX_DELETE(mEdgeList);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: used to be automatic but making it manual saves bytes in the internal mesh

void TriangleMesh::exportExtraData(PxSerializationContext& stream)
{
	//PX_DEFINE_DYNAMIC_ARRAY(TriangleMesh, mVertices, PxField::eVEC3, mNbVertices, Ps::PxFieldFlag::eSERIALIZE),
	if(mVertices)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mVertices, mNbVertices * sizeof(PxVec3));
	}

	if(mTriangles)
	{
		const PxU32 triangleSize = mFlags & PxTriangleMeshFlag::e16_BIT_INDICES ? sizeof(PxU16) : sizeof(PxU32);
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mTriangles, mNbTriangles * 3 * triangleSize);
	}

	//PX_DEFINE_DYNAMIC_ARRAY(TriangleMesh, mExtraTrigData, PxField::eBYTE, mNbTriangles, Ps::PxFieldFlag::eSERIALIZE),
	if(mExtraTrigData)
	{
		// PT: it might not be needed to 16-byte align this array of PxU8....
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mExtraTrigData, mNbTriangles * sizeof(PxU8));
	}

	if(mMaterialIndices)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mMaterialIndices, mNbTriangles * sizeof(PxU16));
	}

	if(mFaceRemap)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mFaceRemap, mNbTriangles * sizeof(PxU32));
	}

	if(mAdjacencies)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mAdjacencies, mNbTriangles * sizeof(PxU32) * 3);
	}

	if(mSdfData.mSdf)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mSdfData.mSdf, mSdfData.mNumSdfs * sizeof(PxReal));
	}

	if (mSdfData.mNumStartSlots)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mSdfData.mSubgridStartSlots, mSdfData.mNumStartSlots * sizeof(PxU32));
	}

	if (mSdfData.mSubgridSdf)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mSdfData.mSubgridSdf, mSdfData.mNumSubgridSdfs * sizeof(PxU8));
	}
}

void TriangleMesh::importExtraData(PxDeserializationContext& context)
{
	// PT: vertices are followed by indices, so it will be safe to V4Load vertices from a deserialized binary file
	if(mVertices)
		mVertices = context.readExtraData<PxVec3, PX_SERIAL_ALIGN>(mNbVertices);

	if(mTriangles)
	{
		if(mFlags & PxTriangleMeshFlag::e16_BIT_INDICES)
			mTriangles = context.readExtraData<PxU16, PX_SERIAL_ALIGN>(3*mNbTriangles);
		else
			mTriangles = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(3*mNbTriangles);
	}

	if(mExtraTrigData)
		mExtraTrigData = context.readExtraData<PxU8, PX_SERIAL_ALIGN>(mNbTriangles);

	if(mMaterialIndices)
		mMaterialIndices = context.readExtraData<PxU16, PX_SERIAL_ALIGN>(mNbTriangles);

	if(mFaceRemap)
		mFaceRemap = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(mNbTriangles);

	if(mAdjacencies)
		mAdjacencies = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(3*mNbTriangles);

	if(mSdfData.mSdf)
		mSdfData.mSdf = context.readExtraData<PxReal, PX_SERIAL_ALIGN>(mSdfData.mNumSdfs);

	if (mSdfData.mSubgridStartSlots)
		mSdfData.mSubgridStartSlots = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(mSdfData.mNumStartSlots);

	if (mSdfData.mSubgridSdf)
		mSdfData.mSubgridSdf = context.readExtraData<PxU8, PX_SERIAL_ALIGN>(mSdfData.mNumSubgridSdfs);


	mGRB_triIndices = NULL;
	mGRB_triAdjacencies = NULL;
	mGRB_faceRemap = NULL;
	mGRB_faceRemapInverse = NULL;
	mGRB_BV32Tree = NULL;
}

void TriangleMesh::onRefCountZero()
{
	::onRefCountZero(this, mMeshFactory, false, "PxTriangleMesh::release: double deletion detected!");
}

void TriangleMesh::release()
{
	Cm::RefCountable_decRefCount(*this);
}

PxVec3* TriangleMesh::getVerticesForModification()
{
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxTriangleMesh::getVerticesForModification() is not supported for this type of meshes.");

	return NULL;
}

PxBounds3 TriangleMesh::refitBVH()
{
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxTriangleMesh::refitBVH() is not supported for this type of meshes.");

	return PxBounds3(mAABB.getMin(), mAABB.getMax());
}

void TriangleMesh::setAllEdgesActive()
{
	if(mExtraTrigData)
	{
		const PxU32 nbTris = mNbTriangles;
		for(PxU32 i=0; i<nbTris; i++)
			mExtraTrigData[i] |= ETD_CONVEX_EDGE_ALL;
	}
}

const EdgeList* TriangleMesh::requestEdgeList() const
{
	if(!mEdgeList)
	{
		EDGELISTCREATE create;
		create.NbFaces	= mNbTriangles;
		create.Verts	= mVertices;
		if(has16BitIndices())
			create.WFaces	= reinterpret_cast<const PxU16*>(mTriangles);
		else
			create.DFaces	= reinterpret_cast<const PxU32*>(mTriangles);

		mEdgeList = PX_NEW(EdgeList);
		mEdgeList->init(create);
	}
	return mEdgeList;
}
