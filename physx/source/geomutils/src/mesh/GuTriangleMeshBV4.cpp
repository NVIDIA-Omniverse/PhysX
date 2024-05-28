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

#include "GuTriangleMesh.h"
#include "GuTriangleMeshBV4.h"
#include "geometry/PxGeometryInternal.h"

using namespace physx;
using namespace Gu;

namespace physx
{

// PT: temporary for Kit

BV4TriangleMesh::BV4TriangleMesh(const PxTriangleMeshInternalData& data) : TriangleMesh(data)
{
	mMeshInterface.setNbTriangles(getNbTrianglesFast());
	if(has16BitIndices())
		mMeshInterface.setPointers(NULL, const_cast<IndTri16*>(reinterpret_cast<const IndTri16*>(getTrianglesFast())), getVerticesFast());
	else
		mMeshInterface.setPointers(const_cast<IndTri32*>(reinterpret_cast<const IndTri32*>(getTrianglesFast())), NULL, getVerticesFast());
	mBV4Tree.mMeshInterface = &mMeshInterface;

	mBV4Tree.mLocalBounds.mCenter = data.mAABB_Center;
	mBV4Tree.mLocalBounds.mExtentsMagnitude = data.mAABB_Extents.magnitude();

	mBV4Tree.mNbNodes = data.mNbNodes;
	mBV4Tree.mNodes = data.mNodes;
	mBV4Tree.mInitData = data.mInitData;
	mBV4Tree.mCenterOrMinCoeff = data.mCenterOrMinCoeff;
	mBV4Tree.mExtentsOrMaxCoeff = data.mExtentsOrMaxCoeff;
	mBV4Tree.mQuantized = data.mQuantized;
	mBV4Tree.mUserAllocated = true;
}

bool BV4TriangleMesh::getInternalData(PxTriangleMeshInternalData& data, bool takeOwnership)	const
{
	data.mNbVertices		= mNbVertices;
	data.mNbTriangles		= mNbTriangles;
	data.mVertices			= mVertices;
	data.mTriangles			= mTriangles;
	data.mFaceRemap			= mFaceRemap;
	data.mAABB_Center		= mAABB.mCenter;
	data.mAABB_Extents		= mAABB.mExtents;
	data.mGeomEpsilon		= mGeomEpsilon;
	data.mFlags				= mFlags;

	data.mNbNodes			= mBV4Tree.mNbNodes;
	data.mNodeSize			= mBV4Tree.mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
	data.mNodes				= mBV4Tree.mNodes;
	data.mInitData			= mBV4Tree.mInitData;
	data.mCenterOrMinCoeff	= mBV4Tree.mCenterOrMinCoeff;
	data.mExtentsOrMaxCoeff	= mBV4Tree.mExtentsOrMaxCoeff;
	data.mQuantized			= mBV4Tree.mQuantized;

	if(takeOwnership)
	{
		const_cast<BV4TriangleMesh*>(this)->setBaseFlag(PxBaseFlag::eOWNS_MEMORY, false);
		const_cast<BV4TriangleMesh*>(this)->mBV4Tree.mUserAllocated = true;
	}

	return true;
}

bool PxGetTriangleMeshInternalData(PxTriangleMeshInternalData& data, const PxTriangleMesh& mesh, bool takeOwnership)
{
	return static_cast<const TriangleMesh&>(mesh).getInternalData(data, takeOwnership);
}

//~ PT: temporary for Kit

BV4TriangleMesh::BV4TriangleMesh(MeshFactory* factory, TriangleMeshData& d) : TriangleMesh(factory, d)
{
	PX_ASSERT(d.mType==PxMeshMidPhase::eBVH34);

	BV4TriangleData& bv4Data = static_cast<BV4TriangleData&>(d);
	mMeshInterface = bv4Data.mMeshInterface;
	mBV4Tree = bv4Data.mBV4Tree;
	mBV4Tree.mMeshInterface = &mMeshInterface;
}

TriangleMesh* BV4TriangleMesh::createObject(PxU8*& address, PxDeserializationContext& context)
{
	BV4TriangleMesh* obj = PX_PLACEMENT_NEW(address, BV4TriangleMesh(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(BV4TriangleMesh);	
	obj->importExtraData(context);
	return obj;
}

void BV4TriangleMesh::exportExtraData(PxSerializationContext& stream)
{
	mBV4Tree.exportExtraData(stream);
	TriangleMesh::exportExtraData(stream);
}

void BV4TriangleMesh::importExtraData(PxDeserializationContext& context)
{
	mBV4Tree.importExtraData(context);
	TriangleMesh::importExtraData(context);

	if(has16BitIndices())
		mMeshInterface.setPointers(NULL, const_cast<IndTri16*>(reinterpret_cast<const IndTri16*>(getTrianglesFast())), getVerticesFast());
	else
		mMeshInterface.setPointers(const_cast<IndTri32*>(reinterpret_cast<const IndTri32*>(getTrianglesFast())), NULL, getVerticesFast());
	mBV4Tree.mMeshInterface = &mMeshInterface;
}

PxVec3 * BV4TriangleMesh::getVerticesForModification()
{
	return const_cast<PxVec3*>(getVertices());
}

PxBounds3 BV4TriangleMesh::refitBVH()
{
	PxBounds3 newBounds;

	const float gBoxEpsilon = 2e-4f;
	if(mBV4Tree.refit(newBounds, gBoxEpsilon))
	{
		mAABB.setMinMax(newBounds.minimum, newBounds.maximum);
	}
	else
	{
		newBounds = PxBounds3::centerExtents(mAABB.mCenter, mAABB.mExtents);

		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "BVH34 trees: refit operation only available on non-quantized trees.\n");
	}

	// PT: copied from RTreeTriangleMesh::refitBVH()
	// reset edge flags and remember we did that using a mesh flag (optimization)
	if(!mBV4Tree.mIsEdgeSet)
	{
		mBV4Tree.mIsEdgeSet = true;
		setAllEdgesActive();
	}

	return newBounds;
}

} // namespace physx
