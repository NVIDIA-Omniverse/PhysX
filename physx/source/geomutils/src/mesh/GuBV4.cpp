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

#include "foundation/PxMemory.h"
#include "GuBV4.h"
#include "GuBV4_Common.h"
#include "CmSerialize.h"
#include "foundation/PxVecMath.h"
#include "common/PxSerialFramework.h"

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace physx::aos;

SourceMeshBase::SourceMeshBase(MeshType meshType) : mNbVerts(0), mVerts(NULL), mType(meshType), mRemap(NULL)
{
}

SourceMeshBase::~SourceMeshBase()
{
	PX_FREE(mRemap);
}

///////////////////////////////////////////////////////////////////////////////

TetrahedronSourceMesh::TetrahedronSourceMesh() : SourceMeshBase(MeshType::TET_MESH)
{
	reset();
}

TetrahedronSourceMesh::~TetrahedronSourceMesh()
{
}

void TetrahedronSourceMesh::reset()
{
	mNbVerts = 0;
	mVerts = NULL;
	mNbTetrahedrons = 0;
	mTetrahedrons32 = NULL;
	mTetrahedrons16 = NULL;
	mRemap = NULL;
}

void TetrahedronSourceMesh::operator=(TetrahedronSourceMesh& v)
{
	mNbVerts = v.mNbVerts;
	mVerts = v.mVerts;
	mNbTetrahedrons = v.mNbTetrahedrons;
	mTetrahedrons32 = v.mTetrahedrons32;
	mTetrahedrons16 = v.mTetrahedrons16;
	v.reset();
}

void TetrahedronSourceMesh::remapTopology(const PxU32* order)
{
	if(!mNbTetrahedrons)
		return;

	if(mTetrahedrons32)
	{
		IndTetrahedron32* newTopo = PX_NEW(IndTetrahedron32)[mNbTetrahedrons];
		for(PxU32 i = 0; i<mNbTetrahedrons; i++)
			newTopo[i] = mTetrahedrons32[order[i]];

		PxMemCopy(mTetrahedrons32, newTopo, sizeof(IndTetrahedron32)*mNbTetrahedrons);
		PX_DELETE_ARRAY(newTopo);
	}
	else
	{
		PX_ASSERT(mTetrahedrons16);
		IndTetrahedron16* newTopo = PX_NEW(IndTetrahedron16)[mNbTetrahedrons];
		for(PxU32 i = 0; i<mNbTetrahedrons; i++)
			newTopo[i] = mTetrahedrons16[order[i]];

		PxMemCopy(mTetrahedrons16, newTopo, sizeof(IndTetrahedron16)*mNbTetrahedrons);
		PX_DELETE_ARRAY(newTopo);
	}

	{
		PxU32* newMap = PX_ALLOCATE(PxU32, mNbTetrahedrons, "newMap");
		for(PxU32 i = 0; i<mNbTetrahedrons; i++)
			newMap[i] = mRemap ? mRemap[order[i]] : order[i];

		PX_FREE(mRemap);
		mRemap = newMap;
	}
}

void TetrahedronSourceMesh::getPrimitiveBox(const PxU32 primitiveInd, Vec4V& minV, Vec4V& maxV)
{
	TetrahedronPointers VP;
	getTetrahedron(VP, primitiveInd);

	const Vec4V v0V = V4LoadU(&VP.Vertex[0]->x);
	const Vec4V v1V = V4LoadU(&VP.Vertex[1]->x);
	const Vec4V v2V = V4LoadU(&VP.Vertex[2]->x);
	const Vec4V v3V = V4LoadU(&VP.Vertex[3]->x);
	minV = V4Min(v0V, v1V);
	minV = V4Min(minV, v2V);
	minV = V4Min(minV, v3V);

	maxV = V4Max(v0V, v1V);
	maxV = V4Max(maxV, v2V);
	maxV = V4Max(maxV, v3V);
}

void TetrahedronSourceMesh::refit(const PxU32 primitiveInd, PxBounds3& refitBox)
{
	TetrahedronPointers VP;
	getTetrahedron(VP, primitiveInd);

	refitBox.include(*VP.Vertex[0]);
	refitBox.include(*VP.Vertex[1]);
	refitBox.include(*VP.Vertex[2]);
	refitBox.include(*VP.Vertex[3]);
}

///////////////////////////////////////////////////////////////////////////////

SourceMesh::SourceMesh() : SourceMeshBase(MeshType::TRI_MESH)
{
	reset();
}

SourceMesh::~SourceMesh()
{
}

void SourceMesh::reset()
{
	mNbVerts		= 0;
	mVerts			= NULL;
	mNbTris			= 0;
	mTriangles32	= NULL;
	mTriangles16	= NULL;
	mRemap			= NULL;
}

void SourceMesh::operator=(SourceMesh& v)
{
	mNbVerts		= v.mNbVerts;
	mVerts			= v.mVerts;
	mNbTris			= v.mNbTris;
	mTriangles32	= v.mTriangles32;
	mTriangles16	= v.mTriangles16;
	mRemap			= v.mRemap;
	v.reset();
}

void SourceMesh::remapTopology(const PxU32* order)
{
	if(!mNbTris)
		return;

	if(mTriangles32)
	{
		IndTri32* newTopo = PX_NEW(IndTri32)[mNbTris];
		for(PxU32 i=0;i<mNbTris;i++)
			newTopo[i] = mTriangles32[order[i]];

		PxMemCopy(mTriangles32, newTopo, sizeof(IndTri32)*mNbTris);
		PX_DELETE_ARRAY(newTopo);
	}
	else
	{
		PX_ASSERT(mTriangles16);
		IndTri16* newTopo = PX_NEW(IndTri16)[mNbTris];
		for(PxU32 i=0;i<mNbTris;i++)
			newTopo[i] = mTriangles16[order[i]];

		PxMemCopy(mTriangles16, newTopo, sizeof(IndTri16)*mNbTris);
		PX_DELETE_ARRAY(newTopo);
	}

	{
		PxU32* newMap = PX_ALLOCATE(PxU32, mNbTris, "newMap");
		for(PxU32 i=0;i<mNbTris;i++)
			newMap[i] = mRemap ? mRemap[order[i]] : order[i];

		PX_FREE(mRemap);
		mRemap = newMap;
	}
}

void SourceMesh::getPrimitiveBox(const PxU32 primitiveInd, Vec4V& minV, Vec4V& maxV)
{
	VertexPointers VP;
	getTriangle(VP, primitiveInd);

	const Vec4V v0V = V4LoadU(&VP.Vertex[0]->x);
	const Vec4V v1V = V4LoadU(&VP.Vertex[1]->x);
	const Vec4V v2V = V4LoadU(&VP.Vertex[2]->x);
	minV = V4Min(v0V, v1V);
	minV = V4Min(minV, v2V);

	maxV = V4Max(v0V, v1V);
	maxV = V4Max(maxV, v2V);
}

void SourceMesh::refit(const PxU32 primitiveInd, PxBounds3& refitBox)
{
	VertexPointers VP;
	getTriangle(VP, primitiveInd);

	refitBox.include(*VP.Vertex[0]);
	refitBox.include(*VP.Vertex[1]);
	refitBox.include(*VP.Vertex[2]);
}

bool SourceMesh::isValid() const
{
	if(!mNbTris || !mNbVerts)			return false;
	if(!mVerts)							return false;
	if(!mTriangles32 && !mTriangles16)	return false;
	return true;
}

/////

BV4Tree::BV4Tree(SourceMesh* meshInterface, const PxBounds3& localBounds)
{
	reset();
	init(meshInterface, localBounds);
}

BV4Tree::BV4Tree()
{
	reset();
}

void BV4Tree::release()
{
	if(!mUserAllocated)
	{
#ifdef GU_BV4_USE_SLABS
		PX_FREE(mNodes);
//		PX_DELETE(mNodes);
#else
		PX_DELETE_ARRAY(mNodes);
#endif
	}

	mNodes = NULL;
	mNbNodes = 0;
	reset();
}

BV4Tree::~BV4Tree()
{
	release();
}

void BV4Tree::reset()
{
	mMeshInterface				= NULL;
	//mTetrahedronMeshInterface	= NULL;
	mNbNodes					= 0;
	mNodes						= NULL;
	mInitData					= 0;
	mCenterOrMinCoeff			= PxVec3(0.0f);
	mExtentsOrMaxCoeff			= PxVec3(0.0f);
	mUserAllocated				= false;
	mQuantized					= false;
	mIsEdgeSet					= false;
}

void BV4Tree::operator=(BV4Tree& v)
{
	mMeshInterface				= v.mMeshInterface;
	//mTetrahedronMeshInterface	= v.mTetrahedronMeshInterface;
	mLocalBounds				= v.mLocalBounds;
	mNbNodes					= v.mNbNodes;
	mNodes						= v.mNodes;
	mInitData					= v.mInitData;
	mCenterOrMinCoeff			= v.mCenterOrMinCoeff;
	mExtentsOrMaxCoeff			= v.mExtentsOrMaxCoeff;
	mUserAllocated				= v.mUserAllocated;
	mQuantized					= v.mQuantized;
	mIsEdgeSet					= false;
	v.reset();
}

bool BV4Tree::init(SourceMeshBase* meshInterface, const PxBounds3& localBounds)
{
	mMeshInterface	= meshInterface;
	mLocalBounds.init(localBounds);
	return true;
}

//bool BV4Tree::init(TetrahedronSourceMesh* meshInterface, const PxBounds3& localBounds)
//{
//	mTetrahedronMeshInterface = meshInterface;
//	mLocalBounds.init(localBounds);
//	return true;
//}

// PX_SERIALIZATION
BV4Tree::BV4Tree(const PxEMPTY) : mLocalBounds(PxEmpty)
{
	mUserAllocated = true;
	mIsEdgeSet = false;
}

void BV4Tree::exportExtraData(PxSerializationContext& stream)
{
	if(mNbNodes)
	{
		stream.alignData(16);
		const PxU32 nodeSize = mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
		stream.writeData(mNodes, mNbNodes*nodeSize);
	}
}

void BV4Tree::importExtraData(PxDeserializationContext& context)
{
	if(mNbNodes)
	{
		context.alignExtraData(16);
		if(mQuantized)
			mNodes = context.readExtraData<BVDataPackedQ>(mNbNodes);
		else
			mNodes = context.readExtraData<BVDataPackedNQ>(mNbNodes);
	}
}
//~PX_SERIALIZATION

bool BV4Tree::load(PxInputStream& stream, bool mismatch_)
{
	PX_ASSERT(!mUserAllocated);

	release();

	PxI8 a, b, c, d;
	readChunk(a, b, c, d, stream);
	if(a!='B' || b!='V' || c!='4' || d!=' ')
		return false;

	bool mismatch;
	PxU32 fileVersion;
	if(!readBigEndianVersionNumber(stream, mismatch_, fileVersion, mismatch))
		return false;

	readFloatBuffer(&mLocalBounds.mCenter.x, 3, mismatch, stream);
	mLocalBounds.mExtentsMagnitude = readFloat(mismatch, stream);

	mInitData = readDword(mismatch, stream);

	readFloatBuffer(&mCenterOrMinCoeff.x, 3, mismatch, stream);
	readFloatBuffer(&mExtentsOrMaxCoeff.x, 3, mismatch, stream);

	// PT: version 3
	if(fileVersion>=3)
	{
		const PxU32 Quantized = readDword(mismatch, stream);
		mQuantized = Quantized!=0;
	}
	else
		mQuantized = true;

	const PxU32 nbNodes = readDword(mismatch, stream);
	mNbNodes = nbNodes;

	if(nbNodes)
	{
		PxU32 dataSize = 0;
#ifdef GU_BV4_USE_SLABS
		const PxU32 nodeSize = mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
		dataSize = nodeSize*nbNodes;
		void* nodes = PX_ALLOC(dataSize, "BV4 nodes");	// PT: PX_NEW breaks alignment here
//		BVDataPacked* nodes = reinterpret_cast<BVDataPacked*>(PX_ALLOC(sizeof(BVDataPacked)*nbNodes, "BV4 nodes"));	// PT: PX_NEW breaks alignment here
		mNodes = nodes;
#else
		BVDataPacked* nodes = PX_NEW(BVDataPacked)[nbNodes];
		mNodes = nodes;
#endif
//		PxMarkSerializedMemory(nodes, dataSize);

		stream.read(nodes, dataSize);
		PX_ASSERT(!mismatch);
	}
	else mNodes = NULL;

	mIsEdgeSet = false;

	return true;
}

#define VERSION2
#ifdef VERSION1
bool BV4Tree::refit(PxBounds3& globalBounds, float epsilon)
{
	if(mQuantized)

	if(!mNodes)
	{
		PxBounds3 bounds;
		bounds.setEmpty();
		if(mMeshInterface)
		{
			PxU32 nbVerts = mMeshInterface->getNbVertices();
			const PxVec3* verts = mMeshInterface->getVerts();
			while(nbVerts--)
				bounds.include(*verts++);
			mLocalBounds.init(bounds);
		}
		if(mTetrahedronMeshInterface)
		{
			PX_ASSERT(0);
		}
		return true;
	}

	class PxBounds3Padded : public PxBounds3
	{
	public:
		PX_FORCE_INLINE PxBounds3Padded()	{}
		PX_FORCE_INLINE ~PxBounds3Padded()	{}
		PxU32	padding;
	};

	PX_ASSERT(!(mNbNodes&3));
	PxU32 nb = mNbNodes/4;
	BVDataSwizzledNQ* data = reinterpret_cast<BVDataSwizzledNQ*>(mNodes);
	while(nb--)
	{
		BVDataSwizzledNQ* PX_RESTRICT current = data + nb;

		for(PxU32 j=0;j<4;j++)
		{
			if(current->getChildData(j)==PX_INVALID_U32)
				continue;
			
			Vec4V minV = V4Load(FLT_MAX);
			Vec4V maxV = V4Load(-FLT_MAX);

			if(current->isLeaf(j))
			{
				PxU32 primIndex = current->getPrimitive(j);

				PxU32 nbToGo = getNbPrimitives(primIndex);
				VertexPointers VP;
				do
				{
					PX_ASSERT(primIndex<mMeshInterface->getNbTriangles());
					mMeshInterface->getTriangle(VP, primIndex);

					const Vec4V v0V = V4LoadU(&VP.Vertex[0]->x);	
					const Vec4V v1V = V4LoadU(&VP.Vertex[1]->x);
					const Vec4V v2V = V4LoadU(&VP.Vertex[2]->x);
					minV = V4Min(minV, v0V);
					minV = V4Min(minV, v1V);
					minV = V4Min(minV, v2V);
					maxV = V4Max(maxV, v0V);
					maxV = V4Max(maxV, v1V);
					maxV = V4Max(maxV, v2V);

					primIndex++;
				}while(nbToGo--);

				const Vec4V epsilonV = V4Load(epsilon);
				minV = V4Sub(minV, epsilonV);
				maxV = V4Add(maxV, epsilonV);
			}
			else
			{
				PxU32 childOffset = current->getChildOffset(j);
				PX_ASSERT(!(childOffset&3));
				childOffset>>=2;
				PX_ASSERT(childOffset>nb);
				const PxU32 childType = current->getChildType(j);

				// PT: TODO: revisit SIMD here, not great
				const BVDataSwizzledNQ* PX_RESTRICT next = data + childOffset;
				{
					{
						const Vec4V childMinV = V4LoadXYZW(next->mMinX[0], next->mMinY[0], next->mMinZ[0], 0.0f);
						const Vec4V childMaxV = V4LoadXYZW(next->mMaxX[0], next->mMaxY[0], next->mMaxZ[0], 0.0f);
//						minV = V4Min(minV, childMinV);
//						maxV = V4Max(maxV, childMaxV);
						minV = childMinV;
						maxV = childMaxV;
					}

					{
						const Vec4V childMinV = V4LoadXYZW(next->mMinX[1], next->mMinY[1], next->mMinZ[1], 0.0f);
						const Vec4V childMaxV = V4LoadXYZW(next->mMaxX[1], next->mMaxY[1], next->mMaxZ[1], 0.0f);
						minV = V4Min(minV, childMinV);
						maxV = V4Max(maxV, childMaxV);
					}

/*					{
						const Vec4V childMinV0 = V4LoadXYZW(next->mMinX[0], next->mMinY[0], next->mMinZ[0], 0.0f);
						const Vec4V childMaxV0 = V4LoadXYZW(next->mMaxX[0], next->mMaxY[0], next->mMaxZ[0], 0.0f);
						const Vec4V childMinV1 = V4LoadXYZW(next->mMinX[1], next->mMinY[1], next->mMinZ[1], 0.0f);
						const Vec4V childMaxV1 = V4LoadXYZW(next->mMaxX[1], next->mMaxY[1], next->mMaxZ[1], 0.0f);
						minV = V4Min(childMinV0, childMinV1);
						maxV = V4Max(childMaxV0, childMaxV1);
					}*/

					if(childType>0)
					{
						const Vec4V childMinV = V4LoadXYZW(next->mMinX[2], next->mMinY[2], next->mMinZ[2], 0.0f);
						const Vec4V childMaxV = V4LoadXYZW(next->mMaxX[2], next->mMaxY[2], next->mMaxZ[2], 0.0f);
						minV = V4Min(minV, childMinV);
						maxV = V4Max(maxV, childMaxV);
					}

					if(childType>1)
					{
						const Vec4V childMinV = V4LoadXYZW(next->mMinX[3], next->mMinY[3], next->mMinZ[3], 0.0f);
						const Vec4V childMaxV = V4LoadXYZW(next->mMaxX[3], next->mMaxY[3], next->mMaxZ[3], 0.0f);
						minV = V4Min(minV, childMinV);
						maxV = V4Max(maxV, childMaxV);
					}
				}
			}

			PxBounds3Padded refitBox;
			V4StoreU_Safe(minV, &refitBox.minimum.x);
			V4StoreU_Safe(maxV, &refitBox.maximum.x);

			current->mMinX[j] = refitBox.minimum.x;
			current->mMinY[j] = refitBox.minimum.y;
			current->mMinZ[j] = refitBox.minimum.z;
			current->mMaxX[j] = refitBox.maximum.x;
			current->mMaxY[j] = refitBox.maximum.y;
			current->mMaxZ[j] = refitBox.maximum.z;
		}
	}

	BVDataSwizzledNQ* root = reinterpret_cast<BVDataSwizzledNQ*>(mNodes);
	{
		globalBounds.setEmpty();

		for(PxU32 j=0;j<4;j++)
		{
			if(root->getChildData(j)==PX_INVALID_U32)
				continue;

			PxBounds3 refitBox;
			refitBox.minimum.x = root->mMinX[j];
			refitBox.minimum.y = root->mMinY[j];
			refitBox.minimum.z = root->mMinZ[j];
			refitBox.maximum.x = root->mMaxX[j];
			refitBox.maximum.y = root->mMaxY[j];
			refitBox.maximum.z = root->mMaxZ[j];
			globalBounds.include(refitBox);
		}

		mLocalBounds.init(globalBounds);
	}
	return true;
}
#endif

#ifdef VERSION2
bool BV4Tree::refit(PxBounds3& globalBounds, float epsilon)
{
	if(mQuantized)
		return false;

	if(!mNodes)
	{
		globalBounds.setEmpty();
		if(mMeshInterface)
		{
			PxU32 nbVerts = mMeshInterface->getNbVertices();
			const PxVec3* verts = mMeshInterface->getVerts();
			while(nbVerts--)
				globalBounds.include(*verts++);
			mLocalBounds.init(globalBounds);
		}
		return true;
	}
	
	class PxBounds3Padded : public PxBounds3
	{
	public:
		PX_FORCE_INLINE PxBounds3Padded()	{}
		PX_FORCE_INLINE ~PxBounds3Padded()	{}
		PxU32	padding;
	};

	PX_ASSERT(!(mNbNodes&3));
	PxU32 nb = mNbNodes/4;
	BVDataSwizzledNQ* data = reinterpret_cast<BVDataSwizzledNQ*>(mNodes);
	
	while(nb--)
	{
		BVDataSwizzledNQ* PX_RESTRICT current = data + nb;

		for(PxU32 j=0;j<4;j++)
		{
			if(current->getChildData(j)==PX_INVALID_U32)
				continue;
			
			if(current->isLeaf(j))
			{
				PxU32 primIndex = current->getPrimitive(j);

				Vec4V minV = V4Load(FLT_MAX);
				Vec4V maxV = V4Load(-FLT_MAX);

				PxU32 nbToGo = getNbPrimitives(primIndex);
				//VertexPointers VP;
				do
				{
					PX_ASSERT(primIndex< mMeshInterface->getNbPrimitives());


					//meshInterface->getTriangle(VP, primIndex);
					Vec4V tMin, tMax;
					mMeshInterface->getPrimitiveBox(primIndex, tMin, tMax);
					minV = V4Min(minV, tMin);
					maxV = V4Max(maxV, tMax);

				/*	const Vec4V v0V = V4LoadU(&VP.Vertex[0]->x);	
					const Vec4V v1V = V4LoadU(&VP.Vertex[1]->x);
					const Vec4V v2V = V4LoadU(&VP.Vertex[2]->x);
					minV = V4Min(minV, v0V);
					minV = V4Min(minV, v1V);
					minV = V4Min(minV, v2V);
					maxV = V4Max(maxV, v0V);
					maxV = V4Max(maxV, v1V);
					maxV = V4Max(maxV, v2V);*/

					primIndex++;
				}while(nbToGo--);

				const Vec4V epsilonV = V4Load(epsilon);
				minV = V4Sub(minV, epsilonV);
				maxV = V4Add(maxV, epsilonV);

				PxBounds3Padded refitBox;
				V4StoreU_Safe(minV, &refitBox.minimum.x);
				V4StoreU_Safe(maxV, &refitBox.maximum.x);

				current->mMinX[j] = refitBox.minimum.x;
				current->mMinY[j] = refitBox.minimum.y;
				current->mMinZ[j] = refitBox.minimum.z;
				current->mMaxX[j] = refitBox.maximum.x;
				current->mMaxY[j] = refitBox.maximum.y;
				current->mMaxZ[j] = refitBox.maximum.z;
			}
			else
			{
				PxU32 childOffset = current->getChildOffset(j);
				PX_ASSERT(!(childOffset&3));
				childOffset>>=2;
				PX_ASSERT(childOffset>nb);
				const PxU32 childType = current->getChildType(j);

				const BVDataSwizzledNQ* PX_RESTRICT next = data + childOffset;
				{
					current->mMinX[j] = PxMin(next->mMinX[0], next->mMinX[1]);
					current->mMinY[j] = PxMin(next->mMinY[0], next->mMinY[1]);
					current->mMinZ[j] = PxMin(next->mMinZ[0], next->mMinZ[1]);
					current->mMaxX[j] = PxMax(next->mMaxX[0], next->mMaxX[1]);
					current->mMaxY[j] = PxMax(next->mMaxY[0], next->mMaxY[1]);
					current->mMaxZ[j] = PxMax(next->mMaxZ[0], next->mMaxZ[1]);

					if(childType>0)
					{
						current->mMinX[j] = PxMin(current->mMinX[j], next->mMinX[2]);
						current->mMinY[j] = PxMin(current->mMinY[j], next->mMinY[2]);
						current->mMinZ[j] = PxMin(current->mMinZ[j], next->mMinZ[2]);
						current->mMaxX[j] = PxMax(current->mMaxX[j], next->mMaxX[2]);
						current->mMaxY[j] = PxMax(current->mMaxY[j], next->mMaxY[2]);
						current->mMaxZ[j] = PxMax(current->mMaxZ[j], next->mMaxZ[2]);
					}

					if(childType>1)
					{
						current->mMinX[j] = PxMin(current->mMinX[j], next->mMinX[3]);
						current->mMinY[j] = PxMin(current->mMinY[j], next->mMinY[3]);
						current->mMinZ[j] = PxMin(current->mMinZ[j], next->mMinZ[3]);
						current->mMaxX[j] = PxMax(current->mMaxX[j], next->mMaxX[3]);
						current->mMaxY[j] = PxMax(current->mMaxY[j], next->mMaxY[3]);
						current->mMaxZ[j] = PxMax(current->mMaxZ[j], next->mMaxZ[3]);
					}
				}
			}
		}
	}

	BVDataSwizzledNQ* root = reinterpret_cast<BVDataSwizzledNQ*>(mNodes);
	{
		globalBounds.setEmpty();

		for(PxU32 j=0;j<4;j++)
		{
			if(root->getChildData(j)==PX_INVALID_U32)
				continue;

			PxBounds3 refitBox;
			refitBox.minimum.x = root->mMinX[j];
			refitBox.minimum.y = root->mMinY[j];
			refitBox.minimum.z = root->mMinZ[j];
			refitBox.maximum.x = root->mMaxX[j];
			refitBox.maximum.y = root->mMaxY[j];
			refitBox.maximum.z = root->mMaxZ[j];
			globalBounds.include(refitBox);
		}

		mLocalBounds.init(globalBounds);
	}
	return true;
}
#endif

