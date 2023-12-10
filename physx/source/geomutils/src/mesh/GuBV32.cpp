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

#include "foundation/PxMemory.h"
#include "GuBV32.h"
#include "CmSerialize.h"
#include "CmUtils.h"
#include "foundation/PxUtilities.h"
#include "foundation/PxVecMath.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

BV32Tree::BV32Tree(SourceMesh* meshInterface, const PxBounds3& localBounds)
{
	reset();
	init(meshInterface, localBounds);
}

BV32Tree::BV32Tree()
{
	reset();
}

void BV32Tree::release()
{
	if (!mUserAllocated)
	{
		PX_DELETE_ARRAY(mNodes);
		PX_FREE(mPackedNodes);
		PX_FREE(mTreeDepthInfo);
		PX_FREE(mRemapPackedNodeIndexWithDepth);
	}
	mNodes = NULL;
	mNbNodes = 0;
	mMaxTreeDepth = 0;
}

BV32Tree::~BV32Tree()
{
	release();
}

void BV32Tree::reset()
{
	mMeshInterface = NULL;
	mNbNodes = 0;
	mNodes = NULL;
	mNbPackedNodes = 0;
	mPackedNodes = NULL;
	mMaxTreeDepth = 0;
	mTreeDepthInfo = NULL;
	mRemapPackedNodeIndexWithDepth = NULL;
	mInitData = 0;
	mUserAllocated = false;
}

void BV32Tree::operator=(BV32Tree& v)
{
	mMeshInterface = v.mMeshInterface;
	mLocalBounds = v.mLocalBounds;
	mNbNodes = v.mNbNodes;
	mNodes = v.mNodes;
	mInitData = v.mInitData;
	mUserAllocated = v.mUserAllocated;
	v.reset();
}

bool BV32Tree::init(SourceMeshBase* meshInterface, const PxBounds3& localBounds)
{
	mMeshInterface = meshInterface;
	mLocalBounds.init(localBounds);
	return true;
}

// PX_SERIALIZATION
BV32Tree::BV32Tree(const PxEMPTY)
{
	mUserAllocated = true;
}

void BV32Tree::exportExtraData(PxSerializationContext& stream)
{
	stream.alignData(16);
	stream.writeData(mPackedNodes, mNbPackedNodes*sizeof(BV32DataPacked));
}

void BV32Tree::importExtraData(PxDeserializationContext& context)
{
	context.alignExtraData(16);
	mPackedNodes = context.readExtraData<BV32DataPacked>(mNbPackedNodes);
}
//~PX_SERIALIZATION

bool BV32Tree::load(PxInputStream& stream, bool mismatch_)
{
	PX_ASSERT(!mUserAllocated);

	release();

	PxI8 a, b, c, d;
	readChunk(a, b, c, d, stream);
	if(a != 'B' || b != 'V' || c != '3' || d != '2')
		return false;

	bool mismatch;
	PxU32 fileVersion;
	if(!readBigEndianVersionNumber(stream, mismatch_, fileVersion, mismatch))
		return false;

	mLocalBounds.mCenter.x = readFloat(mismatch, stream);
	mLocalBounds.mCenter.y = readFloat(mismatch, stream);
	mLocalBounds.mCenter.z = readFloat(mismatch, stream);
	mLocalBounds.mExtentsMagnitude = readFloat(mismatch, stream);

	mInitData = readDword(mismatch, stream);

	/*const PxU32 nbNodes = readDword(mismatch, stream);
	mNbNodes = nbNodes;

	if (nbNodes)
	{
		BV32Data* nodes = PX_NEW(BV32Data)[nbNodes];

		mNodes = nodes;
		PxMarkSerializedMemory(nodes, sizeof(BV32Data)*nbNodes);

		for (PxU32 i = 0; i<nbNodes; i++)
		{
			BV32Data& node = nodes[i];

			readFloatBuffer(&node.mCenter.x, 3, mismatch, stream);
			node.mData = readDword(mismatch, stream);
			readFloatBuffer(&node.mExtents.x, 3, mismatch, stream);
		}
	}*/

	//read SOA format node data
	const PxU32 nbPackedNodes = readDword(mismatch, stream);
	mNbPackedNodes = nbPackedNodes;

	if (nbPackedNodes)
	{
		mPackedNodes = reinterpret_cast<BV32DataPacked*>(PX_ALLOC(sizeof(BV32DataPacked)*nbPackedNodes, "BV32DataPacked"));

		PxMarkSerializedMemory(mPackedNodes, sizeof(BV32DataPacked)*nbPackedNodes);

		for (PxU32 i = 0; i < nbPackedNodes; ++i)
		{
			BV32DataPacked& node = mPackedNodes[i];
			node.mNbNodes = readDword(mismatch, stream);
			PX_ASSERT(node.mNbNodes > 0);
			node.mDepth = readDword(mismatch, stream);
			ReadDwordBuffer(node.mData, node.mNbNodes, mismatch, stream);
			const PxU32 nbElements = 4 * node.mNbNodes;
			readFloatBuffer(&node.mMin[0].x, nbElements, mismatch, stream);
			readFloatBuffer(&node.mMax[0].x, nbElements, mismatch, stream);
			
		}
	}

	const PxU32 maxTreeDepth = readDword(mismatch, stream);
	mMaxTreeDepth = maxTreeDepth;

	if (maxTreeDepth > 0)
	{
		mTreeDepthInfo = reinterpret_cast<BV32DataDepthInfo*>(PX_ALLOC(sizeof(BV32DataDepthInfo)*maxTreeDepth, "BV32DataDepthInfo"));

		for (PxU32 i = 0; i < maxTreeDepth; ++i)
		{
			BV32DataDepthInfo& info = mTreeDepthInfo[i];

			info.offset = readDword(mismatch, stream);
			info.count = readDword(mismatch, stream);
		}

		mRemapPackedNodeIndexWithDepth = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*nbPackedNodes, "PxU32"));

		ReadDwordBuffer(mRemapPackedNodeIndexWithDepth, nbPackedNodes, mismatch, stream);
	}
	return true;
}

void BV32Tree::createSOAformatNode(BV32DataPacked& packedData, 
	const BV32Data& node, const PxU32 childOffset, PxU32& currentIndex, PxU32& nbPackedNodes)
{
	//found the next 32 nodes and fill it in SOA format
	
	const PxU32 nbChildren = node.getNbChildren();
	const PxU32 offset = node.getChildOffset();

	packedData.mDepth = node.mDepth;

	for (PxU32 i = 0; i < nbChildren; ++i)
	{
		BV32Data& child = mNodes[offset + i];

		packedData.mMin[i] = PxVec4(child.mMin, 0.f);
		packedData.mMax[i] = PxVec4(child.mMax, 0.f);
		packedData.mData[i] = PxU32(child.mData);
	}

	packedData.mNbNodes = nbChildren;
	
	PxU32 NbToGo = 0;
	PxU32 NextIDs[32];
	PxMemSet(NextIDs, PX_INVALID_U32, sizeof(PxU32) * 32);
	const BV32Data* ChildNodes[32];
	PxMemSet(ChildNodes, 0, sizeof(BV32Data*) * 32);
	
	for (PxU32 i = 0; i< nbChildren; i++)
	{
		BV32Data& child = mNodes[offset + i];

		if (!child.isLeaf())
		{
			const PxU32 NextID = currentIndex;

			const PxU32 ChildSize = child.getNbChildren() - child.mNbLeafNodes;
			currentIndex += ChildSize;

			//packedData.mData[i] = (packedData.mData[i] & ((1 << GU_BV4_CHILD_OFFSET_SHIFT_COUNT) - 1)) | (NextID << GU_BV4_CHILD_OFFSET_SHIFT_COUNT);
			packedData.mData[i] = (packedData.mData[i] & ((1 << GU_BV4_CHILD_OFFSET_SHIFT_COUNT) - 1)) | ((childOffset + NbToGo) << GU_BV4_CHILD_OFFSET_SHIFT_COUNT);

			NextIDs[NbToGo] = NextID;
			ChildNodes[NbToGo] = &child;
			NbToGo++;
		}
	}

	nbPackedNodes += NbToGo;
	for (PxU32 i = 0; i < NbToGo; ++i)
	{
		const BV32Data& child = *ChildNodes[i];
	
		BV32DataPacked& childData = mPackedNodes[childOffset+i];
		
		createSOAformatNode(childData, child, NextIDs[i], currentIndex, nbPackedNodes);
	}
}

bool BV32Tree::refit(float epsilon)
{
	using namespace physx::aos;

	if (!mPackedNodes)
	{
		PxBounds3 bounds;
		bounds.setEmpty();
		if (mMeshInterface)
		{
			PxU32 nbVerts = mMeshInterface->getNbVertices();
			const PxVec3* verts = mMeshInterface->getVerts();
			while (nbVerts--)
				bounds.include(*verts++);
			mLocalBounds.init(bounds);
		}
		return true;
	}

	class PxBounds3Padded : public PxBounds3
	{
	public:
		PX_FORCE_INLINE PxBounds3Padded() {}
		PX_FORCE_INLINE ~PxBounds3Padded() {}
		PxU32	padding;
	};

	PxU32 nb = mNbPackedNodes;

	while (nb--)
	{
		BV32DataPacked* PX_RESTRICT current = mPackedNodes + nb;
		const PxU32 nbChildren = current->mNbNodes;

		for (PxU32 j = 0; j< nbChildren; j++)
		{
			if (current->isLeaf(j))
			{
				PxU32 nbTets = current->getNbReferencedPrimitives(j);
				PxU32 primIndex = current->getPrimitiveStartIndex(j);

				Vec4V minV = V4Load(FLT_MAX);
				Vec4V maxV = V4Load(-FLT_MAX);

				//TetrahedronPointers
				do
				{
					PX_ASSERT(primIndex< mMeshInterface->getNbPrimitives());

					//meshInterface->getTriangle(VP, primIndex);
					Vec4V tMin, tMax;
					mMeshInterface->getPrimitiveBox(primIndex, tMin, tMax);
					minV = V4Min(minV, tMin);
					maxV = V4Max(maxV, tMax);

					primIndex++;
				} while (--nbTets);

				const Vec4V epsilonV = V4Load(epsilon);
				minV = V4Sub(minV, epsilonV);
				maxV = V4Add(maxV, epsilonV);

				PxBounds3Padded refitBox;
				V4StoreU_Safe(minV, &refitBox.minimum.x);
				V4StoreU_Safe(maxV, &refitBox.maximum.x);

				current->mMin[j].x = refitBox.minimum.x;
				current->mMin[j].y = refitBox.minimum.y;
				current->mMin[j].z = refitBox.minimum.z;
				current->mMax[j].x = refitBox.maximum.x;
				current->mMax[j].y = refitBox.maximum.y;
				current->mMax[j].z = refitBox.maximum.z;
			}
			else
			{
				PxU32 childOffset = current->getChildOffset(j);

				PX_ASSERT(childOffset < mNbPackedNodes);
				
				BV32DataPacked* next = mPackedNodes + childOffset;

				const PxU32 nextNbChilds = next->mNbNodes;

				Vec4V minV = V4Load(FLT_MAX);
				Vec4V maxV = V4Load(-FLT_MAX);

				for (PxU32 a = 0; a < nextNbChilds; ++a)
				{
					const Vec4V tMin = V4LoadU(&next->mMin[a].x);
					const Vec4V tMax = V4LoadU(&next->mMax[a].x);

					minV = V4Min(minV, tMin);
					maxV = V4Max(maxV, tMax);
				}

				PxBounds3Padded refitBox;
				V4StoreU_Safe(minV, &refitBox.minimum.x);
				V4StoreU_Safe(maxV, &refitBox.maximum.x);

				current->mMin[j].x = refitBox.minimum.x;
				current->mMin[j].y = refitBox.minimum.y;
				current->mMin[j].z = refitBox.minimum.z;

				current->mMax[j].x = refitBox.maximum.x;
				current->mMax[j].y = refitBox.maximum.y;
				current->mMax[j].z = refitBox.maximum.z;
			}
		}
	}

	BV32DataPacked* root = mPackedNodes;
	{
		PxBounds3 globalBounds;
		globalBounds.setEmpty();

		const PxU32 nbChildren = root->mNbNodes;

		Vec4V minV = V4Load(FLT_MAX);
		Vec4V maxV = V4Load(-FLT_MAX);

		for (PxU32 a = 0; a < nbChildren; ++a)
		{
			const Vec4V tMin = V4LoadU(&root->mMin[a].x);
			const Vec4V tMax = V4LoadU(&root->mMax[a].x);

			minV = V4Min(minV, tMin);
			maxV = V4Max(maxV, tMax);
		}

		PxBounds3Padded refitBox;
		V4StoreU_Safe(minV, &refitBox.minimum.x);
		V4StoreU_Safe(maxV, &refitBox.maximum.x);
		globalBounds.minimum.x = refitBox.minimum.x;
		globalBounds.minimum.y = refitBox.minimum.y;
		globalBounds.minimum.z = refitBox.minimum.z;

		globalBounds.maximum.x = refitBox.maximum.x;
		globalBounds.maximum.y = refitBox.maximum.y;
		globalBounds.maximum.z = refitBox.maximum.z;

		mLocalBounds.init(globalBounds);
	}
	return true;
}
