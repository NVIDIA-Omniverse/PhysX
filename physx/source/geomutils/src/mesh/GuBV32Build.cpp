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

#include "foundation/PxVec4.h"
#include "foundation/PxBasicTemplates.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxMemory.h"
#include "geometry/PxTriangle.h"

#include "GuBV32Build.h"
#include "GuBV32.h"
#include "GuCenterExtents.h"
#include "GuBV4Build.h"

using namespace physx;
using namespace Gu;

#include "foundation/PxVecMath.h"
using namespace physx::aos;

struct BV32Node : public physx::PxUserAllocated
{
	BV32Node() : mNbChildBVNodes(0)
	{}

	BV32Data	mBVData[32];
	PxU32		mNbChildBVNodes;

	PX_FORCE_INLINE	size_t			isLeaf(PxU32 i)			const	{ return mBVData[i].mData & 1; }
	PX_FORCE_INLINE	PxU32			getPrimitive(PxU32 i)	const	{ return PxU32(mBVData[i].mData >> 1); }
	PX_FORCE_INLINE	const BV32Node*	getChild(PxU32 i)		const	{ return reinterpret_cast<BV32Node*>(mBVData[i].mData); }


	PxU32 getSize() const
	{
		return sizeof(BV32Data)*mNbChildBVNodes;
	}
};


static void fillInNodes(const AABBTreeNode* current_node, const PxU32 startIndex, const PxU32 endIndex, const AABBTreeNode** NODES, PxU32& stat)
{

	if (startIndex + 1 == endIndex)
	{
		//fill in nodes
		const AABBTreeNode* P = current_node->getPos();
		const AABBTreeNode* N = current_node->getNeg();
		NODES[startIndex] = P;
		NODES[endIndex] = N;
		stat += 2;
	}
	else
	{
		const AABBTreeNode* P = current_node->getPos();
		const AABBTreeNode* N = current_node->getNeg();
		const PxU32 midIndex = startIndex + ((endIndex - startIndex) / 2);
		if (!P->isLeaf())
			fillInNodes(P, startIndex, midIndex, NODES, stat);
		else
		{
			NODES[startIndex] = P;
			stat++;
		}

		if (!N->isLeaf())
			fillInNodes(N, midIndex + 1, endIndex, NODES, stat);
		else
		{
			NODES[midIndex + 1] = N;
			stat++;
		}
	}
}



static void setPrimitive(const BV4_AABBTree& source, BV32Node* node32, PxU32 i, const AABBTreeNode* node, float epsilon)
{
	const PxU32 nbPrims = node->getNbPrimitives();
	PX_ASSERT(nbPrims<=32);
	const PxU32* indexBase = source.getIndices();
	const PxU32* prims = node->getPrimitives();
	const PxU32 offset = PxU32(prims - indexBase);
	
#if BV32_VALIDATE
	for (PxU32 j = 0; j<nbPrims; j++)
	{
		PX_ASSERT(prims[j] == offset + j);
	}
#endif
	const PxU32 primitiveIndex = (offset << 6) | (nbPrims & 63);

	node32->mBVData[i].mMin = node->getAABB().minimum;
	node32->mBVData[i].mMax = node->getAABB().maximum;
	if (epsilon != 0.0f)
	{
		node32->mBVData[i].mMin -= PxVec3(epsilon);
		node32->mBVData[i].mMax += PxVec3(epsilon);
	}
	node32->mBVData[i].mData = (primitiveIndex << 1) | 1;
}

static BV32Node* setNode(const BV4_AABBTree& source, BV32Node* node32, PxU32 i, const AABBTreeNode* node, float epsilon)
{
	BV32Node* child = NULL;

	if (node)
	{
		if (node->isLeaf())
		{
			setPrimitive(source, node32, i, node, epsilon);
		}
		else
		{

			node32->mBVData[i].mMin = node->getAABB().minimum;
			node32->mBVData[i].mMax = node->getAABB().maximum;
			if (epsilon != 0.0f)
			{
				node32->mBVData[i].mMin -= PxVec3(epsilon);
				node32->mBVData[i].mMax += PxVec3(epsilon);
			}

			child = PX_NEW(BV32Node);
			node32->mBVData[i].mData = size_t(child);
		}
	}
	
	return child;
}


static void buildBV32(const BV4_AABBTree& source, BV32Node* tmp, const AABBTreeNode* current_node, float epsilon, PxU32& nbNodes)
{
	PX_ASSERT(!current_node->isLeaf());

	const AABBTreeNode* NODES[32];
	PxMemSet(NODES, 0, sizeof(AABBTreeNode*) * 32);

	fillInNodes(current_node, 0, 31, NODES, tmp->mNbChildBVNodes);

	PxU32 left = 0;
	PxU32 right = 31;

	while (left < right)
	{
		
		//sweep from the front
		while (left<right)
		{
			//found a hole
			if (NODES[left] == NULL)
				break;
			left++;
		}

		//sweep from the back
		while (left < right)
		{
			//found a node
			if (NODES[right])
				break;
			right--;
		}

		if (left != right)
		{
			//swap left and right
			const AABBTreeNode* node = NODES[right];
			NODES[right] = NODES[left];
			NODES[left] = node;
		}

	}

	nbNodes += tmp->mNbChildBVNodes;

	for (PxU32 i = 0; i < tmp->mNbChildBVNodes; ++i)
	{
		const AABBTreeNode* tempNode = NODES[i];
		BV32Node* Child = setNode(source, tmp, i, tempNode, epsilon);
		if (Child)
		{
			buildBV32(source, Child, tempNode, epsilon, nbNodes);
		}
	}

}

//
//static void validateTree(const AABBTree& Source, const AABBTreeNode* currentNode)
//{
//	if (currentNode->isLeaf())
//	{
//		const PxU32* indexBase = Source.getIndices();
//		const PxU32* prims = currentNode->getPrimitives();
//		const PxU32 offset = PxU32(prims - indexBase);
//		const PxU32 nbPrims = currentNode->getNbPrimitives();
//		for (PxU32 j = 0; j<nbPrims; j++)
//		{
//			PX_ASSERT(prims[j] == offset + j);
//		}
//	}
//	else
//	{
//		const AABBTreeNode* pos = currentNode->getPos();
//		validateTree(Source, pos);
//		const AABBTreeNode* neg = currentNode->getNeg();
//		validateTree(Source, neg);
//	}
//}

#if BV32_VALIDATE
static void validateNodeBound(const BV32Node* currentNode, SourceMeshBase* mesh, float epsilon)
{
	const PxU32 nbPrimitivesFromMesh = mesh->getNbPrimitives();
	const PxReal eps = 1e-5f;
	const PxU32 nbNodes = currentNode->mNbChildBVNodes;
	for (PxU32 i = 0; i < nbNodes; ++i)
	{
		const BV32Node* node = currentNode->getChild(i);
		if (currentNode->isLeaf(i))
		{
			BV32Data data = currentNode->mBVData[i];
			PxU32 nbPrimitives = data.getNbReferencedPrimitives();
			PxU32 startIndex = data.getPrimitiveStartIndex();

			PX_ASSERT(startIndex< nbPrimitivesFromMesh);

			PxVec3 min(PX_MAX_F32, PX_MAX_F32, PX_MAX_F32);
			PxVec3 max(-PX_MAX_F32, -PX_MAX_F32, -PX_MAX_F32);
			const PxVec3* verts = mesh->getVerts();

			if (mesh->getMeshType() == SourceMeshBase::MeshType::TRI_MESH)
			{
				const IndTri32* triIndices = static_cast<SourceMesh*>(mesh)->getTris32();
				for (PxU32 j = 0; j < nbPrimitives; ++j)
				{
					IndTri32 index = triIndices[startIndex + j];

					for (PxU32 k = 0; k < 3; ++k)
					{
						const PxVec3& v = verts[index.mRef[k]];

						min.x = (min.x > v.x) ? v.x : min.x;
						min.y = (min.y > v.y) ? v.y : min.y;
						min.z = (min.z > v.z) ? v.z : min.z;

						max.x = (max.x < v.x) ? v.x : max.x;
						max.y = (max.y < v.y) ? v.y : max.y;
						max.z = (max.z < v.z) ? v.z : max.z;
					}
				}
			}
			else
			{
				const IndTetrahedron32* tetIndices = static_cast<TetrahedronSourceMesh*>(mesh)->getTetrahedrons32();
				for (PxU32 j = 0; j < nbPrimitives; ++j)
				{
					IndTetrahedron32 index = tetIndices[startIndex + j];

					for (PxU32 k = 0; k < 4; ++k)
					{
						const PxVec3& v = verts[index.mRef[k]];

						min.x = (min.x > v.x) ? v.x : min.x;
						min.y = (min.y > v.y) ? v.y : min.y;
						min.z = (min.z > v.z) ? v.z : min.z;

						max.x = (max.x < v.x) ? v.x : max.x;
						max.y = (max.y < v.y) ? v.y : max.y;
						max.z = (max.z < v.z) ? v.z : max.z;
					}
				}
			}

			PxVec3 dMin, dMax;
			data.getMinMax(dMin, dMax);

			const PxVec3 difMin = min - dMin;
			const PxVec3 difMax = dMax - max;
			PX_ASSERT(PxAbs(difMin.x - epsilon) < eps && PxAbs(difMin.y - epsilon) < eps && PxAbs(difMin.z - epsilon) < eps);
			PX_ASSERT(PxAbs(difMax.x - epsilon) < eps && PxAbs(difMax.y - epsilon) < eps && PxAbs(difMax.z - epsilon) < eps);

		}
		else
		{
			validateNodeBound(node, mesh, epsilon);
		}
	}
}
#endif

static bool BuildBV32Internal(BV32Tree& bv32Tree, const BV4_AABBTree& Source, SourceMeshBase* mesh, float epsilon)
{
	GU_PROFILE_ZONE("..BuildBV32Internal")

	const PxU32 nbPrimitives = mesh->getNbPrimitives();
	if (nbPrimitives <= 32)
	{
		bv32Tree.mNbPackedNodes = 1;
		bv32Tree.mPackedNodes = reinterpret_cast<BV32DataPacked*>(PX_ALLOC(sizeof(BV32DataPacked), "BV32DataPacked"));
		BV32DataPacked& packedData = bv32Tree.mPackedNodes[0];
		packedData.mNbNodes = 1;
		packedData.mMin[0] = PxVec4(Source.getBV().minimum, 0.f);
		packedData.mMax[0] = PxVec4(Source.getBV().maximum, 0.f);
		packedData.mData[0] = (nbPrimitives << 1) | 1;
		bv32Tree.mMaxTreeDepth = 1;
		bv32Tree.mTreeDepthInfo = reinterpret_cast<BV32DataDepthInfo*>(PX_ALLOC(sizeof(BV32DataDepthInfo), "BV32DataDepthInfo"));
		bv32Tree.mRemapPackedNodeIndexWithDepth = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32), "PxU32"));
		bv32Tree.mTreeDepthInfo[0].offset = 0;
		bv32Tree.mTreeDepthInfo[0].count = 1;
		bv32Tree.mRemapPackedNodeIndexWithDepth[0] = 0;

		return bv32Tree.init(mesh, Source.getBV());
	}

	{
		GU_PROFILE_ZONE("...._checkMD")
		struct Local
		{
			static void _checkMD(const AABBTreeNode* current_node, PxU32& md, PxU32& cd)
			{
				cd++;
				md = PxMax(md, cd);

				if (current_node->getPos())	{ _checkMD(current_node->getPos(), md, cd);	cd--; }
				if (current_node->getNeg())	{ _checkMD(current_node->getNeg(), md, cd);	cd--; }
			}

			static void _check(AABBTreeNode* current_node)
			{
				if (current_node->isLeaf())
					return;

				AABBTreeNode* P = const_cast<AABBTreeNode*>(current_node->getPos());
				AABBTreeNode* N = const_cast<AABBTreeNode*>(current_node->getNeg());
				{
					PxU32 MDP = 0;	PxU32 CDP = 0;	_checkMD(P, MDP, CDP);
					PxU32 MDN = 0;	PxU32 CDN = 0;	_checkMD(N, MDN, CDN);

					if (MDP>MDN)
						//					if(MDP<MDN)
					{
						PxSwap(*P, *N);
						PxSwap(P, N);
					}
				}
				_check(P);
				_check(N);
			}
		};
		Local::_check(const_cast<AABBTreeNode*>(Source.getNodes()));
	}


	PxU32 nbNodes = 1;
	BV32Node* Root32 = PX_NEW(BV32Node);

	{
		GU_PROFILE_ZONE("....buildBV32")
		buildBV32(Source, Root32, Source.getNodes(), epsilon, nbNodes);
	}

#if BV32_VALIDATE
	validateNodeBound(Root32, mesh, epsilon);
#endif

	if (!bv32Tree.init(mesh, Source.getBV()))
		return false;
	BV32Tree* T = &bv32Tree;

	PxU32 MaxDepth = 0;

	// Version with variable-sized nodes in single stream
	{
		GU_PROFILE_ZONE("...._flatten")

		struct Local
		{
			static void _flatten(BV32Data* const dest, const PxU32 box_id, PxU32& current_id, const BV32Node* current, PxU32& max_depth, PxU32& current_depth, const PxU32 nb_nodes)
			{
				// Entering a new node => increase depth
				current_depth++;
				// Keep track of max depth
				if (current_depth>max_depth)
					max_depth = current_depth;

				for (PxU32 i = 0; i<current->mNbChildBVNodes; i++)
				{
					dest[box_id + i].mMin = current->mBVData[i].mMin;
					dest[box_id + i].mMax = current->mBVData[i].mMax;
					dest[box_id + i].mData = PxU32(current->mBVData[i].mData);
					dest[box_id + i].mDepth = current_depth;

					PX_ASSERT(box_id + i < nb_nodes);
				}

				PxU32 NbToGo = 0;
				PxU32 NextIDs[32];
				PxMemSet(NextIDs, PX_INVALID_U32, sizeof(PxU32)*32); 
				const BV32Node* ChildNodes[32];
				PxMemSet(ChildNodes, 0, sizeof(BV32Node*)*32);

				BV32Data* data = dest + box_id;
				for (PxU32 i = 0; i<current->mNbChildBVNodes; i++)
				{
					PX_ASSERT(current->mBVData[i].mData != PX_INVALID_U32);

					if (!current->isLeaf(i))
					{

						const BV32Node* ChildNode = current->getChild(i);

						const PxU32 NextID = current_id;

						const PxU32 ChildSize = ChildNode->mNbChildBVNodes;
						current_id += ChildSize;

						const PxU32 ChildType = ChildNode->mNbChildBVNodes << 1;
						data[i].mData = size_t(ChildType + (NextID << GU_BV4_CHILD_OFFSET_SHIFT_COUNT));
						//PX_ASSERT(data[i].mData == size_t(ChildType+(NextID<<3)));

						PX_ASSERT(box_id + i < nb_nodes);

						NextIDs[NbToGo] = NextID;
						ChildNodes[NbToGo] = ChildNode;
						NbToGo++;
					}
				}

			

				for (PxU32 i = 0; i<NbToGo; i++)
				{
					_flatten(dest, NextIDs[i], current_id, ChildNodes[i], max_depth, current_depth, nb_nodes);
					current_depth--;
				}

				PX_DELETE(current);
			}
		};


		PxU32 CurID = Root32->mNbChildBVNodes+1;

		BV32Data* Nodes = PX_NEW(BV32Data)[nbNodes];
		Nodes[0].mMin = Source.getBV().minimum;
		Nodes[0].mMax = Source.getBV().maximum;

		const PxU32 ChildType = Root32->mNbChildBVNodes << 1;
		Nodes[0].mData = size_t(ChildType + (1 << GU_BV4_CHILD_OFFSET_SHIFT_COUNT));

		const PxU32 nbChilden = Nodes[0].getNbChildren();

		PX_UNUSED(nbChilden);


		T->mInitData = CurID;
		
		PxU32 CurrentDepth = 0;

		Local::_flatten(Nodes, 1, CurID, Root32, MaxDepth, CurrentDepth, nbNodes);

		PX_ASSERT(CurID == nbNodes);

		T->mNbNodes = nbNodes;

		T->mNodes = Nodes;
	}

	
	{
		GU_PROFILE_ZONE("....calculateLeafNode")

		BV32Data* nodes = bv32Tree.mNodes;

		for(PxU32 i=0; i<nbNodes; i++)
		{
			BV32Data& node = nodes[i];
			if(!node.isLeaf())
			{
				PxU32 nbChildren = node.getNbChildren();
				PxU32 offset = node.getChildOffset();
				//calculate how many children nodes are leaf nodes
				PxU32 nbLeafNodes = 0;
				while(nbChildren--)
				{
					BV32Data& child = nodes[offset++];
					if(child.isLeaf())
						nbLeafNodes++;
				}

				node.mNbLeafNodes = nbLeafNodes;
			}
		}
	}
	
	bv32Tree.mPackedNodes = PX_ALLOCATE(BV32DataPacked, nbNodes, "BV32DataPacked");
	bv32Tree.mNbPackedNodes = nbNodes;
	bv32Tree.mMaxTreeDepth = MaxDepth;

	PxU32 nbPackedNodes = 1;
	PxU32 currentIndex = bv32Tree.mNodes[0].getNbChildren() - bv32Tree.mNodes[0].mNbLeafNodes + 1;
	BV32DataPacked& packedData = bv32Tree.mPackedNodes[0];
	//BV32DataDepth& depthData = bv32Tree.mMaxDepthForPackedNodes[0];
	{
		GU_PROFILE_ZONE("....createSOAformatNode")
		bv32Tree.createSOAformatNode(packedData, bv32Tree.mNodes[0], 1, currentIndex, nbPackedNodes);
	}

	PX_ASSERT(nbPackedNodes == currentIndex);
	PX_ASSERT(nbPackedNodes > 0);

	bv32Tree.mNbPackedNodes = nbPackedNodes;

#if BV32_VALIDATE

	/*for (PxU32 i = 0; i < nbNodes; ++i)
	{
		BV32Data& iNode = bv32Tree.mNodes[i];
		for (PxU32 j =  i+1; j < nbNodes; ++j)
		{
			BV32Data& jNode = bv32Tree.mNodes[j];
			PX_ASSERT(iNode.mDepth <= jNode.mDepth);
		}
	}*/

#endif

	{
		GU_PROFILE_ZONE("....depth stuff")

		//bv32Tree.mMaxDepthForPackedNodes = reinterpret_cast<BV32DataDepth*>(PX_ALLOC(sizeof(BV32DataDepth)*MaxDepth, "BV32DataDepth"));

		bv32Tree.mTreeDepthInfo = PX_ALLOCATE(BV32DataDepthInfo, MaxDepth, "BV32DataDepthInfo");

		PxU32 totalCount = 0;
		for (PxU32 i = 0; i < MaxDepth; ++i)
		{
			PxU32 count = 0;
			for (PxU32 j = 0; j < nbPackedNodes; ++j)
			{
				BV32DataPacked& jPackedData = bv32Tree.mPackedNodes[j];
				if (jPackedData.mDepth == i)
				{
					count++;
				}
			}

			bv32Tree.mTreeDepthInfo[i].offset = totalCount;
			bv32Tree.mTreeDepthInfo[i].count = count;
			totalCount += count;
		}

		PX_ASSERT(totalCount == nbPackedNodes);
		bv32Tree.mRemapPackedNodeIndexWithDepth = PX_ALLOCATE(PxU32, nbPackedNodes, "PxU32");

		for (PxU32 i = 0; i < MaxDepth; ++i)
		{
			PxU32 count = 0;
			const PxU32 offset = bv32Tree.mTreeDepthInfo[i].offset;
			PxU32* treeDepth = &bv32Tree.mRemapPackedNodeIndexWithDepth[offset];
			for (PxU32 j = 0; j < nbPackedNodes; ++j)
			{
				BV32DataPacked& jPackedData = bv32Tree.mPackedNodes[j];
				if (jPackedData.mDepth == i)
				{
					treeDepth[count++] = j;
				}
			}
		}

#if BV32_VALIDATE
		for (PxU32 i = MaxDepth; i > 0; i--)
		{
			const PxU32 iOffset = bv32Tree.mTreeDepthInfo[i - 1].offset;
			const PxU32 iCount = bv32Tree.mTreeDepthInfo[i - 1].count;
			PxU32* iRempapNodeIndex = &bv32Tree.mRemapPackedNodeIndexWithDepth[iOffset];

			for (PxU32 j = 0; j < iCount; ++j)
			{
				const PxU32 nodeIndex = iRempapNodeIndex[j];
				BV32DataPacked& currentNode = bv32Tree.mPackedNodes[nodeIndex];
				PX_ASSERT(currentNode.mDepth == i - 1);
			}
		
		}
#endif
	}
	return true;
}
/////

struct ReorderData32
{
	//const SourceMesh*	mMesh;
	SourceMeshBase*		mMesh;
	PxU32*				mOrder;
	PxU32				mNbPrimitivesPerLeaf;
	PxU32				mIndex;
	PxU32				mNbPrimitives;
	PxU32				mStats[32];
};

static bool gReorderCallback(const AABBTreeNode* current, PxU32 /*depth*/, void* userData)
{
	ReorderData32* Data = reinterpret_cast<ReorderData32*>(userData);
	if (current->isLeaf())
	{
		const PxU32 n = current->getNbPrimitives();
		PX_ASSERT(n > 0);
		PX_ASSERT(n <= Data->mNbPrimitivesPerLeaf);
		Data->mStats[n-1]++;
		PxU32* Prims = const_cast<PxU32*>(current->getPrimitives());

		for (PxU32 i = 0; i<n; i++)
		{
			PX_ASSERT(Prims[i]<Data->mNbPrimitives);
			Data->mOrder[Data->mIndex] = Prims[i];
			PX_ASSERT(Data->mIndex<Data->mNbPrimitives);
			Prims[i] = Data->mIndex;
			Data->mIndex++;
		}
	}
	return true;
}


bool physx::Gu::BuildBV32Ex(BV32Tree& tree, SourceMeshBase& mesh, float epsilon, PxU32 nbPrimitivesPerLeaf)
{
	const PxU32 nbPrimitives = mesh.getNbPrimitives();

	BV4_AABBTree Source;
	{
		GU_PROFILE_ZONE("..BuildBV32Ex_buildFromMesh")

//		if (!Source.buildFromMesh(mesh, nbPrimitivesPerLeaf, BV4_SPLATTER_POINTS_SPLIT_GEOM_CENTER))
		if (!Source.buildFromMesh(mesh, nbPrimitivesPerLeaf, BV4_SAH))
			return false;
	}

	{
		GU_PROFILE_ZONE("..BuildBV32Ex_remap")

		PxU32* order = PX_ALLOCATE(PxU32, nbPrimitives, "BV32");
		ReorderData32 RD;
		RD.mMesh = &mesh;
		RD.mOrder = order;
		RD.mNbPrimitivesPerLeaf = nbPrimitivesPerLeaf;
		RD.mIndex = 0;
		RD.mNbPrimitives = nbPrimitives;
		for (PxU32 i = 0; i<32; i++)
			RD.mStats[i] = 0;
		Source.walk(gReorderCallback, &RD);
		PX_ASSERT(RD.mIndex == nbPrimitives);
		mesh.remapTopology(order);
		PX_FREE(order);
		//		for(PxU32 i=0;i<16;i++)
		//			printf("%d: %d\n", i, RD.mStats[i]);
	}


	/*if (mesh.getNbPrimitives() <= nbPrimitivesPerLeaf)
		return tree.init(&mesh, Source.getBV());*/

	return BuildBV32Internal(tree, Source, &mesh, epsilon);
}
