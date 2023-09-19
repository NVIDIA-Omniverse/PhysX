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

#include "foundation/PxVec4.h"
#include "foundation/PxMemory.h"
#include "GuAABBTreeBuildStats.h"
#include "GuAABBTree.h"
#include "GuSAH.h"
#include "GuBounds.h"
#include "GuBV4Build.h"
#include "GuBV4.h"
#include <stdio.h>

using namespace physx;
using namespace Gu;

#include "foundation/PxVecMath.h"
using namespace physx::aos;

#define GU_BV4_USE_NODE_POOLS

static PX_FORCE_INLINE PxU32 largestAxis(const PxVec4& v)
{
	const float* Vals = &v.x;
	PxU32 m = 0;
	if(Vals[1] > Vals[m]) m = 1;
	if(Vals[2] > Vals[m]) m = 2;
	return m;
}

BV4_AABBTree::BV4_AABBTree() : mIndices(NULL), mPool(NULL), mTotalNbNodes(0)
{
}

BV4_AABBTree::~BV4_AABBTree()
{
	release();
}

void BV4_AABBTree::release()
{
	PX_DELETE_ARRAY(mPool);
	PX_FREE(mIndices);
}

namespace
{
	struct BuildParams
	{
		PX_FORCE_INLINE	BuildParams(const PxBounds3* boxes, const PxVec3* centers, const AABBTreeNode* const node_base, const PxU32 limit, const SourceMesh* mesh) :
			mBoxes(boxes), mCenters(centers), mNodeBase(node_base), mLimit(limit), mMesh(mesh)	{}

		const PxBounds3*			mBoxes;
		const PxVec3*				mCenters;
		const AABBTreeNode* const	mNodeBase;
		const PxU32					mLimit;
		const SourceMesh*			mMesh;
		PX_NOCOPY(BuildParams)
	};
}

static PxU32 local_Split(const AABBTreeNode* PX_RESTRICT node, const PxBounds3* PX_RESTRICT /*Boxes*/, const PxVec3* PX_RESTRICT centers, PxU32 axis, const BuildParams& params)
{
	const PxU32 nb = node->mNbPrimitives;
	PxU32* PX_RESTRICT prims = node->mNodePrimitives;

	// Get node split value
	float splitValue = 0.0f;
	if(params.mMesh)
	{
		VertexPointers VP;
		for(PxU32 i=0;i<nb;i++)
		{
			params.mMesh->getTriangle(VP, prims[i]);
			splitValue += (*VP.Vertex[0])[axis];
			splitValue += (*VP.Vertex[1])[axis];
			splitValue += (*VP.Vertex[2])[axis];
		}
		splitValue /= float(nb*3);
	}
	else
		splitValue = node->mBV.getCenter(axis);

	return reshuffle(nb, prims, centers, splitValue, axis);
}

static bool local_Subdivide(AABBTreeNode* PX_RESTRICT node, BuildStats& stats, const BuildParams& params)
{
	const PxU32* PX_RESTRICT prims = node->mNodePrimitives;
	const PxU32 nb = node->mNbPrimitives;

	const PxBounds3* PX_RESTRICT boxes = params.mBoxes;
	const PxVec3* PX_RESTRICT centers = params.mCenters;

	// Compute bv & means at the same time
	Vec4V meansV;
	{
		Vec4V minV = V4LoadU(&boxes[prims[0]].minimum.x);
		Vec4V maxV = V4LoadU(&boxes[prims[0]].maximum.x);
		meansV = V4LoadU(&centers[prims[0]].x);

		for(PxU32 i=1;i<nb;i++)
		{
			const PxU32 index = prims[i];
			minV = V4Min(minV, V4LoadU(&boxes[index].minimum.x));
			maxV = V4Max(maxV, V4LoadU(&boxes[index].maximum.x));
			meansV = V4Add(meansV, V4LoadU(&centers[index].x));
		}
		const float coeffNb = 1.0f/float(nb);
		meansV = V4Scale(meansV, FLoad(coeffNb));

//		BV4_ALIGN16(PxVec4 mergedMin);
//		BV4_ALIGN16(PxVec4 mergedMax);
		PX_ALIGN_PREFIX(16) PxVec4 mergedMin PX_ALIGN_SUFFIX(16);
		PX_ALIGN_PREFIX(16) PxVec4 mergedMax PX_ALIGN_SUFFIX(16);

		V4StoreA_Safe(minV, &mergedMin.x);
		V4StoreA_Safe(maxV, &mergedMax.x);
		node->mBV.minimum = PxVec3(mergedMin.x, mergedMin.y, mergedMin.z);
		node->mBV.maximum = PxVec3(mergedMax.x, mergedMax.y, mergedMax.z);
	}

#ifndef GU_BV4_FILL_GAPS
//	// Stop subdividing if we reach a leaf node. This is always performed here,
//	// else we could end in trouble if user overrides this.
//	if(nb==1)
//		return false;
	if(nb<=params.mLimit)
		return false;
#endif

	bool validSplit = true;
	PxU32 nbPos;
	{
		// Compute variances
		Vec4V varsV = V4Zero();
		for(PxU32 i=0;i<nb;i++)
		{
			const PxU32 index = prims[i];
			Vec4V centerV = V4LoadU(&centers[index].x);
			centerV = V4Sub(centerV, meansV);
			centerV = V4Mul(centerV, centerV);
			varsV = V4Add(varsV, centerV);
		}
		const float coeffNb1 = 1.0f/float(nb-1);
		varsV = V4Scale(varsV, FLoad(coeffNb1));

//		BV4_ALIGN16(PxVec4 vars);
		PX_ALIGN_PREFIX(16) PxVec4 vars PX_ALIGN_SUFFIX(16);
		V4StoreA_Safe(varsV, &vars.x);

		// Choose axis with greatest variance
		const PxU32 axis = largestAxis(vars);

		// Split along the axis
		nbPos = local_Split(node, boxes, centers, axis, params);

		// Check split validity
		if(!nbPos || nbPos==nb)
			validSplit = false;
	}

	// Check the subdivision has been successful
	if(!validSplit)
	{
		// Here, all boxes lie in the same sub-space. Two strategies:
		// - if the tree *must* be complete, make an arbitrary 50-50 split
		// - else stop subdividing
//		if(nb>limit)
		{
			nbPos = node->mNbPrimitives>>1;

			if(1)
			{
				// Test 3 axes, take the best
				float results[3];
				nbPos = local_Split(node, boxes, centers, 0, params);	results[0] = float(nbPos)/float(node->mNbPrimitives);
				nbPos = local_Split(node, boxes, centers, 1, params);	results[1] = float(nbPos)/float(node->mNbPrimitives);
				nbPos = local_Split(node, boxes, centers, 2, params);	results[2] = float(nbPos)/float(node->mNbPrimitives);
				results[0]-=0.5f;	results[0]*=results[0];
				results[1]-=0.5f;	results[1]*=results[1];
				results[2]-=0.5f;	results[2]*=results[2];
				PxU32 Min=0;
				if(results[1]<results[Min])	Min = 1;
				if(results[2]<results[Min])	Min = 2;

				// Split along the axis
				nbPos = local_Split(node, boxes, centers, Min, params);

				// Check split validity
				if(!nbPos || nbPos==node->mNbPrimitives)
					nbPos = node->mNbPrimitives>>1;
			}
		}
		//else return
	}

#ifdef GU_BV4_FILL_GAPS
	// We split the node a last time before returning when we're below the limit, for the "fill the gaps" strategy
	if(nb<=params.mLimit)
	{
		node->mNextSplit = nbPos;
		return false;
	}
#endif

	// Now create children and assign their pointers.
	// We use a pre-allocated linear pool for complete trees [Opcode 1.3]
	const PxU32 count = stats.getCount();
	node->mPos = size_t(params.mNodeBase + count);

	// Update stats
	stats.increaseCount(2);

	// Assign children
	AABBTreeNode* pos = const_cast<AABBTreeNode*>(node->getPos());
	AABBTreeNode* neg = const_cast<AABBTreeNode*>(node->getNeg());
	pos->mNodePrimitives	= node->mNodePrimitives;
	pos->mNbPrimitives		= nbPos;
	neg->mNodePrimitives	= node->mNodePrimitives + nbPos;
	neg->mNbPrimitives		= node->mNbPrimitives - nbPos;
	return true;
}

static bool local_Subdivide_SAH(AABBTreeNode* PX_RESTRICT node, BuildStats& stats, const BuildParams& params, SAH_Buffers& buffers)
{
	const PxU32* prims = node->mNodePrimitives;
	const PxU32 nb = node->mNbPrimitives;

	const PxBounds3* PX_RESTRICT boxes = params.mBoxes;
	const PxVec3* PX_RESTRICT centers = params.mCenters;

	// Compute bv
	computeGlobalBox(node->mBV, nb, boxes, prims);

#ifndef GU_BV4_FILL_GAPS
//	// Stop subdividing if we reach a leaf node. This is always performed here,
//	// else we could end in trouble if user overrides this.
//	if(nb==1)
//		return false;
	if(nb<=params.mLimit)
		return false;
#endif

	PxU32 leftCount;
	if(!buffers.split(leftCount, nb, prims, boxes, centers))
	{
		// Invalid split => fallback to previous strategy
		return local_Subdivide(node, stats, params);
	}

#ifdef GU_BV4_FILL_GAPS
	// We split the node a last time before returning when we're below the limit, for the "fill the gaps" strategy
	if(nb<=params.mLimit)
	{
		node->mNextSplit = leftCount;
		return false;
	}
#endif

	// Now create children and assign their pointers.
	// We use a pre-allocated linear pool for complete trees [Opcode 1.3]
	const PxU32 count = stats.getCount();
	node->mPos = size_t(params.mNodeBase + count);

	// Update stats
	stats.increaseCount(2);

	// Assign children
	AABBTreeNode* pos = const_cast<AABBTreeNode*>(node->getPos());
	AABBTreeNode* neg = const_cast<AABBTreeNode*>(node->getNeg());
	pos->mNodePrimitives	= node->mNodePrimitives;
	pos->mNbPrimitives		= leftCount;
	neg->mNodePrimitives	= node->mNodePrimitives + leftCount;
	neg->mNbPrimitives		= node->mNbPrimitives - leftCount;
	return true;
}

// PT: TODO: consider local_BuildHierarchy & local_BuildHierarchy_SAH

static void local_BuildHierarchy(AABBTreeNode* PX_RESTRICT node, BuildStats& stats, const BuildParams& params)
{
	if(local_Subdivide(node, stats, params))
	{
		AABBTreeNode* pos = const_cast<AABBTreeNode*>(node->getPos());
		AABBTreeNode* neg = const_cast<AABBTreeNode*>(node->getNeg());
		local_BuildHierarchy(pos, stats, params);
		local_BuildHierarchy(neg, stats, params);
	}
}

static void local_BuildHierarchy_SAH(AABBTreeNode* PX_RESTRICT node, BuildStats& stats, const BuildParams& params, SAH_Buffers& buffers)
{
	if(local_Subdivide_SAH(node, stats, params, buffers))
	{
		AABBTreeNode* pos = const_cast<AABBTreeNode*>(node->getPos());
		AABBTreeNode* neg = const_cast<AABBTreeNode*>(node->getNeg());
		local_BuildHierarchy_SAH(pos, stats, params, buffers);
		local_BuildHierarchy_SAH(neg, stats, params, buffers);
	}
}

bool BV4_AABBTree::buildFromMesh(SourceMeshBase& mesh, PxU32 limit, BV4_BuildStrategy strategy)
{
	const PxU32 nbBoxes = mesh.getNbPrimitives();
	if(!nbBoxes)
		return false;
	PxBounds3* boxes = PX_ALLOCATE(PxBounds3, (nbBoxes + 1), "BV4");	// PT: +1 to safely V4Load/V4Store the last element
	PxVec3* centers = PX_ALLOCATE(PxVec3, (nbBoxes + 1), "BV4");		// PT: +1 to safely V4Load/V4Store the last element
	const FloatV halfV = FLoad(0.5f);
	for (PxU32 i = 0; i<nbBoxes; i++)
	{
		Vec4V minV, maxV;
		mesh.getPrimitiveBox(i, minV, maxV);

		V4StoreU_Safe(minV, &boxes[i].minimum.x);	// PT: safe because 'maximum' follows 'minimum'
		V4StoreU_Safe(maxV, &boxes[i].maximum.x);	// PT: safe because we allocated one more box

		const Vec4V centerV = V4Scale(V4Add(maxV, minV), halfV);
		V4StoreU_Safe(centerV, &centers[i].x);	// PT: safe because we allocated one more PxVec3
	}

	{
		// Release previous tree
		release();

		// Init stats
		BuildStats Stats;
		Stats.setCount(1);

		// Initialize indices. This list will be modified during build.
		mIndices = PX_ALLOCATE(PxU32, nbBoxes, "BV4 indices");
		// Identity permutation
		for (PxU32 i = 0; i<nbBoxes; i++)
			mIndices[i] = i;

		// Use a linear array for complete trees (since we can predict the final number of nodes) [Opcode 1.3]
		// Allocate a pool of nodes
		// PT: TODO: optimize memory here (TA34704)
		mPool = PX_NEW(AABBTreeNode)[nbBoxes * 2 - 1];

		// Setup initial node. Here we have a complete permutation of the app's primitives.
		mPool->mNodePrimitives = mIndices;
		mPool->mNbPrimitives = nbBoxes;

		// Build the hierarchy
		if(strategy==BV4_SPLATTER_POINTS||strategy==BV4_SPLATTER_POINTS_SPLIT_GEOM_CENTER)
		{
			// PT: not sure what the equivalent would be for tet-meshes here
			SourceMesh* triMesh = NULL;
			if(strategy==BV4_SPLATTER_POINTS_SPLIT_GEOM_CENTER)
			{
				if(mesh.getMeshType()==SourceMeshBase::TRI_MESH)
					triMesh = static_cast<SourceMesh*>(&mesh);
			}
			local_BuildHierarchy(mPool, Stats, BuildParams(boxes, centers, mPool, limit, triMesh));
		}
		else if(strategy==BV4_SAH)
		{
			SAH_Buffers sah(nbBoxes);
			local_BuildHierarchy_SAH(mPool, Stats, BuildParams(boxes, centers, mPool, limit, NULL), sah);
		}
		else
			return false;

		// Get back total number of nodes
		mTotalNbNodes = Stats.getCount();
	}

	PX_FREE(centers);
	PX_FREE(boxes);

	if(0)
		printf("Tree depth: %d\n", walk(NULL, NULL));

	return true;
}

PxU32 BV4_AABBTree::walk(WalkingCallback cb, void* userData) const
{
	// Call it without callback to compute max depth
	PxU32 maxDepth = 0;
	PxU32 currentDepth = 0;

	struct Local
	{
		static void _walk(const AABBTreeNode* current_node, PxU32& max_depth, PxU32& current_depth, WalkingCallback callback, void* userData_)
		{
			// Checkings
			if(!current_node)
				return;
			// Entering a new node => increase depth
			current_depth++;
			// Keep track of max depth
			if(current_depth>max_depth)
				max_depth = current_depth;

			// Callback
			if(callback && !(callback)(current_node, current_depth, userData_))
				return;

			// Recurse
			if(current_node->getPos())	{ _walk(current_node->getPos(), max_depth, current_depth, callback, userData_);	current_depth--;	}
			if(current_node->getNeg())	{ _walk(current_node->getNeg(), max_depth, current_depth, callback, userData_);	current_depth--;	}
		}
	};
	Local::_walk(mPool, maxDepth, currentDepth, cb, userData);
	return maxDepth;
}

PxU32 BV4_AABBTree::walkDistance(WalkingCallback cb, WalkingDistanceCallback cb2, void* userData) const
{
	// Call it without callback to compute max depth
	PxU32 maxDepth = 0;
	PxU32 currentDepth = 0;

	struct Local
	{
		static void _walk(const AABBTreeNode* current_node, PxU32& max_depth, PxU32& current_depth, WalkingCallback callback, WalkingDistanceCallback distanceCheck, void* userData_)
		{
			// Checkings
			if (!current_node)
				return;
			// Entering a new node => increase depth
			current_depth++;
			// Keep track of max depth
			if (current_depth > max_depth)
				max_depth = current_depth;

			// Callback
			if (callback && !(callback)(current_node, current_depth, userData_))
				return;

			// Recurse
			bool posHint = distanceCheck && (distanceCheck)(current_node, userData_);
			if (posHint)
			{
				if (current_node->getPos()) { _walk(current_node->getPos(), max_depth, current_depth, callback, distanceCheck, userData_);	current_depth--; }
				if (current_node->getNeg()) { _walk(current_node->getNeg(), max_depth, current_depth, callback, distanceCheck, userData_);	current_depth--; }
			}
			else
			{
				if (current_node->getNeg()) { _walk(current_node->getNeg(), max_depth, current_depth, callback, distanceCheck, userData_);	current_depth--; }
				if (current_node->getPos()) { _walk(current_node->getPos(), max_depth, current_depth, callback, distanceCheck, userData_);	current_depth--; }
			}
		}
	};
	Local::_walk(mPool, maxDepth, currentDepth, cb, cb2, userData);
	return maxDepth;
}



#include "GuBV4_Internal.h"

#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
// PT: see http://www.codercorner.com/blog/?p=734
static PxU32 precomputeNodeSorting(const PxBounds3& box0, const PxBounds3& box1)
{
	const PxVec3 C0 = box0.getCenter();
	const PxVec3 C1 = box1.getCenter();

	PxVec3 dirPPP(1.0f, 1.0f, 1.0f);	dirPPP.normalize();
	PxVec3 dirPPN(1.0f, 1.0f, -1.0f);	dirPPN.normalize();
	PxVec3 dirPNP(1.0f, -1.0f, 1.0f);	dirPNP.normalize();
	PxVec3 dirPNN(1.0f, -1.0f, -1.0f);	dirPNN.normalize();
	PxVec3 dirNPP(-1.0f, 1.0f, 1.0f);	dirNPP.normalize();
	PxVec3 dirNPN(-1.0f, 1.0f, -1.0f);	dirNPN.normalize();
	PxVec3 dirNNP(-1.0f, -1.0f, 1.0f);	dirNNP.normalize();
	PxVec3 dirNNN(-1.0f, -1.0f, -1.0f);	dirNNN.normalize();

	const PxVec3 deltaC = C0 - C1;
	const bool bPPP = deltaC.dot(dirPPP)<0.0f;
	const bool bPPN = deltaC.dot(dirPPN)<0.0f;
	const bool bPNP = deltaC.dot(dirPNP)<0.0f;
	const bool bPNN = deltaC.dot(dirPNN)<0.0f;
	const bool bNPP = deltaC.dot(dirNPP)<0.0f;
	const bool bNPN = deltaC.dot(dirNPN)<0.0f;
	const bool bNNP = deltaC.dot(dirNNP)<0.0f;
	const bool bNNN = deltaC.dot(dirNNN)<0.0f;

	PxU32 code = 0;
	if(!bPPP)
		code |= (1<<7);	// Bit 0: PPP
	if(!bPPN)
		code |= (1<<6);	// Bit 1: PPN
	if(!bPNP)
		code |= (1<<5);	// Bit 2: PNP	
	if(!bPNN)
		code |= (1<<4);	// Bit 3: PNN
	if(!bNPP)
		code |= (1<<3);	// Bit 4: NPP
	if(!bNPN)
		code |= (1<<2);	// Bit 5: NPN
	if(!bNNP)
		code |= (1<<1);	// Bit 6: NNP
	if(!bNNN)
		code |= (1<<0);	// Bit 7: NNN
	return code;
}
#endif

#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Common.h"
#endif

// PT: warning, not the same as CenterExtents::setEmpty()
static void setEmpty(CenterExtents& box)
{
	box.mCenter = PxVec3(0.0f, 0.0f, 0.0f);
	box.mExtents = PxVec3(-1.0f, -1.0f, -1.0f);
}

#define PX_INVALID_U64	0xffffffffffffffff

// Data:
// 1 bit for leaf/no leaf
// 2 bits for child-node type
// 8 bits for PNS
// => 32 - 1 - 2 - 8 = 21 bits left for encoding triangle index or node *offset*
// => limited to 2.097.152 triangles
// => and 2Mb-large trees (this one may not work out well in practice)
// ==> lines marked with //* have been changed to address this. Now we don't store offsets in bytes directly
// but in BVData indices. There's more work at runtime calculating addresses, but now the format can support
// 2 million single nodes.
//
// That being said we only need 3*8 = 24 bits in total, so that could be only 6 bits in each BVData.
// For type0: we have 2 nodes, we need 8 bits        => 6 bits/node = 12 bits available, ok
// For type1: we have 3 nodes, we need 8*2 = 16 bits => 6 bits/node = 18 bits available, ok
// For type2: we have 4 nodes, we need 8*3 = 24 bits => 6 bits/node = 24 bits available, ok
//#pragma pack(1)
struct BVData : public physx::PxUserAllocated
{
	BVData();
	CenterExtents	mAABB;
	size_t			mData64;
#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
	PxU32			mTempPNS;
#endif
};
//#pragma pack()

BVData::BVData() : mData64(PX_INVALID_U64)
{
	setEmpty(mAABB);
#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
	mTempPNS = 0;
#endif
}

struct BV4Node : public physx::PxUserAllocated
{
	PX_FORCE_INLINE	BV4Node()	{}
	PX_FORCE_INLINE	~BV4Node()	{}

	BVData	mBVData[4];

	PX_FORCE_INLINE	size_t			isLeaf(PxU32 i)			const	{ return mBVData[i].mData64 & 1;							}
	PX_FORCE_INLINE	PxU32			getPrimitive(PxU32 i)	const	{ return PxU32(mBVData[i].mData64 >> 1);					}
	PX_FORCE_INLINE	const BV4Node*	getChild(PxU32 i)		const	{ return reinterpret_cast<BV4Node*>(mBVData[i].mData64);	}

	PxU32	getType()	const
	{
		PxU32 Nb=0;
		for(PxU32 i=0;i<4;i++)
		{
			if(mBVData[i].mData64!=PX_INVALID_U64)
				Nb++;
		}
		return Nb;
	}

	PxU32	getSize()	const
	{
		const PxU32 type = getType();
		return sizeof(BVData)*type;
	}
};

#define NB_NODES_PER_SLAB	256
struct BV4BuildParams
{
	PX_FORCE_INLINE	BV4BuildParams(const BV4_AABBTree& source, const SourceMesh* mesh, float epsilon) : mSource(source), mMesh(mesh), mEpsilon(epsilon)
#ifdef GU_BV4_USE_NODE_POOLS
		,mTop(NULL)
#endif
	{}
					~BV4BuildParams();

	const BV4_AABBTree&	mSource;
	const SourceMesh*	mMesh;

	// Stats
	PxU32			mNbNodes;
	PxU32			mStats[4];

	//
	float			mEpsilon;

#ifdef GU_BV4_USE_NODE_POOLS
	//
	struct Slab : public physx::PxUserAllocated
	{
		BV4Node	mNodes[NB_NODES_PER_SLAB];
		PxU32	mNbUsedNodes;
		Slab*	mNext;
	};
	Slab*			mTop;

	BV4Node*		allocateNode();
	void			releaseNodes();
#endif

	PX_NOCOPY(BV4BuildParams)
};

BV4BuildParams::~BV4BuildParams()
{
#ifdef GU_BV4_USE_NODE_POOLS
	releaseNodes();
#endif
}

#ifdef GU_BV4_USE_NODE_POOLS
BV4Node* BV4BuildParams::allocateNode()
{
	if(!mTop || mTop->mNbUsedNodes==NB_NODES_PER_SLAB)
	{
		Slab* newSlab = PX_NEW(Slab);
		newSlab->mNbUsedNodes = 0;
		newSlab->mNext = mTop;
		mTop = newSlab;
	}
	return &mTop->mNodes[mTop->mNbUsedNodes++];
}

void BV4BuildParams::releaseNodes()
{
	Slab* current = mTop;
	while(current)
	{
		Slab* next = current->mNext;
		PX_DELETE(current);
		current = next;
	}
	mTop = NULL;
}
#endif

static PX_FORCE_INLINE void setupBounds(BV4Node* node4, PxU32 i, const AABBTreeNode* node, float epsilon)
{
	node4->mBVData[i].mAABB = node->getAABB();
	if(epsilon!=0.0f)
		node4->mBVData[i].mAABB.mExtents += PxVec3(epsilon);
}

static void setPrimitive(const BV4_AABBTree& source, BV4Node* node4, PxU32 i, const AABBTreeNode* node, float epsilon)
{
	const PxU32 nbPrims = node->getNbPrimitives();
	PX_ASSERT(nbPrims<16);
	const PxU32* indexBase = source.getIndices();
	const PxU32* prims = node->getPrimitives();
	const PxU64 offset = PxU64(prims - indexBase);
	for(PxU32 j=0;j<nbPrims;j++)
	{
		PX_ASSERT(prims[j] == offset+j);
	}
	const PxU64 primitiveIndex = (offset<<4)|(nbPrims&15);

	setupBounds(node4, i, node, epsilon);
	node4->mBVData[i].mData64 = (primitiveIndex<<1)|1;
}

#ifdef GU_BV4_FILL_GAPS
static bool splitPrimitives(const BV4BuildParams& params, BV4Node* node4, PxU32 i, const AABBTreeNode* node)
{
	if(!params.mMesh)
		return false;

	const PxU32 nbPrims = node->getNbPrimitives();
	PX_ASSERT(nbPrims<16);
	if(nbPrims<2)
		return false;

	const PxU32* indexBase = params.mSource.getIndices();
	const PxU32* prims = node->getPrimitives();
	PxU64 offset = PxU64(prims - indexBase);

	// In theory we should reshuffle the list but it would mean updating the remap table again.
	// A way to avoid that would be to virtually split & sort the triangles directly in the BV2
	// source tree, before the initial remap table is created.

	//const PxU32 splitPrims = NbPrims/2;	// ### actually should be the number in the last split in bv2
	const PxU32 splitPrims = node->mNextSplit;
	PxU32 j=0;
	for(PxU32 ii=0;ii<2;ii++)
	{
		const PxU32 currentNb = ii==0 ? splitPrims : (nbPrims-splitPrims);

		PxBounds3 bounds = PxBounds3::empty();
		for(PxU32 k=0;k<currentNb;k++)
		{
			PX_ASSERT(prims[j] == offset+k);

			PxU32 vref0, vref1, vref2;
			getVertexReferences(vref0, vref1, vref2, prims[j], params.mMesh->getTris32(), params.mMesh->getTris16());

			// PT: TODO: SIMD
			const PxVec3* verts = params.mMesh->getVerts();
			bounds.include(verts[vref0]);
			bounds.include(verts[vref1]);
			bounds.include(verts[vref2]);

			j++;
		}
		PX_ASSERT(bounds.isInside(node->mBV));
		const PxU64 primitiveIndex = (offset<<4)|(currentNb&15);

//		SetupBounds(node4, i, node, context.mEpsilon);
			node4->mBVData[i+ii].mAABB = bounds;
			if(params.mEpsilon!=0.0f)
				node4->mBVData[i+ii].mAABB.mExtents += PxVec3(params.mEpsilon);

		node4->mBVData[i+ii].mData64 = (primitiveIndex<<1)|1;

		offset += currentNb;
	}
	return true;
}
#endif

static BV4Node* setNode(const BV4_AABBTree& source, BV4Node* node4, PxU32 i, const AABBTreeNode* node, BV4BuildParams& params)
{
	BV4Node* child = NULL;
	if(node->isLeaf())
	{
		setPrimitive(source, node4, i, node, params.mEpsilon);
	}
	else
	{
		setupBounds(node4, i, node, params.mEpsilon);

		params.mNbNodes++;
#ifdef GU_BV4_USE_NODE_POOLS
		child = params.allocateNode();
#else
		child = PX_NEW(BV4Node);
#endif
		node4->mBVData[i].mData64 = size_t(child);
	}
	return child;
}

#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
static inline PxBounds3 getMinMaxBox(const CenterExtents& box)
{
	if(box.mExtents.x>=0.0f && box.mExtents.y>=0.0f && box.mExtents.z>=0.0f)
//	if(!box.isEmpty())	// ### asserts!
		return PxBounds3::centerExtents(box.mCenter, box.mExtents);
	else
		return PxBounds3::empty();
}

static void FigureOutPNS(BV4Node* node)
{
	//   ____A____
	//   P       N
	// __|__   __|__
	// PP PN   NP NN

	const CenterExtents& box0 = node->mBVData[0].mAABB;
	const CenterExtents& box1 = node->mBVData[1].mAABB;
	const CenterExtents& box2 = node->mBVData[2].mAABB;
	const CenterExtents& box3 = node->mBVData[3].mAABB;

	// PT: TODO: SIMD, optimize
	const PxBounds3 boxPP = getMinMaxBox(box0);
	const PxBounds3 boxPN = getMinMaxBox(box1);
	const PxBounds3 boxNP = getMinMaxBox(box2);
	const PxBounds3 boxNN = getMinMaxBox(box3);

	PxBounds3 boxP = boxPP;	boxP.include(boxPN);
	PxBounds3 boxN = boxNP;	boxN.include(boxNN);

	node->mBVData[0].mTempPNS = precomputeNodeSorting(boxP, boxN);
	node->mBVData[1].mTempPNS = precomputeNodeSorting(boxPP, boxPN);
	node->mBVData[2].mTempPNS = precomputeNodeSorting(boxNP, boxNN);
}
#endif

/*static bool hasTwoLeafChildren(const AABBTreeNode* current_node)
{
	if(current_node->isLeaf())
		return false;
	const AABBTreeNode* P = current_node->getPos();
	const AABBTreeNode* N = current_node->getNeg();
	return P->isLeaf() && N->isLeaf();
}*/

static void buildBV4(const BV4_AABBTree& source, BV4Node* tmp, const AABBTreeNode* current_node, BV4BuildParams& params)
{
	PX_ASSERT(!current_node->isLeaf());

	// In the regular tree we have current node A, and:
	//   ____A____
	//   P       N
	// __|__   __|__
	// PP PN   NP NN
	//
	// For PNS we have:
	// bit0 to sort P|N
	// bit1 to sort PP|PN
	// bit2 to sort NP|NN
	//
	// As much as possible we need to preserve the original order in BV4, if we want to reuse the same PNS bits.
	//
	// bit0|bit1|bit2   Order			8bits code
	// 0    0    0      PP PN NP NN		0 1 2 3
	// 0    0    1      PP PN NN NP		0 1 3 2
	// 0    1    0      PN PP NP NN		1 0 2 3
	// 0    1    1      PN PP NN NP		1 0 3 2
	// 1    0    0      NP NN PP PN		2 3 0 1
	// 1    0    1      NN NP PP PN		3 2	0 1
	// 1    1    0      NP NN PN PP		2 3	1 0
	// 1    1    1      NN NP PN PP		3 2	1 0
	//
	// So we can fetch/compute the sequence from the bits, combine it with limitations from the node type, and process the nodes in order. In theory.
	// 8*8bits => the whole thing fits in a single 64bit register, so we could potentially use a "register LUT" here.

	const AABBTreeNode* P = current_node->getPos();
	const AABBTreeNode* N = current_node->getNeg();

	const bool PLeaf = P->isLeaf();
	const bool NLeaf = N->isLeaf();

	if(PLeaf)
	{
		if(NLeaf)
		{
			// Case 1: P and N are both leaves:
			//   ____A____
			//   P       N
			// => store as (P,N) and keep bit0
#ifndef GU_BV4_FILL_GAPS
			params.mStats[0]++;
			// PN leaves => store 2 triangle pointers, lose 50% of node space
			setPrimitive(source, tmp, 0, P, params.mEpsilon);
			setPrimitive(source, tmp, 1, N, params.mEpsilon);
#else
			{
				PxU32 nextIndex = 2;
				if(!splitPrimitives(params, tmp, 0, P))
				{
					setPrimitive(source, tmp, 0, P, params.mEpsilon);
					nextIndex = 1;
				}

				if(!splitPrimitives(params, tmp, nextIndex, N))
					setPrimitive(source, tmp, nextIndex, N, params.mEpsilon);
			
				const PxU32 statIndex = tmp->getType();
				if(statIndex==4)
					params.mStats[3]++;
				else if(statIndex==3)
					params.mStats[1]++;
				else if(statIndex==2)
					params.mStats[0]++;
				else
					PX_ASSERT(0);
			}
#endif

#ifdef GU_BV4_PRECOMPUTED_NODE_SORT0
			tmp->mBVData[0].mTempPNS = precomputeNodeSorting(P->mBV, N->mBV);
#endif
#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
			FigureOutPNS(tmp);
#endif
		}
		else
		{
			// Case 2: P leaf, N no leaf
			//   ____A____
			//   P       N
			//         __|__
			//         NP NN
			// => store as (P,NP,NN), keep bit0 and bit2
//#define NODE_FUSION
#ifdef NODE_FUSION
			// P leaf => store 1 triangle pointers and 2 node pointers
			// => 3 slots used, 25% wasted

			const AABBTreeNode* NP = N->getPos();
			const AABBTreeNode* NN = N->getNeg();

			BV4Node* ChildNP;
			BV4Node* ChildNN;

	#ifdef GU_BV4_FILL_GAPS
			if(splitPrimitives(params, tmp, 0, P))
			{
				// We used up the empty slot, continue as usual in slot 2
				ChildNP = setNode(source, tmp, 2, NP, params);
				ChildNN = setNode(source, tmp, 3, NN, params);
			}
			else
	#endif
			{
				// We oouldn't split the prims, continue searching for a way to use the empty slot
				setPrimitive(source, tmp, 0, P, params.mEpsilon);

				PxU32 c=0;
				if(hasTwoLeafChildren(NP))
				{
					// Drag the terminal leaves directly into this BV4 node, drop internal node NP
					setPrimitive(source, tmp, 1, NP->getPos(), params.mEpsilon);
					setPrimitive(source, tmp, 2, NP->getNeg(), params.mEpsilon);
					ChildNP = NULL;
					c=1;
				}
				else
				{
					ChildNP = setNode(source, tmp, 1, NP, params);
				}

				if(c==0 && hasTwoLeafChildren(NN))
				{
					// Drag the terminal leaves directly into this BV4 node, drop internal node NN
					setPrimitive(source, tmp, 2, NN->getPos(), params.mEpsilon);
					setPrimitive(source, tmp, 3, NN->getNeg(), params.mEpsilon);
					ChildNN = NULL;
				}
				else
				{
	#ifdef GU_BV4_FILL_GAPS
					if(c==0 && NN->isLeaf())
					{
						ChildNN = NULL;
						if(!splitPrimitives(params, tmp, 2, NN))
							setPrimitive(source, tmp, 2, NN, params.mEpsilon);
					}
					else
	#endif
					{
						ChildNN = setNode(source, tmp, 2+c, NN, params);
					}
				}
			}

			const PxU32 statIndex = tmp->getType();
			if(statIndex==4)
				params.mStats[3]++;
			else if(statIndex==3)
				params.mStats[1]++;
			else if(statIndex==2)
				params.mStats[0]++;
			else
				PX_ASSERT(0);
#else
			params.mStats[1]++;
			// P leaf => store 1 triangle pointers and 2 node pointers
			// => 3 slots used, 25% wasted
			setPrimitive(source, tmp, 0, P, params.mEpsilon);

			const AABBTreeNode* NP = N->getPos();
			const AABBTreeNode* NN = N->getNeg();

			BV4Node* ChildNP = setNode(source, tmp, 1, NP, params);
			BV4Node* ChildNN = setNode(source, tmp, 2, NN, params);
#endif

#ifdef GU_BV4_PRECOMPUTED_NODE_SORT0
			tmp->mBVData[0].mTempPNS = precomputeNodeSorting(P->mBV, N->mBV);
			tmp->mBVData[2].mTempPNS = precomputeNodeSorting(NP->mBV, NN->mBV);
#endif
#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
			FigureOutPNS(tmp);
#endif

			if(ChildNP)
				buildBV4(source, ChildNP, NP, params);
			if(ChildNN)
				buildBV4(source, ChildNN, NN, params);
		}
	}
	else
	{
		if(NLeaf)
		{
			// Note: this case doesn't exist anymore because of the node reorganizing for shadow rays

			// Case 3: P no leaf, N leaf
			//   ____A____
			//   P       N
			// __|__
			// PP PN
			// => store as (PP,PN,N), keep bit0 and bit1
			params.mStats[2]++;

			// N leaf => store 1 triangle pointers and 2 node pointers
			// => 3 slots used, 25% wasted
			setPrimitive(source, tmp, 2, N, params.mEpsilon);

			//

			const AABBTreeNode* PP = P->getPos();
			const AABBTreeNode* PN = P->getNeg();

			BV4Node* ChildPP = setNode(source, tmp, 0, PP, params);
			BV4Node* ChildPN = setNode(source, tmp, 1, PN, params);

#ifdef GU_BV4_PRECOMPUTED_NODE_SORT0
			tmp->mBVData[0].mTempPNS = precomputeNodeSorting(P->mBV, N->mBV);
			tmp->mBVData[1].mTempPNS = precomputeNodeSorting(PP->mBV, PN->mBV);
#endif
#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
			FigureOutPNS(tmp);
#endif

			if(ChildPP)
				buildBV4(source, ChildPP, PP, params);
			if(ChildPN)
				buildBV4(source, ChildPN, PN, params);
		}
		else
		{
			// Case 4: P and N are no leaves:
			// => store as (PP,PN,NP,NN), keep bit0/bit1/bit2
			params.mStats[3]++;

			// No leaves => store 4 node pointers
			const AABBTreeNode* PP = P->getPos();
			const AABBTreeNode* PN = P->getNeg();
			const AABBTreeNode* NP = N->getPos();
			const AABBTreeNode* NN = N->getNeg();

			BV4Node* ChildPP = setNode(source, tmp, 0, PP, params);
			BV4Node* ChildPN = setNode(source, tmp, 1, PN, params);
			BV4Node* ChildNP = setNode(source, tmp, 2, NP, params);
			BV4Node* ChildNN = setNode(source, tmp, 3, NN, params);

#ifdef GU_BV4_PRECOMPUTED_NODE_SORT0
			tmp->mBVData[0].mTempPNS = precomputeNodeSorting(P->mBV, N->mBV);
			tmp->mBVData[1].mTempPNS = precomputeNodeSorting(PP->mBV, PN->mBV);
			tmp->mBVData[2].mTempPNS = precomputeNodeSorting(NP->mBV, NN->mBV);
#endif
#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
			FigureOutPNS(tmp);
#endif
			if(ChildPP)
				buildBV4(source, ChildPP, PP, params);
			if(ChildPN)
				buildBV4(source, ChildPN, PN, params);
			if(ChildNP)
				buildBV4(source, ChildNP, NP, params);
			if(ChildNN)
				buildBV4(source, ChildNN, NN, params);
		}
	}
}

#ifdef GU_BV4_USE_SLABS
static void computeMaxValues(const BV4Node* current, PxVec3& MinMax, PxVec3& MaxMax)
#else
static void computeMaxValues(const BV4Node* current, PxVec3& CMax, PxVec3& EMax)
#endif
{
	for(PxU32 i=0; i<4; i++)
	{
		if(current->mBVData[i].mData64 != PX_INVALID_U64)
		{
			const CenterExtents& Box = current->mBVData[i].mAABB;
#ifdef GU_BV4_USE_SLABS
			const PxVec3 Min = Box.mCenter - Box.mExtents;
			const PxVec3 Max = Box.mCenter + Box.mExtents;
			if(fabsf(Min.x)>MinMax.x)	MinMax.x = fabsf(Min.x);
			if(fabsf(Min.y)>MinMax.y)	MinMax.y = fabsf(Min.y);
			if(fabsf(Min.z)>MinMax.z)	MinMax.z = fabsf(Min.z);
			if(fabsf(Max.x)>MaxMax.x)	MaxMax.x = fabsf(Max.x);
			if(fabsf(Max.y)>MaxMax.y)	MaxMax.y = fabsf(Max.y);
			if(fabsf(Max.z)>MaxMax.z)	MaxMax.z = fabsf(Max.z);
#else
			if(fabsf(Box.mCenter.x)>CMax.x)		CMax.x = fabsf(Box.mCenter.x);
			if(fabsf(Box.mCenter.y)>CMax.y)		CMax.y = fabsf(Box.mCenter.y);
			if(fabsf(Box.mCenter.z)>CMax.z)		CMax.z = fabsf(Box.mCenter.z);
			if(fabsf(Box.mExtents.x)>EMax.x)	EMax.x = fabsf(Box.mExtents.x);
			if(fabsf(Box.mExtents.y)>EMax.y)	EMax.y = fabsf(Box.mExtents.y);
			if(fabsf(Box.mExtents.z)>EMax.z)	EMax.z = fabsf(Box.mExtents.z);
#endif
			if(!current->isLeaf(i))
			{
				const BV4Node* ChildNode = current->getChild(i);
#ifdef GU_BV4_USE_SLABS
				computeMaxValues(ChildNode, MinMax, MaxMax);
#else
				computeMaxValues(ChildNode, CMax, EMax);
#endif
			}
		}
	}
}

template<class T>
static PX_FORCE_INLINE bool copyData(T* PX_RESTRICT dst, const BV4Node* PX_RESTRICT src, PxU32 i)
{
	if(src->isLeaf(i))
	{
		if(src->mBVData[i].mData64 > 0xffffffff)
			return false;
	}

	dst->mData = PxU32(src->mBVData[i].mData64);

	//dst->mData = PxTo32(src->mBVData[i].mData);
//	dst->mData = PxU32(src->mBVData[i].mData);
//	dst->encodePNS(src->mBVData[i].mTempPNS);

	return true;
}

namespace
{
	struct flattenQParams
	{
		PxVec3	mCQuantCoeff;
		PxVec3	mEQuantCoeff;
		PxVec3	mCenterCoeff;
		PxVec3	mExtentsCoeff;
	};
}

template<class T>
static PX_FORCE_INLINE bool processNode(T* PX_RESTRICT data, const BV4Node* PX_RESTRICT current, PxU64* PX_RESTRICT nextIDs, const BV4Node** PX_RESTRICT childNodes, PxU32 i, PxU64& current_id, PxU32& nbToGo)
{
	const BV4Node* childNode = current->getChild(i);

	const PxU64 nextID = current_id;
#ifdef GU_BV4_USE_SLABS
	current_id += 4;
#else
	const PxU32 childSize = childNode->getType();
	current_id += childSize;
#endif
	const PxU32 childType = (childNode->getType() - 2) << 1;
	const PxU64 data64 = size_t(childType + (nextID << GU_BV4_CHILD_OFFSET_SHIFT_COUNT));
	if(data64 <= 0xffffffff)
		data[i].mData = PxU32(data64);
	else
		return false;

	//PX_ASSERT(data[i].mData == size_t(childType+(nextID<<3)));

	nextIDs[nbToGo] = nextID;
	childNodes[nbToGo] = childNode;
	nbToGo++;

#ifdef GU_BV4_PRECOMPUTED_NODE_SORT
	data[i].encodePNS(current->mBVData[i].mTempPNS);
#endif
	return true;
}

static bool flattenQ(const flattenQParams& params, BVDataPackedQ* const dest, const PxU64 box_id, PxU64& current_id, const BV4Node* current, PxU32& max_depth, PxU32& current_depth)
{
	// Entering a new node => increase depth
	current_depth++;
	// Keep track of max depth
	if(current_depth>max_depth)
		max_depth = current_depth;

//	dest[box_id] = *current;
	const PxU32 CurrentType = current->getType();
	for(PxU32 i=0; i<CurrentType; i++)
	{
		const CenterExtents& Box = current->mBVData[i].mAABB;
#ifdef GU_BV4_USE_SLABS
		const PxVec3 m = Box.mCenter - Box.mExtents;
		const PxVec3 M = Box.mCenter + Box.mExtents;

		dest[box_id + i].mAABB.mData[0].mCenter = PxI16(m.x * params.mCQuantCoeff.x);
		dest[box_id + i].mAABB.mData[1].mCenter = PxI16(m.y * params.mCQuantCoeff.y);
		dest[box_id + i].mAABB.mData[2].mCenter = PxI16(m.z * params.mCQuantCoeff.z);
		dest[box_id + i].mAABB.mData[0].mExtents = PxU16(PxI16(M.x * params.mEQuantCoeff.x));
		dest[box_id + i].mAABB.mData[1].mExtents = PxU16(PxI16(M.y * params.mEQuantCoeff.y));
		dest[box_id + i].mAABB.mData[2].mExtents = PxU16(PxI16(M.z * params.mEQuantCoeff.z));

		if (1)
		{
			for (PxU32 j = 0; j<3; j++)
			{
				// Dequantize the min/max
				//								const float qmin = float(dest[box_id+i].mAABB.mData[j].mCenter) * mCenterCoeff[j];
				//								const float qmax = float(PxI16(dest[box_id+i].mAABB.mData[j].mExtents)) * mExtentsCoeff[j];
				// Compare real & dequantized values
				/*								if(qmax<M[j] || qmin>m[j])
				{
				int stop=1;
				}*/
				bool CanLeave;
				do
				{
					CanLeave = true;
					const float qmin = float(dest[box_id + i].mAABB.mData[j].mCenter) * params.mCenterCoeff[j];
					const float qmax = float(PxI16(dest[box_id + i].mAABB.mData[j].mExtents)) * params.mExtentsCoeff[j];

					if (qmax<M[j])
					{
						//										if(dest[box_id+i].mAABB.mData[j].mExtents!=0xffff)
						if (dest[box_id + i].mAABB.mData[j].mExtents != 0x7fff)
						{
							dest[box_id + i].mAABB.mData[j].mExtents++;
							CanLeave = false;
						}
					}
					if (qmin>m[j])
					{
						if (dest[box_id + i].mAABB.mData[j].mCenter)
						{
							dest[box_id + i].mAABB.mData[j].mCenter--;
							CanLeave = false;
						}
					}
				} while (!CanLeave);
			}
		}
#else	// GU_BV4_USE_SLABS
		dest[box_id + i].mAABB.mData[0].mCenter = PxI16(Box.mCenter.x * CQuantCoeff.x);
		dest[box_id + i].mAABB.mData[1].mCenter = PxI16(Box.mCenter.y * CQuantCoeff.y);
		dest[box_id + i].mAABB.mData[2].mCenter = PxI16(Box.mCenter.z * CQuantCoeff.z);
		dest[box_id + i].mAABB.mData[0].mExtents = PxU16(Box.mExtents.x * EQuantCoeff.x);
		dest[box_id + i].mAABB.mData[1].mExtents = PxU16(Box.mExtents.y * EQuantCoeff.y);
		dest[box_id + i].mAABB.mData[2].mExtents = PxU16(Box.mExtents.z * EQuantCoeff.z);

		// Fix quantized boxes
		if (1)
		{
			// Make sure the quantized box is still valid
			const PxVec3 Max = Box.mCenter + Box.mExtents;
			const PxVec3 Min = Box.mCenter - Box.mExtents;
			// For each axis
			for (PxU32 j = 0; j<3; j++)
			{	// Dequantize the box center
				const float qc = float(dest[box_id + i].mAABB.mData[j].mCenter) * mCenterCoeff[j];
				bool FixMe = true;
				do
				{	// Dequantize the box extent
					const float qe = float(dest[box_id + i].mAABB.mData[j].mExtents) * mExtentsCoeff[j];
					// Compare real & dequantized values
					if (qc + qe<Max[j] || qc - qe>Min[j])	dest[box_id + i].mAABB.mData[j].mExtents++;
					else								FixMe = false;
					// Prevent wrapping
					if (!dest[box_id + i].mAABB.mData[j].mExtents)
					{
						dest[box_id + i].mAABB.mData[j].mExtents = 0xffff;
						FixMe = false;
					}
				} while (FixMe);
			}
		}
#endif	// GU_BV4_USE_SLABS

		if(!copyData(&dest[box_id + i], current, i))
			return false;
	}

	PxU32 NbToGo = 0;
	PxU64 NextIDs[4] = { PX_INVALID_U64, PX_INVALID_U64, PX_INVALID_U64, PX_INVALID_U64 };
	const BV4Node* ChildNodes[4] = { NULL, NULL, NULL, NULL };

	BVDataPackedQ* data = dest + box_id;
	for(PxU32 i=0; i<4; i++)
	{
		if(current->mBVData[i].mData64 != PX_INVALID_U64 && !current->isLeaf(i))
		{
			if(!processNode(data, current, NextIDs, ChildNodes, i, current_id, NbToGo))
				return false;

//#define DEPTH_FIRST
#ifdef DEPTH_FIRST
			if(!flattenQ(params, dest, NextID, current_id, ChildNode, max_depth, current_depth, CQuantCoeff, EQuantCoeff, mCenterCoeff, mExtentsCoeff))
				return false;
			current_depth--;
#endif
		}
#ifdef GU_BV4_USE_SLABS
		if (current->mBVData[i].mData64 == PX_INVALID_U64)
		{
			data[i].mAABB.mData[0].mExtents = 0;
			data[i].mAABB.mData[1].mExtents = 0;
			data[i].mAABB.mData[2].mExtents = 0;
			data[i].mAABB.mData[0].mCenter = 0;
			data[i].mAABB.mData[1].mCenter = 0;
			data[i].mAABB.mData[2].mCenter = 0;
			data[i].mData = PX_INVALID_U32;
		}
#endif
	}

#ifndef DEPTH_FIRST
	for(PxU32 i=0; i<NbToGo; i++)
	{
		if(!flattenQ(params, dest, NextIDs[i], current_id, ChildNodes[i], max_depth, current_depth))
			return false;
		current_depth--;
	}
#endif
#ifndef GU_BV4_USE_NODE_POOLS
	PX_DELETE(current);
#endif
	return true;
}

static bool flattenNQ(BVDataPackedNQ* const dest, const PxU64 box_id, PxU64& current_id, const BV4Node* current, PxU32& max_depth, PxU32& current_depth)
{
	// Entering a new node => increase depth
	current_depth++;
	// Keep track of max depth
	if(current_depth>max_depth)
		max_depth = current_depth;

//	dest[box_id] = *current;
	const PxU32 CurrentType = current->getType();
	for(PxU32 i=0; i<CurrentType; i++)
	{
#ifdef GU_BV4_USE_SLABS
		// Compute min & max right here. Store temp as Center/Extents = Min/Max
		const CenterExtents& Box = current->mBVData[i].mAABB;
		dest[box_id + i].mAABB.mCenter = Box.mCenter - Box.mExtents;
		dest[box_id + i].mAABB.mExtents = Box.mCenter + Box.mExtents;
#else	// GU_BV4_USE_SLABS
		dest[box_id + i].mAABB = current->mBVData[i].mAABB;
#endif	// GU_BV4_USE_SLABS

		if(!copyData(&dest[box_id + i], current, i))
			return false;
	}

	PxU32 NbToGo = 0;
	PxU64 NextIDs[4] = { PX_INVALID_U64, PX_INVALID_U64, PX_INVALID_U64, PX_INVALID_U64 };
	const BV4Node* ChildNodes[4] = { NULL, NULL, NULL, NULL };

	BVDataPackedNQ* data = dest + box_id;
	for(PxU32 i=0; i<4; i++)
	{
		if(current->mBVData[i].mData64 != PX_INVALID_U64 && !current->isLeaf(i))
		{
			if(!processNode(data, current, NextIDs, ChildNodes, i, current_id, NbToGo))
				return false;

//#define DEPTH_FIRST
#ifdef DEPTH_FIRST
			if(!flattenNQ(dest, NextID, current_id, ChildNode, max_depth, current_depth))
				return false;
			current_depth--;
#endif
		}
#ifdef GU_BV4_USE_SLABS
		if (current->mBVData[i].mData64 == PX_INVALID_U64)
		{
			data[i].mAABB.mCenter = PxVec3(0.0f);
			data[i].mAABB.mExtents = PxVec3(0.0f);
			data[i].mData = PX_INVALID_U32;
		}
#endif
	}

#ifndef DEPTH_FIRST
	for(PxU32 i=0; i<NbToGo; i++)
	{
		if(!flattenNQ(dest, NextIDs[i], current_id, ChildNodes[i], max_depth, current_depth))
			return false;
		current_depth--;
	}
#endif
#ifndef GU_BV4_USE_NODE_POOLS
	PX_DELETE(current);
#endif
	return true;
}

static bool BuildBV4FromRoot(BV4Tree& tree, BV4Node* Root, BV4BuildParams& Params, bool quantized, float epsilon)
{
	GU_PROFILE_ZONE("....BuildBV4FromRoot")

	BV4Tree* T = &tree;

	T->mQuantized = quantized;

	// Version with variable-sized nodes in single stream
	{
		const PxU32 NbSingleNodes = Params.mStats[0] * 2 + (Params.mStats[1] + Params.mStats[2]) * 3 + Params.mStats[3] * 4;

		PxU64 CurID = Root->getType();
		PxU32 InitData = PX_INVALID_U32;
#ifdef GU_BV4_USE_SLABS
		PX_UNUSED(NbSingleNodes);
		const PxU32 NbNeeded = (Params.mStats[0] + Params.mStats[1] + Params.mStats[2] + Params.mStats[3]) * 4;

		// PT: TODO: refactor with code in BV4Tree::load
//		BVDataPacked* Nodes = reinterpret_cast<BVDataPacked*>(PX_ALLOC(sizeof(BVDataPacked)*NbNeeded, "BV4 nodes"));	// PT: PX_NEW breaks alignment here
//		BVDataPacked* Nodes = PX_NEW(BVDataPacked)[NbNeeded];
		void* nodes;
		{
			const PxU32 nodeSize = T->mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
			const PxU32 dataSize = nodeSize*NbNeeded;
			nodes = PX_ALLOC(dataSize, "BV4 nodes");	// PT: PX_NEW breaks alignment here
		}

		if (CurID == 2)
		{
			InitData = 0;
		}
		else if (CurID == 3)
		{
			InitData = 2;
		}
		else if (CurID == 4)
		{
			InitData = 4;
		}

		CurID = 4;
//		PxU32 CurID = 4;
//		PxU32 InitData = 4;
#else
		BVDataPacked* Nodes = PX_NEW(BVDataPacked)[NbSingleNodes];

		if (CurID == 2)
		{
			InitData = 0;
		}
		else if (CurID == 3)
		{
			InitData = 2;
		}
		else if (CurID == 4)
		{
			InitData = 4;
		}
#endif

		T->mInitData = InitData;
		PxU32 MaxDepth = 0;
		PxU32 CurrentDepth = 0;

		bool buildStatus;

		if(T->mQuantized)
		{
			flattenQParams params;
#ifdef GU_BV4_USE_SLABS
			PxVec3 MinQuantCoeff, MaxQuantCoeff;

			// Get max values
			PxVec3 MinMax(-FLT_MAX);
			PxVec3 MaxMax(-FLT_MAX);
			computeMaxValues(Root, MinMax, MaxMax);

			const PxU32 nbm = 15;

			// Compute quantization coeffs
			const float MinCoeff = float((1 << nbm) - 1);
			const float MaxCoeff = float((1 << nbm) - 1);
			MinQuantCoeff.x = MinMax.x != 0.0f ? MinCoeff / MinMax.x : 0.0f;
			MinQuantCoeff.y = MinMax.y != 0.0f ? MinCoeff / MinMax.y : 0.0f;
			MinQuantCoeff.z = MinMax.z != 0.0f ? MinCoeff / MinMax.z : 0.0f;
			MaxQuantCoeff.x = MaxMax.x != 0.0f ? MaxCoeff / MaxMax.x : 0.0f;
			MaxQuantCoeff.y = MaxMax.y != 0.0f ? MaxCoeff / MaxMax.y : 0.0f;
			MaxQuantCoeff.z = MaxMax.z != 0.0f ? MaxCoeff / MaxMax.z : 0.0f;
			// Compute and save dequantization coeffs
			T->mCenterOrMinCoeff.x = MinMax.x / MinCoeff;
			T->mCenterOrMinCoeff.y = MinMax.y / MinCoeff;
			T->mCenterOrMinCoeff.z = MinMax.z / MinCoeff;
			T->mExtentsOrMaxCoeff.x = MaxMax.x / MaxCoeff;
			T->mExtentsOrMaxCoeff.y = MaxMax.y / MaxCoeff;
			T->mExtentsOrMaxCoeff.z = MaxMax.z / MaxCoeff;

			params.mCQuantCoeff = MinQuantCoeff;
			params.mEQuantCoeff = MaxQuantCoeff;
#else
			// Get max values
			PxVec3 CMax(-FLT_MAX);
			PxVec3 EMax(-FLT_MAX);
			computeMaxValues(Root, CMax, EMax);

			const PxU32 nbc = 15;
			const PxU32 nbe = 16;
//			const PxU32 nbc=7;
//			const PxU32 nbe=8;

			const float UnitQuantError = 2.0f / 65535.0f;
			EMax.x += CMax.x*UnitQuantError;
			EMax.y += CMax.y*UnitQuantError;
			EMax.z += CMax.z*UnitQuantError;

			// Compute quantization coeffs
			const float CCoeff = float((1 << nbc) - 1);
			CQuantCoeff.x = CMax.x != 0.0f ? CCoeff / CMax.x : 0.0f;
			CQuantCoeff.y = CMax.y != 0.0f ? CCoeff / CMax.y : 0.0f;
			CQuantCoeff.z = CMax.z != 0.0f ? CCoeff / CMax.z : 0.0f;
			const float ECoeff = float((1 << nbe) - 32);
			EQuantCoeff.x = EMax.x != 0.0f ? ECoeff / EMax.x : 0.0f;
			EQuantCoeff.y = EMax.y != 0.0f ? ECoeff / EMax.y : 0.0f;
			EQuantCoeff.z = EMax.z != 0.0f ? ECoeff / EMax.z : 0.0f;
			// Compute and save dequantization coeffs
			T->mCenterOrMinCoeff.x = CMax.x / CCoeff;
			T->mCenterOrMinCoeff.y = CMax.y / CCoeff;
			T->mCenterOrMinCoeff.z = CMax.z / CCoeff;
			T->mExtentsOrMaxCoeff.x = EMax.x / ECoeff;
			T->mExtentsOrMaxCoeff.y = EMax.y / ECoeff;
			T->mExtentsOrMaxCoeff.z = EMax.z / ECoeff;

			params.mCQuantCoeff = CQuantCoeff;
			params.mEQuantCoeff = EQuantCoeff;
#endif
			params.mCenterCoeff = T->mCenterOrMinCoeff;
			params.mExtentsCoeff = T->mExtentsOrMaxCoeff;

			buildStatus = flattenQ(params, reinterpret_cast<BVDataPackedQ*>(nodes), 0, CurID, Root, MaxDepth, CurrentDepth);
		}
		else
		{
			buildStatus = flattenNQ(reinterpret_cast<BVDataPackedNQ*>(nodes), 0, CurID, Root, MaxDepth, CurrentDepth);
		}

#ifdef GU_BV4_USE_NODE_POOLS
		Params.releaseNodes();
#endif
		if(!buildStatus)
		{
			T->mNodes = nodes;
			return false;
		}

#ifdef GU_BV4_USE_SLABS
		// PT: TODO: revisit this, don't duplicate everything
		if(T->mQuantized)
		{
			BVDataPackedQ* _nodes = reinterpret_cast<BVDataPackedQ*>(nodes);

			PX_COMPILE_TIME_ASSERT(sizeof(BVDataSwizzledQ) == sizeof(BVDataPackedQ) * 4);
			BVDataPackedQ* Copy = PX_ALLOCATE(BVDataPackedQ, NbNeeded, "BVDataPackedQ");
			PxMemCopy(Copy, nodes, sizeof(BVDataPackedQ)*NbNeeded);
			for (PxU32 i = 0; i<NbNeeded / 4; i++)
			{
				const BVDataPackedQ* Src = Copy + i * 4;
				BVDataSwizzledQ* Dst = reinterpret_cast<BVDataSwizzledQ*>(_nodes + i * 4);
				for (PxU32 j = 0; j<4; j++)
				{
					// We previously stored m/M within c/e so we just need to swizzle now
					const QuantizedAABB& Box = Src[j].mAABB;
					Dst->mX[j].mMin = Box.mData[0].mCenter;
					Dst->mY[j].mMin = Box.mData[1].mCenter;
					Dst->mZ[j].mMin = Box.mData[2].mCenter;
					Dst->mX[j].mMax = PxI16(Box.mData[0].mExtents);
					Dst->mY[j].mMax = PxI16(Box.mData[1].mExtents);
					Dst->mZ[j].mMax = PxI16(Box.mData[2].mExtents);
					Dst->mData[j] = Src[j].mData;
				}
			}
			PX_FREE(Copy);
		}
		else
		{
			BVDataPackedNQ* _nodes = reinterpret_cast<BVDataPackedNQ*>(nodes);
			PX_COMPILE_TIME_ASSERT(sizeof(BVDataSwizzledNQ) == sizeof(BVDataPackedNQ) * 4);
			BVDataPackedNQ* Copy = PX_ALLOCATE(BVDataPackedNQ, NbNeeded, "BVDataPackedNQ");
			PxMemCopy(Copy, nodes, sizeof(BVDataPackedNQ)*NbNeeded);
			for (PxU32 i = 0; i<NbNeeded / 4; i++)
			{
				const BVDataPackedNQ* Src = Copy + i * 4;
				BVDataSwizzledNQ* Dst = reinterpret_cast<BVDataSwizzledNQ*>(_nodes + i * 4);
				for (PxU32 j = 0; j<4; j++)
				{
					// We previously stored m/M within c/e so we just need to swizzle now
					const CenterExtents& Box = Src[j].mAABB;
					Dst->mMinX[j] = Box.mCenter.x;
					Dst->mMinY[j] = Box.mCenter.y;
					Dst->mMinZ[j] = Box.mCenter.z;
					Dst->mMaxX[j] = Box.mExtents.x;
					Dst->mMaxY[j] = Box.mExtents.y;
					Dst->mMaxZ[j] = Box.mExtents.z;
					Dst->mData[j] = Src[j].mData;
				}
			}
			PX_FREE(Copy);

			if(0)
			{
				const PxVec3 eps(epsilon);
				float maxError = 0.0f;
				PxU32 nb = NbNeeded/4;
				BVDataSwizzledNQ* data = reinterpret_cast<BVDataSwizzledNQ*>(nodes);
				while(nb--)
				{
					BVDataSwizzledNQ* current = data + nb;

					for(PxU32 j=0;j<4;j++)
					{
						if(current->getChildData(j)==PX_INVALID_U32)
							continue;

						const PxBounds3 localBox(	PxVec3(current->mMinX[j], current->mMinY[j], current->mMinZ[j]),
													PxVec3(current->mMaxX[j], current->mMaxY[j], current->mMaxZ[j]));
						PxBounds3 refitBox;
						refitBox.setEmpty();

						if(current->isLeaf(j))
						{
							PxU32 primIndex = current->getPrimitive(j);

							PxU32 nbToGo = getNbPrimitives(primIndex);
							
							do
							{
								PX_ASSERT(primIndex<T->mMeshInterface->getNbPrimitives());

								T->mMeshInterface->refit(primIndex, refitBox);

								primIndex++;
							}while(nbToGo--);
						}
						else
						{
							PxU32 childOffset = current->getChildOffset(j);
							PX_ASSERT(!(childOffset&3));
							childOffset>>=2;
							PX_ASSERT(childOffset>nb);
							const PxU32 childType = current->getChildType(j);

							const BVDataSwizzledNQ* next = data + childOffset;
							{
								if(childType>1)
								{
									const PxBounds3 childBox(	PxVec3(next->mMinX[3], next->mMinY[3], next->mMinZ[3]),
																PxVec3(next->mMaxX[3], next->mMaxY[3], next->mMaxZ[3]));
									refitBox.include(childBox);
								}

								if(childType>0)
								{
									const PxBounds3 childBox(	PxVec3(next->mMinX[2], next->mMinY[2], next->mMinZ[2]),
																PxVec3(next->mMaxX[2], next->mMaxY[2], next->mMaxZ[2]));
									refitBox.include(childBox);
								}

								{
									const PxBounds3 childBox(	PxVec3(next->mMinX[1], next->mMinY[1], next->mMinZ[1]),
																PxVec3(next->mMaxX[1], next->mMaxY[1], next->mMaxZ[1]));
									refitBox.include(childBox);
								}

								{
									const PxBounds3 childBox(	PxVec3(next->mMinX[0], next->mMinY[0], next->mMinZ[0]),
																PxVec3(next->mMaxX[0], next->mMaxY[0], next->mMaxZ[0]));
									refitBox.include(childBox);
								}
							}
						}
						refitBox.minimum -= eps;
						refitBox.maximum += eps;
						{
							float error = (refitBox.minimum - localBox.minimum).magnitude();
							if(error>maxError)
								maxError = error;
						}
						{
							float error = (refitBox.maximum - localBox.maximum).magnitude();
							if(error>maxError)
								maxError = error;
						}
					}
				}
				printf("maxError: %f\n", double(maxError));
			}
		}
		PX_ASSERT(CurID == NbNeeded);
		T->mNbNodes = NbNeeded;
#else
		PX_ASSERT(CurID == NbSingleNodes);
		T->mNbNodes = NbSingleNodes;
#endif
		T->mNodes = nodes;
	}
	return true;
}

static bool BuildBV4Internal(BV4Tree& tree, const BV4_AABBTree& source, SourceMeshBase* mesh, float epsilon, bool quantized)
{
	GU_PROFILE_ZONE("..BuildBV4Internal")

	if(mesh->getNbPrimitives()<=4)
		return tree.init(mesh, source.getBV());

	{
		GU_PROFILE_ZONE("....CheckMD")

		struct Local
		{
			static void _checkMD(const AABBTreeNode* current_node, PxU32& md, PxU32& cd)
			{
				cd++;
				md = PxMax(md, cd);

				if(current_node->getPos())	{ _checkMD(current_node->getPos(), md, cd);	cd--;	}
				if(current_node->getNeg())	{ _checkMD(current_node->getNeg(), md, cd);	cd--;	}
			}

			static void _check(AABBTreeNode* current_node)
			{
				if(current_node->isLeaf())
					return;

				AABBTreeNode* P = const_cast<AABBTreeNode*>(current_node->getPos());
				AABBTreeNode* N = const_cast<AABBTreeNode*>(current_node->getNeg());
				{
					PxU32 MDP = 0;	PxU32 CDP = 0;	_checkMD(P, MDP, CDP);
					PxU32 MDN = 0;	PxU32 CDN = 0;	_checkMD(N, MDN, CDN);

					if(MDP>MDN)
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
		Local::_check(const_cast<AABBTreeNode*>(source.getNodes()));
	}

	// PT: not sure what the equivalent would be for tet-meshes here
	SourceMesh* triMesh = NULL;
	if(mesh->getMeshType()==SourceMeshBase::TRI_MESH)
		triMesh = static_cast<SourceMesh*>(mesh);

	BV4BuildParams Params(source, triMesh, epsilon);
	Params.mNbNodes=1;	// Root node
	Params.mStats[0]=0;
	Params.mStats[1]=0;
	Params.mStats[2]=0;
	Params.mStats[3]=0;

#ifdef GU_BV4_USE_NODE_POOLS
	BV4Node* Root = Params.allocateNode();
#else
	BV4Node* Root = PX_NEW(BV4Node);
#endif
	{
		GU_PROFILE_ZONE("....buildBV4")
		buildBV4(source, Root, source.getNodes(), Params);
	}

	if(!tree.init(mesh, source.getBV()))
		return false;
	
	return BuildBV4FromRoot(tree, Root, Params, quantized, epsilon);
}

/////

#define	REORDER_STATS_SIZE		16
struct ReorderData
{
public:
	PxU32*					mOrder;
	PxU32					mNbPrimsPerLeaf;
	PxU32					mIndex;
	PxU32					mNbPrims;
	PxU32					mStats[REORDER_STATS_SIZE];
	const SourceMeshBase*	mMesh;
};

static bool gReorderCallback(const AABBTreeNode* current, PxU32 /*depth*/, void* userData)
{
	ReorderData* Data = reinterpret_cast<ReorderData*>(userData);
	if(current->isLeaf())
	{
		const PxU32 n = current->getNbPrimitives();
		PX_ASSERT(n<=Data->mNbPrimsPerLeaf);
		Data->mStats[n]++;
		PxU32* Prims = const_cast<PxU32*>(current->getPrimitives());

		for(PxU32 i=0;i<n;i++)
		{
			PX_ASSERT(Prims[i]<Data->mNbPrims);
			Data->mOrder[Data->mIndex] = Prims[i];
			PX_ASSERT(Data->mIndex<Data->mNbPrims);
			Prims[i] = Data->mIndex;
			Data->mIndex++;
		}
	}
	return true;
}

bool physx::Gu::BuildBV4Ex(BV4Tree& tree, SourceMeshBase& mesh, float epsilon, PxU32 nbPrimitivePerLeaf, bool quantized, BV4_BuildStrategy strategy)
{
	//either number of triangle or number of tetrahedron
	const PxU32 nbPrimitives = mesh.getNbPrimitives();

	BV4_AABBTree Source;
	{
		GU_PROFILE_ZONE("..BuildBV4Ex_buildFromMesh")
		if(!Source.buildFromMesh(mesh, nbPrimitivePerLeaf, strategy))
			return false;
	}

	{
		GU_PROFILE_ZONE("..BuildBV4Ex_remap")
		PxU32* orderArray = PX_ALLOCATE(PxU32, nbPrimitives, "BV4");
		ReorderData RD;
		RD.mMesh			= &mesh;
		RD.mOrder			= orderArray;
		RD.mNbPrimsPerLeaf	= nbPrimitivePerLeaf;
		RD.mIndex			= 0;
		RD.mNbPrims			= nbPrimitives;
		for(PxU32 i=0;i<REORDER_STATS_SIZE;i++)
			RD.mStats[i] = 0;
		Source.walk(gReorderCallback, &RD);
		PX_ASSERT(RD.mIndex== nbPrimitives);
		mesh.remapTopology(orderArray);
		PX_FREE(orderArray);
//		for(PxU32 i=0;i<16;i++)
//			printf("%d: %d\n", i, RD.mStats[i]);
	}

	if(mesh.getNbPrimitives() <= nbPrimitivePerLeaf)
		return tree.init(&mesh, Source.getBV());

	return BuildBV4Internal(tree, Source, &mesh, epsilon, quantized);
}

