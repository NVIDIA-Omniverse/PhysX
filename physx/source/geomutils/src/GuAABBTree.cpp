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

#include "GuAABBTreeBounds.h"
#include "GuAABBTree.h"
#include "GuAABBTreeBuildStats.h"
#include "GuBounds.h"
#include "GuAABBTreeNode.h"
#include "GuSAH.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxFPU.h"

using namespace physx;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

void AABBTreeBounds::init(PxU32 nbBounds, const PxBounds3* bounds)
{
	PX_FREE(mBounds);
	// PT: we always allocate one extra box, to make sure we can safely use V4 loads on the array
	mBounds = PX_ALLOCATE(PxBounds3, (nbBounds + 1), "AABBTreeBounds");
	if(bounds)
		PxMemCopy(mBounds, bounds, nbBounds*sizeof(PxBounds3));
}

void AABBTreeBounds::resize(PxU32 newSize, PxU32 previousSize)
{
	PxBounds3* newBounds = PX_ALLOCATE(PxBounds3, (newSize + 1), "AABBTreeBounds");
	if(mBounds && previousSize)
		PxMemCopy(newBounds, mBounds, sizeof(PxBounds3)*previousSize);
	PX_FREE(mBounds);
	mBounds = newBounds;
}

void AABBTreeBounds::release()
{
	if(!mUserAllocated)
		PX_FREE(mBounds);
}

///////////////////////////////////////////////////////////////////////////////

NodeAllocator::NodeAllocator() : mPool(NULL), mCurrentSlabIndex(0), mTotalNbNodes(0)
{
}

NodeAllocator::~NodeAllocator()
{
	release();
}

void NodeAllocator::release()
{
	const PxU32 nbSlabs = mSlabs.size();
	for (PxU32 i = 0; i<nbSlabs; i++)
	{
		Slab& s = mSlabs[i];
		PX_DELETE_ARRAY(s.mPool);
	}

	mSlabs.reset();
	mCurrentSlabIndex = 0;
	mTotalNbNodes = 0;
}

void NodeAllocator::init(PxU32 nbPrimitives, PxU32 limit)
{
	const PxU32 maxSize = nbPrimitives * 2 - 1;	// PT: max possible #nodes for a complete tree
	const PxU32 estimatedFinalSize = maxSize <= 1024 ? maxSize : maxSize / limit;
	mPool = PX_NEW(AABBTreeBuildNode)[estimatedFinalSize];
	PxMemZero(mPool, sizeof(AABBTreeBuildNode)*estimatedFinalSize);

	// Setup initial node. Here we have a complete permutation of the app's primitives.
	mPool->mNodeIndex = 0;
	mPool->mNbPrimitives = nbPrimitives;

	mSlabs.pushBack(Slab(mPool, 1, estimatedFinalSize));
	mCurrentSlabIndex = 0;
	mTotalNbNodes = 1;
}

// PT: TODO: inline this?
AABBTreeBuildNode* NodeAllocator::getBiNode()
{
	mTotalNbNodes += 2;
	Slab& currentSlab = mSlabs[mCurrentSlabIndex];
	if (currentSlab.mNbUsedNodes + 2 <= currentSlab.mMaxNbNodes)
	{
		AABBTreeBuildNode* biNode = currentSlab.mPool + currentSlab.mNbUsedNodes;
		currentSlab.mNbUsedNodes += 2;
		return biNode;
	}
	else
	{
		// Allocate new slab
		const PxU32 size = 1024;
		AABBTreeBuildNode* pool = PX_NEW(AABBTreeBuildNode)[size];
		PxMemZero(pool, sizeof(AABBTreeBuildNode)*size);

		mSlabs.pushBack(Slab(pool, 2, size));
		mCurrentSlabIndex++;
		return pool;
	}
}

///////////////////////////////////////////////////////////////////////////////

PxU32 Gu::reshuffle(PxU32 nb, PxU32* const PX_RESTRICT prims, const PxVec3* PX_RESTRICT centers, float splitValue, PxU32 axis)
{
	// PT: to avoid calling the unsafe [] operator
	const size_t ptrValue = size_t(centers) + axis*sizeof(float);
	const PxVec3* PX_RESTRICT centersX = reinterpret_cast<const PxVec3*>(ptrValue);

	// Loop through all node-related primitives. Their indices range from mNodePrimitives[0] to mNodePrimitives[mNbPrimitives-1].
	// Those indices map the global list in the tree builder.
	PxU32 nbPos = 0;
	for(PxU32 i=0; i<nb; i++)
	{
		// Get index in global list
		const PxU32 index = prims[i];

		// Test against the splitting value. The primitive value is tested against the enclosing-box center.
		// [We only need an approximate partition of the enclosing box here.]
		const float primitiveValue = centersX[index].x;
		PX_ASSERT(primitiveValue == centers[index][axis]);

		// Reorganize the list of indices in this order: positive - negative.
		if (primitiveValue > splitValue)
		{
			// Swap entries
			prims[i] = prims[nbPos];
			prims[nbPos] = index;
			// Count primitives assigned to positive space
			nbPos++;
		}
	}
	return nbPos;
}

static PxU32 split(const PxBounds3& box, PxU32 nb, PxU32* const PX_RESTRICT prims, PxU32 axis, const AABBTreeBuildParams& params)
{
	// Get node split value
	float splitValue = 0.0f;
	//float defaultSplitValue = box.getCenter(axis);
	//(void)defaultSplitValue;
	if(params.mBuildStrategy==BVH_SPLATTER_POINTS_SPLIT_GEOM_CENTER)
	{
		// PT: experimental attempt at replicating BV4_SPLATTER_POINTS_SPLIT_GEOM_CENTER, but with boxes instead of triangles.
		const PxBounds3* bounds = params.mBounds->getBounds();
		for(PxU32 i=0;i<nb;i++)
		{
			const PxBounds3& current = bounds[prims[i]];
			splitValue += current.getCenter(axis);
//			splitValue += (*VP.Vertex[0])[axis];
//			splitValue += (*VP.Vertex[1])[axis];
//			splitValue += (*VP.Vertex[2])[axis];
		}
//		splitValue /= float(nb*3);
		splitValue /= float(nb);
	}
	else
	{
		// Default split value = middle of the axis (using only the box)
		splitValue = box.getCenter(axis);
	}

	return reshuffle(nb, prims, params.mCache, splitValue, axis);
}

void AABBTreeBuildNode::subdivide(const AABBTreeBuildParams& params, BuildStats& stats, NodeAllocator& allocator, PxU32* const indices)
{
	PxU32* const PX_RESTRICT primitives = indices + mNodeIndex;
	const PxU32 nbPrims = mNbPrimitives;

	// Compute global box & means for current node. The box is stored in mBV.
	Vec4V meansV;
	{
		const PxBounds3* PX_RESTRICT boxes = params.mBounds->getBounds();
		PX_ASSERT(boxes);
		PX_ASSERT(primitives);
		PX_ASSERT(nbPrims);

		Vec4V minV = V4LoadU(&boxes[primitives[0]].minimum.x);
		Vec4V maxV = V4LoadU(&boxes[primitives[0]].maximum.x);

		meansV = V4LoadU(&params.mCache[primitives[0]].x);

		for (PxU32 i = 1; i<nbPrims; i++)
		{
			const PxU32 index = primitives[i];
			const Vec4V curMinV = V4LoadU(&boxes[index].minimum.x);
			const Vec4V curMaxV = V4LoadU(&boxes[index].maximum.x);
			meansV = V4Add(meansV, V4LoadU(&params.mCache[index].x));
			minV = V4Min(minV, curMinV);
			maxV = V4Max(maxV, curMaxV);
		}

		StoreBounds(mBV, minV, maxV);

		const float coeff = 1.0f / float(nbPrims);
		meansV = V4Scale(meansV, FLoad(coeff));
	}

	// Check the user-defined limit. Also ensures we stop subdividing if we reach a leaf node.
	if (nbPrims <= params.mLimit)
		return;

	bool validSplit = true;
	PxU32 nbPos;
	{
		// Compute variances
		Vec4V varsV = V4Zero();
		for (PxU32 i = 0; i<nbPrims; i++)
		{
			const PxU32 index = primitives[i];
			Vec4V centerV = V4LoadU(&params.mCache[index].x);
			centerV = V4Sub(centerV, meansV);
			centerV = V4Mul(centerV, centerV);
			varsV = V4Add(varsV, centerV);
		}
		const float coeffNb1 = 1.0f / float(nbPrims - 1);
		varsV = V4Scale(varsV, FLoad(coeffNb1));
		PX_ALIGN(16, PxVec4) vars;
		V4StoreA(varsV, &vars.x);

		// Choose axis with greatest variance
		const PxU32 axis = PxLargestAxis(PxVec3(vars.x, vars.y, vars.z));

		// Split along the axis
		nbPos = split(mBV, nbPrims, primitives, axis, params);

		// Check split validity
		if (!nbPos || nbPos == nbPrims)
			validSplit = false;
	}

	// Check the subdivision has been successful
	if (!validSplit)
	{
		// Here, all boxes lie in the same sub-space. Two strategies:
		// - if we are over the split limit, make an arbitrary 50-50 split
		// - else stop subdividing
		if (nbPrims>params.mLimit)
		{
			nbPos = nbPrims >> 1;
		}
		else return;
	}

	// Now create children and assign their pointers.
	mPos = allocator.getBiNode();

	stats.increaseCount(2);

	// Assign children
	PX_ASSERT(!isLeaf());
	AABBTreeBuildNode* Pos = const_cast<AABBTreeBuildNode*>(mPos);
	AABBTreeBuildNode* Neg = Pos + 1;
	Pos->mNodeIndex = mNodeIndex;
	Pos->mNbPrimitives = nbPos;
	Neg->mNodeIndex = mNodeIndex + nbPos;
	Neg->mNbPrimitives = mNbPrimitives - nbPos;
}

void AABBTreeBuildNode::_buildHierarchy(const AABBTreeBuildParams& params, BuildStats& stats, NodeAllocator& nodeBase, PxU32* const indices)
{
	// Subdivide current node
	subdivide(params, stats, nodeBase, indices);

	// Recurse
	if (!isLeaf())
	{
		AABBTreeBuildNode* Pos = const_cast<AABBTreeBuildNode*>(getPos());
		PX_ASSERT(Pos);
		AABBTreeBuildNode* Neg = Pos + 1;
		Pos->_buildHierarchy(params, stats, nodeBase, indices);
		Neg->_buildHierarchy(params, stats, nodeBase, indices);
	}

	stats.mTotalPrims += mNbPrimitives;
}

void AABBTreeBuildNode::subdivideSAH(const AABBTreeBuildParams& params, SAH_Buffers& buffers, BuildStats& stats, NodeAllocator& allocator, PxU32* const indices)
{
	PxU32* const PX_RESTRICT primitives = indices + mNodeIndex;
	const PxU32 nbPrims = mNbPrimitives;

	// Compute global box for current node. The box is stored in mBV.
	computeGlobalBox(mBV, nbPrims, params.mBounds->getBounds(), primitives);

	// Check the user-defined limit. Also ensures we stop subdividing if we reach a leaf node.
	if (nbPrims <= params.mLimit)
		return;

	/////

	PxU32 leftCount;
	if(!buffers.split(leftCount, nbPrims, primitives, params.mBounds->getBounds(), params.mCache))
	{
		// Invalid split => fallback to previous strategy
		subdivide(params, stats, allocator, indices);
		return;
	}

	/////

	// Now create children and assign their pointers.
	mPos = allocator.getBiNode();

	stats.increaseCount(2);

	// Assign children
	PX_ASSERT(!isLeaf());
	AABBTreeBuildNode* Pos = const_cast<AABBTreeBuildNode*>(mPos);
	AABBTreeBuildNode* Neg = Pos + 1;
	Pos->mNodeIndex = mNodeIndex;
	Pos->mNbPrimitives = leftCount;
	Neg->mNodeIndex = mNodeIndex + leftCount;
	Neg->mNbPrimitives = mNbPrimitives - leftCount;
}

void AABBTreeBuildNode::_buildHierarchySAH(const AABBTreeBuildParams& params, SAH_Buffers& sah, BuildStats& stats, NodeAllocator& nodeBase, PxU32* const indices)
{
	// Subdivide current node
	subdivideSAH(params, sah, stats, nodeBase, indices);

	// Recurse
	if (!isLeaf())
	{
		AABBTreeBuildNode* Pos = const_cast<AABBTreeBuildNode*>(getPos());
		PX_ASSERT(Pos);
		AABBTreeBuildNode* Neg = Pos + 1;
		Pos->_buildHierarchySAH(params, sah, stats, nodeBase, indices);
		Neg->_buildHierarchySAH(params, sah, stats, nodeBase, indices);
	}

	stats.mTotalPrims += mNbPrimitives;
}

///////////////////////////////////////////////////////////////////////////////

static PxU32* initAABBTreeBuild(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats)
{
	const PxU32 numPrimitives = params.mNbPrimitives;

	if(!numPrimitives)
		return NULL;

	// Init stats
	stats.setCount(1);

	// Initialize indices. This list will be modified during build.
	PxU32* indices = PX_ALLOCATE(PxU32, numPrimitives, "AABB tree indices");
	// Identity permutation
	for(PxU32 i=0;i<numPrimitives;i++)
		indices[i] = i;

	// Allocate a pool of nodes
	nodeAllocator.init(numPrimitives, params.mLimit);

	// Compute box centers only once and cache them
	params.mCache = PX_ALLOCATE(PxVec3, (numPrimitives+1), "cache");
	const PxBounds3* PX_RESTRICT boxes = params.mBounds->getBounds();
	const float half = 0.5f;
	const FloatV halfV = FLoad(half);
	for(PxU32 i=0;i<numPrimitives;i++)
	{
		const Vec4V curMinV = V4LoadU(&boxes[i].minimum.x);
		const Vec4V curMaxV = V4LoadU(&boxes[i].maximum.x);
		const Vec4V centerV = V4Scale(V4Add(curMaxV, curMinV), halfV);
		V4StoreU(centerV, &params.mCache[i].x);
	}

	return indices;
}

PxU32* Gu::buildAABBTree(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats)
{
	// initialize the build first
	PxU32* indices = initAABBTreeBuild(params, nodeAllocator, stats);
	if(!indices)
		return NULL;

	// Build the hierarchy
	if(params.mBuildStrategy==BVH_SAH)
	{
		SAH_Buffers buffers(params.mNbPrimitives);
		nodeAllocator.mPool->_buildHierarchySAH(params, buffers, stats, nodeAllocator, indices);
	}
	else
		nodeAllocator.mPool->_buildHierarchy(params, stats, nodeAllocator, indices);

	return indices;
}

void Gu::flattenTree(const NodeAllocator& nodeAllocator, BVHNode* dest, const PxU32* remap)
{
	// PT: gathers all build nodes allocated so far and flatten them to a linear destination array of smaller runtime nodes
	PxU32 offset = 0;
	const PxU32 nbSlabs = nodeAllocator.mSlabs.size();
	for(PxU32 s=0;s<nbSlabs;s++)
	{
		const NodeAllocator::Slab& currentSlab = nodeAllocator.mSlabs[s];

		AABBTreeBuildNode* pool = currentSlab.mPool;
		for(PxU32 i=0;i<currentSlab.mNbUsedNodes;i++)
		{
			dest[offset].mBV = pool[i].mBV;
			if(pool[i].isLeaf())
			{
				PxU32 index = pool[i].mNodeIndex;
				if(remap)
					index = remap[index];

				const PxU32 nbPrims = pool[i].getNbPrimitives();
				PX_ASSERT(nbPrims<16);

				dest[offset].mData = (index<<5)|((nbPrims&15)<<1)|1;
			}
			else
			{
				PX_ASSERT(pool[i].mPos);
				PxU32 localNodeIndex = 0xffffffff;
				PxU32 nodeBase = 0;
				for(PxU32 j=0;j<nbSlabs;j++)
				{
					if(pool[i].mPos >= nodeAllocator.mSlabs[j].mPool && pool[i].mPos < nodeAllocator.mSlabs[j].mPool + nodeAllocator.mSlabs[j].mNbUsedNodes)
					{
						localNodeIndex = PxU32(pool[i].mPos - nodeAllocator.mSlabs[j].mPool);
						break;
					}
					nodeBase += nodeAllocator.mSlabs[j].mNbUsedNodes;
				}
				const PxU32 nodeIndex = nodeBase + localNodeIndex;
				dest[offset].mData = nodeIndex << 1;
			}
			offset++;
		}
	}
}

void Gu::buildAABBTree(PxU32 nbBounds, const AABBTreeBounds& bounds, PxArray<BVHNode>& tree)
{
	PX_SIMD_GUARD
	// build the BVH
	BuildStats stats;
	NodeAllocator nodeAllocator;

	PxU32* indices = buildAABBTree(AABBTreeBuildParams(1, nbBounds, &bounds), nodeAllocator, stats);
	PX_ASSERT(indices);

	// store the computed hierarchy
	tree.resize(stats.getCount());
	PX_ASSERT(tree.size() == nodeAllocator.mTotalNbNodes);

	// store the results into BVHNode list
	flattenTree(nodeAllocator, tree.begin(), indices);
	PX_FREE(indices);	// PT: we don't need the indices for a complete tree
}

///////////////////////////////////////////////////////////////////////////////

// Progressive building
class Gu::FIFOStack : public PxUserAllocated
{
public:
												FIFOStack() : mStack("SQFIFOStack"), mCurIndex(0)	{}
												~FIFOStack()										{}

	PX_FORCE_INLINE	PxU32						getNbEntries()	const			{ return mStack.size();		}
	PX_FORCE_INLINE	void						push(AABBTreeBuildNode* entry)	{ mStack.pushBack(entry);	}
					bool						pop(AABBTreeBuildNode*& entry);
private:
					PxArray<AABBTreeBuildNode*>	mStack;
					PxU32						mCurIndex;			//!< Current index within the container
};

bool Gu::FIFOStack::pop(AABBTreeBuildNode*& entry)
{
	const PxU32 NbEntries = mStack.size(); // Get current number of entries
	if (!NbEntries)
		return false; // Can be NULL when no value has been pushed. This is an invalid pop call.
	entry = mStack[mCurIndex++]; // Get oldest entry, move to next one
	if (mCurIndex == NbEntries)
	{
		// All values have been poped
		mStack.clear();
		mCurIndex = 0;
	}
	return true;
}
//~Progressive building

///////////////////////////////////////////////////////////////////////////////

BVHPartialRefitData::BVHPartialRefitData() : mParentIndices(NULL), mUpdateMap(NULL), mRefitHighestSetWord(0)
{
}

BVHPartialRefitData::~BVHPartialRefitData()
{
	releasePartialRefitData(true);
}

void BVHPartialRefitData::releasePartialRefitData(bool clearRefitMap)
{
	PX_FREE(mParentIndices);
	PX_FREE(mUpdateMap);

	if(clearRefitMap)
		mRefitBitmask.clearAll();
	mRefitHighestSetWord = 0;
}

static void createParentArray(PxU32 totalNbNodes, PxU32* parentIndices, const BVHNode* parentNode, const BVHNode* currentNode, const BVHNode* root)
{
	const PxU32 parentIndex = PxU32(parentNode - root);
	const PxU32 currentIndex = PxU32(currentNode - root);
	PX_ASSERT(parentIndex<totalNbNodes);
	PX_ASSERT(currentIndex<totalNbNodes);
	PX_UNUSED(totalNbNodes);
	parentIndices[currentIndex] = parentIndex;

	if(!currentNode->isLeaf())
	{
		createParentArray(totalNbNodes, parentIndices, currentNode, currentNode->getPos(root), root);
		createParentArray(totalNbNodes, parentIndices, currentNode, currentNode->getNeg(root), root);
	}
}

PxU32* BVHPartialRefitData::getParentIndices()
{
	// PT: lazy-create parent array. Memory is not wasted for purely static trees, or dynamic trees that only do "full refit".
	if(!mParentIndices)
	{
		mParentIndices = PX_ALLOCATE(PxU32, mNbNodes, "AABB parent indices");
		createParentArray(mNbNodes, mParentIndices, mNodes, mNodes, mNodes);
	}
	return mParentIndices;
}

void BVHPartialRefitData::createUpdateMap(PxU32 nbObjects)
{
	// PT: we need an "update map" for PxBVH
	// PT: TODO: consider refactoring with the AABBtree version
	PX_FREE(mUpdateMap);

	if(!nbObjects)
		return;

	mUpdateMap = PX_ALLOCATE(PxU32, nbObjects, "UpdateMap");
	PxMemSet(mUpdateMap, 0xff, sizeof(PxU32)*nbObjects);

	const PxU32 nbNodes = mNbNodes;
	const BVHNode* nodes = mNodes;
	const PxU32* indices = mIndices;
	for(TreeNodeIndex i=0;i<nbNodes;i++)
	{
		if(nodes[i].isLeaf())
		{
			const PxU32 nbPrims = nodes[i].getNbRuntimePrimitives();
			if(indices)
			{
				// PT: with multiple primitives per node, several mapping entries will point to the same node.
				PX_ASSERT(nbPrims<16);
				for(PxU32 j=0;j<nbPrims;j++)
				{
					const PxU32 index = nodes[i].getPrimitives(indices)[j];
					PX_ASSERT(index<nbObjects);
					mUpdateMap[index] = i;
				}
			}
			else
			{
				PX_ASSERT(nbPrims==1);
				const PxU32 index = nodes[i].getPrimitiveIndex();
				PX_ASSERT(index<nbObjects);
				mUpdateMap[index] = i;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE PxU32 BitsToDwords(PxU32 nb_bits)
{
	return (nb_bits>>5) + ((nb_bits&31) ? 1 : 0);
}

bool BitArray::init(PxU32 nb_bits)
{
	mSize = BitsToDwords(nb_bits);
	// Get ram for n bits
	PX_FREE(mBits);
	mBits = PX_ALLOCATE(PxU32, mSize, "BitArray::mBits");
	// Set all bits to 0
	clearAll();
	return true;
}

void BitArray::resize(PxU32 maxBitNumber)
{
	const PxU32 newSize = BitsToDwords(maxBitNumber);
	if (newSize <= mSize)
		return;

	PxU32* newBits = PX_ALLOCATE(PxU32, newSize, "BitArray::mBits");
	PxMemZero(newBits + mSize, (newSize - mSize) * sizeof(PxU32));
	PxMemCopy(newBits, mBits, mSize*sizeof(PxU32));
	PX_FREE(mBits);
	mBits = newBits;
	mSize = newSize;
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE	PxU32			getNbPrimitives(PxU32 data)						{ return (data>>1)&15;		}
static PX_FORCE_INLINE	const PxU32*	getPrimitives(const PxU32* base, PxU32 data)	{ return base + (data>>5);	}
static PX_FORCE_INLINE	const BVHNode*	getPos(const BVHNode* base, PxU32 data)			{ return base + (data>>1);	}
static PX_FORCE_INLINE	PxU32			isLeaf(PxU32 data)								{ return data&1;			}

template<const bool hasIndices>
static PX_FORCE_INLINE void refitNode(BVHNode* PX_RESTRICT current, const PxBounds3* PX_RESTRICT boxes, const PxU32* PX_RESTRICT indices, BVHNode* PX_RESTRICT const nodeBase)
{
	// PT: we can safely use V4 loads on both boxes and nodes here:
	// - it's safe on boxes because we allocated one extra box in the pruning pool
	// - it's safe on nodes because there's always some data within the node, after the BV

	const PxU32 data = current->mData;

	Vec4V resultMinV, resultMaxV;
	if(isLeaf(data))
	{
		const PxU32 nbPrims = getNbPrimitives(data);
		if(nbPrims)
		{
			if(hasIndices)
			{
				const PxU32* primitives = getPrimitives(indices, data);
				resultMinV = V4LoadU(&boxes[*primitives].minimum.x);
				resultMaxV = V4LoadU(&boxes[*primitives].maximum.x);

				if(nbPrims>1)
				{
					const PxU32* last = primitives + nbPrims;
					primitives++;

					while(primitives!=last)
					{
						resultMinV = V4Min(resultMinV, V4LoadU(&boxes[*primitives].minimum.x));
						resultMaxV = V4Max(resultMaxV, V4LoadU(&boxes[*primitives].maximum.x));
						primitives++;
					}
				}
			}
			else
			{
				PX_ASSERT(nbPrims==1);
				const PxU32 primIndex = data>>5;
				resultMinV = V4LoadU(&boxes[primIndex].minimum.x);
				resultMaxV = V4LoadU(&boxes[primIndex].maximum.x);
			}
		}
		else
		{
			// Might happen after a node has been invalidated
			const float max = GU_EMPTY_BOUNDS_EXTENTS;
			resultMinV = V4Load(max);
			resultMaxV = V4Load(-max);
		}
	}
	else
	{
		const BVHNode* pos = getPos(nodeBase, data);
		const BVHNode* neg = pos+1;

		const PxBounds3& posBox = pos->mBV;
		const PxBounds3& negBox = neg->mBV;

		resultMinV = V4Min(V4LoadU(&posBox.minimum.x), V4LoadU(&negBox.minimum.x));
//		resultMaxV = V4Max(V4LoadU(&posBox.maximum.x), V4LoadU(&negBox.maximum.x));

#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		Vec4V posMinV = V4LoadU(&posBox.minimum.z);
		Vec4V negMinV = V4LoadU(&negBox.minimum.z);
		posMinV = _mm_shuffle_ps(posMinV, posMinV, _MM_SHUFFLE(0, 3, 2, 1));
		negMinV = _mm_shuffle_ps(negMinV, negMinV, _MM_SHUFFLE(0, 3, 2, 1));
		resultMaxV = V4Max(posMinV, negMinV);
#else
		// PT: fixes the perf issue but not really convincing
		resultMaxV = Vec4V_From_Vec3V(V3Max(V3LoadU(&posBox.maximum.x), V3LoadU(&negBox.maximum.x)));
#endif
	}

	// PT: the V4 stores overwrite the data after the BV, but we just put it back afterwards
	V4StoreU(resultMinV, &current->mBV.minimum.x);
	V4StoreU(resultMaxV, &current->mBV.maximum.x);
	current->mData = data;
}

template<const bool hasIndices>
static void refitLoop(const PxBounds3* PX_RESTRICT boxes, BVHNode* const PX_RESTRICT nodeBase, const PxU32* PX_RESTRICT indices, PxU32 nbNodes)
{
	PX_ASSERT(boxes);
	PX_ASSERT(nodeBase);
	// Bottom-up update
	PxU32 index = nbNodes;
	while(index--)
	{
		BVHNode* current = nodeBase + index;
		if(index)
			PxPrefetch(current - 1);

//		PxBounds3 before = current->mBV;

		if(hasIndices)
			refitNode<1>(current, boxes, indices, nodeBase);
		else
			refitNode<0>(current, boxes, indices, nodeBase);

//		if(current->mBV.minimum==before.minimum && current->mBV.maximum==before.maximum)
//			break;
	}
}

void BVHCoreData::fullRefit(const PxBounds3* boxes)
{
	if(mIndices)
		refitLoop<1>(boxes, mNodes, mIndices, mNbNodes);
	else
		refitLoop<0>(boxes, mNodes, mIndices, mNbNodes);
}

void BVHPartialRefitData::markNodeForRefit(TreeNodeIndex nodeIndex)
{
	BitArray* PX_RESTRICT refitBitmask = &mRefitBitmask;

	if(!refitBitmask->getBits())
		refitBitmask->init(mNbNodes);

	PX_ASSERT(nodeIndex<mNbNodes);

	const PxU32* PX_RESTRICT parentIndices = getParentIndices();

	PxU32 refitHighestSetWord = mRefitHighestSetWord;

	PxU32 currentIndex = nodeIndex;
	while(1)
	{
		PX_ASSERT(currentIndex<mNbNodes);
		if(refitBitmask->isSet(currentIndex))
		{
			// We can early exit if we already visited the node!
			goto Exit;
		}
		else
		{
			refitBitmask->setBit(currentIndex);
			const PxU32 currentMarkedWord = currentIndex>>5;
			refitHighestSetWord = PxMax(refitHighestSetWord, currentMarkedWord);

			const PxU32 parentIndex = parentIndices[currentIndex];			
			PX_ASSERT(parentIndex == 0 || parentIndex < currentIndex);
			if(currentIndex == parentIndex)
				break;
			currentIndex = parentIndex;
		}
	}
Exit:
	mRefitHighestSetWord = refitHighestSetWord;
}

#define FIRST_VERSION
#ifdef FIRST_VERSION
template<const bool hasIndices>
static void refitMarkedLoop(const PxBounds3* PX_RESTRICT boxes, BVHNode* const PX_RESTRICT nodeBase, const PxU32* PX_RESTRICT indices, PxU32* PX_RESTRICT bits, PxU32 nbToGo)
{
#ifdef _DEBUG
	PxU32 nbRefit=0;
#endif

	PxU32 size = nbToGo;

	while(size--)
	{
		// Test 32 bits at a time
		const PxU32 currentBits = bits[size];
		if(!currentBits)
			continue;

		PxU32 index = (size+1)<<5;
		PxU32 mask = PxU32(1<<((index-1)&31));
		PxU32 count=32;
		while(count--)
		{
			index--;
			PxPrefetch(nodeBase + index);

			PX_ASSERT(size==index>>5);
			PX_ASSERT(mask==PxU32(1<<(index&31)));
			if(currentBits & mask)
			{
				if(hasIndices)
					refitNode<1>(nodeBase + index, boxes, indices, nodeBase);
				else
					refitNode<0>(nodeBase + index, boxes, indices, nodeBase);
#ifdef _DEBUG
				nbRefit++;
#endif
			}
			mask>>=1;
		}
		bits[size] = 0;
	}
}

void BVHPartialRefitData::refitMarkedNodes(const PxBounds3* boxes)
{
	if(!mRefitBitmask.getBits())
		return;	// No refit needed

	{
		/*const*/ PxU32* bits = const_cast<PxU32*>(mRefitBitmask.getBits());
		PxU32 size = mRefitHighestSetWord+1;
#ifdef _DEBUG
		if(1)
		{
			const PxU32 totalSize = mRefitBitmask.getSize();
			for(PxU32 i=size;i<totalSize;i++)
			{
				PX_ASSERT(!bits[i]);
			}
		}
#endif
		if(mIndices)
			refitMarkedLoop<1>(boxes, mNodes, mIndices, bits, size);
		else
			refitMarkedLoop<0>(boxes, mNodes, mIndices, bits, size);

		mRefitHighestSetWord = 0;
//		mRefitBitmask.clearAll();
	}
}
#endif


//#define SECOND_VERSION
#ifdef SECOND_VERSION
void BVHPartialRefitData::refitMarkedNodes(const PxBounds3* boxes)
{
	/*const*/ PxU32* bits = const_cast<PxU32*>(mRefitBitmask.getBits());
	if(!bits)
		return;	// No refit needed

	const PxU32 lastSetBit = mRefitBitmask.findLast();

	const PxU32* indices = mIndices;
	BVHNode* const nodeBase = mNodes;

	// PT: ### bitmap iterator pattern
	for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
	{
		for(PxU32 b = bits[w]; b; b &= b-1)
		{
			const PxU32 index = (PxU32)(w<<5|PxLowestSetBit(b));



		while(size--)
		{
			// Test 32 bits at a time
			const PxU32 currentBits = bits[size];
			if(!currentBits)
				continue;

			PxU32 index = (size+1)<<5;
			PxU32 mask = PxU32(1<<((index-1)&31));
			PxU32 count=32;
			while(count--)
			{
				index--;
				PxPrefetch(nodeBase + index);

				PX_ASSERT(size==index>>5);
				PX_ASSERT(mask==PxU32(1<<(index&31)));
				if(currentBits & mask)
				{
					refitNode(nodeBase + index, boxes, indices, nodeBase);
#ifdef _DEBUG
					nbRefit++;
#endif
				}
				mask>>=1;
			}
			bits[size] = 0;
		}
		mRefitHighestSetWord = 0;
//		mRefitBitmask.clearAll();
	}
}
#endif

///////////////////////////////////////////////////////////////////////////////

AABBTree::AABBTree() : mTotalPrims(0)
{
// Progressive building
	mStack = NULL;
//~Progressive building
}

AABBTree::~AABBTree()
{
	release(false);
}

void AABBTree::release(bool clearRefitMap)
{
// Progressive building
	PX_DELETE(mStack);
//~Progressive building
	releasePartialRefitData(clearRefitMap);
	// PT: TODO: move some to BVHCoreData dtor
	PX_DELETE_ARRAY(mNodes);
	PX_FREE(mIndices);
	mNbNodes = 0;
	mNbIndices = 0;
}

// Initialize nodes/indices from the input tree merge data
void AABBTree::initTree(const AABBTreeMergeData& tree)
{
	PX_ASSERT(mIndices == NULL);
	PX_ASSERT(mNodes == NULL);
	PX_ASSERT(mParentIndices == NULL);

	// allocate,copy indices
	mIndices = PX_ALLOCATE(PxU32, tree.mNbIndices, "AABB tree indices");
	mNbIndices = tree.mNbIndices;
	PxMemCopy(mIndices, tree.mIndices, sizeof(PxU32)*tree.mNbIndices);

	// allocate,copy nodes
	mNodes = PX_NEW(BVHNode)[tree.mNbNodes];
	mNbNodes = tree.mNbNodes;
	PxMemCopy(mNodes, tree.mNodes, sizeof(BVHNode)*tree.mNbNodes);
}

// Shift indices of the tree by offset. Used for merged trees, when initial indices needs to be shifted to match indices in current pruning pool
void AABBTree::shiftIndices(PxU32 offset)
{
	for (PxU32 i = 0; i < mNbIndices; i++)
	{
		mIndices[i] += offset;
	}
}

bool AABBTree::buildInit(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats)
{
	// Checkings
	const PxU32 nbPrimitives = params.mNbPrimitives;
	if(!nbPrimitives)
		return false;

	// Release previous tree
	release();

	// Initialize indices. This list will be modified during build.
	mNbIndices = nbPrimitives;

	PxU32* indices = initAABBTreeBuild(params, nodeAllocator, stats);
	if(!indices)
		return false;

	PX_ASSERT(!mIndices);
	mIndices = indices;

	return true;
}

void AABBTree::buildEnd(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, const BuildStats& stats)
{
	PX_FREE(params.mCache);
	// Get back total number of nodes
	mNbNodes	= stats.getCount();
	mTotalPrims	= stats.mTotalPrims;

	mNodes = PX_NEW(BVHNode)[mNbNodes];
	PX_ASSERT(mNbNodes==nodeAllocator.mTotalNbNodes);
	flattenTree(nodeAllocator, mNodes);
	nodeAllocator.release();
}

bool AABBTree::build(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator)
{
	const PxU32 nbPrimitives = params.mNbPrimitives;
	if(!nbPrimitives)
		return false;

	// Release previous tree
	release();

	BuildStats stats;
	mNbIndices = nbPrimitives;

	mIndices = buildAABBTree(params, nodeAllocator, stats);
	if(!mIndices)
		return false;

	buildEnd(params, nodeAllocator, stats);
	return true;
}

void AABBTree::shiftOrigin(const PxVec3& shift)
{
	BVHNode* const nodeBase = mNodes;
	const PxU32 totalNbNodes = mNbNodes;
	for(PxU32 i=0; i<totalNbNodes; i++)
	{
		BVHNode& current = nodeBase[i];
		if((i+1) < totalNbNodes)
			PxPrefetch(nodeBase + i + 1);

		current.mBV.minimum -= shift;
		current.mBV.maximum -= shift;
	}
}

// Progressive building
static PxU32 incrementalBuildHierarchy(FIFOStack& stack, AABBTreeBuildNode* node, const AABBTreeBuildParams& params, BuildStats& stats, NodeAllocator& nodeBase, PxU32* const indices)
{
	node->subdivide(params, stats, nodeBase, indices);

	if(!node->isLeaf())
	{
		AABBTreeBuildNode* pos = const_cast<AABBTreeBuildNode*>(node->getPos());
		PX_ASSERT(pos);
		AABBTreeBuildNode* neg = pos + 1;
		stack.push(neg);
		stack.push(pos);
	}

	stats.mTotalPrims += node->mNbPrimitives;
	return node->mNbPrimitives;
}

PxU32 AABBTree::progressiveBuild(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats, PxU32 progress, PxU32 limit)
{
	if(progress==0)
	{
		if(!buildInit(params, nodeAllocator, stats))
			return PX_INVALID_U32;

		mStack = PX_NEW(FIFOStack);
		mStack->push(nodeAllocator.mPool);
		return progress++;
	}
	else if(progress==1)
	{
		PxU32 stackCount = mStack->getNbEntries();
		if(stackCount)
		{
			PxU32 Total = 0;
			const PxU32 Limit = limit;
			while(Total<Limit)
			{
				AABBTreeBuildNode* Entry;
				if(mStack->pop(Entry))
					Total += incrementalBuildHierarchy(*mStack, Entry, params, stats, nodeAllocator, mIndices);
				else
					break;
			}
			return progress;
		}

		buildEnd(params, nodeAllocator, stats);

		PX_DELETE(mStack);

		return 0;	// Done!
	}
	return PX_INVALID_U32;
}
//~Progressive building

PX_FORCE_INLINE static void setLeafData(PxU32& leafData, const BVHNode& node, const PxU32 indicesOffset)
{
	const PxU32 index = indicesOffset + (node.mData >> 5);
	const PxU32 nbPrims = node.getNbPrimitives();
	PX_ASSERT(nbPrims < 16);
	leafData = (index << 5) | ((nbPrims & 15) << 1) | 1;
}

// Copy the tree into nodes. Update node indices, leaf indices.
void AABBTree::addRuntimeChilds(PxU32& nodeIndex, const AABBTreeMergeData& treeParams)
{
	PX_ASSERT(nodeIndex < mNbNodes + treeParams.mNbNodes + 1);	
	const PxU32 baseNodeIndex = nodeIndex;	

	// copy the src tree into dest tree nodes, update its data
	for (PxU32 i = 0; i < treeParams.mNbNodes; i++)
	{
		PX_ASSERT(nodeIndex < mNbNodes + treeParams.mNbNodes  + 1);
		mNodes[nodeIndex].mBV = treeParams.mNodes[i].mBV;
		if (treeParams.mNodes[i].isLeaf())
		{
			setLeafData(mNodes[nodeIndex].mData, treeParams.mNodes[i], mNbIndices);
		}
		else
		{
			const PxU32 srcNodeIndex = baseNodeIndex + (treeParams.mNodes[i].getPosIndex());
			mNodes[nodeIndex].mData = srcNodeIndex << 1;
			mParentIndices[srcNodeIndex] = nodeIndex;
			mParentIndices[srcNodeIndex + 1] = nodeIndex;
		}
		nodeIndex++;
	}
}

// Merge tree into targetNode, where target node is a leaf
// 1. Allocate new nodes/parent, copy all the nodes/parents
// 2. Create new node at the end, copy the data from target node	
// 3. Copy the merge tree after the new node, create the parent map for them, update the leaf indices
// Schematic view:
// Target Nodes: ...Tn...
// Input tree: R1->Rc0, Rc1...
// Merged tree: ...Tnc->...->Nc0,R1->Rc0,Rc1...
//		where new node:		Nc0==Tn and Tnc is not a leaf anymore and points to Nc0

void AABBTree::mergeRuntimeLeaf(BVHNode& targetNode, const AABBTreeMergeData& treeParams, PxU32 targetMergeNodeIndex)
{
	PX_ASSERT(mParentIndices);
	PX_ASSERT(targetNode.isLeaf());	

	// 1. Allocate new nodes/parent, copy all the nodes/parents
	// allocate new runtime pool with max combine number of nodes
	// we allocate only 1 additional node each merge
	BVHNode* newRuntimePool = PX_NEW(BVHNode)[mNbNodes + treeParams.mNbNodes + 1];
	PxU32* newParentIndices = PX_ALLOCATE(PxU32, (mNbNodes + treeParams.mNbNodes + 1), "AABB parent indices");

	// copy the whole target nodes, we will add the new node at the end together with the merge tree
	PxMemCopy(newRuntimePool, mNodes, sizeof(BVHNode)*(mNbNodes));
	PxMemCopy(newParentIndices, mParentIndices, sizeof(PxU32)*(mNbNodes));

	// 2. Create new node at the end, copy the data from target node	
	PxU32 nodeIndex = mNbNodes;	
	// copy the targetNode at the end of the new nodes
	newRuntimePool[nodeIndex].mBV = targetNode.mBV;
	newRuntimePool[nodeIndex].mData = targetNode.mData;
	// update the parent information
	newParentIndices[nodeIndex] = targetMergeNodeIndex;

	// mark for refit
	if (mRefitBitmask.getBits() && mRefitBitmask.isSet(targetMergeNodeIndex))
	{
		mRefitBitmask.setBit(nodeIndex);
		const PxU32 currentMarkedWord = nodeIndex >> 5;
		mRefitHighestSetWord = PxMax(mRefitHighestSetWord, currentMarkedWord);
	}

	// swap pointers
	PX_DELETE_ARRAY(mNodes);
	mNodes = newRuntimePool;
	PX_FREE(mParentIndices);
	mParentIndices = newParentIndices;

	// 3. Copy the merge tree after the new node, create the parent map for them, update the leaf indices
	nodeIndex++;
	addRuntimeChilds(nodeIndex, treeParams);
	PX_ASSERT(nodeIndex == mNbNodes + 1 + treeParams.mNbNodes);	

	// update the parent information for the input tree root node
	mParentIndices[mNbNodes + 1] = targetMergeNodeIndex;

	// fix the child information for the target node, was a leaf before
	mNodes[targetMergeNodeIndex].mData = mNbNodes << 1;

	// update the total number of nodes
	mNbNodes = mNbNodes + 1 + treeParams.mNbNodes;
}

// Merge tree into targetNode, where target node is not a leaf
// 1. Allocate new nodes/parent, copy the nodes/parents till targetNodePosIndex
// 2. Create new node , copy the data from target node	
// 3. Copy the rest of the target tree nodes/parents at the end -> targetNodePosIndex + 1 + treeParams.mNbNodes
// 4. Copy the merge tree after the new node, create the parent map for them, update the leaf indices
// 5. Go through the nodes copied at the end and fix the parents/childs
// Schematic view:
// Target Nodes: ...Tn->...->Tc0,Tc1...
// Input tree: R1->Rc0, Rc1...
// Merged tree: ...Tn->...->Nc0,R1->Rc0,Rc1...,Tc0,Tc1...       
//		where new node:		Nc0->...->Tc0,Tc1
void AABBTree::mergeRuntimeNode(BVHNode& targetNode, const AABBTreeMergeData& treeParams, PxU32 targetMergeNodeIndex)
{
	PX_ASSERT(mParentIndices);	
	PX_ASSERT(!targetNode.isLeaf());

	// Get the target node child pos, this is where we insert the new node and the input tree
	const PxU32 targetNodePosIndex = targetNode.getPosIndex();

	// 1. Allocate new nodes/parent, copy the nodes/parents till targetNodePosIndex
	// allocate new runtime pool with max combine number of nodes
	// we allocate only 1 additional node each merge
	BVHNode* newRuntimePool = PX_NEW(BVHNode)[mNbNodes + treeParams.mNbNodes + 1];
	PxU32* newParentIndices = PX_ALLOCATE(PxU32, (mNbNodes + treeParams.mNbNodes + 1), "AABB parent indices");
	// copy the untouched part of the nodes and parents
	PxMemCopy(newRuntimePool, mNodes, sizeof(BVHNode)*(targetNodePosIndex));
	PxMemCopy(newParentIndices, mParentIndices, sizeof(PxU32)*(targetNodePosIndex));

	PxU32 nodeIndex = targetNodePosIndex;
	// 2. Create new node , copy the data from target node	
	newRuntimePool[nodeIndex].mBV = targetNode.mBV;
	newRuntimePool[nodeIndex].mData = ((targetNode.mData >> 1) + 1 + treeParams.mNbNodes) << 1;
	// update parent information
	newParentIndices[nodeIndex] = targetMergeNodeIndex;

	// handle mark for refit
	if(mRefitBitmask.getBits() && mRefitBitmask.isSet(targetMergeNodeIndex))
	{
		mRefitBitmask.setBit(nodeIndex);
		const PxU32 currentMarkedWord = nodeIndex >> 5;
		mRefitHighestSetWord = PxMax(mRefitHighestSetWord, currentMarkedWord);
	}

	// 3. Copy the rest of the target tree nodes/parents at the end -> targetNodePosIndex + 1 + treeParams.mNbNodes
	if(mNbNodes - targetNodePosIndex)
	{
		PX_ASSERT(mNbNodes - targetNodePosIndex > 0);
		PxMemCopy(newRuntimePool + targetNodePosIndex + 1 + treeParams.mNbNodes, mNodes + targetNodePosIndex, sizeof(BVHNode)*(mNbNodes - targetNodePosIndex));
		PxMemCopy(newParentIndices + targetNodePosIndex + 1 + treeParams.mNbNodes, mParentIndices + targetNodePosIndex, sizeof(PxU32)*(mNbNodes - targetNodePosIndex));
	}
	// swap the pointers, release the old memory
	PX_DELETE_ARRAY(mNodes);
	mNodes = newRuntimePool;
	PX_FREE(mParentIndices);
	mParentIndices = newParentIndices;

	// 4. Copy the merge tree after the new node, create the parent map for them, update the leaf indices
	nodeIndex++;
	addRuntimeChilds(nodeIndex, treeParams);
	PX_ASSERT(nodeIndex == targetNodePosIndex + 1 + treeParams.mNbNodes);
	// update the total number of nodes
	mNbNodes = mNbNodes + 1 + treeParams.mNbNodes;	

	// update the parent information for the input tree root node
	mParentIndices[targetNodePosIndex + 1] = targetMergeNodeIndex;
	
	// 5. Go through the nodes copied at the end and fix the parents/childs
	for (PxU32 i = targetNodePosIndex + 1 + treeParams.mNbNodes; i < mNbNodes; i++)
	{
		// check if the parent is the targetNode, if yes update the parent to new node
		if(mParentIndices[i] == targetMergeNodeIndex)
		{
			mParentIndices[i] = targetNodePosIndex;
		}
		else
		{
			// if parent node has been moved, update the parent node
			if(mParentIndices[i] >= targetNodePosIndex)
			{
				mParentIndices[i] = mParentIndices[i] + 1 + treeParams.mNbNodes;
			}
			else
			{
				// if parent has not been moved, update its child information
				const PxU32 parentIndex = mParentIndices[i];
				// update the child information to point to Pos child
				if(i % 2 != 0)
				{
					const PxU32 srcNodeIndex = mNodes[parentIndex].getPosIndex();
					// if child index points to a node that has been moved, update the child index
					PX_ASSERT(!mNodes[parentIndex].isLeaf());
					PX_ASSERT(srcNodeIndex > targetNodePosIndex);
					mNodes[parentIndex].mData = (1 + treeParams.mNbNodes + srcNodeIndex) << 1;
				}
			}
		}
		if(!mNodes[i].isLeaf())
		{
			// update the child node index
			const PxU32 srcNodeIndex = 1 + treeParams.mNbNodes + mNodes[i].getPosIndex();
			mNodes[i].mData = srcNodeIndex << 1;
		}
	}
}

// traverse the target node, the tree is inside the targetNode, and find the best place where merge the tree
void AABBTree::traverseRuntimeNode(BVHNode& targetNode, const AABBTreeMergeData& treeParams, PxU32 nodeIndex)
{
	const BVHNode& srcNode = treeParams.getRootNode();
	PX_ASSERT(srcNode.mBV.isInside(targetNode.mBV));

	// Check if the srcNode(tree) can fit inside any of the target childs. If yes, traverse the target tree child
	BVHNode& targetPosChild = *targetNode.getPos(mNodes);	
	if(srcNode.mBV.isInside(targetPosChild.mBV))
	{
		return traverseRuntimeNode(targetPosChild, treeParams, targetNode.getPosIndex());		
	}

	BVHNode& targetNegChild = *targetNode.getNeg(mNodes);
	if (srcNode.mBV.isInside(targetNegChild.mBV))
	{
		return traverseRuntimeNode(targetNegChild, treeParams, targetNode.getNegIndex());		
	}

	// we cannot traverse target anymore, lets add the srcTree to current target node
	if(targetNode.isLeaf())
		mergeRuntimeLeaf(targetNode, treeParams, nodeIndex);
	else
		mergeRuntimeNode(targetNode, treeParams, nodeIndex);	
}

// Merge the input tree into current tree.
// Traverse the tree and find the smallest node, where the whole new tree fits. When we find the node 
// we create one new node pointing to the original children and the to the input tree root.
void AABBTree::mergeTree(const AABBTreeMergeData& treeParams)
{ 
	// allocate new indices buffer 	
	PxU32* newIndices = PX_ALLOCATE(PxU32, (mNbIndices + treeParams.mNbIndices), "AABB tree indices");
	PxMemCopy(newIndices, mIndices, sizeof(PxU32)*mNbIndices);
	PX_FREE(mIndices);
	mIndices = newIndices;
	mTotalPrims += treeParams.mNbIndices;

	// copy the new indices, re-index using the provided indicesOffset. Note that indicesOffset 
	// must be provided, as original mNbIndices can be different than indicesOffset dues to object releases.	
	for (PxU32 i = 0; i < treeParams.mNbIndices; i++)
	{
		mIndices[mNbIndices + i] = treeParams.mIndicesOffset + treeParams.mIndices[i];
	}	

	// check the mRefitBitmask if we fit all the new nodes
	mRefitBitmask.resize(mNbNodes + treeParams.mNbNodes + 1);	

	// create the parent information so we can update it
	getParentIndices();
	
	// if new tree is inside the root AABB we will traverse the tree to find better node where to attach the tree subnodes
	// if the root is a leaf we merge with the root. 		
	if(treeParams.getRootNode().mBV.isInside(mNodes[0].mBV) && !mNodes[0].isLeaf())
	{
		traverseRuntimeNode(mNodes[0], treeParams, 0);
	}
	else
	{				
		if(mNodes[0].isLeaf())
		{			
			mergeRuntimeLeaf(mNodes[0], treeParams, 0);
		}
		else		
		{			
			mergeRuntimeNode(mNodes[0], treeParams, 0);		
		}

		// increase the tree root AABB
		mNodes[0].mBV.include(treeParams.getRootNode().mBV);
	}

#ifdef _DEBUG
	//verify parent indices
	for (PxU32 i = 0; i < mNbNodes; i++)
	{
		if (i)
		{
			PX_ASSERT(mNodes[mParentIndices[i]].getPosIndex() == i || mNodes[mParentIndices[i]].getNegIndex() == i);
		}
		if (!mNodes[i].isLeaf())
		{
			PX_ASSERT(mParentIndices[mNodes[i].getPosIndex()] == i);
			PX_ASSERT(mParentIndices[mNodes[i].getNegIndex()] == i);
		}
	}

	// verify the tree nodes, leafs
	for (PxU32 i = 0; i < mNbNodes; i++)
	{
		if (mNodes[i].isLeaf())
		{
			const PxU32 index = mNodes[i].mData >> 5;
			const PxU32 nbPrim = mNodes[i].getNbPrimitives();
			PX_ASSERT(index + nbPrim <= mNbIndices + treeParams.mNbIndices);
		}
		else
		{
			const PxU32 nodeIndex = (mNodes[i].getPosIndex());
			PX_ASSERT(nodeIndex < mNbNodes);
		}
	}
#endif // _DEBUG

	mNbIndices += treeParams.mNbIndices;
}

