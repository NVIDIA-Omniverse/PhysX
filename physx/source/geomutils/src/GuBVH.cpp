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
#include "foundation/PxFPU.h"
#include "foundation/PxPlane.h"
#include "geometry/PxGeometryInternal.h"
#include "GuBVH.h"
#include "GuAABBTreeQuery.h"
#include "GuAABBTreeNode.h"
#include "GuAABBTreeBuildStats.h"
#include "GuMeshFactory.h"
#include "GuQuery.h"
#include "CmSerialize.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

///////////////////////////////////////////////////////////////////////////////

// PT: these two functions moved from cooking

bool BVHData::build(PxU32 nbBounds, const void* boundsData, PxU32 boundsStride, float enlargement, PxU32 nbPrimsPerLeaf, BVHBuildStrategy bs)
{
	if(!nbBounds || !boundsData || boundsStride<sizeof(PxBounds3) || enlargement<0.0f || nbPrimsPerLeaf>=16)
		return false;

	mBounds.init(nbBounds);

	if(nbBounds)
	{
		const PxU8* sB = reinterpret_cast<const PxU8*>(boundsData);
		for(PxU32 i=0; i<nbBounds-1; i++)
		{
			inflateBounds<true>(mBounds.getBounds()[i], *reinterpret_cast<const PxBounds3*>(sB), enlargement);
			sB += boundsStride;
		}
		inflateBounds<false>(mBounds.getBounds()[nbBounds-1], *reinterpret_cast<const PxBounds3*>(sB), enlargement);
	}
	mNbIndices = nbBounds;

	// build the BVH
	BuildStats stats;
	NodeAllocator nodeAllocator;
	mIndices = buildAABBTree(AABBTreeBuildParams(nbPrimsPerLeaf, nbBounds, &mBounds, bs), nodeAllocator, stats);
	if(!mIndices)
		return false;

	// store the computed hierarchy
	mNbNodes = stats.getCount();
	mNodes = PX_ALLOCATE(BVHNode, mNbNodes, "AABB tree nodes");
	PX_ASSERT(mNbNodes==nodeAllocator.mTotalNbNodes);

	// store the results into BVHNode list
	if(nbPrimsPerLeaf==1)
	{
		// PT: with 1 prim/leaf we don't need the remap table anymore, we can just store the prim index in each tree node directly.
		flattenTree(nodeAllocator, mNodes, mIndices);
		PX_FREE(mIndices);
	}
	else
		flattenTree(nodeAllocator, mNodes);
	return true;
}

// A.B. move to load code
#define PX_BVH_STRUCTURE_VERSION 1

bool BVHData::save(PxOutputStream& stream, bool endian) const
{
	// write header
	if(!writeHeader('B', 'V', 'H', 'S', PX_BVH_STRUCTURE_VERSION, endian, stream))
		return false;

	// write mData members
	writeDword(mNbIndices, endian, stream);
	writeDword(mNbNodes, endian, stream);

	// write indices and bounds
	for(PxU32 i=0; i<mNbIndices; i++)
		writeDword(mIndices[i], endian, stream);

	const PxBounds3* bounds = mBounds.getBounds();
	for(PxU32 i=0; i<mNbIndices; i++)
	{
		writeFloatBuffer(&bounds[i].minimum.x, 3, endian, stream);
		writeFloatBuffer(&bounds[i].maximum.x, 3, endian, stream);
	}

	// write nodes
	for(PxU32 i=0; i<mNbNodes; i++)
	{
		writeDword(mNodes[i].mData, endian, stream);

		writeFloatBuffer(&mNodes[i].mBV.minimum.x, 3, endian, stream);
		writeFloatBuffer(&mNodes[i].mBV.maximum.x, 3, endian, stream);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////

// PT: temporary for Kit

BVH::BVH(const PxBVHInternalData& data) :
	PxBVH			(PxType(PxConcreteType::eBVH), PxBaseFlags(0)),
	mMeshFactory	(NULL)
{
	mData.mNbIndices	= data.mNbIndices;
	mData.mNbNodes		= data.mNbNodes;
	mData.mIndices		= data.mIndices;
	mData.mNodes		= reinterpret_cast<BVHNode*>(data.mNodes);
	mData.mBounds.setBounds(reinterpret_cast<PxBounds3*>(data.mBounds));
}

bool BVH::getInternalData(PxBVHInternalData& data, bool takeOwnership) const
{
	data.mNbIndices	= mData.mNbIndices;
	data.mNbNodes	= mData.mNbNodes;
	data.mNodeSize	= sizeof(BVHNode);
	data.mNodes		= mData.mNodes;
	data.mIndices	= mData.mIndices;
	data.mBounds	= const_cast<PxBounds3*>(mData.mBounds.getBounds());

	if(takeOwnership)
		const_cast<BVH*>(this)->mData.mBounds.takeOwnership();

	return true;
}

bool physx::PxGetBVHInternalData(PxBVHInternalData& data, const PxBVH& bvh, bool takeOwnership)
{
	return static_cast<const BVH&>(bvh).getInternalData(data, takeOwnership);
}

//~ PT: temporary for Kit

BVH::BVH(MeshFactory* factory) :
	PxBVH			(PxType(PxConcreteType::eBVH), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMeshFactory	(factory)
{
}

BVH::BVH(MeshFactory* factory, BVHData& bvhData) :
	PxBVH			(PxType(PxConcreteType::eBVH), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	mMeshFactory	(factory),
	mData			(bvhData)
{
}

BVH::~BVH()
{
}

bool BVH::init(PxU32 nbPrims, AABBTreeBounds* bounds, const void* boundsData, PxU32 stride, BVHBuildStrategy bs, PxU32 nbPrimsPerLeaf, float enlargement)
{
	if(!nbPrims)
		return false;

	if(bounds)
	{
		mData.mBounds.moveFrom(*bounds);
	}
	else
	{
		mData.mBounds.init(nbPrims);
		PxBounds3* dst = mData.mBounds.getBounds();
		if(stride==sizeof(PxBounds3))
		{
			PxMemCopy(dst, boundsData, sizeof(PxBounds3)*nbPrims);
		}
		else
		{
			if(nbPrims)
			{
				const PxU8* sB = reinterpret_cast<const PxU8*>(boundsData);
				for(PxU32 i=0; i<nbPrims-1; i++)
				{
					inflateBounds<true>(mData.mBounds.getBounds()[i], *reinterpret_cast<const PxBounds3*>(sB), enlargement);
					sB += stride;
				}
				inflateBounds<false>(mData.mBounds.getBounds()[nbPrims-1], *reinterpret_cast<const PxBounds3*>(sB), enlargement);
			}
		}
	}
	mData.mNbIndices = nbPrims;

	// build the BVH
	BuildStats stats;
	NodeAllocator nodeAllocator;
	mData.mIndices = buildAABBTree(AABBTreeBuildParams(nbPrimsPerLeaf, nbPrims, &mData.mBounds, bs), nodeAllocator, stats);
	if(!mData.mIndices)
		return false;

	// store the computed hierarchy
	mData.mNbNodes = stats.getCount();
	mData.mNodes = PX_ALLOCATE(BVHNode, mData.mNbNodes, "AABB tree nodes");
	PX_ASSERT(mData.mNbNodes==nodeAllocator.mTotalNbNodes);

	// store the results into BVHNode list
	if(nbPrimsPerLeaf==1)
	{
		// PT: with 1 prim/leaf we don't need the remap table anymore, we can just store the prim index in each tree node directly.
		flattenTree(nodeAllocator, mData.mNodes, mData.mIndices);
		PX_FREE(mData.mIndices);
	}
	else
		flattenTree(nodeAllocator, mData.mNodes);
	return true;
}

bool BVH::load(PxInputStream& stream)
{
	// Import header
	PxU32 version;
	bool mismatch;
	if(!readHeader('B', 'V', 'H', 'S', version, mismatch, stream))
		return false;

	// read numVolumes, numNodes together 
	//ReadDwordBuffer(&mData.mNbIndices, 2, mismatch, stream);	
	mData.mNbIndices = readDword(mismatch, stream);
	mData.mNbNodes = readDword(mismatch, stream);

	// read indices
	mData.mIndices = PX_ALLOCATE(PxU32, mData.mNbIndices, "BVH indices");
	ReadDwordBuffer(mData.mIndices, mData.mNbIndices, mismatch, stream);

	// read bounds
	mData.mBounds.init(mData.mNbIndices);
	readFloatBuffer(&mData.mBounds.getBounds()->minimum.x, mData.mNbIndices*(3 + 3), mismatch, stream);

	// read nodes
	mData.mNodes = PX_ALLOCATE(BVHNode, mData.mNbNodes, "BVH nodes");
	for(PxU32 i = 0; i < mData.mNbNodes; i++)
	{
		ReadDwordBuffer(&mData.mNodes[i].mData, 1, mismatch, stream);

		readFloatBuffer(&mData.mNodes[i].mBV.minimum.x, 3 + 3, mismatch, stream);		
	}
	return true;
}

void BVH::release()
{
	decRefCount();
}

void BVH::onRefCountZero()
{
	::onRefCountZero(this, mMeshFactory, false, "PxBVH::release: double deletion detected!");
}

namespace
{
	struct BVHTree
	{
		PX_FORCE_INLINE	BVHTree(const BVHData& data) : mRootNode(data.mNodes), mIndices(data.mIndices)	{}

		const BVHNode*	getNodes()		const { return mRootNode;	}
		const PxU32*	getIndices()	const { return mIndices;	}

		const BVHNode*	mRootNode;
		const PxU32*	mIndices;
	};
}

namespace
{
	struct RaycastAdapter
	{
		RaycastAdapter(PxBVH::RaycastCallback& cb) : mCallback(cb), mAbort(false)	{}
		PX_FORCE_INLINE bool invoke(PxReal& distance, PxU32 index)
		{
			if(mAbort || !mCallback.reportHit(index, distance))
			{
				mAbort = true;
				return false;
			}
			return true;
		}
		PxBVH::RaycastCallback& mCallback;
		bool					mAbort;
		PX_NOCOPY(RaycastAdapter)
	};
}

bool BVH::raycast(const PxVec3& origin, const PxVec3& unitDir, float distance, RaycastCallback& cb, PxGeometryQueryFlags flags) const
{
	PX_SIMD_GUARD_CNDT(flags & PxGeometryQueryFlag::eSIMD_GUARD)
	RaycastAdapter ra(cb);
	if(mData.mIndices)
		return AABBTreeRaycast<false, true, BVHTree, BVHNode, RaycastAdapter>()(mData.mBounds, BVHTree(mData), origin, unitDir, distance, PxVec3(0.0f), ra);
	else
		return AABBTreeRaycast<false, false, BVHTree, BVHNode, RaycastAdapter>()(mData.mBounds, BVHTree(mData), origin, unitDir, distance, PxVec3(0.0f), ra);
}

namespace
{
	struct OverlapAdapter
	{
		OverlapAdapter(PxBVH::OverlapCallback& cb) : mCallback(cb), mAbort(false)	{}
		PX_FORCE_INLINE bool invoke(PxU32 index)
		{
			if(mAbort || !mCallback.reportHit(index))
			{
				mAbort = true;
				return false;
			}
			return true;
		}
		PxBVH::OverlapCallback& mCallback;
		bool					mAbort;
		PX_NOCOPY(OverlapAdapter)
	};
}

bool BVH::overlap(const ShapeData& queryVolume, OverlapCallback& cb, PxGeometryQueryFlags flags) const
{
	PX_SIMD_GUARD_CNDT(flags & PxGeometryQueryFlag::eSIMD_GUARD)

	OverlapAdapter oa(cb);

	switch(queryVolume.getType())
	{
		case PxGeometryType::eBOX:
		{
			if(queryVolume.isOBB())
			{	
				const DefaultOBBAABBTest test(queryVolume);
				if(mData.mIndices)
					return AABBTreeOverlap<true, OBBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
				else
					return AABBTreeOverlap<false, OBBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
			}
			else
			{
				const DefaultAABBAABBTest test(queryVolume);
				if(mData.mIndices)
					return AABBTreeOverlap<true, AABBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
				else
					return AABBTreeOverlap<false, AABBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
			}
		}
		case PxGeometryType::eCAPSULE:
		{
			const DefaultCapsuleAABBTest test(queryVolume, 1.0f);
			if(mData.mIndices)
				return AABBTreeOverlap<true, CapsuleAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
			else
				return AABBTreeOverlap<false, CapsuleAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
		}
		case PxGeometryType::eSPHERE:
		{
			const DefaultSphereAABBTest test(queryVolume);
			if(mData.mIndices)
				return AABBTreeOverlap<true, SphereAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
			else
				return AABBTreeOverlap<false, SphereAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
		}
		case PxGeometryType::eCONVEXMESH:
		{
			const DefaultOBBAABBTest test(queryVolume);
			if(mData.mIndices)
				return AABBTreeOverlap<true, OBBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
			else
				return AABBTreeOverlap<false, OBBAABBTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
		}
	default:
		PX_ALWAYS_ASSERT_MESSAGE("unsupported overlap query volume geometry type");
	}
	return false;
}

bool BVH::overlap(const PxGeometry& geom, const PxTransform& pose, OverlapCallback& cb, PxGeometryQueryFlags flags) const
{
	const ShapeData queryVolume(geom, pose, 0.0f);
	return overlap(queryVolume, cb, flags);
}

bool BVH::sweep(const ShapeData& queryVolume, const PxVec3& unitDir, float distance, RaycastCallback& cb, PxGeometryQueryFlags flags) const
{
	PX_SIMD_GUARD_CNDT(flags & PxGeometryQueryFlag::eSIMD_GUARD)

	const PxBounds3& aabb = queryVolume.getPrunerInflatedWorldAABB();
	RaycastAdapter ra(cb);
	if(mData.mIndices)
		return AABBTreeRaycast<true, true, BVHTree, BVHNode, RaycastAdapter>()(mData.mBounds, BVHTree(mData), aabb.getCenter(), unitDir, distance, aabb.getExtents(), ra);
	else
		return AABBTreeRaycast<true, false, BVHTree, BVHNode, RaycastAdapter>()(mData.mBounds, BVHTree(mData), aabb.getCenter(), unitDir, distance, aabb.getExtents(), ra);
}

bool BVH::sweep(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float distance, RaycastCallback& cb, PxGeometryQueryFlags flags) const
{
	const ShapeData queryVolume(geom, pose, 0.0f);
	return sweep(queryVolume, unitDir, distance, cb, flags);
}

namespace
{
	PX_FORCE_INLINE bool planesAABBOverlap(const PxVec3& m, const PxVec3& d, const PxPlane* p, PxU32& outClipMask, PxU32 inClipMask)
	{
		PxU32 mask = 1;
		PxU32 tmpOutClipMask = 0;

		while(mask<=inClipMask)
		{
			if(inClipMask & mask)
			{               
				const float NP = d.x*fabsf(p->n.x) + d.y*fabsf(p->n.y) + d.z*fabsf(p->n.z);
				const float MP = m.x*p->n.x + m.y*p->n.y + m.z*p->n.z + p->d;

				if(NP < MP)
					return false;
				if((-NP) < MP)
					tmpOutClipMask |= mask;
			}
			mask+=mask;
			p++;
		}

		outClipMask = tmpOutClipMask;
		return true;
	}

	struct FrustumTest
	{
		FrustumTest(PxU32 nbPlanes, const PxPlane* planes) : mPlanes(planes), mMask((1<<nbPlanes)-1), mNbPlanes(nbPlanes), mOutClipMask(0)
		{
		}

		PX_FORCE_INLINE PxIntBool operator()(const Vec3V boxCenter, const Vec3V boxExtents) const		
		{
			// PT: TODO: rewrite all this in SIMD
			PxVec3 center, extents;
			V3StoreU(boxCenter, center);
			V3StoreU(boxExtents, extents);

			if(!planesAABBOverlap(center, extents, mPlanes, mOutClipMask, mMask))
				return PxIntFalse;

			// PT: unfortunately the AABBTreeOverlap template doesn't support this case where we know we can
			// immediately dump the rest of the tree (i.e. the old "containment tests" in Opcode). We might
			// want to revisit this at some point.
			//
			// In fact it's worse than this: we lost the necessary data to make this quick, in "flattenTree"
			// when going from AABBTreeBuildNodes to BVHNodes. The BVHNodes lost the primitive-related info
			// for internal (non-leaf) nodes so we cannot just dump a list of primitives when an internal
			// node is fully visible (like we did in Opcode 1.x). Best we can do is keep traversing the tree
			// and skip VFC tests.
			//if(!outClipMask)
			
			return PxIntTrue;
		}

		const PxPlane*	mPlanes;
		const PxU32		mMask;
		const PxU32		mNbPlanes;
		mutable PxU32	mOutClipMask;

		PX_NOCOPY(FrustumTest)
	};
}

static bool dumpNode(OverlapAdapter& oa, const BVHNode* const nodeBase, const BVHNode* node0, const PxU32* indices)
{
	PxInlineArray<const BVHNode*, RAW_TRAVERSAL_STACK_SIZE> stack;
	stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
	stack[0] = node0;
	PxU32 stackIndex = 1;

	while(stackIndex > 0)
	{
		const BVHNode* node = stack[--stackIndex];

		while(1)
		{
			if(node->isLeaf())
			{
				PxU32 nbPrims = node->getNbPrimitives();
				const PxU32* prims = indices ? node->getPrimitives(indices) : NULL;
				while(nbPrims--)
				{
					const PxU32 primIndex = indices ? *prims++ : node->getPrimitiveIndex();
					if(!oa.invoke(primIndex))
						return false;
				}
				break;
			}
			else
			{
				const BVHNode* children = node->getPos(nodeBase);
				node = children;
				stack[stackIndex++] = children + 1;
				if(stackIndex == stack.capacity())
					stack.resizeUninitialized(stack.capacity() * 2);
			}
		}
	}
	return true;
}

bool BVH::cull(PxU32 nbPlanes, const PxPlane* planes, OverlapCallback& cb, PxGeometryQueryFlags flags) const
{
	PX_SIMD_GUARD_CNDT(flags & PxGeometryQueryFlag::eSIMD_GUARD)

	OverlapAdapter oa(cb);
	const FrustumTest test(nbPlanes, planes);

	if(0)
	{
		// PT: this vanilla codepath is slower
		if(mData.mIndices)
			return AABBTreeOverlap<true, FrustumTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
		else
			return AABBTreeOverlap<false, FrustumTest, BVHTree, BVHNode, OverlapAdapter>()(mData.mBounds, BVHTree(mData), test, oa);
	}
	else
	{
		const PxBounds3* bounds = mData.mBounds.getBounds();
		const bool hasIndices = mData.mIndices!=NULL;

		PxInlineArray<const BVHNode*, RAW_TRAVERSAL_STACK_SIZE> stack;
		stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
		const BVHNode* const nodeBase = mData.mNodes;
		stack[0] = nodeBase;
		PxU32 stackIndex = 1;

		while(stackIndex > 0)
		{
			const BVHNode* node = stack[--stackIndex];
			Vec3V center, extents;
			node->getAABBCenterExtentsV(&center, &extents);
			while(test(center, extents))
			{
				if(!test.mOutClipMask)
				{
					if(!dumpNode(oa, nodeBase, node, mData.mIndices))
						return false;
					break;
				}
				else
				{
					if(node->isLeaf())
					{
						PxU32 nbPrims = node->getNbPrimitives();
						const bool doBoxTest = nbPrims > 1;
						const PxU32* prims = hasIndices ? node->getPrimitives(mData.mIndices) : NULL;
						while(nbPrims--)
						{
							const PxU32 primIndex = hasIndices ? *prims++ : node->getPrimitiveIndex();
							if(doBoxTest)
							{
								Vec4V center2, extents2;
								getBoundsTimesTwo(center2, extents2, bounds, primIndex);

								const float half = 0.5f;
								const FloatV halfV = FLoad(half);

								const Vec4V extents_ = V4Scale(extents2, halfV);
								const Vec4V center_ = V4Scale(center2, halfV);

								if(!test(Vec3V_From_Vec4V(center_), Vec3V_From_Vec4V(extents_)))
									continue;
							}

							if(!oa.invoke(primIndex))
								return false;
						}
						break;
					}

					const BVHNode* children = node->getPos(nodeBase);

					node = children;
					stack[stackIndex++] = children + 1;
					if(stackIndex == stack.capacity())
						stack.resizeUninitialized(stack.capacity() * 2);
					node->getAABBCenterExtentsV(&center, &extents);
				}
			}
		}
		return true;
	}
}

void BVH::refit()
{
	mData.fullRefit(mData.mBounds.getBounds());
}

bool BVH::updateBoundsInternal(PxU32 localIndex, const PxBounds3& newBounds)
{
	if(localIndex>=mData.mNbIndices)
		return false;

	PxBounds3* bounds = mData.mBounds.getBounds();

	bounds[localIndex] = newBounds;

	// Lazy-create update map
	if(!mData.getUpdateMap())
		mData.createUpdateMap(mData.mNbIndices);

	PxU32* mMapping = mData.getUpdateMap();
	if(mMapping)
	{
		const PxU32 treeNodeIndex = mMapping[localIndex];
		if(treeNodeIndex!=0xffffffff)
		{
			mData.markNodeForRefit(treeNodeIndex);
			return true;
		}
	}
	return false;
}

bool BVH::updateBounds(PxU32 boundsIndex, const PxBounds3& newBounds)
{
	return updateBoundsInternal(boundsIndex, newBounds);
}

void BVH::partialRefit()
{
	mData.refitMarkedNodes(mData.mBounds.getBounds());
}

bool BVH::traverse(TraversalCallback& cb) const
{
	// PT: copy-pasted from AABBTreeOverlap and modified

	PxInlineArray<const BVHNode*, RAW_TRAVERSAL_STACK_SIZE> stack;
	stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
	const BVHNode* const nodeBase = mData.getNodes();
	stack[0] = nodeBase;
	PxU32 stackIndex = 1;

	while(stackIndex > 0)
	{
		const BVHNode* node = stack[--stackIndex];

		while(cb.visitNode(node->mBV))
		{
			if(node->isLeaf())
			{
				if(mData.getIndices())
				{
					if(!cb.reportLeaf(node->getNbPrimitives(), node->getPrimitives(mData.getIndices())))
						return false;
				}
				else
				{
					PX_ASSERT(node->getNbPrimitives()==1);
					const PxU32 primIndex = node->getPrimitiveIndex();
					if(!cb.reportLeaf(node->getNbPrimitives(), &primIndex))
						return false;
				}
				break;
			}

			const BVHNode* children = node->getPos(nodeBase);
			node = children;
			stack[stackIndex++] = children + 1;
			if(stackIndex == stack.capacity())
				stack.resizeUninitialized(stack.capacity() * 2);
		}
	}
	return true;
}




#include "geometry/PxMeshQuery.h"

#define GU_BVH_STACK_SIZE	1024	// Default size of local stacks for non-recursive traversals.

static bool doLeafVsLeaf(PxReportCallback<PxGeomIndexPair>& callback,	const BVHNode* node0, const PxBounds3* bounds0, const PxU32* indices0,
																		const BVHNode* node1, const PxBounds3* bounds1, const PxU32* indices1,
																		bool& abort)
{
	PxGeomIndexPair* dst = callback.mBuffer;
	PxU32 capacity = callback.mCapacity;
	PxU32 currentSize = callback.mSize;
	PX_ASSERT(currentSize<capacity);

	bool foundHit = false;
	abort = false;

	const FloatV halfV = FLoad(0.5f);

	PxU32 nbPrims0 = node0->getNbPrimitives();
	const PxU32* prims0 = indices0 ? node0->getPrimitives(indices0) : NULL;
	while(nbPrims0--)
	{
		const PxU32 primIndex0 = prims0 ? *prims0++ : node0->getPrimitiveIndex();

		Vec3V center0, extents0;
		{
			Vec4V center2, extents2;
			getBoundsTimesTwo(center2, extents2, bounds0, primIndex0);

			extents0 = Vec3V_From_Vec4V(V4Scale(extents2, halfV));
			center0 = Vec3V_From_Vec4V(V4Scale(center2, halfV));
		}

		PxU32 nbPrims1 = node1->getNbPrimitives();
		const PxU32* prims1 = indices1 ? node1->getPrimitives(indices1) : NULL;
		while(nbPrims1--)
		{
			const PxU32 primIndex1 = prims1 ? *prims1++ : node1->getPrimitiveIndex();

			Vec3V center1, extents1;
			{
				Vec4V center2, extents2;
				getBoundsTimesTwo(center2, extents2, bounds1, primIndex1);

				extents1 = Vec3V_From_Vec4V(V4Scale(extents2, halfV));
				center1 = Vec3V_From_Vec4V(V4Scale(center2, halfV));
			}

			if(PxIntBool(V3AllGrtrOrEq(V3Add(extents0, extents1), V3Abs(V3Sub(center1, center0)))))
			{
				foundHit = true;

				// PT: TODO: refactor callback management code with BVH34
				dst[currentSize].id0 = primIndex0;
				dst[currentSize].id1 = primIndex1;
				currentSize++;
				if(currentSize==capacity)
				{
					callback.mSize = 0;
					if(!callback.flushResults(currentSize, dst))
					{
						abort = true;
						return foundHit;
					}
					dst = callback.mBuffer;
					capacity = callback.mCapacity;
					currentSize = callback.mSize;
				}

			}
		}
	}
	callback.mSize = currentSize;
	return foundHit;
}

static PX_FORCE_INLINE void pushChildren(PxGeomIndexPair* stack, PxU32& nb, PxU32 a, PxU32 b, PxU32 c, PxU32 d)
{
	stack[nb].id0 = a;
	stack[nb].id1 = b;
	nb++;

	stack[nb].id0 = c;
	stack[nb].id1 = d;
	nb++;
}

static PX_NOINLINE bool abortQuery(PxReportCallback<PxGeomIndexPair>& callback, bool& abort)
{
	abort = true;
	callback.mSize = 0;
	return true;
}

static bool BVH_BVH(PxReportCallback<PxGeomIndexPair>& callback, const BVH& tree0, const BVH& tree1, bool& _abort)
{
	const BVHNode* PX_RESTRICT node0 = tree0.getNodes();
	const BVHNode* PX_RESTRICT node1 = tree1.getNodes();
	PX_ASSERT(node0 && node1);

	const PxBounds3* bounds0 = tree0.getData().mBounds.getBounds();
	const PxBounds3* bounds1 = tree1.getData().mBounds.getBounds();

	const PxU32* indices0 = tree0.getIndices();
	const PxU32* indices1 = tree1.getIndices();

	{
		PxU32 nb=1;
		PxGeomIndexPair stack[GU_BVH_STACK_SIZE];
		stack[0].id0 = 0;
		stack[0].id1 = 0;

		bool status = false;

		const BVHNode* const root0 = node0;
		const BVHNode* const root1 = node1;

		do
		{
			const PxGeomIndexPair& childData = stack[--nb];
			node0 = root0 + childData.id0;
			node1 = root1 + childData.id1;

			if(node0->mBV.intersects(node1->mBV))
			{
				const PxU32 isLeaf0 = node0->isLeaf();
				const PxU32 isLeaf1 = node1->isLeaf();

				if(isLeaf0)
				{
					if(isLeaf1)
					{
						bool abort;
						if(doLeafVsLeaf(callback, node0, bounds0, indices0, node1, bounds1, indices1, abort))
							status = true;
						if(abort)
							return abortQuery(callback, _abort);
					}
					else
					{
						const PxU32 posIndex1 = node1->getPosIndex();
						pushChildren(stack, nb, childData.id0, posIndex1, childData.id0, posIndex1 + 1);
					}
				}
				else if(isLeaf1)
				{
					const PxU32 posIndex0 = node0->getPosIndex();
					pushChildren(stack, nb, posIndex0, childData.id1, posIndex0 + 1, childData.id1);
				}
				else
				{
					const PxU32 posIndex0 = node0->getPosIndex();
					const PxU32 posIndex1 = node1->getPosIndex();
					pushChildren(stack, nb, posIndex0, posIndex1, posIndex0, posIndex1 + 1);
					pushChildren(stack, nb, posIndex0 + 1, posIndex1, posIndex0 + 1, posIndex1 + 1);
				}
			}
		}while(nb);

		return status;
	}
}

bool physx::PxFindOverlap(PxReportCallback<PxGeomIndexPair>& callback, const PxBVH& bvh0, const PxBVH& bvh1)
{
	PX_SIMD_GUARD

	// PT: TODO: refactor callback management code with BVH34

	PxGeomIndexPair stackBuffer[256];
	bool mustResetBuffer;
	if(callback.mBuffer)
	{
		PX_ASSERT(callback.mCapacity);
		mustResetBuffer = false;
	}
	else
	{
		callback.mBuffer = stackBuffer;
		PX_ASSERT(callback.mCapacity<=256);
		if(callback.mCapacity==0 || callback.mCapacity>256)
		{
			callback.mCapacity = 256;
		}
		callback.mSize = 0;
		mustResetBuffer = true;
	}

	bool abort = false;
	const bool status = BVH_BVH(callback, static_cast<const BVH&>(bvh0), static_cast<const BVH&>(bvh1), abort);
	if(!abort)
	{
		const PxU32 currentSize = callback.mSize;
		if(currentSize)
		{
			callback.mSize = 0;
			callback.flushResults(currentSize, callback.mBuffer);
		}
	}

	if(mustResetBuffer)
		callback.mBuffer = NULL;
	return status;
}


