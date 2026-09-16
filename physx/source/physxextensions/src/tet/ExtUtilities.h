// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_TET_CPU_BVH_H
#define EXT_TET_CPU_BVH_H

#include "foundation/PxArray.h"

#include "foundation/PxBounds3.h"

#include "GuBVH.h"
#include "GuAABBTree.h"
#include "GuAABBTreeNode.h"
#include "GuAABBTreeBounds.h"
#include "GuAABBTreeQuery.h"
#include "GuTriangle.h"
#include "ExtVec3.h"

namespace physx
{
namespace Ext
{
	using Triangle = Gu::IndexedTriangleT<PxI32>;

	//Creates an unique 64bit bit key out of two 32bit values, the key is order independent, useful as hash key for edges
	//Use this functions to compute the edge keys used in the edgesToSplit parameter of the split function below.
	PX_FORCE_INLINE PxU64 key(PxI32 a, PxI32 b)
	{
		if (a < b)
			return ((PxU64(a)) << 32) | (PxU64(b));
		else
			return ((PxU64(b)) << 32) | (PxU64(a));
	}

	void buildTree(const PxU32* triangles, const PxU32 numTriangles, const PxVec3d* points, PxArray<Gu::BVHNode>& tree, PxF32 enlargement = 1e-4f);
	
	//Builds a BVH from a set of triangles
	PX_FORCE_INLINE void buildTree(const PxArray<Triangle>& triangles, const PxArray<PxVec3d>& points, PxArray<Gu::BVHNode>& tree, PxF32 enlargement = 1e-4f)
	{
		buildTree(reinterpret_cast<const PxU32*>(triangles.begin()), triangles.size(), points.begin(), tree, enlargement);
	}

	template<typename T>
	void traverseBVH(const PxArray<Gu::BVHNode>& nodes, T& traversalController, PxI32 rootNodeIndex = 0)
	{
		traverseBVH(nodes.begin(), traversalController, rootNodeIndex);
	}

	class IntersectionCollectingTraversalController
	{
		PxBounds3 box;
		PxArray<PxI32>& candidateTriangleIndices;

	public:
		IntersectionCollectingTraversalController(PxArray<PxI32>& candidateTriangleIndices_) :
			box(PxBounds3::empty()), candidateTriangleIndices(candidateTriangleIndices_)
		{ }

		IntersectionCollectingTraversalController(const PxBounds3& box_, PxArray<PxI32>& candidateTriangleIndices_) : 
			box(box_), candidateTriangleIndices(candidateTriangleIndices_)
		{ }

		void reset(const PxBounds3& box_)
		{
			box = box_;
			candidateTriangleIndices.clear();
		}

		Gu::TraversalControl::Enum analyze(const Gu::BVHNode& node, PxI32)
		{
			if (node.isLeaf())
			{
				candidateTriangleIndices.pushBack(node.getPrimitiveIndex());
				return Gu::TraversalControl::eDontGoDeeper;
			}

			if (node.mBV.intersects(box))
				return Gu::TraversalControl::eGoDeeper;
			return Gu::TraversalControl::eDontGoDeeper;
		}
	private:
		PX_NOCOPY(IntersectionCollectingTraversalController)
	};
}
}
#endif
