// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_CONVEX_HULL_UTILS_H
#define GU_COOKING_CONVEX_HULL_UTILS_H

#include "foundation/PxMemory.h"
#include "foundation/PxPlane.h"
#include "cooking/PxConvexMeshDesc.h"

#include "foundation/PxUserAllocated.h"
#include "foundation/PxArray.h"

namespace physx
{

	//////////////////////////////////////////////////////////////////////////
	// helper class for hull construction, holds the vertices and planes together
	// while cropping the hull with planes
	class ConvexHull : public PxUserAllocated
	{
	public:

		// Helper class for halfedge representation
		class HalfEdge
		{
		public:
			PxI16 ea;         // the other half of the edge (index into edges list)
			PxU8 v;  // the vertex at the start of this edge (index into vertices list)
			PxU8 p;  // the facet on which this edge lies (index into facets list)
			HalfEdge(){}
			HalfEdge(PxI16 _ea, PxU8 _v, PxU8 _p) :ea(_ea), v(_v), p(_p){}
		};

		ConvexHull& operator = (const ConvexHull&);

		// construct the base cube hull from given max/min AABB
		ConvexHull(const PxVec3& bmin, const PxVec3& bmax, const PxArray<PxPlane>& inPlanes);

		// construct the base cube hull from given OBB
		ConvexHull(const PxVec3& extent, const PxTransform& transform, const PxArray<PxPlane>& inPlanes);

		// copy constructor
		ConvexHull(const ConvexHull& srcHull)
			: mInputPlanes(srcHull.getInputPlanes())
		{
			copyHull(srcHull);
		}

		// construct plain hull
		ConvexHull(const PxArray<PxPlane>& inPlanes)
			: mInputPlanes(inPlanes)
		{
		}

		// finds the candidate plane, returns -1 otherwise
		PxI32 findCandidatePlane(float planetestepsilon, float epsilon) const;

		// internal check of the hull integrity
		bool assertIntact(float epsilon) const;

		// return vertices
		const PxArray<PxVec3>& getVertices() const
		{
			return mVertices;
		}

		// return edges
		const PxArray<HalfEdge>& getEdges() const
		{
			return mEdges;
		}

		// return faces
		const PxArray<PxPlane>& getFacets() const
		{
			return mFacets;
		}

		// return input planes
		const PxArray<PxPlane>& getInputPlanes() const
		{
			return mInputPlanes;
		}

		// return vertices
		PxArray<PxVec3>& getVertices()
		{
			return mVertices;
		}

		// return edges
		PxArray<HalfEdge>& getEdges()
		{
			return mEdges;
		}

		// return faces
		PxArray<PxPlane>& getFacets()
		{
			return mFacets;
		}

		// returns the maximum number of vertices on a face
		PxU32 maxNumVertsPerFace() const;

		// copy the hull from source
		void copyHull(const ConvexHull& src)
		{
			mVertices.resize(src.getVertices().size());
			mEdges.resize(src.getEdges().size());
			mFacets.resize(src.getFacets().size());

			PxMemCopy(mVertices.begin(), src.getVertices().begin(), src.getVertices().size()*sizeof(PxVec3));
			PxMemCopy(mEdges.begin(), src.getEdges().begin(), src.getEdges().size()*sizeof(HalfEdge));
			PxMemCopy(mFacets.begin(), src.getFacets().begin(), src.getFacets().size()*sizeof(PxPlane));
		}

	private:
		PxArray<PxVec3>	mVertices;
		PxArray<HalfEdge> mEdges;
		PxArray<PxPlane>  mFacets;
		const PxArray<PxPlane>&	mInputPlanes;
	};

	//////////////////////////////////////////////////////////////////////////|
	// Crops the hull with a provided plane and with given epsilon
	// returns new hull if succeeded
	ConvexHull* convexHullCrop(const ConvexHull& convex, const PxPlane& slice, float planetestepsilon);

	//////////////////////////////////////////////////////////////////////////|
	// three planes intersection
	PX_FORCE_INLINE PxVec3 threePlaneIntersection(const PxPlane& p0, const PxPlane& p1, const PxPlane& p2)
	{
		PxMat33 mp = (PxMat33(p0.n, p1.n, p2.n)).getTranspose();
		PxMat33 mi = (mp).getInverse();
		PxVec3 b(p0.d, p1.d, p2.d);
		return -mi.transform(b);
	}

	//////////////////////////////////////////////////////////////////////////
	// Compute OBB around given convex hull
	bool computeOBBFromConvex(const PxConvexMeshDesc& desc, PxVec3& sides, PxTransform& matrix);
}

#endif
