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

#ifndef EXT_DELAUNAY_TETRAHEDRALIZER_H
#define EXT_DELAUNAY_TETRAHEDRALIZER_H

#include "foundation/PxArray.h"
#include "foundation/PxVec3.h"
#include "foundation/PxHashSet.h"
#include "GuTetrahedron.h"
#include "ExtVec3.h"

namespace physx
{	

namespace Ext
{
	using Edge = PxPair<PxI32, PxI32>;
	using Tetrahedron = Gu::TetrahedronT<PxI32>;
	using Tetrahedron16 = Gu::TetrahedronT<PxI16>;

	void buildNeighborhood(const PxArray<Tetrahedron>& tets, PxArray<PxI32>& result);
	void buildNeighborhood(const PxI32* tets, PxU32 numTets, PxArray<PxI32>& result);

	PX_FORCE_INLINE PxF64 tetVolume(const PxVec3d& a, const PxVec3d& b, const PxVec3d& c, const PxVec3d& d)
	{
		return (-1.0 / 6.0) * (a - d).dot((b - d).cross(c - d));
	}

	PX_FORCE_INLINE PxF64 tetVolume(const Tetrahedron& tet, const PxArray<PxVec3d>& points)
	{
		return tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]);
	}

	//Returns the intersection (set operation) of two sorted lists
	PX_FORCE_INLINE void intersectionOfSortedLists(const PxArray<PxI32>& sorted1, const PxArray<PxI32>& sorted2, PxArray<PxI32>& result)
	{
		PxU32 a = 0;
		PxU32 b = 0;
		result.clear();
		while (a < sorted1.size() && b < sorted2.size())
		{
			if (sorted1[a] == sorted2[b])
			{
				result.pushBack(sorted1[a]);
				++a;
				++b;
			}
			else if (sorted1[a] > sorted2[b])
				++b;
			else
				++a;
		}
	}
	
	PX_FORCE_INLINE bool intersectionOfSortedListsContainsElements(const PxArray<PxI32>& sorted1, const PxArray<PxI32>& sorted2)
	{
		PxU32 a = 0;
		PxU32 b = 0;
		while (a < sorted1.size() && b < sorted2.size())
		{
			if (sorted1[a] == sorted2[b])			
				return true;			
			else if (sorted1[a] > sorted2[b])
				++b;
			else
				++a;
		}
		return false;
	}


	class BaseTetAnalyzer
	{
	public:
		virtual PxF64 quality(const PxArray<PxI32> tetIndices) const = 0;

		virtual PxF64 quality(const PxArray<Tetrahedron> tetrahedraToCheck) const = 0;

		virtual bool improved(PxF64 previousQuality, PxF64 newQuality) const = 0;

		virtual ~BaseTetAnalyzer() {}
	};

	class MinimizeMaxAmipsEnergy : public BaseTetAnalyzer
	{
		const PxArray<PxVec3d>& points;
		const PxArray<Tetrahedron>& tetrahedra;

	public:
		MinimizeMaxAmipsEnergy(const PxArray<PxVec3d>& points_, const PxArray<Tetrahedron>& tetrahedra_) : points(points_), tetrahedra(tetrahedra_)
		{}

		PxF64 quality(const PxArray<PxI32> tetIndices) const;

		PxF64 quality(const PxArray<Tetrahedron> tetrahedraToCheck) const;

		bool improved(PxF64 previousQuality, PxF64 newQuality) const;

		virtual ~MinimizeMaxAmipsEnergy() {}

	private:
		PX_NOCOPY(MinimizeMaxAmipsEnergy)
	};

	class MaximizeMinTetVolume : public BaseTetAnalyzer
	{
		const PxArray<PxVec3d>& points;
		const PxArray<Tetrahedron>& tetrahedra;

	public:
		MaximizeMinTetVolume(const PxArray<PxVec3d>& points_, const PxArray<Tetrahedron>& tetrahedra_) : points(points_), tetrahedra(tetrahedra_)
		{}

		PxF64 quality(const PxArray<PxI32> tetIndices) const;

		PxF64 quality(const PxArray<Tetrahedron> tetrahedraToCheck) const;

		bool improved(PxF64 previousQuality, PxF64 newQuality) const;

		virtual ~MaximizeMinTetVolume() {}

	private:
		PX_NOCOPY(MaximizeMinTetVolume)
	};



	//Helper class to extract surface triangles from a tetmesh
	struct SortedTriangle
	{
	public:
		PxI32 A;
		PxI32 B;
		PxI32 C;
		bool Flipped;

		PX_FORCE_INLINE SortedTriangle(PxI32 a, PxI32 b, PxI32 c)
		{
			A = a; B = b; C = c; Flipped = false;
			if (A > B) { PxSwap(A, B); Flipped = !Flipped; }
			if (B > C) { PxSwap(B, C); Flipped = !Flipped; }
			if (A > B) { PxSwap(A, B); Flipped = !Flipped; }
		}
	};

	struct TriangleHash
	{
		PX_FORCE_INLINE std::size_t operator()(const SortedTriangle& k) const
		{
			return k.A ^ k.B ^ k.C;
		}

		PX_FORCE_INLINE bool equal(const SortedTriangle& first, const SortedTriangle& second) const
		{
			return first.A == second.A && first.B == second.B && first.C == second.C;
		}
	};


	struct RecoverEdgeMemoryCache
	{
		PxArray<PxI32> facesStart;
		PxHashSet<PxI32> tetsDoneStart;
		PxArray<PxI32> resultStart;
		PxArray<PxI32> facesEnd;
		PxHashSet<PxI32> tetsDoneEnd;
		PxArray<PxI32> resultEnd;
	};

	class StackMemory
	{
	public:

		void clear()
		{
			faces.forceSize_Unsafe(0);
			hashSet.clear();
		}

		PxArray<PxI32> faces;
		PxHashSet<PxI32> hashSet;
	
	};

	//Incremental delaunay tetrahedralizer
	class DelaunayTetrahedralizer
	{
	public:
		//The bounds specified must contain all points that will get inserted by calling insertPoints.
		DelaunayTetrahedralizer(const PxVec3d& min, const PxVec3d& max);

		DelaunayTetrahedralizer(PxArray<PxVec3d>& points, PxArray<Tetrahedron>& tets);

		void initialize(PxArray<PxVec3d>& points, PxArray<Tetrahedron>& tets);

		//Inserts a bunch of new points into the tetrahedralization and keeps the delaunay condition satisfied. The new result will
		//get stored in the tetrahedra array. Points to insert must already be present in inPoints, the indices of the points to insert
		//can be controlled with start and end index (end index is exclusive, start index is inclusive)
		void insertPoints(const PxArray<PxVec3d>& inPoints, PxI32 start, PxI32 end, PxArray<Tetrahedron>& tetrahedra);
		
		bool insertPoints(const PxArray<PxVec3d>& inPoints, PxI32 start, PxI32 end);

		void exportTetrahedra(PxArray<Tetrahedron>& tetrahedra);

		bool canCollapseEdge(PxI32 edgeVertexToKeep, PxI32 edgeVertexToRemove, PxF64 volumeChangeThreshold = 0.1, BaseTetAnalyzer* tetAnalyzer = NULL);
		bool canCollapseEdge(PxI32 edgeVertexToKeep, PxI32 edgeVertexToRemove, const PxArray<PxI32>& tetsConnectedToA, const PxArray<PxI32>& tetsConnectedToB,
			PxF64& qualityAfterCollapse, PxF64 volumeChangeThreshold = 0.1, BaseTetAnalyzer* tetAnalyzer = NULL);

		void collapseEdge(PxI32 edgeVertexToKeep, PxI32 edgeVertexToRemove);

		void collapseEdge(PxI32 edgeVertexAToKeep, PxI32 edgeVertexBToRemove, const PxArray<PxI32>& tetsConnectedToA, const PxArray<PxI32>& tetsConnectedToB);

		void collectTetsConnectedToVertex(PxI32 vertexIndex, PxArray<PxI32>& tetIds);

		void collectTetsConnectedToVertex(PxArray<PxI32>& faces, PxHashSet<PxI32>& tetsDone, PxI32 vertexIndex, PxArray<PxI32>& tetIds);

		void collectTetsConnectedToEdge(PxI32 edgeStart, PxI32 edgeEnd, PxArray<PxI32>& tetIds);

		PX_FORCE_INLINE const PxVec3d& point(PxI32 index) const { return centeredNormalizedPoints[index]; }

		PX_FORCE_INLINE PxU32 numPoints() const { return centeredNormalizedPoints.size(); }

		PX_FORCE_INLINE PxArray<PxVec3d>& points() { return centeredNormalizedPoints; }
		PX_FORCE_INLINE const PxArray<PxVec3d>& points() const { return centeredNormalizedPoints; }

		PxU32 addPoint(const PxVec3d& p)
		{
			centeredNormalizedPoints.pushBack(p);
			vertexToTet.pushBack(-1);
			return centeredNormalizedPoints.size() - 1;
		}

		PX_FORCE_INLINE const Tetrahedron& tetrahedron(PxI32 index) const { return result[index]; }

		PX_FORCE_INLINE PxU32 numTetrahedra() const { return result.size(); }

		PX_FORCE_INLINE PxArray<Tetrahedron>& tetrahedra() { return result; }
		PX_FORCE_INLINE const PxArray<Tetrahedron>& tetrahedra() const { return result; }

		void copyInternalPointsTo(PxArray<PxVec3d>& points) { points = centeredNormalizedPoints; }

		bool optimizeByFlipping(PxArray<PxI32>& affectedFaces, const BaseTetAnalyzer& qualityAnalyzer);

		void insertPointIntoEdge(PxI32 newPointIndex, PxI32 edgeA, PxI32 edgeB, PxArray<PxI32>& affectedTets, BaseTetAnalyzer* qualityAnalyzer = NULL);

		bool removeEdgeByFlip(PxI32 edgeA, PxI32 edgeB, PxArray<PxI32>& tetIndices, BaseTetAnalyzer* qualityAnalyzer = NULL);

		void addLockedEdges(const PxArray<Gu::IndexedTriangleT<PxI32>>& triangles);

		void addLockedTriangles(const PxArray<Gu::IndexedTriangleT<PxI32>>& triangles);

		void clearLockedEdges() { lockedEdges.clear(); }

		void clearLockedTriangles() { lockedTriangles.clear(); }

		bool recoverEdgeByFlip(PxI32 eStart, PxI32 eEnd, RecoverEdgeMemoryCache& cache);

		void generateTetmeshEnforcingEdges(const PxArray<PxVec3d>& trianglePoints, const PxArray<Gu::IndexedTriangleT<PxI32>>& triangles, PxArray<PxArray<PxI32>>& allEdges,
			PxArray<PxArray<PxI32>>& pointToOriginalTriangle, 
			PxArray<PxVec3d>& points, PxArray<Tetrahedron>& finalTets);

	private:
		PxArray<PxVec3d> centeredNormalizedPoints;
		PxArray<PxI32> neighbors;
		PxArray<PxI32> unusedTets;
		PxArray<PxI32> vertexToTet;
		PxArray<Tetrahedron> result;
		PxI32 numAdditionalPointsAtBeginning = 4;

		PxHashSet<SortedTriangle, TriangleHash> lockedTriangles;
		PxHashSet<PxU64> lockedEdges;

		StackMemory		stackMemory;
		PX_NOCOPY(DelaunayTetrahedralizer)
	};

	struct EdgeWithLength
	{
		PxI32 A;
		PxI32 B;
		PxF64 Length;

		EdgeWithLength(PxI32 a_, PxI32 b_, PxF64 length_)
		{
			A = a_;
			B = b_;
			Length = length_;
		}		
	};

	PX_FORCE_INLINE bool operator <(const EdgeWithLength& lhs, const EdgeWithLength& rhs)
	{
		return lhs.Length < rhs.Length;
	}

	struct SplitEdge
	{
		PxI32 A;
		PxI32 B;
		PxF64 Q;
		PxF64 L;
		bool InteriorEdge;

		SplitEdge(PxI32 a, PxI32 b, PxF64 q, PxF64 l, bool interiorEdge)
		{
			A = a;
			B = b;
			Q = q;
			L = l;
			InteriorEdge = interiorEdge;
		}		
	};

	PX_FORCE_INLINE bool operator >(const SplitEdge& lhs, const SplitEdge& rhs)
	{
		if (lhs.Q == rhs.Q)
			return lhs.L > rhs.L;
		return lhs.Q > rhs.Q;
	}
	
	
	bool optimizeByCollapsing(DelaunayTetrahedralizer& del, const PxArray<EdgeWithLength>& edges,
		PxArray<PxArray<PxI32>>& pointToOriginalTriangle, PxI32 numFixPoints, BaseTetAnalyzer* qualityAnalyzer = NULL);

	bool optimizeBySwapping(DelaunayTetrahedralizer& del, const PxArray<EdgeWithLength>& edges,
		const PxArray<PxArray<PxI32>>& pointToOriginalTriangle, BaseTetAnalyzer* qualityAnalyzer);

	bool optimizeBySplitting(DelaunayTetrahedralizer& del, const PxArray<EdgeWithLength>& edges, const PxArray<PxArray<PxI32>>& pointToOriginalTriangle,
		PxI32 maxPointsToInsert = -1, bool sortByQuality = false, BaseTetAnalyzer* qualityAnalyzer = NULL, PxF64 qualityThreshold = 10);

	//Modified tetmesh quality improvement implementation of the method described in https://cs.nyu.edu/~yixinhu/tetwild.pdf Section 3.2 Mesh Improvement
	void optimize(DelaunayTetrahedralizer& del, PxArray<PxArray<PxI32>>& pointToOriginalTriangle, PxI32 numFixPoints,
		PxArray<PxVec3d>& optimizedPoints, PxArray<Tetrahedron>& optimizedTets, PxI32 numPasses = 10);
}
}

#endif

