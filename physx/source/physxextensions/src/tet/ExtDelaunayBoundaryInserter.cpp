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

#include "ExtDelaunayBoundaryInserter.h"
#include "ExtTetSplitting.h"
#include "ExtFastWindingNumber.h"
#include "ExtTetUnionFind.h"

#include "ExtVec3.h"
#include "foundation/PxSort.h"
#include "foundation/PxBasicTemplates.h"
#include "CmRandom.h"

#include "tet/ExtUtilities.h"
#include "GuAABBTree.h"
#include "GuAABBTreeQuery.h"
#include "GuIntersectionTriangleTriangle.h"
#include "GuIntersectionTetrahedronBox.h"
#include "foundation/PxMathUtils.h"
#include "GuInternal.h"
#include "common/GuMeshAnalysis.h"

#include <stdio.h>

#define PI 3.141592653589793238462643383

namespace physx
{
namespace Ext
{
	using namespace Gu;

	template<typename T>
	bool contains(const PxArray<T>& list, const T& value)
	{
		for (PxU32 i = 0; i < list.size(); ++i)
			if (list[i] == value)
				return true;
		return false;
	}

	//Returns a value proportional to the signed volume of the tetrahedron specified by the points a, b, c and d
	//Usualy only the result's sign is used in algorithms checking for intersections etc.
	PX_FORCE_INLINE PxF64 orient3D(const PxVec3d& a, const PxVec3d& b, const PxVec3d& c, const PxVec3d& d)
	{
		return (a - d).dot((b - d).cross(c - d));
	}

	PX_FORCE_INLINE PxF64 sign(const PxF64 value)
	{
		if (value == 0)
			return 0.0;
		if (value > 0)
			return 1.0;
		return -1.0;
	}

	PX_FORCE_INLINE PxF64 signedDistancePointPlane(const PxVec3d& point, const PxVec3d& normal, PxF64 planeD)
	{
		return point.dot(normal) + planeD;
	}

	PX_FORCE_INLINE bool lineSegmentIntersectsTriangle(const PxVec3d& segmentStart, const PxVec3d& segmentEnd,
		const PxVec3d& triA, const PxVec3d& triB, const PxVec3d& triC, PxVec3d& intersectionPoint)
	{
		PxVec3d n = (triB - triA).cross(triC - triA);

		PxF64 l2 = n.magnitudeSquared();
		if (l2 < 1e-12)
			return false;

		//const PxF64 s = orient3D(segmentStart, triA, triB, triC);
		//const PxF64 e = orient3D(segmentEnd, triA, triB, triC);

		PxF64 planeD = -n.dot(triA);

		PxF64 s = signedDistancePointPlane(segmentStart, n, planeD);
		PxF64 e = signedDistancePointPlane(segmentEnd, n, planeD);

		if (s == 0 || e == 0)
			return false;

		if (/*s == 0 || e == 0 ||*/ sign(s) != sign(e))
		{
			const PxF64 ab = orient3D(segmentStart, segmentEnd, triA, triB);			
			const PxF64 bc = orient3D(segmentStart, segmentEnd, triB, triC);			
			const PxF64 ca = orient3D(segmentStart, segmentEnd, triC, triA);			

			const bool signAB = ab > 0;
			const bool signBC = bc > 0;
			const bool signCA = ca > 0;
			
			if ((ab == 0 && signBC == signCA) || 
				(bc == 0 && signAB == signCA) ||
				(ca == 0 && signAB == signBC) ||
				(signAB == signBC && signAB == signCA))
			{
				s = PxAbs(s);
				e = PxAbs(e);
				intersectionPoint = (segmentEnd * s + segmentStart * e) / (s + e);
				return true;
			}
			return false;
		}
		else
			return false;
	}

	//Helper class that controls the BVH traversal when checking if a specified edge intersects a triangle mesh with a BVH build around it
	//Edges intersecting the triangle mesh are scheduled to get split at the point where they intersect the mesh's surface
	class IntersectionFixingTraversalController
	{
	private:
		const PxArray<Triangle>& triangles;
		PxArray<PxVec3d>& points;
		PxHashMap<PxU64, PxI32>& edgesToSplit;
		PxArray<PxArray<PxI32>>& pointToOriginalTriangle;

		PxU64 edge;
		PxI32 a;
		PxI32 b;
		//PxF32 minX, minY, minZ, maxX, maxY, maxZ;
		PxBounds3 box;
		bool repeat = false;

	public:
		PX_FORCE_INLINE IntersectionFixingTraversalController(const PxArray<Triangle>& triangles_, PxArray<PxVec3d>& points_,
			PxHashMap<PxU64, PxI32>& edgesToSplit_, PxArray<PxArray<PxI32>>& pointToOriginalTriangle_) :
			triangles(triangles_), points(points_), edgesToSplit(edgesToSplit_), pointToOriginalTriangle(pointToOriginalTriangle_)
		{ }

		bool shouldRepeat() const { return repeat; }
		void resetRepeat() { repeat = false; }

		PX_FORCE_INLINE void update(PxU64 e)
		{
			edge = e;

			a = PxI32(e >> 32);
			b = PxI32(e);
			const PxVec3d& pA = points[a];
			const PxVec3d& pB = points[b];
			box = PxBounds3(PxVec3(PxF32(PxMin(pA.x, pB.x)), PxF32(PxMin(pA.y, pB.y)), PxF32(PxMin(pA.z, pB.z))),
				PxVec3(PxF32(PxMax(pA.x, pB.x)), PxF32(PxMax(pA.y, pB.y)), PxF32(PxMax(pA.z, pB.z))));
		}

		PX_FORCE_INLINE TraversalControl::Enum analyze(const BVHNode& node, PxI32)
		{
			if (node.isLeaf())
			{
				PxI32 j = node.getPrimitiveIndex();
				if (!contains(pointToOriginalTriangle[a], PxI32(j)) && !contains(pointToOriginalTriangle[b], PxI32(j)))
				{
					const Triangle& tri = triangles[j];
					const PxVec3d& triA = points[tri[0]];
					const PxVec3d& triB = points[tri[1]];
					const PxVec3d& triC = points[tri[2]];

					PxVec3d intersectionPoint;
					if (lineSegmentIntersectsTriangle(points[a], points[b], triA, triB, triC, intersectionPoint))
					{
						if (edgesToSplit.find(edge) == NULL)
						{
							edgesToSplit.insert(edge, PxI32(points.size()));
							points.pushBack(intersectionPoint);
							PxArray<PxI32> arr;
							arr.pushBack(j);
							pointToOriginalTriangle.pushBack(arr);
						}
						else
						{
							repeat = true;
						}
					}
				}

				return TraversalControl::eDontGoDeeper;
			}

			if (node.mBV.intersects(box))				
				return TraversalControl::eGoDeeper;				
			return TraversalControl::eDontGoDeeper;
		}

	private:
		PX_NOCOPY(IntersectionFixingTraversalController)
	};

#if PX_LINUX
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#endif

	void minMax(const PxArray<PxVec3d>& points, PxVec3d& min, PxVec3d& max)
	{
		min = PxVec3d(DBL_MAX, DBL_MAX, DBL_MAX);
		max = PxVec3d(-DBL_MAX, -DBL_MAX, -DBL_MAX);

		for (PxU32 i = 0; i < points.size(); ++i)
		{
			const PxVec3d& p = points[i];
			if (!PxIsFinite(p.x) || !PxIsFinite(p.y) || !PxIsFinite(p.z))
				continue;
			if (p.x > max.x) max.x = p.x; if (p.y > max.y) max.y = p.y; if (p.z > max.z) max.z = p.z;
			if (p.x < min.x) min.x = p.x; if (p.y < min.y) min.y = p.y; if (p.z < min.z) min.z = p.z;
		}
	}

#if PX_LINUX
#pragma GCC diagnostic pop
#endif

	//Creates a delaunay tetrahedralization out of the specified points
	void generateDelaunay3D(const PxArray<PxVec3d>& points, PxArray<Tetrahedron>& tetrahedra)
	{
		PxVec3d min, max;
		minMax(points, min, max);

		DelaunayTetrahedralizer delaunay(min, max);

		delaunay.insertPoints(points, 0, points.size(), tetrahedra);
	}

	PX_FORCE_INLINE PxF64 determinant(const PxVec3d& col1, const PxVec3d& col2, const PxVec3d& col3)
	{
		return col1.dot(col2.cross(col3));
	}

	//Simple but slow implementation of winding numbers to determine if a point is inside a mesh or not
	//https://igl.ethz.ch/projects/winding-number/robust-inside-outside-segmentation-using-generalized-winding-numbers-siggraph-2013-jacobson-et-al.pdf
	PxF64 windingNumber(const PxArray<PxVec3d>& points, const PxArray<Triangle>& triangles, const PxVec3d& p)
	{
		PxF64 sum = 0;

		for (PxU32 i = 0; i < triangles.size(); i++)
		{
			const Triangle& tri = triangles[i];
			const PxVec3d a = points[tri[0]] - p;
			const PxVec3d b = points[tri[1]] - p;
			const PxVec3d c = points[tri[2]] - p;
			PxF64 la = a.magnitude(), lb = b.magnitude(), lc = c.magnitude();
			PxF64 omega = atan2(determinant(a, b, c), (la * lb * lc + a.dot(b) * lc + b.dot(c) * la + c.dot(a) * lb));
			sum += omega;
		}

		sum *= 2;
		sum /= (4 * PI);
		return sum;
	}

	//Helper class to record the subdivision history of an edge
	struct SubdivisionEdge
	{
		PxI32 Start;
		PxI32 End;

		PxI32 ChildA;
		PxI32 ChildB;

		PX_FORCE_INLINE SubdivisionEdge(PxI32 start, PxI32 end, PxI32 childA = -1, PxI32 childB = -1) : Start(start), End(end), ChildA(childA), ChildB(childB) { }

		PX_FORCE_INLINE bool HasChildren() const { return ChildA >= 0; }
	};

	//A memory friendly implementation to compute a full batch of winding numbers for every specified query point in testPoints
	void multiWindingNumberMemoryFriendly(const PxArray<PxVec3d>& meshPoints, const PxArray<Triangle>& meshTriangles,
		const PxArray<PxVec3d>& testPoints, PxArray<PxF64>& result)
	{
		PxU32 l = testPoints.size();
		result.resize(l);
		for (PxU32 i = 0; i < l; ++i)
			result[i] = 0.0;

		for (PxU32 i = 0; i < meshTriangles.size(); i++)
		{
			const Triangle& tri = meshTriangles[i];
			const PxVec3d aa = meshPoints[tri[0]];
			const PxVec3d bb = meshPoints[tri[1]];
			const PxVec3d cc = meshPoints[tri[2]];

			for (PxU32 j = 0; j < l; ++j)
			{
				const PxVec3d p = testPoints[j];
				PxVec3d a = aa;
				PxVec3d b = bb;
				PxVec3d c = cc;
				a.x -= p.x; a.y -= p.y; a.z -= p.z;
				b.x -= p.x; b.y -= p.y; b.z -= p.z;
				c.x -= p.x; c.y -= p.y; c.z -= p.z;

				const PxF64 la = sqrt(a.x * a.x + a.y * a.y + a.z * a.z),
					lb = sqrt(b.x * b.x + b.y * b.y + b.z * b.z),
					lc = sqrt(c.x * c.x + c.y * c.y + c.z * c.z);

				const PxF64 y = a.x * b.y * c.z - a.x * b.z * c.y - a.y * b.x * c.z + a.y * b.z * c.x + a.z * b.x * c.y - a.z * b.y * c.x;
				const PxF64 x = (la * lb * lc + (a.x * b.x + a.y * b.y + a.z * b.z) * lc +
					(b.x * c.x + b.y * c.y + b.z * c.z) * la + (c.x * a.x + c.y * a.y + c.z * a.z) * lb);
				const PxF64 omega = atan2(y, x);

				result[j] += omega;
			}
		}

		PxF64 scaling = 2.0 / (4 * PI);
		for (PxU32 i = 0; i < l; ++i)
			result[i] *= scaling;
	}
	
	
	//Generates a tetmesh matching the surface of the specified triangle mesh exactly - might insert additional points on the
	//triangle mesh's surface. I also provides access about the location of newly created points.
	void generateTetmesh(const PxArray<PxVec3d>& trianglePoints, const PxArray<Triangle>& triangles,
		PxArray<SubdivisionEdge>& allEdges, PxI32& numOriginalEdges, PxArray<PxArray<PxI32>>& pointToOriginalTriangle,
		PxI32& numPointsBelongingToMultipleTriangles, PxArray<PxVec3d>& points, PxArray<Tetrahedron>& finalTets)
	{
		points.resize(trianglePoints.size());
		for (PxU32 i = 0; i < trianglePoints.size(); ++i)
			points[i] = trianglePoints[i];

		PxVec3d min, max;
		minMax(points, min, max);

		PxHashSet<PxU64> edges;
		for (PxU32 i = 0; i < triangles.size(); ++i)
		{
			const Triangle& tri = triangles[i];
			edges.insert(key(tri[0], tri[1]));
			edges.insert(key(tri[1], tri[2]));
			edges.insert(key(tri[0], tri[2]));
		}
		numOriginalEdges = PxI32(edges.size());

		allEdges.clear();
		for (PxHashSet<PxU64>::Iterator iter = edges.getIterator(); !iter.done(); ++iter)
		{
			allEdges.pushBack(SubdivisionEdge(PxI32((*iter) >> 32), PxI32(*iter)));
		}

		pointToOriginalTriangle.clear();
		for (PxU32 i = 0; i < points.size(); ++i)
			pointToOriginalTriangle.pushBack(PxArray<PxI32>());
		for (PxU32 i = 0; i < triangles.size(); ++i)
		{
			const Triangle& tri = triangles[i];
			pointToOriginalTriangle[tri[0]].pushBack(i);
			pointToOriginalTriangle[tri[1]].pushBack(i);
			pointToOriginalTriangle[tri[2]].pushBack(i);
		}
		for (PxU32 i = 0; i < points.size(); ++i) {
			PxArray<PxI32>& list = pointToOriginalTriangle[i];
			PxSort(list.begin(), list.size());
		}


		PxArray<Tetrahedron> tets;

		DelaunayTetrahedralizer del(min, max);
		PxU32 prevCount = 0;

		bool success = true;
		PxHashSet<PxU64> tetEdges;
		PxArray<PxI32> intersection;
		//Subdivide edges and insert new points ond edges into delaunay tetrahedralization until 
		//all required edges are present in the tetrahedralization
		while (success)
		{
			success = false;

			del.insertPoints(points, prevCount, points.size(), tets);
			prevCount = points.size();

			tetEdges.clear();
			for (PxU32 i = 0; i < tets.size(); ++i)
			{
				const Tetrahedron& tet = tets[i];
				tetEdges.insert(key(tet[0], tet[1]));
				tetEdges.insert(key(tet[0], tet[2]));
				tetEdges.insert(key(tet[0], tet[3]));
				tetEdges.insert(key(tet[1], tet[2]));
				tetEdges.insert(key(tet[1], tet[3]));
				tetEdges.insert(key(tet[2], tet[3]));
			}

			PxU32 l = allEdges.size();
			for (PxU32 i = 0; i < l; ++i)
			{
				SubdivisionEdge e = allEdges[i]; //Copy the edge here because we modify the list below and the reference could get invalid if the list resizes internally
				if (e.HasChildren())
					continue;

				const PxU64 k = key(e.Start, e.End);
				if (!tetEdges.contains(k))
				{
					allEdges.pushBack(SubdivisionEdge(e.Start, PxI32(points.size())));
					allEdges.pushBack(SubdivisionEdge(PxI32(points.size()), e.End));
					e.ChildA = PxI32(allEdges.size()) - 2;
					e.ChildB = PxI32(allEdges.size()) - 1;
					allEdges[i] = e; //Write the modified edge back since we did not capture a reference but made a copy 

					points.pushBack((points[e.Start] + points[e.End]) * 0.5);
					intersection.clear();
					intersectionOfSortedLists(pointToOriginalTriangle[e.Start], pointToOriginalTriangle[e.End], intersection);
					pointToOriginalTriangle.pushBack(intersection);

					success = true;
				}
			}
		}

		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			Tetrahedron& tet = tets[i];
			if (tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]) < 0)
				PxSwap(tet[0], tet[1]);
		}


		PxHashMap<PxU64, PxI32> edgesToSplit;
		numPointsBelongingToMultipleTriangles = PxI32(points.size());

		//Split all tetrahedron edges that penetrate the triangle mesh's surface
		PxArray<BVHNode> tree;
		buildTree(triangles, points, tree);
		IntersectionFixingTraversalController controller(triangles, points, edgesToSplit, pointToOriginalTriangle);

		for (PxHashSet<PxU64>::Iterator iter = tetEdges.getIterator(); !iter.done(); ++iter)
		{
			const PxU64 edge = *iter;
			controller.update(edge);
			traverseBVH(tree.begin(), controller);
		}
		split(tets, points, edgesToSplit);

		//Remove all tetrahedra that are outside of the triangle mesh
		PxHashMap<PxU32, ClusterApproximationF64> clusters;
		precomputeClusterInformation(tree, triangles, points, clusters);

		PxArray<PxF64> windingNumbers;
		windingNumbers.resize(tets.size());
		PxF64 sign = 1;
		PxF64 windingNumberSum = 0;
		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			const Tetrahedron& tet = tets[i];
			PxVec3d q = (points[tet[0]] + points[tet[1]] + points[tet[2]] + points[tet[3]]) * 0.25;
			PxF64 windingNumber = computeWindingNumber(tree, q, 2.0, clusters, triangles, points);

			windingNumbers[i] = windingNumber;
			windingNumberSum += windingNumber;			
		}
		if (windingNumberSum < 0.0)
			sign = -1;

		//Array<PxVec3d> tetCenters;
		//tetCenters.resize(tets.size());
		//for (PxU32 i = 0; i < tets.size(); ++i)
		//{
		//	const Tetrahedron& tet = tets[i];
		//	tetCenters[i] = (points[tet.A] + points[tet.B] + points[tet.c] + points[tet.D]) * 0.25;
		//}
		//multiWindingNumberMemoryFriendly(points, triangles, tetCenters, windingNumbers);

		finalTets.clear();
		for (PxU32 i = 0; i < windingNumbers.size(); ++i)
			if (sign * windingNumbers[i] > 0.5)
				finalTets.pushBack(tets[i]);

		for (PxU32 i = 0; i < finalTets.size(); ++i)
		{
			Tetrahedron& tet = finalTets[i];
			if (tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]) < 0)
				PxSwap(tet[0], tet[1]);
		}
	}

	void generateTetmesh(const PxArray<PxVec3d>& trianglePoints, const PxArray<Triangle>& triangles,
		PxArray<PxVec3d>& points, PxArray<Tetrahedron>& finalTets)
	{
		PxArray<SubdivisionEdge> allEdges;
		PxI32 numOriginalEdges;
		PxArray<PxArray<PxI32>> pointToOriginalTriangle;
		PxI32 numPointsBelongingToMultipleTriangles;
		generateTetmesh(trianglePoints, triangles, allEdges, numOriginalEdges, pointToOriginalTriangle, numPointsBelongingToMultipleTriangles, points, finalTets);
	}

	static const PxI32 tetFaces[4][3] = { {0, 1, 2},  {0, 3, 1},  {0, 2, 3}, {1, 3, 2} };
	void extractTetmeshSurface(const PxArray<PxI32>& tets, PxArray<PxI32>& triangles)
	{
		PxHashMap<SortedTriangle, PxI32, TriangleHash> tris;

		for (PxU32 i = 0; i < tets.size(); i += 4)
		{
			for (PxU32 j = 0; j < 4; ++j)
			{
				SortedTriangle tri(tets[i + tetFaces[j][0]], tets[i + tetFaces[j][1]], tets[i + tetFaces[j][2]]);
				if (const PxPair<const SortedTriangle, PxI32>* ptr = tris.find(tri))
					tris[tri] = ptr->second + 1;
				else
					tris.insert(tri, 1);
			}
		}

		triangles.clear();
		for (PxHashMap<SortedTriangle, PxI32, TriangleHash>::Iterator iter = tris.getIterator(); !iter.done(); ++iter)
		{
			if (iter->second == 1) {
				triangles.pushBack(iter->first.A);
				if (iter->first.Flipped)
				{
					triangles.pushBack(iter->first.C);
					triangles.pushBack(iter->first.B);
				}
				else
				{
					triangles.pushBack(iter->first.B);
					triangles.pushBack(iter->first.C);
				}
			}
		}
	}

	struct TriangleWithTetLink
	{
		PxI32 triA;
		PxI32 triB;
		PxI32 triC;
		PxI32 tetId;

		TriangleWithTetLink(PxI32 triA_, PxI32 triB_, PxI32 triC_, PxI32 tetId_) : triA(triA_), triB(triB_), triC(triC_), tetId(tetId_)
		{}
	};

	void extractTetmeshSurfaceWithTetLink(const PxArray<Tetrahedron>& tets, PxArray<TriangleWithTetLink>& surface)
	{
		PxHashMap<SortedTriangle, PxI32, TriangleHash> tris;

		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			if (tets[i][0] < 0)
				continue;
			for (PxU32 j = 0; j < 4; ++j)
			{
				SortedTriangle tri(tets[i][tetFaces[j][0]], tets[i][tetFaces[j][1]], tets[i][tetFaces[j][2]]);
				if (tris.find(tri))
					tris[tri] = -1;
				else
					tris.insert(tri, i);
			}
		}

		surface.clear();
		for (PxHashMap<SortedTriangle, PxI32, TriangleHash>::Iterator iter = tris.getIterator(); !iter.done(); ++iter)
		{
			if (iter->second >= 0)			
				surface.pushBack(TriangleWithTetLink(iter->first.A, iter->first.B, iter->first.C, iter->second));
		}
	}

	//Removes vertices not referenced by any tetrahedron and maps the tet's indices to match the compacted vertex list
	void removeUnusedVertices(PxArray<PxVec3d>& vertices, PxArray<Tetrahedron>& tets, PxU32 numPointsToKeepAtBeginning = 0)
	{
		PxArray<PxI32> compressorMap;
		compressorMap.resize(vertices.size());

		for (PxU32 i = 0; i < numPointsToKeepAtBeginning; ++i)
			compressorMap[i] = 0;
		for (PxU32 i = numPointsToKeepAtBeginning; i < compressorMap.size(); ++i)
			compressorMap[i] = -1;

		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			const Tetrahedron& tet = tets[i];
			if (tet[0] < 0)
				continue;
			compressorMap[tet[0]] = 0;
			compressorMap[tet[1]] = 0;
			compressorMap[tet[2]] = 0;
			compressorMap[tet[3]] = 0;
		}

		PxU32 indexer = 0;
		for (PxU32 i = 0; i < compressorMap.size(); ++i)
		{
			if (compressorMap[i] >= 0)
			{
				compressorMap[i] = indexer;
				vertices[indexer] = vertices[i];
				indexer++;
			}
		}

		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			Tetrahedron&  tet = tets[i];
			if (tet[0] < 0)
				continue;
			tet[0] = compressorMap[tet[0]];
			tet[1] = compressorMap[tet[1]];
			tet[2] = compressorMap[tet[2]];
			tet[3] = compressorMap[tet[3]];
		}

		if (indexer < vertices.size())
			vertices.removeRange(indexer, vertices.size() - indexer);
	}

	PxF64 tetQuality(const PxVec3d& p0, const PxVec3d& p1, const PxVec3d& p2, const PxVec3d& p3)
	{
		const PxVec3d d0 = p1 - p0;
		const PxVec3d d1 = p2 - p0;
		const PxVec3d d2 = p3 - p0;
		const PxVec3d d3 = p2 - p1;
		const PxVec3d d4 = p3 - p2;
		const PxVec3d d5 = p1 - p3;

		PxF64 s0 = d0.magnitudeSquared();
		PxF64 s1 = d1.magnitudeSquared();
		PxF64 s2 = d2.magnitudeSquared();
		PxF64 s3 = d3.magnitudeSquared();
		PxF64 s4 = d4.magnitudeSquared();
		PxF64 s5 = d5.magnitudeSquared();

		PxF64 ms = (1.0 / 6.0) * (s0 + s1 + s2 + s3 + s4 + s5);
		PxF64 rms = PxSqrt(ms);

		PxF64 s = 12.0 / PxSqrt(2.0);

		PxF64 vol = PxAbs((1.0 / 6.0) * d0.dot(d1.cross(d2)));
		return s * vol / (rms * rms * rms); // Ideal tet has quality 1
	}
	
	//The face must be one of the 4 faces from the tetrahedron, otherwise the result will be incorrect
	PX_FORCE_INLINE PxI32 getTetCornerOppositeToFace(const Tetrahedron& tet, PxI32 faceA, PxI32 faceB, PxI32 faceC)
	{
		return tet[0] + tet[1] + tet[2] + tet[3] - faceA - faceB - faceC;
	}

	void improveTetmesh(PxArray<PxVec3d>& points, PxArray<Tetrahedron>& finalTets, PxI32 numPointsBelongingToMultipleTriangles,
		PxArray<PxArray<PxI32>>& pointToOriginalTriangle, PxArray<PxArray<PxI32>>& edges, PxI32 numOriginalPoints)
	{
		DelaunayTetrahedralizer del(points, finalTets);
		bool success = true;
		//Collapse edges as long as we find collapsible edges
		//Only collaps edges such that the input triangle mesh's edges are preserved
		while (success)
		{
			success = false;

			PxArray<PxI32> adjTets;
			// Try to remove points that are on the interior of an original face
			for (PxU32 i = numPointsBelongingToMultipleTriangles; i < points.size(); ++i)
			{
				PxI32 tri = pointToOriginalTriangle[i][0];

				adjTets.forceSize_Unsafe(0);
				del.collectTetsConnectedToVertex(i, adjTets);
				for (PxU32 j = 0; j < adjTets.size(); ++j)
				{
					const Tetrahedron& tet = del.tetrahedron(adjTets[j]);
					if (tet[0] < 0)
						continue;

					for (PxI32 k = 0; k < 4; ++k) {
						PxI32 id = tet[k];
						if (id != PxI32(i) && contains(pointToOriginalTriangle[id], tri))
						{
							if (del.canCollapseEdge(id, i))
							{
								del.collapseEdge(id, i);
								break;
							}
						}
					}
				}
			}

			// Try to remove points that are on the edge between two original faces
			for (PxU32 i = 0; i < edges.size(); ++i)
			{
				PxArray<PxI32>& edge = edges[i];
				if (edge.size() == 2)
					continue;

				for (PxU32 j = edge.size() - 1; j >= 1; --j)
				{
					const PxI32 remove = edge[j - 1];
					const PxI32 keep = edge[j];
					if (remove >= numOriginalPoints && del.canCollapseEdge(keep, remove))
					{
						del.collapseEdge(keep, remove);
						success = true;
						edge.remove(j - 1);
					}
				}

				for (PxU32 j = 1; j < edge.size(); ++j)
				{
					const PxI32 keep = edge[j - 1];
					const PxI32 remove = edge[j];
					if (remove >= numOriginalPoints && del.canCollapseEdge(keep, remove))
					{
						del.collapseEdge(keep, remove);
						success = true;
						edge.remove(j);
						--j;
					}
				}
			}
		}

		optimize(del, pointToOriginalTriangle, numOriginalPoints, points, finalTets, 10);

		//Remove sliver tets on surface
		success = true;
		while (success)
		{
			success = false;
			PxArray<TriangleWithTetLink> surface;
			extractTetmeshSurfaceWithTetLink(finalTets, surface);

			for (PxU32 i = 0; i < surface.size(); ++i)
			{
				const TriangleWithTetLink& link = surface[i];
				const Tetrahedron& tet = finalTets[link.tetId];
				if (tet[0] < 0)
					continue;
				PxI32 other = getTetCornerOppositeToFace(tet, link.triA, link.triB, link.triC);

				const PxVec3d& a = points[link.triA];
				const PxVec3d& b = points[link.triB];
				const PxVec3d& c = points[link.triC];
				PxVec3d n = (b - a).cross(c - a);
				//n.normalize();
				PxF64 planeD = -(n.dot(a));

				PxF64 dist = PxAbs(signedDistancePointPlane(points[other], n, planeD));
				if (dist < 1e-4 * n.magnitude())
				{
					finalTets[link.tetId] = Tetrahedron(-1, -1, -1, -1);
					success = true;
				}
			}
		}


		PxU32 indexer = 0;
		for (PxU32 i = 0; i < finalTets.size(); ++i)
		{
			const Tetrahedron& tet = finalTets[i];
			if (tet[0] >= 0)
				finalTets[indexer++] = tet;
		}
		if (indexer < finalTets.size())
			finalTets.removeRange(indexer, finalTets.size() - indexer);

		removeUnusedVertices(points, finalTets, numOriginalPoints);

		for (PxU32 i = 0; i < finalTets.size(); ++i)
		{
			Tetrahedron& tet = finalTets[i];
			if (tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]) < 0)
				PxSwap(tet[0], tet[1]);
		}
	}

	PxU32 removeDisconnectedIslands(PxI32* finalTets, PxU32 numTets)
	{
		//Detect islands
		PxArray<PxI32> neighborhood;
		buildNeighborhood(finalTets, numTets, neighborhood);
		PxArray<PxI32> tetColors;
		tetColors.resize(numTets, -1);
		PxU32 start = 0;
		PxI32 color = -1;
		PxArray<PxI32> stack;
		while (true)
		{
			stack.clear();
			while (start < tetColors.size())
			{
				if (tetColors[start] < 0)
				{
					stack.pushBack(start);
					++color;
					tetColors[start] = color;
					break;
				}
				++start;
			}

			if (start == tetColors.size())
				break;

			while (stack.size() > 0)
			{
				PxI32 id = stack.popBack();
				for (PxI32 i = 0; i < 4; ++i)
				{
					PxI32 a = neighborhood[4 * id + i];
					PxI32 tetId = a >> 2;
					if (tetId >= 0 && tetColors[tetId] == -1)
					{
						stack.pushBack(tetId);
						tetColors[tetId] = color;
					}
				}
			}
		}

		if (color > 0)
		{
			//Found more than one island: Count number of tets per color
			PxArray<PxU32> numTetsPerColor;
			numTetsPerColor.resize(color + 1, 0);
			for (PxU32 i = 0; i < tetColors.size(); ++i)
				numTetsPerColor[tetColors[i]] += 1;

			PxI32 colorWithHighestTetCount = 0;
			for (PxU32 i = 1; i < numTetsPerColor.size(); ++i)
				if (numTetsPerColor[i] > numTetsPerColor[colorWithHighestTetCount])
					colorWithHighestTetCount = i;

			PxU32 indexer = 0;
			for (PxU32 i = 0; i < numTets; ++i)
			{
				for (PxU32 j = 0; j < 4; ++j)
					finalTets[4 * indexer + j] = finalTets[4 * i + j];
				if (tetColors[i] == colorWithHighestTetCount)
					++indexer;
			}
			//if (indexer < finalTets.size())
			//	finalTets.removeRange(indexer, finalTets.size() - indexer);
			return numTets - indexer;
		}
		return 0;
	}

	void removeDisconnectedIslands(PxArray<Tetrahedron>& finalTets)
	{
		PxU32 numRemoveAtEnd = removeDisconnectedIslands(reinterpret_cast<PxI32*>(finalTets.begin()), finalTets.size());
		if (numRemoveAtEnd > 0)		
			finalTets.removeRange(finalTets.size() - numRemoveAtEnd, numRemoveAtEnd);		
	}

	//Generates a tetmesh matching the surface of the specified triangle mesh exactly - might insert additional points on the
	//triangle mesh's surface. It will try to remove as many points inserted during construction as possible by applying an 
	//edge collapse post processing step.
	void generateTetsWithCollapse(const PxArray<PxVec3d>& trianglePoints, const PxArray<Triangle>& triangles,
		PxArray<PxVec3d>& points, PxArray<Tetrahedron>& finalTets)
	{
		const PxI32 numOriginalPoints = PxI32(trianglePoints.size());
	
		PxArray<PxArray<PxI32>> pointToOriginalTriangle;
		
		PxI32 numPointsBelongingToMultipleTriangles;

		PxArray<PxArray<PxI32>> edges;
	

		PxVec3d min, max;
		minMax(trianglePoints, min, max);
		DelaunayTetrahedralizer del(min, max);
		PxArray<Tetrahedron> tets;
		del.generateTetmeshEnforcingEdges(trianglePoints, triangles, edges, pointToOriginalTriangle, points, tets);
 

		PxHashSet<PxU64> tetEdges;

		numPointsBelongingToMultipleTriangles = points.size();
		PxHashMap<PxU64, PxI32> edgesToSplit;
		
		IntersectionFixingTraversalController controller(triangles, points, edgesToSplit, pointToOriginalTriangle);

		PxArray<BVHNode> tree;
		buildTree(triangles, points, tree);
		
		PxI32 counter = 0;
		do
		{
			controller.resetRepeat();
			tetEdges.clear();
			for (PxU32 i = 0; i < tets.size(); ++i)
			{
				const Tetrahedron& tet = tets[i];
				tetEdges.insert(key(tet[0], tet[1]));
				tetEdges.insert(key(tet[0], tet[2]));
				tetEdges.insert(key(tet[0], tet[3]));
				tetEdges.insert(key(tet[1], tet[2]));
				tetEdges.insert(key(tet[1], tet[3]));
				tetEdges.insert(key(tet[2], tet[3]));
			}

			edgesToSplit.clear();
			for (PxHashSet<PxU64>::Iterator iter = tetEdges.getIterator(); !iter.done(); ++iter)
			{
				const PxU64 edge = *iter;
				controller.update(edge);
				traverseBVH(tree.begin(), controller);
			}
			
			split(tets, points, /*remaining*/edgesToSplit);
			++counter;
			if (counter >= 2)
				break;
		} while (controller.shouldRepeat());

		//Remove all tetrahedra that are outside of the triangle mesh
		PxHashMap<PxU32, ClusterApproximationF64> clusters;
		precomputeClusterInformation(tree, triangles, points, clusters);

		PxArray<PxF64> windingNumbers;
		windingNumbers.resize(tets.size());
		PxF64 sign = 1;
		PxF64 windingNumberSum = 0;
		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			const Tetrahedron& tet = tets[i];
			PxVec3d q = (points[tet[0]] + points[tet[1]] + points[tet[2]] + points[tet[3]]) * 0.25;
			PxF64 windingNumber = computeWindingNumber(tree, q, 2.0, clusters, triangles, points);
			windingNumbers[i] = windingNumber;
			windingNumberSum += windingNumber;
		}
		if (windingNumberSum < 0.0)
			sign = -1;

		finalTets.clear();
		for (PxU32 i = 0; i < windingNumbers.size(); ++i)
			if (sign * windingNumbers[i] > 0.5)
				finalTets.pushBack(tets[i]);

		for (PxU32 i = 0; i < finalTets.size(); ++i)
		{
			Tetrahedron& tet = finalTets[i];
			if (tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]) < 0)
				PxSwap(tet[0], tet[1]);
		}

		improveTetmesh(points, finalTets, numPointsBelongingToMultipleTriangles, pointToOriginalTriangle, edges, numOriginalPoints);
	}
	
	bool convexTetmesh(PxArray<PxVec3d>& points, const PxArray<Triangle>& tris, PxArray<Tetrahedron>& tets)
	{
		PxVec3d centroid = PxVec3d(0.0, 0.0, 0.0);
		PxI32 counter = 0;
		for (PxU32 i = 0; i < points.size(); ++i) 
		{
			const PxVec3d& p = points[i];
			if (!PxIsFinite(p.x) || !PxIsFinite(p.y) || !PxIsFinite(p.z))
				continue;
			centroid += p;
			++counter;
		}

		centroid /= counter;

		PxF64 volSign = 0;
		PxU32 centerIndex = points.size();
		points.pushBack(centroid);
		tets.clear();
		tets.reserve(tris.size());
		for (PxU32 i = 0; i < tris.size(); ++i)
		{
			const Triangle& tri = tris[i];
			Tetrahedron tet = Tetrahedron(centerIndex, tri[0], tri[1], tri[2]);			
			const PxF64 vol = tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]);

			if (vol < 0)			
				PxSwap(tet[2], tet[3]);
			
			tets.pushBack(tet);

			if (volSign == 0)
			{
				volSign = vol > 0 ? 1 : -1;
			}
			else if (volSign * vol < 0)
			{
				points.remove(points.size() - 1);
				tets.clear();
				return false;
			}
		}

		return true;
	}
	
	void convert(const PxArray<PxF32>& points, PxArray<PxVec3d>& result)
	{
		result.resize(points.size() / 3);
		for (PxU32 i = 0; i < result.size(); ++i)
			result[i] = PxVec3d(PxF64(points[3 * i]), PxF64(points[3 * i + 1]), PxF64(points[3 * i + 2]));
	}

	void convert(const PxArray<PxVec3d>& points, PxArray<PxF32>& result)
	{
		result.resize(3 * points.size());
		for (PxU32 i = 0; i < points.size(); ++i)
		{
			const PxVec3d& p = points[i];
			result[3 * i] = PxF32(p.x);
			result[3 * i + 1] = PxF32(p.y);
			result[3 * i + 2] = PxF32(p.z);
		}
	}

	void convert(const PxArray<PxVec3d>& points, PxArray<PxVec3>& result)
	{
		result.resize(points.size());
		for (PxU32 i = 0; i < points.size(); ++i)
		{
			const PxVec3d& p = points[i];
			result[i] = PxVec3(PxF32(p.x), PxF32(p.y), PxF32(p.z));
		}
	}

	void convert(const PxBoundedData& points, PxArray<PxVec3d>& result)
	{
		result.resize(points.count);
		for (PxU32 i = 0; i < points.count; ++i)
		{
			const PxVec3& p = points.at<PxVec3>(i);
			result[i] = PxVec3d(PxF64(p.x), PxF64(p.y), PxF64(p.z));
		}
	}

	void convert(const PxArray<PxI32>& indices, PxArray<Triangle>& result)
	{
		//static cast possible?
		result.resize(indices.size() / 3);
		for (PxU32 i = 0; i < result.size(); ++i)
			result[i] = Triangle(indices[3 * i], indices[3 * i + 1], indices[3 * i + 2]);
	}

	void convert(const PxBoundedData& indices, bool has16bitIndices, PxArray<Triangle>& result)
	{
		result.resize(indices.count);
		if (has16bitIndices)
		{
			for (PxU32 i = 0; i < indices.count; ++i)
			{
				const Triangle16& tri = indices.at<Triangle16>(i);
				result[i] = Triangle(tri[0], tri[1], tri[2]);
			}
		}
		else
		{			
			for (PxU32 i = 0; i < indices.count; ++i)
				result[i] = indices.at<Triangle>(i);
		}
	}

	void convert(const PxArray<Tetrahedron>& tetrahedra, PxArray<PxU32>& result)
	{
		//static cast possible?
		result.resize(4 * tetrahedra.size());
		for (PxU32 i = 0; i < tetrahedra.size(); ++i)
		{
			const Tetrahedron& t = tetrahedra[i];
			result[4 * i] = t[0];
			result[4 * i + 1] = t[1];
			result[4 * i + 2] = t[2];
			result[4 * i + 3] = t[3];
		}
	}

	//Keep for debugging & verification
	void writeTets(const char* path, const PxArray<PxVec3>& tetPoints, const PxArray<PxU32>& tets)
	{
		FILE *fp;

		fp = fopen(path, "w+");
		fprintf(fp, "# Tetrahedral mesh generated using\n\n");


		fprintf(fp, "# %d vertices\n", tetPoints.size());
		for (PxU32 i = 0; i < tetPoints.size(); ++i)
		{
			fprintf(fp, "v %f %f %f\n", PxF64(tetPoints[i].x), PxF64(tetPoints[i].y), PxF64(tetPoints[i].z));
		}

		fprintf(fp, "\n");
		fprintf(fp, "# %d tetrahedra\n", (tets.size() / 4));
		for (PxU32 i = 0; i < tets.size(); i += 4)
		{
			fprintf(fp, "t %d %d %d %d\n", tets[i], tets[i + 1], tets[i + 2], tets[i + 3]);
		}

		fclose(fp);
	}

	//Keep for debugging & verification
	void writeOFF(const char* path, const PxArray<PxF32>& vertices, const PxArray<PxI32>& tris)
	{
		FILE *fp;

		fp = fopen(path, "w+");
		fprintf(fp, "OFF\n");

		fprintf(fp, "%d %d 0\n", vertices.size() / 3, tris.size() / 3);
		for (PxU32 i = 0; i < vertices.size(); i += 3)
		{
			fprintf(fp, "%f %f %f\n", PxF64(vertices[i]), PxF64(vertices[i + 1]), PxF64(vertices[i + 2]));
		}

		for (PxU32 i = 0; i < tris.size(); i += 3)
		{
			fprintf(fp, "3 %d %d %d\n", tris[i], tris[i + 1], tris[i + 2]);
		}

		fclose(fp);
	}

	//Keep for debugging & verification
	void writeSTL(const char* path, const PxArray<PxVec3>& vertices, const PxArray<PxI32>& tris)
	{
		FILE *fp;

		fp = fopen(path, "w+");
		fprintf(fp, "solid mesh\n");

		for (PxU32 i = 0; i < tris.size(); i += 3)
		{
			const PxI32* tri = &tris[i];
			const PxVec3& a = vertices[tri[0]];
			const PxVec3& b = vertices[tri[1]];
			const PxVec3& c = vertices[tri[2]];
			PxVec3 n = (b - a).cross(c - a);
			n.normalize();

			fprintf(fp, "facet normal %f %f %f\n", PxF64(n.x), PxF64(n.y), PxF64(n.z));
			fprintf(fp, "%s", "outer loop\n");

			fprintf(fp, "    vertex %f %f %f\n", PxF64(a.x), PxF64(a.y), PxF64(a.z));
			fprintf(fp, "    vertex %f %f %f\n", PxF64(b.x), PxF64(b.y), PxF64(b.z));
			fprintf(fp, "    vertex %f %f %f\n", PxF64(c.x), PxF64(c.y), PxF64(c.z));

			fprintf(fp, "%s", "endloop\n");
			fprintf(fp, "%s", "endfacet\n");
		}

		fprintf(fp, "endsolid mesh\n");
		fclose(fp);
	}

	void generateTetmesh(const PxBoundedData& inputPoints, const PxBoundedData& inputTriangles, const bool has16bitIndices,
		PxArray<PxVec3>& tetPoints, PxArray<PxU32>& finalTets)
	{
		//writeOFF("c:\\tmp\\debug.off", trianglePoints, triangles);

		PxArray<PxVec3d> points;
		convert(inputPoints, points);
		PxArray<Triangle> tris;
		convert(inputTriangles, has16bitIndices, tris);

		//PxTriangleMeshAnalysisResults result = validateTriangleMesh(inputPoints, inputTriangles, has16bitIndices);
		//PX_ASSERT(!(result & PxTriangleMeshAnalysisResult::eMESH_IS_INVALID));

		PxArray<PxI32> map;
		MeshAnalyzer::mapDuplicatePoints<PxVec3d, PxF64>(points.begin(), points.size(), map);
		for (PxI32 i = 0; i < PxI32(points.size()); ++i)
		{
			if(map[i] != i)
				points[i] = PxVec3d(PxF64(NAN), PxF64(NAN), PxF64(NAN));
		}
		for (PxU32 i = 0; i < tris.size(); ++i)
		{
			Triangle& t = tris[i];
			for (PxU32 j = 0; j < 3; ++j)
				t[j] = map[t[j]];
			if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2])
			{
				tris[i] = tris[tris.size() - 1];
				tris.remove(tris.size() - 1);
				--i;
			}
		}

		PxArray<PxVec3d> tetPts;
		PxArray<Tetrahedron> tets;
		//if (makeTriOrientationConsistent(tris))
		{
			if (convexTetmesh(points, tris, tets))
			{
				tetPts.clear();
				tetPts.reserve(tris.size());
				for (PxU32 i = 0/*l*/; i < map.size(); ++i)
				{
					tetPts.pushBack(points[map[i]]);
				}
				for (PxU32 i = map.size(); i < points.size(); ++i)
				{
					tetPts.pushBack(points[i]);
				}
			}
			else
			{
				//Transform points such that the are located inside the unit cube
				PxVec3d min, max;
				minMax(points, min, max);
				PxVec3d size = max - min;
				PxF64 scaling = 1.0 / PxMax(size.x, PxMax(size.y, size.z));

				//Add some noise to avoid geometric degeneracies
				Cm::RandomR250 r(0);
				PxF64 randomMagnitude = 1e-6;
				for (PxU32 i = 0; i < points.size(); ++i)
				{
					PxVec3d& p = points[i];
					p = (p - min) * scaling;
					p.x += PxF64(r.rand(-0.5f, 0.5f)) * randomMagnitude;
					p.y += PxF64(r.rand(-0.5f, 0.5f)) * randomMagnitude;
					p.z += PxF64(r.rand(-0.5f, 0.5f)) * randomMagnitude;
				}

				generateTetsWithCollapse(points, tris, tetPts, tets);

				//Scale back to original size
				scaling = 1.0 / scaling;
				//for (PxU32 i = 0; i < l; ++i)
				//	tetPts[i] = PxVec3d(trianglePoints[3 * i], trianglePoints[3 * i + 1], trianglePoints[3 * i + 2]);
				for (PxU32 i = 0; i < map.size(); ++i)
				{
					tetPts[i] = tetPts[map[i]];
				}
				for (PxU32 i = 0; i < tetPts.size(); ++i)
				{
					tetPts[i] = tetPts[i] * scaling + min;
				}
			}
		}
		convert(tetPts, tetPoints);
		convert(tets, finalTets);

		//writeTets("c:\\tmp\\bottle.tet", tetPoints, finalTets);
	}

	PX_FORCE_INLINE PxF32 tetVolume(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
	{
		return (-1.0f / 6.0f) * (a - d).dot((b - d).cross(c - d));
	}

	void pointMasses(const PxArray<PxVec3>& tetVerts, const PxArray<PxU32>& tets, PxF32 density, PxArray<PxF32>& mass)
	{
		mass.resize(tetVerts.size());
		for (PxU32 i = 0; i < mass.size(); ++i)
			mass[i] = 0.0f;

		//const PxVec3* verts = (PxVec3*)&tetVerts[0];
		for (PxU32 i = 0; i < tets.size(); i += 4)
		{
			PxF32 weightDiv4 = density * 0.25f * PxAbs(tetVolume(tetVerts[tets[i]], tetVerts[tets[i + 1]], tetVerts[tets[i + 2]], tetVerts[tets[i + 3]]));
			mass[tets[i]] += weightDiv4;
			mass[tets[i + 1]] += weightDiv4;
			mass[tets[i + 2]] += weightDiv4;
			mass[tets[i + 3]] += weightDiv4;
		}
	}

	void restPoses(const PxArray<PxVec3>& tetVerts, const PxArray<PxU32>& tets, PxArray<PxMat33>& restPoses)
	{
		restPoses.resize(tets.size() / 4);

		//const PxVec3* verts = (PxVec3*)&tetVerts[0];
		for (PxU32 i = 0; i < tets.size(); i += 4)
		{
			const PxVec3 u1 = tetVerts[tets[i + 1]] - tetVerts[tets[i]];
			const PxVec3 u2 = tetVerts[tets[i + 2]] - tetVerts[tets[i]];
			const PxVec3 u3 = tetVerts[tets[i + 3]] - tetVerts[tets[i]];

			const PxMat33 m = PxMat33(u1, u2, u3);
			const PxMat33 rest = m.getInverse();
			restPoses[i / 4] = rest;
		}
	}

	void tetFibers(const PxArray<PxVec3>& /*tetVerts*/, const PxArray<PxU32>& tets, PxArray<PxVec3>& tetFibers)
	{
		//Just use dummy data for the moment. Could solve a heat equation on the tetmesh to get better fibers but the boundary conditions of the heat quations need to be known
		tetFibers.resize(tets.size() / 4);
		for (PxU32 i = 0; i < tets.size(); i += 4)
		{
			tetFibers[i / 4] = PxVec3(1.0f, 0.f, 0.f);
		}
	}



	void minMax(const PxBoundedData& points, PxVec3& min, PxVec3& max)
	{
		min = PxVec3(PX_MAX_F32);
		max = PxVec3(-PX_MAX_F32);

		for (PxU32 i = 0; i < points.count; ++i)
		{
			const PxVec3& p = points.at<PxVec3>(i);
			if (!PxIsFinite(p.x) || !PxIsFinite(p.y) || !PxIsFinite(p.z))
				continue;
			max = max.maximum(p);
			min = min.minimum(p);
		}
	}

	const PxI32 xNegFace[4] = { 0, 1, 2, 3 };
	const PxI32 xPosFace[4] = { 4, 5, 6, 7 };	
	const PxI32 yNegFace[4] = { 0, 1, 4, 5 };
	const PxI32 yPosFace[4] = { 2, 3, 6, 7 };	
	const PxI32 zNegFace[4] = { 0, 2, 4, 6 };
	const PxI32 zPosFace[4] = { 1, 3, 5, 7 };

	const PxI32 offsetsX[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
	const PxI32 offsetsY[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
	const PxI32 offsetsZ[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };


	const PxI32 tets6PerVoxel[24] = { 0,1,6,2,  0,1,4,6,  1,4,6,5,  1,2,3,6,  1,3,7,6,  1,5,6,7 };

	struct VoxelNodes
	{
		PxI32 c[8];

		//   XYZ
		PxI32 c000() { return c[0]; }
		PxI32 c001() { return c[1]; }
		PxI32 c010() { return c[2]; }
		PxI32 c011() { return c[3]; }
		PxI32 c100() { return c[4]; }
		PxI32 c101() { return c[5]; }
		PxI32 c110() { return c[6]; }
		PxI32 c111() { return c[7]; }

		VoxelNodes(PxI32& nodeIndexer)
		{
			for (PxI32 i = 0; i < 8; ++i)
				c[i] = nodeIndexer++;
		}
	};


	static const PxI32 neighborFacesAscending[4][3] = { { 0, 1, 2 }, { 0, 1, 3 }, { 0, 2, 3 }, { 1, 2, 3 } };
	struct Vox
	{
		PxU32 mLocationX, mLocationY, mLocationZ;
		PxArray<PxI32> mTets;
		PxArray<PxArray<PxI32>> mClusters;
		PxArray<VoxelNodes> mNodes;
		PxU32 mBaseTetIndex;
		PxU32 mNumEmittedTets;

		Vox(PxU32 locationX, PxU32 locationY, PxU32 locationZ) : mLocationX(locationX), mLocationY(locationY), mLocationZ(locationZ), mBaseTetIndex(0), mNumEmittedTets(0)
		{ }

		void operator=(const Vox &v) 
		{
			mLocationX = v.mLocationX;
			mLocationY = v.mLocationY;
			mLocationZ = v.mLocationZ;
			mTets = v.mTets;
			mClusters = v.mClusters;
			mNodes = v.mNodes;
			mBaseTetIndex = v.mBaseTetIndex;
			mNumEmittedTets = v.mNumEmittedTets;
		}

		void initNodes(PxI32& nodeIndexer)
		{
			for (PxU32 i = 0; i < mClusters.size(); ++i)			
				mNodes.pushBack(VoxelNodes(nodeIndexer));			
		}

		void buildLocalTetAdjacency(const PxBoundedData& tetrahedra, const PxArray<PxI32>& indices, PxArray<PxI32>& result)
		{
			PxU32 l = 4 * indices.size();
			result.clear();
			result.resize(l, -1);

			PxHashMap<PxU64, PxI32> faces(indices.size());
			for (PxU32 i = 0; i < indices.size(); ++i)
			{
				Tetrahedron tet = tetrahedra.at<Tetrahedron>(indices[i]);
				if (tet[0] < 0)
					continue;

				tet.sort();
				for (PxI32 j = 0; j < 4; ++j)
				{
					const PxU64 tri = ((PxU64(tet[neighborFacesAscending[j][0]])) << 42) | ((PxU64(tet[neighborFacesAscending[j][1]])) << 21) | ((PxU64(tet[neighborFacesAscending[j][2]])));
					if (const PxPair<const PxU64, PxI32>* ptr = faces.find(tri))
					{
						result[4 * i + j] = ptr->second;
						result[ptr->second] = 4 * i + j;
						faces.erase(tri); //Keep memory low
					}
					else
						faces.insert(tri, 4 * i + j);
				}
			}
		}

		void computeClusters(const PxBoundedData& tetrahedra)
		{
			PxArray<PxI32> adj;
			buildLocalTetAdjacency(tetrahedra, mTets, adj);

			PxArray<bool> done;
			done.resize(mTets.size(), false);
			PxU32 start = 0;
			PxArray<PxI32> stack;
			while (true)
			{		
				stack.clear();
				while (start < done.size())
				{
					if (!done[start])
					{
						stack.pushBack(start);
						done[start] = true;
						PxArray<PxI32> c;
						c.pushBack(mTets[start]);
						mClusters.pushBack(c);
						break;
					}
					++start;
				}

				if (start == done.size())
					break;

				while (stack.size() > 0)
				{
					PxI32 id = stack.popBack();
					for (PxI32 i = 0; i < 4; ++i)
					{
						PxI32 a = adj[4 * id + i];
						PxI32 tetId = a >> 2;
						if (tetId >= 0 && !done[tetId])
						{
							stack.pushBack(tetId);
							done[tetId] = true;
							mClusters[mClusters.size() - 1].pushBack(mTets[tetId]);
						}
					}
				}
			}

#if PX_DEBUG
			if (mClusters.size() > 1)
			{
				PxI32 abc = 0;
				++abc;
			}
#endif

			for (PxU32 i = 0; i < mClusters.size(); ++i)			
				PxSort(mClusters[i].begin(), mClusters[i].size());			
		}

		void embed(PxArray<PxReal>& embeddingError, PxI32 id, const PxVec3& p, const PxU32 startIndex, const PxU32 endIndex, const Tetrahedron* voxelTets, const PxArray<PxVec3>& voxelPoints, PxI32* embeddings)
		{
			//PxVec4 best(1000, 1000, 1000, 1000);
			PxReal bestError = embeddingError[id];
			PxI32 bestIndex = -1;
			for (PxU32 i = startIndex; i < endIndex; ++i)
			{
				const Tetrahedron& candidate = voxelTets[i];
				
				PxVec4 bary;
				computeBarycentric(voxelPoints[candidate[0]], voxelPoints[candidate[1]], voxelPoints[candidate[2]], voxelPoints[candidate[3]], p, bary);
								
				const PxReal eps = 0;
				if ((bary.x >= -eps && bary.x <= 1.f + eps) && (bary.y >= -eps && bary.y <= 1.f + eps) &&
					(bary.z >= -eps && bary.z <= 1.f + eps) && (bary.w >= -eps && bary.w <= 1.f + eps))
				{
					embeddings[id] = i;
					embeddingError[id] = 0;
					return;
				}
				else
				{
					PxReal error = 0;
					PxReal min = PxMin(PxMin(bary.x, bary.y), PxMin(bary.z, bary.w));
					if (min < 0)
						error = -min;

					PxReal max = PxMax(PxMax(bary.x, bary.y), PxMax(bary.z, bary.w));
					if (max > 1)
					{
						PxReal e = max - 1;
						if (e > error)
							error = e;
					}

					if (error < bestError)
					{
						//best = bary;
						bestError = error;
						bestIndex = i;
					}
				}
			}

			if (bestIndex >= 0) 
			{
				embeddings[id] = bestIndex;
				embeddingError[id] = bestError;
			}
		}


		bool embed(const PxU32 anchorNodeIndex, const PxBoundedData& colTets, PxI32 numTetsPerVoxel, PxArray<PxReal>& embeddingError, PxI32 id, const PxVec3& p, const Tetrahedron* voxelTets, const PxArray<PxVec3>& voxelPoints, PxI32* embeddings)
		{
			if (mClusters.size() > 1)
			{
				for (PxU32 i = 0; i < mClusters.size(); ++i)
				{
					const PxArray<PxI32>& c = mClusters[i];
					for (PxU32 j = 0; j < c.size(); ++j)
					{
						const Tetrahedron& candidate = colTets.at<Tetrahedron>(c[j]);
						if (candidate.contains(anchorNodeIndex))
						{
							embed(embeddingError, id, p, mBaseTetIndex + i * numTetsPerVoxel, mBaseTetIndex + (i + 1) * numTetsPerVoxel, voxelTets, voxelPoints, embeddings);
							return true;
						}
					}
				}
				return false;
			}			
			
			embed(embeddingError, id, p, mBaseTetIndex, mBaseTetIndex + numTetsPerVoxel, voxelTets, voxelPoints, embeddings);
			return true;
		}



		void embed(const PxBoundedData& colPoints, const PxBoundedData& colTets, PxI32 numTetsPerVoxel, PxArray<PxReal>& embeddingError, PxI32 id, const PxVec3& p, const Tetrahedron* voxelTets, const PxArray<PxVec3>& voxelPoints, PxI32* embeddings)
		{
			PxReal bestError = embeddingError[id];
			PxI32 bestIndex = 0;
			if (mClusters.size() > 1) 
			{
				for (PxU32 i = 0; i < mClusters.size(); ++i)
				{
					const PxArray<PxI32>& c = mClusters[i];
					for (PxU32 j = 0; j < c.size(); ++j)
					{
						const Tetrahedron& candidate = colTets.at<Tetrahedron>(c[j]);

						PxVec4 bary;
						computeBarycentric(colPoints.at< PxVec3>(candidate[0]), colPoints.at<PxVec3>(candidate[1]), colPoints.at<PxVec3>(candidate[2]), colPoints.at<PxVec3>(candidate[3]), p, bary);

						const PxReal eps = 0;
						if ((bary.x >= -eps && bary.x <= 1.f + eps) && (bary.y >= -eps && bary.y <= 1.f + eps) &&
							(bary.z >= -eps && bary.z <= 1.f + eps) && (bary.w >= -eps && bary.w <= 1.f + eps))
						{
							embed(embeddingError, id, p, mBaseTetIndex + i * numTetsPerVoxel, mBaseTetIndex + (i + 1) * numTetsPerVoxel, voxelTets, voxelPoints, embeddings);
							return;
						}
						else
						{
							PxReal error = 0;
							PxReal min = PxMin(PxMin(bary.x, bary.y), PxMin(bary.z, bary.w));
							if (min < 0)
								error = -min;

							PxReal max = PxMax(PxMax(bary.x, bary.y), PxMax(bary.z, bary.w));
							if (max > 1)
							{
								PxReal e = max - 1;
								if (e > error)
									error = e;
							}

							if (error < bestError)
							{
								//best = bary;
								bestError = error;
								bestIndex = i;
							}
						}
					}
				}
			}

			if (bestIndex >= 0)
			{
				embed(embeddingError, id, p, mBaseTetIndex + bestIndex * numTetsPerVoxel, mBaseTetIndex + (bestIndex + 1) * numTetsPerVoxel, voxelTets, voxelPoints, embeddings);
			}
		}


		void emitTets(PxArray<PxU32>& voxelTets, PxArray<PxVec3>& voxelPoints, PxI32* embeddings,/* UnionFind uf,*/ const PxVec3& voxelBlockMin, const PxVec3& voxelSize,
			const PxBoundedData& inputPoints, const PxBoundedData& inputTets, PxArray<PxReal>& embeddingError, const PxI32& numTetsPerVoxel)
		{
			for (PxU32 i = 0; i < mNodes.size(); ++i)
			{
				const VoxelNodes& n = mNodes[i];
				for (PxI32 j = 0; j < 8; ++j)
				{
					PxI32 id = n.c[j];
					voxelPoints[id] = PxVec3(voxelBlockMin.x + (mLocationX + offsetsX[j]) * voxelSize.x,
						voxelBlockMin.y + (mLocationY + offsetsY[j]) * voxelSize.y,
						voxelBlockMin.z + (mLocationZ + offsetsZ[j]) * voxelSize.z);
				}

				//Emit 5 or 6 tets
				//if (numTetsPerVoxel == 5) {
				//	PxI32 flip = (mLocationX + mLocationY + mLocationZ) % 2;
				//	PxI32 offset = flip * 20;
				//	for (PxI32 j = 0; j < 20; j += 4)
				//	{
				//		PxI32 a = n.c[tets5PerVoxel[j + 0 + offset]];
				//		PxI32 b = n.c[tets5PerVoxel[j + 1 + offset]];
				//		PxI32 c = n.c[tets5PerVoxel[j + 2 + offset]];
				//		PxI32 d = n.c[tets5PerVoxel[j + 3 + offset]];
				//		//Tetrahedron tet(a, b, c, d);
				//		//voxelTets.pushBack(tet);
				//		voxelTets.pushBack(a);
				//		voxelTets.pushBack(b);
				//		voxelTets.pushBack(c);
				//		voxelTets.pushBack(d);
				//	}
				//}
				//else
				{					
					for (PxI32 j = 0; j < 24; j += 4)
					{
						PxI32 a = n.c[tets6PerVoxel[j + 0]];
						PxI32 b = n.c[tets6PerVoxel[j + 1]];
						PxI32 c = n.c[tets6PerVoxel[j + 2]];
						PxI32 d = n.c[tets6PerVoxel[j + 3]];
						//Tetrahedron tet(a, b, c, d);	
						//voxelTets.pushBack(tet);
						voxelTets.pushBack(a);
						voxelTets.pushBack(b);
						voxelTets.pushBack(c);
						voxelTets.pushBack(d);
					}
				}

				if (embeddings) 
				{
					PxVec3 min(voxelBlockMin.x + (mLocationX + 0) * voxelSize.x,
						voxelBlockMin.y + (mLocationY + 0) * voxelSize.y,
						voxelBlockMin.z + (mLocationZ + 0) * voxelSize.z);
					PxVec3 max(voxelBlockMin.x + (mLocationX + 1) * voxelSize.x,
						voxelBlockMin.y + (mLocationY + 1) * voxelSize.y,
						voxelBlockMin.z + (mLocationZ + 1) * voxelSize.z);
					PxBounds3 box(min, max);
					box.fattenFast(1e-5f);
					//Embedding
					const PxArray<PxI32>& cluster = mClusters[i];
					Tetrahedron* voxelTetPtr = reinterpret_cast<Tetrahedron*>(voxelTets.begin());
					for (PxU32 j = 0; j < cluster.size(); ++j)
					{
						const Tetrahedron& tet = inputTets.at<Tetrahedron>(cluster[j]);

						const PxU32 end = voxelTets.size() / 4;
						const PxU32 start = end - numTetsPerVoxel;

						for (PxU32 k = 0; k < 4; ++k)
							if (embeddingError[tet[k]] > 0 && box.contains(inputPoints.at<PxVec3>(tet[k])))
								embed(embeddingError, tet[k], inputPoints.at<PxVec3>(tet[k]), start, end, voxelTetPtr, voxelPoints, embeddings);
					}
				}
			}
		}
	
		void mapNodes(UnionFind& uf)
		{
			for (PxU32 i = 0; i < mNodes.size(); ++i)
			{
				VoxelNodes& n = mNodes[i];
				for (PxI32 j = 0; j < 8; ++j)
					n.c[j] = uf.getSetNr(n.c[j]);
				mNodes[i] = n;
			}
		}
	};


	struct Int3
	{
		PxU32 x;
		PxU32 y;
		PxU32 z;
	};

#if PX_LINUX
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#endif

	void getVoxelRange(const PxBoundedData& points, Tetrahedron tet, const PxVec3& voxelBlockMin, const PxVec3& voxelSize,
		PxU32 numVoxelsX, PxU32 numVoxelsY, PxU32 numVoxelsZ, Int3& min, Int3& max, PxBounds3& box, PxReal enlarge)
	{
		PxVec3 mi = points.at<PxVec3>(tet[0]);
		PxVec3 ma = mi;
		for (PxI32 i = 1; i < 4; ++i) {
			const PxVec3& p = points.at<PxVec3>(tet[i]);
			if (p.x < mi.x) mi.x = p.x; if (p.y < mi.y) mi.y = p.y; if (p.z < mi.z) mi.z = p.z;
			if (p.x > ma.x) ma.x = p.x; if (p.y > ma.y) ma.y = p.y; if (p.z > ma.z) ma.z = p.z;
		}
		mi.x -= enlarge;
		mi.y -= enlarge;
		mi.z -= enlarge;
		ma.x += enlarge;
		ma.y += enlarge;
		ma.z += enlarge;
		box.minimum = mi;
		box.maximum = ma;

		min.x = PxU32((mi.x - voxelBlockMin.x) / voxelSize.x);
		min.y = PxU32((mi.y - voxelBlockMin.y) / voxelSize.y);
		min.z = PxU32((mi.z - voxelBlockMin.z) / voxelSize.z);
		max.x = PxU32((ma.x - voxelBlockMin.x) / voxelSize.x);
		max.y = PxU32((ma.y - voxelBlockMin.y) / voxelSize.y);
		max.z = PxU32((ma.z - voxelBlockMin.z) / voxelSize.z);
		
		min.x = PxMax(0u, min.x);
		min.y = PxMax(0u, min.y);
		min.z = PxMax(0u, min.z);
		max.x = PxMin(numVoxelsX - 1, max.x);
		max.y = PxMin(numVoxelsY - 1, max.y);
		max.z = PxMin(numVoxelsZ - 1, max.z);
	}

#if PX_LINUX
#pragma GCC diagnostic pop
#endif

	PX_FORCE_INLINE void connect(const VoxelNodes& voxelNodesA, const PxI32* faceVoxelA, const VoxelNodes& voxelNodesB, const PxI32* faceVoxelB, UnionFind& uf)
	{
		for (PxI32 i = 0; i < 4; ++i)
			uf.makeSet(voxelNodesA.c[faceVoxelA[i]], voxelNodesB.c[faceVoxelB[i]]);
	}

	PX_FORCE_INLINE bool intersectionOfSortedListsNotEmpty(const PxArray<PxI32>& sorted1, const PxArray<PxI32>& sorted2)
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

	void connect(const Vox& voxelA, const PxI32* faceVoxelA, const Vox& voxelB, const PxI32* faceVoxelB, UnionFind& uf)
	{
		for (PxU32 i = 0; i < voxelA.mClusters.size(); ++i)
		{
			const PxArray<PxI32>& clusterA = voxelA.mClusters[i];
			for (PxU32 j = 0; j < voxelB.mClusters.size(); ++j)
			{
				const PxArray<PxI32>& clusterB = voxelB.mClusters[j];
				if (intersectionOfSortedListsNotEmpty(clusterA, clusterB))
					connect(voxelA.mNodes[i], faceVoxelA, voxelB.mNodes[j], faceVoxelB, uf);				
			}
		}
	}

	
	PX_FORCE_INLINE PxU32 cell(PxU32 x, PxU32 y, PxU32 z, PxU32 numX, PxU32 numXtimesNumY)
	{
		return x + y * numX + z * numXtimesNumY;
	}

	Int3 getCell(const PxVec3& p, const PxVec3& voxelBlockMin, const PxVec3& voxelSize, const PxU32 numVoxelsX, const PxU32 numVoxelsY, const PxU32 numVoxelsZ)
	{
		Int3 cell;
		PxVec3 c = p - voxelBlockMin;
		cell.x = PxU32(c.x / voxelSize.x);
		cell.y = PxU32(c.y / voxelSize.y);
		cell.z = PxU32(c.z / voxelSize.z);
		if (cell.x >= numVoxelsX) cell.x = numVoxelsX - 1;
		if (cell.y >= numVoxelsY) cell.y = numVoxelsY - 1;
		if (cell.z >= numVoxelsZ) cell.z = numVoxelsZ - 1;
		return cell;
	}

	PX_FORCE_INLINE PxReal aabbDistanceSquaredToPoint(const PxVec3& min, const PxVec3& max, const PxVec3& p)
	{
		PxReal sqDist = 0.0f;

		if (p.x < min.x) sqDist += (min.x - p.x) * (min.x - p.x);
		if (p.x > max.x) sqDist += (p.x - max.x) * (p.x - max.x);

		if (p.y < min.y) sqDist += (min.y - p.y) * (min.y - p.y);
		if (p.y > max.y) sqDist += (p.y - max.y) * (p.y - max.y);

		if (p.z < min.z) sqDist += (min.z - p.z) * (min.z - p.z);
		if (p.z > max.z) sqDist += (p.z - max.z) * (p.z - max.z);

		return sqDist;
	}

	PxI32 getVoxelId(PxArray<PxI32> voxelIds, const PxVec3& p, const PxVec3& voxelBlockMin, const PxVec3& voxelSize, const PxU32 numVoxelsX, const PxU32 numVoxelsY, const PxU32 numVoxelsZ)
	{
		PxVec3 pt = p - voxelBlockMin;		
		
		PxU32 ix = PxU32(PxMax(0.0f, pt.x / voxelSize.x));
		PxU32 iy = PxU32(PxMax(0.0f, pt.y / voxelSize.y));
		PxU32 iz = PxU32(PxMax(0.0f, pt.z / voxelSize.z));		
		if (ix >= numVoxelsX) ix = numVoxelsX - 1;
		if (iy >= numVoxelsY) iy = numVoxelsY - 1;
		if (iz >= numVoxelsZ) iz = numVoxelsZ - 1;

		PxU32 id = cell(ix, iy, iz, numVoxelsX, numVoxelsX * numVoxelsY);
		if (voxelIds[id] >= 0)
			return voxelIds[id];

		const PxI32 numOffsets = 6;
		const PxI32 offsets[numOffsets][3] = { {-1, 0, 0}, {+1, 0, 0}, {0, -1, 0}, {0, +1, 0}, {0, 0, -1}, {0, 0, +1} };

		PxReal minDist = FLT_MAX;
		for (PxU32 i = 0; i < numOffsets; ++i) 
		{
			const PxI32* o = offsets[i];
			PxI32 newX = ix + o[0];
			PxI32 newY = iy + o[1];
			PxI32 newZ = iz + o[2];
			if (newX >= 0 && newX < PxI32(numVoxelsX) && newY >= 0 && newY < PxI32(numVoxelsY) && newZ >= 0 && newZ < PxI32(numVoxelsZ))
			{
				PxU32 candidate = cell(newX, newY, newZ, numVoxelsX, numVoxelsX * numVoxelsY);
				if (voxelIds[candidate] >= 0)
				{
					PxVec3 min(voxelBlockMin.x + newX * voxelSize.x, voxelBlockMin.y + newY * voxelSize.y, voxelBlockMin.z + newZ * voxelSize.z);
					PxVec3 max = min + voxelSize;
					PxReal d = aabbDistanceSquaredToPoint(min, max, p);
					if (d < minDist)
					{
						id = candidate;
						minDist = d;
					}
				}
			}
		}

		PxI32 result = voxelIds[id];		

		if (result < 0)
		{
			//Search the closest voxel over all voxels
			minDist = FLT_MAX;
			for (PxU32 newX = 0; newX < numVoxelsX; ++newX)
				for (PxU32 newY = 0; newY < numVoxelsY; ++newY)
					for (PxU32 newZ = 0; newZ < numVoxelsZ; ++newZ)
					{
						PxU32 candidate = cell(newX, newY, newZ, numVoxelsX, numVoxelsX * numVoxelsY);
						if (voxelIds[candidate] >= 0)
						{
							PxVec3 min(voxelBlockMin.x + newX * voxelSize.x, voxelBlockMin.y + newY * voxelSize.y, voxelBlockMin.z + newZ * voxelSize.z);
							PxVec3 max = min + voxelSize;
							PxReal d = aabbDistanceSquaredToPoint(min, max, p);
							if (d < minDist)
							{
								id = candidate;
								minDist = d;
							}
						}
					}
		}
		result = voxelIds[id];

		if (result < 0)
			return 0;

		return result;
	}

	void generateVoxelTetmesh(const PxBoundedData& inputPointsOrig, const PxBoundedData& inputTets, PxU32 numVoxelsX, PxU32 numVoxelsY, PxU32 numVoxelsZ,
		PxArray<PxVec3>& voxelPoints, PxArray<PxU32>& voxelTets, PxI32* intputPointToOutputTetIndex, const PxU32* anchorNodeIndices)
	{
		if (inputTets.count == 0)
			return; //No input, so there is no basis for creating an output

		PxU32 xy = numVoxelsX * numVoxelsY;
		
		PxVec3 origMin, origMax;
		minMax(inputPointsOrig, origMin, origMax);
		PxVec3 size = origMax - origMin;
		PxReal scaling = 1.0f / PxMax(size.x, PxMax(size.y, size.z));
		PxArray<PxVec3> scaledPoints;
		scaledPoints.resize(inputPointsOrig.count);
		for (PxU32 i = 0; i < inputPointsOrig.count; ++i)
		{
			PxVec3 p = inputPointsOrig.at<PxVec3>(i);
			scaledPoints[i] = (p - origMin) * scaling;			
		}

		PxBoundedData inputPoints;
		inputPoints.count = inputPointsOrig.count;
		inputPoints.stride = sizeof(PxVec3);
		inputPoints.data = scaledPoints.begin();

		PxVec3 voxelBlockMin, voxelBlockMax;
		minMax(inputPoints, voxelBlockMin, voxelBlockMax);
				
		PxVec3 voxelBlockSize = voxelBlockMax - voxelBlockMin;
		PxVec3 voxelSize(voxelBlockSize.x / numVoxelsX, voxelBlockSize.y / numVoxelsY, voxelBlockSize.z / numVoxelsZ);

		PxArray<PxI32> voxelIds;
		voxelIds.resize(numVoxelsX * numVoxelsY * numVoxelsZ, -1);
		
		PxArray<Vox> voxels;

		for(PxU32 i=0;i< inputTets.count;++i)
		{
			Int3 min, max;
			const Tetrahedron& tet = inputTets.at<Tetrahedron>(i);
			PxBounds3 tetBox;
			getVoxelRange(inputPoints, tet, voxelBlockMin, voxelSize, numVoxelsX, numVoxelsY, numVoxelsZ, min, max, tetBox, 1.e-4f);

			//bool success = false;
			for (PxU32 x = min.x; x <= max.x; ++x)
			{
				for (PxU32 y = min.y; y <= max.y; ++y)
				{
					for (PxU32 z = min.z; z <= max.z; ++z)
					{
						PxBounds3 box(PxVec3(voxelBlockMin.x + x * voxelSize.x, voxelBlockMin.y + y * voxelSize.y, voxelBlockMin.z + z * voxelSize.z),
							PxVec3(voxelBlockMin.x + (x + 1) * voxelSize.x, voxelBlockMin.y + (y + 1) * voxelSize.y, voxelBlockMin.z + (z + 1) * voxelSize.z));
						
						if (intersectTetrahedronBox(inputPoints.at<PxVec3>(tet[0]), inputPoints.at<PxVec3>(tet[1]), inputPoints.at<PxVec3>(tet[2]), inputPoints.at<PxVec3>(tet[3]), box))
						{
							//success = true;
							PxI32 voxelId = voxelIds[cell(x, y, z, numVoxelsX, xy)];
							if (voxelId < 0)
							{
								voxelId = voxels.size();
								voxelIds[cell(x, y, z, numVoxelsX, xy)] = voxelId;
								voxels.pushBack(Vox(x, y, z));
							}
							Vox& v = voxels[voxelId];
							v.mTets.pushBack(i);							
						}
					}
				}
			}
			/*if (!success)
			{
				PxI32 abc = 0;
				++abc;
			}*/
		}

		PxI32 nodeIndexer = 0;
		for(PxU32 i=0;i<voxels.size();++i)
		{
			voxels[i].computeClusters(inputTets);
		}

		for (PxU32 i = 0; i < voxels.size(); ++i)
		{
			voxels[i].initNodes(nodeIndexer);
		}

		UnionFind uf(nodeIndexer);
		for (PxU32 i = 0; i < voxels.size(); ++i)
		{
			Vox& v = voxels[i];
			if (v.mLocationX > 0 && voxelIds[cell(v.mLocationX - 1, v.mLocationY, v.mLocationZ, numVoxelsX, xy)] >= 0)
				connect(voxels[voxelIds[cell(v.mLocationX - 1, v.mLocationY, v.mLocationZ, numVoxelsX, xy)]], xPosFace, v, xNegFace, uf);
			if (v.mLocationX < numVoxelsX - 1 && voxelIds[cell(v.mLocationX + 1, v.mLocationY, v.mLocationZ, numVoxelsX, xy)] >= 0)
				connect(v, xPosFace, voxels[voxelIds[cell(v.mLocationX + 1, v.mLocationY, v.mLocationZ, numVoxelsX, xy)]], xNegFace, uf);

			if (v.mLocationY > 0 && voxelIds[cell(v.mLocationX, v.mLocationY - 1, v.mLocationZ, numVoxelsX, xy)] >= 0)
				connect(voxels[voxelIds[cell(v.mLocationX, v.mLocationY - 1, v.mLocationZ, numVoxelsX, xy)]], yPosFace, v, yNegFace, uf);
			if (v.mLocationY < numVoxelsY - 1 && voxelIds[cell(v.mLocationX, v.mLocationY + 1, v.mLocationZ, numVoxelsX, xy)] >= 0)
				connect(v, yPosFace, voxels[voxelIds[cell(v.mLocationX, v.mLocationY + 1, v.mLocationZ, numVoxelsX, xy)]], yNegFace, uf);

			if (v.mLocationZ > 0 && voxelIds[cell(v.mLocationX, v.mLocationY, v.mLocationZ - 1, numVoxelsX, xy)] >= 0)
				connect(voxels[voxelIds[cell(v.mLocationX, v.mLocationY, v.mLocationZ - 1, numVoxelsX, xy)]], zPosFace, v, zNegFace, uf);
			if (v.mLocationZ < numVoxelsZ - 1 && voxelIds[cell(v.mLocationX, v.mLocationY, v.mLocationZ + 1, numVoxelsX, xy)] >= 0)
				connect(v, zPosFace, voxels[voxelIds[cell(v.mLocationX, v.mLocationY, v.mLocationZ + 1, numVoxelsX, xy)]], zNegFace, uf);
		}

		PxI32 numVertices = uf.computeSetNrs();

		for (PxU32 i = 0; i < voxels.size(); ++i)
		{
			voxels[i].mapNodes(uf);
		}

		const PxU32 numTetsPerVoxel = 6;
		voxelPoints.resize(numVertices);
		//intputPointToOutputTetIndex.resize(numInputPoints);
		voxelTets.clear();
		PxArray<PxReal> embeddingError;
		embeddingError.resize(inputPoints.count, 1000);
		for (PxU32 i = 0; i < voxels.size(); ++i)
		{
			Vox& v = voxels[i];
			v.mBaseTetIndex = voxelTets.size() / 4;
			v.emitTets(voxelTets, voxelPoints, intputPointToOutputTetIndex, voxelBlockMin, voxelSize, inputPoints, inputTets, embeddingError, numTetsPerVoxel);
			v.mNumEmittedTets = voxelTets.size() / 4 - v.mBaseTetIndex;
		}

#if PX_DEBUG
		PxArray<bool> pointUsed;
		pointUsed.resize(inputPoints.count, false);
		for (PxU32 i = 0; i < inputTets.count; ++i)
		{
			const Tetrahedron& tet = inputTets.at<Tetrahedron>(i);
			for (PxU32 j = 0; j < 4; ++j)
				pointUsed[tet[j]] = true;
		}
#endif
		Tetrahedron* voxelTetPtr = reinterpret_cast<Tetrahedron*>(voxelTets.begin());
		for (PxU32 i = 0; i < embeddingError.size(); ++i)
		{
			if (embeddingError[i] == 1000)
			{
				const PxVec3& p = inputPoints.at<PxVec3>(i);
				
				PxI32 voxelId = getVoxelId(voxelIds, p, voxelBlockMin, voxelSize, numVoxelsX, numVoxelsY, numVoxelsZ);
				PX_ASSERT(voxelId >= 0);

				Vox& vox = voxels[voxelId];

				if (anchorNodeIndices && anchorNodeIndices[i] < inputPoints.count)
				{
					if (!vox.embed(anchorNodeIndices[i], inputTets, numTetsPerVoxel, embeddingError, i, p, voxelTetPtr, voxelPoints, intputPointToOutputTetIndex))
					{
						PxVec3 pt = inputPoints.at<PxVec3>(anchorNodeIndices[i]);

						voxelId = getVoxelId(voxelIds, pt, voxelBlockMin, voxelSize, numVoxelsX, numVoxelsY, numVoxelsZ);
						PX_ASSERT(voxelId >= 0);

						Vox& v = voxels[voxelId];
						if (!v.embed(anchorNodeIndices[i], inputTets, numTetsPerVoxel, embeddingError, i, p, voxelTetPtr, voxelPoints, intputPointToOutputTetIndex))
							v.embed(inputPoints, inputTets, numTetsPerVoxel, embeddingError, i, p, voxelTetPtr, voxelPoints, intputPointToOutputTetIndex);
					}
				}
				else
					vox.embed(inputPoints, inputTets, numTetsPerVoxel, embeddingError, i, p, voxelTetPtr, voxelPoints, intputPointToOutputTetIndex);
			}
		}

		//Scale back to original size
		scaling = 1.0f / scaling;
		for(PxU32 i=0;i<voxelPoints.size();++i)
			voxelPoints[i] = voxelPoints[i] * scaling + origMin;
	}	

	void generateVoxelTetmesh(const PxBoundedData& inputPoints, const PxBoundedData& inputTets, PxReal voxelEdgeLength,
		PxArray<PxVec3>& voxelPoints, PxArray<PxU32>& voxelTets, PxI32* intputPointToOutputTetIndex, const PxU32* anchorNodeIndices)
	{
		PxVec3 min, max;
		minMax(inputPoints, min, max);
		PxVec3 blockSize = max - min;
		PxU32 numCellsX = PxMax(1u, PxU32(blockSize.x / voxelEdgeLength + 0.5f));
		PxU32 numCellsY = PxMax(1u, PxU32(blockSize.y / voxelEdgeLength + 0.5f));
		PxU32 numCellsZ = PxMax(1u, PxU32(blockSize.z / voxelEdgeLength + 0.5f));

		generateVoxelTetmesh(inputPoints, inputTets, numCellsX, numCellsY, numCellsZ, voxelPoints, voxelTets, intputPointToOutputTetIndex, anchorNodeIndices);
	}

	void generateVoxelTetmesh(const PxBoundedData& inputPoints, const PxBoundedData& inputTets, PxU32 numVoxelsAlongLongestBoundingBoxAxis,
		PxArray<PxVec3>& voxelPoints, PxArray<PxU32>& voxelTets, PxI32* intputPointToOutputTetIndex, const PxU32* anchorNodeIndices)
	{
		PxVec3 min, max;
		minMax(inputPoints, min, max);	
		PxVec3 size = max - min;
		PxReal voxelEdgeLength = PxMax(size.x, PxMax(size.y, size.z)) / numVoxelsAlongLongestBoundingBoxAxis;
		generateVoxelTetmesh(inputPoints, inputTets, voxelEdgeLength, voxelPoints, voxelTets, intputPointToOutputTetIndex, anchorNodeIndices);
	}


	
	static PxReal computeMeshVolume(const PxArray<PxVec3>& points, const PxArray<Triangle>& triangles)
	{
		PxVec3 center(0, 0, 0);
		for (PxU32 i = 0; i < points.size(); ++i)
		{
			center += points[i];
		}
		center /= PxReal(points.size());

		PxReal volume = 0;
		for (PxU32 i = 0; i < triangles.size(); ++i)
		{
			const Triangle& tri = triangles[i];
			volume += tetVolume(points[tri[0]], points[tri[1]], points[tri[2]], center);
		}

		return PxAbs(volume);
	}


	static PxTriangleMeshAnalysisResults validateConnectivity(const PxArray<Triangle>& triangles)
	{
		PxArray<bool> flip;
		PxHashMap<PxU64, PxI32> edges;
		PxArray<PxArray<PxU32>> connectedTriangleGroups;
		if (!MeshAnalyzer::buildConsistentTriangleOrientationMap(triangles.begin(), triangles.size(), flip, edges, connectedTriangleGroups))
			return PxTriangleMeshAnalysisResult::Enum::eEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES;

		PxTriangleMeshAnalysisResults result = PxTriangleMeshAnalysisResult::eVALID;
		for (PxHashMap<PxU64, PxI32>::Iterator iter = edges.getIterator(); !iter.done(); ++iter)
			if (iter->second >= 0) 
			{
				result = result | PxTriangleMeshAnalysisResult::eOPEN_BOUNDARIES;
				break;
			}

		for (PxU32 i = 0; i < flip.size(); ++i)
		{
			if (flip[i])
			{
				return result | PxTriangleMeshAnalysisResult::Enum::eINCONSISTENT_TRIANGLE_ORIENTATION;
			}
		}
		return result;
	}


	static bool trianglesIntersect(const Triangle& tri1, const Triangle& tri2, const PxArray<PxVec3>& points)
	{
		int counter = 0;
		if (tri1.contains(tri2[0])) ++counter;
		if (tri1.contains(tri2[1])) ++counter;
		if (tri1.contains(tri2[2])) ++counter;

		if (counter > 0)
			return false; //Triangles share at leat one point

		return Gu::trianglesIntersect(points[tri1[0]], points[tri1[1]], points[tri1[2]], points[tri2[0]], points[tri2[1]], points[tri2[2]]);
	}

	static PxBounds3 triBounds(const PxArray<PxVec3>& points, const Triangle& tri, PxReal enlargement)
	{
		PxBounds3 box = PxBounds3::empty();
		box.include(points[tri[0]]);
		box.include(points[tri[1]]);
		box.include(points[tri[2]]);
		box.fattenFast(enlargement);
		return box;
	}

	static bool meshContainsSelfIntersections(const PxArray<PxVec3>& points, const PxArray<Triangle>& triangles)
	{
		PxReal enlargement = 1e-6f;
		AABBTreeBounds boxes;
		boxes.init(triangles.size());
		for (PxU32 i = 0; i < triangles.size(); ++i)
		{
			boxes.getBounds()[i] = triBounds(points, triangles[i], enlargement);
		}

		PxArray<Gu::BVHNode> tree;
		Gu::buildAABBTree(triangles.size(), boxes, tree);

		PxArray<PxI32> candidateTriangleIndices;
		IntersectionCollectingTraversalController tc(candidateTriangleIndices);

		for (PxU32 i = 0; i < triangles.size(); ++i)
		{
			const Triangle& tri = triangles[i];
			PxBounds3 box = triBounds(points, tri, enlargement);
			tc.reset(box);

			traverseBVH(tree, tc);

			for (PxU32 j = 0; j < candidateTriangleIndices.size(); ++j)
			{
				Triangle tri2 = triangles[j];
				if (trianglesIntersect(tri, tri2, points))
					return true;
			}
		}
		return false;
	}

	static PxReal maxDotProduct(const PxVec3& a, const PxVec3& b, const PxVec3& c)
	{
		const PxVec3 ab = b - a;
		const PxVec3 ac = c - a;
		const PxVec3 bc = c - b;

		PxReal maxDot = ab.dot(ac) / PxSqrt(ab.magnitudeSquared() * ac.magnitudeSquared());
		PxReal dot = ac.dot(bc) / PxSqrt(ac.magnitudeSquared() * bc.magnitudeSquared());
		if (dot > maxDot) maxDot = dot;
		dot = -ab.dot(bc) / PxSqrt(ab.magnitudeSquared() * bc.magnitudeSquared());
		if (dot > maxDot) maxDot = dot;

		return maxDot;
	}

	static PxReal minimumAngle(const PxArray<PxVec3>& points, const PxArray<Triangle>& triangles)
	{
		PxReal maxDot = -1;
		for (PxU32 i = 0; i < triangles.size(); ++i)
		{
			const Triangle& tri = triangles[i];
			PxReal dot = maxDotProduct(points[tri[0]], points[tri[1]], points[tri[2]]);
			if (dot > maxDot)
				maxDot = dot;
		}
		if (maxDot > 1)
			maxDot = 1;

		return PxAcos(maxDot); //Converts to the minimal angle
	}

	PxTetrahedronMeshAnalysisResults validateTetrahedronMesh(const PxBoundedData& points, const PxBoundedData& tetrahedra, const bool has16BitIndices, const PxReal minTetVolumeThreshold)
	{
		PxTetrahedronMeshAnalysisResults result = PxTetrahedronMeshAnalysisResult::eVALID;

		PxArray<Tetrahedron> tets;
		tets.reserve(tetrahedra.count);

		for (PxU32 i = 0; i < tetrahedra.count; ++i)
		{
			Tetrahedron t;
			if (has16BitIndices)
			{
				Tetrahedron16 t16 = tetrahedra.at<Tetrahedron16>(i);
				t = Tetrahedron(t16[0], t16[1], t16[2], t16[3]);
			}
			else
				t = tetrahedra.at<Tetrahedron>(i);

			tets.pushBack(t);
		}

		for (PxU32 i = 0; i < tetrahedra.count; ++i)
		{
			const Tetrahedron& tetInd = tets[i];
			const PxReal volume = computeTetrahedronVolume(points.at<PxVec3>(tetInd.v[0]), points.at<PxVec3>(tetInd.v[1]), points.at<PxVec3>(tetInd.v[2]), points.at<PxVec3>(tetInd.v[3]));
			if (volume <= minTetVolumeThreshold)
			{
				result |= PxTetrahedronMeshAnalysisResult::eDEGENERATE_TETRAHEDRON;
				break;
			}
		}

		if (result & PxTetrahedronMeshAnalysisResult::eDEGENERATE_TETRAHEDRON) result |= PxTetrahedronMeshAnalysisResult::eMESH_IS_INVALID;

		return result;
	}

	PxTriangleMeshAnalysisResults validateTriangleMesh(const PxBoundedData& points, const PxBoundedData& triangles, const bool has16BitIndices, const PxReal minVolumeThreshold, const PxReal minTriangleAngleRadians)
	{
		PxVec3 min, max;
		minMax(points, min, max);
		PxVec3 size = max - min;
		PxReal scaling = 1.0f / PxMax(PxMax(size.x, size.y), PxMax(1e-6f, size.z));

		PxArray<PxVec3> normalizedPoints;
		normalizedPoints.reserve(points.count);

		PxTriangleMeshAnalysisResults result = PxTriangleMeshAnalysisResult::eVALID;

		if (has16BitIndices && points.count > PX_MAX_U16)
			result |= PxTriangleMeshAnalysisResult::eREQUIRES_32BIT_INDEX_BUFFER;

		for (PxU32 i = 0; i < points.count; ++i)
		{
			const PxVec3& p = points.at<PxVec3>(i);
			if (!PxIsFinite(p.x) || !PxIsFinite(p.y) || !PxIsFinite(p.z))
			{
				result |= PxTriangleMeshAnalysisResult::eCONTAINS_INVALID_POINTS;
				normalizedPoints.pushBack(p);
				continue;
			}
			normalizedPoints.pushBack((p - min) * scaling);
		}


		PxArray<PxI32> map;
		MeshAnalyzer::mapDuplicatePoints<PxVec3, PxF32>(normalizedPoints.begin(), normalizedPoints.size(), map);
		PxArray<Triangle> mappedTriangles;
		mappedTriangles.reserve(triangles.count);
		for (PxU32 i = 0; i < triangles.count; ++i)
		{
			Triangle t;
			if (has16BitIndices)
			{
				Triangle16 t16 = triangles.at<Triangle16>(i);
				t = Triangle(t16[0], t16[1], t16[2]);
			}
			else
				t = triangles.at<Triangle>(i);
		
			mappedTriangles.pushBack(t);
		}


		for (PxU32 i = 0; i < map.size(); ++i)
			if (map[i] != PxI32(i))
			{
				result = result | PxTriangleMeshAnalysisResult::eCONTAINS_DUPLICATE_POINTS;
				break;
			}

		if(minimumAngle(normalizedPoints, mappedTriangles) < minTriangleAngleRadians)
			result = result | PxTriangleMeshAnalysisResult::eCONTAINS_ACUTE_ANGLED_TRIANGLES;
		
		PxReal volume = computeMeshVolume(normalizedPoints, mappedTriangles);
		if (volume < minVolumeThreshold)
			result = result | PxTriangleMeshAnalysisResult::eZERO_VOLUME;

		result = result | validateConnectivity(mappedTriangles);

		if(meshContainsSelfIntersections(normalizedPoints, mappedTriangles))
			result = result | PxTriangleMeshAnalysisResult::eSELF_INTERSECTIONS;

		if (result & PxTriangleMeshAnalysisResult::eZERO_VOLUME) result |= PxTriangleMeshAnalysisResult::eMESH_IS_INVALID;
		if (result & PxTriangleMeshAnalysisResult::eOPEN_BOUNDARIES) result |= PxTriangleMeshAnalysisResult::eMESH_IS_PROBLEMATIC;
		if (result & PxTriangleMeshAnalysisResult::eSELF_INTERSECTIONS) result |= PxTriangleMeshAnalysisResult::eMESH_IS_PROBLEMATIC;
		if (result & PxTriangleMeshAnalysisResult::eINCONSISTENT_TRIANGLE_ORIENTATION) result |= PxTriangleMeshAnalysisResult::eMESH_IS_INVALID;
		if (result & PxTriangleMeshAnalysisResult::eCONTAINS_ACUTE_ANGLED_TRIANGLES) result |= PxTriangleMeshAnalysisResult::eMESH_IS_PROBLEMATIC;
		if (result & PxTriangleMeshAnalysisResult::eEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES) result |= PxTriangleMeshAnalysisResult::eMESH_IS_PROBLEMATIC;
		if (result & PxTriangleMeshAnalysisResult::eCONTAINS_INVALID_POINTS) result |= PxTriangleMeshAnalysisResult::eMESH_IS_INVALID;
		if (result & PxTriangleMeshAnalysisResult::eREQUIRES_32BIT_INDEX_BUFFER) result |= PxTriangleMeshAnalysisResult::eMESH_IS_INVALID;

		return result;
	}
	
	//Consistent triangle orientation
}
}
