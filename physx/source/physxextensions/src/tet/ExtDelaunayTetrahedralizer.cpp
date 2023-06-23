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

#include "ExtDelaunayTetrahedralizer.h"
#include "foundation/PxSort.h"

#include "foundation/PxHashMap.h"
#include "ExtUtilities.h"


namespace physx
{
namespace Ext
{
	using Triangle = Gu::IndexedTriangleT<PxI32>;

	//http://tizian.cs.uni-bonn.de/publications/BaudsonKlein.pdf Page 44
	static const PxI32 neighborFaces[4][3] = { { 0, 1, 2 }, { 0, 3, 1 }, { 0, 2, 3 }, { 1, 3, 2 } };
	static const PxI32 tetTip[4] = { 3, 2, 1, 0 };

	struct Plane
	{
		PxVec3d normal;
		PxF64 planeD;

		PX_FORCE_INLINE Plane(const PxVec3d& n, PxF64 d) : normal(n), planeD(d)
		{ }

		PX_FORCE_INLINE Plane(const PxVec3d& n, const PxVec3d& pointOnPlane) : normal(n)
		{
			planeD = -pointOnPlane.dot(normal);
		}

		PX_FORCE_INLINE Plane(const PxVec3d& a, const PxVec3d& b, const PxVec3d& c)
		{
			normal = (b - a).cross(c - a);
			PxF64 norm = normal.magnitude();
			normal /= norm;

			planeD = -a.dot(normal);
		}

		PX_FORCE_INLINE PxF64 signedDistance(const PxVec3d& p)
		{
			return normal.dot(p) + planeD;
		}

		PX_FORCE_INLINE PxF64 unsignedDistance(const PxVec3d& p)
		{
			return PxAbs(signedDistance(p));
		}
	};

	DelaunayTetrahedralizer::DelaunayTetrahedralizer(const PxVec3d& min, const PxVec3d& max)
	{
		for(PxI32 i = 0; i < 4; ++i)
			neighbors.pushBack(-1);

		PxF64 radius = 1.1 * 0.5 * (max - min).magnitude();

		PxF64 scaledRadius = 6 * radius;
		centeredNormalizedPoints.pushBack(PxVec3d(-scaledRadius, -scaledRadius, -scaledRadius));
		centeredNormalizedPoints.pushBack(PxVec3d(scaledRadius, scaledRadius, -scaledRadius));
		centeredNormalizedPoints.pushBack(PxVec3d(-scaledRadius, scaledRadius, scaledRadius));
		centeredNormalizedPoints.pushBack(PxVec3d(scaledRadius, -scaledRadius, scaledRadius));

		numAdditionalPointsAtBeginning = 4;

		result.pushBack(Tetrahedron(0, 1, 2, 3));	
		for (PxI32 i = 0; i < 4; ++i)
			vertexToTet.pushBack(0);
	}

	void buildNeighborhood(const PxI32* tets, PxU32 numTets, PxArray<PxI32>& result)
	{
		PxU32 l = 4 * numTets;
		result.clear();
		result.resize(l, -1);

		PxHashMap<SortedTriangle, PxI32, TriangleHash> faces;
		for (PxU32 i = 0; i < numTets; ++i)
		{
			const PxI32* tet = &tets[4 * i];
			if (tet[0] < 0)
				continue;

			for (PxI32 j = 0; j < 4; ++j)
			{
				SortedTriangle tri(tet[neighborFaces[j][0]], tet[neighborFaces[j][1]], tet[neighborFaces[j][2]]);
				if (const PxPair<const SortedTriangle, PxI32>* ptr = faces.find(tri))
				{
					if (ptr->second < 0) 
					{
						//PX_ASSERT(false); //Invalid tetmesh since a face is shared by more than 2 tetrahedra
						continue;
					}

					result[4 * i + j] = ptr->second;
					result[ptr->second] = 4 * i + j;

					faces[tri] = -1;
				}
				else
					faces.insert(tri, 4 * i + j);
			}
		}
	}

	void buildNeighborhood(const PxArray<Tetrahedron>& tets, PxArray<PxI32>& result)
	{
		buildNeighborhood(reinterpret_cast<const PxI32*>(tets.begin()), tets.size(), result);
	}

	bool shareFace(const Tetrahedron& t1, const Tetrahedron& t2)
	{
		PxI32 counter = 0;

		if (t1.contains(t2[0])) ++counter;
		if (t1.contains(t2[1])) ++counter;
		if (t1.contains(t2[2])) ++counter;
		if (t1.contains(t2[3])) ++counter;

		if (counter == 4)
			PX_ASSERT(false);
		return counter == 3;
	}

	//Keep for debugging & verification
	void validateNeighborhood(const PxArray<Tetrahedron>& tets, PxArray<PxI32>& neighbors)
	{
		PxI32 borderCounter = 0;
		for (PxU32 i = 0; i < neighbors.size(); ++i)
			if (neighbors[i] == -1)
				++borderCounter;

		//if (borderCounter != 4)
		//    throw new Exception();


		PxArray<PxI32> n;
		buildNeighborhood(tets, n);

		for (PxU32 i = 0; i < n.size(); ++i)
		{
			if (n[i] < 0)
				continue;
			if (n[i] != neighbors[i])			
				PX_ASSERT(false);			
		}

		for (PxU32 i = 0; i < neighbors.size(); ++i)
		{
			if (neighbors[i] < 0)
				continue;

			Tetrahedron tet1 = tets[i >> 2];
			Tetrahedron tet2 = tets[neighbors[i] >> 2];

			if (!shareFace(tet1, tet2))
				PX_ASSERT(false);

			PxI32 face1 = i & 3;
			SortedTriangle tri1(tet1[neighborFaces[face1][0]], tet1[neighborFaces[face1][1]], tet1[neighborFaces[face1][2]]);

			PxI32 face2 = neighbors[i] & 3;
			SortedTriangle tri2(tet2[neighborFaces[face2][0]], tet2[neighborFaces[face2][1]], tet2[neighborFaces[face2][2]]);

			if (tri1.A != tri2.A || tri1.B != tri2.B || tri1.C != tri2.C)
				PX_ASSERT(false);
		}
	}

	void buildVertexToTet(const PxArray<Tetrahedron>& tets, PxI32 numPoints, PxArray<PxI32>& result)
	{
		result.clear();
		result.resize(numPoints, -1);	

		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			const Tetrahedron& tet = tets[i];
			if (tet[0] < 0)
				continue;
			result[tet[0]] = i;
			result[tet[1]] = i;
			result[tet[2]] = i;
			result[tet[3]] = i;
		}
	}

	DelaunayTetrahedralizer::DelaunayTetrahedralizer(PxArray<PxVec3d>& points, PxArray<Tetrahedron>& tets)
	{
		initialize(points, tets);
	}

	void DelaunayTetrahedralizer::initialize(PxArray<PxVec3d>& points, PxArray<Tetrahedron>& tets)
	{
		clearLockedEdges();
		clearLockedTriangles();

		centeredNormalizedPoints = points;
		result = tets;

		numAdditionalPointsAtBeginning = 0;

		buildNeighborhood(tets, neighbors);
		buildVertexToTet(tets, points.size(), vertexToTet);

		for (PxU32 i = 0; i < tets.size(); ++i)
			if (tets[i][0] < 0)
				unusedTets.pushBack(i);
	}

	PX_FORCE_INLINE PxF64 inSphere(const PxVec3d& pa, const PxVec3d& pb, const PxVec3d& pc, const PxVec3d& pd, const PxVec3d& candidiate)
	{
		PxF64 aex = pa.x - candidiate.x;
		PxF64 bex = pb.x - candidiate.x;
		PxF64 cex = pc.x - candidiate.x;
		PxF64 dex = pd.x - candidiate.x;
		PxF64 aey = pa.y - candidiate.y;
		PxF64 bey = pb.y - candidiate.y;
		PxF64 cey = pc.y - candidiate.y;
		PxF64 dey = pd.y - candidiate.y;
		PxF64 aez = pa.z - candidiate.z;
		PxF64 bez = pb.z - candidiate.z;
		PxF64 cez = pc.z - candidiate.z;
		PxF64 dez = pd.z - candidiate.z;

		PxF64 ab = aex * bey - bex * aey;
		PxF64 bc = bex * cey - cex * bey;
		PxF64 cd = cex * dey - dex * cey;
		PxF64 da = dex * aey - aex * dey;

		PxF64 ac = aex * cey - cex * aey;
		PxF64 bd = bex * dey - dex * bey;

		PxF64 abc = aez * bc - bez * ac + cez * ab;
		PxF64 bcd = bez * cd - cez * bd + dez * bc;
		PxF64 cda = cez * da + dez * ac + aez * cd;
		PxF64 dab = dez * ab + aez * bd + bez * da;

		PxF64 alift = aex * aex + aey * aey + aez * aez;
		PxF64 blift = bex * bex + bey * bey + bez * bez;
		PxF64 clift = cex * cex + cey * cey + cez * cez;
		PxF64 dlift = dex * dex + dey * dey + dez * dez;

		return (dlift * abc - clift * dab) + (blift * cda - alift * bcd);
	}

	PX_FORCE_INLINE bool isDelaunay(const PxArray<PxVec3d>& points, const PxArray<Tetrahedron>& tets, const PxArray<PxI32>& neighbors, PxI32 faceId)
	{
		PxI32 neighborPointer = neighbors[faceId];
		if (neighborPointer < 0)
			return true; //Border faces are always delaunay

		PxI32 tet1Id = faceId >> 2;
		PxI32 tet2Id = neighborPointer >> 2;
		const PxI32* localTriangle1 = neighborFaces[faceId & 3];
		PxI32 localTip1 = tetTip[faceId & 3];
		PxI32 localTip2 = tetTip[neighborPointer & 3];

		Tetrahedron tet1 = tets[tet1Id];
		Tetrahedron tet2 = tets[tet2Id];

		return inSphere(points[tet1[localTriangle1[0]]], points[tet1[localTriangle1[1]]], points[tet1[localTriangle1[2]]], points[tet1[localTip1]], points[tet2[localTip2]]) > 0;
	}

	PX_FORCE_INLINE PxI32 storeNewTet(PxArray<Tetrahedron>& tets, PxArray<PxI32>& neighbors, const Tetrahedron& tet, PxArray<PxI32>& unusedTets)
	{
		if (unusedTets.size() == 0)
		{
			PxU32 tetId = tets.size();
			tets.pushBack(tet);
			for (PxI32 i = 0; i < 4; ++i)
				neighbors.pushBack(-1);
			return tetId;
		}
		else
		{
			PxI32 tetId = unusedTets[unusedTets.size() - 1];
			unusedTets.remove(unusedTets.size() - 1);
			tets[tetId] = tet;
			return tetId;
		}
	}

	PX_FORCE_INLINE PxI32 localFaceId(PxI32 localA, PxI32 localB, PxI32 localC)
	{
		PxI32 result = localA + localB + localC - 3;
		PX_ASSERT(result >= 0 || result <= 3);
		return result;
	}

	PX_FORCE_INLINE PxI32 localFaceId(const Tetrahedron& tet, PxI32 globalA, PxI32 globalB, PxI32 globalC)
	{
		PxI32 result = localFaceId(tet.indexOf(globalA), tet.indexOf(globalB), tet.indexOf(globalC));
		return result;
	}
	
	PX_FORCE_INLINE void setFaceNeighbor(PxArray<PxI32>& neighbors, PxI32 tetId, PxI32 faceId, PxI32 newNeighborTet, PxI32 newNeighborFace)
	{
		neighbors[4 * tetId + faceId] = 4 * newNeighborTet + newNeighborFace;
		neighbors[4 * newNeighborTet + newNeighborFace] = 4 * tetId + faceId;
	}
	PX_FORCE_INLINE void setFaceNeighbor(PxArray<PxI32>& neighbors, PxI32 tetId, PxI32 faceId, PxI32 newNeighbor)
	{
		neighbors[4 * tetId + faceId] = newNeighbor;

		if (newNeighbor >= 0)
			neighbors[newNeighbor] = 4 * tetId + faceId;
	}
	PX_FORCE_INLINE void setFaceNeighbor(PxArray<PxI32>& affectedFaces, PxArray<PxI32>& neighbors, PxI32 tetId, PxI32 faceId, PxI32 newNeighbor)
	{
		setFaceNeighbor(neighbors, tetId, faceId, newNeighbor);
		affectedFaces.pushBack(newNeighbor);
	}
	
	void flip1to4(PxI32 pointToInsert, PxI32 tetId, PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, PxArray<Tetrahedron>& tets, PxArray<PxI32>& unusedTets, PxArray<PxI32>& affectedFaces)
	{
		const Tetrahedron origTet = tets[tetId];
		Tetrahedron tet(origTet[0], origTet[1], origTet[2], pointToInsert);
		tets[tetId] = tet;
		Tetrahedron tet2(origTet[0], origTet[3], origTet[1], pointToInsert);
		Tetrahedron tet3(origTet[0], origTet[2], origTet[3], pointToInsert);
		Tetrahedron tet4(origTet[1], origTet[3], origTet[2], pointToInsert);

		PxI32 tet2Id = storeNewTet(tets, neighbors, tet2, unusedTets);
		PxI32 tet3Id = storeNewTet(tets, neighbors, tet3, unusedTets);
		PxI32 tet4Id = storeNewTet(tets, neighbors, tet4, unusedTets);

		PxI32 n1 = neighbors[4 * tetId + localFaceId(origTet, origTet[0], origTet[1], origTet[2])];
		PxI32 n2 = neighbors[4 * tetId + localFaceId(origTet, origTet[0], origTet[1], origTet[3])];
		PxI32 n3 = neighbors[4 * tetId + localFaceId(origTet, origTet[0], origTet[2], origTet[3])];
		PxI32 n4 = neighbors[4 * tetId + localFaceId(origTet, origTet[1], origTet[2], origTet[3])];

		setFaceNeighbor(affectedFaces, neighbors, tetId, localFaceId(tet, origTet[0], origTet[1], origTet[2]), n1);
		setFaceNeighbor(affectedFaces, neighbors, tet2Id, localFaceId(tet2, origTet[0], origTet[1], origTet[3]), n2);
		setFaceNeighbor(affectedFaces, neighbors, tet3Id, localFaceId(tet3, origTet[0], origTet[2], origTet[3]), n3);
		setFaceNeighbor(affectedFaces, neighbors, tet4Id, localFaceId(tet4, origTet[1], origTet[2], origTet[3]), n4);

		setFaceNeighbor(neighbors, tetId, localFaceId(tet, pointToInsert, origTet[0], origTet[1]), tet2Id, localFaceId(tet2, pointToInsert, origTet[0], origTet[1]));
		setFaceNeighbor(neighbors, tetId, localFaceId(tet, pointToInsert, origTet[0], origTet[2]), tet3Id, localFaceId(tet3, pointToInsert, origTet[0], origTet[2]));
		setFaceNeighbor(neighbors, tetId, localFaceId(tet, pointToInsert, origTet[1], origTet[2]), tet4Id, localFaceId(tet4, pointToInsert, origTet[1], origTet[2]));

		setFaceNeighbor(neighbors, tet2Id, localFaceId(tet2, pointToInsert, origTet[0], origTet[3]), tet3Id, localFaceId(tet3, pointToInsert, origTet[0], origTet[3]));
		setFaceNeighbor(neighbors, tet2Id, localFaceId(tet2, pointToInsert, origTet[1], origTet[3]), tet4Id, localFaceId(tet4, pointToInsert, origTet[1], origTet[3]));

		setFaceNeighbor(neighbors, tet3Id, localFaceId(tet3, pointToInsert, origTet[2], origTet[3]), tet4Id, localFaceId(tet4, pointToInsert, origTet[2], origTet[3]));

		vertexToTet[tet[0]] = tetId; vertexToTet[tet[1]] = tetId; vertexToTet[tet[2]] = tetId; vertexToTet[tet[3]] = tetId;
		vertexToTet[tet2[0]] = tet2Id; vertexToTet[tet2[1]] = tet2Id; vertexToTet[tet2[2]] = tet2Id; vertexToTet[tet2[3]] = tet2Id;
		vertexToTet[tet3[0]] = tet3Id; vertexToTet[tet3[1]] = tet3Id; vertexToTet[tet3[2]] = tet3Id; vertexToTet[tet3[3]] = tet3Id;
		vertexToTet[tet4[0]] = tet4Id; vertexToTet[tet4[1]] = tet4Id; vertexToTet[tet4[2]] = tet4Id; vertexToTet[tet4[3]] = tet4Id;
	}

	bool flip2to3(PxI32 tet1Id, PxI32 tet2Id,
		PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, PxArray<Tetrahedron>& tets, PxArray<PxI32>& unusedTets, PxArray<PxI32>& affectedFaces,
		PxI32 tip1, PxI32 tip2, const Triangle& tri)
	{
		// 2->3 flip
		Tetrahedron tet1(tip2, tip1, tri[0], tri[1]);
		Tetrahedron tet2(tip2, tip1, tri[1], tri[2]);
		Tetrahedron tet3(tip2, tip1, tri[2], tri[0]);


		PxI32 n1 = neighbors[4 * tet1Id + localFaceId(tets[tet1Id], tri[0], tri[1], tip1)];
		PxI32 n2 = neighbors[4 * tet2Id + localFaceId(tets[tet2Id], tri[0], tri[1], tip2)];
		if (n1 >= 0 && (n1 >> 2) == (n2 >> 2))
		{
			if (Tetrahedron::identical(tet1, tets[n1 >> 2]))
				return false;
		}

		PxI32 n3 = neighbors[4 * tet1Id + localFaceId(tets[tet1Id], tri[1], tri[2], tip1)];
		PxI32 n4 = neighbors[4 * tet2Id + localFaceId(tets[tet2Id], tri[1], tri[2], tip2)];
		if (n3 >= 0 && (n3 >> 2) == (n4 >> 2))
		{
			if (Tetrahedron::identical(tet2, tets[n3 >> 2]))
				return false;
		}

		PxI32 n5 = neighbors[4 * tet1Id + localFaceId(tets[tet1Id], tri[2], tri[0], tip1)];
		PxI32 n6 = neighbors[4 * tet2Id + localFaceId(tets[tet2Id], tri[2], tri[0], tip2)];
		if (n5 >= 0 && (n5 >> 2) == (n6 >> 2))
		{
			if (Tetrahedron::identical(tet3, tets[n5 >> 2]))
				return false;
		}

		PxI32 tet3Id = storeNewTet(tets, neighbors, tet3, unusedTets);		

		setFaceNeighbor(affectedFaces, neighbors, tet1Id, localFaceId(tet1, tri[0], tri[1], tip1), n1);
		setFaceNeighbor(affectedFaces, neighbors, tet1Id, localFaceId(tet1, tri[0], tri[1], tip2), n2);
		setFaceNeighbor(neighbors, tet1Id, localFaceId(tet1, tri[0], tip1, tip2), tet3Id, localFaceId(tet3, tri[0], tip1, tip2)); //Interior face
		setFaceNeighbor(neighbors, tet1Id, localFaceId(tet1, tri[1], tip1, tip2), tet2Id, localFaceId(tet2, tri[1], tip1, tip2)); //Interior face

		setFaceNeighbor(affectedFaces, neighbors, tet2Id, localFaceId(tet2, tri[1], tri[2], tip1), n3);
		setFaceNeighbor(affectedFaces, neighbors, tet2Id, localFaceId(tet2, tri[1], tri[2], tip2), n4);
		setFaceNeighbor(neighbors, tet2Id, localFaceId(tet2, tri[2], tip1, tip2), tet3Id, localFaceId(tet3, tri[2], tip1, tip2)); //Interior face

		setFaceNeighbor(affectedFaces, neighbors, tet3Id, localFaceId(tet3, tri[2], tri[0], tip1), n5);
		setFaceNeighbor(affectedFaces, neighbors, tet3Id, localFaceId(tet3, tri[2], tri[0], tip2), n6);

		tets[tet1Id] = tet1;
		tets[tet2Id] = tet2;

		vertexToTet[tet1[0]] = tet1Id; vertexToTet[tet1[1]] = tet1Id; vertexToTet[tet1[2]] = tet1Id; vertexToTet[tet1[3]] = tet1Id;
		vertexToTet[tet2[0]] = tet2Id; vertexToTet[tet2[1]] = tet2Id; vertexToTet[tet2[2]] = tet2Id; vertexToTet[tet2[3]] = tet2Id;
		vertexToTet[tet3[0]] = tet3Id; vertexToTet[tet3[1]] = tet3Id; vertexToTet[tet3[2]] = tet3Id; vertexToTet[tet3[3]] = tet3Id;

		return true;
	}

	bool flip3to2(PxI32 tet1Id, PxI32 tet2Id, PxI32 tet3Id,
		PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, PxArray<Tetrahedron>& tets, PxArray<PxI32>& unusedTets, PxArray<PxI32>& affectedFaces,
		PxI32 tip1, PxI32 tip2, PxI32 reflexEdgeA, PxI32 reflexEdgeB, PxI32 nonReflexTrianglePoint)
	{
		// 3->2 flip
		Tetrahedron tet1(tip1, tip2, reflexEdgeA, nonReflexTrianglePoint);
		Tetrahedron tet2(tip2, tip1, reflexEdgeB, nonReflexTrianglePoint);
		
		PxI32 n1 = neighbors[4 * tet1Id + localFaceId(tets[tet1Id], reflexEdgeA, tip1, nonReflexTrianglePoint)];
		PxI32 n2 = neighbors[4 * tet2Id + localFaceId(tets[tet2Id], reflexEdgeA, tip2, nonReflexTrianglePoint)];
		PxI32 n3 = neighbors[4 * tet3Id + localFaceId(tets[tet3Id], reflexEdgeA, tip1, tip2)];
		if (n1 >= 0 && n2 >= 0 && (n1 >> 2) == (n2 >> 2) && Tetrahedron::identical(tet1, tets[n1 >> 2]))
			return false;
		if (n1 >= 0 && n3 >= 0 && (n1 >> 2) == (n3 >> 2) && Tetrahedron::identical(tet1, tets[n1 >> 2]))
			return false;
		if (n2 >= 0 && n3 >= 0 && (n2 >> 2) == (n3 >> 2) && Tetrahedron::identical(tet1, tets[n2 >> 2]))
			return false;
		
		PxI32 n4 = neighbors[4 * tet1Id + localFaceId(tets[tet1Id], reflexEdgeB, tip1, nonReflexTrianglePoint)];
		PxI32 n5 = neighbors[4 * tet2Id + localFaceId(tets[tet2Id], reflexEdgeB, tip2, nonReflexTrianglePoint)];
		PxI32 n6 = neighbors[4 * tet3Id + localFaceId(tets[tet3Id], reflexEdgeB, tip1, tip2)];
		if (n4 >= 0 && n5 >= 0 && (n4 >> 2) == (n5 >> 2) && Tetrahedron::identical(tet2, tets[n4 >> 2]))
			return false;
		if (n4 >= 0 && n6 >= 0 && (n4 >> 2) == (n6 >> 2) && Tetrahedron::identical(tet2, tets[n4 >> 2]))
			return false;
		if (n5 >= 0 && n6 >= 0 && (n5 >> 2) == (n6 >> 2) && Tetrahedron::identical(tet2, tets[n5 >> 2]))
			return false;


		setFaceNeighbor(affectedFaces, neighbors, tet1Id, localFaceId(tet1, reflexEdgeA, tip1, nonReflexTrianglePoint), n1);
		setFaceNeighbor(affectedFaces, neighbors, tet1Id, localFaceId(tet1, reflexEdgeA, tip2, nonReflexTrianglePoint), n2);
		setFaceNeighbor(affectedFaces, neighbors, tet1Id, localFaceId(tet1, reflexEdgeA, tip1, tip2), n3);
		setFaceNeighbor(neighbors, tet1Id, localFaceId(tet1, nonReflexTrianglePoint, tip1, tip2), tet2Id, localFaceId(tet2, nonReflexTrianglePoint, tip1, tip2)); //Interior face // Marker (*)

		setFaceNeighbor(affectedFaces, neighbors, tet2Id, localFaceId(tet2, reflexEdgeB, tip1, nonReflexTrianglePoint), n4);
		setFaceNeighbor(affectedFaces, neighbors, tet2Id, localFaceId(tet2, reflexEdgeB, tip2, nonReflexTrianglePoint), n5);
		setFaceNeighbor(affectedFaces, neighbors, tet2Id, localFaceId(tet2, reflexEdgeB, tip1, tip2), n6);
		
		tets[tet1Id] = tet1;
		tets[tet2Id] = tet2;

		tets[tet3Id] = Tetrahedron(-1, -1, -1, -1);
		for (PxI32 i = 0; i < 4; ++i)
			neighbors[4 * tet3Id + i] = -2;
		unusedTets.pushBack(tet3Id);

		vertexToTet[tet1[0]] = tet1Id; vertexToTet[tet1[1]] = tet1Id; vertexToTet[tet1[2]] = tet1Id; vertexToTet[tet1[3]] = tet1Id;
		vertexToTet[tet2[0]] = tet2Id; vertexToTet[tet2[1]] = tet2Id; vertexToTet[tet2[2]] = tet2Id; vertexToTet[tet2[3]] = tet2Id;
		return true;
	}

	PX_FORCE_INLINE PxF64 orient3D(const PxVec3d& a, const PxVec3d& b, const PxVec3d& c, const PxVec3d& d)
	{
		return (a - d).dot((b - d).cross(c - d));
	}

	PX_FORCE_INLINE bool edgeIsLocked(const PxHashSet<PxU64>& lockedEdges, int edgeA, int edgeB)
	{
		return lockedEdges.contains(key(edgeA, edgeB));
	}

	PX_FORCE_INLINE bool faceIsLocked(const PxHashSet<SortedTriangle, TriangleHash>& lockedTriangles, int triA, int triB, int triC)
	{
		return lockedTriangles.contains(SortedTriangle(triA, triB, triC));
	}

	void flip(const PxArray<PxVec3d>& points, PxArray<Tetrahedron>& tets, PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, PxI32 faceId, PxArray<PxI32>& unusedTets, PxArray<PxI32>& affectedFaces, 
		const PxHashSet<SortedTriangle, TriangleHash>& lockedFaces, const PxHashSet<PxU64>& lockedEdges)
	{
		PxI32 neighborPointer = neighbors[faceId];
		if (neighborPointer < 0)
			return;

		PxI32 tet1Id = faceId >> 2;
		const PxI32* localTriangle1 = neighborFaces[faceId & 3];
		Tetrahedron tet1 = tets[tet1Id];
		Triangle tri(tet1[localTriangle1[0]], tet1[localTriangle1[1]], tet1[localTriangle1[2]]);
		PxI32 localTip1 = tetTip[faceId & 3];
		PxI32 localTip2 = tetTip[neighborPointer & 3];
		PxI32 tip1 = tet1[localTip1];

		PxI32 tet2Id = neighborPointer >> 2;
		Tetrahedron tet2 = tets[tet2Id];
		PxI32 tip2 = tet2[localTip2];

		PX_ASSERT(tet2.contains(tri[0]) && tet2.contains(tri[1]) && tet2.contains(tri[2]));
		
		PxI32 face1 = -1;
		PxI32 face2 = -1;
		PxI32 numReflexEdges = 0;
		PxI32 reflexEdgeA = -1;
		PxI32 reflexEdgeB = -1;
		PxI32 nonReflexTrianglePoint = -1;
		PxF64 ab = orient3D(points[tri[0]], points[tri[1]], points[tip1], points[tip2]);
		if (ab < 0) { ++numReflexEdges; face1 = localFaceId(localTriangle1[0], localTriangle1[1], localTip1); reflexEdgeA = tri[0]; reflexEdgeB = tri[1]; nonReflexTrianglePoint = tri[2]; face2 = localFaceId(tet2, tri[0], tri[1], tip2); }
		PxF64 bc = orient3D(points[tri[1]], points[tri[2]], points[tip1], points[tip2]);
		if (bc < 0) { ++numReflexEdges; face1 = localFaceId(localTriangle1[1], localTriangle1[2], localTip1); reflexEdgeA = tri[1]; reflexEdgeB = tri[2]; nonReflexTrianglePoint = tri[0]; face2 = localFaceId(tet2, tri[1], tri[2], tip2); }
		PxF64 ca = orient3D(points[tri[2]], points[tri[0]], points[tip1], points[tip2]);
		if (ca < 0) { ++numReflexEdges; face1 = localFaceId(localTriangle1[2], localTriangle1[0], localTip1); reflexEdgeA = tri[2]; reflexEdgeB = tri[0]; nonReflexTrianglePoint = tri[1]; face2 = localFaceId(tet2, tri[2], tri[0], tip2); }

		if (numReflexEdges == 0)
		{
			if (!faceIsLocked(lockedFaces, tri[0], tri[1], tri[2]))
				flip2to3(tet1Id, tet2Id, neighbors, vertexToTet, tets, unusedTets, affectedFaces, tip1, tip2, tri);
		}
		else if (numReflexEdges == 1)
		{
			PxI32 candidate1 = neighbors[4 * tet1Id + face1] >> 2;
			PxI32 candidate2 = neighbors[4 * tet2Id + face2] >> 2;
			if (candidate1 == candidate2 && candidate1 >= 0)
			{
				if (!edgeIsLocked(lockedEdges, reflexEdgeA, reflexEdgeB) &&
					!faceIsLocked(lockedFaces, reflexEdgeA, reflexEdgeB, nonReflexTrianglePoint) &&
					!faceIsLocked(lockedFaces, reflexEdgeA, reflexEdgeB, tip1) &&
					!faceIsLocked(lockedFaces, reflexEdgeA, reflexEdgeB, tip2))
					flip3to2(tet1Id, tet2Id, candidate1, neighbors, vertexToTet, tets, unusedTets, affectedFaces, tip1, tip2, reflexEdgeA, reflexEdgeB, nonReflexTrianglePoint);
			}
		}
		else if (numReflexEdges == 2)
		{
			//Cannot do anything			
		}
		else if (numReflexEdges == 3)
		{
			//Something is wrong if we end up here - maybe there are degenerate tetrahedra or issues due to numerical rounding			
		}
	}

	void flip(PxArray<PxI32>& faces, PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, const PxArray<PxVec3d>& points,
		PxArray<Tetrahedron>& tets, PxArray<PxI32>& unusedTets, PxArray<PxI32>& affectedFaces,
		const PxHashSet<SortedTriangle, TriangleHash>& lockedFaces, const PxHashSet<PxU64>& lockedEdges)
	{
		while (faces.size() > 0)
		{
			PxI32 faceId = faces.popBack();
			if (faceId < 0)
				continue;

			if (isDelaunay(points, tets, neighbors, faceId))
				continue;

			affectedFaces.clear();
			flip(points, tets, neighbors, vertexToTet, faceId, unusedTets, affectedFaces, lockedFaces, lockedEdges);
			
			for (PxU32 j = 0; j < affectedFaces.size(); ++j)
				if (faces.find(affectedFaces[j]) == faces.end())
					faces.pushBack(affectedFaces[j]);
		}
	}

	void insertAndFlip(PxI32 pointToInsert, PxI32 tetId, PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, const PxArray<PxVec3d>& points,
		PxArray<Tetrahedron>& tets, PxArray<PxI32>& unusedTets, PxArray<PxI32>& affectedFaces,
		const PxHashSet<SortedTriangle, TriangleHash>& lockedFaces, const PxHashSet<PxU64>& lockedEdges)
	{
		flip1to4(pointToInsert, tetId, neighbors, vertexToTet, tets, unusedTets, affectedFaces);
		
		PxArray<PxI32> stack;
		for (PxU32 j = 0; j < affectedFaces.size(); ++j)
			if (stack.find(affectedFaces[j]) == stack.end())
				stack.pushBack(affectedFaces[j]);

		flip(stack, neighbors, vertexToTet, points, tets, unusedTets, affectedFaces, lockedFaces, lockedEdges);
	}

	PxI32 searchAll(const PxVec3d& p, const PxArray<Tetrahedron>& tets, const PxArray<PxVec3d>& points)
	{
		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			const Tetrahedron& tet = tets[i];
			if (tet[0] < 0)
				continue;

			PxI32 j = 0;
			for (; j < 4; j++)
			{
				Plane plane(points[tet[neighborFaces[j][0]]], points[tet[neighborFaces[j][1]]], points[tet[neighborFaces[j][2]]]);
				PxF64 distP = plane.signedDistance(p);
				if (distP < 0)
					break;
			}
			if (j == 4)
				return i;
		}
		return -1;
	}

	bool runDelaunay(const PxArray<PxVec3d>& points, PxI32 start, PxI32 end, PxArray<Tetrahedron>& tets, PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, PxArray<PxI32>& unusedTets,
		const PxHashSet<SortedTriangle, TriangleHash>& lockedFaces, const PxHashSet<PxU64>& lockedEdges)
	{
		PxI32 tetId = 0;

		PxArray<PxI32> affectedFaces;
		for (PxI32 i = start; i < end; ++i)
		{
			const PxVec3d p = points[i];
			if (!PxIsFinite(p.x) || !PxIsFinite(p.y) || !PxIsFinite(p.z))
				continue;

			if (tetId < 0 || unusedTets.find(tetId) != unusedTets.end())
				tetId = 0;

			while (unusedTets.find(tetId) != unusedTets.end())
				++tetId;

			PxU32 counter = 0;
			bool tetLocated = false;
			while (!tetLocated)
			{
				const Tetrahedron& tet = tets[tetId];
				const PxVec3d center = (points[tet[0]] + points[tet[1]] + points[tet[2]] + points[tet[3]]) * 0.25;

				PxF64 minDist = DBL_MAX;
				PxI32 minFaceNr = -1;

				for (PxI32 j = 0; j < 4; j++)
				{
					Plane plane(points[tet[neighborFaces[j][0]]], points[tet[neighborFaces[j][1]]], points[tet[neighborFaces[j][2]]]);
					PxF64 distP = plane.signedDistance(p);
					PxF64 distCenter = plane.signedDistance(center);
					PxF64 delta = distP - distCenter;
					if (delta == 0.0)
						continue;
					delta = -distCenter / delta;
					if (delta >= 0.0 && delta < minDist)
					{
						minDist = delta;
						minFaceNr = j;
					}
				}
				if (minDist >= 1.0)
					tetLocated = true;
				else
				{
					tetId = neighbors[4 * tetId + minFaceNr] >> 2;
					if (tetId < 0)
					{
						tetId = searchAll(p, tets, points);
						tetLocated = true;
					}
				}

				++counter;
				if (counter > tets.size())
				{
					tetId = searchAll(p, tets, points);
					if (tetId < 0)
						return false;
					tetLocated = true;
				}
			}

			insertAndFlip(i, tetId, neighbors, vertexToTet, points, tets, unusedTets, affectedFaces, lockedFaces, lockedEdges);
		}
		return true;
	}

	bool DelaunayTetrahedralizer::insertPoints(const PxArray<PxVec3d>& inPoints, PxI32 start, PxI32 end)
	{
		for (PxI32 i = start; i < end; ++i) 
		{
			centeredNormalizedPoints.pushBack(inPoints[i]);
			vertexToTet.pushBack(-1);
		}

		if (!runDelaunay(centeredNormalizedPoints, start + numAdditionalPointsAtBeginning, end + numAdditionalPointsAtBeginning, result, neighbors, vertexToTet, unusedTets, lockedTriangles, lockedEdges))
			return false;
		return true;
	}

	void  DelaunayTetrahedralizer::exportTetrahedra(PxArray<Tetrahedron>& tetrahedra)
	{
		tetrahedra.clear();
		for (PxU32 i = 0; i < result.size(); ++i)
		{
			const Tetrahedron& tet = result[i];
			if (tet[0] >= numAdditionalPointsAtBeginning && tet[1] >= numAdditionalPointsAtBeginning && tet[2] >= numAdditionalPointsAtBeginning && tet[3] >= numAdditionalPointsAtBeginning)
				tetrahedra.pushBack(Tetrahedron(tet[0] - numAdditionalPointsAtBeginning, tet[1] - numAdditionalPointsAtBeginning, tet[2] - numAdditionalPointsAtBeginning, tet[3] - numAdditionalPointsAtBeginning));
		}
	}

	void DelaunayTetrahedralizer::insertPoints(const PxArray<PxVec3d>& inPoints, PxI32 start, PxI32 end, PxArray<Tetrahedron>& tetrahedra)
	{
		insertPoints(inPoints, start, end);
		exportTetrahedra(tetrahedra);
	}

	//Code below this line is for tetmesh manipulation and not directly required to generate a Delaunay tetrahedron mesh. It is used to impmrove the quality of a tetrahedral mesh.

	//https://cs.nyu.edu/~panozzo/papers/SLIM-2016.pdf
	PxF64 evaluateAmipsEnergyPow3(const PxVec3d& a, const PxVec3d& b, const PxVec3d& c, const PxVec3d& d)
	{
		PxF64 x1 = a.x + d.x - 2.0 * b.x;
		PxF64 x2 = a.x + b.x + d.x - 3.0 * c.x;
		PxF64 y1 = a.y + d.y - 2.0 * b.y;
		PxF64 y2 = a.y + b.y + d.y - 3.0 * c.y;
		PxF64 z1 = a.z + d.z - 2.0 * b.z;
		PxF64 z2 = a.z + b.z + d.z - 3.0 * c.z;

		PxF64 f = (1.0 / (PxSqrt(3.0) * PxSqrt(6.0))) *
			((a.x - d.x) * (y1 * z2 - y2 * z1) -
			(a.y - d.y) * (z2 * x1 - x2 * z1) +
				(a.z - d.z) * (y2 * x1 - y1 * x2));

		if (f >= 0)
			return 0.25 * DBL_MAX;

		PxF64 g = a.x * (b.x + c.x + d.x - 3.0 * a.x) +
			a.y * (b.y + c.y + d.y - 3.0 * a.y) +
			a.z * (b.z + c.z + d.z - 3.0 * a.z) +
			b.x * (a.x + c.x + d.x - 3.0 * b.x) +
			b.y * (a.y + c.y + d.y - 3.0 * b.y) +
			b.z * (a.z + c.z + d.z - 3.0 * b.z) +
			c.x * x2 +
			c.y * y2 +
			c.z * z2 +
			d.x * (a.x + b.x + c.x - 3.0 * d.x) +
			d.y * (a.y + b.y + c.y - 3.0 * d.y) +
			d.z * (a.z + b.z + c.z - 3.0 * d.z);

		return -0.125 * (g * g * g) / (f * f);
		//return -0.5 * PxPow(f * f, -1.0 / 3.0) * g;
	}

	PX_FORCE_INLINE PxF64 pow(const PxF64 a, const PxF64 b)
	{
		return PxF64(PxPow(PxF32(a), PxF32(b)));
	}

	PX_FORCE_INLINE PxF64 evaluateAmipsEnergy(const PxVec3d& a, const PxVec3d& b, const PxVec3d& c, const PxVec3d& d)
	{
		return pow(evaluateAmipsEnergyPow3(a, b, c, d), 1.0 / 3.0);
	}

	PxF64 maxEnergyPow3(const PxArray<PxVec3d>& points, const PxArray<Tetrahedron>& tets)
	{
		PxF64 maxEnergy = 0;
		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			const Tetrahedron& tet = tets[i];
			if (tet[0] < 0)
				continue;
			PxF64 e = evaluateAmipsEnergyPow3(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]);
			if (e > maxEnergy)
				maxEnergy = e;
		}
		return maxEnergy;
	}

	PX_FORCE_INLINE PxF64 maxEnergy(const PxArray<PxVec3d>& points, const PxArray<Tetrahedron>& tets)
	{
		return pow(maxEnergyPow3(points, tets), 1.0 / 3.0);
	}

	PxF64 maxEnergyPow3(const PxArray<PxVec3d>& points, const PxArray<Tetrahedron>& tets, const PxArray<PxI32>& tetIds)
	{
		PxF64 maxEnergy = 0;
		for (PxU32 i = 0; i < tetIds.size(); ++i)
		{
			const Tetrahedron& tet = tets[tetIds[i]];
			if (tet[0] < 0)
				continue;
			PxF64 e = evaluateAmipsEnergyPow3(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]);
			if (e > maxEnergy)
				maxEnergy = e;
		}
		return maxEnergy;
	}

	PxF64 minAbsTetVolume(const PxArray<PxVec3d>& points, const PxArray<Tetrahedron>& tets)
	{
		PxF64 minVol = DBL_MAX;
		for (PxU32 i = 0; i < tets.size(); ++i) {
			const Tetrahedron& tet = tets[i];
			if (tet[0] < 0)
				continue;
			PxF64 vol = PxAbs(tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]));
			if (vol < minVol)
				minVol = vol;
		}
		return minVol;
	}

	PxF64 minTetVolume(const PxArray<PxVec3d>& points, const PxArray<Tetrahedron>& tets)
	{
		PxF64 minVol = DBL_MAX;
		for (PxU32 i = 0; i < tets.size(); ++i) {
			const Tetrahedron& tet = tets[i];
			if (tet[0] < 0)
				continue;
			PxF64 vol = tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]);
			if (vol < minVol)
				minVol = vol;
		}
		return minVol;
	}
	
	PxF64 minTetVolume(const PxArray<PxVec3d>& points, const PxArray<Tetrahedron>& tets, const PxArray<PxI32>& indices)
	{
		PxF64 minVol = DBL_MAX;
		for (PxU32 i = 0; i < indices.size(); ++i) {
			const Tetrahedron& tet = tets[indices[i]];
			if (tet[0] < 0)
				continue;
			PxF64 vol = tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]);
			if (vol < minVol)
				minVol = vol;
		}
		return minVol;
	}

	PxF64 MinimizeMaxAmipsEnergy::quality(const PxArray<PxI32> tetIndices) const
	{
		return maxEnergyPow3(points, tetrahedra, tetIndices);
	}

	PxF64 MinimizeMaxAmipsEnergy::quality(const PxArray<Tetrahedron> tetrahedraToCheck) const
	{
		return maxEnergyPow3(points, tetrahedraToCheck);
	}

	bool MinimizeMaxAmipsEnergy::improved(PxF64 previousQuality, PxF64 newQuality) const
	{
		return newQuality < previousQuality; //Minimize quality for Amips energy
	}

	PxF64 MaximizeMinTetVolume::quality(const PxArray<PxI32> tetIndices) const
	{
		return minTetVolume(points, tetrahedra, tetIndices);
	}

	PxF64 MaximizeMinTetVolume::quality(const PxArray<Tetrahedron> tetrahedraToCheck) const
	{
		return minTetVolume(points, tetrahedraToCheck);
	}

	bool MaximizeMinTetVolume::improved(PxF64 previousQuality, PxF64 newQuality) const
	{
		return newQuality > previousQuality; //Maximize quality for min volume
	}

	void fixVertexToTet(PxArray<PxI32>& vertexToTet, const PxArray<PxI32>& newTetIds, const PxArray<Tetrahedron>& tets)
	{
		for (PxU32 i = 0; i < newTetIds.size(); ++i)
		{
			PxI32 id = newTetIds[i];
			const Tetrahedron& tet = tets[id];
			if (tet[0] < 0)
				continue;
			while (PxU32(tet[0]) >= vertexToTet.size()) vertexToTet.pushBack(-1);
			while (PxU32(tet[1]) >= vertexToTet.size()) vertexToTet.pushBack(-1);
			while (PxU32(tet[2]) >= vertexToTet.size()) vertexToTet.pushBack(-1);
			while (PxU32(tet[3]) >= vertexToTet.size()) vertexToTet.pushBack(-1);
			vertexToTet[tet[0]] = id; vertexToTet[tet[1]] = id; vertexToTet[tet[2]] = id; vertexToTet[tet[3]] = id;
		}
	}

	void fixNeighborhoodLocally(const PxArray<PxI32>& removedTets, const PxArray<PxI32>& tetIndices, const PxArray<Tetrahedron>& tets, 
		PxArray<PxI32>& neighborhood, PxArray<PxI32>* affectedFaces = NULL)
	{
		for (PxU32 k = 0; k < removedTets.size(); ++k)
		{
			PxI32 i = removedTets[k];
			for (PxI32 j = 0; j < 4; ++j)
			{
				PxI32 faceId = 4 * i + j;
				neighborhood[faceId] = -1;

				if (affectedFaces != NULL && affectedFaces->find(faceId) == affectedFaces->end())
					affectedFaces->pushBack(faceId);
			}
		}

		PxHashMap<SortedTriangle, PxI32, TriangleHash> faces;
		for(PxU32 k = 0; k< tetIndices.size(); ++k)
		{
			PxI32 i = tetIndices[k];
			if (i < 0)
				continue;

			const Tetrahedron& tet = tets[i];
			if (tet[0] < 0)
				continue;

			for (PxI32 j = 0; j < 4; ++j)
			{
				SortedTriangle tri(tet[neighborFaces[j][0]], tet[neighborFaces[j][1]], tet[neighborFaces[j][2]]);
				if (const PxPair<const SortedTriangle, PxI32>* ptr = faces.find(tri))
				{
					neighborhood[4 * i + j] = ptr->second;
					neighborhood[ptr->second] = 4 * i + j;
				}
				else
					faces.insert(tri, 4 * i + j);
			}
		}

		//validateNeighborhood(tets, neighborhood);
	}

	void collectTetsConnectedToVertex(PxArray<PxI32>& faces, PxHashSet<PxI32>& tetsDone, const PxArray<Tetrahedron>& tets, const PxArray<PxI32>& tetNeighbors, const PxArray<PxI32>& vertexToTet,
		PxI32 vertexIndex, PxArray<PxI32>& tetIds, PxI32 secondVertexId = -1, PxI32 thirdVertexId = -1)
	{
		tetIds.clear();
		int tetIndex = vertexToTet[vertexIndex];
		if (tetIndex < 0)
			return; //Free floating point

		if ((secondVertexId < 0 || tets[tetIndex].contains(secondVertexId)) && (thirdVertexId < 0 || tets[tetIndex].contains(thirdVertexId)))
			tetIds.pushBack(tetIndex);

		faces.clear();
		tetsDone.clear();
		for (int i = 0; i < 4; ++i)
		{
			if (tetNeighbors[4 * tetIndex + i] >= 0)
				faces.pushBack(4 * tetIndex + i);
		}
		tetsDone.insert(tetIndex);

		while (faces.size() > 0)
		{
			PxI32 faceId = faces.popBack();
			int tetId = tetNeighbors[faceId] >> 2;

			if (tetId < 0 || tetIds.find(tetId) != tetIds.end())
				continue;

			const Tetrahedron& tet = tets[tetId];
			int id = tet.indexOf(vertexIndex);
			if (id >= 0)
			{
				if ((secondVertexId < 0 || tets[tetId].contains(secondVertexId)) && (thirdVertexId < 0 || tets[tetId].contains(thirdVertexId)))
					tetIds.pushBack(tetId);

				const PxI32* f = neighborFaces[id];
				for (int i = 0; i < 3; ++i)
				{
					int candidate = 4 * tetId + f[i];
					if (tetsDone.insert(tetNeighbors[candidate] >> 2))
					{
						int otherTetId = tetNeighbors[candidate] >> 2;
						if (otherTetId >= 0 && tets[otherTetId].contains(vertexIndex))
							faces.pushBack(candidate);
					}
				}
			}
		}
	}

	void DelaunayTetrahedralizer::collectTetsConnectedToVertex(PxI32 vertexIndex, PxArray<PxI32>& tetIds)
	{
		stackMemory.clear();
		physx::Ext::collectTetsConnectedToVertex(stackMemory.faces, stackMemory.hashSet, result, neighbors, vertexToTet, vertexIndex, tetIds, -1, -1);
	}

	void DelaunayTetrahedralizer::collectTetsConnectedToVertex(PxArray<PxI32>& faces, PxHashSet<PxI32>& tetsDone, PxI32 vertexIndex, PxArray<PxI32>& tetIds)
	{
		faces.clear();
		tetsDone.clear();
		physx::Ext::collectTetsConnectedToVertex(faces, tetsDone, result, neighbors, vertexToTet, vertexIndex, tetIds);
	}

	void DelaunayTetrahedralizer::collectTetsConnectedToEdge(PxI32 edgeStart, PxI32 edgeEnd, PxArray<PxI32>& tetIds)
	{
		if (edgeStart < edgeEnd)
			PxSwap(edgeStart, edgeEnd);

		stackMemory.clear();
		physx::Ext::collectTetsConnectedToVertex(stackMemory.faces, stackMemory.hashSet, result, neighbors, vertexToTet, edgeStart, tetIds, edgeEnd);
	}

	PX_FORCE_INLINE bool containsDuplicates(PxI32 a, PxI32 b, PxI32 c, PxI32 d)
	{
		return a == b || a == c || a == d || b == c || b == d || c == d;
	}

	//Returns true if the volume of a tet would become negative if a vertex it contains would get replaced by another one 
	bool tetFlipped(const Tetrahedron& tet, PxI32 vertexToReplace, PxI32 replacement, const PxArray<PxVec3d>& points, PxF64 volumeChangeThreshold = 0.1)
	{
		PxF64 volumeBefore = tetVolume(points[tet[0]], points[tet[1]], points[tet[2]], points[tet[3]]);

		PxI32 a = tet[0] == vertexToReplace ? replacement : tet[0];
		PxI32 b = tet[1] == vertexToReplace ? replacement : tet[1];
		PxI32 c = tet[2] == vertexToReplace ? replacement : tet[2];
		PxI32 d = tet[3] == vertexToReplace ? replacement : tet[3];

		if (containsDuplicates(a, b, c, d))
			return false;

		PxF64 volume = tetVolume(points[a], points[b], points[c], points[d]);
		if (volume < 0)
			return true;
		if (PxAbs(volume / volumeBefore) < volumeChangeThreshold)
			return true;
		return false;
	}

	PX_FORCE_INLINE Tetrahedron getTet(const Tetrahedron& tet, PxI32 vertexToReplace, PxI32 replacement)
	{
		return Tetrahedron(tet[0] == vertexToReplace ? replacement : tet[0],
			tet[1] == vertexToReplace ? replacement : tet[1],
			tet[2] == vertexToReplace ? replacement : tet[2],
			tet[3] == vertexToReplace ? replacement : tet[3]);
	}

	PX_FORCE_INLINE bool containsDuplicates(const Tetrahedron& tet) 
	{
		return tet[0] == tet[1] || tet[0] == tet[2] || tet[0] == tet[3] || tet[1] == tet[2] || tet[1] == tet[3] || tet[2] == tet[3];
	}

	//Returns true if a tetmesh's edge, defined by vertex indices keep and remove, can be collapsed without leading to an invalid tetmesh topology
	bool canCollapseEdge(PxI32 keep, PxI32 remove, const PxArray<PxVec3d>& points, 
		const PxArray<PxI32>& tetsConnectedToKeepVertex, const PxArray<PxI32>& tetsConnectedToRemoveVertex, const PxArray<Tetrahedron>& allTet, 
		PxF64& qualityAfterCollapse, PxF64 volumeChangeThreshold = 0.1, BaseTetAnalyzer* tetAnalyzer = NULL)
	{
		const PxArray<PxI32>& tets = tetsConnectedToRemoveVertex;
		//If a tet would get a negative volume due to the edge collapse, then this edge is not collapsible
		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			const Tetrahedron& tet = allTet[tets[i]];
			if (tet[0] < 0)
				return false;
			if (tetFlipped(tet, remove, keep, points, volumeChangeThreshold))
				return false;
		}

		PxHashMap<SortedTriangle, PxI32, TriangleHash> triangles;

		PxHashSet<PxI32> candidates;
		for (PxU32 i = 0; i < tets.size(); ++i)
			candidates.insert(tets[i]);
		const PxArray<PxI32>& tets2 = tetsConnectedToKeepVertex;
		for (PxU32 i = 0; i < tets2.size(); ++i)
			if (allTet[tets2[i]][0] >= 0)
				candidates.insert(tets2[i]);

		PxArray<PxI32> affectedTets;
		affectedTets.reserve(candidates.size());
		PxArray<Tetrahedron> remainingTets;
		remainingTets.reserve(candidates.size());
		//If a tet-face would get referenced by more than 2 tetrahedra due to the edge collapse, then this edge is not collapsible 
		for (PxHashSet<PxI32>::Iterator iter = candidates.getIterator(); !iter.done(); ++iter)
		{
			PxI32 tetRef = *iter;
			affectedTets.pushBack(tetRef);
			Tetrahedron tet = getTet(allTet[tetRef], remove, keep);
			if (containsDuplicates(tet))
				continue;

			remainingTets.pushBack(tet);

			for (PxU32 j = 0; j < 4; ++j)
			{
				SortedTriangle tri(tet[neighborFaces[j][0]], tet[neighborFaces[j][1]], tet[neighborFaces[j][2]]);
				if (const PxPair<const SortedTriangle, PxI32>* ptr = triangles.find(tri))
					triangles[tri] = ptr->second + 1;
				else
					triangles.insert(tri, 1);
			}
		}

		for (PxHashMap<SortedTriangle, PxI32, TriangleHash>::Iterator iter = triangles.getIterator(); !iter.done(); ++iter)
			if (iter->second > 2)
				return false;

		if (tetAnalyzer == NULL)
			return true;

		qualityAfterCollapse = tetAnalyzer->quality(remainingTets);
		return tetAnalyzer->improved(tetAnalyzer->quality(affectedTets), qualityAfterCollapse);
	}

	bool DelaunayTetrahedralizer::canCollapseEdge(PxI32 edgeVertexToKeep, PxI32 edgeVertexToRemove, PxF64 volumeChangeThreshold, BaseTetAnalyzer* tetAnalyzer)
	{
		PxF64 qualityAfterCollapse;
		PxArray<PxI32> tetsA, tetsB;
		stackMemory.clear();
		physx::Ext::collectTetsConnectedToVertex(stackMemory.faces, stackMemory.hashSet, result, neighbors, vertexToTet, edgeVertexToKeep, tetsA);
		stackMemory.clear();
		physx::Ext::collectTetsConnectedToVertex(stackMemory.faces, stackMemory.hashSet, result, neighbors, vertexToTet, edgeVertexToRemove, tetsB);
		return canCollapseEdge(edgeVertexToKeep, edgeVertexToRemove, tetsA, tetsB, qualityAfterCollapse, volumeChangeThreshold, tetAnalyzer);
	}

	bool DelaunayTetrahedralizer::canCollapseEdge(PxI32 edgeVertexAToKeep, PxI32 edgeVertexBToRemove, const PxArray<PxI32>& tetsConnectedToA, const PxArray<PxI32>& tetsConnectedToB,
		PxF64& qualityAfterCollapse, PxF64 volumeChangeThreshold, BaseTetAnalyzer* tetAnalyzer)
	{
		return physx::Ext::canCollapseEdge(edgeVertexAToKeep, edgeVertexBToRemove, centeredNormalizedPoints,
			tetsConnectedToA, //physx::Ext::collectTetsConnectedToVertex(result, neighbors, vertexToTet, edgeVertexAToKeep),
			tetsConnectedToB, //physx::Ext::collectTetsConnectedToVertex(result, neighbors, vertexToTet, edgeVertexBToRemove),
			result, qualityAfterCollapse, volumeChangeThreshold, tetAnalyzer);
	}

	void collapseEdge(PxI32 keep, PxI32 remove, const PxArray<PxI32>& tetsConnectedToKeepVertex, const PxArray<PxI32>& tetsConnectedToRemoveVertex,
		PxArray<Tetrahedron>& allTets, PxArray<PxI32>& neighborhood, PxArray<PxI32>& vertexToTet, PxArray<PxI32>& unusedTets, PxArray<PxI32>& changedTets)
	{
		PxArray<PxI32> removedTets;
		changedTets.clear();

		const PxArray<PxI32>& tets = tetsConnectedToRemoveVertex;
		for (PxI32 i = tets.size() - 1; i >= 0; --i)
		{
			PxI32 tetId = tets[i];
			Tetrahedron tet = allTets[tetId];
			tet.replace(remove, keep);
			if (containsDuplicates(tet))
			{
				for (PxU32 j = 0; j < 4; ++j) 
				{
					if (vertexToTet[tet[j]] == tetId) vertexToTet[tet[j]] = -1;
					tet[j] = -1;
				}
				removedTets.pushBack(tetId);
				unusedTets.pushBack(tetId);
			}
			else
			{
				changedTets.pushBack(tetId);
			}
			allTets[tetId] = tet;
		}

		for (PxU32 i = 0; i < tetsConnectedToKeepVertex.size(); ++i)
		{
			PxI32 id = tetsConnectedToKeepVertex[i];
			if (removedTets.find(id) == removedTets.end())
				changedTets.pushBack(id);
		}

		for (PxU32 i = 0; i < removedTets.size(); ++i)
		{
			PxI32 id = removedTets[i];
			for (PxI32 j = 0; j < 4; ++j)
			{
				PxI32 n = neighborhood[4 * id + j];
				if (n >= 0)
					neighborhood[n] = -1;
			}
		}

		fixNeighborhoodLocally(removedTets, changedTets, allTets, neighborhood);
		fixVertexToTet(vertexToTet, changedTets, allTets);
		vertexToTet[remove] = -1;
	}

	void DelaunayTetrahedralizer::collapseEdge(PxI32 edgeVertexToKeep, PxI32 edgeVertexToRemove)
	{
		PxArray<PxI32> changedTets;

		PxArray<PxI32> keepTetIds;
		PxArray<PxI32> removeTetIds;
		stackMemory.clear();
		physx::Ext::collectTetsConnectedToVertex(stackMemory.faces, stackMemory.hashSet, result, neighbors, vertexToTet, edgeVertexToKeep, keepTetIds);
		stackMemory.clear();
		physx::Ext::collectTetsConnectedToVertex(stackMemory.faces, stackMemory.hashSet, result, neighbors, vertexToTet, edgeVertexToRemove, removeTetIds);
		physx::Ext::collapseEdge(edgeVertexToKeep, edgeVertexToRemove,
			keepTetIds,
			removeTetIds,
			result, neighbors, vertexToTet, unusedTets, changedTets);
	}

	void DelaunayTetrahedralizer::collapseEdge(PxI32 edgeVertexAToKeep, PxI32 edgeVertexBToRemove, const PxArray<PxI32>& tetsConnectedToA, const PxArray<PxI32>& tetsConnectedToB)
	{
		PxArray<PxI32> changedTets;
		physx::Ext::collapseEdge(edgeVertexAToKeep, edgeVertexBToRemove,
			tetsConnectedToA, tetsConnectedToB,
			result, neighbors, vertexToTet, unusedTets, changedTets);
	}

	bool buildRing(const PxArray<Edge>& edges, PxArray<PxI32>& result)
	{
		result.reserve(edges.size() + 1);
		const Edge& first = edges[0];
		result.pushBack(first.first);
		result.pushBack(first.second);

		PxU32 currentEdge = 0;
		PxI32 connector = first.second;

		for (PxU32 j = 1; j < edges.size(); ++j)
		{
			for (PxU32 i = 1; i < edges.size(); ++i)
			{
				if (i == currentEdge)
					continue;

				const Edge& e = edges[i];
				if (e.first == connector)
				{
					currentEdge = i;
					result.pushBack(e.second);
					connector = e.second;
				}
				else if (e.second == connector)
				{
					currentEdge = i;
					result.pushBack(e.first);
					connector = e.first;
				}
				if (connector == result[0])
				{
					result.remove(result.size() - 1);
					if (result.size() != edges.size())
					{
						result.clear();
						return false; //Multiple segments - unexpected input
					}
					return true; //Cyclic
				}
			}
		}
		result.clear();
		return false;
	}

	void addRange(PxHashSet<PxI32>& set, const PxArray<PxI32>& list)
	{
		for (PxU32 i = 0; i < list.size(); ++i)
			set.insert(list[i]);
	}

	static Edge createEdge(PxI32 x, PxI32 y, bool swap)
	{
		if (swap)
			return Edge(y, x);
		else
			return Edge(x, y);
	}

	Edge getOtherEdge(const Tetrahedron& tet, PxI32 x, PxI32 y)
	{
		x = tet.indexOf(x);
		y = tet.indexOf(y);

		bool swap = x > y;
		if (swap)
			PxSwap(x, y);

		if (x == 0 && y == 1) return createEdge(tet[2], tet[3], swap);
		if (x == 0 && y == 2) return createEdge(tet[3], tet[1], swap);
		if (x == 0 && y == 3) return createEdge(tet[1], tet[2], swap);
		if (x == 1 && y == 2) return createEdge(tet[0], tet[3], swap);
		if (x == 1 && y == 3) return createEdge(tet[2], tet[0], swap);
		if (x == 2 && y == 3) return createEdge(tet[0], tet[1], swap);

		return Edge(-1, -1);
	}

	bool removeEdgeByFlip(PxI32 edgeA, PxI32 edgeB, PxArray<PxI32>& faces, PxHashSet<PxI32>& hashset, PxArray<PxI32>& tetIndices,
		PxArray<Tetrahedron>& tets, PxArray<PxVec3d>& pts, PxArray<PxI32>& unusedTets, PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, 
		BaseTetAnalyzer* qualityAnalyzer = NULL)
	{
		//validateNeighborhood(tets, neighbors);

		PxArray<Edge> ringEdges;
		ringEdges.reserve(tetIndices.size());
		for (PxU32 i = 0; i < tetIndices.size(); ++i)
			ringEdges.pushBack(getOtherEdge(tets[tetIndices[i]], edgeA, edgeB));

		PxArray<PxI32> ring;
		buildRing(ringEdges, ring);
		if (ring.size() == 0)
			return false;

		PxArray<PxI32> ringCopy(ring);

		const PxVec3d& a = pts[edgeA];
		const PxVec3d& b = pts[edgeB];

		PxArray<Tetrahedron> newTets;
		newTets.reserve(ring.size() * 2);
		PxArray<Triangle> newFaces;
		while (ring.size() >= 3)
		{
			PxF64 shortestDist = DBL_MAX;
			PxI32 id = -1;
			for (PxI32 i = 0; i < PxI32(ring.size()); ++i)
			{
				const PxVec3d& s = pts[ring[(i - 1 + ring.size()) % ring.size()]];
				const PxVec3d& middle = pts[ring[i]];
				const PxVec3d& e = pts[ring[(i + 1) % ring.size()]];
				const PxF64 d = (s - e).magnitudeSquared();
				if (d < shortestDist)
				{
					const PxF64 vol1 = tetVolume(s, middle, e, b);
					const PxF64 vol2 = tetVolume(a, s, middle, e);
					if (vol1 > 0 && vol2 > 0)
					{
						shortestDist = d;
						id = i;
					}
				}
			}

			if (id < 0)
				return false;

			{
				const PxI32& s = ring[(id - 1 + ring.size()) % ring.size()];
				const PxI32& middle = ring[id];
				const PxI32& e = ring[(id + 1) % ring.size()];
				newTets.pushBack(Tetrahedron(s, middle, e, edgeB));
				newTets.pushBack(Tetrahedron(edgeA, s, middle, e));

				newFaces.pushBack(Triangle(s, middle, e));
				if (ring.size() > 3)
				{
					newFaces.pushBack(Triangle(s, e, edgeA));
					newFaces.pushBack(Triangle(s, e, edgeB));
				}
			}
			
			ring.remove(id);
		}

		//Analyze and decide if operation should be done
		if (qualityAnalyzer != NULL && !qualityAnalyzer->improved(qualityAnalyzer->quality(tetIndices), qualityAnalyzer->quality(newTets))) 
			return false;

		PxArray<PxI32> newTetIds;
		for (PxU32 i = 0; i < tetIndices.size(); ++i)
		{
			for (PxI32 j = 0; j < 4; ++j)
			{
				PxI32 id = 4 * tetIndices[i] + j;
				PxI32 neighborTet = neighbors[id] >> 2;
				if (neighborTet >= 0 && tetIndices.find(neighborTet) == tetIndices.end() && newTetIds.find(neighborTet) == newTetIds.end())
					newTetIds.pushBack(neighborTet);
			}
		}

	
		PxHashSet<PxI32> set;
		PxArray<PxI32> tetIds;
		collectTetsConnectedToVertex(faces, hashset, tets, neighbors, vertexToTet, edgeA, tetIds);
		addRange(set, tetIds);

		faces.forceSize_Unsafe(0);
		hashset.clear();
		tetIds.forceSize_Unsafe(0);
		collectTetsConnectedToVertex(faces, hashset, tets, neighbors, vertexToTet, edgeB, tetIds);

		addRange(set, tetIds);

		for (PxU32 i = 0; i < ringCopy.size(); ++i)
		{
			faces.forceSize_Unsafe(0);
			hashset.clear();
			tetIds.forceSize_Unsafe(0);
			collectTetsConnectedToVertex(faces, hashset, tets, neighbors, vertexToTet, ringCopy[i], tetIds);
			addRange(set, tetIds);
		}
		for (PxHashSet<PxI32>::Iterator iter = set.getIterator(); !iter.done(); ++iter)
		{
			PxI32 i = *iter;
			const Tetrahedron& neighborTet = tets[i];
			for (PxU32 j = 0; j < newFaces.size(); ++j)
				if (neighborTet.containsFace(newFaces[j]))
					return false;

			for (PxU32 j = 0; j < newTets.size(); ++j)
			{
				const Tetrahedron& t = newTets[j];
				if (Tetrahedron::identical(t, neighborTet))
					return false;
			}
		}

		for (PxU32 i = 0; i < tetIndices.size(); ++i)
		{
			tets[tetIndices[i]] = Tetrahedron(-1, -1, -1, -1);
			unusedTets.pushBack(tetIndices[i]);
		}

		PxU32 l = newTetIds.size();
		for (PxU32 i = 0; i < newTets.size(); ++i)
			newTetIds.pushBack(storeNewTet(tets, neighbors, newTets[i], unusedTets));

		fixNeighborhoodLocally(tetIndices, newTetIds, tets, neighbors); 
		tetIndices.clear();
		for (PxU32 i = l; i < newTetIds.size(); ++i)
			tetIndices.pushBack(newTetIds[i]);
		fixVertexToTet(vertexToTet, tetIndices, tets);

		return true;
	}

	bool DelaunayTetrahedralizer::removeEdgeByFlip(PxI32 edgeA, PxI32 edgeB, PxArray<PxI32>& tetIndices, BaseTetAnalyzer* qualityAnalyzer)
	{
		stackMemory.clear();
		return physx::Ext::removeEdgeByFlip(edgeA, edgeB, stackMemory.faces, stackMemory.hashSet, tetIndices, result, centeredNormalizedPoints, unusedTets, neighbors, vertexToTet, qualityAnalyzer);
	}

	void DelaunayTetrahedralizer::addLockedEdges(const PxArray<Triangle>& triangles)
	{
		for (PxU32 i = 0; i < triangles.size(); ++i)
		{
			const Triangle& t = triangles[i];
			lockedEdges.insert(key(t[0] + numAdditionalPointsAtBeginning, t[1] + numAdditionalPointsAtBeginning));
			lockedEdges.insert(key(t[0] + numAdditionalPointsAtBeginning, t[2] + numAdditionalPointsAtBeginning));
			lockedEdges.insert(key(t[1] + numAdditionalPointsAtBeginning, t[2] + numAdditionalPointsAtBeginning));
		}
	}

	void DelaunayTetrahedralizer::addLockedTriangles(const PxArray<Triangle>& triangles)
	{
		for (PxU32 i = 0; i < triangles.size(); ++i)
		{
			const Triangle& tri = triangles[i];
			lockedTriangles.insert(SortedTriangle(tri[0] + numAdditionalPointsAtBeginning, tri[1] + numAdditionalPointsAtBeginning, tri[2] + numAdditionalPointsAtBeginning));
		}
	}

	void previewFlip3to2(PxI32 tip1, PxI32 tip2, PxI32 reflexEdgeA, PxI32 reflexEdgeB, PxI32 nonReflexTrianglePoint, Tetrahedron& tet1, Tetrahedron& tet2)
	{
		// 3->2 flip
		tet1 = Tetrahedron(tip1, tip2, reflexEdgeA, nonReflexTrianglePoint);
		tet2 = Tetrahedron(tip2, tip1, reflexEdgeB, nonReflexTrianglePoint);
	}

	bool previewFlip2to3(PxI32 tet1Id, PxI32 tet2Id, const PxArray<PxI32>& neighbors, const PxArray<Tetrahedron>& tets, 
		PxI32 tip1, PxI32 tip2, Triangle tri, Tetrahedron& tet1, Tetrahedron& tet2, Tetrahedron& tet3)
	{
		// 2->3 flip
		tet1 = Tetrahedron(tip2, tip1, tri[0], tri[1]);
		tet2 = Tetrahedron(tip2, tip1, tri[1], tri[2]);
		tet3 = Tetrahedron(tip2, tip1, tri[2], tri[0]);

		PxI32 n1 = neighbors[4 * tet1Id + localFaceId(tets[tet1Id], tri[0], tri[1], tip1)];
		PxI32 n2 = neighbors[4 * tet2Id + localFaceId(tets[tet2Id], tri[0], tri[1], tip2)];
		if (n1 >= 0 && (n1 >> 2) == (n2 >> 2))
		{
			if (Tetrahedron::identical(tet1, tets[n1 >> 2]))
				return false;
		}

		PxI32 n3 = neighbors[4 * tet1Id + localFaceId(tets[tet1Id], tri[1], tri[2], tip1)];
		PxI32 n4 = neighbors[4 * tet2Id + localFaceId(tets[tet2Id], tri[1], tri[2], tip2)];
		if (n3 >= 0 && (n3 >> 2) == (n4 >> 2))
		{
			if (Tetrahedron::identical(tet2, tets[n3 >> 2]))
				return false;
		}

		PxI32 n5 = neighbors[4 * tet1Id + localFaceId(tets[tet1Id], tri[2], tri[0], tip1)];
		PxI32 n6 = neighbors[4 * tet2Id + localFaceId(tets[tet2Id], tri[2], tri[0], tip2)];
		if (n5 >= 0 && (n5 >> 2) == (n6 >> 2))
		{
			if (Tetrahedron::identical(tet3, tets[n5 >> 2]))
				return false;
		}
		return true;
	}

	bool previewFlip(const PxArray<PxVec3d>& points, const PxArray<Tetrahedron>& tets, const PxArray<PxI32>& neighbors, PxI32 faceId, 
		PxArray<Tetrahedron>& newTets, PxArray<PxI32>& removedTets,
		const PxHashSet<SortedTriangle, TriangleHash>& lockedFaces, const PxHashSet<PxU64>& lockedEdges)
	{
		newTets.clear();
		removedTets.clear();

		PxI32 neighborPointer = neighbors[faceId];
		if (neighborPointer < 0)
			return false;

		PxI32 tet1Id = faceId >> 2;
		const PxI32* localTriangle1 = neighborFaces[faceId & 3];
		Tetrahedron tet1 = tets[tet1Id];
		Triangle tri(tet1[localTriangle1[0]], tet1[localTriangle1[1]], tet1[localTriangle1[2]]);
		PxI32 localTip1 = tetTip[faceId & 3];
		PxI32 localTip2 = tetTip[neighborPointer & 3];
		PxI32 tip1 = tet1[localTip1];

		PxI32 tet2Id = neighborPointer >> 2;
		Tetrahedron tet2 = tets[tet2Id];
		PxI32 tip2 = tet2[localTip2];
		
		if (!tet2.contains(tri[0]) || !tet2.contains(tri[1]) || !tet2.contains(tri[2]))
		{
			return false;
		}

		PxI32 face1 = -1;
		PxI32 face2 = -1;
		PxI32 numReflexEdges = 0;
		PxI32 reflexEdgeA = -1;
		PxI32 reflexEdgeB = -1;
		PxI32 nonReflexTrianglePoint = -1;
		PxF64 ab = orient3D(points[tri[0]], points[tri[1]], points[tip1], points[tip2]);
		if (ab < 0) { ++numReflexEdges; face1 = localFaceId(localTriangle1[0], localTriangle1[1], localTip1); reflexEdgeA = tri[0]; reflexEdgeB = tri[1]; nonReflexTrianglePoint = tri[2]; face2 = localFaceId(tet2, tri[0], tri[1], tip2); }
		PxF64 bc = orient3D(points[tri[1]], points[tri[2]], points[tip1], points[tip2]);
		if (bc < 0) { ++numReflexEdges; face1 = localFaceId(localTriangle1[1], localTriangle1[2], localTip1); reflexEdgeA = tri[1]; reflexEdgeB = tri[2]; nonReflexTrianglePoint = tri[0]; face2 = localFaceId(tet2, tri[1], tri[2], tip2); }
		PxF64 ca = orient3D(points[tri[2]], points[tri[0]], points[tip1], points[tip2]);
		if (ca < 0) { ++numReflexEdges; face1 = localFaceId(localTriangle1[2], localTriangle1[0], localTip1); reflexEdgeA = tri[2]; reflexEdgeB = tri[0]; nonReflexTrianglePoint = tri[1]; face2 = localFaceId(tet2, tri[2], tri[0], tip2); }

		if (numReflexEdges == 0)
		{
			if (!faceIsLocked(lockedFaces, tri[0], tri[1], tri[2]))
			{
				Tetrahedron tet3;
				if (previewFlip2to3(tet1Id, tet2Id, neighbors, tets, tip1, tip2, tri, tet1, tet2, tet3))
				{
					newTets.pushBack(tet1); newTets.pushBack(tet2); newTets.pushBack(tet3);
					removedTets.pushBack(tet1Id); removedTets.pushBack(tet2Id);
					return true;
				}
				else return true;
			}
		}
		else if (numReflexEdges == 1)
		{
			PxI32 candidate1 = neighbors[4 * tet1Id + face1] >> 2;
			PxI32 candidate2 = neighbors[4 * tet2Id + face2] >> 2;
			if (candidate1 == candidate2 && candidate1 >= 0)
			{
				if (!edgeIsLocked(lockedEdges, reflexEdgeA, reflexEdgeB) &&
					!faceIsLocked(lockedFaces, reflexEdgeA, reflexEdgeB, nonReflexTrianglePoint) &&
					!faceIsLocked(lockedFaces, reflexEdgeA, reflexEdgeB, tip1) &&
					!faceIsLocked(lockedFaces, reflexEdgeA, reflexEdgeB, tip2))
				{
					previewFlip3to2(tip1, tip2, reflexEdgeA, reflexEdgeB, nonReflexTrianglePoint, tet1, tet2);
					newTets.pushBack(tet1); newTets.pushBack(tet2);
					removedTets.pushBack(tet1Id); removedTets.pushBack(tet2Id); removedTets.pushBack(candidate1);
					return true;;
				}
			}
		}
		else if (numReflexEdges == 2)
		{
			//Cannot do anything
		}
		else if (numReflexEdges == 3)
		{
		}
		return true;
	}

	void collectCommonFaces(const PxArray<PxI32>& a, const PxArray<PxI32>& b, const PxArray<PxI32>& neighbors, PxArray<PxI32>& result)
	{
		result.clear();
		for (PxU32 i = 0; i < a.size(); ++i)
		{
			PxI32 tetId = a[i];
			for (int j = 0; j < 4; ++j)
			{
				int id = 4 * tetId + j;
				if (b.find(neighbors[id] >> 2) != b.end())
				{
					if (result.find(id) == result.end())
						result.pushBack(id);
				}
			}
		}
	}

	void previewFlip2to3(PxI32 tip1, PxI32 tip2, const Triangle& tri, Tetrahedron& tet1, Tetrahedron& tet2, Tetrahedron& tet3)
	{
		// 2->3 flip
		tet1 = Tetrahedron(tip2, tip1, tri[0], tri[1]);
		tet2 = Tetrahedron(tip2, tip1, tri[1], tri[2]);
		tet3 = Tetrahedron(tip2, tip1, tri[2], tri[0]);
	}

	bool DelaunayTetrahedralizer::recoverEdgeByFlip(PxI32 eStart, PxI32 eEnd, RecoverEdgeMemoryCache& cache)
	{
		PxArray<PxI32>& tetsStart = cache.resultStart;
		PxArray<PxI32>& tetsEnd = cache.resultEnd;
		collectTetsConnectedToVertex(cache.facesStart, cache.tetsDoneStart, eStart, tetsStart);
		collectTetsConnectedToVertex(cache.facesEnd, cache.tetsDoneEnd, eEnd, tetsEnd);

		for (PxU32 i = 0; i < tetsEnd.size(); ++i)
		{
			const Tetrahedron& tet = result[tetsEnd[i]];
			if (tet.contains(eStart))
				return true;
		}

		PxArray<PxI32> commonFaces;
		collectCommonFaces(tetsStart, tetsEnd, neighbors, commonFaces);

		if (commonFaces.size() > 0)
		{
			for (PxU32 i = 0; i < commonFaces.size(); ++i)
			{
				PxI32 faceId = commonFaces[i];
				PxI32 tetId = faceId >> 2;
				const Tetrahedron& tet = result[tetId];
				const PxI32* local = neighborFaces[faceId & 3];
				Triangle tri(tet[local[0]], tet[local[1]], tet[local[2]]);

				if (tri.contains(eStart) || tri.contains(eEnd))
					continue;

				int n = neighbors[faceId];
				int tetId2 = n >> 2;
				int tip1 = tet[tetTip[faceId & 3]];
				int tip2 = result[tetId2][tetTip[n & 3]];

				Tetrahedron tet1, tet2, tet3;
				previewFlip2to3(tip1, tip2, tri, tet1, tet2, tet3);

				if (tetVolume(tet1, centeredNormalizedPoints) > 0 &&
					tetVolume(tet2, centeredNormalizedPoints) > 0 &&
					tetVolume(tet3, centeredNormalizedPoints) > 0)
				{
					PxArray<PxI32> affectedFaces;
					if (flip2to3(tetId, tetId2, neighbors, vertexToTet, result, unusedTets, affectedFaces, tip1, tip2, tri))
						return true;
				}
			}
		}
		return false;
	}

	void insert(PxArray<int>& list, int a, int b, int id)
	{
		for (PxI32 i = 0; i < PxI32(list.size()); ++i)
			if (list[i] == a || list[i] == b)
			{
				list.pushBack(-1);
				for (PxI32 j = list.size() - 2; j > i; --j)
				{
					list[j + 1] = list[j];
				}
				list[i + 1] = id;
				return;
			}
		PX_ASSERT(false);
	}

	void DelaunayTetrahedralizer::generateTetmeshEnforcingEdges(const PxArray<PxVec3d>& trianglePoints, const PxArray<Triangle>& triangles, PxArray<PxArray<PxI32>>& allEdges,
		PxArray<PxArray<PxI32>>& pointToOriginalTriangle, 
		PxArray<PxVec3d>& points, PxArray<Tetrahedron>& finalTets)
	{
		clearLockedEdges();
		clearLockedTriangles();
		addLockedEdges(triangles);
		addLockedTriangles(triangles);
		
		points.resize(trianglePoints.size());
		for (PxU32 i = 0; i < trianglePoints.size(); ++i)
			points[i] = trianglePoints[i];
		
		insertPoints(points, 0, points.size());

		allEdges.clear();
		allEdges.reserve(lockedEdges.size());
		PxHashMap<PxU64, PxU32> edgeToIndex;
		PxArray<PxI32> list;
		for (PxHashSet<PxU64>::Iterator iter = lockedEdges.getIterator(); !iter.done(); ++iter)
		{
			edgeToIndex.insert(*iter, allEdges.size());
			
			list.forceSize_Unsafe(0);
			list.pushBack(PxI32((*iter) >> 32));
			list.pushBack(PxI32((*iter)));
			allEdges.pushBack(list);
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
	
		for (PxU32 i = 0;i < points.size(); ++i) {
			PxArray<PxI32>& tempList = pointToOriginalTriangle[i];
			PxSort(tempList.begin(), tempList.size());
		}

		PxArray<PxI32> intersection;
		RecoverEdgeMemoryCache cache;
		while (true) {
			PxArray<PxU64> copy;
			copy.reserve(lockedEdges.size());
			for (PxHashSet<PxU64>::Iterator e = lockedEdges.getIterator(); !e.done(); ++e)
				copy.pushBack(*e);			
			PxI32 missing = 0;
			for (PxU32 k = 0; k < copy.size(); ++k)
			{
				PxU64 e = copy[k];
				PxI32 a = PxI32(e >> 32);
				PxI32 b = PxI32(e);
				if (!recoverEdgeByFlip(a, b, cache))
				{
					PxU32 id = centeredNormalizedPoints.size();
					PxU32 i = edgeToIndex[e];
					if (allEdges[i].size() < 10)
					{
						PxVec3d p = (centeredNormalizedPoints[a] + centeredNormalizedPoints[b]) * 0.5;					
						points.pushBack(p);
						insertPoints(points, points.size() - 1, points.size());

						intersection.forceSize_Unsafe(0);
						intersectionOfSortedLists(pointToOriginalTriangle[a - numAdditionalPointsAtBeginning], pointToOriginalTriangle[b - numAdditionalPointsAtBeginning], intersection);
						pointToOriginalTriangle.pushBack(intersection);

						insert(allEdges[i], a, b, id);
						edgeToIndex.insert(key(a, id), i);
						edgeToIndex.insert(key(b, id), i);
						edgeToIndex.erase(e);

						lockedEdges.erase(e);
						lockedEdges.insert(key(a, id));
						lockedEdges.insert(key(b, id));
						++missing;
					}
				}
			}
			if (missing == 0)
				break;
		}

		for (PxU32 i = 0; i < allEdges.size(); ++i)
		{
			PxArray<PxI32>& tempList = allEdges[i];
			for (PxU32 j = 0; j < tempList.size(); ++j)
				tempList[j] -= numAdditionalPointsAtBeginning;
		}

		exportTetrahedra(finalTets);
	}


	PxI32 optimizeByFlipping(PxArray<PxI32>& faces, PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet,
		const PxArray<PxVec3d>& points, PxArray<Tetrahedron>& tets, PxArray<PxI32>& unusedTets, const BaseTetAnalyzer& qualityAnalyzer,
		PxArray<PxI32>& affectedFaces,
		const PxHashSet<SortedTriangle, TriangleHash>& lockedFaces, const PxHashSet<PxU64>& lockedEdges)
	{
		PxI32 counter = 0;
		while (faces.size() > 0)
		{
			PxI32 faceId = faces.popBack();
			if (faceId < 0)
				continue;

			PxArray<Tetrahedron> newTets;
			PxArray<PxI32> removedTets;
			if (!previewFlip(points, tets, neighbors, faceId, newTets, removedTets, lockedFaces, lockedEdges))
				continue;

			if (!qualityAnalyzer.improved(qualityAnalyzer.quality(removedTets), qualityAnalyzer.quality(newTets)))
				continue;

			affectedFaces.clear();
			flip(points, tets, neighbors, vertexToTet, faceId, unusedTets, affectedFaces, lockedFaces, lockedEdges);
			
			for (PxU32 j = 0; j < affectedFaces.size(); ++j)
				if (faces.find(affectedFaces[j]) == faces.end())
					faces.pushBack(affectedFaces[j]);

			++counter;
		}
		return counter;
	}

	bool DelaunayTetrahedralizer::optimizeByFlipping(PxArray<PxI32>& affectedFaces, const BaseTetAnalyzer& qualityAnalyzer)
	{
		PxArray<PxI32> stack;
		for (PxU32 i = 0; i < affectedFaces.size(); ++i)
			stack.pushBack(affectedFaces[i]);
		PxI32 counter = physx::Ext::optimizeByFlipping(stack, neighbors, vertexToTet, centeredNormalizedPoints, result, unusedTets, qualityAnalyzer, affectedFaces, lockedTriangles, lockedEdges);
		return counter > 0;
	}


	void insertPointIntoEdge(PxI32 newPointIndex, PxI32 edgeA, PxI32 edgeB, PxArray<Tetrahedron>& tets,
		PxArray<PxI32>& neighbors, PxArray<PxI32>& vertexToTet, PxArray<PxI32>& unusedTets, PxArray<PxI32>* affectedFaces, PxArray<PxI32>& affectedTets)
	{
		PxU32 l = affectedTets.size();
		if (l == 0)
			return;

		PxArray<PxI32> removedTets;
		for (PxU32 i = 0; i < affectedTets.size(); ++i)
			removedTets.pushBack(affectedTets[i]);

		PxArray<PxI32> newTets;
		for (PxU32 i = 0; i < affectedTets.size(); ++i)
			newTets.pushBack(affectedTets[i]);

		for (PxU32 i = 0; i < l; ++i)
		{
			for (PxI32 j = 0; j < 4; ++j)
			{
				PxI32 id = 4 * affectedTets[i] + j;
				PxI32 neighborTet = neighbors[id] >> 2;
				if (neighborTet >= 0 && affectedTets.find(neighborTet) == affectedTets.end())
					affectedTets.pushBack(neighborTet);
			}
		}
		PxU32 l2 = affectedTets.size();

		for (PxU32 i = 0; i < l; ++i)
		{
			PxI32 id = affectedTets[i];
			const Tetrahedron& tet = tets[id];

			Edge oppositeEdge = getOtherEdge(tet, edgeA, edgeB);

			tets[id] = Tetrahedron(oppositeEdge.first, oppositeEdge.second, edgeA, newPointIndex);
			PxI32 j = storeNewTet(tets, neighbors, Tetrahedron(oppositeEdge.first, oppositeEdge.second, newPointIndex, edgeB), unusedTets);
			affectedTets.pushBack(j);
			newTets.pushBack(j);
		}

		fixNeighborhoodLocally(removedTets, affectedTets, tets, neighbors, affectedFaces);
		fixVertexToTet(vertexToTet, newTets, tets);
		affectedTets.removeRange(l, l2 - l);
	}

	void DelaunayTetrahedralizer::insertPointIntoEdge(PxI32 newPointIndex, PxI32 edgeA, PxI32 edgeB, PxArray<PxI32>& affectedTets, BaseTetAnalyzer* qualityAnalyzer)
	{
		PxArray<PxI32> affectedFaces;
		physx::Ext::insertPointIntoEdge(newPointIndex, edgeA, edgeB, result, neighbors, vertexToTet, unusedTets, &affectedFaces, affectedTets);

		if (qualityAnalyzer != NULL)
			optimizeByFlipping(affectedFaces, *qualityAnalyzer);
	}


	bool containsAll(const PxArray<PxI32>& list, const PxArray<PxI32>& itemsToBePresentInList)
	{
		for (PxU32 i = 0; i < itemsToBePresentInList.size(); ++i)
			if (list.find(itemsToBePresentInList[i]) == list.end())
				return false;
		return true;
	}

	bool optimizeByCollapsing(DelaunayTetrahedralizer& del, const PxArray<EdgeWithLength>& edges,
		PxArray<PxArray<PxI32>>& pointToOriginalTriangle, PxI32 numFixPoints, BaseTetAnalyzer* qualityAnalyzer)
	{
		PxI32 l = PxI32(pointToOriginalTriangle.size());

		bool success = false;
		PxArray<PxI32> tetsA;
		PxArray<PxI32> tetsB;

		for (PxU32 i = 0; i < edges.size(); ++i)
		{
			const EdgeWithLength& e = edges[i];

			tetsA.forceSize_Unsafe(0);
			del.collectTetsConnectedToVertex(e.A, tetsA);
			if (tetsA.empty()) continue; 
			
			bool edgeExists = false;
			for (PxU32 j = 0; j < tetsA.size(); ++j)
			{
				const Tetrahedron& tet = del.tetrahedron(tetsA[j]);
				if (tet.contains(e.B))
				{
					edgeExists = true;
					break;
				}
			}
			if (!edgeExists)
				continue;

			tetsB.forceSize_Unsafe(0);
			del.collectTetsConnectedToVertex(e.B, tetsB);
			if (tetsB.empty()) continue;					   

			PxF64 firstQuality = 0, secondQuality = 0;
			bool firstOptionValid = e.B >= numFixPoints && (e.B >= l || (e.A < l && containsAll(pointToOriginalTriangle[e.A], pointToOriginalTriangle[e.B]))) &&
				del.canCollapseEdge(e.A, e.B, tetsA, tetsB, firstQuality, 0, qualityAnalyzer);
			bool secondOptionValid = e.A >= numFixPoints && (e.A >= l || (e.B < l && containsAll(pointToOriginalTriangle[e.B], pointToOriginalTriangle[e.A]))) &&
				del.canCollapseEdge(e.B, e.A, tetsB, tetsA, secondQuality, 0, qualityAnalyzer);

			if (qualityAnalyzer != NULL && firstOptionValid && secondOptionValid)
			{
				if (qualityAnalyzer->improved(firstQuality, secondQuality))
					firstOptionValid = false;
				else
					secondOptionValid = false;
			}
			if (firstOptionValid)
			{
				if (e.B >= numFixPoints)
				{
					del.collapseEdge(e.A, e.B, tetsA, tetsB);
					if (e.B < l)
						pointToOriginalTriangle[e.B].clear();
					success = true;
				}
			}
			else if (secondOptionValid)
			{
				if (e.A >= numFixPoints)
				{
					del.collapseEdge(e.B, e.A, tetsB, tetsA);
					if (e.A < l)
						pointToOriginalTriangle[e.A].clear();
					success = true;
				}
			}
		}
		return success;
	}

	bool optimizeBySwapping(DelaunayTetrahedralizer& del, const PxArray<EdgeWithLength>& edges,
		const PxArray<PxArray<PxI32>>& pointToOriginalTriangle, BaseTetAnalyzer* qualityAnalyzer)
	{
		PxI32 l = PxI32(pointToOriginalTriangle.size());

		bool success = false;
		PxArray<PxI32> tetIds;
		for (PxU32 i = 0; i < edges.size(); ++i)
		{
			const EdgeWithLength& e = edges[i];

			const bool isInteriorEdge = e.A >= l || e.B >= l || !intersectionOfSortedListsContainsElements(pointToOriginalTriangle[e.A], pointToOriginalTriangle[e.B]);
			if (isInteriorEdge)
			{
				tetIds.forceSize_Unsafe(0);
				del.collectTetsConnectedToEdge(e.A, e.B, tetIds);
				if (tetIds.size() <= 2)
					continue; //This would mean we have a surface edge

				if (del.removeEdgeByFlip(e.A, e.B, tetIds, qualityAnalyzer))
					success = true;
			}
		}
		return success;
	}

	void patternSearchOptimize(PxI32 pointToModify, PxF64 step, PxArray<PxVec3d>& points, BaseTetAnalyzer& score, const PxArray<PxI32>& tetrahedra, PxF64 minStep)
	{
		const PxVec3d dir[] = { PxVec3d(1.0, 0.0, 0.0), PxVec3d(-1.0, 0.0, 0.0), PxVec3d(0.0, 1.0, 0.0), PxVec3d(0.0, -1.0, 0.0), PxVec3d(0.0, 0.0, 1.0), PxVec3d(0.0, 0.0, -1.0), };
		const PxI32 oppositeDir[] = { 1, 0, 3, 2, 5, 4 };
		PxF64 currentScore = score.quality(tetrahedra); // score();
		PxVec3d p = points[pointToModify];
		PxI32 skip = -1;

		PxI32 maxIter = 100;
		PxI32 iter = 0;
		while (step > minStep && iter < maxIter)
		{
			PxF64 best = currentScore;
			PxI32 id = -1;
			for (PxI32 i = 0; i < 6; ++i)
			{
				if (i == skip)
					continue; //That point was the best before current iteration - it cannot be the best anymore

				points[pointToModify] = p + dir[i] * step;
				PxF64 s = score.quality(tetrahedra); // score();
				if (score.improved(best, s))
				{
					best = s;
					id = i;
				}
			}
			if (id >= 0)
			{
				p = p + dir[id] * step;
				points[pointToModify] = p;
				currentScore = best;
				skip = oppositeDir[id];
			}
			else
			{
				points[pointToModify] = p;
				step = step * 0.5;
				skip = -1;
			}
			++iter;
		}
	}

	bool optimizeBySplitting(DelaunayTetrahedralizer& del, const PxArray<EdgeWithLength>& edges, const PxArray<PxArray<PxI32>>& pointToOriginalTriangle,
		PxI32 maxPointsToInsert, bool sortByQuality, BaseTetAnalyzer* qualityAnalyzer, PxF64 qualityThreshold)
	{
		const PxF64 qualityThresholdPow3 = qualityThreshold * qualityThreshold * qualityThreshold;

		PxArray<PxF64> tetQualityBuffer;
		tetQualityBuffer.resize(del.numTetrahedra());
		for (PxU32 i = 0; i < del.numTetrahedra(); ++i) {
			const Tetrahedron& tet = del.tetrahedron(i);
			if (tet[0] >= 0)
				tetQualityBuffer[i] = evaluateAmipsEnergyPow3(del.point(tet[0]), del.point(tet[1]), del.point(tet[2]), del.point(tet[3]));
		}

		PxI32 l = PxI32(pointToOriginalTriangle.size());
		PxArray<SplitEdge> edgesToSplit;
		PxArray<PxI32> tetIds;
		for (PxI32 i = edges.size() - 1; i >= 0; --i)
		{
			const EdgeWithLength& e = edges[i];

			const bool isInteriorEdge = e.A >= l || e.B >= l || !intersectionOfSortedListsContainsElements(pointToOriginalTriangle[e.A], pointToOriginalTriangle[e.B]);
			if (isInteriorEdge)
			{
				tetIds.forceSize_Unsafe(0);
				del.collectTetsConnectedToEdge(e.A, e.B, tetIds);
				if (tetIds.size() <= 2)
					continue; //This would mean we got a surface edge...
				PxF64 max = 0;
				for (PxU32 j = 0; j < tetIds.size(); ++j)
				{
					if (tetQualityBuffer[tetIds[j]] > max)
						max = tetQualityBuffer[tetIds[j]];
				}
				if (max > qualityThresholdPow3)
					edgesToSplit.pushBack(SplitEdge(e.A, e.B, max, (del.point(e.A) - del.point(e.B)).magnitudeSquared(), isInteriorEdge));
			}
		}

		if (sortByQuality) 
		{
			PxSort(edgesToSplit.begin(), edgesToSplit.size(), PxGreater<SplitEdge>());
		}
		PxI32 counter = 0;
		PxArray<PxI32> tetsConnectedToEdge;
		PxArray<PxI32> tetsConnectedToVertex;
		for (PxU32 i = 0; i < edgesToSplit.size(); ++i)
		{
			const SplitEdge& e = edgesToSplit[i];
			if (i > 0 && edgesToSplit[i - 1].Q == e.Q)
				continue;

			if (counter == maxPointsToInsert)
				break;

			PxU32 id = del.addPoint((del.point(e.A) + del.point(e.B)) * 0.5);
			tetsConnectedToEdge.forceSize_Unsafe(0);
			
			del.collectTetsConnectedToEdge(e.A, e.B, tetsConnectedToEdge);
			del.insertPointIntoEdge(id, e.A, e.B, tetsConnectedToEdge, qualityAnalyzer);

			if (e.InteriorEdge && qualityAnalyzer)
			{
				tetsConnectedToVertex.forceSize_Unsafe(0);
				del.collectTetsConnectedToVertex(id, tetsConnectedToVertex);
				patternSearchOptimize(id, e.L, del.points(), *qualityAnalyzer, tetsConnectedToVertex, 0.01 * e.L);
			}

			++counter;
		}
		return edgesToSplit.size() > 0;
	}

	void updateEdges(PxArray<EdgeWithLength>& edges, PxHashSet<PxU64>& tetEdges, const PxArray<Tetrahedron>& tets, const PxArray<PxVec3d>& points)
	{
		edges.clear();
		tetEdges.clear();
		for (PxU32 i = 0; i < tets.size(); ++i)
		{
			const Tetrahedron& tet = tets[i];
			if (tet[0] < 0) continue;
			tetEdges.insert(key(tet[0], tet[1]));
			tetEdges.insert(key(tet[0], tet[2]));
			tetEdges.insert(key(tet[0], tet[3]));
			tetEdges.insert(key(tet[1], tet[2]));
			tetEdges.insert(key(tet[1], tet[3]));
			tetEdges.insert(key(tet[2], tet[3]));
		}
		for (PxHashSet<PxU64>::Iterator iter = tetEdges.getIterator(); !iter.done(); ++iter)
		{
			PxU64 e = *iter;
			PxI32 a = PxI32(e >> 32);
			PxI32 b = PxI32(e);
			edges.pushBack(EdgeWithLength(a, b, (points[a] - points[b]).magnitudeSquared()));
		}
		PxSort(edges.begin(), edges.size(), PxLess<EdgeWithLength>());
	}

	void optimize(DelaunayTetrahedralizer& del, PxArray<PxArray<PxI32>>& pointToOriginalTriangle, PxI32 numFixPoints,
		PxArray<PxVec3d>& optimizedPoints, PxArray<Tetrahedron>& optimizedTets, PxI32 numPasses)
	{
		PxHashSet<PxU64> tetEdges;
		PxArray<EdgeWithLength> edges;
		updateEdges(edges, tetEdges, del.tetrahedra(), del.points());

		MinimizeMaxAmipsEnergy qualityAnalyzer(del.points(), del.tetrahedra());
		//MaximizeMinTetVolume qualityAnalyzer(del.points(), del.tetrahedra());

		optimizedTets = PxArray<Tetrahedron>(del.tetrahedra());
		optimizedPoints = PxArray<PxVec3d>(del.points());

		PxF64 minVolBefore = minAbsTetVolume(del.points(), del.tetrahedra());
		PxF64 maxEnergyBefore = maxEnergy(del.points(), del.tetrahedra());
		PxI32 numPointsToInsertPerPass = PxMax(20, PxI32(0.05 * del.numPoints()));
		for (PxI32 j = 0; j < numPasses; ++j)
		{
			//(1) Splitting
			updateEdges(edges, tetEdges, del.tetrahedra(), del.points());
			optimizeBySplitting(del, edges, pointToOriginalTriangle, numPointsToInsertPerPass, true, &qualityAnalyzer);
				
			//(3) Swapping
			updateEdges(edges, tetEdges, del.tetrahedra(), del.points());
			optimizeBySwapping(del, edges, pointToOriginalTriangle, &qualityAnalyzer);
			
			//(2) Collapsing
			updateEdges(edges, tetEdges, del.tetrahedra(), del.points());
			optimizeByCollapsing(del, edges, pointToOriginalTriangle, numFixPoints, &qualityAnalyzer);
			
			//(4) Smoothing
			//Note done here - seems not to improve the quality that much

			PxF64 energy = maxEnergy(del.points(), del.tetrahedra());
			PxF64 minVol = minAbsTetVolume(del.points(), del.tetrahedra());

			if (energy / minVol >= maxEnergyBefore / minVolBefore/*minVol <= minVolBefore*/)
			{
				del.initialize(optimizedPoints, optimizedTets);

				bool swapped = true, collapsed = true;
				PxI32 maxReductionLoops = 30;
				PxI32 counter = 0;
				while (swapped || collapsed)
				{
					//(3) Swapping
					updateEdges(edges, tetEdges, del.tetrahedra(), del.points());
					swapped = optimizeBySwapping(del, edges, pointToOriginalTriangle, &qualityAnalyzer);

					//(2) Collapsing
					updateEdges(edges, tetEdges, del.tetrahedra(), del.points());
					collapsed = optimizeByCollapsing(del, edges, pointToOriginalTriangle, numFixPoints, &qualityAnalyzer);

					if (counter >= maxReductionLoops)
						break;

					++counter;
				}

				optimizedTets = PxArray<Tetrahedron>(del.tetrahedra());
				optimizedPoints = PxArray<PxVec3d>(del.points());
				return;
			}

			optimizedTets = PxArray<Tetrahedron>(del.tetrahedra());
			optimizedPoints = PxArray<PxVec3d>(del.points());

			minVolBefore = minVol;
			maxEnergyBefore = energy;
		}
	}
}
}
