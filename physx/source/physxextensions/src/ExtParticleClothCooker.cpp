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

#include "extensions/PxParticleClothCooker.h"

#include "foundation/PxArray.h"
#include "foundation/PxSort.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxHashSet.h"
#include "GuInternal.h"

namespace physx
{
namespace ExtGpu
{

namespace
{

struct Edge
{
	Edge(PxU32 v0, PxU32 v1, PxU32 triangle0, PxU32 triangle1, bool isLongest0, bool isLongest1)
	{
		a = PxMin(v0, v1);
		b = PxMax(v0, v1);
		triangleA = triangle0;
		triangleB = triangle1;
		longestA = isLongest0;
		longestB = isLongest1;
		PX_ASSERT_WITH_MESSAGE(a != b, "PxCreateInflatableFromMesh encountered a degenerate edge inside a triangle.");
	}
	PxU32 a, b;
	PxU32 triangleA, triangleB;
	bool longestA, longestB; //true if it is the longest edge of triangle

	bool operator<(const Edge& other) const
	{
		if(a == other.a)
			return b < other.b;
		return a < other.a;
	}
	bool operator==(const Edge& other) const
	{
		if(a == other.a)
			return b == other.b;
		return false;
	}
};

class EdgeHash
{
public:
	uint32_t operator()(const Edge& e) const
	{
		return PxComputeHash(e.a) ^ PxComputeHash(e.b);
	}
	bool equal(const Edge& e0, const Edge& e1) const
	{
		return e0.a == e1.a && e0.b == e1.b;
	}
};

class EdgeSet : public PxHashSet<Edge, EdgeHash>
{
public:
	typedef PxHashSet<Edge, EdgeHash> Base;
	typedef Base::Iterator Iterator;

	void insert(const Edge& newEdge)
	{
		PX_ASSERT(newEdge.a < newEdge.b);
		bool exists;
		Edge* edge = mBase.create(newEdge, exists);
		if (!exists)
		{
			PX_PLACEMENT_NEW(edge, Edge)(newEdge);
		}
		else
		{
			PX_ASSERT_WITH_MESSAGE(edge->triangleB == 0xffffffff, "Edge with more than 2 triangles found in PxCreateInflatableFromMesh");
			edge->triangleB = newEdge.triangleA; //Add triangle info from duplicate to Unique edge
			edge->longestB = newEdge.longestA;
		}
	}
};

template<typename T>
void partialSum(T* begin, T* end, T* dest)
{
	*dest = *begin;
	dest++;
	begin++;
	while(begin!=end)
	{
		*dest = dest[-1] + *begin;
		dest++;
		begin++;
	}
}

/*
outAdjacencies is a list of adjacent vertices in outAdjacencies
outAdjacencyIndices is a list of indices to quickly find adjacent vertices in outAdjacencies.

all the adjacent vertices to vertex V are stored in outAdjacencies starting at outAdjacencyIndices[V] and ending at outAdjacencyIndices[V+1]
so the first vertex is outAdjacencies[outAdjacencyIndices[V]], and the last one is outAdjacencies[outAdjacencyIndices[V+1]-1]

*/
void gatherAdjacencies(PxArray<PxU32>& outAdjacencyIndices, PxArray<PxU32>& outAdjacencies,
	PxU32 vertexCount, PxArray<Edge> const& inUniqueEdges, bool ignoreDiagonals = false
)
{
	PX_ASSERT(outAdjacencyIndices.size() == 0);
	PX_ASSERT(outAdjacencies.size() == 0);

	outAdjacencyIndices.resize(vertexCount+1, 0);

	//calculate valency
	for(PxU32 i = 0; i < inUniqueEdges.size(); i++)
	{
		const Edge& edge = inUniqueEdges[i];
		if(ignoreDiagonals && edge.longestA && edge.longestB)
			continue;
		outAdjacencyIndices[edge.a]++;
		outAdjacencyIndices[edge.b]++;
	}

	partialSum(outAdjacencyIndices.begin(), outAdjacencyIndices.end(), outAdjacencyIndices.begin());
	outAdjacencyIndices.back() = outAdjacencyIndices[vertexCount-1];
	outAdjacencies.resize(outAdjacencyIndices.back(),0xffffffff);

	for(PxU32 i = 0; i < inUniqueEdges.size(); i++)
	{
		const Edge& edge = inUniqueEdges[i];
		if(ignoreDiagonals && edge.longestA && edge.longestB)
			continue;
		outAdjacencyIndices[edge.a]--;
		outAdjacencies[outAdjacencyIndices[edge.a]]=edge.b;
		outAdjacencyIndices[edge.b]--;
		outAdjacencies[outAdjacencyIndices[edge.b]] = edge.a;
	}
}

template <typename T, typename A>
A MaxArg(T const& vA, T const& vB, A const& aA, A const& aB)
{
	if(vA > vB)
		return aA;
	return aB;
}

PxU32 GetOppositeVertex(PxU32* inTriangleIndices, PxU32 triangleIndex, PxU32 a, PxU32 b)
{
	for(int i = 0; i<3; i++)
	{
		if(inTriangleIndices[triangleIndex+i] != a && inTriangleIndices[triangleIndex + i] !=b)
			return inTriangleIndices[triangleIndex + i];
	}
	PX_ASSERT_WITH_MESSAGE(0, "Degenerate Triangle found in PxCreateInflatableFromMesh");
	return 0;
}

Edge GetAlternateDiagonal(Edge const& edge, PxU32* inTriangleIndices)
{
	PxU32 vA = GetOppositeVertex(inTriangleIndices, edge.triangleA, edge.a, edge.b);
	PxU32 vB = GetOppositeVertex(inTriangleIndices, edge.triangleB, edge.a, edge.b);
	bool longestA = true;
	bool longestB = true;
	PxU32 tA = 0xffffffff;
	PxU32 tB = 0xffffffff;
	return Edge(vA, vB, tA, tB, longestA, longestB);
}
} //namespace

class PxParticleClothCookerImpl : public PxParticleClothCooker, public PxUserAllocated
{
public:
	PxParticleClothCookerImpl(PxU32 vertexCount, physx::PxVec4* inVertices, PxU32 triangleIndexCount, PxU32* inTriangleIndices,
		PxU32 constraintTypeFlags, PxVec3 verticalDirection, PxReal bendingConstraintMaxAngle)
		:
		mVertexCount(vertexCount),
		mVertices(inVertices),
		mTriangleIndexCount(triangleIndexCount),
		mTriangleIndices(inTriangleIndices),
		mConstraintTypeFlags(constraintTypeFlags),
		mVerticalDirection(verticalDirection),
		mBendingConstraintMaxAngle(bendingConstraintMaxAngle)
	{
	}
	virtual void release()
	{
		PX_DELETE_THIS;
	}

	/**
	\brief generate the constraint and triangle per vertex information.
	*/
	virtual void cookConstraints(const PxParticleClothConstraint* constraints, const PxU32 numConstraints);
	virtual PxU32* getTriangleIndices() { return mTriangleIndexBuffer.begin(); }
	virtual PxU32 getTriangleIndicesCount() { return mTriangleIndexBuffer.size(); }
	virtual PxParticleClothConstraint* getConstraints() { return mConstraintBuffer.begin(); }
	virtual PxU32 getConstraintCount() { return mConstraintBuffer.size(); }

	/**
	\brief Computes the volume of a closed mesh and the contraintScale. Expects vertices in local space - 'close' to origin.
	*/
	virtual void calculateMeshVolume();
	virtual float getMeshVolume() {return mMeshVolume;}

private:
	PxArray<PxU32> mTriangleIndexBuffer;
	PxArray<PxParticleClothConstraint> mConstraintBuffer;

	PxU32 mVertexCount;
	physx::PxVec4* mVertices; //we don't own this
	PxU32 mTriangleIndexCount;
	PxU32* mTriangleIndices; //we don't own this
	PxU32 mConstraintTypeFlags;
	PxVec3 mVerticalDirection;
	float mBendingConstraintMaxAngle;

	float mMeshVolume;

	void addTriangle(PxArray<PxU32>& trianglesPerVertex, PxU32 triangleIndex)
	{
		for(int j = 0; j < 3; j++)
		{
			PxU32 vertexIndex = mTriangleIndices[triangleIndex + j];
			mTriangleIndexBuffer.pushBack(vertexIndex);
			trianglesPerVertex[vertexIndex]++;
		}
	}
};

void PxParticleClothCookerImpl::cookConstraints(const PxParticleClothConstraint* constraints, const PxU32 numConstraints)
{
	EdgeSet edgeSet;
	edgeSet.reserve(mTriangleIndexCount);

	PxArray<PxU32> trianglesPerVertex;
	trianglesPerVertex.resize(mVertexCount, 0);

	mTriangleIndexBuffer.clear();
	mTriangleIndexBuffer.reserve(mTriangleIndexCount);

	mConstraintBuffer.clear();
	mConstraintBuffer.reserve(mVertexCount*12);

	//Add all edges to Edges
	for(PxU32 i = 0; i<mTriangleIndexCount; i+=3)
	{
		//Get vertex indices
		PxU32 v0 = mTriangleIndices[i + 0];
		PxU32 v1 = mTriangleIndices[i + 1];
		PxU32 v2 = mTriangleIndices[i + 2];

		//Get vertex points
		PxVec3 p0 = mVertices[v0].getXYZ();
		PxVec3 p1 = mVertices[v1].getXYZ();
		PxVec3 p2 = mVertices[v2].getXYZ();

		//check which edge is the longest
		float len0 = (p0 - p1).magnitude();
		float len1 = (p1 - p2).magnitude();
		float len2 = (p2 - p0).magnitude();
		int longest = MaxArg(len0, PxMax(len1,len2), 0, MaxArg(len1, len2, 1, 2));

		//Store edges
		edgeSet.insert(Edge(v0, v1, i, 0xffffffff, longest == 0, false));
		edgeSet.insert(Edge(v1, v2, i, 0xffffffff, longest == 1, false));
		edgeSet.insert(Edge(v2, v0, i, 0xffffffff, longest == 2, false));

		//Add triangle to mTriangleIndexBuffer and increment trianglesPerVertex values
		addTriangle(trianglesPerVertex,i);
	}

	if (constraints)
	{
		//skip constraints cooking if provided by user
		mConstraintBuffer.assign(constraints, constraints + numConstraints);
		return;
	}

	trianglesPerVertex.clear();
	trianglesPerVertex.shrink();

	PxArray<Edge> uniqueEdges;
	uniqueEdges.reserve(mTriangleIndexCount); //over allocate to avoid resizes
	for (EdgeSet::Iterator iter = edgeSet.getIterator(); !iter.done(); ++iter)
	{
		const Edge& e = *iter;
		uniqueEdges.pushBack(e);
	}

	//Maximum angle before it is a horizontal constraint
	const float cosAngle45 = cosf(45.0f / 360.0f * PxTwoPi);

	//Add all horizontal, vertical and shearing constraints
	PxU32 constraintCount = uniqueEdges.size(); //we are going to push back more edges, but we don't need to process them
	for(PxU32 i = 0; i < constraintCount; i++)
	{
		const Edge& edge = uniqueEdges[i];
		PxParticleClothConstraint c;
		c.particleIndexA = edge.a;
		c.particleIndexB = edge.b;

		//Get vertices's
		PxVec3 vA = mVertices[c.particleIndexA].getXYZ();
		PxVec3 vB = mVertices[c.particleIndexB].getXYZ();

		//Calculate rest length
		c.length = (vA - vB).magnitude();

		if(edge.longestA && edge.longestB && (mConstraintTypeFlags & PxParticleClothConstraint::eTYPE_DIAGONAL_CONSTRAINT))
		{
			//Shearing constraint
			c.constraintType = c.eTYPE_DIAGONAL_CONSTRAINT;
			//add constraint
			mConstraintBuffer.pushBack(c);

			//We only have one of the quad diagonals in a triangle mesh, get the other one here
			const Edge alternateEdge = GetAlternateDiagonal(edge, mTriangleIndices);
			c.particleIndexA = alternateEdge.a;
			c.particleIndexB = alternateEdge.b;

			//Get vertices's
			PxVec3 vA2 = mVertices[c.particleIndexA].getXYZ();
			PxVec3 vB2 = mVertices[c.particleIndexB].getXYZ();

			//Calculate rest length
			c.length = (vA2 - vB2).magnitude();

			//add constraint
			mConstraintBuffer.pushBack(c);

			if (mConstraintTypeFlags & PxParticleClothConstraint::eTYPE_DIAGONAL_BENDING_CONSTRAINT)
			{
				if (!edgeSet.contains(alternateEdge))
				{
					edgeSet.insert(alternateEdge);
					uniqueEdges.pushBack(alternateEdge); //Add edge for bending constraint step
				}
			}
		}
		else
		{
			//horizontal/vertical constraint
			PxVec3 dir = (vA - vB) / c.length;
			if(mVerticalDirection.dot(dir)> cosAngle45)
				c.constraintType = c.eTYPE_VERTICAL_CONSTRAINT;
			else
				c.constraintType = c.eTYPE_HORIZONTAL_CONSTRAINT;

			if(mConstraintTypeFlags & c.constraintType)
			{
				//add constraint
				mConstraintBuffer.pushBack(c);
			}
		}
	}

	if(!(mConstraintTypeFlags & PxParticleClothConstraint::eTYPE_BENDING_CONSTRAINT))
		return;

	//Get adjacency information needed for the bending constraints
	PxArray<PxU32> adjacencyIndices;
	PxArray<PxU32> adjacencies;
	gatherAdjacencies(adjacencyIndices, adjacencies, mVertexCount, uniqueEdges, !(mConstraintTypeFlags & PxParticleClothConstraint::eTYPE_DIAGONAL_BENDING_CONSTRAINT));

	//Maximum angle we consider to be parallel for the bending constraints
	const float maxCosAngle = PxCos(mBendingConstraintMaxAngle);

	for(PxU32 i = 0; i<mVertexCount; i++)
	{
		//For each vertex, find all adjacent vertex pairs, and add bending constraints for pairs that form roughly a straight line
		PxVec3 center = mVertices[i].getXYZ();

		for(PxU32 adjA = adjacencyIndices[i]; PxI32(adjA) < PxI32(adjacencyIndices[i+1])-1; adjA++)
		{
			PxVec3 a = mVertices[adjacencies[adjA]].getXYZ();
			PxVec3 dir1 = (a-center).getNormalized();

			float bestCosAngle = -1.0f;
			PxU32 bestAdjB = 0xffffffff;

			//Choose the most parallel adjB
			for(PxU32 adjB = adjA+1; adjB < adjacencyIndices[i + 1]; adjB++)
			{
				PxVec3 b = mVertices[adjacencies[adjB]].getXYZ();
				PxVec3 dir2 = (b - center).getNormalized();
				float cosAngleAbs = PxAbs(dir1.dot(dir2));
				if(cosAngleAbs > bestCosAngle)
				{
					bestCosAngle = cosAngleAbs;
					bestAdjB = adjB;
				}
			}

			//Check if the lines a-center and center-b are roughly parallel
			if(bestCosAngle > maxCosAngle)
			{
				//Add bending constraint
				PxParticleClothConstraint c;
				c.particleIndexA = adjacencies[adjA];
				c.particleIndexB = adjacencies[bestAdjB];

				PX_ASSERT(c.particleIndexA != c.particleIndexB);

				//Get vertices's
				PxVec3 vA = mVertices[c.particleIndexA].getXYZ();
				PxVec3 vB = mVertices[c.particleIndexB].getXYZ();

				//Calculate rest length
				c.length = (vA - vB).magnitude();

				c.constraintType = c.eTYPE_BENDING_CONSTRAINT;
				//add constraint
				mConstraintBuffer.pushBack(c);
			}
		}
	}
}

void PxParticleClothCookerImpl::calculateMeshVolume()
{
	// the physx api takes volume*6 now.
	mMeshVolume = Gu::computeTriangleMeshVolume(mVertices, mTriangleIndices, mTriangleIndexCount / 3) * 6.0f;
}

} // namespace ExtGpu

ExtGpu::PxParticleClothCooker* PxCreateParticleClothCooker(PxU32 vertexCount, PxVec4* inVertices, PxU32 triangleIndexCount, PxU32* inTriangleIndices,
	PxU32 constraintTypeFlags, PxVec3 verticalDirection, PxReal bendingConstraintMaxAngle)
{
	return PX_NEW(ExtGpu::PxParticleClothCookerImpl)(vertexCount, inVertices, triangleIndexCount, inTriangleIndices,
		constraintTypeFlags, verticalDirection, bendingConstraintMaxAngle);
}

} // namespace physx
