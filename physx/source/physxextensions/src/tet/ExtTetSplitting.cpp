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

#include "ExtTetSplitting.h"
#include "ExtUtilities.h"
#include "foundation/PxBasicTemplates.h"

using namespace physx;
using namespace Ext;

//Last four bits are used
//Last bit set stands for A
//Second last bit stands for B
//Third last bit stands for C
//Fourth last bit stands for D

typedef PxU32 TetCorner;
typedef PxU32 TetEdge;

static const TetCorner None = 0x00000000;
static const TetCorner A = 0x00000001;
static const TetCorner B = 0x00000002;
static const TetCorner C = 0x00000004;
static const TetCorner D = 0x00000008;
	 
static const TetEdge AB = 0x00000003;
static const TetEdge AC = 0x00000005;
static const TetEdge AD = 0x00000009;
static const TetEdge BC = 0x00000006;
static const TetEdge BD = 0x0000000A;
static const TetEdge CD = 0x0000000C;
	 
static const TetCorner tetCorners[4] = { A, B, C, D };

namespace
{
	//Specifies which edges of a tetrahedron should get a point inserted (=split)
	struct TetSubdivisionInfo
	{
		//If an index has value -1 then this means that this edge won't get split
		PxI32 ab;
		PxI32 ac;
		PxI32 ad;

		PxI32 bc;
		PxI32 bd;

		PxI32 cd;

		Tetrahedron tet;
		PxI32 id;

		PX_FORCE_INLINE TetSubdivisionInfo() : ab(-1), ac(-1), ad(-1), bc(-1), bd(-1), cd(-1), tet(Tetrahedron(-1, -1, -1, -1)), id(-1) {}

		PX_FORCE_INLINE TetSubdivisionInfo(Tetrahedron tet_, PxI32 aBPointInsertIndex, PxI32 aCPointInsertIndex, PxI32 aDPointInsertIndex, 
			PxI32 bCPointInsertIndex, PxI32 bDPointInsertIndex, PxI32 cDPointInsertIndex, PxI32 id_ = -1) : 
			ab(aBPointInsertIndex), ac(aCPointInsertIndex), ad(aDPointInsertIndex), bc(bCPointInsertIndex), bd(bDPointInsertIndex), cd(cDPointInsertIndex), tet(tet_), id(id_)
		{ }

		PX_FORCE_INLINE void swapInternal(TetCorner corner1, TetCorner corner2, TetCorner& cornerToProcess)
		{
			if (cornerToProcess == corner1)
				cornerToProcess = corner2;
			else if (cornerToProcess == corner2)
				cornerToProcess = corner1;
		}

		// PT: this function seems unused
		//Helper method for sorting
		/*template<typename T>
		void swap(T& a, T& b)
		{
			T tmp = a;
			a = b;
			b = tmp;
		}*/

		//Helper method for sorting
		bool swap(TetCorner corner1, TetCorner corner2, TetCorner& additionalCornerToProcess1, TetCorner& additionalCornerToProcess2)
		{
			swapInternal(corner1, corner2, additionalCornerToProcess1);
			swapInternal(corner1, corner2, additionalCornerToProcess2);
			return swap(corner1, corner2);
		}

		//Helper method for sorting
		bool swap(TetCorner corner1, TetCorner corner2, TetCorner& additionalCornerToProcess)
		{
			swapInternal(corner1, corner2, additionalCornerToProcess);
			return swap(corner1, corner2);
		}

		//Helper method for sorting
		bool swap(TetCorner corner1, TetCorner corner2)
		{
			if (corner1 == corner2)
				return false;

			if (corner1 > corner2)
			{
				TetCorner tmp = corner1;
				corner1 = corner2;
				corner2 = tmp;
			}

			if (corner1 == A && corner2 == B)
			{
				PxSwap(ad, bd);
				PxSwap(ac, bc);
				PxSwap(tet[0], tet[1]);
			}
			else if (corner1 == A && corner2 == C)
			{
				PxSwap(ad, cd);
				PxSwap(ab, bc);
				PxSwap(tet[0], tet[2]);
			}
			else if (corner1 == A && corner2 == D)
			{
				PxSwap(ac, cd);
				PxSwap(ab, bd);
				PxSwap(tet[0], tet[3]);
			}
			else if (corner1 == B && corner2 == C)
			{
				PxSwap(ac, ab);
				PxSwap(cd, bd);
				PxSwap(tet[1], tet[2]);
			}
			else if (corner1 == B && corner2 == D)
			{
				PxSwap(ab, ad);
				PxSwap(bc, cd);
				PxSwap(tet[1], tet[3]);
			}
			else if (corner1 == C && corner2 == D)
			{
				PxSwap(ac, ad);
				PxSwap(bc, bd);
				PxSwap(tet[2], tet[3]);
			}
			return true;
		}

		//Allows to sort the information such that edges of neighboring tets have the same orientation
		PxI32 sort()
		{
			PxI32 counter = 0;
			if (tet[0] > tet[1]) { swap(A, B); ++counter; }
			if (tet[0] > tet[2]) { swap(A, C); ++counter; }
			if (tet[0] > tet[3]) { swap(A, D); ++counter; }

			if (tet[1] > tet[2]) { swap(B, C); ++counter; }
			if (tet[1] > tet[3]) { swap(B, D); ++counter; }
			if (tet[2] > tet[3]) { swap(C, D); ++counter; }
			return counter;
		}
	};
}

//Returns true if the edge is adjacent to the specified corner
static PX_FORCE_INLINE bool edgeContainsCorner(TetEdge edge, TetCorner corner)
{
	return (edge & corner) != 0;
}

//Returns the common point of two edges, will be None if there is no common point
static PX_FORCE_INLINE TetCorner getCommonPoint(TetEdge edge1, TetEdge edge2)
{
	return edge1 & edge2;
}

//Extracts the global indices from a tet given a local edge
static Edge getTetEdge(const Tetrahedron& tet, TetEdge edge)
{
	switch (edge)
	{
	case AB:
		return Edge(tet[0], tet[1]);
	case AC:
		return Edge(tet[0], tet[2]);
	case AD:
		return Edge(tet[0], tet[3]);
	case BC:
		return Edge(tet[1], tet[2]);
	case BD:
		return Edge(tet[1], tet[3]);
	case CD:
		return Edge(tet[2], tet[3]);
	}
	return Edge(-1, -1);
}
	
static TetCorner getStart(TetEdge e)
{
	switch (e)
	{
	case AB:
		return A;
	case AC:
		return A;
	case AD:
		return A;
	case BC:
		return B;
	case BD:
		return B;
	case CD:
		return C;
	}
	return None;
}

static TetCorner getEnd(TetEdge e)
{
	switch (e)
	{
	case AB:
		return B;
	case AC:
		return C;
	case AD:
		return D;
	case BC:
		return C;
	case BD:
		return D;
	case CD:
		return D;
	}
	return None;
}

static PX_FORCE_INLINE TetEdge getOppositeEdge(TetEdge edge)
{
	return 0x0000000F ^ edge;
}

//Finds the index of the first instance of value in list
static PxI32 getIndexOfFirstValue(PxI32 list[4], PxI32 value = 0, PxI32 startAt = 0)
{
	for (PxI32 i = startAt; i < 4; ++i)
		if (list[i] == value)
			return i;

	PX_ASSERT(false); // we should never reach this line
	return 0;
}

//Counts how many times every corner is referenced by the specified set of edges - useful for corner classification
static void getCornerAccessCounter(TetEdge edges[6], PxI32 edgesLength, PxI32 cornerAccessCounter[4])
{
	for (PxI32 i = 0; i < 4; ++i)
		cornerAccessCounter[i] = 0;

	for (PxI32 j = 0; j < edgesLength; ++j)
	{
		switch (edges[j])
		{
		case AB:
			++cornerAccessCounter[0];
			++cornerAccessCounter[1];
			break;
		case AC:
			++cornerAccessCounter[0];
			++cornerAccessCounter[2];
			break;
		case AD:
			++cornerAccessCounter[0];
			++cornerAccessCounter[3];
			break;
		case BC:
			++cornerAccessCounter[1];
			++cornerAccessCounter[2];
			break;
		case BD:
			++cornerAccessCounter[1];
			++cornerAccessCounter[3];
			break;
		case CD:
			++cornerAccessCounter[2];
			++cornerAccessCounter[3];
			break;
		}
	}
}

// PT: using static exposed the fact that this function was not used. Expected?
//Returns the tet's edge that does not contain corner1 and neither corner2
/*static Edge getRemainingEdge(const Tetrahedron& tet, PxI32 corner1, PxI32 corner2)
{
	PxI32 indexer = 0;
	Edge result(-1, -1);
	for (PxU32 i = 0; i < 4; ++i) 
	{
		if (tet[i] != corner1 && tet[i] != corner2)
		{
			if (indexer == 0)
				result.first = tet[i];
			else if (indexer == 1)
				result.second = tet[i];
			++indexer;
		}			
	}
	return result;
}*/

static PX_FORCE_INLINE TetCorner getOtherCorner(TetEdge edge, TetCorner corner)
{
	return edge ^ corner;
}	

static PX_FORCE_INLINE Tetrahedron flip(bool doFlip, Tetrahedron t)
{
	if (doFlip) PxSwap(t[2], t[3]);
	return t;
}

//Splits all tets according to the specification in tetSubdivisionInfos. The resulting mesh will be watertight if the tetSubdivisionInfos are specified such
//that all tets sharing and edge will get the same point inserted on their corresponding edge
static void split(PxArray<Tetrahedron>& tets, const PxArray<PxVec3d>& points, const PxArray<TetSubdivisionInfo>& tetSubdivisionInfos)
{
	PxU32 originalNumTets = tets.size();
	for (PxU32 i = 0; i < originalNumTets; ++i)
	{
		TetSubdivisionInfo info = tetSubdivisionInfos[i];
		PxI32 counter = info.sort();

		TetEdge splitEdges[6];
		PxI32 splitEdgesLength = 0;
		TetEdge nonSplitEdges[6];
		PxI32 nonSplitEdgesLength = 0;
		PxI32 insertionIndices[6];
		PxI32 insertionIndicesLength = 0;
		if (info.ab >= 0) { splitEdges[splitEdgesLength++] = AB; insertionIndices[insertionIndicesLength++] = info.ab; }
		else nonSplitEdges[nonSplitEdgesLength++] = AB;
		if (info.ac >= 0) { splitEdges[splitEdgesLength++] = AC; insertionIndices[insertionIndicesLength++] = info.ac; }
		else nonSplitEdges[nonSplitEdgesLength++] = AC;
		if (info.ad >= 0) { splitEdges[splitEdgesLength++] = AD; insertionIndices[insertionIndicesLength++] = info.ad; }
		else nonSplitEdges[nonSplitEdgesLength++] = AD;

		if (info.bc >= 0) { splitEdges[splitEdgesLength++] = BC; insertionIndices[insertionIndicesLength++] = info.bc; }
		else nonSplitEdges[nonSplitEdgesLength++] = BC;
		if (info.bd >= 0) { splitEdges[splitEdgesLength++] = BD; insertionIndices[insertionIndicesLength++] = info.bd; }
		else nonSplitEdges[nonSplitEdgesLength++] = BD;

		if (info.cd >= 0) { splitEdges[splitEdgesLength++] = CD; insertionIndices[insertionIndicesLength++] = info.cd; }
		else nonSplitEdges[nonSplitEdgesLength++] = CD;

		//Depending on how many tet edges get a point inserted, a different topology results. 
		//The created topology will make sure all neighboring tet faces will be tessellated identically to keep the mesh watertight
		switch (splitEdgesLength)
		{
		case 0:
			//Nothing to do here
			break;
		case 1:
		{
			PxI32 pointIndex = insertionIndices[0];
			Edge splitEdge = getTetEdge(info.tet, splitEdges[0]);

			Edge oppositeEdge = getTetEdge(info.tet, getOppositeEdge(splitEdges[0]));

			tets[i] = Tetrahedron(oppositeEdge.first, oppositeEdge.second, splitEdge.first, pointIndex);
			tets.pushBack(Tetrahedron(oppositeEdge.first, oppositeEdge.second, pointIndex, splitEdge.second));

			break;
		}
		case 2:
		{
			TetCorner corner = getCommonPoint(splitEdges[0], splitEdges[1]);
			if (corner != None)
			{
				//edges have a common point                  
				//Rearrange such that common corner is a and first edge is from a to b while second edge is from a to c
				TetCorner p1 = getOtherCorner(splitEdges[0], corner);
				TetCorner p2 = getOtherCorner(splitEdges[1], corner);
				if (info.swap(corner, A, p1, p2)) ++counter;
				if (info.swap(p1, B, p2)) ++counter;
				if (info.swap(p2, C)) ++counter;

				if (info.tet[1] > info.tet[2]) 
				{
					if (info.swap(B, C)) ++counter;
				}

				const bool f = counter % 2 == 1;

				tets[i] = flip(f, Tetrahedron(info.tet[0], info.tet[3], info.ab, info.ac));
				tets.pushBack(flip(f, Tetrahedron(info.tet[3], info.tet[1], info.ab, info.ac)));
				tets.pushBack(flip(f, Tetrahedron(info.tet[3], info.tet[2], info.tet[1], info.ac)));
			}
			else
			{
				//Edges don't have a common point (opposite edges)
				TetEdge edge1 = splitEdges[0];
				//TetEdge edge2 = splitEdges[1];

				//Permute the tetrahedron such that edge1 becomes the edge AB
				if (info.swap(getStart(edge1), A)) ++counter;
				if (info.swap(getEnd(edge1), B)) ++counter;

				if (info.tet[0] > info.tet[1]) 
				{
					if (info.swap(A, B)) ++counter;
				}
				if (info.tet[2] > info.tet[3]) 
				{
					if (info.swap(C, D)) ++counter;
				}

				const bool f = counter % 2 == 1;

				tets[i] = flip(f, Tetrahedron(info.tet[0], info.ab, info.tet[2], info.cd));
				tets.pushBack(flip(f, Tetrahedron(info.tet[1], info.ab, info.cd, info.tet[2])));

				tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ab, info.cd, info.tet[3])));
				tets.pushBack(flip(f, Tetrahedron(info.tet[1], info.ab, info.tet[3], info.cd)));
			}
			break;
		}
		case 3:
		{
			//There are three sub cases called a, b and c				
			TetCorner commonPoint01 = getCommonPoint(splitEdges[0], splitEdges[1]);
			TetCorner commonPoint02 = getCommonPoint(splitEdges[0], splitEdges[2]);
			TetCorner commonPoint12 = getCommonPoint(splitEdges[1], splitEdges[2]);
			if (commonPoint01 == None || commonPoint02 == None || commonPoint12 == None)
			{
				//The three edges form a non closed strip
				//The strip's end points are connected by a tet edge - map this edge such that it becomes edge AB
				//Then sort AB

				PxI32 cnt[4];
				getCornerAccessCounter(splitEdges, splitEdgesLength, cnt);

				PxI32 index = getIndexOfFirstValue(cnt, 1);
				TetCorner refStart = tetCorners[index];
				TetCorner refEnd = tetCorners[getIndexOfFirstValue(cnt, 1, index + 1)];


				TetCorner cornerToMapOntoC = None;
				if (edgeContainsCorner(splitEdges[0], refEnd))
				{
					cornerToMapOntoC = getOtherCorner(splitEdges[0], refEnd);
				}
				else if (edgeContainsCorner(splitEdges[1], refEnd))
				{
					cornerToMapOntoC = getOtherCorner(splitEdges[1], refEnd);
				}
				else if (edgeContainsCorner(splitEdges[2], refEnd))
				{
					cornerToMapOntoC = getOtherCorner(splitEdges[2], refEnd);
				}

				if (info.swap(refStart, A, refEnd, cornerToMapOntoC)) ++counter;
				if (info.swap(refEnd, B, cornerToMapOntoC)) ++counter;
				if (info.swap(cornerToMapOntoC, C)) ++counter;

				if (info.tet[0] > info.tet[1])
				{
					if (info.swap(A, B)) ++counter;
					if (info.swap(C, D)) ++counter;
				}

				const bool f = counter % 2 == 1;

				tets[i] = flip(f, Tetrahedron(info.tet[0], info.tet[1], info.bc, info.ad));

				if (info.tet[0] > info.tet[2])
				{
					tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.cd, info.ad, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.tet[2], info.ad, info.bc)));
				}
				else 
				{
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.tet[2], info.cd, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ad, info.bc, info.cd)));
				}

				if (info.tet[1] > info.tet[3])
				{
					tets.pushBack(flip(f, Tetrahedron(info.tet[3], info.bc, info.ad, info.cd)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[3], info.tet[1], info.ad, info.bc)));
				}
				else 
				{
					tets.pushBack(flip(f, Tetrahedron(info.tet[1], info.tet[3], info.cd, info.ad)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[1], info.bc, info.ad, info.cd)));
				}
			}
			else if (edgeContainsCorner(splitEdges[2], commonPoint01))
			{
				//All three edges share one common point
				//Permute tetrahedron such that the common tip point is a
				if (info.swap(commonPoint01, A)) ++counter;
				//Sort the remaining values
				if (info.tet[1] > info.tet[2]) 
				{
					if (info.swap(B, C)) ++counter;
				}
				if (info.tet[2] > info.tet[3]) 
				{
					if (info.swap(C, D)) ++counter;
				}
				if (info.tet[1] > info.tet[2])
				{
					if (info.swap(B, C)) ++counter;
				}

				const bool f = counter % 2 == 1;

				tets[i] = flip(f, Tetrahedron(info.tet[0], info.ab, info.ac, info.ad));
				tets.pushBack(flip(f, Tetrahedron(info.tet[1], info.ab, info.ad, info.ac)));
				tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.tet[1], info.ad, info.ac)));
				tets.pushBack(flip(f, Tetrahedron(info.tet[1], info.tet[2], info.ad, info.tet[3])));
			}
			else
			{
				//Edges form a triangle
				//Rearrange such that point opposite of triangle loop is point d
				//Triangle loop is a, b and c, make sure they're sorted

				if (!(commonPoint01 == A || commonPoint02 == A || commonPoint12 == A)) 
				{
					if (info.swap(A, D)) ++counter;
				}
				else if (!(commonPoint01 == B || commonPoint02 == B || commonPoint12 == B)) 
				{
					if (info.swap(B, D)) ++counter;
				}
				else if (!(commonPoint01 == C || commonPoint02 == C || commonPoint12 == C)) 
				{
					if (info.swap(C, D)) ++counter;
				}
				else if (!(commonPoint01 == D || commonPoint02 == D || commonPoint12 == D)) 
				{
					if (info.swap(D, D)) ++counter;
				}

				//Sort a,b and c
				if (info.tet[0] > info.tet[1])
					if (info.swap(A, B)) ++counter;
				if (info.tet[1] > info.tet[2])
					if (info.swap(B, C)) ++counter;
				if (info.tet[0] > info.tet[1])
					if (info.swap(A, B)) ++counter;

				const bool f = counter % 2 == 1;

				tets[i] = flip(f, Tetrahedron(info.tet[3], info.ab, info.ac, info.bc));
				tets.pushBack(flip(f, Tetrahedron(info.tet[3], info.ab, info.ac, info.tet[0])));
				tets.pushBack(flip(f, Tetrahedron(info.tet[3], info.ab, info.bc, info.tet[1])));
				tets.pushBack(flip(f, Tetrahedron(info.tet[3], info.ac, info.bc, info.tet[2])));
			}
			break;
		}
		case 4:
		{
			TetCorner commonPoint = getCommonPoint(nonSplitEdges[0], nonSplitEdges[1]);

			if (commonPoint != None)
			{
				//Three edges form a triangle and the two edges that are not split share a common point
				TetCorner p1 = getOtherCorner(nonSplitEdges[0], commonPoint);
				TetCorner p2 = getOtherCorner(nonSplitEdges[1], commonPoint);

				if (info.swap(commonPoint, A, p1, p2)) ++counter;

				if (info.swap(p1, B, p2)) ++counter;
				if (info.swap(p2, C)) ++counter;

				if (info.tet[1] > info.tet[2])
					if (info.swap(B, C)) ++counter;

				const bool f = counter % 2 == 0;

				//Tip
				tets[i] = flip(f, Tetrahedron(info.tet[3], info.ad, info.bd, info.cd));

				//Center
				tets.pushBack(flip(f, Tetrahedron(info.bd, info.ad, info.bc, info.cd)));

				if (info.tet[0] < info.tet[1] && info.tet[0] < info.tet[2])
				{
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.tet[1], info.bd, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.tet[2], info.bc, info.cd)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.bc, info.bd, info.ad)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.bc, info.ad, info.cd)));
				}
				else if (info.tet[0] > info.tet[1] && info.tet[0] < info.tet[2])
				{
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.tet[2], info.bc, info.cd)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.bc, info.ad, info.cd)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.tet[1], info.ad, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[1], info.bc, info.bd, info.ad)));
				}
				else 
				{ 
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.tet[1], info.ad, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[1], info.bc, info.bd, info.ad)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.tet[0], info.ad, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.bc, info.ad, info.cd)));
				}
			}
			else
			{
				//All four edges form a loop
				TetEdge edge1 = nonSplitEdges[0];

				//Permute the tetrahedron such that edge1 becomes the edge AB
				TetCorner end = getEnd(edge1);
				if (info.swap(getStart(edge1), A, end)) ++counter;
				if (info.swap(end, B)) ++counter;

				//Sort
				if (info.tet[0] > info.tet[1])
					if (info.swap(A, B)) ++counter;
				if (info.tet[2] > info.tet[3])
					if (info.swap(C, D)) ++counter;

				const bool f = counter % 2 == 1;

				tets[i] = flip(f, Tetrahedron(info.tet[0], info.tet[1], info.bc, info.bd));
				tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.tet[3], info.ad, info.bd)));

				PxF64 dist1 = (points[info.ad] - points[info.bc]).magnitudeSquared();
				PxF64 dist2 = (points[info.ac] - points[info.bd]).magnitudeSquared();

				if (dist1 < dist2)
				{
					//Diagonal from AD to BC
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ad, info.bc, info.ac)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ad, info.bd, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.ad, info.ac, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.ad, info.bc, info.bd)));
				}
				else
				{
					//Diagonal from AC to BD
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ad, info.bd, info.ac)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ac, info.bd, info.bc)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.bc, info.bd, info.ac)));
					tets.pushBack(flip(f, Tetrahedron(info.tet[2], info.bd, info.ad, info.ac)));
				}
			}
			break;
		}
		case 5:
		{
			//There is only one edge that does not get split

			//First create 2 small tetrahedra in every corner that is not an end point of the unsplit edge
			TetEdge nonSplitEdge;
			if (info.ab < 0)
				nonSplitEdge = AB;
			else if (info.ac < 0)
				nonSplitEdge = AC;
			else if (info.ad < 0)
				nonSplitEdge = AD;
			else if (info.bc < 0)
				nonSplitEdge = BC;
			else if (info.bd < 0)
				nonSplitEdge = BD;
			else //if (info.CDPointInsertIndex < 0)
				nonSplitEdge = CD;

			TetCorner end = getEnd(nonSplitEdge);
			if (info.swap(getStart(nonSplitEdge), A, end)) ++counter;
			if (info.swap(end, B)) ++counter;

			if (info.tet[0] > info.tet[1])
				if (info.swap(A, B)) ++counter;
			if (info.tet[2] > info.tet[3])
				if (info.swap(C, D)) ++counter;

			const bool f = counter % 2 == 1;

			//Two corner tets at corner C and corner D
			tets[i] = flip(f, Tetrahedron(info.tet[2], info.ac, info.bc, info.cd));
			tets.pushBack(flip(f, Tetrahedron(info.tet[3], info.ad, info.cd, info.bd)));

			tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.tet[1], info.bc, info.bd)));


			//There are two possible diagonals -> take the shorter 
			PxF64 dist1 = (points[info.ac] - points[info.bd]).magnitudeSquared();
			PxF64 dist2 = (points[info.ad] - points[info.bc]).magnitudeSquared();
			if (dist1 < dist2)
			{
				//Diagonal from AC to BD
				tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ad, info.bd, info.ac)));
				tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ac, info.bd, info.bc)));
				//Tip pyramid
				tets.pushBack(flip(f, Tetrahedron(info.cd, info.bc, info.bd, info.ac)));
				tets.pushBack(flip(f, Tetrahedron(info.cd, info.bd, info.ad, info.ac)));
			}
			else
			{
				//Diagonal from AD to BC
				tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.ac, info.ad, info.bc)));
				tets.pushBack(flip(f, Tetrahedron(info.tet[0], info.bd, info.bc, info.ad)));
				//Tip pyramid
				tets.pushBack(flip(f, Tetrahedron(info.cd, info.bc, info.ad, info.ac)));
				tets.pushBack(flip(f, Tetrahedron(info.cd, info.bd, info.ad, info.bc)));
			}
			break;
		}
		case 6:
		{
			//First create a small tetrahedron in every corner
			const bool f = counter % 2 == 1;
			if (f)
				info.swap(A, B);
						
			tets[i] = Tetrahedron(info.tet[0], info.ab, info.ac, info.ad);
			tets.pushBack(Tetrahedron(info.tet[1], info.ab, info.bd, info.bc));
			tets.pushBack(Tetrahedron(info.tet[2], info.ac, info.bc, info.cd));
			tets.pushBack(Tetrahedron(info.tet[3], info.ad, info.cd, info.bd));

			//Now fill the remaining octahedron in the middle
			//An octahedron can be constructed using 4 tetrahedra
			//There are three diagonal candidates -> pick the shortest diagonal
			PxF64 dist1 = (points[info.ab] - points[info.cd]).magnitudeSquared();
			PxF64 dist2 = (points[info.ac] - points[info.bd]).magnitudeSquared();
			PxF64 dist3 = (points[info.ad] - points[info.bc]).magnitudeSquared();

			if (dist1 <= dist2 && dist1 <= dist3)
			{
				tets.pushBack(Tetrahedron(info.ab, info.cd, info.ad, info.bd));
				tets.pushBack(Tetrahedron(info.ab, info.cd, info.bd, info.bc));
				tets.pushBack(Tetrahedron(info.ab, info.cd, info.bc, info.ac));
				tets.pushBack(Tetrahedron(info.ab, info.cd, info.ac, info.ad));
			}
			else if (dist2 <= dist1 && dist2 <= dist3)
			{
				tets.pushBack(Tetrahedron(info.ac, info.bd, info.cd, info.ad));
				tets.pushBack(Tetrahedron(info.ac, info.bd, info.ad, info.ab));
				tets.pushBack(Tetrahedron(info.ac, info.bd, info.ab, info.bc));
				tets.pushBack(Tetrahedron(info.ac, info.bd, info.bc, info.cd));
			}
			else
			{
				tets.pushBack(Tetrahedron(info.ad, info.bc, info.bd, info.ab));
				tets.pushBack(Tetrahedron(info.ad, info.bc, info.cd, info.bd));
				tets.pushBack(Tetrahedron(info.ad, info.bc, info.ac, info.cd));
				tets.pushBack(Tetrahedron(info.ad, info.bc, info.ab, info.ac));
			}
			break;
		}
		}
	}
}

void physx::Ext::split(PxArray<Tetrahedron>& tets, const PxArray<PxVec3d>& points, const PxHashMap<PxU64, PxI32>& edgesToSplit)
{
	PxArray<TetSubdivisionInfo> subdivisionInfos;
	subdivisionInfos.resize(tets.size());
	for (PxU32 i = 0; i < tets.size(); ++i)
	{
		const Tetrahedron& tet = tets[i];
		TetSubdivisionInfo info(tet, -1, -1, -1, -1, -1, -1, i);

		if (const PxPair<const PxU64, PxI32>* ptr = edgesToSplit.find(key(tet[0], tet[1])))
			info.ab = ptr->second;
		if (const PxPair<const PxU64, PxI32>* ptr = edgesToSplit.find(key(tet[0], tet[2])))
			info.ac = ptr->second;
		if (const PxPair<const PxU64, PxI32>* ptr = edgesToSplit.find(key(tet[0], tet[3])))
			info.ad = ptr->second;

		if (const PxPair<const PxU64, PxI32>* ptr = edgesToSplit.find(key(tet[1], tet[2])))
			info.bc = ptr->second;
		if (const PxPair<const PxU64, PxI32>* ptr = edgesToSplit.find(key(tet[1], tet[3])))
			info.bd = ptr->second;

		if (const PxPair<const PxU64, PxI32>* ptr = edgesToSplit.find(key(tet[2], tet[3])))
			info.cd = ptr->second;

		subdivisionInfos[i] = info;
	}

	::split(tets, points, subdivisionInfos);
}
