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

#ifndef EXT_MESH_SIMPLIFICATOR_H
#define EXT_MESH_SIMPLIFICATOR_H


#include "foundation/PxBounds3.h"
#include "foundation/PxArray.h"
#include "geometry/PxSimpleTriangleMesh.h"

#include "ExtQuadric.h"
#include "ExtRandomAccessHeap.h"
#include "GuSDF.h"

// ------------------------------------------------------------------------------

// MM: implementation of paper Garland and Heckbert: "Surface Simplification Using Quadric Error Metrics"

namespace physx
{
	namespace Ext
	{
		struct PxVec3Ex
		{
			PxVec3 p;
			PxU32 i = 0xFFFFFFFF;
			
			explicit PxVec3Ex() : p(0.0f), i(0xFFFFFFFF)
			{
			}

			explicit PxVec3Ex(PxVec3 point, PxU32 sourceTriangleIndex = 0xFFFFFFFF) : p(point), i(sourceTriangleIndex)
			{
			}
		};

		class MeshSimplificator
		{
		public:

			MeshSimplificator();

			void init(const PxSimpleTriangleMesh& inputMesh, PxReal edgeLengthCostWeight_ = 1e-1f, PxReal flatnessDetectionThreshold_ = 1e-2f, bool projectSimplifiedPointsOnInputMeshSurface = false);
			void init(const PxArray<PxVec3> &vertices, const PxArray<PxU32> &triIds, PxReal edgeLengthCostWeight_ = 1e-1f, PxReal flatnessDetectionThreshold_ = 1e-2f, bool projectSimplifiedPointsOnInputMeshSurface = false);
			void decimateByRatio(PxF32 relativeOutputMeshSize = 0.5f, PxF32 maximalEdgeLength = 0.0f);
			void decimateBySize(PxI32 targetTriangleCount, PxF32 maximalEdgeLength = 0.0f);
			void readBack(PxArray<PxVec3>& vertices, PxArray<PxU32>& triIds, PxArray<PxU32> *vertexMap = NULL, PxArray<PxU32> *outputVertexToInputTriangle = NULL);
			
			~MeshSimplificator();

		private:
			PxArray<PxVec3Ex> vertices;
			PxArray<PxI32> triIds;
			PxArray<PxVec3> scaledOriginalVertices;
			PxArray<PxU32> originalTriIds;
			Gu::PxPointOntoTriangleMeshProjector* projector;

			void init();
			bool step(PxF32 maximalEdgeLength);
			bool getAdjTris(PxI32 triNr, PxI32 vertNr, PxI32& valence, bool& open,
				PxArray<PxI32>* tris) const;
			bool getAdjTris(PxI32 triNr, PxI32 vertNr, PxArray<PxI32>& tris) const;

			void replaceNeighbor(PxI32 triNr, PxI32 oldNeighbor, PxI32 newNeighbor);
			PxI32 getEdgeId(PxI32 triNr, PxI32 edgeNr);
			bool collapseEdge(PxI32 triNr, PxI32 edgeNr);
			PxVec3Ex evalEdgeCost(PxI32 triNr, PxI32 edgeNr, PxReal& costt);
			PxVec3Ex projectPoint(const PxVec3& p);
			void findTriNeighbors();

			void transformPointsToUnitBox(PxArray<PxVec3Ex>& points);
			void transformPointsToOriginalPosition(PxArray<PxVec3>& points);

			PxI32 numMeshTris;

			PxArray<Quadric> quadrics;
			PxArray<PxI32> vertMarks;
			PxArray<PxI32> adjTris;
			PxI32 currentVertMark;
			PxArray<PxI32> triNeighbors;

			//Scale input points into 0...1 unit-box
			PxReal scaling;
			PxVec3 origin;

			PxReal edgeLengthCostWeight;
			PxReal flatnessDetectionThreshold;

			PxArray<PxI32> simplificationMap;

			struct HeapElem 
			{
				HeapElem() : triNr(0), edgeNr(0), cost(0.0f) {}
				HeapElem(PxI32 triNr_, PxI32 edgeNr_, float cost_) :
					triNr(triNr_), edgeNr(edgeNr_), cost(cost_) {}

				PxI32 triNr, edgeNr;
				float cost;

				PX_FORCE_INLINE bool operator < (const HeapElem& e) const 
				{
					return cost < e.cost;
				}
			};

			RandomAccessHeap<HeapElem> heap;
		};
	}
}

#endif
