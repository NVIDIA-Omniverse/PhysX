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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

#ifndef EXT_MESH_SIMPLIFICATOR_H
#define EXT_MESH_SIMPLIFICATOR_H


#include "foundation/PxBounds3.h"
#include "foundation/PxArray.h"
#include "geometry/PxSimpleTriangleMesh.h"

#include "ExtQuadric.h"
#include "ExtRandomAccessHeap.h"

// ------------------------------------------------------------------------------

// MM: implementation of paper Garland and Heckbert: "Surface Simplification Using Quadric Error Metrics"

namespace physx
{
	namespace Ext
	{

		class MeshSimplificator
		{
		public:

			MeshSimplificator();

			void init(const PxSimpleTriangleMesh& inputMesh, PxReal edgeLengthCostWeight_ = 1e-1f, PxReal flatnessDetectionThreshold_ = 1e-2f);
			void init(const PxArray<PxVec3> &vertices, const PxArray<PxU32> &triIds, PxReal edgeLengthCostWeight_ = 1e-1f, PxReal flatnessDetectionThreshold_ = 1e-2f);
			void decimateByRatio(PxF32 relativeOutputMeshSize = 0.5f, PxF32 maximalEdgeLength = 0.0f);
			void decimateBySize(PxI32 targetTriangleCount, PxF32 maximalEdgeLength = 0.0f);
			void readBack(PxArray<PxVec3>& vertices, PxArray<PxU32>& triIds, PxArray<PxU32> *vertexMap = NULL);

		private:
			PxArray<PxVec3> vertices;
			PxArray<PxI32> triIds;

			void init();
			bool step(PxF32 maximalEdgeLength);
			bool getAdjTris(PxI32 triNr, PxI32 vertNr, PxI32& valence, bool& open,
				PxArray<PxI32>* tris) const;
			bool getAdjTris(PxI32 triNr, PxI32 vertNr, PxArray<PxI32>& tris) const;

			void replaceNeighbor(PxI32 triNr, PxI32 oldNeighbor, PxI32 newNeighbor);
			PxI32 getEdgeId(PxI32 triNr, PxI32 edgeNr);
			bool collapseEdge(PxI32 triNr, PxI32 edgeNr);
			void evalEdgeCost(PxI32 triNr, PxI32 edgeNr, float& cost, float& ratio);
			void findTriNeighbors();

			void transformPointsToUnitBox(PxArray<PxVec3>& points);
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
