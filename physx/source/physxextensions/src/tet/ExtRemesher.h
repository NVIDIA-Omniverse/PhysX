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


#ifndef EXT_REMESHER_H
#define EXT_REMESHER_H

#include "foundation/PxBounds3.h"
#include "foundation/PxArray.h"

namespace physx
{
	namespace Ext
	{

		// ------------------------------------------------------------------------------

		class Remesher {
		public:

			Remesher() {}
			~Remesher() {}

			void remesh(const PxArray<PxVec3>& verts, const PxArray<PxU32>& triIds, PxI32 resolution = 100, PxArray<PxU32> *vertexMap = nullptr);

			void clear();
			void readBack(PxArray<PxVec3>& vertices, PxArray<PxU32>& triIds);

		private:
			PxArray<PxVec3> vertices;
			PxArray<PxI32> triIds;

			void addCell(PxI32 xi, PxI32 yi, PxI32 zi);
			PxI32  getCellNr(PxI32 xi, PxI32 yi, PxI32 zi) const;
			bool cellExists(PxI32 xi, PxI32 yi, PxI32 zi) const;

			void removeDuplicateVertices();
			void pruneInternalSurfaces();
			void computeNormals();
			void findTriNeighbors();

			void project(const PxArray<PxVec3>& inputVerts, const PxArray<PxU32>& inputTriIds,
				float searchDist, float surfaceDist);

			void createVertexMap(const PxArray<PxVec3>& inputVerts, const PxVec3 &gridOrigin, PxF32 &gridSpacing,
				PxArray<PxU32> &vertexMap);

			// -------------------------------------------------------------------------------------
			struct Cell
			{
				void init(PxI32 _xi, PxI32 _yi, PxI32 _zi) {
					this->xi = _xi; this->yi = _yi; this->zi = _zi;
					this->next = -1;
				}
				PxI32 xi, yi, zi;
				PxI32 next;
			};

			PxArray<Cell> cells;
			PxArray<PxI32> firstCell;
			PxArray<PxVec3> normals;
			PxArray<PxI32> triNeighbors;
			PxArray<PxI32> cellOfVertex;

			PxArray<PxBounds3> bvhBounds;
			PxArray<PxI32> bvhTris;
		};
	}
}


#endif
