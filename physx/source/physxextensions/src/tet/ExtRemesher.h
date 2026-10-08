// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


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

			void remesh(const PxVec3* verts, PxU32 nbVertices, const PxU32* triIds, PxU32 nbTriangleIndices, PxU32 resolution = 100, PxArray<PxU32> *vertexMap = nullptr);
			void remesh(const PxArray<PxVec3>& verts, const PxArray<PxU32>& triIds, PxU32 resolution = 100, PxArray<PxU32> *vertexMap = nullptr);

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

			void project(const PxVec3* inputVerts, const PxU32* inputTriIds, PxU32 nbTriangleIndices,
				float searchDist, float surfaceDist);

			void createVertexMap(const PxVec3* verts, PxU32 nbVertices, const PxVec3 &gridOrigin, PxF32 &gridSpacing,
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
