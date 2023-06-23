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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef EXT_OCTREE_TETRAHEDRALIZER_H
#define EXT_OCTREE_TETRAHEDRALIZER_H

#include "ExtMultiList.h"
#include "ExtVec3.h"
#include "foundation/PxVec3.h"
#include "ExtInsideTester.h"

namespace physx
{
	namespace Ext
	{

		class InsideTester;

		// ------------------------------------------------------------------------------

		class OctreeTetrahedralizer 
		{
		public:
			OctreeTetrahedralizer();

			void clear();
			void createTetMesh(const PxArray<PxVec3> &verts, const PxArray<PxU32> &triIds,
				bool includeOctreeNodes = true, PxI32 maxVertsPerCell = 20, PxI32 maxTreeDepth = 5);

			void readBack(PxArray<PxVec3> &tetVertices, PxArray<PxU32> &tetIndices);			

		private:
			// input mesh

			PxArray<PxVec3> surfaceVerts;
			PxArray<PxI32> surfaceTriIds;

			// octree

			PxI32 maxVertsPerCell;
			PxI32 maxTreeDepth;

			struct Cell
			{
				void init()
				{
					firstChild = -1;
					orig = PxVec3d(0.0, 0.0, 0.0);
					size = 0.0;
					numVerts = 0;
					closestTetNr = -1;
					depth = 0;
				}

				PxI32 getChildNr(const PxVec3d& p);

				PX_FORCE_INLINE PxI32 getChildNr(const PxVec3& p) 
				{
					return getChildNr(PxVec3d(PxF64(p.x), PxF64(p.y), PxF64(p.z)));
				}

				PxI32 firstChild;
				PxI32 firstCellVert;
				PxI32 firstCellTet;
				PxVec3d orig;
				double size;
				PxI32 numVerts;
				PxI32 closestTetNr;
				PxI32 depth;
			};

			PxArray<Cell> cells;
			MultiList<PxI32> vertsOfCell;

			// tet mesh

			PxArray<PxVec3d> tetVerts;
			PxArray<PxI32> tetIds;
			PxArray<PxI32> tetNeighbors;
			PxArray<PxI32> tetMarks;
			PxI32 currentTetMark;
			PxArray<PxI32> stack;
			PxArray<PxI32> violatingTets;
			PxI32 firstABBVert;

			struct Edge 
			{

				PxI32 id0, id1;
				PxI32 faceNr, tetNr;

				void init(PxI32 _id0, PxI32 _id1, PxI32 _tetNr, PxI32 _faceNr) 
				{
					this->id0 = _id0 < _id1 ? _id0 : _id1;
					this->id1 = _id0 > _id1 ? _id0 : _id1;
					this->tetNr = _tetNr;
					this->faceNr = _faceNr;
				}

				PX_FORCE_INLINE bool operator < (Edge e) const
				{
					if (id0 < e.id0) return true;
					if (id0 > e.id0) return false;
					return id1 < e.id1;
				}

				PX_FORCE_INLINE bool operator == (Edge e)
				{
					return id0 == e.id0 && id1 == e.id1;
				}
			};
			PxArray<Edge> edges;

			void clearTets();
			void createTree();
			void treeInsertVert(PxI32 cellNr, PxI32 vertNr);
			void createTetVerts(bool includeOctreeNodes);

			bool findSurroundingTet(const PxVec3d& p, PxI32 startTetNr, PxI32& tetNr);
			bool findSurroundingTet(const PxVec3d& p, PxI32& tetNr);
			void treeInsertTet(PxI32 tetNr);
			void treeRemoveTet(PxI32 tetNr);

			PxI32 firstFreeTet;
			PxI32 getNewTetNr();
			void removeTetNr(PxI32 tetNr);

			PxVec3d getTetCenter(PxI32 tetNr) const;
			bool meshInsertTetVert(PxI32 vertNr);

			InsideTester insideTester;
			void pruneTets();

			mutable float prevClip;
			mutable float prevScale;
			mutable PxArray<PxVec3> renderVerts;
			mutable PxArray<PxVec3> renderNormals;
			mutable PxArray<PxI32> renderTriIds;
		};
	}
}

#endif
