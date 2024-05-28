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


#ifndef EXT_VOXEL_TETRAHEDRALIZER_H
#define EXT_VOXEL_TETRAHEDRALIZER_H

#include "ExtMultiList.h"
#include "ExtBVH.h"
#include "foundation/PxVec3.h"
#include "foundation/PxBounds3.h"

namespace physx
{
	namespace Ext
	{

		// ------------------------------------------------------------------------------

		class VoxelTetrahedralizer
		{
		public:
			VoxelTetrahedralizer();

			void clear();
			void createTetMesh(const PxArray<PxVec3>& verts, const PxArray<PxU32>& triIds,
				PxI32 resolution, PxI32 numRelaxationIters = 5, PxF32 relMinTetVolume = 0.05f);

			void readBack(PxArray<PxVec3>& tetVertices, PxArray<PxU32>& tetIndices);

		private:
			void voxelize(PxU32 resolution);
			void createTets(bool subdivBorder, PxU32 numTetsPerVoxel);
			void buildBVH();
			void createUniqueTetVertices();
			void findTargetPositions(PxF32 surfaceDist);
			void conserveVolume(PxF32 relMinVolume);
			void relax(PxI32 numIters, PxF32 relMinVolume);

			// input mesh

			PxArray<PxVec3> surfaceVerts;
			PxArray<PxI32> surfaceTriIds;
			PxBounds3 surfaceBounds;

			// voxel grid

			struct Voxel {
				void init(PxI32 _xi, PxI32 _yi, PxI32 _zi)
				{
					xi = _xi; yi = _yi; zi = _zi;
					for (PxI32 i = 0; i < 6; i++)
						neighbors[i] = -1;
					for (PxI32 i = 0; i < 8; i++)
						ids[i] = -1;
					parent = -1;
					inner = false;
				}
				bool isAt(PxI32 _xi, PxI32 _yi, PxI32 _zi) {
					return xi == _xi && yi == _yi && zi == _zi;
				}
				PxI32 xi, yi, zi;
				PxI32 neighbors[6];
				PxI32 parent;
				PxI32 ids[8];
				bool inner;
			};

			PxVec3 gridOrigin;
			PxF32 gridSpacing;
			PxArray<Voxel> voxels;

			BVHDesc bvh;

			// tet mesh

			PxArray<PxVec3> tetVerts;
			PxArray<PxVec3> origTetVerts;
			PxArray<PxI32> tetIds;

			// relaxation

			PxArray<bool> isSurfaceVert;
			PxArray<PxVec3> targetVertPos;
			PxArray<PxI32> queryTris;
			PxArray<PxI32> edgeIds;
		};
	}
}

#endif
