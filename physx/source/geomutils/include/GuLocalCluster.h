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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_LOCAL_CLUSTER_H
#define GU_LOCAL_CLUSTER_H

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxTransform.h"
#include "foundation/PxInlineArray.h"
#include "geometry/PxTriangle.h"

namespace physx
{
	class PxTriangleMeshGeometry;
	class PxMeshScale;

namespace Gu
{
	// PT: a local cluster is a triangle and its immediate neighbor.
	// For a regular triangle with at most 3 neighbors the structure will not allocate extra memory.
	struct LocalCluster
	{
		PxInlineArray<PxTriangle, 4>	mTris;		// Space for a source triangle and its 3 direct neighbors.
		PxInlineArray<PxVec3, 4>    	mNormals;	// Precomputed triangle normals.
		float							mExtent;	// Precomputed size of the bounds around all triangles.

		PX_PHYSX_COMMON_API	void reset();
		PX_PHYSX_COMMON_API	void init(const float* src, PxU32 nbFloats);
		PX_PHYSX_COMMON_API	void addTriangle(PxU32 triangleIndex, const PxMeshScale& scale, const PxTransform* meshPose, const PxVec3* verts, const void* triangles, PxU32 has16BitIndices);
	};

	class EdgeList;

	PX_PHYSX_COMMON_API	void createLocalCluster(LocalCluster& mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform* meshPose, PxU32 triIndex, const EdgeList* edgeList = NULL);
	PX_PHYSX_COMMON_API	float depenetrateLocalClusters(const LocalCluster& mesh0, const LocalCluster& mesh1, const PxVec3& inputDir, bool iterative);

} // namespace Gu
}

#endif
