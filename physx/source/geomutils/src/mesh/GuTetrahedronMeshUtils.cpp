// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuTetrahedronMeshUtils.h"
#include "GuDistancePointTetrahedron.h"

namespace physx
{
namespace Gu
{

void convertDeformableVolumeCollisionToSimMeshTets(const PxTetrahedronMesh& simMesh, const DeformableVolumeAuxData& simState, const BVTetrahedronMesh& collisionMesh,
												   PxU32 inTetId, const PxVec4& inTetBarycentric, PxU32& outTetId, PxVec4& outTetBarycentric, bool bClampToClosestPoint)
{
	if (inTetId == 0xFFFFFFFF)
	{
		outTetId = 0xFFFFFFFF;
		outTetBarycentric = PxVec4(0.0f);
		return;
	}

	// Map from CPU tet ID (corresponds to the ID in the BV4 mesh) to the GPU tet ID (corresponds to the ID in
	// the BV32 mesh)
	inTetId = collisionMesh.mGRB_faceRemapInverse[inTetId];

	const PxU32 endIdx = simState.mTetsAccumulatedRemapColToSim[inTetId];
	const PxU32 startIdx = inTetId != 0 ? simState.mTetsAccumulatedRemapColToSim[inTetId - 1] : 0;

	const PxU32* const tetRemapColToSim = simState.mTetsRemapColToSim;

	typedef PxVec4T<unsigned int> uint4;

	const uint4* const collInds = reinterpret_cast<const uint4*>(collisionMesh.mGRB_tetraIndices /*collisionMesh->mTetrahedrons*/);
	const uint4* const simInds = reinterpret_cast<const uint4*>(simMesh.getTetrahedrons());

	const PxVec3* const collVerts = collisionMesh.mVertices;
	const PxVec3* const simVerts = simMesh.getVertices();

	const uint4 ind = collInds[inTetId];

	const PxVec3 point = collVerts[ind.x] * inTetBarycentric.x + collVerts[ind.y] * inTetBarycentric.y +
	                     collVerts[ind.z] * inTetBarycentric.z + collVerts[ind.w] * inTetBarycentric.w;

	PxReal currDist = PX_MAX_F32;

	for (PxU32 i = startIdx; i < endIdx; ++i)
	{
		const PxU32 simTet = tetRemapColToSim[i];

		const uint4 simInd = simInds[simTet];

		const PxVec3 a = simVerts[simInd.x];
		const PxVec3 b = simVerts[simInd.y];
		const PxVec3 c = simVerts[simInd.z];
		const PxVec3 d = simVerts[simInd.w];

		const PxVec3 tmpClosest = closestPtPointTetrahedronWithInsideCheck(point, a, b, c, d);
		const PxVec3 v = point - tmpClosest;
		const PxReal tmpDist = v.dot(v);
		if (tmpDist < currDist)
		{
			PxVec4 tmpBarycentric;
			if (bClampToClosestPoint)
				PxComputeBarycentric(a, b, c, d, tmpClosest, tmpBarycentric);
			else
				PxComputeBarycentric(a, b, c, d, point, tmpBarycentric);

			currDist = tmpDist;
			outTetId = simTet;
			outTetBarycentric = tmpBarycentric;

			if (tmpDist < 1e-6f)
				break;
		}
	}

	PX_ASSERT(outTetId != 0xFFFFFFFF);
}

}
}