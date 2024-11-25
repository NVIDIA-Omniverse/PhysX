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

PxVec4 addAxisToSimMeshBarycentric(const PxTetrahedronMesh& simMesh, const PxU32 simTetId, const PxVec4& simBary, const PxVec3& axis)
{
	const PxVec3* simMeshVerts = simMesh.getVertices();
	PxVec3 tetVerts[4];
	if(simMesh.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES)
	{
		const PxU16* indices = reinterpret_cast<const PxU16*>(simMesh.getTetrahedrons());
		tetVerts[0] = simMeshVerts[indices[simTetId*4 + 0]];
		tetVerts[1] = simMeshVerts[indices[simTetId*4 + 1]];
		tetVerts[2] = simMeshVerts[indices[simTetId*4 + 2]];
		tetVerts[3] = simMeshVerts[indices[simTetId*4 + 3]];
	}
	else
	{
		const PxU32* indices = reinterpret_cast<const PxU32*>(simMesh.getTetrahedrons());
		tetVerts[0] = simMeshVerts[indices[simTetId*4 + 0]];
		tetVerts[1] = simMeshVerts[indices[simTetId*4 + 1]];
		tetVerts[2] = simMeshVerts[indices[simTetId*4 + 2]];
		tetVerts[3] = simMeshVerts[indices[simTetId*4 + 3]];
	}

	const PxVec3 simPoint = tetVerts[0]*simBary.x + tetVerts[1]*simBary.y + tetVerts[2]*simBary.z + tetVerts[3]*simBary.w;
	const PxVec3 offsetPoint = simPoint + axis;

	PxVec4 offsetBary;
	PxComputeBarycentric(tetVerts[0], tetVerts[1], tetVerts[2], tetVerts[3], offsetPoint, offsetBary);
	return offsetBary;
}


}
}