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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "PxgArticulationLink.h"
#include "DyArticulationJointCore.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "DyArticulationCore.h"
#include "PxgArticulationCoreKernelIndices.h"
#include "PxgBodySim.h" 
#include "reduction.cuh"
#include "copy.cuh"
#include "articulationDynamic.cuh"
#include "articulationImpulseResponse.cuh"
#include "MemoryAllocator.cuh"
#include "PxSpatialMatrix.h"
#include "foundation/PxMath.h"
#include "DyFeatherstoneArticulation.h"
#include "PxConstraintDesc.h"
#include "DyConstraintPrep.h"
#include "PxsRigidBody.h"
#include <stdio.h>

using namespace physx;
using namespace Dy;

extern "C" __host__ void initArticulationKernels4() {}

// VR: Old version for reference
#if 0
// This is a simple port of FeatherstoneArticulation::getDenseJacobian.
// We process one articulation per thread, which is not very efficient.
extern "C" __global__ void computeArtiDenseJacobians(
	const PxIndexDataPair* indexMap,
	const PxU32 nbIndices,
	const PxgArticulation* articulations)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbIndices)
	{
		PxU32 artiIndex = indexMap[globalThreadIndex].index;
		const PxgArticulation& arti = articulations[artiIndex];

		PxU32 linkCount = arti.data.numLinks;
		//PxU32 jointCount = linkCount - 1;
		PxU32 dofCount = arti.data.numJointDofs;
		bool fixBase = arti.data.flags & PxArticulationFlag::eFIX_BASE;

		PxU32 nCols = (fixBase ? 0 : 6) + dofCount;
		//PxU32 nRows = (fixBase ? 0 : 6) + jointCount * 6;

		//printf("~!~!~! processing arti %u (%u): %u links, %u dofs, fixed? %u, %u rows, %u cols\n",
		//	globalThreadIndex, artiIndex, linkCount, dofCount, PxU32(fixBase), nRows, nCols);

		float* dataPtr = static_cast<float*>(indexMap[globalThreadIndex].data);

#define jacobian(row, col) dataPtr[nCols * (row) + (col)]

		PxU32 destRow = 0;
		PxU32 destCol = 0;

		if (!fixBase)
		{
			jacobian(0, 0) = 1.0f;
			jacobian(0, 1) = 0.0f;
			jacobian(0, 2) = 0.0f;
			jacobian(0, 3) = 0.0f;
			jacobian(0, 4) = 0.0f;
			jacobian(0, 5) = 0.0f;

			jacobian(1, 0) = 0.0f;
			jacobian(1, 1) = 1.0f;
			jacobian(1, 2) = 0.0f;
			jacobian(1, 3) = 0.0f;
			jacobian(1, 4) = 0.0f;
			jacobian(1, 5) = 0.0f;

			jacobian(2, 0) = 0.0f;
			jacobian(2, 1) = 0.0f;
			jacobian(2, 2) = 1.0f;
			jacobian(2, 3) = 0.0f;
			jacobian(2, 4) = 0.0f;
			jacobian(2, 5) = 0.0f;

			jacobian(3, 0) = 0.0f;
			jacobian(3, 1) = 0.0f;
			jacobian(3, 2) = 0.0f;
			jacobian(3, 3) = 1.0f;
			jacobian(3, 4) = 0.0f;
			jacobian(3, 5) = 0.0f;

			jacobian(4, 0) = 0.0f;
			jacobian(4, 1) = 0.0f;
			jacobian(4, 2) = 0.0f;
			jacobian(4, 3) = 0.0f;
			jacobian(4, 4) = 1.0f;
			jacobian(4, 5) = 0.0f;

			jacobian(5, 0) = 0.0f;
			jacobian(5, 1) = 0.0f;
			jacobian(5, 2) = 0.0f;
			jacobian(5, 3) = 0.0f;
			jacobian(5, 4) = 0.0f;
			jacobian(5, 5) = 1.0f;

			destRow += 6;
			destCol += 6;
		}

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)//each iteration of this writes 6 rows in the matrix
		{
			//const PxgArticulationLink& link = arti.links[linkID];
			const PxTransform& body2World = arti.linkBody2Worlds[linkID];
			const ArticulationJointCoreData& jointDatum = arti.jointData[linkID];
			const PxU32 parentLinkID = arti.parents[linkID];

			if (parentLinkID || !fixBase)
			{
				// VR: It arti.jointData[0] isn't initialized
				//const PxU32 parentsFirstDestCol = arti.jointData[parentLinkID].jointOffset + (fixBase ? 0 : 6);
				const PxU32 parentsLastDestCol = parentLinkID ? arti.jointData[parentLinkID].jointOffset + (fixBase ? 0 : 6) + arti.jointData[parentLinkID].dof : 6;

				// VR: For parentLinkID == 0, it's two unsigned it overflows, though the result is still correct.
				const PxU32 parentsDestRow = (fixBase ? 0 : 6) + (parentLinkID - 1) * 6;

				// no commonInit, so need to compute the rw vector here
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLinkID];
				const PxVec3 rw = body2World.p - parentBody2World.p;

				// VR: parentsLastDestCol is rather column count than the last column index
				//for (PxU32 col = 0; col <= parentsLastDestCol; col++)
				for (PxU32 col = 0; col < parentsLastDestCol; col++)
				{
					//copy downward the 6 cols from parent
					const PxVec3 parentAng(jacobian(parentsDestRow + 3, col),
										   jacobian(parentsDestRow + 4, col),
										   jacobian(parentsDestRow + 5, col));

					const PxVec3 parentAngxRw = parentAng.cross(rw);

					jacobian(destRow + 0, col) = jacobian(parentsDestRow + 0, col) + parentAngxRw.x;
					jacobian(destRow + 1, col) = jacobian(parentsDestRow + 1, col) + parentAngxRw.y;
					jacobian(destRow + 2, col) = jacobian(parentsDestRow + 2, col) + parentAngxRw.z;

					jacobian(destRow + 3, col) = parentAng.x;
					jacobian(destRow + 4, col) = parentAng.y;
					jacobian(destRow + 5, col) = parentAng.z;
				}

				// VR: parentsLastDestCol is rather column count than the last column index
				//for (PxU32 col = parentsLastDestCol + 1; col < destCol; col++)
				for (PxU32 col = parentsLastDestCol; col < destCol; col++)
				{
					//fill with zeros.
					jacobian(destRow + 0, col) = 0.0f;
					jacobian(destRow + 1, col) = 0.0f;
					jacobian(destRow + 2, col) = 0.0f;

					jacobian(destRow + 3, col) = 0.0f;
					jacobian(destRow + 4, col) = 0.0f;
					jacobian(destRow + 5, col) = 0.0f;
				}
			}

			//diagonal block:
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& v = arti.motionMatrix[linkID][ind];

				const PxVec3 ang = body2World.rotate(v.top);
				const PxVec3 lin = body2World.rotate(v.bottom);

				jacobian(destRow + 0, destCol) = lin.x;
				jacobian(destRow + 1, destCol) = lin.y;
				jacobian(destRow + 2, destCol) = lin.z;

				jacobian(destRow + 3, destCol) = ang.x;
				jacobian(destRow + 4, destCol) = ang.y;
				jacobian(destRow + 5, destCol) = ang.z;

				destCol++;
			}

			//above diagonal block: always zero
			for (PxU32 col = destCol; col < nCols; col++)
			{
				jacobian(destRow + 0, col) = 0.0f;
				jacobian(destRow + 1, col) = 0.0f;
				jacobian(destRow + 2, col) = 0.0f;

				jacobian(destRow + 3, col) = 0.0f;
				jacobian(destRow + 4, col) = 0.0f;
				jacobian(destRow + 5, col) = 0.0f;
			}

			destRow += 6;
		}
#undef jacobian
	}
}
#endif

extern "C" __global__ void computeArtiDenseJacobians(
	float* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbIndices,
	const PxU32 maxLinks,
	const PxU32 maxDofs,
	const PxgArticulation* PX_RESTRICT articulations)
{
	const PxU32 jobIndex = threadIdx.y + blockIdx.x * blockDim.y;
	const PxU32 threadIndex = threadIdx.x;

	if (jobIndex < nbIndices)
	{
		const PxArticulationGPUIndex artiIndex = gpuIndices[jobIndex];
		const PxgArticulation& arti = articulations[artiIndex];

		const PxU32 linkCount = arti.data.numLinks;
		const PxU32 dofCount = arti.data.numJointDofs;
		const bool fixedBase = arti.data.flags & PxArticulationFlag::eFIX_BASE;
		const PxU32 baseDofs = fixedBase ? 0 : 6;
		const PxU32 colCount = baseDofs + dofCount;

		const PxU32 maxCols = 6 + maxDofs;
		const PxU32 maxRows = 6 + (maxLinks - 1) * 6;

		float* PX_RESTRICT jacobian = &data[jobIndex * maxCols * maxRows];

		if (!fixedBase)
		{
			for (PxU32 i = threadIndex; i < 6 * colCount; i += WARP_SIZE)
				jacobian[i] = (i / colCount == i % colCount) ? 1.0f : 0.0f;

			__syncwarp();
		}

		for (PxU32 link = 1; link < linkCount; ++link)
		{
			const PxU32 linkOffset = arti.jointData[link].jointOffset + baseDofs;
			const PxU32 linkDofs = arti.jointData[link].nbDof;
			const PxU32 linkRow0 = (link - 1) * 6 + baseDofs;
			const PxTransform& body2World = arti.linkBody2Worlds[link];
			const PxU32 parentLink = arti.parents[link];
			const PxU32 parentDofs = parentLink ? arti.jointData[parentLink].jointOffset + arti.jointData[parentLink].nbDof + baseDofs : baseDofs;

			for (PxU32 dof = threadIndex; dof < colCount; dof += WARP_SIZE)
			{
				PxVec3 linkLin(0), linkAng(0);

				if (dof < parentDofs)
				{
					const PxU32 parentRow0 = parentLink ? (parentLink - 1) * 6 + baseDofs : 0;

					const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
					const PxVec3 rw = body2World.p - parentBody2World.p;

					const PxVec3 parentLin(jacobian[(parentRow0 + 0) * colCount + dof],
						jacobian[(parentRow0 + 1) * colCount + dof],
						jacobian[(parentRow0 + 2) * colCount + dof]);

					const PxVec3 parentAng(jacobian[(parentRow0 + 3) * colCount + dof],
						jacobian[(parentRow0 + 4) * colCount + dof],
						jacobian[(parentRow0 + 5) * colCount + dof]);

					const PxVec3 parentAngxRw = parentAng.cross(rw);

					linkLin = parentLin + parentAngxRw;
					linkAng = parentAng;
				}
				else if (dof >= linkOffset && dof < linkOffset + linkDofs)
				{
					const Cm::UnAlignedSpatialVector& v = arti.motionMatrix[link][dof - linkOffset];

					linkLin = body2World.rotate(v.bottom);
					linkAng = body2World.rotate(v.top);
				}

				__syncwarp();

				jacobian[(linkRow0 + 0) * colCount + dof] = linkLin.x;
				jacobian[(linkRow0 + 1) * colCount + dof] = linkLin.y;
				jacobian[(linkRow0 + 2) * colCount + dof] = linkLin.z;

				jacobian[(linkRow0 + 3) * colCount + dof] = linkAng.x;
				jacobian[(linkRow0 + 4) * colCount + dof] = linkAng.y;
				jacobian[(linkRow0 + 5) * colCount + dof] = linkAng.z;
			}
		}
	}
}

__device__ static void translateInertia(const PxMat33& sTod, Dy::SpatialMatrix& inertia)
{
	const PxMat33 dTos = sTod.getTranspose();

	PxMat33 bl = sTod * inertia.topLeft + inertia.bottomLeft;
	PxMat33 br = sTod * inertia.topRight + inertia.getBottomRight();

	inertia.topLeft = inertia.topLeft + inertia.topRight * dTos;
	inertia.bottomLeft = bl + br * dTos;

	//aligned inertia - make it symmetrical! OPTIONAL!!!!
	inertia.bottomLeft = (inertia.bottomLeft + inertia.bottomLeft.getTranspose()) * 0.5f;
}

// This is a simple port of FeatherstoneArticulation::getGeneralizedMassMatrixCRB.
// I process one articulation per warp (32 threads).
extern "C" __global__ void computeArtiMassMatrices(
	const PxU32 nbIndices,
	float* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 maxDofs,
	const bool rootMotion,
	const PxgArticulation* PX_RESTRICT articulations)
{
	const PxU32 jobIndex = threadIdx.y + blockIdx.x * blockDim.y;
	const PxU32 threadIndex = threadIdx.x;

	if (jobIndex < nbIndices)
	{
		const PxU32 artiIndex = gpuIndices[jobIndex];
		const PxgArticulation& arti = articulations[artiIndex];

		// I use this as a tmp buffer to store world space spatial inertias of all links
		Dy::SpatialMatrix* PX_RESTRICT spatialInertias = reinterpret_cast<Dy::SpatialMatrix*>(arti.worldSpatialArticulatedInertia); // ???
		// I use this as a tmp buffer to store world space motiom vectors multiplied by their links inertias
		Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrices = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.worldMotionMatrix); // ???

		const PxU32 linkCount = arti.data.numLinks;
		const PxU32 dofCount = arti.data.numJointDofs;
		const bool fixBase = arti.data.flags & PxArticulationFlag::eFIX_BASE;
		const PxU32 rootDof = (rootMotion && !fixBase) ? 6 : 0; // Add the DoF of the root in the floating base case
		const PxU32 bufferDof = rootMotion ? 6 : 0;
		const PxU32 matSize = dofCount + rootDof;

		float* massMatrix = &data[jobIndex * (maxDofs + bufferDof) * (maxDofs + bufferDof)];
		for (int i = threadIndex; i < matSize * matSize; i += WARP_SIZE)
			massMatrix[i] = 0;

		__syncwarp();

		for (PxU32 link = threadIndex; link < linkCount; link += WARP_SIZE)
		{
			const float4 invIM = arti.linkProps[link].invInertiaXYZ_invMass;
			const PxReal mass = invIM.w == 0.f ? 0.f : (1.f / invIM.w);
			const PxVec3 localInertia = PxVec3(invIM.x == 0.f ? 0.f : (1.f / invIM.x),
												invIM.y == 0.f ? 0.f : (1.f / invIM.y),
												invIM.z == 0.f ? 0.f : (1.f / invIM.z));
			Dy::SpatialMatrix spatialInertia;
			spatialInertia.topLeft = PxMat33(PxZero);
			spatialInertia.topRight = PxMat33::createDiagonal(PxVec3(mass));
			spatialInertia.padding = 0; // VR: I use this to store the order to compute combined inertias
			Cm::transformInertiaTensor(localInertia, PxMat33(arti.linkBody2Worlds[link].q), spatialInertia.bottomLeft);
			spatialInertias[link] = spatialInertia;
		}

		__syncwarp();

#if 1 // VR: This optimization allows computing some of combined inertias in parallel. It's a bit faster for humanoids, but not much.
		PxU32 maxStep = 0;
		if (threadIndex == 0)
		{
			for (PxU32 link = linkCount - 1; link > 0; --link)
			{
				PxU32& s = spatialInertias[link].padding;
				const PxU32 parentLink = arti.parents[link];
				PxU32& ps = spatialInertias[parentLink].padding;
				if (s < ps) s = ps;
				ps = s + 1;
				if (maxStep < ps) maxStep = ps;
			}
		}
		__syncwarp();
		maxStep = __shfl_sync(0xffffffff, maxStep, 0);
		for (PxU32 step = 0; step <= maxStep; ++step)
		{
			for (PxU32 link = threadIndex + 1; link < linkCount; link += WARP_SIZE)
			{
				const PxU32& s = spatialInertias[link].padding;
				if (s == step)
				{
					const PxTransform& body2World = arti.linkBody2Worlds[link];
					const PxU32 parentLink = arti.parents[link];
					const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
					const PxVec3 rw = body2World.p - parentBody2World.p;
					PxMat33 skewSymmetric(PxVec3(0, rw.z, -rw.y), PxVec3(-rw.z, 0, rw.x), PxVec3(rw.y, -rw.x, 0));
					Dy::SpatialMatrix parentSpaceSpatialInertia = spatialInertias[link];
					translateInertia(skewSymmetric, parentSpaceSpatialInertia);
					spatialInertias[parentLink] += parentSpaceSpatialInertia;
				}
			}
			__syncwarp();
		}
#else
		if (threadIndex == 0) // @@@ ??? Can I optimize it somehow?
		{
			for (PxU32 link = linkCount - 1; link > 0; --link)
			{
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				const PxU32 parentLink = arti.parents[link];
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
				const PxVec3 rw = body2World.p - parentBody2World.p;
				PxMat33 skewSymmetric(PxVec3(0, rw.z, -rw.y), PxVec3(-rw.z, 0, rw.x), PxVec3(rw.y, -rw.x, 0));
				Dy::SpatialMatrix parentSpaceSpatialInertia = spatialInertias[link];
				translateInertia(skewSymmetric, parentSpaceSpatialInertia);
				spatialInertias[parentLink] += parentSpaceSpatialInertia;
			}
		}

		__syncwarp();
#endif
		// Loop is for the case where there are more than 32 links in one articulation
		for (PxU32 link = threadIndex + 1; link < linkCount; link += WARP_SIZE)
		{
			const PxTransform& body2World = arti.linkBody2Worlds[link];
			const ArticulationJointCoreData& jointData = arti.jointData[link];
			const Dy::SpatialMatrix& spatialInertia = spatialInertias[link];

			Cm::UnAlignedSpatialVector worldMotionMatrix[3];
			Cm::UnAlignedSpatialVector* spatialInertia_worldMotionMatrix = &worldMotionMatrices[jointData.jointOffset];
			for (PxU32 dof = 0; dof < jointData.nbDof; ++dof)
			{
				const Cm::UnAlignedSpatialVector& m = arti.motionMatrix[link][dof];
				worldMotionMatrix[dof] = m.rotate(body2World);
				spatialInertia_worldMotionMatrix[dof] = spatialInertia * worldMotionMatrix[dof];
			}

			// A unit joint acceleration is assumed for each dof of the considered link
			// This calculates the diagonal terms using the spatial articulated inertia for obtaining
			// the force applied to the considered dof due to the applied unit joint acceleration
			for (PxU32 dof0 = 0; dof0 < jointData.nbDof; ++dof0)
			{
				const Cm::UnAlignedSpatialVector& Im0 = spatialInertia_worldMotionMatrix[dof0];
				const PxU32 row = jointData.jointOffset + dof0 + rootDof;
				for (PxU32 dof1 = 0; dof1 < jointData.nbDof; ++dof1)
				{
					const Cm::UnAlignedSpatialVector& m1 = worldMotionMatrix[dof1];
					const PxU32 col = jointData.jointOffset + dof1 + rootDof;
					massMatrix[row * matSize + col] = m1.innerProduct(Im0);
				}
			}

			// The calculated force is then propagated inward to calculate the above diagonal terms of the mass matrix
			// This is sufficient as the mass matrix is symmetric
			PxTransform childBody2World = body2World;
			PxU32 parentLink = arti.parents[link];
			while (true)
			{
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];

				// The zero acceleration force is propagated inward
				// This is correct because all links inward the considered link have no joint acceleration
				const PxVec3 rw = childBody2World.p - parentBody2World.p;
				for (PxU32 dof = 0; dof < jointData.nbDof; ++dof)
				{
					spatialInertia_worldMotionMatrix[dof] = FeatherstoneArticulation::translateSpatialVector(rw, spatialInertia_worldMotionMatrix[dof]);
				}
				childBody2World = parentBody2World;

				if (parentLink == 0)
				{
					// For floating base, calculate the resulting force on the root
					if (!fixBase && rootMotion)
					{
						for (PxU32 dof0 = 0; dof0 < jointData.nbDof; ++dof0)
						{
							const Cm::UnAlignedSpatialVector& Im0 = spatialInertia_worldMotionMatrix[dof0];
							const PxU32 col = jointData.jointOffset + dof0 + rootDof;
							for (PxU32 row = 0; row < 6; ++row)
							{
								massMatrix[col * matSize + row] += Im0[row];
								massMatrix[col + row * matSize] += Im0[row];
							}
						}
					}
					break;
				}

				const ArticulationJointCoreData& parentJointData = arti.jointData[parentLink];

				Cm::UnAlignedSpatialVector parentWorldMotionMatrix[3];
				for (PxU32 dof = 0; dof < parentJointData.nbDof; ++dof)
				{
					const Cm::UnAlignedSpatialVector& m = arti.motionMatrix[parentLink][dof];
					parentWorldMotionMatrix[dof].top = parentBody2World.rotate(m.top);
					parentWorldMotionMatrix[dof].bottom = parentBody2World.rotate(m.bottom);
				}

				// The joint force is calculated from the zero acceleration force
				for (PxU32 dof0 = 0; dof0 < jointData.nbDof; ++dof0)
				{
					const Cm::UnAlignedSpatialVector& Im0 = spatialInertia_worldMotionMatrix[dof0];
					const PxU32 row = jointData.jointOffset + dof0 + rootDof;
					for (PxU32 dof1 = 0; dof1 < parentJointData.nbDof; ++dof1)
					{
						const Cm::UnAlignedSpatialVector& m1 = parentWorldMotionMatrix[dof1];
						const PxU32 col = parentJointData.jointOffset + dof1 + rootDof;
						float m = m1.innerProduct(Im0);
						massMatrix[row * matSize + col] = m;
						massMatrix[row + col * matSize] = m;
					}
				}

				parentLink = arti.parents[parentLink];
			}
		}

		// Adding the spatial articulated inertia of the root
		if (threadIndex == 0 && !fixBase && rootMotion)
		{
			// Note that the spatial articulated inertia assumes that the root angular acceleration comes first,
			// while the mass matrix assumes that the root linear acceleration comes first
			// We have therefore to invert the angular and linear component of the spatial articulated inertia
			// This also ensures that the mass matrix is symmetric
			const PxReal* rootSpatialInertia = reinterpret_cast<PxReal*>(&spatialInertias[0]);
			for (PxU32 row = 0; row < 6; ++row)
			{
				const PxU32 rowSpatialInertia = (row < 3) ? row : row - 3; // Convert to the index of a 3 x 3 matrix
				// Only process elements above the diagonal as the matrix is symmetric
				for (PxU32 col = row; col < 6; ++col)
				{
					// This offset is due to how the spatial matrix is indexed and how the linear amd angular components
					// of the acceleration should be inverted, the index is as follows
					//	0	3	6	9	12	15								9	12	15	0	3	6
					//	1	4	7	10	13	16								10	23	16	1	4	7
					//	2	5	8	11	14	17	inversion linear/angular	11	24	17	2	5	8
					//	18	21	24	0	1	2	------------------------>	0	1	2	18	21	24
					//	19	22	25	3	4	5								3	4	5	19	22	25
					//	20	23	26	6	7	8								6	7	8	20	23	26
					const PxU32 offset = (row > 2) ? 18 : (col < 3) * 9;
					const PxU32 colSpatialInertia = (col < 3) ? col : col - 3;  // Convert to the index of a 3 x 3 matrix
					const PxU32 index = offset + colSpatialInertia * 3 + rowSpatialInertia;
					massMatrix[row * matSize + col] = rootSpatialInertia[index];
					massMatrix[col * matSize + row] = rootSpatialInertia[index];
				}
			}
		}
		else if (!fixBase && !rootMotion)
		{
			__syncwarp();

			Dy::SpatialMatrix baseInvI = spatialInertias[0].invertInertia();

			for (PxU32 row = threadIndex; row < dofCount; row += WARP_SIZE)
			{
				const Cm::UnAlignedSpatialVector& m0 = worldMotionMatrices[row];
				for (PxU32 col = 0; col < dofCount; ++col)
				{
					const Cm::UnAlignedSpatialVector& m1 = worldMotionMatrices[col];
					massMatrix[row * dofCount + col] -= m0.innerProduct(baseInvI * m1);
				}
			}
		}
		__syncwarp();
	}
}

__device__ static Cm::UnAlignedSpatialVector translateSpatialVector(const PxVec3& offset, const Cm::UnAlignedSpatialVector& vec)
{
	return Cm::UnAlignedSpatialVector(vec.top, vec.bottom + offset.cross(vec.top));
}

// This is an optimized port of FeatherstoneArticulation::getGeneralizedGravityForce.
// We process one articulation per warp (32 threads).
extern "C" __global__ void computeArtiGravityForces(
	const PxU32 nbIndices,
	float* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 maxDofs,
	const bool rootMotion,
	const PxgArticulation* articulations, const PxVec3 gravity)
{
	const PxU32 jobIndex = threadIdx.y + blockIdx.x * blockDim.y;
	const PxU32 threadIndex = threadIdx.x;

	if (jobIndex < nbIndices)
	{
		const PxU32 artiIndex = gpuIndices[jobIndex];
		const PxgArticulation& arti = articulations[artiIndex];

		const PxU32 linkCount = arti.data.numLinks;
		const PxU32 dofCount = arti.data.numJointDofs;
		const bool fixBase = arti.data.flags & PxArticulationFlag::eFIX_BASE;
		const PxU32 rootDof = (rootMotion && !fixBase) ? 6 : 0; // Add the DoF of the root in the floating base case
		const PxU32 bufferDof = rootMotion ? 6 : 0;
		const PxVec3 tGravity = -gravity;

		float* PX_RESTRICT gravityCompensationForces = &data[jobIndex * (maxDofs + bufferDof)];

		if (rootMotion || fixBase)
		{
			// I use this as a tmp buffer to store ZAForce vectors of all links
			Cm::UnAlignedSpatialVector* PX_RESTRICT zAForces = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.zAForces); // ???

			for (PxU32 link = threadIndex; link < linkCount; link += WARP_SIZE)
			{
				const float4 invIM = arti.linkProps[link].invInertiaXYZ_invMass;
				const PxReal mass = invIM.w == 0.f ? 0.f : (1.f / invIM.w);
				zAForces[link].top = tGravity * mass;
				zAForces[link].bottom = PxVec3(0);
			}

			__syncwarp();

			for (PxU32 link = (linkCount - 1); link > 0; --link)
			{
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				const PxU32 parentLink = arti.parents[link];
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];

				if (threadIndex == 0)
					zAForces[parentLink] += translateSpatialVector(body2World.p - parentBody2World.p, zAForces[link]);

				const ArticulationJointCoreData& jointData = arti.jointData[link];
				PxReal* force = &gravityCompensationForces[jointData.jointOffset];

				if (threadIndex < jointData.nbDof)
					force[threadIndex + rootDof] = link; // I'll just store dof's link index here and use it in the next loop
			}

			__syncwarp();

			for (PxU32 dof = threadIndex; dof < dofCount; dof += WARP_SIZE)
			{
				PxU32 link = (PxU32)gravityCompensationForces[dof + rootDof];
				const ArticulationJointCoreData& jointData = arti.jointData[link];
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				Cm::UnAlignedSpatialVector dofMotion = arti.motionMatrix[link][dof - jointData.jointOffset].rotate(body2World);
				gravityCompensationForces[dof + rootDof] = dofMotion.innerProduct(zAForces[link]);
			}

			// Add root DoFs contribution
			if (threadIndex == 0 && rootDof == 6)
			{
				gravityCompensationForces[0] = zAForces[0].top.x;
				gravityCompensationForces[1] = zAForces[0].top.y;
				gravityCompensationForces[2] = zAForces[0].top.z;
				gravityCompensationForces[3] = zAForces[0].bottom.x;
				gravityCompensationForces[4] = zAForces[0].bottom.y;
				gravityCompensationForces[5] = zAForces[0].bottom.z;
			}

			__syncwarp();
		}
		else
		{
			// I use this as a tmp buffer to store world space spatial inertias of all links
			Dy::SpatialMatrix* PX_RESTRICT spatialInertias = reinterpret_cast<Dy::SpatialMatrix*>(arti.worldSpatialArticulatedInertia); // ???
			// I use this as a tmp buffer to store motion velocities of all links ??? Or not. The velocities seem valid. Maybe I don't need to compute them ???
			Cm::UnAlignedSpatialVector* PX_RESTRICT motionVelocities = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.motionVelocities); // ???
			// I use this as a tmp buffer to store motion accelerations of all links
			Cm::UnAlignedSpatialVector* PX_RESTRICT motionAccelerations = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.motionAccelerations); // ???
			// I use this as a tmp buffer to store ZAForce vectors of all links
			Cm::UnAlignedSpatialVector* PX_RESTRICT zAForces = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.zAForces); // ???

			// Compute links inertias

			for (PxU32 link = threadIndex; link < linkCount; link += WARP_SIZE)
			{
				const float4 invIM = arti.linkProps[link].invInertiaXYZ_invMass;
				const PxReal mass = invIM.w == 0.f ? 0.f : (1.f / invIM.w);
				const PxVec3 localInertia = PxVec3(invIM.x == 0.f ? 0.f : (1.f / invIM.x), invIM.y == 0.f ? 0.f : (1.f / invIM.y), invIM.z == 0.f ? 0.f : (1.f / invIM.z));
				Dy::SpatialMatrix spatialInertia;
				spatialInertia.topLeft = PxMat33(PxZero);
				spatialInertia.topRight = PxMat33::createDiagonal(PxVec3(mass));
				Cm::transformInertiaTensor(localInertia, PxMat33(arti.linkBody2Worlds[link].q), spatialInertia.bottomLeft);
				spatialInertias[link] = spatialInertia;
			}

			__syncwarp();

			if (threadIndex == 0) // @@@ ??? Can I optimize it somehow?
			{
				for (PxU32 link = linkCount - 1; link > 0; --link)
				{
					const PxTransform& body2World = arti.linkBody2Worlds[link];
					const PxU32 parentLink = arti.parents[link];
					const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
					const PxVec3 rw = body2World.p - parentBody2World.p;
					PxMat33 skewSymmetric(PxVec3(0, rw.z, -rw.y), PxVec3(-rw.z, 0, rw.x), PxVec3(rw.y, -rw.x, 0));
					Dy::SpatialMatrix parentSpaceSpatialInertia = spatialInertias[link];
					translateInertia(skewSymmetric, parentSpaceSpatialInertia);
					spatialInertias[parentLink] += parentSpaceSpatialInertia;
				}
			}

			__syncwarp();

			// Compute motion velocities ??? Should I even do this?

			if (threadIndex == 0)
			{
				for (PxU32 link = 1; link < linkCount; ++link)
				{
					const PxTransform& body2World = arti.linkBody2Worlds[link];
					const PxU32 parentLink = arti.parents[link];
					const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
					motionVelocities[link] = translateSpatialVector(parentBody2World.p - body2World.p, motionVelocities[parentLink]);
				}
			}

			__syncwarp();

			// Init za-forces

			for (PxU32 link = threadIndex; link < linkCount; link += WARP_SIZE)
			{
				const float4 invIM = arti.linkProps[link].invInertiaXYZ_invMass;
				const PxReal mass = invIM.w == 0.f ? 0.f : (1.f / invIM.w);
				const PxMat33& I = spatialInertias[link].bottomLeft;
				const PxVec3& vA = motionVelocities[link].top;
				zAForces[link].top = -tGravity * mass; // ??? @@@ Yea, fix-base version has no minus here.
				zAForces[link].bottom = vA.cross(I * vA);
			}

			__syncwarp();

			for (PxU32 link = (linkCount - 1); link > 0; --link)
			{
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				const PxU32 parentLink = arti.parents[link];
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];

				if (threadIndex == 0)
					zAForces[parentLink] += translateSpatialVector(body2World.p - parentBody2World.p, zAForces[link]);
			}

			__syncwarp();

			// Compute motion acceleration

			if (threadIndex == 0)
			{
				Dy::SpatialMatrix invInertia = spatialInertias[0].invertInertia();
				motionAccelerations[0] = -(invInertia * zAForces[0]);
			}
			for (PxU32 link = 1; link < linkCount; ++link)
			{
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				const PxU32 parentLink = arti.parents[link];
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
				if (threadIndex == 0)
				{
					motionAccelerations[link] = translateSpatialVector(parentBody2World.p - body2World.p, motionAccelerations[parentLink]);
					zAForces[link] = spatialInertias[link] * motionAccelerations[link] + zAForces[link];
				}

				const ArticulationJointCoreData& jointData = arti.jointData[link];
				PxReal* force = &gravityCompensationForces[jointData.jointOffset];

				if (threadIndex < jointData.nbDof)
					force[threadIndex] = link; // I'll just store dof's link index here and use it in the next loop
			}

			__syncwarp();

			// Compute joint forces

			for (PxU32 dof = threadIndex; dof < dofCount; dof += WARP_SIZE)
			{
				PxU32 link = (PxU32)gravityCompensationForces[dof];
				const ArticulationJointCoreData& jointData = arti.jointData[link];
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				Cm::UnAlignedSpatialVector dofMotion = arti.motionMatrix[link][dof - jointData.jointOffset].rotate(body2World);
				gravityCompensationForces[dof] = dofMotion.innerProduct(zAForces[link]);
			}

			__syncwarp();
		}
	}
}

// This is an optimized port of FeatherstoneArticulation::getCoriolisAndCentrifugalForce.
// We process one articulation per warp (32 threads).
extern "C" __global__ void computeArtiCentrifugalForces(
	const PxU32 nbIndices,
	float* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 maxDofs,
	const bool rootMotion,
	const PxgArticulation* articulations)
{
	const PxU32 jobIndex = threadIdx.y + blockIdx.x * blockDim.y;
	const PxU32 threadIndex = threadIdx.x;

	if (jobIndex < nbIndices)
	{
		const PxU32 artiIndex = gpuIndices[jobIndex];
		const PxgArticulation& arti = articulations[artiIndex];

		Dy::SpatialMatrix* PX_RESTRICT spatialInertias = reinterpret_cast<Dy::SpatialMatrix*>(arti.worldSpatialArticulatedInertia);
		PxReal* PX_RESTRICT jointVelocities = reinterpret_cast<PxReal*>(arti.jointVelocities);
		Cm::UnAlignedSpatialVector* PX_RESTRICT motionVelocities = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.motionVelocities);
		Cm::UnAlignedSpatialVector* PX_RESTRICT corioliseVectors = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.corioliseVectors);
		// corioliseVectors is used as a tmp buffer to store motion accelerations of all links
		// arti.motionAccelerations cannot be used because it stores important information that should not be erased
		Cm::UnAlignedSpatialVector* PX_RESTRICT motionAccelerations = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.corioliseVectors);
		Cm::UnAlignedSpatialVector* PX_RESTRICT zAForces = reinterpret_cast<Cm::UnAlignedSpatialVector*>(arti.zAForces);

		const PxU32 linkCount = arti.data.numLinks;
		const PxU32 dofCount = arti.data.numJointDofs;
		const bool fixBase = arti.data.flags & PxArticulationFlag::eFIX_BASE;
		const PxU32 rootDof = (rootMotion && !fixBase) ? 6 : 0; // Add the DoF of the root in the floating base case
		const PxU32 bufferDof = rootMotion ? 6 : 0;

		float* PX_RESTRICT coriolisForces = &data[jobIndex * (maxDofs + bufferDof)];

		// Velocities
		// It seems to be unnecessary to recalculate the motion velocities as we always call
		// the update kinematic function before calling this function
		if (threadIndex == 0)
		{
			for (PxU32 link = 1; link < linkCount; ++link)
			{
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				const PxU32 parentLink = arti.parents[link];
				const ArticulationJointCoreData& jointData = arti.jointData[link];
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
				const PxVec3 rw = body2World.p - parentBody2World.p;
				const Cm::UnAlignedSpatialVector pVel = motionVelocities[parentLink];

				Cm::UnAlignedSpatialVector vel = translateSpatialVector(-rw, pVel);

				Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
				for (PxU32 dof = 0; dof < jointData.nbDof; ++dof)
				{
					const PxReal maxJointVelocity = arti.joints[link].maxJointVelocity[dof];
					const PxReal jointVelocity = jointVelocities[jointData.jointOffset + dof];
					PxReal jVel = PxMin(jointVelocity, maxJointVelocity);
					Cm::UnAlignedSpatialVector dofMotion = arti.motionMatrix[link][dof].rotate(body2World);
					deltaV += dofMotion * jVel;
				}

				vel.top += deltaV.top;
				vel.bottom += deltaV.bottom;

				const PxVec3 aVec = deltaV.top;
				const PxVec3 force = pVel.top.cross(aVec);
				const PxVec3 lVel = deltaV.bottom;
				const PxVec3 torque = pVel.top.cross(pVel.top.cross(rw)) + 2.f * pVel.top.cross(lVel) + aVec.cross(lVel);

				corioliseVectors[link] = Cm::SpatialVectorF(force, torque);
				motionVelocities[link] = vel;
			}
		}

		__syncwarp();

		if (threadIndex == 0)
		{
			motionAccelerations[0] = Cm::SpatialVectorF::Zero();

			for (PxU32 link = 1; link < linkCount; ++link)
			{
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				const PxU32 parentLink = arti.parents[link];
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
				const PxVec3 rw = body2World.p - parentBody2World.p;

				Cm::UnAlignedSpatialVector pMotionAcceleration = translateSpatialVector(-rw, motionAccelerations[parentLink]);

				motionAccelerations[link] = pMotionAcceleration + corioliseVectors[link];
			}
		}

		__syncwarp();

		for (PxU32 link = threadIndex; link < linkCount; link += WARP_SIZE)
		{
			const PxTransform& body2World = arti.linkBody2Worlds[link];
			const float4 invIM = arti.linkProps[link].invInertiaXYZ_invMass;
			const PxReal mass = invIM.w == 0.f ? 0.f : (1.f / invIM.w);
			const PxVec3 localInertia = PxVec3(invIM.x == 0.f ? 0.f : (1.f / invIM.x), invIM.y == 0.f ? 0.f : (1.f / invIM.y), invIM.z == 0.f ? 0.f : (1.f / invIM.z));
			PxMat33 I; Cm::transformInertiaTensor(localInertia, PxMat33(body2World.q), I);
			const PxVec3& vA = motionVelocities[link].top;
			zAForces[link].top = motionAccelerations[link].bottom * mass;
			zAForces[link].bottom = vA.cross(I * vA) + I * motionAccelerations[link].top;
		}

		__syncwarp();

		if (threadIndex == 0)
		{
			for (PxU32 link = (linkCount - 1); link > 0; --link)
			{
				const PxTransform& body2World = arti.linkBody2Worlds[link];
				const PxU32 parentLink = arti.parents[link];
				const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];

				zAForces[parentLink] += translateSpatialVector(body2World.p - parentBody2World.p, zAForces[link]);

				const ArticulationJointCoreData& jointData = arti.jointData[link];
				PxReal* force = &coriolisForces[jointData.jointOffset];

				for (PxU32 dof = 0; dof < jointData.nbDof; ++dof)
					force[dof + rootDof] = link; // I'll just store dof's link index here and use it in the last loop
			}
		}

		__syncwarp();

		if (!fixBase)
		{
			// Compute links inertias

			for (PxU32 link = threadIndex; link < linkCount; link += WARP_SIZE)
			{
				const float4 invIM = arti.linkProps[link].invInertiaXYZ_invMass;
				const PxReal mass = invIM.w == 0.f ? 0.f : (1.f / invIM.w);
				const PxVec3 localInertia = PxVec3(invIM.x == 0.f ? 0.f : (1.f / invIM.x), invIM.y == 0.f ? 0.f : (1.f / invIM.y), invIM.z == 0.f ? 0.f : (1.f / invIM.z));
				Dy::SpatialMatrix spatialInertia;
				spatialInertia.topLeft = PxMat33(PxZero);
				spatialInertia.topRight = PxMat33::createDiagonal(PxVec3(mass));
				Cm::transformInertiaTensor(localInertia, PxMat33(arti.linkBody2Worlds[link].q), spatialInertia.bottomLeft);
				spatialInertias[link] = spatialInertia;
			}

			__syncwarp();

			if (threadIndex == 0)
			{
				for (PxU32 link = linkCount - 1; link > 0; --link)
				{
					const PxTransform& body2World = arti.linkBody2Worlds[link];
					const PxU32 parentLink = arti.parents[link];
					const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
					const PxVec3 rw = body2World.p - parentBody2World.p;
					PxMat33 skewSymmetric(PxVec3(0, rw.z, -rw.y), PxVec3(-rw.z, 0, rw.x), PxVec3(rw.y, -rw.x, 0));
					Dy::SpatialMatrix parentSpaceSpatialInertia = spatialInertias[link];
					translateInertia(skewSymmetric, parentSpaceSpatialInertia);
					spatialInertias[parentLink] += parentSpaceSpatialInertia;
				}
			}

			__syncwarp();

			// Compute motion acceleration

			if (threadIndex == 0)
			{
				if (!rootMotion)
				{
					Dy::SpatialMatrix invInertia = spatialInertias[0].invertInertia();
					motionAccelerations[0] = -(invInertia * zAForces[0]);
				}

				for (PxU32 link = 1; link < linkCount; ++link)
				{
					const PxTransform& body2World = arti.linkBody2Worlds[link];
					const PxU32 parentLink = arti.parents[link];
					const PxTransform& parentBody2World = arti.linkBody2Worlds[parentLink];
					motionAccelerations[link] = translateSpatialVector(parentBody2World.p - body2World.p, motionAccelerations[parentLink]);
					zAForces[link] = spatialInertias[link] * motionAccelerations[link] + zAForces[link];
				}
			}

			__syncwarp();
		}

		// Compute joint forces

		for (PxU32 dof = threadIndex; dof < dofCount; dof += WARP_SIZE)
		{
			PxU32 link = (PxU32)coriolisForces[dof + rootDof];
			const ArticulationJointCoreData& jointData = arti.jointData[link];
			const PxTransform& body2World = arti.linkBody2Worlds[link];
			Cm::UnAlignedSpatialVector dofMotion = arti.motionMatrix[link][dof - jointData.jointOffset].rotate(body2World);
			coriolisForces[dof + rootDof] = dofMotion.innerProduct(zAForces[link]);
		}

		// Add root contribution

		if (rootMotion && threadIndex == 0 && !fixBase)
		{
			coriolisForces[0] = zAForces[0].top.x;
			coriolisForces[1] = zAForces[0].top.y;
			coriolisForces[2] = zAForces[0].top.z;
			coriolisForces[3] = zAForces[0].bottom.x;
			coriolisForces[4] = zAForces[0].bottom.y;
			coriolisForces[5] = zAForces[0].bottom.z;
		}

		__syncwarp();
	}
}

// This is an optimized port of FeatherstoneArticulation::getArticulationCOM
// We process one articulation per warp (32 threads) and one link per thread.
// We have to separate the calculation of the total mass from the calculation
// of the sum of COM in case there are more than 32 links in one articulation.
extern "C" __global__ void computeArtiCOM(
	PxVec3* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const bool rootFrame,
	const PxgArticulation* PX_RESTRICT articulations)
{
	const PxU32 jobIndex = threadIdx.y + blockIdx.x * blockDim.y;
	const PxU32 threadIndex = threadIdx.x;

	const PxU32 artiIndex = gpuIndices[jobIndex];
	const PxgArticulation& arti = articulations[artiIndex];

	PxVec3* articulationCOM = &data[jobIndex];
	const PxU32 linkCount = arti.data.numLinks;

	// calculate the total mass of the articulation and store it temporarily in articulationCOM->x
	for(PxU32 link = threadIndex; link < linkCount; link += WARP_SIZE)
	{
		const PxReal mass = 1.0f / arti.linkProps[link].invInertiaXYZ_invMass.w;

		// calculate the total mass and store it temporarily in articulationCOM->x
		PxRedAddGlobal(&articulationCOM->x, mass);
	}

	__syncwarp();

	PxReal totalMass = articulationCOM->x;

	// calculate the COM as the sum of body COM
	for(PxU32 link = threadIndex; link < linkCount; link += WARP_SIZE)
	{
		const PxVec3 childPose = arti.linkBody2Worlds[link].p;
		const PxReal mass = 1.0f / arti.linkProps[link].invInertiaXYZ_invMass.w;
		const PxVec3 COM = childPose * mass;

		// calculate the COM as the sum of body COM
		PxRedAddGlobal(&articulationCOM->x, COM[0]);
		PxRedAddGlobal(&articulationCOM->y, COM[1]);
		PxRedAddGlobal(&articulationCOM->z, COM[2]);
	}

	if(threadIndex == 0)
	{
		// remove the total mass from articulationCOM->x
		articulationCOM->x -= totalMass;

		*articulationCOM /= totalMass;

		// move to the root frame
		if(rootFrame)
		{
			*articulationCOM = arti.linkBody2Worlds[0].getInverse().transform(*articulationCOM);
		}
	}

	__syncwarp();
}

// This is an optimized port of FeatherstoneArticulation::getCentroidalMomentumMatrix
// We process one articulation per warp (32 threads) and one link per thread.
extern "C" __global__ void computeArtiCentroidalMomentumMatrices(
	float* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbIndices,
	const PxU32 maxDofs,
	const PxgArticulation* PX_RESTRICT articulations)
{
	const PxU32 jobIndex = threadIdx.y + blockIdx.x * blockDim.y;
	const PxU32 threadIndex = threadIdx.x;

	if(jobIndex < nbIndices)
	{
		const PxU32 artiIndex = gpuIndices[jobIndex];
		const PxgArticulation& arti = articulations[artiIndex];

		const PxU32 dofCount = arti.data.numJointDofs;
		const bool fixBase = arti.data.flags & PxArticulationFlag::eFIX_BASE;

		if(!fixBase)
		{
			const PxU32 startCoriolisForces = (maxDofs + 6) * (maxDofs + 6) * nbIndices;
			const PxU32 startCentroidalMomentumMatrices = startCoriolisForces + (maxDofs + 6) * nbIndices;
			const PxU32 startCentroidalMomentumBias = startCentroidalMomentumMatrices + 6 * (maxDofs + 6) * nbIndices;

			float* PX_RESTRICT massMatrices = &data[jobIndex * (maxDofs + 6) * (maxDofs + 6)];
			float* PX_RESTRICT coriolisForces = &data[startCoriolisForces + jobIndex * (maxDofs + 6)];
			float* PX_RESTRICT centroidalMomentumMatrix = &data[startCentroidalMomentumMatrices + jobIndex * 6 * (maxDofs + 6)];
			float* PX_RESTRICT centroidalMomentumBias = &data[startCentroidalMomentumBias + jobIndex * 6];

			// adding mass matrix terms corresponding to the root DoFs
			for(PxU32 j = threadIndex; j < 6 * (dofCount + 6); j += WARP_SIZE)
			{
				centroidalMomentumMatrix[j] = massMatrices[j];
			}

			// calculating the COM momentum from the spatial articulated inertia of the root in the mass matrix
			const PxReal totalMass = massMatrices[2 * (dofCount + 6) + 2];
			const PxVec3 pg = PxVec3(massMatrices[2 * (dofCount + 6) + 4], massMatrices[5], massMatrices[(dofCount + 6) + 3]) / totalMass;
			const PxMat33 mat(PxVec3(0, pg.z, -pg.y), PxVec3(-pg.z, 0, pg.x), PxVec3(pg.y, -pg.x, 0));

			// adding remaining contributions
			for(PxU32 col = threadIndex; col < dofCount + 6; col += WARP_SIZE)
			{
				const PxVec3 linMassMatrices = PxVec3(massMatrices[0 * (dofCount + 6) + col], massMatrices[1 * (dofCount + 6) + col],
												massMatrices[2 * (dofCount + 6) + col]);
				const PxVec3 crossTermsCentroidalMomentumMatrix = mat * linMassMatrices;
				for(PxU32 row = 3; row < 6; ++row)
				{
					centroidalMomentumMatrix[col + row * (dofCount + 6)] += crossTermsCentroidalMomentumMatrix[row - 3];
				}
			}

			if(threadIndex == 0)
			{
				// adding coriolis force terms corresponding to the root DoFs
				for(PxU32 row = 0; row < 6; ++row)
				{
					centroidalMomentumBias[row] = coriolisForces[row];
				}

				// adding remaining contributions
				const PxVec3 linCoriolisForces = PxVec3(coriolisForces[0], coriolisForces[1], coriolisForces[2]);
				const PxVec3 crossTermsCentroidalMomentumBias = mat * linCoriolisForces;
				for(PxU32 row = 3; row < 6; ++row)
				{
					centroidalMomentumBias[row] += crossTermsCentroidalMomentumBias[row - 3];
				}
			}
		}
	}
}
