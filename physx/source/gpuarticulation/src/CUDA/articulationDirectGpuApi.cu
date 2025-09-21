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

#include "PxDirectGPUAPI.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMath.h"

#include "PxArticulationFlag.h"
#include "PxArticulationTendonData.h"
#include "PxgArticulation.h"
#include "PxgArticulationLink.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgShapeSim.h"
#include "PxsTransformCache.h"


#include "DyArticulationCore.h"
#include "DyArticulationJointCore.h"
#include "DyFeatherstoneArticulation.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "DyFeatherstoneArticulationJointData.h"

#include "CmSpatialVector.h"

#include "utils.cuh"
#include "reduction.cuh"
#include "updateCacheAndBound.cuh"

#include <assert.h>

using namespace physx;
using namespace Dy;

extern "C" __host__ void initArticulationKernels3() {}

template<bool zeroSimOutput>
static PX_FORCE_INLINE __device__ void updateKinematicInternal(
	const PxgArticulation& articulation,
	const PxU32 threadIndexInWarp,
	const PxgShapeSim* PX_RESTRICT gShapeSimPool,
	const PxgShape* PX_RESTRICT gConvexShapes,
	PxsCachedTransform* PX_RESTRICT gTransformCache,
	const PxNodeIndex* PX_RESTRICT gRigidNodeIndices,//sorted rigid body node index 
	const PxU32* PX_RESTRICT gShapeIndices,//the corresponding shape index
	const PxU32 numShapes,
	PxBounds3* PX_RESTRICT bounds)
{
	const PxgArticulationData artiData = articulation.data;
	const PxU32 linkCount = artiData.numLinks;

	const PxU32 bodySimIndex = artiData.bodySimIndex;

	PxQuat* tempNewParentToChilds = articulation.tempParentToChilds;
	PxVec3* tempRs = articulation.tempRs;

	PxTransform* body2Worlds = articulation.linkBody2Worlds;
	ArticulationJointCore* joints = articulation.joints;
	ArticulationJointCoreData* jointData = articulation.jointData;

	const PxQuat* const PX_RESTRICT relativeQuats = articulation.relativeQuat;
	const PxU32* const PX_RESTRICT parents = articulation.parents;

	const SpatialSubspaceMatrix* const PX_RESTRICT motionMatrix = articulation.motionMatrix;

	//each thread deals with a joint
	for (PxU32 linkID = 1 + threadIndexInWarp; linkID < linkCount; linkID += WARP_SIZE)
	{
		const ArticulationJointCoreData& jointDatum = jointData[linkID];

		const ArticulationJointCore& joint = joints[linkID];

		const PxReal* jPosition = &articulation.jointPositions[jointDatum.jointOffset];

		PxQuat& newParentToChild = tempNewParentToChilds[linkID];
		PxVec3& r = tempRs[linkID];

		const PxVec3 childOffset = -joint.childPose.p;
		const PxVec3 parentOffset = joint.parentPose.p;

		const PxQuat relativeQuat = relativeQuats[linkID];

		PxVec3 e, d;

		switch (joint.jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		{
			newParentToChild = relativeQuat;
			const PxVec3& u = motionMatrix[linkID][0].bottom;
			const PxVec3 e = newParentToChild.rotate(parentOffset);
			const PxVec3 d = childOffset;

			r = e + d + u * jPosition[0];

			break;
		}
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			const PxVec3& u = motionMatrix[linkID][0].top;

			PxQuat jointRotation = PxQuat(-jPosition[0], u);
			if (jointRotation.w < 0)	//shortest angle.
				jointRotation = -jointRotation;

			/*printf("LinkID %i: jointRotation = (%f, %f, %f, %f), joint->relativeQuat = (%f, %f, %f,%f)\n", linkID, jointRotation.x, jointRotation.y,
				jointRotation.z, jointRotation.w, joint.relativeQuat.x, joint.relativeQuat.y, joint.relativeQuat.z, joint.relativeQuat.w);*/

			newParentToChild = (jointRotation * relativeQuat).getNormalized();

			const PxVec3 e = newParentToChild.rotate(parentOffset);
			const PxVec3 d = childOffset;
			r = e + d;

			assert(r.isFinite());

			break;
		}
		case PxArticulationJointType::eSPHERICAL:
		{
			PxQuat jointRotation(PxIdentity);

			PxVec3 ang(0.f);
			for (PxU32 d = 0; d < jointDatum.nbDof; ++d)
			{
				ang += motionMatrix[linkID][d].top * -jPosition[d];
			}
			PxReal angle = ang.normalize();

			jointRotation = angle < 1e-10f ? PxQuat(PxIdentity) : PxQuat(angle, ang);
			if(jointRotation.w < 0.f)
				jointRotation = -jointRotation;

			newParentToChild = (jointRotation * relativeQuat).getNormalized();

			const PxVec3 e = newParentToChild.rotate(parentOffset);
			const PxVec3 d = childOffset;
			r = e + d;
			break;
		}
		case PxArticulationJointType::eFIX:
		{
			//this is fix joint so joint don't have velocity
			newParentToChild = relativeQuat;

			const PxVec3 e = newParentToChild.rotate(parentOffset);
			const PxVec3 d = childOffset;

			r = e + d;
			break;
		}
		default:
			break;
		}
	}

	__syncwarp();

	if (threadIndexInWarp == 0)
	{
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const PxU32 parent = parents[linkID];

			const PxTransform pBody2World = body2Worlds[parent];

			PxQuat& newParentToChild = tempNewParentToChilds[linkID];
			PxVec3& r = tempRs[linkID];

			PxTransform& body2World = body2Worlds[linkID];

			body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			body2World.p = pBody2World.p + body2World.q.rotate(r);

            // we do NOT calculate updated link velocities if this is the automatic pre-sim updateKinematic call, because we will immediately do it again because we also need to clamp the joint velocities.
            // if this was a velocity-only update we already skip in the parent function, so we only end up here if positions are dirty.
            if (zeroSimOutput)
            {
                // link velocity update - unfortunately also dependent on parent position and velocity.
                Cm::UnAlignedSpatialVector parentVel = articulation.motionVelocities[parent];
                const PxVec3 c2p = body2World.p - pBody2World.p;
                Cm::UnAlignedSpatialVector linkVelocity = FeatherstoneArticulation::translateSpatialVector(-c2p, parentVel);

                // AD unfortunately this is more-or-less the same code as in computeLinkVelocities, minus the maxJointVel clamping.
                const ArticulationJointCoreData& jointDatum = jointData[linkID];
                const PxReal* jVelocity = &articulation.jointVelocities[jointDatum.jointOffset];
                for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
                {
                    const Cm::UnAlignedSpatialVector worldCol = motionMatrix[linkID][ind].rotate(body2World);
                    const PxReal jVel = jVelocity[ind];
                    linkVelocity += worldCol * jVel;
                }

                articulation.motionVelocities[linkID] = linkVelocity;
            }
		}
	}

	__syncwarp();

    if (zeroSimOutput) // AD: we could potentially ingest this into the link traversal++, but let's not get ahead of ourselves.
    {
        const PxU32 numLinks = articulation.data.numLinks;
        const PxU32 numDofs = articulation.data.numJointDofs;
        PxReal* PX_RESTRICT linkAccelData = reinterpret_cast<PxReal*>(articulation.motionAccelerations);
        PxReal* PX_RESTRICT linkIncomingJointForceData = reinterpret_cast<PxReal*>(articulation.linkIncomingJointForces);

        const PxU32 numRealsForSpatialVector = sizeof(Cm::UnAlignedSpatialVector) / 4;
		const PxU32 maxLinksReal = articulation.data.numLinks * numRealsForSpatialVector;
        const PxU32 linkCountReal = numLinks * numRealsForSpatialVector;

        for (PxU32 index = threadIndexInWarp; index < PxMax(numDofs, maxLinksReal); index += WARP_SIZE)
        {
            if (index < linkCountReal)
            {
                linkAccelData[index] = 0.f;
                linkIncomingJointForceData[index] = 0.f;
            }

            if (index < numDofs)
            {
                articulation.jointAccelerations[index] = 0.f;
            }
        }
    }

	if (numShapes == 0)
	{
		// guard against no actors have shapes in scene, in which case gRigidNodeIndices, etc. would have zero elements
		return;
	}

    __syncwarp();

	const PxTransform* PX_RESTRICT linkBody2Actors = articulation.linkBody2Actors;

	//each thread deals with a link
	for (PxU32 linkID = threadIndexInWarp; linkID < linkCount; linkID += WARP_SIZE)
	{
		const PxNodeIndex linkNodeIndex(bodySimIndex, linkID);
		const PxTransform body2World = body2Worlds[linkID];
		const PxTransform body2Actor = linkBody2Actors[linkID];

		//this will search for the first pos for the matched node index
		PxU32 pos = binarySearch<PxNodeIndex>(gRigidNodeIndices, numShapes, linkNodeIndex);

		// go backward through the sorted rigid node index array which has an entry for each
		// shape of the link, and update the shape if it belongs to the link, i.e. the rigid
		// node index matches the link node index
		while (pos != 0xFFffFFff && gRigidNodeIndices[pos] == linkNodeIndex)
		{
			const PxU32 shapeIndex = gShapeIndices[pos];
			if (shapeIndex != 0xFFffFFff)
			{
				const PxgShapeSim& shapeSim = gShapeSimPool[shapeIndex];

				const PxTransform absPos = getAbsPose(body2World, shapeSim.mTransform, body2Actor);

				//update broad phase bound, transform cache
				updateCacheAndBound(absPos, shapeSim, shapeIndex, gTransformCache, bounds, gConvexShapes, true);
			}
			pos--;
		}
	}
}

//This function is called after user update gpu buffer(Dy::ArticulationDirtyFlag::eDIRTY_ROOT || Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
extern "C" __global__ void artiUpdateKinematic(
	const PxgArticulationCoreDesc* const PX_RESTRICT scDesc,
	const PxgShapeSim* PX_RESTRICT gShapeSimPool,
	const PxgShape* PX_RESTRICT gConvexShapes,
	PxsCachedTransform* PX_RESTRICT gTransformCache,
	const PxNodeIndex* PX_RESTRICT gRigidNodeIndices,
	const PxU32* PX_RESTRICT gShapeIndices,
	const PxU32 numShapes,
	PxBounds3* PX_RESTRICT bounds,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,	// NULL: process all the dirty articulations
	const PxU32 nbElements,						// can be 0 in combination with NULL index buffer.
    bool zeroSimOutput)
{
	// we launch blocks of 32x2 threads. 1 warp deals with 1 articulation.
	assert(blockDim.x == 32);
	assert(blockDim.y == 2);

	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;
	const PxU32 warpsPerGrid = gridDim.x * blockDim.y;

	const PxU32 nbArticulations = gpuIndices ? nbElements : scDesc->nbArticulations;

	for (PxU32 i = globalWarpIndex; i < nbArticulations; i += warpsPerGrid)
	{
		const PxU32 articulationIndex = gpuIndices ? gpuIndices[i] : i;

		PxgArticulation& articulation = scDesc->articulations[articulationIndex];
		if (articulation.data.gpuDirtyFlag & ArticulationDirtyFlag::eNEEDS_KINEMATIC_UPDATE)
		{
			// reset while in cache.
			articulation.data.gpuDirtyFlag &= ~(ArticulationDirtyFlag::eNEEDS_KINEMATIC_UPDATE);

            // if we only have dirty velocities and this is the pre-sim automatic call, we can skip that articulation.
            // The link velocities will be updated in computeLinkVelocities anyway.
            PxU32 mask = (ArticulationDirtyFlag::eDIRTY_ROOT_TRANSFORM | ArticulationDirtyFlag::eDIRTY_POSITIONS);
            bool positionsDirty = articulation.data.gpuDirtyFlag & mask;
            
            if (!zeroSimOutput && !positionsDirty)
                continue;

            if (zeroSimOutput)
            {
                updateKinematicInternal<true>(articulation, threadIndexInWarp, gShapeSimPool, gConvexShapes,
                    gTransformCache, gRigidNodeIndices, gShapeIndices, numShapes, bounds);
            }
            else
            {
                updateKinematicInternal<false>(articulation, threadIndexInWarp, gShapeSimPool, gConvexShapes,
                    gTransformCache, gRigidNodeIndices, gShapeIndices, numShapes, bounds);
            }
		}
	}
}

extern "C" __global__ void getArtiDofStates(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxReal* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT index,
	const PxU32 nbElements,
	const PxU32 maxDofs,
	PxArticulationGPUAPIReadType::Enum type
)
{
	// 1 thread - 1 dof.
	// input has maxDofs * PxReal * nbArticulations size.

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxDofs;
	const PxU32 dofIndex = globalThreadIndex % maxDofs;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = index[groupIndex];
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		PxReal* PX_RESTRICT dstData = &data[groupIndex * maxDofs];
		const PxReal* PX_RESTRICT srcData;

		switch(type)
		{
			case PxArticulationGPUAPIReadType::eJOINT_POSITION:
			{
				srcData = articulation.jointPositions;
				break;
			}
			case PxArticulationGPUAPIReadType::eJOINT_VELOCITY:
			{
				srcData = articulation.jointVelocities;
				break;
			}
			case PxArticulationGPUAPIReadType::eJOINT_ACCELERATION:
			{
				srcData = articulation.jointAccelerations;
				break;
			}
			case PxArticulationGPUAPIReadType::eJOINT_FORCE:
			{
				srcData = articulation.jointForce;
				break;
			}
			case PxArticulationGPUAPIReadType::eJOINT_TARGET_VELOCITY:
			{
				srcData = articulation.jointTargetVelocities;
				break;
			}
			case PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION:
			{
				srcData = articulation.jointTargetPositions;
				break;
			}
			default:
				assert(0);
		}

		const PxU32 artiDofs = articulation.data.numJointDofs;
		if (dofIndex < artiDofs)
		{
			dstData[dofIndex] = srcData[dofIndex];
		}
	}
}

extern "C" __global__ void getArtiTransformStates(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxTransform* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT index,
	const PxU32 nbElements,
	const PxU32 maxLinks,
	PxArticulationGPUAPIReadType::Enum type
)
{
	// 1 thread - 1 link, 1 transform
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxLinks;
	const PxU32 linkIndex = globalThreadIndex % maxLinks;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = index[groupIndex];
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		PxU32 numLinks = (type == PxArticulationGPUAPIReadType::eROOT_GLOBAL_POSE) ? 1 : articulation.data.numLinks;

		PxTransform* dstData = &data[groupIndex * maxLinks];

		if (linkIndex < numLinks)
		{
			const PxTransform body2Actor = articulation.linkBody2Actors[linkIndex];
			const PxTransform body2World = articulation.linkBody2Worlds[linkIndex];
			dstData[linkIndex] = body2World * body2Actor.getInverse();
		}
	}
}

extern "C" __global__ void getArtiVelocityStates(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxVec3* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT index,
	const PxU32 nbElements,
	const PxU32 maxLinks,
	PxArticulationGPUAPIReadType::Enum type
)
{
	// 1 thread - 1 vec3 element.
	const PxU32 threadPerGroup = 3u;
	const PxU32 threadPerArticulation = threadPerGroup * maxLinks;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / threadPerArticulation;

	const PxU32 localIndex = globalThreadIndex % threadPerArticulation;
	const PxU32 linkIndex = localIndex / threadPerGroup;
	const PxU32 elementIndex = localIndex % threadPerGroup;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = index[groupIndex];
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		PxU32 numLinks;
		const PxReal* PX_RESTRICT srcData;

		switch (type)
		{
			case PxArticulationGPUAPIReadType::eROOT_LINEAR_VELOCITY:
			{
				srcData = reinterpret_cast<const PxReal*>(articulation.motionVelocities) + 3;
				numLinks = 1;
				break;
			}
			case PxArticulationGPUAPIReadType::eROOT_ANGULAR_VELOCITY:
			{
				srcData = reinterpret_cast<const PxReal*>(articulation.motionVelocities);
				numLinks = 1;
				break;
			}
			case PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY:
			{
				srcData = reinterpret_cast<const PxReal*>(articulation.motionVelocities) + 3;
				numLinks = articulation.data.numLinks;
				break;
			}
			case PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY:
			{
				srcData = reinterpret_cast<const PxReal*>(articulation.motionVelocities);
				numLinks = articulation.data.numLinks;
				break;
			}
			case PxArticulationGPUAPIReadType::eLINK_LINEAR_ACCELERATION:
			{
				srcData = reinterpret_cast<const PxReal*>(articulation.motionAccelerations) + 3;
				numLinks = articulation.data.numLinks;
				break;
			}
			case PxArticulationGPUAPIReadType::eLINK_ANGULAR_ACCELERATION:
			{
				srcData = reinterpret_cast<const PxReal*>(articulation.motionAccelerations);
				numLinks = articulation.data.numLinks;
				break;
			}
			default:
				assert(0);
		}

		if (linkIndex < numLinks)
		{
			PxReal* PX_RESTRICT dstData = reinterpret_cast<PxReal*>(&data[groupIndex * maxLinks + linkIndex]);
			const PxReal* PX_RESTRICT srcDataU = &srcData[linkIndex * threadPerGroup * 2]; // careful because source is Cm::UnAlignedSpatialVector

			if (elementIndex < threadPerGroup)
			{
				dstData[elementIndex] = srcDataU[elementIndex];
			}
		}
	}
}

extern "C" __global__ void getArtiSpatialForceStates(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxReal* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements,
	const PxU32 maxLinks
)
{
	// 1 thread - 1 element of Cm::UnAlignedSpatialVector.

	// AD sizeof is probably evaluated at compile time, so just use sizeof for the constant?
	PX_COMPILE_TIME_ASSERT((sizeof(Cm::UnAlignedSpatialVector) / 4) == 6u);

	//we need 6 threads for the velocities - for each link
	const PxU32 threadPerGroup = 6u;
	const PxU32 threadPerArticulation = threadPerGroup * maxLinks;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / threadPerArticulation;

	const PxU32 localIndex = globalThreadIndex % threadPerArticulation;
	const PxU32 linkIndex = localIndex / threadPerGroup;
	const PxU32 elementIndex = localIndex % threadPerGroup;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		const Cm::UnAlignedSpatialVector* PX_RESTRICT srcData = articulation.linkIncomingJointForces;
		const PxU32 numLinks = articulation.data.numLinks;

		if (linkIndex < numLinks)
		{
			// 6 PxReal per link
			PxReal* PX_RESTRICT dstData = reinterpret_cast<PxReal*>(&data[groupIndex * threadPerArticulation + linkIndex * threadPerGroup]);
			const PxReal* PX_RESTRICT srcDataU = reinterpret_cast<const PxReal*>(&srcData[linkIndex]);

			if (elementIndex < 6u)
			{
				dstData[elementIndex] = srcDataU[elementIndex];
			}
		}
	}
}

extern "C" __global__ void setArtiDofStates(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const PxReal* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements,
	const PxU32 maxDofs,
	PxArticulationGPUAPIWriteType::Enum type
)
{
	// 1 thread - 1 dof.
	// input has maxDofs * PxReal * nbElements size.

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxDofs;
	const PxU32 dofIndex = globalThreadIndex % maxDofs;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		PxReal* PX_RESTRICT dstData;

		switch (type)
		{
			case (PxArticulationGPUAPIWriteType::eJOINT_POSITION):
			{
				dstData = articulation.jointPositions;
				if (dofIndex == 0)
					articulation.data.gpuDirtyFlag |= (Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS | Dy::ArticulationDirtyFlag::eNEEDS_KINEMATIC_UPDATE);
				break;
			}
			case (PxArticulationGPUAPIWriteType::eJOINT_VELOCITY):
			{
				dstData = articulation.jointVelocities;
				if (dofIndex == 0)
					articulation.data.gpuDirtyFlag |= (Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES | Dy::ArticulationDirtyFlag::eNEEDS_KINEMATIC_UPDATE);
				break;
			}
			case (PxArticulationGPUAPIWriteType::eJOINT_FORCE):
			{
				dstData = articulation.jointForce;
				if (dofIndex == 0)
					articulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_FORCES; 
				break;
			}
			case (PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION):
			{
				dstData = articulation.jointTargetPositions;
				if (dofIndex == 0)
					articulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS;
				break;
			}
			case (PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY):
			{
				dstData = articulation.jointTargetVelocities;
				if (dofIndex == 0)
					articulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL;
				break;
			}
			default:
				assert(false);
		}
		
		const PxU32 artiDofs = articulation.data.numJointDofs;
		
		const PxReal* PX_RESTRICT srcData = &data[groupIndex * maxDofs];

		if (dofIndex < artiDofs)
		{
			dstData[dofIndex] = srcData[dofIndex];
		}
	}
}

extern "C" __global__ void setArtiRootGlobalPoseState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const PxTransform* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT index,
	const PxU32 nbElements
)
{
	// 1 thread - 1 transform value.
	// input has PxTransform * nbArticulations size.

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	// currently 1 thread does the new calc.
	if (globalThreadIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = index[globalThreadIndex];
		PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		articulation.data.gpuDirtyFlag |= (Dy::ArticulationDirtyFlag::eDIRTY_ROOT_TRANSFORM | Dy::ArticulationDirtyFlag::eNEEDS_KINEMATIC_UPDATE);
		
		const PxTransform actorPose = data[globalThreadIndex];
		const PxTransform body2Actor = articulation.linkBody2Actors[0]; // TODO AD: this could be outdated - how do we resolve this?

		const PxTransform pose = actorPose * body2Actor;

		articulation.linkBody2Worlds[0] = pose;
	}
}

extern "C" __global__ void setArtiRootVelocityState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const PxVec3* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT index,
	const PxU32 nbElements,
	const PxArticulationGPUAPIWriteType::Enum operation
)
{
	// 1 thread - 1 vec3 element.
	const PxU32 threadPerArticulation = 3u;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 artiIndex = globalThreadIndex / threadPerArticulation;

	const PxU32 localIndex = globalThreadIndex % threadPerArticulation;

	if (artiIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = index[artiIndex];
		PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		if(localIndex == 0)
			articulation.data.gpuDirtyFlag |= (Dy::ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES | Dy::ArticulationDirtyFlag::eNEEDS_KINEMATIC_UPDATE);

		PxReal* dstData;

		switch (operation)
		{
			case PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY:
			{
				dstData = reinterpret_cast<PxReal*>(articulation.motionVelocities) + 3;
				break;
			}
			case PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY:
			{
				dstData = reinterpret_cast<PxReal*>(articulation.motionVelocities);
				break;
			}
			default:
				assert(0);
		}
		
		const PxReal* PX_RESTRICT srcData = reinterpret_cast<const PxReal*>(&data[artiIndex]);

		if (localIndex < threadPerArticulation)
		{
			dstData[localIndex] = srcData[localIndex];
		}
	}
}

extern "C" __global__ void setArtiLinkForceState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const PxVec3* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT index,
	const PxU32 nbElements,
	const PxU32 maxLinks
)
{
	// 1 thread - 1 float element
	const PxU32 threadPerGroup = 3u; // PxVec3
	const PxU32 threadPerArticulation = threadPerGroup * maxLinks;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 artiSourceIndex = globalThreadIndex / threadPerArticulation;

	const PxU32 localIndex = globalThreadIndex % threadPerArticulation;
	const PxU32 linkIndex = localIndex / threadPerGroup;
	const PxU32 elementIndex = localIndex % threadPerGroup;

	if (artiSourceIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = index[artiSourceIndex];
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		const PxgArticulationLinkProp* const PX_RESTRICT linkProps = articulation.linkProps;
		Cm::UnAlignedSpatialVector* externalAccel = articulation.externalAccelerations;

		const PxU32 artiNumLinks = articulation.data.numLinks;
		const PxVec3* PX_RESTRICT srcData = &data[artiSourceIndex * maxLinks];

		if (linkIndex < artiNumLinks)
		{
			if (elementIndex < threadPerGroup)
			{
				const float4 invInertiaXYZ_invMass = linkProps[linkIndex].invInertiaXYZ_invMass;

				const PxReal* forces = reinterpret_cast<const PxReal*>(&srcData[linkIndex]);
				PxReal* dst = reinterpret_cast<PxReal*>(&externalAccel[linkIndex]);

				dst[elementIndex] = forces[elementIndex] * invInertiaXYZ_invMass.w;
			}
		}
	}
}

extern "C" __global__ void setArtiLinkTorqueState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const PxVec3* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements,
	const PxU32 maxLinks
)
{
	// 1 thread - 1 link.
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxLinks;
	const PxU32 linkIndex = globalThreadIndex % maxLinks;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		const PxgArticulationLinkProp* const PX_RESTRICT linkProps = articulation.linkProps;
		Cm::UnAlignedSpatialVector* externalAccel = articulation.externalAccelerations;
		const PxTransform* PX_RESTRICT body2Worlds = articulation.linkBody2Worlds;

		const PxVec3* PX_RESTRICT srcData = &data[groupIndex * maxLinks];

		const PxU32 artiNumLinks = articulation.data.numLinks;
		if (linkIndex < artiNumLinks)
		{
			const PxQuat& q = body2Worlds[linkIndex].q;
			const PxVec3& linkTorque = srcData[linkIndex];
			const PxVec3 localLinkTorque = q.rotateInv(linkTorque);

			//turn localLinkTorque into acceleration in local frame
			const PxVec3 invInertia = PxLoad3(linkProps[linkIndex].invInertiaXYZ_invMass);
			const PxVec3 localAccel = invInertia.multiply(localLinkTorque);
			//turn the localAccel into world space
			const PxVec3 worldAccel = q.rotate(localAccel);
				
			externalAccel[linkIndex].bottom = PxVec3(worldAccel.x, worldAccel.y, worldAccel.z);
		}
	}
}

// AD: might make sense to split this to more threads!
extern "C" __global__ void setArtiTendonState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const void* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements,
	const PxU32 maxTendons,
	const PxArticulationGPUAPIWriteType::Enum operation
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxTendons;
	const PxU32 tendonIndex = globalThreadIndex % maxTendons;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		switch (operation)
		{
			case (PxArticulationGPUAPIWriteType::eSPATIAL_TENDON):
			{
				const PxU32 numSpatialTendons = articulation.data.numSpatialTendons;
				if (tendonIndex < numSpatialTendons)
				{
					const PxGpuSpatialTendonData* srcData = &(reinterpret_cast<const PxGpuSpatialTendonData*>(data)[groupIndex * maxTendons]); 
					articulation.spatialTendonParams[tendonIndex] = srcData[tendonIndex];
				}
				if (tendonIndex == 0)
					articulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON;
				break;
			}
			case (PxArticulationGPUAPIWriteType::eFIXED_TENDON):
			{
				const PxU32 numFixedTendons = articulation.data.numFixedTendons;
				if (tendonIndex < numFixedTendons)
				{
					const PxGpuFixedTendonData* srcData = &(reinterpret_cast<const PxGpuFixedTendonData*>(data)[groupIndex * maxTendons]);
					articulation.fixedTendonParams[tendonIndex] = srcData[tendonIndex];
				}
				if (tendonIndex == 0)
					articulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON;
				break;
			}
			default:
				assert(false);
		}
	}
}

extern "C" __global__ void getArtiTendonState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	void* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements,
	const PxU32 maxTendons,
	const PxArticulationGPUAPIReadType::Enum operation
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxTendons;
	const PxU32 tendonIndex = globalThreadIndex % maxTendons;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		switch (operation)
		{
			case (PxArticulationGPUAPIReadType::eSPATIAL_TENDON):
			{
				const PxU32 numSpatialTendons = articulation.data.numSpatialTendons;
				if (tendonIndex < numSpatialTendons)
				{
					PxGpuSpatialTendonData* dstData = &(reinterpret_cast<PxGpuSpatialTendonData*>(data)[groupIndex * maxTendons]);
					dstData[tendonIndex] = articulation.spatialTendonParams[tendonIndex];
				}
				break;
			}
			case (PxArticulationGPUAPIReadType::eFIXED_TENDON):
			{
				const PxU32 numFixedTendons = articulation.data.numFixedTendons;
				if (tendonIndex < numFixedTendons)
				{
					PxGpuFixedTendonData* dstData = &(reinterpret_cast<PxGpuFixedTendonData*>(data)[groupIndex * maxTendons]);
					dstData[tendonIndex] = articulation.fixedTendonParams[tendonIndex];
				}
				break;
			}
			default:
				assert(false);
		}
	}
}

extern "C" __global__ void setArtiSpatialTendonAttachmentState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const PxGpuTendonAttachmentData* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements,
	const PxU32 maxTendons
)
{
	const PxU32 maxTendonAttachments = scDesc->mMaxAttachmentPerArticulation;
	const PxU32 maxSpatialTendons = scDesc->mMaxSpatialTendonsPerArticulation;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxTendons;
	const PxU32 elementIndex = globalThreadIndex % maxTendons;
	const PxU32 tendonIndex = elementIndex / maxTendonAttachments;
	const PxU32 attachmentIndex = elementIndex % maxTendonAttachments;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		if (elementIndex == 0)
			articulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT;

		const PxU32 numSpatialTendons = articulation.data.numSpatialTendons;

		if (tendonIndex < numSpatialTendons)
		{
			PxgArticulationTendon& tendon = articulation.spatialTendons[tendonIndex];

			const PxGpuTendonAttachmentData* srcData = &data[groupIndex * maxSpatialTendons * maxTendonAttachments];
			PxGpuTendonAttachmentData* attachData = reinterpret_cast<PxGpuTendonAttachmentData*>(tendon.mModElements);
			const PxU32 numTendonAttachments = tendon.mNbElements;

			if (attachmentIndex < numTendonAttachments)
			{
				//PxGpuTendonAttachmentData is 32 byte
				PX_COMPILE_TIME_ASSERT(sizeof(PxGpuTendonAttachmentData) == 32);
				const PxU32 numIteration = sizeof(PxGpuTendonAttachmentData) / sizeof(uint4);
				const uint4* tData = reinterpret_cast<const uint4*>(&srcData[attachmentIndex]);
				uint4* aData = reinterpret_cast<uint4*>(&attachData[attachmentIndex]);
				for (PxU32 i = 0; i < numIteration; ++i)
				{
					aData[i] = tData[i];
				}
			}
		}
	}
}

extern "C" __global__ void getArtiSpatialTendonAttachmentState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxGpuTendonAttachmentData* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements,
	const PxU32 maxTendons
)
{
	const PxU32 maxTendonAttachments = scDesc->mMaxAttachmentPerArticulation;
	const PxU32 maxSpatialTendons = scDesc->mMaxSpatialTendonsPerArticulation;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxTendons;
	const PxU32 elementIndex = globalThreadIndex % maxTendons;
	const PxU32 tendonIndex = elementIndex / maxTendonAttachments;
	const PxU32 attachmentIndex = elementIndex % maxTendonAttachments;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		const PxU32 numSpatialTendons = articulation.data.numSpatialTendons;

		if (tendonIndex < numSpatialTendons)
		{
			const PxgArticulationTendon& tendon = articulation.spatialTendons[tendonIndex];

			PxGpuTendonAttachmentData* dstData = &data[groupIndex * maxSpatialTendons * maxTendonAttachments];
			const PxGpuTendonAttachmentData* attachData = reinterpret_cast<const PxGpuTendonAttachmentData*>(tendon.mModElements);
			const PxU32 numTendonAttachments = tendon.mNbElements;

			if (attachmentIndex < numTendonAttachments)
			{
				//PxGpuTendonAttachmentData is 32 byte
				PX_COMPILE_TIME_ASSERT(sizeof(PxGpuTendonAttachmentData) == 32);
				const PxU32 numIteration = sizeof(PxGpuTendonAttachmentData) / sizeof(uint4);
				uint4* tData = reinterpret_cast<uint4*>(&dstData[attachmentIndex]);
				const uint4* aData = reinterpret_cast<const uint4*>(&attachData[attachmentIndex]);
				for (PxU32 i = 0; i < numIteration; ++i)
				{
					tData[i] = aData[i];
				}
			}
		}
	}
}

extern "C" __global__ void setArtiFixedTendonJointState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	const PxGpuTendonJointCoefficientData* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements
)
{
	const PxU32 maxFixedTendons = scDesc->mMaxFixedTendonsPerArticulation;
	const PxU32 maxTendonJoints = scDesc->mMaxTendonJointPerArticulation;
	const PxU32 maxTendons = maxFixedTendons * maxTendonJoints;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxTendons;
	const PxU32 elementIndex = globalThreadIndex % maxTendons;
	const PxU32 tendonIndex = elementIndex / maxTendonJoints;
	const PxU32 tendonJointIndex = elementIndex % maxTendonJoints;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		if (elementIndex == 0)
			articulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT;

		PxgArticulationTendon* fixedTendons = articulation.fixedTendons;
		const PxU32 numFixedTendonJoints = articulation.data.numFixedTendons;

		if (tendonIndex < numFixedTendonJoints)
		{
			PxgArticulationTendon& tendon = fixedTendons[tendonIndex];

			const PxGpuTendonJointCoefficientData* srcData = &data[groupIndex * maxFixedTendons * maxTendonJoints];

			PxGpuTendonJointCoefficientData* coefficientData = reinterpret_cast<PxGpuTendonJointCoefficientData*>(tendon.mModElements);
			const PxU32 numTendonJoints = tendon.mNbElements;

			if (tendonJointIndex < numTendonJoints)
			{
				//PxGpuTendonJointCoefficientData is 16 byte
				PX_COMPILE_TIME_ASSERT(sizeof(PxGpuTendonJointCoefficientData) == 16);
				const uint4 tData = reinterpret_cast<const uint4&>(srcData[tendonJointIndex]);
				uint4& coefData = reinterpret_cast<uint4&>(coefficientData[tendonJointIndex]);
				coefData = tData;
			}
		}
	}
}

extern "C" __global__ void getArtiFixedTendonJointState(
	const PxgArticulationCoreDesc* PX_RESTRICT scDesc,
	PxGpuTendonJointCoefficientData* PX_RESTRICT data,
	const PxArticulationGPUIndex* PX_RESTRICT gpuIndices,
	const PxU32 nbElements
)
{
	const PxU32 maxFixedTendons = scDesc->mMaxFixedTendonsPerArticulation;
	const PxU32 maxTendonJoints = scDesc->mMaxTendonJointPerArticulation;
	const PxU32 maxTendons = maxFixedTendons * maxTendonJoints;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 groupIndex = globalThreadIndex / maxTendons;
	const PxU32 elementIndex = globalThreadIndex % maxTendons;
	const PxU32 tendonIndex = elementIndex / maxTendonJoints;
	const PxU32 tendonJointIndex = elementIndex % maxTendonJoints;

	if (groupIndex < nbElements)
	{
		const PxArticulationGPUIndex articulationIndex = gpuIndices[groupIndex];
		const PxgArticulation& articulation = scDesc->articulations[articulationIndex];

		const PxgArticulationTendon* fixedTendons = articulation.fixedTendons;
		const PxU32 numFixedTendonJoints = articulation.data.numFixedTendons;

		if (tendonIndex < numFixedTendonJoints)
		{
			const PxgArticulationTendon& tendon = fixedTendons[tendonIndex];

			PxGpuTendonJointCoefficientData* dstData = &data[groupIndex * maxFixedTendons * maxTendonJoints];

			const PxGpuTendonJointCoefficientData* coefficientData = reinterpret_cast<const PxGpuTendonJointCoefficientData*>(tendon.mModElements);
			const PxU32 numTendonJoints = tendon.mNbElements;

			if (tendonJointIndex < numTendonJoints)
			{
				//PxGpuTendonJointCoefficientData is 16 byte
				PX_COMPILE_TIME_ASSERT(sizeof(PxGpuTendonJointCoefficientData) == 16);
				uint4& tData = reinterpret_cast<uint4&>(dstData[tendonJointIndex]);
				const uint4& coefData = reinterpret_cast<const uint4&>(coefficientData[tendonJointIndex]);
				tData = coefData;
			}
		}
	}
}
