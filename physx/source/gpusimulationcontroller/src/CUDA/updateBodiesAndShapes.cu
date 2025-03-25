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
#include "PxNodeIndex.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgShapeSim.h"
#include "PxgBodySim.h"
#include "PxsRigidBody.h"
#include "PxgD6JointData.h"
#include "PxgCommonDefines.h"
#include "PxgConstraintPrep.h"
#include "PxgArticulation.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgSoftBody.h"
#include "DyArticulationJointCore.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "PxgSimulationCoreKernelIndices.h"
#include "CmUtils.h"
#include "copy.cuh"
#include "DyFeatherstoneArticulation.h"
#include "PxArticulationTendonData.h"
#include "updateCacheAndBound.cuh"
#include "PxgMemCopyDispatcher.h"
#include "assert.h"
#include "CmSpatialVector.h"
#include "utils.cuh"
#include "reduction.cuh"
#include "PxgConstraintWriteBack.h"
#include "PxgConstraintIdMap.h"

using namespace physx;

extern "C" __host__ void initSimulationControllerKernels0() {}

static const PxU32 PXG_BODY_SIM_UINT4_SIZE = sizeof(PxgBodySim) / sizeof(uint4);
static const PxU32 PXG_BODY_SIM_MAX_LIN_VEL_IND = offsetof(PxgBodySim, maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW) / sizeof(uint4);
static const PxU32 PXG_BODY_SIM_BODYSIM_INDEX_IND = offsetof(PxgBodySim, freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex) / sizeof(uint4);
static const PxU32 PXG_BODY_SIM_BODY2WORLD_IND = offsetof(PxgBodySim, body2World) / sizeof(uint4);
static const PxU32 PXG_BODY_SIM_FLAGS_IND = offsetof(PxgBodySim, articulationRemapId) / sizeof(uint4);
static const PxU32 PXG_BODY_SIM_BODY2ACTOR_IND = offsetof(PxgBodySim, body2Actor_maxImpulseW) / sizeof(uint4);
static const PxU32 PXG_BODY_SIM_SIZE_WITHOUT_ACCELERATION = offsetof(PxgBodySim, externalLinearAcceleration) / sizeof(uint4);

// assert to ensure proper updateBodiesLaunch/updateBodiesLaunchDirectAPI
PX_COMPILE_TIME_ASSERT((sizeof(PxgBodySim) < 16 * sizeof(uint4)));
PX_COMPILE_TIME_ASSERT((offsetof(PxgBodySim, linearVelocityXYZ_inverseMassW) == 0));
PX_COMPILE_TIME_ASSERT((offsetof(PxgBodySim, angularVelocityXYZ_maxPenBiasW) == sizeof(float4)));
PX_COMPILE_TIME_ASSERT((offsetof(PxgBodySim, maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW) == (2 * sizeof(float4))));
PX_COMPILE_TIME_ASSERT((offsetof(PxgBodySim, articulationRemapId) % sizeof(uint4) == 0));
PX_COMPILE_TIME_ASSERT(((offsetof(PxgBodySim, internalFlags) - offsetof(PxgBodySim, articulationRemapId)) == sizeof(PxU32)));

extern "C" __global__ void updateBodiesLaunch(const PxgNewBodiesDesc* scDesc)
{
	const uint4* gBodySim = reinterpret_cast<const uint4*>(scDesc->mNewBodySim);
	const PxU32 totalNbBodies = scDesc->mNbNewBodies;

	//persistent data
	uint4* gBodySimPool = reinterpret_cast<uint4*>(scDesc->mBodySimBufferDeviceData);

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	//each PxgBodySim has 224 bytes, so we need to use 16 threads to read one body. In fact,
	//we just need to use 14 threads(each thread read one 16 bytes), however, we should use 16 threads because shfl is working in a warp
	//for (PxU32 i = idx / 16; i < totalNbBodies; i += (blockDim.x * gridDim.x) / 16)
	PxU32 mask_loop = FULL_MASK;
	for (PxU32 i = idx / 16; (i < totalNbBodies) | ((mask_loop = __ballot_sync(mask_loop, i < totalNbBodies)) & 0); i += (blockDim.x * gridDim.x) / 16)
	{
		const PxU32 index = idx & 15;
		uint4 data;
		if (index < PXG_BODY_SIM_UINT4_SIZE)
			data = gBodySim[i * PXG_BODY_SIM_UINT4_SIZE + index];

		const PxU32 bodyIndex = __shfl_sync(mask_loop, data.w, PXG_BODY_SIM_BODYSIM_INDEX_IND, 16);

		if (index < PXG_BODY_SIM_UINT4_SIZE)
			gBodySimPool[bodyIndex * PXG_BODY_SIM_UINT4_SIZE + index] = data;
	}
}

extern "C" __global__ void updateBodiesLaunchDirectAPI(const PxgNewBodiesDesc* scDesc)
{
	const uint4* gBodySim = reinterpret_cast<const uint4*>(scDesc->mNewBodySim);
	const PxU32 totalNbBodies = scDesc->mNbNewBodies;

	//persistent data
	uint4* gBodySimPool = reinterpret_cast<uint4*>(scDesc->mBodySimBufferDeviceData);

	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;

	//each PxgBodySim has 224 bytes, so we need to use 16 threads to read one body. In fact,
	//we just need to use 14 threads(each thread read one 16 bytes), however, we should use 16 threads because shfl is working in a warp
	//for (PxU32 i = idx / 16; i < totalNbBodies; i += (blockDim.x * gridDim.x) / 16)
	PxU32 mask_loop = FULL_MASK;
	for(PxU32 i = idx / 16; (i < totalNbBodies) | ((mask_loop = __ballot_sync(mask_loop, i < totalNbBodies)) & 0);
	    i += (blockDim.x * gridDim.x) / 16)
	{
		const PxU32 index = idx & 15;
		uint4 data;
		if(index < PXG_BODY_SIM_UINT4_SIZE)
			data = gBodySim[i * PXG_BODY_SIM_UINT4_SIZE + index];

		const PxU32 bodyIndex = __shfl_sync(mask_loop, data.w, PXG_BODY_SIM_BODYSIM_INDEX_IND, 16);
		const PxU32 internalFlags = __shfl_sync(mask_loop, data.y, PXG_BODY_SIM_FLAGS_IND, 16);
		// preist: note that we copy this flag to persistent GPU memory on first transfer, but that is no problem
		// because we only check the update data flag here which will be reset on CPU
		const bool firstTransfer = internalFlags & PxsRigidBody::eFIRST_BODY_COPY_GPU;
		const bool copyVel = internalFlags & PxsRigidBody::eVELOCITY_COPY_GPU;


		// figure out which threads will execute the else below.
		const PxU32 sync_mask = __ballot_sync(mask_loop, (index < PXG_BODY_SIM_SIZE_WITHOUT_ACCELERATION) && !firstTransfer);

		if(index < PXG_BODY_SIM_UINT4_SIZE)
		{
			// on the first DMA, we copy the transform data that the user setup on CPU
			if(firstTransfer)
			{
				gBodySimPool[bodyIndex * PXG_BODY_SIM_UINT4_SIZE + index] = data;
			}
			else if (index < PXG_BODY_SIM_SIZE_WITHOUT_ACCELERATION)
			{
				// AD: all of this complexity just to make sure we can update CMassLocalPose from the CPU.

				// This is far from optimal. We reserve space for 2 PxAlignedTransforms and load them using only 1 thread.
				// and then we do transform math and store them again using only 1 thread as well.
				// The most important part is correctness for the moment, and we need to make sure the syncwarp()s below have
				// the right mask.
				
				// ideally we split the loads & stores to threads and use shared memory to store the intermediate values.
				// But that does not help a lot if we then continue on and do all the transform math with 1 thread - because
				// we still allocate registers for every thread.

				// store the old body2world and body2actor.
				PxAlignedTransform body2World;
				PxAlignedTransform oldBody2Actor;
				if (index == 0)
				{
					body2World = *reinterpret_cast<PxAlignedTransform*>(&gBodySimPool[bodyIndex * PXG_BODY_SIM_UINT4_SIZE + PXG_BODY_SIM_BODY2WORLD_IND]);
					oldBody2Actor = *reinterpret_cast<PxAlignedTransform*>(&gBodySimPool[bodyIndex * PXG_BODY_SIM_UINT4_SIZE + PXG_BODY_SIM_BODY2ACTOR_IND]);
				}

				// load the new stuff no matter what
				// keep linear and angular velocity intact, but copy inverseMass and pen bias:
				if(index < PXG_BODY_SIM_MAX_LIN_VEL_IND && !copyVel)
				{
					gBodySimPool[bodyIndex * PXG_BODY_SIM_UINT4_SIZE + index].w = data.w;
				}
				else
				{
					gBodySimPool[bodyIndex * PXG_BODY_SIM_UINT4_SIZE + index] = data;
				}

				// need to make sure the data is loaded.
				__syncwarp(sync_mask);

				// recalculate body2world with new body2actor
				if (index == 0)
				{
					PxAlignedTransform newBody2Actor = *reinterpret_cast<PxAlignedTransform*>(&gBodySimPool[bodyIndex * PXG_BODY_SIM_UINT4_SIZE + PXG_BODY_SIM_BODY2ACTOR_IND]);

					if (oldBody2Actor != newBody2Actor)
					{
						PxAlignedTransform actor2World = body2World * oldBody2Actor.getInverse();
						body2World = actor2World * newBody2Actor;
					}

					PxAlignedTransform* body2WorldPtr = reinterpret_cast<PxAlignedTransform*>(&gBodySimPool[bodyIndex * PXG_BODY_SIM_UINT4_SIZE + PXG_BODY_SIM_BODY2WORLD_IND]);
					*body2WorldPtr = body2World;
				}

				__syncwarp(sync_mask);
			}
		}
	}
}

extern "C" __global__ void updateShapesLaunch(const PxgNewShapeSim* PX_RESTRICT newShapeSimsBufferDeviceData, PxgShapeSim* PX_RESTRICT shapeSimsBufferDeviceData, PxU32 nbNewShapes)
{
	const PxU32 startIndex = threadIdx.x + blockIdx.x * blockDim.x;

	for (PxU32 idx = startIndex; idx < nbNewShapes; idx += blockDim.x * gridDim.x) 
	{
		const PxgNewShapeSim& src = newShapeSimsBufferDeviceData[idx];
		const PxTransform pose = src.mTransform;
		const PxBounds3 bounds = src.mLocalBounds;
		const PxU32 elementIndex = src.mElementIndex;
		const PxNodeIndex nodeIndex = src.mBodySimIndex;
		const PxU32 hullDataIndex = src.mHullDataIndex;
		const PxU16 shapeFlags = src.mShapeFlags;
		const PxU16 shapeType = src.mShapeType;

		PxgShapeSim& dst = shapeSimsBufferDeviceData[elementIndex];
		dst.mTransform = pose;
		dst.mLocalBounds = bounds;
		dst.mBodySimIndex = nodeIndex;
		dst.mHullDataIndex = hullDataIndex;
		dst.mShapeFlags = shapeFlags;
		dst.mShapeType = shapeType;
	}
}

//one warp to deal with one articulation
extern "C" __global__ void newArticulationsLaunch(const PxgUpdateArticulationDesc* scDesc, const PxReal* dofData)
{

	PxgArticulation* gNewArticulations = scDesc->mNewArticulations;
	PxgArticulationLink* gNewLinks = scDesc->mNewLinks;
	PxgArticulationLinkProp* gNewLinkProps = scDesc->mNewLinkProps;
	PxReal*	gNewLinkWakeCounters = scDesc->mNewLinkWakeCounters;
	Cm::UnAlignedSpatialVector* gNewLinkAccels = scDesc->mNewLinkExtAccels;
	PxU32* gNewLinkParents = scDesc->mNewLinkParents;
	ArticulationBitField* gNewLinkChildren = scDesc->mNewLinkChildren;
	PxTransform* gNewLinkBody2Worlds = scDesc->mNewLinkBody2Worlds;
	PxTransform* gNewLinkBody2Actors = scDesc->mNewLinkBody2Actors;

	Dy::ArticulationJointCore* gNewJointCore = scDesc->mNewJointCores;
	Dy::ArticulationJointCoreData* gNewJointData = scDesc->mNewJointData;

	//const PxU32 warpIndex = threadIdx.y;

	//shared memory shared within a block
	const PxU32 numWarpsPerBlock = PxgSimulationCoreKernelBlockDim::NEW_ARTICULATION / WARP_SIZE;

	__shared__ PxU32 msData[sizeof(PxgArticulation)*numWarpsPerBlock];
	PxgArticulation* msArticulations = reinterpret_cast<PxgArticulation*>(msData);

	//We flatten the new joint/links/tendon/attachment array for all the new articulations. Each new articulation has
	//index to the new joint/links/tendon array.
	PxgArticulationSimUpdate* gLinkIndex = scDesc->mIndicesOffset; 

	PxgArticulation* gArticulations = scDesc->mArticulationPool;

	PxgSolverBodySleepData* gArticulationSleepData = scDesc->mArticulationSleepDataPool;

	const PxU32 nbArticulations = scDesc->mNbNewArticulations;

	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;

	if (globalWarpIndex < nbArticulations)
	{
		PxgArticulation& newArticulation = gNewArticulations[globalWarpIndex];

		PxgArticulation& msArticulation = msArticulations[threadIdx.y];

		uint* sArticulation = reinterpret_cast<uint*>(&newArticulation);

		{
			//copy articulation to shared memory
			uint* dArticulation = reinterpret_cast<uint*>(&msArticulation);
			//each PxgArticulation is 16 bytes aligned
			warpCopy<uint>(dArticulation, sArticulation, sizeof(PxgArticulation));
		}

		__syncwarp();

		PxgArticulationData& msArtiData = msArticulation.data;

		const PxU32 articulationIndex = msArtiData.index;

		PxgArticulation& articulation = gArticulations[articulationIndex];
		{
			//copy articulation to persistent global memory
			uint* dArticulation = reinterpret_cast<uint*>(&articulation);
			warpCopy<uint>(dArticulation, reinterpret_cast<uint*>(&msArticulation), sizeof(PxgArticulation));
		}

		PxgSolverBodySleepData& artiSleepData = gArticulationSleepData[articulationIndex];
		{
			uint* dSleepData = reinterpret_cast<uint*>(&artiSleepData);
			uint zero = 0;
			warpCopy<uint>(dSleepData, zero, sizeof(PxgSolverBodySleepData));
		}

		const PxU32 nbLinks = msArtiData.numLinks;

		//read link and joint index, nbLinks is the same as nbJoints. For baseLink, joint is empty
		const PxgArticulationSimUpdate& update = gLinkIndex[globalWarpIndex];
		const PxU32 startIndex = update.linkStartIndex;
		const PxU32 dofStartIndex = update.dofDataStartIndex;
		const PxU32 spatialTendonStartIndex = update.spatialTendonStartIndex;
		const PxU32 fixedTendonStartIndex = update.fixedTendonStartIndex;

		PxU32 dirtyFlags = update.dirtyFlags;

		uint4* sLinks = reinterpret_cast<uint4*>(&gNewLinks[startIndex]);
		uint4* dLinks = reinterpret_cast<uint4*>(msArticulation.links);

		//each PxgArticulationLink is 76 bytes, each thread read 16 bytes
		warpCopy<uint4>(dLinks, sLinks, (sizeof(PxgArticulationLink) * nbLinks));

		//read wake counter for each link
		PxReal* sLinkWakeCounter = &gNewLinkWakeCounters[startIndex];
		PxReal* dLinkWakeCounter = msArticulation.linkWakeCounters;

		warpCopy<PxReal>(dLinkWakeCounter, sLinkWakeCounter, sizeof(PxReal) * nbLinks);

		//copy lin properties
		uint4* sLinkProp = reinterpret_cast<uint4*>(&gNewLinkProps[startIndex]);
		uint4* dLinkProp = reinterpret_cast<uint4*>(msArticulation.linkProps);
		
		warpCopy<uint4>(dLinkProp, sLinkProp, sizeof(uint4) * nbLinks);

		//read link parent
		uint* sParents = &gNewLinkParents[startIndex];
		uint* dParents = msArticulation.parents;

		warpCopy<uint>(dParents, sParents, sizeof(uint) * nbLinks);

		uint* sChildren = reinterpret_cast<uint*>(&gNewLinkChildren[startIndex]);
		uint* dChildren = reinterpret_cast<uint*>(msArticulation.children);

		warpCopy<uint>(dChildren, sChildren, sizeof(ArticulationBitField)*nbLinks);

		//read link body2World
		uint* sBody2Worlds = reinterpret_cast<uint*>(&gNewLinkBody2Worlds[startIndex]);
		uint* dBody2Worlds = reinterpret_cast<uint*>(msArticulation.linkBody2Worlds);

		//each PxTransform is 28 bytes, each thread read 4 bytes
		warpCopy<uint>(dBody2Worlds, sBody2Worlds, sizeof(PxTransform) * nbLinks);

		//read link body2Actor
		uint* sBody2Actors = reinterpret_cast<uint*>(&gNewLinkBody2Actors[startIndex]);
		uint* dBody2Actors = reinterpret_cast<uint*>(msArticulation.linkBody2Actors);

		//each PxTransform is 28 bytes, each thread read 4 bytes
		warpCopy<uint>(dBody2Actors, sBody2Actors, sizeof(PxTransform) * nbLinks);

		//read joint core, each thread read 16 bytes

		uint4* sJoints = reinterpret_cast<uint4*>(&gNewJointCore[startIndex]);
		uint4* dJoints = reinterpret_cast<uint4*>(msArticulation.joints);

		//each ArticulationJointCore is less than 272 bytes, each thread read 16 bytes
		warpCopy<uint4>(dJoints, sJoints, sizeof(Dy::ArticulationJointCore) * nbLinks);

		//read joint data, each thread read 4 bytes
		uint* sJointData = reinterpret_cast<uint*>(&gNewJointData[startIndex]);
		uint* dJointData = reinterpret_cast<uint*>(msArticulation.jointData);

		//each ArticulationJointCoreData is 20 bytes, each thread read 4 bytes
		warpCopy<uint>(dJointData, sJointData, sizeof(Dy::ArticulationJointCoreData) * nbLinks);

		//copy spatial tendon
		{
			const PxU32 nbSpatialTendons = msArtiData.numSpatialTendons;

			PxGpuSpatialTendonData* spatialTendonParams = scDesc->mNewSpatialTendonParamsPool;
			PxgArticulationTendon* spatialTendons = scDesc->mNewSpatialTendonPool;

			PxgArticulationTendonElementFixedData* fixedData = scDesc->mNewAttachmentFixedPool;
			PxGpuTendonAttachmentData* modData = scDesc->mNewAttachmentModPool;
			PxU32* tendonAttachmentRemap = scDesc->mNewTendonAttachmentRemapPool;

			const PxU32 endIndex = spatialTendonStartIndex + nbSpatialTendons;

			//copy attachment
			for (PxU32 i = spatialTendonStartIndex; i < endIndex; ++i)
			{
				PxgArticulationTendon& tendon = spatialTendons[i];
				const PxU32 attachmentStartIndex = tendonAttachmentRemap[i];

				//each PxgArticulationTendonElementFixedData has 16 bytes, each thread should read 16 bytes
				uint4* sFixedData = reinterpret_cast<uint4*>(&fixedData[attachmentStartIndex]);
				uint4* dFixedData = reinterpret_cast<uint4*>(tendon.mFixedElements);

				//each PxGpuTendonAttachmentData has 32 bytes, each thread should read 16 bytes
				uint4* sModData = reinterpret_cast<uint4*>(&modData[attachmentStartIndex]);
				uint4* dModData = reinterpret_cast<uint4*>(tendon.mModElements);

				warpCopy<uint4>(dFixedData, sFixedData, sizeof(PxgArticulationTendonElementFixedData) * tendon.mNbElements);
				warpCopy<uint4>(dModData, sModData, sizeof(PxGpuTendonAttachmentData) * tendon.mNbElements);
			}

			//__syncwarp();

			//copy tendons
			uint* sTendons = reinterpret_cast<uint*>(&spatialTendons[spatialTendonStartIndex]);
			uint* dTendons = reinterpret_cast<uint*>(msArticulation.spatialTendons);

			uint4* sTendonParams = reinterpret_cast<uint4*>(&spatialTendonParams[spatialTendonStartIndex]);
			uint4* dTendonParams = reinterpret_cast<uint4*>(msArticulation.spatialTendonParams);

			warpCopy<uint>(dTendons, sTendons, sizeof(PxgArticulationTendon) * nbSpatialTendons);
			warpCopy<uint4>(dTendonParams, sTendonParams, sizeof(PxGpuSpatialTendonData) * nbSpatialTendons);
		}

		//copy fixed tendon
		{
			const PxU32 nbFixedTendons = msArtiData.numFixedTendons;

			PxGpuFixedTendonData* fixedTendonParams = scDesc->mNewFixedTendonParamsPool;
			PxgArticulationTendon* fixedTendons = scDesc->mNewFixedTendonPool;

			PxgArticulationTendonElementFixedData* tendonFixed = scDesc->mNewTendonJointFixedPool;
			PxU32* tendonToTendonJointRemap = scDesc->mNewTendonTendonJointRemapPool;

			PxGpuTendonJointCoefficientData* tendonCoefficient = scDesc->mNewTendonJointCoefficientPool;

			const PxU32 endIndex = fixedTendonStartIndex + nbFixedTendons;

			//copy tendon joint
			for (PxU32 i = fixedTendonStartIndex; i < endIndex; ++i)
			{
				PxgArticulationTendon& tendon = fixedTendons[i];
				const PxU32 tendonJointStartIndex = tendonToTendonJointRemap[i];

				//each PxgArticulationTendonElementFixedData has 16 bytes, each thread should read 16 bytes
				uint4* sTendonJoint = reinterpret_cast<uint4*>(&tendonFixed[tendonJointStartIndex]);
				uint4* dTendonJoint = reinterpret_cast<uint4*>(tendon.mFixedElements);

				//each PxGpuTendonJointCoefficientData is 8 bytes, each thread read 8 bytes
				uint2* sTendonJointCoefficient = reinterpret_cast<uint2*>(&tendonCoefficient[tendonJointStartIndex]);
				uint2* dTendonJointCoefficient = reinterpret_cast<uint2*>(tendon.mModElements);

				warpCopy<uint4>(dTendonJoint, sTendonJoint, sizeof(PxgArticulationTendonElementFixedData) * tendon.mNbElements);
				warpCopy<uint2>(dTendonJointCoefficient, sTendonJointCoefficient, sizeof(PxGpuTendonJointCoefficientData) * tendon.mNbElements);	
			}

			//__syncwarp();

			//copy tendons
			uint* sTendons = reinterpret_cast<uint*>(&fixedTendons[fixedTendonStartIndex]);
			uint* dTendons = reinterpret_cast<uint*>(msArticulation.fixedTendons);

			uint4* sTendonParams = reinterpret_cast<uint4*>(&fixedTendonParams[fixedTendonStartIndex]);
			uint4* dTendonParams = reinterpret_cast<uint4*>(msArticulation.fixedTendonParams);

			warpCopy<uint>(dTendons, sTendons, sizeof(PxgArticulationTendon) * nbFixedTendons);
			warpCopy<uint4>(dTendonParams, sTendonParams, sizeof(PxGpuFixedTendonData) * nbFixedTendons);
		}

		warpCopy<uint4>(reinterpret_cast<uint4*>(msArticulation.mimicJointCores), reinterpret_cast<uint4*>(&scDesc->mNewArticulationMimicJointPool[update.mimicJointStartIndex]),
			sizeof(Dy::ArticulationMimicJointCore) * msArtiData.numMimicJoints);

		warpCopy<uint>(reinterpret_cast<uint*>(msArticulation.pathToRoot), reinterpret_cast<uint*>(&scDesc->mNewPathToRootPool[update.pathToRootIndex]),
			sizeof(uint) * msArtiData.numPathToRoots);
		//warpCopy<uint>(reinterpret_cast<>);
		//initialize articulation link data to zero
		uint* c0 = reinterpret_cast<uint*>(msArticulation.motionVelocities);		//Cannot guarantee 8-byte alignment so need to use uint
		uint* c1 = reinterpret_cast<uint*>(msArticulation.motionAccelerations);		//Cannot guarantee 8-byte alignment so need to use uint
		uint2* c2 = reinterpret_cast<uint2*>(msArticulation.corioliseVectors);
		uint2 zero2 = make_uint2(0, 0);

		PxU32 totalSize = (sizeof(Cm::UnAlignedSpatialVector) * nbLinks);

		warpCopy<uint>(c0, 0u, totalSize);
		warpCopy<uint>(c1, 0u, totalSize);
		warpCopy<uint2>(c2, zero2, totalSize);

		//zero the sleepData for each link
		uint4* sleepData = reinterpret_cast<uint4*>(msArticulation.linkSleepData);

		totalSize = sizeof(PxgArticulationLinkSleepData) * nbLinks;
		uint4 zero4 = make_uint4(0, 0, 0, 0);

		warpCopy<uint4>(sleepData, zero4, totalSize);

		//initialize joint data to zero
		const PxU8 dofs = msArticulation.data.numJointDofs;

		totalSize = (sizeof(PxReal) * dofs);

		//KS - these *may* be non-zero on the host. We really need to make sure that these are set up correctly!!!
		float* jv = msArticulation.jointVelocities;
		float* jp = msArticulation.jointPositions;
		float* jf = msArticulation.jointForce;
		float* jtp = msArticulation.jointTargetPositions;
		float* jtv = msArticulation.jointTargetVelocities;

		float zero = 0.f;

		__syncwarp(); 

		//Now we need to see if there was any dof data that required updating!!!
		const PxU32 numDofs = msArticulation.data.numJointDofs;

		PxU32 offset = dofStartIndex;

		if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
		{
			warpCopy<PxReal>(msArticulation.jointPositions, dofData + offset, sizeof(PxReal)*numDofs);
			offset += numDofs;
		}
		else
			warpCopy<float>(jp, zero, totalSize);

		if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
		{
			warpCopy<PxReal>(msArticulation.jointVelocities, dofData + offset, sizeof(PxReal)*numDofs);
			offset += numDofs;
		}
		else
			warpCopy<float>(jv, zero, totalSize);

		if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_FORCES)
		{
			warpCopy<PxReal>(msArticulation.jointForce, dofData + offset, sizeof(PxReal)*numDofs);
			offset += numDofs;
		}
		else
			warpCopy<float>(jf, zero, totalSize);
		
		if(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
		{
			warpCopy<PxReal>(msArticulation.jointTargetPositions, dofData + offset, sizeof(PxReal) * numDofs);
			offset += numDofs;
		}
		else
			warpCopy<float>(jtp, zero, totalSize);

		if(dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
		{
			warpCopy<PxReal>(msArticulation.jointTargetVelocities, dofData + offset, sizeof(PxReal) * numDofs);
			offset += numDofs;
		}
		else
			warpCopy<float>(jtv, zero, totalSize);

		if (dirtyFlags & Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL)
		{
			warpCopy<uint2>(reinterpret_cast<uint2*>(msArticulation.externalAccelerations), reinterpret_cast<uint2*>(&gNewLinkAccels[startIndex]),
				sizeof(Cm::UnAlignedSpatialVector)*nbLinks);	
		}
		else
			warpCopy<uint2>(reinterpret_cast<uint2*>(msArticulation.externalAccelerations), make_uint2(0,0),
				sizeof(Cm::UnAlignedSpatialVector)*nbLinks);

		// AD: I wonder whether we should raise all the flags here to make jcalc more simple.
	}
}


//we need to think through which case we will need to update body2Actor
extern "C" __global__ void updateArticulationsLaunch(const PxgUpdateArticulationDesc* scDesc, const PxgArticulationSimUpdate* simUpdates, PxU32 nbSimUpdates,
	const PxReal* dofData, const bool directAPI)
{
	//simUpdates and nbSimUpdates are in mapped host memory, so we have to really efficiently read them in...
	Dy::ArticulationJointCore* gNewJointCore = scDesc->mNewJointCores;
	Dy::ArticulationJointCoreData* gNewJointData = scDesc->mNewJointData;
	PxgArticulationLink* gNewLinks = scDesc->mNewLinks;
	PxReal* gNewLinkWakeCounters = scDesc->mNewLinkWakeCounters;
	Cm::UnAlignedSpatialVector* gNewLinkAccels = scDesc->mNewLinkExtAccels;
	PxgArticulationLinkProp* gNewLinkProps = scDesc->mNewLinkProps;
	PxU32* gNewLinkParents = scDesc->mNewLinkParents;
	ArticulationBitField* gNewLinkChildren = scDesc->mNewLinkChildren;
	PxTransform* gNewLinkBody2Worlds = scDesc->mNewLinkBody2Worlds;
	PxTransform* gNewLinkBody2Actors = scDesc->mNewLinkBody2Actors;

	// one articulation is copied by one warp in the block
	const PxU32 numWarpsPerBlock = PxgSimulationCoreKernelBlockDim::NEW_ARTICULATION / WARP_SIZE;

	// we copy the update data PxgArticulationSimUpdate to sharedUpdates in block-shared memory
	const PxU32 startIndex = blockIdx.x*numWarpsPerBlock;
	const PxU32 endIndex = PxMin(startIndex + numWarpsPerBlock, nbSimUpdates);

	__shared__ PxgArticulationSimUpdate sharedUpdates[numWarpsPerBlock];

	__shared__ PxgArticulation shArticulation[numWarpsPerBlock];

	const PxU32 nbThreadsPerItem = sizeof(PxgArticulationSimUpdate) / sizeof(PxU32);

	const PxU32 nbToProcess = endIndex - startIndex;
	// each thread in the block is copying 4 bytes, so total number of threads needed
	// for full copy is sizeof(PxgArticulationSimUpdate) / sizeof(PxU32) * nbToProcess

	const PxU32 blockThreadIdx = threadIdx.x + threadIdx.y*blockDim.x;

	if (blockThreadIdx < (nbThreadsPerItem*nbToProcess))
	{
		PxU32* dst = reinterpret_cast<PxU32*>(sharedUpdates);
		const PxU32* src = reinterpret_cast<const PxU32*>(&simUpdates[startIndex]);

		dst[blockThreadIdx] = src[blockThreadIdx];
	}

	// sync all threads in block - i.e. make sure copy to shared memory has completed
	__syncthreads();

	// each block has numWarpsPerBlock = blockDim.y warps (with 32 threads = blockDim.x)
	// so warp index = articulation index is:
	const PxU32 globalWarpIndex = threadIdx.y + blockDim.y*blockIdx.x;

	if (globalWarpIndex < nbSimUpdates)
	{
		// copy PxgArticulation (the articulation header) into block-shared memory:
		// threadIdx.y is equal to per-block articulation index = per-block warp index
		const PxgArticulationSimUpdate& msUpdate = sharedUpdates[threadIdx.y];
		PxU32 flags = msUpdate.dirtyFlags;
		PxU32 articIndex = msUpdate.articulationIndex;

		PxgArticulation& msArticulation = shArticulation[threadIdx.y];

		warpCopy<PxU32>(reinterpret_cast<PxU32*>(&msArticulation),
			reinterpret_cast<PxU32*>(&scDesc->mArticulationPool[articIndex]), sizeof(PxgArticulation));

		__syncwarp();

		const PxU32 nbLinks = msArticulation.data.numLinks;

		// AD: this will most likely copy across stale joint positions/velocities with direct-GPU API
		// because pos/vel is also stored in Dy::ArticulationJointCore. I couldn't find a place where it's read though.
		if (flags & Dy::ArticulationDirtyFlag::eDIRTY_JOINTS)
		{
			//Copy across joints...

			uint4* sJoints = reinterpret_cast<uint4*>(&gNewJointCore[msUpdate.linkStartIndex]);
			uint4* dJoints = reinterpret_cast<uint4*>(msArticulation.joints);

			//each ArticulationJointCore is less than 304 bytes, each thread read 16 bytes
			warpCopy<uint4>(dJoints, sJoints, sizeof(Dy::ArticulationJointCore) * nbLinks);

			//read joint data, each thread read 4 bytes
			uint* sJointData = reinterpret_cast<uint*>(&gNewJointData[msUpdate.linkStartIndex]);
			uint* dJointData = reinterpret_cast<uint*>(msArticulation.jointData);

			//each ArticulationJointCoreData is 20 bytes, each thread read 4 bytes
			warpCopy<uint>(dJointData, sJointData, sizeof(Dy::ArticulationJointCoreData) * nbLinks);

			// we declare confiDirty in this case? not sure. For armature it could be updateDirty because we only update the values?
			// we need to figure out to what degree there actually cound be a confiDirty coming from the joints. Maybe if we change motion etc? Is that even allowed?
			msArticulation.data.confiDirty = true;
		}

		if (flags & Dy::ArticulationDirtyFlag::eDIRTY_MIMIC_JOINT)
		{
			const PxU32 mimicJointStartIndex = msUpdate.mimicJointStartIndex;
			Dy::ArticulationMimicJointCore* mimicJointCores = &scDesc->mNewArticulationMimicJointPool[mimicJointStartIndex];

			const PxU32 nbMimicJoints = msArticulation.data.numMimicJoints;

			//read mimic joint param, each thread read 16 bytes
			uint4* sMimicJoints = reinterpret_cast<uint4*>(mimicJointCores);
			uint4* dMimicJoints = reinterpret_cast<uint4*>(msArticulation.mimicJointCores);

			//each ArticulationMimicJointCore is 32 bytes, each thread read 16 bytes
			warpCopy<uint4>(dMimicJoints, sMimicJoints, sizeof(Dy::ArticulationMimicJointCore) * nbMimicJoints);
		}

		// only write tendon/tendon joints if direct-GPU API is off.
		if (!directAPI)
		{
			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON)
			{
				const PxU32 spatialTendonStartIndex = msUpdate.spatialTendonStartIndex;
				PxGpuSpatialTendonData* spatialTendonParams = &scDesc->mNewSpatialTendonParamsPool[spatialTendonStartIndex];
			
				const PxU32 nbSpatialTendons = msArticulation.data.numSpatialTendons;

				//read tendon param, each thread read 16 bytes
				uint4* sTendonParams = reinterpret_cast<uint4*>(spatialTendonParams);
				uint4* dTendonParams = reinterpret_cast<uint4*>(msArticulation.spatialTendonParams);

				//each tendon param is 16 bytes, each thread read 16 bytes
				warpCopy<uint4>(dTendonParams, sTendonParams, sizeof(PxGpuSpatialTendonData) * nbSpatialTendons);
			}

			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_SPATIAL_TENDON_ATTACHMENT)
			{
				const PxU32 spatialTendonAttachmentStartIndex = msUpdate.spatialTendonAttachmentStartIndex;

				PxGpuTendonAttachmentData* modData = &scDesc->mNewAttachmentModPool[spatialTendonAttachmentStartIndex];

				const PxU32 nbSpatialTendons = msArticulation.data.numSpatialTendons;

				PxU32 index = 0;
				for (PxU32 i = 0; i < nbSpatialTendons; ++i)
				{
					//copy tendon joint
					PxgArticulationTendon& tendon = msArticulation.spatialTendons[i];
					const PxU32 nbElements = tendon.mNbElements;
					//each PxGpuTendonAttachmentData is 32 bytes, each thread read 16 bytes
					uint4* sTendonAttachment = reinterpret_cast<uint4*>(&modData[index]);
					uint4* dTendonAttachment = reinterpret_cast<uint4*>(tendon.mModElements);

					warpCopy<uint4>(dTendonAttachment, sTendonAttachment, sizeof(PxGpuTendonAttachmentData) * nbElements);

					index += nbElements;
				}
			}
		
			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON)
			{
				const PxU32 fixedTendonStartIndex = msUpdate.fixedTendonStartIndex;
				PxGpuFixedTendonData* fixedTendonParams = &scDesc->mNewFixedTendonParamsPool[fixedTendonStartIndex];
			
				const PxU32 nbFixedTendons = msArticulation.data.numFixedTendons;

				//read tendon param, each thread read 16 bytes
				uint4* sTendonParams = reinterpret_cast<uint4*>(fixedTendonParams);
				uint4* dTendonParams = reinterpret_cast<uint4*>(msArticulation.fixedTendonParams);


				//each tendon param is 12 bytes, each thread read 4 bytes
				warpCopy<uint4>(dTendonParams, sTendonParams, sizeof(PxGpuFixedTendonData) * nbFixedTendons);
			}

			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_FIXED_TENDON_JOINT)
			{
				const PxU32 fixedTendonJointStartIndex = msUpdate.fixedTendonJointStartIndex;

				PxGpuTendonJointCoefficientData* coefficientData = &scDesc->mNewTendonJointCoefficientPool[fixedTendonJointStartIndex];

				const PxU32 nbFixedTendons = msArticulation.data.numFixedTendons;
				PxU32 index = 0;
				for (PxU32 i = 0; i < nbFixedTendons; ++i)
				{
					//copy tendon joint
					PxgArticulationTendon& tendon = msArticulation.fixedTendons[i];
					const PxU32 nbElements = tendon.mNbElements;
					//each PxGpuTendonJointCoefficientData is 8 bytes, each thread read 8 bytes
					uint2* sTendonJointCoefficient = reinterpret_cast<uint2*>(&coefficientData[index]);
					uint2* dTendonJointCoefficient = reinterpret_cast<uint2*>(tendon.mModElements);

					warpCopy<uint2>(dTendonJointCoefficient, sTendonJointCoefficient, sizeof(PxGpuTendonJointCoefficientData) * nbElements);

					index += nbElements;
				}
			}
		}

		if ((flags & Dy::ArticulationDirtyFlag::eDIRTY_LINKS) || (flags & Dy::ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES))
		{
			const PxU32 nbLinks = msArticulation.data.numLinks;

			uint4* sLinks = reinterpret_cast<uint4*>(&gNewLinks[msUpdate.linkStartIndex]);
			uint4* dLinks = reinterpret_cast<uint4*>(msArticulation.links);

			//each PxgArticulationLink is 74 bytes, each thread read 16 bytes
			warpCopy<uint4>(dLinks, sLinks, (sizeof(PxgArticulationLink) * nbLinks));

			if(flags & Dy::ArticulationDirtyFlag::eDIRTY_LINKS)
			{
				//copy link properties
				uint4* sLinkProp = reinterpret_cast<uint4*>(&gNewLinkProps[msUpdate.linkStartIndex]);
				uint4* dLinkProp = reinterpret_cast<uint4*>(msArticulation.linkProps);

				warpCopy<uint4>(dLinkProp, sLinkProp, sizeof(uint4) * nbLinks);

				//read link parent
				uint* sParents = &gNewLinkParents[msUpdate.linkStartIndex];
				uint* dParents = msArticulation.parents;

				warpCopy<uint>(dParents, sParents, sizeof(uint) * nbLinks);

				uint* sChildren = reinterpret_cast<uint*>(&gNewLinkChildren[msUpdate.linkStartIndex]);
				uint* dChildren = reinterpret_cast<uint*>(msArticulation.children);

				warpCopy<uint>(dChildren, sChildren, sizeof(ArticulationBitField)*nbLinks);

				// we update all body2Worlds to a potentially updated CMassLocalPose.
				if (directAPI)
				{
					// AD: this should be flag-guarded because we're doing quite a bit of work here.
					// But currently impossible because we lose track of that information going down the 
					// layers and then just mark the entire body dirty.
					const uint idxInWarp = threadIdx.x & (WARP_SIZE - 1);
					for (uint linkIndex = idxInWarp; linkIndex < nbLinks; linkIndex += WARP_SIZE)
					{
						const PxTransform oldBody2Actor = msArticulation.linkBody2Actors[linkIndex];
						const PxTransform newBody2Actor = gNewLinkBody2Actors[msUpdate.linkStartIndex + linkIndex];

						if (!(oldBody2Actor == newBody2Actor))
						{
							const PxTransform actor2World = msArticulation.linkBody2Worlds[linkIndex] * oldBody2Actor.getInverse();
							msArticulation.linkBody2Worlds[linkIndex] = actor2World * newBody2Actor;
						}
					}
					__syncwarp(); // should be fine, direct-API is always same for everyone!
				}
				else
				{
					//read link body2World
					uint* sBody2Worlds = reinterpret_cast<uint*>(&gNewLinkBody2Worlds[msUpdate.linkStartIndex]);
					uint* dBody2Worlds = reinterpret_cast<uint*>(msArticulation.linkBody2Worlds);

					// each PxTransform is 28 bytes, each thread read 4 bytes
					warpCopy<uint>(dBody2Worlds, sBody2Worlds, sizeof(PxTransform) * nbLinks);
				}

				//read link body2actor
				uint* sBody2Actors = reinterpret_cast<uint*>(&gNewLinkBody2Actors[msUpdate.linkStartIndex]);
				uint* dBody2Actors = reinterpret_cast<uint*>(msArticulation.linkBody2Actors);

				//each PxTransform is 28 bytes, each thread read 4 bytes
				warpCopy<uint>(dBody2Actors, sBody2Actors, sizeof(PxTransform) * nbLinks);
			}
		}
		// I think we need to guard here anyway because updateKinematic now writes directly into the persistent data.
		else if (flags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS && !directAPI)
		{
			//If we didn't update links, but we did update positions, we need to read link poses in regardless!
			//shouldn't updateKinematic just raise a flag?

			//read link body2World
			uint* sBody2Worlds = reinterpret_cast<uint*>(&gNewLinkBody2Worlds[msUpdate.linkStartIndex]);
			uint* dBody2Worlds = reinterpret_cast<uint*>(msArticulation.linkBody2Worlds);

			//each PxTransform is 28 bytes, each thread read 4 bytes
			warpCopy<uint>(dBody2Worlds, sBody2Worlds, sizeof(PxTransform) * nbLinks);
		}

		PxU32 offset = msUpdate.dofDataStartIndex;
		const PxU32 numDofs = msArticulation.data.numJointDofs;

		// only write joint state / forces if direct GPU API is off:
		if (!directAPI)
		{
			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS)
			{
				warpCopy<PxReal>(msArticulation.jointPositions, dofData + offset, sizeof(PxReal) * numDofs);
				offset += numDofs;
				if ((threadIdx.x & 31) == 0)
					msArticulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_POSITIONS;
			}
			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES)
			{
				warpCopy<PxReal>(msArticulation.jointVelocities, dofData + offset, sizeof(PxReal) * numDofs);
				offset += numDofs;
				if ((threadIdx.x & 31) == 0)
					msArticulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_VELOCITIES;
			}
			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_FORCES)
			{
				warpCopy<PxReal>(msArticulation.jointForce, dofData + offset, sizeof(PxReal) * numDofs);
				offset += numDofs;
				if ((threadIdx.x & 31) == 0)
					msArticulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_FORCES;
			}
			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS)
			{
				warpCopy<PxReal>(msArticulation.jointTargetPositions, dofData + offset, sizeof(PxReal) * numDofs);
				offset += numDofs;
				if ((threadIdx.x & 31) == 0)
					msArticulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS;
			}
			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL)
			{
				warpCopy<PxReal>(msArticulation.jointTargetVelocities, dofData + offset, sizeof(PxReal) * numDofs);
				offset += numDofs;
				if ((threadIdx.x & 31) == 0)
					msArticulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL;
			}
			__syncwarp();
		}

		// AD this should theoretically also be covered by the direct API if - it's state but it cannot be set/get from direct-GPU API yet.
		if (flags & Dy::ArticulationDirtyFlag::eDIRTY_WAKECOUNTER)
		{
			//read link body2World
			PxReal* sLinkWakeCounters = &gNewLinkWakeCounters[msUpdate.linkStartIndex];
			PxReal* dLinkWakeCounters = msArticulation.linkWakeCounters;

			//each thread read 4 bytes
			warpCopy<PxReal>(dLinkWakeCounters, sLinkWakeCounters, sizeof(PxReal) * nbLinks);
		}

		if(flags & Dy::ArticulationDirtyFlag::eDIRTY_USER_FLAGS)
		{			
			scDesc->mArticulationPool[articIndex].data.flags = msUpdate.userFlags;
			scDesc->mArticulationPool[articIndex].data.confiDirty = true;
		}

		if (!directAPI)
		{
			if (flags & Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL)
			{
				warpCopy<uint2>(reinterpret_cast<uint2*>(msArticulation.externalAccelerations), reinterpret_cast<uint2*>(&gNewLinkAccels[msUpdate.linkStartIndex]),
					sizeof(Cm::UnAlignedSpatialVector)* nbLinks);
				if ((threadIdx.x & 31) == 0)
					msArticulation.data.gpuDirtyFlag |= Dy::ArticulationDirtyFlag::eDIRTY_EXT_ACCEL;
			}
			__syncwarp();
		}

		// AD the comment below is also outdated, but I'm leaving it here for future reference until the jcalc is cleanup up completely.
		//KS - if only forces are dirty, the confi is considered to *not* be dirty. Any other state forces a complete
		//recreation of the articulation block data format.
		//scDesc->mArticulationPool[articIndex].data.dataDirty = (flags & (~(Dy::ArticulationDirtyFlag::eDIRTY_FORCES | Dy::ArticulationDirtyFlag::eIN_DIRTY_LIST))) != 0;
		
		scDesc->mArticulationPool[articIndex].data.updateDirty = flags & (~(Dy::ArticulationDirtyFlag::eDIRTY_FORCES | Dy::ArticulationDirtyFlag::eIN_DIRTY_LIST));
	}
}

extern "C" __global__ void updateBodyExternalVelocitiesLaunch(const PxgUpdatedBodiesDesc* scDesc)
{
	PxgBodySim*	PX_RESTRICT gBodySim = scDesc->mBodySimBufferDeviceData;
	const PxgBodySimVelocityUpdate* PX_RESTRICT gUpdatedBodies = scDesc->mUpdatedBodySim;
	const PxU32 gNbBodies = scDesc->mNbUpdatedBodies;
	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;
	for (PxU32 i = idx; i < gNbBodies; i += blockDim.x * gridDim.x)
	{
		const PxgBodySimVelocityUpdate& updatedBody = gUpdatedBodies[i];
		float4 linearVel = updatedBody.linearVelocityXYZ_bodySimIndexW;
		const PxU32 bodyIndex = reinterpret_cast<PxU32&>(linearVel.w);
		const float4 originalLinearV = gBodySim[bodyIndex].linearVelocityXYZ_inverseMassW;
		linearVel.w = originalLinearV.w;
		gBodySim[bodyIndex].linearVelocityXYZ_inverseMassW = linearVel;
		gBodySim[bodyIndex].angularVelocityXYZ_maxPenBiasW = updatedBody.angularVelocityXYZ_maxPenBiasW;

		gBodySim[bodyIndex].externalLinearAcceleration = updatedBody.externalLinearAccelerationXYZ;
		gBodySim[bodyIndex].externalAngularAcceleration = updatedBody.externalAngularAccelerationXYZ;
	}
}

//one warp read one PxgD6JointData. This kernel updates the newly added joints/removed joints
extern "C" __global__ void updateJointsLaunch(const PxgUpdatedJointsDesc* jointDesc)
{
	if (blockIdx.y == 0)
	{
		const PxgD6JointData* cpuJointData = jointDesc->mD6RigidJointCPUPool; //in map memory
		PxgD6JointData* gpuJointData = jointDesc->mD6RigidJointGPUPool; //in device memory

		const PxgConstraintPrePrep* cpuJointPrePrep = jointDesc->mD6RigidJointPrePrepCPUPool; //in map memory
		PxgConstraintPrePrep* gpuJointPrePrep = jointDesc->mD6RigidJointPrePrepGPUPool; //in device memory

		const PxU32* updateJointIndices = jointDesc->mUpdatedRigidJointIndices;
		const PxU32 nbUpdatedJoints = jointDesc->mNbUpdatedRigidJoints;

		const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;

		if (globalWarpIndex < nbUpdatedJoints)
		{
			const PxU32 jointIndex = updateJointIndices[globalWarpIndex];

			uint4* dJointData = reinterpret_cast<uint4*>(&gpuJointData[jointIndex]);
			const uint4* sJointData = reinterpret_cast<const uint4*>(&cpuJointData[jointIndex]);

			uint* dJointPrePrep = reinterpret_cast<uint*>(&gpuJointPrePrep[jointIndex]);
			const uint* sJointPrePrep = reinterpret_cast<const uint*>(&cpuJointPrePrep[jointIndex]);

			//using a warp to read one of the PxgD6JointData(496 bytes) and PxgConstraintPrePrep(20). 
			//for the first 31 thread, each thread read 16 bytes so we will can read one PxgD6JointData
			//for the other 5 thread, each thread read 4 bytes so we can read one PxgConstraintPrePrep

			warpCopy<uint4>(dJointData, sJointData, sizeof(PxgD6JointData));

			warpCopy<uint>(dJointPrePrep, sJointPrePrep, sizeof(PxgConstraintPrePrep));
		}
	}
	else
	{
		const PxgD6JointData* cpuJointData = jointDesc->mD6ArtiJointCPUPool; //in map memory
		PxgD6JointData* gpuJointData = jointDesc->mD6ArtiJointGPUPool; //in device memory

		const PxgConstraintPrePrep* cpuJointPrePrep = jointDesc->mD6ArtiJointPrePrepCPUPool; //in map memory
		PxgConstraintPrePrep* gpuJointPrePrep = jointDesc->mD6ArtiJointPrePrepGPUPool; //in device memory

		const PxU32* updateJointIndices = jointDesc->mUpdatedArtiJointIndices;
		const PxU32 nbUpdatedArtiJoints = jointDesc->mNbUpdatedArtiJoints;

		const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;

		if (globalWarpIndex < nbUpdatedArtiJoints)
		{
			const PxU32 jointIndex = updateJointIndices[globalWarpIndex];

			uint4* dJointData = reinterpret_cast<uint4*>(&gpuJointData[jointIndex]);
			const uint4* sJointData = reinterpret_cast<const uint4*>(&cpuJointData[jointIndex]);

			uint* dJointPrePrep = reinterpret_cast<uint*>(&gpuJointPrePrep[jointIndex]);
			const uint* sJointPrePrep = reinterpret_cast<const uint*>(&cpuJointPrePrep[jointIndex]);

			//using a warp to read one of the PxgD6JointData(496 bytes) and PxgConstraintPrePrep(20). 
			//for the first 31 thread, each thread read 16 bytes so we will can read one PxgD6JointData
			//for the other 5 thread, each thread read 4 bytes so we can read one PxgConstraintPrePrep

			warpCopy<uint4>(dJointData, sJointData, sizeof(PxgD6JointData));

			warpCopy<uint>(dJointPrePrep, sJointPrePrep, sizeof(PxgConstraintPrePrep));
		}
	}
}

extern "C" __global__ void getRigidDynamicGlobalPose(
	PxTransform* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgBodySim* PX_RESTRICT bodySimBufferDeviceData,
	const PxU32 nbElements
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		const PxU32 index = gpuIndices[globalThreadIndex];
		const PxgBodySim& bodySim = bodySimBufferDeviceData[index];

		PxAlignedTransform pose = bodySim.body2World * bodySim.body2Actor_maxImpulseW.getInverse();

		data[globalThreadIndex] = pose.getTransform();
	}
}

extern "C" __global__ void getRigidDynamicLinearVelocity(
	PxVec3* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgBodySim* PX_RESTRICT bodySimBufferDeviceData,
	const PxU32 nbElements
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		const PxU32 index = gpuIndices[globalThreadIndex];
		const PxgBodySim& bodySim = bodySimBufferDeviceData[index];

		data[globalThreadIndex] = PxLoad3(bodySim.linearVelocityXYZ_inverseMassW);
	}
}

extern "C" __global__ void getRigidDynamicAngularVelocity(
	PxVec3* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgBodySim* PX_RESTRICT bodySimBufferDeviceData,
	const PxU32 nbElements
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		const PxU32 index = gpuIndices[globalThreadIndex];
		const PxgBodySim& bodySim = bodySimBufferDeviceData[index];

		data[globalThreadIndex] = PxLoad3(bodySim.angularVelocityXYZ_maxPenBiasW);
	}
}

// PT: the accelerations are computed in these kernels (and not directly in the integration kernels)
// to minimize the amount of work done for all bodies. In this version the accelerations are only
// computed for a subset of bodies (the ones passed to the Direct GPU API), while computing them
// during integration would mean computing them for all bodies all the time.
// Also we have two integration kernels (PGS / TGS), so doing the work there would cause more code
// duplication and potential divergences if we only change one version one day, etc.

// PT: for acceleration getters (eENABLE_BODY_ACCELERATIONS)
extern "C" __global__ void getRigidDynamicLinearAcceleration(
	PxVec3* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgBodySim* PX_RESTRICT bodySimBufferDeviceData,
	const PxgBodySimVelocities* PX_RESTRICT prevVelocities,
	const PxU32 nbElements, const float oneOverDt
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		const PxU32 index = gpuIndices[globalThreadIndex];
		const PxgBodySim& bodySim = bodySimBufferDeviceData[index];

		const float4 linVel = bodySim.linearVelocityXYZ_inverseMassW;
		const float4 deltaLinVel = linVel - prevVelocities[index].linearVelocity;
		const float4 linAccel = deltaLinVel * oneOverDt;
		data[globalThreadIndex] = PxLoad3(linAccel);
	}
}

// PT: for acceleration getters (eENABLE_BODY_ACCELERATIONS)
extern "C" __global__ void getRigidDynamicAngularAcceleration(
	PxVec3* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgBodySim* PX_RESTRICT bodySimBufferDeviceData,
	const PxgBodySimVelocities* PX_RESTRICT prevVelocities,
	const PxU32 nbElements, const float oneOverDt
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		const PxU32 index = gpuIndices[globalThreadIndex];
		const PxgBodySim& bodySim = bodySimBufferDeviceData[index];

		const float4 angVel = bodySim.angularVelocityXYZ_maxPenBiasW;
		const float4 deltaAngVel = angVel - prevVelocities[index].angularVelocity;
		const float4 angAccel = deltaAngVel * oneOverDt;
		data[globalThreadIndex] = PxLoad3(angAccel);
	}
}

extern "C" __global__ void setRigidDynamicGlobalPose(
	const PxTransform* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgUpdateActorDataDesc* PX_RESTRICT updateActorDataDesc,
	const PxU32 nbElements,
	const PxU32 totalNumShapes
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		PxgBodySim* PX_RESTRICT gBodySimPool = updateActorDataDesc->mBodySimBufferDeviceData;
		PxNodeIndex* PX_RESTRICT gRigidNodeIndices = updateActorDataDesc->mRigidNodeIndices;
		PxU32* PX_RESTRICT gShapeIndices = updateActorDataDesc->mShapeIndices;
		PxsCachedTransform* PX_RESTRICT gTransformCache = updateActorDataDesc->mTransformCache;
		PxBounds3* PX_RESTRICT bounds = updateActorDataDesc->mBounds;
		PxgShape* PX_RESTRICT gConvexShapes = updateActorDataDesc->mShapes;
		const PxgShapeSim* PX_RESTRICT gShapeSimPool = updateActorDataDesc->mShapeSimsBufferDeviceData;
		PxU32* PX_RESTRICT updated = updateActorDataDesc->mUpdated;

		const PxU32 index = gpuIndices[globalThreadIndex];
		const PxTransform inputTransform = data[globalThreadIndex];

		PxAlignedTransform trans(inputTransform);
		PxgBodySim& bodySim = gBodySimPool[index];

		PxTransform body2World = (trans.transform(bodySim.body2Actor_maxImpulseW)).getTransform();

		bodySim.body2World.p = make_float4(body2World.p.x, body2World.p.y, body2World.p.z, 0.f);
		bodySim.body2World.q.q = make_float4(body2World.q.x, body2World.q.y, body2World.q.z, body2World.q.w);

		PxAlignedTransform body2Actor_maxImpulseW = bodySim.body2Actor_maxImpulseW;
		PxTransform body2Actor;
		body2Actor.p = PxVec3(body2Actor_maxImpulseW.p.x, body2Actor_maxImpulseW.p.y, body2Actor_maxImpulseW.p.z);
		body2Actor.q = body2Actor_maxImpulseW.q;

		if (totalNumShapes == 0)
		{
			// guard against no actors have shapes in scene, in which case gRigidNodeIndices, etc. would have zero elements
			return;
		}

		// we know this is not an articulation link, so all fine.
		const PxNodeIndex nodeIndex(index);

		//this will search for the first pos for the matched node index
		PxU32 pos = binarySearch<PxNodeIndex>(gRigidNodeIndices, totalNumShapes, nodeIndex);

		// go backward through the sorted rigid node index array which has an entry for each
		// shape of the rigid body, and update the shape if it belongs to the rigid body, i.e. the rigid
		// node index matches the rigid body node index
		while (pos != 0xFFffFFff && gRigidNodeIndices[pos] == nodeIndex)
		{
			const PxU32 shapeIndex = gShapeIndices[pos];
			if (shapeIndex != 0xFFffFFff)
			{
				const PxgShapeSim& shapeSim = gShapeSimPool[shapeIndex];

				const PxTransform absPos = getAbsPose(body2World, shapeSim.mTransform, body2Actor);

				//update broad phase bound, transform cache
				updateCacheAndBound(absPos, shapeSim, shapeIndex, gTransformCache, bounds, gConvexShapes, true);

				updated[shapeIndex] = 1;
			}
			pos--;
		}
	}
}

extern "C" __global__ void setRigidDynamicLinearVelocity(
	const PxVec3* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgUpdateActorDataDesc* PX_RESTRICT updateActorDataDesc,
	PxgBodySimVelocities* PX_RESTRICT prevVelocities,
	const PxU32 nbElements
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		PxgBodySim* gBodySimPool = updateActorDataDesc->mBodySimBufferDeviceData;
		const PxU32 index = gpuIndices[globalThreadIndex];
		PxgBodySim& bodySim = gBodySimPool[index];
		const float4 linearVelocityXYZ_inverseMassW = bodySim.linearVelocityXYZ_inverseMassW;

		const PxVec3 linVel = data[globalThreadIndex];
		const float4 lv = make_float4(linVel.x, linVel.y, linVel.z, linearVelocityXYZ_inverseMassW.w);
		bodySim.linearVelocityXYZ_inverseMassW = lv;
		if(prevVelocities)
			prevVelocities[index].linearVelocity = lv;
	}
}

extern "C" __global__ void setRigidDynamicAngularVelocity(
	const PxVec3* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgUpdateActorDataDesc* PX_RESTRICT updateActorDataDesc,
	PxgBodySimVelocities* PX_RESTRICT prevVelocities,
	const PxU32 nbElements
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		PxgBodySim* gBodySimPool = updateActorDataDesc->mBodySimBufferDeviceData;
		const PxU32 index = gpuIndices[globalThreadIndex];
		PxgBodySim& bodySim = gBodySimPool[index];
		const float4 angularVelocityXYZ_maxPenBiasW = bodySim.angularVelocityXYZ_maxPenBiasW;

		const PxVec3 angVel = data[globalThreadIndex];
		const float4 av = make_float4(angVel.x, angVel.y, angVel.z, angularVelocityXYZ_maxPenBiasW.w);
		bodySim.angularVelocityXYZ_maxPenBiasW = av;
		if(prevVelocities)
			prevVelocities[index].angularVelocity = av;
	}
}

extern "C" __global__ void setRigidDynamicForce(
	const PxVec3* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgUpdateActorDataDesc* PX_RESTRICT updateActorDataDesc,
	const PxU32 nbElements
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		PxgBodySim* gBodySimPool = updateActorDataDesc->mBodySimBufferDeviceData;
		const PxU32 index = gpuIndices[globalThreadIndex];
		PxgBodySim& bodySim = gBodySimPool[index];

		const PxVec3 force = data[globalThreadIndex];

		const float4 linearVelocityXYZ_inverseMassW = bodySim.linearVelocityXYZ_inverseMassW;

		const PxVec3 deltaLin = force * linearVelocityXYZ_inverseMassW.w;
		bodySim.externalLinearAcceleration = make_float4(deltaLin.x, deltaLin.y, deltaLin.z, 0.f);
	}
}

extern "C" __global__ void setRigidDynamicTorque(
	const PxVec3* PX_RESTRICT data,
	const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices,
	const PxgUpdateActorDataDesc* PX_RESTRICT updateActorDataDesc,
	const PxU32 nbElements
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbElements)
	{
		PxgBodySim* gBodySimPool = updateActorDataDesc->mBodySimBufferDeviceData;
		const PxU32 index = gpuIndices[globalThreadIndex];
		PxgBodySim& bodySim = gBodySimPool[index];

		const PxVec3 torque = data[globalThreadIndex];
		const float4 invInertia = bodySim.inverseInertiaXYZ_contactReportThresholdW;

		// TODO AD: transform the torque into body space, multiply with inertia, transform acceleration back into world space.
		PxMat33 inverseInertiaWorldSpace;
		Cm::transformInertiaTensor(PxVec3(invInertia.x, invInertia.y, invInertia.z), PxMat33(bodySim.body2World.q), inverseInertiaWorldSpace);

		const PxVec3 deltaAng = inverseInertiaWorldSpace * torque;
		bodySim.externalAngularAcceleration = make_float4(deltaAng.x, deltaAng.y, deltaAng.z, 0.0f);
	}
}

extern "C" __global__ void copyUserData(PxgPtrPair* pairs, const PxU32 numToProcess)
{
	if (blockIdx.y < numToProcess)
	{
		const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
		const PxU32 totalThreads = blockDim.x * gridDim.x;
		PxgPtrPair& pair = pairs[blockIdx.y];
		const PxU32 size = (PxU32)(pair.size/4);
		const PxU32* src = (PxU32*)pair.src;
		PxU32* dst = (PxU32*)pair.dst;

		for (size_t i = globalThreadIdx; i < size; i += totalThreads)
		{
			dst[i] = src[i];
		}
	}
}

// only used for template instantiation to describe the meaning of the template integer parameter
static const PxU32 gExtractForce = 0;
static const PxU32 gExtractTorque = 1;

//
// tExtractForce defines whether the constraint/joint force or torque is extracted
//
template<PxU32 tExtractOperation>
static PX_FORCE_INLINE __device__  void getD6JointForceOrTorque(
	PxU32 globalThreadIndex,
	PxVec3* PX_RESTRICT data,
	const PxD6JointGPUIndex* PX_RESTRICT gpuIndices,
	PxU32 nbElements,
	const PxgConstraintWriteback* PX_RESTRICT constraintWriteBackBuffer,
	PxF32 oneOverDt,
	const PxgConstraintIdMapEntry* PX_RESTRICT constraintIdMap,
	PxU32 constraintIdMapSize)
{
	if (globalThreadIndex < nbElements)
	{
		const PxU32 index = gpuIndices[globalThreadIndex];

		// Note: the gpu API index is the constraint ID

		if ((index < constraintIdMapSize) && (constraintIdMap[index].isJointDataIdValid()))
		{
			// the constraint ID is the force write back index
			const PxgConstraintWriteback& constraintWriteBack = constraintWriteBackBuffer[index];

			if (tExtractOperation == gExtractForce)
				data[globalThreadIndex] = PxLoad3(constraintWriteBack.linearImpulse_broken) * oneOverDt;
			else
				data[globalThreadIndex] = PxLoad3(constraintWriteBack.angularImpulse_residual) * oneOverDt;
		}
		else
		{
			assert(index < constraintIdMapSize);
			// with correct usage the above should always hold but a user might, for example, add
			// joints and directly get some parameter via direct GPU API without simulating first.

			// the joint might be inactive/sleeping (note that at the time of writing this, joints could not
			// go to sleep with direct GPU API enabled since the sleep state of rigid bodies did not get sent
			// back to the host). Nonetheless, this case is covered here already and the only valid scenario
			// to reach the else-statement.

			data[globalThreadIndex] = PxVec3(0.0f);
		}
	}
}

extern "C" __global__ void getD6JointForces(
	PxVec3* PX_RESTRICT data,
	const PxD6JointGPUIndex* PX_RESTRICT gpuIndices,
	PxU32 nbElements,
	const PxgConstraintWriteback* PX_RESTRICT constraintWriteBackBuffer,
	PxF32 oneOverDt,
	const PxgConstraintIdMapEntry* PX_RESTRICT constraintIdMap,
	PxU32 constraintIdMapSize
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	getD6JointForceOrTorque<gExtractForce>(
		globalThreadIndex,
		data, gpuIndices, nbElements,
		constraintWriteBackBuffer, oneOverDt, 
		constraintIdMap, constraintIdMapSize);
}

extern "C" __global__ void getD6JointTorques(
	PxVec3* PX_RESTRICT data,
	const PxD6JointGPUIndex* PX_RESTRICT gpuIndices,
	PxU32 nbElements,
	const PxgConstraintWriteback* PX_RESTRICT constraintWriteBackBuffer,
	PxF32 oneOverDt,
	const PxgConstraintIdMapEntry* PX_RESTRICT constraintIdMap,
	PxU32 constraintIdMapSize
)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	getD6JointForceOrTorque<gExtractTorque>(
		globalThreadIndex,
		data, gpuIndices, nbElements,
		constraintWriteBackBuffer, oneOverDt, 
		constraintIdMap, constraintIdMapSize);
}
