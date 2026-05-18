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

#ifndef PXG_ARTICULATION_H
#define PXG_ARTICULATION_H

#include "PxgArticulationBlockData.h"
#include "PxgArticulationLink.h"
#include "PxgArticulationTendon.h"
#include "DyFeatherstoneArticulation.h"

namespace physx
{
	class PxGpuSpatialTendonData;
	class PxGpuFixedTendonData;

	namespace Dy
	{
		struct ArticulationJointCore;
		class ArticulationJointCoreData;
		struct SpatialSubspaceMatrix;
	}

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4324)
#endif
	PX_ALIGN_PREFIX(16)
	class PxgArticulation
	{
	public:
	
		PxgArticulationData						data;							

		PxgArticulationLink*					links;							
		Dy::ArticulationJointCore*				joints;							
		Dy::ArticulationJointCoreData*			jointData;						
		
		Cm::UnAlignedSpatialVector*				motionVelocities;				
		Cm::UnAlignedSpatialVector*				motionAccelerations;			
		Cm::UnAlignedSpatialVector*				linkIncomingJointForces;	
		Cm::UnAlignedSpatialVector*				coriolisVectors;				
		Cm::UnAlignedSpatialVector*				zAForces; // used as temporary propagation buffer in inverseDynamics and to store TGS per substep isolated forces while the solver runs. Not cleared after use.
		
		Cm::UnAlignedSpatialVector*				externalAccelerations;			

		Cm::UnAlignedSpatialVector*				rootPreMotionVelocity;

		PxReal*									jointPositions;					
		PxReal*									jointVelocities;				
		PxReal*									jointAccelerations;				
		PxReal*									jointForce;						
		PxReal*									jointTargetPositions;
		PxReal*									jointTargetVelocities;
		
		PxU32*									jointOffsets;					

		PxSpatialMatrix*						worldSpatialArticulatedInertia;	
		PxSpatialMatrix*						spatialResponseMatrixW;
		PxTransform*							linkBody2Worlds;	
		PxU8*									linkJointRootStateDataBuffer;			
		PxTransform*							linkBody2Actors;
		PxU32*									parents;						
		//Local space motion matrix - constant unless properties are changed
		Dy::SpatialSubspaceMatrix*				motionMatrix;					
		//World space motion matrix - computed from local matrix each frame
		Dy::SpatialSubspaceMatrix*				worldMotionMatrix; // AD: only inverse dynamics now.				

		PxReal*									linkWakeCounters;
		PxgArticulationLinkSleepData*			linkSleepData;
		PxgArticulationLinkProp*				linkProps;						
		ArticulationBitField*					children;
		PxU32*									pathToRoot;

		PxQuat*									relativeQuat;

		PxQuat*									tempParentToChilds;
		PxVec3*									tempRs;

		PxGpuSpatialTendonData*					spatialTendonParams;
		PxgArticulationTendon*					spatialTendons;

		PxGpuFixedTendonData*					fixedTendonParams;
		PxgArticulationTendon*					fixedTendons;

		PxReal*									cfms;
		PxReal*									cfmScale;

		Dy::ArticulationMimicJointCore*			mimicJointCores;

		PX_ALIGN(16, PxSpatialMatrix)			invSpatialArticulatedInertiaW;
	}
	PX_ALIGN_SUFFIX(16);
#if PX_VC
#pragma warning(pop)
#endif

	/*
	\brief We aggregate link, joint, and root link state data into a single char buffer.
	We do this in two ways:
	a) a single buffer for each articulation
	b) a single buffer for all articulations
	The typical pattern of data flow is as follows:
	a) we store state data in a unique device buffer for each articulation.
	b) we copy from the individual device buffers per articulation to the single device buffer for all articulations.
	c) we copy the single buffer for all articulations from device to host 
	d) we distribute state data from the single host buffer on the host to the individual articulation instances on the host.
	The state data that we store is as follows:
	a) link body2Worlds array, link velocities array, link accelerations array, link incoming joint forces array
	b) joint positions array, joint velocities array, joint accelerations array
	d) root link pre-sim velocity
	The struct PxgLinkJointRootStateData contains helper functions for allocating and querying 
	state data buffers.
	*/
	struct PxgArticulationLinkJointRootStateData
	{/**
		\brief Compute the number of bytes required for an articulation with known 
		maximum link count and known maximum dof count.
		\param[in] maxNbLinks is the maximum number of links of any articulation in the ensemble of articulations.
		\param[in] maxNbDofs is the maximum number of dofs of any articulation in the ensemble of articulations.
		\note This does not return an aligned size, use computeStateDataBufferByteSizeAligned16 for that purpose
		\return The number of bytes required to store the state data for an articulation.
		*/
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 computeSingleArticulationStateDataBufferByteSizeRaw
		(const PxU32 maxNbLinks, const PxU32 maxNbDofs)
		{
			PxU32 byteSizePerArt =
				(sizeof(PxTransform) + 3 * sizeof(Cm::UnAlignedSpatialVector)) * maxNbLinks;		//link pose + link velocity + link acceleration + link incoming joint force
			byteSizePerArt += sizeof(PxReal) * maxNbDofs; //joint pos
			byteSizePerArt += sizeof(PxReal) * maxNbDofs; //joint vel
			byteSizePerArt += sizeof(PxReal) * maxNbDofs; //joint accel
			byteSizePerArt += sizeof(Cm::UnAlignedSpatialVector);											//root pre-sim vel
			return byteSizePerArt;
		}

		/**
		\brief Compute the number of bytes required for an ensemble of articulations with known 
		maximum link count and known maximum dof count.
		\param[in] maxNbLinks is the maximum number of links of any articulation in the ensemble of articulations.
		\param[in] maxNbDofs is the maximum number of dofs of any articulation in the ensemble of articulations.
		\param[in] nbArticulations is the number of articulations in the ensemble. 
		\note This may be used to compute the number of bytes required for a single articulation by setting 
		nbArticulations to 1 and setting maxNbLinks etc to be the link and dof count of that articulation.
		\return The number of bytes required to store the state data for an ensemble of articulations.
		*/
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 computeStateDataBufferByteSizeAligned16
		(const PxU32 maxNbLinks, const PxU32 maxNbDofs, const PxU32 nbArticulations)
		{
			const PxU32 byteSizePerArt = computeSingleArticulationStateDataBufferByteSizeRaw(maxNbLinks, maxNbDofs);
			const PxU32 byteSize16PerArt = ((byteSizePerArt + 15) & ~15);						//align the size upwards to the next 16-byte boundary
			return (byteSize16PerArt * nbArticulations);											
		}

		/**
		\brief Return the pointer to a single articulation's state data buffer.
		\param[in] inputBufferForAllArticulations is a pointer to the memory containing the state
		data for the entire ensemble of articulations.
		\param[in] maxNbLinks is the maximum number of links of any articulation in the ensemble of articulations.
		\param[in] maxNbDofs is the maximum number of dofs of any articulation in the ensemble of articulations.
		\param[in] articulationId is the index of a single articulation within the ensemble. 
		\return The pointer to a single articulation's state data buffer.
		*/
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU8* getArticulationStateDataBuffer
		(PxU8* inputBufferForAllArticulations,
		 const PxU32 maxNbLinks, const PxU32 maxNbDofs,
		 const PxU32 articulationId)
		{
			PxU8* singleArticulationStateBuffer =
				inputBufferForAllArticulations + 
				computeStateDataBufferByteSizeAligned16(maxNbLinks, maxNbDofs, articulationId);

			return singleArticulationStateBuffer;
		}

		/**
		\brief Decompose the state data buffer of a single articulation into pointers to arrays of 
		poses, velocities etc.
		\param[in] singleArticulationStateBuffer is the state data buffer for a single articulation.
		\param[in] nbLinks is the number of links for the single articulation. 
		\param[in] nbDofs is the number of dofs for the single articulation. 
		\param[out] linkBody2Worlds is an array of link poses with one element per link.
		\param[out] linkVels is a pointer to an array of link spatial velocities with one element per link.
		\param[out] linkIncomingJointForces is a pointer to an array of link incoming joint forces with one element per link.
		\param[out] jointPositions is a pointer to an array of joint positions with one element per dof.
		\param[out] jointVelocities is a pointer to an array of joint velocities with one element per link.
		\param[out] jointAccelerations is a pointer to an array of joint accelerations with one element per dof.
		\param[out] rootPreVel is a pointer to the pre-sim velocity of the single articulation's root link. 
		*/
		static PX_CUDA_CALLABLE PX_FORCE_INLINE void decomposeArticulationStateDataBuffer
		(PxU8* singleArticulationStateBuffer,
		 const PxU32 nbLinks, const PxU32 nbDofs,
		 PxTransform*& linkBody2Worlds, Cm::UnAlignedSpatialVector*& linkVels, Cm::UnAlignedSpatialVector*& linkAccels, Cm::UnAlignedSpatialVector*& linkIncomingJointForces,
		 PxReal*& jointPositions, PxReal*& jointVelocities, PxReal*& jointAccelerations,
		 Cm::UnAlignedSpatialVector*& rootPreVel)
		{
			PxU8* buffer = singleArticulationStateBuffer;
			linkBody2Worlds = reinterpret_cast<PxTransform*>(buffer);
			buffer += sizeof(PxTransform) * nbLinks;
			linkVels = reinterpret_cast<Cm::UnAlignedSpatialVector*>(buffer);
			buffer += sizeof(Cm::UnAlignedSpatialVector) * nbLinks;
			linkAccels = reinterpret_cast<Cm::UnAlignedSpatialVector*>(buffer);
			buffer += sizeof(Cm::UnAlignedSpatialVector) * nbLinks;
			linkIncomingJointForces = reinterpret_cast<Cm::UnAlignedSpatialVector*>(buffer);
			buffer += sizeof(Cm::UnAlignedSpatialVector) * nbLinks;
			jointPositions = reinterpret_cast<PxReal*>(buffer);
			buffer += sizeof(PxReal) * nbDofs;
			jointVelocities = reinterpret_cast<PxReal*>(buffer);
			buffer += sizeof(PxReal) * nbDofs;
			jointAccelerations = reinterpret_cast<PxReal*>(buffer);
			buffer += sizeof(PxReal) * nbDofs;
			rootPreVel =  reinterpret_cast<Cm::UnAlignedSpatialVector*>(buffer);
			PX_ASSERT(
				singleArticulationStateBuffer + computeStateDataBufferByteSizeAligned16(nbLinks, nbDofs, 1) == 
				reinterpret_cast<PxU8*>(((reinterpret_cast<size_t>(buffer) + sizeof(Cm::UnAlignedSpatialVector) + 15) & ~15)));
		}

		/**
		\brief Compute the pointer to the array of link poses for a single articulation.
		\param[in] singleArticulationStateBuffer is the state data buffer for a single articulation.
		\param[in] nbLinks is the number of links for the single articulation. 
		\param[in] nbDofs is the number of dofs for the single articulation. 
		\return The pointer to the array of link poses for a single articulation. 
		*/
		//Get the body2World array from the state data buffer of a single articulation.
		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform* getArticulationBody2Worlds
		(PxU8* singleArticulationStateBuffer, 
		 const PxU32 nbLinks, const PxU32 nbDofs)
		{
			PxTransform* linkBody2Worlds;
			Cm::UnAlignedSpatialVector* linkVels;
			Cm::UnAlignedSpatialVector* linkAccels;
			Cm::UnAlignedSpatialVector* linkIncomingJointForces;
			PxReal* jointPositions;
			PxReal* jointVelocities;
			PxReal* jointAccelerations;
			Cm::UnAlignedSpatialVector* rootPreVel;

			decomposeArticulationStateDataBuffer(
				singleArticulationStateBuffer,
				nbLinks, nbDofs, 
				linkBody2Worlds, linkVels, linkAccels, linkIncomingJointForces,
				jointPositions, jointVelocities, jointAccelerations,
				rootPreVel);
	
			return linkBody2Worlds;
		}
	};

	//Helper function to compute the index of a particular link's deltaV value in the deltaV buffer.
	//We store this in a particular order to try and minimize cache misses
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 computeDeltaVIndex(const PxU32 maxArticulations, const PxU32 maxLinks, 
		const PxU32 articulationID, const PxU32 linkID, const PxU32 slabID)
	{
		return articulationID + linkID * maxArticulations + slabID*maxArticulations*maxLinks;
	}

}

#endif
