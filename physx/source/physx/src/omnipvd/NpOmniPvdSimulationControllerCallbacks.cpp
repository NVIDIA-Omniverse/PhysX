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

#include "NpOmniPvdSimulationControllerCallbacks.h"

#if PX_SUPPORT_OMNI_PVD

#include "omnipvd/NpOmniPvdSetData.h"
#include "NpArticulationReducedCoordinate.h"
#include "PxArticulationTendonData.h"
#include "DyFeatherstoneArticulation.h"
#include "ScArticulationSim.h"
#include "ScPhysics.h"

using namespace physx;

NpOmniPvdSimulationControllerCallbacks::NpOmniPvdSimulationControllerCallbacks(NpScene& scene) :
	mNpScene(scene)
{
}

const PxRigidDynamic* NpOmniPvdSimulationControllerCallbacks::castPxsRigidBodyToPxRigidDynamic(PxsRigidBody* const rigidBody)
{
	PxsBodyCore& pxsBodyCoreRef = rigidBody->getCore();
	Sc::BodyCore& bodyCoreRef = Sc::BodyCore::getCore(pxsBodyCoreRef);
	PxRigidDynamic* rigidDynamic = static_cast<PxRigidDynamic*>(bodyCoreRef.getPxActor());
	PX_ASSERT(rigidDynamic->getConcreteType() == PxConcreteType::eRIGID_DYNAMIC);
	return rigidDynamic;
}

const NpArticulationReducedCoordinate* NpOmniPvdSimulationControllerCallbacks::castFeatherstoneToNpArticulation(Dy::FeatherstoneArticulation* const featherstone)
{
	// Dy::FeatherstoneArticulation's constructor takes a void pointer argument, which is stored as userData in the
	// object. This userData argument is the Sc::ArticulationSim pointer for the purpose of the PxgArticulationCore
	// See Sc::ArticulationSim constructor
	Sc::ArticulationSim* articulationSim = (Sc::ArticulationSim*)(featherstone->getUserData());
	Sc::ArticulationCore& articulationCore = articulationSim->getCore();
	NpArticulationReducedCoordinate* npArticulation = static_cast<NpArticulationReducedCoordinate*>(Sc::gOffsetTable.convertScArticulation2Px(&articulationCore));
	PX_ASSERT(npArticulation->getConcreteType() == PxConcreteType::eARTICULATION_REDUCED_COORDINATE);
	return npArticulation;
}

// Returns the number of elements in a data block see PxArticulationGPUAPIWriteType::Enum for where the sizes etc are derived
PxU32 NpOmniPvdSimulationControllerCallbacks::getArticulationDataMaxSubElementsNb(PxArticulationGPUAPIWriteType::Enum dataType,
	PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments)
{
	PxU32 nbSubElements = 1;
	switch(dataType)
	{
	case PxArticulationGPUAPIWriteType::eJOINT_POSITION:
	{
		// The joint positions. 1 PxReal per dof. Block size per articulation: maxDofs.
		nbSubElements = maxDofs;
		break;
	}
	case PxArticulationGPUAPIWriteType::eJOINT_VELOCITY:
	{
		// The joint velocities. 1 PxReal per dof. Block size per articulation: maxDofs.
		nbSubElements = maxDofs;
		break;
	}
	case PxArticulationGPUAPIWriteType::eJOINT_FORCE:
	{
		// The applied joint forces or torques. 1 PxReal per dof. Block size per articulation: maxDofs.
		nbSubElements = maxDofs;
		break;
	}
	case PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY:
	{
		// The velocity targets for the joint drives. 1 PxReal per dof. Block size per articulation: maxDofs.
		nbSubElements = maxDofs;
		break;
	}
	case PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION:
	{
		// The position targets for the joint drives. 1 PxReal per dof. Block size per articulation: maxDofs.
		nbSubElements = maxDofs;
		break;
	}
	case PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE:
	{
		// The root link transform. 1 PxTransform per articulation. Block size per articulation: 1.
		nbSubElements = 1;
		break;
	}
	case PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY:
	{
		// The root link linear velocity. 1 PxVec3 per articulation. Block size per articulation: 1.
		nbSubElements = 1;
		break;
	}
	case PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY:
	{
		// The root link angular velocity. 1 PxVec3 per articulation. Block size per articulation: 1.
		nbSubElements = 1;
		break;
	}
	case PxArticulationGPUAPIWriteType::eLINK_FORCE:
	{
		// The forces to apply to links. 1 PxVec3 per link. Block size per articulation: maxLinks.
		nbSubElements = maxLinks;
		break;
	}
	case PxArticulationGPUAPIWriteType::eLINK_TORQUE:
	{
		// The torques to apply to links. 1 PxVec3 per link. Block size per articulation: maxLinks.
		nbSubElements = maxLinks;
		break;
	}
	case PxArticulationGPUAPIWriteType::eFIXED_TENDON:
	{
		// Fixed tendon data. 1 PxGpuFixedTendonData per fixed tendon. Block size per articulation: maxFixedTendons.
		nbSubElements = maxFixedTendons;
		break;
	}
	case PxArticulationGPUAPIWriteType::eFIXED_TENDON_JOINT:
	{
		// Fixed tendon joint data. 1 PxGpuTendonJointCoefficientData per fixed tendon joint. Block size per articulation: maxFixedTendons * maxFixedTendonJoints.
		nbSubElements = maxFixedTendons * maxTendonJoints;
		break;
	}
	case PxArticulationGPUAPIWriteType::eSPATIAL_TENDON:
	{
		// Spatial tendon data. 1 PxGpuSpatialTendonData per spatial tendon. Block size per articulation: maxSpatialTendons.
		nbSubElements = maxSpatialTendons;
		break;
	}
	case PxArticulationGPUAPIWriteType::eSPATIAL_TENDON_ATTACHMENT:
	{
		// Spatial tendon attachment data. 1 PxGpuTendonAttachmentData per spatial tendon attachment. Block size per articulation: maxSpatialTendons * maxSpatialTendonAttachments.
		nbSubElements = maxSpatialTendons * maxSpatialTendonAttachments;
		break;
	}
	default:
		PX_ASSERT(false);
		nbSubElements = 0;
		break;
	}
	return nbSubElements;
}

void NpOmniPvdSimulationControllerCallbacks::processRigidDynamicSet(PxsRigidBody** rigids, void* dataVec, PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements)
{
	PX_COMPILE_TIME_ASSERT(sizeof(PxVec3) == (sizeof(PxF32) * 3));
	PX_COMPILE_TIME_ASSERT(sizeof(PxTransform) == (sizeof(PxF32) * 7));

	switch (dataType)		
	{
	case PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE:
	{
		PxTransform* tformsDataVec = (PxTransform*)dataVec;
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		for (PxU32 i = 0; i < nbElements; i++)
		{
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, *castPxsRigidBodyToPxRigidDynamic(rigids[gpuIndices[i]]), tformsDataVec[i]);
		}
		OMNI_PVD_WRITE_SCOPE_END
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY:
	{		
		PxVec3* linearVelocitiesDataVec = (PxVec3*)dataVec;
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		for (PxU32 i = 0; i < nbElements; i++)
		{
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, *castPxsRigidBodyToPxRigidDynamic(rigids[gpuIndices[i]]), linearVelocitiesDataVec[i])
		}
		OMNI_PVD_WRITE_SCOPE_END
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY:
	{
		PxVec3* angularVelocitiesDataVec = (PxVec3*)dataVec;
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		for (PxU32 i = 0; i < nbElements; i++)
		{
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, *castPxsRigidBodyToPxRigidDynamic(rigids[gpuIndices[i]]), angularVelocitiesDataVec[i])
		}
		OMNI_PVD_WRITE_SCOPE_END
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eFORCE:
	{
		NpOmniPvdSceneClient& ovdClient = mNpScene.getSceneOvdClientInternal();
		PxVec3* forcesDataVec = (PxVec3*)dataVec;
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		for (PxU32 i = 0; i < nbElements; i++)
		{
			const PxRigidDynamic* ridiDynamic = castPxsRigidBodyToPxRigidDynamic(rigids[gpuIndices[i]]);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, force, *ridiDynamic, forcesDataVec[i])
			ovdClient.addRigidDynamicForceReset(ridiDynamic);
		}
		OMNI_PVD_WRITE_SCOPE_END
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eTORQUE:
	{
		NpOmniPvdSceneClient& ovdClient = mNpScene.getSceneOvdClientInternal();
		PxVec3* torquesDataVec = (PxVec3*)dataVec;
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		for (PxU32 i = 0; i < nbElements; i++)
		{
			const PxRigidDynamic* ridiDynamic = castPxsRigidBodyToPxRigidDynamic(rigids[gpuIndices[i]]);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, torque, *ridiDynamic, torquesDataVec[i])
			ovdClient.addRigidDynamicTorqueReset(ridiDynamic);
		}
		OMNI_PVD_WRITE_SCOPE_END
		break;
	}
	default:
		break;
	}
}

void NpOmniPvdSimulationControllerCallbacks::setDofOffsetVec(PxArray<PxU32>& dofStarts, PxU32 nbLinks, const NpArticulationLink* const * npLinks)
{
	for(PxU32 i = 0; i < nbLinks; i++)
	{
		dofStarts[i]=0;
	}
	for(PxU32 i = 1; i < nbLinks; i++)
	{
		dofStarts[npLinks[i]->getLinkIndex()] = npLinks[i]->getInboundJointDof();
	}
	PxU32 count = 0;
	for(PxU32 i = 1; i < nbLinks; i++)
	{
		PxU32 dofs = dofStarts[i];
		dofStarts[i] = count;
		count += dofs;
	}
}

void NpOmniPvdSimulationControllerCallbacks::streamJointValues(const PxArticulationGPUAPIWriteType::Enum dataType, Dy::FeatherstoneArticulation** articulations, PxReal* realsDataVec, PxArticulationGPUIndex* nodeIndices,
	PxU32 nbArticulations, PxU32 maxLinks, PxU32 maxSubElementsInBlock)
{
	NpOmniPvdSceneClient& ovdClient = mNpScene.getSceneOvdClientInternal();
	PxArray<PxU32> dofStarts(maxLinks);
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	for (PxU32 artId = 0; artId < nbArticulations; artId++)
	{
		const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]);		
		if (dataType==PxArticulationGPUAPIWriteType::eJOINT_FORCE)
		{
			ovdClient.addArticulationJointsForceReset(static_cast<const PxArticulationReducedCoordinate*>(npArticulation));
		}
		PxReal* realsForArticulation = &realsDataVec[ artId * maxSubElementsInBlock ];
		const PxU32 nbLinks = npArticulation->getNbLinks();
		const NpArticulationLink* const * npLinks = npArticulation->getLinks();
		setDofOffsetVec(dofStarts, nbLinks, npLinks);
		for(PxU32 linkId = 1; linkId < nbLinks; linkId++)
		{
			const NpArticulationLink* npLink = npLinks[linkId];						
			PxReal* realsForDofs = &realsForArticulation[dofStarts[npLink->getLinkIndex()]];
			const PxArticulationJointReducedCoordinate& joint = *static_cast<const PxArticulationJointReducedCoordinate*>(npLink->getInboundJoint());
			const PxU32 nbrDofs = npLink->getInboundJointDof();
			switch(dataType)
			{
				case PxArticulationGPUAPIWriteType::eJOINT_POSITION:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointPosition, joint, realsForDofs, nbrDofs);
					break;
				}
				case PxArticulationGPUAPIWriteType::eJOINT_VELOCITY:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointVelocity, joint, realsForDofs, nbrDofs);
					break;
				}
				case PxArticulationGPUAPIWriteType::eJOINT_FORCE:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointForce, joint, realsForDofs, nbrDofs);					
					break;
				}
				case PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveVelocity, joint, realsForDofs, nbrDofs);
					break;
				}
				case PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveTarget, joint, realsForDofs, nbrDofs);
					break;
				}
				default:
					break;					
			}
		}
	}
	OMNI_PVD_WRITE_SCOPE_END
}

// The nodeIndices are expected to be remapped from gpuIndices to nodeIndices before this function is called
// The remapping is done in PxgArticulationCore::ovdArticulationCallback
void NpOmniPvdSimulationControllerCallbacks::processArticulationSet(Dy::FeatherstoneArticulation** articulations, void* dataVec, PxArticulationGPUIndex* nodeIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements,
		PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments)
{
	PxU32 nbrSubElementsPerBlock = getArticulationDataMaxSubElementsNb(dataType,
		maxLinks, maxDofs, maxFixedTendons, maxTendonJoints, maxSpatialTendons, maxSpatialTendonAttachments);

	switch(dataType)
	{
		case PxArticulationGPUAPIWriteType::eJOINT_POSITION:
		case PxArticulationGPUAPIWriteType::eJOINT_VELOCITY:
		case PxArticulationGPUAPIWriteType::eJOINT_FORCE:
		case PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY:
		case PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION:
		{
			streamJointValues(dataType, articulations, (PxReal*)dataVec, nodeIndices, nbElements, maxLinks, nbrSubElementsPerBlock);
			break;
		}
		case PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE:
		{
			OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
			PxTransform* tformDataVec = (PxTransform*)dataVec;
			for (PxU32 artId = 0; artId < nbElements; artId++)
			{
				const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]);
				PxTransform* tform = &tformDataVec[ artId * nbrSubElementsPerBlock ];
				PxRigidActor& rb = *static_cast<PxRigidActor*>(npArticulation->mArticulationLinks[0]);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, rb, *tform)
			}
			OMNI_PVD_WRITE_SCOPE_END
			break;
		}
        case PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY:
		{
			OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
			PxVec3* velocitiesDataVec = (PxVec3*)dataVec;
			for (PxU32 artId = 0; artId < nbElements; artId++)
			{
				const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]);
				PxVec3* velocity = &velocitiesDataVec[ artId * nbrSubElementsPerBlock ];
				PxRigidBody& rb = *static_cast<PxRigidBody*>(npArticulation->mArticulationLinks[0]);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity , rb, *velocity)
			}
			OMNI_PVD_WRITE_SCOPE_END
			break;
		}
		case PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY:
		{
			OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
			PxVec3* velocitiesDataVec = (PxVec3*)dataVec;
			for (PxU32 artId = 0; artId < nbElements; artId++)
			{
				const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]);
				PxVec3* velocity = &velocitiesDataVec[ artId * nbrSubElementsPerBlock ];
				PxRigidBody& rb = *static_cast<PxRigidBody*>(npArticulation->mArticulationLinks[0]);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity , rb, *velocity)
			}
			OMNI_PVD_WRITE_SCOPE_END
			break;
		}
		case PxArticulationGPUAPIWriteType::eLINK_FORCE:
		{
			NpOmniPvdSceneClient& ovdClient = mNpScene.getSceneOvdClientInternal();
			OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
			PxVec3* forcesDataVec = (PxVec3*)dataVec;
			for (PxU32 artId = 0; artId < nbElements; artId++)
			{
				const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]);
				ovdClient.addArticulationLinksForceReset(static_cast<const PxArticulationReducedCoordinate*>(npArticulation));
				PxVec3* forcesForArticulation = &forcesDataVec[ artId * nbrSubElementsPerBlock ];
				const PxU32 nbLinks = npArticulation->getNbLinks();
				NpArticulationLink* const * npLinks = const_cast<NpArticulationReducedCoordinate*>(npArticulation)->getLinks();
				for (PxU32 linkId = 0; linkId < nbLinks; linkId++)
				{
					const NpArticulationLink* npLink = npLinks[linkId];
					const PxU32 linkLowLevelIndex = npLink->getLinkIndex();
					const PxRigidBody& rb = *static_cast<const PxRigidBody*>(npLink);
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, force, rb, forcesForArticulation[linkLowLevelIndex])
				}
			}
			OMNI_PVD_WRITE_SCOPE_END
			break;
		}
		case PxArticulationGPUAPIWriteType::eLINK_TORQUE:
		{
			NpOmniPvdSceneClient& ovdClient = mNpScene.getSceneOvdClientInternal();
			OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
			PxVec3* torquesDataVec = (PxVec3*)dataVec;
			for (PxU32 artId = 0; artId < nbElements; artId++)
			{
				const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]);
				ovdClient.addArticulationLinksTorqueReset(static_cast<const PxArticulationReducedCoordinate*>(npArticulation));
				PxVec3* torquesForArticulation = &torquesDataVec[ artId * nbrSubElementsPerBlock ];
				const PxU32 nbLinks = npArticulation->getNbLinks();
				NpArticulationLink* const * npLinks = const_cast<NpArticulationReducedCoordinate*>(npArticulation)->getLinks();
				for (PxU32 linkId = 0; linkId < nbLinks; linkId++)
				{
					const NpArticulationLink* npLink = npLinks[linkId];
					const PxU32 linkLowLevelIndex = npLink->getLinkIndex();
					const PxRigidBody& rb = *static_cast<const PxRigidBody*>(npLink);
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, torque, rb, torquesForArticulation[linkLowLevelIndex])
				}
			}
			OMNI_PVD_WRITE_SCOPE_END
			break;
		}
		case PxArticulationGPUAPIWriteType::eFIXED_TENDON:
		{
			break;
		}
		case PxArticulationGPUAPIWriteType::eFIXED_TENDON_JOINT:
		{
			break;
		}
		case PxArticulationGPUAPIWriteType::eSPATIAL_TENDON:
		{
			break;
		}
		case PxArticulationGPUAPIWriteType::eSPATIAL_TENDON_ATTACHMENT:
		{
			break;
		}
		default:
			PX_ALWAYS_ASSERT();
	}
}	
#endif