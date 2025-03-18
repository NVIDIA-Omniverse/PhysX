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

const PxRigidDynamic* NpOmniPvdSimulationControllerCallbacks::castPxsRigidBodyToPxRigidDynamic(const PxsRigidBody* rigidBody)
{
	const PxsBodyCore& pxsBodyCoreRef = rigidBody->getCore();
	Sc::BodyCore& bodyCoreRef = Sc::BodyCore::getCore(pxsBodyCoreRef);
	PxRigidDynamic* rigidDynamic = static_cast<PxRigidDynamic*>(bodyCoreRef.getPxActor());
	PX_ASSERT(rigidDynamic->getConcreteType() == PxConcreteType::eRIGID_DYNAMIC);
	return rigidDynamic;
}

const NpArticulationReducedCoordinate* NpOmniPvdSimulationControllerCallbacks::castFeatherstoneToNpArticulation(const Dy::FeatherstoneArticulation* const featherstone)
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

#define SET_RIGID_DYNAMIC_ATTRIBUTES(rigids, dataVec, gpuIndices, nbElements, PxActorType, DataVecType, attribute) \
{ \
	DataVecType* dataVecCasted = (DataVecType*)dataVec; \
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData) \
	for (PxU32 i = 0; i < nbElements; i++) \
	{ \
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActorType, attribute, *castPxsRigidBodyToPxRigidDynamic(rigids[gpuIndices[i]]), dataVecCasted[i]); \
	} \
	OMNI_PVD_WRITE_SCOPE_END \
}

#define SET_RIGID_DYNAMIC_ATTRIBUTES_WITH_RESET(rigids, dataVec, gpuIndices, nbElements, npScene, resetAttributeFunction, PxActorType, DataVecType, attribute) \
{ \
	NpOmniPvdSceneClient& ovdClient = npScene.getSceneOvdClientInternal(); \
	DataVecType* dataVecCasted = (DataVecType*)dataVec; \
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData) \
	for (PxU32 i = 0; i < nbElements; i++) \
	{ \
		const PxRigidDynamic* ridiDynamic = castPxsRigidBodyToPxRigidDynamic(rigids[gpuIndices[i]]); \
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActorType, attribute, *ridiDynamic, dataVecCasted[i]); \
		ovdClient.resetAttributeFunction(ridiDynamic); \
	} \
	OMNI_PVD_WRITE_SCOPE_END \
}

void NpOmniPvdSimulationControllerCallbacks::processRigidDynamicSet(const PxsRigidBody* const * rigids, const void* dataVec, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements)
{
	PX_COMPILE_TIME_ASSERT(sizeof(PxVec3) == (sizeof(PxF32) * 3));
	PX_COMPILE_TIME_ASSERT(sizeof(PxTransform) == (sizeof(PxF32) * 7));

	switch (dataType)		
	{
	case PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE:
	{
		SET_RIGID_DYNAMIC_ATTRIBUTES(rigids, dataVec, gpuIndices, nbElements, PxRigidActor, PxTransform, globalPose)
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY:
	{	
		SET_RIGID_DYNAMIC_ATTRIBUTES(rigids, dataVec, gpuIndices, nbElements, PxRigidBody, PxVec3, linearVelocity)
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY:
	{
		SET_RIGID_DYNAMIC_ATTRIBUTES(rigids, dataVec, gpuIndices, nbElements, PxRigidBody, PxVec3, angularVelocity)
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eFORCE:
	{
		SET_RIGID_DYNAMIC_ATTRIBUTES_WITH_RESET(rigids, dataVec, gpuIndices, nbElements, mNpScene, addRigidDynamicForceReset, PxRigidBody, PxVec3, force)
		break;
	}
	case PxRigidDynamicGPUAPIWriteType::eTORQUE:
	{
		SET_RIGID_DYNAMIC_ATTRIBUTES_WITH_RESET(rigids, dataVec, gpuIndices, nbElements, mNpScene, addRigidDynamicTorqueReset, PxRigidBody, PxVec3, torque)
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

void NpOmniPvdSimulationControllerCallbacks::streamJointValues(const PxArticulationGPUAPIWriteType::Enum dataType, const Dy::FeatherstoneArticulation* const * articulations, PxReal* realsDataVec, const PxArticulationGPUIndex* nodeIndices,
	PxU32 nbArticulations, PxU32 maxLinks, PxU32 maxSubElementsInBlock)
{
	NpOmniPvdSceneClient& ovdClient = mNpScene.getSceneOvdClientInternal();
	mDofStarts.resizeUninitialized(maxLinks);
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
		setDofOffsetVec(mDofStarts, nbLinks, npLinks);
		for(PxU32 linkId = 1; linkId < nbLinks; linkId++)
		{
			const NpArticulationLink* npLink = npLinks[linkId];						
			PxReal* realsForDofs = &realsForArticulation[mDofStarts[npLink->getLinkIndex()]];
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

#define SET_ARTICULATION_ROOT_ATTRIBUTES(articulations, dataVec, nbrSubElementsPerBlock, PxActorType, DataVecType, attribute) \
{ \
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData) \
	DataVecType* dataVecCasted = (DataVecType*)dataVec; \
	for (PxU32 artId = 0; artId < nbElements; artId++) \
	{ \
		const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]); \
		DataVecType* attribData = &dataVecCasted[ artId * nbrSubElementsPerBlock ]; \
		PxActorType& actor = *static_cast<PxActorType*>(npArticulation->mArticulationLinks[0]); \
		OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActorType, attribute, actor, *attribData) \
	} \
	OMNI_PVD_WRITE_SCOPE_END \
}

#define SET_ARTICULATION_LINK_ATTRIBUTES(articulations, dataVec, nbrSubElementsPerBlock, npScene, linkAttributeResetFunction, PxActorType, DataVecType, attribute) \
{ \
	NpOmniPvdSceneClient& ovdClient = npScene.getSceneOvdClientInternal(); \
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData) \
	DataVecType* dataVecCasted = (DataVecType*)dataVec; \
	for (PxU32 artId = 0; artId < nbElements; artId++) \
	{ \
		const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]); \
		ovdClient.linkAttributeResetFunction(static_cast<const PxArticulationReducedCoordinate*>(npArticulation)); \
		DataVecType* attribData = &dataVecCasted[ artId * nbrSubElementsPerBlock ]; \
		const PxU32 nbLinks = npArticulation->getNbLinks(); \
		const NpArticulationLink* const * npLinks = npArticulation->getLinks(); \
		for (PxU32 linkId = 0; linkId < nbLinks; linkId++) \
		{ \
			const NpArticulationLink* npLink = npLinks[linkId]; \
			const PxU32 linkLowLevelIndex = npLink->getLinkIndex(); \
			const PxActorType& rb = *static_cast<const PxActorType*>(npLink); \
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActorType, attribute, rb, attribData[linkLowLevelIndex]) \
		} \
	} \
	OMNI_PVD_WRITE_SCOPE_END \
} 

// The nodeIndices are expected to be remapped from gpuIndices to nodeIndices before this function is called
// The remapping is done in PxgArticulationCore::ovdArticulationCallback
void NpOmniPvdSimulationControllerCallbacks::processArticulationSet(const Dy::FeatherstoneArticulation* const *articulations, const void* dataVec, const PxArticulationGPUIndex* nodeIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements,
		PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments)
{
	PxU32 nbrSubElementsPerBlock;
	PxU32 blockSize;
	getArticulationDataElements(dataType, maxLinks, maxDofs, maxFixedTendons, maxTendonJoints, maxSpatialTendons, maxSpatialTendonAttachments, nbrSubElementsPerBlock, blockSize);

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
			SET_ARTICULATION_ROOT_ATTRIBUTES(articulations, dataVec, nbrSubElementsPerBlock, PxRigidActor, PxTransform, globalPose)
			break;
		}
        case PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY:
		{
			SET_ARTICULATION_ROOT_ATTRIBUTES(articulations, dataVec, nbrSubElementsPerBlock, PxRigidBody, PxVec3, linearVelocity)
			break;
		}
		case PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY:
		{
			SET_ARTICULATION_ROOT_ATTRIBUTES(articulations, dataVec, nbrSubElementsPerBlock, PxRigidBody, PxVec3, angularVelocity)
			break;
		}
		case PxArticulationGPUAPIWriteType::eLINK_FORCE:
		{
			SET_ARTICULATION_LINK_ATTRIBUTES(articulations, dataVec, nbrSubElementsPerBlock, mNpScene, addArticulationLinksForceReset, PxRigidBody, PxVec3, force)
			break;
		}
		case PxArticulationGPUAPIWriteType::eLINK_TORQUE:
		{
			SET_ARTICULATION_LINK_ATTRIBUTES(articulations, dataVec, nbrSubElementsPerBlock, mNpScene, addArticulationLinksTorqueReset, PxRigidBody, PxVec3, torque)
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