// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "NpOmniPvdSimulationControllerCallbacks.h"

#if PX_SUPPORT_OMNI_PVD

#include "omnipvd/NpOmniPvdSetData.h"
#include "NpArticulationReducedCoordinate.h"
#include "PxArticulationTendonData.h"
#include "DyFeatherstoneArticulation.h"
#include "ScArticulationSim.h"
#include "ScPhysics.h"
#include "OmniPvdPxSampler.h"
#include "NpOmniPvd.h"
#include "NpRigidDynamic.h"
#include "NpShape.h"
#include "GuBounds.h"

using namespace physx;

static PxBounds3 getCapturedWorldBounds(const NpShapeManager& shapeManager, const PxTransform& actorPose)
{
	PxBounds3 bounds = PxBounds3::empty();
	NpShape* const* shapes = shapeManager.getShapes();
	for (PxU32 i = 0; i < shapeManager.getNbShapes(); ++i)
		bounds.include(Gu::computeBounds(shapes[i]->getCore().getGeometry(), actorPose * shapes[i]->getLocalPoseFast()));
	if (bounds.isEmpty()) return bounds;
	return PxBounds3::centerExtents(bounds.getCenter(), bounds.getExtents() * 1.01f);
}

NpOmniPvdSimulationControllerCallbacks::NpOmniPvdSimulationControllerCallbacks(NpScene& scene) :
	mNpScene(scene)
{
}

bool NpOmniPvdSimulationControllerCallbacks::isCaptureActive() const
{
	return NpOmniPvdCaptureScene(mNpScene);
}

bool NpOmniPvdSimulationControllerCallbacks::isSelectiveCapture() const
{
	return NpOmniPvdCaptureIsSelective();
}

bool NpOmniPvdSimulationControllerCallbacks::isCapturingAllObjects() const
{
	OmniPvdPxSampler* sampler = OmniPvdPxSampler::getSamplingInstance();
	return sampler && sampler->isCapturingAllObjects();
}

bool NpOmniPvdSimulationControllerCallbacks::isTakingSnapshot() const
{
	OmniPvdPxSampler* sampler = OmniPvdPxSampler::getInstance();
	return sampler && sampler->isTakingSnapshot();
}

bool NpOmniPvdSimulationControllerCallbacks::isRigidDynamicReadEnabled(PxRigidDynamicGPUAPIReadType::Enum dataType) const
{
	if (!isCaptureActive()) return false;
	switch (dataType)
	{
	case PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE: return true;
	case PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY:
	case PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY: return true;
	case PxRigidDynamicGPUAPIReadType::eLINEAR_ACCELERATION:
	case PxRigidDynamicGPUAPIReadType::eANGULAR_ACCELERATION:
		return mNpScene.getFlagsFast().isSet(PxSceneFlag::eENABLE_BODY_ACCELERATIONS);
	default: return false;
	}
}

bool NpOmniPvdSimulationControllerCallbacks::isRigidDynamicWriteEnabled(PxRigidDynamicGPUAPIWriteType::Enum dataType) const
{
	if (!isCaptureActive()) return false;
	switch (dataType)
	{
	case PxRigidDynamicGPUAPIWriteType::eGLOBAL_POSE: return true;
	case PxRigidDynamicGPUAPIWriteType::eLINEAR_VELOCITY:
	case PxRigidDynamicGPUAPIWriteType::eANGULAR_VELOCITY: return true;
	case PxRigidDynamicGPUAPIWriteType::eFORCE:
	case PxRigidDynamicGPUAPIWriteType::eTORQUE: return NpOmniPvdRecordSecondaryData();
	default: return false;
	}
}

bool NpOmniPvdSimulationControllerCallbacks::isArticulationReadEnabled(PxArticulationGPUAPIReadType::Enum dataType) const
{
	if (!isCaptureActive()) return false;
	switch (dataType)
	{
	case PxArticulationGPUAPIReadType::eJOINT_POSITION: return true;
	case PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE: return true;
	case PxArticulationGPUAPIReadType::eJOINT_VELOCITY:
	case PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY:
	case PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY: return true;
	case PxArticulationGPUAPIReadType::eJOINT_ACCELERATION:
	case PxArticulationGPUAPIReadType::eLINK_LINEAR_ACCELERATION:
	case PxArticulationGPUAPIReadType::eLINK_ANGULAR_ACCELERATION:
		return mNpScene.getFlagsFast().isSet(PxSceneFlag::eENABLE_BODY_ACCELERATIONS);
	case PxArticulationGPUAPIReadType::eJOINT_FORCE: return NpOmniPvdRecordSecondaryData();
	case PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE: return NpOmniPvdRecordSecondaryData();
	case PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION:
	case PxArticulationGPUAPIReadType::eJOINT_TARGET_VELOCITY: return NpOmniPvdRecordSecondaryData();
	default: return false;
	}
}

bool NpOmniPvdSimulationControllerCallbacks::isArticulationWriteEnabled(PxArticulationGPUAPIWriteType::Enum dataType) const
{
	if (!isCaptureActive()) return false;
	switch (dataType)
	{
	case PxArticulationGPUAPIWriteType::eJOINT_POSITION:
	case PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE: return true;
	case PxArticulationGPUAPIWriteType::eJOINT_VELOCITY:
	case PxArticulationGPUAPIWriteType::eROOT_LINEAR_VELOCITY:
	case PxArticulationGPUAPIWriteType::eROOT_ANGULAR_VELOCITY: return true;
	case PxArticulationGPUAPIWriteType::eJOINT_FORCE:
	case PxArticulationGPUAPIWriteType::eLINK_FORCE:
	case PxArticulationGPUAPIWriteType::eLINK_TORQUE: return NpOmniPvdRecordSecondaryData();
	case PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION:
	case PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY: return NpOmniPvdRecordSecondaryData();
	default: return false;
	}
}

bool NpOmniPvdSimulationControllerCallbacks::isRigidDynamicCaptured(const PxsRigidBody& rigidBody) const
{
	return NpOmniPvdCaptureActor(*castPxsRigidBodyToPxRigidDynamic(&rigidBody));
}

bool NpOmniPvdSimulationControllerCallbacks::isArticulationCaptured(const Dy::FeatherstoneArticulation& articulation) const
{
	return NpOmniPvdCaptureArticulation(*castFeatherstoneToNpArticulation(&articulation));
}

bool NpOmniPvdSimulationControllerCallbacks::isRigidDynamicStateRequired(const PxsRigidBody& rigidBody, PxRigidDynamicGPUAPIReadType::Enum dataType) const
{
	const PxRigidDynamic& actor = *castPxsRigidBodyToPxRigidDynamic(&rigidBody);
	if (NpOmniPvdCaptureActor(actor))
		return true;
	// Counterparts and joint-state actors get pose and velocities only.
	return (dataType == PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE || dataType == PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY
		|| dataType == PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY)
		&& (NpOmniPvdCaptureActorState(actor) || OmniPvdPxSampler::getInstance()->isJointStateActor(actor));
}

bool NpOmniPvdSimulationControllerCallbacks::isArticulationStateRequired(const Dy::FeatherstoneArticulation& articulation, PxArticulationGPUAPIReadType::Enum dataType) const
{
	const NpArticulationReducedCoordinate& actor = *castFeatherstoneToNpArticulation(&articulation);
	const bool linkPayload = dataType == PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE
		|| dataType == PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY || dataType == PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY;
	if (NpOmniPvdCaptureArticulation(actor)) return true;
	if (linkPayload)
		for (PxU32 i = 0; i < actor.getNbLinks(); ++i)
			if (NpOmniPvdCaptureActorState(*actor.getLinks()[i]) || OmniPvdPxSampler::getInstance()->isJointStateActor(*actor.getLinks()[i])) return true;
	return false;
}

bool NpOmniPvdSimulationControllerCallbacks::isContactPairCaptured(const PxActor& actor0, const PxActor& actor1) const
{
	return isContactReadEnabled()
		&& (NpOmniPvdCaptureActor(actor0) || NpOmniPvdCaptureActor(actor1));
}

bool NpOmniPvdSimulationControllerCallbacks::isContactReadEnabled() const
{
	return isCaptureActive() && NpOmniPvdRecordSecondaryData();
}

void NpOmniPvdSimulationControllerCallbacks::reportReadbackFailure()
{
	PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
		"OmniPVD GPU capture readback failed; recording stopped to avoid incomplete state.");
	OmniPvdPxSampler* sampler = OmniPvdPxSampler::getInstance();
	if (sampler) sampler->stopSampling();
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
	const Sc::ArticulationSim* articulationSim = static_cast<const Sc::ArticulationSim*>(featherstone);
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

static bool captureArticulationTopologyFits(const NpArticulationReducedCoordinate& articulation, PxU32 maxLinks, PxU32 maxDofs)
{
	const PxU32 count = articulation.getNbLinks();
	if (count > maxLinks) return false;
	const NpArticulationLink* const* links = articulation.getLinks();
	PxU32 dofs = 0;
	for (PxU32 i = 0; i < count; ++i)
	{
		if (links[i]->getLinkIndex() >= maxLinks || links[i]->getInboundJointDof() > maxDofs - dofs) return false;
		dofs += links[i]->getInboundJointDof();
	}
	return true;
}

// Device data packs unlocked axes; OmniPVD uses PxArticulationAxis order with zeros for locked axes.
static void scatterJointDofs(const PxArticulationJointReducedCoordinate& joint, const PxReal* dofs, PxU32 nbDofs, PxReal (&axes)[PxArticulationAxis::eCOUNT])
{
	PxU32 dof = 0;
	for (PxU32 axis = 0; axis < PxArticulationAxis::eCOUNT; ++axis)
		axes[axis] = (joint.getMotion(static_cast<PxArticulationAxis::Enum>(axis)) != PxArticulationMotion::eLOCKED && dof < nbDofs) ? dofs[dof++] : 0.0f;
}

void NpOmniPvdSimulationControllerCallbacks::streamJointValues(const PxArticulationGPUAPIWriteType::Enum dataType, const Dy::FeatherstoneArticulation* const * articulations, PxReal* realsDataVec, const PxArticulationGPUIndex* nodeIndices,
	PxU32 nbArticulations, PxU32 maxLinks, PxU32 maxSubElementsInBlock)
{
	mDofStarts.resizeUninitialized(maxLinks);
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	for (PxU32 artId = 0; artId < nbArticulations; artId++)
	{
		const NpArticulationReducedCoordinate* npArticulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]);
		if (!captureArticulationTopologyFits(*npArticulation, maxLinks, maxSubElementsInBlock)) continue;
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
			PxReal axes[PxArticulationAxis::eCOUNT];
			scatterJointDofs(joint, realsForDofs, nbrDofs, axes);
			switch(dataType)
			{
				case PxArticulationGPUAPIWriteType::eJOINT_POSITION:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointPosition, joint, axes, PxArticulationAxis::eCOUNT);
					break;
				}
				case PxArticulationGPUAPIWriteType::eJOINT_VELOCITY:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointVelocity, joint, axes, PxArticulationAxis::eCOUNT);
					break;
				}
				case PxArticulationGPUAPIWriteType::eJOINT_FORCE:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointForce, joint, axes, PxArticulationAxis::eCOUNT);
					break;
				}
				case PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveVelocity, joint, axes, PxArticulationAxis::eCOUNT);
					break;
				}
				case PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION:
				{
					OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, driveTarget, joint, axes, PxArticulationAxis::eCOUNT);
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
		if (!captureArticulationTopologyFits(*npArticulation, nbrSubElementsPerBlock, maxDofs)) continue; \
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
void NpOmniPvdSimulationControllerCallbacks::processRigidDynamicRead(const PxsRigidBody* const* rigids, const void* data,
	const PxRigidDynamicGPUIndex* indices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements)
{
	switch (dataType)
	{
	case PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE:
	{
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		const bool emitState = NpOmniPvdCaptureIsSelective() || OmniPvdPxSampler::getInstance()->isTakingSnapshot();
		for (PxU32 i = 0; i < nbElements; ++i)
		{
			const NpRigidDynamic& actor = static_cast<const NpRigidDynamic&>(*castPxsRigidBodyToPxRigidDynamic(rigids[indices[i]]));
			const PxTransform& pose = static_cast<const PxTransform*>(data)[i];
			OmniPvdPxSampler::getInstance()->storeJointActorPose(actor, pose);
			if (!emitState || !NpOmniPvdCaptureActorState(actor)) continue;
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, actor, pose);
			OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, actor, getCapturedWorldBounds(actor.getShapeManager(), pose));
		}
		OMNI_PVD_WRITE_SCOPE_END
		break;
	}
	case PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY:
	case PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY:
	{
		const bool angular = dataType == PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY;
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		const bool emitState = NpOmniPvdCaptureIsSelective() || OmniPvdPxSampler::getInstance()->isTakingSnapshot();
		for (PxU32 i = 0; i < nbElements; ++i)
		{
			const PxRigidDynamic& actor = *castPxsRigidBodyToPxRigidDynamic(rigids[indices[i]]);
			const PxVec3& velocity = static_cast<const PxVec3*>(data)[i];
			OmniPvdPxSampler::getInstance()->storeJointActorVelocity(actor, velocity, angular);
			if (!emitState || !NpOmniPvdCaptureActorState(actor)) continue;
			if (angular) { OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, actor, velocity); }
			else { OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, actor, velocity); }
		}
		OMNI_PVD_WRITE_SCOPE_END
		break;
	}
	case PxRigidDynamicGPUAPIReadType::eLINEAR_ACCELERATION:
	{
		SET_RIGID_DYNAMIC_ATTRIBUTES(rigids, data, indices, nbElements, PxRigidBody, const PxVec3, linearAcceleration)
		break;
	}
	case PxRigidDynamicGPUAPIReadType::eANGULAR_ACCELERATION:
	{
		SET_RIGID_DYNAMIC_ATTRIBUTES(rigids, data, indices, nbElements, PxRigidBody, const PxVec3, angularAcceleration)
		break;
	}
	default: PX_ASSERT(false); break;
	}
}

void NpOmniPvdSimulationControllerCallbacks::processArticulationRead(const Dy::FeatherstoneArticulation* const* articulations,
	const void* data, const PxArticulationGPUIndex* nodeIndices, PxArticulationGPUAPIReadType::Enum dataType,
	PxU32 nbElements, PxU32 maxLinks, PxU32 maxDofs)
{
	PxArticulationGPUAPIWriteType::Enum writeType;
	switch (dataType)
	{
	case PxArticulationGPUAPIReadType::eJOINT_POSITION: writeType = PxArticulationGPUAPIWriteType::eJOINT_POSITION; break;
	case PxArticulationGPUAPIReadType::eJOINT_VELOCITY: writeType = PxArticulationGPUAPIWriteType::eJOINT_VELOCITY; break;
	case PxArticulationGPUAPIReadType::eJOINT_FORCE: writeType = PxArticulationGPUAPIWriteType::eJOINT_FORCE; break;
	case PxArticulationGPUAPIReadType::eJOINT_TARGET_POSITION: writeType = PxArticulationGPUAPIWriteType::eJOINT_TARGET_POSITION; break;
	case PxArticulationGPUAPIReadType::eJOINT_TARGET_VELOCITY: writeType = PxArticulationGPUAPIWriteType::eJOINT_TARGET_VELOCITY; break;
	default:
	{
		OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
		const bool emitState = NpOmniPvdCaptureIsSelective() || OmniPvdPxSampler::getInstance()->isTakingSnapshot();
		const bool linkPayload = dataType == PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE ||
			dataType == PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY || dataType == PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY;
		for (PxU32 artId = 0; artId < nbElements; ++artId)
		{
			const NpArticulationReducedCoordinate* articulation = castFeatherstoneToNpArticulation(articulations[nodeIndices[artId]]);
			if (!captureArticulationTopologyFits(*articulation, maxLinks, maxDofs)) continue;
			const NpArticulationLink* const* links = articulation->getLinks();
			PxBounds3 articulationBounds = PxBounds3::empty();
			if (dataType == PxArticulationGPUAPIReadType::eJOINT_ACCELERATION)
			{
				mDofStarts.resizeUninitialized(maxLinks);
				setDofOffsetVec(mDofStarts, articulation->getNbLinks(), links);
			}
			for (PxU32 linkId = 0; linkId < articulation->getNbLinks(); ++linkId)
			{
				const NpArticulationLink& link = *links[linkId];
				const PxU32 offset = artId * maxLinks + link.getLinkIndex();
				if (dataType == PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE)
					OmniPvdPxSampler::getInstance()->storeJointActorPose(link, static_cast<const PxTransform*>(data)[offset]);
				else if (dataType == PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY || dataType == PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY)
					OmniPvdPxSampler::getInstance()->storeJointActorVelocity(link, static_cast<const PxVec3*>(data)[offset], dataType == PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY);
				if (linkPayload ? (!emitState || !NpOmniPvdCaptureActorState(link)) : !NpOmniPvdCaptureArticulation(*articulation)) continue;
				switch (dataType)
				{
				case PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE:
				{
					const PxTransform& pose = static_cast<const PxTransform*>(data)[offset];
					const PxBounds3 bounds = getCapturedWorldBounds(link.getShapeManager(), pose);
					articulationBounds.include(bounds);
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidActor, globalPose, link, pose);
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxActor, worldBounds, link, bounds);
					break;
				}
				case PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY:
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearVelocity, link, static_cast<const PxVec3*>(data)[offset]); break;
				case PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY:
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularVelocity, link, static_cast<const PxVec3*>(data)[offset]); break;
				case PxArticulationGPUAPIReadType::eLINK_LINEAR_ACCELERATION:
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, linearAcceleration, link, static_cast<const PxVec3*>(data)[offset]); break;
				case PxArticulationGPUAPIReadType::eLINK_ANGULAR_ACCELERATION:
					OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxRigidBody, angularAcceleration, link, static_cast<const PxVec3*>(data)[offset]); break;
				case PxArticulationGPUAPIReadType::eJOINT_ACCELERATION:
					if (link.getInboundJoint())
					{
						const PxReal* values = static_cast<const PxReal*>(data) + artId * maxDofs + mDofStarts[link.getLinkIndex()];
						PxReal axes[PxArticulationAxis::eCOUNT];
						scatterJointDofs(*link.getInboundJoint(), values, link.getInboundJointDof(), axes);
						OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, jointAcceleration, *link.getInboundJoint(), axes, PxArticulationAxis::eCOUNT);
					}
					break;
				case PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE:
					if (link.getInboundJoint())
					{
						const PxVec3* values = static_cast<const PxVec3*>(data) + 2 * offset;
						OmniPvdJointReaction force;
						force.force = values[0]; force.torque = values[1];
						OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationJointReducedCoordinate, incomingJointForce, *link.getInboundJoint(), force);
					}
					break;
				default: PX_ASSERT(false); break;
				}
			}
			if (emitState && dataType == PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE && NpOmniPvdCaptureArticulation(*articulation))
			{
				if (!articulationBounds.isEmpty())
					articulationBounds = PxBounds3::centerExtents(articulationBounds.getCenter(), articulationBounds.getExtents() * 1.01f);
				OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxArticulationReducedCoordinate, worldBounds, *articulation, articulationBounds);
			}
		}
		OMNI_PVD_WRITE_SCOPE_END
		return;
	}
	}
	processArticulationSet(articulations, data, nodeIndices, writeType, nbElements, maxLinks, maxDofs, 0, 0, 0, 0);
}
#endif
