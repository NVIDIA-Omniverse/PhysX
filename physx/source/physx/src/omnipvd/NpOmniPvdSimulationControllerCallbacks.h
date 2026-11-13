// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_OMNI_PVD_SIMULATION_CONTROLLER_CALLBACKS_H
#define NP_OMNI_PVD_SIMULATION_CONTROLLER_CALLBACKS_H

#if PX_SUPPORT_OMNI_PVD

#include "PxsSimulationController.h"
#include "foundation/PxArray.h"

namespace physx
{
class NpArticulationLink;
class NpArticulationReducedCoordinate;
class NpScene;

class NpOmniPvdSimulationControllerCallbacks : public PxsSimulationControllerOVDCallbacks
{  
public:
	NpOmniPvdSimulationControllerCallbacks(NpScene& scene);
	virtual bool isCaptureActive() const PX_OVERRIDE;
	virtual bool isSelectiveCapture() const PX_OVERRIDE;
	virtual bool isCapturingAllObjects() const PX_OVERRIDE;
	virtual bool isTakingSnapshot() const PX_OVERRIDE;
	virtual bool isRigidDynamicReadEnabled(PxRigidDynamicGPUAPIReadType::Enum dataType) const PX_OVERRIDE;
	virtual bool isRigidDynamicWriteEnabled(PxRigidDynamicGPUAPIWriteType::Enum dataType) const PX_OVERRIDE;
	virtual bool isArticulationReadEnabled(PxArticulationGPUAPIReadType::Enum dataType) const PX_OVERRIDE;
	virtual bool isArticulationWriteEnabled(PxArticulationGPUAPIWriteType::Enum dataType) const PX_OVERRIDE;
	virtual bool isRigidDynamicCaptured(const PxsRigidBody& rigidBody) const PX_OVERRIDE;
	virtual bool isArticulationCaptured(const Dy::FeatherstoneArticulation& articulation) const PX_OVERRIDE;
	virtual bool isRigidDynamicStateRequired(const PxsRigidBody& rigidBody, PxRigidDynamicGPUAPIReadType::Enum dataType) const PX_OVERRIDE;
	virtual bool isArticulationStateRequired(const Dy::FeatherstoneArticulation& articulation, PxArticulationGPUAPIReadType::Enum dataType) const PX_OVERRIDE;
	virtual bool isContactReadEnabled() const PX_OVERRIDE;
	virtual bool isContactPairCaptured(const PxActor& actor0, const PxActor& actor1) const PX_OVERRIDE;
	virtual void reportReadbackFailure() PX_OVERRIDE;
	virtual void processRigidDynamicRead(const PxsRigidBody* const* rigids, const void* data, const PxRigidDynamicGPUIndex* indices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements) PX_OVERRIDE;
	virtual void processArticulationRead(const Dy::FeatherstoneArticulation* const* articulations, const void* data, const PxArticulationGPUIndex* nodeIndices, PxArticulationGPUAPIReadType::Enum dataType, PxU32 nbElements, PxU32 maxLinks, PxU32 maxDofs) PX_OVERRIDE;
	virtual void processRigidDynamicSet(const PxsRigidBody* const * rigids, const void* dataVec, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements) PX_OVERRIDE;
	virtual void processArticulationSet(const Dy::FeatherstoneArticulation* const * articulations, const void* dataVec, const PxArticulationGPUIndex* nodeIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements,
			PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments) PX_OVERRIDE;
private:
	static const PxRigidDynamic* castPxsRigidBodyToPxRigidDynamic(const PxsRigidBody* rigidBody);
	static const NpArticulationReducedCoordinate* castFeatherstoneToNpArticulation(const Dy::FeatherstoneArticulation* const featherstone);
	void setDofOffsetVec(PxArray<PxU32>& dofStarts, PxU32 nbLinks, const NpArticulationLink* const * npLinks);
	void streamJointValues(const PxArticulationGPUAPIWriteType::Enum dataType, const Dy::FeatherstoneArticulation* const * articulations, PxReal* realsDataVec, const PxArticulationGPUIndex* nodeIndices,
		PxU32 nbArticulations, PxU32 maxLinks, PxU32 maxSubElementsInBlock);
	NpScene& mNpScene;
	PxArray<PxU32> mDofStarts;
};

}

#endif
#endif
