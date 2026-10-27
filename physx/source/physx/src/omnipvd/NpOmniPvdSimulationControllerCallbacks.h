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
	virtual void processRigidDynamicSet(const PxsRigidBody* const * rigids, const void* dataVec, const PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements) PX_OVERRIDE;
	virtual void processArticulationSet(const Dy::FeatherstoneArticulation* const * articulations, const void* dataVec, const PxArticulationGPUIndex* nodeIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements,
			PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments) PX_OVERRIDE;
private:
	const PxRigidDynamic* castPxsRigidBodyToPxRigidDynamic(const PxsRigidBody* rigidBody);
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