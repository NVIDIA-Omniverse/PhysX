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

#ifndef NP_OMNI_PVD_SIMULATION_CONTROLLER_CALLBACKS_H
#define NP_OMNI_PVD_SIMULATION_CONTROLLER_CALLBACKS_H

#if PX_SUPPORT_OMNI_PVD

#include "PxsSimulationController.h"

namespace physx
{
class NpArticulationLink;
class NpArticulationReducedCoordinate;
class NpScene;

class NpOmniPvdSimulationControllerCallbacks : public PxsSimulationControllerOVDCallbacks
{  
public:
	NpOmniPvdSimulationControllerCallbacks(NpScene& scene);
	virtual void processRigidDynamicSet(PxsRigidBody** rigids, void* dataVec, PxRigidDynamicGPUIndex* gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements) PX_OVERRIDE;
	virtual void processArticulationSet(Dy::FeatherstoneArticulation** articulations, void* dataVec, PxArticulationGPUIndex* nodeIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements,
			PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments) PX_OVERRIDE;
private:
	const PxRigidDynamic* castPxsRigidBodyToPxRigidDynamic(PxsRigidBody* const rigidBody);
	static const NpArticulationReducedCoordinate* castFeatherstoneToNpArticulation(Dy::FeatherstoneArticulation* const featherstone);
	PxU32 getArticulationDataMaxSubElementsNb(PxArticulationGPUAPIWriteType::Enum dataType,
		PxU32 maxLinks, PxU32 maxDofs, PxU32 maxFixedTendons, PxU32 maxTendonJoints, PxU32 maxSpatialTendons, PxU32 maxSpatialTendonAttachments);
	void setDofOffsetVec(PxArray<PxU32>& dofStarts, PxU32 nbLinks, const NpArticulationLink* const * npLinks);
	void streamJointValues(const PxArticulationGPUAPIWriteType::Enum dataType, Dy::FeatherstoneArticulation** articulations, PxReal* realsDataVec, PxArticulationGPUIndex* nodeIndices,
		PxU32 nbArticulations, PxU32 maxLinks, PxU32 maxSubElementsInBlock);
	NpScene& mNpScene;
};

}

#endif
#endif