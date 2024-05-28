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

#include "PxDirectGPUAPI.h"
#include "foundation/PxFoundation.h"

#include "NpDirectGPUAPI.h"
#include "NpBase.h"

using namespace physx;

PX_IMPLEMENT_OUTPUT_ERROR

NpDirectGPUAPI::NpDirectGPUAPI(NpScene& scene) :
	mNpScene(scene)
{
}

bool NpDirectGPUAPI::getRigidDynamicData(void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices, PxRigidDynamicGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) const
{
	if (mNpScene.isAPIWriteForbidden())
		return NP_API_READ_WRITE_ERROR_MSG("PxDirectGPUAPI::getRigidDynamicData(): not allowed while simulation is running. Call will be ignored.");

	if (!mNpScene.isDirectGPUAPIInitialized())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::getRigidDynamicData(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");

	if (!data || !gpuIndices)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::getRigidDynamicData(): data and/or gpuIndices has to be valid pointer.");

	const float elapsedTime = mNpScene.getElapsedTime();
	const float oneOverDt = elapsedTime != 0.0f ? 1.0f/elapsedTime : 0.0f;

	return mNpScene.getScScene().getSimulationController()->getRigidDynamicData(data, gpuIndices, dataType, nbElements, oneOverDt, startEvent, finishEvent);
}

bool NpDirectGPUAPI::setRigidDynamicData(const void* PX_RESTRICT data, const PxRigidDynamicGPUIndex* PX_RESTRICT gpuIndices, PxRigidDynamicGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent)
{
	if (mNpScene.isAPIWriteForbidden())
		return NP_API_READ_WRITE_ERROR_MSG("PxDirectGPUAPI::setRigidDynamicData(): not allowed while simulation is running. Call will be ignored.");

	if (!mNpScene.isDirectGPUAPIInitialized())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::setRigidDynamicData(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");

	if (!data || !gpuIndices)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::setRigidDynamicData(): data and/or gpuIndices has to be valid pointer.");

	return mNpScene.getScScene().getSimulationController()->setRigidDynamicData(data, gpuIndices, dataType, nbElements, startEvent, finishEvent);
}

bool NpDirectGPUAPI::getArticulationData(void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxArticulationGPUAPIReadType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent) const
{
	if (mNpScene.isAPIWriteForbidden())
		return NP_API_READ_WRITE_ERROR_MSG("PxDirectGPUAPI::getArticulationData(): not allowed while simulation is running. Call will be ignored.");

	if (!mNpScene.isDirectGPUAPIInitialized())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::getArticulationData(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");

	if (!data || !gpuIndices)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::getArticulationData(): data and/or gpuIndices has to be valid pointer.");

	return mNpScene.getScScene().getSimulationController()->getArticulationData(data, gpuIndices, dataType, nbElements, startEvent, finishEvent);
}

bool NpDirectGPUAPI::setArticulationData(const void* PX_RESTRICT data, const PxArticulationGPUIndex* PX_RESTRICT gpuIndices, PxArticulationGPUAPIWriteType::Enum dataType, PxU32 nbElements, CUevent startEvent, CUevent finishEvent)
{
	if (mNpScene.isAPIWriteForbidden())
		return NP_API_READ_WRITE_ERROR_MSG("PxDirectGPUAPI::setArticulationData(): not allowed while simulation is running. Call will be ignored.");

	if (!mNpScene.isDirectGPUAPIInitialized())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::setArticulationData(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");

	if (!data || !gpuIndices)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::setArticulationData(): data and/or gpuIndices has to be valid pointer.");

	return mNpScene.getScScene().getSimulationController()->setArticulationData(data, gpuIndices, dataType, nbElements, startEvent, finishEvent);
}

bool NpDirectGPUAPI::computeArticulationData(void* data, const PxArticulationGPUIndex* gpuIndices, PxArticulationGPUAPIComputeType::Enum operation, PxU32 nbElements, CUevent startEvent, CUevent finishEvent)
{
	if (mNpScene.isAPIWriteForbidden())
		return NP_API_READ_WRITE_ERROR_MSG("PxDirectGPUAPI::computeArticulationData(): not allowed while simulation is running. Call will be ignored.");

	if (!mNpScene.isDirectGPUAPIInitialized())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::computeArticulationData(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");

	// PT: data can be null for updateKinematics
	if (/*!data ||*/ !gpuIndices)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::computeArticulationData(): data and/or gpuIndices has to be valid pointer.");

	return mNpScene.getScScene().getSimulationController()->computeArticulationData(data, gpuIndices, operation, nbElements, startEvent, finishEvent);
}

bool NpDirectGPUAPI::copyContactData(void* PX_RESTRICT data, PxU32* PX_RESTRICT numContactPairs, PxU32 maxPairs, CUevent startEvent, CUevent finishEvent) const
{
	if (mNpScene.isAPIWriteForbidden())
		return NP_API_READ_WRITE_ERROR_MSG("PxDirectGPUAPI::copyContactData(): not allowed while simulation is running. Call will be ignored.");

	if (!mNpScene.isDirectGPUAPIInitialized())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::copyContactData(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");

	if (!data || !numContactPairs)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::copyContactData(): data and/or numContactPairs has to be valid pointer.");

	return mNpScene.getScScene().getSimulationController()->copyContactData(data, numContactPairs, maxPairs, startEvent, finishEvent);
}

bool NpDirectGPUAPI::evaluateSDFDistances(PxVec4* PX_RESTRICT localGradientAndSDFConcatenated, const PxShapeGPUIndex* PX_RESTRICT gpuIndices, const PxVec4* PX_RESTRICT localSamplePointsConcatenated, const PxU32* PX_RESTRICT samplePointCountPerShape, PxU32 nbElements, PxU32 maxPointCount, CUevent startEvent, CUevent finishEvent) const
{
	if (mNpScene.isAPIWriteForbidden())
		return NP_API_READ_WRITE_ERROR_MSG("PxDirectGPUAPI::evaluateSDFDistances(): not allowed while simulation is running. Call will be ignored.");

	if (!mNpScene.isDirectGPUAPIInitialized())
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::evaluateSDFDistances(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");

	if (!localGradientAndSDFConcatenated || !gpuIndices || !localSamplePointsConcatenated || !samplePointCountPerShape)
		return outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::evaluateSDFDistances(): pointer arguments have to be valid pointers.");

	return mNpScene.getScScene().getSimulationController()->evaluateSDFDistances(localGradientAndSDFConcatenated, gpuIndices, localSamplePointsConcatenated, samplePointCountPerShape, nbElements, maxPointCount, startEvent, finishEvent);
}

PxArticulationGPUAPIMaxCounts NpDirectGPUAPI::getArticulationGPUAPIMaxCounts() const
{
	if (!mNpScene.isDirectGPUAPIInitialized())
	{
		outputError<PxErrorCode::eINVALID_OPERATION>(__LINE__, "PxDirectGPUAPI::getArticulationGPUAPIMaxCounts(): it is illegal to call this function if the scene is not configured for direct-GPU access or the direct-GPU API has not been initialized yet.");
		return PxArticulationGPUAPIMaxCounts();
	}

	return mNpScene.getScScene().getSimulationController()->getArticulationGPUAPIMaxCounts();
}
