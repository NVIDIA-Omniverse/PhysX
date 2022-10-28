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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "Base.h"

namespace snippetvehicle2
{

BaseVehicleParams BaseVehicleParams::transformAndScale
(const PxVehicleFrame& srcFrame, const PxVehicleFrame& trgFrame, const PxVehicleScale& srcScale, const PxVehicleScale& trgScale) const
{
	BaseVehicleParams r = *this;
	r.axleDescription = axleDescription;
	r.frame = trgFrame;
	r.scale = trgScale;

	r.suspensionStateCalculationParams = suspensionStateCalculationParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);

	r.brakeResponseParams[0] = brakeResponseParams[0].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.brakeResponseParams[1] = brakeResponseParams[1].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.steerResponseParams = steerResponseParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	r.ackermannParams[0] = ackermannParams[0].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);

	for (PxU32 i = 0; i < r.axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = r.axleDescription.wheelIdsInAxleOrder[i];

		r.suspensionParams[wheelId] = suspensionParams[wheelId].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
		r.suspensionComplianceParams[wheelId] = suspensionComplianceParams[wheelId].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
		r.suspensionForceParams[wheelId] = suspensionForceParams[wheelId].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);

		r.tireForceParams[wheelId] = tireForceParams[wheelId].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);

		r.wheelParams[wheelId] = wheelParams[wheelId].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	}

	r.rigidBodyParams = rigidBodyParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);

	return r;
}


bool BaseVehicle::initialize()
{
	if (!mBaseParams.isValid())
		return false;

	//Set the base state to default.
	mBaseState.setToDefault();

	return true;
}

void BaseVehicle::step(const PxReal dt, const PxVehicleSimulationContext& context)
{
	mComponentSequence.update(dt, context);
}

}//namespace snippetvehicle2
