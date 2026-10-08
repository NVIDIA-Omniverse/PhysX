// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "Base.h"

namespace snippetvehicle
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

}//namespace snippetvehicle
