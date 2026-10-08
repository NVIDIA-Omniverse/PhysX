// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScArticulationTendonJointCore.h"
#include "ScArticulationTendonSim.h"

using namespace physx;

void Sc::ArticulationTendonJointCore::setCoefficient(PxArticulationAxis::Enum axis_, const PxReal coefficient_, const PxReal recipCoefficient_)
{
	axis = axis_;
	coefficient = coefficient_;
	recipCoefficient = recipCoefficient_;

	if (mTendonSim)
	{
		mTendonSim->setTendonJointCoefficient(*this, axis_, coefficient, recipCoefficient);
	}
}
