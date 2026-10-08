// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_ARTICULATION_CORE_H
#define DY_ARTICULATION_CORE_H

#include "PxArticulationReducedCoordinate.h"

namespace physx
{
	namespace Dy
	{
		struct ArticulationCore
		{
// PX_SERIALIZATION
			ArticulationCore(const PxEMPTY) : flags(PxEmpty) {}
			ArticulationCore() {}
//~PX_SERIALIZATION

			PxU16					solverIterationCounts; //KS - made a U16 so that it matches PxsRigidCore
			PxArticulationFlags		flags;
			PxReal					sleepThreshold;
			PxReal					freezeThreshold;
			PxReal					wakeCounter;
			PxU32					gpuRemapIndex;
		};
	}
}

#endif

