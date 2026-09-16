// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXC_NP_BATCH_H
#define PXC_NP_BATCH_H

#include "PxPhysXConfig.h"

namespace physx
{
	struct PxcNpWorkUnit;
	class PxcNpThreadContext;
	struct PxsContactManagerOutput;

	namespace Gu
	{
		struct Cache;
	}

	void PxcDiscreteNarrowPhase(PxcNpThreadContext& context, const PxcNpWorkUnit& cmInput, Gu::Cache& cache, PxsContactManagerOutput& output, PxU64 contextID);
	void PxcDiscreteNarrowPhasePCM(PxcNpThreadContext& context, const PxcNpWorkUnit& cmInput, Gu::Cache& cache, PxsContactManagerOutput& output, PxU64 contextID);
}

#endif
