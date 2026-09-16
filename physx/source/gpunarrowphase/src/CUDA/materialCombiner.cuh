// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __MATERIAL_COMBINER_CUH__
#define __MATERIAL_COMBINER_CUH__

#include "PxsMaterialCombiner.h"

namespace physx
{

__device__ static __forceinline__
void combineMaterials(const PxsMaterialData* PX_RESTRICT materials, PxU16 index0, PxU16 index1,
	PxU32& combinedFlags,
	PxReal& combinedStaticFriction,
	PxReal& combinedDynamicFriction,
	PxReal& combinedRestitution,
	PxReal& combinedDamping
)
{
	const PxsMaterialData& mat0 = materials[index0];
	const PxsMaterialData& mat1 = materials[index1];

	PxsCombineMaterials(mat0, mat1,
		combinedStaticFriction, combinedDynamicFriction, 
		combinedRestitution, combinedFlags, combinedDamping);
}

} // namespace physx

#endif