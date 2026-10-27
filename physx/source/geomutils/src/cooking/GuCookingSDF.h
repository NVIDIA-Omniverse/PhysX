// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_SDF_BUILDER_H
#define GU_COOKING_SDF_BUILDER_H

#include "foundation/PxArray.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
	class PxTriangleMeshDesc;

	bool buildSDF(PxTriangleMeshDesc& desc, PxArray<PxReal>& sdf, PxArray<PxU8>& sdfDataSubgrids, PxArray<PxU32>& sdfSubgridsStartSlots);
}

#endif
