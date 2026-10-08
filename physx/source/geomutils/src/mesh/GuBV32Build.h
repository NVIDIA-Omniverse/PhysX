// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_BV32_BUILD_H
#define GU_BV32_BUILD_H

#include "foundation/PxSimpleTypes.h"
#include "common/PxPhysXCommonConfig.h"

#define BV32_VALIDATE	0

namespace physx
{
	namespace Gu
	{
		class BV32Tree;
		class SourceMeshBase;

		bool BuildBV32Ex(BV32Tree& tree, SourceMeshBase& mesh, float epsilon, PxU32 nbPrimitivesPerLeaf);

	} // namespace Gu
}

#endif // GU_BV32_BUILD_H
