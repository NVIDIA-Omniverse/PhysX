// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_FEATURE_CODE_H
#define GU_FEATURE_CODE_H

namespace physx
{
namespace Gu
{
	enum FeatureCode
	{
		FC_VERTEX0,
		FC_VERTEX1,
		FC_VERTEX2,
		FC_EDGE01,
		FC_EDGE12,
		FC_EDGE20,
		FC_FACE,

		FC_UNDEFINED
	};

	bool		selectNormal(PxU8 data, PxReal u, PxReal v);
}
}

#endif
