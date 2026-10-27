// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __TRIANGLE_COMMON_CUH__
#define __TRIANGLE_COMMON_CUH__

#define TRI_FEATURE_IDX		0x7FffFFff
#define EDGE_FEATURE_IDX	0x80000000

namespace physx
{

static const PxU32 BOUNDARY = 0xffffffff;
static const PxU32 NONCONVEX_FLAG = 0x80000000;

__device__ static bool isEdgeNonconvex(PxU32 adjEdgeIndex)
{
	return (adjEdgeIndex != BOUNDARY) && (adjEdgeIndex & NONCONVEX_FLAG);
}

} // namespace physx

#endif