// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef __TRI32DATA_H__
#define __TRI32DATA_H__

struct Triangle32Data
{
	PxU32	v[3];

	Triangle32Data() {}

	Triangle32Data(PxU32 v0,PxU32 v1,PxU32 v2)
	{
		v[0]=v0;
		v[1]=v1;
		v[2]=v2;
	}
};

struct Triangle32DataPad
{
	PxU32	v[3];
	PxU32	pad;

	PX_CUDA_CALLABLE Triangle32DataPad() {}

	PX_CUDA_CALLABLE Triangle32DataPad(PxU32 v0, PxU32 v1, PxU32 v2, PxU32 v3)
	{
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
		pad = v3;
	}

	PX_CUDA_CALLABLE Triangle32DataPad(PxU32 v0, PxU32 v1, PxU32 v2)
	{
		v[0] = v0;
		v[1] = v1;
		v[2] = v2;
		pad = 0;
	}

	PX_CUDA_CALLABLE Triangle32DataPad(Triangle32Data t)
	{
		v[0] = t.v[0];
		v[1] = t.v[1];
		v[2] = t.v[2];
		pad = 0;
	}

	PX_CUDA_CALLABLE Triangle32DataPad(Triangle32Data t, PxU32 ipad = 0)
	{
		v[0] = t.v[0];
		v[1] = t.v[1];
		v[2] = t.v[2];
		pad = ipad;
	}
};

#endif
