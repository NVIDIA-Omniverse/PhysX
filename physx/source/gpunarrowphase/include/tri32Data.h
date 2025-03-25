// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.


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
