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

#ifndef PXG_NP_KERNEL_INDICES_H
#define PXG_NP_KERNEL_INDICES_H

#define MIDPHASE_WARPS_PER_BLOCK              2
#define NP_TRIMESH_WARPS_PER_BLOCK            2
#define CORRELATE_WARPS_PER_BLOCK             2
#define PS_MIDPHASE_COLLISION_WAPRS_PER_BLOCK 1
#define SB_REFIT_WAPRS_PER_BLOCK              32

namespace physx
{

	struct PxgNarrowPhaseBlockDims
	{
		enum
		{
			REMOVE_CONTACT_MANAGERS		= 512,
			COMPACT_LOST_FOUND_PAIRS    = 512,
			FINISH_CONTACTS    = 128,
			EARLY_OUT_KERNEL = 128,
			COLLIDE_KERNEL = 128,
			INITIALIZE_MANIFOLDS = 512,
			COMPRESS_CONTACT = 256
		};
	};

	struct PxgNarrowPhaseGridDims
	{
		enum
		{
			REMOVE_CONTACT_MANAGERS		= 32,
			COMPACT_LOST_FOUND_PAIRS    = 32,
			EARLY_OUT_KERNEL = 32,
			COLLIDE_KERNEL = 32,
			INITIALIZE_MANIFOLDS = 128,
			COMPRESS_CONTACT = 32,

		};
	};
}

#endif

