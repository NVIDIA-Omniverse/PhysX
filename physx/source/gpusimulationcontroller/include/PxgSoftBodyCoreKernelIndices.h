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

#ifndef PXG_SOFTBODY_CORE_KERNEL_INDICES_H
#define PXG_SOFTBODY_CORE_KERNEL_INDICES_H

namespace physx
{

	struct PxgSoftBodyKernelBlockDim
	{
		enum
		{
			SB_PREINTEGRATION = 1024,
			SB_REFIT = 256,
			SB_INTERNALSOLVE = 256,
			SB_UPDATEROTATION = 256,
			SB_SOLVETETRA = 64,
			SB_SOLVETETRA_LOW = 32,
			SB_REORDERCONTACTS = 256,
			SB_ACCUMULATE_DELTA = 512,
		};
	};

	struct PxgSoftBodyKernelGridDim
	{
		enum
		{
			SB_REFIT = 32,
			SB_SBMIDPHASE = 1024,
			SB_SBCG = 1024,
			SB_MESHCG = 1024,
			SB_HFCG = 1024,
			SB_UPDATEROTATION = 1024,
			SB_SOLVETETRA = 64,
			SB_REORDERCONTACTS = 1024,
			SB_ACCUMULATE_DELTA = 32,
		};
	};


}

#endif
