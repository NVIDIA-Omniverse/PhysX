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

#ifndef PXG_CONVEX_CONVEX_SHAPE_H
#define PXG_CONVEX_CONVEX_SHAPE_H

#include "geometry/PxMeshScale.h"

#define PXG_MAX_PCM_CONTACTS 4

namespace physx
{
	template<PxU32 byteSize> 
	class FlexiblePad
	{
		PxU8							pad[byteSize];
	};

	template<> 
	class FlexiblePad<0>
	{
	};

	//ML: PxgShape don't need to have contactOffset because we will dma a separated contact offset array later
	struct PX_ALIGN_PREFIX(16) PxgShape
	{
		PxMeshScale						scale;									//28
		PxU32							materialIndex;							//32
		size_t							hullOrMeshPtr;							//36 or 40

		PxU32							type;									//40 or 44
		PxU32							particleOrSoftbodyId;					//44 or 48		

		//FlexiblePad<16 - sizeof(size_t) - sizeof(PxU32)>	pad;			//48
#if !PX_P64_FAMILY
		PxU32    pad0;
#endif
	}
	PX_ALIGN_SUFFIX(16);

	PX_COMPILE_TIME_ASSERT(sizeof(PxgShape) == 48);
}

#endif
