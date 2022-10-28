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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_BV4_SETTINGS_H
#define GU_BV4_SETTINGS_H

	// PT: "BV4" ported from "Opcode 2.0". Available compile-time options are:
	#define GU_BV4_STACK_SIZE	256				// Default size of local stacks for non-recursive traversals.
	#define GU_BV4_PRECOMPUTED_NODE_SORT		// Use node sorting or not. This should probably always be enabled.
//	#define GU_BV4_QUANTIZED_TREE				// Use AABB quantization/compression or not.
	#define GU_BV4_USE_SLABS					// Use swizzled data format or not. Swizzled = faster raycasts, but slower overlaps & larger trees.
//	#define GU_BV4_COMPILE_NON_QUANTIZED_TREE	// 
	#define GU_BV4_FILL_GAPS

//#define PROFILE_MESH_COOKING
#ifdef PROFILE_MESH_COOKING
	#include <intrin.h>
	#include <stdio.h>

	struct LocalProfileZone
	{
		LocalProfileZone(const char* name)
		{
			mName = name;
			mTime = __rdtsc();
		}
		~LocalProfileZone()
		{
			mTime = __rdtsc() - mTime;
			printf("%s: %d\n", mName, unsigned int(mTime/1024));
		}

		const char*	mName;
		unsigned long long mTime;
	};
	#define GU_PROFILE_ZONE(name)	LocalProfileZone zone(name);
#else
	#define GU_PROFILE_ZONE(name)
#endif

#endif // GU_BV4_SETTINGS_H
