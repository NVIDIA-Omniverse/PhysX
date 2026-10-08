// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
