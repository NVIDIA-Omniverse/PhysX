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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXS_HEAP_MEMORY_ALLOCATOR_H
#define PXS_HEAP_MEMORY_ALLOCATOR_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
	struct PxsHeapStats
	{
		enum Enum
		{
			eOTHER = 0,
			eBROADPHASE,
			eNARROWPHASE,
			eSOLVER,
			eARTICULATION,
			eSIMULATION,
			eSIMULATION_ARTICULATION,
			eSIMULATION_PARTICLES,
			eSIMULATION_SOFTBODY,
			eSIMULATION_FEMCLOTH,
			eSIMULATION_HAIRSYSTEM,
			eSHARED_PARTICLES,
			eSHARED_SOFTBODY,
			eSHARED_FEMCLOTH,
			eSHARED_HAIRSYSTEM,
			eHEAPSTATS_COUNT
		};

		PxU64 stats[eHEAPSTATS_COUNT];

		PxsHeapStats()
		{
			for (PxU32 i = 0; i < eHEAPSTATS_COUNT; i++)
			{
				stats[i] = 0;
			}
		}
	};

	// PT: TODO: consider dropping this class
	class PxsHeapMemoryAllocator : public PxVirtualAllocatorCallback, public PxUserAllocated
	{
	public:
		virtual ~PxsHeapMemoryAllocator(){}

		// PxVirtualAllocatorCallback
		//virtual void* allocate(size_t size, int group, const char* file, int line) = 0;
		//virtual void deallocate(void* ptr) = 0;
		//~PxVirtualAllocatorCallback
	};

	class PxsHeapMemoryAllocatorManager : public PxUserAllocated
	{
	public:
		virtual ~PxsHeapMemoryAllocatorManager() {}

		virtual PxU64 getDeviceMemorySize() const = 0;
		virtual PxsHeapStats getDeviceHeapStats() const = 0;
		virtual void flushDeferredDeallocs() = 0;

		PxsHeapMemoryAllocator* mMappedMemoryAllocators;
	};
}

#endif
