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

#include "PxgBroadPhase.h"

namespace physx
{

	extern "C" void initNarrowphaseKernels0();
	extern "C" void initNarrowphaseKernels1();
	extern "C" void initNarrowphaseKernels2();
	extern "C" void initNarrowphaseKernels3();
	extern "C" void initNarrowphaseKernels4();
	extern "C" void initNarrowphaseKernels5();
	extern "C" void initNarrowphaseKernels6();
	extern "C" void initNarrowphaseKernels7();
	extern "C" void initNarrowphaseKernels8();
	extern "C" void initNarrowphaseKernels9();
	extern "C" void initNarrowphaseKernels10();
	extern "C" void initNarrowphaseKernels11();
	extern "C" void initNarrowphaseKernels12();
	extern "C" void initNarrowphaseKernels13();
	extern "C" void initNarrowphaseKernels14();
	extern "C" void initNarrowphaseKernels15();
	extern "C" void initNarrowphaseKernels16();
	extern "C" void initNarrowphaseKernels17();
	extern "C" void initNarrowphaseKernels18();
	extern "C" void initNarrowphaseKernels19();
	extern "C" void initNarrowphaseKernels20();
	extern "C" void initNarrowphaseKernels21();
	extern "C" void initNarrowphaseKernels22();
	extern "C" void initNarrowphaseKernels23();
	extern "C" void initNarrowphaseKernels24();

	void createPxgNarrowphase()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXNarrowphaseGpu linkage as Static Library!
		initNarrowphaseKernels0();
		initNarrowphaseKernels1();
		initNarrowphaseKernels2();
		initNarrowphaseKernels3();
		initNarrowphaseKernels4();
		initNarrowphaseKernels5();
		initNarrowphaseKernels6();
		initNarrowphaseKernels7();
		initNarrowphaseKernels8();
		initNarrowphaseKernels9();
		initNarrowphaseKernels10();
		initNarrowphaseKernels11();
		initNarrowphaseKernels12();
		initNarrowphaseKernels13();
		initNarrowphaseKernels14();
		initNarrowphaseKernels15();
		initNarrowphaseKernels16();
		initNarrowphaseKernels17();
		initNarrowphaseKernels18();
		initNarrowphaseKernels19();
		initNarrowphaseKernels20();
		initNarrowphaseKernels21();
		initNarrowphaseKernels22();
		initNarrowphaseKernels23();
		initNarrowphaseKernels24();
#endif
	}

}
