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

#ifndef __CUDA_KERNEL_WRANGLER__
#define __CUDA_KERNEL_WRANGLER__

#include "foundation/PxPreprocessor.h"

// Make this header is safe for inclusion in headers that are shared with device code.
#if !PX_CUDA_COMPILER

#include "foundation/PxUserAllocated.h"
#include "foundation/PxArray.h"

#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdisabled-macro-expansion"
#endif
#include <cuda.h>
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif

namespace physx
{
	class PxCudaContextManager;
	class PxCudaContext;

class KernelWrangler : public PxUserAllocated
{
	PX_NOCOPY(KernelWrangler)
public:
	KernelWrangler(PxCudaContextManager& cudaContextManager, PxErrorCallback& errorCallback, const char** funcNames, uint16_t numFuncs);
	virtual ~KernelWrangler() {}

	PX_FORCE_INLINE	CUfunction getCuFunction(uint16_t funcIndex) const
	{
		CUfunction func = mCuFunctions[ funcIndex ];
		PX_ASSERT(func);
		return func;
	}

	const char* getCuFunctionName(uint16_t funcIndex) const;

	PX_FORCE_INLINE	bool hadError() const { return mError; }

protected:
	bool					mError;
	const char**			mKernelNames;
	PxArray<CUfunction>		mCuFunctions;
	PxCudaContextManager&	mCudaContextManager;
	PxCudaContext*			mCudaContext;
	PxErrorCallback&		mErrorCallback;
};

}

#endif

#endif
