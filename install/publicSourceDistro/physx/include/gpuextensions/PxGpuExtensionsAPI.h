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

#ifndef PX_GPU_EXTENSIONS_API_H
#define PX_GPU_EXTENSIONS_API_H
/** \addtogroup extensions
  @{
*/

#include "foundation/PxPreprocessor.h"
#include "cudamanager/PxCudaContextManager.h"

/** \brief Initialize the PhysXGpuExtensions library.

This should be called before calling any functions or methods in the gpu extensions which may launch cuda kernels.

\param cudaContextManager A cuda context manager

@see PxCloseGpuExtensions
*/
PX_C_EXPORT bool PX_CALL_CONV PxInitGpuExtensions(physx::PxCudaContextManager& cudaContextManager);

/** \brief Shut down the PhysXGpuExtensions library.

This function should be called to cleanly shut down the PhysXGpuExtensions library before application exit.

\note This function is required to be called to release kernel manager usage.

@see PxInitGpuExtensions
*/
PX_C_EXPORT void PX_CALL_CONV PxCloseGpuExtensions();

/** @} */
#endif