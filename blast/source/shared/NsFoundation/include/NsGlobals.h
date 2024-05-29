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

#ifndef NV_NSFOUNDATION_NSGLOBALS_H
#define NV_NSFOUNDATION_NSGLOBALS_H

#include "NvErrors.h"

namespace nvidia
{

class NvAssertHandler;
class NvErrorCallback;
class NvAllocatorCallback;
class NvProfilerCallback;

namespace shdfnd
{

// note: it's illegal to initialize the shared foundation twice without terminating in between

NV_FOUNDATION_API void initializeSharedFoundation(uint32_t version, NvAllocatorCallback&, NvErrorCallback&);
NV_FOUNDATION_API bool sharedFoundationIsInitialized();
NV_FOUNDATION_API void terminateSharedFoundation();

// number of times foundation has been init'd. 0 means never initialized, so if we wrap we go from UINT32_MAX to 1. Used
// for things that happen at most once (e.g. some warnings)
NV_FOUNDATION_API uint32_t getInitializationCount();

NV_FOUNDATION_API NvAllocatorCallback& getAllocator();
NV_FOUNDATION_API NvErrorCallback& getErrorCallback();

// on some platforms (notably 360) the CRT does non-recoverable allocations when asked for type names. Hence
// we provide a mechanism to disable this capability
NV_FOUNDATION_API void setReflectionAllocatorReportsNames(bool val);
NV_FOUNDATION_API bool getReflectionAllocatorReportsNames();


NV_FOUNDATION_API NvProfilerCallback *getProfilerCallback();
NV_FOUNDATION_API void setProfilerCallback(NvProfilerCallback *profiler);


}
}

#endif // #ifndef NV_NSFOUNDATION_NSGLOBALS_H
