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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#include "NvBlastAtomic.h"

#include <string.h>
#include <stdlib.h>


namespace Nv
{
namespace Blast
{


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Windows Implementation
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if NV_WINDOWS_FAMILY

#include "NvBlastIncludeWindows.h"

int32_t atomicIncrement(volatile int32_t* val)
{
    return (int32_t)InterlockedIncrement((volatile LONG*)val);
}

int32_t atomicDecrement(volatile int32_t* val)
{
    return (int32_t)InterlockedDecrement((volatile LONG*)val);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Unix Implementation
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#elif(NV_UNIX_FAMILY)

int32_t atomicIncrement(volatile int32_t* val)
{
    return __sync_add_and_fetch(val, 1);
}

int32_t atomicDecrement(volatile int32_t* val)
{
    return __sync_sub_and_fetch(val, 1);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                              Unsupported Platforms
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#else

#error "Platform not supported!"

#endif


} // namespace Blast
} // namespace Nv

