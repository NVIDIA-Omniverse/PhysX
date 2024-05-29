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


#ifndef NVBLASTTKGUID_H
#define NVBLASTTKGUID_H

#include "NvPreprocessor.h"

#if NV_WINDOWS_FAMILY
#include <rpc.h>
#else
//#include <uuid/uuid.h>
#include "NvBlastTime.h"
#endif

#include "NsHash.h"

namespace Nv
{
namespace Blast
{

#if NV_WINDOWS_FAMILY

NV_INLINE NvBlastID TkGenerateGUID(void* ptr)
{
    NV_UNUSED(ptr);

    NV_COMPILE_TIME_ASSERT(sizeof(UUID) == sizeof(NvBlastID));

    NvBlastID guid;
    UuidCreate(reinterpret_cast<UUID*>(&guid));

    return guid;
}

#else

NV_INLINE NvBlastID TkGenerateGUID(void* ptr)
{
//  NV_COMPILE_TIME_ASSERT(sizeof(uuid_t) == sizeof(NvBlastID));
    Time time;

    NvBlastID guid;
    //  uuid_generate_random(reinterpret_cast<uuid_t&>(guid));

    *reinterpret_cast<uint64_t*>(guid.data) = reinterpret_cast<uintptr_t>(ptr);
    *reinterpret_cast<int64_t*>(guid.data + 8) = time.getLastTickCount();

    return guid;
}

#endif


/**
Compares two NvBlastIDs.

\param[in]  id1 A pointer to the first id to compare.
\param[in]  id2 A pointer to the second id to compare.

\return true iff ids are equal.
*/
NV_INLINE bool TkGUIDsEqual(const NvBlastID* id1, const NvBlastID* id2)
{
    return !memcmp(id1, id2, sizeof(NvBlastID));
}


/**
Clears an NvBlastID (sets all of its fields to zero).

\param[out] id  A pointer to the ID to clear.
*/
NV_INLINE void TkGUIDReset(NvBlastID* id)
{
    memset(id, 0, sizeof(NvBlastID));
}


/**
Tests an NvBlastID to determine if it's zeroed.  After calling TkGUIDReset
on an ID, passing it to this function will return a value of true.

\param[in]  id  A pointer to the ID to test.
*/
NV_INLINE bool TkGUIDIsZero(const NvBlastID* id)
{
    return *reinterpret_cast<const uint64_t*>(&id->data[0]) == 0 && *reinterpret_cast<const uint64_t*>(&id->data[8]) == 0;
}

} // namespace Blast
} // namespace Nv


namespace nvidia
{
namespace shdfnd
{

// hash specialization for NvBlastID
template <>
struct Hash<NvBlastID>
{
    uint32_t operator()(const NvBlastID& k) const
    {
        // "DJB" string hash
        uint32_t h = 5381;
        for (uint32_t i = 0; i < sizeof(k.data) / sizeof(k.data[0]); ++i)
            h = ((h << 5) + h) ^ uint32_t(k.data[i]);
        return h;
    }
    bool equal(const NvBlastID& k0, const NvBlastID& k1) const
    {
        return Nv::Blast::TkGUIDsEqual(&k0, &k1);
    }
};

} // namespace shdfnd
} // namespace nvidia


#endif // #ifndef NVBLASTTKGUID_H
