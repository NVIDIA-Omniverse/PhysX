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
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.


#include "PxVec3DTO.h"
#include "NvBlastAssert.h"

namespace Nv
{
namespace Blast
{

bool PxVec3DTO::serialize(Nv::Blast::Serialization::PxVec3::Builder builder, const physx::PxVec3 * poco)
{
    NVBLAST_ASSERT(poco != nullptr);

    builder.setX(poco->x);
    builder.setY(poco->y);
    builder.setZ(poco->z);

    return true;
}

physx::PxVec3* PxVec3DTO::deserialize(Nv::Blast::Serialization::PxVec3::Reader reader)
{
    //TODO: Allocate using ExtContext and return
    NV_UNUSED(reader);
    return nullptr;
}

bool PxVec3DTO::deserializeInto(Nv::Blast::Serialization::PxVec3::Reader reader, physx::PxVec3* target)
{
    target->x = reader.getX();
    target->y = reader.getY();
    target->z = reader.getZ();

    return true;
}

}   // namespace Blast
}   // namespace Nv
