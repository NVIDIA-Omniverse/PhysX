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


#include "NvBlastChunkDTO.h"
#include "NvBlastAssert.h"

namespace Nv
{
namespace Blast
{

bool NvBlastChunkDTO::serialize(Nv::Blast::Serialization::NvBlastChunk::Builder builder, const NvBlastChunk* poco)
{
    NVBLAST_ASSERT(poco != nullptr);

    kj::ArrayPtr<const float> centArray(poco->centroid, 3);
    builder.setCentroid(centArray);

    builder.setVolume(poco->volume);

    builder.setParentChunkIndex(poco->parentChunkIndex);
    builder.setFirstChildIndex(poco->firstChildIndex);
    builder.setChildIndexStop(poco->childIndexStop);
    builder.setUserData(poco->userData);

    return true;
}


NvBlastChunk* NvBlastChunkDTO::deserialize(Nv::Blast::Serialization::NvBlastChunk::Reader reader)
{
    //FIXME
    NV_UNUSED(reader);

    return nullptr;
}


bool NvBlastChunkDTO::deserializeInto(Nv::Blast::Serialization::NvBlastChunk::Reader reader, NvBlastChunk* target)
{
    NVBLAST_ASSERT(target != nullptr);

    auto readerCentroid = reader.getCentroid();
    target->centroid[0] = readerCentroid[0];
    target->centroid[1] = readerCentroid[1];
    target->centroid[2] = readerCentroid[2];

    target->childIndexStop = reader.getChildIndexStop();
    target->firstChildIndex = reader.getFirstChildIndex();
    target->parentChunkIndex = reader.getParentChunkIndex();
    target->userData = reader.getUserData();
    target->volume = reader.getVolume();

    return true;
}

}   // namespace Blast
}   // namespace Nv
