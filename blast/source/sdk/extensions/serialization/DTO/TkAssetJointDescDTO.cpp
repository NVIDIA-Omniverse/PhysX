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


#include "TkAssetJointDescDTO.h"
#include "NvVec3DTO.h"


namespace Nv
{
namespace Blast
{

bool TkAssetJointDescDTO::serialize(Nv::Blast::Serialization::TkAssetJointDesc::Builder builder, const Nv::Blast::TkAssetJointDesc * poco)
{
    kj::ArrayPtr<const uint32_t> nodeIndices(poco->nodeIndices, 2);
    builder.setNodeIndices(nodeIndices);
    builder.initAttachPositions(2);
    
    for (int i = 0; i < 2; i++)
    {
        NvVec3DTO::serialize(builder.getAttachPositions()[i], &poco->attachPositions[i]);
    }

    return true;
}


Nv::Blast::TkAssetJointDesc* TkAssetJointDescDTO::deserialize(Nv::Blast::Serialization::TkAssetJointDesc::Reader reader)
{
    //TODO: Allocate with ExtContent and return

    NV_UNUSED(reader);

    return nullptr;
}


bool TkAssetJointDescDTO::deserializeInto(Nv::Blast::Serialization::TkAssetJointDesc::Reader reader, Nv::Blast::TkAssetJointDesc * poco)
{
    auto readerAttachPositions = reader.getAttachPositions();
    NvVec3DTO::deserializeInto(readerAttachPositions[0], &poco->attachPositions[0]);
    NvVec3DTO::deserializeInto(readerAttachPositions[1], &poco->attachPositions[1]);

    auto readerNodeIndices = reader.getNodeIndices();
    poco->nodeIndices[0] = readerNodeIndices[0];
    poco->nodeIndices[1] = readerNodeIndices[1];

    return true;
}

}   // namespace Blast
}   // namespace Nv
