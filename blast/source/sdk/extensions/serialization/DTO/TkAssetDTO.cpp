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


#include "TkAssetDTO.h"
#include "AssetDTO.h"
#include "TkAssetJointDescDTO.h"
#include <vector>
#include "NvBlastTkFramework.h"
#include "NvBlastGlobals.h"


namespace Nv
{
namespace Blast
{

extern TkFramework* sExtTkSerializerFramework;


bool TkAssetDTO::serialize(Nv::Blast::Serialization::TkAsset::Builder builder, const Nv::Blast::TkAsset * poco)
{
    const Asset* assetLL = reinterpret_cast<const Nv::Blast::Asset*>(poco->getAssetLL());

    Nv::Blast::AssetDTO::serialize(builder.getAssetLL(), assetLL);

    uint32_t jointDescCount = poco->getJointDescCount();

    capnp::List<Nv::Blast::Serialization::TkAssetJointDesc>::Builder jointDescs = builder.initJointDescs(jointDescCount);

    for (uint32_t i = 0; i < jointDescCount; i++)
    {
        TkAssetJointDescDTO::serialize(jointDescs[i], &poco->getJointDescs()[i]);
    }

    return true;
}


Nv::Blast::TkAsset* TkAssetDTO::deserialize(Nv::Blast::Serialization::TkAsset::Reader reader)
{
    const NvBlastAsset* assetLL = reinterpret_cast<const NvBlastAsset*>(AssetDTO::deserialize(reader.getAssetLL()));

    std::vector<Nv::Blast::TkAssetJointDesc> jointDescs;

    const uint32_t jointDescCount = reader.getJointDescs().size();
    jointDescs.resize(jointDescCount);
    auto readerJointDescs = reader.getJointDescs();
    for (uint32_t i = 0; i < jointDescCount; i++)
    {
        TkAssetJointDescDTO::deserializeInto(readerJointDescs[i], &jointDescs[i]);
    }

    // Make sure to set ownsAsset to true - this is serialization and no one else owns it.
    Nv::Blast::TkAsset* asset = NvBlastTkFrameworkGet()->createAsset(assetLL, jointDescs.data(), jointDescCount, true);

    return asset;
}


bool TkAssetDTO::deserializeInto(Nv::Blast::Serialization::TkAsset::Reader reader, Nv::Blast::TkAsset * poco)
{
    NV_UNUSED(reader);
    poco = nullptr;
    // NOTE: Because of the way TkAsset is currently structured, this won't work.
    return false;
}

}   // namespace Blast
}   // namespace Nv
