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


#pragma once

#include "NvBlastExtSerializationCAPN.h"
#include "NvBlastAsset.h"
#include "NvBlastFamily.h"
#include "AssetDTO.h"
#include "FamilyDTO.h"


/**
Specializations of ExtSerializationCAPN for Blast LL
*/

namespace Nv
{
namespace Blast
{

//// Nv::Blast::Asset ////

template<>
NV_INLINE bool ExtSerializationCAPN<Asset, Serialization::Asset::Reader, Serialization::Asset::Builder>::serializeIntoBuilder(Serialization::Asset::Builder& assetBuilder, const Asset* asset)
{
    return AssetDTO::serialize(assetBuilder, asset);
}


template<>
NV_INLINE bool ExtSerializationCAPN<Asset, Serialization::Asset::Reader, Serialization::Asset::Builder>::serializeIntoMessage(capnp::MallocMessageBuilder& message, const Asset* asset)
{
    Serialization::Asset::Builder assetBuilder = message.initRoot<Serialization::Asset>();

    return serializeIntoBuilder(assetBuilder, asset);
}


template<>
NV_INLINE Asset* ExtSerializationCAPN<Asset, Serialization::Asset::Reader, Serialization::Asset::Builder>::deserializeFromStreamReader(capnp::InputStreamMessageReader &message)
{
    Serialization::Asset::Reader reader = message.getRoot<Serialization::Asset>();

    return AssetDTO::deserialize(reader);
}

//// Nv::Blast::FamilyHeader ////

template<>
NV_INLINE bool ExtSerializationCAPN<FamilyHeader, Serialization::Family::Reader, Serialization::Family::Builder>::serializeIntoBuilder(Serialization::Family::Builder& familyBuilder, const FamilyHeader* family)
{
    return FamilyDTO::serialize(familyBuilder, family);
}


template<>
NV_INLINE bool ExtSerializationCAPN<FamilyHeader, Serialization::Family::Reader, Serialization::Family::Builder>::serializeIntoMessage(capnp::MallocMessageBuilder& message, const FamilyHeader* family)
{
    Serialization::Family::Builder familyBuilder = message.initRoot<Serialization::Family>();

    return serializeIntoBuilder(familyBuilder, family);
}


template<>
NV_INLINE FamilyHeader* ExtSerializationCAPN<FamilyHeader, Serialization::Family::Reader, Serialization::Family::Builder>::deserializeFromStreamReader(capnp::InputStreamMessageReader &message)
{
    Serialization::Family::Reader reader = message.getRoot<Serialization::Family>();

    return FamilyDTO::deserialize(reader);
}

}   // namespace Blast
}   // namespace Nv
