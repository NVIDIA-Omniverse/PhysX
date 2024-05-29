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


#include "NvBlastExtSerializationInternal.h"
#include "NvBlastExtTkSerialization.h"
#include "NvBlastExtTkSerializerCAPN.h"
#include "NvBlastExtTkSerializerRAW.h"


namespace Nv
{
namespace Blast
{

TkFramework* sExtTkSerializerFramework = nullptr;


class ExtTkSerializerAsset_CPNB : public ExtSerializer
{
public:
    ExtSerializerBoilerplate("TkAsset_CPNB", "Blast high-level asset (Nv::Blast::TkAsset) serialization using Cap'n Proto binary format.", TkObjectTypeID::Asset, ExtSerialization::EncodingID::CapnProtoBinary);
    ExtSerializerDefaultFactoryAndRelease(ExtTkSerializerAsset_CPNB);

    virtual void* deserializeFromBuffer(const void* buffer, uint64_t size) override
    {
        return ExtSerializationCAPN<TkAsset, Serialization::TkAsset::Reader, Serialization::TkAsset::Builder>::deserializeFromBuffer(reinterpret_cast<const unsigned char*>(buffer), size);
    }

    virtual uint64_t serializeIntoBuffer(void*& buffer, ExtSerialization::BufferProvider& bufferProvider, const void* object, uint64_t offset = 0) override
    {
        uint64_t usedSize;
        if (!ExtSerializationCAPN<TkAsset, Serialization::TkAsset::Reader, Serialization::TkAsset::Builder>::serializeIntoBuffer(reinterpret_cast<const TkAsset*>(object),
            reinterpret_cast<unsigned char*&>(buffer), usedSize, &bufferProvider, offset))
        {
            return 0;
        }
        return usedSize;
    }
};


class ExTkSerializerAsset_RAW : public ExtSerializer
{
public:
    ExtSerializerBoilerplate("TkAsset_RAW", "Blast high-level asset (Nv::Blast::TkAsset) serialization using raw memory format.", TkObjectTypeID::Asset, ExtSerialization::EncodingID::RawBinary);
    ExtSerializerDefaultFactoryAndRelease(ExTkSerializerAsset_RAW);
    ExtSerializerReadOnly(ExTkSerializerAsset_RAW);

    virtual void* deserializeFromBuffer(const void* buffer, uint64_t size) override
    {
        ExtIStream stream(buffer, size);
        return deserializeTkAsset(stream, *sExtTkSerializerFramework);
    }
};

}   // namespace Blast
}   // namespace Nv


///////////////////////////////////////


size_t NvBlastExtTkSerializerLoadSet(Nv::Blast::TkFramework& framework, Nv::Blast::ExtSerialization& serialization)
{
    Nv::Blast::sExtTkSerializerFramework = &framework;

    Nv::Blast::ExtSerializer* (*factories[])() =
    {
        Nv::Blast::ExtTkSerializerAsset_CPNB::create,
        Nv::Blast::ExTkSerializerAsset_RAW::create
    };

    return Nv::Blast::ExtSerializationLoadSet(static_cast<Nv::Blast::ExtSerializationInternal&>(serialization), factories);
}


uint64_t NvBlastExtSerializationSerializeTkAssetIntoBuffer(void*& buffer, Nv::Blast::ExtSerialization& serialization, const Nv::Blast::TkAsset* asset)
{
    return serialization.serializeIntoBuffer(buffer, asset, Nv::Blast::TkObjectTypeID::Asset);
}
