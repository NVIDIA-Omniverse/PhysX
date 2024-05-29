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
#include "NvBlastExtLlSerialization.h"
#include "NvBlastExtLlSerializerCAPN.h"


namespace Nv
{
namespace Blast
{

class ExtLlSerializerAsset_CPNB : public ExtSerializer
{
public:
    ExtSerializerBoilerplate("LLAsset_CPNB", "Blast low-level asset (NvBlastAsset) serialization using Cap'n Proto binary format.", LlObjectTypeID::Asset, ExtSerialization::EncodingID::CapnProtoBinary);
    ExtSerializerDefaultFactoryAndRelease(ExtLlSerializerAsset_CPNB);

    virtual void* deserializeFromBuffer(const void* buffer, uint64_t size) override
    {
        return ExtSerializationCAPN<Asset, Serialization::Asset::Reader, Serialization::Asset::Builder>::deserializeFromBuffer(reinterpret_cast<const unsigned char*>(buffer), size);
    }

    virtual uint64_t serializeIntoBuffer(void*& buffer, ExtSerialization::BufferProvider& bufferProvider, const void* object, uint64_t offset = 0) override
    {
        uint64_t usedSize;
        if (!ExtSerializationCAPN<Asset, Serialization::Asset::Reader, Serialization::Asset::Builder>::serializeIntoBuffer(reinterpret_cast<const Asset*>(object),
            reinterpret_cast<unsigned char*&>(buffer), usedSize, &bufferProvider, offset))
        {
            return 0;
        }
        return usedSize;
    }
};


class ExtLlSerializerFamily_CPNB : public ExtSerializer
{
public:
    ExtSerializerBoilerplate("LLFamily_CPNB", "Blast low-level family (NvBlastFamily) serialization using Cap'n Proto binary format.", LlObjectTypeID::Family, ExtSerialization::EncodingID::CapnProtoBinary);
    ExtSerializerDefaultFactoryAndRelease(ExtLlSerializerFamily_CPNB);

    virtual void* deserializeFromBuffer(const void* buffer, uint64_t size) override
    {
        return ExtSerializationCAPN<FamilyHeader, Serialization::Family::Reader, Serialization::Family::Builder>::deserializeFromBuffer(reinterpret_cast<const unsigned char*>(buffer), size);
    }

    virtual uint64_t serializeIntoBuffer(void*& buffer, ExtSerialization::BufferProvider& bufferProvider, const void* object, uint64_t offset = 0) override
    {
        uint64_t usedSize;
        if (!ExtSerializationCAPN<FamilyHeader, Serialization::Family::Reader, Serialization::Family::Builder>::serializeIntoBuffer(reinterpret_cast<const FamilyHeader*>(object),
            reinterpret_cast<unsigned char*&>(buffer), usedSize, &bufferProvider, offset))
        {
            return 0;
        }
        return usedSize;
    }
};


class ExtLlSerializerObject_RAW : public ExtSerializer
{
public:
    virtual void* deserializeFromBuffer(const void* buffer, uint64_t size) override
    {
        const NvBlastDataBlock* block = reinterpret_cast<const NvBlastDataBlock*>(buffer);
        if (static_cast<uint64_t>(block->size) > size)
        {
            return nullptr;
        }
        void* llobject = NVBLAST_ALLOC(block->size);
        return memcpy(llobject, block, block->size);
    }

    virtual uint64_t serializeIntoBuffer(void*& buffer, ExtSerialization::BufferProvider& bufferProvider, const void* object, uint64_t offset = 0) override
    {
        const NvBlastDataBlock* block = reinterpret_cast<const NvBlastDataBlock*>(object);
        const uint64_t size = block->size + offset;
        buffer = bufferProvider.requestBuffer(size);
        if (buffer == nullptr)
        {
            return 0;
        }
        memcpy(static_cast<char*>(buffer) + offset, object, block->size);
        return size;
    }
};


class ExtLlSerializerAsset_RAW : public ExtLlSerializerObject_RAW
{
public:
    ExtSerializerBoilerplate("LLAsset_RAW", "Blast low-level asset (NvBlastAsset) serialization using raw memory format.", LlObjectTypeID::Asset, ExtSerialization::EncodingID::RawBinary);
    ExtSerializerDefaultFactoryAndRelease(ExtLlSerializerAsset_RAW);
};


class ExtLlSerializerFamily_RAW : public ExtLlSerializerObject_RAW
{
public:
    ExtSerializerBoilerplate("LLFamily_RAW", "Blast low-level family (NvBlastFamily) serialization using raw memory format.", LlObjectTypeID::Family, ExtSerialization::EncodingID::RawBinary);
    ExtSerializerDefaultFactoryAndRelease(ExtLlSerializerFamily_RAW);
};

}   // namespace Blast
}   // namespace Nv


///////////////////////////////////////


size_t NvBlastExtLlSerializerLoadSet(Nv::Blast::ExtSerialization& serialization)
{
    Nv::Blast::ExtSerializer* (*factories[])() =
    {
        Nv::Blast::ExtLlSerializerAsset_CPNB::create,
        Nv::Blast::ExtLlSerializerAsset_RAW::create,
        Nv::Blast::ExtLlSerializerFamily_CPNB::create,
        Nv::Blast::ExtLlSerializerFamily_RAW::create
    };

    return Nv::Blast::ExtSerializationLoadSet(static_cast<Nv::Blast::ExtSerializationInternal&>(serialization), factories);
}


uint64_t NvBlastExtSerializationSerializeAssetIntoBuffer(void*& buffer, Nv::Blast::ExtSerialization& serialization, const NvBlastAsset* asset)
{
    return serialization.serializeIntoBuffer(buffer, asset, Nv::Blast::LlObjectTypeID::Asset);
}


uint64_t NvBlastExtSerializationSerializeFamilyIntoBuffer(void*& buffer, Nv::Blast::ExtSerialization& serialization, const NvBlastFamily* family)
{
    return serialization.serializeIntoBuffer(buffer, family, Nv::Blast::LlObjectTypeID::Family);
}
