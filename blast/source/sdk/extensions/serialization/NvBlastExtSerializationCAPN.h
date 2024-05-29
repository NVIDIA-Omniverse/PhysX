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

#include "capnp/serialize.h"
#include "NvBlastExtInputStream.h"
#include "NvBlastExtOutputStream.h"
#include "NvBlastArray.h"
#include "NvBlastExtSerialization.h"


namespace Nv
{
namespace Blast
{

template<typename TObject, typename TSerializationReader, typename TSerializationBuilder>
class ExtSerializationCAPN
{
public:
    static TObject* deserializeFromBuffer(const unsigned char* input, uint64_t size);
    static TObject* deserializeFromStream(std::istream& inputStream);

    static uint64_t serializationBufferSize(const TObject* object);

    static bool     serializeIntoBuffer(const TObject* object, unsigned char* buffer, uint64_t maxSize, uint64_t& usedSize);
    static bool     serializeIntoBuffer(const TObject *object, unsigned char*& buffer, uint64_t& size, ExtSerialization::BufferProvider* bufferProvider = nullptr, uint64_t offset = 0);
    static bool     serializeIntoStream(const TObject* object, std::ostream& outputStream);

private:
    // Specialized
    static bool     serializeIntoBuilder(TSerializationBuilder& objectBuilder, const TObject* object);
    static bool     serializeIntoMessage(capnp::MallocMessageBuilder& message, const TObject* object);
    static TObject* deserializeFromStreamReader(capnp::InputStreamMessageReader& message);
};


template<typename TObject, typename TSerializationReader, typename TSerializationBuilder>
TObject* ExtSerializationCAPN<TObject, TSerializationReader, TSerializationBuilder>::deserializeFromBuffer(const unsigned char* input, uint64_t size)
{
    kj::ArrayPtr<const unsigned char> source(input, size);

    kj::ArrayInputStream inputStream(source);

    Nv::Blast::Array<uint64_t>::type scratch(static_cast<uint32_t>(size));
    kj::ArrayPtr<capnp::word> scratchArray((capnp::word*) scratch.begin(), size);

    capnp::InputStreamMessageReader message(inputStream, capnp::ReaderOptions(), scratchArray);

    return deserializeFromStreamReader(message);
}


template<typename TObject, typename TSerializationReader, typename TSerializationBuilder>
TObject* ExtSerializationCAPN<TObject, TSerializationReader, TSerializationBuilder>::deserializeFromStream(std::istream& inputStream)
{
    ExtInputStream readStream(inputStream);

    capnp::InputStreamMessageReader message(readStream);

    return deserializeFromStreamReader(message);
}


template<typename TObject, typename TSerializationReader, typename TSerializationBuilder>
uint64_t ExtSerializationCAPN<TObject, TSerializationReader, TSerializationBuilder>::serializationBufferSize(const TObject* object)
{
    capnp::MallocMessageBuilder message;

    bool result = serializeIntoMessage(message, object);

    if (result == false)
    {
        return 0;
    }

    return computeSerializedSizeInWords(message) * sizeof(uint64_t);
}


template<typename TObject, typename TSerializationReader, typename TSerializationBuilder>
bool ExtSerializationCAPN<TObject, TSerializationReader, TSerializationBuilder>::serializeIntoBuffer(const TObject* object, unsigned char* buffer, uint64_t maxSize, uint64_t& usedSize)
{
    capnp::MallocMessageBuilder message;

    bool result = serializeIntoMessage(message, object);

    if (result == false)
    {
        usedSize = 0;
        return false;
    }

    uint64_t messageSize = computeSerializedSizeInWords(message) * sizeof(uint64_t);

    if (maxSize < messageSize)
    {
        NVBLAST_LOG_ERROR("When attempting to serialize into an existing buffer, the provided buffer was too small.");
        usedSize = 0;
        return false;
    }

    kj::ArrayPtr<unsigned char> outputBuffer(buffer, maxSize);
    kj::ArrayOutputStream outputStream(outputBuffer);

    capnp::writeMessage(outputStream, message);

    usedSize = messageSize;
    return true;
}


template<typename TObject, typename TSerializationReader, typename TSerializationBuilder>
bool ExtSerializationCAPN<TObject, TSerializationReader, TSerializationBuilder>::serializeIntoBuffer(const TObject *object, unsigned char*& buffer, uint64_t& size, ExtSerialization::BufferProvider* bufferProvider, uint64_t offset)
{
    capnp::MallocMessageBuilder message;

    bool result = serializeIntoMessage(message, object);

    if (result == false)
    {
        buffer = nullptr;
        size = 0;
        return false;
    }

    const uint64_t blockSize = computeSerializedSizeInWords(message) * sizeof(uint64_t);

    size = blockSize + offset;

    buffer = static_cast<unsigned char *>(bufferProvider != nullptr ? bufferProvider->requestBuffer(size) : NVBLAST_ALLOC(size));

    kj::ArrayPtr<unsigned char> outputBuffer(buffer + offset, blockSize);
    kj::ArrayOutputStream outputStream(outputBuffer);

    capnp::writeMessage(outputStream, message);

    return true;
}


template<typename TObject, typename TSerializationReader, typename TSerializationBuilder>
bool ExtSerializationCAPN<TObject, TSerializationReader, TSerializationBuilder>::serializeIntoStream(const TObject* object, std::ostream& outputStream)
{
    capnp::MallocMessageBuilder message;

    bool result = serializeIntoMessage(message, object);

    if (result == false)
    {
        return false;
    }

    ExtOutputStream blastOutputStream(outputStream);

    writeMessage(blastOutputStream, message);

    return true;
}

}   // namespace Blast
}   // namespace Nv
