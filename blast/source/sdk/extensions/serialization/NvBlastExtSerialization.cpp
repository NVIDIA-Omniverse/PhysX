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


#include "NvBlastExtSerialization.h"
#include "NvBlastExtLlSerialization.h"
#include "NvBlastArray.h"
#include "NvBlastHashMap.h"
#include "NvBlastExtSerializationInternal.h"


namespace Nv
{
namespace Blast
{

class ExtSerializationImpl : public ExtSerializationInternal
{
public:
    // Default buffer provider
    class AllocBufferProvider : public ExtSerialization::BufferProvider
    {
    public:
        virtual void*   requestBuffer(size_t size) override;
    };


    ExtSerializationImpl();
    ~ExtSerializationImpl();

    // ExtSerialization interface begin
    virtual bool            setSerializationEncoding(uint32_t encodingID) override;
    virtual uint32_t        getSerializationEncoding() const override;

    virtual void            setBufferProvider(BufferProvider* bufferProvider) override;

    virtual bool            peekHeader(uint32_t* objectTypeID, uint32_t* encodingID, uint64_t* dataSize, const void* buffer, uint64_t bufferSize) override;
    virtual const void*     skipObject(uint64_t& bufferSize, const void* buffer) override;

    virtual void*           deserializeFromBuffer(const void* buffer, uint64_t size, uint32_t* objectTypeIDPtr = nullptr) override;
    virtual uint64_t        serializeIntoBuffer(void*& buffer, const void* object, uint32_t objectTypeID) override;

    virtual void            release() override;
    // ExtSerialization interface end

    // ExtSerializationInternal interface begin
    virtual bool            registerSerializer(ExtSerializer& serializer) override;
    virtual bool            unregisterSerializer(ExtSerializer& serializer) override;

    virtual ExtSerializer*  findSerializer(uint32_t objectTypeID, uint32_t encodingID) override;
    // ExtSerializationInternal interface end

private:
    char*                   writeHeaderIntoBuffer(char* buffer, uint64_t bufferSize, uint32_t objectTypeID, uint32_t encodingID, uint64_t dataSize) const;
    const char*             readHeaderFromBuffer(uint32_t* objectTypeID, uint32_t* encodingID, uint64_t* dataSize, const char* buffer, uint64_t bufferSize) const;

    //// Static data ////
    static const char*                      s_identifier;
    static const char*                      s_version;
    static AllocBufferProvider              s_defaultBufferProvider;

    //// Member data ////
    HashMap<uint64_t, ExtSerializer*>::type m_serializers;
    uint32_t                                m_serializationEncoding;
    BufferProvider*                         m_bufferProvider;
};


//////// ExtSerializationImpl static member variables ////////

/** Module identifying header.  This should never change. */
const char* ExtSerializationImpl::s_identifier = "NVidia(r) GameWorks Blast(tm) v.";

const char* ExtSerializationImpl::s_version = "1";

ExtSerializationImpl::AllocBufferProvider   ExtSerializationImpl::s_defaultBufferProvider;


//////// Local utility functions ////////

static NV_INLINE uint64_t generateKey(uint32_t objectTypeID, uint32_t encodingID)
{
    return static_cast<uint64_t>(encodingID) << 32 | static_cast<uint64_t>(objectTypeID);
}


static NV_INLINE uint64_t generateKey(const ExtSerializer& serializer)
{
    return generateKey(serializer.getObjectTypeID(), serializer.getEncodingID());
}


static NV_INLINE void writeIDToBuffer(char* buffer, uint32_t id)
{
    for (int i = 0; i < 4; ++i, id >>= 8)
    {
        *buffer++ = static_cast<char>(id & 0xFF);
    }
}


static NV_INLINE uint32_t readIDFromBuffer(const char* buffer)
{
    return NVBLAST_FOURCC(buffer[0], buffer[1], buffer[2], buffer[3]);
}


static NV_INLINE void writeU64InHexToBuffer(char* buffer, uint64_t val)
{
    for (char* curr = buffer + 16; curr-- > buffer; val >>= 4)
    {
        *curr = "0123456789ABCDEF"[val & 0xF];
    }
}


static NV_INLINE uint64_t readU64InHexFromBuffer(const char* buffer)
{
    uint64_t val = 0;
    for (const char* curr = buffer; curr < buffer + 16; ++curr)
    {
        const char c = *curr;
        const char msn = c >> 4;
        const char mask = ((88 >> msn) & 1) - 1;
        const unsigned char digit = "\x0\x1\x2\x3\x4\x5\x6\x7\x8\x9\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xA\xB\xC\xD\xE\xF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF"[((msn - 3) & 1) << 4 | (c & 0xF)] | mask;
        if (digit == 0xFF)
        {
            return 0;   //  Not a hexidecimal digit
        }
        val = val << 4 | digit;
    }
    return val;
}


//////// ExtSerialization member functions ////////

ExtSerializationImpl::ExtSerializationImpl() : m_serializationEncoding(EncodingID::CapnProtoBinary), m_bufferProvider(&s_defaultBufferProvider)
{
}


ExtSerializationImpl::~ExtSerializationImpl()
{
    // Release and remove all registered serializers
    Array<ExtSerializer*>::type registeredSerializers;
    registeredSerializers.reserve(m_serializers.size());
    for (auto it = m_serializers.getIterator(); !it.done(); ++it)
    {
        registeredSerializers.pushBack(it->second);
    }
    m_serializers.clear();
    for (uint32_t i = 0; i < registeredSerializers.size(); ++i)
    {
        registeredSerializers[i]->release();
    }
}


char* ExtSerializationImpl::writeHeaderIntoBuffer(char* buffer, uint64_t bufferSize, uint32_t objectTypeID, uint32_t encodingID, uint64_t dataSize) const
{
    if (bufferSize < HeaderSize)
    {
        return nullptr;
    }

    char* stop = buffer + HeaderSize;

    size_t versionLen = strlen(s_version);
    if (versionLen > 63)
    {
        versionLen = 63;
    }

    memset(buffer, ' ', HeaderSize);
    memcpy(buffer, s_identifier, 32);           buffer += 32;
    memcpy(buffer, s_version, versionLen);      buffer += 64;
    writeIDToBuffer(buffer, objectTypeID);      buffer += 5;
    writeIDToBuffer(buffer, encodingID);        buffer += 5;
    writeU64InHexToBuffer(buffer, dataSize);    buffer += 16;
    *(stop - 1) = '\n';

    return stop;
}


const char* ExtSerializationImpl::readHeaderFromBuffer(uint32_t* objectTypeID, uint32_t* encodingID, uint64_t* dataSize, const char* buffer, uint64_t bufferSize) const
{
    if (bufferSize < HeaderSize)
    {
        NVBLAST_LOG_ERROR("ExtSerializationImpl::readHeaderFromBuffer: header terminator not found.");
        return nullptr;
    }

    const char* stop = buffer + HeaderSize;

    if (memcmp(buffer, s_identifier, 32))
    {
        NVBLAST_LOG_ERROR("ExtSerializationImpl::readHeaderFromBuffer: file identifier does not match expected value.");
        return nullptr;
    }
    buffer += 32;

    const char* s = strchr(buffer, ' ');
    if (s == nullptr)
    {
        NVBLAST_LOG_ERROR("ExtSerializationImpl::readHeaderFromBuffer: file format error reading serializer library version.");
    }
    if (memcmp(buffer, s_version, s - buffer))
    {
        NVBLAST_LOG_ERROR("ExtSerializationImpl::readHeaderFromBuffer: file version does not match serializer library version.");
        return nullptr;
    }
    buffer += 64;

    if (objectTypeID != nullptr)
    {
        *objectTypeID = readIDFromBuffer(buffer);
    }
    buffer += 5;

    if (encodingID != nullptr)
    {
        *encodingID = readIDFromBuffer(buffer);
    }
    buffer += 5;

    if (dataSize != nullptr)
    {
        *dataSize = readU64InHexFromBuffer(buffer);
    }
    buffer += 16;

    return stop;
}


bool ExtSerializationImpl::registerSerializer(ExtSerializer& serializer)
{
    return m_serializers.insert(generateKey(serializer), &serializer);
}


bool ExtSerializationImpl::unregisterSerializer(ExtSerializer& serializer)
{
    const uint64_t key = generateKey(serializer);
    const auto entry = m_serializers.find(key);
    if (entry == nullptr)
    {
        return false;
    }
    entry->second->release();
    return m_serializers.erase(key);
}


ExtSerializer* ExtSerializationImpl::findSerializer(uint32_t objectTypeID, uint32_t encodingID)
{
    auto entry = m_serializers.find(generateKey(objectTypeID, encodingID));
    return entry != nullptr ? entry->second : nullptr;
}


bool ExtSerializationImpl::setSerializationEncoding(uint32_t encodingID)
{
    m_serializationEncoding = encodingID;

    return true;
}


uint32_t ExtSerializationImpl::getSerializationEncoding() const
{
    return m_serializationEncoding;
}


void ExtSerializationImpl::setBufferProvider(BufferProvider* bufferProvider)
{
    m_bufferProvider = bufferProvider != nullptr ? bufferProvider : &s_defaultBufferProvider;
}


bool ExtSerializationImpl::peekHeader(uint32_t* objectTypeID, uint32_t* encodingID, uint64_t* dataSize, const void* buffer, uint64_t bufferSize)
{
    return nullptr != readHeaderFromBuffer(objectTypeID, encodingID, dataSize, reinterpret_cast<const char*>(buffer), bufferSize);
}


const void* ExtSerializationImpl::skipObject(uint64_t& bufferSize, const void* buffer)
{
    uint64_t dataSize;
    const char* next = readHeaderFromBuffer(nullptr, nullptr, &dataSize, static_cast<const char*>(buffer), bufferSize);
    if (next == nullptr)
    {
        return nullptr;
    }
    next += dataSize;
    const uint64_t skipSize = next - static_cast<const char*>(buffer);
    NVBLAST_CHECK_ERROR(skipSize <= bufferSize, "Object size in buffer is too large for given buffer size.", return nullptr);
    bufferSize -= skipSize;
    return next;
}


void* ExtSerializationImpl::deserializeFromBuffer(const void* buffer, uint64_t bufferSize, uint32_t* objectTypeIDPtr)
{
    uint32_t objectTypeID;
    uint32_t encodingID;
    uint64_t dataSize;
    void* result = nullptr;

    buffer = readHeaderFromBuffer(&objectTypeID, &encodingID, &dataSize, reinterpret_cast<const char*>(buffer), bufferSize);
    if (buffer != nullptr)
    {
        auto entry = m_serializers.find(generateKey(objectTypeID, encodingID));
        if (entry != nullptr && entry->second != nullptr)
        {
            result = entry->second->deserializeFromBuffer(buffer, dataSize);
        }
    }

    if (objectTypeIDPtr != nullptr)
    {
        *objectTypeIDPtr = result != nullptr ? objectTypeID : 0;
    }

    return result;
}


uint64_t ExtSerializationImpl::serializeIntoBuffer(void*& buffer, const void* object, uint32_t objectTypeID)
{
    if (!m_serializationEncoding)
    {
        NVBLAST_LOG_ERROR("ExtSerializationImpl::serializeIntoBuffer: no serialization encoding has been set.");
        return false;   // No encoding available
    }

    auto entry = m_serializers.find(generateKey(objectTypeID, m_serializationEncoding));
    if (entry == nullptr || entry->second == nullptr)
    {
        return false;
    }

    const uint64_t size = entry->second->serializeIntoBuffer(buffer, *m_bufferProvider, object, HeaderSize);
    if (size < HeaderSize)
    {
        NVBLAST_LOG_ERROR("ExtSerializationImpl::serializeIntoBuffer: failed to write data to buffer.");
        return 0;
    }

    writeHeaderIntoBuffer(reinterpret_cast<char*>(buffer), HeaderSize, objectTypeID, m_serializationEncoding, size - HeaderSize);

    return size;
}


void ExtSerializationImpl::release()
{
    NVBLAST_DELETE(this, ExtSerializationImpl);
}


//////// ExtSerializationImpl::AllocBufferProvider member functions ////////

void* ExtSerializationImpl::AllocBufferProvider::requestBuffer(size_t size)
{
    return NVBLAST_ALLOC(size);
}

}   // namespace Blast
}   // namespace Nv


Nv::Blast::ExtSerialization* NvBlastExtSerializationCreate()
{
    Nv::Blast::ExtSerializationImpl* serialization = NVBLAST_NEW(Nv::Blast::ExtSerializationImpl) ();

    // Automatically load LL serializers
    NvBlastExtLlSerializerLoadSet(*serialization);

    return serialization;
}
