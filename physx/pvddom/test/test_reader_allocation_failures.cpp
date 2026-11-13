// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "../../pvdruntime/src/OmniPvdReaderImpl.h"
#include "OmniPvdCommands.h"
#include "OmniPvdLibraryFunctions.h"
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string.h>
#include <limits>
#include <new>
#include <string>
#include <vector>

namespace
{
size_t failAfter = (std::numeric_limits<size_t>::max)();
ptrdiff_t liveAllocations = 0;
bool failureTriggered = false;
bool captureNextAllocation = false;
size_t capturedAllocationSize = 0;
size_t diagnosticRecords = 0, diagnosticErrors = 0, diagnosticMaxLength = 0;
void OMNI_PVD_CALL countDiagnostic(char* text)
{
    ++diagnosticRecords;
    diagnosticMaxLength = (std::max)(diagnosticMaxLength, std::strlen(text));
    if (std::strstr(text, "[parser] ERROR:") == text) ++diagnosticErrors;
}

struct Stream : OmniPvdReadStream
{
    std::vector<uint8_t> bytes;
    size_t position = 0;
    template<typename T> void put(T value)
    {
        const uint8_t* first = reinterpret_cast<const uint8_t*>(&value);
        bytes.insert(bytes.end(), first, first + sizeof(value));
    }
    void name(const char* value)
    {
        const uint16_t size = static_cast<uint16_t>(strnlen(value, 0xFFFF));
        put(size);
        bytes.insert(bytes.end(), value, value + size);
    }
    uint64_t OMNI_PVD_CALL readBytes(uint8_t* out, uint64_t count) override
    {
        const size_t length = size_t((std::min)(count, uint64_t(bytes.size() - position)));
        if (length) std::memcpy(out, bytes.data() + position, length);
        position += length;
        return length;
    }
    uint64_t OMNI_PVD_CALL skipBytes(uint64_t count) override
    {
        const size_t length = size_t((std::min)(count, uint64_t(bytes.size() - position)));
        position += length;
        return length;
    }
    bool OMNI_PVD_CALL openStream() override { return true; }
    bool OMNI_PVD_CALL closeStream() override { return true; }
};

Stream commands()
{
    Stream stream;
    stream.put(uint32_t(0)); stream.put(uint32_t(4)); stream.put(uint32_t(0));
    stream.put(uint8_t(OmniPvdCommand::eREGISTER_CLASS));
    stream.put(uint32_t(1)); stream.put(uint32_t(0)); stream.name("Thing");
    stream.put(uint8_t(OmniPvdCommand::eREGISTER_ATTRIBUTE));
    stream.put(uint32_t(1)); stream.put(uint32_t(1));
    stream.put(uint16_t(OmniPvdDataType::eUINT32)); stream.put(uint32_t(1)); stream.name("value");
    stream.put(uint8_t(OmniPvdCommand::eCREATE_OBJECT));
    stream.put(uint64_t(1)); stream.put(uint32_t(1)); stream.put(uint64_t(10)); stream.name("object");
    stream.put(uint8_t(OmniPvdCommand::eSET_ATTRIBUTE));
    stream.put(uint64_t(1)); stream.put(uint64_t(10)); stream.put(uint8_t(1));
    stream.put(uint32_t(1)); stream.put(uint32_t(4)); stream.put(uint32_t(42));
    return stream;
}

bool readerPayloadFailure(bool verbose)
{
    Stream stream = commands();
    const ptrdiff_t baseline = liveAllocations;
    {
        OmniPvdReaderImpl reader;
        reader.setLogFunction(countDiagnostic); reader.mVerbose = verbose;
        diagnosticErrors = 0;
        reader.setReadStream(stream);
        for (unsigned i = 0; i < 3; ++i)
            if (reader.getNextCommand() == OmniPvdCommand::eINVALID) return false;
        failAfter = 0;
        failureTriggered = false;
        bool threw = false;
        try { reader.getNextCommand(); }
        catch (const std::bad_alloc&) { threw = true; }
        failAfter = (std::numeric_limits<size_t>::max)();
        const size_t stopped = stream.position;
        if (threw || !failureTriggered || reader.mStatus != OmniPvdReaderStatus::eLIMIT ||
            reader.getNextCommand() != OmniPvdCommand::eINVALID || stream.position != stopped ||
            reader.getAttributeDataPointer() != nullptr || diagnosticErrors != 1) return false;
    }
    return liveAllocations == baseline;
}

// A failed growth must keep (not free) the previous payload buffer.
bool readerPayloadGrowthFailureKeepsPreviousBuffer(bool verbose)
{
    Stream stream = commands();
    stream.put(uint8_t(OmniPvdCommand::eSET_ATTRIBUTE));
    stream.put(uint64_t(1)); stream.put(uint64_t(10)); stream.put(uint8_t(1));
    stream.put(uint32_t(1)); stream.put(uint32_t(8)); stream.put(uint64_t(7));
    const ptrdiff_t baseline = liveAllocations;
    {
        OmniPvdReaderImpl reader;
        reader.setLogFunction(countDiagnostic); reader.mVerbose = verbose;
        diagnosticErrors = 0;
        reader.setReadStream(stream);
        for (unsigned i = 0; i < 4; ++i)
            if (reader.getNextCommand() == OmniPvdCommand::eINVALID) return false;
        const ptrdiff_t withBuffer = liveAllocations;
        failAfter = 0;
        failureTriggered = false;
        bool threw = false;
        try { reader.getNextCommand(); }
        catch (const std::bad_alloc&) { threw = true; }
        failAfter = (std::numeric_limits<size_t>::max)();
        const size_t stopped = stream.position;
        if (threw || !failureTriggered || reader.mStatus != OmniPvdReaderStatus::eLIMIT ||
            reader.getNextCommand() != OmniPvdCommand::eINVALID || stream.position != stopped ||
            liveAllocations != withBuffer || diagnosticErrors != 1) return false;
    }
    return liveAllocations == baseline;
}

bool diagnosticsDoNotAllocate()
{
    Stream stream = commands();
    stream.bytes.resize(stream.bytes.size() - 30); // Drop the allocating SET payload.
    const std::string escaped(2047, char(0x80));
    stream.put(uint8_t(OmniPvdCommand::eREGISTER_CLASS));
    stream.put(uint32_t(2)); stream.put(uint32_t(0)); stream.name(escaped.c_str());
    for (auto command : {OmniPvdCommand::eSTART_FRAME, OmniPvdCommand::eSTOP_FRAME})
    {
        stream.put(uint8_t(command)); stream.put(UINT64_MAX); stream.put(UINT64_MAX);
    }
    stream.put(uint8_t(OmniPvdCommand::eSET_ATTRIBUTE));
    stream.put(UINT64_MAX); stream.put(UINT64_MAX); stream.put(uint8_t(32));
    for (unsigned i = 0; i < 32; ++i) stream.put(UINT32_MAX);
    stream.put(uint32_t(0));
    stream.put(uint8_t(OmniPvdCommand::eRECORD_MESSAGE)); stream.put(UINT64_MAX);
    stream.name(escaped.c_str()); stream.name(escaped.c_str());
    stream.put(UINT32_MAX); stream.put(UINT32_MAX); stream.put(UINT32_MAX);
    for (unsigned mode = 0; mode < 3; ++mode)
    {
        stream.position = 0;
        const ptrdiff_t baseline = liveAllocations;
        OmniPvdReader* reader = createOmniPvdReader();
        if (!reader) return false;
        OmniPvdReaderDiagnostics options;
        options.verbose = mode != 0; options.logFunction = mode == 2 ? nullptr : countDiagnostic;
        setOmniPvdReaderDiagnostics(*reader, options);
        reader->setReadStream(stream);
        diagnosticRecords = diagnosticErrors = diagnosticMaxLength = 0;
        failureTriggered = false; failAfter = 0;
        unsigned decoded = 0;
        bool threw = false;
        try { while (reader->getNextCommand() != OmniPvdCommand::eINVALID) ++decoded; }
        catch (const std::bad_alloc&) { threw = true; }
        const bool injectionUnconsumed = failAfter == 0 && !failureTriggered;
        failAfter = (std::numeric_limits<size_t>::max)();
        const bool valid = !threw && injectionUnconsumed && decoded == 8 &&
            getOmniPvdReaderStatus(*reader).code == OmniPvdReaderStatus::eEND_OF_STREAM &&
            diagnosticRecords == (mode == 1 ? 30u : 0u) && !diagnosticErrors && diagnosticMaxLength < 2048;
        destroyOmniPvdReader(*reader);
        if (!valid || liveAllocations != baseline) return false;
    }
    return true;
}

bool readerPayloadGrowthUsesBoundedSlack()
{
    captureNextAllocation = false;
    capturedAllocationSize = 0;
    Stream stream = commands();
    auto set = [&](uint32_t size) {
        stream.put(uint8_t(OmniPvdCommand::eSET_ATTRIBUTE));
        stream.put(uint64_t(1)); stream.put(uint64_t(10)); stream.put(uint8_t(1));
        stream.put(uint32_t(1)); stream.put(size);
        for (uint32_t i = 0; i < size; ++i) stream.put(uint8_t(0));
    };
    set(5); set(7); set(10); set(10);
    OmniPvdReaderImpl reader(10, 32);
    reader.setReadStream(stream);
    for (unsigned i = 0; i < 3; ++i)
        if (reader.getNextCommand() == OmniPvdCommand::eINVALID) return false;

    captureNextAllocation = true;
    if (reader.getNextCommand() != OmniPvdCommand::eSET_ATTRIBUTE || capturedAllocationSize != 5) return false;
    const uint8_t* first = reader.getAttributeDataPointer();
    capturedAllocationSize = 0;
    captureNextAllocation = true;
    if (reader.getNextCommand() != OmniPvdCommand::eSET_ATTRIBUTE || capturedAllocationSize ||
        reader.getAttributeDataPointer() != first) return false;
    captureNextAllocation = true;
    if (reader.getNextCommand() != OmniPvdCommand::eSET_ATTRIBUTE || capturedAllocationSize != 9) return false;
    captureNextAllocation = true;
    if (reader.getNextCommand() != OmniPvdCommand::eSET_ATTRIBUTE || capturedAllocationSize != 10) return false;
    const uint8_t* atLimit = reader.getAttributeDataPointer();
    capturedAllocationSize = 0;
    captureNextAllocation = true;
    const bool result = reader.getNextCommand() == OmniPvdCommand::eSET_ATTRIBUTE && !capturedAllocationSize &&
        reader.getAttributeDataPointer() == atLimit;
    captureNextAllocation = false;
    return result;
}

bool readerPayloadGrowthClampsWithoutOverflow()
{
    const uint32_t maximum = (std::numeric_limits<uint32_t>::max)();
    for (uint32_t requested : {maximum - 1, maximum})
    {
        Stream stream;
        stream.put(uint32_t(0)); stream.put(uint32_t(4)); stream.put(uint32_t(0));
        stream.put(uint8_t(OmniPvdCommand::eSET_ATTRIBUTE));
        stream.put(uint64_t(1)); stream.put(uint64_t(10)); stream.put(uint8_t(1));
        stream.put(uint32_t(1)); stream.put(requested);
        OmniPvdReaderImpl reader(maximum, 32);
        reader.setReadStream(stream);
        capturedAllocationSize = 0;
        captureNextAllocation = true;
        failAfter = 0;
        failureTriggered = false;
        const OmniPvdCommand::Enum command = reader.getNextCommand();
        failAfter = (std::numeric_limits<size_t>::max)();
        captureNextAllocation = false;
        if (command != OmniPvdCommand::eINVALID || !failureTriggered ||
            reader.mStatus != OmniPvdReaderStatus::eLIMIT || capturedAllocationSize != maximum) return false;
    }
    return true;
}

}

void* operator new(size_t size)
{
    if (captureNextAllocation)
    {
        capturedAllocationSize = size;
        captureNextAllocation = false;
    }
    if (failAfter != (std::numeric_limits<size_t>::max)() && failAfter-- == 0)
    {
        failAfter = (std::numeric_limits<size_t>::max)();
        failureTriggered = true;
        throw std::bad_alloc();
    }
    if (void* result = std::malloc(size ? size : 1)) { ++liveAllocations; return result; }
    throw std::bad_alloc();
}
void* operator new[](size_t size) { return ::operator new(size); }
void* operator new(size_t size, const std::nothrow_t&) noexcept
{
    try { return ::operator new(size); }
    catch (const std::bad_alloc&) { return nullptr; }
}
void* operator new[](size_t size, const std::nothrow_t& tag) noexcept { return ::operator new(size, tag); }
void operator delete(void* pointer) noexcept { if (pointer) --liveAllocations; std::free(pointer); }
void operator delete[](void* pointer) noexcept { ::operator delete(pointer); }
void operator delete(void* pointer, size_t) noexcept { ::operator delete(pointer); }
void operator delete[](void* pointer, size_t) noexcept { ::operator delete(pointer); }
void operator delete(void* pointer, const std::nothrow_t&) noexcept { ::operator delete(pointer); }
void operator delete[](void* pointer, const std::nothrow_t&) noexcept { ::operator delete(pointer); }

int main()
{
    if (!diagnosticsDoNotAllocate())
    {
        std::fprintf(stderr, "reader diagnostics allocated, lost records or exceeded the callback bound\n");
        return 8;
    }
    if (!readerPayloadFailure(false) || !readerPayloadFailure(true))
    {
        std::fprintf(stderr, "reader allocation failure was not a sticky limit or leaked storage\n");
        return 1;
    }
    if (!readerPayloadGrowthFailureKeepsPreviousBuffer(false) || !readerPayloadGrowthFailureKeepsPreviousBuffer(true))
    {
        std::fprintf(stderr, "reader payload growth failure released or leaked the previous buffer\n");
        return 3;
    }
    if (!readerPayloadGrowthUsesBoundedSlack())
    {
        std::fprintf(stderr, "reader payload growth did not retain bounded amortized slack\n");
        return 4;
    }
    if (!readerPayloadGrowthClampsWithoutOverflow())
    {
        std::fprintf(stderr, "reader payload growth did not clamp overflow boundaries\n");
        return 5;
    }
    return 0;
}
