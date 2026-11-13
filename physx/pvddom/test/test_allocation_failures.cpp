// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PvdDomDocument.h"
#include "OmniPvdDefines.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReader.h"
#include "../../pvdruntime/src/OmniPvdWriterImpl.h"
#include <cstdio>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string.h>
#include <limits>
#include <new>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#include <crtdbg.h>
#endif

namespace
{
size_t failAfter = (std::numeric_limits<size_t>::max)();
size_t proxySizedExemptions = 0;
ptrdiff_t liveAllocations = 0;
size_t diagnosticRecords = 0;
void OMNI_PVD_CALL countDiagnostic(char*) { ++diagnosticRecords; }

template<typename T> void put(std::vector<uint8_t>& bytes, T value)
{
    const uint8_t* start = reinterpret_cast<const uint8_t*>(&value);
    bytes.insert(bytes.end(), start, start + sizeof(value));
}

void name(std::vector<uint8_t>& bytes, const char* value)
{
    const uint16_t size = static_cast<uint16_t>(strnlen(value, 0xFFFF));
    put(bytes, size);
    bytes.insert(bytes.end(), value, value + size);
}

struct Stream : OmniPvdReadStream, OmniPvdWriteStream
{
    std::vector<uint8_t> bytes;
    size_t position = 0;
    uint64_t OMNI_PVD_CALL readBytes(uint8_t* out, uint64_t count) override
    {
        const size_t n = size_t((std::min)(count, uint64_t(bytes.size() - position)));
        if (n) memcpy(out, bytes.data() + position, n);
        position += n;
        return n;
    }
    uint64_t OMNI_PVD_CALL skipBytes(uint64_t) override { return 0; }
    uint64_t OMNI_PVD_CALL writeBytes(const uint8_t* data, uint64_t count) override
    {
        bytes.insert(bytes.end(), data, data + count);
        return count;
    }
    bool OMNI_PVD_CALL openStream() override { return true; }
    bool OMNI_PVD_CALL closeStream() override { return true; }
    bool OMNI_PVD_CALL flush() override { return true; }
};

bool readerFailure()
{
    Stream stream;
    OmniPvdWriterImpl writer; writer.setWriteStream(stream);
    uint64_t payload[4] = {uint64_t(OmniPvdCommand::eSTART_FRAME), 1, 9, 0};
    const OmniPvdAttributeHandle attribute = 1;
    writer.setAttribute(1, 10, &attribute, 1, reinterpret_cast<uint8_t*>(payload), 8);
    writer.setAttribute(1, 10, &attribute, 1, reinterpret_cast<uint8_t*>(payload), sizeof(payload));
    OmniPvdReader* reader = createOmniPvdReaderWithLimits(64);
    if (!reader) return false;
    reader->setReadStream(stream);
    bool ok = reader->getNextCommand() == OmniPvdCommand::eSET_ATTRIBUTE;
    const uint8_t* previous = reader->getAttributeDataPointer();
    failAfter = 0;
    try { ok = (reader->getNextCommand() == OmniPvdCommand::eINVALID) && ok; }
    catch (const std::bad_alloc&) { ok = false; }
    failAfter = (std::numeric_limits<size_t>::max)();
    const size_t stopped = stream.position;
    ok = ok && getOmniPvdReaderStatus(*reader).code == OmniPvdReaderStatus::eLIMIT &&
        reader->getNextCommand() == OmniPvdCommand::eINVALID && stream.position == stopped &&
        reader->getAttributeDataPointer() == nullptr && previous[0] == uint8_t(OmniPvdCommand::eSTART_FRAME);
    destroyOmniPvdReader(*reader);
    stream.position = 0;
    reader = createOmniPvdReaderWithLimits(64);
    if (!reader) return false;
    reader->setReadStream(stream);
    ok = reader->getNextCommand() == OmniPvdCommand::eSET_ATTRIBUTE && ok;
    ok = reader->getNextCommand() == OmniPvdCommand::eSET_ATTRIBUTE && ok;
    destroyOmniPvdReader(*reader);
    return ok;
}

bool sweep(const char* path, size_t first, size_t end, const PvdDomInspect::LoadLimits& limits = {},
           bool requirePayloadFailure = false, bool verbose = false)
{
    OmniPvdReaderDiagnostics diagnostics;
    diagnostics.logFunction = countDiagnostic; diagnostics.verbose = verbose;
    diagnosticRecords = 0;
    // Warm process-wide lazy state before the per-load baseline. No test framework
    // runs inside the scope; Error strings and the entire Document die before comparison.
    {
        PvdDomInspect::Document document; PvdDomInspect::Error error;
        if (!document.load(path, limits, error, diagnostics) || (verbose && !diagnosticRecords)) return false;
    }
    size_t failures = 0, payloadFailures = 0;
    for (size_t index = first; index < end; ++index)
    {
        const ptrdiff_t baseline = liveAllocations;
        bool loaded = false, valid = false;
        {
            PvdDomInspect::Document document; PvdDomInspect::Error error;
            proxySizedExemptions = 0;
            failAfter = index;
            loaded = document.load(path, limits, error, diagnostics);
            failAfter = (std::numeric_limits<size_t>::max)();
            valid = loaded || (error.code == PvdDomInspect::Error::Code::Limit &&
                error.limitKind == PvdDomInspect::Error::LimitKind::None && !document.dom() &&
                document.frames().empty() && document.messages().empty());
            if (!loaded && error.message == "Attribute payload allocation failed") ++payloadFailures;
            if (!loaded) valid = document.load(path, limits, error, diagnostics) && document.dom() && valid;
        }
        if (!valid || liveAllocations != baseline)
        {
            std::fprintf(stderr, "%s allocation %zu: valid=%d, leaked allocations=%td\n",
                         path, index, int(valid), liveAllocations - baseline);
            return false;
        }
        if (loaded)
        {
            if (requirePayloadFailure && end != first + 1 && !payloadFailures)
            {
                std::fprintf(stderr, "%s: no attribute payload allocation diagnostic observed\n", path);
                return false;
            }
            std::printf("%s: %zu failure points checked, zero live allocation delta\n", path, failures);
            return true;
        }
        ++failures;
    }
    return end == first + 1;
}

bool sweepRecovery(const char* path, size_t first, size_t end, bool verbose)
{
    OmniPvdReaderDiagnostics diagnostics;
    diagnostics.logFunction = countDiagnostic; diagnostics.verbose = verbose;
    {
        PvdDomInspect::Document document; PvdDomInspect::Error error;
        if (!document.load(path, {}, error, PvdDomInspect::LoadMode::RecoverTruncatedTail, diagnostics) ||
            !document.loadInfo().recoveredTail) return false;
    }
    size_t failures = 0;
    for (size_t index = first; index < end; ++index)
    {
        const ptrdiff_t baseline = liveAllocations;
        bool loaded = false, valid = false;
        {
            PvdDomInspect::Document document; PvdDomInspect::Error error;
            proxySizedExemptions = 0;
            failAfter = index;
            loaded = document.load(path, {}, error, PvdDomInspect::LoadMode::RecoverTruncatedTail, diagnostics);
            failAfter = (std::numeric_limits<size_t>::max)();
            const auto& stats = document.stats();
            const auto& info = document.loadInfo();
            valid = loaded ? document.dom() && info.recoveredTail &&
                info.terminalError.code == PvdDomInspect::Error::Code::Invalid :
                error.code == PvdDomInspect::Error::Code::Limit &&
                error.limitKind == PvdDomInspect::Error::LimitKind::None && !document.dom() &&
                document.frames().empty() && document.messages().empty() &&
                stats.streamBytes == 0 && stats.commands == 0 && stats.ignoredCommands == 0 &&
                stats.unresolvedOptionalEdges == 0 && stats.creates == 0 && stats.classes == 0 &&
                stats.attributes == 0 && stats.samples == 0 && stats.handlerSteps == 0 &&
                stats.accountedRetainedBytes == 0 && !info.recoveredTail && info.sourceBytes == 0 &&
                info.terminalError.code == PvdDomInspect::Error::Code::None &&
                info.terminalError.message.empty() && info.terminalError.byteOffset == 0 &&
                info.terminalError.commandIndex == 0;
            if (!loaded)
                valid = document.load(path, {}, error, PvdDomInspect::LoadMode::RecoverTruncatedTail, diagnostics) &&
                    document.dom() && document.loadInfo().recoveredTail && valid;
        }
        if (!valid || liveAllocations != baseline)
        {
            std::fprintf(stderr, "%s recovery allocation %zu: valid=%d, leaked allocations=%td\n",
                         path, index, int(valid), liveAllocations - baseline);
            return false;
        }
        if (loaded)
        {
            std::printf("%s: %zu recovery failure points checked, zero live allocation delta\n", path, failures);
            return true;
        }
        ++failures;
    }
    return end == first + 1;
}

bool specializedFixture(const char* path)
{
    Stream stream; OmniPvdWriterImpl writer; writer.setWriteStream(stream);
    auto cls = [&](const char* name, uint64_t handle) {
        const auto type = writer.registerClass(name, 0);
        writer.createObject(1, type, handle, ""); return type;
    };
    auto reference = [&](OmniPvdClassHandle type, uint64_t object, const char* name, uint64_t target, bool list) {
        const auto attr = list ? writer.registerUniqueListAttribute(type, name, OmniPvdDataType::eOBJECT_HANDLE) :
            writer.registerAttribute(type, name, OmniPvdDataType::eOBJECT_HANDLE, 1);
        const auto* data = reinterpret_cast<const uint8_t*>(&target);
        if (list) writer.addToUniqueListAttribute(1, object, &attr, 1, data, 8);
        else writer.setAttribute(1, object, &attr, 1, data, 8);
        return attr;
    };
    const auto scene = cls("PxScene", 10), actor = cls("PxRigidDynamic", 20);
    const auto volume = cls("PxDeformableVolume", 30), shape = cls("PxShape", 40);
    cls("PxTetrahedronMesh", 50); const auto geom = cls("PxHeightFieldGeometry", 60); cls("PxHeightField", 70);
    const auto generic = cls("Thing", 80); writer.createObject(1, generic, 81, "");
    const auto joint = cls("PxFixedJoint", 90);
    const auto exclusive = writer.registerAttribute(shape, "isExclusive", OmniPvdDataType::eUINT8, 1);
    const uint8_t shared = 0; writer.setAttribute(1, 40, &exclusive, 1, &shared, 1);
    const auto actors = reference(scene, 10, "actors", 20, true);
    reference(actor, 20, "shapes", 40, true);
    reference(volume, 30, "shapes", 40, true);
    reference(volume, 30, "simulationMeshShapes", 50, true);
    reference(geom, 60, "heightField", 70, false);
    reference(generic, 80, "reference", 81, false);
    reference(generic, 80, "references", 81, true);
    reference(scene, 10, "constraints", 100, true);
    reference(joint, 90, "constraint", 100, false);
    for (uint64_t handle : {21ull, 22ull})
    {
        writer.createObject(1, actor, handle, "");
        writer.addToUniqueListAttribute(1, 10, &actors, 1, reinterpret_cast<const uint8_t*>(&handle), 8);
    }
    for (uint64_t handle : {21ull, 20ull, 22ull})
    {
        writer.removeFromUniqueListAttribute(1, 10, &actors, 1, reinterpret_cast<const uint8_t*>(&handle), 8);
        writer.addToUniqueListAttribute(1, 10, &actors, 1, reinterpret_cast<const uint8_t*>(&handle), 8);
    }
    const auto metadata = cls("PxOmniPvdMetaData", 200);
    writer.createObject(1, metadata, 201, "");
    FILE* file = std::fopen(path, "wb");
    if (!file) return false;
    const bool written = std::fwrite(stream.bytes.data(), 1, stream.bytes.size(), file) == stream.bytes.size();
    std::fclose(file);
    return written;
}
}

void* operator new(size_t size)
{
#if defined(_MSC_VER) && _ITERATOR_DEBUG_LEVEL != 0
    // Debug STL constructors allocate proxies in noexcept contexts. Exempt all
    // allocations of this size, including any same-sized product allocations.
    const bool exempt = size == sizeof(std::_Container_proxy);
    if (exempt && failAfter != (std::numeric_limits<size_t>::max)()) ++proxySizedExemptions;
#else
    const bool exempt = false;
#endif
    if (!exempt && failAfter != (std::numeric_limits<size_t>::max)() && failAfter-- == 0)
    {
        failAfter = (std::numeric_limits<size_t>::max)();
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

int main(int argc, char** argv)
{
#ifdef _WIN32
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
    for (int report : {_CRT_WARN, _CRT_ERROR, _CRT_ASSERT})
    {
        _CrtSetReportMode(report, _CRTDBG_MODE_FILE);
        _CrtSetReportFile(report, _CRTDBG_FILE_STDERR);
    }
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
    std::vector<uint8_t> bytes;
    put(bytes, uint32_t(0)); put(bytes, uint32_t(4)); put(bytes, uint32_t(0));
    put(bytes, uint8_t(OmniPvdCommand::eREGISTER_CLASS));
    put(bytes, uint32_t(1)); put(bytes, uint32_t(0)); name(bytes, "Thing");
    put(bytes, uint8_t(OmniPvdCommand::eREGISTER_ATTRIBUTE));
    put(bytes, uint32_t(1)); put(bytes, uint32_t(1));
    put(bytes, uint16_t(OmniPvdDataType::eUINT32)); put(bytes, uint32_t(1)); name(bytes, "value");
    put(bytes, uint8_t(OmniPvdCommand::eCREATE_OBJECT));
    put(bytes, uint64_t(1)); put(bytes, uint32_t(1)); put(bytes, uint64_t(10)); name(bytes, "object");
    put(bytes, uint8_t(OmniPvdCommand::eSET_ATTRIBUTE));
    put(bytes, uint64_t(1)); put(bytes, uint64_t(10)); put(bytes, uint8_t(1));
    put(bytes, uint32_t(1)); put(bytes, uint32_t(4)); put(bytes, uint32_t(42));
    put(bytes, uint8_t(OmniPvdCommand::eRECORD_MESSAGE));
    put(bytes, uint64_t(1)); name(bytes, "message"); name(bytes, "source");
    put(bytes, uint32_t(7)); put(bytes, uint32_t(1)); put(bytes, uint32_t(1));
    put(bytes, uint8_t(OmniPvdCommand::eSTART_FRAME));
    put(bytes, uint64_t(1)); put(bytes, uint64_t(1));
    put(bytes, uint8_t(OmniPvdCommand::eSTOP_FRAME));
    put(bytes, uint64_t(1)); put(bytes, uint64_t(2));

    // Two ignored payloads exercise allocation growth after prior command metadata.
    for (uint32_t size : {512u, 2048u})
    {
        put(bytes, uint8_t(OmniPvdCommand::eSET_ATTRIBUTE));
        put(bytes, uint64_t(1)); put(bytes, uint64_t(99)); put(bytes, uint8_t(1));
        put(bytes, uint32_t(1)); put(bytes, size);
        bytes.insert(bytes.end(), size, uint8_t(0));
    }

    std::vector<uint8_t> recoveryBytes = bytes;
    put(recoveryBytes, uint8_t(OmniPvdCommand::eSTART_FRAME));
    put(recoveryBytes, uint64_t(1)); put(recoveryBytes, uint32_t(3));

    const char* path = "allocation_failure.ovd";
    FILE* file = std::fopen(path, "wb");
    if (!file) return 1;
    const bool written = std::fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size();
    std::fclose(file);
    if (!written) return 2;
    const char* recoveryPath = "allocation_recovery.ovd";
    file = std::fopen(recoveryPath, "wb");
    if (!file) return 3;
    const bool recoveryWritten = std::fwrite(recoveryBytes.data(), 1, recoveryBytes.size(), file) == recoveryBytes.size();
    std::fclose(file);
    if (!recoveryWritten) return 4;

    failAfter = 0;
    OmniPvdReader* failedReader = createOmniPvdReaderWithLimits(1u << 20);
    const bool injectedFailure = failAfter == (std::numeric_limits<size_t>::max)();
    failAfter = (std::numeric_limits<size_t>::max)();
    if (failedReader || !injectedFailure)
    {
        if (failedReader) destroyOmniPvdReader(*failedReader);
        std::remove(path);
        std::fprintf(stderr, "bounded reader factory did not consume injection and return null\n");
        return 6;
    }

    if (!readerFailure()) { std::fprintf(stderr, "reader payload allocation failure was not sticky eLIMIT\n"); return 7; }
    const size_t first = argc > 1 ? static_cast<size_t>(std::strtoull(argv[1], nullptr, 10)) : 0;
    const size_t end = argc > 1 ? first + 1 : 100000;
    const char* specialized = "allocation_specialized.ovd";
    PvdDomInspect::LoadLimits exactCap; exactCap.maxAttributeBytes = 2048;
    bool passed = specializedFixture(specialized);
    for (bool verbose : {false, true})
        passed = passed && sweep(path, first, end, {}, false, verbose) &&
            sweep(PVDDOM_TEST_DATA_DIR "/test_scene.ovd", first, end, {}, false, verbose) &&
            sweep(specialized, first, end, {}, false, verbose) && sweepRecovery(recoveryPath, first, end, verbose) &&
            sweep(path, first, end, exactCap, true, verbose);
    std::remove(specialized);
    std::remove(recoveryPath);
    std::remove(path);
#if defined(_MSC_VER) && _ITERATOR_DEBUG_LEVEL != 0
    std::printf("last load exempted %zu allocations of Debug proxy size %zu from injection only; all are leak-counted\n",
                proxySizedExemptions, sizeof(std::_Container_proxy));
#endif
    return passed ? 0 : 5;
}
