// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "PvdDomParser.h"
#include <memory>
#include <map>
#include <string>
#include <vector>

namespace PvdDomInspect
{
struct LoadLimits
{
    uint64_t maxStreamBytes = 512ull << 20;
    uint64_t maxCommands = 2000000;
    uint64_t maxCreates = 100000;
    uint64_t maxClasses = 4096;
    uint64_t maxAttributes = 65535;
    uint64_t maxInheritanceDepth = 64;
    uint64_t maxAttributeBytes = 64ull << 20;
    uint64_t maxFrameRecords = 2000000;
    uint64_t maxHandlerSteps = 100000000;
    uint64_t maxRetainedBytes = 512ull << 20;
};

struct Error
{
    enum class Code { None, Io, Invalid, Unsupported, Limit };
    enum class LimitKind
    {
        None, StreamBytes, Commands, Creates, Classes, Attributes, InheritanceDepth,
        AttributeBytes, FrameRecords, HandlerSteps, RetainedBytes
    };
    Code code = Code::None;
    std::string message;
    uint64_t byteOffset = 0;
    // One-based command index. Zero identifies the header or file open.
    uint64_t commandIndex = 0;
    // Adjustable LoadLimits field that rejected the load. None indicates fixed or allocation limits.
    LimitKind limitKind = LimitKind::None;
};

// Recovery excludes a partial final command only after verifying its readable physical remainder.
enum class LoadMode { Strict, RecoverTruncatedTail };

struct LoadInfo
{
    bool recoveredTail = false;
    uint64_t sourceBytes = 0;
    Error terminalError;
};

struct RecordedFrame
{
    uint64_t recordingSegment;
    OmniPvdContextHandle context;
    uint64_t recordedFrame;
    uint64_t sourceTimestamp;
    bool completed;
};

struct LoadStats
{
    uint64_t streamBytes = 0;
    uint64_t commands = 0;
    uint64_t ignoredCommands = 0;
    // Unresolved optional SET/ADD validation records, counted per edge command, not per target.
    uint64_t unresolvedOptionalEdges = 0;
    uint64_t creates = 0;
    uint64_t classes = 0;
    uint64_t attributes = 0;
    uint64_t samples = 0;
    uint64_t handlerSteps = 0;
    uint64_t membershipVisits = 0;
    // Conservative logical charges for retained DOM, metadata and diagnostics, not hard RSS.
    uint64_t accountedRetainedBytes = 0;
};

struct SceneMembership
{
    uint64_t recordingSegment = 0;
    uint64_t context = 0;
    uint64_t startFrame = 0;
    uint64_t endFrame = 0;
    uint64_t startCommand = 0;
    uint64_t stopOrdinal = 0;
};
struct SampleWrite
{
    uint64_t recordingSegment = 0;
    uint64_t context = 0;
    uint64_t frame = 0;
    bool frameKnown = false;
    uint64_t rawContext = 0;
    uint64_t rawFrame = 0;
    bool hasRawFrame = false;
    uint64_t command = 0;
    const OmniPvdAttributeDef* attribute = nullptr;
    const OmniPvdAttributeSample* sample = nullptr;
};
struct ObjectHistory
{
    const OmniPvdObject* object = nullptr;
    std::vector<SceneMembership> memberships;
    std::map<uint64_t, std::vector<SampleWrite>> writes;
};
struct FrameIndex
{
    std::vector<uint64_t> recordedFrames;
    bool monotonic = true;
    bool sceneMapped = false;
    bool sceneContextRecreated = false;
};

// Owns all returned storage until the next load or destruction. A failed load leaves no DOM.
// Concurrent reads are allowed after load. Mutation and legacy parser calls require external serialization.
class Document
{
public:
    Document();
    ~Document();
    Document(const Document&) = delete;
    Document& operator=(const Document&) = delete;
    bool load(const char* path, const LoadLimits& limits, Error& error);
    bool load(const char* path, const LoadLimits& limits, Error& error, LoadMode mode);
    // Diagnostics apply only during this synchronous load and are not retained by the Document.
    // Reader errors may be reported even when truncated-tail recovery makes the load succeed.
    // The callback must follow the lifetime, serialization and re-entry rules
    // documented for setOmniPvdReaderDiagnostics().
    bool load(const char* path, const LoadLimits& limits, Error& error,
              const OmniPvdReaderDiagnostics& diagnostics);
    bool load(const char* path, const LoadLimits& limits, Error& error, LoadMode mode,
              const OmniPvdReaderDiagnostics& diagnostics);
    const LoadInfo& loadInfo() const;
    const OmniPvdDOMState* dom() const;
    const std::vector<RecordedFrame>& frames() const;
    const LoadStats& stats() const;
    const OmniPvdMessages& messages() const;
    // Bounded ingestion records. Pointers refer to this document's DOM.
    const std::map<uint64_t, ObjectHistory>& history() const;
    const std::map<std::pair<uint64_t, uint64_t>, FrameIndex>& frameIndex() const;
private:
    friend struct DocumentAccess;
    struct Storage;
    std::unique_ptr<Storage> mStorage;
    LoadInfo mLoadInfo;
};
}
