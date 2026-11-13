// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "OmniPvdReader.h"
#include "OmniPvdWriter.h"
#include <algorithm>
#include <cstring>
#include <string>
#include <string.h>
#include <vector>

namespace
{
inline std::vector<std::string> withoutReaderSuccessRecords(const std::vector<std::string>& records)
{
    std::vector<std::string> remaining;
    for (const auto& record : records)
    {
        if (record == "OmniPvdRuntimeReaderImpl::startReading majorVersion(0), minorVersion(4), patch(0)" ||
            record.find("[parser] trace command=") == 0 || record.find("[parser] text command=") == 0)
            continue;
        remaining.push_back(record);
    }
    return remaining;
}

struct Bytes : OmniPvdReadStream
{
    std::vector<uint8_t> data;
    size_t position = 0;
    Bytes() { put(uint32_t(0)); put(uint32_t(4)); put(uint32_t(0)); }
    template<class T> void put(T value)
    {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
        data.insert(data.end(), p, p + sizeof(value));
    }
    void name(const char* value)
    {
        const uint16_t size = static_cast<uint16_t>(strnlen(value, 0xFFFF));
        put(size);
        data.insert(data.end(), value, value + size);
    }
    void command(OmniPvdCommand::Enum value) { put(static_cast<uint8_t>(value)); }
    void cls(uint32_t id = 1, const char* text = "Thing", uint32_t base = 0)
    {
        command(OmniPvdCommand::eREGISTER_CLASS); put(id); put(base); name(text);
    }
    void attr(uint32_t id = 1, const char* text = "value", uint32_t owner = 1,
              OmniPvdDataType::Enum type = OmniPvdDataType::eUINT32, uint32_t count = 1)
    {
        command(OmniPvdCommand::eREGISTER_ATTRIBUTE); put(owner); put(id);
        put(static_cast<uint16_t>(type)); put(count); name(text);
    }
    void create(uint64_t id = 10, uint32_t type = 1, uint64_t context = 1)
    {
        command(OmniPvdCommand::eCREATE_OBJECT); put(context); put(type); put(id); name("");
    }
    void set(uint32_t size = 4, uint32_t id = 1, uint64_t object = 10, uint8_t depth = 1, uint64_t context = 1)
    {
        command(OmniPvdCommand::eSET_ATTRIBUTE); put(context); put(object); put(depth);
        for (unsigned i = 0; i < depth; ++i) put(id);
        put(size);
        for (uint32_t i = 0; i < size; ++i) put(uint8_t(0));
    }
    void frame(uint64_t timestamp, bool stop = false, uint64_t context = 1)
    {
        command(stop ? OmniPvdCommand::eSTOP_FRAME : OmniPvdCommand::eSTART_FRAME);
        put(context); put(timestamp);
    }
    void list(uint32_t id, const char* text, uint32_t owner)
    {
        command(OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE); put(owner); put(id);
        put(uint16_t(OmniPvdDataType::eOBJECT_HANDLE)); name(text);
    }
    void reference(uint64_t object, uint32_t attribute, uint64_t target, bool list = false, uint64_t context = 1)
    {
        const size_t start = data.size();
        set(8, attribute, object, 1, context);
        if (list) data[start] = uint8_t(OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE);
        memcpy(data.data() + data.size() - 8, &target, 8);
    }
    void removeReference(uint64_t object, uint32_t attribute, uint64_t target, uint64_t context = 1)
    {
        const size_t start = data.size();
        reference(object, attribute, target, true, context);
        data[start] = uint8_t(OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE);
    }
    uint64_t OMNI_PVD_CALL readBytes(uint8_t* out, uint64_t count) override
    {
        const size_t n = static_cast<size_t>((std::min)(count, uint64_t(data.size() - position)));
        if (n) memcpy(out, data.data() + position, n);
        position += n;
        return n;
    }
    uint64_t OMNI_PVD_CALL skipBytes(uint64_t count) override
    {
        const size_t n = static_cast<size_t>((std::min)(count, uint64_t(data.size() - position)));
        position += n;
        return n;
    }
    bool OMNI_PVD_CALL openStream() override { return true; }
    bool OMNI_PVD_CALL closeStream() override { return true; }
};

struct EncodedStreamBytes : Bytes, OmniPvdWriteStream
{
    EncodedStreamBytes() { data.clear(); }
    uint64_t OMNI_PVD_CALL writeBytes(const uint8_t* input, uint64_t count) override
    {
        data.insert(data.end(), input, input + count);
        return count;
    }
    bool OMNI_PVD_CALL flush() override { return true; }
    bool OMNI_PVD_CALL openStream() override { return true; }
    bool OMNI_PVD_CALL closeStream() override { return true; }
};

}
