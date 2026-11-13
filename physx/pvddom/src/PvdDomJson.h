// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "PvdDomInspection.h"
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace PvdDomInspect
{
class Json
{
public:
    Json() { out.imbue(std::locale::classic()); out << std::setprecision(std::numeric_limits<double>::max_digits10); }
    void text(const std::string& value)
    {
        static const char hex[] = "0123456789abcdef";
        out << '"';
        for (size_t i = 0; i < value.size();)
        {
            const unsigned char c = static_cast<unsigned char>(value[i]);
            if (c < 0x80)
            {
                ++i;
                switch (c)
                {
                case '"': out << "\\\""; break;
                case '\\': out << "\\\\"; break;
                case '\n': out << "\\n"; break;
                case '\r': out << "\\r"; break;
                case '\t': out << "\\t"; break;
                default:
                    if (c < 0x20) out << "\\u00" << hex[c >> 4] << hex[c & 15];
                    else out << char(c);
                }
                continue;
            }
            const size_t width = c >= 0xc2 && c <= 0xdf ? 2 : c >= 0xe0 && c <= 0xef ? 3 : c >= 0xf0 && c <= 0xf4 ? 4 : 0;
            uint32_t code = width ? c & ((1u << (7 - width)) - 1) : 0;
            bool valid = width && width <= value.size() - i;
            for (size_t j = 1; valid && j < width; ++j)
            {
                const unsigned char next = static_cast<unsigned char>(value[i + j]);
                valid = (next & 0xc0) == 0x80;
                code = (code << 6) | (next & 0x3f);
            }
            const uint32_t minimum = width == 2 ? 0x80 : width == 3 ? 0x800 : 0x10000;
            valid = valid && code >= minimum && code <= 0x10ffff && (code < 0xd800 || code > 0xdfff);
            if (valid) { out.write(value.data() + i, static_cast<std::streamsize>(width)); i += width; }
            else { out << "\\ufffd"; ++i; textWasReplaced = true; }
        }
        out << '"';
    }
    void number(double value) { if (std::isfinite(value)) out << value; else out << "null"; }
    void integer(int64_t value) { out << value; }
    void integer(uint64_t value) { out << value; }
    void scalar(double value)
    {
        if (!std::isfinite(value)) { out << "null"; return; }
        std::ostringstream text;
        text.imbue(std::locale::classic());
        text << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
        std::string token = text.str();
        if (token.find_first_of(".eE") == std::string::npos) token += ".0";
        out << token;
    }
    void id(uint64_t value) { out << '"' << value << '"'; }
    void boolean(bool value) { out << (value ? "true" : "false"); }
    void frame(const FrameRef& f)
    {
        out << "{\"recordingSegment\":"; id(f.recordingSegment);
        out << ",\"context\":"; id(f.context);
        out << ",\"recordedFrame\":"; id(f.recordedFrame); out << '}';
    }
    void identity(const ObjectIdentity& object)
    {
        out << "{\"objectId\":"; id(object.objectId);
        out << ",\"externalHandle\":"; id(object.externalHandle);
        out << ",\"recordingSegment\":"; id(object.recordingSegment);
        out << ",\"className\":"; text(object.className);
        out << ",\"name\":"; text(object.name);
        out << ",\"lifetimeStart\":"; id(object.lifetimeStart);
        out << ",\"lifetimeEnd\":"; if (object.hasLifetimeEnd) id(object.lifetimeEnd); else out << "null";
        out << ",\"lifetimeStartCommand\":"; id(object.lifetimeStartCommand);
        out << ",\"lifetimeEndCommand\":"; if (object.hasLifetimeEnd) id(object.lifetimeEndCommand); else out << "null";
        out << '}';
    }
    void sample(const SampleResult& s)
    {
        out << "{\"identity\":"; identity(s.identity);
        out << ",\"requested\":"; frame(s.requested);
        out << ",\"source\":"; if (s.state != SampleState::Missing && s.hasSourceFrame) frame(s.source); else out << "null";
        out << ",\"hasSourceFrame\":"; boolean(s.hasSourceFrame);
        out << ",\"rawContext\":"; id(s.rawContext);
        out << ",\"rawFrame\":"; if (s.hasRawFrame) id(s.rawFrame); else out << "null";
        out << ",\"commandOrdinal\":"; id(s.commandOrdinal);
        out << ",\"sampleTimestamp\":"; id(s.sampleTimestamp);
        out << ",\"age\":"; if (s.ageAvailable) id(s.age); else out << "null";
        out << ",\"preMembership\":"; boolean(s.preMembership);
        out << ",\"state\":"; text(s.state == SampleState::Exact ? "Exact" : s.state == SampleState::Carried ? "Carried" : "Missing");
        out << ",\"dataType\":" << unsigned(s.dataType) << '}';
    }
    void provenance(const CaptureProvenance& p)
    {
        out << "{\"sourcePath\":"; text(p.sourcePath);
        out << ",\"fileHash\":"; text(p.fileHash);
        out << ",\"sdkVersion\":"; text(p.sdkVersion);
        out << ",\"metadata\":"; text(p.metadata); out << '}';
    }
    void selector(const ObjectSelector& s)
    {
        out << "{\"objectId\":"; id(s.objectId);
        out << ",\"name\":"; text(s.name);
        out << ",\"className\":"; text(s.className); out << '}';
    }
    void vector(const double* v, uint32_t count = 3)
    {
        out << '[';
        for (uint32_t i = 0; i != count; ++i) { if (i) out << ','; number(v[i]); }
        out << ']';
    }
    std::ostringstream out;
    bool textWasReplaced = false;
};
}
