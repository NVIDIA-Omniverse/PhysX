// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReader.h"
#include "test_bytes.h"
#include "../../pvdruntime/src/OmniPvdReaderImpl.h"
#include "../../pvdruntime/src/OmniPvdWriterImpl.h"
#include <memory>
#include <string>
#include <vector>

namespace
{
std::vector<std::string> lines;
std::vector<std::string> otherLines;
void OMNI_PVD_CALL capture(char* text) { lines.emplace_back(text); }
void OMNI_PVD_CALL captureOther(char* text) { otherLines.emplace_back(text); }

struct ReaderDeleter
{
    void operator()(OmniPvdReader* reader) const { if (reader) destroyOmniPvdReader(*reader); }
};
using Reader = std::unique_ptr<OmniPvdReader, ReaderDeleter>;

class PvdReaderDiagnostics : public testing::Test
{
protected:
    void SetUp() override { lines.clear(); otherLines.clear(); }
};

TEST_F(PvdReaderDiagnostics, DefaultQuietWithoutExplicitVerbosity)
{
    Bytes bytes;
    bytes.frame(9);
    Reader reader(createOmniPvdReader());
    reader->setReadStream(bytes);
    reader->setLogFunction(capture);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader->getNextCommand());
    EXPECT_TRUE(lines.empty());
    EXPECT_EQ(9u, reader->getFrameTimeStart());
}

TEST_F(PvdReaderDiagnostics, SetLogFunctionOnlyPreservesRejectedHeaderPair)
{
    const uint32_t versions[][3] = {{1, 0, 0}, {0, 5, 0}, {0, 4, 1}};
    const char* reasons[] = {"[parser] major version too new\n", "[parser] minor version too new\n",
        "[parser] patch too new\n"};
    for (unsigned mode = 0; mode < 3; ++mode)
    for (unsigned field = 0; field < 3; ++field)
    {
        SCOPED_TRACE(mode);
        SCOPED_TRACE(field);
        Bytes bytes;
        bytes.data.clear();
        for (uint32_t value : versions[field]) bytes.put(value);
        Reader reader(createOmniPvdReader());
        reader->setReadStream(bytes);
        reader->setLogFunction(capture);
        if (mode)
        {
            OmniPvdReaderDiagnostics options;
            options.logFunction = capture;
            options.verbose = mode == 2;
            setOmniPvdReaderDiagnostics(*reader, options);
        }
        lines.clear();
        uint32_t major = 0, minor = 0, patch = 0;
        EXPECT_FALSE(reader->startReading(major, minor, patch));
        EXPECT_EQ(versions[field][0], major);
        EXPECT_EQ(versions[field][1], minor);
        EXPECT_EQ(versions[field][2], patch);
        const std::string versionLine = "OmniPvdRuntimeReaderImpl::startReading majorVersion(" +
            std::to_string(major) + "), minorVersion(" + std::to_string(minor) + "), patch(" +
            std::to_string(patch) + ")";
        const std::vector<std::string> expected = {versionLine, reasons[field]};
        EXPECT_EQ(expected, lines);
        EXPECT_EQ(OmniPvdReaderStatus::eUNSUPPORTED, getOmniPvdReaderStatus(*reader).code);
        EXPECT_EQ(12u, bytes.position);
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
        EXPECT_FALSE(reader->startReading(major, minor, patch));
        EXPECT_EQ(12u, bytes.position);
        bytes.position = 0;
        EXPECT_FALSE(reader->startReading(major, minor, patch));
        EXPECT_EQ(0u, bytes.position);
        EXPECT_EQ(expected, lines);
    }
}

TEST_F(PvdReaderDiagnostics, ExplicitRestoreAndSetterOrder)
{
    Bytes bytes;
    for (uint64_t time = 1; time <= 6; ++time) bytes.frame(time);
    Reader reader(createOmniPvdReader());
    reader->setReadStream(bytes);
    OmniPvdReaderDiagnostics options;
    EXPECT_EQ(nullptr, options.logFunction);
    EXPECT_FALSE(options.verbose);
    options.logFunction = capture;
    options.verbose = true;
    setOmniPvdReaderDiagnostics(*reader, options);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader->getNextCommand());
    ASSERT_EQ(2u, lines.size()); // Accepted header plus the frame.
    reader->setLogFunction(captureOther);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader->getNextCommand());
    EXPECT_EQ(1u, otherLines.size());
    EXPECT_EQ(2u, lines.size());
    options.verbose = false;
    setOmniPvdReaderDiagnostics(*reader, options);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader->getNextCommand());
    EXPECT_EQ(2u, lines.size());
    EXPECT_EQ(1u, otherLines.size());
    options.verbose = true;
    options.logFunction = nullptr;
    setOmniPvdReaderDiagnostics(*reader, options);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader->getNextCommand());
    EXPECT_EQ(2u, lines.size());
    reader->setLogFunction(capture);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader->getNextCommand());
    EXPECT_EQ(3u, lines.size());
    reader->setLogFunction(nullptr);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader->getNextCommand());
    EXPECT_EQ(3u, lines.size());
    EXPECT_EQ(6u, reader->getFrameTimeStart());
}

TEST_F(PvdReaderDiagnostics, ConfigurationDoesNotReplayOrReset)
{
    Bytes bytes;
    bytes.data[0] = 1;
    Reader reader(createOmniPvdReader());
    reader->setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
    EXPECT_EQ(12u, bytes.position);
    OmniPvdReaderDiagnostics options;
    options.logFunction = capture;
    options.verbose = true;
    setOmniPvdReaderDiagnostics(*reader, options);
    EXPECT_TRUE(lines.empty());
    EXPECT_EQ(OmniPvdReaderStatus::eUNSUPPORTED, getOmniPvdReaderStatus(*reader).code);
    EXPECT_EQ(1u, reader->getMajorVersion());
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
    EXPECT_EQ(12u, bytes.position);
    EXPECT_TRUE(lines.empty());
}

TEST_F(PvdReaderDiagnostics, ReadersStayIndependent)
{
    Bytes first, second;
    first.frame(11);
    second.frame(22);
    Reader a(createOmniPvdReader()), b(createOmniPvdReaderWithLimits(1024));
    a->setReadStream(first);
    b->setReadStream(second);
    a->setLogFunction(capture);
    OmniPvdReaderDiagnostics options;
    options.logFunction = captureOther;
    options.verbose = true;
    setOmniPvdReaderDiagnostics(*b, options);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, a->getNextCommand());
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, b->getNextCommand());
    EXPECT_TRUE(lines.empty());
    EXPECT_EQ(2u, otherLines.size());
    EXPECT_EQ(11u, a->getFrameTimeStart());
    EXPECT_EQ(22u, b->getFrameTimeStart());
}

TEST_F(PvdReaderDiagnostics, OlderHeaderRemainsQuiet)
{
    Bytes bytes;
    bytes.data.clear();
    bytes.put(uint32_t(0)); bytes.put(uint32_t(1)); bytes.put(uint32_t(42));
    Reader reader(createOmniPvdReader());
    reader->setReadStream(bytes);
    reader->setLogFunction(capture);
    uint32_t major = 0, minor = 0, patch = 0;
    EXPECT_TRUE(reader->startReading(major, minor, patch));
    EXPECT_EQ(42u, patch);
    EXPECT_TRUE(lines.empty());
}

void configure(OmniPvdReader& reader, OmniPvdLogFunction sink = capture, bool verbose = true)
{
    OmniPvdReaderDiagnostics options;
    options.logFunction = sink;
    options.verbose = verbose;
    setOmniPvdReaderDiagnostics(reader, options);
}

void rawName(Bytes& bytes, const std::string& text)
{
    bytes.put(static_cast<uint16_t>(text.size()));
    bytes.data.insert(bytes.data.end(), text.begin(), text.end());
}

Bytes textCommand(unsigned field, const std::string& text)
{
    Bytes bytes;
    switch (field)
    {
    case 0:
        bytes.command(OmniPvdCommand::eREGISTER_CLASS); bytes.put(uint32_t(1)); bytes.put(uint32_t(0));
        rawName(bytes, text); break;
    case 1:
        bytes.command(OmniPvdCommand::eREGISTER_ATTRIBUTE); bytes.put(uint32_t(1)); bytes.put(uint32_t(2));
        bytes.put(uint16_t(OmniPvdDataType::eUINT8)); bytes.put(uint32_t(1)); rawName(bytes, text); break;
    case 2:
        bytes.command(OmniPvdCommand::eCREATE_OBJECT); bytes.put(uint64_t(1)); bytes.put(uint32_t(1));
        bytes.put(uint64_t(2)); rawName(bytes, text); break;
    default:
        bytes.command(OmniPvdCommand::eRECORD_MESSAGE); bytes.put(uint64_t(UINT64_MAX));
        rawName(bytes, field == 3 ? text : "plain"); rawName(bytes, field == 4 ? text : "plain");
        bytes.put(uint32_t(4000000000u)); bytes.put(uint32_t(UINT32_MAX)); bytes.put(uint32_t(UINT32_MAX));
        break;
    }
    return bytes;
}

std::vector<std::string> matchingLines(const std::string& text, bool atStart = false)
{
    std::vector<std::string> result;
    for (const auto& line : lines)
    {
        const size_t position = line.find(text);
        if (atStart ? position == 0 : position != std::string::npos) result.push_back(line);
    }
    return result;
}

unsigned number(const std::string& text, const std::string& key)
{
    const size_t start = text.find(key);
    if (start == std::string::npos) { ADD_FAILURE() << "Missing " << key << " in " << text; return 0; }
    return static_cast<unsigned>(std::stoul(text.substr(start + key.size())));
}

std::string unescape(const std::string& text)
{
    std::string decoded;
    for (size_t i = 0; i < text.size(); ++i)
    {
        const unsigned char ch = static_cast<unsigned char>(text[i]);
        if (ch != '\\')
        {
            EXPECT_GE(ch, 0x20); EXPECT_LE(ch, 0x7e); EXPECT_NE('"', ch);
            decoded.push_back(static_cast<char>(ch));
            continue;
        }
        if (++i == text.size()) { ADD_FAILURE() << "Unfinished escape"; break; }
        if (text[i] == '\\' || text[i] == '"') { decoded.push_back(text[i]); continue; }
        if (text[i] != 'x' || i + 2 >= text.size()) { ADD_FAILURE() << "Invalid escape"; break; }
        unsigned value = 0;
        for (unsigned digit = 0; digit < 2; ++digit)
        {
            const char hex = text[++i];
            if (hex >= '0' && hex <= '9') value = 16 * value + unsigned(hex - '0');
            else if (hex >= 'A' && hex <= 'F') value = 16 * value + unsigned(hex - 'A' + 10);
            else { ADD_FAILURE() << "Invalid hex"; return {}; }
        }
        decoded.push_back(static_cast<char>(value));
    }
    return decoded;
}

void expectTextChunks(const std::vector<std::string>& chunks, const std::string& original)
{
    const size_t retained = (std::min)(original.size(), size_t(2047));
    const size_t count = (std::max)(size_t(1), (retained + 383) / 384);
    ASSERT_EQ(count, chunks.size());
    std::string decoded;
    for (size_t i = 0; i < chunks.size(); ++i)
    {
        const auto& line = chunks[i];
        EXPECT_LT(line.size(), 2048u);
        EXPECT_EQ(std::string::npos, line.find('\n'));
        EXPECT_EQ(std::string::npos, line.find('\r'));
        EXPECT_EQ(i + 1, number(line, "chunk="));
        EXPECT_NE(std::string::npos, line.find("/" + std::to_string(count) + " offset="));
        EXPECT_EQ(i * 384, number(line, "offset="));
        EXPECT_EQ((std::min)(size_t(384), retained - i * 384), number(line, "bytes="));
        EXPECT_EQ(original.size(), number(line, "encoded="));
        EXPECT_EQ(retained, number(line, "retained="));
        EXPECT_EQ(original.size() > retained ? 1u : 0u, number(line, "truncated="));
        const size_t value = line.find(" value=\"");
        ASSERT_NE(std::string::npos, value);
        ASSERT_EQ('"', line.back());
        decoded += unescape(line.substr(value + 8, line.size() - value - 9));
    }
    EXPECT_EQ(original.substr(0, retained), decoded);
}

TEST_F(PvdReaderDiagnostics, EscapesBytesAndNeverUsesDataAsFormat)
{
    const char* fields[] = {"className", "attributeName", "objectName", "message", "file"};
    std::string pattern = "%s%n%%\"\\\n[parser] trace command=FORGED\n";
    for (unsigned byte = 0; byte <= 255; ++byte) pattern.push_back(static_cast<char>(byte));
    for (unsigned field = 0; field < 5; ++field)
    for (size_t length : {size_t(0), size_t(1), size_t(383), size_t(384), size_t(385), size_t(2047), size_t(2048), size_t(65535)})
    {
        SCOPED_TRACE(field);
        SCOPED_TRACE(length);
        std::string text;
        while (text.size() < length) text += pattern;
        text.resize(length);
        if (length) text.back() = '"';
        Bytes bytes = textCommand(field, text);
        Reader reader(createOmniPvdReader()); reader->setReadStream(bytes); configure(*reader);
        lines.clear();
        ASSERT_NE(OmniPvdCommand::eINVALID, reader->getNextCommand());
        EXPECT_EQ(bytes.data.size(), bytes.position);
        EXPECT_EQ(1u, matchingLines("[parser] trace command=", true).size());
        expectTextChunks(matchingLines(std::string("field=") + fields[field] + " "), text);
    }
}

TEST_F(PvdReaderDiagnostics, AllEscapeRetentionBoundHasSixCompleteChunks)
{
    const std::string text(2047, char(0x80));
    Bytes bytes = textCommand(3, text);
    Reader reader(createOmniPvdReader()); reader->setReadStream(bytes); configure(*reader);
    ASSERT_EQ(OmniPvdCommand::eRECORD_MESSAGE, reader->getNextCommand());
    const auto chunks = matchingLines("field=message ");
    expectTextChunks(chunks, text);
    ASSERT_EQ(6u, chunks.size());
    size_t total = 0;
    for (const auto& line : chunks) total += line.size() - line.find(" value=\"") - 9;
    EXPECT_EQ(8188u, total);
    EXPECT_NE(std::string::npos, chunks.back().find("offset=1920 bytes=127"));
    const auto summaries = matchingLines("[parser] trace command=RECORD_MESSAGE", true);
    ASSERT_EQ(1u, summaries.size());
    EXPECT_NE(std::string::npos, summaries[0].find("context=18446744073709551615"));
    EXPECT_NE(std::string::npos, summaries[0].find("line=4000000000 type=4294967295 class=4294967295"));
}

TEST_F(PvdReaderDiagnostics, LongestTextLabelRetainsEveryEscapedByteAndClosingQuote)
{
    const std::string text(2047, char(0x80));
    Bytes bytes;
    bytes.list(UINT32_MAX, text.c_str(), 4000000001u);
    Reader reader(createOmniPvdReader()); reader->setReadStream(bytes); configure(*reader);
    ASSERT_EQ(OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE, reader->getNextCommand());
    EXPECT_EQ(bytes.data.size(), bytes.position);
    ASSERT_EQ(8u, lines.size());
    EXPECT_EQ("[parser] trace command=REGISTER_UNIQUE_LIST_ATTRIBUTE opcode=5 class=4000000001 attribute=4294967295 type=OBJECT_HANDLE typeId=11", lines[1]);
    const std::vector<std::string> chunks(lines.begin() + 2, lines.end());
    expectTextChunks(chunks, text);
    for (size_t i = 0; i < chunks.size(); ++i)
    {
        const size_t count = i == 5 ? 127 : 384;
        std::string escaped;
        for (size_t j = 0; j < count; ++j) escaped += "\\x80";
        EXPECT_EQ("[parser] text command=REGISTER_UNIQUE_LIST_ATTRIBUTE field=attributeName chunk=" +
            std::to_string(i + 1) + "/6 offset=" + std::to_string(i * 384) + " bytes=" + std::to_string(count) +
            " encoded=2047 retained=2047 truncated=0 value=\"" + escaped + "\"", chunks[i]);
        EXPECT_LE(chunks[i].size(), 1689u);
    }
    EXPECT_EQ(text, std::string(reader->getAttributeName(), text.size()));
}

Bytes allCommands()
{
    Bytes bytes;
    bytes.cls(UINT32_MAX, "Thing", 4000000001u);
    bytes.attr(12, "values", 11, OmniPvdDataType::eFLOAT64, 3);
    bytes.attr(13, "enum", 11, OmniPvdDataType::eENUM_VALUE, UINT32_MAX);
    bytes.attr(14, "flags", 11, OmniPvdDataType::eFLAGS_WORD, 17);
    bytes.command(OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE);
    bytes.put(uint32_t(11)); bytes.put(uint32_t(15)); bytes.put(uint32_t(19)); bytes.name("child");
    bytes.list(16, "items", 11);
    bytes.set(4, UINT32_MAX, UINT64_MAX, 32, 4294967297ull);
    bytes.reference(UINT64_MAX, 16, 1, true, 4294967298ull);
    bytes.removeReference(UINT64_MAX, 16, 1, 4294967299ull);
    bytes.create(UINT64_MAX, 11, 4294967300ull);
    bytes.command(OmniPvdCommand::eDESTROY_OBJECT); bytes.put(uint64_t(4294967301ull)); bytes.put(uint64_t(UINT64_MAX));
    bytes.frame(UINT64_MAX, false, 4294967302ull);
    bytes.frame(uint64_t(1) << 40, true, 4294967303ull);
    Bytes message = textCommand(3, "message%s%n%%");
    bytes.data.insert(bytes.data.end(), message.data.begin() + 12, message.data.end());
    return bytes;
}

TEST_F(PvdReaderDiagnostics, EveryOpcodeReportsRelevantFields)
{
    Bytes bytes = allCommands();
    Reader reader(createOmniPvdReader()); reader->setReadStream(bytes); configure(*reader);
    while (reader->getNextCommand() != OmniPvdCommand::eINVALID) {}
    EXPECT_EQ(bytes.data.size(), bytes.position);
    EXPECT_EQ(OmniPvdReaderStatus::eEND_OF_STREAM, getOmniPvdReaderStatus(*reader).code);
    std::string path = "path=[";
    for (unsigned i = 0; i < 32; ++i) path += i ? ",4294967295" : "4294967295";
    path += "]";
    const std::vector<std::string> expected = {
        "OmniPvdRuntimeReaderImpl::startReading majorVersion(0), minorVersion(4), patch(0)",
        "[parser] trace command=REGISTER_CLASS opcode=1 class=4294967295 base=4000000001",
        "[parser] text command=REGISTER_CLASS field=className chunk=1/1 offset=0 bytes=5 encoded=5 retained=5 truncated=0 value=\"Thing\"",
        "[parser] trace command=REGISTER_ATTRIBUTE opcode=3 class=11 attribute=12 type=FLOAT64 typeId=9 elements=3",
        "[parser] text command=REGISTER_ATTRIBUTE field=attributeName chunk=1/1 offset=0 bytes=6 encoded=6 retained=6 truncated=0 value=\"values\"",
        "[parser] trace command=REGISTER_ATTRIBUTE opcode=3 class=11 attribute=13 type=ENUM_VALUE typeId=12 enumValue=4294967295",
        "[parser] text command=REGISTER_ATTRIBUTE field=attributeName chunk=1/1 offset=0 bytes=4 encoded=4 retained=4 truncated=0 value=\"enum\"",
        "[parser] trace command=REGISTER_ATTRIBUTE opcode=3 class=11 attribute=14 type=FLAGS_WORD typeId=13 enumClass=17",
        "[parser] text command=REGISTER_ATTRIBUTE field=attributeName chunk=1/1 offset=0 bytes=5 encoded=5 retained=5 truncated=0 value=\"flags\"",
        "[parser] trace command=REGISTER_CLASS_ATTRIBUTE opcode=4 class=11 attribute=15 attributeClass=19",
        "[parser] text command=REGISTER_CLASS_ATTRIBUTE field=attributeName chunk=1/1 offset=0 bytes=5 encoded=5 retained=5 truncated=0 value=\"child\"",
        "[parser] trace command=REGISTER_UNIQUE_LIST_ATTRIBUTE opcode=5 class=11 attribute=16 type=OBJECT_HANDLE typeId=11",
        "[parser] text command=REGISTER_UNIQUE_LIST_ATTRIBUTE field=attributeName chunk=1/1 offset=0 bytes=5 encoded=5 retained=5 truncated=0 value=\"items\"",
        "[parser] trace command=SET_ATTRIBUTE opcode=6 context=4294967297 object=18446744073709551615 depth=32 " + path + " bytes=4",
        "[parser] trace command=ADD_TO_UNIQUE_LIST_ATTRIBUTE opcode=7 context=4294967298 object=18446744073709551615 depth=1 path=[16] bytes=8",
        "[parser] trace command=REMOVE_FROM_UNIQUE_LIST_ATTRIBUTE opcode=8 context=4294967299 object=18446744073709551615 depth=1 path=[16] bytes=8",
        "[parser] trace command=CREATE_OBJECT opcode=9 context=4294967300 object=18446744073709551615 class=11",
        "[parser] text command=CREATE_OBJECT field=objectName chunk=1/1 offset=0 bytes=0 encoded=0 retained=0 truncated=0 value=\"\"",
        "[parser] trace command=DESTROY_OBJECT opcode=10 context=4294967301 object=18446744073709551615",
        "[parser] trace command=START_FRAME opcode=11 context=4294967302 timestamp=18446744073709551615",
        "[parser] trace command=STOP_FRAME opcode=12 context=4294967303 timestamp=1099511627776",
        "[parser] trace command=RECORD_MESSAGE opcode=13 context=18446744073709551615 line=4000000000 type=4294967295 class=4294967295",
        "[parser] text command=RECORD_MESSAGE field=message chunk=1/1 offset=0 bytes=13 encoded=13 retained=13 truncated=0 value=\"message%s%n%%\"",
        "[parser] text command=RECORD_MESSAGE field=file chunk=1/1 offset=0 bytes=5 encoded=5 retained=5 truncated=0 value=\"plain\""
    };
    EXPECT_EQ(expected, lines);
}

TEST_F(PvdReaderDiagnostics, EveryValidatedDataTypeHasAReadableName)
{
    const char* names[] = {"INT8", "INT16", "INT32", "INT64", "UINT8", "UINT16", "UINT32", "UINT64", "FLOAT32", "FLOAT64",
        "STRING", "OBJECT_HANDLE", "ENUM_VALUE", "FLAGS_WORD"};
    for (unsigned type = 0; type < 14; ++type)
    {
        Bytes bytes; bytes.attr(2, "value", 1, static_cast<OmniPvdDataType::Enum>(type), 7);
        Reader reader(createOmniPvdReader()); reader->setReadStream(bytes); configure(*reader);
        lines.clear();
        ASSERT_EQ(OmniPvdCommand::eREGISTER_ATTRIBUTE, reader->getNextCommand());
        ASSERT_EQ(1u, matchingLines(std::string("type=") + names[type] + " typeId=" + std::to_string(type)).size());
    }
}

TEST_F(PvdReaderDiagnostics, OldClassFormatDoesNotInventABaseHandle)
{
    Bytes bytes;
    bytes.data[4] = 2;
    bytes.command(OmniPvdCommand::eREGISTER_CLASS); bytes.put(uint32_t(7)); bytes.name("old");
    Reader reader(createOmniPvdReader()); reader->setReadStream(bytes); configure(*reader);
    ASSERT_EQ(OmniPvdCommand::eREGISTER_CLASS, reader->getNextCommand());
    const auto summaries = matchingLines("[parser] trace command=REGISTER_CLASS", true);
    ASSERT_EQ(1u, summaries.size());
    EXPECT_EQ(std::string::npos, summaries[0].find("base="));
    EXPECT_NE(std::string::npos, summaries[0].find("class=7"));
}

void expectSameValues(OmniPvdReader& a, OmniPvdReader& b)
{
    EXPECT_EQ(a.getMajorVersion(), b.getMajorVersion());
    EXPECT_EQ(a.getMinorVersion(), b.getMinorVersion());
    EXPECT_EQ(a.getPatch(), b.getPatch());
    EXPECT_EQ(a.getContextHandle(), b.getContextHandle());
    EXPECT_EQ(a.getObjectHandle(), b.getObjectHandle());
    EXPECT_EQ(a.getClassHandle(), b.getClassHandle());
    EXPECT_EQ(a.getBaseClassHandle(), b.getBaseClassHandle());
    EXPECT_EQ(a.getAttributeHandle(), b.getAttributeHandle());
    EXPECT_STREQ(a.getClassName(), b.getClassName());
    EXPECT_STREQ(a.getAttributeName(), b.getAttributeName());
    EXPECT_STREQ(a.getObjectName(), b.getObjectName());
    EXPECT_EQ(a.getAttributeDataType(), b.getAttributeDataType());
    EXPECT_EQ(a.getAttributeDataLength(), b.getAttributeDataLength());
    EXPECT_EQ(a.getAttributeNumberElements(), b.getAttributeNumberElements());
    EXPECT_EQ(a.getAttributeClassHandle(), b.getAttributeClassHandle());
    EXPECT_EQ(a.getEnumClassHandle(), b.getEnumClassHandle());
    EXPECT_EQ(a.getEnumValue(), b.getEnumValue());
    EXPECT_EQ(a.getFrameTimeStart(), b.getFrameTimeStart());
    EXPECT_EQ(a.getFrameTimeStop(), b.getFrameTimeStop());
    const auto& ai = static_cast<const OmniPvdReaderImpl&>(a);
    const auto& bi = static_cast<const OmniPvdReaderImpl&>(b);
    EXPECT_EQ(ai.mCmdAttributeHandleDepth, bi.mCmdAttributeHandleDepth);
    for (unsigned i = 0; i < ai.mCmdAttributeHandleDepth && i < 32; ++i)
        EXPECT_EQ(ai.mCmdAttributeHandleStack[i], bi.mCmdAttributeHandleStack[i]);
    if (a.getAttributeDataLength() && a.getAttributeDataPointer() && b.getAttributeDataPointer())
        EXPECT_EQ(0, std::memcmp(a.getAttributeDataPointer(), b.getAttributeDataPointer(), a.getAttributeDataLength()));
    const char *am = nullptr, *af = nullptr, *bm = nullptr, *bf = nullptr;
    uint32_t al = 0, at = 0, ah = 0, bl = 0, bt = 0, bh = 0;
    EXPECT_EQ(a.getMessageData(am, af, al, at, ah), b.getMessageData(bm, bf, bl, bt, bh));
    EXPECT_STREQ(am, bm); EXPECT_STREQ(af, bf);
    EXPECT_EQ(al, bl); EXPECT_EQ(at, bt); EXPECT_EQ(ah, bh);
}

TEST_F(PvdReaderDiagnostics, TracePreservesDecodeAndConsumption)
{
    Bytes quiet = allCommands(), verbose = quiet, noSink = quiet;
    Reader a(createOmniPvdReader()), b(createOmniPvdReader()), c(createOmniPvdReader());
    a->setReadStream(quiet); b->setReadStream(verbose); c->setReadStream(noSink);
    configure(*a, captureOther, false); configure(*b); configure(*c, nullptr);
    unsigned decoded = 0;
    for (;;)
    {
        const auto command = a->getNextCommand();
        ASSERT_EQ(command, b->getNextCommand());
        ASSERT_EQ(command, c->getNextCommand());
        EXPECT_EQ(quiet.position, verbose.position); EXPECT_EQ(quiet.position, noSink.position);
        EXPECT_EQ(getOmniPvdReaderStatus(*a).code, getOmniPvdReaderStatus(*b).code);
        EXPECT_EQ(getOmniPvdReaderStatus(*a).code, getOmniPvdReaderStatus(*c).code);
        expectSameValues(*a, *b); expectSameValues(*a, *c);
        if (command == OmniPvdCommand::eINVALID) break;
        ++decoded;
    }
    EXPECT_EQ(14u, decoded);
    EXPECT_EQ(14u, matchingLines("[parser] trace command=", true).size());
    EXPECT_TRUE(otherLines.empty());
}

TEST_F(PvdReaderDiagnostics, PartialCommandsHaveNoSuccessTrace)
{
    std::vector<Bytes> inputs;
    for (unsigned field = 0; field < 5; ++field) inputs.push_back(textCommand(field, "message%s%n%%"));
    Bytes payload; payload.set(17); inputs.push_back(payload);
    for (const auto& input : inputs)
    for (size_t cut = 12; cut < input.data.size(); ++cut)
    for (bool verbose : {false, true})
    {
        SCOPED_TRACE(cut);
        SCOPED_TRACE(verbose);
        Bytes bytes = input; bytes.data.resize(cut);
        Reader reader(createOmniPvdReader()); reader->setReadStream(bytes); configure(*reader, capture, verbose);
        lines.clear();
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
        std::vector<std::string> expected;
        if (verbose) expected.emplace_back("OmniPvdRuntimeReaderImpl::startReading majorVersion(0), minorVersion(4), patch(0)");
        if (cut > 12) expected.emplace_back("[parser] ERROR: failed to read complete field\n");
        EXPECT_EQ(expected, lines);
        EXPECT_EQ(cut, bytes.position);
        if (cut > 12)
        {
            EXPECT_EQ(OmniPvdReaderStatus::eINVALID, getOmniPvdReaderStatus(*reader).code);
            const auto stopped = lines;
            EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
            EXPECT_EQ(stopped, lines);
            EXPECT_EQ(cut, bytes.position);
        }
        const char *message = nullptr, *file = nullptr;
        uint32_t line = 0, type = 0, cls = 0;
        EXPECT_FALSE(reader->getMessageData(message, file, line, type, cls));
    }
    for (bool verbose : {false, true})
    {
        Bytes longTail = textCommand(3, std::string(2049, 'x'));
        // Cut inside the two discarded message bytes, before the file and message metadata.
        longTail.data.resize(12 + 1 + 8 + 2 + 2048);
        Reader reader(createOmniPvdReader()); reader->setReadStream(longTail); configure(*reader, capture, verbose);
        lines.clear();
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
        std::vector<std::string> expected;
        if (verbose) expected.emplace_back("OmniPvdRuntimeReaderImpl::startReading majorVersion(0), minorVersion(4), patch(0)");
        expected.emplace_back("[parser] ERROR: failed to read complete field\n");
        EXPECT_EQ(expected, lines);
        EXPECT_EQ(longTail.data.size(), longTail.position);
        EXPECT_EQ(OmniPvdReaderStatus::eINVALID, getOmniPvdReaderStatus(*reader).code);
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
        EXPECT_EQ(expected, lines);
        EXPECT_EQ(longTail.data.size(), longTail.position);
    }
}

TEST_F(PvdReaderDiagnostics, TerminalErrorTextIsIndependentOfVerbosity)
{
    struct Case { Bytes bytes; const char* reason; OmniPvdReaderStatus::Code status; };
    std::vector<Case> cases;
    Bytes unknown; unknown.put(uint8_t(255));
    cases.push_back({unknown, "[parser] ERROR: unknown command 255\n", OmniPvdReaderStatus::eINVALID});
    Bytes unsupported; unsupported.command(OmniPvdCommand::eREGISTER_ENUM);
    cases.push_back({unsupported, "[parser] ERROR: unsupported command eREGISTER_ENUM\n", OmniPvdReaderStatus::eUNSUPPORTED});
    Bytes invalidType; invalidType.attr(1, "bad", 1, static_cast<OmniPvdDataType::Enum>(14));
    cases.push_back({invalidType, "[parser] ERROR: invalid attribute data type\n", OmniPvdReaderStatus::eINVALID});
    Bytes zero; zero.set(0, 1, 10, 0);
    cases.push_back({zero, "[parser] ERROR: zero attribute path depth\n", OmniPvdReaderStatus::eINVALID});
    Bytes deep; deep.set(0, 1, 10, 33);
    cases.push_back({deep, "[parser] ERROR: attribute path depth 33 exceeds limit 32\n", OmniPvdReaderStatus::eLIMIT});
    Bytes cap; cap.set(5);
    cases.push_back({cap, "[parser] ERROR: attribute payload 5 bytes exceeds limit 4\n", OmniPvdReaderStatus::eLIMIT});
    for (const auto& item : cases)
    for (bool verbose : {false, true})
    {
        SCOPED_TRACE(item.reason);
        Bytes bytes = item.bytes;
        Reader reader(createOmniPvdReaderWithLimits(4)); reader->setReadStream(bytes); configure(*reader, capture, verbose);
        lines.clear();
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
        EXPECT_EQ(item.status, getOmniPvdReaderStatus(*reader).code);
        std::vector<std::string> expected;
        if (verbose) expected.emplace_back("OmniPvdRuntimeReaderImpl::startReading majorVersion(0), minorVersion(4), patch(0)");
        expected.emplace_back(item.reason);
        EXPECT_EQ(expected, lines);
        const size_t stopped = bytes.position;
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
        EXPECT_EQ(stopped, bytes.position);
        EXPECT_EQ(expected, lines);
    }
}

TEST_F(PvdReaderDiagnostics, WriterOutputRemainsEnabledByItsCallback)
{
    EncodedStreamBytes bytes;
    OmniPvdWriterImpl writer;
    writer.setWriteStream(bytes);
    writer.setLogFunction(capture);
    writer.registerClass("writer-name", 0);
    EXPECT_FALSE(lines.empty());
}
}
