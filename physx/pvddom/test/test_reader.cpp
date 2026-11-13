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
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace
{
TEST(PvdNativeReader, RejectsTruncatedFixedField)
{
    Bytes bytes;
    bytes.command(OmniPvdCommand::eSTART_FRAME);
    bytes.put(uint32_t(1));
    OmniPvdReader* reader = createOmniPvdReader();
    reader->setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
    destroyOmniPvdReader(*reader);
}

TEST(PvdNativeReader, PreservesAttributePath)
{
    Bytes bytes;
    bytes.set(4, 1234);
    OmniPvdReader* reader = createOmniPvdReader();
    reader->setReadStream(bytes);
    ASSERT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, reader->getNextCommand());
    EXPECT_EQ(1234u, static_cast<OmniPvdReaderImpl*>(reader)->getAttributeHandles()[0]);
    destroyOmniPvdReader(*reader);
}

TEST(PvdNativeReader, CanResumeAtCleanCommandBoundary)
{
    Bytes bytes;
    OmniPvdReader* reader = createOmniPvdReader();
    reader->setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
    bytes.frame(9);
    EXPECT_EQ(OmniPvdCommand::eSTART_FRAME, reader->getNextCommand());
    EXPECT_EQ(9u, reader->getFrameTimeStart());
    destroyOmniPvdReader(*reader);
}

TEST(PvdNativeReader, FailedOpenCanBeRetried)
{
    struct RetryBytes : Bytes
    {
        bool ready = false;
        bool OMNI_PVD_CALL openStream() override { return ready; }
    } bytes;
    bytes.frame(9);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    OmniPvdVersionType major = 0, minor = 0, patch = 0;
    EXPECT_FALSE(reader.startReading(major, minor, patch));
    EXPECT_EQ(0u, bytes.position);
    bytes.ready = true;
    ASSERT_TRUE(reader.startReading(major, minor, patch));
    EXPECT_EQ(0u, major);
    EXPECT_EQ(4u, minor);
    EXPECT_EQ(0u, patch);
    EXPECT_EQ(12u, bytes.position);
    EXPECT_EQ(OmniPvdCommand::eSTART_FRAME, reader.getNextCommand());
    EXPECT_EQ(9u, reader.getFrameTimeStart());
}

TEST(PvdNativeReader, TruncatedHeaderCanBeRetriedAfterRewind)
{
    Bytes bytes;
    bytes.data.resize(8);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    OmniPvdVersionType major = 0, minor = 0, patch = 0;
    EXPECT_FALSE(reader.startReading(major, minor, patch));
    bytes.put(uint32_t(0));
    bytes.position = 0;
    ASSERT_TRUE(reader.startReading(major, minor, patch));
    EXPECT_EQ(12u, bytes.position);
    EXPECT_EQ(4u, minor);
    EXPECT_EQ(OmniPvdReaderStatus::eNONE, reader.mStatus);
}

TEST(PvdNativeReader, EmptyUnstartedStreamCanReceiveHeaderAndCommand)
{
    Bytes bytes;
    bytes.data.clear();
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_NE(OmniPvdReaderStatus::eINVALID, reader.mStatus);
    bytes = Bytes();
    bytes.frame(9);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader.getNextCommand());
    EXPECT_EQ(9u, reader.getFrameTimeStart());
}

TEST(PvdNativeReader, RejectsRawInvalidDataType)
{
    for (const auto command : {OmniPvdCommand::eREGISTER_ATTRIBUTE, OmniPvdCommand::eREGISTER_UNIQUE_LIST_ATTRIBUTE})
    for (const uint16_t type : {uint16_t(0xffff), uint16_t(OmniPvdDataType::eFLAGS_WORD + 1)})
    {
        SCOPED_TRACE(command);
        Bytes bytes;
        bytes.command(command);
        bytes.put(uint32_t(1)); bytes.put(uint32_t(1)); bytes.put(type);
        if (command == OmniPvdCommand::eREGISTER_ATTRIBUTE) bytes.put(uint32_t(1));
        bytes.name("invalid");
        OmniPvdReaderImpl reader;
        reader.setReadStream(bytes);
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
        EXPECT_EQ(OmniPvdReaderStatus::eINVALID, reader.mStatus);
        EXPECT_EQ(23u, bytes.position);
    }
}

TEST(PvdNativeReader, FailedImplicitHeaderIsStickyUntilExplicitRetry)
{
    Bytes bytes; bytes.data.resize(8);
    OmniPvdReaderImpl reader; reader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    bytes.put(uint32_t(255)); bytes.put(uint32_t(0)); bytes.put(uint32_t(0));
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eINVALID, reader.mStatus);
    EXPECT_EQ(8u, bytes.position);
    bytes = Bytes(); bytes.frame(9);
    OmniPvdVersionType major = 0, minor = 0, patch = 0;
    ASSERT_TRUE(reader.startReading(major, minor, patch));
    EXPECT_EQ(OmniPvdCommand::eSTART_FRAME, reader.getNextCommand());
}

TEST(PvdNativeReader, TruncatedMessageCannotBeRetrieved)
{
    Bytes bytes; bytes.command(OmniPvdCommand::eRECORD_MESSAGE); bytes.put(uint64_t(1));
    bytes.name("message"); bytes.name("file"); bytes.put(uint32_t(7));
    OmniPvdReaderImpl reader; reader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    const char* text = nullptr; const char* file = nullptr;
    uint32_t line = 0, type = 0; OmniPvdClassHandle handle = 0;
    EXPECT_FALSE(reader.getMessageData(text, file, line, type, handle));
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eINVALID, reader.mStatus);
}

TEST(PvdNativeReader, RejectedPathDepthSurvivesStickyFailure)
{
    Bytes bytes; bytes.set(4, 1, 10, 33);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(33u, reader.mCmdAttributeHandleDepth);
    const auto consumed = bytes.position;
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eLIMIT, reader.mStatus);
    EXPECT_EQ(33u, reader.mCmdAttributeHandleDepth);
    EXPECT_EQ(consumed, bytes.position);
}

std::vector<std::string> readerErrors;
void OMNI_PVD_CALL collectReaderErrors(char* text)
{
    if (strstr(text, "ERROR")) readerErrors.emplace_back(text);
}

TEST(PvdNativeReader, EveryInvalidRawOpcodeIsStickyAndLogsOnce)
{
    for (unsigned opcode = 0; opcode <= 255; ++opcode)
    {
        if (opcode >= OmniPvdCommand::eREGISTER_CLASS && opcode <= OmniPvdCommand::eRECORD_MESSAGE)
            continue;
        SCOPED_TRACE(opcode);
        Bytes bytes; bytes.put(static_cast<uint8_t>(opcode)); bytes.frame(9);
        OmniPvdReaderImpl reader; reader.setReadStream(bytes); reader.setLogFunction(collectReaderErrors);
        readerErrors.clear();
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
        EXPECT_EQ(OmniPvdReaderStatus::eINVALID, reader.mStatus);
        EXPECT_EQ(13u, bytes.position);
        ASSERT_EQ(1u, readerErrors.size());
        EXPECT_NE(std::string::npos, readerErrors[0].find("unknown command " + std::to_string(opcode)));
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
        EXPECT_EQ(13u, bytes.position);
        EXPECT_EQ(1u, readerErrors.size());
    }
}

TEST(PvdNativeReader, StickyStopsReportSpecificReasonOnce)
{
    const struct
    {
        const char* reason;
        uint8_t command;
        uint8_t depth;
        uint32_t payloadBytes;
        OmniPvdReaderStatus::Code status;
        const char* details[3];
    } cases[] = {
        {"deep path", OmniPvdCommand::eSET_ATTRIBUTE, 5, 4, OmniPvdReaderStatus::eLIMIT,
            {"path depth 5", "limit 3", nullptr}},
        {"zero path", OmniPvdCommand::eSET_ATTRIBUTE, 0, 4, OmniPvdReaderStatus::eINVALID,
            {"zero", "path depth", nullptr}},
        {"unsupported enum", OmniPvdCommand::eREGISTER_ENUM, 1, 4, OmniPvdReaderStatus::eUNSUPPORTED,
            {"unsupported", "eREGISTER_ENUM", nullptr}},
        {"unknown opcode", 255, 1, 4, OmniPvdReaderStatus::eINVALID,
            {"unknown", "255", nullptr}},
        {"large payload", OmniPvdCommand::eSET_ATTRIBUTE, 1, 19, OmniPvdReaderStatus::eLIMIT,
            {"payload", "19 bytes", "limit 17"}}
    };
    for (const auto& test : cases)
    {
        SCOPED_TRACE(test.reason);
        Bytes bytes;
        if (test.command == OmniPvdCommand::eSET_ATTRIBUTE) bytes.set(test.payloadBytes, 1, 10, test.depth);
        else bytes.put(test.command);
        OmniPvdReaderImpl reader(17, 3);
        reader.setReadStream(bytes);
        reader.setLogFunction(collectReaderErrors);
        readerErrors.clear();
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
        EXPECT_EQ(test.status, reader.mStatus);
        EXPECT_EQ(1u, readerErrors.size());
        if (readerErrors.size() == 1)
        {
            for (const char* detail : test.details)
                if (detail) EXPECT_NE(std::string::npos, readerErrors[0].find(detail)) << readerErrors[0];
        }
        const size_t stopped = bytes.position;
        bytes.frame(9);
        for (unsigned retry = 0; retry < 3; ++retry)
        {
            EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
            EXPECT_EQ(test.status, reader.mStatus);
            EXPECT_EQ(stopped, bytes.position);
            EXPECT_EQ(1u, readerErrors.size());
        }
    }
}

TEST(PvdNativeReader, ShortFieldReportsOnceAndStaysInvalid)
{
    Bytes bytes;
    bytes.command(OmniPvdCommand::eSTART_FRAME); bytes.put(uint32_t(1));
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes); reader.setLogFunction(collectReaderErrors);
    readerErrors.clear();
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    ASSERT_EQ(1u, readerErrors.size());
    const size_t consumed = bytes.position;
    bytes.frame(9);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(consumed, bytes.position);
    EXPECT_EQ(1u, readerErrors.size());
}

TEST(PvdNativeReader, ShortHeaderReportsOnceAcrossRetries)
{
    Bytes bytes; bytes.data.resize(11);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes); reader.setLogFunction(collectReaderErrors);
    readerErrors.clear();
    OmniPvdVersionType major = 0, minor = 0, patch = 0;
    EXPECT_FALSE(reader.startReading(major, minor, patch));
    EXPECT_EQ(1u, readerErrors.size());
    EXPECT_FALSE(reader.startReading(major, minor, patch));
    EXPECT_EQ(1u, readerErrors.size());
}

TEST(PvdNativeReader, CleanEofAndResumeDoNotReportErrors)
{
    Bytes bytes;
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes); reader.setLogFunction(collectReaderErrors);
    readerErrors.clear();
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    bytes.frame(9);
    EXPECT_EQ(OmniPvdCommand::eSTART_FRAME, reader.getNextCommand());
    EXPECT_TRUE(readerErrors.empty());
}

class PvdNativeStringBoundary : public testing::TestWithParam<size_t> {};

TEST_P(PvdNativeStringBoundary, WriterNamesRemainDecodable)
{
    EncodedStreamBytes bytes;
    OmniPvdWriterImpl writer;
    writer.setWriteStream(bytes);
    const std::string name(GetParam(), 'n');
    const auto cls = writer.registerClass(name.c_str(), 0);
    writer.registerAttribute(cls, name.c_str(), OmniPvdDataType::eUINT32, 1);
    writer.createObject(1, cls, 10, name.c_str());
    writer.startFrame(1, 9);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    const std::string prefix(2047, 'n');
    ASSERT_EQ(OmniPvdCommand::eREGISTER_CLASS, reader.getNextCommand());
    EXPECT_EQ(prefix, reader.getClassName());
    ASSERT_EQ(OmniPvdCommand::eREGISTER_ATTRIBUTE, reader.getNextCommand());
    EXPECT_EQ(prefix, reader.getAttributeName());
    ASSERT_EQ(OmniPvdCommand::eCREATE_OBJECT, reader.getNextCommand());
    EXPECT_EQ(prefix, reader.getObjectName());
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader.getNextCommand());
    EXPECT_EQ(9u, reader.getFrameTimeStart());
    EXPECT_EQ(bytes.data.size(), bytes.position);
}

TEST_P(PvdNativeStringBoundary, WriterMessageAndPathRemainDecodable)
{
    EncodedStreamBytes bytes;
    OmniPvdWriterImpl writer;
    writer.setWriteStream(bytes);
    const std::string message(GetParam(), 'm'), path(GetParam(), 'p');
    writer.recordMessage(1, message.c_str(), path.c_str(), 42, 7, 3);
    writer.startFrame(1, 9);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    ASSERT_EQ(OmniPvdCommand::eRECORD_MESSAGE, reader.getNextCommand());
    const char* decodedMessage = nullptr;
    const char* decodedPath = nullptr;
    uint32_t line = 0, type = 0;
    OmniPvdClassHandle handle = 0;
    ASSERT_TRUE(reader.getMessageData(decodedMessage, decodedPath, line, type, handle));
    EXPECT_EQ(std::string(2047, 'm'), decodedMessage);
    EXPECT_EQ(std::string(2047, 'p'), decodedPath);
    EXPECT_EQ(42u, line);
    EXPECT_EQ(7u, type);
    EXPECT_EQ(3u, handle);
    ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader.getNextCommand());
    EXPECT_EQ(9u, reader.getFrameTimeStart());
    EXPECT_EQ(bytes.data.size(), bytes.position);
}

INSTANTIATE_TEST_SUITE_P(WriterBoundary, PvdNativeStringBoundary, testing::Values(size_t(2047), size_t(2048), size_t(2049)));

TEST(PvdNativeReader, LegacyLongNamesKeepPrefixAndDecodeFollowingCommand)
{
    for (size_t length : {size_t(2049), size_t(65535)})
    {
        Bytes bytes; bytes.cls(1, std::string(length, 'n').c_str());
        ASSERT_EQ(23u + length, bytes.data.size());
        bytes.frame(9);
        OmniPvdReaderImpl reader;
        reader.setReadStream(bytes);
        ASSERT_EQ(OmniPvdCommand::eREGISTER_CLASS, reader.getNextCommand());
        EXPECT_EQ(std::string(2047, 'n'), reader.getClassName());
        ASSERT_EQ(OmniPvdCommand::eSTART_FRAME, reader.getNextCommand());
        EXPECT_EQ(9u, reader.getFrameTimeStart());
        EXPECT_EQ(bytes.data.size(), bytes.position);
    }
}

TEST(PvdNativeReader, TruncatedLongNameDiscardIsInvalid)
{
    Bytes bytes; bytes.cls(1, std::string(4096, 'n').c_str()); bytes.data.pop_back();
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eINVALID, reader.mStatus);
    EXPECT_EQ(bytes.data.size(), bytes.position);
    EXPECT_STREQ("", reader.getClassName());
}

TEST(PvdNativeReader, TruncatedNameClearsOldBuffer)
{
    Bytes bytes; bytes.cls(1, "previous_name"); bytes.cls(2, "next_name"); bytes.data.resize(bytes.data.size() - 5);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    ASSERT_EQ(OmniPvdCommand::eREGISTER_CLASS, reader.getNextCommand());
    ASSERT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_STREQ("", reader.getClassName());
}

TEST(PvdNativeReader, TruncatedDiscardedStringByteIsInvalid)
{
    const std::string text(2048, 'x');
    for (unsigned field = 0; field < 3; ++field)
    {
        SCOPED_TRACE(field);
        EncodedStreamBytes bytes;
        OmniPvdWriterImpl writer;
        writer.setWriteStream(bytes);
        if (field == 0)
        {
            writer.registerClass(text.c_str(), 0);
            bytes.data.pop_back();
        }
        else
        {
            writer.recordMessage(1, field == 1 ? text.c_str() : "", field == 2 ? text.c_str() : "", 42, 7, 3);
            // Keep the prefix but omit the final encoded string byte and subsequent fields.
            bytes.data.resize(12 + 1 + 8 + 2 + (field == 2 ? 2 : 0) + 2047);
        }
        OmniPvdReaderImpl reader;
        reader.setReadStream(bytes);
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
        EXPECT_EQ(OmniPvdReaderStatus::eINVALID, reader.mStatus);
        EXPECT_EQ(bytes.data.size(), bytes.position);
    }
}

TEST(PvdNativeReader, RequestedDepthAboveStackCapacityIsClamped)
{
    Bytes bytes; bytes.set(4, 1, 10, 32);
    OmniPvdReaderImpl reader(1024, 255);
    reader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, reader.getNextCommand());
    bytes = Bytes(); bytes.set(4, 1, 10, 33);
    OmniPvdReaderImpl deepReader(1024, 255);
    deepReader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, deepReader.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eLIMIT, deepReader.mStatus);
}
TEST(PvdNativeReader, ZeroAttributePathAndRegisterEnumHaveDistinctStatus)
{
    Bytes bytes; bytes.set(4, 1, 10, 0);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eINVALID, reader.mStatus);
    Bytes enumeration; enumeration.command(OmniPvdCommand::eREGISTER_ENUM);
    OmniPvdReaderImpl enumReader;
    enumReader.setReadStream(enumeration);
    EXPECT_EQ(OmniPvdCommand::eINVALID, enumReader.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eUNSUPPORTED, enumReader.mStatus);
}

TEST(PvdNativeReader, UnknownCommandIsStickyInvalid)
{
    Bytes bytes; bytes.put(uint8_t(255)); bytes.frame(9);
    OmniPvdReaderImpl reader;
    reader.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eINVALID, reader.mStatus);
    EXPECT_EQ(13u, bytes.position);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
    EXPECT_EQ(13u, bytes.position);
}

TEST(PvdNativeReader, TooNewHeaderIsStickyUnsupported)
{
    for (unsigned field = 0; field < 3; ++field)
    {
        SCOPED_TRACE(field);
        uint32_t version[3] = {OMNI_PVD_VERSION_MAJOR, OMNI_PVD_VERSION_MINOR, OMNI_PVD_VERSION_PATCH};
        ++version[field];
        Bytes bytes; bytes.data.clear();
        bytes.put(version[0]); bytes.put(version[1]); bytes.put(version[2]);
        // A supported header and a frame follow; a non-sticky retry would decode them.
        bytes.put(uint32_t(OMNI_PVD_VERSION_MAJOR)); bytes.put(uint32_t(OMNI_PVD_VERSION_MINOR)); bytes.put(uint32_t(0));
        bytes.frame(9);
        OmniPvdReaderImpl reader;
        reader.setReadStream(bytes);
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
        EXPECT_EQ(OmniPvdReaderStatus::eUNSUPPORTED, reader.mStatus);
        EXPECT_EQ(12u, bytes.position);
        EXPECT_EQ(OmniPvdCommand::eINVALID, reader.getNextCommand());
        EXPECT_EQ(12u, bytes.position);
    }
}

TEST(PvdNativeReader, PayloadLimitIsInclusiveAndStickyLimit)
{
    Bytes bytes; bytes.set(4);
    OmniPvdReaderImpl exact(4, 32);
    exact.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, exact.getNextCommand());

    bytes.position = 0;
    OmniPvdReaderImpl over(3, 32);
    over.setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, over.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eLIMIT, over.mStatus);
    EXPECT_EQ(bytes.data.size() - 4, bytes.position); // Declared payload is never read.
    const size_t stopped = bytes.position;
    EXPECT_EQ(OmniPvdCommand::eINVALID, over.getNextCommand());
    EXPECT_EQ(stopped, bytes.position);

    // The default 1 GiB cap rejects a larger declaration before allocation or payload read.
    Bytes huge; huge.set(0); huge.data.resize(huge.data.size() - 4); huge.put(uint32_t((1u << 30) + 1));
    OmniPvdReaderImpl defaults;
    defaults.setReadStream(huge);
    EXPECT_EQ(OmniPvdCommand::eINVALID, defaults.getNextCommand());
    EXPECT_EQ(OmniPvdReaderStatus::eLIMIT, defaults.mStatus);
    EXPECT_EQ(huge.data.size(), huge.position);
}

TEST(PvdNativeReader, PayloadGrowthUsesBoundedAmortizedCapacity)
{
    Bytes bytes; bytes.set(4); bytes.set(5); bytes.set(7); bytes.set(10); bytes.set(10);
    OmniPvdReaderImpl reader(10, 32);
    reader.setReadStream(bytes);

    ASSERT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, reader.getNextCommand());
    EXPECT_EQ(5u, reader.mDataBuffAllocatedLen);
    const uint8_t* first = reader.getAttributeDataPointer();
    ASSERT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, reader.getNextCommand());
    EXPECT_EQ(5u, reader.mDataBuffAllocatedLen);
    EXPECT_EQ(first, reader.getAttributeDataPointer());
    ASSERT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, reader.getNextCommand());
    EXPECT_EQ(9u, reader.mDataBuffAllocatedLen);
    ASSERT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, reader.getNextCommand());
    EXPECT_EQ(10u, reader.mDataBuffAllocatedLen);
    const uint8_t* atLimit = reader.getAttributeDataPointer();
    ASSERT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, reader.getNextCommand());
    EXPECT_EQ(10u, reader.mDataBuffAllocatedLen);
    EXPECT_EQ(atLimit, reader.getAttributeDataPointer());

}
}
