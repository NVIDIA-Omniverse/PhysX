// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdReader.h"
#include "../../pvdruntime/src/OmniPvdReaderImpl.h"
#include "test_bytes.h"
#include "../../pvdruntime/src/OmniPvdWriterImpl.h"
#include "PvdDom.h"
#include "PvdDomLog.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <utility>
#include <vector>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

#include "PvdDomDocument.h"
#include "PvdDomParser.h"
#include "PvdDomQuery.h"
#include "../src/PvdDomHierarchy.h"
using namespace PvdDomInspect;

namespace
{
TEST(PvdReaderFactory, FixedPathCapacityAccepts32AndRejects33Handles)
{
    Bytes bytes; bytes.set(4, 1, 10, 32);
    OmniPvdReader* reader = createOmniPvdReaderWithLimits(1024);
    ASSERT_NE(nullptr, reader);
    reader->setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eSET_ATTRIBUTE, reader->getNextCommand());
    destroyOmniPvdReader(*reader);

    bytes = Bytes(); bytes.set(4, 1, 10, 33);
    reader = createOmniPvdReaderWithLimits(1024);
    ASSERT_NE(nullptr, reader);
    reader->setReadStream(bytes);
    EXPECT_EQ(OmniPvdCommand::eINVALID, reader->getNextCommand());
    const OmniPvdReaderStatus status = getOmniPvdReaderStatus(*reader);
    EXPECT_EQ(OmniPvdReaderStatus::eLIMIT, status.code);
    EXPECT_EQ(33u, status.attributePathDepth);
    destroyOmniPvdReader(*reader);
}

class PvdDocument : public testing::Test
{
protected:
    Bytes bytes;
    Document document;
    Error error;
    LoadLimits limits;
    std::string path;
    void SetUp() override
    {
        path = std::string("document_") + testing::UnitTest::GetInstance()->current_test_info()->name() + ".ovd";
    }
    void TearDown() override { std::remove(path.c_str()); }
    bool writeFile()
    {
        FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) return false;
        const size_t written = std::fwrite(bytes.data.data(), 1, bytes.data.size(), f);
        std::fclose(f);
        return written == bytes.data.size();
    }
    bool load(LoadMode mode = LoadMode::Strict)
    {
        if (!writeFile()) return false;
        return document.load(path.c_str(), limits, error, mode);
    }
    void rejected(Error::Code code)
    {
        EXPECT_FALSE(load());
        EXPECT_EQ(code, error.code) << error.message;
        EXPECT_EQ(nullptr, document.dom());
        EXPECT_TRUE(document.frames().empty());
        EXPECT_FALSE(error.message.empty());
    }
    void schema() { bytes.cls(); bytes.attr(); bytes.create(); }
};

std::vector<std::string> documentReaderLines;
void OMNI_PVD_CALL captureDocumentReader(char* text) { documentReaderLines.emplace_back(text); }
std::vector<std::pair<PvdDomLogLevel, std::string>> documentDomLines;
void captureDocumentDom(PvdDomLogLevel level, const char* text)
{
    documentDomLines.emplace_back(level, text ? text : "");
}

void expectErrorValues(const Error& a, const Error& b)
{
    EXPECT_EQ(a.code, b.code); EXPECT_EQ(a.message, b.message);
    EXPECT_EQ(a.byteOffset, b.byteOffset); EXPECT_EQ(a.commandIndex, b.commandIndex);
    EXPECT_EQ(a.limitKind, b.limitKind);
}

void expectDocumentValues(const Document& a, const Document& b)
{
    const uint64_t LoadStats::* fields[] = {&LoadStats::streamBytes, &LoadStats::commands,
        &LoadStats::ignoredCommands, &LoadStats::unresolvedOptionalEdges, &LoadStats::creates,
        &LoadStats::classes, &LoadStats::attributes, &LoadStats::samples, &LoadStats::handlerSteps,
        &LoadStats::membershipVisits, &LoadStats::accountedRetainedBytes};
    for (auto field : fields) EXPECT_EQ(a.stats().*field, b.stats().*field);
    EXPECT_EQ(a.loadInfo().recoveredTail, b.loadInfo().recoveredTail);
    EXPECT_EQ(a.loadInfo().sourceBytes, b.loadInfo().sourceBytes);
    expectErrorValues(a.loadInfo().terminalError, b.loadInfo().terminalError);
    ASSERT_EQ(a.frames().size(), b.frames().size());
    for (size_t i = 0; i < a.frames().size(); ++i)
    {
        EXPECT_EQ(a.frames()[i].recordingSegment, b.frames()[i].recordingSegment);
        EXPECT_EQ(a.frames()[i].context, b.frames()[i].context);
        EXPECT_EQ(a.frames()[i].recordedFrame, b.frames()[i].recordedFrame);
        EXPECT_EQ(a.frames()[i].sourceTimestamp, b.frames()[i].sourceTimestamp);
        EXPECT_EQ(a.frames()[i].completed, b.frames()[i].completed);
    }
    ASSERT_EQ(a.messages().size(), b.messages().size());
    for (size_t i = 0; i < a.messages().size(); ++i)
    {
        const auto& x = a.messages()[i]; const auto& y = b.messages()[i];
        EXPECT_STREQ(x.message, y.message); EXPECT_STREQ(x.file, y.file); EXPECT_STREQ(x.typeName, y.typeName);
        EXPECT_EQ(x.line, y.line); EXPECT_EQ(x.type, y.type); EXPECT_EQ(x.handle, y.handle); EXPECT_EQ(x.frameId, y.frameId);
    }
    ASSERT_EQ(a.dom() == nullptr, b.dom() == nullptr);
    if (!a.dom()) return;
    EXPECT_EQ(a.dom()->mMinFrame, b.dom()->mMinFrame);
    EXPECT_EQ(a.dom()->mMaxFrame, b.dom()->mMaxFrame);
    EXPECT_EQ(a.dom()->mExternalToInternalHandleMap, b.dom()->mExternalToInternalHandleMap);
    ASSERT_EQ(a.dom()->mClassHandleToClassMap.size(), b.dom()->mClassHandleToClassMap.size());
    for (const auto& entry : a.dom()->mClassHandleToClassMap)
    {
        const auto found = b.dom()->mClassHandleToClassMap.find(entry.first);
        ASSERT_NE(b.dom()->mClassHandleToClassMap.end(), found);
        EXPECT_EQ(entry.second->mClassName, found->second->mClassName);
        ASSERT_EQ(entry.second->mAttributeDefinitions.size(), found->second->mAttributeDefinitions.size());
        for (size_t i = 0; i < entry.second->mAttributeDefinitions.size(); ++i)
        {
            const auto* x = entry.second->mAttributeDefinitions[i];
            const auto* y = found->second->mAttributeDefinitions[i];
            EXPECT_EQ(x->mAttributeName, y->mAttributeName); EXPECT_EQ(x->mDataType, y->mDataType);
            EXPECT_EQ(x->mNbrFields, y->mNbrFields);
        }
    }
    ASSERT_EQ(a.dom()->mObjectHandleToObjectMap.size(), b.dom()->mObjectHandleToObjectMap.size());
    for (const auto& entry : a.dom()->mObjectHandleToObjectMap)
    {
        const auto found = b.dom()->mObjectHandleToObjectMap.find(entry.first);
        ASSERT_NE(b.dom()->mObjectHandleToObjectMap.end(), found);
        EXPECT_EQ(entry.second->mObjectName, found->second->mObjectName);
        EXPECT_EQ(entry.second->mOmniPvdClass->mOmniClassHandle, found->second->mOmniPvdClass->mOmniClassHandle);
        const auto* x = getLatestSampleAtFrame(entry.second, "value", UINT64_MAX);
        const auto* y = getLatestSampleAtFrame(found->second, "value", UINT64_MAX);
        ASSERT_EQ(x == nullptr, y == nullptr);
        if (x)
        {
            EXPECT_EQ(x->mTimeStamp, y->mTimeStamp); ASSERT_EQ(x->mDataLen, y->mDataLen);
            if (x->mDataLen) EXPECT_EQ(0, std::memcmp(x->mData, y->mData, x->mDataLen));
        }
    }
}

class PvdDocumentDiagnostics : public PvdDocument
{
protected:
    void SetUp() override
    {
        PvdDocument::SetUp(); documentReaderLines.clear(); documentDomLines.clear();
        pvdDomSetLogFunction(captureDocumentDom);
    }
    void TearDown() override { pvdDomSetLogFunction(nullptr); PvdDocument::TearDown(); }
    OmniPvdReaderDiagnostics options(bool verbose)
    {
        OmniPvdReaderDiagnostics result;
        result.logFunction = captureDocumentReader; result.verbose = verbose;
        return result;
    }
};

TEST_F(PvdDocumentDiagnostics, OptionsArePerLoadAndOldSymbolsRemainCallable)
{
    schema(); bytes.frame(17); bytes.set(); bytes.frame(18, true);
    bytes.command(OmniPvdCommand::eRECORD_MESSAGE); bytes.put(uint64_t(1));
    bytes.name("recorded message"); bytes.name("test.cpp");
    bytes.put(uint32_t(42)); bytes.put(uint32_t(3)); bytes.put(uint32_t(1));
    ASSERT_TRUE(writeFile());
    using OldStrict = bool (Document::*)(const char*, const LoadLimits&, Error&);
    using OldMode = bool (Document::*)(const char*, const LoadLimits&, Error&, LoadMode);
    const OldStrict oldStrict = &Document::load;
    const OldMode oldMode = &Document::load;
    Document baseline; Error baselineError;
    ASSERT_TRUE((baseline.*oldStrict)(path.c_str(), limits, baselineError));
    ASSERT_TRUE(document.load(path.c_str(), limits, error, options(false)));
    EXPECT_TRUE(documentReaderLines.empty()); expectDocumentValues(baseline, document);
    ASSERT_TRUE(document.load(path.c_str(), limits, error, options(true)));
    EXPECT_FALSE(documentReaderLines.empty()); expectDocumentValues(baseline, document);
    ASSERT_EQ(1u, document.messages().size());
    documentReaderLines.clear();
    ASSERT_TRUE((document.*oldMode)(path.c_str(), limits, error, LoadMode::Strict));
    EXPECT_TRUE(documentReaderLines.empty()); expectDocumentValues(baseline, document);

    bytes.put(uint8_t(255)); ASSERT_TRUE(writeFile());
    EXPECT_FALSE(document.load(path.c_str(), limits, error, options(true)));
    documentReaderLines.clear();
    bytes.data.pop_back(); ASSERT_TRUE(writeFile());
    ASSERT_TRUE((document.*oldStrict)(path.c_str(), limits, error));
    EXPECT_TRUE(documentReaderLines.empty()); expectDocumentValues(baseline, document);

    const char* fixture = PVDDOM_TEST_DATA_DIR "/test_scene.ovd";
    ASSERT_TRUE(baseline.load(fixture, limits, baselineError));
    ASSERT_TRUE(document.load(fixture, limits, error, options(true)));
    expectDocumentValues(baseline, document);
}

TEST_F(PvdDocumentDiagnostics, HeaderFailuresReachAllLineSink)
{
    const uint32_t versions[][3] = {{1, 0, 0}, {0, 5, 0}, {0, 4, 1}};
    const char* reasons[] = {"[parser] major version too new\n", "[parser] minor version too new\n", "[parser] patch too new\n"};
    for (unsigned field = 0; field < 3; ++field)
    for (LoadMode mode : {LoadMode::Strict, LoadMode::RecoverTruncatedTail})
    {
        bytes.data.clear();
        for (uint32_t value : versions[field]) bytes.put(value);
        ASSERT_TRUE(writeFile());
        Document baseline; Error baselineError;
        documentDomLines.clear();
        EXPECT_FALSE(baseline.load(path.c_str(), limits, baselineError, mode));
        const auto baselineDomLines = documentDomLines;
        const std::string version = "OmniPvdRuntimeReaderImpl::startReading majorVersion(" +
            std::to_string(versions[field][0]) + "), minorVersion(" + std::to_string(versions[field][1]) +
            "), patch(" + std::to_string(versions[field][2]) + ")";
        for (bool verbose : {false, true})
        {
            documentReaderLines.clear(); documentDomLines.clear();
            EXPECT_FALSE(document.load(path.c_str(), limits, error, mode, options(verbose)));
            EXPECT_EQ(Error::Code::Unsupported, error.code);
            expectErrorValues(baselineError, error);
            expectDocumentValues(baseline, document);
            EXPECT_EQ((std::vector<std::string>{version, reasons[field]}), documentReaderLines);
            EXPECT_EQ(baselineDomLines, documentDomLines);
        }
    }
}

TEST_F(PvdDocumentDiagnostics, ResultsAndErrorsAreIndependentOfTracing)
{
    const char* readerErrors[] = {
        nullptr, nullptr, "[parser] ERROR: failed to read complete field\n",
        "[parser] ERROR: unknown command 255\n", "[parser] ERROR: unsupported command eREGISTER_ENUM\n",
        "[parser] ERROR: invalid attribute data type\n", "[parser] ERROR: zero attribute path depth\n",
        "[parser] ERROR: attribute path depth 33 exceeds limit 32\n",
        "[parser] ERROR: failed to read complete field\n", "[parser] ERROR: failed to read complete field\n",
        nullptr, nullptr, "[parser] ERROR: attribute payload 5 bytes exceeds limit 4\n",
        "[parser] ERROR: attribute payload 5 bytes exceeds limit 4\n", nullptr, nullptr, nullptr
    };
    for (unsigned failure = 0; failure < 17; ++failure)
    for (LoadMode mode : {LoadMode::Strict, LoadMode::RecoverTruncatedTail})
    {
        SCOPED_TRACE(failure);
        SCOPED_TRACE(static_cast<int>(mode));
        bytes = Bytes(); schema(); bytes.frame(17); bytes.set(); bytes.frame(18, true);
        limits = LoadLimits();
        switch (failure)
        {
        case 0: break;
        case 1: bytes.data.clear(); break;
        case 2: bytes.data.resize(5); break;
        case 3: bytes.put(uint8_t(255)); break;
        case 4: bytes.command(OmniPvdCommand::eREGISTER_ENUM); break;
        case 5: bytes.attr(2, "bad", 1, static_cast<OmniPvdDataType::Enum>(65535)); break;
        case 6: bytes.set(4, 1, 10, 0); break;
        case 7: bytes.set(4, 1, 10, 33); break;
        case 8: bytes.command(OmniPvdCommand::eCREATE_OBJECT); break;
        case 9: bytes.set(); bytes.data.pop_back(); break;
        case 10: bytes.set(3); break;
        case 11: limits.maxCommands = 1; break;
        case 12: limits.maxAttributeBytes = 4; bytes.set(5); break;
        case 13: limits.maxAttributeBytes = 4; bytes.set(5); bytes.data.pop_back(); break;
        case 14: limits.maxStreamBytes = 1; break;
        case 15: limits.maxRetainedBytes = 1; break;
        case 16: bytes.attr(2, "missing owner", 99); break;
        }
        ASSERT_TRUE(writeFile());
        Document baseline; Error baselineError;
        documentDomLines.clear();
        const bool result = baseline.load(path.c_str(), limits, baselineError, mode);
        const auto baselineDomLines = documentDomLines;
        std::vector<std::string> expected;
        if (readerErrors[failure]) expected.emplace_back(readerErrors[failure]);
        for (bool verbose : {false, true})
        {
            documentReaderLines.clear(); documentDomLines.clear();
            EXPECT_EQ(result, document.load(path.c_str(), limits, error, mode, options(verbose)));
            expectErrorValues(baselineError, error); expectDocumentValues(baseline, document);
            EXPECT_EQ(expected, verbose ? withoutReaderSuccessRecords(documentReaderLines) : documentReaderLines);
            EXPECT_EQ(baselineDomLines, documentDomLines);
            if (failure == 1 || failure == 2 || failure == 14 || failure == 15)
                EXPECT_EQ(expected, documentReaderLines);
        }
    }
    for (LoadMode mode : {LoadMode::Strict, LoadMode::RecoverTruncatedTail})
    {
        const char* missing = "__missing_omnipvd_directory__/diagnostics.ovd";
        Document baseline; Error baselineError;
        documentDomLines.clear();
        EXPECT_FALSE(baseline.load(missing, LoadLimits(), baselineError, mode));
        const auto baselineDomLines = documentDomLines;
        for (bool verbose : {false, true})
        {
            documentReaderLines.clear(); documentDomLines.clear();
            EXPECT_FALSE(document.load(missing, LoadLimits(), error, mode, options(verbose)));
            EXPECT_EQ(Error::Code::Io, error.code); expectErrorValues(baselineError, error);
            EXPECT_TRUE(documentReaderLines.empty()); EXPECT_EQ(baselineDomLines, documentDomLines);
        }
    }
}

Bytes actorRecording(uint32_t count, uint64_t frames = 10)
{
    EncodedStreamBytes output;
    OmniPvdWriterImpl implementation;
    OmniPvdWriter& writer = implementation;
    writer.setWriteStream(output);
    const auto scene = writer.registerClass("PxScene");
    const auto actor = writer.registerClass("PxActor");
    const auto dynamic = writer.registerClass("PxRigidDynamic", actor);
    const auto shape = writer.registerClass("PxShape");
    const auto geometry = writer.registerClass("PxGeometry");
    const auto box = writer.registerClass("PxBoxGeometry", geometry);
    const auto actors = writer.registerUniqueListAttribute(scene, "actors", OmniPvdDataType::eOBJECT_HANDLE);
    const auto shapes = writer.registerUniqueListAttribute(actor, "shapes", OmniPvdDataType::eOBJECT_HANDLE);
    const auto geom = writer.registerAttribute(shape, "geom", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto exclusive = writer.registerAttribute(shape, "isExclusive", OmniPvdDataType::eUINT8, 1);
    const auto pose = writer.registerAttribute(dynamic, "globalPose", OmniPvdDataType::eFLOAT32, 7);
    const auto velocity = writer.registerAttribute(dynamic, "linearVelocity", OmniPvdDataType::eFLOAT32, 3);
    writer.createObject(1, scene, 1, "");
    const uint8_t isExclusive = 1;
    for (uint32_t i = 0; i < count; ++i)
    {
        const uint64_t actorHandle = 10 + 3 * uint64_t(i), shapeHandle = actorHandle + 1, geomHandle = actorHandle + 2;
        writer.createObject(1, dynamic, actorHandle, "");
        writer.createObject(1, shape, shapeHandle, "");
        writer.createObject(1, box, geomHandle, "");
        writer.setAttribute(1, shapeHandle, exclusive, &isExclusive, 1);
        writer.setAttribute(1, shapeHandle, geom, reinterpret_cast<const uint8_t*>(&geomHandle), 8);
        writer.addToUniqueListAttribute(1, actorHandle, shapes, reinterpret_cast<const uint8_t*>(&shapeHandle), 8);
        writer.addToUniqueListAttribute(1, 1, actors, reinterpret_cast<const uint8_t*>(&actorHandle), 8);
    }
    for (uint64_t frame = 1; frame <= frames; ++frame)
    {
        writer.startFrame(1, frame);
        for (uint32_t i = 0; i < count; ++i)
        {
            const float transform[7] = {0, 0, 0, 1, float(i), float(frame), 0};
            const float speed[3] = {0, float(frame), 0};
            writer.setAttribute(1, 10 + 3 * uint64_t(i), pose, reinterpret_cast<const uint8_t*>(transform), sizeof(transform));
            writer.setAttribute(1, 10 + 3 * uint64_t(i), velocity, reinterpret_cast<const uint8_t*>(speed), sizeof(speed));
        }
        writer.stopFrame(1, frame);
    }
    return output;
}

TEST_F(PvdDocument, FourThousandActorsFitDefaultRetainedBudget)
{
    bytes = actorRecording(4000);
    ASSERT_TRUE(load()) << error.message << " at command " << error.commandIndex;
    EXPECT_EQ(12001u, document.stats().creates);
    EXPECT_EQ(96000u, document.stats().samples);
    EXPECT_EQ(10u, document.frames().size());
    // Per actor: two shape setters, twenty frame writes, four attribute indexes, three objects.
    const uint64_t semanticBytes = 4000ull * (22 * (2 * sizeof(SampleWrite) + 2 * sizeof(size_t)) +
        4 * (128 + 192) + 3 * (256 + 2 * sizeof(SceneMembership))) +
        256 + 2 * sizeof(SceneMembership) + 128 + 10 * 2 * sizeof(uint64_t);
    EXPECT_LT(document.stats().accountedRetainedBytes, 48 * document.stats().streamBytes + semanticBytes);
    const auto* dom = document.dom();
    auto* actor = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(10));
    auto* shape = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(11));
    auto* geometry = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(12));
    EXPECT_EQ(actor, shape->mAncestor);
    EXPECT_EQ(shape, geometry->mAncestor);
}

TEST_F(PvdDocument, MembershipWorkScalesWithIndependentSubtrees)
{
    uint64_t previous = 0;
    for (uint32_t count : {1000u, 2000u})
    {
        SCOPED_TRACE(count);
        bytes = actorRecording(count);
        ASSERT_TRUE(load()) << error.message;
        const uint64_t visits = document.stats().membershipVisits;
        EXPECT_LE(visits, 32 * uint64_t(count) + 8);
        if (previous) EXPECT_LE(visits, 3 * previous + 8);
        previous = visits;
        for (const auto& entry : document.history())
        {
            ASSERT_EQ(1u, entry.second.memberships.size());
            EXPECT_EQ(1u, entry.second.memberships[0].context);
            EXPECT_EQ(0u, entry.second.memberships[0].stopOrdinal);
        }
        bytes = actorRecording(count, 0);
        ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(visits, document.stats().membershipVisits) << "Pose/velocity writes must not refresh membership";
        // These schema/handle values come from actorRecording's writer order.
        bytes.frame(11);
        bytes.removeReference(1, 1, 10);
        bytes.reference(1, 1, 10, true);
        ASSERT_TRUE(load()) << error.message;
        EXPECT_LE(document.stats().membershipVisits - visits, 32u);
        const uint64_t beforeMove = document.stats().membershipVisits;
        bytes.create(2, 1); bytes.frame(20, false, 2); bytes.reference(2, 1, 10, true);
        ASSERT_TRUE(load()) << error.message;
        EXPECT_LE(document.stats().membershipVisits - beforeMove, 16u);
        for (uint64_t handle : {10ull, 11ull, 12ull})
        {
            const auto id = document.dom()->mExternalToInternalHandleMap.at(handle);
            const auto& membership = document.history().at(id).memberships.back();
            EXPECT_EQ(2u, membership.context); EXPECT_EQ(20u, membership.startFrame);
        }
    }
}

TEST_F(PvdDocument, RetainedAccountingScalesWithIndependentActors)
{
    uint64_t previous = 0;
    for (uint32_t count : {1000u, 2000u, 4000u})
    {
        SCOPED_TRACE(count);
        bytes = actorRecording(count);
        ASSERT_TRUE(load()) << error.message;
        const uint64_t current = document.stats().accountedRetainedBytes;
        if (previous) EXPECT_LE(current, 2 * previous);
        previous = current;
    }
}

const ObjectHistory& historyFor(const Document& document, uint64_t handle)
{
    return document.history().at(document.dom()->mExternalToInternalHandleMap.at(handle));
}

TEST_F(PvdDocument, MembershipFollowsMovedGeometryAndSharedShapeDescendants)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
    bytes.cls(2, "PxRigidDynamic"); bytes.list(2, "shapes", 2);
    bytes.cls(3, "PxShape"); bytes.attr(3, "geom", 3, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.attr(4, "isExclusive", 3, OmniPvdDataType::eUINT8);
    bytes.cls(4, "PxBoxGeometry");
    bytes.create(100); bytes.create(200); bytes.create(10, 2); bytes.create(20, 2);
    bytes.create(11, 3); bytes.create(21, 3); bytes.create(12, 4);
    bytes.frame(1, false, 100); bytes.frame(10, false, 200);
    bytes.reference(100, 1, 10, true); bytes.reference(200, 1, 20, true);
    bytes.reference(10, 2, 11, true); bytes.reference(20, 2, 21, true);
    bytes.reference(11, 3, 12);
    bytes.frame(2, false, 100); bytes.frame(20, false, 200);
    bytes.reference(21, 3, 12);
    bytes.frame(30, false, 200); bytes.set(1, 4, 21);
    bytes.reference(10, 2, 21, true); bytes.removeReference(10, 2, 21);
    ASSERT_TRUE(load()) << error.message;
    const auto& geometry = historyFor(document, 12).memberships;
    ASSERT_EQ(2u, geometry.size());
    EXPECT_EQ(100u, geometry[0].context); EXPECT_EQ(2u, geometry[0].endFrame);
    EXPECT_EQ(200u, geometry[1].context); EXPECT_EQ(20u, geometry[1].startFrame);
    EXPECT_EQ(30u, geometry[1].endFrame); EXPECT_NE(0u, geometry[1].stopOrdinal);
    const auto& shape = historyFor(document, 21).memberships;
    ASSERT_EQ(1u, shape.size()); EXPECT_EQ(30u, shape[0].endFrame);
}

TEST_F(PvdDocument, MembershipFollowsEverySubtreeListHandler)
{
    for (const char* cls : {"PxRigidDynamic", "PxDeformableVolume", "PxDeformableSurface", "PxPBDParticleSystem"})
    {
        SCOPED_TRACE(cls);
        bytes = Bytes();
        const bool particles = std::string(cls) == "PxPBDParticleSystem";
        bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, cls); bytes.list(2, particles ? "particleBuffers" : "shapes", 2);
        bytes.cls(3, particles ? "PxParticleBuffer" : "PxShape");
        bytes.create(100); bytes.create(200); bytes.create(10, 2); bytes.create(11, 3);
        bytes.frame(1, false, 100); bytes.frame(10, false, 200);
        bytes.reference(100, 1, 10, true); bytes.reference(10, 2, 11, true);
        bytes.frame(2, false, 100); bytes.removeReference(10, 2, 11);
        bytes.frame(3, false, 100); bytes.reference(10, 2, 11, true);
        bytes.frame(4, false, 100); bytes.reference(200, 1, 10, true);
        bytes.frame(20, false, 200); bytes.removeReference(200, 1, 10);
        bytes.frame(30, false, 200); bytes.reference(200, 1, 10, true);
        ASSERT_TRUE(load()) << error.message;
        const auto& members = historyFor(document, 11).memberships;
        ASSERT_EQ(4u, members.size());
        EXPECT_EQ(100u, members[0].context); EXPECT_EQ(2u, members[0].endFrame);
        EXPECT_EQ(3u, members[1].startFrame); EXPECT_EQ(4u, members[1].endFrame);
        EXPECT_EQ(200u, members[2].context); EXPECT_EQ(10u, members[2].startFrame);
        EXPECT_EQ(20u, members[2].endFrame); EXPECT_EQ(30u, members[3].startFrame);
        EXPECT_EQ(0u, members[3].stopOrdinal);
    }
}

TEST_F(PvdDocument, ArticulationMembershipRefreshesBothMovedRoots)
{
    for (bool inbound : {false, true})
    {
        SCOPED_TRACE(inbound);
        bytes = Bytes();
        bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, "PxArticulationLink");
        bytes.attr(2, "inboundJoint", 2, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.cls(3, "PxArticulationJointReducedCoordinate");
        bytes.attr(3, "parentLink", 3, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.attr(4, "childLink", 3, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.create(100); bytes.create(200); bytes.create(10, 2); bytes.create(20, 2); bytes.create(30, 3);
        bytes.frame(1, false, 100); bytes.frame(10, false, 200);
        bytes.reference(100, 1, 10, true); bytes.reference(200, 1, 20, true);
        bytes.reference(30, 3, 10);
        if (inbound) bytes.reference(20, 2, 30); else bytes.reference(30, 4, 20);
        bytes.frame(2, false, 100); bytes.frame(20, false, 200);
        bytes.reference(30, 3, 20); // Cached parent alone does not change membership.
        ASSERT_TRUE(load()) << error.message;
        const auto& link = historyFor(document, 20).memberships;
        ASSERT_EQ(2u, link.size()); EXPECT_EQ(200u, link[0].context); EXPECT_EQ(10u, link[0].endFrame);
        EXPECT_EQ(100u, link[1].context); EXPECT_EQ(1u, link[1].startFrame);
        const auto& joint = historyFor(document, 30).memberships;
        ASSERT_EQ(1u, joint.size()); EXPECT_EQ(100u, joint[0].context); EXPECT_EQ(1u, joint[0].startFrame);
        EXPECT_EQ(0u, joint[0].stopOrdinal);
    }
}

TEST_F(PvdDocument, FloatingArticulationAndSupersededSubtreeMembership)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "articulations", 1);
    bytes.cls(2, "PxArticulationReducedCoordinate");
    bytes.cls(3, "PxArticulationLink"); bytes.attr(2, "articulation", 3, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(100); bytes.create(11, 3); bytes.reference(11, 2, 10);
    bytes.create(10, 2); bytes.frame(1, false, 100); bytes.reference(100, 1, 10, true);
    bytes.frame(2, false, 100); bytes.create(10, 2);
    bytes.frame(3, false, 100); bytes.reference(100, 1, 10, true);
    ASSERT_TRUE(load()) << error.message;
    const auto& link = historyFor(document, 11).memberships;
    ASSERT_EQ(1u, link.size()); EXPECT_EQ(1u, link[0].startFrame); EXPECT_EQ(2u, link[0].endFrame);
    EXPECT_NE(0u, link[0].stopOrdinal);
    const auto& replacement = historyFor(document, 10).memberships;
    ASSERT_EQ(1u, replacement.size()); EXPECT_EQ(3u, replacement[0].startFrame);
}

TEST_F(PvdDocument, ConstraintRemovalAndSceneDestroyTraverseSyntheticBranches)
{
    for (bool destroyScene : {false, true})
    {
        SCOPED_TRACE(destroyScene);
        bytes = Bytes();
        bytes.cls(1, "PxScene"); bytes.list(1, "constraints", 1);
        bytes.cls(2, "PxFixedJoint"); bytes.attr(2, "constraint", 2, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.create(100); bytes.create(10, 2); bytes.frame(1, false, 100);
        bytes.reference(100, 1, 500, true); bytes.reference(10, 2, 500);
        bytes.frame(2, false, 100);
        if (destroyScene)
        { bytes.command(OmniPvdCommand::eDESTROY_OBJECT); bytes.put(uint64_t(1)); bytes.put(uint64_t(100)); }
        else bytes.removeReference(100, 1, 500);
        ASSERT_TRUE(load()) << error.message;
        const auto& joint = historyFor(document, 10).memberships;
        ASSERT_EQ(1u, joint.size()); EXPECT_EQ(100u, joint[0].context);
        EXPECT_EQ(1u, joint[0].startFrame); EXPECT_EQ(2u, joint[0].endFrame); EXPECT_NE(0u, joint[0].stopOrdinal);
    }
}

TEST_F(PvdDocument, MembershipTraversalPrunesSyntheticGenericReferences)
{
    bytes.cls(); bytes.create(10); bytes.create(20);
    ASSERT_TRUE(load()) << error.message;
    ASSERT_TRUE(load());
    const uint64_t creationVisits = document.stats().membershipVisits;
    for (uint32_t attribute = 1; attribute <= 32; ++attribute)
    {
        const std::string name = "reference" + std::to_string(attribute);
        bytes.attr(attribute, name.c_str(), 1, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.reference(10, attribute, 20);
    }
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(creationVisits, document.stats().membershipVisits);
    bytes.frame(1);
    bytes.command(OmniPvdCommand::eDESTROY_OBJECT); bytes.put(uint64_t(1)); bytes.put(uint64_t(10));
    ASSERT_TRUE(load()) << error.message;
    EXPECT_GT(document.dom()->mObjectCreations.size(), document.stats().creates + 32);
    EXPECT_GT(document.stats().membershipVisits, creationVisits);
    EXPECT_LT(document.stats().membershipVisits, creationVisits + 16);
    EXPECT_EQ(0u, historyFor(document, 20).object->mLifeSpans.back().mFrameStop);
    EXPECT_TRUE(historyFor(document, 20).memberships.empty());
}

TEST_F(PvdDocument, MovedOpenMembershipClosesAtSegmentClear)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1); bytes.cls(2, "PxRigidDynamic");
    bytes.cls(3, "PxOmniPvdMetaData"); bytes.create(1, 3); bytes.create(100); bytes.frame(1, false, 100);
    for (uint64_t handle : {10ull, 11ull, 12ull})
    {
        bytes.create(handle, 2); bytes.reference(100, 1, handle, true);
    }
    bytes.frame(2, false, 100);
    // Removing 10 moves 12 into its open slot; removing 12 must preserve 11.
    bytes.removeReference(100, 1, 10); bytes.removeReference(100, 1, 12);
    bytes.create(2, 3);
    ASSERT_TRUE(load()) << error.message;
    const auto& memberships = historyFor(document, 11).memberships;
    ASSERT_FALSE(memberships.empty());
    EXPECT_NE(0u, memberships.back().stopOrdinal);
    EXPECT_EQ(3u, memberships.back().endFrame);
}

TEST_F(PvdDocument, SegmentAdvanceClosesUntouchedMemberships)
{
    bytes.cls(1, "PxOmniPvdMetaData"); bytes.create(1);
    bytes.cls(2, "PxScene"); bytes.list(1, "actors", 2);
    bytes.cls(3, "PxRigidDynamic"); bytes.create(100, 2); bytes.create(10, 3);
    bytes.frame(7, false, 100); bytes.reference(100, 1, 10, true);
    bytes.create(2);
    ASSERT_TRUE(load()) << error.message;
    for (uint64_t handle : {100ull, 10ull})
    {
        const auto& members = historyFor(document, handle).memberships;
        ASSERT_EQ(1u, members.size()); EXPECT_EQ(0u, members[0].recordingSegment);
        EXPECT_EQ(8u, members[0].endFrame); EXPECT_EQ(document.stats().commands, members[0].stopOrdinal);
    }
}

TEST_F(PvdDocument, MovingGenericReferenceHeavyLinkSkipsSyntheticBranches)
{
    uint64_t plainVisits = 0;
    for (uint32_t referencesCount : {0u, 64u})
    {
        bytes = Bytes();
        bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, "PxActor"); bytes.cls(5, "PxArticulationLink", 2); bytes.cls(3, "CustomLink", 5);
        bytes.cls(4, "PxArticulationJointReducedCoordinate");
        bytes.attr(3, "parentLink", 4, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.attr(4, "childLink", 4, OmniPvdDataType::eOBJECT_HANDLE);
        for (uint32_t i = 0; i < referencesCount; ++i)
            bytes.attr(100 + i, ("reference" + std::to_string(i)).c_str(), 3, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.create(100); bytes.create(200); bytes.create(10, 3); bytes.create(11, 3);
        bytes.create(20, 3); bytes.create(30, 4);
        bytes.frame(1, false, 100); bytes.frame(1, false, 200);
        bytes.reference(100, 1, 10, true); bytes.reference(200, 1, 11, true);
        bytes.reference(200, 1, 20, true);
        for (uint32_t i = 0; i < referencesCount; ++i) bytes.reference(20, 100 + i, 10);
        bytes.reference(30, 3, 10); bytes.reference(30, 4, 20);
        for (uint64_t frame = 2; frame <= 4; ++frame)
        {
            bytes.frame(frame, false, 100); bytes.frame(frame, false, 200);
            bytes.reference(30, 3, frame % 2 ? 10 : 11); bytes.reference(30, 4, 20);
        }
        ASSERT_TRUE(load()) << error.message << " at command " << error.commandIndex;
        const uint64_t visits = document.stats().membershipVisits;
        if (referencesCount) EXPECT_LT(visits - plainVisits, 32u);
        else plainVisits = visits;
    }
}

TEST_F(PvdDocument, RealChildAfterSyntheticReferencesKeepsSceneMembership)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
    bytes.cls(2, "PxActor"); bytes.cls(5, "PxArticulationLink", 2); bytes.cls(3, "CustomLink", 5);
    bytes.cls(4, "PxArticulationJointReducedCoordinate");
    bytes.attr(3, "parentLink", 4, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.attr(4, "childLink", 4, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.attr(100, "reference", 3, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(100); bytes.create(10, 3); bytes.create(20, 3); bytes.create(30, 3); bytes.create(40, 4);
    bytes.frame(1, false, 100); bytes.reference(100, 1, 10, true);
    bytes.reference(10, 100, 30);
    bytes.reference(40, 3, 10); bytes.reference(40, 4, 20);
    bytes.frame(2, false, 100); bytes.removeReference(100, 1, 10);
    ASSERT_TRUE(load()) << error.message << " at command " << error.commandIndex;
    const auto& child = historyFor(document, 20).memberships;
    ASSERT_EQ(1u, child.size());
    EXPECT_EQ(1u, child[0].startFrame);
    EXPECT_EQ(2u, child[0].endFrame);
}

TEST_F(PvdDocument, FirstHistoricalWriteChargesIndexesBeyondRepeatedNativeSet)
{
    bytes.cls(); bytes.attr(); bytes.create();
    ASSERT_TRUE(load()) << error.message;
    const uint64_t before = document.stats().accountedRetainedBytes;
    bytes.set(); ASSERT_TRUE(load()) << error.message;
    const uint64_t firstDelta = document.stats().accountedRetainedBytes - before;
    bytes.set(); ASSERT_TRUE(load()) << error.message;
    const uint64_t secondDelta = document.stats().accountedRetainedBytes - before - firstDelta;
    EXPECT_GE(secondDelta, 256u + 2 * sizeof(SampleWrite));
    ASSERT_GE(firstDelta, secondDelta);
    EXPECT_GE(firstDelta - secondDelta, 128u + 160u);
    const auto id = document.dom()->mExternalToInternalHandleMap.at(10);
    const auto& writes = document.history().at(id).writes;
    ASSERT_EQ(1u, writes.size()); EXPECT_EQ(2u, writes.begin()->second.size());
}

TEST_F(PvdDocument, MembershipVisitStatsAreIndependentPerDocument)
{
    bytes.cls(); bytes.create(10);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t firstVisits = document.stats().membershipVisits;
    ASSERT_GT(firstVisits, 0u);
    Document independent; Error independentError;
    ASSERT_TRUE(independent.load(path.c_str(), limits, independentError)) << independentError.message;
    EXPECT_EQ(firstVisits, independent.stats().membershipVisits);
    bytes.create(11); ASSERT_TRUE(load()) << error.message;
    EXPECT_GT(document.stats().membershipVisits, firstVisits);
    EXPECT_EQ(firstVisits, independent.stats().membershipVisits);
}

TEST(PvdDomMembership, StructuralAndMembershipListsAgreeAfterEveryMutation)
{
    OmniPvdObject parents[2], children[6];
    children[4].mSkipMembershipTraversal = children[5].mSkipMembershipTraversal = true;
    auto check = [&]()
    {
        for (auto& parent : parents)
        {
            std::vector<OmniPvdObject*> expected, forward, reverse;
            for (auto* child = parent.mFirstChild; child; child = child->mNextSibling)
            {
                ASSERT_LT(expected.size(), 7u);
                EXPECT_EQ(&parent, child->mAncestor);
                if (!child->mSkipMembershipTraversal) expected.push_back(child);
            }
            for (auto* child = parent.mFirstMembershipChild; child; child = child->mNextMembershipSibling)
            { ASSERT_LT(forward.size(), 7u); forward.push_back(child); }
            for (auto* child = parent.mLastMembershipChild; child; child = child->mPrevMembershipSibling)
            { ASSERT_LT(reverse.size(), 7u); reverse.push_back(child); }
            EXPECT_EQ(expected, forward); std::reverse(reverse.begin(), reverse.end()); EXPECT_EQ(expected, reverse);
        }
    };
    for (unsigned i : {0u, 4u, 1u, 2u}) { parents[0].appendChild(&children[i]); check(); }
    parents[0].insertChildFirst(&children[3]); check();
    parents[0].insertChildFirst(&children[5]); check();
    parents[0].removeChild(&children[3]); check();
    parents[0].removeChild(&children[1]); check();
    parents[0].removeChild(&children[2]); check();
    parents[1].appendChild(&children[0]); check();
    parents[0].removeChild(&children[4]); check();
    parents[0].removeChild(&children[5]); check();
    parents[1].removeChild(&children[0]); check();
}

TEST_F(PvdDocument, MovedFirstMiddleAndLastActorsSurviveOriginalSceneDestruction)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1); bytes.cls(2, "PxRigidDynamic");
    bytes.create(100); bytes.create(900); bytes.frame(1, false, 100); bytes.frame(1, false, 900);
    for (uint64_t handle = 10; handle < 15; ++handle)
    { bytes.create(handle, 2); bytes.reference(100, 1, handle, true); }
    bytes.frame(2, false, 100); bytes.frame(2, false, 900);
    for (uint64_t handle : {10ull, 12ull, 14ull}) bytes.reference(900, 1, handle, true);
    bytes.frame(3, false, 100);
    bytes.command(OmniPvdCommand::eDESTROY_OBJECT); bytes.put(uint64_t(1)); bytes.put(uint64_t(100));
    ASSERT_TRUE(load()) << error.message;
    for (uint64_t handle : {10ull, 12ull, 14ull})
    {
        const auto& spans = historyFor(document, handle).memberships;
        ASSERT_EQ(2u, spans.size()); EXPECT_EQ(2u, spans[0].endFrame);
        EXPECT_EQ(900u, spans[1].context); EXPECT_EQ(0u, spans[1].stopOrdinal);
    }
    for (uint64_t handle : {11ull, 13ull})
    { const auto& spans = historyFor(document, handle).memberships; ASSERT_EQ(1u, spans.size()); EXPECT_EQ(3u, spans[0].endFrame); }
}

TEST_F(PvdDocument, SegmentAdvanceClearsLargeOpenMembershipSet)
{
    bytes.cls(1, "PxOmniPvdMetaData"); bytes.create(1);
    bytes.cls(2, "PxScene"); bytes.list(1, "actors", 2); bytes.cls(3, "PxRigidDynamic");
    bytes.create(100, 2); bytes.frame(1, false, 100);
    for (uint64_t handle = 1000; handle < 1256; ++handle)
    { bytes.create(handle, 3); bytes.reference(100, 1, handle, true); }
    bytes.create(2); ASSERT_TRUE(load()) << error.message;
    const auto before = document.stats().membershipVisits;
    for (uint64_t handle = 3; handle < 11; ++handle) bytes.create(handle);
    ASSERT_TRUE(load()) << error.message;
    EXPECT_LT(document.stats().membershipVisits - before, 32u);
}

TEST_F(PvdDocument, MappedJointLookupWorkIsConstantAcrossWideSceneAndReplacement)
{
    for (unsigned width : {1u, 128u})
    {
        bytes = Bytes(); limits = LoadLimits();
        bytes.cls(1, "PxScene"); bytes.list(1, "constraints", 1);
        bytes.cls(2, "PxFixedJoint"); bytes.attr(2, "constraint", 2, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.cls(3, "PxActor"); bytes.list(3, "actors", 1); bytes.create(100); bytes.create(10, 2);
        for (unsigned i = 0; i < width; ++i)
        {
            bytes.create(1000 + i, 3); bytes.reference(100, 3, 1000 + i, true);
        }
        bytes.reference(10, 2, 500); ASSERT_TRUE(load()) << error.message;
        const auto* scene = historyFor(document, 100).object;
        unsigned children = 0;
        for (auto* child = scene->mFirstChild; child; child = child->mNextSibling)
        { ASSERT_LT(children, width); EXPECT_EQ(1000u + children++, child->mOmniAPIHandle); }
        ASSERT_EQ(width, children);
        const auto before = document.stats().handlerSteps;
        for (unsigned i = 0; i < 64; ++i)
        { bytes.reference(100, 1, 500, true); bytes.removeReference(100, 1, 500); }
        ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(64u, document.stats().handlerSteps - before);
        scene = historyFor(document, 100).object;
        const auto* branch = historyFor(document, 10).object->mAncestor;
        ASSERT_NE(nullptr, branch); EXPECT_EQ(scene, branch->mAncestor);
        EXPECT_EQ(scene->mLastChild, branch); EXPECT_EQ("Joints", branch->mObjectName);
        children = 0;
        for (auto* child = scene->mFirstChild; child != branch; child = child->mNextSibling)
        { ASSERT_NE(nullptr, child); ASSERT_LT(children, width); ++children; }
        ASSERT_EQ(width, children);

        // Native consumers may rename a branch's display label. Attachment must
        // retain its identity instead of scanning labels and creating another branch.
        OmniPvdDOMState replay; OmniPvdReaderImpl reader; bytes.position = 0; reader.setReadStream(bytes);
        OmniPvdObject* firstBranch = nullptr;
        size_t creations = 0; unsigned attachments = 0;
        for (auto command = reader.getNextCommand(); command != OmniPvdCommand::eINVALID; command = reader.getNextCommand())
        {
            applyPvdDomCommand(reader, replay, command);
            if (command != OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE || reader.getAttributeHandle() != 1) continue;
            auto* joint = replay.mObjectHandleToObjectMap.at(replay.mExternalToInternalHandleMap.at(10));
            if (!attachments++)
            {
                firstBranch = joint->mAncestor; ASSERT_NE(nullptr, firstBranch);
                EXPECT_EQ("Joints", firstBranch->mObjectName);
                EXPECT_EQ(firstBranch, firstBranch->mAncestor->mLastChild);
                firstBranch->mObjectName = "Renamed joint branch";
                creations = replay.mObjectCreations.size();
            }
            EXPECT_EQ(firstBranch, joint->mAncestor);
            EXPECT_EQ(creations, replay.mObjectCreations.size());
        }
        EXPECT_EQ(64u, attachments);
        limits.maxHandlerSteps = before + 63; rejected(Error::Code::Limit);
        EXPECT_EQ("Handler-step limit exceeded", error.message); ++limits.maxHandlerSteps;
        ASSERT_TRUE(load()) << error.message;
        limits = LoadLimits(); bytes.create(100); bytes.create(10, 2);
        bytes.reference(100, 1, 500, true); bytes.reference(10, 2, 500);
        ASSERT_TRUE(load()) << error.message;
        const auto* joint = historyFor(document, 10).object;
        ASSERT_NE(nullptr, joint->mAncestor);
        EXPECT_EQ(historyFor(document, 100).object, joint->mAncestor->mAncestor);
        EXPECT_EQ("Joints", joint->mAncestor->mObjectName);
        EXPECT_EQ(document.stats().handlerSteps, before + 65);
    }
}

TEST_F(PvdDocument, OpenMembershipReservationTracksPeakConcurrentActors)
{
    uint64_t retained[2] = {};
    size_t streamBytes[2] = {};
    for (unsigned grouped = 0; grouped < 2; ++grouped)
    {
        bytes = Bytes(); limits = LoadLimits();
        bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, "PxRigidDynamic"); bytes.create(100, 1); bytes.frame(1, false, 100);
        for (uint64_t actor = 1000; actor < 1064; ++actor) bytes.create(actor, 2);
        if (grouped)
        {
            for (uint64_t actor = 1000; actor < 1064; ++actor) bytes.reference(100, 1, actor, true);
            for (uint64_t actor = 1000; actor < 1064; ++actor) bytes.removeReference(100, 1, actor);
        }
        else
        {
            for (uint64_t actor = 1000; actor < 1064; ++actor)
            { bytes.reference(100, 1, actor, true); bytes.removeReference(100, 1, actor); }
        }
        streamBytes[grouped] = bytes.data.size();
        ASSERT_TRUE(load()) << error.message;
        retained[grouped] = document.stats().accountedRetainedBytes;
        for (uint64_t actor = 1000; actor < 1064; ++actor)
        {
            const auto& memberships = historyFor(document, actor).memberships;
            ASSERT_EQ(1u, memberships.size());
            EXPECT_NE(0u, memberships[0].stopOrdinal);
        }
    }
    EXPECT_EQ(streamBytes[0], streamBytes[1]);
    ASSERT_GE(retained[1], retained[0]);
    EXPECT_GE(retained[1] - retained[0], 63u * 128u);
}

TEST_F(PvdDocument, MembershipsSurviveMiddleRemovalRejoinAndSegmentClear)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1); bytes.cls(2, "PxRigidDynamic");
    bytes.cls(3, "PxOmniPvdMetaData"); bytes.create(1, 3); bytes.create(100); bytes.frame(1, false, 100);
    for (uint64_t handle : {10ull, 11ull, 12ull}) { bytes.create(handle, 2); bytes.reference(100, 1, handle, true); }
    std::vector<uint64_t> open{10, 11, 12};
    uint64_t closeFrame = 2;
    auto check = [&]()
    {
        ASSERT_TRUE(load()) << error.message;
        for (uint64_t handle : {10ull, 11ull, 12ull})
        {
            const auto& memberships = historyFor(document, handle).memberships;
            ASSERT_FALSE(memberships.empty());
            const bool expectedOpen = std::find(open.begin(), open.end(), handle) != open.end();
            EXPECT_EQ(expectedOpen, memberships.back().stopOrdinal == 0) << handle;
            if (!expectedOpen) EXPECT_EQ(closeFrame, memberships.back().endFrame) << handle;
        }
    };
    check();
    bytes.frame(2, false, 100);
    for (uint64_t handle : {11ull, 12ull, 10ull})
    { bytes.removeReference(100, 1, handle); open.erase(std::find(open.begin(), open.end(), handle)); check(); }
    for (uint64_t handle : {12ull, 11ull, 10ull})
    { bytes.reference(100, 1, handle, true); open.push_back(handle); check(); }
    bytes.create(2, 3); open.clear(); closeFrame = 3; check();
}

TEST_F(PvdDocument, OwnedSubtreeToggleWorkScalesLinearly)
{
    uint64_t previous = 0;
    for (uint64_t count : {128ull, 256ull})
    {
        bytes = Bytes(); bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, "PxRigidDynamic"); bytes.list(2, "shapes", 2); bytes.cls(3, "PxShape");
        bytes.create(100); bytes.create(10, 2); bytes.frame(1, false, 100); bytes.reference(100, 1, 10, true);
        for (uint64_t i = 0; i < count; ++i)
        { bytes.create(1000 + i, 3); bytes.reference(10, 2, 1000 + i, true); }
        for (uint64_t f = 2; f <= 17; ++f)
        {
            bytes.frame(f, false, 100); bytes.removeReference(100, 1, 10);
            bytes.reference(100, 1, 10, true);
        }
        ASSERT_TRUE(load()) << error.message;
        const auto visits = document.stats().membershipVisits;
        EXPECT_LT(visits, 120 * count);
        if (previous) EXPECT_LT(visits, previous * 2 + 128);
        previous = visits;
        for (uint64_t i = 0; i < count; ++i)
        {
            const auto& spans = historyFor(document, 1000 + i).memberships;
            ASSERT_EQ(17u, spans.size()); EXPECT_EQ(0u, spans.back().stopOrdinal);
        }
    }
}

TEST_F(PvdDocument, SegmentAdvanceRefreshesOnlyOpenMemberships)
{
    bytes.cls(1, "PxOmniPvdMetaData"); bytes.create(1);
    bytes.cls(2, "PxScene"); bytes.list(1, "actors", 2);
    bytes.cls(3, "PxRigidDynamic"); bytes.create(100, 2); bytes.frame(1, false, 100);
    for (uint64_t handle = 1000; handle < 1128; ++handle)
    {
        bytes.create(handle, 3); bytes.reference(100, 1, handle, true);
        if (handle != 1000) bytes.removeReference(100, 1, handle);
    }
    ASSERT_TRUE(load()) << error.message;
    const uint64_t before = document.stats().membershipVisits;
    bytes.create(2); bytes.create(3); bytes.create(4);
    ASSERT_TRUE(load()) << error.message;
    EXPECT_LT(document.stats().membershipVisits - before, 16u);
    const auto& active = historyFor(document, 1000).memberships;
    ASSERT_EQ(1u, active.size()); EXPECT_NE(0u, active.back().stopOrdinal);
    const auto& closed = historyFor(document, 1001).memberships;
    ASSERT_EQ(1u, closed.size()); EXPECT_NE(0u, closed.back().stopOrdinal);
}

void references(Bytes& bytes, OmniPvdCommand::Enum command, uint64_t object, uint32_t attribute,
                std::initializer_list<uint64_t> handles)
{
    const size_t start = bytes.data.size();
    const uint32_t size = static_cast<uint32_t>(handles.size() * sizeof(uint64_t));
    bytes.set(size, attribute, object);
    bytes.data[start] = uint8_t(command);
    memcpy(bytes.data.data() + bytes.data.size() - size, handles.begin(), size);
}

TEST_F(PvdDocument, RepeatedGenericReferencesRefreshEveryReopenedTarget)
{
    for (unsigned mode : {0u, 1u, 2u})
    {
        SCOPED_TRACE(mode);
        bytes = Bytes();
        bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, "PxActor"); bytes.cls(3, "CustomActor", 2);
        if (mode == 2) bytes.list(2, "references", 3);
        else bytes.attr(2, "references", 3, OmniPvdDataType::eOBJECT_HANDLE, mode == 0 ? 1 : 0);
        const auto command = mode == 2 ? OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE : OmniPvdCommand::eSET_ATTRIBUTE;
        bytes.create(100); bytes.create(10, 3); bytes.create(20, 3); bytes.create(30, 3);
        bytes.frame(1, false, 100);
        for (uint64_t handle : {10ull, 20ull, 30ull}) bytes.reference(100, 1, handle, true);
        if (mode == 1) references(bytes, command, 10, 2, {20, 30, 20});
        else for (uint64_t handle : {20ull, 30ull}) references(bytes, command, 10, 2, {handle});
        bytes.frame(2, false, 100);
        for (uint64_t handle : {20ull, 30ull}) bytes.removeReference(100, 1, handle);
        bytes.frame(3, false, 100);
        if (mode == 1) references(bytes, command, 10, 2, {20, 30, 20});
        else for (uint64_t handle : {20ull, 30ull}) references(bytes, command, 10, 2, {handle});
        ASSERT_TRUE(load()) << error.message;
        for (uint64_t handle : {20ull, 30ull})
        {
            const auto& target = historyFor(document, handle);
            ASSERT_EQ(2u, target.object->mLifeSpans.size());
            EXPECT_EQ(0u, target.object->mLifeSpans.back().mFrameStop);
            EXPECT_EQ(2u, target.memberships.size());
            if (target.memberships.size() != 2) continue;
            EXPECT_EQ(2u, target.memberships[0].endFrame);
            EXPECT_EQ(3u, target.memberships[1].startFrame);
            EXPECT_EQ(document.stats().commands - (mode != 1 && handle == 20 ? 1 : 0), target.memberships[1].startCommand);
            EXPECT_EQ(0u, target.memberships[1].stopOrdinal);
        }
    }
}

TEST_F(PvdDocument, DuplicateFirstGenericArrayReferenceReopensOnlyRepeatedHandle)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
    bytes.cls(2, "PxActor"); bytes.cls(3, "CustomActor", 2);
    bytes.attr(2, "references", 3, OmniPvdDataType::eOBJECT_HANDLE, 0);
    bytes.create(100); bytes.create(10, 3); bytes.create(20, 3); bytes.create(30, 3);
    bytes.frame(1, false, 100);
    for (uint64_t handle : {10ull, 20ull, 30ull}) bytes.reference(100, 1, handle, true);
    bytes.frame(2, false, 100);
    bytes.removeReference(100, 1, 20); bytes.removeReference(100, 1, 30);
    bytes.frame(3, false, 100);
    references(bytes, OmniPvdCommand::eSET_ATTRIBUTE, 10, 2, {20, 30, 20});
    ASSERT_TRUE(load()) << error.message;
    const auto& repeated = historyFor(document, 20);
    ASSERT_EQ(2u, repeated.object->mLifeSpans.size());
    ASSERT_EQ(2u, repeated.memberships.size()); EXPECT_EQ(3u, repeated.memberships[1].startFrame);
    const auto& fresh = historyFor(document, 30);
    ASSERT_EQ(1u, fresh.object->mLifeSpans.size()); EXPECT_NE(0u, fresh.object->mLifeSpans.back().mFrameStop);
    ASSERT_EQ(1u, fresh.memberships.size()); EXPECT_EQ(2u, fresh.memberships[0].endFrame);
}

TEST_F(PvdDocument, DistinctFirstGenericReferencesDoNotWalkClosedTargets)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
    bytes.cls(2, "PxActor"); bytes.cls(3, "CustomActor", 2);
    bytes.attr(2, "references", 3, OmniPvdDataType::eOBJECT_HANDLE, 0);
    bytes.create(100); bytes.create(10, 3); bytes.frame(1, false, 100);
    bytes.reference(100, 1, 10, true);
    for (uint64_t handle = 1000; handle < 1016; ++handle)
    { bytes.create(handle, 3); bytes.reference(100, 1, handle, true); }
    bytes.frame(2, false, 100);
    for (uint64_t handle = 1000; handle < 1016; ++handle) bytes.removeReference(100, 1, handle);
    bytes.frame(3, false, 100);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t before = document.stats().membershipVisits;
    references(bytes, OmniPvdCommand::eSET_ATTRIBUTE, 10, 2,
        {1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009, 1010, 1011, 1012, 1013, 1014, 1015});
    ASSERT_TRUE(load()) << error.message;
    EXPECT_LT(document.stats().membershipVisits - before, 8u);
    for (uint64_t handle = 1000; handle < 1016; ++handle)
        EXPECT_EQ(1u, historyFor(document, handle).memberships.size());
}

TEST_F(PvdDocument, GenericReopenUsesMappedIncarnationAndRejectsSpecializedTargets)
{
    for (bool genericReplacement : {false, true})
    {
        SCOPED_TRACE(genericReplacement);
        bytes = Bytes();
        bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, "PxActor"); bytes.cls(3, "CustomActor", 2); bytes.cls(4, "PxRigidDynamic", 2);
        bytes.attr(2, "reference", 3, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.create(100); bytes.create(10, 3); bytes.create(20, 3); bytes.frame(1, false, 100);
        bytes.reference(100, 1, 10, true); bytes.reference(100, 1, 20, true); bytes.reference(10, 2, 20);
        ASSERT_TRUE(load()) << error.message;
        const uint64_t previousIncarnation = historyFor(document, 20).object->mUID;
        bytes.frame(2, false, 100); bytes.create(20, genericReplacement ? 3 : 4);
        bytes.frame(3, false, 100); bytes.reference(10, 2, 20);
        ASSERT_TRUE(load()) << error.message;
        const auto& original = document.history().at(previousIncarnation);
        ASSERT_EQ(genericReplacement ? 2u : 1u, original.object->mLifeSpans.size());
        ASSERT_EQ(genericReplacement ? 2u : 1u, original.memberships.size());
        if (genericReplacement) EXPECT_EQ(3u, original.memberships.back().startFrame);
        else EXPECT_EQ(2u, original.memberships.back().endFrame);
        EXPECT_TRUE(historyFor(document, 20).memberships.empty());
    }
}

TEST_F(PvdDocument, GenericReopenRefreshesOpenDescendantsOfClosedParent)
{
    for (bool reopenChildFirst : {false, true})
    {
        SCOPED_TRACE(reopenChildFirst);
        bytes = Bytes();
        bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, "PxActor"); bytes.cls(5, "PxArticulationLink", 2); bytes.cls(3, "CustomLink", 5);
        bytes.attr(2, "reference", 3, OmniPvdDataType::eOBJECT_HANDLE, 0);
        bytes.cls(4, "PxArticulationJointReducedCoordinate");
        bytes.attr(3, "parentLink", 4, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.attr(4, "childLink", 4, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.create(100); bytes.create(10, 3); bytes.create(20, 3); bytes.create(21, 3); bytes.create(22, 4);
        bytes.frame(1, false, 100); bytes.reference(100, 1, 10, true); bytes.reference(100, 1, 20, true);
        bytes.reference(22, 3, 20); bytes.reference(22, 4, 21);
        bytes.reference(10, 2, 20); bytes.reference(10, 2, 21);
        bytes.frame(2, false, 100); bytes.removeReference(100, 1, 20);
        bytes.frame(3, false, 100);
        if (reopenChildFirst) bytes.reference(10, 2, 21);
        ASSERT_TRUE(load()) << error.message;
        const uint64_t visits = document.stats().membershipVisits;
        bytes.frame(4, false, 100);
        references(bytes, OmniPvdCommand::eSET_ATTRIBUTE, 10, 2, {20, 21, 20, 21, 20, 21, 20, 21, 20, 21});
        ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(6u, document.stats().membershipVisits - visits) << "Three subtree visits and three deduplicated refreshes";
        for (uint64_t handle : {20ull, 21ull})
        {
            const auto& target = historyFor(document, handle);
            ASSERT_EQ(2u, target.memberships.size()); EXPECT_EQ(4u, target.memberships.back().startFrame);
        }
        const auto& joint = historyFor(document, 22);
        ASSERT_EQ(1u, joint.memberships.size()); EXPECT_EQ(2u, joint.memberships[0].endFrame);
        const uint64_t reopeningCommandOrdinal = document.stats().commands;
        limits.maxRetainedBytes = document.stats().accountedRetainedBytes - 1;
        rejected(Error::Code::Limit);
        EXPECT_EQ(reopeningCommandOrdinal, error.commandIndex);
        limits = LoadLimits();
    }
}

TEST_F(PvdDocument, GenericRemoveAndSpecializedOwnersDoNotReopenReferences)
{
    for (const char* owner : {"CustomActor", "PxRigidDynamic", "PxConvexMeshGeometry"})
    {
        SCOPED_TRACE(owner);
        bytes = Bytes();
        bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
        bytes.cls(2, "PxActor"); bytes.cls(3, "CustomTarget", 2); bytes.cls(4, owner, 2);
        bytes.list(2, "references", 4);
        bytes.create(100); bytes.create(10, 4); bytes.create(20, 3); bytes.frame(1, false, 100);
        bytes.reference(100, 1, 20, true); bytes.reference(10, 2, 20, true);
        bytes.frame(2, false, 100); bytes.removeReference(100, 1, 20);
        ASSERT_TRUE(load()) << error.message;
        const uint64_t visits = document.stats().membershipVisits;
        bytes.frame(3, false, 100);
        if (std::string(owner) == "CustomActor") bytes.removeReference(10, 2, 20);
        else bytes.reference(10, 2, 20, true);
        ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(visits, document.stats().membershipVisits);
        const auto& target = historyFor(document, 20);
        ASSERT_EQ(1u, target.object->mLifeSpans.size()); EXPECT_NE(0u, target.object->mLifeSpans.back().mFrameStop);
        ASSERT_EQ(1u, target.memberships.size()); EXPECT_EQ(2u, target.memberships[0].endFrame);
    }
}

TEST_F(PvdDocument, ReopeningChargesOnlyNewSubtreeLifespans)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
    bytes.cls(2, "PxRigidDynamic"); bytes.list(2, "shapes", 2);
    bytes.cls(3, "PxShape");
    bytes.create(1); bytes.create(10, 2);
    for (uint64_t i = 0; i < 256; ++i)
    {
        bytes.create(100 + i, 3); bytes.reference(10, 2, 100 + i, true);
    }
    bytes.reference(1, 1, 10, true);
    bytes.frame(1);
    const size_t removal = bytes.data.size();
    bytes.reference(1, 1, 10, true);
    bytes.frame(2);
    bytes.reference(1, 1, 10, true);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t withoutReopening = document.stats().accountedRetainedBytes;
    bytes.data[removal] = uint8_t(OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t withReopening = document.stats().accountedRetainedBytes;
    EXPECT_EQ(257 * 2 * (sizeof(OmniPvdObjectLifeSpan) + sizeof(SceneMembership)), withReopening - withoutReopening);
    const auto* dom = document.dom();
    for (uint64_t handle : {10ull, 100ull, 355ull})
    {
        const auto* object = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(handle));
        ASSERT_EQ(2u, object->mLifeSpans.size());
        EXPECT_EQ(2u, object->mLifeSpans.back().mFrameStart);
    }
    limits.maxRetainedBytes = withReopening - 1;
    rejected(Error::Code::Limit);
    EXPECT_EQ(bytes.data.size() - 34, error.byteOffset);
}

TEST_F(PvdDocument, ReopeningBudgetPrunesIndependentlyDetachedDescendants)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1);
    bytes.cls(2, "PxRigidDynamic"); bytes.list(2, "shapes", 2);
    bytes.cls(3, "PxShape"); bytes.attr(3, "geom", 3, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.cls(4, "PxBoxGeometry");
    bytes.create(1); bytes.create(10, 2);
    for (uint64_t shape : {20ull, 30ull})
    {
        bytes.create(shape, 3); bytes.create(shape + 1, 4);
        bytes.reference(shape, 3, shape + 1); bytes.reference(10, 2, shape, true);
    }
    bytes.reference(1, 1, 10, true); bytes.frame(1);
    bytes.removeReference(10, 2, 20); bytes.removeReference(1, 1, 10);
    bytes.frame(2);
    const size_t finalCommand = bytes.data.size();
    bytes.removeReference(1, 1, 10);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t withoutReopening = document.stats().accountedRetainedBytes;
    bytes.data[finalCommand] = uint8_t(OmniPvdCommand::eADD_TO_UNIQUE_LIST_ATTRIBUTE);
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(3 * 2 * (sizeof(OmniPvdObjectLifeSpan) + sizeof(SceneMembership)), document.stats().accountedRetainedBytes - withoutReopening);
    const auto* dom = document.dom();
    for (uint64_t handle : {20ull, 21ull, 30ull, 31ull})
    {
        const auto* object = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(handle));
        EXPECT_EQ(handle < 30 ? 1u : 2u, object->mLifeSpans.size());
        EXPECT_EQ(handle < 30 ? 1u : 0u, object->mLifeSpans.back().mFrameStop);
    }
}

TEST_F(PvdDocument, EverySubtreeReopenHandlerChargesGrowth)
{
    struct Edge { const char* owner; const char* attribute; const char* target; };
    const Edge edges[] = {
        {"PxScene", "actors", "PxRigidDynamic"},
        {"PxScene", "articulations", "PxArticulationReducedCoordinate"},
        {"PxRigidDynamic", "shapes", "PxShape"},
        {"PxDeformableVolume", "shapes", "PxShape"},
        {"PxDeformableSurface", "shapes", "PxShape"},
        {"PxPBDParticleSystem", "particleBuffers", "PxParticleBuffer"}
    };
    for (const Edge& edge : edges)
    {
        SCOPED_TRACE(edge.owner);
        SCOPED_TRACE(edge.attribute);
        bytes = Bytes();
        bytes.cls(1, edge.owner); bytes.list(1, edge.attribute, 1);
        bytes.cls(2, edge.target); bytes.create(10); bytes.create(20, 2);
        bytes.reference(10, 1, 20, true); bytes.frame(1);
        const size_t removal = bytes.data.size();
        bytes.reference(10, 1, 20, true); bytes.reference(10, 1, 20, true);
        ASSERT_TRUE(load()) << error.message;
        const uint64_t withoutReopening = document.stats().accountedRetainedBytes;
        bytes.data[removal] = uint8_t(OmniPvdCommand::eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE);
        ASSERT_TRUE(load()) << error.message;
        const uint64_t membershipBytes = std::string(edge.owner) == "PxScene" ? 2 * sizeof(SceneMembership) : 0;
        EXPECT_EQ(2 * sizeof(OmniPvdObjectLifeSpan) + membershipBytes, document.stats().accountedRetainedBytes - withoutReopening);
        const auto* dom = document.dom();
        const auto* object = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(20));
        EXPECT_EQ(2u, object->mLifeSpans.size());
    }
}

TEST_F(PvdDocument, CleanEof)
{
    schema(); bytes.set();
    ASSERT_TRUE(load()) << error.message;
    ASSERT_NE(nullptr, document.dom());
    EXPECT_EQ(1u, document.stats().creates);
    EXPECT_EQ(4u, document.stats().commands);
    EXPECT_EQ(0u, document.stats().ignoredCommands);
}

TEST_F(PvdDocument, UnknownOwnerAndAttributeCommandsPreserveKnownState)
{
    EncodedStreamBytes output;
    OmniPvdWriterImpl implementation;
    OmniPvdWriter& writer = implementation;
    writer.setWriteStream(output);
    const auto cls = writer.registerClass("PxMaterial");
    const auto value = writer.registerAttribute(cls, "dynamicFriction", OmniPvdDataType::eFLOAT32, 1);
    const auto list = writer.registerUniqueListAttribute(cls, "values", OmniPvdDataType::eUINT32);
    writer.createObject(1, cls, 10, "known");
    const float friction = 0.75f;
    const uint32_t element = 17;
    writer.setAttribute(1, 10, value, reinterpret_cast<const uint8_t*>(&friction), sizeof(friction));
    writer.destroyObject(1, 276);
    for (const auto owner : {uint64_t(20), uint64_t(10)})
    {
        const auto scalarAttribute = owner == 20 ? value : 999;
        const auto listAttribute = owner == 20 ? list : 999;
        writer.setAttribute(1, owner, scalarAttribute, reinterpret_cast<const uint8_t*>(&friction), sizeof(friction));
        writer.addToUniqueListAttribute(1, owner, listAttribute, reinterpret_cast<const uint8_t*>(&element), sizeof(element));
        writer.removeFromUniqueListAttribute(1, owner, listAttribute, reinterpret_cast<const uint8_t*>(&element), sizeof(element));
    }
    writer.setAttribute(1, 20, 0, reinterpret_cast<const uint8_t*>(&friction), sizeof(friction));
    writer.startFrame(1, 9);
    writer.addToUniqueListAttribute(1, 10, list, reinterpret_cast<const uint8_t*>(&element), sizeof(element));
    bytes = output;
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(1u, document.stats().creates);
    EXPECT_EQ(2u, document.stats().samples);
    EXPECT_EQ(15u, document.stats().commands);
    EXPECT_EQ(8u, document.stats().ignoredCommands);
    EXPECT_EQ(bytes.data.size(), document.stats().streamBytes);
    ASSERT_EQ(1u, document.frames().size());
    EXPECT_EQ(9u, document.frames()[0].recordedFrame);
    const auto* dom = document.dom();
    const auto* object = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(10));
    const auto& slots = object->mInheritedClassInstances[0].mClassAttributeLists;
    ASSERT_EQ(2u, slots.size());
    auto* sample = static_cast<OmniPvdAttributeSample*>(slots[0]->mLast);
    ASSERT_NE(nullptr, sample);
    ASSERT_EQ(sizeof(friction), sample->mDataLen);
    EXPECT_EQ(0, memcmp(&friction, sample->mData, sizeof(friction)));
    auto* values = static_cast<OmniPvdUniqueList*>(slots[1]->mLast);
    ASSERT_NE(nullptr, values);
    ASSERT_EQ(1u, values->mElements.size());
    EXPECT_EQ(0, memcmp(&element, values->mElements.front()->mData, sizeof(element)));
    limits.maxCommands = 12;
    rejected(Error::Code::Limit);
    EXPECT_EQ(13u, error.commandIndex);
    limits = LoadLimits(); limits.maxStreamBytes = bytes.data.size() - 1;
    rejected(Error::Code::Limit);
}

TEST_F(PvdDocument, EachUnknownCommandIsCountedOnce)
{
    for (unsigned variant = 0; variant < 8; ++variant)
    {
        SCOPED_TRACE(variant);
        EncodedStreamBytes output;
        OmniPvdWriterImpl implementation;
        OmniPvdWriter& writer = implementation;
        writer.setWriteStream(output);
        const auto cls = writer.registerClass("PxMaterial");
        const auto value = writer.registerAttribute(cls, "dynamicFriction", OmniPvdDataType::eFLOAT32, 1);
        const auto list = writer.registerUniqueListAttribute(cls, "values", OmniPvdDataType::eUINT32);
        writer.createObject(1, cls, 10, "known");
        const uint32_t data = 0;
        const auto* payload = reinterpret_cast<const uint8_t*>(&data);
        if (variant == 0) writer.destroyObject(1, 276);
        else if (variant == 7) writer.setAttribute(1, 20, 0, payload, sizeof(data));
        else
        {
            const uint64_t owner = variant <= 3 ? 20 : 10;
            const auto attribute = variant <= 3 ? (variant == 1 ? value : list) : 999;
            switch ((variant - 1) % 3)
            {
            case 0: writer.setAttribute(1, owner, attribute, payload, sizeof(data)); break;
            case 1: writer.addToUniqueListAttribute(1, owner, attribute, payload, sizeof(data)); break;
            case 2: writer.removeFromUniqueListAttribute(1, owner, attribute, payload, sizeof(data)); break;
            }
        }
        writer.startFrame(1, 9);
        bytes = output;
        if (!load()) { ADD_FAILURE() << error.message; continue; }
        EXPECT_EQ(1u, document.stats().ignoredCommands);
        EXPECT_EQ(6u, document.stats().commands);
        EXPECT_EQ(0u, document.stats().samples);
        ASSERT_EQ(1u, document.frames().size());
        EXPECT_EQ(9u, document.frames()[0].recordedFrame);
        EXPECT_EQ(1u, document.dom()->mExternalToInternalHandleMap.size());
    }
}

TEST_F(PvdDocument, UnknownCommandsStillRejectTruncationAndPayloadLimits)
{
    EncodedStreamBytes output;
    OmniPvdWriterImpl implementation;
    OmniPvdWriter& writer = implementation;
    writer.setWriteStream(output);
    const uint32_t payload = 17;
    writer.setAttribute(1, 20, 0, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
    bytes = output;
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(1u, document.stats().ignoredCommands);
    limits.maxAttributeBytes = sizeof(payload) - 1;
    rejected(Error::Code::Limit);
    limits = LoadLimits();
    bytes.data.pop_back();
    rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, HeaderOnlyIsComplete)
{
    ASSERT_TRUE(load()) << error.message;
    EXPECT_NE(nullptr, document.dom());
    EXPECT_EQ(0u, document.stats().commands);
}
TEST_F(PvdDocument, TruncatedFixedField)
{
    schema(); bytes.frame(1); bytes.data.pop_back(); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, TruncatedPayload)
{
    schema(); bytes.set(); bytes.data.pop_back(); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, TruncatedFinalPayloadAtStreamCapIsInvalid)
{
    schema(); bytes.set(); bytes.data.pop_back();
    limits.maxStreamBytes = bytes.data.size();
    rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, MalformedTrailingByte)
{
    schema(); bytes.put(uint8_t(255)); rejected(Error::Code::Invalid);
    EXPECT_GT(error.byteOffset, 12u);
    EXPECT_EQ(4u, error.commandIndex);
}
TEST_F(PvdDocument, DeclaredPayloadPastEofIsInvalid)
{
    schema(); bytes.set(0); bytes.data.resize(bytes.data.size() - 4); bytes.put(uint32_t(65));
    limits.maxAttributeBytes = 64; rejected(Error::Code::Invalid);
    EXPECT_EQ(Error::LimitKind::None, error.limitKind);
}
TEST_F(PvdDocument, PayloadLimitAcceptsExactSize)
{
    schema(); bytes.set(4);
    limits.maxAttributeBytes = 4;
    ASSERT_TRUE(load()) << error.message;
    limits.maxAttributeBytes = 3;
    rejected(Error::Code::Limit);
    EXPECT_EQ("Attribute payload exceeds the payload limit", error.message);
}
TEST_F(PvdDocument, RetainedBudgetChargesPayloadBytes)
{
    bytes.cls(); bytes.attr(1, "data", 1, OmniPvdDataType::eUINT8, 200 * 1024); bytes.create();
    ASSERT_TRUE(load()) << error.message;
    const uint64_t withoutPayload = document.stats().accountedRetainedBytes;
    bytes.set(200 * 1024);
    ASSERT_TRUE(load()) << error.message;
    EXPECT_GE(document.stats().accountedRetainedBytes - withoutPayload, 200u * 1024u);
    for (unsigned i = 1; i < 8; ++i) bytes.set(200 * 1024);
    limits.maxRetainedBytes = 1ull << 20;
    rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, RetainedBudgetAcceptsExactFit)
{
    ASSERT_TRUE(load()) << error.message;
    const uint64_t exact = document.stats().accountedRetainedBytes;
    limits.maxRetainedBytes = exact;
    ASSERT_TRUE(load()) << error.message;
    limits.maxRetainedBytes = exact - 1;
    rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, RetainedBudgetChargesNamesUpToTheReaderLimit)
{
    const auto accounted = [&](size_t cls, size_t attr, size_t obj) {
        const std::string c(cls, 'c'), a(attr, 'a'), o(obj, 'o');
        bytes = Bytes(); bytes.cls(1, c.c_str()); bytes.attr(1, a.c_str());
        bytes.command(OmniPvdCommand::eCREATE_OBJECT); bytes.put(uint64_t(1)); bytes.put(uint32_t(1));
        bytes.put(uint64_t(10)); bytes.name(o.c_str());
        EXPECT_TRUE(load()) << error.message;
        return document.stats().accountedRetainedBytes;
    };
    const uint64_t base = accounted(1, 1, 1);
    for (size_t length : {size_t(100), size_t(2047), size_t(2049)})
    {
        const uint64_t extra = 4 * ((std::min)(length, size_t(OMNI_PVD_MAX_STRING_LENGTH - 1)) - 1);
        EXPECT_EQ(base + extra, accounted(length, 1, 1)) << length;
        EXPECT_EQ(base + extra, accounted(1, length, 1)) << length;
        EXPECT_EQ(base + extra, accounted(1, 1, length)) << length;
    }
}
TEST_F(PvdDocument, StreamByteLimit)
{
    limits.maxStreamBytes = 11; rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, CommandLimit)
{
    schema(); limits.maxCommands = 2; rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, CreateLimit)
{
    schema(); bytes.create(11); limits.maxCreates = 1; rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, ClassLimit)
{
    bytes.cls(); bytes.cls(2); limits.maxClasses = 1; rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, AttributeLimit)
{
    bytes.cls(); bytes.attr(); bytes.attr(2, "second"); limits.maxAttributes = 1; rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, InheritanceDepthLimit)
{
    bytes.cls(); bytes.cls(2, "Derived", 1); limits.maxInheritanceDepth = 1; rejected(Error::Code::Limit);
    limits.maxInheritanceDepth = 2; ASSERT_TRUE(load()) << error.message;
}
TEST_F(PvdDocument, PendingReferenceConsumesRetainedBudget)
{
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.cls(2, "PxArticulationJointReducedCoordinate"); bytes.create(10); bytes.create(20, 2);
    bytes.reference(10, 1, 20);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t withoutPending = document.stats().accountedRetainedBytes;
    bytes = Bytes();
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.cls(2, "PxArticulationJointReducedCoordinate"); bytes.create(10);
    bytes.reference(10, 1, 20); bytes.create(20, 2);
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(withoutPending + 192, document.stats().accountedRetainedBytes);
}
TEST_F(PvdDocument, UnregisteredBase)
{
    bytes.cls(2, "Derived", 1); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, WrongAttributeOwner)
{
    schema(); bytes.cls(2); bytes.attr(2, "other", 2); bytes.set(4, 2); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, NestedAttributeUnsupported)
{
    bytes.cls(); bytes.command(OmniPvdCommand::eREGISTER_CLASS_ATTRIBUTE);
    bytes.put(uint32_t(1)); bytes.put(uint32_t(1)); bytes.put(uint32_t(1)); bytes.name("nested");
    rejected(Error::Code::Unsupported);
}
TEST_F(PvdDocument, NestedPathUnsupported)
{
    schema(); bytes.set(4, 1, 10, 2); rejected(Error::Code::Unsupported);
}
TEST_F(PvdDocument, ExcessReaderPathDepth)
{
    schema(); bytes.set(4, 1, 10, 33); rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, WrongSchemaForActorType)
{
    bytes.cls(1, "PxRigidDynamic"); bytes.attr(1, "type", 1, OmniPvdDataType::eUINT8);
    bytes.create(); bytes.set(1); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, WrongSchemaForShapeHandle)
{
    bytes.cls(1, "PxShape"); bytes.attr(1, "geom", 1, OmniPvdDataType::eUINT8);
    bytes.create(); bytes.set(1); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, WrongSchemaForMetadata)
{
    bytes.cls(1, "PxOmniPvdMetaData"); bytes.attr(1, "ovdIntegrationVersionMajor", 1, OmniPvdDataType::eUINT8);
    bytes.create(); bytes.set(1); bytes.create(11); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, FixedWidthMismatch)
{
    schema(); bytes.set(1); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, ValidFinalOpenFrame)
{
    schema(); bytes.frame(27);
    ASSERT_TRUE(load()) << error.message;
    ASSERT_EQ(1u, document.frames().size());
    EXPECT_EQ(0u, document.frames()[0].recordingSegment);
    EXPECT_EQ(1u, document.frames()[0].context);
    EXPECT_EQ(27u, document.frames()[0].recordedFrame);
    EXPECT_EQ(27u, document.frames()[0].sourceTimestamp);
    EXPECT_FALSE(document.frames()[0].completed);
}
TEST_F(PvdDocument, CompletedFramesAndLimit)
{
    schema(); bytes.frame(27); bytes.frame(28, true); bytes.frame(30);
    ASSERT_TRUE(load()) << error.message;
    ASSERT_EQ(2u, document.frames().size());
    EXPECT_TRUE(document.frames()[0].completed);
    limits.maxFrameRecords = 1; rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, FrameRetainedBudgetReportsRetainedLimit)
{
    ASSERT_TRUE(load()) << error.message;
    limits.maxRetainedBytes = document.stats().accountedRetainedBytes + 191;
    bytes.frame(1);
    rejected(Error::Code::Limit);
    EXPECT_EQ("Retained-data limit exceeded", error.message);
}
TEST_F(PvdDocument, LegacyFrameWalkChargesEveryScene)
{
    bytes.cls(1, "PxScene"); bytes.create(10); bytes.create(11); bytes.frame(1);
    limits.maxHandlerSteps = 1;
    rejected(Error::Code::Limit);
    EXPECT_EQ("Handler-step limit exceeded", error.message);
    limits.maxHandlerSteps = 2;
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(2u, document.stats().handlerSteps);
}
TEST_F(PvdDocument, FloatingLinkScanChargesEveryCreatedObject)
{
    bytes.cls(1, "PxArticulationLink"); bytes.cls(2, "PxArticulationReducedCoordinate");
    bytes.create(10); bytes.create(11); bytes.create(20, 2);
    ASSERT_TRUE(load()) << error.message;
    // This fixture performs one scan, after its final creation-list insertion.
    // Three streamed objects plus the ten initial synthetic roots/branches.
    const uint64_t expectedSteps = 13;
    EXPECT_EQ(expectedSteps, document.stats().handlerSteps);
    limits.maxHandlerSteps = expectedSteps - 1;
    rejected(Error::Code::Limit);
    EXPECT_EQ("Handler-step limit exceeded", error.message);
    limits.maxHandlerSteps = expectedSteps;
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(expectedSteps, document.stats().handlerSteps);
}
TEST_F(PvdDocument, IntegrationVersionFrameWalkGate)
{
    for (uint32_t minor : {3u, 4u})
    {
        bytes = Bytes(); limits = LoadLimits();
        bytes.cls(1, "PxOmniPvdMetaData");
        bytes.attr(1, "ovdIntegrationVersionMajor"); bytes.attr(2, "ovdIntegrationVersionMinor");
        bytes.create(); bytes.set(); bytes.data[bytes.data.size() - 4] = 1;
        bytes.set(4, 2); bytes.data[bytes.data.size() - 4] = uint8_t(minor);
        bytes.cls(2, "PxScene"); bytes.create(20, 2); bytes.create(21, 2);
        ASSERT_TRUE(load()) << error.message;
        const uint64_t schemaSteps = document.stats().handlerSteps;
        bytes.frame(1); limits.maxHandlerSteps = schemaSteps;
        if (minor == 3) rejected(Error::Code::Limit);
        else ASSERT_TRUE(load()) << error.message;
        limits.maxHandlerSteps = schemaSteps + 2;
        ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(schemaSteps + (minor == 3 ? 2 : 0), document.stats().handlerSteps);
    }
}
TEST_F(PvdDocument, FloatingLinkScanWithoutLinkClassDoesNoWork)
{
    bytes.cls(1, "PxArticulationReducedCoordinate"); bytes.create();
    limits.maxHandlerSteps = 0;
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(0u, document.stats().handlerSteps);
}
TEST_F(PvdDocument, ReverseSchemaScanChargesAncestorsClassesAndChains)
{
    for (uint32_t owner : {3u, 1u})
    {
        SCOPED_TRACE(owner);
        bytes = Bytes(); limits = LoadLimits();
        bytes.cls(1, "Base"); bytes.cls(2, "Mid", 1); bytes.cls(3, "Leaf", 2);
        bytes.attr(1, "value", owner);
        ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(9u, document.stats().handlerSteps);
        limits.maxHandlerSteps = 8;
        rejected(Error::Code::Limit);
        EXPECT_EQ("Handler-step limit exceeded", error.message);
        limits.maxHandlerSteps = 9;
        ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(9u, document.stats().handlerSteps);
    }
}
TEST_F(PvdDocument, ContactArraysDoNotPayForSyntheticNodes)
{
    for (const char* cls : {"PxScene", "Thing"})
    {
        bytes = Bytes(); limits = LoadLimits();
        bytes.cls(1, cls); bytes.attr(1, "pairsActors", 1, OmniPvdDataType::eOBJECT_HANDLE, 0);
        bytes.create(); ASSERT_TRUE(load()) << error.message;
        const uint64_t base = document.stats().accountedRetainedBytes;
        bytes.set(16);
        const uint64_t sampleCharge = (cls[0] == 'P' ? 288 : 12576) +
            2 * sizeof(SampleWrite) + 128 + 192 + 2 * sizeof(size_t);
        limits.maxRetainedBytes = base + sampleCharge - 1; rejected(Error::Code::Limit);
        ++limits.maxRetainedBytes; ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(base + sampleCharge, document.stats().accountedRetainedBytes);
    }
}
TEST_F(PvdDocument, DefaultParsedNonHandleSamplesDoNotPayForSyntheticNodes)
{
    bytes.cls(1, "Thing"); bytes.attr(1, "values", 1, OmniPvdDataType::eUINT32, 4); bytes.create();
    ASSERT_TRUE(load()) << error.message;
    const uint64_t base = document.stats().accountedRetainedBytes;
    bytes.set(16);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t semanticCharge = 2 * sizeof(SampleWrite) + 128 + 192 + 2 * sizeof(size_t);
    EXPECT_EQ(288u + semanticCharge, document.stats().accountedRetainedBytes - base);
}
TEST_F(PvdDocument, SchemaObjectCrossProductLimit)
{
    bytes.cls();
    ASSERT_TRUE(load()) << error.message;
    const uint64_t emptySchemaCost = document.stats().accountedRetainedBytes;
    for (uint64_t i = 10; i < 110; ++i) bytes.create(i);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t bareObjectCost = document.stats().accountedRetainedBytes - emptySchemaCost;
    bytes = Bytes(); bytes.cls();
    for (uint32_t i = 1; i <= 128; ++i) bytes.attr(i, ("value" + std::to_string(i)).c_str());
    ASSERT_TRUE(load()) << error.message;
    const uint64_t schemaCost = document.stats().accountedRetainedBytes;
    for (uint64_t i = 10; i < 110; ++i) bytes.create(i);
    limits.maxRetainedBytes = schemaCost + bareObjectCost + 1024;
    rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, FailedReloadDiscardsPreviousDocument)
{
    schema(); ASSERT_TRUE(load()) << error.message;
    bytes.put(uint8_t(255)); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, EnumAliasesAccepted)
{
    bytes.cls(1, "Flags");
    bytes.attr(1, "first", 1, OmniPvdDataType::eENUM_VALUE, 1);
    bytes.attr(2, "alias", 1, OmniPvdDataType::eENUM_VALUE, 1);
    bytes.cls(2); bytes.attr(3, "flags", 2, OmniPvdDataType::eFLAGS_WORD, 1);
    bytes.create(10, 2); bytes.set(4, 3);
    ASSERT_TRUE(load()) << error.message;
}
TEST_F(PvdDocument, VehicleEnumLabelsMayRepeat)
{
    // Legacy recordings can contain repeated labels; the writer now rejects them.
    const uint32_t cls = 1;
    bytes.cls(cls, "DirectionSpecifier");
    bytes.attr(1, "suspensionDir", cls, OmniPvdDataType::eENUM_VALUE, 0);
    bytes.attr(2, "geomNormalDir", cls, OmniPvdDataType::eENUM_VALUE, 1);
    bytes.attr(3, "geomNormalDir", cls, OmniPvdDataType::eENUM_VALUE, 2);
    ASSERT_TRUE(load()) << error.message;
    const auto* enumeration = document.dom()->mClassHandleToClassMap.at(cls);
    ASSERT_TRUE(enumeration->mIsEnumClass);
    ASSERT_EQ(3u, enumeration->mAttributeDefinitions.size());
    EXPECT_EQ(1u, enumeration->mAttributeDefinitions[1]->mNbrFields);
    EXPECT_EQ(2u, enumeration->mAttributeDefinitions[2]->mNbrFields);
    EXPECT_EQ("geomNormalDir", enumeration->mAttributeDefinitions[1]->mAttributeName);
    EXPECT_EQ("geomNormalDir", enumeration->mAttributeDefinitions[2]->mAttributeName);
}
TEST_F(PvdDocument, RepeatedEnumLabelsRespectLimits)
{
    bytes.cls(1, "Enumeration");
    bytes.attr(1, "label", 1, OmniPvdDataType::eENUM_VALUE, 1);
    ASSERT_TRUE(load()) << error.message;
    const auto firstCharge = document.stats().accountedRetainedBytes;
    bytes.attr(2, "label", 1, OmniPvdDataType::eENUM_VALUE, 2);
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(2u, document.stats().attributes);
    EXPECT_GT(document.stats().accountedRetainedBytes, firstCharge);
    limits.maxAttributes = 1;
    rejected(Error::Code::Limit);
    limits = LoadLimits();
    limits.maxRetainedBytes = firstCharge;
    rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, RejectsDuplicateEnumHandle)
{
    bytes.cls(1, "Enumeration");
    bytes.attr(1, "label", 1, OmniPvdDataType::eENUM_VALUE, 1);
    bytes.attr(1, "label", 1, OmniPvdDataType::eENUM_VALUE, 2);
    rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, RejectsMixedEnumAndValueAttributes)
{
    for (unsigned enumFirst = 0; enumFirst < 2; ++enumFirst)
    {
        bytes = Bytes();
        bytes.cls();
        bytes.attr(1, "first", 1, enumFirst ? OmniPvdDataType::eENUM_VALUE : OmniPvdDataType::eUINT32);
        bytes.attr(2, "second", 1, enumFirst ? OmniPvdDataType::eUINT32 : OmniPvdDataType::eENUM_VALUE);
        rejected(Error::Code::Invalid);
    }
}
TEST_F(PvdDocument, RejectsOrdinaryAttributeNameCollision)
{
    bytes.cls(); bytes.attr(1, "value"); bytes.attr(2, "value"); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, RejectsInheritedAttributeNameCollision)
{
    bytes.cls(); bytes.attr(1, "value"); bytes.cls(2, "Derived", 1);
    bytes.attr(2, "VALUE", 2); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, RejectsReverseOrderInheritedSanitizedCollision)
{
    const struct { const char* derived; const char* base; } labels[] = {
        {"A-B", "a_b"}, {"VALUE", "value"}
    };
    for (const auto& label : labels)
    {
        bytes = Bytes();
        bytes.cls(); bytes.cls(2, "Derived", 1);
        bytes.attr(1, label.derived, 2); bytes.attr(2, label.base, 1);
        rejected(Error::Code::Invalid);
        EXPECT_EQ("Ambiguous attribute name", error.message);
    }
}
TEST_F(PvdDocument, AllowsSameAttributeNameOnSiblingClasses)
{
    bytes.cls(); bytes.cls(2, "Left", 1); bytes.cls(3, "Right", 1);
    bytes.attr(1, "A-B", 2); bytes.attr(2, "a_b", 3);
    ASSERT_TRUE(load()) << error.message;
}
TEST_F(PvdDocument, ReverseCollisionChecksEveryAncestor)
{
    for (uint32_t ancestor : {1u, 2u})
    {
        bytes = Bytes(); bytes.cls(1, "Base"); bytes.cls(2, "Mid", 1); bytes.cls(3, "Leaf", 2);
        bytes.attr(1, "VALUE", 3); bytes.attr(2, "value", ancestor);
        rejected(Error::Code::Invalid);
        EXPECT_EQ("Ambiguous attribute name", error.message);
    }
}
TEST_F(PvdDocument, RepeatedFixtureLoads)
{
    for (unsigned i = 0; i < 4; ++i)
    {
        ASSERT_TRUE(document.load(PVDDOM_TEST_DATA_DIR "/test_scene.ovd", limits, error)) << error.message;
        ASSERT_NE(nullptr, document.dom());
        EXPECT_GT(document.dom()->mSceneCreations.size(), 0u);
    }
}
TEST_F(PvdDocument, MessagesStayLocal)
{
    const size_t initial = gOmniPvdMessages.size();
    bytes.command(OmniPvdCommand::eRECORD_MESSAGE); bytes.put(uint64_t(1));
    bytes.name("hello"); bytes.name("source.cpp"); bytes.put(uint32_t(12));
    bytes.put(uint32_t(1)); bytes.put(uint32_t(0));
    ASSERT_TRUE(load()) << error.message;
    ASSERT_EQ(1u, document.messages().size());
    EXPECT_STREQ("hello", document.messages()[0].message);
    EXPECT_EQ(initial, gOmniPvdMessages.size());
}
TEST_F(PvdDocument, SdkFlagsUseOneTwoOrFourBytes)
{
    bytes.cls(1, "Flags"); bytes.attr(1, "first", 1, OmniPvdDataType::eENUM_VALUE, 1);
    bytes.cls(2, "PxMaterial"); bytes.attr(2, "flags", 2, OmniPvdDataType::eFLAGS_WORD, 1);
    bytes.create(10, 2); bytes.set(1, 2); bytes.set(2, 2); bytes.set(4, 2);
    ASSERT_TRUE(load()) << error.message;
}
TEST_F(PvdDocument, RejectsThreeByteFlagsPayload)
{
    bytes.cls(1, "Flags"); bytes.attr(1, "first", 1, OmniPvdDataType::eENUM_VALUE, 1);
    bytes.cls(2, "PxMaterial"); bytes.attr(2, "flags", 2, OmniPvdDataType::eFLAGS_WORD, 1);
    bytes.create(10, 2); bytes.set(3, 2);
    rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, RejectsSanitizedAttributeNameCollision)
{
    bytes.cls(); bytes.attr(1, "a-b"); bytes.attr(2, "a_b"); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, RejectsScalarHandlerAsList)
{
    bytes.cls(1, "PxShape"); bytes.list(1, "geom", 1); bytes.create(); bytes.reference(10, 1, 0, true);
    rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, RejectsWrongTargetClass)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1); bytes.create(); bytes.create(11);
    bytes.reference(10, 1, 11, true); rejected(Error::Code::Invalid);
    EXPECT_EQ("Incompatible object edge target", error.message);
}
TEST_F(PvdDocument, RejectsSpecialClassWithForgedTargetAncestry)
{
    bytes.cls(1, "PxArticulationLink"); bytes.cls(2, "PxArticulationJointReducedCoordinate", 1);
    bytes.attr(1, "parentLink", 2, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(20, 2); bytes.reference(20, 1, 20); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, RejectsArticulationCycle)
{
    bytes.cls(1, "PxArticulationLink"); bytes.cls(2, "PxArticulationJointReducedCoordinate");
    bytes.attr(1, "parentLink", 2, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.attr(2, "childLink", 2, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(10); bytes.create(11); bytes.create(20, 2); bytes.create(21, 2);
    bytes.reference(20, 1, 10); bytes.reference(20, 2, 11);
    bytes.reference(21, 1, 11); bytes.reference(21, 2, 10);
    rejected(Error::Code::Invalid);
}
TEST(PvdHierarchy, CombinedArticulationReparentChecksProspectiveAncestry)
{
    OmniPvdObject joint, parent, child;
    parent.mAncestor = &joint;
    EXPECT_TRUE(detail::canParent(&parent, &child, 3));
    EXPECT_TRUE(detail::canParent(&child, &joint, 3));
    EXPECT_FALSE(detail::canParentArticulation(&parent, &child, &joint, 3));
    EXPECT_EQ(&joint, parent.mAncestor);
    EXPECT_EQ(nullptr, child.mAncestor);
    EXPECT_TRUE(detail::canParentArticulation(nullptr, &child, &joint, 3));
    parent.mAncestor = nullptr;
    child.mAncestor = &joint;
    EXPECT_FALSE(detail::canParentArticulation(nullptr, &child, &joint, 3));
    EXPECT_TRUE(detail::canParentArticulation(&parent, &child, &joint, 3));
    child.mAncestor = nullptr;
}
TEST_F(PvdDocument, RejectsCachedParentWithRecreatedIncompatibleClass)
{
    for (const char* replacement : {"PxShape", "PxArticulationJointReducedCoordinate"})
    for (bool inbound : {false, true})
    {
        SCOPED_TRACE(replacement);
        SCOPED_TRACE(inbound); bytes = Bytes();
        bytes.cls(1, "PxArticulationLink"); bytes.cls(2, "PxArticulationJointReducedCoordinate");
        bytes.cls(3, replacement);
        bytes.attr(1, "parentLink", 2, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.attr(2, "childLink", 2, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.attr(3, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
        bytes.create(10); bytes.create(20, 2); bytes.reference(20, 1, 10);
        bytes.create(10, 3); bytes.create(30);
        if (inbound) bytes.reference(30, 3, 20);
        else bytes.reference(20, 2, 30);
        rejected(Error::Code::Invalid);
    }
}
TEST_F(PvdDocument, CachedParentAfterChildDoesNotMutateHierarchy)
{
    bytes.cls(1, "PxArticulationLink"); bytes.cls(2, "PxArticulationJointReducedCoordinate");
    bytes.attr(1, "parentLink", 2, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.attr(2, "childLink", 2, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.attr(3, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(10); bytes.create(11); bytes.create(20, 2); bytes.create(21, 2);
    bytes.reference(20, 1, 10); bytes.reference(20, 2, 11);
    bytes.reference(21, 2, 10); bytes.reference(21, 1, 11);
    ASSERT_TRUE(load()) << error.message;
    const auto* dom = document.dom();
    auto* root = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(10));
    auto* child = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(11));
    auto* joint = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(21));
    EXPECT_EQ(nullptr, root->mAncestor);
    EXPECT_EQ(root, child->mAncestor);
    EXPECT_EQ(root, joint->mAncestor);
    const Bytes cached = bytes;
    bytes.reference(21, 2, 10);
    rejected(Error::Code::Invalid);
    bytes = cached;
    bytes.reference(10, 3, 21);
    rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, ArticulationReferencesMayPrecedeJointCreation)
{
    bytes.cls(1, "PxArticulationLink"); bytes.cls(2, "PxArticulationJointReducedCoordinate");
    bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.attr(2, "childLink", 2, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(10); bytes.reference(10, 1, 20); bytes.create(20, 2); bytes.reference(20, 2, 10);
    ASSERT_TRUE(load()) << error.message;
    const auto* dom = document.dom();
    auto* link = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(10));
    auto* joint = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(20));
    EXPECT_EQ(link, joint->mAncestor);
}
TEST_F(PvdDocument, UnresolvedForwardReferenceFails)
{
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(10); bytes.reference(10, 1, 20); rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, UnresolvedArticulationReferenceFails)
{
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "articulation", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(10); bytes.reference(10, 1, 20); rejected(Error::Code::Invalid);
    EXPECT_EQ("Unresolved forward reference at end of file", error.message);
    EXPECT_EQ(4u, error.commandIndex);
}
TEST_F(PvdDocument, OptionalChildLinkValidatesCreateWithoutReplayingParenting)
{
    bytes.cls(1, "PxArticulationJointReducedCoordinate");
    bytes.attr(1, "childLink", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.cls(2, "PxArticulationLink"); bytes.create(); bytes.reference(10, 1, 20); bytes.create(20, 2);
    ASSERT_TRUE(load()) << error.message;
    const auto* dom = document.dom();
    auto* joint = dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(10));
    EXPECT_EQ(nullptr, joint->mAncestor);
    EXPECT_EQ(0u, document.stats().unresolvedOptionalEdges);
    bytes = Bytes(); bytes.cls(1, "PxArticulationJointReducedCoordinate");
    bytes.attr(1, "childLink", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.cls(2, "PxScene"); bytes.create(); bytes.reference(10, 1, 20); bytes.create(20, 2);
    rejected(Error::Code::Invalid);
    EXPECT_EQ("Incompatible forward-reference target", error.message);
}
TEST_F(PvdDocument, MissingOptionalEdgesKeepSamplesAndRemainBounded)
{
    bytes.cls(1, "PxHeightFieldGeometry");
    bytes.attr(1, "heightField", 1, OmniPvdDataType::eOBJECT_HANDLE); bytes.create();
    ASSERT_TRUE(load()) << error.message;
    const uint64_t base = document.stats().accountedRetainedBytes;
    bytes.reference(10, 1, 20); bytes.reference(10, 1, 20);
    limits.maxRetainedBytes = base + 2 * (192 + 272 + 10240 + 2 * sizeof(SampleWrite) +
        2 * sizeof(size_t)) + 128 + 192 - 1;
    rejected(Error::Code::Limit);
    ++limits.maxRetainedBytes; ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(2u, document.stats().samples);
    EXPECT_EQ(2u, document.stats().unresolvedOptionalEdges);
    auto* object = document.dom()->mObjectHandleToObjectMap.at(document.dom()->mExternalToInternalHandleMap.at(10));
    const auto* sample = static_cast<OmniPvdAttributeSample*>(object->mInheritedClassInstances[0].mClassAttributeLists[0]->mFirst);
    uint64_t target = 0; memcpy(&target, sample->mData, 8); EXPECT_EQ(20u, target);
}
TEST_F(PvdDocument, MissingAddValidatesButMissingRemoveDoesNotObligateFutureCreate)
{
    for (bool remove : {false, true})
    {
        bytes = Bytes(); bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1); bytes.create();
        if (remove) bytes.removeReference(10, 1, 20); else bytes.reference(10, 1, 20, true);
        bytes.create(20);
        if (remove) ASSERT_TRUE(load()) << error.message;
        else { rejected(Error::Code::Invalid); EXPECT_EQ("Incompatible forward-reference target", error.message); }
    }
}
TEST_F(PvdDocument, RequiredEdgesSurviveOwnerDestructionAndSegments)
{
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.cls(2, "PxOmniPvdMetaData"); bytes.create(30, 2); bytes.create(); bytes.reference(10, 1, 20);
    bytes.command(OmniPvdCommand::eDESTROY_OBJECT); bytes.put(uint64_t(1)); bytes.put(uint64_t(10));
    bytes.create(31, 2); rejected(Error::Code::Invalid);
    EXPECT_EQ(6u, error.commandIndex);
}
TEST_F(PvdDocument, OptionalAddRemainsANoopWhenTargetAppears)
{
    bytes.cls(1, "PxRigidDynamic"); bytes.list(1, "shapes", 1);
    bytes.cls(2, "PxShape"); bytes.create(); bytes.reference(10, 1, 20, true);
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(1u, document.stats().unresolvedOptionalEdges);
    bytes.create(20, 2); ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(0u, document.stats().unresolvedOptionalEdges);
    const auto* dom = document.dom();
    EXPECT_EQ(nullptr, dom->mObjectHandleToObjectMap.at(dom->mExternalToInternalHandleMap.at(20))->mAncestor);
}
TEST_F(PvdDocument, OptionalEdgeDoesNotHideEarliestRequiredLocation)
{
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.list(2, "shapes", 1); bytes.create(); bytes.reference(10, 2, 5, true);
    const uint64_t requiredOffset = bytes.data.size();
    bytes.reference(10, 1, 30); bytes.reference(10, 1, 20);
    rejected(Error::Code::Invalid);
    EXPECT_EQ(6u, error.commandIndex); EXPECT_EQ(requiredOffset, error.byteOffset);
}
TEST_F(PvdDocument, ForwardReferenceRejectsWrongClassWhenCreated)
{
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.cls(2, "PxScene"); bytes.create(10); bytes.reference(10, 1, 20); bytes.create(20, 2);
    rejected(Error::Code::Invalid);
    EXPECT_EQ("Incompatible forward-reference target", error.message);
}
TEST_F(PvdDocument, NullUniqueListEdgeIsInvalid)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "actors", 1); bytes.create();
    bytes.reference(10, 1, 0, true);
    rejected(Error::Code::Invalid);
    EXPECT_EQ("Null unique-list edge", error.message);
}
TEST_F(PvdDocument, StopFrameWithoutStartIsInvalid)
{
    bytes.frame(1, true);
    rejected(Error::Code::Invalid);
    EXPECT_EQ("Frame stop without matching start", error.message);
}
TEST_F(PvdDocument, UnresolvedForwardReferenceReportsEarliestCommand)
{
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(10); bytes.reference(10, 1, 30); bytes.reference(10, 1, 20);
    rejected(Error::Code::Invalid);
    EXPECT_EQ(4u, error.commandIndex);
    EXPECT_EQ(93u, error.byteOffset);
}
TEST_F(PvdDocument, ConstraintHandlesAreOpaqueCorrelationIds)
{
    bytes.cls(1, "PxScene"); bytes.list(1, "constraints", 1);
    bytes.cls(2, "PxFixedJoint"); bytes.attr(2, "constraint", 2, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(10); bytes.create(20, 2); bytes.reference(10, 1, 1000, true); bytes.reference(20, 2, 1000);
    ASSERT_TRUE(load()) << error.message;
    EXPECT_EQ(1u, document.dom()->mConstraintToJointMap.size());
}
TEST_F(PvdDocument, RejectsInvalidShapeBoolean)
{
    bytes.cls(1, "PxShape"); bytes.attr(1, "isExclusive", 1, OmniPvdDataType::eUINT8);
    bytes.create(); bytes.set(1); bytes.data.back() = 2; rejected(Error::Code::Invalid);
}
TEST_F(PvdDocument, RetainedSamplesAndMessagesAreBounded)
{
    schema(); ASSERT_TRUE(load()) << error.message;
    limits.maxRetainedBytes = document.stats().accountedRetainedBytes + 64;
    bytes.set(); rejected(Error::Code::Limit);
    bytes = Bytes();
    ASSERT_TRUE(load()) << error.message;
    limits.maxRetainedBytes = document.stats().accountedRetainedBytes + 64;
    bytes.command(OmniPvdCommand::eRECORD_MESSAGE); bytes.put(uint64_t(1));
    bytes.name("x"); bytes.name(""); bytes.put(uint32_t(0)); bytes.put(uint32_t(0)); bytes.put(uint32_t(0));
    rejected(Error::Code::Limit);
}
TEST_F(PvdDocument, RecordingSegmentsKeepFrameIdentity)
{
    bytes.cls(1, "PxOmniPvdMetaData"); bytes.create(); bytes.frame(1); bytes.frame(1, true);
    bytes.create(11); bytes.frame(1); bytes.frame(1, true);
    ASSERT_TRUE(load()) << error.message;
    ASSERT_EQ(2u, document.frames().size());
    EXPECT_EQ(0u, document.frames()[0].recordingSegment);
    EXPECT_EQ(1u, document.frames()[1].recordingSegment);
    EXPECT_EQ(1u, document.frames()[1].recordedFrame);
}
TEST_F(PvdDocument, MissingPathIsIoError)
{
    EXPECT_FALSE(document.load("__missing_directory__/capture.ovd", limits, error));
    EXPECT_EQ(Error::Code::Io, error.code);
    EXPECT_EQ(nullptr, document.dom());
}
TEST_F(PvdDocument, UnsupportedStreamVersion)
{
    bytes.data[0] = 255; rejected(Error::Code::Unsupported);
}
TEST_F(PvdDocument, EveryTruncatedCommandPrefixFails)
{
    schema(); bytes.set(); bytes.frame(9);
    const auto full = bytes.data;
    // Every strict prefix of the final fixed-width frame command is invalid.
    for (size_t missing = 1; missing < 17; ++missing)
    {
        bytes.data.assign(full.begin(), full.end() - missing);
        rejected(Error::Code::Invalid);
    }
}
TEST_F(PvdDocument, RecoveryKeepsOnlyCompleteCommandsAcrossTailKinds)
{
    schema();
    bytes.frame(7, false, 1);
    bytes.frame(8, false, 2);
    const auto prefix = bytes.data;
    const uint64_t prefixCommands = 5;
    for (unsigned kind = 0; kind < 5; ++kind)
    {
        bytes.data = prefix;
        switch (kind)
        {
        case 0: bytes.set(); break;
        case 1: bytes.create(11); break;
        case 2: bytes.frame(9, false, 1); break;
        case 3: bytes.frame(7, true, 1); break;
        default:
            bytes.command(OmniPvdCommand::eRECORD_MESSAGE);
            bytes.put(uint64_t(1)); bytes.name("message"); bytes.name("source");
            bytes.put(uint32_t(4)); bytes.put(uint32_t(0)); bytes.put(uint32_t(0));
            break;
        }
        bytes.data.pop_back();
        const uint64_t sourceBytes = bytes.data.size();
        ASSERT_TRUE(load(LoadMode::RecoverTruncatedTail)) << kind << ": " << error.message;
        ASSERT_NE(nullptr, document.dom());
        EXPECT_EQ(Error::Code::None, error.code);
        EXPECT_TRUE(document.loadInfo().recoveredTail);
        EXPECT_EQ(sourceBytes, document.loadInfo().sourceBytes);
        EXPECT_EQ(Error::Code::Invalid, document.loadInfo().terminalError.code);
        EXPECT_EQ(prefix.size(), document.loadInfo().terminalError.byteOffset);
        EXPECT_EQ(prefixCommands + 1, document.loadInfo().terminalError.commandIndex);
        EXPECT_EQ(prefix.size(), document.stats().streamBytes);
        EXPECT_EQ(prefixCommands, document.stats().commands);
        ASSERT_EQ(2u, document.frames().size());
        EXPECT_FALSE(document.frames()[0].completed);
        EXPECT_FALSE(document.frames()[1].completed);
        EXPECT_EQ(0u, document.stats().samples);
        EXPECT_TRUE(document.messages().empty());
        EXPECT_EQ(1u, document.stats().creates);
        EXPECT_FALSE(load());
        EXPECT_EQ(nullptr, document.dom());
        EXPECT_FALSE(document.loadInfo().recoveredTail);
        EXPECT_EQ(0u, document.loadInfo().sourceBytes);
    }
}
TEST_F(PvdDocument, RecoveryRetainsCompletedFramesAndPostStopSamples)
{
    schema(); bytes.frame(1); bytes.frame(1, true); bytes.set();
    const uint64_t prefixBytes = bytes.data.size();
    bytes.frame(2); bytes.data.pop_back();
    ASSERT_TRUE(load(LoadMode::RecoverTruncatedTail)) << error.message;
    ASSERT_EQ(1u, document.frames().size());
    EXPECT_TRUE(document.frames()[0].completed);
    EXPECT_EQ(1u, document.stats().samples);
    EXPECT_EQ(prefixBytes, document.stats().streamBytes);
    bytes.data.resize(prefixBytes);
    ASSERT_TRUE(load(LoadMode::RecoverTruncatedTail)) << error.message;
    EXPECT_FALSE(document.loadInfo().recoveredTail);
    EXPECT_EQ(prefixBytes, document.loadInfo().sourceBytes);
    EXPECT_EQ(Error::Code::None, document.loadInfo().terminalError.code);
}
TEST_F(PvdDocument, RecoveryRejectsCutFirstCommandAndCorruption)
{
    bytes.cls(); bytes.data.pop_back();
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(nullptr, document.dom());
    EXPECT_EQ(0u, document.loadInfo().sourceBytes);
    bytes = Bytes(); schema(); bytes.put(uint8_t(255));
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(Error::Code::Invalid, error.code);
    EXPECT_EQ(nullptr, document.dom());
    bytes = Bytes(); schema(); bytes.frame(2, true);
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(Error::Code::Invalid, error.code);
    EXPECT_EQ(nullptr, document.dom());
}
TEST_F(PvdDocument, RecoveryRejectsRequiredEdgesAndAllLimits)
{
    bytes.cls(1, "PxArticulationLink"); bytes.attr(1, "inboundJoint", 1, OmniPvdDataType::eOBJECT_HANDLE);
    bytes.create(10); bytes.reference(10, 1, 30); bytes.frame(1); bytes.data.pop_back();
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(Error::Code::Invalid, error.code);
    EXPECT_EQ(4u, error.commandIndex);
    bytes = Bytes(); schema(); bytes.set();
    limits.maxAttributeBytes = 3;
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(Error::Code::Limit, error.code);
    limits = LoadLimits(); limits.maxCommands = 3;
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(Error::Code::Limit, error.code);
    limits = LoadLimits(); limits.maxStreamBytes = bytes.data.size() - 1;
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(Error::Code::Limit, error.code);
    limits = LoadLimits(); limits.maxRetainedBytes = 1;
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(Error::Code::Limit, error.code);
    EXPECT_EQ(nullptr, document.dom());
}
TEST_F(PvdDocument, ThreeArgumentLoadRemainsStrictAfterRecovery)
{
    schema(); bytes.set(); bytes.data.pop_back();
    ASSERT_TRUE(load(LoadMode::RecoverTruncatedTail)) << error.message;
    EXPECT_TRUE(document.loadInfo().recoveredTail);
    EXPECT_FALSE(document.load(path.c_str(), limits, error));
    EXPECT_EQ(Error::Code::Invalid, error.code);
    EXPECT_EQ(nullptr, document.dom());
    EXPECT_TRUE(document.frames().empty());
    EXPECT_TRUE(document.messages().empty());
    EXPECT_EQ(0u, document.stats().commands);
    EXPECT_FALSE(document.loadInfo().recoveredTail);
    EXPECT_EQ(0u, document.loadInfo().sourceBytes);
}
TEST_F(PvdDocument, RecoveryAtInclusiveStreamCapCoversPartialAndEmptyFinalFields)
{
    schema(); bytes.frame(7, false, 1); bytes.frame(8, false, 2);
    const auto prefix = bytes.data;
    for (unsigned kind = 0; kind < 3; ++kind)
    {
        bytes.data = prefix;
        if (kind == 0) bytes.frame(9);
        else if (kind == 1) bytes.set();
        else
        {
            bytes.command(OmniPvdCommand::eCREATE_OBJECT); bytes.put(uint64_t(1));
            bytes.put(uint32_t(1)); bytes.put(uint64_t(11)); bytes.name("tail");
        }
        const auto full = bytes.data;
        const size_t finalFieldBytes = kind == 0 ? 8 : 4;
        for (size_t missing : {size_t(1), finalFieldBytes})
        for (uint64_t headroom : {uint64_t(0), uint64_t(64)})
        {
            SCOPED_TRACE(testing::Message() << kind << ":" << missing << ":" << headroom);
            bytes.data.assign(full.begin(), full.end() - missing);
            limits.maxStreamBytes = bytes.data.size() + headroom;
            ASSERT_TRUE(load(LoadMode::RecoverTruncatedTail)) << error.message;
            EXPECT_TRUE(document.loadInfo().recoveredTail);
            EXPECT_EQ(bytes.data.size(), document.loadInfo().sourceBytes);
            EXPECT_EQ(Error::Code::Invalid, document.loadInfo().terminalError.code);
            EXPECT_EQ(prefix.size(), document.loadInfo().terminalError.byteOffset);
            EXPECT_EQ(6u, document.loadInfo().terminalError.commandIndex);
            EXPECT_EQ(prefix.size(), document.stats().streamBytes);
            EXPECT_EQ(5u, document.stats().commands);
            EXPECT_EQ(1u, document.stats().creates);
            EXPECT_EQ(0u, document.stats().samples);
            ASSERT_EQ(2u, document.frames().size());
            EXPECT_FALSE(document.frames()[0].completed);
            EXPECT_FALSE(document.frames()[1].completed);
            EXPECT_TRUE(document.messages().empty());
            EXPECT_FALSE(load());
            EXPECT_EQ(Error::Code::Invalid, error.code);
        }
    }
}
TEST_F(PvdDocument, InclusiveStreamCapAcceptsFullFileAndRejectsLargerSource)
{
    schema(); bytes.set(); bytes.frame(1);
    for (LoadMode mode : {LoadMode::Strict, LoadMode::RecoverTruncatedTail})
    {
        limits.maxStreamBytes = bytes.data.size();
        ASSERT_TRUE(load(mode)) << error.message;
        EXPECT_FALSE(document.loadInfo().recoveredTail);
        EXPECT_EQ(bytes.data.size(), document.stats().streamBytes);
        --limits.maxStreamBytes;
        EXPECT_FALSE(load(mode));
        EXPECT_EQ(Error::Code::Limit, error.code);
        EXPECT_EQ(nullptr, document.dom());
        EXPECT_EQ(0u, document.loadInfo().sourceBytes);
    }
}
TEST_F(PvdDocument, AdjustableLimitsIdentifyTheirActualBoundaryAndClearOnRetry)
{
    schema(); bytes.cls(2, "Derived", 1); bytes.attr(2, "second", 2);
    bytes.create(11, 2); bytes.set(); bytes.frame(1); bytes.frame(1, true); bytes.frame(2);
    ASSERT_TRUE(load()) << error.message;
    const auto stats = document.stats();
    struct Boundary { uint64_t LoadLimits::* field; uint64_t exact; Error::LimitKind kind; };
    const Boundary boundaries[] = {
        {&LoadLimits::maxStreamBytes, bytes.data.size(), Error::LimitKind::StreamBytes},
        {&LoadLimits::maxCommands, stats.commands, Error::LimitKind::Commands},
        {&LoadLimits::maxCreates, 2, Error::LimitKind::Creates},
        {&LoadLimits::maxClasses, 2, Error::LimitKind::Classes},
        {&LoadLimits::maxAttributes, 2, Error::LimitKind::Attributes},
        {&LoadLimits::maxInheritanceDepth, 2, Error::LimitKind::InheritanceDepth},
        {&LoadLimits::maxAttributeBytes, 4, Error::LimitKind::AttributeBytes},
        {&LoadLimits::maxFrameRecords, 2, Error::LimitKind::FrameRecords},
        {&LoadLimits::maxHandlerSteps, stats.handlerSteps, Error::LimitKind::HandlerSteps},
        {&LoadLimits::maxRetainedBytes, stats.accountedRetainedBytes, Error::LimitKind::RetainedBytes}
    };
    for (const auto& boundary : boundaries)
    {
        SCOPED_TRACE(static_cast<unsigned>(boundary.kind));
        limits = LoadLimits(); limits.*boundary.field = boundary.exact - 1;
        rejected(Error::Code::Limit);
        EXPECT_EQ(boundary.kind, error.limitKind);
        limits.*boundary.field = boundary.exact;
        ASSERT_TRUE(load()) << error.message;
        EXPECT_EQ(Error::LimitKind::None, error.limitKind);
        EXPECT_EQ(Error::LimitKind::None, document.loadInfo().terminalError.limitKind);
    }
    bytes.put(uint8_t(255)); rejected(Error::Code::Invalid);
    EXPECT_EQ(Error::LimitKind::None, error.limitKind);
    EXPECT_FALSE(document.load("__missing_directory__/capture.ovd", limits, error));
    EXPECT_EQ(Error::Code::Io, error.code);
    EXPECT_EQ(Error::LimitKind::None, error.limitKind);
    bytes = Bytes(); bytes.data[0] = 255; rejected(Error::Code::Unsupported);
    EXPECT_EQ(Error::LimitKind::None, error.limitKind);
}
TEST_F(PvdDocument, PayloadLimitNamesSmallerUserCapAndAttributeOnTie)
{
    // Unknown targets still enforce the reader payload cap before any DOM sample charge.
    bytes.set(40001);
    for (LoadMode mode : {LoadMode::Strict, LoadMode::RecoverTruncatedTail})
    {
        limits = LoadLimits(); limits.maxAttributeBytes = 40000;
        EXPECT_FALSE(load(mode)); EXPECT_EQ(Error::Code::Limit, error.code);
        EXPECT_EQ(Error::LimitKind::AttributeBytes, error.limitKind);
        limits = LoadLimits(); limits.maxRetainedBytes = 40000;
        EXPECT_FALSE(load(mode)); EXPECT_EQ(Error::Code::Limit, error.code);
        EXPECT_EQ(Error::LimitKind::RetainedBytes, error.limitKind);
        limits.maxAttributeBytes = 40000;
        EXPECT_FALSE(load(mode)); EXPECT_EQ(Error::Code::Limit, error.code);
        EXPECT_EQ(Error::LimitKind::AttributeBytes, error.limitKind);
        ++limits.maxAttributeBytes;
        EXPECT_FALSE(load(mode)); EXPECT_EQ(Error::Code::Limit, error.code);
        EXPECT_EQ(Error::LimitKind::RetainedBytes, error.limitKind);
        ++limits.maxRetainedBytes;
        ASSERT_TRUE(load(mode)) << error.message;
        EXPECT_EQ(Error::LimitKind::None, error.limitKind);
        EXPECT_EQ(1u, document.stats().ignoredCommands);
    }
}
TEST_F(PvdDocument, ReaderPathLimitAfterPayloadHasNoAdjustableIdentity)
{
    bytes.set(64); const size_t prefix = bytes.data.size(); bytes.set(128, 1, 10, 33);
    limits.maxAttributeBytes = 64;
    rejected(Error::Code::Limit);
    EXPECT_EQ(2u, error.commandIndex);
    EXPECT_EQ(prefix, error.byteOffset);
    EXPECT_EQ(Error::LimitKind::None, error.limitKind);
    EXPECT_FALSE(load(LoadMode::RecoverTruncatedTail));
    EXPECT_EQ(Error::Code::Limit, error.code);
    EXPECT_EQ(Error::LimitKind::None, error.limitKind);
    EXPECT_EQ("Attribute path depth exceeds 32", error.message);
}

TEST_F(PvdDocument, OversizeTrailingPayloadIsVerifiedAndRecoverableAboveEitherCap)
{
    for (bool retained : {false, true})
    {
        bytes = Bytes(); schema(); bytes.frame(9);
        const auto prefix = bytes.data;
        bytes.set(3); bytes.data.resize(bytes.data.size() - 7); bytes.put(uint32_t(200000));
        for (unsigned i = 0; i < 10000; ++i) bytes.put(uint8_t(0));
        limits = LoadLimits(); limits.maxStreamBytes = bytes.data.size();
        if (retained) { limits.maxRetainedBytes = 100000; limits.maxAttributeBytes = 300000; }
        else limits.maxAttributeBytes = 40000;
        rejected(Error::Code::Invalid);
        EXPECT_EQ(Error::LimitKind::None, error.limitKind);
        EXPECT_EQ("Attribute payload extends past end of file", error.message);
        ASSERT_TRUE(load(LoadMode::RecoverTruncatedTail)) << error.message;
        EXPECT_TRUE(document.loadInfo().recoveredTail);
        EXPECT_EQ(prefix.size(), document.stats().streamBytes);
        EXPECT_EQ(4u, document.stats().commands);
        EXPECT_EQ(0u, document.stats().samples);
        EXPECT_EQ(Error::Code::Invalid, document.loadInfo().terminalError.code);
        EXPECT_EQ(Error::LimitKind::None, document.loadInfo().terminalError.limitKind);
    }
}

TEST_F(PvdDocument, OversizeTailOneBytePastEofIsRecoverable)
{
    schema(); bytes.set(4); bytes.data.pop_back();
    limits.maxAttributeBytes = 3;
    rejected(Error::Code::Invalid);
    EXPECT_EQ(Error::LimitKind::None, error.limitKind);
    EXPECT_EQ("Attribute payload extends past end of file", error.message);
    ASSERT_TRUE(load(LoadMode::RecoverTruncatedTail)) << error.message;
    const auto& info = document.loadInfo();
    EXPECT_TRUE(info.recoveredTail);
    EXPECT_EQ(Error::Code::Invalid, info.terminalError.code);
    EXPECT_EQ(Error::LimitKind::None, info.terminalError.limitKind);
    EXPECT_EQ("Attribute payload extends past end of file", info.terminalError.message);
    EXPECT_EQ(bytes.data.size(), info.sourceBytes);
    EXPECT_LT(document.stats().streamBytes, info.sourceBytes);
}
TEST_F(PvdDocument, CommandStopMessagesIdentifyAvailableReaderState)
{
    schema(); bytes.frame(1); bytes.data.pop_back(); rejected(Error::Code::Invalid);
    EXPECT_EQ("Command truncated at end of file", error.message);
    bytes = Bytes(); schema(); bytes.put(uint8_t(255)); rejected(Error::Code::Invalid);
    EXPECT_EQ("Invalid command or field", error.message);
    bytes = Bytes(); schema(); bytes.set(4, 1, 10, 0); rejected(Error::Code::Invalid);
    EXPECT_EQ("Invalid command or field", error.message);
    bytes = Bytes(); schema(); bytes.command(OmniPvdCommand::eREGISTER_ENUM); rejected(Error::Code::Unsupported);
    EXPECT_EQ("Unsupported command", error.message);
}
#ifdef _WIN32
TEST_F(PvdDocument, LockedResidualOfOversizeTailFailsIoInBothModes)
{
    schema(); bytes.set(0); bytes.data.resize(bytes.data.size() - 4); bytes.put(uint32_t(200000));
    bytes.data.insert(bytes.data.end(), 20000, uint8_t(0));
    limits.maxAttributeBytes = 40000;
    EXPECT_FALSE(load());
    struct Lock
    {
        HANDLE handle = INVALID_HANDLE_VALUE;
        OVERLAPPED range = {};
        bool locked = false;
        ~Lock() { if (locked) UnlockFileEx(handle, 0, 64, 0, &range);
            if (handle != INVALID_HANDLE_VALUE) CloseHandle(handle); }
    } lock;
    lock.handle = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                              nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    ASSERT_NE(INVALID_HANDLE_VALUE, lock.handle);
    lock.range.Offset = 16384;
    lock.locked = LockFileEx(lock.handle, LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY,
                            0, 64, 0, &lock.range) != 0;
    ASSERT_TRUE(lock.locked) << GetLastError();
    for (LoadMode mode : {LoadMode::Strict, LoadMode::RecoverTruncatedTail})
    {
        EXPECT_FALSE(document.load(path.c_str(), limits, error, mode));
        EXPECT_EQ(Error::Code::Io, error.code) << error.message;
        EXPECT_EQ(Error::LimitKind::None, error.limitKind);
        EXPECT_EQ("File read failed before the measured end of file", error.message);
        EXPECT_EQ(nullptr, document.dom()); EXPECT_TRUE(document.frames().empty());
        EXPECT_FALSE(document.loadInfo().recoveredTail);
    }
}

TEST_F(PvdDocument, LockedMiddleOfIntactRecordingFailsIoInBothModes)
{
    bytes = actorRecording(128);
    ASSERT_GT(bytes.data.size(), 73728u + 64u);
    ASSERT_TRUE(load()) << error.message;
    const uint64_t completeCommands = document.stats().commands;
    struct LockedRange
    {
        HANDLE handle = INVALID_HANDLE_VALUE;
        OVERLAPPED range = {};
        bool locked = false;
        ~LockedRange()
        {
            if (locked) UnlockFileEx(handle, 0, 64, 0, &range);
            if (handle != INVALID_HANDLE_VALUE) CloseHandle(handle);
        }
    } lock;
    lock.handle = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                              nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    ASSERT_NE(INVALID_HANDLE_VALUE, lock.handle);
    for (DWORD lockedOffset : {0u, 73728u})
    {
        lock.range.Offset = lockedOffset;
        lock.locked = LockFileEx(lock.handle, LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY,
                                 0, 64, 0, &lock.range) != 0;
        ASSERT_TRUE(lock.locked) << GetLastError();
        for (LoadMode mode : {LoadMode::Strict, LoadMode::RecoverTruncatedTail})
        {
            EXPECT_FALSE(document.load(path.c_str(), limits, error, mode));
            EXPECT_EQ(Error::Code::Io, error.code) << error.message;
            EXPECT_EQ(lockedOffset ? "File read failed before the measured end of file" :
                "File read failed while reading the stream header", error.message);
            EXPECT_EQ(Error::LimitKind::None, error.limitKind);
            if (lockedOffset)
            {
                EXPECT_GT(error.commandIndex, 1u);
                EXPECT_LT(error.byteOffset, lock.range.Offset);
            }
            else EXPECT_EQ(0u, error.commandIndex);
            EXPECT_EQ(nullptr, document.dom());
            EXPECT_TRUE(document.frames().empty());
            EXPECT_TRUE(document.messages().empty());
            const auto& stats = document.stats();
            EXPECT_EQ(0u, stats.streamBytes); EXPECT_EQ(0u, stats.commands);
            EXPECT_EQ(0u, stats.ignoredCommands); EXPECT_EQ(0u, stats.unresolvedOptionalEdges);
            EXPECT_EQ(0u, stats.creates); EXPECT_EQ(0u, stats.classes); EXPECT_EQ(0u, stats.attributes);
            EXPECT_EQ(0u, stats.samples); EXPECT_EQ(0u, stats.handlerSteps); EXPECT_EQ(0u, stats.accountedRetainedBytes);
            EXPECT_FALSE(document.loadInfo().recoveredTail);
            EXPECT_EQ(0u, document.loadInfo().sourceBytes);
            EXPECT_EQ(Error::Code::None, document.loadInfo().terminalError.code);
            EXPECT_EQ(Error::LimitKind::None, document.loadInfo().terminalError.limitKind);
            EXPECT_TRUE(document.loadInfo().terminalError.message.empty());
            EXPECT_EQ(0u, document.loadInfo().terminalError.byteOffset);
            EXPECT_EQ(0u, document.loadInfo().terminalError.commandIndex);
        }
        ASSERT_TRUE(UnlockFileEx(lock.handle, 0, 64, 0, &lock.range));
        lock.locked = false;
    }
    ASSERT_TRUE(document.load(path.c_str(), limits, error)) << error.message;
    EXPECT_EQ(completeCommands, document.stats().commands);
    EXPECT_EQ(bytes.data.size(), document.stats().streamBytes);
    EXPECT_FALSE(document.loadInfo().recoveredTail);
}
#endif
}
