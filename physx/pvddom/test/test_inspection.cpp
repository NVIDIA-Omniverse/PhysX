// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "PvdDomQuery.h"
#include "foundation/PxTransform.h"
#include "PvdDomInspection.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdWriter.h"
#include "OmniPvdCommands.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <locale>

using namespace PvdDomInspect;

namespace
{
struct Capture : OmniPvdWriteStream
{
    std::vector<uint8_t> bytes;
    OmniPvdWriter* writer = createOmniPvdWriter();
    OmniPvdClassHandle sceneClass, actorClass;
    OmniPvdAttributeHandle actors, velocity, pose, label;
    uint64_t scene, actor;
    Capture(uint64_t scene = 100, uint64_t actor = 200, bool startBeforeCreate = false,
            bool extraObject = false) : scene(scene), actor(actor)
    {
        writer->setWriteStream(*this);
        sceneClass = writer->registerClass("PxScene");
        actorClass = writer->registerClass("PxRigidDynamic");
        actors = writer->registerUniqueListAttribute(sceneClass, "actors", OmniPvdDataType::eOBJECT_HANDLE);
        velocity = writer->registerAttribute(actorClass, "linearVelocity", OmniPvdDataType::eFLOAT32, 3);
        pose = writer->registerAttribute(actorClass, "globalPose", OmniPvdDataType::eFLOAT32, 7);
        label = writer->registerAttribute(actorClass, "name", OmniPvdDataType::eSTRING, 0);
        if (startBeforeCreate) writer->startFrame(scene, 1);
        writer->createObject(1, sceneClass, scene, "scene");
        if (extraObject) create(actor + 1, "other body");
        create(actor, "body");
    }
    ~Capture() { destroyOmniPvdWriter(*writer); }
    uint64_t OMNI_PVD_CALL writeBytes(const uint8_t* data, uint64_t size) override
    { bytes.insert(bytes.end(), data, data + size); return size; }
    bool OMNI_PVD_CALL openStream() override { return true; }
    bool OMNI_PVD_CALL closeStream() override { return true; }
    bool OMNI_PVD_CALL flush() override { return true; }
    void create(uint64_t handle, const char* name) { writer->createObject(1, actorClass, handle, name); }
    void join(uint64_t handle = 0)
    {
        if (!handle) handle = actor;
        writer->addToUniqueListAttribute(1, scene, actors, reinterpret_cast<const uint8_t*>(&handle), 8);
    }
    void leave()
    { writer->removeFromUniqueListAttribute(1, scene, actors, reinterpret_cast<const uint8_t*>(&actor), 8); }
    void frame(uint64_t f) { writer->startFrame(scene, f); }
    void value(float x, float y = 2, float z = 3, uint64_t handle = 0)
    {
        float v[] = {x, y, z};
        writer->setAttribute(1, handle ? handle : actor, velocity, reinterpret_cast<const uint8_t*>(v), sizeof(v));
    }
    bool load(Document& document, Error::Code expected = Error::Code::None, LoadMode mode = LoadMode::Strict)
    {
        static unsigned serial = 0;
        const std::string path = std::string("inspection-") + testing::UnitTest::GetInstance()->current_test_info()->name() +
            "-" + std::to_string(++serial) + ".ovd";
        { std::ofstream file(path, std::ios::binary); file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size()); }
        Error error;
        const bool ok = document.load(path.c_str(), LoadLimits(), error, mode);
        std::remove(path.c_str());
        EXPECT_EQ(expected, error.code) << error.message;
        return ok;
    }
};
ObjectSelector body() { ObjectSelector s; s.name = "body"; return s; }
FrameRef frame(uint64_t n, uint64_t context = 100, uint64_t segment = 0)
{ FrameRef f; f.recordedFrame = n; f.context = context; f.recordingSegment = segment; return f; }
ComparisonOptions options()
{
    ComparisonOptions o;
    o.leftObject = body(); o.rightObject = body(); o.leftContext = 100; o.rightContext = 900;
    o.startFrame = 1; o.endFrame = 3; o.tolerance = 0.1;
    return o;
}
std::string leftSampleJson(const ComparisonResult& result)
{
    const std::string json = evidenceToJson(result, {}, {});
    const size_t begin = json.find("\"leftSample\":");
    const size_t end = json.find(",\"rightSample\":", begin);
    return begin == std::string::npos || end == std::string::npos ? std::string() : json.substr(begin, end - begin);
}
}

TEST(PvdInspection, ReadsActualSdkTransformLayout)
{
    const physx::PxTransform pose(physx::PxVec3(1, 2, 3), physx::PxQuat(0, 0, 0.6f, 0.8f));
    OmniPvdAttributeSample sample;
    sample.assignData(reinterpret_cast<const uint8_t*>(&pose), sizeof(pose));
    float p[3], q[4];
    ASSERT_TRUE(readTransform(&sample, p, q));
    EXPECT_FLOAT_EQ(1, p[0]); EXPECT_FLOAT_EQ(2, p[1]); EXPECT_FLOAT_EQ(3, p[2]);
    EXPECT_FLOAT_EQ(0, q[0]); EXPECT_FLOAT_EQ(0, q[1]);
    EXPECT_FLOAT_EQ(0.6f, q[2]); EXPECT_FLOAT_EQ(0.8f, q[3]);
}

TEST(PvdInspection, NormalizesQuaternionsAndRejectsInvalidInputs)
{
    const double a[] = {0, 0, 0, 1}, opposite[] = {0, 0, 0, -2};
    const double huge[] = {0, 0, 0, (std::numeric_limits<double>::max)()};
    const double quarter[] = {0, 0, std::sqrt(0.5), std::sqrt(0.5)};
    const double tiny[] = {0, 0, std::sin(5e-9), std::cos(5e-9)};
    double distance = 123;
    ASSERT_TRUE(quaternionAngularDistance(a, opposite, distance)); EXPECT_NEAR(0, distance, 1e-12);
    ASSERT_TRUE(quaternionAngularDistance(a, huge, distance)); EXPECT_NEAR(0, distance, 1e-12);
    ASSERT_TRUE(quaternionAngularDistance(a, quarter, distance)); EXPECT_NEAR(1.5707963267948966, distance, 1e-12);
    ASSERT_TRUE(quaternionAngularDistance(a, tiny, distance)); EXPECT_NEAR(1e-8, distance, 1e-20);
    const double rotated[] = {0, 0, 0.6, 0.8}, scaled[] = {0, 0, -6, -8};
    ASSERT_TRUE(quaternionAngularDistance(rotated, scaled, distance)); EXPECT_NEAR(0, distance, 1e-15);
    for (double invalid : {0.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        const double q[] = {0, 0, 0, invalid};
        EXPECT_FALSE(quaternionAngularDistance(a, q, distance));
    }
}

TEST(PvdInspection, SceneWritesUseMembershipAndExposeExactCarriedMissing)
{
    Capture c; c.frame(1); c.join(); c.frame(2); c.value(1); c.frame(3);
    Document d; ASSERT_TRUE(c.load(d));
    ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(1), id, error)) << error;
    SampleResult sample;
    ASSERT_TRUE(readSample(d, id.objectId, frame(1), "linearVelocity", sample, error)) << error;
    EXPECT_EQ(SampleState::Missing, sample.state); EXPECT_EQ(nullptr, sample.sample);
    ComparisonResult missingEvidence; missingEvidence.leftSample = sample;
    EXPECT_NE(std::string::npos, leftSampleJson(missingEvidence).find("\"source\":null"));
    ASSERT_TRUE(readSample(d, id.objectId, frame(2), "linearVelocity", sample, error)) << error;
    EXPECT_EQ(SampleState::Exact, sample.state); EXPECT_EQ(1u, sample.rawContext);
    EXPECT_EQ(100u, sample.source.context); EXPECT_EQ(2u, sample.source.recordedFrame);
    ComparisonResult exactEvidence; exactEvidence.leftSample = sample;
    EXPECT_NE(std::string::npos, leftSampleJson(exactEvidence).find(
        "\"source\":{\"recordingSegment\":\"0\",\"context\":\"100\",\"recordedFrame\":\"2\"}"));
    double v[3]; ASSERT_TRUE(PvdDomInspect::readVec3(sample, v)); EXPECT_EQ(1, v[0]); EXPECT_EQ(3, v[2]);
    ASSERT_TRUE(readSample(d, id.objectId, frame(3), "linearVelocity", sample, error)) << error;
    EXPECT_EQ(SampleState::Carried, sample.state); EXPECT_EQ(1u, sample.age);
    EXPECT_FALSE(selectObject(d, body(), frame(4), id, error));
    EXPECT_FALSE(selectObject(d, body(), frame(2, 1), id, error));
    EXPECT_FALSE(readSample(d, id.objectId, frame(2), "unknown", sample, error));
}

TEST(PvdInspection, ScopedWritesStayPerObject)
{
    Capture c(100, 200, false, true);
    c.frame(1); c.join(); c.join(201); c.value(1);
    for (uint64_t f = 2; f <= 4; ++f) { c.frame(f); c.value(float(f)); }
    c.value(9, 2, 3, 201); c.value(10, 2, 3, 201);
    Document d; ASSERT_TRUE(c.load(d));
    ObjectIdentity id; std::string error; SampleResult sample; double values[3];
    ASSERT_TRUE(selectObject(d, body(), frame(4), id, error)) << error;
    ASSERT_TRUE(readSample(d, id.objectId, frame(4), "linearVelocity", sample, error)) << error;
    ASSERT_EQ(SampleState::Exact, sample.state);
    ASSERT_TRUE(readVec3(sample, values)); ASSERT_EQ(4, values[0]);
    ObjectSelector other; other.name = "other body";
    ASSERT_TRUE(selectObject(d, other, frame(4), id, error)) << error;
    ASSERT_TRUE(readSample(d, id.objectId, frame(4), "linearVelocity", sample, error)) << error;
    ASSERT_EQ(SampleState::Exact, sample.state);
    ASSERT_TRUE(readVec3(sample, values)); EXPECT_EQ(10, values[0]);
}

TEST(PvdInspection, PreMembershipSnapshotNeverBecomesExact)
{
    Capture c; c.frame(1); c.value(7); c.join(); c.frame(2);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(1), id, error)) << error;
    SampleResult sample; ASSERT_TRUE(readSample(d, id.objectId, frame(1), "linearVelocity", sample, error)) << error;
    EXPECT_EQ(SampleState::Carried, sample.state); EXPECT_TRUE(sample.preMembership);
    EXPECT_EQ(0u, sample.source.context); EXPECT_EQ(1u, sample.sampleTimestamp);
    EXPECT_FALSE(sample.hasSourceFrame);
    ComparisonResult evidence; evidence.leftSample = sample;
    EXPECT_NE(std::string::npos, leftSampleJson(evidence).find("\"source\":null"));
}

TEST(PvdInspection, HistoricalMembershipAndHandleReuseKeepIncarnationsSeparate)
{
    Capture c; c.frame(1); c.join(); c.value(1);
    c.frame(2); c.leave(); c.writer->destroyObject(1, c.actor);
    c.frame(3); c.create(c.actor, "body"); c.join(); c.value(9);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity oldId, newId; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(1), oldId, error)) << error;
    EXPECT_FALSE(selectObject(d, body(), frame(2), newId, error));
    ASSERT_TRUE(selectObject(d, body(), frame(3), newId, error)) << error;
    EXPECT_NE(oldId.objectId, newId.objectId); EXPECT_EQ(oldId.externalHandle, newId.externalHandle);
    EXPECT_TRUE(oldId.hasLifetimeEnd); EXPECT_EQ(2u, oldId.lifetimeEnd);
    SampleResult sample;
    ASSERT_TRUE(readSample(d, oldId.objectId, frame(1), "linearVelocity", sample, error)) << error;
    double v[3]; ASSERT_TRUE(PvdDomInspect::readVec3(sample, v)); EXPECT_EQ(1, v[0]);
    EXPECT_FALSE(readSample(d, oldId.objectId, frame(3), "linearVelocity", sample, error));
}

TEST(PvdInspection, DuplicateNamesAreAmbiguousAndDiscoveryIsPaged)
{
    Capture c; c.frame(1); c.join(); c.create(201, "body"); c.join(201);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    EXPECT_FALSE(selectObject(d, body(), frame(1), id, error)); EXPECT_FALSE(error.empty());
    std::vector<ObjectIdentity> all, page;
    ASSERT_TRUE(listObjects(d, frame(1), 0, 10, all, error)); ASSERT_EQ(3u, all.size());
    ASSERT_TRUE(listObjects(d, frame(1), 1, 1, page, error)); ASSERT_EQ(1u, page.size());
    EXPECT_EQ(all[1].objectId, page[0].objectId);
    ObjectSelector s; s.objectId = page[0].objectId;
    ASSERT_TRUE(selectObject(d, s, frame(1), id, error)); EXPECT_EQ(s.objectId, id.objectId);
}

TEST(PvdInspection, EqualIndependentCapturesAndFirstDifferentEvidence)
{
    Capture a, b(900, 987); a.join(); b.join();
    for (int i = 1; i <= 3; ++i) { a.frame(i); a.value(float(i)); b.frame(i); b.value(float(i)); }
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    ComparisonResult r = compareLinearVelocity(left, right, options());
    EXPECT_EQ(ComparisonStatus::Equal, r.status) << r.reason; EXPECT_EQ(3u, r.comparedFrames);
    b.value(4); ASSERT_TRUE(b.load(right)); r = compareLinearVelocity(left, right, options());
    ASSERT_EQ(ComparisonStatus::Different, r.status) << r.reason; EXPECT_TRUE(r.hasEvidence);
    EXPECT_EQ(3u, r.leftSample.requested.recordedFrame); EXPECT_DOUBLE_EQ(1, r.distance);
    EXPECT_EQ(3u, r.componentCount);
    EXPECT_EQ(3, r.leftValue[0]); EXPECT_EQ(4, r.rightValue[0]);
    EXPECT_NE(r.leftSample.identity.externalHandle, r.rightSample.identity.externalHandle);
    auto inclusive = options(); inclusive.tolerance = 1;
    EXPECT_EQ(ComparisonStatus::Equal, compareLinearVelocity(left, right, inclusive).status);
    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        inclusive.tolerance = invalid;
        EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(left, right, inclusive).status);
    }
    CaptureProvenance lp; lp.sourcePath = "left\"capture\n.ovd";
    const auto json = evidenceToJson(r, lp, CaptureProvenance());
    EXPECT_NE(std::string::npos, json.find("left\\\"capture\\n.ovd"));
    EXPECT_NE(std::string::npos, json.find("\"version\":1"));
    EXPECT_NE(std::string::npos, json.find("\"recordedFrame\":\"3\""));
    EXPECT_NE(std::string::npos, json.find("scene_length_units/s"));
}

TEST(PvdInspection, GenericVec3UsesSelectedAttributeAndFloatWidths)
{
    for (bool wide : {false, true})
    {
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
        const auto type = wide ? OmniPvdDataType::eFLOAT64 : OmniPvdDataType::eFLOAT32;
        const auto leftAttribute = a.writer->registerAttribute(a.actorClass, "customVector", type, 3);
        const auto rightAttribute = b.writer->registerAttribute(b.actorClass, "customVector", type, 3);
        if (wide)
        {
            const double lhs[3] = {1, 2, 3}, rhs[3] = {4, 6, 3};
            a.writer->setAttribute(1, a.actor, leftAttribute, reinterpret_cast<const uint8_t*>(lhs), sizeof(lhs));
            b.writer->setAttribute(1, b.actor, rightAttribute, reinterpret_cast<const uint8_t*>(rhs), sizeof(rhs));
        }
        else
        {
            const float lhs[3] = {1, 2, 3}, rhs[3] = {4, 6, 3};
            a.writer->setAttribute(1, a.actor, leftAttribute, reinterpret_cast<const uint8_t*>(lhs), sizeof(lhs));
            b.writer->setAttribute(1, b.actor, rightAttribute, reinterpret_cast<const uint8_t*>(rhs), sizeof(rhs));
        }
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1; o.tolerance = 4.99; o.units = "wrong-default";
        ComparisonMeasure measure; measure.attributeName = "customVector";
        measure.distance = DistanceKind::Vec3Euclidean; measure.units = "declared_length";
        auto result = compareAttribute(left, right, o, measure);
        ASSERT_EQ(ComparisonStatus::Different, result.status) << result.reason;
        EXPECT_DOUBLE_EQ(5, result.distance);
        EXPECT_EQ(3u, result.componentCount);
        EXPECT_DOUBLE_EQ(1, result.leftValue[0]); EXPECT_DOUBLE_EQ(4, result.rightValue[0]);
        EXPECT_EQ("declared_length", result.options.units);
        EXPECT_EQ("customVector", result.measure.attributeName);
        const auto json = attributeEvidenceToJson(result, {}, {});
        EXPECT_NE(std::string::npos, json.find("\"version\":2"));
        EXPECT_NE(std::string::npos, json.find("\"attributeName\":\"customVector\""));
        EXPECT_NE(std::string::npos, json.find("\"distanceKind\":\"Vec3Euclidean\""));
        EXPECT_NE(std::string::npos, json.find("\"componentCount\":3"));
        EXPECT_NE(std::string::npos, json.find("\"leftValue\":[1,2,3]"));
        EXPECT_EQ(std::string::npos, json.find("\"leftVelocity\""));
        const auto legacy = evidenceToJson(result, {}, {});
        EXPECT_NE(std::string::npos, legacy.find("\"distance\":5"));
        EXPECT_NE(std::string::npos, legacy.find("\"leftVelocity\":null,\"rightVelocity\":null"));
        o.tolerance = 5;
        EXPECT_EQ(ComparisonStatus::Equal, compareAttribute(left, right, o, measure).status);
    }
}

TEST(PvdInspection, GenericTransformPositionAndOrientationUseRawSdkLayout)
{
    for (bool wide : {false, true})
    {
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
        const auto type = wide ? OmniPvdDataType::eFLOAT64 : OmniPvdDataType::eFLOAT32;
        const auto lhsAttr = wide ? a.writer->registerAttribute(a.actorClass, "poseForComparison", type, 7) : a.pose;
        const auto rhsAttr = wide ? b.writer->registerAttribute(b.actorClass, "poseForComparison", type, 7) : b.pose;
        if (wide)
        {
            const double lhs[7] = {0, 0, 0, 1, 1, 2, 3};
            const double rhs[7] = {0, 0, std::sqrt(0.5), std::sqrt(0.5), 4, 6, 3};
            a.writer->setAttribute(1, a.actor, lhsAttr, reinterpret_cast<const uint8_t*>(lhs), sizeof(lhs));
            b.writer->setAttribute(1, b.actor, rhsAttr, reinterpret_cast<const uint8_t*>(rhs), sizeof(rhs));
        }
        else
        {
            const float lhs[7] = {0, 0, 0, 1, 1, 2, 3};
            const float rhs[7] = {0, 0, float(std::sqrt(0.5)), float(std::sqrt(0.5)), 4, 6, 3};
            a.writer->setAttribute(1, a.actor, lhsAttr, reinterpret_cast<const uint8_t*>(lhs), sizeof(lhs));
            b.writer->setAttribute(1, b.actor, rhsAttr, reinterpret_cast<const uint8_t*>(rhs), sizeof(rhs));
        }
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1; o.tolerance = 4.99;
        ComparisonMeasure measure; measure.attributeName = wide ? "poseForComparison" : "globalPose";
        measure.distance = DistanceKind::TransformPositionEuclidean; measure.units = "cm";
        auto result = compareAttribute(left, right, o, measure);
        ASSERT_EQ(ComparisonStatus::Different, result.status) << result.reason;
        EXPECT_DOUBLE_EQ(5, result.distance); EXPECT_EQ(3u, result.componentCount);
        EXPECT_DOUBLE_EQ(1, result.leftValue[0]); EXPECT_DOUBLE_EQ(4, result.rightValue[0]);
        EXPECT_NE(std::string::npos, evidenceToJson(result, {}, {}).find("\"leftVelocity\":null,\"rightVelocity\":null"));
        measure.distance = DistanceKind::TransformOrientationRadians; measure.units = "radians";
        o.tolerance = 1;
        result = compareAttribute(left, right, o, measure);
        ASSERT_EQ(ComparisonStatus::Different, result.status) << result.reason;
        EXPECT_NEAR(1.5707963267948966, result.distance, 2e-7);
        EXPECT_EQ(4u, result.componentCount);
        EXPECT_DOUBLE_EQ(1, result.leftValue[3]);
        EXPECT_NEAR(std::sqrt(0.5), result.rightValue[2], 1e-7);
        EXPECT_NE(std::string::npos, attributeEvidenceToJson(result, {}, {}).find("\"componentCount\":4"));
        EXPECT_NE(std::string::npos, evidenceToJson(result, {}, {}).find("\"leftVelocity\":null,\"rightVelocity\":null"));
    }
}

TEST(PvdInspection, GenericOrientationTreatsOppositeQuaternionsAsEqualAndRejectsZero)
{
    Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
    const float lhs[7] = {1, 2, 3, 4, 1, 2, 3};
    const float opposite[7] = {-2, -4, -6, -8, 1, 2, 3};
    a.writer->setAttribute(1, a.actor, a.pose, reinterpret_cast<const uint8_t*>(lhs), sizeof(lhs));
    b.writer->setAttribute(1, b.actor, b.pose, reinterpret_cast<const uint8_t*>(opposite), sizeof(opposite));
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 1; o.tolerance = 0;
    ComparisonMeasure m; m.attributeName = "globalPose";
    m.distance = DistanceKind::TransformOrientationRadians; m.units = "radians";
    auto result = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Equal, result.status) << result.reason;
    EXPECT_NEAR(0, result.distance, 1e-15);
    EXPECT_DOUBLE_EQ(-8, result.rightValue[3]);
    const auto json = attributeEvidenceToJson(result, {}, {});
    EXPECT_NE(std::string::npos, json.find("\"leftValue\":[1,2,3,4],\"rightValue\":[-2,-4,-6,-8]"));
    const float zero[7] = {0, 0, 0, 0, 1, 2, 3};
    b.writer->setAttribute(1, b.actor, b.pose, reinterpret_cast<const uint8_t*>(zero), sizeof(zero));
    ASSERT_TRUE(b.load(right));
    EXPECT_EQ(ComparisonStatus::Incomparable, compareAttribute(left, right, o, m).status);
}

TEST(PvdInspection, LegacyEvidenceUsesDecodedGenericVelocityValues)
{
    Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1); a.value(1); b.value(4, 6, 3);
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 1;
    ComparisonMeasure m; m.attributeName = "linearVelocity"; m.units = o.units;
    const auto generic = compareAttribute(left, right, o, m);
    ASSERT_EQ(ComparisonStatus::Different, generic.status); EXPECT_DOUBLE_EQ(5, generic.distance);
    const auto json = evidenceToJson(generic, {}, {});
    EXPECT_NE(std::string::npos, json.find("\"leftVelocity\":[1,2,3],\"rightVelocity\":[4,6,3]"));
    EXPECT_NE(std::string::npos, json.find("\"distance\":5"));
    EXPECT_NE(std::string::npos, evidenceToJson(compareLinearVelocity(left, right, o), {}, {}).find(
        "\"leftVelocity\":[1,2,3],\"rightVelocity\":[4,6,3]"));
}

TEST(PvdInspection, InvalidQuaternionRetainsFiniteRawValuesWithoutDistance)
{
    for (unsigned invalidSide : {0u, 1u, 2u})
    {
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
        const float good[7] = {1, 2, 3, 4, 1, 2, 3}, zero[7] = {0, 0, 0, 0, 1, 2, 3};
        a.writer->setAttribute(1, a.actor, a.pose, reinterpret_cast<const uint8_t*>(invalidSide != 1 ? zero : good), sizeof(good));
        b.writer->setAttribute(1, b.actor, b.pose, reinterpret_cast<const uint8_t*>(invalidSide != 0 ? zero : good), sizeof(good));
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1;
        ComparisonMeasure m; m.attributeName = "globalPose"; m.distance = DistanceKind::TransformOrientationRadians; m.units = "radians";
        const auto result = compareAttribute(left, right, o, m);
        EXPECT_EQ(ComparisonStatus::Incomparable, result.status); EXPECT_FALSE(result.hasEvidence); EXPECT_TRUE(result.hasValues);
        EXPECT_EQ(0u, result.comparedFrames); EXPECT_EQ(4u, result.componentCount);
        const char* reasons[] = {"Invalid left transform quaternion", "Invalid right transform quaternion",
            "Invalid left and right transform quaternions"};
        EXPECT_EQ(reasons[invalidSide], result.reason);
        EXPECT_EQ(SampleState::Exact, result.leftSample.state); EXPECT_EQ(SampleState::Exact, result.rightSample.state);
        EXPECT_EQ(100u, result.leftSample.source.context); EXPECT_EQ(900u, result.rightSample.source.context);
        EXPECT_GT(result.leftSample.commandOrdinal, 0u); EXPECT_GT(result.rightSample.commandOrdinal, 0u);
        const auto json = attributeEvidenceToJson(result, {}, {});
        EXPECT_NE(std::string::npos, json.find("\"hasEvidence\":false,\"distance\":null"));
        EXPECT_NE(std::string::npos, json.find(invalidSide == 1 ? "\"leftValue\":[1,2,3,4]" : "\"leftValue\":[0,0,0,0]"));
        EXPECT_NE(std::string::npos, json.find(invalidSide == 0 ? "\"rightValue\":[1,2,3,4]" : "\"rightValue\":[0,0,0,0]"));
    }
}

TEST(PvdInspection, FailedLaterTransformDoesNotLeakEarlierDecodedValues)
{
    for (unsigned malformed : {0u, 1u, 2u, 3u})
    {
        const bool malformedLeft = (malformed & 1) != 0, wrongType = malformed < 2;
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
        const float good[7] = {1, 2, 3, 4, 1, 2, 3}, other[7] = {1, 2, 3.1f, 4, 2, 3, 4}, zero[7] = {0, 0, 0, 0, 1, 2, 3};
        a.writer->setAttribute(1, a.actor, a.pose, reinterpret_cast<const uint8_t*>(good), sizeof(good));
        b.writer->setAttribute(1, b.actor, b.pose, reinterpret_cast<const uint8_t*>(other), sizeof(other));
        a.frame(2); b.frame(2);
        auto& bad = malformedLeft ? a : b; auto& finite = malformedLeft ? b : a;
        const auto wrong = bad.writer->registerAttribute(bad.actorClass, "probe",
            wrongType ? OmniPvdDataType::eUINT32 : OmniPvdDataType::eFLOAT32, wrongType ? 7 : 6);
        const auto raw = finite.writer->registerAttribute(finite.actorClass, "probe", OmniPvdDataType::eFLOAT32, 7);
        bad.writer->setAttribute(1, bad.actor, wrong, reinterpret_cast<const uint8_t*>(good), wrongType ? sizeof(good) : 6 * sizeof(float));
        finite.writer->setAttribute(1, finite.actor, raw, reinterpret_cast<const uint8_t*>(zero), sizeof(zero));
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 2; o.requireExactSamples = false; o.tolerance = 1;
        ComparisonMeasure m; m.attributeName = "globalPose"; m.distance = DistanceKind::TransformOrientationRadians; m.units = "radians";
        // A finite first comparison is followed by a nonfinite transform in the same attribute.
        float nonfinite[7] = {1, 2, 3, 4, 1, 2,
            wrongType ? std::numeric_limits<float>::infinity() : std::numeric_limits<float>::quiet_NaN()};
        bad.writer->setAttribute(1, bad.actor, bad.pose, reinterpret_cast<const uint8_t*>(nonfinite), sizeof(nonfinite));
        finite.writer->setAttribute(1, finite.actor, finite.pose, reinterpret_cast<const uint8_t*>(zero), sizeof(zero));
        ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto result = compareAttribute(left, right, o, m);
        EXPECT_EQ(ComparisonStatus::Incomparable, result.status); EXPECT_EQ(1u, result.comparedFrames);
        EXPECT_FALSE(result.hasEvidence); EXPECT_FALSE(result.hasValues);
        EXPECT_DOUBLE_EQ(0, result.distance);
        for (unsigned i = 0; i < 4; ++i)
        { EXPECT_DOUBLE_EQ(0, result.leftValue[i]); EXPECT_DOUBLE_EQ(0, result.rightValue[i]); }
        auto json = attributeEvidenceToJson(result, {}, {});
        EXPECT_NE(std::string::npos, json.find("\"leftValue\":null,\"rightValue\":null"));
        EXPECT_NE(std::string::npos, json.find("\"distance\":null"));
        EXPECT_EQ(2u, result.leftSample.requested.recordedFrame);
        m.attributeName = "probe"; o.startFrame = 2;
        result = compareAttribute(left, right, o, m);
        EXPECT_EQ(ComparisonStatus::Incomparable, result.status); EXPECT_FALSE(result.hasEvidence); EXPECT_FALSE(result.hasValues);
        EXPECT_NE(std::string::npos, attributeEvidenceToJson(result, {}, {}).find("\"leftValue\":null,\"rightValue\":null"));
    }
}

TEST(PvdInspection, DecodeFailuresNameEveryFailingSide)
{
    for (bool transform : {false, true})
    for (unsigned failure = 0; failure < 5; ++failure)
    for (unsigned side = 0; side < 3; ++side)
    {
        SCOPED_TRACE(testing::Message() << transform << ':' << failure << ':' << side);
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
        const unsigned width = transform ? 7 : 3;
        for (unsigned index = 0; index < 2; ++index)
        {
            auto& c = index ? b : a;
            const bool bad = side == 2 || side == index;
            const unsigned count = width - (bad && failure == 2 ? 1 : 0);
            const auto attr = c.writer->registerAttribute(c.actorClass, "probe",
                bad && failure == 4 ? OmniPvdDataType::eINT64 : OmniPvdDataType::eFLOAT32, count);
            float values[7] = {1, 2, 3, 4, 1, 2, 3};
            if (bad && failure < 2) values[width - 1] = failure ?
                std::numeric_limits<float>::infinity() : std::numeric_limits<float>::quiet_NaN();
            const int64_t integers[7] = {1, 2, 3, 4, 1, 2, 3};
            if (bad && failure == 4) c.writer->setAttribute(1, c.actor, attr,
                reinterpret_cast<const uint8_t*>(integers), count * sizeof(int64_t));
            else if (!(bad && failure == 3)) c.writer->setAttribute(1, c.actor, attr,
                reinterpret_cast<const uint8_t*>(values), count * sizeof(float));
        }
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1; o.requireExactSamples = false;
        ComparisonMeasure m; m.attributeName = "probe"; m.units = "m";
        m.distance = transform ? DistanceKind::TransformPositionEuclidean : DistanceKind::Vec3Euclidean;
        const auto result = compareAttribute(left, right, o, m);
        EXPECT_EQ(ComparisonStatus::Incomparable, result.status);
        EXPECT_EQ(std::string("Missing, nonfinite or incorrectly typed ") +
            (side == 0 ? "left" : side == 1 ? "right" : "left and right") +
            (transform ? " transform value" : " attribute value"), result.reason);
        EXPECT_FALSE(result.hasValues); EXPECT_FALSE(result.hasEvidence); EXPECT_EQ(0u, result.componentCount);
        EXPECT_DOUBLE_EQ(0, result.distance);
        for (unsigned i = 0; i < 4; ++i)
        { EXPECT_DOUBLE_EQ(0, result.leftValue[i]); EXPECT_DOUBLE_EQ(0, result.rightValue[i]); }
        EXPECT_NE(std::string::npos, attributeEvidenceToJson(result, {}, {}).find("\"leftValue\":null,\"rightValue\":null"));
        EXPECT_EQ(failure == 3 && side != 1 ? SampleState::Missing : SampleState::Exact, result.leftSample.state);
        EXPECT_EQ(failure == 3 && side != 0 ? SampleState::Missing : SampleState::Exact, result.rightSample.state);
    }
}

TEST(PvdInspection, ExactModeReportsSideWithNoWrite)
{
    for (unsigned side = 0; side < 2; ++side)
    {
        SCOPED_TRACE(side);
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
        if (side == 0) b.value(1); else a.value(1);
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1;
        const auto result = compareLinearVelocity(left, right, o);
        EXPECT_EQ(ComparisonStatus::Incomparable, result.status);
        EXPECT_EQ("Missing exact attribute write at aligned recorded frame", result.reason);
        EXPECT_EQ(side == 0 ? SampleState::Missing : SampleState::Exact, result.leftSample.state);
        EXPECT_EQ(side == 0 ? SampleState::Exact : SampleState::Missing, result.rightSample.state);
    }
}

TEST(PvdInspection, MissingLaterVec3DoesNotRetainEarlierDistanceOrArrays)
{
    Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1); a.value(1); b.value(1.05f);
    a.frame(2); b.frame(2); a.value(7);
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 2;
    const auto result = compareLinearVelocity(left, right, o);
    EXPECT_EQ(ComparisonStatus::Incomparable, result.status); EXPECT_EQ(1u, result.comparedFrames);
    EXPECT_FALSE(result.hasEvidence); EXPECT_FALSE(result.hasValues); EXPECT_DOUBLE_EQ(0, result.distance);
    EXPECT_EQ("Missing exact attribute write at aligned recorded frame", result.reason);
    EXPECT_EQ(SampleState::Exact, result.leftSample.state); EXPECT_EQ(SampleState::Carried, result.rightSample.state);
    for (unsigned i = 0; i < 4; ++i)
    { EXPECT_DOUBLE_EQ(0, result.leftValue[i]); EXPECT_DOUBLE_EQ(0, result.rightValue[i]); }
}

TEST(PvdInspection, CompletionUsesSelectedSegmentOnBothSides)
{
    for (bool segmentedLeft : {false, true})
    for (bool stopOld : {false, true})
    {
        Capture a, b(900, 987);
        auto& segmented = segmentedLeft ? a : b; auto& plain = segmentedLeft ? b : a;
        const auto metadata = segmented.writer->registerClass("PxOmniPvdMetaData");
        segmented.writer->createObject(1, metadata, 500, ""); segmented.join(); segmented.frame(1); segmented.value(3);
        if (stopOld) segmented.writer->stopFrame(segmented.scene, 1);
        segmented.writer->createObject(1, metadata, 500, "");
        segmented.writer->createObject(1, segmented.sceneClass, segmented.scene, "scene");
        segmented.create(segmented.actor, "body"); segmented.join(); segmented.frame(1); segmented.value(6);
        if (!stopOld) segmented.writer->stopFrame(segmented.scene, 1);
        plain.join(); plain.frame(1); plain.value(6);
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1;
        if (segmentedLeft) o.leftSegment = 1; else o.rightSegment = 1;
        const auto result = compareLinearVelocity(left, right, o);
        ASSERT_EQ(ComparisonStatus::Equal, result.status) << result.reason;
        EXPECT_TRUE(result.hasLeftFrameCompletion); EXPECT_TRUE(result.hasRightFrameCompletion);
        EXPECT_EQ(!stopOld, segmentedLeft ? result.leftFrameCompleted : result.rightFrameCompleted);
        EXPECT_EQ(1u, (segmentedLeft ? result.leftSample : result.rightSample).source.recordingSegment);
    }
}

TEST(PvdInspection, RightCompletionUsesExplicitContext)
{
    for (bool stopFirst : {false, true})
    {
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1); a.value(6); b.value(3);
        if (stopFirst) b.writer->stopFrame(900, 1);
        b.writer->createObject(1, b.sceneClass, 700, "second scene"); b.create(201, "other body");
        b.scene = 700; b.join(201); b.frame(1); b.value(6, 2, 3, 201);
        if (!stopFirst) b.writer->stopFrame(700, 1);
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1; o.rightContext = 700; o.rightObject.name = "other body";
        const auto result = compareLinearVelocity(left, right, o);
        ASSERT_EQ(ComparisonStatus::Equal, result.status) << result.reason;
        EXPECT_TRUE(result.hasRightFrameCompletion); EXPECT_EQ(!stopFirst, result.rightFrameCompleted);
        EXPECT_EQ(700u, result.rightSample.source.context);
    }
}

TEST(PvdInspection, GenericComparisonRejectsWrongWidthAndType)
{
    for (bool wrongType : {false, true})
    {
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
        const auto lhsAttr = a.writer->registerAttribute(a.actorClass, "probe", OmniPvdDataType::eFLOAT32, 3);
        const auto rhsAttr = b.writer->registerAttribute(b.actorClass, "probe",
            wrongType ? OmniPvdDataType::eUINT32 : OmniPvdDataType::eFLOAT32, wrongType ? 3 : 2);
        const float lhs[3] = {1, 2, 3}, rhs[3] = {1, 2, 3};
        a.writer->setAttribute(1, a.actor, lhsAttr, reinterpret_cast<const uint8_t*>(lhs), sizeof(lhs));
        b.writer->setAttribute(1, b.actor, rhsAttr, reinterpret_cast<const uint8_t*>(rhs), wrongType ? 12 : 8);
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1;
        ComparisonMeasure m; m.attributeName = "probe"; m.distance = DistanceKind::Vec3Euclidean; m.units = "m";
        EXPECT_EQ(ComparisonStatus::Incomparable, compareAttribute(left, right, o, m).status);
    }
}

TEST(PvdInspection, GenericComparisonRejectsNonfiniteValuesAndInvalidMeasures)
{
    Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
    const auto lhsAttr = a.writer->registerAttribute(a.actorClass, "probe", OmniPvdDataType::eFLOAT32, 3);
    const auto rhsAttr = b.writer->registerAttribute(b.actorClass, "probe", OmniPvdDataType::eFLOAT32, 3);
    const float good[3] = {1, 2, 3}, bad[3] = {1, std::numeric_limits<float>::infinity(), 3};
    a.writer->setAttribute(1, a.actor, lhsAttr, reinterpret_cast<const uint8_t*>(good), sizeof(good));
    b.writer->setAttribute(1, b.actor, rhsAttr, reinterpret_cast<const uint8_t*>(good), sizeof(good));
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 1;
    ComparisonMeasure m; m.attributeName = "probe"; m.distance = DistanceKind::Vec3Euclidean; m.units = "m";
    EXPECT_EQ(ComparisonStatus::Equal, compareAttribute(left, right, o, m).status);
    b.writer->setAttribute(1, b.actor, rhsAttr, reinterpret_cast<const uint8_t*>(bad), sizeof(bad));
    ASSERT_TRUE(b.load(right));
    EXPECT_EQ(ComparisonStatus::Incomparable, compareAttribute(left, right, o, m).status);
    b.writer->setAttribute(1, b.actor, rhsAttr, reinterpret_cast<const uint8_t*>(good), sizeof(good));
    ASSERT_TRUE(b.load(right));
    EXPECT_EQ(ComparisonStatus::Equal, compareAttribute(left, right, o, m).status);
    m.attributeName.clear();
    auto invalid = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Incomparable, invalid.status);
    EXPECT_EQ("Invalid comparison measure or units", invalid.reason);
    m.attributeName = "probe"; m.units.clear();
    invalid = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Incomparable, invalid.status);
    EXPECT_EQ("Invalid comparison measure or units", invalid.reason);
    m.units = "m"; m.distance = static_cast<DistanceKind>(255);
    invalid = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Incomparable, invalid.status);
    EXPECT_EQ("Invalid comparison measure or units", invalid.reason);
}

TEST(PvdInspection, GenericComparisonRejectsInvalidOrientationUnits)
{
    Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
    const float pose[7] = {0, 0, 0, 1, 1, 2, 3};
    a.writer->setAttribute(1, a.actor, a.pose, reinterpret_cast<const uint8_t*>(pose), sizeof(pose));
    b.writer->setAttribute(1, b.actor, b.pose, reinterpret_cast<const uint8_t*>(pose), sizeof(pose));
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 1;
    ComparisonMeasure m; m.attributeName = "globalPose";
    m.distance = DistanceKind::TransformOrientationRadians; m.units = "radians";
    EXPECT_EQ(ComparisonStatus::Equal, compareAttribute(left, right, o, m).status);
    m.units = "degrees";
    const auto invalid = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Incomparable, invalid.status);
    EXPECT_EQ("Invalid comparison measure or units", invalid.reason);
}

TEST(PvdInspection, GenericComparisonRejectsRecoveredOtherwiseComparableInput)
{
    Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1);
    a.value(1); b.value(1);
    const auto lhsAttr = a.writer->registerAttribute(a.actorClass, "probe", OmniPvdDataType::eFLOAT32, 3);
    const auto rhsAttr = b.writer->registerAttribute(b.actorClass, "probe", OmniPvdDataType::eFLOAT32, 3);
    const float value[3] = {1, 2, 3};
    a.writer->setAttribute(1, a.actor, lhsAttr, reinterpret_cast<const uint8_t*>(value), sizeof(value));
    b.writer->setAttribute(1, b.actor, rhsAttr, reinterpret_cast<const uint8_t*>(value), sizeof(value));
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 1;
    ComparisonMeasure m; m.attributeName = "probe"; m.distance = DistanceKind::Vec3Euclidean; m.units = "m";
    EXPECT_EQ(ComparisonStatus::Equal, compareAttribute(left, right, o, m).status);
    EXPECT_EQ(ComparisonStatus::Equal, compareLinearVelocity(left, right, o).status);
    for (bool recoverLeft : {false, true})
    {
        auto& capture = recoverLeft ? a : b; auto& document = recoverLeft ? left : right;
        const auto size = capture.bytes.size();
        capture.bytes.push_back(uint8_t(OmniPvdCommand::eSTART_FRAME)); capture.bytes.push_back(1);
        ASSERT_TRUE(capture.load(document, Error::Code::None, LoadMode::RecoverTruncatedTail));
        EXPECT_TRUE(document.loadInfo().recoveredTail);
        for (const auto& result : {compareAttribute(left, right, o, m), compareLinearVelocity(left, right, o)})
        {
            EXPECT_EQ(ComparisonStatus::Incomparable, result.status);
            EXPECT_EQ("Recovered-tail documents cannot be compared", result.reason);
            EXPECT_FALSE(result.hasEvidence); EXPECT_EQ(0u, result.comparedFrames);
        }
        capture.bytes.resize(size); ASSERT_TRUE(capture.load(document));
        const auto restored = compareAttribute(left, right, o, m);
        ASSERT_EQ(ComparisonStatus::Equal, restored.status); EXPECT_DOUBLE_EQ(0, restored.distance);
        EXPECT_DOUBLE_EQ(1, restored.leftValue[0]); EXPECT_DOUBLE_EQ(3, restored.rightValue[2]);
    }
}

TEST(PvdInspection, MixedFrameCompletionUsesExplicitContextForSameFrameNumber)
{
    for (bool stopFirst : {false, true})
    {
        Capture a, b(900, 987); a.join(); b.join(); a.frame(1); b.frame(1); a.value(3); b.value(6);
        if (stopFirst) a.writer->stopFrame(100, 1); else b.writer->stopFrame(900, 1);
        a.writer->createObject(1, a.sceneClass, 700, "second scene"); a.create(201, "other body");
        a.scene = 700; a.join(201); a.frame(1); a.value(6, 2, 3, 201);
        if (!stopFirst) a.writer->stopFrame(700, 1);
        Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
        auto o = options(); o.endFrame = 1;
        ComparisonMeasure m; m.attributeName = "linearVelocity"; m.units = "m/s";
        for (bool second : {false, true})
        {
            o.leftContext = second ? 700 : 100; o.leftObject.name = second ? "other body" : "body";
            const auto result = compareAttribute(left, right, o, m);
            ASSERT_EQ(second ? ComparisonStatus::Equal : ComparisonStatus::Different, result.status) << result.reason;
            EXPECT_DOUBLE_EQ(second ? 0 : 3, result.distance);
            EXPECT_DOUBLE_EQ(second ? 6 : 3, result.leftValue[0]); EXPECT_DOUBLE_EQ(6, result.rightValue[0]);
            EXPECT_EQ(second ? !stopFirst : stopFirst, result.leftFrameCompleted);
            EXPECT_EQ(!stopFirst, result.rightFrameCompleted);
            EXPECT_EQ(o.leftContext, result.leftSample.source.context);
            const auto json = attributeEvidenceToJson(result, {}, {});
            EXPECT_NE(std::string::npos, json.find(second ? "\"leftValue\":[6,2,3]" : "\"leftValue\":[3,2,3]"));
            EXPECT_NE(std::string::npos, json.find("\"rightValue\":[6,2,3]"));
            EXPECT_NE(std::string::npos, json.find(result.leftFrameCompleted ? "\"leftFrameCompleted\":true" : "\"leftFrameCompleted\":false"));
            EXPECT_NE(std::string::npos, json.find(stopFirst ? "\"rightFrameCompleted\":false" : "\"rightFrameCompleted\":true"));
        }
    }
}

TEST(PvdInspection, GenericComparisonKeepsAlignmentCarriedAndCompletionEvidence)
{
    Capture a, b(900, 987); a.join(); b.join();
    a.frame(1); a.value(1); a.writer->stopFrame(a.scene, 1);
    b.frame(11); b.value(1); b.writer->stopFrame(b.scene, 11);
    a.frame(2); b.frame(12); a.value(2); b.value(2);
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 2; o.rightFrameOffset = 10;
    ComparisonMeasure m; m.attributeName = "linearVelocity"; m.distance = DistanceKind::Vec3Euclidean; m.units = "m/s";
    auto result = compareAttribute(left, right, o, m);
    ASSERT_EQ(ComparisonStatus::Equal, result.status) << result.reason;
    EXPECT_EQ(2u, result.comparedFrames);
    EXPECT_TRUE(result.hasLeftFrameCompletion); EXPECT_TRUE(result.hasRightFrameCompletion);
    EXPECT_FALSE(result.leftFrameCompleted); EXPECT_FALSE(result.rightFrameCompleted);
    EXPECT_NE(std::string::npos, attributeEvidenceToJson(result, {}, {}).find("\"leftFrameCompleted\":false"));
    o.endFrame = 1; result = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Equal, result.status) << result.reason;
    EXPECT_TRUE(result.leftFrameCompleted); EXPECT_TRUE(result.rightFrameCompleted);
    o.endFrame = 2; a.value(4); ASSERT_TRUE(a.load(left));
    result = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Different, result.status); EXPECT_EQ(2u, result.comparedFrames);
    o.tolerance = 2; EXPECT_EQ(ComparisonStatus::Equal, compareAttribute(left, right, o, m).status);
    a.value(2); a.frame(3); b.frame(13); ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    o.endFrame = 3; o.tolerance = 0;
    result = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Incomparable, result.status);
    EXPECT_EQ(2u, result.comparedFrames);
    EXPECT_FALSE(result.hasEvidence);
    EXPECT_NE(std::string::npos, attributeEvidenceToJson(result, {}, {}).find("\"leftValue\":null"));
    o.requireExactSamples = false;
    EXPECT_EQ(ComparisonStatus::Equal, compareAttribute(left, right, o, m).status);
    a.frame(4); b.frame(14); a.writer->destroyObject(1, a.actor);
    ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    o.endFrame = 4;
    EXPECT_EQ(ComparisonStatus::Incomparable, compareAttribute(left, right, o, m).status);
    o.frameStride = 0; EXPECT_EQ(ComparisonStatus::Incomparable, compareAttribute(left, right, o, m).status);
    o.frameStride = 1; o.startFrame = 5; o.endFrame = 5;
    result = compareAttribute(left, right, o, m);
    EXPECT_EQ(ComparisonStatus::Incomparable, result.status);
    EXPECT_FALSE(result.hasLeftFrameCompletion); EXPECT_FALSE(result.hasRightFrameCompletion);
    EXPECT_NE(std::string::npos, attributeEvidenceToJson(result, {}, {}).find("\"leftFrameCompleted\":null"));
}

TEST(PvdInspection, MissingNonfiniteAmbiguousAndNoOverlapAreIncomparable)
{
    Capture a, b(900, 987); a.join(); b.join();
    a.frame(1); a.value(1); b.frame(1);
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 1;
    EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(left, right, o).status);
    b.value(std::numeric_limits<float>::quiet_NaN()); ASSERT_TRUE(b.load(right));
    EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(left, right, o).status);
    b.value(1); b.create(988, "body"); b.join(988); ASSERT_TRUE(b.load(right));
    EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(left, right, o).status);
    o.rightFrameOffset = 100;
    EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(left, right, o).status);
}

TEST(PvdInspection, ExplicitOffsetStrideAndCarriedOptIn)
{
    Capture a, b(900, 987); a.join(); b.join();
    a.frame(2); a.value(1); a.frame(3); a.frame(4); a.value(2);
    b.frame(12); b.value(1); b.frame(13); b.frame(14); b.value(2);
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.startFrame = 2; o.endFrame = 4; o.rightFrameOffset = 10; o.frameStride = 2;
    auto r = compareLinearVelocity(left, right, o); EXPECT_EQ(ComparisonStatus::Equal, r.status) << r.reason;
    EXPECT_EQ(2u, r.comparedFrames);
    o.frameStride = 1; EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(left, right, o).status);
    o.requireExactSamples = false; EXPECT_EQ(ComparisonStatus::Equal, compareLinearVelocity(left, right, o).status);
    o.frameStride = 0; EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(left, right, o).status);
    o.frameStride = 1; o.rightFrameOffset = (std::numeric_limits<int64_t>::min)();
    EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(left, right, o).status);
    o.startFrame = o.endFrame = UINT64_MAX - 1; o.rightFrameOffset = 2;
    const auto overflow = compareLinearVelocity(left, right, o);
    EXPECT_EQ(ComparisonStatus::Incomparable, overflow.status);
    EXPECT_EQ(0u, overflow.comparedFrames);
    EXPECT_NE(std::string::npos, overflow.reason.find("overflows"));
}

TEST(PvdInspection, ClassRestrictionSelectsOnlyMatchingNamedObject)
{
    Capture c; c.frame(1); c.join();
    const auto staticClass = c.writer->registerClass("PxRigidStatic");
    c.writer->createObject(1, staticClass, 201, "body"); c.join(201);
    Document d; ASSERT_TRUE(c.load(d));
    ObjectIdentity selected; std::string error;
    EXPECT_FALSE(selectObject(d, body(), frame(1), selected, error));
    ObjectSelector dynamic = body(); dynamic.className = "PxRigidDynamic";
    ASSERT_TRUE(selectObject(d, dynamic, frame(1), selected, error)) << error;
    EXPECT_EQ(200u, selected.externalHandle);
    ObjectSelector stationary = body(); stationary.className = "PxRigidStatic";
    ASSERT_TRUE(selectObject(d, stationary, frame(1), selected, error)) << error;
    EXPECT_EQ(201u, selected.externalHandle);
}

TEST(PvdInspection, EvidencePreservesUtf8EscapesInvalidTextAndLargeIdentities)
{
    struct Comma : std::numpunct<char> { char do_decimal_point() const override { return ','; } };
    const std::locale previous = std::locale();
    std::locale::global(std::locale(previous, new Comma));
    ComparisonResult r;
    r.status = ComparisonStatus::Different; r.hasEvidence = true; r.distance = 0.25; r.options.tolerance = 0.125;
    r.leftSample.identity.objectId = UINT64_MAX;
    r.leftSample.identity.name = std::string("invalid\xFF", 8);
    r.reason = "quotes\" slash\\ tab\t control\x01";
    CaptureProvenance p; p.sourcePath = "caf\xC3\xA9/\xF0\x9F\x8C\x8D.ovd";
    const auto json = evidenceToJson(r, p, CaptureProvenance());
    std::locale::global(previous);
    EXPECT_NE(std::string::npos, json.find(p.sourcePath));
    EXPECT_EQ(std::string::npos, json.find('\xFF'));
    EXPECT_NE(std::string::npos, json.find("\\ufffd"));
    EXPECT_NE(std::string::npos, json.find("\"textWasReplaced\":true"));
    EXPECT_NE(std::string::npos, json.find("\"objectId\":\"18446744073709551615\""));
    EXPECT_NE(std::string::npos, json.find("\"distance\":0.25"));
    EXPECT_NE(std::string::npos, json.find("\"tolerance\":0.125"));
    EXPECT_NE(std::string::npos, json.find("\\u0001"));
    r.options.tolerance = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NE(std::string::npos, evidenceToJson(r, p, p).find("\"tolerance\":null"));
}

TEST(PvdInspection, SceneMoveCarriesStateWithoutSubtractingDifferentClocks)
{
    Capture c; c.frame(10); c.join(); c.value(7); c.frame(11); c.leave();
    c.scene = 900; c.writer->createObject(1, c.sceneClass, c.scene, "second scene"); c.frame(1); c.join();
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(1, 900), id, error)) << error;
    SampleResult sample; ASSERT_TRUE(readSample(d, id.objectId, frame(1, 900), "linearVelocity", sample, error)) << error;
    EXPECT_EQ(SampleState::Carried, sample.state); EXPECT_TRUE(sample.preMembership); EXPECT_FALSE(sample.ageAvailable);
    EXPECT_EQ(100u, sample.source.context); EXPECT_EQ(10u, sample.source.recordedFrame);
    EXPECT_EQ(900u, sample.requested.context); EXPECT_EQ(1u, sample.requested.recordedFrame);
    double value[3]; ASSERT_TRUE(PvdDomInspect::readVec3(sample, value)); EXPECT_EQ(7, value[0]);
    ASSERT_TRUE(selectObject(d, body(), frame(10), id, error)) << error;
    EXPECT_FALSE(selectObject(d, body(), frame(11), id, error));
}

TEST(PvdInspection, BeforeFirstFrameIsCarriedAndAfterStopStillUsesLatestStart)
{
    Capture c; c.join(); c.value(1); c.frame(0);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(0), id, error)); SampleResult sample;
    ASSERT_TRUE(readSample(d, id.objectId, frame(0), "linearVelocity", sample, error));
    EXPECT_EQ(SampleState::Carried, sample.state); EXPECT_FALSE(sample.hasSourceFrame);
    EXPECT_EQ(0u, sample.source.recordedFrame);
    ComparisonResult evidence; evidence.leftSample = sample;
    EXPECT_NE(std::string::npos, leftSampleJson(evidence).find("\"source\":null"));
    c.writer->stopFrame(c.scene, 0); c.value(2); ASSERT_TRUE(c.load(d));
    ASSERT_TRUE(readSample(d, id.objectId, frame(0), "linearVelocity", sample, error));
    EXPECT_EQ(SampleState::Exact, sample.state); EXPECT_TRUE(d.frames()[0].completed);
    double value[3]; ASSERT_TRUE(PvdDomInspect::readVec3(sample, value)); EXPECT_EQ(2, value[0]);
}

TEST(PvdInspection, HistoricalNamesAndComparisonKeepTheSelectedIncarnation)
{
    Capture a, b(900, 987, false, true); a.join(); b.join(); a.frame(1); b.frame(1); a.value(1); b.value(1);
    a.frame(2); b.frame(2); a.value(2); b.value(2);
    const char renamed[] = "renamed";
    b.writer->setAttribute(1, b.actor, b.label, reinterpret_cast<const uint8_t*>(renamed), sizeof(renamed));
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    ObjectIdentity id; std::string error;
    ObjectIdentity leftSelected;
    ASSERT_TRUE(selectObject(left, body(), frame(1), leftSelected, error));
    ASSERT_TRUE(selectObject(right, body(), frame(1, 900), id, error));
    const ObjectIdentity rightSelected = id;
    ASSERT_NE(leftSelected.objectId, rightSelected.objectId);
    EXPECT_FALSE(selectObject(right, body(), frame(2, 900), id, error));
    auto selector = body(); selector.name = renamed;
    EXPECT_FALSE(selectObject(right, selector, frame(1, 900), id, error));
    ASSERT_TRUE(selectObject(right, selector, frame(2, 900), id, error));
    auto o = options(); o.endFrame = 2;
    EXPECT_EQ(ComparisonStatus::Equal, compareLinearVelocity(left, right, o).status);
    b.writer->destroyObject(1, b.actor); b.create(b.actor, "body"); b.join(); b.value(2); ASSERT_TRUE(b.load(right));
    const auto rightFailure = compareLinearVelocity(left, right, o);
    EXPECT_EQ(ComparisonStatus::Incomparable, rightFailure.status);
    EXPECT_TRUE(rightFailure.hasLeftIdentity); EXPECT_TRUE(rightFailure.hasRightIdentity);
    EXPECT_EQ(leftSelected.objectId, rightFailure.leftIdentity.objectId);
    EXPECT_EQ(rightSelected.objectId, rightFailure.rightIdentity.objectId);
    EXPECT_EQ("body", rightFailure.rightIdentity.name);
    const auto json = evidenceToJson(rightFailure, {}, {});
    EXPECT_NE(std::string::npos, json.find("\"leftIdentity\":{\"objectId\":\"" + std::to_string(leftSelected.objectId)));
    EXPECT_NE(std::string::npos, json.find("\"rightIdentity\":{\"objectId\":\"" + std::to_string(rightSelected.objectId)));
    a.writer->destroyObject(1, a.actor); a.create(a.actor, "body"); a.join(); a.value(2); ASSERT_TRUE(a.load(left));
    const auto leftFailure = compareLinearVelocity(left, right, o);
    EXPECT_EQ(ComparisonStatus::Incomparable, leftFailure.status);
    EXPECT_TRUE(leftFailure.hasLeftIdentity); EXPECT_TRUE(leftFailure.hasRightIdentity);
    EXPECT_EQ(leftSelected.objectId, leftFailure.leftIdentity.objectId);
    EXPECT_EQ(rightSelected.objectId, leftFailure.rightIdentity.objectId);
}

TEST(PvdInspection, MissingLeftFramePreservesExactRightSample)
{
    Capture a, b(900, 987); a.join(); b.join();
    for (uint64_t f = 1; f <= 8; ++f)
    {
        if (f <= 6) { a.frame(f); a.value(float(f), 0, 0); }
        b.frame(f); b.value(float(f), 0, 0);
    }
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 8; o.frameStride = 7;
    const auto result = compareLinearVelocity(left, right, o);
    EXPECT_EQ(ComparisonStatus::Incomparable, result.status);
    EXPECT_EQ(SampleState::Missing, result.leftSample.state);
    EXPECT_EQ(SampleState::Exact, result.rightSample.state);
    double value[3]; ASSERT_TRUE(readVec3(result.rightSample, value));
    EXPECT_EQ(8, value[0]); EXPECT_EQ(0, value[1]); EXPECT_EQ(0, value[2]);
    EXPECT_NE(std::string::npos, result.reason.find("left"));
    EXPECT_EQ(std::string::npos, result.reason.find("right"));
}

TEST(PvdInspection, FrameResetsAreUnsupportedWithinScopeButIndependentAcrossSegments)
{
    for (uint64_t next : {uint64_t(1), uint64_t(0)})
    {
        Capture c; c.join(); c.frame(1); c.value(1); c.frame(next); c.value(2);
        Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
        EXPECT_FALSE(selectObject(d, body(), frame(1), id, error));
        EXPECT_NE(std::string::npos, error.find("Unsupported"));
    }
    Capture c;
    const auto metadata = c.writer->registerClass("PxOmniPvdMetaData");
    c.writer->createObject(1, metadata, 500, ""); c.join(); c.frame(1); c.value(1);
    c.writer->createObject(1, metadata, 500, "");
    c.writer->createObject(1, c.sceneClass, c.scene, "scene"); c.create(c.actor, "body"); c.join(); c.frame(1); c.value(9);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity first, second; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(1), first, error)) << error;
    ASSERT_TRUE(selectObject(d, body(), frame(1, 100, 1), second, error)) << error;
    EXPECT_NE(first.objectId, second.objectId);
    SampleResult sample;
    ASSERT_TRUE(readSample(d, first.objectId, frame(1), "linearVelocity", sample, error));
    double value[3]; ASSERT_TRUE(PvdDomInspect::readVec3(sample, value)); EXPECT_EQ(1, value[0]);
    ASSERT_TRUE(readSample(d, second.objectId, frame(1, 100, 1), "linearVelocity", sample, error));
    ASSERT_TRUE(PvdDomInspect::readVec3(sample, value)); EXPECT_EQ(9, value[0]);
    EXPECT_FALSE(readSample(d, first.objectId, frame(1, 100, 1), "linearVelocity", sample, error));
}

TEST(PvdInspection, PreMembershipWriteDoesNotBorrowAnotherSceneClock)
{
    Capture c; c.frame(10);
    c.writer->createObject(1, c.sceneClass, 900, "other scene");
    c.writer->startFrame(900, 99);
    c.value(7); c.join(); c.frame(11);
    Document d; ASSERT_TRUE(c.load(d));
    ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(11), id, error)) << error;
    SampleResult sample;
    ASSERT_TRUE(readSample(d, id.objectId, frame(11), "linearVelocity", sample, error)) << error;
    EXPECT_EQ(SampleState::Carried, sample.state);
    EXPECT_FALSE(sample.hasSourceFrame); EXPECT_FALSE(sample.ageAvailable);
    EXPECT_EQ(0u, sample.source.recordedFrame);
    EXPECT_EQ(1u, sample.rawContext); EXPECT_FALSE(sample.hasRawFrame);
    EXPECT_GT(sample.commandOrdinal, 0u);
    const auto actorId = d.dom()->mExternalToInternalHandleMap.at(c.actor);
    const auto& writes = d.history().at(actorId).writes.at(c.velocity);
    ASSERT_EQ(1u, writes.size());
    EXPECT_FALSE(writes[0].frameKnown); EXPECT_EQ(0u, writes[0].frame);
    ComparisonResult evidence; evidence.leftSample = sample;
    EXPECT_NE(std::string::npos, leftSampleJson(evidence).find("\"source\":null"));
}

TEST(PvdInspection, EvidenceReplacesMalformedUtf8Sequences)
{
    const std::string malformed[] = {
        std::string("\x80", 1), std::string("\xED\xA0\x80", 3),
        std::string("\xE0\x80\x80", 3), std::string("\xF0\x9F\x8C", 3)
    };
    for (const auto& input : malformed)
    {
        ComparisonResult result; result.reason = input;
        const auto json = evidenceToJson(result, {}, {});
        EXPECT_EQ(std::string::npos, json.find(input));
        EXPECT_NE(std::string::npos, json.find("\\ufffd"));
        EXPECT_NE(std::string::npos, json.find("\"textWasReplaced\":true"));
    }
}

TEST(PvdInspection, GeneratedAndTrackedSceneFixturesExposePoseAtSceneFrame)
{
    for (const char* path : {PVDDOM_GENERATED_TEST_FILE, PVDDOM_TEST_DATA_DIR "/test_scene.ovd"})
    {
        SCOPED_TRACE(path);
        Document d; Error fixtureError;
        ASSERT_TRUE(d.load(path, LoadLimits(), fixtureError)) << fixtureError.message;
        const auto index = d.frameIndex().find({0, 100});
        ASSERT_NE(d.frameIndex().end(), index);
        ASSERT_EQ(10u, index->second.recordedFrames.size());
        EXPECT_EQ(1u, index->second.recordedFrames.front());
        ObjectSelector selector; selector.name = "FallingBox_0";
        ObjectIdentity id; std::string queryError;
        ASSERT_TRUE(selectObject(d, selector, frame(1), id, queryError)) << queryError;
        SampleResult sample;
        ASSERT_TRUE(readSample(d, id.objectId, frame(1), "globalPose", sample, queryError)) << queryError;
        double position[3], quaternion[4];
        ASSERT_TRUE(readTransform(sample, position, quaternion));
        EXPECT_NEAR(-2.0, position[0], 1e-5);
        EXPECT_NEAR(9.997275, position[1], 1e-4);
        EXPECT_NEAR(0.0, position[2], 1e-5);
        EXPECT_NEAR(0.0, quaternion[0], 1e-6);
        EXPECT_NEAR(0.0, quaternion[1], 1e-6);
        EXPECT_NEAR(0.0, quaternion[2], 1e-6);
        EXPECT_NEAR(1.0, quaternion[3], 1e-6);
    }
}

TEST(PvdInspection, RecreatedSceneContextIsUnsupportedAndDoesNotCarryOldClock)
{
    for (uint64_t next : {uint64_t(1), uint64_t(3)})
    {
        Capture c; c.join(); c.frame(1); c.value(1); c.frame(2);
        c.writer->destroyObject(1, c.scene);
        c.writer->createObject(1, c.sceneClass, c.scene, "second scene");
        c.create(201, "second body"); c.join(201); c.value(9, 2, 3, 201);
        c.frame(next);
        Document d; ASSERT_TRUE(c.load(d));
        const auto& index = d.frameIndex().at({0, c.scene});
        EXPECT_TRUE(index.sceneContextRecreated);
        for (uint64_t queried : {uint64_t(1), next})
        {
            ObjectIdentity id; std::string error;
            EXPECT_FALSE(selectObject(d, body(), frame(queried), id, error));
            EXPECT_EQ("Unsupported scene context re-created within recording segment", error);
        }
        const auto second = d.dom()->mExternalToInternalHandleMap.at(201);
        const auto& writes = d.history().at(second).writes.at(c.velocity);
        ASSERT_EQ(1u, writes.size());
        EXPECT_FALSE(writes[0].frameKnown);
        EXPECT_EQ(0u, writes[0].frame);
    }
}

TEST(PvdInspection, SupersededSceneClosesHistoryAtLastValidClock)
{
    Capture c;
    const auto shapeClass = c.writer->registerClass("PxShape");
    const auto shapes = c.writer->registerUniqueListAttribute(c.actorClass, "shapes", OmniPvdDataType::eOBJECT_HANDLE);
    c.create(201, "second body"); c.writer->createObject(1, shapeClass, 300, "shape");
    c.frame(7); c.join(); c.join(201);
    const uint64_t shapeHandle = 300;
    c.writer->addToUniqueListAttribute(1, c.actor, shapes, reinterpret_cast<const uint8_t*>(&shapeHandle), 8);
    c.value(1); c.frame(10);
    Document before; ASSERT_TRUE(c.load(before));
    const auto* oldScene = before.history().at(before.dom()->mExternalToInternalHandleMap.at(c.scene)).object;
    const auto* firstActor = before.history().at(before.dom()->mExternalToInternalHandleMap.at(c.actor)).object;
    const auto* secondActor = before.history().at(before.dom()->mExternalToInternalHandleMap.at(201)).object;
    const auto* shape = before.history().at(before.dom()->mExternalToInternalHandleMap.at(shapeHandle)).object;
    EXPECT_EQ(oldScene, firstActor->mAncestor);
    EXPECT_EQ(oldScene, secondActor->mAncestor);
    EXPECT_EQ(firstActor, shape->mAncestor);
    const auto visitsBefore = before.stats().membershipVisits;
    c.writer->createObject(1, c.sceneClass, c.scene, "replacement scene");
    Document d; ASSERT_TRUE(c.load(d));
    EXPECT_EQ(10u, d.stats().membershipVisits - visitsBefore);
    for (uint64_t handle : {c.actor, uint64_t(201), shapeHandle})
    {
        const auto id = d.dom()->mExternalToInternalHandleMap.at(handle);
        const auto& spans = d.history().at(id).memberships;
        ASSERT_EQ(1u, spans.size()); EXPECT_EQ(10u, spans[0].endFrame);
        EXPECT_NE(0u, spans[0].stopOrdinal);
    }
    ObjectIdentity id; std::string error;
    EXPECT_FALSE(selectObject(d, body(), frame(7), id, error));
    EXPECT_EQ("Unsupported scene context re-created within recording segment", error);
}

TEST(PvdInspection, SnapshotConstraintAddDoesNotRevivePreviousSegmentJoint)
{
    Capture c;
    const auto metadata = c.writer->registerClass("PxOmniPvdMetaData");
    const auto joints = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(joints, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const uint64_t handle = 801;
    c.writer->createObject(1, metadata, 500, "");
    c.writer->createObject(1, joints, 700, "old joint"); c.frame(7);
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document before; ASSERT_TRUE(c.load(before));
    const auto oldId = before.dom()->mExternalToInternalHandleMap.at(700);
    const auto oldAncestorId = before.history().at(oldId).object->mAncestor->mRecordingSegmentId;
    const auto oldSpans = before.history().at(oldId).object->mLifeSpans.size();
    c.writer->createObject(1, metadata, 500, "");
    c.writer->createObject(1, c.sceneClass, c.scene, "new scene"); c.frame(1);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto* old = d.history().at(oldId).object;
    EXPECT_EQ(oldAncestorId, old->mAncestor->mRecordingSegmentId);
    EXPECT_EQ(oldSpans, old->mLifeSpans.size());
    EXPECT_FALSE(isObjectAliveAtFrame(old, 1));
    c.writer->createObject(1, joints, 700, "replacement joint");
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    ASSERT_TRUE(c.load(d));
    old = d.history().at(oldId).object;
    EXPECT_TRUE(old->mWasDestroyed); EXPECT_EQ(oldAncestorId, old->mAncestor->mRecordingSegmentId);
    const auto newId = d.dom()->mExternalToInternalHandleMap.at(700);
    ASSERT_NE(oldId, newId);
    const auto& spans = d.history().at(newId).memberships;
    ASSERT_EQ(1u, spans.size()); EXPECT_EQ(1u, spans[0].recordingSegment); EXPECT_EQ(100u, spans[0].context);
}

TEST(PvdInspection, NewSegmentJointConstraintSetDoesNotAttachToOldSegmentScene)
{
    Capture c;
    const auto metadata = c.writer->registerClass("PxOmniPvdMetaData");
    const auto joints = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(joints, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const uint64_t handle = 801;
    c.writer->createObject(1, metadata, 500, "");
    c.writer->createObject(1, joints, 700, "old joint"); c.frame(7);
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->createObject(1, metadata, 500, "");
    c.writer->createObject(1, c.sceneClass, c.scene, "new scene"); c.frame(1);
    c.writer->createObject(1, joints, 700, "new joint");
    Document before; ASSERT_TRUE(c.load(before));
    const auto jointId = before.dom()->mExternalToInternalHandleMap.at(700);
    const auto baselineSpans = before.history().at(jointId).object->mLifeSpans.size();
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto* joint = d.history().at(jointId).object;
    EXPECT_EQ(nullptr, joint->mAncestor);
    EXPECT_EQ(baselineSpans, joint->mLifeSpans.size());
    EXPECT_TRUE(d.history().at(jointId).memberships.empty());
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    ASSERT_TRUE(c.load(d));
    joint = d.history().at(jointId).object;
    const auto* scene = d.history().at(d.dom()->mExternalToInternalHandleMap.at(c.scene)).object;
    ASSERT_NE(nullptr, joint->mAncestor);
    EXPECT_EQ(scene->mSceneJoints, joint->mAncestor);
    EXPECT_EQ(scene, joint->mAncestor->mAncestor);
    const auto& memberships = d.history().at(jointId).memberships;
    ASSERT_EQ(1u, memberships.size());
    EXPECT_EQ(1u, memberships[0].recordingSegment);
    EXPECT_EQ(100u, memberships[0].context);
}

TEST(PvdInspection, LiveOldSegmentSceneRejectsNewJointConstraintSet)
{
    Capture c;
    const auto metadata = c.writer->registerClass("PxOmniPvdMetaData");
    const auto joints = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(joints, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const uint64_t handle = 801;
    c.writer->createObject(1, metadata, 500, "");
    c.writer->createObject(1, joints, 700, "old joint"); c.frame(7);
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->createObject(1, metadata, 500, "");
    c.scene = 900; c.writer->createObject(1, c.sceneClass, c.scene, "new scene"); c.frame(1);
    c.writer->createObject(1, joints, 700, "new joint");
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto jointId = d.dom()->mExternalToInternalHandleMap.at(700);
    const auto* joint = d.history().at(jointId).object;
    const auto* oldScene = d.history().at(d.dom()->mExternalToInternalHandleMap.at(100)).object;
    EXPECT_FALSE(oldScene->mWasDestroyed);
    EXPECT_EQ(nullptr, joint->mAncestor);
    EXPECT_TRUE(d.history().at(jointId).memberships.empty());
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    ASSERT_TRUE(c.load(d));
    joint = d.history().at(jointId).object;
    const auto* newScene = d.history().at(d.dom()->mExternalToInternalHandleMap.at(c.scene)).object;
    ASSERT_NE(nullptr, joint->mAncestor);
    EXPECT_EQ(newScene->mSceneJoints, joint->mAncestor);
    const auto& memberships = d.history().at(jointId).memberships;
    ASSERT_EQ(1u, memberships.size());
    EXPECT_EQ(1u, memberships[0].recordingSegment);
    EXPECT_EQ(900u, memberships[0].context);
}

TEST(PvdInspection, DestroyedSceneRejectsNewJointConstraintSet)
{
    Capture c;
    const auto joints = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(joints, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const uint64_t handle = 801;
    c.frame(1);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->destroyObject(1, c.scene);
    c.writer->createObject(1, joints, 700, "late joint");
    Document before; ASSERT_TRUE(c.load(before));
    const auto id = before.dom()->mExternalToInternalHandleMap.at(700);
    const auto baselineSpans = before.history().at(id).object->mLifeSpans.size();
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto* joint = d.history().at(id).object;
    EXPECT_EQ(nullptr, joint->mAncestor);
    EXPECT_EQ(baselineSpans, joint->mLifeSpans.size());
    EXPECT_TRUE(d.history().at(id).memberships.empty());
}

TEST(PvdInspection, SupersededSceneRejectsNewJointConstraintSet)
{
    Capture c;
    const auto joints = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(joints, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const uint64_t handle = 801;
    c.frame(1);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.frame(2); c.writer->createObject(1, c.sceneClass, c.scene, "replacement scene");
    c.writer->createObject(1, joints, 700, "late joint");
    Document before; ASSERT_TRUE(c.load(before));
    const auto id = before.dom()->mExternalToInternalHandleMap.at(700);
    const auto baselineSpans = before.history().at(id).object->mLifeSpans.size();
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto* joint = d.history().at(id).object;
    EXPECT_EQ(nullptr, joint->mAncestor);
    EXPECT_EQ(baselineSpans, joint->mLifeSpans.size());
    EXPECT_TRUE(d.history().at(id).memberships.empty());
}

TEST(PvdInspection, SupersededSceneRejectsExistingJointConstraintReset)
{
    Capture c;
    const auto joints = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(joints, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const uint64_t handle = 801;
    c.writer->createObject(1, joints, 700, "joint"); c.frame(1);
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.frame(2); c.writer->createObject(1, c.sceneClass, c.scene, "replacement scene");
    Document before; ASSERT_TRUE(c.load(before));
    const auto id = before.dom()->mExternalToInternalHandleMap.at(700);
    const auto* oldAncestor = before.history().at(id).object->mAncestor;
    ASSERT_NE(nullptr, oldAncestor);
    ASSERT_NE(nullptr, oldAncestor->mAncestor);
    const auto oldSceneId = oldAncestor->mAncestor->mUID;
    const auto baselineSpans = before.history().at(id).object->mLifeSpans.size();
    const auto baselineMemberships = before.history().at(id).memberships.size();
    ASSERT_EQ(1u, baselineMemberships);
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto* joint = d.history().at(id).object;
    ASSERT_NE(nullptr, joint->mAncestor);
    ASSERT_NE(nullptr, joint->mAncestor->mAncestor);
    EXPECT_EQ(oldSceneId, joint->mAncestor->mAncestor->mUID);
    EXPECT_EQ(baselineSpans, joint->mLifeSpans.size());
    EXPECT_EQ(baselineMemberships, d.history().at(id).memberships.size());
    EXPECT_NE(0u, d.history().at(id).memberships.back().stopOrdinal);
}

TEST(PvdInspection, DestroyedJointLookupDoesNotReopenAttachment)
{
    Capture c;
    const auto joints = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(joints, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const uint64_t handle = 801;
    c.writer->createObject(1, joints, 700, "joint"); c.frame(1);
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->destroyObject(1, 700);
    Document before; ASSERT_TRUE(c.load(before));
    const auto id = before.dom()->mExternalToInternalHandleMap.at(700);
    const auto baselineSpans = before.history().at(id).object->mLifeSpans.size();
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto* joint = d.history().at(id).object;
    EXPECT_TRUE(joint->mWasDestroyed);
    EXPECT_EQ(nullptr, joint->mAncestor);
    EXPECT_EQ(baselineSpans, joint->mLifeSpans.size());
    EXPECT_TRUE(d.history().at(id).memberships.empty());
}

TEST(PvdInspection, FirstSceneCreateAfterStartRetainsExactClock)
{
    Capture c(100, 200, true); c.join(); c.value(3);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(1), id, error)) << error;
    SampleResult sample; ASSERT_TRUE(readSample(d, id.objectId, frame(1), "linearVelocity", sample, error));
    EXPECT_EQ(SampleState::Exact, sample.state);
}

TEST(PvdInspection, ObjectIdSelectionEnforcesClassRestriction)
{
    Capture c; c.frame(1); c.join(); Document d; ASSERT_TRUE(c.load(d));
    ObjectSelector selector; selector.objectId = d.dom()->mExternalToInternalHandleMap.at(c.actor);
    selector.className = "PxScene"; ObjectIdentity id; std::string error;
    EXPECT_FALSE(selectObject(d, selector, frame(1), id, error));
    EXPECT_EQ("Selected object does not match class restriction", error);
}

TEST(PvdInspection, SameFrameRejoinCarriesPreMembershipWrite)
{
    Capture c; c.frame(1); c.join(); c.value(3); c.leave(); c.join();
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(1), id, error)); SampleResult sample;
    ASSERT_TRUE(readSample(d, id.objectId, frame(1), "linearVelocity", sample, error));
    EXPECT_EQ(SampleState::Carried, sample.state); EXPECT_TRUE(sample.preMembership);
}

TEST(PvdInspection, CrossSceneCarryHasNoComparableAge)
{
    Capture c; c.frame(10); c.join(); c.value(3); c.leave();
    c.writer->createObject(1, c.sceneClass, 900, "other scene"); c.writer->startFrame(900, 20);
    c.scene = 900; c.join();
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(20, 900), id, error)); SampleResult sample;
    ASSERT_TRUE(readSample(d, id.objectId, frame(20, 900), "linearVelocity", sample, error));
    EXPECT_EQ(SampleState::Carried, sample.state); EXPECT_TRUE(sample.preMembership);
    EXPECT_EQ(100u, sample.source.context); EXPECT_EQ(10u, sample.source.recordedFrame);
    EXPECT_FALSE(sample.ageAvailable);
}

TEST(PvdInspection, EvidenceEscapesInvalidContinuationTruncationAndOverlongFourBytes)
{
    const std::pair<std::string, std::string> cases[] = {
        {std::string("a\xC3\"b"), "a\\ufffd\\\"b"},
        {std::string("x\xE2\x82"), "x\\ufffd\\ufffd"},
        {std::string("\xF0\x80\x80\xA2"), "\\ufffd\\ufffd\\ufffd\\ufffd"},
        {std::string("\xF4\x90\x80\x80", 4), "\\ufffd\\ufffd\\ufffd\\ufffd"}
    };
    for (const auto& item : cases)
    {
        ComparisonResult result; result.reason = item.first;
        const auto json = evidenceToJson(result, {}, {});
        EXPECT_NE(std::string::npos, json.find("\"message\":\"" + item.second + "\""));
        EXPECT_NE(std::string::npos, json.find("\"textWasReplaced\":true"));
        EXPECT_EQ(std::string::npos, json.find(item.first));
        for (unsigned char byte : json) EXPECT_LT(byte, 128u);
    }
}

TEST(PvdInspection, UnresolvedSelectorsHaveNullEvidenceIdentities)
{
    Capture a, b(900, 987, false, true); a.frame(1); b.frame(1); a.join(); b.join(); a.value(1); b.value(1);
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    for (bool missingLeft : {false, true})
    {
        auto o = options(); o.endFrame = 1;
        (missingLeft ? o.leftObject : o.rightObject).name = "missing";
        const auto result = compareLinearVelocity(left, right, o);
        EXPECT_EQ(ComparisonStatus::Incomparable, result.status);
        const auto json = evidenceToJson(result, {}, {});
        EXPECT_NE(std::string::npos, json.find(missingLeft ? "\"leftIdentity\":null" : "\"rightIdentity\":null"));
    }
}

TEST(PvdInspection, RemovedConstraintDoesNotAttachReplacementAndSupersededJointStaysDestroyed)
{
    Capture c;
    const auto joints = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(joints, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const uint64_t handle = 801;
    c.writer->createObject(1, joints, 700, "old joint"); c.frame(1);
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    Document d; ASSERT_TRUE(c.load(d)); const auto oldId = d.dom()->mExternalToInternalHandleMap.at(700);
    c.frame(2); c.writer->removeFromUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->createObject(1, joints, 700, "replacement joint");
    ASSERT_TRUE(c.load(d)); EXPECT_TRUE(d.history().at(oldId).object->mWasDestroyed);
    c.frame(3); c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    ASSERT_TRUE(c.load(d)); EXPECT_TRUE(d.history().at(oldId).object->mWasDestroyed);
    EXPECT_FALSE(isObjectAliveAtFrame(d.history().at(oldId).object, 3));
    c.writer->removeFromUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&handle), 8);
    c.writer->setAttribute(1, 700, constraint, reinterpret_cast<const uint8_t*>(&handle), 8);
    ASSERT_TRUE(c.load(d));
    const auto newId = d.dom()->mExternalToInternalHandleMap.at(700);
    EXPECT_TRUE(d.history().at(newId).memberships.empty());
}

TEST(PvdInspection, JointConstraintSetBeforeSceneAddHasSeparateResidencies)
{
    Capture c;
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const auto jointClass = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(jointClass, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const uint64_t jointHandle = 701, constraintHandle = 801;
    c.writer->createObject(1, jointClass, jointHandle, "joint");
    c.frame(1);
    c.writer->setAttribute(1, jointHandle, constraint, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    c.frame(2);
    c.writer->removeFromUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    c.frame(3);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto jointId = d.dom()->mExternalToInternalHandleMap.at(jointHandle);
    const auto& spans = d.history().at(jointId).memberships;
    ASSERT_EQ(2u, spans.size());
    EXPECT_EQ(1u, spans[0].startFrame); EXPECT_EQ(2u, spans[0].endFrame);
    EXPECT_EQ(3u, spans[1].startFrame); EXPECT_EQ(0u, spans[1].stopOrdinal);
    ObjectSelector s; s.objectId = jointId;
    ObjectIdentity selected; std::string error;
    EXPECT_TRUE(selectObject(d, s, frame(1), selected, error)) << error;
    EXPECT_FALSE(selectObject(d, s, frame(2), selected, error));
    EXPECT_TRUE(selectObject(d, s, frame(3), selected, error)) << error;
}

TEST(PvdInspection, ConstraintHandleReuseDoesNotReviveDestroyedJointBeforeReplacementSet)
{
    Capture c;
    const auto constraints = c.writer->registerUniqueListAttribute(c.sceneClass, "constraints", OmniPvdDataType::eOBJECT_HANDLE);
    const auto jointClass = c.writer->registerClass("PxFixedJoint");
    const auto constraint = c.writer->registerAttribute(jointClass, "constraint", OmniPvdDataType::eOBJECT_HANDLE, 1);
    const uint64_t oldJoint = 701, replacementJoint = 702, constraintHandle = 801;
    c.writer->createObject(1, jointClass, oldJoint, "old joint");
    c.frame(1);
    c.writer->setAttribute(1, oldJoint, constraint, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    c.frame(2);
    c.writer->removeFromUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    c.writer->destroyObject(1, oldJoint);
    c.frame(3);
    c.writer->addToUniqueListAttribute(1, c.scene, constraints, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    c.frame(4);
    c.writer->createObject(1, jointClass, replacementJoint, "replacement joint");
    c.writer->setAttribute(1, replacementJoint, constraint, reinterpret_cast<const uint8_t*>(&constraintHandle), 8);
    Document d; ASSERT_TRUE(c.load(d));
    const auto oldId = d.dom()->mExternalToInternalHandleMap.at(oldJoint);
    const auto newId = d.dom()->mExternalToInternalHandleMap.at(replacementJoint);
    const auto& oldSpans = d.history().at(oldId).memberships;
    ASSERT_EQ(1u, oldSpans.size());
    EXPECT_EQ(1u, oldSpans[0].startFrame);
    EXPECT_EQ(2u, oldSpans[0].endFrame);
    ObjectSelector selector; ObjectIdentity selected; std::string error;
    selector.objectId = oldId;
    EXPECT_FALSE(selectObject(d, selector, frame(3), selected, error));
    EXPECT_FALSE(selectObject(d, selector, frame(4), selected, error));
    selector.objectId = newId;
    EXPECT_TRUE(selectObject(d, selector, frame(4), selected, error)) << error;
}

TEST(PvdInspection, MaximumFrameIsRejectedAndFailedDocumentCannotCompareEqual)
{
    Capture c; c.join(); c.frame(UINT64_MAX - 1); c.value(1);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(UINT64_MAX - 1), id, error)) << error;
    c.frame(UINT64_MAX);
    EXPECT_FALSE(c.load(d, Error::Code::Invalid));
    EXPECT_EQ(nullptr, d.dom());
    EXPECT_FALSE(selectObject(d, body(), frame(UINT64_MAX), id, error));
    auto o = options(); o.rightContext = 100; o.startFrame = o.endFrame = UINT64_MAX;
    EXPECT_EQ(ComparisonStatus::Incomparable, compareLinearVelocity(d, d, o).status);
}

TEST(PvdInspection, GlobalResourcesGenericContextsAndReferenceHistoryAreExplicitlyUnsupported)
{
    Capture c; c.join(); c.frame(1);
    const auto material = c.writer->registerClass("PxMaterial"); c.writer->createObject(1, material, 700, "material");
    c.writer->startFrame(999, 1);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    std::vector<ObjectIdentity> objects;
    EXPECT_FALSE(listObjects(d, frame(1, 999), 0, 100, objects, error));
    EXPECT_NE(std::string::npos, error.find("Unsupported"));
    ObjectSelector s;
    for (const auto& entry : d.history()) if (entry.second.object->mOmniAPIHandle == 700) s.objectId = entry.first;
    ASSERT_NE(0u, s.objectId);
    EXPECT_FALSE(selectObject(d, s, frame(1), id, error)); EXPECT_NE(std::string::npos, error.find("Unsupported"));
    s.objectId = 0; s.name = "scene"; ASSERT_TRUE(selectObject(d, s, frame(1), id, error));
    SampleResult sample; EXPECT_FALSE(readSample(d, id.objectId, frame(1), "actors", sample, error));
    EXPECT_NE(std::string::npos, error.find("Unsupported"));
}

TEST(PvdInspection, TypedReadsRejectWrongTypesWidthsAndNonfiniteValues)
{
    OmniPvdAttributeSample storage;
    float v[] = {1, 2, 3, 4}; storage.assignData(reinterpret_cast<const uint8_t*>(v), sizeof(v));
    SampleResult sample; sample.sample = &storage; sample.state = SampleState::Exact; sample.dataType = OmniPvdDataType::eFLOAT32;
    double out[3] = {99, 99, 99}; EXPECT_FALSE(PvdDomInspect::readVec3(sample, out)); EXPECT_EQ(99, out[0]);
    storage.assignData(reinterpret_cast<const uint8_t*>(v), 12); sample.dataType = OmniPvdDataType::eINT32;
    EXPECT_FALSE(PvdDomInspect::readVec3(sample, out)); sample.dataType = OmniPvdDataType::eFLOAT32;
    v[1] = std::numeric_limits<float>::infinity(); storage.assignData(reinterpret_cast<const uint8_t*>(v), 12);
    EXPECT_FALSE(PvdDomInspect::readVec3(sample, out)); EXPECT_EQ(99, out[0]);
    const physx::PxTransform pose(physx::PxVec3(1, 2, 3), physx::PxQuat(0, 0, 0.6f, 0.8f));
    storage.assignData(reinterpret_cast<const uint8_t*>(&pose), sizeof(pose));
    double q[4]; ASSERT_TRUE(PvdDomInspect::readTransform(sample, out, q));
    EXPECT_EQ(1, out[0]); EXPECT_EQ(3, out[2]); EXPECT_NEAR(0.6, q[2], 1e-6);
    double wide[] = {1, 2, 3}; storage.assignData(reinterpret_cast<const uint8_t*>(wide), sizeof(wide));
    sample.dataType = OmniPvdDataType::eFLOAT64; ASSERT_TRUE(PvdDomInspect::readVec3(sample, out)); EXPECT_EQ(3, out[2]);
}

TEST(PvdInspection, InterleavedSceneClocksDoNotUseLegacyClampedTimestamp)
{
    Capture c; c.frame(10); c.value(7);
    c.scene = 900; c.writer->createObject(1, c.sceneClass, c.scene, "second scene"); c.frame(1); c.join(); c.value(8);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(1, 900), id, error));
    SampleResult sample; ASSERT_TRUE(readSample(d, id.objectId, frame(1, 900), "linearVelocity", sample, error));
    EXPECT_EQ(SampleState::Exact, sample.state); EXPECT_EQ(1u, sample.source.recordedFrame);
    EXPECT_EQ(10u, sample.sampleTimestamp); EXPECT_EQ(1u, sample.rawContext);
    double value[3]; ASSERT_TRUE(PvdDomInspect::readVec3(sample, value)); EXPECT_EQ(8, value[0]);
}

TEST(PvdInspection, LateInheritedNameCollisionDoesNotPickFirstAttribute)
{
    Capture c;
    const auto base = c.writer->registerClass("EmptyBase");
    const auto derived = c.writer->registerClass("PxRigidDynamic", base);
    const auto attribute = c.writer->registerAttribute(derived, "linearVelocity", OmniPvdDataType::eFLOAT32, 3);
    ASSERT_NE(OMNI_PVD_INVALID_HANDLE, attribute);
    // Encode the legacy collision directly; the validating writer rejects it.
    const uint8_t command = static_cast<uint8_t>(OmniPvdCommand::eREGISTER_ATTRIBUTE);
    const OmniPvdAttributeHandle duplicate = attribute + 1;
    const uint16_t dataType = static_cast<uint16_t>(OmniPvdDataType::eFLOAT32);
    const uint32_t elements = 3;
    const char name[] = "linearVelocity";
    const uint16_t nameLength = sizeof(name) - 1;
    auto write = [&](const auto& value) {
        c.writeBytes(reinterpret_cast<const uint8_t*>(&value), sizeof(value));
    };
    write(command); write(base); write(duplicate); write(dataType); write(elements); write(nameLength);
    c.writeBytes(reinterpret_cast<const uint8_t*>(name), nameLength);
    c.actorClass = derived; c.create(201, "shadow"); c.join(201); c.frame(1);
    Document d; EXPECT_FALSE(c.load(d, Error::Code::Invalid));
    EXPECT_EQ(nullptr, d.dom());
    ObjectIdentity id; std::string error;
    auto s = body(); s.name = "shadow";
    EXPECT_FALSE(selectObject(d, s, frame(1), id, error));
}

TEST(PvdInspection, ProvenanceAndIndexesHonorRetainedBudgetAndClearOnReloadFailure)
{
    Capture c; c.join();
    for (uint64_t i = 0; i != 32; ++i) { c.frame(i); c.value(float(i)); }
    Document d; ASSERT_TRUE(c.load(d));
    ASSERT_FALSE(d.history().empty()); ASSERT_FALSE(d.frameIndex().empty());
    LoadLimits limits; limits.maxRetainedBytes = d.stats().accountedRetainedBytes;
    const char* path = "inspection-provenance-budget.ovd";
    { std::ofstream file(path, std::ios::binary); file.write(reinterpret_cast<const char*>(c.bytes.data()), c.bytes.size()); }
    Error error; ASSERT_TRUE(d.load(path, limits, error)) << error.message;
    ObjectIdentity id; std::string queryError; SampleResult sample;
    ASSERT_TRUE(selectObject(d, body(), frame(31), id, queryError)) << queryError;
    ASSERT_TRUE(readSample(d, id.objectId, frame(31), "linearVelocity", sample, queryError)) << queryError;
    double value[3]; ASSERT_TRUE(PvdDomInspect::readVec3(sample, value)); EXPECT_EQ(31, value[0]);
    --limits.maxRetainedBytes;
    EXPECT_FALSE(d.load(path, limits, error)); std::remove(path);
    EXPECT_EQ(Error::Code::Limit, error.code);
    EXPECT_EQ(nullptr, d.dom()); EXPECT_TRUE(d.history().empty()); EXPECT_TRUE(d.frameIndex().empty());
    EXPECT_FALSE(readSample(d, id.objectId, frame(31), "linearVelocity", sample, queryError));
}

TEST(PvdInspection, MultiSceneWritesFollowRecordedContextDespiteLegacySceneClock)
{
    Capture c; c.join(); c.frame(3); c.value(1);
    c.writer->createObject(1, c.sceneClass, 900, "other"); c.writer->startFrame(900, 99); c.value(2);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    ASSERT_TRUE(selectObject(d, body(), frame(3), id, error)); SampleResult sample;
    ASSERT_TRUE(readSample(d, id.objectId, frame(3), "linearVelocity", sample, error));
    EXPECT_EQ(SampleState::Exact, sample.state); EXPECT_EQ(3u, sample.source.recordedFrame);
    EXPECT_EQ(99u, sample.sampleTimestamp);
    double value[3]; ASSERT_TRUE(PvdDomInspect::readVec3(sample, value)); EXPECT_EQ(2, value[0]);
}

TEST(PvdInspection, SceneDestructionClosesAllResidenciesAtItsRecordedFrame)
{
    Capture c; c.value(1); c.frame(1); c.join(); c.writer->destroyObject(1, c.scene); c.writer->stopFrame(c.scene, 1);
    Document d; ASSERT_TRUE(c.load(d));
    std::vector<ObjectIdentity> objects; std::string error;
    ASSERT_TRUE(listObjects(d, frame(1), 0, 100, objects, error)); EXPECT_TRUE(objects.empty());
    EXPECT_EQ(2u, d.stats().creates); EXPECT_EQ(2u, d.history().size());
    ObjectIdentity id; EXPECT_FALSE(selectObject(d, body(), frame(1), id, error));
}

TEST(PvdInspection, IndexedLongHistoryFindsFirstChangedFrame)
{
    Capture a, b(900, 987); a.join(); b.join();
    for (unsigned i = 1; i <= 2048; ++i)
    { a.frame(i); a.value(float(i)); b.frame(i); b.value(float(i) + (i == 2048 ? 1 : 0)); }
    Document left, right; ASSERT_TRUE(a.load(left)); ASSERT_TRUE(b.load(right));
    auto o = options(); o.endFrame = 2048;
    const auto result = compareLinearVelocity(left, right, o);
    EXPECT_EQ(ComparisonStatus::Different, result.status) << result.reason;
    EXPECT_EQ(2048u, result.comparedFrames); EXPECT_EQ(2048u, result.leftSample.requested.recordedFrame);
}

TEST(PvdInspection, SharingAnExclusiveShapeClosesItsSceneMembershipImmediately)
{
    Capture c; c.join();
    const auto shapes = c.writer->registerUniqueListAttribute(c.actorClass, "shapes", OmniPvdDataType::eOBJECT_HANDLE);
    const auto shape = c.writer->registerClass("PxShape");
    const auto exclusive = c.writer->registerAttribute(shape, "isExclusive", OmniPvdDataType::eUINT8, 1);
    const uint64_t handle = 700; c.writer->createObject(1, shape, handle, "shape");
    c.frame(1); c.writer->addToUniqueListAttribute(1, c.actor, shapes, reinterpret_cast<const uint8_t*>(&handle), sizeof(handle));
    c.frame(2); const uint8_t shared = 0;
    c.writer->setAttribute(1, handle, exclusive, &shared, 1);
    Document d; ASSERT_TRUE(c.load(d)); ObjectIdentity id; std::string error;
    auto s = body(); s.name = "shape";
    ASSERT_TRUE(selectObject(d, s, frame(1), id, error));
    EXPECT_FALSE(selectObject(d, s, frame(2), id, error));
}
