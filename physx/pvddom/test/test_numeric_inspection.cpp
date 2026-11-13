// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PvdDomInspectionValue.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace PvdDomInspect;

class PvdNumericInspection : public ::testing::Test
{
protected:
    Document document;
    ObjectIdentity identity;
    FrameRef frame{0, 100, 1};
    std::string error;
    void SetUp() override
    {
        Error loadDiagnostic;
        ASSERT_TRUE(document.load(PVDDOM_NUMERIC_TEST_FILE, LoadLimits(), loadDiagnostic)) << loadDiagnostic.message;
        ObjectSelector selector; selector.name = "numeric-body";
        ASSERT_TRUE(selectObject(document, selector, frame, identity, error)) << error;
    }
    bool read(const char* attribute, SampleResult& sample, InspectionValue& value)
    {
        return readInspectionValue(document, identity.objectId, frame, attribute, sample, value, error);
    }
};

TEST_F(PvdNumericInspection, ScalarBoundariesAndFlags)
{
    struct SignedCase { const char* name; int64_t value; };
    const SignedCase signedCases[] = {{"i8Min", INT8_MIN}, {"i8Max", INT8_MAX}, {"i16Min", INT16_MIN},
        {"i16Max", INT16_MAX}, {"i32Min", INT32_MIN}, {"i32Max", INT32_MAX}, {"i64Min", INT64_MIN}, {"i64Max", INT64_MAX}};
    SampleResult sample; InspectionValue value;
    for (const auto& expected : signedCases)
    {
        ASSERT_TRUE(read(expected.name, sample, value)) << error;
        EXPECT_EQ(InspectionValueKind::SignedScalar, value.kind);
        EXPECT_EQ(expected.value, value.signedValue);
    }
    struct UnsignedCase { const char* name; uint64_t value; };
    const UnsignedCase unsignedCases[] = {{"u8Min", 0}, {"u8Max", UINT8_MAX}, {"u16Min", 0}, {"u16Max", UINT16_MAX},
        {"u32Min", 0}, {"u32Max", UINT32_MAX}, {"u64Min", 0}, {"u64Max", UINT64_MAX},
        {"largeInteger", 9007199254740993ull}, {"flags8", 0x81}, {"flags16", 0x8001}, {"flags32", 0x80000001u}};
    for (const auto& expected : unsignedCases)
    {
        ASSERT_TRUE(read(expected.name, sample, value)) << error;
        EXPECT_EQ(InspectionValueKind::UnsignedScalar, value.kind);
        EXPECT_EQ(expected.value, value.unsignedValue);
    }
    struct FloatCase { const char* name; double value; };
    const FloatCase floatCases[] = {{"f32Max", (std::numeric_limits<float>::max)()},
        {"f32Lowest", (std::numeric_limits<float>::lowest)()}, {"f32Subnormal", std::numeric_limits<float>::denorm_min()},
        {"f64Max", (std::numeric_limits<double>::max)()}, {"f64Lowest", (std::numeric_limits<double>::lowest)()},
        {"f64Subnormal", std::numeric_limits<double>::denorm_min()}, {"f32One", 1}, {"f64One", 1}};
    for (const auto& expected : floatCases)
    {
        ASSERT_TRUE(read(expected.name, sample, value)) << error;
        EXPECT_EQ(InspectionValueKind::FloatingScalar, value.kind);
        EXPECT_EQ(expected.value, value.floating);
    }
    for (const char* name : {"f32NegativeZero", "f64NegativeZero"})
    {
        ASSERT_TRUE(read(name, sample, value)) << error;
        EXPECT_EQ(0.0, value.floating); EXPECT_TRUE(std::signbit(value.floating));
    }
}

TEST_F(PvdNumericInspection, MissingAndTemporalProvenance)
{
    SampleResult sample; InspectionValue value;
    ASSERT_TRUE(read("missingScalar", sample, value));
    EXPECT_EQ(SampleState::Missing, sample.state); EXPECT_EQ(InspectionValueKind::Missing, value.kind);
    EXPECT_EQ(nullptr, sample.sample);
    ASSERT_TRUE(read("preScalar", sample, value));
    EXPECT_EQ(1.5, value.floating); EXPECT_TRUE(sample.preMembership);
    EXPECT_EQ(SampleState::Carried, sample.state); EXPECT_FALSE(sample.hasSourceFrame); EXPECT_FALSE(sample.ageAvailable);
    for (uint64_t n = 1; n <= 3; ++n)
    {
        frame.recordedFrame = n;
        SampleResult original;
        ASSERT_TRUE(readSample(document, identity.objectId, frame, "changing", original, error));
        ASSERT_TRUE(read("changing", sample, value));
        EXPECT_EQ(n == 3 ? 0 : -7, value.signedValue);
        EXPECT_EQ(original.sample, sample.sample); EXPECT_EQ(original.state, sample.state);
        EXPECT_EQ(original.requested.recordedFrame, sample.requested.recordedFrame);
        EXPECT_EQ(original.source.recordedFrame, sample.source.recordedFrame);
        EXPECT_EQ(original.rawContext, sample.rawContext); EXPECT_EQ(original.rawFrame, sample.rawFrame);
        EXPECT_EQ(original.commandOrdinal, sample.commandOrdinal); EXPECT_EQ(original.sampleTimestamp, sample.sampleTimestamp);
        EXPECT_EQ(original.preMembership, sample.preMembership); EXPECT_EQ(original.age, sample.age);
        EXPECT_EQ(n == 2 ? SampleState::Carried : SampleState::Exact, sample.state);
        EXPECT_EQ(n == 2 ? 1u : 0u, sample.age);
    }
}

TEST_F(PvdNumericInspection, LegacyValuesAndProvenance)
{
    SampleResult sample; InspectionValue value;
    for (const char* name : {"vectorValue", "variableVec3"})
    {
        ASSERT_TRUE(read(name, sample, value)) << error;
        EXPECT_EQ(InspectionValueKind::Vec3, value.kind);
        EXPECT_EQ(1, value.components[0]); EXPECT_EQ(3, value.components[2]);
        EXPECT_EQ(SampleState::Exact, sample.state);
    }
    for (const char* name : {"transformValue", "variableTransform"})
    {
        ASSERT_TRUE(read(name, sample, value)) << error;
        EXPECT_EQ(InspectionValueKind::Transform, value.kind);
        EXPECT_EQ(4, value.components[0]); EXPECT_EQ(6, value.components[2]);
        EXPECT_EQ(0, value.quaternion[0]); EXPECT_EQ(1, value.quaternion[3]);
    }
}

TEST_F(PvdNumericInspection, QueryFailuresKeepOutputs)
{
    SampleResult sample; sample.commandOrdinal = 999;
    InspectionValue value; value.kind = InspectionValueKind::SignedScalar; value.signedValue = 123;
    for (const char* name : {"f32NaN", "f32Inf", "f32NegativeInf", "f64NaN", "f64Inf", "f64NegativeInf",
        "shapeTwo", "variableOne", "variableEmpty", "variableIntegers", "textValue", "referenceValue", "listValue", "absent"})
    {
        EXPECT_FALSE(read(name, sample, value)) << name;
        EXPECT_FALSE(error.empty()); EXPECT_EQ(999u, sample.commandOrdinal);
        EXPECT_EQ(InspectionValueKind::SignedScalar, value.kind); EXPECT_EQ(123, value.signedValue);
    }
    EXPECT_FALSE(readInspectionValue(document, UINT64_MAX, frame, "i8Min", sample, value, error));
    frame.recordedFrame = 999;
    EXPECT_FALSE(read("i8Min", sample, value));
    EXPECT_EQ(999u, sample.commandOrdinal); EXPECT_EQ(123, value.signedValue);
}

TEST_F(PvdNumericInspection, DirectGuardsLeaveOutputUnchanged)
{
    OmniPvdAttributeDef declaration;
    declaration.mNbrFields = 1; declaration.mDataType = OmniPvdDataType::eFLOAT32; declaration.mIsUniqueList = 0;
    OmniPvdAttributeSample storage;
    SampleResult sample; sample.sample = &storage; sample.state = SampleState::Exact; sample.dataType = OmniPvdDataType::eFLOAT32;
    InspectionValue value; value.kind = InspectionValueKind::UnsignedScalar; value.unsignedValue = UINT64_MAX;
    value.floating = 99; value.components[0] = 77;
    const auto reject = [&](const char* diagnostic) {
        EXPECT_FALSE(decodeInspectionValue(declaration, sample, value, error));
        EXPECT_NE(std::string::npos, error.find(diagnostic));
        EXPECT_EQ(InspectionValueKind::UnsignedScalar, value.kind); EXPECT_EQ(UINT64_MAX, value.unsignedValue);
        EXPECT_EQ(99, value.floating); EXPECT_EQ(77, value.components[0]);
    };
    const uint8_t bytes[16] = {};
    for (uint32_t size : {0u, 3u, 8u})
    { storage.assignData(bytes, size); reject("payload width"); }
    sample.sample = nullptr; reject("payload width"); sample.sample = &storage;
    for (auto type : {OmniPvdDataType::eFLOAT32, OmniPvdDataType::eFLOAT64})
    {
        declaration.mDataType = type; sample.dataType = type;
        for (double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
        {
            const float single = static_cast<float>(bad);
            if (type == OmniPvdDataType::eFLOAT32) storage.assignData(reinterpret_cast<const uint8_t*>(&single), sizeof(single));
            else storage.assignData(reinterpret_cast<const uint8_t*>(&bad), sizeof(bad));
            reject("Nonfinite");
        }
    }
    for (auto type : {OmniPvdDataType::eSTRING, OmniPvdDataType::eOBJECT_HANDLE, OmniPvdDataType::eENUM_VALUE})
    { declaration.mDataType = type; sample.dataType = type; reject("Unsupported attribute type"); }
    declaration.mDataType = OmniPvdDataType::eFLAGS_WORD; sample.dataType = OmniPvdDataType::eFLAGS_WORD;
    for (uint32_t size : {0u, 3u, 8u})
    { storage.assignData(bytes, size); reject("payload width"); }
    declaration.mDataType = OmniPvdDataType::eINT32; sample.dataType = OmniPvdDataType::eINT32;
    declaration.mNbrFields = 0; storage.assignData(bytes, 4); reject("declaration shape");
    declaration.mNbrFields = 1; sample.dataType = OmniPvdDataType::eFLOAT32; reject("Unsupported attribute type");
}
