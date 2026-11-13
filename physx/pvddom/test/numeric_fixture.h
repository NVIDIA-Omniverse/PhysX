// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "OmniPvdWriter.h"
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace PvdDomTest
{
inline bool numericFixtureOption(const char* option)
{
    const char* modes[] = {"--numeric-values", "--numeric-fixed-empty", "--numeric-fixed-short",
        "--numeric-fixed-long", "--numeric-flags-empty", "--numeric-flags-three", "--numeric-flags-eight"};
    for (const auto* mode : modes) if (std::strcmp(option, mode) == 0) return true;
    return false;
}

struct NumericWrite
{
    OmniPvdAttributeHandle attribute;
    std::vector<uint8_t> bytes;
};
inline void numericBytes(std::vector<NumericWrite>& writes, OmniPvdAttributeHandle attribute,
                         const void* data, uint32_t size)
{
    NumericWrite write;
    write.attribute = attribute;
    write.bytes.resize(size);
    if (size) std::memcpy(write.bytes.data(), data, size);
    writes.push_back(write);
}
template<class T> void numericScalar(OmniPvdWriter& writer, OmniPvdClassHandle cls,
    std::vector<NumericWrite>& writes, const char* name, OmniPvdDataType::Enum type, T value)
{
    const auto attribute = writer.registerAttribute(cls, name, type, 1);
    numericBytes(writes, attribute, &value, sizeof(value));
}
template<class T> void numericBounds(OmniPvdWriter& writer, OmniPvdClassHandle cls,
    std::vector<NumericWrite>& writes, const char* prefix, OmniPvdDataType::Enum type)
{
    numericScalar(writer, cls, writes, (std::string(prefix) + "Min").c_str(), type, (std::numeric_limits<T>::min)());
    numericScalar(writer, cls, writes, (std::string(prefix) + "Max").c_str(), type, (std::numeric_limits<T>::max)());
}
template<class T> void numericFloats(OmniPvdWriter& writer, OmniPvdClassHandle cls,
    std::vector<NumericWrite>& writes, const char* prefix, OmniPvdDataType::Enum type)
{
    const auto add = [&](const char* suffix, T value) {
        numericScalar(writer, cls, writes, (std::string(prefix) + suffix).c_str(), type, value);
    };
    add("Value", T(1.25)); add("One", T(1)); add("Zero", T(0)); add("NegativeZero", -T(0));
    add("Max", (std::numeric_limits<T>::max)()); add("Lowest", (std::numeric_limits<T>::lowest)());
    add("Subnormal", std::numeric_limits<T>::denorm_min());
    add("NaN", std::numeric_limits<T>::quiet_NaN());
    add("Inf", std::numeric_limits<T>::infinity()); add("NegativeInf", -std::numeric_limits<T>::infinity());
}

inline void writeNumericFixture(OmniPvdWriter& writer, const char* mode)
{
    const OmniPvdContextHandle context = 100;
    const OmniPvdObjectHandle scene = 100, body = 400;
    const auto sceneClass = writer.registerClass("PxScene");
    const auto actors = writer.registerUniqueListAttribute(sceneClass, "actors", OmniPvdDataType::eOBJECT_HANDLE);
    const auto actorClass = writer.registerClass("PxActor");
    const auto actorType = writer.registerAttribute(actorClass, "type", OmniPvdDataType::eUINT32, 1);
    const auto rigidClass = writer.registerClass("PxRigidActor", actorClass);
    const auto cls = writer.registerClass("PxRigidDynamic", rigidClass);
    const auto flagsClass = writer.registerClass("NumericFlags");
    writer.registerEnumValue(flagsClass, "lowBit", 1);
    std::vector<NumericWrite> writes;
    numericBounds<int8_t>(writer, cls, writes, "i8", OmniPvdDataType::eINT8);
    numericBounds<int16_t>(writer, cls, writes, "i16", OmniPvdDataType::eINT16);
    numericBounds<int32_t>(writer, cls, writes, "i32", OmniPvdDataType::eINT32);
    numericBounds<int64_t>(writer, cls, writes, "i64", OmniPvdDataType::eINT64);
    numericBounds<uint8_t>(writer, cls, writes, "u8", OmniPvdDataType::eUINT8);
    numericBounds<uint16_t>(writer, cls, writes, "u16", OmniPvdDataType::eUINT16);
    numericBounds<uint32_t>(writer, cls, writes, "u32", OmniPvdDataType::eUINT32);
    numericBounds<uint64_t>(writer, cls, writes, "u64", OmniPvdDataType::eUINT64);
    numericScalar(writer, cls, writes, "largeInteger", OmniPvdDataType::eUINT64, uint64_t(9007199254740993ull));
    numericFloats<float>(writer, cls, writes, "f32", OmniPvdDataType::eFLOAT32);
    numericFloats<double>(writer, cls, writes, "f64", OmniPvdDataType::eFLOAT64);
    const uint8_t flags8 = 0x81;
    const uint16_t flags16 = 0x8001;
    const uint32_t flags32 = 0x80000001u;
    numericBytes(writes, writer.registerFlagsAttribute(cls, "flags8", flagsClass), &flags8, sizeof(flags8));
    numericBytes(writes, writer.registerFlagsAttribute(cls, "flags16", flagsClass), &flags16, sizeof(flags16));
    numericBytes(writes, writer.registerFlagsAttribute(cls, "flags32", flagsClass), &flags32, sizeof(flags32));
    writer.registerAttribute(cls, "missingScalar", OmniPvdDataType::eFLOAT32, 1);
    const auto preScalar = writer.registerAttribute(cls, "preScalar", OmniPvdDataType::eFLOAT32, 1);
    const auto changing = writer.registerAttribute(cls, "changing", OmniPvdDataType::eINT32, 1);
    const float vector[] = {1, 2, 3};
    const double transform[] = {0, 0, 0, 1, 4, 5, 6};
    numericBytes(writes, writer.registerAttribute(cls, "vectorValue", OmniPvdDataType::eFLOAT32, 3), vector, sizeof(vector));
    numericBytes(writes, writer.registerAttribute(cls, "transformValue", OmniPvdDataType::eFLOAT64, 7), transform, sizeof(transform));
    numericBytes(writes, writer.registerAttribute(cls, "shapeTwo", OmniPvdDataType::eFLOAT32, 2), vector, 2 * sizeof(float));
    numericBytes(writes, writer.registerAttribute(cls, "variableVec3", OmniPvdDataType::eFLOAT32, 0), vector, sizeof(vector));
    numericBytes(writes, writer.registerAttribute(cls, "variableTransform", OmniPvdDataType::eFLOAT64, 0), transform, sizeof(transform));
    numericBytes(writes, writer.registerAttribute(cls, "variableOne", OmniPvdDataType::eFLOAT32, 0), vector, sizeof(float));
    numericBytes(writes, writer.registerAttribute(cls, "variableEmpty", OmniPvdDataType::eFLOAT32, 0), vector, 0);
    const uint32_t integers[] = {1, 2, 3};
    numericBytes(writes, writer.registerAttribute(cls, "variableIntegers", OmniPvdDataType::eUINT32, 0), integers, sizeof(integers));
    const char text[] = "recorded text";
    numericBytes(writes, writer.registerAttribute(cls, "textValue", OmniPvdDataType::eSTRING, 1), text, sizeof(text));
    numericBytes(writes, writer.registerAttribute(cls, "referenceValue", OmniPvdDataType::eOBJECT_HANDLE, 1), &body, sizeof(body));
    const auto list = writer.registerUniqueListAttribute(cls, "listValue", OmniPvdDataType::eOBJECT_HANDLE);
    const auto badNumeric = writer.registerAttribute(cls, "badNumeric", OmniPvdDataType::eFLOAT32, 1);
    const auto badFlags = writer.registerFlagsAttribute(cls, "badFlags", flagsClass);

    writer.createObject(1, sceneClass, scene, "numeric-scene");
    writer.createObject(1, cls, body, "numeric-body");
    const uint32_t dynamicType = 1;
    writer.setAttribute(1, body, actorType, reinterpret_cast<const uint8_t*>(&dynamicType), sizeof(dynamicType));
    const float preValue = 1.5f;
    writer.setAttribute(1, body, preScalar, reinterpret_cast<const uint8_t*>(&preValue), sizeof(preValue));
    writer.addToUniqueListAttribute(1, scene, actors, reinterpret_cast<const uint8_t*>(&body), sizeof(body));
    writer.startFrame(context, 1);
    const uint8_t empty = 0;
    for (const auto& write : writes)
        writer.setAttribute(context, body, write.attribute, write.bytes.empty() ? &empty : write.bytes.data(), uint32_t(write.bytes.size()));
    const int32_t first = -7;
    writer.setAttribute(context, body, changing, reinterpret_cast<const uint8_t*>(&first), sizeof(first));
    writer.addToUniqueListAttribute(context, body, list, reinterpret_cast<const uint8_t*>(&body), sizeof(body));

    // Malformed writes precede complete later frames, so they are not recoverable tails.
    const uint8_t invalid[8] = {};
    if (std::strcmp(mode, "--numeric-fixed-empty") == 0) writer.setAttribute(context, body, badNumeric, invalid, 0);
    if (std::strcmp(mode, "--numeric-fixed-short") == 0) writer.setAttribute(context, body, badNumeric, invalid, 3);
    if (std::strcmp(mode, "--numeric-fixed-long") == 0) writer.setAttribute(context, body, badNumeric, invalid, 8);
    if (std::strcmp(mode, "--numeric-flags-empty") == 0) writer.setAttribute(context, body, badFlags, invalid, 0);
    if (std::strcmp(mode, "--numeric-flags-three") == 0) writer.setAttribute(context, body, badFlags, invalid, 3);
    if (std::strcmp(mode, "--numeric-flags-eight") == 0) writer.setAttribute(context, body, badFlags, invalid, 8);
    writer.stopFrame(context, 1);
    writer.startFrame(context, 2); writer.stopFrame(context, 2);
    writer.startFrame(context, 3);
    const int32_t zero = 0;
    writer.setAttribute(context, body, changing, reinterpret_cast<const uint8_t*>(&zero), sizeof(zero));
    writer.stopFrame(context, 3);
}
}
