// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once
#include "PreparedAttribute.h"
#include <carb/logging/Log.h>
#include <array>
#include <tuple>
#include <variant>
#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

namespace omni::physics::parse
{
enum class VehicleLogLevel { error, warning };
struct VehicleDiagnostic
{
    alignas(std::max_align_t) unsigned char arguments[128];
    const char* file = nullptr;
    const char* function = nullptr;
    int line = 0;
    void (*emit)(const VehicleDiagnostic&) = nullptr;
};
namespace vehicle_output_slot
{
inline constexpr size_t angleMultipliers = 0;
inline constexpr size_t averageWheelSpeedRatios = 1;
inline constexpr size_t commandValues = 2;
inline constexpr size_t downRatios = 3;
inline constexpr size_t frictionValues = 4;
inline constexpr size_t materialPaths = 5;
inline constexpr size_t numberOfWheelsPerTrack = 6;
inline constexpr size_t ratios = 7;
inline constexpr size_t speedResponses = 8;
inline constexpr size_t speedResponsesPerCommandValue = 9;
inline constexpr size_t suspensionForceAppPointList = 10;
inline constexpr size_t thrustIndexPerTrack = 11;
inline constexpr size_t tireForceAppPointList = 12;
inline constexpr size_t torqueMultipliers = 13;
inline constexpr size_t torqueRatios = 14;
inline constexpr size_t trackToWheelIndices = 15;
inline constexpr size_t upRatios = 16;
inline constexpr size_t wheelCamberAngleList = 17;
inline constexpr size_t wheelIndicesInTrackOrder = 18;
inline constexpr size_t wheelToeAngleList = 19;
inline constexpr size_t wheels = 20;
}
struct VehicleFillStatus
{
    bool incomplete = false;
    size_t diagnosticCount = 0;
    bool diagnosticsTruncated = false;
    std::array<VehicleDiagnostic, 16> diagnostics;
    std::array<size_t, 32> written{};
    std::array<bool, 32> claimed{};
    void report() const
    {
        for (size_t i = 0; i < diagnosticCount; ++i) diagnostics[i].emit(diagnostics[i]);
        if (diagnosticsTruncated) CARB_LOG_WARN("Usd Physics: further vehicle diagnostics suppressed.");
    }
};
// Diagnostics belong to the current entry and are passed explicitly. No TLS
// lookup (and therefore no first-use dynamic TLS allocation) occurs in fill.
template <VehicleLogLevel Level, typename... Args>
void recordVehicleDiagnostic(VehicleFillStatus& status, const char* file, const char* function, int line,
                             const char* format, Args... args) noexcept
{
    static_assert(((std::is_arithmetic_v<Args> || std::is_pointer_v<Args> || std::is_enum_v<Args>) && ...));
    using Arguments = std::tuple<const char*, Args...>;
    static_assert(sizeof(Arguments) <= sizeof(VehicleDiagnostic::arguments));
    static_assert(alignof(Arguments) <= alignof(VehicleDiagnostic));
    static_assert(std::is_trivially_destructible_v<Arguments>);
    if (status.diagnosticCount == status.diagnostics.size()) { status.diagnosticsTruncated = true; return; }
    VehicleDiagnostic& entry = status.diagnostics[status.diagnosticCount++];
    entry.file = file;
    entry.function = function;
    entry.line = line;
    new (entry.arguments) Arguments(format, args...);
    entry.emit = [](const VehicleDiagnostic& diagnostic)
    {
        std::apply([&](const char* text, Args... values)
        {
            if constexpr (Level == VehicleLogLevel::error) { CARB_LOG_WRITE_EX(carb::logging::kLevelError, text, diagnostic.file, diagnostic.function, diagnostic.line, values...); }
            else { CARB_LOG_WRITE_EX(carb::logging::kLevelWarn, text, diagnostic.file, diagnostic.function, diagnostic.line, values...); }
        }, *reinterpret_cast<const Arguments*>(diagnostic.arguments));
    };
}
// Format strings and pointer arguments must outlive status.report() after join.
// Use literals or storage owned by the prepared input; never local temporaries.
#define VEHICLE_LOG_ERROR(status, ...) ::omni::physics::parse::recordVehicleDiagnostic<::omni::physics::parse::VehicleLogLevel::error>(status, __FILE__, __func__, __LINE__, __VA_ARGS__)
#define VEHICLE_LOG_WARN(status, ...) ::omni::physics::parse::recordVehicleDiagnostic<::omni::physics::parse::VehicleLogLevel::warning>(status, __FILE__, __func__, __LINE__, __VA_ARGS__)

enum class VehicleAttributeType { floating, integer, boolean, float2, float3, token };
struct VehicleAttributeSpec { const char* name; VehicleAttributeType type = VehicleAttributeType::floating; bool authored = false; };
using VehicleAttributeValue = std::variant<PreparedAttribute<float>, PreparedAttribute<int64_t>, PreparedAttribute<bool>,
    PreparedAttribute<carb::Float2>, PreparedAttribute<carb::Float3>, PreparedAttribute<TokenId>>;
struct VehicleAttributeInput
{
    const char* name = nullptr;
    VehicleAttributeValue value;
    bool authored = false;
    std::string tokenText;
};
struct VehiclePathInput { ObjectKey key; std::string path; bool material = false; };
struct PreparedVehicleReads
{
    ObjectKey key;
    std::string ownerPath, empty;
    std::vector<VehicleAttributeInput> attributes;
    std::vector<VehiclePathInput> references;
};
inline PreparedVehicleReads prepareVehicleReads(IPhysicsSource& source, ObjectKey key,
                                                std::initializer_list<VehicleAttributeSpec> specs,
                                                std::string_view instance = {})
{
    PreparedVehicleReads input;
    input.key = key;
    input.ownerPath = source.sourceKeyToString(key);
    input.attributes.reserve(specs.size());
    for (const VehicleAttributeSpec& spec : specs)
    {
        VehicleAttributeInput value;
        value.name = spec.name;
        const bool brakes = std::string_view(spec.name) == "physxVehicleBrakes:maxBrakeTorque";
        const TokenId token = brakes ? source.internToken("physxVehicleBrakes:" + std::string(instance) + ":maxBrakeTorque") : source.internToken(spec.name);
        if (spec.authored) value.authored = source.hasAuthoredAttribute(key, token);
        switch (spec.type)
        {
        case VehicleAttributeType::floating: value.value = prepareAttribute<float>(source, key, token); break;
        case VehicleAttributeType::integer: value.value = prepareAttribute<int64_t>(source, key, token); break;
        case VehicleAttributeType::boolean: value.value = prepareAttribute<bool>(source, key, token); break;
        case VehicleAttributeType::float2: value.value = prepareAttribute<carb::Float2>(source, key, token); break;
        case VehicleAttributeType::float3: value.value = prepareAttribute<carb::Float3>(source, key, token); break;
        case VehicleAttributeType::token:
        {
            PreparedAttribute<TokenId> attribute = prepareAttribute<TokenId>(source, key, token);
            value.value = attribute;
            if (attribute.valid) value.tokenText = source.tokenToString(attribute.value);
            break;
        }
        }
        input.attributes.push_back(std::move(value));
    }
    return input;
}

// A narrow reader over declared typed attributes. It cannot reach a live source,
// allocate, intern names or mutate a cache. Undeclared reads fail explicitly.
class PreparedVehicleReader
{
public:
    PreparedVehicleReader(const PreparedVehicleReads& input, VehicleFillStatus& status) : mInput(input), mStatus(status) {}
    std::string_view attribute(std::string_view name) const noexcept { return name; }
    template <typename T> bool getAttribute(ObjectKey key, std::string_view name, T& out) const noexcept
    {
        const VehicleAttributeInput* value = find(key, name);
        if (!value) return false;
        if (const PreparedAttribute<T>* typed = std::get_if<PreparedAttribute<T>>(&value->value)) return typed->read(out);
        mStatus.incomplete = true;
        return false;
    }
    bool hasAuthoredAttribute(ObjectKey key, std::string_view name) const noexcept
    {
        const VehicleAttributeInput* value = find(key, name);
        return value && value->authored;
    }
    const std::string& sourceKeyToString(ObjectKey key) const noexcept
    {
        if (key == mInput.key) return mInput.ownerPath;
        for (const VehiclePathInput& path : mInput.references) if (path.key == key) return path.path;
        mStatus.incomplete = true;
        return mInput.empty;
    }
    const std::string& tokenToString(TokenId token) const noexcept
    {
        for (const VehicleAttributeInput& value : mInput.attributes)
            if (const PreparedAttribute<TokenId>* attribute = std::get_if<PreparedAttribute<TokenId>>(&value.value))
                if (attribute->valid && attribute->value == token) return value.tokenText;
        mStatus.incomplete = true;
        return mInput.empty;
    }
    bool hasSchema(ObjectKey key, std::string_view name) const noexcept
    {
        if (name == "PhysicsMaterialAPI")
            for (const VehiclePathInput& path : mInput.references) if (path.key == key) return path.material;
        mStatus.incomplete = true;
        return false;
    }
private:
    const VehicleAttributeInput* find(ObjectKey key, std::string_view name) const noexcept
    {
        if (key == mInput.key)
            for (const VehicleAttributeInput& value : mInput.attributes) if (name == value.name) return &value;
        mStatus.incomplete = true;
        return nullptr;
    }
    const PreparedVehicleReads& mInput;
    VehicleFillStatus& mStatus;
};

// Indexed writes preserve existing validation loops without vector growth.
// Final sizes are applied by the coordinator after join.
template <typename T>
class VehicleOutputVector
{
public:
    VehicleOutputVector(std::vector<T>& storage, size_t slot, VehicleFillStatus& status) noexcept : mStorage(storage), mStatus(status), mSlot(slot)
    {
        if (slot >= status.written.size() || status.claimed[slot])
        {
            status.incomplete = true;
            mSlot = status.written.size();
        }
        else status.claimed[slot] = true;
    }
    ~VehicleOutputVector() { if (mSlot < mStatus.written.size()) mStatus.written[mSlot] = mSize; }
    void reserve(size_t count) noexcept { if (count > mStorage.size()) mStatus.incomplete = true; }
    void push_back(const T& value) noexcept
    {
        if (mSize < mStorage.size()) mStorage[mSize++] = value;
        else mStatus.incomplete = true;
    }
    template <typename It> void assign(It begin, It end) noexcept { for (; begin != end; ++begin) push_back(*begin); }
    VehicleOutputVector& operator=(const std::vector<T>& values) noexcept { mSize = 0; assign(values.begin(), values.end()); return *this; }
    VehicleOutputVector& operator=(std::initializer_list<T> values) noexcept { mSize = 0; assign(values.begin(), values.end()); return *this; }
    size_t size() const noexcept { return mSize; }
    bool empty() const noexcept { return !mSize; }
    T& operator[](size_t index) noexcept
    {
        if (index < mSize) return mStorage[index];
        mStatus.incomplete = true;
        return mInvalid;
    }
private:
    std::vector<T>& mStorage;
    VehicleFillStatus& mStatus;
    size_t mSlot, mSize = 0;
    T mInvalid{};
};
}
