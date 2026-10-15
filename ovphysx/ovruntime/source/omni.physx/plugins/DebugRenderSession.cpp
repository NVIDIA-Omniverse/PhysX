// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-DEBUG-RENDER-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-9
 */

#include "DebugRenderSession.h"

#include "PhysXDebugVisualization.h"
#include "usdLoad/LoadUsd.h"

#include <ovstage/ovx_path_dictionary.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <unordered_map>

namespace omni::physx::debug_render {

static_assert(kDebugParamCount == static_cast<int>(ePhysXNumValues),
              "debug-render parameter array must match the PhysX visualization parameter count");
namespace {

constexpr uint32_t kScopeCapacity = 4096;
constexpr int64_t kScopeWriteLength = static_cast<int64_t>(kScopeCapacity) + 1;
constexpr int64_t kCullingWriteLength = 7;

enum class Kind
{
    Enabled,
    Scale,
    Param,
    CullingBox,
    Scope,
    PointPosition,
    PointColor,
    LinePosition,
    LineColor,
    TrianglePosition,
    TriangleColor,
};

struct Attr
{
    const char* name;
    size_t nameLength;
    Kind kind;
    int param;
    int access;
};

#define OVPHYSX_DEBUG_ATTR(literal, kind, param, access) { literal, sizeof(literal) - 1, kind, param, access }

const Attr kAttrs[] = {
    OVPHYSX_DEBUG_ATTR("enabled", Kind::Enabled, -1, 1),
    OVPHYSX_DEBUG_ATTR("scale", Kind::Scale, -1, 1),
    OVPHYSX_DEBUG_ATTR("worldAxes", Kind::Param, eWorldAxes, 1),
    OVPHYSX_DEBUG_ATTR("bodyAxes", Kind::Param, eBodyAxes, 1),
    OVPHYSX_DEBUG_ATTR("bodyMassAxes", Kind::Param, eBodyMassAxes, 1),
    OVPHYSX_DEBUG_ATTR("bodyLinearVelocity", Kind::Param, eBodyLinearVelocity, 1),
    OVPHYSX_DEBUG_ATTR("bodyAngularVelocity", Kind::Param, eBodyAngularVelocity, 1),
    OVPHYSX_DEBUG_ATTR("contactPoint", Kind::Param, eContactPoint, 1),
    OVPHYSX_DEBUG_ATTR("contactNormal", Kind::Param, eContactNormal, 1),
    OVPHYSX_DEBUG_ATTR("contactError", Kind::Param, eContactError, 1),
    OVPHYSX_DEBUG_ATTR("contactImpulse", Kind::Param, eContactImpulse, 1),
    OVPHYSX_DEBUG_ATTR("frictionPoint", Kind::Param, eFrictionPoint, 1),
    OVPHYSX_DEBUG_ATTR("frictionNormal", Kind::Param, eFrictionNormal, 1),
    OVPHYSX_DEBUG_ATTR("frictionImpulse", Kind::Param, eFrictionImpulse, 1),
    OVPHYSX_DEBUG_ATTR("actorAxes", Kind::Param, eActorAxes, 1),
    OVPHYSX_DEBUG_ATTR("collisionAABBs", Kind::Param, eCollisionAABBs, 1),
    OVPHYSX_DEBUG_ATTR("collisionShapes", Kind::Param, eCollisionShapes, 1),
    OVPHYSX_DEBUG_ATTR("collisionAxes", Kind::Param, eCollisionAxes, 1),
    OVPHYSX_DEBUG_ATTR("collisionCompounds", Kind::Param, eCollisionCompounds, 1),
    OVPHYSX_DEBUG_ATTR("collisionFaceNormals", Kind::Param, eCollisionFaceNormals, 1),
    OVPHYSX_DEBUG_ATTR("collisionEdges", Kind::Param, eCollisionEdges, 1),
    OVPHYSX_DEBUG_ATTR("collisionStaticPruner", Kind::Param, eCollisionStaticPruner, 1),
    OVPHYSX_DEBUG_ATTR("collisionDynamicPruner", Kind::Param, eCollisionDynamicPruner, 1),
    OVPHYSX_DEBUG_ATTR("jointLocalFrames", Kind::Param, eJointLocalFrames, 1),
    OVPHYSX_DEBUG_ATTR("jointLimits", Kind::Param, eJointLimits, 1),
    OVPHYSX_DEBUG_ATTR("cullBox", Kind::Param, eCullBox, 1),
    OVPHYSX_DEBUG_ATTR("mbpRegions", Kind::Param, eMBPRegions, 1),
    OVPHYSX_DEBUG_ATTR("simulationMesh", Kind::Param, eSimulationMesh, 1),
    OVPHYSX_DEBUG_ATTR("sdf", Kind::Param, eSDF, 1),
    OVPHYSX_DEBUG_ATTR("cullingBox", Kind::CullingBox, -1, 1),
    OVPHYSX_DEBUG_ATTR("scope", Kind::Scope, -1, 1),
    OVPHYSX_DEBUG_ATTR("pointPosition", Kind::PointPosition, -1, 4),
    OVPHYSX_DEBUG_ATTR("pointColor", Kind::PointColor, -1, 4),
    OVPHYSX_DEBUG_ATTR("linePosition", Kind::LinePosition, -1, 4),
    OVPHYSX_DEBUG_ATTR("lineColor", Kind::LineColor, -1, 4),
    OVPHYSX_DEBUG_ATTR("trianglePosition", Kind::TrianglePosition, -1, 4),
    OVPHYSX_DEBUG_ATTR("triangleColor", Kind::TriangleColor, -1, 4),
};

#undef OVPHYSX_DEBUG_ATTR

std::mutex g_mutex;
std::unordered_map<uint64_t, SavedState> g_states;

bool nameEquals(const char* literal, const char* name, size_t length)
{
    size_t i = 0;
    for (; literal[i] != '\0'; ++i)
    {
        if (i >= length || literal[i] != name[i])
            return false;
    }
    return i == length;
}

const Attr* findByString(const char* name, size_t length)
{
    if (!name || length == 0)
        return nullptr;
    for (const Attr& attr : kAttrs)
        if (nameEquals(attr.name, name, length))
            return &attr;
    return nullptr;
}

const Attr* findAttr(const ovx_string_or_token_t* attribute, ovx_path_dictionary_t* dictionary)
{
    if (!attribute)
        return nullptr;
    if (attribute->string.ptr && attribute->string.length > 0)
        return findByString(attribute->string.ptr, attribute->string.length);
    if (attribute->token == 0 || !dictionary)
        return nullptr;
    ovx_string_t resolved{};
    if (ovx_path_dictionary_token_to_string(dictionary, attribute->token, &resolved) != OVX_OK || !resolved.ptr)
        return nullptr;
    return findByString(resolved.ptr, resolved.length);
}

std::string attributeLabel(const ovx_string_or_token_t* attribute)
{
    if (attribute && attribute->string.ptr && attribute->string.length > 0)
        return std::string(attribute->string.ptr, attribute->string.length);
    if (attribute && attribute->token != 0)
        return "token " + std::to_string(attribute->token);
    return "empty name";
}

ovx_token_t internAttributeName(ovx_path_dictionary_t* dictionary, const char* literal, size_t length)
{
    if (!dictionary || !literal || length == 0)
        return 0;
    ovx_token_t token = 0;
    const ovx_string_t name{ literal, length };
    if (ovx_path_dictionary_intern_token(dictionary, name, &token) != OVX_OK)
        return 0;
    return token;
}

bool isGeometry(Kind kind)
{
    return kind == Kind::PointPosition || kind == Kind::PointColor || kind == Kind::LinePosition ||
           kind == Kind::LineColor || kind == Kind::TrianglePosition || kind == Kind::TriangleColor;
}

DLDataType floatType(uint16_t lanes)
{
    DLDataType dtype{};
    dtype.code = kDLFloat;
    dtype.bits = 32;
    dtype.lanes = lanes;
    return dtype;
}

DLDataType uintType(uint16_t bits, uint16_t lanes)
{
    DLDataType dtype{};
    dtype.code = kDLUInt;
    dtype.bits = bits;
    dtype.lanes = lanes;
    return dtype;
}

SavedState& stateFor(uint64_t instanceId)
{
    SavedState& state = g_states[instanceId];
    state.present = true;
    return state;
}

void fillScalar(HostColumn& column, DLDataType dtype, const void* bytes, size_t size)
{
    column.bytes.assign(size, 0);
    if (bytes && size)
        std::memcpy(column.bytes.data(), bytes, size);
    column.dtype = dtype;
    column.ndim = 1;
    column.shape[0] = 1;
    column.isArray = false;
    column.semantic = OVSTAGE_SEMANTIC_NONE;
}

void fillGeometry(HostColumn& column, Kind kind, const std::vector<DebugPoint>& points,
                  const std::vector<DebugLine>& lines, const std::vector<DebugTriangle>& triangles)
{
    if (kind == Kind::PointPosition || kind == Kind::LinePosition || kind == Kind::TrianglePosition)
    {
        const bool line = kind == Kind::LinePosition;
        const size_t count = kind == Kind::PointPosition ? points.size() : line ? lines.size() : triangles.size();
        const int verts = kind == Kind::PointPosition ? 1 : line ? 2 : 3;
        column.bytes.assign(count * static_cast<size_t>(verts) * 3 * sizeof(float), 0);
        float* dst = reinterpret_cast<float*>(column.bytes.data());
        for (size_t i = 0; i < count; ++i)
        {
            if (kind == Kind::PointPosition)
            {
                std::memcpy(dst, &points[i].mPos, 3 * sizeof(float));
                dst += 3;
            }
            else if (line)
            {
                std::memcpy(dst, &lines[i].mPos0, 3 * sizeof(float));
                std::memcpy(dst + 3, &lines[i].mPos1, 3 * sizeof(float));
                dst += 6;
            }
            else
            {
                std::memcpy(dst, &triangles[i].mPos0, 3 * sizeof(float));
                std::memcpy(dst + 3, &triangles[i].mPos1, 3 * sizeof(float));
                std::memcpy(dst + 6, &triangles[i].mPos2, 3 * sizeof(float));
                dst += 9;
            }
        }
        column.dtype = floatType(3);
        column.ndim = kind == Kind::PointPosition ? 1 : 2;
        column.shape[0] = static_cast<int64_t>(count);
        column.shape[1] = verts;
        column.semantic = OVSTAGE_SEMANTIC_POINT;
    }
    else
    {
        const size_t count = kind == Kind::PointColor ? points.size()
                            : kind == Kind::LineColor ? lines.size() : triangles.size();
        const int colors = kind == Kind::PointColor ? 1 : kind == Kind::LineColor ? 2 : 3;
        column.bytes.assign(count * static_cast<size_t>(colors) * sizeof(uint32_t), 0);
        uint32_t* dst = reinterpret_cast<uint32_t*>(column.bytes.data());
        for (size_t i = 0; i < count; ++i)
        {
            if (kind == Kind::PointColor)
                dst[i] = points[i].mColor;
            else if (kind == Kind::LineColor)
            {
                dst[i * 2] = lines[i].mColor0;
                dst[i * 2 + 1] = lines[i].mColor1;
            }
            else
            {
                dst[i * 3] = triangles[i].mColor0;
                dst[i * 3 + 1] = triangles[i].mColor1;
                dst[i * 3 + 2] = triangles[i].mColor2;
            }
        }
        column.dtype = uintType(32, static_cast<uint16_t>(colors));
        column.ndim = 1;
        column.shape[0] = static_cast<int64_t>(count);
        column.semantic = OVSTAGE_SEMANTIC_COLOR;
    }
    column.isArray = true;
}

void fillSetting(HostColumn& column, const Attr& attr, const SavedState& state)
{
    if (attr.kind == Kind::Enabled)
    {
        const uint8_t value = state.enabled ? 1 : 0;
        fillScalar(column, uintType(8, 1), &value, sizeof(value));
    }
    else if (attr.kind == Kind::Scale)
        fillScalar(column, floatType(1), &state.scale, sizeof(float));
    else if (attr.kind == Kind::Param)
        fillScalar(column, floatType(1), &state.params[attr.param], sizeof(float));
    else if (attr.kind == Kind::CullingBox)
    {
        column.bytes.assign(6 * sizeof(float), 0);
        if (state.hasCull)
        {
            float* dst = reinterpret_cast<float*>(column.bytes.data());
            std::memcpy(dst, state.cullMin, 3 * sizeof(float));
            std::memcpy(dst + 3, state.cullMax, 3 * sizeof(float));
        }
        column.dtype = floatType(6);
        column.ndim = 1;
        column.shape[0] = state.hasCull ? 1 : 0;
        if (!state.hasCull)
            column.bytes.clear();
    }
    else if (attr.kind == Kind::Scope)
    {
        column.bytes.assign(state.scope.size() * sizeof(uint64_t), 0);
        if (!column.bytes.empty())
            std::memcpy(column.bytes.data(), state.scope.data(), column.bytes.size());
        column.dtype = uintType(64, 1);
        column.ndim = 1;
        column.shape[0] = static_cast<int64_t>(state.scope.size());
        column.isArray = true;
    }
}

bool tensorShapeIs(const DLTensor& tensor, int ndim, int64_t shape0, uint8_t code, uint8_t bits, uint16_t lanes)
{
    if (tensor.ndim != ndim || !tensor.shape || (!tensor.data && shape0 != 0))
        return false;
    if (tensor.dtype.code != code || tensor.dtype.bits != bits || tensor.dtype.lanes != lanes)
        return false;
    if (tensor.device.device_type != kDLCPU)
        return false;
    return tensor.shape[0] == shape0;
}

bool finiteNonNegative(float value)
{
    return std::isfinite(value) && value >= 0.0f;
}

WriteKind toWriteKind(Kind kind)
{
    switch (kind)
    {
    case Kind::Enabled: return WriteKind::Enabled;
    case Kind::Scale: return WriteKind::Scale;
    case Kind::Param: return WriteKind::Param;
    case Kind::CullingBox: return WriteKind::CullingBox;
    case Kind::Scope: return WriteKind::Scope;
    default: return WriteKind::Scale;
    }
}

} // namespace

uint64_t currentAttach(uint64_t instanceId)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto found = g_states.find(instanceId);
    return found == g_states.end() ? 0 : found->second.attach;
}

void setAttach(uint64_t instanceId, uint64_t attach, bool clearScope)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto found = g_states.find(instanceId);
    // A detach notification after forget must not recreate the entry that was just erased.
    if (found == g_states.end())
    {
        if (attach == 0)
            return;
        SavedState& state = g_states[instanceId];
        state.attach = attach;
        return;
    }
    found->second.attach = attach;
    if (clearScope)
        found->second.scope.clear();
}

SavedState take(uint64_t instanceId)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto found = g_states.find(instanceId);
    if (found == g_states.end())
        return {};
    SavedState state = std::move(found->second);
    g_states.erase(found);
    return state;
}

void applyOwned(usdparser::AttachedStage& owner, const SavedState& state)
{
    applyDebugRenderToOwner(owner, state.enabled, state.scale, state.params,
                            static_cast<uint32_t>(kDebugParamCount), state.hasCull, state.cullMin, state.cullMax,
                            state.scope.empty() ? nullptr : reinterpret_cast<const ovx_primpath_t*>(state.scope.data()),
                            static_cast<uint32_t>(state.scope.size()));
}

void applyStored(const SavedState& state)
{
    if (!state.present || state.attach == 0)
        return;
    applyDebugRenderForAttach(state.attach, state.enabled, state.scale, state.params,
                              static_cast<uint32_t>(kDebugParamCount), state.hasCull, state.cullMin, state.cullMax,
                              state.scope.empty() ? nullptr : reinterpret_cast<const ovx_primpath_t*>(state.scope.data()),
                              static_cast<uint32_t>(state.scope.size()));
}

void applyStoredHere(const SavedState& state)
{
    if (!state.present || state.attach == 0)
        return;
    usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(state.attach);
    if (owner)
        applyOwned(*owner, state);
}

SavedState copyState(uint64_t instanceId)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto found = g_states.find(instanceId);
    if (found == g_states.end())
        return {};
    return found->second;
}

int writability(const char* name, size_t length)
{
    if (!name)
        return 0;
    for (const Attr& attr : kAttrs)
        if (nameEquals(attr.name, name, length))
            return attr.access;
    return 0;
}

bool buildRead(uint64_t instanceId,
               uint64_t attach,
               ovx_path_dictionary_t* dictionary,
               const ovx_string_or_token_t* attributes,
               size_t attributeCount,
               std::vector<HostColumn>& columns,
               std::string& error)
{
    struct Selected
    {
        const Attr* attr = nullptr;
        ovx_token_t token = 0;
    };
    std::vector<Selected> selected;
    selected.reserve(attributeCount);
    bool wantGeometry = false;
    for (size_t i = 0; i < attributeCount; ++i)
    {
        const Attr* attr = findAttr(&attributes[i], dictionary);
        if (!attr)
        {
            const bool tokenOnly = attributes && attributes[i].token != 0 &&
                                   !(attributes[i].string.ptr && attributes[i].string.length > 0);
            if (tokenOnly && !dictionary)
                error = "ovphysx_read: debug-render attribute token cannot be resolved without an attached stage";
            else
                error = "ovphysx_read: unknown debug-render attribute " + attributeLabel(&attributes[i]);
            return false;
        }
        ovx_token_t token = attributes[i].token;
        if (token == 0)
            token = internAttributeName(dictionary, attr->name, attr->nameLength);
        selected.push_back(Selected{ attr, token });
        wantGeometry = wantGeometry || isGeometry(attr->kind);
    }

    std::vector<DebugPoint> points;
    std::vector<DebugLine> lines;
    std::vector<DebugTriangle> triangles;
    if (wantGeometry && attach != 0)
    {
        usdparser::AttachedStage* owner = usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach);
        if (!owner)
        {
            error = "ovphysx_read: debug-render geometry changed while it was being copied";
            return false;
        }
        copyDebugPrimitivesFromOwner(*owner, points, lines, triangles);
    }

    SavedState state;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        const auto found = g_states.find(instanceId);
        if (found != g_states.end())
            state = found->second;
    }
    columns.clear();
    columns.reserve(selected.size());
    for (const Selected& item : selected)
    {
        columns.emplace_back();
        HostColumn& column = columns.back();
        if (isGeometry(item.attr->kind))
            fillGeometry(column, item.attr->kind, points, lines, triangles);
        else
            fillSetting(column, *item.attr, state);
        column.attribute = item.token;
    }
    return true;
}

bool planWrite(uint64_t instanceId,
               ovx_path_dictionary_t* dictionary,
               const ovx_string_or_token_t* attribute,
               WritePlan& plan,
               std::string& error)
{
    const Attr* attr = findAttr(attribute, dictionary);
    if (!attr || attr->access != 1)
    {
        error = "ovphysx_write: attribute is not writable on OVPHYSX_QUERY_TARGET_DEBUG_RENDER (" +
                attributeLabel(attribute) + ")";
        return false;
    }
    SavedState state;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        state = stateFor(instanceId);
    }
    plan.kind = toWriteKind(attr->kind);
    plan.param = attr->param;
    if (attr->kind == Kind::Scope)
    {
        plan.column.bytes.assign(static_cast<size_t>(kScopeWriteLength) * sizeof(uint64_t), 0);
        auto* slots = reinterpret_cast<uint64_t*>(plan.column.bytes.data());
        const size_t count = std::min(state.scope.size(), static_cast<size_t>(kScopeCapacity));
        slots[0] = static_cast<uint64_t>(count);
        if (count)
            std::memcpy(slots + 1, state.scope.data(), count * sizeof(uint64_t));
        plan.column.dtype = uintType(64, 1);
        plan.column.ndim = 1;
        plan.column.shape[0] = kScopeWriteLength;
    }
    else if (attr->kind == Kind::CullingBox)
    {
        plan.column.bytes.assign(static_cast<size_t>(kCullingWriteLength) * sizeof(float), 0);
        auto* row = reinterpret_cast<float*>(plan.column.bytes.data());
        row[0] = state.hasCull ? 1.0f : 0.0f;
        if (state.hasCull)
        {
            std::memcpy(row + 1, state.cullMin, 3 * sizeof(float));
            std::memcpy(row + 4, state.cullMax, 3 * sizeof(float));
        }
        plan.column.dtype = floatType(1);
        plan.column.ndim = 1;
        plan.column.shape[0] = kCullingWriteLength;
    }
    else
        fillSetting(plan.column, *attr, state);
    return true;
}

bool commit(uint64_t instanceId, const WritePlan& plan, const DLTensor& tensor, std::string& error)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    SavedState& state = stateFor(instanceId);
    if (plan.kind == WriteKind::Enabled)
    {
        if (!tensorShapeIs(tensor, 1, 1, kDLUInt, 8, 1))
        {
            error = "ovphysx_commit_group: enabled must be uint8 shape [1]";
            return false;
        }
        state.enabled = *static_cast<const uint8_t*>(tensor.data) != 0;
    }
    else if (plan.kind == WriteKind::Scale || plan.kind == WriteKind::Param)
    {
        if (!tensorShapeIs(tensor, 1, 1, kDLFloat, 32, 1))
        {
            error = "ovphysx_commit_group: debug-render scalar must be float32 shape [1]";
            return false;
        }
        const float value = *static_cast<const float*>(tensor.data);
        if (!finiteNonNegative(value))
        {
            error = "ovphysx_commit_group: debug-render value must be finite and >= 0";
            return false;
        }
        if (plan.kind == WriteKind::Scale)
            state.scale = value;
        else
            state.params[plan.param] = value;
    }
    else if (plan.kind == WriteKind::CullingBox)
    {
        if (!tensorShapeIs(tensor, 1, kCullingWriteLength, kDLFloat, 32, 1))
        {
            error = "ovphysx_commit_group: cullingBox write must be float32 shape [7]";
            return false;
        }
        const float* row = static_cast<const float*>(tensor.data);
        if (row[0] == 0.0f)
            state.hasCull = false;
        else if (row[0] == 1.0f)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                if (!std::isfinite(row[axis + 1]) || !std::isfinite(row[axis + 4]) || row[axis + 1] > row[axis + 4])
                {
                    error = "ovphysx_commit_group: cullingBox min/max must be finite and min <= max";
                    return false;
                }
            }
            std::memcpy(state.cullMin, row + 1, 3 * sizeof(float));
            std::memcpy(state.cullMax, row + 4, 3 * sizeof(float));
            state.hasCull = true;
        }
        else
        {
            error = "ovphysx_commit_group: cullingBox element 0 must be 0 to clear or 1 to store";
            return false;
        }
    }
    else
    {
        if (!tensorShapeIs(tensor, 1, kScopeWriteLength, kDLUInt, 64, 1))
        {
            error = "ovphysx_commit_group: scope write must be uint64 shape [4097]";
            return false;
        }
        const auto* slots = static_cast<const uint64_t*>(tensor.data);
        const uint64_t count = slots[0];
        if (count > kScopeCapacity)
        {
            error = "ovphysx_commit_group: scope length exceeds the 4096 token capacity";
            return false;
        }
        for (uint64_t i = 0; i < count; ++i)
        {
            if (slots[i + 1] == 0)
            {
                error = "ovphysx_commit_group: scope tokens must be non-zero";
                return false;
            }
        }
        state.scope.assign(slots + 1, slots + 1 + count);
    }
    return true;
}

} // namespace omni::physx::debug_render
