// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <ovstage/ovstage_api/ovstage_api_types.h>
#include <ovstage/ovx_path_dictionary.h>
#include <ovx/string_types.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace omni::physx::debug_render {

constexpr int kDebugParamCount = 28;

struct HostColumn
{
    std::vector<uint8_t> bytes;
    DLDataType dtype{};
    int ndim = 1;
    int64_t shape[2]{};
    bool isArray = false;
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
    ovx_token_t attribute = 0;
};

enum class WriteKind
{
    Enabled,
    Scale,
    Param,
    CullingBox,
    Scope,
};

struct WritePlan
{
    WriteKind kind = WriteKind::Scale;
    int param = -1;
    HostColumn column;
};

struct SavedState
{
    uint64_t attach = 0;
    bool enabled = false;
    float scale = 1.0f;
    float params[kDebugParamCount]{};
    bool hasCull = false;
    float cullMin[3]{};
    float cullMax[3]{};
    std::vector<uint64_t> scope;
    bool present = false;
};

uint64_t currentAttach(uint64_t instanceId);
void setAttach(uint64_t instanceId, uint64_t attach, bool clearScope);
SavedState take(uint64_t instanceId);
void applyStored(const SavedState& state);
void applyStoredHere(const SavedState& state);

int writability(const char* name, size_t length);

bool buildRead(uint64_t instanceId,
               uint64_t attach,
               ovx_path_dictionary_t* dictionary,
               const ovx_string_or_token_t* attributes,
               size_t attributeCount,
               std::vector<HostColumn>& columns,
               std::string& error);

bool planWrite(uint64_t instanceId,
               ovx_path_dictionary_t* dictionary,
               const ovx_string_or_token_t* attribute,
               WritePlan& plan,
               std::string& error);

bool commit(uint64_t instanceId, const WritePlan& plan, const DLTensor& tensor, std::string& error);

SavedState copyState(uint64_t instanceId);

} // namespace omni::physx::debug_render
