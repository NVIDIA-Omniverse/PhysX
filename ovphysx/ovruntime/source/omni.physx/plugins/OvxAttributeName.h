// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ovstage/ovx_path_dictionary.h>

namespace omni::physx
{
// The selected Stage headers define which member of the name is authoritative.
// Stage 0.4 changed both the precedence and the invalid-token sentinel.
inline bool attributeUsesToken(const ovx_string_or_token_t& attribute)
{
#if OVSTAGE_VERSION_MAJOR == 0 && OVSTAGE_VERSION_MINOR == 4
    return attribute.string.ptr == nullptr;
#else
    return attribute.token != OVX_INVALID_TOKEN;
#endif
}
} // namespace omni::physx
