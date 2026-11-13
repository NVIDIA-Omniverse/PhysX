// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-ATTRIBUTE-NAMES-001
// @covers AC-1 AC-3

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_attribute_tokens.h>

static_assert(sizeof(OVPHYSX_NAME_POSITION) == sizeof(ovx_token_or_string_t), "installed position declaration");
static_assert(sizeof(OVPHYSX_NAME_DEBUG_BODY_AXES) == sizeof(ovx_token_or_string_t), "installed debug declaration");
