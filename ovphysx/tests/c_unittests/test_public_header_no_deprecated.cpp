// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-7
 */

#include <ovphysx/experimental/ovphysx.hpp>
#include <type_traits>
#include <ovphysx_companion/companion.h>
#include <ovphysx_companion/interaction.h>
#include <ovphysx_companion/native.h>
// This target has no PhysX SDK include directory. Ordinary companion use must
// compile without either the PhysX or Carbonite SDK.
static_assert(sizeof(ovphysx_companion_session_t) == sizeof(uint64_t));

static_assert(sizeof(ovphysx_instance_handle_t) == sizeof(uint64_t));

static_assert(std::is_same<decltype(ovphysx_attach_ovstage(0, nullptr, 1)), ovphysx_enqueue_result_t>::value);
static_assert(std::is_same<decltype(ovphysx_update_from_ovstage(0, {})), ovphysx_enqueue_result_t>::value);
static_assert(std::is_same<decltype(ovphysx_detach_ovstage(0)), ovphysx_enqueue_result_t>::value);
static_assert(std::is_same<decltype(ovphysx_is_stage_attached(0, nullptr, nullptr)), ovphysx_result_t>::value);
