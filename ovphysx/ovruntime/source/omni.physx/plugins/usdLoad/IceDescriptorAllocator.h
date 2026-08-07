// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * ICE-backed `IDescriptorAllocator` for the parse library.
 *
 * Passed to `scanStage` / `ParseContext` so descriptors produced by the
 * parse-library are allocated through ICE from the start.  Matches the
 * consumer's existing `releaseDesc` / `releaseShapeDesc` release path
 * (both call `ICE_FREE_BASIC`) so descriptors can move out of
 * `ScannedStage` via `.release()` instead of being deep-copied at the
 * boundary.
 *
 * @implements REQ-PARSE-CORE-006
 * @covers AC-6
 */

#pragma once

#include <omni/physics/parse/Allocator.h>

namespace omni::physx::usdparser
{

// Singleton accessor.  Safe to call from any thread.  The returned
// allocator's `allocate` / `deallocate` are thin shims over
// `GetAllocator()->malloc` and `ICE_FREE_BASIC`.
omni::physics::parse::IDescriptorAllocator& iceDescriptorAllocator();

} // namespace omni::physx::usdparser
