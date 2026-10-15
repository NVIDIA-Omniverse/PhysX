// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * ICE-backed `IDescriptorAllocator` for the parse library.
 *
 * Passed to scanStage / ParseContext so descriptor storage uses ICE while
 * DescPtr retains concrete destruction and the matching deallocation policy.
 * Runtime consumers borrow from the owning scan or retain an owning result.
 * Raw ICE_FREE on a parse descriptor skips its member destructors and is invalid.
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
