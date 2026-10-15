// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-3 AC-4
#pragma once
#include <cstdint>
namespace omni::physx
{
/// Result of an explicit simulation dispatch. Busy means no work was submitted
/// or fetched. It signals a conflicting operation/pending interval on the selected
/// attachment, or forbidden callback reentry. A peer attachment may run independently.
/// Fetch the selected owner's outstanding scenes before retrying its mutations.
enum class RuntimeResult : uint32_t
{
    eSuccess = 0,
    eBusy,
    eInvalidAttachment,
    eInvalidScene,
    eInvalidArgument,
};

}
