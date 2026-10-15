// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CONFIG-SNAPSHOT-001
// @covers AC-1 AC-2 AC-3
#pragma once
#include <omni/physx/AttachOptions.h>
namespace omni::physx::native
{
enum class ConfigurationResult : uint32_t { Success, Invalid, Stale, Busy, Error };
struct ConfigurationSnapshot
{
    uint64_t attachment=0, step=0;
    PhysxAttachmentSettings settings;
    PhysxSharedSettings resources;
};
// Capture retained request values from one exact idle owner. They describe the
// copied configuration, not proof that a CUDA context/GPU scene was realized.
// action 0 snapshots, action 1 changes only this owner's contact-processing flag.
// Resource/solver/output settings cannot be changed through this live interface.
// Rejected operations preserve output and state; no resource pointer escapes.
ConfigurationResult configureAttachment(uint64_t attachment,uint32_t action,bool disableContacts,
                                        ConfigurationSnapshot& output) noexcept;
}
