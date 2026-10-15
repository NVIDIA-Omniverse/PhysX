// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-INTERACTION-001
// @covers AC-5 AC-6
#pragma once
namespace physx { class PxScene; }
namespace omni::physx
{
namespace usdparser { class AttachedStage; }
namespace native
{
// Private stepper hook: caller holds the attachment's execution admission.
// Scene workers only read the owner registry; all mutation/retirement is idle.
void stepForceDrags(usdparser::AttachedStage&,::physx::PxScene&,float timeStep) noexcept;
}
}
