// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace omni
{
namespace physx
{
struct IPhysx;
struct IPhysxVisualization;
} // namespace physx
} // namespace omni

namespace ovphysx
{
namespace internal
{
namespace sidecar
{

omni::physx::IPhysx* tryGetInjectedPhysxInterface();
omni::physx::IPhysxVisualization* tryGetInjectedPhysxVisualizationInterface();

} // namespace sidecar
} // namespace internal
} // namespace ovphysx
