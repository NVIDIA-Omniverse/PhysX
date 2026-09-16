// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-USDLIB-LOADABLE-001
 * @covers AC-2
 */

#pragma once

#include <omni/physics/parse/UsdAttachmentAuthor.h> // omni::physics::parse::IUsdAttachmentAuthor

#include <memory>

namespace omni::physics::usd
{

// Factory for the USD deformable auto-attachment authoring seam (ADR-0027 bridge collapse).
// Builds the concrete IUsdAttachmentAuthor that reads the live UsdStageWeakPtr out of the opaque
// handle storage, rebuilds each SdfPath from the resolved path string the omni.physx facade
// passes, and reuses the USD backend authoring (omni::physics::usd::defineAttachmentPrim /
// defineElementCollisionFilterPrim / removeAttachmentsAndFilters / disableAttachmentsAndFilters
// plus the deformable schema tokens). The loader installs the result via
// parse::setUsdAttachmentAuthor; the USD-free omni.physx side drives it through the pxr-free
// IUsdAttachmentAuthor interface only.
std::unique_ptr<omni::physics::parse::IUsdAttachmentAuthor> makeUsdAttachmentAuthor();

} // namespace omni::physics::usd
