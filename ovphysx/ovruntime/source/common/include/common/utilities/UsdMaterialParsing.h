// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <pxr/usd/usd/common.h>

// Utility functions shared by multiple extensions to help with USD physics materials.
//
// Material reads go through the parse-library's `parse::parseMaterial` /
// `parseDeformableMaterial` / `parseSurfaceDeformableMaterial` entry
// points.  This header exposes only `getMaterialBinding`.
namespace usdmaterialutils
{

/// Get the SDF path for physics-purpose bound material
///
/// \param[in] usdPrim      Prim where to search for the bound material
/// \return The SDF path for the bound material (if any)
PXR_NS::SdfPath getMaterialBinding(const PXR_NS::UsdPrim& usdPrim);

} // namespace usdmaterialutils
