// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <pxr/usd/usd/common.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

// Utility functions shared by multiple extensions to help with USD physics materials
namespace usdmaterialutils
{

/// Get the SDF path for physics-purpose bound material
///
/// \param[in] usdPrim      Prim where to search for the bound material
/// \return The SDF path for the bound material (if any)
pxr::SdfPath getMaterialBinding(const pxr::UsdPrim& usdPrim);

/// Parse a material description from the given USD prim by applying a UsdPhysicsMaterialAPI
///
/// \param[in] stage      Weak reference to the prim's stage
/// \param[in] usdPrim    Prim to parse the physics material description from
/// \param[out] desc      Material description output
void parseMaterial(const pxr::UsdStageWeakPtr stage,
                   const pxr::UsdPrim& usdPrim,
                   omni::physics::schema::MaterialDesc& desc);

/// Parse a deformable material description from the given USD prim by applying a UsdPhysicsDeformableMaterialAPI
///
/// \param[in] stage      Weak reference to the prim's stage
/// \param[in] usdPrim    Prim to parse the physics material description from
/// \param[out] desc      Material description output
void parseDeformableMaterial(const pxr::UsdStageWeakPtr stage,
                             const pxr::UsdPrim& usdPrim,
                             omni::physics::schema::DeformableMaterialDesc& desc);

/// Parse a surface deformable material description from the given USD prim by applying a UsdPhysicsSurfaceDeformableMaterialAPI
///
/// \param[in] stage      Weak reference to the prim's stage
/// \param[in] usdPrim    Prim to parse the physics material description from
/// \param[out] desc      Material description output
void parseSurfaceDeformableMaterial(const pxr::UsdStageWeakPtr stage,
                                    const pxr::UsdPrim& usdPrim,
                                    omni::physics::schema::SurfaceDeformableMaterialDesc& desc);

/// Parse a curves deformable material description from the given USD prim by applying a UsdPhysicsCurvesDeformableMaterialAPI
///
/// \param[in] stage      Weak reference to the prim's stage
/// \param[in] usdPrim    Prim to parse the physics material description from
/// \param[out] desc      Material description output
void parseCurvesDeformableMaterial(const pxr::UsdStageWeakPtr stage,
                                   const pxr::UsdPrim& usdPrim,
                                   omni::physics::schema::CurvesDeformableMaterialDesc& desc);

} // namespace usdmaterialutils
