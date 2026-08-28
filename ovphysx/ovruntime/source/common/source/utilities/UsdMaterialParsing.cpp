// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause


// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "UsdMaterialParsing.h"

namespace usdmaterialutils
{

using namespace PXR_NS;

PXR_NS::SdfPath getMaterialBinding(const PXR_NS::UsdPrim& usdPrim)
{
    SdfPath materialPath = SdfPath();

    const static TfToken physicsPurpose("physics");
    UsdShadeMaterialBindingAPI materialBindingAPI = UsdShadeMaterialBindingAPI(usdPrim);
    if (materialBindingAPI)
    {
        UsdShadeMaterial material = materialBindingAPI.ComputeBoundMaterial(physicsPurpose);
        if (material)
        {
            materialPath = material.GetPrim().GetPrimPath();
        }
    }
    else
    {
        // handle material through a direct binding rel search
        std::vector<UsdPrim> prims;
        prims.push_back(usdPrim);
        std::vector<UsdShadeMaterial> materials =
            UsdShadeMaterialBindingAPI::ComputeBoundMaterials(prims, physicsPurpose);
        if (!materials.empty() && materials[0])
        {
            materialPath = materials[0].GetPrim().GetPrimPath();
        }
    }

    return materialPath;
}

} // namespace usdmaterialutils
