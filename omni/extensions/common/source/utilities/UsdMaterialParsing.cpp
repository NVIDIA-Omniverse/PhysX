// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "UsdMaterialParsing.h"

namespace usdmaterialutils
{

using namespace pxr;

void parseMaterial(const UsdStageWeakPtr stage, const UsdPrim& usdPrim, omni::physics::schema::MaterialDesc& desc)
{
    const UsdPhysicsMaterialAPI usdMaterial = UsdPhysicsMaterialAPI::Get(stage, usdPrim.GetPrimPath());

    if (usdMaterial)
    {
        usdMaterial.GetDynamicFrictionAttr().Get(&desc.dynamicFriction);
        usdMaterial.GetStaticFrictionAttr().Get(&desc.staticFriction);

        usdMaterial.GetRestitutionAttr().Get(&desc.restitution);

        usdMaterial.GetDensityAttr().Get(&desc.density);
    }
}


void parseBaseMaterial(const UsdStageWeakPtr stage, const UsdPrim& usdPrim, omni::physics::schema::BaseMaterialDesc& desc)
{
    pxr::TfType schemaType = pxr::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->BaseMaterialAPI);
    if (usdPrim.HasAPI(schemaType))
    {
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->dynamicFriction).Get(&desc.dynamicFriction);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->staticFriction).Get(&desc.staticFriction);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->density).Get(&desc.density);
    }
}

void parseDeformableMaterial(const UsdStageWeakPtr stage, const UsdPrim& usdPrim, omni::physics::schema::DeformableMaterialDesc& desc)
{
    parseBaseMaterial(stage, usdPrim, desc);

    pxr::TfType schemaType = pxr::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableMaterialAPI);
    if (usdPrim.HasAPI(schemaType))
    {
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->youngsModulus).Get(&desc.youngsModulus);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->poissonsRatio).Get(&desc.poissonsRatio);
    }
}

void parseSurfaceDeformableMaterial(const UsdStageWeakPtr stage, const UsdPrim& usdPrim,
                                    omni::physics::schema::SurfaceDeformableMaterialDesc& desc)
{
    parseDeformableMaterial(stage, usdPrim, desc);

    pxr::TfType schemaType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->SurfaceDeformableMaterialAPI);
    if (usdPrim.HasAPI(schemaType))
    {
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->surfaceThickness).Get(&desc.surfaceThickness);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->surfaceStretchStiffness).Get(&desc.surfaceStretchStiffness);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->surfaceShearStiffness).Get(&desc.surfaceShearStiffness);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->surfaceBendStiffness).Get(&desc.surfaceBendStiffness);
    }
}

void parseCurvesDeformableMaterial(const UsdStageWeakPtr stage, const UsdPrim& usdPrim,
                                   omni::physics::schema::CurvesDeformableMaterialDesc& desc)
{
    parseDeformableMaterial(stage, usdPrim, desc);

    pxr::TfType schemaType = pxr::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->CurvesDeformableMaterialAPI);
    if (usdPrim.HasAPI(schemaType))
    {
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->curveThickness).Get(&desc.curveThickness);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->curveStretchStiffness).Get(&desc.curveStretchStiffness);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->curveBendStiffness).Get(&desc.curveBendStiffness);
        usdPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->curveTwistStiffness).Get(&desc.curveTwistStiffness);
    }
}

pxr::SdfPath getMaterialBinding(const pxr::UsdPrim& usdPrim)
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
