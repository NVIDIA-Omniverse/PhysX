// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <private/omni/physx/PhysxUsd.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/UsdMaterialParsing.h>

#include "LoadUsd.h"
#include "LoadTools.h"

#include "Particles.h"
#include "SoftBodyDeprecated.h"
#include "AttributeHelpers.h"

using namespace pxr;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

void setToDefault(PhysxMaterialDesc& desc)
{
    desc.frictionCombineMode = eAverage;
    desc.restitutionCombineMode = eAverage;
    desc.dampingCombineMode = eAverage;

    desc.staticFriction = 0.5f;
    desc.dynamicFriction = 0.5f;
    desc.restitution = 0.0f;
    desc.density = -1.0f;

    desc.materialPath = SdfPath();

    desc.compliantAccelerationSpring = false;
    desc.compliantStiffness = 0.0f;
    desc.compliantDamping = 0.0f;
}

void setToDefault(UsdStageWeakPtr stage, PhysxDeformableMaterialDesc& desc)
{
    double metersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(stage);
    double kilogramsPerUnit = pxr::UsdPhysicsGetStageKilogramsPerUnit(stage);

    //same defaults as in deformableUtils.py
    desc.staticFriction = 0.5f;
    desc.dynamicFriction = 0.25f;
    desc.density = -1.0f;

    desc.youngsModulus = float(5.0e5*metersPerUnit/kilogramsPerUnit);
    desc.poissonsRatio = 0.45f;

    desc.elasticityDamping = 0.0f;

    desc.materialPath = SdfPath();
}

void setToDefault(UsdStageWeakPtr stage, PhysxSurfaceDeformableMaterialDesc& desc)
{
    double metersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(stage);

    setToDefault(stage, static_cast<PhysxDeformableMaterialDesc&>(desc));
    desc.surfaceThickness = float(0.001/metersPerUnit);
    desc.surfaceStretchStiffness = 0.0f;
    desc.surfaceShearStiffness = 0.0f;
    desc.surfaceBendStiffness = 0.0f;

    desc.bendDamping = 0.0f;
}

void setToDefault(PBDMaterialDesc& desc)
{
    desc.friction = 0.2f;
    desc.particleFrictionScale = 1.0f;
    desc.damping = 0.0f;
    desc.viscosity = 0.0f;

    desc.vorticityConfinement = 0.0f;
    desc.surfaceTension = 0.0f;
    desc.cohesion = 0.0f;
    desc.adhesion = 0.0f;
    desc.particleAdhesionScale = 1.0f;
    desc.adhesionOffsetScale = 0.0f;

    desc.lift = 0.0f;
    desc.drag = 0.0f;

    desc.gravityScale = 1.0f;

    desc.cflCoefficient = 1.0f;

    desc.density = 1000.0f;

    desc.materialPath = SdfPath();
}

void setToDefaultDeprecated(FemSoftBodyMaterialDesc& desc)
{
    desc.density = 0.0f;
    desc.damping = 0.005f;
    desc.dampingScale = 1.0f;
    desc.dynamicFriction = 0.25f;
    desc.youngs = 50000000;
    desc.poissons = 0.45f;

    desc.materialPath = SdfPath();
}

void setToDefaultDeprecated(FemClothMaterialDesc& desc)
{
    desc.density = 0.0f;
    desc.dynamicFriction = 0.25f;
    desc.youngs = 5.0e7f;
    desc.poissons = 0.45f;
    desc.thickness = 0.001f;
    desc.materialPath = SdfPath();
}

void parseMaterialDescInt(const pxr::UsdStageWeakPtr stage, const omni::physics::schema::MaterialDesc& inDesc, PhysxMaterialDesc& materialDesc)
{
    materialDesc.dynamicFriction = inDesc.dynamicFriction;
    materialDesc.staticFriction = inDesc.staticFriction;
    materialDesc.restitution = inDesc.restitution;
    materialDesc.density = inDesc.density;

    materialDesc.materialPath = inDesc.usdPrim.GetPrimPath();

    const PhysxSchemaPhysxMaterialAPI physxMaterialAPI = PhysxSchemaPhysxMaterialAPI::Get(stage, inDesc.usdPrim.GetPrimPath());
    if (physxMaterialAPI)
    {
        if (physxMaterialAPI.GetFrictionCombineModeAttr())
        {
            TfToken combMode;
            physxMaterialAPI.GetFrictionCombineModeAttr().Get(&combMode);
            if (combMode == PhysxSchemaTokens.Get()->average)
                materialDesc.frictionCombineMode = eAverage;
            else if (combMode == PhysxSchemaTokens.Get()->min)
                materialDesc.frictionCombineMode = eMin;
            else if (combMode == PhysxSchemaTokens.Get()->max)
                materialDesc.frictionCombineMode = eMax;
            else if (combMode == PhysxSchemaTokens.Get()->multiply)
                materialDesc.frictionCombineMode = eMultiply;
        }
        if (physxMaterialAPI.GetRestitutionCombineModeAttr())
        {
            TfToken combMode;
            physxMaterialAPI.GetRestitutionCombineModeAttr().Get(&combMode);
            if (combMode == PhysxSchemaTokens.Get()->average)
                materialDesc.restitutionCombineMode = eAverage;
            else if (combMode == PhysxSchemaTokens.Get()->min)
                materialDesc.restitutionCombineMode = eMin;
            else if (combMode == PhysxSchemaTokens.Get()->max)
                materialDesc.restitutionCombineMode = eMax;
            else if (combMode == PhysxSchemaTokens.Get()->multiply)
                materialDesc.restitutionCombineMode = eMultiply;
        }
        if (physxMaterialAPI.GetDampingCombineModeAttr())
        {
            TfToken combMode;
            physxMaterialAPI.GetDampingCombineModeAttr().Get(&combMode);
            if (combMode == PhysxSchemaTokens.Get()->average)
                materialDesc.dampingCombineMode = eAverage;
            else if (combMode == PhysxSchemaTokens.Get()->min)
                materialDesc.dampingCombineMode = eMin;
            else if (combMode == PhysxSchemaTokens.Get()->max)
                materialDesc.dampingCombineMode = eMax;
            else if (combMode == PhysxSchemaTokens.Get()->multiply)
                materialDesc.dampingCombineMode = eMultiply;
        }

        // parse compliance:
        getAttribute<bool>(materialDesc.compliantAccelerationSpring, physxMaterialAPI.GetCompliantContactAccelerationSpringAttr(), nullptr);
        getAttribute<float>(materialDesc.compliantStiffness, physxMaterialAPI.GetCompliantContactStiffnessAttr(), 0.0f, FLT_MAX, nullptr);
        getAttribute<float>(materialDesc.compliantDamping, physxMaterialAPI.GetCompliantContactDampingAttr(), 0.0f, FLT_MAX, nullptr);
    }
}

PhysxMaterialDesc* parseMaterialDesc(const pxr::UsdStageWeakPtr stage, const omni::physics::schema::MaterialDesc& inDesc)
{
    PhysxMaterialDesc* materialDesc = ICE_PLACEMENT_NEW(PhysxMaterialDesc)();
    setToDefault(*materialDesc);

    parseMaterialDescInt(stage, inDesc, *materialDesc);

    return materialDesc;
}

void parseDeformableMaterialDescInt(const pxr::UsdStageWeakPtr /*stage*/,
    const omni::physics::schema::DeformableMaterialDesc& inDesc,
    PhysxDeformableMaterialDesc& desc)
{
    desc.dynamicFriction = inDesc.dynamicFriction;
    desc.staticFriction = inDesc.staticFriction;
    desc.density = inDesc.density;
    desc.youngsModulus = inDesc.youngsModulus;
    desc.poissonsRatio = std::min(inDesc.poissonsRatio, 0.4999f);
    desc.materialPath = inDesc.usdPrim.GetPrimPath();

    const TfType matType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->DeformableMaterialAPI);
    if (inDesc.usdPrim.HasAPI(matType))
    {
        getAttribute<float>(desc.elasticityDamping, inDesc.usdPrim, PhysxAdditionAttrTokens->elasticityDamping, 0.0f, FLT_MAX, nullptr);
    }
}

void parseSurfaceDeformableMaterialDescInt(const pxr::UsdStageWeakPtr stage,
    const omni::physics::schema::SurfaceDeformableMaterialDesc& inDesc,
    PhysxSurfaceDeformableMaterialDesc& desc)
{
    parseDeformableMaterialDescInt(stage, inDesc, desc);
    desc.surfaceThickness = inDesc.surfaceThickness;
    desc.surfaceStretchStiffness = inDesc.surfaceStretchStiffness;
    desc.surfaceShearStiffness = inDesc.surfaceShearStiffness;
    desc.surfaceBendStiffness = inDesc.surfaceBendStiffness;

    const TfType matType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->SurfaceDeformableMaterialAPI);
    if (inDesc.usdPrim.HasAPI(matType))
    {
        getAttribute<float>(desc.bendDamping, inDesc.usdPrim, PhysxAdditionAttrTokens->bendDamping, 0.0f, FLT_MAX, nullptr);
    }
}

PhysxDeformableMaterialDesc* parseDeformableMaterialDesc(const pxr::UsdStageWeakPtr stage, const omni::physics::schema::DeformableMaterialDesc& inDesc)
{
    PhysxDeformableMaterialDesc* materialDesc = nullptr;
    if (inDesc.type == omni::physics::schema::ObjectType::eDeformableMaterial)
    {
        materialDesc = ICE_PLACEMENT_NEW(PhysxDeformableMaterialDesc)();
        setToDefault(stage, *materialDesc);
        parseDeformableMaterialDescInt(stage, inDesc, *materialDesc);
    }
    else if (inDesc.type == omni::physics::schema::ObjectType::eSurfaceDeformableMaterial)
    {
        PhysxSurfaceDeformableMaterialDesc* surfaceMaterialDesc = ICE_PLACEMENT_NEW(PhysxSurfaceDeformableMaterialDesc)();
        setToDefault(stage, *surfaceMaterialDesc);
        parseSurfaceDeformableMaterialDescInt(stage,
            static_cast<const omni::physics::schema::SurfaceDeformableMaterialDesc&>(inDesc), *surfaceMaterialDesc);
        materialDesc = surfaceMaterialDesc;
    }

    return materialDesc;
}

void parsePBDMaterialForPrim(const pxr::UsdPrim& usdPrim, const SdfPath& materialPath, PBDMaterialDesc& desc)
{
    setToDefault(desc);

    if (materialPath != SdfPath())
    {
        const UsdPrim materialPrim = usdPrim.GetStage()->GetPrimAtPath(materialPath);
        if (materialPrim.HasAPI<PhysxSchemaPhysxPBDMaterialAPI>())
        {
            PBDMaterialDesc* ptr = ParsePBDParticleMaterial(usdPrim.GetStage(), materialPrim);
            if (ptr)
            {
                desc = *ptr;
                ICE_FREE(ptr);
            }
        }
    }
}

void parseDeformableBodyMaterialForPrimDeprecated(const pxr::UsdPrim& usdPrim, const pxr::SdfPath& materialPath, FemSoftBodyMaterialDesc& desc)
{
    setToDefaultDeprecated(desc);

    if (materialPath != SdfPath())
    {
        const UsdPrim materialPrim = usdPrim.GetStage()->GetPrimAtPath(materialPath);
        if (materialPrim.HasAPI<PhysxSchemaPhysxDeformableBodyMaterialAPI>())
        {
            FemSoftBodyMaterialDesc* ptr = ParseDeformableBodyMaterialDeprecated(materialPrim);
            if (ptr)
            {
                desc = *ptr;
                ICE_FREE(ptr);
            }
        }
    }
}


void parseDeformableSurfaceMaterialForPrimDeprecated(const pxr::UsdPrim& usdPrim, const pxr::SdfPath& materialPath, FemClothMaterialDesc& desc)
{
    setToDefaultDeprecated(desc);

    if (materialPath != SdfPath())
    {
        const UsdPrim materialPrim = usdPrim.GetStage()->GetPrimAtPath(materialPath);
        if (materialPrim.HasAPI<PhysxSchemaPhysxDeformableSurfaceMaterialAPI>())
        {
            FemClothMaterialDesc* ptr = ParseDeformableSurfaceMaterialDeprecated(materialPrim);
            if (ptr)
            {
                desc = *ptr;
                ICE_FREE(ptr);
            }
        }
    }
}

void parseDeformableMaterialForPrim(const pxr::UsdPrim& usdPrim, const SdfPath& materialPath, PhysxDeformableMaterialDesc& desc)
{
    UsdStageWeakPtr stage = usdPrim.GetStage();
    setToDefault(stage, desc);

    if (materialPath != SdfPath())
    {
        const UsdPrim materialPrim = stage->GetPrimAtPath(materialPath);
        omni::physics::schema::DeformableMaterialDesc inDesc;
        inDesc.usdPrim = materialPrim;
        usdmaterialutils::parseDeformableMaterial(stage, materialPrim, inDesc);
        parseDeformableMaterialDescInt(stage, inDesc, desc);
    }
}

void parseSurfaceDeformableMaterialForPrim(const pxr::UsdPrim& usdPrim, const SdfPath& materialPath, PhysxSurfaceDeformableMaterialDesc& desc)
{
    UsdStageWeakPtr stage = usdPrim.GetStage();
    setToDefault(stage, desc);

    if (materialPath != SdfPath())
    {
        const UsdPrim materialPrim = stage->GetPrimAtPath(materialPath);
        omni::physics::schema::SurfaceDeformableMaterialDesc inDesc;
        inDesc.usdPrim = materialPrim;
        usdmaterialutils::parseSurfaceDeformableMaterial(stage, materialPrim, inDesc);
        parseSurfaceDeformableMaterialDescInt(stage, inDesc, desc);
    }
}

void parseMaterialForPrim(const pxr::UsdPrim& usdPrim, const SdfPath& materialPath, PhysxMaterialDesc& desc)
{
    setToDefault(desc);

    if (materialPath != SdfPath())
    {
        const UsdPrim materialPrim = usdPrim.GetStage()->GetPrimAtPath(materialPath);
        omni::physics::schema::MaterialDesc inDesc;
        inDesc.usdPrim = materialPrim;

        usdmaterialutils::parseMaterial(usdPrim.GetStage(), materialPrim, inDesc);

        parseMaterialDescInt(usdPrim.GetStage(), inDesc, desc);
    }
}

ObjectId getMaterial(AttachedStage& attachedStage, const SdfPath& path)
{
    if (path != SdfPath())
        return attachedStage.getObjectDatabase()->findEntry(path, eMaterial);
    else
        return kInvalidObjectId;
}

ObjectId getMaterial(AttachedStage& attachedStage, const SdfPath& path, const ObjectCategory objCategory)
{
    if (path != SdfPath())
        return attachedStage.getObjectDatabase()->findEntry(path, objCategory);
    else
        return kInvalidObjectId;
}

} // namespace usdparser
} // namespace physx
} // namespace omni
