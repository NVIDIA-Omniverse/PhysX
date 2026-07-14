// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <pxr/base/plug/registry.h>
#include <pxr/base/plug/plugin.h>
#include <pxr/usd/usd/schemaRegistry.h>

#include <common/utilities/Utilities.h>

#include <omni/kit/EditorUsd.h>

OMNI_LOG_DECLARE_CHANNEL(kSupportUiLogChannel)

namespace omni
{
namespace physx
{

bool setTokenAttributeWithSdf(PXR_NS::UsdStageRefPtr stage, PXR_NS::UsdPrim& prim, PXR_NS::TfToken attributeToken, PXR_NS::TfToken value)
{
    OMNI_LOG_INFO(kSupportUiLogChannel, "setTokenAttributeWithSdf for prim \"%s\": Attribute[\"%s\"] = \"%s\"",
                  prim.GetPath().GetText(), attributeToken.GetText(), value.GetText());

    const PXR_NS::UsdEditTarget& editTarget = stage->GetEditTarget();
    if (!editTarget.IsValid())
    {
        return false;
    }

    const PXR_NS::SdfPath& targetPath = editTarget.MapToSpecPath(prim.GetPath());
    PXR_NS::SdfPrimSpecHandle primSpec = PXR_NS::SdfCreatePrimInLayer(editTarget.GetLayer(), targetPath);
    if (!primSpec)
    {
        return false;
    }

    PXR_NS::SdfPath attributePath = prim.GetPath().AppendProperty(attributeToken);
    PXR_NS::SdfAttributeSpecHandle attrSpec = primSpec->GetAttributeAtPath(attributePath);
    if (!attrSpec)
    {
        attrSpec = PXR_NS::SdfAttributeSpec::New(primSpec, attributeToken.GetString(), PXR_NS::SdfValueTypeNames->Token);
    }

    if (!attrSpec)
    {
        return false;
    }

    return attrSpec->SetDefaultValue(PXR_NS::VtValue(value));
}

bool isComponentKind(const PXR_NS::UsdPrim& bodyPrim)
{
    bool ret = false;

    PXR_NS::UsdModelAPI modelApi(bodyPrim);
    PXR_NS::TfToken kind;

    if (modelApi.GetKind(&kind) && PXR_NS::KindRegistry::IsA(kind, PXR_NS::KindTokens->component))
    {
        ret = true;
    }

    return ret;
}

bool physicsSceneEnableCcd(PXR_NS::UsdPrim& prim, bool state)
{
    PXR_NS::PhysxSchemaPhysxSceneAPI api = PXR_NS::PhysxSchemaPhysxSceneAPI::Get(prim.GetStage(), prim.GetPath());

    if (!api)
    {
        api = PXR_NS::PhysxSchemaPhysxSceneAPI::Apply(prim);
    }

    if (api)
    {
        return api.CreateEnableCCDAttr().Set(state);
    }

    return false;
}

bool physicsSceneIsCcdEnabled(const PXR_NS::UsdPrim& prim, bool& value)
{
    PXR_NS::PhysxSchemaPhysxSceneAPI api = PXR_NS::PhysxSchemaPhysxSceneAPI::Get(prim.GetStage(), prim.GetPath());

    if (api)
    {
        return api.GetEnableCCDAttr().Get(&value);
    }

    return false;
}

const PXR_NS::UsdEditTarget& SetSessionLayer(PXR_NS::UsdStageRefPtr stage)
{
    const PXR_NS::UsdEditTarget& editTarget = stage->GetEditTarget();

    stage->SetEditTarget(stage->GetSessionLayer());

    return editTarget;
}

bool hasPhysicsPhysXSchemaApplied(const PXR_NS::UsdPrim& prim)
{
    PXR_NS::UsdSchemaRegistry& schemaRegistry = PXR_NS::UsdSchemaRegistry::GetInstance();
    PXR_NS::PlugRegistry& plugRegistry = PXR_NS::PlugRegistry::GetInstance();

    const auto tokens = prim.GetAppliedSchemas();

    for (const auto& token : tokens)
    {
        std::string t = token;
        std::string tokenStr = t.substr(0, t.find(":"));
        auto schemaToken = schemaRegistry.GetTypeFromSchemaTypeName(PXR_NS::TfToken(tokenStr));
        auto physics = plugRegistry.GetPluginWithName("usdPhysics");
        auto physx = plugRegistry.GetPluginWithName("physxSchema");

        if ((physx && physx->DeclaresType(schemaToken)) || (physics && physics->DeclaresType(schemaToken)))
        {
            return true;
        }
    }

    return false;
}

bool hasChangedToken(const PXR_NS::UsdNotice::ObjectsChanged& objectsChanged,
                     const PXR_NS::SdfPath& primPath,
                     const PXR_NS::TfToken token)
{
    const auto& changedFields = objectsChanged.GetChangedFields(primPath);

    for (const auto& field : changedFields)
    {
        if (field == token)
        {
            return true;
        }
    }

    return false;
}

} // namespace physx
} // namespace omni
