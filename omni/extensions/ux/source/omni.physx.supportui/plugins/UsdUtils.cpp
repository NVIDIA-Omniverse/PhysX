// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

bool setTokenAttributeWithSdf(pxr::UsdStageRefPtr stage, pxr::UsdPrim& prim, pxr::TfToken attributeToken, pxr::TfToken value)
{
    OMNI_LOG_INFO(kSupportUiLogChannel, "setTokenAttributeWithSdf for prim \"%s\": Attribute[\"%s\"] = \"%s\"",
                  prim.GetPath().GetText(), attributeToken.GetText(), value.GetText());

    const pxr::UsdEditTarget& editTarget = stage->GetEditTarget();
    if (!editTarget.IsValid())
    {
        return false;
    }

    const pxr::SdfPath& targetPath = editTarget.MapToSpecPath(prim.GetPath());
    pxr::SdfPrimSpecHandle primSpec = pxr::SdfCreatePrimInLayer(editTarget.GetLayer(), targetPath);
    if (!primSpec)
    {
        return false;
    }

    pxr::SdfPath attributePath = prim.GetPath().AppendProperty(attributeToken);
    pxr::SdfAttributeSpecHandle attrSpec = primSpec->GetAttributeAtPath(attributePath);
    if (!attrSpec)
    {
        attrSpec = pxr::SdfAttributeSpec::New(primSpec, attributeToken.GetString(), pxr::SdfValueTypeNames->Token);
    }

    if (!attrSpec)
    {
        return false;
    }

    return attrSpec->SetDefaultValue(pxr::VtValue(value));
}

bool isComponentKind(const pxr::UsdPrim& bodyPrim)
{
    bool ret = false;

    pxr::UsdModelAPI modelApi(bodyPrim);
    pxr::TfToken kind;

    if (modelApi.GetKind(&kind) && pxr::KindRegistry::IsA(kind, pxr::KindTokens->component))
    {
        ret = true;
    }

    return ret;
}

bool physicsSceneEnableCcd(pxr::UsdPrim& prim, bool state)
{
    pxr::PhysxSchemaPhysxSceneAPI api = pxr::PhysxSchemaPhysxSceneAPI::Get(prim.GetStage(), prim.GetPath());

    if (!api)
    {
        api = pxr::PhysxSchemaPhysxSceneAPI::Apply(prim);
    }

    if (api)
    {
        return api.CreateEnableCCDAttr().Set(state);
    }

    return false;
}

bool physicsSceneIsCcdEnabled(const pxr::UsdPrim& prim, bool& value)
{
    pxr::PhysxSchemaPhysxSceneAPI api = pxr::PhysxSchemaPhysxSceneAPI::Get(prim.GetStage(), prim.GetPath());

    if (api)
    {
        return api.GetEnableCCDAttr().Get(&value);
    }

    return false;
}

const pxr::UsdEditTarget& SetSessionLayer(pxr::UsdStageRefPtr stage)
{
    const pxr::UsdEditTarget& editTarget = stage->GetEditTarget();

    stage->SetEditTarget(stage->GetSessionLayer());

    return editTarget;
}

bool hasPhysicsPhysXSchemaApplied(const pxr::UsdPrim& prim)
{
    pxr::UsdSchemaRegistry& schemaRegistry = pxr::UsdSchemaRegistry::GetInstance();
    pxr::PlugRegistry& plugRegistry = pxr::PlugRegistry::GetInstance();

    const auto tokens = prim.GetAppliedSchemas();

    for (const auto& token : tokens)
    {
        std::string t = token;
        std::string tokenStr = t.substr(0, t.find(":"));
        auto schemaToken = schemaRegistry.GetTypeFromSchemaTypeName(pxr::TfToken(tokenStr));
        auto physics = plugRegistry.GetPluginWithName("usdPhysics");
        auto physx = plugRegistry.GetPluginWithName("physxSchema");

        if ((physx && physx->DeclaresType(schemaToken)) || (physics && physics->DeclaresType(schemaToken)))
        {
            return true;
        }
    }

    return false;
}

bool hasChangedToken(const pxr::UsdNotice::ObjectsChanged& objectsChanged,
                     const pxr::SdfPath& primPath,
                     const pxr::TfToken token)
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
