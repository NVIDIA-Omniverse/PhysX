//
// Copyright 2016 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//
#include ".//physxVehicleDriveStandardAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxVehicleDriveStandardAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxVehicleDriveStandardAPI::~PhysxSchemaPhysxVehicleDriveStandardAPI()
{
}

/* static */
PhysxSchemaPhysxVehicleDriveStandardAPI
PhysxSchemaPhysxVehicleDriveStandardAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxVehicleDriveStandardAPI();
    }
    return PhysxSchemaPhysxVehicleDriveStandardAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxVehicleDriveStandardAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxVehicleDriveStandardAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxVehicleDriveStandardAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxVehicleDriveStandardAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxVehicleDriveStandardAPI
PhysxSchemaPhysxVehicleDriveStandardAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxVehicleDriveStandardAPI>()) {
        return PhysxSchemaPhysxVehicleDriveStandardAPI(prim);
    }
    return PhysxSchemaPhysxVehicleDriveStandardAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxVehicleDriveStandardAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxVehicleDriveStandardAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxVehicleDriveStandardAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxVehicleDriveStandardAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdRelationship
PhysxSchemaPhysxVehicleDriveStandardAPI::GetEngineRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleDriveStandardEngine);
}

UsdRelationship
PhysxSchemaPhysxVehicleDriveStandardAPI::CreateEngineRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleDriveStandardEngine,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxVehicleDriveStandardAPI::GetGearsRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleDriveStandardGears);
}

UsdRelationship
PhysxSchemaPhysxVehicleDriveStandardAPI::CreateGearsRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleDriveStandardGears,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxVehicleDriveStandardAPI::GetAutoGearBoxRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleDriveStandardAutoGearBox);
}

UsdRelationship
PhysxSchemaPhysxVehicleDriveStandardAPI::CreateAutoGearBoxRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleDriveStandardAutoGearBox,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxVehicleDriveStandardAPI::GetClutchRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physxVehicleDriveStandardClutch);
}

UsdRelationship
PhysxSchemaPhysxVehicleDriveStandardAPI::CreateClutchRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physxVehicleDriveStandardClutch,
                       /* custom = */ false);
}

/*static*/
const TfTokenVector&
PhysxSchemaPhysxVehicleDriveStandardAPI::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames;
    static TfTokenVector allNames =
        UsdAPISchemaBase::GetSchemaAttributeNames(true);

    if (includeInherited)
        return allNames;
    else
        return localNames;
}

PXR_NAMESPACE_CLOSE_SCOPE

// ===================================================================== //
// Feel free to add custom code below this line. It will be preserved by
// the code generator.
//
// Just remember to wrap code in the appropriate delimiters:
// 'PXR_NAMESPACE_OPEN_SCOPE', 'PXR_NAMESPACE_CLOSE_SCOPE'.
// ===================================================================== //
// --(BEGIN CUSTOM CODE)--
