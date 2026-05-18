//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxCameraFollowVelocityAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxCameraFollowVelocityAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxCameraFollowVelocityAPI::~PhysxSchemaPhysxCameraFollowVelocityAPI()
{
}

/* static */
PhysxSchemaPhysxCameraFollowVelocityAPI
PhysxSchemaPhysxCameraFollowVelocityAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxCameraFollowVelocityAPI();
    }
    return PhysxSchemaPhysxCameraFollowVelocityAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxCameraFollowVelocityAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxCameraFollowVelocityAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxCameraFollowVelocityAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxCameraFollowVelocityAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxCameraFollowVelocityAPI
PhysxSchemaPhysxCameraFollowVelocityAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxCameraFollowVelocityAPI>()) {
        return PhysxSchemaPhysxCameraFollowVelocityAPI(prim);
    }
    return PhysxSchemaPhysxCameraFollowVelocityAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxCameraFollowVelocityAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxCameraFollowVelocityAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxCameraFollowVelocityAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxCameraFollowVelocityAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

/*static*/
const TfTokenVector&
PhysxSchemaPhysxCameraFollowVelocityAPI::GetSchemaAttributeNames(bool includeInherited)
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
