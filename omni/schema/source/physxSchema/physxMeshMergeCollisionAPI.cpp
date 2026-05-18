//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxMeshMergeCollisionAPI.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxMeshMergeCollisionAPI,
        TfType::Bases< UsdAPISchemaBase > >();
    
}

/* virtual */
PhysxSchemaPhysxMeshMergeCollisionAPI::~PhysxSchemaPhysxMeshMergeCollisionAPI()
{
}

/* static */
PhysxSchemaPhysxMeshMergeCollisionAPI
PhysxSchemaPhysxMeshMergeCollisionAPI::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxMeshMergeCollisionAPI();
    }
    return PhysxSchemaPhysxMeshMergeCollisionAPI(stage->GetPrimAtPath(path));
}


/* virtual */
UsdSchemaKind PhysxSchemaPhysxMeshMergeCollisionAPI::_GetSchemaKind() const
{
    return PhysxSchemaPhysxMeshMergeCollisionAPI::schemaKind;
}

/* static */
bool
PhysxSchemaPhysxMeshMergeCollisionAPI::CanApply(
    const UsdPrim &prim, std::string *whyNot)
{
    return prim.CanApplyAPI<PhysxSchemaPhysxMeshMergeCollisionAPI>(whyNot);
}

/* static */
PhysxSchemaPhysxMeshMergeCollisionAPI
PhysxSchemaPhysxMeshMergeCollisionAPI::Apply(const UsdPrim &prim)
{
    if (prim.ApplyAPI<PhysxSchemaPhysxMeshMergeCollisionAPI>()) {
        return PhysxSchemaPhysxMeshMergeCollisionAPI(prim);
    }
    return PhysxSchemaPhysxMeshMergeCollisionAPI();
}

/* static */
const TfType &
PhysxSchemaPhysxMeshMergeCollisionAPI::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxMeshMergeCollisionAPI>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxMeshMergeCollisionAPI::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxMeshMergeCollisionAPI::_GetTfType() const
{
    return _GetStaticTfType();
}

/*static*/
const TfTokenVector&
PhysxSchemaPhysxMeshMergeCollisionAPI::GetSchemaAttributeNames(bool includeInherited)
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

#include "tokens.h"

PXR_NAMESPACE_OPEN_SCOPE

UsdCollectionAPI
PhysxSchemaPhysxMeshMergeCollisionAPI::GetCollisionMeshesCollectionAPI() const
{
    return UsdCollectionAPI(GetPrim(), PhysxSchemaTokens->collisionmeshes);
}

PXR_NAMESPACE_CLOSE_SCOPE
