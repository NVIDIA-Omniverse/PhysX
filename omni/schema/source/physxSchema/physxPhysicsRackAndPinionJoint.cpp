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
#include ".//physxPhysicsRackAndPinionJoint.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxPhysicsRackAndPinionJoint,
        TfType::Bases< UsdPhysicsJoint > >();
    
    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("PhysxPhysicsRackAndPinionJoint")
    // to find TfType<PhysxSchemaPhysxPhysicsRackAndPinionJoint>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, PhysxSchemaPhysxPhysicsRackAndPinionJoint>("PhysxPhysicsRackAndPinionJoint");
}

/* virtual */
PhysxSchemaPhysxPhysicsRackAndPinionJoint::~PhysxSchemaPhysxPhysicsRackAndPinionJoint()
{
}

/* static */
PhysxSchemaPhysxPhysicsRackAndPinionJoint
PhysxSchemaPhysxPhysicsRackAndPinionJoint::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPhysicsRackAndPinionJoint();
    }
    return PhysxSchemaPhysxPhysicsRackAndPinionJoint(stage->GetPrimAtPath(path));
}

/* static */
PhysxSchemaPhysxPhysicsRackAndPinionJoint
PhysxSchemaPhysxPhysicsRackAndPinionJoint::Define(
    const UsdStagePtr &stage, const SdfPath &path)
{
    static TfToken usdPrimTypeName("PhysxPhysicsRackAndPinionJoint");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPhysicsRackAndPinionJoint();
    }
    return PhysxSchemaPhysxPhysicsRackAndPinionJoint(
        stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxPhysicsRackAndPinionJoint::_GetSchemaKind() const
{
    return PhysxSchemaPhysxPhysicsRackAndPinionJoint::schemaKind;
}

/* static */
const TfType &
PhysxSchemaPhysxPhysicsRackAndPinionJoint::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxPhysicsRackAndPinionJoint>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxPhysicsRackAndPinionJoint::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxPhysicsRackAndPinionJoint::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxPhysicsRackAndPinionJoint::GetRatioAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physicsRatio);
}

UsdAttribute
PhysxSchemaPhysxPhysicsRackAndPinionJoint::CreateRatioAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physicsRatio,
                       SdfValueTypeNames->Float,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxPhysicsRackAndPinionJoint::GetHingeRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physicsHinge);
}

UsdRelationship
PhysxSchemaPhysxPhysicsRackAndPinionJoint::CreateHingeRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physicsHinge,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxPhysicsRackAndPinionJoint::GetPrismaticRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physicsPrismatic);
}

UsdRelationship
PhysxSchemaPhysxPhysicsRackAndPinionJoint::CreatePrismaticRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physicsPrismatic,
                       /* custom = */ false);
}

namespace {
static inline TfTokenVector
_ConcatenateAttributeNames(const TfTokenVector& left,const TfTokenVector& right)
{
    TfTokenVector result;
    result.reserve(left.size() + right.size());
    result.insert(result.end(), left.begin(), left.end());
    result.insert(result.end(), right.begin(), right.end());
    return result;
}
}

/*static*/
const TfTokenVector&
PhysxSchemaPhysxPhysicsRackAndPinionJoint::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physicsRatio,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            UsdPhysicsJoint::GetSchemaAttributeNames(true),
            localNames);

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
