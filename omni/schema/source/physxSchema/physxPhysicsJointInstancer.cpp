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
#include ".//physxPhysicsJointInstancer.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/typed.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/assetPath.h"

PXR_NAMESPACE_OPEN_SCOPE

// Register the schema with the TfType system.
TF_REGISTRY_FUNCTION(TfType)
{
    TfType::Define<PhysxSchemaPhysxPhysicsJointInstancer,
        TfType::Bases< PhysxSchemaPhysxPhysicsInstancer > >();
    
    // Register the usd prim typename as an alias under UsdSchemaBase. This
    // enables one to call
    // TfType::Find<UsdSchemaBase>().FindDerivedByName("PhysxPhysicsJointInstancer")
    // to find TfType<PhysxSchemaPhysxPhysicsJointInstancer>, which is how IsA queries are
    // answered.
    TfType::AddAlias<UsdSchemaBase, PhysxSchemaPhysxPhysicsJointInstancer>("PhysxPhysicsJointInstancer");
}

/* virtual */
PhysxSchemaPhysxPhysicsJointInstancer::~PhysxSchemaPhysxPhysicsJointInstancer()
{
}

/* static */
PhysxSchemaPhysxPhysicsJointInstancer
PhysxSchemaPhysxPhysicsJointInstancer::Get(const UsdStagePtr &stage, const SdfPath &path)
{
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPhysicsJointInstancer();
    }
    return PhysxSchemaPhysxPhysicsJointInstancer(stage->GetPrimAtPath(path));
}

/* static */
PhysxSchemaPhysxPhysicsJointInstancer
PhysxSchemaPhysxPhysicsJointInstancer::Define(
    const UsdStagePtr &stage, const SdfPath &path)
{
    static TfToken usdPrimTypeName("PhysxPhysicsJointInstancer");
    if (!stage) {
        TF_CODING_ERROR("Invalid stage");
        return PhysxSchemaPhysxPhysicsJointInstancer();
    }
    return PhysxSchemaPhysxPhysicsJointInstancer(
        stage->DefinePrim(path, usdPrimTypeName));
}

/* virtual */
UsdSchemaKind PhysxSchemaPhysxPhysicsJointInstancer::_GetSchemaKind() const
{
    return PhysxSchemaPhysxPhysicsJointInstancer::schemaKind;
}

/* static */
const TfType &
PhysxSchemaPhysxPhysicsJointInstancer::_GetStaticTfType()
{
    static TfType tfType = TfType::Find<PhysxSchemaPhysxPhysicsJointInstancer>();
    return tfType;
}

/* static */
bool 
PhysxSchemaPhysxPhysicsJointInstancer::_IsTypedSchema()
{
    static bool isTyped = _GetStaticTfType().IsA<UsdTyped>();
    return isTyped;
}

/* virtual */
const TfType &
PhysxSchemaPhysxPhysicsJointInstancer::_GetTfType() const
{
    return _GetStaticTfType();
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::GetPhysicsBody0IndicesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physicsBody0Indices);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::CreatePhysicsBody0IndicesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physicsBody0Indices,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::GetPhysicsBody1IndicesAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physicsBody1Indices);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::CreatePhysicsBody1IndicesAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physicsBody1Indices,
                       SdfValueTypeNames->IntArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::GetPhysicsLocalPos0sAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physicsLocalPos0s);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::CreatePhysicsLocalPos0sAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physicsLocalPos0s,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::GetPhysicsLocalRot0sAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physicsLocalRot0s);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::CreatePhysicsLocalRot0sAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physicsLocalRot0s,
                       SdfValueTypeNames->QuathArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::GetPhysicsLocalPos1sAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physicsLocalPos1s);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::CreatePhysicsLocalPos1sAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physicsLocalPos1s,
                       SdfValueTypeNames->Point3fArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::GetPhysicsLocalRot1sAttr() const
{
    return GetPrim().GetAttribute(PhysxSchemaTokens->physicsLocalRot1s);
}

UsdAttribute
PhysxSchemaPhysxPhysicsJointInstancer::CreatePhysicsLocalRot1sAttr(VtValue const &defaultValue, bool writeSparsely) const
{
    return UsdSchemaBase::_CreateAttr(PhysxSchemaTokens->physicsLocalRot1s,
                       SdfValueTypeNames->QuathArray,
                       /* custom = */ false,
                       SdfVariabilityVarying,
                       defaultValue,
                       writeSparsely);
}

UsdRelationship
PhysxSchemaPhysxPhysicsJointInstancer::GetPhysicsBody0sRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physicsBody0s);
}

UsdRelationship
PhysxSchemaPhysxPhysicsJointInstancer::CreatePhysicsBody0sRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physicsBody0s,
                       /* custom = */ false);
}

UsdRelationship
PhysxSchemaPhysxPhysicsJointInstancer::GetPhysicsBody1sRel() const
{
    return GetPrim().GetRelationship(PhysxSchemaTokens->physicsBody1s);
}

UsdRelationship
PhysxSchemaPhysxPhysicsJointInstancer::CreatePhysicsBody1sRel() const
{
    return GetPrim().CreateRelationship(PhysxSchemaTokens->physicsBody1s,
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
PhysxSchemaPhysxPhysicsJointInstancer::GetSchemaAttributeNames(bool includeInherited)
{
    static TfTokenVector localNames = {
        PhysxSchemaTokens->physicsBody0Indices,
        PhysxSchemaTokens->physicsBody1Indices,
        PhysxSchemaTokens->physicsLocalPos0s,
        PhysxSchemaTokens->physicsLocalRot0s,
        PhysxSchemaTokens->physicsLocalPos1s,
        PhysxSchemaTokens->physicsLocalRot1s,
    };
    static TfTokenVector allNames =
        _ConcatenateAttributeNames(
            PhysxSchemaPhysxPhysicsInstancer::GetSchemaAttributeNames(true),
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
