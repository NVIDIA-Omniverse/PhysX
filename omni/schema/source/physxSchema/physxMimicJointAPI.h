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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXMIMICJOINTAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXMIMICJOINTAPI_H

/// \file physxSchema/physxMimicJointAPI.h

#include "pxr/pxr.h"
#include ".//api.h"
#include "pxr/usd/usd/apiSchemaBase.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include ".//tokens.h"

#include "pxr/base/vt/value.h"

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/matrix4d.h"

#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"

PXR_NAMESPACE_OPEN_SCOPE

class SdfAssetPath;

// -------------------------------------------------------------------------- //
// PHYSXMIMICJOINTAPI                                                         //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxMimicJointAPI
///
/// Applied to a Physics Joint that must be part of an articulation. Supported joint types are:
/// PhysicsRevoluteJoint (with a limit set), PhysicsPrismaticJoint as well as the more generic
/// PhysicsJoint if all linear degrees of freedom are locked.
/// 
/// This schema allows to interconnect the degrees of freedom of two joints according to the relationship:
/// jointPosition + (gearing * referenceJointPosition) + offset = 0. This can be used to have, for example,
/// one joint mimic the motion of another joint. The term "position" does not necessarily mean a translational
/// unit, in the case of an angular degree of freedom it is an angle. The mimic joint operates on a single
/// degree of freedom which is specified through the instance name of this multiple apply schema. The allowed
/// instance name tokens are "rotX", "rotY" and "rotZ". See the documentation of the attribute referenceJointAxis
/// for more info related to specifying the degree of freedom to operate on. Notes: 1) the two joints have to
/// be part of the same articulation. 2) once the simulation has started, a significant performance penalty will
/// be incurred if a mimic joint is added or removed, if the referenceJoint relationship is changed or if the
/// referenceJointAxis attributes is modified. 3) the mimic joint relationship is interpreted as a two-way
/// interaction and an impulse will get applied to the reference joint as well.
///
/// For any described attribute \em Fallback \em Value or \em Allowed \em Values below
/// that are text/tokens, the actual token is published and defined in \ref PhysxSchemaTokens.
/// So to set an attribute to the value "rightHanded", use PhysxSchemaTokens->rightHanded
/// as the value.
///
class PhysxSchemaPhysxMimicJointAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::MultipleApplyAPI;

    /// Construct a PhysxSchemaPhysxMimicJointAPI on UsdPrim \p prim with
    /// name \p name . Equivalent to
    /// PhysxSchemaPhysxMimicJointAPI::Get(
    ///    prim.GetStage(),
    ///    prim.GetPath().AppendProperty(
    ///        "physxMimicJoint:name"));
    ///
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxMimicJointAPI(
        const UsdPrim& prim=UsdPrim(), const TfToken &name=TfToken())
        : UsdAPISchemaBase(prim, /*instanceName*/ name)
    { }

    /// Construct a PhysxSchemaPhysxMimicJointAPI on the prim held by \p schemaObj with
    /// name \p name.  Should be preferred over
    /// PhysxSchemaPhysxMimicJointAPI(schemaObj.GetPrim(), name), as it preserves
    /// SchemaBase state.
    explicit PhysxSchemaPhysxMimicJointAPI(
        const UsdSchemaBase& schemaObj, const TfToken &name)
        : UsdAPISchemaBase(schemaObj, /*instanceName*/ name)
    { }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxMimicJointAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes for a given instance name.  Does not
    /// include attributes that may be authored by custom/extended methods of
    /// the schemas involved. The names returned will have the proper namespace
    /// prefix.
    PHYSXSCHEMA_API
    static TfTokenVector
    GetSchemaAttributeNames(bool includeInherited, const TfToken &instanceName);

    /// Returns the name of this multiple-apply schema instance
    TfToken GetName() const {
        return _GetInstanceName();
    }

    /// Return a PhysxSchemaPhysxMimicJointAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  \p path must be of the format
    /// <path>.physxMimicJoint:name .
    ///
    /// This is shorthand for the following:
    ///
    /// \code
    /// TfToken name = SdfPath::StripNamespace(path.GetToken());
    /// PhysxSchemaPhysxMimicJointAPI(
    ///     stage->GetPrimAtPath(path.GetPrimPath()), name);
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxMimicJointAPI
    Get(const UsdStagePtr &stage, const SdfPath &path);

    /// Return a PhysxSchemaPhysxMimicJointAPI with name \p name holding the
    /// prim \p prim. Shorthand for PhysxSchemaPhysxMimicJointAPI(prim, name);
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxMimicJointAPI
    Get(const UsdPrim &prim, const TfToken &name);

    /// Return a vector of all named instances of PhysxSchemaPhysxMimicJointAPI on the 
    /// given \p prim.
    PHYSXSCHEMA_API
    static std::vector<PhysxSchemaPhysxMimicJointAPI>
    GetAll(const UsdPrim &prim);

    /// Checks if the given name \p baseName is the base name of a property
    /// of PhysxMimicJointAPI.
    PHYSXSCHEMA_API
    static bool
    IsSchemaPropertyBaseName(const TfToken &baseName);

    /// Checks if the given path \p path is of an API schema of type
    /// PhysxMimicJointAPI. If so, it stores the instance name of
    /// the schema in \p name and returns true. Otherwise, it returns false.
    PHYSXSCHEMA_API
    static bool
    IsPhysxMimicJointAPIPath(const SdfPath &path, TfToken *name);

    /// Returns true if this <b>multiple-apply</b> API schema can be applied,
    /// with the given instance name, \p name, to the given \p prim. If this 
    /// schema can not be a applied the prim, this returns false and, if 
    /// provided, populates \p whyNot with the reason it can not be applied.
    /// 
    /// Note that if CanApply returns false, that does not necessarily imply
    /// that calling Apply will fail. Callers are expected to call CanApply
    /// before calling Apply if they want to ensure that it is valid to 
    /// apply a schema.
    /// 
    /// \sa UsdPrim::GetAppliedSchemas()
    /// \sa UsdPrim::HasAPI()
    /// \sa UsdPrim::CanApplyAPI()
    /// \sa UsdPrim::ApplyAPI()
    /// \sa UsdPrim::RemoveAPI()
    ///
    PHYSXSCHEMA_API
    static bool 
    CanApply(const UsdPrim &prim, const TfToken &name, 
             std::string *whyNot=nullptr);

    /// Applies this <b>multiple-apply</b> API schema to the given \p prim 
    /// along with the given instance name, \p name. 
    /// 
    /// This information is stored by adding "PhysxMimicJointAPI:<i>name</i>" 
    /// to the token-valued, listOp metadata \em apiSchemas on the prim.
    /// For example, if \p name is 'instance1', the token 
    /// 'PhysxMimicJointAPI:instance1' is added to 'apiSchemas'.
    /// 
    /// \return A valid PhysxSchemaPhysxMimicJointAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxMimicJointAPI object is returned upon 
    /// failure. See \ref UsdPrim::ApplyAPI() for 
    /// conditions resulting in failure. 
    /// 
    /// \sa UsdPrim::GetAppliedSchemas()
    /// \sa UsdPrim::HasAPI()
    /// \sa UsdPrim::CanApplyAPI()
    /// \sa UsdPrim::ApplyAPI()
    /// \sa UsdPrim::RemoveAPI()
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxMimicJointAPI 
    Apply(const UsdPrim &prim, const TfToken &name);

protected:
    /// Returns the kind of schema this class belongs to.
    ///
    /// \sa UsdSchemaKind
    PHYSXSCHEMA_API
    UsdSchemaKind _GetSchemaKind() const override;

private:
    // needs to invoke _GetStaticTfType.
    friend class UsdSchemaRegistry;
    PHYSXSCHEMA_API
    static const TfType &_GetStaticTfType();

    static bool _IsTypedSchema();

    // override SchemaBase virtuals.
    PHYSXSCHEMA_API
    const TfType &_GetTfType() const override;

public:
    // --------------------------------------------------------------------- //
    // REFERENCEJOINTAXIS 
    // --------------------------------------------------------------------- //
    /// The axis that marks the degree of freedom of the reference joint to mimic. Has to be a valid
    /// degree of freedom on the reference joint. For joint types with a single degree of freedom
    /// (like PhysicsRevoluteJoint or PhysicsPrismaticJoint), this attribute will be ignored since the
    /// axis is defined implicitly. The mimic joint connects <targetJoint, targetJointAxis> to
    /// <referenceJoint, referenceJointAxis> with targetJoint being the prim that has PhysxMimicJointAPI
    /// applied and targetJointAxis being defined through the instance name token of PhysxMimicJointAPI.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token referenceJointAxis = "rotX"` |
    /// | C++ Type | TfToken |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | rotX, rotY, rotZ |
    PHYSXSCHEMA_API
    UsdAttribute GetReferenceJointAxisAttr() const;

    /// See GetReferenceJointAxisAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateReferenceJointAxisAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GEARING 
    // --------------------------------------------------------------------- //
    /// The gearing coefficient (see relationship formulation in the documentation part of PhysxMimicJointAPI).
    /// Range: (-inf, inf)
    /// Units: 
    /// if linear to angular: degrees / distance
    /// if angular to linear: distance / degrees
    /// Otherwise the value is unitless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float gearing = 1` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetGearingAttr() const;

    /// See GetGearingAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGearingAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // OFFSET 
    // --------------------------------------------------------------------- //
    /// The offset coefficient (see relationship formulation in the documentation part of PhysxMimicJointAPI).
    /// Range: (-inf, inf)
    /// Units: distance or degrees (depending on whether the mimic joint's degree of freedom is linear or angular)
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float offset = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetOffsetAttr() const;

    /// See GetOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // REFERENCEJOINT 
    // --------------------------------------------------------------------- //
    /// The joint that shall be mimicked. Relationship to a prim that has the PhysicsJoint API schema
    /// applied. For the supported joint types, see the general documentation of PhysxMimicJointAPI. The
    /// reference joint as well as the joint that mimics the reference joint have to be part of the same
    /// articulation (see PhysicsArticulationRootAPI).
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetReferenceJointRel() const;

    /// See GetReferenceJointRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateReferenceJointRel() const;

public:
    // ===================================================================== //
    // Feel free to add custom code below this line, it will be preserved by 
    // the code generator. 
    //
    // Just remember to: 
    //  - Close the class declaration with }; 
    //  - Close the namespace with PXR_NAMESPACE_CLOSE_SCOPE
    //  - Close the include guard with #endif
    // ===================================================================== //
    // --(BEGIN CUSTOM CODE)--
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif
