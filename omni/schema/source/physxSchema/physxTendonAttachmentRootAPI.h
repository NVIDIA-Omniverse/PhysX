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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXTENDONATTACHMENTROOTAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXTENDONATTACHMENTROOTAPI_H

/// \file physxSchema/physxTendonAttachmentRootAPI.h

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
// PHYSXTENDONATTACHMENTROOTAPI                                               //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxTendonAttachmentRootAPI
///
/// WARNING: Draft API, this design is not fixed and may change in the future.
/// 
/// Applied to an articulation-link rigid-body Xformable.
/// 
/// The root API creates a new spatial tendon tree. The root tendon attachment is the only attachment in a tendon tree
/// that does not have a parent, so the parentLink, parentAttachment, and gearing attributes inherited from
/// PhysxTendonAttachmentAPI are ignored for a root.
/// 
/// Spatial tendons create line-of-sight distance constraints between links of a single articulation. In particular,
/// spatial tendons run through attachments that are positioned relative to a rigid-body link, and their length is
/// defined as a weighted sum of the distance between the attachments in the tendon.
/// 
/// Spatial tendons may branch, in which case the tendon splits up into multiple conceptual sub-tendons, one for each
/// root-to-leaf path in the tendon tree. The tendon tree starts at the root, and its topology is defined by the
/// attachments' parentLink rel and parentAttachment token.
/// 
/// It is possible to create multiple attachments per link, see PhysxTendonAttachmentAPI.
/// 
/// Details on dynamics:
/// 
/// The length of a sub-tendon in the tree is
/// 
/// subTendonLength = sum(gearing[i] * |p[i] - p_parent[i]|)
/// 
/// where the sum is evaluated along the unique tree path between root and leaf. The gearing[i] is the i-th attachment's
/// gearing, and |p[i] - p_parent[i]| is the distance between the positions of the i-th attachment and its parent
/// attachment.
/// 
/// Each subtendon has spring-damper dynamics acting on the length constraint
/// 
/// constraint = 0 = offset + subTendonLength - restLength
/// 
/// where the offset is a common summand for all sub-tendons of a spatial tendon, and the restLength is specific to a
/// sub-tendon, see the PhysxTendonAttachmentLeafAPI.
/// 
/// If limits are not active, the sub-tendon force that acts on the leaf is
/// 
/// F = stiffness * constraint + damping * tendonVelocity
/// 
/// where tendonVelocity is the sum of the time derivatives of the line-of-sight distances between
/// 1) the leaf and its parent attachment and
/// 2) the tendon root and its first child attachment on the path to the leaf.
/// Stiffness and damping are common parameters for all sub-tendons.
/// 
/// The 3D force applied at the leaf attachment is equal to 
/// 
/// F * (p_parent - p_leaf).
/// 
/// where p_leaf and p_parent are the leaf's position and its parent's position, respectively. The reaction force acting
/// on the root attachment is
/// 
/// F * (p_child_leaf - p_root)
/// 
/// where p_child_leaf is the position of the root's child attachment that leads to the leaf that produces F,
/// and p_root is the root attachment position. The tendon force is not further propagated through the tendon, so at
/// intermediate attachments (i.e. created by PhysxTendonAttachmentAPI) no forces are applied.
/// 
/// Note that a spatial tendon may both pull and push on the leaf and root attachments. A string-like, one-sided constraint
/// can be implemented using the sub-tendon's length limits.
/// 
/// Sub-tendon length limits constrain the offset length
/// 
/// lowerLimit <= subTendonLength + offset<= upperLimit
/// 
/// and, when active, add a restoring spring force parametrized by limitStiffness to the tendon force, analogous to the
/// length constraint force above. Limit dynamics are damped by the tendon-length damping that is applied regardless of
/// a limit being active. Limit parameters are attributes of the PhysxTendonAttachmentLeafAPI.
///
class PhysxSchemaPhysxTendonAttachmentRootAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::MultipleApplyAPI;

    /// Construct a PhysxSchemaPhysxTendonAttachmentRootAPI on UsdPrim \p prim with
    /// name \p name . Equivalent to
    /// PhysxSchemaPhysxTendonAttachmentRootAPI::Get(
    ///    prim.GetStage(),
    ///    prim.GetPath().AppendProperty(
    ///        "physxTendon:name"));
    ///
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxTendonAttachmentRootAPI(
        const UsdPrim& prim=UsdPrim(), const TfToken &name=TfToken())
        : UsdAPISchemaBase(prim, /*instanceName*/ name)
    { }

    /// Construct a PhysxSchemaPhysxTendonAttachmentRootAPI on the prim held by \p schemaObj with
    /// name \p name.  Should be preferred over
    /// PhysxSchemaPhysxTendonAttachmentRootAPI(schemaObj.GetPrim(), name), as it preserves
    /// SchemaBase state.
    explicit PhysxSchemaPhysxTendonAttachmentRootAPI(
        const UsdSchemaBase& schemaObj, const TfToken &name)
        : UsdAPISchemaBase(schemaObj, /*instanceName*/ name)
    { }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxTendonAttachmentRootAPI();

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

    /// Return a PhysxSchemaPhysxTendonAttachmentRootAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  \p path must be of the format
    /// <path>.physxTendon:name .
    ///
    /// This is shorthand for the following:
    ///
    /// \code
    /// TfToken name = SdfPath::StripNamespace(path.GetToken());
    /// PhysxSchemaPhysxTendonAttachmentRootAPI(
    ///     stage->GetPrimAtPath(path.GetPrimPath()), name);
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxTendonAttachmentRootAPI
    Get(const UsdStagePtr &stage, const SdfPath &path);

    /// Return a PhysxSchemaPhysxTendonAttachmentRootAPI with name \p name holding the
    /// prim \p prim. Shorthand for PhysxSchemaPhysxTendonAttachmentRootAPI(prim, name);
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxTendonAttachmentRootAPI
    Get(const UsdPrim &prim, const TfToken &name);

    /// Return a vector of all named instances of PhysxSchemaPhysxTendonAttachmentRootAPI on the 
    /// given \p prim.
    PHYSXSCHEMA_API
    static std::vector<PhysxSchemaPhysxTendonAttachmentRootAPI>
    GetAll(const UsdPrim &prim);

    /// Checks if the given name \p baseName is the base name of a property
    /// of PhysxTendonAttachmentRootAPI.
    PHYSXSCHEMA_API
    static bool
    IsSchemaPropertyBaseName(const TfToken &baseName);

    /// Checks if the given path \p path is of an API schema of type
    /// PhysxTendonAttachmentRootAPI. If so, it stores the instance name of
    /// the schema in \p name and returns true. Otherwise, it returns false.
    PHYSXSCHEMA_API
    static bool
    IsPhysxTendonAttachmentRootAPIPath(const SdfPath &path, TfToken *name);

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
    /// This information is stored by adding "PhysxTendonAttachmentRootAPI:<i>name</i>" 
    /// to the token-valued, listOp metadata \em apiSchemas on the prim.
    /// For example, if \p name is 'instance1', the token 
    /// 'PhysxTendonAttachmentRootAPI:instance1' is added to 'apiSchemas'.
    /// 
    /// \return A valid PhysxSchemaPhysxTendonAttachmentRootAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxTendonAttachmentRootAPI object is returned upon 
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
    static PhysxSchemaPhysxTendonAttachmentRootAPI 
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
    // TENDONENABLED 
    // --------------------------------------------------------------------- //
    /// Enables/disables the tendon from contributing to the articulation dynamics.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool tendonEnabled = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetTendonEnabledAttr() const;

    /// See GetTendonEnabledAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTendonEnabledAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // STIFFNESS 
    // --------------------------------------------------------------------- //
    /// Common sub-tendon length spring stiffness.
    /// Range: [0, inf)
    /// Units: force / distance = mass / time / time
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float stiffness = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetStiffnessAttr() const;

    /// See GetStiffnessAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateStiffnessAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // DAMPING 
    // --------------------------------------------------------------------- //
    /// Common sub-tendon length damping.
    /// Range: [0, inf)
    /// Units: force / distance * time = mass / time
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float damping = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetDampingAttr() const;

    /// See GetDampingAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateDampingAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LIMITSTIFFNESS 
    // --------------------------------------------------------------------- //
    /// Common sub-tendon length-limit spring stiffness.
    /// Range: [0, inf)
    /// Units: force / distance = mass / time / time
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float limitStiffness = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLimitStiffnessAttr() const;

    /// See GetLimitStiffnessAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLimitStiffnessAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // OFFSET 
    // --------------------------------------------------------------------- //
    /// Common sub-tendon length offset.
    /// Range: (-inf, inf)
    /// Units: distance
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
