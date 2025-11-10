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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXPHYSICSATTACHMENT_H
#define PHYSXSCHEMA_GENERATED_PHYSXPHYSICSATTACHMENT_H

/// \file physxSchema/physxPhysicsAttachment.h

#include "pxr/pxr.h"
#include ".//api.h"
#include "pxr/usd/usd/typed.h"
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
// PHYSXPHYSICSATTACHMENT                                                     //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxPhysicsAttachment
///
/// Deprecated: Will be replaced by a new deformable schema in a future release.
/// Represents attachments between physics actors, for example, between a rigid body and a deformable body, or a deformable body and a particle cloth.
///
/// For any described attribute \em Fallback \em Value or \em Allowed \em Values below
/// that are text/tokens, the actual token is published and defined in \ref PhysxSchemaTokens.
/// So to set an attribute to the value "rightHanded", use PhysxSchemaTokens->rightHanded
/// as the value.
///
class PhysxSchemaPhysxPhysicsAttachment : public UsdTyped
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::ConcreteTyped;

    /// Construct a PhysxSchemaPhysxPhysicsAttachment on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxPhysicsAttachment::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxPhysicsAttachment(const UsdPrim& prim=UsdPrim())
        : UsdTyped(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxPhysicsAttachment on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxPhysicsAttachment(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxPhysicsAttachment(const UsdSchemaBase& schemaObj)
        : UsdTyped(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxPhysicsAttachment();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxPhysicsAttachment holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxPhysicsAttachment(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxPhysicsAttachment
    Get(const UsdStagePtr &stage, const SdfPath &path);

    /// Attempt to ensure a \a UsdPrim adhering to this schema at \p path
    /// is defined (according to UsdPrim::IsDefined()) on this stage.
    ///
    /// If a prim adhering to this schema at \p path is already defined on this
    /// stage, return that prim.  Otherwise author an \a SdfPrimSpec with
    /// \a specifier == \a SdfSpecifierDef and this schema's prim type name for
    /// the prim at \p path at the current EditTarget.  Author \a SdfPrimSpec s
    /// with \p specifier == \a SdfSpecifierDef and empty typeName at the
    /// current EditTarget for any nonexistent, or existing but not \a Defined
    /// ancestors.
    ///
    /// The given \a path must be an absolute prim path that does not contain
    /// any variant selections.
    ///
    /// If it is impossible to author any of the necessary PrimSpecs, (for
    /// example, in case \a path cannot map to the current UsdEditTarget's
    /// namespace) issue an error and return an invalid \a UsdPrim.
    ///
    /// Note that this method may return a defined prim whose typeName does not
    /// specify this schema class, in case a stronger typeName opinion overrides
    /// the opinion at the current EditTarget.
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxPhysicsAttachment
    Define(const UsdStagePtr &stage, const SdfPath &path);

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
    // ATTACHMENTENABLED 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Enable or disable the attachment.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool attachmentEnabled = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetAttachmentEnabledAttr() const;

    /// See GetAttachmentEnabledAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateAttachmentEnabledAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // POINTS0 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Attachment points in Actor 0 local space, defined in the actor's rest state, if it is deformable. Elements correspond one-to-one to elements in points1 attribute.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `point3f[] points0` |
    /// | C++ Type | VtArray<GfVec3f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Point3fArray |
    PHYSXSCHEMA_API
    UsdAttribute GetPoints0Attr() const;

    /// See GetPoints0Attr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePoints0Attr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // POINTS1 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Attachment points in Actor 1 local space, defined in the actor's rest state, if it is deformable. Elements correspond one-to-one to elements in points0 attribute.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `point3f[] points1` |
    /// | C++ Type | VtArray<GfVec3f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Point3fArray |
    PHYSXSCHEMA_API
    UsdAttribute GetPoints1Attr() const;

    /// See GetPoints1Attr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePoints1Attr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // COLLISIONFILTERINDICES0 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Indices to geometry of Actor 0 that should not generate collisions with Actor 1 as specified by filterType0. Ignored for rigid bodies.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint[] collisionFilterIndices0` |
    /// | C++ Type | VtArray<unsigned int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UIntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetCollisionFilterIndices0Attr() const;

    /// See GetCollisionFilterIndices0Attr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCollisionFilterIndices0Attr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FILTERTYPE0 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Specify if indices in collisionFilterIndices0 correspond to vertices; or mesh cell-geometry, i.e. triangles, tetrahedrons, etc.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token filterType0` |
    /// | C++ Type | TfToken |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | Vertices, Geometry |
    PHYSXSCHEMA_API
    UsdAttribute GetFilterType0Attr() const;

    /// See GetFilterType0Attr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFilterType0Attr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // COLLISIONFILTERINDICES1 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Indices to mesh triangle/tet/hex/etc. of Actor 1 that should not generate collisions with Actor 0. Ignored for rigid bodies.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint[] collisionFilterIndices1` |
    /// | C++ Type | VtArray<unsigned int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UIntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetCollisionFilterIndices1Attr() const;

    /// See GetCollisionFilterIndices1Attr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCollisionFilterIndices1Attr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FILTERTYPE1 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Specify if indices in collisionFilterIndices1 correspond to vertices; or mesh cell-geometry, i.e. triangles, tetrahedrons, etc.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token filterType1` |
    /// | C++ Type | TfToken |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | Vertices, Geometry |
    PHYSXSCHEMA_API
    UsdAttribute GetFilterType1Attr() const;

    /// See GetFilterType1Attr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFilterType1Attr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ACTOR0 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Reference to the first actor.
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetActor0Rel() const;

    /// See GetActor0Rel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateActor0Rel() const;

public:
    // --------------------------------------------------------------------- //
    // ACTOR1 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Reference to the second actor.
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetActor1Rel() const;

    /// See GetActor1Rel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateActor1Rel() const;

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

public:

    /// Helper attribute access via index in [0,1].
    /// See GetPoints0Attr() and GetPoints1Attr()
    PHYSXSCHEMA_API
    UsdAttribute GetPointsAttr(int index) const;

    /// Helper attribute access via index in [0,1].
    /// See GetCollisionFilterIndices0Attr() and GetCollisionFilterIndices1Attr()
    PHYSXSCHEMA_API
    UsdAttribute GetCollisionFilterIndicesAttr(int index) const;

    /// Helper attribute access via index in [0,1].
    /// See GetFilterType0Attr() and GetFilterType1Attr()
    PHYSXSCHEMA_API
    UsdAttribute GetFilterTypeAttr(int index) const;

    /// Helper attribute access via index in [0,1].
    /// See GetActor0Rel() and GetActor1Rel()
    PHYSXSCHEMA_API
    UsdRelationship GetActorRel(int index) const;

};

PXR_NAMESPACE_CLOSE_SCOPE

#endif
