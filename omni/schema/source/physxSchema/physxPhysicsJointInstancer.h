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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXPHYSICSJOINTINSTANCER_H
#define PHYSXSCHEMA_GENERATED_PHYSXPHYSICSJOINTINSTANCER_H

/// \file physxSchema/physxPhysicsJointInstancer.h

#include "pxr/pxr.h"
#include ".//api.h"
#include ".//physxPhysicsInstancer.h"
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
// PHYSXPHYSICSJOINTINSTANCER                                                 //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxPhysicsJointInstancer
///
/// Class to instance physics joints from a set of prototype joints. The prototypes
/// in the PhysxPhysicsInstancer base class (see \em prototypes relationship) are expected
/// to be UsdPhysicsJoint prim types.
///
class PhysxSchemaPhysxPhysicsJointInstancer : public PhysxSchemaPhysxPhysicsInstancer
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::ConcreteTyped;

    /// Construct a PhysxSchemaPhysxPhysicsJointInstancer on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxPhysicsJointInstancer::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxPhysicsJointInstancer(const UsdPrim& prim=UsdPrim())
        : PhysxSchemaPhysxPhysicsInstancer(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxPhysicsJointInstancer on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxPhysicsJointInstancer(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxPhysicsJointInstancer(const UsdSchemaBase& schemaObj)
        : PhysxSchemaPhysxPhysicsInstancer(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxPhysicsJointInstancer();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxPhysicsJointInstancer holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxPhysicsJointInstancer(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxPhysicsJointInstancer
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
    static PhysxSchemaPhysxPhysicsJointInstancer
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
    // PHYSICSBODY0INDICES 
    // --------------------------------------------------------------------- //
    /// <b>Optional property</b>. Only applicable if body0s is a point instancer. 
    /// An index specifies the per-instance body0 rel as the rigid body at the body0s-instancer's protoIndices[index].
    /// 
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] physics:body0Indices` |
    /// | C++ Type | VtArray<int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetPhysicsBody0IndicesAttr() const;

    /// See GetPhysicsBody0IndicesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePhysicsBody0IndicesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PHYSICSBODY1INDICES 
    // --------------------------------------------------------------------- //
    /// <b>Optional property</b>. Only applicable if body1s is a point instancer. 
    /// An index specifies the per-instance body1 rel as the rigid body at the body1s-instancer's protoIndices[index].
    /// 
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] physics:body1Indices` |
    /// | C++ Type | VtArray<int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetPhysicsBody1IndicesAttr() const;

    /// See GetPhysicsBody1IndicesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePhysicsBody1IndicesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PHYSICSLOCALPOS0S 
    // --------------------------------------------------------------------- //
    /// <b>Required property</b>. Per-instance localPos0. This transformation
    /// is added on top of the joint localPos0.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `point3f[] physics:localPos0s` |
    /// | C++ Type | VtArray<GfVec3f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Point3fArray |
    PHYSXSCHEMA_API
    UsdAttribute GetPhysicsLocalPos0sAttr() const;

    /// See GetPhysicsLocalPos0sAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePhysicsLocalPos0sAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PHYSICSLOCALROT0S 
    // --------------------------------------------------------------------- //
    /// <b>Required property</b>. Per-instance localRot0. This transformation
    /// is added on top of the joint localRot0.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `quath[] physics:localRot0s` |
    /// | C++ Type | VtArray<GfQuath> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->QuathArray |
    PHYSXSCHEMA_API
    UsdAttribute GetPhysicsLocalRot0sAttr() const;

    /// See GetPhysicsLocalRot0sAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePhysicsLocalRot0sAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PHYSICSLOCALPOS1S 
    // --------------------------------------------------------------------- //
    /// <b>Required property</b>. Per-instance localPos1. This transformation
    /// is added on top of the joint localPos1.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `point3f[] physics:localPos1s` |
    /// | C++ Type | VtArray<GfVec3f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Point3fArray |
    PHYSXSCHEMA_API
    UsdAttribute GetPhysicsLocalPos1sAttr() const;

    /// See GetPhysicsLocalPos1sAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePhysicsLocalPos1sAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PHYSICSLOCALROT1S 
    // --------------------------------------------------------------------- //
    /// <b>Required property</b>. Per-instance localRot1. This transformation
    /// is added on top of the joint localRot1.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `quath[] physics:localRot1s` |
    /// | C++ Type | VtArray<GfQuath> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->QuathArray |
    PHYSXSCHEMA_API
    UsdAttribute GetPhysicsLocalRot1sAttr() const;

    /// See GetPhysicsLocalRot1sAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePhysicsLocalRot1sAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PHYSICSBODY0S 
    // --------------------------------------------------------------------- //
    /// <b>Required property</b>. The rel must contain either exactly one 
    /// UsdGeomPointInstancer that instances rigid bodies; or one or more rigid-body prims in the stage.
    /// If the rel is to a point instancer, the body0Indices must be specified. 
    /// 
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetPhysicsBody0sRel() const;

    /// See GetPhysicsBody0sRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreatePhysicsBody0sRel() const;

public:
    // --------------------------------------------------------------------- //
    // PHYSICSBODY1S 
    // --------------------------------------------------------------------- //
    /// <b>Required property</b>. The rel must contain either exactly one 
    /// UsdGeomPointInstancer that instances rigid bodies; or one or more rigid-body prims in the stage.
    /// If the rel is to a point instancer, the body1Indices must be specified.
    /// 
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetPhysicsBody1sRel() const;

    /// See GetPhysicsBody1sRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreatePhysicsBody1sRel() const;

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
