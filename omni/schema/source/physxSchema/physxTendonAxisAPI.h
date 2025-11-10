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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXTENDONAXISAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXTENDONAXISAPI_H

/// \file physxSchema/physxTendonAxisAPI.h

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
// PHYSXTENDONAXISAPI                                                         //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxTendonAxisAPI
///
/// WARNING: Draft API, this design is not fixed and may change in the future. At this point, we don't support
/// multi-axis joints (e.g. spherical, D6).
/// 
/// Applied to a Physics Joint that must be part of an articulation, e.g. PhysicsRevoluteJoint or PhysicsPrismaticJoint.
/// 
/// A tendon axis is part of a fixed tendon and contributes to the tendon length via the position of its associated
/// articulation joint axis and the gearing attribute. The tendon applies spring-damper forces to the articulation links,
/// scaled by the force coefficient, that aim to maintain constraints on the tendon lengths.
/// 
/// A joint may have multiple tendon axes that belong to distinct fixed tendons; therefore, the fixed-tendon APIs are
/// multi-apply and are grouped into tendons by their instance names.
/// 
/// Fixed tendons do not allow linking arbitrary joint axes of the articulation: The respective joints must all be
/// directly connected to each other in the articulation structure, i.e. each of the joints in the tendon must be
/// connected by a single articulation link to another joint in the same tendon. This implies that fixed tendons can
/// branch along with a branching articulation as well. In addition, the root tendon axis created by applying the
/// PhysxTendonAxisRootAPI must be applied to the articulation joint that is the common ancestor of all joint
/// axes in the fixed tendon.
/// 
/// In a future version, it will be possible to include multiple axes of a multi-axis joint (e.g. a spherical joint)
/// in the fixed tendon by adding them to the axis token array, and setting their gearing and forceCoefficient accordingly
/// in the respective float arrays (in the same order). Until then, the jointAxis token array is ignored, and only the first
/// element of the gearing array is considered.
/// 
///
class PhysxSchemaPhysxTendonAxisAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::MultipleApplyAPI;

    /// Construct a PhysxSchemaPhysxTendonAxisAPI on UsdPrim \p prim with
    /// name \p name . Equivalent to
    /// PhysxSchemaPhysxTendonAxisAPI::Get(
    ///    prim.GetStage(),
    ///    prim.GetPath().AppendProperty(
    ///        "physxTendon:name"));
    ///
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxTendonAxisAPI(
        const UsdPrim& prim=UsdPrim(), const TfToken &name=TfToken())
        : UsdAPISchemaBase(prim, /*instanceName*/ name)
    { }

    /// Construct a PhysxSchemaPhysxTendonAxisAPI on the prim held by \p schemaObj with
    /// name \p name.  Should be preferred over
    /// PhysxSchemaPhysxTendonAxisAPI(schemaObj.GetPrim(), name), as it preserves
    /// SchemaBase state.
    explicit PhysxSchemaPhysxTendonAxisAPI(
        const UsdSchemaBase& schemaObj, const TfToken &name)
        : UsdAPISchemaBase(schemaObj, /*instanceName*/ name)
    { }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxTendonAxisAPI();

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

    /// Return a PhysxSchemaPhysxTendonAxisAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  \p path must be of the format
    /// <path>.physxTendon:name .
    ///
    /// This is shorthand for the following:
    ///
    /// \code
    /// TfToken name = SdfPath::StripNamespace(path.GetToken());
    /// PhysxSchemaPhysxTendonAxisAPI(
    ///     stage->GetPrimAtPath(path.GetPrimPath()), name);
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxTendonAxisAPI
    Get(const UsdStagePtr &stage, const SdfPath &path);

    /// Return a PhysxSchemaPhysxTendonAxisAPI with name \p name holding the
    /// prim \p prim. Shorthand for PhysxSchemaPhysxTendonAxisAPI(prim, name);
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxTendonAxisAPI
    Get(const UsdPrim &prim, const TfToken &name);

    /// Return a vector of all named instances of PhysxSchemaPhysxTendonAxisAPI on the 
    /// given \p prim.
    PHYSXSCHEMA_API
    static std::vector<PhysxSchemaPhysxTendonAxisAPI>
    GetAll(const UsdPrim &prim);

    /// Checks if the given name \p baseName is the base name of a property
    /// of PhysxTendonAxisAPI.
    PHYSXSCHEMA_API
    static bool
    IsSchemaPropertyBaseName(const TfToken &baseName);

    /// Checks if the given path \p path is of an API schema of type
    /// PhysxTendonAxisAPI. If so, it stores the instance name of
    /// the schema in \p name and returns true. Otherwise, it returns false.
    PHYSXSCHEMA_API
    static bool
    IsPhysxTendonAxisAPIPath(const SdfPath &path, TfToken *name);

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
    /// This information is stored by adding "PhysxTendonAxisAPI:<i>name</i>" 
    /// to the token-valued, listOp metadata \em apiSchemas on the prim.
    /// For example, if \p name is 'instance1', the token 
    /// 'PhysxTendonAxisAPI:instance1' is added to 'apiSchemas'.
    /// 
    /// \return A valid PhysxSchemaPhysxTendonAxisAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxTendonAxisAPI object is returned upon 
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
    static PhysxSchemaPhysxTendonAxisAPI 
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
    // GEARING 
    // --------------------------------------------------------------------- //
    /// Joint gearing(s) per joint axis in axis token array, in the same order.
    /// Range: (-inf, inf)
    /// Units: 
    /// if translational axis: unitless
    /// if rotational axis: distance/degrees
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] gearing = [1]` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
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
    // FORCECOEFFICIENT 
    // --------------------------------------------------------------------- //
    /// Joint force coefficient(s) per joint axis in axis token array, in the same order.
    /// Range: (-inf, inf)
    /// Units: 
    /// if translational axis: unitless
    /// if rotational axis: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] forceCoefficient = [1]` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetForceCoefficientAttr() const;

    /// See GetForceCoefficientAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateForceCoefficientAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // JOINTAXIS 
    // --------------------------------------------------------------------- //
    /// Specifies target joint axis/axes. Ignored when the joint only has a single axis, e.g. a revolute joint.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token[] jointAxis` |
    /// | C++ Type | VtArray<TfToken> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->TokenArray |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | transX, transY, transZ, rotX, rotY, rotZ |
    PHYSXSCHEMA_API
    UsdAttribute GetJointAxisAttr() const;

    /// See GetJointAxisAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateJointAxisAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
