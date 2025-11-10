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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXLIMITAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXLIMITAPI_H

/// \file physxSchema/physxLimitAPI.h

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
// PHYSXLIMITAPI                                                              //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxLimitAPI
///
/// PhysX joint limit extended parameters. Can be applied to a prim that has PhysicsLimitAPI applied.
/// Note that this schema is only supported for joints that are NOT part of an articulation
/// (see PhysicsArticulationRootAPI). PhysxLimitAPI is a multipleApply schema and supports the same set
/// of schema instance names as PhysicsLimitAPI. The instance name used for the PhysxLimitAPI schema should
/// match the one used for the corresponding PhysicsLimitAPI.
///
class PhysxSchemaPhysxLimitAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::MultipleApplyAPI;

    /// Construct a PhysxSchemaPhysxLimitAPI on UsdPrim \p prim with
    /// name \p name . Equivalent to
    /// PhysxSchemaPhysxLimitAPI::Get(
    ///    prim.GetStage(),
    ///    prim.GetPath().AppendProperty(
    ///        "physxLimit:name"));
    ///
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxLimitAPI(
        const UsdPrim& prim=UsdPrim(), const TfToken &name=TfToken())
        : UsdAPISchemaBase(prim, /*instanceName*/ name)
    { }

    /// Construct a PhysxSchemaPhysxLimitAPI on the prim held by \p schemaObj with
    /// name \p name.  Should be preferred over
    /// PhysxSchemaPhysxLimitAPI(schemaObj.GetPrim(), name), as it preserves
    /// SchemaBase state.
    explicit PhysxSchemaPhysxLimitAPI(
        const UsdSchemaBase& schemaObj, const TfToken &name)
        : UsdAPISchemaBase(schemaObj, /*instanceName*/ name)
    { }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxLimitAPI();

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

    /// Return a PhysxSchemaPhysxLimitAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  \p path must be of the format
    /// <path>.physxLimit:name .
    ///
    /// This is shorthand for the following:
    ///
    /// \code
    /// TfToken name = SdfPath::StripNamespace(path.GetToken());
    /// PhysxSchemaPhysxLimitAPI(
    ///     stage->GetPrimAtPath(path.GetPrimPath()), name);
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxLimitAPI
    Get(const UsdStagePtr &stage, const SdfPath &path);

    /// Return a PhysxSchemaPhysxLimitAPI with name \p name holding the
    /// prim \p prim. Shorthand for PhysxSchemaPhysxLimitAPI(prim, name);
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxLimitAPI
    Get(const UsdPrim &prim, const TfToken &name);

    /// Return a vector of all named instances of PhysxSchemaPhysxLimitAPI on the 
    /// given \p prim.
    PHYSXSCHEMA_API
    static std::vector<PhysxSchemaPhysxLimitAPI>
    GetAll(const UsdPrim &prim);

    /// Checks if the given name \p baseName is the base name of a property
    /// of PhysxLimitAPI.
    PHYSXSCHEMA_API
    static bool
    IsSchemaPropertyBaseName(const TfToken &baseName);

    /// Checks if the given path \p path is of an API schema of type
    /// PhysxLimitAPI. If so, it stores the instance name of
    /// the schema in \p name and returns true. Otherwise, it returns false.
    PHYSXSCHEMA_API
    static bool
    IsPhysxLimitAPIPath(const SdfPath &path, TfToken *name);

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
    /// This information is stored by adding "PhysxLimitAPI:<i>name</i>" 
    /// to the token-valued, listOp metadata \em apiSchemas on the prim.
    /// For example, if \p name is 'instance1', the token 
    /// 'PhysxLimitAPI:instance1' is added to 'apiSchemas'.
    /// 
    /// \return A valid PhysxSchemaPhysxLimitAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxLimitAPI object is returned upon 
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
    static PhysxSchemaPhysxLimitAPI 
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
    // RESTITUTION 
    // --------------------------------------------------------------------- //
    /// Controls the amount of bounce when the joint hits a limit. The dynamics solver will target a post simulation
    /// velocity of: restitution * -v (v = velocity along the joint axes).
    /// Note that restitution/bounce can not be combined with the stiffness/damping attributes. Either the limit is a hard
    /// constraint and restitution>=0, stiffness=0, damping=0 or the limit is a soft constraint and restitution=0, (stiffness>0
    /// or damping>0)
    /// Range: [0, 1]
    /// Units: dimensionless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float restitution = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetRestitutionAttr() const;

    /// See GetRestitutionAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateRestitutionAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // BOUNCETHRESHOLD 
    // --------------------------------------------------------------------- //
    /// Determines the minimum impact velocity which will cause the joint to bounce off the limit. Only relevant
    /// if the restitution attribute is larger than zero. If the threshold is not reached, hitting the limit will be
    /// treated as if restitution was zero. See restitution for more details on bounce behavior.
    /// Range: [0, inf)
    /// Units: 
    /// if linear: distance / seconds
    /// if angular: degrees / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float bounceThreshold = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetBounceThresholdAttr() const;

    /// See GetBounceThresholdAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateBounceThresholdAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // STIFFNESS 
    // --------------------------------------------------------------------- //
    /// Spring stiffness for spring-damper limit dynamics. If greater than zero, the limit is soft, 
    /// i.e. a spring pulls the joint back to the limit. Can not be used in combination with restitution>0
    /// (for details see the docu on restitution). The applied force will be proportional to the length/angle
    /// by which the limit is violated. Let the violation be defined as geometricError = jointPosition - highLimit
    /// if jointPosition > highLimit, and geometricError = jointPosition - lowLimit if jointPosition < lowLimit
    /// (jointPosition being an angle in the case of an angular joint axis). In addition, a damping term can be
    /// specified (see attribute damping) such that the applied force can be roughly defined as:
    /// F = (stiffness * geometricError) - (damping * jointVelocity).
    /// Note that an implicit time-stepping scheme will be used to compute the force.
    /// The soft constraint will only be active when the distance/angle between the joint frame
    /// origins/orientations is below the "low" limit value or above the "high" limit value (see PhysicsLimitAPI).
    /// Range: [0, inf)
    /// Units: 
    /// if linear: force / distance = mass / seconds / seconds
    /// if angular: torque / degrees = mass * distance * distance / seconds / seconds / degrees
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
    /// Damping for spring-damper limit dynamics. If greater than zero, the limit is soft,
    /// i.e. a spring pulls the joint back to the limit. See the documentation of the stiffness attribute for details.
    /// Range: [0, inf)
    /// Units: 
    /// if linear: force * seconds / distance = mass / seconds
    /// if angular: torque * seconds / degrees = mass * distance * distance / seconds / degrees
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
