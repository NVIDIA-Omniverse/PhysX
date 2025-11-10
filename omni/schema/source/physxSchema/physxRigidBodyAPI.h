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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXRIGIDBODYAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXRIGIDBODYAPI_H

/// \file physxSchema/physxRigidBodyAPI.h

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
// PHYSXRIGIDBODYAPI                                                          //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxRigidBodyAPI
///
/// PhysX rigid body extended parameters
///
class PhysxSchemaPhysxRigidBodyAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxRigidBodyAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxRigidBodyAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxRigidBodyAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxRigidBodyAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxRigidBodyAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxRigidBodyAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxRigidBodyAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxRigidBodyAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxRigidBodyAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxRigidBodyAPI
    Get(const UsdStagePtr &stage, const SdfPath &path);


    /// Returns true if this <b>single-apply</b> API schema can be applied to 
    /// the given \p prim. If this schema can not be a applied to the prim, 
    /// this returns false and, if provided, populates \p whyNot with the 
    /// reason it can not be applied.
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
    CanApply(const UsdPrim &prim, std::string *whyNot=nullptr);

    /// Applies this <b>single-apply</b> API schema to the given \p prim.
    /// This information is stored by adding "PhysxRigidBodyAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxRigidBodyAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxRigidBodyAPI object is returned upon 
    /// failure. See \ref UsdPrim::ApplyAPI() for conditions 
    /// resulting in failure. 
    /// 
    /// \sa UsdPrim::GetAppliedSchemas()
    /// \sa UsdPrim::HasAPI()
    /// \sa UsdPrim::CanApplyAPI()
    /// \sa UsdPrim::ApplyAPI()
    /// \sa UsdPrim::RemoveAPI()
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxRigidBodyAPI 
    Apply(const UsdPrim &prim);

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
    // LINEARDAMPING 
    // --------------------------------------------------------------------- //
    /// Linear damping coefficient.
    /// Range: [0, inf)
    /// Units: dimensionless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:linearDamping = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLinearDampingAttr() const;

    /// See GetLinearDampingAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLinearDampingAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ANGULARDAMPING 
    // --------------------------------------------------------------------- //
    /// Angular damping coefficient.
    /// Range: [0, inf)
    /// Units: dimensionless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:angularDamping = 0.05` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetAngularDampingAttr() const;

    /// See GetAngularDampingAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateAngularDampingAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXLINEARVELOCITY 
    // --------------------------------------------------------------------- //
    /// Maximum allowable linear velocity for the rigid body.
    /// Range: [0, inf)
    /// Units: distance / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:maxLinearVelocity = inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxLinearVelocityAttr() const;

    /// See GetMaxLinearVelocityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxLinearVelocityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXANGULARVELOCITY 
    // --------------------------------------------------------------------- //
    /// Maximum allowable angular velocity for rigid body.
    /// Range: [0, inf)
    /// Units: degrees / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:maxAngularVelocity = 5729.58` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxAngularVelocityAttr() const;

    /// See GetMaxAngularVelocityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxAngularVelocityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SLEEPTHRESHOLD 
    // --------------------------------------------------------------------- //
    /// Mass-normalized kinetic energy threshold below which an actor may go to sleep.
    /// Range: [0, inf)
    /// Default: 0.00005 * tolerancesSpeed * tolerancesSpeed
    /// Units: distance * distance / seconds / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:sleepThreshold = 0.00005` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetSleepThresholdAttr() const;

    /// See GetSleepThresholdAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSleepThresholdAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // STABILIZATIONTHRESHOLD 
    // --------------------------------------------------------------------- //
    /// Mass-normalized kinetic energy threshold below which an actor may participate in stabilization.
    /// Range: [0, inf)
    /// Default: 0.00001 * tolerancesSpeed * tolerancesSpeed
    /// Units: distance * distance / seconds / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:stabilizationThreshold = 0.00001` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetStabilizationThresholdAttr() const;

    /// See GetStabilizationThresholdAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateStabilizationThresholdAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXDEPENETRATIONVELOCITY 
    // --------------------------------------------------------------------- //
    /// The maximum depenetration velocity permitted to be introduced by the solver.
    /// Range: [0, inf)
    /// Default: 5.0 * tolerancesLength
    /// Units: distance / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:maxDepenetrationVelocity = 3` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxDepenetrationVelocityAttr() const;

    /// See GetMaxDepenetrationVelocityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxDepenetrationVelocityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXCONTACTIMPULSE 
    // --------------------------------------------------------------------- //
    /// Sets a limit on the impulse that may be applied at a contact. The maximum impulse at a contact between two dynamic or kinematic
    /// bodies will be the minimum of the two limit values. For a collision between a static and a dynamic body, the impulse is limited
    /// by the value for the dynamic body.
    /// Range: [0, inf)
    /// Units: force * seconds = mass * distance / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:maxContactImpulse = inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxContactImpulseAttr() const;

    /// See GetMaxContactImpulseAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxContactImpulseAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SOLVERPOSITIONITERATIONCOUNT 
    // --------------------------------------------------------------------- //
    /// Solver position iteration counts for the body. Allowed range [1, 255].
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int physxRigidBody:solverPositionIterationCount = 16` |
    /// | C++ Type | int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
    PHYSXSCHEMA_API
    UsdAttribute GetSolverPositionIterationCountAttr() const;

    /// See GetSolverPositionIterationCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSolverPositionIterationCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SOLVERVELOCITYITERATIONCOUNT 
    // --------------------------------------------------------------------- //
    /// Solver velocity iteration counts for the body. Allowed range [0, 255].
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int physxRigidBody:solverVelocityIterationCount = 1` |
    /// | C++ Type | int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
    PHYSXSCHEMA_API
    UsdAttribute GetSolverVelocityIterationCountAttr() const;

    /// See GetSolverVelocityIterationCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSolverVelocityIterationCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLECCD 
    // --------------------------------------------------------------------- //
    /// Enables swept integration for the actor.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxRigidBody:enableCCD = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableCCDAttr() const;

    /// See GetEnableCCDAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableCCDAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLESPECULATIVECCD 
    // --------------------------------------------------------------------- //
    /// Register a rigid body to dynamically adjust contact offset based on velocity. This can be used to achieve a CCD effect.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxRigidBody:enableSpeculativeCCD = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableSpeculativeCCDAttr() const;

    /// See GetEnableSpeculativeCCDAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableSpeculativeCCDAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // RETAINACCELERATIONS 
    // --------------------------------------------------------------------- //
    /// Carries over forces/accelerations between frames, rather than clearing them.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxRigidBody:retainAccelerations = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetRetainAccelerationsAttr() const;

    /// See GetRetainAccelerationsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateRetainAccelerationsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLEGYROSCOPICFORCES 
    // --------------------------------------------------------------------- //
    /// Enables computation of gyroscopic forces on the rigid body.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxRigidBody:enableGyroscopicForces = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableGyroscopicForcesAttr() const;

    /// See GetEnableGyroscopicForcesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableGyroscopicForcesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // DISABLEGRAVITY 
    // --------------------------------------------------------------------- //
    /// Disable gravity for the actor.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxRigidBody:disableGravity = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetDisableGravityAttr() const;

    /// See GetDisableGravityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateDisableGravityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SOLVECONTACT 
    // --------------------------------------------------------------------- //
    /// Process the contacts of this rigid body in the dynamics solver.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxRigidBody:solveContact = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetSolveContactAttr() const;

    /// See GetSolveContactAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSolveContactAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOCKEDPOSAXIS 
    // --------------------------------------------------------------------- //
    /// Collection of flags providing a mechanism to lock motion along/around a specific axis (1 << 0, 1 << 1, 1 << 2).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int physxRigidBody:lockedPosAxis = 0` |
    /// | C++ Type | int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
    PHYSXSCHEMA_API
    UsdAttribute GetLockedPosAxisAttr() const;

    /// See GetLockedPosAxisAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLockedPosAxisAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOCKEDROTAXIS 
    // --------------------------------------------------------------------- //
    /// Collection of flags providing a mechanism to lock motion along/around a specific axis (1 << 0, 1 << 1, 1 << 2).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int physxRigidBody:lockedRotAxis = 0` |
    /// | C++ Type | int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
    PHYSXSCHEMA_API
    UsdAttribute GetLockedRotAxisAttr() const;

    /// See GetLockedRotAxisAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLockedRotAxisAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CONTACTSLOPCOEFFICIENT 
    // --------------------------------------------------------------------- //
    /// Tolerance on the angular influence of a contact that can help improve the behavior of rolling approximate collision shapes.
    /// Specifically, the angular component of a normal constraint in a contact is zeroed if normal.cross(offset) falls below this tolerance.
    /// The tolerance is scaled such that the behavior improvement persists through a range of angular velocities of a rolling shape.
    /// Range: [0, inf)
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:contactSlopCoefficient = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetContactSlopCoefficientAttr() const;

    /// See GetContactSlopCoefficientAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateContactSlopCoefficientAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CFMSCALE 
    // --------------------------------------------------------------------- //
    /// The cfm scale term can help stabilize an articulation by weakening the constraint's response.
    /// Use small values close to the default and typically in the range [0, 0.1] for best results.
    /// Note that this parameter is currently used only if rigid body is part of an articulation.
    /// Range: [0, 1]
    /// Units: unitless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxRigidBody:cfmScale = 0.025` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetCfmScaleAttr() const;

    /// See GetCfmScaleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCfmScaleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
