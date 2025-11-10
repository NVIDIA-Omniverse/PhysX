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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLEENGINEAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLEENGINEAPI_H

/// \file physxSchema/physxVehicleEngineAPI.h

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
// PHYSXVEHICLEENGINEAPI                                                      //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleEngineAPI
///
/// Properties of a PhysX vehicle engine. If the engine setup does not need to be shared among vehicle
/// instances, it can be applied to the prim which has PhysxVehicleDriveStandardAPI applied.
/// If the intent is to share the engine setup, PhysxVehicleEngineAPI can be applied to a separate
/// prim which can be linked to (see PhysxVehicleDriveStandardAPI).
///
class PhysxSchemaPhysxVehicleEngineAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleEngineAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleEngineAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleEngineAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleEngineAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleEngineAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleEngineAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleEngineAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleEngineAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleEngineAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleEngineAPI
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
    /// This information is stored by adding "PhysxVehicleEngineAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleEngineAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleEngineAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleEngineAPI 
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
    // MOI 
    // --------------------------------------------------------------------- //
    /// The moment of inertia of the engine around the axis of rotation. The value has to be positive.
    /// 
    /// Units: mass * distance * distance
    /// 
    /// Note: if not defined, the value 1.0 will be used. This default value is in meter length scale 
    /// and kilograms and will get adjusted if other units are used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleEngine:moi` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMoiAttr() const;

    /// See GetMoiAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMoiAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PEAKTORQUE 
    // --------------------------------------------------------------------- //
    /// Maximum torque available to apply to the engine when the accelerator pedal is at maximum.
    /// The value has to be greater or equal 0.
    /// 
    /// Units: mass * distance * distance / seconds / seconds
    /// 
    /// Note: the torque available is the value of the accelerator pedal (in range [0, 1]) multiplied
    /// by the normalized torque as computed from torqueCurve (see corresponding attribute)
    /// multiplied by peakTorque.
    /// 
    /// Note: the default value -1 is not a valid value as such but will result in the value 500.0
    /// being used (in meter length scale and kilogram mass scale or the equivalent if other units
    /// are used).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleEngine:peakTorque = -1` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetPeakTorqueAttr() const;

    /// See GetPeakTorqueAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePeakTorqueAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXROTATIONSPEED 
    // --------------------------------------------------------------------- //
    /// Maximum rotation speed of the engine. The value has to be greater
    /// or equal 0.
    /// 
    /// Units: radians / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleEngine:maxRotationSpeed = 600` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxRotationSpeedAttr() const;

    /// See GetMaxRotationSpeedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxRotationSpeedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // IDLEROTATIONSPEED 
    // --------------------------------------------------------------------- //
    /// Rotation speed of the engine when idling. The value has to be greater or equal 0.
    /// 
    /// Units: radians / seconds
    /// 
    /// 
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleEngine:idleRotationSpeed = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetIdleRotationSpeedAttr() const;

    /// See GetIdleRotationSpeedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateIdleRotationSpeedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // TORQUECURVE 
    // --------------------------------------------------------------------- //
    /// Graph of normalized torque (torque / peakTorque) against normalized engine speed (engineRotationSpeed / maxRotationSpeed).
    /// 
    /// Note: the normalized engine speed is the x-axis of the graph, while the normalized torque is the y-axis of the graph.
    /// No more than 8 data points are supported.
    /// 
    /// Note: if not defined, the following values will be used: [(0.0, 0.8), (0.33, 1.0), (1.0, 0.8)].
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float2[] physxVehicleEngine:torqueCurve` |
    /// | C++ Type | VtArray<GfVec2f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float2Array |
    PHYSXSCHEMA_API
    UsdAttribute GetTorqueCurveAttr() const;

    /// See GetTorqueCurveAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTorqueCurveAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // DAMPINGRATEFULLTHROTTLE 
    // --------------------------------------------------------------------- //
    /// Damping rate of engine when full throttle is applied. The value has to be greater
    /// or equal 0.
    /// 
    /// Units: torque * seconds = mass * distance * distance / seconds
    /// 
    /// Note: if the clutch is engaged (any gear except neutral) then the damping rate applied 
    /// at run-time is an interpolation between dampingRateZeroThrottleClutchEngaged (see 
    /// corresponding attribute) and dampingRateFullThrottle:
    /// dampingRateZeroThrottleClutchEngaged + 
    /// ((dampingRateFullThrottle-dampingRateZeroThrottleClutchEngaged) * acceleratorPedal).
    /// 
    /// Note: if the clutch is disengaged (in neutral gear) the damping rate applied at run-time
    /// is an interpolation between dampingRateZeroThrottleClutchDisengaged (see corresponding 
    /// attribute) and dampingRateFullThrottle:
    /// dampingRateZeroThrottleClutchDisengaged +
    /// ((dampingRateFullThrottle-dampingRateZeroThrottleClutchDisengaged) * acceleratorPedal).
    /// 
    /// Note: the default value -1 is not a valid value as such but will result in the value 0.15
    /// being used (in meter length scale and kilogram mass scale or the equivalent if other units
    /// are used).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleEngine:dampingRateFullThrottle = -1` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetDampingRateFullThrottleAttr() const;

    /// See GetDampingRateFullThrottleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateDampingRateFullThrottleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // DAMPINGRATEZEROTHROTTLECLUTCHENGAGED 
    // --------------------------------------------------------------------- //
    /// Damping rate of engine when no throttle is applied and the clutch is engaged. 
    /// See attribute "dampingRateFullThrottle" for information about the effect of the value. 
    /// The value has to be greater or equal 0.
    /// 
    /// Units: torque * seconds = mass * distance * distance / seconds
    /// 
    /// Note: the default value -1 is not a valid value as such but will result in the value 2.0
    /// being used (in meter length scale and kilogram mass scale or the equivalent if other units
    /// are used).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleEngine:dampingRateZeroThrottleClutchEngaged = -1` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetDampingRateZeroThrottleClutchEngagedAttr() const;

    /// See GetDampingRateZeroThrottleClutchEngagedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateDampingRateZeroThrottleClutchEngagedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // DAMPINGRATEZEROTHROTTLECLUTCHDISENGAGED 
    // --------------------------------------------------------------------- //
    /// Damping rate of engine when no throttle is applied and the clutch is disengaged. See attribute "dampingRateFullThrottle"
    /// for information about the effect of the value. The value has to be greater or equal 0.
    /// 
    /// Units: torque * seconds = mass * distance * distance / seconds
    /// 
    /// Note: the default value -1 is not a valid value as such but will result in the value 0.35
    /// being used (in meter length scale and kilogram mass scale or the equivalent if other units
    /// are used).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleEngine:dampingRateZeroThrottleClutchDisengaged = -1` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetDampingRateZeroThrottleClutchDisengagedAttr() const;

    /// See GetDampingRateZeroThrottleClutchDisengagedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateDampingRateZeroThrottleClutchDisengagedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
