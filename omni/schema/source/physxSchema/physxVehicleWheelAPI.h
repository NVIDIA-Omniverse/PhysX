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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLEWHEELAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLEWHEELAPI_H

/// \file physxSchema/physxVehicleWheelAPI.h

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
// PHYSXVEHICLEWHEELAPI                                                       //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleWheelAPI
///
/// Properties of a PhysX vehicle wheel. If the wheel setup does not need to be shared among vehicle
/// instances, it can be applied to the prim which has PhysxVehicleWheelAttachmentAPI applied.
/// If the intent is to share the wheel setup, PhysxVehicleWheelAPI can be applied to a separate
/// prim which can be linked to (see PhysxVehicleWheelAttachmentAPI).
///
class PhysxSchemaPhysxVehicleWheelAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleWheelAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleWheelAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleWheelAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleWheelAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleWheelAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleWheelAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleWheelAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleWheelAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleWheelAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleWheelAPI
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
    /// This information is stored by adding "PhysxVehicleWheelAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleWheelAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleWheelAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleWheelAPI 
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
    // RADIUS 
    // --------------------------------------------------------------------- //
    /// The radius of the wheel (metal wheel plus tire).
    /// 
    /// Note: this attribute has to be specified (there is no default).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:radius` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetRadiusAttr() const;

    /// See GetRadiusAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateRadiusAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // WIDTH 
    // --------------------------------------------------------------------- //
    /// The width of the wheel (metal wheel plus tire).
    /// 
    /// Note: this attribute has to be specified (there is no default).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:width` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetWidthAttr() const;

    /// See GetWidthAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWidthAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MASS 
    // --------------------------------------------------------------------- //
    /// The mass of the wheel (metal wheel plus tire).
    /// 
    /// Note: this attribute has to be specified (there is no default).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:mass` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMassAttr() const;

    /// See GetMassAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMassAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MOI 
    // --------------------------------------------------------------------- //
    /// The moment of inertia (metal wheel plus tire) about the rolling axis.
    /// 
    /// Units: mass * distance * distance
    /// 
    /// Note: this attribute has to be specified (there is no default).
    /// 
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:moi` |
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
    // DAMPINGRATE 
    // --------------------------------------------------------------------- //
    /// The damping rate applied to the wheel.
    /// 
    /// Units: torque * seconds = mass * distance * distance / seconds
    /// 
    /// Note: this attribute has to be specified (there is no default).
    /// 
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:dampingRate` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetDampingRateAttr() const;

    /// See GetDampingRateAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateDampingRateAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXBRAKETORQUE 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleBrakesAPI instead.
    /// 
    /// The maximum brake torque that can be applied to the wheel
    /// 
    /// Units: mass * distance * distance / seconds / seconds
    /// 
    /// Note: will be ignored if PhysxVehicleBrakesAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:maxBrakeTorque` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxBrakeTorqueAttr() const;

    /// See GetMaxBrakeTorqueAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxBrakeTorqueAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXHANDBRAKETORQUE 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleBrakesAPI instead.
    /// 
    /// The maximum hand brake torque that can be applied to the wheel
    /// 
    /// Units: mass * distance * distance / seconds / seconds
    /// 
    /// Note: will be ignored if PhysxVehicleBrakesAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:maxHandBrakeTorque` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxHandBrakeTorqueAttr() const;

    /// See GetMaxHandBrakeTorqueAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxHandBrakeTorqueAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXSTEERANGLE 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleSteeringAPI instead.
    /// 
    /// The maximum steer angle (in radians) that can be achieved by the wheel.
    /// 
    /// Note: will be ignored if PhysxVehicleSteeringAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:maxSteerAngle` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxSteerAngleAttr() const;

    /// See GetMaxSteerAngleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxSteerAngleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // TOEANGLE 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleSuspensionComplianceAPI instead.
    /// 
    /// The wheel toe angle (in radians).
    /// 
    /// Note: currently, this attribute is only considered for vehicles using the
    /// PhysxVehicleDriveStandard drive type.
    /// 
    /// Note: will be ignored if PhysxVehicleSuspensionComplianceAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleWheel:toeAngle` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetToeAngleAttr() const;

    /// See GetToeAngleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateToeAngleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
