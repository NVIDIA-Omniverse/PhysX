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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLEBRAKESAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLEBRAKESAPI_H

/// \file physxSchema/physxVehicleBrakesAPI.h

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
// PHYSXVEHICLEBRAKESAPI                                                      //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleBrakesAPI
///
/// Describes a braking system for a vehicle by specifying which wheels are connected to the
/// brake control and by defining the brake torque that gets applied to those wheels. Currently,
/// up to two braking systems are supported. Use the instance name TfToken "brakes0" and "brakes1"
/// of this multipleApply schema to distinguish between the two braking systems. Note that system
/// "brakes0" will be coupled to the brake control brake0 while system "brakes1" will be coupled
/// to the brake control brake1 (see PhysxVehicleControllerAPI for the brake controls). An example
/// for using two systems is to provide brake and handbrake control (the former applying brake
/// torque to all wheels and the latter applying brake torque to the rear wheels only). This API
/// schema has to be applied to a prim with PhysxVehicleAPI applied. Can only be used for vehicles
/// that have a drive (see PhysxVehicleDriveBasicAPI or PhysxVehicleDriveStandardAPI).
///
class PhysxSchemaPhysxVehicleBrakesAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::MultipleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleBrakesAPI on UsdPrim \p prim with
    /// name \p name . Equivalent to
    /// PhysxSchemaPhysxVehicleBrakesAPI::Get(
    ///    prim.GetStage(),
    ///    prim.GetPath().AppendProperty(
    ///        "physxVehicleBrakes:name"));
    ///
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleBrakesAPI(
        const UsdPrim& prim=UsdPrim(), const TfToken &name=TfToken())
        : UsdAPISchemaBase(prim, /*instanceName*/ name)
    { }

    /// Construct a PhysxSchemaPhysxVehicleBrakesAPI on the prim held by \p schemaObj with
    /// name \p name.  Should be preferred over
    /// PhysxSchemaPhysxVehicleBrakesAPI(schemaObj.GetPrim(), name), as it preserves
    /// SchemaBase state.
    explicit PhysxSchemaPhysxVehicleBrakesAPI(
        const UsdSchemaBase& schemaObj, const TfToken &name)
        : UsdAPISchemaBase(schemaObj, /*instanceName*/ name)
    { }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleBrakesAPI();

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

    /// Return a PhysxSchemaPhysxVehicleBrakesAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  \p path must be of the format
    /// <path>.physxVehicleBrakes:name .
    ///
    /// This is shorthand for the following:
    ///
    /// \code
    /// TfToken name = SdfPath::StripNamespace(path.GetToken());
    /// PhysxSchemaPhysxVehicleBrakesAPI(
    ///     stage->GetPrimAtPath(path.GetPrimPath()), name);
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleBrakesAPI
    Get(const UsdStagePtr &stage, const SdfPath &path);

    /// Return a PhysxSchemaPhysxVehicleBrakesAPI with name \p name holding the
    /// prim \p prim. Shorthand for PhysxSchemaPhysxVehicleBrakesAPI(prim, name);
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleBrakesAPI
    Get(const UsdPrim &prim, const TfToken &name);

    /// Return a vector of all named instances of PhysxSchemaPhysxVehicleBrakesAPI on the 
    /// given \p prim.
    PHYSXSCHEMA_API
    static std::vector<PhysxSchemaPhysxVehicleBrakesAPI>
    GetAll(const UsdPrim &prim);

    /// Checks if the given name \p baseName is the base name of a property
    /// of PhysxVehicleBrakesAPI.
    PHYSXSCHEMA_API
    static bool
    IsSchemaPropertyBaseName(const TfToken &baseName);

    /// Checks if the given path \p path is of an API schema of type
    /// PhysxVehicleBrakesAPI. If so, it stores the instance name of
    /// the schema in \p name and returns true. Otherwise, it returns false.
    PHYSXSCHEMA_API
    static bool
    IsPhysxVehicleBrakesAPIPath(const SdfPath &path, TfToken *name);

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
    /// This information is stored by adding "PhysxVehicleBrakesAPI:<i>name</i>" 
    /// to the token-valued, listOp metadata \em apiSchemas on the prim.
    /// For example, if \p name is 'instance1', the token 
    /// 'PhysxVehicleBrakesAPI:instance1' is added to 'apiSchemas'.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleBrakesAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleBrakesAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleBrakesAPI 
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
    // WHEELS 
    // --------------------------------------------------------------------- //
    /// List of indices, referencing the wheels that receive brake torque. The indices
    /// refer to the attribute "index" of PhysxVehicleWheelAttachmentAPI. If not specified,
    /// each wheel will receive a brake torque using a torque multiplier of 1 (see attribute
    /// torqueMultipliers).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] wheels` |
    /// | C++ Type | VtArray<int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetWheelsAttr() const;

    /// See GetWheelsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWheelsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXBRAKETORQUE 
    // --------------------------------------------------------------------- //
    /// The maximum brake torque that can be generated by the braking system. Has to be greater or equal
    /// zero. For braking system "brakes0", the brake torque applied to wheels[i] will be
    /// defined by maxBrakeTorque * torqueMultipliers[i] * physxVehicleController:brake0
    /// (see PhysxVehicleControllerAPI).
    /// 
    /// Units: mass * distance * distance / seconds / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float maxBrakeTorque = 0` |
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
    // TORQUEMULTIPLIERS 
    // --------------------------------------------------------------------- //
    /// Defines how much of the max brake torque can be delivered to the wheels that are
    /// connected to the braking system. The values are unitless multipliers and have to be
    /// greater or equal to zero. See maxBrakeTorque for how these multipliers affect the
    /// brake torque applied to a wheel. If the attribute is defined, then the length of the
    /// array needs to match the length of the "wheels" array. If the attribute is not defined,
    /// a multiplier value of 1 will be used for all connected wheels.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] torqueMultipliers` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetTorqueMultipliersAttr() const;

    /// See GetTorqueMultipliersAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTorqueMultipliersAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
