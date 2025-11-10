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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLEMULTIWHEELDIFFERENTIALAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLEMULTIWHEELDIFFERENTIALAPI_H

/// \file physxSchema/physxVehicleMultiWheelDifferentialAPI.h

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
// PHYSXVEHICLEMULTIWHEELDIFFERENTIALAPI                                      //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI
///
/// Describes which wheels of a vehicle are driven as well as the distribution of the
/// drive torque among those wheels. Has to be applied to a prim with PhysxVehicleAPI applied.
/// Can only be used for vehicles that have a drive (see PhysxVehicleDriveBasicAPI or
/// PhysxVehicleDriveStandardAPI).
///
class PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI
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
    /// This information is stored by adding "PhysxVehicleMultiWheelDifferentialAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI 
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
    // WHEELS 
    // --------------------------------------------------------------------- //
    /// List of indices, referencing the wheels that are driven (receiving torque from the engine). The
    /// indices refer to the attribute "index" of PhysxVehicleWheelAttachmentAPI.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] physxVehicleMultiWheelDifferential:wheels` |
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
    // TORQUERATIOS 
    // --------------------------------------------------------------------- //
    /// Defines how the drive torque should get distributed among the wheels specified in the "wheels"
    /// attribute. The provided ratios must be in range [-1, 1] and under normal circumstances the absolute
    /// values should sum up to 1. If the attribute is defined, then the length of the array needs to match
    /// the length of the "wheels" array. If the attribute is not defined, the torque will get split equally
    /// among all driven wheels.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] physxVehicleMultiWheelDifferential:torqueRatios` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetTorqueRatiosAttr() const;

    /// See GetTorqueRatiosAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTorqueRatiosAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // AVERAGEWHEELSPEEDRATIOS 
    // --------------------------------------------------------------------- //
    /// Defines how the speeds of the driven wheels get weighted when computing the average wheel
    /// speed at the clutch (for computing clutch slip). The provided ratios must be in range [0, 1]
    /// and under normal circumstances sum up to 1. If the attribute is defined, then the length of 
    /// the array needs to match the length of the "wheels" array. If the attribute is not defined, 
    /// the wheel speeds of all driven wheels will get weighted equally.
    /// 
    /// Note: if a vehicle is using a PhysxVehicleDriveBasicAPI drive, then this attribute should not
    /// be used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] physxVehicleMultiWheelDifferential:averageWheelSpeedRatios` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetAverageWheelSpeedRatiosAttr() const;

    /// See GetAverageWheelSpeedRatiosAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateAverageWheelSpeedRatiosAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
