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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLESUSPENSIONAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLESUSPENSIONAPI_H

/// \file physxSchema/physxVehicleSuspensionAPI.h

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
// PHYSXVEHICLESUSPENSIONAPI                                                  //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleSuspensionAPI
///
/// Properties of a PhysX vehicle wheel suspension. If the suspension setup does not need to be shared 
/// among vehicle instances, it can be applied to the prim which has PhysxVehicleWheelAttachmentAPI
/// applied. If the intent is to share the suspension setup, PhysxVehicleSuspensionAPI can be 
/// applied to a separate prim which can be linked to (see PhysxVehicleWheelAttachmentAPI).
///
class PhysxSchemaPhysxVehicleSuspensionAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleSuspensionAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleSuspensionAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleSuspensionAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleSuspensionAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleSuspensionAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleSuspensionAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleSuspensionAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleSuspensionAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleSuspensionAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleSuspensionAPI
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
    /// This information is stored by adding "PhysxVehicleSuspensionAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleSuspensionAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleSuspensionAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleSuspensionAPI 
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
    // SPRINGSTRENGTH 
    // --------------------------------------------------------------------- //
    /// Spring strength of suspension unit.
    /// 
    /// Units: force / distance = mass / seconds / seconds
    /// 
    /// Note: this attribute has to be specified (there is no default).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:springStrength` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetSpringStrengthAttr() const;

    /// See GetSpringStrengthAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSpringStrengthAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SPRINGDAMPERRATE 
    // --------------------------------------------------------------------- //
    /// Spring damper rate of suspension unit.
    /// 
    /// Units: force * seconds / distance = mass / seconds
    /// 
    /// Note: this attribute has to be specified (there is no default).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:springDamperRate` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetSpringDamperRateAttr() const;

    /// See GetSpringDamperRateAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSpringDamperRateAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // TRAVELDISTANCE 
    // --------------------------------------------------------------------- //
    /// Distance the wheel can travel along the suspension when going from max compression to 
    /// max droop. The value has to be positive.
    /// 
    /// Units: distance
    /// 
    /// Note: either this attribute or the deprecated maxCompression/maxDroop have to
    /// to be specified (with the former taking precedence). When migrating from the
    /// deprecated attributes, travelDistance can be set to the sum of maxCompression
    /// and maxDroop.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:travelDistance` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetTravelDistanceAttr() const;

    /// See GetTravelDistanceAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTravelDistanceAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXCOMPRESSION 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use travelDistance instead.
    /// 
    /// Maximum compression from rest state allowed by suspension spring.
    /// The value has to be positive.
    /// 
    /// Units: distance
    /// 
    /// Note: will be ignored if travelDistance is authored.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:maxCompression` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxCompressionAttr() const;

    /// See GetMaxCompressionAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxCompressionAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXDROOP 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use travelDistance instead.
    /// 
    /// Maximum elongation from rest state allowed by suspension spring.
    /// The value has to be positive unless it should get computed automatically in which case any
    /// negative number can be used to indicate as much. It is highly recommended to chose a value
    /// close to: sprungMass * gravity / springStrength.
    /// 
    /// Units: distance
    /// 
    /// Note: will be ignored if travelDistance is authored.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:maxDroop` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxDroopAttr() const;

    /// See GetMaxDroopAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxDroopAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SPRUNGMASS 
    // --------------------------------------------------------------------- //
    /// Mass of vehicle that is supported by suspension spring.
    /// If set to 0, the sprung mass will get computed automatically.
    /// 
    /// Units: mass
    /// 
    /// Note: it is not possible for a vehicle to have some sprung mass values being user-defined and some 
    /// being computed automatically. Either all values have to be user-defined or all have to be set to 0 
    /// for auto-computation.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:sprungMass = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetSprungMassAttr() const;

    /// See GetSprungMassAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSprungMassAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CAMBERATREST 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleSuspensionComplianceAPI instead.
    /// 
    /// Camber angle (in radians) of wheel when the suspension is at its rest position.
    /// 
    /// Note: will be ignored if PhysxVehicleSuspensionComplianceAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:camberAtRest` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetCamberAtRestAttr() const;

    /// See GetCamberAtRestAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCamberAtRestAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CAMBERATMAXCOMPRESSION 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleSuspensionComplianceAPI instead.
    /// 
    /// Camber angle (in radians) of wheel when the suspension is at maximum compression.
    /// 
    /// Note: will be ignored if PhysxVehicleSuspensionComplianceAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:camberAtMaxCompression` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetCamberAtMaxCompressionAttr() const;

    /// See GetCamberAtMaxCompressionAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCamberAtMaxCompressionAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CAMBERATMAXDROOP 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleSuspensionComplianceAPI instead.
    /// 
    /// Camber angle (in radians) of wheel when the suspension is at maximum droop.
    /// 
    /// Note: will be ignored if PhysxVehicleSuspensionComplianceAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleSuspension:camberAtMaxDroop` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetCamberAtMaxDroopAttr() const;

    /// See GetCamberAtMaxDroopAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCamberAtMaxDroopAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
