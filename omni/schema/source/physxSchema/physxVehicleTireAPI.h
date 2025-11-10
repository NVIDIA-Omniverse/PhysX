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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLETIREAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLETIREAPI_H

/// \file physxSchema/physxVehicleTireAPI.h

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
// PHYSXVEHICLETIREAPI                                                        //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleTireAPI
///
/// Properties of a PhysX vehicle tire. If the tire setup does not need to be shared among vehicle
/// instances, it can be applied to the prim which has PhysxVehicleWheelAttachmentAPI applied.
/// If the intent is to share the tire setup, PhysxVehicleTireAPI can be applied to a separate
/// prim which can be linked to (see PhysxVehicleWheelAttachmentAPI).
///
class PhysxSchemaPhysxVehicleTireAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleTireAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleTireAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleTireAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleTireAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleTireAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleTireAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleTireAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleTireAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleTireAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleTireAPI
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
    /// This information is stored by adding "PhysxVehicleTireAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleTireAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleTireAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleTireAPI 
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
    // LATSTIFFX 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use lateralStiffnessGraph instead.
    /// 
    /// Tire lateral stiffness is a graph of tire load that has linear behavior near zero load and 
    /// flattens at large loads. The parameter describes the minimum normalized load (load/restLoad)
    /// that gives a flat lateral stiffness response to load.
    /// 
    /// Note: will be ignored if lateralStiffnessGraph is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleTire:latStiffX = 2` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLatStiffXAttr() const;

    /// See GetLatStiffXAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLatStiffXAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LATSTIFFY 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use lateralStiffnessGraph instead.
    /// 
    /// Tire lateral stiffness is a graph of tire load that has linear behavior near zero load and 
    /// flattens at large loads. The parameter describes the maximum possible value of lateralStiffness/restLoad
    /// that occurs when (load/restLoad) >= latStiffX. If load/restLoad is greater than latStiffX then
    /// the lateral stiffness is latStiffY*restLoad, else the lateral stiffness is latStiffY*(load/latStiffX).
    /// 
    /// Note: will be ignored if lateralStiffnessGraph is used.
    /// 
    /// Note: if not defined, the value 17.095 will be used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleTire:latStiffY` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLatStiffYAttr() const;

    /// See GetLatStiffYAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLatStiffYAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LATERALSTIFFNESSGRAPH 
    // --------------------------------------------------------------------- //
    /// Tire lateral stiffness is represented as a graph of tire load that has linear behavior
    /// near zero load and flattens at large loads. The first value of the parameter describes the
    /// minimum normalized load (load/restLoad) that gives a flat lateral stiffness response to load.
    /// This value has to be greater or equal 0. The second value of the parameter defines the lateral
    /// stiffness when the response is flat (the max lateral stiffness) This value has to be positive.
    /// 
    /// Units: 
    /// Minimum normalized load: unitless
    /// Lateral stiffness: mass * distance / seconds / seconds
    /// 
    /// Note: the default value (0, 0) is not a valid value as such but indicates that the
    /// deprecated attributes latStiffX and latStiffY should be used instead.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float2 physxVehicleTire:lateralStiffnessGraph = (0, 0)` |
    /// | C++ Type | GfVec2f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float2 |
    PHYSXSCHEMA_API
    UsdAttribute GetLateralStiffnessGraphAttr() const;

    /// See GetLateralStiffnessGraphAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLateralStiffnessGraphAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LONGITUDINALSTIFFNESSPERUNITGRAVITY 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use longitudinalStiffness instead.
    /// 
    /// Tire longitudinal stiffness per unit gravitational acceleration. Longitudinal stiffness of 
    /// the tire is calculated as gravitationalAcceleration*longitudinalStiffnessPerUnitGravity.
    /// 
    /// Note: will be ignored if longitudinalStiffness is used.
    /// 
    /// Note: if not defined, the value 500 will be used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleTire:longitudinalStiffnessPerUnitGravity` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLongitudinalStiffnessPerUnitGravityAttr() const;

    /// See GetLongitudinalStiffnessPerUnitGravityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLongitudinalStiffnessPerUnitGravityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LONGITUDINALSTIFFNESS 
    // --------------------------------------------------------------------- //
    /// Tire longitudinal stiffness.
    /// The value has to be positive. The resulting longitudinal force can be approximated as: 
    /// longitudinalStiffness*longitudinalSlip.
    /// 
    /// Units: mass * distance / seconds / seconds
    /// 
    /// Note: the default value 0 is not a valid value as such but indicates that the deprecated
    /// attribute longitudinalStiffnessPerUnitGravity should be used instead.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleTire:longitudinalStiffness = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLongitudinalStiffnessAttr() const;

    /// See GetLongitudinalStiffnessAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLongitudinalStiffnessAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CAMBERSTIFFNESSPERUNITGRAVITY 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use camberStiffness instead.
    /// 
    /// Tire camber stiffness per unit gravitational acceleration. Camber stiffness of the tire is
    /// calculated as gravitationalAcceleration*camberStiffnessPerUnitGravity.
    /// 
    /// Note: will be ignored if camberStiffness is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleTire:camberStiffnessPerUnitGravity = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetCamberStiffnessPerUnitGravityAttr() const;

    /// See GetCamberStiffnessPerUnitGravityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCamberStiffnessPerUnitGravityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CAMBERSTIFFNESS 
    // --------------------------------------------------------------------- //
    /// Tire camber stiffness.
    /// The value has to be greater or equal 0. Camber force can be approximated as: 
    /// camberStiffness*camberAngle.
    /// 
    /// Units: mass * distance / seconds / seconds / radians
    /// 
    /// Note: the default value -1 is not a valid value as such but indicates that the 
    /// deprecated attribute camberStiffnessPerUnitGravity should be used instead.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleTire:camberStiffness = -1` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetCamberStiffnessAttr() const;

    /// See GetCamberStiffnessAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCamberStiffnessAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FRICTIONVSSLIPGRAPH 
    // --------------------------------------------------------------------- //
    /// Graph of friction vs longitudinal slip with 3 points. The friction value extracted from this
    /// graph will be multiplied with the friction value of the combination of ground material and tire
    /// type. The array is expected to have 3 entries/points. For the first point, the first value is 
    /// always zero and the second value is the friction available at zero longitudinal slip. For the
    /// second point, the first value is the longitudinal slip with maximum friction and the second
    /// value is the maximum friction. For the third point, the first value is the end point of the graph
    /// and the second value is the friction for slips greater than the end point of the graph. Note that
    /// the friction value of the first point is typically smaller than the friction value of the second
    /// point and the friction value of the second point is typically larger than the friction value of
    /// the third point.
    /// 
    /// Note: if not defined, the following values will be used: [(0.0, 1.0), (0.1, 1.0), (1.0, 1.0)].
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float2[] physxVehicleTire:frictionVsSlipGraph` |
    /// | C++ Type | VtArray<GfVec2f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float2Array |
    PHYSXSCHEMA_API
    UsdAttribute GetFrictionVsSlipGraphAttr() const;

    /// See GetFrictionVsSlipGraphAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFrictionVsSlipGraphAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // RESTLOAD 
    // --------------------------------------------------------------------- //
    /// The load that develops on the tire when the vehicle is at rest on a flat plane. 
    /// The value has to be greater or equal 0.
    /// If set to 0, the rest load will get computed automatically using the approximation of:
    /// gravitational acceleration * sprungMass.
    /// 
    /// Units: force = mass * distance / seconds / seconds
    /// 
    /// Note: it is not possible for a vehicle to have some rest load values being user-defined
    /// and some being computed automatically. Either all values have to be user-defined or all
    /// have to be set to 0 for auto-computation.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxVehicleTire:restLoad = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetRestLoadAttr() const;

    /// See GetRestLoadAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateRestLoadAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FRICTIONTABLE 
    // --------------------------------------------------------------------- //
    /// A relationship to a PhysxVehicleTireFrictionTable instance.
    /// 
    /// Note: if not defined, a default friction table will be used.
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetFrictionTableRel() const;

    /// See GetFrictionTableRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateFrictionTableRel() const;

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
