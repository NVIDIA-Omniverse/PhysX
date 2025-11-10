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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLESUSPENSIONCOMPLIANCEAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLESUSPENSIONCOMPLIANCEAPI_H

/// \file physxSchema/physxVehicleSuspensionComplianceAPI.h

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
// PHYSXVEHICLESUSPENSIONCOMPLIANCEAPI                                        //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleSuspensionComplianceAPI
///
/// Compliance describes how toe and camber angle and force application points are 
/// affected by suspension compression. Each compliance term is in the form of a graph
/// with up to 3 points. The points in the graph consist of a normalized jounce value
/// (with 0 meaning fully elongated and 1 fully compressed suspension) and a corresponding
/// compliance value (which can be an angle or point etc. depending on the specific
/// compliance term). The sequence of points must respresent monotonically increasing
/// values of normalized jounce. The actual compliance value will be computed by linear
/// interpolation based on the current normalized jounce. If any graph has zero points
/// in it, a value of 0.0 is used for the compliance value. If any graph has 1 point
/// in it, the compliance value of that point is used directly. This API schema has to
/// be applied to a prim with PhysxVehicleWheelAttachmentAPI applied. If defined, then
/// this setup takes precedence over the deprecated attributes suspensionForceAppPointOffset
/// and tireForceAppPointOffset of the PhysxVehicleWheelAttachmentAPI API schema, the
/// deprecated attributes camberAtRest, camberAtMaxCompression, camberAtMaxDroop of the
/// PhysxVehicleSuspensionAPI API schema as well as the deprecated attribute toeAngle
/// of the PhysxVehicleWheelAPI API schema.
///
class PhysxSchemaPhysxVehicleSuspensionComplianceAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleSuspensionComplianceAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleSuspensionComplianceAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleSuspensionComplianceAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleSuspensionComplianceAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleSuspensionComplianceAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleSuspensionComplianceAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleSuspensionComplianceAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleSuspensionComplianceAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleSuspensionComplianceAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleSuspensionComplianceAPI
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
    /// This information is stored by adding "PhysxVehicleSuspensionComplianceAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleSuspensionComplianceAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleSuspensionComplianceAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleSuspensionComplianceAPI 
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
    // WHEELTOEANGLE 
    // --------------------------------------------------------------------- //
    /// A graph of toe angle against normalized jounce with the toe angle expressed
    /// in radians in the range [-pi, pi]. See the class documentation for general info
    /// about the nature of these graphs and the data requirements.
    /// 
    /// Note: the toe angle is applied in the suspension frame (see PhysxVehicleWheelAttachmentAPI).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float2[] physxVehicleSuspensionCompliance:wheelToeAngle` |
    /// | C++ Type | VtArray<GfVec2f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float2Array |
    PHYSXSCHEMA_API
    UsdAttribute GetWheelToeAngleAttr() const;

    /// See GetWheelToeAngleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWheelToeAngleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // WHEELCAMBERANGLE 
    // --------------------------------------------------------------------- //
    /// A graph of camber angle against normalized jounce with the camber angle expressed
    /// in radians in the range [-pi, pi]. See the class documentation for general info
    /// about the nature of these graphs and the data requirements.
    /// 
    /// Note: the camber angle is applied in the suspension frame (see PhysxVehicleWheelAttachmentAPI).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float2[] physxVehicleSuspensionCompliance:wheelCamberAngle` |
    /// | C++ Type | VtArray<GfVec2f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float2Array |
    PHYSXSCHEMA_API
    UsdAttribute GetWheelCamberAngleAttr() const;

    /// See GetWheelCamberAngleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWheelCamberAngleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SUSPENSIONFORCEAPPPOINT 
    // --------------------------------------------------------------------- //
    /// Suspension forces are applied at an offset from the suspension frame (see 
    /// PhysxVehicleWheelAttachmentAPI). An entry in this array defines this offset for a
    /// given normalized jounce value (which is the first element in the float4 while
    /// the other 3 elements are used to store the x, y, z coordinates of the offset).
    /// See the class documentation for general info about the nature of these compliance
    /// terms and the data requirements.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float4[] physxVehicleSuspensionCompliance:suspensionForceAppPoint` |
    /// | C++ Type | VtArray<GfVec4f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float4Array |
    PHYSXSCHEMA_API
    UsdAttribute GetSuspensionForceAppPointAttr() const;

    /// See GetSuspensionForceAppPointAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSuspensionForceAppPointAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // TIREFORCEAPPPOINT 
    // --------------------------------------------------------------------- //
    /// Tire forces are applied at an offset from the suspension frame (see 
    /// PhysxVehicleWheelAttachmentAPI). An entry in this array defines this offset for a
    /// given normalized jounce value (which is the first element in the float4 while
    /// the other 3 elements are used to store the x, y, z coordinates of the offset).
    /// See the class documentation for general info about the nature of these compliance
    /// terms and the data requirements.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float4[] physxVehicleSuspensionCompliance:tireForceAppPoint` |
    /// | C++ Type | VtArray<GfVec4f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float4Array |
    PHYSXSCHEMA_API
    UsdAttribute GetTireForceAppPointAttr() const;

    /// See GetTireForceAppPointAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTireForceAppPointAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
