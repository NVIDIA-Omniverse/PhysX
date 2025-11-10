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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLEWHEELATTACHMENTAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLEWHEELATTACHMENTAPI_H

/// \file physxSchema/physxVehicleWheelAttachmentAPI.h

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
// PHYSXVEHICLEWHEELATTACHMENTAPI                                             //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleWheelAttachmentAPI
///
/// For every wheel of a vehicle, this class defines the attachment properties. Has to 
/// be applied to a prim that is a descendant of a prim with PhysxVehicleAPI applied. If the wheel
/// attachment prim (the prim with PhysxVehicleWheelAttachmentAPI applied) is a UsdGeomXformable,
/// then the position and orientation of the prim will be set by the vehicle simulation. If the
/// wheel attachment prim has PhysicsCollisionAPI applied, then none of its descendants are allowed
/// to have PhysicsCollisionAPI applied. If the wheel attachment prim is a UsdGeomXformable but does
/// not have PhysicsCollisionAPI applied, then exactly one direct child prim among all descendants
/// is allowed to have PhysicsCollisionAPI applied. That prim will be interpreted as the collision
/// geometry of the wheel and its position and orientation will be set by the vehicle simulation
/// too. Note that the relative transform between the collision geometry prim and the wheel attachment
/// prim at the start of the simulation will be maintained (the relative transform with respect to
/// the center of mass frame of the vehicle rigid body that is). If there is no desire to have the
/// vehicle simulation control the transform of the wheel attachment prim, then a prim type that is
/// not a UsdGeomXformable should be chosen.
///
class PhysxSchemaPhysxVehicleWheelAttachmentAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleWheelAttachmentAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleWheelAttachmentAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleWheelAttachmentAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleWheelAttachmentAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleWheelAttachmentAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleWheelAttachmentAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleWheelAttachmentAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleWheelAttachmentAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleWheelAttachmentAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleWheelAttachmentAPI
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
    /// This information is stored by adding "PhysxVehicleWheelAttachmentAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleWheelAttachmentAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleWheelAttachmentAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleWheelAttachmentAPI 
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
    // INDEX 
    // --------------------------------------------------------------------- //
    /// The index of the wheel attachment. Some other components will reference the wheel attachments
    /// based on this index. Either all indices of a vehicle's wheel attachments need to be -1 or they
    /// need to cover all entries in the group {0, .., (numberOfWheels-1)}. All indices being -1 covers 
    /// the case where wheels are controlled directly through PhysxVehicleWheelControllerAPI, for example,
    /// or when deprecated APIs are used still.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int physxVehicleWheelAttachment:index = -1` |
    /// | C++ Type | int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
    PHYSXSCHEMA_API
    UsdAttribute GetIndexAttr() const;

    /// See GetIndexAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateIndexAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SUSPENSIONTRAVELDIRECTION 
    // --------------------------------------------------------------------- //
    /// The direction of the suspension travel (towards the wheel). The custom metadata
    /// physxVehicle:referenceFrameIsCenterOfMass (on the vehicle prim) defines in what frame
    /// the direction is specified. If set to true, the direction is assumed to be relative
    /// to the vehicle center of mass frame, else relative to the frame of the vehicle prim.
    /// Note that using the center of mass frame as reference is deprecated and will not be
    /// supported for much longer.
    /// 
    /// Note: this attribute has to be specified (there is no default).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `vector3f physxVehicleWheelAttachment:suspensionTravelDirection` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Vector3f |
    PHYSXSCHEMA_API
    UsdAttribute GetSuspensionTravelDirectionAttr() const;

    /// See GetSuspensionTravelDirectionAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSuspensionTravelDirectionAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SUSPENSIONFRAMEPOSITION 
    // --------------------------------------------------------------------- //
    /// The point of the suspension at max compression. The custom metadata
    /// physxVehicle:referenceFrameIsCenterOfMass (on the vehicle prim) defines in what frame
    /// the position is specified. If set to true, the position is assumed to be relative
    /// to the vehicle center of mass frame, else relative to the frame of the vehicle prim.
    /// Note that using the center of mass frame as reference is deprecated and will not be
    /// supported for much longer. Camber, steer and toe angles are all applied in the suspension
    /// frame. The vehicle frame transform of the wheel will be defined by: centerOfMassFrame * 
    /// suspensionFrame * wheelFrame (if physxVehicle:referenceFrameIsCenterOfMass is true,
    /// else centerOfMassFrame has to be omitted).
    /// 
    /// Note: either this attribute or the deprecated wheelCenterOfMassOffset have to
    /// to be specified (with the former taking precedence).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `point3f physxVehicleWheelAttachment:suspensionFramePosition` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Point3f |
    PHYSXSCHEMA_API
    UsdAttribute GetSuspensionFramePositionAttr() const;

    /// See GetSuspensionFramePositionAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSuspensionFramePositionAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SUSPENSIONFRAMEORIENTATION 
    // --------------------------------------------------------------------- //
    /// The orientation of the suspension frame. The custom metadata
    /// physxVehicle:referenceFrameIsCenterOfMass (on the vehicle prim) defines in what frame
    /// the orientation is specified. If set to true, the orientation is assumed to be relative
    /// to the vehicle center of mass frame, else relative to the frame of the vehicle prim.
    /// Note that using the center of mass frame as reference is deprecated and will not be
    /// supported for much longer. Camber, steer and toe angles are all applied in the suspension
    /// frame. The vehicle frame transform of the wheel will be defined by: centerOfMassFrame * 
    /// suspensionFrame * wheelFrame (if physxVehicle:referenceFrameIsCenterOfMass is true,
    /// else centerOfMassFrame has to be omitted).
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `quatf physxVehicleWheelAttachment:suspensionFrameOrientation = (1, 0, 0, 0)` |
    /// | C++ Type | GfQuatf |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Quatf |
    PHYSXSCHEMA_API
    UsdAttribute GetSuspensionFrameOrientationAttr() const;

    /// See GetSuspensionFrameOrientationAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSuspensionFrameOrientationAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SUSPENSIONFORCEAPPPOINTOFFSET 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleSuspensionComplianceAPI instead.
    /// 
    /// The location where the suspension force gets applied. The custom metadata
    /// physxVehicle:referenceFrameIsCenterOfMass (on the vehicle prim) defines in what frame
    /// the offset is specified. If set to true, the offset is assumed to be relative
    /// to the vehicle center of mass frame, else relative to the frame of the vehicle prim.
    /// Note that using the center of mass frame as reference is deprecated and will not be
    /// supported for much longer.
    /// 
    /// Note: will be ignored if PhysxVehicleSuspensionComplianceAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float3 physxVehicleWheelAttachment:suspensionForceAppPointOffset` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float3 |
    PHYSXSCHEMA_API
    UsdAttribute GetSuspensionForceAppPointOffsetAttr() const;

    /// See GetSuspensionForceAppPointOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSuspensionForceAppPointOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // WHEELCENTEROFMASSOFFSET 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use suspensionFramePosition instead.
    /// 
    /// The location of the wheel centre when at rest. The custom metadata
    /// physxVehicle:referenceFrameIsCenterOfMass (on the vehicle prim) defines in what frame
    /// the offset is specified. If set to true, the offset is assumed to be relative
    /// to the vehicle center of mass frame, else relative to the frame of the vehicle prim.
    /// Note that using the center of mass frame as reference is deprecated and will not be
    /// supported for much longer.
    /// 
    /// Note: will be ignored if suspensionFramePosition is authored.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float3 physxVehicleWheelAttachment:wheelCenterOfMassOffset` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float3 |
    PHYSXSCHEMA_API
    UsdAttribute GetWheelCenterOfMassOffsetAttr() const;

    /// See GetWheelCenterOfMassOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWheelCenterOfMassOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // TIREFORCEAPPPOINTOFFSET 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleSuspensionComplianceAPI instead.
    /// 
    /// The location where the tire force gets applied. The custom metadata
    /// physxVehicle:referenceFrameIsCenterOfMass (on the vehicle prim) defines in what frame
    /// the offset is specified. If set to true, the offset is assumed to be relative
    /// to the vehicle center of mass frame, else relative to the frame of the vehicle prim.
    /// Note that using the center of mass frame as reference is deprecated and will not be
    /// supported for much longer.
    /// 
    /// Note: will be ignored if PhysxVehicleSuspensionComplianceAPI is used.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float3 physxVehicleWheelAttachment:tireForceAppPointOffset` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float3 |
    PHYSXSCHEMA_API
    UsdAttribute GetTireForceAppPointOffsetAttr() const;

    /// See GetTireForceAppPointOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTireForceAppPointOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // WHEELFRAMEPOSITION 
    // --------------------------------------------------------------------- //
    /// A position offset of the wheel center relative to the suspension frame.
    /// Non-zero values might be used, for example, if the steer axis should not go
    /// through the wheel center.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `point3f physxVehicleWheelAttachment:wheelFramePosition = (0, 0, 0)` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Point3f |
    PHYSXSCHEMA_API
    UsdAttribute GetWheelFramePositionAttr() const;

    /// See GetWheelFramePositionAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWheelFramePositionAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // WHEELFRAMEORIENTATION 
    // --------------------------------------------------------------------- //
    /// An orientation adjustment of the wheel relative to the suspension frame.
    /// The rotation angle around the wheel's lateral axis is applied in this frame.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `quatf physxVehicleWheelAttachment:wheelFrameOrientation = (1, 0, 0, 0)` |
    /// | C++ Type | GfQuatf |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Quatf |
    PHYSXSCHEMA_API
    UsdAttribute GetWheelFrameOrientationAttr() const;

    /// See GetWheelFrameOrientationAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWheelFrameOrientationAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // DRIVEN 
    // --------------------------------------------------------------------- //
    /// Deprecated. Please use PhysxVehicleMultiWheelDifferentialAPI instead.
    /// 
    /// True if the tire is driven by the engine through the transmission. Will be ignored if the vehicle
    /// has no drive specified (see PhysxVehicleAPI:drive) or if PhysxVehicleMultiWheelDifferentialAPI
    /// is used instead.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxVehicleWheelAttachment:driven` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetDrivenAttr() const;

    /// See GetDrivenAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateDrivenAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // WHEEL 
    // --------------------------------------------------------------------- //
    /// A relationship to a PhysxVehicleWheelAPI prim.
    /// 
    /// Note: either this relationship has to be specified or the prim must have PhysxVehicleWheelAPI
    /// applied (none or both of the two is invalid).
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetWheelRel() const;

    /// See GetWheelRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateWheelRel() const;

public:
    // --------------------------------------------------------------------- //
    // TIRE 
    // --------------------------------------------------------------------- //
    /// A relationship to a PhysxVehicleTireAPI prim.
    /// 
    /// Note: either this relationship has to be specified or the prim must have PhysxVehicleTireAPI
    /// applied (none or both of the two is invalid).
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetTireRel() const;

    /// See GetTireRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateTireRel() const;

public:
    // --------------------------------------------------------------------- //
    // SUSPENSION 
    // --------------------------------------------------------------------- //
    /// A relationship to a PhysxVehicleSuspensionAPI prim.
    /// 
    /// Note: either this relationship has to be specified or the prim must have PhysxVehicleSuspensionAPI
    /// applied (none or both of the two is invalid).
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetSuspensionRel() const;

    /// See GetSuspensionRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateSuspensionRel() const;

public:
    // --------------------------------------------------------------------- //
    // COLLISIONGROUP 
    // --------------------------------------------------------------------- //
    /// A relationship to a PhysicsCollisionGroup instance that defines what the wheel/suspension scene queries should treat as a ground surface to collide against. If not specified, no specific filtering will be applied.
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetCollisionGroupRel() const;

    /// See GetCollisionGroupRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateCollisionGroupRel() const;

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
