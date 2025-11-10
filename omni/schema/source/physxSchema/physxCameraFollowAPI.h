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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXCAMERAFOLLOWAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXCAMERAFOLLOWAPI_H

/// \file physxSchema/physxCameraFollowAPI.h

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
// PHYSXCAMERAFOLLOWAPI                                                       //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxCameraFollowAPI
///
/// PhysX camera that follows behind the subject as it moves.
///
class PhysxSchemaPhysxCameraFollowAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxCameraFollowAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxCameraFollowAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxCameraFollowAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxCameraFollowAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxCameraFollowAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxCameraFollowAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxCameraFollowAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxCameraFollowAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxCameraFollowAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxCameraFollowAPI
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
    /// This information is stored by adding "PhysxCameraFollowAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxCameraFollowAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxCameraFollowAPI object is returned upon 
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
    static PhysxSchemaPhysxCameraFollowAPI 
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
    // YAWANGLE 
    // --------------------------------------------------------------------- //
    /// The yaw angle of the follow vector around the subject. Zero is directly behind the subject. e.g. 0 degrees
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:yawAngle` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetYawAngleAttr() const;

    /// See GetYawAngleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateYawAngleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PITCHANGLE 
    // --------------------------------------------------------------------- //
    /// The pitch angle of the follow vector around the subject. Zero is directly behind the subject. e.g. 15 degrees
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:pitchAngle` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetPitchAngleAttr() const;

    /// See GetPitchAngleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePitchAngleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PITCHANGLETIMECONSTANT 
    // --------------------------------------------------------------------- //
    /// Time constant to filter the pitch angle, in seconds. Used to pitch the camera up and down when driving up or down hills. e.g. 0.2 seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:pitchAngleTimeConstant` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetPitchAngleTimeConstantAttr() const;

    /// See GetPitchAngleTimeConstantAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePitchAngleTimeConstantAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SLOWSPEEDPITCHANGLESCALE 
    // --------------------------------------------------------------------- //
    /// Scale of the camera pitch angle at slow speed. This lowers the camera behind the subject at slow speeds. e.g. 0.5
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:slowSpeedPitchAngleScale` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetSlowSpeedPitchAngleScaleAttr() const;

    /// See GetSlowSpeedPitchAngleScaleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSlowSpeedPitchAngleScaleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SLOWPITCHANGLESPEED 
    // --------------------------------------------------------------------- //
    /// Scale of the camera pitch angle at slow speed. This lowers the camera behind the subject at slow speeds. e.g. 1000 cm/sec
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:slowPitchAngleSpeed` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetSlowPitchAngleSpeedAttr() const;

    /// See GetSlowPitchAngleSpeedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSlowPitchAngleSpeedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // VELOCITYNORMALMINSPEED 
    // --------------------------------------------------------------------- //
    /// The minimum speed, below which, the subject look vector must be used because the normalized velocity vector is too erratic. e.g. 600.0 cm/sec
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:velocityNormalMinSpeed` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetVelocityNormalMinSpeedAttr() const;

    /// See GetVelocityNormalMinSpeedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateVelocityNormalMinSpeedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FOLLOWMINSPEED 
    // --------------------------------------------------------------------- //
    /// The minimum speed used for a linear interpolation to compute the follow distance of the camera. e.g. 300.0 cm/sec
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:followMinSpeed` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetFollowMinSpeedAttr() const;

    /// See GetFollowMinSpeedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFollowMinSpeedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FOLLOWMINDISTANCE 
    // --------------------------------------------------------------------- //
    /// The minimum distance used for a linear interpolation to compute the follow distance of the camera. e.g. 1500.0 cm
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:followMinDistance` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetFollowMinDistanceAttr() const;

    /// See GetFollowMinDistanceAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFollowMinDistanceAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FOLLOWMAXSPEED 
    // --------------------------------------------------------------------- //
    /// The maximum speed used for a linear interpolation to compute the follow distance of the camera. e.g. 3000.0 cm/sec
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:followMaxSpeed` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetFollowMaxSpeedAttr() const;

    /// See GetFollowMaxSpeedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFollowMaxSpeedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FOLLOWMAXDISTANCE 
    // --------------------------------------------------------------------- //
    /// The maximum distance used for a linear interpolation to compute the follow distance of the camera. e.g. 1000.0 cm
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:followMaxDistance` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetFollowMaxDistanceAttr() const;

    /// See GetFollowMaxDistanceAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFollowMaxDistanceAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // YAWRATETIMECONSTANT 
    // --------------------------------------------------------------------- //
    /// Time constant to filter the subject yaw rate, in seconds. Use to look into turns. e.g. 0.2 sec
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:yawRateTimeConstant` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetYawRateTimeConstantAttr() const;

    /// See GetYawRateTimeConstantAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateYawRateTimeConstantAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FOLLOWTURNRATEGAIN 
    // --------------------------------------------------------------------- //
    /// A scale factor that multiplies the filtered yaw rate to yaw the camera position behind the subject. Can be any positive or negative number. e.g. 0.2
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:followTurnRateGain` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetFollowTurnRateGainAttr() const;

    /// See GetFollowTurnRateGainAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFollowTurnRateGainAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CAMERAPOSITIONTIMECONSTANT 
    // --------------------------------------------------------------------- //
    /// Filter time constant for the position of the camera for each axis, in seconds. e.g. (0.5 sec, 0.1 sec, 0.5 sec)
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float3 physxFollowCamera:cameraPositionTimeConstant` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float3 |
    PHYSXSCHEMA_API
    UsdAttribute GetCameraPositionTimeConstantAttr() const;

    /// See GetCameraPositionTimeConstantAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCameraPositionTimeConstantAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // POSITIONOFFSET 
    // --------------------------------------------------------------------- //
    /// Position offset from the subject center of mass from which the camera follow vector and look vector are computed. e.g. (0.0 cm, 10.0 cm, 0.0 cm)
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float3 physxFollowCamera:positionOffset` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float3 |
    PHYSXSCHEMA_API
    UsdAttribute GetPositionOffsetAttr() const;

    /// See GetPositionOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePositionOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOOKAHEADMINSPEED 
    // --------------------------------------------------------------------- //
    /// The minimum speed used for a linear interpolation to compute the look ahead distance of the camera look point. e.g. 0.0 cm/sec
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:lookAheadMinSpeed` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLookAheadMinSpeedAttr() const;

    /// See GetLookAheadMinSpeedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLookAheadMinSpeedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOOKAHEADMINDISTANCE 
    // --------------------------------------------------------------------- //
    /// The minimum distance used for a linear interpolation to compute the look ahead distance of the camera look point. e.g. 0.0 cm
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:lookAheadMinDistance` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLookAheadMinDistanceAttr() const;

    /// See GetLookAheadMinDistanceAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLookAheadMinDistanceAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOOKAHEADMAXSPEED 
    // --------------------------------------------------------------------- //
    /// The maximum speed used for a linear interpolation to compute the look ahead distance of the camera look point. e.g. 2000.0 cm
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:lookAheadMaxSpeed` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLookAheadMaxSpeedAttr() const;

    /// See GetLookAheadMaxSpeedAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLookAheadMaxSpeedAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOOKAHEADMAXDISTANCE 
    // --------------------------------------------------------------------- //
    /// The maximum distance used for a linear interpolation to compute the look ahead distance of the camera look point. e.g.  500.0 cm
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowFollowCamera:lookAheadMaxDistance` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLookAheadMaxDistanceAttr() const;

    /// See GetLookAheadMaxDistanceAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLookAheadMaxDistanceAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOOKAHEADTURNRATEGAIN 
    // --------------------------------------------------------------------- //
    /// A scale factor that multiplies the filtered yaw rate to yaw the camera look point left or right. Can be any positive or negative number. e.g. 0.2
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:lookAheadTurnRateGain` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLookAheadTurnRateGainAttr() const;

    /// See GetLookAheadTurnRateGainAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLookAheadTurnRateGainAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOOKPOSITIONHEIGHT 
    // --------------------------------------------------------------------- //
    /// Distance to elevate the camera look point. e.g. 50.0 cm
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxFollowCamera:lookPositionHeight` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetLookPositionHeightAttr() const;

    /// See GetLookPositionHeightAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLookPositionHeightAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // LOOKPOSITIONTIMECONSTANT 
    // --------------------------------------------------------------------- //
    /// Filter time constant for the camera look point, in seconds. e.g. (0.2 sec, 0.5 sec, 0.2 sec)
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float3 physxFollowCamera:lookPositionTimeConstant` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float3 |
    PHYSXSCHEMA_API
    UsdAttribute GetLookPositionTimeConstantAttr() const;

    /// See GetLookPositionTimeConstantAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateLookPositionTimeConstantAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
