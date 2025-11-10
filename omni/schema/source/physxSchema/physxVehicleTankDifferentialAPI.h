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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLETANKDIFFERENTIALAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLETANKDIFFERENTIALAPI_H

/// \file physxSchema/physxVehicleTankDifferentialAPI.h

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
// PHYSXVEHICLETANKDIFFERENTIALAPI                                            //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleTankDifferentialAPI
///
/// Differential to set up a wheeled tank vehicle. Describes which wheels of a vehicle are part of
/// the tank tracks. The wheels in each tank track have a constraint applied to them to enforce the
/// rule that they all have the same longitudinal speed at the contact point between the wheel and the
/// tank track. Driven wheels that are not part of a tank track receive the torque split specified in
/// physxVehicleMultiWheelDifferential:torqueRatios. Has to be applied to a prim with PhysxVehicleAPI
/// applied. Can only be used for vehicles that have a standard drive (see PhysxVehicleDriveStandardAPI).
///
class PhysxSchemaPhysxVehicleTankDifferentialAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleTankDifferentialAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleTankDifferentialAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleTankDifferentialAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleTankDifferentialAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleTankDifferentialAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleTankDifferentialAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleTankDifferentialAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleTankDifferentialAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleTankDifferentialAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleTankDifferentialAPI
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
    /// This information is stored by adding "PhysxVehicleTankDifferentialAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleTankDifferentialAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleTankDifferentialAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleTankDifferentialAPI 
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
    // NUMBEROFWHEELSPERTRACK 
    // --------------------------------------------------------------------- //
    /// For each track, this array holds the number of wheels assigned to that track.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] physxVehicleTankDifferential:numberOfWheelsPerTrack` |
    /// | C++ Type | VtArray<int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetNumberOfWheelsPerTrackAttr() const;

    /// See GetNumberOfWheelsPerTrackAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateNumberOfWheelsPerTrackAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // THRUSTINDEXPERTRACK 
    // --------------------------------------------------------------------- //
    /// For each track, this array holds the index of the thrust control (0 or 1, see PhysxVehicleTankControllerAPI)
    /// that drives the track. The length of the array needs to match the length of the 
    /// "numberOfWheelsPerTrack" array.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] physxVehicleTankDifferential:thrustIndexPerTrack` |
    /// | C++ Type | VtArray<int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetThrustIndexPerTrackAttr() const;

    /// See GetThrustIndexPerTrackAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateThrustIndexPerTrackAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // TRACKTOWHEELINDICES 
    // --------------------------------------------------------------------- //
    /// For each track, this array holds the start index of the list describing the wheels that
    /// are assigned to that track (e.g., the list of wheel indices assigned to the track with index
    /// 2 starts at: wheelIndicesInTrackOrder[trackToWheelIndices[2]]). The length of the array needs
    /// to match the length of the "numberOfWheelsPerTrack" array.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] physxVehicleTankDifferential:trackToWheelIndices` |
    /// | C++ Type | VtArray<int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetTrackToWheelIndicesAttr() const;

    /// See GetTrackToWheelIndicesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTrackToWheelIndicesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // WHEELINDICESINTRACKORDER 
    // --------------------------------------------------------------------- //
    /// The wheels (identified by wheel attachment indices, see attribtue "index" of 
    /// PhysxVehicleWheelAttachmentAPI) assigned to track 0, followed by the wheels assigned to
    /// track 1 and so on. The length of the array is expected to be equal (or larger than) the number
    /// of wheels that are assigned to tracks.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] physxVehicleTankDifferential:wheelIndicesInTrackOrder` |
    /// | C++ Type | VtArray<int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetWheelIndicesInTrackOrderAttr() const;

    /// See GetWheelIndicesInTrackOrderAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWheelIndicesInTrackOrderAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
