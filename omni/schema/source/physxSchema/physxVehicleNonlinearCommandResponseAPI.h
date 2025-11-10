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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLENONLINEARCOMMANDRESPONSEAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLENONLINEARCOMMANDRESPONSEAPI_H

/// \file physxSchema/physxVehicleNonlinearCommandResponseAPI.h

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
// PHYSXVEHICLENONLINEARCOMMANDRESPONSEAPI                                    //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI
///
/// Describes a system of graphs to define nonlinear responses to PhysxVehicleControllerAPI
/// command values like accelerator, brake0, brake1 and steer. The normalized response will be
/// a function of the command value and the longitudinal vehicle speed. The response will be
/// computed by interpolating between the points of the graph and then interpolating those
/// results again between the closest graphs. One example usage of nonlinear command response
/// is a brake pedal that has an almost flat response when tipped slightly but a very strong
/// response from a certain point on. Another example is the steering wheel showing a strong
/// response for a large input at low speed but only a weak response at high speed.
/// 
/// This multipleApply schema can be used to control the response to steering (use instance name
/// TfToken "steer") and braking (use instance name TfTokens "brakes0" or "brakes1"). It can
/// also control the response to the accelerator but only in combination with the basic drive (use
/// instance name TfToken "drive"). Furthermore, this API has to be applied to the appropriate
/// prims to take effect: "steer" has to be applied to a prim that has PhysxVehicleSteeringAPI or
/// PhysxVehicleAckermannSteeringAPI applied. "brakes0"/"brakes1" have to be applied to a prim that
/// has PhysxVehicleBrakesAPI:brakes0/brakes1 applied. "drive" has to be applied to a prim that
/// has PhysxVehicleDriveBasicAPI applied.
///
class PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::MultipleApplyAPI;

    /// Construct a PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI on UsdPrim \p prim with
    /// name \p name . Equivalent to
    /// PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI::Get(
    ///    prim.GetStage(),
    ///    prim.GetPath().AppendProperty(
    ///        "physxVehicleNCR:name"));
    ///
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI(
        const UsdPrim& prim=UsdPrim(), const TfToken &name=TfToken())
        : UsdAPISchemaBase(prim, /*instanceName*/ name)
    { }

    /// Construct a PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI on the prim held by \p schemaObj with
    /// name \p name.  Should be preferred over
    /// PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI(schemaObj.GetPrim(), name), as it preserves
    /// SchemaBase state.
    explicit PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI(
        const UsdSchemaBase& schemaObj, const TfToken &name)
        : UsdAPISchemaBase(schemaObj, /*instanceName*/ name)
    { }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI();

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

    /// Return a PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  \p path must be of the format
    /// <path>.physxVehicleNCR:name .
    ///
    /// This is shorthand for the following:
    ///
    /// \code
    /// TfToken name = SdfPath::StripNamespace(path.GetToken());
    /// PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI(
    ///     stage->GetPrimAtPath(path.GetPrimPath()), name);
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI
    Get(const UsdStagePtr &stage, const SdfPath &path);

    /// Return a PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI with name \p name holding the
    /// prim \p prim. Shorthand for PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI(prim, name);
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI
    Get(const UsdPrim &prim, const TfToken &name);

    /// Return a vector of all named instances of PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI on the 
    /// given \p prim.
    PHYSXSCHEMA_API
    static std::vector<PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI>
    GetAll(const UsdPrim &prim);

    /// Checks if the given name \p baseName is the base name of a property
    /// of PhysxVehicleNonlinearCommandResponseAPI.
    PHYSXSCHEMA_API
    static bool
    IsSchemaPropertyBaseName(const TfToken &baseName);

    /// Checks if the given path \p path is of an API schema of type
    /// PhysxVehicleNonlinearCommandResponseAPI. If so, it stores the instance name of
    /// the schema in \p name and returns true. Otherwise, it returns false.
    PHYSXSCHEMA_API
    static bool
    IsPhysxVehicleNonlinearCommandResponseAPIPath(const SdfPath &path, TfToken *name);

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
    /// This information is stored by adding "PhysxVehicleNonlinearCommandResponseAPI:<i>name</i>" 
    /// to the token-valued, listOp metadata \em apiSchemas on the prim.
    /// For example, if \p name is 'instance1', the token 
    /// 'PhysxVehicleNonlinearCommandResponseAPI:instance1' is added to 'apiSchemas'.
    /// 
    /// \return A valid PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI object is returned upon 
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
    static PhysxSchemaPhysxVehicleNonlinearCommandResponseAPI 
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
    // COMMANDVALUES 
    // --------------------------------------------------------------------- //
    /// The command values to define response graphs for. Each listed command value
    /// (in range [0, 1], steer commands are treated symmetrically) has to point to a graph
    /// in speedResponses. The command values refer to steer, brake0 etc. The values
    /// have to be strictly increasing. The number of entries has to match the number of
    /// entries in speedResponsesPerCommandValue and is limited to 8. Every command value
    /// needs at least one entry in speedResponses.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] commandValues` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetCommandValuesAttr() const;

    /// See GetCommandValuesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCommandValuesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SPEEDRESPONSESPERCOMMANDVALUE 
    // --------------------------------------------------------------------- //
    /// List of indices pointing to the start of a response graph for a certain command
    /// value. The index values have to be strictly increasing. The graph for command
    /// value commandValues[i] starts at entry speedResponses[speedResponsesPerCommandValue[i]]
    /// and stops at entry speedResponses[speedResponsesPerCommandValue[i+1] - 1]. The
    /// number of entries has to match the number of entries in commandValues and has the
    /// same maximum limit.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int[] speedResponsesPerCommandValue` |
    /// | C++ Type | VtArray<int> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->IntArray |
    PHYSXSCHEMA_API
    UsdAttribute GetSpeedResponsesPerCommandValueAttr() const;

    /// See GetSpeedResponsesPerCommandValueAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSpeedResponsesPerCommandValueAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SPEEDRESPONSES 
    // --------------------------------------------------------------------- //
    /// List of pairs that define points in graphs of longitudinal speed vs. normalized
    /// response. The first value of the pair is the longitudinal speed, the second value
    /// is the expected response (in range [0, 1]). The longitudinal speed values within
    /// a graph have to be strictly increasing. The maximum allowed number of entries is 64.
    /// See speedResponsesPerCommandValue for how the different graphs are accessed.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float2[] speedResponses` |
    /// | C++ Type | VtArray<GfVec2f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float2Array |
    PHYSXSCHEMA_API
    UsdAttribute GetSpeedResponsesAttr() const;

    /// See GetSpeedResponsesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSpeedResponsesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
