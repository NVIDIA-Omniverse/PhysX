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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXSCENEAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXSCENEAPI_H

/// \file physxSchema/physxSceneAPI.h

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
// PHYSXSCENEAPI                                                              //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxSceneAPI
///
/// PhysX scene extended parameters.
/// Default PhysX material for the scene can be set by using material bind with purpose physics onto the PhysicsScene prim.
///
/// For any described attribute \em Fallback \em Value or \em Allowed \em Values below
/// that are text/tokens, the actual token is published and defined in \ref PhysxSchemaTokens.
/// So to set an attribute to the value "rightHanded", use PhysxSchemaTokens->rightHanded
/// as the value.
///
class PhysxSchemaPhysxSceneAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxSceneAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxSceneAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxSceneAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxSceneAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxSceneAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxSceneAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxSceneAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxSceneAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxSceneAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxSceneAPI
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
    /// This information is stored by adding "PhysxSceneAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxSceneAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxSceneAPI object is returned upon 
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
    static PhysxSchemaPhysxSceneAPI 
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
    // BOUNCETHRESHOLD 
    // --------------------------------------------------------------------- //
    /// A contact with a relative velocity below this will not bounce.
    /// Range: [0, inf)
    /// Default: 0
    /// Units: distance / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxScene:bounceThreshold = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetBounceThresholdAttr() const;

    /// See GetBounceThresholdAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateBounceThresholdAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FRICTIONOFFSETTHRESHOLD 
    // --------------------------------------------------------------------- //
    /// A threshold of contact separation distance used to decide if a contact point will experience friction forces.
    /// Range: [0, inf)
    /// Default: 0.04 * tolerancesLength
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxScene:frictionOffsetThreshold = 0.04` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetFrictionOffsetThresholdAttr() const;

    /// See GetFrictionOffsetThresholdAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFrictionOffsetThresholdAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FRICTIONCORRELATIONDISTANCE 
    // --------------------------------------------------------------------- //
    /// Friction correlation distance used to decide whether contacts are close enough to be merged into a single friction anchor point or not.
    /// Range: [0, inf)
    /// Default: 0.025 * tolerancesLength
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxScene:frictionCorrelationDistance = 0.025` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetFrictionCorrelationDistanceAttr() const;

    /// See GetFrictionCorrelationDistanceAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFrictionCorrelationDistanceAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXBIASCOEFFICIENT 
    // --------------------------------------------------------------------- //
    /// The maximum bias coefficient used in the constraint solver.
    /// Range: [0, inf)
    /// Units: dimensionless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxScene:maxBiasCoefficient = inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxBiasCoefficientAttr() const;

    /// See GetMaxBiasCoefficientAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxBiasCoefficientAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // COLLISIONSYSTEM 
    // --------------------------------------------------------------------- //
    /// Collision detection system.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token physxScene:collisionSystem = "PCM"` |
    /// | C++ Type | TfToken |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | PCM, SAT |
    PHYSXSCHEMA_API
    UsdAttribute GetCollisionSystemAttr() const;

    /// See GetCollisionSystemAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCollisionSystemAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SOLVERTYPE 
    // --------------------------------------------------------------------- //
    /// Solver used for the simulation.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token physxScene:solverType = "TGS"` |
    /// | C++ Type | TfToken |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | PGS, TGS |
    PHYSXSCHEMA_API
    UsdAttribute GetSolverTypeAttr() const;

    /// See GetSolverTypeAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSolverTypeAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // BROADPHASETYPE 
    // --------------------------------------------------------------------- //
    /// Broad phase algorithm used in the simulation.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token physxScene:broadphaseType = "GPU"` |
    /// | C++ Type | TfToken |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | GPU, MBP, SAP |
    PHYSXSCHEMA_API
    UsdAttribute GetBroadphaseTypeAttr() const;

    /// See GetBroadphaseTypeAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateBroadphaseTypeAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FRICTIONTYPE 
    // --------------------------------------------------------------------- //
    /// Deprecated since friction type "patch" is the only option now.
    /// 
    /// Friction type used in simulation.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token physxScene:frictionType = "patch"` |
    /// | C++ Type | TfToken |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | patch |
    PHYSXSCHEMA_API
    UsdAttribute GetFrictionTypeAttr() const;

    /// See GetFrictionTypeAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFrictionTypeAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLECCD 
    // --------------------------------------------------------------------- //
    /// Enables a second broad phase check after integration that makes it possible to prevent objects from tunneling through each other.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:enableCCD = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableCCDAttr() const;

    /// See GetEnableCCDAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableCCDAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLESTABILIZATION 
    // --------------------------------------------------------------------- //
    /// Enables additional stabilization pass in solver.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:enableStabilization = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableStabilizationAttr() const;

    /// See GetEnableStabilizationAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableStabilizationAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // UPDATETYPE 
    // --------------------------------------------------------------------- //
    /// Type of update for a PhysX scene.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform token physxScene:updateType = "Synchronous"` |
    /// | C++ Type | TfToken |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Token |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    /// | \ref PhysxSchemaTokens "Allowed Values" | Synchronous, Asynchronous, Disabled |
    PHYSXSCHEMA_API
    UsdAttribute GetUpdateTypeAttr() const;

    /// See GetUpdateTypeAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateUpdateTypeAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLEGPUDYNAMICS 
    // --------------------------------------------------------------------- //
    /// Enables the GPU dynamics pipeline. Required for GPU only features like deformables.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:enableGPUDynamics = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableGPUDynamicsAttr() const;

    /// See GetEnableGPUDynamicsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableGPUDynamicsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLEENHANCEDDETERMINISM 
    // --------------------------------------------------------------------- //
    /// Provides improved determinism at the expense of performance.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:enableEnhancedDeterminism = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableEnhancedDeterminismAttr() const;

    /// See GetEnableEnhancedDeterminismAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableEnhancedDeterminismAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLERESIDUALREPORTING 
    // --------------------------------------------------------------------- //
    /// Calculates solver residuals and reports them through optional Residual Report APIs at a slight expense of performance.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:enableResidualReporting = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableResidualReportingAttr() const;

    /// See GetEnableResidualReportingAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableResidualReportingAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLEEXTERNALFORCESEVERYITERATION 
    // --------------------------------------------------------------------- //
    /// Enables greater TGS solver stability.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:enableExternalForcesEveryIteration = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableExternalForcesEveryIterationAttr() const;

    /// See GetEnableExternalForcesEveryIterationAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableExternalForcesEveryIterationAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLESCENEQUERYSUPPORT 
    // --------------------------------------------------------------------- //
    /// Enables scene query support for collision shapes.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:enableSceneQuerySupport = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetEnableSceneQuerySupportAttr() const;

    /// See GetEnableSceneQuerySupportAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateEnableSceneQuerySupportAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // TIMESTEPSPERSECOND 
    // --------------------------------------------------------------------- //
    /// Simulation scene step defined as number of steps per second.
    /// Note that application might cap the number of simulation steps to avoid
    /// running more simulations steps with a low frame rate.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:timeStepsPerSecond = 60` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetTimeStepsPerSecondAttr() const;

    /// See GetTimeStepsPerSecondAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateTimeStepsPerSecondAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUTEMPBUFFERCAPACITY 
    // --------------------------------------------------------------------- //
    /// Gpu temp buffer capacity.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint64 physxScene:gpuTempBufferCapacity = 16777216` |
    /// | C++ Type | uint64_t |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt64 |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuTempBufferCapacityAttr() const;

    /// See GetGpuTempBufferCapacityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuTempBufferCapacityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUMAXRIGIDCONTACTCOUNT 
    // --------------------------------------------------------------------- //
    /// Gpu max rigid contact count.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuMaxRigidContactCount = 524288` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuMaxRigidContactCountAttr() const;

    /// See GetGpuMaxRigidContactCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuMaxRigidContactCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUMAXRIGIDPATCHCOUNT 
    // --------------------------------------------------------------------- //
    /// Gpu max rigid patch count.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuMaxRigidPatchCount = 81920` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuMaxRigidPatchCountAttr() const;

    /// See GetGpuMaxRigidPatchCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuMaxRigidPatchCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUHEAPCAPACITY 
    // --------------------------------------------------------------------- //
    /// Gpu heap capacity.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuHeapCapacity = 67108864` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuHeapCapacityAttr() const;

    /// See GetGpuHeapCapacityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuHeapCapacityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUFOUNDLOSTPAIRSCAPACITY 
    // --------------------------------------------------------------------- //
    /// Gpu found lost pairs capacity.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuFoundLostPairsCapacity = 262144` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuFoundLostPairsCapacityAttr() const;

    /// See GetGpuFoundLostPairsCapacityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuFoundLostPairsCapacityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUFOUNDLOSTAGGREGATEPAIRSCAPACITY 
    // --------------------------------------------------------------------- //
    /// Gpu found lost aggregate pairs capacity.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuFoundLostAggregatePairsCapacity = 1024` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuFoundLostAggregatePairsCapacityAttr() const;

    /// See GetGpuFoundLostAggregatePairsCapacityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuFoundLostAggregatePairsCapacityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUTOTALAGGREGATEPAIRSCAPACITY 
    // --------------------------------------------------------------------- //
    /// Gpu total aggregate pairs capacity.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuTotalAggregatePairsCapacity = 1024` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuTotalAggregatePairsCapacityAttr() const;

    /// See GetGpuTotalAggregatePairsCapacityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuTotalAggregatePairsCapacityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUMAXSOFTBODYCONTACTS 
    // --------------------------------------------------------------------- //
    /// Gpu max soft body contacts.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuMaxSoftBodyContacts = 1048576` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuMaxSoftBodyContactsAttr() const;

    /// See GetGpuMaxSoftBodyContactsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuMaxSoftBodyContactsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUMAXDEFORMABLESURFACECONTACTS 
    // --------------------------------------------------------------------- //
    /// Gpu max deformable surface contacts.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuMaxDeformableSurfaceContacts = 1048576` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuMaxDeformableSurfaceContactsAttr() const;

    /// See GetGpuMaxDeformableSurfaceContactsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuMaxDeformableSurfaceContactsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUMAXPARTICLECONTACTS 
    // --------------------------------------------------------------------- //
    /// Gpu max particle contacts.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuMaxParticleContacts = 1048576` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuMaxParticleContactsAttr() const;

    /// See GetGpuMaxParticleContactsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuMaxParticleContactsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUMAXNUMPARTITIONS 
    // --------------------------------------------------------------------- //
    /// Limitation for the partitions in the GPU dynamics pipeline. This variable must be a power of 2. A value greater than 32 is currently not supported.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuMaxNumPartitions = 8` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuMaxNumPartitionsAttr() const;

    /// See GetGpuMaxNumPartitionsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuMaxNumPartitionsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GPUCOLLISIONSTACKSIZE 
    // --------------------------------------------------------------------- //
    /// Gpu collision stack size.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxScene:gpuCollisionStackSize = 67108864` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetGpuCollisionStackSizeAttr() const;

    /// See GetGpuCollisionStackSizeAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGpuCollisionStackSizeAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // INVERTCOLLISIONGROUPFILTER 
    // --------------------------------------------------------------------- //
    /// Boolean attribute indicating whether inverted collision group filtering should be used.
    /// By default two collisions, that do have a collisionGroup set, collide with each other. Adding
    /// a collisionGroup into a collisionGroup filtering will mean that the collision between those groups
    /// will be disabled. This boolean attribute does invert the default behavior. Hence two collisions with
    /// defined collisionGroups will not collide with each other by default and one does enable the
    /// collisions between the groups through the "CollisionGroup" filtering.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:invertCollisionGroupFilter = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetInvertCollisionGroupFilterAttr() const;

    /// See GetInvertCollisionGroupFilterAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateInvertCollisionGroupFilterAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // REPORTKINEMATICKINEMATICPAIRS 
    // --------------------------------------------------------------------- //
    /// Boolean attribute indicating whether kinematic vs kinematic pairs
    /// generate contact reports.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:reportKinematicKinematicPairs = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetReportKinematicKinematicPairsAttr() const;

    /// See GetReportKinematicKinematicPairsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateReportKinematicKinematicPairsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // REPORTKINEMATICSTATICPAIRS 
    // --------------------------------------------------------------------- //
    /// Boolean attribute indicating whether kinematic vs static pairs
    /// generate contact reports.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxScene:reportKinematicStaticPairs = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetReportKinematicStaticPairsAttr() const;

    /// See GetReportKinematicStaticPairsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateReportKinematicStaticPairsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MINPOSITIONITERATIONCOUNT 
    // --------------------------------------------------------------------- //
    /// Minimum position iteration count for all actors (rigid bodies, cloth, particles etc).
    /// Range: [1, 255]
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform uint physxScene:minPositionIterationCount = 1` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    PHYSXSCHEMA_API
    UsdAttribute GetMinPositionIterationCountAttr() const;

    /// See GetMinPositionIterationCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMinPositionIterationCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXPOSITIONITERATIONCOUNT 
    // --------------------------------------------------------------------- //
    /// Maximum position iteration count for all actors (rigid bodies, cloth, particles etc).
    /// Note that this setting will override solver iteration settings of individual actors that have requested more iterations.
    /// Range: [1, 255]
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform uint physxScene:maxPositionIterationCount = 255` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxPositionIterationCountAttr() const;

    /// See GetMaxPositionIterationCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxPositionIterationCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MINVELOCITYITERATIONCOUNT 
    // --------------------------------------------------------------------- //
    /// Minimum velocity iteration count for all actors (rigid bodies, cloth, particles etc).
    /// Range: [0, 255]
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform uint physxScene:minVelocityIterationCount = 0` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    PHYSXSCHEMA_API
    UsdAttribute GetMinVelocityIterationCountAttr() const;

    /// See GetMinVelocityIterationCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMinVelocityIterationCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXVELOCITYITERATIONCOUNT 
    // --------------------------------------------------------------------- //
    /// Maximum velocity iteration count for all actors (rigid bodies, cloth, particles etc).
    /// Note that this setting will override solver iteration settings of individual actors that have requested more iterations.
    /// Range: [0, 255]
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform uint physxScene:maxVelocityIterationCount = 255` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxVelocityIterationCountAttr() const;

    /// See GetMaxVelocityIterationCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxVelocityIterationCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
