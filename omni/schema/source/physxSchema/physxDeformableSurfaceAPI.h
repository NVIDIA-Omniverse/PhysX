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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXDEFORMABLESURFACEAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXDEFORMABLESURFACEAPI_H

/// \file physxSchema/physxDeformableSurfaceAPI.h

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
// PHYSXDEFORMABLESURFACEAPI                                                  //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxDeformableSurfaceAPI
///
/// Deprecated: Will be replaced by a new deformable schema in a future release.
/// WARNING: This is a draft API; the design is not fixed and may change in the future.
/// Applied to a UsdGeomMesh that is to be simulated as a deformable surface.
/// See PhysxDeformableAPI for information on how to define the deformable surface's material properties.
///
class PhysxSchemaPhysxDeformableSurfaceAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxDeformableSurfaceAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxDeformableSurfaceAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxDeformableSurfaceAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxDeformableSurfaceAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxDeformableSurfaceAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxDeformableSurfaceAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxDeformableSurfaceAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxDeformableSurfaceAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxDeformableSurfaceAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxDeformableSurfaceAPI
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
    /// This information is stored by adding "PhysxDeformableSurfaceAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxDeformableSurfaceAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxDeformableSurfaceAPI object is returned upon 
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
    static PhysxSchemaPhysxDeformableSurfaceAPI 
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
    // FLATTENINGENABLED 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// If enabled, bending forces are applied such that the surface flattens, i.e., neighboring triangles become coplanar.
    /// If disabled, the forces drive toward the bend at the rest state.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxDeformableSurface:flatteningEnabled = 0` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetFlatteningEnabledAttr() const;

    /// See GetFlatteningEnabledAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFlatteningEnabledAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // BENDINGSTIFFNESSSCALE 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Scales bending stiffness computed from Young's modulus, Poisson's ratio, and the cloth thickness.
    /// Range: [0, inf)
    /// Units: dimensionless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxDeformableSurface:bendingStiffnessScale = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetBendingStiffnessScaleAttr() const;

    /// See GetBendingStiffnessScaleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateBendingStiffnessScaleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // COLLISIONPAIRUPDATEFREQUENCY 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Determines how often cloth-to-cloth collision pairs are updated during each time step. 
    /// Increasing this value results in more frequent updates to the contact pairs, which provides better contact points. 
    /// For example, a value of 2 means collision pairs are updated twice per time step: 
    /// once at the beginning and once in the middle of the time step (i.e., during the middle solver iteration). 
    /// If set to 0, the solver adaptively determines when to update the cloth-to-cloth contact pairs, instead of using a fixed frequency.
    /// Range [1, solverPositionIterationCount]
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxDeformableSurface:collisionPairUpdateFrequency = 1` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetCollisionPairUpdateFrequencyAttr() const;

    /// See GetCollisionPairUpdateFrequencyAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCollisionPairUpdateFrequencyAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // COLLISIONITERATIONMULTIPLIER 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Determines how many collision subiterations are used in each solver iteration.
    /// By default, collision constraints are applied once per solver iteration.
    /// Increasing this value applies collision constraints more frequently within each solver iteration.
    /// For example, a value of 2 means collision constraints are applied twice per solver iteration 
    /// (i.e., collision constraints are applied 2 x solverPositionIterationCount times per time step).
    /// Increasing this value does not update collision pairs more frequently; refer to collisionPairUpdateFrequency for that.
    /// Range [1, solverPositionIterationCount/2]
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uint physxDeformableSurface:collisionIterationMultiplier = 1` |
    /// | C++ Type | unsigned int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->UInt |
    PHYSXSCHEMA_API
    UsdAttribute GetCollisionIterationMultiplierAttr() const;

    /// See GetCollisionIterationMultiplierAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateCollisionIterationMultiplierAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXVELOCITY 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Maximum velocity: the velocity of each cloth vertex is clamped by this maximum velocity value.
    /// If set to a negative value, the simulation determines a default value.
    /// Range: [0, inf)
    /// Units: distance / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxDeformableSurface:maxVelocity = inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxVelocityAttr() const;

    /// See GetMaxVelocityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxVelocityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
