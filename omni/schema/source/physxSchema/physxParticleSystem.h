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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXPARTICLESYSTEM_H
#define PHYSXSCHEMA_GENERATED_PHYSXPARTICLESYSTEM_H

/// \file physxSchema/physxParticleSystem.h

#include "pxr/pxr.h"
#include ".//api.h"
#include "pxr/usd/usdGeom/gprim.h"
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
// PHYSXPARTICLESYSTEM                                                        //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxParticleSystem
///
/// WARNING: This is a draft API; the design is not fixed and may change in the future.
/// PhysX particle system, used to simulate fluids, cloth and inflatables. This prim allows the user to configure the
/// solver parameters that are common to the particle objects associated with this system via their particleSystem relationship.
///
class PhysxSchemaPhysxParticleSystem : public UsdGeomGprim
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::ConcreteTyped;

    /// Construct a PhysxSchemaPhysxParticleSystem on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxParticleSystem::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxParticleSystem(const UsdPrim& prim=UsdPrim())
        : UsdGeomGprim(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxParticleSystem on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxParticleSystem(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxParticleSystem(const UsdSchemaBase& schemaObj)
        : UsdGeomGprim(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxParticleSystem();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxParticleSystem holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxParticleSystem(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxParticleSystem
    Get(const UsdStagePtr &stage, const SdfPath &path);

    /// Attempt to ensure a \a UsdPrim adhering to this schema at \p path
    /// is defined (according to UsdPrim::IsDefined()) on this stage.
    ///
    /// If a prim adhering to this schema at \p path is already defined on this
    /// stage, return that prim.  Otherwise author an \a SdfPrimSpec with
    /// \a specifier == \a SdfSpecifierDef and this schema's prim type name for
    /// the prim at \p path at the current EditTarget.  Author \a SdfPrimSpec s
    /// with \p specifier == \a SdfSpecifierDef and empty typeName at the
    /// current EditTarget for any nonexistent, or existing but not \a Defined
    /// ancestors.
    ///
    /// The given \a path must be an absolute prim path that does not contain
    /// any variant selections.
    ///
    /// If it is impossible to author any of the necessary PrimSpecs, (for
    /// example, in case \a path cannot map to the current UsdEditTarget's
    /// namespace) issue an error and return an invalid \a UsdPrim.
    ///
    /// Note that this method may return a defined prim whose typeName does not
    /// specify this schema class, in case a stronger typeName opinion overrides
    /// the opinion at the current EditTarget.
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxParticleSystem
    Define(const UsdStagePtr &stage, const SdfPath &path);

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
    // PARTICLESYSTEMENABLED 
    // --------------------------------------------------------------------- //
    /// Enable or disable the particle system.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool particleSystemEnabled = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetParticleSystemEnabledAttr() const;

    /// See GetParticleSystemEnabledAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateParticleSystemEnabledAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // CONTACTOFFSET 
    // --------------------------------------------------------------------- //
    /// Contact offset used for collisions with non-particle objects such as rigid or deformable bodies.
    /// Must be larger than restOffset. Default value -inf results in a simulation-determined value.
    /// Range: (restOffset, inf)
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float contactOffset = -inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetContactOffsetAttr() const;

    /// See GetContactOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateContactOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // RESTOFFSET 
    // --------------------------------------------------------------------- //
    /// Rest offset used for collisions with non-particle objects such as rigid or deformable bodies.
    /// Must be smaller than contact offset. Default value -inf results in a simulation-determined value.
    /// Range: [0, contactOffset)
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float restOffset = -inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetRestOffsetAttr() const;

    /// See GetRestOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateRestOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PARTICLECONTACTOFFSET 
    // --------------------------------------------------------------------- //
    /// Contact offset used for interactions between particles. Must be larger than solid and fluid rest offsets.
    /// Range: (max(solidRestOffset, fluidRestOffset), inf)
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float particleContactOffset = 0.05` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetParticleContactOffsetAttr() const;

    /// See GetParticleContactOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateParticleContactOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SOLIDRESTOFFSET 
    // --------------------------------------------------------------------- //
    /// Rest offset used for solid-solid or solid-fluid particle interactions. Must be smaller than particleContactOffset.
    /// Default value -inf results in a simulation-determined value.
    /// Range: [0, particleContactOffset)
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float solidRestOffset = -inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetSolidRestOffsetAttr() const;

    /// See GetSolidRestOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSolidRestOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // FLUIDRESTOFFSET 
    // --------------------------------------------------------------------- //
    /// Rest offset used for fluid-fluid particle interactions. Must be smaller than particleContactOffset.
    /// Default value -inf results in a simulation-determined value.
    /// Range: [0, particleContactOffset)
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float fluidRestOffset = -inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetFluidRestOffsetAttr() const;

    /// See GetFluidRestOffsetAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFluidRestOffsetAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // ENABLECCD 
    // --------------------------------------------------------------------- //
    /// Enable continuous collision detection for particles to help avoid tunneling effects.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool enableCCD = 0` |
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
    // SOLVERPOSITIONITERATIONCOUNT 
    // --------------------------------------------------------------------- //
    /// Number of solver iterations for position.
    /// Range: [1, 255]
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int solverPositionIterationCount = 16` |
    /// | C++ Type | int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
    PHYSXSCHEMA_API
    UsdAttribute GetSolverPositionIterationCountAttr() const;

    /// See GetSolverPositionIterationCountAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSolverPositionIterationCountAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXDEPENETRATIONVELOCITY 
    // --------------------------------------------------------------------- //
    /// The maximum velocity permitted to be introduced by the solver to depenetrate intersecting particles.
    /// Range: [0, inf)
    /// Units: distance / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float maxDepenetrationVelocity = inf` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxDepenetrationVelocityAttr() const;

    /// See GetMaxDepenetrationVelocityAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxDepenetrationVelocityAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // WIND 
    // --------------------------------------------------------------------- //
    /// The wind applied to the current particle system.
    /// Range: (-inf, inf)
    /// Units: distance / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float3 wind = (0, 0, 0)` |
    /// | C++ Type | GfVec3f |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float3 |
    PHYSXSCHEMA_API
    UsdAttribute GetWindAttr() const;

    /// See GetWindAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateWindAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXNEIGHBORHOOD 
    // --------------------------------------------------------------------- //
    /// Defines how many particle neighbors per particle may be considered for interaction computations.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `int maxNeighborhood = 96` |
    /// | C++ Type | int |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int |
    PHYSXSCHEMA_API
    UsdAttribute GetMaxNeighborhoodAttr() const;

    /// See GetMaxNeighborhoodAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateMaxNeighborhoodAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // NEIGHBORHOODSCALE 
    // --------------------------------------------------------------------- //
    /// Defines by how much the default neighborhood volume is inflated to ensure that all relevant neighboring particles are found while particles interact. Simulations with high relative velocities might require larger volumes.
    /// Range: [1, inf)
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float neighborhoodScale = 1.01` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetNeighborhoodScaleAttr() const;

    /// See GetNeighborhoodScaleAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateNeighborhoodScaleAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // MAXVELOCITY 
    // --------------------------------------------------------------------- //
    /// Maximum particle velocity. See also cflCoefficient in PhysxPBDMaterialAPI for limiting particle-particle relative velocity.
    /// Range: [0, inf)
    /// Units: distance / seconds
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float maxVelocity = inf` |
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
    // --------------------------------------------------------------------- //
    // GLOBALSELFCOLLISIONENABLED 
    // --------------------------------------------------------------------- //
    /// If True, self collisions follow particle-object-specific settings. If False, all particle self collisions are disabled, regardless of any other settings.
    /// Improves performance if self collisions are not needed.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool globalSelfCollisionEnabled = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetGlobalSelfCollisionEnabledAttr() const;

    /// See GetGlobalSelfCollisionEnabledAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGlobalSelfCollisionEnabledAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // NONPARTICLECOLLISIONENABLED 
    // --------------------------------------------------------------------- //
    /// Enable or disable particle collision with nonparticle objects for all particles in the system.
    /// Improves performance if nonparticle collisions are not needed.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool nonParticleCollisionEnabled = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetNonParticleCollisionEnabledAttr() const;

    /// See GetNonParticleCollisionEnabledAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateNonParticleCollisionEnabledAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SIMULATIONOWNER 
    // --------------------------------------------------------------------- //
    /// Single PhysicsScene that simulates this particle system. By default,
    /// this is the first PhysicsScene found in the stage using UsdStage::Traverse().
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetSimulationOwnerRel() const;

    /// See GetSimulationOwnerRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateSimulationOwnerRel() const;

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
