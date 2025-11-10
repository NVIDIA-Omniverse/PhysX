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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXPARTICLECLOTHAPI_H
#define PHYSXSCHEMA_GENERATED_PHYSXPARTICLECLOTHAPI_H

/// \file physxSchema/physxParticleClothAPI.h

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
// PHYSXPARTICLECLOTHAPI                                                      //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxParticleClothAPI
///
/// Deprecated: Will be replaced by a new deformable schema in a future release.
/// WARNING: This is a draft API; the design is not fixed and may change in the future.
/// Applied to a UsdGeomMesh. The mesh is simulated as a particle-based cloth or inflatable, depending on the mesh geometry
/// and the pressure attribute. See PhysxAutoParticleClothAPI for an auto-compute helper for cloth springs and dampings.
///
class PhysxSchemaPhysxParticleClothAPI : public UsdAPISchemaBase
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::SingleApplyAPI;

    /// Construct a PhysxSchemaPhysxParticleClothAPI on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxParticleClothAPI::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxParticleClothAPI(const UsdPrim& prim=UsdPrim())
        : UsdAPISchemaBase(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxParticleClothAPI on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxParticleClothAPI(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxParticleClothAPI(const UsdSchemaBase& schemaObj)
        : UsdAPISchemaBase(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxParticleClothAPI();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxParticleClothAPI holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxParticleClothAPI(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxParticleClothAPI
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
    /// This information is stored by adding "PhysxParticleClothAPI" to the 
    /// token-valued, listOp metadata \em apiSchemas on the prim.
    /// 
    /// \return A valid PhysxSchemaPhysxParticleClothAPI object is returned upon success. 
    /// An invalid (or empty) PhysxSchemaPhysxParticleClothAPI object is returned upon 
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
    static PhysxSchemaPhysxParticleClothAPI 
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
    // SELFCOLLISIONFILTER 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Whether the simulation should filter particle-particle collisions based on their rest position distances.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `bool physxParticle:selfCollisionFilter = 1` |
    /// | C++ Type | bool |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Bool |
    PHYSXSCHEMA_API
    UsdAttribute GetSelfCollisionFilterAttr() const;

    /// See GetSelfCollisionFilterAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSelfCollisionFilterAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // RESTPOINTS 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Particle positions in cloth rest configuration.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `point3f[] physxParticle:restPoints` |
    /// | C++ Type | VtArray<GfVec3f> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Point3fArray |
    PHYSXSCHEMA_API
    UsdAttribute GetRestPointsAttr() const;

    /// See GetRestPointsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateRestPointsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SPRINGINDICES 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// List of index pairs describing which points are connected by spring constraints.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `uniform int2[] physxParticle:springIndices` |
    /// | C++ Type | VtArray<GfVec2i> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Int2Array |
    /// | \ref SdfVariability "Variability" | SdfVariabilityUniform |
    PHYSXSCHEMA_API
    UsdAttribute GetSpringIndicesAttr() const;

    /// See GetSpringIndicesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSpringIndicesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SPRINGSTIFFNESSES 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Stiffness values for linear-distance springs between a pair of particles as defined by index pairs in springIndices.
    /// Range: [0, inf)
    /// Units: force / distance = mass / second / second
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] physxParticle:springStiffnesses` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetSpringStiffnessesAttr() const;

    /// See GetSpringStiffnessesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSpringStiffnessesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SPRINGDAMPINGS 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Coefficients for linear-distance dampers between a pair of particles as defined by index pairs in springIndices.
    /// Range: [0, inf)
    /// Units: force * second / distance = mass / second
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] physxParticle:springDampings` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetSpringDampingsAttr() const;

    /// See GetSpringDampingsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSpringDampingsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // SPRINGRESTLENGTHS 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Spring rest lengths corresponding to spring index pairs.
    /// Range: (-inf, inf)
    /// Units: distance
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] physxParticle:springRestLengths` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetSpringRestLengthsAttr() const;

    /// See GetSpringRestLengthsAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateSpringRestLengthsAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // PRESSURE 
    // --------------------------------------------------------------------- //
    /// Deprecated: Will be replaced by a new deformable schema in a future release.
    /// Pressure: If > 0, a particle cloth has an additional pressure constraint 
    /// that provides inflatable (i.e. balloon-like) dynamics. The pressure times the rest volume defines 
    /// the volume the inflatable tries to match. Pressure only works well for closed or approximately closed meshes.
    /// Range: [0, inf)
    /// Units: dimensionless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physxParticle:pressure = 0` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetPressureAttr() const;

    /// See GetPressureAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreatePressureAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

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
