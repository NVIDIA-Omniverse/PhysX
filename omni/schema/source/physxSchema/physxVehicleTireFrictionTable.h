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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXVEHICLETIREFRICTIONTABLE_H
#define PHYSXSCHEMA_GENERATED_PHYSXVEHICLETIREFRICTIONTABLE_H

/// \file physxSchema/physxVehicleTireFrictionTable.h

#include "pxr/pxr.h"
#include ".//api.h"
#include "pxr/usd/usd/typed.h"
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
// PHYSXVEHICLETIREFRICTIONTABLE                                              //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxVehicleTireFrictionTable
///
/// Table defining the friction values of a tire against a given set of ground materials.
///
class PhysxSchemaPhysxVehicleTireFrictionTable : public UsdTyped
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::ConcreteTyped;

    /// Construct a PhysxSchemaPhysxVehicleTireFrictionTable on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxVehicleTireFrictionTable::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxVehicleTireFrictionTable(const UsdPrim& prim=UsdPrim())
        : UsdTyped(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxVehicleTireFrictionTable on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxVehicleTireFrictionTable(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxVehicleTireFrictionTable(const UsdSchemaBase& schemaObj)
        : UsdTyped(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxVehicleTireFrictionTable();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxVehicleTireFrictionTable holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxVehicleTireFrictionTable(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxVehicleTireFrictionTable
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
    static PhysxSchemaPhysxVehicleTireFrictionTable
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
    // FRICTIONVALUES 
    // --------------------------------------------------------------------- //
    /// The friction values to use for the tire against the ground materials listed under groundMaterials. The array needs to have the same size as the list of materials specified in groundMaterials.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float[] frictionValues` |
    /// | C++ Type | VtArray<float> |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->FloatArray |
    PHYSXSCHEMA_API
    UsdAttribute GetFrictionValuesAttr() const;

    /// See GetFrictionValuesAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateFrictionValuesAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // DEFAULTFRICTIONVALUE 
    // --------------------------------------------------------------------- //
    /// The friction value to use for the tire against the ground if the material of the ground surface is not among the materials
    /// listed in groundMaterials. The value has to be greater or equal 0.
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float defaultFrictionValue = 1` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetDefaultFrictionValueAttr() const;

    /// See GetDefaultFrictionValueAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateDefaultFrictionValueAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // GROUNDMATERIALS 
    // --------------------------------------------------------------------- //
    /// Relationships to Material instances that have PhysicsMaterialAPI applied. For each material, a friction value has to be specified in the frictionValues attribute. If a material is encountered that has not been listed, the value declared in defaultFrictionValue will be used.
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetGroundMaterialsRel() const;

    /// See GetGroundMaterialsRel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateGroundMaterialsRel() const;

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
