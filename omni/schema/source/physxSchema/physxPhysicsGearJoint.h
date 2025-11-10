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
#ifndef PHYSXSCHEMA_GENERATED_PHYSXPHYSICSGEARJOINT_H
#define PHYSXSCHEMA_GENERATED_PHYSXPHYSICSGEARJOINT_H

/// \file physxSchema/physxPhysicsGearJoint.h

#include "pxr/pxr.h"
#include ".//api.h"
#include "pxr/usd/usdPhysics/joint.h"
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
// PHYSXPHYSICSGEARJOINT                                                      //
// -------------------------------------------------------------------------- //

/// \class PhysxSchemaPhysxPhysicsGearJoint
///
/// A gear joint connects two existing revolute (a.k.a. hinge) joints and constrains the angular velocities of involved rigid bodies. If omega0 is the angular velocity of the first rigid body around the PhysicsRevoluteJoint axis, omega1 is the angular velocity of the second rigid body around the PhysicsRevoluteJoint axis, and R is the gear ratio, then the enforced constraint is abs(omega1) = abs(omega0) * R. The equation is valid for the absolute values of angular velocities. The angular velocities may otherwise have opposite signs, depending on the orientations of the joint frames. Gear joints can be used with GPU simulation but in that case these specific joints will run through the CPU pipeline. If a gear joint is created between articulation revolute joints, excludeFromArticulation must be set to true for the gear joint itself, otherwise an error occurs. To include the gear joint in the articulation itself, please use PhysxMimicJointAPI instead to implement a gear joint. A known limitation is that current implementation does not work for planetary gears.
///
class PhysxSchemaPhysxPhysicsGearJoint : public UsdPhysicsJoint
{
public:
    /// Compile time constant representing what kind of schema this class is.
    ///
    /// \sa UsdSchemaKind
    static const UsdSchemaKind schemaKind = UsdSchemaKind::ConcreteTyped;

    /// Construct a PhysxSchemaPhysxPhysicsGearJoint on UsdPrim \p prim .
    /// Equivalent to PhysxSchemaPhysxPhysicsGearJoint::Get(prim.GetStage(), prim.GetPath())
    /// for a \em valid \p prim, but will not immediately throw an error for
    /// an invalid \p prim
    explicit PhysxSchemaPhysxPhysicsGearJoint(const UsdPrim& prim=UsdPrim())
        : UsdPhysicsJoint(prim)
    {
    }

    /// Construct a PhysxSchemaPhysxPhysicsGearJoint on the prim held by \p schemaObj .
    /// Should be preferred over PhysxSchemaPhysxPhysicsGearJoint(schemaObj.GetPrim()),
    /// as it preserves SchemaBase state.
    explicit PhysxSchemaPhysxPhysicsGearJoint(const UsdSchemaBase& schemaObj)
        : UsdPhysicsJoint(schemaObj)
    {
    }

    /// Destructor.
    PHYSXSCHEMA_API
    virtual ~PhysxSchemaPhysxPhysicsGearJoint();

    /// Return a vector of names of all pre-declared attributes for this schema
    /// class and all its ancestor classes.  Does not include attributes that
    /// may be authored by custom/extended methods of the schemas involved.
    PHYSXSCHEMA_API
    static const TfTokenVector &
    GetSchemaAttributeNames(bool includeInherited=true);

    /// Return a PhysxSchemaPhysxPhysicsGearJoint holding the prim adhering to this
    /// schema at \p path on \p stage.  If no prim exists at \p path on
    /// \p stage, or if the prim at that path does not adhere to this schema,
    /// return an invalid schema object.  This is shorthand for the following:
    ///
    /// \code
    /// PhysxSchemaPhysxPhysicsGearJoint(stage->GetPrimAtPath(path));
    /// \endcode
    ///
    PHYSXSCHEMA_API
    static PhysxSchemaPhysxPhysicsGearJoint
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
    static PhysxSchemaPhysxPhysicsGearJoint
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
    // GEARRATIO 
    // --------------------------------------------------------------------- //
    /// Gear ratio. For two gears with N0 and N1 teeth respectively, the gear ratio should be N0/N1.
    /// Range: (-inf, inf)
    /// Units: dimensionless
    ///
    /// | ||
    /// | -- | -- |
    /// | Declaration | `float physics:gearRatio = 1` |
    /// | C++ Type | float |
    /// | \ref Usd_Datatypes "Usd Type" | SdfValueTypeNames->Float |
    PHYSXSCHEMA_API
    UsdAttribute GetGearRatioAttr() const;

    /// See GetGearRatioAttr(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create.
    /// If specified, author \p defaultValue as the attribute's default,
    /// sparsely (when it makes sense to do so) if \p writeSparsely is \c true -
    /// the default for \p writeSparsely is \c false.
    PHYSXSCHEMA_API
    UsdAttribute CreateGearRatioAttr(VtValue const &defaultValue = VtValue(), bool writeSparsely=false) const;

public:
    // --------------------------------------------------------------------- //
    // HINGE0 
    // --------------------------------------------------------------------- //
    /// Relationship to first hinge joint. It should be a PhysicsRevoluteJoint. It is mandatory to define one (and only one) relationship here.
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetHinge0Rel() const;

    /// See GetHinge0Rel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateHinge0Rel() const;

public:
    // --------------------------------------------------------------------- //
    // HINGE1 
    // --------------------------------------------------------------------- //
    /// Relationship to second hinge joint. It should be a PhysicsRevoluteJoint. It is mandatory to define one (and only one) relationship here.
    ///
    PHYSXSCHEMA_API
    UsdRelationship GetHinge1Rel() const;

    /// See GetHinge1Rel(), and also 
    /// \ref Usd_Create_Or_Get_Property for when to use Get vs Create
    PHYSXSCHEMA_API
    UsdRelationship CreateHinge1Rel() const;

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
