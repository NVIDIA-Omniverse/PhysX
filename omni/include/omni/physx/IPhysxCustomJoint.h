// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

// Note requires PhysX SDK includes before including this file

namespace omni
{
namespace physx
{
static const size_t kInvalidCustomJointRegId = 0;

/// Custom joint flags
struct CustomJointFlag
{
    enum Enum
    {
        eALWAYS_UPDATE = 1 << 0, //!< updates the constraint each frame
    };
};

/// Create custom joint function
///
/// \param[in] sdfPath SdfPath of the joint prim.
/// \param[in] stageId USD stageId.
/// \param[in] actor0 PxRigidActor for the joint.
/// \param[in] localFrame0 Transformation for local frame 0 for the joint.
/// \param[in] actor1 PxRigidActor for the joint.
/// \param[in] localFrame1 Transformation for local frame 1 for the joint.
/// \param[in] constraintFlags Constraint flag for the constrain creation.
/// \param[in] userData User data passed to ICustomJointCallback struct
typedef bool (*CreateJointFn)(pxr::SdfPath sdfPath,
                              long stageId,
                              ::physx::PxRigidActor* actor0,
                              const ::physx::PxTransform& localFrame0,
                              ::physx::PxRigidActor* actor1,
                              const ::physx::PxTransform& localFrame1,
                              CustomJointFlag::Enum& constraintFlags,
                              void* userData);

/// Release custom joint function
///
/// \param[in] sdfPath SdfPath of the joint prim.
/// \param[in] userData User data passed to ICustomJointCallback struct
typedef void (*ReleaseJointFn)(pxr::SdfPath sdfPath, void* userData);

/// Prepare joint data function
///
/// When the constraint is marked dirty, this function is called at the start of the simulation
/// step for the SDK to copy the constraint data block.
///
/// \param[in] sdfPath SdfPath of the joint prim.
/// \param[in] userData User data passed to ICustomJointCallback struct
/// \return Return the joint data
typedef void* (*PrepareJointDataFn)(pxr::SdfPath sdfPath, void* userData);

/// On CoM Shift function
///
///	This function is called by the SDK when the CoM of one of the actors is moved. Since the
/// API specifies constraint positions relative to actors, and the constraint shader functions
/// are supplied with coordinates relative to bodies, some synchronization is usually required
/// when the application moves an object's center of mass.
///
/// \param[in] sdfPath SdfPath of the joint prim.
/// \param[in] userData User data passed to ICustomJointCallback struct
typedef void (*OnComShiftFn)(pxr::SdfPath sdfPath, uint32_t actor, void* userData);

/// On Origin Shift function
///
/// This function is called by the SDK when the scene origin gets shifted and allows to adjust
/// custom data which contains world space transforms.
///
/// \note If the adjustments affect constraint shader data, it is necessary to call PxConstraint::markDirty()
/// to make sure that the data gets synced at the beginning of the next simulation step.
///
/// \param[in] sdfPath SdfPath of the joint prim.
/// \param[in] shift Translation vector the origin is shifted by.
/// \param[in] userData User data passed to ICustomJointCallback struct
typedef void (*OnOriginShift)(pxr::SdfPath sdfPath, const ::physx::PxVec3& shift, void* userData);

/// Constant block data get function
///
/// Obtain the pointer to the constraint's constant data
///
/// \param[in] sdfPath SdfPath of the joint prim.
/// \param[in] userData User data passed to ICustomJointCallback struct
/// \return Return the constraint data pointer
typedef const void* (*GetConstantBlockFn)(pxr::SdfPath sdfPath, void* userData);

/// Custom joint strcuture holding function pointers for callbacks
struct ICustomJointCallback
{
    CreateJointFn createJointFn = { nullptr };
    ReleaseJointFn releaseJointFn = { nullptr };
    PrepareJointDataFn prepareJointDataFn = { nullptr };
    OnComShiftFn onComShiftFn = { nullptr };
    OnOriginShift onOriginShift = { nullptr };
    GetConstantBlockFn getConstantBlockFn = { nullptr };
    void* userData = { nullptr };
};

struct IPhysxCustomJoint
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxCustomJoint", 1, 0)

    /// Register custom joint
    ///
    /// Provide a callback structure with joint custom function definition. The provided custom joint
    /// prim type will ensure that omni.physx will create on that path a custom PhysX SDK constraint
    /// that will call function provided through the joint callback and the jointSolverPrepFn.
    ///
    /// \param jointPrimType Joint prim type for the custom joint, must inherit from UsdPhysicsJoint.
    /// \param jointCallback Joint callbacks that will get fired when the joint should be created and released.
    /// \param jointSolverPrepFn Joint solver prep function.
    /// \param jointDataSize Joint data size that are provided for the constraint solver prep code.
    /// \return Registration id, used for unregister, return kInvalidCustomJointRegId when failed.
    size_t(CARB_ABI* registerCustomJoint)(const pxr::TfToken& jointPrimType,
                                          ICustomJointCallback& jointCallback,
                                          ::physx::PxConstraintSolverPrep jointSolverPrepFn,
                                          size_t jointDataSize);

    /// Unregister custom joint
    ///
    /// \param id Registration id
    void(CARB_ABI* unregisterCustomJoint)(size_t id);

    /// Mark joint dirty
    ///
    /// Notify the scene that the constraint shader data has been updated by the application
    ///
    /// \param primpath Path of the joint to mark dirty
    void(CARB_ABI* markJointDirty)(const pxr::SdfPath& primPath);

    /// Set joint flags
    ///
    /// Change the joint constaint flags.
    ///
    /// \param primpath Path of the joint to mark dirty
    /// \param flags New joint flags
    void(CARB_ABI* setJointFlags)(const pxr::SdfPath& primPath, CustomJointFlag::Enum flags);

    /// Get joint flags
    ///
    /// Get the joint constaint flags.
    ///
    /// \param primpath Path of the joint to mark dirty
    /// \return Current joint flags
    CustomJointFlag::Enum(CARB_ABI* getJointFlags)(const pxr::SdfPath& primPath);
};

} // namespace physx
} // namespace omni
