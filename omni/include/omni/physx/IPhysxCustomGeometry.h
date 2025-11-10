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
static const size_t kInvalidCustomGeometryRegId = 0;


/// Create custom geometry function
///
/// \param[in] sdfPath SdfPath of the UsdGeomGPrim prim.
/// \param[in] stageId USD stageId.
/// \param[in] typeId Custom geometry typeId
/// \param[in] userData User data passed to ICustomGeometryCallback struct
/// \return  User pointer that gets provided for each implementation function.
typedef void* (*CreateCustomGeometryFn)(pxr::SdfPath sdfPath,
                                        long stageId,
                                        const ::physx::PxCustomGeometry::Type& typeId,
                                        void* userData);

/// Release custom geometry function
///
/// \param[in] sdfPath SdfPath of the geom prim.
/// \param[in] userData User data passed to ICustomGeometryCallback struct
typedef void (*ReleaseCustomGeometryFn)(pxr::SdfPath sdfPath, void* userData);

/// Compute local bounds function
///
/// \param[in] geometry		This geometry.
/// \param[in] userObject	Created user custom geometry through CreateCustomGeometryFn
/// \return  Bounding box in the geometry local space.
typedef ::physx::PxBounds3 (*ComputeCustomGeometryLocalBoundsFn)(const ::physx::PxGeometry& geometry, void* userObject);

/// Contacts generation function. Generate collision contacts between two geometries in given poses.
///
/// \param[in] geom0				This custom geometry
/// \param[in] geom1				The other geometry
/// \param[in] pose0				This custom geometry pose
/// \param[in] pose1				The other geometry pose
/// \param[in] contactDistance		The distance at which contacts begin to be generated between the pairs
/// \param[in] meshContactMargin	The mesh contact margin.
/// \param[in] toleranceLength		The toleranceLength. Used for scaling distance-based thresholds internally to
/// produce appropriate results given simulations in different units \param[out] contactBuffer		A buffer to write
/// contacts to. \param[in] userObject0	        Created user custom geometry through CreateCustomGeometryFn belongs to
/// geom0, this pointer is always valid. \param[in] userObject1	        Created user custom geometry through
/// CreateCustomGeometryFn belongs to geom1, this pointer is only valid if geom1 is a custom geometry too.
///
/// \return True if there are contacts. False otherwise.
typedef bool (*GenerateCustomGeometryContactsFn)(const ::physx::PxGeometry& geom0,
                                                 const ::physx::PxGeometry& geom1,
                                                 const ::physx::PxTransform& pose0,
                                                 const ::physx::PxTransform& pose1,
                                                 const float contactDistance,
                                                 const float meshContactMargin,
                                                 const float toleranceLength,
                                                 ::physx::PxContactBuffer& contactBuffer,
                                                 void* userObject0,
                                                 void* userObject1);

/// Raycast function. Cast a ray against the geometry in given pose. Optional (scene query functionality will not work).
///
/// \param[in] origin			Origin of the ray.
/// \param[in] unitDir			Normalized direction of the ray.
/// \param[in] geom				This custom geometry
/// \param[in] pose				This custom geometry pose
/// \param[in] maxDist			Length of the ray. Has to be in the [0, inf) range.
/// \param[in] hitFlags			Specifies which properties per hit should be computed and returned via the hit callback.
/// \param[in] maxHits			max number of returned hits = size of 'rayHits' buffer
/// \param[out] rayHits			Ray hits.
/// \param[in] stride			Ray hit structure stride.
/// \param[in] threadContext	Optional user-defined per-thread context.
/// \param[in] userObject	    Created user custom geometry through CreateCustomGeometryFn
///
/// \return Number of hits.
typedef uint32_t (*RaycastCustomGeometryFn)(const ::physx::PxVec3& origin,
                                            const ::physx::PxVec3& unitDir,
                                            const ::physx::PxGeometry& geom,
                                            const ::physx::PxTransform& pose,
                                            float maxDist,
                                            ::physx::PxHitFlags hitFlags,
                                            uint32_t maxHits,
                                            ::physx::PxGeomRaycastHit* rayHits,
                                            uint32_t stride,
                                            ::physx::PxRaycastThreadContext* threadContext,
                                            void* userObject);

/// Overlap function. Test if geometries overlap. Optional (scene query functionality will not work).
///
/// \param[in] geom0			This custom geometry
/// \param[in] pose0			This custom geometry pose
/// \param[in] geom1			The other geometry
/// \param[in] pose1			The other geometry pose
/// \param[in] threadContext	Optional user-defined per-thread context.
/// \param[in] userObject	    Created user custom geometry through CreateCustomGeometryFn
///
/// \return True if there is overlap. False otherwise.
typedef bool (*OverlapCustomGeometryFn)(const ::physx::PxGeometry& geom0,
                                        const ::physx::PxTransform& pose0,
                                        const ::physx::PxGeometry& geom1,
                                        const ::physx::PxTransform& pose1,
                                        ::physx::PxOverlapThreadContext* threadContext,
                                        void* userObject);

/// Sweep function. Sweep one geometry against the other. Optional (scene query functionality will not work).
///
/// \param[in] unitDir			Normalized direction of the sweep.
/// \param[in] maxDist			Length of the sweep. Has to be in the [0, inf) range.
/// \param[in] geom0			This custom geometry
/// \param[in] pose0			This custom geometry pose
/// \param[in] geom1			The other geometry
/// \param[in] pose1			The other geometry pose
/// \param[out] sweepHit		Used to report the sweep hit.
/// \param[in] hitFlags			Specifies which properties per hit should be computed and returned via the hit callback.
/// \param[in] inflation		This parameter creates a skin around the swept geometry which increases its extents for
/// sweeping. \param[in] threadContext	Optional user-defined per-thread context. \param[in] userObject	    Created user
/// custom geometry through CreateCustomGeometryFn
///
/// \return True if there is hit. False otherwise.
typedef bool (*SweepCustomGeometryFn)(const ::physx::PxVec3& unitDir,
                                      const float maxDist,
                                      const ::physx::PxGeometry& geom0,
                                      const ::physx::PxTransform& pose0,
                                      const ::physx::PxGeometry& geom1,
                                      const ::physx::PxTransform& pose1,
                                      ::physx::PxGeomSweepHit& sweepHit,
                                      ::physx::PxHitFlags hitFlags,
                                      const float inflation,
                                      ::physx::PxSweepThreadContext* threadContext,
                                      void* userObject);

/// Visualize custom geometry for debugging function. Optional.
///
/// \param[in] geometry		This geometry.
/// \param[in] pose		Geometry absolute transform.
/// \param[in] out			Render output.
/// \param[in] cullbox		Region to visualize.
/// \param[in] userObject	    Created user custom geometry through CreateCustomGeometryFn
typedef void (*VisualizeCustomGeometryFn)(const ::physx::PxGeometry& geometry,
                                          const ::physx::PxTransform& pose,
                                          ::physx::PxRenderOutput& out,
                                          const ::physx::PxBounds3& cullbox,
                                          void* userObject);

/// Compute custom geometry mass properties function. For geometries usable with dynamic rigid bodies.
///
/// \param[in] geometry			This geometry.
/// \param[out] massProperties	Mass properties to compute.
/// \param[in] userObject	    Created user custom geometry through CreateCustomGeometryFn
typedef void (*ComputeCustomGeometryMassPropertiesFn)(const ::physx::PxGeometry& geometry,
                                                      ::physx::PxMassProperties& massProperties,
                                                      void* userObject);

/// Compatible with PhysX's PCM feature function. Allows to optimize contact generation.
///
/// \param[in] geometry				This geometry.
/// \param[out] breakingThreshold	The threshold to trigger contacts re-generation.
/// \param[in] userObject	        Created user custom geometry through CreateCustomGeometryFn
typedef bool (*UseCustomGeometryPersistentContactManifoldFn)(const ::physx::PxGeometry& geometry,
                                                             float& breakingThreshold,
                                                             void* userObject);

/// Custom geometry structure holding function pointers for callbacks
struct ICustomGeometryCallback
{
    CreateCustomGeometryFn createCustomGeometryFn = { nullptr };
    ReleaseCustomGeometryFn releaseCustomGeometryFn = { nullptr };
    ComputeCustomGeometryLocalBoundsFn computeCustomGeometryLocalBoundsFn = { nullptr };
    GenerateCustomGeometryContactsFn generateCustomGeometryContactsFn = { nullptr };
    RaycastCustomGeometryFn raycastCustomGeometryFn = { nullptr };
    OverlapCustomGeometryFn overlapCustomGeometryFn = { nullptr };
    SweepCustomGeometryFn sweepCustomGeometryFn = { nullptr };
    VisualizeCustomGeometryFn visualizeCustomGeometryFn = { nullptr };
    ComputeCustomGeometryMassPropertiesFn computeCustomGeometryMassPropertiesFn = { nullptr };
    UseCustomGeometryPersistentContactManifoldFn useCustomGeometryPersistentContactManifoldFn = { nullptr };
    void* userData = { nullptr };
};

struct IPhysxCustomGeometry
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxCustomGeometry", 1, 0)

    /// Register custom geometry
    ///
    /// Provide a callback structure with custom geometry function definitions. The provided custom geometry
    /// schemaAPI token will ensure that omni.physx will create on that path a custom PhysX SDK geometry
    /// that will call functions provided through the callback.
    ///
    /// \param[in] geometrySchemaAPIToken SchemaAPI applied to the UsdGeomGPrim together with the
    /// UsdPhysicsCollisionAPI. \param[in] geometryCallback Custom geomtry callbacks that will get fired. \return
    /// Registration id, used for unregister. Returns kInvalidCustomGeometryRegId on failure.
    size_t(CARB_ABI* registerCustomGeometry)(const pxr::TfToken& geometrySchemaAPIToken,
                                             ICustomGeometryCallback& geometryCallback);

    /// Unregister custom geometry
    ///
    /// \param[in] id Registration id
    void(CARB_ABI* unregisterCustomGeometry)(size_t id);
};

} // namespace physx
} // namespace omni
