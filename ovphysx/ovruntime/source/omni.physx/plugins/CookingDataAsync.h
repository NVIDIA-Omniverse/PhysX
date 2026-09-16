// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-MATH-001
 * @covers AC-9
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-14 AC-16 AC-27
 *
 * @implements REQ-PUBLICAPI-002
 * @covers AC-10
 */

#pragma once

#include <foundation/PxMat44.h>
#include <vector>
#include <carb/Types.h>
#include <omni/physx/IPhysxCooking.h>
#include <omni/physics/AttachHandle.h>
#include <omni/physics/parse/Handles.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>
// ParticleSamplingDesc / PhysxDeformableBodyDesc / PhysxVolumeDeformableBodyDesc /
// PhysxSurfaceDeformableBodyDesc are `using` aliases of the parse-lib types
// (PhysxUsd.h, ADR-0019 increment 7) -- a forward struct-declaration would
// conflict with the alias, so the full header is included instead.
#include <private/omni/physx/PhysxUsd.h>

// Performs cooking operations asynchronously and non-blocking

// Forward reference the PhysX data types used by this API
namespace physx
{
class PxDefaultMemoryInputData;
class PxDefaultMemoryOutputStream;
class PxConvexMesh;
class PxTriangleMesh;
class PxCooking;
class PxPhysics;
} // namespace physx

// Forward reference the classes in the omni::physx::usdparser namespace used by this API
namespace omni
{
namespace physx
{
struct IPhysxCookingServicePrivate;

struct ConvexMeshData;
struct CollisionRepresentation;
namespace usdparser
{
class MeshKey;
class AttachedStage;

} // namespace usdparser
} // namespace physx
namespace physics
{
namespace parse
{
struct PhysxShapeDesc;
struct ConvexMeshPhysxShapeDesc;
struct ConvexMeshDecompositionPhysxShapeDesc;
struct TriangleMeshPhysxShapeDesc;
struct SpherePointsPhysxShapeDesc;
} // namespace parse
} // namespace physics
namespace physx
{
namespace usdparser
{
using PhysxShapeDesc                        = ::omni::physics::parse::PhysxShapeDesc;
using ConvexMeshPhysxShapeDesc              = ::omni::physics::parse::ConvexMeshPhysxShapeDesc;
using ConvexMeshDecompositionPhysxShapeDesc = ::omni::physics::parse::ConvexMeshDecompositionPhysxShapeDesc;
using TriangleMeshPhysxShapeDesc            = ::omni::physics::parse::TriangleMeshPhysxShapeDesc;
using SpherePointsPhysxShapeDesc            = ::omni::physics::parse::SpherePointsPhysxShapeDesc;
} // namespace usdparser
} // namespace physx
} // namespace omni

// The CookingDataAsync API resides in the 'cookingdataasync' namespace
namespace cookingdataasync
{

/**
 * CookingDataAsync is the public pure virtual API interface in support of all
 * PhysX related cooking operations.  This is treated as a singleton in the omni.physx
 * extension and owned by PhysxSetup.  Whenever the PhysX SDk is reset, this interface
 * is also reset.
 */
class CookingDataAsync
{
public:
    /**
     * This method is called once per logical 'frame' from the main thread to
     * dispatch new cooking tasks as well as process the results of cooking tasks
     * which have completed.
     *
     * @return : Returns the number of cooking tasks still active/pending
     */
    virtual uint32_t pump(void) = 0;

    /**
     * This method returns the PhysX PxConvexMesh associated with this USD prim if available.
     * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
     * to compute the result.
     *
     * @param desc : The convex mesh shape descriptor which defines the properties to apply when creating the convex
     * mesh approximation
     * @param primKey : Source-side ObjectKey of the prim associated with this convex mesh
     * @param attachedStage : The attached stage owning the prim (resolves the key to a path + stage id at cook time)
     * @param asynchronous : If false the convex mesh will be cooked synchronously (blocking) in this thread. If true,
     * it will spawn a background task if necessary.
     *
     * @return : Returns the pointer to the PxConvexMesh if it was available at this time.
     */
    virtual ::physx::PxConvexMesh* getConvexMesh(const omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc,
                                                 omni::physics::parse::ObjectKey primKey,
                                                 const omni::physx::usdparser::AttachedStage& attachedStage,
                                                 bool asynchronous,
                                                 omni::physx::IPhysxCookingCallback* cb = nullptr) = 0;

    virtual const omni::physx::usdparser::SpherePointsPhysxShapeDesc* getSpherePoints(
        const omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc,
        omni::physics::parse::ObjectKey primKey,
        const omni::physx::usdparser::AttachedStage& attachedStage,
        bool asynchronous,
        omni::physx::IPhysxCookingCallback* cb = nullptr) = 0;

    /**
     * This method returns the PhysX PxTriangleMesh associated with this USD prim if available.
     * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
     * to compute the result.
     *
     * @param desc : The triangle mesh shape descriptor which defines the properties to apply when creating the triangle
     * mesh approximation
     * @param primKey : Source-side ObjectKey of the prim associated with this triangle mesh
     * @param attachedStage : The attached stage owning the prim (resolves the key to a path + stage id at cook time)
     * @param asynchronous : If false the triangle mesh will be cooked synchronously (blocking) in this thread. If true,
     * it will spawn a background task if necessary.
     * @param maxMaterialIndex: Returns the highest index of the material used in the triangle mesh
     *
     * @return : Returns the pointer to the PxTriangleMesh if it was available at this time.
     */
    virtual ::physx::PxTriangleMesh* getTriangleMesh(const omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                                                     omni::physics::parse::ObjectKey primKey,
                                                     const omni::physx::usdparser::AttachedStage& attachedStage,
                                                     bool asynchronous,
                                                     omni::physx::IPhysxCookingCallback* cb = nullptr,
                                                     uint16_t* maxMaterialIndex = nullptr) = 0;

    /**
     * This method returns the array of PxConvexMeshes associated with this USD prim if available.
     * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
     * to compute the result.
     *
     * @param desc : The convex decomposition shape descriptor which defines the properties to apply when creating the
     * convex decomposition approximation
     * @param primKey : Source-side ObjectKey of the prim associated with this convex decomposition
     * @param attachedStage : The attached stage owning the prim (resolves the key to a path + stage id at cook time)
     * @param asynchronous : If false the convex decomposition will be cooked synchronously (blocking) in this thread.
     * If true, it will spawn a background task if necessary.
     *
     * @return : Returns an std::vector of PxConvexMeshes if it was available at this time.
     */
    virtual std::vector<::physx::PxConvexMesh*> getConvexMeshDecomposition(
        const omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc,
        omni::physics::parse::ObjectKey primKey,
        const omni::physx::usdparser::AttachedStage& attachedStage,
        bool asynchronous,
        omni::physx::IPhysxCookingCallback* cb = nullptr) = 0;

    /**
     * Helper method, that creates a PxDeformableVolumeMesh from the input data stream and parses the surface triangle
     * to tetrahedron map.
     *
     * @param outCollMeshSurfaceTriToTetMap : output collision mesh surface triangle to tetrahedron map
     * @param inData : input data (@see cookDeformableVolumeMesh)
     *
     * @return : Returns the PxDeformableVolumeMesh instance. The caller of the method is responsible for releasing the
     * PxDeformableVolumeMesh.
     */
    virtual ::physx::PxDeformableVolumeMesh* createDeformableVolumeMesh(
        std::vector<uint32_t>& outCollMeshSurfaceTriToTetMap,
        ::physx::PxDefaultMemoryInputData& inData) = 0;

    /**
     * Parses a deformable body.
     *
     * Note: Parsing can be executed before a deformable has been cooked, since only the prim and API structure
     * are parsed, not the bulk data attributes.
     *
     * @param bodyKey : Source-side ObjectKey of the root prim to parse.
     * @param attachedStage : The attached stage owning the prim.
     * @return : Deformable body descriptor.
     */
    virtual omni::physx::usdparser::PhysxDeformableBodyDesc* parseDeformableBody(
        omni::physics::parse::ObjectKey bodyKey,
        const omni::physx::usdparser::AttachedStage& attachedStage) = 0;

    /**
     * Runs the auto cook for a volume / surface deformable body and publishes the cooked sim and
     * collision geometry back into the scene description.
     *
     * @param desc : The parsed deformable body descriptor.
     * @param bodyKey : Source-side ObjectKey of the prim with UsdPhysicsDeformableBodyAPI.
     * @param attachedStage : The attached stage owning the prim.
     * @param asynchronous : Whether or not to cook asynchronously.
     *
     * @return : Returns true if the cooked data is available in the scene description once the call
     * returns - either because this call published it, because it was already up to date (CRC
     * match), because the body is already registered for simulation (the cook is a re-entrancy
     * no-op), or because an asynchronous cook was submitted. Returns false when the cook could not
     * run or its output could not be published - most notably when the parse backend vends no write
     * sink, in which case every array write is dropped and nothing is produced. A false return is
     * always accompanied by a diagnostic; the caller must not treat it as a completed cook.
     */
    virtual bool cookVolumeDeformableBody(const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          omni::physics::parse::ObjectKey bodyKey,
                                          const omni::physx::usdparser::AttachedStage& attachedStage,
                                          bool asynchronous) = 0;

    /** @see cookVolumeDeformableBody */
    virtual bool cookSurfaceDeformableBody(const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
                                           omni::physics::parse::ObjectKey bodyKey,
                                           const omni::physx::usdparser::AttachedStage& attachedStage,
                                           bool asynchronous) = 0;

    /**
     * Cooks a PxDeformableVolumeMesh, creates a surface triangle list and returns the result in a stream.
     * This operation reads the tetrahedral meshes from the USD primitive and cooks the physx deformable volume mesh.
     *
     * @param outStream : Stream to which the PxDeformableVolumeMesh data and the surface triangle list is written to if
     * available.
     * @param desc : The volume deformable descriptor which defines the usd structure of the deformable.
     * @param bodyKey : Source-side ObjectKey of the prim with UsdPhysicsDeformableAPI
     * @param attachedStage : The attached stage owning the prim.
     * @param asynchronous : Whether or not to cook asynchronously.
     *
     * @return : Returns true if data has been written to outStream.
     */
    virtual bool cookDeformableVolumeMesh(::physx::PxDefaultMemoryOutputStream& outStream,
                                          const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          omni::physics::parse::ObjectKey bodyKey,
                                          const omni::physx::usdparser::AttachedStage& attachedStage,
                                          bool asynchronous) = 0;

    /**
     * Computes the deformable cooking transform.
     *
     * The cooking space is constructed such that
     * - the space is invariant to translation, rotation and uniform scale of the deformable
     * - the provided points in sim space are mapped to the unit cube around the origin
     *
     * @param simToCookingTransform : Pointer to store output sim mesh to cooking space transform or nullptr.
     * @param cookingToWorldTransform : Pointer to store output cooking space to world space transform or nullptr.
     * @param cookingToWorldScale : Pointer to store output cooking to world space scale or nullptr.
     * @param simToWorld : Input sim mesh to world transform.
     * @param boundsFitPoints : Points to compute uniform bounds fit. Expected to be in sim space.
     * @param boundsFitPointCount : Number of points in `boundsFitPoints`.
     *
     * The matrices are the same sixteen doubles a GfMatrix4d holds, in the same
     * flat order (omni.physics.usd/TypeCast.h), so the PxMat44d carries the
     * transpose linear map and Gf's `A * B` is PhysX's `B * A`. Callers that keep
     * Gf's accumulation order must go through omni::physx::gfmath.
     *
     * @return : Returns true if computation was successfull.
     */
    virtual bool computeDeformableCookingTransform(::physx::PxMat44d* simToCookingTransform,
                                                   ::physx::PxMat44d* cookingToWorldTransform,
                                                   double* cookingToWorldScale,
                                                   const ::physx::PxMat44d& simToWorld,
                                                   const carb::Float3* boundsFitPoints,
                                                   size_t boundsFitPointCount) = 0;

    /**
     *  Samples particles on a mesh using poisson sampling
     *
     *  @param primKey : Source-side ObjectKey of the UsdGeomMesh we are sampling
     *  @param attachedStage : The attached stage owning the prim (resolves the key to a path + stage id at cook time)
     *  @param samplingDesc : The particle sampling descriptor which defines properties for this particle sampler
     *  @param forceResampling : Forcing resampling, even if USD data matches USD crc.
     *  @param asynchronous : If false, it will sample the mesh synchronously (blocking). If true, it will start a
     * background cooking task for it.
     */
    virtual void poissonSampleMesh(omni::physics::parse::ObjectKey primKey,
                                   const omni::physx::usdparser::AttachedStage& attachedStage,
                                   const omni::physx::usdparser::ParticleSamplingDesc& samplingDesc,
                                   bool forceResampling,
                                   bool asynchronous) = 0;

    /**
     * Reports the number of active and pending cooking tasks
     *
     * @return : Returns the number of active and pending cooking tasks to be computed asynchronously
     */
    virtual uint32_t getActiveTaskCount(void) = 0;

    /**
     * Attempts to cancel all active and pending cooking tasks. In the current implementation
     * most asynchronous cooking tasks are non-interruptible, but ones which are pending will not
     * be processed and ones in flight will throw away their results once completed.
     *
     * @return :Returns the number of active and pending cooking tasks, marked to be canceled.
     */
    virtual uint32_t cancelAllTasks(void) = 0;

    /**
     * This method is used to indicate to the cooking interface to ignore USD change events.
     * Normally CookingDataAsync listens for collision related attribute changes to know if
     * an asset needs to be recooked due to different property values. However, sometimes
     * property changes are being done by omni.physx itself, rather than because a user
     * made a change via the editor. Calling this method signal to the CookingDataAsnc
     * interface whether it should, or should not, pay attention to notice handler change events.
     *
     * @param blocked : If true then the notice handler should ignore property changes. If false, the default, it will
     * process changes.
     */
    virtual void blockUSDUpdate(bool blocked) = 0;

    /**
     * Returns the total number of cooking tasks which have been performed since the
     * start of the application. This is used by debug visualization (omni.physx.ui) to
     * know whether or not it should refresh the debug visualization of a primitive because
     * the cooking state has changed since the last time.
     */
    virtual uint32_t getFinishedCookingTasksCount(void) const = 0;

    /**
     * Add this primitive to be re-evaluated to see if it needs to be cooked. In the
     * zero gravity mode and some other spots in the code we may have changed some
     * collision properties ourselves while, at the same time, the notice handler was
     * blocked. This allows the user to explicitly add a USD primitive (by path name)
     * to be re-evaluated. What happens next is, during the pump loop, this hash key for
     * this UsdPrim is recomputed and if it has changed (indicating that a new cooking
     * task to deal with the new property values is needed) then an asynchronous cooking
     * task is spawned. This is how the editor can keep recooking assets while people are
     * making changes in the property window.
     *
     * Named by an explicit AttachHandle rather than resolved against "the active attach"
     * (ADR-0016 Decision 4): the caller names the attach `key` was resolved from, and an
     * unresolvable handle is rejected with a diagnostic rather than silently dropping the
     * request.
     *
     * @param key : ObjectKey of the object to be re-evaluated.
     * @param attachHandle : the attach that key was resolved from.
     */
    virtual void addPrimRefreshSet(omni::physics::parse::ObjectKey key, omni::physics::AttachHandle attachHandle) = 0;

    /**
     * Register this cooking driver's change-detection interest on the given attached
     * stage's IChangeFeed (ADR-0003). Called by AttachedStage when it (re)creates its
     * feed, so cooking observes USD edits (recook scheduling) from attach time without
     * a global USD notice listener. The registration is owned by the per-stage feed
     * and is released when that feed is destroyed (detach / source rebuild).
     *
     * @param attachedStage : The attached stage whose change feed to observe.
     */
    virtual void registerOnChangeFeed(omni::physx::usdparser::AttachedStage& attachedStage) = 0;

    /**
     * Returns the solid shaded debug visualization triangle mesh for the cooked collision
     * representation of this asset. In omni.physx.ui debug visualization we now support
     * the ability to visualize as a solid shaded mesh (represented as a UsdGeomMesh instance(s)
     * on the session layer). In this way the user can get a clear visualization of what the
     * collision representation of the source primitive actually looks like.
     * When a collision mesh is cooked a copy of the debug visualization representation is
     * also stored into the localcache so that it can be easily retrieved here.
     * Since a single UsdPrim can have (n) number of collision meshes (in the case of a
     * convex decomposition) it is possible to get more than just one for the source UsdPrim.
     *
     * @param key : The ObjectKey of the primitive we are referring to
     * @param desc : The shape descriptor for this UsdPrim
     *
     * @return : If a graphics collision representation exists, it will return a pointer to it.
     */
    virtual const omni::physx::CollisionRepresentation* getCollisionRepresentation(
        omni::physics::parse::ObjectKey key, const omni::physx::usdparser::PhysxShapeDesc& desc) = 0;

    /**
     * Release a previously queried collision representation.
     *
     * @param cr : A pointer to a previously retrieved collision representation. Each call to
     * 'getCollisionRepresentation' should be paired with a call to 'releaseCollisionRepresentation' or you will get a
     * memory leak.
     */
    virtual void releaseCollisionRepresentation(const omni::physx::CollisionRepresentation* cr) = 0;

    /**
     * Retrieves the unique 128 hash associated with this descriptor.  Takes into account the
     * cooking data version number as well as the signed scale. This method is used by
     * the solid shaded debug visualization to know when it needs to change the debug
     * visualization primitives on property change events.
     *
     * @param desc : The shape descriptor for this primitive.
     * @param meshKey : A reference to return the 128 bit unique hash key corresponding to the cooked data.
     *
     * @return : Returns true if we could generate a hash key from this shape descriptor (only applies to convex hull,
     * triangle mesh, and convex decomposition) other shape types are ignored as they don't have support for the solid
     * shaded debug visualization.
     */
    virtual bool getMeshkey(const omni::physx::usdparser::PhysxShapeDesc& desc,
                            omni::physx::usdparser::MeshKey& meshKey) = 0;

    /**
     * This method releases the instance of the CookingDataAsync class.
     */
    virtual void release(void) = 0;

    virtual void resetLocalMeshCacheContents() = 0;

    virtual omni::physx::PhysxCookingAsyncContext getCookingAsyncContext() = 0;

    virtual omni::physx::PhysxCookingStatistics getCookingStatistics() const = 0;
protected:
    /**
     * The class destructor if declared virtual to prevent anyone from trying to delete
     * this class using the 'delete' keyword. To release the class instance, call 'release'
     */
    virtual ~CookingDataAsync()
    {
    }
};

/**
 * Creates an instance of the 'CookingDataAsync' interface class
 *
 * @return : Returns a pointer to the CookingDataAsync interface class if successful.
 */
CookingDataAsync* createCookingDataAsync(physx::PxPhysics& physics,
                                         omni::physx::IPhysxCookingServicePrivate& cookingServicePrivate,
                                         omni::physx::IPhysxCookingService& cookingService,
                                         omni::physx::PhysxCookingAsyncContext context);
} // namespace cookingdataasync
