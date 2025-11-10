// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"
#include <vector>
#include <carb/Types.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>

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
struct ConvexMeshPhysxShapeDesc;
struct ConvexMeshDecompositionPhysxShapeDesc;
struct PhysxShapeDesc;
struct TriangleMeshPhysxShapeDesc;
struct SoftBodyDesc;
class MeshKey;
struct ParticleSamplingDesc;
struct SpherePointsPhysxShapeDesc;
struct PhysxDeformableBodyDesc;
struct PhysxVolumeDeformableBodyDesc;
struct PhysxSurfaceDeformableBodyDesc;

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
     * @param usdPrim : The UsdPrim associated with this convex mesh
     * @param asynchronous : If false the convex mesh will be cooked synchronously (blocking) in this thread. If true,
     * it will spawn a background task if necessary.
     *
     * @return : Returns the pointer to the PxConvexMesh if it was available at this time.
     */
    virtual ::physx::PxConvexMesh* getConvexMesh(const omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc,
                                                 pxr::UsdPrim& usdPrim,
                                                 bool asynchronous,
                                                 omni::physx::IPhysxCookingCallback* cb = nullptr) = 0;

    virtual const omni::physx::usdparser::SpherePointsPhysxShapeDesc* getSpherePoints(
        const omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc,
        pxr::UsdPrim& usdPrim,
        bool asynchronous,
        omni::physx::IPhysxCookingCallback* cb = nullptr) = 0;

    /**
     * This method returns the PhysX PxTriangleMesh associated with this USD prim if available.
     * If it is an asynchronous request and the mesh needs to be cooked, it will spawn a cooking task
     * to compute the result.
     *
     * @param desc : The triangle mesh shape descriptor which defines the properties to apply when creating the triangle
     * mesh approximation
     * @param usdPrim : The UsdPrim associated with this triangle mesh
     * @param asynchronous : If false the triangle mesh will be cooked synchronously (blocking) in this thread. If true,
     * it will spawn a background task if necessary.
     * @param maxMaterialIndex: Returns the highest index of the material used in the triangle mesh
     *
     * @return : Returns the pointer to the PxTriangleMesh if it was available at this time.
     */
    virtual ::physx::PxTriangleMesh* getTriangleMesh(const omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc,
                                                     pxr::UsdPrim& usdPrim,
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
     * @param usdPrim : The UsdPrim associated with this convex decomposition
     * @param asynchronous : If false the convex decomposition will be cooked synchronously (blocking) in this thread.
     * If true, it will spawn a background task if necessary.
     *
     * @return : Returns an std::vector of PxConvexMeshes if it was available at this time.
     */
    virtual std::vector<::physx::PxConvexMesh*> getConvexMeshDecomposition(
        const omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc,
        pxr::UsdPrim& usdPrim,
        bool asynchronous,
        omni::physx::IPhysxCookingCallback* cb = nullptr) = 0;

    /**
     * Performs the cooking operation on a deformable tetrahedral mesh associated with a particular UsdGeomMesh
     * primitive This operation generates collision and simulation tetrahedral meshes from the UsdGeomMesh primitive and
     * stores the result back to USD.
     *
     * @param usdPrim : The UsdGeomMesh we are cooking
     * @param asynchronous : If false, it will cook the tetrahedral mesh synchronously (blocking). If true, it will
     * start a background cooking task for it.
     */
    virtual void cookDeformableBodyTetMeshDeprecated(pxr::UsdPrim& usdPrim, bool asynchronous) = 0;

    /**
     * Cooks a PxSoftBodyMesh, creates a surface triangle list and returns the result in a stream.
     * This operation reads the tetrahedral meshes from the USD primitive and cooks the physx softbody mesh.
     *
     * @param outStream : Stream to which the PxSoftBodyMesh data and the surface triangle list is written to if
     * available.
     * @param softBodyDesc : The soft body shape descriptor which defines properties for this mesh.
     * @param usdPrim : The UsdGeomMesh associated with this soft body
     * @param asynchronous : Whether or not to cooking the soft body mesh (if needed) asynchronously.
     *
     * @return : Returns true if data has been written to outStream.
     */
    virtual bool cookSoftBodyMeshDeprecated(::physx::PxDefaultMemoryOutputStream& outStream,
                                            const omni::physx::usdparser::SoftBodyDesc& softBodyDesc,
                                            pxr::UsdPrim& usdPrim,
                                            bool asynchronous) = 0;

    /**
     * Helper method, that creates a PxSoftBodyMesh from the input data stream and parses the surface triangle list.
     *
     * @param outCollMeshSurfaceTriangles : output collision mesh surface triangles
     * @param outNumCollMeshSurfaceVertices : output collision mesh surface vertices count
     * @param inData : input data (@see cookSoftBodyMesh)
     *
     * @return : Returns the PxSoftBodyMesh instance. The caller of the method is responsible for releasing the
     * PxSoftBodyMesh.
     */
    virtual ::physx::PxSoftBodyMesh* createSoftBodyMeshDeprecated(std::vector<carb::Uint3>& outCollMeshSurfaceTriangles,
                                                                  ::physx::PxDefaultMemoryInputData& inData) = 0;

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
     * @param bodyPrim : The root prim to parse.
     * @return : Deformable body descriptor.
     */
    virtual omni::physx::usdparser::PhysxDeformableBodyDesc* parseDeformableBody(pxr::UsdPrim bodyPrim) = 0;

    virtual void cookVolumeDeformableBody(const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          pxr::UsdPrim bodyPrim,
                                          bool asynchronous) = 0;

    virtual void cookSurfaceDeformableBody(const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc& desc,
                                           pxr::UsdPrim bodyPrim,
                                           bool asynchronous) = 0;

    /**
     * Cooks a PxDeformableVolumeMesh, creates a surface triangle list and returns the result in a stream.
     * This operation reads the tetrahedral meshes from the USD primitive and cooks the physx deformable volume mesh.
     *
     * @param outStream : Stream to which the PxDeformableVolumeMesh data and the surface triangle list is written to if
     * available.
     * @param desc : The volume deformable descriptor which defines the usd structure of the deformable.
     * @param bodyPrim : The UsdPrim with UsdPhysicsDeformableAPI
     * @param asynchronous : Whether or not to cook asynchronously.
     *
     * @return : Returns true if data has been written to outStream.
     */
    virtual bool cookDeformableVolumeMesh(::physx::PxDefaultMemoryOutputStream& outStream,
                                          const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc& desc,
                                          pxr::UsdPrim bodyPrim,
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
     *
     * @return : Returns true if computation was successfull.
     */
    virtual bool computeDeformableCookingTransform(pxr::GfMatrix4d* simToCookingTransform,
                                                   pxr::GfMatrix4d* cookingToWorldTransform,
                                                   double* cookingToWorldScale,
                                                   const pxr::GfMatrix4d& simToWorld,
                                                   const pxr::VtArray<pxr::GfVec3f>& boundsFitPoints) = 0;

    /**
     *  Samples particles on a mesh using poisson sampling
     *
     *  @param usdPrim : The UsdGeomMesh we are sampling
     *  @param samplingDesc : The particle sampling descriptor which defines properties for this particle sampler
     *  @param forceResampling : Forcing resampling, even if USD data matches USD crc.
     *  @param asynchronous : If false, it will sample the mesh synchronously (blocking). If true, it will start a
     * background cooking task for it.
     */
    virtual void poissonSampleMesh(pxr::UsdPrim& usdPrim,
                                   const omni::physx::usdparser::ParticleSamplingDesc& samplingDesc,
                                   bool forceResampling, 
                                   bool asynchronous) = 0;

    /**
     * Performs the cooking operation on a particle cloth mesh associated with a particular UsdGeomMesh primitive
     * This operation generates spring data and inflatable params and stores the result back to USD.
     *
     * @param usdPrim : The UsdGeomMesh we are cooking
     * @param asynchronous : If false, it will cook the tetrahedral mesh synchronously (blocking). If true, it will
     * start a background cooking task for it.
     */
    virtual void cookParticleClothDeprecated(pxr::UsdPrim& usdPrim, bool asynchronous) = 0;

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
     * @param path : The UsdPrim path to be re-evaluated.
     */
    virtual void addPrimRefreshSet(const pxr::SdfPath& path) = 0;

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
     * @param path : The path of the primitive we are referring to
     * @param desc : The shape descriptor for this UsdPrim
     *
     * @return : If a graphics collision representation exists, it will return a pointer to it.
     */
    virtual const omni::physx::CollisionRepresentation* getCollisionRepresentation(
        const pxr::SdfPath& path, const omni::physx::usdparser::PhysxShapeDesc& desc) = 0;

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

    virtual bool isLocalMeshCacheEnabled() const = 0;

    virtual void setLocalMeshCacheEnabled(bool val) = 0;

    // local mesh cache size in MB
    virtual uint32_t getLocalMeshCacheSize() const = 0;

    // local mesh cache size in MB
    virtual void setLocalMeshCacheSize(uint32_t val) = 0;

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
