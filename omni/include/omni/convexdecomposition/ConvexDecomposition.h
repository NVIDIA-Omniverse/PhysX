// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once


#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace convexdecomposition
{

// Rather than using pointers we use a handle for each V-HACD instance
using VHACDHANDLE = uint64_t;

typedef void(CARB_ABI* NotifyVHACDComplete)(omni::convexdecomposition::VHACDHANDLE id, void* userPtr);

enum class VoxelFillMode : uint32_t
{
    eFloodFill, // This is the default behavior, after the voxelization step it uses a flood fill to determine 'inside'
                // from 'outside'. However, meshes with holes can fail and create hollow results.
    eSurfaceOnly, // Only consider the 'surface', will create 'skins' with hollow centers.
    eRaycastFill // Uses raycasting to determine inside from outside.
};

struct Parameters
{
    double errorPercentage{ 10.0 }; // Percentage error allowed. Default is 10%. If the convex hull is within 10% of the
                                    // volume of the voxels, we treat it as sufficiently convex.
    uint32_t maxHullVertices{ 32 }; // Maximum number of vertices in output convex hull
    uint32_t maxConvexHullCount{ 32 }; // Maximum number of convex hulls to produce.
    uint32_t voxelResolution{ 500000 }; // Approximate number of voxels to use, default value is one million.
    VoxelFillMode voxelFillMode{ VoxelFillMode::eFloodFill }; // Fill mode type to use
    bool shrinkWrap{ true }; // experimental feature to shrinkwrap the convex hulls to the surface of the source mesh

    uint32_t maxSpheres{ 64 }; // if it's a sphere approximation, this is the maximum number of spheres allowed
    uint32_t maxSeedCount{ 1024 }; // maximum number of seed points to consider for a sphere approximation

    NotifyVHACDComplete notifyCompleteCallback{ nullptr }; // Optional completion notification callback, warning comes
                                                           // from another thread.
    void* notifyCompleteCallbackUserPtr{ nullptr }; // User pointer to return when invoking the callback function
};

struct PolygonData
{
    uint32_t startIndex{ 0 }; // starting index in the indices buffer for this polygon
    uint32_t pointCount{ 0 }; // the number of points in this polygon.
    double planeEquation[4]{}; // The plane equation for this polygon
};

struct SimpleMesh
{
    uint32_t vertexCount{ 0 }; // number of vertices in the mesh
    uint32_t triangleCount{ 0 }; // number of triangles in the mesh.
    const double* vertices{ nullptr }; // Vertex positions as doubles for maximum precision when performing geometric
                                       // processing. x1,y1,z1,x2,y2,z2,...
    const uint32_t* indices{ nullptr }; // triangle indices.

    uint32_t polygonCount{ 0 }; // number of polygons in the output convex hull
    const PolygonData* polygons{ nullptr }; // Defines each polygon
    uint32_t polygonIndexCount{ 0 }; // number of polygon indices total
    const uint32_t* polygonIndices{ nullptr }; // indices for each polygon

    double volume{ 0 }; // optional volume of the mesh
    carb::Double3 center{}; // optional center of mesh
    carb::Double3 rootCenter{}; // center of the original mesh
};

struct SimpleSphere
{
    carb::Double3 center{}; // optional center of mesh
    double radius;
};

struct ConvexDecomposition
{
    CARB_PLUGIN_INTERFACE("omni::convexdecomposition::ConvexDecomposition", 2, 0)

    /**
     * Creates an instance of the V-HACD system.
     *
     * @return the unique handle/id for this instance.
     */
    VHACDHANDLE(CARB_ABI* createVHACD)(void);

    /**
     * Releases a previously created V-HACD instance. Returns true if the provided handle
     * was valid and/or the instance is not currently running a background task.
     *
     * @param id The handle of a previously allocated V-HACD instance
     * @return Returns true if the V-HACD instance was safely released.
     */
    bool(CARB_ABI* releaseVHACD)(VHACDHANDLE id);

    /**
     * Begins the convex decomposition process.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param p  : A pointer to the parameters to use for this operation.
     * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
     * @return : Returns true if the input parameters were valid and we have started the operation.
     */
    bool(CARB_ABI* beginVHACD)(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh);

    /**
     * Begins the sphere approximation process.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param p  : A pointer to the parameters to use for this operation.
     * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
     * @return : Returns true if the input parameters were valid and we have started the operation.
     */
    bool(CARB_ABI* beginSphereApproximation)(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh);

    /**
     * Returns true if the convex decomposition operation is completed.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * *return : Returns true if the convex decomposition operation is not currently running.
     */
    bool(CARB_ABI* isComplete)(VHACDHANDLE id);

    /**
     * Returns the number of spheres approximation results
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param sphereCount : A reference that will be assigned the number of spheres in the results
     * @param reducedSpheres : If true, it will return the reduced sphere results. If false, it will return all spheres
     * produced.
     * @return : Returns a pointer to an array of simple spheres with the results
     */
    const SimpleSphere*(CARB_ABI* getSphereApproximation)(VHACDHANDLE id, uint32_t& sphereCount, bool reducedSpheres);

    /**
     * Returns the number of convex hulls generated by the convex decomposition process.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @return : Returns the number of convex hulls generated
     */
    uint32_t(CARB_ABI* getConvexHullCount)(VHACDHANDLE id);

    /**
     * Retrieves the triangle mesh corresponding to the convex hull results.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param hullIndex : Which convex hull we are asking for.
     * @param hullResults : A pointer to a struct which will reflect the convex hull mesh data
     * @return : Returns true if the convex hull was successfully retrieved, false if the handle or index was invalid.
     */
    bool(CARB_ABI* getConvexHull)(VHACDHANDLE id, uint32_t hullIndex, SimpleMesh& hullResults);

    /**
     * Cancels any convex decomposition that is in process.
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @return : Returns true if the handle was valid and the operation was successfully canceled.
     */
    bool(CARB_ABI* cancelVHACD)(VHACDHANDLE id);

    /**
     * A method to save a 'SimpleMesh' as a wavefront OBJ file on disk. Primarily used for debugging inputs and outputs.
     *
     * @param fname : Name of file on disk to save the mesh to
     * @param sm : Pointer to a simple triangle mesh
     * @param flipWindingOrder : Whether or not to flip the winding order of the triangles when saving the Wavefront OBJ
     * @return :Returns true if the wavefront OBJ file was successfully saved, false if it was not.
     */
    bool(CARB_ABI* saveOBJ)(const char* fname, const SimpleMesh& sm, bool flipWindingOrder);

    /**
     * Run the convex decomposition synchronously. Warning, this operation can take a long time and cannot be
     * interrupted
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param p  : A pointer to the parameters to use for this operation.
     * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
     * @return : Returns the number of convex hulls produced.
     */
    uint32_t(CARB_ABI* computeVHACD)(VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh);

    /**
     * Run the sphere approximation synchronously. Warning, this operation can take a long time and cannot be
     * interrupted
     *
     * @param id : The handle of a previously allocated V-HACD instance
     * @param p  : A pointer to the parameters to use for this operation.
     * @param sourceMesh : A pointer to the source raw triangle mesh to perform the operation on.
     * @param sphereCount : A reference that will be assigned to the number of spheres in the result
     * @param reducedResults : If true it will return the reduced results. Otherwise it returns all spheres output.
     * @return : Returns a pointer to the sphere approximation results.
     */
    const SimpleSphere*(CARB_ABI* computeSphereApproximation)(
        VHACDHANDLE id, const Parameters& p, const SimpleMesh& sourceMesh, uint32_t& sphereCount, bool reducedResults);

    bool(CARB_ABI* applySphereApproximation)(const char* primPath, uint32_t stageId);
};


} // namespace convexdecomposition
} // namespace omni
