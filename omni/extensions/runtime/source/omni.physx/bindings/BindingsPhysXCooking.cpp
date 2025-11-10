// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>

#include <common/utilities/Utilities.h>
#include <carb/BindingsPythonUtils.h>

#include "ResultBuffer.h"

namespace omni
{
namespace physx
{

struct OwnedConvexMeshData
{
    std::vector<carb::Float3> vertices;
    std::vector<uint8_t> indices;
    std::vector<ConvexMeshPolygon> polygons;
};
} // namespace physx
} // namespace omni

void PhysXCookingBindings(pybind11::module& m)
{
    using namespace pybind11;
    using namespace carb;
    using namespace omni::physx;
    const char* docString;

    auto dev = defineInterfaceClass<IPhysxCooking>(
        m, "PhysXCooking", "acquire_physx_cooking_interface", "release_physx_cooking_interface");
    dev.doc() = R"(
        This interface is the access point to the omni.physx extension cooking API.
    )";

    docString = R"(
        Create a conforming tetrahedral mesh from a closed source triangle mesh.

        Args:
            src_tri_points      Vertices of the source triangle mesh
            src_tri_indices     Vertex indices of the source triangles

        Returns:
            Dict mapping 'points' and 'indices' to the resulting tetrahedral mesh points and indices

        The conforming tetrahedral mesh is defined as a tetrahedral mesh whose surface triangles align with the closed source triangle mesh and whose
        internal vertices lie on the inside of the closed source triangle mesh.
    )";
    dev.def(
        "compute_conforming_tetrahedral_mesh",
        [](const IPhysxCooking* self, const std::vector<Float3>& srcTriPoints, const std::vector<uint32_t>& srcTriIndices) {
            ResultBuffer<carb::Float3> dstTetPoints;
            ResultBuffer<uint32_t> dstTetIndices;

            self->computeConformingTetrahedralMesh(dstTetPoints.ptr, dstTetPoints.size, dstTetIndices.ptr,
                                                   dstTetIndices.size, &srcTriPoints[0], uint32_t(srcTriPoints.size()),
                                                   &srcTriIndices[0], uint32_t(srcTriIndices.size()),
                                                   ResultBuffer<>::allocate);

            py::gil_scoped_acquire gil;
            py::dict out;
            out["points"] = static_cast<py::list>(dstTetPoints);
            out["indices"] = static_cast<py::list>(dstTetIndices);
            return out;
        },
        docString, py::arg("src_tri_points"), py::arg("src_tri_indices"), py::call_guard<py::gil_scoped_release>());

    docString = R"(
        Create a voxel tetrahedral mesh from a source tetrahedral mesh.

        Args:
            src_tet_points:     Vertices of teh source tetrahedral mesh
            src_tet_indices:    Vertex indices of the source tetrahedral mesh
            src_scale:          Scale of source mesh used to determine resolution of the resulting voxel mesh
            voxel_resolution:   Number of voxels along longest dimension of axis aligned bounding box of source mesh

        Returns:
            Dict mapping 'points' and 'indices' to the resulting tetrahedral mesh points and indices

        The voxel tetrahedral mesh is made by voxelizing the source tetrahedra on a regular grid. The
        resulting voxel tetrahedral mesh embeds all tetrahedra of the source mesh.
        The provided voxel resolution may be lowered automatically in order to match a lower resolution detected in
        the source mesh. This may help to avoid softbody convergence issues with high-resolution tetrahedra
        embedding low resolution collision meshes.
    )";
    dev.def(
        "compute_voxel_tetrahedral_mesh",
        [](const IPhysxCooking* self, const std::vector<Float3>& srcTetPoints,
            const std::vector<uint32_t>& srcTetIndices, const Float3& srcScale, const int voxelResolution) {
                ResultBuffer<carb::Float3> dstTetPoints;
                ResultBuffer<uint32_t> dstTetIndices;
                ResultBuffer<int32_t> dstEmbedding;

                self->computeVoxelTetrahedralMesh(
                    dstTetPoints.ptr, dstTetPoints.size, dstTetIndices.ptr, dstTetIndices.size, dstEmbedding.ptr,
                    dstEmbedding.size, &srcTetPoints[0], uint32_t(srcTetPoints.size()), &srcTetIndices[0],
                    uint32_t(srcTetIndices.size()), srcScale, voxelResolution, ResultBuffer<>::allocate);

                py::gil_scoped_acquire gil;
                py::dict out;
                out["points"] = static_cast<py::list>(dstTetPoints);
                out["indices"] = static_cast<py::list>(dstTetIndices);
                out["embedding"] = static_cast<py::list>(dstEmbedding);
                return out;
        },
        docString, py::arg("src_tet_points"), py::arg("src_tet_indices"), py::arg("src_scale"),
            py::arg("voxel_resolution"), py::call_guard<py::gil_scoped_release>());

    docString = R"(
    Cooks deformable body mesh

    Args:
        stage_id: Stage containing source mesh primitive.
        deformable_body_path: path to UsdGeomMesh with PhysxSchemaPhysxDeformableBodyAPI
    )";
    dev.def(
        "cook_deformable_body_mesh",
        [](const IPhysxCooking* self, const char* deformableBodyPath) {
            return self->cookDeformableBodyMesh(pxr::SdfPath(deformableBodyPath));
        },
        docString, py::arg("deformable_body_path"), py::call_guard<py::gil_scoped_release>()); // OM-75604

    docString = R"(
    Cooks deformable body mesh

    Args:
        stage_id: Stage containing source mesh primitive.
        deformable_body_path: path to prim with UsdPhysicsDeformableBodyAPI
    )";
    dev.def(
        "cook_auto_deformable_body",
        [](const IPhysxCooking* self, const char* deformableBodyPath) {
            return self->cookAutoDeformableBody(pxr::SdfPath(deformableBodyPath));
        },
        docString, py::arg("deformable_body_path"), py::call_guard<py::gil_scoped_release>());

    docString = R"(
        Release Local Mesh Cache.
    )";
    dev.def("release_local_mesh_cache", wrapInterfaceFunction(&IPhysxCooking::releaseLocalMeshCache), docString,
            py::call_guard<py::gil_scoped_release>());
   
    py::enum_<PhysxCollisionRepresentationResult::Enum>(
        m, "PhysxCollisionRepresentationResult", "Collision representation result")
        .value("RESULT_VALID", PhysxCollisionRepresentationResult::eRESULT_VALID)
        .value("RESULT_ERROR_NOT_READY", PhysxCollisionRepresentationResult::eRESULT_ERROR_NOT_READY)
        .value("RESULT_ERROR_INVALID_PARSING", PhysxCollisionRepresentationResult::eRESULT_ERROR_INVALID_PARSING)
        .value("RESULT_ERROR_COOKING_FAILED", PhysxCollisionRepresentationResult::eRESULT_ERROR_COOKING_FAILED)
        .value("RESULT_ERROR_UNSUPPORTED_APPROXIMATION",
               PhysxCollisionRepresentationResult::eRESULT_ERROR_UNSUPPORTED_APPROXIMATION)
        .value("RESULT_ERROR_INVALID_RESULT", PhysxCollisionRepresentationResult::eRESULT_ERROR_INVALID_RESULT);

    py::class_<PhysxCollisionRepresentationTask>(
        m, "PhysxCollisionRepresentationTask", R"(Task returned by request_convex_collision_representation)")
        .def_readonly("task", &PhysxCollisionRepresentationTask::handle, "Task handle");

    py::class_<ConvexMeshPolygon>(m, "PhysxConvexMeshPolygon", "A polygon of a convex mesh")
        .def_property_readonly(
            "plane",
            [](const ConvexMeshPolygon* polygon) {
                return carb::Float4{ polygon->plane[0], polygon->plane[1], polygon->plane[2], polygon->plane[3] };
            })
        .def_readonly("num_vertices", &ConvexMeshPolygon::numVerts)
        .def_readonly("index_base", &ConvexMeshPolygon::indexBase);
    py::class_<OwnedConvexMeshData>(m, "PhysxConvexMeshData", "A convex mesh made of vertices, indices and polygons")
        .def_readonly("vertices", &OwnedConvexMeshData::vertices)
        .def_readonly("indices", &OwnedConvexMeshData::indices)
        .def_readonly("polygons", &OwnedConvexMeshData::polygons);

    docString =
        R"(
        Cancels an async physics collision representation task made with request_convex_collision_representation
        
        Args:
            task:            PhysxCollisionRepresentationTask The task returned by request_convex_collision_representation to cancel
            invoke_callback: bool                             If the on_result callback should be invoked anyway
        )";
    dev.def(
        "cancel_collision_representation_task",
        [](const IPhysxCooking* self, PhysxCollisionRepresentationTask task, bool invokeCallback) {
            self->cancelCollisionRepresentationTask(task, invokeCallback);
        },
        docString,
        py::arg("task"), // Mandatory
        py::arg("invoke_callback") = true, // Optional
        py::call_guard<py::gil_scoped_release>());

    docString = R"(
    Request physics collision representation of an USD mesh

    Args:
        stage_id:           uint64_t    Stage where prim exists
        collision_prim_id:  uint64_t    Prim Id of the prim with CollisionAPI
        run_asynchronously: bool        If the request should be run asynchronously
        on_result:          Callable[PhysxCollisionRepresentationResult, list[PhysxConvexMeshData])  
                The callback with the wanted collision representation.
                result: Value of type PhysxCollisionRepresentationResult
                convexes: list of PhysxConvexMeshData

    Returns: A PhysxCollisionRepresentationTask that can be cancelled with cancel_collision_representation_task 
             (if async request has been made)
)";
    dev.def(
        "request_convex_collision_representation",
        [](const IPhysxCooking* self, uint64_t stage_id, uint64_t collision_prim_id, bool run_asynchronously,
           std::function<void(PhysxCollisionRepresentationResult::Enum result, std::vector<OwnedConvexMeshData> convexResult)>
               on_result) -> PhysxCollisionRepresentationTask {
            PhysxCollisionRepresentationRequest request;
            request.stageId = stage_id;
            request.collisionPrimId = collision_prim_id;
            request.options.setFlag(
                PhysxCollisionRepresentationRequest::Options::kComputeAsynchronously, run_asynchronously);
            auto wrapped_fun = carb::wrapPythonCallback(std::move(on_result));
            return self->requestConvexCollisionRepresentation(
                request, [result_fn = move(wrapped_fun)](PhysxCollisionRepresentationResult::Enum result,
                                                         const PhysxCollisionRepresentationConvexResult& convexResult) {
                    py::gil_scoped_acquire gil;

                    // We must dump all convexResult into something that can be owned by python, as it will not be valid
                    // after the ending scope of this callback
                    std::vector<OwnedConvexMeshData> ownedResults;
                    ownedResults.reserve(convexResult.convexes.size());
                    for (const ConvexMeshData& convex : convexResult.convexes)
                    {
                        OwnedConvexMeshData meshData;
                        meshData.vertices.insert(
                            meshData.vertices.begin(), convex.vertices, convex.vertices + convex.numVertices);
                        meshData.indices.insert(
                            meshData.indices.begin(), convex.indices, convex.indices + convex.numIndices);
                        meshData.polygons.insert(
                            meshData.polygons.begin(), convex.polygons, convex.polygons + convex.numPolygons);
                        ownedResults.push_back(std::move(meshData));
                    }
                    result_fn(result, std::move(ownedResults));
                });
        },
        docString, // Documentation
        py::arg("stage_id"), // Mandatory
        py::arg("collision_prim_id"), // Mandatory
        py::arg("run_asynchronously") = true, // Optional
        py::arg("on_result"), // Mandatory
        py::call_guard<py::gil_scoped_release>());
}

void PhysXCookingPrivateBindings(pybind11::module& m)
{
    using namespace pybind11;
    using namespace carb;
    using namespace omni::physx;
    const char* docString;

    auto dev = defineInterfaceClass<IPhysxCookingPrivate>(
        m, "PhysXCookingPrivate", "acquire_physx_cooking_private_interface", "release_physx_cooking_private_interface");

    dev.doc() = R"(
        This interface is the access point to the omni.physx extension private cooking API.
    )";

    py::class_<PhysxCookingStatistics>(m, "PhysxCookingStatistics", "A convex mesh made of vertices, indices and polygons")
        .def_readonly("total_finished_tasks", &PhysxCookingStatistics::totalFinishedTasks)
        .def_readonly("total_scheduled_tasks", &PhysxCookingStatistics::totalScheduledTasks)
        .def_readonly("total_finished_cache_miss_tasks", &PhysxCookingStatistics::totalFinishedCacheMissTasks)
        .def_readonly("total_finished_cache_hit_tasks", &PhysxCookingStatistics::totalFinishedCacheHitTasks)
        .def_readonly("total_warnings_failed_gpu_compatibility", &PhysxCookingStatistics::totalWarningsFailedGPUCompatibility)
        .def_readonly("total_warnings_convex_polygon_limits_reached", &PhysxCookingStatistics::totalWarningsConvexPolygonLimitsReached)
        ;

    docString = R"(
        Obtains cooking statistics.

        Returns:
            PhysxCookingStatistics structure with all relevant fields

        Cooking statistics allow knowing if a cooking task was scheduled and executed.
        As of today this is mainly needed for internal tests.
    )";
    dev.def("get_cooking_statistics", wrapInterfaceFunction(&IPhysxCookingPrivate::getCookingStatistics), docString, py::call_guard<py::gil_scoped_release>());

    docString = R"(
    Adds Prim to cooking refresh set.

    Args:
        primPath: path to Prim
    )";
    dev.def("add_prim_to_cooking_refresh_set", wrapInterfaceFunction(&IPhysxCookingPrivate::addPrimToCookingRefreshSet),
            docString, py::call_guard<py::gil_scoped_release>());

    docString = R"(
    Release Runtime Mesh Cache.
    )";
    dev.def("release_runtime_mesh_cache", wrapInterfaceFunction(&IPhysxCookingPrivate::releaseRuntimeMeshCache),
            docString, py::call_guard<py::gil_scoped_release>());

}
