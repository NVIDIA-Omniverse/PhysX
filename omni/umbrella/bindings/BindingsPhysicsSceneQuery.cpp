// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include <carb/Framework.h>
#include <carb/BindingsPythonUtils.h>
#include <omni/physics/simulation/IPhysicsSceneQuery.h>
#include <pybind11/functional.h>

namespace
{
void bindPhysicsSceneQuery(py::module& m)
{
    using namespace carb;
    using namespace omni::physics;

    // Define hit report types
    py::class_<SceneQueryHitObject>(m, "SceneQueryHitObject")
        .def(py::init<>())
        .def_readwrite("collision", &SceneQueryHitObject::collision)
        .def_readwrite("rigid_body", &SceneQueryHitObject::rigidBody)
        .def_readwrite("proto_index", &SceneQueryHitObject::protoIndex);

    py::class_<SceneQueryHitLocation, SceneQueryHitObject>(m, "SceneQueryHitLocation")
        .def(py::init<>())
        .def_readwrite("normal", &SceneQueryHitLocation::normal)
        .def_readwrite("position", &SceneQueryHitLocation::position)
        .def_readwrite("distance", &SceneQueryHitLocation::distance)
        .def_readwrite("face_index", &SceneQueryHitLocation::faceIndex)
        .def_readwrite("material", &SceneQueryHitLocation::material);

    py::class_<OverlapHit, SceneQueryHitObject>(m, "OverlapHit")
        .def(py::init<>());

    py::class_<RaycastHit, SceneQueryHitLocation>(m, "RaycastHit")
        .def(py::init<>());

    py::class_<SweepHit, SceneQueryHitLocation>(m, "SweepHit")
        .def(py::init<>());

    // IPhysicsSceneQuery interface
    defineInterfaceClass<IPhysicsSceneQuery>(m, "IPhysicsSceneQuery", "acquire_physics_scene_query_interface", "release_physics_scene_query_interface")
        .def("raycast_closest", [](IPhysicsSceneQuery& self, const Float3& origin, const Float3& unitDir, float distance, bool bothSides) {
            RaycastHit hit;
            bool result = self.raycastClosest(origin, unitDir, distance, hit, bothSides);
            return std::make_tuple(result, hit);
        }, py::arg("origin"), py::arg("unit_dir"), py::arg("distance"), py::arg("both_sides"),
           "Raycast physics scene, return the closest hit found.\n\n"
           "Args:\n"
           "    origin: Origin of the ray\n"
           "    unit_dir: Normalized direction of the ray\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    both_sides: If mesh both triangle sides should be checked\n\n"
           "Returns:\n"
           "    Tuple of (bool, RaycastHit): True if hit was found and the hit information")
        .def("raycast_any", [](IPhysicsSceneQuery& self, const Float3& origin, const Float3& unitDir, float distance, bool bothSides) {
            return self.raycastAny(origin, unitDir, distance, bothSides);
        }, py::arg("origin"), py::arg("unit_dir"), py::arg("distance"), py::arg("both_sides"),
           "Raycast any physics scene, returns only boolean if hit was found or not.\n\n"
           "Args:\n"
           "    origin: Origin of the ray\n"
           "    unit_dir: Normalized direction of the ray\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    both_sides: If mesh both triangle sides should be checked\n\n"
           "Returns:\n"
           "    bool: True if hit was found")
        .def("raycast_all", [](IPhysicsSceneQuery& self, const Float3& origin, const Float3& unitDir, float distance, py::function reportFn, bool bothSides) {
            self.raycastAll(origin, unitDir, distance, [reportFn](const RaycastHit& hit) {
                py::gil_scoped_acquire gil;
                return reportFn(hit).cast<bool>();
            }, bothSides);
        }, py::arg("origin"), py::arg("unit_dir"), py::arg("distance"), py::arg("report_fn"), py::arg("both_sides"),
           "Raycast physics scene, returns all hits found in a raycast callback.\n\n"
           "Args:\n"
           "    origin: Origin of the ray\n"
           "    unit_dir: Normalized direction of the ray\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    report_fn: Scene query hit report function, return True to continue traversal, False to stop traversal\n"
           "    both_sides: If mesh both triangle sides should be checked")
        .def("sweep_sphere_closest", [](IPhysicsSceneQuery& self, float radius, const Float3& origin, const Float3& unitDir, float distance, bool bothSides) {
            SweepHit hit;
            bool result = self.sweepSphereClosest(radius, origin, unitDir, distance, hit, bothSides);
            return std::make_tuple(result, hit);
        }, py::arg("radius"), py::arg("origin"), py::arg("unit_dir"), py::arg("distance"), py::arg("both_sides"),
           "Sweep test of a sphere against all objects in the physics scene, returning the closest hit found.\n\n"
           "Args:\n"
           "    radius: Sphere radius\n"
           "    origin: Origin of the sweep\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    both_sides: If mesh both triangle sides should be checked\n\n"
           "Returns:\n"
           "    Tuple of (bool, SweepHit): True if hit was found and the hit information")
        .def("sweep_sphere_any", [](IPhysicsSceneQuery& self, float radius, const Float3& origin, const Float3& unitDir, float distance, bool bothSides) {
            return self.sweepSphereAny(radius, origin, unitDir, distance, bothSides);
        }, py::arg("radius"), py::arg("origin"), py::arg("unit_dir"), py::arg("distance"), py::arg("both_sides"),
           "Sweep test of a sphere against all objects in the physics scene, returning whether any hit was found.\n\n"
           "Args:\n"
           "    radius: Sphere radius\n"
           "    origin: Origin of the sweep\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    both_sides: If mesh both triangle sides should be checked\n\n"
           "Returns:\n"
           "    bool: True if hit was found")
        .def("sweep_sphere_all", [](IPhysicsSceneQuery& self, float radius, const Float3& origin, const Float3& unitDir, float distance, py::function reportFn, bool bothSides) {
            self.sweepSphereAll(radius, origin, unitDir, distance, [reportFn](const SweepHit& hit) {
                py::gil_scoped_acquire gil;
                return reportFn(hit).cast<bool>();
            }, bothSides);
        }, py::arg("radius"), py::arg("origin"), py::arg("unit_dir"), py::arg("distance"), py::arg("report_fn"), py::arg("both_sides"),
           "Sweep test of a sphere against all objects in the physics scene, returning all the hits found.\n\n"
           "Args:\n"
           "    radius: Sphere radius\n"
           "    origin: Origin of the sweep\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    report_fn: Scene query hit report function, return True to continue traversal, False to stop traversal\n"
           "    both_sides: If mesh both triangle sides should be checked")
        .def("overlap_sphere", [](IPhysicsSceneQuery& self, float radius, const Float3& pos, py::function reportFn) {
            return self.overlapSphere(radius, pos, [reportFn](const OverlapHit& hit) {
                py::gil_scoped_acquire gil;
                return reportFn(hit).cast<bool>();
            });
        }, py::arg("radius"), py::arg("position"), py::arg("report_fn"),
           "Overlap test of a sphere against objects in the physics scene.\n\n"
           "Args:\n"
           "    radius: Sphere radius\n"
           "    position: Sphere position\n"
           "    report_fn: Scene query hit report function, return True to continue traversal, False to stop traversal\n\n"
           "Returns:\n"
           "    int: Number of overlaps found")
        .def("overlap_sphere_any", [](IPhysicsSceneQuery& self, float radius, const Float3& pos) {
            return self.overlapSphereAny(radius, pos);
        }, py::arg("radius"), py::arg("position"),
           "Overlap test of a sphere against objects in the physics scene, reports only boolean.\n\n"
           "Args:\n"
           "    radius: Sphere radius\n"
           "    position: Sphere position\n\n"
           "Returns:\n"
           "    bool: True if overlap found")
        .def("overlap_box", [](IPhysicsSceneQuery& self, const Float3& halfExtent, const Float3& pos, const Float4& rot, py::function reportFn) {
            return self.overlapBox(halfExtent, pos, rot, [reportFn](const OverlapHit& hit) {
                py::gil_scoped_acquire gil;
                return reportFn(hit).cast<bool>();
            });
        }, py::arg("half_extent"), py::arg("position"), py::arg("rotation"), py::arg("report_fn"),
           "Overlap test of a box against objects in the physics scene.\n\n"
           "Args:\n"
           "    half_extent: Box half extent\n"
           "    position: Box position\n"
           "    rotation: Box rotation (quaternion x, y, z, w)\n"
           "    report_fn: Scene query hit report function, return True to continue traversal, False to stop traversal\n\n"
           "Returns:\n"
           "    int: Number of overlaps found")
        .def("overlap_box_any", [](IPhysicsSceneQuery& self, const Float3& halfExtent, const Float3& pos, const Float4& rot) {
            return self.overlapBoxAny(halfExtent, pos, rot);
        }, py::arg("half_extent"), py::arg("position"), py::arg("rotation"),
           "Overlap test of a box against objects in the physics scene, reports only boolean.\n\n"
           "Args:\n"
           "    half_extent: Box half extent\n"
           "    position: Box position\n"
           "    rotation: Box rotation (quaternion x, y, z, w)\n\n"
           "Returns:\n"
           "    bool: True if overlap found")
        .def("overlap_shape", [](IPhysicsSceneQuery& self, uint64_t gPrimPath, py::function reportFn) {
            return self.overlapShape(
                gPrimPath,
                [reportFn](const OverlapHit& hit) {
                    py::gil_scoped_acquire gil;
                    return reportFn(hit).cast<bool>();
                });
        }, py::arg("g_prim_path"), py::arg("report_fn"),
           "Overlap test of a UsdGeomGPrim against objects in the physics scene.\n\n"
           "Note: A convex mesh approximation will be used for meshes, the first query will compute the convex mesh\n"
           "approximation and store the result in a local cache\n\n"
           "Args:\n"
           "    g_prim_path: UsdGeomGPrim path encoded in uint64_t\n"
           "    report_fn: Scene query hit report function, return True to continue traversal, False to stop traversal\n\n"
           "Returns:\n"
           "    int: Number of overlaps found")
        .def("overlap_shape_any", [](IPhysicsSceneQuery& self, uint64_t gPrimPath) {
            return self.overlapShapeAny(gPrimPath);
        }, py::arg("g_prim_path"),
           "Overlap test of a mesh against objects in the physics scene, reports only boolean.\n\n"
           "Note: A convex mesh approximation will be used for the test, the first query will compute the convex mesh\n"
           "approximation and store the result in a local cache\n\n"
           "Args:\n"
           "    g_prim_path: UsdGeomGPrim path encoded in uint64_t\n\n"
           "Returns:\n"
           "    bool: True if overlap found")
        .def("sweep_box_closest", [](IPhysicsSceneQuery& self, const Float3& halfExtent, const Float3& pos, const Float4& rot, const Float3& unitDir, float distance, bool bothSides) {
            SweepHit hit;
            bool result = self.sweepBoxClosest(halfExtent, pos, rot, unitDir, distance, hit, bothSides);
            return std::make_tuple(result, hit);
        }, py::arg("half_extent"), py::arg("position"), py::arg("rotation"), py::arg("unit_dir"), py::arg("distance"), py::arg("both_sides"),
           "Sweep test of a box against all objects in the physics scene, returning the closest hit found.\n\n"
           "Args:\n"
           "    half_extent: Box half extent\n"
           "    position: Box position. This is the origin of the sweep\n"
           "    rotation: Box rotation (quaternion x, y, z, w)\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    both_sides: If mesh both triangle sides should be checked\n\n"
           "Returns:\n"
           "    Tuple of (bool, SweepHit): True if hit was found and the hit information")
        .def("sweep_shape_closest", [](IPhysicsSceneQuery& self, uint64_t gPrimPath, const Float3& unitDir, float distance, bool bothSides) {
            SweepHit hit;
            bool result = self.sweepShapeClosest(gPrimPath, unitDir, distance, hit, bothSides);
            return std::make_tuple(result, hit);
        }, py::arg("g_prim_path"), py::arg("unit_dir"), py::arg("distance"), py::arg("both_sides"),
           "Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning the closest hit found.\n\n"
           "Note: A convex mesh approximation will be used for meshes, the first query will compute the convex mesh\n"
           "approximation and store the result in a local cache\n\n"
           "Args:\n"
           "    g_prim_path: UsdGeomGPrim path encoded in uint64_t\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    both_sides: If mesh both triangle sides should be checked\n\n"
           "Returns:\n"
           "    Tuple of (bool, SweepHit): True if hit was found and the hit information")
        .def("sweep_box_any", [](IPhysicsSceneQuery& self, const Float3& halfExtent, const Float3& pos, const Float4& rot, const Float3& unitDir, float distance, bool bothSides) {
            return self.sweepBoxAny(halfExtent, pos, rot, unitDir, distance, bothSides);
        }, py::arg("half_extent"), py::arg("position"), py::arg("rotation"), py::arg("unit_dir"), py::arg("distance"), py::arg("both_sides"),
           "Sweep test of a box against all objects in the physics scene, returning whether any hit was found.\n\n"
           "Args:\n"
           "    half_extent: Box half extent\n"
           "    position: Box position. This is the origin of the sweep\n"
           "    rotation: Box rotation (quaternion x, y, z, w)\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    both_sides: If mesh both triangle sides should be checked\n\n"
           "Returns:\n"
           "    bool: True if hit was found")
        .def("sweep_shape_any", [](IPhysicsSceneQuery& self, uint64_t gPrimPath, const Float3& unitDir, float distance, bool bothSides) {
            return self.sweepShapeAny(gPrimPath, unitDir, distance, bothSides);
        }, py::arg("g_prim_path"), py::arg("unit_dir"), py::arg("distance"), py::arg("both_sides"),
           "Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning whether any hit was found.\n\n"
           "Note: A convex mesh approximation will be used for meshes, the first query will compute the convex mesh\n"
           "approximation and store the result in a local cache\n\n"
           "Args:\n"
           "    g_prim_path: UsdGeomGPrim path encoded in uint64_t\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    both_sides: If mesh both triangle sides should be checked\n\n"
           "Returns:\n"
           "    bool: True if hit was found")
        .def("sweep_box_all", [](IPhysicsSceneQuery& self, const Float3& halfExtent, const Float3& pos, const Float4& rot, const Float3& unitDir, float distance, py::function reportFn, bool bothSides) {
            self.sweepBoxAll(halfExtent, pos, rot, unitDir, distance, [reportFn](const SweepHit& hit) {
                py::gil_scoped_acquire gil;
                return reportFn(hit).cast<bool>();
            }, bothSides);
        }, py::arg("half_extent"), py::arg("position"), py::arg("rotation"), py::arg("unit_dir"), py::arg("distance"), py::arg("report_fn"), py::arg("both_sides"),
           "Sweep test of a box against all objects in the physics scene, returning all the hits found.\n\n"
           "Args:\n"
           "    half_extent: Box half extent\n"
           "    position: Box position. This is the origin of the sweep\n"
           "    rotation: Box rotation (quaternion x, y, z, w)\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    report_fn: Scene query hit report function, return True to continue traversal, False to stop traversal\n"
           "    both_sides: If mesh both triangle sides should be checked")
        .def("sweep_shape_all", [](IPhysicsSceneQuery& self, uint64_t gPrimPath, const Float3& unitDir, float distance, py::function reportFn, bool bothSides) {
            self.sweepShapeAll(gPrimPath, unitDir, distance, [reportFn](const SweepHit& hit) {
                py::gil_scoped_acquire gil;
                return reportFn(hit).cast<bool>();
            }, bothSides);
        }, py::arg("g_prim_path"), py::arg("unit_dir"), py::arg("distance"), py::arg("report_fn"), py::arg("both_sides"),
           "Sweep test of a UsdGeomGPrim against all objects in the physics scene, returning all the hits found.\n\n"
           "Note: A convex mesh approximation will be used for the test, the first query will compute the convex mesh\n"
           "approximation and store the result in a local cache\n\n"
           "Args:\n"
           "    g_prim_path: UsdGeomGPrim path encoded in uint64_t\n"
           "    unit_dir: Normalized direction of the sweep\n"
           "    distance: Length of the ray. Has to be in the [0, inf) range\n"
           "    report_fn: Scene query hit report function, return True to continue traversal, False to stop traversal\n"
           "    both_sides: If mesh both triangle sides should be checked");
}
} // namespace

void bindPhysicsSceneQuery(py::module& m); 
