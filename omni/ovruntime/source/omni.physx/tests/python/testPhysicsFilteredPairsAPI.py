# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysicsFilteredPairsAPI.py for standalone ovruntime testing.

import _physics_setup  # noqa: F401

import unittest
from filterTestBase import FilterTestBase, create_transform
from physicsBase import PhysicsMemoryStageBaseTestCase
import physicsUtils
import _physx
from pxr import Gf, Usd, UsdPhysics, UsdGeom, Sdf, PhysxSchema
from itertools import product


class PhysicsFilteredPairsAPITestMemoryStage(PhysicsMemoryStageBaseTestCase, FilterTestBase):

    def new_stage(self):
        return super().new_stage(def_up_and_mpu=True, up=UsdGeom.Tokens.z, mpu=1.0, attach_stage=False)

    def step(self, num_steps=1):
        super().step(num_steps=num_steps)

    def setUp(self):
        super().setUp()
        self._stage = self.new_stage()

        self.setup_units_and_scene()

        self.attach_stage()

        #register contact event callback
        self._contact_report_sub = _physx.acquire_physx_simulation_interface().subscribe_contact_report_events(self.on_contact_report_event)

    def tearDown(self):
        self._contact_report_sub = None
        super().tearDown()

    def add_pair_filter(self, prim0, prim1):
        # Replace AddPairFilter command with direct USD API
        paths = [prim0.GetPath(), prim1.GetPath()]
        for path in paths:
            prim = self._stage.GetPrimAtPath(path)
            filteringPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(prim)
            rel = filteringPairsAPI.CreateFilteredPairsRel()
            for otherPath in paths:
                if otherPath != path:
                    rel.AddTarget(Sdf.Path(otherPath))

    def run_filtered_pairs_test(self, use_filter, dyn_type=None, dyn_coll_filter=False,
        stc_type=None, stc_coll_filter=False):
        self.setUp()
        self._num_contact_found = 0

        #stc stands for the physics prim in the center not moving (either truly static or heavy in comparison)
        #dyn stands for the physics prim off center, with inital velocity on collision course
        stc_ext = Gf.Vec3f(10.0, 10.0, 1.0)
        stc_pos = Gf.Vec3f(0.0, 0.0, 0.0)
        stc_vel = Gf.Vec3f(0.0, 0.0, 0.0)
        dyn_ext = Gf.Vec3f(1.0, 1.0, 1.0)
        dyn_pos = Gf.Vec3f(0.0, 0.0, -2.0)
        dyn_vel = Gf.Vec3f(0.0, 0.0, 5.0 / (self._time_step * 20.0))

        stc_prim, support_report = self.setup_colliders(physics_type=stc_type, is_dyn=False, ext=stc_ext, pos=stc_pos, vel=stc_vel)
        dyn_prim, support_report = self.setup_colliders(physics_type=dyn_type, is_dyn=True, ext=dyn_ext, pos=dyn_pos, vel=dyn_vel)

        if use_filter:
            dyn_filter_prims = self.get_filter_prims(dyn_type, dyn_prim, dyn_coll_filter)
            stc_filter_prims = self.get_filter_prims(stc_type, stc_prim, stc_coll_filter)
            for (dyn_prim, stc_prim) in product(dyn_filter_prims, stc_filter_prims):
                self.add_pair_filter(dyn_prim, stc_prim)

        self.step(20)

        stc_min, stc_max = self.get_min_max(stc_prim)
        dyn_min, dyn_max = self.get_min_max(dyn_prim)

        if use_filter:
            if dyn_type != "ArticulationLinkCompound":
                self.assertTrue(dyn_min > stc_max)
            self.assertTrue(not support_report or self._num_contact_found == 0)
        else:
            slack = 0.1
            if dyn_type != "ArticulationLinkCompound":
                self.assertTrue(dyn_max < stc_min + slack)
            self.assertTrue(not support_report or self._num_contact_found > 0)

        self.tearDown()

    def run_filtered_pairs_tests(self, dyn_type, stc_type):
        hierarchical_types = ['RigidBodyCompound', 'ArticulationLinkCompound']
        dyn_coll_filter_params = [False, True] if dyn_type in hierarchical_types else [False]
        stc_coll_filter_params = [False, True] if stc_type in hierarchical_types else [False]

        for dyn_cf, stc_cf in product(dyn_coll_filter_params, stc_coll_filter_params):
            self.run_filtered_pairs_test(use_filter=False, dyn_type=dyn_type, dyn_coll_filter=dyn_cf, stc_type=stc_type, stc_coll_filter=stc_cf)
            self.run_filtered_pairs_test(use_filter=True, dyn_type=dyn_type, dyn_coll_filter=dyn_cf, stc_type=stc_type, stc_coll_filter=stc_cf)

    def test_filtered_pairs_rigidbody_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='RigidBody', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='RigidBody', stc_type='RigidBody')

    def test_filtered_pairs_rigidbodycompound_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='RigidBodyCompound', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='RigidBodyCompound', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='RigidBodyCompound', stc_type='RigidBodyCompound')

    def test_filtered_pairs_articulation_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='ArticulationLink', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='ArticulationLink', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='ArticulationLink', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='ArticulationLink', stc_type='ArticulationLink')

    def test_filtered_pairs_articulationcompound_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='ArticulationLinkCompound', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='ArticulationLinkCompound', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='ArticulationLinkCompound', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='ArticulationLinkCompound', stc_type='ArticulationLink')
        self.run_filtered_pairs_tests(dyn_type='ArticulationLinkCompound', stc_type='ArticulationLinkCompound')

    def test_filtered_pairs_particles_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='ArticulationLink')
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='ArticulationLinkCompound')
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='VolumeDeformableNonHierarchy')
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='VolumeDeformableHierarchyXform')
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='SurfaceDeformableNonHierarchy')
        self.run_filtered_pairs_tests(dyn_type='Particles', stc_type='SurfaceDeformableHierarchyXform')

    def test_filtered_pairs_volume_deformable_hierarchy_xform_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='ArticulationLink')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='ArticulationLinkCompound')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='VolumeDeformableHierarchyCollMesh')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='VolumeDeformableNonHierarchy')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='VolumeDeformableHierarchyXform')

    def test_filtered_pairs_volume_deformable_hierarchy_collmesh_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='ArticulationLink')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='ArticulationLinkCompound')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='VolumeDeformableHierarchyXform')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='VolumeDeformableNonHierarchy')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='VolumeDeformableHierarchyCollMesh')

    def test_filtered_pairs_volume_deformable_non_hierarchy_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='ArticulationLink')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='ArticulationLinkCompound')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='VolumeDeformableHierarchyXform')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='VolumeDeformableHierarchyCollMesh')
        self.run_filtered_pairs_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='VolumeDeformableNonHierarchy')

    def test_filtered_pairs_surface_deformable_hierarchy_xform_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='ArticulationLink')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='ArticulationLinkCompound')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='SurfaceDeformableHierarchyCollMesh')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='SurfaceDeformableNonHierarchy')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='SurfaceDeformableHierarchyXform')

    def test_filtered_pairs_surface_deformable_hierarchy_collmesh_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='ArticulationLink')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='ArticulationLinkCompound')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='SurfaceDeformableHierarchyXform')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='SurfaceDeformableNonHierarchy')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='SurfaceDeformableHierarchyCollMesh')

    def test_filtered_pairs_surface_deformable_non_hierarchy_with_others(self):
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='Collider')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='RigidBody')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='RigidBodyCompound')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='ArticulationLink')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='ArticulationLinkCompound')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='SurfaceDeformableHierarchyXform')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='SurfaceDeformableHierarchyCollMesh')
        self.run_filtered_pairs_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='SurfaceDeformableNonHierarchy')

    #no point in testing filtering between particles and particles. If they are from different particle systems at
    #which level the filtering takes place, they can't collide anyways.


class PhysicsFilteredPairsAPITestMemoryStage2(PhysicsMemoryStageBaseTestCase, FilterTestBase):

    def setUp(self):
        super().setUp()

        #register contact event callback
        self._contact_report_sub = _physx.acquire_physx_simulation_interface().subscribe_contact_report_events(self.on_contact_report_event)

    def tearDown(self):
        self._contact_report_sub = None
        super().tearDown()

    def setup_rigid_body_scenario(self, scenario="simple"):
        self._stage = self.new_stage()
        physics_scene = UsdPhysics.Scene.Define(self._stage, "/World/scene")
        PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())

        ext = Gf.Vec3f(10.0, 10.0, 10.0)
        pos = Gf.Vec3f(0.0, 0.0, 0.0)
        prim = physicsUtils.add_rigid_box(self._stage, "/World/box0", size=ext, position=pos)
        rboAPI = UsdPhysics.RigidBodyAPI.Apply(prim)
        rboAPI.CreateKinematicEnabledAttr().Set(True)
        contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        contactReportAPI.CreateThresholdAttr().Set(0)
        self._box0 = prim

        if scenario == "simple":
            pos = Gf.Vec3f(0.0, 30.0, 0.0)
            prim = physicsUtils.add_rigid_box(self._stage, "/World/box1", size=ext, position=pos)
            self._box1 = prim
        else:
            pos = Gf.Vec3f(0.0, 30.0, 0.0)
            prim = UsdGeom.Xform.Define(self._stage, "/World/box1").GetPrim()
            xformable = UsdGeom.Xform(prim)
            physicsUtils.set_or_add_translate_op(xformable, translate=pos)
            mesh_prim = physicsUtils.create_mesh_concave(self._stage, "/World/box1/mesh", 10.0).GetPrim()
            rboAPI = UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(mesh_prim)
            mesh_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
            mesh_api.CreateApproximationAttr("convexDecomposition")
            self._box1 = prim

    def filtered_pairs_enable_disable(self, scenario):
        self.setup_rigid_body_scenario(scenario)

        self._num_contact_found = 0

        # by default we collide
        for _ in range(20):
            self.step()

        self.assertTrue(self._num_contact_found > 0)

        # add filtered pairs
        filteredPairsAPI = UsdPhysics.FilteredPairsAPI.Apply(self._box1)
        filteredPairsAPI.CreateFilteredPairsRel().AddTarget(self._box0.GetPrimPath())

        self._box1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 30.0, 0.0))
        self._num_contact_found = 0

        for _ in range(20):
            self.step()

        self.assertTrue(self._num_contact_found == 0)

        # remove filtered pairs
        self._box1.RemoveAPI(UsdPhysics.FilteredPairsAPI)

        self._box1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 30.0, 0.0))
        self._num_contact_found = 0

        for _ in range(20):
            self.step()

        self.assertTrue(self._num_contact_found > 0)

    def test_filtered_pairs_enable_disable_simple(self):
        self.filtered_pairs_enable_disable("simple")

    def test_filtered_pairs_enable_disable_decompostion(self):
        self.filtered_pairs_enable_disable("decomposition")

    def filtered_pairs_rel_changes(self, scenario):
        self.setup_rigid_body_scenario(scenario)

        self._num_contact_found = 0

        # by default we collide
        for _ in range(20):
            self.step()

        self.assertTrue(self._num_contact_found > 0)

        # add filtered pairs
        filteredPairsAPI1 = UsdPhysics.FilteredPairsAPI.Apply(self._box1)
        filteredPairsAPI1.CreateFilteredPairsRel().AddTarget(self._box0.GetPrimPath())

        self._box1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 30.0, 0.0))
        self._num_contact_found = 0

        for _ in range(20):
            self.step()

        self.assertTrue(self._num_contact_found == 0)

        # add second rel
        filteredPairsAPI0 = UsdPhysics.FilteredPairsAPI.Apply(self._box0)
        filteredPairsAPI0.CreateFilteredPairsRel().AddTarget(self._box1.GetPrimPath())

        self._box1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 30.0, 0.0))
        self._num_contact_found = 0

        for _ in range(20):
            self.step()

        self.assertTrue(self._num_contact_found == 0)

        # remove second rel
        filteredPairsAPI0.CreateFilteredPairsRel().ClearTargets(True)

        self._box1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 30.0, 0.0))
        self._num_contact_found = 0

        for _ in range(20):
            self.step()

        self.assertTrue(self._num_contact_found == 0)

        # remove first rel
        filteredPairsAPI1.CreateFilteredPairsRel().ClearTargets(True)

        self._box1.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 30.0, 0.0))
        self._num_contact_found = 0

        for _ in range(20):
            self.step()

        self.assertTrue(self._num_contact_found > 0)

    def test_filtered_pairs_rel_changes_simple(self):
        self.filtered_pairs_rel_changes("simple")

    def test_filtered_pairs_rel_changes_decomposition(self):
        self.filtered_pairs_rel_changes("decomposition")


if __name__ == "__main__":
    unittest.main()
