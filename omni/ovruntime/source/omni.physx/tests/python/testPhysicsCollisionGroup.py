# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Ported from omni.physx.tests PhysicsCollisionGroup.py for standalone ovruntime testing.

import _physics_setup  # noqa: F401

import unittest
from filterTestBase import FilterTestBase, create_transform
from physicsBase import PhysicsMemoryStageBaseTestCase
import physicsUtils
import _physx
from pxr import Gf, Usd, UsdPhysics, UsdGeom, Sdf, PhysxSchema
from itertools import product


class PhysicsCollisionGroupTestMemoryStage(PhysicsMemoryStageBaseTestCase, FilterTestBase):

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

    def run_collision_group_test(self, use_filter, dyn_type=None, dyn_coll_filter=False,
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
            stc_group_path = "/collisionGroup_stc"
            dyn_group_path = "/collisionGroup_dyn"
            # Replace AddCollisionGroup command with direct USD API
            stc_collision_group = UsdPhysics.CollisionGroup.Define(self._stage, Sdf.Path(stc_group_path))
            stc_collision_group.CreateFilteredGroupsRel()
            dyn_collision_group = UsdPhysics.CollisionGroup.Define(self._stage, Sdf.Path(dyn_group_path))
            dyn_collision_group.CreateFilteredGroupsRel()
            stc_group = UsdPhysics.CollisionGroup.Get(self._stage, stc_group_path)
            dyn_group = UsdPhysics.CollisionGroup.Get(self._stage, dyn_group_path)
            self.assertTrue(stc_group and dyn_group)
            stc_group.GetFilteredGroupsRel().AddTarget(dyn_group_path)

            dyn_filter_prims = self.get_filter_prims(dyn_type, dyn_prim, dyn_coll_filter)
            stc_filter_prims = self.get_filter_prims(stc_type, stc_prim, stc_coll_filter)
            for stc_prim in stc_filter_prims:
                physicsUtils.add_collision_to_collision_group(self._stage, stc_prim.GetPath(), stc_group_path)

            for dyn_prim in dyn_filter_prims:
                physicsUtils.add_collision_to_collision_group(self._stage, dyn_prim.GetPath(), dyn_group_path)

        self.step(20)

        stc_min, stc_max = self.get_min_max(stc_prim)
        dyn_min, dyn_max = self.get_min_max(dyn_prim)

        if use_filter:
            if dyn_type != "ArticulationLinkCompound":
                self.assertTrue(dyn_min > stc_max,
                    f"filter dyn={dyn_type} stc={stc_type}: dyn_min={dyn_min} should be > stc_max={stc_max}")
            self.assertTrue(not support_report or self._num_contact_found == 0,
                f"filter dyn={dyn_type} stc={stc_type}: expected 0 contacts, got {self._num_contact_found}")
        else:
            slack = 0.1
            if dyn_type != "ArticulationLinkCompound":
                self.assertTrue(dyn_max < stc_min + slack,
                    f"no-filter dyn={dyn_type} stc={stc_type}: dyn_max={dyn_max} should be < stc_min+slack={stc_min + slack}")
            self.assertTrue(not support_report or self._num_contact_found > 0,
                f"no-filter dyn={dyn_type} stc={stc_type}: expected contacts > 0, got {self._num_contact_found}")

        self.tearDown()

    def run_collision_group_tests(self, dyn_type, stc_type):
        hierarchical_types = ['RigidBodyCompound', 'ArticulationLinkCompound']
        dyn_coll_filter_params = [False, True] if dyn_type in hierarchical_types else [False]
        stc_coll_filter_params = [False, True] if stc_type in hierarchical_types else [False]

        for dyn_cf, stc_cf in product(dyn_coll_filter_params, stc_coll_filter_params):
            self.run_collision_group_test(use_filter=False, dyn_type=dyn_type, dyn_coll_filter=dyn_cf, stc_type=stc_type, stc_coll_filter=stc_cf)
            self.run_collision_group_test(use_filter=True, dyn_type=dyn_type, dyn_coll_filter=dyn_cf, stc_type=stc_type, stc_coll_filter=stc_cf)

    def test_collision_group_rigidbody_with_others(self):
        self.run_collision_group_tests(dyn_type='RigidBody', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='RigidBody', stc_type='RigidBody')

    def test_collision_group_rigidbodycompound_with_others(self):
        self.run_collision_group_tests(dyn_type='RigidBodyCompound', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='RigidBodyCompound', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='RigidBodyCompound', stc_type='RigidBodyCompound')

    def test_collision_group_articulation_with_others(self):
        self.run_collision_group_tests(dyn_type='ArticulationLink', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='ArticulationLink', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='ArticulationLink', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='ArticulationLink', stc_type='ArticulationLink')

    def test_collision_group_articulationcompound_with_others(self):
        self.run_collision_group_tests(dyn_type='ArticulationLinkCompound', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='ArticulationLinkCompound', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='ArticulationLinkCompound', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='ArticulationLinkCompound', stc_type='ArticulationLink')
        self.run_collision_group_tests(dyn_type='ArticulationLinkCompound', stc_type='ArticulationLinkCompound')

    def test_collision_group_particles_with_others(self):
        self.run_collision_group_tests(dyn_type='Particles', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='Particles', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='Particles', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='Particles', stc_type='ArticulationLink')
        self.run_collision_group_tests(dyn_type='Particles', stc_type='ArticulationLinkCompound')
        self.run_collision_group_tests(dyn_type='Particles', stc_type='VolumeDeformableNonHierarchy')
        self.run_collision_group_tests(dyn_type='Particles', stc_type='VolumeDeformableHierarchyXform')
        self.run_collision_group_tests(dyn_type='Particles', stc_type='SurfaceDeformableNonHierarchy')
        self.run_collision_group_tests(dyn_type='Particles', stc_type='SurfaceDeformableHierarchyXform')

    def test_collision_group_volume_deformable_hierarchy_xform_with_others(self):
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='ArticulationLink')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='ArticulationLinkCompound')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='VolumeDeformableHierarchyCollMesh')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='VolumeDeformableNonHierarchy')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyXform', stc_type='VolumeDeformableHierarchyXform')

    def test_collision_group_volume_deformable_hierarchy_collmesh_with_others(self):
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='ArticulationLink')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='ArticulationLinkCompound')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='VolumeDeformableHierarchyXform')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='VolumeDeformableNonHierarchy')
        self.run_collision_group_tests(dyn_type='VolumeDeformableHierarchyCollMesh', stc_type='VolumeDeformableHierarchyCollMesh')

    def test_collision_group_volume_deformable_non_hierarchy_with_others(self):
        self.run_collision_group_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='ArticulationLink')
        self.run_collision_group_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='ArticulationLinkCompound')
        self.run_collision_group_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='VolumeDeformableHierarchyXform')
        self.run_collision_group_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='VolumeDeformableHierarchyCollMesh')
        self.run_collision_group_tests(dyn_type='VolumeDeformableNonHierarchy', stc_type='VolumeDeformableNonHierarchy')

    def test_collision_group_surface_deformable_hierarchy_xform_with_others(self):
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='ArticulationLink')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='ArticulationLinkCompound')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='SurfaceDeformableHierarchyCollMesh')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='SurfaceDeformableNonHierarchy')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyXform', stc_type='SurfaceDeformableHierarchyXform')

    def test_collision_group_surface_deformable_hierarchy_collmesh_with_others(self):
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='ArticulationLink')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='ArticulationLinkCompound')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='SurfaceDeformableHierarchyXform')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='SurfaceDeformableNonHierarchy')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableHierarchyCollMesh', stc_type='SurfaceDeformableHierarchyCollMesh')

    def test_collision_group_surface_deformable_non_hierarchy_with_others(self):
        self.run_collision_group_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='Collider')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='RigidBody')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='RigidBodyCompound')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='ArticulationLink')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='ArticulationLinkCompound')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='SurfaceDeformableHierarchyXform')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='SurfaceDeformableHierarchyCollMesh')
        self.run_collision_group_tests(dyn_type='SurfaceDeformableNonHierarchy', stc_type='SurfaceDeformableNonHierarchy')

    #no point in testing filtering between particles and particles. If they are from different particle systems at
    #which level the filtering takes place, they can't collide anyways.


if __name__ == "__main__":
    unittest.main()
