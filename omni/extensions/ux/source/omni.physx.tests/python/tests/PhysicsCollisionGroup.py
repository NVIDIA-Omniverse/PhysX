# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import unittest
from omni.physxtests.testBases.filterTestBase import FilterTestBase
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physx import get_physx_simulation_interface
from pxr import Gf, Usd, UsdPhysics, UsdGeom, Sdf, PhysxSchema
from itertools import product

class PhysicsCollisionGroupTestKitStage(PhysicsKitStageAsyncTestCase, FilterTestBase):
    category = TestCategory.Core
    

    async def setUp(self):
        await super().setUp()
        self._stage = await utils.new_stage_setup()

        self.setup_units_and_scene()

        #register contact event callback
        self._contact_report_sub = get_physx_simulation_interface().subscribe_contact_report_events(self.on_contact_report_event)


    async def tearDown(self):        
        self._contact_report_sub = None
        await super().tearDown()


    async def run_collision_group_test(self, use_filter, dyn_type = None, dyn_coll_filter = False,
        stc_type = None, stc_coll_filter = False):
        await self.setUp()
        self._num_contact_found = 0
        
        #stc stands for the physics prim in the center not moving (either truly static or heavy in comparison)
        #dyn stands for the physics prim off center, with inital velocity on collision course
        stc_ext = Gf.Vec3f(10.0, 10.0, 1.0)
        stc_pos = Gf.Vec3f(0.0, 0.0, 0.0)
        stc_vel = Gf.Vec3f(0.0, 0.0, 0.0)
        dyn_ext = Gf.Vec3f(1.0, 1.0, 1.0)
        dyn_pos = Gf.Vec3f(0.0, 0.0, -2.0)
        dyn_vel = Gf.Vec3f(0.0, 0.0, 5.0 / (self._time_step * 20.0))

        stc_prim, support_report = self.setup_colliders(physics_type = stc_type, is_dyn = False, ext = stc_ext, pos = stc_pos, vel = stc_vel)
        dyn_prim, support_report = self.setup_colliders(physics_type = dyn_type, is_dyn = True, ext = dyn_ext, pos = dyn_pos, vel = dyn_vel)

        if use_filter:
            stc_group_path = "/collisionGroup_stc"
            dyn_group_path = "/collisionGroup_dyn"
            utils.execute_and_check(self, "AddCollisionGroup", stage=self._stage, path=stc_group_path)
            utils.execute_and_check(self, "AddCollisionGroup", stage=self._stage, path=dyn_group_path)
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

        await utils.play_and_step_and_pause(self, 20)

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
  
        await self.tearDown()


    async def run_collision_group_tests(self, dyn_type, stc_type):
        hierarchical_types = ['RigidBodyCompound', 'ArticulationLinkCompound']
        dyn_coll_filter_params = [False, True] if dyn_type in hierarchical_types else [False]
        stc_coll_filter_params = [False, True] if stc_type in hierarchical_types else [False]

        for dyn_cf, stc_cf in product(dyn_coll_filter_params, stc_coll_filter_params):
            await self.run_collision_group_test(use_filter=False, dyn_type=dyn_type, dyn_coll_filter=dyn_cf, stc_type=stc_type, stc_coll_filter=stc_cf)
            await self.run_collision_group_test(use_filter=True, dyn_type=dyn_type, dyn_coll_filter=dyn_cf, stc_type=stc_type, stc_coll_filter=stc_cf)


    async def test_collision_group_rigidbody_with_others(self):
        await self.run_collision_group_tests(dyn_type = 'RigidBody', stc_type = 'Collider')
        await self.run_collision_group_tests(dyn_type = 'RigidBody', stc_type = 'RigidBody')
        
    async def test_collision_group_rigidbodycompound_with_others(self):
        await self.run_collision_group_tests(dyn_type = 'RigidBodyCompound', stc_type = 'Collider')
        await self.run_collision_group_tests(dyn_type = 'RigidBodyCompound', stc_type = 'RigidBody')
        await self.run_collision_group_tests(dyn_type = 'RigidBodyCompound', stc_type = 'RigidBodyCompound')

    async def test_collision_group_articulation_with_others(self):
        await self.run_collision_group_tests(dyn_type = 'ArticulationLink', stc_type = 'Collider')
        await self.run_collision_group_tests(dyn_type = 'ArticulationLink', stc_type = 'RigidBody')
        await self.run_collision_group_tests(dyn_type = 'ArticulationLink', stc_type = 'RigidBodyCompound')
        await self.run_collision_group_tests(dyn_type = 'ArticulationLink', stc_type = 'ArticulationLink')

    async def test_collision_group_articulationcompound_with_others(self):
        await self.run_collision_group_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'Collider')
        await self.run_collision_group_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'RigidBody')
        await self.run_collision_group_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'RigidBodyCompound')
        await self.run_collision_group_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'ArticulationLink')
        await self.run_collision_group_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'ArticulationLinkCompound')

    async def ttest_collision_group_deformablebody_with_others(self):
        await self.run_collision_group_tests(dyn_type = 'DeformableBody', stc_type = 'Collider')
        await self.run_collision_group_tests(dyn_type = 'DeformableBody', stc_type = 'RigidBody')
        await self.run_collision_group_tests(dyn_type = 'DeformableBody', stc_type = 'RigidBodyCompound')
        await self.run_collision_group_tests(dyn_type = 'DeformableBody', stc_type = 'ArticulationLink')
        await self.run_collision_group_tests(dyn_type = 'DeformableBody', stc_type = 'ArticulationLinkCompound')
        await self.run_collision_group_tests(dyn_type = 'DeformableBody', stc_type = 'DeformableBody')

    async def test_collision_group_deformable_surface_with_others(self):
        await self.run_collision_group_tests(dyn_type = 'DeformableSurface', stc_type = 'Collider')
        await self.run_collision_group_tests(dyn_type = 'DeformableSurface', stc_type = 'RigidBody')
        await self.run_collision_group_tests(dyn_type = 'DeformableSurface', stc_type = 'RigidBodyCompound')
        await self.run_collision_group_tests(dyn_type = 'DeformableSurface', stc_type = 'ArticulationLink')
        await self.run_collision_group_tests(dyn_type = 'DeformableSurface', stc_type = 'ArticulationLinkCompound')
        await self.run_collision_group_tests(dyn_type = 'DeformableSurface', stc_type = 'DeformableBody')
        await self.run_collision_group_tests(dyn_type = 'DeformableSurface', stc_type = 'DeformableSurface')

    async def test_collision_group_particles_with_others(self):
        await self.run_collision_group_tests(dyn_type = 'Particles', stc_type = 'Collider')
        await self.run_collision_group_tests(dyn_type = 'Particles', stc_type = 'RigidBody')
        await self.run_collision_group_tests(dyn_type = 'Particles', stc_type = 'RigidBodyCompound')
        await self.run_collision_group_tests(dyn_type = 'Particles', stc_type = 'ArticulationLink')
        await self.run_collision_group_tests(dyn_type = 'Particles', stc_type = 'ArticulationLinkCompound')
        await self.run_collision_group_tests(dyn_type = 'Particles', stc_type = 'DeformableBody')
        await self.run_collision_group_tests(dyn_type = 'Particles', stc_type = 'DeformableSurface')

        #no point in testing filtering between particles and particles. If they are from different particle systems at 
        #which level the filtering takes place, they can't collide anyways.
