# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import unittest
from omni.physxtests.testBases.filterTestBase import FilterTestBase
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, PhysicsMemoryStageBaseAsyncTestCase, TestCategory
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physx import get_physx_simulation_interface
from pxr import Gf, Usd, UsdPhysics, UsdGeom, Sdf, PhysxSchema
from itertools import product


class PhysicsFilteredPairsAPITestKitStage(PhysicsKitStageAsyncTestCase, FilterTestBase):
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


    def add_pair_filter(self, prim0, prim1):
        utils.execute_and_check(self, "AddPairFilter", stage=self._stage, primPaths=[prim0.GetPath(), prim1.GetPath()])


    async def run_filtered_pairs_test(self, use_filter, dyn_type = None, dyn_coll_filter = False,
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
            dyn_filter_prims = self.get_filter_prims(dyn_type, dyn_prim, dyn_coll_filter)
            stc_filter_prims = self.get_filter_prims(stc_type, stc_prim, stc_coll_filter)
            for (dyn_prim, stc_prim) in product(dyn_filter_prims, stc_filter_prims):
                self.add_pair_filter(dyn_prim, stc_prim)

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


    async def run_filtered_pairs_tests(self, dyn_type, stc_type):
        hierarchical_types = ['RigidBodyCompound', 'ArticulationLinkCompound']
        dyn_coll_filter_params = [False, True] if dyn_type in hierarchical_types else [False]
        stc_coll_filter_params = [False, True] if stc_type in hierarchical_types else [False]

        for dyn_cf, stc_cf in product(dyn_coll_filter_params, stc_coll_filter_params):
            await self.run_filtered_pairs_test(use_filter=False, dyn_type=dyn_type, dyn_coll_filter=dyn_cf, stc_type=stc_type, stc_coll_filter=stc_cf)
            await self.run_filtered_pairs_test(use_filter=True, dyn_type=dyn_type, dyn_coll_filter=dyn_cf, stc_type=stc_type, stc_coll_filter=stc_cf)


    async def test_filtered_pairs_rigidbody_with_others(self):
        await self.run_filtered_pairs_tests(dyn_type = 'RigidBody', stc_type = 'Collider')
        await self.run_filtered_pairs_tests(dyn_type = 'RigidBody', stc_type = 'RigidBody')
        
    async def test_filtered_pairs_rigidbodycompound_with_others(self):
        await self.run_filtered_pairs_tests(dyn_type = 'RigidBodyCompound', stc_type = 'Collider')
        await self.run_filtered_pairs_tests(dyn_type = 'RigidBodyCompound', stc_type = 'RigidBody')
        await self.run_filtered_pairs_tests(dyn_type = 'RigidBodyCompound', stc_type = 'RigidBodyCompound')

    async def test_filtered_pairs_articulation_with_others(self):
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLink', stc_type = 'Collider')
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLink', stc_type = 'RigidBody')
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLink', stc_type = 'RigidBodyCompound')
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLink', stc_type = 'ArticulationLink')

    async def test_filtered_pairs_articulationcompound_with_others(self):
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'Collider')
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'RigidBody')
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'RigidBodyCompound')
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'ArticulationLink')
        await self.run_filtered_pairs_tests(dyn_type = 'ArticulationLinkCompound', stc_type = 'ArticulationLinkCompound')

    async def test_filtered_pairs_deformablebody_with_others(self):
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableBody', stc_type = 'Collider')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableBody', stc_type = 'RigidBody')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableBody', stc_type = 'RigidBodyCompound')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableBody', stc_type = 'ArticulationLink')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableBody', stc_type = 'ArticulationLinkCompound')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableBody', stc_type = 'DeformableBody')

    async def test_filtered_pairs_deformable_surface_with_others(self):
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableSurface', stc_type = 'Collider')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableSurface', stc_type = 'RigidBody')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableSurface', stc_type = 'RigidBodyCompound')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableSurface', stc_type = 'ArticulationLink')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableSurface', stc_type = 'ArticulationLinkCompound')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableSurface', stc_type = 'DeformableBody')
        await self.run_filtered_pairs_tests(dyn_type = 'DeformableSurface', stc_type = 'DeformableSurface')

    async def test_filtered_pairs_particles_with_others(self):
        await self.run_filtered_pairs_tests(dyn_type = 'Particles', stc_type = 'Collider')
        await self.run_filtered_pairs_tests(dyn_type = 'Particles', stc_type = 'RigidBody')
        await self.run_filtered_pairs_tests(dyn_type = 'Particles', stc_type = 'RigidBodyCompound')
        await self.run_filtered_pairs_tests(dyn_type = 'Particles', stc_type = 'ArticulationLink')
        await self.run_filtered_pairs_tests(dyn_type = 'Particles', stc_type = 'ArticulationLinkCompound')
        await self.run_filtered_pairs_tests(dyn_type = 'Particles', stc_type = 'DeformableBody')
        await self.run_filtered_pairs_tests(dyn_type = 'Particles', stc_type = 'DeformableSurface')

        
        #no point in testing filtering between particles and particles. If they are from different particle systems at 
        #which level the filtering takes place, they can't collide anyways.

class PhysicsFilteredPairsAPITestMemoryStage(PhysicsMemoryStageBaseAsyncTestCase, FilterTestBase):
    category = TestCategory.Core

    async def setUp(self):
        await super().setUp()
        
        #register contact event callback
        self._contact_report_sub = get_physx_simulation_interface().subscribe_contact_report_events(self.on_contact_report_event)

    async def tearDown(self):                        
        self._contact_report_sub = None
        
        await super().tearDown()

    async def setup_rigid_body_scenario(self, scenario = "simple"):
        self._stage = await self.new_stage()
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

    async def filtered_pairs_enable_disable(self, scenario):
        await self.setup_rigid_body_scenario(scenario)

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
        
    async def test_filtered_pairs_enable_disable_simple(self):
        await self.filtered_pairs_enable_disable("simple")

    async def test_filtered_pairs_enable_disable_decompostion(self):
        await self.filtered_pairs_enable_disable("decomposition")
        
    async def filtered_pairs_rel_changes(self, scenario):
        await self.setup_rigid_body_scenario(scenario)

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

    async def test_filtered_pairs_rel_changes_simple(self):
        await self.filtered_pairs_rel_changes("simple")
        
    async def test_filtered_pairs_rel_changes_decomposition(self):
        await self.filtered_pairs_rel_changes("decomposition")
