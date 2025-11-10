# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb.settings
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physx import get_physx_interface
from omni.physxtests import utils
import omni.usd
from omni.physxui.scripts.settings import get_no_stage_err_msg


class PhysxInterfaceSettingsTestKitStage(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_base_perstage_check(self):
        import omni.physx.bindings._physx as pb
        iface = get_physx_interface()
        settings = carb.settings.get_settings()

        per_stage_wchanges = {
            pb.SETTING_UPDATE_TO_USD: False,
            pb.SETTING_UPDATE_VELOCITIES_TO_USD: False,
            pb.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE: True,
            pb.SETTING_UPDATE_PARTICLES_TO_USD: False,
            pb.SETTING_MIN_FRAME_RATE: 10,
            pb.SETTING_MOUSE_INTERACTION_ENABLED: False,
            pb.SETTING_MOUSE_GRAB: False,
            pb.SETTING_MOUSE_GRAB_IGNORE_INVISBLE: False,
            pb.SETTING_MOUSE_GRAB_WITH_FORCE: False,
            pb.SETTING_MOUSE_PUSH: 100.0,
            pb.SETTING_MOUSE_PICKING_FORCE: 0.0            
        }

        def do_changes():
            # for TC/spawned tests stage will be null at this point, put up ExpectMessage to test for that
            stage = omni.usd.get_context().get_stage()
            for path, change in per_stage_wchanges.items():
                if not stage:
                    with utils.ExpectMessage(self, get_no_stage_err_msg(path)):
                        settings.set(path, change)
                else:
                    settings.set(path, change)

        # do changes before stage is spawned to check for errors
        do_changes()

        # make new stage and reset manually (to make sure) to get defaults
        await self.new_stage()
        iface.reset_settings_in_stage()
        per_stage_defaults = [settings.get(path) for path in per_stage_wchanges.keys()]

        # check if changes in this test are actual changes, compare them with defaults
        for change, default in zip(per_stage_wchanges.values(), per_stage_defaults):
            self.assertTrue(change != default)

        def check_against_current(values, cmp_fn):
            for path, change in zip(per_stage_wchanges.keys(), values):
                current = settings.get(path)
                cmp_fn(current == change)

        # do changes to stage and check if changed properly
        do_changes()
        check_against_current(per_stage_wchanges.values(), self.assertTrue)

        # reset with new stage and check if settings are back to defaults
        await self.new_stage()
        check_against_current(per_stage_defaults, self.assertTrue)

    async def test_reset_all(self):
        import omni.physx.bindings._physx as pb
        iface = get_physx_interface()
        settings = carb.settings.get_settings()

        await self.new_stage()

        # choose one from preferences, one from perstage, one general one
        orig_autocreate = settings.get(pb.SETTING_AUTOCREATE_PHYSICS_SCENE)
        settings.set(pb.SETTING_AUTOCREATE_PHYSICS_SCENE, not orig_autocreate)
        orig_minframe = settings.get(pb.SETTING_MIN_FRAME_RATE)
        settings.set(pb.SETTING_MIN_FRAME_RATE, orig_minframe + 1)
        orig_coll = settings.get(pb.SETTING_DISPLAY_COLLIDERS)
        settings.set(pb.SETTING_DISPLAY_COLLIDERS, orig_coll + 1)
        orig_mass = settings.get(pb.SETTING_DISPLAY_MASS_PROPERTIES)
        settings.set(pb.SETTING_DISPLAY_MASS_PROPERTIES, orig_mass + 1)

        self.assertFalse(settings.get(pb.SETTING_AUTOCREATE_PHYSICS_SCENE) == orig_autocreate)
        self.assertFalse(settings.get(pb.SETTING_MIN_FRAME_RATE) == orig_minframe)
        self.assertFalse(settings.get(pb.SETTING_DISPLAY_COLLIDERS) == orig_coll)
        self.assertFalse(settings.get(pb.SETTING_DISPLAY_MASS_PROPERTIES) == orig_mass)

        iface.reset_settings()

        self.assertTrue(settings.get(pb.SETTING_AUTOCREATE_PHYSICS_SCENE) == orig_autocreate)
        self.assertTrue(settings.get(pb.SETTING_MIN_FRAME_RATE) == orig_minframe)
        self.assertTrue(settings.get(pb.SETTING_DISPLAY_COLLIDERS) == orig_coll)
        self.assertTrue(settings.get(pb.SETTING_DISPLAY_MASS_PROPERTIES) == orig_mass)

    async def test_reset_preferences(self):
        import omni.physx.bindings._physx as pb
        iface = get_physx_interface()
        settings = carb.settings.get_settings()

        await self.new_stage()

        preferences_wchanges = {
            pb.SETTING_AUTOCREATE_PHYSICS_SCENE: False,
            pb.SETTING_RESET_ON_STOP: False,
            pb.SETTING_USE_ACTIVE_CUDA_CONTEXT: True,
            pb.SETTING_NUM_THREADS: 4,
            pb.SETTING_EXPOSE_PROFILER_DATA: False,
            pb.SETTING_USE_LOCAL_MESH_CACHE: False,
            pb.SETTING_LOCAL_MESH_CACHE_SIZE_MB: 256,
        }

        for p, v in preferences_wchanges.items():
            settings.set(p, v)
            self.assertFalse(settings.get(p) == settings.get("/defaults" + p))

        iface.reset_settings_in_preferences()

        for p, v in preferences_wchanges.items():
            self.assertTrue(settings.get(p) == settings.get("/defaults" + p))
