# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb.settings
import omni.kit.test
import omni.physx.scripts.utils
from omni.physx import get_physx_simulation_interface, get_physx_cooking_interface, get_physx_interface
from omni.physxtests import utils
from enum import Enum
from pxr import UsdGeom, UsdUtils
import contextlib
import inspect
import fnmatch
import os
from omni.physx.bindings._physx import (
    SETTING_DISPLAY_JOINTS,
)


def is_running_under_teamcity():
    return bool(os.getenv("TEAMCITY_VERSION"))


class TestCategory(Enum):
    Kit = 1
    Core = 2
    Local = 3


class SubtestTCWrapper():
    def __init__(self, full_name):
        self.full_name = full_name

    def __enter__(self):
        print(f"##teamcity[testStarted name='{self.full_name}' flowId='{self.full_name}']")

    def __exit__(self, type, value, traceback):
        print(f"##teamcity[testFinished name='{self.full_name}' flowId='{self.full_name}']")


class PhysicsBaseAsyncTestCase(omni.kit.test.AsyncTestCase, utils.AssertTestUtils):
    async def setUp(self):
        isregistry = carb.settings.get_settings()
        self._joint_helpers = isregistry.get_as_bool(SETTING_DISPLAY_JOINTS)
        isregistry.set_bool(SETTING_DISPLAY_JOINTS, False)
        self._sub_filter = isregistry.get_as_string("/exts/omni.physx.tests/subTestsFilter")

        get_physx_cooking_interface().release_local_mesh_cache()

        self._stage = None

    async def tearDown(self):
        isregistry = carb.settings.get_settings()
        isregistry.set_bool(SETTING_DISPLAY_JOINTS, self._joint_helpers)

    async def wait(self, frames=1):
        for _ in range(frames):
            await omni.kit.app.get_app().next_update_async()

    def get_stage(self):
        return self._stage

    # Use only if you know what you are doing. The provided stage has to match the type of stage used
    # for the test type. For KitStageTest, it has to be a kit stage, for MemoryStageTest an in-memory
    # stage etc.
    def set_stage(self, stage):
        self._stage = stage

    # overriding subTest and skipTest to provide good TC filtering
    @contextlib.contextmanager
    def subTest(self, subtest_name=None, test_name=None, **params):
        def get_test_name():
            return test_name if test_name is not None else inspect.stack()[3][0].f_code.co_name

        if subtest_name is None and params:
            subtest_name = "_".join([f"{k}={v}" for k, v in params.items()])

        full_name = f"{get_test_name()}_{subtest_name}"

        include = self._sub_filter == "" or fnmatch.fnmatch(subtest_name, self._sub_filter)

        if not is_running_under_teamcity():
            with super().subTest(subtest_name):
                setattr(self._subtest, "full_name", full_name)
                yield include
        else:
            with super().subTest(subtest_name), SubtestTCWrapper(full_name):
                setattr(self._subtest, "full_name", full_name)
                yield include

    def skipTest(self, reason):
        if self._subtest is not None:
            if is_running_under_teamcity():
                print(f"##teamcity[testIgnored name='{self._subtest.full_name}' flowId='{self._subtest.full_name}' message='{reason}']")
            else:
                print(f"Skipping {self._subtest.full_name}. Reason: {reason}")
        else:
            super().skipTest(reason)


class PhysicsKitStageAsyncTestCase(PhysicsBaseAsyncTestCase):
    async def new_stage(self, def_up_and_mpu=True):
        self._stage = await utils.new_stage_setup(def_up_and_mpu)
        return self._stage

    async def step(self, num_steps=1, dt=1.0 / 60.0, stop_timeline_after=False):
        if stop_timeline_after:
            await utils.play_and_step_and_stop(self, num_steps)
        else:
            await utils.play_and_step_and_pause(self, num_steps)

    async def frame(self, num_frames=2, stop_timeline_after=False):
        utils.execute_and_check(self, "ToolbarPlayButtonClicked")

        for _ in range(0, num_frames - 1):
            await omni.kit.app.get_app().next_update_async()

        if stop_timeline_after:
            utils.execute_and_check(self, "ToolbarStopButtonClicked")
        else:
            utils.execute_and_check(self, "ToolbarPauseButtonClicked")

        # wait for stop to process
        await omni.kit.app.get_app().next_update_async()

    async def stop(self):
        await utils.stop_timeline(self)


class PhysicsMemoryStageBaseAsyncTestCase(PhysicsBaseAsyncTestCase):
    """This test case is used to set up a memory stage. Memory stages can be used to test for components without
    external extensions interfering. Using a memory stage is faster than using a kit stage."""
    async def setUp(self):
        await super().setUp()
        self._stage_attached = False

    async def tearDown(self):
        self.release_stage(self._stage_attached)
        await super().tearDown()

    async def new_stage(self, def_up_and_mpu=True, up=UsdGeom.Tokens.y, mpu=0.01, attach_stage=True, file_to_load=""):
        """Creates a new memory stage.

        Note: if a memory stage has already been created it will release it and replace it with the new stage.

        Args:
            def_up_and_mpu: If true the new stage will use the default up axis (y) and meters per unit (0.01).
            attach_stage: If true the stage will be attached after being created.
            file_to_load: A .usd(a) file to to load into the new stage.

        Returns:
            The new stage, the new stage is also assigned to the _stage member.
        """
        self.release_stage(self._stage_attached)
        self._stage = omni.physx.scripts.utils.new_memory_stage(attach_stage, file_to_load)
        self._stage_attached = attach_stage
        if def_up_and_mpu:
            UsdGeom.SetStageUpAxis(self._stage, up)
            UsdGeom.SetStageMetersPerUnit(self._stage, mpu)
        # let ui update itself
        await omni.kit.app.get_app().next_update_async()
        return self._stage

    def release_stage(self, detach_stage=True):
        """Erases the stage from the stage cache.

        Args:
            detach_stage: If true the stage will be detached before being released.
        """
        if self._stage is not None:
            omni.physx.scripts.utils.release_memory_stage(self._stage, detach_stage)
            self._stage = None
            self._stage_attached = not detach_stage

    def attach_stage(self):
        """Attaches _stage member to omni.physx, which will run the physics parser
        and will populate the PhysX SDK with the corresponding simulation objects.

        Note: previous stage will be detached.

        Returns:
            The stage ID of the stage that was attached.

        Asserts:
            attach_stage: Cannot attach because no memory stage in _stage, use new_stage first.
        """
        # shouldn't be able to attach a stage that has been released
        assert self._stage is not None, "attach_stage: Cannot attach because no memory stage in _stage, use new_stage first."

        cache = UsdUtils.StageCache.Get()
        stage_id = cache.GetId(self._stage).ToLongInt()
        get_physx_simulation_interface().attach_stage(stage_id)
        self._stage_attached = True
        return stage_id

    def detach_stage(self):
        """Detach USD stage, this will remove all objects from the PhysX SDK"""
        get_physx_simulation_interface().detach_stage()
        self._stage_attached = False

    def step(self, num_steps=1, dt=1.0 / 60.0, reset_simulation_after=False):
        """Executes simulation steps.

        Note: If the stage is detached then it will be attached before the simulation.

        Args:
            num_steps: The number of steps to take.
            dt: The amount of time each step is.
            reset_simulation_after: If true the simulation will be reset after all steps have completed.

        Asserts:
            step: Cannot step because no memory stage in _stage, use new_stage first.
        """
        # shouldn't be able to step if there is no stage
        assert self._stage is not None, "step: Cannot step because no memory stage in _stage, use new_stage first."

        # stage should be attached before stepping
        if not self._stage_attached:
            self.attach_stage()

        for i in range(num_steps):
            get_physx_simulation_interface().simulate(dt, i * dt)
            get_physx_simulation_interface().fetch_results()
        if reset_simulation_after:
            get_physx_interface().reset_simulation()
