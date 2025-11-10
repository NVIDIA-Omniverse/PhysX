# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import asyncio
import carb
import carb.logging
from pxr import Gf, UsdPhysics, Sdf, UsdGeom
import omni.usd
import omni.kit.stage_templates
from omni.physx import get_physx_interface, get_physxunittests_interface
from omni.physx.scripts.assets_paths import AssetFolders
from omni.kit.commands import execute
from timeit import default_timer as timer
from omni.physx.bindings._physx import SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP
from .assertTestUtils import AssertTestUtils
from omni.physx.scripts.utils import ExpectMessage
import omni.physx.bindings._physx as pb
from omni.physx.scripts.assets_paths import get_asset_path


# # # general utils # # #


async def new_stage_setup(def_up_and_mpu=True, up=UsdGeom.Tokens.y, mpu=0.01):
    # Workaround for Kit event loop issues
    for _ in range(carb.settings.acquire_settings_interface().get(SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP)):
        await omni.kit.app.get_app().next_update_async()
    await omni.kit.stage_templates.new_stage_async()
    for _ in range(carb.settings.acquire_settings_interface().get(SETTING_NUM_EVENT_PUMPS_FOR_TEST_STAGE_SETUP)):
        await omni.kit.app.get_app().next_update_async()
    stage = omni.usd.get_context().get_stage()
    if def_up_and_mpu:
        UsdGeom.SetStageUpAxis(stage, up)
        UsdGeom.SetStageMetersPerUnit(stage, mpu)
    return stage


def get_path_with_default(stage, path):
        return str(stage.GetDefaultPrim().GetPath()) + path


def check_stats(test_case, expStats):
    """Deprecated: When in a test case please use self._check_physx_object_counts"""
    simStats = get_physxunittests_interface().get_physics_stats()
    for key, value in expStats.items():
        test_case.assertEqual(simStats[key], value)


def physics_scene_setup(stage, gravity=Gf.Vec3f(0.0, -981.0, 0.0), path="/physicsScene"):
    path = omni.usd.get_stage_next_free_path(stage, path, True)
    scene = UsdPhysics.Scene.Define(stage, path)
    scene.CreateGravityDirectionAttr().Set(gravity.GetNormalized())
    scene.CreateGravityMagnitudeAttr().Set(gravity.GetLength())


def execute_and_check(test_case, cmd_name, **kwargs):
    """Deprecated: When in a test case please use self._execute_and_check"""
    res, ret = execute(cmd_name, **kwargs)
    test_case.assertTrue(res is True)
    return ret

def get_tests_asset_path(resource_folder: str, resource_relative_path: str) -> str:
    return get_asset_path(pb.SETTING_TESTS_ASSETS_PATH, resource_folder, resource_relative_path)

class AssertErrorMessage(ExpectMessage):
    def __enter__(self):
        carb.log_warn("AssertErrorMessage is a deprecated name. Please rename to ExpectMessage")
        super().__enter__()


# # # step utils # # #


# play timeline, run at least a steps number of physics steps and stop timeline
async def play_and_step_and_stop(test_case, steps, timeout_per_step=1.0 / 6.0):
    execute_and_check(test_case, "ToolbarPlayButtonClicked")
    test_case.assertTrue(await wait_for_physics_steps(steps, timeout_per_step))
    execute_and_check(test_case, "ToolbarStopButtonClicked")
    # wait for stop to process
    await omni.kit.app.get_app().next_update_async()


# play timeline, run at least a steps number of physics steps and pause timeline
async def play_and_step_and_pause(test_case, steps, timeout_per_step=1.0 / 6.0):
    execute_and_check(test_case, "ToolbarPlayButtonClicked")
    test_case.assertTrue(await wait_for_physics_steps(steps, timeout_per_step))
    execute_and_check(test_case, "ToolbarPauseButtonClicked")
    # wait for pause to process
    await omni.kit.app.get_app().next_update_async()


async def stop_timeline(test_case):
    execute_and_check(test_case, "ToolbarStopButtonClicked")
    # wait for stop to process
    await omni.kit.app.get_app().next_update_async()


async def wait_for_physics_steps(steps, timeout_per_step=1.0 / 6.0):
    if steps <= 0:
        carb.log_warn(f"steps must be more than 0!")
        return False
    remaining = steps
    finished = asyncio.Future()
    def on_step(dt):
        nonlocal finished
        nonlocal remaining
        remaining = remaining - 1
        if remaining == 0:
            finished.set_result(True)
    res = get_physx_interface().subscribe_physics_step_events(on_step)
    timeout = 1 + timeout_per_step * steps
    ret = True
    try:
        start = timer()
        await asyncio.wait_for(finished, None)
        end = timer()
        interval = end - start
        msg = f"{steps} step(s) took {interval}s. Timeout was {timeout}s."
        if interval > timeout:
            carb.log_warn(msg)
        else:
            carb.log_info(msg)
    except asyncio.TimeoutError:
        carb.log_warn(f"wait_for_physics_steps timeouted after {timeout}s")
        ret = False
    res = None
    return ret

# run an exact number of physics steps without the timeline
def do_physics_steps(steps):
    physxUTI = get_physxunittests_interface()

    one_step = 1.0 / 60.0
    for i in range(steps):
        physxUTI.update(one_step, one_step)


async def wait_for_stage_loading_status(usd_context=None, max_loops=2):
    if usd_context is None:
        usd_context = omni.usd.get_context()
    loops = 0
    total_waits = 0
    while loops < max_loops:
        total_waits +=1
        _, files_loaded, total_files = usd_context.get_stage_loading_status()
        carb.log_info(f"files loaded {files_loaded}, total_files {total_files}")
        await omni.kit.app.get_app().next_update_async()
        if files_loaded or total_files:
            continue
        loops += 1
    carb.log_info(f"total waits: {total_waits}")


# deprecated
class PhysicsStepper():
    # play timeline, run at least a minSteps number of physics steps and pause timeline
    async def play_and_step_and_pause(self, minSteps, timeout):
        res = await self._play_and_step(minSteps, timeout)
        execute("ToolbarPauseButtonClicked")
        return res

    # play timeline, run at least a minSteps number of physics steps and stop timeline
    async def play_and_step_and_stop(self, minSteps, timeout):
        res = await self._play_and_step(minSteps, timeout)
        execute("ToolbarStopButtonClicked")
        return res

    async def _play_and_step(self, minSteps, timeout):
        execute("ToolbarPlayButtonClicked")
        return await wait_for_physics_steps(minSteps, timeout / minSteps)
