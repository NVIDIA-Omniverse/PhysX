# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.undo
from omni.physx.scripts import physicsUtils
from omni.physxtests import utils
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase

approximations = ["none", "convexHull", "convexDecomposition", "meshSimplification", "convexMeshSimplification"]


class AsyncTestCase(PhysicsKitStageAsyncTestCase):
    async def base_apply_command_test(self, applyCommandFunc, expStats1, expStats2, expStats3,
                                      removeCommandFunc=None, expStatsRem=None):
        stage = await utils.new_stage_setup()

        utils.execute_and_check(self, "AddPhysicsScene", stage=stage, path="/physicsScene")

        stepper = utils.PhysicsStepper()

        # code path 1: create geom, apply command, play and pause
        cubePrim = physicsUtils.add_cube(stage, "/cubeActor1")
        cubePath = cubePrim.GetPath()
        applyCommandFunc(cubePath)
        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))

        utils.check_stats(self, expStats1)

        # code path 2: play and pause, create geom, apply command, play and pause
        spherePrim = physicsUtils.add_sphere(stage, "/sphereActor1")
        spherePath = spherePrim.GetPath()
        applyCommandFunc(spherePath)

        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))

        utils.check_stats(self, expStats2)

        # code path 3: play and pause, create geom, play and pause, apply command, play and pause
        capsulePrim = physicsUtils.add_capsule(stage, "/capsuleActor1")
        capsulePath = capsulePrim.GetPath()
        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))
        applyCommandFunc(capsulePath)
        self.assertTrue(await stepper.play_and_step_and_pause(1, 5))

        utils.check_stats(self, expStats3)

        # apply remove
        if removeCommandFunc is not None and expStatsRem is not None:
            removeCommandFunc(cubePath)
            removeCommandFunc(spherePath)
            removeCommandFunc(capsulePath)

            self.assertTrue(await stepper.play_and_step_and_pause(1, 5))
            utils.check_stats(self, expStatsRem)

    async def base_undoredo_command_test(self, applyCommandFunc):
        stage = await utils.new_stage_setup()

        cubePrim = physicsUtils.add_cube(stage, "/cubeActor1")
        cubePath = cubePrim.GetPath()

        # apply command
        applyCommandFunc(cubePath)

        # delete prim
        utils.execute_and_check(self, "DeletePrims", paths=[cubePath])

        # undo delete, undo apply, redo apply
        omni.kit.undo.undo()
        omni.kit.undo.undo()
        omni.kit.undo.redo()

    async def base_basic_userpath_command_test(self, applyCommandFunc, expStats):
        stage = await utils.new_stage_setup()
        utils.execute_and_check(self, "AddPhysicsScene", stage=stage, path="/physicsScene")
        prim_path = "/cube"
        utils.execute_and_check(self, "CreatePrimWithDefaultXform", prim_type="Cube", prim_path=prim_path)
        applyCommandFunc(prim_path)
        await utils.play_and_step_and_pause(self, 5, 4.0 / 6.0)
        utils.check_stats(self, expStats)
