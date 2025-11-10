# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.asset_validator.core
from omni.asset_validator.core import FixStatus
from omni.asset_validator.core.engine import ValidationEngine
from omni.physxtests.utils.physicsBase import PhysicsMemoryStageBaseAsyncTestCase, TestCategory
import os
from pxr import Usd


class ExpectedResult:
    def __init__(self, message=None, code=None, expect_suggestion=False, suggestion_message=None, issues_num=1):
        self.message = message
        self.code = code
        self.suggestion_message = suggestion_message
        self.issues_num = issues_num


class BaseValidatorTest(PhysicsMemoryStageBaseAsyncTestCase):
    category = TestCategory.Kit

    # list of rules to run, override in your inherited class or pass as param to validation helpers below
    rules = []

    # enable in your inherited class to print results - messages from results can include parsing errors from python code ran in the rules!
    debug_results = False

    async def open_usd(self, filename):
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, "../../../../testdata/")))
        schema_folder = schema_folder.replace("\\", "/") + "/"
        stage = await self.new_stage(attach_stage=False, file_to_load=schema_folder + filename + ".usda")
        return stage

    def create_validation_engine(self, rules=None):
        self._engine = ValidationEngine(initRules=False)
        if rules is None:
            rules = self.rules
        for rule in rules:
            self._engine.enable_rule(rule)

    def run_validation_and_assert_result(self, expected_result: ExpectedResult):
        results = self._engine.validate(self._stage)

        if self.debug_results:
            print(results.issues())

        if expected_result and expected_result.issues_num > 0:
            self.assertEqual(len(results.issues()), expected_result.issues_num)
            if expected_result.message:
                self.assertEqual(results.issues()[0].message, expected_result.message)
            if expected_result.suggestion_message:
                self.assertIsNotNone(results.issues()[0].suggestion)
                self.assertEqual(results.issues()[0].suggestion.message, expected_result.suggestion_message)
            if expected_result.code:
                self.assertEqual(results.issues()[0].code, expected_result.code)
        else:
            self.assertFalse(results.issues())

        return results

    def run_validation_fix(self, results):
        fixer = omni.asset_validator.core.IssueFixer(self._stage)
        results = fixer.fix_at(results.issues(), self._stage.GetRootLayer())
        for result in results:
            self.assertEqual(result.status, FixStatus.SUCCESS, msg=result.exception)

    def validate_and_check_and_fix(self, expected_result: ExpectedResult, rules=None):
        self.create_validation_engine(rules)
        results = self.run_validation_and_assert_result(expected_result)
        self.run_validation_fix(results)
        self.run_validation_and_assert_result(None)

    async def open_usd_and_validate_and_check_and_fix(self, name, expected_result: ExpectedResult, rules=None):
        stage = await self.open_usd(name)
        self.set_stage(stage)
        self.validate_and_check_and_fix(expected_result, rules)

    def create_stage_in_memory(self):
        stage = Usd.Stage.CreateInMemory()
        self.assertTrue(stage)
        self.set_stage(stage)
        return stage
