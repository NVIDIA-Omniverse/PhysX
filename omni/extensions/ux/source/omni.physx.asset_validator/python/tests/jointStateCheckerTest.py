# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.asset_validator.core.tests import ValidationRuleTestCase, IsAFailure
from omni.physxtests import utils as test_utils
from omni.physx.scripts.assets_paths import AssetFolders
import carb
import carb.tokens
from pxr import Usd, PhysxSchema
import omni.usd

from ..scripts.jointStateChecker import JointStateChecker

class JointStateCheckerTestCase(ValidationRuleTestCase):

    def _get_asset_path(self, asset: str):
        return self.asset_paths[asset]

    async def setUp(self):
        super().setUp()
        self._test_data_dir = carb.tokens.get_tokens_interface().resolve("${omni.physx.asset_validator}/testdata")

    async def tearDown(self):
        pass

    @staticmethod
    def get_random_word(length: int) -> str:
        """Generates random ascii lowercase word of 'length' characters."""
        import random
        import string

        letters = string.ascii_lowercase + string.digits
        return "".join(random.choice(letters) for _ in range(length))

    async def test_joint_state_checker(self):

        # This avoids "You are setting a per-stage physics settings /physics/updateToUsd without an active stage."
        # It happens because Asset validator passes in a stage that is not set to the current omni.usd context
        await omni.usd.get_context().new_stage_async()

        # Download the asset locally, so we can modify it (later)
        temp_usd_file = test_utils.TmpFileFromSrc(
            f"{self._test_data_dir}/JointStateValidator",
            "JointAssetValidator.usda",
            "_JointAssetValidator.usd",
            random_prefix=True)

        prim_path = "/World/articulation"

        # Test 1: Fix Existing issues
        self.assertRule(
            url=temp_usd_file.get_path(),
            rule=JointStateChecker,
            asserts=[
                IsAFailure(
                    f'Joint State for "{prim_path}" is not coherent with transforms of rigid bodies belonging to the articulation'
                ),
            ],
        )

        # Fix all issues with the suggestion provided by JointStateChecker
        self.assertSuggestion(url=temp_usd_file.get_path(), rule=JointStateChecker, predicate=None)

        # Test 2: Apply a joint state that will not cause error and check it
        stage: Usd.Stage = Usd.Stage.Open(temp_usd_file.get_path())

        joint1: Usd.Prim = stage.GetPrimAtPath("/World/articulation/revoluteJoint")
        jointStateAPI = PhysxSchema.JointStateAPI.Apply(joint1, "angular")
        jointStateAPI.CreatePositionAttr().Set(0.0)
        Usd.Stage.Save(stage)

        # Check that there are no failures
        self.assertRule(url=temp_usd_file.get_path(), rule=JointStateChecker, asserts=[])

        temp_usd_file.delete()
        omni.usd.get_context().close_stage()
