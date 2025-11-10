# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
__all__ = ["BackwardCompatibilityChecker"]

from omni.asset_validator.core import BaseRuleChecker, registerRule, Suggestion
from pxr import Usd
from .. import get_physx_asset_validator_interface
from .utils import get_stage_id, check_timeline_playing


@registerRule("Omni:Physx")
class BackwardCompatibilityChecker(BaseRuleChecker):
    __doc__ = """
    Check all physics backward compatibility rules for the given stage.
    """

    def CheckStage(self, stage: Usd.Stage):
        if check_timeline_playing(self, stage):
            return

        validator = get_physx_asset_validator_interface()
        if validator.backward_compatibility_check(get_stage_id(stage)):
            log = validator.get_backward_compatibility_log()

            self._AddFailedCheck(
                message=f'{log}',
                at=stage.GetPrimAtPath("/"),
                suggestion=Suggestion(
                    message='Do backward compatibility fixes',
                    callable=self.run_fix,
                    at=[stage.GetRootLayer()],
                ),
            )

    def run_fix(self, stage: Usd.Stage, _: Usd.Prim):
        validator = get_physx_asset_validator_interface()
        validator.run_backward_compatibility_check(get_stage_id(stage))
