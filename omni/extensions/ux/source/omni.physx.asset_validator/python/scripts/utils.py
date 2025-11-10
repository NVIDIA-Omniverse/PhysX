# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Usd, UsdUtils
from omni.asset_validator.core import BaseRuleChecker
import omni.timeline


def get_stage_id(stage: Usd.Stage):
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    if stage_id == -1:
        stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    return stage_id


def check_timeline_playing(checker: BaseRuleChecker, stage: Usd.Stage):
    if omni.timeline.get_timeline_interface().is_playing():
        checker._AddError(
            message="Timeline is playing",
            at=stage,
        )
        return True
    return False
