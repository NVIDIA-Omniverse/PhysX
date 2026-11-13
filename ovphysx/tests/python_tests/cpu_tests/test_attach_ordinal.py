# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-ATTACH-001
# @covers AC-3
# @implements REQ-CAPI-OVSTAGE-UPDATE-001
# @covers AC-5

import ovstage
import pytest


@pytest.mark.parametrize(
    ("ordinal", "error"),
    [
        (0, ValueError),
        (-1, ValueError),
        (-(1 << 64), ValueError),
        (ovstage.ORDINAL_MIN, ValueError),
        (ovstage.ORDINAL_MAX + 1, ValueError),
        ((1 << 64) - 1, ValueError),
        (1 << 64, ValueError),
        ((1 << 64) + 1, ValueError),
        (1.5, TypeError),
        ("1", TypeError),
    ],
)
def test_invalid_attach_ordinal_preserves_stage_for_retry(physx_sdk, ordinal, error):
    with ovstage.Stage("attach-ordinal") as stage:
        stage.advance_write_floor(ordinal=1).wait()
        try:
            with pytest.raises(error):
                physx_sdk.wait_task(physx_sdk.attach_ovstage(stage, read_ordinal=ordinal))
            assert not physx_sdk.is_stage_attached()
            physx_sdk.wait_task(physx_sdk.attach_ovstage(stage, read_ordinal=1))
            assert physx_sdk.is_stage_attached(stage)
        finally:
            physx_sdk.wait_task(physx_sdk.detach_ovstage())


def test_update_ranges_below_one_are_consumed_and_int64_bounded(physx_sdk):
    with ovstage.Stage("update-ordinal") as stage:
        stage.advance_write_floor(ordinal=1).wait()
        physx_sdk.wait_task(physx_sdk.attach_ovstage(stage, read_ordinal=1))
        try:
            # Ordinals below 1 precede every attach ordinal: wholly or partly below 1, these
            # ranges hold nothing unread and succeed as no-ops.
            for start, end in ((ovstage.ORDINAL_MIN, -1), (-5, 0), (0, 1), (ovstage.ORDINAL_MIN, 1)):
                physx_sdk.wait_task(physx_sdk.update_from_ovstage(start, end))
            for start, end in ((ovstage.ORDINAL_MIN - 1, 1), (1, ovstage.ORDINAL_MAX + 1)):
                with pytest.raises(ValueError, match="int64"):
                    physx_sdk.update_from_ovstage(start, end)
            assert physx_sdk.is_stage_attached(stage)
        finally:
            physx_sdk.wait_task(physx_sdk.detach_ovstage())
