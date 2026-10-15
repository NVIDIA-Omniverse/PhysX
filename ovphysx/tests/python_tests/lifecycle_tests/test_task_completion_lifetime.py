# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5

"""Collected Python exceptions survive destruction of their originating instance."""

import pytest

from ovphysx import PhysX, TaskError
from test_utils import data_path, destroy_ovstage_test_attachments, load_usd_with_ovstage


def test_task_error_survives_instance_destruction():
    PhysX.set_cpu_mode(True)
    sdk = PhysX()
    try:
        load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
        sdk.wait_all()
        task = sdk.clone("/World/missing_source", ["/World/target"])
        with pytest.raises(TaskError) as caught:
            sdk.wait_task(task)
        errors = caught.value.errors
        assert errors[0][0] == task
        assert "missing_source" in errors[0][1]
    finally:
        destroy_ovstage_test_attachments(sdk)
        sdk.destroy()
    assert caught.value.errors == errors
    assert "missing_source" in str(caught.value)
    with pytest.raises(RuntimeError, match="has been destroyed"):
        sdk.wait_task(task)
