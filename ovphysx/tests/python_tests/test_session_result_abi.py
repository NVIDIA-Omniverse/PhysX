# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-6
# @maps_to TEST-PYTHON-TASK-001

"""Session result layouts must match the actual linked native library."""

import pytest
from ovphysx import _bindings
from test_abi_checks import _check_struct_layout


@pytest.mark.parametrize("name", [
    "read_attribute_result", "read_result", "write_attribute_result",
    "write_result", "fetched_write_group",
])
def test_session_result_native_layout(name):
    _check_struct_layout(
        name, getattr(_bindings, "ovphysx_" + name + "_t"),
        "ovphysx_internal_" + name + "_sizeof",
        "ovphysx_internal_" + name + "_alignof",
        "ovphysx_internal_" + name + "_offset",
    )
