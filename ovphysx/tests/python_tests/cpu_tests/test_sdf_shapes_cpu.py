# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""SDF shape evaluation on a CPU instance (ADR-0048): what PhysX.evaluate_sdf_shapes refuses there.

Evaluation needs a GPU instance, as the SDF view did; the selection is validated as for every session query.
"""

# @implements REQ-CAPI-SDF-001
# @covers AC-1 AC-4
# @maps_to TEST-CAPI-SDF-001

import numpy as np
import pytest
from test_utils import data_path, load_usd_with_ovstage

_NUL_PATH = "/World/Cube\x00GARBAGE"
_OVERSIZED_LITERAL = "/World/" + "a" * 65536


@pytest.fixture
def sdf_cube(physx_sdk):
    load_usd_with_ovstage(physx_sdk, data_path("sdf_cube.usda"))
    physx_sdk.wait_all()
    return physx_sdk


def _host_arrays():
    return np.zeros((1, 2, 3), dtype=np.float32), np.zeros((1, 2, 4), dtype=np.float32)


def test_evaluate_sdf_shapes_refuses_host_arrays(sdf_cube):
    points, out = _host_arrays()
    with pytest.raises(RuntimeError, match="must be on a CUDA device"):
        sdf_cube.evaluate_sdf_shapes(points, out, pattern="/World/Cube")


def test_evaluate_sdf_shapes_refuses_strided_arrays(sdf_cube):
    """A non-C-contiguous view is refused before any native call, whatever its device."""
    points = np.zeros((1, 4, 3), dtype=np.float32)[:, ::2, :]
    out = np.zeros((1, 2, 4), dtype=np.float32)
    assert not points.flags["C_CONTIGUOUS"]
    with pytest.raises(ValueError, match="C-contiguous"):
        sdf_cube.evaluate_sdf_shapes(points, out, pattern="/World/Cube")


def test_evaluate_sdf_shapes_converts_the_handoff_before_opening_the_query(sdf_cube):
    """A handoff that is not an integer raises before any query opens, so no query is left unreleased: here it
    raises ahead of the empty pattern, which only opening the query would reject."""
    points, out = _host_arrays()
    with pytest.raises(TypeError):
        sdf_cube.evaluate_sdf_shapes(points, out, pattern="", cuda_stream=object())
    with pytest.raises(TypeError):
        sdf_cube.evaluate_sdf_shapes(points, out, pattern="", cuda_wait_event=object())


def test_evaluate_sdf_shapes_validates_the_selection(sdf_cube):
    points, out = _host_arrays()
    with pytest.raises(ValueError):
        sdf_cube.evaluate_sdf_shapes(points, out, pattern="")
    with pytest.raises(RuntimeError, match="embedded NUL"):
        sdf_cube.evaluate_sdf_shapes(points, out, pattern=_NUL_PATH)
    with pytest.raises(RuntimeError, match="longer than"):
        sdf_cube.evaluate_sdf_shapes(points, out, pattern=_OVERSIZED_LITERAL)
