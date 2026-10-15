# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-AUTHORING-POINTS-001
# @covers AC-1 AC-2 AC-3

import ctypes as C
import pytest
from ovphysx_companion.native import match_points, CompanionError, Status, _acquire


def test_nearest_inclusive_ties_empty_and_signed_zero(physics):
    targets = [(1, 0, 0), (-1, 0, 0), (3, 0, 0)]
    source = [(0, 0, 0), (2.9, 0, 0), (10, 0, 0), (-1, 0, 0)]
    assert match_points(physics._lib, source, targets, 1) == (0, 2, -1, 1)
    assert match_points(physics._lib, [(0, 0, 0)], [(-0.0, 0, 0), (0, 0, 0)], 0) == (0,)
    assert match_points(physics._lib, [], targets, 0) == ()
    assert match_points(physics._lib, source, [], 0) == (-1, -1, -1, -1)


def test_invalid_and_failure_atomicity(physics):
    for distance in (-1, float('nan'), float('inf')):
        with pytest.raises(CompanionError):
            match_points(physics._lib, [], [], distance)
    with pytest.raises(CompanionError):
        match_points(physics._lib, [(float('nan'), 0, 0)], [], 0)
    api = _acquire(physics._lib)
    source = (C.c_float * 3)(float('inf'), 0, 0)
    result = (C.c_int32 * 1)(123)
    assert api.match_points(source, 1, None, 0, 1, result, 1) == Status.INVALID_ARGUMENT
    assert result[0] == 123


def test_device_data_requires_explicit_transfer(physics):
    class Device:
        is_cuda = True
        def __iter__(self):
            raise AssertionError('device data must not be iterated')
    with pytest.raises(TypeError, match='CPU'):
        match_points(physics._lib, Device(), [], 0)
    with pytest.raises(TypeError, match='CPU'):
        match_points(physics._lib, [(Device(), 0, 0)], [], 0)
