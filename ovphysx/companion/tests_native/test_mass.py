# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2 AC-3
# @implements REQ-AUTHORING-MASS-001
# @covers AC-1 AC-2 AC-3

import ctypes as C
import math
import pytest

from ovphysx_companion.native import (
    MassShape, PrimitiveKind as Kind, compute_mass_properties, CompanionError,
    _MassResult, _MassShape, _acquire, Status,
)


def test_box_sphere_capsule_analytical(physics):
    box = compute_mass_properties(physics._lib, [MassShape(Kind.BOX, (1, 2, 3), 2)])
    assert box.mass == pytest.approx(96)
    assert box.volume == pytest.approx(48)
    assert sorted(box.diagonal_inertia) == pytest.approx([160, 320, 416])
    assert box.center_of_mass == (0, 0, 0)
    assert box.bounds_min == (-1, -2, -3)
    assert box.bounds_max == (1, 2, 3)
    sphere = compute_mass_properties(physics._lib, [MassShape(Kind.SPHERE, (2, 0, 0), 3)])
    assert sphere.mass == pytest.approx(32 * math.pi)
    assert sphere.diagonal_inertia == pytest.approx([sphere.mass * 1.6] * 3)
    cap = compute_mass_properties(physics._lib, [MassShape(Kind.CAPSULE, (2, 0, 0), 3)])
    assert cap.mass == pytest.approx(sphere.mass)
    assert cap.diagonal_inertia == pytest.approx(sphere.diagonal_inertia)
    cap = compute_mass_properties(physics._lib, [MassShape(Kind.CAPSULE, (1, 2, 0), 1)])
    assert cap.volume == pytest.approx(16 * math.pi / 3)
    assert sorted(cap.diagonal_inertia) == pytest.approx([38*math.pi/15, 213*math.pi/15, 213*math.pi/15])


def test_transforms_and_parallel_axis(physics):
    # Two unit-mass cubes at x=+/-2: Ix=4/3, Iy=Iz=4/3+8.
    shapes = [MassShape(Kind.BOX, (1, 1, 1), 0.125, (x, 0, 0)) for x in (-2, 2)]
    result = compute_mass_properties(physics._lib, shapes)
    assert result.mass == pytest.approx(2)
    assert result.center_of_mass == (0, 0, 0)
    assert sorted(result.diagonal_inertia) == pytest.approx([4/3, 4/3+8, 4/3+8])
    assert result.bounds_min == (-3, -1, -1)
    q = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
    rotated = compute_mass_properties(physics._lib, [MassShape(Kind.BOX, (1, 2, 3), 2, (5, 6, 7), q)])
    assert rotated.center_of_mass == (5, 6, 7)
    assert rotated.bounds_min == pytest.approx((3, 5, 4))
    # Reconstruct the tensor to test the principal frame, allowing equivalent axes.
    x, y, z, w = rotated.principal_axes
    rotation = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    tensor = [[sum(rotation[i][k]*rotated.diagonal_inertia[k]*rotation[j][k]
                   for k in range(3)) for j in range(3)] for i in range(3)]
    assert [tensor[i][i] for i in range(3)] == pytest.approx([320, 416, 160])
    assert max(abs(tensor[i][j]) for i in range(3) for j in range(3) if i != j) < 1e-3


@pytest.mark.parametrize('shape', [
    MassShape(Kind.BOX, (-1, 1, 1)), MassShape(Kind.BOX, (1e38, 1e38, 1e38)),
    MassShape(Kind.BOX, (1e-30, 1e-30, 1e-30)), MassShape(Kind.SPHERE, (float('nan'), 0, 0)),
    MassShape(Kind.CAPSULE, (1, -1, 0)), MassShape(Kind.BOX, (1, 1, 1), -1),
    MassShape(Kind.BOX, (1, 1, 1), rotation=(0, 0, 0, 0)),
    MassShape(Kind.BOX, (1, 1, 1), position=(float('inf'), 0, 0)),
])
def test_invalid_shape_diagnostics(physics, shape):
    with pytest.raises(CompanionError) as caught:
        compute_mass_properties(physics._lib, [MassShape(Kind.BOX, (1, 1, 1)), shape])
    assert caught.value.status == Status.INVALID_ARGUMENT
    assert caught.value.invalid_shape == 1


def test_failure_preserves_native_result(physics):
    api = _acquire(physics._lib)
    result = _MassResult()
    C.memset(C.byref(result), 0xA5, C.sizeof(result))
    before = bytes(result)
    shape = _MassShape()  # degenerate
    invalid = C.c_uint32()
    assert api.compute_primitive_mass(C.byref(shape), 1, C.sizeof(result), C.byref(result),
                                      C.byref(invalid)) == Status.INVALID_ARGUMENT
    assert invalid.value == 0
    assert bytes(result) == before
