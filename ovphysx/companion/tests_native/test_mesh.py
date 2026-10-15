# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-AUTHORING-ELEMENTS-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4
from dataclasses import replace
import math
import pytest
from ovphysx_companion.mesh import embed_points, interpolate_elements
from ovphysx_companion.native import CompanionError
from ovphysx_companion.jobs import AuthoringService

VERTICES = ((0,0,0),(1,0,0),(0,1,0),(0,0,1))


def test_tetrahedron_containment_projection_ties_and_affine_interpolation(physics):
    mapping = embed_points(physics._lib, VERTICES, ((0,1,2,3),(0,1,2,3)),
                           ((.25,.25,.25),(2,0,0),(10,0,0)), element_width=4,
                           topology_version='tet-v1', max_distance=1)
    assert mapping.elements == (0, 0, -1)
    assert mapping.weights[0] == pytest.approx((.25,.25,.25,.25))
    assert mapping.distances[:2] == pytest.approx((0,1))
    assert math.isinf(mapping.distances[2])
    with pytest.raises(ValueError, match='Unmapped'):
        interpolate_elements(physics._lib, VERTICES, mapping, topology_version='tet-v1')
    one = replace(mapping, elements=mapping.elements[:1], weights=mapping.weights[:1],
                  distances=mapping.distances[:1], normal_offsets=mapping.normal_offsets[:1])
    moved = tuple(tuple(v*2+3 for v in point) for point in VERTICES)
    assert interpolate_elements(physics._lib, moved, one, topology_version='tet-v1')[0] == pytest.approx((3.5,3.5,3.5))
    with pytest.raises(ValueError, match='Stale'):
        interpolate_elements(physics._lib, moved, one, topology_version='tet-v2')


def test_triangle_signed_normal_offset_and_deformed_output(physics):
    mapping = embed_points(physics._lib, VERTICES[:3], ((0,1,2),), ((.25,.25,.5),),
                           element_width=3, topology_version='tri', max_distance=.5)
    assert mapping.weights[0] == pytest.approx((.5,.25,.25,0))
    assert mapping.normal_offsets == pytest.approx((.5,))
    assert interpolate_elements(physics._lib, VERTICES[:3], mapping, topology_version='tri')[0] == pytest.approx((.25,.25,.5))
    flipped = (VERTICES[0], VERTICES[2], VERTICES[1])
    assert interpolate_elements(physics._lib, flipped, mapping, topology_version='tri')[0] == pytest.approx((.25,.25,-.5))
    with pytest.raises(CompanionError):
        interpolate_elements(physics._lib, ((0,0,0),)*3, mapping, topology_version='tri')
    no_offset = replace(mapping, normal_offsets=(0,))
    assert interpolate_elements(physics._lib, ((0,0,0),)*3, no_offset, topology_version='tri') == ((0,0,0),)


@pytest.mark.parametrize('width,topology', [(3, ((0,0,1),)), (4, ((0,1,2,2),)), (4, ((0,1,2,99),))])
def test_element_diagnostics(physics, width, topology):
    with pytest.raises(CompanionError) as error:
        embed_points(physics._lib, VERTICES, topology, ((0,0,0),), element_width=width, topology_version='invalid')
    assert error.value.invalid_element == 0


def test_embedding_job_snapshots_and_owned_result(physics):
    vertices = [list(v) for v in VERTICES]
    with AuthoringService(physics._lib) as service:
        job = service.embed_points(vertices, ((0,1,2,3),), ((.25,.25,.25),), element_width=4, topology_version='v1')
        vertices.clear()
        result = job.result(timeout=5)
        assert result.complete
    assert result.weights[0] == pytest.approx((.25,.25,.25,.25))


def test_mesh_input_device_and_bad_weight_validation(physics):
    class Device:
        is_cuda = True
        def __iter__(self):
            raise AssertionError('Device array must not be read implicitly')
    with pytest.raises(TypeError, match='CPU'):
        embed_points(physics._lib, Device(), ((0,1,2),), (), element_width=3, topology_version='gpu')
    mapping = embed_points(physics._lib, VERTICES, ((0,1,2,3),), ((.25,.25,.25),), element_width=4, topology_version='v1')
    with pytest.raises(CompanionError):
        interpolate_elements(physics._lib, VERTICES, replace(mapping, weights=((1,1,1,1),)), topology_version='v1')


def test_native_validation_preserves_caller_arrays_and_table(physics):
    import ctypes as C
    from ovphysx_companion.mesh import _acquire, _MeshApi
    from ovphysx_companion.native import Status
    api = _acquire(physics._lib)
    vertices = (C.c_float*9)(0,0,0, 1,0,0, 0,1,0)
    indices = (C.c_uint32*3)(0,1,99)
    points = (C.c_float*3)(.25,.25,0)
    element = (C.c_int32*1)(123)
    weights, distances, offsets = (C.c_float*4)(1,2,3,4), (C.c_float*1)(123), (C.c_float*1)(456)
    invalid = C.c_uint32()
    assert api.embed(vertices,3,indices,1,3,points,1,0,element,weights,distances,offsets,C.byref(invalid)) == Status.INVALID_ARGUMENT
    assert invalid.value == 0
    assert tuple(element) == (123,) and tuple(weights) == (1,2,3,4)
    assert tuple(distances) == (123,) and tuple(offsets) == (456,)
    table = _MeshApi()
    C.memset(C.byref(table), 0xa5, C.sizeof(table))
    before = bytes(table)
    acquire = physics._lib.ovphysx_companion_mesh_acquire
    assert acquire(999, C.sizeof(table), C.byref(table)) == Status.ABI_MISMATCH
    assert bytes(table) == before


def test_normal_offset_velocity_matches_analytic_and_finite_difference(physics):
    import numpy as np
    from ovphysx_companion.mesh import interpolate_velocities
    points = np.asarray(VERTICES[:3], dtype=float)
    velocity = np.asarray(((0,0,0),(0,0,2),(0,0,0)), dtype=float)
    mapping = embed_points(physics._lib, points, ((0,1,2),), ((.25,.25,.5),),
                           element_width=3, topology_version='moving-triangle', max_distance=.5)
    actual = interpolate_velocities(physics._lib, points, velocity, mapping, topology_version='moving-triangle')
    assert actual[0] == pytest.approx((-1,0,.5), abs=1e-7)
    dt = 1e-3
    before = np.asarray(interpolate_elements(physics._lib, points-velocity*dt, mapping, topology_version='moving-triangle'))
    after = np.asarray(interpolate_elements(physics._lib, points+velocity*dt, mapping, topology_version='moving-triangle'))
    np.testing.assert_allclose(actual, (after-before)/(2*dt), atol=2e-5, rtol=2e-5)
    translated = interpolate_velocities(physics._lib, points, ((1,2,3),)*3, mapping, topology_version='moving-triangle')
    assert translated[0] == pytest.approx((1,2,3))
    with pytest.raises(CompanionError):
        interpolate_velocities(physics._lib, points*0, velocity, mapping, topology_version='moving-triangle')
    zero = replace(mapping, normal_offsets=(0.,))
    assert interpolate_velocities(physics._lib, points*0, velocity, zero, topology_version='moving-triangle')[0] == pytest.approx((0,0,.5))
    with pytest.raises(ValueError, match='Velocity count'):
        interpolate_velocities(physics._lib, points, velocity[:2], mapping, topology_version='moving-triangle')


def test_motion_table_rejection_preserves_canaries(physics):
    import ctypes as C
    from ovphysx_companion.mesh import _MotionApi
    from ovphysx_companion.native import Status
    table = _MotionApi()
    acquire = physics._lib.ovphysx_companion_mesh_motion_acquire
    acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_MotionApi)]
    acquire.restype = C.c_uint32
    C.memset(C.byref(table), 0xa5, C.sizeof(table))
    before = bytes(table)
    assert acquire(2, C.sizeof(table), C.byref(table)) == Status.ABI_MISMATCH
    assert bytes(table) == before
    assert acquire(1, C.sizeof(table)-1, C.byref(table)) == Status.ABI_MISMATCH
    assert bytes(table) == before
    assert acquire(1, C.sizeof(table), C.byref(table)) == Status.OK
    positions = (C.c_float*9)(0,0,0, 1,0,0, 0,1,0)
    velocities = (C.c_float*9)(*[float('nan')]*9)
    indices = (C.c_uint32*3)(0,1,2)
    elements = (C.c_int32*1)(0)
    weights = (C.c_float*4)(.5,.25,.25,0)
    offsets = (C.c_float*1)(.5)
    output = (C.c_float*3)(7,8,9)
    assert table.velocities(positions, velocities, 3, indices, 1, 3, elements, weights, offsets, 1, output) == Status.INVALID_ARGUMENT
    assert tuple(output) == (7,8,9)


def test_embedding_work_budget_rejects_before_allocating_result(physics):
    from ovphysx_companion.native import CompanionError, Status
    # Individually small arrays, but a pathological Cartesian search.
    with pytest.raises(CompanionError) as error:
        embed_points(physics._lib, VERTICES, ((0,1,2,3),)*4000,
                     ((.25,.25,.25),)*3000, element_width=4, topology_version='large')
    assert error.value.status == Status.INVALID_ARGUMENT
