# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-AUTHORING-ELEMENTS-001
# @covers AC-5
import ctypes as C
from dataclasses import replace
from concurrent.futures import CancelledError
from threading import Event
import pytest
from ovphysx_companion.attachments import MeshEndpoint, propose_attachment, propose_filter
from ovphysx_companion.jobs import AuthoringService
from ovphysx_companion.mesh import triangle_filter_pairs, _FilterApi
from ovphysx_companion.native import CompanionError, Status

TET = ((0,0,0),(1,0,0),(0,1,0),(0,0,1))
TRI = TET[:3]


def test_proposal_endpoints_transforms_containment_and_cancellation(physics):
    # Nonuniform scale and shear are retained; query vertices use target-local
    # units after full affine conversion, not an inferred uniform scale.
    transform = ((2.,0.,0.,0.),(.5,3.,0.,0.),(0.,0.,4.,0.),(5.,6.,7.,1.))
    a = MeshEndpoint(('stage',7),'/A','a-v1',((.25,.25,.25),(2,0,0),(9,9,9)),(),3,transform,.01)
    b = MeshEndpoint(('stage',7),'/B','b-v2',TET,((0,1,2,3),),4,transform,.01)
    with AuthoringService(physics._lib) as service:
        result = service.propose_attachment(a,b,max_distance=1.001).result(5)
        assert result.source0 is a and result.source1 is b
        assert result.vertex_indices == (0,)
        assert result.coordinates[0] == pytest.approx((.25,.25,.25),abs=1e-6)
        assert result.mapping.weights[0] == pytest.approx((.25,)*4,abs=1e-6)
        assert result.projected_vertices == (1,) and result.unmatched_vertices == (2,)
        entered,release = Event(),Event()
        blocker = service._submit(lambda lib: (entered.set(),release.wait(5)))
        assert entered.wait(5)
        try:
            pending = service.propose_attachment(a,b)
            assert pending.cancel()
        finally:
            release.set()
        with pytest.raises(CancelledError):
            pending.result(5)
        blocker.result(5)
    assert result.mapping.topology_version == 'b-v2'
    assert b.points == TET
    projected = propose_attachment(physics._lib,a,b,max_distance=1.001,allow_projection=True)
    assert projected.vertex_indices == (0,1)
    with pytest.raises(CompanionError):
        propose_attachment(physics._lib,a,replace(b,topology=((0,1,2,2),)))
    with pytest.raises(ValueError,match='same source'):
        propose_attachment(physics._lib,a,replace(b,source=('stage',8)))


def test_filter_exact_triangle_distance_intersections_and_empty(physics):
    lib = physics._lib
    # Overlapping boxes, disjoint coplanar triangles: AABB overlap is insufficient.
    outside = ((.8,.8,0),(1.8,.8,0),(.8,1.8,0))
    assert triangle_filter_pairs(lib,TRI,((0,1,2),),outside,((0,1,2),),max_distance=.4) == ()
    assert triangle_filter_pairs(lib,TRI,((0,1,2),),outside,((0,1,2),),max_distance=.43) == ((0,0),)
    # Edge pierces the face interior; no vertex or edge-edge distance is zero.
    piercing = ((.2,.2,-1),(.2,.2,1),(2,2,1))
    assert triangle_filter_pairs(lib,TRI,((0,1,2),),piercing,((0,1,2),),max_distance=0) == ((0,0),)
    # Coplanar crossing, parallel faces and exact inclusive distance.
    crossing = ((-.1,.2,0),(1,.2,0),(.5,-.2,0))
    assert triangle_filter_pairs(lib,TRI,((0,1,2),),crossing,((0,1,2),),max_distance=0) == ((0,0),)
    above = tuple((x,y,z+.25) for x,y,z in TRI)
    assert triangle_filter_pairs(lib,TRI,((0,1,2),),above,((0,1,2),),max_distance=.25) == ((0,0),)
    a = MeshEndpoint(('stage',4),'/A','surface-A',TRI,((0,1,2),),3,meters_per_unit=.01)
    b = MeshEndpoint(('stage',4),'/B','surface-B',above,((0,1,2),(0,1,2)),3,meters_per_unit=.01)
    with AuthoringService(lib) as service:
        result = service.propose_filter(a,b,distance_meters=.0026).result(5)
    assert result.pairs == ((0,0),(0,1))
    assert result.groups0 == ((0,),(0,)) and result.groups1 == ((0,),(1,))
    assert result.source0.topology_version == 'surface-A'
    assert propose_filter(lib,a,b,distance_meters=.001).pairs == ()
    with pytest.raises(CompanionError):
        propose_filter(lib,a,b,distance_meters=.01,max_pairs=1)
    with pytest.raises(CompanionError):
        triangle_filter_pairs(lib,TRI,((0,0,1),),TRI,((0,1,2),),max_distance=0)


def test_filter_native_capacity_failure_and_abi_preserve_canaries(physics):
    acquire = physics._lib.ovphysx_companion_mesh_filter_acquire
    acquire.argtypes = [C.c_uint32,C.c_uint32,C.POINTER(_FilterApi)]
    acquire.restype = C.c_uint32
    api = _FilterApi()
    C.memset(C.byref(api),0xa5,C.sizeof(api))
    before = bytes(api)
    assert acquire(999,C.sizeof(api),C.byref(api)) == Status.ABI_MISMATCH
    assert bytes(api) == before
    assert acquire(1,C.sizeof(api),C.byref(api)) == Status.OK
    points = (C.c_float*9)(0,0,0,1,0,0,0,1,0)
    tris = (C.c_uint32*6)(0,1,2,0,1,2)
    output = (C.c_uint32*2)(123,456)
    count = C.c_uint32(789)
    assert api.pairs(points,3,tris,2,points,3,tris,2,0,1,output,C.byref(count)) == Status.INVALID_ARGUMENT
    assert tuple(output) == (123,456) and count.value == 789
    assert api.pairs(points,3,tris,2,points,3,tris,2,float('nan'),1,output,C.byref(count)) == Status.INVALID_ARGUMENT
    assert tuple(output) == (123,456) and count.value == 789


def test_pure_kernels_need_no_runtime_initialization(physics):
    import os,subprocess,sys
    code = '''import ctypes,sys
from ovphysx_companion.attachments import MeshEndpoint,propose_attachment,propose_filter
lib=ctypes.CDLL(sys.argv[1])
a=MeshEndpoint(('file',1),'/A','a',((.25,.25,.25),),(),3)
b=MeshEndpoint(('file',1),'/B','b',((0,0,0),(1,0,0),(0,1,0),(0,0,1)),((0,1,2,3),),4)
r=propose_attachment(lib,a,b)
assert r.mapping.weights == ((.25,.25,.25,.25),)
from ovphysx_companion.mesh import triangle_filter_pairs
assert triangle_filter_pairs(lib,(),(),(),(),max_distance=0) == ()
assert 'pxr' not in sys.modules
'''
    env = dict(os.environ,PYTHONPATH=os.pathsep.join(sys.path))
    subprocess.run([sys.executable,'-c',code,physics._lib._name],env=env,check=True,timeout=30)
