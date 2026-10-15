# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-AUTHORING-MASS-001
# @covers AC-1 AC-2 AC-3 AC-4
import ctypes as C
from dataclasses import replace
import math
import pytest
from ovphysx_companion.mass import Shape,ShapeKind,Request,Overrides,compute_mass_properties,_Api,_Overrides,_Shape
from ovphysx_companion.native import CompanionError,Status,_MassResult
from ovphysx_companion.jobs import AuthoringService

POINTS=((-1,-2,-3),(1,-2,-3),(1,2,-3),(-1,2,-3),(-1,-2,3),(1,-2,3),(1,2,3),(-1,2,3))
TRIANGLES=((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7))

def mesh(**kwargs):return Shape('/Mesh',ShapeKind.MESH,density=10,points=POINTS,triangles=TRIANGLES,**kwargs)
def box(**kwargs):return Shape('/Box',ShapeKind.BOX,(1,2,3),10,**kwargs)

def tensor(properties):
    x,y,z,w=properties.principal_axes
    r=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),(2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),(2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
    return tuple(sum(r[i][k]*properties.diagonal_inertia[k]*r[j][k] for k in range(3)) for i in range(3) for j in range(3))

def test_closed_mesh_matches_analytic_box_and_winding(physics):
    expected=compute_mass_properties(physics._lib,Request((box(),),'revision','/Body'))
    assert expected.properties.mass==pytest.approx(480)
    assert tensor(expected.properties)==pytest.approx((2080,0,0,0,1600,0,0,0,800))
    for triangles in (TRIANGLES,tuple(tuple(reversed(t)) for t in TRIANGLES)):
        result=compute_mass_properties(physics._lib,Request((replace(mesh(),triangles=triangles),),'revision','/Body'))
        assert result.properties.mass==pytest.approx(480)
        assert result.properties.volume==pytest.approx(48)
        assert tensor(result.properties)==pytest.approx(tensor(expected.properties),abs=.001)
        assert result.properties.bounds_min==(-1,-2,-3) and result.properties.bounds_max==(1,2,3)
        assert result.source=='revision' and result.collider_paths==('/Mesh',) and result.units=='SI'

def test_mesh_transform_compound_overrides_and_job_copy(physics):
    q=(0,0,math.sqrt(.5),math.sqrt(.5))
    shaped=mesh(position=(5,6,7),rotation=q,overrides=Overrides(mass=240))
    result=compute_mass_properties(physics._lib,Request((shaped,))).properties
    assert result.mass==pytest.approx(240)
    assert result.center_of_mass==pytest.approx((5,6,7))
    assert result.bounds_min==pytest.approx((3,5,4))
    assert tensor(result)==pytest.approx((800,0,0,0,1040,0,0,0,400),abs=.001)
    shifted=compute_mass_properties(physics._lib,Request((box(),),overrides=Overrides(center=(1,0,0)))).properties
    assert tensor(shifted)==pytest.approx((2080,0,0,0,2080,0,0,0,1280))
    overridden=compute_mass_properties(physics._lib,Request((box(),),overrides=Overrides(mass=20,center=(2,3,4),diagonal=(1,2,3),axes=q))).properties
    assert overridden.mass==20 and overridden.center_of_mass==(2,3,4)
    assert tensor(overridden)==pytest.approx((2,0,0,0,1,0,0,0,3),abs=1e-5)
    source=[box(),replace(mesh(),position=(2,0,0))]
    request=Request(source,('source',2),'/Body');source.clear()
    with AuthoringService(physics._lib) as service:
        outcome=service.compute_general_mass(request).result(10)
    assert outcome.properties.mass==960 and outcome.properties.center_of_mass==(1,0,0)
    assert tensor(outcome.properties)==pytest.approx((4160,0,0,0,4160,0,0,0,2560),abs=.001)

@pytest.mark.parametrize('triangles',[TRIANGLES[:-1],TRIANGLES+(TRIANGLES[0],),((0,0,1),)+TRIANGLES[1:],(tuple(reversed(TRIANGLES[0])),)+TRIANGLES[1:]])
def test_mesh_topology_failure_identifies_collider(physics,triangles):
    with pytest.raises(CompanionError) as caught:
        compute_mass_properties(physics._lib,Request((box(),replace(mesh(),triangles=triangles))))
    assert caught.value.invalid_shape==1 and caught.value.object_path=='/Mesh'

def test_mass_failure_preserves_abi_outputs(physics):
    acquire=physics._lib.ovphysx_companion_mass_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    api=_Api();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    result=_MassResult();C.memset(C.byref(result),0xa5,C.sizeof(result));before=bytes(result)
    shape=_Shape();shape.size=C.sizeof(shape);body=_Overrides();index=C.c_uint32()
    assert api.compute(C.byref(shape),1,C.sizeof(body),C.byref(body),C.sizeof(result),C.byref(result),C.byref(index))==Status.INVALID_ARGUMENT
    assert bytes(result)==before and index.value==0


def test_general_mass_requires_no_runtime_in_fresh_worker():
    import os,subprocess,sys,ovphysx_companion
    env=dict(os.environ);env['PYTHONPATH']=str(__import__('pathlib').Path(ovphysx_companion.__file__).parents[1])
    code='''
import ctypes,os,sys
from ovphysx_companion.mass import *
lib=ctypes.CDLL(os.environ['OVPHYSX_LIB'])
r=compute_mass_properties(lib,Request((Shape('/Box',ShapeKind.BOX,(1,2,3),10),)))
assert r.properties.mass==480
assert 'ovphysx' not in sys.modules and 'pxr' not in sys.modules and 'ovstage' not in sys.modules
'''
    result=subprocess.run([sys.executable,'-c',code],env=env,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stdout+result.stderr


def test_mesh_disconnected_and_flat_volume_rejected(physics):
    disconnected=replace(mesh(),points=POINTS+tuple((x+10,y,z) for x,y,z in POINTS),
        triangles=TRIANGLES+tuple(tuple(i+8 for i in t) for t in TRIANGLES))
    flat=replace(mesh(),points=tuple((x,y,0) for x,y,z in POINTS))
    for shape in (disconnected,flat):
        with pytest.raises(CompanionError) as caught:compute_mass_properties(physics._lib,Request((shape,)))
        assert caught.value.object_path=='/Mesh'
