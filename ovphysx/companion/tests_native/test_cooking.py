# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-COOK-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-DEFORM-COOK-001
# @covers AC-1 AC-2
import ctypes as C
from concurrent.futures import CancelledError
from threading import Event
import zipfile
import numpy as np
import pytest
from ovphysx_companion.cooking import CookingService, Cooked, Kind, _Api, _Mesh, _Options, _Info
from ovphysx_companion.native import CompanionError, Status


POINTS = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                   [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]], dtype=np.float32)
FACES = np.array([[0,3,2,1],[4,5,6,7],[0,1,5,4],[3,7,6,2],[0,4,7,3],[1,2,6,5]], dtype=np.int32)
TRIANGLES = np.array([[f[0],f[1],f[2]] for f in FACES] + [[f[0],f[2],f[3]] for f in FACES], dtype=np.int32)


@pytest.mark.parametrize('kind,options', [
    (Kind.TRIANGLE, {}), (Kind.CONVEX, {}),
    (Kind.DECOMPOSITION, dict(voxel_resolution=10000,max_hulls=4)),
    (Kind.SDF, dict(sdf_resolution=16,sdf_subgrid_resolution=4)),
    (Kind.SPHERES, dict(voxel_resolution=10000,seed_count=50,max_spheres=8))])
def test_raw_cooking_families_without_attach(physics,kind,options):
    with CookingService(physics) as service:
        assert kind in service.capabilities
        job=service.cook(kind,POINTS,FACES,**options)
        result=job.result(timeout=60)
        assert job.progress=='succeeded'
        assert result.kind==kind and result.blobs
        assert all(isinstance(blob,bytes) and len(blob)>0 for blob in result.blobs)
        assert result.sdk_build==service.sdk_build and len(result.sdk_build)==64
        if kind==Kind.TRIANGLE:
            assert result.points.shape==(8,3)
            assert result.indices.shape==(12,3)
            assert result.face_map.shape==(12,)
            assert sorted(set(result.face_map.tolist()))==list(range(6))
    assert result.blobs[0] # owned beyond the service
    assert not result.points.flags.writeable


def test_conforming_and_voxel_tetrahedra(physics):
    with CookingService(physics) as service:
        conforming=service.cook(Kind.CONFORMING_TETS,POINTS,TRIANGLES).result(timeout=60)
        assert conforming.indices.ndim==2 and conforming.indices.shape[1]==4
        assert len(conforming.points)>=8 and len(conforming.indices)>0
        assert conforming.indices.max()<len(conforming.points)
        voxel=service.cook(Kind.VOXEL_TETS,conforming.points,conforming.indices,voxel_resolution=4).result(timeout=60)
        assert voxel.indices.shape[1]==4 and len(voxel.indices)>0
        assert len(voxel.embedding)==len(conforming.points)
        assert np.all(voxel.embedding>=0) and np.all(voxel.embedding<len(voxel.indices))
        # Mesh generation must produce actual non-degenerate volumes.
        t=voxel.points[voxel.indices]
        volume=np.einsum('ij,ij->i',np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]),t[:,3]-t[:,0])/6
        assert np.all(np.abs(volume)>1e-8)


def test_scoped_cache_identity_invalidation_and_result_lifetime(physics):
    with CookingService(physics) as service:
        first=service.cook(Kind.TRIANGLE,POINTS,FACES).result()
        assert service.cook(Kind.TRIANGLE,POINTS,FACES).result() is first
        moved=POINTS.copy();moved[0,0]-=.25
        different=service.cook(Kind.TRIANGLE,moved,FACES).result()
        assert different.cache_key!=first.cache_key
        parameters=service.cook(Kind.TRIANGLE,POINTS,FACES,weld_tolerance=.001).result()
        assert parameters.cache_key!=first.cache_key
        service.invalidate(first.cache_key)
        assert service.cook(Kind.TRIANGLE,POINTS,FACES).result() is not first
        assert service.cook(Kind.TRIANGLE,moved,FACES).result() is different
        assert first.points[0,0]==-1
    assert len(first.blobs[0])>0


def test_cache_archive_compatibility_and_corruption(physics,tmp_path):
    with CookingService(physics) as service:
        result=service.cook(Kind.TRIANGLE,POINTS,FACES).result()
        path=tmp_path/'mesh.cooked'
        result.save(path)
        loaded=service.load(path,expected_key=result.cache_key)
        assert np.array_equal(loaded.indices,result.indices)
        assert loaded.blobs==result.blobs
        assert not loaded.indices.flags.writeable
        with pytest.raises(ValueError,match='SDK'):
            Cooked.load(path,sdk_build='wrong-build')
        with pytest.raises(ValueError,match='geometry/recipe'):
            service.load(path,expected_key='wrong-input')
        with zipfile.ZipFile(path) as source:
            entries={name:source.read(name) for name in source.namelist()}
        entries['blob-0']=b'corrupt'
        with zipfile.ZipFile(path,'w') as output:
            for name,data in entries.items():output.writestr(name,data)
        with pytest.raises(ValueError,match='Corrupt'):
            service.load(path)


def test_queued_cancellation_copied_inputs_and_cache_epoch(physics):
    service=CookingService(physics)
    entered,finish=Event(),Event()
    def block(library):
        entered.set()
        assert finish.wait(10)
    blocker=service._jobs._submit(block)
    try:
        assert entered.wait(5)
        points=POINTS.copy()
        cancelled=service.cook(Kind.TRIANGLE,points,FACES)
        kept=service.cook(Kind.TRIANGLE,points,FACES)
        service.invalidate(kept.cache_key) # in-flight outcome must not repopulate this key
        points[:]=100
        assert cancelled.progress=='queued'
        assert cancelled.cancel()
        assert cancelled.progress=='cancelled'
        with pytest.raises(CancelledError):cancelled.result()
        finish.set()
        result=kept.result(timeout=60)
        assert np.max(result.points)==1
        assert service._cache.get(kept.cache_key)[0] is None
        assert service.cook(Kind.TRIANGLE,POINTS,FACES).result() is not result
    finally:
        finish.set();service.close()


def test_result_buffer_canaries_and_service_independent_lease(physics):
    with CookingService(physics) as service:
        api=service._api
        options=_Options();assert api.defaults(Kind.TRIANGLE,C.byref(options))==Status.OK
        counts=np.full(6,4,dtype=np.int32)
        mesh=_Mesh(C.sizeof(_Mesh),1,8,24,6,1,POINTS.ctypes.data_as(C.POINTER(C.c_float)),
                   FACES.ctypes.data_as(C.POINTER(C.c_int32)),counts.ctypes.data_as(C.POINTER(C.c_int32)),1)
        handle=C.c_uint64()
        assert api.cook(service._handle.value,C.byref(mesh),C.byref(options),C.byref(handle))==Status.OK
    try:
        info=_Info();assert api.describe(handle.value,C.sizeof(info),C.byref(info))==Status.OK
        assert info.point_count==8
        data=(C.c_ubyte*4)(1,2,3,4);required=C.c_uint64()
        assert api.copy(handle.value,0,0,data,4,C.byref(required))==Status.INVALID_ARGUMENT
        assert bytes(data)==bytes([1,2,3,4]) and required.value==8*3*4
    finally:
        assert api.release(handle.value)==Status.OK
    assert api.release(handle.value)==Status.NOT_FOUND


@pytest.mark.parametrize('kind,change',[(Kind.CONVEX,{'max_hull_vertices':3}),(Kind.SDF,{'sdf_bits':7}),
    (Kind.CONVEX,{'min_thickness':-1}),(Kind.CONVEX,{'scale':(0,1,1)}),(Kind.TRIANGLE,{'weld_tolerance':float('nan')})])
def test_invalid_cooking_options_fail_without_result(physics,kind,change):
    with CookingService(physics) as service:
        job=service.cook(kind,POINTS,FACES,**change)
        with pytest.raises(CompanionError) as error:job.result()
        assert error.value.status==Status.INVALID_ARGUMENT
        assert job.progress=='failed'


def test_cooking_table_canaries(physics):
    table=_Api();C.memset(C.byref(table),0xa5,C.sizeof(table));before=bytes(table)
    acquire=physics._lib.ovphysx_companion_cooking_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    assert acquire(2,C.sizeof(table),C.byref(table))==Status.ABI_MISMATCH
    assert bytes(table)==before
    assert acquire(1,C.sizeof(table)-1,C.byref(table))==Status.ABI_MISMATCH
    assert bytes(table)==before


def test_volume_recipe_owns_distinct_meshes_and_visual_embedding(physics):
    from ovphysx_companion.deformable_cooking import cook_volume
    from ovphysx_companion.mesh import interpolate_elements
    with CookingService(physics) as service:
        body=cook_volume(service,POINTS,TRIANGLES,voxel_resolution=4).require_complete()
        assert body.simulation.cache_key!=body.collision.cache_key
        assert body.simulation.frame==body.collision.frame=='body-local'
        assert body.simulation.units=='stage' and body.simulation.meters_per_unit==1
        reconstructed=interpolate_elements(physics._lib,body.simulation.points,body.visual_mapping,
                                           topology_version=body.simulation.cache_key)
        np.testing.assert_allclose(reconstructed,POINTS,atol=1e-6)
    assert body.complete and len(body.simulation.indices)>0


def test_cooking_during_pending_preview_preserves_simulation(physics):
    import ovstage
    from pathlib import Path
    from ovphysx_companion.preview import PreviewSession
    stage=ovstage.Stage('cooking-safe-point')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),
                                    ordinal=1,domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with CookingService(physics) as service, PreviewSession(physics,stage,read_ordinal=1) as preview:
            preview.simulate(.01)
            assert service.cook(Kind.TRIANGLE,POINTS,FACES).result().blobs
            preview.fetch()
            assert service.cook(Kind.TRIANGLE,POINTS,FACES).result().blobs
    finally:
        stage.destroy()


def test_cooking_outlives_last_core_and_eviction_keeps_results(physics):
    import os
    from pathlib import Path
    import subprocess
    import sys
    companion=Path(__file__).resolve().parents[1]
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join([str(companion/'python'),str(companion/'samples'),env.get('PYTHONPATH','')])
    code='''
from ovphysx import PhysX
from ovphysx_companion.cooking import CookingService, Kind
from cooking import POINTS, FACES
PhysX.set_cpu_mode(True)
physics=PhysX()
service=CookingService(physics,cache_bytes=1)
physics.destroy()
try:
    first=service.cook(Kind.TRIANGLE,POINTS,FACES).result()
    second=service.cook(Kind.TRIANGLE,POINTS,FACES).result()
    assert second is not first
    assert service._cache.bytes==0
finally:
    service.close()
assert first.blobs==second.blobs and len(first.indices)==12
'''
    subprocess.run([sys.executable,'-c',code],check=True,capture_output=True,text=True,env=env,timeout=60)


@pytest.mark.parametrize('indices,faces', [([[0,1,90]],None), ([[0,1,2]],[4]), ([[0,1,2]],[2])])
def test_invalid_geometry_fails_without_native_result(physics,indices,faces):
    with CookingService(physics) as service:
        with pytest.raises(CompanionError) as error:
            service.cook(Kind.TRIANGLE,POINTS,indices,faces=faces).result()
        assert error.value.status==Status.INVALID_ARGUMENT


def test_simplified_cook_does_not_expose_source_triangles_as_overlay(physics):
    from ovphysx_companion.debug import collision_overlay
    with CookingService(physics) as service:
        result=service.cook(Kind.TRIANGLE,POINTS,FACES,triangle_mode=1,simplification=.2).result(60)
        assert result.blobs and not result.points.size and not result.indices.size and not result.face_map.size
        with pytest.raises(ValueError,match='surface|triangle'):collision_overlay(result,path='/Collider')
        with pytest.raises(TypeError,match='does not use'):service.cook(Kind.TRIANGLE,POINTS,FACES,max_spheres=0)
        assert service.cook(Kind.TRIANGLE,POINTS,FACES).result(60).indices.shape==(12,3)


def test_volume_recipe_reports_embedding_budget_before_mapping(physics):
    from types import SimpleNamespace
    from unittest.mock import patch
    from ovphysx_companion.deformable_cooking import cook_volume
    class Ready:
        def result(self):return SimpleNamespace(points=POINTS,indices=np.zeros((1_250_001,4),dtype=np.uint32),cache_key='budget')
    class Service:
        _library=None
        def cook(self,*args,**kwargs):return Ready()
    with patch('ovphysx_companion.deformable_cooking.embed_points',side_effect=AssertionError('mapping dispatched')):
        with pytest.raises(ValueError,match='10000000.*voxel_resolution'):cook_volume(Service(),POINTS,TRIANGLES)


def test_simplified_triangle_archive_roundtrip(physics,tmp_path):
    with CookingService(physics) as service:
        result=service.cook(Kind.TRIANGLE,POINTS,FACES,triangle_mode=1,simplification=.2).result(60)
        path=tmp_path/'simplified.cooked';result.save(path)
        loaded=service.load(path,expected_key=result.cache_key)
        assert loaded.blobs==result.blobs and loaded.recipe==result.recipe
        assert not loaded.points.size and not loaded.indices.size and not loaded.face_map.size
        loaded.save(path)
        assert service.load(path).blobs==result.blobs
