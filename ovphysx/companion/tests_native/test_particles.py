# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PARTICLES-001
# @covers AC-1 AC-2 AC-4
# @implements REQ-NATIVE-SAMPLING-001
# @covers AC-1 AC-2 AC-3
import ctypes as C
from concurrent.futures import CancelledError
from dataclasses import replace
from threading import Event,Thread
import numpy as np
import pytest
from ovphysx_companion.particles import SamplingInput,SamplingService,StaleSamplingError,_Api,_Options
from ovphysx_companion.native import CompanionError,Status
from ovphysx_companion.preview import PreviewSession
from ovphysx_companion.mesh import _flat

POINTS=((-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1))
TRIANGLES=((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7))


def test_surface_volume_mapping_bounds_and_capabilities(physics):
    request=SamplingInput(('cube',1),POINTS,TRIANGLES)
    with SamplingService(physics) as service,service.sampler(request) as sampler:
        assert service.capabilities=={'surface','volume'}
        for operation in ('smoothing','anisotropy','isosurface'):
            with pytest.raises(CompanionError) as error:service.require(operation)
            assert error.value.status==Status.UNAVAILABLE
        with pytest.raises(CompanionError):service.require('surface',backend='gpu')
        result=sampler.resample(radius=.3,max_samples=1000).result(30)
        assert 20<len(result.points)<1000
        p=np.asarray(result.points);t=np.asarray(result.triangles);w=np.asarray(result.weights)
        assert t.min()>=0 and t.max()<12
        assert np.allclose(w.sum(axis=1),1,atol=1e-5)
        assert np.allclose((np.asarray(POINTS)[np.asarray(TRIANGLES)[t]]*w[:,:,None]).sum(axis=1),p,atol=1e-5)
        distances=np.linalg.norm(p[:,None,:]-p[None,:,:],axis=2);np.fill_diagonal(distances,np.inf)
        assert distances.min()>=.2999
        volume=sampler.resample(radius=.4,volume_radius=.4,max_samples=1000).result(30)
        interior=np.asarray(volume.triangles)==-1
        assert interior.any() and np.all(np.asarray(volume.weights)[interior]==0)
        assert np.max(np.abs(np.asarray(volume.points)))<=1.00001
        assert len(sampler.resample(radius=.3,max_samples=10).result(30).points)<=10
    assert result.frame=='world' and result.units=='stage' and result.meters_per_unit==1
    assert result.source==('cube',1) and len(result.points)>20


def test_sampling_invalid_geometry_and_abi_canaries(physics):
    acquire=physics._lib.ovphysx_companion_particles_acquire
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    api=_Api();C.memset(C.byref(api),0xa5,C.sizeof(api));before=bytes(api)
    assert acquire(2,C.sizeof(api),C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api)-1,C.byref(api))==Status.ABI_MISMATCH and bytes(api)==before
    assert acquire(1,C.sizeof(api),C.byref(api))==Status.OK
    handle=C.c_uint64();assert api.open(physics._omni_physx_sdk_handle.value,C.byref(handle))==Status.OK
    try:
        points=_flat(POINTS,C.c_float);triangles=_flat(TRIANGLES,C.c_uint32)
        options=_Options(.3,0,20,30);out=(C.c_float*60)(*[123]*60);ids=(C.c_int32*20)(*[321]*20);weights=(C.c_float*60)(*[456]*60);count=C.c_uint32(789)
        before=(bytes(out),bytes(ids),bytes(weights),count.value)
        options.radius=-1
        assert api.sample(handle.value,points,8,triangles,12,C.byref(options),C.sizeof(options),out,ids,weights,C.byref(count))==Status.INVALID_ARGUMENT
        assert (bytes(out),bytes(ids),bytes(weights),count.value)==before
    finally:assert api.close(handle.value)==Status.OK
    with SamplingService(physics) as service:
        for request,opts in ((SamplingInput(('bad',1),POINTS,((0,0,1),)),{}),
                             (SamplingInput(('open',1),POINTS,TRIANGLES[:-1]),{'volume_radius':.2}),
                             (SamplingInput(('nan',1),((float('nan'),0,0),*POINTS[1:]),TRIANGLES),{})):
            with service.sampler(request) as sampler:
                with pytest.raises(CompanionError) as error:sampler.resample(**opts).result(30)
                assert error.value.status==Status.INVALID_ARGUMENT


def test_resample_revision_cancel_and_release_drain(physics):
    # Hold the single worker to deterministically exercise pending invalidation.
    service=SamplingService(physics);entered=Event();release=Event()
    blocker=service._lease._jobs._submit(lambda _: (entered.set(),release.wait(10)))
    assert entered.wait(5)
    request=SamplingInput(('cube',1),POINTS,TRIANGLES)
    sampler=service.sampler(request);old=sampler.resample(radius=.3,max_samples=500)
    current=sampler.resample(replace(request,source=('cube',2),points=tuple((p[0]+5,p[1],p[2]) for p in POINTS)),radius=.3,max_samples=500)
    release.set();blocker.result(10)
    with pytest.raises((CancelledError,StaleSamplingError)):old.result(30)
    result=current.result(30);assert result.source==('cube',2) and min(p[0] for p in result.points)>=3.99999
    sampler.invalidate()
    with pytest.raises(StaleSamplingError):current.result()
    # A running copied operation cannot outlive service teardown.
    entered.clear();release.clear();closed=Event()
    # Use an authoring operation with the same drain owner, without replacing C callbacks.
    pending=service._lease._jobs._submit(lambda _: (entered.set(),release.wait(10)))
    assert entered.wait(5)
    thread=Thread(target=lambda: (service.close(),closed.set()));thread.start()
    assert not closed.wait(.05)
    release.set();thread.join(5);assert closed.is_set()
    with pytest.raises(CancelledError):pending.result()
    assert result.points


def test_sampling_during_pending_preview_preserves_simulation(physics):
    from pathlib import Path
    import ovstage
    stage=ovstage.Stage('sampling-pending')
    try:
        ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),ordinal=1,
                                    domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        with SamplingService(physics) as service,service.sampler(SamplingInput(('cube',1),POINTS,TRIANGLES)) as sampler:
            with PreviewSession(physics,stage,read_ordinal=1) as preview:
                preview.simulate(.01)
                assert sampler.resample(radius=.3,max_samples=100).result(30).points
                preview.fetch()
                assert sampler.resample(radius=.3,max_samples=100).result(30).points
    finally:stage.destroy()


def test_volume_sampling_caps_all_correspondence_arrays(physics):
    with SamplingService(physics) as service,service.sampler(SamplingInput(('bounded',1),POINTS,TRIANGLES)) as sampler:
        result=sampler.resample(radius=.3,volume_radius=.4,max_samples=10).result(30)
        assert 0<len(result.points)<=10
        assert len(result.points)==len(result.triangles)==len(result.weights)
        for point,triangle,weights in zip(result.points,result.triangles,result.weights):
            if triangle>=0:
                assert np.sum(np.asarray(POINTS)[np.asarray(TRIANGLES[triangle])]*np.asarray(weights)[:,None],axis=0)==pytest.approx(point,abs=1e-5)
            else:assert weights==(0,0,0)


@pytest.mark.parametrize('mirrored',[False,True])
def test_inward_volume_input_is_rejected_not_empty_success(physics,mirrored):
    points=tuple((-p[0],p[1],p[2]) for p in POINTS) if mirrored else POINTS
    triangles=TRIANGLES if mirrored else tuple((a,c,b) for a,b,c in TRIANGLES)
    with SamplingService(physics) as service,service.sampler(SamplingInput(('inward',1),points,triangles)) as sampler:
        with pytest.raises(CompanionError) as error:sampler.resample(radius=.3,volume_radius=.4).result(30)
        assert error.value.status==Status.INVALID_ARGUMENT
        surface=sampler.resample(radius=.3).result(30)
        assert surface.points
        for point,triangle,weights in zip(surface.points,surface.triangles,surface.weights):
            assert np.sum(np.asarray(points)[np.asarray(triangles[triangle])]*np.asarray(weights)[:,None],axis=0)==pytest.approx(point,abs=1e-5)
