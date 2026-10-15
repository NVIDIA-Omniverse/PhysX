# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PARTICLES-001
# @covers AC-1 AC-2 AC-4
"""Stopped CPU Poisson sampling with copied inputs and revision-scoped results."""
import ctypes as C
from dataclasses import dataclass
import math
import threading
import weakref
from .native import CompanionError,Status,_check,_host_scalar
from .mesh import _points,_topology,_flat


@dataclass(frozen=True)
class SamplingInput:
    """Source is an immutable host revision token, never a stage/native pointer."""
    source: object
    points: tuple
    triangles: tuple
    frame: str='world'
    meters_per_unit: float=1

    def __post_init__(self):
        hash(self.source)
        if self.source is None:raise ValueError('Declare a source identity and revision')
        if not isinstance(self.frame,str) or not self.frame:raise ValueError('Declare a Cartesian coordinate frame')
        scale=_host_scalar(self.meters_per_unit)
        if not math.isfinite(scale) or scale<=0:raise ValueError('meters_per_unit must be positive')
        object.__setattr__(self,'meters_per_unit',scale)
        object.__setattr__(self,'points',_points(self.points))
        object.__setattr__(self,'triangles',_topology(self.triangles,3))


@dataclass(frozen=True)
class Samples:
    source: object
    points: tuple
    triangles: tuple
    weights: tuple
    frame: str
    meters_per_unit: float
    radius: float
    volume_radius: float
    units: str='stage'


class StaleSamplingError(RuntimeError):pass


class _Options(C.Structure):
    _fields_=[('radius',C.c_float),('volume_radius',C.c_float),('max_samples',C.c_uint32),('attempts',C.c_uint32)]


class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
        ('open',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(C.c_uint64))),
        ('close',C.CFUNCTYPE(C.c_uint32,C.c_uint64)),
        ('sample',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(C.c_float),C.c_uint32,C.POINTER(C.c_uint32),C.c_uint32,
                    C.POINTER(_Options),C.c_uint32,C.POINTER(C.c_float),C.POINTER(C.c_int32),C.POINTER(C.c_float),C.POINTER(C.c_uint32)))]


def _execute(library,api,handle,request,options):
    vertices=_flat(request.points,C.c_float);topology=_flat(request.triangles,C.c_uint32)
    points=(C.c_float*(3*options.max_samples))();triangles=(C.c_int32*options.max_samples)()
    weights=(C.c_float*(3*options.max_samples))();count=C.c_uint32()
    _check(api.sample(handle,vertices,len(request.points),topology,len(request.triangles),C.byref(options),C.sizeof(options),
                      points,triangles,weights,C.byref(count)),'Poisson sampling')
    def triples(values):return tuple(tuple(values[3*i:3*i+3]) for i in range(count.value))
    return Samples(request.source,triples(points),tuple(triangles[:count.value]),triples(weights),request.frame,
                   request.meters_per_unit,options.radius,options.volume_radius)


class SamplingJob:
    def __init__(self,job,sampler,epoch):self._job,self._sampler,self._epoch=job,sampler,epoch
    @property
    def done(self):return self._job.done
    def cancel(self):return self._job.cancel()
    def result(self,timeout=None):
        value=self._job.result(timeout)
        with self._sampler._lock:
            if self._sampler._closed or self._epoch!=self._sampler._epoch:
                raise StaleSamplingError('Sampling input was invalidated, resampled or released')
        return value


class Sampler:
    """One source revision; resample replaces input and invalidates old delivery.

    There is no hidden native source cache. Every request recomputes from its copy.
    invalidate() suppresses old results immediately. close() cancels delivery and drains all this sampler's work;
    SamplingService closes every sampler before releasing runtime services.
    Host source changes/deletion call invalidate/close; USD application independently
    validates its EditSession revision even if a result was already delivered.
    """
    def __init__(self,service,request):
        if not isinstance(request,SamplingInput):raise TypeError('Pass SamplingInput')
        self._service,self._request=service,request
        self._lock=threading.Lock();self._closed=False;self._epoch=0;self._job=None;self._jobs=weakref.WeakSet()

    def resample(self,request=None,*,radius=.1,volume_radius=0,max_samples=100000,attempts=30):
        if request is not None and not isinstance(request,SamplingInput):raise TypeError('Pass SamplingInput')
        if any(isinstance(v,bool) or not isinstance(v,int) for v in (max_samples,attempts)) or not 1<=max_samples<=1000000 or not 1<=attempts<=100:
            raise ValueError('max_samples must be 1..1000000 and attempts 1..100')
        radius,volume_radius=_host_scalar(radius),_host_scalar(volume_radius)
        if not math.isfinite(radius) or radius<=0 or not math.isfinite(volume_radius) or volume_radius<0:
            raise ValueError('Expected positive surface radius and nonnegative volume radius')
        options=_Options(radius,volume_radius,max_samples,attempts)
        with self._lock:
            if self._closed:raise RuntimeError('Sampler is closed')
            if self._job is not None:self._job.cancel()
            self._epoch+=1
            if request is not None:self._request=request
            self._job=self._service._submit(self._request,options)
            self._jobs.add(self._job)
            return SamplingJob(self._job,self,self._epoch)

    def invalidate(self):
        with self._lock:
            self._epoch+=1
            if self._job is not None:self._job.cancel()

    def close(self):
        with self._lock:
            self._closed=True;self._epoch+=1;self._request=None
            jobs=tuple(self._jobs)
        # Wait without holding sampler/service/owner locks. Includes superseded
        # work whose cooperative cancellation could only suppress delivery.
        for job in jobs:
            with job._lock:future=job._future
            job.cancel();job._close()
            if future is not None:
                try:future.result()
                except Exception:pass

    def __enter__(self):return self
    def __exit__(self,*args):self.close()


class SamplingService:
    """CPU surface/volume provider sharing the existing stopped cooking lease.

    close() invalidates all samplers, suppresses results, drains native work, and
    releases services. Already copied results remain valid data. No attachment or
    simulation is created. Surface/volume radii use input stage units; volume input
    must be a closed oriented two-manifold without intersecting shells. Sampling
    order/random seed is not a serialization guarantee. Other postprocess operations
    have independent capabilities and report Unavailable in this version.
    """
    def __init__(self,physics,*,workers=1):
        from .cooking import CookingService
        self._lease=None;self._lock=threading.Lock();self._samplers=weakref.WeakSet();self._closed=False
        api=_Api()
        try:acquire=physics._lib.ovphysx_companion_particles_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'particle table') from error
        acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
        _check(acquire(1,C.sizeof(api),C.byref(api)),'particle table')
        if api.size!=C.sizeof(api) or api.version!=1 or not all((api.open,api.close,api.sample)):
            raise CompanionError(Status.ABI_MISMATCH,'particle layout')
        self._api=api
        self.capabilities=frozenset(name for bit,name in enumerate(('surface','volume','smoothing','anisotropy','isosurface')) if api.capabilities&(1<<bit))
        self._lease=CookingService(physics,workers=workers,cache_bytes=0)

    def require(self,operation,*,backend='cpu'):
        if backend!='cpu' or operation not in self.capabilities:
            raise CompanionError(Status.UNAVAILABLE,f'particle {operation}/{backend}')

    def sampler(self,request):
        with self._lock:
            if self._closed:raise RuntimeError('Sampling service is closed')
            sampler=Sampler(self,request);self._samplers.add(sampler);return sampler

    def _submit(self,request,options):
        with self._lock:
            if self._closed:raise RuntimeError('Sampling service is closed')
            self.require('volume' if options.volume_radius else 'surface')
            return self._lease._jobs._submit(_execute,self._api,self._lease._handle.value,request,options)

    def close(self):
        with self._lock:
            self._closed=True;samplers=tuple(self._samplers)
        for sampler in samplers:sampler.close()
        if self._lease is not None:self._lease.close()

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def __del__(self):
        if not getattr(self,'_closed',True):
            try:self.close()
            except Exception:pass # The cooking lease retains itself on failed close.
