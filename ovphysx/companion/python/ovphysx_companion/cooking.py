# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-COOK-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-COMP-DEFORM-COOK-001
# @covers AC-1 AC-2
# @implements REQ-COMP-BUFFERS-001
# @covers AC-1 AC-2 AC-3
"""Copied CPU cooking jobs and scoped compatible-result caches; no USD imports."""
import atexit
from collections import OrderedDict
import ctypes as C
from dataclasses import dataclass
from enum import IntEnum
import hashlib
import json
import threading
import warnings
import weakref
import zipfile

from .jobs import AuthoringService
from .mesh import _points
from .native import CompanionError, Status, _check


class Kind(IntEnum):
    CONVEX = 1
    TRIANGLE = 2
    DECOMPOSITION = 3
    SDF = 4
    SPHERES = 5
    CONFORMING_TETS = 6
    VOXEL_TETS = 7


class _Mesh(C.Structure):
    _fields_ = [('size', C.c_uint32), ('version', C.c_uint32),
                ('point_count', C.c_uint32), ('index_count', C.c_uint32),
                ('face_count', C.c_uint32), ('right_handed', C.c_uint32),
                ('points', C.POINTER(C.c_float)), ('indices', C.POINTER(C.c_int32)),
                ('faces', C.POINTER(C.c_int32)), ('meters_per_unit', C.c_double)]


class _Options(C.Structure):
    _fields_ = [(name, C.c_uint32) for name in (
        'size', 'version', 'kind', 'triangle_mode', 'max_hull_vertices', 'max_hulls',
        'voxel_resolution', 'shrink_wrap', 'sdf_resolution', 'sdf_subgrid_resolution',
        'sdf_bits', 'sdf_remesh', 'max_spheres', 'seed_count', 'sphere_mode', 'reserved')]
    _fields_ += [(name, C.c_float) for name in (
        'simplification', 'weld_tolerance', 'min_thickness', 'error_percentage',
        'sdf_band', 'sdf_margin', 'sdf_reduction')]
    _fields_ += [('scale', C.c_float * 3)]


class _Info(C.Structure):
    _fields_ = [(name, C.c_uint32) for name in (
        'size', 'version', 'kind', 'warnings', 'point_count', 'index_count',
        'element_width', 'face_map_count', 'embedding_count', 'blob_count')]
    _fields_ += [('recipe_hash', C.c_uint64 * 2), ('sdk_build', C.c_char * 65)]


class _Api(C.Structure):
    _fields_ = [('size', C.c_uint32), ('version', C.c_uint32), ('capabilities', C.c_uint64),
                ('sdk_build', C.c_char_p),
                ('defaults', C.CFUNCTYPE(C.c_uint32, C.c_uint32, C.POINTER(_Options))),
                ('open', C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.POINTER(C.c_uint64))),
                ('close', C.CFUNCTYPE(C.c_uint32, C.c_uint64)),
                ('cook', C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.POINTER(_Mesh), C.POINTER(_Options), C.POINTER(C.c_uint64))),
                ('describe', C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.c_uint32, C.POINTER(_Info))),
                ('copy', C.CFUNCTYPE(C.c_uint32, C.c_uint64, C.c_uint32, C.c_uint32,
                                    C.c_void_p, C.c_uint64, C.POINTER(C.c_uint64))),
                ('release', C.CFUNCTYPE(C.c_uint32, C.c_uint64))]


def _array(data, dtype, width=1):
    import numpy as np
    result = np.frombuffer(data, dtype=dtype)
    return result.reshape(-1, width) if width != 1 else result


@dataclass(frozen=True)
class Cooked:
    """Owned result, readable after service close. Arrays are backed by immutable bytes.

    Blob layouts are SDK-specific. `sdk_build` plus format_version must match
    before reuse; never treat native object addresses as serialized geometry.
    Triangle face_map refers to original polygon indices; embedding names the
    voxel tetrahedron containing each input vertex. A value of -1 is unmatched.
    """
    kind: Kind
    sdk_build: str
    cache_key: str
    recipe: str
    warnings: int
    points: object
    indices: object
    face_map: object
    embedding: object
    blobs: tuple
    format_version: int = 1

    @property
    def frame(self):
        return json.loads(self.recipe)['frame']

    @property
    def units(self):
        return json.loads(self.recipe)['units']

    @property
    def meters_per_unit(self):
        return json.loads(self.recipe)['meters_per_unit']

    @property
    def byte_size(self):
        return sum(a.nbytes for a in (self.points, self.indices, self.face_map, self.embedding)) + sum(map(len, self.blobs))

    def save(self, path):
        """Write a versioned, checksummed data archive with no pickle/native pointers."""
        arrays = {'points': self.points.tobytes(), 'indices': self.indices.tobytes(),
                  'face_map': self.face_map.tobytes(), 'embedding': self.embedding.tobytes()}
        arrays.update({f'blob-{i}': data for i, data in enumerate(self.blobs)})
        metadata = dict(format=1, kind=int(self.kind), sdk_build=self.sdk_build, cache_key=self.cache_key,
                        recipe=self.recipe, warnings=self.warnings, width=self.indices.shape[1] if self.indices.ndim == 2 else 0,
                        blob_count=len(self.blobs), checksums={name: hashlib.sha256(data).hexdigest() for name, data in arrays.items()})
        with zipfile.ZipFile(path, 'w', compression=zipfile.ZIP_STORED) as archive:
            archive.writestr('metadata.json', json.dumps(metadata, sort_keys=True))
            for name, data in arrays.items():
                archive.writestr(name, data)

    @classmethod
    def load(cls, path, *, sdk_build, expected_key=None):
        """Reject mismatched format/SDK/input recipes and corrupt data before reuse."""
        with zipfile.ZipFile(path) as archive:
            metadata = json.loads(archive.read('metadata.json'))
            if metadata['format'] != 1 or metadata['sdk_build'] != sdk_build:
                raise ValueError('Incompatible cooked-result format or SDK build; recook the input')
            if expected_key is not None and metadata['cache_key'] != expected_key:
                raise ValueError('Cooked result does not match this geometry/recipe')
            kind = Kind(metadata['kind'])
            width = metadata['width']
            if width not in (0, 3, 4):
                raise ValueError('Invalid cooked topology width')
            names = ['points', 'indices', 'face_map', 'embedding'] + [f'blob-{i}' for i in range(metadata['blob_count'])]
            arrays = {}
            for name in names:
                data = archive.read(name)
                if hashlib.sha256(data).hexdigest() != metadata['checksums'][name]:
                    raise ValueError('Corrupt cooked-result data')
                arrays[name] = data
        result = cls(kind, sdk_build, metadata['cache_key'], metadata['recipe'], metadata['warnings'],
                   _array(arrays['points'], '<f4', 3), (_array(arrays['indices'], '<u4', width) if width else _array(arrays['indices'], '<u4').reshape(0,0)),
                   _array(arrays['face_map'], '<u4'), _array(arrays['embedding'], '<i4'),
                   tuple(arrays[f'blob-{i}'] for i in range(metadata['blob_count'])))
        import numpy as np
        recipe = json.loads(result.recipe)
        triangle_mode = recipe.get('options', {}).get('triangle_mode', 0)
        triangle_invalid = kind == Kind.TRIANGLE and (
            triangle_mode not in (0, 1) or
            (triangle_mode == 0 and (width != 3 or len(result.face_map) != len(result.indices))) or
            (triangle_mode == 1 and (width != 0 or any((result.points.size, result.indices.size,
                                                     result.face_map.size, result.embedding.size)))))
        if (recipe.get('format') != 1 or recipe.get('sdk_build') != sdk_build or recipe.get('kind') != kind or
                not all(isinstance(recipe.get(name), str) and recipe[name] for name in ('frame', 'units')) or
                not np.isfinite(recipe.get('meters_per_unit', 0)) or recipe['meters_per_unit'] <= 0 or
                not np.all(np.isfinite(result.points)) or
                (result.indices.size and (not len(result.points) or result.indices.max() >= len(result.points))) or
                (kind in (Kind.CONFORMING_TETS, Kind.VOXEL_TETS) and (width != 4 or not result.indices.size)) or
                triangle_invalid or
                (kind < Kind.CONFORMING_TETS and not result.blobs) or
                np.any(result.embedding < -1) or np.any(result.embedding >= len(result.indices))):
            raise ValueError('Invalid cooked-result geometry or recipe')
        return result


class _Cache:
    def __init__(self, byte_limit):
        if isinstance(byte_limit, bool) or not isinstance(byte_limit, int) or byte_limit < 0:
            raise ValueError('cache_bytes must be a nonnegative integer')
        self.limit, self.bytes = byte_limit, 0
        self.entries = OrderedDict()
        self.epochs = {}
        self.generation = 0
        self.lock = threading.Lock()

    def get(self, key):
        with self.lock:
            result = self.entries.get(key)
            if result is not None:
                self.entries.move_to_end(key)
            return result, (self.generation, self.epochs.get(key, 0))

    def put(self, key, epoch, result):
        with self.lock:
            if epoch != (self.generation, self.epochs.get(key, 0)) or result.byte_size > self.limit:
                return
            previous = self.entries.pop(key, None)
            if previous is not None:
                self.bytes -= previous.byte_size
            self.entries[key] = result
            self.bytes += result.byte_size
            while self.bytes > self.limit:
                _, removed = self.entries.popitem(last=False)
                self.bytes -= removed.byte_size

    def invalidate(self, key=None):
        with self.lock:
            if key is None:
                self.entries.clear()
                self.epochs.clear()
                self.bytes = 0
                self.generation += 1
            else:
                removed = self.entries.pop(key, None)
                if removed is not None:
                    self.bytes -= removed.byte_size
                self.epochs[key] = self.epochs.get(key, 0) + 1


class CookingJob:
    def __init__(self, job, cache, key, epoch):
        self._job, self._cache, self.cache_key, self._epoch = job, cache, key, epoch

    @property
    def progress(self):
        """Coarse job phase, not an invented percentage of a native kernel."""
        with self._job._lock:
            if self._job._cancelled:
                return 'cancelled'
            if self._job._closed:
                return 'closed'
            if self._job._ready.is_set():
                return 'failed' if self._job._error is not None else 'succeeded'
            return 'running' if self._job._future.running() else 'queued'

    @property
    def done(self):
        return self._job.done

    def cancel(self):
        return self._job.cancel()

    def result(self, timeout=None):
        result = self._job.result(timeout)
        # Cancelled/closed/failed work never reaches this cache publication.
        self._cache.put(self.cache_key, self._epoch, result)
        return result


def _integers(value):
    import numpy as np
    if hasattr(value, '__cuda_array_interface__') or getattr(value, 'is_cuda', False):
        raise TypeError('Cooking requires ready CPU topology')
    data = np.asarray(value)
    if data.dtype.kind not in 'iu' or data.size > 2**32-1:
        raise ValueError('Topology must contain integer indices')
    if data.size and (data.min() < 0 or data.max() > 2**31-1):
        raise ValueError('Topology index outside int32 range')
    return np.array(data, dtype=np.int32, order='C', copy=True)


def _execute(library, api, handle, points, indices, faces, right_handed, meters_per_unit, options, key, recipe):
    mesh = _Mesh(C.sizeof(_Mesh), 1, len(points), indices.size, len(faces) if faces is not None else 0,
                 int(right_handed), points.ctypes.data_as(C.POINTER(C.c_float)),
                 indices.ctypes.data_as(C.POINTER(C.c_int32)),
                 faces.ctypes.data_as(C.POINTER(C.c_int32)) if faces is not None else None, meters_per_unit)
    result = C.c_uint64()
    _check(api.cook(handle, C.byref(mesh), C.byref(options), C.byref(result)), 'cook')
    try:
        info = _Info()
        _check(api.describe(result.value, C.sizeof(info), C.byref(info)), 'describe cooking result')
        def copied(kind, index=0):
            required = C.c_uint64()
            _check(api.copy(result.value, kind, index, None, 0, C.byref(required)), 'cooking buffer size')
            data = (C.c_ubyte * required.value)()
            _check(api.copy(result.value, kind, index, data, required.value, C.byref(required)), 'copy cooking buffer')
            return bytes(data)
        return Cooked(Kind(info.kind), info.sdk_build.decode('ascii'), key, recipe, info.warnings,
                      _array(copied(0), '<f4', 3), _array(copied(1), '<u4', info.element_width or 1),
                      _array(copied(2), '<u4'), _array(copied(3), '<i4'),
                      tuple(copied(4, i) for i in range(info.blob_count)))
    finally:
        _check(api.release(result.value), 'release cooking result')


_live = weakref.WeakSet()
_retained = []
_registered = False


def _cleanup():
    for service in list(_live) + list(_retained):
        try:
            service.close()
        except Exception as error:
            warnings.warn(f'Cooking service retained after failed close: {error}', RuntimeWarning)


class CookingService:
    """Stopped CPU cooking over existing runtime services, with owned job inputs.

    Dispatch at a runtime safe point; Busy is an explicit conflict, not a queued
    simulation mutation. This owner can outlive the bootstrap PhysX object. close
    suppresses undelivered work, joins running kernels, then releases its service.
    Existing Cooked records remain valid. Cache invalidation never clears the
    runtime's global cache or destroys CUDA contexts.
    """
    def __init__(self, physics, *, workers=1, cache_bytes=64*1024*1024):
        global _registered
        self._handle = C.c_uint64()
        self._lifecycle_acquired = False
        self._lock = threading.Lock()
        self._library = physics._lib
        self._cache = _Cache(cache_bytes)
        self._closed = False
        self._jobs = None
        self._api = _Api()
        try:
            acquire = self._library.ovphysx_companion_cooking_acquire
        except AttributeError as error:
            raise CompanionError(Status.UNAVAILABLE, 'cooking table') from error
        acquire.argtypes = [C.c_uint32, C.c_uint32, C.POINTER(_Api)]
        acquire.restype = C.c_uint32
        _check(acquire(1, C.sizeof(_Api), C.byref(self._api)), 'acquire cooking table')
        if self._api.size != C.sizeof(_Api) or self._api.version != 1 or not self._api.sdk_build or not all(
                (self._api.defaults, self._api.open, self._api.close, self._api.cook,
                 self._api.describe, self._api.copy, self._api.release)):
            raise CompanionError(Status.ABI_MISMATCH, 'cooking table')
        handle = getattr(physics, '_omni_physx_sdk_handle', None)
        if not handle or not handle.value:
            raise CompanionError(Status.NOT_FOUND, 'cooking bootstrap')
        self.sdk_build = self._api.sdk_build.decode('ascii')
        self.capabilities = frozenset(kind for kind in Kind if self._api.capabilities & (1 << kind))
        from ovphysx.api import _acquire_process_lifecycle, _release_process_lifecycle
        self._release_lifecycle = _release_process_lifecycle
        _acquire_process_lifecycle(self)
        try:
            _check(self._api.open(handle.value, C.byref(self._handle)), 'open cooking service')
            self._jobs = AuthoringService(self._library, workers=workers)
        except BaseException:
            if self._handle.value:
                self._api.close(self._handle.value)
                self._handle.value = 0
            self._lifecycle_acquired = False
            self._release_lifecycle()
            raise
        _live.add(self)
        if not _registered:
            atexit.register(_cleanup)
            _registered = True

    def cook(self, kind, points, indices, *, faces=None, right_handed=True, meters_per_unit=1,
             frame='mesh-local', units='stage', **parameters):
        import numpy as np
        from .native import _host_scalar
        if isinstance(kind, str):
            kind = Kind[kind.upper()]
        if isinstance(kind, bool):
            raise TypeError('Expected a cooking Kind')
        kind = Kind(kind)
        if kind not in self.capabilities:
            raise CompanionError(Status.UNAVAILABLE, kind.name)
        if not isinstance(right_handed, bool):
            raise TypeError('right_handed must be bool')
        if not all(isinstance(value, str) and value for value in (frame, units)):
            raise ValueError('Declare a coordinate frame and length units')
        meters_per_unit = _host_scalar(meters_per_unit)
        points = np.array(_points(points), dtype=np.float32).reshape(-1, 3)
        topology = _integers(indices)
        if kind in (Kind.CONFORMING_TETS, Kind.VOXEL_TETS):
            width = 3 if kind == Kind.CONFORMING_TETS else 4
            if topology.ndim != 2 or topology.shape[1] != width or faces is not None:
                raise ValueError(f'{kind.name} requires (N,{width}) elements and no polygon faces')
            counts = None
        else:
            if faces is None:
                if topology.ndim != 2 or topology.shape[1] < 3:
                    raise ValueError('Provide polygon face counts for flat topology')
                counts = np.full(len(topology), topology.shape[1], dtype=np.int32)
            else:
                counts = _integers(faces).reshape(-1)
        topology = topology.reshape(-1)
        options = _Options()
        _check(self._api.defaults(kind, C.byref(options)), 'cooking defaults')
        allowed = dict(_Options._fields_[3:-1])
        recipe_fields = {
            Kind.TRIANGLE:{'triangle_mode','simplification','weld_tolerance'},
            Kind.CONVEX:{'max_hull_vertices','min_thickness','scale'},
            Kind.DECOMPOSITION:{'max_hull_vertices','max_hulls','voxel_resolution','error_percentage','min_thickness','shrink_wrap','scale'},
            Kind.SDF:{'triangle_mode','simplification','weld_tolerance','sdf_resolution','sdf_subgrid_resolution','sdf_bits','sdf_band','sdf_margin','sdf_remesh','sdf_reduction'},
            Kind.SPHERES:{'max_spheres','seed_count','voxel_resolution','sphere_mode','scale'},
            Kind.CONFORMING_TETS:set(),Kind.VOXEL_TETS:{'voxel_resolution','scale'},
        }[kind]
        allowed={name:value for name,value in allowed.items() if name in recipe_fields}
        for name in parameters:
            if name not in recipe_fields:raise TypeError(f'{kind.name} does not use cooking option {name}')
        for name, value in parameters.items():
            if name == 'scale':
                value = tuple(value)
                if len(value) != 3:
                    raise ValueError('scale requires three components')
                options.scale[:] = [_host_scalar(v) for v in value]
            elif name not in allowed:
                raise TypeError(f'Unknown cooking option {name}')
            elif allowed[name] is C.c_uint32:
                if name in ('shrink_wrap', 'sdf_remesh') and isinstance(value, bool):
                    value = int(value)
                if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**32:
                    raise ValueError(f'{name} requires a uint32')
                setattr(options, name, value)
            else:
                setattr(options, name, _host_scalar(value))
        recipe = json.dumps(dict(format=1, sdk_build=self.sdk_build, kind=int(kind), right_handed=right_handed,
            frame=frame, units=units,
            meters_per_unit=meters_per_unit, options={name: list(getattr(options, name)) if name=='scale' else getattr(options, name)
                                                   for name in sorted(recipe_fields)}), sort_keys=True, separators=(',', ':'))
        digest = hashlib.sha256(recipe.encode())
        for array in (points, topology, counts):
            data = b'' if array is None else array.tobytes()
            digest.update(len(data).to_bytes(8, 'little'))
            digest.update(data)
        key = digest.hexdigest()
        with self._lock:
            if self._closed:
                raise RuntimeError('Cooking service is closed')
            cached, epoch = self._cache.get(key)
            if cached is None:
                job = self._jobs._submit(_execute, self._api, self._handle.value, points, topology, counts,
                                         right_handed, meters_per_unit, options, key, recipe)
            else:
                job = self._jobs._submit(lambda library: cached)
            return CookingJob(job, self._cache, key, epoch)

    def invalidate(self, cache_key=None):
        """Invalidate one recipe, or this service's whole cache when omitted."""
        self._cache.invalidate(cache_key)

    def load(self, path, *, expected_key=None):
        return Cooked.load(path, sdk_build=self.sdk_build, expected_key=expected_key)

    def close(self):
        with self._lock:
            self._closed = True
        if self._jobs is not None:
            self._jobs.close()
        with self._lock:
            if self._handle.value:
                _check(self._api.close(self._handle.value), 'close cooking service')
                self._handle.value = 0
            self._cache.invalidate()
            _live.discard(self)
            if self._lifecycle_acquired:
                self._lifecycle_acquired = False
                self._release_lifecycle()

    def __enter__(self):
        if self._closed:
            raise RuntimeError('Cooking service is closed')
        return self

    def __exit__(self, *args):
        self.close()

    def __del__(self):
        if getattr(self, '_handle', None) is not None and (self._handle.value or self._lifecycle_acquired):
            try:
                self.close()
            except Exception:
                _retained.append(self)
