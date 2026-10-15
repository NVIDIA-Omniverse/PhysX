# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-BUFFERS-001
# @covers AC-1 AC-2 AC-3

# @implements REQ-COMP-JOBS-001
# @covers AC-1 AC-2 AC-4

# @implements REQ-COMP-ATTACH-COMPUTE-001
# @covers AC-1 AC-2 AC-3 AC-4

"""Stopped authoring jobs. Live-owner operations remain on the core scheduler."""

from concurrent.futures import CancelledError, ThreadPoolExecutor
from threading import Event, Lock
from weakref import WeakSet

from .native import compute_mass_properties, _snapshot_mass_shapes


class Job:
    """A retained copied input and one terminal outcome.

    result() delivers on the caller's thread. No user callback runs on a native
    completion thread. Accepted cancellation is visible immediately; service close
    still joins native work before releasing its library. Repeated result() calls
    return the same immutable outcome. This differs from core one-shot task waits.
    """

    def __init__(self):
        self._lock = Lock()
        self._ready = Event()
        self._future = None
        self._cancelled = False
        self._closed = False
        self._value = None
        self._error = None

    def _complete(self, future):
        with self._lock:
            if not self._cancelled and not self._closed:
                try:
                    self._value = future.result()
                except BaseException as error:
                    self._error = error
            self._future = None
            self._ready.set()

    def cancel(self):
        with self._lock:
            if self._ready.is_set() or self._future.done():
                return False
            self._cancelled = True
            future = self._future
            self._ready.set()
        # Completion may clear _future after unlocking; retain this exact future.
        # Future cancellation can synchronously call _complete; never hold lock.
        future.cancel()
        return True

    @property
    def done(self):
        return self._ready.is_set()

    def result(self, timeout=None):
        if not self._ready.wait(timeout):
            raise TimeoutError("Authoring job is still running")
        with self._lock:
            if self._cancelled or self._closed:
                raise CancelledError("Authoring job was cancelled or its service closed")
            if self._error is not None:
                raise self._error
            return self._value

    def _close(self):
        with self._lock:
            self._closed = True
            self._value = self._error = None
            self._ready.set()


class AuthoringService:
    """Parallel pure snapshot work using the caller's already loaded core CDLL.

    This owns no attachment and cannot step a scene. close() rejects submissions,
    suppresses undelivered outcomes, joins all work, then releases references.
    Submit/close are serialized internally; no owner lock is held during waits.
    """

    def __init__(self, library, *, workers=1):
        self._library = library
        self._executor = ThreadPoolExecutor(max_workers=workers, thread_name_prefix="companion-authoring")
        self._lock = Lock()
        self._jobs = WeakSet()
        self._closed = False
        self._drained = Event()

    def compute_mass_properties(self, shapes):
        # Copy before publishing: mutation of the host's list cannot race native
        # conversion. MassShape contains only values, never USD/native pointers.
        captured = _snapshot_mass_shapes(shapes)
        return self._submit(compute_mass_properties, captured)

    def compute_general_mass(self, request):
        from .mass import Request, compute_mass_properties
        if not isinstance(request, Request):
            raise TypeError("Pass an immutable mass Request")
        return self._submit(compute_mass_properties, request)

    def embed_points(self, vertices, topology, points, *, element_width, topology_version,
                     max_distance=0, frame='mesh-local', units='stage'):
        from .mesh import _points, _topology, embed_points
        from .native import _host_scalar
        captured = (_points(vertices), _topology(topology, element_width), _points(points))
        return self._submit(embed_points, *captured, element_width=element_width,
                            topology_version=topology_version, max_distance=_host_scalar(max_distance),
                            frame=frame, units=units)

    def propose_attachment(self, source0, source1, **options):
        from .attachments import MeshEndpoint, propose_attachment
        if not isinstance(source0, MeshEndpoint) or not isinstance(source1, MeshEndpoint):
            raise TypeError('Pass immutable MeshEndpoint records')
        return self._submit(propose_attachment, source0, source1, **options)

    def propose_filter(self, source0, source1, **options):
        from .attachments import MeshEndpoint, propose_filter
        if not isinstance(source0, MeshEndpoint) or not isinstance(source1, MeshEndpoint):
            raise TypeError('Pass immutable MeshEndpoint records')
        return self._submit(propose_filter, source0, source1, **options)

    def _submit(self, operation, *args, **kwargs):
        job = Job()
        with self._lock:
            if self._closed:
                raise RuntimeError("Authoring service is closed")
            future = self._executor.submit(operation, self._library, *args, **kwargs)
            job._future = future
            self._jobs.add(job)
        future.add_done_callback(job._complete)
        return job

    def close(self):
        with self._lock:
            first = not self._closed
            self._closed = True
            if first:
                jobs, self._jobs = tuple(self._jobs), WeakSet()
        if not first:
            self._drained.wait()
            return
        try:
            for job in jobs:
                job._close()
            self._executor.shutdown(wait=True, cancel_futures=True)
            self._library = None
        finally:
            self._drained.set()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
