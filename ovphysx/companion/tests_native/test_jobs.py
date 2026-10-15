# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-JOBS-001
# @covers AC-1 AC-2 AC-4

from concurrent.futures import CancelledError
from threading import Event, Thread

import pytest
from ovphysx_companion.jobs import AuthoringService
from ovphysx_companion.native import MassShape, PrimitiveKind, CompanionError
import ovphysx_companion.jobs as jobs


def test_real_mass_job_retains_input_and_terminal_result(physics):
    shapes = [MassShape(PrimitiveKind.BOX, (1, 1, 1), 1)]
    with AuthoringService(physics._lib, workers=2) as service:
        job = service.compute_mass_properties(shapes)
        shapes.clear()
        result = job.result(timeout=5)
        assert result.mass == pytest.approx(8)
        assert job.result() is result
        assert not job.cancel()
        invalid = service.compute_mass_properties([MassShape(PrimitiveKind.SPHERE, (0, 0, 0))])
        with pytest.raises(CompanionError):
            invalid.result(timeout=5)
    with pytest.raises(CancelledError):
        job.result()
    with pytest.raises(RuntimeError, match='closed'):
        service.compute_mass_properties([MassShape(PrimitiveKind.BOX, (1, 1, 1))])


def test_cancel_running_and_queued_work_and_close_drains(physics, monkeypatch):
    started, finish, drained = Event(), Event(), Event()
    def controlled(library, shapes):
        started.set()
        assert finish.wait(5)
        return 'must not deliver'
    monkeypatch.setattr(jobs, 'compute_mass_properties', controlled)
    service = AuthoringService(physics._lib)
    running = service.compute_mass_properties([MassShape(PrimitiveKind.BOX, (1, 1, 1))])
    assert started.wait(5)
    queued = service.compute_mass_properties([MassShape(PrimitiveKind.BOX, (1, 1, 1))])
    assert running.cancel()
    assert queued.cancel()
    for job in (running, queued):
        assert job.done
        assert not job.cancel()
        with pytest.raises(CancelledError):
            job.result(timeout=0)
    closer = Thread(target=lambda: (service.close(), drained.set()))
    closer.start()
    try:
        assert not drained.wait(0.02)
    finally:
        finish.set()
        closer.join(5)
    assert drained.is_set()
    service.close()


def test_job_rejects_device_input_before_copy(physics):
    class Device:
        is_cuda = True
        def __iter__(self):
            raise AssertionError('device input must not be iterated')
        def __deepcopy__(self, memo):
            raise AssertionError('device input must not be copied')
    with AuthoringService(physics._lib) as service:
        with pytest.raises(TypeError, match='CPU'):
            service.compute_mass_properties([MassShape(PrimitiveKind.BOX, Device())])


def test_cancel_completion_race_retains_future():
    """Complete exactly after cancellation publishes, before it reaches Future.cancel."""
    from concurrent.futures import Future
    from threading import Lock
    job = jobs.Job()
    future = Future()
    job._future = future
    future.add_done_callback(job._complete)
    class CompleteOnUnlock:
        def __init__(self):
            self.lock = Lock()
            self.once = True
        def __enter__(self):
            self.lock.acquire()
        def __exit__(self, *_):
            self.lock.release()
            if self.once:
                self.once = False
                future.set_result('late result')
    job._lock = CompleteOnUnlock()
    assert job.cancel()
    with pytest.raises(CancelledError):
        job.result(timeout=0)
