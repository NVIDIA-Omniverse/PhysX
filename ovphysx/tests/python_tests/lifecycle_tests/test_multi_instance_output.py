# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Independent output and stepping, with one process lifecycle for both arms.

@implements REQ-CAPI-WRITE-001
@covers AC-10
@implements REQ-CAPI-ASYNC-002
@covers AC-6
"""

from contextlib import ExitStack, nullcontext
import threading

import numpy as np
import ovstage
from test_utils import data_path, register_physx_schemas_with_ovstage

from ovphysx import PhysX
from ovphysx.types import SimObjectType


def _masses(physx):
    with physx.read_sync(SimObjectType.RIGID_BODY, ["mass"]) as session:
        return np.concatenate([group.tensors[0].numpy().copy().reshape(-1)
                               for group in session.groups])


def test_output_while_peer_steps():
    register_physx_schemas_with_ovstage()
    with ExitStack() as cleanup:
        instances = []
        for name in ("output-owner", "stepping-peer"):
            stage = ovstage.Stage(name)
            cleanup.callback(stage.destroy)
            physx = PhysX()
            cleanup.callback(physx.destroy)
            ovstage.population.open_usd(stage, data_path("boxes_falling_on_groundplane.usda"),
                                        ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
            stage.advance_write_floor(ordinal=1).wait()
            physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
            cleanup.callback(lambda p=physx: p.wait_task(p.detach_ovstage()))
            physx.step_sync(1.0 / 60.0)
            instances.append(physx)
        owner, peer = instances
        original_peer = _masses(peer)
        assert original_peer.size > 0

        # The serialized arm is a positive control using the same data and calls.
        for serialized in (True, False):
            lock = threading.Lock() if serialized else nullcontext()
            stop = threading.Event()
            started = threading.Event()
            errors = []
            steps = 0

            def step_peer():
                nonlocal steps
                try:
                    while not stop.is_set():
                        with lock:
                            peer.step_sync(1.0 / 60.0)
                        steps += 1
                        started.set()
                except Exception as error:
                    errors.append(error)
                    started.set()

            worker = threading.Thread(target=step_peer)
            worker.start()
            try:
                assert started.wait(15), "Peer did not start stepping"
                for iteration in range(300):
                    expected = float(iteration + 1)
                    with lock:
                        with owner.write_sync(SimObjectType.RIGID_BODY, ["mass"]) as session:
                            assert session.groups
                            for group in session.groups:
                                group.tensors[0].numpy()[:] = expected
                                session.commit_sync(group)
                        values = _masses(owner)
                        assert values.size > 0
                        np.testing.assert_allclose(values, expected, rtol=1e-6, atol=0)
            finally:
                stop.set()
                worker.join()
            assert not errors, errors
            assert steps
            np.testing.assert_array_equal(_masses(peer), original_peer)
