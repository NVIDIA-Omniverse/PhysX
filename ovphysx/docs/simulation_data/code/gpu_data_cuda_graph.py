# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# [example-start]
# An excerpt. run_with_graph takes the PhysX object from create_physx, after a scene
# of 1 kg rigid bodies is attached.

import warp as wp
from ovphysx import PhysX, PhysXConfig
from ovphysx.types import SimObjectType


def create_physx():
    """A PhysX object in DirectGPU mode, on GPU 0 like Warp."""
    return PhysX(
        config=PhysXConfig(carbonite_overrides={"/physics/suppressReadback": True}),
        active_cuda_gpus="0",
    )


@wp.kernel
def hover(velocity: wp.array2d(dtype=float), force: wp.array2d(dtype=float)):
    """Stands in for a policy: holds 1 kg bodies up and damps their motion."""
    body = wp.tid()
    for axis in range(3):
        force[body, axis] = -2.0 * velocity[body, axis]
    force[body, 2] += 9.81


def run_with_graph(physx, device, steps):
    """Records one read, the kernel, and one write, then replays them."""
    stream = wp.get_stream(device)
    handle = int(stream.cuda_stream)  # the stream as ovphysx takes it
    physx.step_sync(1.0 / 60.0)  # DirectGPU data exists after the first step
    bodies = physx.query(SimObjectType.RIGID_BODY).wait()  # kept until the end

    # Read once and close the read. The captured read reuses its GPU memory.
    first = physx.read(bodies, ["linearVelocity"], cuda_stream=handle).wait()
    first.fetch_next().wait()  # one scene, so one group
    first.close().wait()
    session = physx.write(bodies, ["force"]).wait()  # stays open until the end
    force = session.fetch_next().wait()
    wp.load_module(device=device)  # load the kernel before the capture
    physx.wait_all()  # leave no ovphysx work to run inside the capture

    with wp.ScopedCapture(device=device, stream=stream) as capture:
        read = physx.read(bodies, ["linearVelocity"], cuda_stream=handle).wait()
        velocity = read.fetch_next().wait()
        wp.launch(hover, dim=velocity.prim_count, device=device, stream=stream,
                  inputs=[velocity.tensors[0], force.tensors[0]])
        session.commit(force, cuda_stream=handle).wait()

    for _ in range(steps):
        wp.capture_launch(capture.graph, stream=stream)
        wp.synchronize_stream(stream)  # the replay ends before the step
        physx.step(1.0 / 60.0)
        physx.wait_all()  # the step ends before the next replay

    session.close().wait()  # release everything outside the capture
    read.close().wait()
    bodies.close().wait()
# [example-end]
