# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-READ-001
# @covers AC-8

"""An IsaacLab-style control loop whose read, policy and write replay as one CUDA graph.

A reinforcement-learning environment repeats the same control step: read the robots' state, evaluate the
policy on the GPU, write the actions, then step the simulation a few times. Here every environment is an Ant
cloned from one, and the read -> policy -> write part is captured once into a CUDA graph on Warp's stream,
then replayed every control step. Only the physics steps run outside the graph.

What the capture needs, all visible below:

1. Physics runs in the CUDA context Warp uses (``active_cuda_gpus``), on a DirectGPU scene
   (``/physics/suppressReadback``).
2. The queries are held for the whole run, and each is read once outside the capture, then released: the
   captured read reuses the device columns that read left on its query.
3. The write session stays open for as long as the graph is replayed.
4. Each replay finishes before the next physics step, and no step runs during a replay.

The policy is a fixed linear map squashed by tanh, standing in for a trained network. Every few control steps
half the environments are reset, outside the graph and with no step, as an RL loop resets the episodes that
ended. Every control step checks that the graph read the current state and that the actions it computed
reached the joint drives.
"""

import os
import re
from pathlib import Path

import numpy as np
import ovstage
import warp as wp

import ovphysx
from ovphysx import PhysX, PhysXConfig
from ovphysx.types import SimObjectType
from session_write import attach_scene

NUM_ENVS = 16
ENV_SPACING = 2.0
DT = 1.0 / 120.0
DECIMATION = 4  # physics steps per control step
CONTROL_STEPS = 20
RESET_EVERY = 5  # control steps between resets

ROOT_ATTRIBUTES = ["rootPosition", "rootOrientation", "rootLinearVelocity", "rootAngularVelocity"]
JOINT_ATTRIBUTES = ["jointPosition", "jointVelocity"]
ACTION_ATTRIBUTE = "jointPositionTarget"
# The Ant's joints in the order the policy indexes them, with each joint's limits in degrees.
JOINT_LIMITS = {
    "front_left_leg": (-40.0, 40.0),
    "front_left_foot": (30.0, 100.0),
    "front_right_leg": (-40.0, 40.0),
    "front_right_foot": (-100.0, -30.0),
    "left_back_leg": (-40.0, 40.0),
    "left_back_foot": (-100.0, -30.0),
    "right_back_leg": (-40.0, 40.0),
    "right_back_foot": (30.0, 100.0),
}
NUM_JOINTS = len(JOINT_LIMITS)
# Per environment: torso height, orientation, linear and angular velocity, then each joint's position and velocity.
NUM_OBS = 1 + 4 + 3 + 3 + 2 * NUM_JOINTS
# Where each root attribute lands in an environment's observations, and which of its components.
ROOT_OBSERVATIONS = {
    "rootPosition": (0, slice(2, 3)),
    "rootOrientation": (1, slice(0, 4)),
    "rootLinearVelocity": (5, slice(0, 3)),
    "rootAngularVelocity": (8, slice(0, 3)),
}


@wp.kernel
def gather_observations(
    root_position: wp.array(dtype=float),
    root_orientation: wp.array(dtype=float),
    root_linear_velocity: wp.array(dtype=float),
    root_angular_velocity: wp.array(dtype=float),
    joint_position: wp.array(dtype=float),
    joint_velocity: wp.array(dtype=float),
    env_root: wp.array(dtype=int),
    env_joints: wp.array2d(dtype=int),
    obs: wp.array2d(dtype=float),
):
    env = wp.tid()
    root = env_root[env]
    obs[env, 0] = root_position[root * 3 + 2]
    for k in range(4):
        obs[env, 1 + k] = root_orientation[root * 4 + k]
    for k in range(3):
        obs[env, 5 + k] = root_linear_velocity[root * 3 + k]
        obs[env, 8 + k] = root_angular_velocity[root * 3 + k]
    joints = env_joints.shape[1]
    for k in range(joints):
        obs[env, 11 + k] = joint_position[env_joints[env, k]]
        obs[env, 11 + joints + k] = joint_velocity[env_joints[env, k]]


@wp.kernel
def evaluate_policy(
    obs: wp.array2d(dtype=float),
    joint_env: wp.array(dtype=int),
    joint_slot: wp.array(dtype=int),
    weights: wp.array2d(dtype=float),
    center: wp.array(dtype=float),
    half_range: wp.array(dtype=float),
    action: wp.array(dtype=float),
):
    row = wp.tid()
    env = joint_env[row]
    slot = joint_slot[row]
    acc = float(0.0)
    for k in range(obs.shape[1]):
        acc += weights[slot, k] * obs[env, k]
    action[row] = center[slot] + half_range[slot] * wp.tanh(acc)


def flat(tensors, device):
    """One float array over a group's column. A root or joint group is one tensor, its rows stacked; a joint
    group has one row per DOF."""
    assert len(tensors) == 1, "a root or joint group is one tensor"
    return wp.array(ptr=tensors[0].ptr, dtype=float, shape=(tensors[0].size,), device=device)


def row_paths(dictionary, group):
    """The prim path of each row of a group."""
    handles = dictionary.get_paths(group.prim_list)
    return [dictionary.path_to_string(handles[group.prim_offset + row]) for row in range(group.prim_count)]


def env_of(path):
    return int(re.search(r"/envs/env(\d+)/", path).group(1))


def open_read(physx, query, attributes, cuda_stream):
    """Opens a read on `cuda_stream` and fetches every group."""
    result = physx.read(query, attributes, cuda_stream=cuda_stream).wait()
    groups = []
    while (group := result.fetch_next().wait()) is not None:
        groups.append(group)
    return result, groups


def by_attribute(stage, groups):
    """Each group under its attribute name. One scene, so one group per attribute."""
    with ovstage.PathDictionary(stage) as dictionary:
        named = {dictionary.token_to_string(group.attribute): group for group in groups}
    assert len(named) == len(groups)
    return named


def read_layout(physx, stage, roots, joints, cuda_stream):
    """Reads both queries once outside the capture, and returns where each environment's rows are.

    The reads are closed, which leaves their device columns on the queries for the captured reads to reuse.
    """
    root_result, root_groups = open_read(physx, roots, ROOT_ATTRIBUTES, cuda_stream)
    joint_result, joint_groups = open_read(physx, joints, JOINT_ATTRIBUTES, cuda_stream)
    with ovstage.PathDictionary(stage) as dictionary:
        root_paths = row_paths(dictionary, root_groups[0])
        joint_paths = row_paths(dictionary, joint_groups[0])
    root_result.close().wait()
    joint_result.close().wait()

    env_root = np.zeros(NUM_ENVS, dtype=np.int32)
    for row, path in enumerate(root_paths):
        env_root[env_of(path)] = row
    names = list(JOINT_LIMITS)
    joint_env = np.array([env_of(path) for path in joint_paths], dtype=np.int32)
    joint_slot = np.array([names.index(path.rsplit("/", 1)[1]) for path in joint_paths], dtype=np.int32)
    env_joints = np.zeros((NUM_ENVS, NUM_JOINTS), dtype=np.int32)
    env_joints[joint_env, joint_slot] = np.arange(len(joint_paths))
    return {
        "env_root": env_root,
        "env_joints": env_joints,
        "joint_env": joint_env,
        "joint_slot": joint_slot,
        "joint_paths": joint_paths,
    }


def observe_on_host(physx, stage, layout):
    """The observations of every environment, read outside the graph, for the checks."""
    obs = np.zeros((NUM_ENVS, NUM_OBS), dtype=np.float32)
    for object_type, attributes in ((SimObjectType.ARTICULATION, ROOT_ATTRIBUTES),
                                    (SimObjectType.ARTICULATION_JOINT, JOINT_ATTRIBUTES)):
        with physx.read_sync(object_type, attributes) as result, ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                name = dictionary.token_to_string(group.attribute)
                # One copy to the host per group: a group holds every row in one tensor.
                column = group.tensors[0].numpy()
                for row, path in enumerate(row_paths(dictionary, group)):
                    env = env_of(path)
                    if name in ROOT_OBSERVATIONS:
                        start, components = ROOT_OBSERVATIONS[name]
                        values = column[row][components]
                        obs[env, start:start + len(values)] = values
                    else:
                        slot = list(JOINT_LIMITS).index(path.rsplit("/", 1)[1])
                        offset = 11 if name == "jointPosition" else 11 + NUM_JOINTS
                        obs[env, offset + slot] = column[row]
    return obs


def joint_targets_on_host(physx, stage):
    """Each joint's drive target, by prim path, read outside the graph."""
    targets = {}
    with physx.read_sync(SimObjectType.ARTICULATION_JOINT, [ACTION_ATTRIBUTE]) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                column = group.tensors[0].numpy()
                for row, path in enumerate(row_paths(dictionary, group)):
                    targets[path] = float(column[row])
    return targets


def reset_envs(physx, stage, joints, layout, envs, device, cuda_stream):
    """Puts the joints of `envs` back at the middle of their limits, at rest, through a masked write outside the
    graph."""
    session = physx.write(joints, ["jointPosition", "jointVelocity"]).wait()
    groups = []
    while (group := session.fetch_next().wait()) is not None:
        groups.append(group)
    with ovstage.PathDictionary(stage) as dictionary:
        assert all(row_paths(dictionary, group) == layout["joint_paths"] for group in groups)
    middles = np.array(list(JOINT_LIMITS.values()), dtype=np.float32).mean(axis=1)[layout["joint_slot"]]
    values = {"jointPosition": middles, "jointVelocity": np.zeros_like(middles)}
    mask = wp.array(np.isin(layout["joint_env"], envs).astype(np.uint8), dtype=wp.uint8, device=device)
    for name, group in by_attribute(stage, groups).items():
        flat(group.tensors, device).assign(values[name])
        session.commit(group, cuda_stream=cuda_stream, mask=mask).wait()
    session.close().wait()
    # The mask is read on the stream.
    wp.synchronize_device(device)


def run(physx, stage, device, stream):
    cuda_stream = int(stream.cuda_stream)

    # IsaacLab's cloner: one Ant in env0, copied on a grid. Copies do not collide with each other.
    grid = int(np.ceil(np.sqrt(NUM_ENVS)))
    physx.clone(
        "/World/envs/env0",
        [f"/World/envs/env{i}" for i in range(1, NUM_ENVS)],
        anchor_transforms=[((i % grid) * ENV_SPACING, (i // grid) * ENV_SPACING, 0.5, 0, 0, 0, 1)
                           for i in range(1, NUM_ENVS)],
    )
    physx.wait_all()
    # A DirectGPU scene serves reads and writes once it has stepped.
    physx.warmup()
    physx.step(DT)
    physx.wait_all()

    # Held for the whole run: the captured reads and the write keep their device buffers on them.
    roots = physx.query(SimObjectType.ARTICULATION).wait()
    joints = physx.query(SimObjectType.ARTICULATION_JOINT).wait()
    layout = read_layout(physx, stage, roots, joints, cuda_stream)
    # Collects the releases the closes above submitted, so none is left to run inside the capture.
    physx.wait_all()

    session = physx.write(joints, [ACTION_ATTRIBUTE]).wait()
    action_groups = []
    while (group := session.fetch_next().wait()) is not None:
        action_groups.append(group)
    (action_group,) = action_groups
    with ovstage.PathDictionary(stage) as dictionary:
        assert row_paths(dictionary, action_group) == layout["joint_paths"], "reads and writes share row order"
    action = flat(action_group.tensors, device)

    # The policy's parameters, and the observation buffer it reads, live on the device.
    rng = np.random.default_rng(0)
    obs_scale = np.array([1.0] + [1.0] * 4 + [1.0] * 3 + [5.0] * 3 + [100.0] * NUM_JOINTS + [500.0] * NUM_JOINTS,
                         dtype=np.float32)
    weights_host = (rng.normal(0.0, 0.5, (NUM_JOINTS, NUM_OBS)) / obs_scale).astype(np.float32)
    limits = np.array(list(JOINT_LIMITS.values()), dtype=np.float32)
    center_host = limits.mean(axis=1)
    half_range_host = 0.5 * (limits[:, 1] - limits[:, 0])
    weights = wp.array(weights_host, dtype=float, device=device)
    center = wp.array(center_host, dtype=float, device=device)
    half_range = wp.array(half_range_host, dtype=float, device=device)
    env_root = wp.array(layout["env_root"], dtype=int, device=device)
    env_joints = wp.array(layout["env_joints"], dtype=int, device=device)
    joint_env = wp.array(layout["joint_env"], dtype=int, device=device)
    joint_slot = wp.array(layout["joint_slot"], dtype=int, device=device)
    obs = wp.zeros((NUM_ENVS, NUM_OBS), dtype=float, device=device)
    # Kernels load outside the capture.
    wp.load_module(device=device)

    # One control step: read, policy, write. Each call waits for its task before the capture ends.
    with wp.ScopedCapture(device=device, stream=stream) as capture:
        root_read, root_groups = open_read(physx, roots, ROOT_ATTRIBUTES, cuda_stream)
        joint_read, joint_groups = open_read(physx, joints, JOINT_ATTRIBUTES, cuda_stream)
        state = {name: flat(group.tensors, device) for name, group in by_attribute(stage, root_groups).items()}
        state.update({name: flat(group.tensors, device) for name, group in by_attribute(stage, joint_groups).items()})
        wp.launch(gather_observations, dim=NUM_ENVS, device=device, stream=stream,
                  inputs=[state["rootPosition"], state["rootOrientation"], state["rootLinearVelocity"],
                          state["rootAngularVelocity"], state["jointPosition"], state["jointVelocity"],
                          env_root, env_joints, obs])
        wp.launch(evaluate_policy, dim=action.shape[0], device=device, stream=stream,
                  inputs=[obs, joint_env, joint_slot, weights, center, half_range, action])
        session.commit(action_group, cuda_stream=cuda_stream).wait()

    for step in range(CONTROL_STEPS):
        expected_obs = observe_on_host(physx, stage, layout)

        wp.capture_launch(capture.graph, stream=stream)
        # The replay finishes before the physics steps.
        wp.synchronize_stream(stream)

        # The graph read the current state, computed the policy's actions from it, and wrote them to the drives.
        graph_obs = obs.numpy()
        assert np.allclose(graph_obs, expected_obs, atol=1e-4), f"control step {step}: stale observations"
        graph_action = action.numpy()
        expected_action = center_host[layout["joint_slot"]] + half_range_host[layout["joint_slot"]] * np.tanh(
            np.einsum("rk,rk->r", weights_host[layout["joint_slot"]], graph_obs[layout["joint_env"]]))
        assert np.allclose(graph_action, expected_action, atol=1e-3), f"control step {step}: wrong actions"
        targets = joint_targets_on_host(physx, stage)
        assert np.allclose([targets[path] for path in layout["joint_paths"]], graph_action, atol=1e-3), (
            f"control step {step}: the actions did not reach the drives")

        for _ in range(DECIMATION):
            physx.step(DT)
        # No step may run during the next replay.
        physx.wait_all()
        # The next replay reads the reset joints, with no step between.
        if step % RESET_EVERY == RESET_EVERY - 1:
            envs = np.arange(step % 2, NUM_ENVS, 2)
            reset_envs(physx, stage, joints, layout, envs, device, cuda_stream)
            reset_obs = observe_on_host(physx, stage, layout)
            assert np.allclose(reset_obs[envs, 11:11 + NUM_JOINTS], limits.mean(axis=1), atol=1e-3), (
                f"control step {step}: the reset did not reach the joints")
        if step % 5 == 0:
            print(f"control step {step}: mean torso height {graph_obs[:, 0].mean():.3f} m, "
                  f"mean |action| {np.abs(graph_action).mean():.1f} deg")

    # Released outside any capture.
    session.close().wait()
    root_read.close().wait()
    joint_read.close().wait()
    roots.close().wait()
    joints.close().wait()
    print(f"{CONTROL_STEPS} control steps replayed from one CUDA graph over {NUM_ENVS} environments")


def main():
    wp.init()
    if wp.get_cuda_device_count() == 0:
        # A GPU test runner sets OVPHYSX_TEST_REQUIRE_CUDA=1: a missing device there is a failure, not a skip.
        if os.environ.get("OVPHYSX_TEST_REQUIRE_CUDA") == "1":
            raise RuntimeError("rl_step_capture needs a CUDA device, and OVPHYSX_TEST_REQUIRE_CUDA=1")
        print("rl_step_capture needs a CUDA device; skipping")
        return

    # Prefer package data so a copied sample works. Fall back to the checked-in
    # sample's adjacent data directory when package data is absent.
    usd_path = Path(ovphysx.__file__).resolve().parent / "samples" / "data" / "ant_envs_gpu.usda"
    if not usd_path.is_file():
        usd_path = Path(__file__).resolve().parent.parent / "data" / "ant_envs_gpu.usda"
    if not usd_path.is_file():
        raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

    # Physics in Warp's CUDA context, on DirectGPU.
    physx = PhysX(config=PhysXConfig(carbonite_overrides={"/physics/suppressReadback": True}), active_cuda_gpus="0")
    stage = None
    try:
        stage = attach_scene(physx, usd_path)
        device = wp.get_device("cuda:0")
        run(physx, stage, device, wp.get_stream(device))
    finally:
        # Every read array is gone with run(); the instance can be destroyed.
        physx.wait_all()
        if stage is not None:
            physx.wait_task(physx.detach_ovstage())
            stage.destroy()
        physx.destroy()


if __name__ == "__main__":
    main()
