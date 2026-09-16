# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-USD-KINEMATIC-SUPPORT-001
# @covers AC-1 AC-3

"""Kinematic support sample: transform motion, surface velocity, and both."""

from pathlib import Path

import numpy as np
import ovstage

import ovphysx
from ovphysx import PhysX
from ovphysx.types import TensorType


_physx_schemas_registered = False


def attach_scene(physx, usd_path):
    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")
    # ovphysx ships its PhysX USD schemas as codeless resources and does not register
    # them itself. Register them with ovstage once, before the first population
    # call in the process.
    global _physx_schemas_registered
    if not _physx_schemas_registered:
        ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
        _physx_schemas_registered = True
    stage = ovstage.Stage("ovphysx-kinematic-support-sample")
    try:
        ovstage.population.open_usd(
            stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
        )
        stage.advance_write_floor(ordinal=1).wait()
        physx.attach_ovstage(stage, read_ordinal=1)
        return stage
    except Exception:
        stage.destroy()
        raise


def make_binding(physx, path, tensor_type):
    binding = physx.create_tensor_binding(pattern=path, tensor_type=tensor_type)
    if binding.count != 1:
        binding.destroy()
        raise RuntimeError(f"Expected one rigid body at {path}, found {binding.count}")
    return binding


# NOTE: this sample uses the deprecated tensor-binding API. New code should use the session
# read/write API (PhysX.read / PhysX.write).
def read_x(binding):
    poses = np.zeros(binding.shape, dtype=np.float32)
    binding.read(poses)
    return float(poses[0, 0])


class TransformWriter:
    """Publish world transforms for one prim through an ovstage query."""

    def __init__(self, stage, path):
        self._stage = stage
        self._paths = ovstage.PathDictionary(stage)
        self._path_list = None
        self._query = None
        try:
            self._path_list = self._paths.create_path_list_from_strings([path])
            self._query = stage.query_from_path_list(self._path_list)
        except Exception:
            if self._path_list is not None:
                self._paths.destroy_path_list(self._path_list)
            self._paths.destroy()
            raise

    def write_attribute(self, attribute, tensor, ordinal):
        self._stage.write_attribute(
            self._query, attribute, ordinal, tensor, is_array=False
        ).wait()

    def write_pose(self, x, z, ordinal):
        target = np.eye(4, dtype=np.float64)
        target[0, 0] = 6.0
        target[2, 2] = 6.0
        target[3, :3] = (x, 0.0, z)
        tensor = ovstage.make_dltensor(
            target,
            dtype=ovstage.DLDataType(ovstage.DLDataTypeCode.kDLFloat, 64, 16),
            shape=[1],
            ndim=1,
        )
        for attribute in ("omni:xform", "omni:fabric:worldMatrix"):
            self.write_attribute(attribute, tensor, ordinal)

    def destroy(self):
        if self._query is not None:
            self._stage.release_query(self._query).wait()
            self._query = None
        if self._path_list is not None:
            self._paths.destroy_path_list(self._path_list)
            self._path_list = None
        self._paths.destroy()


def main():
    PhysX.set_cpu_mode(True)
    physx = PhysX()
    stage = None
    bindings = []
    writers = []
    try:
        usd_path = Path(__file__).resolve().parent / ".." / "data" / "kinematic_support.usda"
        stage = attach_scene(physx, usd_path)
        physx.wait_all()

        def add_binding(path, tensor_type):
            binding = make_binding(physx, path, tensor_type)
            bindings.append(binding)
            return binding

        target_platform = TransformWriter(stage, "/World/TargetPlatform")
        writers.append(target_platform)
        combined_platform = TransformWriter(stage, "/World/CombinedPlatform")
        writers.append(combined_platform)
        target_rider = add_binding("/World/TargetRider", TensorType.RIGID_BODY_POSE)
        conveyor_rider = add_binding("/World/ConveyorRider", TensorType.RIGID_BODY_POSE)
        combined_rider = add_binding("/World/CombinedRider", TensorType.RIGID_BODY_POSE)

        dt = 1.0 / 60.0
        for _ in range(10):
            physx.step_sync(dt)

        start = np.array(
            [read_x(target_rider), read_x(conveyor_rider), read_x(combined_rider)]
        )
        for frame in range(1, 121):
            x = frame * dt
            ordinal = frame + 1
            target_platform.write_pose(x, 0.0, ordinal)
            combined_platform.write_pose(x, 16.0, ordinal)
            stage.advance_write_floor(ordinal=ordinal).wait()
            physx.update_from_ovstage(ordinal, ordinal)
            physx.step_sync(dt)

        displacement = (
            np.array([read_x(target_rider), read_x(conveyor_rider), read_x(combined_rider)])
            - start
        )
        target_dx, conveyor_dx, combined_dx = displacement
        print(
            "Rider displacement: "
            f"transform={target_dx:.3f}, surface={conveyor_dx:.3f}, combined={combined_dx:.3f}"
        )

        if target_dx < 0.5:
            raise RuntimeError("ovstage transform did not carry its rider")
        if conveyor_dx < 0.5:
            raise RuntimeError("Surface velocity did not carry its rider")
        if combined_dx <= max(target_dx, conveyor_dx) + 0.2:
            raise RuntimeError("Combined transform and surface velocity were not observably additive")
    finally:
        for writer in reversed(writers):
            writer.destroy()
        for binding in reversed(bindings):
            binding.destroy()
        if stage is not None:
            physx.detach_ovstage()
            stage.destroy()
        physx.destroy()


if __name__ == "__main__":
    main()
