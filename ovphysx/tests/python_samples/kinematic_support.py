# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-USD-KINEMATIC-SUPPORT-001
# @covers AC-1 AC-3

"""Kinematic support sample: transform motion, surface velocity, and both."""

# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1
# @implements REQ-PYTHON-SAMPLE-002
# @covers AC-1 AC-2

from pathlib import Path

import numpy as np
import ovstage

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import PhysX
from ovphysx.types import SimObjectType


_physx_schemas_registered = False


def attach_scene(physx, usd_path):
    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")
    # Register before population; this scene can omit the Newton provider.
    global _physx_schemas_registered
    if not _physx_schemas_registered:
        schema_roots = [str(p) for p in physx_schema_paths()]
        try:
            schema_roots.append(str(ovphysx.newton_schema_root()))
        except FileNotFoundError:
            pass  # newton-usd-schemas is optional; this scene has no newton:* attributes.
        ovstage.population.register_usd_schemas(schema_roots)
        _physx_schemas_registered = True
    stage = ovstage.Stage("ovphysx-kinematic-support-sample")
    try:
        ovstage.population.open_usd(
            stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.ALL
        )
        stage.advance_write_floor(ordinal=1).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        return stage
    except Exception:
        if physx.is_stage_attached(stage):
            physx.wait_task(physx.detach_ovstage())
        stage.destroy()
        raise


RIDERS = ("/World/TargetRider", "/World/ConveyorRider", "/World/CombinedRider")


def read_rider_x(physx, stage):
    """World x of each rider, in RIDERS order, through one read selected by the riders' paths."""
    with ovstage.PathDictionary(stage) as paths:
        handles = [paths.intern_path(path) for path in RIDERS]
        x = {}
        with physx.read_sync(SimObjectType.RIGID_BODY, ["position"], prim_paths=handles) as result:
            for group in result.groups:
                listed = paths.get_paths(group.prim_list)
                positions = group.tensors[0].numpy().reshape(group.prim_count, -1)
                for row in range(group.prim_count):
                    x[paths.path_to_string(listed[group.prim_offset + row])] = float(positions[row, 0])
    missing = [path for path in RIDERS if path not in x]
    if missing:
        raise RuntimeError(f"Expected a rigid body at each of {missing}")
    return np.array([x[path] for path in RIDERS])


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
    writers = []
    try:
        # Prefer package data so a copied sample works. Fall back to the checked-in
        # sample's adjacent data directory when package data is absent.
        usd_path = Path(ovphysx.__file__).resolve().parent / "samples" / "data" / "kinematic_support.usda"
        if not usd_path.is_file():
            usd_path = Path(__file__).resolve().parent.parent / "data" / "kinematic_support.usda"
        if not usd_path.is_file():
            raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")
        stage = attach_scene(physx, usd_path)
        physx.wait_all()

        target_platform = TransformWriter(stage, "/World/TargetPlatform")
        writers.append(target_platform)
        combined_platform = TransformWriter(stage, "/World/CombinedPlatform")
        writers.append(combined_platform)

        dt = 1.0 / 60.0
        for _ in range(10):
            physx.step_sync(dt)

        start = read_rider_x(physx, stage)
        for frame in range(1, 121):
            x = frame * dt
            ordinal = frame + 1
            target_platform.write_pose(x, 0.0, ordinal)
            combined_platform.write_pose(x, 16.0, ordinal)
            stage.advance_write_floor(ordinal=ordinal).wait()
            physx.wait_task(physx.update_from_ovstage(ordinal, ordinal))
            physx.step_sync(dt)

        displacement = read_rider_x(physx, stage) - start
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
        if stage is not None:
            physx.wait_task(physx.detach_ovstage())
            stage.destroy()
        physx.destroy()


if __name__ == "__main__":
    main()
