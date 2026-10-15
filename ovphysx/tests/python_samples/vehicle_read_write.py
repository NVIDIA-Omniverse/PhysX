# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Select one car's chassis, controls, and wheels through independent queries."""

# @implements REQ-CAPI-VEHICLE-001
# @covers AC-1 AC-2 AC-4

from pathlib import Path

from ovphysx import PhysX, set_global_config
from ovphysx.types import GlobalConfigKey, SimObjectType
from session_write import attach_scene


def print_column(physx, query, attribute):
    with physx.read(query, [attribute]).wait() as result:
        while (group := result.fetch_next().wait()) is not None:
            paths = query.dictionary.get_path_strings(group.prim_list)
            paths = paths[group.prim_offset:group.prim_offset + group.prim_count]
            values = group.tensors[0].numpy()
            for path, value in zip(paths, values):
                print(f"{path} {attribute}: {value}")


def drive_one_car(physx):
    # The chassis and vehicle use the same prim; wheels use its descendants.
    with physx.query(SimObjectType.RIGID_BODY, pattern="/World/Standard").wait() as chassis, \
         physx.query(SimObjectType.VEHICLE, pattern="/World/Standard").wait() as vehicle, \
         physx.query(SimObjectType.VEHICLE_WHEEL, pattern="/World/Standard/**").wait() as wheels:
        counts = tuple(q.metadata.total_prim_count for q in (chassis, vehicle, wheels))
        if counts != (1, 1, 4):
            raise RuntimeError(f"Expected one chassis, one vehicle and four wheels, got {counts}")
        with physx.write(vehicle, ["accelerator"]).wait() as session:
            while (group := session.fetch_next().wait()) is not None:
                group.tensors[0].fill_(0.5)  # Complete CPU fill before commit.
                session.commit(group).wait()
        physx.step_sync(1.0 / 60.0)
        print_column(physx, chassis, "position")
        print_column(physx, vehicle, "engineRotationSpeed")
        print_column(physx, wheels, "rotationSpeed")
    # Collect the releases, including read storage after print_column drops its aliases.
    physx.wait_all()


def main():
    set_global_config(GlobalConfigKey.CPU_ONLY, True)
    physx = PhysX()
    stage = None
    try:
        scene = Path(__file__).resolve().parent.parent / "data" / "vehicle_read_write.usda"
        stage = attach_scene(physx, scene)
        drive_one_car(physx)
    finally:
        if stage is not None:
            physx.wait_task(physx.detach_ovstage())
            stage.destroy()
        physx.destroy()


if __name__ == "__main__":
    main()
