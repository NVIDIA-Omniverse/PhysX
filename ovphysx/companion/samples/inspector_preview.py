# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PREVIEW-001
# @covers AC-1 AC-2 AC-4 AC-5 AC-6 AC-7
"""Two real attachments, independent clocks, both retirement orders and recording.

A native host starts at PreviewSession(playback, snapshot_stage, ...). A customer
USD editor can run this simulation half in a worker and exchange copied poses;
USD authoring stays in the editor. This sample imports no customer pxr.
"""
from pathlib import Path
from ovphysx_companion.preview import PreviewSession
from ovphysx_companion.native import CompanionError, Status


def run(*, playback_first=False, recording=None):
    import ovphysx
    import ovstage
    from ovphysx import PhysX, GlobalConfigKey, set_global_config, OmniPvdDestination

    from ovphysx_companion.schemas import physx_schema_paths
    schemas = [str(p) for p in physx_schema_paths()]
    if ovphysx.newton_schema_root():
        schemas.append(str(ovphysx.newton_schema_root()))
    ovstage.population.register_usd_schemas(schemas)
    PhysX.set_cpu_mode(True)
    set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, bool(recording))
    physics = PhysX()
    stages = [ovstage.Stage('playback'), ovstage.Stage('inspector-snapshot')]
    preview = None
    unused = PhysX()  # Unattached owners cannot finalize another owner's recording.
    try:
        for stage in stages:
            ovstage.population.open_usd(stage, str(Path(__file__).parent / 'scenes/two_boxes.usda'),
                                       ordinal=1, domains=ovstage.PopulationDomain.ALL)
            stage.advance_write_floor(ordinal=1).wait()
        physics.wait_task(physics.attach_ovstage(stages[0], read_ordinal=1))
        physics.step(1 / 60)
        physics.wait_all()
        preview = PreviewSession(physics, stages[1], read_ordinal=1, threads=2, gpu_pipeline=0)
        preview.step(1 / 60)
        if recording:
            physics.start_recording(OmniPvdDestination.file(str(recording)))
        preview.simulate(1 / 60)
        unused.destroy()  # Allowed even while preview B is pending and A records.
        playback_task = physics.step(1 / 120)
        if recording:
            try:
                physics.stop_recording()
                raise AssertionError('Recording transition accepted with pending preview')
            except RuntimeError as error:
                assert 'safe point' in str(error)
        if playback_first:
            if recording:
                # A owns recording finalization: it cannot retire until B is idle.
                try:
                    physics.destroy()
                    raise AssertionError('Recording owner retired with pending preview')
                except RuntimeError:
                    pass
                preview.fetch()
            physics.destroy()
            if not recording:
                preview.fetch()  # B survives even the last core owner and its pending interval
            before = preview.rigid_pose('/World/BoxA')
            preview.step(1 / 60)
            assert preview.rigid_pose('/World/BoxA').position[2] < before.position[2]
            preview.close()
        else:
            preview.close()
            physics.wait_task(playback_task)  # B close did not consume A's receipt
            if recording:
                assert physics.is_recording()
                physics.stop_recording()
            physics.step(1 / 60)
            physics.wait_all()
            physics.destroy()
        print('Inspector preview retirement and playback isolation passed')
    finally:
        unused.destroy()
        if preview:
            preview.close()
        physics.destroy()
        for stage in stages:
            stage.destroy()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--playback-first', action='store_true')
    parser.add_argument('--recording')
    args = parser.parse_args()
    run(playback_first=args.playback_first, recording=args.recording)
