# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVD-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Record a shared SDK capture, decode natively, then convert in customer USD."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def worker(output):
    import ovstage
    from ovphysx import PhysX, GlobalConfigKey, set_global_config, OmniPvdDestination
    from ovphysx_companion.preview import PreviewSession
    from ovphysx_companion.ovd import RecordingCoordinator, decode
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    PhysX.set_cpu_mode(True)
    set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
    physics=PhysX()
    library=physics._lib
    stages=[]
    filename=Path(output).with_suffix('.ovd')
    try:
        for name in ('record-playback','record-inspector'):
            stage=ovstage.Stage(name);stages.append(stage)
            ovstage.population.open_usd(stage,str(Path(__file__).with_name('scenes')/'two_boxes.usda'),
                                        ordinal=1,domains=ovstage.PopulationDomain.ALL)
            stage.advance_write_floor(ordinal=1).wait()
        physics.wait_task(physics.attach_ovstage(stages[0],read_ordinal=1))
        with PreviewSession(physics,stages[1],read_ordinal=1) as preview:
            with RecordingCoordinator(physics) as recording:
                try:RecordingCoordinator(physics)
                except RuntimeError:pass
                else:raise AssertionError('Duplicate process recording coordinator accepted')
                preview.simulate(1/60)
                try:recording.start(OmniPvdDestination.file(str(filename)))
                except RuntimeError:pass
                else:raise AssertionError('Recording transition consumed pending preview work')
                preview.fetch()
                recording.start(OmniPvdDestination.file(str(filename)))
                for _ in range(3):
                    physics.step_sync(1/60);preview.step(1/60)
                preview.simulate(1/60)
                try:recording.stop()
                except RuntimeError:pass
                else:raise AssertionError('Recording stop consumed pending preview work')
                preview.fetch()
                preview.close()
                assert recording.active # preview retirement did not finalize core recording
                recording.stop()
    finally:
        physics.destroy()
        for stage in stages:stage.destroy()
    document=decode(library,filename.read_bytes()) # stopped: runtime has already shut down
    assert any(r.command==9 for r in document.records)
    assert any(r.command==11 for r in document.records)
    assert any(r.name=='globalPose' for r in document.records)
    Path(output).write_text(document.to_json(),encoding='utf-8')


def run():
    from ovphysx_companion.ovd import Document
    from ovphysx_companion.usd import EditSession
    from ovphysx_companion.usd.ovd import convert
    from pxr import Usd
    with tempfile.TemporaryDirectory(prefix='companion-ovd-') as temporary:
        output=Path(temporary)/'capture.json'
        import ovphysx_companion
        env=dict(os.environ)
        env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
        subprocess.run([sys.executable,str(Path(__file__).resolve()),'--worker',str(output)],
                        check=True,capture_output=True,text=True,timeout=90,env=env)
        document=Document.from_json(output.read_text(encoding='utf-8'))
    stage=Usd.Stage.CreateInMemory()
    with EditSession(stage) as session:
        result=convert(session,document,root='/Capture',layer=stage.GetRootLayer(),meters_per_unit=1)
        assert result.objects and result.frames
        after=stage.GetRootLayer().ExportToString()
        result.patch.undo();assert not stage.GetPrimAtPath('/Capture')
        result.patch.redo();assert stage.GetRootLayer().ExportToString()==after
    return len(result.objects)


if __name__=='__main__':
    if '--worker' in sys.argv:worker(sys.argv[-1])
    else:print(f'{run()} OVD object lifetimes converted')
