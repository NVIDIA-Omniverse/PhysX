# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CAMERA-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Follow-camera extension flow with completed native reads and customer USD.

Native simulation runs in an explicit worker. The editor creates a camera, uses
a selected preview layer, and restores its original opinions on disable.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

SCENE = Path(__file__).with_name('scenes')/'two_boxes.usda'


def worker(output):
    from dataclasses import asdict
    from ovphysx import PhysX
    import ovstage
    from ovphysx_companion.output import capture_body_output
    from ovphysx_companion.schemas import physx_schema_paths
    ovstage.population.register_usd_schemas([str(p) for p in physx_schema_paths()])
    PhysX.set_cpu_mode(True)
    physics = PhysX()
    stage = ovstage.Stage('camera-reader')
    frames = []
    try:
        ovstage.population.open_usd(stage, str(SCENE), ordinal=1, domains=ovstage.PopulationDomain.ALL)
        stage.advance_write_floor(ordinal=1).wait()
        physics.wait_task(physics.attach_ovstage(stage, read_ordinal=1))
        for step in range(1, 7):
            physics.step(1/60); physics.wait_all()
            frame = capture_body_output(physics, source_revision='sealed-input-1', step=step,
                                         simulation_time=step/60, meters_per_unit=1, pattern='/World/BoxA')
            assert len(frame) == 1
            frames.append(asdict(frame[0]))
    finally:
        physics.destroy()
        stage.destroy()
    Path(output).write_text(json.dumps(frames), encoding='utf-8')


def run():
    from ovphysx.utils import codeless
    from ovphysx_companion.camera import FollowCamera
    from ovphysx_companion.output import BodyTarget, BodyOutput
    from ovphysx_companion.native import Identity
    from ovphysx_companion.usd import EditSession
    from ovphysx_companion.usd.camera import FollowCameraOutput
    codeless.register_schemas(verify=True)
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(Usd.Stage.Open(str(SCENE)).Flatten())
    camera = UsdGeom.Camera.Define(stage, '/World/Camera')
    camera.AddTranslateOp().Set((10,10,10))
    layer = stage.GetSessionLayer()
    before = layer.ExportToString()
    with EditSession(stage) as session:
        source = session.source
        with tempfile.TemporaryDirectory(prefix='companion-camera-') as temporary:
            output = Path(temporary)/'frames.json'
            import ovphysx_companion
            env = dict(os.environ)
            env['PYTHONPATH'] = os.pathsep.join(filter(None, [str(Path(ovphysx_companion.__file__).parents[1]),
                                                             env.get('PYTHONPATH')]))
            subprocess.run([sys.executable, str(Path(__file__).resolve()), '--worker', str(output)],
                            check=True, capture_output=True, text=True, timeout=60, env=env)
            frames = json.loads(output.read_text(encoding='utf-8'))
        identity = frames[0]['target']
        target = BodyTarget(Identity(**identity['owner']), identity['path'], source)
        positions = []
        with FollowCameraOutput(session, '/World/Camera',
                FollowCamera(target, offset=(0,-5,2), up=(0,0,1)), layer=layer) as adapter:
            for frame in frames:
                assert frame['target'] == identity
                frame = dict(frame, target=target)
                assert adapter.update(BodyOutput(**frame))
                positions.append(tuple(UsdGeom.XformCache().GetLocalToWorldTransform(camera.GetPrim()).ExtractTranslation()))
        assert layer.ExportToString() == before
    assert positions[-1][2] < positions[0][2]
    return positions


if __name__ == '__main__':
    if '--worker' in sys.argv:
        worker(sys.argv[-1])
    else:
        print(json.dumps(run()))
