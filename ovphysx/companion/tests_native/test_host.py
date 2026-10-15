# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-HOST-001
# @covers AC-1 AC-2 AC-4
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import pytest
from ovphysx_companion.host import FrameCoordinator, CoordinatorConflict, cpu_tool_availability
from ovphysx_companion.native import ToolsSession, CompanionError
from ovphysx_companion.preview import PreviewSession


def test_host_shared_stageupdate_coordinator_steps_once_and_gates_actions(attached):
    with ToolsSession(attached) as session, FrameCoordinator(session) as host:
        with ToolsSession(attached) as another:
            with pytest.raises(CoordinatorConflict):FrameCoordinator(another)
        calls=[]
        frame=host.tick(10,1/60,before_step=lambda:calls.append('input'),after_step=lambda f:calls.append(f))
        assert host.tick(10,1/60,after_step=lambda f:calls.append('duplicate')) is frame
        assert calls==['input',frame]
        with pytest.raises(ValueError):host.tick(10,.1)
        with pytest.raises(ValueError):host.tick(9,1/60)
        host.pause()
        assert host.tick(11,1/60) is None
        host.resume()
        assert host.tick(11,1/60).step==2
        assert cpu_tool_availability(session,'character-controller').available
        assert cpu_tool_availability(session,'debug-geometry').available
        missing=cpu_tool_availability(session,'legacy-viewer')
        assert not missing.available and missing.reason
        with pytest.raises(CompanionError,match='legacy-viewer'):missing.require()
        with ThreadPoolExecutor(1) as pool:
            with pytest.raises(RuntimeError,match='approved thread'):pool.submit(host.tick,12,1/60).result()
    with ToolsSession(attached) as replacement, FrameCoordinator(replacement) as host:
        assert host.tick(0,1/60).step==1


def test_host_reentrant_or_output_failure_cannot_replay_step(attached):
    with ToolsSession(attached) as session, FrameCoordinator(session) as host:
        def input():
            with pytest.raises(RuntimeError,match='reentry'):host.tick(2,.01)
            with pytest.raises(RuntimeError,match='callback'):host.close()
        def output(frame):raise ValueError('presentation failed')
        with pytest.raises(ValueError,match='presentation failed'):host.tick(1,.01,before_step=input,after_step=output)
        assert host.paused and host._steps==1
        with pytest.raises(RuntimeError,match='latched'):host.tick(1,.01)
        with pytest.raises(RuntimeError,match='replace'):host.resume()
        assert host._steps==1


def test_host_preview_never_consumes_uncollected_playback(physics,attached):
    import ovstage
    stage=ovstage.Stage('host-inspector')
    ovstage.population.open_usd(stage,str(Path(__file__).resolve().parents[1]/'samples/scenes/two_boxes.usda'),
                                ordinal=1,domains=ovstage.PopulationDomain.ALL)
    stage.advance_write_floor(ordinal=1).wait()
    try:
        with PreviewSession(physics,stage,read_ordinal=1) as preview, FrameCoordinator(preview) as inspector:
            playback_task=attached.step(1/60)
            frame=inspector.tick(1,1/60)
            assert frame.step==1
            assert inspector.tick(1,1/60) is frame
            attached.wait_task(playback_task) # succeeds: preview did not consume the core receipt
    finally:stage.destroy()


def test_host_extension_sample_in_native_worker():
    import os
    import subprocess
    import sys
    import ovphysx_companion
    env=dict(os.environ)
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(ovphysx_companion.__file__).parents[1]),env.get('PYTHONPATH')]))
    sample=Path(__file__).resolve().parents[1]/'samples/host_stageupdate.py'
    result=subprocess.run([sys.executable,str(sample)],env=env,capture_output=True,text=True,timeout=60)
    assert result.returncode==0,result.stdout+result.stderr
