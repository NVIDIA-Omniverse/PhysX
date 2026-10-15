# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2
# @implements REQ-COMP-DEBUG-001
# @covers AC-2

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('sample', ['live_statistics', 'mass_properties', 'mesh_mapping'])
def test_native_sample_cli(sample):
    companion = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env['PYTHONPATH'] = str(companion/'python')
    # Both examples must keep customer pxr out of the native process.
    script = "import runpy,sys; runpy.run_path(sys.argv[1], run_name='__main__'); assert 'pxr' not in sys.modules"
    run = subprocess.run([sys.executable, '-c', script, str(companion/'samples'/f'{sample}.py')],
                         env=env, capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stdout + run.stderr
    if sample == 'mass_properties':
        assert json.loads(run.stdout)['mass'] == 2
    elif sample == 'mesh_mapping':
        assert json.loads(run.stdout)['visual_positions'] == [[.25, .25, 1.25]]
    else:
        assert run.stdout.count('SceneStatistics(') == 3
