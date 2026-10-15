# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PYTHON-001
# @covers AC-5
"""The companion version comes only from ovphysx/VERSION, including sdist builds."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import zipfile

COMPANION = Path(__file__).resolve().parents[1]
VERSION = (COMPANION.parent / 'VERSION').read_text().strip()


def _uv():
    # `uv run` exports UV; CI and run_pytest.sh run this suite through it.
    uv = os.environ.get('UV') or shutil.which('uv')
    assert uv, 'Run the companion suite through uv; the sdist check needs an isolated build'
    return uv


def _run(*command, **kwargs):
    result = subprocess.run(command, capture_output=True, text=True, **kwargs)
    assert result.returncode == 0, f'{command[:3]} failed:\n{result.stdout}\n{result.stderr}'
    return result.stdout


def _normalized(version):
    return re.sub(r'[-_.]', '', version.lower())


def test_sdist_to_isolated_wheel_keeps_shared_version(tmp_path):
    # Copy the package with the shared VERSION at its repo-relative location so the
    # build writes no egg-info or stamp into the source tree.
    source = tmp_path/'ovphysx'/'companion'/'python'
    shutil.copytree(COMPANION/'python', source,
                    ignore=shutil.ignore_patterns('__pycache__', '*.egg-info', 'build', 'dist', '_version.py'))
    shutil.copy2(COMPANION.parent/'VERSION', tmp_path/'ovphysx'/'VERSION')
    out = tmp_path/'dist'
    uv = _uv()
    _run(uv, 'build', '--sdist', '--python', sys.executable, '-o', str(out), str(source))
    [sdist] = out.glob('*.tar.gz')
    with tarfile.open(sdist) as sdist_archive:
        names = sdist_archive.getnames()
    assert not any(name.endswith('/VERSION') for name in names)
    assert any(name.endswith('/ovphysx_companion/_version.py') for name in names)
    # uv extracts the sdist into an isolated location with no parent ovphysx tree.
    _run(uv, 'build', '--wheel', '--python', sys.executable, '-o', str(out), str(sdist))
    [wheel] = out.glob('*.whl')
    with zipfile.ZipFile(wheel) as wheel_archive:
        metadata = next(wheel_archive.read(n).decode()
                        for n in wheel_archive.namelist() if n.endswith('.dist-info/METADATA'))
    built = re.search(r'^Version: (.+)$', metadata, re.MULTILINE).group(1).strip()
    assert _normalized(built) == _normalized(VERSION)
    # The package is pure Python, so import it straight from the wheel via zipimport.
    origin, *imported = _run(sys.executable, '-c', 'import ovphysx_companion as c,ovphysx_companion.compatibility as k;'
                                                   'print(c.__file__,c.__version__,k.QUALIFIED_CORE_VERSION)',
                             cwd=tmp_path, env={**os.environ, 'PYTHONPATH': str(wheel)}).split()
    assert origin.startswith(str(wheel)), origin
    assert imported == [built, built]


def test_source_tree_has_no_generated_version_stamp():
    assert not (COMPANION/'python'/'ovphysx_companion'/'_version.py').exists()
    import ovphysx_companion
    assert ovphysx_companion.__version__ == VERSION
