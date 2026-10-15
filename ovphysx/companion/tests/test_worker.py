# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-JOBS-001
# @covers AC-5
import json
from pathlib import Path
import sys
import pytest
from ovphysx_companion import _worker


def test_reply_destination_is_fixed(monkeypatch, tmp_path):
    directory = tmp_path/'private'
    directory.mkdir()
    monkeypatch.chdir(directory)
    monkeypatch.setattr(sys, 'argv', ['worker.py', '--worker', '../unexpected.json'])
    _worker.write_reply({'answer': 42})
    assert json.loads((directory/'reply.json').read_text()) == {'answer': 42}
    assert not (tmp_path/'unexpected.json').exists()


def test_private_reply_transport_ignores_native_logging(tmp_path):
    script = tmp_path/'worker.py'
    package_root = str(Path(_worker.__file__).resolve().parents[1])
    script.write_text(
        'import json, sys\n'
        f'sys.path.insert(0, {package_root!r})\n'
        'from ovphysx_companion._worker import write_reply\n'
        'print("native diagnostics")\n'
        'write_reply({"echo": json.load(sys.stdin)})\n')
    assert _worker.run_worker(script, {'value': 7}) == {'echo': {'value': 7}}
    assert not (tmp_path/'reply.json').exists()


def test_reply_does_not_overwrite_an_existing_destination(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    reply = tmp_path/'reply.json'
    reply.write_text('previous content')
    with pytest.raises(FileExistsError):
        _worker.write_reply({'answer': 42})
    assert reply.read_text() == 'previous content'
