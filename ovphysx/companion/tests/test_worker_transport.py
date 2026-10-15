# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-JOBS-001
# @covers AC-2 AC-3 AC-5
"""Worker replies remain unambiguous when native libraries log to stdout."""
import subprocess
import pytest

from ovphysx_companion._worker import run_worker


def test_worker_reply_ignores_logs_before_and_after_json(tmp_path):
    script = tmp_path / 'noisy.py'
    script.write_text('''import json,sys
from pathlib import Path
print('native debug initialization')
value=json.load(sys.stdin)
Path('reply.json').write_text(json.dumps({'value':value}))
print('{"looks_like_json":"but_is_a_log"}')
print('native shutdown')
''')
    assert run_worker(script, {'id': 17}) == {'value': {'id': 17}}


def test_worker_failure_does_not_publish_written_reply(tmp_path):
    script = tmp_path / 'failed.py'
    script.write_text("from pathlib import Path\nPath('reply.json').write_text('{}')\nraise SystemExit(4)\n")
    with pytest.raises(subprocess.CalledProcessError):
        run_worker(script, {})
