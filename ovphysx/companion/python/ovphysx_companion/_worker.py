# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-JOBS-001
# @covers AC-2 AC-3 AC-5
"""One-shot sample transport with a reply channel independent of native logging."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def run_worker(script, request, *, env=None, timeout=60):
    script = Path(script).resolve(strict=True)
    with tempfile.TemporaryDirectory(prefix='companion-reply-') as directory:
        reply = Path(directory) / 'reply.json'
        subprocess.run([sys.executable, str(script), '--worker'],
                       input=json.dumps(request), text=True, capture_output=True,
                       check=True, timeout=timeout, env=env, cwd=directory)
        return json.loads(reply.read_text(encoding='utf-8'))


def write_reply(value):
    # A failed worker is rejected by its exit status even if it wrote a reply.
    # The parent creates a private working directory. A fixed reply name keeps
    # worker arguments from selecting or escaping the destination directory.
    with open('reply.json', 'x', encoding='utf-8') as reply:
        json.dump(value, reply)
