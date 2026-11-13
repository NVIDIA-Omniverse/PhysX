# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-JOBS-001
# @covers AC-2 AC-3 AC-5
# @implements REQ-COMP-MASS-001
# @covers AC-1
# @implements REQ-COMP-VEHICLE-001
# @covers AC-1
"""One-shot sample transport with a reply channel independent of native logging."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def load_library():
    """Load OVPHYSX_LIB in a native worker, without initializing simulation.

    Linux wheel dependencies are preloaded from the same interpreter's packages.
    SDK overrides retain their explicit runtime search paths. Call only in the
    native process, never in a customer-USD editor process.
    """
    import ctypes
    import importlib.util

    library_path = Path(os.environ['OVPHYSX_LIB'])
    dependency = None
    if sys.platform == 'linux':
        spec = importlib.util.find_spec('ovphysx')
        if spec is not None and spec.origin is not None:
            bundled = Path(spec.origin).parent / 'lib' / 'libovphysx.so'
            if bundled.is_file() and library_path.is_file() and bundled.samefile(library_path):
                stage_spec = importlib.util.find_spec('ovstage')
                if stage_spec is None or stage_spec.origin is None:
                    raise RuntimeError('Install ovstage alongside the selected ovphysx wheel')
                stage_library = Path(stage_spec.origin).parent / 'bin' / 'libovstage.so'
                dependency = ctypes.CDLL(str(stage_library), mode=os.RTLD_GLOBAL)
    library = ctypes.CDLL(str(library_path))
    # Keep the dependency handle alive for as long as this library is retained.
    library._ovstage_dependency = dependency
    return library


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
