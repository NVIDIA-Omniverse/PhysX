# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Regression for OMPE-91565: importing ovphysx must not set PYTHONHOME.

Older ovphysx wheel-mode bootstrap set ``os.environ["PYTHONHOME"] = sys.prefix``
to prepare ``carb.scripting-python.plugin``. That plugin is no longer shipped or
loaded, but the env mutation outlived it and started polluting child processes.
On Windows in particular it broke libraries that spawn a venv interpreter
(rerun.io was the reported case): the venv launcher honored the inherited
PYTHONHOME and mis-resolved sys.prefix to the parent interpreter's prefix.

The verification is done in a subprocess so the parent's existing PYTHONHOME
state (if any) does not influence the assertion.
"""

import os
import subprocess
import sys
import textwrap


def test_ovphysx_import_does_not_set_pythonhome():
    inner = textwrap.dedent("""
        import os
        assert "PYTHONHOME" not in os.environ, (
            "PYTHONHOME unexpectedly set in subprocess env"
        )
        from ovphysx import PhysX  # noqa: F401  -- triggers native bootstrap
        leaked = os.environ.get("PYTHONHOME")
        assert leaked is None, f"ovphysx bootstrap set PYTHONHOME={leaked!r}"
    """)

    env = os.environ.copy()
    env.pop("PYTHONHOME", None)

    result = subprocess.run(
        [sys.executable, "-c", inner],
        capture_output=True,
        text=True,
        env=env,
        timeout=120,
    )

    assert result.returncode == 0, (
        f"Subprocess rc={result.returncode}.\n" f"stdout:\n{result.stdout}\n" f"stderr:\n{result.stderr}"
    )
