# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Windows regression for OMPE-91565: a venv subprocess must resolve its own prefix.

The reporter hit this with rerun.io running inside a venv: importing ovphysx in
the parent set PYTHONHOME, the venv launcher inherited it, and the venv
interpreter mis-resolved sys.prefix to the parent's prefix instead of the venv
root. This test reproduces that scenario end-to-end and is gated to Windows
because the launcher's PYTHONHOME handling is where the user-visible breakage
manifested.
"""

import os
import subprocess
import sys
import textwrap

import pytest


@pytest.mark.skipif(sys.platform != "win32", reason="venv launcher repro is Windows-specific")
def test_venv_subprocess_resolves_to_venv_prefix():
    script = textwrap.dedent("""
        import os, subprocess, sys, tempfile
        from pathlib import Path

        from ovphysx import PhysX  # noqa: F401  -- triggers native bootstrap

        leaked = os.environ.get("PYTHONHOME")
        assert leaked is None, f"ovphysx bootstrap set PYTHONHOME={leaked!r}"

        with tempfile.TemporaryDirectory() as tmp:
            venv_dir = Path(tmp) / "venv"
            proc = subprocess.run(
                [sys.executable, "-m", "venv", "--without-pip", str(venv_dir)],
                capture_output=True, text=True, timeout=120,
            )
            assert proc.returncode == 0, (
                f"venv creation failed (rc={proc.returncode}).\\n"
                f"stdout:\\n{proc.stdout}\\nstderr:\\n{proc.stderr}"
            )

            venv_py = venv_dir / "Scripts" / "python.exe"
            assert venv_py.exists(), f"venv python missing at {venv_py}"

            out = subprocess.check_output(
                [str(venv_py), "-c", "import sys; print(sys.prefix)"],
                text=True, timeout=60,
            ).strip()

            assert os.path.samefile(out, str(venv_dir)), (
                f"venv sys.prefix resolved to {out!r}, expected venv root {venv_dir!s}. "
                "This usually means PYTHONHOME leaked from the parent."
            )
    """)

    env = os.environ.copy()
    env.pop("PYTHONHOME", None)

    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        env=env,
        timeout=300,
    )

    assert result.returncode == 0, (
        f"Subprocess rc={result.returncode}.\n" f"stdout:\n{result.stdout}\n" f"stderr:\n{result.stderr}"
    )
