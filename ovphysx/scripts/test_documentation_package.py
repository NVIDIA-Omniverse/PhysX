#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-1 AC-2 AC-3

"""Validate both packaged documentation layouts without building native code.

By default --html-docs must name the rendered Sphinx artifact. --portable-only
explicitly limits a local run to portable documentation and relative links.
The fresh temporary layouts are removed on success and on failure.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import tempfile


def validate_packages(project_root: Path, html_docs: Path | None, *, cmake: str = "cmake") -> None:
    project_root = project_root.resolve()
    if html_docs is not None:
        html_docs = html_docs.resolve()
        if not html_docs.is_dir():
            raise ValueError(f"Rendered documentation directory does not exist: {html_docs}")
    else:
        print("Portable-only validation: rendered documentation is not checked.", flush=True)
    with tempfile.TemporaryDirectory(prefix="ovphysx-documentation-") as temporary:
        for artifact in ("sdk", "wheel"):
            package_root = Path(temporary) / artifact
            subprocess.run([
                cmake, f"-DPYTHON={sys.executable}", f"-DPROJECT_ROOT={project_root}",
                f"-DPACKAGE_ROOT={package_root}", f"-DARTIFACT={artifact}",
                f"-DHTML_DOCS={html_docs or ''}", f"-DREQUIRE_HTML={'ON' if html_docs else 'OFF'}",
                "-P", str(project_root / "scripts/stage_documentation.cmake"),
            ], check=True)
            command = [
                sys.executable, str(project_root / "tests/python_tests/wheel_documentation_smoke.py"),
                "--package-root", str(package_root), "--source-docs", str(project_root / "docs"),
            ]
            if artifact == "wheel":
                command.append("--allow-unbundled-header-links")
            if html_docs is not None:
                command.append("--require-rendered-docs")
            subprocess.run(command, check=True)
            print(f"{artifact}: packaged documentation validated", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--cmake", default="cmake")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--html-docs", type=Path, help="Sphinx artifact; required for complete validation")
    mode.add_argument("--portable-only", action="store_true", help="Local subset without rendered-doc validation")
    args = parser.parse_args()
    try:
        validate_packages(args.project_root, args.html_docs, cmake=args.cmake)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Documentation package validation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
