#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-IO-CONTRACT-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Check committed I/O codegen and its tests using only the standard library."""

import os
import subprocess
import sys
from pathlib import Path


def main():
    domain = Path(__file__).resolve().parents[1]
    project = domain.parent.parent
    commands = [
        [str(domain / "tools/gen_contract.py"), "--check"],
        ["-m", "unittest", "discover", "-s", str(domain / "tests"), "-v"],
        [
            "-m",
            "unittest",
            "discover",
            "-s",
            str(project / "tests/python_tests"),
            "-p",
            "test_attribute_metadata.py",
            "-v",
        ],
    ]
    env = {**os.environ, "PYTHONPATH": str(project / "python")}
    for command in commands:
        result = subprocess.run([sys.executable, "-S", *command], env=env, check=False)
        if result.returncode:
            return result.returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
