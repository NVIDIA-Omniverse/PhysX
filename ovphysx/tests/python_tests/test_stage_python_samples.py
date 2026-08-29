# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import subprocess
from pathlib import Path


def test_wheel_staging_leaves_developer_local_state_behind(tmp_path):
    """NVBug 6543059: .venv/__pycache__/.pytest_cache must never be copied."""
    ovphysx_root = Path(__file__).parents[2]

    result = subprocess.run(
        [
            "cmake",
            f"-DTEST_ROOT={tmp_path}",
            f"-DHELPERS={ovphysx_root / 'scripts' / 'crossplatform_helpers.cmake'}",
            "-P",
            str(ovphysx_root / "tests" / "cmake" / "test_stage_python_samples.cmake"),
        ],
        capture_output=True,
        text=True,
        timeout=120,
    )

    assert result.returncode == 0, result.stdout + result.stderr
