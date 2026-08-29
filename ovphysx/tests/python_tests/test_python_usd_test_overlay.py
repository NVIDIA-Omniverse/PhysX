# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

import subprocess
from pathlib import Path


def test_python_usd_overlay_restores_pyless_monolith_after_child_failure(tmp_path):
    ovphysx_root = Path(__file__).parents[2]
    helper = ovphysx_root / "scripts" / "python_usd_test_overlay.cmake"
    test_script = ovphysx_root / "tests" / "cmake" / "test_python_usd_test_overlay.cmake"

    result = subprocess.run(
        [
            "cmake",
            f"-DTEST_ROOT={tmp_path}",
            f"-DOVERLAY_HELPER={helper}",
            "-P",
            str(test_script),
        ],
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr
