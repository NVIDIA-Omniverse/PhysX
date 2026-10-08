# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-001
# @covers AC-1 AC-4

# Focused contract tests for scripts/test_benchmarks_cpp.cmake policy helpers.
# They check that the driver rejects vacuous pass selections, scopes exact-row
# assertions to one pass, propagates --hidden, and counts only successful
# positive report rows.

import json
import subprocess
import tempfile
import unittest
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
OVPHYSX_ROOT = THIS_DIR.parents[2]
HELPER = OVPHYSX_ROOT / "scripts" / "benchmark_driver_common.cmake"
PROBE = THIS_DIR / "benchmark_driver_probe.cmake"


def _cmake_define(name, value):
    return f"-D{name}={value}"


class BenchmarkDriverPolicyTests(unittest.TestCase):
    def _configure(self, *, gpu=True, cpu=True, cpu_st=True, hidden="", expect_rows=""):
        return subprocess.run(
            [
                "cmake",
                _cmake_define("HELPER", HELPER),
                "-DMODE=configure",
                _cmake_define("RUN_GPU", "TRUE" if gpu else "FALSE"),
                _cmake_define("RUN_CPU", "TRUE" if cpu else "FALSE"),
                _cmake_define("RUN_CPU_ST", "TRUE" if cpu_st else "FALSE"),
                _cmake_define("HIDDEN", hidden),
                _cmake_define("EXPECT_ROWS", expect_rows),
                "-P",
                str(PROBE),
            ],
            text=True,
            capture_output=True,
            check=False,
        )

    def test_hidden_one_appends_hidden_argument(self):
        result = self._configure(hidden="1")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("HIDDEN=TRUE", result.stdout)
        self.assertIn("ARGS=--base|--hidden", result.stdout)

    def test_hidden_unset_does_not_append_hidden_argument(self):
        result = self._configure()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("HIDDEN=FALSE", result.stdout)
        self.assertIn("ARGS=--base", result.stdout)
        self.assertNotIn("--hidden", result.stdout)

    def test_empty_pass_selection_fails(self):
        result = self._configure(gpu=False, cpu=False, cpu_st=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("at least one benchmark pass", (result.stdout + result.stderr).lower())

    def test_expect_rows_accepts_exactly_one_enabled_pass(self):
        result = self._configure(gpu=False, cpu=True, cpu_st=False, expect_rows="11")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("ENABLED_PASSES=1", result.stdout)
        self.assertIn("EXPECT_ROWS=11", result.stdout)

    def test_expect_rows_rejects_multiple_enabled_passes(self):
        result = self._configure(gpu=True, cpu=True, cpu_st=False, expect_rows="11")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("exactly one benchmark pass", result.stdout + result.stderr)

    def test_expect_rows_rejects_zero(self):
        result = self._configure(gpu=False, cpu=True, cpu_st=False, expect_rows="0")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("positive integer", result.stdout + result.stderr)

    def test_expect_rows_rejects_non_integer(self):
        result = self._configure(gpu=False, cpu=True, cpu_st=False, expect_rows="eleven")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("positive integer", result.stdout + result.stderr)


class BenchmarkDriverReportTests(unittest.TestCase):
    REGENERATE_REPORT = (
        "Name                                      Avg (us)           Memory\n"
        "----                                      --------           ------\n"
        "Authoring.population_add_drip_cpu          123 (+/-4)           0\n"
        "WriteScaling.velocity_ovstage_4096_cpu     456 (+/-9)           0\n"
    )

    def _run_check(self, report, expected_rows):
        return subprocess.run(
            [
                "cmake",
                _cmake_define("HELPER", HELPER),
                "-DMODE=check-report",
                _cmake_define("REPORT", report),
                _cmake_define("EXPECTED_ROWS", expected_rows),
                "-P",
                str(PROBE),
            ],
            text=True,
            capture_output=True,
            check=False,
        )

    def _check_report(self, report_text, expected_rows):
        with tempfile.TemporaryDirectory() as tmp:
            report = Path(tmp) / "report.txt"
            report.write_text(report_text, encoding="utf-8")
            return self._run_check(report, expected_rows)

    def test_exact_positive_regenerate_row_count_passes(self):
        result = self._check_report(self.REGENERATE_REPORT, 2)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("exactly 2 positive data row(s)", result.stdout)

    def test_wrong_positive_row_count_fails(self):
        result = self._check_report(self.REGENERATE_REPORT, 3)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("expected exactly 3 positive data row(s), found 2", result.stdout + result.stderr)

    def test_empty_report_selection_fails(self):
        result = self._check_report(
            "Name                                      Avg (us)           Memory\n"
            "----                                      --------           ------\n",
            1,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("expected exactly 1 positive data row(s), found 0", result.stdout + result.stderr)

    def test_missing_report_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self._run_check(Path(tmp) / "missing.txt", 1)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("report not found", result.stdout + result.stderr)

    def test_duplicate_data_line_counts_twice(self):
        row = "Authoring.population_add_drip_cpu          123 (+/-4)           0\n"
        result = self._check_report(row + row, 1)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("expected exactly 1 positive data row(s), found 2", result.stdout + result.stderr)

    def test_positive_compare_row_counts(self):
        result = self._check_report(
            "Authoring.population_add_drip_cpu          123 ( +0.0%)         0 ( +0.0%)  No baseline\n",
            1,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_skipped_zero_time_row_does_not_count(self):
        result = self._check_report(
            "Authoring.population_add_drip_cpu            0 ( +0.0%)         0 ( +0.0%)  Skipped\n",
            1,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("expected exactly 1 positive data row(s), found 0", result.stdout + result.stderr)

    def test_status_sidecar_preserves_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            status = Path(tmp) / "cpu.status.json"
            result = subprocess.run(
                [
                    "cmake",
                    _cmake_define("HELPER", HELPER),
                    "-DMODE=write-status",
                    _cmake_define("STATUS_FILE", status),
                    "-DRETURN_CODE=0",
                    "-P",
                    str(PROBE),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(json.loads(status.read_text()), {"return_code": "0"})

    def test_status_sidecar_escapes_loader_status_text(self):
        with tempfile.TemporaryDirectory() as tmp:
            status = Path(tmp) / "gpu.status.json"
            result = subprocess.run(
                [
                    "cmake",
                    _cmake_define("HELPER", HELPER),
                    "-DMODE=write-status",
                    _cmake_define("STATUS_FILE", status),
                    "-DRETURN_CODE=Exit code 0xc0000135",
                    "-P",
                    str(PROBE),
                ],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(
                json.loads(status.read_text()),
                {"return_code": "Exit code 0xc0000135"},
            )


if __name__ == "__main__":
    unittest.main()
