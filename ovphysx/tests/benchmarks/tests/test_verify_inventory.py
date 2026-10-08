# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Contract tests for verify_inventory.py. They pin the live producer contract.
#
# The inventory is pinned at eighteen rows: twelve CPU-canonical rows and six
# requested-GPU diagnostic rows across Authoring and WriteScaling.

import json
import tempfile
import unittest
from pathlib import Path

import verify_inventory as vi

THIS_DIR = Path(__file__).resolve().parent
PRODUCER_INVENTORY_PATH = THIS_DIR / "producer_inventory.json"
OUTPUT_READ_INVENTORY_PATH = Path(__file__).resolve().parent / "outputread_inventory.json"


def _write(tmp_dir, name, text):
    path = Path(tmp_dir) / name
    path.write_text(text, encoding="utf-8")
    return str(path)


def _list_line(name, hidden):
    return name + "\n" + (" (hidden)" if hidden else "") + "\n"


class ParseListOutputTests(unittest.TestCase):
    """The harness prints two lines per row: the bare name, then either the
    literal marker ' (hidden)' or an empty line. See Harness.cpp's --list
    branch (printFormatted("%s", name) followed by puts(hidden ? ... : ""))."""

    def test_pairs_the_real_two_line_shape(self):
        text = _list_line("Authoring.population_add_drip_cpu", True) + _list_line(
            "Authoring.population_add_drip_gpu", True
        )
        names = vi.parse_list_output(text)
        self.assertEqual(names, ["Authoring.population_add_drip_cpu", "Authoring.population_add_drip_gpu"])

    def test_empty_text_yields_no_rows(self):
        self.assertEqual(vi.parse_list_output(""), [])

    def test_rejects_odd_line_count(self):
        # A name line with no paired marker line at all (truncated output).
        with self.assertRaises(vi.MalformedListOutputError):
            vi.parse_list_output("Authoring.population_add_drip_cpu\n")

    def test_rejects_unrecognized_marker_line(self):
        text = "Authoring.population_add_drip_cpu\nsome garbage\n"
        with self.assertRaises(vi.MalformedListOutputError):
            vi.parse_list_output(text)

    def test_rejects_unhidden_row(self):
        # Real two-line shape, but the marker line is empty (not hidden).
        text = _list_line("Authoring.population_add_drip_cpu", False)
        with self.assertRaises(vi.UnhiddenRowError) as ctx:
            vi.parse_list_output(text)
        self.assertIn("Authoring.population_add_drip_cpu", str(ctx.exception))

    def test_rejects_unhidden_row_among_hidden_rows(self):
        text = _list_line("Authoring.population_add_drip_cpu", True) + _list_line(
            "Authoring.population_add_packed_cpu", False
        )
        with self.assertRaises(vi.UnhiddenRowError):
            vi.parse_list_output(text)


class LoadInventoryTests(unittest.TestCase):
    def test_loads_valid_inventory(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(
                tmp,
                "inv.json",
                json.dumps(
                    {
                        "rows": [
                            {"name": "Authoring.population_add_drip_cpu", "device": "cpu"},
                            {"name": "Authoring.population_add_drip_gpu", "device": "gpu"},
                        ]
                    }
                ),
            )
            rows = vi.load_inventory(path)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0].name, "Authoring.population_add_drip_cpu")
            self.assertEqual(rows[0].device, "cpu")

    def test_rejects_duplicate_row_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(
                tmp,
                "inv.json",
                json.dumps(
                    {
                        "rows": [
                            {"name": "Authoring.x_cpu", "device": "cpu"},
                            {"name": "Authoring.x_cpu", "device": "cpu"},
                        ]
                    }
                ),
            )
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(path)

    def test_rejects_substring_collision(self):
        # BmOutput's baseline lookup is strstr()-based: a shorter frozen name
        # that is a substring of a longer one would match the wrong baseline
        # record. This must be rejected at load time.
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(
                tmp,
                "inv.json",
                json.dumps(
                    {
                        "rows": [
                            {"name": "Authoring.drain_step_only_cpu", "device": "cpu"},
                            {"name": "Authoring.drain_step_only_cpu_v2", "device": "cpu"},
                        ]
                    }
                ),
            )
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(path)

    def test_rejects_missing_device_field(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "inv.json", json.dumps({"rows": [{"name": "Authoring.x_cpu"}]}))
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(path)

    def test_rejects_invalid_device_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(
                tmp,
                "inv.json",
                json.dumps({"rows": [{"name": "Authoring.x_cpu", "device": "tpu"}]}),
            )
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(path)

    def test_rejects_empty_rows_array(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "inv.json", json.dumps({"rows": []}))
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(path)

    def test_rejects_missing_rows_key(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "inv.json", json.dumps({}))
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(path)


class VerifyListAgainstInventoryTests(unittest.TestCase):
    """The five contractually distinct outcomes: exact match, missing rows,
    extra rows, duplicate rows, and (covered above) an unhidden row, which
    parse_list_output itself rejects before this comparison ever runs."""

    def test_exact_match_passes(self):
        inventory = ["Authoring.a_cpu", "Authoring.b_cpu"]
        listed = ["Authoring.a_cpu", "Authoring.b_cpu"]
        # Must not raise.
        vi.verify_list_against_inventory(inventory, listed)

    def test_detects_missing_row(self):
        inventory = ["Authoring.a_cpu", "Authoring.b_cpu"]
        listed = ["Authoring.a_cpu"]
        with self.assertRaises(vi.InventoryMismatchError) as ctx:
            vi.verify_list_against_inventory(inventory, listed)
        self.assertEqual(ctx.exception.missing, ["Authoring.b_cpu"])
        self.assertEqual(ctx.exception.extra, [])
        self.assertEqual(ctx.exception.duplicates, [])

    def test_detects_extra_row(self):
        inventory = ["Authoring.a_cpu"]
        listed = ["Authoring.a_cpu", "Authoring.unexpected_cpu"]
        with self.assertRaises(vi.InventoryMismatchError) as ctx:
            vi.verify_list_against_inventory(inventory, listed)
        self.assertEqual(ctx.exception.extra, ["Authoring.unexpected_cpu"])
        self.assertEqual(ctx.exception.missing, [])

    def test_detects_duplicate_row(self):
        inventory = ["Authoring.a_cpu", "Authoring.b_cpu"]
        listed = ["Authoring.a_cpu", "Authoring.a_cpu", "Authoring.b_cpu"]
        with self.assertRaises(vi.InventoryMismatchError) as ctx:
            vi.verify_list_against_inventory(inventory, listed)
        self.assertEqual(ctx.exception.duplicates, ["Authoring.a_cpu"])

    def test_missing_and_extra_reported_together(self):
        inventory = ["Authoring.a_cpu", "Authoring.b_cpu"]
        listed = ["Authoring.a_cpu", "Authoring.unexpected_cpu"]
        with self.assertRaises(vi.InventoryMismatchError) as ctx:
            vi.verify_list_against_inventory(inventory, listed)
        self.assertEqual(ctx.exception.missing, ["Authoring.b_cpu"])
        self.assertEqual(ctx.exception.extra, ["Authoring.unexpected_cpu"])


class PositiveReportRowTests(unittest.TestCase):
    """Helpers backing test_benchmark_contract.cmake's positive/negative/
    vacuity checks against a --regenerate report (or captured stdout)."""

    REPORT = (
        "Name                                      Avg (us)           Memory\n"
        "----                                      --------           ------\n"
        "Authoring.population_add_drip_cpu          123 (+/-4)           0\n"
        "Authoring.population_add_packed_cpu         456 (+/-9)           0\n"
    )

    def test_count_positive_rows_counts_exact_prefix_matches(self):
        counts = vi.count_positive_rows(
            self.REPORT, ["Authoring.population_add_drip_cpu", "Authoring.population_add_packed_cpu"]
        )
        self.assertEqual(counts["Authoring.population_add_drip_cpu"], 1)
        self.assertEqual(counts["Authoring.population_add_packed_cpu"], 1)

    def test_count_positive_rows_counts_positive_compare_shape(self):
        report = "Authoring.population_add_drip_cpu          " "123 ( +0.0%)         0 ( +0.0%)  No baseline\n"
        counts = vi.count_positive_rows(report, ["Authoring.population_add_drip_cpu"])
        self.assertEqual(counts["Authoring.population_add_drip_cpu"], 1)

    def test_count_positive_rows_rejects_zero_average(self):
        report = "Authoring.population_add_drip_cpu            0 (+/-4)           0\n"
        counts = vi.count_positive_rows(report, ["Authoring.population_add_drip_cpu"])
        self.assertEqual(counts["Authoring.population_add_drip_cpu"], 0)

    def test_count_positive_rows_rejects_skipped_line(self):
        report = "Authoring.population_add_drip_cpu          " "123 ( +0.0%)         0 ( +0.0%)  Skipped\n"
        counts = vi.count_positive_rows(report, ["Authoring.population_add_drip_cpu"])
        self.assertEqual(counts["Authoring.population_add_drip_cpu"], 0)

    def test_count_positive_rows_rejects_fail_line(self):
        report = "Authoring.population_add_drip_cpu          " "123 ( +0.0%)         0 ( +0.0%)  FAIL\n"
        counts = vi.count_positive_rows(report, ["Authoring.population_add_drip_cpu"])
        self.assertEqual(counts["Authoring.population_add_drip_cpu"], 0)

    def test_count_positive_rows_does_not_confuse_substring_names(self):
        # Regression guard for the same strstr()-shaped hazard as the
        # inventory-load check, this time on the report-parsing side: a name
        # must not be counted as a match for a report line naming a longer
        # row that merely starts with it.
        report = "Authoring.population_add_drip_cpu_v2   999 (+/-1)   0\n"
        counts = vi.count_positive_rows(report, ["Authoring.population_add_drip_cpu"])
        self.assertEqual(counts["Authoring.population_add_drip_cpu"], 0)

    def test_verify_positive_rows_passes_on_exact_set(self):
        names = ["Authoring.population_add_drip_cpu", "Authoring.population_add_packed_cpu"]
        vi.verify_positive_rows(self.REPORT, names)  # must not raise

    def test_verify_positive_rows_rejects_unexpected_positive(self):
        names = ["Authoring.population_add_drip_cpu", "Authoring.population_add_packed_cpu"]
        report = self.REPORT + "Authoring.unexpected_cpu                 777 (+/-7)         0\n"
        with self.assertRaises(vi.InventoryMismatchError) as ctx:
            vi.verify_positive_rows(report, names)
        self.assertEqual(ctx.exception.extra, ["Authoring.unexpected_cpu"])

    def test_verify_positive_rows_detects_missing(self):
        names = ["Authoring.population_add_drip_cpu", "Authoring.never_ran_cpu"]
        with self.assertRaises(vi.InventoryMismatchError) as ctx:
            vi.verify_positive_rows(self.REPORT, names)
        self.assertEqual(ctx.exception.missing, ["Authoring.never_ran_cpu"])

    def test_verify_absent_row_passes_when_row_missing(self):
        vi.verify_absent_row(self.REPORT, "Authoring.population_add_drip_cpu_against_missing_data")

    def test_verify_absent_row_fails_when_row_present(self):
        with self.assertRaises(vi.InventoryError):
            vi.verify_absent_row(self.REPORT, "Authoring.population_add_drip_cpu")

    def test_verify_zero_rows_passes_on_empty_report(self):
        vi.verify_zero_rows("", ["Authoring.population_add_drip_cpu"])

    def test_verify_zero_rows_fails_when_any_row_present(self):
        # This is the vacuity control: running the L1B filter without
        # --hidden must select nothing. If a row leaks through, that is a
        # contract failure, not a silent "PASSED".
        with self.assertRaises(vi.InventoryError):
            vi.verify_zero_rows(self.REPORT, ["Authoring.population_add_drip_cpu"])


class UnpublishedRowTests(unittest.TestCase):
    """A row the harness never executed, either a device-gated row in the other
    pass or a row whose bootstrap failed, must publish no line at all.

    verify_absent_row() is not enough for that. It only rejects a positive
    metric, and the zero-valued regenerate line BmOutput emits for a skipped
    row ('name 0 (+/-0) 0') slips past it while still writing a zero baseline
    record that permanently disables that row's regression comparison
    (BmOutput::performanceDelta() returns 0 whenever either side is 0).
    """

    def test_passes_when_the_row_is_entirely_absent(self):
        report = "Authoring.population_add_drip_cpu          123 (+/-4)           0\n"
        vi.verify_unpublished_rows(report, ["Authoring.population_add_drip_gpu"])

    def test_rejects_a_zero_valued_regenerate_line(self):
        report = "Authoring.population_add_drip_gpu            0 (+/-0)           0\n"
        with self.assertRaises(vi.InventoryError) as ctx:
            vi.verify_unpublished_rows(report, ["Authoring.population_add_drip_gpu"])
        self.assertIn("Authoring.population_add_drip_gpu", str(ctx.exception))

    def test_rejects_a_positive_line(self):
        report = "Authoring.population_add_drip_gpu          123 (+/-4)           0\n"
        with self.assertRaises(vi.InventoryError):
            vi.verify_unpublished_rows(report, ["Authoring.population_add_drip_gpu"])

    def test_rejects_a_skipped_compare_line(self):
        report = "Authoring.population_add_drip_gpu            0 ( +0.0%)         0 ( +0.0%)  Skipped\n"
        with self.assertRaises(vi.InventoryError):
            vi.verify_unpublished_rows(report, ["Authoring.population_add_drip_gpu"])

    def test_ignores_diagnostic_prose_that_merely_mentions_the_row(self):
        # The harness prints 'Benchmark <name> failed to initialize, skipping.'
        # and the suppression notice on stdout. Neither is a published metric
        # line, and a report captured from stdout will contain them.
        report = (
            "Benchmark Authoring.population_add_drip_gpu failed to initialize, skipping.\n"
            "Authoring.population_add_drip_gpu: skipped -- no timing record published\n"
        )
        vi.verify_unpublished_rows(report, ["Authoring.population_add_drip_gpu"])

    def test_does_not_confuse_a_longer_row_name(self):
        report = "Authoring.population_add_drip_gpu_v2         0 (+/-0)           0\n"
        vi.verify_unpublished_rows(report, ["Authoring.population_add_drip_gpu"])

    def test_cli_reports_the_offending_row(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = _write(tmp, "report.txt", "Authoring.a_gpu    0 (+/-0)    0\n")
            self.assertEqual(vi.main(["check-unpublished", report, "Authoring.a_gpu"]), 1)
            self.assertEqual(vi.main(["check-unpublished", report, "Authoring.b_gpu"]), 0)

    def test_catches_a_row_published_under_a_harness_postfix(self):
        """Harness.cpp appends '_<n>T' and/or '_GPU' to the registered name
        before it runs. The contract's current pass adds neither, but a caller
        that passes --forceGpu or a thread count would otherwise get a silent
        pass, because the zero-valued line would be published under a name this
        check no longer recognizes."""
        for postfix in ("_GPU", "_8T", "_8T_GPU", "_-2T", "_-2T_GPU"):
            with self.subTest(postfix=postfix):
                report = "Authoring.a_gpu" + postfix + "    0 (+/-0)    0\n"
                with self.assertRaises(vi.InventoryError):
                    vi.verify_unpublished_rows(report, ["Authoring.a_gpu"])

    def test_still_does_not_confuse_a_neighbouring_row(self):
        """Postfix tolerance must not become substring tolerance."""
        report = (
            "Authoring.a_gpu_v2      0 (+/-0)    0\n"
            "Authoring.a_gpu_extra   0 (+/-0)    0\n"
            "Authoring.a_gpu_8       0 (+/-0)    0\n"
            "Authoring.a_gpu_T       0 (+/-0)    0\n"
        )
        vi.verify_unpublished_rows(report, ["Authoring.a_gpu"])


class DecoratedPositiveRowTests(unittest.TestCase):
    """count_positive_rows() backs verify_absent_row() (the negative run) and
    verify_zero_rows() (the vacuity control). Both ask "did this row publish a
    positive metric?", so a name pattern that misses a documented harness
    postfix answers "no" for a row that did publish. That is a silent false
    pass, the same hole verify_unpublished_rows() closes."""

    def test_counts_a_row_published_under_a_harness_postfix(self):
        for postfix in ("_GPU", "_8T", "_8T_GPU", "_-2T", "_-2T_GPU"):
            with self.subTest(postfix=postfix):
                report = "Authoring.a_cpu" + postfix + "    100 (+/-0)    0\n"
                self.assertEqual(
                    vi.count_positive_rows(report, ["Authoring.a_cpu"]), {"Authoring.a_cpu": 1}
                )

    def test_absent_row_check_catches_a_decorated_positive_row(self):
        report = "Authoring.population_add_drip_cpu_8T    4200 (+/-0)    0\n"
        with self.assertRaises(vi.InventoryError):
            vi.verify_absent_row(report, "Authoring.population_add_drip_cpu")

    def test_vacuity_check_catches_a_decorated_positive_row(self):
        report = "Authoring.a_gpu_GPU    4200 (+/-0)    0\n"
        with self.assertRaises(vi.InventoryError):
            vi.verify_zero_rows(report, ["Authoring.a_gpu"])

    def test_rejects_lookalike_suffixes(self):
        """Postfix tolerance must not become substring tolerance. Only the
        forms Harness.cpp's addPostfix() can produce are the same row."""
        for suffix in ("_v2", "_extra", "_8", "_T", "_GPUX", "_8TX", "_gpu", "_-T"):
            with self.subTest(suffix=suffix):
                report = "Authoring.a_cpu" + suffix + "    100 (+/-0)    0\n"
                self.assertEqual(
                    vi.count_positive_rows(report, ["Authoring.a_cpu"]), {"Authoring.a_cpu": 0}
                )
                vi.verify_absent_row(report, "Authoring.a_cpu")

    def test_still_counts_the_undecorated_name(self):
        report = "Authoring.a_cpu    100 (+/-0)    0\n"
        self.assertEqual(
            vi.count_positive_rows(report, ["Authoring.a_cpu"]), {"Authoring.a_cpu": 1}
        )


class SkippedRowTests(unittest.TestCase):
    """'No row published a metric' is also true when the run selected no rows
    at all, so the suppression check needs the other half, that each requested
    row was constructed and then gated out. Harness.cpp prints exactly one
    'Benchmark <name> failed to initialize, skipping.' per such row."""

    def test_accepts_one_skip_diagnostic_per_row(self):
        log = (
            "Benchmark Authoring.a_gpu failed to initialize, skipping.\n"
            "Benchmark Authoring.b_gpu failed to initialize, skipping.\n"
        )
        vi.verify_skipped_rows(log, ["Authoring.a_gpu", "Authoring.b_gpu"])

    def test_rejects_a_row_that_was_never_selected(self):
        log = "Benchmark Authoring.a_gpu failed to initialize, skipping.\n"
        with self.assertRaises(vi.InventoryError):
            vi.verify_skipped_rows(log, ["Authoring.a_gpu", "Authoring.b_gpu"])

    def test_rejects_an_empty_run(self):
        with self.assertRaises(vi.InventoryError):
            vi.verify_skipped_rows("", ["Authoring.a_gpu"])

    def test_rejects_a_duplicated_skip(self):
        log = (
            "Benchmark Authoring.a_gpu failed to initialize, skipping.\n"
            "Benchmark Authoring.a_gpu failed to initialize, skipping.\n"
        )
        with self.assertRaises(vi.InventoryError):
            vi.verify_skipped_rows(log, ["Authoring.a_gpu"])

    def test_does_not_accept_a_neighbouring_row_as_the_skip(self):
        log = "Benchmark Authoring.a_gpu_v2 failed to initialize, skipping.\n"
        with self.assertRaises(vi.InventoryError):
            vi.verify_skipped_rows(log, ["Authoring.a_gpu"])

    def test_accepts_the_harness_postfixes(self):
        log = "Benchmark Authoring.a_gpu_8T_GPU failed to initialize, skipping.\n"
        vi.verify_skipped_rows(log, ["Authoring.a_gpu"])

    def test_cli_reports_the_missing_row(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = _write(
                tmp, "log.txt", "Benchmark Authoring.a_gpu failed to initialize, skipping.\n"
            )
            self.assertEqual(vi.main(["check-skipped", log, "Authoring.a_gpu"]), 0)
            self.assertEqual(vi.main(["check-skipped", log, "Authoring.b_gpu"]), 1)


class MainCliTests(unittest.TestCase):
    def _inventory_path(self, tmp):
        return _write(
            tmp,
            "inv.json",
            json.dumps(
                {
                    "rows": [
                        {"name": "Authoring.a_cpu", "device": "cpu"},
                        {"name": "Authoring.a_gpu", "device": "gpu"},
                    ]
                }
            ),
        )

    def test_counts_command_reports_total_and_cpu(self):
        with tempfile.TemporaryDirectory() as tmp:
            inv = self._inventory_path(tmp)
            import contextlib
            import io

            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rc = vi.main(["counts", inv])
            self.assertEqual(rc, 0)
            self.assertIn("TOTAL=2", buf.getvalue())
            self.assertIn("CPU=1", buf.getvalue())

    def test_check_list_command_passes_on_exact_match(self):
        with tempfile.TemporaryDirectory() as tmp:
            inv = self._inventory_path(tmp)
            list_output = _write(
                tmp,
                "list.txt",
                _list_line("Authoring.a_cpu", True) + _list_line("Authoring.a_gpu", True),
            )
            rc = vi.main(["check-list", inv, list_output])
            self.assertEqual(rc, 0)

    def test_check_list_command_fails_on_mismatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            inv = self._inventory_path(tmp)
            list_output = _write(tmp, "list.txt", _list_line("Authoring.a_cpu", True))
            rc = vi.main(["check-list", inv, list_output])
            self.assertEqual(rc, 1)

    def test_check_list_command_fails_on_empty_producer_output(self):
        # Empty --list output must remain a hard mismatch rather than a
        # vacuous pass for the expected Authoring inventory.
        with tempfile.TemporaryDirectory() as tmp:
            inv = self._inventory_path(tmp)
            list_output = _write(tmp, "list.txt", "")
            rc = vi.main(["check-list", inv, list_output])
            self.assertEqual(rc, 1)


class ProducerInventoryFixtureTests(unittest.TestCase):
    """Pins the committed producer_inventory.json to the L1B scope:
    sixteen Authoring rows plus two CPU WriteScaling rows."""

    def test_fixture_has_eighteen_rows_twelve_cpu_six_gpu(self):
        rows = vi.load_inventory(str(PRODUCER_INVENTORY_PATH))
        self.assertEqual(len(rows), 18)
        cpu_rows = [r for r in rows if r.device == "cpu"]
        gpu_rows = [r for r in rows if r.device == "gpu"]
        self.assertEqual(len(cpu_rows), 12)
        self.assertEqual(len(gpu_rows), 6)
        write_scaling_names = {r.name for r in rows if r.name.startswith("WriteScaling.")}
        self.assertEqual(
            write_scaling_names,
            {
                "WriteScaling.velocity_ovstage_4096_cpu",
                "WriteScaling.velocity_tensor_4096_cpu",
            },
        )

    def test_fixture_holds_only_authoring_and_write_scaling_families(self):
        # The contract only runs the benchmark binary under the fixed family
        # filter 'Authoring.*:WriteScaling.*'. A row in neither family would
        # still be required by this inventory while being invisible to every
        # filtered --list and run meant to prove it, so the counts above alone
        # are not enough. Sixteen Authoring rows, the two WriteScaling rows
        # pinned above, nothing else.
        rows = vi.load_inventory(str(PRODUCER_INVENTORY_PATH))
        non_write_scaling = [r.name for r in rows if not r.name.startswith("WriteScaling.")]
        self.assertEqual(len(non_write_scaling), 16)
        for name in non_write_scaling:
            self.assertTrue(name.startswith("Authoring."), name)

    def test_fixture_names_are_unique_and_collision_free(self):
        # load_inventory() already runs validate_no_substring_collisions().
        # This test pins that guarantee against the real committed fixture,
        # not only synthetic names.
        rows = vi.load_inventory(str(PRODUCER_INVENTORY_PATH))
        names = [r.name for r in rows]
        self.assertEqual(len(names), len(set(names)))


if __name__ == "__main__":
    unittest.main()


class OwnedInventoryTests(unittest.TestCase):
    """require_owner and the per-row hidden flag keep the OutputRead family from growing
    anonymously, and a DirectGPU row from becoming visible."""

    def _inv(self, tmp, **overrides):
        row = {"name": "OutputRead.readonly_rb_8192_cpu", "device": "cpu", "hidden": False, "owner": "a@b.c"}
        row.update(overrides.pop("row", {}))
        body = {"require_owner": True, "rows": [row]}
        body.update(overrides)
        return _write(tmp, "owned.json", json.dumps(body))

    def test_loads_owner_and_hidden(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows = vi.load_inventory(self._inv(tmp))
            self.assertEqual(rows[0].owner, "a@b.c")
            self.assertFalse(rows[0].hidden)

    def test_rejects_row_with_no_owner(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(self._inv(tmp, row={"owner": None}))

    def test_rejects_blank_owner(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(self._inv(tmp, row={"owner": "   "}))

    def test_owner_optional_without_require_owner(self):
        # producer_inventory.json carries no owners and must keep loading.
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "plain.json", json.dumps({"rows": [{"name": "A.b_cpu", "device": "cpu"}]}))
            rows = vi.load_inventory(path)
            self.assertIsNone(rows[0].owner)
            self.assertTrue(rows[0].hidden)  # defaults to the L1B convention

    def test_rejects_non_boolean_hidden(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(vi.InventoryValidationError):
                vi.load_inventory(self._inv(tmp, row={"hidden": "yes"}))

    def test_visibility_mismatch_is_rejected(self):
        rows = [vi.InventoryRow("A.b_gpu", "gpu", hidden=True, owner="a@b.c")]
        # Registered visible where the inventory says hidden.
        with self.assertRaises(vi.InventoryMismatchError):
            vi.verify_list_against_inventory_with_visibility(rows, [("A.b_gpu", False)])

    def test_visibility_match_passes(self):
        rows = [vi.InventoryRow("A.b_gpu", "gpu", hidden=True, owner="a@b.c")]
        vi.verify_list_against_inventory_with_visibility(rows, [("A.b_gpu", True)])

    def test_parse_with_visibility_accepts_mixed(self):
        parsed = vi.parse_list_output_with_visibility("A.b_cpu\n\nA.c_gpu\n (hidden)\n")
        self.assertEqual(parsed, [("A.b_cpu", False), ("A.c_gpu", True)])

    def test_parse_hidden_only_still_rejects_visible(self):
        with self.assertRaises(vi.UnhiddenRowError):
            vi.parse_list_output("A.b_cpu\n\n")


class OutputReadInventoryFixtureTests(unittest.TestCase):
    """Pins the committed outputread_inventory.json to its matrix of 15 visible CPU rows and
    19 hidden DirectGPU rows, every one owned."""

    def test_fixture_shape(self):
        rows = vi.load_inventory(str(OUTPUT_READ_INVENTORY_PATH))
        self.assertEqual(len(rows), 34)
        self.assertEqual(len([r for r in rows if r.device == "cpu"]), 15)
        self.assertEqual(len([r for r in rows if r.device == "gpu"]), 19)

    def test_every_row_is_owned_and_purposed(self):
        with open(OUTPUT_READ_INVENTORY_PATH, encoding="utf-8") as handle:
            raw = json.load(handle)
        self.assertTrue(raw["require_owner"])
        for row in raw["rows"]:
            self.assertTrue(row.get("owner", "").strip(), row["name"])
            self.assertTrue(row.get("purpose", "").strip(), row["name"])

    def test_cpu_rows_visible_and_gpu_rows_hidden(self):
        # The device suffix and the hidden flag must agree. A visible GPU row runs on every default
        # developer pass and fails on a host with no CUDA device. A hidden CPU row silently drops out
        # of the scheduled CI pass that is supposed to consume it.
        for row in vi.load_inventory(str(OUTPUT_READ_INVENTORY_PATH)):
            self.assertEqual(row.hidden, row.device == "gpu", row.name)

    def test_family_is_only_output_read(self):
        for row in vi.load_inventory(str(OUTPUT_READ_INVENTORY_PATH)):
            self.assertTrue(row.name.startswith("OutputRead."), row.name)


HIGH_SCALE_INVENTORY_PATH = THIS_DIR / "high_scale_inventory.json"
CONTACT_INVENTORY_PATH = THIS_DIR / "contact_report_inventory.json"


class HighScaleInventoryFixtureTests(unittest.TestCase):
    """Pins the committed high_scale_inventory.json: exactly five hidden CPU rows, in two families
    the frozen L1B `Authoring.*:WriteScaling.*` selection cannot see."""

    EXPECTED_NAMES = {
        "WriteScalingHighN.velocity_ovstage_8192_cpu",
        "WriteScalingHighN.velocity_tensor_8192_cpu",
        "WriteScalingHighN.velocity_ovstage_16384_cpu",
        "WriteScalingHighN.velocity_tensor_16384_cpu",
        "RuntimeSpawnScaling.collider_heavy_1280_cpu",
    }

    def test_fixture_is_exactly_the_five_hidden_cpu_rows(self):
        rows = vi.load_inventory(str(HIGH_SCALE_INVENTORY_PATH))
        self.assertEqual({r.name for r in rows}, self.EXPECTED_NAMES)
        self.assertEqual(len(rows), 5)
        for row in rows:
            self.assertEqual(row.device, "cpu", row.name)
            self.assertTrue(row.hidden, row.name)

    def test_fixture_rows_are_outside_the_frozen_l1b_selection(self):
        # The L1B lane selects `Authoring.*_cpu:WriteScaling.*_cpu` and the L1B
        # contract lists `Authoring.*:WriteScaling.*`. A high-scale row matching
        # either glob would silently widen the pinned twelve-CPU-row contract. The
        # harness glob treats `.` literally and `*` as any run, which fnmatch
        # reproduces for these patterns.
        import fnmatch

        for row in vi.load_inventory(str(HIGH_SCALE_INVENTORY_PATH)):
            for pattern in ("Authoring.*", "WriteScaling.*", "Authoring.*_cpu", "WriteScaling.*_cpu"):
                self.assertFalse(fnmatch.fnmatchcase(row.name, pattern), f"{row.name} matches {pattern}")

    def test_fixture_union_with_all_committed_inventories_is_collision_free(self):
        vi.validate_inventory_union(
            [
                vi.load_inventory(str(PRODUCER_INVENTORY_PATH)),
                vi.load_inventory(str(OUTPUT_READ_INVENTORY_PATH)),
                vi.load_inventory(str(HIGH_SCALE_INVENTORY_PATH)),
                vi.load_inventory(str(CONTACT_INVENTORY_PATH)),
            ]
        )


class InventoryUnionTests(unittest.TestCase):
    """All four inventories share one strstr()-matched baseline lookup, so a name that is unique
    within its own file can still shadow a row of another file, before or after the harness
    decorates the record name."""

    @staticmethod
    def _rows(*specs):
        return [vi.InventoryRow(name=name, device=device) for name, device in specs]

    def test_accepts_disjoint_inventories(self):
        vi.validate_inventory_union(
            [self._rows(("Family.a_cpu", "cpu")), self._rows(("Other.b_cpu", "cpu"), ("Other.c_gpu", "gpu"))]
        )

    def test_rejects_a_name_repeated_across_inventories(self):
        with self.assertRaises(vi.InventoryValidationError) as ctx:
            vi.validate_inventory_union([self._rows(("Family.a_cpu", "cpu")), self._rows(("Family.a_cpu", "cpu"))])
        self.assertIn("duplicate", str(ctx.exception))

    def test_rejects_a_substring_collision_across_inventories(self):
        with self.assertRaises(vi.InventoryValidationError):
            vi.validate_inventory_union(
                [self._rows(("Family.a_cpu", "cpu")), self._rows(("Family.a_cpu_v2", "cpu"))]
            )

    def test_rejects_a_collision_through_the_gpu_postfix(self):
        # The harness reports a --forceGpu record as `<name>_GPU`. These two
        # registered names do not collide, but the decorated `Family.x_GPU`
        # contains the whole registered name `x_GPU`, so a lookup of that row
        # would bind the other row's record.
        with self.assertRaises(vi.InventoryValidationError) as ctx:
            vi.validate_inventory_union([self._rows(("Family.x", "gpu")), self._rows(("x_GPU", "cpu"))])
        self.assertIn("decorated", str(ctx.exception))

    def test_rejects_a_collision_through_a_thread_postfix(self):
        # `--threads=N` decorates a record as `<name>_<N>T`, possibly followed
        # by `_GPU`. The check covers every N without enumerating them.
        for shadow in ("x_1T", "x_16T", "x_-2T", "x_8T_GPU", "x_8", "x_"):
            with self.subTest(shadow=shadow), self.assertRaises(vi.InventoryValidationError) as ctx:
                vi.validate_inventory_union([self._rows(("Family.x", "cpu")), self._rows((shadow, "cpu"))])
            self.assertIn("decorated", str(ctx.exception))

    def test_accepts_a_shared_stem_with_a_non_decoration_tail(self):
        # `x_tensor` cannot be produced by decorating `Family.x`. The tail is
        # not drawn from the harness postfix alphabet.
        vi.validate_inventory_union([self._rows(("Family.x", "cpu")), self._rows(("x_tensor", "cpu"))])

    def test_plain_substring_collision_is_reported_before_decoration(self):
        with self.assertRaises(vi.InventoryValidationError) as ctx:
            vi.validate_inventory_union([self._rows(("Family.a", "gpu")), self._rows(("Family.a_GPU", "cpu"))])
        self.assertIn("is a substring of", str(ctx.exception))

    def test_cli_check_union(self):
        with tempfile.TemporaryDirectory() as tmp:
            first = _write(tmp, "first.json", json.dumps({"rows": [{"name": "Family.a_cpu", "device": "cpu"}]}))
            second = _write(tmp, "second.json", json.dumps({"rows": [{"name": "Other.b_cpu", "device": "cpu"}]}))
            colliding = _write(
                tmp, "colliding.json", json.dumps({"rows": [{"name": "Family.a_cpu_v2", "device": "cpu"}]})
            )
            self.assertEqual(vi.main(["check-union", first, second]), 0)
            self.assertEqual(vi.main(["check-union", first, colliding]), 1)
