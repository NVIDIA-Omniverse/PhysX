# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import json
import tempfile
import unittest
from pathlib import Path

import verify_benchmark_diagnostics as diagnostics
import verify_contact_report_inventory as contact
import verify_inventory as shared


THIS_DIR = Path(__file__).resolve().parent
CONTACT_INVENTORY = THIS_DIR / "contact_report_inventory.json"
PRODUCER_INVENTORY = THIS_DIR / "producer_inventory.json"


def _write(directory, name, text):
    path = Path(directory) / name
    path.write_text(text, encoding="utf-8")
    return str(path)


def _list_row(name, hidden=True):
    return name + "\n" + (" (hidden)" if hidden else "") + "\n"


class ContactInventoryDefinitionTests(unittest.TestCase):
    def test_committed_inventory_is_exact_two_row_contract(self):
        rows = contact.load_contact_inventory(str(CONTACT_INVENTORY))
        self.assertEqual(
            [(row.name, row.device) for row in rows],
            [
                ("ContactReport.persistent_pairs_512_step_read_cpu", "cpu"),
                ("ContactReport.persistent_pairs_512_step_read_gpu", "gpu"),
            ],
        )
        self.assertEqual(contact.canonical_report_name(rows[0]), rows[0].name)
        self.assertEqual(
            contact.canonical_report_name(rows[1]),
            "ContactReport.persistent_pairs_512_step_read_gpu_GPU",
        )

    def test_committed_inventory_and_frozen_l1b_union_are_collision_free(self):
        rows = contact.load_and_validate_union(str(CONTACT_INVENTORY), str(PRODUCER_INVENTORY))
        self.assertEqual(len(rows), 2)

    def test_rejects_wrong_row_count(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _write(
                directory,
                "contact.json",
                json.dumps(
                    {
                        "rows": [
                            {
                                "name": "ContactReport.persistent_pairs_512_step_read_cpu",
                                "device": "cpu",
                            }
                        ]
                    }
                ),
            )
            with self.assertRaises(contact.ContactInventoryError):
                contact.load_contact_inventory(path)

    def test_rejects_wrong_literal_name(self):
        with tempfile.TemporaryDirectory() as directory:
            path = _write(
                directory,
                "contact.json",
                json.dumps(
                    {
                        "rows": [
                            {"name": "ContactReport.persistent_pairs_1024_step_read_cpu", "device": "cpu"},
                            {"name": "ContactReport.persistent_pairs_1024_step_read_gpu", "device": "gpu"},
                        ]
                    }
                ),
            )
            with self.assertRaises(contact.ContactInventoryError):
                contact.load_contact_inventory(path)

    def test_union_rejects_exact_duplicate_before_substring_check(self):
        contact_rows = contact.load_contact_inventory(str(CONTACT_INVENTORY))
        producer_rows = [shared.InventoryRow(contact_rows[0].name, "cpu")]
        with self.assertRaises(contact.ContactInventoryError) as context:
            contact.validate_inventory_union(producer_rows, contact_rows)
        self.assertIn("duplicate", str(context.exception))

    def test_union_rejects_cross_inventory_substring(self):
        contact_rows = contact.load_contact_inventory(str(CONTACT_INVENTORY))
        producer_rows = [shared.InventoryRow("ContactReport.persistent_pairs_512", "cpu")]
        with self.assertRaises(contact.ContactInventoryError) as context:
            contact.validate_inventory_union(producer_rows, contact_rows)
        self.assertIn("substring collision", str(context.exception))

    def test_emitted_domain_rejects_exact_duplicate(self):
        with self.assertRaises(contact.ContactInventoryError) as context:
            contact._validate_name_domain(["row_GPU", "row_GPU"], "emitted inventory union")
        self.assertIn("emitted inventory union contains duplicate", str(context.exception))

    def test_emitted_domain_rejects_substring_collision(self):
        with self.assertRaises(contact.ContactInventoryError) as context:
            contact._validate_name_domain(["row_GPU", "row_GPU_extra"], "emitted inventory union")
        self.assertIn("emitted inventory union contains substring collision", str(context.exception))


class ContactListTests(unittest.TestCase):
    def setUp(self):
        self.rows = contact.load_contact_inventory(str(CONTACT_INVENTORY))
        self.exact = "".join(_list_row(row.name) for row in self.rows)

    def test_accepts_exact_two_hidden_rows(self):
        contact.verify_list(self.rows, self.exact)

    def test_rejects_missing_row(self):
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_list(self.rows, _list_row(self.rows[0].name))

    def test_rejects_extra_row(self):
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_list(self.rows, self.exact + _list_row("ContactReport.extra_cpu"))

    def test_rejects_duplicate_row(self):
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_list(self.rows, self.exact + _list_row(self.rows[0].name))

    def test_rejects_unhidden_row(self):
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_list(self.rows, _list_row(self.rows[0].name, False) + _list_row(self.rows[1].name))

    def test_rejects_malformed_list(self):
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_list(self.rows, self.rows[0].name + "\n")


class SharedReportContractTests(unittest.TestCase):
    cpu = "ContactReport.persistent_pairs_512_step_read_cpu"
    gpu = "ContactReport.persistent_pairs_512_step_read_gpu_GPU"

    @staticmethod
    def report(*rows):
        return "\n".join((diagnostics._REPORT_HEADER_1, diagnostics._REPORT_HEADER_2, *rows)) + "\n"

    @staticmethod
    def metric(name, average=100):
        return f"{name} {average} (+/-2) 0"

    def test_accepts_exact_cpu_and_decorated_gpu_rows(self):
        for expected in (self.cpu, self.gpu):
            with self.subTest(expected=expected):
                diagnostics.parse_single_regenerate_row(
                    self.report(self.metric(expected)), expected
                )

    def test_shared_parser_rejects_bad_contact_reports(self):
        cases = {
            "missing": (self.report(), self.cpu),
            "extra": (self.report(self.metric(self.cpu), self.metric("Smoke.extra")), self.cpu),
            "duplicate": (self.report(self.metric(self.cpu), self.metric(self.cpu)), self.cpu),
            "zero": (self.report(self.metric(self.cpu, 0)), self.cpu),
            "malformed": (self.report(f"{self.cpu} not-a-metric"), self.cpu),
            "gpu-undecorated": (
                self.report(self.metric("ContactReport.persistent_pairs_512_step_read_gpu")),
                self.gpu,
            ),
            "cpu-decorated": (self.report(self.metric(self.cpu + "_GPU")), self.cpu),
            "gpu-double-decorated": (self.report(self.metric(self.gpu + "_GPU")), self.gpu),
        }
        for label, (report, expected) in cases.items():
            with self.subTest(label=label), self.assertRaises(diagnostics.DiagnosticsContractError):
                diagnostics.parse_single_regenerate_row(report, expected)

    def test_no_metrics_rejects_even_a_zero_contact_row(self):
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_no_metrics(self.metric(self.cpu, 0))

    def test_no_metrics_accepts_headers_and_diagnostics(self):
        contact.verify_no_metrics(
            "Name Avg Memory\n"
            "Benchmark ContactReport.persistent_pairs_512_step_read_gpu failed to initialize, skipping.\n"
        )

    def test_no_metrics_cli_rejects_missing_report_by_default(self):
        with tempfile.TemporaryDirectory() as directory:
            report = str(Path(directory) / "missing-report.txt")
            self.assertEqual(contact.main(["check-no-metrics", report]), 1)

    def test_no_metrics_cli_allows_explicitly_missing_report(self):
        with tempfile.TemporaryDirectory() as directory:
            report = str(Path(directory) / "missing-report.txt")
            self.assertEqual(contact.main(["check-no-metrics", report, "--allow-missing"]), 0)

    def test_no_metrics_cli_allow_missing_still_checks_existing_report(self):
        with tempfile.TemporaryDirectory() as directory:
            report = _write(directory, "report.txt", self.metric(self.cpu, 0))
            self.assertEqual(contact.main(["check-no-metrics", report, "--allow-missing"]), 1)


class ContactSkipTests(unittest.TestCase):
    def setUp(self):
        self.rows = contact.load_contact_inventory(str(CONTACT_INVENTORY))

    def test_cpu_pass_requires_exact_undecorated_gpu_skip(self):
        log = (
            "Benchmark ContactReport.persistent_pairs_512_step_read_gpu "
            "failed to initialize, skipping.\n"
        )
        contact.verify_skipped_once(self.rows, log, "gpu", "cpu")

    def test_gpu_pass_requires_exact_decorated_cpu_skip(self):
        log = (
            "Benchmark ContactReport.persistent_pairs_512_step_read_cpu_GPU "
            "failed to initialize, skipping.\n"
        )
        contact.verify_skipped_once(self.rows, log, "cpu", "gpu")

    def test_rejects_missing_skip(self):
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_skipped_once(self.rows, "", "gpu", "cpu")

    def test_rejects_duplicate_skip(self):
        line = (
            "Benchmark ContactReport.persistent_pairs_512_step_read_gpu "
            "failed to initialize, skipping.\n"
        )
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_skipped_once(self.rows, line + line, "gpu", "cpu")

    def test_rejects_wrong_gpu_pass_decoration(self):
        log = (
            "Benchmark ContactReport.persistent_pairs_512_step_read_cpu "
            "failed to initialize, skipping.\n"
        )
        with self.assertRaises(contact.ContactInventoryError):
            contact.verify_skipped_once(self.rows, log, "cpu", "gpu")


if __name__ == "__main__":
    unittest.main()
