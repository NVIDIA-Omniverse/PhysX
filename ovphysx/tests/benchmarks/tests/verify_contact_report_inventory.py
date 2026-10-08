#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-004
# @covers AC-1 AC-5

"""Inventory and suppression checks for the hidden ContactReport row pair.

Positive report grammar is owned by ``verify_benchmark_diagnostics.py``. This
module keeps the contact inventory separate from the frozen
Authoring/WriteScaling inventory, checks both raw and canonical emitted-name
unions because baseline lookup uses substring matching, and verifies exact
wrong-device selection without a published metric.
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import verify_inventory as shared


EXPECTED_REGISTERED_NAMES: Dict[str, str] = {
    "cpu": "ContactReport.persistent_pairs_512_step_read_cpu",
    "gpu": "ContactReport.persistent_pairs_512_step_read_gpu",
}


class ContactInventoryError(Exception):
    """The contact inventory or one of its producer artifacts is invalid."""


def canonical_report_name(row: shared.InventoryRow) -> str:
    """Return the exact name the harness emits for this row's device pass."""
    return row.name + ("_GPU" if row.device == "gpu" else "")


def _validate_name_domain(names: Sequence[str], label: str) -> None:
    counts = Counter(names)
    duplicates = sorted(name for name, count in counts.items() if count > 1)
    if duplicates:
        raise ContactInventoryError(f"{label} contains duplicate names: {', '.join(duplicates)}")

    for first_index, first in enumerate(names):
        for second_index, second in enumerate(names):
            if first_index != second_index and first in second:
                raise ContactInventoryError(
                    f"{label} contains substring collision '{first}' in '{second}'; "
                    "BmOutput baseline lookup uses strstr()"
                )


def load_contact_inventory(path: str) -> List[shared.InventoryRow]:
    try:
        rows = shared.load_inventory(path)
    except shared.InventoryError as exc:
        raise ContactInventoryError(str(exc)) from exc

    if len(rows) != 2:
        raise ContactInventoryError(f"contact inventory must contain exactly two rows, got {len(rows)}")

    by_device: Dict[str, shared.InventoryRow] = {}
    for row in rows:
        if row.device in by_device:
            raise ContactInventoryError(f"contact inventory contains more than one {row.device} row")
        by_device[row.device] = row

    if set(by_device) != set(EXPECTED_REGISTERED_NAMES):
        raise ContactInventoryError("contact inventory must contain exactly one CPU row and one GPU row")

    for device, expected_name in EXPECTED_REGISTERED_NAMES.items():
        actual_name = by_device[device].name
        if actual_name != expected_name:
            raise ContactInventoryError(
                f"contact {device} registration must be '{expected_name}', got '{actual_name}'"
            )

    return rows


def validate_inventory_union(
    producer_rows: Sequence[shared.InventoryRow], contact_rows: Sequence[shared.InventoryRow]
) -> None:
    all_rows = list(producer_rows) + list(contact_rows)
    _validate_name_domain([row.name for row in all_rows], "raw inventory union")
    _validate_name_domain([canonical_report_name(row) for row in all_rows], "emitted inventory union")


def load_and_validate_union(contact_path: str, producer_path: str) -> List[shared.InventoryRow]:
    contact_rows = load_contact_inventory(contact_path)
    try:
        producer_rows = shared.load_inventory(producer_path)
    except shared.InventoryError as exc:
        raise ContactInventoryError(str(exc)) from exc
    validate_inventory_union(producer_rows, contact_rows)
    return contact_rows


def verify_list(contact_rows: Sequence[shared.InventoryRow], text: str) -> None:
    try:
        listed = shared.parse_list_output(text)
        shared.verify_list_against_inventory([row.name for row in contact_rows], listed)
    except shared.InventoryError as exc:
        raise ContactInventoryError(str(exc)) from exc


def _row_for_device(contact_rows: Sequence[shared.InventoryRow], device: str) -> shared.InventoryRow:
    matches = [row for row in contact_rows if row.device == device]
    if len(matches) != 1:
        raise ContactInventoryError(f"expected exactly one {device} contact row, got {len(matches)}")
    return matches[0]


def verify_no_metrics(text: str) -> None:
    try:
        shared.verify_unpublished_rows(text, list(EXPECTED_REGISTERED_NAMES.values()))
    except shared.InventoryError as exc:
        raise ContactInventoryError(str(exc)) from exc


def verify_skipped_once(
    contact_rows: Sequence[shared.InventoryRow], text: str, row_device: str, pass_device: str
) -> None:
    row = _row_for_device(contact_rows, row_device)
    reported_name = row.name + ("_GPU" if pass_device == "gpu" else "")
    expected = f"Benchmark {reported_name} failed to initialize, skipping."
    count = sum(1 for line in text.splitlines() if line.strip() == expected)
    if count != 1:
        raise ContactInventoryError(
            f"expected exactly one wrong-device skip diagnostic '{expected}', got {count}"
        )


def _read(path: str) -> str:
    return Path(path).read_text(encoding="utf-8")


def _read_optional(path: str) -> str:
    candidate = Path(path)
    return candidate.read_text(encoding="utf-8") if candidate.exists() else ""


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    definition = sub.add_parser("check-definition")
    definition.add_argument("contact_inventory")
    definition.add_argument("producer_inventory")

    check_list = sub.add_parser("check-list")
    check_list.add_argument("contact_inventory")
    check_list.add_argument("list_output")

    check_no_metrics = sub.add_parser("check-no-metrics")
    check_no_metrics.add_argument("report")
    check_no_metrics.add_argument(
        "--allow-missing",
        action="store_true",
        help="allow an absent report for the expected DirectGPU refusal",
    )

    check_skipped = sub.add_parser("check-skipped")
    check_skipped.add_argument("contact_inventory")
    check_skipped.add_argument("log")
    check_skipped.add_argument("--row-device", choices=shared.VALID_DEVICES, required=True)
    check_skipped.add_argument("--pass-device", choices=shared.VALID_DEVICES, required=True)

    args = parser.parse_args(argv)
    try:
        if args.command == "check-definition":
            rows = load_and_validate_union(args.contact_inventory, args.producer_inventory)
            print(f"OK: {len(rows)} contact rows and union names are exact and collision-free")
        elif args.command == "check-list":
            rows = load_contact_inventory(args.contact_inventory)
            verify_list(rows, _read(args.list_output))
            print("OK: exact two-row hidden ContactReport list")
        elif args.command == "check-no-metrics":
            report_text = _read_optional(args.report) if args.allow_missing else _read(args.report)
            verify_no_metrics(report_text)
            print("OK: no contact metric was published")
        elif args.command == "check-skipped":
            rows = load_contact_inventory(args.contact_inventory)
            verify_skipped_once(rows, _read(args.log), args.row_device, args.pass_device)
            print("OK: wrong-device contact row was selected and skipped exactly once")
        else:  # pragma: no cover - argparse enforces this
            parser.error(f"unknown command {args.command!r}")
    except (ContactInventoryError, OSError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
