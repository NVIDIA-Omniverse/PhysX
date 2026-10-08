#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-BENCHMARK-001
# @covers AC-1 AC-4
#
# @implements REQ-CAPI-BENCHMARK-005
# @covers AC-1

# Stdlib-only contract helpers for the ovphysx L1B (Authoring / WriteScaling)
# hidden benchmark inventory. The separate hidden high-scale inventory
# (high_scale_inventory.json) is served by the same CLI: it is another exact
# hidden row set, and `check-union` proves that all four inventories (L1B,
# OutputRead, high-scale, ContactReport) cannot shadow each other in the
# shared baseline lookup, before or after the harness decorates a record name.
#
# This module has two jobs:
#
#   1. Parse and validate the two-line-per-row shape that
#      `ovphysx_benchmarks --list --filter='Authoring.*:WriteScaling.*'`
#      prints (see Harness.cpp's --list branch: `printFormatted("%s", name)`
#      followed by `puts(hidden ? " (hidden)" : "")`), and compare it against
#      the frozen row names in producer_inventory.json.
#   2. Parse a `--regenerate` report (or captured stdout) well enough to
#      confirm a positive run published exactly the expected rows, a
#      deliberately-failing row published none, and a run without --hidden
#      selects nothing (the vacuity control). "Published none" is asserted
#      against the metric line itself, not against a positive value: a
#      zero-valued line is still a published record, and `--regenerate` would
#      write it into the baseline as a comparison-disabling zero.
#
# test_benchmark_contract.cmake drives this module as a CLI so the contract's
# total/CPU row counts and --filter patterns come from producer_inventory.json
# rather than hard-coded constants.

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

VALID_DEVICES = ("cpu", "gpu")


class InventoryError(Exception):
    """Base class for every contract violation this module can detect."""


class MalformedListOutputError(InventoryError):
    """--list output does not match the documented two-line-per-row shape."""


class UnhiddenRowError(InventoryError):
    """A row in --list output is not marked hidden."""


class InventoryValidationError(InventoryError):
    """producer_inventory.json itself is malformed or unsafe to use."""


class InventoryMismatchError(InventoryError):
    """Parsed --list (or report) output does not exactly match the inventory."""

    def __init__(self, missing: Sequence[str], extra: Sequence[str], duplicates: Sequence[str]):
        self.missing = list(missing)
        self.extra = list(extra)
        self.duplicates = list(duplicates)
        parts = []
        if self.missing:
            parts.append("missing: " + ", ".join(self.missing))
        if self.extra:
            parts.append("extra: " + ", ".join(self.extra))
        if self.duplicates:
            parts.append("duplicate: " + ", ".join(self.duplicates))
        super().__init__("; ".join(parts) if parts else "inventory mismatch")


@dataclass(frozen=True)
class InventoryRow:
    name: str
    device: str
    # Defaults describe the L1B producer inventory, where every row is hidden and no owner is
    # recorded. The OutputRead inventory sets both: it is a mixed hidden/visible family, and its
    # whole point is that a row cannot exist without someone answering for it.
    hidden: bool = True
    owner: Optional[str] = None


def parse_list_output(text: str) -> List[str]:
    """Parse the two-line-per-row shape emitted by
    `ovphysx_benchmarks --list --filter=...`.

    Line 2*k is the registered benchmark name without device/thread decoration,
    since --list prints the raw registered name. Line 2*k+1 is either the literal
    marker ' (hidden)' or an empty line for a benchmark that is not hidden.

    Returns the ordered list of row names. Every L1B row is required to be
    hidden, so an unhidden row is rejected here rather than passed through
    for a caller to notice later.
    """
    # str.splitlines() does not manufacture a spurious trailing empty element
    # for a single trailing newline, unlike text.split("\n"), so a
    # process-captured "name\n (hidden)\n" round-trips to exactly two lines.
    lines = text.splitlines()

    if len(lines) % 2 != 0:
        raise MalformedListOutputError(
            f"--list output has an odd number of lines ({len(lines)}); " "expected pairs of (name, hidden-marker) lines"
        )

    names: List[str] = []
    for name, hidden in _parse_list_pairs(lines):
        if not hidden:
            raise UnhiddenRowError(
                f"row '{name}' is not hidden; every frozen Authoring./WriteScaling. "
                "row must be registered with Register<T, true>"
            )
        names.append(name)

    return names


def _parse_list_pairs(lines: Sequence[str]) -> List[tuple]:
    """The (name, hidden) pairs behind parse_list_output, without its hidden-only policy."""
    pairs: List[tuple] = []
    for i in range(0, len(lines), 2):
        name = lines[i]
        marker = lines[i + 1]
        if marker not in ("", " (hidden)"):
            raise MalformedListOutputError(
                f"row {i // 2} ('{name}') has an unrecognized marker line: {marker!r}; " "expected '' or ' (hidden)'"
            )
        pairs.append((name, marker == " (hidden)"))
    return pairs


def parse_list_output_with_visibility(text: str) -> List[tuple]:
    """parse_list_output for a family that is legitimately mixed hidden and visible.

    OutputRead is such a family. The CPU rows run on the default pass and the DirectGPU ones are
    hidden behind --hidden. Which is which is part of the contract, because a GPU row that stopped
    being hidden would run on every developer's default pass and fail on a machine with no CUDA
    device, so the flag is returned rather than discarded.
    """
    lines = text.splitlines()
    if len(lines) % 2 != 0:
        raise MalformedListOutputError(
            f"--list output has an odd number of lines ({len(lines)}); expected pairs of (name, hidden-marker) lines"
        )
    return _parse_list_pairs(lines)


def validate_no_substring_collisions(names: Sequence[str]) -> None:
    """BmOutput's baseline lookup (BmOutput.cpp's emit()) matches with
    strstr(), a substring test, not equality. If one frozen row name is a
    substring of another, a baseline record for the shorter name would also
    satisfy a lookup for the longer name, silently comparing the wrong
    benchmark's numbers. Reject that shape outright.
    """
    for a in names:
        for b in names:
            if a != b and a in b:
                raise InventoryValidationError(
                    f"row name '{a}' is a substring of row name '{b}'; "
                    "BmOutput's baseline lookup uses strstr() and would confuse them"
                )


def load_inventory(path: str) -> List[InventoryRow]:
    """Load and validate producer_inventory.json (or a synthetic fixture with
    the same shape): a JSON object with a non-empty 'rows' array of
    {"name": str, "device": "cpu"|"gpu"} objects, unique names, and no
    substring collisions between names.
    """
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        raise InventoryValidationError(f"{path}: not valid JSON: {exc}") from exc

    if not isinstance(data, dict) or "rows" not in data:
        raise InventoryValidationError(f"{path}: expected a JSON object with a 'rows' array")

    raw_rows = data["rows"]
    if not isinstance(raw_rows, list) or not raw_rows:
        raise InventoryValidationError(f"{path}: 'rows' must be a non-empty array")

    require_owner = data.get("require_owner", False)
    if not isinstance(require_owner, bool):
        raise InventoryValidationError(f"{path}: 'require_owner' must be a boolean")

    rows: List[InventoryRow] = []
    seen_names = set()
    for index, entry in enumerate(raw_rows):
        if not isinstance(entry, dict) or "name" not in entry or "device" not in entry:
            raise InventoryValidationError(f"{path}: rows[{index}] must be an object with 'name' and 'device'")
        name = entry["name"]
        device = entry["device"]
        if not isinstance(name, str) or not name:
            raise InventoryValidationError(f"{path}: rows[{index}].name must be a non-empty string")
        if device not in VALID_DEVICES:
            raise InventoryValidationError(
                f"{path}: rows[{index}].device must be one of {VALID_DEVICES}, got {device!r}"
            )
        if name in seen_names:
            raise InventoryValidationError(f"{path}: duplicate row name '{name}'")
        seen_names.add(name)

        hidden = entry.get("hidden", True)
        if not isinstance(hidden, bool):
            raise InventoryValidationError(f"{path}: rows[{index}].hidden must be a boolean")

        owner = entry.get("owner")
        # Enforced only where the inventory asks for it, so producer_inventory.json, a frozen
        # contract whose rows carry no owner, keeps loading unchanged.
        if require_owner and not (isinstance(owner, str) and owner.strip()):
            raise InventoryValidationError(
                f"{path}: rows[{index}] ('{name}') has no 'owner'; this inventory sets "
                "require_owner, so every row must name someone who answers for it"
            )
        rows.append(InventoryRow(name=name, device=device, hidden=hidden, owner=owner))

    validate_no_substring_collisions([row.name for row in rows])
    return rows


def verify_list_against_inventory(inventory_names: Sequence[str], list_names: Sequence[str]) -> None:
    """Raise InventoryMismatchError unless `list_names` is an exact,
    duplicate-free match for `inventory_names`. Passes silently on an exact
    match. `list_names` is expected to already have passed through
    parse_list_output(), which guarantees every entry was marked hidden.
    """
    inventory_set = set(inventory_names)
    list_set = set(list_names)
    counts = Counter(list_names)

    missing = sorted(inventory_set - list_set)
    extra = sorted(list_set - inventory_set)
    duplicates = sorted(name for name, count in counts.items() if count > 1)

    if missing or extra or duplicates:
        raise InventoryMismatchError(missing, extra, duplicates)


# Harness.cpp calls addPostfix() before it runs a benchmark, so the name a row
# is reported under is its registered name plus an optional '_<n>T' thread
# count (n may be negative) and an optional '_GPU'. The contract's passes add
# neither, but a check that only accepts the bare name would silently stop
# finding the row if one did, and "row not found" is the passing answer for
# the suppression checks. BenchmarkFailure.cpp's bmRowHasFailure() accepts the
# same set and has to stay in step with this pattern.
_HARNESS_POSTFIX = r"(?:_-?[0-9]+T)?(?:_GPU)?"


def verify_list_against_inventory_with_visibility(rows: Sequence[InventoryRow], parsed: Sequence[tuple]) -> None:
    """Names and hidden flags must both match the inventory exactly.

    Checking names alone would let a DirectGPU row silently become visible, which runs it on every
    default developer pass and fails on any machine without a CUDA device.
    """
    verify_list_against_inventory([row.name for row in rows], [name for name, _ in parsed])

    expected = {row.name: row.hidden for row in rows}
    wrong = sorted(
        f"{name} (registered {'hidden' if hidden else 'visible'}, inventory says "
        f"{'hidden' if expected[name] else 'visible'})"
        for name, hidden in parsed
        if name in expected and expected[name] != hidden
    )
    if wrong:
        raise InventoryMismatchError([], wrong, [])


def _reported_name_pattern(name: str) -> "re.Pattern[str]":
    """Match `name` at the start of a line, with or without the postfixes the
    harness appends, and only when a whitespace character follows, so that a
    strictly longer neighbouring row name is never mistaken for this one."""
    return re.compile(r"^" + re.escape(name) + _HARNESS_POSTFIX + r"(?=\s)")


_REGENERATE_ROW = re.compile(r"\s+([0-9]+)\s+\(\+/-[0-9]+\)\s+[0-9]+\s*")
_COMPARE_ROW = re.compile(
    r"\s+([0-9]+)\s+\(\s*[+-][0-9]+\.[0-9]+%\)\s+"
    r"[0-9]+\s+\(\s*[+-][0-9]+\.[0-9]+%\)"
    r"(?:\s+(No baseline|FAIL|Skipped))?\s*"
)


def _match_metric_row(line: str, name_pattern: "re.Pattern[str]"):
    """Match one report line against a row-name pattern and either metric row
    shape BmOutput::emit() can print. Returns (average, status) for a metric
    line, or None when the line is not one (a diagnostic, a header, prose).
    `status` is None for a --regenerate line.

    Name matching requires a following whitespace character so a name that is
    a strict prefix of a different, longer row name (e.g. during a substring-
    collision regression) is never miscounted as a match for the shorter one.
    """
    name_match = name_pattern.match(line)
    if not name_match:
        return None

    fields = line[name_match.end() :]
    row_match = _REGENERATE_ROW.fullmatch(fields)
    if row_match is not None:
        return int(row_match.group(1)), None
    row_match = _COMPARE_ROW.fullmatch(fields)
    if row_match is None:
        return None
    return int(row_match.group(1)), row_match.group(2)


def count_positive_rows(report_text: str, names: Sequence[str]) -> Dict[str, int]:
    """Count, for each name, how many lines in a benchmark report (or
    captured stdout) have that registered name, with or without a documented
    harness postfix, a strictly positive average, and a successful regenerate
    or compare row shape. Compare rows ending in FAIL or Skipped are not
    positive metrics.

    The postfix tolerance matters for the two callers that ask whether a row
    published anything positive. verify_absent_row() and verify_zero_rows()
    both read "no match" as a pass, so a decorated row they could not recognize
    would pass vacuously.
    """
    counts = {name: 0 for name in names}
    patterns = {name: _reported_name_pattern(name) for name in names}
    for line in report_text.splitlines():
        for name, pattern in patterns.items():
            row = _match_metric_row(line, pattern)
            if row is None:
                continue
            average, status = row
            if average > 0 and status not in ("FAIL", "Skipped"):
                counts[name] += 1
    return counts


def verify_positive_rows(report_text: str, names: Sequence[str]) -> None:
    """Verify a --hidden --regenerate run produced exactly the expected
    positive rows: every name appears exactly once, nothing is missing, and
    nothing appears more than once.
    """
    candidate_names = sorted(
        {line.split(None, 1)[0] for line in report_text.splitlines() if line.strip()}
    )
    counts = count_positive_rows(report_text, candidate_names)
    expected = set(names)
    actual = {name for name, count in counts.items() if count > 0}
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    duplicated = sorted(name for name, count in counts.items() if count > 1)
    if missing or extra or duplicated:
        raise InventoryMismatchError(missing=missing, extra=extra, duplicates=duplicated)


def verify_absent_row(report_text: str, name: str) -> None:
    """Verify `name` published no positive metric row. This is the negative-run
    check. A row pointed at a missing data directory must route through
    bmRecordFailure() and be suppressed, not merely slow.
    """
    counts = count_positive_rows(report_text, [name])
    if counts[name] != 0:
        raise InventoryError(f"row '{name}' unexpectedly published a positive metric")


def verify_unpublished_rows(report_text: str, names: Sequence[str]) -> None:
    """Verify each name published no metric line of any kind, not even the
    zero-valued one BmOutput emits for a row the harness never executed.

    verify_absent_row() only rejects a positive metric, so a skipped row's
    'name 0 (+/-0) 0' passes it while still writing a zero baseline record.
    A zero baseline permanently disables that row's comparison, because
    BmOutput::performanceDelta() returns 0 whenever either side is 0. An
    unexecuted row must therefore be suppressed outright.

    Pair this with verify_skipped_rows(): on its own, "nothing was published"
    is also true of a run that selected nothing.
    """
    published: List[str] = []
    for name in names:
        pattern = _reported_name_pattern(name)
        for line in report_text.splitlines():
            if _match_metric_row(line, pattern) is not None:
                published.append(name)
                break
    if published:
        raise InventoryError(
            "rows that were not executed published a metric line anyway: " + ", ".join(published)
        )


def verify_skipped_rows(log_text: str, names: Sequence[str]) -> None:
    """Verify each name was selected by the run and then gated out exactly
    once, by matching Harness.cpp's device-gating diagnostic:

        Benchmark <reported name> failed to initialize, skipping.

    This is the half verify_unpublished_rows() cannot supply. A filter typo, a
    renamed row, or an inventory drift would make the run select nothing, and
    "no row published a metric" would then pass vacuously.
    """
    missing: List[str] = []
    duplicated: List[str] = []
    lines = log_text.splitlines()
    for name in names:
        pattern = re.compile(
            r"^Benchmark " + re.escape(name) + _HARNESS_POSTFIX + r" failed to initialize, skipping\.$"
        )
        count = sum(1 for line in lines if pattern.match(line.strip()))
        if count == 0:
            missing.append(name)
        elif count > 1:
            duplicated.append(name)
    if missing or duplicated:
        details = []
        if missing:
            details.append("never selected and skipped: " + ", ".join(sorted(missing)))
        if duplicated:
            details.append("skipped more than once: " + ", ".join(sorted(duplicated)))
        raise InventoryError("device-gated rows did not match the harness skip diagnostic; " + "; ".join(details))


def verify_zero_rows(report_text: str, names: Sequence[str]) -> None:
    """The vacuity control. Running the L1B filter without --hidden must
    select nothing, since every frozen row is hidden. A run that exits 0 with
    zero rows selected must be treated as a contract failure by the caller
    (never printed as "PASSED"). This raises if any named row published a
    metric anyway, which would mean it was not hidden.
    """
    counts = count_positive_rows(report_text, names)
    published = sorted(name for name, n in counts.items() if n > 0)
    if published:
        raise InventoryError("vacuity control failed: rows published without --hidden: " + ", ".join(published))


# Harness.cpp decorates every reported record name with "_<N>T" under
# --threads=N (N may be negative, matching _HARNESS_POSTFIX) and then "_GPU"
# under --forceGpu, so the names that reach the shared baseline file are
# <registered> followed by a tail over this alphabet. Any prefix of a
# decoration counts, because strstr() can stop partway through it.
_DECORATION_TAIL = re.compile(r"_[-0-9TGPU_]*")


def shadows_decorated_record(lookup: str, record: str) -> bool:
    """True when `lookup` could be a strstr() match inside `record` plus some
    harness decoration without being a substring of `record` itself. Because
    every decoration is appended and starts with "_", that can only happen
    when a "_"-initial tail of `lookup` is drawn from the decoration alphabet
    and the remaining head is a suffix of `record`; checking that property
    covers every thread count and the GPU postfix at once."""
    for index, char in enumerate(lookup):
        if char == "_" and _DECORATION_TAIL.fullmatch(lookup[index:]) and record.endswith(lookup[:index]):
            return True
    return False


def validate_inventory_union(inventories: Sequence[Sequence[InventoryRow]]) -> None:
    """Every inventory this contract serves runs against one shared baseline
    lookup, and BmOutput matches it with strstr(). A name that is unique inside
    its own inventory can therefore still shadow a row of another inventory, so
    the union of every registered name must be duplicate-free and
    substring-collision-free, and no registered name may match inside another
    row's record once the harness has decorated that record."""
    names = [row.name for inventory in inventories for row in inventory]
    duplicates = sorted(name for name, count in Counter(names).items() if count > 1)
    if duplicates:
        raise InventoryValidationError(
            "inventory union contains duplicate names: " + ", ".join(duplicates)
        )
    try:
        validate_no_substring_collisions(names)
    except InventoryValidationError as exc:
        raise InventoryValidationError(f"inventory union: {exc}") from exc
    for lookup in names:
        for record in names:
            if lookup != record and shadows_decorated_record(lookup, record):
                raise InventoryValidationError(
                    f"inventory union: row name '{lookup}' would match inside a decorated "
                    f"record of row '{record}' (the harness appends _<N>T and _GPU postfixes); "
                    "BmOutput's baseline lookup uses strstr() and would confuse them"
                )


def _load_names(path: str, device: Optional[str] = None) -> List[str]:
    rows = load_inventory(path)
    if device:
        rows = [row for row in rows if row.device == device]
    return [row.name for row in rows]


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_counts = sub.add_parser("counts", help="print TOTAL=n and CPU=n for the inventory")
    p_counts.add_argument("inventory")

    p_filter = sub.add_parser("filter", help="print a ':'-joined --filter pattern for the inventory")
    p_filter.add_argument("inventory")
    p_filter.add_argument("--device", choices=VALID_DEVICES, default=None)

    p_check_list = sub.add_parser("check-list", help="verify --list output against the inventory")
    p_check_list.add_argument("inventory")
    p_check_list.add_argument("list_output")

    p_check_owned = sub.add_parser(
        "check-list-owned",
        help="verify --list output against a mixed hidden/visible inventory that requires owners",
    )
    p_check_owned.add_argument("inventory")
    p_check_owned.add_argument("list_output")

    p_check_report = sub.add_parser("check-report", help="verify a positive run report")
    p_check_report.add_argument("inventory")
    p_check_report.add_argument("report")
    p_check_report.add_argument("--device", choices=VALID_DEVICES, default=None)

    p_check_absent = sub.add_parser("check-absent", help="verify a row published no metric")
    p_check_absent.add_argument("report")
    p_check_absent.add_argument("row")

    p_check_unpublished = sub.add_parser(
        "check-unpublished", help="verify rows published no metric line at all, not even a zero"
    )
    p_check_unpublished.add_argument("report")
    p_check_unpublished.add_argument("rows", nargs="+")

    p_check_skipped = sub.add_parser(
        "check-skipped", help="verify each row was selected and then device-gated exactly once"
    )
    p_check_skipped.add_argument("log")
    p_check_skipped.add_argument("rows", nargs="+")

    p_check_vacuity = sub.add_parser(
        "check-vacuity", help="verify no inventory row published a metric without --hidden"
    )
    p_check_vacuity.add_argument("inventory")
    p_check_vacuity.add_argument("report")

    p_check_union = sub.add_parser(
        "check-union",
        help="verify the union of several inventories has no duplicate or substring-colliding names, "
        "including collisions that only appear once the harness appends its _<N>T / _GPU postfixes",
    )
    p_check_union.add_argument("inventories", nargs="+")

    args = parser.parse_args(argv)

    try:
        if args.command == "counts":
            rows = load_inventory(args.inventory)
            cpu = sum(1 for row in rows if row.device == "cpu")
            print(f"TOTAL={len(rows)}")
            print(f"CPU={cpu}")
        elif args.command == "filter":
            names = _load_names(args.inventory, args.device)
            print(":".join(names))
        elif args.command == "check-list":
            names = _load_names(args.inventory)
            list_names = parse_list_output(_read(args.list_output))
            verify_list_against_inventory(names, list_names)
            print(f"OK: {len(names)} rows matched, all hidden")
        elif args.command == "check-list-owned":
            rows = load_inventory(args.inventory)
            parsed = parse_list_output_with_visibility(_read(args.list_output))
            verify_list_against_inventory_with_visibility(rows, parsed)
            hidden = sum(1 for row in rows if row.hidden)
            owners = sorted({row.owner for row in rows if row.owner})
            print(
                f"OK: {len(rows)} rows matched ({len(rows) - hidden} visible, {hidden} hidden), "
                f"every row owned by one of: {', '.join(owners)}"
            )
        elif args.command == "check-report":
            names = _load_names(args.inventory, args.device)
            verify_positive_rows(_read(args.report), names)
            print(f"OK: {len(names)} positive rows found")
        elif args.command == "check-absent":
            verify_absent_row(_read(args.report), args.row)
            print(f"OK: '{args.row}' published no metric")
        elif args.command == "check-unpublished":
            verify_unpublished_rows(_read(args.report), args.rows)
            print(f"OK: {len(args.rows)} unexecuted row(s) published no metric line")
        elif args.command == "check-skipped":
            verify_skipped_rows(_read(args.log), args.rows)
            print(f"OK: {len(args.rows)} row(s) were selected and device-gated")
        elif args.command == "check-vacuity":
            names = _load_names(args.inventory)
            verify_zero_rows(_read(args.report), names)
            print("OK: vacuity control confirmed zero rows without --hidden")
        elif args.command == "check-union":
            inventories = [load_inventory(path) for path in args.inventories]
            validate_inventory_union(inventories)
            total = sum(len(inventory) for inventory in inventories)
            print(f"OK: {total} rows across {len(inventories)} inventories are collision-free")
        else:  # pragma: no cover - argparse enforces this
            parser.error(f"unknown command {args.command!r}")
    except InventoryError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
