#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# DEPRECATED (tensor-binding-deprecation): the binding-create-probe validators retire with the binding; generic ones stay.

# @implements REQ-CAPI-BENCHMARK-002
# @covers AC-1 AC-2 AC-3
#
# @implements REQ-CAPI-BENCHMARK-003
# @covers AC-1 AC-2

"""Validate benchmark timing-diagnostics artifacts and binding-row structure."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


class DiagnosticsContractError(Exception):
    """A generated artifact or source contract is invalid."""


_REPORT_HEADER_1 = "Name                                      Avg (us)           Memory"
_REPORT_HEADER_2 = "----                                      --------           ------"
_DIAGNOSTIC_KEYS = {
    "schema_version",
    "cmd",
    "all_steps_count",
    "all_steps_mean_us",
    "all_steps_sd_us",
    "all_steps_min_us",
    "all_steps_max_us",
}
_BINDING_ROW = "Probe.cartpole_4096_tensor_binding_create"
_BINDING_CASES = [
    ("OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32", "actuation force"),
    ("OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32", "position target"),
    ("OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32", "velocity target"),
    ("OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32", "DOF position"),
    ("OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32", "DOF velocity"),
]


def _read_text(path: str) -> str:
    try:
        return Path(path).read_text(encoding="utf-8")
    except OSError as exc:
        raise DiagnosticsContractError(f"could not read '{path}': {exc}") from exc


def _require_plain_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DiagnosticsContractError(f"{field} must be a JSON number")
    number = float(value)
    if not math.isfinite(number):
        raise DiagnosticsContractError(f"{field} must be finite")
    return number


def parse_single_regenerate_row(report_text: str, expected_cmd: str) -> Dict[str, int]:
    """Parse exactly one canonical four-field regenerate row."""
    lines = [line.rstrip() for line in report_text.splitlines() if line.strip()]
    if len(lines) != 3:
        raise DiagnosticsContractError(
            f"report must contain two headers and one result row, found {len(lines)} non-empty lines"
        )
    if lines[0] != _REPORT_HEADER_1 or lines[1] != _REPORT_HEADER_2:
        raise DiagnosticsContractError("report headers changed from the canonical format")

    fields = lines[2].split()
    if len(fields) != 4:
        raise DiagnosticsContractError(
            f"canonical result must have exactly four fields, found {len(fields)}: {fields!r}"
        )
    if fields[0] != expected_cmd:
        raise DiagnosticsContractError(f"report cmd is {fields[0]!r}, expected exact decorated cmd {expected_cmd!r}")
    if re.fullmatch(r"[0-9]+", fields[1]) is None:
        raise DiagnosticsContractError(f"report average is not an unsigned integer: {fields[1]!r}")
    sd_match = re.fullmatch(r"\(\+/-([0-9]+)\)", fields[2])
    if sd_match is None:
        raise DiagnosticsContractError(f"report std-dev field changed shape: {fields[2]!r}")
    if re.fullmatch(r"[0-9]+", fields[3]) is None:
        raise DiagnosticsContractError(f"report memory is not an unsigned integer: {fields[3]!r}")

    average_us = int(fields[1])
    if average_us <= 0:
        raise DiagnosticsContractError(f"{expected_cmd} must publish a positive average")
    return {
        "avg_us": average_us,
        "std_dev": int(sd_match.group(1)),
        "memory_bytes": int(fields[3]),
    }


def parse_single_diagnostic_row(diagnostics_text: str, expected_cmd: str, expected_count: int) -> Dict[str, Any]:
    """Parse one bounded schema-v1 JSONL object and validate its ranges."""
    lines = diagnostics_text.splitlines()
    if len(lines) != 1 or not lines[0].strip():
        raise DiagnosticsContractError(f"diagnostics must contain exactly one non-empty JSONL row, found {len(lines)}")
    try:
        row = json.loads(lines[0])
    except json.JSONDecodeError as exc:
        raise DiagnosticsContractError(f"diagnostics row is not valid JSON: {exc}") from exc
    if not isinstance(row, dict):
        raise DiagnosticsContractError("diagnostics row must be a JSON object")

    keys = set(row)
    if keys != _DIAGNOSTIC_KEYS:
        missing = sorted(_DIAGNOSTIC_KEYS - keys)
        extra = sorted(keys - _DIAGNOSTIC_KEYS)
        raise DiagnosticsContractError(f"diagnostics schema mismatch; missing={missing}, extra={extra}")
    if (
        isinstance(row["schema_version"], bool)
        or not isinstance(row["schema_version"], int)
        or row["schema_version"] != 1
    ):
        raise DiagnosticsContractError("schema_version must be integer 1")
    if row["cmd"] != expected_cmd:
        raise DiagnosticsContractError(
            f"diagnostics cmd is {row['cmd']!r}, expected exact decorated cmd {expected_cmd!r}"
        )

    count = row["all_steps_count"]
    if isinstance(count, bool) or not isinstance(count, int) or count != expected_count:
        raise DiagnosticsContractError(f"all_steps_count must be integer {expected_count}, got {count!r}")
    mean_us = _require_plain_number(row["all_steps_mean_us"], "all_steps_mean_us")
    sd_us = _require_plain_number(row["all_steps_sd_us"], "all_steps_sd_us")
    min_us = _require_plain_number(row["all_steps_min_us"], "all_steps_min_us")
    max_us = _require_plain_number(row["all_steps_max_us"], "all_steps_max_us")

    if mean_us <= 0.0 or min_us <= 0.0:
        raise DiagnosticsContractError(f"{expected_cmd} timing values must be positive")
    if sd_us < 0.0:
        raise DiagnosticsContractError("all_steps_sd_us must be non-negative")
    if not min_us <= mean_us <= max_us:
        raise DiagnosticsContractError(f"timing range is inconsistent: min={min_us}, mean={mean_us}, max={max_us}")
    max_population_sd_us = (max_us - min_us) / 2.0
    if sd_us > max_population_sd_us:
        raise DiagnosticsContractError(
            f"timing spread exceeds half its observed range: sd={sd_us}, "
            f"half_range={max_population_sd_us}"
        )
    return row


def verify_result_artifacts(report_path: str, diagnostics_path: str, expected_cmd: str, expected_count: int) -> None:
    parse_single_regenerate_row(_read_text(report_path), expected_cmd)
    parse_single_diagnostic_row(_read_text(diagnostics_path), expected_cmd, expected_count)


def verify_no_report_row(path: str, expected_cmd: str) -> None:
    report_path = Path(path)
    if not report_path.exists():
        return
    text = _read_text(path)
    pattern = re.compile(r"^" + re.escape(expected_cmd) + r"(?=\s)")
    if any(pattern.match(line) for line in text.splitlines()):
        raise DiagnosticsContractError(f"{expected_cmd} unexpectedly published a report row")


def verify_empty_sidecar(path: str) -> None:
    text = _read_text(path)
    if text != "":
        raise DiagnosticsContractError(f"sidecar '{path}' must be byte-empty, found {len(text)} bytes")


def verify_hidden_binding_list(list_text: str) -> None:
    lines = list_text.splitlines()
    expected = [_BINDING_ROW, " (hidden)"]
    if lines != expected:
        raise DiagnosticsContractError(f"binding --list output must be exactly {expected!r}, got {lines!r}")


def _extract_class_bodies(source: str, class_name: str) -> List[str]:
    declaration = re.compile(r"\bclass\s+" + re.escape(class_name) + r"\s*:\s*public\s+BmBenchmark\s*\{")
    bodies: List[str] = []
    for match in declaration.finditer(source):
        opening = source.find("{", match.start())
        depth = 0
        closing: Optional[int] = None
        for index in range(opening, len(source)):
            character = source[index]
            if character == "{":
                depth += 1
            elif character == "}":
                depth -= 1
                if depth == 0:
                    closing = index
                    break
        if closing is None:
            raise DiagnosticsContractError(f"class {class_name} has no matching closing brace")
        bodies.append(source[opening + 1 : closing])
    return bodies


def _strip_comments(source: str) -> str:
    without_blocks = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", without_blocks)


def _extract_method_body(class_body: str, signature: str) -> str:
    match = re.search(signature, class_body)
    if match is None:
        raise DiagnosticsContractError(f"method matching {signature!r} was not found")
    opening = class_body.find("{", match.end())
    if opening < 0:
        raise DiagnosticsContractError(f"method matching {signature!r} has no body")
    depth = 0
    for index in range(opening, len(class_body)):
        character = class_body[index]
        if character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                return class_body[opening + 1 : index]
    raise DiagnosticsContractError(f"method matching {signature!r} has no matching closing brace")


def verify_harness_one_shot_source(harness_text: str) -> None:
    """Pin the one-shot sidecar path that removes legacy sample triplication."""
    source = _strip_comments(harness_text)
    single_run = re.search(r"const\s+bool\s+singleActualRun\s*=\s*runCount\s*==\s*0\s*;", source)
    triplication = re.search(
        r"times\.push_back\s*\(\s*stepTimeMS\s*\)\s*;\s*"
        r"times\.push_back\s*\(\s*stepTimeMS\s*\)\s*;\s*"
        r"times\.push_back\s*\(\s*stepTimeMS\s*\)\s*;",
        source,
    )
    compatibility_runs = re.search(r"runCount\s*=\s*3\s*;", source)
    extraction = re.search(
        r"if\s*\(\s*singleActualRun\s*\)\s*\{\s*"
        r"actualTimes\.clear\s*\(\s*\)\s*;\s*"
        r"for\s*\(\s*uint32_t\s+s\s*=\s*0\s*;\s*s\s*<\s*stepCount\s*;\s*\+\+s\s*\)\s*\{\s*"
        r"actualTimes\.push_back\s*\(\s*times\s*\[\s*s\s*\*\s*3\s*\]\s*\)\s*;\s*"
        r"\}\s*"
        r"timingDiagnostics\s*\[\s*i\s*\]\s*=\s*computeTimingDiagnostics\s*\(\s*actualTimes\s*\)\s*;",
        source,
        flags=re.DOTALL,
    )
    if single_run is None or triplication is None or compatibility_runs is None or extraction is None:
        raise DiagnosticsContractError(
            "harness one-shot diagnostics must extract one real sample per step from the legacy triplication"
        )
    if not single_run.start() < triplication.start() < compatibility_runs.start() < extraction.start():
        raise DiagnosticsContractError("harness one-shot sample extraction occurs in the wrong order")
    if len(re.findall(r"actualTimes\.push_back\s*\(", source)) != 1:
        raise DiagnosticsContractError("harness one-shot diagnostics must append exactly one actual sample per step")


def verify_binding_source(source_text: str, harness_text: str) -> None:
    registration = re.compile(
        r"Register\s*<\s*CartpoleTensorBindingCreate\s*,\s*true\s*>\s+"
        r"\w+\s*\(\s*\"" + re.escape(_BINDING_ROW) + r"\"\s*\)\s*;"
    )
    if registration.search(source_text) is None:
        raise DiagnosticsContractError("binding row is not registered hidden under its exact name")

    bodies = _extract_class_bodies(source_text, "CartpoleTensorBindingCreate")
    if len(bodies) != 2:
        raise DiagnosticsContractError(
            f"expected CUDA and non-CUDA CartpoleTensorBindingCreate definitions, found {len(bodies)}"
        )

    default_steps = re.compile(r"uint32_t\s+getNbSteps\s*\(\s*\)\s+const\s+override\s*\{\s*return\s+1\s*;\s*\}")
    default_runs = re.compile(r"uint32_t\s+getNbRuns\s*\(\s*\)\s+const\s+override\s*\{\s*return\s+0\s*;\s*\}")
    for index, body in enumerate(bodies):
        uncommented = _strip_comments(body)
        if default_steps.search(uncommented) is None:
            raise DiagnosticsContractError(f"binding class definition {index} does not default to one step")
        if default_runs.search(uncommented) is None:
            raise DiagnosticsContractError(f"binding class definition {index} does not default to zero runs")

    timed_bodies = [body for body in bodies if re.search(r"Time::Second\s+timedStep\s*\(", body)]
    if len(timed_bodies) != 1:
        raise DiagnosticsContractError("CUDA binding class must define exactly one timedStep override")
    timed_body = timed_bodies[0]
    guard = re.search(r"if\s*\(\s*mMeasured\s*\)", timed_body)
    assignment = re.search(r"mMeasured\s*=\s*true\s*;", timed_body)
    timer = re.search(r"\bTime\s+timer\s*;", timed_body)
    if guard is None or assignment is None or timer is None:
        raise DiagnosticsContractError("binding timedStep is missing its second-measurement guard")
    if not guard.start() < assignment.start() < timer.start():
        raise DiagnosticsContractError("binding second-measurement guard must run before the timer starts")
    guarded_region = timed_body[guard.start() : assignment.start()]
    if "throw std::runtime_error" not in guarded_region:
        raise DiagnosticsContractError("binding second-measurement guard does not fail the invocation")

    cases_match = re.search(
        r"constexpr\s+std::array\s*<\s*CartpoleBindingCase\s*,\s*5\s*>\s+"
        r"kCartpoleBindingCases\s*=\s*\{\s*\{(?P<cases>.*?)\}\s*\}\s*;",
        source_text,
        flags=re.DOTALL,
    )
    if cases_match is None:
        raise DiagnosticsContractError("five-entry Cartpole binding case table was not found")
    cases = re.findall(
        r"\{\s*(OVPHYSX_[A-Z0-9_]+)\s*,\s*\"([^\"]+)\"\s*\}",
        cases_match.group("cases"),
    )
    if cases != _BINDING_CASES:
        raise DiagnosticsContractError(f"binding case order changed: expected {_BINDING_CASES!r}, got {cases!r}")

    method = _strip_comments(_extract_method_body(timed_body, r"Time::Second\s+timedStep\s*\(\s*\)\s+override"))
    timed_sequence = re.search(
        r"Time\s+timer\s*;\s*"
        r"for\s*\([^)]*kCartpoleBindingCases\.size\s*\(\s*\)[^)]*\)\s*\{\s*"
        r"const\s+CartpoleBindingCase\s*&\s*bindingCase\s*=\s*"
        r"kCartpoleBindingCases\s*\[\s*bindingIndex\s*\]\s*;\s*"
        r"requireSuccess\s*\(\s*"
        r"mPhysX->createTensorBinding\s*\(\s*mBindings\s*\[\s*bindingIndex\s*\]\s*,\s*"
        r"kArticulationPattern\s*,\s*bindingCase\.tensorType\s*\)\s*,\s*"
        r"bindingCase\.name\s*\)\s*;\s*"
        r"requireSuccess\s*\(\s*"
        r"mBindings\s*\[\s*bindingIndex\s*\]\.spec\s*\(\s*"
        r"mSpecs\s*\[\s*bindingIndex\s*\]\s*\)\s*,\s*bindingCase\.name\s*\)\s*;\s*"
        r"\}\s*"
        r"const\s+Time::Second\s+elapsed\s*=\s*timer\.getElapsedSeconds\s*\(\s*\)\s*;\s*"
        r"validateBindings\s*\(\s*\)\s*;\s*"
        r"return\s+elapsed\s*;",
        method,
        flags=re.DOTALL,
    )
    if timed_sequence is None:
        raise DiagnosticsContractError(
            "binding timer must contain only the ordered create/spec loop and stop before validation"
        )
    if "destroyBindings" in method:
        raise DiagnosticsContractError("binding destruction must remain outside timedStep")

    verify_harness_one_shot_source(harness_text)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    result = sub.add_parser("check-result", help="validate one report row and diagnostics sidecar")
    result.add_argument("report")
    result.add_argument("diagnostics")
    result.add_argument("--expected-cmd", required=True)
    result.add_argument("--expected-count", required=True, type=int)

    no_result = sub.add_parser("check-no-result", help="require no exact report row")
    no_result.add_argument("report")
    no_result.add_argument("--expected-cmd", required=True)

    empty = sub.add_parser("check-empty", help="require an existing byte-empty sidecar")
    empty.add_argument("diagnostics")

    binding_list = sub.add_parser("check-binding-list", help="validate the hidden binding list row")
    binding_list.add_argument("list_output")

    binding_source = sub.add_parser("check-binding-source", help="validate binding and harness one-shot structure")
    binding_source.add_argument("source")
    binding_source.add_argument("harness")

    args = parser.parse_args(argv)
    try:
        if args.command == "check-result":
            verify_result_artifacts(args.report, args.diagnostics, args.expected_cmd, args.expected_count)
            print(f"OK: {args.expected_cmd} has {args.expected_count} all-sample timings and a four-field report")
        elif args.command == "check-no-result":
            verify_no_report_row(args.report, args.expected_cmd)
            print(f"OK: {args.expected_cmd} published no report row")
        elif args.command == "check-empty":
            verify_empty_sidecar(args.diagnostics)
            print(f"OK: '{args.diagnostics}' exists and contains no diagnostics row")
        elif args.command == "check-binding-list":
            verify_hidden_binding_list(_read_text(args.list_output))
            print(f"OK: {_BINDING_ROW} is listed exactly once and hidden")
        elif args.command == "check-binding-source":
            verify_binding_source(_read_text(args.source), _read_text(args.harness))
            print(
                "OK: binding row has the exact five-call timer, one-shot defaults, "
                "second-call guard, and one-sample diagnostics extraction"
            )
        else:  # pragma: no cover - argparse enforces this
            parser.error(f"unknown command {args.command!r}")
    except DiagnosticsContractError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
