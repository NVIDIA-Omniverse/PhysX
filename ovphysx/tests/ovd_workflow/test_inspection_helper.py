# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OVD-INVESTIGATION-001
# @covers AC-4 AC-5 AC-6

"""Focused, runtime-free checks of the installed-inspector JSON boundary."""

import json
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SAMPLE = Path(__file__).resolve().parents[1] / "python_samples" / "omnipvd_inspection.py"


def response(document, returncode=0, stderr=b""):
    output = json.dumps(document, ensure_ascii=False).encode("utf-8")
    return subprocess.CompletedProcess([], returncode, output, stderr)


COMMANDS = (
    (("--version",), {"tool": "ovd-inspect", "evidenceVersion": 1}, "evidenceVersion"),
    (("summary", "capture.ovd"), {"format": "physx.ovd.summary", "version": 1}, "version"),
    (("inspect", "capture.ovd"), {"format": "physx.ovd.inspect", "version": 1}, "version"),
    (("compare", "left.ovd", "right.ovd"),
     {"format": "physx.ovd.inspection-evidence", "version": 1,
      "status": "Equal", "hasEvidence": True}, "version"),
    (("compare", "left.ovd", "right.ovd", "--attribute", "globalPose", "--distance", "position"),
     {"format": "physx.ovd.inspection-evidence", "version": 2,
      "status": "Equal", "hasEvidence": True}, "version"),
)


class InspectionHelperTests(unittest.TestCase):
    def test_missing_schemas_preserve_new_output_path(self):
        sample = runpy.run_path(str(SAMPLE))
        error = FileNotFoundError("PhysX schemas are not installed")
        provider = SimpleNamespace(physx_schema_paths=Mock(side_effect=error))
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            with patch.dict(sys.modules, {"ovphysx.samples.python_samples.schema_support": provider, "ovphysx": None}), \
                    patch.object(subprocess, "run", return_value=response(COMMANDS[0][1])):
                with self.assertRaises(FileNotFoundError) as caught:
                    sample["run"](SAMPLE, output, "cpu")
            self.assertIs(caught.exception, error)
            self.assertFalse(output.exists())

    @staticmethod
    def inspect(result, *arguments):
        inspect_json = runpy.run_path(str(SAMPLE))["inspect_json"]
        with patch.object(subprocess, "run", return_value=result):
            return inspect_json(Path("unused-inspector"), *arguments)

    def test_utf8_identity_is_preserved_without_locale_decoding(self):
        evidence = {"format": "physx.ovd.inspect", "version": 1,
                    "sample": {"identity": {"name": "/World/W\u00fcrfel"}}}
        output = json.dumps(evidence, ensure_ascii=False).encode("utf-8")
        # Model subprocess's Windows locale decoding when text mode is requested.
        # The actual helper must retain the byte response and decode UTF-8 itself.
        def run_inspector(*_arguments, **options):
            stdout = output.decode("cp1252") if options.get("text") else output
            stderr = "" if options.get("text") else b""
            return subprocess.CompletedProcess([], 0, stdout, stderr)

        inspect_json = runpy.run_path(str(SAMPLE))["inspect_json"]
        with patch.object(subprocess, "run", side_effect=run_inspector):
            actual = inspect_json(Path("unused-inspector"), "inspect", "capture.ovd")
        self.assertEqual(actual["sample"]["identity"]["name"], "/World/W\u00fcrfel")

    def test_invalid_utf8_stdout_is_not_replaced(self):
        result = subprocess.CompletedProcess([], 0,
            b'{"format":"physx.ovd.inspect","version":1,"name":"\xff"}', b"")
        with self.assertRaises(UnicodeDecodeError):
            self.inspect(result, "inspect", "capture.ovd")

    def test_invalid_utf8_stderr_preserves_tool_failure(self):
        result = response(COMMANDS[1][1], 2, b"load failed: \xff")
        with self.assertRaisesRegex(RuntimeError, "Inspector failed \\(2\\): load failed: \ufffd"):
            self.inspect(result, "summary", "capture.ovd")

    def test_each_command_requires_an_exact_supported_integer_version(self):
        for arguments, valid, key in COMMANDS:
            for version in (True, False, float(valid[key]), 1.5, str(valid[key]), None, 0, 3):
                with self.subTest(command=arguments[0], version=version):
                    with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence format or version"):
                        self.inspect(response(dict(valid, **{key: version})), *arguments)

    def test_each_command_checks_its_own_tool_or_format(self):
        for arguments, valid, _key in COMMANDS:
            field = "tool" if arguments[0] == "--version" else "format"
            for value in ("wrong-tool-or-format", None):
                with self.subTest(command=arguments[0], field=field, value=value):
                    with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence format or version"):
                        self.inspect(response(dict(valid, **{field: value})), *arguments)

    def test_each_command_rejects_malformed_nonobject_or_trailing_json(self):
        for arguments, valid, _key in COMMANDS:
            valid_output = json.dumps(valid).encode("utf-8")
            for output in (b"", b"not JSON", b"[]", b"null", b"42", b'"result"',
                           valid_output + b" {}", valid_output + b" trailing"):
                with self.subTest(command=arguments[0], output=output):
                    result = subprocess.CompletedProcess([], 0, output, b"")
                    with self.assertRaises((RuntimeError, ValueError)):
                        self.inspect(result, *arguments)

    def test_compare_exit_zero_requires_equal_status(self):
        for status in ("Different", "Incomparable", None):
            with self.subTest(status=status):
                with self.assertRaisesRegex(RuntimeError, "comparison status disagrees with exit code 0"):
                    self.inspect(response(dict(COMMANDS[3][1], status=status)), *COMMANDS[3][0])

    def test_compare_exit_one_preserves_different_v1_evidence(self):
        evidence = {
            "format": "physx.ovd.inspection-evidence", "version": 1,
            "status": "Different", "hasEvidence": True,
            "leftIdentity": {"objectId": "17", "name": "/World/Left"},
            "rightIdentity": {"objectId": "23", "name": "/World/Right"},
            "leftVelocity": [1.25, 2.5, 3.75],
            "rightVelocity": [4.5, 5.25, 6.0],
            "distance": 5.5,
        }
        result = response(evidence, 1, b"comparison differs")
        self.assertEqual(self.inspect(result, "compare", "left.ovd", "right.ovd"), evidence)

    def test_compare_exit_one_preserves_different_v2_values(self):
        evidence = {
            "format": "physx.ovd.inspection-evidence", "version": 2,
            "status": "Different", "hasEvidence": True,
            "measure": {"attributeName": "globalPose",
                        "distanceKind": "TransformPositionEuclidean", "units": "m"},
            "leftFrameCompleted": False, "rightFrameCompleted": True,
            "comparedFrames": "1", "viewerImportSupported": False,
            "units": "m", "tolerance": 0.0, "textWasReplaced": False,
            "componentCount": 3, "leftValue": [3.0, 4.0, 5.0],
            "rightValue": [6.0, 8.0, 5.0], "distance": 5.0,
            "leftIdentity": {"objectId": "17", "name": "/World/Left"},
            "rightIdentity": {"objectId": "23", "name": "/World/Right"},
        }
        result = response(evidence, 1)
        self.assertEqual(self.inspect(result, "compare", "left.ovd", "right.ovd",
                                      "--attribute", "globalPose", "--distance", "position"), evidence)

    def test_exit_one_requires_compare_and_different_status(self):
        different = {"format": "physx.ovd.inspection-evidence", "version": 1,
                     "status": "Different", "hasEvidence": True}
        for arguments, evidence in (
            (("inspect", "capture.ovd"), dict(different, format="physx.ovd.inspect")),
            (("summary", "capture.ovd"), dict(different, format="physx.ovd.summary")),
            (("--version",), {"tool": "ovd-inspect", "evidenceVersion": 1}),
            (("compare", "left.ovd", "right.ovd"), dict(different, status="Equal")),
            (("compare", "left.ovd", "right.ovd"), dict(different, status="Incomparable")),
            (("compare", "left.ovd", "right.ovd"), dict(different, hasEvidence=False)),
            (("compare", "left.ovd", "right.ovd"), dict(different, hasEvidence=1)),
            (("compare", "left.ovd", "right.ovd"), {key: value for key, value in different.items()
                                                    if key != "hasEvidence"}),
            (("compare", "left.ovd", "right.ovd"), dict(different, format="physx.ovd.inspect")),
        ):
            with self.subTest(arguments=arguments, evidence=evidence):
                result = response(evidence, 1)
                with self.assertRaisesRegex(RuntimeError, r"Inspector failed \(1\)|Unsupported inspector evidence"):
                    self.inspect(result, *arguments)

    def test_empty_malformed_and_nonobject_output_fail(self):
        for returncode in (0, 1):
            for output in ("", "not JSON", "{} {}", "[]", "null", "42", '"different"'):
                with self.subTest(returncode=returncode, output=output):
                    result = subprocess.CompletedProcess([], returncode, output.encode("utf-8"), b"")
                    with self.assertRaises((RuntimeError, ValueError, TypeError)):
                        self.inspect(result, "compare", "left.ovd", "right.ovd")

    def test_unsupported_format_or_version_fails(self):
        cases = (
            {"format": "physx.ovd.inspection-evidence", "version": 3, "status": "Different"},
            {"format": "physx.ovd.inspect", "version": 2},
            {"format": "physx.ovd.unknown", "version": 1},
            {"evidenceVersion": 2, "tool": "ovd-inspect"},
            {"version": 1, "status": "Different"},
        )
        for returncode in (0, 1):
            for evidence in cases:
                with self.subTest(returncode=returncode, evidence=evidence):
                    result = response(evidence, returncode)
                    with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence"):
                        self.inspect(result, "compare", "left.ovd", "right.ovd")

    def test_exit_two_fails_even_with_valid_equal_evidence(self):
        evidence = {"format": "physx.ovd.inspection-evidence", "version": 1,
                    "status": "Equal", "hasEvidence": True}
        result = response(evidence, 2)
        with self.assertRaisesRegex(RuntimeError, r"Inspector failed \(2\)"):
            self.inspect(result, "compare", "left.ovd", "right.ovd")

    def test_negative_process_return_fails_with_valid_summary(self):
        result = response(COMMANDS[1][1], -11, b"terminated")
        with self.assertRaisesRegex(RuntimeError, r"Inspector failed \(-11\): terminated"):
            self.inspect(result, *COMMANDS[1][0])

    def test_summary_and_inspect_reject_swapped_sibling_formats(self):
        for requested, sibling in ((1, 2), (2, 1)):
            with self.subTest(command=COMMANDS[requested][0][0]):
                with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence format or version"):
                    self.inspect(response(COMMANDS[sibling][1]), *COMMANDS[requested][0])

    def test_failed_process_uses_stdout_only_diagnostic_fallback(self):
        result = subprocess.CompletedProcess([], 2, b'{"format":"physx.ovd.error"}', b"")
        with self.assertRaisesRegex(RuntimeError, r'Inspector failed \(2\): .*physx\.ovd\.error'):
            self.inspect(result, "summary", "capture.ovd")

    def test_failed_process_fallback_replaces_invalid_utf8(self):
        result = subprocess.CompletedProcess([], -9, b'{"name":"/World/W\xc3', b"")
        with self.assertRaisesRegex(RuntimeError, "Inspector failed \\(-9\\): .*W\ufffd"):
            self.inspect(result, "summary", "capture.ovd")

    def test_unknown_command_rejects_valid_summary_format(self):
        with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence format or version"):
            self.inspect(response(COMMANDS[1][1]), "summarize", "capture.ovd")

    def test_exit_zero_supports_existing_inspection_and_tool_version(self):
        for arguments, evidence in (
            (("--version",), {"tool": "ovd-inspect", "evidenceVersion": 1}),
            (("summary", "capture.ovd"), {"format": "physx.ovd.summary", "version": 1}),
            (("inspect", "capture.ovd"), {"format": "physx.ovd.inspect", "version": 1}),
            (("compare", "left.ovd", "right.ovd"),
             {"format": "physx.ovd.inspection-evidence", "version": 1, "status": "Equal"}),
            (("compare", "left.ovd", "right.ovd", "--attribute", "globalPose", "--distance", "position"),
             {"format": "physx.ovd.inspection-evidence", "version": 2, "status": "Equal"}),
        ):
            with self.subTest(arguments=arguments):
                result = response(evidence)
                self.assertEqual(self.inspect(result, *arguments), evidence)

    def test_compare_rejects_evidence_version_for_other_mode(self):
        legacy = ("compare", "left.ovd", "right.ovd")
        generic = (*legacy, "--attribute", "globalPose", "--distance", "position")
        for returncode, status in ((0, "Equal"), (1, "Different")):
            for arguments, version in ((legacy, 2), (generic, 1)):
                with self.subTest(returncode=returncode, arguments=arguments, version=version):
                    evidence = {"format": "physx.ovd.inspection-evidence", "version": version,
                                "status": status, "hasEvidence": True}
                    result = response(evidence, returncode)
                    with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence"):
                        self.inspect(result, *arguments)

    def test_compare_requires_attribute_and_distance_together(self):
        legacy = ("compare", "left.ovd", "right.ovd")
        for arguments in ((*legacy, "--attribute", "globalPose"),
                          (*legacy, "--distance", "position")):
            for version in (1, 2):
                with self.subTest(arguments=arguments, version=version):
                    evidence = {"format": "physx.ovd.inspection-evidence", "version": version,
                                "status": "Equal"}
                    result = response(evidence)
                    with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence"):
                        self.inspect(result, *arguments)

    def test_compare_rejects_nonstandard_json_numeric_constants(self):
        for constant in ("NaN", "Infinity", "-Infinity"):
            with self.subTest(constant=constant):
                output = ('{"format":"physx.ovd.inspection-evidence","version":2,'
                          '"status":"Different","hasEvidence":true,"distance":' + constant + '}')
                result = subprocess.CompletedProcess([], 1, output.encode("utf-8"), b"")
                with self.assertRaises(ValueError):
                    self.inspect(result, "compare", "left.ovd", "right.ovd",
                                 "--attribute", "globalPose", "--distance", "position")

    def test_compare_rejects_boolean_evidence_version(self):
        evidence = {"format": "physx.ovd.inspection-evidence", "version": True,
                    "status": "Equal"}
        result = response(evidence)
        with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence"):
            self.inspect(result, "compare", "left.ovd", "right.ovd")


if __name__ == "__main__":
    unittest.main()
