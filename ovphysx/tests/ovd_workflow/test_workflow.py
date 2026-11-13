# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OVD-INVESTIGATION-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1

"""Installed-wheel capture/inspection contract; execute with an explicit inspector."""

import argparse
from contextlib import nullcontext
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

# Stands in for ovphysx.population when the sample runs against stub modules: the descriptions it
# would pass, physics first and RENDERING after it when asked.
FAKE_OVPHYSX_POPULATION = SimpleNamespace(
    physics_population_descs=lambda rendering=False: ["physics"] + (["rendering"] if rendering else []))


class WorkflowTests(unittest.TestCase):
    captures = {}

    def test_cpu_only_conflicts_with_require_gpu_before_runtime(self):
        result = subprocess.run(
            [sys.executable, __file__, "--inspector", "absent-inspector",
             "--sample", "absent-sample", "--output-root", "unused-output",
             "--cpu-only", "--require-gpu"],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertRegex(result.stderr.strip().splitlines()[-1],
                         r"argument --(cpu-only|require-gpu): not allowed with argument --(require-gpu|cpu-only)")

    def test_inspector_rejects_unsupported_evidence_version(self):
        sample = runpy.run_path(str(ARGS.sample))
        for document, command in (({"tool": "ovd-inspect", "evidenceVersion": 2}, "--version"),
                                  ({"format": "physx.ovd.inspect", "version": 2}, "inspect")):
            with self.subTest(command=command):
                result = subprocess.CompletedProcess([], 0, json.dumps(document).encode("utf-8"), b"")
                with patch.object(subprocess, "run", return_value=result):
                    with self.assertRaisesRegex(RuntimeError, "Unsupported inspector evidence format or version"):
                        sample["inspect_json"](Path("unused-inspector"), command)

    def test_inspector_exit_two_with_json_is_failure(self):
        sample = runpy.run_path(str(ARGS.sample))
        result = subprocess.CompletedProcess([], 2,
            b'{"format":"physx.ovd.inspection-evidence","version":1,"status":"Equal","hasEvidence":true}', b"")
        with patch.object(subprocess, "run", return_value=result):
            with self.assertRaisesRegex(RuntimeError, "Inspector failed \\(2\\)"):
                sample["inspect_json"](Path("unused-inspector"), "compare")

    def test_live_read_uses_current_synchronous_type_api(self):
        sample = runpy.run_path(str(ARGS.sample))
        rigid_body, all_objects = object(), object()
        names = ("/World/Cube1", "/World/Cube2", "/World/KinematicCube")
        vectors = {
            "position": ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (0.0, 2.0, 0.0)),
            "linearVelocity": ((0.0, -1.0, 0.0), (0.0, -2.0, 0.0), (0.0, 0.0, 0.0)),
        }

        class Row(list):
            def tolist(self):
                return list(self)

        class CurrentPhysX:
            # Current read accepts a completed Query, not type/scope arguments.
            def read(self, query, attributes, *, metadata=False):
                raise AssertionError("Live snapshots require the synchronous read convenience")

            def read_sync(self, object_type, attribute_names, *, scope=all_objects,
                          prim_paths=None, pattern=None, contacts=None, metadata=False):
                if object_type is not rigid_body or scope is not all_objects:
                    raise ValueError("Expected all rigid bodies, including the fixed kinematic body")
                if len(attribute_names) != 1 or attribute_names[0] not in vectors:
                    raise ValueError("Expected one live vector attribute")
                values = [Row(row) for row in vectors[attribute_names[0]]]
                tensor = SimpleNamespace(numpy=lambda: values, device="cpu")
                # ReadGroup fields from the current runtime contract.
                group = SimpleNamespace(attribute=1, object_type=rigid_body, ordinal=1,
                    is_array=False, is_delete=False, semantic=0, prim_list=17,
                    prim_offset=0, prim_count=3, prim_index_map=None, index_map=None,
                    layout_generation=1, write_floor_ordinal=1, tensors=[tensor],
                    cuda_stream=0, cuda_wait_event=0)
                return nullcontext(SimpleNamespace(groups=[group]))

        class Paths:
            def get_path_strings(self, prim_list):
                if prim_list != 17:
                    raise ValueError("Unknown primitive list")
                return names

        fake_ovstage = SimpleNamespace(PathDictionary=lambda stage: nullcontext(Paths()))
        fake_types = SimpleNamespace(ObjectScope=SimpleNamespace(ALL=all_objects),
                                     SimObjectType=SimpleNamespace(RIGID_BODY=rigid_body))
        with patch.dict(sys.modules, {"ovstage": fake_ovstage, "ovphysx": SimpleNamespace(),
                                      "ovphysx.types": fake_types}):
            try:
                actual = sample["read_live"](CurrentPhysX(), object())
            except TypeError as error:
                self.fail(f"Live read is incompatible with the current runtime signature: {error}")
        self.assertEqual(actual, {"bodies": {
            "/World/Cube1": {"position": [1.0, 2.0, 3.0], "linearVelocity": [0.0, -1.0, 0.0]},
            "/World/Cube2": {"position": [4.0, 5.0, 6.0], "linearVelocity": [0.0, -2.0, 0.0]},
            "/World/KinematicCube": {"position": [0.0, 2.0, 0.0], "linearVelocity": [0.0, 0.0, 0.0]},
        }, "read_devices": ["cpu"]})

    def test_live_read_rejects_sparse_or_duplicate_groups(self):
        sample = runpy.run_path(str(ARGS.sample))
        names = ("/World/Cube1", "/World/Cube2", "/World/KinematicCube")
        rigid_body, all_objects = object(), object()

        class Row(list):
            def tolist(self):
                return list(self)

        def group(**changes):
            tensor = SimpleNamespace(numpy=lambda: [Row((0.0, 0.0, 0.0)) for _ in names], device="cpu")
            fields = dict(attribute=1, object_type=rigid_body, ordinal=1, is_array=False,
                is_delete=False, semantic=0, prim_list=17, prim_offset=0, prim_count=3,
                prim_index_map=None, index_map=None, layout_generation=1,
                write_floor_ordinal=1, tensors=[tensor], cuda_stream=0, cuda_wait_event=0)
            fields.update(changes)
            return SimpleNamespace(**fields)

        paths = SimpleNamespace(get_path_strings=lambda prim_list: names)
        fake_ovstage = SimpleNamespace(PathDictionary=lambda stage: nullcontext(paths))
        fake_types = SimpleNamespace(ObjectScope=SimpleNamespace(ALL=all_objects),
                                     SimObjectType=SimpleNamespace(RIGID_BODY=rigid_body))
        for label, groups, diagnostic in (
            ("index_map", [group(index_map=object())], "dense live read groups"),
            ("prim_index_map", [group(prim_index_map=object())], "dense live read groups"),
            ("duplicate", [group(), group()], "Ambiguous live body"),
        ):
            class CurrentPhysX:
                def read_sync(self, object_type, attribute_names, *, scope=all_objects,
                              prim_paths=None, pattern=None, contacts=None, metadata=False):
                    if object_type is not rigid_body or scope is not all_objects or attribute_names not in (
                        ["position"], ["linearVelocity"],
                    ):
                        raise ValueError("Unexpected live vector read")
                    return nullcontext(SimpleNamespace(groups=groups))

            with self.subTest(label=label):
                with patch.dict(sys.modules, {"ovstage": fake_ovstage, "ovphysx": SimpleNamespace(),
                                              "ovphysx.types": fake_types}):
                    with self.assertRaisesRegex(RuntimeError, diagnostic):
                        sample["read_live"](CurrentPhysX(), object())

    def run_sample(self, output, *options, inspector=None):
        return subprocess.run(
            [sys.executable, str(ARGS.sample), "--inspector", str(inspector or ARGS.inspector),
             "--output-dir", str(output), *options],
            cwd=WORK, capture_output=True, text=True, timeout=120,
        )

    def verify_capture(self, mode):
        if mode == "gpu" and ARGS.cpu_only:
            self.skipTest("Focused CPU-only phase")
        output = WORK / mode
        if mode not in self.captures:
            if mode == "gpu":
                import warp as wp
                if not wp.is_cuda_available() and not ARGS.require_gpu:
                    self.skipTest("No CUDA device; CPU evidence remains required")
            result = self.run_sample(output, "--device", mode)
            (WORK / f"{mode}.log").write_text(result.stdout + result.stderr, encoding="utf-8")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.captures[mode] = json.loads((output / "evidence.json").read_text())
        evidence = self.captures[mode]
        self.assertEqual(evidence["device"], mode)
        self.assertIs(evidence["cpu_mode"], mode == "cpu")
        # These labels check report format, not lifecycle execution. The live
        # post-inspection motion and real failed-stop test provide that evidence.
        self.assertEqual(evidence["inspection_phase"], "stopped_before_teardown")
        self.assertEqual([item["step"] for item in evidence["live"]], list(range(1, 11)))
        self.assertEqual(evidence["post_inspection"]["step"], 11)
        self.assertEqual(evidence["capture_sha256"], hashlib.sha256((output / "capture.ovd").read_bytes()).hexdigest())
        self.assertEqual(len(evidence["live"]), 10)
        version = subprocess.run([str(ARGS.inspector), "--version"],
                                 capture_output=True, text=True, timeout=30, check=True)
        self.assertEqual(evidence["inspector"], json.loads(version.stdout))
        self.assertEqual(evidence["inspector"]["tool"], "ovd-inspect")
        self.assertEqual(evidence["inspector"]["evidenceVersion"], 1)
        self.assertRegex(evidence["inspector"]["sdkVersion"], r"^\d+\.\d+\.\d+$")
        self.assertTrue(evidence["inspector"]["build"])
        contexts = evidence["summary"]["contexts"]
        self.assertEqual(len(contexts), 1)
        context = contexts[0]
        self.assertTrue(context["sceneMapped"])
        self.assertGreater(int(context["context"]), 1)
        self.assertEqual(int(context["recordingSegment"]), 0)
        identities = []
        for name in ("/World/Cube1", "/World/Cube2", "/World/KinematicCube"):
            recorded = evidence["objects"][name]
            self.assertEqual(len(recorded), 2)
            identity = recorded[0]["pose"]["sample"]["identity"]
            identities.append(identity["objectId"])
            self.assertEqual(identity["name"], name)
            self.assertEqual(identity["className"], "PxRigidDynamic")
            self.assertGreater(int(identity["objectId"]), 0)
            self.assertGreater(int(identity["externalHandle"]), 0)
            self.assertEqual(identity["recordingSegment"], context["recordingSegment"])
            for frame, record in zip((2, 20), recorded):
                for attribute, attribute_name in (("pose", "globalPose"), ("velocity", "linearVelocity")):
                    self.assertEqual(record[attribute]["attribute"], attribute_name)
                    sample = record[attribute]["sample"]
                    self.assertEqual(sample["identity"], identity)
                    self.assertEqual(sample["requested"], {
                        "recordingSegment": context["recordingSegment"],
                        "context": context["context"], "recordedFrame": str(frame),
                    })
                    # This producer writes attributes in raw context 1 without
                    # raw frame markers; mapped scene frames are a separate clock.
                    self.assertEqual(sample["rawContext"], "1")
                    self.assertIsNone(sample["rawFrame"])
                    self.assertGreater(int(sample["commandOrdinal"]), 0)
                    self.assertLessEqual(int(sample["commandOrdinal"]), int(evidence["summary"]["commands"]))
        self.assertEqual(len(set(identities)), 3)
        for name in ("/World/Cube1", "/World/Cube2"):
            live = [frame["bodies"][name] for frame in evidence["live"]]
            self.assertLess(live[-1]["position"][1], live[0]["position"][1] - 0.1)
            self.assertLess(live[-1]["linearVelocity"][1], -1)
            recorded = evidence["objects"][name]
            self.assertLess(recorded[-1]["pose"]["position"][1], recorded[0]["pose"]["position"][1] - 0.1)
            self.assertLess(recorded[-1]["velocity"]["value"][1], -1)
            for frame, live_index, sample in ((2, 0, recorded[0]), (20, -1, recorded[-1])):
                for attribute, key, live_key in (("pose", "position", "position"),
                                                 ("velocity", "value", "linearVelocity")):
                    item = sample[attribute]
                    self.assertEqual(item["sample"]["state"], "Exact")
                    self.assertFalse(item["sample"]["preMembership"])
                    self.assertEqual(int(item["sample"]["requested"]["recordedFrame"]), frame)
                    self.assertTrue(item["sample"]["hasSourceFrame"])
                    self.assertEqual(int(item["sample"]["source"]["recordedFrame"]), frame)
                    self.assertEqual(item["sample"]["requested"]["context"],
                                     item["sample"]["source"]["context"])
                    self.assertEqual(item["sample"]["requested"]["recordingSegment"],
                                     item["sample"]["source"]["recordingSegment"])
                    recorded_vector = item[key]
                    live_vector = evidence["live"][live_index]["bodies"][name][live_key]
                    self.assertEqual(len(recorded_vector), 3)
                    self.assertEqual(len(live_vector), 3)
                    for actual, expected in zip(recorded_vector, live_vector):
                        self.assertAlmostEqual(actual, expected, delta=1.0e-5)
            for attribute in ("pose", "velocity"):
                self.assertLess(int(recorded[0][attribute]["sample"]["commandOrdinal"]),
                                int(recorded[-1][attribute]["sample"]["commandOrdinal"]))
            self.assertLess(evidence["post_inspection"]["bodies"][name]["position"][1],
                            evidence["live"][-1]["bodies"][name]["position"][1])
        for frame in evidence["live"]:
            self.assertTrue(frame["read_devices"])
            self.assertTrue(all(device.startswith("cuda:") if mode == "gpu" else device == "cpu"
                                for device in frame["read_devices"]))
            fixed = frame["bodies"]["/World/KinematicCube"]
            self.assertEqual(fixed["position"], [0.0, 2.0, 0.0])
            self.assertEqual(fixed["linearVelocity"], [0.0, 0.0, 0.0])
        kinematic = evidence["objects"]["/World/KinematicCube"]
        for frame, fixed in zip((2, 20), kinematic):
            self.assertEqual(fixed["pose"]["position"], [0, 2, 0])
            self.assertEqual(fixed["velocity"]["value"], [0, 0, 0])
            for attribute in ("pose", "velocity"):
                sample = fixed[attribute]["sample"]
                self.assertEqual(sample["state"], "Carried")
                self.assertTrue(sample["preMembership"])
                self.assertFalse(sample["hasSourceFrame"])
                self.assertIsNone(sample["source"])
                self.assertEqual(int(sample["requested"]["recordedFrame"]), frame)
                self.assertEqual(sample["commandOrdinal"], kinematic[0][attribute]["sample"]["commandOrdinal"])
                self.assertLess(int(sample["commandOrdinal"]), int(sample["identity"]["lifetimeStartCommand"]))
        self.assertIn("Carried initial snapshot", evidence["kinematic_evidence"])
        self.assertIn("not a fresh write", evidence["kinematic_evidence"])
        self.assertTrue(evidence["post_inspection"]["read_devices"])
        self.assertTrue(all(device.startswith("cuda:") if mode == "gpu" else device == "cpu"
                            for device in evidence["post_inspection"]["read_devices"]))
        self.assertEqual(evidence["post_inspection"]["bodies"]["/World/KinematicCube"]["position"],
                         [0.0, 2.0, 0.0])
        scene = (output / "scene.usda").read_text(encoding="utf-8")
        self.assertEqual("physxScene:enableGPUDynamics" in scene, mode == "gpu")
        self.assertEqual('physxScene:broadphaseType = "GPU"' in scene, mode == "gpu")
        return evidence

    def test_cpu_finalized_capture_matches_live_motion(self):
        self.verify_capture("cpu")

    def test_gpu_finalized_capture_matches_live_motion(self):
        self.verify_capture("gpu")

    def test_cpu_gpu_comparison_preserves_annotation_provenance(self):
        if ARGS.cpu_only:
            self.skipTest("Focused CPU-only phase")
        cpu = self.verify_capture("cpu")
        gpu = self.verify_capture("gpu")
        contexts = [item["summary"]["contexts"][0] for item in (cpu, gpu)]
        report = WORK / "cpu-gpu-comparison.json"
        command = [str(ARGS.inspector), "compare", str(WORK / "cpu/capture.ovd"), str(WORK / "gpu/capture.ovd"),
                   "--left-name", "/World/Cube1", "--right-name", "/World/Cube1",
                   "--left-class", "PxRigidDynamic", "--right-class", "PxRigidDynamic",
                   "--left-segment", contexts[0]["recordingSegment"], "--right-segment", contexts[1]["recordingSegment"],
                   "--left-context", contexts[0]["context"], "--right-context", contexts[1]["context"],
                   "--start", "2", "--end", "20", "--stride", "2", "--offset", "0", "--tolerance", "0.0001",
                   "--units", "m/s", "--annotations", str(report)]
        result = subprocess.run(command, cwd=WORK, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        evidence = json.loads(report.read_text())
        self.assertEqual(evidence["status"], "Equal")
        self.assertEqual(int(evidence["comparedFrames"]), 10)
        self.assertEqual(evidence, json.loads(result.stdout))
        for mode, recording in (("cpu", cpu), ("gpu", gpu)):
            self.assertEqual(hashlib.sha256((WORK / mode / "capture.ovd").read_bytes()).hexdigest(),
                             recording["capture_sha256"])

    def test_real_different_comparison_preserves_values_and_identities(self):
        cpu = self.verify_capture("cpu")
        sample = runpy.run_path(str(ARGS.sample))
        capture = WORK / "cpu" / "capture.ovd"
        context = cpu["summary"]["contexts"][0]
        annotation = WORK / "different-comparison.json"
        evidence = sample["inspect_json"](
            ARGS.inspector, "compare", capture, capture,
            "--left-name", "/World/Cube1", "--right-name", "/World/Cube2",
            "--left-class", "PxRigidDynamic", "--right-class", "PxRigidDynamic",
            "--left-segment", context["recordingSegment"],
            "--right-segment", context["recordingSegment"],
            "--left-context", context["context"], "--right-context", context["context"],
            "--start", 2, "--end", 2, "--stride", 1, "--offset", 0,
            "--tolerance", 0, "--units", "m", "--attribute", "globalPose",
            "--distance", "position", "--annotations", annotation,
        )
        self.assertEqual(evidence["format"], "physx.ovd.inspection-evidence")
        self.assertEqual(evidence["version"], 2)
        self.assertEqual(evidence["status"], "Different")
        self.assertTrue(evidence["hasEvidence"])
        self.assertEqual(evidence["componentCount"], 3)
        self.assertEqual(evidence["leftIdentity"]["name"], "/World/Cube1")
        self.assertEqual(evidence["rightIdentity"]["name"], "/World/Cube2")
        for side, name in (("left", "/World/Cube1"), ("right", "/World/Cube2")):
            expected = cpu["objects"][name][0]["pose"]["position"]
            self.assertEqual(len(evidence[side + "Value"]), 3)
            for actual, recorded in zip(evidence[side + "Value"], expected):
                self.assertAlmostEqual(actual, recorded, delta=1.0e-5)
        self.assertGreater(evidence["distance"], 2.0)
        self.assertEqual(json.loads(annotation.read_text()), evidence)
        self.assertEqual(hashlib.sha256(capture.read_bytes()).hexdigest(), cpu["capture_sha256"])

    def test_existing_output_is_preserved(self):
        output = WORK / "existing"
        output.mkdir()
        capture = output / "capture.ovd"
        capture.write_bytes(b"preserve existing recording")
        result = self.run_sample(output, "--device", "cpu")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("FileExistsError", result.stderr)
        self.assertEqual(capture.read_bytes(), b"preserve existing recording")
        self.assertFalse((output / "evidence.json").exists())

    def test_parent_file_is_an_explicit_directory_failure(self):
        parent = WORK / "file-parent"
        parent.write_text("keep")
        result = self.run_sample(parent / "capture", "--device", "cpu")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Error", result.stderr)
        self.assertEqual(parent.read_text(), "keep")

    def test_missing_inspector_cannot_pass(self):
        result = self.run_sample(WORK / "missing-tool", inspector=WORK / "absent-inspector")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("FileNotFoundError", result.stderr)
        self.assertFalse((WORK / "missing-tool" / "capture.ovd").exists())

    def test_non_inspector_executable_cannot_pass(self):
        result = self.run_sample(WORK / "wrong-tool", inspector=sys.executable)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("JSONDecodeError", result.stderr)
        self.assertFalse((WORK / "wrong-tool").exists())

    def test_stop_failure_prevents_inspection(self):
        output = WORK / "stop-failure"
        output.mkdir()
        code = """
import runpy, sys
from pathlib import Path
from ovphysx import GlobalConfigKey, PhysX, set_global_config
sample = runpy.run_path(sys.argv[1])
PhysX.set_cpu_mode(True)
set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
physx = PhysX()
try:
    try:
        sample['finalize_and_inspect'](physx, None, Path(sys.argv[2]), Path(sys.argv[3]), [], 'cpu', True)
    except RuntimeError as error:
        assert 'stop' in str(error).lower(), str(error)
    else:
        raise AssertionError('Inactive recording stop succeeded')
finally:
    physx.destroy()
"""
        result = subprocess.run([sys.executable, "-c", code, str(ARGS.sample), str(ARGS.inspector), str(output)],
                                cwd=WORK, capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse((output / "evidence.json").exists())

    def test_live_mismatch_fails_without_success_evidence(self):
        cpu = self.verify_capture("cpu")
        sample = runpy.run_path(str(ARGS.sample))
        stopped = SimpleNamespace(stop_recording=lambda: None, is_recording=lambda: False,
                                  step_sync=lambda _: None)
        for live_index in (0, -1):
            for name in ("/World/Cube1", "/World/Cube2", "/World/KinematicCube"):
                for attribute in ("position", "linearVelocity"):
                    with self.subTest(live_index=live_index, name=name, attribute=attribute):
                        output = Path(tempfile.mkdtemp(prefix="live-mismatch-", dir=WORK))
                        (output / "capture.ovd").write_bytes((WORK / "cpu" / "capture.ovd").read_bytes())
                        live = copy.deepcopy(cpu["live"])
                        live[live_index]["bodies"][name][attribute][1] += 1.0e-4
                        with self.assertRaisesRegex(RuntimeError, "disagrees"):
                            sample["finalize_and_inspect"](stopped, None, ARGS.inspector, output, live, "cpu", True)
                        self.assertFalse((output / "evidence.json").exists())

        for component in range(3):
            for sign in (-1, 1):
                with self.subTest(component=component, sign=sign):
                    output = Path(tempfile.mkdtemp(prefix="component-mismatch-", dir=WORK))
                    (output / "capture.ovd").write_bytes((WORK / "cpu" / "capture.ovd").read_bytes())
                    live = copy.deepcopy(cpu["live"])
                    live[0]["bodies"]["/World/Cube1"]["position"][component] += sign * 2.0e-5
                    with self.assertRaisesRegex(RuntimeError, "disagrees"):
                        sample["finalize_and_inspect"](stopped, None, ARGS.inspector, output, live, "cpu", True)
                    self.assertFalse((output / "evidence.json").exists())

    def test_real_inspector_negative_evidence_guards(self):
        cpu = self.verify_capture("cpu")
        sample = runpy.run_path(str(ARGS.sample))
        stopped = SimpleNamespace(stop_recording=lambda: None, is_recording=lambda: False,
                                  step_sync=lambda _: None)
        real_run = subprocess.run
        cases = (
            ("duplicate scene context", "exactly one recorded scene context"),
            ("incomplete frame 20", "Required completed frames"),
            ("carried dynamic sample", "lacks an exact frame write"),
            ("wrong source frame", "Unexpected source frame"),
            ("kinematic not carried", "Kinematic body lacks carried initial state"),
            ("kinematic velocity not carried", "Kinematic body lacks carried initial state"),
            ("kinematic not pre-membership", "Kinematic body lacks carried initial state"),
            ("kinematic source-backed", "Kinematic body lacks carried initial state"),
            ("live kinematic middle step", "Live kinematic body moved"),
            ("dynamic body did not fall", "Dynamic body did not fall"),
            ("source capture mutation", "Inspector modified source capture"),
        )
        for case, diagnostic in cases:
            with self.subTest(case=case):
                output = Path(tempfile.mkdtemp(prefix="real-inspector-negative-", dir=WORK))
                capture = output / "capture.ovd"
                capture.write_bytes((WORK / "cpu" / "capture.ovd").read_bytes())
                live = copy.deepcopy(cpu["live"])
                if case == "live kinematic middle step":
                    live[4]["bodies"]["/World/KinematicCube"]["position"][0] += 2.0e-5
                elif case == "dynamic body did not fall":
                    live[-1]["bodies"]["/World/Cube1"]["position"] = copy.deepcopy(
                        live[0]["bodies"]["/World/Cube1"]["position"])

                def option(command, flag):
                    return str(command[command.index(flag) + 1]) if flag in command else None

                def inspected(command, name, frame, attribute):
                    return (len(command) > 1 and command[1] == "inspect" and
                            option(command, "--name") == name and
                            option(command, "--frame") == str(frame) and
                            option(command, "--attribute") == attribute)

                def run_inspector(command, **kwargs):
                    result = real_run(command, **kwargs)
                    if result.returncode or len(command) < 2 or command[1] not in ("summary", "inspect"):
                        return result
                    document = json.loads(result.stdout)
                    if case == "duplicate scene context" and command[1] == "summary":
                        context = next(item for item in document["contexts"] if item["sceneMapped"])
                        document["contexts"].append(copy.deepcopy(context))
                    elif case == "incomplete frame 20" and command[1] == "summary":
                        context = next(item for item in document["contexts"] if item["sceneMapped"])
                        frame = next(item for item in document["frames"] if
                                     item["frame"]["recordedFrame"] == "20" and
                                     item["frame"]["context"] == context["context"])
                        frame["completed"] = False
                    elif inspected(command, "/World/Cube1", 20, "globalPose"):
                        if case == "carried dynamic sample":
                            document["sample"]["state"] = "Carried"
                        elif case == "wrong source frame":
                            document["sample"]["source"]["recordedFrame"] = "19"
                        elif case == "dynamic body did not fall":
                            document["position"] = copy.deepcopy(live[-1]["bodies"]["/World/Cube1"]["position"])
                    elif inspected(command, "/World/KinematicCube", 20, "globalPose"):
                        if case == "kinematic not carried":
                            document["sample"]["state"] = "Exact"
                        elif case == "kinematic not pre-membership":
                            document["sample"]["preMembership"] = False
                        elif case == "kinematic source-backed":
                            document["sample"]["source"] = copy.deepcopy(document["sample"]["requested"])
                            document["sample"]["hasSourceFrame"] = True
                    elif inspected(command, "/World/KinematicCube", 20, "linearVelocity"):
                        if case == "kinematic velocity not carried":
                            document["sample"]["state"] = "Exact"
                        elif case == "source capture mutation":
                            with capture.open("ab") as stream:
                                stream.write(b"x")
                    return subprocess.CompletedProcess(result.args, result.returncode,
                                                       json.dumps(document).encode("utf-8"), result.stderr)

                with patch.object(subprocess, "run", side_effect=run_inspector):
                    with patch.dict(sample["finalize_and_inspect"].__globals__,
                                    {"read_live": lambda *_: copy.deepcopy(cpu["post_inspection"])}):
                        with self.assertRaisesRegex(RuntimeError, diagnostic):
                            sample["finalize_and_inspect"](stopped, None, ARGS.inspector, output, live, "cpu", True)
                self.assertFalse((output / "evidence.json").exists())

    def test_vector_comparison_rejects_different_lengths(self):
        sample = runpy.run_path(str(ARGS.sample))
        self.assertFalse(sample["close_vector"]([0.0, 1.0], [0.0, 1.0, 7.0]))

    def test_recording_active_after_stop_prevents_inspection(self):
        sample = runpy.run_path(str(ARGS.sample))
        output = Path(tempfile.mkdtemp(prefix="recording-active-", dir=WORK))
        stopped = SimpleNamespace(stop_recording=lambda: None, is_recording=lambda: True)
        with patch.object(subprocess, "run", side_effect=AssertionError("Inspector was invoked")):
            with self.assertRaisesRegex(RuntimeError, "Recording is still active after stop"):
                sample["finalize_and_inspect"](stopped, None, ARGS.inspector, output, [], "cpu", True)
        self.assertFalse((output / "evidence.json").exists())

    def test_vector_comparison_rejects_extra_components(self):
        cpu = self.verify_capture("cpu")
        vectors = (
            ("recorded pose", lambda evidence: evidence["objects"]["/World/Cube1"][0]["pose"]["position"]),
            ("recorded velocity", lambda evidence: evidence["objects"]["/World/Cube1"][0]["velocity"]["value"]),
            ("live position", lambda evidence: evidence["live"][0]["bodies"]["/World/Cube1"]["position"]),
            ("live velocity", lambda evidence: evidence["live"][0]["bodies"]["/World/Cube1"]["linearVelocity"]),
        )
        try:
            for label, select_vector in vectors:
                with self.subTest(label=label):
                    evidence = copy.deepcopy(cpu)
                    select_vector(evidence).append(123.0)
                    self.captures["cpu"] = evidence
                    with self.assertRaises(AssertionError):
                        self.verify_capture("cpu")
        finally:
            self.captures["cpu"] = cpu

    def test_attach_scene_registers_schemas_populates_with_rendering_and_seals_before_attach(self):
        sample = runpy.run_path(str(ARGS.sample))
        for newton_available in (True, False):
            with self.subTest(newton_available=newton_available):
                events = []
                def newton_root():
                    if not newton_available:
                        raise FileNotFoundError("newton-usd-schemas absent")
                    return "/schemas/newton"
                class Stage:
                    def __init__(self, name):
                        self.name = name
                    def advance_write_floor(self, ordinal):
                        events.append(("seal", ordinal))
                        return SimpleNamespace(wait=lambda: events.append(("sealed", ordinal)))
                    def destroy(self):
                        events.append("destroy")
                population = SimpleNamespace(
                    available=lambda: True,
                    register_usd_schemas=lambda roots: events.append(("register", roots)),
                    open_usd_with_desc=lambda *args, **kwargs: events.append(("populate", kwargs.get("descs"))),
                    DEFAULT_TIME=float("nan"),
                )
                fake_ovstage = SimpleNamespace(population=population, Stage=Stage)
                fake_ovphysx = SimpleNamespace(newton_schema_root=newton_root)
                schema_support = SimpleNamespace(physx_schema_paths=lambda: ["/schemas/physx", "/schemas/usdPhysics"])
                attach_task = object()
                def attach(*args, **kwargs):
                    events.append(("attach", kwargs.get("read_ordinal")))
                    return attach_task
                fake_physx = SimpleNamespace(attach_ovstage=attach,
                    wait_task=lambda task: events.append(("collected", task)))
                with patch.dict(sys.modules, {"ovphysx": fake_ovphysx, "ovstage": fake_ovstage,
                                              "ovphysx.population": FAKE_OVPHYSX_POPULATION,
                                              "ovphysx.samples.python_samples.schema_support": schema_support}):
                    stage = sample["attach_scene"](fake_physx, Path("scene.usda"))
                self.assertEqual(events[0], ("register", ["/schemas/physx", "/schemas/usdPhysics"] +
                                 (["/schemas/newton"] if newton_available else [])))
                self.assertEqual(events[1], ("populate", ["physics", "rendering"]))
                self.assertEqual(events[2], ("seal", 1))
                self.assertEqual(events[3:], [("sealed", 1), ("attach", 1), ("collected", attach_task)])
                self.assertIsInstance(stage, Stage)

    def test_missing_physx_provider_fails_before_scene_actions(self):
        sample = runpy.run_path(str(ARGS.sample))
        for newton_available in (True, False):
            with self.subTest(newton_available=newton_available):
                events = []
                error = FileNotFoundError("physx-usd-schemas absent")
                def physx_paths():
                    raise error
                def newton_root():
                    if not newton_available:
                        raise FileNotFoundError("newton-usd-schemas absent")
                    return "/schemas/newton"
                population = SimpleNamespace(available=lambda: True,
                    register_usd_schemas=lambda roots: events.append("register"),
                    open_usd_with_desc=lambda *args, **kwargs: events.append("populate"),
                    DEFAULT_TIME=float("nan"))
                stage = SimpleNamespace(advance_write_floor=lambda **kwargs: SimpleNamespace(wait=lambda: None),
                    destroy=lambda: events.append("destroy"))
                def create_stage(name):
                    events.append("stage")
                    return stage
                physx = SimpleNamespace(attach_ovstage=lambda *args, **kwargs: events.append("attach"),
                    wait_task=lambda task: None)
                modules = {"ovphysx": SimpleNamespace(newton_schema_root=newton_root),
                    "ovstage": SimpleNamespace(population=population, Stage=create_stage),
                    "ovphysx.population": FAKE_OVPHYSX_POPULATION,
                    "ovphysx.samples.python_samples.schema_support": SimpleNamespace(physx_schema_paths=physx_paths)}
                with patch.dict(sys.modules, modules):
                    with self.assertRaises(FileNotFoundError) as caught:
                        sample["attach_scene"](physx, Path("scene.usda"))
                self.assertIs(caught.exception, error)
                self.assertEqual(events, [])

    def test_failed_attach_collection_preserves_borrowed_stage_until_detached(self):
        sample = runpy.run_path(str(ARGS.sample))
        for attached, detach_fails in ((False, False), (True, False), (True, True)):
            with self.subTest(attached=attached, detach_fails=detach_fails):
                events = []
                stage = SimpleNamespace(advance_write_floor=lambda **kwargs: SimpleNamespace(wait=lambda: None),
                                        destroy=lambda: events.append("destroy"))
                def wait(task):
                    events.append(task)
                    if task == "attach" or detach_fails:
                        raise RuntimeError(f"{task} failed")
                physx = SimpleNamespace(attach_ovstage=lambda *args, **kwargs: "attach", wait_task=wait,
                    is_stage_attached=lambda item: attached, detach_ovstage=lambda: "detach")
                population = SimpleNamespace(available=lambda: True, register_usd_schemas=lambda roots: None,
                                             open_usd_with_desc=lambda *args, **kwargs: None,
                                             DEFAULT_TIME=float("nan"))
                modules = {"ovphysx": SimpleNamespace(newton_schema_root=lambda: "/schemas/newton"),
                    "ovstage": SimpleNamespace(population=population, Stage=lambda name: stage),
                    "ovphysx.population": FAKE_OVPHYSX_POPULATION,
                    "ovphysx.samples.python_samples.schema_support": SimpleNamespace(physx_schema_paths=lambda: ["/schemas/physx", "/schemas/usdPhysics"])}
                with patch.dict(sys.modules, modules), self.assertRaisesRegex(RuntimeError, "failed"):
                    sample["attach_scene"](physx, Path("scene.usda"))
                self.assertEqual(events, ["attach"] + (["detach"] if attached else []) +
                                 ([] if detach_fails else ["destroy"]))

    def test_run_configures_capability_and_collects_detach(self):
        sample = runpy.run_path(str(ARGS.sample))
        for device, detach_fails in (("cpu", False), ("gpu", False), ("cpu", True)):
            with self.subTest(device=device, detach_fails=detach_fails):
                events = []
                capability = object()
                def set_global_config(key, value):
                    self.assertIs(key, capability)
                    self.assertIs(value, True)
                    events.append("capable")
                def config(*, carbonite_overrides):
                    self.assertEqual(carbonite_overrides, {"/physics/suppressReadback": device == "gpu"})
                    return carbonite_overrides
                class CurrentPhysX:
                    @staticmethod
                    def set_cpu_mode(value):
                        self.assertIs(value, True)
                    @staticmethod
                    def get_cpu_mode():
                        return device == "cpu"
                    def __init__(instance, config):
                        self.assertEqual(events, ["capable"])
                        events.append("create")
                    def wait_all(self):
                        pass
                    def start_recording(self, destination):
                        events.append("record")
                    def step_sync(self, duration):
                        pass
                    def detach_ovstage(self):
                        events.append("detach")
                        return "detach-task"
                    def wait_task(instance, task):
                        self.assertEqual(task, "detach-task")
                        events.append("collected")
                        if detach_fails:
                            raise RuntimeError("detach failed")
                    def destroy(self):
                        events.append("physx-destroy")
                stage = SimpleNamespace(destroy=lambda: events.append("stage-destroy"))
                fake = SimpleNamespace(__file__=str(ARGS.sample), PhysX=CurrentPhysX, PhysXConfig=config,
                    GlobalConfigKey=SimpleNamespace(OVD_RECORDING_CAPABLE=capability),
                    set_global_config=set_global_config, OmniPvdDestination=SimpleNamespace(file=lambda path: path))
                overrides = {"inspect_json": lambda *args: {}, "attach_scene": lambda *args: stage,
                    "read_live": lambda *args: {"read_devices": ["cuda:0" if device == "gpu" else "cpu"]},
                    "finalize_and_inspect": lambda *args: events.append("inspect")}
                with tempfile.TemporaryDirectory(dir=WORK) as directory:
                    schemas = SimpleNamespace(physx_schema_paths=lambda: ["/schemas/physx", "/schemas/usdPhysics"])
                    with patch.dict(sys.modules, {"ovphysx": fake, "ovphysx.samples.python_samples.schema_support": schemas}), \
                            patch.dict(sample["run"].__globals__, overrides):
                        if detach_fails:
                            with self.assertRaisesRegex(RuntimeError, "detach failed"):
                                sample["run"](ARGS.sample, Path(directory) / "capture", device)
                        else:
                            sample["run"](ARGS.sample, Path(directory) / "capture", device)
                expected = ["capable", "create", "record", "inspect", "detach", "collected"]
                self.assertEqual(events, expected + ([] if detach_fails else ["stage-destroy", "physx-destroy"]))


def load_tests(loader, tests, pattern):
    helper_path = Path(__file__).with_name("test_inspection_helper.py")
    spec = importlib.util.spec_from_file_location("ovd_inspection_helper_tests", helper_path)
    helper_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper_module)
    tests.addTests(loader.loadTestsFromModule(helper_module))
    return tests


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--inspector", type=Path, required=True)
    parser.add_argument("--sample", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--require-gpu", action="store_true", help="Fail if the GPU workflow cannot run")
    mode.add_argument("--cpu-only", action="store_true", help="Run focused CPU checks; skip GPU and comparison tests")
    ARGS = parser.parse_args()
    ARGS.inspector = ARGS.inspector.resolve(strict=True)
    ARGS.sample = ARGS.sample.resolve()
    ARGS.output_root.mkdir(parents=True, exist_ok=True)
    WORK = Path(tempfile.mkdtemp(prefix="ovd workflow ", dir=ARGS.output_root.resolve()))
    print(f"Preserved workflow artifacts: {WORK}", flush=True)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
