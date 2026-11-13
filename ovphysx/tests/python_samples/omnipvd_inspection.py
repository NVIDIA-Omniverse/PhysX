# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OVD-INVESTIGATION-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
# @implements REQ-CAPI-OVSTAGE-XFORM-001
# @covers AC-1

"""Record ten steps, stop, and inspect using an explicitly installed SDK CLI.

Run --help for arguments. Output is preserved in a new directory. Importing this
module starts no runtime. The inspector is a separate PhysX SDK installation.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess


NAMES = ("/World/Cube1", "/World/Cube2", "/World/KinematicCube")


def _reject_non_json_constant(value):
    raise ValueError(f"Invalid JSON constant: {value}")


def _version_is(value, expected):
    return type(value) is int and value == expected


def inspect_json(inspector, *arguments):
    result = subprocess.run([str(inspector), *map(str, arguments)],
                            capture_output=True, timeout=30)
    stderr = result.stderr.decode("utf-8", errors="replace")
    if result.returncode not in (0, 1):
        diagnostic = stderr or result.stdout.decode("utf-8", errors="replace")
        raise RuntimeError(f"Inspector failed ({result.returncode}): {diagnostic}")
    stdout = result.stdout.decode("utf-8")
    document = json.loads(stdout, parse_constant=_reject_non_json_constant)
    if not isinstance(document, dict):
        raise RuntimeError("Unsupported inspector evidence object")
    command = arguments[0] if arguments else None
    if command == "--version":
        supported = document.get("tool") == "ovd-inspect" and _version_is(document.get("evidenceVersion"), 1)
    elif command in ("summary", "inspect"):
        supported = (document.get("format") == f"physx.ovd.{command}" and
                     _version_is(document.get("version"), 1))
    elif command == "compare":
        flags = {str(argument) for argument in arguments[1:]}
        has_attribute = "--attribute" in flags
        has_distance = "--distance" in flags
        supported = (document.get("format") == "physx.ovd.inspection-evidence" and
                     has_attribute == has_distance and
                     _version_is(document.get("version"), 2 if has_attribute else 1))
    else:
        supported = False
    if not supported:
        raise RuntimeError("Unsupported inspector evidence format or version")
    if result.returncode == 1 and not (command == "compare" and document.get("status") == "Different" and
                                       document.get("hasEvidence") is True):
        raise RuntimeError(f"Inspector failed (1): {stderr or stdout}")
    if command == "compare" and result.returncode == 0 and document.get("status") != "Equal":
        raise RuntimeError("Inspector comparison status disagrees with exit code 0")
    return document


def attach_scene(physx, scene):
    import ovphysx
    import ovstage
    from ovphysx.population import physics_population_descs
    from ovphysx.samples.python_samples.schema_support import physx_schema_paths

    if not ovstage.population.available():
        raise RuntimeError("ovstage population bridge is unavailable")
    schema_roots = [str(path) for path in physx_schema_paths()]
    try:
        schema_roots.append(str(ovphysx.newton_schema_root()))
    except FileNotFoundError:
        pass  # The bundled scene has no newton:* attributes.
    ovstage.population.register_usd_schemas(schema_roots)
    stage = ovstage.Stage("ovphysx-ovd-investigation")
    try:
        ovstage.population.open_usd_with_desc(stage, str(scene), ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True))
        stage.advance_write_floor(ordinal=1).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        return stage
    except Exception:
        if physx.is_stage_attached(stage):
            physx.wait_task(physx.detach_ovstage())
        stage.destroy()
        raise


def read_live(physx, stage):
    import ovstage
    from ovphysx.types import ObjectScope, SimObjectType

    bodies = {name: {} for name in NAMES}
    devices = set()
    with ovstage.PathDictionary(stage) as paths:
        for attribute in ("position", "linearVelocity"):
            with physx.read_sync(SimObjectType.RIGID_BODY, [attribute], scope=ObjectScope.ALL) as result:
                for group in result.groups:
                    if group.is_delete or group.is_array or not group.tensors:
                        continue
                    if group.index_map is not None or group.prim_index_map is not None:
                        raise RuntimeError("This fixed scene requires dense live read groups")
                    values = group.tensors[0].numpy()
                    devices.add(str(group.tensors[0].device))
                    for row, name in enumerate(paths.get_path_strings(group.prim_list)):
                        if name in bodies:
                            if attribute in bodies[name]:
                                raise RuntimeError(f"Ambiguous live body: {name}")
                            bodies[name][attribute] = values[row].tolist()
    for name, body in bodies.items():
        if set(body) != {"position", "linearVelocity"}:
            raise RuntimeError(f"Missing live body state: {name}")
        if not all(math.isfinite(value) for vector in body.values() for value in vector):
            raise RuntimeError(f"Non-finite live state: {name}")
    return {"bodies": bodies, "read_devices": sorted(devices)}


def close_vector(left, right):
    return len(left) == len(right) == 3 and all(
        math.isfinite(a) and math.isfinite(b) and abs(a - b) <= 1.0e-5
        for a, b in zip(left, right)
    )


def finalize_and_inspect(physx, physx_stage, inspector, output, live, device, cpu_mode):
    # A failed stop must never proceed to inspection, even if a partial file exists.
    physx.stop_recording()
    if physx.is_recording():
        raise RuntimeError("Recording is still active after stop")
    capture = output / "capture.ovd"
    before = hashlib.sha256(capture.read_bytes()).hexdigest()
    summary = inspect_json(inspector, "summary", capture, "--limit", 100)
    contexts = [context for context in summary["contexts"] if context["sceneMapped"]]
    if len(contexts) != 1:
        raise RuntimeError("Expected exactly one recorded scene context")
    context = contexts[0]
    # This sample checks these START-interval selections against its live post-step
    # reads below. The mapping is not assumed for arbitrary producers/recordings.
    frames = (2, 20)
    completed = {int(frame["frame"]["recordedFrame"]) for frame in summary["frames"]
                 if frame["completed"] and frame["frame"]["context"] == context["context"]}
    if not set(frames).issubset(completed):
        raise RuntimeError("Required completed frames 2 and 20 are absent")
    objects = {}
    for name in NAMES:
        objects[name] = []
        for frame, observed in zip(frames, (live[0], live[-1])):
            selection = ["--segment", context["recordingSegment"], "--context", context["context"],
                         "--frame", frame, "--name", name, "--class", "PxRigidDynamic"]
            pose = inspect_json(inspector, "inspect", capture, *selection, "--attribute", "globalPose")
            velocity = inspect_json(inspector, "inspect", capture, *selection, "--attribute", "linearVelocity")
            expected = observed["bodies"][name]
            if not close_vector(pose["position"], expected["position"]) or not close_vector(
                velocity["value"], expected["linearVelocity"]
            ):
                raise RuntimeError(f"Recorded frame {frame} disagrees with live state for {name}")
            if name != "/World/KinematicCube":
                for item in (pose, velocity):
                    sample = item["sample"]
                    if sample["state"] != "Exact" or not sample["hasSourceFrame"]:
                        raise RuntimeError(f"Moving body lacks an exact frame write: {name}")
                    for key in ("requested", "source"):
                        metadata = sample[key]
                        if (int(metadata["recordedFrame"]) != frame or
                                metadata["context"] != context["context"] or
                                metadata["recordingSegment"] != context["recordingSegment"]):
                            raise RuntimeError(f"Unexpected {key} frame for {name}: {frame}")
            else:
                for item in (pose, velocity):
                    sample = item["sample"]
                    if (sample["state"] != "Carried" or not sample["preMembership"] or
                            sample["hasSourceFrame"] or sample["source"] is not None):
                        raise RuntimeError(f"Kinematic body lacks carried initial state: {name}")
                    requested = sample["requested"]
                    if (int(requested["recordedFrame"]) != frame or
                            requested["context"] != context["context"] or
                            requested["recordingSegment"] != context["recordingSegment"]):
                        raise RuntimeError(f"Unexpected requested frame for {name}: {frame}")
            objects[name].append({"pose": pose, "velocity": velocity})
    for name in NAMES[:2]:
        if live[-1]["bodies"][name]["position"][1] >= live[0]["bodies"][name]["position"][1] - 0.1:
            raise RuntimeError(f"Dynamic body did not fall: {name}")
    for sample in live:
        fixed = sample["bodies"]["/World/KinematicCube"]
        if not close_vector(fixed["position"], [0, 2, 0]) or not close_vector(fixed["linearVelocity"], [0, 0, 0]):
            raise RuntimeError("Live kinematic body moved")
    # A further step proves inspection completed before instance/stage teardown.
    physx.step_sync(1.0 / 60.0)
    post_inspection = read_live(physx, physx_stage)
    post_inspection["step"] = 11
    for name in NAMES[:2]:
        if post_inspection["bodies"][name]["position"][1] >= live[-1]["bodies"][name]["position"][1]:
            raise RuntimeError(f"Post-inspection body did not continue falling: {name}")
    if not post_inspection["read_devices"] or not all(
        item.startswith("cuda:") if device == "gpu" else item == "cpu"
        for item in post_inspection["read_devices"]
    ):
        raise RuntimeError("Post-inspection read device disagrees with requested mode")
    if hashlib.sha256(capture.read_bytes()).hexdigest() != before:
        raise RuntimeError("Inspector modified source capture")
    evidence = {
        "format": "ovphysx.ovd.investigation", "version": 1, "device": device,
        "cpu_mode": cpu_mode, "post_inspection": post_inspection,
        "inspection_phase": "stopped_before_teardown", "capture_sha256": before,
        "inspector": inspect_json(inspector, "--version"), "summary": summary,
        "live": live, "objects": objects,
        "kinematic_evidence": "Carried initial snapshot is corroborated by ten live reads; it is not a fresh write.",
    }
    with (output / "evidence.json").open("x", encoding="utf-8") as stream:
        json.dump(evidence, stream, indent=2, allow_nan=False)
    return evidence


def run(inspector, output, device):
    inspector = inspector.resolve(strict=True)
    if not inspector.is_file():
        raise ValueError("--inspector must name an installed executable")
    inspect_json(inspector, "--version")
    from ovphysx.samples.python_samples.schema_support import physx_schema_paths
    physx_schema_paths()
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)

    import ovphysx
    from ovphysx import GlobalConfigKey, OmniPvdDestination, PhysX, PhysXConfig, set_global_config

    scene = Path(ovphysx.__file__).resolve().parent / "samples" / "data" / "simple_physics_scene.usda"
    if not scene.is_file():
        scene = Path(__file__).resolve().parent.parent / "data" / "simple_physics_scene.usda"
    scene_text = scene.read_text(encoding="utf-8")
    marker = 'def PhysicsScene "PhysicsScene"\n    {'
    if scene_text.count(marker) != 1:
        raise RuntimeError("Expected one PhysicsScene in the bundled fixture")
    if device == "gpu":
        scene_text = scene_text.replace(marker,
                                        'def PhysicsScene "PhysicsScene" (\n'
                                        '        prepend apiSchemas = ["PhysxSceneAPI"]\n    )\n    {\n'
                                        '        bool physxScene:enableGPUDynamics = true\n'
                                        '        token physxScene:broadphaseType = "GPU"')
    scene = output / "scene.usda"
    scene.write_text(scene_text, encoding="utf-8")
    if device == "cpu":
        PhysX.set_cpu_mode(True)
    elif PhysX.get_cpu_mode():
        raise RuntimeError("GPU capture requested in a CPU-only process")
    cpu_mode = PhysX.get_cpu_mode()
    if cpu_mode != (device == "cpu"):
        raise RuntimeError("PhysX CPU mode disagrees with requested mode")
    set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)
    physx = PhysX(config=PhysXConfig(carbonite_overrides={"/physics/suppressReadback": device == "gpu"}))
    stage = None
    try:
        stage = attach_scene(physx, scene)
        physx.wait_all()
        physx.start_recording(OmniPvdDestination.file(str(output / "capture.ovd")))
        live = []
        for step in range(1, 11):
            physx.step_sync(1.0 / 60.0)
            sample = read_live(physx, stage)
            sample["step"] = step
            if not sample["read_devices"] or not all(
                item.startswith("cuda:") if device == "gpu" else item == "cpu"
                for item in sample["read_devices"]
            ):
                raise RuntimeError(f"{device} capture did not produce matching simulation-state reads")
            live.append(sample)
        finalize_and_inspect(physx, stage, inspector, output, live, device, cpu_mode)
        print(f"Verified {device} capture before teardown; preserved {output}")
    finally:
        if stage is not None:
            physx.wait_task(physx.detach_ovstage())
            stage.destroy()
        physx.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspector", type=Path, required=True, help="Explicit installed SDK ovd-inspect executable")
    parser.add_argument("--output-dir", type=Path, required=True, help="New persistent directory; existing paths are refused")
    parser.add_argument("--device", choices=("cpu", "gpu"), default="cpu")
    args = parser.parse_args()
    run(args.inspector, args.output_dir, args.device)


if __name__ == "__main__":
    main()
