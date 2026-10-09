# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-6
# @maps_to TEST-CAPI-OVSTAGE-SCHEMA-001

"""attach_ovstage warns when the Newton USD schema was not registered (REQ-CAPI-OVSTAGE-SCHEMA-001 AC-6).

The parser honours newton:velocityLimit as the fallback for physxJoint:maxJointVelocity,
but population delivers a newton:* attribute only when the Newton schema
(newton-usd-schemas) was registered with ovstage before the first population. ovphysx
does not ship that schema (NVBug 6742287: a joint authoring only newton:velocityLimit ran
unlimited), so attach_ovstage checks the registration of the installed package and warns.
The registry is process-global, so every case runs in a fresh interpreter.
"""

import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

_FLT_MAX = 3.4028234663852886e38

# A revolute joint authoring only the Newton spelling of its velocity limit, plus a
# Newton attribute that exists only in the test's modified copy of the schema.
_USDA = """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "World"
{
    def PhysicsScene "physicsScene"
    {
    }
    def Xform "Arm" (
        prepend apiSchemas = ["PhysicsArticulationRootAPI"]
    )
    {
        def Cube "base" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI"]
        )
        {
            float physics:mass = 10.0
            double size = 0.2
            double3 xformOp:translate = (0, 0, 3)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }
        def Cube "link" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI", "PhysicsMassAPI"]
        )
        {
            float physics:mass = 2.0
            double size = 0.4
            double3 xformOp:translate = (0.5, 0, 3)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }
        def PhysicsFixedJoint "rootJoint"
        {
            rel physics:body1 = </World/Arm/base>
            point3f physics:localPos0 = (0, 0, 3)
        }
        def PhysicsRevoluteJoint "hinge" (
            prepend apiSchemas = ["PhysxJointAPI", "NewtonJointAPI"]
        )
        {
            float newton:velocityLimit = 111.0
            float newton:ovphysxTestOnly = 7.0
            uniform token physics:axis = "Y"
            rel physics:body0 = </World/Arm/base>
            rel physics:body1 = </World/Arm/link>
            point3f physics:localPos0 = (0, 0, 0)
            point3f physics:localPos1 = (-0.5, 0, 0)
        }
    }
}
"""

_CHILD_SCRIPT = textwrap.dedent(
    """
    import json
    import logging
    import shutil
    import sys
    import tempfile
    import warnings

    import numpy as np
    import ovstage
    import ovphysx
    from ovphysx.types import ObjectScope, SimObjectType

    mode, usd_path = sys.argv[1], sys.argv[2]
    if not ovstage.population.available():
        print("SKIP|ovstage population bridge is unavailable", flush=True)
        sys.exit(0)

    SETTING = "/ovphysx/schemas/warnMissingNewtonSchema"
    caught = []
    logged = []

    class _Capture(logging.Handler):
        def emit(self, record):
            if record.levelno >= logging.WARNING and "attach_ovstage" in record.getMessage():
                logged.append(record.getMessage())

    logging.getLogger("ovphysx").addHandler(_Capture())

    def attach_recording(physx, stage):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            physx.attach_ovstage(stage, read_ordinal=1)
        caught.extend(str(x.message) for x in w if issubclass(x.category, RuntimeWarning) and "attach_ovstage" in str(x.message))

    def copy_newton_schema(extra_attribute=None):
        # A complete copy of the installed package in another directory, optionally
        # extended with one attribute the installed copy does not define.
        src = ovphysx.newton_schema_root()
        dst = tempfile.mkdtemp(prefix="ovphysx-newton-copy-") + "/newton_usd_schemas"
        shutil.copytree(src, dst)
        if extra_attribute:
            usda = dst + "/generatedSchema.usda"
            text = open(usda).read()
            anchor = 'class NewtonJointAPI "NewtonJointAPI" ('
            head, sep, tail = text.partition(anchor)
            assert sep, "NewtonJointAPI class not found in the Newton schema"
            body_start = tail.index("{") + 1
            tail = tail[:body_start] + "\\n    float " + extra_attribute + " = 0 (\\n        doc = \\"test only\\"\\n    )\\n" + tail[body_start:]
            open(usda, "w").write(head + sep + tail)
        return dst

    if mode == "not-installed":
        # Hide the installed package from the discovery helper only; nothing else changes.
        import importlib.util
        real_find_spec = importlib.util.find_spec
        importlib.util.find_spec = lambda name, *a, **k: None if name == "newton_usd_schemas" else real_find_spec(name, *a, **k)

    ovphysx.PhysX.set_cpu_mode(True)
    config = None
    if mode == "unregistered-optout":
        config = ovphysx.PhysXConfig(carbonite_overrides={SETTING: False})
    elif mode == "unregistered-optout-string":
        config = ovphysx.PhysXConfig(carbonite_overrides={SETTING: "false"})

    extra_present = None
    if mode == "procedural-then-usd":
        # A procedurally authored stage reaches attach before any USD population: the
        # check must not register the installed schema on the application's behalf.
        from ovphysx import population as pop
        ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
        proc_stage = ovstage.Stage("newton-schema-procedural")
        paths = ovstage.PathDictionary(proc_stage)
        batch = pop.PrimBatch(paths, ["/World/Scene"])
        batch.define_physics_scene()
        batch.write(proc_stage, 1)
        proc_stage.advance_write_floor(ordinal=1).wait()
        proc = ovphysx.PhysX()
        attach_recording(proc, proc_stage)
        proc.detach_ovstage()
        proc.destroy()
        paths.destroy() if hasattr(paths, "destroy") else None
        proc_stage.destroy()
        # The application then chooses its own Newton schema: a copy that also defines
        # newton:ovphysxTestOnly. Had the check registered the installed package first,
        # this family would already be taken and the extra attribute would never populate.
        ovstage.population.register_usd_schemas([copy_newton_schema("newton:ovphysxTestOnly")])
    else:
        roots = [str(ovphysx.codeless_schema_root())]
        if mode == "registered":
            roots.append(str(ovphysx.newton_schema_root()))
        elif mode == "registered-other-directory":
            roots.append(copy_newton_schema())
        ovstage.population.register_usd_schemas(roots)

    stage = ovstage.Stage("newton-schema-check")
    ovstage.population.open_usd(stage, usd_path, ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
    stage.advance_write_floor(ordinal=1).wait()

    if mode == "procedural-then-usd":
        with ovstage.PathDictionary(stage) as paths:
            token = paths.intern_token("newton:ovphysxTestOnly")
            with paths.create_path_list_from_strings(["/World/Arm/hinge"]) as path_list:
                with stage.query_from_path_list(path_list) as query:
                    values = []
                    with stage.read_attributes(query, [token], ovstage.OrdinalRange.latest(1)) as read:
                        read.wait()
                        for group in read.groups():
                            with group:
                                for i in range(group.tensor_count):
                                    values.extend(float(v) for v in np.ravel(np.array(group.array(i), copy=True)))
        extra_present = values

    physx = ovphysx.PhysX(config=config)
    try:
        if mode == "unregistered-error":
            # A warning promoted to an error must propagate before the native attach:
            # the instance stays detached, so a retry (the check has latched) attaches.
            with warnings.catch_warnings():
                warnings.simplefilter("error", RuntimeWarning)
                try:
                    physx.attach_ovstage(stage, read_ordinal=1)
                except RuntimeWarning as exc:
                    print("RAISED|" + str(exc).replace("\\n", " "), flush=True)
                else:
                    print("RAISED|<nothing>", flush=True)
            physx.attach_ovstage(stage, read_ordinal=1)
            print("RETRY_ATTACHED|ok", flush=True)
        else:
            attach_recording(physx, stage)
            # A second attach in the same process must not repeat the warning.
            physx.detach_ovstage()
            attach_recording(physx, stage)
        for message in caught:
            print("WARN|" + message.replace("\\n", " "), flush=True)
        for message in logged:
            print("LOG|" + message.replace("\\n", " "), flush=True)
        physx.step(1.0 / 60.0)
        physx.wait_all()
        with physx.read(SimObjectType.ARTICULATION_JOINT, ["jointMaxVelocity"], scope=ObjectScope.ALL) as result:
            values = [float(v) for group in result.groups for tensor in group.tensors for v in np.ravel(tensor.numpy())]
        physx.detach_ovstage()
        print("VALUES|" + json.dumps(values), flush=True)
        if extra_present is not None:
            print("EXTRA|" + json.dumps(extra_present), flush=True)
    finally:
        physx.destroy()
        stage.destroy()
    """
)


def _run_child(tmp_path: Path, mode: str):
    usd_path = tmp_path / "newton_velocity_limit.usda"
    usd_path.write_text(_USDA)
    child_env = os.environ.copy()
    child_env.pop("OV_PXR_PLUGINPATH_2511", None)
    child_env.pop("PXR_PLUGINPATH_NAME", None)
    result = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT, mode, str(usd_path)],
        check=False,
        capture_output=True,
        text=True,
        env=child_env,
        timeout=180,
    )
    assert result.returncode == 0, f"child failed\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    lines = result.stdout.splitlines()
    skip = [line for line in lines if line.startswith("SKIP|")]
    if skip:
        pytest.skip(skip[0][5:])
    out = {
        "warns": [line[len("WARN|"):] for line in lines if line.startswith("WARN|")],
        "logged": [line[len("LOG|"):] for line in lines if line.startswith("LOG|")],
        "raised": [line[len("RAISED|"):] for line in lines if line.startswith("RAISED|")],
        "retry": [line for line in lines if line.startswith("RETRY_ATTACHED|")],
        "extra": [json.loads(line[len("EXTRA|"):]) for line in lines if line.startswith("EXTRA|")],
    }
    values = [line for line in lines if line.startswith("VALUES|")]
    assert values, f"child produced no read-back:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    parsed = json.loads(values[-1][len("VALUES|"):])
    assert len(parsed) == 1, parsed
    out["value"] = parsed[0]
    return out


def test_registered_newton_schema_is_honored_without_warning(tmp_path):
    out = _run_child(tmp_path, "registered")
    assert not out["warns"], out["warns"]
    assert abs(out["value"] - 111.0) < 1e-3, out["value"]


def test_newton_schema_registered_from_another_directory_is_recognized(tmp_path):
    """ovstage keys registration on the plugin family: a complete copy registered from a
    different directory before population satisfies the check."""
    out = _run_child(tmp_path, "registered-other-directory")
    assert not out["warns"], out["warns"]
    assert abs(out["value"] - 111.0) < 1e-3, out["value"]


def test_unregistered_newton_schema_warns_once_and_names_the_fix(tmp_path):
    out = _run_child(tmp_path, "unregistered")
    assert len(out["warns"]) == 1, out["warns"]
    message = out["warns"][0]
    assert "was not registered with ovstage before the first population" in message
    assert "newton_schema_root" in message
    assert "register_usd_schemas" in message
    assert "warnMissingNewtonSchema" in message
    # The behaviour the warning describes: the authored Newton value never reached PhysX.
    assert out["value"] >= _FLT_MAX, out["value"]


def test_warning_promoted_to_error_leaves_the_instance_detached(tmp_path):
    out = _run_child(tmp_path, "unregistered-error")
    assert out["raised"] and "was not registered with ovstage" in out["raised"][0], out["raised"]
    assert out["retry"], "the retry after the raised warning did not attach"
    assert out["value"] >= _FLT_MAX, out["value"]


def test_missing_newton_package_logs_the_install_hint_without_a_python_warning(tmp_path):
    """Not installed is the default for consumers without newton:* content, so this case
    goes to the ovphysx logger and never trips a warnings-as-errors filter."""
    out = _run_child(tmp_path, "not-installed")
    assert not out["warns"], out["warns"]
    assert len(out["logged"]) == 1, out["logged"]
    message = out["logged"][0]
    assert "is not installed" in message
    assert "pip install newton-usd-schemas" in message
    assert "github.com/newton-physics/newton-usd-schemas" in message
    assert out["value"] >= _FLT_MAX, out["value"]


def test_registered_and_unregistered_cases_log_nothing(tmp_path):
    for mode in ("registered", "unregistered"):
        out = _run_child(tmp_path, mode)
        assert not out["logged"], (mode, out["logged"])


@pytest.mark.parametrize("mode", ["unregistered-optout", "unregistered-optout-string"])
def test_opt_out_setting_silences_the_check(tmp_path, mode):
    out = _run_child(tmp_path, mode)
    assert not out["warns"], out["warns"]
    assert out["value"] >= _FLT_MAX, out["value"]


def test_procedural_attach_does_not_register_a_schema_for_the_application(tmp_path):
    """A stage authored through ovphysx.population is attached before any USD population.
    The check must leave the still-open registry alone: the application's later
    registration of its own Newton copy is the one USD uses, so the attribute only that
    copy defines populates, the joint limit is honoured, and no warning is emitted."""
    out = _run_child(tmp_path, "procedural-then-usd")
    assert not out["warns"], out["warns"]
    assert abs(out["value"] - 111.0) < 1e-3, out["value"]
    assert out["extra"] and out["extra"][0] and abs(out["extra"][0][0] - 7.0) < 1e-6, out["extra"]
