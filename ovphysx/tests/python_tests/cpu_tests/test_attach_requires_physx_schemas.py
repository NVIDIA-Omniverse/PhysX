# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-8
# @maps_to TEST-CAPI-OVSTAGE-SCHEMA-001

"""attach_ovstage checks PhysX schemas after USD schema reads (REQ-CAPI-OVSTAGE-SCHEMA-001 AC-8).

ovstage assembles its USD schema registry on the first population or export, so whether the
codeless PhysX schemas were registered in time is process-global state. Each case
therefore runs in a fresh interpreter: one populates without registering (NVBug
6745103, where the dropped PhysxArticulationAPI / PhysxJointAPI let the scene
diverge inside PhysX), the other registers first as the positive control.
"""

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

_CHILD_SCRIPT = textwrap.dedent(
    """
    import shutil
    import sys
    from pathlib import Path

    import ovstage
    import ovphysx
    from schema_support import physx_schema_paths

    mode, usd_path, scratch = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    if not ovstage.population.available():
        print("SKIP|ovstage population bridge is unavailable", flush=True)
        sys.exit(0)
    ovphysx.PhysX.set_cpu_mode(True)
    overrides = {"/ovphysx/schemas/warnMissingNewtonSchema": False}
    if mode == "unregistered-optout":
        overrides["/ovphysx/schemas/requireRegistration"] = False
    config = ovphysx.PhysXConfig(carbonite_overrides=overrides)
    physx = ovphysx.PhysX(config=config)
    if mode in ("procedural-then-usd", "procedural-then-unregistered-usd"):
        # A procedural attach must neither freeze USD nor claim the PhysX families
        # before the application selects its own external schema provider.
        from ovphysx import population as pop
        with ovstage.Stage("procedural-schema-gate") as procedural:
            with ovstage.PathDictionary(procedural) as paths:
                batch = pop.PrimBatch(paths, ["/World/Scene"])
                batch.define_physics_scene()
                batch.write(procedural, 1)
            procedural.advance_write_floor(ordinal=1).wait()
            physx.wait_task(physx.attach_ovstage(procedural, read_ordinal=1))
            physx.wait_task(physx.detach_ovstage())
            print("PROCEDURAL_ATTACH_OK", flush=True)

    roots = physx_schema_paths()
    if mode in ("registered-other-directory", "procedural-then-usd"):
        copied = []
        for root in roots:
            destination = scratch / root.parent.name
            shutil.copytree(root.parent, destination)
            copied.append(destination / "resources")
        roots = copied
        # Extend the application-owned provider. Reading its authored value back
        # proves an earlier procedural attach did not register a competing copy.
        schema = roots[0] / "generatedSchema.usda"
        content = schema.read_text(encoding="utf-8")
        original = "float physxRigidBody:maxLinearVelocity = inf"
        assert original in content
        schema.write_text(content.replace(original, "float physxRigidBody:ovphysxGuardTest = 0\\n    " + original), encoding="utf-8")
        scene = scratch / "custom-provider.usda"
        content = Path(usd_path).read_text(encoding="utf-8")
        content = content.replace('"PhysicsRigidBodyAPI",', '"PhysicsRigidBodyAPI", "PhysxRigidBodyAPI",')
        content = content.replace("float physics:mass = 10.0", "float physics:mass = 10.0\\n                float physxRigidBody:ovphysxGuardTest = 123")
        scene.write_text(content, encoding="utf-8")
        usd_path = str(scene)

    if mode in ("registered", "registered-other-directory", "procedural-then-usd"):
        ovstage.population.register_usd_schemas([str(path) for path in roots])
    elif mode == "physx-only":
        ovstage.population.register_usd_schemas([str(roots[0])])
    elif mode == "deformable-only":
        ovstage.population.register_usd_schemas([str(roots[1])])

    stage = ovstage.Stage("schema-gate")
    if mode in ("registered-export", "unregistered-export"):
        # Export reads USD schema definitions even when this process never populates
        # from USD. An earlier procedural attach must leave the guard armed.
        from ovphysx import population as pop
        with ovstage.PathDictionary(stage) as paths:
            batch = pop.PrimBatch(paths, ["/World/Scene"])
            batch.define_physics_scene()
            batch.write(stage, 1)
        stage.advance_write_floor(ordinal=1).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        physx.wait_task(physx.detach_ovstage())
        print("PROCEDURAL_ATTACH_OK", flush=True)
        if mode == "registered-export":
            ovstage.population.register_usd_schemas([str(path) for path in roots])
        exported = scratch / "procedural-export.usda"
        ovstage.population.export_typed_hierarchy_to_usd_file(
            stage, str(exported), "/World/Scene", 1
        )
        assert "PhysicsScene" in exported.read_text(encoding="utf-8")
        print("EXPORT_OK", flush=True)
    else:
        ovstage.population.open_usd(stage, usd_path, ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
        stage.advance_write_floor(ordinal=1).wait()
    if mode == "late-application-registration":
        # OVStage inserts plugin identities even when it rejects late registration.
        # A guard first called afterwards cannot distinguish these from timely
        # registration. Characterize this dependency limitation: callers must
        # restart on the original ordering error, regardless of a later attach.
        try:
            ovstage.population.register_usd_schemas([str(path) for path in roots])
        except ovstage.OvstageError as exc:
            assert "already built" in str(exc), str(exc)
            print("LATE_REGISTRATION_ERROR", flush=True)
        else:
            raise AssertionError("late registration unexpectedly succeeded")
    if mode in ("registered-other-directory", "procedural-then-usd"):
        import numpy as np
        with ovstage.PathDictionary(stage) as paths:
            token = paths.intern_token("physxRigidBody:ovphysxGuardTest")
            with paths.create_path_list_from_strings(["/World/envs/env0/table"]) as path_list:
                with stage.query_from_path_list(path_list) as query:
                    values = []
                    with stage.read_attributes(query, [token], ovstage.OrdinalRange.latest(1)) as read:
                        read.wait()
                        for group in read.groups():
                            with group:
                                for i in range(group.tensor_count):
                                    values.extend(float(value) for value in np.ravel(np.array(group.array(i), copy=True)))
        assert values == [123.0], values
        print("CUSTOM_PROVIDER_OK", flush=True)
    try:
        try:
            physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        except RuntimeError as exc:
            assert not physx.is_stage_attached()
            print("ATTACH_ERROR|" + str(exc).replace("\\n", " "), flush=True)
            # A refused attach leaves the instance detached: the same call fails the same way.
            try:
                physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
            except RuntimeError as again:
                assert not physx.is_stage_attached()
                print("ATTACH_ERROR_AGAIN|" + str(again).replace("\\n", " "), flush=True)
        else:
            physx.step(1.0 / 60.0)
            physx.wait_all()
            physx.wait_task(physx.detach_ovstage())
            print("ATTACH_OK", flush=True)
    finally:
        physx.destroy()
        stage.destroy()
    """
)


def _run_child(tmp_path: Path, mode: str, *, temp_root: str | None = None) -> str:
    usd_path = Path(__file__).resolve().parents[2] / "data" / "basic_simulation.usda"
    # A plugin-path variable in the parent would let the "unregistered" child resolve the
    # schemas anyway (ovstage keys the registration on the plugin family, not the path).
    child_env = os.environ.copy()
    child_env.pop("OV_PXR_PLUGINPATH_2511", None)
    child_env.pop("PXR_PLUGINPATH_NAME", None)
    if mode == "registered-environment":
        from schema_support import physx_schema_paths

        child_env["OV_PXR_PLUGINPATH_2511"] = os.pathsep.join(str(path) for path in physx_schema_paths())
    if temp_root is not None:
        child_env["TMPDIR"] = temp_root
    result = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT, mode, str(usd_path), str(tmp_path)],
        check=False,
        capture_output=True,
        text=True,
        env=child_env,
        cwd=tmp_path if temp_root is not None else None,
        timeout=180,
    )
    assert result.returncode == 0, f"child failed\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    skip = [line for line in result.stdout.splitlines() if line.startswith("SKIP|")]
    if skip:
        pytest.skip(skip[0][5:])
    return result.stdout + result.stderr


@pytest.mark.skipif(sys.platform == "win32", reason="TMPDIR is not the temp-root selector on Windows")
@pytest.mark.parametrize("mode", ["registered", "unregistered"])
@pytest.mark.parametrize("temp_kind", ["relative", "missing", "file"])
def test_invalid_temp_root_preserves_schema_verification(tmp_path: Path, mode: str, temp_kind: str):
    configured = tmp_path / "configured-temp"
    if temp_kind == "relative":
        configured.mkdir()
        temp_root = configured.name
    elif temp_kind == "file":
        configured.write_text("keep this file", encoding="utf-8")
        temp_root = str(configured)
    else:
        temp_root = str(configured)

    out = _run_child(tmp_path, mode, temp_root=temp_root)
    if mode == "registered":
        assert "ATTACH_OK" in out, out
        assert "ATTACH_ERROR" not in out, out
    else:
        errors = [line for line in out.splitlines() if line.startswith("ATTACH_ERROR")]
        assert len(errors) == 2 and all("register_usd_schemas" in line for line in errors), out
        assert "ATTACH_OK" not in out, out
    if temp_kind == "relative":
        assert not list(configured.glob("ovphysx-schema-probe-*"))
    elif temp_kind == "file":
        assert configured.read_text(encoding="utf-8") == "keep this file"
    else:
        assert not configured.exists()


@pytest.mark.parametrize("mode", ["unregistered", "physx-only", "deformable-only", "procedural-then-unregistered-usd"])
def test_attach_fails_when_schemas_were_not_registered_before_population(tmp_path: Path, mode: str):
    out = _run_child(tmp_path, mode)
    lines = [line for line in out.splitlines() if line.startswith("ATTACH_")]
    assert lines and lines[0].startswith("ATTACH_ERROR|"), f"attach did not fail:\n{out}"
    message = lines[0]
    assert "register_usd_schemas" in message
    assert "physx-usd-schemas" in message
    assert "Physx" in message
    assert any(line.startswith("ATTACH_ERROR_AGAIN|") and "register_usd_schemas" in line for line in lines), out
    assert "ATTACH_OK" not in lines
    if mode == "procedural-then-unregistered-usd":
        assert "PROCEDURAL_ATTACH_OK" in out, out


@pytest.mark.parametrize("mode", ["registered", "registered-environment", "registered-other-directory", "procedural-then-usd"])
def test_attach_succeeds_when_schemas_were_registered_first(tmp_path: Path, mode: str):
    out = _run_child(tmp_path, mode)
    assert "ATTACH_OK" in out, out
    assert "ATTACH_ERROR" not in out, out
    if mode in ("registered-other-directory", "procedural-then-usd"):
        assert "CUSTOM_PROVIDER_OK" in out, out
    if mode == "procedural-then-usd":
        assert "PROCEDURAL_ATTACH_OK" in out, out


@pytest.mark.parametrize("mode", ["registered-export", "unregistered-export"])
def test_export_arms_schema_verification_after_procedural_attach(tmp_path: Path, mode: str):
    out = _run_child(tmp_path, mode)
    assert "PROCEDURAL_ATTACH_OK" in out, out
    assert "EXPORT_OK" in out, out
    lines = [line for line in out.splitlines() if line.startswith("ATTACH_")]
    if mode == "registered-export":
        assert lines == ["ATTACH_OK"], out
    else:
        assert len(lines) == 2, out
        assert lines[0].startswith("ATTACH_ERROR|"), out
        assert lines[1].startswith("ATTACH_ERROR_AGAIN|"), out
        assert all("register_usd_schemas" in line and "physx-usd-schemas" in line for line in lines), out


def test_attach_opt_out_setting_downgrades_the_refusal_to_a_warning(tmp_path: Path):
    out = _run_child(tmp_path, "unregistered-optout")
    assert "ATTACH_OK" in out, out
    assert "ATTACH_ERROR" not in out, out
    assert "requireRegistration is false" in out, out


def test_prior_late_application_registration_is_outside_the_guards_detection(tmp_path: Path):
    out = _run_child(tmp_path, "late-application-registration")
    assert "LATE_REGISTRATION_ERROR" in out, out
    assert "ATTACH_OK" in out, out
    assert "ATTACH_ERROR" not in out, out
