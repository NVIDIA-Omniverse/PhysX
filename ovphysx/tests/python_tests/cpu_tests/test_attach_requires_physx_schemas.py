# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-5
# @maps_to TEST-CAPI-OVSTAGE-SCHEMA-001

"""attach_ovstage refuses a stage populated without the PhysX schemas (REQ-CAPI-OVSTAGE-SCHEMA-001 AC-5).

ovstage assembles its USD schema registry on the first population, so whether the
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
    import sys

    import ovstage
    import ovphysx

    mode, usd_path = sys.argv[1], sys.argv[2]
    if not ovstage.population.available():
        print("SKIP|ovstage population bridge is unavailable", flush=True)
        sys.exit(0)
    if mode == "registered":
        ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])

    stage = ovstage.Stage("schema-gate")
    ovstage.population.open_usd(stage, usd_path, ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
    stage.advance_write_floor(ordinal=1).wait()

    ovphysx.PhysX.set_cpu_mode(True)
    config = None
    if mode == "unregistered-optout":
        config = ovphysx.PhysXConfig(carbonite_overrides={"/ovphysx/schemas/requireRegistration": False})
    physx = ovphysx.PhysX(config=config)
    try:
        try:
            physx.attach_ovstage(stage, read_ordinal=1)
        except RuntimeError as exc:
            print("ATTACH_ERROR|" + str(exc).replace("\\n", " "), flush=True)
            # A refused attach leaves the instance detached: the same call fails the same way.
            try:
                physx.attach_ovstage(stage, read_ordinal=1)
            except RuntimeError as again:
                print("ATTACH_ERROR_AGAIN|" + str(again).replace("\\n", " "), flush=True)
        else:
            physx.step(1.0 / 60.0)
            physx.wait_all()
            physx.detach_ovstage()
            print("ATTACH_OK", flush=True)
    finally:
        physx.destroy()
        stage.destroy()
    """
)


def _run_child(mode: str) -> str:
    usd_path = Path(__file__).resolve().parents[2] / "data" / "basic_simulation.usda"
    # A plugin-path variable in the parent would let the "unregistered" child resolve the
    # schemas anyway (ovstage keys the registration on the plugin family, not the path).
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
    skip = [line for line in result.stdout.splitlines() if line.startswith("SKIP|")]
    if skip:
        pytest.skip(skip[0][5:])
    return result.stdout


def test_attach_fails_when_schemas_were_not_registered_before_population():
    out = _run_child("unregistered")
    lines = [line for line in out.splitlines() if line.startswith("ATTACH_")]
    assert lines and lines[0].startswith("ATTACH_ERROR|"), f"attach did not fail:\n{out}"
    message = lines[0]
    assert "register_usd_schemas" in message
    assert "codeless_schema_root" in message
    assert "Physx" in message
    assert any(line.startswith("ATTACH_ERROR_AGAIN|") and "register_usd_schemas" in line for line in lines), out
    assert "ATTACH_OK" not in out


def test_attach_succeeds_when_schemas_were_registered_first():
    out = _run_child("registered")
    assert "ATTACH_OK" in out, out
    assert "ATTACH_ERROR" not in out, out


def test_attach_opt_out_setting_downgrades_the_refusal_to_a_warning():
    out = _run_child("unregistered-optout")
    assert "ATTACH_OK" in out, out
    assert "ATTACH_ERROR" not in out, out
    assert "requireRegistration is false" in out, out
