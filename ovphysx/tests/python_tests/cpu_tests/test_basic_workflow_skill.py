# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Execute the shipped additive-population recipe against a moving scene."""

import re
from pathlib import Path

import numpy as np
from ovphysx.types import SimObjectType
from session_helpers import read_column
from test_utils import attach_usd_with_ovstage, data_path


def test_add_usd_file_preserves_existing_bodies(physx_sdk, tmp_path):
    skill = Path(__file__).resolve().parents[3] / "skills/basic-workflow/SKILL.md"
    blocks = re.findall(r"^```python\n(.*?)^```", skill.read_text(encoding="utf-8"), re.DOTALL | re.MULTILINE)
    helper = next(block for block in blocks if "def add_usd_file_and_drain" in block)
    namespace = {}
    exec(compile(helper, str(skill), "exec"), namespace)

    extra = tmp_path / "extra.usda"
    extra.write_text('''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Y"
)
def Xform "World"
{
    def Sphere "ExtraBall" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI", "PhysicsMassAPI"])
    {
        double radius = 0.25
        double3 xformOp:translate = (6, 12, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
''', encoding="utf-8")
    stage = attach_usd_with_ovstage(physx_sdk, data_path("simple_physics_scene.usda"))
    physx_sdk.step_sync(1.0 / 60.0)
    before = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    assert len(before) == 4
    assert before["/World/Cube1"][1] < 5.0

    namespace["add_usd_file_and_drain"](stage, physx_sdk, extra, "/World/Spawned", 2)
    after = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    for path, position in before.items():
        np.testing.assert_allclose(after[path], position, rtol=0, atol=1e-6, err_msg=path)
    assert after.keys() == before.keys() | {"/World/Spawned/ExtraBall"}
    np.testing.assert_allclose(after["/World/Spawned/ExtraBall"], [6, 12, 0], rtol=0, atol=1e-6)

    physx_sdk.wait_task(physx_sdk.detach_ovstage())
    physx_sdk.wait_task(physx_sdk.attach_ovstage(stage, read_ordinal=2))
    fresh = read_column(physx_sdk, stage, SimObjectType.RIGID_BODY, "position")
    assert fresh.keys() == after.keys()
