# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run separately from runtime tests: only customer pxr, never ovstage/native USD."""

from pathlib import Path
import sys

import pytest

COMPANION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMPANION / "python"))
sys.path.insert(0, str(COMPANION / "samples"))


@pytest.fixture(scope="session", autouse=True)
def customer_schemas():
    # Use existing codeless helper; fail instead of silently skipping the suite.
    from ovphysx.utils import codeless
    codeless.register_schemas(verify=True)


@pytest.fixture
def scene():
    from pxr import Usd, UsdGeom
    from ovphysx.utils import codeless
    from ovphysx_companion.usd import EditSession

    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/World", "Xform")
    for path in ("/World/A", "/World/B"):
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
        mesh.CreateFaceVertexCountsAttr([3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        codeless.apply_api(mesh.GetPrim(), "OmniPhysicsDeformableBodyAPI")
        codeless.apply_api(mesh.GetPrim(), "OmniPhysicsSurfaceDeformableSimAPI")
    with EditSession(stage) as session:
        yield stage, session
