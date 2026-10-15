# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Invalid unit metadata and cone parameters must not produce unusable geometry."""

# @implements REQ-PYTHON-UTILS-001
# @covers AC-4 AC-6

import math

import pytest

pytest.importorskip("pxr", reason="ovphysx.utils requires a caller-supplied USD runtime")

from pxr import Gf, UsdGeom  # noqa: E402

from ovphysx import utils as physicsUtils  # noqa: E402


@pytest.mark.parametrize("meters_per_unit", [0.0, -0.0, -1.0, math.nan, math.inf, -math.inf])
def test_unit_scale_invalid_metadata(stage, caplog, meters_per_unit):
    stage.SetMetadata("metersPerUnit", meters_per_unit)
    before = stage.GetRootLayer().ExportToString()
    assert physicsUtils.get_unit_scale_factor(stage) == 1.0
    assert "Invalid metersPerUnit" in caplog.text
    assert stage.GetRootLayer().ExportToString() == before


@pytest.mark.parametrize("meters_per_unit", [None, 0.01, 0.5, 1.0, 2.0])
def test_unit_scale_valid_metadata(stage, caplog, meters_per_unit):
    if meters_per_unit is not None:
        UsdGeom.SetStageMetersPerUnit(stage, meters_per_unit)
    expected = 1.0 / UsdGeom.GetStageMetersPerUnit(stage)
    assert physicsUtils.get_unit_scale_factor(stage) == pytest.approx(expected)
    assert not caplog.records


@pytest.mark.parametrize(
    "parameter,value",
    [("height", value) for value in (0, 0.0, -1.0, math.nan, math.inf, -math.inf)]
    + [("radius", value) for value in (-1.0, math.nan, math.inf, -math.inf)]
    + [("tesselation", value) for value in (0, -1, 1, 2)],
)
@pytest.mark.parametrize("existing", [False, True])
def test_cone_invalid_parameters_leave_stage_unchanged(stage, parameter, value, existing):
    if existing:
        mesh = UsdGeom.Mesh.Define(stage, "/cone")
        mesh.CreatePointsAttr([Gf.Vec3f(1, 2, 3)])
    before = stage.GetRootLayer().ExportToString()
    args = {"height": 2.0, "radius": 0.5, "tesselation": 8}
    args[parameter] = value
    with pytest.raises(ValueError, match=parameter):
        physicsUtils.create_mesh_cone(stage, "/cone", **args)
    assert stage.GetRootLayer().ExportToString() == before


@pytest.mark.parametrize("tesselation", [3, 8, 32])
@pytest.mark.parametrize("radius", [0.0, 0.5])
def test_cone_valid_parameters(stage, tesselation, radius):
    mesh = physicsUtils.create_mesh_cone(stage, "/cone", 2.0, radius, tesselation)
    points = mesh.GetPointsAttr().Get()
    normals = mesh.GetNormalsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    assert len(points) == len(normals) == 4 * tesselation + 1
    assert list(counts) == [3] * (4 * tesselation)
    assert len(indices) == 12 * tesselation
    assert all(0 <= index < len(points) for index in indices)
    assert all(math.isfinite(component) for vector in points for component in vector)
    assert all(normal.GetLength() == pytest.approx(1.0) for normal in normals)
    assert points[0] == Gf.Vec3f(0, 0, 1)
    assert points[2] == Gf.Vec3f(radius, 0, -1)
    assert points[-1] == Gf.Vec3f(0, 0, -1)
