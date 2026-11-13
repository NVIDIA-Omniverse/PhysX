# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""The physics population description ovphysx publishes (ADR-0061), in its C and Python forms.

Both forms are generated from the population contract, and tests/data/physics_population_desc.json is
the generator's normalized dump of the same description: each form is walked into that dump's form
(population_desc_dump.py) and compared. A small stage is then populated through the description the
two ways the documentation names: alone, and beside a RENDERING description. Either way the seal
composes the world matrix of a body under a translated parent.
"""

# @implements REQ-CAPI-POPULATION-DESC-001
# @covers AC-1 AC-2 AC-3
# @maps_to TEST-CAPI-POPULATION-DESC-001

import subprocess
import sys

import numpy as np
import pytest
import ovphysx.population as pop
from population_desc_dump import dump_c_desc, dump_python_desc
from test_utils import data_path

_USDA = """#usda 1.0
(
    metersPerUnit = 0.01
    upAxis = "Y"
)
def Xform "World"
{
    double3 xformOp:translate = (10, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]

    def PhysicsScene "Scene"
    {
    }
    def Cube "Box" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"])
    {
        double3 xformOp:translate = (1, 2, 3)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def Plane "Ground" (prepend apiSchemas = ["PhysicsCollisionAPI"])
    {
    }
    def Sphere "RenderOnly"
    {
        double radius = 3
    }
}
"""


def _expected_dump() -> str:
    with open(data_path("physics_population_desc.json"), encoding="utf-8") as f:
        return f.read()


def test_c_form_is_the_generated_description():
    from ovphysx import _bindings

    pointer = _bindings._lib.ovphysx_get_physics_population_desc()
    assert pointer
    assert pointer == _bindings._lib.ovphysx_get_physics_population_desc()
    assert dump_c_desc(pointer) == _expected_dump()


def test_python_form_is_the_generated_description():
    from ovstage import PopulationDomain

    desc = pop.physics_population_desc()
    assert desc is not pop.physics_population_desc()
    assert desc.domains == PopulationDomain.NONE
    assert dump_python_desc(desc) == _expected_dump()


def test_import_neither_builds_it_nor_loads_ovstage():
    code = (
        "import sys, ovphysx.population as p; "
        "assert 'ovstage' not in sys.modules and 'warp' not in sys.modules"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def _read(stage, paths, prim: str, name: str) -> list:
    from ovstage import OrdinalRange

    values = []
    with paths.create_path_list_from_strings([prim]) as selected, stage.query_from_path_list(selected) as query:
        with stage.read_attributes(query, [paths.intern_token(name)], OrdinalRange.latest(1)) as read:
            read.wait()
            for group in read.groups():
                with group:
                    for column in group.columns:
                        if column.is_delete:
                            continue
                        data = column.array(0)
                        for i in range(group.prim_count):
                            row = column.data_row_index(i) if column.has_data_index_map else i
                            values.append(column.array(row).tolist() if column.is_array else data[row].tolist())
    return values


def _populate(descs) -> dict:
    import ovstage
    from ovstage import population
    from test_utils import register_physx_schemas_with_ovstage

    register_physx_schemas_with_ovstage()

    with ovstage.Stage("ovphysx.physics-population-desc") as stage, ovstage.PathDictionary(stage) as paths:
        population.open_usd_from_string_with_desc(stage, _USDA, 1, population.DEFAULT_TIME, descs)
        stage.advance_write_floor(ordinal=1).wait()
        return {
            "gravity": _read(stage, paths, "/World/Scene", "physics:gravityMagnitude"),
            "plane_offset": _read(stage, paths, "/World/Ground", "physxCollision:contactOffset"),
            "box_offset": _read(stage, paths, "/World/Box", "physxCollision:contactOffset"),
            "sleep": _read(stage, paths, "/World/Box", "physxRigidBody:sleepThreshold"),
            "render_radius": _read(stage, paths, "/World/RenderOnly", "radius"),
            # Row-major 4x4: the translation is the last row.
            "box_world": [
                np.asarray(m, dtype=float).reshape(-1)[12:15].tolist()
                for m in _read(stage, paths, "/World/Box", "omni:fabric:worldMatrix")
            ],
        }


def test_alone_it_populates_physics_and_its_stage_unit_defaults():
    got = _populate([pop.physics_population_desc()])
    assert got["gravity"] == [981.0]  # 9.81 / metersPerUnit
    assert got["sleep"] == [50.0]  # 5e-5 * (10 / metersPerUnit) ** 2
    assert got["plane_offset"] == [2.0]  # 0.02 / metersPerUnit, on a Plane collider only
    assert got["box_offset"] == []
    assert got["render_radius"] == []
    # Population seeds the column with the local (1, 2, 3); the seal composes /World's translation.
    assert got["box_world"] == [[11.0, 2.0, 3.0]]


def test_beside_rendering_it_populates_render_prims_too():
    from ovstage import PopulationDomain
    from ovstage.population import Desc

    got = _populate([pop.physics_population_desc(), Desc(domains=PopulationDomain.RENDERING)])
    assert got["gravity"] == [981.0]
    assert got["render_radius"] == [3.0]
    assert got["box_world"] == [[11.0, 2.0, 3.0]]


@pytest.mark.parametrize("shape", ["Cylinder", "Cone"])
@pytest.mark.parametrize("rendering", [False, True])
@pytest.mark.parametrize("margin", [None, 0.0, 0.1])
def test_convex_margin_survives_usd_population(shape, rendering, margin):
    import ovstage
    from ovstage import population

    attribute = "" if margin is None else f"custom float physxConvexGeometry:margin = {margin}"
    usda = f'''#usda 1.0
def {shape} "Collider" (prepend apiSchemas = ["PhysicsCollisionAPI"])
{{
    {attribute}
}}
def {shape} "Visual"
{{
    custom float physxConvexGeometry:margin = 0.2
}}
'''
    with ovstage.Stage("convex-margin") as stage, ovstage.PathDictionary(stage) as paths:
        population.open_usd_from_string_with_desc(
            stage, usda, 1, population.DEFAULT_TIME, pop.physics_population_descs(rendering=rendering)
        )
        stage.advance_write_floor(ordinal=1).wait()
        assert _read(stage, paths, "/Collider", "physxConvexGeometry:margin") == pytest.approx(
            [0.0 if margin is None else margin]
        )
        if not rendering:
            assert _read(stage, paths, "/Visual", "physxConvexGeometry:margin") == []


def test_nested_dtype_mutation_does_not_escape_to_another_caller():
    desc = pop.physics_population_desc()
    rule = next(r for s in desc.selectors for r in s.derived_properties
                if r.name == 'physxRigidBody:sleepThreshold')
    rule.dtype.lanes = 4
    assert dump_python_desc(pop.physics_population_desc()) == _expected_dump()
    assert dump_python_desc(pop.physics_population_descs()[0]) == _expected_dump()


def test_physics_only_population_keeps_authored_non_schema_inputs_and_target_geometry():
    import ovstage
    from test_utils import register_physx_schemas_with_ovstage

    register_physx_schemas_with_ovstage()
    usda = '''#usda 1.0
def PhysicsScene "Scene" (prepend apiSchemas = ["PhysxSceneAPI"])
{
    int physxScene:envIdInBoundsBitCount = 14
}
def Mesh "Sampling" (prepend apiSchemas = ["PhysxParticleSamplingAPI"])
{
    point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
}
def Xform "Deformable" (prepend apiSchemas = ["OmniPhysicsDeformableBodyAPI", "PhysxAutoDeformableBodyAPI"])
{
    rel physxDeformableBody:cookingSourceMesh = </Cooking>
}
def Mesh "Cooking"
{
    point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    uniform token orientation = "leftHanded"
    int[] holeIndices = [0]
}
def Material "Material" (prepend apiSchemas = ["PhysicsMaterialAPI"]) {}
def Xform "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxSplinesSurfaceVelocityAPI", "PhysxVehicleAPI"]
    customData = { dictionary physxVehicle = { bool referenceFrameIsCenterOfMass = false } }
)
{
    rel physxSplinesSurfaceVelocity:surfaceVelocityCurve = </Body/Curve>
    def BasisCurves "Curve" (prepend apiSchemas = ["MaterialBindingAPI"])
    {
        rel material:binding:physics = </Material>
        point3f[] points = [(0, 0, 0), (1, 0, 0)]
        int[] curveVertexCounts = [2]
        uniform token type = "linear"
    }
}
'''
    with ovstage.Stage('population.authored-inputs') as stage, ovstage.PathDictionary(stage) as paths:
        ovstage.population.open_usd_from_string_with_desc(
            stage, usda, 1, ovstage.population.DEFAULT_TIME, pop.physics_population_descs())
        stage.advance_write_floor(ordinal=1).wait()
        assert _read(stage, paths, '/Scene', 'physxScene:envIdInBoundsBitCount') == [14]
        assert _read(stage, paths, '/Sampling', 'points') == [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]]
        assert _read(stage, paths, '/Sampling', 'faceVertexIndices') == [[0, 1, 2]]
        assert _read(stage, paths, '/Cooking', 'holeIndices') == [[0]]
        # Token IDs are interned in this stage's dictionary.
        orientation = _read(stage, paths, '/Cooking', 'orientation')
        assert orientation == [paths.intern_token('leftHanded').raw_id]
        assert _read(stage, paths, '/Body/Curve', 'curveVertexCounts') == [[2]]
        assert _read(stage, paths, '/Body/Curve', 'points') == [[[0, 0, 0], [1, 0, 0]]]
        assert _read(stage, paths, '/Body', 'usd-metadata:customData:physxVehicle:referenceFrameIsCenterOfMass') == [False]
