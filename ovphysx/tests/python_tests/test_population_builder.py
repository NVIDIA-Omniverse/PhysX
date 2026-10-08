# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Behavior of the generated Python population builder (ovphysx.population).

What the builder writes is verified against PhysX by the runtime's generated doctest suite; this
file covers the Python-only contract: lazy dependency import, DLPack dtypes of the published
columns (a contract bool must reach ovstage as kDLBool, not as a uint8 column), the contract rules
that need no PhysX, path-list ownership, and that a legitimately authored value equal to a
divergent raw schema fallback (zero friction) is accepted.
"""

# @implements REQ-CAPI-POPULATION-001
# @covers AC-1 AC-2

import gc
import subprocess
import sys
import warnings

import pytest

import ovphysx.population as pop

K_DL_BOOL = 6
K_DL_FLOAT = 2


def test_import_does_not_pull_warp_or_ovstage():
    # A fresh interpreter: importing the module must not import its heavy dependencies.
    code = (
        "import sys, ovphysx.population as p; "
        "assert 'warp' not in sys.modules and 'ovstage' not in sys.modules, sorted(m for m in sys.modules if m in ('warp', 'ovstage')); "
        "assert p.resolved_defaults('Cube')['size'] == 2.0"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_resolved_defaults_follow_stage_units():
    scene = pop.resolved_defaults("PhysicsScene", meters_per_unit=0.01)
    assert scene["physics:gravityMagnitude"] == pytest.approx(981.0)
    assert tuple(scene["physics:gravityDirection"]) == (0.0, 0.0, -1.0)
    assert tuple(pop.resolved_defaults("PhysicsScene", up_axis="Y")["physics:gravityDirection"]) == (0.0, -1.0, 0.0)


def test_ancestor_paths():
    assert pop.ancestor_paths(["/World/Env/Box_0", "/World/Env/Box_1", "/Other"]) == ["/World", "/World/Env"]


def test_bool_columns_publish_as_dlpack_bool():
    wp = pytest.importorskip("warp")
    ovstage = pytest.importorskip("ovstage")
    tensor, arr = pop._tensor(wp, ovstage, "physics:collisionEnabled", [True, False], "bool", 2, 1)
    assert arr.dtype is wp.bool
    assert (tensor.dtype.code, tensor.dtype.bits, tensor.dtype.lanes) == (K_DL_BOOL, 8, 1)
    tensor, arr = pop._tensor(wp, ovstage, "physics:mass", [1.5], "float32", 1, 1)
    assert (tensor.dtype.code, tensor.dtype.bits) == (K_DL_FLOAT, 32)


def test_prebuilt_arrays_must_match_dtype():
    wp = pytest.importorskip("warp")
    ovstage = pytest.importorskip("ovstage")
    wrong = wp.array([1], dtype=wp.int32, device="cpu")
    with pytest.raises(TypeError, match="expected a float32 array"):
        pop._tensor(wp, ovstage, "physics:mass", wrong, "float32", 1, 1)


@pytest.fixture
def stage_and_paths():
    ovstage = pytest.importorskip("ovstage")
    pytest.importorskip("warp")
    with ovstage.Stage("ovphysx-population-builder") as stage:
        yield stage, ovstage.PathDictionary(stage)


def test_rules_that_need_no_physx(stage_and_paths):
    stage, paths = stage_and_paths
    batch = pop.PrimBatch(paths, ["/World/Scene"])
    batch.define_physics_scene()
    with pytest.raises(ValueError, match="does not apply"):
        batch.apply_physics_rigid_body_api()

    batch = pop.PrimBatch(paths, ["/World/Box"])
    batch.define_cube()
    batch.apply_physx_collision_api()  # requires PhysicsCollisionAPI on the same prim
    with pytest.raises(ValueError, match="requires"):
        batch.write(stage, 1)

    batch = pop.PrimBatch(paths, ["/World/Joint"])
    batch.define_physics_joint()
    batch.apply_physics_drive_api("rotX")
    batch.apply_physx_drive_performance_envelope_api("rotY")  # extends the same-instance drive only
    with pytest.raises(ValueError, match="PhysicsDriveAPI:rotY"):
        batch.write(stage, 1)


def test_transform_fields_are_rejected_on_non_xformable_types(stage_and_paths):
    stage, paths = stage_and_paths
    # The public define_<type> of a non-Xformable type has no transform keywords at all ...
    with pytest.raises(TypeError):
        pop.PrimBatch(paths, ["/World/Scene"]).define_physics_scene(reset_xform_stack=[True])
    # ... and the shared _define rejects every supplied transform field, the reset flag included.
    for local, reset, world in (([[1.0] * 16], None, None), (None, [True], None), (None, None, [[1.0] * 16])):
        batch = pop.PrimBatch(paths, ["/World/Scene"])
        with pytest.raises(ValueError, match="not Xformable"):
            batch._define("PhysicsScene", local, reset, world)


def test_batch_is_reusable_and_resends_the_full_prim(stage_and_paths):
    # Like the C++ builder, the payload columns survive write(); a retry or a second write must send
    # the whole prim again rather than only the structural columns appended last.
    stage, paths = stage_and_paths
    batch = pop.PrimBatch(paths, ["/World/Box"])
    batch.define_cube(size=[2.0])
    batch.apply_physics_rigid_body_api()
    batch.apply_physics_collision_api()
    payload = list(batch._writes)
    batch.write(stage, 1)
    assert batch._writes == payload
    batch.write(stage, 2)
    assert batch._writes == payload


def test_zero_friction_is_an_accepted_authored_value(stage_and_paths):
    # 0.0 is the raw schema fallback and parses differently from an absent column (0.5), but a value
    # does not encode authoredness: an explicitly authored zero-friction material must be written.
    stage, paths = stage_and_paths
    batch = pop.PrimBatch(paths, ["/World/Ice"])
    batch.define_material()
    batch.apply_physics_material_api(static_friction=[0.0], dynamic_friction=[0.0])
    batch.write(stage, 1)
    assert "physics:staticFriction" in pop.RAW_FALLBACK_DIVERGES


def test_write_releases_its_path_list(stage_and_paths):
    stage, paths = stage_and_paths
    pop.write_stage_units(stage, paths, 1, meters_per_unit=0.01)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        pop.create_rigid_body_cubes(stage, paths, ["/World/Box_0", "/World/Box_1"], 1, cube={"size": [2.0, 2.0]})
        gc.collect()
    leaked = [w for w in caught if issubclass(w.category, ResourceWarning)]
    assert not leaked, [str(w.message) for w in leaked]
