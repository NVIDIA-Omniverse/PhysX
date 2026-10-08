# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# TODO(tensor-binding-deprecation): the clone assertions count matches through binding.count
# (a pattern / prim_paths -> match-count accessor). The session read has no equivalent (its query
# is by object type + scope, not by path pattern), so the binding stays until read gains path
# selection. Only the count probe needs it. The clone behaviour under test is API-independent.

"""Python tests for clone functionality.

Tests the high-level Python API for cloning USD prim hierarchies.
Comprehensive clone functionality is tested in the C layer (test_clone.cpp).
These tests verify the Python wrapper correctly calls through and handles errors.
"""

import os

import pytest
from ovphysx.dlpack import DLDataTypeCode
from ovphysx.types import ObjectScope, SimObjectType, TensorType
from test_utils import load_usd_with_ovstage


def test_clone_basic_functionality(physx_sdk):
    """Smoke test: verify clone works through Python wrapper."""
    sdk = physx_sdk

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    # Clone to multiple targets (proves list marshaling works)
    target_paths = [f"/World/envs/env{i}" for i in range(1, 4)]
    sdk.clone("/World/envs/env0", target_paths)
    sdk.wait_all()

    # The clones must actually materialize as addressable rigid bodies, not be a
    # silent no-op: env0/table is the source body, so each clone env must carry a
    # replicated table body. The tables are the scene's only rigid bodies, so a whole-set
    # read must see exactly 4 (source + 3 clones). Warm up first, because a read needs the first step.
    sdk.warmup()
    sdk.wait_all()
    with sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        body_count = sum(g.prim_count for g in result.groups)
    assert body_count == 4, f"expected 4 bodies (source + 3 clones), got {body_count}"

    # Verify simulation still works with clones
    sdk.step(1.0 / 60.0)
    sdk.wait_all()


def test_clone_error_handling(physx_sdk):
    """Verify Python wrapper converts C errors to Python exceptions correctly."""
    sdk = physx_sdk

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    with pytest.raises(ValueError, match="source_path must be a non-empty string"):
        sdk.clone("", ["/World/envs/env1"])

    with pytest.raises(ValueError, match="target_paths must be a non-empty list"):
        sdk.clone("/World/envs/env0", [])

    with pytest.raises(ValueError, match="cannot be the same as source path"):
        sdk.clone("/World/envs/env0", ["/World/envs/env0"])


def test_clone_transform_tuple_length_validation(physx_sdk):
    """Each anchor_transforms entry must be exactly 7 finite floats.

    A short tuple would make the native path read past the ctypes buffer. A
    long one would shift every later target's pose. Both are rejected before
    the C call.
    """
    sdk = physx_sdk

    # Short (6 values) -> reject.
    with pytest.raises(ValueError, match="exactly 7"):
        sdk.clone("/World/envs/env0", ["/World/envs/env1"],
                  anchor_transforms=[(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)])

    # Long (8 values) -> reject.
    with pytest.raises(ValueError, match="exactly 7"):
        sdk.clone("/World/envs/env0", ["/World/envs/env1"],
                  anchor_transforms=[(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)])

    # Non-finite value -> reject.
    with pytest.raises(ValueError, match="finite"):
        sdk.clone("/World/envs/env0", ["/World/envs/env1"],
                  anchor_transforms=[(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float("inf"))])


def test_clone_env_ids_validation(physx_sdk):
    """env_ids must match target_paths in length and stay below 0x00FFFFFF.

    Wrong lengths would desync targets from ids in the native call. PhysX requires
    every environment id to be < 1<<24 (runtime id = env_ids[i] + 1), so the caller
    id must be < 0x00FFFFFF. The boundary 0x00FFFFFF and too-wide values are all
    rejected before the C call.
    """
    sdk = physx_sdk

    # Length mismatch -> reject.
    with pytest.raises(ValueError, match="env_ids length"):
        sdk.clone("/World/envs/env0", ["/World/envs/env1", "/World/envs/env2"], env_ids=[0])

    # Out-of-range values (the boundary and too-wide values) -> reject.
    for bad in (-1, 0x00FFFFFF, 0x01000000, 0xFFFFFFFE, 0xFFFFFFFF):
        with pytest.raises(ValueError, match="env_ids\\[0\\]"):
            sdk.clone("/World/envs/env0", ["/World/envs/env1"], env_ids=[bad])

    # Non-integers must not be silently coerced (for example 1.5 -> 1 or "7" -> 7).
    for bad in ("zero", "7", 1.5):
        with pytest.raises(ValueError, match="env_ids\\[0\\]"):
            sdk.clone("/World/envs/env0", ["/World/envs/env1"], env_ids=[bad])

    # The maximum valid caller id (0x00FFFFFE -> runtime 0x00FFFFFF) must pass validation.
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    load_usd_with_ovstage(sdk, os.path.join(test_dir, "data", "basic_simulation.usda"))
    sdk.wait_all()
    sdk.clone("/World/envs/env0", ["/World/envs/env1"], env_ids=[0x00FFFFFE])
    sdk.wait_all()


def test_clone_with_env_ids(physx_sdk):
    """Caller-supplied logical env ids pass end-to-end through the Python wrapper.

    The CPU fixture never engages env-id filtering (GPU-only), so this validates
    marshaling and that the clone succeeds and simulates. The id semantics
    (same id across calls -> same runtime environment) are covered by the
    GPU-side ovruntime doctest.
    """
    sdk = physx_sdk
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    # Two calls sharing logical ids, the heterogeneous ClonePlan shape.
    transforms = [(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), (10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)]
    sdk.clone("/World/envs/env0", ["/World/envs/env1", "/World/envs/env2"],
              anchor_transforms=transforms, env_ids=[7, 3])
    sdk.wait_all()

    sdk.step(1.0 / 60.0)
    sdk.wait_all()


def test_clone_error_no_usd_loaded(physx_sdk):
    """Verify clone without an attached ovstage raises RuntimeError."""
    sdk = physx_sdk

    with pytest.raises(RuntimeError, match="no ovstage is attached"):
        sdk.clone("/World/envs/env0", ["/World/envs/env1"])


def test_clone_preserves_source_scale(physx_sdk):
    """Verify that cloning a source with xformOp:scale does not crash or error.

    Exercises the Python clone path end-to-end with a source that has
    non-trivial root transforms (translate + orient + scale), placing the
    clone via an explicit anchor transform and simulating a few steps.
    """
    sdk = physx_sdk
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "clone_with_root_transforms.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    # Clone with explicit transform: place env1 at (5, 0, 0) with identity rotation
    sdk.clone(
        "/World/envs/env0",
        ["/World/envs/env1"],
        anchor_transforms=[(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)],
    )
    sdk.wait_all()

    # Simulate to verify no crashes with scaled source
    for _ in range(10):
        sdk.step(1.0 / 60.0)
        sdk.wait_all()


def test_clone_multiple_targets_stress(physx_sdk):
    """Test cloning with many target paths (stress test).

    Covered APIs:
        ovstage attach/update helper
        PhysX.clone (with many targets)
        PhysX.wait_all

    Args:
        physx_sdk: PhysX SDK fixture

    Returns:
        None: Ensures clone can handle many targets or reports resource limits.
    """
    # Use basic_simulation.usda which has proper cloneable structure
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(physx_sdk, usd_path)
    physx_sdk.wait_all()

    target_paths = [f"/World/envs/env{i}" for i in range(10, 20)]  # env10-env19

    try:
        physx_sdk.clone("/World/envs/env0", target_paths)
        physx_sdk.wait_all()
    except RuntimeError as e:
        # If clone fails, verify error indicates resource/limit issue, not a bug
        error_msg = str(e).lower()
        assert any(
            keyword in error_msg for keyword in ["resource", "limit", "memory", "too many", "exceeded", "capacity"]
        ), f"Stress test failure should indicate resource limits, not a bug. Got: {e}"


def test_clone_after_warmup_raises_runtime_error(physx_sdk):
    """Regression test for NVBug 6172717: cloning after warmup must raise.

    Cloning after :meth:`warmup` reallocates physics structures and corrupts
    already-initialised solver state. The C runtime rejects this ordering with
    ``OVPHYSX_API_INVALID_ARGUMENT``, which the Python wrapper propagates as
    ``RuntimeError``. This guard applies in both CPU and GPU mode.
    """
    sdk = physx_sdk
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    # Lock in warmup. It is idempotent and synchronous, and subsequent clones must fail.
    sdk.warmup()

    with pytest.raises(RuntimeError, match="clone\\(\\) must be called before warmup"):
        sdk.clone("/World/envs/env0", ["/World/envs/env_after_warmup"])


def test_clone_after_reset_succeeds(physx_sdk):
    """Clean-recovery: warmup() -> reset() -> ovstage attach/update -> clone() succeeds.

    Exercises the warmup-then-reset-and-start-over path with no failed clone in
    between. Regression for NVBug 6172717: proves reset() clears
    the C-side ``warmup_done`` flag so subsequent clone() works.

    Pair with ``test_clone_failed_after_warmup_then_reset_succeeds`` which
    covers the user-facing failure-recovery path (warmup -> failed clone ->
    reset -> ovstage attach/update -> clone).
    """
    sdk = physx_sdk
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()
    sdk.warmup()

    # reset() invalidates usd handles and clears the C-side warmup_done flag.
    sdk.reset_stage()
    sdk.wait_all()
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    # Clone should now succeed since warmup state was cleared by reset.
    sdk.clone("/World/envs/env0", ["/World/envs/env_post_reset"])
    sdk.wait_all()


def test_clone_failed_after_warmup_then_reset_succeeds(physx_sdk):
    """Documented recovery: warmup -> clone (raises) -> reset -> ovstage attach/update -> clone.

    This is the exact user-facing path the error message advises (reset, then
    reload or reattach the source stage). Regression for NVBug 6172717 and the
    orphan async op: the synchronous precondition failure in ``ovphysx_clone``
    uses ``set_enqueue_error`` rather than ``fail_with_event``, so no failed op
    is registered for ``ovphysx_reset_stage()``'s pending-op drain to trip over.
    """
    sdk = physx_sdk
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()
    sdk.warmup()

    # First: clone() must raise.
    with pytest.raises(RuntimeError, match="clone\\(\\) must be called before warmup"):
        sdk.clone("/World/envs/env0", ["/World/envs/env_after_warmup"])

    # Then: the documented recovery path must work end-to-end. reset_stage() drains
    # pending ops internally. If the failed clone were registered as a failed async
    # op, this call would fail.
    sdk.reset_stage()
    sdk.wait_all()
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    sdk.clone("/World/envs/env0", ["/World/envs/env_after_reset"])
    sdk.wait_all()


def test_clone_subtree_with_physics_scene(physx_sdk):
    """Repro NVBugs 6421194: clone a subtree that contains the singleton PhysicsScene.

    ``boxes_falling_on_groundplane.usda`` has 11 dynamic bodies plus the scene's
    only PhysicsScene directly under ``/World``, so cloning ``/World`` clones a
    subtree that contains the singleton scene. The runtime skips the non-replicable
    scene node (``eScene`` / type 262144) and replicates the 11 bodies, so the
    clone must be functional and fully addressable.
    """
    sdk = physx_sdk

    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "boxes_falling_on_groundplane.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    def count(**binding_kwargs):
        binding = sdk.create_tensor_binding(
            tensor_type=TensorType.RIGID_BODY_POSE, **binding_kwargs
        )
        try:
            return binding.count
        finally:
            binding.destroy()

    # Source sanity: the wildcard resolves every dynamic body in the USD-authored source.
    assert count(pattern="/World/*") == 11, "expected 11 source bodies"

    # Clone the whole /World subtree (which contains the singleton PhysicsScene).
    # clone() reports success: the non-replicable scene node is skipped, bodies replicate.
    sdk.clone("/World", ["/World_clone0"])
    sdk.wait_all()

    # The clone must be functional: every cloned body addressable by explicit path.
    cube_paths = [f"/World_clone0/Cube{i}" for i in range(1, 12)]
    assert count(prim_paths=cube_paths) == 11, (
        "replication should materialize all 11 cloned bodies (explicit paths)"
    )

    # The wildcard over the clone root must resolve the same 11 bodies.
    clone_wildcard = count(pattern="/World_clone0/*")
    assert clone_wildcard == 11, (
        f"wildcard /World_clone0/* should resolve 11 cloned bodies, got {clone_wildcard}"
    )


def test_clone_single_body_to_top_level_target(physx_sdk):
    """NVBugs 6421194: clone a single body that is itself the top-level clone root.

    Cloning ``/World/Cube1`` -> ``/SoloClone`` makes the cloned body the subtree
    root with no children, so the registration helper (``addPrimSubtree``) is a
    no-op and creates nothing. The clone root must still be materialized under the
    pseudo-root, or both a literal pattern and a wildcard miss it. This is the
    direct-body top-level case the 11-child fixture above hides.
    """
    sdk = physx_sdk
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "boxes_falling_on_groundplane.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    def count(**binding_kwargs):
        binding = sdk.create_tensor_binding(
            tensor_type=TensorType.RIGID_BODY_POSE, **binding_kwargs
        )
        try:
            return binding.count
        finally:
            binding.destroy()

    sdk.clone("/World/Cube1", ["/SoloClone"])
    sdk.wait_all()

    assert count(prim_paths=["/SoloClone"]) == 1, "explicit path must resolve the single-body clone"
    assert count(pattern="/SoloClone") == 1, "literal pattern must resolve the single-body clone"
    assert count(pattern="/Solo*") == 1, "wildcard must resolve the single-body clone root"


def test_clone_nested_subtree_to_top_level_target(physx_sdk):
    """NVBugs 6421194: clone a nested subtree onto a brand-new top-level target.

    Cloning ``/World/envs`` -> ``/EnvsClone`` places the body two levels below the
    clone root (``/EnvsClone/env0/table``). ``addPrimSubtree`` leaves the immediate
    child detached (empty parent) and never creates a root-prim clone root, so the
    merge must materialize the root and re-home the intermediate node beneath it.
    Otherwise multi-segment and recursive wildcards cannot reach the nested body.
    """
    sdk = physx_sdk
    test_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    usd_path = os.path.join(test_dir, "data", "basic_simulation.usda")
    load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_all()

    def count(**binding_kwargs):
        binding = sdk.create_tensor_binding(
            tensor_type=TensorType.RIGID_BODY_POSE, **binding_kwargs
        )
        try:
            return binding.count
        finally:
            binding.destroy()

    sdk.clone("/World/envs", ["/EnvsClone"])
    sdk.wait_all()

    assert count(prim_paths=["/EnvsClone/env0/table"]) == 1, "explicit nested clone body must resolve"
    assert count(pattern="/EnvsClone/**") == 1, "recursive wildcard must reach the nested clone body"
    assert count(pattern="/EnvsClone/*/table") == 1, "multi-segment wildcard must resolve the nested body"
