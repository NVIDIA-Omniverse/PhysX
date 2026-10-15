# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-PATTERN-002
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
# @maps_to TEST-CAPI-PATTERN-002

"""The path-pattern grammar of the "Path patterns" documentation page, asserted rule by rule.

Every row of the page's examples table runs over the same hierarchy (tests/data/path_patterns.usda).
Each pattern is resolved through the session read's prim-set selection and through the deprecated
tensor binding, which share one matcher, so both must select exactly the documented rigid bodies.
"""

import warnings

import ovstage
import pytest
from ovphysx.types import SimObjectType, TensorType
from test_utils import attach_usd_with_ovstage, data_path

_E0_BASE = "/envs/env0/Robot/base_link"
_E0_LINK = "/envs/env0/Robot/arm/link_0"
_E0_GROUP = "/envs/env0/Group/Robot/base_link"
_E1_BASE = "/envs/env1/Robot/base_link"
_E1_LINK = "/envs/env1/Robot/arm/link_0"
_EA_BASE = "/envs/envA/Robot/base_link"
# /nested: a base_link body inside a base_link body, and one inside a base_link transform.
_N_OUTER = "/nested/Robot/base_link"
_N_INNER = "/nested/Robot/base_link/tool/base_link"
_W_INNER = "/nested/Wrapped/base_link/base_link"

# (pattern, selected rigid bodies). The first block is the page's examples table, in order.
_ROWS = [
    ("/envs/*/Robot/base_link", [_E0_BASE, _E1_BASE, _EA_BASE]),
    ("/envs/*/**/Robot/base_link", [_E0_BASE, _E0_GROUP, _E1_BASE, _EA_BASE]),
    ("/envs/*/Robot/link_0", [_E0_LINK, _E1_LINK]),
    ("/envs/*/Robot/*", [_E0_BASE, _E1_BASE, _EA_BASE]),
    ("/envs/*/Robot/**", [_E0_BASE, _E0_LINK, _E1_BASE, _E1_LINK, _EA_BASE]),
    ("/envs/env[0-9]/Robot/base_link", [_E0_BASE, _E1_BASE]),
    ("/envs/env?/Robot/base_link", [_E0_BASE, _E1_BASE, _EA_BASE]),
    ("/envs/env0/Robot/base_link|link_0", [_E0_BASE, _E0_LINK]),
    ("/envs/env0/Robot/base|link_0", [_E0_LINK]),
    ("/envs/env.*/Robot/base_link", []),
    ("/envs/env0/link_0", [_E0_LINK]),
    ("/envs/env[0/Robot/base_link", []),
    ("/envs/env0/Robot/(base_link|arm/link_0)", [_E0_BASE]),
    # A trailing slash is ignored.
    ("/envs/*/Robot/base_link/", [_E0_BASE, _E1_BASE, _EA_BASE]),
    # Repetition and a parenthesized alternation in an earlier component.
    ("/envs/env[0-9]+/Robot/base_link", [_E0_BASE, _E1_BASE]),
    ("/envs/(env0|envA)/Robot/base_link", [_E0_BASE, _EA_BASE]),
    # `**` inside a longer component is two `*`, so the last component is still searched at any depth.
    ("/envs/e**/Robot/link_0", [_E0_LINK, _E1_LINK]),
    # `**` matches zero levels; with a `**` in the pattern the last component is direct children only.
    ("/envs/env0/Robot/**/base_link", [_E0_BASE]),
    # A literal path to an existing prim selects exactly that prim.
    (_E0_BASE, [_E0_BASE]),
    # Same-name suppression, also when the outer match is a transform rather than a body.
    ("/nested/*/base_link", [_N_OUTER]),
    ("/nested/*/**/base_link", [_N_OUTER, _N_INNER, _W_INNER]),
]


def _read_paths(sdk, stage, pattern):
    """Sorted prim paths a pattern-selected rigid-body read covers."""
    out = []
    with sdk.read_sync(SimObjectType.RIGID_BODY, ["position"], pattern=pattern) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                handles = dictionary.get_paths(group.prim_list)
                out.extend(
                    dictionary.path_to_string(h)
                    for h in handles[group.prim_offset : group.prim_offset + group.prim_count]
                )
    return sorted(out)


def _binding_paths(sdk, pattern):
    """Sorted prim paths a pattern-bound rigid-body tensor binding covers."""
    # The tensor binding is deprecated; this parity check is an intentional compat use.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        binding = sdk.create_tensor_binding(pattern=pattern, tensor_type=TensorType.RIGID_BODY_POSE)
        try:
            return sorted(binding.prim_paths)
        finally:
            binding.destroy()


@pytest.fixture
def pattern_scene(physx_sdk_cpu):
    stage = attach_usd_with_ovstage(physx_sdk_cpu, data_path("path_patterns.usda"))
    physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    return physx_sdk_cpu, stage


def test_documented_patterns_select_documented_bodies(pattern_scene):
    """Every pattern selects exactly its documented bodies, through the read and the tensor binding alike."""
    sdk, stage = pattern_scene
    mismatches = []
    for pattern, expected in _ROWS:
        got_read = _read_paths(sdk, stage, pattern)
        got_binding = _binding_paths(sdk, pattern)
        if got_read != sorted(expected) or got_binding != sorted(expected):
            mismatches.append(f"{pattern!r}: expected {sorted(expected)}, read {got_read}, binding {got_binding}")
    assert not mismatches, "\n".join(mismatches)


def test_relative_pattern_is_refused(pattern_scene):
    """A pattern must be absolute: the read rejects a relative one, and the tensor binding selects nothing."""
    sdk, stage = pattern_scene
    with pytest.raises(RuntimeError, match="absolute path"):
        _read_paths(sdk, stage, "envs/*/Robot/base_link")
    assert _binding_paths(sdk, "envs/*/Robot/base_link") == []


def test_physics_only_clones_follow_the_same_rules(physx_sdk_cpu):
    """A replicator clone with no authored prim is matched like an authored one."""
    sdk = physx_sdk_cpu
    stage = attach_usd_with_ovstage(sdk, data_path("path_patterns.usda"))
    # Cloning must precede the first step.
    sdk.clone("/envs/envA", ["/envs/envB"], anchor_transforms=[(18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)])
    sdk.wait_all()
    sdk.step(1.0 / 60.0)
    sdk.wait_all()

    expected = [_EA_BASE, "/envs/envB/Robot/base_link"]
    assert _read_paths(sdk, stage, "/envs/env[AB]/Robot/base_link") == expected
    assert _binding_paths(sdk, "/envs/env[AB]/Robot/base_link") == expected
