# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Repro tests for NVBugs 6078079 / OMPE-89777:
#   create_rigid_body_view / create_articulation_view fail to match bodies
#   when they are nested deeper in the USD hierarchy than the pattern level.
#
# Isaac Sim v3.0 URDF / MJCF converters produce a USD hierarchy in which
# articulation links (rigid bodies) are nested inside each other — the
# kinematic parent link in USD contains its child link as a descendant:
#
#     /World/envs/env_*/Robot/base_link/torso_yaw_link/...   (v3.0)
#
# The tensor API's `findMatchingPaths` (BaseSimulationView.cpp) splits
# patterns by `/` and matches level-by-level. Three features address the
# nested-layout scenarios this file exercises:
#
#   1. A `**` token matches zero or more path components, so
#      `/envs/*/Robot/**` collects every rigid body under Robot/ regardless
#      of depth. Preferred form for new code.
#
#   2. Leaf-recursive matching: when the last token is a *named* glob
#      (e.g. `link_1`, `foo*`, `(hip|thigh)`) it is matched against every
#      descendant name, not just direct children. A bare `*` is NOT
#      promoted — callers who want "every body below this prim" must use
#      `**` explicitly. Gated by the setting
#      `/physics/tensors/recursiveLeafPatternMatch` (default true).
#
#   3. The tokenizer no longer splits `/` that sits inside a balanced
#      `(...)` group. Names still can't contain `/`, so group alternations
#      spanning multiple prim names remain unsupported — documented and
#      intentionally marked `@unittest.expectedFailure` by the
#      `TestRigidBodyViewNestedAlternation` scenario below. See that
#      class's docstring for a full explanation and supported alternatives.
#
# `create_articulation_view` is unaffected by nesting once the pattern
# reaches any prim that carries ArticulationRootAPI or is an articulation
# root link — PhysX then walks the articulation tree itself and reports
# all links regardless of USD nesting depth.
#
# These tests construct two minimal scenes that mirror the converter's
# nested layout and exercise both `create_rigid_body_view` and
# `create_articulation_view` with patterns similar to those used by
# Isaac Lab's ContactSensorCfg.

import sys
import unittest

import numpy as np  # noqa: F401
import warp as wp  # noqa: F401

import _tensors_setup  # noqa: F401

import carb.settings
import omni.physics.tensors
import warp_utils as wp_utils  # noqa: F401

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema

from omni.physx.scripts import physicsUtils

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory,
)


_WARM_START = True
_FRONTEND = "warp"


# ---------------------------------------------------------------------------
# Scene builders
# ---------------------------------------------------------------------------

def _build_nested_link_articulation(scenario, robot_path, transform, num_links=3):
    """Build an articulation whose links are nested hierarchically in USD.

    Layout (mirrors Isaac Sim v3.0 URDF converter output):
        <robot_path>                            Xform, ArticulationRootAPI
            base_link                           rigid body (articulation root link)
                BaseFixedJoint                  FixedJoint — fixes base to world
                link_0                          rigid body NESTED inside base_link
                    Joint_0                     RevoluteJoint (base_link -> link_0)
                    link_1                      rigid body NESTED inside link_0
                        Joint_1                 RevoluteJoint (link_0 -> link_1)
                        ...

    Every `link_i` is a USD descendant of its kinematic parent, so a
    pattern that matches only direct children of <robot_path> will miss
    all of them except base_link.
    """
    stage = scenario.stage
    robot_path = Sdf.Path(robot_path)

    robot_xform = UsdGeom.Xform.Define(stage, robot_path)
    physicsUtils.set_or_add_scale_orient_translate(
        robot_xform, scale=Gf.Vec3f(1.0), orient=transform.q, translate=transform.p
    )
    robot_prim = robot_xform.GetPrim()

    UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    arti_api = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
    arti_api.CreateSleepThresholdAttr(0.0)
    arti_api.CreateEnabledSelfCollisionsAttr().Set(False)

    base_link_path = robot_path.AppendChild("base_link")
    physicsUtils.add_rigid_sphere(stage, base_link_path, 0.1, Gf.Vec3f(0.0))

    # Fix base_link to world.
    fixed_joint = UsdPhysics.FixedJoint.Define(
        stage, base_link_path.AppendChild("BaseFixedJoint")
    )
    fixed_joint.CreateBody1Rel().SetTargets([base_link_path])

    link_size = Gf.Vec3f(0.3, 0.05, 0.05)
    parent_link_path = base_link_path
    link_paths = [base_link_path]
    for i in range(num_links):
        link_path = parent_link_path.AppendChild(f"link_{i}")
        # Child link placed relative to the parent (nesting makes the position
        # relative to the parent's xform).
        link_position = Gf.Vec3f(0.3, 0.0, 0.0) if i > 0 else Gf.Vec3f(0.15, 0.0, 0.0)
        physicsUtils.add_rigid_box(stage, link_path, link_size, link_position)
        physicsUtils.add_mass(stage, link_path, 1.0)

        joint = UsdPhysics.RevoluteJoint.Define(
            stage, link_path.AppendChild(f"Joint_{i}")
        )
        joint.CreateAxisAttr("Z")
        joint.CreateBody0Rel().SetTargets([parent_link_path])
        joint.CreateBody1Rel().SetTargets([link_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.15, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.15, 0.0, 0.0))

        link_paths.append(link_path)
        parent_link_path = link_path

    return {
        "robot": robot_path,
        "base_link": base_link_path,
        "links": link_paths,
    }


def _build_nested_standalone_rigid_bodies(scenario, root_path, transform, num_levels=3):
    """Build a chain of standalone rigid bodies nested under intermediate
    Xforms at increasing depths. No articulation.

    Layout:
        <root_path>                       Xform
            body_0                        rigid body (depth +1)
            sub_1                         Xform
                body_1                    rigid body (depth +2)
                sub_2                     Xform
                    body_2                rigid body (depth +3)
                    ...

    This is the simplest reproducer for the level-by-level matching issue
    on non-articulated (standalone) rigid bodies — every `body_i` lives
    at a different USD depth so no single fixed-depth pattern finds them all.
    """
    stage = scenario.stage
    root_path = Sdf.Path(root_path)

    root_xform = UsdGeom.Xform.Define(stage, root_path)
    physicsUtils.set_or_add_scale_orient_translate(
        root_xform, scale=Gf.Vec3f(1.0), orient=transform.q, translate=transform.p
    )

    size = Gf.Vec3f(0.1, 0.1, 0.1)
    body_paths = []
    parent_path = root_path
    for i in range(num_levels):
        body_path = parent_path.AppendChild(f"body_{i}")
        pos = Gf.Vec3f(0.0, 0.0, 0.2 * (i + 1))
        physicsUtils.add_rigid_box(stage, body_path, size, pos)
        physicsUtils.add_mass(stage, body_path, 1.0)
        body_paths.append(body_path)

        if i < num_levels - 1:
            sub_path = parent_path.AppendChild(f"sub_{i + 1}")
            UsdGeom.Xform.Define(stage, sub_path)
            parent_path = sub_path

    return {
        "root": root_path,
        "bodies": body_paths,
    }


def _build_repeated_leaf_name_articulation(scenario, robot_path, transform):
    """Build an articulation whose hierarchy reuses the leaf name ``base_link``
    at two different depths, with each occurrence a distinct rigid body.

    Layout:
        <robot_path>                          Xform, ArticulationRootAPI
            base_link                         rigid body (outer, depth +1)
                BaseFixedJoint                FixedJoint — fixes outer base to world
                tool                          Xform (depth +2)
                    base_link                 rigid body (inner, depth +3)
                        ToolJoint             RevoluteJoint (outer -> inner)

    Both ``base_link`` prims are separate rigid bodies. The pattern
    ``/envs/*/Robot/base_link`` used to be an unambiguous direct-child
    match for the outer body; with recursive-leaf matching now the
    default, it additionally matches the inner ``tool/base_link`` because
    the leaf is searched subtree-wide. The two scenarios below pin down
    both branches of that behavioural fork.
    """
    stage = scenario.stage
    robot_path = Sdf.Path(robot_path)

    robot_xform = UsdGeom.Xform.Define(stage, robot_path)
    physicsUtils.set_or_add_scale_orient_translate(
        robot_xform, scale=Gf.Vec3f(1.0), orient=transform.q, translate=transform.p
    )
    robot_prim = robot_xform.GetPrim()

    UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    arti_api = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
    arti_api.CreateSleepThresholdAttr(0.0)
    arti_api.CreateEnabledSelfCollisionsAttr().Set(False)

    outer_base_link_path = robot_path.AppendChild("base_link")
    physicsUtils.add_rigid_sphere(stage, outer_base_link_path, 0.1, Gf.Vec3f(0.0))

    fixed_joint = UsdPhysics.FixedJoint.Define(
        stage, outer_base_link_path.AppendChild("BaseFixedJoint")
    )
    fixed_joint.CreateBody1Rel().SetTargets([outer_base_link_path])

    tool_xform_path = outer_base_link_path.AppendChild("tool")
    UsdGeom.Xform.Define(stage, tool_xform_path)

    inner_base_link_path = tool_xform_path.AppendChild("base_link")
    physicsUtils.add_rigid_box(
        stage, inner_base_link_path, Gf.Vec3f(0.2, 0.05, 0.05), Gf.Vec3f(0.3, 0.0, 0.0)
    )
    physicsUtils.add_mass(stage, inner_base_link_path, 1.0)

    joint = UsdPhysics.RevoluteJoint.Define(
        stage, inner_base_link_path.AppendChild("ToolJoint")
    )
    joint.CreateAxisAttr("Z")
    joint.CreateBody0Rel().SetTargets([outer_base_link_path])
    joint.CreateBody1Rel().SetTargets([inner_base_link_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.15, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-0.15, 0.0, 0.0))

    return {
        "robot": robot_path,
        "outer_base_link": outer_base_link_path,
        "inner_base_link": inner_base_link_path,
    }


# ---------------------------------------------------------------------------
# Scenarios: articulation with nested links
# ---------------------------------------------------------------------------

class _NestedLinkArticulationBase(GridTestBase):
    """Grid scenario: each env has a Robot whose articulation links are
    nested hierarchically in USD under `/envs/env_i/Robot/base_link/...`.
    """

    NUM_CHILD_LINKS = 3
    LINKS_PER_ROBOT = 1 + NUM_CHILD_LINKS  # base_link + link_0..link_{N-1}

    def __init__(self, test_case, device_params, num_envs=4):
        grid_params = GridParams(num_envs, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        robot_path = self.env_template_path.AppendChild("Robot")
        transform = Transform(Gf.Vec3f(0.0, 0.0, 1.0))
        self._paths = _build_nested_link_articulation(
            self, robot_path, transform, num_links=self.NUM_CHILD_LINKS
        )


class TestArticulationViewNestedLinks(_NestedLinkArticulationBase):
    """create_articulation_view should locate the articulation regardless of
    how deeply its root link is nested — the pattern reaches the articulation
    root prim, and PhysX then exposes every link in the articulation tree.
    """

    def on_start(self, sim):
        # Point the pattern at the articulation root (the Robot Xform that
        # carries ArticulationRootAPI). PhysX exposes all nested links.
        arti_view = sim.create_articulation_view("/envs/*/Robot")

        self.check_articulation_view(
            arti_view,
            expected_count=self.num_envs,
            expected_max_links=self.LINKS_PER_ROBOT,
            expected_max_dofs=self.NUM_CHILD_LINKS,
            require_homogeneous=True,
        )

        # prim_paths returns the articulation-root prim (Robot Xform),
        # not any of the individual nested links.
        for i in range(self.num_envs):
            self.test_case.assertIn(f"/envs/env{i}/Robot", arti_view.prim_paths)

        # Metatype must enumerate every nested link.
        mt = arti_view.shared_metatype
        self.test_case.assertIsNotNone(mt)
        self.test_case.assertIn("base_link", mt.link_indices)
        for i in range(self.NUM_CHILD_LINKS):
            self.test_case.assertIn(f"link_{i}", mt.link_indices)

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationViewAtNestedRootLink(_NestedLinkArticulationBase):
    """Same articulation, but the pattern points at the nested articulation
    root *link* rather than the Robot Xform. This is the pattern shape the
    bug report calls out as currently working:

        `create_articulation_view("/World/envs/env_*/Robot/base_link")`.
    """

    def on_start(self, sim):
        arti_view = sim.create_articulation_view("/envs/*/Robot/base_link")

        self.check_articulation_view(
            arti_view,
            expected_count=self.num_envs,
            expected_max_links=self.LINKS_PER_ROBOT,
            expected_max_dofs=self.NUM_CHILD_LINKS,
            require_homogeneous=True,
        )
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewExplicitNestedPaths(_NestedLinkArticulationBase):
    """Baseline workaround: a list of fully-qualified per-link paths
    bypasses the regex tokenizer and matches all nested links. Tests the
    fallback Isaac Lab users currently reach for.
    """

    def on_start(self, sim):
        paths = []
        for i in range(self.num_envs):
            paths.append(f"/envs/env{i}/Robot/base_link")
            # Build the nested chain path for each child link.
            chain = f"/envs/env{i}/Robot/base_link"
            for j in range(self.NUM_CHILD_LINKS):
                chain = f"{chain}/link_{j}"
                paths.append(chain)

        rb_view = sim.create_rigid_body_view(paths)
        self.check_rigid_body_view(rb_view, self.num_envs * self.LINKS_PER_ROBOT)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewRecursiveUnderRobot(_NestedLinkArticulationBase):
    """Fix verification (bug 6078079): the `**` recursive-descent token
    collects rigid bodies at any depth inside a subtree. This is the
    supported replacement for `.../Robot/*` patterns once users move to
    the v3.0 URDF layout.
    """

    def on_start(self, sim):
        rb_view = sim.create_rigid_body_view("/envs/*/Robot/**")

        expected_count = self.num_envs * self.LINKS_PER_ROBOT
        self.check_rigid_body_view(rb_view, expected_count)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewRecursiveNamedLink(_NestedLinkArticulationBase):
    """Fix verification: `**` can appear between fixed components to select a
    specific link by name regardless of nesting depth — e.g. every `link_1`
    under each Robot even though link_1 lives at a different depth per
    robot layout.
    """

    def on_start(self, sim):
        rb_view = sim.create_rigid_body_view("/envs/*/Robot/**/link_1")

        # Each robot has exactly one link_1.
        self.check_rigid_body_view(rb_view, self.num_envs)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestArticulationViewRecursiveRootLink(_NestedLinkArticulationBase):
    """Fix verification: `**` works for articulation-view patterns too. This
    locates every articulation whose root link happens to be named
    `base_link`, regardless of how deeply it sits below the env root.
    """

    def on_start(self, sim):
        arti_view = sim.create_articulation_view("/envs/*/**/base_link")

        self.check_articulation_view(
            arti_view,
            expected_count=self.num_envs,
            expected_max_links=self.LINKS_PER_ROBOT,
            expected_max_dofs=self.NUM_CHILD_LINKS,
            require_homogeneous=True,
        )
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewWildcardUnderRobot(_NestedLinkArticulationBase):
    """Bare `*` is strict: `/envs/*/Robot/*` matches only direct children of
    Robot, regardless of the recursive-leaf setting. That's `base_link` per
    env in this nested layout. Callers who want every rigid body under
    Robot/ must use `/envs/*/Robot/**` instead.
    """

    def on_start(self, sim):
        rb_view = sim.create_rigid_body_view("/envs/*/Robot/*")

        # Only base_link is a direct child of Robot/ — one per env.
        self.check_rigid_body_view(rb_view, self.num_envs)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewNestedAlternation(_NestedLinkArticulationBase):
    """Documents an out-of-scope limitation of pattern matching in the
    tensor API (NVBugs 6078079). The companion test method carries
    `@unittest.expectedFailure` because the scenario here *cannot* be
    satisfied by the current matching model, by design.

    --------------------------------------------------------------------
    The pattern
    --------------------------------------------------------------------
        /envs/*/Robot/(base_link|base_link/link_0)

    The user's intent is "match both `base_link` and its nested
    `base_link/link_0`." With the paren-aware tokenizer added in
    BaseSimulationView.cpp, the leaf token is preserved whole instead of
    being split into garbage — that fixes one half of the legacy bug.

    --------------------------------------------------------------------
    Why it still can't match
    --------------------------------------------------------------------
    The regex produced for the leaf token is

        ^(base_link|base_link/link_0)$

    and it is applied by `TfPatternMatcher` against a single *prim name*
    (never against a full path). USD prim names contain no `/`, so the
    second alternative has nothing to match against. Result: only
    `base_link` is ever found, one per env — so N_envs rigid bodies
    instead of 2 * N_envs.

    This is not a tokenizer bug; it is a direct consequence of how
    `findMatchingChildren` / `collectMatchingDescendants` feed prim
    names (not paths) into the matcher. Fixing it would require a new
    preprocessing stage that brace-expands `/`-bearing alternatives into
    multiple whole-path patterns (tracked as a possible follow-up; see
    the design notes alongside this test).

    --------------------------------------------------------------------
    Supported alternatives (prefer these)
    --------------------------------------------------------------------
    - Names-only alternation (no `/` inside the group):
          /envs/*/Robot/(base_link|link_0)
      With recursive-leaf enabled (default) the leaf is searched
      subtree-wide, so both prim names are found regardless of depth.

    - Explicit `**` anchors, one per alternative:
          ["/envs/*/Robot/base_link",
           "/envs/*/Robot/**/link_0"]

    - A list of fully-qualified per-body paths.

    --------------------------------------------------------------------
    If this test flips to "unexpected pass"
    --------------------------------------------------------------------
    That means someone extended the tokenizer or added pattern
    preprocessing that handles `/`-in-group alternation. When that
    happens, remove the `@unittest.expectedFailure` decorator on
    `test_rigid_body_view_nested_alternation_cpu` and keep this test as
    a passing regression guard.
    """

    def on_start(self, sim):
        rb_view = sim.create_rigid_body_view(
            "/envs/*/Robot/(base_link|base_link/link_0)"
        )

        expected_count = self.num_envs * 2
        self.test_case.assertEqual(
            rb_view.count, expected_count,
            f"Expected {expected_count} rigid bodies from nested alternation, "
            f"got {rb_view.count}. This case is expected to fail — glob "
            "matching runs on single prim names which cannot contain '/'. "
            "See the class docstring for supported alternatives."
        )
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


# ---------------------------------------------------------------------------
# Scenarios: standalone rigid bodies nested under intermediate Xforms
# ---------------------------------------------------------------------------

class _NestedStandaloneBase(GridTestBase):
    """Grid scenario: each env has a cluster of standalone rigid bodies at
    increasing USD depths — body_0 at depth +1, body_1 at depth +2, etc.
    No articulation involved; this isolates the pattern-matching issue
    from any PhysX articulation-root traversal.
    """

    NUM_LEVELS = 3  # body_0, body_1, body_2 at depths +1, +2, +3

    def __init__(self, test_case, device_params, num_envs=4):
        grid_params = GridParams(num_envs, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        cluster_path = self.env_template_path.AppendChild("cluster")
        transform = Transform(Gf.Vec3f(0.0, 0.0, 1.0))
        self._paths = _build_nested_standalone_rigid_bodies(
            self, cluster_path, transform, num_levels=self.NUM_LEVELS
        )


class TestRigidBodyViewStandaloneExplicitPaths(_NestedStandaloneBase):
    """Baseline workaround: a list of explicit per-body paths matches all
    nested standalone rigid bodies regardless of depth.
    """

    def on_start(self, sim):
        paths = []
        for i in range(self.num_envs):
            chain = f"/envs/env{i}/cluster"
            for j in range(self.NUM_LEVELS):
                # body_j sits inside sub_j (if j > 0), else directly under cluster.
                if j > 0:
                    chain = f"{chain}/sub_{j}"
                paths.append(f"{chain}/body_{j}")

        rb_view = sim.create_rigid_body_view(paths)
        self.check_rigid_body_view(rb_view, self.num_envs * self.NUM_LEVELS)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewStandaloneRecursiveUnderCluster(_NestedStandaloneBase):
    """Fix verification (bug 6078079) — standalone rigid-body variant. Using
    `/envs/*/cluster/**` collects every rigid body in the cluster subtree
    regardless of depth.
    """

    def on_start(self, sim):
        rb_view = sim.create_rigid_body_view("/envs/*/cluster/**")

        expected_count = self.num_envs * self.NUM_LEVELS
        self.check_rigid_body_view(rb_view, expected_count)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewStandaloneRecursiveNamedBody(_NestedStandaloneBase):
    """Fix verification: `**` between fixed components picks a specific
    body by name at any depth — here `body_2` only ever exists at depth +3.
    """

    def on_start(self, sim):
        rb_view = sim.create_rigid_body_view("/envs/*/cluster/**/body_2")

        self.check_rigid_body_view(rb_view, self.num_envs)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewStandaloneWildcardUnderCluster(_NestedStandaloneBase):
    """Bare `*` is strict (standalone variant): `/envs/*/cluster/*` matches
    only the direct children of cluster — body_0 and sub_1. sub_1 is an
    Xform, not a rigid body, so only body_0 (one per env) is returned.
    """

    def on_start(self, sim):
        rb_view = sim.create_rigid_body_view("/envs/*/cluster/*")

        self.check_rigid_body_view(rb_view, self.num_envs)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


# ---------------------------------------------------------------------------
# Scenarios: repeated leaf name at two depths (behavioural-change pins)
# ---------------------------------------------------------------------------

class _RepeatedLeafNameBase(GridTestBase):
    """Grid scenario: each env has a Robot whose hierarchy reuses the leaf
    name ``base_link`` at two different depths — a distinct rigid body
    at each occurrence. Exists to pin down what
    ``/envs/*/Robot/base_link`` matches before vs. after recursive-leaf
    matching becomes the default.
    """

    def __init__(self, test_case, device_params, num_envs=4):
        grid_params = GridParams(num_envs, 2.0)
        sim_params = SimParams()
        super().__init__(test_case, grid_params, sim_params, device_params)

        robot_path = self.env_template_path.AppendChild("Robot")
        transform = Transform(Gf.Vec3f(0.0, 0.0, 1.0))
        self._paths = _build_repeated_leaf_name_articulation(self, robot_path, transform)


class TestRigidBodyViewRepeatedLeafNameDefault(_RepeatedLeafNameBase):
    """With recursive-leaf matching enabled (default), a pre-existing
    pattern like ``/envs/*/Robot/base_link`` now matches BOTH the outer
    ``base_link`` (depth +1) and the inner ``base_link`` under ``tool/``
    (depth +3), because the named leaf is searched subtree-wide.

    This test pins down that behaviour so the compat implications of the
    MR are explicit: callers who wrote this pattern expecting a single
    direct-child match will see the view size grow.
    """

    def on_start(self, sim):
        rb_view = sim.create_rigid_body_view("/envs/*/Robot/base_link")

        # Two bodies per env: outer base_link and tool/base_link.
        expected_count = self.num_envs * 2
        self.check_rigid_body_view(rb_view, expected_count)

        expected_paths = set()
        for i in range(self.num_envs):
            expected_paths.add(f"/envs/env{i}/Robot/base_link")
            expected_paths.add(f"/envs/env{i}/Robot/base_link/tool/base_link")
        self.test_case.assertEqual(set(rb_view.prim_paths), expected_paths)

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


class TestRigidBodyViewRepeatedLeafNameStrict(_RepeatedLeafNameBase):
    """With ``/physics/tensors/recursiveLeafPatternMatch`` set to
    ``false``, the same pattern ``/envs/*/Robot/base_link`` falls back
    to strict per-level matching — only the direct-child outer
    ``base_link`` is returned, matching the pre-MR behaviour.

    Serves two purposes: (1) documents the escape hatch for callers who
    need the legacy semantics, and (2) regression-guards the setting
    now that it's part of the public contract.
    """

    SETTING = "/physics/tensors/recursiveLeafPatternMatch"

    def on_start(self, sim):
        settings = carb.settings.acquire_settings_interface()
        saved = settings.get_as_bool(self.SETTING)
        settings.set(self.SETTING, False)
        try:
            rb_view = sim.create_rigid_body_view("/envs/*/Robot/base_link")

            # Only the direct-child outer base_link per env.
            self.check_rigid_body_view(rb_view, self.num_envs)

            expected_paths = {
                f"/envs/env{i}/Robot/base_link" for i in range(self.num_envs)
            }
            self.test_case.assertEqual(set(rb_view.prim_paths), expected_paths)
        finally:
            settings.set(self.SETTING, saved)

        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass


# ---------------------------------------------------------------------------
# Test case
# ---------------------------------------------------------------------------

class PhysxTensorsNestedRigidBodiesTests(unittest.TestCase):
    """Repro for NVBugs 6078079 — nested rigid-body pattern matching."""

    def _run_test(self, scenario):
        sync_params = SyncParams(sync_usd=True, sync_fabric=False, transforms_only=False)
        runner = RunnerInMemory(scenario, _FRONTEND, sync_params)
        exc_info = None
        try:
            runner.start(_WARM_START)
            runner.simulate()
        except Exception:
            exc_info = sys.exc_info()
        finally:
            try:
                runner.stop()
            except Exception:
                if exc_info is None:
                    exc_info = sys.exc_info()
        if exc_info is not None:
            raise exc_info[1].with_traceback(exc_info[2])

    # ---- articulation view: baselines expected to pass today ----

    def test_articulation_view_nested_links_cpu(self):
        self._run_test(TestArticulationViewNestedLinks(self, DeviceParams(False, False)))

    def test_articulation_view_nested_links_gpu(self):
        self._run_test(TestArticulationViewNestedLinks(self, DeviceParams(True, True)))

    def test_articulation_view_at_nested_root_link_cpu(self):
        self._run_test(TestArticulationViewAtNestedRootLink(self, DeviceParams(False, False)))

    def test_articulation_view_at_nested_root_link_gpu(self):
        self._run_test(TestArticulationViewAtNestedRootLink(self, DeviceParams(True, True)))

    # ---- rigid body view: list-of-paths workaround (should pass today) ----

    def test_rigid_body_view_explicit_nested_paths_cpu(self):
        self._run_test(TestRigidBodyViewExplicitNestedPaths(self, DeviceParams(False, False)))

    def test_rigid_body_view_standalone_explicit_paths_cpu(self):
        self._run_test(TestRigidBodyViewStandaloneExplicitPaths(self, DeviceParams(False, False)))

    # ---- rigid body view: '**' recursive descent (fix verification) ----
    # These confirm the new '**' token discovers rigid bodies at any depth.

    def test_rigid_body_view_recursive_under_robot_cpu(self):
        self._run_test(TestRigidBodyViewRecursiveUnderRobot(self, DeviceParams(False, False)))

    def test_rigid_body_view_recursive_under_robot_gpu(self):
        self._run_test(TestRigidBodyViewRecursiveUnderRobot(self, DeviceParams(True, True)))

    def test_rigid_body_view_recursive_named_link_cpu(self):
        self._run_test(TestRigidBodyViewRecursiveNamedLink(self, DeviceParams(False, False)))

    def test_rigid_body_view_standalone_recursive_under_cluster_cpu(self):
        self._run_test(TestRigidBodyViewStandaloneRecursiveUnderCluster(self, DeviceParams(False, False)))

    def test_rigid_body_view_standalone_recursive_named_body_cpu(self):
        self._run_test(TestRigidBodyViewStandaloneRecursiveNamedBody(self, DeviceParams(False, False)))

    # ---- articulation view: '**' recursive descent (fix verification) ----

    def test_articulation_view_recursive_root_link_cpu(self):
        self._run_test(TestArticulationViewRecursiveRootLink(self, DeviceParams(False, False)))

    def test_articulation_view_recursive_root_link_gpu(self):
        self._run_test(TestArticulationViewRecursiveRootLink(self, DeviceParams(True, True)))

    # ---- bare '*' is strict regardless of setting ----
    # A bare `*` is always treated as "direct children only". Callers who
    # want nested matching must use an explicit `**` — this keeps the
    # default semantics predictable and stops existing tests that rely on
    # per-level matching from silently breaking.

    def test_rigid_body_view_wildcard_under_robot_cpu(self):
        self._run_test(TestRigidBodyViewWildcardUnderRobot(self, DeviceParams(False, False)))

    def test_rigid_body_view_standalone_wildcard_under_cluster_cpu(self):
        self._run_test(TestRigidBodyViewStandaloneWildcardUnderCluster(self, DeviceParams(False, False)))

    # ---- repeated leaf name at two depths: behavioural-change pins ----
    # These are the silent-behaviour-change cases: a scene where the same
    # prim name is intentionally reused at different depths as two
    # distinct rigid bodies. The default test pins what recursive-leaf
    # matching does to a pre-existing `/Robot/base_link` pattern (it
    # grows the view); the strict variant pins that the public setting
    # `/physics/tensors/recursiveLeafPatternMatch=false` restores the
    # legacy direct-children-only semantics.

    def test_rigid_body_view_repeated_leaf_name_default_cpu(self):
        self._run_test(TestRigidBodyViewRepeatedLeafNameDefault(self, DeviceParams(False, False)))

    def test_rigid_body_view_repeated_leaf_name_strict_cpu(self):
        self._run_test(TestRigidBodyViewRepeatedLeafNameStrict(self, DeviceParams(False, False)))

    # ---- out-of-scope limitation: xfail, documents what cannot match ----
    #
    # `/` inside a glob alternation group such as
    #     /envs/*/Robot/(base_link|base_link/link_0)
    # cannot be matched by the current pattern model — `TfPatternMatcher`
    # consumes a single prim name (never a path), and prim names have no
    # `/` to match against. The companion scenario class documents this
    # in detail and lists supported alternatives.
    #
    # Marked `@unittest.expectedFailure` so CI stays green; if someone
    # later adds brace-expansion preprocessing for `/`-bearing groups the
    # assertion will succeed and pytest will flag it as unexpected-pass
    # — that's the cue to drop the decorator.

    @unittest.expectedFailure
    def test_rigid_body_view_nested_alternation_cpu(self):
        self._run_test(TestRigidBodyViewNestedAlternation(self, DeviceParams(False, False)))
