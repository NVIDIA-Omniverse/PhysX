# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Edge-case coverage for the tensor-plugin path pattern matcher.

import sys
import unittest

import numpy as np  # noqa: F401
import warp as wp  # noqa: F401

import _tensors_setup  # noqa: F401

import carb.settings
import omni.physics.tensors

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema

from scenario import (
    GridTestBase, GridParams, SimParams, DeviceParams,
    SyncParams, Transform, RunnerInMemory,
)

_WARM_START = True
_FRONTEND = "warp"
_SETTING = "/physics/tensors/recursiveLeafPatternMatch"


def _xform(stage, path):
    UsdGeom.Xform.Define(stage, Sdf.Path(path))


class _EdgeCaseBase(GridTestBase):
    __test__ = False

    def __init__(self, test_case, device_params, num_envs=4):
        super().__init__(test_case, GridParams(num_envs, 2.0), SimParams(), device_params)
        self.build_scene()

    def _box(self, path, x=0.0):
        return self.create_rigid_box(Sdf.Path(path), Transform((x, 0.0, 2.0)), Gf.Vec3f(0.1, 0.1, 0.1))

    def build_scene(self):
        raise NotImplementedError

    def check(self, sim):
        raise NotImplementedError

    def on_start(self, sim):
        self.check(sim)
        self.finish()

    def on_physics_step(self, sim, stepno, dt):
        pass

    def _with_recursive_leaf(self, enabled, fn):
        settings = carb.settings.acquire_settings_interface()
        saved = settings.get_as_bool(_SETTING)
        settings.set(_SETTING, enabled)
        try:
            return fn()
        finally:
            settings.set(_SETTING, saved)


# leaf name repeated NESTED INSIDE its own match:
#   {env}/robot/Disc001
#   {env}/robot/Disc001/robot/Disc001
class _PivotNestedScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        _xform(self.stage, et.AppendChild("robot"))
        self._box(et.AppendChild("robot").AppendChild("Disc001"), x=0.4)
        inner = et.AppendChild("robot").AppendChild("Disc001").AppendChild("robot")
        _xform(self.stage, inner)
        self._box(inner.AppendChild("Disc001"), x=0.8)
        cube = self.create_rigid_box(et.AppendChild("cube"), Transform((0.0, 0.0, 2.0)), Gf.Vec3f(0.2, 0.2, 0.2))
        cr = PhysxSchema.PhysxContactReportAPI.Apply(cube)
        cr.CreateThresholdAttr().Set(0.0)


class PivotNestedDefault(_PivotNestedScene):
    """/envs/*/robot/Disc001 -> one Disc001 per env."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/robot/Disc001")
        self.check_rigid_body_view(rb, self.num_envs)


class PivotNestedExplicitRecursive(_PivotNestedScene):
    """/envs/*/robot/**/Disc001 -> explicit any-depth -> 2N on every version."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/robot/**/Disc001")
        self.check_rigid_body_view(rb, self.num_envs * 2)


class PivotNestedContactFilter(_PivotNestedScene):
    """sensor /envs/*/cube, filter /envs/*/robot/Disc001.
    Filter must resolve to 1 per env -> valid view."""
    def check(self, sim):
        view = sim.create_rigid_contact_view("/envs/*/cube", filter_patterns=["/envs/*/robot/Disc001"])
        self.test_case.assertIsNotNone(
            view, "contact view should be valid: filter must match exactly 1 per env")
        self.check_rigid_contact_view(view, self.num_envs, 1)


# glob leaf, two discs same env, no nesting
class _GlobLeafScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        _xform(self.stage, et.AppendChild("robot"))
        self._box(et.AppendChild("robot").AppendChild("Disc001"), x=0.4)
        self._box(et.AppendChild("robot").AppendChild("Disc002"), x=0.8)


class GlobLeaf(_GlobLeafScene):
    """/envs/*/robot/Disc* -> both discs per env -> 2N."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/robot/Disc*")
        self.check_rigid_body_view(rb, self.num_envs * 2)


# two wildcards before a literal leaf
class _MultiWildcardScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        for grp, x in (("grpA", 0.0), ("grpB", 0.6)):
            _xform(self.stage, et.AppendChild(grp))
            _xform(self.stage, et.AppendChild(grp).AppendChild("robot"))
            self._box(et.AppendChild(grp).AppendChild("robot").AppendChild("link"), x=x)


class MultiWildcard(_MultiWildcardScene):
    """/envs/*/grp*/robot/link -> grpA + grpB per env -> 2N."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/grp*/robot/link")
        self.check_rigid_body_view(rb, self.num_envs * 2)


# leaf '|' alternation.
# Nested-links: base_link (depth1) + link_0 nested UNDER base_link (depth2), distinct names.
class _NestedLinksScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        _xform(self.stage, et.AppendChild("Robot"))
        base = et.AppendChild("Robot").AppendChild("base_link")
        self._box(base, x=0.0)
        self._box(base.AppendChild("link_0"), x=0.4)


# Repeated-leaf: base_link nested UNDER base_link (same name, two depths).
class _RepeatedLeafScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        _xform(self.stage, et.AppendChild("Robot"))
        outer = et.AppendChild("Robot").AppendChild("base_link")
        self._box(outer, x=0.0)
        tool = outer.AppendChild("tool")
        _xform(self.stage, tool)
        self._box(tool.AppendChild("base_link"), x=0.4)


class AlternationDifferentDepths(_NestedLinksScene):
    """/envs/*/Robot/base_link|link_0 -> base_link (depth1) + nested link_0 (depth2)
    -> 2N. Distinct alternatives at different depths must both be found."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/Robot/base_link|link_0")
        self.check_rigid_body_view(rb, self.num_envs * 2)


class AlternationSameNameNested(_RepeatedLeafScene):
    """/envs/*/Robot/base_link|zzz -> only the outer base_link per env -> N. The
    alternation descent must suppress the same name nested inside its own match."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/Robot/base_link|zzz")
        self.check_rigid_body_view(rb, self.num_envs)


class AlternationSameNameNestedPivot(_PivotNestedScene):
    """/envs/*/robot/Disc001|zzz on the pivot-nested scene -> N."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/robot/Disc001|zzz")
        self.check_rigid_body_view(rb, self.num_envs)


# Ordering guards -- views are position-indexed, so the SEQUENCE (not just the set)
# of prim paths is part of the contract. Assert env-major, pre-order within env.
class OrderingAlternation(_NestedLinksScene):
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/Robot/base_link|link_0")
        expected = []
        for i in range(self.num_envs):
            e = f"/envs/env{i}/Robot/base_link"
            expected += [e, e + "/link_0"]
        self.test_case.assertEqual(list(rb.prim_paths), expected)


class OrderingGlob(_GlobLeafScene):
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/robot/Disc*")
        expected = []
        for i in range(self.num_envs):
            e = f"/envs/env{i}/robot/"
            expected += [e + "Disc001", e + "Disc002"]
        self.test_case.assertEqual(list(rb.prim_paths), expected)


class OrderingMultiWildcard(_MultiWildcardScene):
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/grp*/robot/link")
        expected = []
        for i in range(self.num_envs):
            e = f"/envs/env{i}/"
            expected += [e + "grpA/robot/link", e + "grpB/robot/link"]
        self.test_case.assertEqual(list(rb.prim_paths), expected)


# a distinct link nested BELOW the pattern level (URDF converter shape).
class _NestedChainScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        _xform(self.stage, et.AppendChild("Robot"))
        b = et.AppendChild("Robot").AppendChild("base_link")
        self._box(b)
        self._box(b.AppendChild("link_0"), x=0.4)


class NestedDistinctLink(_NestedChainScene):
    """/envs/*/Robot/link_0 -> the leaf is found at any depth under Robot -> N."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/Robot/link_0")
        self.check_rigid_body_view(rb, self.num_envs)


# pivot buried under a Group (pre-leaf tokens are strict).
class _GroupPivotScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        _xform(self.stage, et.AppendChild("Robot"))
        self._box(et.AppendChild("Robot").AppendChild("base_link"))
        _xform(self.stage, et.AppendChild("Group"))
        gr = et.AppendChild("Group").AppendChild("Robot")
        _xform(self.stage, gr)
        self._box(gr.AppendChild("base_link"), x=0.6)


class GroupBuriedPivotStrict(_GroupPivotScene):
    """/envs/*/Robot/base_link -> only the direct-child Robot (strict pre-leaf) -> N.
    The buried Group/Robot is NOT reached by a plain pattern."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/Robot/base_link")
        self.check_rigid_body_view(rb, self.num_envs)


class GroupBuriedPivotExplicitRecursive(_GroupPivotScene):
    """/envs/*/**/Robot/base_link -> explicit ** reaches the buried Group/Robot too -> 2N."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/**/Robot/base_link")
        self.check_rigid_body_view(rb, self.num_envs * 2)


def _rb_count(sim, pattern):
    # A no-match returns None (or a view whose .count raises); both mean 0.
    v = sim.create_rigid_body_view(pattern)
    try:
        return v.count if v is not None else 0
    except AttributeError:
        return 0


# Deep nested chain: Robot/base_link(d1)/link_0(d2)/link_1(d3)/link_2(d4).
class _NestedLinkChainScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        _xform(self.stage, et.AppendChild("Robot"))
        p = et.AppendChild("Robot").AppendChild("base_link")
        self._box(p)
        for i in range(3):
            p = p.AppendChild("link_" + str(i))
            self._box(p, x=0.3 * (i + 1))


class MatrixNestedLink(_NestedLinkChainScene):
    def check(self, sim):
        n = self.num_envs
        eq = self.test_case.assertEqual
        eq(_rb_count(sim, "/envs/*/Robot/base_link"), n)    # direct child
        eq(_rb_count(sim, "/envs/*/Robot/link_0"), n)       # nested d2
        eq(_rb_count(sim, "/envs/*/Robot/link_1"), n)       # nested d3
        eq(_rb_count(sim, "/envs/*/Robot/link_2"), n)       # nested d4
        eq(_rb_count(sim, "/envs/*/Robot/link_0|zzz"), n)   # same via alternation
        eq(_rb_count(sim, "/envs/*/Robot/**/link_1"), n)    # explicit **
        eq(_rb_count(sim, "/envs/*/Robot/**"), n * 4)       # ** -> all 4 bodies
        eq(_rb_count(sim, "/envs/*/Robot/*"), n)            # bare '*' strict -> base_link
        # Single-token, name-only pattern: no parent tokens and no exact prim at
        # that literal path, so the leaf is searched stage-wide -> one per env.
        eq(_rb_count(sim, "/base_link"), n)

        def strict():
            eq(_rb_count(sim, "/envs/*/Robot/base_link"), n)  # strict: direct child
            eq(_rb_count(sim, "/envs/*/Robot/link_0"), 0)     # strict: not a direct child
            eq(_rb_count(sim, "/base_link"), 0)               # strict: no root-level base_link
        self._with_recursive_leaf(False, strict)


class MatrixRepeatedLeaf(_RepeatedLeafScene):
    def check(self, sim):
        n = self.num_envs
        eq = self.test_case.assertEqual
        eq(_rb_count(sim, "/envs/*/Robot/base_link"), n)         # shallowest-wins
        eq(_rb_count(sim, "/envs/*/Robot/base_link|zzz"), n)     # alternation same-name suppressed
        eq(_rb_count(sim, "/envs/*/Robot/**/base_link"), n * 2)  # explicit ** -> both
        self._with_recursive_leaf(False, lambda: eq(_rb_count(sim, "/envs/*/Robot/base_link"), n))


class MatrixGroupPivot(_GroupPivotScene):
    def check(self, sim):
        n = self.num_envs
        eq = self.test_case.assertEqual
        eq(_rb_count(sim, "/envs/*/Robot/base_link"), n)          # only the direct Robot
        eq(_rb_count(sim, "/envs/*/Group/Robot/base_link"), n)    # literal to the buried one
        eq(_rb_count(sim, "/envs/*/**/base_link"), n * 2)         # explicit ** -> both
        self._with_recursive_leaf(False, lambda: eq(_rb_count(sim, "/envs/*/Robot/base_link"), n))


class MatrixPivotNested(_PivotNestedScene):
    def check(self, sim):
        n = self.num_envs
        eq = self.test_case.assertEqual
        eq(_rb_count(sim, "/envs/*/robot/Disc001"), n)           # suppress self-nested
        eq(_rb_count(sim, "/envs/*/robot/Disc001|zzz"), n)       # alternation same-name suppressed
        eq(_rb_count(sim, "/envs/*/robot/**/Disc001"), n * 2)    # explicit ** -> both


class MatrixGlob(_GlobLeafScene):
    def check(self, sim):
        n = self.num_envs
        eq = self.test_case.assertEqual
        eq(_rb_count(sim, "/envs/*/robot/Disc001"), n)
        eq(_rb_count(sim, "/envs/*/robot/Disc*"), n * 2)          # glob -> both siblings
        eq(_rb_count(sim, "/envs/*/robot/Disc001|Disc002"), n * 2)  # alternation -> both


class MatrixMultiWildcard(_MultiWildcardScene):
    def check(self, sim):
        eq = self.test_case.assertEqual
        eq(_rb_count(sim, "/envs/*/grp*/robot/link"), self.num_envs * 2)


# Alternation must anchor each alternative: base_link|link_0 must NOT match the
# prefix/suffix decoys base_link_extra / prefix_link_0.
class _AlternationDecoyScene(_EdgeCaseBase):
    def build_scene(self):
        et = self.env_template_path
        _xform(self.stage, et.AppendChild("Robot"))
        base = et.AppendChild("Robot").AppendChild("base_link")
        self._box(base)
        self._box(base.AppendChild("link_0"), x=0.4)
        self._box(et.AppendChild("Robot").AppendChild("base_link_extra"), x=0.8)
        self._box(base.AppendChild("prefix_link_0"), x=1.2)


class AlternationAnchoring(_AlternationDecoyScene):
    """/envs/*/Robot/base_link|link_0 matches base_link and link_0 only, never the
    prefix/suffix decoys (a mis-anchored ^base_link|link_0$ would match them)."""
    def check(self, sim):
        rb = sim.create_rigid_body_view("/envs/*/Robot/base_link|link_0")
        self.check_rigid_body_view(rb, self.num_envs * 2)
        expected = set()
        for i in range(self.num_envs):
            expected.add(f"/envs/env{i}/Robot/base_link")
            expected.add(f"/envs/env{i}/Robot/base_link/link_0")
        self.test_case.assertEqual(set(rb.prim_paths), expected)


class ContactFilterExplicitRecursiveNoDup(_EdgeCaseBase):
    """A filter pattern with '**' must resolve to one prim per sensor, not once per
    ancestor root -- else the filter cardinality (found 2, expected 1) breaks."""
    def build_scene(self):
        et = self.env_template_path
        cube = self.create_rigid_box(et.AppendChild("cube"), Transform((0.0, 0.0, 2.0)), Gf.Vec3f(0.2, 0.2, 0.2))
        cr = PhysxSchema.PhysxContactReportAPI.Apply(cube)
        cr.CreateThresholdAttr().Set(0.0)
        _xform(self.stage, et.AppendChild("robot"))
        self._box(et.AppendChild("robot").AppendChild("Disc001"), x=0.4)

    def check(self, sim):
        view = sim.create_rigid_contact_view("/envs/*/cube", filter_patterns=["/envs/*/**/Disc001"])
        self.test_case.assertIsNotNone(view, "'**' filter must resolve to one prim per sensor")
        self.check_rigid_contact_view(view, self.num_envs, 1)


class MatcherEdgeCaseTests(unittest.TestCase):
    def _run(self, scenario):
        runner = RunnerInMemory(scenario, _FRONTEND,
                                SyncParams(sync_usd=True, sync_fabric=False, transforms_only=False))
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

    def test_pivot_nested_default_cpu(self):
        self._run(PivotNestedDefault(self, DeviceParams(False, False)))

    def test_pivot_nested_explicit_recursive_cpu(self):
        self._run(PivotNestedExplicitRecursive(self, DeviceParams(False, False)))

    def test_pivot_nested_contact_filter_cpu(self):
        self._run(PivotNestedContactFilter(self, DeviceParams(False, False)))

    def test_glob_leaf_cpu(self):
        self._run(GlobLeaf(self, DeviceParams(False, False)))

    def test_multi_wildcard_cpu(self):
        self._run(MultiWildcard(self, DeviceParams(False, False)))

    def test_different_depths_cpu(self):
        self._run(AlternationDifferentDepths(self, DeviceParams(False, False)))

    def test_same_name_cpu(self):
        self._run(AlternationSameNameNested(self, DeviceParams(False, False)))

    def test_same_name_pivot_cpu(self):
        self._run(AlternationSameNameNestedPivot(self, DeviceParams(False, False)))

    def test_matrix_nested_link_cpu(self):
        self._run(MatrixNestedLink(self, DeviceParams(False, False)))

    def test_matrix_repeated_leaf_cpu(self):
        self._run(MatrixRepeatedLeaf(self, DeviceParams(False, False)))

    def test_matrix_group_pivot_cpu(self):
        self._run(MatrixGroupPivot(self, DeviceParams(False, False)))

    def test_matrix_pivot_nested_cpu(self):
        self._run(MatrixPivotNested(self, DeviceParams(False, False)))

    def test_matrix_glob_cpu(self):
        self._run(MatrixGlob(self, DeviceParams(False, False)))

    def test_matrix_multi_wildcard_cpu(self):
        self._run(MatrixMultiWildcard(self, DeviceParams(False, False)))

    def test_alternation_anchoring_decoys_cpu(self):
        self._run(AlternationAnchoring(self, DeviceParams(False, False)))

    def test_contact_filter_explicit_recursive_no_dup_cpu(self):
        self._run(ContactFilterExplicitRecursiveNoDup(self, DeviceParams(False, False)))

    def test_nested_distinct_link_cpu(self):
        self._run(NestedDistinctLink(self, DeviceParams(False, False)))

    def test_buried_pivot_strict_cpu(self):
        self._run(GroupBuriedPivotStrict(self, DeviceParams(False, False)))

    def test_buried_pivot_explicit_cpu(self):
        self._run(GroupBuriedPivotExplicitRecursive(self, DeviceParams(False, False)))

    def test_ORD_alternation_cpu(self):
        self._run(OrderingAlternation(self, DeviceParams(False, False)))

    def test_ORD_glob_cpu(self):
        self._run(OrderingGlob(self, DeviceParams(False, False)))

    def test_ORD_multi_wildcard_cpu(self):
        self._run(OrderingMultiWildcard(self, DeviceParams(False, False)))


if __name__ == "__main__":
    unittest.main()
