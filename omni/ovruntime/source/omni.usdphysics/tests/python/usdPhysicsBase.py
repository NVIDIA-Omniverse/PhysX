# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""
Base test infrastructure for ovruntime usdphysics tests.

Ported from omni.usdphysics.tests for standalone (non-Kit) use.
Uses unittest.TestCase and the _usdphysics bindings module directly.
"""

import _usdphysics_setup  # noqa: F401 - loads Carbonite + usdphysics plugin + PhysX schemas

import os
import math
import unittest
from pathlib import Path
from pxr import Usd, UsdUtils
import carb
import _usdphysics


# --- Test data directory ---
_DATA_DIR = Path(__file__).resolve().parent.parent / "data"


# --- Utility functions (ported from omni.usdphysics.tests utils) ---

def open_usd(filename):
    """Open a .usda test file and insert it into the UsdUtils stage cache.

    Returns:
        The stage cache ID as a long int.
    """
    filepath = str(_DATA_DIR / (filename + ".usda"))
    stage = Usd.Stage.Open(filepath)
    cache = UsdUtils.StageCache.Get()
    cache.Insert(stage)
    return cache.GetId(stage).ToLongInt()


def compare_values(item0, item1):
    """Compare two values with tolerance for floats and carb types."""
    epsilon = 0.0001
    if item0 is None and item1 is None:
        return True
    if item0 is not None and item1 is None:
        return False
    if item0 is None and item1 is not None:
        return False
    if type(item0) is not type(item1):
        return False
    if isinstance(item0, str):
        return True if item0 == item1 else False
    elif isinstance(item0, int):
        return True if item0 == item1 else False
    elif isinstance(item0, bool):
        return True if item0 == item1 else False
    elif isinstance(item0, float):
        return True if math.fabs(item0 - item1) < max(math.fabs(item0) * epsilon, epsilon) else False
    elif isinstance(item0, carb.Float3):
        if math.fabs(item0.x - item1.x) > max(math.fabs(item0.x) * epsilon, epsilon):
            return False
        if math.fabs(item0.y - item1.y) > max(math.fabs(item0.y) * epsilon, epsilon):
            return False
        if math.fabs(item0.z - item1.z) > max(math.fabs(item0.z) * epsilon, epsilon):
            return False
        return True
    elif isinstance(item0, carb.Float4) or isinstance(item0, carb.Double4):
        if math.fabs(item0.x - item1.x) > epsilon:
            return False
        if math.fabs(item0.y - item1.y) > epsilon:
            return False
        if math.fabs(item0.z - item1.z) > epsilon:
            return False
        if math.fabs(item0.w - item1.w) > epsilon:
            return False
        return True
    elif isinstance(item0, list):
        if len(item0) != len(item1):
            return False
        for i0, i1 in zip(item0, item1):
            if not compare_values(i0, i1):
                return False
        return True

    print("Unchecked type: " + str(type(item0)))
    return True


# --- Base test class ---

class UsdPhysicsBaseTest(unittest.TestCase):

    def report_callback(self, dict):
        prim_dict = self.expected_prims.get(dict["prim_path"] + "/" + dict["object_type"])
        if prim_dict is not None:
            for key, val in prim_dict.items():
                report_val = dict.get(key)
                if val is not None or report_val is not None:
                    if report_val is None:
                        print(f'Reported value is None for key "{key}" and prim "{dict["prim_path"]}"')
                        self.assertTrue(False)
                    elif val is None:
                        print(f'Expected value is None for key "{key}" and prim "{dict["prim_path"]}"')
                        self.assertTrue(False)
                    else:
                        values_match = compare_values(report_val, val)
                        if not values_match:
                            print(f'Mismatch between reported "{report_val}" and expected "{val}" values for key "{key}" and prim "{dict["prim_path"]}"')
                            self.assertTrue(False)
            prim_dict["parsed"] = True

    def parse(self, fileName):
        self.stage_id = open_usd(fileName)

        _usdphysics.report_object_desc_callback(self.report_callback)
        _usdphysics.load_from_stage(self.stage_id)
        _usdphysics.report_object_desc_callback(None)

        for dictKey, dictVal in self.expected_prims.items():
            parsed_correctly = False
            for key, val in dictVal.items():
                if key == "parsed" and val:
                    parsed_correctly = True
            if not parsed_correctly:
                print(f"parsing check failed for expected_prims key: {dictKey}")
            self.assertTrue(parsed_correctly)

    def tearDown(self):
        if hasattr(self, 'stage_id'):
            cache = UsdUtils.StageCache.Get()
            stage = cache.Find(Usd.StageCache.Id.FromLongInt(self.stage_id))
            cache.Erase(stage)
