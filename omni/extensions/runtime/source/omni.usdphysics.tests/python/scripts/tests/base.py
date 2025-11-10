# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.kit.test
from pxr import UsdUtils, Usd
from omni.usdphysicstests.scripts.tests import utils
from omni.usdphysicstests import get_usd_physics_test_interface


class UsdPhysicsBaseTest(omni.kit.test.AsyncTestCase):

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
                        values_match = utils.compare_values(report_val, val)
                        if not values_match:
                            print(f'Mismatch between reported "{report_val}" and expected "{val}" values for key "{key}" and prim "{dict["prim_path"]}"')
                            self.assertTrue(False)
            prim_dict["parsed"] = True

    async def parse(self, fileName):
        self.stage_id = utils.open_usd(fileName)

        testI = get_usd_physics_test_interface()

        testI.report_object_desc_callback(self.report_callback)
        testI.attach_stage(self.stage_id)
        testI.deattach_stage()
        testI.report_object_desc_callback(None)

        for dictKey, dictVal in self.expected_prims.items():
            parsed_correctly = False
            for key, val in dictVal.items():
                if key == "parsed" and val:
                    parsed_correctly = True
            if not parsed_correctly:
                print(f"parsing check failed for expected_prims key: {dictKey}")
            self.assertTrue(parsed_correctly)

    async def tearDown(self):
        cache = UsdUtils.StageCache.Get()
        stage = cache.Find(Usd.StageCache.Id.FromLongInt(self.stage_id))
        cache.Erase(stage)
