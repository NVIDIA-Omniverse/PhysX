# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import asyncio
import carb
import os
import math
from pxr import Gf, Sdf, UsdGeom, Usd, UsdUtils


# # # general utils # # #


def open_usd(filename):
    data_path = "../../../../../../data/UsdPhysics"
    schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
    schema_folder = schema_folder.replace("\\", "/")
    
    stage = Usd.Stage.Open(schema_folder + "/" + filename + ".usda")
    cache = UsdUtils.StageCache.Get()
    cache.Insert(stage)
    return cache.GetId(stage).ToLongInt()

def compare_values(item0, item1):
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
        if math.fabs(item0.y - item1.y) > max(math.fabs(item0.y) *epsilon, epsilon):
            return False
        if math.fabs(item0.z - item1.z) > max(math.fabs(item0.z) *epsilon, epsilon):
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
