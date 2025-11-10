# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import sys
from pxr import Usd, UsdGeom, Gf, Vt, UsdUtils
import omni.physxdemos as demo


def main():
    if not hasattr(sys, 'argv'):
        sys.argv  = ['']
    if len(sys.argv) == 6:
        stageId = int(sys.argv[1])
        triggerPath = sys.argv[2]
        otherPath = sys.argv[3]
        eventName = sys.argv[4]
        scriptFileName = sys.argv[5]
        print("Executed script: " + scriptFileName)
        print("Event name: " + eventName)
        cache = UsdUtils.StageCache.Get()
        stage = cache.Find(Usd.StageCache.Id.FromLongInt(stageId))
        if stage:
            otherPrim = stage.GetPrimAtPath(otherPath)
            usdGeom = UsdGeom.Mesh.Get(stage, otherPrim.GetPath())
            if eventName == "LeaveEvent":
                color = Vt.Vec3fArray([demo.get_primary_color()])
                usdGeom.GetDisplayColorAttr().Set(color)
            else:
                color = Vt.Vec3fArray([demo.get_hit_color()])
                usdGeom.GetDisplayColorAttr().Set(color)
    pass


main()
