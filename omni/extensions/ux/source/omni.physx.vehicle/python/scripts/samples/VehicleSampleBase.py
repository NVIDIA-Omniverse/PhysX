# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import UsdLux, Gf
import omni.physxdemos as demo

class VehicleSampleBase(demo.Base):

    def create(self, stage):
        rootPath = str(stage.GetDefaultPrim().GetPath())

        domeLightPath = rootPath + "/DomeLight"
        domeLight = UsdLux.DomeLight.Define(stage, domeLightPath)
        domeLight.CreateIntensityAttr(400)
