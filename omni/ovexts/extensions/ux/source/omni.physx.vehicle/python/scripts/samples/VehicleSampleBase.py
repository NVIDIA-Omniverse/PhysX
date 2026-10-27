# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from pxr import UsdLux, Gf
import omni.physxdemos as demo

class VehicleSampleBase(demo.Base):

    def create(self, stage):
        rootPath = str(stage.GetDefaultPrim().GetPath())

        domeLightPath = rootPath + "/DomeLight"
        domeLight = UsdLux.DomeLight.Define(stage, domeLightPath)
        domeLight.CreateIntensityAttr(400)
