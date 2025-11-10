# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physxdemos as demo
from omni.physx.scripts.assets_paths import AssetFolders

class AnalogDigitalClockDemo(demo.AsyncDemoBase):
    title = "Analog Digital Clock"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Mechanical seven segment display using SDF collision"
    description = (
        "Demo showcasing the PhysX signed-distance-field (SDF) collision feature enabling "
        "dynamic objects with arbitrary collision shapes."
    )

    kit_settings = {
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }

    def __init__(self):
        super().__init__(enable_fabric=True)
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.ANALOG_DIGITAL_CLOCK, "AnalogDigitalClock.usd")

    def on_startup(self):
        pass

    def create(self, stage):
        pass

    def on_shutdown(self):
        pass
