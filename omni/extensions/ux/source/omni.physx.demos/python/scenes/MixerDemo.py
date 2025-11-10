# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physxdemos as demo
import omni.timeline
from omni.physx.scripts.assets_paths import AssetFolders

class MixerDemo(demo.Base):
    title = "Mixer"
    category = demo.Categories.COMPLEX_SHOWCASES
    short_description = "Update frequency demo"
    description = "Update frequency demo"
    tags = ["Timecodes", "Timeline"]

    params = {
        "tcps": demo.IntParam(24, 1, 120, 1, name="Timecodes Per Second"),
        "tpf": demo.IntParam(3, 1, 60, 1, name="Ticks Per Frame"),
        "fps": demo.IntParam(60, 1, 120, 1, name="Target Framerate"),
        "fm": demo.BoolParam(False, name="Fast Mode"),
    }

    def __init__(self):
        super().__init__()
        self.demo_base_usd_url = demo.get_demo_asset_path(AssetFolders.MIXER, "Mixer.usd")
        self.old_tpf = None
        self.old_fps = None
        self.old_fm = None

    def create(self, stage, tcps, tpf, fps, fm):
        stage.SetTimeCodesPerSecond(tcps)

        timeline = omni.timeline.get_timeline_interface()
        self.old_tpf = timeline.get_ticks_per_frame()
        self.old_fps = timeline.get_target_framerate()
        self.old_fm = timeline.get_fast_mode()

        timeline.set_ticks_per_frame(tpf)
        timeline.set_target_framerate(fps)
        timeline.set_fast_mode(fm)

    def on_shutdown(self):
        if self.old_tpf:
            omni.timeline.get_timeline_interface().set_ticks_per_frame(self.old_tpf)

        if self.old_fps:
            omni.timeline.get_timeline_interface().set_target_framerate(self.old_fps)

        if self.old_fm:
            omni.timeline.get_timeline_interface().set_fast_mode(self.old_fm)
