# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physxdemos as demo
from .BasicSetup import _open_test_usd_file
import carb.settings

class OmniGraphSceneQueryRaycastDemo(demo.Base):
    title = "Raycast (OmniGraph)"
    category = demo.Categories.SCENE_QUERY
    short_description = "Demo using raycast OmniGraph nodes with ActionGraph to make laser rays respond to colliders."
    description = "A demonstration of how to use PhysX raycast OmniGraph nodes with ActionGraph to make laser rays " \
        "respond to colliders in the scene. The lengths of the laser rays and the lights at the intersections have " \
        "position and length adjusted by raycast scene queries. "  \
        "You can experiment yourself with inserting other colliders and observe how the rays respond." \

    def create(self, stage):
        _open_test_usd_file(stage, "ogn_scene_query_raycast.usda")
