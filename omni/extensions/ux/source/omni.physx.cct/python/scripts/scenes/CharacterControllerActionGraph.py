# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.physxdemos as demo
from pathlib import Path


class CharacterControllerActionGraph(demo.Base):
    title = "Action Graph"
    category = demo.Categories.CCT
    short_description = "Base demo showing character controller setup through Action Graph"
    description = "Play (space) to run the simulation. Use Arrows to move the controller, Right Shift to jump, mouse move to look. All available Character Controller nodes are used: Character Controller for general control of the controller, Spawn Capsule to spawn a capsule saved in the Action Graph subtree and Controls Settings for key rebinding."
    demo_base_usd_url = str(Path(__file__).parent.parent.parent.parent.parent.joinpath("data/demos").joinpath("actiongraph.usd"))
    tags = ["Character Controller", "Action Graph", "Omni Graph"]
