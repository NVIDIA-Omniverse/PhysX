# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.ext
from .simulation_config import SimulationConfigManager

class PhysicsUIExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()
    
    def on_startup(self, _ext_id):
        self._simulation_config_manager = SimulationConfigManager()

    def on_shutdown(self):
        self._simulation_config_manager.destroy()
        self._simulation_config_manager = None
