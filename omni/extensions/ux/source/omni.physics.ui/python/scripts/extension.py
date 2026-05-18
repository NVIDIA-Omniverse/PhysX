# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.ext
from .simulation_config import SimulationConfigManager
from .simulationDataVisualizer import SimulationDataVisualizerWindowManager

class PhysicsUIExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()
    
    def on_startup(self, _ext_id):
        self._simulation_config_manager = SimulationConfigManager()
        self._simulation_data_visualizer_manager = SimulationDataVisualizerWindowManager()

    def on_shutdown(self):
        if self._simulation_config_manager:
            self._simulation_config_manager.destroy()
            self._simulation_config_manager = None
        if self._simulation_data_visualizer_manager:
            self._simulation_data_visualizer_manager.destroy()
            self._simulation_data_visualizer_manager = None
