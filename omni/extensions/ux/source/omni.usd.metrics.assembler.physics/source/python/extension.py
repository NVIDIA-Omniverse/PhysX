# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext

from .metricsAssemblerPhysics import MetricsAssemblerPhysics


class OmniUsdMetricsAssemblerPhysics(omni.ext.IExt):
    def __init__(self):
        super().__init__()
        self._metricsAssemblerPhysics = None

    def on_startup(self, ext_id):
        self._metricsAssemblerPhysics = MetricsAssemblerPhysics()

    def on_shutdown(self):
        self._metricsAssemblerPhysics.on_shutdown()
        self._metricsAssemblerPhysics = None
