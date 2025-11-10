# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import os
import omni.ext
from omni.physxtests import import_tests_auto


import_tests_auto("omni.physicstensorstests.scripts", ["tests"])


class PhysicsTensorsTestsExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        pass
        
    def on_shutdown(self):
        pass
