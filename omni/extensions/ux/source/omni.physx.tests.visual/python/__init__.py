# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from omni.physxtests.utils.loaders import import_tests_auto

import_tests_auto("omni.physxtestsvisual")

class Extension(omni.ext.IExt): 
    def on_startup(self):
        pass

    def on_shutdown(self):
        pass
