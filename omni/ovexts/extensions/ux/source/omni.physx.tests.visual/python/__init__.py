# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import omni.ext
from omni.physxtests.utils.loaders import import_tests_auto

import_tests_auto("omni.physxtestsvisual")

class Extension(omni.ext.IExt): 
    def on_startup(self):
        pass

    def on_shutdown(self):
        pass
