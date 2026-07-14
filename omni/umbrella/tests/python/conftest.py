# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Pytest conftest: initializes Carbonite before any test modules are imported.

import sys
from pathlib import Path

# Ensure this directory is on sys.path so _carb_setup can be found
# when pytest is invoked with an explicit --rootdir (e.g. by VS Code).
_this_dir = str(Path(__file__).resolve().parent)
if _this_dir not in sys.path:
    sys.path.insert(0, _this_dir)

import _carb_setup  # noqa: F401 - initializes Carbonite framework and loads plugins
