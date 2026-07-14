# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Pytest conftest: initializes Carbonite and verifies USD Python bindings.
#
# _carb_setup starts the Carbonite framework and loads core plugins.
# The pxr module is made importable by a .pth file installed into the venv
# by build.sh.

import _carb_setup  # noqa: F401 - initializes Carbonite framework and loads plugins

try:
    import pxr  # noqa: F401
except ImportError as exc:
    raise ImportError(
        "USD Python bindings (pxr) not found. "
        "Run build.sh to set up the Python environment."
    ) from exc
