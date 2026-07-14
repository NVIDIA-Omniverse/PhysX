# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

import _carb_setup  # noqa: F401 - Carbonite framework init (must be first)

# Verify USD Python bindings are reachable
from pxr import Usd  # noqa: F401
