# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

# Skip all tests on ETM
import os
if not os.getenv("ETM_ACTIVE"):
    from .tests import *
    from .testsPhysxImmediate import *
    from .testsPhysxTriggers import *
    from .testsPhysxVisual import *
