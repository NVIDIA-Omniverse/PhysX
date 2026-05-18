# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import Tf

# PreparePythonModule didn't make it's way into USD
# until 21.08 - older versions import the module
# manually and call PrepareModule
if hasattr(Tf, "PreparePythonModule"):
    Tf.PreparePythonModule()
else:
    from . import _physxSchema
    Tf.PrepareModule(_physxSchema, locals())

del Tf