# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import os
import sys

py38 = (3,8)
current_version = sys.version_info
if os.name == 'nt' and current_version >= py38:
    from pathlib import Path
    os.add_dll_directory(Path.joinpath(Path(os.path.dirname(__file__)), Path('..\\..\\')).__str__())

from . import _physicsSchemaTools
from pxr import Tf
Tf.PrepareModule(_physicsSchemaTools, locals())
del Tf

try:
    import __DOC
    __DOC.Execute(locals())
    del __DOC
except Exception:
    try:
        import __tmpDoc
        __tmpDoc.Execute(locals())
        del __tmpDoc
    except:
        pass
