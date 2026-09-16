# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-1

from __future__ import annotations

from pathlib import Path
from typing import Any

from .api import (
    ContactBinding as ContactBinding,
    PhysX as PhysX,
    TensorBindingSpec as TensorBindingSpec,
    disable_python_logging as disable_python_logging,
    enable_default_log_output as enable_default_log_output,
    enable_python_logging as enable_python_logging,
    flush_log as flush_log,
    get_log_level as get_log_level,
    set_log_level as set_log_level,
)
from .config import OmniPvdDestination as OmniPvdDestination
from .config import PhysXConfig as PhysXConfig
from .contact_types import (
    ContactEventHeader as ContactEventHeader,
    ContactPoint as ContactPoint,
    FrictionAnchor as FrictionAnchor,
)
from .dlpack import (
    DLPACK_VERSION as DLPACK_VERSION,
    DLDataType as DLDataType,
    DLDataTypeCode as DLDataTypeCode,
    DLDevice as DLDevice,
    DLDeviceType as DLDeviceType,
    DLManagedTensor as DLManagedTensor,
    DLTensor as DLTensor,
)
from .schemas import codeless_schema_paths as codeless_schema_paths
from .schemas import codeless_schema_root as codeless_schema_root
from .types import (
    ApiStatus as ApiStatus,
    BindingPrimMode as BindingPrimMode,
    ConfigBool as ConfigBool,
    ConfigFloat as ConfigFloat,
    ConfigInt32 as ConfigInt32,
    ConfigString as ConfigString,
    LogLevel as LogLevel,
    SceneQueryGeometryType as SceneQueryGeometryType,
    SceneQueryMode as SceneQueryMode,
    TensorType as TensorType,
)

__version__: str
OP_INDEX_ALL: int

def bootstrap() -> None: ...
def ai_skills_path() -> dict[str, Path]: ...
def __getattr__(name: str) -> Any: ...

__all__: list[str]
