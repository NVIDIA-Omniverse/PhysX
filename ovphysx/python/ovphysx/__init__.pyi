# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5

# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-1

from __future__ import annotations

from pathlib import Path
from typing import Any

from .api import (
    Operation as Operation,
    Query as Query,
    QueryMetadata as QueryMetadata,
    ReadResult as ReadResult,
    ReadGroup as ReadGroup,
    ReadMetadata as ReadMetadata,
    ReadAttributeResult as ReadAttributeResult,
    ReadOmission as ReadOmission,
    WriteSession as WriteSession,
    WriteGroup as WriteGroup,
    WriteMetadata as WriteMetadata,
    WriteAttributeResult as WriteAttributeResult,
    ContactBinding as ContactBinding,
    PhysX as PhysX,
    Task as Task,
    TaskError as TaskError,
    TaskTimeoutError as TaskTimeoutError,
    TensorBindingSpec as TensorBindingSpec,
    disable_python_logging as disable_python_logging,
    enable_default_log_output as enable_default_log_output,
    enable_python_logging as enable_python_logging,
    flush_log as flush_log,
    get_global_config as get_global_config,
    get_log_level as get_log_level,
    set_global_config as set_global_config,
    set_log_level as set_log_level,
)
from .config import OmniPvdDestination as OmniPvdDestination
from .config import PhysXConfig as PhysXConfig
from .dlpack import (
    DLPACK_VERSION as DLPACK_VERSION,
    DLDataType as DLDataType,
    DLDataTypeCode as DLDataTypeCode,
    DLDevice as DLDevice,
    DLDeviceType as DLDeviceType,
    DLManagedTensor as DLManagedTensor,
    DLTensor as DLTensor,
)
from .schemas import newton_schema_root as newton_schema_root
from .types import (
    ApiStatus as ApiStatus,
    BindingPrimMode as BindingPrimMode,
    ConfigKey as ConfigKey,
    ConfigBool as ConfigBool,
    ConfigFloat as ConfigFloat,
    ConfigInt32 as ConfigInt32,
    ConfigString as ConfigString,
    GlobalConfigKey as GlobalConfigKey,
    LogLevel as LogLevel,
    ProfilerBackend as ProfilerBackend,
    SceneQueryGeometryType as SceneQueryGeometryType,
    SceneQueryMode as SceneQueryMode,
    TensorType as TensorType,
)

__version__: str

def bootstrap() -> None: ...
def ai_skills_path() -> dict[str, Path]: ...
def __getattr__(name: str) -> Any: ...

__all__: list[str]
