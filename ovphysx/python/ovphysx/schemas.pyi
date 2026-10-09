# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

__all__: list[str]

NEWTON_SCHEMA_PACKAGE: str
NEWTON_SCHEMA_PIP_NAME: str
NEWTON_SCHEMA_URL: str
NEWTON_SCHEMA_INSTALL_HINT: str

def codeless_schema_root() -> Path: ...
def codeless_schema_paths() -> list[Path]: ...
def find_newton_schema_root() -> Path | None: ...
def newton_schema_root() -> Path: ...
