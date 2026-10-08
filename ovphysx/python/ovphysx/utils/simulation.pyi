# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-FRAME-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5
# @implements REQ-PYTHON-UTILS-001
# @covers AC-2

from collections.abc import Mapping, Sequence

from ..api import PhysX
from ..types import SimObjectType


class OvStageOutputCache:
    def __init__(self, physx: PhysX) -> None: ...
    def refresh(self) -> None: ...
    def close(self) -> None: ...
    def __enter__(self) -> OvStageOutputCache: ...
    def __exit__(self, *exc: object) -> None: ...


def step_and_write_to_ovstage(
    physx: PhysX,
    *,
    dt: float,
    output_ordinal: int,
    cache: OvStageOutputCache | None = ...,
    outputs: Mapping[SimObjectType, Sequence[str]] | None = ...,
) -> int: ...

__all__: list[str]
