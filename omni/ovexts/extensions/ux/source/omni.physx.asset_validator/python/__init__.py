# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from .bindings._physxAssetValidator import IPhysxAssetValidator
from .bindings._physxAssetValidator import acquire_physx_asset_validator_interface

from .ifacecache import cached_interface

def get_physx_asset_validator_interface() -> IPhysxAssetValidator:
    return cached_interface(get_physx_asset_validator_interface, acquire_physx_asset_validator_interface)

from .scripts.extension import *
