# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.ext
from .. import get_physx_cooking_service_private_interface
from ..bindings._physxCooking import release_physx_cooking_service_private, release_physx_cooking_service_private_scripting

class PhysxExtension(omni.ext.IExt):
    def on_startup(self):
        self._physx_cooking_private_interface = get_physx_cooking_service_private_interface()

    def on_shutdown(self):
        release_physx_cooking_service_private(self._physx_cooking_private_interface)
        release_physx_cooking_service_private_scripting(self._physx_cooking_private_interface) # OM-60917
        self._physx_cooking_private_interface = None
