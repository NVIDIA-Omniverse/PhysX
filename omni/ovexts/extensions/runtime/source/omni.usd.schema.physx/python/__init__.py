# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import os

import omni.ext
import omni.kit.app
from pxr import Plug

from .scripts.schema_metadata_patches import (
    patch_physics_approximation_allowed_tokens,
    unpatch_physics_approximation_allowed_tokens,
)

pluginsRoot = os.path.join(os.path.dirname(__file__), '../../../plugins')

physxSchemaPath = pluginsRoot + '/PhysxSchema/resources'
Plug.Registry().RegisterPlugins(physxSchemaPath)

physxSchemaAdditionPath = pluginsRoot + "/PhysxSchemaAddition/resources"
Plug.Registry().RegisterPlugins(physxSchemaAdditionPath)

omniUsdPhysicsDeformablePath = pluginsRoot + "/OmniUsdPhysicsDeformableSchema/resources"
Plug.Registry().RegisterPlugins(omniUsdPhysicsDeformablePath)


class PhysxSchemaExtension(omni.ext.IExt):
    def on_startup(self, _ext_id):
        # Defer to the first update tick. Schema extensions auto-load very early,
        # so applying the patch directly in on_startup would force the USD
        # SchemaRegistry to materialize before other extensions (e.g. omni.rtx)
        # have run their `Plug.Registry().RegisterPlugins(...)` calls — late
        # registrations would then be invisible to FindAppliedAPIPrimDefinition.
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self._apply_patch_once,
                name="omni.usd.schema.physx.approximation_allowedTokens_patch",
            )
        )

    def _apply_patch_once(self, _event):
        self._update_sub = None
        patch_physics_approximation_allowed_tokens()

    def on_shutdown(self):
        self._update_sub = None
        unpatch_physics_approximation_allowed_tokens()
