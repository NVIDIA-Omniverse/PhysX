# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from .. import get_physx_asset_validator_interface
from ..bindings._physxAssetValidator import (
    release_physx_asset_validator_interface,
    release_physx_asset_validator_interface_scripting,
)
from omni.asset_validator.core import ValidationRulesRegistry

from .jointStateChecker import JointStateChecker
from .backwardCompatibilityChecker import BackwardCompatibilityChecker
from .cookingApproximationChecker import CookingApproximationChecker
from .simulationOwnerChecker import SimulationOwnerChecker
from .massChecker import MassChecker
from .deformableSchemaChecker import DeformableSchemaChecker
from .apiConflictChecker import APIConflictChecker
from .jointPoseChecker import JointPoseChecker
from .articulationJointsChecker import ArticulationJointsChecker
from omni.physx.scripts.utils import safe_import_tests

safe_import_tests("omni.physxassetvalidator.tests")


class PhysxAssetValidatorExtension(omni.ext.IExt):
    def on_startup(self):
        self._iface = get_physx_asset_validator_interface()

        try:
            # Import it here to avoid tripping over doc generation with error "DLL load failed while importing _ui"
            import omni.asset_validator.ui
            omni.asset_validator.ui.get_instance().reset(resetRules=True)
        except:
            pass

    def on_shutdown(self):
        ValidationRulesRegistry.deregisterRule(JointStateChecker)
        ValidationRulesRegistry.deregisterRule(BackwardCompatibilityChecker)
        ValidationRulesRegistry.deregisterRule(CookingApproximationChecker)
        ValidationRulesRegistry.deregisterRule(SimulationOwnerChecker)
        ValidationRulesRegistry.deregisterRule(MassChecker)
        ValidationRulesRegistry.deregisterRule(DeformableSchemaChecker)
        ValidationRulesRegistry.deregisterRule(APIConflictChecker)
        ValidationRulesRegistry.deregisterRule(JointPoseChecker)
        ValidationRulesRegistry.deregisterRule(ArticulationJointsChecker)
        try:
            # import it here to avoid tripping over doc generation with error "DLL load failed while importing _ui"
            import omni.asset_validator.ui
            omni.asset_validator.ui.get_instance().reset(resetRules=True)
        except:
            pass

        release_physx_asset_validator_interface(self._iface)
        release_physx_asset_validator_interface_scripting(self._iface)  # OM-60917
