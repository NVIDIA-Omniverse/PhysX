# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ext
from .. import get_physx_stage_update_node_interface
from omni.physxstageupdate.bindings._physxStageUpdateNode import release_physx_stage_update_node_interface, release_physx_stage_update_node_scripting
from .live_sync import StageUpdateLiveSync
import carb

class OmniPhysXStageUpdateExtension(omni.ext.IExt):
    def on_startup(self):        
        self._physx_stage_update_node_interface = get_physx_stage_update_node_interface()
        self._live_sync = StageUpdateLiveSync()
        self._live_sync.startup(self._physx_stage_update_node_interface)

        try:
            from omni.physics.core import get_physics_interface
            physics_interface = get_physics_interface()
            
            # Check if PhysX simulation is registered and active
            simulation_ids = physics_interface.get_simulation_ids()
            
            for sim_id in simulation_ids:
                sim_name = physics_interface.get_simulation_name(sim_id)
                if sim_name == "PhysX":
                    if physics_interface.is_simulation_active(sim_id):                        
                        carb.log_warn("PhysX simulation is active through omni.physx umbrella extension. Either disable omni.physx.stageupdate or omni.physics.physx extension, omni.physics.physx was deactivated.")
                        physics_interface.deactivate_simulation(sim_id)
                    break

        except Exception as e:            
            pass


    def on_shutdown(self):
        self._live_sync.shutdown()
        self._live_sync = None
        release_physx_stage_update_node_interface(self._physx_stage_update_node_interface)
        release_physx_stage_update_node_scripting(self._physx_stage_update_node_interface) # OM-60917
        self._physx_stage_update_node_interface = None
