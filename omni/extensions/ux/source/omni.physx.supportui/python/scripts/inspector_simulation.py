# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import asyncio
import weakref
from omni.physxsupportui.bindings._physxSupportUi import PhysXInspectorModelState
from omni.physxsupportui import get_physx_supportui_private_interface
import omni.kit.undo
import omni.kit.notification_manager as nm
import omni.ui as ui

# For how long will the inspector simulation run during authoring
INSPECTOR_SIMULATION_DURATION_IN_SECONDS = 3
# Step size for running the inspector simulation
INSPECTOR_SIMULATION_STEP_SIZE = 1.0 / 60


class InspectorSimulation:
    def __init__(self):
        super().__init__()
        self._simulation_dirty = ui.SimpleBoolModel(False)
        self._simulation_time = 0
        self._num_steps = 0
        self.__ui_controls_to_disable = []
        self._undo_change_subs = omni.kit.undo.subscribe_on_change_detailed(
            self._on_kit_undo_change
        )
        self._sub_async_sim_run = None
        self._supportui_private = get_physx_supportui_private_interface()
        self._sub_model_event = self._supportui_private.get_inspector_event_stream().create_subscription_to_pop(
            self._on_inspector_event, name="Inspector Simulation"
        )
        self._inspector_state = PhysXInspectorModelState.DISABLED

    def _on_inspector_event(self, e):
        self._inspector_state = PhysXInspectorModelState(e.type)
        if self._inspector_state == PhysXInspectorModelState.RUNNING_SIMULATION:
            if self.simulation_dirty.as_bool:
                self.simulation_dirty.set_value(False)
                nm.post_notification(
                    f"Discarding uncommitted Physics Inspector authoring changes.\nPreserving changes to Target Position or other attributes.",
                    duration=5,
                )
        self._enable_ui_controls_depending_on_state()

    def _enable_ui_controls_depending_on_state(self):
        if (
            self._inspector_state == PhysXInspectorModelState.AUTHORING
            and self._sub_async_sim_run
        ):
            self._enable_ui_controls(False)
        else:
            self._enable_ui_controls(True)

    @property
    def simulation_dirty(self) -> ui.SimpleBoolModel:
        return self._simulation_dirty

    def clean(self):
        self.__ui_controls_to_disable = []
        self._supportui_private = None
        self._sub_async_sim_run = None
        self._sub_model_event = None
        omni.kit.undo.unsubscribe_on_change_detailed(self._on_kit_undo_change)
        self._undo_change_subs = None
        self.stop_authoring_simulation()

    def add_control_to_disable(self, control):
        self.__ui_controls_to_disable.append(weakref.ref(control))

    def _on_kit_undo_change(self, cmds):
        if self._inspector_state != PhysXInspectorModelState.AUTHORING:
            return
        try:
            cmd = cmds[0]
            if cmd.name == "ChangeProperty":
                path = cmd[1]["prop_path"]
                if (
                    path.name == "drive:angular:physics:targetPosition"
                    or path.name == "drive:linear:physics:targetPosition"
                    or path.name == "state:angular:physics:position"
                    or path.name == "state:linear:physics:position"
                ):
                    self.start_authoring_simulation(path.name)
        except:
            pass

    def _enable_ui_controls(self, enable):
        if enable:
            omni.physxui.get_physicsui_instance().mouse_interaction_override_toggle(
                omni.physxui.PhysxUIMouseInteraction.DEFAULT
            )
        else:
            omni.physxui.get_physicsui_instance().mouse_interaction_override_toggle(
                omni.physxui.PhysxUIMouseInteraction.DISABLED
            )

        to_remove = []
        for control_weak in self.__ui_controls_to_disable:
            control = control_weak()
            if control:
                control.enabled = enable
            else:
                to_remove.append(control_weak)
        for control_weak in to_remove:
            self.__ui_controls_to_disable.remove(control_weak)

    async def authoring_simulation_run(
        self, num_steps: int = None, simulation_time: float = None
    ):
        self._simulation_time = simulation_time
        self._num_steps = num_steps
        await asyncio.sleep(INSPECTOR_SIMULATION_STEP_SIZE)
        continue_running = True
        while self._supportui_private and continue_running:
            if (
                self._supportui_private.get_inspector_state()
                != PhysXInspectorModelState.AUTHORING
            ):
                self.stop_authoring_simulation()
                await asyncio.sleep(INSPECTOR_SIMULATION_STEP_SIZE)
            self._simulation_dirty.set_value(True)
            self._supportui_private.step_inspector_simulation(
                INSPECTOR_SIMULATION_STEP_SIZE
            )
            if num_steps is None:
                self._simulation_time = (
                    self._simulation_time - INSPECTOR_SIMULATION_STEP_SIZE
                )
                continue_running = self._simulation_time > 0
            else:
                self._num_steps = self._num_steps - 1
                continue_running = self._num_steps > 0

            if not continue_running:
                break
            await omni.kit.app.get_app().next_update_async()
        if self._supportui_private:
            self._supportui_private.refresh_all_inspector_models_values()
        self.stop_authoring_simulation()

    def start_authoring_simulation(self, attribute_name: str):
        self._supportui_private.enable_notice_handler(False)
        if (
            attribute_name == "state:angular:physics:position"
            or attribute_name == "state:linear:physics:position"
        ):
            if self._sub_async_sim_run is None:
                self._sub_async_sim_run = asyncio.ensure_future(
                    self.authoring_simulation_run(num_steps=1)
                )
            else:
                self._num_steps = 1
        else:
            if self._sub_async_sim_run is None:
                self._sub_async_sim_run = asyncio.ensure_future(
                    self.authoring_simulation_run(
                        simulation_time=INSPECTOR_SIMULATION_DURATION_IN_SECONDS
                    )
                )
                self._enable_ui_controls_depending_on_state()
            else:
                self._simulation_time = INSPECTOR_SIMULATION_DURATION_IN_SECONDS

    def stop_authoring_simulation(self):
        if self._sub_async_sim_run is not None:
            self._enable_ui_controls(True)
            self._supportui_private.enable_notice_handler(True)
            self._sub_async_sim_run.cancel()
            self._sub_async_sim_run = None
