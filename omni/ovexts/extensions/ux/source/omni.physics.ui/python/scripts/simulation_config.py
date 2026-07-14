# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.ui as ui
import omni.kit.viewport.menubar.core as menubar
import omni.usd
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from omni.physics.core import Simulation, SimulationId, SimulationRegistryEventType, get_physics_interface, k_invalid_simulation_id
import carb.settings

SETTING_PHYSICS_SIMULATOR_TOGGLE_EXCLUSIVE = "/persistent/physics/ui/simulator_toggle_exclusive"

icons_folder = Path(__file__).parents[4].joinpath("icons")

UI_STYLE = {
    "Menu.Item.Icon::Simulation": {"image_url": str(icons_folder.joinpath("gear.svg"))}
    }

@dataclass
class Simulator:
    id : SimulationId
    name : str
    active : bool

class SimulationConfigViewportMenu(menubar.ViewportMenuContainer): 
    def __init__(self):
        self._simulators = []
        self._ui_menu = None

        settings = carb.settings.acquire_settings_interface()
        settings.set_default_bool(SETTING_PHYSICS_SIMULATOR_TOGGLE_EXCLUSIVE, True)
        self._setting_toggle_exclusive_model = menubar.SettingModelWithDefaultValue(SETTING_PHYSICS_SIMULATOR_TOGGLE_EXCLUSIVE, True)

        super().__init__("Simulation", 
                         delegate=menubar.IconMenuDelegate("Simulation", text=True),
                         style=UI_STYLE,
                         hide_on_click=False
                         )

    def build_fn(self, factory: dict):
        self._ui_menu = ui.Menu(self.name, delegate=self._delegate, on_build_fn=partial(self._build_menu, factory), style=self._style, hide_on_click=False)

    def _build_menu(self, args):
        ui.Separator("Simulators")

        def toggle_simulator(id, enable):
            physics = get_physics_interface()

            def deactivate_simulation(simulation_id):
                simulation : Simulation = physics.get_simulation(simulation_id)
                if simulation and physics.is_simulation_active(simulation_id):
                    if simulation.stage_update_fns.on_detach:
                        simulation.stage_update_fns.on_detach()
                    physics.deactivate_simulation(simulation_id)

            if enable:
                stage_id = omni.usd.get_context().get_stage_id()

                if self._setting_toggle_exclusive_model.get_value_as_bool():
                    for simulator in self._simulators:
                        if simulator.id != id:
                            deactivate_simulation(simulator.id)

                physics.activate_simulation(id)
                simulation : Simulation = physics.get_simulation(id)
                if simulation.stage_update_fns.on_attach:
                    simulation.stage_update_fns.on_attach(stage_id)
            else:
                deactivate_simulation(id)

        current = "None"
        for simulator in self._simulators:
            menubar.SelectableMenuItem(simulator.name, checkable=True, checked=simulator.active, hide_on_click=False, triggered_fn=partial(toggle_simulator, simulator.id, not simulator.active))
            if simulator.active:
                if current != "None":
                    current = "Multiple"
                else:
                    current = simulator.name

        self._ui_menu.text = current

        ui.Separator("Config")
        menubar.SelectableMenuItem("Toggle Exclusive", self._setting_toggle_exclusive_model, tooltip="When enabled, activating a simulator will deactivate all other simulators.")

    def invalidate(self):
        if self._ui_menu:
            self._ui_menu.invalidate()
        return super().invalidate()

    @property
    def simulators(self) -> list:
        return self._simulators

    @simulators.setter 
    def simulators(self, simulators):
        self._simulators = simulators
        self.invalidate()

class SimulationConfigManager:
    def __init__(self):
        self._viewport_menu = SimulationConfigViewportMenu()

        physics = get_physics_interface()
        self._simulation_registry_sub = physics.subscribe_simulation_registry_events(self._on_simulation_registry_event)
        self.refresh()

    def _on_simulation_registry_event(self, event_type: SimulationRegistryEventType, id: SimulationId, name: str):
        self.refresh()

    def refresh(self):
        physics = get_physics_interface()

        simulators = [Simulator(id=sim_id, name=physics.get_simulation_name(sim_id), active=physics.is_simulation_active(sim_id)) 
                    for sim_id in physics.get_simulation_ids() if sim_id != k_invalid_simulation_id]
        simulators.sort(key=lambda entry: entry.name)

        if len(simulators) != len(self._viewport_menu.simulators) or any(simulators != current for simulators, current in zip(simulators, self._viewport_menu.simulators)):
            self._viewport_menu.simulators = simulators

    def destroy(self):
        self._viewport_menu.destroy()
        self._viewport_menu = None
        self._simulation_registry_sub = None
