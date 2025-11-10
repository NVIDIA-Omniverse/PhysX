# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import os

import numpy as np
import warp as wp
from  . import warp_utils as wp_utils

from .scenario import *


class GridBenchmarkBase(GridScenarioBase):
    def __init__(self, grid_params: GridParams, sim_params: SimParams, device_params: DeviceParams):
        super().__init__(grid_params, sim_params, device_params)
        self.minsteps = 600
        self.maxsteps = 600

    def start(self, frontend="warp", stage_id=-1):
        with wp.ScopedDevice(self.wp_device):
            # create simulation view
            self.sim = omni.physics.tensors.create_simulation_view(frontend, stage_id)

            # call implementation
            self.on_start(self.sim)

            # submit physics commands
            self.sim.flush()

    def physics_step(self, stepno, dt):
        with wp.ScopedDevice(self.wp_device):
            # call implementation
            self.on_physics_step(self.sim, stepno, dt)

            # submit physics commands
            self.sim.flush()

    @abstractmethod
    def on_start(self, sim):
        """Called when simulation starts"""

    @abstractmethod
    def on_physics_step(self, sim, stepno, dt):
        """Called after every physics simulation step"""


class BenchCartpole(GridBenchmarkBase):
    def __init__(self, device_params, num_envs=5000, verbose=False):
        # set up stage
        grid_params = GridParams(num_envs)
        #grid_params.num_rows = grid_params.num_envs // 2
        grid_params.row_spacing = 2.1
        grid_params.col_spacing = 2.1
        sim_params = SimParams()
        sim_params.gravity_dir = Gf.Vec3f(0.0, 0.0, 0.0)
        sim_params.gravity_mag = 0.0
        super().__init__(grid_params, sim_params, device_params)

        # set up env template
        asset_path = os.path.join(get_asset_root(), "CartPoleNoRail.usda")
        actor_path = self.env_template_path.AppendChild("cartpole")
        transform = Transform((0.0, 0.0, 1.1))
        self.create_actor_from_asset(actor_path, transform, asset_path)

        self.verbose = verbose

    def on_start(self, sim):
        if self.verbose:
            print("!!!!!!!!!!!!!!! START")

        cartpoles = sim.create_articulation_view("/envs/*/cartpole")

        self.cartpoles = cartpoles
        self.all_indices = wp_utils.arange(cartpoles.count, device=sim.device)

        # allocate force buffer
        num_dofs = self.cartpoles.max_dofs
        self.forces = wp.zeros((self.cartpoles.count, self.cartpoles.max_dofs), dtype=float, device=sim.device)

        self.reset(sim)

    def reset(self, sim):
        if self.verbose:
            print("!!!!!!!!!!!!!!! RESET")

        # set initial positions
        pmin = -0.95 * math.pi
        pmax = 0.95 * math.pi
        dof_pos = wp_utils.linspace(self.cartpoles.count, pmin, pmax, include_end=True, device=sim.device)
        #print(dof_pos.numpy().squeeze())
        self.cartpoles.set_dof_positions(dof_pos, self.all_indices)

    def on_physics_step(self, sim, stepno, dt):
        if stepno > 0 and stepno % 60 == 0:
            self.reset(sim)
        else:
            dof_pos = self.cartpoles.get_dof_positions()
            dof_vel = self.cartpoles.get_dof_velocities()

            if self.verbose:
                p = dof_pos.numpy().squeeze()
                print(stepno, p[0])

            # compute and apply forces
            stiffness = 20.0
            damping = 4.0
            wp_utils.compute_dof_forces(dof_pos, dof_vel, self.forces, stiffness, damping, device=sim.device)
            self.cartpoles.set_dof_actuation_forces(self.forces, self.all_indices)
