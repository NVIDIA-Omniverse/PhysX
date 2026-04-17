//! High-level stress solver with NvBlast actor management and fracture support.

use crate::ffi;
use crate::types::*;

/// High-level stress solver that manages NvBlast families, actors, and fracture commands.
///
/// This is the primary API for most users. It wraps the C++ `ExtStressSolver` which internally
/// manages an NvBlast asset, family, and actors alongside the stress computation.
pub struct ExtStressSolver {
    handle: *mut ffi::ExtStressSolverHandle,
}

unsafe impl Send for ExtStressSolver {}
unsafe impl Sync for ExtStressSolver {}

impl ExtStressSolver {
    /// Create a solver from node and bond descriptors with the given settings.
    pub fn new(nodes: &[NodeDesc], bonds: &[BondDesc], settings: &SolverSettings) -> Option<Self> {
        if nodes.is_empty() || bonds.is_empty() {
            return None;
        }

        let ffi_nodes: Vec<ffi::FfiExtStressNodeDesc> = nodes
            .iter()
            .map(|n| ffi::FfiExtStressNodeDesc {
                centroid: n.centroid,
                mass: n.mass,
                volume: n.volume,
            })
            .collect();

        let ffi_bonds: Vec<ffi::FfiExtStressBondDesc> = bonds
            .iter()
            .map(|b| ffi::FfiExtStressBondDesc {
                centroid: b.centroid,
                normal: b.normal,
                area: b.area,
                node0: b.node0,
                node1: b.node1,
            })
            .collect();

        let ffi_settings = to_ffi_settings(settings);

        let handle = unsafe {
            ffi::ext_stress_solver_create(
                ffi_nodes.as_ptr(),
                ffi_nodes.len() as u32,
                ffi_bonds.as_ptr(),
                ffi_bonds.len() as u32,
                &ffi_settings,
            )
        };

        if handle.is_null() {
            None
        } else {
            Some(Self { handle })
        }
    }

    /// Update solver settings.
    pub fn set_settings(&mut self, settings: &SolverSettings) {
        let ffi_settings = to_ffi_settings(settings);
        unsafe { ffi::ext_stress_solver_set_settings(self.handle, &ffi_settings) }
    }

    /// Reset accumulated forces.
    pub fn reset(&mut self) {
        unsafe { ffi::ext_stress_solver_reset(self.handle) }
    }

    /// Apply a force to a specific node.
    pub fn add_force(&mut self, node_index: u32, position: Vec3, force: Vec3, mode: ForceMode) {
        unsafe {
            ffi::ext_stress_solver_add_force(
                self.handle,
                node_index,
                &position,
                &force,
                mode as u32,
            )
        }
    }

    /// Apply gravity to all actors.
    pub fn add_gravity(&mut self, gravity: Vec3) {
        unsafe { ffi::ext_stress_solver_add_gravity(self.handle, &gravity) }
    }

    /// Apply gravity to a specific actor.
    pub fn add_actor_gravity(&mut self, actor_index: u32, gravity: Vec3) -> bool {
        unsafe { ffi::ext_stress_solver_add_actor_gravity(self.handle, actor_index, &gravity) != 0 }
    }

    /// Run one solver update (computes stresses from accumulated forces).
    pub fn update(&mut self) {
        unsafe { ffi::ext_stress_solver_update(self.handle) }
    }

    /// Number of bonds that exceeded their fatal stress limit after the last `update()`.
    pub fn overstressed_bond_count(&self) -> u32 {
        unsafe { ffi::ext_stress_solver_overstressed_bond_count(self.handle) }
    }

    /// Whether the solver converged in the last `update()`.
    pub fn converged(&self) -> bool {
        unsafe { ffi::ext_stress_solver_converged(self.handle) != 0 }
    }

    /// Linear error residual from the last solve.
    pub fn linear_error(&self) -> f32 {
        unsafe { ffi::ext_stress_solver_get_linear_error(self.handle) }
    }

    /// Angular error residual from the last solve.
    pub fn angular_error(&self) -> f32 {
        unsafe { ffi::ext_stress_solver_get_angular_error(self.handle) }
    }

    /// Number of actors currently in the family.
    pub fn actor_count(&self) -> u32 {
        unsafe { ffi::ext_stress_solver_actor_count(self.handle) }
    }

    /// Total number of graph nodes.
    pub fn node_count(&self) -> u32 {
        unsafe { ffi::ext_stress_solver_graph_node_count(self.handle) }
    }

    /// Total number of bonds.
    pub fn bond_count(&self) -> u32 {
        unsafe { ffi::ext_stress_solver_bond_count(self.handle) }
    }

    /// Collect the current actor table.
    pub fn actors(&self) -> Vec<Actor> {
        let actor_count = self.actor_count();
        if actor_count == 0 {
            return Vec::new();
        }
        let node_count = self.node_count();

        let mut actor_buffer = vec![
            ffi::FfiExtStressActor {
                actor_index: u32::MAX,
                nodes: std::ptr::null(),
                node_count: 0,
            };
            actor_count as usize
        ];
        let mut nodes_buffer = vec![0u32; node_count as usize];
        let mut out_actor_count = 0u32;
        let mut out_node_count = 0u32;

        unsafe {
            ffi::ext_stress_solver_collect_actors(
                self.handle,
                actor_buffer.as_mut_ptr(),
                actor_count,
                nodes_buffer.as_mut_ptr(),
                node_count,
                &mut out_actor_count,
                &mut out_node_count,
            );
        }

        let mut result = Vec::with_capacity(out_actor_count as usize);
        for i in 0..out_actor_count as usize {
            let ffi_actor = &actor_buffer[i];
            let nodes = if !ffi_actor.nodes.is_null() && ffi_actor.node_count > 0 {
                let offset = unsafe { ffi_actor.nodes.offset_from(nodes_buffer.as_ptr()) } as usize;
                nodes_buffer[offset..offset + ffi_actor.node_count as usize].to_vec()
            } else {
                Vec::new()
            };
            result.push(Actor {
                actor_index: ffi_actor.actor_index,
                nodes,
            });
        }
        result
    }

    /// Generate fracture commands for all actors with overstressed bonds.
    pub fn generate_fracture_commands(&self) -> Vec<FractureCommand> {
        let actor_count = self.actor_count();
        let bond_count = self.bond_count();
        if actor_count == 0 || bond_count == 0 {
            return Vec::new();
        }

        let mut command_buffer = vec![
            ffi::FfiExtStressFractureCommands {
                actor_index: u32::MAX,
                bond_fractures: std::ptr::null_mut(),
                bond_fracture_count: 0,
            };
            actor_count as usize
        ];
        let mut bond_buffer = vec![ffi::FfiExtStressBondFracture::default(); bond_count as usize];
        let mut out_command_count = 0u32;
        let mut out_bond_count = 0u32;

        unsafe {
            ffi::ext_stress_solver_generate_fracture_commands_per_actor(
                self.handle,
                command_buffer.as_mut_ptr(),
                actor_count,
                bond_buffer.as_mut_ptr(),
                bond_count,
                &mut out_command_count,
                &mut out_bond_count,
            );
        }

        let mut result = Vec::with_capacity(out_command_count as usize);
        for i in 0..out_command_count as usize {
            let cmd = &command_buffer[i];
            let fractures = if !cmd.bond_fractures.is_null() && cmd.bond_fracture_count > 0 {
                let offset =
                    unsafe { cmd.bond_fractures.offset_from(bond_buffer.as_ptr()) } as usize;
                bond_buffer[offset..offset + cmd.bond_fracture_count as usize]
                    .iter()
                    .map(|f| BondFracture {
                        userdata: f.userdata,
                        node_index0: f.node_index0,
                        node_index1: f.node_index1,
                        health: f.health,
                    })
                    .collect()
            } else {
                Vec::new()
            };
            result.push(FractureCommand {
                actor_index: cmd.actor_index,
                bond_fractures: fractures,
            });
        }
        result
    }

    /// Apply fracture commands and return split events.
    pub fn apply_fracture_commands(&mut self, commands: &[FractureCommand]) -> Vec<SplitEvent> {
        if commands.is_empty() {
            return Vec::new();
        }

        // Flatten bond fractures into a contiguous buffer
        let total_bonds: usize = commands.iter().map(|c| c.bond_fractures.len()).sum();
        let mut flat_bonds = vec![ffi::FfiExtStressBondFracture::default(); total_bonds];
        let mut ffi_commands = Vec::with_capacity(commands.len());
        let mut offset = 0usize;

        for cmd in commands {
            for (i, f) in cmd.bond_fractures.iter().enumerate() {
                flat_bonds[offset + i] = ffi::FfiExtStressBondFracture {
                    userdata: f.userdata,
                    node_index0: f.node_index0,
                    node_index1: f.node_index1,
                    health: f.health,
                };
            }
            ffi_commands.push(ffi::FfiExtStressFractureCommands {
                actor_index: cmd.actor_index,
                bond_fractures: if cmd.bond_fractures.is_empty() {
                    std::ptr::null_mut()
                } else {
                    unsafe { flat_bonds.as_mut_ptr().add(offset) }
                },
                bond_fracture_count: cmd.bond_fractures.len() as u32,
            });
            offset += cmd.bond_fractures.len();
        }

        // Allocate output buffers
        let node_count = self.node_count() as usize;
        let max_events = commands.len();
        let max_children = node_count;

        let mut events_buffer = vec![
            ffi::FfiExtStressSplitEvent {
                parent_actor_index: u32::MAX,
                children: std::ptr::null_mut(),
                child_count: 0,
            };
            max_events
        ];
        let mut child_buffer = vec![
            ffi::FfiExtStressActor {
                actor_index: u32::MAX,
                nodes: std::ptr::null(),
                node_count: 0,
            };
            max_children
        ];
        let mut nodes_buffer = vec![0u32; node_count];
        let mut out_event_count = 0u32;
        let mut out_child_count = 0u32;
        let mut out_node_count = 0u32;

        unsafe {
            ffi::ext_stress_solver_apply_fracture_commands(
                self.handle,
                ffi_commands.as_ptr(),
                ffi_commands.len() as u32,
                events_buffer.as_mut_ptr(),
                max_events as u32,
                child_buffer.as_mut_ptr(),
                max_children as u32,
                &mut out_event_count,
                &mut out_child_count,
                nodes_buffer.as_mut_ptr(),
                node_count as u32,
                &mut out_node_count,
            );
        }

        // Parse results
        let mut result = Vec::with_capacity(out_event_count as usize);
        for i in 0..out_event_count as usize {
            let evt = &events_buffer[i];
            let mut children = Vec::with_capacity(evt.child_count as usize);

            if !evt.children.is_null() && evt.child_count > 0 {
                let child_offset =
                    unsafe { evt.children.offset_from(child_buffer.as_ptr()) } as usize;
                for ci in 0..evt.child_count as usize {
                    let child = &child_buffer[child_offset + ci];
                    let nodes = if !child.nodes.is_null() && child.node_count > 0 {
                        let node_off =
                            unsafe { child.nodes.offset_from(nodes_buffer.as_ptr()) } as usize;
                        nodes_buffer[node_off..node_off + child.node_count as usize].to_vec()
                    } else {
                        Vec::new()
                    };
                    children.push(SplitChild {
                        actor_index: child.actor_index,
                        nodes,
                    });
                }
            }

            result.push(SplitEvent {
                parent_actor_index: evt.parent_actor_index,
                children,
            });
        }
        result
    }

    /// Get excess forces for an actor that separated from the structure.
    /// Returns `(force, torque)` if the actor exists.
    pub fn get_excess_forces(&self, actor_index: u32, com: Vec3) -> Option<(Vec3, Vec3)> {
        let mut force = Vec3::ZERO;
        let mut torque = Vec3::ZERO;
        let ok = unsafe {
            ffi::ext_stress_solver_get_excess_forces(
                self.handle,
                actor_index,
                &com,
                &mut force,
                &mut torque,
            )
        };
        if ok != 0 {
            Some((force, torque))
        } else {
            None
        }
    }
}

impl Drop for ExtStressSolver {
    fn drop(&mut self) {
        unsafe { ffi::ext_stress_solver_destroy(self.handle) }
    }
}

fn to_ffi_settings(s: &SolverSettings) -> ffi::FfiExtStressSolverSettingsDesc {
    ffi::FfiExtStressSolverSettingsDesc {
        max_solver_iterations_per_frame: s.max_solver_iterations_per_frame,
        graph_reduction_level: s.graph_reduction_level,
        compression_elastic_limit: s.compression_elastic_limit,
        compression_fatal_limit: s.compression_fatal_limit,
        tension_elastic_limit: s.tension_elastic_limit,
        tension_fatal_limit: s.tension_fatal_limit,
        shear_elastic_limit: s.shear_elastic_limit,
        shear_fatal_limit: s.shear_fatal_limit,
    }
}
