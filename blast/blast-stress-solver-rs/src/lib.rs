//! # Blast Stress Solver
//!
//! Rust library wrapping the NVIDIA Blast stress solver for destructible structures.
//!
//! ## Features
//!
//! - **Core solver**: `ExtStressSolver` — manages nodes, bonds, actors, fracture detection and splitting
//! - **Low-level solver**: `StressProcessor` — direct conjugate-gradient solver access
//! - **Bond stress analysis**: `compute_bond_stress` — decompose impulses into compression/tension/shear
//! - **Scenarios** (feature `scenarios`): Pre-built wall, tower, and bridge scenario builders
//! - **Rapier integration** (feature `rapier`): `DestructionRuntime` for existing Rapier apps and
//!   `DestructibleSet` as the low-level escape hatch
//!
//! `DestructionRuntime` is the recommended integration point when you already
//! own a Rapier world and want normal Rapier contacts to drive fracture while
//! still keeping `PhysicsPipeline::step(...)` in your app.
//!
//! ## Quick Start (without Rapier)
//!
//! ```no_run
//! use blast_stress_solver::*;
//!
//! let nodes = vec![
//!     NodeDesc { centroid: Vec3::new(0.0, 0.0, 0.0), mass: 0.0, volume: 1.0 },  // support
//!     NodeDesc { centroid: Vec3::new(0.0, 1.0, 0.0), mass: 10.0, volume: 1.0 },  // dynamic
//! ];
//! let bonds = vec![
//!     BondDesc {
//!         centroid: Vec3::new(0.0, 0.5, 0.0),
//!         normal: Vec3::new(0.0, 1.0, 0.0),
//!         area: 1.0,
//!         node0: 0,
//!         node1: 1,
//!     },
//! ];
//! let settings = SolverSettings::default();
//! let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();
//!
//! solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
//! solver.update();
//!
//! let overstressed = solver.overstressed_bond_count();
//! ```

mod ffi;

// On `wasm32-unknown-unknown` the Blast C++ backend references dozens
// of libc symbols (malloc, fwrite, abort, …) through libc++'s STL
// helpers.  We provide pure-Rust stubs for all of them in
// `wasm_runtime_shims`, so the final wasm module imports neither
// `env.*` libc functions nor `wasi_snapshot_preview1.*` wasi calls
// — it is a pure library module.
#[cfg(target_arch = "wasm32")]
mod wasm_runtime_shims;

// `-fno-exceptions` is not enough to strip every mention of
// `__cxa_allocate_exception` / `__cxa_throw` from libc++ — STL
// containers still emit them behind `throw_bad_alloc`-style helpers.
// Provide trapping stubs so the wasm module stays self-contained.
#[cfg(target_arch = "wasm32")]
mod wasm_cxa_stubs;

pub mod bond_stress;
pub mod ext_stress_solver;
pub mod stress_processor;
pub mod types;

#[cfg(feature = "scenarios")]
pub mod scenarios;

#[cfg(feature = "rapier")]
pub mod rapier;

// Re-export primary types at crate root for convenience
pub use bond_stress::compute_bond_stress;
pub use ext_stress_solver::ExtStressSolver;
pub use stress_processor::{
    StressBondDesc, StressDataParams, StressErrorSq, StressImpulse, StressNodeDesc,
    StressProcessor, StressSolverParams, StressVelocity,
};
pub use types::*;
