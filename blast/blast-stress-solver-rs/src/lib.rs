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
//! - **Rapier integration** (feature `rapier`): `DestructibleSet` — full pipeline with Rapier3D physics
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

pub mod types;
pub mod stress_processor;
pub mod bond_stress;
pub mod ext_stress_solver;

#[cfg(feature = "scenarios")]
pub mod scenarios;

#[cfg(feature = "rapier")]
pub mod rapier;

// Re-export primary types at crate root for convenience
pub use types::*;
pub use stress_processor::{
    StressProcessor, StressNodeDesc, StressBondDesc, StressVelocity, StressImpulse,
    StressDataParams, StressSolverParams, StressErrorSq,
};
pub use bond_stress::compute_bond_stress;
pub use ext_stress_solver::ExtStressSolver;
