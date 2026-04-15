//! Fixture consumer used by `tests/wasm_smoke_test.rs`.
//!
//! The goal is to exercise the full C++ code path that libc / libc++
//! can drag in on `wasm32-unknown-unknown`:
//!
//! - `ExtStressSolver::new` → `std::vector` allocations inside the
//!   C++ backend (malloc / free / realloc path).
//! - `add_gravity` + `update` → the core conjugate-gradient solver,
//!   which transitively touches every compiled C++ TU.
//! - `overstressed_bond_count` / `node_count` → FFI read-backs.
//!
//! If any of those paths references a libc symbol that isn't covered
//! by `src/wasm_runtime_shims.rs`, the linker resolves it against
//! `env.*` and the resulting wasm imports it — which is exactly what
//! `tests/wasm_smoke_test.rs::wasm_fixture_has_no_libc_imports`
//! forbids.
//!
//! We use `#[wasm_bindgen]` for the entry point (rather than a raw
//! `extern "C"` export) because Bug B — the `command_export` runtime
//! trap — only manifests when wasm-bindgen has at least one export
//! to wrap. Running the `wasm_fixture_instantiates_and_runs` test
//! against a bindgen-annotated export is what lets us catch any
//! future wasm-bindgen change that re-introduces the trap.
//!
//! On native targets this crate still compiles (as a cdylib with no
//! exported symbols) so that downstream workflows which recursively
//! invoke `cargo build` over every manifest don't choke on it.

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use blast_stress_solver::{BondDesc, ExtStressSolver, NodeDesc, SolverSettings, Vec3};

/// Build a trivial 2-node / 1-bond scenario, run one solver tick
/// under gravity, and return the overstressed bond count plus the
/// node count.
///
/// Returning a numeric summary (rather than just calling into the
/// solver for its side effects) prevents the optimizer from dead-
/// code-eliminating the entire call graph.
#[cfg_attr(target_arch = "wasm32", wasm_bindgen)]
pub fn blast_wasm_smoke() -> u32 {
    let nodes = [
        // Fixed support at the origin.
        NodeDesc {
            centroid: Vec3::new(0.0, 0.0, 0.0),
            mass: 0.0,
            volume: 1.0,
        },
        // Dynamic mass 1 m above.
        NodeDesc {
            centroid: Vec3::new(0.0, 1.0, 0.0),
            mass: 10.0,
            volume: 1.0,
        },
    ];
    let bonds = [BondDesc {
        centroid: Vec3::new(0.0, 0.5, 0.0),
        normal: Vec3::new(0.0, 1.0, 0.0),
        area: 1.0,
        node0: 0,
        node1: 1,
    }];
    let settings = SolverSettings::default();

    let Some(mut solver) = ExtStressSolver::new(&nodes, &bonds, &settings) else {
        return u32::MAX;
    };

    // Apply gravity + one solver tick. This is the hot path that
    // reaches into every compiled C++ translation unit.
    solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
    solver.update();

    // Combine two read-backs so nothing in the call graph is dead.
    solver
        .overstressed_bond_count()
        .wrapping_add(solver.node_count())
}
